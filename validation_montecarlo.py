# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# Robustness / model-mismatch validation harness for the thermal MPC core.
"""
Helios UAV Avionics — Monte-Carlo Robustness Harness
=====================================================
Closes the "perfect-model fallacy": the standard flight_loop_sim validates the
MPC by letting it predict with a shadow copy of the EXACT plant it controls, so
success is guaranteed by construction. That proves the control *logic*, not
robustness.

This harness breaks the symmetry that matters for a real product:

    ┌─────────────────────────┐        noisy measured T_bat
    │  MPC predictor          │◄───────────────────────────────┐
    │  (NOMINAL parameters)   │                                 │
    └───────────┬─────────────┘                                 │
                │ vent command                                  │
                ▼                                                │
    ┌─────────────────────────┐   true (hidden) T_bat           │
    │  TRUE plant             │─────────────────────────────────┘
    │  (PERTURBED parameters, │
    │   +ambient disturbance) │
    └─────────────────────────┘

Per Monte-Carlo trial we perturb the plant the MPC does NOT know about:
  * battery internal resistance R_internal   (±10% — the user's example)
  * insulation conductance      k_insulation (±10%)
  * vent effectiveness          h_air        (±10%)
  * thermal capacity            C_thermal    (±10%)
  * initial battery temperature (Gaussian offset)
and corrupt what the MPC can see / faces:
  * sensor noise on the measured battery temperature (Gaussian)
  * an unmodelled ambient-temperature disturbance the weather oracle missed

We then ask the only question that matters commercially: across many randomised
aircraft, does Helios keep the TRUE battery inside [T_MIN, T_MAX]?

Run:
    PYTHONUTF8=1 python validation_montecarlo.py
"""

import random
import statistics
from dataclasses import dataclass, field

from thermal_simulator import ThermalSimulator
from weather_oracle    import WeatherOracle
from mpc_core          import (
    ModelPredictiveController,
    T_MIN_SAFE, T_MAX_SAFE,
)
from flight_loop_sim   import generate_mission_profile, MPC_HORIZON


# ── Nominal ("datasheet") plant parameters the MPC believes ─────────────
NOMINAL = dict(
    C_thermal    = 1500.0,
    k_insulation =    0.15,
    R_internal   =    0.03,
    h_air        =    0.5,
)
START_TEMP_NOMINAL: float = 22.0   # °C — take-off battery temp the MPC assumes


@dataclass
class MismatchConfig:
    """Everything the MPC is NOT told about the real world."""
    n_trials: int = 500
    seed:     int = 20260702

    # Fractional tolerance (± uniform) applied to each true plant parameter.
    param_tol: dict = field(default_factory=lambda: {
        "R_internal":   0.10,   # battery resistance varies by ±10%
        "k_insulation": 0.10,
        "h_air":        0.10,
        "C_thermal":    0.10,
    })

    init_temp_sigma:  float = 1.5   # °C — spread of true take-off temperature
    sensor_sigma:     float = 0.5   # °C — battery temp sensor noise (1σ)
    ambient_sigma:    float = 2.0   # °C — unmodelled weather bias on T_ext (1σ)


@dataclass
class TrialResult:
    min_t_bat:     float
    max_t_bat:     float
    cold_breaches: int
    hot_breaches:  int

    @property
    def safe(self) -> bool:
        return self.cold_breaches == 0 and self.hot_breaches == 0


def _sample_true_params(rng: random.Random, cfg: MismatchConfig) -> dict:
    """Draw one perturbed plant realisation the MPC has no knowledge of."""
    return {
        name: NOMINAL[name] * (1.0 + rng.uniform(-tol, tol))
        for name, tol in cfg.param_tol.items()
    }


def run_trial(rng: random.Random, cfg: MismatchConfig, mission) -> TrialResult:
    """
    Simulate one randomised aircraft over the full mission.

    Key invariant: the MPC plans with NOMINAL parameters and a NOISY measured
    temperature; the TRUE plant evolves with perturbed parameters and an
    ambient disturbance. The two never share state.
    """
    oracle = WeatherOracle(T_ground_base=18.0, V_wind_base=5.0)
    oracle.imposta_ondata_di_gelo(attiva=True)   # same cold-wave stress scenario

    # MPC predictor — parameterised ONLY with the nominal model.
    nominal_sim = ThermalSimulator(T_internal=START_TEMP_NOMINAL, **NOMINAL)
    mpc = ModelPredictiveController(oracle, nominal_sim)

    # TRUE plant — perturbed parameters + perturbed initial temperature.
    true_params = _sample_true_params(rng, cfg)
    true_init   = START_TEMP_NOMINAL + rng.gauss(0.0, cfg.init_temp_sigma)
    true_sim    = ThermalSimulator(T_internal=true_init, **true_params)

    # Per-trial unmodelled ambient bias (weather the oracle failed to predict).
    ambient_bias = rng.gauss(0.0, cfg.ambient_sigma)

    min_t = max_t = true_sim.T_internal
    cold = hot = 0

    for i, fm in enumerate(mission):
        # Rolling MPC look-ahead (next MPC_HORIZON waypoints), padded at the end.
        look_ahead = [m.altitude_m for m in mission[i: i + MPC_HORIZON]]
        while len(look_ahead) < MPC_HORIZON:
            look_ahead.append(look_ahead[-1])

        # The MPC only ever sees a NOISY measurement of the true temperature.
        measured_temp = true_sim.T_internal + rng.gauss(0.0, cfg.sensor_sigma)

        decision = mpc.predict_thermal_trajectory(
            current_temp           = measured_temp,
            flight_plan            = look_ahead,
            current_current_motor  = fm.current_motor_a,
            current_current_solar  = fm.current_solar_a,
        )

        # Advance the TRUE plant: real atmosphere + unmodelled bias, perturbed
        # physics, and the vent command the MPC actually issued.
        atm = oracle.get_state_at_altitude(fm.altitude_m)
        true_sim.update(
            temp_esterna     = atm.temp_ext_c + ambient_bias,
            corrente_motore  = fm.current_motor_a,
            corrente_solare  = fm.current_solar_a,
            v_pitot          = atm.wind_speed_ms,
            posizione_botola = decision.vent_command,
        )

        t = true_sim.T_internal
        min_t = min(min_t, t)
        max_t = max(max_t, t)
        if t < T_MIN_SAFE:
            cold += 1
        if t > T_MAX_SAFE:
            hot += 1

    return TrialResult(min_t, max_t, cold, hot)


def run_campaign(cfg: MismatchConfig) -> None:
    rng     = random.Random(cfg.seed)
    mission = generate_mission_profile()

    DIV = "─" * 74
    print()
    print("  Helios UAV Avionics — Monte-Carlo Robustness Harness")
    print("  Model-Mismatch Validation · Cold-Wave Stress Scenario")
    print(DIV)
    print(f"  Trials              : {cfg.n_trials}")
    print(f"  Parameter tolerance : " +
          ", ".join(f"{k} ±{int(v*100)}%" for k, v in cfg.param_tol.items()))
    print(f"  Sensor noise (1σ)   : {cfg.sensor_sigma:.2f} °C")
    print(f"  Ambient bias (1σ)   : {cfg.ambient_sigma:.2f} °C")
    print(f"  Init-temp spread(1σ): {cfg.init_temp_sigma:.2f} °C")
    print(f"  Safe band           : [{T_MIN_SAFE:.0f}, {T_MAX_SAFE:.0f}] °C")
    print(DIV)

    results = [run_trial(rng, cfg, mission) for _ in range(cfg.n_trials)]

    safe        = sum(r.safe for r in results)
    cold_fail   = sum(r.cold_breaches > 0 for r in results)
    hot_fail    = sum(r.hot_breaches  > 0 for r in results)
    min_temps   = sorted(r.min_t_bat for r in results)
    worst_min   = min_temps[0]
    p05_min     = min_temps[max(0, int(0.05 * len(min_temps)) - 1)]
    median_min  = statistics.median(min_temps)

    breach_rate = 100.0 * (cfg.n_trials - safe) / cfg.n_trials

    print(f"  Safe trials (0 breaches) : {safe} / {cfg.n_trials}  "
          f"({100.0*safe/cfg.n_trials:.1f}%)")
    print(f"  Trials with COLD breach  : {cold_fail}")
    print(f"  Trials with HOT  breach  : {hot_fail}")
    print(DIV)
    print(f"  Min T_bat — worst trial  : {worst_min:+.2f} °C  "
          f"(margin vs T_MIN: {worst_min - T_MIN_SAFE:+.2f} K)")
    print(f"  Min T_bat —  5th pctile  : {p05_min:+.2f} °C  "
          f"(margin vs T_MIN: {p05_min - T_MIN_SAFE:+.2f} K)")
    print(f"  Min T_bat —  median      : {median_min:+.2f} °C")
    print(DIV)

    if breach_rate == 0.0:
        verdict = "PASS — 0% breach rate under model mismatch (robust)"
    else:
        verdict = (f"FAIL — {breach_rate:.1f}% of trials breach the safe band "
                   f"under expected tolerances")
    print(f"  VERDICT: {verdict}")
    print()


if __name__ == "__main__":
    run_campaign(MismatchConfig())
