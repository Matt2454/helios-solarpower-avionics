# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# Model-mismatch / robustness validation harness for the thermal MPC core.
"""
Helios UAV Avionics — Monte-Carlo Model-Mismatch Validation ("Sim Flight Test")
================================================================================
Closes the "perfect-model fallacy": flight_loop_sim lets the MPC predict with a
shadow copy of the EXACT plant it controls, so success is guaranteed by
construction. That proves control *logic*, not robustness.

This harness breaks the symmetry that matters for a real product:

    ┌─────────────────────────┐        NOISY measured T_bat
    │  MPC predictor          │◄───────────────────────────────┐
    │  (NOMINAL parameters)   │                                 │
    └───────────┬─────────────┘                                 │
                │ vent command                                  │
                ▼                                                │
    ┌─────────────────────────┐   TRUE (hidden) T_bat           │
    │  TRUE plant             │─────────────────────────────────┘
    │  (PERTURBED parameters, │
    │   +ambient disturbance) │
    └─────────────────────────┘

Injected uncertainty (all invisible to the MPC):
  * R_internal / k_insulation / h_air / C_thermal  ±15% (uniform)
  * Gaussian battery-temp SENSOR noise (drift/jitter)
  * Gaussian initial-temperature spread
  * Unmodelled ambient-temperature disturbance the weather oracle missed

The pass question is NOT merely "zero breaches". A safety-critical controller is
acceptable if it EITHER holds the band OR flags that it cannot. So every trial
is classified into one of three outcomes:

  SAFE    — no breach.
  WARNED  — a breach occurred, but the MPC raised BEST_EFFORT_INFEASIBLE at or
            before the breach: it KNEW it could not hold and said so. Honest
            degradation — the aircraft can escalate (shed load, abort).
  SILENT  — a breach occurred with NO prior infeasible signal. The controller
            believed it was safe and was wrong. THIS is the disqualifying case.

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
    TriggerReason,
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
        "R_internal":   0.15,   # battery resistance varies by ±15%
        "k_insulation": 0.15,
        "h_air":        0.15,
        "C_thermal":    0.15,
    })

    init_temp_sigma:  float = 1.5   # °C — spread of true take-off temperature
    sensor_sigma:     float = 0.5   # °C — battery temp sensor noise (1σ)
    ambient_sigma:    float = 2.0   # °C — unmodelled weather bias on T_ext (1σ)


@dataclass
class TrialResult:
    realized_min_t:  float          # coldest TRUE battery temp reached
    realized_max_t:  float          # hottest TRUE battery temp reached
    predicted_min_t: float          # MPC's most-pessimistic in-horizon belief
    cold_breaches:   int
    hot_breaches:    int
    breach_step:     int | None     # first minute a breach occurred (0-based)
    infeasible_step: int | None     # first minute MPC raised INFEASIBLE

    # ── Derived classification ─────────────────────────────────────────
    @property
    def breached(self) -> bool:
        return self.cold_breaches > 0 or self.hot_breaches > 0

    @property
    def outcome(self) -> str:
        if not self.breached:
            return "SAFE"
        warned = (
            self.infeasible_step is not None
            and self.breach_step is not None
            and self.infeasible_step <= self.breach_step
        )
        return "WARNED" if warned else "SILENT"

    @property
    def realized_margin(self) -> float:
        return self.realized_min_t - T_MIN_SAFE

    @property
    def predicted_margin(self) -> float:
        return self.predicted_min_t - T_MIN_SAFE


def _sample_true_params(rng: random.Random, cfg: MismatchConfig) -> dict:
    """Draw one perturbed plant realisation the MPC has no knowledge of."""
    return {
        name: NOMINAL[name] * (1.0 + rng.uniform(-tol, tol))
        for name, tol in cfg.param_tol.items()
    }


def run_trial(rng: random.Random, cfg: MismatchConfig, mission) -> TrialResult:
    """
    Simulate one randomised aircraft over the full mission.

    Invariant: the MPC plans with NOMINAL parameters and a NOISY measured
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

    realized_min = realized_max = true_sim.T_internal
    predicted_min = float("inf")
    cold = hot = 0
    breach_step = infeasible_step = None

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

        # Predictive margin: the MPC's most-pessimistic belief along the horizon
        # it CHOSE. This is what Helios "thinks" the margin is.
        predicted_min = min(predicted_min, min(decision.horizon_temps))

        # Record the first moment Helios admits it cannot hold the band.
        if (decision.trigger_code == TriggerReason.BEST_EFFORT_INFEASIBLE
                and infeasible_step is None):
            infeasible_step = i

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
        realized_min = min(realized_min, t)
        realized_max = max(realized_max, t)
        if t < T_MIN_SAFE:
            cold += 1
            if breach_step is None:
                breach_step = i
        if t > T_MAX_SAFE:
            hot += 1
            if breach_step is None:
                breach_step = i

    return TrialResult(
        realized_min_t  = realized_min,
        realized_max_t  = realized_max,
        predicted_min_t = predicted_min,
        cold_breaches   = cold,
        hot_breaches    = hot,
        breach_step     = breach_step,
        infeasible_step = infeasible_step,
    )


def _pctile(sorted_vals: list[float], q: float) -> float:
    """Simple lower-bound percentile on a pre-sorted list."""
    idx = max(0, int(q * len(sorted_vals)) - 1)
    return sorted_vals[idx]


def run_campaign(cfg: MismatchConfig) -> None:
    rng     = random.Random(cfg.seed)
    mission = generate_mission_profile()

    DIV = "─" * 76
    print()
    print("  Helios UAV Avionics — Model-Mismatch Validation (Sim Flight Test)")
    print("  Cold-Wave Stress Scenario · MPC predicts NOMINAL, plant runs PERTURBED")
    print(DIV)
    print(f"  Trials              : {cfg.n_trials}   (seed {cfg.seed})")
    print(f"  Parameter tolerance : " +
          ", ".join(f"{k} ±{int(v*100)}%" for k, v in cfg.param_tol.items()))
    print(f"  Sensor noise (1σ)   : {cfg.sensor_sigma:.2f} °C")
    print(f"  Ambient bias (1σ)   : {cfg.ambient_sigma:.2f} °C")
    print(f"  Init-temp spread(1σ): {cfg.init_temp_sigma:.2f} °C")
    print(f"  Safe band           : [{T_MIN_SAFE:.0f}, {T_MAX_SAFE:.0f}] °C")
    print(DIV)

    results = [run_trial(rng, cfg, mission) for _ in range(cfg.n_trials)]
    n = len(results)

    safe   = [r for r in results if r.outcome == "SAFE"]
    warned = [r for r in results if r.outcome == "WARNED"]
    silent = [r for r in results if r.outcome == "SILENT"]

    # ── Outcome triage ─────────────────────────────────────────────────
    print("  OUTCOME TRIAGE")
    print(f"    SAFE   (no breach)                 : {len(safe):4d}  "
          f"({100.0*len(safe)/n:5.1f}%)")
    print(f"    WARNED (breach, flagged infeasible): {len(warned):4d}  "
          f"({100.0*len(warned)/n:5.1f}%)")
    print(f"    SILENT (breach, NO warning)        : {len(silent):4d}  "
          f"({100.0*len(silent)/n:5.1f}%)   <-- disqualifying")
    print(DIV)

    # ── Predictive margin: belief vs reality ───────────────────────────
    realized = sorted(r.realized_margin for r in results)
    predicted = sorted(r.predicted_margin for r in results)
    gaps      = [r.predicted_margin - r.realized_margin for r in results]

    print("  PREDICTIVE MARGIN vs T_MIN  (K above the 10 °C floor)")
    print(f"    Realized  — worst   : {realized[0]:+.2f} K")
    print(f"    Realized  —  5th pct: {_pctile(realized, 0.05):+.2f} K")
    print(f"    Realized  — median  : {statistics.median(realized):+.2f} K")
    print(f"    Predicted — median  : {statistics.median(predicted):+.2f} K "
          f"(what Helios believed)")
    print(f"    Optimism gap median : {statistics.median(gaps):+.2f} K "
          f"(belief − reality; >0 = over-confident)")
    print(DIV)

    # ── Infeasibility situational awareness ────────────────────────────
    ever_infeasible = sum(r.infeasible_step is not None for r in results)
    print("  SELF-AWARENESS")
    print(f"    Trials that ever raised INFEASIBLE : {ever_infeasible} / {n}")
    if silent:
        worst = min(silent, key=lambda r: r.realized_min_t)
        print(f"    Worst SILENT breach                : "
              f"{worst.realized_min_t:+.2f} °C true "
              f"(believed {worst.predicted_min_t:+.2f} °C)")
    print(DIV)

    # ── Verdict ────────────────────────────────────────────────────────
    if not warned and not silent:
        verdict = "PASS — 0 breaches across all trials (robust to ±tolerances)"
    elif not silent:
        verdict = ("CONDITIONAL — breaches occurred but ALL were flagged "
                   "infeasible in advance (honest degradation, not silent)")
    else:
        verdict = (f"FAIL — {len(silent)} SILENT breach(es) "
                   f"({100.0*len(silent)/n:.1f}%): Helios was confident and wrong")
    print(f"  VERDICT: {verdict}")
    print()


if __name__ == "__main__":
    run_campaign(MismatchConfig())
