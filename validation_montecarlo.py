# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# Model-mismatch / robustness validation harness for the thermal MPC core.
"""
Helios UAV Avionics — Robust-MPC Margin Sweep ("Sim Flight Test")
================================================================================
Closes the "perfect-model fallacy": flight_loop_sim lets the MPC predict with a
shadow copy of the EXACT plant it controls, so success is guaranteed by
construction. This harness instead makes the MPC plan with NOMINAL parameters
and a NOISY measured temperature while the TRUE plant runs on PERTURBED physics:

    ┌─────────────────────────┐        NOISY measured T_bat
    │  MPC predictor          │◄───────────────────────────────┐
    │  (NOMINAL params +      │                                 │
    │   robustness margin)    │                                 │
    └───────────┬─────────────┘                                 │
                │ vent command                                  │
                ▼                                                │
    ┌─────────────────────────┐   TRUE (hidden) T_bat           │
    │  TRUE plant             │─────────────────────────────────┘
    │  (PERTURBED params,     │
    │   +ambient disturbance) │
    └─────────────────────────┘

Injected uncertainty (all invisible to the MPC):
  * R_internal / k_insulation / h_air / C_thermal  ±15% (uniform)
  * Gaussian battery-temp SENSOR noise (drift/jitter)
  * Gaussian initial-temperature spread
  * Unmodelled ambient-temperature disturbance the weather oracle missed

Outcome classification per trial (breach measured against the TRUE certified
limits, never the tightened band the controller plans to):
  SAFE    — no breach.
  WARNED  — breach, but the MPC raised BEST_EFFORT_INFEASIBLE at/before it
            (honest degradation — the aircraft can escalate).
  SILENT  — breach with NO prior warning. The disqualifying case.

Margin sweep: re-runs the identical 500-aircraft population for each candidate
robustness back-off (t_min_margin), so the ONLY variable is the buffer. The
smallest margin that drives SILENT to 0.0% is the "Certified Safety Buffer".

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
    VENT_CRUISE,
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

# ── Margin sweep grid (cold-side robustness back-off, Kelvin) ───────────
MARGIN_SWEEP = [round(0.5 * i, 1) for i in range(9)]   # 0.0 … 4.0 K, step 0.5

# ── Certified-buffer confirmation run ───────────────────────────────────
DEFAULT_MARGIN: float = 1.5      # candidate Certified Safety Buffer to confirm
CONFIRM_TRIALS: int   = 10_000   # large sample → tight statistical confidence


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
    realized_min_t:  float
    cold_breaches:   int
    hot_breaches:    int
    breach_step:     int | None
    infeasible_step: int | None
    n_cruise:        int          # decisions issued at efficient CRUISE vent
    n_steps:         int

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
    def cruise_fraction(self) -> float:
        return self.n_cruise / self.n_steps if self.n_steps else 0.0


@dataclass
class CampaignStats:
    margin:       float
    n:            int
    safe:         int
    warned:       int
    silent:       int
    worst_margin: float
    p05_margin:   float
    cruise_pct:   float

    @property
    def silent_pct(self) -> float:
        return 100.0 * self.silent / self.n


def _sample_true_params(rng: random.Random, cfg: MismatchConfig) -> dict:
    """Draw one perturbed plant realisation the MPC has no knowledge of."""
    return {
        name: NOMINAL[name] * (1.0 + rng.uniform(-tol, tol))
        for name, tol in cfg.param_tol.items()
    }


def run_trial(
    rng: random.Random,
    cfg: MismatchConfig,
    mission,
    t_min_margin: float,
) -> TrialResult:
    """
    Simulate one randomised aircraft over the full mission at a given margin.

    The MPC plans with NOMINAL parameters, a NOISY measured temperature, and the
    supplied robustness back-off; the TRUE plant evolves with perturbed
    parameters and an ambient disturbance. Breach is judged against the TRUE
    certified limits.
    """
    oracle = WeatherOracle(T_ground_base=18.0, V_wind_base=5.0)
    oracle.imposta_ondata_di_gelo(attiva=True)   # cold-wave stress scenario

    # MPC predictor — nominal model + robustness back-off (constraint tightening).
    nominal_sim = ThermalSimulator(T_internal=START_TEMP_NOMINAL, **NOMINAL)
    mpc = ModelPredictiveController(oracle, nominal_sim, t_min_margin=t_min_margin)

    # TRUE plant — perturbed parameters + perturbed initial temperature.
    true_params = _sample_true_params(rng, cfg)
    true_init   = START_TEMP_NOMINAL + rng.gauss(0.0, cfg.init_temp_sigma)
    true_sim    = ThermalSimulator(T_internal=true_init, **true_params)

    ambient_bias = rng.gauss(0.0, cfg.ambient_sigma)

    realized_min = true_sim.T_internal
    cold = hot = n_cruise = 0
    breach_step = infeasible_step = None

    for i, fm in enumerate(mission):
        look_ahead = [m.altitude_m for m in mission[i: i + MPC_HORIZON]]
        while len(look_ahead) < MPC_HORIZON:
            look_ahead.append(look_ahead[-1])

        measured_temp = true_sim.T_internal + rng.gauss(0.0, cfg.sensor_sigma)

        decision = mpc.predict_thermal_trajectory(
            current_temp           = measured_temp,
            flight_plan            = look_ahead,
            current_current_motor  = fm.current_motor_a,
            current_current_solar  = fm.current_solar_a,
        )

        if decision.vent_command == VENT_CRUISE:
            n_cruise += 1
        if (decision.trigger_code == TriggerReason.BEST_EFFORT_INFEASIBLE
                and infeasible_step is None):
            infeasible_step = i

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
        if t < T_MIN_SAFE:                       # TRUE certified floor
            cold += 1
            if breach_step is None:
                breach_step = i
        if t > T_MAX_SAFE:
            hot += 1
            if breach_step is None:
                breach_step = i

    return TrialResult(
        realized_min_t  = realized_min,
        cold_breaches   = cold,
        hot_breaches    = hot,
        breach_step     = breach_step,
        infeasible_step = infeasible_step,
        n_cruise        = n_cruise,
        n_steps         = len(mission),
    )


def _pctile(sorted_vals: list[float], q: float) -> float:
    """Lower-bound percentile on a pre-sorted list."""
    idx = max(0, int(q * len(sorted_vals)) - 1)
    return sorted_vals[idx]


def run_campaign(cfg: MismatchConfig, mission, t_min_margin: float) -> CampaignStats:
    """Run the full n_trials campaign at one margin, on the seeded population."""
    rng = random.Random(cfg.seed)   # SAME population for every margin (fair sweep)
    results = [run_trial(rng, cfg, mission, t_min_margin) for _ in range(cfg.n_trials)]
    n = len(results)

    safe   = sum(r.outcome == "SAFE"   for r in results)
    warned = sum(r.outcome == "WARNED" for r in results)
    silent = sum(r.outcome == "SILENT" for r in results)

    margins    = sorted(r.realized_margin for r in results)
    cruise_pct = 100.0 * statistics.mean(r.cruise_fraction for r in results)

    return CampaignStats(
        margin       = t_min_margin,
        n            = n,
        safe         = safe,
        warned       = warned,
        silent       = silent,
        worst_margin = margins[0],
        p05_margin   = _pctile(margins, 0.05),
        cruise_pct   = cruise_pct,
    )


def run_sweep(cfg: MismatchConfig, margins: list[float]) -> None:
    mission = generate_mission_profile()

    DIV = "═" * 82
    SUB = "─" * 82
    print()
    print("  Helios UAV Avionics — Robust-MPC Margin Sweep (Sim Flight Test)")
    print("  Cold-Wave Stress · MPC plans NOMINAL+margin, TRUE plant runs PERTURBED")
    print(DIV)
    print(f"  Trials/margin : {cfg.n_trials}   (seed {cfg.seed}, identical population per margin)")
    print(f"  Param spread  : " +
          ", ".join(f"{k} ±{int(v*100)}%" for k, v in cfg.param_tol.items()))
    print(f"  Sensor 1σ {cfg.sensor_sigma:.2f}°C | Ambient 1σ {cfg.ambient_sigma:.2f}°C | "
          f"Init 1σ {cfg.init_temp_sigma:.2f}°C | Band [{T_MIN_SAFE:.0f},{T_MAX_SAFE:.0f}]°C")
    print(DIV)
    print(f"  {'Margin':>6} │ {'SAFE%':>6} {'WARN%':>6} {'SILENT%':>7} │ "
          f"{'Worst':>7} {'5thPct':>7} │ {'Cruise%':>7}")
    print(f"  {'(K)':>6} │ {'':>6} {'':>6} {'':>7} │ "
          f"{'Rlz(K)':>7} {'Rlz(K)':>7} │ {'(eff.)':>7}")
    print(SUB)

    all_stats: list[CampaignStats] = []
    for m in margins:
        s = run_campaign(cfg, mission, m)
        all_stats.append(s)
        flag = "  ← 0 SILENT" if s.silent == 0 else ""
        print(f"  {s.margin:>6.1f} │ {100.0*s.safe/s.n:>6.1f} "
              f"{100.0*s.warned/s.n:>6.1f} {s.silent_pct:>7.1f} │ "
              f"{s.worst_margin:>+7.2f} {s.p05_margin:>+7.2f} │ "
              f"{s.cruise_pct:>7.1f}{flag}")

    print(DIV)

    # ── Certified Safety Buffer: smallest margin with 0 SILENT breaches ──
    certified = next((s for s in all_stats if s.silent == 0), None)
    if certified is None:
        print("  RESULT: no margin in the swept range eliminated SILENT breaches.")
        print("          Widen the sweep or revisit sensor/model assumptions.")
    else:
        zero_all = certified.warned == 0
        print(f"  CERTIFIED SAFETY BUFFER : t_min_margin = {certified.margin:.1f} K")
        print(f"    → SILENT breaches      : 0.0%  (was "
              f"{all_stats[0].silent_pct:.1f}% at 0.0 K)")
        print(f"    → Residual WARNED      : {100.0*certified.warned/certified.n:.1f}%  "
              f"({'none — 0 breaches total' if zero_all else 'breaches flagged in advance'})")
        print(f"    → Worst realized margin: {certified.worst_margin:+.2f} K vs T_MIN")
        print(f"    → Efficiency retained  : {certified.cruise_pct:.1f}% cruise "
              f"(vs {all_stats[0].cruise_pct:.1f}% at 0.0 K)")
    print()


def _upper95_one_sided_zero(n: int) -> float:
    """
    Exact one-sided 95% upper confidence bound on an event rate when ZERO events
    are observed in n trials (Clopper-Pearson): p_hi = 1 − 0.05**(1/n).
    Approximates the "rule of three" (≈ 3/n) but is exact.
    """
    return 1.0 - 0.05 ** (1.0 / n)


def run_confirmation(
    cfg: MismatchConfig,
    mission,
    margin: float,
    n_trials: int,
) -> None:
    """
    Large-sample confirmation at a single margin, on a population INDEPENDENT of
    the sweep (seed offset), so the buffer is validated on data it was not tuned
    against. Reports outcome rates and exact 95% confidence bounds.
    """
    rng = random.Random(cfg.seed + 1)   # independent of the tuning sweep
    results = [run_trial(rng, cfg, mission, margin) for _ in range(n_trials)]
    n = len(results)

    safe     = sum(r.outcome == "SAFE"   for r in results)
    warned   = sum(r.outcome == "WARNED" for r in results)
    silent   = sum(r.outcome == "SILENT" for r in results)
    breaches = warned + silent
    margins  = sorted(r.realized_margin for r in results)

    DIV = "═" * 82
    print()
    print(f"  CONFIRMATION — t_min_margin = {margin:.1f} K · {n:,} INDEPENDENT trials "
          f"(seed {cfg.seed + 1})")
    print(DIV)
    print(f"    SAFE    : {safe:>6,d}  ({100.0*safe/n:7.3f}%)")
    print(f"    WARNED  : {warned:>6,d}  ({100.0*warned/n:7.3f}%)")
    print(f"    SILENT  : {silent:>6,d}  ({100.0*silent/n:7.3f}%)")
    print(f"    Worst realized margin : {margins[0]:+.2f} K vs T_MIN")
    print(f"    5th-pctile margin     : {_pctile(margins, 0.05):+.2f} K")
    print("  " + "─" * 80)
    if silent == 0:
        print(f"    SILENT rate : 0 / {n:,} observed → < {100.0*_upper95_one_sided_zero(n):.3f}%"
              f"  (95% one-sided CI)")
    else:
        print(f"    SILENT rate : {100.0*silent/n:.3f}% observed")
    if breaches == 0:
        print(f"    ANY breach  : 0 / {n:,} observed → < {100.0*_upper95_one_sided_zero(n):.3f}%"
              f"  (95% one-sided CI)")
    else:
        print(f"    ANY breach  : {100.0*breaches/n:.3f}% observed")
    print(DIV)
    print()


if __name__ == "__main__":
    _cfg     = MismatchConfig()
    _mission = generate_mission_profile()
    run_sweep(_cfg, MARGIN_SWEEP)
    run_confirmation(_cfg, _mission, DEFAULT_MARGIN, CONFIRM_TRIALS)
