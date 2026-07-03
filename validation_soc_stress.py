# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# SoC / fault-injection stress harness for the layered MPC + Safety Monitor.
"""
Helios UAV Avionics — SoC Stress & Fault-Injection Harness
================================================================================
Validation logic (why these stress parameters)
----------------------------------------------
The certified Monte-Carlo campaign (validation_montecarlo.py) proved the MPC
under PARAMETRIC mismatch at full charge. This harness attacks the three things
that campaign could not see:

 1. ENERGY STATES — an 8-hour HALE afternoon-to-dusk mission with a coulomb SoC
    model driven by the solar geometry model. The battery drains into the
    energy-aware region exactly when the sun stops helping (dusk), and a
    high-drain event is injected at critical SoC: the worst-case corner by
    construction, not by luck.
 2. FAULTS — injected sensor STALE bursts, a stuck (STALLED) vent actuator, a
    calibrated sensor drift, and a numerically hostile parameter draw. Each
    scenario has a PREDICTED Safety-Monitor response; the harness measures the
    actual one, in ticks.
 3. THE MONITOR ITSELF — a line-faithful Python mirror of core/safety_monitor.h
    runs in the loop between MPC and actuator. (Mirror, not the C++ artifact:
    back-to-back C++ equivalence is a separate golden-vector task.)

Latency semantics — stated honestly
-----------------------------------
This harness measures LOGICAL intervention latency: control ticks from fault
manifestation to monitor override. Wall-clock latency is a property of the
target loop (100 Hz ⇒ decision-to-actuation ≤ one 10 ms cycle by construction)
and cannot be demonstrated in a 60 s/tick simulation. The designed sensor
debounce (3 samples) is a deliberate filter delay, not compute latency.

Black box
---------
One BlackBoxRecord per control tick in a bounded ring (last 64 ticks), frozen
at the first breach for post-mortem reconstruction. On target this is a
fixed-size ring buffer at control rate in reserved RAM.

Run:
    PYTHONUTF8=1 python validation_soc_stress.py
"""

import math
import random
import statistics
from collections import deque
from dataclasses import dataclass, field

import mpc_core
from mpc_core import (
    ModelPredictiveController, TriggerReason, VENT_CRUISE,
    T_MIN_SAFE, T_MAX_SAFE,
)
from thermal_simulator      import ThermalSimulator
from weather_oracle         import WeatherOracle
from solar_model            import SolarModel
from validation_montecarlo  import (
    NOMINAL, MismatchConfig, _sample_true_params,
    _upper95_one_sided_zero, _pctile,
)

# ══════════════════════════════════════════════════════════════════════
# Safety-Monitor reference mirror (of core/safety_monitor.h)
# ══════════════════════════════════════════════════════════════════════

V_PASS, V_CLAMPED, V_OVR_REACTIVE, V_OVR_FAILSAFE = "PASS", "CLAMPED", "OVR_REACTIVE", "OVR_FAILSAFE"

F_SENSOR_INVALID, F_SENSOR_PERSIST = 1 << 0, 1 << 1
F_MPC_STALE                        = 1 << 2
F_ENV_COLD, F_ENV_HOT              = 1 << 3, 1 << 4
F_CMD_RANGE, F_CMD_SLEW            = 1 << 5, 1 << 6


class SafetyMonitorRef:
    """Python mirror of core/safety_monitor.h (same precedence, same defaults).
    NOTE: slew is per CALL — this harness calls once per 60 s tick, so the
    effective slew rate here is 0.25/min vs 0.25/10 ms on target. See findings.
    """

    def __init__(self, t_min_hard=10.0, t_max_hard=45.0,
                 vent_failsafe=0.2, slew_per_step=0.25, sensor_debounce=3,
                 failsafe_cold_below=20.0, failsafe_hot_above=40.0):
        self.t_min_hard, self.t_max_hard = t_min_hard, t_max_hard
        self.vent_failsafe       = vent_failsafe
        self.slew_per_step       = slew_per_step
        self.sensor_debounce     = sensor_debounce
        self.failsafe_cold_below = failsafe_cold_below
        self.failsafe_hot_above  = failsafe_hot_above
        self._last_cmd   = -1.0
        self._last_good  = 0.0
        self._have_temp  = False
        self._bad_streak = 0

    def _failsafe_posture(self) -> float:
        """Temperature-CONDITIONED failsafe. Forensic finding (S2, first run):
        a STATIC 0.2 posture during a sensor blackout actively cooled a
        cold-marginal pack the monitor could no longer see and CAUSED the very
        breach it exists to prevent. Condition the blind posture on the last
        trustworthy temperature instead: cold half → retain heat (0.0),
        hot region → dump heat (1.0), otherwise cruise."""
        if not self._have_temp:
            return self.vent_failsafe
        if self._last_good < self.failsafe_cold_below:
            return 0.0
        if self._last_good > self.failsafe_hot_above:
            return 1.0
        return self.vent_failsafe

    def _slew(self, target: float) -> float:
        if self._last_cmd < 0.0:
            self._last_cmd = target
            return target
        lo, hi = self._last_cmd - self.slew_per_step, self._last_cmd + self.slew_per_step
        v = min(max(target, lo), hi)
        self._last_cmd = v
        return v

    def step(self, t_meas: float, sample_ok: bool, adv_vent: float):
        faults = 0
        if sample_ok:
            self._bad_streak = 0
            self._last_good  = t_meas
            self._have_temp  = True
        else:
            faults |= F_SENSOR_INVALID
            self._bad_streak = min(255, self._bad_streak + 1)

        trustworthy = sample_ok or (self._have_temp and self._bad_streak < self.sensor_debounce)
        if not trustworthy:
            faults |= F_SENSOR_PERSIST
            return self._slew(self._failsafe_posture()), V_OVR_FAILSAFE, faults

        t = t_meas if sample_ok else self._last_good
        if t <= self.t_min_hard:
            return self._slew(0.0), V_OVR_REACTIVE, faults | F_ENV_COLD
        if t >= self.t_max_hard:
            return self._slew(1.0), V_OVR_REACTIVE, faults | F_ENV_HOT

        verdict, cmd = V_PASS, adv_vent
        if cmd < 0.0 or cmd > 1.0:
            faults |= F_CMD_RANGE
            cmd, verdict = min(max(cmd, 0.0), 1.0), V_CLAMPED
        shaped = self._slew(cmd)
        if shaped != cmd:
            faults |= F_CMD_SLEW
        return shaped, verdict, faults


# ══════════════════════════════════════════════════════════════════════
# Black box
# ══════════════════════════════════════════════════════════════════════

@dataclass
class BlackBoxRecord:
    tick:        int
    t_true:      float   # hidden plant truth (sim-only; on target: not logged)
    t_meas:      float
    sensor_ok:   bool
    soc:         float
    urgency:     float
    mpc_vent:    float
    trigger:     str
    feasible:    bool
    verdict:     str
    faults:      int
    cmd_out:     float   # what the monitor forwarded
    vent_actual: float   # what the actuator actually did (≠ cmd if STALLED)


# ══════════════════════════════════════════════════════════════════════
# Fault injection & mission
# ══════════════════════════════════════════════════════════════════════

@dataclass
class FaultConfig:
    stale_windows:  list = field(default_factory=list)   # [(start, len_ticks)]
    stall_window:   tuple | None = None                  # (start, len, stuck_pos)
    drift_bias_c:   float = 0.0                          # calibrated sensor bias
    hostile_params: bool  = False                        # numerically unstable draw
    ambient_extra_c: float = 0.0                         # worst-case cold offset


@dataclass
class MissionMinute:
    minute:     int
    altitude_m: float
    i_motor_a:  float
    solar_hour: float


def build_hale_mission(n_minutes: int = 480) -> list[MissionMinute]:
    """8-hour afternoon→dusk HALE segment (solar hour 12:00 → 20:00).
    Cruise at 3000 m; descent to 800 m in the final hour; HIGH-DRAIN event
    (18 A, minutes 400–460) lands exactly in the critical-SoC / no-sun corner.

    Altitude was retuned 4000→3000 m after the first campaign: at 4000 m the
    night closed-vent equilibrium sits BELOW T_MIN — the mission itself exceeds
    vent-only thermal authority (57% baseline breach, all honestly flagged
    INFEASIBLE). That is a real HALE design finding (a heater path is needed
    for -8 °C cruise), but the harness baseline must be feasible-but-tight for
    controller metrics to mean anything."""
    out = []
    for m in range(n_minutes):
        alt = 3000.0 if m < 420 else max(800.0, 3000.0 - (m - 420) * (2200.0 / 60.0))
        i_m = 18.0 if 400 <= m < 460 else 5.0
        out.append(MissionMinute(m, alt, i_m, 12.0 + m / 60.0))
    return out


# ══════════════════════════════════════════════════════════════════════
# The stress tester
# ══════════════════════════════════════════════════════════════════════

MPC_HORIZON   = 3
CAPACITY_AH   = 30.0
SOC_START     = 0.85
SOC_FLOOR     = 0.02
T_BAT_START   = 24.0
GUARD_ABS_T   = 500.0    # numerical-divergence guard on the plant


@dataclass
class TrialOutcome:
    breached:          bool
    warned:            bool       # any INFEASIBLE or monitor override ≤ breach
    min_true_t:        float
    min_soc:           float
    n_reactive:        int
    n_failsafe:        int
    cruise_frac:       float
    failsafe_latency:  list       # ticks, per stale-window onset
    diverged:          bool
    blackbox_on_fail:  list       # frozen ring at first breach (else empty)


class SoCStressTester:
    """Runs the layered system (MPC → SafetyMonitorRef → actuator → plant)
    through the 8-h mission with SoC dynamics, mismatch, and injected faults."""

    def __init__(self, cfg: MismatchConfig, faults: FaultConfig):
        self.cfg     = cfg
        self.faults  = faults
        self.mission = build_hale_mission()
        self.sun     = SolarModel(latitude_deg=45.0, day_of_year=172)

    def run_trial(self, rng: random.Random) -> TrialOutcome:
        f      = self.faults
        oracle = WeatherOracle(T_ground_base=18.0, V_wind_base=5.0)
        mpc    = ModelPredictiveController(
            oracle, ThermalSimulator(T_internal=T_BAT_START, **NOMINAL))
        mon    = SafetyMonitorRef()

        true_params = _sample_true_params(rng, self.cfg)
        if f.hostile_params:                       # numerically hostile draw
            true_params["h_air"] *= 25.0
        plant = ThermalSimulator(
            T_internal=T_BAT_START + rng.gauss(0.0, self.cfg.init_temp_sigma),
            **true_params)
        ambient_bias = rng.gauss(0.0, self.cfg.ambient_sigma)

        soc          = SOC_START
        vent_actual  = VENT_CRUISE
        last_good    = plant.T_internal
        ring         = deque(maxlen=64)

        min_t, min_soc   = plant.T_internal, soc
        breached = warned = diverged = False
        n_react = n_fs = n_cruise = 0
        bb_frozen: list = []
        fs_seen_after: dict[int, int | None] = {s: None for s, _ in f.stale_windows}

        for mm in self.mission:
            k = mm.minute
            sol = self.sun.state_at(mm.solar_hour)

            # ── SoC coulomb model ────────────────────────────────────
            soc += (sol.charge_current - mm.i_motor_a) / (60.0 * CAPACITY_AH)
            soc  = min(1.0, max(SOC_FLOOR, soc))
            min_soc = min(min_soc, soc)

            # ── Sensor (noise + drift + STALE injection) ─────────────
            stale = any(s <= k < s + n for s, n in f.stale_windows)
            t_meas = plant.T_internal + rng.gauss(0.0, self.cfg.sensor_sigma) + f.drift_bias_c
            if not stale:
                last_good = t_meas
            mpc_input_t = last_good           # host feeds last-good to the MPC

            # ── L1: MPC advisory ─────────────────────────────────────
            look = [m.altitude_m for m in self.mission[k:k + MPC_HORIZON]]
            while len(look) < MPC_HORIZON:
                look.append(look[-1])
            dec = mpc.predict_thermal_trajectory(
                current_temp=mpc_input_t, flight_plan=look,
                current_current_motor=mm.i_motor_a,
                current_current_solar=sol.charge_current,
                battery_soc=soc,
            )
            if dec.trigger_code == TriggerReason.BEST_EFFORT_INFEASIBLE and not breached:
                warned = True

            # ── L2: Safety Monitor ───────────────────────────────────
            cmd, verdict, faults = mon.step(t_meas, not stale, dec.vent_command)
            if verdict == V_OVR_REACTIVE:
                n_react += 1
                if not breached:
                    warned = True
            if verdict == V_OVR_FAILSAFE:
                n_fs += 1
                for s in fs_seen_after:
                    if k >= s and fs_seen_after[s] is None:
                        fs_seen_after[s] = k - s
            if dec.vent_command == VENT_CRUISE and verdict == V_PASS:
                n_cruise += 1

            # ── Actuator (STALL injection) ───────────────────────────
            if f.stall_window and f.stall_window[0] <= k < f.stall_window[0] + f.stall_window[1]:
                vent_actual = f.stall_window[2]
            else:
                vent_actual = cmd

            # ── TRUE plant ───────────────────────────────────────────
            atm = oracle.get_state_at_altitude(mm.altitude_m)
            plant.update(
                temp_esterna=atm.temp_ext_c + ambient_bias + f.ambient_extra_c,
                corrente_motore=mm.i_motor_a,
                corrente_solare=sol.charge_current,
                v_pitot=atm.wind_speed_ms,
                posizione_botola=vent_actual,
                p_solar_ext=sol.radiative_heat,
            )
            t = plant.T_internal
            # Numerical guard: non-finite, absurd magnitude, or a per-step jump
            # beyond any physical rate (catches bounded Euler OSCILLATION, not
            # just explosion — the forward-Euler stability bound (k+h·v)·dt/C<2
            # is violated in the hostile-parameter regime).
            if (not math.isfinite(t) or abs(t) > GUARD_ABS_T
                    or abs(plant.last_dT) > 25.0):
                diverged = True
                break
            min_t = min(min_t, t)

            ring.append(BlackBoxRecord(
                k, t, t_meas, not stale, soc, dec.soc_urgency,
                dec.vent_command, dec.trigger_code.name, dec.feasible,
                verdict, faults, cmd, vent_actual))

            if (t < T_MIN_SAFE or t > T_MAX_SAFE) and not breached:
                breached  = True
                bb_frozen = list(ring)           # freeze forensic window

        lat = [v for v in fs_seen_after.values() if v is not None]
        n_steps = max(1, len(self.mission))
        return TrialOutcome(breached, warned, min_t, min_soc, n_react, n_fs,
                            n_cruise / n_steps, lat, diverged, bb_frozen)

    def run_campaign(self, n_trials: int, seed: int) -> dict:
        rng = random.Random(seed)
        outs = [self.run_trial(rng) for _ in range(n_trials)]
        breaches = [o for o in outs if o.breached]
        silent   = [o for o in breaches if not o.warned]
        lat      = [v for o in outs for v in o.failsafe_latency]
        return {
            "n": n_trials,
            "breach":  len(breaches),
            "silent":  len(silent),
            "warned":  len(breaches) - len(silent),
            "react":   statistics.mean(o.n_reactive > 0 for o in outs),
            "fs":      statistics.mean(o.n_failsafe > 0 for o in outs),
            "cruise":  statistics.mean(o.cruise_frac for o in outs),
            "min_t":   min(o.min_true_t for o in outs),
            "min_soc": min(o.min_soc for o in outs),
            "lat_max": max(lat) if lat else None,
            "diverged": sum(o.diverged for o in outs),
            "bb": next((o.blackbox_on_fail for o in outs if o.blackbox_on_fail), []),
        }


# ══════════════════════════════════════════════════════════════════════
# KPIs and Safety Integrity Score
# ══════════════════════════════════════════════════════════════════════

SILENT_GATE   = 0.001   # KPI-1 hard gate: silent-breach rate (95% upper CI)
OVERRIDE_GATE = 0.02    # KPI-2 hard gate: L2-reactive engagement rate
CRUISE_REF    = 0.50    # KPI-3 reference cruise fraction (nominal economy)


def safety_integrity_score(silent_ci: float, react_rate: float,
                           cruise_frac: float) -> tuple[float, dict]:
    """
    Aggregate the three flight-readiness KPIs into one score via the WEAKEST
    LINK (min), never a weighted average — averaging would let efficiency mask
    a safety deficit.

      KPI-1 SAFETY      : silent-breach probability (95% upper CI) vs 0.1% gate
      KPI-2 CONTAINMENT : L2 reactive-override engagement rate (L2 firing means
                          L1 failed to contain — must be rare in nominal ops)
      KPI-3 ECONOMY     : cruise fraction vs nominal reference (mission
                          viability; a controller that is safe but never
                          cruises is not flight-ready either)
    """
    s1 = 0.0 if silent_ci >= SILENT_GATE else 1.0 - silent_ci / SILENT_GATE
    s2 = max(0.0, 1.0 - react_rate / OVERRIDE_GATE)
    s3 = min(1.0, cruise_frac / CRUISE_REF)
    return min(s1, s2, s3), {"safety": s1, "containment": s2, "economy": s3}


# ══════════════════════════════════════════════════════════════════════
# Campaign driver
# ══════════════════════════════════════════════════════════════════════

def _row(name, r):
    lat = f"{r['lat_max']}t" if r["lat_max"] is not None else "  -"
    return (f"  {name:<26} {r['n']:>4}  {r['breach']:>3} {r['silent']:>3}  "
            f"{100*r['react']:>5.1f} {100*r['fs']:>5.1f}  {100*r['cruise']:>5.1f}  "
            f"{r['min_t']:>+7.2f}  {r['min_soc']:>5.2f}  {lat:>4}  {r['diverged']:>3}")


def main() -> None:
    cfg  = MismatchConfig()
    seed = 20260703
    DIV  = "─" * 108

    print()
    print("  Helios — SoC Stress & Fault-Injection Campaign (8 h HALE mission, ±15% mismatch)")
    print(DIV)
    print(f"  {'scenario':<26} {'n':>4}  {'brc':>3} {'sil':>3}  "
          f"{'rct%':>5} {'fs%':>5}  {'cru%':>5}  {'minT°C':>7}  {'mSoC':>5}  {'lat':>4}  {'div':>3}")
    print(DIV)

    scenarios = [
        ("S0 nominal (no faults)",      200, FaultConfig()),
        ("S1 worst-case cold + drain",  200, FaultConfig(ambient_extra_c=-4.0)),
        ("S2 sensor STALE burst",       100, FaultConfig(stale_windows=[(200, 10), (430, 10)])),
        ("S3 vent STALLED open, dusk",  100, FaultConfig(stall_window=(410, 40, 1.0))),
        ("S4 sensor drift +2.0 °C",     100, FaultConfig(drift_bias_c=2.0)),
        ("S5 hostile params (Euler)",    40, FaultConfig(hostile_params=True)),
    ]
    results = {}
    for name, n, fc in scenarios:
        results[name] = SoCStressTester(cfg, fc).run_campaign(n, seed)
        print(_row(name, results[name]))
    print(DIV)

    # ── Sensitivity matrix: W_COMFORT × SOC_MARGIN_BOOST ───────────────
    print("\n  Sensitivity matrix (fault-free mission, 40 trials/cell): "
          "breach / L2-react% / cruise%")
    print(DIV)
    w0, b0 = mpc_core.W_COMFORT, mpc_core.SOC_MARGIN_BOOST
    try:
        header = "  W_COMFORT \\ BOOST " + "".join(f"{b:>22.1f}" for b in (0.5, 1.0, 2.0))
        print(header)
        for w in (0.25, 0.5, 1.0):
            cells = []
            for b in (0.5, 1.0, 2.0):
                mpc_core.W_COMFORT, mpc_core.SOC_MARGIN_BOOST = w, b
                r = SoCStressTester(cfg, FaultConfig()).run_campaign(40, seed)
                cells.append(f"{r['breach']}/{100*r['react']:.0f}%/{100*r['cruise']:.0f}%")
            print(f"  {w:>17.2f} " + "".join(f"{c:>22}" for c in cells))
    finally:
        mpc_core.W_COMFORT, mpc_core.SOC_MARGIN_BOOST = w0, b0
    print(DIV)

    # ── Safety Integrity Score from the fault-free scenarios ───────────
    base = results["S0 nominal (no faults)"]
    n_ff = base["n"]
    silent_ci = (_upper95_one_sided_zero(n_ff) if base["silent"] == 0
                 else base["silent"] / n_ff)
    sis, sub = safety_integrity_score(silent_ci, base["react"], base["cruise"])
    print(f"\n  SAFETY INTEGRITY SCORE (fault-free baseline, n={n_ff}): {sis:.3f}")
    print(f"    safety={sub['safety']:.3f}  containment={sub['containment']:.3f}  "
          f"economy={sub['economy']:.3f}   (weakest-link aggregation)")
    print(f"    NOTE: n={n_ff} ⇒ CI floor {100*_upper95_one_sided_zero(n_ff):.2f}% — "
          f"scale to ≥10k trials for a certification-grade score.")

    # ── Forensic reconstruction demo ────────────────────────────────────
    for name, r in results.items():
        if r["bb"]:
            print(f"\n  BLACK BOX — first breach in '{name}' (last ticks before/at breach):")
            for rec in r["bb"][-6:]:
                print(f"    t={rec.tick:>3}  true={rec.t_true:+6.2f}  meas={rec.t_meas:+6.2f}"
                      f"  ok={int(rec.sensor_ok)}  soc={rec.soc:.2f} u={rec.urgency:.2f}"
                      f"  mpc={rec.mpc_vent:.1f}({rec.trigger[:12]})"
                      f"  mon={rec.verdict}[{rec.faults:03d}]  out={rec.cmd_out:.1f}"
                      f"  act={rec.vent_actual:.1f}")
            break
    print()


if __name__ == "__main__":
    main()
