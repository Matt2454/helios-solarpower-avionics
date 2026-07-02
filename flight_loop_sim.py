# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# Internal integration/validation harness that exercises the proprietary core.
"""
Helios UAV Avionics — Flight Loop Simulator
============================================
Step 1.4 (Phase 1 finale): Integrated Mission Simulator

Coordinates ThermalSimulator, WeatherOracle and ModelPredictiveController
across a full 20-minute flight profile, producing a per-minute mission log.

Mission profile
---------------
  Min  1–5  : Climb   0 → 2500 m  | motor 12 A | solar  1 A
  Min  6–15 : Cruise  @ 3500 m    | motor  5 A | solar  4 A
  Min 16–20 : Descent 3500 → 0 m  | motor  0 A | solar  0 A (glide/dusk)

MPC look-ahead : 3 minutes (rolling horizon)
Cold wave      : ACTIVE (stress scenario)

Author   : Helios UAV Avionics Team
Lead Dev : Matt
"""

from dataclasses import dataclass

from thermal_simulator import ThermalSimulator
from weather_oracle    import WeatherOracle
from mpc_core          import ModelPredictiveController, T_MIN_SAFE, T_MAX_SAFE


# ── Operational limits (repeated here for status logic) ───────────────
T_WARN_COLD: float = 13.0   # °C — early warning margin above T_MIN
T_WARN_HOT:  float = 40.0   # °C — early warning margin below T_MAX
MPC_HORIZON: int   = 3      # minutes look-ahead


# ======================================================================
# Mission profile generator
# ======================================================================

@dataclass
class FlightMinute:
    minute:          int
    altitude_m:      float
    current_motor_a: float
    current_solar_a: float


def generate_mission_profile() -> list[FlightMinute]:
    """
    Build the full 20-minute flight plan as a list of FlightMinute records.

    Altitude and current values are linearly interpolated between
    phase endpoints so the simulator sees smooth, realistic transitions
    rather than hard steps.
    """
    profile: list[FlightMinute] = []

    def lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    # Phase A — Climb: minutes 1–5
    for i in range(5):
        t = i / 4.0 if i < 4 else 1.0
        profile.append(FlightMinute(
            minute          = i + 1,
            altitude_m      = lerp(0.0, 2500.0, t),
            current_motor_a = 12.0,
            current_solar_a =  1.0,
        ))

    # Phase B — Cruise: minutes 6–15
    for i in range(10):
        profile.append(FlightMinute(
            minute          = i + 6,
            altitude_m      = 3500.0,
            current_motor_a =  5.0,
            current_solar_a =  4.0,
        ))

    # Phase C — Descent: minutes 16–20
    for i in range(5):
        t = i / 4.0 if i < 4 else 1.0
        profile.append(FlightMinute(
            minute          = i + 16,
            altitude_m      = lerp(3500.0, 0.0, t),
            current_motor_a =  0.0,
            current_solar_a =  0.0,
        ))

    return profile


# ======================================================================
# Status classifier
# ======================================================================

def classify_status(t_bat: float, vent: float, phase: str) -> str:
    """Return a concise system status string for the mission log."""
    if t_bat < T_MIN_SAFE:
        return "!! CRITICAL  COLD BREACH"
    if t_bat > T_MAX_SAFE:
        return "!! CRITICAL  HOT  BREACH"
    if t_bat < T_WARN_COLD:
        return "⚠  WARN     LOW TEMP"
    if t_bat > T_WARN_HOT:
        return "⚠  WARN     HIGH TEMP"
    if vent == 0.0 and phase == "CRUISE":
        return "»  WARMING   VENT CLOSED"
    if vent == 1.0:
        return "»  COOLING   VENT OPEN"
    return "✓  NOMINAL"


# ======================================================================
# Main flight loop
# ======================================================================

def run_mission() -> None:

    # ── System initialisation ──────────────────────────────────────────
    oracle = WeatherOracle(T_ground_base=18.0, V_wind_base=5.0)
    sim    = ThermalSimulator(T_internal=22.0)
    mpc    = ModelPredictiveController(oracle, sim)

    oracle.imposta_ondata_di_gelo(attiva=True)   # activate stress scenario

    mission = generate_mission_profile()

    # Phase labels indexed by minute
    def phase_label(minute: int) -> str:
        if minute <= 5:  return "CLIMB"
        if minute <= 15: return "CRUISE"
        return "DESCENT"

    # ── Report header ──────────────────────────────────────────────────
    DIVIDER = "═" * 88
    SUBDIV  = "─" * 88
    COL = (
        "{:>4}  {:>8}  {:>10}  {:>9}  {:>9}  {:>9}  {:>7}  {:<24}"
    )

    print()
    print("  Helios UAV Avionics Team — Flight Loop Simulator v1.4")
    print("  Phase 1 Integration Test · Full 20-Minute Mission")
    print(f"  Cold wave: ACTIVE  |  T_ground effective: "
          f"{oracle.T_ground_base - oracle.COLD_WAVE_DT:.1f} °C  "
          f"|  MPC horizon: {MPC_HORIZON} min  "
          f"|  T_bat start: {sim.T_internal:.1f} °C")
    print(DIVIDER)
    print(COL.format(
        "Min", "Phase", "Alt [m]", "T_ext °C",
        "I_mot A", "Vent", "T_bat", "Status"
    ))
    print(DIVIDER)

    # Tracking variables
    prev_vent      = 0.2          # start in cruise mode
    min_t_bat      = sim.T_internal
    max_t_bat      = sim.T_internal
    cold_breaches  = 0
    hot_breaches   = 0
    last_phase     = ""

    for fm in mission:

        phase = phase_label(fm.minute)

        # Phase transition separator
        if phase != last_phase:
            if last_phase:
                print(SUBDIV)
            last_phase = phase

        # Build rolling MPC look-ahead (next MPC_HORIZON waypoints)
        future_idx   = mission.index(fm)
        look_ahead   = [
            m.altitude_m
            for m in mission[future_idx: future_idx + MPC_HORIZON]
        ]
        # Pad with last altitude if near end of mission
        while len(look_ahead) < MPC_HORIZON:
            look_ahead.append(look_ahead[-1])

        # MPC decision (shadow simulation — real sim unchanged)
        decision = mpc.predict_thermal_trajectory(
            current_temp           = sim.T_internal,
            flight_plan            = look_ahead,
            current_current_motor  = fm.current_motor_a,
            current_current_solar  = fm.current_solar_a,
        )

        # Real atmospheric conditions at current altitude
        atm = oracle.get_state_at_altitude(fm.altitude_m)

        # Advance real simulator with MPC-chosen vent
        sim.update(
            temp_esterna     = atm.temp_ext_c,
            corrente_motore  = fm.current_motor_a,
            corrente_solare  = fm.current_solar_a,
            v_pitot          = atm.wind_speed_ms,
            posizione_botola = decision.vent_command,
        )

        # Status
        status = classify_status(sim.T_internal, decision.vent_command, phase)

        # Breach counters
        if sim.T_internal < T_MIN_SAFE:
            cold_breaches += 1
        if sim.T_internal > T_MAX_SAFE:
            hot_breaches  += 1

        # Running extremes
        min_t_bat = min(min_t_bat, sim.T_internal)
        max_t_bat = max(max_t_bat, sim.T_internal)

        print(COL.format(
            fm.minute,
            phase,
            f"{fm.altitude_m:.0f}",
            f"{atm.temp_ext_c:+.2f}",
            f"{fm.current_motor_a:.1f}",
            f"{decision.vent_command:.1f}",
            f"{sim.T_internal:+.2f}",
            status,
        ))

        prev_vent = decision.vent_command

    # ── Mission summary ────────────────────────────────────────────────
    print(DIVIDER)
    print()
    print("  ── MISSION SUMMARY ──────────────────────────────────────────")
    print(f"  Final battery temperature  : {sim.T_internal:+.2f} °C")
    print(f"  Min T_bat observed         : {min_t_bat:+.2f} °C  "
          f"(margin vs T_MIN: {min_t_bat - T_MIN_SAFE:+.2f} K)")
    print(f"  Max T_bat observed         : {max_t_bat:+.2f} °C  "
          f"(margin vs T_MAX: {T_MAX_SAFE - max_t_bat:+.2f} K)")
    print(f"  Cold breaches (< {T_MIN_SAFE:.0f} °C)    : {cold_breaches}")
    print(f"  Hot  breaches (> {T_MAX_SAFE:.0f} °C)    : {hot_breaches}")
    overall = "MISSION SUCCESS" if (cold_breaches + hot_breaches) == 0 else "MISSION FAILED — THERMAL BREACH"
    print(f"  Overall result             : {overall}")
    print()
    print("  Helios UAV Avionics Team — flight_loop_sim.py build OK")
    print()


if __name__ == "__main__":
    run_mission()
