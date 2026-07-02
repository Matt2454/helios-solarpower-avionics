# SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Helios-Commercial
#
# PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
# Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
# Source redistribution is prohibited; shipped to integrators in compiled
# form only. Contains the pre-emptive thermal-vent MPC algorithm.
"""
Helios UAV Avionics — Model Predictive Controller
===================================================
Step 1.3: Predictive Thermal Brain (mpc_core.py)

Looks ahead along the flight plan, simulates future battery temperature
under three vent strategies (CLOSED / CRUISE / OPEN), and issues a
pre-emptive vent command before a thermal limit is breached.

MPC horizon  : one step per flight-plan waypoint (typically 1 min each)
Vent states  : 0.0 closed | 0.2 cruise | 1.0 fully open
Safety bands :
  T_bat < T_MIN (10 °C)  → CLOSE vent  (accumulate Joule heat)
  T_bat > T_MAX (45 °C)  → OPEN  vent  (maximum forced convection)
  else                   → CRUISE mode (0.2 — parasitic drag minimised)

Author : Helios UAV Avionics Team
Lead Dev: Matt
"""

import copy

from thermal_simulator import ThermalSimulator
from weather_oracle    import WeatherOracle


# ── Safety thresholds ──────────────────────────────────────────────────
T_MIN_SAFE:  float = 10.0   # °C — below this: battery at risk of cold damage
T_MAX_SAFE:  float = 45.0   # °C — above this: thermal runaway territory

# ── Vent command set ───────────────────────────────────────────────────
VENT_CLOSED: float = 0.0
VENT_CRUISE: float = 0.2
VENT_OPEN:   float = 1.0


class MPCDecision:
    """Result object returned by the MPC after a planning cycle."""

    def __init__(
        self,
        vent_command:      float,
        trigger_reason:    str,
        horizon_temps:     list[float],
        horizon_altitudes: list[float],
        cold_risk_at_min:  int | None,
        hot_risk_at_min:   int | None,
    ):
        self.vent_command      = vent_command       # float [0.0 – 1.0]
        self.trigger_reason    = trigger_reason     # human-readable label
        self.horizon_temps     = horizon_temps      # predicted T_bat per step
        self.horizon_altitudes = horizon_altitudes  # altitudes in the window
        self.cold_risk_at_min  = cold_risk_at_min   # first step at risk (cold)
        self.hot_risk_at_min   = hot_risk_at_min    # first step at risk (hot)

    def __repr__(self) -> str:
        return (
            f"MPCDecision(vent={self.vent_command:.1f}, "
            f"reason='{self.trigger_reason}', "
            f"cold_risk_min={self.cold_risk_at_min}, "
            f"hot_risk_min={self.hot_risk_at_min})"
        )


class ModelPredictiveController:
    """
    Predictive thermal controller for the Helios sealed battery box.

    Uses a shadow ThermalSimulator to roll out future battery temperature
    trajectories along the declared flight plan, then selects a vent
    command that keeps the battery inside its safe operating window.

    The real ThermalSimulator state is never modified during planning —
    only the shadow copy is mutated.

    Parameters
    ----------
    weather_oracle : WeatherOracle
        Shared oracle instance (may have anomalies active).
    base_simulator : ThermalSimulator
        The live simulator whose parameters (R, k, h, C) are copied
        into shadow instances during prediction rolls.
    """

    def __init__(
        self,
        weather_oracle: WeatherOracle,
        base_simulator: ThermalSimulator,
    ):
        self.oracle   = weather_oracle
        self.sim_live = base_simulator   # the "real" simulator — never mutated here

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _make_shadow(self, current_temp: float) -> ThermalSimulator:
        """
        Clone the live simulator parameters into a fresh shadow instance
        initialised at current_temp. Used exclusively inside prediction rolls.
        """
        shadow = ThermalSimulator(
            C_thermal    = self.sim_live.C_thermal,
            k_insulation = self.sim_live.k_insulation,
            R_internal   = self.sim_live.R_internal,
            h_air        = self.sim_live.h_air,
            T_internal   = current_temp,
        )
        return shadow

    def _roll_trajectory(
        self,
        current_temp:         float,
        flight_plan:          list[float],
        corrente_motore:      float,
        corrente_solare:      float,
        vent_position_fixed:  float,
    ) -> list[float]:
        """
        Simulate temperature evolution for every waypoint in flight_plan
        holding vent_position_fixed constant throughout the horizon.

        Returns a list of predicted T_bat values (one per waypoint).
        """
        shadow = self._make_shadow(current_temp)
        temps  = []

        for altitude_m in flight_plan:
            atm = self.oracle.get_state_at_altitude(altitude_m)
            shadow.update(
                temp_esterna     = atm.temp_ext_c,
                corrente_motore  = corrente_motore,
                corrente_solare  = corrente_solare,
                v_pitot          = atm.wind_speed_ms,
                posizione_botola = vent_position_fixed,
            )
            temps.append(shadow.T_internal)

        return temps

    def _first_breach(self, temps: list[float], threshold: float, above: bool) -> int | None:
        """Return 1-based index of the first step breaching the threshold, or None."""
        for i, t in enumerate(temps):
            if above and t > threshold:
                return i + 1
            if not above and t < threshold:
                return i + 1
        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict_thermal_trajectory(
        self,
        current_temp:         float,
        flight_plan:          list[float],
        current_current_motor: float,
        current_current_solar: float,
    ) -> MPCDecision:
        """
        Core MPC planning cycle.

        Rolls three parallel trajectories (CLOSED / CRUISE / OPEN),
        inspects each for safety violations, and returns the safest
        pre-emptive vent command.

        Decision priority
        -----------------
        1. If CRUISE trajectory breaches T_MIN → CLOSE  (accumulate heat)
        2. If CRUISE trajectory breaches T_MAX → OPEN   (dump heat)
        3. Otherwise                           → CRUISE (0.2, efficient)

        Parameters
        ----------
        current_temp           : current battery temperature [°C]
        flight_plan            : list of altitudes [m] for the next N minutes
        current_current_motor  : motor draw right now [A]
        current_current_solar  : solar charge right now [A]

        Returns
        -------
        MPCDecision with vent command and full diagnostic data
        """
        # Roll all three strategies
        t_closed = self._roll_trajectory(
            current_temp, flight_plan,
            current_current_motor, current_current_solar,
            VENT_CLOSED,
        )
        t_cruise = self._roll_trajectory(
            current_temp, flight_plan,
            current_current_motor, current_current_solar,
            VENT_CRUISE,
        )
        t_open = self._roll_trajectory(
            current_temp, flight_plan,
            current_current_motor, current_current_solar,
            VENT_OPEN,
        )

        # Detect risks on the CRUISE (default) trajectory
        cold_risk = self._first_breach(t_cruise, T_MIN_SAFE, above=False)
        hot_risk  = self._first_breach(t_cruise, T_MAX_SAFE, above=True)

        # Decision logic
        if cold_risk is not None:
            vent_cmd = VENT_CLOSED
            reason   = (
                f"PRE-EMPTIVE CLOSE — cruise trajectory hits "
                f"T_MIN ({T_MIN_SAFE}°C) at step {cold_risk}"
            )
            chosen_traj = t_closed

        elif hot_risk is not None:
            vent_cmd = VENT_OPEN
            reason   = (
                f"PRE-EMPTIVE OPEN  — cruise trajectory hits "
                f"T_MAX ({T_MAX_SAFE}°C) at step {hot_risk}"
            )
            chosen_traj = t_open

        else:
            vent_cmd = VENT_CRUISE
            reason   = "CRUISE — battery temperature within safe window"
            chosen_traj = t_cruise

        return MPCDecision(
            vent_command      = vent_cmd,
            trigger_reason    = reason,
            horizon_temps     = chosen_traj,
            horizon_altitudes = list(flight_plan),
            cold_risk_at_min  = cold_risk,
            hot_risk_at_min   = hot_risk,
        )


# ======================================================================
# Integration test — rapid climb into cold wave
# ======================================================================

if __name__ == "__main__":

    DIVIDER = "─" * 78
    COL     = "{:>5}  {:>9}  {:>10}  {:>10}  {:>8}  {:>10}  {:>14}"

    # ── System init ────────────────────────────────────────────────────
    oracle = WeatherOracle(T_ground_base=22.0, V_wind_base=4.5)
    sim    = ThermalSimulator(T_internal=28.0)   # warm battery at take-off
    mpc    = ModelPredictiveController(oracle, sim)

    # Activate cold front — the MPC must detect and react pre-emptively
    oracle.imposta_ondata_di_gelo(attiva=True)

    # Rapid climb plan: 500 m steps every minute, 0 → 4000 m
    FULL_FLIGHT_ALTITUDES = [
        500, 1000, 1500, 2000, 2500, 3000, 3500, 4000
    ]

    # Flight electrical constants (representative cruise values)
    I_MOTOR  = 8.0   # A
    I_SOLAR  = 3.0   # A

    print()
    print("  Helios UAV Avionics — MPC Core v1.3")
    print("  Step 1.3 — Predictive Thermal Brain · Rapid Climb / Cold Wave Test")
    print(f"  Cold wave active: YES  |  T_ground effective: "
          f"{oracle.T_ground_base - oracle.COLD_WAVE_DT:.1f} °C  "
          f"|  Initial T_bat: {sim.T_internal:.1f} °C")
    print(DIVIDER)
    print(COL.format(
        "Min", "Alt [m]", "T_ext [°C]", "T_bat [°C]",
        "Vent", "MPC cmd", "Decision"
    ))
    print(DIVIDER)

    prev_vent = VENT_CRUISE   # start in cruise mode

    for step, altitude in enumerate(FULL_FLIGHT_ALTITUDES):

        # Remaining horizon from this step onward
        look_ahead = FULL_FLIGHT_ALTITUDES[step:]

        # MPC planning cycle — uses shadow copies, live sim unchanged
        decision = mpc.predict_thermal_trajectory(
            current_temp          = sim.T_internal,
            flight_plan           = look_ahead,
            current_current_motor = I_MOTOR,
            current_current_solar = I_SOLAR,
        )

        # Atmospheric conditions at this altitude
        atm = oracle.get_state_at_altitude(altitude)

        # Advance the REAL simulator one step with the MPC command
        sim.update(
            temp_esterna     = atm.temp_ext_c,
            corrente_motore  = I_MOTOR,
            corrente_solare  = I_SOLAR,
            v_pitot          = atm.wind_speed_ms,
            posizione_botola = decision.vent_command,
        )

        # Collapse reason to a short column label
        if decision.vent_command == VENT_CLOSED:
            cmd_label = "CLOSE (0.0)"
            dec_label = f"⚠ COLD RISK min+{decision.cold_risk_at_min}"
        elif decision.vent_command == VENT_OPEN:
            cmd_label = "OPEN  (1.0)"
            dec_label = f"⚠ HOT  RISK min+{decision.hot_risk_at_min}"
        else:
            cmd_label = "CRUISE(0.2)"
            dec_label = "OK"

        print(COL.format(
            step + 1,
            int(altitude),
            f"{atm.temp_ext_c:+.2f}",
            f"{sim.T_internal:+.2f}",
            f"{prev_vent:.1f}",
            cmd_label,
            dec_label,
        ))

        prev_vent = decision.vent_command

    print(DIVIDER)
    print(f"  Final battery temperature  : {sim.T_internal:+.2f} °C")
    print(f"  Final MPC vent command     : {prev_vent:.1f}")
    cold_margin = sim.T_internal - T_MIN_SAFE
    print(f"  Safety margin vs T_MIN     : {cold_margin:+.2f} K  "
          f"({'SAFE' if cold_margin > 0 else 'BREACHED'})")
    print()
    print("  Helios UAV Avionics Team — mpc_core.py build OK")
    print()
