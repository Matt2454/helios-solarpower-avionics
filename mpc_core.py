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

from enum import IntEnum

from thermal_simulator import ThermalSimulator
from weather_oracle    import WeatherOracle


# ── Safety thresholds ──────────────────────────────────────────────────
T_MIN_SAFE:  float = 10.0   # °C — below this: battery at risk of cold damage
T_MAX_SAFE:  float = 45.0   # °C — above this: thermal runaway territory

# ── Vent command set ───────────────────────────────────────────────────
VENT_CLOSED: float = 0.0
VENT_CRUISE: float = 0.2
VENT_OPEN:   float = 1.0

# ── Receding-horizon objective weights ─────────────────────────────────
# The MPC selects a vent command by minimising a scalar cost over each
# candidate's PREDICTED trajectory:
#
#     J(v) = W_SAFETY · Σ band_violation(T_k)  +  W_EFFORT · effort(v)
#
# Safety dominates efficiency by orders of magnitude: any predicted breach of
# the [T_MIN, T_MAX] band outweighs every possible vent-effort saving. Among
# strategies that keep the whole horizon in-band, the lowest-effort (most
# aerodynamically efficient) one wins.
W_SAFETY: float = 1000.0   # cost per Kelvin·step of band violation
W_EFFORT: float =    1.0   # cost per unit of vent effort (parasitic-drag proxy)

# Vent-effort cost per candidate. CRUISE is the efficient nominal (zero cost);
# CLOSED is a deliberate heat-retention action (slightly off-nominal); OPEN is
# the most expensive (maximum parasitic drag from forced convection). Encoding
# the preference here — rather than in branchy if/else logic — is what lets the
# optimiser default to CRUISE yet still choose CLOSE/OPEN when safety demands.
VENT_EFFORT: dict[float, float] = {
    VENT_CRUISE: 0.0,
    VENT_CLOSED: 0.3,
    VENT_OPEN:   1.0,
}

# Numerical tolerance below which a residual band violation counts as zero.
_FEASIBLE_EPS: float = 1e-9

# ── Certified Safety Buffer (cold-side robustness back-off) ─────────────
# Default t_min_margin, applied unless the integrator overrides it. Validated by
# the Monte-Carlo margin sweep (validation_montecarlo.py, Integration Manual §8):
# at ±15% component tolerance + sensor/ambient noise, 1.5 K yields 0 breaches
# across 10,000 randomised aircraft — 95% one-sided CI < 0.03% breach rate.
# Applied BY DEFAULT ("safety by design"): a controller left unconfigured is
# already robust, not merely nominal.
DEFAULT_T_MIN_MARGIN: float = 1.5


class TriggerReason(IntEnum):
    """
    Wire-ready reason code accompanying every vent decision.

    Emitted as an integer so it can be carried directly in a MAVLink field
    (e.g. HELIOS_THERMAL_DECISION.trigger_code) without string marshalling.
    The human-readable `trigger_reason` string remains available for logs.
    """
    CRUISE_NOMINAL         = 0   # horizon stays in-band under nominal cruise vent
    PREEMPTIVE_CLOSE_COLD  = 1   # closed pre-emptively to retain heat (cold threat)
    PREEMPTIVE_OPEN_HOT    = 2   # opened pre-emptively to dump heat (hot threat)
    BEST_EFFORT_INFEASIBLE = 3   # NO vent strategy keeps the horizon in-band


class MPCDecision:
    """Result object returned by the MPC after a planning cycle."""

    def __init__(
        self,
        vent_command:      float,
        trigger_reason:    str,
        trigger_code:      "TriggerReason",
        feasible:          bool,
        cost:              float,
        horizon_temps:     list[float],
        horizon_altitudes: list[float],
        cold_risk_at_min:  int | None,
        hot_risk_at_min:   int | None,
    ):
        self.vent_command      = vent_command       # float [0.0 – 1.0]
        self.trigger_reason    = trigger_reason     # human-readable label
        self.trigger_code      = trigger_code       # wire-ready enum (MAVLink)
        self.feasible          = feasible           # horizon kept in-band?
        self.cost              = cost               # objective value of choice
        self.horizon_temps     = horizon_temps      # predicted T_bat per step
        self.horizon_altitudes = horizon_altitudes  # altitudes in the window
        self.cold_risk_at_min  = cold_risk_at_min   # first step at risk (cold)
        self.hot_risk_at_min   = hot_risk_at_min    # first step at risk (hot)

    def __repr__(self) -> str:
        return (
            f"MPCDecision(vent={self.vent_command:.1f}, "
            f"code={self.trigger_code.name}, "
            f"feasible={self.feasible}, cost={self.cost:.3f}, "
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

    Robust MPC (constraint tightening)
    ----------------------------------
    The controller plans against a *tightened* thermal band
        [T_MIN + t_min_margin,  T_MAX − t_max_margin]
    rather than the raw certified limits. This back-off is what makes the
    controller robust: because the predictor uses a NOMINAL plant model, model
    mismatch and sensor error can push the TRUE battery colder/hotter than
    predicted. Holding the *prediction* a margin inside the limits keeps the
    *true* trajectory inside the certified [T_MIN, T_MAX] band. The required
    margin (the "Certified Safety Buffer") is tuned empirically by the
    Monte-Carlo harness — see validation_montecarlo.py.

    Parameters
    ----------
    weather_oracle : WeatherOracle
        Shared oracle instance (may have anomalies active).
    base_simulator : ThermalSimulator
        The live simulator whose parameters (R, k, h, C) are copied
        into shadow instances during prediction rolls.
    t_min_margin : float
        Cold-side back-off [K]. Effective floor = T_MIN_SAFE + t_min_margin.
        Defaults to the validated Certified Safety Buffer (DEFAULT_T_MIN_MARGIN
        = 1.5 K) so the controller is robust even if left unconfigured.
    t_max_margin : float
        Hot-side back-off [K]. Effective ceiling = T_MAX_SAFE − t_max_margin.
        Defaults to 0.0: the hot limit was non-binding across all validation
        trials (>20 K margin), so no back-off is warranted by the data.
    """

    def __init__(
        self,
        weather_oracle: WeatherOracle,
        base_simulator: ThermalSimulator,
        t_min_margin:   float = DEFAULT_T_MIN_MARGIN,
        t_max_margin:   float = 0.0,
    ):
        self.oracle   = weather_oracle
        self.sim_live = base_simulator   # the "real" simulator — never mutated here

        # Robustness back-off and the resulting effective (tightened) limits the
        # controller actually enforces. Defaults of 0.0 reproduce nominal MPC.
        self.t_min_margin = t_min_margin
        self.t_max_margin = t_max_margin
        self.t_min_eff    = T_MIN_SAFE + t_min_margin
        self.t_max_eff    = T_MAX_SAFE - t_max_margin

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

    def _trajectory_cost(
        self,
        temps:         list[float],
        vent_position: float,
    ) -> tuple[float, float]:
        """
        Score one candidate trajectory against the receding-horizon objective.

            J = W_SAFETY · Σ band_violation(T_k)  +  W_EFFORT · effort(vent)

        band_violation is the per-step distance OUTSIDE the TIGHTENED band
        [t_min_eff, t_max_eff] (0 while in-band), summed over the whole horizon.
        Using the tightened band — not the raw certified limits — is what turns
        this into a *robust* MPC: the controller is penalised for merely
        approaching the limits, leaving a buffer to absorb model mismatch.

        Returns
        -------
        (cost, total_violation)
            cost            : full objective value (lower is better)
            total_violation : summed band violation [K·steps]; 0.0 ⇒ feasible
        """
        total_violation = 0.0
        for t in temps:
            if t < self.t_min_eff:
                total_violation += (self.t_min_eff - t)
            elif t > self.t_max_eff:
                total_violation += (t - self.t_max_eff)

        effort = VENT_EFFORT[vent_position]
        cost   = W_SAFETY * total_violation + W_EFFORT * effort
        return cost, total_violation

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
        Core MPC planning cycle — receding-horizon cost minimisation.

        Rolls every candidate vent strategy (CLOSED / CRUISE / OPEN) forward
        over the full horizon, scores each PREDICTED trajectory against the
        safety-weighted objective (see _trajectory_cost), and commits the
        cost-minimising command.

        This is a true predictive selection, not a reactive heuristic:
          * The chosen action is scored on ITS OWN trajectory, so the command
            is only issued if it actually keeps the horizon in-band — the
            controller can no longer "CLOSE to fix cold" without verifying that
            closing works.
          * Cold and hot threats within the same horizon are handled uniformly
            (both contribute to each candidate's violation sum), rather than by
            a fixed cold-before-hot branch order.
          * If NO strategy can keep the horizon in-band, the least-violating
            one is returned and flagged BEST_EFFORT_INFEASIBLE, signalling the
            caller (safety layer) that vent authority alone is insufficient and
            escalation is required (e.g. shed motor current, abort the climb).

        Parameters
        ----------
        current_temp           : current battery temperature [°C]
        flight_plan            : list of altitudes [m] for the next N minutes
        current_current_motor  : motor draw right now [A]
        current_current_solar  : solar charge right now [A]

        Returns
        -------
        MPCDecision with vent command, wire-ready trigger code, feasibility,
        objective cost and full diagnostic data.
        """
        candidates = (VENT_CLOSED, VENT_CRUISE, VENT_OPEN)

        # ── 1. Roll and score every candidate over the full horizon ────────
        trajectories: dict[float, list[float]] = {}
        costs:        dict[float, float]        = {}
        violations:   dict[float, float]        = {}
        for v in candidates:
            temps = self._roll_trajectory(
                current_temp, flight_plan,
                current_current_motor, current_current_solar,
                v,
            )
            cost, violation = self._trajectory_cost(temps, v)
            trajectories[v] = temps
            costs[v]        = cost
            violations[v]   = violation

        # ── 2. Diagnose the THREAT on the do-nothing (cruise) trajectory ───
        # These indices explain *why* an action is taken (what would happen if
        # we held nominal cruise); the action itself is chosen by optimisation.
        cruise_temps = trajectories[VENT_CRUISE]
        cold_risk = self._first_breach(cruise_temps, self.t_min_eff, above=False)
        hot_risk  = self._first_breach(cruise_temps, self.t_max_eff, above=True)

        # ── 3. Select the cost-minimising vent command ─────────────────────
        best_vent   = min(candidates, key=lambda v: costs[v])
        chosen_traj = trajectories[best_vent]
        feasible    = violations[best_vent] <= _FEASIBLE_EPS

        # ── 4. Classify the decision (wire code + human string) ────────────
        if not feasible:
            code   = TriggerReason.BEST_EFFORT_INFEASIBLE
            reason = (
                f"BEST-EFFORT — no vent strategy keeps the horizon in-band; "
                f"min residual violation {violations[best_vent]:.2f} K·steps "
                f"at vent={best_vent:.1f}"
            )
        elif best_vent == VENT_CLOSED:
            code   = TriggerReason.PREEMPTIVE_CLOSE_COLD
            reason = (
                f"PRE-EMPTIVE CLOSE — cruise would cross the effective floor "
                f"({self.t_min_eff:.1f}°C = T_MIN+{self.t_min_margin:.1f}) at "
                f"step {cold_risk}; closing keeps the horizon in-band"
            )
        elif best_vent == VENT_OPEN:
            code   = TriggerReason.PREEMPTIVE_OPEN_HOT
            reason = (
                f"PRE-EMPTIVE OPEN  — cruise would cross the effective ceiling "
                f"({self.t_max_eff:.1f}°C = T_MAX−{self.t_max_margin:.1f}) at "
                f"step {hot_risk}; opening keeps the horizon in-band"
            )
        else:
            code   = TriggerReason.CRUISE_NOMINAL
            reason = "CRUISE — battery temperature within safe window"

        return MPCDecision(
            vent_command      = best_vent,
            trigger_reason    = reason,
            trigger_code      = code,
            feasible          = feasible,
            cost              = costs[best_vent],
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
