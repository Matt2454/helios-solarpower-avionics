"""
Solar Plane UAV — Thermal Simulator
====================================
Step 1.1: Sealed Box Thermal Model

Simulates the battery pack temperature evolution inside
an insulated polystyrene box with an active servo-driven vent.

Physics model (dt = 60 s per tick):
  P_joule     = (I_motor² + I_solar²) × R_internal     [Joule effect]
  P_conduction = k_insulation × (T_ext - T_int)         [Passive wall conduction]
  P_convection = h_air × vent_position × v_pitot        [Forced convection via vent]
                 × (T_ext - T_int)
  P_total     = P_joule + P_conduction + P_convection
  dT          = (P_total × dt) / C_thermal
  T_int_new   = T_int + dT


"""


class ThermalSimulator:
    """
    Lumped-parameter thermal model of the battery sealed box.

    The box is treated as a single thermal mass (the battery pack)
    exchanging heat with the environment through three mechanisms:
      1. Internal generation  — Joule heating from charge/discharge currents
      2. Passive conduction   — heat leak through polystyrene walls
      3. Forced convection    — active cooling when the servo vent is open

    Parameters
    ----------
    C_thermal : float
        Thermal capacity of the battery pack [J/K].
        Default 1500 J/K ≈ a small LiPo pack + enclosure.
    k_insulation : float
        Global conduction coefficient of the box walls [W/K].
        Lumps area, thickness and polystyrene λ into one number.
    R_internal : float
        Total internal resistance of the cell pack [Ω].
        Used in Joule: P = I² × R.
    h_air : float
        Vent effectiveness coefficient [W/(m/s · K)].
        Scales convective power with Pitot speed and ΔT.
    T_internal : float
        Initial battery temperature [°C].
    dt : int
        Integration time step [s]. Fixed at 60 s (1 minute).
    """

    DT: int = 60  # seconds — fixed time step

    def __init__(
        self,
        C_thermal: float    = 1500.0,   # J/K
        k_insulation: float =    0.15,  # W/K
        R_internal: float   =    0.03,  # Ω
        h_air: float        =    0.5,   # W·s/(m·K)  — vent effectiveness
        T_internal: float   =   20.0,   # °C
    ):
        self.C_thermal    = C_thermal
        self.k_insulation = k_insulation
        self.R_internal   = R_internal
        self.h_air        = h_air
        self.T_internal   = T_internal

        # Diagnostic breakdown of the last tick — useful for logging / GCS
        self.last_P_joule      = 0.0
        self.last_P_conduction = 0.0
        self.last_P_convection = 0.0
        self.last_P_total      = 0.0
        self.last_dT           = 0.0

    # ------------------------------------------------------------------
    # Core physics
    # ------------------------------------------------------------------

    def _joule_heating(self, I_motor: float, I_solar: float) -> float:
        """
        P_joule = (I_motor² + I_solar²) × R_internal   [W]

        Both currents contribute positive heat regardless of sign
        (discharge warms cells; charge also warms cells via internal R).
        """
        return (I_motor ** 2 + I_solar ** 2) * self.R_internal

    def _conduction(self, T_external: float) -> float:
        """
        P_conduction = k_insulation × (T_ext − T_int)   [W]

        Positive → heat flows in (cold battery, warm outside).
        Negative → heat leaks out (hot battery, cold altitude air).
        """
        return self.k_insulation * (T_external - self.T_internal)

    def _convection(
        self,
        T_external: float,
        v_pitot: float,
        vent_position: float,
    ) -> float:
        """
        P_convection = h_air × vent_position × v_pitot × (T_ext − T_int)   [W]

        vent_position ∈ [0.0, 1.0]
          0.0 → vent fully closed, no forced convection
          1.0 → vent fully open, maximum airflow

        Negative value → cooling effect (hot battery, cold outside air).
        """
        vent_position = max(0.0, min(1.0, vent_position))  # clamp to [0, 1]
        return (
            self.h_air
            * vent_position
            * v_pitot
            * (T_external - self.T_internal)
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        temp_esterna: float,
        corrente_motore: float,
        corrente_solare: float,
        v_pitot: float,
        posizione_botola: float,
    ) -> float:
        """
        Advance the thermal state by one time step (60 s).

        Parameters
        ----------
        temp_esterna     : Ambient / outside air temperature [°C]
        corrente_motore  : Motor current draw (discharge)    [A]
        corrente_solare  : Solar panel charge current        [A]
        v_pitot          : Airspeed measured by Pitot tube   [m/s]
        posizione_botola : Servo vent opening, 0.0–1.0       [-]

        Returns
        -------
        float : Updated internal battery temperature [°C]
        """
        P_joule      = self._joule_heating(corrente_motore, corrente_solare)
        P_conduction = self._conduction(temp_esterna)
        P_convection = self._convection(temp_esterna, v_pitot, posizione_botola)

        P_total = P_joule + P_conduction + P_convection
        dT      = (P_total * self.DT) / self.C_thermal

        self.T_internal += dT

        # Store diagnostics for external inspection
        self.last_P_joule      = P_joule
        self.last_P_conduction = P_conduction
        self.last_P_convection = P_convection
        self.last_P_total      = P_total
        self.last_dT           = dT

        return self.T_internal

    def reset(self, T_internal: float = 20.0) -> None:
        """Reset internal temperature (useful between test scenarios)."""
        self.T_internal = T_internal


# ======================================================================
# Integration test — critical scenario
# ======================================================================

if __name__ == "__main__":

    HEADER = "─" * 72
    COL    = "{:>5}  {:>10}  {:>10}  {:>10}  {:>10}  {:>10}  {:>10}"

    sim = ThermalSimulator()

    print()
    print("  Solar Plane UAV — Thermal Simulator v1.1")
    print("  Step 1.1 — Sealed Box · Critical Flight Scenario")
    print(HEADER)

    # ── Phase 1: take-off (0–5 min) ────────────────────────────────────
    print("\n  PHASE 1 · TAKE-OFF  |  vent CLOSED (0.0)  |  motor 25 A")
    print(HEADER)
    print(COL.format(
        "Min", "T_int °C", "P_Joule W", "P_Cond W", "P_Conv W", "P_tot W", "dT K"
    ))
    print(HEADER)

    for minute in range(1, 6):
        sim.update(
            temp_esterna     = 20.0,   # ground-level ambient
            corrente_motore  = 25.0,   # full throttle take-off
            corrente_solare  =  2.0,   # modest solar on climb
            v_pitot          = 12.0,   # climbing airspeed [m/s]
            posizione_botola =  0.0,   # vent closed
        )
        print(COL.format(
            minute,
            f"{sim.T_internal:+.2f}",
            f"{sim.last_P_joule:+.2f}",
            f"{sim.last_P_conduction:+.2f}",
            f"{sim.last_P_convection:+.2f}",
            f"{sim.last_P_total:+.2f}",
            f"{sim.last_dT:+.4f}",
        ))

    # ── Phase 2: cruise cooling (6–10 min) ─────────────────────────────
    print()
    print("  PHASE 2 · CRUISE COOLING  |  vent OPEN (1.0)  |  T_ext 5 °C  |  Pitot 15 m/s")
    print(HEADER)
    print(COL.format(
        "Min", "T_int °C", "P_Joule W", "P_Cond W", "P_Conv W", "P_tot W", "dT K"
    ))
    print(HEADER)

    for minute in range(6, 11):
        sim.update(
            temp_esterna     =  5.0,   # cold air at altitude
            corrente_motore  =  8.0,   # cruise throttle
            corrente_solare  =  3.0,   # good solar at altitude
            v_pitot          = 15.0,   # cruise airspeed [m/s]
            posizione_botola =  1.0,   # vent fully open
        )
        print(COL.format(
            minute,
            f"{sim.T_internal:+.2f}",
            f"{sim.last_P_joule:+.2f}",
            f"{sim.last_P_conduction:+.2f}",
            f"{sim.last_P_convection:+.2f}",
            f"{sim.last_P_total:+.2f}",
            f"{sim.last_dT:+.4f}",
        ))

    print(HEADER)
    print(f"\n  Final battery temperature : {sim.T_internal:+.2f} °C")
    print()
