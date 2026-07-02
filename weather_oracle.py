# SPDX-FileCopyrightText: 2026 Helios Avionics
# SPDX-License-Identifier: Apache-2.0
#
# Open Component of the Helios Avionics Middleware (see LICENSING.md).
# Licensed under the Apache License, Version 2.0 (see LICENSE).
"""
Solar Plane UAV — Weather Oracle
==================================
Step 1.2: Atmospheric Data Model

Provides ambient temperature and wind speed estimates at any altitude
using standard tropospheric lapse rate physics. Designed to work with
real API data (Step 1.2b) or deterministic mock data for HIL testing.

Physics model:
  T_ext(h)   = T_ground - (6.5 × h / 1000)     [ISA lapse rate, °C]
  V_wind(h)  = V_wind_ground + (1.5 × h / 1000) [linear wind shear, m/s]
"""

from dataclasses import dataclass


@dataclass
class AtmosphericState:
    """Snapshot of atmospheric conditions at a given altitude."""
    altitude_m:    float   # [m]
    temp_ext_c:    float   # [°C]  — ambient air temperature
    wind_speed_ms: float   # [m/s] — estimated airspeed contribution from wind
    cold_wave:     bool    # True if a cold-front anomaly is active


class WeatherOracle:
    """
    Lumped atmospheric model for the Sealed Box thermal prediction chain.

    Computes ISA-derived temperature and empirical wind shear at any
    altitude above the launch site. Supports injection of anomaly events
    (cold fronts, thunderstorm cells) for stress-testing the MPC algorithm.

    Parameters
    ----------
    T_ground_base : float
        Sea-level / ground air temperature [°C]. Default 22.0 °C.
    V_wind_base : float
        Ground-level wind speed [m/s]. Default 4.5 m/s.

    Constants (ISA / empirical)
    ---------------------------
    LAPSE_RATE   : 6.5 °C / 1000 m  — standard tropospheric lapse rate
    WIND_SHEAR   : 1.5 m/s / 1000 m — linear wind-speed increase with altitude
    COLD_WAVE_DT : 15.0 °C          — temperature penalty for a cold-front event
    """

    LAPSE_RATE:   float = 6.5    # °C / 1000 m
    WIND_SHEAR:   float = 1.5    # m/s / 1000 m
    COLD_WAVE_DT: float = 15.0   # °C

    def __init__(
        self,
        T_ground_base: float = 22.0,
        V_wind_base:   float =  4.5,
    ):
        self._T_ground_base = T_ground_base
        self._V_wind_base   = V_wind_base
        self._cold_wave_active = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_ambient_temp_at_altitude(self, altitude_meters: float) -> float:
        """
        Return estimated ambient air temperature at a given altitude.

        T_ext = T_ground - (LAPSE_RATE × altitude / 1000)

        If a cold-wave event is active, COLD_WAVE_DT is subtracted from
        the effective ground temperature before applying the lapse rate,
        propagating the anomaly across the entire altitude profile.

        Parameters
        ----------
        altitude_meters : float — altitude above ground level [m]

        Returns
        -------
        float : ambient temperature [°C]
        """
        T_ground_effective = self._T_ground_base
        if self._cold_wave_active:
            T_ground_effective -= self.COLD_WAVE_DT

        return T_ground_effective - (self.LAPSE_RATE * altitude_meters / 1000.0)

    def get_wind_speed_at_altitude(self, altitude_meters: float) -> float:
        """
        Return estimated wind speed at a given altitude.

        V_wind = V_wind_base + (WIND_SHEAR × altitude / 1000)

        Wind speed is clamped to 0 m/s (never negative).

        Parameters
        ----------
        altitude_meters : float — altitude above ground level [m]

        Returns
        -------
        float : wind speed [m/s]
        """
        v = self._V_wind_base + (self.WIND_SHEAR * altitude_meters / 1000.0)
        return max(0.0, v)

    def get_state_at_altitude(self, altitude_meters: float) -> AtmosphericState:
        """
        Convenience method: returns a full AtmosphericState snapshot.
        Ready for direct injection into ThermalSimulator.update().
        """
        return AtmosphericState(
            altitude_m    = altitude_meters,
            temp_ext_c    = self.get_ambient_temp_at_altitude(altitude_meters),
            wind_speed_ms = self.get_wind_speed_at_altitude(altitude_meters),
            cold_wave     = self._cold_wave_active,
        )

    def set_cold_wave(self, active: bool) -> None:
        """
        Inject / remove a cold-front anomaly (stress test).

        When active, ground temperature is artificially reduced by
        COLD_WAVE_DT (15 °C), simulating sudden entry into a thunderstorm
        cell or a sharp cold front. The penalty propagates through the
        lapse-rate formula to all altitudes.

        Parameters
        ----------
        active : bool — True to activate the cold-wave event
        """
        self._cold_wave_active = active

    # Alias with Italian name as required by the architecture spec
    def imposta_ondata_di_gelo(self, attiva: bool) -> None:
        """Italian alias for set_cold_wave() — architecture spec compliance."""
        self.set_cold_wave(attiva)

    @property
    def T_ground_base(self) -> float:
        return self._T_ground_base

    @property
    def V_wind_base(self) -> float:
        return self._V_wind_base

    @property
    def cold_wave_active(self) -> bool:
        return self._cold_wave_active


# ======================================================================
# Integration test — altitude profile tables
# ======================================================================

if __name__ == "__main__":

    ALTITUDES_M  = range(0, 4500, 500)   # 0 → 4000 m, step 500 m
    DIVIDER      = "─" * 62
    COL          = "{:>10}  {:>14}  {:>14}  {:>14}"

    oracle = WeatherOracle()

    def print_profile_table(label: str) -> None:
        cold = oracle.cold_wave_active
        anomaly_tag = "  ⚠  COLD WAVE ACTIVE  (−15 °C penalty)" if cold else ""
        print()
        print(f"  {label}{anomaly_tag}")
        print(DIVIDER)
        print(COL.format("Alt [m]", "T_ext [°C]", "V_wind [m/s]", "Status"))
        print(DIVIDER)
        for alt in ALTITUDES_M:
            state  = oracle.get_state_at_altitude(alt)
            # Flag operationally relevant thresholds
            if state.temp_ext_c < -10.0:
                status = "CRITICAL — FREEZE RISK"
            elif state.temp_ext_c < 0.0:
                status = "CAUTION  — BELOW 0°C"
            elif state.temp_ext_c < 10.0:
                status = "WATCH    — COLD AIR"
            else:
                status = "NOMINAL"
            print(COL.format(
                f"{int(alt)}",
                f"{state.temp_ext_c:+.2f}",
                f"{state.wind_speed_ms:.2f}",
                status,
            ))
        print(DIVIDER)
        print(COL.format(
            "Ground base",
            f"{oracle.T_ground_base:+.2f} °C",
            f"{oracle.V_wind_base:.2f} m/s",
            f"Cold wave: {'ON' if cold else 'OFF'}",
        ))

    # ── Profile 1: standard conditions ─────────────────────────────────
    print()
    print("  Solar Plane UAV — Weather Oracle v1.2")
    print("  Step 1.2 — Atmospheric Profile · Mock Data Test")

    print_profile_table("PROFILE A — STANDARD CONDITIONS")

    # ── Profile 2: cold-wave anomaly ───────────────────────────────────
    oracle.imposta_ondata_di_gelo(attiva=True)
    print_profile_table("PROFILE B — COLD WAVE / THUNDERSTORM CELL")

    print()
