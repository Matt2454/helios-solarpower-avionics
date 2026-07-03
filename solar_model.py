# SPDX-FileCopyrightText: 2026 Helios Avionics
# SPDX-License-Identifier: Apache-2.0
#
# Open Component of the Helios Avionics Middleware (see LICENSING.md).
# Licensed under the Apache License, Version 2.0 (see LICENSE).
"""
Helios UAV Avionics — Solar Model
==================================
Simplified clear-sky solar model for a solar-powered glider. Turns time of day
(and site/date) into the two ways sunlight touches the battery:

  1. ELECTRICAL path — irradiance on the panels → charge current (I_solar),
     which warms the pack through I²R Joule heating.
  2. THERMAL path — direct radiative heating of the enclosure surface
     (P_solar_ext [W]), independent of the electrical path.

Geometry (standard, published):
  declination   δ = 23.45° · sin(360°·(284 + day_of_year)/365)
  hour angle    H = 15°·(solar_hour − 12)
  elevation     sin(α) = sin(lat)·sin(δ) + cos(lat)·cos(δ)·cos(H)
  irradiance    G = G0 · max(0, sin(α))     [clear-sky, horizontal]

The two output coefficients are deliberately LUMPED (area × efficiency, and
area × absorptivity) and tuned so peak-sun magnitudes are commensurate with the
existing ThermalSimulator power terms. They are the obvious per-airframe
calibration knobs.
"""

import math
from dataclasses import dataclass


@dataclass
class SolarState:
    """Snapshot of the solar environment at one instant."""
    solar_hour:    float   # local solar time [h]
    elevation_deg: float   # sun elevation above horizon [°] (negative = night)
    irradiance:    float   # clear-sky horizontal irradiance [W/m²]
    charge_current: float  # panel charge current into the pack [A]
    radiative_heat: float  # direct enclosure heating [W]


class SolarModel:
    """
    Clear-sky solar model. Defaults describe a mid-latitude summer flight.

    Parameters
    ----------
    latitude_deg : site latitude [°]
    day_of_year  : 1–365 (drives declination); default ≈ summer solstice
    g0           : clear-sky peak irradiance at altitude [W/m²]
    panel_coeff  : lumped panel area × efficiency / bus voltage → [A per W/m²]
    absorb_coeff : lumped exposed area × absorptivity          → [W per W/m²]
    """

    def __init__(
        self,
        latitude_deg: float = 45.0,
        day_of_year:  int   = 172,     # ~21 June
        g0:           float = 1100.0,  # W/m² (clear sky, elevated glider)
        panel_coeff:  float = 0.0035,  # → ~3.9 A at G0 (matches cruise I_solar)
        absorb_coeff: float = 0.0040,  # → ~4.4 W radiative at G0
    ):
        self.latitude_deg = latitude_deg
        self.day_of_year  = day_of_year
        self.g0           = g0
        self.panel_coeff  = panel_coeff
        self.absorb_coeff = absorb_coeff

    def _declination_deg(self) -> float:
        return 23.45 * math.sin(math.radians(360.0 * (284 + self.day_of_year) / 365.0))

    def elevation_deg(self, solar_hour: float) -> float:
        """Sun elevation angle above the horizon [°] at the given solar hour."""
        lat = math.radians(self.latitude_deg)
        dec = math.radians(self._declination_deg())
        hour_angle = math.radians(15.0 * (solar_hour - 12.0))
        sin_elev = (math.sin(lat) * math.sin(dec)
                    + math.cos(lat) * math.cos(dec) * math.cos(hour_angle))
        sin_elev = max(-1.0, min(1.0, sin_elev))
        return math.degrees(math.asin(sin_elev))

    def irradiance(self, solar_hour: float) -> float:
        """Clear-sky horizontal irradiance [W/m²]; 0 when the sun is down."""
        elev = self.elevation_deg(solar_hour)
        if elev <= 0.0:
            return 0.0
        return self.g0 * math.sin(math.radians(elev))

    def state_at(self, solar_hour: float) -> SolarState:
        """Full solar snapshot ready to feed ThermalSimulator.update()."""
        elev = self.elevation_deg(solar_hour)
        g    = self.irradiance(solar_hour)
        return SolarState(
            solar_hour     = solar_hour,
            elevation_deg  = elev,
            irradiance     = g,
            charge_current = self.panel_coeff * g,
            radiative_heat = self.absorb_coeff * g,
        )


# ======================================================================
# Demonstration — how solar input moves battery temperature
# ======================================================================

if __name__ == "__main__":
    from thermal_simulator import ThermalSimulator

    DIV = "─" * 72
    sun = SolarModel(latitude_deg=45.0, day_of_year=172)

    print()
    print("  Helios UAV Avionics — Solar Model")
    print("  Clear-sky profile (lat 45°, ~21 June) and its thermal effect")
    print(DIV)
    print(f"  {'Hour':>5}  {'Elev°':>7}  {'G [W/m²]':>9}  "
          f"{'I_sol [A]':>9}  {'P_sol [W]':>9}")
    print(DIV)
    for h in range(4, 21, 2):
        s = sun.state_at(float(h))
        print(f"  {h:>5.0f}  {s.elevation_deg:>7.1f}  {s.irradiance:>9.1f}  "
              f"{s.charge_current:>9.2f}  {s.radiative_heat:>9.2f}")
    print(DIV)

    # ── Thermal comparison: 15 min of cold cruise, sun HIGH vs DOWN ────
    # Same cold-wave cruise conditions; only the solar input differs.
    def cruise_15min(solar_hour: float | None) -> float:
        sim = ThermalSimulator(T_internal=16.0)   # already cold-soaked
        for _ in range(15):
            if solar_hour is None:                 # night / no sun baseline
                i_sol, p_sol = 0.0, 0.0
            else:
                st = sun.state_at(solar_hour)
                i_sol, p_sol = st.charge_current, st.radiative_heat
            sim.update(
                temp_esterna    = -20.0,           # cold air at altitude
                corrente_motore = 5.0,             # cruise
                corrente_solare = i_sol,
                v_pitot         = 15.0,
                posizione_botola= 0.0,             # vent closed (retain heat)
                p_solar_ext     = p_sol,
            )
        return sim.T_internal

    t_night = cruise_15min(None)
    t_dusk  = cruise_15min(18.0)
    t_noon  = cruise_15min(12.0)

    print("  15-minute cold cruise (T_ext −20°C, vent closed), T_bat start 16.0°C")
    print(DIV)
    print(f"  No sun (night)   : {t_night:+.2f} °C")
    print(f"  Dusk  (18:00)    : {t_dusk:+.2f} °C   (Δ vs night {t_dusk - t_night:+.2f} K)")
    print(f"  Solar noon (12)  : {t_noon:+.2f} °C   (Δ vs night {t_noon - t_night:+.2f} K)")
    print(DIV)
    print("  Takeaway: solar gain is a warming term. It is strongest at midday and")
    print("  vanishes at dusk/night — so the cold-breach risk the MPC guards against")
    print("  is worst exactly when the sun can't help (low-light glide, dawn, dusk).")
    print()
