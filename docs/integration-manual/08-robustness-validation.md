<!--
SPDX-FileCopyrightText: 2026 Helios Avionics
SPDX-License-Identifier: Apache-2.0
Open Component — Helios Avionics Middleware Integration Manual (see LICENSING.md).
-->

# 8. Robustness Validation & the Certified Safety Buffer

> **Purpose.** This section documents how the Helios thermal MPC is validated
> against real-world uncertainty — component tolerance, sensor noise, and model
> error — and defines the **Certified Safety Buffer** that ships enabled by
> default. It is written so an integrating engineer can reproduce every number.

## 8.1 Why nominal simulation is not evidence

A controller that predicts with a perfect copy of the plant it controls will
always "succeed" — success is guaranteed by construction, and proves only that
the control *logic* is internally consistent. It says nothing about behaviour on
a *real* aircraft, whose battery resistance, insulation, vent effectiveness, and
thermal mass differ from any datasheet model, and whose temperature sensor is
noisy.

Helios is therefore validated under deliberate **model mismatch**: the MPC plans
with a *nominal* model and a *noisy* measurement, while the simulated *true*
plant runs on *perturbed* physics it cannot see.

```
        ┌─────────────────────────┐      noisy measured T_bat
        │  MPC predictor          │◄──────────────────────────────┐
        │  (NOMINAL params +      │                                │
        │   robustness margin)    │                                │
        └───────────┬─────────────┘                                │
                    │ vent command                                 │
                    ▼                                              │
        ┌─────────────────────────┐   true (hidden) T_bat          │
        │  TRUE plant             │────────────────────────────────┘
        │  (PERTURBED params,     │
        │   + ambient disturbance)│
        └─────────────────────────┘
```

## 8.2 Injected uncertainty

All quantities below are **invisible to the controller** and drawn independently
per simulated aircraft (harness: `validation_montecarlo.py`).

| Source | Distribution | Rationale |
|---|---|---|
| `R_internal` (battery resistance) | ±15% uniform | Cell-to-cell + ageing tolerance |
| `k_insulation` (wall conductance) | ±15% uniform | Build / material tolerance |
| `h_air` (vent effectiveness) | ±15% uniform | Airflow / geometry tolerance |
| `C_thermal` (thermal mass) | ±15% uniform | Pack / enclosure tolerance |
| Initial battery temperature | Gaussian, σ = 1.5 °C | Pre-flight thermal state spread |
| Battery-temp sensor | Gaussian, σ = 0.5 °C | Sensor noise / drift / jitter |
| Ambient temperature | Gaussian bias, σ = 2.0 °C | Weather the oracle did not predict |

Scenario: the cold-wave stress mission (20-minute climb/cruise/descent, ground
air artificially reduced 15 °C). Certified band: **[10 °C, 45 °C]**.

## 8.3 Outcome classification

Breach is always judged against the **true certified limits**, never the
tightened band the controller plans to. Each trial is one of:

| Outcome | Definition | Acceptable? |
|---|---|---|
| **SAFE** | No breach of [10, 45] °C. | Yes |
| **WARNED** | Breach occurred, but the MPC raised `BEST_EFFORT_INFEASIBLE` at or before it. | Tolerable — the aircraft has notice and can escalate (shed load, abort). |
| **SILENT** | Breach with **no prior warning**. | **No — disqualifying.** The controller was confident and wrong. |

The engineering goal is not merely "few breaches" but **zero silent breaches**:
a safety architecture can be built around a controller that admits it cannot
hold; it cannot be built around one that fails silently.

## 8.4 Robust MPC: constraint tightening

The MPC is made robust by **constraint tightening**: it plans against a
*tightened* band `[T_MIN + t_min_margin, T_MAX − t_max_margin]` rather than the
raw limits. Because prediction uses a nominal model, holding the *prediction* a
margin inside the limits keeps the *true* trajectory inside the certified band.
The required cold-side margin is tuned empirically below. (The hot side was
non-binding — >20 K of margin in every trial — so `t_max_margin` defaults to 0.)

## 8.5 Margin sweep (data-driven tuning)

The identical 500-aircraft population (fixed seed) was replayed at each candidate
`t_min_margin`, so the **only** variable is the buffer.

| `t_min_margin` (K) | SAFE % | WARNED % | SILENT % | Worst realized margin (K) | 5th-pct margin (K) | Cruise % (efficiency) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 0.0 | 95.6 | 2.2 | **2.2** | −0.49 | +0.01 | 51.7 |
| 0.5 | 99.8 | 0.2 | **0.0** | −0.21 | +0.51 | 49.2 |
| **1.0** | 100.0 | 0.0 | **0.0** | +0.51 | +1.01 | 47.1 |
| **1.5** | 100.0 | 0.0 | **0.0** | +1.01 | +1.52 | 44.8 |
| 2.0 | 100.0 | 0.0 | 0.0 | +1.26 | +2.00 | 42.6 |
| 2.5 | 100.0 | 0.0 | 0.0 | +1.86 | +2.54 | 40.3 |
| 3.0 | 100.0 | 0.0 | 0.0 | +2.52 | +3.04 | 38.1 |
| 3.5 | 100.0 | 0.0 | 0.0 | +2.74 | +3.52 | 35.9 |
| 4.0 | 100.0 | 0.0 | 0.0 | +3.12 | +4.04 | 33.6 |

Reading:
- **0.5 K** removes all *silent* breaches, but one aircraft still breaches (−0.21 K) with warning.
- **1.0 K** is the first margin with **zero breaches of any kind**.
- The efficiency cost is small and roughly linear (~2.3 cruise-points per 0.5 K).

## 8.6 Certified Safety Buffer & confirmation

The production buffer is set to **`t_min_margin = 1.5 K`** — one full degree of
margin above the zero-breach floor (1.0 K), to cover (a) the finite-sample
confidence bound and (b) tolerances exceeding the ±15% assumption.

It was then confirmed on an **independent** 10,000-aircraft population (different
seed — validation, not the tuning set):

| Metric | Result |
|---|---|
| SAFE | **10,000 / 10,000 (100.000%)** |
| WARNED / SILENT | 0 / 0 |
| Worst realized margin | **+0.43 K** vs T_MIN (no breach) |
| 5th-percentile margin | +1.48 K |
| Silent-breach rate | 0 observed → **< 0.030%** (95% one-sided CI) |
| Any-breach rate | 0 observed → **< 0.030%** (95% one-sided CI) |

The confidence bound uses the exact one-sided Clopper-Pearson limit for zero
events, `p < 1 − 0.05^(1/n)`. At n = 10,000 this is **< 0.03%**, an order of
magnitude below a 0.1% acceptance threshold.

**Safety by design:** 1.5 K is the *default* value of `t_min_margin` in
`ModelPredictiveController`. An integrator who never configures a margin still
runs in the certified-robust configuration.

## 8.7 Reproduce it yourself

```bash
PYTHONUTF8=1 python validation_montecarlo.py
```

Runs the full margin sweep and the 10,000-trial confirmation (~7 s). All
parameters — tolerances, noise, trial count, sweep grid — are in
`MismatchConfig` / module constants at the top of the harness. Re-run with your
own airframe's tolerances to derive a buffer specific to your hardware.

## 8.8 Scope & limitations (read before relying on this)

This validation is rigorous **within its stated envelope**, and deliberately
states that envelope:

- **Parametric, not structural, mismatch.** The true plant perturbs the
  *parameters* of the reference thermal model; it does not introduce a
  *different* model structure (e.g. multi-node dynamics, phase change). Real
  batteries may exhibit effects the lumped model omits.
- **±15% is an assumption.** Integrators whose components vary more widely must
  re-run the sweep and re-derive the buffer.
- **Single mission profile.** Validated on the cold-wave stress mission; deploy
  against your own mission set before flight.
- **Simulation, not flight test.** This is a simulated flight test and evidence
  of design diligence — **not** a substitute for hardware-in-the-loop testing,
  airworthiness certification, or the integrator's own V&V. Helios remains an
  advisory subsystem (see §7).

---

*Artifacts: `validation_montecarlo.py` (harness), `mpc_core.py`
(`DEFAULT_T_MIN_MARGIN`, `t_min_margin`/`t_max_margin`). Results above generated
at seed 20260702 (sweep) / 20260703 (confirmation).*
