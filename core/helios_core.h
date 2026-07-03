// SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Helios-Commercial
//
// PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
// Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
//
// helios_core.h — Zero-allocation C++ port target of the thermal MPC
//
// PORT CONTRACT: mpc_core.py is the REFERENCE model. This implementation must
// reproduce its outputs on shared golden vectors (same inputs ⇒ same decision,
// temperatures within float tolerance) before it may fly. Where the reference
// has known modelling smells (e.g. the I_motor² + I_solar² Joule term), this
// port reproduces them bit-faithfully — model corrections happen in the
// reference first, then re-port. Never "fix physics" silently in the port.
//
// Embedded contract:
//   * Zero heap: all buffers are fixed-size members or stack POD.
//   * No exceptions, no RTTI, no STL containers across any boundary.
//   * step() is a pure function of (inputs, construction parameters):
//     deterministic, bounded O(N_CANDIDATES · HORIZON_MAX), no hidden state.
//   * The core never touches hardware — it consumes a measured temperature and
//     produces an ADVISORY decision. Actuation goes through the L2
//     SafetyMonitor (core/safety_monitor.h) and then the HAL.
//
// Architectural note vs the Python reference: the C++ core takes a
// PRE-RESOLVED atmospheric plan (T_ext / wind per step) instead of calling a
// weather oracle. The host resolves altitudes → atmosphere before each cycle.
// This removes the oracle dependency from the flight core entirely — smaller
// port surface, and the core stays a pure function.

#ifndef HELIOS_CORE_H
#define HELIOS_CORE_H

#include <cstdint>

namespace helios::core {

// ── Compile-time envelope (mirrors mpc_core.py) ─────────────────────────────
constexpr float   T_MIN_SAFE           = 10.0f;  // °C — certified cold limit
constexpr float   T_MAX_SAFE           = 45.0f;  // °C — certified hot limit
constexpr float   DEFAULT_T_MIN_MARGIN = 1.5f;   // K  — Certified Safety Buffer
constexpr float   DT_S                 = 60.0f;  // s  — thermal step (ref model)

// Objective weights (mirrors mpc_core.py — keep in lockstep with the reference)
constexpr float   W_SAFETY   = 1000.0f;
constexpr float   W_EFFORT   = 1.0f;
constexpr float   W_COMFORT  = 0.5f;   // PROVISIONAL — pending harness tuning
constexpr float   T_COMFORT  = 0.5f * (T_MIN_SAFE + T_MAX_SAFE);

// Energy-awareness (mirrors mpc_core.py)
constexpr float   SOC_CRITICAL     = 0.20f;
constexpr float   SOC_RESERVE      = 0.05f;
constexpr float   SOC_MARGIN_BOOST = 1.0f;  // K at urgency 1.0 — PROVISIONAL

// Candidate set. Indices are stable and shared with telemetry.
constexpr uint8_t N_CANDIDATES = 3;
constexpr float   VENT_LEVEL [N_CANDIDATES] = { 0.0f, 0.2f, 1.0f }; // CLOSE/CRUISE/OPEN
constexpr float   VENT_EFFORT[N_CANDIDATES] = { 0.3f, 0.0f, 1.0f };

// Horizon bound. The planner never allocates: plans longer than this are
// truncated (and the truncation is visible in the decision).
constexpr uint8_t HORIZON_MAX = 16;

constexpr float   FEASIBLE_EPS = 1e-6f;

// ── Wire-ready trigger codes (mirrors mpc_core.TriggerReason) ───────────────
enum class Trigger : uint8_t {
    CRUISE_NOMINAL         = 0,
    PREEMPTIVE_CLOSE_COLD  = 1,
    PREEMPTIVE_OPEN_HOT    = 2,
    BEST_EFFORT_INFEASIBLE = 3,
};

// ── POD I/O types ───────────────────────────────────────────────────────────

struct PlantParams {                 // nominal ("datasheet") thermal model
    float c_thermal    = 1500.0f;    // J/K
    float k_insulation = 0.15f;      // W/K
    float r_internal   = 0.03f;      // Ω
    float h_air        = 0.5f;       // W·s/(m·K)
};

struct AtmoStep {                    // pre-resolved atmosphere for one step
    float t_ext_c = 0.0f;
    float wind_ms = 0.0f;
};

struct CoreInputs {
    float    t_meas_c  = 0.0f;       // validated pack temperature (from HAL/monitor)
    float    i_motor_a = 0.0f;
    float    i_solar_a = 0.0f;
    float    soc       = 1.0f;       // pack state of charge [0..1]
    AtmoStep plan[HORIZON_MAX];      // resolved flight-plan atmosphere
    uint8_t  plan_len  = 0;          // steps actually populated (≤ HORIZON_MAX)
};

struct CoreDecision {
    float   vent_cmd     = 0.2f;     // ADVISORY — must pass the SafetyMonitor
    Trigger trigger      = Trigger::CRUISE_NOMINAL;
    bool    feasible     = true;
    float   cost         = 0.0f;
    float   soc_urgency  = 0.0f;
    float   t_min_active = 0.0f;     // cold floor enforced this cycle
    float   traj[HORIZON_MAX] = {};  // predicted T_bat for the chosen candidate
    uint8_t traj_len     = 0;
};

// ── The controller ──────────────────────────────────────────────────────────

class ThermalMpcCore {
public:
    explicit ThermalMpcCore(const PlantParams& p,
                            float t_min_margin = DEFAULT_T_MIN_MARGIN,
                            float t_max_margin = 0.0f)
        : p_(p),
          t_min_eff_(T_MIN_SAFE + t_min_margin),
          t_max_eff_(T_MAX_SAFE - t_max_margin) {}

    /// One planning cycle. Deterministic; no allocation; bounded time.
    CoreDecision step(const CoreInputs& in) {
        CoreDecision d{};
        const uint8_t n =
            (in.plan_len > HORIZON_MAX) ? HORIZON_MAX : in.plan_len;

        // Energy-awareness (mirrors mpc_core._soc_urgency)
        const float u   = socUrgency(in.soc);
        const float flr = t_min_eff_ + SOC_MARGIN_BOOST * u;

        // Roll and score every candidate into pre-allocated buffers.
        float cost[N_CANDIDATES];
        float viol[N_CANDIDATES];
        for (uint8_t c = 0; c < N_CANDIDATES; ++c) {
            rollout(in, n, VENT_LEVEL[c], traj_[c]);
            score(traj_[c], n, VENT_EFFORT[c], u, flr, t_max_eff_,
                  cost[c], viol[c]);
        }

        // Cost-minimising candidate.
        uint8_t best = 0;
        for (uint8_t c = 1; c < N_CANDIDATES; ++c)
            if (cost[c] < cost[best]) best = c;

        // Threat diagnosis on the do-nothing (CRUISE, index 1) trajectory.
        bool cold_risk = false, hot_risk = false;
        for (uint8_t k = 0; k < n; ++k) {
            if (traj_[1][k] < flr)        { cold_risk = true; break; }
            if (traj_[1][k] > t_max_eff_) { hot_risk  = true; break; }
        }

        d.vent_cmd     = VENT_LEVEL[best];
        d.feasible     = viol[best] <= FEASIBLE_EPS;
        d.cost         = cost[best];
        d.soc_urgency  = u;
        d.t_min_active = flr;
        d.traj_len     = n;
        for (uint8_t k = 0; k < n; ++k) d.traj[k] = traj_[best][k];

        if      (!d.feasible) d.trigger = Trigger::BEST_EFFORT_INFEASIBLE;
        else if (best == 0)   d.trigger = Trigger::PREEMPTIVE_CLOSE_COLD;
        else if (best == 2)   d.trigger = Trigger::PREEMPTIVE_OPEN_HOT;
        else                  d.trigger = Trigger::CRUISE_NOMINAL;

        (void)cold_risk; (void)hot_risk;  // exported via telemetry in the host
        return d;
    }

private:
    static float socUrgency(float soc) {
        if (soc >= SOC_CRITICAL) return 0.0f;
        const float span = SOC_CRITICAL - SOC_RESERVE;
        const float u    = (SOC_CRITICAL - soc) / span;
        return (u < 0.0f) ? 0.0f : (u > 1.0f ? 1.0f : u);
    }

    /// Forward-Euler rollout of the reference thermal model at fixed vent.
    /// Bit-faithful to ThermalSimulator.update() — including the known
    /// (I_motor² + I_solar²)·R Joule term; see PORT CONTRACT above.
    void rollout(const CoreInputs& in, uint8_t n, float vent, float* out) const {
        float t = in.t_meas_c;
        const float p_joule =
            (in.i_motor_a * in.i_motor_a + in.i_solar_a * in.i_solar_a)
            * p_.r_internal;
        for (uint8_t k = 0; k < n; ++k) {
            const float dte    = in.plan[k].t_ext_c - t;
            const float p_cond = p_.k_insulation * dte;
            const float p_conv = p_.h_air * vent * in.plan[k].wind_ms * dte;
            t += (p_joule + p_cond + p_conv) * DT_S / p_.c_thermal;
            out[k] = t;
        }
    }

    /// Energy-aware objective (mirrors mpc_core._trajectory_cost):
    ///   J = W_SAFETY·viol + W_EFFORT·(1−u)·effort + W_COMFORT·u·Σ|T−T_COMFORT|
    static void score(const float* temps, uint8_t n, float effort, float u,
                      float flr, float ceil, float& cost_out, float& viol_out) {
        float viol = 0.0f, load = 0.0f;
        for (uint8_t k = 0; k < n; ++k) {
            const float t = temps[k];
            if      (t < flr)                 viol += (flr - t);
            else if (t > ceil)                viol += (t - ceil);
            load += (t > T_COMFORT) ? (t - T_COMFORT) : (T_COMFORT - t);
        }
        viol_out = viol;
        cost_out = W_SAFETY * viol
                 + W_EFFORT * (1.0f - u) * effort
                 + W_COMFORT * u * load;
    }

    PlantParams p_;
    float t_min_eff_;
    float t_max_eff_;
    float traj_[N_CANDIDATES][HORIZON_MAX];  // pre-allocated rollout buffers
};

} // namespace helios::core

#endif // HELIOS_CORE_H
