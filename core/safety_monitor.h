// SPDX-FileCopyrightText: 2026 Helios Avionics. All rights reserved.
// SPDX-License-Identifier: LicenseRef-Helios-Commercial
//
// PROPRIETARY AND CONFIDENTIAL — Core Component of the Helios Avionics
// Middleware (see LICENSING.md). Licensed, not sold, under LICENSE-CORE.
//
// safety_monitor.h — L2 Safety Monitor (run-time assurance)
//
// Sits BETWEEN the L1 advisory controller (thermal MPC) and the vent actuator.
// Every command the MPC issues passes through step(); the monitor holds full
// override authority and the MPC holds none.
//
// Design rule (Simplex / run-time-assurance pattern): the monitor is
// deliberately DUMBER than the controller it guards — pure reactive threshold
// logic with a handful of states — so it can be exhaustively table-tested and
// reviewed line-by-line. It shares no code with the MPC: a defect in the
// predictive core cannot also live here (no common-mode failure).
//
// Override precedence (highest wins):
//   1. OVERRIDE_FAILSAFE  — inputs untrustworthy: hold the failsafe posture
//   2. OVERRIDE_REACTIVE  — measured temperature at/over a hard limit:
//                           bang-bang response, MPC ignored entirely
//   3. CLAMPED            — advisory accepted but range-clamped
//   4. PASS               — advisory forwarded (slew-shaped)
//
// Embedded contract: no dynamic allocation, no exceptions, fixed-size state,
// deterministic — same inputs + same internal state ⇒ same output.

#ifndef HELIOS_SAFETY_MONITOR_H
#define HELIOS_SAFETY_MONITOR_H

#include <cstdint>

#include "../hal/helios_hal.h"

namespace helios::safety {

// ── Fault bitmask reported to the host autopilot ────────────────────────────
// Carried verbatim in HELIOS_SAFETY_STATE.faults (MAVLink dialect). Any set bit
// above FAULT_CMD_SLEW_LIMITED should raise a caution on the GCS; either
// OVERRIDE_* verdict below means Helios advisory output is degraded and the
// autopilot should treat thermal optimisation as unavailable.
enum FaultBits : uint16_t {
    FAULT_NONE             = 0,
    FAULT_SENSOR_INVALID   = 1u << 0,  // current sample not OK (STALE/OOR/FAULT)
    FAULT_SENSOR_PERSIST   = 1u << 1,  // invalid streak reached debounce limit
    FAULT_MPC_STALE        = 1u << 2,  // no fresh advisory within timeout
    FAULT_ENVELOPE_COLD    = 1u << 3,  // measured T at/below hard cold floor
    FAULT_ENVELOPE_HOT     = 1u << 4,  // measured T at/above hard hot ceiling
    FAULT_CMD_OUT_OF_RANGE = 1u << 5,  // advisory outside [0,1]; clamped
    FAULT_CMD_SLEW_LIMITED = 1u << 6,  // advisory shaped by the slew limiter
    FAULT_ACTUATOR_STALL   = 1u << 7,  // commanded vent not achieved (jam/fault)
};

enum class Verdict : uint8_t {
    PASS = 0,           // advisory forwarded (possibly slew-shaped)
    CLAMPED,            // advisory range-clamped, then forwarded
    OVERRIDE_REACTIVE,  // hard envelope violated: monitor substituted bang-bang
    OVERRIDE_FAILSAFE,  // inputs untrustworthy / advisory stale: failsafe posture
};

struct Config {
    // HARD limits — the true certified band, NOT the MPC's tightened band.
    // The MPC plans conservatively inside these; the monitor only fires when
    // reality (the measured temperature) actually reaches the real limit.
    float    t_min_hard_c    = 10.0f;
    float    t_max_hard_c    = 45.0f;

    float    vent_failsafe   = 0.2f;   // blind posture when nothing is known
    // Temperature-conditioned failsafe bounds (validation finding S2: a STATIC
    // failsafe posture during a sensor blackout actively cooled a cold-marginal
    // pack and caused the breach it should prevent — see
    // validation_soc_stress.py). Below/above these, the blind posture becomes
    // CLOSE (retain heat) / OPEN (dump heat) based on the last good sample.
    float    failsafe_cold_below_c = 20.0f;
    float    failsafe_hot_above_c  = 40.0f;
    float    slew_per_step   = 0.25f;  // max |Δcmd| per step() call
    uint8_t  sensor_debounce = 3;      // consecutive bad samples → PERSIST fault
    float    stall_tol       = 0.15f;  // |actual − commanded| beyond this = stuck
    uint8_t  stall_debounce  = 3;      // consecutive stuck cycles → STALL fault
    uint32_t mpc_timeout_ms  = 10000;  // advisory staleness bound; set to
                                       // 3–5× the advisory period per integration
};

// Last advisory received from the L1 controller (MPC), with provenance.
struct Advisory {
    float    vent_norm     = 0.2f;
    uint32_t timestamp_ms  = 0;
    bool     ever_received = false;
};

struct Output {
    float    vent_cmd = 0.2f;   // the command actually sent to the HAL actuator
    Verdict  verdict  = Verdict::OVERRIDE_FAILSAFE;
    uint16_t faults   = FAULT_NONE;
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(const Config& cfg) : cfg_(cfg) {}

    /// One monitor cycle. Call every control tick, AFTER reading the sensor and
    /// BEFORE writing the actuator:   sensor → MPC (advisory) → step() → HAL.
    ///
    /// @param vent_actual  actuator position feedback from
    ///   IVentActuator::actualPosition() (this cycle's reading reflects LAST
    ///   cycle's command). Pass a negative value if the actuator has no
    ///   feedback — stall detection is then skipped, not falsely tripped.
    ///
    /// A confirmed FAULT_ACTUATOR_STALL is ESCALATION, not mitigation: a jammed
    /// single vent cannot be corrected in software. The bit tells the autopilot
    /// that thermal authority is degraded so it can take a FLIGHT action (e.g.
    /// descend to warmer air). The monitor still issues its best command.
    Output step(const hal::TemperatureReading& temp,
                const Advisory&                mpc,
                float                          vent_actual,
                uint32_t                       now_ms) {
        Output out{};

        // ── 0. Actuator-stall detection ────────────────────────────────────
        // Did LAST cycle's command actually take effect? Compare the feedback
        // against the command we issued. Debounced so a servo mid-travel is not
        // mistaken for a jam. Detected here (top) so the bit is reported on
        // EVERY return path, including the overrides below.
        if (last_cmd_ >= 0.0f && vent_actual >= 0.0f) {
            const float e  = vent_actual - last_cmd_;
            const float ae = (e < 0.0f) ? -e : e;
            if (ae > cfg_.stall_tol) { if (stall_streak_ < 0xFF) ++stall_streak_; }
            else                     { stall_streak_ = 0; }
        }
        uint16_t faults =
            (stall_streak_ >= cfg_.stall_debounce) ? FAULT_ACTUATOR_STALL
                                                   : FAULT_NONE;

        // ── 1. Sensor validation filter (STALE / OUT_OF_RANGE / FAULT) ─────
        const bool sample_ok = temp.valid();
        if (sample_ok) {
            bad_streak_      = 0;
            last_good_temp_  = temp.celsius;
            have_temp_       = true;
        } else {
            faults |= FAULT_SENSOR_INVALID;
            if (bad_streak_ < 0xFF) ++bad_streak_;
        }
        // A short dropout rides on the last good sample; a persistent one
        // means we no longer know the temperature at all.
        const bool trustworthy =
            sample_ok || (have_temp_ && bad_streak_ < cfg_.sensor_debounce);

        // ── 2. Untrustworthy input → failsafe posture ───────────────────────
        // Without a believable temperature the envelope cannot be verified, so
        // neither the MPC's advisory nor a reactive response is justified. The
        // posture is CONDITIONED on the last good sample, never static: a
        // static posture was shown (harness scenario S2) to actively cool a
        // cold-marginal pack during the blackout and cause the breach itself.
        if (!trustworthy) {
            faults |= FAULT_SENSOR_PERSIST;
            out.vent_cmd = slewTo(failsafePosture());
            out.verdict  = Verdict::OVERRIDE_FAILSAFE;
            out.faults   = faults;
            return out;
        }
        const float t_c = sample_ok ? temp.celsius : last_good_temp_;

        // ── 3. Reactive envelope override (full authority, MPC ignored) ─────
        if (t_c <= cfg_.t_min_hard_c) {
            faults |= FAULT_ENVELOPE_COLD;
            out.vent_cmd = slewTo(0.0f);            // close: retain heat
            out.verdict  = Verdict::OVERRIDE_REACTIVE;
            out.faults   = faults;
            return out;
        }
        if (t_c >= cfg_.t_max_hard_c) {
            faults |= FAULT_ENVELOPE_HOT;
            out.vent_cmd = slewTo(1.0f);            // open: dump heat
            out.verdict  = Verdict::OVERRIDE_REACTIVE;
            out.faults   = faults;
            return out;
        }

        // ── 4. Advisory acceptance path ─────────────────────────────────────
        float cmd = mpc.vent_norm;
        const bool stale =
            !mpc.ever_received || (now_ms - mpc.timestamp_ms) > cfg_.mpc_timeout_ms;
        if (stale) {
            faults |= FAULT_MPC_STALE;
            out.vent_cmd = slewTo(failsafePosture());
            out.verdict  = Verdict::OVERRIDE_FAILSAFE;
            out.faults   = faults;
            return out;
        }

        Verdict verdict = Verdict::PASS;
        if (cmd < 0.0f || cmd > 1.0f) {
            faults |= FAULT_CMD_OUT_OF_RANGE;
            cmd     = (cmd < 0.0f) ? 0.0f : 1.0f;
            verdict = Verdict::CLAMPED;
        }

        const float shaped = slewTo(cmd);
        if (shaped != cmd) faults |= FAULT_CMD_SLEW_LIMITED;

        out.vent_cmd = shaped;
        out.verdict  = verdict;
        out.faults   = faults;
        return out;
    }

private:
    /// Blind-mode posture, conditioned on the last trustworthy temperature.
    /// Cold half of the band → close (retain heat); hot region → open (dump
    /// heat); otherwise the neutral cruise posture. If no sample was ever
    /// good, fall back to the configured neutral posture.
    float failsafePosture() const {
        if (!have_temp_) return cfg_.vent_failsafe;
        if (last_good_temp_ < cfg_.failsafe_cold_below_c) return 0.0f;
        if (last_good_temp_ > cfg_.failsafe_hot_above_c)  return 1.0f;
        return cfg_.vent_failsafe;
    }

    /// Rate-limit command motion so a single cycle can never slam the servo
    /// full-travel (one bad frame ≠ one mechanical shock). First call jumps
    /// directly to the target (no prior position to slew from).
    float slewTo(float target) {
        if (last_cmd_ < 0.0f) { last_cmd_ = target; return target; }
        const float lo = last_cmd_ - cfg_.slew_per_step;
        const float hi = last_cmd_ + cfg_.slew_per_step;
        const float v  = (target < lo) ? lo : (target > hi ? hi : target);
        last_cmd_ = v;
        return v;
    }

    Config  cfg_;
    float   last_cmd_       = -1.0f;  // <0 ⇒ no command issued yet
    float   last_good_temp_ = 0.0f;
    bool    have_temp_      = false;
    uint8_t bad_streak_     = 0;
    uint8_t stall_streak_   = 0;      // consecutive cycles command ≠ feedback
};

} // namespace helios::safety

#endif // HELIOS_SAFETY_MONITOR_H
