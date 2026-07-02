/**
 * @file    main.cpp
 * @brief   Helios UAV — Flight Controller Firmware Core
 *
 * Bare-metal architecture stub for the STM32 flight controller.
 * Simulates a deterministic 100 Hz real-time control loop with:
 *   - Sensor acquisition pipeline  (IMU · Barometer · Pitot)
 *   - Control law processing layer (PID stability · thermal MPC bridge)
 *   - Actuator output stage        (ESC PWM · hatch servo PWM)
 *   - 1 Hz telemetry heartbeat     (jitter · altitude · hatch state)
 *
 * Build (host simulation):
 *   g++ -std=c++17 -O2 -Wall -o helios_fw main.cpp
 *
 * @author  Helios Avionics Firmware Core v2.0
 * @version 2.0.0
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

// ── Compile-time configuration ─────────────────────────────────────────────

static constexpr uint32_t LOOP_RATE_HZ         = 100;
static constexpr uint32_t LOOP_PERIOD_US        = 1'000'000 / LOOP_RATE_HZ;   // 10 000 µs
static constexpr uint32_t TELEMETRY_INTERVAL    = LOOP_RATE_HZ;               // every 100 ticks = 1 s
static constexpr uint32_t JITTER_WARN_US        = 500;                        // warn if |jitter| > 0.5 ms
static constexpr float    MISSION_DURATION_S    = 20.0f;                      // sim ends after 20 s

// ── Physical / hardware constants ──────────────────────────────────────────

static constexpr float    IMU_NOISE_AMP         = 0.15f;    // deg — simulated noise amplitude
static constexpr float    BARO_NOISE_AMP        = 0.8f;     // m
static constexpr float    PITOT_NOISE_AMP       = 0.05f;    // m/s

// PWM output range matching a standard 50 Hz RC servo/ESC bus
static constexpr uint16_t PWM_MIN_US            = 1000;     // µs — full reverse / fully closed
static constexpr uint16_t PWM_MID_US            = 1500;     // µs — neutral
static constexpr uint16_t PWM_MAX_US            = 2000;     // µs — full throttle / fully open


// ══════════════════════════════════════════════════════════════════════════════
// Data structures
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Raw sensor bus data — populated once per loop tick by readSensors().
 *        All values are SI-unit floating-point after conversion from raw ADC/bus words.
 */
struct SensorFrame {
    // IMU (gyro-integrated angles, degrees)
    float pitch_deg  = 0.0f;
    float roll_deg   = 0.0f;

    // Barometric altimeter (metres above launch elevation)
    float altitude_m = 0.0f;

    // Pitot-static differential pressure → calibrated airspeed (m/s)
    float airspeed_ms = 0.0f;
};

/**
 * @brief PID controller state — one instance per axis / loop.
 *        Integration and derivative terms persist across ticks.
 *
 * Hardened over a textbook PID with two fixes that matter the moment this
 * leaves the benign simulation and meets a real actuator:
 *
 *   1. Derivative on MEASUREMENT (not error) — removes "derivative kick".
 *   2. Conditional integration + integral clamp — removes integral windup.
 *
 * See compute() for the derivation of each.
 */
struct PidState {
    // Gains. Kept as the first three members so existing designated-
    // initializer construction { .kp=…, .ki=…, .kd=… } remains valid.
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;

    // ── Output saturation limits (actuator authority, in PID output units) ──
    // The controller MUST know where its own output saturates; otherwise the
    // integrator keeps winding while the actuator is already hard against its
    // stop. Downstream, elevator/aileron demand = output × 0.1, so ±10 here
    // corresponds to the ±1.0 normalised control-surface travel limit.
    float out_min = -10.0f, out_max = 10.0f;

    // ── Persistent state ────────────────────────────────────────────────
    float integral      = 0.0f;
    float prev_measured = 0.0f;   // for derivative-on-measurement
    float output        = 0.0f;
    bool  initialised   = false;  // seeds prev_measured on the first call

    /**
     * @brief Compute one PID step (anti-windup + derivative-on-measurement).
     * @param setpoint  Desired value
     * @param measured  Current measured value
     * @param dt        Time step [s]
     *
     * FIX 1 — Derivative on measurement, not error.
     *   d_term = -kd · (measured − prev_measured) / dt        (note the sign)
     *   A textbook D term differentiates the ERROR, so a step change in the
     *   SETPOINT — e.g. the pitch target stepping 4°→1.5°→−3° at the flight-
     *   phase boundaries — injects a one-tick impulse (the "derivative kick")
     *   that slams the servo. Differentiating the measurement instead makes
     *   the D term blind to setpoint steps while still damping real motion.
     *   (Mid-phase, with a constant setpoint, this is algebraically identical
     *   to the old form, so nominal behaviour is unchanged.)
     *
     * FIX 2 — Anti-windup (conditional integration + hard clamp).
     *   While the output is saturated (servo pinned), a naive integrator keeps
     *   accumulating and must later "unwind", producing a large overshoot on
     *   recovery. Here the integrator is (a) hard-clamped so its term ki·I can
     *   never exceed the output range on its own, and (b) frozen whenever the
     *   output is saturated AND the error would drive it further into the rail.
     */
    void compute(float setpoint, float measured, float dt) {
        const float error = setpoint - measured;

        // Seed derivative memory on first call to avoid a spurious kick from
        // prev_measured == 0.
        if (!initialised) { prev_measured = measured; initialised = true; }

        // ── FIX 1: derivative on measurement ───────────────────────────
        const float d_term = -kd * (measured - prev_measured) / dt;
        prev_measured = measured;

        // ── Trial integral, hard-clamped so ki·I stays within output range ─
        float integral_trial = integral + error * dt;
        if (ki > 0.0f) {
            const float i_max = out_max / ki;
            const float i_min = out_min / ki;
            if (integral_trial > i_max) integral_trial = i_max;
            if (integral_trial < i_min) integral_trial = i_min;
        }

        // ── Unsaturated output with the trial integral ─────────────────
        const float u = kp * error + ki * integral_trial + d_term;

        // ── Saturate to actuator authority ─────────────────────────────
        float u_sat = u;
        if (u_sat > out_max) u_sat = out_max;
        if (u_sat < out_min) u_sat = out_min;

        // ── FIX 2: conditional integration ─────────────────────────────
        // Commit the trial integral only when NOT winding deeper into a
        // saturated rail. If the output is pinned high and the error is still
        // positive (or pinned low with negative error), freeze the integrator;
        // otherwise accept it (integration is helping the output recover).
        const bool winding_up   = (u > out_max) && (error > 0.0f);
        const bool winding_down = (u < out_min) && (error < 0.0f);
        if (!winding_up && !winding_down) {
            integral = integral_trial;
        }
        // else: hold `integral` at its previous value this tick.

        output = u_sat;
    }
};

/**
 * @brief Actuator demand bus — produced by processControlLaws(),
 *        consumed by outputActuators().
 */
struct ActuatorDemand {
    float throttle_norm  = 0.0f;   // 0.0 – 1.0  → ESC PWM
    float hatch_norm     = 0.0f;   // 0.0 – 1.0  → thermal hatch servo (from MPC bridge)
    float elevator_norm  = 0.0f;   // -1.0 – 1.0 → elevator servo
    float aileron_norm   = 0.0f;   // -1.0 – 1.0 → aileron servo
};

/**
 * @brief Telemetry snapshot reported at 1 Hz to the ground link / console.
 */
struct TelemetryFrame {
    uint32_t tick             = 0;

    // ── Real-time timing metrics ───────────────────────────────────────
    // Three DISTINCT quantities, deliberately not merged, because each
    // answers a different question about loop health:
    //
    //   compute_us       (c_k)             — "did the work fit the budget?"
    //   slip_us          (Φ_k = s_k − d_k) — "did the tick land on the grid?"
    //   period_jitter_us (Φ_k − Φ_{k−1})   — "was the effective Δt uniform?"
    //
    // Only the latter two carry control-stability meaning:
    //   • slip_us          is dead time  → phase-margin loss
    //   • period_jitter_us is Δt error   → distorts the PID I/D discretisation
    // compute_us is mere headroom and must NEVER gate stability.
    float    compute_us       = 0.0f;   // c_k             — execution time only
    float    slip_us          = 0.0f;   // Φ_k = s_k − d_k — schedule slip vs grid
    float    period_jitter_us = 0.0f;   // Φ_k − Φ_{k−1}   — first difference of slip
    uint32_t missed_deadlines = 0;      // cumulative periods shed by recovery logic

    // ── Flight state snapshot ──────────────────────────────────────────
    float    altitude_m  = 0.0f;
    float    airspeed_ms = 0.0f;
    float    pitch_deg   = 0.0f;
    float    hatch_pos   = 0.0f;   // 0.0 closed → 1.0 fully open
    float    throttle    = 0.0f;
    bool     jitter_warn = false;  // set on real timing error (slip / missed), not headroom
};


// ══════════════════════════════════════════════════════════════════════════════
// Simulation state (global, mimicking MCU SRAM layout)
// ══════════════════════════════════════════════════════════════════════════════

static SensorFrame    g_sensors;
static ActuatorDemand g_actuators;
static TelemetryFrame g_telemetry;

// PID instances — pitch stabilisation and roll stabilisation
static PidState g_pid_pitch { .kp=1.2f, .ki=0.05f, .kd=0.3f };
static PidState g_pid_roll  { .kp=1.0f, .ki=0.04f, .kd=0.25f };

// Simulated mission clock (seconds since arm)
static float g_mission_time_s = 0.0f;


// ══════════════════════════════════════════════════════════════════════════════
// Utility helpers
// ══════════════════════════════════════════════════════════════════════════════

using Clock     = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Micros    = std::chrono::microseconds;

/** Convert a normalised demand [0,1] or [-1,1] to a PWM pulse width [µs]. */
static uint16_t normToPwm(float norm, float norm_min = 0.0f, float norm_max = 1.0f) {
    const float clamped = (norm < norm_min) ? norm_min : (norm > norm_max ? norm_max : norm);
    const float t = (clamped - norm_min) / (norm_max - norm_min);
    return static_cast<uint16_t>(PWM_MIN_US + t * (PWM_MAX_US - PWM_MIN_US));
}

/**
 * Lightweight pseudo-random noise source (no stdlib rand dependency).
 * Returns a value in [-amplitude, +amplitude].
 */
static float simNoise(float amplitude, uint32_t seed_mix) {
    static uint32_t s = 0xDEADBEEFu;
    s ^= seed_mix;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;   // xorshift32
    const float unit = static_cast<float>(s & 0xFFFF) / 65535.0f;  // [0,1]
    return (unit * 2.0f - 1.0f) * amplitude;
}

/** Synthetic altitude profile matching the Python mission: climb → cruise → descent. */
static float missionAltitude(float t_s) {
    if (t_s <= 5.0f)
        return (t_s / 5.0f) * 2500.0f;
    if (t_s <= 15.0f)
        return 2500.0f + ((t_s - 5.0f) / 10.0f) * 1000.0f;   // ease up to 3500 m
    return 3500.0f * (1.0f - (t_s - 15.0f) / 5.0f);           // glide to ground
}

/** Synthetic airspeed: high on climb, cruise settles, drops on descent. */
static float missionAirspeed(float t_s) {
    if (t_s <= 5.0f)  return 10.0f + (t_s / 5.0f) * 5.0f;
    if (t_s <= 15.0f) return 15.0f;
    return 15.0f * (1.0f - (t_s - 15.0f) / 5.0f);
}

/** Synthetic hatch demand from MPC bridge (placeholder ramp). */
static float mpcHatchDemand(float t_s) {
    // During climb: closed. Cruise onset: MPC opens to 0.2. Descent: closes again.
    if (t_s < 5.0f)  return 0.0f;
    if (t_s < 15.0f) return 0.2f;
    return 0.0f;
}


// ══════════════════════════════════════════════════════════════════════════════
// Pipeline stage 1 — Sensor acquisition
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief   Reads and converts all sensor buses.
 *
 * On hardware this function drives:
 *   - SPI/I2C burst read of IMU FIFO
 *   - ADC sample of differential pressure transducer (Pitot)
 *   - I2C read of barometric sensor (BMP388 / MS5611)
 *
 * Here we inject the synthetic mission profile plus band-limited noise
 * to produce a statistically representative stimulus for the control laws.
 */
static void readSensors() {
    const uint32_t t_ticks = static_cast<uint32_t>(g_mission_time_s * LOOP_RATE_HZ);

    // ── IMU (MPU-6000 / ICM-42688 SPI) ────────────────────────────────
    // Simulated: aircraft in coordinated flight, slight pitch-up during climb
    const float pitch_trim  = (g_mission_time_s < 5.0f)  ?  4.0f :
                              (g_mission_time_s < 15.0f)  ?  1.5f : -3.0f;
    g_sensors.pitch_deg  = pitch_trim + simNoise(IMU_NOISE_AMP, t_ticks * 0x9E37u);
    g_sensors.roll_deg   =              simNoise(IMU_NOISE_AMP, t_ticks * 0xC4F3u);

    // ── Barometer (BMP388 I2C) ─────────────────────────────────────────
    g_sensors.altitude_m = missionAltitude(g_mission_time_s)
                         + simNoise(BARO_NOISE_AMP, t_ticks * 0xA1B2u);

    // ── Pitot-static (ADC differential) ───────────────────────────────
    g_sensors.airspeed_ms = missionAirspeed(g_mission_time_s)
                          + simNoise(PITOT_NOISE_AMP, t_ticks * 0x5F3Cu);
    if (g_sensors.airspeed_ms < 0.0f) g_sensors.airspeed_ms = 0.0f;
}


// ══════════════════════════════════════════════════════════════════════════════
// Pipeline stage 2 — Control law processing
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief   Runs all inner-loop control laws and MPC bridge logic.
 *
 * Execution order on hardware:
 *   1. Attitude PID  — pitch / roll → elevator / aileron demands
 *   2. Throttle law  — energy/altitude hold or pilot command mix
 *   3. Thermal MPC bridge — receives hatch position from Raspberry Pi
 *      over UART2 at 2 s cadence; holds last command between updates
 *
 * All PID outputs are normalised to [-1, 1] or [0, 1] before passing
 * to outputActuators() to decouple control math from PWM calibration.
 */
static void processControlLaws() {
    const float dt = 1.0f / static_cast<float>(LOOP_RATE_HZ);

    // ── Stability PID — pitch axis ─────────────────────────────────────
    const float pitch_setpoint = (g_mission_time_s < 5.0f) ? 4.0f :
                                 (g_mission_time_s < 15.0f)? 1.5f : -3.0f;
    g_pid_pitch.compute(pitch_setpoint, g_sensors.pitch_deg, dt);
    g_actuators.elevator_norm = g_pid_pitch.output * 0.1f;   // gain-scale to [-1,1]

    // ── Stability PID — roll axis ──────────────────────────────────────
    g_pid_roll.compute(0.0f, g_sensors.roll_deg, dt);
    g_actuators.aileron_norm = g_pid_roll.output * 0.1f;

    // ── Throttle schedule (energy management placeholder) ──────────────
    // Phase-based open-loop schedule; will be replaced by airspeed hold PID
    if      (g_mission_time_s < 5.0f)  g_actuators.throttle_norm = 0.75f;  // climb
    else if (g_mission_time_s < 15.0f) g_actuators.throttle_norm = 0.30f;  // cruise
    else                                g_actuators.throttle_norm = 0.0f;   // glide

    // ── Thermal hatch MPC bridge ───────────────────────────────────────
    // On hardware: last value received from Raspberry Pi MPC via UART2.
    // Here: synthetic demand replays the Python MPC output profile.
    g_actuators.hatch_norm = mpcHatchDemand(g_mission_time_s);
}


// ══════════════════════════════════════════════════════════════════════════════
// Pipeline stage 3 — Actuator output
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief   Converts normalised demands to PWM pulse widths and writes outputs.
 *
 * On hardware this drives STM32 TIM1/TIM8 compare registers directly.
 * Each channel is rate-limited in hardware by the timer period (50 Hz for
 * servos, up to 400 Hz for digital ESCs).
 *
 * Channel mapping (hardware connector labels):
 *   CH1 — Motor ESC    (throttle_norm  → 1000–2000 µs)
 *   CH2 — Elevator     (elevator_norm  → 1000–2000 µs, centred at 1500)
 *   CH3 — Ailerons     (aileron_norm   → 1000–2000 µs, centred at 1500)
 *   CH4 — Hatch servo  (hatch_norm     → 1000–2000 µs)
 */
static void outputActuators() {
    // Compute PWM values (not written to hardware registers in simulation)
    const uint16_t pwm_motor    = normToPwm(g_actuators.throttle_norm);
    const uint16_t pwm_elevator = normToPwm(g_actuators.elevator_norm, -1.0f, 1.0f);
    const uint16_t pwm_aileron  = normToPwm(g_actuators.aileron_norm,  -1.0f, 1.0f);
    const uint16_t pwm_hatch    = normToPwm(g_actuators.hatch_norm);

    // On hardware: TIM1->CCR1 = pwm_motor; TIM1->CCR2 = pwm_elevator; etc.
    (void)pwm_motor;    // suppress unused-variable warnings in simulation build
    (void)pwm_elevator;
    (void)pwm_aileron;
    (void)pwm_hatch;
}


// ══════════════════════════════════════════════════════════════════════════════
// 1 Hz telemetry heartbeat
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief   Formats and prints one telemetry frame to the console / UART GCS link.
 *
 * In flight this function serialises g_telemetry into the binary MAVLink-like
 * frame defined in serial_bridge.py and fires it over the LoRa radio UART.
 * In simulation it emits a human-readable ASCII digest for validation.
 */
static void emitTelemetry() {
    const char* jitter_flag = g_telemetry.jitter_warn ? " !! JITTER" : "";
    const char* phase =
        (g_mission_time_s < 5.0f)  ? "CLIMB  " :
        (g_mission_time_s < 15.0f) ? "CRUISE " : "DESCENT";

    std::printf(
        "  [%5.1fs | %s | tick %05u]  "
        "Alt: %7.1f m  |  IAS: %5.2f m/s  |  Pitch: %+6.2f°  |  "
        "Hatch: %.1f  |  Thr: %4.1f%%  |  "
        "Comp: %5.1fµs  |  Slip: %+6.1fµs  |  PJit: %+6.1fµs  |  Miss: %u%s\n",
        g_mission_time_s,
        phase,
        g_telemetry.tick,
        g_telemetry.altitude_m,
        g_telemetry.airspeed_ms,
        g_telemetry.pitch_deg,
        g_telemetry.hatch_pos,
        g_telemetry.throttle * 100.0f,
        g_telemetry.compute_us,
        g_telemetry.slip_us,
        g_telemetry.period_jitter_us,
        g_telemetry.missed_deadlines,
        jitter_flag
    );
}


// ══════════════════════════════════════════════════════════════════════════════
// Entry point
// ══════════════════════════════════════════════════════════════════════════════

int main() {

    std::printf("\n");
    std::printf("  ╔══════════════════════════════════════════════════════════╗\n");
    std::printf("  ║         Helios Avionics Firmware Core v2.0              ║\n");
    std::printf("  ║   Phase 2 — 100 Hz Real-Time Flight Controller Loop     ║\n");
    std::printf("  ╚══════════════════════════════════════════════════════════╝\n");
    std::printf("  Loop period target : %u µs (%u Hz)\n", LOOP_PERIOD_US, LOOP_RATE_HZ);
    std::printf("  Telemetry cadence  : 1 Hz (every %u ticks)\n", TELEMETRY_INTERVAL);
    std::printf("  Mission duration   : %.0f s\n\n", MISSION_DURATION_S);
    std::printf("  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────\n");

    uint32_t tick = 0;

    // ── Deadline scheduler state ───────────────────────────────────────
    // The loop is anchored to a FIXED time grid:  d_k = d_0 + k·T,
    // where T = LOOP_PERIOD_US. `tick_deadline` always holds d_k — the
    // instant at which tick k SHOULD begin. Timing error is recovered by
    // comparing the ACTUAL start s_k against d_k, NOT by measuring how long
    // the work took (compute time is blind to the grid — see below).
    TimePoint tick_deadline = Clock::now();   // d_0 — ideal start of tick 0
    float     prev_slip_us  = 0.0f;           // Φ_{k−1} — kept for the first difference

    while (g_mission_time_s <= MISSION_DURATION_S) {

        // ── Timing capture: schedule slip  Φ_k = s_k − d_k ─────────────
        // s_k = actual start of this tick;  d_k = tick_deadline.
        // Φ_k is the loop's PHASE ERROR against the ideal grid — i.e. the
        // dead time injected into the control law this cycle. Φ_k > 0 means
        // the tick fired late (a transport delay → phase-margin loss);
        // Φ_k ≈ 0 is healthy. This is the metric that governs stability,
        // and it is captured BEFORE the pipeline runs.
        const TimePoint tick_start = Clock::now();
        const float slip_us = static_cast<float>(
            std::chrono::duration_cast<Micros>(tick_start - tick_deadline).count()
        );

        // ── Period jitter = Φ_k − Φ_{k−1}  (first difference of the slip) ─
        // The effective sample interval is  Δ_k = s_k − s_{k−1}
        //                                       = T + (Φ_k − Φ_{k−1}).
        // So this first difference IS the error in Δt that the PID silently
        // absorbs: it multiplies the integral term (err·Δt) and divides the
        // derivative term (Δerr/Δt), both of which are coded assuming Δ_k = T.
        const float period_jitter_us = slip_us - prev_slip_us;
        prev_slip_us = slip_us;

        // ── Pipeline ───────────────────────────────────────────────────
        readSensors();
        processControlLaws();
        outputActuators();

        // ── Compute time  c_k  (budget headroom — NOT a timing error) ──
        // Retained only to answer "did the work fit inside the 10 ms slot?"
        // It contains no phase information; the OLD jitter metric was exactly
        // this value minus T, which is why it read a near-constant −9995 µs
        // and tripped the warning on every tick.
        const TimePoint tick_end = Clock::now();
        const float compute_us = static_cast<float>(
            std::chrono::duration_cast<Micros>(tick_end - tick_start).count()
        );

        // ── Advance to the next grid deadline:  d_{k+1} = d_k + T ───────
        tick_deadline += Micros(LOOP_PERIOD_US);

        // ── Deadline-miss recovery (backlog shedding) ──────────────────
        // If we overran so badly that d_{k+1} is ALREADY in the past,
        // sleep_until() would return instantly and the loop would fire a
        // burst of catch-up ticks with Δ_k ≪ T — collapsing the effective
        // sample interval and spiking the derivative term (Δerr/Δt → ∞).
        // Instead we SNAP FORWARD by whole periods until the deadline is
        // once again in the future. Snapping by integer T preserves phase
        // alignment to the original grid (we drop a slot rather than reset
        // it); the discarded cycles are counted so the loss stays visible.
        uint32_t missed = 0;
        while (tick_deadline < tick_end) {
            tick_deadline += Micros(LOOP_PERIOD_US);
            ++missed;
        }

        // ── Populate telemetry struct ──────────────────────────────────
        g_telemetry.tick             = tick;
        g_telemetry.compute_us       = compute_us;
        g_telemetry.slip_us          = slip_us;
        g_telemetry.period_jitter_us = period_jitter_us;
        g_telemetry.missed_deadlines += missed;          // cumulative over the mission
        g_telemetry.altitude_m       = g_sensors.altitude_m;
        g_telemetry.airspeed_ms      = g_sensors.airspeed_ms;
        g_telemetry.pitch_deg        = g_sensors.pitch_deg;
        g_telemetry.hatch_pos        = g_actuators.hatch_norm;
        g_telemetry.throttle         = g_actuators.throttle_norm;

        // Warn on REAL timing error: excessive phase slip OR a dropped
        // deadline this cycle — never on spare compute headroom.
        g_telemetry.jitter_warn =
            (std::abs(slip_us) > static_cast<float>(JITTER_WARN_US)) || (missed > 0);

        // ── 1 Hz telemetry heartbeat ───────────────────────────────────
        if (tick % TELEMETRY_INTERVAL == 0) {
            emitTelemetry();
        }

        // ── Deterministic non-blocking wait ───────────────────────────
        // Sleep until the next grid deadline. On the host this defers to the
        // OS scheduler, so residual error resurfaces as slip_us on the NEXT
        // cycle. On bare metal this is replaced by a SysTick IRQ or TIM6
        // overflow, where slip collapses to hardware-timer precision.
        std::this_thread::sleep_until(tick_deadline);

        // ── Advance clocks ─────────────────────────────────────────────
        // Wall time advanced by (1 + missed) periods: the tick just run plus
        // any deliberately shed. Keeping the sim clock aligned to real
        // elapsed time matters once a hardware RTC drives the flight profile.
        g_mission_time_s += static_cast<float>(1 + missed) / static_cast<float>(LOOP_RATE_HZ);
        ++tick;
    }

    // ── End-of-mission report ──────────────────────────────────────────
    std::printf("  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────\n");
    std::printf("\n  Mission complete. %u ticks executed at target %u Hz.\n", tick, LOOP_RATE_HZ);

    // Real-time integrity summary. missed_deadlines is the count of grid slots
    // shed by the snap-forward recovery — the headline figure for whether the
    // loop held hard real-time. On healthy host runs this must read 0.
    std::printf("  Real-time integrity : %u deadline(s) missed / shed  (%s)\n",
                g_telemetry.missed_deadlines,
                (g_telemetry.missed_deadlines == 0) ? "HARD REAL-TIME HELD"
                                                    : "DEADLINES SHED — INVESTIGATE");
    std::printf("  Helios Avionics Firmware Core v2.0 — halted.\n\n");

    return 0;
}
