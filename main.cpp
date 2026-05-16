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
 */
struct PidState {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    float integral   = 0.0f;
    float prev_error = 0.0f;
    float output     = 0.0f;

    /**
     * @brief Compute one PID step.
     * @param setpoint  Desired value
     * @param measured  Current measured value
     * @param dt        Time step [s]
     */
    void compute(float setpoint, float measured, float dt) {
        const float error  = setpoint - measured;
        integral          += error * dt;
        const float deriv  = (error - prev_error) / dt;
        output             = kp * error + ki * integral + kd * deriv;
        prev_error         = error;
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
    uint32_t tick        = 0;
    float    loop_time_us= 0.0f;   // actual loop execution time
    float    jitter_us   = 0.0f;   // deviation from target period
    float    altitude_m  = 0.0f;
    float    airspeed_ms = 0.0f;
    float    pitch_deg   = 0.0f;
    float    hatch_pos   = 0.0f;   // 0.0 closed → 1.0 fully open
    float    throttle    = 0.0f;
    bool     jitter_warn = false;
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
        "Hatch: %.1f  |  Thr: %4.1f%%  |  Loop: %6.1fµs  |  Jitter: %+6.1fµs%s\n",
        g_mission_time_s,
        phase,
        g_telemetry.tick,
        g_telemetry.altitude_m,
        g_telemetry.airspeed_ms,
        g_telemetry.pitch_deg,
        g_telemetry.hatch_pos,
        g_telemetry.throttle * 100.0f,
        g_telemetry.loop_time_us,
        g_telemetry.jitter_us,
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

    uint32_t tick          = 0;
    float    loop_time_us  = 0.0f;

    // Deadline scheduler: track when each tick SHOULD have started
    TimePoint loop_start   = Clock::now();
    TimePoint tick_deadline = loop_start;

    while (g_mission_time_s <= MISSION_DURATION_S) {

        const TimePoint tick_start = Clock::now();

        // ── Pipeline ───────────────────────────────────────────────────
        readSensors();
        processControlLaws();
        outputActuators();

        // ── Measure actual execution time ──────────────────────────────
        const TimePoint tick_end   = Clock::now();
        loop_time_us = static_cast<float>(
            std::chrono::duration_cast<Micros>(tick_end - tick_start).count()
        );

        // ── Populate telemetry struct ──────────────────────────────────
        const float actual_us = static_cast<float>(
            std::chrono::duration_cast<Micros>(tick_end - tick_start).count()
        );
        const float jitter_us = actual_us - static_cast<float>(LOOP_PERIOD_US);
        tick_deadline += Micros(LOOP_PERIOD_US);

        g_telemetry.tick         = tick;
        g_telemetry.loop_time_us = loop_time_us;
        g_telemetry.jitter_us    = jitter_us;
        g_telemetry.altitude_m   = g_sensors.altitude_m;
        g_telemetry.airspeed_ms  = g_sensors.airspeed_ms;
        g_telemetry.pitch_deg    = g_sensors.pitch_deg;
        g_telemetry.hatch_pos    = g_actuators.hatch_norm;
        g_telemetry.throttle     = g_actuators.throttle_norm;
        g_telemetry.jitter_warn  = std::abs(jitter_us) > static_cast<float>(JITTER_WARN_US);

        // ── 1 Hz telemetry heartbeat ───────────────────────────────────
        if (tick % TELEMETRY_INTERVAL == 0) {
            emitTelemetry();
        }

        // ── Deterministic non-blocking wait ───────────────────────────
        // Sleep until the next scheduled deadline. std::this_thread::sleep_until
        // uses the OS scheduler; residual error is measured as jitter above.
        // On bare metal this is replaced by SysTick IRQ or TIM6 overflow.
        std::this_thread::sleep_until(tick_deadline);

        // ── Advance clocks ─────────────────────────────────────────────
        g_mission_time_s += 1.0f / static_cast<float>(LOOP_RATE_HZ);
        ++tick;
    }

    // ── End-of-mission report ──────────────────────────────────────────
    std::printf("  ─────────────────────────────────────────────────────────────────────────────────────────────────────────────\n");
    std::printf("\n  Mission complete. %u ticks executed at target %u Hz.\n", tick, LOOP_RATE_HZ);
    std::printf("  Helios Avionics Firmware Core v2.0 — halted.\n\n");

    return 0;
}
