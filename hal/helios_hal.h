// SPDX-FileCopyrightText: 2026 Helios Avionics
// SPDX-License-Identifier: Apache-2.0
//
// Open Component of the Helios Avionics Middleware (see LICENSING.md).
// Licensed under the Apache License, Version 2.0 (see LICENSE).
//
// helios_hal.h — Hardware Abstraction Layer
//
// The Helios core (thermal MPC + safety state machine) talks to the physical
// world ONLY through the pure-virtual interfaces declared here. It never
// includes a driver, touches a register, or names a specific sensor part. To
// port Helios to new hardware you implement these interfaces; the algorithm is
// unchanged. The simulation ("SITL") build is just one more implementation.
//
// Embedded contract:
//   * No exceptions and no dynamic allocation across this boundary — status is
//     reported by return value, never thrown.
//   * All methods are non-blocking and safe to call from the control loop.
//   * Units and ranges are fixed by this header and must not vary by platform.

#ifndef HELIOS_HAL_H
#define HELIOS_HAL_H

#include <cstdint>

namespace helios::hal {

// ─────────────────────────────────────────────────────────────────────────────
// Temperature sensor
// ─────────────────────────────────────────────────────────────────────────────

/// Health of a single sensor sample. The core must treat anything other than
/// OK as "do not trust this value" and fall back to its safe policy.
enum class SensorStatus : uint8_t {
    OK = 0,        ///< Fresh, in-range sample.
    STALE,         ///< No new sample since the last read (bus slow / dropped).
    OUT_OF_RANGE,  ///< Sample outside the physically plausible window.
    FAULT,         ///< Sensor unreachable / hardware fault.
};

/// One battery-pack temperature measurement.
struct TemperatureReading {
    float        celsius      = 0.0f;                 ///< Pack temperature [°C].
    SensorStatus status       = SensorStatus::FAULT;  ///< Sample health.
    uint32_t     timestamp_ms = 0;                    ///< Monotonic sample time.

    /// True only when the sample is fresh and trustworthy.
    bool valid() const { return status == SensorStatus::OK; }
};

/// Reads battery-pack temperature. Backed on hardware by a thermistor / digital
/// sensor (I2C/SPI/ADC); backed in SITL by the ThermalSimulator plant.
class ITemperatureSensor {
public:
    virtual ~ITemperatureSensor() = default;

    /// Return the latest pack temperature with a validity status.
    /// Non-blocking; must not throw. Implementations are responsible for
    /// range-checking and stamping @c timestamp_ms.
    virtual TemperatureReading read() = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Vent actuator
// ─────────────────────────────────────────────────────────────────────────────

/// Result of commanding the vent.
enum class ActuatorStatus : uint8_t {
    OK = 0,     ///< Command accepted.
    SATURATED,  ///< Command was clamped to a travel limit.
    STALLED,    ///< Commanded motion not achieved (servo jam / fault).
    FAULT,      ///< Actuator unreachable / hardware fault.
};

/// Drives the battery-box cooling vent. Position is normalised so the core is
/// independent of servo travel, PWM ranges, and linkage geometry.
///   0.0 = fully closed (retain heat)   1.0 = fully open (max forced convection)
class IVentActuator {
public:
    virtual ~IVentActuator() = default;

    /// Command the vent to @p normalized in [0.0, 1.0]. Implementations MUST
    /// clamp out-of-range input (reporting SATURATED) — the core relies on the
    /// HAL, not itself, to enforce mechanical limits. Non-blocking; must not throw.
    virtual ActuatorStatus setPosition(float normalized) = 0;

    /// Best-known actual position in [0.0, 1.0] from feedback, for closed-loop
    /// use and stall detection. Returns a negative value if the actuator has no
    /// position feedback (open-loop servo).
    virtual float actualPosition() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Bundle handed to the core
// ─────────────────────────────────────────────────────────────────────────────

/// The complete hardware surface the Helios core depends on. Assembled once by
/// the platform layer (real board or SITL) and injected into the controller,
/// which stores only these references and never learns the concrete types.
struct HardwareContext {
    ITemperatureSensor* battery_temp = nullptr;
    IVentActuator*      vent         = nullptr;
};

} // namespace helios::hal

#endif // HELIOS_HAL_H
