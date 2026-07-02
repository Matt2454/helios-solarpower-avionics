<!--
SPDX-FileCopyrightText: 2026 Helios Avionics
SPDX-License-Identifier: Apache-2.0
Open Component — Helios Avionics Middleware Integration Manual (see LICENSING.md).
-->

# 7. Provenance, Trust & Integrator Assurance

> **Purpose.** This section exists to answer, up front and in writing, the
> questions an integrator's engineering and legal teams *should* ask before
> placing third-party software anywhere near a flight-safety path. Nothing here
> asks you to take our word for it — every assurance is backed by an artifact,
> a mechanism, or something you can run yourself.

## 7.1 Authority model — Helios is advisory by default

Helios does **not** hold final authority over the aircraft. The middleware
emits *recommendations* — a thermal vent command and a safety-state assessment —
which your certified autopilot is free to accept, rate-limit, override, or
ignore. Final actuation authority always remains with your flight controller.

| Property | Guarantee |
|---|---|
| Flight authority | Retained by the integrator's autopilot at all times |
| Helios role | Advisory subsystem (thermal + safety recommendations) |
| Removal behaviour | Aircraft remains flyable with Helios unpowered or disconnected |
| Command validation | Every Helios output is range-clamped and rate-limited at the integration boundary before it can reach an actuator |

**Why this matters to you:** because Helios cannot command an unsafe state on
its own, integrating it does not transfer airworthiness responsibility to an
external party. It bounds your liability exposure to that of any other advisory
input to your control law.

## 7.2 Airworthiness status — stated plainly

Helios is **not certified** to DO-178C, ED-12C, or any equivalent airworthiness
standard, and is provided as an advisory subsystem only. The integrator retains
sole responsibility for the safety, testing, and certification of any system
into which Helios is integrated. We will not represent otherwise, and we will
support your certification evidence-gathering (see §7.5) rather than overstate
our maturity.

## 7.3 IP provenance & indemnity posture

A clean, auditable ownership trail is maintained so your diligence team can
verify title:

- **No third-party code dependencies.** Helios is built on the C++ and Python
  standard libraries only. There is no copyleft (GPL/AGPL) anywhere in the
  dependency graph — verify with the machine-readable SPDX headers on every
  source file and the license-scan step in CI.
- **Documented licensing split.** `LICENSING.md` is the authoritative,
  file-by-file map of Apache-2.0 (open) vs. proprietary (core) components,
  including the project's full relicensing history.
- **Disclosed development method.** Portions were prototyped with AI-assisted
  tooling; all resulting contributions are owned by Helios Avionics. We disclose
  this rather than conceal it — full provenance is available for diligence.
- **Ecosystem compatibility.** Target stacks (PX4 — BSD-3-Clause; MAVLink —
  MIT-style) are license-compatible with proprietary integration.

Formal IP indemnification is addressed in the commercial agreement, not this
manual; the point here is that the provenance is clean enough to indemnify.

## 7.4 Continuity & long-term support (bus-factor mitigation)

We do not ask you to bet your product line on a single point of failure:

- **Source escrow.** The proprietary Core Components are placed with a
  third-party escrow agent. Defined release conditions — including cessation of
  support or failure to meet agreed maintenance obligations — release the full
  source to you, so you are never stranded with an unsupported binary in your
  flight stack.
- **Scoped, honest support terms.** The commercial agreement defines a support
  scope we can genuinely honour, rather than an SLA we cannot staff. We would
  rather under-promise and meet it.
- **Buildable from escrow.** Escrowed source ships with its build toolchain and
  reproducible-build instructions, so a release is actionable, not just legally
  satisfying.

## 7.5 Verify it yourself — reproducible validation

The strongest assurance is the one you don't have to trust:

- **Self-run validation suite.** Helios ships with its unit tests and a
  simulation/HIL harness. You run them, on your hardware, against *your*
  airframe's thermal and flight parameters.
- **Model-mismatch testing.** Because your battery, enclosure, and airframe
  differ from our reference model, the harness supports injecting parameter
  mismatch, sensor noise, and unmodelled disturbances so you can characterise
  robustness under *your* conditions rather than our idealised ones.
- **Traceable metrics.** Real-time integrity (schedule slip, period jitter,
  missed-deadline count) and thermal-margin telemetry are exposed for capture,
  giving you objective evidence rather than assurances.

## 7.6 Integrator responsibilities (the boundary of our assurance)

To keep the authority model honest, the integrator is responsible for:

1. Enforcing final command arbitration in the certified autopilot.
2. Applying range and rate limits to Helios outputs at the boundary.
3. Defining and testing the failsafe behaviour when Helios is absent or times out.
4. Validating Helios against the integrator's own airframe and mission profile.
5. Meeting all airworthiness and regulatory obligations for the end system.

---

*This section is a trust framework, not a legal instrument. The binding terms —
indemnity, escrow release conditions, support SLAs, and warranty — live in the
executed commercial agreement referenced by `LICENSE-CORE`.*
