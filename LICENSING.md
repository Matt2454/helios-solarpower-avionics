# Licensing — Helios Avionics Middleware

Helios uses an **open-core / dual-license** model. This document is the
**authoritative mapping** of every source file to its license bucket. Where a
file's SPDX header and this document ever disagree, treat it as a bug and open
an issue — they must stay in sync.

> **Status: plumbing complete, legal wording pending.**
> The Apache-2.0 shell (`LICENSE`) is final and standard. The proprietary core
> terms (`LICENSE-CORE`) are a **placeholder to be finalized by IP counsel**,
> and the copyright holder name **"Helios Avionics"** is a placeholder for the
> registered legal entity. Both are marked with `TODO` in-file.

## The two buckets

| Bucket | License | File | SPDX identifier | Distributed as |
|--------|---------|------|-----------------|----------------|
| **Open shell** — adoption driver; the "socket," not the "brain" | Apache-2.0 | [`LICENSE`](LICENSE) | `Apache-2.0` | Source |
| **Proprietary core** — the licensable IP | Commercial | [`LICENSE-CORE`](LICENSE-CORE) | `LicenseRef-Helios-Commercial` | Object/binary only |

## Component-by-component mapping

### Open Components — Apache-2.0
Permissively licensed so OEMs can read, port, and integrate freely. Contains no
protected algorithm.

| File | Role |
|------|------|
| `main.cpp` | SITL / host reference port of the flight-loop harness, incl. the (non-novel) attitude PID reference controller |
| `weather_oracle.py` | ISA atmospheric model — standard, published physics |

### Core Components — Proprietary (`LICENSE-CORE`)
The moat. Source is confidential; shipped to OEMs as a precompiled library only.

| File | Role |
|------|------|
| `mpc_core.py` | Model Predictive Control engine — the pre-emptive thermal-vent algorithm |
| `thermal_simulator.py` | Reference thermal plant model used by the MPC rollout |
| `flight_loop_sim.py` | Internal integration/validation harness that exercises the proprietary core |

## SPDX conventions

Every source file begins with a two-line SPDX header:

```
# SPDX-FileCopyrightText: 2026 Helios Avionics
# SPDX-License-Identifier: Apache-2.0
```

Proprietary files additionally carry a `LicenseRef-Helios-Commercial` identifier
and a confidentiality banner. SPDX is machine-readable, so license scanners
(e.g. `reuse`, FOSSA, ScanCode) can verify the repository automatically in CI.

## Third-party / ecosystem compatibility

Helios currently has **no third-party code dependencies** (pure C++ stdlib and
Python stdlib), which is what makes this relicensing clean. Target ecosystems
are compatible with a proprietary product:

| Ecosystem | License | Compatible with our model? |
|-----------|---------|----------------------------|
| PX4 flight stack | BSD-3-Clause | Yes (permissive) |
| MAVLink | MIT-style | Yes (permissive) |

**Rule:** any future dependency must be reviewed before adoption. Reject GPL/AGPL
and other strong-copyleft licenses in code that links against the Core
Components.

## Historical note

This project was previously licensed under **GPLv3**, which is incompatible with
proprietary OEM integration (strong copyleft would force licensees to open-source
their derivative firmware). The GPLv3 `LICENSE` was replaced as part of the
open-core pivot. As the repository had no external contributors or third-party
GPL dependencies, the copyright holder was free to relicense.

## Open TODOs before external distribution

- [ ] IP counsel to finalize `LICENSE-CORE` commercial terms.
- [ ] Replace placeholder holder name "Helios Avionics" with the registered entity.
- [ ] Replace placeholder contact `licensing@helios-avionics.example`.
- [ ] Add a license-scan step (e.g. `reuse lint`) to CI.
