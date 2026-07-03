# Helios

Thermal and safety middleware for solar-powered UAVs.

### Executive Summary

Helios is an advisory avionics middleware for long-endurance solar-electric
UAVs. It provides predictive battery thermal management and fail-safe safety
logic as a module that layers onto an existing flight stack (PX4, ArduPilot, or
custom) rather than replacing it.

The technical differentiator is *pre-emptive* thermal control. Where
conventional firmware reacts to a temperature threshold after it is crossed,
Helios runs a receding-horizon Model Predictive Controller that projects battery
temperature along the flight plan and acts before a limit is reached. It is
designed to fail safe: advisory to the host autopilot, degrading to a reactive
baseline if the high-level layer is lost, and explicitly signalling when it can
no longer guarantee the thermal envelope rather than failing silently.

Robustness is validated, not asserted. Across a 10,000-trial Monte-Carlo
campaign with ±15% component tolerance, sensor noise, and unmodelled atmospheric
disturbance, the controller held the battery within its safe band in every trial
— a certified safety margin with a 95% upper confidence bound below 0.03% on the
breach rate. Methodology, results, and limitations are documented in full.

Helios is currently a simulation-validated core (TRL ~4), packaged as licensable
open-core IP. It is not a certified flight system; the near-term objective is a
hardware design-partnership to move the validated core onto a real airframe.

---

Most flight controllers (PX4, ArduPilot, and friends) are good at flying the
aircraft and fairly basic at managing the battery's temperature. Helios is meant
to sit on top of one of those and handle the parts they don't specialize in:
keeping the battery pack inside its safe temperature window with a predictive
controller, and making thermal/safety calls that degrade gracefully instead of
falling over.

## Heads up: this is a simulation

Nothing here has flown. It's a simulation of the plant plus the control logic
that would run against it. I'm putting that up front because "avionics" tends to
imply flight-proven hardware, and this isn't that yet.

What it does have going for it is that the control logic has been beaten on
pretty hard in sim. More on that below.

## What's in here

- `mpc_core.py` — the thermal controller. A receding-horizon MPC that looks
  ahead along the flight plan and vents the battery *before* it hits a limit,
  not after.
- `thermal_simulator.py` — lumped-parameter model of the sealed battery box (the
  "plant" being controlled).
- `weather_oracle.py` — ISA atmosphere model, with a cold-wave stress mode.
- `flight_loop_sim.py` — runs a full 20-minute mission and prints a per-minute log.
- `validation_montecarlo.py` — the robustness harness. The interesting part.
- `main.cpp` — a 100 Hz flight-loop skeleton in C++ (attitude PID, real-time
  timing instrumentation). Host simulation for now, structured to move to an MCU.

## Running it

Python 3.10+, standard library only, no dependencies to install.

```
python flight_loop_sim.py        # full mission log
python validation_montecarlo.py  # robustness sweep + 10k-trial confirmation
```

On Windows, if the box-drawing characters choke the console, put `PYTHONUTF8=1`
in front. The C++ side builds with:

```
g++ -std=c++17 -O2 -Wall -o helios_fw main.cpp && ./helios_fw
```

## The robustness part

The easy way to make a controller look good is to test it against a perfect copy
of the thing it's controlling. It always passes, and it proves nothing. So the
harness does the opposite: the MPC plans using a *nominal* model and a *noisy*
sensor reading, while the simulated battery it's actually controlling runs on
randomized parameters it can't see (±15% on internal resistance, insulation,
vent effectiveness, and thermal mass), plus sensor noise and weather the model
never predicted.

First pass, it failed. About 2% of runs let the battery cross its cold limit with
no warning at all. That's exactly the thing you want to catch in sim and not in
the air.

The fix is constraint tightening (robust MPC): the controller plans to stay a
margin *inside* the real limits, so model error eats into the buffer instead of
the safety limit. A sweep found the smallest buffer that holds up, and 1.5 K now
ships as the default so you get the safe behaviour even if you never configure
anything. With that buffer, across 10,000 randomized aircraft: zero breaches.

The full write-up, including the numbers and the honest limitations, is in
`docs/integration-manual/08-robustness-validation.md`.

## Licensing

Open-core. The interfaces, the harness, and the reference pieces are Apache-2.0;
the MPC and safety logic are proprietary. `LICENSING.md` has the file-by-file
breakdown. The commercial terms in `LICENSE-CORE` are still placeholder text
pending real legal review, so don't treat them as final.

## What's not done

Honestly, the to-do list is longer than what's built:

- No hardware. Everything is host simulation.
- The Raspberry Pi to MCU link the architecture assumes is still a design, not code.
- The MPC is in Python and would need a C/C++ port to run on a real flight MCU.
- The hardware abstraction layer is sketched, not extracted.
- Only the cold-wave mission is validated. Hot-climate and high-discharge cases
  aren't covered yet.

So it's a tested core, not a finished product. If you're reading this to figure
out whether it's real: the control logic and the validation are real, the flight
hardware story isn't there yet.
