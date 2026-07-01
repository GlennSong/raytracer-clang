# Reactive agents + one composable vehicle — plan (ADR-0060)

The shift: agents act on what they can SEE (a vision cone) and decide locally via
a small state machine, steering continuously (boids-style) instead of snapping
along a precomputed A* rail. Every vehicle becomes the same composable object the
player drives, controlled by either player input or a reactive `DriverAgent`.

## Phase 1 — Pedestrians go reactive (headless) 🚧
- A per-agent finite state machine (`Resting / Walking / Avoiding / Waiting`).
- Perception-GATED local avoidance: a walker evades only the neighbours (and the
  player) it can see in its vision cone, biased to a consistent side, so people
  part around each other. Unseen neighbours (behind) are not reacted to. A small
  body-overlap floor keeps them from ever interpenetrating.
- *Tests:* a walker steps around someone it sees ahead; it does NOT react to
  someone only behind it; bodies never overlap; walkers still reach their goals;
  deterministic.

## Phase 2 — Debug widgets (render)
- Per-agent ground footprint (bounding circle/rect) and a forward trajectory
  vector, drawn as instanced decals, toggleable. Makes perception/steering legible.

## Phase 3 — Drivers go reactive
- Driver FSM (`Cruising / Following / Yielding / Stopping / Turning`) fed by the
  vision cone: follow the lane toward a goal, keep distance to the car seen ahead,
  stop for a seen signal, yield at a junction based on what is seen — outputting a
  steering intent, not a snap. Continuous motion removes the leg-snap
  discontinuities and one-step merge overlaps.

## Phase 4 — One vehicle body
- NPC cars are built from `vehicles.lua` recipes (car/truck/van/tractor-trailer),
  the same bodies the player's car uses, so they look identical. A shared body
  builder both the Lua path and the render bridge can call.

## Phase 5 — DriverAgent drives real physics (device)
- Split `Vehicle` into components (Body / Chassis / Handling / Seats / Lights /
  Controller). `DriverAgent` outputs `{throttle, steer, brake}` into a real Jolt
  vehicle near the camera; far cars stay kinematic (LOD) with a velocity-matched
  handoff. Player and AI drive the identical vehicle through the same seam.

## Notes
- Memory (recently-seen obstacles) is deferred — first pass is memoryless: agents
  act only on the current cone.
- The NavGraph still supplies road/sidewalk geometry and a coarse goal; it stops
  being a rigid rail. Determinism (ADR-0002) is preserved: fixed iteration order,
  seeded RNG, no wall-clock.
