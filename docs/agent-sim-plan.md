# Agent-Based City Simulation — Plan (ADR-0060)

A realistic, deterministic agent simulation built as an **application** of the
engine (not core), living under `src/apps/citysim/`. Agents are data; vehicles,
pedestrians, signs, and traffic lights are models with simulation aspects. The
deterministic decision core is headless and unit-tested; the physics/render layer
(Jolt wheeled vehicles, instanced models) is the device-verified skin on top.

## Vision

> Agents move around the road network obeying the rules of traffic (lanes, signs,
> lights, yielding) or the rules of walking (sidewalks, crosswalks, signals).
> An agent perceives the world through a forward vision cone, decides, and acts —
> imperfectly (agents can make faults). A car has no agency on its own; an agent
> *possesses* a vehicle and drives it. A pedestrian is an agent with no vehicle.
> **The player is just an agent whose brain is input** — the same possession model.

## Architecture

**Agent = brain (data).** An `Agent` carries a goal/route, perception params, a
fault profile, and decision outputs (throttle/steer/brake, or a walk velocity).
A brain is either *AI* (perceive → rules → act) or *player input* — identical
downstream, which unifies the player and the crowd.

**Possession.** An agent optionally *possesses* a `Vehicle` entity. Possessed →
the agent's decisions become the vehicle's controls. Unpossessed agent = a
pedestrian (walks). A `Vehicle` with no driver is inert (parked, braked).

**Models are entities (instanced).** Cars, pedestrians, signs, and traffic lights
are ECS entities drawn via `InstanceGroup` (one batch per model). Their *pose*
each step comes from the simulation; their *mesh* is a procgen/Lua model.

**Two layers.**
- **Deterministic sim core** (headless, CI-tested): perception, traffic rules,
  signal controllers, agent decisions, kinematic motion along the NavGraph. Pure
  `(state, dt) → state`; reproducible from a seed (ADR-0002).
- **Physics + render integration** (device-verified): cars near/driven by the
  player run **Jolt wheeled physics** fed by the agent's decisions; far cars stay
  kinematic (the ADR-0057 hybrid). Models drawn instanced.

## Core vs. application split

| Piece | Home | Why |
|---|---|---|
| `NavGraph`, A* pathfinding | **core** `engine/ai/` | generic navigation, reusable |
| Perception (vision cone) | **core** `engine/ai/perception.h` | generic, any game/agent |
| `PhysicsWorld` wheeled vehicle | **core** `engine/physics/` | generic vehicle physics |
| Traffic signals, traffic rules | **app** `apps/citysim/` | city-traffic-specific |
| Agent brains, schedules, faults | **app** `apps/citysim/` | the simulation's policy |
| `CitySim` driver + ECS wiring | **app** `apps/citysim/` | the application itself |

Anything that proves generally useful (e.g. perception, a steering/turn-radius
helper) graduates from the app into `engine/`.

## Phases (each lands with tests; build in this order)

**Phase 1 — Foundation (this commit, headless).**
- Core `perception.h`: a forward `VisionCone` + `sees()` / `forwardDistance()`.
- App `traffic_signal.{h,cpp}`: a deterministic `SignalController` over NavGraph
  junctions — opposing arms share a phase, phases cycle green→yellow→red.
- App `traffic_rules.{h,cpp}`: pure decision helpers — comfortable stop approach,
  signal speed cap, nearest obstacle ahead in a cone.
- *Tests:* cone sees front not behind / in not out of range; signal cycles and
  perpendicular arms are never both green; a red light caps speed to a smooth
  stop; an obstacle in the cone is detected and one behind is ignored.

**Phase 2 — Agent framework + decision core.**
- `Agent` (data) with a brain; possession of a `Vehicle`; pedestrians as
  brain-only agents. Migrate the existing kinematic `AgentSim` behaviour into the
  app's `CitySim` using the Phase-1 rules. The player becomes an agent that
  possesses a vehicle.
- *Tests:* an agent drives its possessed vehicle along a route; an unpossessed
  agent walks; a vehicle with no driver stays put.

**Phase 3 — Signals & crosswalks in the loop.**
- Wire `SignalController` into `CitySim`: cars stop at red, go on green, clear the
  box. Pedestrians wait at a crosswalk for the walk signal, then cross.
- *Tests:* a car approaching a red stops before the line and resumes on green; a
  pedestrian waits for the signal then crosses; no car enters on red.

**Phase 4 — Perception-driven avoidance + faults.** ✅ done
- Cars brake for cars/pedestrians in their vision cone (not just same-lane
  car-following). A per-agent *fault profile* (reaction delay, miss chance) makes
  behaviour imperfect.
- *Tests:* a car stops for a pedestrian crossing ahead; a car with a fault
  occasionally reacts late (deterministic with seed); no pedestrian is struck in
  a soak test.

**Phase 5 — Turn radius / smooth steering.** ✅ done
- Cars follow a curved arc through junction turns (bounded turn radius), not an
  instant heading snap; steering rate-limited.
- *Tests:* a turning car's path curvature never exceeds 1/turnRadius; heading
  changes are rate-limited (`tests/test_city_steering.cpp`): per-step yaw is
  bounded by `speed / minRadius`, turns still complete (~60 deg at a Y junction),
  and steering stays deterministic.

**Phase 6 — Physics + render integration.** 🚧 render done; hybrid physics device-side
- *Render bridge (done, headless-tested).* `citysim::CityRenderSystem`
  (`src/apps/citysim/city_render.{h,cpp}`) builds the NavGraph from the level's
  RoadNets, runs the CitySim, and bakes poses into InstanceGroups: one for cars,
  one for pedestrians, the city's `street_kit` traffic-signal assembly (pole + mast
  arm + 3-lamp head) per signalled approach, and a lit emissive lens placed on the
  matching head lamp to show each stoplight phase. `arena_state` now registers it
  in place of the old AgentSim-backed `TrafficSystem`. Covered by
  `tests/test_city_render.cpp` (build from RoadNet, agents move when stepped, one
  head assembly per approach, lit lens changes state with the phase) and
  `tests/test_city_flow.cpp` (a busy junction doesn't gridlock; cars pass oncoming
  traffic). The bridge is an APPLICATION layer that depends on core; core never
  depends on it.
- *Reuses the city's stoplights.* The signal heads are the same `trafficSignalProto`
  model the city generator places, positioned with the same near-right-corner
  geometry facing oncoming traffic; the `SignalController` (which agents already
  obey) drives which lamp is lit. The stoplights authored for the city now work as
  live traffic lights that cars and pedestrians respond to.
- *Anti-gridlock.* The perception cone yields only to pedestrians and the player,
  not to other AI cars: car-vs-car conflict is handled by lanes, same-lane
  car-following, and the signals. Braking for oncoming/cross cars in a wide cone
  deadlocked traffic with no way to clear.
- *Remaining (device).* Near/driven cars running Jolt wheeled physics fed by the
  agent brain (far cars stay kinematic — the hybrid model). The player car
  already uses Jolt via `VehicleSystem`; wiring AI cars onto Jolt near the camera
  is the device-verified follow-up. Instanced car/ped meshes are simple boxes
  today (signals use the real street_kit head); richer authored vehicle/ped models
  are a later content pass.

## Test strategy

The sim core is pure and deterministic, so phases 1–5 are fully covered by the
standard headless `make test` / CTest runner — no GPU. Each phase adds a focused
test file. Phase 6 (physics/render) is hand-verified on a device build, like the
rest of the Jolt/Metal/Vulkan path.

## Notes

- Determinism (ADR-0002): same seed + dt sequence → identical sim, including
  faults (seeded per agent). Enables reproducible tests and replays.
- This supersedes nothing in ADR-0057/0058 — it *builds on* the NavGraph, A*,
  AgentSim, and the wheeled-vehicle seam already shipped, reorganising the
  city-specific behaviour into an application and enriching it toward realism.
