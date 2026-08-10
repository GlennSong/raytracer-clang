# Vehicle unification + the Lua/C++ boundary — handoff

Written at the end of the P8.2 perf round. This is the brief for the NEXT
session: what was decided, what was measured, what to do, and what is still
unknown. Read this before touching vehicle or citysim code.

Companion: **`docs/piedmont-project-memory.md`** — the accumulated history and
hard-won root causes (why things are the way they are). This file is the
forward plan (what to do next). Read the memory first if you have no context
on this project at all; read this one first if you are here to work.

## The architecture Glenn set

**C++ is capability. Lua is description.** C++ owns primitives — `mesh.car`,
`mesh.lathe`, polygon inset, Coons patches, bisection, curve displacement, the
graph welders, the meshers. Lua owns what a thing *is*: a sedan, an old-town
block, a day in an agent's life. "We can't possibly describe everything in
C++. C++ should be the functionality that lua can then call to build those
things."

**Scripting is engine infrastructure, not a build option.** No `#ifdef`
deciding whether the Lua layer exists. Vehicles are OPTIONAL CONTENT: a level
with no vehicle catalogue, or `cars: 0`, must build a perfectly good empty
city. Absence is a data decision, never a compile-time one — and there is
nothing to "fall back" to, because *no cars* is legitimate and *box cars*
never were.

**The procedural car is the vehicle, not a physics prop.** `mesh.car` +
`vehicle_classes.lua` is the model for how everything should work: a new
vehicle is a catalogue entry, not a code change. LODs must be DECIMATIONS of
that authored geometry — never a second, cheaper re-authoring. A car at 200 m
is a simpler version of the same car; it is never a box.

### Standing criticism to answer

Everything built in the P8 round is C++ with no Lua surface: `city_footprint`,
`arterial_skeleton`, the tiered fill. Worse, the *choices* inside them are
string-dispatched enums in C++ (`fabric: "chords"|"bisect"|"court"|"mix"`,
`skeleton: "footprint"`) — recipes wearing a JSON knob. Adding a city style
means editing C++. That is the gap to close.

## What is measured and settled (do not re-litigate)

- **The tier system is correct.** Headless A/B over the real skeleton, matched
  simulated time, 60 / 30 / 7.5 Hz: the active/far split is IDENTICAL hour by
  hour, and CPU scales 125 -> 67 -> 21 ms per simulated second. Three separate
  "causes" I proposed (sample staleness, rate-dependence, tethering) all
  evaporated under tighter control. `tests/test_sim_rate_ab.cpp` pins this.
- **The real downtown problem is that the bubble is a RADIUS**, so cost scales
  with local density. At midday the workforce is parked within 500 m of the
  player and `active` legitimately hits ~2100. Nothing is malfunctioning; the
  design has no upper bound.
- **A viewer number is a symptom until a deterministic harness reproduces it.**
  `tools/piedmont_perf.sh` waits for the city to exist, pins the camera, and
  samples a SIM-HOUR window; it prints the census above the framerate so a
  mismatched comparison is obviously invalid. Wall-clock comparisons are void:
  a slower build drops more clock backlog and reaches a different sim hour.

## The audit to run FIRST (nothing is committed until this lands)

1. **Every reference to the box path** — `fleetCarMesh`, `fleetBodySize`,
   `kFleet`, the box composition in `city_meshes.cpp`, and the box `parts`
   wheels still in `vehicles.lua`. Include tests, `car_gallery`, headless
   paths. Produce a deletion list that is verified, not hopeful.
2. **What mesh reduction exists.** Real decimated LODs need a simplification
   pass over authored geometry. If the engine has none, that is a NEW
   component and Glenn should hear it before work starts, not midway.
3. **What procgen is hardcoded in C++ that should be a recipe**, and which
   primitives need Lua bindings to make that real (candidates:
   `city.footprint{}`, polygon bisection, line displacement, chain stationing,
   the fabric variants, the architect's building tables).

## Then, in order — each independently landable and gated

1. **Catalogue unification.** `vehicle_classes.lua` becomes the single source
   for dimensions, wheelbase, track, wheel diameter, mass, steer/torque/brake
   limits. Delete `kFleet`. This removes the fit-scaling hack in
   `vehicles.lua` that currently DISTORTS the mesh to match a hardcoded box —
   collision extents, drawn mesh and physics chassis then derive from one set
   of numbers.
2. **One pose authority.** Resolve each agent's draw pose once per bake; body,
   lamps, debug widgets and spectator all read it. (P8.2k fixed the lamps to
   prefer `physPose_`, but the structure still lets them diverge.) NOTE: do
   NOT pass an agent index to `agentPose()` from a second call site — it
   ADVANCES the tilt low-pass, and running it twice a tick reintroduces the
   vibration `car_pose_probe_no_vibration_on_jagged_ground` exists to catch.
3. **LOD by decimation** of the authored car: LOD0 multi-part (possessed +
   player), LOD1 merged vertex-coloured (instanced traffic), LOD2 distant.
   Same silhouette; wheels stay round.
4. **Motion envelope from the catalogue.** The kinematic tier should emulate
   the physics envelope, not the reverse: corner speed from lateral grip,
   deceleration from brake torque, acceleration from torque/mass. Today it
   drives at `classSpeed(roadClass)` — a per-road-type constant — which is why
   possession can visibly jolt. Gates: the drive probes and the gridlock soak,
   not just a green build.
5. **Deletions last**, once nothing references them.

## Independent of all the above: the downtown budget

This is what actually recovers the framerate, and it does not depend on the
vehicle work:

- **Cap the simulated set** — keep the N nearest agents fully simulated,
  demote the rest to V regardless of distance (sort by distance, uid as
  deterministic tiebreak). The bubble then shrinks in dense areas and stretches
  on an empty road: cost bounded instead of density-proportional.
- **Decouple "simulated" from "drawn"** — a V agent inside render range must
  still be DRAWN from its coarse pose (extrapolation already exists), it just
  should not sense, car-follow, or own a physics proxy. Today V means neither,
  which is why capping the sim would pop visible cars out of existence.

## Known open items elsewhere

- GPU passes are still unattributed: ~8.4 M triangles, 44 ms, with clouds /
  SSAO / SSR never isolated. Needs per-pass toggles.
- Cloud grain in dense banks (Perlin-Worley round).
- P8-G: the editable footprint polygon in the viewport — the last unbuilt
  piece of the approved masterplan.
- Parcel gaps: `courtMinArea` is documentation-only; ~8% of lots still green
  from heavy-clip fill rejects; acute corners can stay unparceled.
