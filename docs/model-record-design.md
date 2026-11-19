# The model record: design note (#191 step 5)

Draft for Glenn, 2026-11-19. Nothing here is built. It says what the model record is, how a recipe writes one, how it
becomes things in the world, and the order to build it in. The questions at the end are his to answer before code.

## Words used here

- **Recipe:** a Lua script that builds a thing.
- **Model:** what a recipe returns. Not only triangles: the whole description of a thing.
- **Node:** one named piece of a model, with its own position and turn under its parent. A car's door is a node under
  the car; the handle is a node under the door.
- **Marker:** a named point with a direction on a node, where something goes or happens: a seat, a lamp, a hook.
- **Component:** something a node carries besides its shape: a collider, a script, a light, a sound, an offer.
- **Offer:** a thing a person can do at a node: sit, open, pick up, pull a lever.
- **Motion:** how a node is allowed to move: turn about an axis or slide along one, between limits.
- **Assembler:** the one piece of engine code that turns a model into things in the world.
- **Drawn form / live form:** the cheap way a thing is drawn when nobody is near (one of thousands of copies in one
  draw), and the full thing with its own parts, colliders and scripts when someone is.
- **Level of detail:** a simpler version of a mesh, drawn when the thing is far away.

## What exists today

A recipe already returns a model. `model.new()` in Lua makes a `ProcModel` (`procgen/proc_model.h`), which holds:

- parts: a mesh and a material each;
- placed copies: one mesh drawn at many places;
- colliders: a box, sphere, capsule or mesh;
- ground-flattening requests for the terrain.

Two engine paths read it: the level loader for the game, and `bakeProcModel` for the offline renderer and the model
sheet. So the record to grow is `ProcModel`, and the Lua word to grow is `model`. This is not a new system.

What it lacks is everything that makes a model a thing and not a lump:

- **It is flat.** No piece has a name or a parent. A door cannot be found or moved.
- **No markers.** Furniture keeps its seats in `FurnSpot` and `FurnAnchor`, cars keep theirs in the vehicle spec, kerb
  props in their own table. Each family has its own.
- **No motion.** Car doors swing on hinges (ADR-0352) through car-only code.
- **No components.** A script, a light or an offer cannot be put on a model.
- **No levels of detail.** Cars carry near, mid and far forms by hand; furniture has one form.
- **One way into the world.** Scripted models go through the loader's script path; cars, furniture and kerb props
  each have their own spawning code.

## The record

A model is a tree of nodes. Everything hangs on a node.

```
Model
  name, size, the levels of detail's switch distances
  nodes[]            the root is node 1
    name             unique in the model: "body", "door_fl", "boom_2"
    parent           the node it hangs from
    frame            where it sits and how it is turned, relative to its parent
    parts[]          mesh + material name, one list per level of detail
    markers[]        name, role, frame, size
    colliders[]      box / sphere / capsule / hull / mesh; solid or not; friction
    motion           none | turn{axis, min, max, speed} | slide{axis, min, max, speed}
    components[]     script, light, sound, particles, offer, ...
    uses             another model placed here (a wheel used four times, a bolt forty)
```

Rules that keep it one system:

1. **A node is the only place anything lives.** A model with one node is today's flat model, so every existing recipe
   keeps working unchanged.
2. **Materials are named, not numbered.** "paint", "glass", "rubber". Each family's own material list goes.
3. **A marker's role is data.** `role = "seat"`, `"lamp"`, `"hook"`, `"door"`. No meaning hidden in a name's prefix.
4. **A shared piece is stored once.** `uses` points at another model; the wheel's mesh exists once however many
   wheels there are. A stocked shelf is a model that uses other models.
5. **Levels of detail belong to the node's parts.** Level 0 is the full mesh. Lower levels come from the recipe, or
   from `mesh.simplify` when the recipe gives none.

## In Lua

The existing words stay. New ones add structure.

```lua
local m = model.new{ name = "crane" }
local base  = m:node("base")
local cabin = m:node("cabin", { parent = base, at = {0, 2.1, 0},
                                turn = { axis = {0,1,0} } })               -- no limits: all the way round
local boom  = m:node("boom",  { parent = cabin, at = {0, 1.2, 1.0},
                                turn = { axis = {1,0,0}, min = 0, max = 75, speed = 8 } })
local boom2 = m:node("boom_2", { parent = boom,
                                 slide = { axis = {0,0,1}, min = 0, max = 6, speed = 0.5 } })

cabin:add(cabin_mesh, "paint")
cabin:collider{ box = {2.2, 2.0, 2.4} }
cabin:marker("driver", { role = "seat", at = {0, 0.6, 0.2} })
cabin:offer{ verb = "pull", marker = "lever_turn" }
cabin:script("crane.lua")
boom2:marker("hook", { role = "hook", at = {0, -0.3, 6.0} })

for i, at in ipairs(bolt_places) do base:use(bolt, { at = at }) end        -- one bolt mesh, many places
return m
```

`m:add(mesh, material)` with no node still works and means the root.

## From a model to the world: one assembler

One function, `assemble(model, where)`, replaces the per-family spawning code. It can make a thing two ways from the
same record:

- **Drawn form.** For things that stand in their thousands. Each node's parts join that mesh's shared draw; its
  colliders become plain static shapes. No scripts run, nothing moves. This is how street props, furniture and
  traffic are drawn today.
- **Live form.** For the copy someone is near or touching. Each node becomes an entity under its parent, with its
  parts, colliders, markers and components. Scripts run, motions can be driven, offers appear.

A thing changes from drawn to live and back by one rule the assembler owns (distance, or being touched). Cars and
kerb props already do this, each by hand; this makes it one way.

**Motion** is a value per moving node (an angle or a distance), held by the live form. A script sets a target:
`model:set("cabin", 90)`. The assembler moves the node toward it at the node's speed and stops at its limits, and
moves the node's colliders with it. This is the "animated hinge" chosen for car doors, for any node.

**Scripts** are the existing `ScriptBehaviour` (ADR-0024). A model's script gets its own model, so it can find nodes
and markers by name.

**Caching.** A model is built once per recipe and arguments, and reused. Its key must cover the recipe, its arguments,
every library it requires, and the toolkit's version. How the furniture cache keys itself today has to be read before
this builds on it.

## What it does not do yet

- **Physics joints.** A swinging load on the crane's hook needs a real joint or rope (#311). Motion here is driven,
  not simulated.
- **Skinned bodies.** Characters keep their skeleton; weights from a script are later work.
- **Breaking** (#312), **motion clips** (#310) and **materials and decals** (#309) build on this record.

## Order of building

Each is one commit with its own tests, and each leaves everything that exists working.

1. **Nodes in the record and in Lua.** `m:node`, parents, frames, parts on nodes. The flat model is the one-node
   case. The model sheet and the loader read the tree. Proof: every existing recipe gives the same triangles as
   before; a two-node test model draws its child in the right place.
2. **Markers and named materials.** Proof: a model's markers come back by name and role in C++ and in Lua.
3. **Uses.** One model placed inside another, stored once. Proof: forty bolts, one bolt mesh.
4. **Motion.** Turn and slide, limits and speed, colliders that follow. Proof: a test that drives a node and reads
   where its child's marker ends up; the model sheet takes `--set cabin=90` to draw a pose.
5. **The assembler's live form.** Entities under parents, components, the script. Proof: a small scene with the
   crane; a physics test that the boom's collider stops a box.
6. **The assembler's drawn form and the change between them.** Proof: a hundred cranes in one draw; walk up to one
   and it comes alive.
7. **Levels of detail on parts.** Proof: triangle counts by distance in a lab scene.
8. **First ports.** Kerb props (ADR-0354), then furniture's spots and anchors as markers, then the car.

The crane is built alongside steps 1 to 5 as the thing the record has to be able to say.

## Questions for Glenn

1. **Where does a model's script live?** In the same file as its recipe (one file is the whole thing), or beside it
   (`crane.lua` builds, `crane_behaviour.lua` runs)? One file is simpler to hand around; two keeps the build script
   from being loaded every frame. I lean to two files named in the recipe, as in the sketch.
2. **Units for turning:** degrees in Lua (as the sketch), radians inside. Agreed?
3. **The first port after the crane:** kerb props are the smallest and already data. Furniture is the biggest win
   (92 pieces, seats and anchors become markers) but touches interiors everywhere. I would do kerb props first.
4. **Does the drawn form keep colliders for every copy?** A thousand benches with colliders is what happens today.
   Keeping that is the plain choice; the other is colliders only near the player.
