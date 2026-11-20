# Models and prefabs: design note (#191 step 5)

Draft 2 for Glenn, 2026-11-19. The model half is built: steps 1 to 6 (nodes; markers and named materials; uses; motion, poses and rig tables;
levels of detail; collision shapes, models to and from glTF files, and the model store). The prefab half is not. See "What is built" at the end. Draft 1 put scripts, lights, sounds and offers inside the
model; Glenn's review split that in two: "an entity will contain the model and it would also then contain other
components to handle physics, audio, collision, gameplay logic". This draft follows that split.

## Words used here

- **Recipe:** a Lua script that builds or loads a thing.
- **Model:** the shape of a thing and nothing it does: its pieces, materials, rig, markers and collision shapes.
- **Node:** one named piece of a model, with its own position and turn under its parent. A car's door is a node
  under the car; the handle is a node under the door.
- **Rig:** what can move in a model and how. For a body, a skeleton and the weights that bind the skin to it. For a
  machine, which nodes turn or slide, about which axis and between which limits.
- **Marker:** a named point with a position, a direction and a size, where something can hook on. The same idea as
  an empty in Blender. It does nothing by itself.
- **Collision shape:** a simple solid (box, sphere, capsule, hull, or the mesh itself) that stands in for the model
  when things bump into it.
- **Level of detail (level):** one version of the model's meshes. The high level is drawn close up, lower ones
  farther away.
- **Entity:** a thing in the world. The engine's entities are bags of components.
- **Component:** one kind of data on an entity: what to draw, how it collides, what script runs.
- **Prefab:** a template for an entity: which model it shows and which components it carries. Stamped into the world
  as many times as wanted. (Unity's word, and the common one in engines built this way.)
- **Model store:** where built models are kept, each under a name, so many entities share one.

## The split

| | Model | Prefab |
|---|---|---|
| What it is | A shape | A kind of thing |
| Holds | Nodes, meshes, materials and maps, the rig, markers, collision shapes, levels of detail | A model's name, plus components: drawing, physics, sound, offers, script, light |
| Does anything? | No | Yes, through its components |
| Built by | A recipe, once, then cached in the model store | Lua data |
| Example | The crane's steel, its cab, the boom that can tilt 0 to 75 degrees, a marker named `hook` | "A crane is that model, solid and static, with an engine sound at `exhaust`, a lever offer at `lever_turn`, and `crane.lua`" |

The model answers "what is its shape and what can move". The prefab answers "what is it in the world".

## The model

```
Model
  name, size
  nodes[]            the rig for a machine; the root is node 1
    name             unique in the model: "body", "door_front_l", "boom_2"
    parent, frame    where it hangs and how it sits, relative to its parent
    motion           none | turn{axis, min, max} | slide{axis, min, max}     (what MAY move; nothing moves it here)
    markers[]        name, role, frame, size
    shapes[]         collision shapes: name, box / sphere / capsule / hull / mesh
    uses             another model placed here (a wheel used four times, a bolt forty)
  skeleton, weights  the rig for a soft body (as engine/anim has today), when there is one
  levels[]           high first
    switch distance  the default; a prefab may override it
    pieces[]         node name, mesh, material name
  materials{}        by name: "paint", "glass", "rubber": colour, roughness, maps
```

Rules:

1. **One rig, one set of markers, one set of collision shapes per model.** Every level hangs its meshes on the same
   nodes, so the hook is in one place whatever the distance.
2. **A lower level may be simpler in structure.** A far crane can be one mesh on the root node.
3. **Each level has its own materials.** The high level may use full maps, the low one a single baked map.
4. **A shared piece is stored once** (`uses`).
5. **A model with one node and one level is today's flat model.** Every existing recipe keeps working unchanged.
   `ProcModel` and Lua's `model.new()` are what grows; this is not a second system.

### Where a model comes from

Either way the result is the same record in the model store.

- **Built:** a recipe makes the meshes with the toolkit's words.
- **Loaded:** a recipe names files. A glTF file usually holds one level, so three levels are three files, and the
  recipe is what lists them:

```lua
return model.from_files{
  name = "crane",
  levels = { "crane_high.glb", "crane_mid.glb", "crane_low.glb" },   -- the first gives the rig and the markers
  distances = { 0, 40, 150 },
}
```

  - The first file's node tree is the rig. Its empties become markers. Meshes named with a collision prefix become
    collision shapes (the convention is to be chosen; `UCX_` is Unreal's).
  - The other files' meshes are matched to nodes by name; a mesh with no match hangs on the root.
  - One file may hold every level instead, by a name suffix (`_LOD0`, `_LOD1`).
  - A model with only a high level gets its lower ones from `mesh.simplify`.
  - Mismatches (a level naming a node the rig lacks) are reported when the model is built, not found in the game.
- Today's importer (`model_importer.h`) returns a flat list of meshes and materials, and `anim/skin_import.h` reads a
  skinned body. Loading fills the model record through them; they are extended, not replaced.

So nothing in the world lists files. The recipe does, the store keeps the result under the model's name, and
entities refer to that name.

## The prefab and the entity

A prefab is Lua data. Kinds are already data here (`street_props.lua`, `places.lua`, the furniture library); a prefab
is one general form for what those tables each say their own way.

```lua
prefab.define{
  name = "crane",
  model = "crane",
  physics = { body = "static", shapes = "all", friction = 0.7 },
  sounds = { { at = "exhaust", patch = "diesel_idle" } },
  offers = { { verb = "sit", at = "driver" }, { verb = "pull", at = "lever_turn" } },
  lights = { { at = "lamp_top", color = { 1, 0.6, 0.1 }, blink = 1.0 } },
  script = "crane.lua",
  draw = { distances = { 0, 60, 200 } },        -- optional: override the model's own
}
```

Stamping a prefab makes an entity with these components:

- **Model instance:** which model (a handle into the store) and this copy's own state: the angle or distance of
  each moving node. This is the one reference to the model; the other components find the model through it.
- **Drawing:** picks the level by distance and draws the pieces at their nodes' current places.
- **Physics:** reads the model's collision shapes through the model instance and makes the bodies. It keeps the
  body ids, not a copy of the shapes, so there is one source for the shape. Mass, friction and static or moving are
  here. When a node moves, its shapes' bodies follow.
- **Sound, lights, particles:** each says a marker's name; its position is read from the model, at the node's
  current place.
- **Offers:** what a person can do, each at a marker or in a collision shape used as a volume.
- **Script:** the existing `ScriptBehaviour` (ADR-0024). It can ask its entity for nodes and markers by name and set
  a moving node's target: `self.model:set("cabin", 90)`.

**Motion** lives on the entity. The model says the boom may tilt 0 to 75 degrees. The model instance holds this
crane's boom at 40. A script or a motion clip (#310) changes it; a small system moves each node toward its target
and stops at the model's limits.

**Thousands of copies.** A bench in every park cannot be a thousand full entities. A prefab's copies are drawn
together as one group until someone is near or touches one; that copy then becomes a full entity, and goes back
after. Cars and kerb props do this today, each by hand. It is a property of how prefabs are stamped, not of the
model.

## What it does not do yet

- **Physics joints.** A swinging load on the hook needs a real joint or rope (#311). Motion here is driven.
- **Weights from a script** for soft bodies: later work, on the same rig slot.
- **Breaking** (#312), **motion clips** (#310), **materials and decals** (#309) build on these two records.

## Order of building

Each is one commit with tests, and each leaves everything that exists working. The crane is built alongside, as the
thing both records have to be able to say.

**The model**
1. **Nodes.** `m:node`, parents, frames, meshes on nodes. The flat model is the one-node case. The model sheet and
   the loader read the tree. Proof: every existing recipe gives the same triangles; a child node draws in place.
2. **Markers and named materials.**
3. **Uses.** Forty bolts, one bolt mesh.
4. **Motion limits and poses.** Turn and slide on nodes; the model sheet takes `--set cabin=90` to draw a pose.
5. **Levels.** Several per model, by recipe or by `mesh.simplify`. Proof: triangle counts per level on the sheet.
6. **The model store and loading from files.** Names, handles, the cache key (recipe, arguments, required
   libraries, toolkit version); `model.from_files`.

**The prefab**
7. **Prefab data and stamping one entity.** Model instance, drawing, physics from the model's shapes. Proof: a
   scene with the crane; a physics test that the boom's shape stops a box.
8. **Motion on the entity and the script.** Proof: a test drives a node and reads where a child's marker ends up.
9. **Sound, lights and offers at markers.**
10. **Many copies:** drawn as a group, one made full when approached.
11. **First ports:** kerb props (ADR-0354), then furniture's spots and anchors as markers, then the car.

## Glenn's calls (2026-11-19)

1. **Angles:** degrees in Lua, radians inside the engine.
2. **The first port after the crane:** kerb props (the things standing at the edge of a street: hydrants, bins,
   mailboxes, posts), before furniture.
3. **Colliders on distant copies:** open. "It might make sense if it's only objects close to the player, but that
   depends if we allow physics collisions on things far away from the player." To settle at step 10 by finding out
   what far from the player still collides today (far traffic, thrown or falling things); until then every copy keeps
   its collider, as now.
4. **Names in a model file are our own conventions**, written in AGENTS.md ("Model file names"): `col_<kind>_<name>`
   for a collision shape, `_lod<N>` for a level, an empty for a marker. Glenn asked for a convention of ours and
   offered `CM` / `RCM`; `col_` is the proposal, because it reads without a key and the shape's kind can follow it.
   His to change.

## What is built

### Model step 1: nodes (2026-11-19)

```lua
local m = model.new()
m:add(base_mesh)                                              -- on the root, as every recipe did before
local cabin = m:node("cabin", { pos = { 0, 4.4, 0 }, turn = { 0, 40, 0 } })
cabin:add(cabin_mesh, paint)                                  -- the mesh is in the cabin's own space
local boom = cabin:node("boom", { pos = { 0, 1, 0.6 }, turn = { -30, 0, 0 } })
boom:add(boom_mesh, paint)
```

- **The record** (`procgen/proc_model.h`): `ProcModel::nodes`, each a name, a parent and a frame (a position and a
  rotation, no scale). Node 0 is the root and is always there; its frame is no move and no turn. A part has a `node`,
  0 unless said. `nodeFrame(i)` is a node's place in the model.
- **Lua:** `m:node(name, opts)` adds a node under the root, `n:node(name, opts)` under `n`; `opts.parent = "name"`
  names another parent. `pos` is metres in the parent's space; `turn` is degrees about x, then y, then z. `n:add`
  hangs a mesh. `m:find(name)`, `m:node_count()`, `n:name()`, `n:parent()`, `n:point{x,y,z}` ask the tree.
- **A wrong name stops the recipe:** a name used twice, or a parent the model lacks, is an error where it is written.
- **Merging** (`m:merge(other)`) puts the other model's nodes under this model's root and keeps their tree. Both
  models having a node of one name is an error.
- **The readers:** the offline bake, the level loader (each part's entity stands at its node's frame) and
  `m:flatten` all place a part by its node. The model sheet lists the tree and where each node ends up.
- **The proof:** the model sheet now prints a fingerprint of the drawn triangles. All 30 recipes in `assets/scripts`
  and the scratch folder that return a model gave the same fingerprint on the code before nodes and after.
  `tests/test_model_nodes.cpp` holds the rest.

Left as they were, on purpose, for the steps that own them:

- **Flatten footprints** are in the root's space. (Collision shapes went onto nodes at step 6.)
- **A textured part's tiling** is laid in its node's space in the game (so the pattern stays with the piece) and in
  the world's space in the path tracer. The two differ only for a textured part on a node that is not the root. The
  game and the path tracer lay a tiled map by two separate pieces of code today, so making them one is materials
  work (#309), not a step here.
- **In the game each part is still its own entity** with no link back to its node. The model instance (step 7) is
  what will hold the tree at run time.

### Model step 2: markers and named materials (2026-11-19)

```lua
local m = model.new()
m:material("steel", material.new{ roughness = 0.55, metallic = 0.6 })
local cabin = m:node("cabin", { pos = { 0, 4.4, 0 }, turn = { 0, 40, 0 } })
cabin:add(cabin_mesh, "steel")                                -- made of the model's "steel"
cabin:marker("driver", { pos = { 0, 0.4, 0.2 }, role = "seat", size = 0.5 })
cabin:marker("lamp_top", { pos = { 0, 1.05, 0 }, turn = { -90, 0, 0 }, role = "lamp" })
local at, facing = m:marker_at("driver")                      -- where it is in the model, and the way it faces
```

- **Markers** (`ProcMarker`, on `ProcNode::markers`): a name, a role, a frame under its node, a size.
  - The name is unique in the whole model, not only on its node, so a prefab can say `at = "exhaust"` and nothing
    more.
  - A marker faces along its own +z. `turn` is degrees, as a node's is.
  - `role` is `"point"` when none is said (the same default as a model file's empty). `size` is 0 when none is said.
  - `ProcModel::markerFrame(name)` and Lua's `m:marker_at(name)` give where it is in the model, through every node
    above it.
- **Materials** are one table on the model (`ProcModel::materials`). A part holds an index into it, not a material
  of its own.
  - Entry 0 is always the plain material (no maps, the colour on the vertices). A part that says nothing is made of
    it.
  - `m:material("paint", mat)` names an entry. `m:add(mesh, "paint")` makes a part of it; a name the model lacks
    stops the recipe.
  - `m:add(mesh, mat)` still works as before. The same material given forty times is one entry.
  - One name, one material: naming a different material "paint" is an error.
- **Merging** carries markers and materials across. The same material under the same name in both models is held
  once. Two markers of one name, or one name for two different materials, is an error.
- **The readers:**
  - The offline bake makes one scene material, and one copy of its maps, per material of the model. Before, every
    part brought its own copy.
  - The game's loader sends a material's maps to the GPU once, however many parts are made of it.
  - The model sheet lists the materials and the markers, and draws each marker as three bars: red x, green y, and a
    longer blue z for the way it faces.
- **The proof:** the fingerprint now also covers what each triangle is made of (its material's numbers and the
  pixels of its maps). 30 of the 31 recipes gave the same fingerprint before and after. The other, `bake_lab`, does
  not give the same number twice even on unchanged code: it bakes normal maps from a car's parts, and the car kit
  hands those over in a different order each run. Forty runs of the new code gave twenty or so different numbers,
  the old code's among them. `tests/test_model_markers.cpp` holds the rest.

Left as they were, on purpose:

- **Nothing reads a marker yet.** Sounds, lights and offers at markers are prefab steps 8 and 9.
- **A material is still what `material.new` makes** (roughness, metallic, maps, opacity, emission), with the colour
  on the vertices. A colour per material, and changing "paint" on one copy of a prefab, belong to #309 and step 7.
- **The path tracer still ignores a material's opacity and emission**, as before this step.

### Model step 3: uses (2026-11-19)

```lua
local tire = model.new()
tire:add(wall_mesh, "rubber")                                 -- a piece is a whole model: its own parts,
tire:add(rim_mesh, "alloy")                                   -- materials, nodes and markers
tire:use(bolt_mesh, ring_of_five)                             -- and its own uses: five bolts, one mesh
tire:marker("valve", { pos = { 0.12, 0.17, 0 } })

local car = model.new()
car:node("wheel_fl", { pos = { 0.8, 0.33, 1.3 } }):use(tire)                          -- the one tire,
car:node("wheel_fr", { pos = { -0.8, 0.33, 1.3 }, turn = { 0, 180, 0 } }):use(tire)   -- used at each wheel
local at = car:marker_at("wheel_fl/valve")
```

Glenn's call: the instance group is extended into this, not kept beside it. There is one way to place a shared piece.

- **The record** (`ProcUse`, in `ProcModel::uses`): the piece (a shared model), the node whose space it is placed
  in, and one placement per copy (a position, a rotation, one scale). `ProcInstanceGroup` is gone: it was a bare mesh
  with a roughness, placed by matrices in the root's space, and is now the case "a piece of one mesh, used on the
  root".
- **Stored once, drawn once.** The tire used at four nodes is one piece. Each part of it is one instanced draw with
  four copies. The bolts inside it are one draw with twenty.
- **Names inside a piece are its own.** They are reached through the node that uses the piece:
  `car:marker_at("wheel_fl/valve")`, as deep as the pieces go (`rear/left/valve`). A copy's turn and scale carry the
  marker with it. When a node places many copies, the path means the first.
- **Lua:**
  - `n:use(piece)` places it once, at the node. `n:use(piece, placements)` places it at each
    `{ pos =, turn =, scale = }` in the node's space, `turn` in degrees. `m:use` does the same on the root.
  - The piece is a model, or a bare mesh (with `{ roughness =, metallic =, alpha_foliage = }` for its look).
  - `{ collide = "mesh" }` or `{ collide = { shape = "capsule", radius =, height = } }` stamps a collider at every
    copy, as placed props always could.
  - `m:add_instances(mesh, placements, opts)` is the older word for the same thing and still works, with `yaw` in
    radians as its recipes wrote it.
  - A model cannot use itself, or a piece that uses it. That is an error where it is written.
- **One walk, four readers.** `placedUses` and `drawnPieces` (`procgen/proc_model.cpp`) say where every copy stands.
  The offline bake, the game's loader, `m:flatten` and the model sheet all read them.
- **A material is dressed one way.** A part of a used piece goes through the same material code as a plain part, in
  the bake and in the loader. A placed copy can now have maps, opacity and emission, which the instance group could
  not.
- **The proof:** every recipe gives the same fingerprint before and after (the fingerprint now also covers a placed
  piece's material), `blockdemo`'s lamps among them, with `bake_lab` differing only in its usual run-to-run way.
  `tests/test_model_uses.cpp` holds the rest.

Left as they were, on purpose:

- **No mirroring by a copy's placement.** A right door from a left door is the mirrored twin, below.
- **A used piece's own collision shapes** come with the model's `shapes` (step 7). Today a use says how its copies
  collide.

### The mirrored twin (2026-11-19)

```lua
car:node("door_front_l"):use(door)
car:node("door_front_r"):use(door:mirrored())     -- the right door is the left door, mirrored; authored once
```

- **`m:mirrored()` gives a new model that is `m` mirrored left to right** (across x = 0 through its origin). `m` is
  not changed.
- **The model is mirrored, not the copy's matrix.** Both renderers decide which side of a face is the front by a
  fixed winding, so a copy placed by a mirroring matrix would be drawn inside out. The twin's meshes are mirrored
  and wound to face out again (`MeshBuilder::mirrored`, which the toolkit already had), and its nodes, markers and
  copies are put where the mirror puts them. It is an ordinary model: no renderer changed.
- **The cost** is that the twin's meshes are held as well as the first's. A piece used many times inside the model
  is still mirrored only once.
- **A frame keeps its up and its facing, mirrored; its own x swaps sides.** A handle marker that faces out of the
  left door faces out of the right one.
- **Left to right only.** For another plane, hang the twin on a turned node. (A general plane was tried and cut: a
  frame has to stay a rotation, so one of its axes must swap, and which one only has a natural answer left to right.)
- **A model that grades terrain has no twin**; asking for one is an error.

### Rules for a rig, from Glenn's car example (2026-11-19)

Written down here so step 4 builds them in.

- **Limits belong to the rig, and the rig is part of the model.** A node that may move says how (turn or slide),
  about which axis and between which limits. They depend on that model's geometry. The current value is on the
  entity; what drives it is a component.
- **Motion sits above any flip or mirror.** For a wheel: `susp_front_r -> steer_front_r -> spin_front_r -> the tire (turned half a
  turn)`. The moving nodes are in the car's space, so the same steer value turns both front wheels the same way and
  the same spin rolls both forward.
- **The same value means the same action on both sides.** A right door's hinge axis points the other way, so "open
  40" opens either door. A mirrored twin's hinge node comes out with its turn already running the other way round,
  which is the first thing step 4 has to check against this rule.
- **One rig table per kind of thing** (Glenn, 2026-11-19: "I think a rig table makes sense"). A rig table is Lua
  data for a kind (car, crane, door): the joints' names, how each moves (turn or slide), about which axis, and its
  default limits. A recipe applies the table and gives only what is its own: where each joint is, and any limit
  that differs. So every car has a `door_front_l` that opens the same way, and one script drives them all.

### Model step 4: motion, poses and rig tables (2026-11-19)

```lua
local rigs = require "rigs"
local joint = rigs.apply(car, "car", {
  door_front_l = { pos = { 0.775, 0.4, 0.75 } },      -- the hinge line; the rig says how a door moves
  door_front_r = { pos = { -0.775, 0.4, 0.75 } },
  susp_front_l = { pos = { 0.8, 0.33, 1.3 } },        -- steer_front_l and spin_front_l follow, under it
  hood    = { pos = { 0, 0.86, 0.8 }, max = 55 },   -- this car's hood opens less far than the table's
})
joint.door_front_l:use(door)
joint.door_front_r:use(door:mirrored())
joint.spin_front_l:use(tire)
local ajar = car:posed{ door_front_l = 40, steer_front_l = 20, susp_front_l = 0.03 }   -- a still, for a picture or a prop
```

- **Motion on a node** (`ProcMotion`): none, a turn about an axis, or a slide along one, in the node's own space.
  - A turn's limits are degrees in Lua (radians inside) and may be left out: a wheel turns without end.
  - A slide's limits are metres and must be given.
  - The node as the recipe placed it is value 0, so the limits must have 0 between them. A door is built closed.
  - A value past a limit stops at the limit.
- **A pose** (`ProcPose`) gives values to moving nodes by name; every other node is at 0. A moving node inside a used
  piece is named through the node that uses it (`door_front_l/window`). `ProcModel::nodeFrame(node, pose)` is where a node
  stands in a pose.
- **The model holds no current value.** That is the entity's (step 7). What exists now for anything that draws a
  still is `posedModel` / `m:posed{...}`: a new model held in the pose, whose posed nodes no longer move. A used
  piece is held apart from its other copies only where the pose reaches into it.
- **Rig tables** (`assets/scripts/rigs.lua`) are Lua data, one per kind: each joint's name, its parent joint, how it
  moves and its default limits. `rigs.apply(model, kind, places)` makes the nodes and returns them by name.
  - The recipe gives each joint's place, and may give its own `min` / `max`.
  - A joint given no place is left out (a two-door car has no rear doors). A joint under another follows it without
    a place of its own (`steer_front_l` sits at `susp_front_l`).
  - A place for a name the table lacks is an error.
  - There are two tables so far: `car` (four doors, hood, trunk, and for each wheel a suspension slide, a steering
    turn on the fronts, and a spin) and `crane` (cabin, boom, hook line).
  - A new kind is a new table. No C++ changes.
- **The rules above hold in the car table and are tested:** the same steer value swings both front wheels the same
  way, the same spin rolls every wheel forward with the right-hand tires flipped under their spin nodes, and the same
  door value opens both doors outward by the same amount.
- **The mirrored twin moves the mirrored way:** the same value gives the mirror image of what it gave. (So a door
  with a sliding window inside it, mirrored, still lowers its window with the same value.)
- **The model sheet** takes `--set NAME=VALUE` (repeatable) to draw a pose, and its text lists every moving node,
  its axis, its limits and what it was drawn at.

Left as they were, on purpose:

- **Nothing moves in the game.** The loader draws every model at value 0. A value per copy, and the script, physics
  or clip that changes it, are the model instance (steps 7 and 8) and motion clips (#310).
- **One motion per node.** A joint that both slides and turns is two nodes, one under the other, as the wheel is.
- **The crane's hook line leans with the boom.** A line that hangs plumb and swings needs a real joint (#311).

### Damage and deformation on the model record (Glenn asked, 2026-11-19)

Not built. Written down so steps 7, 10 and 11 and the breaking work (#312) build it in.

**Today** (ADR-0352, ADR-0353) a drivable car keeps this on its own `Vehicle` component:

- A knock pushes the vertices near the contact inward (`dentMesh` in `engine/vehicle_damage.h`) and recomputes the
  normals.
- Cars share their body mesh until one is hit; that car then gets its own copy of the body and of each panel near
  the hit ("copied on the first dent"). A repair puts the built mesh back.
- A panel is dented in its own hinged frame, and comes off as a loose box at zero health. Its glass and trim are
  dropped.
- The component has a private rig for this: `Vehicle::Panel` (a pivot, an axis, an open angle, a slide), with lamp
  places and a driver's seat point read from the recipe.

**On the model record:**

- **The model never deforms.** A dent is one copy's state, like its door angle, so it is the model instance's
  (step 7).
- **The model instance can hold its own mesh for a part.** It is made on the first dent, for the part that was hit
  only, in that part's node's space. Everything else stays shared: the other parts, the tires, every other car.
  A dented door swings on its hinge because the dent is in the door's own space.
- **A part with its own mesh leaves the shared draw** and is drawn alone for that copy: the same way out of the
  group as step 10's "one copy made full when touched".
- **A piece coming off is a node leaving the tree.** Its subtree becomes an entity of its own that uses the same
  piece model (a door keeps its window and handle).
- **A repair** drops the instance's own meshes.
- **`Vehicle::Panel` is the same thing as motion on a node, written a second time.** The car port (step 11) moves the
  car onto the car rig table and retires it; the lamp places and the seat point become markers.

### An existing kit car on the car rig (2026-11-19)

`assets/scripts/car_rig_lab.lua` builds any car of `vehicle_kit.lua` as a model on the car rig, for the model sheet
only. Nothing in the game reads it. It is the first look at what the car port (step 11) will make.

```
./build-viewer/rt_model_sheet assets/scripts/car_rig_lab.lua --out build-viewer/sheets/car.png \
    --set door_front_l=50 --set door_rear_r=40 --set hood=45 --set trunk=50 --set steer_front_l=25 --set steer_front_r=25
./build-viewer/rt_model_sheet assets/scripts/car_rig_lab.lua --args '{"name":"pickup"}' --set tailgate=90
```

What it showed:

- **The kit needed no change.** It already gives each panel as its own meshes with a hinge point, the wheels' places,
  the lamps and the driver's hip point. The sedan comes out as 19 nodes: six panels, and for each wheel a
  suspension, a steering turn (fronts) and a spin, with ONE tire model used four times.
- **The rig's joints are named as the kit and the game already name a car's panels** (`door_front_l`, `hood`,
  `trunk`, `tailgate`), and the wheels follow (`susp_front_l`, `steer_front_l`, `spin_front_l`).
- **The kit's hinge axes agree with the rig table's** for every panel tried. They are still written twice:
  `kit.hinge` gives a pivot, an axis and an opening, and the rig table gives the axis again. At the port `kit.hinge`
  keeps the pivot and this car's own opening, and the axis is the rig's alone.
- **A bus's plug door slides and turns at once** (ADR-0356). The rig has one motion per node, so it is two joints,
  one under the other. Not in the table yet.

What the rig makes possible that the cars do not do today (ideas, none built):

- Wheels that turn with the steering and spin with the speed, and bodies that ride on their suspension travel,
  drawn from the same joints the physics writes.
- A steering wheel joint that follows the front wheels.
- Windows that lower: a slide joint under each door.
- A door that comes off keeps its glass, liner and handle, because it is a subtree and not a re-centred skin.
- Traffic cars (drawn in groups, not physics bodies) can share the rig, so a parked car can have a door open.

### Model step 5: levels of detail (2026-11-19)

```lua
rock:level(40, { simplify = 0.3 })                  -- a level made from the full one: each mesh at 30%
local mid = car:level(40)                           -- a level the recipe fills itself
door:add(low_poly_door, "body", { level = mid })    -- the same joints, lower builds of the same pieces
local far = car:level(150)
car:add(low_poly_body, "body", { level = far })     -- simpler in structure: one mesh on the root
car:use(tire, wheel_places, { level = far })        -- (the tire has its own levels, by its own distances)
```

- **The record:** `ProcModel::levelFrom` holds the distance each level is drawn from, the full model first at 0.
  A part and a use each have a `level`. A model that adds none has the one full level and is as it always was.
- **One rig, one set of markers.** Levels do not have nodes of their own: a level's parts hang on the model's
  nodes, or all on the root when the level is simpler in structure.
- **A used piece has its own levels.** The tire switches at the tire's distances and the car at the car's. A level
  of the car says which uses it draws at all (the far car places four plain discs and no bolts).
- **Two ways to make a level:**
  - *From the full one*, by the toolkit's simplify: `m:level(distance, { simplify = ratio })`. Each part becomes a
    simpler copy on the same node in the same material; each use is drawn again. For things with no recipe to
    build them lower (a rock, a scanned model).
  - *By the recipe*: `m:level(distance)` and then `{ level = n }` on what belongs to it. The first choice whenever
    the recipe can build the thing at a lower detail, and needed for a far level.
- **What the car lab showed about far levels.** Simplifying is not enough far away. Crushing the whole car (interior
  and all) into one mesh loses the wheels first; simplifying the outside alone to 12% crumples the panels, which
  are thin solids with back faces. The far level that reads well is the kit's own low-poly build of the same car
  plus plain discs for wheels.
- **And about middle levels (2026-11-20, Glenn looked at the file in Blender: "crazy looking, not in a good way").**
  The car's middle level was `simplify = 0.3` of the full one. Measured, it was worse than both neighbours: on the
  hood 46% of triangles thinner than 10:1 (26% on the full hood), an edge the length of the panel beside ones of
  0.2 mm, and none of the kit's lines left. The simplifier weighs only how far the surface moves, so on a flat
  panel it removes edges in no order. Glenn's call: build the middle level with the tools too. It is now the kit's
  body before subdivision, shaded smooth, with its panels on the same joints and the tire's 16-sided build. The
  sedan: 34,625 triangles near, 9,528 from 40 m, 3,912 from 150 m. A level that is built beats one that is derived.
- **Still owed from that measurement:** the kit's full mesh has thin triangles of its own (59% of the full hood's
  have a corner under 10 degrees, most of them the long narrow quads of the cage, some slivers under a tenth of a
  millimetre) and 480 edges shared by three or more faces on the body; the model sheet's audit does not measure
  triangle shape; and `simplify` has no guard on triangle shape. None is fixed here.
- **One simplify.** `MeshBuilder::simplifiedSurface` is the weld, simplify, weld and creasing that `mesh.simplify`
  did in the Lua binding; the binding and the model's levels both call it now.
- **The one walk takes a distance:** `placedUses`, `drawnPieces`, `flattenModel` and `bakeProcModel` draw what is
  seen from that far (0, the default, is the full model). The model sheet takes `--at METRES` and lists the
  triangles each level draws.

Left as they were, on purpose:

- **The game draws the full level only.** A script entity's parts are separate entities today, and each would
  measure its own distance and switch at a different moment (the fault Glenn saw on buildings, 2026-10-02: "parts of
  buildings disappear ... then pop in later"). One model switching as one thing is the prefab's drawing (step 7).
  It will use what every drawn thing already has: `minDistance` and `drawDistance` on `Renderable` and
  `InstanceGroup`, and the per-pixel crossfade `FLAG_LOD_BAND` that trees and building tiers use.
- **A prefab's own distances** (the design's `draw = { distances = ... }`) come with the prefab.
- **A level's materials are entries of the model's one table.** A far level that wants a single baked map names one
  more entry; baking it is materials work (#309).

### Model step 6, first piece: collision shapes on nodes (2026-11-19)

```lua
car:shape("body", { box = { 1.8, 0.6, 4.6 }, pos = { 0, 0.5, 0 } })
joint.door_front_l:shape("door_front_l", { box = { 0.1, 1.0, 1.1 }, pos = { 0.05, 0, -0.55 } })   -- goes with the door
joint.susp_front_l:shape("wheel_front_l", { sphere = 0.33 })
boom:shape("boom", { capsule = { radius = 0.2, height = 5 }, pos = { 0, 0, 2.5 }, turn = { 90, 0, 0 } })
rock:shape("rock", { hull = rock_mesh })
```

- **The record:** the model's colliders (`ProcCollider`, ADR-0042) are extended, as the instance group was. A
  collider gains a `name`, the `node` it sits on, a full rotation where it had only a turn about the vertical, and
  one more kind, the hull. The older colliders are the case "no name, on the root".
- **A name is unique among the model's named shapes**, so a prefab's physics or an offer can say "the cabin".
  Unnamed shapes are allowed, as many as wanted, for the recipes that used the older words.
- **Kinds:** a box, a sphere, a capsule (its length along its own y), a hull (the convex shape round a mesh's
  points) and a mesh (its triangles as they are, for static things).
- **Lua:** `m:shape(name, spec)` on the root, `n:shape(name, spec)` on a node. `m:add_collider`, `m:collide` and
  `m:add_solid` are unchanged.
- **A shape goes with its node:** in a pose, in the mirrored twin, and when two models are merged.
- **The game** stands each shape's collider where its node stands, turned as the node and the shape are. (A capsule
  collider used to ignore its turn.)
- **The model sheet** lists the shapes, and `--shapes` draws them in place of the meshes, coloured by kind.

Left as they were, on purpose:

- **The physics has no hull shape.** A hull is handed to it as its faces, which is right for static scenery. A hull
  on a body that moves needs a real convex shape in the physics, with the prefab's physics (step 7).
- **A used piece's own shapes are not stamped at its copies.** A use still says how its copies collide
  (`collide =`). The prefab's physics reads the model's shapes through the tree.
- **Nothing in the game moves a shape with a moving node**, because nothing moves a node in the game yet.

### Model step 6, second piece: a model to a glTF file, and back (2026-11-19)

```
./build-viewer/rt_model_sheet assets/scripts/car_rig_lab.lua --glb build-viewer/sheets/car.glb    # write it; open in Blender
./build-viewer/rt_model_sheet build-viewer/sheets/car.glb --out build-viewer/sheets/car.png       # the sheet of a file
```
```lua
local crane = model.from_file("crane.glb")                                  -- one file, levels by `_lod<N>` names
local crane = model.from_files{ name = "crane",
                                levels = { "crane_high.glb", "crane_mid.glb", "crane_low.glb" },
                                distances = { 0, 40, 150 } }                -- or a file per level
```

- **One convention, both ways.** AGENTS.md "Model file names" says how a file's names are read; a model is written
  by the same names. The convention grew by what the round trip needed, each a custom property so a file still opens
  anywhere: `motion` on a joint, `node = true` on a joint with no mesh, `use` on each copy of a shared piece, `levels`
  on the root, and the root being an object named `root`.
- **Writing a model is new to the plan** (the design listed only loading). It was added because the loader needs
  files with trees, empties and named shapes to be tested against, and a model written by the engine is the honest
  source of them. It also means any recipe's model can be opened and looked at in Blender.
- **What is written:** every node with its place and turn; a node's parts as one mesh with a material slot each; a
  joint's motion and limits; markers as empties with `role` and `size`; each collision shape as a `col_` object
  holding the solid it is; each lower level's parts as a `_lod<N>` object under its node; materials with their
  numbers and maps; and each copy of a used piece as an object, the first holding the piece and the rest sharing its
  meshes.
- **What is read from a file made in Blender:** the same, plus what an artist's file has that ours never does. An
  object's scale is put into its mesh. A file with no `root` gets one. A level named with no distance gets 40 m and
  doubling. Each of these is noted (the reader returns notes; a recipe's load logs them).
- **Levels as separate files** (`model.from_files`): the first file gives the nodes, markers and shapes. Each later
  file's meshes are matched to those nodes by name and become that level's parts. A mesh on a node the first file
  lacks hangs on the root where it stood, and is noted.
- **A script gives a file's name, never a path** (letters, digits, `_`, `-`, `.`, ending `.glb` or `.gltf`), as
  `require` takes a module's name. The host maps names to files under `assets/models`, then beside the level. A host
  that opens no model files says so.
- **The file is true glTF.** Triangles are counter-clockwise and every vertex has its colour, so another tool reads
  the model right and the engine's plain importer reads the same triangles, facing out.
- **Proof:** `tests/test_model_file.cpp`. A car with moving joints, markers, each kind of shape, five materials (one
  with a map, one brighter than glTF allows), three levels and a tire used four times with bolts inside it is
  written, read, compared field by field, posed, written and read again. Blender 4, headless, reads the kit sedan's
  file as 80 objects in the right tree with the tire copies sharing one mesh.

Found on the way, not changed:

- **The older exporter writes triangles the engine's way round** (`writeGlb`, the flat one behind the bundle, lanes
  and sign exports), and the importer turns glTF's the other way on reading. A file written by `writeGlb` and read by
  `loadCpu` therefore comes back inside out. Nothing has shown it because those files are for looking at in Blender,
  which draws both sides. `writeGlbTree` writes glTF's way. The two writers should become one.
- **The plain importer does not read vertex colours.** The model reader does; `loadCpu` was left as it is, because
  honouring them there would change how every existing glTF asset is drawn.

Left as they were, on purpose:

- **Files hold positions in single precision**, so a model read back agrees with the one written to about a micron.
- **A model that grades terrain, and a use with a collision mesh of its own, are refused** by the writer with a
  reason. Nothing is silently left out.
- **Baked maps beyond albedo and normal** (roughness, occlusion) are read by the importer but a model's material has
  no slot for them yet: materials work (#309).

### Model step 6, last piece: the model store and named models (2026-11-19)

```lua
-- assets/scripts/models.lua: the models the world can ask for by name
return {
  sedan  = { recipe = "car_rig_lab", args = { name = "sedan" } },
  pickup = { recipe = "car_rig_lab", args = { name = "pickup" } },
  crane  = { files = { "crane_high.glb", "crane_low.glb" }, distances = { 0, 80 } },   -- (an example; no such files yet)
}
```
```lua
local tire = model.get("tire")          -- in a recipe: the one tire, shared by every car that asks
joint.spin_front_l:use(tire)
```

- **A named model is a line of data.** A recipe to run with its arguments, or model files to read. A new kind of
  model is a new line. No C++ changes.
- **Built once, kept, shared.** `obtainModel("sedan")` gives the one sedan. The store is the furniture kit's way
  (model a piece once per variant and keep it) for any model. It is in memory; it writes nothing to disk.
- **A kept model is frozen.** It is shared, so the recipe words that add to a model or change it refuse it, and say
  to use it as a piece. A model made from a kept one (posed, mirrored) is the recipe's own.
- **Each model knows what it was built from:**
  - a *key*: the toolkit's tag (`kModelToolkitTag`, bumped when a mesh word's output changes), its line in the list
    and its recipe's text;
  - its *inputs*: each library the recipe required, each file it read, each other model it got, with the hash each
    had;
  - a *signature* over the key and all the inputs. This is what a model that uses another records for it, so a
    change two models down (the library the tire requires) rebuilds the tire and the car that uses the tire.
- **It is built again only when one of those has changed.** An edited recipe shows the next time the model is asked
  for, without a restart. A model that the change does not touch is left as it is.
- **A model that needs itself** (through others) is refused, as is a name the list lacks or a line with neither a
  recipe nor files.
- **`rt_model_sheet sedan`** takes a name in place of a recipe.

Left as they were, on purpose:

- **Nothing in the game asks the store yet.** A level's script entity still runs its recipe itself. The prefab
  (step 7) names its model and gets it here; that is the customer.
- **A handle is the model's shared pointer.** The design's "a handle into the store" needs nothing more until an
  entity's component must refer to a model by something that survives a rebuild; then it is the name.
- **A change is seen when the model is next asked for**, not while it is on screen. Swapping a rebuilt model under
  live entities is hot reload's business, with the prefab.
- **The lab recipe is what the list names.** `car_rig_lab.lua` was written for the model sheet; when the cars are
  ported (step 11) the list names the kit's own recipe.

### Prefab step 7: prefab data and stamping one entity (2026-11-20)

Glenn's go: "Prefabs would come first and then we should test it with various procgen/meshbuilder models."

**The list.** `assets/scripts/prefabs.lua` returns a table of name = line, as `models.lua` does (not the
`prefab.define{}` call sketched above: one list file reads the same way as the other, and is checked the same way).

```lua
return {
  crane = { model = "crane", physics = { body = "static" }, draw = { class = "structure" } },
  parked_sedan = { model = "sedan", physics = { body = "static" } },
}
```

A line may say `model` (a name in `models.lua`), `physics = { body = "static" | "none", friction = n }` and
`draw = { class = "<a level's draw class>", distance = metres }`. Anything else is refused with a message, never
half obeyed: a misspelt key names itself, and `sounds`, `lights`, `offers`, `particles` and a body that falls
each say they are not built yet (`script` came with step 8). (`distance`, not `until`: `until` is a Lua keyword.) Read by
`scripting/prefab_list.{h,cpp}` (`findPrefab`, `namedPrefabs`) into the plain record `procgen/prefab.h`.

**Stamping** (`engine/prefab_stamp.{h,cpp}`, `stampPrefab`) makes one entity with a `ModelInstance` component: the
prefab's name, the model (the store's own, shared by every copy) and this copy's pose. Around it:

- **Pieces.** One entity per part of the model, and per part of each copy of a used piece, each with a `ModelPiece`
  (its owner, its place in the model, the band of distances it is drawn in) and a `Renderable`. The list comes from
  `modelDraws` in `procgen/proc_model.cpp`: the model's parts and `drawnPieces` at every distance, cut into bands at
  the model's level distances and at every used piece's own, a part that is the same across bands being one draw.
- **One model, one distance.** `Renderable` gains `lodAnchored` / `lodAnchor`: the distance tests measure to that
  world point, not to the mesh's own bounds. Every piece of an instance is given the entity's position, so the body
  and the four tires change level at the same moment, and cross-fade the way a city cell's tiers do. The draw index
  is told how far a piece is from its anchor (`lodAnchorReach`) so it does not cull a cell too soon.
- **Bodies.** With `physics.body = "static"`, the model's collision shapes become physics entities where the
  entity's place and each shape's node put them. A box, sphere or capsule is a body of its own; meshes and hulls are
  merged by friction into one static mesh body each.
- **A level** places one with `{ "shape": "prefab", "prefab": "crane", "position": [...], "orientation": {...} }`.
  The stamped entity carries the `SourceSpec`, so the editor saves it back as the same line. The offline tracer
  bakes the prefab's model at the entity's position (it cannot turn a model yet, and says so).
- `unstampPrefab` destroys an instance with its pieces and bodies, handing each physics body id to the caller to
  remove from its physics world.

**One copy of the code that puts a model in a world.** The level loader's recipe entity (`shape: "script"`) had the
material dressing and the collider building inside it. Both moved to `engine/model_spawn.{h,cpp}` (`ModelDresser`,
`spawnModelColliders`) and the loader and the stamp both call them. The loader's behaviour is unchanged.

**The crane** is `assets/scripts/crane.lua`, in `models.lua`: boxes on the crane rig, a collision shape on each
solid node, a far level at 120 m. It is boxes on purpose; see "The crane Glenn asked for" below.

**Proof.** `assets/levels/prefab_lab.json`: four cranes (one turned a quarter, one 200 m off) and four parked kit
cars. `tests/test_prefab.cpp` (unit): the list and its refusals, the draw bands against `flattenModel` at every
distance, the stamp's pieces and bodies, the renderer's fields, shared meshes across copies, the sedan with its
used tires, and the lab level loaded and saved back. `tests/test_prefab_physics.cpp` (physics): a box dropped over a
turned crane's boom rests on it at 5.78 m; one dropped where the boom would be unturned reaches the floor; with
the crane unstamped a box falls through where the boom was.

**What step 7 left alone:**

- **A pose does nothing yet.** Done in step 8, below.
- **A copy of a used piece is one entity per part.** Right for one crane; step 10 draws many copies as groups.
- **Bodies that move** (a bin that can be knocked over) wait for the first port that needs one, with the real hull
  shape.
- **An occluded piece still casts its shadow from every level** (the renderer's shadow-only draw comes before the
  distance test, as it does for city tiers). Co-located levels cast nearly the same shadow.
- **`draw.distances`**, overriding the model's own level distances per prefab, is not built.

### Prefab step 8: motion on the entity and the script (2026-11-20)

The model says the boom may tilt 0 to 75 degrees. A copy's `ModelInstance` holds this crane's boom at 40.

- **The words** (`engine/model_motion.{h,cpp}`, no physics needed): `setModelTarget(world, e, "boom", value, speed)`
  heads a moving node for a value at a speed (no speed: there at once), stopped at the model's limits;
  `advanceModelMotion` takes one step toward every target; `modelValue` reads a node; `modelNodeWorld` and
  `modelMarkerWorld` say where a node or a marker is now, in the world. A node inside a used piece is named through
  the node that uses it, as in a pose.
- **The pieces follow** without a posed copy of the model: `placedUses`, `drawnPieces` and `modelDraws` take a pose,
  and give the draws in the same order with their places moved, so piece i is draw i (`poseModelPieces`).
- **The bodies follow.** A box, sphere or capsule on a node that can move (`ProcModel::nodeMoves`) is stamped as a
  kinematic body with a `ModelShape` (which of the model's shapes it is). `ModelMotionSystem`
  (`systems/model_motion_system.{h,cpp}`, in the game's system list after the elevators) advances every instance
  with a target each fixed step and drives each such body to where its node now is (`moveKinematic`), so it pushes
  what it meets.
- **The script.** A prefab line takes `script = "crane_swing.lua"`: a behaviour script (the existing
  `ScriptBehaviour`, ADR-0024) every copy runs. Its words: `entity.model_set(e, "boom", 40 [, speed])`,
  `entity.model_get(e, "boom")`, `entity.model_marker(e, "hook")`; degrees for a turn and metres for a slide, per
  Glenn's call 1. (The design sketched `self.model:set("cabin", 90)`; the script API passes the entity to every
  hook, so the words sit beside `entity.set_position`.)
- **Proof.** Unit: a cab driven a quarter turn at 30 degrees a second arrives in three seconds, the hook marker,
  child of the boom, child of the cab, ends where the turn puts it, and running the line out 2 m lowers it 2 m; the
  sedan's front door opens and only the door's pieces move. Physics
  (`tests/test_prefab_motion_physics.cpp`): the boom, swung by the cab, sweeps a box off a post beside the crane.
  `prefab_lab.json`'s middle crane is `crane_working` and runs `crane_swing.lua`.

**What step 8 left alone:**

- **A mesh or hull shape on a moving node stays where it is at rest**, with a warning at the stamp: the physics has
  no moving mesh or hull shape yet. Boxes, spheres and capsules move.
- **A used piece's own per-copy colliders** (the old instance-group kind) do not move.
- **Two uses that coincide only at rest** would merge into one draw at rest and be two when posed; such an instance
  keeps its pieces where they were, with a warning. No model does this today.
- **The hook line leans with the boom,** as the rig says: a line that hangs plumb is a real joint (#311).
- **Motion clips** (#310) would set targets the same way.

### The crane Glenn asked for (2026-11-20)

"The crane would be an interesting test of the rig and building a more complex mesh using the mesh building tools.
I'd also want to see PBR materials on its surface, actual windows, and the ability to have a pulley and rope with
some rope physics holding up a steel girder. And then have the ability to rotate it around via entering in a
control cabin and having a special set of interactions for it." And: "building out the systems are higher
priority." So the crane is the test at the end, and each wish is a system first:

| The crane needs | The system | Where |
|---|---|---|
| a lattice boom and mast, rounded edges, bolts | truss, edge bevel, scatter on a surface | modelling tools, below |
| actual windows | an opening with a frame and a pane | modelling tools, below |
| PBR surfaces | materials and decals | #309 |
| a pulley, a rope, a girder on it | cable mesh (tools), rope physics and joints | #311 |
| turning it from the cab | motion on the entity and the script | step 8 |
| entering the cab, its own interactions | offers at markers | step 9 |

### Modelling tools still to build (2026-11-20)

Glenn: modelling tools are "of higher interest" than the crane itself. What the toolkit lacks, in the order
proposed to him:

1. **Edge bevel:** round or chamfer chosen edges of a hard-surface part. BUILT 2026-11-20 as a chamfer: `bevel` in
   `procgen/polymesh.{h,cpp}`, Lua `P:bevel(width [, angle [, sel]])`. Every edge whose faces turn more than the
   angle is cut back to a flat strip, each face keeping its plane; a corner where three or more meet gets a small
   face. Bevel twice (a second, smaller width) for a rounder edge. It does not round in one go, does not stop a
   width wider than a face from folding it, and leaves the open border of a mesh alone.
2. **Loop cut and bridge** on the quad mesh: add a ring of edges; join two open rings. Also what would let the car
   cage's hood be more than one quad across. BUILT 2026-11-20, in `procgen/polymesh.{h,cpp}`:
   - `P:loop_cut(at_a, at_b [, t])`: a ring of edges through the quads, starting across the edge whose ends are
     nearest two places; it stops at the open border or a face that is not a quad, and that face gains the point.
   - `P:slice(point, normal)`: the same kind of cut along a plane, whatever the faces. Nothing moves.
   - `P:bridge(sel_a, sel_b)`: the two regions' faces go and their rims are joined by quads: two solids become one,
     or a block gets a hole through it. The rims must have the same number of points.
   The car cage has not been changed with them.
3. **Opening with a frame:** cut a hole in a wall or panel and get the frame and the pane back as parts. BUILT
   2026-11-20 as a Lua library over the quad mesh words, `assets/scripts/openings.lua`:
   `openings.window(wall, { at, normal, width, height, thickness, frame, set_back, bars, pane })` cuts the wall in
   place (four slices, then a bridge front to back) and returns `{ frame, pane }`; `openings.hole` cuts only. The
   wall stays one closed solid. A doorway down to the floor is open at the bottom: `bridge` drops a new quad that
   would lie back to back with a face already there. Rectangular openings in a flat wall only; an arched head or a
   curved wall is not built.
4. **Truss:** chords and diagonals along a path, the section free to taper.
5. **Scatter on a surface:** copies of one piece (bolts, rivets) along edges or over faces, as uses.
6. **Cable and rope:** a hanging curve between two points; a twisted strand along a path.
7. **Cage deform:** move a few control points and the mesh inside follows.
8. **Un-subdivide:** drop every other loop of a quad mesh, for an orderly lower level.

## The model half, done; what the prefab half starts from

Every part of the model in "The model" above is built: nodes, the rig, markers, collision shapes, uses, levels of
detail, named materials; models from recipes and from files; and a store that keeps each under a name. Nothing in
the game is drawn or moved by it yet. Step 7 starts the prefab half and is where that changes. What it has to build
on, each noted where it arose above:

- **A model instance on the entity:** the model's name, each moving node's value, and (for dents) its own mesh for a
  part.
- **Drawing by level,** one model switching as one thing, on `minDistance` / `drawDistance` and `FLAG_LOD_BAND`.
- **Physics from the model's shapes,** through the tree, a shape following its node; a real hull shape.
- **The copies of a used piece** drawn in groups, and one made full when it is touched (step 10).
- **Two descriptions to make one** when the car is ported: `Vehicle::Panel` and the rig; `kit.hinge`'s axes and the
  rig table's.
