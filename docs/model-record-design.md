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

**The crane, rebuilt with the first tools (2026-11-20).** `assets/scripts/crane.lua` is no longer boxes: the mast
and the boom are lattices (`truss.lua`), the cab is a floor, a roof and four walls with a framed, glazed window in
three of them (`openings.lua`), and the base, the cab and the hook block have bevelled edges. Its nodes, markers and
collision shapes are as they were, so it is the same prefab. What the sheet's numbers say honestly: a lattice
member is an open-ended tube that passes through its neighbours at a joint (1,920 open edges on the crane), which
reads right at any distance a crane is seen from and would not survive a close look at a joint.

### Modelling tools still to build (2026-11-20)

Glenn: modelling tools are "of higher interest" than the crane itself. What the toolkit lacks, in the order
proposed to him:

1. **Edge bevel:** round or chamfer chosen edges of a hard-surface part. BUILT 2026-11-20 as a chamfer: `bevel` in
   `procgen/polymesh.{h,cpp}`, Lua `P:bevel(width [, angle [, sel]])`. Every edge whose faces turn more than the
   angle is cut back to a flat strip, each face keeping its plane; a corner where three or more meet gets a small
   face. Bevel twice (a second, smaller width) for a rounder edge. A fifth argument, `rounds`, rounds the edge in more passes (2: three
   facets, 3: seven; added the same day). A width wider than a face has room for is held back, corner by corner,
   so no face folds over. It leaves the open border of a mesh alone.
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
4. **Truss:** chords and diagonals along a path, the section free to taper. BUILT 2026-11-20 as a Lua library,
   `assets/scripts/truss.lua`: `truss.build{ path, bay, section, size, size_end, chord, brace, pattern, ... }` gives
   one mesh and `info` (counts, the length, every joint frame by frame, for hanging things on it). Square,
   triangular or your own section; zigzag, crossed or no diagonals; the path may bend. Members are round tubes that
   pass through each other at the joints; `nodes = true` puts a ball on every joint, which covers that (no gusset
   plates; round members only).
5. **Scatter on a surface:** copies of one piece (bolts, rivets) along edges or over faces, as uses. BUILT
   2026-11-20 as a Lua library, `assets/scripts/scatter.lua`: `scatter.ring`, `scatter.along`, `scatter.grid`,
   `scatter.faces` and `scatter.join` each give a list of placements for `m:use(piece, placements)`. A placement
   may now say `up = {x,y,z}` (and `spin`) in place of `turn`: the piece's own +y stands along it. `scatter.jitter(list, { seed,
   move, spin, scale, drop })` nudges a list by a seed. Nothing follows a curved surface between the points given.
6. **Cable and rope:** a hanging curve between two points; a twisted strand along a path. BUILT 2026-11-20 as a
   Lua library, `assets/scripts/cable.lua`: `cable.hang(a, b, { sag })` gives the points of a hanging curve (the
   true one when the ends are level), `cable.wire` one round wire along points, `cable.rope` strands twisted along
   them, `cable.length` the length. Shapes that stand still: a rope that swings is #311.
7. **Cage deform:** move a few control points and the mesh inside follows. BUILT 2026-11-20 as a Lua library,
   `assets/scripts/cage.lua`: `cage.new{ lo, hi, counts }`, `c:move(i, j, k, delta)` or `c:move_where(rule)`, then
   `c:mesh(m)` or `c:poly(P)`. The classic free-form deformation: smooth through the whole box with three or more
   control points along an axis, straight with two. A box-shaped cage only, and an inner control point moves the
   surface by less than it moved.
8. **Un-subdivide:** drop every other loop of a quad mesh, for an orderly lower level. BUILT 2026-11-20:
   `unsubdivide` in `procgen/polymesh.{h,cpp}`, Lua `P:unsubdivide([near])`. A mesh with a subdivided mesh's
   pattern loses a level: a quarter of the faces, the same grid. The points that stay do not move. A mesh with an open border works (added the same
   day); one with a face that is not a quad is refused. Not yet tried on a kit car's body.

All eight tools on this list are built. What each does not do is said in its line.

**Curves and surfaces from control points (Glenn asked about splines and NURBS, 2026-11-20).** The toolkit already
had a spline THROUGH points (`mesh.path.spline`), Beziers, arcs and helices, and everything that follows a path. Added:
`mesh.path.bspline(control, { closed, count, degree })`, a smooth path pulled TOWARD its control points (moving one
reshapes only its own stretch), and `mesh.bspline_surface(rows, { across, up, degree, ...look })`, a smooth sheet
pulled toward a grid of control points, through its four corners. A path is a plain list of points, so the truss,
cable and scatter libraries take any `mesh.path.*` result as it is (`scatter.along` now goes once round a closed
one). Not built: weights and trimming curves (full NURBS: wanted only to read CAD data, since circles and conics are
made directly), a surface that closes on itself, and moving one control point of a path already made.

### The model studio: a script pane beside the engine's viewport (2026-11-20)

Glenn: "create another mode in that editor that would allow me and you to interactively build a Lua script right
there in that tool and then see the output in a 3-D panel next to it ... the actual engine runtime ... as I walk
around the model I could give you feedback and then you could update the Lua script or I could." It is the ask
parked as #104, now cheap because the model record, the store and prefabs exist.

**Built so far (the working loop):**

- **The dock.** `src/editor_app/model_studio.{h,cpp}`, "Model script" in the Qt editor, tabbed with the Hierarchy.
  New (a starter model under a name), Open (a `.lua` in `assets/scripts`, or double-click one in Assets), Save
  (Ctrl+S in the pane), Reload. `editor_app --model crane` starts in it.
- **What the viewport shows.** The panel writes a small level, `cache/model_studio/<name>.json` (not tracked):
  daylight, a checkered ground with physics, and one prefab entity that names the recipe directly:
  `"prefab": { "recipe": "<name>" }`. So the model is a stamped prefab in the real engine (levels of detail,
  collision shapes, moving nodes), with no line in `models.lua` or `prefabs.lua` yet. Play runs it: walk round
  it, bump into it.
- **Rebuilding is the engine's own script watcher,** not a second mechanism. The loader hands the watcher every
  file the model is built from (`modelSourceFiles`: the recipe, each library it requires, each file it reads, the
  same for models it uses). Saving in the pane rebuilds; so does a write to the file from anywhere else (an
  assistant editing it), and so does an edit to `truss.lua` if the script requires it. The pane follows the file
  when it has no unsaved edits, and says so when it has.
- **A mistake** shows under the pane in red with its line, and the pane's cursor goes to the line. The recipe stays
  watched, so mending it rebuilds.
- **The camera** opens looking at the model (a level with `"editor_view": "models"` is framed on what its prefabs
  draw) and is left alone on a rebuild.

**Engine pieces under it:** `obtainRecipeModel` / `recipeModelName` (a model straight from a recipe, kept in the
store as `recipe:<name>`), `Prefab::recipe`, the level entity's object form, `modelSourceFiles`, the loader's
watch list for prefabs (which also makes `prefab_lab` rebuild when `crane.lua` changes).

**Not built yet, in the order planned:**

1. DONE the same day: while a recipe has a mistake the viewport shows the model as it last built (the stamp
   falls back to the store's last model for a prefab straight from a recipe, and still reports the error).
2. "What made this?" PARTLY DONE the same day. A part and a use record the recipe line that made them
   (`ProcPart::line`, `ProcUse::line`: the outermost Lua frame, so a part added inside `truss.build` is put down to
   the recipe's line that called it); a stamped piece carries it with a description (`ModelPiece::line`, `what`:
   "steel on boom, 1,240 triangles"); picking a piece in the viewport puts the pane on that line and says what it
   is. Tested up to the stamped piece; the click in the editor has not been tried by a person yet. Not done: nodes,
   markers and collision shapes recording their lines; a label passed through a library for finer tags than one
   line; a highlight on the picked piece that covers every piece of the same line.
3. DONE the same day: "Copy view" puts on the clipboard the script, the piece last picked with its line and
   that line's text, the cursor's line, any error shown, the model's numbers, where the camera is and the path of
   a screenshot taken then. Pasted into a message, it says exactly what is being looked at.
4. DONE the same day: "Add to lists" gives the script its line in `models.lua` and `prefabs.lua` (static, solid),
   refusing while the script has a mistake and leaving a name already listed alone.
5. DONE the same day: line numbers, Lua colouring, and the model's numbers under the pane (size, triangles per
   level, pieces, collision shapes, nodes, markers).

6. DONE the same day (Glenn: "would it help to also show a tree of all the nodes and how they were placed?"):
   **the model's tree** under the script. Every node under its parent with where it was placed (position and
   turn relative to the parent) and, for one that moves, how and between what limits, with a box to move it (the
   viewport follows: swing the cab, open a door). Under each node what hangs on it: parts (material, triangles,
   level), used pieces (copies), markers (role, place), collision shapes (kind, size, place). Every row has its
   script line (nodes, markers and shapes record theirs now, as parts do); clicking a row goes to the line and,
   for a part, selects its piece in the viewport. `RT_STUDIO_PRESS=pose` moves two nodes unattended.

7. DONE the same day: **an Arguments box.** A Lua table typed there (`{ kind = "tree", seed = 3 }`) is what the
   script reads as `args`; Enter rebuilds. The engine turns the text into the JSON the level carries
   (`luaTableToJson`); the studio writes it into its level, whose own file is watched. Kept with the script's
   studio level for next time, shown in Copy view, and written into the `models.lua` line by Add to lists.
   Two things it showed, both mended: the camera now frames the model again when a rebuild gives something
   nothing like what was framed (a model where there was none, or one over twice or under half the size); and a
   part's line is the deepest one in the recipe's own chunk, so a part added in a function the recipe defines
   points at the line in that function, not at the call.

8. DONE the same day (Glenn: "how do I know what the kind is?"): **a recipe says what it takes, and the studio
   shows a form.** `local a = params{ kind = { "tree", "rock" }, seed = 1, tall = { 8, min = 2, max = 30 }, name =
   "", solid = true }`: a list of names is a choice (the first, or `default =`), then numbers (a whole one stays
   whole; limits hold it), texts, yes or no. `a` is the arguments given over the defaults. An argument the recipe
   does not take, or a choice not on its list, is an error that names what it does take, so a mistyped argument
   is never quietly ignored. The declaration is kept by the recipe's name (`recipeParams`), even when the
   arguments given were wrong, and the studio builds a control for each (a drop-down, a number box, a text box, a
   tick), choices first, in a scrolling strip above the script; moving one rebuilds. The free-text box stays, and
   shows what the form set. `prop_models.lua` and `car_rig_lab.lua` declare theirs (the car's choice is whatever
   `vehicle_kit.lua`'s `SPECS` holds).

9. DONE the same day (Glenn: "do you have MCP access into that editor?"): **the studio and the model on the
   control channel** (docs/control-channel.md), which the repo's MCP server (`tools/viewer-mcp.py`, `.mcp.json`)
   turns into tools by reading the running program's own command list. The editor contains the engine, so it is one
   program with one socket: there is no second server.
   - In the ENGINE (the viewer has them too, for any stamped prefab in any level): `models?` (the stamped models,
     numbered), `model? [n]` (one's tree as JSON), `model_set <node> <value> [speed] [n]` (degrees or metres).
   - In the EDITOR only (registered by it, so a plain viewer does not offer what it cannot do): `studio?` (the
     script, its arguments, unsaved edits, the status and whether it is an error, the model's numbers, what was
     picked and its line), `studio_open <script>`, `studio_args <a Lua table | none>`, `studio_pick <piece>`.
   - The model's tree has ONE description now, `engine/model_inspect.{h,cpp}` (`modelTreeJson`): the studio's tree
     draws it and `model?` hands it out.
   Checked by launching the editor and sending every command over the socket, and by asking the MCP server for its
   tool list beside a running editor (`model_q`, `model_set`, `models_q`, `studio_q`, `studio_args`,
   `studio_open`, `studio_pick`) and calling one. An assistant's session has the tools once the server is
   registered with it; this session's was not.

Still open: finer tags than one line per library call; a "walk" button (Play already does it); typing and clicking in the pane have been
checked only by pressing the buttons unattended (`RT_STUDIO_PRESS=copy|lists`), not by a person.

### The studio's next shape: a model or a prefab, play, and an inspector (agreed 2026-11-20, not built)

Glenn's calls, from talking it through:

- **The studio views one of two things.** A MODEL (a recipe script, as now: shape only, nothing to press) or a
  PREFAB (an entry in `prefabs.lua`: a model plus what makes it a thing in the world). Opening a prefab shows its
  model's recipe in the script pane, so the recipe is still edited there, by hand, by either of us.
- **Sound, physics, interactions and behaviour live on the prefab, not the model** (as steps 7 to 9 already say).
  Each attaches to one of the model's markers or nodes by name. The model only names the places.
- **Play is the editor's own Play button on the studio's level.** Viewing a prefab, the real prefab is stamped:
  solid from the model's collision shapes, moving parts as moving bodies, its script running. Viewing a bare model,
  Play is a walk-around of a solid, still object. An edit while playing rebuilds it (the script watcher already
  runs in play); the player should keep their place across that rebuild, which is to be checked.
- **An inspector for the prefab's components.** The editor's existing Inspector dock, not a second one. The tree
  gains a prefab half (body, script, then step 9's sounds, lights and offers), each row with the marker or node it
  sits on. Selecting a row shows its fields; each component kind declares its fields as a recipe declares its
  arguments with `params`, so the form is built from data and a marker field is a list of the model's own markers.
  A component naming a marker the model lacks is an error on its row. Model rows are read-only facts.
- **The editor rewrites prefab data only.** An inspector edit rewrites that prefab's entry in `prefabs.lua`
  (comments and layout inside the entry are not kept) and the watcher restamps. It never rewrites a recipe script.
- **Overlays** drawn with the engine's debug lines (`engine/debug_draw`): collision shapes, joint axes and limits,
  markers; later a mark and a reach for each sound, light and offer. In play, live values in the inspector.

Order: (1) the studio opens a prefab entry, body and script as its first components, and Play stamps it; (2) the
physics overlay; (3) step 9 as prefab components with rows, overlays and attachment checks.

**Built the same day: (1) and (2).**

- **The studio views a model or a prefab.** A "Showing" list in the dock: the model alone (the script run
  directly), or any prefab in `prefabs.lua` whose model that script builds. "Open prefab..." picks one by name and
  puts its model's recipe in the pane; "Make prefab" (was "Add to lists") gives the script its two lines and views
  the new prefab. `editor_app --prefab <name>` starts there. A prefab's studio level is
  `cache/model_studio/<name>.prefab.json`, with the plain `"prefab": "<name>"` entity a real level uses, so what
  the viewport shows is what a level would get. The loader now watches `prefabs.lua` for such an entity, found or
  not, so a wrong line can be mended and tried again. The Arguments box is greyed while a prefab is viewed and
  shows its model's arguments from `models.lua`.
- **The prefab's own rows in the tree**, above the model's: its model, physics, draw and script, each in words, with
  the line of `prefabs.lua` the entry starts on. Clicking one brings the Inspector forward.
- **The Inspector's prefab form.** At the top of the editor's existing Inspector dock, shown only while a prefab is
  viewed: model (the names in `models.lua`), physics body and friction, draw class and distance, script. It is
  built from data the engine gives (`prefabFieldsJson`), one control per thing a line can say; an optional number
  has a "not set" position that leaves its key out of the line. Changing the model puts the new model's recipe in
  the pane.
- **The one rewrite.** A change in the form calls `savePrefab` (`scripting/prefab_list.{h,cpp}`): the line is
  checked exactly as the list's reader checks it (`readPrefabLine`, which `findPrefab` now also uses), and only then
  written. `putPrefabLine` replaces that prefab's entry and nothing else: every other character of the file is as
  typed, including comments before and after the entry; comments inside the entry go. A new prefab is added as the
  table's last line. A wrong value writes nothing and is said under the form. No recipe script is ever rewritten.
- **Play.** The toolbar's Play works on the studio's level as on any other: the prefab is solid, its script runs
  (the crane's cab, boom and hook move), and the model overlay and the tree's values follow. A script edit while
  playing rebuilds the level, and the player now stays where they stood (`ArenaState::reloaded` hands the fresh
  state the player's place; before, every rebuild sent them back to the spawn point). A model viewed alone was
  already solid in play (a recipe run directly is stamped with a static body).
- **Lines over the model** (`engine/model_overlay.{h,cpp}`, `systems/model_overlay_system`): three boxes in the
  dock, "shapes", "markers", "joints". Collision shapes where they now are (green: still; orange: on a node that
  moves; a mesh or hull as the box round its points), markers as three axes, and each moving node's axis, limits
  and present value. Drawn with the engine's debug lines, in the editor (which now has the debug line system) and
  in play, for every stamped model in any level.
- **On the control channel** (docs/control-channel.md): `prefabs?`, `prefab? <name>`, `model_overlay ...` in the
  engine; `studio_prefab <name | none>`, `studio_prefab_set <part>[.<key>] <value | none>`, `play [here]`, `stop`
  in the editor. `studio?` now says whether a model or a prefab is shown, the prefab's data, and which lines are on.

Checked: unit tests for the entry rewrite, the prefab as data and the overlay's lines; and the editor run unattended
(`build-viewer/sheets/studio/prefab_check.py`, `play_check.py`, `overlay_check.py`, not tracked): the form's write
seen in `prefabs.lua` and undone, a wrong class and a missing script refused, the joints' values changing in play,
the player at the same place after a rebuild in play, both captures looked at. NOT checked by a person: the form's
controls and the boxes under a mouse.

**The first thing made in it: a park sculpture** (`assets/scripts/park_sculpture.lua`; Glenn asked for "some kind of
sculpture that could be placed in a park as a piece of furniture"). A plinth 0.45 m tall to sit on (rounded with
`P:bevel`, a seat marker on each side, a plaque held by four bolts from `scatter.grid`), and on it a slab with a
hole cut through it (`openings.hole`), made smooth (`P:subdivide`), then twisted, narrowed and leant (`P:map`),
with wires strung across the hole (`cable.wire`) whose ends are moved by the same rule. Its `params` are finish,
height, twist, taper, lean and strings. Two models in `models.lua` (`park_sculpture`, bronze; `park_sculpture_steel`,
taller and turned the other way) and two prefabs: `park_sculpture`, and `park_sculpture_turning`, whose script
(`park_sculpture_turn.lua`) turns the form on its node, 3 degrees a second. It is not yet placed by the city: a
park's furniture comes from the city's own lists, and wiring a prefab into those is its own piece of work.
Seen: the sheet (`rt_model_sheet assets/scripts/park_sculpture.lua`), the editor's prefab view, and play (the form
turning). Its collision box is one turned box and fits a strongly twisted form loosely.

**The second: playground pieces** (Glenn, 2026-11-20: build the pieces as models, each a prefab; "the playground
itself would not be a prefab but a site", the place the pieces are set out in). Four recipes over one small library
(`assets/scripts/playground_kit.lua`: the shared materials and paint colours, a pipe along a line of points with
its corners rounded, a foot plate):

| Recipe | What it is | What moves | What it takes |
|---|---|---|---|
| `playground_swing.lua` | a top bar on two A-shaped ends, seats on chains | each seat, `seat_1`..., swings about the bar, 65 degrees either way | seats (1 to 4), height, paint |
| `playground_seesaw.lua` | a plank on a pivot, a seat and a handle each end, half a tyre under each end | the plank, `beam`, tips until an end meets its tyre (the limit is worked out from the plank's length) | length, paint, seats' colour |
| `playground_roundabout.lua` | a round deck with studs and grab rails on a bearing | the deck, `deck`, turns without end, rails and post with it | radius, rails (3 to 8), two colours |
| `playground_slide.lua` | steps to a railed platform, a steel chute down from it | nothing | height, paint |

Each has collision shapes on the node they belong to (so a seat's box swings with the seat), a level seen from
40 or 50 m, and markers for what step 9 will hang on them: `seat_N` and `ride_N` (role seat or stand), `push`,
`hold_N`, `slide_start`, `slide_end`, and `creak` or `rumble` where a sound would come from. The slide's steps,
platform and chute are solid: a player dropped on the platform stays there, and one put on the chute slides down
it (its shapes have a friction of 0.08).

Six models and nine prefabs (`models.lua`, `prefabs.lua`). The three that move each have a still prefab and an
`_in_use` one with a script that moves it the way riders would: `playground_swing_sway.lua` (each seat a sine
swing, out of step with the next; it finds how many seats the model has), `playground_seesaw_rock.lua` (asks for
more tilt than the model allows and lets the limits stop it), `playground_roundabout_spin.lua` (pushed up to
speed, then slowing). When prefabs carry interactions (step 9) the riders move them and the scripts go.
Seen: a sheet of each (`rt_model_sheet assets/scripts/playground_swing.lua`, with `--shapes` for the collision
shapes), and each prefab in the editor in play, its joints read twice 1.5 s apart.

Not done, on purpose: **the playground itself.** Setting pieces out on a patch of park, with a surface under them
and a fence round them, is a site: something that holds prefabs at places, like the building of #316. Nothing here
places a piece; a level does, by name. That is part of #315.

**The third: furniture and street pieces with moving parts** (Glenn, 2026-11-20: an office chair that adjusts, a
desk that goes up and down, and "what else can we make that we can later use in the city as furniture"; then
"build them all"). 39 models and 67 prefabs from 21 recipes.

Two libraries carry them:

- `assets/scripts/model_kit.lua` builds a model's moving parts out of the FURNITURE KIT's own shape words
  (`furniture_models.lua`: rounded boxes, rods, turned shapes, cushions, drawer fronts) in the kit's finishes and
  colours, so the new pieces match what the interiors already place. The kit now hands out its maker as the global
  `furniture_kit`; nothing about the kit's pieces changed. `prop_models.lua` and this use the same material names.
- `assets/scripts/model_moves.lua` is one behaviour for every prefab that moves: a prefab's script is a short list
  saying what each node does (go there and back, spin, sway, step), and `return require("model_moves").script{...}`.
  A move stopped short by the model's limits counts as arrived, so a script may ask for more than a part can give.

One engine change went with it: **a behaviour script can `require`** (`ScriptSystem::setModuleSource`; the arena
gives it the same finder recipes use, and a library it reads is added to the watch, `ScriptWatch::add`). Before,
only recipes could, and each of 25 scripts would have carried its own copy of the loop.

| Where | Models | Moving nodes |
|---|---|---|
| Office | `office_chair_adjustable` (and `_navy`) | `lift` (gas column, 12 cm), `swivel`, `recline` (20 degrees), `caster_1..5` |
| | `standing_desk` (and `_wide`) | `stage` and its child `top`, 0.3 m each: 0.72 m to 1.32 m |
| | `filing_cabinet`, `desk_pedestal` | `drawer_N` slide out, each a real box |
| | `monitor_arm`, `desk_lamp` | `shoulder`, `elbow`, `pan`, `tilt`; `swing`, `lower`, `upper`, `head` |
| | `whiteboard`, `desk_fan`, `stand_fan`, `ceiling_fan`, `wall_clock` | `board` flips; `blades` spin, `head` sweeps; `hour`, `minute`, `second` |
| Home | `wardrobe`, `kitchen_cupboard`, `kitchen_wall_cupboard`, `store_cupboard`, `fridge` | `door_N`; `fridge_door`, `freezer_door` |
| | `rocking_chair`, `recliner` | `rock`; `back`, `footrest` |
| Laundromat | `washing_machine`, `dryer` | `door`, `drum` |
| Street | `parking_barrier`, `rising_bollard`, `turnstile` | `arm` lifts 90 degrees; `post` sinks; `arms` turn |
| | `litter_bin`, `mailbox`, `newspaper_box` | `flap` |
| Shops | `patio_umbrella`, `awning`, `barber_pole` | `rib_1..8` fold; `bar` and `strip_1..10` roll out; `stripes` turn |
| Park | `picnic_table`, `park_bench`, `drinking_fountain`, `bike_rack` | none: they are here so a park's set comes by name |
| Buildings | `revolving_door`, `roller_shutter`, `lift_doors` | `wings` turn; `section_N` rise into the box; `door_left`, `door_right` part |

Each moving piece has a still prefab and an `_in_use` one (the clock always runs; the barber's pole always turns).
Handles, levers, buttons, seats and places to stand are markers (`handle_N`, `lever`, `control`, `call`, `seat`,
`front` ...), for step 9. Cupboards, the fridge and cabinets are solid as their walls, not as one box, so an open
one can be reached into.

Two tricks worth knowing, both for cloth or a curtain that cannot stretch: the awning's cloth is ten strips and
the shutter's curtain is sections, each on its own node with its own limit; send them all the same way at one
speed and each stops at its place, which looks like one sheet unrolling.

Seen: a sheet of every model in an opened pose, and all 30 moving prefabs in the editor in play, joints read twice.

What these do NOT do yet, and why:

- **Nothing responds to a person.** Pulling the chair's lever, pressing the desk's control, opening a drawer by its
  handle are interactions: step 9. Until then a script stands in.
- **The chair does not roll**, and nothing can be pushed: a prefab's body is fixed or absent. A body with weight,
  and hinges that are real joints, is #311 (the swing's seat is the same gap).
- **The clock shows this computer's time**, not the game's: a script cannot ask for the time of day.
- **Pieces that hang** (clock, ceiling fan, awning, barber's pole) have their origin on the wall or ceiling, so in
  the studio, which stands every model on the floor, they sit half in the ground. They are right in a level that
  places them at a height.
- **Nothing in the city places any of them.** The interiors still place the kit's still `office_chair` and `desk`.
  Swapping furniture over is #315.

### The fourth: pieces for the city's own kinds of place (2026-11-21)

Glenn: "if you know what kind of buildings are in the city you can design prefabs for them". The city's places
(`assets/scripts/places.lua`, `trades.lua`, `floor_programs.lua`) are homes and apartments, offices, shops by trade
(bakery, bookshop, boutique, electronics, grocery, konbini, pharmacy), cafe, restaurant, bar, club, supermarket,
department store, hotel, the campus (teaching hall, library, residence hall, sports field), civic buildings and
parks. 46 more models and 81 more prefabs from 22 recipes, chosen against that list and against the furniture
kit's 92 still pieces (which already has the counters, booths, lecture rows, bookcases and beds):

| Place | Models | What moves |
|---|---|---|
| Any building | `door`, `door_glass`, `double_door`, `kitchen_swing_doors`, `automatic_doors` | leaves on hinges (a swing door both ways); glass panels that part |
| Konbini, grocery, supermarket | `drinks_fridge` (and `_single`), `ice_cream_freezer`, `cash_register`, `microwave`, `vending_machine` (and `_red`), `shopping_cart`, `basket_stack`, `atm` | glass doors; sliding lids; the till's drawer and screen; the plate; the flap; the cart's child seat and casters |
| Cafe, restaurant | `espresso_machine`, `round_table_turning`, `sandwich_board`, `diner_stool` | the lever and steam wand; the table's middle; the board folds; the seat turns |
| Bar, club | `bar_stool_swivel`, `beer_taps`, `mirror_ball`, `dj_deck` | seat; tap handles; the ball; two records and the crossfader |
| Campus | `lecture_seats`, `lecture_blackboards`, `projector_screen`, `lab_stool`, `lockers`, `book_truck` | seat pans fold and tablets swing; one board up as the other comes down; the screen unrolls; doors |
| Cinema, station | `cinema_seats`, `coin_lockers`, `ticket_machine` | seat pans; doors |
| Civic | `queue_posts`, `flagpole`, `ticket_machine` | the flag runs up the pole |
| Hotel, apartments | `luggage_cart`, `mailboxes` | casters; doors |
| Mall, department store, boutique, bookshop | `escalator`, `mall_directory`, `clothes_rail_round`, `postcard_rack`, `display_turntable`, `capsule_machines`, `kiddie_ride_car`, `kiddie_ride_rocket` | steps; racks and the turntable turn; knobs; the ride rocks and bobs |
| Park, plaza | `fountain`, `fountain_plaza` | none (markers for water and sound) |

New in how they are made:

- **Stock that is drawn many times.** The fridge's and the vending machine's bottles and cans, and the capsule
  machines' capsules, are each stored once and placed by `scatter.grid` / `scatter.along` (`m:use`).
- **A loop.** `model_moves.lua` gained `{ node, loop = metres, speed }`: slide that far and jump back. The
  escalator's steps each move up by exactly one step's place, so the jump cannot be seen and the stairs flow. Its
  script reads how far that is from two markers on the model (`run_from`, `run_to`), because it depends on the
  rise and slope the model was built with. `escalator_up` and `escalator_down` are the same model.
- **Cloth that unrolls** again (the projector screen), as the awning and the shutter.

Seen: a sheet of each, and all 37 moving prefabs in the editor in play (the escalator from its foot).

### The fifth: pieces for a home (2026-11-21)

Glenn: "Things for a home?" 34 more models and 67 more prefabs from 11 recipes (two of the models are older
recipes at a home's size), chosen against the furniture kit's still pieces (which already has the sofa, beds,
tables, kitchen units, bathtub and vanity) and the earlier batches (fridge, washer, cupboard, recliner, lamp, fan):

| Room | Models | What moves |
|---|---|---|
| Kitchen | `oven`, `dishwasher`, `toaster`, `stand_mixer`, `rice_cooker` | the oven's door and five knobs; the dishwasher's door and two racks; the lever and the toast; the mixer's head and whisk; the cooker's lid |
| Bathroom | `toilet_lidded`, `pedal_bin`, `bathroom_cabinet`, `shower` | lid, seat and flush handle; pedal and lid; two mirrored doors; a glass door |
| Bedroom | `wall_bed`, `sliding_wardrobe` | the bed comes down out of its cabinet and its legs fold out; two doors pass each other |
| Child's room | `cradle`, `rocking_horse`, `toy_chest`, `baby_mobile` | rocking; the lid; the mobile turns |
| Windows | `sash_window`, `casement_window`, `shoji_screens`, `curtains`, `venetian_blind`, `roller_blind` | the lower sash; two leaves; two paper screens on two tracks; twelve folds; every slat; the blind unrolls |
| Living room | `upright_piano`, `grandfather_clock`, `record_player`, `metronome`, `globe`, `laptop` | the key cover and top; pendulum and three hands; platter, arm and cover; the arm; the globe; the screen |
| Garden, porch, garage | `garden_gate`, `porch_swing`, `bbq`, `rotary_dryer`, `weathervane`, `wind_chime`, `garage_door` | the gate; the bench on its chains; the lid; the line's head; the vane; five tubes and a striker; the door's sections |

`roller_blind` is the projector screen's recipe and `garage_door` the roller shutter's, each with a home's sizes
in `models.lua`: a new size of a thing is a line of data.

New in how they are made:

- **Moves by the clock.** A part that goes and comes back used to time its wait from when it arrived. Parts with
  different limits arrive at different times, so a shutter's sections, an awning's strips and a screen's strips
  slowly fell out of step with each other. `model_moves.lua` gained `travel = seconds`: that long is allowed for
  each move however soon the part stops, so parts given the same numbers stay in step for good. The shutter, the
  awning and the projector screen now use it, as do the curtains, the blind, the wall bed and its legs, and any
  two parts that must take turns (the wardrobe's doors).
- **Curtains.** The strips trick sideways: each fold is a node that slides toward its side and stops where it is
  gathered, so sending all of them outward at one speed bunches the curtain up fold by fold.
- **Three more shared words** in `model_kit.lua`: `kit.carcass` (a cabinet's shell), `kit.facing` (a turned shape
  laid over to face forward: a dial, a knob) and `kit.ball` (a ball of about a hundred triangles; `mesh.sphere`
  makes about four thousand, which is too many for a knob).
- **A script made of two.** `grandfather_clock_run.lua` calls the wall clock's script for the hands and a
  `model_moves` list for the pendulum, so neither is copied.

Seen: a sheet of each, and all 34 moving prefabs in the editor in play (and the shutter, the awning and the projector screen again).

Not done: the window and door pieces are not yet what the building grammar cuts openings for (it draws its own
windows); hanging pieces (`baby_mobile`, `roller_blind`) still sit half in the floor in the studio; and the older
recipes that use `mesh.sphere` for small knobs (the espresso lever, the kiddie rides, the capsule machines, the
racks, the barber pole, the queue posts) should change to `kit.ball`.

### The sixth: gyms, clinics, offices, stations, counters, schools and bakers (2026-11-21)

Glenn: "Keep building things!" 30 more models and 60 more prefabs from 6 recipes, for kinds of place the earlier
batches had little for:

| Place | Models | What moves |
|---|---|---|
| Gym | `exercise_bike`, `rowing_machine`, `weight_machine`, `punch_bag`, `treadmill` | cranks and flywheel; seat, handle and fan; the bar down as the weights go up; the bag; the deck tilts and marks run along the belt |
| Clinic, hospital | `care_bed`, `iv_stand`, `wheelchair`, `exam_lamp` | the bed rises, its head end tilts, its rails drop; the pole; wheels and foot rests; the lamp's arm and head |
| Office | `printer`, `shredder`, `server_rack`, `flip_chart`, `paper_trimmer` | lid, paper drawer and a sheet coming out; the bin and a sheet going in; a glass door and a server on rails; the top sheet; the blade |
| Station, public hall | `flap_gate`, `departure_board`, `baggage_scanner`, `turnstile_full` | two paddles; 60 flap drums, a quarter turn at a time; rollers, curtain strips and a tray riding through; the rotor |
| Civic counter, hotel, corridor | `ballot_box`, `room_safe`, `fire_hose_cabinet`, `reception_bell`, `teller_window`, `ticket_dispenser` | a paper dropping in; door and dial; door, reel and drum; the button; a tray and a blind of strips; a ticket |
| School, library | `flip_desk`, `school_bell`, `library_ladder` | the lid; bell and clapper; the ladder rolls along its rail |
| Baker, grocer | `dough_mixer`, `deck_oven`, `shop_scales` | bowl and hook, opposite ways; three drop doors; the pan and the needle |

New in how they are made:

- **Things that keep coming.** `loop` (made for the escalator) also serves anything fed out one after another: the
  copier's sheets, the shredder's, ballot papers, queue tickets, the tray through the scanner, and the marks on the
  treadmill's belt, which are how a belt that cannot scroll is seen to run.
- **A board of drums.** The departure board's flaps are 60 small four-sided drums, each a node, stepped a quarter
  turn at a time with waits that differ a little by row and column, so a change ripples across it. It shows marks,
  not letters: text on models is not built.

Seen: a sheet of each, and all 30 moving prefabs in the editor in play. The desk bell ran without error but its quarter-second press fell
between the two readings, so it has not been seen to move.

Not done: the wheelchair, the ladder's climber and the bed do not carry a person (step 9), and the wheelchair does
not travel (#311).

### One catalog: everything placed is a prefab, and the old paths retire (agreed with Glenn, 2026-11-21)

Glenn: "the old recipes vs these new prefabs ... now sound like competing code paths and we don't want to have a
ton of dual representation." Checked against the code, and this is what was agreed.

Words: a RECIPE is a Lua script that builds a shape. A MODEL is the shape it builds (pieces, materials, moving
parts, collision shapes, markers). A PREFAB is a model plus what makes it a thing in the world. A PLACEMENT is one
prefab at one place.

**What does not compete: shapes.** The furniture kit's recipes (`furniture_models.lua`), the kerb props
(`street_props.lua`), trees and rocks are each already a model through one recipe, `prop_models.lua`, which calls
the same builders the city uses. No shape is written twice, and none should be.

**What is double today, and which side stays:**

| Thing | The older path (retires) | The one that stays |
|---|---|---|
| The list of what exists | the C++ list of kit pieces (`Piece`, `furniture_kit.h`); each system's own prop list | `models.lua` and `prefabs.lua` |
| Where a thing is solid | a kit piece's footprint box | the model's collision shapes |
| Where a person uses it | spots and verbs (`furniture_library.lua`) | the same spots and verbs, read from the prefab: a spot is a marker on the model. The verbs system stays and grows ("open", "pull"); it is not rebuilt (see "What step 9 must build on") |
| What a piece is for | family and tags in `furniture_library.lua` | the same tags, on prefabs. A room program asks for tags and gets prefabs |
| How a copy is placed | the floor planner emits a kit piece, a variant and a place (`PlacedPiece`, `BuildingMesh::furniture`) | the same list, naming a prefab |
| How a copy is drawn | the kit's batches | one drawer for placed prefabs: far copies in batches by model, near ones usable, a touched one a full entity ("How many copies would work near and far") |

So a prefab far from the player is served as its model and nothing more. It is still a prefab in the data; only
how the engine serves it changes with distance.

**A finding that makes the switch small.** The floor planner already hands back a list of placements
(`BuildingMesh::furniture`: piece, variant, transform) and the drawing is done from that list afterwards. Placing
prefabs indoors is then a change to what one entry names, not a new planner.

**Rules that follow:**

1. Anything the city places is a line in `prefabs.lua`. A kit piece becomes one through `prop_models.lua` (a line,
   no new recipe).
2. No system keeps its own list of pieces, its own collision for a piece, or its own idea of where a person sits
   on one. When a system is ported, its old list is deleted in the same change.
3. A prefab with moving parts has STATES ("idle", "running", "open"). An interaction changes the state; a state
   says what the parts do (a `model_moves.lua` list). Nothing in the game world runs a loop for its own sake. The
   `_in_use` prefabs are test rigs for the model studio and the test level, and the city never places them.
4. Tags live in one place (the furniture library's), cover every prefab, and are what room programs, parks and
   kerbs choose by.
5. A SITE (a playground, a cafe terrace, a furnished room) is a list of placements, not a prefab.

**Order:** a test level on this branch (`prefab_town`, below); states and interactions; many copies; tags. Then,
after the parent branch's agent-brain work is merged: parks and kerbs choose prefabs, then the floor planner, then
the building grammar's doors and windows, each deleting what it replaces. The last three change baked output.

### Arguments on a placement, and one palette (agreed with Glenn, 2026-11-21; not built)

Glenn, on windows of different sizes: "I like the idea of arguments." And on finishes: "we should have a palette
of a dozen colors or materials so we can have a variety but not an infinite variety."

**What it is.** A placement may carry arguments: `{ prefab = "curtains", args = { width = 2.3, height = 2.1 } }`.
The engine lays them over the prefab's own model line and asks the model store for that model. Copies with the
same arguments share one model, as copies of any model do now. Collision, markers, levels of detail and moving
parts come from the model, so they follow the size without being told.

**Rules, so it does not get muddy:**

1. **A prefab line lists what a placement may set** (`open = { "width", "height", "fabric" }`). Anything else is
   fixed by the line. An argument that changes what the thing is (the kitchen recipe's `kind`) is never open.
2. **Sizes snap.** A size is rounded to the recipe's step (10 cm unless the recipe says otherwise) before the
   model is asked for, so a street of nearly equal windows is a few models, not hundreds.
3. **Out of range is an error at the placement**, reported with the place. It is not clamped: 4 m curtains on a
   6 m window is a planner's mistake to fix, not to hide.
4. **No script or interaction hard-codes a count or a distance.** It reads the model (its nodes, its markers), as
   the escalator and the shutter scripts do.
5. **A line or an argument?** A separate prefab line when the city treats it as a different thing (other tags,
   other places it belongs: `garage_door`). An argument when it is the same thing fitted to its spot.
6. **No scaling of a built model**, and no arguments on states or behaviour: size and finish only.

**One palette.** Finishes are chosen by NAME from short lists, never as a free colour:

| List | How many | Where it is today |
|---|---|---|
| woods | 4 (oak, walnut, birch, ebonised) | the furniture kit's swatch book |
| fabrics | 8 | the furniture kit's swatch book |
| paints | 5 (red, blue, yellow, green, orange) | `playground_kit.lua` |
| metals and plain finishes | white, black, steel, brass, chrome | written again in each recipe, with slightly different greys |

The last row is a duplication found while checking: about a dozen of the new recipes each define their own
"steel" and "white". The work is to bring all four lists to about a dozen entries each IN ONE PLACE (the kit's
swatch books, which `model_kit.lua` already reads), and have every recipe's `finish` choose from it by name.

**Built (2026-11-21), the palette half.** The lists are in `furniture_models.lua` under "THE PALETTE": 8 woods, 12
upholsteries (8 cloths and 4 leathers, the leathers a material of their own, `furniture_leather`), 12 paints and 6
metals. `model_kit.lua` gives them by name (`kit.finish`, `kit.upholstery`, `kit.paint`, `kit.metal`), the playground
kit reads the same paints, and the 16 recipes that wrote their own steel, white or black now ask the palette. A
kit piece's variant still picks among the first four woods and first eight cloths only, so nothing the city
already builds has changed. Every one of the 187 models was rebuilt against it.

**A style** is then a few names (this flat: walnut, sage, brass) applied as arguments to every piece placed in
that flat or on that floor. That is how homes and offices differ without a second system, and it keeps the
number of distinct models small: a piece in four woods is four models, however many flats use them.

**Windows.** The building grammar knows each opening's width, height and sill. It passes them to whatever dresses
the window (curtains in a home, blinds in an office, a shutter on a shop: the style decides which), the way the
floor planner passes a place to a chair.

**Where it goes in the order:** with "many copies", because both are about how copies share a model. The palette
clean-up can be done before either. The proof is in `prefab_town`: windows of several sizes cut into its rooms,
each dressed from its own size, and the flat in one style and the office in another.

### The test level: prefab_town (2026-11-21)

A small place to stand prefabs in before the city does it: `assets/levels/prefab_town.json`.

- **The place** is one script, `assets/scripts/prefab_town.lua`: a street 84 m long with a pavement each side, eleven
  rooms along it (floor, four walls, a doorway to the street, no ceiling), a park and a paved square. It makes no
  furniture.
- **Everything else is a prefab**, 286 of them, one level entity each: a konbini, a cafe, an office, a gym and a
  station hall on one side; a flat in three rooms (kitchen and living room, bedroom and bathroom, child's room), a
  clinic, a schoolroom with a library corner and a bakery on the other; kerb pieces along both pavements; a
  playground and garden things in the park; a fountain, an escalator, doors, windows and seats on the square.
- **Both come from one table**, in `tools/prefab_town.py`: each room's or site's list of (prefab, across, toward the
  door, facing, height). Run it and it writes the level. Edit the table, not the level file.
- **Two levels from the one table.** `prefab_town` has every piece still, as the city would place it.
  `prefab_town_busy` has each piece's `_in_use` prefab where there is one, every moving part working at once: a
  load test, and the only place those prefabs are placed.
- 22 more of the furniture kit's pieces became prefab lines for it (counters, worktops, tables, beds, bookcases),
  each through `prop_models.lua`: rule 1 above, in use.

What it showed:

- **It loads in 2.2 seconds with no errors, and is 5,376 entities for 286 prefabs.** One stocked drinks fridge is
  566 drawn pieces, because every bottle is a piece of its own. That is the case for "many copies" in one number:
  a stamped prefab is right for a crane and wrong for a shop.
- In play, with every script running, no script errors; of 74 moving parts sampled, 46 had moved in four seconds
  (the rest were waiting their turn). Frame time was not measured: the viewer has no command that reports it.
- Placing by hand works, and is exactly as tedious as expected: a facing or a height wrong by eye, found in a
  picture. That is the argument for sites and the floor planner placing prefabs from tags.

What it is not: the rooms are stand-ins, not rooms grown by the building grammar, and nothing here is chosen by
the city. To see it: `./build-viewer/viewer assets/levels/prefab_town.json --play`.

### What step 9 must build on: the furniture kit already has seats and verbs

Found while choosing these (not known when the markers above were named): `assets/scripts/furniture_library.lua`
already gives each kit piece SPOTS (places a body goes) and VERBS (how a body uses them: `sit`, `lie`, with where
the eye goes and where the person stands up), and the player uses them with E. That is an interaction system for
furniture, in data, in use. Step 9's "offers at markers" must be that system extended, with a marker as a spot and
new verbs beside sit and lie (pull, press, open, push), and not a second one. The `seat`, `handle_N`, `lever`,
`control` markers on the pieces above are the hooks for it; none is wired to it yet.

### Step 9, read before building: four things the engine already has (found 2026-11-21)

Read in the code before writing any of step 9. Each of these is the system to extend; none is to be built again.

1. **One owner of the E key: the interaction broker** (`engine/interact_broker.h`). Every usable thing OFFERS what
   it can do from where the player stands (`offerInteraction`), the broker picks the one in focus and hands a
   command back to the provider that offered it (`takeInteractCommand`). Providers today: furniture, elevator,
   train, vehicle, ladder, carry, playground. Prefab interactions are ONE MORE PROVIDER ("prefab"). Nothing else may
   read E.
2. **Sitting and lying** (`engine/interaction.h`, `systems/interaction_system.cpp`). The `Seated` component on the
   player holds the eye, the way they face and where they stand up; the player system does the rest. A prefab's
   seat marker should put the same `Seated` on the player. `Seated` names a furniture set today, so it needs to be
   able to name a prefab copy instead; that is the one change to it.
3. **The furniture sets are already "many copies"** (`Interactables`): one entity for a whole interior's pieces,
   each piece a transform and a record of which spots are taken, "so thousands of chairs cost nothing until
   someone stands next to one". That is the "ready" state of the many-copies design, built. Step 10 extends it to
   prefabs; it does not invent it.
4. **A swing the player rides, and carrying** (`engine/playground.h`: `PlaySwing`, `MonkeyBars`;
   `systems/carry_system.h`). The player can already sit on a swing that swings by its own weight and die down,
   and pick up, carry and put down loose things. The playground prefab's seats should BE `PlaySwing`s (its seat
   node following the swing's angle), and a thing a person can pick up should be the carry system's `Carryable`.

Also already data: the furniture library has, per piece, a family, tags, spots, verbs, ANCHORS (a surface that
takes small things: a nightstand's top takes a lamp) and CLEARANCE (the floor kept free in front), and room
programs (`furniture_rooms.lua`) choose pieces by family and tags. All of it is keyed by the kit's C++ list of
pieces (`Piece`). "One catalog" means that key becomes a prefab's name; the rest of that system stays.

**The shape step 9 takes from this:**

- A prefab line gains `uses`: each names a marker on the model, the word shown ("open", "sit", "switch on") and
  what it does: move a node between two values (a door, a drawer, a desk's height), seat the player (`Seated`), or
  put the prefab in a named STATE for a time (a washing machine running: the list of what its parts do in that
  state is the `model_moves.lua` list its test rig has now).
- One system, the "prefab" provider, offers the uses of prefab copies within reach and acts on the broker's
  commands. It moves parts through `engine/model_motion.h`, as a behaviour script does.
- A use is offered only for a marker the model has, so prefabs with arguments (more or fewer seats) need no
  special case.

### Step 9, first part, built: uses on a prefab (2026-11-21)

Glenn: "make all those prefabs interactive (at least the ones that ought to be: sit on chairs, exercise bikes,
raise desk, open cupboards, laundry machine)."

A prefab line may now say `uses`: what a person standing at a copy can do with E.

```lua
washing_machine = { model = "washing_machine", physics = { body = "static" },
  uses = { { at = "handle", say = "open", back = "close", move = { door = 130 }, speed = 110 },
           { at = "controls", say = "switch on", back = "switch off", run = { drum = 300 }, seconds = 40 } } },
```

| Word | What it means |
|---|---|
| `at` | a marker on the model: where the offer shows and how near a person must stand |
| `say`, `back` | the word offered, and the word offered once it is done. No `back`: it can only be done again (a button) |
| `move`, `speed` | parts and where each goes (degrees or metres); done again, each returns to 0 |
| `run`, `seconds` | parts that keep turning while it is on; with `seconds` it stops by itself. This is a STATE with a time: a wash that runs and ends |
| `sit` | the person sits at the marker, and is carried with it if its part moves |

How it is built, all on what was there:

- **One more provider on the broker.** `PrefabUseSystem` (`systems/prefab_use_system.h`) offers each use whose
  marker is within reach and acts on the broker's command. The on-screen prompt, Tab between offers and the E key
  are the broker's, unchanged.
- **Sitting is the furniture's `Seated`**, naming the prefab copy as its set. The furniture's own system leaves
  such a seat alone. While seated the eye follows the marker, so a rocking chair carries its sitter.
- **Parts move through `model_motion.h`**, the same calls a behaviour script makes.
- **What a copy keeps** is one small component (`PrefabUses`: which uses are on, and time left). A use whose
  marker or part the model does not have is left out when the copy is stamped, so a line serves a model built
  with more or fewer of a thing.
- `doPrefabUse(world, copy, index)` (`engine/prefab_use.h`) is the one call. The player makes it through the
  broker; a city person or a script can make the same call.

123 prefabs have uses. They were proposed by rule from each model's parts and markers (a part with a handle
marker opens; a part with no limits is switched on; a seat marker is sat on), then about 40 were written by hand
(the desk, the chair, the wall bed, the gym, the bathroom, the laundry, doors that open themselves and shut).

Seen in `prefab_town` in play, through the control socket's `interact`: the fridge's door opened and its prompt
moved to the freezer; the laptop opened; the rocking chair seated the player (the eye went to the seat and the
offer became "stand up"); the washing machine switched on and offered "switch off". Tests:
`tests/test_prefab_use.cpp` (a line read and written back; uses done on a stamped copy; a timed use ending).

**The kit's chairs, sofas and beds, the same day.** Glenn: "make the kit chairs, sofas and beds sittable next."
Two rules did it, and no line in `prefabs.lua` changed:

- **A seat is a seat.** A marker whose role is "seat" or "lie" offers sitting or lying on every prefab whose model
  has one, unless the line's own uses name that marker. `lie = true` is the lying use: the eye low over the
  marker, looking up and along it.
- **The furniture library's description becomes markers.** `furniture.uses{ name, variant }` gives a recipe the
  library's verbs for a kit piece, and `prop_models.lua` writes each as a seat or lie marker placed so the eye
  comes out where the library says. So `sofa` has three seats and one place to lie, `bed` a place to lie and two
  edges to sit on, from the ONE description the city's interiors already use. This is the first piece of "One
  catalog" in code: a spot is a marker.

Seen in `prefab_town`: sat on the sofa (eye 1.25 m, the offer became "stand up"); lay on the bed from its side
(eye 0.87 m, "get up"). A seat is offered without the broker's sight test, because a kit piece's collision shape
is its whole footprint box and the seat is inside it.

Not carried over from the library: a spot being TAKEN (two people cannot yet be refused the same seat, and lying
on a sofa does not need its three seats free), and "stand" verbs (a lectern). Both matter when city people use
prefabs. The view direction when lying was not checked by eye.

Not done, in the order I would do them:

- **Riding.** Sitting on the exercise bike does not pedal it; "pedal" is a second press. A use that runs while
  someone sits is a small addition (`while_seated`).
- **The swing, the seesaw and the roundabout are not physical.** Sitting works; the swing should become the
  engine's own `PlaySwing` (see "four things the engine already has").
- **Only the player.** City people do not use prefabs yet.
- **Many of the rule-made uses have not been tried one by one.** Words like "pull" on a part that should say
  "press" are likely; each is a one-word fix in `prefabs.lua`.
- **The editor does not show or edit uses** (its rewrite of a line keeps them).
- Lockers, the departure board and other things with many like parts have no uses: one marker per part is needed.

### How many copies would work near and far (agreed with Glenn, 2026-11-21; not built, this is step 10)

Glenn: "If you're on a floor with a ton of desks and chairs then all that stuff should work." And on what is
remembered: "if you leave a floor and return it should just be reset."

Everything placed is a prefab, always. A PLACED COPY is one prefab at one place in one pose (the values of its
moving parts). What changes with distance is how the engine serves a copy, not what it is:

| State | What the engine does for the copy | Where |
|---|---|---|
| drawn | draws it in a batch with every other copy of its model; nothing else | other floors, across the street, far away |
| ready | still drawn in the batch, and its markers can be found and used; solid as its model's shapes | every copy on the floor the player is on |
| awake | a full prefab entity: its own moving nodes, moving collision, its interaction or script running | the one just touched, the few a person nearby is using |

- A copy WAKES when a person (the player or a city person near the player) uses it, at the pose the batch was
  drawing it in, and SLEEPS when it has been still a while, in the pose it was left in. The batch draws a sleeping
  copy in that pose (each moving part is its own drawn copy, as the kit cars' wheels are).
- Both ways of serving a copy read the ONE model and the ONE pose. Neither keeps its own copy of a shape or a
  collider, or the hand-off shows.
- **A pose lasts only while the floor is loaded.** When the floor's contents unload, every pose on it is forgotten
  and the floor comes back as placed. So there is no table of changes to keep, no stable name needed for a copy
  across unloads, and nothing extra in a saved game. A thing that should be in some pose for a reason (a shop's
  shutter down at night) gets that pose from the reason (the shop's hours), not from memory.
- A prefab says in its own line whether a part returns by itself (a lift's doors, a bin's flap) or stays where it
  was put while the floor is loaded (a raised desk, an open drawer).
- The levels of detail a model already has (its far shape from 20 to 60 m) are about drawing only, and are separate
  from the three states.
- This is the pattern the city already uses for people and cars (a line in a schedule far away, a body near the
  player). Build it as part of that, not as a second system.

Open, for when it is built: what "leaving the floor" means exactly, so a quick trip up one storey and back does not
reset a desk raised a minute ago (tie it to the contents actually unloading, which has slack).

Left from what was agreed: the field declarations are written in C++ next to the reader (`prefabFieldsJson`), not
declared by each component kind in data; that comes with step 9, when there is more than one kind to declare.
Model rows in the Inspector (read-only facts) are not there: a picked piece shows the entity's ordinary properties.
The body's fixed-or-moving choice has only "static" and "none" because the engine has no other prefab body yet.

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
