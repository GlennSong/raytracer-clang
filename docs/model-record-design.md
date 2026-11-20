# Models and prefabs: design note (#191 step 5)

Draft 2 for Glenn, 2026-11-19. Nothing here is built. Draft 1 put scripts, lights, sounds and offers inside the
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
    name             unique in the model: "body", "door_fl", "boom_2"
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
