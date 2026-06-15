# Procedural assets — the `TreeAsset` shape (design sketch)

A concrete sketch for ADR-0026, to pressure-test *before* building. It pins what
a generator outputs, how a recipe is authored in Lua, how an instance references
it, and where the engine pieces live. Nothing here is built yet; it exists to be
argued with.

See ADR-0026 (the decision), ADR-0021/0022 (substrate + realness spectrum),
ADR-0025 (Lua authoring), and `docs/lsystem-botany-plan.md` (the botany roadmap).

## The four layers (recap)

```
Operations (C++)   growTree, lsystem, sdf, mesh ops          — the systems
Recipe (Lua asset) assets/recipes/oak.lua  = params + compose — "a species"
Instance (data)    level/editor entity: recipe ref + xform + seed + overrides
Baked (optional)   frozen mesh/rig on disk
```

The split that matters: **C++ defines systems, Lua defines recipes, data places
instances.** Today's `shape:"tree"` JSON block collapses recipe+instance and
skips Lua — ADR-0026 removes it.

## What a generator emits — `TreeAsset`

Not a bare `Mesh`. A multi-output asset whose skeleton *is* the rig (built free
from the L-system node tree we currently discard):

```cpp
struct Bone {
    int   parent;        // -1 for root
    Vec3  restHead;      // node positions in the L-system skeleton
    Vec3  restTail;
    float radius;        // pipe-model radius (drives collision + skinning falloff)
    float stiffness;     // 1 at trunk -> ~0 at twig tips; wind/sway weight
};

struct SkinnedMesh {
    RenderMesh           mesh;        // positions/normals/tangents/UVs as today
    std::vector<uint8_t> boneIndex;   // up to N influences per vertex (N=2..4)
    std::vector<float>   boneWeight;  // bound to the nearest branch bone(s)
};

struct TreeAsset {
    SkinnedMesh            bark;       // skinned to the skeleton
    SkinnedMesh            leaves;     // leaf cards, skinned to twig bones
    std::vector<Bone>      skeleton;   // the rig (and the wind animation target)
    // Gameplay collision: a capsule per major limb (cheap, static), NOT a
    // triangle soup. Below a radius/order threshold, twigs carry no collider.
    std::vector<CapsuleCollider> collision;
    MaterialRef            barkMaterial;   // recipe-named, resolved by asset mgr
    MaterialRef            leafMaterial;
};
```

Why each piece:
- **skeleton** — reused as the wind rig; no separate rigging step.
- **skin weights** — bark/leaf vertices follow their bone, so animating the
  skeleton sways the mesh.
- **stiffness per bone** — the one wind parameter that makes trunk rigid and
  twigs floppy; also usable by a vertex-shader wind path with no rig.
- **capsule collision** — gameplay collision (bump/shoot) decoupled from the
  render mesh; cheaper and more robust than `MeshShape` for a busy canopy.

## Engine pieces this implies (each its own step)

- **Skinned-mesh render path** — a vertex skinning stage (bone matrices ->
  vertex transform). New: a `Skeleton` component + per-instance bone palette.
- **`AnimatorSystem` / wind** — drives bone transforms (see cost ladder below).
- **Collision** — a multi-shape (compound capsule) collider, not just the
  existing `MeshCollider`.
- **`ProcgenSource` component** — `{ recipeAssetId, seed, paramOverrides }`,
  making the instance a real document entity (selectable, re-runnable) — the
  editor iteration surface (ADR-0022 editable-instance tier).
- **Recipe asset + loader** — load a Lua recipe, read its declared param schema,
  evaluate `build(params, seed) -> TreeAsset`.
- **Lua bindings** over the multi-output generator.

## Wind — the cost ladder (decide per project, not now)

1. **Vertex-shader wind** — no rig, no physics; sway from per-vertex stiffness
   (bake `Bone.stiffness` into the vertex). The shipping default (SpeedTree-style).
2. **Rig + procedural wind** — `AnimatorSystem` sways bones with layered noise;
   mesh follows via skinning. Allows interaction (push, impacts).
3. **Rig + Jolt joints** — a body + spring/cone-twist per bone; wind applies
   forces. The literal "physics object for sway"; expensive, rarely per-branch.

All three are *orthogonal* to the static capsule collider, which stays for
gameplay regardless.

## Recipe asset — Lua (`assets/recipes/oak.lua`)

A recipe declares a **param schema** (drives the editor inspector) and a `build`
that composes operations into a `TreeAsset`:

```lua
return {
  name = "oak",
  params = {                              -- schema: editor reads this
    iterations   = { type="int",   min=3, max=7, default=6 },
    branchAngle  = { type="float", min=10, max=60, default=35 },
    radiusScale  = { type="float", min=0.5, max=3, default=1.4 },
    barkColor    = { type="color", default={0.32,0.23,0.15} },
    leafColor    = { type="color", default={0.20,0.46,0.14} },
  },
  build = function(p, seed)              -- p = resolved params, seed = variant
    return tree.grow {                   -- C++ growTree, bound -> TreeAsset
      iterations   = p.iterations,
      branchAngle  = p.branchAngle,
      radiusScale  = p.radiusScale,
      barkColor    = p.barkColor,
      leafColor    = p.leafColor,
      seed         = seed,
    }
  end,
}
```

C++ stays the *system* (`tree.grow`); Lua is the *recipe* (which params, what
defaults, how composed).

## Instance — level/editor data

References the recipe asset; carries only placement + variant + overrides:

```json
{
  "name": "oak-01",
  "recipe": "recipes/oak.lua",
  "position": [-15, 0, -8],
  "seed": 7,
  "params": { "radiusScale": 1.6 },
  "wind": "vertex"
}
```

The same data is what the editor writes when you place and tweak a tree; "build
by script" (edit `oak.lua`) and "build in editor" (edit these fields) are the
same path.

## Pressure-test these before building

1. **Skinning influences** — is 2 bones/vertex enough for branch forks, or do
   we need 4? (Cost vs. crease quality at junctions.)
2. **Collision granularity** — capsule per limb down to which branch order?
   Too deep = many shapes; too shallow = you shoot "through" mid branches.
3. **Recipe ↔ schema** — is the schema declared in Lua (above) or in a sidecar?
   The editor and the baker both need it without running `build`.
4. **Baking** — does a baked `TreeAsset` keep the skeleton (animated static
   prop) or collapse to a static mesh (no sway)? Probably both, as LODs.
5. **Generality** — does `TreeAsset` want to be a generic `ProcAsset`
   (mesh(es) + skeleton + collision + materials) so a building/creature reuses
   it? ADR-0026's revisit trigger says validate against a second asset first.
6. **Wind without a rig** — if the default is vertex-shader wind, we still need
   `Bone.stiffness` baked per vertex; confirm that path needs no runtime skeleton.
