# Node Graph — Plan (procgen "language", ADR-0021 Phase C)

The presentation layer the substrate was built for: author generators as a
**graph of composable nodes (data)** instead of C++. "A rock is a generator
graph, not rock.cpp" (ADR-0022). We've now passed ADR-0021's trigger — four
generators exist (terrain, L-system trees, SDF rocks, scatter) across the
grammar / field / frame paradigms — so we know what the nodes must express.

**Scope:** the *geometry* graph (Mesh / Field / Material / Frame). Shader graphs,
particle graphs, and bullet-pattern graphs are *separate graph types* for later
(ADR-0021) — do not conflate them with this one.

**Guiding principle (from the whole project):** the **graph model + evaluator
are pure data/logic — build and unit-test them headless on Linux first**; the
visual editor is a thin presentation layer on top, added on macOS. Don't start
with the UI.

---

## The value types (the wires)

Nodes pass a small, tagged set of values — exactly the substrate's types:

| Value | Backed by | Examples |
|---|---|---|
| `Scalar` / `Vec3` | `double` / `Vec3` | radius, seed, position, color |
| `Field` | `Sdf` (`function<double(Vec3)>`) | sphere SDF, noise, smooth-union |
| `Mesh` | `RenderMesh` | polygonized field, primitive, L-system |
| `Frames` | `std::vector<Transform>` | scatter output, instancing |
| `Material` | `RenderMaterial` | albedo/roughness (+ procedural inputs) |

A connection is valid only if output type == input type (simple type check).
This is the "node graph over Mesh/Field/Frame/attributes" ADR-0021 named.

## The nodes (almost all wrap code we already have)

The node library is thin — each node calls an existing substrate function:

- **Inputs/params:** `ScalarParam`, `Vec3Param`, `Seed`, `ColorParam`.
- **Fields:** `SdfSphere`/`Box`/`Capsule`, `Union`/`SmoothUnion`/`Subtract`/
  `Intersect`, `Noise`/`FBM` (→ a `Field`), `Displace` (field + noise).
- **Field→Mesh:** `Polygonize` (`polygonizeSdf`), `Heightfield` (`generateTerrain`).
- **Mesh:** `MeshPrimitive` (`MeshBuilder::box/sphere/...`), `Transform`,
  `Merge`, `RecomputeNormals`, `BakeHeightColor`.
- **Grammar:** `LSystem` (expand) + `Skin` (turtle → capsules → field/mesh).
- **Frames:** `Scatter` (`scatterOnTerrain`) → `Frames`; `Instance` (Frames +
  Mesh → an InstanceGroup-ish output).
- **Output:** `MeshOutput` (+ optional `MaterialOutput`) — what the generator
  produces.

So "rock" = `SdfSphere → SmoothUnion(lumps) → Subtract(cuts) → Polygonize →
MeshOutput`; "tree" = `LSystem → Skin → Polygonize → BakeHeightColor`. These are
the C++ generators we already wrote, redrawn as graphs.

## The graph model + evaluator

- **Data-driven node registry** (mirrors `ComponentRegistry`): one registration
  per node type — its typed input/output sockets and an `evaluate(inputs) ->
  outputs` thunk. New node = one registration; the editor and serializer never
  name concrete nodes (same payoff the property layer gave the inspector).
- **Graph** = nodes + connections (a DAG). **Pull-based evaluation:** evaluate
  the output node, recursively pull upstream inputs, **memoize** each node's
  result. Deterministic — `Seed` is just an input that flows through.
- **Caching/dirty:** editing a param dirties that node and its descendants;
  re-evaluate only those. (Phase 3 nicety; Phase 1 can re-eval whole graph.)

## Serialization — the graph *is* the asset

A graph saves to JSON (nodes, params, connections) — the authorable, round-
tripping "generator file" (`assets/generators/oak.graph.json`). This is exactly
ADR-0022's realness spectrum:
- **runtime:** the level references a graph; it's evaluated at load (today's
  inline params become a graph reference).
- **baked:** evaluate offline → a mesh/material asset on disk.
- **editable:** a graph instance with exposed params, tuned in the editor.

Exposed inputs make one graph parametric — "rocks of many sizes" is one `rock`
graph with a `radius`/`seed` input, not many files.

## The editor UI (macOS; the one real new decision)

A node canvas: add nodes, drag connections, edit params, **live-preview the
output mesh in the viewport**, save/load graph assets. Options:
1. **Hand-rolled ImGui canvas** — ImGui is already vendored (ADR-0011); draw
   nodes/wires with draw-lists. Most control, no new dep, but node editors are a
   lot of UI work.
2. **Vendored ImGui node-editor** (e.g. imgui-node-editor) — fastest to a good
   UX, but a **new dependency → needs an ADR** (AGENTS.md).
3. **Qt `QGraphicsView`** in the editor shell — native, powerful, but a second
   UI stack for this feature.

Recommendation: decide at Phase 3. The graph engine (Phases 1–2) is identical
regardless, so this choice doesn't block the valuable, testable core.

## Phases

- **Phase 1 — Graph engine (headless, Linux-tested).** Value variant, node
  registry, DAG + pull evaluation + memoization, JSON (de)serialization. Port
  the **rock** and a simple **SDF shape** as graphs; a test asserts the graph
  output matches the hand-written `generateRockSdf` (proof the model is
  sufficient). *Deliverable: a `.graph.json` evaluates to a `RenderMesh`.*
- **Phase 2 — Graphs as level assets.** The loader can take a generator graph
  (by path) for a vegetation/prop species and evaluate it, alongside the
  existing inline params. Exposed inputs + seed drive variants. C++ generators
  remain as fast built-ins. *All Linux-testable.*
- **Phase 3 — The node editor UI.** The chosen canvas: author/preview/save
  graphs in the editor. **macOS.**
- **Phase 4 — Library + ergonomics.** More nodes, subgraphs/groups, a node
  palette, and promoting the L-system/terrain/scatter generators to graph nodes
  so whole scenes are authorable.

## Risks / honesty

- **`Field` is a `std::function`** — cheap to pass between nodes, but deeply
  nested graphs build deeply nested closures; fine at our scale, revisit if a
  graph gets huge.
- **Type system stays minimal** (5 value types, exact-match connections) — no
  generics/auto-convert at first; add coercions (Scalar→Vec3) only as needed.
- **The UI is the iceberg.** The engine (Phases 1–2) is bounded and testable;
  the editor (Phase 3) is where the effort and the macOS-only verification live.
  Sequencing the engine first means we get authorable, reusable generators (run
  at load or baked) *before* committing to UI work.
- **Don't over-unify.** This is the geometry graph only. Resist folding shaders/
  particles/bullets in — they're separate graph types (ADR-0021).

## Why now

Every node is a thin wrapper over a function we've already written and tested
(SDF, noise, MeshBuilder, L-system, scatter, polygonize). The graph engine turns
that pile of generators into a *composable, authorable, serializable* system —
which is the entire point of the substrate (ADR-0021/0022). Phase 1 is mostly
plumbing over proven parts.
