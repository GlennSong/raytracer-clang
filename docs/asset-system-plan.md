# Asset System — Plan (ROADMAP 3.1)

The engine-side owner of GPU-backed resources — meshes today, materials and
textures as 3.2 lands, fields/animation later. It is the keystone of the procgen
substrate (ADR-0021): the thing every generator emits *into* and every consumer
references *by handle*. It also closes a standing tech-debt item — the editor
mesh-upload leak.

**Builds on:** `Handle<Tag>` / `SlotMap<T>` (ADR-0007 — already migrated; the
Metal backend already stores GPU meshes in a `SlotMap<GPUMesh, MeshTag>` and
exposes `uploadMesh`/`removeMesh`/`getMeshBounds`). The `AssetManager` is the
*owner* that ADR-0007's revisit trigger anticipated.

---

## The problem it solves

Today resource lifetime is hand-managed and leaks:

- `MeshBuilder::box(size)` → `RenderMesh` (CPU data) → `Renderer::uploadMesh()` →
  `MeshHandle` (GPU). Call sites: `editor_system.cpp` (add primitive, size-edit
  re-upload, duplicate), `camera_system.cpp` (gizmo), the level loader.
- `Renderer::removeMesh()` **exists but is essentially never called.** So:
  - A **size edit** (`editor_system.cpp:198`) overwrites `Renderable.mesh` with a
    fresh upload and leaks the old GPU mesh.
  - An **edit↔play cycle** tears down and rebuilds the world's Renderables
    without freeing their meshes.
  - **Reloading a level** re-uploads every primitive from scratch — including N
    identical boxes that should share one GPU mesh.

Net: GPU memory grows across mode switches and edits (TECH_DEBT: "Editor mesh
re-uploads leak"). The renderer *can* free; nobody *owns* the decision to. That
ownership is the AssetManager.

## Design

The `AssetManager` is **engine-side and backend-neutral**: it holds a
`Renderer&` and drives the GPU through the existing seam (ADR-0001) — it never
touches Metal. It is the single place that decides when a GPU resource is
created and destroyed.

Three responsibilities:

1. **Dedup / caching by key.** Identical content uploads once. A primitive's key
   is its shape + size (`"box:1,2,1"`); a loaded mesh's key is its file path; a
   generated mesh's key is a caller-supplied id (or "no key" = always unique).
2. **Refcounted lifetime.** `acquire` bumps a refcount and returns the shared
   handle; `release` drops it; the GPU resource is freed (`removeMesh`) when the
   count hits zero. `clear()` frees everything (world teardown).
3. **Metadata custody.** Bounds (`getMeshBounds`) and the source key live with
   the record, so the editor's picking/AABB path stops re-querying the backend
   per entity.

### Asset kinds

One manager, parameterized per resource type so 3.2 slots in without a redesign:

```
AssetManager
  meshes    : Registry<MeshHandle,    MeshSource>     // 3.1, now
  materials : Registry<MaterialHandle, MaterialDesc>  // 3.2
  textures  : Registry<TextureHandle,  TextureSource> // 3.2
  // fields / animation clips: later (ADR-0021 value types)
```

`Registry<H, Source>` is the reusable core: a `SlotMap`-backed table mapping a
content **key → handle**, plus per-entry refcount, source, and metadata. Mesh,
material, and texture registries differ only in their upload/free thunks (the
renderer calls) — the dedup + refcount machinery is shared.

### API sketch

```cpp
// Backend-neutral; constructed with the renderer it uploads through.
class AssetManager {
public:
    explicit AssetManager(Renderer& renderer);

    // --- meshes (3.1) ---------------------------------------------------
    // Acquire a primitive by shape+size; identical requests share one GPU
    // mesh (dedup by key) and bump its refcount.
    MeshHandle acquirePrimitive(const std::string& shape, Vec3 size);

    // Acquire a generated/loaded mesh. `key` empty = always a fresh upload
    // (no dedup); non-empty = cached and shared (procedural meshes that want
    // reuse, or a glTF path).
    MeshHandle acquireMesh(const RenderMesh& mesh, const std::string& key = "");

    // Release one reference; frees the GPU mesh when the last ref drops.
    void releaseMesh(MeshHandle handle);

    // Cached bounds (no per-frame backend round-trip).
    BoundingSphere meshBounds(MeshHandle handle) const;

    // Free every owned resource (world teardown / edit<->play swap).
    void clear();

    // --- materials / textures (3.2) ------------------------------------
    // MaterialHandle acquireMaterial(const MaterialDesc&);  // later
    // TextureHandle  acquireTexture(const TextureSource&);  // later
};
```

`Renderable.mesh` stays a `MeshHandle` (no component change). What changes is
*who hands it out*: call sites move from `renderer.uploadMesh(...)` to
`assets.acquirePrimitive(...)` / `assets.acquireMesh(...)`, paired with
`releaseMesh` on overwrite/destroy.

### How it fixes the leak (concretely)

- **Size edit** (`editor_system.cpp:198`): `releaseMesh(old)` then
  `acquirePrimitive(shape, newSize)`. Old refcount→0→`removeMesh`. No leak.
- **Add / duplicate primitive**: `acquirePrimitive` — duplicates of the same
  shape+size now *share* one GPU mesh (refcount 2) instead of two uploads.
- **Edit↔play swap / level reload**: the world's owner calls
  `assets.clear()` (or releases per-entity on world clear); the rebuild
  re-acquires, hitting the dedup cache so identical primitives upload once.

### Ownership & placement

`Application` owns the `AssetManager` (it already owns the `Renderer`), exposed
on `FrameContext` next to `renderer`. Lifetime is the app's; `clear()` is the
explicit teardown hook the world-rebuild paths call. Entities reference assets
only by handle — no raw GPU ownership escapes the manager.

### Procgen path (ADR-0021)

A generated mesh takes the **same** path as a loaded one — the design
principle. A generator produces a `RenderMesh` (via the 3.3 mesh builder) and
calls `acquireMesh(mesh, key)`; there is no separate "procedural upload." The
manager is the value-type custodian the substrate emits into.

### Async (deferred seam, not built)

Loading from disk / generating on a worker is a later need (ROADMAP 3.1). The
seam: `acquire*` may return a handle to a **pending** asset (a placeholder GPU
resource) while a `JobSystem` (ADR-0014) task produces the `RenderMesh`
off-thread; the GPU upload itself stays on the render thread (Metal command
submission is not thread-safe). Designed for, not implemented in the first cut —
the synchronous API above is a strict subset, so adding async is additive.

## Phases

- **3.1a — Mesh ownership + dedup + leak fix.** `AssetManager` + mesh
  `Registry`; migrate the `uploadMesh` call sites (editor, camera gizmo, level
  loader) to `acquire`/`release`; wire `clear()` into world teardown. Closes the
  leak. Pure engine-side; the GPU calls stay behind the renderer seam, so the
  logic is **Linux/CI-testable** with a mock/stub renderer.
- **3.1b — Materials + textures (with 3.2).** Add the material/texture
  registries; procedural textures (Tier 4 Phase D) acquire through the same
  path.
- **3.1c — Async loading.** The `JobSystem`-backed pending-asset seam above.

## Testing (Linux-first)

The manager is backend-neutral, so a **stub `Renderer`** (counts
`uploadMesh`/`removeMesh`, hands out fake handles) makes the whole thing
headless-testable, like the rest of engine_core:
- acquire twice with the same key → one upload, refcount 2, one handle;
- release to zero → exactly one `removeMesh`;
- size-edit pattern (acquire new, release old) → net zero leaked uploads;
- `clear()` → every outstanding mesh freed exactly once;
- empty-key acquire → always a fresh upload (no dedup).

A `tests/test_asset_manager.cpp` over the stub renderer pins all of the above.

## Open questions

- **Key scheme:** plain strings (simple, debuggable) vs a typed key / content
  hash (collision-proof, opaque). Lean strings for 3.1a; revisit if procedural
  keys get unwieldy.
- **Refcount vs scope-GC:** explicit refcounting (chosen) vs a sweep that frees
  assets no live `Renderable` references at clear time. Refcount is more precise
  and enables share-on-duplicate; the sweep is a fallback if discipline proves
  error-prone.
- **Who calls `clear()`:** the level loader on (re)load vs a world-owned RAII
  scope. Prefer tying it to the world-rebuild path so edit↔play is automatic.
