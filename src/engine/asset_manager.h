#ifndef RAYTRACER_ENGINE_ASSET_MANAGER_H
#define RAYTRACER_ENGINE_ASSET_MANAGER_H

#include "mesh_uploader.h"
#include "../rt_math.h"
#include <map>
#include <string>
#include <unordered_map>

namespace engine {

// Owns the lifetime of GPU-backed meshes (ROADMAP 3.1, docs/asset-system-plan.md).
// Loaders and generators acquire meshes by content; identical content (same key)
// is uploaded once and shared via a refcount, and freed when the last reference
// is released. It is the single place that decides when a GPU mesh is created or
// destroyed — which is what closes the editor mesh-leak (the renderer's
// removeMesh existed, but nothing owned the call). Backend-neutral: it drives a
// MeshUploader seam, so it is fully testable headless against a stub.
class AssetManager {
public:
    explicit AssetManager(MeshUploader& uploader) : uploader_(uploader) {}

    // Acquire a primitive by shape + size. Identical requests share one GPU mesh
    // (dedup by "shape:x,y,z"); the mesh is only built (MeshBuilder) and uploaded
    // on a cache miss. Refcount is bumped on every call.
    MeshHandle acquirePrimitive(const std::string& shape, Vec3 size);

    // Acquire a loaded/generated mesh. A non-empty key dedups and shares as
    // above; an empty key always uploads a fresh, unshared mesh (a one-off
    // generated mesh with no reuse). Refcount is bumped on every call.
    MeshHandle acquireMesh(const RenderMesh& mesh, const std::string& key = "");

    // Release one reference; the GPU mesh is freed (MeshUploader::removeMesh)
    // when its refcount reaches zero. A null or unknown handle is ignored.
    void releaseMesh(MeshHandle handle);

    // Cached bounds for a live mesh (no backend round-trip); null for unknown.
    BoundingSphere meshBounds(MeshHandle handle) const;

    // Free every live mesh (world teardown / edit<->play swap). The manager owns
    // nothing afterward, and the dedup cache is reset.
    void clear();

    // --- introspection (tests, debug overlays) ---
    std::size_t liveMeshCount() const { return records_.size(); }
    int refCount(MeshHandle handle) const;

private:
    // The seam for LOD chains (a logical asset = several MeshHandles by detail):
    // 3.1a keeps one handle per record, but the key→record split is where a
    // `std::vector<MeshHandle> lods` lands later without touching consumers.
    struct MeshRecord {
        MeshHandle handle;
        BoundingSphere bounds;
        int refs = 0;
        std::string key;   // empty = un-keyed (no dedup entry)
    };

    static std::string primitiveKey(const std::string& shape, Vec3 size);

    MeshUploader& uploader_;
    std::map<MeshHandle, MeshRecord> records_;           // by handle: release/bounds
    std::unordered_map<std::string, MeshHandle> byKey_;  // dedup: key -> handle
};

}  // namespace engine

#endif
