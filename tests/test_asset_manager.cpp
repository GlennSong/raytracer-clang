#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include <cstdint>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// Counts uploads/removes and hands out incrementing fake handles, so the
// manager's dedup + refcount + lifetime logic is verifiable with no GPU backend.
struct StubUploader : MeshUploader {
    int uploads = 0;
    int removes = 0;
    uint32_t next = 1;

    MeshHandle uploadMesh(const RenderMesh&) override {
        uploads++;
        return MeshHandle{next++, 1};   // index, generation (valid)
    }
    void removeMesh(MeshHandle) override { removes++; }
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

}  // namespace

TEST_CASE(asset_manager_dedups_by_key) {
    StubUploader up;
    AssetManager assets(up);
    MeshHandle a = assets.acquireMesh(RenderMesh{}, "box:1,1,1");
    MeshHandle b = assets.acquireMesh(RenderMesh{}, "box:1,1,1");
    CHECK(a == b);                       // shared handle
    CHECK(up.uploads == 1);              // uploaded once
    CHECK(assets.liveMeshCount() == 1);
    CHECK(assets.refCount(a) == 2);
}

TEST_CASE(asset_manager_frees_on_last_release) {
    StubUploader up;
    AssetManager assets(up);
    MeshHandle h = assets.acquireMesh(RenderMesh{}, "k");
    assets.acquireMesh(RenderMesh{}, "k");   // refs = 2
    assets.releaseMesh(h);
    CHECK(up.removes == 0);                  // still referenced
    CHECK(assets.liveMeshCount() == 1);
    assets.releaseMesh(h);
    CHECK(up.removes == 1);                  // freed exactly once
    CHECK(assets.liveMeshCount() == 0);
}

TEST_CASE(asset_manager_empty_key_never_dedups) {
    StubUploader up;
    AssetManager assets(up);
    MeshHandle a = assets.acquireMesh(RenderMesh{}, "");
    MeshHandle b = assets.acquireMesh(RenderMesh{}, "");
    CHECK(a != b);
    CHECK(up.uploads == 2);
    CHECK(assets.liveMeshCount() == 2);
}

TEST_CASE(asset_manager_size_edit_nets_zero_leak) {
    // The editor size-edit pattern (the leak this fixes): acquire the new mesh,
    // release the old. The old's last ref drops, so uploads/removes stay
    // balanced and nothing leaks.
    StubUploader up;
    AssetManager assets(up);
    MeshHandle oldH = assets.acquireMesh(RenderMesh{}, "box:1,1,1");
    MeshHandle newH = assets.acquireMesh(RenderMesh{}, "box:2,1,1");
    assets.releaseMesh(oldH);
    CHECK(up.uploads == 2);
    CHECK(up.removes == 1);
    CHECK(assets.liveMeshCount() == 1);
    CHECK(assets.refCount(newH) == 1);
}

TEST_CASE(asset_manager_clear_frees_all) {
    StubUploader up;
    AssetManager assets(up);
    assets.acquireMesh(RenderMesh{}, "a");
    assets.acquireMesh(RenderMesh{}, "b");
    assets.acquireMesh(RenderMesh{}, "");   // un-keyed
    CHECK(assets.liveMeshCount() == 3);
    assets.clear();
    CHECK(up.removes == 3);                 // each freed once
    CHECK(assets.liveMeshCount() == 0);
    assets.acquireMesh(RenderMesh{}, "a");  // cache was reset, so a fresh upload
    CHECK(up.uploads == 4);
}

TEST_CASE(asset_manager_retain_bumps_shared_refcount) {
    // The duplicate pattern: a second reference to a mesh that has no key/source
    // to re-acquire from. retain keeps it alive until both refs are released.
    StubUploader up;
    AssetManager assets(up);
    MeshHandle h = assets.acquireMesh(RenderMesh{}, "");   // refs = 1
    assets.retain(h);                                      // refs = 2
    CHECK(assets.refCount(h) == 2);
    assets.releaseMesh(h);
    CHECK(up.removes == 0);                                // still referenced
    assets.releaseMesh(h);
    CHECK(up.removes == 1);                                // freed once
    CHECK(assets.liveMeshCount() == 0);
}

TEST_CASE(asset_manager_primitive_dedups_and_builds_once) {
    StubUploader up;
    AssetManager assets(up);
    MeshHandle a = assets.acquirePrimitive("box", Vec3(1, 2, 1));
    MeshHandle b = assets.acquirePrimitive("box", Vec3(1, 2, 1));
    MeshHandle c = assets.acquirePrimitive("box", Vec3(2, 2, 1));   // different size
    CHECK(a == b);
    CHECK(a != c);
    CHECK(up.uploads == 2);          // two distinct primitives uploaded
    CHECK(assets.refCount(a) == 2);
}
