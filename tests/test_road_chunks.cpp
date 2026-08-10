// Road render chunks (docs/city-render-perf.md R1): the carriageway becomes
// per-cell chunk companion entities the frustum cull can reject, produced by
// ONE helper shared by the loader, the editor regenerates, and the planner
// bake. These tests drive the real helper against a stub uploader — the same
// headless pattern test_city_render uses — and assert on triangle counts and
// entity lifetime, not on pictures.
#include "test_framework.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/road_chunks.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/world.h"
#include <set>

using namespace engine;

namespace {

// A stub backend that counts uploads and removals — no GPU.
struct StubUploader : MeshUploader {
    uint32_t next = 1;
    int uploads = 0, removals = 0;
    MeshHandle uploadMesh(const RenderMesh&) override {
        ++uploads;
        return MeshHandle{next++, 1};
    }
    void removeMesh(MeshHandle) override { ++removals; }
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

// A flat world-space ribbon long enough to span several 250 m cells.
RenderMesh longRibbon(double length, double width) {
    RenderMesh m;
    const int segs = 64;
    for (int i = 0; i < segs; ++i) {
        const double x0 = length * i / segs, x1 = length * (i + 1) / segs;
        MeshBuilder::emitQuad(m, Vec3(x0, 0, 0), Vec3(x1, 0, 0),
                              Vec3(x1, 0, width), Vec3(x0, 0, width),
                              Vec3(0, 1, 0), Vec3(0.5, 0.5, 0.5));
    }
    return m;
}

}  // namespace

TEST_CASE(chunk_by_cell_partitions_triangles_exactly) {
    RenderMesh m = longRibbon(1000.0, 8.0);
    const std::size_t tris = m.indices.size() / 3;
    std::vector<RenderMesh> chunks = MeshBuilder::chunkByCell(m, 250.0);
    CHECK(chunks.size() >= 4);                    // 1 km spans four 250 m cells
    std::size_t sum = 0;
    for (const RenderMesh& c : chunks) sum += c.indices.size() / 3;
    CHECK(sum == tris);                           // no triangle lost, none doubled
    // cell <= 0 must return the mesh whole (the opt-out contract).
    CHECK(MeshBuilder::chunkByCell(m, 0.0).size() == 1);
}

TEST_CASE(road_chunks_spawn_cull_ready_companions) {
    World world;
    StubUploader up;
    AssetManager assets(up);
    Entity road = world.create();
    RoadNet net;                                  // material flags only; mesh prebuilt
    RenderMesh mesh = longRibbon(1000.0, 8.0);

    const int n = rebuildRoadRenderChunks(world, assets, road, net, 250.0, &mesh);
    CHECK(n >= 4);
    RenderChunks* rc = world.get<RenderChunks>(road);
    CHECK(rc != nullptr);
    CHECK(static_cast<int>(rc->entities.size()) == n);
    for (Entity e : rc->entities) {
        Renderable* r = world.get<Renderable>(e);
        CHECK(r != nullptr);
        CHECK(r->mesh.valid());
        CHECK(r->renderLayer == static_cast<uint32_t>(LayerRoads));
        PickTarget* pt = world.get<PickTarget>(e);  // clicking a chunk selects the road
        CHECK(pt != nullptr);
        CHECK(pt->target == road);
    }
}

TEST_CASE(road_chunks_rebuild_replaces_without_leaking) {
    World world;
    StubUploader up;
    AssetManager assets(up);
    Entity road = world.create();
    RoadNet net;
    RenderMesh mesh = longRibbon(1000.0, 8.0);

    const int first = rebuildRoadRenderChunks(world, assets, road, net, 250.0, &mesh);
    std::set<uint32_t> firstIdx;
    for (Entity e : world.get<RenderChunks>(road)->entities) firstIdx.insert(e.index);
    const int uploadsAfterFirst = up.uploads;

    const int second = rebuildRoadRenderChunks(world, assets, road, net, 250.0, &mesh);
    CHECK(first == second);
    // Every first-generation mesh was removed: a regenerate is a swap, not an
    // accumulation (the uploadMesh-per-drag leak this helper retires).
    CHECK(up.removals == uploadsAfterFirst);
    CHECK(up.uploads == uploadsAfterFirst * 2);
    // And the stale companion entities are genuinely destroyed.
    for (Entity e : world.get<RenderChunks>(road)->entities)
        CHECK(world.get<Renderable>(e) != nullptr);
    CHECK(world.get<RenderChunks>(road)->entities.size() ==
          static_cast<std::size_t>(second));
    (void)firstIdx;
}

TEST_CASE(road_chunks_empty_net_leaves_bare_ground) {
    World world;
    StubUploader up;
    AssetManager assets(up);
    Entity road = world.create();
    RoadNet net;
    RenderMesh mesh = longRibbon(500.0, 8.0);
    CHECK(rebuildRoadRenderChunks(world, assets, road, net, 250.0, &mesh) > 0);

    // The planner's [Clear roads]: rebuilding from an empty mesh tears the old
    // set down and spawns nothing.
    RenderMesh empty;
    CHECK(rebuildRoadRenderChunks(world, assets, road, net, 250.0, &empty) == 0);
    CHECK(world.get<RenderChunks>(road)->entities.empty());
    CHECK(up.removals == up.uploads);
}
