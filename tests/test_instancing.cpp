#include "test_framework.h"

#include "../src/engine/procgen/scatter.h"
#include "../src/engine/components.h"
#include "../src/engine/world.h"
#include "../src/engine/systems/render_system.h"   // frustumCullInstances
#include "../src/renderer/renderer.h"
#include "../src/rt_math.h"

#include <cstdint>
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// Minimal Renderer that counts draws, to verify the default drawMeshInstanced.
struct CountingRenderer : Renderer {
    int meshDraws = 0;
    bool initialize(void*, int, int) override { return true; }
    void shutdown() override {}
    void resize(int, int) override {}
    MeshHandle uploadMesh(const RenderMesh&) override { return MeshHandle{1, 1}; }
    void removeMesh(MeshHandle) override {}
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
    TextureHandle uploadTexture(int, int, int, const uint8_t*) override { return {}; }
    void removeTexture(TextureHandle) override {}
    RenderStats getRenderStats() const override { return {}; }
    void beginFrame() override {}
    void setCamera(const CameraState&) override {}
    void setLights(const SceneLighting&) override {}
    void drawMesh(MeshHandle, const Mat4&, const RenderMaterial&) override { meshDraws++; }
    void endFrame() override {}
};

}  // namespace

TEST_CASE(scatter_buckets_lose_no_placements_and_are_deterministic) {
    std::vector<Placement> ps;
    for (int i = 0; i < 200; ++i)
        ps.push_back({Vec3(static_cast<Real>(i), 0, 0), 0.0f, 1.0f});

    auto a = bucketPlacementsBySpecies(ps, 4, 7);
    CHECK(a.size() == 4);
    std::size_t total = 0;
    for (const auto& b : a) total += b.size();
    CHECK(total == 200);                 // every placement lands in exactly one bucket

    auto b = bucketPlacementsBySpecies(ps, 4, 7);
    for (std::size_t i = 0; i < 4; ++i) CHECK(a[i].size() == b[i].size());  // deterministic

    int nonEmpty = 0;
    for (const auto& bucket : a) if (!bucket.empty()) nonEmpty++;
    CHECK(nonEmpty >= 2);                // species actually mix
}

TEST_CASE(scatter_bucket_bakes_position_yaw_scale_into_the_matrix) {
    std::vector<Placement> ps = {{Vec3(5, 2, -3), 0.0f, 2.0f}};
    auto buckets = bucketPlacementsBySpecies(ps, 1, 1);
    CHECK(buckets[0].size() == 1);
    const Mat4& m = buckets[0][0];
    CHECK_APPROX(m.m[0][3], 5.0, 1e-9);   // translation column = position
    CHECK_APPROX(m.m[1][3], 2.0, 1e-9);
    CHECK_APPROX(m.m[2][3], -3.0, 1e-9);
    // uniform scale 2 -> each rotation column has length 2 (yaw=0 here)
    Vec3 col0(m.m[0][0], m.m[1][0], m.m[2][0]);
    CHECK_APPROX(col0.length(), 2.0, 1e-9);
}

TEST_CASE(instance_group_collapses_entities) {
    // 1000 placements over 3 species become at most 3 InstanceGroup entities,
    // not 1000 — the whole point of the path.
    std::vector<Placement> ps;
    for (int i = 0; i < 1000; ++i)
        ps.push_back({Vec3(static_cast<Real>(i % 50), 0, static_cast<Real>(i / 50)), 0.0f, 1.0f});
    auto buckets = bucketPlacementsBySpecies(ps, 3, 42);

    World world;
    std::size_t instances = 0;
    for (auto& bucket : buckets) {
        if (bucket.empty()) continue;
        InstanceGroup g;
        g.mesh = MeshHandle{1, 1};
        g.transforms = bucket;
        instances += g.transforms.size();
        world.add<InstanceGroup>(world.create(), g);
    }
    CHECK(world.entityCount() <= 3);     // a handful of groups, not 1000 entities
    CHECK(instances == 1000);            // but all 1000 plants are represented
}

TEST_CASE(instance_group_culls_per_instance) {
    // Restores per-plant culling lost when scatter collapsed to one group: only
    // instances inside the frustum survive (not the whole region every frame).
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 100.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    std::vector<Mat4> xf = {
        Mat4::translate(0, 0, 0),      // in front of the camera -> visible
        Mat4::translate(0, 0, 50),     // behind the camera -> culled
        Mat4::translate(200, 0, 0),    // far to the side -> culled
    };
    auto vis = frustumCullInstances(xf, f, Vec3(0, 0, 0), 0.5);
    CHECK(vis.size() == 1);
    CHECK_APPROX(vis[0].m[2][3], 0.0, 1e-9);   // the one at the origin survived
}

TEST_CASE(cull_into_scratch_reuses_the_buffer) {
    // The draw path culls into a reused buffer instead of returning a fresh
    // vector sized to the FULL instance count every group every frame. Reusing
    // it only works if the callee clears first — otherwise groups accumulate
    // into each other and the second group draws the first group's plants too.
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 100.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    std::vector<Mat4> xf = {Mat4::translate(0, 0, 0), Mat4::translate(0, 0, 50)};
    std::vector<Mat4> scratch;
    frustumCullInstances(xf, f, Vec3(0, 0, 0), 0.5, Vec3(0, 0, 5), 0, scratch);
    CHECK(scratch.size() == 1);
    const size_t cap = scratch.capacity();
    frustumCullInstances(xf, f, Vec3(0, 0, 0), 0.5, Vec3(0, 0, 5), 0, scratch);
    CHECK(scratch.size() == 1);          // cleared, not appended
    CHECK(scratch.capacity() == cap);    // and no reallocation the second time
}

TEST_CASE(draw_policy_resolves_distance_by_class) {
    // The whole point of DrawClass: the distance for a class is stated ONCE and
    // every drawable of that class inherits it, while a site that computed its
    // own distance (the HLOD building chunks) keeps it.
    DrawPolicy p;
    p.distance[static_cast<int>(DrawClass::Scenery)] = 600.0;
    p.distance[static_cast<int>(DrawClass::GroundPaint)] = 220.0;

    CHECK_APPROX(p.distanceFor(DrawClass::Scenery), 600.0, 1e-9);
    CHECK_APPROX(p.distanceFor(DrawClass::GroundPaint), 220.0, 1e-9);
    // Unlimited is explicit and stays unlimited; Unset resolves to the same
    // thing (0), which is what every drawable did before the class existed —
    // so adding the class to the engine changes nothing until a level opts in.
    CHECK_APPROX(p.distanceFor(DrawClass::Unlimited), 0.0, 1e-9);
    CHECK_APPROX(p.distanceFor(DrawClass::Unset), 0.0, 1e-9);
    // A class the level did not mention is unlimited, not accidentally zero-range.
    CHECK_APPROX(p.distanceFor(DrawClass::Structure), 0.0, 1e-9);
}

TEST_CASE(renderer_default_instanced_draw_loops_draw_mesh) {
    CountingRenderer r;
    std::vector<Mat4> xf = {Mat4::identity(), Mat4::translate(1, 0, 0),
                            Mat4::translate(2, 0, 0)};
    r.drawMeshInstanced(MeshHandle{1, 1}, xf, RenderMaterial{});
    CHECK(r.meshDraws == 3);             // default fans out to drawMesh (Metal then batches)
}
