#include "test_framework.h"

#include "../src/engine/procgen/terrain_lod.h"
#include "../src/engine/procgen/noise.h"
#include "../src/job_system.h"

#include <atomic>
#include <cmath>
#include <set>
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

// Async CDLOD streaming (P0.4): node meshes built on JobSystem workers must be
// byte-identical to the sync path, the readiness-gated cut must never emit an
// unbuilt node (coarser, never holes), and results built against a stale
// revision (a re-conform raced the job) must never reach the cache.

namespace {

TerrainParams streamTestParams() {
    TerrainParams p;
    p.heightScale = 40.0f;
    p.noiseScale = 0.01;
    p.octaves = 5;
    p.warp = 0.6;
    p.mountainHeight = 120.0f;
    p.mountainScale = 0.004;
    // A cut/fill footprint so the flatten + morph-pin paths run too.
    p.flatten.push_back(makeFlattenPad(
        {Vec3(-10, 0, -10), Vec3(30, 0, -10), Vec3(30, 0, 30), Vec3(-10, 0, 30)},
        5.0));
    rebuildFlattenIndex(p);
    return p;
}

bool sameVec(const Vec3& a, const Vec3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;   // exact, not approximate
}

bool sameMesh(const LodNodeMesh& a, const LodNodeMesh& b) {
    if (a.mesh.vertices.size() != b.mesh.vertices.size()) return false;
    if (a.mesh.indices != b.mesh.indices) return false;
    if (!sameVec(a.boundsMin, b.boundsMin) || !sameVec(a.boundsMax, b.boundsMax))
        return false;
    for (size_t i = 0; i < a.mesh.vertices.size(); i++) {
        const Vertex& va = a.mesh.vertices[i];
        const Vertex& vb = b.mesh.vertices[i];
        if (!sameVec(va.position, vb.position)) return false;
        if (!sameVec(va.normal, vb.normal)) return false;
        if (!sameVec(va.tangent, vb.tangent)) return false;
        if (!sameVec(va.color, vb.color)) return false;
        if (va.u != vb.u || va.v != vb.v) return false;
    }
    return true;
}

// Mirrors the system's cache key: level + integer grid coords.
int64_t testKey(const LodNode& n) {
    int ix = static_cast<int>(std::lround(n.minX / n.size));
    int iz = static_cast<int>(std::lround(n.minZ / n.size));
    return (static_cast<int64_t>(n.level) << 52) |
           ((static_cast<int64_t>(ix) & 0x3FFFFFF) << 26) |
           (static_cast<int64_t>(iz) & 0x3FFFFFF);
}

}  // namespace

// --- Determinism: worker build == render-thread build, byte for byte ---

TEST_CASE(async_node_mesh_matches_sync_exactly) {
    TerrainParams p = streamTestParams();
    Noise noise(1234);
    LodNode node{-64.0f, -32.0f, 64.0f, 1};
    const int gridRes = 16;
    const double eps = 0.5;

    LodNodeMesh sync = generateLodNodeMesh(p, noise, node, gridRes, eps);

    // What a worker sees: COPIES of the recipe (the per-revision snapshot),
    // executed on a pool thread. generateLodNodeMesh is pure, so the bytes
    // must match — only arrival timing may differ.
    TerrainParams pCopy = p;
    Noise noiseCopy(1234);
    JobSystem jobs(2);
    LodNodeMesh fromWorker;
    std::atomic<int> counter{0};
    jobs.run([&] {
        fromWorker = generateLodNodeMesh(pCopy, noiseCopy, node, gridRes, eps);
    }, &counter);
    jobs.wait(&counter);

    CHECK(sync.mesh.vertices.size() == 17u * 17u);
    CHECK(sameMesh(sync, fromWorker));
}

// --- Gating: the cut never emits an unready node, tiles exactly, converges ---

TEST_CASE(gated_cut_only_emits_ready_nodes_and_converges) {
    const float worldHalf = 512.0f;
    const int numLods = 5;
    auto ranges = lodRangesForWorld(worldHalf, numLods);
    const float camX = 100.0f, camZ = -60.0f;

    std::set<int64_t> ready;
    std::vector<LodNode> requested;
    auto ensure = [&](const LodNode& n) {
        int64_t k = testKey(n);
        if (ready.count(k)) return true;
        if (n.level >= numLods - 2) {   // coarse levels build synchronously
            ready.insert(k);
            return true;
        }
        requested.push_back(n);
        return false;
    };

    auto ideal = selectLodNodes(worldHalf, numLods, ranges, camX, camZ);
    const double world = static_cast<double>(worldHalf * 2.0) * (worldHalf * 2.0);

    std::vector<LodNode> cut;
    int frames = 0;
    for (;; frames++) {
        CHECK(frames <= numLods);   // refines at least one level per completed batch
        requested.clear();
        cut = selectLodNodesGated(worldHalf, numLods, ranges, camX, camZ, ensure);
        // Never a node whose mesh isn't cached...
        for (const LodNode& n : cut) CHECK(ready.count(testKey(n)) == 1);
        // ...and never a hole while refining: exact world coverage every frame.
        double area = 0.0;
        for (const LodNode& n : cut)
            area += static_cast<double>(n.size) * n.size;
        CHECK_APPROX(area, world, world * 1e-6);
        if (requested.empty()) break;
        for (const LodNode& n : requested) ready.insert(testKey(n));  // jobs land
    }

    // Once everything requested has arrived, the cut IS the ideal sync cut.
    std::set<int64_t> got, want;
    for (const LodNode& n : cut) got.insert(testKey(n));
    for (const LodNode& n : ideal) want.insert(testKey(n));
    CHECK(got == want);
    CHECK(frames >= 1);   // fine levels actually streamed (weren't all sync)
}

// --- Revision correctness: stale results are dropped, never cached ---

TEST_CASE(stream_drops_stale_revision_results) {
    LodMeshStream s;
    CHECK(s.begin(42, 1));
    CHECK(!s.begin(42, 1));   // deduped while in flight
    CHECK(s.inFlightCount() == 1u);

    LodMeshStream::Result r;
    r.key = 42;
    r.revision = 1;
    s.complete(r);

    // A re-conform bumped the revision before the job drained: dropped.
    auto ready = s.drain(2);
    CHECK(ready.empty());
    CHECK(s.inFlightCount() == 0u);   // but the slot is free to re-request
}

TEST_CASE(stream_survives_reconform_race_without_double_insert) {
    LodMeshStream s;
    CHECK(s.begin(7, 1));   // job A enqueued at revision 1
    s.invalidate();          // re-conform: revision bumps to 2, in-flight forgotten
    CHECK(s.inFlightCount() == 0u);
    CHECK(s.begin(7, 2));   // same node re-requested at revision 2 (job B)

    // Job A (stale) lands first: dropped, and job B's accounting is untouched.
    LodMeshStream::Result stale;
    stale.key = 7;
    stale.revision = 1;
    s.complete(stale);
    auto ready = s.drain(2);
    CHECK(ready.empty());
    CHECK(s.inFlightCount() == 1u);
    CHECK(!s.begin(7, 2));   // B still flying — no third job

    // Job B lands: accepted exactly once.
    LodMeshStream::Result fresh;
    fresh.key = 7;
    fresh.revision = 2;
    s.complete(fresh);
    ready = s.drain(2);
    CHECK(ready.size() == 1u);
    CHECK(ready[0].key == 7);
    CHECK(ready[0].revision == 2u);
    CHECK(s.inFlightCount() == 0u);
}

TEST_CASE(stream_delivers_current_revision_results) {
    LodMeshStream s;
    CHECK(s.begin(1, 3));
    CHECK(s.begin(2, 3));
    LodMeshStream::Result a, b;
    a.key = 1;
    a.revision = 3;
    b.key = 2;
    b.revision = 3;
    s.complete(a);
    auto first = s.drain(3);
    CHECK(first.size() == 1u);
    CHECK(first[0].key == 1);
    CHECK(s.inFlightCount() == 1u);   // key 2 still building
    s.complete(b);
    auto second = s.drain(3);
    CHECK(second.size() == 1u);
    CHECK(second[0].key == 2);
    CHECK(s.inFlightCount() == 0u);
}
