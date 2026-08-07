#include "terrain_lod_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../asset_manager.h"
#include "../procgen/terrain_lod.h"
#include "../procgen/noise.h"
#include "../../profile.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine {

// Everything a worker needs to build a node mesh, copied out of the live
// component once per revision (generateLodNodeMesh is a pure function of these,
// so a copy built off-thread is byte-identical to the sync path). The embedded
// shared_ptrs (flattenIndex, erodedBase) are shared read-only.
struct TerrainGenInputs {
    TerrainParams params;
    Noise noise{0};
    int gridRes = 0;
    double normalEps = 0.0;
    uint32_t revision = 0;
};

namespace {
// Exact cache key for a node: level (4 bits) + integer grid coords (26 bits each,
// signed). Node corners are at integer multiples of the node size, so ix/iz round
// cleanly. The grid coord ranges (|coord| < 2^(numLods-1)) fit well inside 26 bits.
int64_t nodeKey(const LodNode& n) {
    int ix = static_cast<int>(std::lround(n.minX / n.size));
    int iz = static_cast<int>(std::lround(n.minZ / n.size));
    return (static_cast<int64_t>(n.level) << 52) |
           ((static_cast<int64_t>(ix) & 0x3FFFFFF) << 26) |
           (static_cast<int64_t>(iz) & 0x3FFFFFF);
}
}  // namespace

void TerrainLodSystem::render(FrameContext& ctx) {
    RT_PROFILE_ZONE_NAMED("terrainLod");
    // One CDLOD terrain per level. Grab the first config (if any).
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) return;

    // RT_SYNC_CDLOD=1: the pre-streaming path — every cut node built inline on
    // this thread. Kept for bisection.
    static const bool syncOnly = [] {
        const char* e = std::getenv("RT_SYNC_CDLOD");
        return e && e[0] == '1';
    }();

    // A re-conform bumped the config: drop the cached tiles so they rebuild from the new
    // params (the new grading). In-flight jobs keep running against the old
    // snapshot; drain drops their (stale-revision) results.
    if (cfg->revision != cacheRevision_) {
        for (auto& kv : cache_) ctx.assets.releaseMesh(kv.second.mesh);
        cache_.clear();
        cacheRevision_ = cfg->revision;
        genInputs_.reset();
        if (stream_) stream_->invalidate();
    }

    const CameraState& cam = ctx.view.camera;

    // Forward-Z frustum (the view volume is identical to the GPU's reverse-Z; CPU
    // culling stays forward-Z — ADR-0034/0035), matching RenderSystem.
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                             cam.nearPlane, cam.farPlane);
    Frustum frustum = Frustum::fromViewProjection(proj * view);

    std::vector<float> ranges =
        lodRangesForWorld(cfg->worldHalf, cfg->numLods, cfg->rangeFactor);

    // World-consistent normal eps (the finest leaf step), the SAME for every node
    // so normals match across LOD boundaries — no shading seam (ADR-0036).
    float leafSize = (cfg->worldHalf * 2.0f) /
                     static_cast<float>(1 << std::max(0, cfg->numLods - 1));
    double normalEps = leafSize / static_cast<float>(std::max(2, cfg->gridRes));

    ++frame_;

    // Mesh upload/acquire stays on this (render) thread.
    auto insertNode = [&](int64_t key, const LodNodeMesh& built) {
        CachedNode cached;
        cached.mesh = ctx.assets.acquireMesh(built.mesh,
                                             "cdlod_" + std::to_string(key));
        cached.boundsMin = built.boundsMin;
        cached.boundsMax = built.boundsMax;
        cached.lastUsed = frame_;
        cache_.emplace(key, cached);
    };

    std::vector<LodNode> nodes;
    if (syncOnly) {
        RT_PROFILE_ZONE_NAMED("cdlod_generate_sync");
        Noise noise(cfg->seed);
        nodes = selectLodNodes(
            cfg->worldHalf, cfg->numLods, ranges,
            static_cast<float>(cam.position.x), static_cast<float>(cam.position.z));
        for (const LodNode& node : nodes) {
            int64_t key = nodeKey(node);
            if (cache_.count(key)) continue;
            insertNode(key, generateLodNodeMesh(cfg->params, noise, node,
                                                cfg->gridRes, normalEps));
        }
    } else {
        if (!stream_) stream_ = std::make_shared<LodMeshStream>();
        if (!genInputs_) {
            auto gi = std::make_shared<TerrainGenInputs>();
            gi->params = cfg->params;
            gi->noise = Noise(cfg->seed);
            gi->gridRes = cfg->gridRes;
            gi->normalEps = normalEps;
            gi->revision = cfg->revision;
            genInputs_ = std::move(gi);
        }

        {
            RT_PROFILE_ZONE_NAMED("cdlod_drain");
            for (LodMeshStream::Result& r : stream_->drain(cacheRevision_))
                if (!cache_.count(r.key)) insertNode(r.key, r.built);
        }

        // Gated selection: descend only where all four child meshes are cached,
        // else emit the (cached) parent and collect the missing children —
        // coarser for a few frames, never a hole. The root and its children are
        // built inline so there is always ground, even on the first frame.
        std::vector<LodNode> missing;
        auto ensure = [&](const LodNode& node) {
            int64_t key = nodeKey(node);
            auto it = cache_.find(key);
            if (it != cache_.end()) {
                it->second.lastUsed = frame_;   // wanted: outlive a slow sibling
                return true;
            }
            if (node.level >= cfg->numLods - 2) {
                RT_PROFILE_ZONE_NAMED("cdlod_generate_coarse");
                insertNode(key, generateLodNodeMesh(genInputs_->params,
                                                    genInputs_->noise, node,
                                                    genInputs_->gridRes,
                                                    genInputs_->normalEps));
                return true;
            }
            missing.push_back(node);
            return false;
        };
        nodes = selectLodNodesGated(
            cfg->worldHalf, cfg->numLods, ranges,
            static_cast<float>(cam.position.x), static_cast<float>(cam.position.z),
            ensure);

        // Coarse levels first: one coarse mesh unblocks a whole subtree.
        std::stable_sort(missing.begin(), missing.end(),
                         [](const LodNode& a, const LodNode& b) {
                             return a.level > b.level;
                         });
        for (const LodNode& node : missing) {
            if (stream_->inFlightCount() >= kMaxInFlight) break;
            int64_t key = nodeKey(node);
            if (!stream_->begin(key, cacheRevision_)) continue;   // already flying
            std::shared_ptr<const TerrainGenInputs> gi = genInputs_;
            std::shared_ptr<LodMeshStream> stream = stream_;
            ctx.jobs.run([gi, stream, node, key] {
                RT_PROFILE_ZONE_NAMED("cdlod_generate_job");
                LodMeshStream::Result r;
                r.key = key;
                r.revision = gi->revision;
                r.built = generateLodNodeMesh(gi->params, gi->noise, node,
                                              gi->gridRes, gi->normalEps);
                stream->complete(std::move(r));
            });
        }
    }

    for (const LodNode& node : nodes) {
        auto it = cache_.find(nodeKey(node));
        if (it == cache_.end()) continue;   // gated selection never emits these
        CachedNode& cn = it->second;
        cn.lastUsed = frame_;   // in the cut this frame (keep it cached)
        if (!frustum.containsAABB(cn.boundsMin, cn.boundsMax)) continue;

        MorphRange m = lodMorphRange(node.level, ranges);
        ctx.renderer.drawTerrain(cn.mesh, cfg->material, m.start, m.end);
    }

    // Evict node meshes that have been out of the cut for a while (the player moved
    // away). The cut itself is bounded, so this keeps the cache ~ a few recent cuts
    // instead of every node ever visited.
    for (auto it = cache_.begin(); it != cache_.end();) {
        if (frame_ - it->second.lastUsed > kEvictAfterFrames) {
            ctx.assets.releaseMesh(it->second.mesh);
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}

// Static collider window: keep a triangle-mesh body under the leaf-level nodes
// around the player so they walk on the surface, freeing nodes that fall outside
// the window. Bodies are owned directly (addMesh/removeBody), not via ECS, so there
// is no entity/body churn. Runs only with physics (play mode).
void TerrainLodSystem::fixedUpdate(FrameContext& ctx) {
    RT_PROFILE_ZONE_NAMED("terrainColliders");
    if (!physics_) return;

    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) return;

    // A re-conform bumped the config: drop the collider window so it rebuilds against
    // the new grading.
    if (cfg->revision != colliderRevision_) {
        for (auto& kv : colliders_) physics_->physicsWorld().removeBody(kv.second);
        colliders_.clear();
        colliderRevision_ = cfg->revision;
    }

    // Centre the window on the player (the entity PlayerSystem drives), and
    // PREFETCH along the velocity: at driving speed a new leaf row must have
    // its collider before the car arrives, not when it does (P0.6 — building
    // several leaves in one fixed step spiked past the clock clamp and read
    // as rubber-banding).
    Vec3 player, playerPrev;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>(
        [&](Entity e, Transform& t, ControlledBy&) {
            if (found) return;
            // DRIVING (device: "as it accelerates the car begins to lag out"):
            // while InVehicle the on-foot character is PARKED at the kerb and
            // PlayerSystem stops moving it — centring the window there strands
            // the floor at the entry point and the car outruns its own terrain
            // collider within ~one window radius. Follow the CAR.
            Entity tracked = e;
            if (const InVehicle* iv = ctx.world.get<InVehicle>(e))
                if (iv->vehicle.valid() && ctx.world.alive(iv->vehicle) &&
                    ctx.world.has<Transform>(iv->vehicle))
                    tracked = iv->vehicle;
            const Transform* tt =
                tracked == e ? &t : ctx.world.get<Transform>(tracked);
            player = tt->position;
            playerPrev = player;
            if (const PrevTransform* pt = ctx.world.get<PrevTransform>(tracked))
                playerPrev = pt->value.position;
            found = true;
        });
    if (!found) return;
    Vec3 vel = (player - playerPrev) * (1.0 / std::max(1e-6, ctx.clock.fixedStep()));
    Vec3 lookahead = player + vel * 1.2;   // ~1.2 s of travel ahead

    const int levels = std::max(1, cfg->numLods);
    const int leafCount = 1 << (levels - 1);                  // leaf cells per side
    const float worldHalf = cfg->worldHalf;
    const float leafSize = (worldHalf * 2.0f) / static_cast<float>(leafCount);
    const float radius =
        cfg->colliderRadius > 0.0f ? cfg->colliderRadius : leafSize * 1.5f;

    auto cellIndex = [&](float w) {
        return static_cast<int>(std::floor((w + worldHalf) / leafSize));
    };
    auto windowBox = [&](const Vec3& c, int& x0, int& x1, int& z0, int& z1) {
        x0 = std::max(0, cellIndex(static_cast<float>(c.x) - radius));
        x1 = std::min(leafCount - 1, cellIndex(static_cast<float>(c.x) + radius));
        z0 = std::max(0, cellIndex(static_cast<float>(c.z) - radius));
        z1 = std::min(leafCount - 1, cellIndex(static_cast<float>(c.z) + radius));
    };
    int ix0, ix1, iz0, iz1, jx0, jx1, jz0, jz1;
    windowBox(player, ix0, ix1, iz0, iz1);
    windowBox(lookahead, jx0, jx1, jz0, jz1);

    const double normalEps = leafSize / static_cast<float>(std::max(2, cfg->gridRes));
    Noise noise(cfg->seed);

    // Desired = union of the window around the player and around the lookahead
    // point. Missing cells are built CLOSEST-FIRST under a per-step budget: one
    // build is the expensive unit (~10K height evals), and several in one fixed
    // step is exactly the spike that trips the clock clamp. The cell under the
    // player's feet is exempt from the budget — the floor is never deferred.
    std::unordered_set<int64_t> desired;
    struct Missing { int64_t key; LodNode node; double d2; };
    std::vector<Missing> missing;
    auto gather = [&](int x0, int x1, int z0, int z1) {
        for (int iz = z0; iz <= z1; iz++)
            for (int ix = x0; ix <= x1; ix++) {
                LodNode node{-worldHalf + ix * leafSize, -worldHalf + iz * leafSize,
                             leafSize, 0};
                int64_t key = nodeKey(node);
                if (!desired.insert(key).second) continue;
                if (colliders_.count(key)) continue;
                const double cx = node.minX + leafSize * 0.5, cz = node.minZ + leafSize * 0.5;
                const double dx = cx - player.x, dz = cz - player.z;
                missing.push_back({key, node, dx * dx + dz * dz});
            }
    };
    gather(ix0, ix1, iz0, iz1);
    gather(jx0, jx1, jz0, jz1);
    std::sort(missing.begin(), missing.end(),
              [](const Missing& a, const Missing& b) {
                  return a.d2 != b.d2 ? a.d2 < b.d2 : a.key < b.key;
              });
    const double feet2 = leafSize * static_cast<double>(leafSize);
    int budget = 1;
    for (const Missing& m : missing) {
        const bool underFeet = m.d2 <= feet2;   // player's own / adjacent cell
        if (!underFeet && budget <= 0) continue;
        LodNodeMesh built = generateLodNodeMesh(cfg->params, noise, m.node,
                                                cfg->gridRes, normalEps);
        std::vector<Vec3> verts;
        verts.reserve(built.mesh.vertices.size());
        for (const Vertex& v : built.mesh.vertices) verts.push_back(v.position);
        PhysicsBodyId id = physics_->physicsWorld().addMesh(
            verts, built.mesh.indices, Vec3(0, 0, 0), 0.8);
        if (id != INVALID_PHYSICS_BODY) colliders_[m.key] = id;
        if (!underFeet) --budget;
    }
    for (auto it = colliders_.begin(); it != colliders_.end();) {
        if (desired.count(it->first)) { ++it; continue; }
        physics_->physicsWorld().removeBody(it->second);
        it = colliders_.erase(it);
    }
}

void TerrainLodSystem::onStop(FrameContext&) {
    if (!physics_) return;
    for (auto& kv : colliders_) physics_->physicsWorld().removeBody(kv.second);
    colliders_.clear();
}

}  // namespace engine
