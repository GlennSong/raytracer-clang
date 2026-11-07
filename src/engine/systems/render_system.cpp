#include "render_system.h"
#include "../components.h"
#include "../../profile.h"
#include "../occlusion.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace engine {

// Cull one group's instances into `out`. `out` is a caller-owned scratch buffer
// reused across groups and frames: this used to return by value with
// reserve(transforms.size()), so every group heap-allocated its FULL instance
// count every frame (~510 KB for the 7964-instance parking-bay group) and then
// copied the survivors again into the draw.
void frustumCullInstances(const std::vector<Mat4>& transforms,
                          const Frustum& frustum,
                          const Vec3& meshCenter, Real meshRadius,
                          const Vec3& cameraPos, Real maxDistance,
                          std::vector<Mat4>& out, Real minDistance) {
    out.clear();
    Real maxDistSq = maxDistance * maxDistance;
    const Real minDistSq = minDistance * minDistance;
    for (const Mat4& m : transforms) {
        const Vec3 center = instanceCentre(m, meshCenter);   // affine: the bottom row is data (rt_math.h)
        const Real d2 = (center - cameraPos).lengthSquared();
        if (maxDistance > 0 && d2 > maxDistSq)
            continue;
        if (minDistance > 0 && d2 < minDistSq)   // (a nearer level of detail draws it there)
            continue;
        if (frustum.containsSphere(center, meshRadius * instanceScale(m)))
            out.push_back(m);
    }
}

std::vector<Mat4> frustumCullInstances(const std::vector<Mat4>& transforms,
                                       const Frustum& frustum,
                                       const Vec3& meshCenter, Real meshRadius,
                                       const Vec3& cameraPos, Real maxDistance) {
    std::vector<Mat4> visible;
    frustumCullInstances(transforms, frustum, meshCenter, meshRadius, cameraPos,
                         maxDistance, visible);
    return visible;
}

void RenderSystem::onStart(FrameContext& ctx) {
    // Adopt the level's exposure (set by the level loader from JSON) rather than
    // forcing our own value — other owners (level JSON, the ImGui slider) drive it.
    exposure = ctx.view.lighting.exposure;
}

void tierFadeBand(RenderMaterial& m, double dist, float share, bool coming) {
    // the dither's share as a band so wide (kBand) that it reads the same at every pixel of a cell: smoothstep(e0,
    // e0 + kBand, d) = s at d = dist when e0 = dist - t kBand, t = smoothstep^-1(s). Coming in keeps the pixels the
    // band's fade-in keeps (a share `share`), going out those its fade-out keeps: complementary thresholds
    // (mesh.frag), so a pair at shares a and 1 - a covers every pixel once.
    constexpr double kBand = 1.0e5;
    auto inv = [](double s) { return 0.5 - std::sin(std::asin(1.0 - 2.0 * s) / 3.0); };
    m.flags |= RenderMaterial::FLAG_LOD_BAND;
    const double e0 = dist - inv(coming ? share : 1.0 - share) * kBand;
    if (coming) { m.lodIn0 = static_cast<float>(e0); m.lodIn1 = static_cast<float>(e0 + kBand); m.lodOut0 = m.lodOut1 = 0.0f; }
    else { m.lodOut0 = static_cast<float>(e0); m.lodOut1 = static_cast<float>(e0 + kBand); m.lodIn0 = m.lodIn1 = 0.0f; }
}

void RenderSystem::update(FrameContext& ctx) {
    // Track the current exposure (the level JSON or the ImGui slider may have set
    // it) so the Up/Down keys ramp from there. Only write back when a key is
    // actually held, so we don't stomp the slider/JSON value every frame.
    exposure = ctx.view.lighting.exposure;
    if (ctx.input.keyUp || ctx.input.keyDown) {
        if (ctx.input.keyUp)   exposure *= 1.0f + 2.0f * static_cast<float>(ctx.frameDelta);
        if (ctx.input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(ctx.frameDelta);
        exposure = std::clamp(exposure, 0.05f, 20.0f);
        ctx.view.lighting.exposure = exposure;
    }
}

void RenderSystem::render(FrameContext& ctx) {
    RT_PROFILE_ZONE_NAMED("renderSubmit");
    ctx.renderer.setCamera(ctx.view.camera);
    // THE FRAME'S LIGHTS (ADR-0362): the level's own, then what LightSystem picked from everything that gives off
    // light (lamps, rooms, beacons, headlights, the flashlight), on a COPY so the level's are never displaced
    if (ctx.view.lights.points.empty() && ctx.view.lights.spots.empty()) {
        ctx.renderer.setLights(ctx.view.lighting);
    } else {
        SceneLighting withLights = ctx.view.lighting;
        withLights.pointLights.insert(withLights.pointLights.end(), ctx.view.lights.points.begin(), ctx.view.lights.points.end());
        withLights.spotLights.insert(withLights.spotLights.end(), ctx.view.lights.spots.begin(), ctx.view.lights.spots.end());
        ctx.renderer.setLights(withLights);
    }

    const auto& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                              cam.nearPlane, cam.farPlane);
    Frustum frustum = Frustum::fromViewProjection(proj * view);
    // OCCLUSION (engine/occlusion.h): a recent frame's depth, while it still stands for this view
    const OcclusionDepth* occ = ctx.renderer.occlusionDepth();
    Vec3 camFwd = cam.target - cam.position;
    camFwd = camFwd.length() > 1e-9 ? camFwd * (1.0 / camFwd.length()) : Vec3(0, 0, -1);
    if (occ && (cam.projection != CameraProjection::Perspective || !occlusionUsable(*occ, cam.position, camFwd))) occ = nullptr;
    const double occSlack = occ ? (cam.position - occ->eye).length() + 0.25 : 0.0;
    static const bool occStats = std::getenv("RT_OCC_STATS") != nullptr;
    long occTested = 0, occCulled = 0, grpTested = 0, grpCulled = 0;

    // RT_DUMP_DRAWS=1: one-shot draw audit — every Renderable's bounds and the
    // cull verdict, printed once around frame ~60. The tool that ends "the mesh
    // exists but nothing shows" mysteries.
    static int auditFrame = std::getenv("RT_DUMP_DRAWS") ? 60 : -1;
    const bool audit = auditFrame >= 0 && --auditFrame == 0;

    // CONTENT GATES for attribution: hide a whole class of geometry and read the
    // triangle/draw delta off the next [stats] line. `hiddenLayers` was already
    // honoured below (and by the InstanceGroup pass), but nothing anywhere set
    // it — so the cheapest attribution tool in the engine was unreachable
    // outside an ImGui build. Parsed here rather than in the renderer, which
    // treats the mask as opaque and does not know the RenderLayer bits.
    //
    // Read before trusting a subtraction: terrain, trees, lot pads, street
    // lamps, signal posts and road paint are all layer 0, so they are never
    // hidden and always land in the residual.
    static const uint32_t envHidden = []() {
        uint32_t m = 0;
        if (std::getenv("RT_NO_ROADS"))     m |= LayerRoads;
        if (std::getenv("RT_NO_BUILDINGS")) m |= LayerBuildings;
        if (std::getenv("RT_NO_SIM"))       m |= LayerSim;
        if (std::getenv("RT_NO_FOLIAGE"))   m |= LayerFoliage;
        return m;
    }();
    ctx.renderer.hiddenLayers |= envHidden;

    // THE draw-distance owner. Every drawable states a DrawClass; this table
    // turns the class into metres, once, with the level's scale in hand. A
    // level that stamps no policy resolves everything to unlimited, which is
    // byte-for-byte the behaviour before DrawClass existed — so opting in is
    // per level and nothing else moves.
    const DrawPolicy* policy = nullptr;
    ctx.world.each<DrawPolicy>(
        [&](Entity, DrawPolicy& p) { if (!policy) policy = &p; });
    // An explicit drawDistance at the site still wins: the HLOD building chunks
    // and their mass-box proxies compute a matched pair and must stay matched.
    auto resolveDistance = [&](Real explicitDist, DrawClass cls) -> Real {
        if (explicitDist > 0) return explicitDist;
        return policy ? policy->distanceFor(cls) : Real(0);
    };

    if (audit)
        std::fprintf(stderr,
                     "[draw-audit] cam (%.0f,%.0f,%.0f) fov %.1f far %.0f\n",
                     cam.position.x, cam.position.y, cam.position.z,
                     cam.fovDegrees, cam.farPlane);

    Real alpha = ctx.interpolation;
    // the tier crossfade (TierFade): a fade's step this frame; RT_NO_TIER_FADE=1 pops as before (A/B)
    static const bool noTierFade = std::getenv("RT_NO_TIER_FADE") != nullptr;
    // s; RT_TIER_FADE_S=<s> to look at one (a slow fade shows in a screenshot)
    static const double kTierFade = std::getenv("RT_TIER_FADE_S") ? std::max(0.01, std::atof(std::getenv("RT_TIER_FADE_S"))) : 0.35;
    const bool tierFades = !noTierFade && ctx.renderer.supportsLodBand();
    const auto now = std::chrono::steady_clock::now();   // (frameDelta is update()'s: render keeps its own clock)
    const double renderDt = frameNo_ ? std::chrono::duration<double>(now - lastRender_).count() : 0.0;
    lastRender_ = now;
    const float fadeStep = static_cast<float>(std::clamp(renderDt / kTierFade, 0.0, 1.0));
    ++frameNo_;
    SparseSet<SourceSpec>* sourceSpecs = ctx.world.componentPool<SourceSpec>();   // (once, not per entity)
    // RT_DUMP_STATS: the encode walk's two halves timed and counted (entities visited / drawn, groups and instances
    // visited / drawn), every 120 frames
    static const bool encStats = std::getenv("RT_DUMP_STATS") != nullptr;
    static double encEntMs = 0, encGrpMs = 0;
    static long encEnt = 0, encEntDrawn = 0, encGrp = 0, encGrpIn = 0, encInst = 0, encInstDrawn = 0, encFrames = 0, encFading = 0;
    const auto encT0 = std::chrono::steady_clock::now();
    // One drawable's cull + draw (reached through the draw index's visible cells, or the ECS walk with
    // RT_NO_DRAW_INDEX=1): the same tests whichever way it is found.
    auto drawEntity = [&](uint32_t entityIndex, Transform& t, PrevTransform& prev, Renderable& r) {
            // (the camera's own body is not drawn -- but only when there IS a camera entity: an empty handle's index is
            // 0, the first entity a level makes, and skipping that hid every lab's ground in play since 28fc0763)
            if (ctx.view.activeCameraEntity.valid() && entityIndex == ctx.view.activeCameraEntity.index) return;
            ++encEnt;
            if (audit) {
                BoundingSphere ab = ctx.renderer.getMeshBounds(r.mesh);
                std::fprintf(stderr,
                             "[draw-audit] e%u mesh %u layer %u hidden %d aabb "
                             "(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
                             entityIndex, r.mesh.index, r.renderLayer,
                             (r.renderLayer & ctx.renderer.hiddenLayers) ? 1 : 0,
                             ab.boxMin.x, ab.boxMin.y, ab.boxMin.z, ab.boxMax.x,
                             ab.boxMax.y, ab.boxMax.z);
            }
            if (r.renderLayer & ctx.renderer.hiddenLayers) return;   // debug layer hidden
            // Interpolated local matrix, then composed through any parent
            // chain (editor only — PLAY flattens parenting at load, so there
            // parentId is 0 and this is just the interpolated local).
            Mat4 model = lerp(prev.value, t, alpha).matrix();
            SourceSpec* s = sourceSpecs ? sourceSpecs->get(entityIndex) : nullptr;
            if (s && s->parentId != 0) {
                Entity parent = findByDocumentId(ctx.world, s->parentId);
                if (parent.valid())
                    model = worldMatrix(ctx.world, parent) * model;
            }
            // Cull against the world-space AABB (tight for large/flat meshes like
            // terrain chunks, where a bounding sphere swallows the sky and culls
            // wrongly — ADR-0034 Phase 1).
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            Vec3 worldMin, worldMax;
            transformedAABB(model, bounds.boxMin, bounds.boxMax, worldMin, worldMax);
            if (!frustum.containsAABB(worldMin, worldMax)) {
                if (audit)
                    std::fprintf(stderr, "[draw-audit] e%u FRUSTUM-CULLED world "
                                 "(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
                                 entityIndex, worldMin.x, worldMin.y, worldMin.z,
                                 worldMax.x, worldMax.y, worldMax.z);
                return;
            }
            if (audit)
                std::fprintf(stderr, "[draw-audit] e%u DRAWN world "
                             "(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
                             entityIndex, worldMin.x, worldMin.y, worldMin.z,
                             worldMax.x, worldMax.y, worldMax.z);
            if (occ && !(r.material.flags & RenderMaterial::FLAG_OVERLAY)) {
                ++occTested;
                if (occludedBox(*occ, worldMin, worldMax, occSlack)) {
                    ++occCulled;
                    // hidden from the camera, but its shadow may fall where the camera sees
                    if (r.material.opacity >= 1.0f && !(r.material.flags & RenderMaterial::FLAG_GRASS)) {
                        RenderMaterial shadowOnly = r.material;
                        shadowOnly.flags |= RenderMaterial::FLAG_SHADOW_ONLY;
                        ctx.renderer.drawMesh(r.mesh, model, shadowOnly);
                    }
                    return;
                }
            }
            // HLOD distance policy (P1.2): detail chunks fade out past
            // drawDistance, their mass-box proxies fade IN at minDistance —
            // measured to the bounds centre so the pair swaps in lockstep.
            const Real drawDist = resolveDistance(r.drawDistance, r.drawClass);
            if (drawDist > 0 || r.minDistance > 0) {
                Vec3 c = (worldMin + worldMax) * 0.5;
                if (r.lodCell > 0) {
                    // the cell's centre, the camera's own height clamped into the chunk's span
                    c.x = (std::floor(c.x / r.lodCell) + 0.5) * r.lodCell;
                    c.z = (std::floor(c.z / r.lodCell) + 0.5) * r.lodCell;
                    c.y = std::clamp(cam.position.y, worldMin.y, worldMax.y);
                }
                const Real dist = (c - cam.position).length();
                const bool want = !(drawDist > 0 && dist > drawDist) && !(r.minDistance > 0 && dist <= r.minDistance);
                const bool fades = r.lodCell > 0 && tierFades && !(r.material.flags & (RenderMaterial::FLAG_LOD_BAND | RenderMaterial::FLAG_GRASS));
                if (!fades) {
                    if (!want) return;
                } else {
                    if (tierFade_.size() <= entityIndex) tierFade_.resize(entityIndex + 1);
                    TierFade& f = tierFade_[entityIndex];
                    // off screen last frame, or another drawable in this slot: no fade, it is simply what it is now
                    if (f.mesh != r.mesh.index || f.seen + 1 != frameNo_) f.a = want ? 1.0f : 0.0f;
                    else f.a = std::clamp(f.a + (want ? 1.0f : -1.0f) * fadeStep, 0.0f, 1.0f);
                    f.mesh = r.mesh.index;
                    f.seen = frameNo_;
                    if (f.a <= 0.0f) return;
                    if (f.a < 1.0f) {
                        RenderMaterial m = r.material;
                        tierFadeBand(m, dist, f.a, want);
                        ctx.renderer.drawMesh(r.mesh, model, m);
                        ++encEntDrawn;
                        ++encFading;
                        return;
                    }
                }
            }
            if (r.instanceData != 0.0f) model.m[3][0] = r.instanceData;   // (after the culls: data, not geometry)
            ctx.renderer.drawMesh(r.mesh, model, r.material);
            ++encEntDrawn;
        };

    // Instanced groups (static scatter — the forest). Cheap coarse reject on the
    // group's bounds, then PER-INSTANCE frustum cull so only the on-screen plants
    // draw (not the whole region) — then one instanced draw of the visible subset.
    auto drawGroup = [&](InstanceGroup& g) {

            if (g.transforms.empty()) return;
            if (g.renderLayer & ctx.renderer.hiddenLayers) return;   // debug layer hidden
            if (!frustum.containsSphere(g.boundsCenter, g.boundsRadius)) return;
            // Live override (slider) wins over the level's per-group value, which
            // in turn wins over the class policy. Lets draw distance be balanced
            // against fog at runtime.
            Real drawDist = ctx.renderer.vegetationDrawDistance > 0.0f
                                ? static_cast<Real>(ctx.renderer.vegetationDrawDistance)
                                : resolveDistance(g.drawDistance, g.drawClass);
            // Whole-group distance reject: skip if the nearest point of the group's
            // bounds is beyond the draw distance (cheap before the per-instance pass).
            if (drawDist > 0 &&
                (g.boundsCenter - cam.position).length() - g.boundsRadius > drawDist)
                return;
            bool hidden = false;
            if (occ) {
                ++grpTested;
                const Vec3 rr(g.boundsRadius, g.boundsRadius, g.boundsRadius);
                if (occludedBox(*occ, g.boundsCenter - rr, g.boundsCenter + rr, occSlack)) {
                    ++grpCulled;
                    hidden = true;
                    if (g.material.flags & RenderMaterial::FLAG_GRASS) return;   // grass casts no shadow
                }
            }
            BoundingSphere mb = ctx.renderer.getMeshBounds(g.mesh);
            ++encGrpIn;
            encInst += static_cast<long>(g.transforms.size());
            // MOVING: each instance drawn between its last step's pose and this one's (InstanceGroup::ids), as an
            // entity is between PrevTransform and Transform -- the same `alpha`, so a bus and the rider pinned in it
            // move together
            const std::vector<Mat4>* drawn = &g.transforms;
            static const bool noLerp = std::getenv("RT_NO_INSTANCE_LERP") != nullptr;   // the A/B: as baked
            if (!noLerp && !g.ids.empty() && g.ids.size() == g.transforms.size() && !g.prevTransforms.empty() &&
                g.prevIds.size() == g.prevTransforms.size() && alpha < 1.0) {
                lerpScratch_.resize(g.transforms.size());
                bool mapped = false;
                for (std::size_t i = 0; i < g.transforms.size(); ++i) {
                    const Mat4* from = nullptr;
                    if (i < g.prevIds.size() && g.prevIds[i] == g.ids[i]) {
                        from = &g.prevTransforms[i];
                    } else {
                        if (!mapped) {
                            prevById_.clear();
                            for (std::size_t j = 0; j < g.prevIds.size(); ++j) prevById_[g.prevIds[j]] = j;
                            mapped = true;
                        }
                        const auto it = prevById_.find(g.ids[i]);
                        if (it != prevById_.end()) from = &g.prevTransforms[it->second];
                    }
                    Mat4& out = lerpScratch_[i];
                    out = g.transforms[i];
                    if (!from) continue;   // new this step: where it is
                    // a step apart is a few centimetres and a degree or two: blending the matrix is the blend of
                    // the poses (the columns stay a rotation to well within a pixel); a jump (a placement, a
                    // teleport) is drawn where it landed
                    const Real jx = out.m[0][3] - from->m[0][3], jz = out.m[2][3] - from->m[2][3];
                    if (jx * jx + jz * jz > 25.0) continue;
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 4; ++c)
                            out.m[r][c] = from->m[r][c] + (out.m[r][c] - from->m[r][c]) * alpha;
                }
                drawn = &lerpScratch_;
            }
            frustumCullInstances(*drawn, frustum, mb.center, mb.radius,
                                 cam.position, drawDist, instanceScratch_, g.minDistance);
            encInstDrawn += static_cast<long>(instanceScratch_.size());
            if (instanceScratch_.empty()) return;
            if (hidden) {   // hidden from the camera, but its shadow may fall where the camera sees
                RenderMaterial shadowOnly = g.material;
                shadowOnly.flags |= RenderMaterial::FLAG_SHADOW_ONLY;
                ctx.renderer.drawMeshInstanced(g.mesh, instanceScratch_, shadowOnly);
            } else {
                ctx.renderer.drawMeshInstanced(g.mesh, instanceScratch_, g.material);
            }
        };
    // THE DRAW INDEX (Glenn: "see what kind of performance you can eke out"): the island visited ~27,000 entities
    // and ~50,000 instance groups a frame to draw ~340 and ~410 -- 6 ms of encode. Static drawables (terrain,
    // structure, scenery, furniture, ground paint) live in 200 m cells; a frame tests the cells and visits only
    // the members of the visible ones, plus a short list of the rest (bodies, effects, the unclassed). It keeps
    // itself right without help: each frame re-classifies a slice of the pools (new, moved, gone, reclassed), so
    // anything changes cell within a few frames; a member whose index no longer holds a drawable is dropped.
    static const bool noIndex = std::getenv("RT_NO_DRAW_INDEX") != nullptr;
    SparseSet<Transform>* tPool = ctx.world.componentPool<Transform>();
    SparseSet<PrevTransform>* pPool = ctx.world.componentPool<PrevTransform>();
    SparseSet<Renderable>* rPool = ctx.world.componentPool<Renderable>();
    SparseSet<InstanceGroup>* gPool = ctx.world.componentPool<InstanceGroup>();
    std::chrono::steady_clock::time_point encT1;
    if (noIndex || !tPool || !pPool || !rPool) {
        ctx.world.each<Transform, PrevTransform, Renderable>(
            [&](Entity e, Transform& t, PrevTransform& prev, Renderable& r) { drawEntity(e.index, t, prev, r); });
        encT1 = std::chrono::steady_clock::now();
        ctx.world.each<InstanceGroup>([&](Entity, InstanceGroup& g) { ++encGrp; drawGroup(g); });
    } else {
        drawIndex_.refresh(*tPool, *rPool, gPool, ctx.renderer, policy);
        auto entityAt = [&](uint32_t i) {
            Transform* t = tPool->get(i);
            PrevTransform* pv = t ? pPool->get(i) : nullptr;
            Renderable* r = pv ? rPool->get(i) : nullptr;
            if (r) drawEntity(i, *t, *pv, *r);
            return r != nullptr;
        };
        auto groupAt = [&](uint32_t i) {
            InstanceGroup* g = gPool ? gPool->get(i) : nullptr;
            if (g) { ++encGrp; drawGroup(*g); }
            return g != nullptr;
        };
        // a cell hidden whole behind nearer geometry (the occlusion depth), and too far for its members' shadows to
        // reach the cascades (150 m of view depth; 400 m leaves room for a tall building's evening shadow), is
        // skipped whole -- its members would only have drawn shadow-only
        auto cellOk = [&](const Vec3& lo, const Vec3& hi, Real dist) {
            if (!occ || dist < 400.0) return true;
            return !occludedBox(*occ, lo, hi, occSlack);
        };
        drawIndex_.visit(frustum, cam.position, static_cast<Real>(ctx.renderer.vegetationDrawDistance), entityAt, groupAt,
                         cellOk, encT1);
    }

    if (encStats) {
        const auto encT2 = std::chrono::steady_clock::now();
        encEntMs += std::chrono::duration<double, std::milli>(encT1 - encT0).count();
        encGrpMs += std::chrono::duration<double, std::milli>(encT2 - encT1).count();
        if (++encFrames == 120) {
            std::fprintf(stderr, "[encode] per frame: entities %.2f ms (%ld visited, %ld drawn) | instance groups %.2f ms (%ld groups, "
                         "%ld in view; %ld instances tested, %ld drawn); tier fades: %ld draws over the 120 frames\n", encEntMs / 120,
                         encEnt / 120, encEntDrawn / 120, encGrpMs / 120, encGrp / 120, encGrpIn / 120, encInst / 120, encInstDrawn / 120, encFading);
            encEntMs = encGrpMs = 0;
            encEnt = encEntDrawn = encGrp = encGrpIn = encInst = encInstDrawn = encFrames = encFading = 0;
        }
    }
    // RT_OCC_STATS=1: what the occlusion test saved, every 120 frames
    if (occStats) {
        static long frames = 0, t = 0, c = 0, gt = 0, gc = 0, used = 0;
        t += occTested; c += occCulled; gt += grpTested; gc += grpCulled; used += occ ? 1 : 0;
        if (++frames % 120 == 0) {
            std::fprintf(stderr, "[occlusion] %ld/120 frames used it; entities %ld/%ld culled, groups %ld/%ld culled (per frame)\n",
                         used, c / 120, t / 120, gc / 120, gt / 120);
            t = c = gt = gc = used = 0;
        }
    }
}

void RenderSystem::onStop(FrameContext& /*ctx*/) {
    // Exposure is owned by the level (cascade: defaults -> level -> runtime), so it
    // is no longer persisted to settings.json — that key used to stomp the level on
    // load. The Up/Down keys still ramp the live value within a session.
}

}  // namespace engine

