#include "render_system.h"
#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp: street lights share the headlight boundary
#include "../../profile.h"

#include <algorithm>
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
                          std::vector<Mat4>& out) {
    out.clear();
    Real maxDistSq = maxDistance * maxDistance;
    for (const Mat4& m : transforms) {
        Vec3 center = m.transformPoint(meshCenter);
        if (maxDistance > 0 && (center - cameraPos).lengthSquared() > maxDistSq)
            continue;
        Vec3 cx(m.m[0][0], m.m[1][0], m.m[2][0]);
        Vec3 cy(m.m[0][1], m.m[1][1], m.m[2][1]);
        Vec3 cz(m.m[0][2], m.m[1][2], m.m[2][2]);
        Real maxScale = std::max({cx.length(), cy.length(), cz.length()});
        if (frustum.containsSphere(center, meshRadius * maxScale))
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
    // Street lamps become REAL point lights near the camera (device: "working
    // street lights"): the loader's StreetFurniture carries every bulb
    // position; each frame the nearest few join the light list on a COPY, so
    // level-authored point lights are never displaced or accumulated into.
    // NIGHT-GATED (WS3): the dusk ramp fades them in as the sun sinks — they
    // ran 24/7 before, invisible against daylight and stealing 14 of the 32
    // light slots at noon for nothing.
    {
        const Real dusk = duskRamp(ctx.view.lighting.solarElevation);
        bool lit = false;
        SceneLighting withLamps;
        // Headlight cones staged by VehicleSystem merge on the same copy —
        // they exist whenever headlights are lit (dusk by the shared ramp, or
        // the player's manual L toggle at noon).
        const auto& coneSpots = ctx.view.lighting.vehicleSpots;
        if (dusk > 0.01) {
            ctx.world.each<StreetFurniture>([&](Entity, StreetFurniture& f) {
                if (f.lampHeads.empty() || lit) return;
                const Vec3 eye = ctx.view.camera.position;
                // 20 of the 32 slots: more lamps only brighten the street if
                // more of them are REAL lights, and 14 ran out within half a
                // block. Leaves the sun/moon, 6 headlight cones and authored
                // lights room (the renderer loops every light per fragment,
                // so this is the honest cost knob for night).
                constexpr int kMaxLampLights = 20;
                const Real maxDist2 = 160.0 * 160.0;   // beyond this a bulb is subpixel
                std::vector<std::pair<Real, const Vec3*>> nearBulbs;
                for (const Vec3& h : f.lampHeads) {
                    const Real d2 = (h - eye).lengthSquared();
                    if (d2 < maxDist2) nearBulbs.emplace_back(d2, &h);
                }
                if (nearBulbs.empty()) return;
                if (static_cast<int>(nearBulbs.size()) > kMaxLampLights) {
                    std::nth_element(nearBulbs.begin(),
                                     nearBulbs.begin() + kMaxLampLights,
                                     nearBulbs.end(),
                                     [](const auto& a, const auto& b) {
                                         return a.first < b.first;
                                     });
                    nearBulbs.resize(kMaxLampLights);
                }
                withLamps = ctx.view.lighting;
                // Range 34 against 26 m spacing, so consecutive pools MEET —
                // at 30/34 they fell short and the pavement went dark between
                // every pair of lamps. Intensity rides the dusk ramp so lamps
                // warm up through twilight instead of popping.
                for (const auto& [d2, h] : nearBulbs) {
                    PointLight lamp(*h - Vec3(0, 0.35, 0),
                                    Vec3(1.0, 0.82, 0.55),
                                    48.0f * static_cast<float>(dusk));
                    lamp.range = 34.0f;
                    withLamps.pointLights.push_back(lamp);
                }
                lit = true;
            });
        }
        if (!coneSpots.empty()) {
            if (!lit) { withLamps = ctx.view.lighting; lit = true; }
            for (const SpotLight& s : coneSpots)
                withLamps.spotLights.push_back(s);
        }
        ctx.renderer.setLights(lit ? withLamps : ctx.view.lighting);
    }

    const auto& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                              cam.nearPlane, cam.farPlane);
    Frustum frustum = Frustum::fromViewProjection(proj * view);

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
    ctx.world.each<Transform, PrevTransform, Renderable>(
        [&](Entity entity, Transform& t, PrevTransform& prev, Renderable& r) {
            if (entity == ctx.view.activeCameraEntity) return;
            if (audit) {
                BoundingSphere ab = ctx.renderer.getMeshBounds(r.mesh);
                std::fprintf(stderr,
                             "[draw-audit] e%u mesh %u layer %u hidden %d aabb "
                             "(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
                             entity.index, r.mesh.index, r.renderLayer,
                             (r.renderLayer & ctx.renderer.hiddenLayers) ? 1 : 0,
                             ab.boxMin.x, ab.boxMin.y, ab.boxMin.z, ab.boxMax.x,
                             ab.boxMax.y, ab.boxMax.z);
            }
            if (r.renderLayer & ctx.renderer.hiddenLayers) return;   // debug layer hidden
            // Interpolated local matrix, then composed through any parent
            // chain (editor only — PLAY flattens parenting at load, so there
            // parentId is 0 and this is just the interpolated local).
            Mat4 model = lerp(prev.value, t, alpha).matrix();
            SourceSpec* s = ctx.world.get<SourceSpec>(entity);
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
                                 entity.index, worldMin.x, worldMin.y, worldMin.z,
                                 worldMax.x, worldMax.y, worldMax.z);
                return;
            }
            if (audit)
                std::fprintf(stderr, "[draw-audit] e%u DRAWN world "
                             "(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
                             entity.index, worldMin.x, worldMin.y, worldMin.z,
                             worldMax.x, worldMax.y, worldMax.z);
            // HLOD distance policy (P1.2): detail chunks fade out past
            // drawDistance, their mass-box proxies fade IN at minDistance —
            // measured to the bounds centre so the pair swaps in lockstep.
            const Real drawDist = resolveDistance(r.drawDistance, r.drawClass);
            if (drawDist > 0 || r.minDistance > 0) {
                const Vec3 c = (worldMin + worldMax) * 0.5;
                const Real dist = (c - cam.position).length();
                if (drawDist > 0 && dist > drawDist) return;
                if (r.minDistance > 0 && dist <= r.minDistance) return;
            }
            ctx.renderer.drawMesh(r.mesh, model, r.material);
        });

    // Instanced groups (static scatter — the forest). Cheap coarse reject on the
    // group's bounds, then PER-INSTANCE frustum cull so only the on-screen plants
    // draw (not the whole region) — then one instanced draw of the visible subset.
    ctx.world.each<InstanceGroup>(
        [&](Entity, InstanceGroup& g) {
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
            BoundingSphere mb = ctx.renderer.getMeshBounds(g.mesh);
            frustumCullInstances(g.transforms, frustum, mb.center, mb.radius,
                                 cam.position, drawDist, instanceScratch_);
            if (!instanceScratch_.empty())
                ctx.renderer.drawMeshInstanced(g.mesh, instanceScratch_, g.material);
        });
}

void RenderSystem::onStop(FrameContext& /*ctx*/) {
    // Exposure is owned by the level (cascade: defaults -> level -> runtime), so it
    // is no longer persisted to settings.json — that key used to stomp the level on
    // load. The Up/Down keys still ramp the live value within a session.
}

}  // namespace engine

