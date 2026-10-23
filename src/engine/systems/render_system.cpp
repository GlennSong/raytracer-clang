#include "render_system.h"
#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp: street lights share the headlight boundary
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
        // AFFINE, no perspective divide: an instance matrix's bottom row can carry DATA -- the grass
        // clump's thinning rank (GrassSystem) -- and transformPoint divided the world position by
        // (rank * centre.x + 1): 1.01-1.06 for a clump, which put its culling sphere 4-140 m away from
        // it. A wide forward view still caught the displaced spheres; looking down at your feet, the
        // grass under you was culled (#50, Glenn: "If I look down at the grass it disappears").
        const Vec3 center(m.m[0][0] * meshCenter.x + m.m[0][1] * meshCenter.y + m.m[0][2] * meshCenter.z + m.m[0][3],
                          m.m[1][0] * meshCenter.x + m.m[1][1] * meshCenter.y + m.m[1][2] * meshCenter.z + m.m[1][3],
                          m.m[2][0] * meshCenter.x + m.m[2][1] * meshCenter.y + m.m[2][2] * meshCenter.z + m.m[2][3]);
        const Real d2 = (center - cameraPos).lengthSquared();
        if (maxDistance > 0 && d2 > maxDistSq)
            continue;
        if (minDistance > 0 && d2 < minDistSq)   // (a nearer level of detail draws it there)
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
    // Street lamps become REAL point lights near the camera (device: "working
    // street lights"): the loader's StreetFurniture carries every bulb
    // position; each frame the nearest few join the light list on a COPY, so
    // level-authored point lights are never displaced or accumulated into.
    // NIGHT-GATED (WS3): the dusk ramp fades them in as the sun sinks — they
    // ran 24/7 before, invisible against daylight and stealing 14 of the 32
    // light slots at noon for nothing.
    {
        // `overlay lamplights off` kills the lamp POINT LIGHTS while leaving
        // their emissive heads lit — the live A/B for night artifacts.
        const bool lampLightsOn = ctx.settings.getBool("lamps.lights", true);
        const Real dusk =
            lampLightsOn ? duskRamp(ctx.view.lighting.solarElevation) : Real(0);
        bool lit = false;
        SceneLighting withLamps;
        // Headlight cones staged by VehicleSystem merge on the same copy —
        // they exist whenever headlights are lit (dusk by the shared ramp, or
        // the player's manual L toggle at noon).
        const auto& coneSpots = ctx.view.lighting.vehicleSpots;
        if (dusk > 0.01) {
            // The lamps' near tier from the level's light LOD block (defaults:
            // 20 lights within 160 m, range 34). 20 of the 32 slots: more lamps
            // only brighten the street if more of them are REAL lights, and 14
            // ran out within half a block. Leaves the sun/moon, 6 headlight
            // cones and authored lights room (the renderer loops every light
            // per fragment, so this is the honest cost knob for night).
            int kMaxLampLights = 20;
            Real lampRadius = 160.0, lampRange = 34.0;
            ctx.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
                kMaxLampLights = std::max(1, c.lampLightCount);
                lampRadius = std::max(Real(10), Real(c.lampLightRadius));
                lampRange = std::max(Real(2), Real(c.lampLightRange));
            });
            // the candidates: every street lamp's bulb (the first StreetFurniture's), and every station's canopy
            // lights (RailStop::lamps -- cooler, a shorter pool under a roof)
            struct Bulb { Real d2; Vec3 at; bool station; };
            std::vector<Bulb> nearBulbs;
            const Vec3 eye = ctx.view.camera.position;
            const Real maxDist2 = lampRadius * lampRadius;   // beyond this a bulb is the sprite tier's
            bool streetSeen = false;
            ctx.world.each<StreetFurniture>([&](Entity, StreetFurniture& f) {
                if (f.lampHeads.empty() || streetSeen) return;
                streetSeen = true;
                for (const Vec3& h : f.lampHeads) {
                    const Real d2 = (h - eye).lengthSquared();
                    if (d2 < maxDist2) nearBulbs.push_back({d2, h, false});
                }
            });
            ctx.world.each<RailTracks>([&](Entity, RailTracks& rt) {
                for (const RailTrack& t : rt.tracks)
                    for (const RailStop& st : t.stops)
                        for (const Vec3& h : st.lamps) {
                            const Real d2 = (h - eye).lengthSquared();
                            if (d2 < maxDist2) nearBulbs.push_back({d2, h, true});
                        }
            });
            if (!nearBulbs.empty()) {
                if (static_cast<int>(nearBulbs.size()) > kMaxLampLights) {
                    std::nth_element(nearBulbs.begin(),
                                     nearBulbs.begin() + kMaxLampLights,
                                     nearBulbs.end(),
                                     [](const Bulb& a, const Bulb& b) {
                                         return a.d2 < b.d2;
                                     });
                    nearBulbs.resize(kMaxLampLights);
                }
                // FADE AT THE BOUNDARY. The set is a hard "nearest N within
                // R" pick, so driving made lamps SNAP to full intensity the
                // moment they entered it and vanish when they left — the
                // "lights ahead turn on and it doesn't look seamless". Fade
                // each lamp out toward whichever boundary it will cross
                // first: the radius, or (when the count cap is binding) the
                // distance of the farthest lamp currently selected, which is
                // the edge that actually moves as the camera drives.
                Real cutoff = lampRadius;
                if (static_cast<int>(nearBulbs.size()) >= kMaxLampLights) {
                    Real farthest = 0;
                    for (const Bulb& bl : nearBulbs)
                        farthest = std::max(farthest, bl.d2);
                    cutoff = std::min(cutoff, std::sqrt(farthest));
                }
                withLamps = ctx.view.lighting;
                // Range 34 against 26 m spacing, so consecutive pools MEET —
                // at 30/34 they fell short and the pavement went dark between
                // every pair of lamps. Intensity rides the dusk ramp so lamps
                // warm up through twilight instead of popping.
                for (const Bulb& bl : nearBulbs) {
                    // 1 at the near end, easing to 0 across the last quarter
                    // of the reach, so a lamp joins and leaves the set
                    // invisibly instead of popping.
                    const Real d = std::sqrt(bl.d2);
                    const Real edge = std::max(Real(1), cutoff);
                    Real fade = (edge - d) / (0.25 * edge);
                    fade = std::min(Real(1), std::max(Real(0), fade));
                    fade = fade * fade * (3.0 - 2.0 * fade);   // smoothstep
                    if (bl.station) {   // a canopy fitting: cool white, a pool down the platform (12 m apart)
                        PointLight lamp(bl.at, Vec3(0.92, 0.96, 1.0), 26.0f * static_cast<float>(dusk * fade));
                        lamp.range = 16.0f;
                        withLamps.pointLights.push_back(lamp);
                        continue;
                    }
                    PointLight lamp(bl.at - Vec3(0, 0.35, 0),
                                    Vec3(1.0, 0.82, 0.55),
                                    48.0f * static_cast<float>(dusk * fade));
                    lamp.range = static_cast<float>(lampRange);
                    withLamps.pointLights.push_back(lamp);
                }
                lit = true;
            }
        }
        if (!coneSpots.empty()) {
            if (!lit) { withLamps = ctx.view.lighting; lit = true; }
            for (const SpotLight& s : coneSpots)
                withLamps.spotLights.push_back(s);
        }
        if (!ctx.view.lighting.toolSpots.empty()) {   // the flashlight (FlashlightSystem)
            if (!lit) { withLamps = ctx.view.lighting; lit = true; }
            for (const SpotLight& s : ctx.view.lighting.toolSpots)
                withLamps.spotLights.push_back(s);
        }
        // Interior room lights (ADR-0080), staged by BuildingInteriorSystem
        // for the streamed interior around the player — always on, not
        // dusk-gated: a roofed room is sun-blind at noon too.
        const auto& roomPoints = ctx.view.lighting.interiorPoints;
        if (!roomPoints.empty()) {
            if (!lit) { withLamps = ctx.view.lighting; lit = true; }
            for (const PointLight& p : roomPoints)
                withLamps.pointLights.push_back(p);
        }
        // Aviation-beacon lights (BeaconLightSystem): the nearest few lamps,
        // red, flashing — the near tier's light on the roof and the HVACs.
        const auto& beaconPoints = ctx.view.lighting.beaconPoints;
        if (!beaconPoints.empty()) {
            if (!lit) { withLamps = ctx.view.lighting; lit = true; }
            for (const PointLight& p : beaconPoints)
                withLamps.pointLights.push_back(p);
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
            frustumCullInstances(g.transforms, frustum, mb.center, mb.radius,
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

