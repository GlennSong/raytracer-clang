#include "xr_surface_system.h"

#include <cstdlib>

#include "../../log.h"
#include "../mesh_builder.h"
#include "physics_system.h"

namespace engine {

namespace {

// MeshHandle <-> the ledger's opaque token. The ledger stays renderer-free
// (and therefore host-testable) by never seeing a MeshHandle; a Handle is two
// uint32s, so it packs losslessly. generation != 0 for any valid handle, so a
// packed token is never the ledger's "no mesh" zero.
uint64_t packMesh(MeshHandle handle) {
    return (static_cast<uint64_t>(handle.generation) << 32) | handle.index;
}
MeshHandle unpackMesh(uint64_t token) {
    MeshHandle handle;
    handle.index = static_cast<uint32_t>(token & 0xffffffffu);
    handle.generation = static_cast<uint32_t>(token >> 32);
    return handle;
}

// An anchor's geometry as the renderer wants it: classification tint in the
// vertex color (the debug material's albedo stays white), flat normals
// accumulated from faces when the runtime sent none.
RenderMesh buildSurfaceMesh(const XrSurfaceUpdate& update) {
    RenderMesh mesh;
    const Vec3 tint = xrSurfaceClassColor(update.cls);
    const bool haveNormals = update.normals.size() == update.positions.size();

    mesh.vertices.reserve(update.positions.size());
    for (size_t i = 0; i < update.positions.size(); i++) {
        Vertex v(update.positions[i],
                 haveNormals ? update.normals[i] : Vec3(0, 0, 0));
        v.color = tint;
        mesh.vertices.push_back(v);
    }
    mesh.indices = update.indices;

    if (!haveNormals) {
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            Vertex& a = mesh.vertices[mesh.indices[t]];
            Vertex& b = mesh.vertices[mesh.indices[t + 1]];
            Vertex& c = mesh.vertices[mesh.indices[t + 2]];
            const Vec3 n = cross(b.position - a.position,
                                 c.position - a.position);
            a.normal += n;
            b.normal += n;
            c.normal += n;
        }
        for (Vertex& v : mesh.vertices) {
            const Real len = v.normal.length();
            v.normal = (len > 1e-12) ? v.normal / len : Vec3(0, 1, 0);
        }
    }
    return mesh;
}

Real worldScaleOf(FrameContext& ctx) {
    return (ctx.renderer.xrWorldScale > 0.01f)
        ? static_cast<Real>(ctx.renderer.xrWorldScale) : 1.0;
}

// Which surface classes the sandbox DRAWS: the horizontal, actionable ones.
// Walls/windows/doors/ceiling still ingest and collide, but painting them
// tinted every frame was most of the on-device visual noise.
bool sandboxShowsClass(XrSurfaceClass cls) {
    return cls == XrSurfaceClass::Floor || cls == XrSurfaceClass::Table ||
           cls == XrSurfaceClass::Seat || cls == XrSurfaceClass::Unknown;
}

// Collider refresh intervals, per anchor kind. Chunk cooks scale with
// triangle count and the furniture they model doesn't move; planes are tiny.
constexpr Real kPlaneColliderInterval = 2.0;
constexpr Real kChunkColliderInterval = 6.0;

// Reach for gaze placement, real metres. Generous enough for a far wall of a
// normal room, short enough that pinching at the far virtual landscape still
// means teleport.
constexpr Real kPlaceReachMetres = 3.0;
constexpr size_t kMaxMarkers = 64;

}  // namespace

XrSurfaceLedger::MeshOps XrSurfaceSystem::meshOps(FrameContext& ctx) {
    // Every surface keeps a GPU mesh again — sandbox chunks included, since
    // occlusion draws them depth-only every frame. The upload churn that
    // once made chunks worth skipping was fixed at the source (pose
    // dead-band + geometry gating + the 4-updates/frame ingest cap).
    return {
        [&ctx](const XrSurfaceUpdate& update) {
            return packMesh(ctx.renderer.uploadMesh(buildSurfaceMesh(update)));
        },
        [&ctx](uint64_t token) { ctx.renderer.removeMesh(unpackMesh(token)); },
    };
}

void XrSurfaceSystem::onStart(FrameContext&) {
    // Surface drawing is OFF by default (device feedback: the tinted fills
    // read as clutter once grabbing worked — the room itself is the visual).
    // RT_XR_SURFACES=1 shows them at boot; the settings toggle flips them
    // live. Ingest/colliders/occlusion/shadows are never gated by this.
    const char* env = std::getenv("RT_XR_SURFACES");
    visibleDefault_ = env && env[0] == '1';
    visible_ = visibleDefault_;
}

void XrSurfaceSystem::ingest(FrameContext& ctx) {
    // Drain everything, PROCESS a few. Each processed update re-uploads its
    // anchor's render mesh on the frame thread; a refinement burst (session
    // start delivers the whole room at once) used to spend tens of
    // milliseconds in one frame — the compositor reprojects the stale frame
    // and the world visibly drags and snaps (device: "loses anchoring when I
    // turn my head"). Four per frame absorbs the same burst over a dozen
    // frames instead.
    drainScratch_.clear();
    ctx.renderer.xrSurfaceStore()->drain(drainScratch_);
    pendingUpdates_.insert(pendingUpdates_.end(),
                           std::make_move_iterator(drainScratch_.begin()),
                           std::make_move_iterator(drainScratch_.end()));
    if (pendingUpdates_.empty()) return;

    constexpr size_t kMaxUpdatesPerFrame = 4;
    const size_t take = std::min(kMaxUpdatesPerFrame, pendingUpdates_.size());
    processScratch_.assign(std::make_move_iterator(pendingUpdates_.begin()),
                           std::make_move_iterator(pendingUpdates_.begin() + take));
    pendingUpdates_.erase(pendingUpdates_.begin(),
                          pendingUpdates_.begin() + take);

    const Real scale = worldScaleOf(ctx);
    for (XrSurfaceUpdate& u : processScratch_) {
        if (u.op == XrSurfaceUpdate::Op::Removed) {
            outlines_.erase(u.anchorId);
            planes_.erase(u.anchorId);
            if (physics_) {
                auto body = colliderBodies_.find(u.anchorId);
                if (body != colliderBodies_.end()) {
                    physics_->physicsWorld().removeBody(body->second);
                    colliderBodies_.erase(body);
                }
                colliderGeom_.erase(u.anchorId);
                colliderPolicy_.noteRemoved(u.anchorId);
            }
            // Orphan this anchor's markers at their last composed world pose
            // rather than deleting them: a marker that silently vanished
            // would read as "placement is broken", when the truth is "the
            // runtime merged/replaced that plane" — worth SEEING.
            auto ledgerIt = ledger_.surfaces().find(u.anchorId);
            if (ledgerIt != ledger_.surfaces().end()) {
                const Mat4 world = xrSurfaceWorldTransform(
                    ctx.xr.originBase, scale,
                    ledgerIt->second.originFromAnchor);
                int orphaned = 0;
                for (Marker& m : markers_) {
                    if (m.anchorId != u.anchorId) continue;
                    m.frozenWorld =
                        world * Mat4::translate(m.anchorPoint.x, m.anchorPoint.y,
                                                m.anchorPoint.z);
                    m.anchorId = 0;
                    orphaned++;
                }
                if (orphaned)
                    LOG_INFO << "[xr] plane removed; " << orphaned
                             << " marker(s) orphaned in place";
            }
            continue;
        }
        // Pose dead-band (the settle-toward-static half of a persistent room
        // model): ARKit micro-refines anchor poses continuously, which reads
        // as the whole surface JITTERING in place — and every accepted pose
        // ripples into re-uploads and re-cooks. A pose within 5mm / ~0.6° of
        // the one we hold is re-stamped with the OLD pose; a real move (new
        // extent, relocalisation) passes through untouched.
        bool poseSteady = false;
        {
            auto ledgerIt = ledger_.surfaces().find(u.anchorId);
            if (ledgerIt != ledger_.surfaces().end()) {
                const Mat4& oldM = ledgerIt->second.originFromAnchor;
                Real dt2 = 0, dr = 0;
                for (int r = 0; r < 3; r++) {
                    const Real d = u.originFromAnchor.m[r][3] - oldM.m[r][3];
                    dt2 += d * d;
                    for (int c = 0; c < 3; c++)
                        dr = std::max(dr,
                                      std::abs(u.originFromAnchor.m[r][c] -
                                               oldM.m[r][c]));
                }
                if (dt2 < 0.005 * 0.005 && dr < 0.01) {
                    u.originFromAnchor = oldM;
                    poseSteady = true;
                }
            }
        }

        // Colliders want EVERY surface with geometry — the reconstruction
        // chunks are the furniture. The cook itself is deferred to
        // rebuildDueColliders via the policy; here only the geometry is
        // kept. A steady pose with near-identical geometry (< 5% triangle
        // delta) doesn't even mark dirty: a mapped room settles to ZERO
        // steady-state cooking instead of rebuilding forever.
        if (physics_ && !u.indices.empty() && !u.positions.empty()) {
            ColliderGeometry& geom = colliderGeom_[u.anchorId];
            const size_t oldTris = geom.indices.size();
            geom.positions = u.positions;
            geom.indices = u.indices;
            const long delta =
                std::labs(static_cast<long>(geom.indices.size()) -
                          static_cast<long>(oldTris));
            const bool geomSteady =
                oldTris > 0 && delta * 20 < static_cast<long>(oldTris);
            if (!(poseSteady && geomSteady))
                colliderPolicy_.noteUpdate(u.anchorId, timeSeconds_,
                                           u.cls == XrSurfaceClass::Mesh
                                               ? kChunkColliderInterval
                                               : kPlaneColliderInterval);
        }

        if (u.cls == XrSurfaceClass::Mesh || u.indices.empty()) continue;

        // Plane bookkeeping: outline for drawing, triangles for placement
        // raycasts, extent for the dimensions readout.
        auto& segments = outlines_[u.anchorId];
        segments.clear();
        for (const auto& [ia, ib] : xrSurfaceBoundaryEdges(u.indices))
            segments.emplace_back(u.positions[ia], u.positions[ib]);

        PlaneGeometry& plane = planes_[u.anchorId];
        const Vec3 oldExtent = plane.extent;
        plane.positions = u.positions;
        plane.indices = u.indices;
        plane.extent = xrSurfaceExtent(u.positions).size();
        // The dimensions display, by log line (the engine draws no 3D text):
        // announced when a plane appears and whenever refinement moves a
        // dimension by more than 5 cm, so the console shows sizes settling
        // without spamming every refinement tick.
        if (std::abs(plane.extent.x - oldExtent.x) > 0.05 ||
            std::abs(plane.extent.z - oldExtent.z) > 0.05) {
            LOG_INFO("[xr] plane %s %.2fm x %.2fm (id %llx)",
                     xrSurfaceClassName(u.cls), plane.extent.x, plane.extent.z,
                     static_cast<unsigned long long>(u.anchorId));
        }
    }
    ledger_.apply(processScratch_, meshOps(ctx));
}

void XrSurfaceSystem::placeOnPinch(FrameContext& ctx) {
    // Sandbox mode (outline display): no gaze placement at all — the hand
    // palette owns spawning there, and a pinch must only ever mean grab or
    // palette pick. The arena keeps its marker probe.
    if (!fillSurfaces_) return;
    // Quick pinch, same window PlayerSystem calls a teleport (< 0.8 s hold).
    if (!ctx.xr.pinchEnded || ctx.xr.pinchHeldSeconds >= 0.8) return;
    if (!ctx.xr.gazeValid || !ctx.xr.originBaseValid) return;

    const Real scale = worldScaleOf(ctx);
    const Vec3 rayOrigin = ctx.xr.originBase + ctx.xr.gazeOrigin;
    const Vec3 rayDir = ctx.xr.gazeDir;

    // Nearest plane hit within reach. t comes back in real metres for every
    // plane (the anchor spaces are unscaled), so hits compare directly.
    bool found = false;
    Real bestT = kPlaceReachMetres;
    uint64_t bestAnchor = 0;
    Vec3 bestPoint;
    for (const auto& [anchorId, plane] : planes_) {
        auto ledgerIt = ledger_.surfaces().find(anchorId);
        if (ledgerIt == ledger_.surfaces().end()) continue;
        const Mat4 world = xrSurfaceWorldTransform(
            ctx.xr.originBase, scale, ledgerIt->second.originFromAnchor);
        const Mat4 inv = world.inverse();
        Vec3 o = inv.transformPoint(rayOrigin);
        Vec3 d = inv.transformDirection(rayDir);
        const Real dLen = d.length();
        if (dLen < 1e-12) continue;
        d /= dLen;
        Real t = 0;
        if (!xrRaycastTriangles(o, d, plane.positions, plane.indices, t))
            continue;
        if (t < bestT) {
            found = true;
            bestT = t;
            bestAnchor = anchorId;
            bestPoint = o + d * t;
        }
    }
    if (!found) return;   // pinch stays; PlayerSystem may teleport with it

    // CONSUME the pinch so this gesture is a placement, not also a teleport.
    // ctx.xr is the frame's shared state and this system runs before
    // PlayerSystem precisely so this write is seen there.
    ctx.xr.pinchEnded = false;

    const auto& cls = ledger_.surfaces().at(bestAnchor).cls;

    if (physics_) {
        // Physics mode (the sandbox): drop a dynamic object from 25 real cm
        // above the hit — the visible, audible proof that the pinched
        // surface's collider is really there to catch it.
        const Mat4 world = xrSurfaceWorldTransform(
            ctx.xr.originBase, scale,
            ledger_.surfaces().at(bestAnchor).originFromAnchor);
        const Vec3 spawn = world.transformPoint(bestPoint + Vec3(0, 0.25, 0));
        if (dropSpawner_) {
            dropSpawner_(ctx, spawn);
            LOG_INFO("[xr] gaze drop onto %s at %.2fm (anchor %llx)",
                     xrSurfaceClassName(cls), bestT,
                     static_cast<unsigned long long>(bestAnchor));
            return;
        }
        const PhysicsBodyId id = physics_->physicsWorld().addBox(
            Vec3(0.05, 0.05, 0.05), spawn, Quat::identity(),
            BodyMotion::Dynamic, /*restitution=*/0.25, /*friction=*/0.5);
        if (id != INVALID_PHYSICS_BODY) {
            if (dropCubes_.size() >= kMaxMarkers) {
                physics_->physicsWorld().removeBody(dropCubes_.front());
                dropCubes_.erase(dropCubes_.begin());
            }
            dropCubes_.push_back(id);
            LOG_INFO("[xr] dropped cube %zu onto %s at %.2fm (anchor %llx)",
                     dropCubes_.size(), xrSurfaceClassName(cls), bestT,
                     static_cast<unsigned long long>(bestAnchor));
        }
        return;
    }

    Marker marker;
    marker.anchorId = bestAnchor;
    marker.anchorPoint = bestPoint;
    if (markers_.size() >= kMaxMarkers) markers_.erase(markers_.begin());
    markers_.push_back(marker);

    LOG_INFO("[xr] placed marker %zu on %s at %.2fm (anchor %llx)",
             markers_.size(), xrSurfaceClassName(cls), bestT,
             static_cast<unsigned long long>(bestAnchor));
}

void XrSurfaceSystem::rebuildDueColliders(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.originBaseValid) return;

    // World-space colliders bake origin + scale in; if either moved (teleport
    // locomotion, a scale change), every live collider is somewhere the room
    // no longer is. Invalidate them all — the policy staggers the rebuild.
    const Real scale = worldScaleOf(ctx);
    const bool baked = colliderScale_ > 0;
    if (baked && ((ctx.xr.originBase - colliderOrigin_).length() > 1e-3 ||
                  std::abs(scale - colliderScale_) > 1e-6)) {
        colliderPolicy_.invalidateAll();
        LOG_INFO("[xr] origin/scale moved; room colliders invalidated");
    }
    colliderOrigin_ = ctx.xr.originBase;
    colliderScale_ = scale;

    // One cook per frame: a 6-25k-triangle chunk's MeshShape cook runs
    // milliseconds; several in one frame is a visible hitch (see ingest).
    std::vector<Vec3> worldVerts;
    for (uint64_t anchorId : colliderPolicy_.drainDue(timeSeconds_, 1)) {
        auto geom = colliderGeom_.find(anchorId);
        auto entry = ledger_.surfaces().find(anchorId);
        if (geom == colliderGeom_.end() || entry == ledger_.surfaces().end())
            continue;
        const Mat4 world = xrSurfaceWorldTransform(
            ctx.xr.originBase, scale, entry->second.originFromAnchor);
        worldVerts.clear();
        worldVerts.reserve(geom->second.positions.size());
        for (const Vec3& p : geom->second.positions)
            worldVerts.push_back(world.transformPoint(p));

        auto body = colliderBodies_.find(anchorId);
        if (body != colliderBodies_.end())
            physics_->physicsWorld().removeBody(body->second);
        // BOTH windings of every triangle. Jolt mesh triangles are solid from
        // one side (their winding's normal side), and addMesh flips winding
        // assuming the ENGINE's clockwise convention — which ARKit does not
        // follow, so single-sided room colliders came out solid from BELOW
        // and dropped cubes fell straight through the real floor (device
        // log, first sandbox session). Doubling the triangles makes every
        // surface solid from both sides for 2x cook cost — the room is
        // static scenery, and 100k one-sided tris is still small for Jolt.
        const std::vector<uint32_t>& src = geom->second.indices;
        std::vector<uint32_t> twoSided;
        twoSided.reserve(src.size() * 2);
        twoSided.insert(twoSided.end(), src.begin(), src.end());
        for (size_t i = 0; i + 2 < src.size(); i += 3) {
            twoSided.push_back(src[i]);
            twoSided.push_back(src[i + 2]);
            twoSided.push_back(src[i + 1]);
        }
        const PhysicsBodyId id = physics_->physicsWorld().addMesh(
            worldVerts, twoSided, Vec3(0, 0, 0), 0.6);
        if (id != INVALID_PHYSICS_BODY) colliderBodies_[anchorId] = id;
        else colliderBodies_.erase(anchorId);
    }
}

void XrSurfaceSystem::update(FrameContext& ctx) {
    XrSurfaceStore* store = ctx.renderer.xrSurfaceStore();
    if (!store) return;

    // Runtime display controls (settings panel / prefs); env var is the boot
    // default. Ingest, colliders, occlusion and shadow catching are never
    // gated by visibility — visibility is only the debug drawing.
    visible_ = ctx.settings.getBool("xr.showSurfaces", visibleDefault_);
    surfaceOpacity_ = ctx.settings.getDouble("xr.surfaceOpacity", 0.5);
    showNormals_ = ctx.settings.getBool("xr.showNormals", false);
    // One log line whenever the display state CHANGES (and once at start):
    // "why am I seeing surface shapes" must be answerable from the console
    // — a saved settings.json can override the compile-time defaults, and
    // that mismatch cost a device round.
    const int displayState = (visible_ ? 1 : 0) | (showNormals_ ? 2 : 0) |
                             (ctx.settings.getBool("xr.shadows", true) ? 4 : 0);
    if (displayState != loggedDisplayState_ ||
        std::fabs(surfaceOpacity_ - loggedOpacity_) > 0.005) {
        loggedDisplayState_ = displayState;
        loggedOpacity_ = surfaceOpacity_;
        LOG_INFO("[xr] surfaces display: show=%d opacity=%.2f shadows=%d normals=%d",
                 visible_ ? 1 : 0, surfaceOpacity_,
                 (displayState & 4) ? 1 : 0, showNormals_ ? 1 : 0);
    }
    shadowsEnabled_ = ctx.settings.getBool("xr.shadows", true);
    if (!fillSurfaces_) {
        // Depth view (sandbox): the composite's depth debug mode over the
        // whole display — with occluders writing room depth, this is a live
        // depth map of the real room plus the virtual objects in it.
        const bool depthView = ctx.settings.getBool("xr.depthView", false);
        if (depthView != lastDepthView_) {
            ctx.renderer.debugView = depthView ? 3 : 0;
            lastDepthView_ = depthView;
        }
    }
    timeSeconds_ += ctx.frameDelta;
    ingest(ctx);
    rebuildDueColliders(ctx);
    if (visible_ && ctx.xr.active) placeOnPinch(ctx);

    // The numeric readout — how surface mapping is verified without seeing the
    // render. Logged when the census changes shape and as a slow heartbeat.
    const auto census = ledger_.census();
    const bool beat = (++frame_ % 450) == 0;   // ~5 s at 90 Hz
    if (census.total != lastLoggedTotal_ || (beat && census.total > 0)) {
        lastLoggedTotal_ = census.total;
        auto line = LOG_INFO;
        line << "[xr] surfaces: total=" << census.total;
        for (int i = 0; i < XR_SURFACE_CLASS_COUNT; i++) {
            if (census.countByClass[i] == 0) continue;
            line << " " << xrSurfaceClassName(static_cast<XrSurfaceClass>(i))
                 << "=" << census.countByClass[i];
        }
        line << " tris=" << census.triangles
             << " markers=" << markers_.size();
        if (physics_)
            line << " colliders=" << colliderBodies_.size()
                 << " pending=" << colliderPolicy_.pendingCount();
        if (census.floorValid) line << " floorY=" << census.floorY << "m";
    }
}

void XrSurfaceSystem::render(FrameContext& ctx) {
    if (!ctx.xr.active || !ctx.xr.originBaseValid) return;

    const Real scale = worldScaleOf(ctx);

    // Presence layer (sandbox only), independent of the debug display
    // toggle — occlusion and shadows are features, not diagnostics:
    // - EVERY surface (chunks + all plane classes) draws as a depth-only
    //   OCCLUDER, so real furniture hides virtual objects and the
    //   compositor gets true room depth.
    // - The horizontal planes also draw as SHADOW CATCHERS: virtual objects
    //   cast the noon sun's shadows onto the real floor and table.
    if (!fillSurfaces_) {
        RenderMaterial occluder;
        occluder.flags = RenderMaterial::FLAG_OCCLUDER;
        RenderMaterial catcher;
        catcher.flags = RenderMaterial::FLAG_SHADOW_CATCHER |
                        RenderMaterial::FLAG_STIPPLE |
                        RenderMaterial::FLAG_TWO_SIDED;
        for (const auto& [anchorId, entry] : ledger_.surfaces()) {
            if (!entry.meshToken) continue;
            const Mat4 world = xrSurfaceWorldTransform(
                ctx.xr.originBase, scale, entry.originFromAnchor);
            ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world,
                                  occluder);
            // Catchers only on genuinely HORIZONTAL planes (anchor +Y near
            // world up): a catcher near-parallel to the noon sun fails the
            // shadow test at grazing angles and renders its WHOLE plane as
            // a dithered ghost — the "outlines with surfaces off" report.
            // Class alone was not enough: rooms grow dozens of Unknown
            // planes, many vertical-ish.
            const Vec3 planeUp = normalize(Vec3(
                entry.originFromAnchor.m[0][1], entry.originFromAnchor.m[1][1],
                entry.originFromAnchor.m[2][1]));
            if (shadowsEnabled_ && entry.cls != XrSurfaceClass::Mesh &&
                sandboxShowsClass(entry.cls) && planeUp.y > 0.85)
                ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world,
                                      catcher);
        }
    }

    if (!visible_) return;
    if (ledger_.surfaces().empty() && markers_.empty() && dropCubes_.empty())
        return;

    RenderMaterial material;
    material.albedo = Vec3(1, 1, 1);   // tint rides Vertex::color
    material.metallic = 0.0f;
    material.roughness = 1.0f;
    material.flags = RenderMaterial::FLAG_TWO_SIDED;

    for (const auto& [anchorId, entry] : ledger_.surfaces()) {
        // Sandbox display: only the horizontal, actionable surfaces draw —
        // floor/table/seat/unknown as stippled translucent fills the room
        // stays visible through. Walls, windows, doors and the ceiling were
        // most of the on-device noise ('I'm not sure what all these planes
        // mean'); they keep ingesting and colliding, invisibly.
        const bool shown =
            fillSurfaces_ || (entry.cls != XrSurfaceClass::Mesh &&
                              sandboxShowsClass(entry.cls));
        if (!shown) continue;
        const Mat4 world = xrSurfaceWorldTransform(ctx.xr.originBase, scale,
                                                   entry.originFromAnchor);
        if (entry.meshToken) {
            if (fillSurfaces_) {
                ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world,
                                      material);
            } else if (surfaceOpacity_ > 0.01) {
                RenderMaterial fill = material;
                fill.flags |= RenderMaterial::FLAG_STIPPLE;
                // The opacity slider dims the tint (the stipple pattern
                // already lets the room through); 0 removes fills entirely.
                fill.albedo = material.albedo * surfaceOpacity_ * 2.0;
                ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world,
                                      fill);
            }
        }

        auto outline = outlines_.find(anchorId);
        if (outline != outlines_.end()) {
            const Vec3 color = xrSurfaceClassColor(entry.cls);
            if (fillSurfaces_) {
                for (const auto& [a, b] : outline->second)
                    ctx.debug.line(world.transformPoint(a),
                                   world.transformPoint(b), color);
            }
            // Normal tick: ARKit planes face +Y in anchor space. 10 real cm.
            // Off by default now — a settings toggle brings them back.
            if (showNormals_) {
                const Vec3 foot = world.transformPoint(Vec3(0, 0, 0));
                const Vec3 tip = world.transformPoint(Vec3(0, 0.1, 0));
                ctx.debug.line(foot, tip, color);
            }
        }

        // Extent rectangle, dimmed: the measured bounding box the dimension
        // log lines refer to, so "2.40m x 1.95m" has a visible referent.
        auto plane = planes_.find(anchorId);
        if (plane != planes_.end()) {
            const auto extent = xrSurfaceExtent(plane->second.positions);
            if (extent.valid) {
                const Vec3 dim = xrSurfaceClassColor(entry.cls) * 0.45;
                const Vec3 c[4] = {
                    world.transformPoint(Vec3(extent.min.x, 0, extent.min.z)),
                    world.transformPoint(Vec3(extent.max.x, 0, extent.min.z)),
                    world.transformPoint(Vec3(extent.max.x, 0, extent.max.z)),
                    world.transformPoint(Vec3(extent.min.x, 0, extent.max.z)),
                };
                for (int i = 0; i < 4; i++)
                    ctx.debug.line(c[i], c[(i + 1) % 4], dim);
            }
        }
    }

    // Drop cubes (physics mode): drawn at the pose Jolt says, which IS the
    // verification — a cube resting flush on the real table means the
    // collider matches the surface.
    if (!dropCubes_.empty() && physics_) {
        if (!markerMesh_.valid())
            markerMesh_ = ctx.renderer.uploadMesh(
                MeshBuilder::box(Vec3(0.1, 0.1, 0.1)));
        RenderMaterial cubeMaterial;
        cubeMaterial.albedo = Vec3(0.95, 0.55, 0.15);   // orange: dynamic
        cubeMaterial.metallic = 0.0f;
        cubeMaterial.roughness = 0.5f;
        for (PhysicsBodyId id : dropCubes_) {
            const Vec3 p = physics_->physicsWorld().bodyPosition(id);
            const Mat4 world = Mat4::translate(p.x, p.y, p.z) *
                               physics_->physicsWorld().bodyOrientation(id).toMat4();
            ctx.renderer.drawMesh(markerMesh_, world, cubeMaterial);
        }
    }

    // Markers: 10 real cm cubes, magenta, riding their plane's anchor — the
    // live probe of anchoring quality. Orphans draw at their frozen pose.
    if (!markers_.empty()) {
        if (!markerMesh_.valid())
            markerMesh_ = ctx.renderer.uploadMesh(
                MeshBuilder::box(Vec3(0.1, 0.1, 0.1)));
        RenderMaterial markerMaterial;
        markerMaterial.albedo = Vec3(0.95, 0.2, 0.9);
        markerMaterial.metallic = 0.0f;
        markerMaterial.roughness = 0.6f;
        const Mat4 lift = Mat4::translate(0, 0.05, 0);   // sit ON the surface
        for (const Marker& m : markers_) {
            Mat4 world;
            if (m.anchorId == 0) {
                world = m.frozenWorld * lift;
            } else {
                auto it = ledger_.surfaces().find(m.anchorId);
                if (it == ledger_.surfaces().end()) continue;
                world = xrSurfaceWorldTransform(ctx.xr.originBase, scale,
                                                it->second.originFromAnchor)
                      * Mat4::translate(m.anchorPoint.x, m.anchorPoint.y,
                                        m.anchorPoint.z)
                      * lift;
            }
            ctx.renderer.drawMesh(markerMesh_, world, markerMaterial);
        }
    }
}

void XrSurfaceSystem::onStop(FrameContext& ctx) {
    if (physics_) {
        for (const auto& [anchorId, body] : colliderBodies_)
            physics_->physicsWorld().removeBody(body);
        for (PhysicsBodyId id : dropCubes_)
            physics_->physicsWorld().removeBody(id);
    }
    dropCubes_.clear();
    colliderBodies_.clear();
    colliderGeom_.clear();
    colliderPolicy_ = XrColliderPolicy{};
    colliderScale_ = 0;
    ledger_.clear(meshOps(ctx));
    outlines_.clear();
    planes_.clear();
    markers_.clear();
    pendingUpdates_.clear();
    processScratch_.clear();
    if (markerMesh_.valid()) {
        ctx.renderer.removeMesh(markerMesh_);
        markerMesh_ = MeshHandle{};
    }
    lastLoggedTotal_ = -1;
}

}  // namespace engine
