#ifndef RAYTRACER_ENGINE_XR_SURFACE_SYSTEM_H
#define RAYTRACER_ENGINE_XR_SURFACE_SYSTEM_H

#include <map>
#include <vector>

#include "../physics/physics_world.h"   // PhysicsBodyId
#include "../system.h"
#include "../xr/xr_surfaces.h"

namespace engine {

class PhysicsSystem;

// Draws what ARKit knows about the user's real room. Drains the renderer's
// XrSurfaceStore once a frame (nullptr everywhere but a visionOS device, so
// the system is inert on desktop, in tests, and in the simulator), feeds the
// XrSurfaceLedger, and renders the result: reconstruction chunks as
// classification-tinted translucent-ish meshes, detected planes additionally
// outlined with debug lines and a normal tick.
//
// The mesh lifecycle (one live mesh per anchor, updates replace, removes
// destroy) is the ledger's, tested in test_xr_surfaces.cpp; this system only
// packs MeshHandle into the ledger's opaque tokens and turns census numbers
// into the periodic `[xr] surfaces:` log line — the readout that lets surface
// mapping be verified from a console instead of by eye.
//
// Visibility: RT_XR_SURFACES=0 in the environment disables drawing (data is
// still ingested, so flipping it back on shows the full room, not a restart).
//
// Placement demo: a QUICK pinch while gazing at a detected plane within reach
// (~3 real metres) drops a 10 cm marker cube on it — and CONSUMES the pinch,
// so PlayerSystem's teleport never sees it. That interception is why this
// system registers BEFORE PlayerSystem in arena_state.cpp; gazing past every
// plane leaves the pinch alone and teleport behaves as always. Markers are
// stored in ANCHOR space, keyed by the plane's anchor id: when ARKit refines
// or re-poses the plane, its markers ride along — which makes them a live
// probe of how well surfaces anchor. A marker whose plane is removed keeps
// its last world pose, orphaned (logged), so evidence never silently vanishes.
// Physics (optional): constructed with a PhysicsSystem, every surface — the
// planes AND the reconstruction chunks — also gets a static Jolt mesh
// collider in world space, so dynamic bodies land on the real floor and real
// furniture. Rebuilds ride XrColliderPolicy (first build immediate, refresh
// throttled per anchor); an XR origin or world-scale change invalidates
// everything, since world-space colliders bake both in. Null physics (the
// default) keeps the system render-only.
// Display: fillSurfaces=false draws OUTLINES ONLY — no filled plane meshes,
// no reconstruction chunks. The sandbox uses it: passthrough already shows
// the real room, so opaque fills were 40+ overlapping tinted sheets stacked
// over walls and windows (device feedback: "messy as hell"), where thin
// class-colored outlines + extent rectangles read cleanly over video.
// EVERYTHING still ingests and colliders still build — visibility and
// physics are independent.
class XrSurfaceSystem : public System {
public:
    explicit XrSurfaceSystem(PhysicsSystem* physics = nullptr,
                             bool fillSurfaces = true)
        : physics_(physics), fillSurfaces_(fillSurfaces) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    XrSurfaceLedger ledger_;
    std::vector<XrSurfaceUpdate> drainScratch_;
    // Anchor-space outline segments per plane anchor, cached at ingest so the
    // per-frame render cost is a transform + line per segment, not an edge
    // count over the mesh.
    std::map<uint64_t, std::vector<std::pair<Vec3, Vec3>>> outlines_;
    // Retained anchor-space triangles per PLANE anchor (never the room mesh:
    // tens of triangles each, cheap to keep and to raycast linearly). This is
    // what gaze placement intersects.
    struct PlaneGeometry {
        std::vector<Vec3> positions;
        std::vector<uint32_t> indices;
        Vec3 extent;              // anchor-space size (x by z = printed dims)
    };
    std::map<uint64_t, PlaneGeometry> planes_;
    // Placed markers. anchorId = 0 marks an orphan frozen at originFromWorld.
    struct Marker {
        uint64_t anchorId = 0;
        Vec3 anchorPoint;         // anchor space, real metres
        Mat4 frozenWorld;         // orphans only: last composed world pose
    };
    std::vector<Marker> markers_;
    // Physics mode only: quick-pinch drops a DYNAMIC cube instead of a static
    // marker — the on-device proof that room colliders work (it should thud
    // onto the real table and can be knocked to the real floor). Poses are
    // read back from Jolt each frame.
    std::vector<PhysicsBodyId> dropCubes_;
    MeshHandle markerMesh_;
    uint64_t frame_ = 0;
    bool visible_ = true;
    bool fillSurfaces_ = true;
    int lastLoggedTotal_ = -1;

    // Room colliders (only populated when physics_ != nullptr).
    PhysicsSystem* physics_ = nullptr;
    XrColliderPolicy colliderPolicy_;
    // Anchor-space geometry retained for cooking. Separate from planes_
    // because colliders also cover Mesh chunks, which planes_ deliberately
    // never stores.
    struct ColliderGeometry {
        std::vector<Vec3> positions;
        std::vector<uint32_t> indices;
    };
    std::map<uint64_t, ColliderGeometry> colliderGeom_;
    std::map<uint64_t, PhysicsBodyId> colliderBodies_;
    // The origin/scale the live colliders were baked with; a change beyond
    // noise means every world-space collider is wrong.
    Vec3 colliderOrigin_{0, 0, 0};
    Real colliderScale_ = 0;
    double timeSeconds_ = 0;

    XrSurfaceLedger::MeshOps meshOps(FrameContext& ctx);
    void ingest(FrameContext& ctx);
    void placeOnPinch(FrameContext& ctx);
    void rebuildDueColliders(FrameContext& ctx);
};

}  // namespace engine

#endif  // RAYTRACER_ENGINE_XR_SURFACE_SYSTEM_H
