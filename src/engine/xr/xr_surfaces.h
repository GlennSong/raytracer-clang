// Room-surface mapping: the engine-side model of what ARKit knows about the
// user's real surroundings — detected planes (floor / walls / tables, with
// semantic classifications) and the reconstructed room mesh (everything else:
// furniture, clutter), delivered as anchors that appear, change, and vanish.
//
// Like xr_state.h, NOTHING Apple crosses this boundary: the visionOS backend
// converts anchors into XrSurfaceUpdate (engine math types, plain buffers) and
// pushes them into an XrSurfaceStore; an engine system drains the store once a
// frame and feeds the XrSurfaceLedger, which owns the anchor -> mesh
// bookkeeping. The split exists for the same reason as xr_view_math.h: the
// providers only run on a physical headset (the simulator supports neither),
// so the logic that can go wrong — lifecycle, ordering, bad-event tolerance —
// must live where any host can test it (tests/test_xr_surfaces.cpp), leaving
// the device build a thin adapter.
//
// Spaces and units: surface data is ORIGIN space (the tracking origin), REAL
// METRES — the backend does not world-scale it, unlike originFromHead, because
// geometry needs a uniform scale applied at render time (vertices too, not
// just the transform); xrSurfaceWorldTransform composes that.
//
// Threading: the backend pushes from ARKit's dispatch queue; the engine drains
// on the engine thread. The store is the only object that crosses, and it is a
// mutex'd queue — the same shape as CompositorXrBackend's input queue.

#ifndef ENGINE_XR_SURFACES_H
#define ENGINE_XR_SURFACES_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <vector>

#include "../../rt_math.h"

namespace engine {

// What kind of surface an anchor is. Planes carry the runtime's semantic
// classification; reconstruction chunks are Mesh — geometry with no claim
// about what it is a surface OF.
enum class XrSurfaceClass : uint8_t {
    Unknown = 0,
    Wall,
    Floor,
    Ceiling,
    Table,
    Seat,
    Window,
    Door,
    Mesh,   // scene-reconstruction chunk
};
constexpr int XR_SURFACE_CLASS_COUNT = 9;

const char* xrSurfaceClassName(XrSurfaceClass cls);

// Debug tint per class, chosen to read apart at a glance: floor green,
// ceiling blue, walls neutral, things you can put objects ON (table/seat)
// warm, openings (window/door) cyan/magenta, raw mesh dim grey.
Vec3 xrSurfaceClassColor(XrSurfaceClass cls);

// One anchor event, copied out of the runtime callback (anchor geometry is
// only valid inside it). Removed events carry no geometry.
struct XrSurfaceUpdate {
    enum class Op : uint8_t { Added, Updated, Removed };

    uint64_t anchorId = 0;
    Op op = Op::Added;
    XrSurfaceClass cls = XrSurfaceClass::Unknown;
    Mat4 originFromAnchor;              // rigid, ORIGIN space, real metres
    std::vector<Vec3> positions;        // anchor space
    std::vector<Vec3> normals;          // parallel to positions (may be empty)
    std::vector<uint32_t> indices;      // triangle list
};

// The thread crossing. push() from the runtime's queue, drain() once per
// engine frame. drain() appends and clears, preserving arrival order —
// ordering matters because Updated must land after the Added it follows.
class XrSurfaceStore {
public:
    void push(XrSurfaceUpdate&& update) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(std::move(update));
    }
    void drain(std::vector<XrSurfaceUpdate>& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        out.insert(out.end(), std::make_move_iterator(queue_.begin()),
                   std::make_move_iterator(queue_.end()));
        queue_.clear();
    }

private:
    std::mutex mutex_;
    std::vector<XrSurfaceUpdate> queue_;
};

// A live surface as the ledger tracks it. `meshToken` is an opaque nonzero
// value minted by the owner's create callback (the render system packs a
// MeshHandle into it; tests hand out counters) — the ledger never interprets
// it, which is what keeps this module renderer-free and host-testable.
struct XrSurfaceEntry {
    XrSurfaceClass cls = XrSurfaceClass::Unknown;
    Mat4 originFromAnchor;
    uint64_t meshToken = 0;
    uint32_t triangleCount = 0;
};

// Anchor -> mesh bookkeeping. Consumes drained updates and calls back into its
// owner to create/destroy renderer meshes, so exactly one mesh exists per live
// anchor. Tolerates the runtime's rough edges, which are real, not
// hypothetical: Updated can arrive before Added (treat as Added), Removed can
// arrive for an id never seen or already removed (no-op), and a session
// restart needs clear() to release everything.
class XrSurfaceLedger {
public:
    // create returns a nonzero token for the update's geometry; destroy
    // releases one. destroy is ALWAYS called for a replaced or removed token —
    // the mutation the tests guard hardest, because a leaked mesh per anchor
    // update is invisible until the frame ledger shows creation churn.
    struct MeshOps {
        std::function<uint64_t(const XrSurfaceUpdate&)> create;
        std::function<void(uint64_t token)> destroy;
    };

    void apply(const std::vector<XrSurfaceUpdate>& updates, const MeshOps& ops);
    void clear(const MeshOps& ops);

    const std::map<uint64_t, XrSurfaceEntry>& surfaces() const {
        return surfaces_;
    }

    // Census for the numeric readout: per-class surface counts, total
    // triangles, and the ORIGIN-space height of the lowest Floor-classified
    // plane — the number a future "put the arena on the floor/table" round
    // consumes. floorValid is false until a floor has been seen.
    struct Census {
        int countByClass[XR_SURFACE_CLASS_COUNT] = {};
        int total = 0;
        uint64_t triangles = 0;
        bool floorValid = false;
        Real floorY = 0;
    };
    Census census() const;

private:
    std::map<uint64_t, XrSurfaceEntry> surfaces_;
};

// Where surface geometry lands in the world: base + worldScale * (anchor
// transform applied in real metres). A uniform scale column-scales the whole
// composition — unlike the rigid eye poses (xrScaleOriginTransform), the
// VERTICES must scale too, so this is scale-times-transform, not a scaled
// translation.
inline Mat4 xrSurfaceWorldTransform(const Vec3& originBase, Real worldScale,
                                    const Mat4& originFromAnchor) {
    return Mat4::translate(originBase.x, originBase.y, originBase.z)
         * Mat4::scale(worldScale, worldScale, worldScale)
         * originFromAnchor;
}

// The boundary of a triangle mesh: every edge used by exactly one triangle,
// as index pairs. For a detected plane this traces its outline polygon —
// what the debug view draws — and for damaged input it simply returns more
// edges rather than asserting.
std::vector<std::pair<uint32_t, uint32_t>> xrSurfaceBoundaryEdges(
    const std::vector<uint32_t>& indices);

// Anchor-space bounding box of a surface's vertices. For a detected plane
// (which lies in its anchor's XZ, +Y normal) extent.x by extent.z ARE the
// surface's real-world dimensions in metres — the numbers the debug readout
// prints, since the engine has no 3D text to draw them with.
struct XrSurfaceExtent {
    bool valid = false;
    Vec3 min, max;
    Vec3 size() const { return max - min; }
};
XrSurfaceExtent xrSurfaceExtent(const std::vector<Vec3>& positions);

// Nearest hit of a ray against a triangle list, both spaces the caller's
// (placement transforms a world gaze ray into anchor space and gets t back in
// real metres). Two-sided deliberately: a plane's winding depends on which
// side ARKit meshed it from, and a placement ray must not fall through a
// table because the runtime wound it face-down. Returns false on no hit;
// degenerate triangles are skipped, not fatal.
bool xrRaycastTriangles(const Vec3& origin, const Vec3& dir,
                        const std::vector<Vec3>& positions,
                        const std::vector<uint32_t>& indices, Real& tOut);

// When an anchor's static physics collider gets (re)built. ARKit refines
// anchors in storms — a wall can update several times a second while the
// user looks at it — and cooking a Jolt MeshShape per update would spend the
// frame budget on geometry the renderer already shrugged off. The policy
// throttles rebuilds to one per anchor per `minInterval` seconds, EXCEPT the
// first build of a never-seen anchor, which is immediate: a new surface with
// no collider yet is a hole in the floor, worth a cook right now.
//
// Pure and renderer/physics-free so test_xr_surfaces.cpp can drive it with a
// synthetic clock; the system feeds it real time and does the Jolt calls.
class XrColliderPolicy {
public:
    explicit XrColliderPolicy(Real minInterval = 1.0)
        : minInterval_(minInterval) {}

    // Geometry arrived for this anchor (Added or Updated).
    void noteUpdate(uint64_t anchorId, Real now);
    // The anchor is gone; forget it entirely.
    void noteRemoved(uint64_t anchorId);
    // Everything is stale (the XR origin or world scale moved — every world-
    // space collider is now wrong). Existing anchors keep their last-build
    // times, so the rebuild wave still respects the throttle and staggers.
    void invalidateAll();

    // Anchors due for a (re)build at `now`: dirty, and either never built or
    // past the interval since their last build. Returned anchors are marked
    // built-at-`now` and clean; call once per frame and cook what comes back.
    // maxCount caps the batch (cooks hitch — spread them across frames);
    // anchors past the cap stay dirty and return on later calls.
    std::vector<uint64_t> drainDue(Real now, size_t maxCount = SIZE_MAX);

    // Dirty anchors still waiting on the throttle (the census readout).
    size_t pendingCount() const;

private:
    struct State {
        Real lastBuild = 0;
        bool everBuilt = false;
        bool dirty = false;
    };
    std::map<uint64_t, State> anchors_;
    Real minInterval_;
};

}  // namespace engine

#endif  // ENGINE_XR_SURFACES_H
