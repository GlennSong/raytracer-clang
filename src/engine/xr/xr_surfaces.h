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

}  // namespace engine

#endif  // ENGINE_XR_SURFACES_H
