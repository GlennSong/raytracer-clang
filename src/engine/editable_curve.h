#ifndef RAYTRACER_ENGINE_EDITABLE_CURVE_H
#define RAYTRACER_ENGINE_EDITABLE_CURVE_H

#include "../rt_math.h"
#include <vector>

namespace engine {

// How a knot's two handles relate while editing (ADR-0050):
//   Auto    — handles derived from the neighbours (Catmull-Rom); not stored.
//   Aligned — in/out kept collinear (a smooth pass-through); dragging one rotates
//             the other to stay opposite (its length preserved).
//   Broken  — in/out independent (a corner / sharp change of direction).
enum class KnotMode { Auto, Aligned, Broken };

// One control point of an EditableCurve. The handles are cubic-Bezier control
// points stored as OFFSETS from the knot position, so moving the knot carries them
// along. `inHandle` points back toward the previous knot, `outHandle` toward the next.
struct CurveKnot {
    Vec3 position;
    Vec3 inHandle{0, 0, 0};
    Vec3 outHandle{0, 0, 0};
    KnotMode mode = KnotMode::Auto;
};

// A 3D cubic-Bezier path: the shared, editable spline behind the road editor and
// (next) animation paths (ADR-0050). Knots carry handle POINTS; an Auto knot derives
// its handles from its neighbours on the fly, so evaluation is always correct without
// a separate bake. Distinct from the road's RoadEntity (a branching GRAPH); this is one
// continuous path, the unit an animation follows and the tool manipulates.
struct EditableCurve {
    std::vector<CurveKnot> knots;
    bool closed = false;

    int  segmentCount() const;
    // Absolute Bezier control points of segment `s` (from knot s to s+1), using each
    // knot's effective handles (Auto knots resolved from their neighbours here).
    void segmentControls(int s, Vec3& p0, Vec3& p1, Vec3& p2, Vec3& p3) const;

    // Evaluation. `t` runs [0,1] across the whole path (uniform in segment index).
    Vec3 position(double t) const;
    Vec3 tangent(double t) const;                 // unnormalised derivative

    // Arc length, for constant-speed traversal (animation). `perSeg` sets the
    // sampling resolution of the length table.
    double length(int perSeg = 24) const;
    Vec3   atDistance(double dist, int perSeg = 24) const;   // dist in [0, length]

    std::vector<Vec3> sample(int perSegment) const;          // polyline for drawing

    // The effective (possibly auto-derived) handle world positions of knot `i` — what
    // the editor draws + drags. `outHandleWorld`/`inHandleWorld` = position + handle.
    Vec3 inHandleWorld(int i) const;
    Vec3 outHandleWorld(int i) const;

    // --- edits ---------------------------------------------------------------
    void moveKnot(int i, const Vec3& position);   // handles ride along (they're offsets)
    void setOutHandle(int i, const Vec3& worldPos);   // mode Aligned mirrors the in-handle
    void setInHandle(int i, const Vec3& worldPos);    // mode Aligned mirrors the out-handle
    void setMode(int i, KnotMode mode);           // switching off Auto bakes current handles
    int  addKnot(const Vec3& position);           // append; returns index
    bool removeKnot(int i);
};

}  // namespace engine

#endif
