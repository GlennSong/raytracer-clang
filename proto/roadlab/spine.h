#ifndef ROADLAB_SPINE_H
#define ROADLAB_SPINE_H

// THE KEYSTONE. A road's geometry is a reference line parameterised by arc
// length s, plus three independent 1D functions of s: elevation, superelevation
// (bank) and lateral offset. Everything else in roadlab — lane widths, markings,
// texture coordinates, sign placement, where a car is — is a function of (s, t).
// One coordinate frame shared by the renderer, the simulator and the placers is
// what stops them from ever disagreeing.
//
// Plan-view geometry is a chain of line / arc / CLOTHOID primitives. Clothoids
// (curvature linear in s) rather than Beziers because that is what real road
// design uses: it gives constant steering rate through a transition, it makes
// superelevation runoff derivable, and it keeps curvature continuous so an AI
// driver never sees a step change in required lateral acceleration.

#include "rl_math.h"
#include <vector>

namespace roadlab {

enum class GeomKind : uint8_t { Line, Arc, Spiral };

struct GeomPrim {
    GeomKind kind = GeomKind::Line;
    double s0 = 0;        // arc length at the primitive's start
    double length = 0;
    Vec2 p0{0, 0};        // plan start point
    double hdg0 = 0;      // plan start heading
    double curv0 = 0;     // curvature at start (1/m, +ve = left turn)
    double curv1 = 0;     // curvature at end (== curv0 for Line/Arc)
};

// A piecewise-cubic scalar function of s. Elevation, superelevation and lateral
// offset are all one of these.
struct Profile1D {
    struct Piece {
        double s0 = 0;
        Poly3 poly;
    };
    std::vector<Piece> pieces;

    void clear() { pieces.clear(); }
    void set(double value) {
        pieces.clear();
        pieces.push_back({0.0, Poly3::constant(value)});
    }
    void add(double s0, const Poly3& p) { pieces.push_back({s0, p}); }
    double eval(double s) const;
    double slope(double s) const;
    bool empty() const { return pieces.empty(); }
};

// An orthonormal frame on the road surface at arc length s, with the bank
// already applied. `left` is +t.
struct Frame {
    double s = 0;
    Vec2 planPos{0, 0};
    Vec3 pos{0, 0, 0};       // reference-line point, elevation applied
    double heading = 0;      // plan heading (XZ, +X toward +Z)
    double curvature = 0;    // 1/m, +ve turns left
    double grade = 0;        // dz/ds
    double roll = 0;         // superelevation, radians (+ve raises the left)
    Vec3 forward{1, 0, 0};
    Vec3 left{0, 0, 1};
    Vec3 up{0, 1, 0};
};

// One sampled point of the reference line, cached for tessellation seeding and
// for the world -> (s,t) inverse.
struct SpineSample {
    double s = 0;
    Vec2 planPos{0, 0};
    double heading = 0;
    double curvature = 0;
};

class Spine {
public:
    void setStart(Vec2 p, double heading);
    void addLine(double length);
    void addArc(double length, double curvature);
    void addSpiral(double length, double curvStart, double curvEnd);

    // Elevation: either flat, a constant grade, or a set of (s, height) knots
    // joined by Hermite cubics with continuous slope (real vertical curves).
    void setFlatElevation(double y);
    void setElevationKnots(const std::vector<Vec2>& sHeight);
    Profile1D& elevation() { return elevation_; }
    Profile1D& superelevation() { return super_; }
    Profile1D& lateralOffset() { return offset_; }
    const Profile1D& elevationConst() const { return elevation_; }

    // Crossfall: the drainage slope from the crown outward (typically 2%). Not a
    // profile — it is a property of the cross-section shape, applied as
    // -|t| * crossfall so both sides shed water.
    void setCrossfall(double slope) { crossfall_ = slope; }
    double crossfall() const { return crossfall_; }

    // Derive banking from the geometry rather than authoring it: e = v^2/(127 R)
    // clamped to maxE, with the runoff laid over the spiral that leads into each
    // curve. This is why clothoids earn their place — the transition length the
    // bank needs is exactly the transition the geometry already has.
    void applySuperelevation(double designSpeedKph, double maxE = 0.06);

    // Chains primitive start points, resolves total length and builds the sample
    // cache. Must be called after the last add*(); adding again re-dirties it.
    void finalize(double maxSampleStep = 2.0);
    bool finalized() const { return finalized_; }

    double length() const { return length_; }
    bool empty() const { return prims_.empty(); }
    const std::vector<GeomPrim>& prims() const { return prims_; }
    const std::vector<SpineSample>& samples() const { return samples_; }

    Frame frameAt(double s) const;
    double curvatureAt(double s) const;

    // (s, t, h) -> world. `t` is lateral (+left) in metres, `h` is height above
    // the road surface. Lateral offset, bank and crossfall are all applied here,
    // so no caller ever reimplements the surface shape.
    Vec3 toWorld(double s, double t, double h = 0.0) const;
    Vec2 toPlan(double s, double t) const;

    // World plan point -> (s, t). Coarse search over the sample cache then a few
    // Newton steps. Returns false when the point projects outside [0, length]
    // by more than `sTolerance`.
    bool toST(Vec2 planPoint, double& s, double& t, double sTolerance = 0.0) const;

    void planBounds(Vec2& lo, Vec2& hi) const;

private:
    std::vector<GeomPrim> prims_;
    Profile1D elevation_, super_, offset_;
    double crossfall_ = 0.02;
    Vec2 start_{0, 0};
    double startHdg_ = 0;
    double length_ = 0;
    bool finalized_ = false;
    std::vector<SpineSample> samples_;

    const GeomPrim* primAt(double s) const;
    void primEnd(const GeomPrim& g, Vec2& p, double& hdg) const;
    void evalPrim(const GeomPrim& g, double ds, Vec2& p, double& hdg, double& curv) const;
};

// --- builders -------------------------------------------------------------

// Chain a polyline into a G2-continuous spine: straights joined by
// spiral-arc-spiral corners of the given radius. This is the entry point for
// both hand-authored roads and procgen, so every road in the system gets real
// transition curves without anyone having to think about it.
Spine spineFromPolyline(const std::vector<Vec2>& points, double cornerRadius,
                        bool useSpirals = true);

// One arc of a circle, from bearing a0 to a1 about `center`. Roundabout rings
// are built from these: a ring is not a primitive, it is a chain of arcs between
// the junctions its arms make.
Spine spineArc(Vec2 center, double radius, double a0, double a1, bool ccw = true);

// A closed ring (roundabouts, loop ramps). `ccw` picks the circulation sense —
// left-hand-traffic worlds flip it.
Spine spineCircle(Vec2 center, double radius, bool ccw = true);

// An arc between two poses (position + heading). Used to build junction
// connectors: it lands on the exit pose, so turn paths are real curves that
// actually meet the outgoing lane.
Spine spineConnector(Vec2 p0, double hdg0, Vec2 p1, double hdg1);

}  // namespace roadlab

#endif
