#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ALIGNMENT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ALIGNMENT_H

#include "polygon.h"   // Vec2, Real
#include <vector>

namespace engine {

// The CORRIDOR MODEL (plan §8, device: "we should build this all in a very
// real sense because the traffic simulation will depend on it"). A road is
// not a graph edge with a width: it is an ALIGNMENT — a stationed centreline
// (every point addressed by its distance along the line), plus a VERTICAL
// PROFILE designed separately, plus a LANE SCHEDULE. Meshes, markings, ramps,
// piers, and the per-lane nav all read the same stationed description.

// --- Horizontal alignment ---------------------------------------------------
// Built from a control polyline whose corners are filleted the way highways
// are: tangent -> clothoid spiral -> circular arc -> mirror spiral -> tangent.
// The clothoid ramps curvature LINEARLY (steering input is gradual — the
// device's "the curves have to spiral down" tool). Represented sampled: a
// dense polyline with per-sample station, heading and curvature, so callers
// interpolate by station without re-deriving the geometry.
struct AlignmentSample {
    Vec2 pos;
    Vec2 tangent;     // unit
    Real station = 0; // arclength from the start (m)
    Real curvature = 0;   // signed 1/R (left turn positive)
};

class Alignment {
public:
    // Fillet every interior corner of `control` with design radius `radius`
    // and spiral length `spiralLen` (clamped per corner when the deflection
    // is too small for the full spiral pair). `step` is the sampling
    // interval along the curve (m).
    static Alignment fromPolyline(const std::vector<Vec2>& control,
                                  Real radius, Real spiralLen,
                                  Real step = 2.0);

    Real length() const { return samples_.empty() ? 0 : samples_.back().station; }
    bool empty() const { return samples_.size() < 2; }
    const std::vector<AlignmentSample>& samples() const { return samples_; }

    // Interpolated queries by station (clamped to [0, length]).
    Vec2 pos(Real s) const;
    Vec2 tangent(Real s) const;          // unit
    Vec2 normal(Real s) const;           // unit, LEFT of travel
    Real curvature(Real s) const;        // signed, left positive
    // A point offset laterally from the centreline: +offset is LEFT of travel.
    Vec2 offset(Real s, Real off) const { return pos(s) + normal(s) * off; }

private:
    std::vector<AlignmentSample> samples_;
    std::size_t indexFor(Real s) const;  // sample interval containing s
};

// --- Vertical profile --------------------------------------------------------
// Constant grades joined by parabolic crest/sag curves at each PVI (point of
// vertical intersection) — the standard highway profile. Stations are on the
// horizontal alignment; elevations are absolute world Y.
struct VerticalProfile {
    struct Pvi {
        Real station = 0;
        Real elevation = 0;
        Real curveLength = 0;   // parabolic transition centred on the PVI
                                // (0 at the endpoints)
    };
    std::vector<Pvi> pvis;      // strictly increasing station; >= 2 entries

    Real elevation(Real s) const;
    Real grade(Real s) const;   // dz/ds (e.g. 0.04 = 4%)
};

// --- Lane schedule -----------------------------------------------------------
// lanes(station), per direction. Through lanes run the whole corridor; spans
// add AUXILIARY lanes (deceleration/exit or acceleration/merge) on the OUTER
// (right) edge — lane count changes happen only at gores (plan §8).
struct LaneSchedule {
    int throughLanes = 3;               // per direction, continuous end-to-end
    struct AuxSpan {
        Real s0 = 0, s1 = 0;            // station range carrying the extra lane
        bool upStation = true;          // which carriageway carries it
    };
    std::vector<AuxSpan> aux;

    // Lane count for ONE carriageway at station s (through + its aux spans).
    int lanesAt(Real s, bool upStation = true) const;
};


// An EXIT (plan §8 P8.3): the lane schedule grows an auxiliary lane over the
// deceleration length; at the gore the aux lane's edge becomes the ramp,
// which clothoids away and grades down to `target` (a street junction). The
// exit direction is the corridor's up-station carriageway when `upStation`,
// else the opposing one.
struct ExitDef {
    Real station = 0;        // the GORE station (where the nose paints)
    bool upStation = true;   // which carriageway the exit serves
    Vec2 target;             // where the ramp lands (world XZ, on a street)
    Real targetY = 0;        // landing elevation (the street's grade)
    Real decelLength = 220;  // aux-lane length before the gore
    Real rampRadius = 70;    // ramp design radius (low speed)
    Real rampSpiral = 30;    // ramp clothoid length
    // ON-RAMP: the mirror primitive (device: "if there's an exit there
    // should be an onramp ... build these in pairs"). The aux lane becomes an
    // ACCELERATION lane after the merge point and the ramp arrives from
    // `target` instead of leaving for it. Same geometry machinery, reversed.
    bool onRamp = false;
};


// --- The corridor ------------------------------------------------------------
// One directed pair of carriageways around a median, described by station.
struct CorridorDef {
    Alignment horizontal;
    VerticalProfile vertical;
    LaneSchedule lanes;
    Real laneWidth = 3.6;
    Real shoulderOut = 2.4;             // outer (right) shoulder per carriageway
    Real shoulderIn = 0.9;              // inner shoulder against the median
    Real medianWidth = 1.4;             // barrier strip between directions
    Real superelevationMax = 0.07;      // bank cap (7%)
    Real designSpeed = 30.0;            // m/s (~110 km/h): e = v^2/(gR), capped

    // The corridor's exits (P8.3): the sweep grows each one's aux lane into
    // the schedule and emits its ramp as a single-carriageway sweep.
    // (Declared below CorridorDef; stored here as the corridor OWNS them.)
    std::vector<ExitDef> exits;

    // Paved half-width at station s (median centre to outer edge) for one
    // side: -1 = the up-station carriageway (negative offsets), +1 = the
    // down-station one. Aux lanes TAPER in over ~35 m so the deck edge
    // flares smoothly instead of stepping. No-arg overload = widest side.
    Real halfWidthAt(Real s, int sideSign) const;
    Real halfWidthAt(Real s) const;
    // Bank angle cross-slope at s (signed: positive tilts the LEFT edge up,
    // i.e. banking INTO a left curve).
    Real superelevationAt(Real s) const;
};



}  // namespace engine

#endif
