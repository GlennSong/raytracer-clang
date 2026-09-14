#ifndef ENGINE_PROCGEN_CITY_SITE_PLAN_H
#define ENGINE_PROCGEN_CITY_SITE_PLAN_H

// SITE PLANS (skyscrapers v2, ADR-0086): buildings are rectilinear; lots are
// not. The parceller cuts a block into frontage strips clipped by the corner
// bisectors, so a lot beside a non-orthogonal street corner is a trapezoid by
// construction — and the old massing extruded that trapezoid verbatim, inset
// by a metre. Real cities do the other thing: the building is a rectangle (or a
// union of rectangles) squared to the street it fronts, and the difference
// between the lot and the building is paving, planting, a plaza or a yard.
//
// This module is that blueprint layer. It is pure geometry — a function of the
// lot polygon, its frontage direction and the district's yards — so the loader,
// the bundle producers, the SVG map and the tests all derive the same site.
//
//   SiteFrame           an orthogonal frame on the lot's frontage edge
//   largestAlignedRect  the biggest frame-aligned rectangle the lot holds with
//                       its yards honoured — the BUILDABLE
//   openSpacePieces     the lot minus the building, classified by where each
//                       piece lies (forecourt / side yard / rear yard)

#include "polygon.h"
#include <functional>
#include <string>
#include <vector>

namespace engine {

// An orthogonal frame on the lot: `u` runs along the street (the frontage
// edge, walked in the lot's CCW order), `v` runs inward, away from the street.
// Frame coordinates are (along, depth) in metres from `origin`, the frontage
// edge's first vertex.
struct SiteFrame {
    Vec2 origin{0, 0};
    Vec2 u{1, 0};
    Vec2 v{0, 1};
    Vec2 toFrame(const Vec2& w) const {
        const Vec2 d = w - origin;
        return {dot(d, u), dot(d, v)};
    }
    Vec2 toWorld(const Vec2& f) const { return origin + u * f.x + v * f.y; }
};

// Setbacks from the lot lines, by which side of the lot an edge is: the
// street side (front), the party-wall sides, and the back. Zero is a zero-lot-
// line building (a downtown street wall); a suburban house has all three.
struct Yards {
    Real front = 0;
    Real side = 0;
    Real rear = 0;
};

enum class LotSide { Front, Side, Rear };

// Which side of the lot an edge is on, from its OUTWARD normal against the lot's
// frontage direction: within ~45° of the frontage is the front, within ~45° of
// the opposite is the rear, anything else is a side.
LotSide lotSideOf(const Vec2& outwardNormal, const Vec2& frontage);

// The frame on the lot's frontage edge: among the edges whose outward normal
// faces the street (dot with `frontage` >= 0.7), the longest; failing any, the
// edge whose normal agrees best. The polygon is taken CCW (a CW copy is
// reversed first), so the inward direction is the edge's left normal.
SiteFrame siteFrame(const Poly2& lot, const Vec2& frontage);

// The largest frame-aligned rectangle inside `lot` whose every point is at
// least the side's yard away from every lot edge, found on a `cell`-metre
// raster of the lot (cell centres tested; the rectangle spans the usable
// centres, so it sits up to half a cell inside the exact answer). Returns the
// four corners CCW in world XZ, or empty when no rectangle with both sides
// >= `minSide` fits. O(cells × edges); a 60 m lot at 0.5 m is 14 k cells.
Poly2 largestAlignedRect(const Poly2& lot, const SiteFrame& frame, const Yards& yards,
                         Real cell = 0.5, Real minSide = 6.0);

// What the ground around the building is, by where it lies relative to the
// frontage. The kinds a district dresses differently: an urban forecourt is
// paving continuous with the sidewalk, a residential one is a front lawn.
enum class OpenKind { Forecourt, SideYard, RearYard, Courtyard, Plaza, Lawn, Paving, Parking };
const char* openKindName(OpenKind k);

struct OpenSpace {
    Poly2 poly;   // world XZ, CCW
    OpenKind kind = OpenKind::Paving;
};

// The lot minus a frame-aligned rectangle `mass`, cut by the rectangle's four
// edge lines: the strip in front of it, the strip behind it, and the two side
// pieces beside it. Pieces under `minArea` m² are dropped. Exact for convex
// lots; a mildly concave lot may yield a piece that overlaps the rectangle's
// line extension, which is why the cut is by half-planes rather than a general
// boolean difference (polygon.h has no difference; Clipper stays in lanelab).
std::vector<OpenSpace> openSpacePieces(const Poly2& lot, const Poly2& mass, const SiteFrame& frame,
                                       Real minArea = 1.0);

// One site: the frame, the buildable rectangle and the open ground around it.
// `buildable` is empty when the lot holds no rectangle of `minSide`.
struct SitePlan {
    SiteFrame frame;
    Poly2 buildable;
    std::vector<OpenSpace> open;
};
SitePlan planSite(const Poly2& lot, const Vec2& frontage, const Yards& yards, Real cell = 0.5,
                  Real minSide = 6.0);

}  // namespace engine

#endif
