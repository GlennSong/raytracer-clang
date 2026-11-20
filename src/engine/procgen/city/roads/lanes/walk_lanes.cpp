#include "engine/procgen/city/roads/lanes/walk_lanes.h"

#include "engine/procgen/city/roads/lanes/lane_expand.h"   // ribbonLane: the walks are laid as the lanes are
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/road_model.h"         // plannedRoadModel, ringPairs, towardKerb

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace engine {
namespace roads::lanes {

namespace {

// The kinds of walk the build lays, as the path users' table names them (assets/data/path_users.json).
constexpr const char* kSidewalkKind = "sidewalk";
constexpr const char* kCornerKind = "corner";

// ONE ARM'S WALK AS THE CORNER READS IT: its KERB END on the corner's kerb -- the crosswalk's end there (ModelCrosswalk::
// ends, where a person steps off), or with no crosswalk the carriageway's edge level with the arm's stated box edge (else
// its mouth); a footway's middle there -- and how wide its walk is. False: the arm has no street, or no road the build
// knows.
struct ArmWalk {
    Vec2 anchor;
    double width = 0;
};
bool armWalk(const RoadModel& m, const RoadLabGraph& g, ModelJunctionId j, uint32_t arm, int kerb, ArmWalk& out) {
    const ModelArm& A = m.junctions[j.index].arms[arm];
    const ModelStreet* street = m.streetOf(A);
    const std::map<std::string, int>::const_iterator road = g.index.find(A.road);
    if (!street || road == g.index.end()) return false;
    const EdgeSpec& e = g.edges[static_cast<size_t>(road->second)];
    if (e.xy.size() < 2) return false;
    const ModelSection& s = street->section;
    out.width = s.footway ? s.travelWidth() : s.sidewalk;
    const ModelCrosswalk* c = m.crosswalkAcross(j, arm);
    if (c && !s.footway) {
        out.anchor = c->ends[kerb > 0 ? 0 : 1];
        return true;
    }
    const double station = kerbEndStation(m, j, arm);   // (the one statement: the lane level cuts the walks there too)
    const Vec2 at = pointAt(e.xy, e.s, station);
    out.anchor = s.footway ? at : at + towardKerb(tangentAtStation(e.xy, e.s, station), kerb) * s.carriagewayHalfWidth();
    return true;
}

// WHERE A POINT IS ON A CLOSED RING: its distance from the ring, the nearest point, and how far round the ring that is
// from its first point.
struct OnRing {
    double distance = std::numeric_limits<double>::infinity();
    double around = 0;
    Vec2 at;
    size_t segment = 0;
};
OnRing onRing(const Ring& r, const Vec2& p) {
    OnRing best;
    double around = 0;
    for (size_t i = 0; i < r.size(); ++i) {
        const Vec2 a = r[i], b = r[(i + 1) % r.size()];
        const Vec2 ab = b - a;
        const double l2 = dot(ab, ab);
        const double t = l2 > 0 ? std::clamp(dot(p - a, ab) / l2, 0.0, 1.0) : 0.0;
        const Vec2 q = a + ab * t;
        const double d = distance(p, q);
        if (d < best.distance) {
            best.distance = d;
            best.at = q;
            best.around = around + std::sqrt(l2) * t;
            best.segment = i;
        }
        around += std::sqrt(l2);
    }
    return best;
}
double ringLength(const Ring& r) {
    double L = 0;
    for (size_t i = 0; i < r.size(); ++i) L += distance(r[i], r[(i + 1) % r.size()]);
    return L;
}
// THE SHORTER WAY ROUND RING `r` from `a` to `b`: their own points and the ring's corners between.
std::vector<Vec2> shorterWay(const Ring& r, const OnRing& a, const OnRing& b) {
    const double L = ringLength(r);
    double forward = b.around - a.around;
    if (forward < 0) forward += L;
    const bool goForward = forward <= L - forward;
    std::vector<Vec2> out{a.at};
    const size_t n = r.size();
    if (goForward) {
        // a's segment ends at vertex a.segment + 1; walk vertices up to and including b's segment's start
        if (!(a.segment == b.segment && b.around >= a.around))
            for (size_t k = (a.segment + 1) % n;; k = (k + 1) % n) {
                out.push_back(r[k]);
                if (k == b.segment) break;
            }
    } else if (!(a.segment == b.segment && b.around <= a.around)) {
        for (size_t k = a.segment;; k = (k + n - 1) % n) {
            out.push_back(r[k]);
            if (k == (b.segment + 1) % n) break;
        }
    }
    out.push_back(b.at);
    // (no two points the same: the shrunk edge is cut to the millimetre grid, rt_math.h kSamePoint)
    std::vector<Vec2> clean;
    for (const Vec2& p : out)
        if (clean.empty() || strictlyPast(distance(clean.back(), p), 0.0, kSamePoint)) clean.push_back(p);
    return clean;
}

}  // namespace

const char* walkCornerFaultWord(WalkCornerFault f) {
    switch (f) {
    case WalkCornerFault::None: return "a walk was laid";
    case WalkCornerFault::ArmUnknown: return "an arm's road is not one the build knows";
    case WalkCornerFault::KerbEndOut: return "a kerb end lies out in the paved junction";
    case WalkCornerFault::TwoPieces: return "the drawn sidewalk there is in two pieces";
    }
    return "?";
}

void layWalks(Result& r) {
    const RoadLabGraph& g = r.graph;
    LaneSet& L = r.lanes;
    L.walks.clear();
    r.walkCorners.clear();
    if (!L.curves) return;
    // ---- the sidewalk lanes: each kerb's sidewalk middle, along each road that has one there --------------------------
    for (size_t ei = 0; ei < g.edges.size(); ++ei) {
        const EdgeSpec& e = g.edges[ei];
        const RoadClassSpec& c = g.cls(e);
        if (e.xy.size() < 2 || !(c.sidewalk > 0) || !c.carriesCars() || e.laneCount() == 0) continue;
        std::vector<Vec2> tan, nrm;
        frames(e.xy, tan, nrm);
        for (const int kerb : {+1, -1}) {
            if (!e.sidewalks[kerb > 0 ? 0 : 1]) continue;
            // the builder's frame: a point at spine + nrm * off, nrm = perp(tangent); the kerb lies toward towardKerb
            const double side = dot(towardKerb(tan.front(), kerb), nrm.front()) > 0 ? 1.0 : -1.0;
            std::vector<double> off(e.xy.size(), side * (g.carriagewayHalfWidth(e) + c.sidewalk * 0.5));
            Lane w = ribbonLane(L.curves, e.id + (kerb > 0 ? ".walk+" : ".walk-"), static_cast<int>(ei), kSidewalkKind, e.cls, c.sidewalk,
                                kerb, e.xy, nrm, off);
            w.slot = 0;   // (a walk lies in no row of lanes: its `dir` is the kerb it runs beside)
            w.spineFrom = e.s.front();
            w.spineTo = e.s.back();
            L.walks.push_back(std::move(w));
        }
    }
    // ---- the corners: along the drawn pavement between each ring pair's two kerb ends ------------------------------------
    const RoadModel m = plannedRoadModel(g);
    // the drawn walk: the sidewalk layer and every footway's own paving
    PolySet walkArea = r.pavement.sidewalk;
    for (size_t li = 0; li < L.lanes.size(); ++li)
        if (L.lanes[li].onRoad() && g.cls(g.edges[L.lanes[li].edge]).footway && li < r.pavement.footprints.size())
            walkArea = unionSets(walkArea, r.pavement.footprints[li]);
    std::vector<Box2> areaBoxes;
    for (const Polygon2& p : walkArea) areaBoxes.push_back(bounds(p));
    for (const RingPair& p : ringPairs(m, g)) {
        if (p.gap != CornerGap::None && p.gap != CornerGap::ApartAtSpread) continue;
        WalkCorner wc;
        wc.junction = p.junction;
        wc.armA = p.armA;
        wc.armB = p.armB;
        wc.kerbA = p.kerbA;
        wc.kerbB = p.kerbB;
        wc.spread = p.gap == CornerGap::ApartAtSpread;
        const ModelJunction& J = m.junctions[p.junction.index];
        ArmWalk a, b;
        if (!armWalk(m, g, p.junction, p.armA, p.kerbA, a) || !armWalk(m, g, p.junction, p.armB, p.kerbB, b)) {
            wc.fault = WalkCornerFault::ArmUnknown;
            wc.why = walkCornerFaultWord(wc.fault);
            r.walkCorners.push_back(wc);
            continue;
        }
        wc.anchorA = a.anchor;
        wc.anchorB = b.anchor;
        // the drawn walk round the corner: cut to a window about the two kerb ends, and kept kWalkFromKerb in from its edges
        const double minX = std::min(a.anchor.x, b.anchor.x) - kCornerWindow, maxX = std::max(a.anchor.x, b.anchor.x) + kCornerWindow;
        const double minY = std::min(a.anchor.y, b.anchor.y) - kCornerWindow, maxY = std::max(a.anchor.y, b.anchor.y) + kCornerWindow;
        PolySet near;
        for (size_t k = 0; k < walkArea.size(); ++k) {
            const Box2& bx = areaBoxes[k];
            if (bx.maxX >= minX && bx.minX <= maxX && bx.maxY >= minY && bx.minY <= maxY) near.push_back(walkArea[k]);
        }
        // (its seams closed: two roads' sidewalks drawn up to one another meet with a hairline between, kSeamBridge)
        const PolySet window = closing(intersectSets(near, fromRing(Ring{{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}})), kSeamBridge * 0.5);
        const PolySet sliver = offsetSet(window, -kWalkFromKerb);
        // the one edge of it both kerb ends lie by (each no farther than its own walk is wide, and kWalkFromKerb: a drawn
        // kerb can stand out from the stated carriageway's edge, beside a turn pocket): the nearest to the farther of the two
        const double reachA = a.width + kWalkFromKerb, reachB = b.width + kWalkFromKerb;
        const Ring* best = nullptr;
        OnRing atA, atB;
        double bestWorst = std::numeric_limits<double>::infinity();
        double nearestA = std::numeric_limits<double>::infinity(), nearestB = nearestA;   // each kerb end to any edge
        for (const Polygon2& poly : sliver) {
            std::vector<const Ring*> rings{&poly.outer};
            for (const Ring& h : poly.holes) rings.push_back(&h);
            for (const Ring* ring : rings) {
                if (ring->size() < 3) continue;
                const OnRing oa = onRing(*ring, a.anchor), ob = onRing(*ring, b.anchor);
                nearestA = std::min(nearestA, oa.distance);
                nearestB = std::min(nearestB, ob.distance);
                const double worst = std::max(oa.distance / reachA, ob.distance / reachB);
                if (worst < bestWorst) {
                    bestWorst = worst;
                    best = ring;
                    atA = oa;
                    atB = ob;
                }
            }
        }
        if (!best || bestWorst > 1.0) {
            // WHY, said: a kerb end with no drawn sidewalk by it (it lies out in a paved junction wider than the stated
            // carriageway), or two kerb ends each by a drawn sidewalk, but not the same piece of it
            char buf[200];
            if (nearestA > reachA || nearestB > reachB) {
                wc.fault = WalkCornerFault::KerbEndOut;
                const bool armAFar = nearestA / reachA >= nearestB / reachB;
                std::snprintf(buf, sizeof buf, "arm %u's kerb end lies %.1f m from the nearest drawn sidewalk (out in the paved junction)",
                              armAFar ? p.armA : p.armB, armAFar ? nearestA : nearestB);
            } else {
                wc.fault = WalkCornerFault::TwoPieces;
                std::snprintf(buf, sizeof buf, "the drawn sidewalks by the two kerb ends are not joined (two pieces)");
            }
            wc.why = buf;
            r.walkCorners.push_back(wc);
            continue;
        }
        std::vector<Vec2> line = shorterWay(*best, atA, atB);
        // (the two kerb ends one place: where the crosswalks meet at the kerb's corner, a walk of no length joins them)
        if (line.size() < 2) line = {atA.at, atB.at};
        std::vector<Vec2> tan, nrm;
        frames(line, tan, nrm);
        const std::vector<double> zero(line.size(), 0.0);
        Lane w = ribbonLane(L.curves, J.id + ":" + std::to_string(p.armA) + "-" + std::to_string(p.armB), -1, kCornerKind, std::string(),
                            std::min(a.width, b.width), +1, line, nrm, zero);
        w.slot = 0;
        wc.lane = static_cast<int>(L.walks.size());
        L.walks.push_back(std::move(w));
        r.walkCorners.push_back(wc);
    }
}

}  // namespace roads::lanes
}  // namespace engine
