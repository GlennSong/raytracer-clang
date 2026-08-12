#include "junction.h"

#include "diag.h"

#include <cstdio>

namespace roadlab {

const char* turnName(TurnKind t) {
    switch (t) {
        case TurnKind::Through: return "through";
        case TurnKind::Left: return "left";
        case TurnKind::Right: return "right";
        case TurnKind::UTurn: return "u-turn";
        case TurnKind::Merge: return "merge";
        case TurnKind::Diverge: return "diverge";
    }
    return "?";
}

const char* controlName(JunctionControl c) {
    switch (c) {
        case JunctionControl::Uncontrolled: return "uncontrolled";
        case JunctionControl::Yield: return "yield";
        case JunctionControl::PriorityStop: return "priority-stop";
        case JunctionControl::AllWayStop: return "all-way-stop";
        case JunctionControl::Signalized: return "signalized";
        case JunctionControl::RoundaboutEntry: return "roundabout";
        case JunctionControl::Merge: return "merge";
    }
    return "?";
}

TurnKind classifyTurn(double headingInFrom, double headingOutTo) {
    double d = angleDiff(headingOutTo, headingInFrom);
    double ad = std::fabs(d);
    if (ad > 150.0 * kDeg2Rad) return TurnKind::UTurn;
    if (ad < 35.0 * kDeg2Rad) return TurnKind::Through;
    return d > 0 ? TurnKind::Left : TurnKind::Right;
}

double Junction::cycleLength() const {
    double total = 0;
    for (const SignalPhase& p : phases) total += p.green + p.yellow + p.allRed;
    return total;
}

int Junction::phaseAt(double t, double& remaining) const {
    remaining = 0;
    if (phases.empty()) return -1;
    double cycle = cycleLength();
    if (cycle <= 1e-6) return -1;
    double u = std::fmod(std::fmod(t, cycle) + cycle, cycle);
    for (size_t i = 0; i < phases.size(); ++i) {
        double len = phases[i].green + phases[i].yellow + phases[i].allRed;
        if (u < len) {
            remaining = phases[i].green - u;   // negative once amber starts
            return int(i);
        }
        u -= len;
    }
    return int(phases.size()) - 1;
}

bool Junction::isGreen(int connectionIndex, double t) const {
    if (connectionIndex < 0 || size_t(connectionIndex) >= connections.size()) return true;
    const Connection& c = connections[size_t(connectionIndex)];
    if (c.signalGroup < 0) return true;
    double remaining = 0;
    int phase = phaseAt(t, remaining);
    return phase == c.signalGroup && remaining > 0.0;
}

// --- building -------------------------------------------------------------

namespace {

struct ArmGeom {
    double outward = 0;      // plan heading pointing away from the junction
    double halfWidth = 0;
};

// Pull the arm back along its own arc length until it is `radius` away from the
// junction centre. Walking in s rather than solving radially matters for a
// roundabout, whose ring arms are curved and never get further from the centre.
double stationAtRadius(const Road& r, bool atEnd, Vec2 center, double radius, double limit) {
    double s0 = atEnd ? r.end() : r.begin();
    double step = atEnd ? -0.25 : 0.25;
    double s = s0;
    for (int i = 0; i < 4000; ++i) {
        Vec2 p = r.spine.toPlan(s, 0);
        if (length(p - center) >= radius) break;
        double next = s + step;
        if (std::fabs(next - s0) > limit) {
            // The arm could not be pulled back far enough to clear the junction.
            // This is the clamp that stopped a roundabout ring being eaten whole,
            // so it firing is not automatically wrong — but it does mean the pad
            // and the arm overlap more than the geometry wanted.
            RL_FALLBACK("stationAtRadius hit the trim limit before clearing the junction");
            break;
        }
        if (next < r.begin() || next > r.end()) break;
        s = next;
    }
    return clampd(s, r.begin(), r.end());
}

Vec2 lineIntersect(Vec2 p, Vec2 dp, Vec2 q, Vec2 dq, bool& ok) {
    double den = cross(dp, dq);
    if (std::fabs(den) < 1e-6) {
        ok = false;
        return p;
    }
    double u = cross(q - p, dq) / den;
    ok = true;
    return p + dp * u;
}

}  // namespace

namespace {

bool polygonSelfIntersects(const std::vector<Vec2>& poly) {
    const size_t n = poly.size();
    if (n < 4) return false;
    auto properlyCrosses = [](Vec2 p, Vec2 p2, Vec2 q, Vec2 q2) {
        Vec2 r = p2 - p, s = q2 - q;
        double d = cross(r, s);
        if (std::fabs(d) < 1e-12) return false;   // parallel; touching is not crossing
        double t = cross(q - p, s) / d;
        double u = cross(q - p, r) / d;
        return t > 1e-9 && t < 1 - 1e-9 && u > 1e-9 && u < 1 - 1e-9;
    };
    for (size_t i = 0; i < n; ++i) {
        for (size_t k = i + 2; k < n; ++k) {
            if (i == 0 && k == n - 1) continue;   // adjacent across the wrap
            if (properlyCrosses(poly[i], poly[(i + 1) % n], poly[k], poly[(k + 1) % n]))
                return true;
        }
    }
    return false;
}

// Andrew's monotone chain, carrying each kept point's height with it so the pad
// surface still meets its arms at their own elevations.
void convexHullWithHeights(std::vector<Vec2>& poly, std::vector<double>& heights) {
    const size_t n = poly.size();
    if (n < 3 || heights.size() != n) return;
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return poly[a].x != poly[b].x ? poly[a].x < poly[b].x : poly[a].y < poly[b].y;
    });
    std::vector<size_t> hull(2 * n);
    size_t k = 0;
    for (size_t pass = 0; pass < 2; ++pass) {
        size_t start = k;
        for (size_t idx = 0; idx < n; ++idx) {
            size_t i = pass == 0 ? order[idx] : order[n - 1 - idx];
            while (k >= start + 2 &&
                   cross(poly[hull[k - 1]] - poly[hull[k - 2]], poly[i] - poly[hull[k - 2]]) <= 0)
                --k;
            hull[k++] = i;
        }
        --k;   // the last point of each pass is the first of the next
    }
    if (k < 3) return;   // degenerate; leave the original alone
    std::vector<Vec2> outPoly;
    std::vector<double> outH;
    outPoly.reserve(k);
    outH.reserve(k);
    for (size_t i = 0; i < k; ++i) {
        outPoly.push_back(poly[hull[i]]);
        outH.push_back(heights[hull[i]]);
    }
    poly.swap(outPoly);
    heights.swap(outH);
}

// The pad polygon, the conflict table, the right-of-way ranking and the signal
// phases are the same work whether a junction was resolved from scratch or
// adopted from a file, so they live here and both paths call them.
void buildJunctionPad(Network& net, Junction& j, const std::vector<ArmGeom>& geom) {
    // 3. Pad polygon. Arms sorted by outward bearing; between neighbours the
    // corner is a rounded kerb return rather than a chamfer, so the footway
    // wraps the corner without a notch.
    std::vector<size_t> order(j.arms.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) { return geom[a].outward < geom[b].outward; });

    struct ArmCorners {
        Vec2 first, second;   // in CCW order around the junction
        double firstH, secondH;
        Vec2 outward;
    };
    std::vector<ArmCorners> corners(order.size());
    for (size_t oi = 0; oi < order.size(); ++oi) {
        size_t i = order[oi];
        const JunctionArm& a = j.arms[i];
        const Road& r = net.road(a.road);
        Vec2 pLeft = r.spine.toPlan(a.sContact, a.leftExtent);
        Vec2 pRight = r.spine.toPlan(a.sContact, a.rightExtent);
        // The two corners of an arm are at DIFFERENT heights whenever the road is
        // banked or crowned. Taking them from the arm's own surface is what lets
        // the pad meet a superelevated approach without a lip.
        double hLeft = r.surfacePoint(a.sContact, a.leftExtent).y;
        double hRight = r.surfacePoint(a.sContact, a.rightExtent).y;
        Vec2 u = dirOf(geom[i].outward);
        // Going counter-clockwise we meet the corner on the -perpLeft(u) side
        // first. Which of the road's own edges that is depends on which end of
        // the road the junction holds.
        if (a.atEnd) {
            corners[oi] = {pLeft, pRight, hLeft, hRight, u};
        } else {
            corners[oi] = {pRight, pLeft, hRight, hLeft, u};
        }
    }
    for (size_t oi = 0; oi < corners.size(); ++oi) {
        const ArmCorners& cur = corners[oi];
        const ArmCorners& nxt = corners[(oi + 1) % corners.size()];
        j.boundary.push_back(cur.first);
        j.boundaryHeight.push_back(cur.firstH);
        j.boundary.push_back(cur.second);
        j.boundaryHeight.push_back(cur.secondH);
        bool ok = false;
        Vec2 c = lineIntersect(cur.second, cur.outward, nxt.first, nxt.outward, ok);
        // A kerb return only makes sense if the two edges meet somewhere near
        // the junction. A shallow pair of arms can put the intersection hundreds
        // of metres away, which would drag the pad out into a spike.
        double reach = 2.5 * (geom[order[oi]].halfWidth + j.cornerRadius) + 8.0;
        bool nearEnough = ok && length(c - j.center) < reach;
        // ...and it must stick OUT. Where the two arms are the halves of a street
        // running straight through, their outward rays diverge and "where the
        // edges meet" lands behind both of them, inside the pad. The distance
        // test waves that through — the point is close, just on the wrong side —
        // and the Bezier then dives inward across the far edge, leaving a
        // self-intersecting outline that no ear-clipper can trianglulate.
        //
        // Comparing against the junction centre rather than against a winding
        // convention keeps this true whichever way the boundary happens to run.
        Vec2 chord = nxt.first - cur.second;
        double sideOfC = cross(chord, c - cur.second);
        double sideOfCentre = cross(chord, j.center - cur.second);
        bool bulgesOutward = sideOfC * sideOfCentre < 0;
        if (!nearEnough)
            RL_FALLBACK("junction pad kerb return skipped (edges meet too far away)");
        else if (!bulgesOutward)
            RL_FALLBACK("junction pad kerb return skipped (corner would fold into the pad)");
        if (nearEnough && bulgesOutward) {
            // Quadratic Bezier through the edge intersection: a real kerb return,
            // with its height ramping between the two arms it joins.
            for (int k = 1; k < 5; ++k) {
                double u = double(k) / 5.0;
                Vec2 p = cur.second * ((1 - u) * (1 - u)) + c * (2 * u * (1 - u)) +
                         nxt.first * (u * u);
                j.boundary.push_back(p);
                j.boundaryHeight.push_back(lerp(cur.secondH, nxt.firstH, u));
            }
        }
    }

    // The outline has to be a simple polygon. Everything downstream assumes it:
    // ear clipping needs one, mean value coordinates are only defined on one, and
    // the terrain conform tests point-in-polygon against it. Walking the arms in
    // bearing order gives one for any ordinary junction — but not for a
    // roundabout entry, where two arms are 19 m-wide slices of the SAME
    // circulating carriageway taken 5 m apart. Their corner quads overlap almost
    // exactly, and the outline zig-zags between them.
    //
    // Rather than special-case that shape, repair whatever comes out: the convex
    // hull of the same points is always simple, always contains every arm, and
    // for a junction pad — convex in all but pathological cases — is within a
    // metre or two of the intended outline. Generous beats self-crossing.
    if (polygonSelfIntersects(j.boundary)) {
        RL_FALLBACK("junction pad outline self-intersected -> convex hull");
        convexHullWithHeights(j.boundary, j.boundaryHeight);
    }
}

void buildJunctionConflicts(Network& net, Junction& j) {
    // 5. Conflict points. Sample every connector once, then intersect pairwise.
    std::vector<std::vector<Vec2>> paths(j.connections.size());
    std::vector<std::vector<double>> stations(j.connections.size());
    for (size_t i = 0; i < j.connections.size(); ++i) {
        const Road& r = net.road(j.connections[i].connectorRoad);
        double len = r.spineLength();
        int n = std::max(2, int(len / 1.0));
        for (int k = 0; k <= n; ++k) {
            double s = len * double(k) / double(n);
            paths[i].push_back(r.spine.toPlan(s, 0));
            stations[i].push_back(s);
        }
    }
    for (size_t a = 0; a < j.connections.size(); ++a) {
        for (size_t b = a + 1; b < j.connections.size(); ++b) {
            bool sameFrom = j.connections[a].from == j.connections[b].from;
            bool sameTo = j.connections[a].to == j.connections[b].to;
            bool found = false;
            for (size_t i = 0; i + 1 < paths[a].size() && !found; ++i) {
                for (size_t k = 0; k + 1 < paths[b].size() && !found; ++k) {
                    Vec2 p = paths[a][i], pd = paths[a][i + 1] - p;
                    Vec2 q = paths[b][k], qd = paths[b][k + 1] - q;
                    double den = cross(pd, qd);
                    if (std::fabs(den) < 1e-9) continue;
                    double u = cross(q - p, qd) / den;
                    double v = cross(q - p, pd) / den;
                    if (u < 0 || u > 1 || v < 0 || v > 1) continue;
                    ConflictPoint cp;
                    cp.connA = int(a);
                    cp.connB = int(b);
                    cp.sA = lerp(stations[a][i], stations[a][i + 1], u);
                    cp.sB = lerp(stations[b][k], stations[b][k + 1], v);
                    cp.angle = std::fabs(angleDiff(headingOf(pd), headingOf(qd)));
                    cp.kind = sameTo ? ConflictKind::Merging
                                     : (sameFrom ? ConflictKind::Diverging : ConflictKind::Crossing);
                    j.conflicts.push_back(cp);
                    found = true;
                }
            }
            if (!found && sameTo) {
                // Converging without crossing is still a merge: the two paths
                // arrive at the same lane and have to be sequenced.
                ConflictPoint cp;
                cp.connA = int(a);
                cp.connB = int(b);
                cp.sA = net.road(j.connections[a].connectorRoad).spineLength();
                cp.sB = net.road(j.connections[b].connectorRoad).spineLength();
                cp.kind = ConflictKind::Merging;
                j.conflicts.push_back(cp);
            }
        }
    }

}

void resolvePriority(Network& net, Junction& j) {
    // 6. Right of way.
    double maxPriority = 0;
    for (const JunctionArm& a : j.arms) maxPriority = std::max(maxPriority, a.priority);
    for (Connection& c : j.connections) {
        const JunctionArm& from = j.arms[size_t(c.fromArm)];
        c.priority = from.priority;
        switch (j.control) {
            case JunctionControl::RoundaboutEntry:
                // Circulating traffic is absolute. An arm on a ring road is
                // recognised by the road's kind, not by a flag someone had to
                // remember to set.
                c.yields = net.road(from.road).kind != RoadKind::RoundaboutRing;
                if (!c.yields) c.priority = maxPriority + 100.0;
                break;
            case JunctionControl::AllWayStop:
                c.yields = true;
                break;
            case JunctionControl::Yield:
            case JunctionControl::PriorityStop:
                c.yields = from.priority < maxPriority - 1e-6;
                break;
            case JunctionControl::Merge:
                c.yields = net.road(from.road).kind == RoadKind::Ramp;
                break;
            case JunctionControl::Signalized:
            case JunctionControl::Uncontrolled:
                c.yields = false;
                break;
        }
        // Turning across opposing traffic always gives way, whatever the arm's
        // rank says — a permissive left is a yield even on the major road.
        if (c.turn == TurnKind::Left || c.turn == TurnKind::UTurn) c.yields = true;
    }

}

void buildSignalPhases(Junction& j) {
    // 7. Signal phases by colouring the crossing-conflict graph. The phases are
    // DERIVED from the same conflict table the simulator arbitrates with, so a
    // movement can never be green at the same time as one it would hit.
    if (j.control == JunctionControl::Signalized && !j.connections.empty()) {
        const size_t n = j.connections.size();
        std::vector<std::vector<char>> adj(n, std::vector<char>(n, 0));
        for (const ConflictPoint& cp : j.conflicts) {
            if (cp.kind != ConflictKind::Crossing) continue;
            adj[size_t(cp.connA)][size_t(cp.connB)] = 1;
            adj[size_t(cp.connB)][size_t(cp.connA)] = 1;
        }
        // Opposing lefts do not cross each other but must not run with the
        // through movement they oppose; the crossing table already says so.
        std::vector<size_t> byDegree(n);
        for (size_t i = 0; i < n; ++i) byDegree[i] = i;
        std::sort(byDegree.begin(), byDegree.end(), [&](size_t a, size_t b) {
            int da = 0, db = 0;
            for (size_t k = 0; k < n; ++k) {
                da += adj[a][k];
                db += adj[b][k];
            }
            return da > db;
        });
        std::vector<int> colour(n, -1);
        int maxColour = -1;
        for (size_t idx : byDegree) {
            std::vector<char> taken(n + 1, 0);
            for (size_t k = 0; k < n; ++k)
                if (adj[idx][k] && colour[k] >= 0) taken[size_t(colour[k])] = 1;
            int c = 0;
            while (c < int(n) && taken[size_t(c)]) ++c;
            colour[idx] = c;
            maxColour = std::max(maxColour, c);
        }
        j.phases.resize(size_t(maxColour + 1));
        for (size_t i = 0; i < n; ++i) {
            j.connections[i].signalGroup = colour[i];
            j.phases[size_t(colour[i])].connections.push_back(int(i));
        }
        // Green time proportional to how much traffic the phase serves, with a
        // floor that keeps a protected-left phase from being unusably short.
        size_t busiest = 1;
        for (const SignalPhase& p : j.phases) busiest = std::max(busiest, p.connections.size());
        for (SignalPhase& p : j.phases) {
            p.green = clampd(8.0 + 22.0 * double(p.connections.size()) / double(busiest), 8.0, 30.0);
            p.yellow = 3.5;
            p.allRed = 1.5;
        }
    }
}

}  // namespace

void buildJunction(Network& net, Junction& j) {
    if (j.arms.empty()) return;
    j.connections.clear();
    j.conflicts.clear();
    j.phases.clear();
    j.boundary.clear();
    j.boundaryHeight.clear();

    // 0. Undo any previous trim so the resolve is idempotent — the parametric
    // source was never destroyed, only windowed.
    for (JunctionArm& a : j.arms) {
        if (a.trim <= 0) continue;
        Road& r = net.road(a.road);
        if (a.atEnd) {
            r.sEnd = std::min(r.spineLength(), r.end() + a.trim);
        } else {
            r.sBegin = std::max(0.0, r.begin() - a.trim);
        }
        a.trim = 0;
    }

    // 1. Measure the arms at their untrimmed ends.
    std::vector<ArmGeom> geom(j.arms.size());
    Vec2 centreAcc{0, 0};
    for (size_t i = 0; i < j.arms.size(); ++i) {
        JunctionArm& a = j.arms[i];
        const Road& r = net.road(a.road);
        double s = a.atEnd ? r.end() : r.begin();
        Frame f = r.spine.frameAt(s);
        a.headingIn = a.atEnd ? f.heading : wrapPi(f.heading + kPi);
        a.contact = f.planPos;
        a.leftExtent = r.xs.leftExtentAt(s);
        a.rightExtent = r.xs.rightExtentAt(s);
        geom[i].outward = wrapPi(a.headingIn + kPi);
        geom[i].halfWidth = std::max(std::fabs(a.leftExtent), std::fabs(a.rightExtent));
        a.priority = r.designSpeed * std::max(1, r.xs.sectionAt(s).laneCount());
        centreAcc = centreAcc + f.planPos;
        a.sContact = s;
    }
    j.center = centreAcc * (1.0 / double(j.arms.size()));
    double elevAcc = 0;
    for (const JunctionArm& a : j.arms) elevAcc += net.road(a.road).surfacePoint(a.sContact, 0).y;
    j.elevation = elevAcc / double(j.arms.size());

    // A roundabout entry is not an open intersection and must not be trimmed like
    // one. The circulating carriageway runs THROUGH, and an approach meets it at
    // its outer edge — so the setback an approach needs is the ring's half-width,
    // not a multiple of it. Applying the general formula pulled entries 45 m back
    // from a ring they were supposed to touch, which left the pad reaching across
    // the gap with a self-intersecting outline.
    bool hasRing = false;
    double ringHalfWidth = 0;
    for (size_t i = 0; i < j.arms.size(); ++i) {
        if (net.road(j.arms[i].road).kind != RoadKind::RoundaboutRing) continue;
        hasRing = true;
        ringHalfWidth = std::max(ringHalfWidth, geom[i].halfWidth);
    }

    // 2. Trim. Each arm must be pulled back far enough that its edges clear
    // every other arm's edges; the controlling case is the most acute pair.
    for (size_t i = 0; i < j.arms.size(); ++i) {
        double need = geom[i].halfWidth + 1.0;
        double widest = geom[i].halfWidth;
        for (size_t k = 0; k < j.arms.size(); ++k) {
            if (k == i) continue;
            widest = std::max(widest, geom[k].halfWidth);
            double d = std::fabs(angleDiff(geom[k].outward, geom[i].outward));
            // An arm pointing roughly the opposite way is the far side of a
            // straight-through pair: its edges never cut across this arm, and
            // running the acute-angle formula on it demands an absurd setback
            // (sin -> 0) that eats hundreds of metres of road.
            if (d > 120.0 * kDeg2Rad) continue;
            if (std::sin(d) < 0.20) RL_FALLBACK("junction trim angle clamped (arms nearly parallel)");
            double sn = std::max(0.20, std::sin(d));
            need = std::max(need, geom[k].halfWidth / sn);
        }
        if (need > 3.0 * widest + 6.0) RL_FALLBACK("junction trim setback capped");
        need = std::min(need, 3.0 * widest + 6.0);
        if (hasRing && net.road(j.arms[i].road).kind != RoadKind::RoundaboutRing)
            need = std::min(need, ringHalfWidth + 1.0);
        need += j.cornerRadius;
        const Road& r = net.road(j.arms[i].road);
        double limit = std::max(2.0, r.activeLength() * 0.45);
        // A circulating carriageway is not trimmed back to make room for the arm
        // that joins it — the ring runs through, and the approach yields into it.
        // Trimming it like an ordinary arm eats a short ring arc entirely.
        if (r.kind == RoadKind::RoundaboutRing) limit = std::min(limit, 2.5);
        double s = stationAtRadius(r, j.arms[i].atEnd, j.center, need, limit);
        JunctionArm& a = j.arms[i];
        Road& rw = net.road(a.road);
        if (a.atEnd) {
            a.trim = std::max(0.0, rw.end() - s);
            rw.sEnd = s;
        } else {
            a.trim = std::max(0.0, s - rw.begin());
            rw.sBegin = s;
        }
        a.sContact = s;
        Frame f = rw.spine.frameAt(s);
        a.contact = f.planPos;
        a.headingIn = a.atEnd ? f.heading : wrapPi(f.heading + kPi);
        a.leftExtent = rw.xs.leftExtentAt(s);
        a.rightExtent = rw.xs.rightExtentAt(s);
        geom[i].outward = wrapPi(a.headingIn + kPi);
        geom[i].halfWidth = std::max(std::fabs(a.leftExtent), std::fabs(a.rightExtent));
    }

    buildJunctionPad(net, j, geom);

    // 4. Connectors, one per turning movement.
    struct PendingConn {
        Connection c;
        Spine spine;
        double width;
        double speed;
    };
    std::vector<PendingConn> pending;

    for (size_t ai = 0; ai < j.arms.size(); ++ai) {
        const JunctionArm& A = j.arms[ai];
        const Road& RA = net.road(A.road);
        EndLanes ea = endLanes(RA, A.atEnd);
        if (ea.incoming.empty()) continue;
        int secA = RA.xs.sectionIndexAt(A.atEnd ? std::max(RA.begin(), A.sContact - 1e-3)
                                                : A.sContact + 1e-3);

        for (size_t bi = 0; bi < j.arms.size(); ++bi) {
            if (bi == ai) continue;
            const JunctionArm& B = j.arms[bi];
            const Road& RB = net.road(B.road);
            EndLanes eb = endLanes(RB, B.atEnd);
            if (eb.outgoing.empty()) continue;
            int secB = RB.xs.sectionIndexAt(B.atEnd ? std::max(RB.begin(), B.sContact - 1e-3)
                                                    : B.sContact + 1e-3);
            double headingOut = wrapPi(B.headingIn + kPi);
            TurnKind turn = classifyTurn(A.headingIn, headingOut);
            if (turn == TurnKind::UTurn && j.arms.size() > 2) continue;

            // Lane assignment. Through movements zip from the kerb outward;
            // turns take the lane a driver would actually use, preferring a turn
            // bay when the cross-section provides one.
            std::vector<std::pair<int, int>> pairs;
            const std::vector<int>& in = ea.incoming;
            const std::vector<int>& out = eb.outgoing;
            auto stripKindOf = [&](const Road& r, int sec, int lane) {
                const Strip* st = r.xs.sections[size_t(std::max(0, sec))].strip(lane);
                return st ? st->kind : StripKind::Travel;
            };
            if (turn == TurnKind::Through) {
                size_t n = std::min(in.size(), out.size());
                for (size_t k = 0; k < n; ++k) {
                    if (stripKindOf(RA, secA, in[k]) == StripKind::Turn) continue;
                    pairs.push_back({in[k], out[k]});
                }
            } else if (turn == TurnKind::Right) {
                pairs.push_back({in.front(), out.front()});
            } else {
                int fromLane = in.back();
                for (int cand : in) {
                    if (stripKindOf(RA, secA, cand) == StripKind::Turn) {
                        fromLane = cand;
                        break;
                    }
                }
                pairs.push_back({fromLane, out.back()});
            }

            for (const auto& pr : pairs) {
                Vec2 p0, p1;
                double h0 = 0, h1 = 0;
                if (!RA.lanePose(pr.first, A.sContact, p0, h0)) continue;
                if (!RB.lanePose(pr.second, B.sContact, p1, h1)) continue;
                // lanePose reports the lane's own travel heading; at the exit arm
                // that is already the outbound direction.
                Spine sp = spineConnector(p0, h0, p1, h1);
                if (sp.length() < 0.5) continue;
                double y0 = RA.surfacePoint(A.sContact, RA.laneCenterT(pr.first, A.sContact)).y;
                double y1 = RB.surfacePoint(B.sContact, RB.laneCenterT(pr.second, B.sContact)).y;
                sp.setElevationKnots({{0.0, y0}, {sp.length(), y1}});
                sp.finalize();
                double w = std::max(2.8, RA.xs.laneWidthAt(secA, pr.first, A.sContact));

                PendingConn pc;
                pc.c.fromArm = int(ai);
                pc.c.toArm = int(bi);
                pc.c.from = {A.road, secA, pr.first};
                pc.c.to = {B.road, secB, pr.second};
                pc.c.turn = turn;
                pc.spine = sp;
                pc.width = w;
                // Turning speed is bounded by the path, not by the road it came
                // from: 0.28 g lateral on the tightest radius.
                double kmax = 0;
                for (const GeomPrim& g : sp.prims())
                    kmax = std::max(kmax, std::max(std::fabs(g.curv0), std::fabs(g.curv1)));
                double vmax = kmax > 1e-6 ? designSpeedForRadius(1.0 / kmax) : 1e9;
                pc.speed = std::min({RA.designSpeed, RB.designSpeed, vmax});
                pc.speed = std::max(12.0, pc.speed);
                pending.push_back(std::move(pc));
            }
        }
    }

    for (PendingConn& pc : pending) {
        Road conn;
        char buf[96];
        std::snprintf(buf, sizeof buf, "j%d.%s.%d-%d", j.id, turnName(pc.c.turn), pc.c.fromArm,
                      pc.c.toArm);
        conn.name = buf;
        conn.kind = RoadKind::Connector;
        conn.junctionId = j.id;
        conn.designSpeed = pc.speed;
        conn.allowsPedestrians = false;
        conn.spine = pc.spine;
        LaneSection sec;
        Strip lane;
        lane.kind = StripKind::Travel;
        lane.surface = SurfaceKind::Asphalt;
        lane.width = Poly3::constant(pc.width);
        lane.dir = +1;
        lane.outerMark = markNone();
        sec.right.push_back(lane);
        sec.centerMark = markNone();
        sec.assignIds();
        conn.xs.sections.push_back(sec);
        // Offsetting the stack by half a lane centres the single lane on the
        // path the connector was built along.
        conn.xs.laneOffset.set(pc.width * 0.5);
        pc.c.connectorRoad = net.addRoad(std::move(conn));
        j.connections.push_back(pc.c);
    }

    buildJunctionConflicts(net, j);
    resolvePriority(net, j);
    buildSignalPhases(j);
}

void adoptJunction(Network& net, Junction& j) {
    if (j.arms.empty()) return;
    j.conflicts.clear();
    j.phases.clear();
    j.boundary.clear();
    j.boundaryHeight.clear();

    // Measure the arms where they already are. The file trimmed them; trimming
    // again would eat road that the connectors were built to reach.
    std::vector<ArmGeom> geom(j.arms.size());
    Vec2 centreAcc{0, 0};
    for (size_t i = 0; i < j.arms.size(); ++i) {
        JunctionArm& a = j.arms[i];
        const Road& r = net.road(a.road);
        double s = a.atEnd ? r.end() : r.begin();
        Frame f = r.spine.frameAt(s);
        a.sContact = s;
        a.trim = 0;
        a.contact = f.planPos;
        a.headingIn = a.atEnd ? f.heading : wrapPi(f.heading + kPi);
        a.leftExtent = r.xs.leftExtentAt(s);
        a.rightExtent = r.xs.rightExtentAt(s);
        a.priority = r.designSpeed * std::max(1, r.xs.sectionAt(s).laneCount());
        geom[i].outward = wrapPi(a.headingIn + kPi);
        geom[i].halfWidth = std::max(std::fabs(a.leftExtent), std::fabs(a.rightExtent));
        centreAcc = centreAcc + f.planPos;
    }
    j.center = centreAcc * (1.0 / double(j.arms.size()));
    double elevAcc = 0;
    for (const JunctionArm& a : j.arms) elevAcc += net.road(a.road).surfacePoint(a.sContact, 0).y;
    j.elevation = elevAcc / double(j.arms.size());

    // The turn a movement makes is geometry, not something the file states, so
    // it is derived here — and it has to be, because right of way keys off it.
    for (Connection& c : j.connections) {
        if (c.fromArm < 0 || c.toArm < 0) continue;
        double headingOut = wrapPi(j.arms[size_t(c.toArm)].headingIn + kPi);
        c.turn = classifyTurn(j.arms[size_t(c.fromArm)].headingIn, headingOut);
    }

    buildJunctionPad(net, j, geom);
    buildJunctionConflicts(net, j);
    resolvePriority(net, j);
    buildSignalPhases(j);
}

std::vector<Crosswalk> junctionCrosswalks(const Network& net, const Junction& j) {
    std::vector<Crosswalk> out;
    if (!j.generateCrosswalks) return out;
    for (const JunctionArm& a : j.arms) {
        if (!a.crosswalk) continue;
        const Road& r = net.road(a.road);
        if (!r.allowsPedestrians) continue;
        // A grade-separated road has no street crossing, whatever the flag says.
        if (r.designSpeed >= 90) continue;
        Crosswalk cw;
        cw.road = a.road;
        cw.depth = 3.0;
        // Just outside the pad, on the road side of the trim.
        cw.s = a.atEnd ? a.sContact - cw.depth * 0.5 - 0.5 : a.sContact + cw.depth * 0.5 + 0.5;
        if (cw.s < r.begin() || cw.s > r.end()) continue;
        cw.tMin = r.xs.rightExtentAt(cw.s);
        cw.tMax = r.xs.leftExtentAt(cw.s);
        // Trim to the carriageway: the zebra stops at the kerb.
        const LaneSection& sec = r.xs.sectionAt(cw.s);
        cw.tMax = std::min(cw.tMax, sec.carriagewayHalfWidthAt(cw.s, +1) + r.xs.laneOffset.eval(cw.s));
        cw.tMin = std::max(cw.tMin, -sec.carriagewayHalfWidthAt(cw.s, -1) + r.xs.laneOffset.eval(cw.s));
        cw.signalised = j.control == JunctionControl::Signalized;
        if (cw.tMax - cw.tMin > 2.0) out.push_back(cw);
    }
    return out;
}

}  // namespace roadlab
