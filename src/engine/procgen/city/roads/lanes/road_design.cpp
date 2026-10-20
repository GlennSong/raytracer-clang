#include "engine/procgen/city/roads/lanes/road_design.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace engine {
namespace roads::lanes {

namespace {
// a planned profile at a station (linear between its samples, held at its ends)
double profileAt(const EdgeSpec& e, double s) {
    const double k = std::max(0.0, s / std::max(e.profileStep, 1e-9));
    const std::size_t i = std::min(static_cast<std::size_t>(k), e.profile.size() - 1);
    if (i + 1 >= e.profile.size()) return e.profile.back();
    const double u = k - static_cast<double>(i);
    return e.profile[i] * (1.0 - u) + e.profile[i + 1] * u;
}

// A DESIGNED GRAPH (every road carries its planned profile): the heights as planned, never re-designed -- the ground
// under each road sampled, each ramp's anchors and free length measured as rampProfile measures them, and the
// junctions and level crossings measured for the checks, not corrected
// one planned height a sample of the spine it was designed on: exactly those; else (a spine resampled differently)
// read along it by station
void planned(EdgeSpec& e) {
    if (e.profile.size() == e.s.size()) { e.z = e.profile; return; }
    e.z.resize(e.s.size());
    for (std::size_t i = 0; i < e.s.size(); ++i) e.z[i] = profileAt(e, e.s[i]);
}

DesignReport applyDesign(RoadLabGraph& g, const HeightField& terrain) {
    DesignReport rep;
    const Rules& R = g.rules;
    for (EdgeSpec& e : g.edges) {
        if (e.isRamp() || e.xy.size() < 2) continue;
        e.s = stations(e.xy); e.t.resize(e.xy.size());
        for (std::size_t i = 0; i < e.xy.size(); ++i) e.t[i] = terrain(e.xy[i].x, e.xy[i].y);
        planned(e);
    }
    for (EdgeSpec& e : g.edges) {
        if (!e.isRamp()) continue;
        rampProfile(g, e, terrain);   // (its anchors, free length and climb, from its hosts' planned heights)
        planned(e);
    }
    const std::vector<CrossingMeet> meets = crossingMeets(g);
    NodeMismatchWhere w;
    const CrossingMeet* xw = nullptr;
    const double nm = nodeMismatch(g, R.endpointTol, R.bridgeH, &w), xm = crossingMismatch(g, meets, R.bridgeH, R.rampLevelDz, &xw);
    rep.nodeMismatchBefore = rep.nodeMismatchAfter = std::max(nm, xm);
    if (nm >= xm && nm > 0.02)
        rep.nodeMismatchWhere = w.a + (w.atEnd ? " meets " : " ends on ") + w.b + " at (" + std::to_string(static_cast<int>(w.at.x)) + ", " + std::to_string(static_cast<int>(w.at.y)) + ")";
    else if (xw && xm > 0.02)   // a level crossing whose planned heights do not meet
        rep.nodeMismatchWhere = g.edges[xw->hi].id + " crosses " + g.edges[xw->lo].id + " at (" + std::to_string(static_cast<int>(xw->p.x)) + ", " + std::to_string(static_cast<int>(xw->p.y)) + ")";
    rep.undeclared = undeclaredCrossings(g, meets);
    return rep;
}
}  // namespace

DesignReport designRoads(RoadLabGraph& g, const HeightField& terrain) {
    if (g.designed()) return applyDesign(g, terrain);
    DesignReport rep;
    const Rules& R = g.rules;
    // Which ends are open: nothing else ends there, and no other road's paved band
    // covers them. A road that meets the world only at its middle has to come down to
    // the ground at both tips.
    for (EdgeSpec& e : g.edges) {
        if (e.xy.size() < 2) continue;
        for (int k = 0; k < 2; ++k) {
            const Vec2 q = k ? e.xy.back() : e.xy.front();
            bool met = false;
            // a FOOTWAY's end its own floor holds stays up: a footbridge ends over the far platform, not on the ground
            // there (any class at all lifted the island's diamond ramps off the ground at their ends: 0.5 % steeper)
            if (g.cls(e).footway) for (const auto& fp : e.floorPts)
                if (std::fabs(project(e.xy, e.s, Vec2(fp[0], fp[1])).station - (k ? e.s.back() : 0.0)) <= fp[3]) met = true;
            for (const EdgeSpec& o : g.edges) {
                if (met) break;
                if (&o == &e || o.xy.size() < 2) continue;
                if (std::min(distance(o.xy.front(), q), distance(o.xy.back(), q)) <= R.endpointTol) { met = true; break; }
                if (project(o.xy, o.s, q).distance <= g.hw(o) + g.hw(e)) { met = true; break; }
            }
            e.openEnd[k] = !met;
        }
    }
    for (EdgeSpec& e : g.edges) if (!e.isRamp()) throughProfile(e, terrain, g.cls(e));
    chainTrackFloors(g, R.endpointTol);   // (needs every edge's stations) a track's holds run on across its joints
    for (EdgeSpec& e : g.edges) if (!e.chainFloors.empty()) throughProfile(e, terrain, g.cls(e));
    for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);   // ramps need heights to take part in crossing consistency
    // where the roads meet, found once (only the heights move from here on)
    const std::vector<CrossingMeet> meets = crossingMeets(g);
    auto agree = [&](bool first) {
        // Iterate to the BEST state, not the last one. Each pass only partially
        // resolves (a correction is smoothed over a radius), so a city whose
        // crossings all have to agree needs many more rounds than a lab scene:
        // metro_hills reads 35 cm at 12 iterations, 19 at 30, 5.5 at 60, 0.5 at 120.
        // But some inputs DIVERGE — a freeway traced off a city's outline with 9 m
        // hairpins cannot be satisfied — and there a longer run simply travels
        // further from the answer: living_city went 410 cm at 12 and 710 at 120.
        // So: keep the profiles from the best round, stop once it stops improving,
        // and restore them. Convergent scenes are unaffected; divergent ones end at
        // their best instead of their worst.
        double best = 1e300;
        int stalls = 0;
        std::vector<std::vector<double>> keep;
        for (int it = 0; it < 120; ++it) {
            double mm = nodeConsistency(g, R.endpointTol, R.bridgeH), xm = crossingConsistency(g, meets, R.bridgeH, R.rampLevelDz);
            chainTrackFloors(g, R.endpointTol);
            for (EdgeSpec& e : g.edges) if (!e.isRamp()) applyFloors(e, g.cls(e));   // lifts survive the blends
            for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);   // hosts moved
            const double cur = std::max(mm, xm);
            if (first && it == 0) rep.nodeMismatchBefore = cur;
            if (cur < best - 1e-4) {
                best = cur; stalls = 0;
                keep.clear(); keep.reserve(g.edges.size());
                for (const EdgeSpec& e : g.edges) keep.push_back(e.z);
            } else if (++stalls >= 4) {
                break;
            }
            if (cur < 0.005) break;
        }
        if (stalls >= 4 && keep.size() == g.edges.size()) {
            for (std::size_t i = 0; i < g.edges.size(); ++i) g.edges[i].z = keep[i];
            for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);
        }
    };
    agree(true);
    // Grade-separated crossings clear their structure (Glenn: "roads that go underneath collide with the
    // bottom of the freeway's undercarriage"): wherever two roads cross with the upper one already a bridge
    // (dz >= bridge_h), the upper deck's TOP must be at least under_clearance + its slab + the girders above
    // the lower road's surface. Short crossings get a floor tent on the upper road and its profile is
    // re-run — through roads via their tents, ramps by lifting under the same design-grade cone.
    {
        std::vector<EdgeSpec*> paved; for (EdgeSpec& e : g.edges) if (e.z.size() > 1 && e.laneCount() > 0) paved.push_back(&e);
        // Crossings including a road END lying on the other road's interior: the ring's freeway arcs meet at
        // a node straight above the arterial, and the exact-bounds segment test finds the arc that starts
        // there but misses the one that ends a hair short of the line, leaving that side of the node low.
        auto crossingsInclusive = [&](const EdgeSpec& a, const EdgeSpec& b) {
            std::vector<Vec2> out = crossings(a.xy, b.xy);
            auto addEnd = [&](const EdgeSpec& e, const EdgeSpec& o) {
                // a street ending on a street is a junction (levelled by crossingConsistency), never an
                // overpass to lift -- ends count here only where a freeway or a ramp is one of the two
                if (!e.isRamp() && !o.isRamp() && e.cls != "freeway" && o.cls != "freeway") return;
                for (const Vec2& q : {e.xy.front(), e.xy.back()}) {
                    // a road ending anywhere under the other's paved band counts (streets the importer clipped at a
                    // frontage road's edge end 5 m from its centreline); at-grade tees are excluded by the bridge test
                    const Projection pr = project(o.xy, o.s, q); if (pr.distance > g.hw(o) + g.hw(e)) continue;
                    if (std::min(distance(o.xy.front(), q), distance(o.xy.back(), q)) <= R.endpointTol) continue;   // a shared node: a junction
                    bool dup = false; for (const Vec2& c : out) if (distance(c, q) < 1.0) dup = true; if (!dup) out.push_back(q);
                }
            };
            addEnd(a, b); addEnd(b, a); return out;
        };
        int lifted = 0;
        for (int pass = 0; pass < 3; ++pass) {
            bool any = false;
            for (size_t i = 0; i < paved.size(); ++i) for (size_t j = i + 1; j < paved.size(); ++j) {
                EdgeSpec& a = *paved[i]; EdgeSpec& b = *paved[j];
                if (a.isRamp() && (a.from.edge == b.id || a.to.edge == b.id)) continue;   // a ramp and its own host: the gore, not a crossing
                if (b.isRamp() && (b.from.edge == a.id || b.to.edge == a.id)) continue;
                const std::vector<Vec2> meetsAB = crossingsInclusive(a, b);
                const std::size_t crossedAB = crossings(a.xy, b.xy).size();   // crossingsInclusive lists the true crossings first
                for (std::size_t mi = 0; mi < meetsAB.size(); ++mi) {
                    const Vec2& p = meetsAB[mi];
                    // a SHARED END is a node, never an overpass: two pieces of one line joined end to start "cross" there
                    // (the island loop's sections lifted each other 7 m at every joint, and rode on viaducts between)
                    if (std::min(distance(a.xy.front(), p), distance(a.xy.back(), p)) < R.endpointTol &&
                        std::min(distance(b.xy.front(), p), distance(b.xy.back(), p)) < R.endpointTol) continue;
                    // AS PLANNED (ADR-0215): a declared level meeting is never lifted, a declared bridge always clears, its
                    // named road over; with no plan, the kind is read as it always was
                    const CrossingSpec* planned = g.declaresCrossings ? g.declaredCrossing(a.id, b.id, p) : nullptr;
                    if (planned && planned->kind == "at_grade") continue;
                    const bool sep = planned ? true : (mi < crossedAB || anyTrack(g, a, b)) && separates(g, a, b);   // a freeway (or a track) crossing is structure at any height; a track's tee too
                    const double za = projectZ(a, p), zb = projectZ(b, p); if (!sep && std::fabs(za - zb) < ((a.isRamp() || b.isRamp()) ? R.rampLevelDz : R.bridgeH)) continue;   // below: a level crossing the street meets
                    // which goes over: the higher; for a forced separation, the one the scene asked to (a floor
                    // point over the crossing), else the freeway
                    bool aHi = za > zb;
                    if (planned) aHi = planned->over == a.id;
                    else if (sep) {
                        auto floored = [&](const EdgeSpec& e) { for (const auto& fp : e.floorPts) if (std::hypot(fp[0] - p.x, fp[1] - p.y) < 40.0) return true; return false; };
                        const bool fa = floored(a), fb = floored(b);
                        const bool ta = g.cls(a).track, tb = g.cls(b).track;
                        if (fa != fb) aHi = fa;
                        else if (ta != tb) {
                            // A TRACK MOVES, NOT THE ROAD: it rises over the road at its own gentle grade, unless the road
                            // already clears it (a link on its way up over the freeway). Lifting the higher of the two lifted
                            // Saltwood's streets over the island loop at 18 % and the links under the diamonds' ramps.
                            const EdgeSpec& road = ta ? b : a;
                            const double zRoad = ta ? zb : za, zTrack = ta ? za : zb;
                            const bool roadClears = zRoad - zTrack >= R.underClearance + g.cls(road).thick + R.structureDepth;
                            aHi = roadClears ? !ta : ta;
                        }
                        else if (std::fabs(za - zb) < R.bridgeH && (a.cls == "freeway") != (b.cls == "freeway")) aHi = a.cls == "freeway";
                    }
                    EdgeSpec& hi = aHi ? a : b; EdgeSpec& lo = aHi ? b : a;
                    // The hold must span the whole crossing, not the centreline point: a tent peaks at its station
                    // and falls away at design grade, so the deck over the far kerb of a 20 m road sat half a
                    // metre low. The floor point holds flat over a half-span covering the lower road's paved band
                    // plus the upper road's own width, measured along the upper road (over the crossing sine).
                    auto pavedW = [&](const EdgeSpec& e) { const RoadClassSpec& c = g.cls(e); return e.laneCount() * e.lanes.w + e.lanes.gap + 2.0 * c.shoulder; };
                    const double sHi = project(hi.xy, hi.s, p).station, sLo = project(lo.xy, lo.s, p).station;
                    auto tangent = [](const EdgeSpec& e, double st) { const Vec2 q0 = pointAt(e.xy, e.s, std::max(0.0, st - 0.5)), q1 = pointAt(e.xy, e.s, std::min(e.s.back(), st + 0.5)); const Vec2 d = q1 - q0; const double L = std::hypot(d.x, d.y); return L > 1e-9 ? d / L : Vec2(1, 0); };
                    const Vec2 th = tangent(hi, sHi), tl = tangent(lo, sLo); const double sinT = std::max(0.25, std::fabs(cross(th, tl)));
                    const double span = (0.5 * pavedW(lo) + 0.5 * pavedW(hi) + 1.0) / sinT;
                    // clear the lower road's HIGHEST point under the upper deck, not its height at the centreline:
                    // the pass climbs 12% under the island freeway, a metre across its 20 m carriageway. Along the lower road
                    // the two bands overlap for the upper's half width over the sine AND the lower's own half width over the
                    // tangent: at a skew its outer lane passes under the upper's edge further on (the island loop over the
                    // pass at 28 deg: 6 m on, the pass 13.9 % up, 21 cm short)
                    const double cosT = std::sqrt(std::max(0.0, 1.0 - sinT * sinT));
                    const double spanLo = (0.5 * pavedW(hi) + 1.0) / sinT + 0.5 * pavedW(lo) * cosT / sinT;
                    double zLo = aHi ? zb : za;
                    for (int k = -4; k <= 4; ++k) zLo = std::max(zLo, interp(lo.s, lo.z, std::clamp(sLo + spanLo * k / 4.0, 0.0, lo.s.back())));
                    // over a TRACK, the thicker of the two slabs, as the clearance invariant measures it (a footbridge over
                    // the 0.8 m bed, a link over the island loop, both came up short with the upper's own). Road over road
                    // keeps the upper's: the thicker one lifted the island's diamond ramps over the freeway 0.2 m, steeper.
                    const bool overTrack = g.cls(hi).track || g.cls(lo).track;
                    // (and a track 0.2 m more: the street under it settles a few cm after the lift, as the agree passes
                    // level its junctions -- Weyby's p1_s115 came 9 cm short under the light rail, link35 7 cm under the loop)
                    const double clear = planned && planned->clearance >= 0.0 ? planned->clearance : R.underClearance;
                    const double need = zLo + clear + (overTrack ? std::max(g.cls(hi).thick, g.cls(lo).thick) + 0.2 : g.cls(hi).thick) + R.structureDepth;
                    const double sts[3] = {std::max(0.0, sHi - span), sHi, std::min(hi.s.back(), sHi + span)};
                    bool low = false; for (double st : sts) if (interp(hi.s, hi.z, st) < need - 0.02) low = true;
                    // A STATION OVER ITS WAY IN (a city's light rail: the way passes under, a flight up to each platform):
                    // the scene's hold on the track over its platforms, covering this crossing, is set LEVEL along its
                    // reach -- as high as this way needs, or as the track stands anywhere along it (a lift over a street
                    // beside the station would otherwise tilt its platforms)
                    if (g.cls(hi).track && g.cls(lo).footway && !hi.platforms.empty()) {
                        bool held = false;
                        for (auto& fp : hi.floorPts) {
                            const double sc = project(hi.xy, hi.s, Vec2(fp[0], fp[1])).station;
                            if (fp[3] <= span || std::fabs(sHi - sc) > fp[3]) continue;
                            double zMax = -1e30, zMin = 1e30;
                            for (int k = -8; k <= 8; ++k) {
                                const double z = interp(hi.s, hi.z, std::clamp(sc + fp[3] * k / 8.0, 0.0, hi.s.back()));
                                zMax = std::max(zMax, z); zMin = std::min(zMin, z);
                            }
                            held = true;
                            if (zMax - zMin <= 0.3 && interp(hi.s, hi.z, sHi) >= need - 0.02) continue;
                            fp[2] = std::max({fp[2], need, zMax});
                            any = true; ++lifted;
                            throughProfile(hi, terrain, g.cls(hi));
                        }
                        if (held) continue;
                    }
                    if (!low) continue;
                    static const bool trace = std::getenv("LANELAB_TRACE_CLEARANCE") != nullptr;
                    if (trace) std::fprintf(stderr, "clearance: %s over %s at (%.0f, %.0f) need %.2f, z before %.2f %.2f %.2f (span %.1f)", hi.id.c_str(), lo.id.c_str(), p.x, p.y, need, interp(hi.s, hi.z, sts[0]), interp(hi.s, hi.z, sts[1]), interp(hi.s, hi.z, sts[2]), span);
                    // (a hold the scene placed with no height -- its reach only, z far below -- is this crossing's within
                    // 2 m: it put it on the line, the spine is resampled. Any other merges only at the point: a wider merge
                    // folded the diamonds' neighbouring holds into one and steepened their ramps.)
                    { bool have = false; for (auto& fp : hi.floorPts) if (std::hypot(fp[0] - p.x, fp[1] - p.y) < (fp[2] < -100.0 ? 2.0 : 0.5)) { fp[2] = std::max(fp[2], need); fp[3] = std::max(fp[3], span); have = true; } if (!have) hi.floorPts.push_back({p.x, p.y, need, span}); }
                    any = true; ++lifted;
                    if (hi.isRamp()) rampProfile(g, hi, terrain); else throughProfile(hi, terrain, g.cls(hi));
                    if (trace) std::fprintf(stderr, " -> after %.2f %.2f %.2f\n", interp(hi.s, hi.z, sts[0]), interp(hi.s, hi.z, sts[1]), interp(hi.s, hi.z, sts[2]));
                }
            }
            if (!any) break;
            agree(false);   // lifted decks must still agree at their nodes and level crossings
        }
        if (lifted) std::fprintf(stderr, "lanelab: %d grade-separated crossings lifted to clear their structure\n", lifted);
        rep.lifted = lifted;
    }
    { NodeMismatchWhere w; const double nm = nodeMismatch(g, R.endpointTol, R.bridgeH, &w), xm = crossingConsistency(g, meets, R.bridgeH, R.rampLevelDz); chainTrackFloors(g, R.endpointTol); for (EdgeSpec& e : g.edges) if (!e.isRamp()) applyFloors(e, g.cls(e)); rep.nodeMismatchAfter = std::max(nm, xm);
      if (nm >= xm && nm > 0.02) rep.nodeMismatchWhere = w.a + (w.atEnd ? " meets " : " ends on ") + w.b + " at (" + std::to_string(static_cast<int>(w.at.x)) + ", " + std::to_string(static_cast<int>(w.at.y)) + ")"; }
    for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);
    rep.undeclared = undeclaredCrossings(g, meets);
    return rep;
}

}  // namespace roads::lanes
}  // namespace engine
