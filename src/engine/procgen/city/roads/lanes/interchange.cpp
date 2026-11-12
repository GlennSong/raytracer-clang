#include "engine/procgen/city/roads/lanes/interchange.h"

#include "engine/procgen/city/roads/lanes/lane_graph_build.h"   // kSolidBeforeStopLine: where a merge lane must have ended
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace engine {
namespace roads::lanes {
namespace {

nlohmann::json pointList(const std::vector<Vec2>& xy) {
    nlohmann::json a = nlohmann::json::array();
    for (const Vec2& p : xy) a.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100});
    return a;
}


}  // namespace

DiamondResult diamondRamps(const std::vector<Vec2>& route, const std::vector<RampStreet>& streets,
                           const DiamondOptions& o) {
    DiamondResult out;
    if (route.size() < 2) return out;
    const std::vector<double> cs = stations(route);
    const double routeLen = cs.back();
    // Offsets from the centreline, all measured outward on the ramp's side: the auxiliary lane
    // the builder lays beside the outer lane (where a ramp spine starts), and the band the ramp
    // runs in, clear of the deck by more than the pavement's closing distance.
    const double auxOff = o.carriage + o.freewayLanes * o.freewayLaneW / 2 + o.freewayLaneW / 2;
    const double rBand = o.edgeReach + o.rampHalf + o.bandGap;
    // Right-hand traffic in plan coordinates (keep-left on screen: see interchange.h): a, which travels with increasing station, sits at -normal. Its
    // outer (right-hand) edge faces AWAY from the median, and that is the edge its ramps use.
    const double aSide = -1.0;
    std::vector<double> routeZ;
    if (o.ground) { HeightField hf = o.ground; routeZ = profileAlong(route, hf, o.window, o.gMax); }
    auto clampS = [&](double st) { return std::max(0.0, std::min(routeLen, st)); };
    auto deckZ = [&](double st) {
        if (o.deck) return o.deck(clampS(st));
        return routeZ.empty() ? 0.0 : interp(cs, routeZ, clampS(st)) + o.clearance;
    };
    auto groundAt = [&](const Vec2& p) { return o.ground ? o.ground(p.x, p.y) : 0.0; };
    auto atS = [&](double st) { return pointAt(route, cs, clampS(st)); };
    auto nrmS = [&](double st) { return perp(tangentAtStation(route, cs, clampS(st))); };
    // The band line on one side, over a window of stations.
    auto bandLine = [&](double s0, double s1, double side) {
        std::vector<Vec2> pts;
        for (double st = clampS(s0); st < clampS(s1); st += 4.0) pts.push_back(atS(st) + nrmS(st) * (rBand * side));
        pts.push_back(atS(s1) + nrmS(s1) * (rBand * side));
        return pts;
    };

    // Streets closed by a diamond already built, and streets a built diamond stands on or lands on.
    std::vector<char> closed(streets.size(), 0), used(streets.size(), 0);
    // Candidates: every street crossing — a ramp meeting its street at 20
    // degrees is a merge, not a junction.
    struct Cand { double s; Vec2 x; std::size_t street; double angle; };
    std::vector<Cand> cands;
    for (std::size_t ci = 0; ci < streets.size(); ++ci) {
        const RampStreet& c = streets[ci];
        if (c.xy.size() < 2) continue;
        const std::vector<double> ss = stations(c.xy);
        for (const Vec2& x : crossings(route, c.xy)) {
            const Projection pr = project(route, cs, x);
            const Projection pc = project(c.xy, ss, x);
            const Vec2 tr = tangentAtStation(route, cs, pr.station);
            const Vec2 tc = tangentAt(c.xy, pc.segment);
            const double d = std::fabs(dot(tr, tc));
            cands.push_back({pr.station, x, ci, std::acos(std::min(1.0, d)) * 180.0 / M_PI});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.angle > b.angle; });
    out.candidates = static_cast<int>(cands.size());

    // The terminal on one side: where that side's band crosses a CROSS street — the crossing
    // street or, where a junction under the deck cuts it into chains, the nearest cross street
    // the band meets within `splitReach` of it — the point both of that side's ramps meet. Not "the
    // nearest street out there": on a freeway with frontage roads that search found the
    // frontage road, running parallel, and the ramp's run-out along it doubled back into a
    // loop. The street must carry on at least `clear` past the terminal, away from the
    // freeway, or the ramp lands in the mouth of its next junction.
    auto terminal = [&](const Cand& cd, double side, double clear, Vec2& term, double& sTerm, std::size_t& street) {
        const std::vector<Vec2> band = bandLine(cd.s - o.splitReach, cd.s + o.splitReach, side);
        const Vec2 t = tangentAtStation(route, cs, clampS(cd.s)), n = nrmS(cd.s) * side;
        double best = 1e300;
        for (std::size_t ci = 0; ci < streets.size(); ++ci) {
            const RampStreet& c = streets[ci];
            if (c.xy.size() < 2 || closed[ci]) continue;
            const std::vector<double> ss = stations(c.xy);
            for (const Vec2& x : crossings(band, c.xy)) {
                if (std::fabs(dot(tangentAt(c.xy, project(c.xy, ss, x).segment), t)) > 0.85) continue;   // parallel: not a cross street
                bool continues = false;
                for (const Vec2& end : {c.xy.front(), c.xy.back()})
                    if (dot(end - x, n) > clear) continues = true;
                const double d = distance(x, cd.x) + (ci == cd.street ? 0.0 : 5.0);   // the crossing street itself, if it will do
                if (continues && d < best) { best = d; term = x; street = ci; }
            }
        }
        if (best > 1e299) return false;
        sTerm = project(route, cs, term).station;
        return true;
    };
    // THE ROAD BESIDE THE FREEWAY at a station, if there is one: the first street out past the
    // band on that side that runs parallel to the route. Its offset, and which street.
    auto besideAt = [&](double st, double side, double& offset, std::size_t& street) {
        const Vec2 p0 = atS(st), n = nrmS(st) * side, t = tangentAtStation(route, cs, clampS(st));
        const std::vector<Vec2> ray{p0 + n * (rBand + 2.0), p0 + n * o.frontageReach};
        offset = 1e300;
        for (std::size_t ci = 0; ci < streets.size(); ++ci) {
            const RampStreet& c = streets[ci];
            if (c.xy.size() < 2) continue;
            const std::vector<double> ss = stations(c.xy);
            for (const Vec2& x : crossings(ray, c.xy)) {
                if (std::fabs(dot(tangentAt(c.xy, project(c.xy, ss, x).segment), t)) < 0.85) continue;   // a cross street
                const double d = dot(x - p0, n);
                if (d < offset) { offset = d; street = ci; }
            }
        }
        return offset < 1e299;
    };
    // The FREE run a ramp needs to fall `climb` at its own grade: the builder eases a ramp's
    // height from its departure to its arrival along a smoothstep, whose steepest point is 1.5x
    // its mean (vertical_profile.cpp, rampProfile). Free means clear of both hosts' reach: it
    // ends where the ramp comes within a half-width of the street it lands on.
    // + 15% of climb and 15 m of run for the estimate's slack: the street a ramp lands on is its own
    // smoothed profile, not the raw ground, and at the mountain foot it stood a metre or more higher
    // + 25 m for the ramp's first stretch, still within reach of the carriageway it leaves, which
    // the builder does not count as free run either
    const double peak = o.rampPeak > 0.0 ? o.rampPeak : o.gRamp / 1.15;
    // x1.2: the planned deck still runs a little under the built one (the builder smooths and agrees its profile
    // after); without it 5 of 44 island ramps came out 2-15% short of their climb
    auto runFor = [&](double climb) { return std::fabs(climb) * 1.2 / peak * 1.5 + 40.0; };

    // A four-way's spine, in its direction of travel. It starts on the auxiliary lane (the builder
    // replaces the first/last point with the exact gore), eases out to the band over `diverge`
    // and falls in it, then runs straight to the crossing street. (A touch-down's: landOn.)
    auto spine = [&](double sGore, double sTerm, double side, const Vec2& term, bool off) {
        std::vector<Vec2> pts;
        const double dir = sTerm > sGore ? 1.0 : -1.0, span = std::fabs(sTerm - sGore);
        for (double d = 0; d < span - 2.0; d += 4.0) {
            const double st = sGore + dir * d;
            const double r = auxOff + (rBand - auxOff) * smoothstep01(d / o.diverge);
            pts.push_back(atS(st) + nrmS(st) * (r * side));
        }
        pts.push_back(term);
        if (!off) std::reverse(pts.begin(), pts.end());   // an on-ramp reads street -> band -> gore
        return pts;
    };
    // Every other street the ramp crosses on its way down. Near the gore the ramp is still at
    // deck height and bridges it; partway down it can do neither.
    // (Except in its landing: the stretch where it runs along or into the street it lands on it is at
    // street level on purpose, and passing the side streets that meet that street is a merge, not a
    // conflict — a town's grid met its main street there and refused its diamond.)
    // Its height is the builder's: level across the arrival, a smoothstep over the free run — near the
    // gore it is still at deck height, and a straight line from deck to street put it a metre or
    // more too low there, refusing a ramp for bridging a street it clears.
    std::vector<std::size_t> closing;   // the closable streets the candidate under test would close
    auto conflicts = [&](const std::vector<Vec2>& sp, std::size_t landing, std::size_t landing2, double zStart, double zEnd, bool landsAtEnd, double arrive) {
        const std::vector<double> ss = stations(sp);
        const double land = o.along + o.landing + 10.0;
        for (std::size_t ci = 0; ci < streets.size(); ++ci) {
            if (ci == landing || ci == landing2 || streets[ci].xy.size() < 2 || closed[ci]) continue;
            for (const Vec2& x : crossings(sp, streets[ci].xy)) {
                const double sx = project(sp, ss, x).station;
                if (landsAtEnd ? sx > ss.back() - land : sx < land) continue;
                const double run = std::max(ss.back() - arrive, 1.0);
                const double z = zStart + (zEnd - zStart) * smoothstep01(landsAtEnd ? sx / run : (sx - arrive) / run);
                if (z - groundAt(x) < o.clearance - 0.2) {
                    if (streets[ci].closable && !used[ci]) { closing.push_back(ci); continue; }
                    if (std::getenv("RT_DIAMOND_WHY")) std::fprintf(stderr, "    spine meets %s at %.0f of %.0f m, %.1f m up\n", streets[ci].id.c_str(), sx, ss.back(), z - groundAt(x));
                    return true;
                }
            }
        }
        return false;
    };

    // THE TOUCH-DOWN, ON THE CARRIAGEWAY GOING THE RAMP'S WAY (#282; interchange.h). A ramp's traffic travels the
    // freeway's way (`fwd` along the route) where it meets the road beside the freeway, so it joins (an off-ramp) or
    // leaves (an on-ramp) that road's carriageway travelling that way, by a lane of its own beside its kerb lane: the
    // anchor the builder composes the merge or deceleration lane from. When that carriageway is the far one, the ramp
    // crosses OVER the road first, holding the builder's clearance there (a floor point: its rule, under_clearance + the
    // thicker slab + structure depth), comes down on the far side, then turns in to its lane. Its heights are planned
    // by the builder's own model (rampFreeHeight), so every street it passes is cleared or refused here, not found out
    // in the build.
    const Rules builderRules;   // the builder's rules as the planner writes the graph (it writes no "rules" of its own)
    const RoadClassSpec& rampClass = roadClass("ramp");
    auto needOver = [&](const std::string& cls) {   // a ramp's deck over a street's, surface to surface
        return builderRules.underClearance + std::max(rampClass.thick, roadClass(cls).thick) + builderRules.structureDepth;
    };
    struct Landing {
        bool far = false, forward = true;
        std::string side;                 // the anchor's lane: "kerb" (the street's forward lanes) or "back-kerb" (its backward ones)
        double sAnchor = 0;               // the freeway station of the gore on the street
        Vec2 gore;                        // the gore's place on the street's centreline (the anchor's "at")
        std::vector<Vec2> spine;          // freeway gore -> street gore (reversed for an on-ramp when written)
        bool held = false; Vec2 holdAt; double holdZ = 0, holdHalf = 0, clearance = 0; std::size_t holdStreet = 0;
    };
    auto landOn = [&](double sCross, double sTouch, double side, double fwd, bool off, std::size_t k, double f, Landing& L) -> bool {
        static const bool why = std::getenv("RT_DIAMOND_WHY") != nullptr;
        static const bool whyAll = why && std::string(std::getenv("RT_DIAMOND_WHY")) == "2";   // and every length tried
        const RampStreet& fr = streets[k];
        const RoadClassSpec& sc = roadClass(fr.cls.empty() ? std::string("local") : fr.cls);
        const std::vector<LaneSlot> slots = laneSlots(sc, sc.lanes);
        const std::vector<double> ss = stations(fr.xy);
        const double len = ss.back();
        const Vec2 n = nrmS(sTouch) * side;   // outward, away from the freeway
        const Projection at = project(fr.xy, ss, atS(sTouch) + n * f);
        const Vec2 tS = tangentAt(fr.xy, at.segment);
        // the carriageway going the ramp's way, and its kerb lane
        L.forward = dot(tangentAtStation(route, cs, clampS(sTouch)) * fwd, tS) > 0;
        const int count = L.forward ? sc.lanes.fwd : sc.lanes.back;
        if (count < 1) { if (why) std::fprintf(stderr, "    %s: no carriageway going its way\n", fr.id.c_str()); return false; }
        const std::string want = (L.forward ? "f" : "b") + std::to_string(count - 1);
        double kerbOff = 0, reach = 0;
        bool found = false;
        for (const LaneSlot& sl : slots) {
            reach = std::max(reach, std::fabs(sl.offset));
            if (sl.name == want) { kerbOff = sl.offset; found = true; }
        }
        if (!found) return false;
        const double edge = reach + sc.lanes.w / 2 + sc.shoulder + sc.sidewalk;   // the street's outer edge, sidewalk and all
        L.side = L.forward ? "kerb" : "back-kerb";
        L.far = dot(perp(tS) * kerbOff, n) > 0;
        // offsets from the freeway's centreline, outward: the lane of its own beside the kerb lane, the far side's band
        const double rAux = f + (L.far ? 1.0 : -1.0) * (std::fabs(kerbOff) + sc.lanes.w);
        const double rFar = f + edge + o.bandGap + o.rampHalf;
        // THE GORE ON THE STREET: the lane of its own lies on this one street (the builder lays it along its host's
        // stations), after the gore for an off-ramp, before it for an on-ramp. An off-ramp's merge lane has ended
        // before the junction ahead's mouth and the solid lines before its stop line (no car changes lanes there:
        // lane_graph_build.h); an on-ramp's deceleration lane begins `mouth` past the junction behind. The other end
        // of the lane of its own keeps `mouth` from a junction too, and only a little from where the street merely
        // runs on as the next piece of the same road.
        const double lane = off ? o.streetAux + o.streetTaper : o.streetDecel + o.streetTaper;
        const bool aheadAtEnd = L.forward;   // the street's end its traffic runs toward: its last point when forward
        auto junctionAt = [&](bool atEnd) {
            const Vec2 q = atEnd ? fr.xy.back() : fr.xy.front();
            int meeting = 0;
            for (std::size_t ci = 0; ci < streets.size(); ++ci)
                if (ci != k && streets[ci].xy.size() >= 2 && (distance(streets[ci].xy.front(), q) < 1.0 || distance(streets[ci].xy.back(), q) < 1.0)) ++meeting;
            return meeting != 1;   // one other road's end there: the same road running on
        };
        const double marginAhead = off ? o.mouth + kSolidBeforeStopLine : (junctionAt(aheadAtEnd) ? o.mouth : 2.0);
        const double marginBehind = off ? (junctionAt(!aheadAtEnd) ? o.mouth : 2.0) : o.mouth;
        // in "how far along the traffic's way" from the end behind: the gore, then (off) its lane after it
        const double uG = off ? len - marginAhead - lane : marginBehind + lane;
        if ((off && uG < marginBehind) || (!off && uG > len - marginAhead)) {
            if (why) std::fprintf(stderr, "    %s: %.0f m long, no room for %.0f m of lane and its mouths (%.0f, %.0f)\n", fr.id.c_str(), len, lane, marginBehind, marginAhead);
            return false;
        }
        const double stG = aheadAtEnd ? uG : len - uG;
        L.gore = pointAt(fr.xy, ss, stG);
        L.sAnchor = project(route, cs, L.gore).station;
        const double away = off ? -fwd : fwd;   // off: the freeway gore is upstream; on: downstream
        if ((L.sAnchor - sCross) * away < 0) { if (why) std::fprintf(stderr, "    %s: its gore falls past the crossing\n", fr.id.c_str()); return false; }
        const double zStreet = groundAt(L.gore);
        const double clear = needOver(fr.cls.empty() ? std::string("local") : fr.cls);
        const double gd = kDesignGrade * rampClass.gMax;
        const double crossLen = o.crossPerMetre * std::fabs(rFar - rBand);
        const double descend = clear / gd + 10.0;   // the far side's run down from the hold, at the design grade, and a little
        const double before = L.far ? crossLen + descend : 0.0;
        // the shortest ramp that fits: its heights as the builder will make them (rampFreeHeight) meet the hold, are down
        // by the time it turns in to its lane, and clear every street it passes
        for (double span = o.diverge + before + o.shift + 10.0; span < 1200.0; span += 10.0) {
            const double sGore = L.sAnchor + away * span, dir = -away;
            if (sGore < 0.0 || sGore > routeLen) break;
            const double dCrossEnd = span - o.shift - descend;   // where it reaches the far side's band
            std::vector<Vec2> pts;
            std::vector<double> rs;
            for (double d = 0; d < span - 2.0; d += 4.0) {
                double r = auxOff + (rBand - auxOff) * smoothstep01(d / o.diverge);
                if (L.far) r += (rFar - rBand) * smoothstep01((d - (dCrossEnd - crossLen)) / crossLen);
                const double rLast = L.far ? rFar : rBand;
                r += (rAux - rLast) * smoothstep01((d - (span - o.shift)) / o.shift);
                const double st = sGore + dir * d;
                pts.push_back(atS(st) + nrmS(st) * (r * side));
                rs.push_back(r);
            }
            pts.push_back(atS(L.sAnchor) + nrmS(L.sAnchor) * (rAux * side));
            rs.push_back(rAux);
            const std::vector<double> ds = stations(pts);
            const double za = deckZ(sGore), zb = zStreet;
            if (std::fabs(za - zb) / rampClass.gMax * 1.5 > ds.back()) continue;   // the builder's own check: free run for the climb
            std::vector<RampHold> holds;
            L.held = false;
            if (L.far) {
                // the hold: centred where it crosses the street's centreline, over the stretch either side of that
                // where it is within its half-width of the street's outer edge (not its turn in to its lane at the end)
                double in = -1, out = -1, centre = -1;
                std::size_t ic = 0;
                for (std::size_t i = 1; i < rs.size() && centre < 0; ++i)
                    if ((rs[i - 1] - f) * (rs[i] - f) <= 0) { centre = ds[i - 1] + (ds[i] - ds[i - 1]) * (f - rs[i - 1]) / (rs[i] - rs[i - 1] + 1e-12); ic = i; }
                if (centre >= 0) {
                    std::size_t a = ic, b = ic;
                    while (a > 0 && std::fabs(rs[a - 1] - f) < edge + o.rampHalf) --a;
                    while (b + 1 < rs.size() && std::fabs(rs[b + 1] - f) < edge + o.rampHalf) ++b;
                    in = ds[a];
                    out = ds[b];
                }
                if (in < 0 || centre < 0 || centre < o.diverge) { if (whyAll) std::fprintf(stderr, "      %.0f m: no hold (in %.0f centre %.0f)\n", ds.back(), in, centre); continue; }
                L.held = true;
                L.holdHalf = std::max(centre - in, out - centre) + 4.0;   // a sample's slack either side
                L.holdAt = pointAt(pts, ds, centre);
                L.holdZ = groundAt(L.holdAt) + clear;
                L.clearance = clear;
                holds.push_back({centre, L.holdZ, L.holdHalf});
            }
            auto zAt = [&](double d) { return rampFreeHeight(d, 0.0, ds.back(), za, zb, holds, rampClass.gMax); };
            bool ok = true;
            if (L.held)
                for (double d = std::max(0.0, holds[0].s - holds[0].half); d <= holds[0].s + holds[0].half && ok; d += 1.0)
                    if (zAt(d) < holds[0].z - 0.05) ok = false;
            if (!ok) { if (whyAll) std::fprintf(stderr, "      %.0f m: under its hold (%.1f m wanted at %.0f m, half %.0f)\n", ds.back(), holds[0].z, holds[0].s, holds[0].half); continue; }
            if (zAt(ds.back() - o.shift) - zb > builderRules.sameLevelDz) {   // still up in the air where it turns in
                if (whyAll) std::fprintf(stderr, "      %.0f m: %.1f m up where it turns in\n", ds.back(), zAt(ds.back() - o.shift) - zb);
                continue;
            }
            // every street it passes: cleared by the builder's clearance, or a refusal (a closable one closes)
            std::vector<std::size_t> closes;
            bool blocked = false;
            for (std::size_t ci = 0; ci < streets.size() && !blocked; ++ci) {
                if (streets[ci].xy.size() < 2 || closed[ci]) continue;
                for (const Vec2& x : crossings(pts, streets[ci].xy)) {
                    const double dx = project(pts, ds, x).station;
                    if (dx > ds.back() - 1.0) continue;   // the gore itself, on its lane beside the street
                    const double over = zAt(dx) - groundAt(x);
                    if (over >= needOver(streets[ci].cls.empty() ? std::string("local") : streets[ci].cls) - 0.2) {
                        if (L.held && distance(x, L.holdAt) < L.holdHalf) L.holdStreet = ci;
                        continue;
                    }
                    if (streets[ci].closable && !used[ci]) { closes.push_back(ci); continue; }
                    if (why) std::fprintf(stderr, "    landing on %s, %.0f m long: passes %s at %.0f m along, %.1f m over it\n", fr.id.c_str(), ds.back(), streets[ci].id.c_str(), dx, over);
                    // past the crossing's start, the ramp is on its way down to a gore that does not move: a longer ramp
                    // passes the same street as low
                    if (dx > dCrossEnd - crossLen) return false;
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            for (std::size_t ci : closes) closing.push_back(ci);
            L.spine = pts;
            return true;
        }
        if (why) std::fprintf(stderr, "    landing on %s: no length fits\n", fr.id.c_str());
        return false;
    };

    // Which crossing next: the one FARTHEST from the diamonds already built and from the route's
    // ends and keep-outs (squareness weighing in), so however many fit, they spread evenly along
    // the route instead of crowding wherever the squarest crossings happen to be. Every refusal
    // is final — it depends only on what is already built, which only grows — so each candidate
    // is tried once.
    std::vector<double> taken;
    std::vector<char> tried(cands.size(), 0);
    auto openness = [&](const Cand& cd) {
        double d = std::min(cd.s, routeLen - cd.s);
        for (double t : taken) d = std::min(d, std::fabs(t - cd.s));
        for (const auto& k : o.keepOut) d = std::min(d, cd.s < k.first ? k.first - cd.s : cd.s > k.second ? cd.s - k.second : 0.0);
        return d * std::sin(cd.angle * M_PI / 180.0);
    };
    while (out.built < o.maxDiamonds) {
        std::size_t pick = cands.size();
        for (std::size_t i = 0; i < cands.size(); ++i)
            if (!tried[i] && (pick == cands.size() || openness(cands[i]) > openness(cands[pick]))) pick = i;
        if (pick == cands.size()) break;
        tried[pick] = 1;
        const Cand& cd = cands[pick];
        if (closed[cd.street]) continue;   // a diamond already built closed this street
        closing.clear();
        if (cd.angle < 32.0) { ++out.rejectedOblique; out.squarestRejected = std::max(out.squarestRejected, cd.angle); continue; }
        bool near = false;
        for (double t : taken) if (std::fabs(t - cd.s) < o.spacing) near = true;
        if (near) {
            if (std::getenv("RT_DIAMOND_WHY")) std::fprintf(stderr, "[diamond] %s crossing at s=%.0f (street %s): within %.0f m of a diamond\n", o.idPrefix.c_str(), cd.s, streets[cd.street].id.c_str(), o.spacing);
            ++out.rejectedSpacing; continue;
        }
        // Each side: a four-way on the cross street where the band meets it, if that street
        // carries on well clear of its next junction; else a touch-down on the road beside the
        // freeway, the off-ramp upstream of the crossing and the on-ramp downstream of it; else,
        // with no road beside, a four-way closer to the next junction.
        Vec2 tA, tB;
        double sA = 0, sB = 0;
        std::size_t kA = cd.street, kB = cd.street;
        const bool crossA = terminal(cd, aSide, o.mouth, tA, sA, kA), crossB = terminal(cd, -aSide, o.mouth, tB, sB, kB);
        // Four ramps. `fwd` is the carriageway's direction of travel in station: a exits
        // upstream of its terminal and merges downstream of it; b is the mirror.
        struct Plan { const char* name; const std::string* host; bool off; double side, fwd; bool cross; double sTerm; Vec2 term; double sGore; std::size_t street; double beside; bool lands; Landing landing; };
        Plan ps[4] = {{"a_off", &o.aId, true, aSide, 1.0, crossA, sA, tA, 0, kA, 0, false, {}}, {"a_on", &o.aId, false, aSide, 1.0, crossA, sA, tA, 0, kA, 0, false, {}},
                      {"b_off", &o.bId, true, -aSide, -1.0, crossB, sB, tB, 0, kB, 0, false, {}}, {"b_on", &o.bId, false, -aSide, -1.0, crossB, sB, tB, 0, kB, 0, false, {}}};
        bool found = true, room = true, clear = true, landing = true;
        std::vector<std::vector<Vec2>> spines;
        for (Plan& p : ps) {
            const double away = p.off ? -p.fwd : p.fwd;   // off: the gore is upstream; on: downstream
            if (!p.cross) {
                // the touch-down, `mouth` clear of the crossing, on the road beside the freeway, on the carriageway
                // going the ramp's way (landOn); where it will not fit, a four-way nearer the next junction
                const double sTouch = cd.s + away * o.mouth;
                double f = 0; std::size_t k = 0;
                if (besideAt(sTouch, p.side, f, k)) {
                    if (!landOn(cd.s, sTouch, p.side, p.fwd, p.off, k, f, p.landing)) { found = false; landing = false; break; }   // no room to land as it must
                    p.lands = true; p.street = k; p.beside = f;
                    p.sTerm = p.landing.sAnchor; p.term = p.landing.gore;
                } else if (terminal(cd, p.side, o.crossMouth, p.term, p.sTerm, p.street)) {
                    p.cross = true;   // no road beside to land on: a four-way nearer the next junction
                } else { found = false; break; }
            }
            const double lead = p.off ? o.decel + o.taperOff : o.aux + o.taperOn;
            if (p.lands) {
                // its length, its spine and the streets it passes were settled by landOn
                std::vector<Vec2> sp = p.landing.spine;
                p.sGore = project(route, cs, sp.front()).station;
                if (!p.off) std::reverse(sp.begin(), sp.end());   // an on-ramp reads street -> band -> gore
                spines.push_back(sp);
            } else {
                // free run, then the part within reach of the landing street: the level approach
                // across the street's half-width
                const double arrive = o.landing;
                // THE SHORTEST RAMP LONG ENOUGH FOR THE CLIMB AT ITS OWN GORE (#40): the deck falls away from the
                // crossing it bridges, so the climb depends on the length. Two fixed-point steps (the climb at the
                // crossing, then at that long ramp's gore) landed on the short answer -- 165 m for a gore 14.8 m up,
                // a 15% ramp by the builder's own check. Scan out until the run covers the climb where it ends.
                const double zTermG = groundAt(p.term);
                double L = std::max(runFor(deckZ(p.sTerm) - zTermG), o.diverge) + arrive;   // the upper bound: the peak's climb
                for (double Ltry = o.diverge + arrive; Ltry < L; Ltry += 10.0)
                    if (Ltry >= std::max(runFor(deckZ(p.sTerm + away * Ltry) - zTermG), o.diverge) + arrive) { L = Ltry; break; }
                p.sGore = p.sTerm + away * L;
                spines.push_back(spine(p.sGore, p.sTerm, p.side, p.term, p.off));
                const double zGore = deckZ(p.sGore), zTerm = groundAt(p.term);
                if (conflicts(spines.back(), cd.street, p.street, p.off ? zGore : zTerm, p.off ? zTerm : zGore, p.off, arrive)) clear = false;
            }
            const double lo = std::min({p.sGore, p.sGore + away * lead, p.sTerm}), hi = std::max({p.sGore, p.sGore + away * lead, p.sTerm});
            if (lo < 20.0 || hi > routeLen - 20.0) room = false;
            for (const auto& kk : o.keepOut) if (lo < kk.second && hi > kk.first) room = false;
            // ...nor may its decel or aux lane, up on the freeway, pass over a street crossing under
            // it: that lane is one with the ramp, which comes down to the ground, and a lane both
            // stacked over a street and level with the streets it meets tore the deck (the SW town).
            const double l0 = std::min(p.sGore, p.sGore + away * lead), l1 = std::max(p.sGore, p.sGore + away * lead);
            for (std::size_t ci = 0; ci < streets.size() && clear; ++ci) {
                if (ci == cd.street || ci == p.street || streets[ci].xy.size() < 2 || closed[ci]) continue;
                for (const Vec2& x : crossings(route, streets[ci].xy)) {
                    const double sx = project(route, cs, x).station;
                    if (sx > l0 - 10 && sx < l1 + 10) {
                        if (streets[ci].closable && !used[ci]) { closing.push_back(ci); break; }
                        if (std::getenv("RT_DIAMOND_WHY")) std::fprintf(stderr, "    lead lane over %s at s=%.0f\n", streets[ci].id.c_str(), sx);
                        clear = false; break;
                    }
                }
            }
        }
        for (std::size_t ci : closing)
            if (ci == cd.street || ci == ps[0].street || ci == ps[1].street || ci == ps[2].street || ci == ps[3].street) clear = false;
        static const bool why = std::getenv("RT_DIAMOND_WHY") != nullptr;
        if (why) std::fprintf(stderr, "[diamond] %s crossing at s=%.0f (%.0f deg, street %s): side a %s, side b %s -> %s\n", o.idPrefix.c_str(), cd.s, cd.angle,
                              streets[cd.street].id.c_str(), crossA ? "four-way" : "frontage?", crossB ? "four-way" : "frontage?",
                              !landing ? "no landing" : !found ? "no terminal" : !room ? "no room" : !clear ? "over a street" : "built");
        if (why && !room) { std::fprintf(stderr, "    route %.0f m, keep-outs", routeLen); for (const auto& k : o.keepOut) std::fprintf(stderr, " [%.0f, %.0f]", k.first, k.second); std::fprintf(stderr, "\n"); }
        if (why) for (const Plan& p : ps) std::fprintf(stderr, "    %s: term s=%.0f gore s=%.0f cross=%d lands=%d (%s half, %s) beside=%.0f deck %.1f over ground %.1f (band %.0f)\n", p.name, p.sTerm, p.sGore, p.cross, p.lands,
                                                     p.landing.far ? "far" : "near", p.landing.side.c_str(), p.beside, deckZ(p.sGore), groundAt(p.term), rBand);
        if (!landing) { ++out.rejectedLanding; continue; }
        if (!found) { ++out.rejectedTerminal; continue; }
        if (!room) { ++out.rejectedRoom; continue; }
        if (!clear) { ++out.rejectedConflict; continue; }

        const std::string pre = o.idPrefix + "_" + std::to_string(out.built);
        for (int k = 0; k < 4; ++k) {
            const Plan& p = ps[k];
            const Vec2 gore = atS(p.sGore) + nrmS(p.sGore) * (o.carriage * p.side);
            nlohmann::json e; e["id"] = pre + "_" + p.name; e["class"] = "ramp";
            nlohmann::json anchor = {{"edge", *p.host}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"approach", o.approach}};
            const std::string sid = streets[p.street].id;
            // the street end: a touch-down's anchor on the kerb lane of the carriageway going its way, with its lane of
            // its own (#282); a four-way's bare street, which it meets at a junction
            nlohmann::json street = sid;
            if (p.lands) {
                const Landing& L = p.landing;
                street = {{"edge", sid}, {"at", {std::round(L.gore.x * 100) / 100, std::round(L.gore.y * 100) / 100}}, {"side", L.side}, {"approach", 20.0}};
                if (p.off) { street["aux"] = o.streetAux; street["taper"] = o.streetTaper; }
                else       { street["decel"] = o.streetDecel; street["taper"] = o.streetTaper; }
            }
            if (p.off) { anchor["decel"] = o.decel; anchor["taper"] = o.taperOff; e["from"] = anchor; e["to"] = street; }
            else       { anchor["aux"] = o.aux;     anchor["taper"] = o.taperOn;  e["to"] = anchor; e["from"] = street; }
            e["path"] = {{"type", "polyline"}, {"points", pointList(spines[static_cast<std::size_t>(k)])}};
            // THE LANDING AS PLANNED (#282): which street, which of its carriageways and how the ramp gets there -- for
            // the builder to check (checkRampLandings), never to work out
            nlohmann::json landing = {{"street", sid}, {"joins", p.lands ? "merge" : "junction"}};
            if (p.lands) {
                const Landing& L = p.landing;
                landing["carriageway"] = L.forward ? "forward" : "backward";
                landing["half"] = L.far ? "far" : "near";   // of the street, from the freeway
                if (L.held) {
                    const double z = std::round(L.holdZ * 100) / 100;
                    const nlohmann::json at = {std::round(L.holdAt.x * 100) / 100, std::round(L.holdAt.y * 100) / 100};
                    landing["crossing"] = {{"kind", "over"}, {"street", streets[L.holdStreet].id}, {"at", at}, {"clearance", std::round(L.clearance * 100) / 100}};
                    e["floor"] = nlohmann::json::array({{at[0], at[1], z, std::round(L.holdHalf * 100) / 100}});
                } else {
                    landing["crossing"] = "none";
                }
            }
            e["landing"] = landing;
            out.ramps.push_back(e);
        }
        taken.push_back(cd.s);
        used[cd.street] = 1;
        for (const Plan& p : ps) used[p.street] = 1;
        for (std::size_t ci : closing)
            if (!closed[ci]) { closed[ci] = 1; out.closed.push_back(streets[ci].id); }
        ++out.built;
    }
    return out;
}

namespace {

// One road as a frame: a point at a station and an offset (+ is left of increasing station).
struct Frame {
    const SystemRoad& road;
    std::vector<double> s;
    explicit Frame(const SystemRoad& r) : road(r), s(stations(r.route)) {}
    double clampS(double st) const { return std::max(0.0, std::min(s.back(), st)); }
    Vec2 at(double st, double off) const {
        const double c = clampS(st);
        return pointAt(road.route, s, c) + perp(tangentAtStation(road.route, s, c)) * off;
    }
    Vec2 tangent(double st) const { return tangentAtStation(road.route, s, clampS(st)); }
    double auxOff() const { return road.carriage + road.lanes * road.laneW / 2 + road.laneW / 2; }
    // the carriageway that runs AWAY from the crossing along ray `sigma`, and the one that runs toward it
    const std::string& outbound(double sigma) const { return sigma > 0 ? road.aId : road.bId; }
    const std::string& inbound(double sigma) const { return sigma > 0 ? road.bId : road.aId; }
};

}  // namespace

double systemReach(const SystemOptions& o) { return o.linkReach + std::max(o.decel + o.taperOff, o.aux + o.taperOn); }

SystemResult systemInterchange(const SystemRoad& through, const SystemRoad& stem, double outward, const SystemOptions& o) {
    SystemResult out;
    const Frame T(through), S(stem);
    const std::vector<Vec2> xs = crossings(through.route, stem.route);
    if (xs.size() != 1) { out.why = xs.empty() ? "the roads do not cross" : "the roads cross more than once"; return out; }
    const Vec2 X = xs.front();
    out.sThrough = project(through.route, T.s, X).station;
    out.sStem = project(stem.route, S.s, X).station;
    const Vec2 tT = T.tangent(out.sThrough), tS = S.tangent(out.sStem);
    if (std::fabs(cross(tT, tS)) < 0.8) { out.why = "the crossing is too oblique for a cloverleaf"; return out; }
    const double rBand = std::max(through.edgeReach, stem.edgeReach) + o.rampHalf + o.bandGap;

    // A quadrant is two rays from the crossing, p along road P and q along road Q, with q on
    // p's RIGHT. The carriageway running out along p and the one running in along q both face
    // it. (u, v) are distances along p and along q, placed in the THROUGH road's own frame —
    // its station and its offset — which is exact for both roads when the stem is a radius of
    // the ring: a radial line IS a line of constant station in a circle's frame. (Blending the
    // two roads' frames instead dented the loops where the ring curves.)
    struct Quadrant { const Frame* P; double sigP; const Frame* Q; double sigQ; };
    auto place = [&](const Quadrant& k, double u, double v) {
        if (k.P == &T) return T.at(out.sThrough + k.sigP * u, -k.sigP * v);   // the quadrant is on the right of p: -normal when p runs with the station
        return T.at(out.sThrough + k.sigQ * v, k.sigQ * u);                   // and on the left of q
    };
    auto rayOf = [&](const Frame& f, double sigma) { return f.tangent(&f == &T ? out.sThrough : out.sStem) * sigma; };
    auto quadrant = [&](const Frame& f1, double sig1, const Frame& f2, double sig2) {
        return cross(rayOf(f1, sig1), rayOf(f2, sig2)) < 0 ? Quadrant{&f1, sig1, &f2, sig2} : Quadrant{&f2, sig2, &f1, sig1};
    };
    // The auxiliary lane's offset from a road's CENTRELINE, on the quadrant side, for the
    // first/last `diverge` metres of a ramp, easing out to the band.
    auto bandOff = [&](const Frame& f, double d) { return f.auxOff() + (rBand - f.auxOff()) * smoothstep01(d / o.diverge); };

    // The four quadrants by what they hold. r is the stem's outward ray; c1 the ring's ray to its right.
    const double sigC1 = cross(rayOf(S, outward), tT) < 0 ? 1.0 : -1.0;
    const Quadrant outR = quadrant(S, outward, T, sigC1), outL = quadrant(T, -sigC1, S, outward);
    const Quadrant inR = quadrant(T, sigC1, S, -outward), inL = quadrant(S, -outward, T, -sigC1);

    auto anchorOff = [&](const std::string& host, const Vec2& gore, double decel, double taper) {
        return nlohmann::json{{"edge", host}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"decel", decel}, {"taper", taper}, {"approach", o.approach}};
    };
    auto anchorOn = [&](const std::string& host, const Vec2& gore, double aux, double taper) {
        return nlohmann::json{{"edge", host}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"aux", aux}, {"taper", taper}, {"approach", o.approach}};
    };
    auto sOf = [&](const Frame* f) { return f == &T ? out.sThrough : out.sStem; };
    // A point on a ramp's run along one of its roads: in that road's own frame at the gore, eased
    // into the ring's frame by the curve (w = 1 at the gore, 0 at the curve), so a stem that is
    // not quite a radius of a wobbling ring leaves no kink where the two meet.
    auto along = [&](const Quadrant& k, bool onP, double dist, double off, double w) {
        const Frame* f = onP ? k.P : k.Q;
        const double sig = onP ? k.sigP : k.sigQ;
        const Vec2 own = f->at(sOf(f) + sig * dist, (onP ? -sig : sig) * off);
        if (f == &T) return own;
        const Vec2 ring = onP ? place(k, dist, off) : place(k, off, dist);
        return ring + (own - ring) * smoothstep01(w);
    };
    auto emit = [&](const std::string& id, const std::vector<Vec2>& spine, nlohmann::json from, nlohmann::json to) {
        nlohmann::json e; e["id"] = id; e["class"] = "ramp"; e["from"] = std::move(from); e["to"] = std::move(to);
        e["path"] = {{"type", "polyline"}, {"points", pointList(resample(spine, 4.0))}};
        out.ramps.push_back(std::move(e));
    };

    // A CORNER CONNECTOR: in along q, a right turn round the corner, out along p.
    auto link = [&](const Quadrant& k, const std::string& name) {
        const double c = rBand + o.linkR, L = o.linkReach;
        std::vector<Vec2> sp;
        for (double v = L; v > c; v -= 4.0) sp.push_back(along(k, false, v, bandOff(*k.Q, L - v), (v - c) / (L - c)));
        for (int i = 0; i <= 24; ++i) {
            const double th = M_PI + 0.5 * M_PI * i / 24.0;   // 180 -> 270 degrees, round the corner
            sp.push_back(place(k, c + o.linkR * std::cos(th), c + o.linkR * std::sin(th)));
        }
        for (double u = c + 4.0; u <= L; u += 4.0) sp.push_back(along(k, true, u, bandOff(*k.P, L - u), (u - c) / (L - c)));
        const Vec2 gIn = k.Q->at(sOf(k.Q) + k.sigQ * L, k.sigQ * k.Q->road.carriage);
        const Vec2 gOut = k.P->at(sOf(k.P) + k.sigP * L, -k.sigP * k.P->road.carriage);
        emit(o.idPrefix + "_" + name + "_link", sp, anchorOff(k.Q->inbound(k.sigQ), gIn, o.decel, o.taperOff),
             anchorOn(k.P->outbound(k.sigP), gOut, o.aux, o.taperOn));
    };
    // A LOOP: out along p past the crossing, 270 degrees round the far side, in along q.
    auto loop = [&](const Quadrant& k, const std::string& name) -> bool {
        const double H = rBand + o.loopR, g = H - o.diverge;
        std::vector<Vec2> sp;
        for (double u = g; u < H; u += 4.0) sp.push_back(along(k, true, u, bandOff(*k.P, u - g), (H - u) / (H - g)));
        for (int i = 0; i <= 72; ++i) {
            const double th = -0.5 * M_PI + 1.5 * M_PI * i / 72.0;   // -90 -> 180 degrees, the far side
            sp.push_back(place(k, H + o.loopR * std::cos(th), H + o.loopR * std::sin(th)));
        }
        for (double v = H - 4.0; v >= g; v -= 4.0) sp.push_back(along(k, false, v, bandOff(*k.Q, v - g), (H - v) / (H - g)));
        const Vec2 gOut = k.P->at(sOf(k.P) + k.sigP * g, -k.sigP * k.P->road.carriage);
        const Vec2 gIn = k.Q->at(sOf(k.Q) + k.sigQ * g, k.sigQ * k.Q->road.carriage);
        // how much lane each end has room for, toward the crossing: it stops clear of the other
        // road's deck. A loop's lane is one lane from its exit to its merge, so a decel lane on
        // the ring reaching back over the stem made one lane both stacked over the stem and level
        // with it at the far end — and the deck tore where the builder had to choose.
        auto room = [&](const Frame* host) { const Frame* other = host == &T ? &S : &T; return std::min(o.loopLane, g - other->road.edgeReach - 6.0); };
        const double lp = room(k.P), lq = room(k.Q);
        if (lp < 20 || lq < 20) { out.why = "no room for a loop's lanes: raise loopR"; return false; }
        emit(o.idPrefix + "_" + name + "_loop", sp, anchorOff(k.P->outbound(k.sigP), gOut, 0.4 * lp, 0.6 * lp),
             anchorOn(k.Q->inbound(k.sigQ), gIn, 0.4 * lq, 0.6 * lq));
        return true;
    };

    link(outR, "outR");   // round the ring the far way -> back out
    link(outL, "outL");   // in from outside -> round the ring the near way
    if (!loop(inR, "inR") || !loop(inL, "inL")) { out.ramps.clear(); return out; }   // left turns: round the ring -> back out, in -> round the ring
    out.reach = systemReach(o);
    out.built = true;
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
