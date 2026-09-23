#include "engine/procgen/city/roads/lanes/interchange.h"

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

double smoothStep(double t) { t = std::clamp(t, 0.0, 1.0); return t * t * (3 - 2 * t); }

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
    auto deckZ = [&](double st) { return routeZ.empty() ? 0.0 : interp(cs, routeZ, clampS(st)) + o.clearance; };
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

    // Candidates: every street crossing, squarest first — a ramp meeting its street at 20
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
            if (c.xy.size() < 2) continue;
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
    auto runFor = [&](double climb) { return std::fabs(climb) / o.gRamp * 1.5 + 15.0; };   // + the deck-height estimate's slack

    // A ramp's spine, in its direction of travel. It starts on the auxiliary lane (the builder
    // replaces the first/last point with the exact gore), eases out to the band over `diverge`
    // and falls in it. A crossing terminal is then a straight run to the street; a frontage
    // terminal shifts across to the road beside the freeway over `shift`, AT GRADE — the builder
    // counts the ramp as arrived once it is within reach of that road — and runs `along` its
    // line to the terminal, finishing ALONG a street, which is how the builder joins them.
    auto spine = [&](double sGore, double sTerm, double side, const Vec2& term, bool off, double rEnd) {
        std::vector<Vec2> pts;
        const double dir = sTerm > sGore ? 1.0 : -1.0, span = std::fabs(sTerm - sGore);
        for (double d = 0; d < span - 2.0; d += 4.0) {
            const double st = sGore + dir * d;
            double r = auxOff + (rBand - auxOff) * smoothStep(d / o.diverge);
            if (rEnd != rBand) r += (rEnd - rBand) * smoothStep((d - (span - o.along - o.shift)) / o.shift);
            pts.push_back(atS(st) + nrmS(st) * (r * side));
        }
        pts.push_back(term);
        if (!off) std::reverse(pts.begin(), pts.end());   // an on-ramp reads street -> band -> gore
        return pts;
    };
    // Every other street the ramp crosses on its way down. Near the gore the ramp is still at
    // deck height and bridges it; partway down it can do neither.
    auto conflicts = [&](const std::vector<Vec2>& sp, std::size_t landing, std::size_t landing2, double zStart, double zEnd) {
        const std::vector<double> ss = stations(sp);
        for (std::size_t ci = 0; ci < streets.size(); ++ci) {
            if (ci == landing || ci == landing2 || streets[ci].xy.size() < 2) continue;
            for (const Vec2& x : crossings(sp, streets[ci].xy)) {
                const double u = project(sp, ss, x).station / ss.back();
                const double z = zStart + (zEnd - zStart) * u;   // the ramp's height there, roughly
                if (z - groundAt(x) < o.clearance - 0.2) return true;
            }
        }
        return false;
    };

    std::vector<double> taken;
    for (const Cand& cd : cands) {
        if (out.built >= o.maxDiamonds) break;
        if (cd.angle < 32.0) { ++out.rejectedOblique; out.squarestRejected = std::max(out.squarestRejected, cd.angle); continue; }
        bool near = false;
        for (double t : taken) if (std::fabs(t - cd.s) < o.spacing) near = true;
        if (near) { ++out.rejectedSpacing; continue; }
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
        struct Plan { const char* name; const std::string* host; bool off; double side, fwd; bool cross; double sTerm; Vec2 term; double sGore; std::size_t street; double beside; };
        Plan ps[4] = {{"a_off", &o.aId, true, aSide, 1.0, crossA, sA, tA, 0, kA, 0}, {"a_on", &o.aId, false, aSide, 1.0, crossA, sA, tA, 0, kA, 0},
                      {"b_off", &o.bId, true, -aSide, -1.0, crossB, sB, tB, 0, kB, 0}, {"b_on", &o.bId, false, -aSide, -1.0, crossB, sB, tB, 0, kB, 0}};
        bool found = true, room = true, clear = true;
        std::vector<std::vector<Vec2>> spines;
        for (Plan& p : ps) {
            const double away = p.off ? -p.fwd : p.fwd;   // off: the gore is upstream; on: downstream
            if (!p.cross) {
                // the touch-down, `mouth` clear of the crossing, on the road beside the freeway —
                // and a road beside it where the shift begins: the same road, though side streets
                // meeting it from outside cut it into several chains
                const double sTouch = cd.s + away * o.mouth;
                double f = 0, f0 = 0; std::size_t k = 0, k0 = 0;
                if (besideAt(sTouch, p.side, f, k) && besideAt(sTouch + away * (o.along + o.shift), p.side, f0, k0)) {
                    const RampStreet& fr = streets[k];
                    p.sTerm = sTouch; p.beside = f; p.street = k;
                    p.term = pointAt(fr.xy, stations(fr.xy), project(fr.xy, stations(fr.xy), atS(sTouch) + nrmS(sTouch) * (f * p.side)).station);
                } else if (terminal(cd, p.side, o.crossMouth, p.term, p.sTerm, p.street)) {
                    p.cross = true;   // no road beside to land on: a four-way nearer the next junction
                } else { found = false; break; }
            }
            // free run, then the part within reach of the landing street: the level approach
            // across the street's half-width, or the shift and the run along the frontage road
            const double arrive = p.cross ? o.landing : o.shift + o.along;
            double L = std::max(runFor(deckZ(p.sTerm) - groundAt(p.term)), o.diverge) + arrive;
            L = std::max(runFor(deckZ(p.sTerm + away * L) - groundAt(p.term)), o.diverge) + arrive;   // the deck's height at the gore, not the street
            p.sGore = p.sTerm + away * L;
            const double lead = p.off ? o.decel + o.taperOff : o.aux + o.taperOn;
            if (std::min(p.sGore, p.sGore + away * lead) < 20.0 || std::max(p.sGore, p.sGore + away * lead) > routeLen - 20.0) room = false;
            spines.push_back(spine(p.sGore, p.sTerm, p.side, p.term, p.off, p.cross ? rBand : p.beside));
            const double zGore = deckZ(p.sGore), zTerm = groundAt(p.term);
            if (conflicts(spines.back(), cd.street, p.street, p.off ? zGore : zTerm, p.off ? zTerm : zGore)) clear = false;
        }
        static const bool why = std::getenv("RT_DIAMOND_WHY") != nullptr;
        if (why) std::fprintf(stderr, "[diamond] %s crossing at s=%.0f (%.0f deg, street %s): side a %s, side b %s -> %s\n", o.idPrefix.c_str(), cd.s, cd.angle,
                              streets[cd.street].id.c_str(), crossA ? "four-way" : "frontage?", crossB ? "four-way" : "frontage?",
                              !found ? "no terminal" : !room ? "no room" : !clear ? "over a street" : "built");
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
            if (p.off) { anchor["decel"] = o.decel; anchor["taper"] = o.taperOff; e["from"] = anchor; e["to"] = sid; }
            else       { anchor["aux"] = o.aux;     anchor["taper"] = o.taperOn;  e["to"] = anchor; e["from"] = sid; }
            e["path"] = {{"type", "polyline"}, {"points", pointList(spines[static_cast<std::size_t>(k)])}};
            out.ramps.push_back(e);
        }
        taken.push_back(cd.s);
        ++out.built;
    }
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
