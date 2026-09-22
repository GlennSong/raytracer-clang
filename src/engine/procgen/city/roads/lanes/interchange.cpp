#include "engine/procgen/city/roads/lanes/interchange.h"

#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"

#include <algorithm>
#include <cmath>

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
    const double rBand = o.edgeReach + 6.7;   // the band centre, clear of the shoulder
    const double tOut = 45.0;                 // the T, past the freeway's edge
    std::vector<double> routeZ;
    if (o.ground) { HeightField hf = o.ground; routeZ = profileAlong(route, hf, o.window, o.gMax); }
    auto clampS = [&](double st) { return std::max(0.0, std::min(routeLen, st)); };
    auto deckZ = [&](double st) { return routeZ.empty() ? 0.0 : interp(cs, routeZ, clampS(st)) + o.clearance; };
    auto atS = [&](double st) { return pointAt(route, cs, clampS(st)); };
    auto nrmS = [&](double st) { return perp(tangentAtStation(route, cs, clampS(st))); };

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

    // The terminal: the nearest STREET point to where the ramp wants to end — out past the
    // freeway's edge on the side asked for, beside the crossing. It need not be the crossing
    // street: a diamond lands on whatever road is there, which is what a city looks like.
    auto terminal = [&](const Cand& cd, double side, Vec2& term, Vec2& dir, std::size_t& street) {
        const Vec2 ideal = atS(cd.s) + nrmS(cd.s) * ((o.edgeReach + tOut) * side);
        double best = 1e300;
        for (std::size_t ci = 0; ci < streets.size(); ++ci) {
            const RampStreet& c = streets[ci];
            if (c.xy.size() < 2) continue;
            const std::vector<double> ss = stations(c.xy);
            const Projection pr = project(c.xy, ss, ideal);
            if (pr.distance > 90.0) continue;                        // not beside this crossing
            const Vec2 q = pointAt(c.xy, ss, pr.station);
            if (dot(q - atS(cd.s), nrmS(cd.s)) * side < o.edgeReach + 15.0) continue;   // still under the freeway
            if (pr.distance < best) {
                best = pr.distance; term = q; street = ci;
                dir = tangentAt(c.xy, pr.segment);
                // point it AWAY from the freeway, so the ramp finishes running along the
                // street rather than crossing back under the deck
                if (dot(dir, nrmS(cd.s)) * side < 0) dir = dir * -1.0;
            }
        }
        return best < 1e299;
    };

    std::vector<double> taken;
    for (const Cand& cd : cands) {
        if (out.built >= o.maxDiamonds) break;
        if (cd.angle < 32.0) { ++out.rejectedOblique; out.squarestRejected = std::max(out.squarestRejected, cd.angle); continue; }
        bool near = false;
        for (double t : taken) if (std::fabs(t - cd.s) < o.spacing) near = true;
        if (near) { ++out.rejectedSpacing; continue; }
        Vec2 tLeft, tRight, dLeft, dRight;
        std::size_t cLeft = 0, cRight = 0;
        if (!terminal(cd, +1.0, tLeft, dLeft, cLeft) || !terminal(cd, -1.0, tRight, dRight, cRight)) { ++out.rejectedTerminal; continue; }
        // The run each ramp needs for its climb, at 6% with the approach on top, measured to
        // the TERMINAL's ground.
        const double gTerm = o.ground ? std::min(o.ground(tLeft.x, tLeft.y), o.ground(tRight.x, tRight.y)) : 0.0;
        const double climb = std::fabs(deckZ(cd.s) - gTerm);
        const double L = std::max(170.0, climb / 0.06) + o.approach;
        if (cd.s - L < 20.0 || cd.s + L > routeLen - 20.0) { ++out.rejectedRoom; continue; }   // no room

        // gore -> band -> the T on the street. `side` picks the carriageway: +1 is the one
        // offset along +normal (a), -1 its opposite (b).
        auto band = [&](double sFrom, double sTo, double side, const Vec2& t, const Vec2& tdir) {
            std::vector<Vec2> pts;
            pts.push_back(atS(sFrom) + nrmS(sFrom) * (o.carriage * side));
            const int n = 12;
            for (int i = 0; i <= n; ++i) {
                const double st = sFrom + (sTo - sFrom) * (static_cast<double>(i) / n);
                pts.push_back(atS(st) + nrmS(st) * (rBand * side));
            }
            // approach the street from 40 m back and finish ALONG it: a ramp driven into a
            // centreline at right angles is a sliver in the pavement union, not a junction.
            pts.push_back(t - tdir * 40.0);
            pts.push_back(t);
            return resample(pts, 4.0);
        };
        auto ramp = [&](const std::string& id, const std::string& arc, bool off, const Vec2& gore,
                        const std::vector<Vec2>& spine, std::size_t street) {
            nlohmann::json e; e["id"] = id; e["class"] = "ramp";
            nlohmann::json anchor = {{"edge", arc}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"approach", o.approach}};
            const std::string sid = streets[street].id;
            if (off) { anchor["decel"] = o.decel; anchor["taper"] = o.taperOff; e["from"] = anchor; e["to"] = sid; }
            else     { anchor["aux"] = o.aux;     anchor["taper"] = o.taperOn;  e["to"] = anchor; e["from"] = sid; }
            e["path"] = {{"type", "polyline"}, {"points", pointList(spine)}};
            return e;
        };
        const std::string pre = o.idPrefix + "_" + std::to_string(out.built);
        // a travels with increasing station: it exits upstream of the crossing and merges
        // downstream of it. b is the mirror.
        const Vec2 gAoff = atS(cd.s - L) + nrmS(cd.s - L) * o.carriage;
        const Vec2 gAon  = atS(cd.s + L) + nrmS(cd.s + L) * o.carriage;
        const Vec2 gBoff = atS(cd.s + L) - nrmS(cd.s + L) * o.carriage;
        const Vec2 gBon  = atS(cd.s - L) - nrmS(cd.s - L) * o.carriage;
        out.ramps.push_back(ramp(pre + "_a_off", o.aId, true,  gAoff, band(cd.s - L, cd.s, +1.0, tLeft, dLeft), cLeft));
        out.ramps.push_back(ramp(pre + "_a_on",  o.aId, false, gAon,  band(cd.s + L, cd.s, +1.0, tLeft, dLeft), cLeft));
        out.ramps.push_back(ramp(pre + "_b_off", o.bId, true,  gBoff, band(cd.s + L, cd.s, -1.0, tRight, dRight), cRight));
        out.ramps.push_back(ramp(pre + "_b_on",  o.bId, false, gBon,  band(cd.s - L, cd.s, -1.0, tRight, dRight), cRight));
        taken.push_back(cd.s);
        ++out.built;
    }
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
