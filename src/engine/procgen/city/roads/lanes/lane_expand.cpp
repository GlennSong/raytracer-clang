#include "engine/procgen/city/roads/lanes/lane_expand.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "curve.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>

namespace engine {
namespace roads::lanes {

// (declared in lane_expand.h: the walks are laid by it too, walk_lanes.cpp)
Lane ribbonLane(const std::shared_ptr<LaneCurves>& curves, const std::string& id, int parent, const std::string& kind,
                const std::string& cls, double w, int dir,
                const std::vector<Vec2>& spine, const std::vector<Vec2>& nrm, const std::vector<double>& off) {
    Lane l; l.id = id; l.edge = parent >= 0 ? static_cast<uint32_t>(parent) : kNoIndex; l.kind = kind; l.cls = cls; l.width = w; l.dir = dir;
    std::vector<Vec2> xy;
    xy.reserve(spine.size()); l.left.reserve(spine.size()); l.right.reserve(spine.size());
    for (size_t i = 0; i < spine.size(); ++i) {
        xy.push_back(spine[i] + nrm[i] * off[i]);
        l.left.push_back(spine[i] + nrm[i] * (off[i] + w / 2));
        l.right.push_back(spine[i] + nrm[i] * (off[i] - w / 2));
    }
    const std::vector<double> s = stations(xy);
    l.curve = Range{static_cast<uint32_t>(curves->points.size()), static_cast<uint32_t>(xy.size())};
    curves->points.insert(curves->points.end(), xy.begin(), xy.end());
    curves->along.insert(curves->along.end(), s.begin(), s.end());
    l.curves = curves;
    return l;
}

LaneSlot anchorSlot(const RoadLabGraph& g, const EdgeSpec& host, const std::string& side) {
    std::vector<LaneSlot> slots = laneSlots(g.cls(host), host.lanes);
    std::string want = side == "right" ? "f" + std::to_string(host.lanes.fwd - 1) : side == "back-right" ? "b" + std::to_string(host.lanes.back - 1) : "f0";
    for (const LaneSlot& s : slots) if (s.name == want) return s;
    throw std::runtime_error("ramp anchor: host " + host.id + " has no lane on side " + side);
}

double anchorStation(const EdgeSpec& host, const RampAnchor& a) {
    if (a.hasStation) return a.s;
    if (a.axis == 'p') return project(host.xy, host.s, Vec2(a.px, a.py)).station;
    size_t best = 0; double bd = 1e300;
    for (size_t i = 0; i < host.xy.size(); ++i) { double d = std::fabs((a.axis == 'x' ? host.xy[i].x : host.xy[i].y) - a.coord); if (d < bd) { bd = d; best = i; } }
    return host.s[best];
}

namespace {

// Points along the host spine at stations ss, displaced by off.
std::vector<Vec2> hostPath(const EdgeSpec& host, const std::vector<double>& ss, const std::vector<double>& off) {
    std::vector<Vec2> out; out.reserve(ss.size());
    for (size_t k = 0; k < ss.size(); ++k) out.push_back(pointAt(host.xy, host.s, ss[k]) + perp(tangentAtStation(host.xy, host.s, ss[k])) * off[k]);
    return out;
}

}  // namespace

// THE ONE LIST OF DISTANCES A RAMP'S END IS LAID AT (issue #292): every whole metre from 0, then the length itself where
// it is not whole metres (a part-metre under kSamePoint is no step of its own: the last whole metre is moved onto the
// length). It ends exactly at `length`, so an entry's lane meets its host at the stated station, not up to a metre early.
std::vector<double> rampEndDistances(double length) {
    std::vector<double> out;
    const int whole = static_cast<int>(length);
    out.reserve(static_cast<size_t>(whole) + 2);
    for (int k = 0; k <= whole; ++k) out.push_back(static_cast<double>(k));
    if (approxEqual(out.back(), length, kSamePoint)) out.back() = length;
    else out.push_back(length);
    return out;
}

namespace {

void composeRamp(RoadLabGraph& g, EdgeSpec& e) {
    nlohmann::json path = e.path; std::string kind = path.value("type", std::string("polyline"));
    nlohmann::json pts = path.value("points", nlohmann::json::array());
    std::vector<Vec2> pre, post; e.anchorIdx[0] = e.anchorIdx[1] = -1; e.anchorLanes.clear();
    e.anchorMeets[0] = e.anchorMeets[1] = std::numeric_limits<double>::quiet_NaN();
    // Both ends are laid along the host over the whole stated length (auxiliary or deceleration lane and taper), a sample
    // each metre and one at the length itself (rampEndDistances). The station laid to is recorded (anchorMeets) and the
    // road model copies it, so the model and the drawing say the same place.
    if (e.to.set) {
        const EdgeSpec& host = *g.find(e.to.edge); LaneSlot slot = anchorSlot(g, host, e.to.side);
        double sg = anchorStation(host,e.to), aux = e.to.aux, tap = e.to.taper, appr = e.to.approach;
        double sgn = slot.dir > 0 ? 1.0 : -1.0, outward = (e.to.side == "right") ? -1.0 : +1.0;   // the auxiliary lane lies outside the outer lane
        std::vector<double> ss, off;
        for (double d : rampEndDistances(aux + tap)) { ss.push_back(sg + sgn * d); off.push_back(slot.offset + outward * host.lanes.w * (1 - smoothstep01((d - aux) / tap))); }
        post = hostPath(host, ss, off); Vec2 G = post.front(), T = tangentAtStation(host.xy, host.s, sg) * sgn;
        e.anchorLanes["to"] = host.id + "." + slot.name;
        e.anchorMeets[1] = ss.back();   // where its line meets the host lane's
        if (kind == "bezier" && pts.size() == 4) { pts[3] = {G.x, G.y}; Vec2 c = G - T * appr; pts[2] = {c.x, c.y}; }
        else if (!pts.empty()) pts[pts.size() - 1] = {G.x, G.y};
    }
    if (e.from.set) {
        const EdgeSpec& host = *g.find(e.from.edge); LaneSlot slot = anchorSlot(g, host, e.from.side);
        double sg = anchorStation(host,e.from), dec = e.from.decel, tap = e.from.taper, appr = e.from.approach;
        double sgn = slot.dir > 0 ? 1.0 : -1.0, outward = (e.from.side == "right") ? -1.0 : +1.0;
        std::vector<double> ss, off;
        for (double d : rampEndDistances(dec + tap)) { ss.push_back(sg - sgn * (dec + tap) + sgn * d); off.push_back(slot.offset + outward * host.lanes.w * smoothstep01(d / tap)); }
        pre = hostPath(host, ss, off); Vec2 G = pre.back(), T = tangentAtStation(host.xy, host.s, sg) * sgn;
        e.anchorLanes["from"] = host.id + "." + slot.name;
        e.anchorMeets[0] = ss.front();   // where its line leaves the host lane's
        if (kind == "bezier" && pts.size() == 4) { pts[0] = {G.x, G.y}; Vec2 c = G + T * appr; pts[1] = {c.x, c.y}; }
        else if (!pts.empty()) pts[0] = {G.x, G.y};
    }
    path["points"] = pts; path.erase("reverse");
    std::vector<Vec2> mid = pts.empty() ? std::vector<Vec2>{} : resolvePath(path, g, g.rules.step);
    std::vector<Vec2> xy;
    auto append = [&](const std::vector<Vec2>& part) { for (size_t i = 0; i < part.size(); ++i) { if (i == 0 && !xy.empty()) continue; xy.push_back(part[i]); } };
    append(pre); append(mid); append(post);
    xy = resample(xy, g.rules.step); e.xy = xy; e.s = stations(xy);
    if (!pre.empty()) { double sp = stations(pre).back(); e.anchorIdx[0] = static_cast<int>(std::lower_bound(e.s.begin(), e.s.end(), sp) - e.s.begin()); }
    if (!post.empty()) { double sp = stations(post).back(); e.anchorIdx[1] = static_cast<int>(std::lower_bound(e.s.begin(), e.s.end(), e.s.back() - sp) - e.s.begin()); }
    for (int& i : e.anchorIdx) if (i >= static_cast<int>(e.s.size())) i = static_cast<int>(e.s.size()) - 1;
}

void roadLanes(const RoadLabGraph& g, int ei, LaneSet& out, const std::shared_ptr<LaneCurves>& curves) {
    const EdgeSpec& e = g.edges[static_cast<size_t>(ei)]; const RoadClassSpec& c = g.cls(e);
    std::vector<Vec2> tan, nrm; frames(e.xy, tan, nrm); double w = e.lanes.w;
    std::vector<LaneSlot> slots = laneSlots(c, e.lanes);
    std::map<std::string, int> laneOfSlot;   // a slot's name -> its lane's index (what a pocket is swept out of)
    for (const LaneSlot& sl : slots) {
        laneOfSlot[sl.name] = static_cast<int>(out.lanes.size());
        std::vector<double> off(e.xy.size(), sl.offset);
        // (a footway's lane is a walk: its kind is the path users' word for one, ADR-0465; laid and drawn as before)
        out.lanes.push_back(ribbonLane(curves, e.id + "." + sl.name, ei, c.footway ? "footway" : "through", e.cls, w, sl.dir, e.xy, nrm, off));
        out.lanes.back().slot = std::atoi(sl.name.c_str() + 1);   // f0 / b0: by the centre line (laneSlots' names)
        out.lanes.back().spineFrom = e.s.front();
        out.lanes.back().spineTo = e.s.back();
    }
    for (const PocketSpec& pk : e.pockets) {
        int d = pk.dir; std::vector<LaneSlot> cand; for (const LaneSlot& sl : slots) if (sl.dir == d) cand.push_back(sl);
        if (cand.empty()) continue;
        // inner = nearest the median (offset * d largest), outer = the kerb side
        const LaneSlot* inner = &cand[0]; const LaneSlot* outer = &cand[0];
        for (const LaneSlot& sl : cand) { if (sl.offset * d > inner->offset * d) inner = &sl; if (sl.offset * d < outer->offset * d) outer = &sl; }
        double baseOff = (pk.side == "right" ? outer : inner)->offset; double sgn = pk.side == "right" ? -d : d;
        double ti = pk.taper, to = pk.taperOut;
        std::vector<Vec2> sp, nn; std::vector<double> off;
        double first = -1, last = -1;
        for (size_t i = 0; i < e.xy.size(); ++i) {
            double st = e.s[i]; if (st < pk.s0 || st > pk.s1) continue;
            double birth = d > 0 ? smoothstep01((st - pk.s0) / ti) : smoothstep01((pk.s1 - st) / ti);
            double death = to <= 0 ? 1.0 : (d > 0 ? smoothstep01((pk.s1 - st) / to) : smoothstep01((st - pk.s0) / to));
            sp.push_back(e.xy[i]); nn.push_back(nrm[i]); off.push_back(baseOff + sgn * w * std::min(birth, death));
            if (first < 0) first = st;
            last = st;
        }
        if (sp.size() >= 2) {
            out.lanes.push_back(ribbonLane(curves, e.id + "." + pk.id, ei, pk.kind, e.cls, w, d, sp, nn, off));
            out.lanes.back().slot = pk.side == "right" ? static_cast<int>(cand.size()) : -1;   // beyond the kerb lane, or inside lane 0
            out.lanes.back().spineFrom = first;
            out.lanes.back().spineTo = last;
            // swept out of the lane on its side (its offset at birth is that lane's), and back into it where it tapers out
            const int host = laneOfSlot.at((pk.side == "right" ? outer : inner)->name);
            out.lanes.back().sweptFrom = host;
            if (to > 0) out.lanes.back().diesInto = host;
        }
    }
}

void rampLanes(const RoadLabGraph& g, int ei, LaneSet& out, const std::shared_ptr<LaneCurves>& curves) {
    const EdgeSpec& e = g.edges[static_cast<size_t>(ei)]; std::vector<Vec2> tan, nrm; frames(e.xy, tan, nrm); double w = e.lanes.w;
    std::vector<double> zero(e.xy.size(), 0.0);
    out.lanes.push_back(ribbonLane(curves, e.id + ".f0", ei, "ramp", e.cls, w, +1, e.xy, nrm, zero));
    out.lanes.back().slot = 0;
    out.lanes.back().spineFrom = e.s.front();
    out.lanes.back().spineTo = e.s.back();
    if (e.lanes.fwd >= 2) {                                        // the second lane dies into the first: the dovetail
        double dv = e.lanes.dovetail, tap = 20 * w; std::vector<Vec2> sp, nn; std::vector<double> off;
        double last = 0;
        for (size_t i = 0; i < e.xy.size(); ++i) { if (e.s[i] > dv + tap) break; sp.push_back(e.xy[i]); nn.push_back(nrm[i]); off.push_back(-w * (1 - smoothstep01((e.s[i] - dv) / tap))); last = e.s[i]; }
        if (sp.size() >= 2) {
            out.lanes.push_back(ribbonLane(curves, e.id + ".f1", ei, "ramp", e.cls, w, +1, sp, nn, off));
            out.lanes.back().slot = 1;
            out.lanes.back().spineFrom = e.s.front();
            out.lanes.back().spineTo = last;
            out.lanes.back().diesInto = static_cast<int>(out.lanes.size()) - 2;   // into the ramp's first lane
        }
    }
}

std::vector<Vec2> hermite(const Vec2& p0, const Vec2& t0, const Vec2& p1, const Vec2& t1, int n = 80) {
    std::vector<Vec2> out; out.reserve(n); double L = distance(p0, p1);
    for (int i = 0; i < n; ++i) {
        out.push_back(engine::hermite(p0, t0 * L, p1, t1 * L, static_cast<double>(i) / (n - 1)));
    }
    return out;
}

}  // namespace

double LaneSet::rank(int lane, const RoadLabGraph& g) const {
    const Lane& l = lanes[static_cast<size_t>(lane)];
    if (l.isConnector()) return -0.5;
    return g.cls(g.edges[static_cast<size_t>(l.road())]).rank;
}

std::string LaneSet::roadOf(int lane, const RoadLabGraph& g) const {
    const Lane& l = lanes[static_cast<size_t>(lane)]; return l.road() >= 0 ? g.edges[static_cast<size_t>(l.road())].id : l.id;
}

LaneSet expand(RoadLabGraph& g) {
    for (EdgeSpec& e : g.edges) if (e.isRamp()) composeRamp(g, e);
    LaneSet L;
    const std::shared_ptr<LaneCurves> curves = std::make_shared<LaneCurves>();   // every lane's centre line, held once
    L.curves = curves;
    for (size_t i = 0; i < g.edges.size(); ++i) {
        const EdgeSpec& e = g.edges[i]; if (e.laneCount() == 0) continue;
        if (e.isRamp()) rampLanes(g, static_cast<int>(i), L, curves); else roadLanes(g, static_cast<int>(i), L, curves);
    }
    for (size_t i = 0; i < L.lanes.size(); ++i) L.byId[L.lanes[i].id] = static_cast<int>(i);
    for (const ConnectorSpec& c : g.connectors) {
        int a = L.find(c.from), b = L.find(c.to);
        if (a < 0 || b < 0) throw std::runtime_error("connector references unknown lane " + (a < 0 ? c.from : c.to));
        const Lane& A = L.lanes[static_cast<size_t>(a)]; const Lane& B = L.lanes[static_cast<size_t>(b)];
        Vec2 p0 = A.xy().back(), t0 = tangentAt(A.xy(), A.xy().size() - 1);
        Vec2 p1 = pointAt(B.xy(), B.s(), c.toS), t1 = tangentAtStation(B.xy(), B.s(), c.toS) * (B.dir > 0 ? 1.0 : -1.0);
        std::vector<Vec2> xy = hermite(p0, t0, p1, t1); std::vector<Vec2> tan, nrm; frames(xy, tan, nrm); std::vector<double> zero(xy.size(), 0.0);
        Lane ln = ribbonLane(curves, A.id + ">" + B.id, -1, "connector", A.cls, A.width, +1, xy, nrm, zero); ln.srcLane = a; ln.dstLane = b;
        L.byId[ln.id] = static_cast<int>(L.lanes.size()); L.lanes.push_back(std::move(ln));
    }
    return L;
}

double laneHeightAt(const Lane& l, const RoadLabGraph& g, const Vec2& p) {
    if (l.isConnector()) { Projection pr = project(l.xy(), l.s(), p); double u = l.s().back() > 1e-9 ? pr.station / l.s().back() : 0.0; return (1 - u) * l.z0 + u * l.z1; }
    const EdgeSpec& e = g.edges[static_cast<size_t>(l.road())]; Projection pr = project(e.xy, e.s, p); return interp(e.s, e.z, pr.station);
}

std::vector<std::pair<int, int>> adjacency(const LaneSet& L) {
    std::vector<std::pair<int, int>> out; std::vector<std::array<double, 4>> bx(L.lanes.size());
    for (size_t i = 0; i < L.lanes.size(); ++i) { auto& b = bx[i]; b = {1e300, -1e300, 1e300, -1e300}; for (const Vec2& p : L.lanes[i].xy()) { b[0] = std::min(b[0], p.x); b[1] = std::max(b[1], p.x); b[2] = std::min(b[2], p.y); b[3] = std::max(b[3], p.y); } }
    for (size_t i = 0; i < L.lanes.size(); ++i) for (size_t j = i + 1; j < L.lanes.size(); ++j) {
        const Lane& a = L.lanes[i]; const Lane& b = L.lanes[j]; double w = (a.width + b.width) / 2; int close = 0; double dots = 0;
        if (bx[i][0] > bx[j][1] + 2 * w || bx[j][0] > bx[i][1] + 2 * w || bx[i][2] > bx[j][3] + 2 * w || bx[j][2] > bx[i][3] + 2 * w) continue;
        for (int k = 0; k < 25; ++k) {
            double f = 0.02 + 0.96 * k / 24; Vec2 p = pointAt(a.xy(), a.s(), f * a.s().back()); Projection pr = project(b.xy(), b.s(), p);
            if (std::fabs(pr.distance - w) < 0.25 * w) { ++close; dots += dot(tangentAtStation(a.xy(), a.s(), f * a.s().back()) * static_cast<double>(a.dir), tangentAtStation(b.xy(), b.s(), pr.station) * static_cast<double>(b.dir)); }
        }
        if (close * a.s().back() / 25 < 20 || close == 0) continue;
        if (dots / close > 0.7) out.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
