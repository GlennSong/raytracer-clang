#include "engine/procgen/lanelab/lane_expand.h"
#include "engine/procgen/lanelab/polyline_ops.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace engine {
namespace lanelab {

namespace {

double smooth01(double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); }

Lane ribbonLane(const std::string& id, int parent, const std::string& kind, const std::string& cls, double w, int dir,
                const std::vector<Vec2>& spine, const std::vector<Vec2>& nrm, const std::vector<double>& off) {
    Lane l; l.id = id; l.parent = parent; l.kind = kind; l.cls = cls; l.w = w; l.dir = dir;
    l.xy.reserve(spine.size()); l.left.reserve(spine.size()); l.right.reserve(spine.size());
    for (size_t i = 0; i < spine.size(); ++i) {
        l.xy.push_back(spine[i] + nrm[i] * off[i]);
        l.left.push_back(spine[i] + nrm[i] * (off[i] + w / 2));
        l.right.push_back(spine[i] + nrm[i] * (off[i] - w / 2));
    }
    l.s = stations(l.xy); return l;
}

// The slot a ramp attaches to: 'right' = forward outer lane, 'back-right' = backward outer, 'left' = forward inner.
LaneSlot outerSlot(const RoadLabGraph& g, const EdgeSpec& host, const std::string& side) {
    std::vector<LaneSlot> slots = laneSlots(g.cls(host), host.lanes);
    std::string want = side == "right" ? "f" + std::to_string(host.lanes.fwd - 1) : side == "back-right" ? "b" + std::to_string(host.lanes.back - 1) : "f0";
    for (const LaneSlot& s : slots) if (s.name == want) return s;
    throw std::runtime_error("ramp anchor: host " + host.id + " has no lane on side " + side);
}

double hostStation(const EdgeSpec& host, const RampAnchor& a) {
    if (a.hasStation) return a.s;
    if (a.axis == 'p') return project(host.xy, host.s, Vec2(a.px, a.py)).station;
    size_t best = 0; double bd = 1e300;
    for (size_t i = 0; i < host.xy.size(); ++i) { double d = std::fabs((a.axis == 'x' ? host.xy[i].x : host.xy[i].y) - a.coord); if (d < bd) { bd = d; best = i; } }
    return host.s[best];
}

// Points along the host spine at stations ss, displaced by off.
std::vector<Vec2> hostPath(const EdgeSpec& host, const std::vector<double>& ss, const std::vector<double>& off) {
    std::vector<Vec2> out; out.reserve(ss.size());
    for (size_t k = 0; k < ss.size(); ++k) out.push_back(pointAt(host.xy, host.s, ss[k]) + perp(tangentAtStation(host.xy, host.s, ss[k])) * off[k]);
    return out;
}

void composeRamp(RoadLabGraph& g, EdgeSpec& e) {
    nlohmann::json path = e.path; std::string kind = path.value("type", std::string("polyline"));
    nlohmann::json pts = path.value("points", nlohmann::json::array());
    std::vector<Vec2> pre, post; e.anchorIdx[0] = e.anchorIdx[1] = -1; e.anchorLanes.clear();
    if (e.to.set) {
        const EdgeSpec& host = *g.find(e.to.edge); LaneSlot slot = outerSlot(g, host, e.to.side);
        double sg = hostStation(host, e.to), aux = e.to.aux, tap = e.to.taper > 0 ? e.to.taper : 30 * host.lanes.w, appr = e.to.approach;
        double sgn = slot.dir > 0 ? 1.0 : -1.0, outward = (e.to.side == "right") ? -1.0 : +1.0;   // the auxiliary lane lies outside the outer lane
        std::vector<double> ss, off; int n = static_cast<int>(aux + tap) + 1;
        for (int k = 0; k < n; ++k) { double d = std::min<double>(k, aux + tap); ss.push_back(sg + sgn * d); off.push_back(slot.offset + outward * host.lanes.w * (1 - smooth01((d - aux) / tap))); }
        post = hostPath(host, ss, off); Vec2 G = post.front(), T = tangentAtStation(host.xy, host.s, sg) * sgn;
        e.anchorLanes["to"] = host.id + "." + slot.name;
        if (kind == "bezier" && pts.size() == 4) { pts[3] = {G.x, G.y}; Vec2 c = G - T * appr; pts[2] = {c.x, c.y}; }
        else if (!pts.empty()) pts[pts.size() - 1] = {G.x, G.y};
    }
    if (e.from.set) {
        const EdgeSpec& host = *g.find(e.from.edge); LaneSlot slot = outerSlot(g, host, e.from.side);
        double sg = hostStation(host, e.from), dec = e.from.decel, tap = e.from.taper > 0 ? e.from.taper : 30 * host.lanes.w, appr = e.from.approach;
        double sgn = slot.dir > 0 ? 1.0 : -1.0, outward = (e.from.side == "right") ? -1.0 : +1.0;
        std::vector<double> ss, off; int n = static_cast<int>(dec + tap) + 1;
        for (int k = 0; k < n; ++k) { double d = std::min<double>(k, dec + tap); ss.push_back(sg - sgn * (dec + tap) + sgn * d); off.push_back(slot.offset + outward * host.lanes.w * smooth01(d / tap)); }
        pre = hostPath(host, ss, off); Vec2 G = pre.back(), T = tangentAtStation(host.xy, host.s, sg) * sgn;
        e.anchorLanes["from"] = host.id + "." + slot.name;
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

void roadLanes(const RoadLabGraph& g, int ei, LaneSet& out) {
    const EdgeSpec& e = g.edges[static_cast<size_t>(ei)]; const RoadClassSpec& c = g.cls(e);
    std::vector<Vec2> tan, nrm; frames(e.xy, tan, nrm); double w = e.lanes.w;
    std::vector<LaneSlot> slots = laneSlots(c, e.lanes);
    for (const LaneSlot& sl : slots) {
        std::vector<double> off(e.xy.size(), sl.offset);
        out.lanes.push_back(ribbonLane(e.id + "." + sl.name, ei, "through", e.cls, w, sl.dir, e.xy, nrm, off));
    }
    for (const PocketSpec& pk : e.pockets) {
        int d = pk.dir; std::vector<LaneSlot> cand; for (const LaneSlot& sl : slots) if (sl.dir == d) cand.push_back(sl);
        if (cand.empty()) continue;
        // inner = nearest the median (offset * d largest), outer = the kerb side
        const LaneSlot* inner = &cand[0]; const LaneSlot* outer = &cand[0];
        for (const LaneSlot& sl : cand) { if (sl.offset * d > inner->offset * d) inner = &sl; if (sl.offset * d < outer->offset * d) outer = &sl; }
        double baseOff = (pk.side == "right" ? outer : inner)->offset; double sgn = pk.side == "right" ? -d : d;
        double ti = pk.taper > 0 ? pk.taper : 10 * w, to = pk.taperOut;
        std::vector<Vec2> sp, nn; std::vector<double> off;
        for (size_t i = 0; i < e.xy.size(); ++i) {
            double st = e.s[i]; if (st < pk.s0 || st > pk.s1) continue;
            double birth = d > 0 ? smooth01((st - pk.s0) / ti) : smooth01((pk.s1 - st) / ti);
            double death = to <= 0 ? 1.0 : (d > 0 ? smooth01((pk.s1 - st) / to) : smooth01((st - pk.s0) / to));
            sp.push_back(e.xy[i]); nn.push_back(nrm[i]); off.push_back(baseOff + sgn * w * std::min(birth, death));
        }
        if (sp.size() >= 2) out.lanes.push_back(ribbonLane(e.id + "." + pk.id, ei, pk.kind, e.cls, w, d, sp, nn, off));
    }
}

void rampLanes(const RoadLabGraph& g, int ei, LaneSet& out) {
    const EdgeSpec& e = g.edges[static_cast<size_t>(ei)]; std::vector<Vec2> tan, nrm; frames(e.xy, tan, nrm); double w = e.lanes.w;
    std::vector<double> zero(e.xy.size(), 0.0);
    out.lanes.push_back(ribbonLane(e.id + ".f0", ei, "ramp", e.cls, w, +1, e.xy, nrm, zero));
    if (e.lanes.fwd >= 2) {                                        // the second lane dies into the first: the dovetail
        double dv = e.lanes.dovetail, tap = 20 * w; std::vector<Vec2> sp, nn; std::vector<double> off;
        for (size_t i = 0; i < e.xy.size(); ++i) { if (e.s[i] > dv + tap) break; sp.push_back(e.xy[i]); nn.push_back(nrm[i]); off.push_back(-w * (1 - smooth01((e.s[i] - dv) / tap))); }
        if (sp.size() >= 2) out.lanes.push_back(ribbonLane(e.id + ".f1", ei, "ramp", e.cls, w, +1, sp, nn, off));
    }
}

std::vector<Vec2> hermite(const Vec2& p0, const Vec2& t0, const Vec2& p1, const Vec2& t1, int n = 80) {
    std::vector<Vec2> out; out.reserve(n); double L = distance(p0, p1);
    for (int i = 0; i < n; ++i) {
        double u = static_cast<double>(i) / (n - 1), u2 = u * u, u3 = u2 * u;
        out.push_back(p0 * (2*u3 - 3*u2 + 1) + t0 * ((u3 - 2*u2 + u) * L) + p1 * (-2*u3 + 3*u2) + t1 * ((u3 - u2) * L));
    }
    return out;
}

}  // namespace

double LaneSet::rank(int lane, const RoadLabGraph& g) const {
    const Lane& l = lanes[static_cast<size_t>(lane)];
    if (l.isConnector()) return -0.5;
    return g.cls(g.edges[static_cast<size_t>(l.parent)]).rank;
}

std::string LaneSet::roadOf(int lane, const RoadLabGraph& g) const {
    const Lane& l = lanes[static_cast<size_t>(lane)]; return l.parent >= 0 ? g.edges[static_cast<size_t>(l.parent)].id : l.id;
}

LaneSet expand(RoadLabGraph& g) {
    for (EdgeSpec& e : g.edges) if (e.isRamp()) composeRamp(g, e);
    LaneSet L;
    for (size_t i = 0; i < g.edges.size(); ++i) {
        const EdgeSpec& e = g.edges[i]; if (e.laneCount() == 0) continue;
        if (e.isRamp()) rampLanes(g, static_cast<int>(i), L); else roadLanes(g, static_cast<int>(i), L);
    }
    for (size_t i = 0; i < L.lanes.size(); ++i) L.byId[L.lanes[i].id] = static_cast<int>(i);
    for (const ConnectorSpec& c : g.connectors) {
        int a = L.find(c.from), b = L.find(c.to);
        if (a < 0 || b < 0) throw std::runtime_error("connector references unknown lane " + (a < 0 ? c.from : c.to));
        const Lane& A = L.lanes[static_cast<size_t>(a)]; const Lane& B = L.lanes[static_cast<size_t>(b)];
        Vec2 p0 = A.xy.back(), t0 = tangentAt(A.xy, A.xy.size() - 1);
        Vec2 p1 = pointAt(B.xy, B.s, c.toS), t1 = tangentAtStation(B.xy, B.s, c.toS) * (B.dir > 0 ? 1.0 : -1.0);
        std::vector<Vec2> xy = hermite(p0, t0, p1, t1); std::vector<Vec2> tan, nrm; frames(xy, tan, nrm); std::vector<double> zero(xy.size(), 0.0);
        Lane ln = ribbonLane(A.id + ">" + B.id, -1, "connector", A.cls, A.w, +1, xy, nrm, zero); ln.srcLane = a; ln.dstLane = b;
        L.byId[ln.id] = static_cast<int>(L.lanes.size()); L.lanes.push_back(std::move(ln));
    }
    return L;
}

double laneHeightAt(const Lane& l, const RoadLabGraph& g, const Vec2& p) {
    if (l.isConnector()) { Projection pr = project(l.xy, l.s, p); double u = l.s.back() > 1e-9 ? pr.station / l.s.back() : 0.0; return (1 - u) * l.z0 + u * l.z1; }
    const EdgeSpec& e = g.edges[static_cast<size_t>(l.parent)]; Projection pr = project(e.xy, e.s, p); return interp(e.s, e.z, pr.station);
}

std::vector<std::pair<int, int>> adjacency(const LaneSet& L) {
    std::vector<std::pair<int, int>> out; std::vector<std::array<double, 4>> bx(L.lanes.size());
    for (size_t i = 0; i < L.lanes.size(); ++i) { auto& b = bx[i]; b = {1e300, -1e300, 1e300, -1e300}; for (const Vec2& p : L.lanes[i].xy) { b[0] = std::min(b[0], p.x); b[1] = std::max(b[1], p.x); b[2] = std::min(b[2], p.y); b[3] = std::max(b[3], p.y); } }
    for (size_t i = 0; i < L.lanes.size(); ++i) for (size_t j = i + 1; j < L.lanes.size(); ++j) {
        const Lane& a = L.lanes[i]; const Lane& b = L.lanes[j]; double w = (a.w + b.w) / 2; int close = 0; double dots = 0;
        if (bx[i][0] > bx[j][1] + 2 * w || bx[j][0] > bx[i][1] + 2 * w || bx[i][2] > bx[j][3] + 2 * w || bx[j][2] > bx[i][3] + 2 * w) continue;
        for (int k = 0; k < 25; ++k) {
            double f = 0.02 + 0.96 * k / 24; Vec2 p = pointAt(a.xy, a.s, f * a.s.back()); Projection pr = project(b.xy, b.s, p);
            if (std::fabs(pr.distance - w) < 0.25 * w) { ++close; dots += dot(tangentAtStation(a.xy, a.s, f * a.s.back()) * static_cast<double>(a.dir), tangentAtStation(b.xy, b.s, pr.station) * static_cast<double>(b.dir)); }
        }
        if (close * a.s.back() / 25 < 20 || close == 0) continue;
        if (dots / close > 0.7) out.emplace_back(static_cast<int>(i), static_cast<int>(j));
    }
    return out;
}

}  // namespace lanelab
}  // namespace engine
