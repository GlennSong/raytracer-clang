#include "city_rail_riders.h"

#include <algorithm>
#include <cmath>
#include <queue>

using engine::Real;
using engine::Vec2;
using engine::Vec3;

namespace citysim {

namespace {

constexpr Real kWalkPace = 1.4;          // m/s along a way in (the stairs included)
constexpr Real kRidePace = 25.0;         // m/s, a train's average between stops (accelerating, curves)
constexpr Real kPlatformBand0 = 0.8, kPlatformBand1 = 2.3;   // the band waiting riders stand in: metres in from the
                                                             // platform's edge (the stair runs along its back)

Real dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

// where the platform is: the line's railhead at `s`, the way it runs, and the platform side's plan direction
void frameAt(const RailLine& L, double s, int side, Vec2& p, Vec2& fwd, Vec2& out) {
    const Vec3 a = L.at(s - 2.0), b = L.at(s + 2.0), c = L.at(s);
    fwd = Vec2(b.x - a.x, b.z - a.z);
    const Real fl = std::hypot(fwd.x, fwd.y);
    fwd = fl > 1e-9 ? Vec2(fwd.x / fl, fwd.y / fl) : Vec2(0, 1);
    const Vec2 left(fwd.y, -fwd.x);   // cross(up, f) in plan: rail_sim.cpp's car frame
    out = Vec2(left.x * side, left.y * side);
    p = Vec2(c.x, c.z);
}

}  // namespace

void RailRiders::clear() {
    rail_ = nullptr;
    comp_.clear();
    stations_.clear();
    trips_.clear();
    stats = Stats{};
}

void RailRiders::build(const RailSim* rail, const engine::NavGraph& nav, Real reach, Real minNetworkStreet) {
    clear();
    if (!rail || rail->lines().empty() || nav.nodeCount() == 0) return;
    rail_ = rail;

    // STREET NETWORKS: components over walkable, non-freeway links (a town is one; the freeway joins them for cars only)
    const int N = nav.nodeCount();
    std::vector<std::vector<int>> adj(static_cast<std::size_t>(N));
    for (const engine::NavLink& L : nav.links) {
        if (!L.walkable || L.klass == engine::RoadClass::Freeway || L.klass == engine::RoadClass::Ramp) continue;
        adj[static_cast<std::size_t>(L.from)].push_back(L.to);
        adj[static_cast<std::size_t>(L.to)].push_back(L.from);
    }
    comp_.assign(static_cast<std::size_t>(N), -1);
    std::vector<Real> street;
    for (int s = 0; s < N; ++s) {
        if (comp_[static_cast<std::size_t>(s)] >= 0 || adj[static_cast<std::size_t>(s)].empty()) continue;
        const int c = static_cast<int>(street.size());
        street.push_back(0);
        std::queue<int> q;
        q.push(s);
        comp_[static_cast<std::size_t>(s)] = c;
        while (!q.empty()) {
            const int u = q.front(); q.pop();
            for (int v : adj[static_cast<std::size_t>(u)])
                if (comp_[static_cast<std::size_t>(v)] < 0) { comp_[static_cast<std::size_t>(v)] = c; q.push(v); }
        }
    }
    for (const engine::NavLink& L : nav.links)
        if (L.walkable && L.klass != engine::RoadClass::Freeway && L.klass != engine::RoadClass::Ramp)
            street[static_cast<std::size_t>(comp_[static_cast<std::size_t>(L.from)])] += 0.5 * L.length;
    const int biggest = street.empty() ? -1 : static_cast<int>(std::max_element(street.begin(), street.end()) - street.begin());

    // THE STATIONS, by name across the lines (a double track's two lines both stop at each)
    const auto& lines = rail->lines();
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const RailLine& L = lines[li];
        for (std::size_t si = 0; si < L.stops.size(); ++si) {
            const engine::RailStop& st = L.stops[si];
            int k = -1;
            for (std::size_t j = 0; j < stations_.size(); ++j) if (stations_[j].name == st.name) k = static_cast<int>(j);
            if (k < 0) {
                RailStation S;
                S.name = st.name;
                S.stopOnLine.assign(lines.size(), -1);
                // the way in; a station without one (a lab's) is reached from beside its platform
                Vec2 p, f, o;
                frameAt(L, 0.5 * (st.s0 + st.s1), st.side, p, f, o);
                const Vec2 mid(p.x + o.x * (engine::kPlatformGap + 2.25), p.y + o.y * (engine::kPlatformGap + 2.25));
                S.access = st.hasAccess ? st.access : mid;
                S.walkIn = std::max<Real>(20.0, dist(S.access, mid) * 1.4 / kWalkPace);
                stations_.push_back(S);
                k = static_cast<int>(stations_.size()) - 1;
            }
            stations_[static_cast<std::size_t>(k)].stopOnLine[li] = static_cast<int>(si);
        }
    }
    // each pinned to a street node
    for (RailStation& S : stations_) {
        Real bd = reach;
        for (const engine::NavLink& L : nav.links) {
            if (!L.walkable || L.klass == engine::RoadClass::Freeway || L.klass == engine::RoadClass::Ramp) continue;
            const int c = comp_[static_cast<std::size_t>(L.from)];
            if (c != biggest && street[static_cast<std::size_t>(c)] < minNetworkStreet) continue;
            const Real d = dist(nav.nodes[static_cast<std::size_t>(L.from)], S.access);
            if (d < bd) { bd = d; S.accessNode = L.from; S.network = c; }
        }
    }
}

std::vector<int> RailRiders::passengers() const {
    std::vector<int> out;
    out.reserve(trips_.size());
    for (const auto& kv : trips_) out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

bool RailRiders::plan(int fromNode, Vec2 fromPos, int toNode, Vec2 toPos, RailTrip& out, Real* seconds) const {
    if (!active()) return false;
    const int na = networkOf(fromNode), nb = networkOf(toNode);
    if (na < 0 || nb < 0) return false;
    // the station in each end's network nearest that end (straight line: the street leg is the sim's to plan)
    int a = -1, b = -1;
    Real da = 1e30, db = 1e30;
    for (std::size_t k = 0; k < stations_.size(); ++k) {
        const RailStation& S = stations_[k];
        if (S.accessNode < 0) continue;
        if (S.network == na && dist(S.access, fromPos) < da) { da = dist(S.access, fromPos); a = static_cast<int>(k); }
        if (S.network == nb && dist(S.access, toPos) < db) { db = dist(S.access, toPos); b = static_cast<int>(k); }
    }
    if (a < 0 || b < 0 || a == b) return false;
    // the line with the shorter ride from a to b
    const auto& lines = rail_->lines();
    const auto& trains = rail_->trains();
    Real bestT = 1e30;
    RailTrip best;
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const RailLine& L = lines[li];
        const int sa = stations_[static_cast<std::size_t>(a)].stopOnLine[li];
        const int sb = stations_[static_cast<std::size_t>(b)].stopOnLine[li];
        if (sa < 0 || sb < 0) continue;
        double arc = L.stops[static_cast<std::size_t>(sb)].s1 - L.stops[static_cast<std::size_t>(sa)].s1;
        if (arc <= 0) {
            if (!L.loop) continue;   // an open line runs one way
            arc += L.length;
        }
        int between = (sb - sa + static_cast<int>(L.stops.size())) % static_cast<int>(L.stops.size()) - 1;
        if (!L.loop) between = sb - sa - 1;
        int count = 0;
        for (const Train& t : trains) if (t.line == static_cast<int>(li)) ++count;
        if (count == 0) continue;
        const Real dwell = trains.empty() ? 30.0 : trains.front().kind.dwell;
        const Real lap = L.length / kRidePace + dwell * static_cast<Real>(L.stops.size());
        const Real wait = 0.5 * lap / count;
        const Real t = wait + arc / kRidePace + dwell * std::max(0, between);
        if (t < bestT) {
            bestT = t;
            best = RailTrip{};
            best.from = a; best.to = b; best.line = static_cast<int>(li); best.fromStop = sa; best.toStop = sb;
        }
    }
    if (best.line < 0) return false;
    out = best;
    if (seconds) *seconds = bestT;
    return true;
}

bool RailRiders::platformSpot(int st, int line, int k, Vec2& pos, Real& top, Vec2& face) const {
    if (!rail_ || st < 0 || st >= static_cast<int>(stations_.size())) return false;
    const auto& lines = rail_->lines();
    if (line < 0 || line >= static_cast<int>(lines.size())) return false;
    const int si = stations_[static_cast<std::size_t>(st)].stopOnLine[static_cast<std::size_t>(line)];
    if (si < 0) return false;
    const RailLine& L = lines[static_cast<std::size_t>(line)];
    const engine::RailStop& S = L.stops[static_cast<std::size_t>(si)];
    // spots spread over the platform's middle 70 %, a hash of k so a crowd fills it unevenly, two rows deep
    const uint32_t h = static_cast<uint32_t>(k) * 2654435761u;
    const Real u = static_cast<Real>((h >> 8) % 1000) / 1000.0;
    const Real s = S.s0 + (S.s1 - S.s0) * (0.15 + 0.7 * u);
    const Real band = kPlatformBand0 + (kPlatformBand1 - kPlatformBand0) * static_cast<Real>((h >> 20) % 100) / 100.0;
    Vec2 p, f, o;
    frameAt(L, s, S.side, p, f, o);
    const Real lat = engine::kPlatformGap + band;
    pos = Vec2(p.x + o.x * lat, p.y + o.y * lat);
    top = S.top;
    face = Vec2(-o.x, -o.y);
    return true;
}

}  // namespace citysim
