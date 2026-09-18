#include "city_bus.h"

#include "../../engine/ai/pathfind.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Real;
using engine::Vec2;

namespace {

Real dist(Vec2 a, Vec2 b) {
    const Real dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

void BusNetwork::clear() {
    routes_.clear();
    waiting_.clear();
}

void BusNetwork::build(const engine::NavGraph& nav, int routeCount,
                       int stopsPerRoute, uint32_t seed) {
    routes_.clear();
    waiting_.clear();
    hubs_.clear();
    const int n = nav.nodeCount();
    if (n < 4 || routeCount <= 0 || stopsPerRoute < 2) return;

    // --- 1. HUBS --------------------------------------------------------
    // Junctions, because a hub wants several streets meeting: a rider changing
    // buses there can also walk somewhere. Spread by farthest-point so the
    // network spans the city, ties on the lower index for determinism.
    std::vector<int> cand;
    for (int i = 0; i < n; ++i)
        if (nav.isJunction(i)) cand.push_back(i);
    if (static_cast<int>(cand.size()) < 3)
        for (int i = 0; i < n; ++i) cand.push_back(i);   // a city with no junctions

    const int wantHubs = std::max(3, std::min(routeCount + 2, 8));
    uint32_t h = seed ? seed : 1u;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    std::vector<int> hubNodes;
    hubNodes.push_back(cand[h % cand.size()]);
    std::vector<Real> best(cand.size(), 0);
    for (std::size_t i = 0; i < cand.size(); ++i)
        best[i] = dist(nav.nodes[static_cast<std::size_t>(cand[i])],
                       nav.nodes[static_cast<std::size_t>(hubNodes[0])]);
    while (static_cast<int>(hubNodes.size()) < wantHubs) {
        std::size_t pick = cand.size();
        Real pickD = -1;
        for (std::size_t i = 0; i < cand.size(); ++i)
            if (best[i] > pickD) { pickD = best[i]; pick = i; }
        if (pick == cand.size() || pickD <= 0) break;
        hubNodes.push_back(cand[pick]);
        for (std::size_t i = 0; i < cand.size(); ++i)
            best[i] = std::min(best[i],
                               dist(nav.nodes[static_cast<std::size_t>(cand[i])],
                                    nav.nodes[static_cast<std::size_t>(cand[pick])]));
    }
    if (hubNodes.size() < 3) return;
    for (int hn : hubNodes) hubs_.push_back(nav.nodes[static_cast<std::size_t>(hn)]);

    // --- 2. one LOOP per route, over a rotating subset of the hubs -------
    // Rotating means consecutive routes SHARE hubs, which is the whole point:
    // a shared stop is a transfer.
    const int perRoute = std::min<int>(4, static_cast<int>(hubNodes.size()));
    for (int r = 0; r < routeCount; ++r) {
        // CONSECUTIVE hubs, rotated by route. The first cut strided by two
        // ((r + k*2) % H), which wrapped the fourth hub back onto the first --
        // so route 2's ring [2,4,0,2] was route 0's [0,2,4,0] traversed from a
        // different start, and the two drew the same line. Rotating by ONE
        // gives every route a distinct ring that still OVERLAPS its neighbours,
        // which is what makes a hub a transfer rather than a decoration.
        const int H = static_cast<int>(hubNodes.size());
        std::vector<int> ring;
        for (int k = 0; k < perRoute; ++k) {
            const int hub = hubNodes[static_cast<std::size_t>((r + k) % H)];
            if (std::find(ring.begin(), ring.end(), hub) == ring.end())
                ring.push_back(hub);
        }
        if (ring.size() < 3) continue;   // a loop needs three corners

        // --- 3. the legs, PATHFOUND, so the line follows streets ---------
        std::vector<int> pathNodes;
        bool ok = true;
        for (std::size_t k = 0; k < ring.size() && ok; ++k) {
            const int a0 = ring[k], b0 = ring[(k + 1) % ring.size()];
            if (a0 == b0) continue;
            const engine::Route leg = engine::findRoute(nav, a0, b0, /*onFoot=*/false);
            if (leg.links.empty()) { ok = false; break; }
            if (pathNodes.empty()) pathNodes.push_back(a0);
            for (int li : leg.links) {
                const int to = nav.links[static_cast<std::size_t>(li)].to;
                if (pathNodes.empty() || pathNodes.back() != to) pathNodes.push_back(to);
            }
        }
        if (!ok || pathNodes.size() < 4) continue;

        // --- 4. STOPS along the path, plus one at every hub --------------
        Real total = 0;
        for (std::size_t k = 1; k < pathNodes.size(); ++k)
            total += dist(nav.nodes[static_cast<std::size_t>(pathNodes[k - 1])],
                          nav.nodes[static_cast<std::size_t>(pathNodes[k])]);
        Real spacing = total / std::max(1, stopsPerRoute);
        if (spacing < 180.0) spacing = 180.0;      // not every kerb
        if (spacing > 500.0) spacing = 500.0;      // but never a hike between stops

        BusRoute route;
        route.path.reserve(pathNodes.size());
        for (int nd : pathNodes)
            route.path.push_back(nav.nodes[static_cast<std::size_t>(nd)]);

        Real since = spacing;   // a stop at the very first node
        for (std::size_t k = 0; k < pathNodes.size(); ++k) {
            const int nd = pathNodes[k];
            if (k > 0)
                since += dist(nav.nodes[static_cast<std::size_t>(pathNodes[k - 1])],
                              nav.nodes[static_cast<std::size_t>(nd)]);
            const bool isHub =
                std::find(ring.begin(), ring.end(), nd) != ring.end();
            if (since < spacing && !isHub) continue;
            // ONCE PER CIRCUIT. The loop closes back onto its first hub, so
            // without this the origin appears as both the first stop and the
            // last, and the A* legs can re-cross a node mid-route -- a bus
            // would "serve" the same stop twice a lap and a rider waiting there
            // could be picked up on the wrong pass.
            bool already = false;
            for (const BusStop& have : route.stops)
                if (have.node == nd) { already = true; break; }
            if (already) continue;
            if (isHub) route.hubStops.push_back(static_cast<int>(route.stops.size()));
            route.stops.push_back({nd, nav.nodes[static_cast<std::size_t>(nd)]});
            since = 0;
        }
        if (!route.valid()) continue;

        // The depot: the most peripheral stop, so an off-duty bus leaves town.
        Vec2 mid(0, 0);
        for (const Vec2& p : nav.nodes) mid = mid + p;
        mid = mid * (1.0 / static_cast<Real>(nav.nodes.size()));
        Real far = -1;
        for (const BusStop& st : route.stops) {
            const Real d = dist(st.pos, mid);
            if (d > far) { far = d; route.depotNode = st.node; route.depotPos = st.pos; }
        }
        routes_.push_back(std::move(route));
    }
}


int BusNetwork::nearestStop(int r, Vec2 p) const {
    if (r < 0 || r >= routeCount()) return -1;
    const BusRoute& route = routes_[static_cast<std::size_t>(r)];
    int best = -1;
    Real bestD = 0;
    for (std::size_t i = 0; i < route.stops.size(); ++i) {
        const Real d = dist(p, route.stops[i].pos);
        if (best < 0 || d < bestD) { best = static_cast<int>(i); bestD = d; }
    }
    return best;
}

BusTrip BusNetwork::planTrip(Vec2 from, Vec2 to, Real maxWalk) const {
    BusTrip best;
    Real bestWalk = 0;
    ++stats_.asked;
    if (routes_.empty()) { ++stats_.noRoutes; return best; }
    bool sawSame = false, sawFarFrom = false, sawFarTo = false, sawNoSaving = false;
    for (int r = 0; r < routeCount(); ++r) {
        const int a = nearestStop(r, from);
        const int b = nearestStop(r, to);
        if (a < 0 || b < 0 || a == b) { sawSame = true; continue; }
        const BusRoute& route = routes_[static_cast<std::size_t>(r)];
        const Real walkA = dist(from, route.stops[static_cast<std::size_t>(a)].pos);
        const Real walkB = dist(to, route.stops[static_cast<std::size_t>(b)].pos);
        if (walkA > maxWalk) { sawFarFrom = true; continue; }
        if (walkB > maxWalk) { sawFarTo = true; continue; }
        // Score on the walking left over at BOTH ends: a route that drops you
        // half a mile from the door is worse than one that does not, however
        // convenient its pickup.
        const Real walk = walkA + walkB;
        // A bus is only worth taking if it actually saves walking.
        if (walk >= dist(from, to)) { sawNoSaving = true; continue; }
        if (!best.valid() || walk < bestWalk) {
            best.route = r;
            best.fromStop = a;
            best.toStop = b;
            bestWalk = walk;
        }
    }
    if (best.valid()) ++stats_.ok;
    else if (sawNoSaving) ++stats_.noSaving;
    else if (sawFarTo) ++stats_.farTo;
    else if (sawFarFrom) ++stats_.farFrom;
    else if (sawSame) ++stats_.sameStop;
    return best;
}

bool BusNetwork::waitFor(int passenger, const BusTrip& trip) {
    if (passenger < 0 || !trip.valid()) return false;
    if (trip.route >= routeCount()) return false;
    if (waiting_.count(passenger)) return false;
    waiting_[passenger] = trip;
    return true;
}

void BusNetwork::stopWaiting(int passenger) { waiting_.erase(passenger); }

const BusTrip* BusNetwork::tripOf(int passenger) const {
    const auto it = waiting_.find(passenger);
    return it == waiting_.end() ? nullptr : &it->second;
}

std::vector<int> BusNetwork::waitingAt(int route, int stop) const {
    std::vector<int> out;
    for (const auto& kv : waiting_)
        if (!kv.second.aboard && kv.second.route == route &&
            kv.second.fromStop == stop)
            out.push_back(kv.first);
    std::sort(out.begin(), out.end());   // hash order must not reach the sim
    return out;
}

std::vector<int> BusNetwork::alightingAt(int route, int stop) const {
    std::vector<int> out;
    for (const auto& kv : waiting_)
        if (kv.second.aboard && kv.second.route == route &&
            kv.second.toStop == stop)
            out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

void BusNetwork::markAboard(int passenger) {
    const auto it = waiting_.find(passenger);
    if (it != waiting_.end()) it->second.aboard = true;
}

}  // namespace citysim
