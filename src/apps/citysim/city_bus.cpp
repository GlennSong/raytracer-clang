#include "city_bus.h"

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

void BusNetwork::build(const std::vector<Vec2>& nodes, int routeCount,
                       int stopsPerRoute, uint32_t seed) {
    routes_.clear();
    waiting_.clear();
    const int n = static_cast<int>(nodes.size());
    if (n < 2 || routeCount <= 0 || stopsPerRoute < 2) return;
    const int want = std::min(stopsPerRoute, n);

    for (int r = 0; r < routeCount; ++r) {
        // A different first anchor per route, walked deterministically off the
        // seed, so two routes do not come out as the same loop.
        uint32_t h = seed ^ (0x9e3779b9u * static_cast<uint32_t>(r + 1));
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        std::vector<int> chosen;
        chosen.push_back(static_cast<int>(h % static_cast<uint32_t>(n)));

        // FARTHEST-POINT SAMPLING: each new stop is the node furthest from
        // every stop already chosen, so a route spans the city instead of
        // clumping in whichever district the first node landed in. Ties take
        // the lower node index, which is what makes this reproducible.
        std::vector<Real> best(static_cast<std::size_t>(n), 0);
        for (int i = 0; i < n; ++i)
            best[static_cast<std::size_t>(i)] =
                dist(nodes[static_cast<std::size_t>(i)],
                     nodes[static_cast<std::size_t>(chosen[0])]);
        while (static_cast<int>(chosen.size()) < want) {
            int pick = -1;
            Real pickD = -1;
            for (int i = 0; i < n; ++i) {
                const Real d = best[static_cast<std::size_t>(i)];
                if (d > pickD) { pickD = d; pick = i; }
            }
            if (pick < 0 || pickD <= 0) break;   // no distinct node left
            chosen.push_back(pick);
            for (int i = 0; i < n; ++i)
                best[static_cast<std::size_t>(i)] =
                    std::min(best[static_cast<std::size_t>(i)],
                             dist(nodes[static_cast<std::size_t>(i)],
                                  nodes[static_cast<std::size_t>(pick)]));
        }
        if (chosen.size() < 2) continue;

        // Order them into a LOOP, nearest-neighbour from the first anchor, so
        // consecutive stops are actually near each other and the bus does not
        // criss-cross the city between every pair. Ties: lower index.
        std::vector<int> tour;
        std::vector<char> used(chosen.size(), 0);
        tour.push_back(chosen[0]);
        used[0] = 1;
        for (std::size_t step = 1; step < chosen.size(); ++step) {
            const Vec2 cur = nodes[static_cast<std::size_t>(tour.back())];
            std::size_t pick = chosen.size();
            Real pickD = 0;
            for (std::size_t c = 0; c < chosen.size(); ++c) {
                if (used[c]) continue;
                const Real d = dist(cur, nodes[static_cast<std::size_t>(chosen[c])]);
                if (pick == chosen.size() || d < pickD) { pick = c; pickD = d; }
            }
            if (pick == chosen.size()) break;
            used[pick] = 1;
            tour.push_back(chosen[pick]);
        }

        BusRoute route;
        route.stops.reserve(tour.size());
        for (int node : tour)
            route.stops.push_back({node, nodes[static_cast<std::size_t>(node)]});
        if (route.valid()) routes_.push_back(std::move(route));
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
    for (int r = 0; r < routeCount(); ++r) {
        const int a = nearestStop(r, from);
        const int b = nearestStop(r, to);
        if (a < 0 || b < 0 || a == b) continue;   // same stop: the bus is no help
        const BusRoute& route = routes_[static_cast<std::size_t>(r)];
        const Real walkA = dist(from, route.stops[static_cast<std::size_t>(a)].pos);
        const Real walkB = dist(to, route.stops[static_cast<std::size_t>(b)].pos);
        if (walkA > maxWalk || walkB > maxWalk) continue;
        // Score on the walking left over at BOTH ends: a route that drops you
        // half a mile from the door is worse than one that does not, however
        // convenient its pickup.
        const Real walk = walkA + walkB;
        // A bus is only worth taking if it actually saves walking.
        if (walk >= dist(from, to)) continue;
        if (!best.valid() || walk < bestWalk) {
            best.route = r;
            best.fromStop = a;
            best.toStop = b;
            bestWalk = walk;
        }
    }
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
