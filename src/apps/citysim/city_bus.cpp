#include "city_bus.h"

#include "../../engine/ai/pathfind.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Real;
using engine::Vec2;

namespace {

// What a street already on a bus route costs the NEXT route, per route using
// it, as a multiple of its travel time: 4.0 makes a shared street cost five
// times as much. Measured on metro: 0 -> 28% of streets with a bus, 1 -> 50%,
// 2 -> 54%, 4 -> 65%, with loop lengths unchanged.
//
// A high price could send a leg far out of its way to dodge a shared street,
// so a priced leg longer than kMaxDetour x the plain fastest leg is dropped in
// favour of the plain one: sharing a street beats a bus that meanders.
constexpr Real kReusePrice = 4.0;
constexpr Real kMaxDetour = 1.6;

// A city bus's average pace between stops (signals, turns, traffic) and the
// time a stop costs it, dwell plus pulling in and out: what a rider's choice
// weighs a ride by. Measured, not guessed: metro buses drive ~8 m/s at cruise
// and stand 10 s + 3 s a rider (city_sim.cpp).
constexpr Real kBusPace = 6.5;
constexpr Real kBusStopSeconds = 16.0;
// A person on foot, and how much longer the streets are than the straight line.
constexpr Real kWalkPace = 1.4;
constexpr Real kStreetFactor = 1.25;
// Changing buses: stops this close count as one interchange, and the change
// itself costs a minute over the walk and the wait (people dislike it).
constexpr Real kTransferWalk = 100.0;
constexpr Real kTransferPenalty = 60.0;

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
    // A STREET junction: a bus stops where a rider can walk, and its legs route on foot-
    // walkable streets. Once a lane-built city's ramps were welded to their freeway, the
    // weld nodes became junctions too — and farthest-point spreading put every hub on the
    // freeway ring around the city, where no walkable leg can reach: metro_lanes derived 0
    // bus routes. Count only walkable, non-freeway, non-ramp approaches.
    auto streetJunction = [&](int i) {
        if (!nav.isJunction(i)) return false;
        int streets = 0;
        for (int li : nav.outLinks[static_cast<std::size_t>(i)]) {
            const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
            if (L.walkable && L.klass != engine::RoadClass::Freeway && L.klass != engine::RoadClass::Ramp) ++streets;
        }
        return streets >= 3;
    };
    std::vector<int> cand;
    for (int i = 0; i < n; ++i)
        if (streetJunction(i)) cand.push_back(i);
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
    // FEWER hubs per ring than there are hubs, whenever there is more than one
    // route. A ring over EVERY hub is the same cycle whichever hub it starts
    // from, so rotating it changes the start and nothing else: with two routes
    // and four hubs, both routes were one loop with the stops renumbered.
    const int H0 = static_cast<int>(hubNodes.size());
    const int perRoute = std::min<int>(4, routeCount > 1 ? H0 - 1 : H0);

    // STREETS ALREADY SERVED COST MORE. Left to the plain fastest-path A*,
    // every leg between two hubs takes the same few arterials, so four loops
    // drove 23.6 km on only 8.5 km of distinct street and 71% of metro's
    // streets never saw a bus (Glenn, 2026-09-18: "I couldn't find one and I
    // walked around for a while"). Charging for a street each time a route
    // uses it -- in either direction, and including the route's own earlier
    // legs, so a loop does not go out and back on one street -- makes the next
    // route prefer the parallel street. It is a price, not a ban: where there
    // is no parallel street, routes still share.
    std::vector<int> streetUses(nav.links.size(), 0);
    std::unordered_map<long long, int> linkAt;   // (from, to) -> link
    for (std::size_t li = 0; li < nav.links.size(); ++li)
        linkAt[static_cast<long long>(nav.links[li].from) * n + nav.links[li].to] =
            static_cast<int>(li);
    auto markStreet = [&](int li) {
        ++streetUses[static_cast<std::size_t>(li)];
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
        const auto back = linkAt.find(static_cast<long long>(L.to) * n + L.from);
        if (back != linkAt.end()) ++streetUses[static_cast<std::size_t>(back->second)];
    };
    std::vector<Real> costScale(nav.links.size(), 1.0);

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

        // IN MAP ORDER. The hubs are numbered in the order farthest-point
        // picked them, and each pick is by construction the point FARTHEST
        // from the last -- so visiting them in that order drove each loop
        // corner to corner across the city, a bow-tie rather than a loop.
        // Sorting by bearing around the ring's own centre makes the loop go
        // round its hubs.
        Vec2 ringMid(0, 0);
        for (int hn : ring) ringMid = ringMid + nav.nodes[static_cast<std::size_t>(hn)];
        ringMid = ringMid * (1.0 / static_cast<Real>(ring.size()));
        std::stable_sort(ring.begin(), ring.end(), [&](int a0, int b0) {
            const Vec2 pa = nav.nodes[static_cast<std::size_t>(a0)];
            const Vec2 pb = nav.nodes[static_cast<std::size_t>(b0)];
            return std::atan2(pa.y - ringMid.y, pa.x - ringMid.x) <
                   std::atan2(pb.y - ringMid.y, pb.x - ringMid.x);
        });

        // --- 3. the legs, PATHFOUND, so the line follows streets ---------
        std::vector<int> pathNodes;
        bool ok = true;
        for (std::size_t k = 0; k < ring.size() && ok; ++k) {
            const int a0 = ring[k], b0 = ring[(k + 1) % ring.size()];
            if (a0 == b0) continue;
            for (std::size_t li = 0; li < costScale.size(); ++li)
                costScale[li] = 1.0 + kReusePrice * streetUses[li];
            // STREETS ONLY: the pedestrian rule skips freeways and ramps, and
            // a city bus has nobody to serve on either. One-way streets keep
            // only their forward link in the graph, so this is still a legal
            // drive.
            engine::Route leg =
                engine::findRoute(nav, a0, b0, /*onFoot=*/true, &costScale);
            if (leg.links.empty()) { ok = false; break; }
            const engine::Route plain = engine::findRoute(nav, a0, b0, /*onFoot=*/true);
            if (!plain.links.empty() && leg.length(nav) > kMaxDetour * plain.length(nav))
                leg = plain;
            if (pathNodes.empty()) pathNodes.push_back(a0);
            for (int li : leg.links) {
                const int to = nav.links[static_cast<std::size_t>(li)].to;
                if (pathNodes.empty() || pathNodes.back() != to) pathNodes.push_back(to);
                markStreet(li);
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
        if (spacing > 300.0) spacing = 300.0;      // but never a hike between stops

        BusRoute route;
        route.pathNodes = pathNodes;
        route.path.reserve(pathNodes.size());
        for (int nd : pathNodes)
            route.path.push_back(nav.nodes[static_cast<std::size_t>(nd)]);

        Real since = spacing;   // a stop at the very first node
        Real arc = 0;           // metres along the loop from its first node
        // Metres since the path last crossed a junction. A bus stands a bus
        // length or so SHORT of its stop node, so a stop a few metres past a
        // junction (on streets cut into ~4 m links, most nodes are) parked the
        // bus inside that junction for its whole dwell. A stop sits AT a
        // junction (the bus waits behind the crosswalk on its approach) or
        // well clear past one.
        constexpr Real kStopPastJunction = 30.0;
        Real sinceJunction = kStopPastJunction;
        for (std::size_t k = 0; k < pathNodes.size(); ++k) {
            const int nd = pathNodes[k];
            if (k > 0) {
                const Real step = dist(nav.nodes[static_cast<std::size_t>(pathNodes[k - 1])],
                                       nav.nodes[static_cast<std::size_t>(nd)]);
                since += step;
                arc += step;
                sinceJunction += step;
            }
            const bool atJunction = nav.isJunction(nd);
            const bool tooNear = !atJunction && sinceJunction < kStopPastJunction;
            if (atJunction) sinceJunction = 0;
            const bool isHub =
                std::find(ring.begin(), ring.end(), nd) != ring.end();
            if ((since < spacing || tooNear) && !isHub) continue;
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
            route.stopArc.push_back(arc);
            since = 0;
        }
        route.loopLength = arc;   // the path closes back onto its first node
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

    // CHANGE POINTS: a stop of one route within kTransferWalk of a stop of
    // another -- a shared hub (0 m) or the next corner. One per stop pair.
    transfers_.clear();
    const int builtRoutes = static_cast<int>(routes_.size());
    for (int r = 0; r < builtRoutes; ++r)
        for (int q = 0; q < builtRoutes; ++q) {
            if (q == r) continue;
            const BusRoute& A = routes_[static_cast<std::size_t>(r)];
            const BusRoute& B = routes_[static_cast<std::size_t>(q)];
            for (std::size_t i = 0; i < A.stops.size(); ++i)
                for (std::size_t j = 0; j < B.stops.size(); ++j) {
                    const Real w = dist(A.stops[i].pos, B.stops[j].pos);
                    if (w <= kTransferWalk)
                        transfers_.push_back({r, static_cast<int>(i), q, static_cast<int>(j), w});
                }
        }
}


Real BusNetwork::coverage(const engine::NavGraph& nav, Real maxWalk) const {
    // Sample every walkable link every few metres and weigh by length, so a
    // long avenue counts for more than a stub. Links are directed, so each
    // street is counted twice -- which cancels in the ratio.
    std::vector<Vec2> stops;
    for (const BusRoute& r : routes_)
        for (const BusStop& s : r.stops) stops.push_back(s.pos);
    const Real r2 = maxWalk * maxWalk;
    constexpr Real kStep = 10.0;
    Real total = 0, covered = 0;
    for (const engine::NavLink& L : nav.links) {
        if (!L.walkable) continue;
        const Vec2 a = nav.nodes[static_cast<std::size_t>(L.from)];
        const Vec2 b = nav.nodes[static_cast<std::size_t>(L.to)];
        const Real len = dist(a, b);
        if (len <= 0) continue;
        const int n = std::max(1, static_cast<int>(len / kStep));
        const Real w = len / n;
        for (int k = 0; k < n; ++k) {
            const Real t = (k + 0.5) / n;
            const Vec2 p(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            total += w;
            for (const Vec2& s : stops) {
                const Real dx = p.x - s.x, dy = p.y - s.y;
                if (dx * dx + dy * dy <= r2) { covered += w; break; }
            }
        }
    }
    return total > 0 ? covered / total : 0;
}

Real BusNetwork::streetShare(const engine::NavGraph& nav) const {
    // A street is an undirected node pair; the directed links of a two-way
    // street would otherwise count it twice on one side of the ratio only.
    const long long n = nav.nodeCount();
    auto key = [n](int a, int b) {
        return a < b ? static_cast<long long>(a) * n + b : static_cast<long long>(b) * n + a;
    };
    std::unordered_map<long long, bool> driven;
    for (const BusRoute& r : routes_)
        for (std::size_t k = 0; k < r.pathNodes.size(); ++k)
            driven[key(r.pathNodes[k], r.pathNodes[(k + 1) % r.pathNodes.size()])] = true;
    std::unordered_map<long long, bool> seen;
    Real total = 0, onRoute = 0;
    for (const engine::NavLink& L : nav.links) {
        if (!L.walkable) continue;
        const long long k = key(L.from, L.to);
        if (seen.count(k)) continue;
        seen[k] = true;
        total += L.length;
        if (driven.count(k)) onRoute += L.length;
    }
    return total > 0 ? onRoute / total : 0;
}

Real BusNetwork::rideMetres(int r, int a, int b) const {
    if (r < 0 || r >= routeCount()) return 0;
    const BusRoute& route = routes_[static_cast<std::size_t>(r)];
    const int n = static_cast<int>(route.stopArc.size());
    if (a < 0 || b < 0 || a >= n || b >= n || route.loopLength <= 0) return 0;
    Real d = route.stopArc[static_cast<std::size_t>(b)] - route.stopArc[static_cast<std::size_t>(a)];
    if (d <= 0) d += route.loopLength;
    return d;
}

Real BusNetwork::rideSeconds(int r, int a, int b) const {
    if (r < 0 || r >= routeCount()) return 0;
    const int n = static_cast<int>(routes_[static_cast<std::size_t>(r)].stops.size());
    if (n <= 0) return 0;
    const int between = ((b - a) % n + n) % n;
    return rideMetres(r, a, b) / kBusPace + between * kBusStopSeconds;
}

Real BusNetwork::waitSeconds(int r) const {
    if (r < 0 || r >= routeCount()) return 0;
    const BusRoute& route = routes_[static_cast<std::size_t>(r)];
    const Real lap = route.loopLength / kBusPace +
                     static_cast<Real>(route.stops.size()) * kBusStopSeconds;
    const int buses = r < static_cast<int>(fleet_.size()) ? std::max(1, fleet_[static_cast<std::size_t>(r)]) : 1;
    return 0.5 * lap / buses;
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

// CHOOSE BY TIME (Glenn, 2026-09-18: "How does an agent decide to take a bus
// to where they are going? Is there some algorithm that ... takes bus routing
// into account to help them get across the city faster?"). The old rule took
// a bus whenever the walks to and from its stops summed shorter than the trip:
// it never asked how long the RIDE was, or which way the loop runs. Measured
// on metro, 58% of the bus trips it accepted were slower than walking and 49%
// rode more than half way round the loop.
//
// Door-to-door time, the way a person weighs it: walk to a stop, wait half a
// headway, ride FORWARD round the loop, walk on -- or change once, at stops of
// two routes within kTransferWalk of each other. The bus is taken only if it
// beats walking by a real margin. A transfer is planned as its FIRST leg, to
// the change stop; stepping off there, the rider plans again, and the second
// route is the obvious answer from where they stand.
BusTrip BusNetwork::planTrip(Vec2 from, Vec2 to, Real maxWalk) const {
    BusTrip best;
    ++stats_.asked;
    if (routes_.empty()) { ++stats_.noRoutes; return best; }
    const Real walkAll = dist(from, to) * kStreetFactor / kWalkPace;
    auto walkT = [](Vec2 p, Vec2 q) { return dist(p, q) * kStreetFactor / kWalkPace; };

    // Stops within the walk of each end, per route.
    const int R = routeCount();
    std::vector<std::vector<int>> nearFrom(static_cast<std::size_t>(R)),
        nearTo(static_cast<std::size_t>(R));
    for (int r = 0; r < R; ++r) {
        const BusRoute& route = routes_[static_cast<std::size_t>(r)];
        for (std::size_t s = 0; s < route.stops.size(); ++s) {
            if (dist(from, route.stops[s].pos) <= maxWalk)
                nearFrom[static_cast<std::size_t>(r)].push_back(static_cast<int>(s));
            if (dist(to, route.stops[s].pos) <= maxWalk)
                nearTo[static_cast<std::size_t>(r)].push_back(static_cast<int>(s));
        }
    }
    Real bestT = 1e30;
    bool bestIsTransfer = false, anyReach = false;
    // One bus.
    for (int r = 0; r < R; ++r) {
        const BusRoute& route = routes_[static_cast<std::size_t>(r)];
        for (int a : nearFrom[static_cast<std::size_t>(r)])
            for (int b : nearTo[static_cast<std::size_t>(r)]) {
                if (a == b) continue;
                anyReach = true;
                const Real t = walkT(from, route.stops[static_cast<std::size_t>(a)].pos) +
                               waitSeconds(r) + rideSeconds(r, a, b) +
                               walkT(route.stops[static_cast<std::size_t>(b)].pos, to);
                if (t < bestT) {
                    bestT = t;
                    best.route = r; best.fromStop = a; best.toStop = b;
                    bestIsTransfer = false;
                }
            }
    }
    // Two buses, changing once.
    for (const Transfer& x : transfers_) {
        const BusRoute& r1 = routes_[static_cast<std::size_t>(x.fromRoute)];
        const BusRoute& r2 = routes_[static_cast<std::size_t>(x.toRoute)];
        for (int a : nearFrom[static_cast<std::size_t>(x.fromRoute)]) {
            if (a == x.fromStop) continue;
            const Real leg1 = walkT(from, r1.stops[static_cast<std::size_t>(a)].pos) +
                              waitSeconds(x.fromRoute) + rideSeconds(x.fromRoute, a, x.fromStop);
            if (leg1 >= bestT) continue;
            for (int b : nearTo[static_cast<std::size_t>(x.toRoute)]) {
                if (b == x.toStop) continue;
                anyReach = true;
                const Real t = leg1 + x.walk * kStreetFactor / kWalkPace + kTransferPenalty +
                               waitSeconds(x.toRoute) + rideSeconds(x.toRoute, x.toStop, b) +
                               walkT(r2.stops[static_cast<std::size_t>(b)].pos, to);
                if (t < bestT) {
                    bestT = t;
                    best.route = x.fromRoute; best.fromStop = a; best.toStop = x.fromStop;
                    bestIsTransfer = true;
                }
            }
        }
    }
    // Only if it is really quicker: a minute and a sixth of the walk at least.
    if (best.valid() && bestT < walkAll - std::max(Real(60), walkAll * Real(0.15))) {
        ++stats_.ok;
        if (bestIsTransfer) ++stats_.transfers;
        return best;
    }
    if (best.valid()) ++stats_.noSaving;
    else if (!anyReach) ++stats_.farFrom;
    return BusTrip{};
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
