#include "city_bus.h"

#include "../../engine/ai/pathfind.h"

#include <algorithm>
#include <map>
#include <cstdlib>
#include <cstdio>
#include <tuple>
#include <queue>
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
// A place out along the freeway earns local buses from this much street up; the
// regional route stops once per this much of a place's extent (at most three).
constexpr Real kMinPlaceStreet = 3000.0;
constexpr Real kRegionalStopSpan = 1500.0;
// A bus's pace on the freeway and its ramps, against the class speed limit.
constexpr Real kFreewayBusShare = 0.8;
// A rider changes buses at most this many times (town bus, regional, city bus).
constexpr int kMaxChanges = 2;

Real dist(Vec2 a, Vec2 b) {
    const Real dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

void BusNetwork::clear() {
    routes_.clear();
    waiting_.clear();
    comp_.clear();
}

// THE REGIONAL ROUTE (Glenn, 2026-09-23: "regional buses ... that go between towns").
// One loop joining every street network that has local buses, driven by freeway.
// It stops only at stops the local routes already have -- a change of bus there is
// a 0 m walk -- one per kRegionalStopSpan of a place's extent, so the 3 km mountain
// city gets two and a town one: the stop nearest the place's middle first, then the
// farthest from those already chosen. The loop visits them in the shortest closed
// order by straight line (nearest neighbour, then 2-opt), and each leg is the car
// router's, so the bus takes the freeway wherever it is faster.
void BusNetwork::buildRegional(const engine::NavGraph& nav) {
    std::map<int, std::vector<BusStop>> byNet;
    for (const BusRoute& r : routes_) {
        if (r.regional || r.network < 0) continue;
        for (const BusStop& st : r.stops) {
            auto& v = byNet[r.network];
            bool dup = false;
            for (const BusStop& o : v) if (o.node == st.node) { dup = true; break; }
            if (!dup) v.push_back(st);
        }
    }
    if (byNet.size() < 2) return;
    std::vector<BusStop> picks;
    for (auto& kv : byNet) {
        const std::vector<BusStop>& sts = kv.second;
        Vec2 mid(0, 0);
        for (const BusStop& st : sts) mid = mid + st.pos;
        mid = mid * (1.0 / static_cast<Real>(sts.size()));
        Real extent = 0;
        for (const BusStop& a : sts) for (const BusStop& b : sts) extent = std::max(extent, dist(a.pos, b.pos));
        const int want = std::clamp(static_cast<int>(std::lround(extent / kRegionalStopSpan)), 1, 3);
        std::vector<BusStop> mine;
        std::size_t first = 0;
        for (std::size_t i = 1; i < sts.size(); ++i) if (dist(sts[i].pos, mid) < dist(sts[first].pos, mid)) first = i;
        mine.push_back(sts[first]);
        while (static_cast<int>(mine.size()) < want) {
            std::size_t best = sts.size(); Real bestD = -1;
            for (std::size_t i = 0; i < sts.size(); ++i) {
                Real d = 1e30;
                for (const BusStop& m : mine) d = std::min(d, dist(sts[i].pos, m.pos));
                if (d > bestD) { bestD = d; best = i; }
            }
            if (best == sts.size() || bestD <= 0) break;
            mine.push_back(sts[best]);
        }
        picks.insert(picks.end(), mine.begin(), mine.end());
    }
    // the visiting order
    const std::size_t m = picks.size();
    std::vector<std::size_t> order{0};
    std::vector<char> used(m, 0); used[0] = 1;
    while (order.size() < m) {
        std::size_t best = m;
        for (std::size_t i = 0; i < m; ++i)
            if (!used[i] && (best == m || dist(picks[order.back()].pos, picks[i].pos) < dist(picks[order.back()].pos, picks[best].pos))) best = i;
        used[best] = 1; order.push_back(best);
    }
    auto at = [&](std::size_t k) { return picks[order[k % m]].pos; };
    for (bool improved = true; improved;) {
        improved = false;
        for (std::size_t i = 0; i + 1 < m; ++i)
            for (std::size_t j = i + 2; j < m; ++j) {
                if (i == 0 && j == m - 1) continue;
                const Real before = dist(at(i), at(i + 1)) + dist(at(j), at(j + 1));
                const Real after = dist(at(i), at(j)) + dist(at(i + 1), at(j + 1));
                if (after < before - 1e-6) { std::reverse(order.begin() + static_cast<long>(i) + 1, order.begin() + static_cast<long>(j) + 1); improved = true; }
            }
    }
    // the legs, by car
    BusRoute route;
    route.regional = true;
    Real metres = 0, seconds = 0;
    for (std::size_t k = 0; k < m; ++k) {
        const int a0 = picks[order[k]].node, b0 = picks[order[(k + 1) % m]].node;
        if (route.pathNodes.empty()) route.pathNodes.push_back(a0);
        route.stopArc.push_back(metres);
        route.stops.push_back(picks[order[k]]);
        route.stops.back().pathIndex = static_cast<int>(route.pathNodes.size()) - 1;   // a0 is the path's last node here
        const engine::Route leg = engine::findRoute(nav, a0, b0, /*onFoot=*/false);
        if (leg.links.empty()) return;   // a place the car router cannot reach: no regional route
        for (int li : leg.links) {
            const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
            const bool fast = L.klass == engine::RoadClass::Freeway || L.klass == engine::RoadClass::Ramp;
            metres += L.length;
            seconds += L.length / (fast ? kFreewayBusShare * engine::classSpeed(L.klass) : kBusPace);
            if (route.pathNodes.back() != L.to) route.pathNodes.push_back(L.to);
        }
    }
    if (route.pathNodes.size() > 1 && route.pathNodes.back() == route.pathNodes.front()) route.pathNodes.pop_back();
    for (int nd : route.pathNodes) route.path.push_back(nav.nodes[static_cast<std::size_t>(nd)]);
    route.loopLength = metres;
    route.pace = seconds > 0 ? metres / seconds : kBusPace;
    Vec2 mid(0, 0);
    for (const Vec2& p : nav.nodes) mid = mid + p;
    mid = mid * (1.0 / static_cast<Real>(nav.nodes.size()));
    Real far = -1;
    for (const BusStop& st : route.stops)
        if (dist(st.pos, mid) > far) { far = dist(st.pos, mid); route.depotNode = st.node; route.depotPos = st.pos; }
    if (route.valid()) routes_.push_back(std::move(route));
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
    // ...ONE STREET NETWORK AT A TIME. With towns out along a freeway (metro_planned), the
    // farthest-point spread put hubs in them, no street leg reached them, and every route was
    // dropped: "24 buses on 0 derived routes". A network is a set of junctions joined by
    // walkable streets: the city gets the level's routes, each place out along the freeway
    // (a town, the mountain city) loops of its own, and a REGIONAL loop joins them by freeway.
    comp_.assign(static_cast<std::size_t>(n), -1);
    std::vector<int>& comp = comp_;
    std::vector<int> compSize;
    std::vector<Real> compStreet;   // metres of street in each network
    {
        std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
        for (const engine::NavLink& L : nav.links)
            if (L.walkable && L.klass != engine::RoadClass::Freeway && L.klass != engine::RoadClass::Ramp) {
                adj[static_cast<std::size_t>(L.from)].push_back(L.to);
                adj[static_cast<std::size_t>(L.to)].push_back(L.from);
            }
        for (int s0 = 0; s0 < n; ++s0) {
            if (comp[static_cast<std::size_t>(s0)] >= 0) continue;
            const int c = static_cast<int>(compSize.size());
            compSize.push_back(0);
            compStreet.push_back(0);
            std::vector<int> stack{s0};
            comp[static_cast<std::size_t>(s0)] = c;
            while (!stack.empty()) {
                const int u = stack.back(); stack.pop_back();
                ++compSize.back();
                for (int v : adj[static_cast<std::size_t>(u)])
                    if (comp[static_cast<std::size_t>(v)] < 0) { comp[static_cast<std::size_t>(v)] = c; stack.push_back(v); }
            }
        }
    }
    for (const engine::NavLink& L : nav.links)   // each two-way street is two links: half each
        if (L.walkable && L.klass != engine::RoadClass::Freeway && L.klass != engine::RoadClass::Ramp)
            compStreet[static_cast<std::size_t>(comp[static_cast<std::size_t>(L.from)])] += 0.5 * L.length;
    const int mainComp = static_cast<int>(std::max_element(compSize.begin(), compSize.end()) - compSize.begin());

    // Loops over ONE network `c`: steps 1-4 below.
    std::vector<int> streetUses(nav.links.size(), 0);
    auto buildLoops = [&](int c, int routeCount, uint32_t netSeed) {
    std::vector<int> cand;
    for (int i = 0; i < n; ++i)
        if (streetJunction(i) && comp[static_cast<std::size_t>(i)] == c) cand.push_back(i);
    if (static_cast<int>(cand.size()) < 3)   // a network with no junctions
        for (int i = 0; i < n; ++i) if (comp[static_cast<std::size_t>(i)] == c) cand.push_back(i);
    if (static_cast<int>(cand.size()) < 3) return;

    const int wantHubs = std::max(3, std::min(routeCount + 2, 8));
    uint32_t h = netSeed ? netSeed : 1u;
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
            route.stops.push_back({nd, nav.nodes[static_cast<std::size_t>(nd)], static_cast<int>(k)});
            route.stopArc.push_back(arc);
            since = 0;
        }
        // an OPEN loop, as the regional builder leaves it: the sweep ends back on its first node
        // (the drawn `path` keeps the closing point for maps)
        if (route.pathNodes.size() > 1 && route.pathNodes.back() == route.pathNodes.front()) route.pathNodes.pop_back();
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
        route.network = c;
        routes_.push_back(std::move(route));
    }
    };

    buildLoops(mainComp, routeCount, seed);
    // THE PLACES OUT ALONG THE FREEWAY: loops of their own, as many as their streets would
    // earn at the city's rate (street per route), at least one, never more than the city.
    // A network under kMinPlaceStreet is a hamlet, a stray cul-de-sac or an interchange's
    // frontage stub, and gets none.
    const Real perRoute = compStreet[static_cast<std::size_t>(mainComp)] / std::max(1, routeCount);
    if (std::getenv("RT_BUS_WHY"))
        for (int c = 0; c < static_cast<int>(compSize.size()); ++c)
            if (compStreet[static_cast<std::size_t>(c)] > 300) {
                Vec2 m(0, 0); int k = 0;
                for (int i = 0; i < n; ++i) if (comp[static_cast<std::size_t>(i)] == c) { m = m + nav.nodes[static_cast<std::size_t>(i)]; ++k; }
                std::fprintf(stderr, "[bus] network %d: %.0f m of street, %d nodes, around (%.0f, %.0f)\n", c, compStreet[static_cast<std::size_t>(c)], compSize[static_cast<std::size_t>(c)], m.x / k, m.y / k);
            }
    for (int c = 0; c < static_cast<int>(compSize.size()); ++c) {
        if (c == mainComp || compStreet[static_cast<std::size_t>(c)] < kMinPlaceStreet) continue;
        const int want = std::clamp(static_cast<int>(std::lround(compStreet[static_cast<std::size_t>(c)] / perRoute)), 1, routeCount);
        buildLoops(c, want, (seed ? seed : 1u) ^ (static_cast<uint32_t>(c) * 0x9E3779B9u));
    }
    buildRegional(nav);

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
    const Real pace = routes_[static_cast<std::size_t>(r)].pace > 0 ? routes_[static_cast<std::size_t>(r)].pace : kBusPace;
    return rideMetres(r, a, b) / pace + between * kBusStopSeconds;
}

Real BusNetwork::waitSeconds(int r) const {
    if (r < 0 || r >= routeCount()) return 0;
    const BusRoute& route = routes_[static_cast<std::size_t>(r)];
    const Real lap = route.loopLength / (route.pace > 0 ? route.pace : kBusPace) +
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
    // UP TO kMaxChanges CHANGES (a town bus, the regional, a city bus), as the least
    // door-to-door time over states "at stop s of route r, about to board" and "just
    // stepped off route r at stop s", each with how many changes it took. Settling
    // them in time order (Dijkstra) finds the same trip the one-bus and one-change
    // loops did whenever that is the fastest, and the two-change trip only when it is.
    // The trip handed back is still its FIRST leg: stepping off at the change the
    // rider plans again from there.
    struct Lab { Real t; int kind, r, s, changes; int firstR, firstA, firstB; };
    auto cmp = [](const Lab& x, const Lab& y) { return x.t > y.t; };
    std::priority_queue<Lab, std::vector<Lab>, decltype(cmp)> pq(cmp);
    std::map<std::tuple<int, int, int, int>, Real> settled;   // (kind, r, s, changes) -> time
    for (int r = 0; r < R; ++r)
        for (int a : nearFrom[static_cast<std::size_t>(r)])
            pq.push({walkT(from, routes_[static_cast<std::size_t>(r)].stops[static_cast<std::size_t>(a)].pos) + waitSeconds(r), 0, r, a, 0, r, a, -1});
    Real bestT = 1e30;
    bool bestIsTransfer = false, anyReach = false;
    while (!pq.empty()) {
        const Lab L = pq.top(); pq.pop();
        if (L.t >= bestT) break;
        const auto key = std::make_tuple(L.kind, L.r, L.s, L.changes);
        if (settled.count(key)) continue;
        settled[key] = L.t;
        const BusRoute& route = routes_[static_cast<std::size_t>(L.r)];
        const int n = static_cast<int>(route.stops.size());
        if (L.kind == 0) {   // aboard at L.s: ride forward to any other stop
            for (int b = 0; b < n; ++b) {
                if (b == L.s) continue;
                pq.push({L.t + rideSeconds(L.r, L.s, b), 1, L.r, b, L.changes, L.firstR, L.firstA, L.firstB < 0 ? b : L.firstB});
            }
            continue;
        }
        // stepped off at L.s: walk to the destination, or change
        const std::vector<int>& ends = nearTo[static_cast<std::size_t>(L.r)];
        if (std::find(ends.begin(), ends.end(), L.s) != ends.end()) {
            anyReach = true;
            const Real t = L.t + walkT(route.stops[static_cast<std::size_t>(L.s)].pos, to);
            if (t < bestT) {
                bestT = t;
                best.route = L.firstR; best.fromStop = L.firstA; best.toStop = L.firstB;
                bestIsTransfer = L.changes > 0;
            }
        }
        if (L.changes >= kMaxChanges) continue;
        for (const Transfer& x : transfers_)
            if (x.fromRoute == L.r && x.fromStop == L.s)
                pq.push({L.t + x.walk * kStreetFactor / kWalkPace + kTransferPenalty + waitSeconds(x.toRoute), 0, x.toRoute, x.toStop,
                         L.changes + 1, L.firstR, L.firstA, L.firstB});
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
