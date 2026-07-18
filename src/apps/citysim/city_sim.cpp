#include "city_sim.h"

#include "traffic_rules.h"   // approachStop
#include "../../engine/ai/idm.h"   // IDM car-following (roads-v2 S7)

#include <algorithm>
#include <cmath>
#include <cstdio>    // RT_CRASH_DEBUG contact telemetry
#include <cstdlib>
#include <limits>
#include <unordered_map>

namespace citysim {

using engine::Vec2;
using engine::NavGraph;

namespace {
constexpr Real kWalkSpeed = 1.4;
constexpr Real kCarAccel = 6.0;
constexpr Real kPedAccel = 1.0;
constexpr Real kCarMinGap = 5.0, kCarSlowZone = 14.0;
constexpr Real kPedMinGap = 0.8, kPedSlowZone = 2.5;
constexpr Real kCarBumperGap = 0.8;   // clear space kept between two cars' bumpers

// The fleet of body slots (ADR-0061 Phase 4). A sedan is the player's body
// exactly (4.2 long) and its follow gap works out to the historical 5.0 m; larger
// bodies keep proportionally more room. Lengths are all >= the sedan, so adding
// them never lets cars pack tighter than before. Order is mirrored by the render
// fleet (city_render.cpp kCarVariants), slot for slot.
const VehicleBody kFleet[] = {
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {5.2, 1.95, 1.60, VehicleType::Pickup},
    {5.4, 2.00, 2.10, VehicleType::Van},
    {6.6, 2.40, 2.80, VehicleType::BoxTruck},
};
constexpr int kFleetSize = static_cast<int>(sizeof(kFleet) / sizeof(kFleet[0]));
constexpr Real kJunctionApproach = 9.0, kJunctionSpeed = 4.0;
constexpr Real kSignalApproach = 14.0;          // start braking for a light this far out
constexpr Real kCarDecel = 6.0, kPedDecel = 3.0;
constexpr Real kLayerClearance = 5.8;   // bridge-deck height per grade layer
constexpr Real kCarMinTurnRadius = 6.0; // tightest arc a car can trace (m)
constexpr Real kPedVisionRange = 4.5;      // how far ahead a walker perceives (m)
constexpr Real kPedVisionHalfAngle = 1.2;  // ~69 deg to each side (wide peripheral)
constexpr Real kPedMaxLateral = 1.6;       // furthest a walker leans off its path (m)
constexpr Real kPedLateralRate = 1.6;      // how fast that lean changes (m/s) — smooth, not a pop
constexpr Real kPedBodyMin = 0.5;          // hard floor: bodies never closer than this
constexpr Real kPoleClearance = 0.7;       // a walker keeps its centre this far from a pole
constexpr Real kPlayerClearance = 1.1;     // ...and this far from the PLAYER (a wide berth,
                                           // so a near miss is a step-around, not a brush)
constexpr Real kPedClearance = 4.0;     // a car aims to stop this far short of a ped/player
constexpr Real kPedHardStop = 3.0;      // and will NOT roll closer than this (a real wall)
// The zebra band is painted 0.5..3.6 m past the junction MOUTH (road-texture
// shaders, ADR-0062); a car held at a red must stop with its BUMPER short of
// that band — not at the node, which put a legally-waiting car visually in the
// middle of the intersection, on top of the crosswalk, looking like it ran the
// light (device round 3).
constexpr Real kCrosswalkFarEdge = 3.6;   // band's far edge past the mouth (m)
constexpr Real kStopLineMargin = 0.6;     // bumper clearance short of the band
// The cognition loop (ADR-0063): agents act on MEMORY, not on the momentary
// snapshot. A track must hold at least this much confidence to be acted on —
// with the default 4 s memory horizon that means a body is reacted to for ~3 s
// after it was last actually seen (extrapolated along where it was heading).
constexpr Real kMemoryActConfidence = 0.25;
constexpr Real kTtcHorizon = 2.0;       // brake when a collision is predicted this soon (s)
constexpr Real kCollisionRadius = 1.5;  // car-vs-person combined disc radius for that test
constexpr Real kPedAnticipation = 0.4;  // walkers dodge where a neighbour WILL be (s ahead)

// Lane spacing that fits THIS road: split the right half of the carriageway
// evenly among the direction's lanes, so a car sits centred in its own lane and
// clearly on its (right-hand) side. A fixed lane width instead leaves a car
// hugging the centreline on a wide road, which reads as driving on the wrong
// side. laneCenter places lane i at (0.5 + i) * spacing off the centreline.
Real laneSpacing(const engine::NavLink& l) {
    int lanes = l.lanes < 1 ? 1 : l.lanes;
    return (l.oneWay ? l.width : l.width * 0.5) / static_cast<Real>(lanes);
}

// Speed a follower may travel given the centre-to-centre gap to its leader.
Real followCap(Real freeSpeed, Real gap, Real minGap, Real slowZone) {
    if (gap <= minGap) return 0.0;
    if (gap >= slowZone) return freeSpeed;
    return freeSpeed * (gap - minGap) / (slowZone - minGap);
}

// Rotate unit vector `from` toward unit vector `to` by at most `maxRad` radians.
Vec2 rotateToward(Vec2 from, Vec2 to, Real maxRad) {
    Real fl = std::sqrt(from.x * from.x + from.y * from.y);
    Real tl = std::sqrt(to.x * to.x + to.y * to.y);
    if (fl < 1e-9) return to;
    if (tl < 1e-9) return from;
    from = Vec2(from.x / fl, from.y / fl);
    to = Vec2(to.x / tl, to.y / tl);
    Real dot = from.x * to.x + from.y * to.y;
    if (dot > 1) dot = 1; else if (dot < -1) dot = -1;
    Real ang = std::acos(dot);
    if (ang <= maxRad) return to;        // close enough: snap onto target
    Real sign = (from.x * to.y - from.y * to.x) >= 0 ? 1.0 : -1.0;
    Real a = maxRad * sign;
    Real ca = std::cos(a), sa = std::sin(a);
    return Vec2(from.x * ca - from.y * sa, from.x * sa + from.y * ca);
}

// Personality traits read from FIXED bits of an agent's `brain` (ADR-0062): no
// rng draw, so seeded build streams are unchanged. Each helper owns its bits.

// Personal pace in [0.85, 1.15] of the nominal speed — owns brain bits 9..23.
Real paceFactor(uint32_t brain) {
    return 0.85 + 0.30 * (((brain >> 9) & 0x7FFF) / Real(0x7FFF));
}

// Think-clock stagger as a 0..1 fraction of the think period — owns bits 17..24.
Real thinkStagger(uint32_t brain) {
    return ((brain >> 17) & 0xFF) / Real(0xFF);
}

// Park/idle setback slot (0..7) spacing same-node agents along the verge — owns
// bits 3..5.
Real parkSetbackSlot(uint32_t brain) {
    return static_cast<Real>((brain >> 3) & 7);
}

// Post-fender-bender freeze (1.0..2.5 s, staggered so tangles unwind car by
// car) — owns bits 5..6.
Real crashHoldSeconds(uint32_t brain) {
    return 1.0 + static_cast<Real>((brain >> 5) & 3) * 0.5;
}
}  // namespace

int vehicleFleetSize() { return kFleetSize; }

const VehicleBody& vehicleFleetBody(int slot) {
    int s = ((slot % kFleetSize) + kFleetSize) % kFleetSize;   // wrap, handle negatives
    return kFleet[s];
}

Real CitySim::vehicleLength(int agentIndex) const {
    const Agent& a = agents_[agentIndex];
    if (a.mode == Agent::Mode::Driver && a.vehicle >= 0 &&
        a.vehicle < static_cast<int>(vehicles_.size()))
        return vehicles_[a.vehicle].length;
    return kPedBodyMin;   // a walker's footprint along its path
}

Real CitySim::pairMinGap(int follower, int leader) const {
    if (agents_[follower].mode == Agent::Mode::Pedestrian) return kPedMinGap;
    // Bumper-to-bumper: half of each body plus a clear buffer. Sedan-to-sedan =
    // 2.1 + 2.1 + 0.8 = 5.0, the historical constant, so all-sedan traffic is
    // unchanged; a longer body simply demands (and is granted) more room.
    return 0.5 * vehicleLength(follower) + 0.5 * vehicleLength(leader) + kCarBumperGap;
}

Real CitySim::junctionRadius(int node) const {
    if (node < 0 || node >= static_cast<int>(nodeBoxRadius_.size())) return 0;
    return nodeBoxRadius_[node];
}

bool CitySim::nearJunction(Vec2 pos, Real margin) const {
    for (const auto& j : junctions_) {
        Real r = j.second + margin;
        if ((pos - j.first).lengthSquared() <= r * r) return true;
    }
    return false;
}

std::vector<Vec2> CitySim::lanePath(int agentIndex, Real step) const {
    std::vector<Vec2> out;
    if (!nav_ || agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
        return out;
    const Agent& a = agents_[agentIndex];
    if (step < 0.5) step = 0.5;
    for (int li : a.route.links) {
        const engine::NavLink& L = nav_->links[li];
        Real spacing = laneSpacing(L);
        int n = std::max(1, static_cast<int>(std::ceil(L.length / step)));
        int lane = std::min(a.lane, std::max(1, L.lanes) - 1);
        for (int k = 0; k <= n; ++k) {
            Real t = static_cast<Real>(k) / n;
            Vec2 p = (a.mode == Agent::Mode::Driver)
                         ? nav_->laneCenter(li, lane, t, spacing)
                         : nav_->sidewalkPoint(li, t);
            if (out.empty() || (p - out.back()).length() > 1e-6) out.push_back(p);
        }
    }
    return out;
}

uint32_t CitySim::rnd() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
}
Real CitySim::rndUnit() { return (rnd() >> 8) * (1.0 / 16777216.0); }

Real CitySim::brainUnit(Agent& a) {
    a.brain ^= a.brain << 13;
    a.brain ^= a.brain >> 17;
    a.brain ^= a.brain << 5;
    return (a.brain >> 8) * (1.0 / 16777216.0);
}

void CitySim::build(const NavGraph& graph, int driverCount, int pedCount, uint32_t seed) {
    nav_ = &graph;
    agents_.clear();
    vehicles_.clear();
    sensed_.clear();
    externalObstacles_.clear();   // a REBUILD must not keep the previous level's
    staticObstacles_.clear();     // player/pole positions as phantom obstacles
    clockHours_ = 8.5;   // start mid morning-rush so agents commute right away
    simSeconds_ = 0;
    faultCount_ = 0;
    rng_ = seed ? seed : 0x6c078965u;
    ids_.reset();   // fresh scene → UIDs restart at 0 (reproducible from seed)
    signals_.build(graph);
    // Goal tables (ADR-0064): a rebuild resets to the built-ins matching the
    // (persistent) wander flag; a host with custom tables re-installs after.
    goalPed_ = wander_ ? wanderGoals(false) : defaultScheduleGoals();
    goalDriver_ = wander_ ? wanderGoals(true) : defaultScheduleGoals();

    const int n = graph.nodeCount();
    if (n == 0) return;

    // Per-node box radius (widest incident half-width) — every junction rule and
    // the launch clearance read this via junctionRadius(). Junction boxes (centre
    // + radius) additionally feed the bridge's don't-block-the-box and
    // spawn-placement checks (nearJunction).
    junctions_.clear();
    nodeBoxRadius_.assign(n, 0.0);
    for (int v = 0; v < n; ++v) {
        Real r = 0;
        // The box must FIT its local geometry: a curvy crossing gets sampled into
        // a KNOT of nodes a few metres apart, and a half-width-sized box there
        // (arterial: 6.5 m vs 4.5 m links) swallows the neighbouring nodes' stop
        // lines — every held car sits inside someone else's box, a circular wait,
        // permanent gridlock (device: "cars all stuck at one intersection"). Cap
        // the radius at 60% of the shortest incident link so adjacent boxes can
        // never overlap; open grids (80 m links) are untouched.
        Real shortestLink = 1e9;
        for (int li : graph.outLinks[v]) {
            r = std::max(r, graph.links[li].width * 0.5);
            shortestLink = std::min(shortestLink, graph.links[li].length);
        }
        if (shortestLink < 1e9) r = std::min(r, std::max(Real(1.5), shortestLink * 0.6));
        nodeBoxRadius_[v] = r;
        if (graph.isJunction(v)) junctions_.push_back({graph.nodes[v], r});
    }

    const int total = driverCount + pedCount;
    agents_.reserve(total);
    vehicles_.reserve(driverCount);
    for (int i = 0; i < total; ++i) {
        Agent a;
        a.uid = ids_.next();   // stable identity for relationships / memory / jobs
        a.mode = (i < driverCount) ? Agent::Mode::Driver : Agent::Mode::Pedestrian;
        a.home = static_cast<int>(rnd() % n);
        a.work = static_cast<int>(rnd() % n);
        // Pick a work node the home can actually REACH. An unroutable pair used to
        // teleport the agent to its goal on departure (it "disappeared" and
        // reappeared); instead insist on a routable pair, and if none turns up
        // just keep the agent home (work == home -> it never commutes).
        {
            // BOTH directions must route (the graph is directed — one-way ramps):
            // a valid home->work with no work->home used to retry a failing A*
            // every single tick once the agent wanted to come home.
            auto commutable = [&](int h, int w) {
                return w != h && engine::findRoute(graph, h, w).valid() &&
                       engine::findRoute(graph, w, h).valid();
            };
            bool ok = commutable(a.home, a.work);
            for (int tries = 0; tries < 8 && !ok && n > 1; ++tries) {
                a.work = static_cast<int>(rnd() % n);
                ok = commutable(a.home, a.work);
            }
            if (!ok) a.work = a.home;   // stranded: stay put, never depart
        }
        a.departWork = 7.5 + rndUnit() * 1.5;
        a.departHome = 16.5 + rndUnit() * 1.5;
        a.activity = Agent::Activity::AtHome;
        a.goal = goalsFor(a.mode).entry();   // the day starts at the table's entry
        a.brain = rnd() | 1u;            // per-agent fault RNG (non-zero)
        // Personality from the brain's own bits (NOT an rnd() draw, so the build
        // stream — and every seeded test scenario — is unchanged).
        a.speedFactor = paceFactor(a.brain);
        // Stagger the think clocks so the crowd doesn't re-decide in lockstep.
        a.thinkTimer = thinkPeriod_ * thinkStagger(a.brain);
        a.restNode = a.home;
        a.pos = idlePose(a.home, a.mode, a.brain);
        // Idle bodies face along their road from the start — a parked car with
        // the default (1,0) heading sat at a random angle to the verge it was
        // parked on, which read as "failed to be placed" (device round 3).
        if (!graph.outLinks[a.home].empty())
            a.heading = graph.direction(graph.outLinks[a.home][0]);

        // A driver possesses a freshly-created car (two-way possession link). Its
        // body comes from the fleet slot matching its index, so the renderer (which
        // maps the same index to a body) draws exactly this shape + size.
        if (a.mode == Agent::Mode::Driver) {
            SimVehicle v;
            const VehicleBody& body = vehicleFleetBody(static_cast<int>(vehicles_.size()));
            v.length = body.length;
            v.width = body.width;
            v.height = body.height;
            v.type = body.type;
            v.driver = static_cast<int>(agents_.size());
            v.pos = a.pos;
            a.vehicle = static_cast<int>(vehicles_.size());
            vehicles_.push_back(v);
        }
        agents_.push_back(a);
    }
}

void CitySim::assignPlaces(const PlaceMap& places, const NavGraph& graph) {
    relationships_.clear();
    for (Agent& a : agents_) { a.homePlace = kNoPlace; a.workPlace = kNoPlace; }
    if (places.empty() || graph.nodeCount() == 0) return;

    const std::vector<PlaceId>& homes = places.ofType(PlaceType::Home);
    const std::vector<PlaceId>& parks = places.ofType(PlaceType::Park);
    // A "job" is a workplace place: shop / office / civic (a park is not a job).
    std::vector<PlaceId> jobs;
    for (PlaceType t : {PlaceType::Shop, PlaceType::Office, PlaceType::Civic})
        for (PlaceId id : places.ofType(t)) jobs.push_back(id);
    if (homes.empty()) return;   // nowhere to live → leave the built schedule alone

    // The nav node a place routes through (nearest to its snapped entrance).
    auto nodeOf = [&](PlaceId id) { return graph.nearestNode(places[id].entrance); };
    auto commutable = [&](int h, int w) {
        return w != h && engine::findRoute(graph, h, w).valid() &&
               engine::findRoute(graph, w, h).valid();
    };

    // A place's DOORSTEP: its sidewalk entrance nudged toward the building site,
    // so a resting body stands at the door — clearly off the carriageway.
    auto doorOf = [&](PlaceId id) {
        const Place& p = places[id];
        return p.entrance + (p.site - p.entrance) * 0.4;
    };

    for (Agent& a : agents_) {
        // Home: deterministic pick from the agent's own brain bits (no rng draw).
        PlaceId hp = homes[a.brain % homes.size()];
        int hn = nodeOf(hp);
        a.homePlace = hp;
        a.home = hn;
        a.restNode = hn;
        a.homeDoor = doorOf(hp);
        // Walkers start the day at their door (indoors); cars stay parked on the
        // verge (idlePose) — a car "at home" is a car parked outside it.
        a.pos = a.mode == Agent::Mode::Pedestrian ? a.homeDoor
                                                  : idlePose(hn, a.mode, a.brain);
        if (!graph.outLinks[hn].empty())
            a.heading = graph.direction(graph.outLinks[hn][0]);

        // Role (Phase 4) decides how the day runs. ~1 in 5 (when the city has a
        // park) is a Stroller — no job, a day out at the park; the rest hold a job
        // and are a Shopkeeper (if their workplace is a shop) or a Commuter. All
        // deterministic from the brain's bits (no rng draw).
        a.work = a.home;
        a.role = Agent::Role::Commuter;
        const uint32_t roleRoll = (a.brain >> 20) & 0xFF;   // 0..255
        const bool stroller = !parks.empty() && roleRoll < 51;   // ~20%
        auto unit = [](uint32_t bits) { return (bits & 0xFF) / 255.0; };

        if (stroller) {
            PlaceId pk = parks[(a.brain >> 4) % parks.size()];
            if (commutable(hn, nodeOf(pk))) {
                a.role = Agent::Role::Stroller;
                a.work = nodeOf(pk);              // a daytime destination, not a job
                a.workPlace = kNoPlace;
                a.workDoor = doorOf(pk);          // rests at the park, not the kerb
                a.departWork = 9.5 + 2.0 * unit(a.brain);         // late-morning out
                a.departHome = 15.5 + 1.5 * unit(a.brain >> 8);   // home mid-afternoon
            }
        } else if (!jobs.empty()) {
            // Prefer the brain-picked workplace; if it isn't routable from home,
            // scan for any that is; if none, the agent works from home.
            PlaceId pick = jobs[(a.brain >> 8) % jobs.size()];
            if (!commutable(hn, nodeOf(pick))) {
                pick = kNoPlace;
                for (PlaceId cand : jobs)
                    if (commutable(hn, nodeOf(cand))) { pick = cand; break; }
            }
            if (pick != kNoPlace) {
                a.workPlace = pick;
                a.work = nodeOf(pick);
                a.workDoor = doorOf(pick);
                if (places[pick].type == PlaceType::Shop) {
                    // Shopkeeper: open the shop before it opens, close it after.
                    a.role = Agent::Role::Shopkeeper;
                    Real open = places[pick].openHour, close = places[pick].closeHour;
                    a.departWork = open > 0.5 ? open - 0.5 : 0.0;
                    a.departHome = close < 23.5 ? close + 0.5 : 24.0;
                }
                // else Commuter — keep the jittered office hours from build().
            }
        }
    }

    // Seed the surface-level social graph: agents sharing a workplace are
    // coworkers; those sharing a home are neighbors (housemates). Insertion order.
    for (std::size_t i = 0; i < agents_.size(); ++i)
        for (std::size_t j = i + 1; j < agents_.size(); ++j) {
            const Agent& A = agents_[i];
            const Agent& B = agents_[j];
            if (A.workPlace != kNoPlace && A.workPlace == B.workPlace)
                relationships_.set(A.uid, B.uid, Relationship::Coworker);
            else if (A.homePlace != kNoPlace && A.homePlace == B.homePlace)
                relationships_.set(A.uid, B.uid, Relationship::Neighbor);
        }
}

Vec2 CitySim::idlePose(int node, Agent::Mode mode, uint32_t brain) const {
    if (!nav_ || node < 0 || node >= nav_->nodeCount()) return Vec2();
    const std::vector<int>& out = nav_->outLinks[node];
    if (out.empty()) return nav_->nodes[node];
    int li = out[0];
    Vec2 dir = nav_->direction(li);
    Vec2 right(dir.y, -dir.x);
    Real hw = nav_->links[li].width * 0.5;
    // Both idle OFF the carriageway: a pedestrian just beyond the kerb, a driver
    // parked on the verge (ADR-0062 — an idle car is a PHYSICAL body now; parked
    // in lane 0 it blocked the road until its first trip).
    Real off = (mode == Agent::Mode::Driver) ? hw + 2.8 : hw + 1.2;
    if (off < 0.5) off = 0.5;
    // Per-agent setback ALONG the road (mirrors arrival parking): several agents
    // sharing a home node must not stack in one spot — two cars occupying the
    // same verge pose spawned interpenetrating the moment both departed.
    Real back = 2.0 + parkSetbackSlot(brain) *
                          ((mode == Agent::Mode::Driver) ? 1.4 : 0.6);
    back = std::min(back, nav_->links[li].length * 0.45);
    return nav_->nodes[node] + dir * back + right * off;
}

// Pick a random routable wander goal from `from` and start the trip. Prefers a
// goal that doesn't begin by U-TURNING back up the link just arrived on (a
// standstill 180 flips the lane offset to the other side of the road — a
// visible pop); falls back to a U-turn when nothing else routes (dead ends).
bool CitySim::startWanderTrip(Agent& a, int from, bool fromRest) {
    const int n = nav_ ? nav_->nodeCount() : 0;
    if (n <= 1 || from < 0 || from >= n) return false;
    auto reversesArrival = [&](const engine::Route& r) {
        if (a.arrivedLink < 0 || r.links.empty()) return false;
        const engine::NavLink& in = nav_->links[a.arrivedLink];
        const engine::NavLink& out = nav_->links[r.links.front()];
        return out.from == in.to && out.to == in.from;
    };
    // Scan EVERY node from a random start, so a non-reversing goal is found
    // whenever one exists — dice rolls occasionally picked only U-turn goals,
    // and each of those flipped the car to the other side of the road in place.
    int start = static_cast<int>(rnd() % static_cast<uint32_t>(n));
    int fallback = -1;
    for (int k = 0; k < n; ++k) {
        int goal = (start + k) % n;
        if (goal == from) continue;
        engine::Route r = engine::findRoute(*nav_, from, goal,
                                            a.mode == Agent::Mode::Pedestrian);
        if (!r.valid()) continue;
        if (reversesArrival(r)) { if (fallback < 0) fallback = goal; continue; }
        startTrip(a, from, goal, fromRest);
        return a.moving;
    }
    if (fallback >= 0) {
        startTrip(a, from, fallback, fromRest);
        return a.moving;
    }
    return false;
}

// --- the GOAL layer (ADR-0064) ----------------------------------------------
// An agent's day is a GoalTable (city_goals.h): states that each bind one small
// C++ action, transitions on the events emitted below. The built-in tables
// mirror the historical schedule/wander control flow bit-exactly — same rng
// draws in the same order, same tick the transitions fire; a scripting build
// may replace them at load (setGoalTables), and every per-tick transition still
// executes right here, in C++.

// Where a rest departure starts from: the node the agent last arrived at, or
// its home before the first trip. Matches the historical schedule origins too —
// an agent resting AtHome has restNode == home, one AtWork has restNode == work.
int CitySim::departNode(const Agent& a) const {
    return (nav_ && a.restNode >= 0 && a.restNode < nav_->nodeCount()) ? a.restNode
                                                                       : a.home;
}

// Execute the agent's current (GoTo) goal state: start the trip toward its
// target. On failure the table's NoRoute row (if any) decides the fallback
// state — the historical "no path: fall back to the origin's resting state".
bool CitySim::startGoalTrip(Agent& a, int origin, bool fromRest) {
    const GoalState& s = goalsFor(a.mode).state(a.goal);
    bool started = false;
    switch (s.target) {
        case GoalTarget::Random:
            started = startWanderTrip(a, origin, fromRest);
            break;
        case GoalTarget::Work:
        case GoalTarget::Home:
            startTrip(a, origin, s.target == GoalTarget::Work ? a.work : a.home,
                      fromRest);
            started = a.moving;
            break;
        default:
            break;   // a GoTo with no target: nothing to do
    }
    if (started) {
        a.activity = s.activity;   // the day now reads as this state's label
        return true;
    }
    int next = goalsFor(a.mode).onEvent(a.goal, GoalEvent::NoRoute);
    if (next >= 0) {
        a.goal = next;
        a.goalHours = 0;
        a.activity = goalsFor(a.mode).state(next).activity;
    }
    return false;
}

// Deliver one event: take the first table row that matches the agent's state.
// A row into a GoTo state is launch-gated for drivers (a car departs only once
// its spawn area is clear of moving traffic — it waits out the traffic and the
// event simply fires again next tick, like anyone pulling out of a spot).
CitySim::GoalFire CitySim::tryGoalEvent(Agent& a, GoalEvent event) {
    const GoalTable& t = goalsFor(a.mode);
    int next = t.onEvent(a.goal, event);
    if (next < 0) return GoalFire::NoRow;
    const GoalState& to = t.state(next);
    if (to.action == GoalAction::GoTo) {
        int from = departNode(a);
        if (a.mode == Agent::Mode::Driver && !launchClear(a, from))
            return GoalFire::Blocked;
        a.goal = next;
        a.goalHours = 0;
        startGoalTrip(a, from, /*fromRest=*/true);
    } else {
        a.goal = next;
        a.goalHours = 0;
        a.activity = to.activity;
    }
    return GoalFire::Fired;
}

// step()'s pass 1 for one agent: run its current goal state. A GoTo state whose
// trip isn't running retries the departure each tick; a Rest state emits this
// tick's events in a fixed order — DwellDone, the clock-window depart, Idle —
// and the first row that fires (or blocks on launch clearance) wins.
void CitySim::goalThink(Agent& a, Real dtHours) {
    const GoalTable& t = goalsFor(a.mode);
    if (a.goal < 0 || a.goal >= t.stateCount()) return;   // no table: inert
    const GoalState& s = t.state(a.goal);
    if (s.action == GoalAction::GoTo) {
        if (!a.moving) {
            int from = departNode(a);
            if (a.mode != Agent::Mode::Driver || launchClear(a, from))
                startGoalTrip(a, from, /*fromRest=*/true);
        }
        return;
    }
    // Rest: the dwell clock runs on in-world hours, like the schedule windows.
    a.goalHours += dtHours;
    if (s.dwellHours > 0 && a.goalHours >= s.dwellHours)
        if (tryGoalEvent(a, GoalEvent::DwellDone) != GoalFire::NoRow) return;
    // The commute clock: inside the [departWork, departHome) window the day
    // says "be at work", outside it "be at home". Only meaningful for agents
    // with a real commute — a stranded pair (work == home, no route at build)
    // never departs, exactly as before.
    if (a.home != a.work) {
        bool inWindow = clockHours_ >= a.departWork && clockHours_ < a.departHome;
        if (tryGoalEvent(a, inWindow ? GoalEvent::DepartWork
                                     : GoalEvent::DepartHome) != GoalFire::NoRow)
            return;
    }
    tryGoalEvent(a, GoalEvent::Idle);
}

// Re-seat every agent onto the (new) tables: the first state wearing the
// agent's current activity label, else the entry state. Mid-trip agents keep
// driving — only their next transition consults the new table.
void CitySim::installGoalTables(GoalTable pedestrian, GoalTable driver) {
    goalPed_ = std::move(pedestrian);
    goalDriver_ = std::move(driver);
    for (Agent& a : agents_) {
        const GoalTable& t = goalsFor(a.mode);
        int mapped = t.entry();
        for (int s = 0; s < t.stateCount(); ++s)
            if (t.state(s).activity == a.activity) { mapped = s; break; }
        a.goal = mapped;
        a.goalHours = 0;
    }
}

void CitySim::setGoalTables(GoalTable pedestrian, GoalTable driver) {
    installGoalTables(std::move(pedestrian), std::move(driver));
}

void CitySim::setWander(bool on) {
    wander_ = on;
    installGoalTables(on ? wanderGoals(false) : defaultScheduleGoals(),
                      on ? wanderGoals(true) : defaultScheduleGoals());
}

// Is the spawn area at `node` free of moving cars? Agents departing from rest
// materialize on the lane there — launching under a crossing (or same-node)
// car spawned two bodies inside each other, which the crash rule then locked.
bool CitySim::launchClear(const Agent& a, int node) const {
    if (!nav_ || node < 0 || node >= nav_->nodeCount()) return true;
    const Vec2 p = nav_->nodes[node];
    // The radius must cover every place the launch could put the body: the lane
    // start at the node PLUS the junction-origin box skip (jr + 2 m down the
    // first link) — a 5 m check let a second car spawn exactly on a first one
    // that had been skipped 6 m out.
    Real clear = junctionRadius(node) + 2.0 + 6.0;
    for (const Agent& b : agents_) {
        if (&b == &a || b.mode != Agent::Mode::Driver) continue;
        if (!b.moving || b.released) continue;
        if (std::fabs(b.elevation - a.elevation) > 3.0) continue;
        Real dx = b.pos.x - p.x, dy = b.pos.y - p.y;
        if (dx * dx + dy * dy < clear * clear) return false;
    }
    return true;
}

void CitySim::startTrip(Agent& a, int origin, int goal, bool fromRest) {
    a.route = engine::findRoute(*nav_, origin, goal,
                                a.mode == Agent::Mode::Pedestrian);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    if (!a.route.valid()) {
        // No path: do NOT teleport to the goal (that was the "disappear/reappear"
        // bug). Stay parked at the ORIGIN so the agent simply doesn't take this
        // trip; the goal layer's NoRoute row decides where its day falls back to.
        a.moving = false;
        a.elevation = 0;
        a.pos = idlePose(origin, a.mode, a.brain);
        return;
    }
    int lanes = nav_->links[a.route.links.front()].lanes;
    a.lane = (a.mode == Agent::Mode::Driver && lanes > 1)
                 ? static_cast<int>(rnd() % static_cast<uint32_t>(lanes))
                 : 0;
    a.laneF = a.lane;
    a.laneTimer = 4.0 + (a.home % 11);
    ++a.trips;   // a new route: the pursuit bridge rebuilds its path off this
    a.moving = true;
    a.crashTimer = 0;    // a fresh trip carries no wreck state: without this an
    a.crashCount = 0;    // escapee that PARKED near its wreck departed hours
                         // later still contact-immune and ghosted through traffic
    // A REST departure whose origin is a junction starts a few metres down the
    // first link, past the box — materializing a parked car among crossing
    // traffic spawned collisions the crash rule then locked in place.
    if (fromRest && a.mode == Agent::Mode::Driver && nav_->isJunction(origin)) {
        Real L0 = nav_->links[a.route.links.front()].length;
        a.distOnLeg = std::min(junctionRadius(origin) + 2.0, L0 * 0.4);
    }
    refreshPose(a);
    a.heading = nav_->direction(a.route.links.front());   // start pointed down leg 0
}

void CitySim::refreshPose(Agent& a) {
    if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) return;
    int li = a.route.links[a.leg];
    Real L = nav_->links[li].length;
    Real s = a.distOnLeg;
    if (s < 0) s = 0; else if (s > L) s = L;

    // Sample this agent's own guide line (lane centre / sidewalk) on a link.
    auto sample = [&](int link, Real t) {
        if (t < 0) t = 0; else if (t > 1) t = 1;
        if (a.mode != Agent::Mode::Driver) return nav_->sidewalkPoint(link, t);
        const engine::NavLink& LL = nav_->links[link];
        const int lanes = std::max(1, LL.lanes);
        // FRACTIONAL lane (device: visible lane changes): laneF eases toward
        // a.lane, so the car glides across the dashes instead of teleporting.
        const Real flMax = Real(lanes - 1) +
            (LL.klass == engine::RoadClass::Freeway ? Real(1) : Real(0));
        const Real fl = std::min(std::max(a.laneF, Real(0)), flMax);
        const Real spacing = laneSpacing(LL);
        Real off = (0.5 + fl) * spacing;
        if (LL.oneWay) off -= lanes * 0.5 * spacing;
        // WAVER (device): a human hand is never perfectly still — a slow,
        // speed-scaled weave inside the lane. Deterministic per agent.
        const Real phase = (a.home * 2.399 + a.work * 1.117);
        off += 0.12 * std::sin(simSeconds_ * (0.5 + a.speed * 0.05) + phase) *
               std::min(Real(1), a.speed / 4.0);
        Vec2 cpt = nav_->pointOnLink(link, t);
        const Vec2 d = nav_->direction(link);
        return cpt + Vec2(d.y, -d.x) * off;
    };
    a.pos = sample(li, L > 1e-9 ? s / L : 0.0);
    {   // continuous carriageway height: corridor decks/ramps lerp their
        // node elevations along the link; layer keeps legacy bridges lifted
        const engine::NavLink& EL = nav_->links[li];
        const Real et = L > 1e-9 ? s / L : 0.0;
        if (EL.elevAbsolute) {
            a.deckY = EL.elevA + (EL.elevB - EL.elevA) * et;
            a.elevation = a.deckY;   // sensors gate on RELATIVE height; an
                                     // absolute deck vs a street reads > 3 m
        } else {
            a.deckY = -1e30;
            a.elevation = EL.layer * kLayerClearance +
                          EL.elevA + (EL.elevB - EL.elevA) * et;
        }
        a.grade = (EL.elevB - EL.elevA) / std::max(Real(1), EL.length);
    }

    // Corner-cut blending (device fix): the lane/sidewalk offset direction
    // rotates with each leg, so sampling only the CURRENT leg makes the pose
    // JUMP sideways the tick a node is crossed — on a wide arterial several
    // metres in one frame, a car visibly blinking across the turn. Near an
    // interior node, trace a quadratic Bezier from the incoming guide line to
    // the outgoing one instead: continuous through the corner (u = 0.5 exactly
    // at the leg change, both halves sampling the same curve), and it reads as
    // the arc the rate-limited heading was already pretending to drive.
    auto blendSpan = [&](int la, int lb) {
        Real B = std::max(nav_->links[la].width, nav_->links[lb].width) * 0.5 + 1.0;
        B = std::min(B, nav_->links[la].length * 0.45);
        B = std::min(B, nav_->links[lb].length * 0.45);
        return B;
    };
    auto corner = [&](int la, int lb, Real B, Real u) {   // u in [0,1] across the node
        Real La = nav_->links[la].length, Lb = nav_->links[lb].length;
        engine::Vec2 p0 = sample(la, La > 1e-9 ? (La - B) / La : 0.0);
        engine::Vec2 c = (sample(la, 1.0) + sample(lb, 0.0)) * 0.5;
        if (a.mode == Agent::Mode::Pedestrian) {
            // MITER the walker's corner (roads-v2 S8): averaging the two
            // guide endpoints CHORDS the offset envelope's arc, so on the
            // inside of a bend the blended path dipped ~1.5 m into the
            // carriageway — measured by the road-walk gate as 738 asphalt
            // ticks, all at degree-2 bends. Push the control point out to
            // the miter (offset / cos(halfAngle), clamped 2x — the same rule
            // the mesher's rings use). A car keeps the cut: arcing INTO the
            // junction on the carriageway is what driving through it is.
            const engine::Vec2 nodeP = nav_->nodes[nav_->links[la].to];
            const engine::Vec2 m = c - nodeP;
            const Real m2 = m.lengthSquared();
            const Real off = std::max((sample(la, 1.0) - nodeP).length(),
                                      (sample(lb, 0.0) - nodeP).length());
            if (m2 > 1e-9) {
                Real f = std::min((off * off) / m2, Real(4.0));
                c = nodeP + m * f;
            }
        }
        engine::Vec2 p2 = sample(lb, Lb > 1e-9 ? B / Lb : 1.0);
        Real v = 1.0 - u;
        return p0 * (v * v) + c * (2.0 * u * v) + p2 * (u * u);
    };
    const int legCount = static_cast<int>(a.route.links.size());
    if (a.leg + 1 < legCount) {              // exit half: approaching the node
        int nli = a.route.links[a.leg + 1];
        Real B = blendSpan(li, nli);
        if (B > 1e-6 && L - s < B)
            a.pos = corner(li, nli, B, (B - (L - s)) / (2.0 * B));
    }
    if (a.leg > 0) {                         // entry half: just past the node
        int pli = a.route.links[a.leg - 1];
        Real B = blendSpan(pli, li);
        if (B > 1e-6 && s < B)
            a.pos = corner(pli, li, B, 0.5 + s / (2.0 * B));
    } else if (wander_ && a.arrivedLink >= 0 &&
               nav_->links[a.arrivedLink].to == nav_->links[li].from) {
        // A CHAINED wander trip (arrival rolled straight into the next route):
        // blend leg 0 from the link the agent arrived along, exactly like an
        // interior corner — without this the restart popped onto the new lane
        // line, stacking simultaneous chainers on the node point.
        Real B = blendSpan(a.arrivedLink, li);
        if (B > 1e-6 && s < B)
            a.pos = corner(a.arrivedLink, li, B, 0.5 + s / (2.0 * B));
    }
}

// Turn the heading toward the current leg's direction. A pedestrian pivots
// freely; a car is rate-limited so its path curvature never tightens past
// kCarMinTurnRadius — at speed v it may yaw at most v / radius rad/s, which is
// what traces a smooth arc through a junction instead of an instant snap.
void CitySim::steer(Agent& a, Real dt) {
    if (a.leg >= static_cast<int>(a.route.links.size())) return;
    Vec2 desired = nav_->direction(a.route.links[a.leg]);
    if (a.mode != Agent::Mode::Driver) { a.heading = desired; return; }
    // Yaw rate is proportional to speed: at v the tightest arc is kCarMinTurnRadius,
    // so the car may turn at most v / radius rad/s. A stopped car cannot change
    // heading at all (like a real car) — it holds until it rolls, which also means
    // a car halted at a light never snaps its heading. Trip start seeds the initial
    // heading directly (startTrip), so a just-launched car is already aligned.
    Real rate = a.speed / kCarMinTurnRadius;
    a.heading = rotateToward(a.heading, desired, rate * dt);
}

// The junction rules of advance(): the stop-line geometry, the signal brake, and
// the box-occupancy / turn-yield scan. Returns the speed target after those caps
// plus the effective stop line the hard clamp in advance() holds at. Pure query:
// no rng draws, no agent mutation.
CitySim::JunctionGate CitySim::junctionSpeedCap(const Agent& a, int li,
                                                Real target) const {
    JunctionGate gate;
    bool car = a.mode == Agent::Mode::Driver;
    // Where this agent's signal STOP LINE sits, measured back from the node. A
    // walker holds at the corner; a car holds with its front bumper short of the
    // painted zebra band on its approach: junction box radius (the mouth) + the
    // band + a margin + half its own body. Stopping at the node itself parked a
    // legally-waiting car in the middle of the intersection (device round 3).
    Real stopSetback = 0.5;
    int toNode = nav_->links[li].to;
    if (car && nav_->isJunction(toNode)) {
        Real halfLen = 2.1;   // sedan fallback
        if (a.vehicle >= 0 && a.vehicle < static_cast<int>(vehicles_.size()))
            halfLen = vehicles_[a.vehicle].length * 0.5;
        stopSetback = junctionRadius(toNode) + kCrosswalkFarEdge + kStopLineMargin +
                      halfLen;
    }
    // The ONE effective stop line every junction rule agrees on: the setback
    // short of the node, but never behind the link start — a link shorter than
    // the setback gets a line partway down it, and the smooth brake, the yield
    // scan, and the hard clamp all use THIS. (Gating the rules on the raw
    // setback used to disable box occupancy + turn yield entirely on short
    // links — exactly where junctions are densest.)
    Real stopLinePos = 0;
    if (nav_->isJunction(toNode)) {
        Real L0 = nav_->links[li].length;
        stopLinePos = std::max(L0 - stopSetback, std::min(L0 - 0.5, L0 * 0.4));
    }
    bool yieldAtLine = false;   // a TURNING car holding for oncoming traffic
    if (nav_->isJunction(toNode)) {
        Real distToEnd = nav_->links[li].length - a.distOnLeg;
        if (car && distToEnd < kJunctionApproach) target = std::min(target, kJunctionSpeed);
        // Obey the stoplight: ease to a stop at the LINE unless this approach is
        // green. A pedestrian on a green approach crosses the PERPENDICULAR road
        // (which is then red), so the same condition is safe for cars and peds.
        Real distToLine = stopLinePos - a.distOnLeg;
        // Only cars still BEFORE the line brake for the signal — a car already
        // past it is COMMITTED and must clear the box (the all-red clearance
        // exists for exactly that). Capping a committed car to zero pinned it
        // mid-intersection every time a green expired under it.
        if (distToLine >= 0 && distToLine < kSignalApproach && signals_.hasSignal(li) &&
            signals_.stateForLink(li) != SignalState::Green) {
            target = std::min(target, approachStop(distToLine, car ? kCarDecel : kPedDecel,
                                                   target));
        }
        // HOLDS at the line (cars still before it, any signal state):
        //  1. BOX OCCUPANCY — never enter while a car on a CROSSING heading is
        //     inside the box (a lingerer from the last phase, a mid-turn car):
        //     entering anyway is where mid-box wrecks came from. Same-axis
        //     occupants (leaders ahead, opposing straight-through on the other
        //     side of the road) don't hold anyone.
        //  2. TURN YIELD — a TURN also waits for live ONCOMING approach traffic
        //     (opposing arms share a green; a left turn crosses it). Going
        //     straight never yields, so two held turners can't deadlock; two
        //     opposing stopped turners break the tie by agent order.
        if (car && distToLine >= 0 && distToLine < kSignalApproach &&
            a.leg < static_cast<int>(a.route.links.size())) {
            Vec2 d0 = nav_->direction(li);
            bool turning = false;
            if (a.leg + 1 < static_cast<int>(a.route.links.size())) {
                Vec2 d1 = nav_->direction(a.route.links[a.leg + 1]);
                turning = d0.x * d1.x + d0.y * d1.y < 0.85;   // a real bend
            } else if (wander_) {
                // Final leg in wander mode: arrival CHAINS straight through this
                // node onto an unknown next route — treat it as a turn so a
                // chained continuation yields to live oncoming traffic instead
                // of sweeping across it (the crash-at-the-junction generator).
                turning = true;
            }
            Vec2 jc = nav_->nodes[toNode];
            Real jr = junctionRadius(toNode);
            Real range = jr + 6.0;
            // Gridlock escape: held this long by nothing but STALLED occupants, a
            // real driver inches through the box. Staggered per agent (brain bits)
            // so a ring of mutual waiters releases one at a time, deterministic.
            const bool gridlocked =
                a.holdTimer > 6.0 + static_cast<Real>(a.brain % 4u) * 1.5;
            for (const Agent& b : agents_) {
                if (&b == &a || b.mode != Agent::Mode::Driver) continue;
                if (!b.moving || b.released) continue;
                if (std::fabs(b.elevation - a.elevation) > 3.0) continue;
                Real dx = b.pos.x - jc.x, dy = b.pos.y - jc.y;
                Real db2 = dx * dx + dy * dy;
                if (db2 > range * range) continue;
                Real along = b.heading.x * d0.x + b.heading.y * d0.y;
                // 1: crossing-heading occupant INSIDE the box proper. A stalled
                // occupant stops counting once this car is gridlocked — waiting
                // on a car that is itself waiting is how the circular wait forms.
                if (db2 < jr * jr && std::fabs(along) < 0.7) {
                    if (gridlocked && b.speed < 0.3) continue;
                    yieldAtLine = true;
                    break;
                }
                // 2: turn yield against oncoming approach traffic.
                if (!turning || along >= -0.3) continue;
                if (b.speed > 0.5) { yieldAtLine = true; break; }  // live traffic
                // A gridlocked car stops waiting on anything STALLED — including
                // the stopped-turner tie-break below, whose cross-junction chains
                // (A waits on B waits on C...) were the 400-second holds. Only
                // live moving traffic holds it now; the staggered release means
                // opposing turners still don't sweep simultaneously.
                if (gridlocked) continue;
                if (&b < &a && b.leg + 1 < static_cast<int>(b.route.links.size())) {
                    int bli = b.route.links[b.leg];
                    if (nav_->links[bli].to == toNode) {
                        Vec2 bd0 = nav_->direction(bli);
                        Vec2 bd1 = nav_->direction(b.route.links[b.leg + 1]);
                        if (bd0.x * bd1.x + bd0.y * bd1.y < 0.85) {
                            yieldAtLine = true;
                            break;
                        }
                    }
                }
            }
            if (yieldAtLine)
                target = std::min(target, approachStop(distToLine, kCarDecel, target));
        }
    }
    gate.cap = target;
    gate.stopLinePos = stopLinePos;
    gate.yieldAtLine = yieldAtLine;
    return gate;
}

// The perception block of advance() (ADR-0063: sense -> remember -> predict ->
// decide -> act). Each tick the car SENSES pedestrians and the player through its
// 2.5D sensor — the forward wedge plus a height band, so a walker crossing the
// overpass is not a phantom to brake for — and REMEMBERS the sightings as tracks
// with velocity estimates. It then acts on the MEMORY, not the snapshot: a person
// who slipped out of the cone is still yielded to where they're HEADING for a
// few seconds (object permanence), and a crosser on a collision course is
// braked for BEFORE entering the lane (time-to-collision on the tracked
// velocity). We still do NOT sense other AI cars: car-vs-car is resolved by
// lanes, same-lane car-following (advance()'s gap cap) and the signals —
// braking for every car in a wide cone deadlocked junctions (see step()).
// Imperfect: with probability (1 - reliability) this step's sighting is
// missed (a fault) — memory makes a missed frame degrade gracefully instead
// of blinding the car outright. Returns the distance to the nearest person (or
// predicted collision point) ahead; infinity when nothing demands a brake.
Real CitySim::senseAhead(Agent& a) {
    Real seenAhead = std::numeric_limits<Real>::infinity();   // dist to a person ahead
    engine::VisionCone cone;
    cone.origin = a.pos;
    cone.forward = a.heading;
    cone.range = 18.0;
    cone.halfAngleRad = 0.45;    // ~26 deg: a crosser in the lane ahead,
                                 // not someone standing on the far sidewalk
    if (brainUnit(a) <= a.reliability) {
        engine::SensorVolume sensor;
        sensor.cone = cone;
        for (const SensedGhost& g : sensed_) {
            // External points (the live player) carry no elevation from the
            // host — skip the height gate rather than invent one for them.
            Real dh = g.id < 0 ? 0.0 : g.elevation - a.elevation;
            if (engine::sees(sensor, g.pos, dh))
                a.memory.observe(g.id, g.pos, simSeconds_);
        }
    } else {
        ++faultCount_;
    }
    a.memory.update(simSeconds_);
    for (const engine::TrackedBody& t : a.memory.tracks()) {
        if (t.confidence < kMemoryActConfidence) continue;
        // Where memory says the body IS (extrapolated while unseen): yield
        // if that estimate sits in the corridor ahead.
        if (engine::sees(cone, t.pos)) {
            Real fd = engine::forwardDistance(cone, t.pos);
            if (fd > 0) seenAhead = std::min(seenAhead, fd);
        }
        // Where memory says it's GOING: if our courses collide within the
        // horizon, brake as if the collision point were an obstacle that
        // far ahead — anticipation, not reaction.
        Real ttc = engine::timeToCollision(a.pos, a.heading * a.speed,
                                           t.pos, t.vel, kCollisionRadius);
        if (ttc < kTtcHorizon)
            seenAhead = std::min(seenAhead, std::max(Real(0), a.speed * ttc));
    }
    return seenAhead;
}

void CitySim::advance(Agent& a, Real dt, Real gap, Real minGap) {
    if (!a.moving) return;
    int li = a.route.links[a.leg];
    bool car = a.mode == Agent::Mode::Driver;
    if (car) {
        // LANE LIFE (device: "see the cars change lanes ... signal with
        // their turn signals"): laneF EASES toward a.lane (~2 s per lane) so
        // the change is a visible glide the lamp bake can indicate; a paced
        // discretionary change picks a neighbouring lane now and then on any
        // multi-lane link. Deterministic: agent-keyed hash, sim-clock paced.
        const Real ease = dt * 0.55;
        const Real dl = Real(a.lane) - a.laneF;
        a.laneF += std::max(-ease, std::min(ease, dl));
        a.laneTimer -= dt;
        if (a.laneTimer <= 0) {
            const engine::NavLink& LL = nav_->links[li];
            const int lanes = std::max(1, LL.lanes);
            uint32_t h = static_cast<uint32_t>(a.home * 73 + a.work * 131 +
                                               a.trips * 17 + a.leg) *
                         2654435761u;
            a.laneTimer = 4.5 + (h >> 8) % 7;
            if (lanes > 1 && a.speed > 3.0 && std::fabs(dl) < 0.05) {
                // PURPOSEFUL lane choice (§9.6 device rules): fast drivers
                // work toward the median (lane 0), slow ones stay right; an
                // exit on the route within ~380 m pulls to the SLOW lane.
                // One ADJACENT step at a time, and only when there is room.
                int preferred = lanes - 1 -
                    std::min(lanes - 1,
                             std::max(0, static_cast<int>(
                                             (a.speedFactor - 0.97) * 25)));
                const int legCount = static_cast<int>(a.route.links.size());
                Real ahead = LL.length - a.distOnLeg;
                for (int lg = a.leg + 1; lg < legCount && ahead < 380.0; ++lg) {
                    const engine::NavLink& NL = nav_->links[a.route.links[lg]];
                    if (NL.klass == engine::RoadClass::Ramp) {
                        preferred = lanes - 1;   // exit ahead: work right
                        break;
                    }
                    if (NL.klass != engine::RoadClass::Freeway) break;
                    ahead += NL.length;
                }
                const int dir = (preferred > a.lane) - (preferred < a.lane);
                if (dir != 0) {
                    const int want = a.lane + dir;
                    // room check: nobody in the target lane alongside
                    bool room = true;
                    for (const Agent& b : agents_) {
                        if (&b == &a || b.mode != Agent::Mode::Driver ||
                            !b.moving || b.leg >= (int)b.route.links.size())
                            continue;
                        if (b.route.links[b.leg] != li) continue;
                        if (std::fabs(b.laneF - Real(want)) > 0.6) continue;
                        if (std::fabs(b.distOnLeg - a.distOnLeg) < 13.0) {
                            room = false;
                            break;
                        }
                    }
                    if (room) a.lane = want;
                }
            }
        }
    }
    // Nominal pace scaled by personality (ADR-0062): drivers and walkers each
    // hold their OWN fraction of the limit, so traffic doesn't move in lockstep.
    // Junction/signal caps are shared road rules and stay unscaled.
    Real target = (car ? engine::classSpeed(nav_->links[li].klass) : kWalkSpeed) *
                  a.speedFactor;
    Real accel = car ? kCarAccel : kPedAccel;
    // Junction rules (stop line, signal brake, box occupancy / turn yield) cap
    // the target; the gate carries the line the hard clamp below holds at.
    JunctionGate gate = junctionSpeedCap(a, li, target);
    target = gate.cap;
    // Perception: how far ahead the nearest person (or predicted collision) is;
    // ease to a stop kPedClearance short of it.
    Real seenAhead = std::numeric_limits<Real>::infinity();
    if (car) {
        seenAhead = senseAhead(a);
        if (seenAhead < 1e9)
            target = std::min(target, approachStop(seenAhead - kPedClearance, kCarDecel,
                                                   target));
    }

    // minGap is length-aware (computeGaps → pairMinGap): sedan traffic reproduces
    // the old 5.0 m, and a longer body keeps a bigger bumper gap so nothing packs
    // tighter than before. Peds fall back to the fixed footprint gap.
    if (car) {
        // IDM car-following (roads-v2 plan Â§3.1): one smooth law for free
        // acceleration toward the (signal/junction/perception-capped) target
        // and braking for the leader â the anti-pile-up backbone, replacing
        // the linear followCap ramp + instantaneous down-jump. The net gap is
        // bumper-to-bumper: centre gap minus the bodies (minGap carries them
        // plus kCarBumperGap, which doubles as IDM's jam gap s0).
        const int myIdx = static_cast<int>(&a - agents_.data());
        engine::IdmParams idm;
        idm.accelMax = accel;
        idm.decelComf = kCarDecel;
        idm.minGap = kCarBumperGap;
        Real netGap = gap < 1e8
            ? std::max(Real(0.05), gap - (minGap - kCarBumperGap))
            : std::numeric_limits<Real>::infinity();
        Real dv = gap < 1e8 ? a.speed - leaderSpeeds_[myIdx] : 0.0;
        // The vision wedge may know a NEARER body than the lane chain does
        // (a merger's nose, a wreck on another link). Whoever is closest is
        // the leader IDM brakes for. A gridlock-creeping car ignores the
        // wedge: the creep exists to unpick stalemates, and the fender-bender
        // rule arbitrates any resulting brush.
        if (myIdx < static_cast<int>(carAheadGap_.size()) &&
            carAheadGap_[myIdx] < netGap && a.holdTimer <= 6.0) {
            netGap = carAheadGap_[myIdx];
            dv = a.speed - carAheadSpeed_[myIdx];
        }
        a.speed = engine::idmStep(a.speed, std::max(Real(0.1), target), netGap,
                                  dv, dt, kCarDecel * 2.0, idm);
        // The capped target is still a hard ceiling (signals/junction rules
        // must bind immediately, not on the IDM relaxation curve).
        a.speed = std::min(a.speed, std::max(Real(0), target));
    } else {
        Real slowZone = kPedSlowZone;
        target = followCap(target, gap, minGap, slowZone);
        a.speed = std::min(target, a.speed + accel * dt);
    }

    // Gridlock clock + escape. A jam's terminal form is a RING: cars pinned
    // nose-to-nose across adjacent junctions by the follow gap (each behind the
    // next stalled car), which no yield rule will ever release. So ANY car pinned
    // to a standstill near a junction — by a yield or by a stalled leader, but
    // never by a red (the light will change) — counts its hold; once gridlocked,
    // it stops yielding to stalled traffic (junctionSpeedCap) and CREEPS, so the
    // ring inches apart the way real drivers unpick a blocked box. If bodies
    // brush during the creep, the fender-bender freeze-and-recover arbitrates.
    if (car) {
        const int toNode = nav_->links[li].to;
        const bool redAhead = signals_.hasSignal(li) &&
                              signals_.stateForLink(li) != SignalState::Green;
        const bool nearJunc =
            nav_->isJunction(toNode) &&
            nav_->links[li].length - a.distOnLeg < kSignalApproach + 12.0;
        if (a.speed < 0.05 && nearJunc && !redAhead && a.crashTimer <= 0)
            a.holdTimer += dt;
        else if (a.speed > 1.5)   // above creep: genuinely rolling again — a
            a.holdTimer = 0;      // reset at creep speed sawtoothed the escape
        const bool gridlocked =
            a.holdTimer > 6.0 + static_cast<Real>(a.brain % 4u) * 1.5;
        if (gridlocked && !redAhead && a.crashTimer <= 0 && a.speed < 0.7)
            a.speed = 0.7;   // creep — the hard line clamp below still applies
    }

    // Hard stop line at a red light: the smooth cap above slows the agent but
    // never to exactly zero, so without this the leftover motion would carry it
    // THROUGH the light. Clamp advance so it cannot pass the line while its
    // approach is not green; it waits here until the signal clears.
    {
        int toNode = nav_->links[li].to;
        bool redAhead = signals_.hasSignal(li) &&
                        signals_.stateForLink(li) != SignalState::Green;
        if (nav_->isJunction(toNode) && (redAhead || gate.yieldAtLine)) {
            // The hold applies only BEFORE the line. A car already past it is
            // committed to the box and drives on — clamping it there trapped it
            // mid-intersection whenever a green expired under it, and everything
            // arriving cross-phase piled into it.
            if (a.distOnLeg <= gate.stopLinePos + 1e-6) {
                Real room = std::max(Real(0), gate.stopLinePos - a.distOnLeg);
                Real motion = std::min(a.speed * dt, room);
                if (motion < a.speed * dt) a.speed = 0;   // held at the line
                a.distOnLeg += motion;
                // FSM: red = Waiting; holding a turn for oncoming = Yielding.
                // A pedestrian held at the kerb is Waiting too (honest red ring).
                a.state = car ? (redAhead ? Agent::State::Waiting : Agent::State::Yielding)
                              : Agent::State::Waiting;
                refreshPose(a);
                steer(a, dt);
                return;
            }
        }
    }

    // S8 KERB DISCIPLINE (roads-v2 plan §3.3): at an UNSIGNALLED junction a
    // walker GAP-ACCEPTS — it waits at the kerb while cars are inbound on the
    // crossing and steps off into a gap, instead of walking out and relying
    // on every driver's brakes (the signalled case is already the stop-line
    // hold above). Assertiveness cap: after ~12 s of no gap the walker
    // crosses anyway — traffic never fully starves a pedestrian, and the
    // cars' person-sense stays the safety net, exactly like a real kerb.
    if (!car) {
        const int toNode = nav_->links[li].to;
        const Real toEnd = nav_->links[li].length - a.distOnLeg;
        // The kerb line sits BEFORE the corner-cut blend window (half the
        // road width + margin): the sidewalk path cuts through the junction
        // interior around the node, so a hold any later parks the walker IN
        // the carriageway. Past the wait band the walker is committed —
        // stopping mid-box would be strictly worse than walking on.
        const Real kerb = nav_->links[li].width * 0.5 + 3.0;
        if (nav_->isJunction(toNode) && !signals_.hasSignal(li) &&
            toEnd < kerb && toEnd > kerb - 1.8 && a.holdTimer < 12.0) {
            const Vec2 nodeP = nav_->nodes[toNode];
            bool carInbound = false;
            for (const Agent& c : agents_) {
                if (c.mode != Agent::Mode::Driver || !c.moving || c.speed < 0.5)
                    continue;
                if (std::fabs(c.elevation - a.elevation) > 2.5) continue;
                const Real cd = (c.pos - nodeP).length();
                // Inside the box, or reaching it within ~2.5 s at its speed.
                if (cd < 7.0 || cd < c.speed * 2.5 + 4.0) { carInbound = true; break; }
            }
            if (carInbound) {
                a.holdTimer += dt;
                a.speed = 0;
                a.state = Agent::State::Waiting;
                refreshPose(a);
                steer(a, dt);
                return;
            }
        }
        if (toEnd >= kerb + 2.0) a.holdTimer = 0;   // clear of the kerb: reset patience
    }

    Real motion = a.speed * dt;

    // Hard stop for a pedestrian/player ahead: the smooth approachStop above eases
    // the car down but never to exactly zero, so on its own the car would still
    // creep forward and run the person over. Refuse to advance within kPedHardStop
    // of whatever it sees in its lane — the car holds until the path is clear.
    if (seenAhead < 1e9) {
        Real room = std::max(Real(0), seenAhead - kPedHardStop);
        if (motion > room) { motion = room; a.speed = 0; }
    }

    int legCount = static_cast<int>(a.route.links.size());
    while (motion > 0 && a.leg < legCount) {
        Real L = nav_->links[a.route.links[a.leg]].length;
        Real remain = L - a.distOnLeg;
        if (motion < remain) { a.distOnLeg += motion; motion = 0; }
        else {
            motion -= remain;
            // MISSED EXIT (§9.6 device rule): about to turn onto an exit ramp
            // but not in the slow lane — you miss it. Carry on with traffic
            // and re-plan from the freeway continuation.
            if (a.leg + 1 < legCount) {
                const engine::NavLink& cur = nav_->links[a.route.links[a.leg]];
                const engine::NavLink& nxt =
                    nav_->links[a.route.links[a.leg + 1]];
                if (cur.klass == engine::RoadClass::Freeway &&
                    nxt.klass == engine::RoadClass::Ramp &&
                    a.lane < cur.lanes - 1) {
                    int cont = -1;   // the freeway link straight ahead
                    for (int ol : nav_->outLinks[cur.to])
                        if (nav_->links[ol].klass ==
                                engine::RoadClass::Freeway &&
                            nav_->links[ol].to != cur.from)
                            cont = ol;
                    if (cont >= 0) {
                        const int goal = a.route.links.back();
                        engine::Route r2 = engine::findRoute(
                            *nav_, nav_->links[cont].to,
                            nav_->links[goal].to,
                            a.mode == Agent::Mode::Pedestrian);
                        if (r2.valid()) {
                            r2.links.insert(r2.links.begin(), cont);
                            a.route = r2;
                            a.leg = 0;
                            a.distOnLeg = 0;
                            legCount = static_cast<int>(a.route.links.size());
                        }
                    }
                }
            }
            ++a.leg;
            a.distOnLeg = 0;
            // Re-clamp the lane to the NEW leg's lane count: keeping lane 1 from
            // a two-lane arterial onto a one-lane local put the car on the kerb
            // and out of every same-link follower's gap key.
            if (a.leg < legCount) {
                const engine::NavLink& newL = nav_->links[a.route.links[a.leg]];
                int lanes = std::max(1, newL.lanes);
                if (a.lane >= lanes) a.lane = lanes - 1;
                // laneF must follow: it survived a narrow link untouched and
                // POPPED back two lanes on the next wide one (device: "not
                // changing to adjacent lanes, teleporting between lanes")
                if (a.laneF > Real(lanes - 1)) a.laneF = Real(lanes - 1);
                // Merging from a ramp: ENTER on the slow lane, gliding in
                // from the accel-lane side (device: "cars should start on
                // the lane they enter on").
                if (a.leg > 0 && newL.klass == engine::RoadClass::Freeway &&
                    nav_->links[a.route.links[a.leg - 1]].klass ==
                        engine::RoadClass::Ramp) {
                    a.lane = lanes - 1;
                    a.laneF = Real(lanes - 1);   // enter ON the slow lane
                }
            }
        }
    }
    // ARRIVE a few metres short of the exact endpoint (schedule mode): several
    // commuters can share a destination node, and with real collisions (the
    // fender-bender rule) cars converging on the same point crash-locked just
    // shy of it and never completed. Close enough is arrived; the park pose
    // takes over. Wander keeps exact arrival — its trips chain THROUGH the node.
    if (!wander_ && car && a.leg == legCount - 1) {
        Real L = nav_->links[a.route.links[a.leg]].length;
        if (L - a.distOnLeg < std::min(Real(3.0), L * 0.5)) a.leg = legCount;
    }

    if (a.leg >= legCount) {
        arriveOrChain(a, a.speed);   // pass the still-rolling speed: a chain keeps it
    } else {
        refreshPose(a);
        steer(a, dt);
        if (car) labelDriverState(a, seenAhead, gap, legCount);
    }
}

// The arrival branch of advance(), table-driven (ADR-0064): the Arrived event
// asks the agent's GoalTable what comes next. A DRIVER whose next state is
// another trip (a GoTo state — wander's Roam self-loop) CHAINS straight into
// it; otherwise the agent rests — a driver parks on the verge, a walker rests
// at its sidewalk end — wearing the new state's activity label.
// `vArrive` is the speed still carried at the arrival tick — a chain keeps it.
void CitySim::arriveOrChain(Agent& a, Real vArrive) {
    a.moving = false;
    a.speed = 0;
    int lastLink = a.route.links.back();
    // Rest at the ARRIVAL link's elevation — zeroing it parked bridge-deck
    // arrivals at ground level, under their own road.
    a.elevation = nav_->links[lastLink].layer * kLayerClearance +
                  nav_->links[lastLink].elevB;
    const GoalTable& t = goalsFor(a.mode);
    int next = t.onEvent(a.goal, GoalEvent::Arrived);
    if (a.mode == Agent::Mode::Driver && next >= 0 &&
        t.state(next).action == GoalAction::GoTo) {
        // CHAIN straight into the next trip THIS tick (wander, the agent
        // lab) — no parking pose (which read on device as the car "jumping
        // backwards in time, then shooting forward") and no one-frame rest
        // INSIDE the junction box for cross traffic to overlap. The car
        // keeps its speed and heading through the node; steer() rotates it
        // onto the new route at the bounded yaw rate, like any other turn.
        a.restNode = nav_->links[lastLink].to;
        a.arrivedLink = lastLink;
        a.goal = next;
        a.goalHours = 0;
        Vec2 keepHeading = a.heading;
        if (startGoalTrip(a, a.restNode, /*fromRest=*/false)) {
            a.speed = vArrive;
            a.heading = keepHeading;   // no snap: the yaw stays rate-limited
            return;
        }
        // No route anywhere (isolated node): pull onto the VERGE like a
        // schedule arrival — resting in-lane left a body no gap key, no box
        // rule, and no crash pass could see. The goal pass retries the trip.
        {
            Vec2 dir = nav_->direction(lastLink);
            Vec2 right(dir.y, -dir.x);
            Real hw = nav_->links[lastLink].width * 0.5;
            Real back = 4.0 + parkSetbackSlot(a.brain) * 1.3;
            back = std::min(back, nav_->links[lastLink].length * 0.5);
            a.pos = nav_->nodes[nav_->links[lastLink].to] - dir * back +
                    right * (hw + 2.8);
            a.heading = dir;
        }
        a.route.links.clear();
        return;
    }
    if (a.mode == Agent::Mode::Driver) {
        // Park OFF the carriageway (ADR-0062). Resting at the lane's very end
        // was harmless for a kinematic ghost, but the PHYSICAL car that now
        // follows this pose became a roadblock parked at the junction mouth —
        // everyone behind piled up until the next trip, hours later. Pull to
        // the verge beside the link, a few metres short of the node, with a
        // per-agent setback so arrivals at one destination don't stack.
        Vec2 dir = nav_->direction(lastLink);
        Vec2 right(dir.y, -dir.x);
        Real hw = nav_->links[lastLink].width * 0.5;
        Real back = 4.0 + parkSetbackSlot(a.brain) * 1.3;   // 4..13 m
        back = std::min(back, nav_->links[lastLink].length * 0.5);
        a.pos = nav_->nodes[nav_->links[lastLink].to] - dir * back +
                right * (hw + 2.8);
        a.heading = dir;
    } else {
        // A walker rests at the end of its sidewalk (continuous with the final
        // motion — not snapped across the node, which was the old visible jump).
        a.pos = nav_->sidewalkPoint(lastLink, 1.0);
        // Living City: an arrival AT one of its places rests at THAT DOOR — the
        // walker is semantically indoors. The corner pose could sit inside the
        // traffic corridor (and, sensed, froze every approaching car — device).
        const int node = nav_->links[lastLink].to;
        const bool atHome = a.homePlace != kNoPlace && node == a.home;
        const bool atWork =
            node == a.work &&
            (a.workPlace != kNoPlace ||
             (a.role == Agent::Role::Stroller && a.work != a.home));
        if (atHome) a.pos = a.homeDoor;
        else if (atWork) a.pos = a.workDoor;
    }
    if (next >= 0) {
        a.goal = next;
        a.goalHours = 0;
        a.activity = t.state(next).activity;
    } else {
        // No Arrived row (a sparse custom table): keep the historical flip so
        // the day's label still reads sensibly.
        a.activity = (a.activity == Agent::Activity::Commuting)
                         ? Agent::Activity::AtWork
                         : Agent::Activity::AtHome;
    }
    a.restNode = nav_->links[lastLink].to;   // the next departure starts here
    a.arrivedLink = lastLink;                // ...and avoids U-turning back up this
    a.route.links.clear();
}

// Driver FSM (ADR-0061): label what's governing the car this step, from what it
// sees, so the behaviour is legible (debug widgets read a.state). Precedence
// mirrors how advance() actually capped the speed: a ped/player in the cone
// (Yielding) dominates a same-lane leader (Following), which dominates a heading
// change at the coming node (Turning); otherwise the car runs free (Cruising).
// The Waiting-at-a-red case returned from advance() earlier.
void CitySim::labelDriverState(Agent& a, Real seenAhead, Real gap,
                               int legCount) const {
    bool bendAhead = false;
    if (a.leg + 1 < legCount) {
        Vec2 d0 = nav_->direction(a.route.links[a.leg]);
        Vec2 d1 = nav_->direction(a.route.links[a.leg + 1]);
        Real align = d0.x * d1.x + d0.y * d1.y;   // 1 = straight through
        Real distToEnd = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;
        if (align < 0.98 && distToEnd < kJunctionApproach) bendAhead = true;
    }
    if (seenAhead < 1e9)          a.state = Agent::State::Yielding;
    else if (gap < kCarSlowZone)  a.state = Agent::State::Following;
    else if (bendAhead)           a.state = Agent::State::Turning;
    else                          a.state = Agent::State::Cruising;
}

void CitySim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    gaps_.assign(agents_.size(), INF);
    minGaps_.assign(agents_.size(), kCarMinGap);   // overwritten where a leader exists
    leaderSpeeds_.assign(agents_.size(), 0.0);     // valid only where gaps_ < INF
    auto laneKeyOf = [this](const Agent& a, int li) {
        // Clamp per link so a car keyed onto a narrower continuation matches the
        // followers/leaders actually driving that link's lanes.
        int laneKey = (a.mode == Agent::Mode::Driver)
                          ? std::min(a.lane, std::max(1, nav_->links[li].lanes) - 1)
                          : 1024;
        return static_cast<long long>(li) * 4096 + laneKey;
    };
    std::unordered_map<long long, std::vector<std::pair<Real, int>>> lanes;
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        const Agent& a = agents_[i];
        if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
        lanes[laneKeyOf(a, a.route.links[a.leg])].push_back({a.distOnLeg, i});
    }
    // (link,lane) -> the car nearest the entry {distOnLeg, agentIndex}, so a
    // follower crossing a node can pick up the leader on its next link AND that
    // leader's length (for a length-aware gap).
    std::unordered_map<long long, std::pair<Real, int>> minEntry;
    for (auto& kv : lanes) {
        std::vector<std::pair<Real, int>>& v = kv.second;
        std::sort(v.begin(), v.end(), [](const std::pair<Real, int>& a,
                                          const std::pair<Real, int>& b) {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });
        minEntry[kv.first] = { v.front().first, v.front().second };
        for (std::size_t k = 0; k + 1 < v.size(); ++k) {
            gaps_[v[k].second] = v[k + 1].first - v[k].first;
            minGaps_[v[k].second] = pairMinGap(v[k].second, v[k + 1].second);
            leaderSpeeds_[v[k].second] = agents_[v[k + 1].second].speed;
        }
    }
    // Car-following ACROSS a node: the front car on a link (no leader ahead on its
    // own link) keeps its gap to the car just ahead on its NEXT link — the one it's
    // chasing through the intersection or around the bend. Without this, followers
    // overlap the leader as they cross a node (same-lane keys don't span it). We
    // chain across up to a couple of short links so a leader that has already moved
    // onto the link-after-next is still seen.
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        const Agent& a = agents_[i];
        if (gaps_[i] != INF) continue;                 // has a same-link leader already
        if (!a.moving) continue;
        int legN = static_cast<int>(a.route.links.size());
        Real ahead = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;  // to end of this link
        for (int step = 1; step <= 2 && a.leg + step < legN; ++step) {
            int nextLi = a.route.links[a.leg + step];
            auto it = minEntry.find(laneKeyOf(a, nextLi));
            if (it != minEntry.end()) {
                gaps_[i] = ahead + it->second.first;
                minGaps_[i] = pairMinGap(i, it->second.second);
                leaderSpeeds_[i] = agents_[it->second.second].speed;
                break;
            }
            ahead += nav_->links[nextLi].length;       // no car on this link; look one further
        }
    }
}

// The car-vs-car VISION WEDGE (roads-v2 plan Â§3.1, sense 2 of the driver
// agent) â slice 3: the CORRIDOR sense. The lane-keyed gap logic only sees
// leaders on MY route's lane chain; a merging car's nose, a wreck on another
// link, or a body stalled inside the junction box is invisible to it â and
// the soak telemetry traced real contacts to exactly those. Each car senses
// bodies in its forward swept corridor and hands the nearest to IDM as a
// leader. Guard rails that keep the historic deadlock out (cars were
// deliberately blind to cars before this â braking for everything on the
// heading ray froze junctions):
//   - ROUTE-AWARE horizon: my corridor ends at my link's end node (+ a box
//     allowance) â past it my path turns; bodies there (cross-street
//     queues!) are not ahead of me whatever the ray says.
//   - ONCOMING cars in their own lane pass; only a truly head-on body binds.
//   - a STOPPED cross-oriented car beyond close range is a cross-street
//     queue â the signal owns that conflict, not mutual corridor braking.
void CitySim::computeCarWedge() {
    const Real INF = std::numeric_limits<Real>::infinity();
    carAheadGap_.assign(agents_.size(), INF);
    carAheadSpeed_.assign(agents_.size(), 0.0);
    constexpr Real kRange = 38.0;   // sense horizon (m)
    constexpr Real kLat = 2.15;     // corridor half-width: two bodies abreast
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.mode != Agent::Mode::Driver || !a.moving) continue;
        const Real fx = a.heading.x, fy = a.heading.y;
        Real linkLeft = kRange;
        if (nav_ && a.leg < static_cast<int>(a.route.links.size())) {
            const int li = a.route.links[a.leg];
            linkLeft = nav_->links[li].length - a.distOnLeg + 10.0;
        }
        const Real horizon = std::min(kRange, linkLeft);
        for (std::size_t j = 0; j < agents_.size(); ++j) {
            if (j == i) continue;
            const Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver) continue;
            // Different decks never conflict (viaduct vs the street below).
            if (std::fabs(b.elevation - a.elevation) > 2.5) continue;
            const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            if (dx * dx + dy * dy > kRange * kRange) continue;
            const Real along = fx * dx + fy * dy;
            const Real across = -fy * dx + fx * dy;   // + = b on my left
            if (along <= 0 || along >= horizon || std::fabs(across) >= kLat)
                continue;
            const Real hdot = b.heading.x * fx + b.heading.y * fy;
            // ONCOMING bodies are not the corridor's problem, in-lane or not:
            // a lane-bound car cannot dodge sideways, so braking converts a
            // (rare, U-turn-made) head-on into a nose-to-nose STANDOFF that
            // the creep/contact/escape machinery then has to grind through --
            // measured as escape time trebling on the dead-end wander map.
            // The contact rule arbitrates real head-on touches, as before.
            if (hdot < -0.6) continue;
            if (b.speed < 0.5 && std::fabs(hdot) < 0.5 && along > 9.0)
                continue;                                           // cross queue: signal's job
            const Real bodies = 0.5 * (vehicleLength(static_cast<int>(i)) +
                                       vehicleLength(static_cast<int>(j)));
            const Real gap = std::max(Real(0.05), along - bodies);
            if (gap < carAheadGap_[i]) {
                carAheadGap_[i] = gap;
                carAheadSpeed_[i] = std::max(Real(0), b.speed * hdot);
            }
        }
        // CROSSING sense (slice 4): a car on a predicted collision course —
        // closest approach inside the combined body envelope within my
        // stopping horizon. The conflicts the corridor can't see: two greens
        // turning across each other, unsignalled crossings, merge angles.
        // Anti-deadlock is ASYMMETRY: I yield only to a crosser approaching
        // from my RIGHT (right-hand traffic); it sees me on ITS left and
        // drives on — mutual yield cannot form. A crosser that a red signal
        // will stop before the box is exempt (a green driver assumes signal-
        // stopped traffic stays stopped); one already committed is not.
        for (std::size_t j = 0; j < agents_.size(); ++j) {
            if (j == i) continue;
            const Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver || !b.moving || b.speed < 0.3)
                continue;
            if (std::fabs(b.elevation - a.elevation) > 2.5) continue;
            const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            if (dx * dx + dy * dy > kRange * kRange) continue;
            const Real across = -fy * dx + fx * dy;
            if (across > 0) continue;             // my LEFT: their yield, not mine
            const Real hdot = b.heading.x * fx + b.heading.y * fy;
            if (hdot < -0.6 || hdot > 0.85) continue;   // oncoming/parallel: not a crossing
            if (nav_ && b.leg < static_cast<int>(b.route.links.size())) {
                const int bLi = b.route.links[b.leg];
                const bool redBound =
                    signals_.hasSignal(bLi) &&
                    signals_.stateForLink(bLi) != SignalState::Green &&
                    b.distOnLeg < nav_->links[bLi].length - 3.0;
                if (redBound) continue;           // the signal will stop them
            }
            const Real rvx = b.heading.x * b.speed - fx * a.speed;
            const Real rvy = b.heading.y * b.speed - fy * a.speed;
            const Real rv2 = rvx * rvx + rvy * rvy;
            if (rv2 < 1e-6) continue;
            const Real tStar = -(dx * rvx + dy * rvy) / rv2;
            // Look ahead at least my own stopping time — a fast car needs the
            // conflict called early enough to brake for it.
            const Real myHorizon = std::max(Real(2.6), a.speed / kCarDecel + Real(1.0));
            if (tStar <= 0 || tStar > myHorizon) continue;
            const Real mx = dx + rvx * tStar, my = dy + rvy * tStar;
            if (mx * mx + my * my > (2.0 * kLat) * (2.0 * kLat)) continue;
            const Real bodies = 0.5 * (vehicleLength(static_cast<int>(i)) +
                                       vehicleLength(static_cast<int>(j)));
            // Brake as if a stopped body sat at my share of the closing run.
            const Real gap = std::max(Real(0.05), a.speed * tStar - bodies);
            if (gap < carAheadGap_[i]) {
                carAheadGap_[i] = gap;
                carAheadSpeed_[i] = 0.0;
            }
        }
    }
}

void CitySim::step(Real dt, Real hoursPerSecond) {
    if (!nav_ || agents_.empty()) return;

    clockHours_ += dt * hoursPerSecond;
    clockHours_ = std::fmod(clockHours_, 24.0);
    if (clockHours_ < 0) clockHours_ += 24.0;
    simSeconds_ += dt;   // memory's time base: sightings age and fade against this
    signals_.update(dt);

    // Pass 1: the GOAL layer (ADR-0064) — AI agents only, the host drives
    // players. Each agent runs its current state in its archetype's GoalTable:
    // a resting agent emits this tick's events (clock windows, dwell, idle) and
    // takes the first matching transition; a GoTo agent whose trip isn't
    // running retries the departure (launch-clearance gated for drivers, so a
    // car waits out traffic near its spot and pulls out when clear). The
    // built-in tables reproduce the historical schedule and wander behaviour
    // exactly; see goalThink / city_goals.h.
    for (Agent& a : agents_) {
        if (a.playerControlled || a.released) continue;
        goalThink(a, dt * hoursPerSecond);
    }

    // Snapshot the bodies agents may SENSE this step — pedestrians and the
    // player — so perception sees a consistent world. Each carries a STABLE id
    // (agent index; -(1+k) for the k-th host-injected point) so an observer's
    // memory can track it across steps, and its elevation so the 2.5D sensor can
    // ignore bodies a bridge deck away. AI cars are deliberately excluded (see
    // advance()): lanes + car-following + signals govern car-vs-car, and braking
    // for cross/oncoming cars in the cone deadlocked traffic.
    sensed_.clear();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        const Agent& a = agents_[i];
        // A RESTING pedestrian is INSIDE its place (home/shop/office) — not a
        // body on the street. Sensing it anyway had cars braking forever for a
        // person who is semantically indoors (a rest pose near the kerb sat in
        // the sensed corridor and froze traffic for good — device jam).
        if (a.mode == Agent::Mode::Pedestrian && !a.moving && !a.playerControlled)
            continue;
        if (a.mode == Agent::Mode::Pedestrian || a.playerControlled) {
            // A tethered walker has a physical BODY (the tether anchor the
            // bridge feeds every step) that may trail its ghost by metres —
            // cars must brake for where the person IS, not where the plan is.
            Vec2 sp = (a.mode == Agent::Mode::Pedestrian && a.tethered)
                          ? a.tetherAnchor : a.pos;
            sensed_.push_back({sp, a.elevation, static_cast<int>(i)});
        }
    }
    // The live player (host-injected) is a body too — cars brake for it.
    for (std::size_t k = 0; k < externalObstacles_.size(); ++k)
        sensed_.push_back({externalObstacles_[k], 0.0, -(1 + static_cast<int>(k))});

    // Pass 2: leading gaps. Pass 3: advance. `advanced` records who really ran
    // advance() (and so had refreshPose re-anchor its base position) this tick —
    // the reactive lean below is only valid on a re-anchored pose.
    std::vector<uint8_t> advanced(agents_.size(), 0);
    computeGaps();
    computeCarWedge();   // S7 senses: bodies in the forward corridor
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.playerControlled || a.released) continue;
        // Crashed (fender-bender): the car sits where it hit until its hold
        // expires — a crash is a real stop, not a suggestion.
        if (a.crashTimer > 0) {
            a.crashTimer -= dt;
            a.speed = 0;
            a.state = Agent::State::Waiting;
            continue;
        }
        if (a.crashCount > 0) {
            Real dx = a.pos.x - a.crashAnchor.x, dy = a.pos.y - a.crashAnchor.y;
            if (dx * dx + dy * dy > 6.0 * 6.0) a.crashCount = 0;   // clear of the wreck
        }
        // Tethered ghost too far from its physical car: WAIT for it. The plan can
        // never outrun the physics — a car knocked back, climbing, or slow off the
        // line finds its ghost holding just ahead instead of gone (ADR-0062).
        if (a.moving && a.tethered) {
            Real dx = a.pos.x - a.tetherAnchor.x, dy = a.pos.y - a.tetherAnchor.y;
            if (std::sqrt(dx * dx + dy * dy) > a.tetherLead) {
                a.speed = 0;
                a.state = Agent::State::Waiting;
                continue;
            }
        }
        if (a.moving) { advance(a, dt, gaps_[i], minGaps_[i]); advanced[i] = 1; }
    }

    // FENDER-BENDERS (device: cars must collide, not ghost). Ambient cars are
    // planner-owned (ADR-0062) — no rigid bodies between them — so contact is
    // resolved here: when two moving cars' bodies meet on a CLOSING course, both
    // freeze on the spot (speed 0, red ring) and resume on staggered per-brain
    // holds, so a junction tangle crunches to a stop and unwinds car by car.
    // Triggering only on a closing approach lets the first resumer drive OUT
    // through the residual overlap without instantly re-freezing the pair.
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.mode != Agent::Mode::Driver || !a.moving || a.released ||
            a.playerControlled)
            continue;
        for (std::size_t j = i + 1; j < agents_.size(); ++j) {
            Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver || !b.moving || b.released ||
                b.playerControlled)
                continue;
            if (std::fabs(b.elevation - a.elevation) > 3.0) continue;  // bridge deck
            // ESCAPE VALVE: a car wedged through many consecutive freezes (a
            // wreck neither party can steer or reverse out of) stops being a
            // contact until it drives clear — the tow-truck resolution. Without
            // it a crossing-path contact never resolves and the junction dies.
            if (a.crashCount > 5 || b.crashCount > 5) continue;
            Real rs = 0.35 * (vehicleLength(static_cast<int>(i)) +
                              vehicleLength(static_cast<int>(j)));
            Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            Real d2 = dx * dx + dy * dy;
            if (d2 >= rs * rs) continue;              // broad phase (cheap reject)
            // ORIENTED narrow phase (roads-v2 S7 slice 2): a car is ~4.5 m
            // long but only ~2 m wide — the isotropic disc called every
            // adjacent-lane pass and turn-arc convergence a "crash" (measured
            // 115 phantom-heavy contacts in the 10-minute soak). Each body is
            // a CAPSULE (axis segment along the heading, half-width radius);
            // contact = exact segment-segment distance under the summed radii:
            // fires exactly at bumper touch nose-to-tail, at body overlap in a
            // side-swipe, and never for a lane-apart pass (>= 3 m centrelines).
            {
                const Real kHalfW = 0.92;   // body half-width + a small skin
                auto axis = [&](const Agent& c, int idx, Real& ex, Real& ey) {
                    const Real half =
                        std::max(Real(0), 0.5 * vehicleLength(idx) - kHalfW);
                    ex = c.heading.x * half;
                    ey = c.heading.y * half;
                };
                Real ax, ay, bx2, by2;
                axis(a, static_cast<int>(i), ax, ay);
                axis(b, static_cast<int>(j), bx2, by2);
                const Real A0x = -ax, A0y = -ay, A1x = ax, A1y = ay;
                const Real B0x = dx - bx2, B0y = dy - by2;
                const Real B1x = dx + bx2, B1y = dy + by2;
                auto ori = [](Real ox, Real oy, Real px, Real py,
                              Real qx, Real qy) {
                    const Real v = (px - ox) * (qy - oy) - (py - oy) * (qx - ox);
                    return v > 1e-9 ? 1 : (v < -1e-9 ? -1 : 0);
                };
                Real best2 = 1e30;
                if (ori(A0x, A0y, A1x, A1y, B0x, B0y) !=
                        ori(A0x, A0y, A1x, A1y, B1x, B1y) &&
                    ori(B0x, B0y, B1x, B1y, A0x, A0y) !=
                        ori(B0x, B0y, B1x, B1y, A1x, A1y)) {
                    best2 = 0;   // axes cross: bodies interpenetrate
                } else {
                    auto p2s2 = [](Real px, Real py, Real sx, Real sy,
                                   Real tx, Real ty) {
                        const Real ux = tx - sx, uy = ty - sy;
                        const Real l2 = ux * ux + uy * uy;
                        Real t = l2 > 1e-12
                                     ? ((px - sx) * ux + (py - sy) * uy) / l2
                                     : 0.0;
                        t = t < 0 ? 0 : (t > 1 ? 1 : t);
                        const Real cx = sx + ux * t - px, cy = sy + uy * t - py;
                        return cx * cx + cy * cy;
                    };
                    best2 = std::min(
                        std::min(p2s2(B0x, B0y, A0x, A0y, A1x, A1y),
                                 p2s2(B1x, B1y, A0x, A0y, A1x, A1y)),
                        std::min(p2s2(A0x, A0y, B0x, B0y, B1x, B1y),
                                 p2s2(A1x, A1y, B0x, B0y, B1x, B1y)));
                }
                if (best2 >= (2.0 * kHalfW) * (2.0 * kHalfW)) continue;
            }
            Real rvx = b.heading.x * b.speed - a.heading.x * a.speed;
            Real rvy = b.heading.y * b.speed - a.heading.y * a.speed;
            // Separating bodies part without a crash — EXCEPT when both are
            // still driving fast through each other: crossing/passing
            // geometry flips to "separating" the instant the centres pass,
            // while the bodies are still interpenetrating. That IS a crash
            // (it read as ghosting, observably), not a near miss.
            if (rvx * dx + rvy * dy >= 0 && !(a.speed > 2.0 && b.speed > 2.0))
                continue;
            // Undo the overshoot: whoever MOVED into the contact backs its path
            // progress out again, split by speed, so the pair rests touching —
            // without this a resumed car RATCHETED deeper each freeze cycle.
            Real dist = std::sqrt(d2);
            Real pen = rs - dist;
            Real vsum = a.speed + b.speed;
            if (pen > 0 && vsum > 1e-6) {
                // Returns whatever couldn't be applied (a car at the very start
                // of its leg can't back past it) so the PARTNER absorbs it —
                // otherwise a freshly-chained car at distOnLeg 0 left half the
                // penetration in place.
                auto rollBack = [&](Agent& c, Agent& other, Real amount) -> Real {
                    if (amount <= 0 || !c.moving ||
                        c.leg >= static_cast<int>(c.route.links.size()))
                        return amount;
                    // Backing up must SEPARATE the pair — a car that has already
                    // slightly passed its partner would back INTO it instead.
                    Real toOther = c.heading.x * (other.pos.x - c.pos.x) +
                                   c.heading.y * (other.pos.y - c.pos.y);
                    if (toOther <= 0) return amount;   // partner behind: hold still
                    Real applied = std::min(amount, c.distOnLeg);
                    c.distOnLeg -= applied;
                    refreshPose(c);
                    return amount - applied;
                };
                Real leftA = rollBack(a, b, pen * (a.speed / vsum));
                Real leftB = rollBack(b, a, pen * (b.speed / vsum));
                if (leftA > 0) rollBack(b, a, leftA);
                if (leftB > 0) rollBack(a, b, leftB);
            }
            if (std::getenv("RT_CRASH_DEBUG")) {
                const Real hdot = a.heading.x * b.heading.x + a.heading.y * b.heading.y;
                const Real along = a.heading.x * dx + a.heading.y * dy;
                const Real across = -a.heading.y * dx + a.heading.x * dy;
                const int la = a.leg < (int)a.route.links.size() ? a.route.links[a.leg] : -1;
                const int lb = b.leg < (int)b.route.links.size() ? b.route.links[b.leg] : -1;
                std::printf("[crash] t=%.1f pos(%.1f,%.1f) va=%.1f vb=%.1f hdot=%+.2f "
                            "along=%+.1f across=%+.1f la=%d(%.0f/%.0f) lb=%d(%.0f/%.0f) "
                            "cca=%d ccb=%d\n",
                            simSeconds_, a.pos.x, a.pos.y, a.speed, b.speed, hdot,
                            along, across,
                            la, a.distOnLeg, la >= 0 ? nav_->links[la].length : 0.0,
                            lb, b.distOnLeg, lb >= 0 ? nav_->links[lb].length : 0.0,
                            a.crashCount, b.crashCount);
            }
            if (a.speed > 2.0 && b.speed > 2.0) ++fastCrashEvents_;
            auto crash = [this](Agent& c) {
                if (c.crashTimer <= 0) {
                    c.crashTimer = crashHoldSeconds(c.brain);
                    if (c.crashCount == 0) c.crashAnchor = c.pos;
                    ++c.crashCount;
                    ++crashEvents_;   // soak gate: total contacts over a run
                }
                // Contact means BODIES TOUCHING now (capsule), so the impact
                // kills the velocity WITH the event — without this, a pair
                // that cannot be rolled apart (both freshly chained at a
                // dead-end tip, distOnLeg ~0) spends the pre-freeze tick
                // interpenetrating at full speed: ghosting, observably.
                c.speed = 0;
                c.state = Agent::State::Waiting;
            };
            crash(a);
            crash(b);
        }
    }

    // Reactive pedestrian behaviour (ADR-0061): each walker acts on what it SEES
    // in its vision cone. It steers to one side to go AROUND the neighbours (and
    // the player) it sees ahead — and never reacts to what's behind it. A hard
    // body-overlap floor below is the physical backstop so two people can't occupy
    // the same spot. Deterministic (index order); resets to the sidewalk each
    // step, so the sidestep is a transient lean while someone is in view.
    for (Agent& a : agents_)
        if (!a.moving) { a.state = Agent::State::Resting; a.lateralOffset = 0; }
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (!a.moving) continue;
        if (a.mode == Agent::Mode::Driver) continue;   // driver FSM is set in advance()
        // Only agents advance() re-anchored this tick may lean: the lean is
        // POS += offset on top of a fresh refreshPose. A tether-held (or host-
        // controlled) ghost skipped advance(), so leaning it again would stack
        // offset on offset — a pinned walker's ghost slid sideways without
        // bound, dragging its body after it through the tether feedback.
        if (!advanced[i]) continue;
        // `travel` is the walker's path direction this step (set by steer()); it
        // leans to `right` of that to go around what it SEES ahead.
        Vec2 travel = a.heading;
        Vec2 rightv(travel.y, -travel.x);

        // THINK on the slow clock, ACT every tick (ADR-0062). The reactive scan
        // (who's ahead, which side do I pass) runs only when this agent's think
        // timer expires — staggered per agent — and its answer is COMMITTED to
        // `leanTarget` + the FSM state until the next think. Re-deciding every
        // tick made walkers flip-flop ("wigging out"); a held decision reads as
        // intent, and the integration below still moves smoothly every tick.
        a.thinkTimer -= dt;
        if (a.thinkTimer <= 0) {
            a.thinkTimer += thinkPeriod_;
            engine::VisionCone cone;
            cone.origin = a.pos; cone.forward = travel;
            cone.range = kPedVisionRange; cone.halfAngleRad = kPedVisionHalfAngle;
            // SENSE -> REMEMBER (ADR-0063): sight the walking neighbours (and the
            // player) into memory at think cadence — successive sightings a think
            // apart are what give each track its velocity estimate.
            engine::SensorVolume sensor;
            sensor.cone = cone;
            for (const SensedGhost& g : sensed_) {
                if (g.id == static_cast<int>(i)) continue;   // not myself
                // Skip player-agent ghosts: the host already injects the live
                // player as an external point — seeing both would double it.
                if (g.id >= 0 && agents_[g.id].playerControlled) continue;
                Real dh = g.id < 0 ? 0.0 : g.elevation - a.elevation;
                if (engine::sees(sensor, g.pos, dh))
                    a.memory.observe(g.id, g.pos, simSeconds_);
            }
            a.memory.update(simSeconds_);
            // PREDICT -> DECIDE: lean away from where each remembered body will
            // BE shortly (its track velocity run a beat ahead), not where the
            // snapshot last had it — so two approaching walkers part early and
            // smoothly. The answer is COMMITTED (leanTarget) until the next
            // think. Poles are static and eternal: no memory needed.
            Real bias = 0; bool saw = false;
            auto consider = [&](const Vec2& p) {
                if (!engine::sees(cone, p)) return;
                Real fd = engine::forwardDistance(cone, p);
                if (fd <= 0.1 || fd >= kPedVisionRange) return;
                saw = true;
                Real side = (a.pos.x - p.x) * rightv.x + (a.pos.y - p.y) * rightv.y;
                Real w = (kPedVisionRange - fd) / kPedVisionRange;   // nearer -> stronger
                bias += (side >= 0 ? 1.0 : -1.0) * w;                // pass on the side I'm on
            };
            for (const engine::TrackedBody& t : a.memory.tracks()) {
                if (t.confidence < kMemoryActConfidence) continue;
                consider(t.pos + t.vel * kPedAnticipation);
            }
            for (const Vec2& o : staticObstacles_) consider(o);   // signal poles
            // Cars are bodies too (device: walkers pinned against a car): the
            // walker considers the CLOSEST POINT on each car's rectangle — not
            // its centre, which sits outside the short vision cone when you're
            // up against a long body — so the lean routes around the car. Both
            // driven and parked cars: vehicles_ carries every body's live pose.
            for (const SimVehicle& v : vehicles_) {
                const Real ve = (v.driver >= 0 &&
                                 v.driver < static_cast<int>(agents_.size()))
                                    ? agents_[v.driver].elevation : 0.0;
                if (std::fabs(ve - a.elevation) > 2.0) continue;   // a deck away
                const Vec2 f = v.heading, r(v.heading.y, -v.heading.x);
                const Vec2 rel(a.pos.x - v.pos.x, a.pos.y - v.pos.y);
                const Real hl = v.length * 0.5, hw = v.width * 0.5;
                const Real lx = std::max(-hl, std::min(hl, rel.x * f.x + rel.y * f.y));
                const Real ly = std::max(-hw, std::min(hw, rel.x * r.x + rel.y * r.y));
                consider(Vec2(v.pos.x + f.x * lx + r.x * ly,
                              v.pos.y + f.y * lx + r.y * ly));
            }
            a.leanTarget = std::max(Real(-1), std::min(Real(1), bias)) * kPedMaxLateral;
            // Held at a signal (advance() parked us at the kerb, speed 0,
            // Waiting): keep the honest red ring rather than repainting Walking.
            if (!(a.speed == 0 && a.state == Agent::State::Waiting))
                a.state = saw ? Agent::State::Avoiding : Agent::State::Walking;
        }

        // ACT: move the lean toward the COMMITTED target at a bounded RATE
        // (continuous, not a pop): grows while the decision says "step aside",
        // decays back to the path once a think says clear. This is what lets you
        // herd a walker like a boid.
        Real prev = a.lateralOffset;
        Real maxDelta = kPedLateralRate * dt;
        Real delta = std::max(-maxDelta, std::min(maxDelta, a.leanTarget - prev));
        a.lateralOffset = prev + delta;

        a.pos.x += rightv.x * a.lateralOffset;
        a.pos.y += rightv.y * a.lateralOffset;
        // Turn to face where it's actually moving (forward walk + the sideways
        // drift), so the body visibly rotates as it steers away. Uses the ped's
        // OWN speed (personality-scaled), not the nominal walk speed.
        Real walk = a.speed > 0.1 ? a.speed : kWalkSpeed * a.speedFactor;
        Vec2 vel(travel.x * walk + rightv.x * (delta / dt),
                 travel.y * walk + rightv.y * (delta / dt));
        Real vl = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        if (vl > 1e-6) a.heading = Vec2(vel.x / vl, vel.y / vl);
    }
    // Hard body-overlap floor: several symmetric relaxation passes so two people
    // (whether or not they saw each other) never interpenetrate.
    for (int iter = 0; iter < 6; ++iter)
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            Agent& a = agents_[i];
            if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
            for (std::size_t j = i + 1; j < agents_.size(); ++j) {
                Agent& b = agents_[j];
                if (b.mode != Agent::Mode::Pedestrian || !b.moving) continue;
                Real dx = a.pos.x - b.pos.x, dy = a.pos.y - b.pos.y;
                Real d = std::sqrt(dx * dx + dy * dy);
                if (d > 1e-4 && d < kPedBodyMin) {
                    Real push = (kPedBodyMin - d) * 0.5;
                    a.pos.x += dx / d * push; a.pos.y += dy / d * push;
                    b.pos.x -= dx / d * push; b.pos.y -= dy / d * push;
                } else if (d <= 1e-4) {
                    a.pos.x += kPedBodyMin * 0.5;
                    b.pos.x -= kPedBodyMin * 0.5;
                }
            }
        }

    // Hard radial push-out from static obstacles (signal poles) and the PLAYER: a
    // walker never ends up standing inside a pole — or brushing through the
    // player (a wider berth, so a near miss reads as a step-around). The cone
    // bias above makes it lean away in advance; this is the physical backstop.
    for (Agent& a : agents_) {
        if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
        auto pushOut = [&](const Vec2& o, Real clearance) {
            Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
            Real d = std::sqrt(dx * dx + dy * dy);
            if (d > 1e-4 && d < clearance) {
                Real push = clearance - d;
                a.pos.x += dx / d * push;
                a.pos.y += dy / d * push;
            } else if (d <= 1e-4) {
                a.pos.x += clearance;        // dead-centre: pick a direction
            }
        };
        for (const Vec2& o : staticObstacles_) pushOut(o, kPoleClearance);
        for (const Vec2& o : externalObstacles_) pushOut(o, kPlayerClearance);
        // ...and never stand INSIDE a car's footprint (device: walkers stuck
        // bumping a car). The push resolves along the SHALLOWEST axis of the
        // car-local box, so a walker pressed against a door is squeezed out
        // sideways and SLIDES along the body until it clears the bumper —
        // combined with the cone lean above, it walks around the car.
        for (const SimVehicle& v : vehicles_) {
            const Real ve = (v.driver >= 0 &&
                             v.driver < static_cast<int>(agents_.size()))
                                ? agents_[v.driver].elevation : 0.0;
            if (std::fabs(ve - a.elevation) > 2.0) continue;
            const Vec2 f = v.heading, r(v.heading.y, -v.heading.x);
            const Vec2 rel(a.pos.x - v.pos.x, a.pos.y - v.pos.y);
            const Real cx = v.length * 0.5 + 0.35, cy = v.width * 0.5 + 0.35;
            const Real lx = rel.x * f.x + rel.y * f.y;
            const Real ly = rel.x * r.x + rel.y * r.y;
            if (std::fabs(lx) >= cx || std::fabs(ly) >= cy) continue;
            const Real px = cx - std::fabs(lx), py = cy - std::fabs(ly);
            if (px < py) {
                const Real s = lx >= 0 ? 1.0 : -1.0;
                a.pos.x += f.x * s * px; a.pos.y += f.y * s * px;
            } else {
                const Real s = ly >= 0 ? 1.0 : -1.0;
                a.pos.x += r.x * s * py; a.pos.y += r.y * s * py;
            }
        }
    }

    // A possessed car mirrors its driver; an unpossessed (parked) car stays put.
    for (Agent& a : agents_) {
        if (a.mode == Agent::Mode::Driver && a.vehicle >= 0 &&
            a.vehicle < static_cast<int>(vehicles_.size())) {
            vehicles_[a.vehicle].pos = a.pos;
            vehicles_[a.vehicle].heading = a.heading;
        }
    }
}

}  // namespace citysim
