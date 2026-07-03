#include "city_sim.h"

#include "traffic_rules.h"   // approachStop

#include <algorithm>
#include <cmath>
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
    return (l.width * 0.5) / static_cast<Real>(lanes);
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
    signals_.build(graph);

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
        for (int li : graph.outLinks[v]) r = std::max(r, graph.links[li].width * 0.5);
        nodeBoxRadius_[v] = r;
        if (graph.isJunction(v)) junctions_.push_back({graph.nodes[v], r});
    }

    const int total = driverCount + pedCount;
    agents_.reserve(total);
    vehicles_.reserve(driverCount);
    for (int i = 0; i < total; ++i) {
        Agent a;
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
        a.brain = rnd() | 1u;            // per-agent fault RNG (non-zero)
        // Personality from the brain's own bits (NOT an rnd() draw, so the build
        // stream — and every seeded test scenario — is unchanged): pace in
        // [0.85, 1.15] of nominal.
        a.speedFactor = 0.85 + 0.30 * (((a.brain >> 9) & 0x7FFF) / Real(0x7FFF));
        // Stagger the think clocks so the crowd doesn't re-decide in lockstep.
        a.thinkTimer = thinkPeriod_ * (((a.brain >> 17) & 0xFF) / Real(0xFF));
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
    Real back = 2.0 + static_cast<Real>((brain >> 3) & 7) *
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
        engine::Route r = engine::findRoute(*nav_, from, goal);
        if (!r.valid()) continue;
        if (reversesArrival(r)) { if (fallback < 0) fallback = goal; continue; }
        a.activity = Agent::Activity::Commuting;
        startTrip(a, from, goal, fromRest);
        return a.moving;
    }
    if (fallback >= 0) {
        a.activity = Agent::Activity::Commuting;
        startTrip(a, from, fallback, fromRest);
        return a.moving;
    }
    return false;
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
    a.route = engine::findRoute(*nav_, origin, goal);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    if (!a.route.valid()) {
        // No path: do NOT teleport to the goal (that was the "disappear/reappear"
        // bug). Stay parked at the ORIGIN and fall back to the origin's resting
        // state, so the agent simply doesn't take this trip.
        a.moving = false;
        a.elevation = 0;
        a.pos = idlePose(origin, a.mode, a.brain);
        a.activity = (a.activity == Agent::Activity::Commuting) ? Agent::Activity::AtHome
                                                               : Agent::Activity::AtWork;
        return;
    }
    int lanes = nav_->links[a.route.links.front()].lanes;
    a.lane = (a.mode == Agent::Mode::Driver && lanes > 1)
                 ? static_cast<int>(rnd() % static_cast<uint32_t>(lanes))
                 : 0;
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
        // Clamp the lane PER LINK: the blend samples neighbouring legs whose
        // lane counts can be smaller than the current lane index.
        int lane = std::min(a.lane, std::max(1, nav_->links[link].lanes) - 1);
        return (a.mode == Agent::Mode::Driver)
                   ? nav_->laneCenter(link, lane, t, laneSpacing(nav_->links[link]))
                   : nav_->sidewalkPoint(link, t);
    };
    a.pos = sample(li, L > 1e-9 ? s / L : 0.0);
    a.elevation = nav_->links[li].layer * kLayerClearance;

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
            for (const Agent& b : agents_) {
                if (&b == &a || b.mode != Agent::Mode::Driver) continue;
                if (!b.moving || b.released) continue;
                if (std::fabs(b.elevation - a.elevation) > 3.0) continue;
                Real dx = b.pos.x - jc.x, dy = b.pos.y - jc.y;
                Real db2 = dx * dx + dy * dy;
                if (db2 > range * range) continue;
                Real along = b.heading.x * d0.x + b.heading.y * d0.y;
                // 1: crossing-heading occupant INSIDE the box proper.
                if (db2 < jr * jr && std::fabs(along) < 0.7) {
                    yieldAtLine = true;
                    break;
                }
                // 2: turn yield against oncoming approach traffic.
                if (!turning || along >= -0.3) continue;
                if (b.speed > 0.5) { yieldAtLine = true; break; }  // live traffic
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
    Real slowZone = car ? kCarSlowZone : kPedSlowZone;
    target = followCap(target, gap, minGap, slowZone);
    a.speed = std::min(target, a.speed + accel * dt);

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

    Real motion = a.speed * dt;

    // Hard stop for a pedestrian/player ahead: the smooth approachStop above eases
    // the car down but never to exactly zero, so on its own the car would still
    // creep forward and run the person over. Refuse to advance within kPedHardStop
    // of whatever it sees in its lane — the car holds until the path is clear.
    if (seenAhead < 1e9) {
        Real room = std::max(Real(0), seenAhead - kPedHardStop);
        if (motion > room) { motion = room; a.speed = 0; }
    }

    const int legCount = static_cast<int>(a.route.links.size());
    while (motion > 0 && a.leg < legCount) {
        Real L = nav_->links[a.route.links[a.leg]].length;
        Real remain = L - a.distOnLeg;
        if (motion < remain) { a.distOnLeg += motion; motion = 0; }
        else {
            motion -= remain;
            ++a.leg;
            a.distOnLeg = 0;
            // Re-clamp the lane to the NEW leg's lane count: keeping lane 1 from
            // a two-lane arterial onto a one-lane local put the car on the kerb
            // and out of every same-link follower's gap key.
            if (a.leg < legCount) {
                int lanes = std::max(1, nav_->links[a.route.links[a.leg]].lanes);
                if (a.lane >= lanes) a.lane = lanes - 1;
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

// The arrival branch of advance(): a wander driver CHAINS into its next trip, a
// schedule driver parks on the verge, a walker rests at its sidewalk end.
// `vArrive` is the speed still carried at the arrival tick — a chain keeps it.
void CitySim::arriveOrChain(Agent& a, Real vArrive) {
    a.moving = false;
    a.speed = 0;
    int lastLink = a.route.links.back();
    // Rest at the ARRIVAL link's elevation — zeroing it parked bridge-deck
    // arrivals at ground level, under their own road.
    a.elevation = nav_->links[lastLink].layer * kLayerClearance;
    if (a.mode == Agent::Mode::Driver && wander_) {
        // WANDER (the agent lab): CHAIN straight into the next trip THIS
        // tick — no parking pose (which read on device as the car "jumping
        // backwards in time, then shooting forward") and no one-frame rest
        // INSIDE the junction box for cross traffic to overlap. The car
        // keeps its speed and heading through the node; steer() rotates it
        // onto the new route at the bounded yaw rate, like any other turn.
        a.restNode = nav_->links[lastLink].to;
        a.arrivedLink = lastLink;
        Vec2 keepHeading = a.heading;
        if (startWanderTrip(a, a.restNode, /*fromRest=*/false)) {
            a.speed = vArrive;
            a.heading = keepHeading;   // no snap: the yaw stays rate-limited
            return;
        }
        // No route anywhere (isolated node): pull onto the VERGE like a
        // schedule arrival — resting in-lane left a body no gap key, no box
        // rule, and no crash pass could see. The scheduling pass retries.
        {
            Vec2 dir = nav_->direction(lastLink);
            Vec2 right(dir.y, -dir.x);
            Real hw = nav_->links[lastLink].width * 0.5;
            Real back = 4.0 + static_cast<Real>((a.brain >> 3) & 7) * 1.3;
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
        Real back = 4.0 + static_cast<Real>((a.brain >> 3) & 7) * 1.3;   // 4..13 m
        back = std::min(back, nav_->links[lastLink].length * 0.5);
        a.pos = nav_->nodes[nav_->links[lastLink].to] - dir * back +
                right * (hw + 2.8);
        a.heading = dir;
    } else {
        // A walker rests at the end of its sidewalk (continuous with the final
        // motion — not snapped across the node, which was the old visible jump).
        a.pos = nav_->sidewalkPoint(lastLink, 1.0);
    }
    a.activity = (a.activity == Agent::Activity::Commuting) ? Agent::Activity::AtWork
                                                            : Agent::Activity::AtHome;
    a.restNode = nav_->links[lastLink].to;   // wander departs from here
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
                break;
            }
            ahead += nav_->links[nextLi].length;       // no car on this link; look one further
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

    // Pass 1: schedule transitions (AI agents only — the host drives players).
    // A stranded agent (work == home: no route was found at build) never departs.
    for (Agent& a : agents_) {
        if (a.playerControlled || a.released) continue;
        // WANDER mode (the agent lab): no schedule — the moment an agent is at
        // rest it picks a fresh random reachable goal and goes, so the lab's one
        // car laps the circuit continuously instead of parking for hours.
        if (wander_) {
            // Drivers CHAIN trips at arrival (see advance()); this pass only
            // launches the very first trip from the build pose, restarts a
            // walker, or recovers an agent whose last pick found no route.
            if (!a.moving) {
                int from = (a.restNode >= 0 && a.restNode < nav_->nodeCount())
                               ? a.restNode : a.home;
                // LAUNCH CLEARANCE (drivers): agents sharing a home node rest
                // near the same verge — launching them the same tick spawned
                // cars inside each other. Hold this launch while another moving
                // car is still near the departure node; index order staggers
                // same-node fleets a few metres apart, like cars pulling out.
                if (a.mode != Agent::Mode::Driver || launchClear(a, from))
                    startWanderTrip(a, from);
            }
            continue;
        }
        if (a.home == a.work) continue;
        switch (a.activity) {
            // A driver departs only once its spawn area is clear of moving cars
            // (launchClear) — it just waits out the traffic and retries next
            // step, like anyone pulling out of a parking spot.
            case Agent::Activity::AtHome:
                if (clockHours_ >= a.departWork && clockHours_ < a.departHome &&
                    (a.mode != Agent::Mode::Driver || launchClear(a, a.home))) {
                    a.activity = Agent::Activity::Commuting;
                    startTrip(a, a.home, a.work);
                }
                break;
            case Agent::Activity::AtWork:
                if ((clockHours_ >= a.departHome || clockHours_ < a.departWork) &&
                    (a.mode != Agent::Mode::Driver || launchClear(a, a.work))) {
                    a.activity = Agent::Activity::Returning;
                    startTrip(a, a.work, a.home);
                }
                break;
            default: break;
        }
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
            if (d2 >= rs * rs) continue;
            Real rvx = b.heading.x * b.speed - a.heading.x * a.speed;
            Real rvy = b.heading.y * b.speed - a.heading.y * a.speed;
            if (rvx * dx + rvy * dy >= 0) continue;   // separating: let them part
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
            // NOTE: do not zero speed here — this tick's motion (and steering)
            // already used it honestly; the hold branch freezes them next tick.
            auto crash = [](Agent& c) {
                if (c.crashTimer <= 0) {
                    c.crashTimer = 1.0 + static_cast<Real>((c.brain >> 5) & 3) * 0.5;
                    if (c.crashCount == 0) c.crashAnchor = c.pos;
                    ++c.crashCount;
                }
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
