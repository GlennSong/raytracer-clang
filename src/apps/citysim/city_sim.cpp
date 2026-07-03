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
        for (int k = 0; k <= n; ++k) {
            Real t = static_cast<Real>(k) / n;
            Vec2 p = (a.mode == Agent::Mode::Driver)
                         ? nav_->laneCenter(li, a.lane, t, spacing)
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
    clockHours_ = 8.5;   // start mid morning-rush so agents commute right away
    simSeconds_ = 0;
    faultCount_ = 0;
    rng_ = seed ? seed : 0x6c078965u;
    signals_.build(graph);

    const int n = graph.nodeCount();
    if (n == 0) return;

    // Junction boxes (centre + radius = widest incident half-width): the bridge's
    // don't-block-the-box and spawn-placement checks read these.
    junctions_.clear();
    for (int v = 0; v < n; ++v) {
        if (!graph.isJunction(v)) continue;
        Real r = 0;
        for (int li : graph.outLinks[v]) r = std::max(r, graph.links[li].width * 0.5);
        junctions_.push_back({graph.nodes[v], r});
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
            bool ok = a.work != a.home && engine::findRoute(graph, a.home, a.work).valid();
            for (int tries = 0; tries < 8 && !ok && n > 1; ++tries) {
                a.work = static_cast<int>(rnd() % n);
                ok = a.work != a.home && engine::findRoute(graph, a.home, a.work).valid();
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
        a.pos = idlePose(a.home, a.mode);
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

Vec2 CitySim::idlePose(int node, Agent::Mode mode) const {
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
    return nav_->nodes[node] + right * off;
}

void CitySim::startTrip(Agent& a, int origin, int goal) {
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
        a.pos = idlePose(origin, a.mode);
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
        return (a.mode == Agent::Mode::Driver)
                   ? nav_->laneCenter(link, a.lane, t, laneSpacing(nav_->links[link]))
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

void CitySim::advance(Agent& a, Real dt, Real gap, Real minGap) {
    if (!a.moving) return;
    int li = a.route.links[a.leg];
    bool car = a.mode == Agent::Mode::Driver;
    // Nominal pace scaled by personality (ADR-0062): drivers and walkers each
    // hold their OWN fraction of the limit, so traffic doesn't move in lockstep.
    // Junction/signal caps below are shared road rules and stay unscaled.
    Real target = (car ? engine::classSpeed(nav_->links[li].klass) : kWalkSpeed) *
                  a.speedFactor;
    Real accel = car ? kCarAccel : kPedAccel;
    // Where this agent's signal STOP LINE sits, measured back from the node. A
    // walker holds at the corner; a car holds with its front bumper short of the
    // painted zebra band on its approach: junction box radius (the mouth) + the
    // band + a margin + half its own body. Stopping at the node itself parked a
    // legally-waiting car in the middle of the intersection (device round 3).
    Real stopSetback = 0.5;
    int toNode = nav_->links[li].to;
    if (car && nav_->isJunction(toNode)) {
        Real jr = 0;
        for (int out : nav_->outLinks[toNode])
            jr = std::max(jr, nav_->links[out].width * 0.5);
        Real halfLen = 2.1;   // sedan fallback
        if (a.vehicle >= 0 && a.vehicle < static_cast<int>(vehicles_.size()))
            halfLen = vehicles_[a.vehicle].length * 0.5;
        stopSetback = jr + kCrosswalkFarEdge + kStopLineMargin + halfLen;
    }
    if (nav_->isJunction(toNode)) {
        Real distToEnd = nav_->links[li].length - a.distOnLeg;
        if (car && distToEnd < kJunctionApproach) target = std::min(target, kJunctionSpeed);
        // Obey the stoplight: ease to a stop at the LINE unless this approach is
        // green. A pedestrian on a green approach crosses the PERPENDICULAR road
        // (which is then red), so the same condition is safe for cars and peds.
        Real distToLine = distToEnd - stopSetback;
        if (distToLine < kSignalApproach && signals_.hasSignal(li) &&
            signals_.stateForLink(li) != SignalState::Green) {
            target = std::min(target, approachStop(distToLine, car ? kCarDecel : kPedDecel,
                                                   target));
        }
    }

    // Perception (ADR-0063: sense -> remember -> predict -> decide -> act). Each
    // tick the car SENSES pedestrians and the player through its 2.5D sensor —
    // the forward wedge plus a height band, so a walker crossing the overpass is
    // not a phantom to brake for — and REMEMBERS the sightings as tracks with
    // velocity estimates. It then acts on the MEMORY, not the snapshot: a person
    // who slipped out of the cone is still yielded to where they're HEADING for a
    // few seconds (object permanence), and a crosser on a collision course is
    // braked for BEFORE entering the lane (time-to-collision on the tracked
    // velocity). We still do NOT sense other AI cars: car-vs-car is resolved by
    // lanes, same-lane car-following (the gap cap below) and the signals —
    // braking for every car in a wide cone deadlocked junctions (see step()).
    // Imperfect: with probability (1 - reliability) this step's sighting is
    // missed (a fault) — memory makes a missed frame degrade gracefully instead
    // of blinding the car outright.
    Real seenAhead = std::numeric_limits<Real>::infinity();   // dist to a person ahead
    if (car) {
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
        if (nav_->isJunction(toNode) && signals_.hasSignal(li) &&
            signals_.stateForLink(li) != SignalState::Green) {
            // A link shorter than the setback still gets a usable line partway
            // down it — the car holds early rather than never entering at all.
            Real L = nav_->links[li].length;
            Real stopLine = std::max(L - stopSetback, std::min(L - 0.5, L * 0.4));
            Real room = std::max(Real(0), stopLine - a.distOnLeg);
            Real motion = std::min(a.speed * dt, room);
            if (motion < a.speed * dt) a.speed = 0;   // held at the line
            a.distOnLeg += motion;
            if (car) a.state = Agent::State::Waiting;   // FSM: stopped for a red
            refreshPose(a);
            steer(a, dt);
            return;
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
        else { motion -= remain; ++a.leg; a.distOnLeg = 0; }
    }

    if (a.leg >= legCount) {
        a.moving = false;
        a.speed = 0;
        a.elevation = 0;
        int lastLink = a.route.links.back();
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
        a.route.links.clear();
    } else {
        refreshPose(a);
        steer(a, dt);
        // Driver FSM (ADR-0061): label what's governing the car this step, from
        // what it sees, so the behaviour is legible (debug widgets read a.state).
        // Precedence mirrors how the speed was actually capped above: a ped/player
        // in the cone (Yielding) dominates a same-lane leader (Following), which
        // dominates a heading change at the coming node (Turning); otherwise the
        // car runs free (Cruising). The Waiting-at-a-red case returned earlier.
        if (car) {
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
    }
}

void CitySim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    gaps_.assign(agents_.size(), INF);
    minGaps_.assign(agents_.size(), kCarMinGap);   // overwritten where a leader exists
    auto laneKeyOf = [](const Agent& a, int li) {
        int laneKey = (a.mode == Agent::Mode::Driver) ? a.lane : 1024;
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
            if (!a.moving) {
                const int n = nav_->nodeCount();
                int from = (a.restNode >= 0 && a.restNode < n) ? a.restNode : a.home;
                for (int tries = 0; tries < 8 && n > 1; ++tries) {
                    int goal = static_cast<int>(rnd() % static_cast<uint32_t>(n));
                    if (goal == from) continue;
                    if (!engine::findRoute(*nav_, from, goal).valid()) continue;
                    a.activity = Agent::Activity::Commuting;
                    startTrip(a, from, goal);
                    break;
                }
            }
            continue;
        }
        if (a.home == a.work) continue;
        switch (a.activity) {
            case Agent::Activity::AtHome:
                if (clockHours_ >= a.departWork && clockHours_ < a.departHome) {
                    a.activity = Agent::Activity::Commuting;
                    startTrip(a, a.home, a.work);
                }
                break;
            case Agent::Activity::AtWork:
                if (clockHours_ >= a.departHome || clockHours_ < a.departWork) {
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
        if (a.mode == Agent::Mode::Pedestrian || a.playerControlled)
            sensed_.push_back({a.pos, a.elevation, static_cast<int>(i)});
    }
    // The live player (host-injected) is a body too — cars brake for it.
    for (std::size_t k = 0; k < externalObstacles_.size(); ++k)
        sensed_.push_back({externalObstacles_[k], 0.0, -(1 + static_cast<int>(k))});

    // Pass 2: leading gaps. Pass 3: advance.
    computeGaps();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.playerControlled || a.released) continue;
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
        if (a.moving) advance(a, dt, gaps_[i], minGaps_[i]);
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
