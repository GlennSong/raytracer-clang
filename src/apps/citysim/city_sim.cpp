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

// The fleet of body slots (ADR-0060 Phase 4). A sedan is the player's body
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
constexpr Real kPedClearance = 4.0;     // a car aims to stop this far short of a ped/player
constexpr Real kPedHardStop = 3.0;      // and will NOT roll closer than this (a real wall)

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
    faultCount_ = 0;
    rng_ = seed ? seed : 0x6c078965u;
    signals_.build(graph);

    const int n = graph.nodeCount();
    if (n == 0) return;

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
        a.pos = idlePose(a.home, a.mode);

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
    // Driver idles where it would drive (lane 0 centre); a pedestrian waits just
    // beyond the kerb on the same (right) side.
    Real off = (mode == Agent::Mode::Driver) ? laneSpacing(nav_->links[li]) * 0.5
                                             : hw + 1.2;
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
    a.moving = true;
    refreshPose(a);
    a.heading = nav_->direction(a.route.links.front());   // start pointed down leg 0
}

void CitySim::refreshPose(Agent& a) {
    if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) return;
    int li = a.route.links[a.leg];
    Real L = nav_->links[li].length;
    Real t = L > 1e-9 ? a.distOnLeg / L : 0.0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    a.pos = (a.mode == Agent::Mode::Driver)
                ? nav_->laneCenter(li, a.lane, t, laneSpacing(nav_->links[li]))
                : nav_->sidewalkPoint(li, t);
    a.elevation = nav_->links[li].layer * kLayerClearance;
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
    Real target = car ? engine::classSpeed(nav_->links[li].klass) : kWalkSpeed;
    Real accel = car ? kCarAccel : kPedAccel;
    if (nav_->isJunction(nav_->links[li].to)) {
        Real distToEnd = nav_->links[li].length - a.distOnLeg;
        if (car && distToEnd < kJunctionApproach) target = std::min(target, kJunctionSpeed);
        // Obey the stoplight: ease to a stop at the line unless this approach is
        // green. A pedestrian on a green approach crosses the PERPENDICULAR road
        // (which is then red), so the same condition is safe for cars and peds.
        if (distToEnd < kSignalApproach && signals_.hasSignal(li) &&
            signals_.stateForLink(li) != SignalState::Green) {
            target = std::min(target, approachStop(distToEnd, car ? kCarDecel : kPedDecel,
                                                   target));
        }
    }

    // Perception: brake for a PEDESTRIAN (or the player) the car sees crossing
    // ahead in its vision cone — the safety case car-following can't cover. We do
    // NOT brake for other AI cars here: car-vs-car conflicts are resolved by lanes
    // (opposing traffic sits on the other side), same-lane car-following (the gap
    // cap below), and the traffic signals. Braking for every car in a wide cone
    // made oncoming traffic and cross traffic at junctions stop for each other and
    // deadlock — a snarl with no way to clear. `positions_` therefore holds only
    // pedestrians + the player (see step()).
    // Imperfect: with probability (1 - reliability) the agent misses it this step
    // (a fault), so it may not brake in time.
    Real seenAhead = std::numeric_limits<Real>::infinity();   // dist to a ped/player ahead
    if (car) {
        if (brainUnit(a) <= a.reliability) {
            engine::VisionCone cone;
            cone.origin = a.pos;
            cone.forward = a.heading;
            cone.range = 18.0;
            cone.halfAngleRad = 0.45;    // ~26 deg: a crosser in the lane ahead,
                                         // not someone standing on the far sidewalk
            seenAhead = nearestObstacleAhead(cone, positions_);
            if (seenAhead < 1e9)
                target = std::min(target, approachStop(seenAhead - kPedClearance, kCarDecel,
                                                       target));
        } else {
            ++faultCount_;
        }
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
        if (nav_->isJunction(toNode) && signals_.hasSignal(li) &&
            signals_.stateForLink(li) != SignalState::Green) {
            Real stopLine = nav_->links[li].length - 0.5;
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
        // Rest at the very end of the last link (continuous with the final motion),
        // NOT snapped to an idle pose on some other side of the node — that snap
        // was the visible jump.
        int lastLink = a.route.links.back();
        a.pos = (a.mode == Agent::Mode::Driver)
                    ? nav_->laneCenter(lastLink, a.lane, 1.0, laneSpacing(nav_->links[lastLink]))
                    : nav_->sidewalkPoint(lastLink, 1.0);
        a.activity = (a.activity == Agent::Activity::Commuting) ? Agent::Activity::AtWork
                                                               : Agent::Activity::AtHome;
        a.route.links.clear();
    } else {
        refreshPose(a);
        steer(a, dt);
        // Driver FSM (ADR-0060): label what's governing the car this step, from
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
    signals_.update(dt);

    // Pass 1: schedule transitions (AI agents only — the host drives players).
    // A stranded agent (work == home: no route was found at build) never departs.
    for (Agent& a : agents_) {
        if (a.playerControlled || a.home == a.work) continue;
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

    // Snapshot the positions cars must yield to — pedestrians and the player —
    // so perception sees a consistent world this step. AI cars are deliberately
    // excluded (see advance()): lanes + car-following + signals govern car-vs-car,
    // and braking for cross/oncoming cars in the cone deadlocked traffic.
    positions_.clear();
    for (const Agent& a : agents_)
        if (a.mode == Agent::Mode::Pedestrian || a.playerControlled)
            positions_.push_back(a.pos);
    // The live player (host-injected) is an obstacle too — cars brake for it.
    for (const Vec2& o : externalObstacles_) positions_.push_back(o);

    // Pass 2: leading gaps. Pass 3: advance.
    computeGaps();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.playerControlled) continue;
        if (a.moving) advance(a, dt, gaps_[i], minGaps_[i]);
    }

    // Reactive pedestrian behaviour (ADR-0060): each walker acts on what it SEES
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
        engine::VisionCone cone;
        cone.origin = a.pos; cone.forward = travel;
        cone.range = kPedVisionRange; cone.halfAngleRad = kPedVisionHalfAngle;
        Real bias = 0; bool saw = false;
        auto consider = [&](const Vec2& p) {
            if (!engine::sees(cone, p)) return;
            Real fd = engine::forwardDistance(cone, p);
            if (fd <= 0.1 || fd >= kPedVisionRange) return;
            saw = true;
            Real side = (a.pos.x - p.x) * rightv.x + (a.pos.y - p.y) * rightv.y;
            Real w = (kPedVisionRange - fd) / kPedVisionRange;   // nearer -> stronger
            bias += (side >= 0 ? 1.0 : -1.0) * w;                // pass on the side I'm already on
        };
        for (std::size_t j = 0; j < agents_.size(); ++j) {
            if (j == i) continue;
            if (agents_[j].mode == Agent::Mode::Pedestrian && agents_[j].moving)
                consider(agents_[j].pos);
        }
        for (const Vec2& o : externalObstacles_) consider(o);
        for (const Vec2& o : staticObstacles_) consider(o);   // steer around signal poles

        // Move the lean toward its target at a bounded RATE (continuous, not a
        // pop): grows while something is in view, decays back to the path when
        // clear. This is what lets you herd a walker like a boid.
        Real target = std::max(Real(-1), std::min(Real(1), bias)) * kPedMaxLateral;
        Real prev = a.lateralOffset;
        Real maxDelta = kPedLateralRate * dt;
        Real delta = std::max(-maxDelta, std::min(maxDelta, target - prev));
        a.lateralOffset = prev + delta;
        a.state = saw ? Agent::State::Avoiding : Agent::State::Walking;

        a.pos.x += rightv.x * a.lateralOffset;
        a.pos.y += rightv.y * a.lateralOffset;
        // Turn to face where it's actually moving (forward walk + the sideways
        // drift), so the body visibly rotates as it steers away.
        Vec2 vel(travel.x * kWalkSpeed + rightv.x * (delta / dt),
                 travel.y * kWalkSpeed + rightv.y * (delta / dt));
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

    // Hard radial push-out from static obstacles (signal poles): a walker never
    // ends up standing inside a pole — if a step would leave it within the pole's
    // clearance, shove it back out along the pole→ped direction. The cone bias
    // above makes it lean away in advance; this is the physical backstop.
    for (Agent& a : agents_) {
        if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
        for (const Vec2& o : staticObstacles_) {
            Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
            Real d = std::sqrt(dx * dx + dy * dy);
            if (d > 1e-4 && d < kPoleClearance) {
                Real push = kPoleClearance - d;
                a.pos.x += dx / d * push;
                a.pos.y += dy / d * push;
            } else if (d <= 1e-4) {
                a.pos.x += kPoleClearance;   // dead-centre: pick a direction
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
