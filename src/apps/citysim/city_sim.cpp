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
constexpr Real kJunctionApproach = 9.0, kJunctionSpeed = 4.0;
constexpr Real kSignalApproach = 14.0;          // start braking for a light this far out
constexpr Real kCarDecel = 6.0, kPedDecel = 3.0;
constexpr Real kLayerClearance = 5.8;   // bridge-deck height per grade layer
constexpr Real kCarMinTurnRadius = 6.0; // tightest arc a car can trace (m)

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
    clockHours_ = 6.0;
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
        for (int tries = 0; tries < 4 && a.work == a.home && n > 1; ++tries)
            a.work = static_cast<int>(rnd() % n);
        a.departWork = 7.5 + rndUnit() * 1.5;
        a.departHome = 16.5 + rndUnit() * 1.5;
        a.activity = Agent::Activity::AtHome;
        a.brain = rnd() | 1u;            // per-agent fault RNG (non-zero)
        a.pos = idlePose(a.home, a.mode);

        // A driver possesses a freshly-created car (two-way possession link).
        if (a.mode == Agent::Mode::Driver) {
            SimVehicle v;
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
    Real off = (mode == Agent::Mode::Driver) ? hw - 1.2 : hw + 1.2;
    if (off < 0.5) off = 0.5;
    return nav_->nodes[node] + right * off;
}

void CitySim::startTrip(Agent& a, int origin, int goal) {
    a.route = engine::findRoute(*nav_, origin, goal);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    if (!a.route.valid()) {
        a.moving = false;
        a.elevation = 0;
        a.pos = idlePose(goal, a.mode);
        a.activity = (a.activity == Agent::Activity::Commuting) ? Agent::Activity::AtWork
                                                               : Agent::Activity::AtHome;
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
    a.pos = (a.mode == Agent::Mode::Driver) ? nav_->laneCenter(li, a.lane, t)
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

void CitySim::advance(Agent& a, Real dt, Real gap) {
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
    if (car) {
        if (brainUnit(a) <= a.reliability) {
            engine::VisionCone cone;
            cone.origin = a.pos;
            cone.forward = a.heading;
            cone.range = 18.0;
            cone.halfAngleRad = 0.45;    // ~26 deg: a crosser in the lane ahead,
                                         // not someone standing on the far sidewalk
            Real obstacle = nearestObstacleAhead(cone, positions_);
            if (obstacle < 1e9)
                target = std::min(target, approachStop(obstacle - 4.0, kCarDecel, target));
        } else {
            ++faultCount_;
        }
    }

    Real minGap = car ? kCarMinGap : kPedMinGap;
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
            refreshPose(a);
            steer(a, dt);
            return;
        }
    }

    Real motion = a.speed * dt;
    const int legCount = static_cast<int>(a.route.links.size());
    while (motion > 0 && a.leg < legCount) {
        Real L = nav_->links[a.route.links[a.leg]].length;
        Real remain = L - a.distOnLeg;
        if (motion < remain) { a.distOnLeg += motion; motion = 0; }
        else { motion -= remain; ++a.leg; a.distOnLeg = 0; }
    }

    if (a.leg >= legCount) {
        int dest = nav_->links[a.route.links.back()].to;
        a.moving = false;
        a.speed = 0;
        a.elevation = 0;
        a.pos = idlePose(dest, a.mode);
        a.activity = (a.activity == Agent::Activity::Commuting) ? Agent::Activity::AtWork
                                                               : Agent::Activity::AtHome;
        a.route.links.clear();
    } else {
        refreshPose(a);
        steer(a, dt);
    }
}

void CitySim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    gaps_.assign(agents_.size(), INF);
    std::unordered_map<long long, std::vector<std::pair<Real, int>>> lanes;
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        const Agent& a = agents_[i];
        if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
        int li = a.route.links[a.leg];
        int laneKey = (a.mode == Agent::Mode::Driver) ? a.lane : 1024;
        long long key = static_cast<long long>(li) * 4096 + laneKey;
        lanes[key].push_back({a.distOnLeg, i});
    }
    for (auto& kv : lanes) {
        std::vector<std::pair<Real, int>>& v = kv.second;
        std::sort(v.begin(), v.end(), [](const std::pair<Real, int>& a,
                                          const std::pair<Real, int>& b) {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });
        for (std::size_t k = 0; k + 1 < v.size(); ++k)
            gaps_[v[k].second] = v[k + 1].first - v[k].first;
    }
}

void CitySim::step(Real dt, Real hoursPerSecond) {
    if (!nav_ || agents_.empty()) return;

    clockHours_ += dt * hoursPerSecond;
    clockHours_ = std::fmod(clockHours_, 24.0);
    if (clockHours_ < 0) clockHours_ += 24.0;
    signals_.update(dt);

    // Pass 1: schedule transitions (AI agents only — the host drives players).
    for (Agent& a : agents_) {
        if (a.playerControlled) continue;
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

    // Pass 2: leading gaps. Pass 3: advance.
    computeGaps();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.playerControlled) continue;
        if (a.moving) advance(a, dt, gaps_[i]);
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
