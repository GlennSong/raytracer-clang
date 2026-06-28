#include "agent_sim.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace engine {

namespace {
constexpr Real kWalkSpeed = 1.4;   // m/s, comfortable pedestrian pace
constexpr Real kCarAccel  = 6.0;   // m/s^2
constexpr Real kPedAccel  = 1.0;   // m/s^2

// Car-following spacing per kind: minGap = full stop distance (bumper-to-bumper
// plus a buffer), slowZone = start easing off the throttle below this gap.
constexpr Real kCarMinGap   = 5.0;
constexpr Real kCarSlowZone = 14.0;
constexpr Real kPedMinGap   = 0.8;
constexpr Real kPedSlowZone = 2.5;
}  // namespace

Real carFollowingCap(Real freeSpeed, Real gap, Real minGap, Real slowZone) {
    if (gap <= minGap) return 0.0;
    if (gap >= slowZone) return freeSpeed;
    return freeSpeed * (gap - minGap) / (slowZone - minGap);
}

uint32_t AgentSim::nextRandom() {
    // xorshift32 — the house deterministic RNG (matches procgen's Rng).
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}

Real AgentSim::randomUnit() {
    return (nextRandom() >> 8) * (1.0 / 16777216.0);
}

void AgentSim::build(const NavGraph& graph, int carCount, int pedCount, uint32_t seed) {
    nav = &graph;
    agentList.clear();
    clockHours = 6.0;
    rngState = seed ? seed : 0x6c078965u;

    const int n = graph.nodeCount();
    if (n == 0) return;

    const int total = carCount + pedCount;
    agentList.reserve(total);
    for (int i = 0; i < total; ++i) {
        Agent a;
        a.kind = (i < carCount) ? AgentKind::Car : AgentKind::Pedestrian;
        a.home = static_cast<int>(nextRandom() % n);
        a.work = static_cast<int>(nextRandom() % n);
        // Nudge work off home so the commute is a real trip (best-effort).
        for (int tries = 0; tries < 4 && a.work == a.home && n > 1; ++tries)
            a.work = static_cast<int>(nextRandom() % n);
        a.departWork = 7.5 + randomUnit() * 1.5;     // ~07:30–09:00
        a.departHome = 16.5 + randomUnit() * 1.5;    // ~16:30–18:00
        a.activity = AgentActivity::AtHome;
        a.pos = graph.nodes[a.home];
        agentList.push_back(a);
    }
}

void AgentSim::startTrip(Agent& a, int originNode, int goalNode) {
    a.route = findRoute(*nav, originNode, goalNode);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    if (!a.route.valid()) {
        // Unreachable (or origin == goal): treat as already arrived so the daily
        // loop still advances instead of stalling.
        a.moving = false;
        a.pos = nav->nodes[goalNode];
        a.activity = (a.activity == AgentActivity::Commuting) ? AgentActivity::AtWork
                                                              : AgentActivity::AtHome;
        return;
    }
    // Cars pick a lane within the first link; pedestrians walk the verge (lane 0).
    int lanes = nav->links[a.route.links.front()].lanes;
    a.lane = (a.kind == AgentKind::Car && lanes > 1)
                 ? static_cast<int>(nextRandom() % static_cast<uint32_t>(lanes))
                 : 0;
    a.moving = true;
    refreshPose(a);
}

void AgentSim::refreshPose(Agent& a) {
    if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) return;
    int li = a.route.links[a.leg];
    Real L = nav->links[li].length;
    Real t = L > 1e-9 ? a.distOnLeg / L : 0.0;
    if (t < 0) t = 0; else if (t > 1) t = 1;
    a.pos = (a.kind == AgentKind::Car) ? nav->laneCenter(li, a.lane, t)
                                       : nav->sidewalkPoint(li, t);
    a.heading = nav->direction(li);
}

void AgentSim::advance(Agent& a, Real dt, Real gap) {
    if (!a.moving) return;

    int li = a.route.links[a.leg];
    Real target = (a.kind == AgentKind::Car) ? classSpeed(nav->links[li].klass) : kWalkSpeed;
    Real accel = (a.kind == AgentKind::Car) ? kCarAccel : kPedAccel;
    // Don't drive into the agent ahead: cap the target by the leading gap.
    Real minGap = (a.kind == AgentKind::Car) ? kCarMinGap : kPedMinGap;
    Real slowZone = (a.kind == AgentKind::Car) ? kCarSlowZone : kPedSlowZone;
    target = carFollowingCap(target, gap, minGap, slowZone);
    a.speed = std::min(target, a.speed + accel * dt);

    Real motion = a.speed * dt;
    const int legCount = static_cast<int>(a.route.links.size());
    while (motion > 0 && a.leg < legCount) {
        Real L = nav->links[a.route.links[a.leg]].length;
        Real remain = L - a.distOnLeg;
        if (motion < remain) { a.distOnLeg += motion; motion = 0; }
        else { motion -= remain; ++a.leg; a.distOnLeg = 0; }
    }

    if (a.leg >= legCount) {
        // Arrived at the destination node (the head of the last link).
        int dest = nav->links[a.route.links.back()].to;
        a.moving = false;
        a.speed = 0;
        a.pos = nav->nodes[dest];
        a.activity = (a.activity == AgentActivity::Commuting) ? AgentActivity::AtWork
                                                              : AgentActivity::AtHome;
        a.route.links.clear();
    } else {
        refreshPose(a);
    }
}

void AgentSim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    gaps_.assign(agentList.size(), INF);

    // Bucket moving agents by (link, lane): cars share a lane bucket, pedestrians
    // share one verge bucket per link (distinct from any car lane).
    std::unordered_map<long long, std::vector<std::pair<Real, int>>> lanes;
    for (int i = 0; i < static_cast<int>(agentList.size()); ++i) {
        const Agent& a = agentList[i];
        if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
        int li = a.route.links[a.leg];
        int laneKey = (a.kind == AgentKind::Car) ? a.lane : 1024;
        long long key = static_cast<long long>(li) * 4096 + laneKey;
        lanes[key].push_back({a.distOnLeg, i});
    }

    // On each lane, an agent's leader is the next one further along; the gap is
    // their centre-to-centre distance. Sort by (dist, index) for determinism.
    for (auto& kv : lanes) {
        std::vector<std::pair<Real, int>>& v = kv.second;
        std::sort(v.begin(), v.end(), [](const std::pair<Real, int>& a,
                                          const std::pair<Real, int>& b) {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });
        for (std::size_t k = 0; k + 1 < v.size(); ++k)
            gaps_[v[k].second] = v[k + 1].first - v[k].first;   // frontmost keeps INF
    }
}

void AgentSim::step(Real dt, Real hoursPerSecond) {
    if (!nav || agentList.empty()) return;

    clockHours += dt * hoursPerSecond;
    clockHours = std::fmod(clockHours, 24.0);
    if (clockHours < 0) clockHours += 24.0;

    // Pass 1: schedule transitions — may start trips, but moves no one yet.
    for (Agent& a : agentList) {
        switch (a.activity) {
            case AgentActivity::AtHome:
                if (clockHours >= a.departWork && clockHours < a.departHome) {
                    a.activity = AgentActivity::Commuting;
                    startTrip(a, a.home, a.work);
                }
                break;
            case AgentActivity::AtWork:
                if (clockHours >= a.departHome || clockHours < a.departWork) {
                    a.activity = AgentActivity::Returning;
                    startTrip(a, a.work, a.home);
                }
                break;
            case AgentActivity::Commuting:
            case AgentActivity::Returning:
                break;
        }
    }

    // Pass 2: leading gaps from everyone's current position. Pass 3: advance.
    computeGaps();
    for (std::size_t i = 0; i < agentList.size(); ++i)
        if (agentList[i].moving) advance(agentList[i], dt, gaps_[i]);
}

}  // namespace engine
