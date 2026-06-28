#include "agent_sim.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr Real kWalkSpeed = 1.4;   // m/s, comfortable pedestrian pace
constexpr Real kCarAccel  = 6.0;   // m/s^2
constexpr Real kPedAccel  = 1.0;   // m/s^2
}  // namespace

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

void AgentSim::advance(Agent& a, Real dt) {
    if (!a.moving) return;

    int li = a.route.links[a.leg];
    Real target = (a.kind == AgentKind::Car) ? classSpeed(nav->links[li].klass) : kWalkSpeed;
    Real accel = (a.kind == AgentKind::Car) ? kCarAccel : kPedAccel;
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

void AgentSim::step(Real dt, Real hoursPerSecond) {
    if (!nav || agentList.empty()) return;

    clockHours += dt * hoursPerSecond;
    clockHours = std::fmod(clockHours, 24.0);
    if (clockHours < 0) clockHours += 24.0;

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
        if (a.moving) advance(a, dt);
    }
}

}  // namespace engine
