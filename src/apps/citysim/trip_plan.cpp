#include "trip_plan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace citysim {

namespace {
// How many of the nodes nearest each end are tried before a leg is called impossible -- the nearest node can sit on
// a piece of the graph the other end cannot reach (sendAgentTo tries the same few).
constexpr int kTryNodes = 8;

std::string minutesText(Real m) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", m);
    return b;
}
}  // namespace

int TravelCatalog::roadKindOf(engine::RoadClass k) const {
    for (std::size_t i = 0; i < roadKinds.size(); ++i)
        if (std::find(roadKinds[i].classes.begin(), roadKinds[i].classes.end(), k) != roadKinds[i].classes.end())
            return static_cast<int>(i);
    return -1;
}

int TravelCatalog::find(const std::string& name) const {
    for (std::size_t i = 0; i < modes.size(); ++i)
        if (modes[i].name == name) return static_cast<int>(i);
    return -1;
}

bool planLeg(const engine::NavGraph& nav, const TravelCatalog& cat, int mode, Vec2 from, Vec2 to, TripLeg& out) {
    if (mode < 0 || mode >= static_cast<int>(cat.modes.size()) || nav.nodeCount() == 0) return false;
    const TravelMode& m = cat.modes[static_cast<std::size_t>(mode)];
    const bool onFoot = !m.roads;
    const std::vector<int> ends = nav.nearestNodes(to, kTryNodes);
    for (int s : nav.nearestNodes(from, kTryNodes)) {
        // one flood from this start says which end, if any, it reaches: one search, not one per end
        const std::vector<char> reach = engine::reachableFrom(nav, s, onFoot);
        for (int e : ends) {
            engine::Route r;
            if (e != s) {
                if (!reach[static_cast<std::size_t>(e)]) continue;
                r = engine::findRoute(nav, s, e, onFoot);
                if (!r.valid()) continue;
            }
            TripLeg leg;
            leg.mode = mode;
            leg.from = from;
            leg.to = to;
            leg.fromNode = s;
            leg.toNode = e;
            leg.metres = (from - nav.nodes[static_cast<std::size_t>(s)]).length() + r.length(nav) +
                         (nav.nodes[static_cast<std::size_t>(e)] - to).length();
            leg.route = std::move(r);
            if (m.roads) {
                // THE ROUTER'S OWN TIME (ADR-0416): each road class at its speed and a few seconds a street junction,
                // what findRoute chose this route by; the bits off the network at a local street's speed
                const Real off = leg.metres - leg.route.length(nav);
                leg.minutes = (engine::routeSeconds(nav, leg.route) + off / engine::classSpeed(engine::RoadClass::Local)) / 60.0;
                // ...and the kinds of road it is on, with the bits off the network counted as the street they join
                leg.kindMetres.assign(cat.roadKinds.size(), 0);
                leg.kindStretches.assign(cat.roadKinds.size(), 0);
                int was = -1;
                for (int li : leg.route.links) {
                    const int k = cat.roadKindOf(nav.links[static_cast<std::size_t>(li)].klass);
                    if (k >= 0) {
                        leg.kindMetres[static_cast<std::size_t>(k)] += nav.links[static_cast<std::size_t>(li)].length;
                        if (k != was) ++leg.kindStretches[static_cast<std::size_t>(k)];
                    }
                    was = k;
                }
                const int ends = cat.roadKindOf(engine::RoadClass::Local);
                if (ends >= 0) leg.kindMetres[static_cast<std::size_t>(ends)] += off;
            } else {
                leg.minutes = leg.metres / m.speed / 60.0;
            }
            leg.minutes += m.startMinutes + m.endMinutes;
            out = std::move(leg);
            return true;
        }
    }
    return false;
}

TripPlan planTrip(const engine::NavGraph& nav, const TravelCatalog& cat, const TripRequest& req) {
    TripPlan plan;
    plan.from = req.from;
    plan.to = req.to;
    for (std::size_t i = 0; i < cat.modes.size(); ++i) {
        const TravelMode& m = cat.modes[i];
        const int mode = static_cast<int>(i);
        TripOption o;
        o.mode = mode;
        auto leg = [&](int legMode, Vec2 a, Vec2 b, const std::string& whyNot) {
            TripLeg l;
            if (!planLeg(nav, cat, legMode, a, b, l)) { o.why = whyNot; return false; }
            o.legs.push_back(std::move(l));
            return true;
        };
        if (std::find(req.ruledOut.begin(), req.ruledOut.end(), mode) != req.ruledOut.end()) {
            o.why = "ruled out (a leg of it could not be done)";
        } else if (!m.ownVehicle) {
            o.possible = leg(mode, req.from, req.to, "no route on the " + std::string(m.roads ? "roads" : "footpaths"));
        } else {
            const OwnedVehicle* v = nullptr;
            for (const OwnedVehicle& ov : req.vehicles)
                if (ov.mode == mode) { v = &ov; break; }
            const std::string access = m.access >= 0 ? cat.modes[static_cast<std::size_t>(m.access)].name : "?";
            if (!v) {
                o.why = "no " + m.name;
            } else {
                o.possible = leg(m.access, req.from, v->at, "no route to the " + m.name + " by " + access) &&
                             leg(mode, v->at, v->leaveAt, "no route by " + m.name) &&
                             leg(m.access, v->leaveAt, req.to, "no route from where the " + m.name + " is left by " + access);
            }
        }
        if (o.possible) {
            for (const TripLeg& l : o.legs) {
                o.minutes += l.minutes;
                o.weighted += l.minutes * cat.modes[static_cast<std::size_t>(l.mode)].weight;
            }
        } else {
            o.legs.clear();
        }
        plan.options.push_back(std::move(o));
    }
    for (std::size_t i = 0; i < plan.options.size(); ++i) {
        const TripOption& o = plan.options[i];
        if (o.possible && (plan.chosen < 0 || o.weighted < plan.options[static_cast<std::size_t>(plan.chosen)].weighted))
            plan.chosen = static_cast<int>(i);
    }
    for (const TripOption& o : plan.options) plan.reason += (plan.reason.empty() ? "" : ", ") + describeOption(cat, o);
    plan.reason += ": " + (plan.chosen >= 0 ? cat.modes[static_cast<std::size_t>(plan.choice()->mode)].name
                                            : std::string("no way there"));
    return plan;
}

std::string describeOption(const TravelCatalog& cat, const TripOption& o) {
    const std::string& name = cat.modes[static_cast<std::size_t>(o.mode)].name;
    if (!o.possible) return name + " -- " + o.why;
    std::string s = name + " " + minutesText(o.minutes) + " min";
    if (o.legs.size() > 1) {
        s += " (";
        for (std::size_t i = 0; i < o.legs.size(); ++i)
            s += (i ? " + " : "") + minutesText(o.legs[i].minutes) + " " +
                 cat.modes[static_cast<std::size_t>(o.legs[i].mode)].name;
        s += ")";
    }
    return s;
}

std::string metresText(Real m) {
    const long v = std::lround(std::max(Real(0), m));
    std::string digits = std::to_string(v), s;
    for (std::size_t i = 0; i < digits.size(); ++i) {
        if (i > 0 && (digits.size() - i) % 3 == 0) s += ',';
        s += digits[i];
    }
    return s + " m";
}

std::string describeRoads(const TravelCatalog& cat, const TripLeg& leg) {
    std::string s;
    for (std::size_t k = 0; k < leg.kindMetres.size() && k < cat.roadKinds.size(); ++k) {
        if (leg.kindMetres[k] < 0.5) continue;
        const RoadKind& rk = cat.roadKinds[k];
        const int n = k < leg.kindStretches.size() ? leg.kindStretches[k] : 0;
        s += (s.empty() ? "" : ", ") +
             (rk.count ? std::to_string(n) + " " + (n == 1 && !rk.one.empty() ? rk.one : rk.name)
                       : metresText(leg.kindMetres[k]) + " " + rk.name);
    }
    return s;
}

Real kindMetres(const TravelCatalog& cat, const TripLeg& leg, const std::string& name) {
    for (std::size_t k = 0; k < cat.roadKinds.size() && k < leg.kindMetres.size(); ++k)
        if (cat.roadKinds[k].name == name) return leg.kindMetres[k];
    return 0;
}

}  // namespace citysim
