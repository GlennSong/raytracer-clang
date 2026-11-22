#include "trip_plan.h"

#include "../../engine/procgen/city/roads/road_model.h"
#include "../driving/driving.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace citysim {

namespace {
// Two lengths of time in minutes this close are the same (a leg exactly as long as its way's longest is allowed).
constexpr Real kSameMinutes = 1e-9;

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

int TravelCatalog::trait(const std::string& name) const {
    for (std::size_t i = 0; i < traits.size(); ++i)
        if (traits[i] == name) return static_cast<int>(i);
    return -1;
}

int TravelCatalog::find(const std::string& name) const {
    for (std::size_t i = 0; i < modes.size(); ++i)
        if (modes[i].name == name) return static_cast<int>(i);
    return -1;
}

bool TripNetwork::valid() const { return model && model->built && walker && !walker->open.empty(); }

bool planLeg(const TripNetwork& net, const TravelCatalog& cat, int mode, Vec2 from, Vec2 to, TripLeg& out,
             uint32_t vehicleClass, const engine::PathPoint* fromLane) {
    if (mode < 0 || mode >= static_cast<int>(cat.modes.size()) || !net.valid()) return false;
    const TravelMode& m = cat.modes[static_cast<std::size_t>(mode)];
    const engine::RoadModel& model = *net.model;
    TripLeg leg;
    leg.mode = mode;
    leg.from = from;
    leg.to = to;
    if (m.roads) {
        // ON THE ROADS: from the kerb lane beside one end to the kerb lane beside the other, as the driving routes its
        // cars (each lane and turn at the limit it is driven at). Getting from a space into its kerb lane and back into
        // a space at the end is the way's start and end minutes (pulling out, parking), so the leg is the route alone.
        if (!net.driving) return false;
        const engine::KerbLanePlace a = fromLane ? engine::KerbLanePlace() : model.kerbLaneBeside(model.frontageAt(from));
        const engine::KerbLanePlace b = model.kerbLaneBeside(model.frontageAt(to));
        if ((!fromLane && !a.valid()) || !b.valid()) return false;
        engine::PathPoint pa, pb;
        if (fromLane) {
            pa = *fromLane;
        } else {
            pa.path = a.path;
            pa.along = a.along;
        }
        pb.path = b.path;
        pb.along = b.along;
        const engine::roads::Route r = net.driving->routeBetween(vehicleClass, pa, pb);
        if (!r.found) return false;
        leg.metres = r.metres;
        leg.minutes = r.seconds / 60.0;
        leg.kindMetres.assign(cat.roadKinds.size(), 0);
        leg.kindStretches.assign(cat.roadKinds.size(), 0);
        int was = -1;
        const auto count = [&](engine::PathId path, Real metres) {
            const engine::ModelStreet* st = model.streetOf(path);
            const int k = cat.roadKindOf(st ? st->section.klass : engine::RoadClass::Local);
            if (k >= 0) {
                leg.kindMetres[static_cast<std::size_t>(k)] += metres;
                if (k != was) ++leg.kindStretches[static_cast<std::size_t>(k)];
            }
            was = k;
        };
        for (std::size_t i = 0; i < r.legs.size(); ++i) {
            count(r.legs[i].lane, r.legs[i].metres);
            if (i < r.turns.size() && r.turns[i].connector.valid()) count(r.turns[i].connector, r.turns[i].metres);
        }
    } else {
        // ON FOOT: over the walks, from where each end joins them
        const engine::WalkJoin a = model.walkJoinNear(from, net.walker->open);
        const engine::WalkJoin b = model.walkJoinNear(to, net.walker->open);
        if (!a.valid() || !b.valid()) return false;
        engine::roads::WalkRoute r;
        if (a.at != b.at) {
            r = engine::roads::findWalkRoute(model, *net.walker, a.at, b.at);
            if (!r.found) return false;
        }
        leg.metres = (from - a.point).length() + (r.found ? r.metres : 0) + (b.point - to).length();
        leg.minutes = leg.metres / m.speed / 60.0;
        leg.route = std::move(r);
    }
    leg.minutes += m.startMinutes + m.endMinutes;
    out = std::move(leg);
    return true;
}

TripPlan planTrip(const TripNetwork& nav, const TravelCatalog& cat, const TripRequest& req) {
    TripPlan plan;
    plan.from = req.from;
    plan.to = req.to;
    const int maxLegs = std::min(req.maxLegs, kMaxPlanLegs);
    // Whether the person may use a row this time: not ruled out, and they have what it requires.
    auto ruledOut = [&](int mode) {
        return std::find(req.ruledOut.begin(), req.ruledOut.end(), mode) != req.ruledOut.end();
    };
    auto has = [&](const TravelMode& m) { return m.needsTrait < 0 || ((req.traits >> m.needsTrait) & 1u) != 0; };
    // A leg longer than its way suits is not made that way.
    auto tooLong = [&](const TravelMode& m, const TripLeg& l) {
        return m.longestLegMinutes > 0 && engine::strictlyPast(l.minutes, m.longestLegMinutes, kSameMinutes);
    };
    // A VEHICLE IN HAND: the person is in it on the road, so the trip is made in it and no other way is priced.
    const OwnedVehicle* inHand = nullptr;
    for (const OwnedVehicle& ov : req.vehicles)
        if (ov.inHand && !inHand) inHand = &ov;
    for (std::size_t i = 0; i < cat.modes.size(); ++i) {
        const TravelMode& m = cat.modes[i];
        const int mode = static_cast<int>(i);
        TripOption o;
        o.mode = mode;
        auto leg = [&](int legMode, Vec2 a, Vec2 b, LegEnd end, const std::string& whyNot, uint32_t vehicleClass = 0,
                       const engine::PathPoint* fromLane = nullptr) {
            TripLeg l;
            if (!planLeg(nav, cat, legMode, a, b, l, vehicleClass, fromLane)) { o.why = whyNot; return false; }
            const TravelMode& lm = cat.modes[static_cast<std::size_t>(legMode)];
            if (tooLong(lm, l)) {
                o.why = "too far for " + lm.name + " (" + minutesText(l.minutes) + " min, at most " +
                        minutesText(lm.longestLegMinutes) + ")";
                return false;
            }
            l.end = end;
            o.legs.push_back(std::move(l));
            return true;
        };
        // TO AND FROM A VEHICLE: by the way the table fixes for it, or else by the cheapest way on foot the person has
        // (weighted minutes; ties to the earlier row), so someone with a board skates to the car when that is quicker.
        auto footLeg = [&](Vec2 a, Vec2 b, LegEnd end, const std::string& whyNot) {
            if (m.access >= 0) return leg(m.access, a, b, end, whyNot + " by " + cat.modes[static_cast<std::size_t>(m.access)].name);
            TripLeg best;
            Real bestWeighted = 0;
            for (std::size_t f = 0; f < cat.modes.size(); ++f) {
                const TravelMode& fm = cat.modes[f];
                if (fm.roads || fm.ownVehicle || !has(fm) || ruledOut(static_cast<int>(f))) continue;
                TripLeg l;
                if (!planLeg(nav, cat, static_cast<int>(f), a, b, l) || tooLong(fm, l)) continue;
                const Real weighted = l.minutes * fm.weight;
                if (best.mode < 0 || weighted < bestWeighted) {
                    best = std::move(l);
                    bestWeighted = weighted;
                }
            }
            if (best.mode < 0) { o.why = whyNot + " on foot"; return false; }
            best.end = end;
            o.legs.push_back(std::move(best));
            return true;
        };
        if (!has(m)) {
            o.offered = false;
            o.why = "needs " + m.needs;
        } else if (ruledOut(mode)) {
            o.why = "ruled out (a leg of it could not be done)";
        } else if (inHand && inHand->mode != mode) {
            o.why = "at the wheel of the " + cat.modes[static_cast<std::size_t>(inHand->mode)].name;
        } else if (inHand) {
            o.possible = leg(mode, inHand->at, inHand->leaveAt, LegEnd::VehicleLeft, "no route by " + m.name, inHand->vehicleClass,
                             inHand->onLane ? &inHand->lane : nullptr) &&
                         footLeg(inHand->leaveAt, req.to, LegEnd::Door, "no route from where the " + m.name + " is left");
        } else if (!m.ownVehicle) {
            o.possible = leg(mode, req.from, req.to, LegEnd::Door,
                             "no route on the " + std::string(m.roads ? "roads" : "footpaths"));
        } else {
            const OwnedVehicle* v = nullptr;
            for (const OwnedVehicle& ov : req.vehicles)
                if (ov.mode == mode) { v = &ov; break; }
            if (!v) {
                o.why = "no " + m.name;
            } else if (!v->free) {
                o.why = "the " + m.name + " is in use";
            } else {
                o.possible = footLeg(req.from, v->at, LegEnd::Vehicle, "no route to the " + m.name) &&
                             leg(mode, v->at, v->leaveAt, LegEnd::VehicleLeft, "no route by " + m.name, v->vehicleClass) &&
                             footLeg(v->leaveAt, req.to, LegEnd::Door, "no route from where the " + m.name + " is left");
            }
        }
        if (o.possible && static_cast<int>(o.legs.size()) > maxLegs) {
            o.possible = false;
            o.why = std::to_string(o.legs.size()) + " legs, and a plan holds at most " + std::to_string(maxLegs);
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
    for (const TripOption& o : plan.options)
        if (o.offered) plan.reason += (plan.reason.empty() ? "" : ", ") + describeOption(cat, o);
    plan.reason += ": " + (plan.chosen >= 0 ? cat.modes[static_cast<std::size_t>(plan.choice()->mode)].name
                                            : std::string("no way there"));
    return plan;
}

bool fixedLegs(const TravelCatalog& cat, const TripOption& o, FixedLegs& out, std::string* err) {
    auto refuse = [&](const std::string& why) {
        const std::string name = o.mode >= 0 && o.mode < static_cast<int>(cat.modes.size())
                                     ? cat.modes[static_cast<std::size_t>(o.mode)].name : std::string("?");
        if (err) *err = "travel.lua: modes `" + name + "`: " + why;
        return false;
    };
    if (!o.possible) return refuse("this way is not possible (" + o.why + ")");
    if (o.legs.size() > static_cast<std::size_t>(kMaxPlanLegs))
        return refuse("a plan of " + std::to_string(o.legs.size()) + " legs; a plan holds at most " +
                      std::to_string(kMaxPlanLegs) + " (kMaxPlanLegs)");
    FixedLegs f;
    for (const TripLeg& l : o.legs) {
        const long seconds = std::lround(std::max(Real(0), l.minutes) * 60.0);
        if (seconds > kMaxLegSeconds)
            return refuse("leg " + std::to_string(f.count + 1) + " takes " + std::to_string(seconds) +
                          " s; a stored leg holds at most " + std::to_string(kMaxLegSeconds));
        if (l.mode < 0 || l.mode > 255) return refuse("leg " + std::to_string(f.count + 1) + " names no row");
        FixedLeg& d = f.legs[f.count++];
        d.way = static_cast<uint8_t>(l.mode);
        d.end = l.end;
        d.seconds = static_cast<uint16_t>(seconds);
    }
    out = f;
    return true;
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
