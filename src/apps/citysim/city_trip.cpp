#include "city_trip.h"

#include "city_render.h"
#include "city_sim.h"
#include "city_spectate.h"
#include "../../engine/script_assets.h"
#include "../../engine/waypoint.h"
#include "../../hash.h"
#include "../../log.h"
#include "../../renderer/event.h"   // KeyCode
#ifdef RT_ENABLE_SCRIPTING
#include "../../engine/scripting/script_vm.h"
#include "scripting/travel_lua.h"
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace citysim {

using engine::Vec3;

namespace {
constexpr Real kPlanLift = 0.4;                          // the plan's line, above the surface it follows (m)
constexpr Real kLineStep = 2.0;                          // ...a point every this many metres along the way (m)
constexpr Real kMarkHeight = 14.0;                       // the destination's post (m)
const Vec3 kMarkColour(1.00, 0.78, 0.16);                // the waypoint's gold (the map's and the watch's)
const char* kOwner = "CityTripSystem";

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// The nearest node a car can leave from (a place's nearest node can be a park path's)
int roadNode(const engine::NavGraph& nav, engine::Vec2 p) {
    for (int n : nav.nearestNodes(p, 8))
        for (int li : nav.outLinks[static_cast<std::size_t>(n)])
            if (!nav.links[static_cast<std::size_t>(li)].footpath) return n;
    return -1;
}

std::string fmt(const char* f, double a) {
    char b[64];
    std::snprintf(b, sizeof(b), f, a);
    return b;
}
}  // namespace

void CityTripSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("trip_near", engine::KeyCode::Y);
    ctx.actions.bindButton("trip_far", engine::KeyCode::I);
    ctx.actions.bindButton("trip_freeway", engine::KeyCode::U);
    if (ctx.commands) {
        ctx.commands->add("trip", "trip near|far|freeway|to <place name or id>|at <x> <z>|mark",
                          "Send the city's person somewhere -- near or far (they pick the building), the building whose "
                          "drive uses the freeway furthest, a place, a point, or the map's waypoint -- to plan the trip "
                          "and make it; poll trip?.",
                          [this](const engine::ControlCommand& c) { return stage(c); }, kOwner);
        ctx.commands->add("trip?", "trip?",
                          "The person's trip: where to, every option with its legs and minutes, the choice and why, "
                          "the leg under way, metres left, and arrived.",
                          [this](const engine::ControlCommand&) { return "ok " + status(); }, kOwner);
    }
#ifdef RT_ENABLE_SCRIPTING
    const std::string code = engine::loadScriptCode("travel.lua", "");
    if (code.empty()) {
        travelError_ = "travel.lua not found";
    } else {
        engine::ScriptVM vm;
        std::string err;
        haveTravel_ = vm.doString(code, &err) && engine::loadTravelCatalog(vm, travel_, &err);
        travelError_ = haveTravel_ ? "" : err;
    }
#else
    travelError_ = "the ways of travelling are travel.lua: this build has no scripting";
#endif
    if (!haveTravel_) LOG_WARN << "[trip] " << travelError_ << " -- trips are off";
}

void CityTripSystem::onStop(engine::FrameContext& ctx) {
    if (ctx.commands) ctx.commands->removeOwnedBy(kOwner);
    ui_.release(ctx.renderer);
}

void CityTripSystem::fixedUpdate(engine::FrameContext& ctx) {
    simTime_ += ctx.clock.fixedStep();
    // the straight steps of a walk (step() sees them through)
    if (active_ && straight_ && !arrived_ && plan_.valid() && who_ >= 0 && city_.built()) {
        const int walk = plan_.choice()->legs[static_cast<std::size_t>(leg_)].mode;
        if (steps_.empty())
            steps_ = footSteps(city_.sim().agents()[static_cast<std::size_t>(who_)].pos, walkTarget(static_cast<std::size_t>(leg_)));
        if (city_.simMutable().stepAgentToward(who_, steps_.front(), travel_.modes[static_cast<std::size_t>(walk)].speed,
                                               ctx.clock.fixedStep())) {
            steps_.erase(steps_.begin());
            straightDone_ = steps_.empty();
        }
    }
}

std::string CityTripSystem::stage(const engine::ControlCommand& cmd) {
    const std::string usage = "err usage: trip near|far|freeway|to <place name or id>|at <x> <z>|mark";
    if (cmd.args.empty()) return usage;
    const std::string what = lower(cmd.args[0]);
    if (what == "near" || what == "far" || what == "freeway" || what == "mark") {
        requests_.push_back(what);
    } else if (what == "at") {
        if (cmd.args.size() < 3) return usage;
        char* end = nullptr;
        std::strtod(cmd.args[1].c_str(), &end);
        if (*end) return usage;
        std::strtod(cmd.args[2].c_str(), &end);
        if (*end) return usage;
        requests_.push_back("at " + cmd.args[1] + " " + cmd.args[2]);
    } else if (what == "to") {
        const std::size_t at = cmd.rest.find_first_not_of(" \t", cmd.rest.find_first_of(" \t", cmd.rest.find("to")));
        if (cmd.args.size() < 2 || at == std::string::npos) return usage;
        requests_.push_back("to " + cmd.rest.substr(at));
    } else {
        return usage;
    }
    return "ok staged (poll trip?)";
}

int CityTripSystem::person() const {
    const std::vector<Agent>& agents = city_.sim().agents();
    int withHome = -1;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const Agent& a = agents[i];
        if (a.homePlace == kNoPlace || a.released || a.playerControlled) continue;
        if (a.car >= 0) return static_cast<int>(i);
        if (withHome < 0) withHome = static_cast<int>(i);
    }
    return withHome;
}

std::string CityTripSystem::placeLabel(PlaceId id) const {
    const Place& p = city_.places()[id];
    std::string s = placeTypeName(p.type);
    if (!p.name.empty()) s += " \"" + p.name + "\"";
    return s + " (#" + std::to_string(id) + ")";
}

// THE PERSON PICKS: a building (Glenn: "We should have buildings for the agent to go to") -- any place but a park --
// whose door is a near (or far) distance from where they stand, by their own dice and
// the count of trips they have made -- the same pick on every run. Somewhere to go and do something: not a home, while
// any other kind of place lies in the band; homes (never their own) only when nothing else does.
bool CityTripSystem::pickPlace(bool far, PlaceId& out, std::string& why) {
    const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
    const Real lo = far ? travel_.farMin : travel_.nearMin, hi = far ? travel_.farMax : travel_.nearMax;
    std::vector<PlaceId> places, homes;
    for (const Place& p : city_.places().places()) {
        if (p.id == a.homePlace || !placeIsIndoors(p.type)) continue;   // a building (a park is not one)
        const Real d = (p.entrance - a.pos).length();
        if (d < lo || d > hi) continue;
        (p.type == PlaceType::Home ? homes : places).push_back(p.id);
    }
    if (places.empty() && homes.empty()) {
        why = "no building " + fmt("%.0f", lo) + "-" + fmt("%.0f m", hi) + " from where the person stands";
        return false;
    }
    // The person picks one THEY CAN GET TO: starting from their own pick, the first building the planner finds a way
    // to (shops and the like before other people's homes). A building across water with no bridge in walking or
    // driving reach is passed over, not picked and then given up on.
    const uint32_t h = engine::hashPair(a.brain, static_cast<uint32_t>(trips_ * 2 + (far ? 1 : 0)));
    const std::vector<int> none;
    for (const std::vector<PlaceId>* from : {&places, &homes}) {
        for (std::size_t k = 0; k < from->size(); ++k) {
            const PlaceId id = (*from)[(h + k) % from->size()];
            if (!planTrip(*city_.sim().graph(), travel_, requestTo(city_.places()[id].entrance, none)).valid()) continue;
            out = id;
            return true;
        }
    }
    why = "no building " + fmt("%.0f", lo) + "-" + fmt("%.0f m", hi) + " away that the person can get to (" +
          std::to_string(places.size() + homes.size()) + " in range, none with a way there)";
    return false;
}

// THE FREEWAY TRIP (ADR-0416; Glenn: "I have still yet to see the agent get on the freeway ... could we start the
// agent near the freeway and then have them get into the car to get to the restaurant?"): of the buildings the person
// can drive to from where they stand, the one whose planned drive -- the router's fastest, as any trip's -- runs
// furthest on the freeway, a restaurant before any other kind. The router is not bent: a drive that does not pay to
// take the freeway does not take it, and when no building's does, the person says so. For the pick, the plan's
// minutes are set against the best drive that keeps to the streets (the same router, the freeway and its ramps
// priced out), so the choice can be checked.
bool CityTripSystem::pickFreeway(PlaceId& out, std::string& why) {
    const CitySim& sim = city_.sim();
    const engine::NavGraph& nav = *sim.graph();
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    const std::vector<int> none;
    int drivable = 0, onFreeway = 0;
    Real best = 0;
    bool bestIsRestaurant = false;
    PlaceId pick = kNoPlace;
    TripLeg pickDrive;
    for (const Place& p : city_.places().places()) {
        if (p.id == a.homePlace || !placeIsIndoors(p.type)) continue;   // a building, not their own home
        const TripRequest req = requestTo(p.entrance, none);
        if (req.vehicles.empty()) { why = "the person has no car"; return false; }
        const OwnedVehicle& v = req.vehicles.front();
        TripLeg drive;
        if (!planLeg(nav, travel_, v.mode, v.at, v.leaveAt, drive)) continue;
        ++drivable;
        Real fw = 0;
        for (int li : drive.route.links)
            if (nav.links[static_cast<std::size_t>(li)].klass == engine::RoadClass::Freeway)
                fw += nav.links[static_cast<std::size_t>(li)].length;
        if (fw <= 0) continue;
        ++onFreeway;
        const bool restaurant = p.type == PlaceType::Restaurant;
        if ((restaurant && !bestIsRestaurant) || (restaurant == bestIsRestaurant && fw > best)) {
            best = fw;
            bestIsRestaurant = restaurant;
            pick = p.id;
            pickDrive = drive;
        }
    }
    freewayNote_ = std::to_string(onFreeway) + " of the " + std::to_string(drivable) +
                   " buildings the person can drive to from " + fmt("%.0f", a.pos.x) + " " + fmt("%.0f", a.pos.y) +
                   " have a fastest drive that uses the freeway";
    if (pick == kNoPlace) {
        // HOW CLOSE IT CAME: the router's time to each building on the streets alone and through the freeway (one
        // doubled search from the car's node), and the building where the freeway loses least
        std::vector<char> freeway(static_cast<std::size_t>(nav.linkCount()), 0);
        for (int li = 0; li < nav.linkCount(); ++li)
            freeway[static_cast<std::size_t>(li)] = nav.links[static_cast<std::size_t>(li)].klass == engine::RoadClass::Freeway;
        std::vector<Real> streets, via;
        engine::travelSecondsVia(nav, roadNode(nav, sim.vehicles()[static_cast<std::size_t>(a.car)].pos), freeway, streets, via);
        Real least = std::numeric_limits<Real>::infinity();
        PlaceId closest = kNoPlace;
        for (const Place& p : city_.places().places()) {
            if (p.id == a.homePlace || !placeIsIndoors(p.type)) continue;
            const int to = roadNode(nav, p.entrance);
            if (to < 0 || !std::isfinite(via[static_cast<std::size_t>(to)])) continue;
            const Real slower = via[static_cast<std::size_t>(to)] - streets[static_cast<std::size_t>(to)];
            if (slower < least) { least = slower; closest = p.id; }
        }
        why = "the fastest drive to every building stays on streets (" + freewayNote_ + ")" +
              (closest == kNoPlace ? std::string("; the freeway cannot be reached from here")
                                   : "; by the freeway the closest call is " + placeLabel(closest) + ", " +
                                         fmt("%.0f s", least) + " slower than by the streets");
        return false;
    }
    // ...against the best drive on the streets alone: the router's own, with the freeway and its ramps priced out
    std::vector<Real> streetsOnly(static_cast<std::size_t>(nav.linkCount()), 1.0);
    for (int li = 0; li < nav.linkCount(); ++li) {
        const engine::RoadClass k = nav.links[static_cast<std::size_t>(li)].klass;
        if (k == engine::RoadClass::Freeway || k == engine::RoadClass::Ramp) streetsOnly[static_cast<std::size_t>(li)] = 1e6;
    }
    const TravelMode& car = travel_.modes[static_cast<std::size_t>(pickDrive.mode)];
    const engine::Route streets = engine::findRoute(nav, pickDrive.fromNode, pickDrive.toNode, false, &streetsOnly);
    std::string versus = "no drive there keeps to the streets";
    if (streets.valid() && std::none_of(streets.links.begin(), streets.links.end(), [&](int li) {
            return streetsOnly[static_cast<std::size_t>(li)] > 1.0;
        })) {
        const Real off = pickDrive.metres - pickDrive.route.length(nav);
        const Real minutes = (engine::routeSeconds(nav, streets) + off / engine::classSpeed(engine::RoadClass::Local)) / 60.0 +
                             car.startMinutes + car.endMinutes;
        versus = "the best drive on streets alone is " + metresText(streets.length(nav) + off) + ", " +
                 fmt("%.1f min", minutes);
    }
    freewayNote_ += "; this one's drive is " + fmt("%.1f min", pickDrive.minutes) + " (" + describeRoads(travel_, pickDrive) +
                    "), " + versus;
    out = pick;
    return true;
}

void CityTripSystem::take(engine::FrameContext& ctx, const std::string& request) {
    if (!city_.built() || !city_.sim().graph()) { note_ = "the city is not built yet"; return; }
    if (!haveTravel_) { note_ = travelError_; return; }
    const int p = person();
    if (p < 0) { note_ = "nobody in this city has a home to set out from"; return; }
    const Agent& a = city_.sim().agents()[static_cast<std::size_t>(p)];
    // Changing their mind is fine on foot; at the wheel the car would be left in the lane (version 1: they finish)
    if (a.vehicle >= 0) { note_ = "refused \"trip " + request + "\": the person is driving; ask again once they are out"; return; }
    who_ = p;
    const std::string what = request.substr(0, request.find(' '));
    if (what != "freeway") freewayNote_.clear();
    const std::string arg = request.size() > what.size() ? request.substr(what.size() + 1) : "";
    if (what == "near" || what == "far") {
        PlaceId id = kNoPlace;
        std::string why;
        if (!pickPlace(what == "far", id, why)) { note_ = "trip " + what + ": " + why; return; }
        begin(ctx, city_.places()[id].entrance, id, placeLabel(id) + ", picked " + what);
    } else if (what == "freeway") {
        PlaceId id = kNoPlace;
        std::string why;
        const bool found = pickFreeway(id, why);
        if (!found) { note_ = "trip freeway: " + why; return; }
        begin(ctx, city_.places()[id].entrance, id, placeLabel(id) + ", picked for the freeway");
    } else if (what == "to") {
        const PlaceMap& places = city_.places();
        PlaceId id = kNoPlace;
        if (!arg.empty() && std::all_of(arg.begin(), arg.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; })) {
            const long n = std::strtol(arg.c_str(), nullptr, 10);
            if (n >= 0 && n < places.size()) id = static_cast<PlaceId>(n);
        }
        const std::string want = lower(arg);
        for (std::size_t i = 0; id == kNoPlace && i < places.places().size(); ++i)
            if (!places.places()[i].name.empty() && lower(places.places()[i].name).find(want) != std::string::npos)
                id = static_cast<PlaceId>(i);
        PlaceType t;
        if (id == kNoPlace && parsePlaceType(want, t)) id = places.nearest(t, a.pos);   // "cafe": the nearest one
        if (id == kNoPlace) { note_ = "trip to " + arg + ": no place by that id, name or kind"; return; }
        begin(ctx, places[id].entrance, id, placeLabel(id));
    } else if (what == "at") {
        const double x = std::strtod(arg.c_str(), nullptr);
        const double z = std::strtod(arg.substr(arg.find(' ') + 1).c_str(), nullptr);
        begin(ctx, Vec2(x, z), kNoPlace, "the point " + fmt("%.1f", x) + " " + fmt("%.1f", z));
    } else if (what == "mark") {
        const engine::Waypoint* mark = nullptr;
        ctx.world.each<engine::Waypoint>([&](engine::Entity, engine::Waypoint& w) { if (!mark) mark = &w; });
        if (!mark) { note_ = "trip mark: no waypoint set (mark one on the map, tool 3)"; return; }
        begin(ctx, Vec2(mark->at.x, mark->at.z), kNoPlace,
              "the map's waypoint " + fmt("%.1f", mark->at.x) + " " + fmt("%.1f", mark->at.z));
    }
}

void CityTripSystem::begin(engine::FrameContext& ctx, Vec2 dest, PlaceId place, const std::string& label) {
    (void)ctx;
    CitySim& sim = city_.simMutable();
    // The trip is the director's plan (ADR-0091): the person's own day, if they keep one, is held off until it is done
    sim.setAgentDirected(who_, true);
    sim.pinAgent(who_, true);   // simulated in full wherever it goes
    active_ = true;
    dest_ = dest;
    place_ = place;
    where_ = label;
    ruledOut_.clear();
    note_.clear();
    startedAt_ = simTime_;
    ++trips_;
    plan("");
    if (plan_.valid()) plannedMinutes_ = plan_.choice()->minutes;
    spectate_.follow(who_);
}

// WHAT THE PLANNER IS ASKED for a trip to `dest` from where the person stands, with what they own.
TripRequest CityTripSystem::requestTo(Vec2 dest_, const std::vector<int>& ruledOut) const {
    const CitySim& sim = city_.sim();
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    TripRequest req;
    req.from = a.pos;
    req.to = dest_;
    req.ruledOut = ruledOut;
    // THE CAR THEY OWN (Agent::car) is driven on the roads, so it serves every way that needs a vehicle of one's own
    // and uses the roads; it would be left in the free space nearest the destination (the sim reserves the one nearest
    // the destination's street when the drive starts: this is the planner's estimate of it).
    if (a.car >= 0 && a.car < static_cast<int>(sim.vehicles().size())) {
        const SimVehicle& v = sim.vehicles()[static_cast<std::size_t>(a.car)];
        Vec2 leaveAt = dest_;
        Real best = 1e30;
        for (const CitySim::ParkingBay& b : sim.parkingBays()) {
            if (b.busOnly || (b.occupant >= 0 && b.occupant != who_)) continue;
            const Real d = (b.pos - dest_).lengthSquared();
            if (d < best) { best = d; leaveAt = b.pos; }
        }
        for (std::size_t i = 0; i < travel_.modes.size(); ++i)
            if (travel_.modes[i].ownVehicle && travel_.modes[i].roads)
                req.vehicles.push_back(OwnedVehicle{static_cast<int>(i), v.pos, leaveAt});
    }
    return req;
}

bool CityTripSystem::plan(const std::string& note) {
    const CitySim& sim = city_.sim();
    plan_ = planTrip(*sim.graph(), travel_, requestTo(dest_, ruledOut_));
    lines_.clear();
    legActual_.clear();
    leg_ = 0;
    legUnderWay_ = false;
    straight_ = straightDone_ = false;
    steps_.clear();
    arrived_ = false;
    if (!note.empty()) note_ = note;
    LOG_INFO << "[trip] person " << who_ << " to " << where_ << " at " << dest_.x << "," << dest_.y << ": "
             << plan_.reason << (note.empty() ? "" : " | ") << note;
    return plan_.valid();
}

// A LEG BEGINS: into the vehicle first when the leg is by one, then sent (sendAgentTo moves them by what they are now:
// on foot, or at the wheel). A leg that cannot begin rules its way out for this trip and re-plans from here.
bool CityTripSystem::startLeg() {
    CitySim& sim = city_.simMutable();
    const TripOption& o = *plan_.choice();
    const TripLeg& L = o.legs[static_cast<std::size_t>(leg_)];
    const TravelMode& m = travel_.modes[static_cast<std::size_t>(L.mode)];
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    std::string why;
    if (m.ownVehicle && a.vehicle < 0) {
        const SimVehicle* v = a.car >= 0 && a.car < static_cast<int>(sim.vehicles().size())
                                  ? &sim.vehicles()[static_cast<std::size_t>(a.car)] : nullptr;
        if (!v) why = "the " + m.name + " is gone";
        else if ((v->pos - a.pos).length() > kBoardReach)
            why = "the " + m.name + " is " + fmt("%.0f m", (v->pos - a.pos).length()) + " away, out of reach";
        else if (!sim.boardVehicle(who_, a.car)) why = "could not get into the " + m.name;
        else LOG_INFO << "[trip] person " << who_ << " got into the " << m.name;
    }
    const Vec2 target = walkTarget(static_cast<std::size_t>(leg_));
    if (why.empty() && !m.roads && (target - a.pos).length() <= kBoardReach) {
        // A FEW STEPS: walked straight (stepAgentToward), not routed round the block
        straight_ = true;
        legUnderWay_ = true;
    } else if (why.empty()) {
        if (sim.sendAgentTo(who_, m.roads ? L.to : target)) {
            legUnderWay_ = true;
        } else if ((L.to - a.pos).length() <= kBoardReach) {
            // Within reach and nothing routes closer (a park's door on a walk the pavement does not lead into): they
            // stay where they are, and trip? says how far from the door that is
            LOG_INFO << "[trip] leg " << (leg_ + 1) << " by " << m.name << ": no route from here, "
                     << fmt("%.1f m", (L.to - a.pos).length()) << " to go -- within reach, taken as there";
            legUnderWay_ = true;   // (finished next frame, standing still)
        } else {
            why = "no route from here by " + m.name;
        }
    }
    if (why.empty() && m.ownVehicle && a.targetBay >= 0 && a.targetBay < static_cast<int>(sim.parkingBays().size())) {
        // THE BAY IS KNOWN NOW (the sim reserved it as the drive began): the drive ends there, and the walk after it
        // starts there -- routed again from it, its planned minutes kept (the planner's bay was an estimate)
        TripOption& oc = plan_.options[static_cast<std::size_t>(plan_.chosen)];
        const Vec2 bay = sim.parkingBays()[static_cast<std::size_t>(a.targetBay)].pos;
        oc.legs[static_cast<std::size_t>(leg_)].to = bay;
        if (static_cast<std::size_t>(leg_) + 1 < oc.legs.size()) {
            TripLeg& next = oc.legs[static_cast<std::size_t>(leg_) + 1];
            TripLeg again;
            if (planLeg(*sim.graph(), travel_, next.mode, bay, next.to, again)) {
                again.minutes = next.minutes;
                next = std::move(again);
            }
        }
        lines_.clear();
    }
    if (why.empty()) {
        legStartedAt_ = simTime_;
        LOG_INFO << "[trip] leg " << (leg_ + 1) << "/" << o.legs.size() << " by " << m.name << " to " << L.to.x << ","
                 << L.to.y << " (" << fmt("%.0f m", L.metres) << ", " << fmt("%.1f min", L.minutes) << ")";
        return true;
    }
    if (a.vehicle >= 0) sim.alightVehicle(who_);   // a drive that could not start: out again, where it stands
    ruledOut_.push_back(o.mode);
    const std::string was = travel_.modes[static_cast<std::size_t>(o.mode)].name;
    if (!plan("leg " + std::to_string(leg_ + 1) + " (" + m.name + ") could not start: " + why + "; re-planned without " +
              was + " from where they stand"))
        note_ += "; no other way there";
    return false;
}

void CityTripSystem::finishLeg() {
    CitySim& sim = city_.simMutable();
    const TripOption& o = *plan_.choice();
    const TripLeg& L = o.legs[static_cast<std::size_t>(leg_)];
    const TravelMode& m = travel_.modes[static_cast<std::size_t>(L.mode)];
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    if (m.ownVehicle) {
        std::string bay = "none reserved";
        if (a.targetBay >= 0 && a.targetBay < static_cast<int>(sim.parkingBays().size())) {
            const CitySim::ParkingBay& b = sim.parkingBays()[static_cast<std::size_t>(a.targetBay)];
            bay = std::string(b.garage >= 0 ? "in a garage, " : "at the kerb, ") + "reserved " +
                  fmt("%.1f m", (b.pos - a.pos).length()) + " away";
        }
        LOG_INFO << "[trip] stopped at " << a.pos.x << "," << a.pos.y << ", parking bay " << bay;
        sim.alightVehicle(who_);   // out, the car left in the space it was driven to
    }
    LOG_INFO << "[trip] leg " << (leg_ + 1) << "/" << o.legs.size() << " by " << m.name << " done in "
             << fmt("%.1f min", (simTime_ - legStartedAt_) / 60.0) << " (planned " << fmt("%.1f", L.minutes)
             << "), " << fmt("%.1f m", (a.pos - L.to).length()) << " from its end"
             << (!m.ownVehicle ? std::string() : a.parkedBay >= 0 ? ", the " + m.name + " left in a parking bay"
                                                       : ", the " + m.name + " left where it stopped");
    legActual_.push_back(static_cast<Real>((simTime_ - legStartedAt_) / 60.0));
    ++leg_;
    legUnderWay_ = false;
    straight_ = straightDone_ = false;
    steps_.clear();
    if (leg_ >= static_cast<int>(o.legs.size())) {
        arrived_ = true;
        arrivedAt_ = simTime_;
        LOG_INFO << "[trip] person " << who_ << " arrived at " << where_ << " after "
                 << fmt("%.1f min", (arrivedAt_ - startedAt_) / 60.0) << " (planned " << fmt("%.1f", plannedMinutes_)
                 << "), " << fmt("%.1f m", (a.pos - dest_).length()) << " from the door";
    }
}

void CityTripSystem::step() {
    if (!active_ || arrived_ || !plan_.valid() || who_ < 0) return;
    if (!legUnderWay_) { startLeg(); return; }
    const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
    if (straight_) {
        if (straightDone_) finishLeg();
        return;
    }
    if (a.moving) return;
    // ON FOOT, the routed walk over: the last few steps off the pavement (to the car, to a door) are walked straight
    const TravelMode& m = travel_.modes[static_cast<std::size_t>(plan_.choice()->legs[static_cast<std::size_t>(leg_)].mode)];
    const Vec2 target = walkTarget(static_cast<std::size_t>(leg_));
    const Real off = (target - a.pos).length();
    if (!m.roads && a.mode == Agent::Mode::Pedestrian && off > 0.3 && off <= kBoardReach) { straight_ = true; return; }
    finishLeg();
}

// WHERE A WALK LEG ENDS: at the car's door when the leg after it is by the car (beside it, on the kerb side, level
// with the front seat), else at the leg's end (a door).
Vec2 CityTripSystem::walkTarget(std::size_t k) const {
    const TripOption& o = *plan_.choice();
    const CitySim& sim = city_.sim();
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    if (k + 1 < o.legs.size() && travel_.modes[static_cast<std::size_t>(o.legs[k + 1].mode)].ownVehicle &&
        a.car >= 0 && a.car < static_cast<int>(sim.vehicles().size())) {
        const SimVehicle& v = sim.vehicles()[static_cast<std::size_t>(a.car)];
        const Real hl = v.heading.length();
        const Vec2 f = hl > 1e-6 ? v.heading * (1.0 / hl) : Vec2(0, 1);
        return v.pos + engine::rightOf(f) * (v.width * 0.5 + 0.5) + f * (v.length * 0.1);
    }
    return o.legs[k].to;
}

// A FEW STEPS OFF THE PATHS, as a person takes them: out to the pavement, along it, and in to where they are going --
// not straight along a front wall or across a corner. The pavement is the walkers' own (the verge they keep), on the
// side of the street the point is on.
std::vector<Vec2> CityTripSystem::footSteps(Vec2 from, Vec2 to) const {
    const CitySim& sim = city_.sim();
    const engine::NavGraph& nav = *sim.graph();
    const Real verge = sim.walkerVerge(sim.agents()[static_cast<std::size_t>(who_)]);
    auto pavement = [&](Vec2 p) {
        const int l = nav.nearestLink(p);
        if (l < 0) return p;
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(l)];
        const Vec2 A = nav.nodes[static_cast<std::size_t>(L.from)];
        const Real t = L.length > 1e-6 ? std::clamp(dot(p - A, nav.direction(l)) / L.length, Real(0), Real(1)) : Real(0);
        const Vec2 q = nav.pointOnLink(l, t);
        const Vec2 side = nav.sidewalkPoint(l, t, verge) - q;
        return dot(p - q, side) >= 0 ? q + side : q - side;   // (the pavement on p's side of the street)
    };
    std::vector<Vec2> steps;
    for (const Vec2 p : {pavement(from), pavement(to), to})
        if ((p - (steps.empty() ? from : steps.back())).length() > 0.5) steps.push_back(p);
    if (steps.empty() || (steps.back() - to).length() > 1e-6) steps.push_back(to);
    return steps;
}

void CityTripSystem::update(engine::FrameContext& ctx) {
    // The keys only where the city keeps no day (citysim.schedule = false: the trip lab); the verbs work anywhere
    const bool lab = city_.built() && !city_.sim().daySchedule();
    if (lab && ctx.actions.pressed("trip_near")) requests_.push_back("near");
    if (lab && ctx.actions.pressed("trip_far")) requests_.push_back("far");
    if (lab && ctx.actions.pressed("trip_freeway")) requests_.push_back("freeway");
    std::vector<std::string> requests;
    requests.swap(requests_);
    for (const std::string& r : requests) take(ctx, r);
    if (!city_.built()) return;
    step();
    draw(ctx);
}

Real CityTripSystem::metresLeft() const {
    if (!active_ || !plan_.valid() || arrived_) return 0;
    const TripOption& o = *plan_.choice();
    Real left = 0;
    for (std::size_t k = static_cast<std::size_t>(leg_) + (legUnderWay_ ? 1 : 0); k < o.legs.size(); ++k)
        left += o.legs[k].metres;
    if (legUnderWay_) {
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
        const engine::NavGraph& nav = *city_.sim().graph();
        if (a.moving && a.route.valid()) {
            const int n = static_cast<int>(a.route.links.size());
            for (int k = a.leg; k < n; ++k) {
                const Real len = nav.links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(k)])].length;
                left += (k == n - 1 && a.stopAtDist >= 0) ? a.stopAtDist : len;
            }
            left -= a.distOnLeg;
        }
        if (straight_) {
            Vec2 at = a.pos;
            for (const Vec2 p : steps_.empty() ? std::vector<Vec2>{walkTarget(static_cast<std::size_t>(leg_))} : steps_) {
                left += (p - at).length();
                at = p;
            }
        }
    }
    return std::max(Real(0), left);
}

// THE PLAN, DRAWN (ADR-0415 as built): each leg still to come as the line its traveller will be posed along --
// CitySim::walkPoses, the stepper's own rule for the pavement, the lane, the corners and the junction turns, sampled
// every couple of metres -- so the person and the car are on their line through every turn, with nothing doubling
// back. The leg under way runs from where the traveller is drawn to where its route ends (the stop-short beside the
// door, the reserved bay the car pulls into) and, on foot, on to the point it is for; a leg still to come is its
// planned route cut to the stretch alongside its two end points and joined to them. Every point stands on the surface
// under it: a car's on its link's driving surface, a walker's on the ground.
std::vector<Vec3> CityTripSystem::legLine(std::size_t k, std::vector<Vec3>& colours) const {
    const CitySim& sim = city_.sim();
    const TripLeg& L = plan_.choice()->legs[k];
    const Agent& a = sim.agents()[static_cast<std::size_t>(who_)];
    const Vec3 way = travel_.modes[static_cast<std::size_t>(L.mode)].colour;
    std::vector<Vec3> pts;
    colours.clear();
    // a point on the surface under it, in the colour of the road it is on (a car's: its kind of road's, ADR-0416)
    auto on = [&](Vec2 p, int link, Real station, bool car) {
        colours.push_back(car ? roadColour(link, way) : way);
        return Vec3(p.x, city_.surfaceY(car ? link : -1, station, p) + kPlanLift, p.y);
    };
    auto straight = [&](Vec2 b) {   // a piece off the network, on the ground
        const Vec2 a0(pts.back().x, pts.back().z);
        const int n = std::max(1, static_cast<int>(std::ceil((b - a0).length() / kLineStep)));
        for (int i = 1; i <= n; ++i) pts.push_back(on(a0 + (b - a0) * (static_cast<Real>(i) / n), -1, 0, false));
    };
    auto linkOf = [](const Agent& c) { return c.leg < static_cast<int>(c.route.links.size()) ? c.route.links[static_cast<std::size_t>(c.leg)] : -1; };
    auto drawnAt = [&](const Agent& c, int index) {   // where a pose is drawn: a car with its pull out of / into its bay
        if (c.mode != Agent::Mode::Driver) return index >= 0 ? city_.drawnAgentPos(index) : c.pos;
        return city_.drawnPlace(c);
    };
    if (k == static_cast<std::size_t>(leg_) && legUnderWay_) {
        const bool car = a.mode == Agent::Mode::Driver;
        pts.push_back(on(drawnAt(a, who_), linkOf(a), a.distOnLeg, car));
        bool first = true;
        sim.walkPoses(who_, kLineStep, [&](const Agent& c) {
            if (first) { first = false; return; }   // (where it is: drawn above)
            pts.push_back(on(drawnAt(c, -1), linkOf(c), c.distOnLeg, car));
        });
        if (!car) {   // on foot, on from the pavement to the door (or the car's): the steps still to take
            const Vec2 here(pts.back().x, pts.back().z);
            for (const Vec2 p : straight_ && !steps_.empty() ? steps_ : footSteps(here, walkTarget(k))) straight(p);
        }
        return pts;
    }
    // A LEG TO COME: its planned route, cut where it passes its ends
    const bool car = travel_.modes[static_cast<std::size_t>(L.mode)].roads;
    struct S { Vec2 p; int link; Real station; int leg; };
    std::vector<S> ss;
    if (!L.route.links.empty())
        sim.walkPoses(who_, kLineStep, [&](const Agent& c) { ss.push_back({c.pos, linkOf(c), c.distOnLeg, c.leg}); },
                      &L.route, car ? Agent::Mode::Driver : Agent::Mode::Pedestrian);
    pts.push_back(on(L.from, -1, 0, false));
    if (!ss.empty()) {
        const int lastLeg = static_cast<int>(L.route.links.size()) - 1;
        std::size_t i0 = 0, i1 = ss.size() - 1;
        Real b0 = 1e30, b1 = 1e30;
        for (std::size_t i = 0; i < ss.size(); ++i)
            if (ss[i].leg == 0 && (ss[i].p - L.from).length() < b0) { b0 = (ss[i].p - L.from).length(); i0 = i; }
        for (std::size_t i = i0; i < ss.size(); ++i)
            if (ss[i].leg == lastLeg && (ss[i].p - L.to).length() < b1) { b1 = (ss[i].p - L.to).length(); i1 = i; }
        straight(ss[i0].p);
        for (std::size_t i = i0 + 1; i <= i1; ++i) pts.push_back(on(ss[i].p, ss[i].link, ss[i].station, car));
    }
    if (car) {
        straight(L.to);
    } else {
        const Vec2 here(pts.back().x, pts.back().z);
        for (const Vec2 p : footSteps(here, walkTarget(k))) straight(p);
    }
    return pts;
}

// The colour of the line on `link` for a way drawn in `way`: its kind of road's, where travel.lua gives that kind one.
Vec3 CityTripSystem::roadColour(int link, const Vec3& way) const {
    const engine::NavGraph* nav = city_.sim().graph();
    if (!nav || link < 0 || link >= nav->linkCount()) return way;
    const int kind = travel_.roadKindOf(nav->links[static_cast<std::size_t>(link)].klass);
    return kind >= 0 && travel_.roadKinds[static_cast<std::size_t>(kind)].hasColour
               ? travel_.roadKinds[static_cast<std::size_t>(kind)].colour : way;
}

void CityTripSystem::draw(engine::FrameContext& ctx) {
    if (!active_) return;
    if (plan_.valid() && !arrived_) {
        const TripOption& o = *plan_.choice();
        if (lines_.size() != o.legs.size()) lines_.assign(o.legs.size(), {});
        if (lineColours_.size() != o.legs.size()) lineColours_.assign(o.legs.size(), {});
        for (std::size_t k = static_cast<std::size_t>(leg_); k < o.legs.size(); ++k) {
            // (a leg to come is the same line every frame: drawn once; the leg under way follows its traveller)
            const bool live = k == static_cast<std::size_t>(leg_) && legUnderWay_;
            if (live || lines_[k].empty()) lines_[k] = legLine(k, lineColours_[k]);
            for (std::size_t i = 1; i < lines_[k].size(); ++i)
                ctx.debug.line(lines_[k][i - 1], lines_[k][i], lineColours_[k][i]);
        }
    }
    const Vec3 foot(dest_.x, city_.surfaceY(-1, 0, dest_) + kPlanLift, dest_.y);
    ctx.debug.line(foot, foot + Vec3(0, kMarkHeight, 0), kMarkColour);
    ctx.debug.circle(foot, Vec3(0, 1, 0), 2.5, kMarkColour);
    ctx.debug.circle(foot + Vec3(0, kMarkHeight, 0), Vec3(0, 1, 0), 1.0, kMarkColour);
}

std::string CityTripSystem::status() const {
    if (!haveTravel_) return "trips are off: " + travelError_;
    if (!city_.built()) return "the city is not built yet";
    if (!active_) {
        const int p = person();
        if (p < 0) return "nobody in this city has a home to set out from";
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(p)];
        return "no trip yet: person " + std::to_string(p) + " at " + fmt("%.1f", a.pos.x) + " " + fmt("%.1f", a.pos.y) +
               (a.indoors ? " (indoors)" : "") + (a.vehicle >= 0 ? " (driving)" : "") +
               (a.car >= 0 ? ", owns car " + std::to_string(a.car) : ", no car") +
               " (trip near|far|freeway|to <place>|at <x> <z>|mark; keys Y near, I far, U freeway)" +
               (note_.empty() ? std::string() : " | " + note_);
    }
    std::string s = "trip " + std::to_string(trips_) + ": person " + std::to_string(who_) + " to " + where_ + " at " +
                    fmt("%.1f", dest_.x) + " " +
                    fmt("%.1f", dest_.y) + " | options:";
    for (const TripOption& o : plan_.options) {
        s += " " + describeOption(travel_, o);
        if (!o.legs.empty()) {
            s += " [";
            for (std::size_t k = 0; k < o.legs.size(); ++k) {
                const std::string roads = describeRoads(travel_, o.legs[k]);
                s += (k ? ", " : "") + travel_.modes[static_cast<std::size_t>(o.legs[k].mode)].name + " " +
                     fmt("%.0f m", o.legs[k].metres) + " " + fmt("%.1f min", o.legs[k].minutes) +
                     (roads.empty() ? "" : ": " + roads);
            }
            s += "]";
        }
        s += ";";
    }
    if (!plan_.valid()) return s + " | no way there from where they stand" + (note_.empty() ? "" : " | " + note_);
    const TripOption& o = *plan_.choice();
    s += " | chosen " + travel_.modes[static_cast<std::size_t>(o.mode)].name + ", because " + plan_.reason;
    if (arrived_) {
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
        s += " | arrived after " + fmt("%.1f min", (arrivedAt_ - startedAt_) / 60.0) + " (planned " +
             fmt("%.1f", plannedMinutes_) + "), " + fmt("%.1f m", (a.pos - dest_).length()) + " from the door";
    } else {
        const TripLeg& L = o.legs[static_cast<std::size_t>(leg_)];
        s += " | leg " + std::to_string(leg_ + 1) + "/" + std::to_string(o.legs.size()) + " " +
             travel_.modes[static_cast<std::size_t>(L.mode)].name + (legUnderWay_ ? " under way" : " starting") + ", " +
             fmt("%.0f m left", metresLeft()) + ", " + fmt("%.1f min", (simTime_ - startedAt_) / 60.0) + " so far";
    }
    if (!legActual_.empty()) {   // each leg done: its minutes, against the plan's
        s += " | legs (actual/planned min):";
        for (std::size_t k = 0; k < legActual_.size() && k < o.legs.size(); ++k)
            s += (k ? ", " : " ") + travel_.modes[static_cast<std::size_t>(o.legs[k].mode)].name + " " +
                 fmt("%.1f", legActual_[k]) + "/" + fmt("%.1f", o.legs[k].minutes);
    }
    {
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
        s += " | person at " + fmt("%.1f", a.pos.x) + " " + fmt("%.1f", a.pos.y) +
             (a.vehicle >= 0 ? " (driving)" : a.indoors ? " (indoors)" : " (on foot)");
    }
    if (!freewayNote_.empty()) s += " | freeway: " + freewayNote_;
    if (!note_.empty()) s += " | " + note_;
    return s;
}

// THE LINES ON SCREEN (the game UI layer, engine/ui/ui_text.h): where to, the options, the choice, the leg.
void CityTripSystem::render(engine::FrameContext& ctx) {
    if (!city_.built() || ctx.actions.layerUp()) return;
    std::vector<std::string> lines;
    if (!active_) {
        // (the keys' hint on the trip lab only: elsewhere nothing is drawn until a `trip` verb starts one)
        if (!haveTravel_ || city_.sim().daySchedule() || person() < 0) return;
        lines.push_back("Y: trip somewhere near    I: trip somewhere far    U: a trip on the freeway");
        if (!note_.empty()) lines.push_back(note_);
    } else {
        lines.push_back("Going to " + where_);
        for (std::size_t i = 0; i < plan_.options.size(); ++i)
            lines.push_back((static_cast<int>(i) == plan_.chosen ? "> " : "   ") + describeOption(travel_, plan_.options[i]));
        if (plan_.valid())   // the chosen way's roads, leg by leg (ADR-0416)
            for (const TripLeg& L : plan_.choice()->legs) {
                const std::string roads = describeRoads(travel_, L);
                if (!roads.empty())
                    lines.push_back("   " + travel_.modes[static_cast<std::size_t>(L.mode)].name + " on: " + roads);
            }
        if (!plan_.valid()) {
            lines.push_back("No way there from here");
        } else if (arrived_) {
            lines.push_back("Arrived after " + fmt("%.1f min", (arrivedAt_ - startedAt_) / 60.0) + " (planned " +
                            fmt("%.1f", plannedMinutes_) + ")");
        } else {
            const TripOption& o = *plan_.choice();
            const TripLeg& L = o.legs[static_cast<std::size_t>(leg_)];
            lines.push_back("Now: " + travel_.modes[static_cast<std::size_t>(L.mode)].name + ", leg " +
                            std::to_string(leg_ + 1) + " of " + std::to_string(o.legs.size()) + ", " +
                            fmt("%.0f m to go", metresLeft()));
        }
        if (!note_.empty()) lines.push_back(note_);
    }
    const float W = static_cast<float>(ctx.framebufferWidth);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    const float fs = 17.0f * k, gap = fs * 1.35f, pad = 10.0f * k, x0 = 16.0f * k, y0 = 16.0f * k;
    float wide = 0;
    for (const std::string& l : lines) wide = std::max(wide, ui_.measure(l, fs));
    const float white[4] = {1, 1, 1, 0.95f}, gold[4] = {1.0f, 0.82f, 0.3f, 1.0f}, back[4] = {0, 0, 0, 0.6f};
    std::vector<engine::Renderer::UiQuad> q;
    engine::UiText::rect(q, x0, y0, x0 + wide + 2 * pad, y0 + gap * static_cast<float>(lines.size()) + pad, back);
    for (std::size_t i = 0; i < lines.size(); ++i)
        ui_.text(ctx.renderer, q, x0 + pad, y0 + pad + gap * static_cast<float>(i) + ui_.ascent(fs) * 0.8f, lines[i],
                 fs, (lines[i].rfind("> ", 0) == 0) ? gold : white);
    ctx.renderer.submitUi(q);
}

}  // namespace citysim
