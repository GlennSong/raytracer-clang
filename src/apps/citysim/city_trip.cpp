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

namespace citysim {

using engine::Vec3;

namespace {
constexpr Real kPlanLift = 0.4;                          // the plan's line, above the surface it follows (m)
constexpr Real kMarkHeight = 14.0;                       // the destination's post (m)
const Vec3 kMarkColour(1.00, 0.78, 0.16);                // the waypoint's gold (the map's and the watch's)
const char* kOwner = "CityTripSystem";

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
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
    if (ctx.commands) {
        ctx.commands->add("trip", "trip near|far|to <place name or id>|at <x> <z>|mark",
                          "Send the city's person somewhere -- near or far (they pick the building), a place, a point, "
                          "or the map's waypoint -- to plan the trip and make it; poll trip?.",
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

void CityTripSystem::fixedUpdate(engine::FrameContext& ctx) { simTime_ += ctx.clock.fixedStep(); }

std::string CityTripSystem::stage(const engine::ControlCommand& cmd) {
    const std::string usage = "err usage: trip near|far|to <place name or id>|at <x> <z>|mark";
    if (cmd.args.empty()) return usage;
    const std::string what = lower(cmd.args[0]);
    if (what == "near" || what == "far" || what == "mark") {
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
    const std::string arg = request.size() > what.size() ? request.substr(what.size() + 1) : "";
    if (what == "near" || what == "far") {
        PlaceId id = kNoPlace;
        std::string why;
        if (!pickPlace(what == "far", id, why)) { note_ = "trip " + what + ": " + why; return; }
        begin(ctx, city_.places()[id].entrance, id, placeLabel(id) + ", picked " + what);
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
    leg_ = 0;
    legUnderWay_ = false;
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
    if (why.empty()) {
        if (sim.sendAgentTo(who_, L.to)) {
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
    ++leg_;
    legUnderWay_ = false;
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
    if (city_.sim().agents()[static_cast<std::size_t>(who_)].moving) return;
    finishLeg();
}

void CityTripSystem::update(engine::FrameContext& ctx) {
    // The keys only where the city keeps no day (citysim.schedule = false: the trip lab); the verbs work anywhere
    const bool lab = city_.built() && !city_.sim().daySchedule();
    if (lab && ctx.actions.pressed("trip_near")) requests_.push_back("near");
    if (lab && ctx.actions.pressed("trip_far")) requests_.push_back("far");
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
    }
    return std::max(Real(0), left);
}

// THE PLAN, DRAWN: each leg still to come along its route, in its way's colour -- the one under way along the route
// the person is really on -- and the destination's gold post.
void CityTripSystem::draw(engine::FrameContext& ctx) const {
    if (!active_) return;
    const engine::NavGraph& nav = *city_.sim().graph();
    int lane = 0;   // a planned drive on the lane by the centre line; the drive under way on the car's own lane
    auto at = [&](int li, Real t, bool roads) {
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
        const Vec2 p = roads ? nav.laneCenter(li, std::min(lane, std::max(0, L.lanes - 1)), t)
                             : (L.footpath ? L.footA + (L.footB - L.footA) * t : nav.sidewalkPoint(li, t));
        return Vec3(p.x, city_.surfaceY(li, t * L.length, p) + kPlanLift, p.y);
    };
    auto ground = [&](Vec2 p) { return Vec3(p.x, city_.surfaceY(-1, 0, p) + kPlanLift, p.y); };
    // (links are straight: one segment each; `firstT` is how far along the first one the line starts; `from`, when
    // given, is the point off the network it starts from -- the leg under way starts where the person is, on it)
    auto drawLinks = [&](const std::vector<int>& links, std::size_t first, Real firstT, bool roads, Vec3 c,
                         const Vec2* from, Vec2 to) {
        bool haveLast = from != nullptr;
        Vec3 last = from ? ground(*from) : Vec3();
        for (std::size_t k = first; k < links.size(); ++k) {
            const Vec3 a = at(links[k], k == first ? firstT : 0, roads), b = at(links[k], 1, roads);
            if (haveLast) ctx.debug.line(last, a, c);
            ctx.debug.line(a, b, c);
            last = b;
            haveLast = true;
        }
        if (haveLast) ctx.debug.line(last, ground(to), c);
    };
    if (plan_.valid() && !arrived_) {
        const TripOption& o = *plan_.choice();
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
        for (std::size_t k = static_cast<std::size_t>(leg_); k < o.legs.size(); ++k) {
            const TripLeg& L = o.legs[k];
            const TravelMode& m = travel_.modes[static_cast<std::size_t>(L.mode)];
            if (k == static_cast<std::size_t>(leg_) && legUnderWay_ && a.moving && a.leg >= 0 &&
                a.leg < static_cast<int>(a.route.links.size())) {
                const Real len = nav.links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(a.leg)])].length;
                lane = a.lane;
                drawLinks(a.route.links, static_cast<std::size_t>(a.leg), len > 1e-6 ? std::min(Real(1), a.distOnLeg / len) : 0,
                          a.mode == Agent::Mode::Driver, m.colour, nullptr, L.to);
            } else {
                lane = 0;
                drawLinks(L.route.links, 0, 0, m.roads, m.colour, &L.from, L.to);
            }
        }
    }
    const Vec3 foot = ground(dest_);
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
               " (trip near|far|to <place>|at <x> <z>|mark; keys Y near, I far)" +
               (note_.empty() ? std::string() : " | " + note_);
    }
    std::string s = "trip " + std::to_string(trips_) + ": person " + std::to_string(who_) + " to " + where_ + " at " +
                    fmt("%.1f", dest_.x) + " " +
                    fmt("%.1f", dest_.y) + " | options:";
    for (const TripOption& o : plan_.options) {
        s += " " + describeOption(travel_, o);
        if (!o.legs.empty()) {
            s += " [";
            for (std::size_t k = 0; k < o.legs.size(); ++k)
                s += (k ? ", " : "") + travel_.modes[static_cast<std::size_t>(o.legs[k].mode)].name + " " +
                     fmt("%.0f m", o.legs[k].metres) + " " + fmt("%.1f min", o.legs[k].minutes);
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
    {
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(who_)];
        s += " | person at " + fmt("%.1f", a.pos.x) + " " + fmt("%.1f", a.pos.y) +
             (a.vehicle >= 0 ? " (driving)" : a.indoors ? " (indoors)" : " (on foot)");
    }
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
        lines.push_back("Y: trip somewhere near    I: trip somewhere far");
        if (!note_.empty()) lines.push_back(note_);
    } else {
        lines.push_back("Going to " + where_);
        for (std::size_t i = 0; i < plan_.options.size(); ++i)
            lines.push_back((static_cast<int>(i) == plan_.chosen ? "> " : "   ") + describeOption(travel_, plan_.options[i]));
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
