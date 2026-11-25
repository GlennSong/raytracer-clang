#include "city_possess.h"
#include "../../engine/control/city_requests.h"

#include "city_brain.h"   // the households' cars: board and alight (ADR-0444)
#include "city_render.h"
#include "city_sim.h"

#include "../../engine/components.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>

namespace citysim {

namespace {

// Names for the agent's OWN mind, kept local: the goal layer's activityName()
// lives in city_goals.cpp without a header declaration, and telemetry is not
// worth widening an interface for.
const char* activityLabel(Activity a) {
    switch (a) {
        case Activity::AtHome:    return "at-home";
        case Activity::Commuting: return "commuting";
        case Activity::AtWork:    return "at-work";
        case Activity::Returning: return "returning";
        case Activity::Shopping:  return "shopping";
        case Activity::Outing:    return "outing";
        case Activity::Lunch:     return "lunch";
    }
    return "?";
}

const char* stateLabel(Agent::State s) {
    using S = Agent::State;
    switch (s) {
        case S::Resting:   return "resting";
        case S::Walking:   return "walking";
        case S::Avoiding:  return "avoiding";
        case S::Waiting:   return "waiting";
        case S::Cruising:  return "cruising";
        case S::Following: return "following";
        case S::Yielding:  return "yielding";
        case S::Turning:   return "turning";
        case S::Count:     break;
    }
    return "?";
}

// How far a director's menu reaches. Places are a destination you would cross
// a street for; agents are who is actually around you.
constexpr engine::Real kLookRadiusPlaces = 250.0;
constexpr engine::Real kLookRadiusAgents = 40.0;
constexpr int kLookMaxAgents = 6;
// How close to a door you have to be to walk through it (as close as to a car: kBoardReach, city_sim.h).
constexpr engine::Real kEnterReach = 25.0;


}  // namespace

using engine::Entity;
using engine::Vec2;
using engine::Vec3;

namespace {

// The spectate rig's presets — a car reads best a little farther and higher
// than a pedestrian.
void configureFollow(engine::FollowCameraController& f, bool car) {
    f.distance = car ? 8.0 : 5.0;
    f.minDistance = car ? 3.0 : 2.0;
    f.maxDistance = car ? 22.0 : 12.0;
    f.targetHeight = car ? 1.5 : 1.2;
    f.orbitPitch = car ? -14.0 : -12.0;
    f.orbitYaw = 0.0;
    f.lateralOffset = 0.0;
}

}  // namespace

void CityPossessSystem::onStart(engine::FrameContext& ctx) {
    onRequest_ = ctx.events.listen<engine::PossessRequest>([this](const engine::PossessRequest& r) {
        requests_.push_back(r.command);
    });
    if (ctx.commands == nullptr) return;
    auto answer = [](const std::string* line) {
        return [line](const engine::ControlCommand&) { return "ok " + *line; };
    };
    answerStatus_ = ctx.commands->bind("possess?", "CityPossessSystem", answer(&statusLine_));
    answerLook_ = ctx.commands->bind("look?", "CityPossessSystem", answer(&lookLine_));
    answerAgent_ = ctx.commands->bind("agent?", "CityPossessSystem", answer(&agentLine_));
}

void CityPossessSystem::update(engine::FrameContext& ctx) {
    // The channel's requests, in the order they came. Taken even when the
    // city isn't built so the sender gets a status explaining why nothing
    // happened, not silence.
    std::vector<std::string> requests;
    requests.swap(requests_);
    for (const std::string& line : requests) handleCommand(ctx, parsePossessCmd(line));

    updateCamera(ctx);
    publishStatus();
    publishLook();
}

void CityPossessSystem::handleCommand(engine::FrameContext& ctx,
                                      const PossessCmd& cmd) {
    switch (cmd.kind) {
        case PossessCmd::Kind::None:
            // NOTHING STAGED. Clearing the error here — which is what this did
            // — meant a refusal lived for exactly one frame: a director that
            // sends a command and polls a second later NEVER sees why it was
            // refused, and reads the stale success instead. The error stands
            // until the next real command replaces it.
            return;
        case PossessCmd::Kind::Invalid:
            error_ = cmd.error;
            return;
        default:
            break;
    }
    error_.clear();   // a real command: start clean, then let it set its own
    switch (cmd.kind) {
        case PossessCmd::Kind::Walker:
            possessWalker(ctx, cmd);
            return;
        case PossessCmd::Kind::WalkTo:
            if (walkerAgent_ >= 0) walkTo(cmd.x, cmd.z);
            else error_ = "nothing possessed — `possess walker` first";
            return;
        case PossessCmd::Kind::Stop:
            hasDest_ = false;
            state_ = walkerAgent_ >= 0 ? PossessState::Idle : PossessState::None;
            return;
        case PossessCmd::Kind::Direct:
            // THE GOAL LAYER STOPS FIGHTING THE DIRECTOR (ADR-0091). Without
            // this an errand ends, the schedule chains the next trip seconds
            // later, and a director spends the session arguing with the table
            // it is supposed to be replacing.
            if (walkerAgent_ >= 0) {
                city_.simMutable().setAgentDirected(walkerAgent_, cmd.on);
                directed_ = cmd.on;
            } else {
                error_ = "nothing possessed — `possess walker` first";
            }
            return;
        case PossessCmd::Kind::Board:
            boardNearby(cmd);
            return;
        case PossessCmd::Kind::Alight:
            alightHere();
            return;
        case PossessCmd::Kind::Enter:
        case PossessCmd::Kind::Exit: {
            if (walkerAgent_ < 0) { error_ = "nothing possessed"; return; }
            const bool inside = cmd.kind == PossessCmd::Kind::Enter;
            // YOU HAVE TO BE AT A DOOR. Without this `enter` is teleportation
            // into the nearest wall: the director could put the agent inside a
            // building from across the district, and the one thing possession
            // has to keep true is that the agent only does what it could walk
            // up and do.
            std::string what = "the street";
            if (inside) {
                const auto& agents = city_.sim().agents();
                if (walkerAgent_ >= static_cast<int>(agents.size())) return;
                const Vec2 me = agents[static_cast<std::size_t>(walkerAgent_)].pos;
                const PlaceMap& places = city_.places();
                engine::Real bestD = kEnterReach;
                PlaceId bestId = kNoPlace;
                for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
                    const PlaceId id = places.nearest(static_cast<PlaceType>(t), me);
                    if (id == kNoPlace) continue;
                    const Place& pl = places[id];
                    if (!placeIsIndoors(pl.type)) continue;
                    const engine::Real d =
                        std::sqrt((pl.entrance.x - me.x) * (pl.entrance.x - me.x) +
                                  (pl.entrance.y - me.y) * (pl.entrance.y - me.y));
                    if (d < bestD) { bestD = d; bestId = id; }
                }
                if (bestId == kNoPlace) {
                    error_ = "no door within reach — walk to one first";
                    return;
                }
                what = placeTypeName(places[bestId].type);
            }
            city_.simMutable().setAgentIndoors(walkerAgent_, inside);
            LOG_INFO << (inside ? "enter: agent " : "exit: agent ") << walkerAgent_
                     << " " << (inside ? "into " : "out of ") << what;
            hasDest_ = false;
            state_ = PossessState::Idle;
            return;
        }
        case PossessCmd::Kind::Release:
            releasePossession();
            return;
        case PossessCmd::Kind::None:
        case PossessCmd::Kind::Invalid:
            return;   // (taken above)
    }
}

void CityPossessSystem::possessWalker(engine::FrameContext& ctx,
                                      const PossessCmd& cmd) {
    if (!city_.built()) {
        error_ = "no city here — possession needs play mode on a city level";
        return;
    }
    releasePossession();

    // Commandeer the nearest live pedestrian: they already have a routed life,
    // a body when near, and a walk cycle — possession just redirects them.
    Vec2 ref(cmd.x, cmd.z);
    if (!cmd.hasPos)
        ref = Vec2(ctx.view.camera.position.x, ctx.view.camera.position.z);
    const auto& agents = city_.sim().agents();
    int best = -1;
    engine::Real bestD2 = 1e30;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const Agent& a = agents[i];
        if (a.released || a.inCar >= 0) continue;   // (in a household's car: no walker)
        const engine::Real dx = a.pos.x - ref.x, dz = a.pos.y - ref.y;
        const engine::Real d2 = dx * dx + dz * dz;
        if (d2 < bestD2) { bestD2 = d2; best = static_cast<int>(i); }
    }
    if (best < 0) {
        error_ = "no pedestrians to possess";
        return;
    }
    walkerAgent_ = best;
    // (ADR-0443) a resident with the agent brain: its `possessed` state holds them while a director moves them
    city_.simMutable().brainNoteHeld(walkerAgent_, true);
    configureFollow(follow_, /*car=*/false);
    state_ = PossessState::Idle;
    hasDest_ = false;
}

void CityPossessSystem::walkTo(engine::Real x, engine::Real z) {
    if (!city_.sim().roads()) { error_ = "no road model"; return; }
    // One line per director errand. It prints where the agent STOOD and where
    // it was placed, because those differing is the whole bug this path had:
    // sendAgentTo used to put the planner ghost on its new route's first leg,
    // away from the physical body, and the ADR-0062 tether then held it for
    // ever. They should now be the same point.
    const Vec2 was = city_.sim().agents()[static_cast<std::size_t>(walkerAgent_)].pos;
    if (!city_.simMutable().sendAgentTo(walkerAgent_, Vec2(x, z))) {
        LOG_INFO << "walk_to: agent " << walkerAgent_ << " REFUSED " << x << ","
                 << z << " — nothing near it routes from where the agent stands;"
                 << " it keeps its own plan";
        state_ = PossessState::NoRoute;
        return;
    }
    {
        const Agent& a = city_.sim().agents()[static_cast<std::size_t>(walkerAgent_)];
        // Why an errand ends where it does: its walk route (#306 piece 6 W3) ends where the destination joins the walks
        LOG_INFO << "walk_to: agent " << walkerAgent_ << " -> " << x << "," << z
                 << " | stood at " << was.x << "," << was.y << ", placed at "
                 << a.pos.x << "," << a.pos.y << " on step " << a.leg << "/"
                 << a.route.steps.size() << ", " << a.route.metres << " m of walks, " << a.route.crossings.size()
                 << " crossings";
    }
    dest_ = Vec2(x, z);
    hasDest_ = true;
    state_ = PossessState::Walking;
}

// WALK UP TO A CAR AND TAKE IT (ADR-0091). The reach is deliberately short —
// a director has to actually send the agent to the car first, which is the
// whole point of the exercise: the errand, the arrival and the boarding are
// three decisions, not one teleport.
void CityPossessSystem::boardNearby(const PossessCmd& cmd) {
    if (walkerAgent_ < 0) {
        error_ = "nothing possessed — `possess walker` first";
        return;
    }
    CitySim& sim = city_.simMutable();
    const auto& agents = sim.agents();
    if (walkerAgent_ >= static_cast<int>(agents.size())) return;
    const Agent& a = agents[static_cast<std::size_t>(walkerAgent_)];
    if (a.inCar >= 0) { error_ = "already driving"; return; }
    const Vec2 from = cmd.hasPos ? Vec2(cmd.x, cmd.z) : a.pos;
    // a household's car of the agent brain, which is an entity (ADR-0444): the only cars there are to get into
    CityBrain* households = sim.brain();
    const int car = households ? households->nearestFreeCar(from, kBoardReach) : -1;
    Vec2 where, heading;
    if (car < 0 || !households->carPose(car, where, heading)) {
        error_ = "no free car within reach — walk to one first";
        state_ = PossessState::Idle;
        return;
    }
    const engine::Real walk = std::sqrt((where.x - a.pos.x) * (where.x - a.pos.x) +
                                        (where.y - a.pos.y) * (where.y - a.pos.y));
    if (walk > kBoardReach) {
        error_ = "that car is too far to get into — walk to it first";
        return;
    }
    if (!households->board(walkerAgent_, car)) {
        error_ = "that car is taken";
        return;
    }
    LOG_INFO << "board: agent " << walkerAgent_ << " took household car " << car << " at " << where.x << ","
             << where.y << " (" << walk << " m away)";
    configureFollow(follow_, /*car=*/true);
    hasDest_ = false;
    state_ = PossessState::Idle;
}

void CityPossessSystem::alightHere() {
    if (walkerAgent_ < 0) { error_ = "nothing possessed"; return; }
    // (out of a household's car of the agent brain: ADR-0444)
    CityBrain* households = city_.simMutable().brain();
    if (!(households && households->alight(walkerAgent_))) {
        error_ = "not in a car";
        return;
    }
    LOG_INFO << "alight: agent " << walkerAgent_ << " got out";
    configureFollow(follow_, /*car=*/false);
    hasDest_ = false;
    state_ = PossessState::Idle;
}

void CityPossessSystem::releasePossession() {
    // GIVE THE AGENT ITS DAY BACK. A released agent left `directed` would stand
    // wherever the director abandoned it for the rest of the run, because the
    // goal layer has been told to leave it alone (ADR-0091).
    if (walkerAgent_ >= 0 && city_.built()) {
        city_.simMutable().setAgentDirected(walkerAgent_, false);
        // (ADR-0443) a resident with the agent brain is let go: they plan again from where they stand, with the
        // intent they had (and stay the brain's, not the day table's)
        city_.simMutable().brainNoteHeld(walkerAgent_, false);
    }
    directed_ = false;
    walkerAgent_ = -1;
    hasDest_ = false;
    state_ = PossessState::None;
}

void CityPossessSystem::fixedUpdate(engine::FrameContext&) {
    // Walker arrival is judged from the ghost — the sim drives the walking.
    if (walkerAgent_ >= 0 && state_ == PossessState::Walking && hasDest_) {
        const auto& agents = city_.sim().agents();
        if (walkerAgent_ < static_cast<int>(agents.size())) {
            const Agent& a = agents[walkerAgent_];
            const engine::Real dx = a.pos.x - dest_.x, dz = a.pos.y - dest_.y;
            if (dx * dx + dz * dz < 5.0 * 5.0) state_ = PossessState::Arrived;
        }
    }
}

bool CityPossessSystem::walkerPose(Vec3& outPos, Vec2& outHeading) const {
    // Physical body pose when the walker has one; the sim GHOST otherwise —
    // agentWorldPose only reports external bodies, and a possessed pedestrian
    // beyond the body radius read as pos 0,0,0 (frozen camera, frozen
    // remaining) on the first live walk.
    if (city_.agentWorldPose(walkerAgent_, outPos, outHeading)) return true;
    const auto& agents = city_.sim().agents();
    if (walkerAgent_ < 0 || walkerAgent_ >= static_cast<int>(agents.size()))
        return false;
    const Agent& a = agents[walkerAgent_];
    outPos = Vec3(a.pos.x, city_.groundHeightAt(a.pos.x, a.pos.y) + 0.9,
                  a.pos.y);
    outHeading = a.heading;
    return true;
}

void CityPossessSystem::updateCamera(engine::FrameContext& ctx) {
    Vec3 worldPos;
    if (walkerAgent_ < 0) return;
    Vec2 heading;
    if (!walkerPose(worldPos, heading)) return;
    const engine::Real yawDeg = engine::radiansToDegrees(std::atan2(heading.x, -heading.y));

    // Zoom-only input, heading owns the yaw (the spectate idiom — a full
    // update() would double-apply look input over PlayerSystem's).
    follow_.setTarget(worldPos, yawDeg);
    engine::CameraInput zoom;
    zoom.zoomDelta = ctx.input.scrollDelta;
    follow_.update(zoom, ctx.frameDelta);
    const float aspect =
        (ctx.framebufferHeight > 0)
            ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
            : 1.0f;
    ctx.view.camera = follow_.cameraState(aspect);
    ctx.view.activeCameraEntity = Entity{};
}

// WHAT THE AGENT CAN SEE (ADR-0091). The menu a director chooses from, and the
// reason it is built HERE rather than by the director: every entry carries a
// point `walk_to` accepts, so a choice is one command and never a coordinate
// the agent cannot reach. A place's `at` is its ENTRANCE — already snapped to
// the pedestrian network by PlaceMap — not its footprint centroid, which is
// inside a building and routes to nothing.
//
// Bearings are degrees from where the agent faces, positive to its RIGHT, the
// same convention (and the same two lines of arithmetic) as the bus HUD the
// player reads, so the two never disagree about which way "left" is.
void CityPossessSystem::publishLook() {
    if (!city_.built() || walkerAgent_ < 0) {
        lookLine_ = "none";
        return;
    }
    const std::vector<Agent>& agents = city_.sim().agents();
    if (walkerAgent_ >= static_cast<int>(agents.size())) {
        lookLine_ = "none";
        return;
    }
    const CitySim& sim = city_.sim();
    const Agent& me = agents[static_cast<std::size_t>(walkerAgent_)];
    const Vec2 eye = me.pos;
    Vec2 f = me.heading;
    const engine::Real fl = std::sqrt(f.x * f.x + f.y * f.y);
    f = fl > 1e-6 ? Vec2(f.x / fl, f.y / fl) : Vec2(0, 1);
    const engine::Real clock = sim.clockHours();

    auto bearing = [&](Vec2 to) {
        const Vec2 d(to.x - eye.x, to.y - eye.y);
        return std::atan2(-d.x * f.y + d.y * f.x, d.x * f.x + d.y * f.y) *
               57.29577951308232;
    };
    auto dist = [&](Vec2 to) {
        const engine::Real dx = to.x - eye.x, dy = to.y - eye.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    std::string out;
    char buf[320];
    std::snprintf(buf, sizeof(buf), "clock=%.2f pos=%.1f,%.1f facing=%.1f,%.1f",
                  static_cast<double>(clock), static_cast<double>(eye.x),
                  static_cast<double>(eye.y), static_cast<double>(f.x),
                  static_cast<double>(f.y));
    out = buf;

    // WHERE ITS OWN PLAN GOES NEXT, so a director can see what it is
    // interrupting before it interrupts it.
    if (CitySim::onRoute(me)) {
        // the end of the step it is on (W3: its walk route's steps)
        const Vec2 wp = sim.routePoint(me, me.leg, sim.legLength(me, me.leg));
        std::snprintf(buf, sizeof(buf),
                      " | next d=%.0f b=%.0f at=%.1f,%.1f legs=%d",
                      static_cast<double>(dist(wp)),
                      static_cast<double>(bearing(wp)),
                      static_cast<double>(wp.x), static_cast<double>(wp.y),
                      static_cast<int>(me.route.steps.size()) - me.leg);
        out += buf;
    }

    // ONE PLACE OF EACH KIND, the nearest. A menu of seven is a menu a model
    // can hold in its head; every shop in the district is a search problem.
    const PlaceMap& places = city_.places();
    for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
        const PlaceType type = static_cast<PlaceType>(t);
        const PlaceId id = places.nearest(type, eye);
        if (id == kNoPlace) continue;
        const Place& pl = places[id];
        const engine::Real d = dist(pl.entrance);
        if (d > kLookRadiusPlaces) continue;
        std::string name = pl.name;
        for (char& ch : name) if (ch == ' ') ch = '_';   // one token per field
        std::snprintf(buf, sizeof(buf),
                      " | place=%s id=%u d=%.0f b=%.0f open=%d at=%.1f,%.1f%s%s",
                      placeTypeName(type), static_cast<unsigned>(id),
                      static_cast<double>(d), static_cast<double>(bearing(pl.entrance)),
                      pl.openAt(clock) ? 1 : 0,
                      static_cast<double>(pl.entrance.x),
                      static_cast<double>(pl.entrance.y),
                      name.empty() ? "" : " name=", name.c_str());
        out += buf;
    }


    // A CAR IT COULD TAKE — the nearest one with nobody in it, out to the same
    // range as the places, because a car two streets away is a destination you
    // can walk to. `boardable` is the different question: can it reach in and
    // open the door from where it is standing right now.
    // (A household's car of the new driving: the only cars `board` gets into, ADR-0459.)
    if (const CityBrain* households = sim.brain()) {
        const int veh = households->nearestFreeCar(eye, kLookRadiusPlaces);
        Vec2 vp, vh;
        if (veh >= 0 && households->carPose(veh, vp, vh)) {
            const engine::Real d = dist(vp);
            std::snprintf(buf, sizeof(buf),
                          " | car_free=%d d=%.1f b=%.0f at=%.1f,%.1f boardable=%d",
                          veh, static_cast<double>(d),
                          static_cast<double>(bearing(vp)),
                          static_cast<double>(vp.x), static_cast<double>(vp.y),
                          d <= kBoardReach ? 1 : 0);
            out += buf;
        }
    }

    // THE PEOPLE AROUND IT, nearest first, capped. The grid answers by CELL, so
    // this is a superset — the distance test below is the exact one.
    {
        std::vector<int> near;
        sim.queryAgentsNear(eye, kLookRadiusAgents, near);
        std::vector<std::pair<engine::Real, int>> byDist;
        byDist.reserve(near.size());
        for (int ai : near) {
            if (ai == walkerAgent_ || ai < 0 ||
                ai >= static_cast<int>(agents.size()))
                continue;
            const Agent& o = agents[static_cast<std::size_t>(ai)];
            if (o.indoors || o.released) continue;   // not on the street
            const engine::Real d = dist(o.pos);
            if (d <= kLookRadiusAgents) byDist.emplace_back(d, ai);
        }
        std::sort(byDist.begin(), byDist.end());
        int shown = 0;
        for (const auto& [d, ai] : byDist) {
            if (shown++ >= kLookMaxAgents) break;
            const Agent& o = agents[static_cast<std::size_t>(ai)];
            std::snprintf(buf, sizeof(buf),
                          " | %s=%d d=%.1f b=%.0f state=%s speed=%.1f",
                          o.inCar >= 0 ? "car" : "ped", ai,
                          static_cast<double>(d), static_cast<double>(bearing(o.pos)),
                          stateLabel(o.state), static_cast<double>(o.speed));
            out += buf;
        }
        std::snprintf(buf, sizeof(buf), " | crowd=%d", static_cast<int>(byDist.size()));
        out += buf;
    }

    lookLine_ = out;
}

void CityPossessSystem::publishStatus() {
    std::string line;
    if (!error_.empty()) {
        line = "err " + error_;
    } else if (walkerAgent_ >= 0) {
        Vec3 wp(0, 0, 0);
        Vec2 heading;
        walkerPose(wp, heading);
        engine::Real remaining = 0;
        if (hasDest_) {
            const engine::Real dx = wp.x - dest_.x, dz = wp.z - dest_.y;
            remaining = std::sqrt(dx * dx + dz * dz);
        }
        const auto& agents = city_.sim().agents();
        const engine::Real spd =
            walkerAgent_ < static_cast<int>(agents.size())
                ? agents[walkerAgent_].speed
                : 0.0;
        line = formatPossessStatus("walker", state_, wp, spd, remaining);
    } else {
        line = city_.built() ? "none" : "none (city not built — play mode only)";
    }
    statusLine_ = line;

    // THE AGENT'S OWN MIND (ADR-0091 telemetry). possess.status answers "what
    // is the puppeteer doing"; this answers "what is this person doing" — the
    // activity and goal state the sim is running them under, and the four
    // fields that decide whether they take a step (moving/indoors/restDwell/
    // route). A director cannot reason about an agent it cannot read, and the
    // walk_to freeze was invisible precisely because none of this was exposed.
    std::string agentLine = "none";
    if (walkerAgent_ >= 0) {
        const auto& sim = city_.sim();
        const auto& agents = sim.agents();
        if (walkerAgent_ < static_cast<int>(agents.size())) {
            const Agent& a = agents[static_cast<std::size_t>(walkerAgent_)];
            char buf[512];
            std::snprintf(
                buf, sizeof(buf),
                "agent=%d mode=%s activity=%s state=%s moving=%d "
                "indoors=%d restDwell=%.1f speed=%.2f pos=%.1f,%.1f "
                "leg=%d/%d routeValid=%d tripGoal=%d home=%d work=%d "
                "train=%d inCar=%d stepped=%d gap=%.1f minGap=%.1f "
                "tethered=%d anchorDist=%.1f lead=%.1f playerCtl=%d released=%d "
                "cityHeld=%d",
                walkerAgent_,
                a.inCar >= 0 ? "car" : "ped",   // in a household's car (inCar), else on foot
                activityLabel(a.activity), stateLabel(a.state),
                a.moving ? 1 : 0, a.indoors ? 1 : 0,
                static_cast<double>(a.restDwell), static_cast<double>(a.speed),
                static_cast<double>(a.pos.x), static_cast<double>(a.pos.y),
                a.leg, static_cast<int>(a.route.steps.size()),
                a.route.found ? 1 : 0, a.tripGoal, a.home, a.work,
                // what is carrying it: a train (past the street on a rail journey), or a household's car
                sim.onTrainJourney(walkerAgent_) ? 1 : 0, a.inCar,
                sim.steppedLastTick(walkerAgent_) ? 1 : 0,
                static_cast<double>(sim.gapOf(walkerAgent_)),
                static_cast<double>(sim.minGapOf(walkerAgent_)),
                // The ADR-0062 leash from the director's side: a ghost more than
                // `lead` metres from its physical body is held, not stepped.
                a.tethered ? 1 : 0,
                a.tethered ? static_cast<double>((a.pos - a.tetherAnchor).length())
                           : -1.0,
                static_cast<double>(a.tetherLead),
                a.playerControlled ? 1 : 0, a.released ? 1 : 0, sim.tetherHeld());
            agentLine = buf;
        }
    }
    agentLine_ = agentLine;
}

}  // namespace citysim
