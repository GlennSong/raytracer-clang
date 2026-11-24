#include "city_brain_system.h"

#include "city_brain.h"
#include "city_render.h"
#include "city_sim.h"
#include "city_spectate.h"
#include "screen_project.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace citysim {

using engine::Mat4;
using engine::Vec3;

namespace {
const char* kOwner = "CityBrainSystem";
constexpr Real kPostHeight = 9.0;        // a marker's post (m)
// (the ring is round the spot where a person at the marker stands, and the post beside it: kMarkerRingRadius and
// kMarkerPostOffset, city_brain.h)
constexpr Real kLift = 0.4;              // lines drawn this far above the surface they follow (m)
// A DRAWN LINE IS LAID IN PIECES NO LONGER THAN THIS (m). The debug lines are ribbons whose width is worked out from the
// camera's distance to each piece's middle (engine/debug_draw.h), so they stay a few pixels wide only while a piece is
// short beside that distance: one 700 m piece starting at the camera was drawn 1.4 m wide where it began.
constexpr Real kLineStep = 2.0;
// A leg that has no route yet (a walk or a drive to come, a car that has not left its space) is a straight DASHED line
// between its ends: this many metres drawn, then as many left out.
constexpr Real kDash = 4.0;
// Where a drive's line begins on its route: the route's point nearest the car's front, looked for no further on than
// the leg the car is on and the one after it (so a route that comes back past the car is not cut short).
constexpr std::size_t kLegsLookedAtForTheCar = 2;
constexpr Real kLabelRange = 450.0;      // a marker's name is drawn within this of the camera (m)
constexpr Real kNameRange = 120.0;       // ...and a resident's within this
constexpr Real kNameHeight = 2.1;        // a resident's name is drawn this far above their feet (m)
// INSPECT BY LOOKING (city_brain_system.h): a resident or a household car counts as looked at when it is within
// kLookWithin of the line through the middle of the view and no farther than kLookRange from the camera; failing
// that, the resident within kLookNear of the camera. Measured to a point kLookHeight above the ground (a chest, a
// car's roof line).
constexpr Real kLookWithin = 1.5;
constexpr Real kLookRange = 80.0;
constexpr Real kLookNear = 6.0;
constexpr Real kLookHeight = 1.0;
const Vec3 kMarkerColour(1.00, 0.78, 0.16);   // the waypoint's gold (the trip lab's destination post)
const Vec3 kWantedColour(1.00, 1.00, 1.00);   // the marker the followed resident wants to be at

// A straight line from `a` to `b` in pieces no longer than kLineStep; `dashed`: kDash drawn, kDash left out.
void pieces(engine::DebugDraw& debug, const Vec3& a, const Vec3& b, const Vec3& colour, bool dashed) {
    const Real length = (b - a).length();
    if (!(length > 0)) return;
    const Vec3 way = (b - a) * (1.0 / length);
    for (Real s = 0; s < length; s += kLineStep) {
        if (dashed && static_cast<long>(s / kDash) % 2 == 1) continue;
        debug.line(a + way * s, a + way * std::min(length, s + kLineStep), colour);
    }
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}
std::string fmt(const char* f, double a) {
    char b[64];
    std::snprintf(b, sizeof(b), f, a);
    return b;
}
// A `brain?` line as the lines of a panel: one for each part between its " | " marks.
std::vector<std::string> parts(const std::string& line) {
    std::vector<std::string> out;
    const std::string mark = " | ";
    std::size_t from = 0;
    while (from <= line.size()) {
        const std::size_t at = line.find(mark, from);
        out.push_back(line.substr(from, at == std::string::npos ? std::string::npos : at - from));
        if (at == std::string::npos) break;
        from = at + mark.size();
    }
    return out;
}
const char* const kNoBrain = "err this level states no residents (citysim.residents, citysim.households)";
}  // namespace

void CityBrainSystem::onStart(engine::FrameContext& ctx) {
    if (!ctx.commands) return;
    ctx.commands->add("brain?", "brain? [who]",
                      "A resident with the agent brain in one line (the followed one, or the one named): what they want, the "
                      "plan, the leg under way, the state and since when, what they wait for, where they and their "
                      "household's cars are, and why the plan was chosen.",
                      [this](const engine::ControlCommand& c) {
                          const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
                          if (!b) return std::string("err this level states no markers and no residents (citysim.markers, citysim.residents)");
                          if (c.args.empty()) return "ok " + b->describe(followed_);
                          const int who = b->residentNamed(c.args[0]);
                          return who < 0 ? "err no resident \"" + c.args[0] + "\" (residents?)" : "ok " + b->describe(who);
                      },
                      kOwner);
    ctx.commands->add("residents?", "residents?",
                      "Every resident with the agent brain: name, household, what they are doing and where.",
                      [this](const engine::ControlCommand&) { return residentsLine(); }, kOwner);
    ctx.commands->add("markers?", "markers?",
                      "Every marker of the level: its name, street, station, kerb and where a person stands at it.",
                      [this](const engine::ControlCommand&) { return markersLine(); }, kOwner);
    ctx.commands->add("goto", "goto [who] <marker>",
                      "A resident with the agent brain (the followed one, or the one named) wants to be at that marker from "
                      "now: they plan the trip and make it. Stops their day table (brain day on resumes it). Poll brain?.",
                      [this](const engine::ControlCommand& c) { return gotoVerb(c); }, kOwner);
    ctx.commands->add("brain", "brain trait [who] <name> on|off | day [who|all] on|off | follow [who|off] | labels on|off | attention",
                      "Give or take one of a resident's traits (travel.lua: has_board, has_scooter), used from their next "
                      "plan; switch a day table; put the camera on the next resident or the one named, or let it go (then "
                      "`camera` places it); show or hide the places' and residents' names. With no name: the followed "
                      "resident.",
                      [this](const engine::ControlCommand& c) { return brainVerb(c); }, kOwner);
}

void CityBrainSystem::onStop(engine::FrameContext& ctx) {
    if (ctx.commands) ctx.commands->removeOwnedBy(kOwner);
    ui_.release(ctx.renderer);
}

std::string CityBrainSystem::markersLine() const {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b) return "err this level states no markers (citysim.markers)";
    const engine::RoadModel* model = city_.sim().drivingV2() ? city_.sim().drivingV2()->model() : nullptr;
    std::string s = "ok " + std::to_string(b->markers().size()) + " markers:";
    for (const BrainMarker& m : b->markers()) {
        const std::string street = model && m.frontage.street.index < model->streets.size()
                                       ? model->streets[m.frontage.street.index].id : std::string("?");
        s += " " + m.name + " (street " + street + ", station " + fmt("%.1f", m.frontage.station) + ", the kerb of the traffic running " +
             (m.frontage.dir > 0 ? "along" : "against") + " the centre line, " + (m.stated ? "stated" : "looked up from a point") +
             "; stand at " + fmt("%.1f", m.stand.x) + " " + fmt("%.1f", m.stand.y) + ");";
    }
    return s;
}

std::string CityBrainSystem::residentsLine() const {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b || !b->hasResident()) return kNoBrain;
    std::string s = "ok " + std::to_string(b->residentCount()) + " residents:";
    for (int who = 0; who < b->residentCount(); ++who) {
        const Vec2 pos = city_.sim().agents()[static_cast<std::size_t>(b->residentAgent(who))].pos;   // (in a car: the car's)
        const int wants = b->wantedMarker(who);
        const std::string at = b->standingAt(who);
        s += " " + std::to_string(who) + " " + b->residentName(who) + " (household " +
             (b->householdName(who).empty() ? std::string("none") : b->householdName(who)) + "; " + b->stateLabel(who) +
             (at.empty() ? std::string() : " at " + at) + "; wants " +
             (wants >= 0 ? b->markers()[static_cast<std::size_t>(wants)].name : std::string("?")) + "; at " + fmt("%.1f", pos.x) +
             " " + fmt("%.1f", pos.y) + (b->inCar(who) ? " in the car" : "") + (who == followed_ ? "; followed" : "") + ");";
    }
    return s;
}

std::string CityBrainSystem::gotoVerb(const engine::ControlCommand& cmd) {
    CityBrain* b = city_.built() ? city_.simMutable().brain() : nullptr;
    if (!b || !b->hasResident()) return kNoBrain;
    if (cmd.args.empty() || cmd.args.size() > 2) return "err usage: goto [who] <marker>";
    const int who = cmd.args.size() == 2 ? b->residentNamed(cmd.args[0]) : followed_;
    if (who < 0) return "err no resident \"" + cmd.args[0] + "\" (residents?)";
    std::string why;
    if (!b->goTo(who, cmd.args.back(), &why)) return "err " + why;
    return "ok " + b->residentName(who) + " wants to be at " + cmd.args.back() + " (poll brain? " + b->residentName(who) + ")";
}

std::string CityBrainSystem::brainVerb(const engine::ControlCommand& cmd) {
    const std::string usage =
        "err usage: brain trait [who] <name> on|off | day [who|all] on|off | follow [who|off] | labels on|off | attention";
    CityBrain* b = city_.built() ? city_.simMutable().brain() : nullptr;
    if (!b || !b->hasResident()) return kNoBrain;
    if (cmd.args.empty()) return usage;
    const std::string what = lower(cmd.args[0]);
    const auto onOff = [&](const std::string& word, bool& on) {
        const std::string w = lower(word);
        on = w == "on";
        return w == "on" || w == "off";
    };
    const std::string noSuch = "err no resident \"" + (cmd.args.size() > 1 ? cmd.args[1] : std::string()) + "\" (residents?)";
    bool on = false;
    // the places' names and the residents' names over the view (#299: a probe's pictures turn them off)
    // who is far, and why each near person is near (ADR-0493)
    if (what == "attention" && cmd.args.size() == 1) return "ok " + b->attentionText();
    if (what == "labels" && cmd.args.size() == 2 && onOff(cmd.args[1], on)) {
        labels_ = on;
        return std::string("ok labels ") + (on ? "on" : "off");
    }
    if (what == "trait" && (cmd.args.size() == 3 || cmd.args.size() == 4) && onOff(cmd.args.back(), on)) {
        const int who = cmd.args.size() == 4 ? b->residentNamed(cmd.args[1]) : followed_;
        if (who < 0) return noSuch;
        const std::string& trait = cmd.args[cmd.args.size() - 2];
        std::string why;
        if (!b->setTrait(who, trait, on, &why)) return "err " + why;
        return "ok " + b->residentName(who) + (on ? " has " : " no longer has ") + trait + " (used from their next plan)";
    }
    if (what == "day" && (cmd.args.size() == 2 || cmd.args.size() == 3) && onOff(cmd.args.back(), on)) {
        if (cmd.args.size() == 3 && lower(cmd.args[1]) == "all") {
            b->setDay(on);
            return std::string("ok every resident's day table is ") + (on ? "on (where the level states one)" : "off");
        }
        const int who = cmd.args.size() == 3 ? b->residentNamed(cmd.args[1]) : followed_;
        if (who < 0) return noSuch;
        b->setDay(who, on);
        return "ok " + b->residentName(who) + "'s day table is " + (b->dayOn(who) ? "on" : on ? "off: the level states no day for them" : "off");
    }
    if (what == "follow" && cmd.args.size() == 1) {
        // the next resident, round and round
        followed_ = (followed_ + 1) % b->residentCount();
        followAsked_ = true;
        return "ok following " + b->residentName(followed_) + " (resident " + std::to_string(followed_) + " of " +
               std::to_string(b->residentCount()) + ")";
    }
    if (what == "follow" && cmd.args.size() == 2 && lower(cmd.args[1]) == "off") {
        spectate_.letGo();
        return "ok the camera is let go (`camera <x> <y> <z> <pitch> <yaw>` places it; `brain follow <who>` takes it back)";
    }
    if (what == "follow" && cmd.args.size() == 2) {
        const int who = b->residentNamed(cmd.args[1]);
        if (who < 0) return noSuch;
        followed_ = who;
        followAsked_ = true;
        return "ok following " + b->residentName(who);
    }
    return usage;
}

void CityBrainSystem::update(engine::FrameContext& ctx) {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b) {
        camOnce_ = false;
        followed_ = 0;
        return;
    }
    if (b->hasResident() && followed_ >= b->residentCount()) followed_ = 0;
    // the camera on the followed resident: once when the level comes up, and when asked
    if (b->hasResident() && (!camOnce_ || followAsked_)) {
        spectate_.follow(b->residentAgent(followed_));
        camOnce_ = true;
        followAsked_ = false;
    }
    draw(ctx);
}

void CityBrainSystem::draw(engine::FrameContext& ctx) {
    const CitySim& sim = city_.sim();
    const CityBrain& b = *sim.brain();
    const auto on = [&](Vec2 p) { return Vec3(p.x, city_.surfaceY(p) + kLift, p.y); };
    // THE MARKERS: a ring round the spot where a person at the marker stands, and the post beside the ring (city_brain.h
    // says where and why: a post at the spot itself was drawn through the person)
    const int wanted = b.hasResident() ? b.wantedMarker(followed_) : -1;
    for (std::size_t i = 0; i < b.markers().size(); ++i) {
        const Vec3 colour = static_cast<int>(i) == wanted ? kWantedColour : kMarkerColour;
        const Vec3 foot = on(b.markers()[i].post);
        ctx.debug.circle(on(b.markers()[i].stand), Vec3(0, 1, 0), kMarkerRingRadius, colour);
        ctx.debug.line(foot, foot + Vec3(0, kPostHeight, 0), colour);
        ctx.debug.circle(foot + Vec3(0, kPostHeight, 0), Vec3(0, 1, 0), kMarkerRingRadius * 0.5, colour);
    }
    if (!b.hasResident()) return;

    // THE FOLLOWED RESIDENT'S PLAN, each leg in its way's colour: along its route once it has one, and until then a
    // straight dashed line
    const int who = b.residentAgent(followed_);
    const Agent& a = sim.agents()[static_cast<std::size_t>(who)];
    const bool onFoot = !b.inCar(followed_);
    const Vec2 at = a.pos;   // (in a car: the car's)
    const std::vector<BrainPlanLeg> legs = b.planLegs(followed_);
    for (const BrainPlanLeg& leg : legs) {
        if (leg.done) continue;
        std::vector<Vec3> pts;
        if (leg.underWay && onFoot && a.moving && a.route.found) {
            // on foot: the pavements they will take, then straight to where the walk ends
            pts.push_back(on(a.pos));
            sim.walkPoses(who, kLineStep, [&](const Agent& c) { pts.push_back(on(c.pos)); });
            pts.push_back(on(leg.to));
        } else if (leg.underWay && onFoot) {
            // the last steps off the pavements, walked straight
            pts.push_back(on(a.pos));
            pts.push_back(on(leg.to));
        } else if (leg.underWay && b.residentCar(followed_) >= 0) {
            // at the wheel, once the driving has a route for the trip: its lanes and the turns between them, from the
            // car's front bumper on (so the line does not lie over the car)
            const driving::Driving* d = sim.drivingV2();
            const driving::CarId car(static_cast<uint32_t>(b.residentCar(followed_)));
            const driving::Trip* trip = d ? d->trip(d->tripOf(car)) : nullptr;
            const driving::Car* placed = d ? d->car(car) : nullptr;
            if (trip && placed && d->roads()) {
                const engine::LaneGraph& roads = *d->roads();
                std::vector<Vec2> line;
                const auto along = [&](engine::PathId path, Real from, Real to) {
                    if (path.index >= roads.paths.size()) return;
                    for (Real s = from; s < to; s += kLineStep) line.push_back(roads.pointAt(path, s));
                    line.push_back(roads.pointAt(path, to));
                };
                std::size_t nearCar = 0;   // how many of `line` are near enough the start to be where the car is
                for (std::size_t k = trip->legReached; k < trip->route.legs.size(); ++k) {
                    const driving::RouteLeg& rl = trip->route.legs[k];
                    if (rl.lane.index >= roads.paths.size()) continue;
                    const Real length = roads.paths[rl.lane.index].length;
                    along(rl.lane, rl.enterFraction * length, rl.leaveFraction * length);
                    if (k < trip->route.turns.size()) {
                        const engine::PathId turn = trip->route.turns[k].connector;
                        if (turn.index < roads.paths.size()) along(turn, 0, roads.paths[turn.index].length);
                    }
                    if (k < trip->legReached + kLegsLookedAtForTheCar) nearCar = line.size();
                }
                const Vec2 front = roads.pointAt(placed->path, placed->along);
                std::size_t from = 0;
                for (std::size_t i = 1; i < nearCar; ++i)
                    if ((line[i] - front).lengthSquared() < (line[from] - front).lengthSquared()) from = i;
                pts.push_back(on(front));
                for (std::size_t i = from + 1; i < line.size(); ++i) pts.push_back(on(line[i]));
                pts.push_back(on(leg.to));
            }
        }
        if (pts.empty()) {
            // no route yet (a leg to come, or a car that has not left its space): straight, and dashed
            pieces(ctx.debug, on(leg.underWay ? at : leg.from), on(leg.to), leg.colour, true);
            continue;
        }
        for (std::size_t i = 1; i < pts.size(); ++i) pieces(ctx.debug, pts[i - 1], pts[i], leg.colour, false);
    }
}

void CityBrainSystem::lookedAt(const engine::FrameContext& ctx, const CityBrain& b, int& who, int& car) const {
    who = -1;
    car = -1;
    const CitySim& sim = city_.sim();
    const engine::CameraState& cam = ctx.view.camera;
    const Vec3 toward = cam.target - cam.position;
    const Real reach = toward.length();
    if (!(reach > 0)) return;
    const Vec3 way = toward * (1.0 / reach);
    // how far `p` is from the line through the middle of the view (past kLookRange, or behind the camera: not on it)
    const auto offLine = [&](Vec2 p, Real& out) {
        const Vec3 at(p.x, city_.surfaceY(p) + kLookHeight, p.y);
        const Real along = engine::dot(at - cam.position, way);
        if (along < 0 || along > kLookRange) return false;
        out = (at - (cam.position + way * along)).length();
        return true;
    };
    Real best = kLookWithin;
    for (int r = 0; r < b.residentCount(); ++r) {
        const Agent& a = sim.agents()[static_cast<std::size_t>(b.residentAgent(r))];
        Real off = 0;
        // (a person at the wheel is where their car is: the car is what is looked at)
        if (b.inCar(r) || !offLine(a.pos, off) || off > best) continue;
        best = off;
        who = r;
        car = -1;
    }
    for (const int v : b.cars()) {
        Vec2 pos, heading;
        Real off = 0;
        // (the car's own pose: its entity; a car the player has as their own is not here to look at)
        if (!b.carPose(v, pos, heading) || !offLine(pos, off) || off > best) continue;
        best = off;
        who = -1;
        car = v;
    }
    if (who >= 0 || car >= 0) return;
    // nothing on the line: the resident nearest the camera, when one is within a few metres
    Real nearest = kLookNear;
    for (int r = 0; r < b.residentCount(); ++r) {
        const Agent& a = sim.agents()[static_cast<std::size_t>(b.residentAgent(r))];
        const Real away = (Vec3(a.pos.x, city_.surfaceY(a.pos) + kLookHeight, a.pos.y) - cam.position).length();
        if (b.inCar(r) || away > nearest) continue;
        nearest = away;
        who = r;
    }
}

// THE NAMES, the followed resident in a few lines, and whoever is looked at (the game UI layer).
void CityBrainSystem::render(engine::FrameContext& ctx) {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b || ctx.actions.layerUp()) return;
    const float W = static_cast<float>(ctx.framebufferWidth), H = static_cast<float>(ctx.framebufferHeight);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    const float white[4] = {1, 1, 1, 0.95f}, gold[4] = {1.0f, 0.82f, 0.3f, 1.0f}, back[4] = {0, 0, 0, 0.6f};
    const float pale[4] = {0.75f, 0.9f, 1.0f, 1.0f};
    std::vector<engine::Renderer::UiQuad> q;

    const engine::CameraState& cam = ctx.view.camera;
    const Mat4 vp = Mat4::perspective(cam.fovDegrees * engine::DEG_TO_RAD, cam.aspectRatio, cam.nearPlane, cam.farPlane) *
                    Mat4::lookAt(cam.position, cam.target, cam.up);
    for (const BrainMarker& m : b->markers()) {
        const Vec3 top(m.post.x, city_.surfaceY(m.post) + kPostHeight, m.post.y);
        if (!labels_ || (top - cam.position).length() > kLabelRange) continue;
        Vec2 px;
        if (!worldToScreen(vp, top, W, H, px)) continue;
        ui_.label(ctx.renderer, q, static_cast<float>(px.x), static_cast<float>(px.y) - 30.0f * k, m.name, 16.0f * k, gold, back, 5.0f * k);
    }
    // each resident's name above their head (on foot) or their car (at its wheel)
    for (int who = 0; labels_ && who < b->residentCount(); ++who) {
        const Agent& body = city_.sim().agents()[static_cast<std::size_t>(b->residentAgent(who))];
        if (body.indoors && body.inCar < 0) continue;   // (out of sight: indoors, or far on a leg, ADR-0493)
        const Vec2 at = body.pos;   // (at the wheel: the car's)
        const Vec3 head(at.x, city_.surfaceY(at) + kNameHeight, at.y);
        if ((head - cam.position).length() > kNameRange) continue;
        Vec2 px;
        if (!worldToScreen(vp, head, W, H, px)) continue;
        ui_.label(ctx.renderer, q, static_cast<float>(px.x), static_cast<float>(px.y) - 22.0f * k, b->residentName(who), 14.0f * k,
                  who == followed_ ? white : pale, back, 4.0f * k);
    }
    const float fs = 17.0f * k, gap = fs * 1.35f, pad = 10.0f * k, x0 = 16.0f * k, y0 = 16.0f * k;
    if (b->hasResident()) {
        std::vector<std::string> lines;
        const brain::AgentBrain& a = b->machine()->agent(static_cast<brain::AgentId>(followed_));
        const int wants = b->wantedMarker(followed_);
        const std::string want = wants >= 0 ? b->markers()[static_cast<std::size_t>(wants)].name : std::string("?");
        const std::string house = b->householdName(followed_);
        lines.push_back("Following: " + b->residentName(followed_) + (house.empty() ? std::string() : ", household " + house) +
                        "   (resident " + std::to_string(followed_ + 1) + " of " + std::to_string(b->residentCount()) +
                        "; brain follow, brain? " + b->residentName(followed_) + ")");
        lines.push_back("Wants to be at: " + want + "   (goto " + b->residentName(followed_) + " <marker>)");
        lines.push_back("Plan: " + (b->planText(followed_).empty() ? std::string("none") : b->planText(followed_)));
        // what they are doing, in the data's words (brain.lua `label`), and the marker they stand at, when at one
        const std::string at = b->standingAt(followed_);
        lines.push_back("Now: " + b->stateLabel(followed_) + (at.empty() ? std::string() : ", at " + at) +
                        (a.travelling() ? ", leg " + std::to_string(a.legNow + 1) + " of " + std::to_string(a.legCount) : std::string()) +
                        ", since " + fmt("%.0f s", static_cast<double>(b->now() - a.stateEntered) / 1000.0) + " ago" +
                        (b->dayOn(followed_) ? "   (day table on)" : ""));
        // (why there is no plan, when there is none and they want to be somewhere else)
        if (!a.travelling() && b->stateName(followed_) != b->machine()->table().states[b->machine()->table().intents[a.intentKind].arrive].name &&
            !b->planReason(followed_).empty())
            lines.push_back("Why: " + b->planReason(followed_));
        float wide = 0;
        for (const std::string& l : lines) wide = std::max(wide, ui_.measure(l, fs));
        engine::UiText::rect(q, x0, y0, x0 + wide + 2 * pad, y0 + gap * static_cast<float>(lines.size()) + pad, back);
        for (std::size_t i = 0; i < lines.size(); ++i)
            ui_.text(ctx.renderer, q, x0 + pad, y0 + pad + gap * static_cast<float>(i) + ui_.ascent(fs) * 0.8f, lines[i], fs,
                     i == 0 ? gold : white);
    }
    // WHOEVER IS LOOKED AT: their `brain?` line (or the car's own), a part to a row, at the foot of the screen
    int looked = -1, lookedCar = -1;
    if (b->hasResident()) lookedAt(ctx, *b, looked, lookedCar);
    if (looked >= 0 || lookedCar >= 0) {
        std::vector<std::string> lines = parts(looked >= 0 ? b->describe(looked) : b->describeCar(lookedCar));
        lines.insert(lines.begin(), looked >= 0 ? "Looking at: " + b->residentName(looked) : std::string("Looking at: a household's car"));
        const float small = 13.0f * k, row = small * 1.3f;
        float wide = 0;
        for (const std::string& l : lines) wide = std::max(wide, ui_.measure(l, small));
        const float top = H - y0 - row * static_cast<float>(lines.size()) - pad;
        engine::UiText::rect(q, x0, top, std::min(W - x0, x0 + wide + 2 * pad), H - y0, back);
        for (std::size_t i = 0; i < lines.size(); ++i)
            ui_.text(ctx.renderer, q, x0 + pad, top + pad * 0.5f + row * static_cast<float>(i) + ui_.ascent(small) * 0.8f, lines[i], small,
                     i == 0 ? gold : white);
    }
    if (!q.empty()) ctx.renderer.submitUi(q);
}

}  // namespace citysim
