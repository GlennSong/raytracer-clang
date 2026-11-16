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
const Vec3 kMarkerColour(1.00, 0.78, 0.16);   // the waypoint's gold (the trip lab's destination post)
const Vec3 kWantedColour(1.00, 1.00, 1.00);   // the marker the resident wants to be at

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
}  // namespace

void CityBrainSystem::onStart(engine::FrameContext& ctx) {
    if (!ctx.commands) return;
    ctx.commands->add("brain?", "brain?",
                      "The resident with the agent brain in one line: what they want, the plan, the leg under way, the "
                      "state and since when, what they wait for, where they and their car are.",
                      [this](const engine::ControlCommand&) {
                          const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
                          return b ? "ok " + b->describe() : std::string("err this level states no markers and no resident (citysim.markers, citysim.resident)");
                      },
                      kOwner);
    ctx.commands->add("markers?", "markers?",
                      "Every marker of the level: its name, street, station, kerb and where a person stands at it.",
                      [this](const engine::ControlCommand&) { return markersLine(); }, kOwner);
    ctx.commands->add("goto", "goto <marker>",
                      "The resident with the agent brain wants to be at that marker from now: they plan the trip and "
                      "make it. Stops their day table (brain day on resumes it). Poll brain?.",
                      [this](const engine::ControlCommand& c) { return gotoVerb(c); }, kOwner);
    ctx.commands->add("brain", "brain trait <name> on|off | day on|off | follow [off]",
                      "Give or take one of the resident's traits (travel.lua: has_board, has_scooter), used from their "
                      "next plan; switch their day table; put the camera on them, or let it go (then `camera` places it).",
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

std::string CityBrainSystem::gotoVerb(const engine::ControlCommand& cmd) {
    CityBrain* b = city_.built() ? city_.simMutable().brain() : nullptr;
    if (!b) return "err this level states no resident (citysim.resident)";
    if (cmd.args.size() != 1) return "err usage: goto <marker>";
    std::string why;
    if (!b->goTo(cmd.args[0], &why)) return "err " + why;
    return "ok the resident wants to be at " + cmd.args[0] + " (poll brain?)";
}

std::string CityBrainSystem::brainVerb(const engine::ControlCommand& cmd) {
    const std::string usage = "err usage: brain trait <name> on|off | day on|off | follow [off]";
    CityBrain* b = city_.built() ? city_.simMutable().brain() : nullptr;
    if (!b || !b->hasResident()) return "err this level states no resident (citysim.resident)";
    if (cmd.args.empty()) return usage;
    const std::string what = lower(cmd.args[0]);
    const auto onOff = [&](const std::string& word, bool& on) {
        const std::string w = lower(word);
        on = w == "on";
        return w == "on" || w == "off";
    };
    bool on = false;
    if (what == "trait" && cmd.args.size() == 3 && onOff(cmd.args[2], on)) {
        std::string why;
        if (!b->setTrait(cmd.args[1], on, &why)) return "err " + why;
        return std::string("ok the resident ") + (on ? "has " : "no longer has ") + cmd.args[1] + " (used from their next plan)";
    }
    if (what == "day" && cmd.args.size() == 2 && onOff(cmd.args[1], on)) {
        b->setDay(on);
        return std::string("ok the day table is ") + (b->dayOn() ? "on" : on ? "off: the level states no day" : "off");
    }
    if (what == "follow" && cmd.args.size() == 1) {
        followAsked_ = true;
        return "ok following the resident";
    }
    if (what == "follow" && cmd.args.size() == 2 && lower(cmd.args[1]) == "off") {
        spectate_.letGo();
        return "ok the camera is let go (`camera <x> <y> <z> <pitch> <yaw>` places it; `brain follow` takes it back)";
    }
    return usage;
}

void CityBrainSystem::update(engine::FrameContext& ctx) {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b) {
        followed_ = false;
        return;
    }
    // the camera on the person: once when the level comes up, and when asked
    if (b->hasResident() && (!followed_ || followAsked_)) {
        spectate_.follow(b->residentAgent());
        followed_ = true;
        followAsked_ = false;
    }
    draw(ctx);
}

void CityBrainSystem::draw(engine::FrameContext& ctx) {
    const CitySim& sim = city_.sim();
    const CityBrain& b = *sim.brain();
    const auto on = [&](Vec2 p) { return Vec3(p.x, city_.surfaceY(-1, 0, p) + kLift, p.y); };
    // THE MARKERS: a post, a ring at its foot and a small one at its top
    int wanted = -1;
    if (b.hasResident()) {
        const uint32_t target = b.machine()->agent(0).intentTarget;
        if (target < b.markers().size()) wanted = static_cast<int>(target);
    }
    for (std::size_t i = 0; i < b.markers().size(); ++i) {
        // a ring round the spot where a person at the marker stands, and the post beside the ring (city_brain.h says
        // where and why: a post at the spot itself was drawn through the person)
        const Vec3 colour = static_cast<int>(i) == wanted ? kWantedColour : kMarkerColour;
        const Vec3 foot = on(b.markers()[i].post);
        ctx.debug.circle(on(b.markers()[i].stand), Vec3(0, 1, 0), kMarkerRingRadius, colour);
        ctx.debug.line(foot, foot + Vec3(0, kPostHeight, 0), colour);
        ctx.debug.circle(foot + Vec3(0, kPostHeight, 0), Vec3(0, 1, 0), kMarkerRingRadius * 0.5, colour);
    }
    if (!b.hasResident()) return;

    // THE PLAN, each leg in its way's colour: along its route once it has one, and until then a straight dashed line
    const int who = b.residentAgent();
    const Agent& a = sim.agents()[static_cast<std::size_t>(who)];
    const std::vector<BrainPlanLeg> legs = b.planLegs();
    for (const BrainPlanLeg& leg : legs) {
        if (leg.done) continue;
        std::vector<Vec3> pts;
        if (leg.underWay && a.mode == Agent::Mode::Pedestrian && a.moving && a.route.valid()) {
            // on foot: the pavements they will take, then straight to where the walk ends
            pts.push_back(on(a.pos));
            sim.walkPoses(who, kLineStep, [&](const Agent& c) { pts.push_back(on(c.pos)); });
            pts.push_back(on(leg.to));
        } else if (leg.underWay && a.mode == Agent::Mode::Pedestrian) {
            // the last steps off the pavements, walked straight
            pts.push_back(on(a.pos));
            pts.push_back(on(leg.to));
        } else if (leg.underWay) {
            // at the wheel, once the driving has a route for the trip: its lanes and the turns between them, from the
            // car's front bumper on (so the line does not lie over the car)
            const driving::Driving* d = sim.drivingV2();
            const driving::CarId car(static_cast<uint32_t>(b.residentVehicle()));
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
            pieces(ctx.debug, on(leg.underWay ? a.pos : leg.from), on(leg.to), leg.colour, true);
            continue;
        }
        for (std::size_t i = 1; i < pts.size(); ++i) pieces(ctx.debug, pts[i - 1], pts[i], leg.colour, false);
    }
}

// THE NAMES, and the resident in a few lines (the game UI layer).
void CityBrainSystem::render(engine::FrameContext& ctx) {
    const CityBrain* b = city_.built() ? city_.sim().brain() : nullptr;
    if (!b || ctx.actions.layerUp()) return;
    const float W = static_cast<float>(ctx.framebufferWidth), H = static_cast<float>(ctx.framebufferHeight);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    const float white[4] = {1, 1, 1, 0.95f}, gold[4] = {1.0f, 0.82f, 0.3f, 1.0f}, back[4] = {0, 0, 0, 0.6f};
    std::vector<engine::Renderer::UiQuad> q;

    const engine::CameraState& cam = ctx.view.camera;
    const Mat4 vp = Mat4::perspective(cam.fovDegrees * engine::DEG_TO_RAD, cam.aspectRatio, cam.nearPlane, cam.farPlane) *
                    Mat4::lookAt(cam.position, cam.target, cam.up);
    for (const BrainMarker& m : b->markers()) {
        const Vec3 top(m.post.x, city_.surfaceY(-1, 0, m.post) + kPostHeight, m.post.y);
        if ((top - cam.position).length() > kLabelRange) continue;
        Vec2 px;
        if (!worldToScreen(vp, top, W, H, px)) continue;
        ui_.label(ctx.renderer, q, static_cast<float>(px.x), static_cast<float>(px.y) - 30.0f * k, m.name, 16.0f * k, gold, back, 5.0f * k);
    }
    if (b->hasResident()) {
        std::vector<std::string> lines;
        const brain::AgentBrain& a = b->machine()->agent(0);
        const std::string want = a.intentTarget < b->markers().size() ? b->markers()[a.intentTarget].name : std::string("?");
        lines.push_back("Resident wants to be at: " + want + "   (goto <marker>, brain?)");
        lines.push_back("Plan: " + (b->planText().empty() ? std::string("none") : b->planText()));
        // what they are doing, in the data's words (brain.lua `label`), and the marker they stand at, when at one
        const std::string at = b->standingAt();
        lines.push_back("Now: " + b->stateLabel() + (at.empty() ? std::string() : ", at " + at) +
                        (a.travelling() ? ", leg " + std::to_string(a.legNow + 1) + " of " + std::to_string(a.legCount) : std::string()) +
                        ", since " + fmt("%.0f s", static_cast<double>(b->now() - a.stateEntered) / 1000.0) + " ago" +
                        (b->dayOn() ? "   (day table on)" : ""));
        const float fs = 17.0f * k, gap = fs * 1.35f, pad = 10.0f * k, x0 = 16.0f * k, y0 = 16.0f * k;
        float wide = 0;
        for (const std::string& l : lines) wide = std::max(wide, ui_.measure(l, fs));
        engine::UiText::rect(q, x0, y0, x0 + wide + 2 * pad, y0 + gap * static_cast<float>(lines.size()) + pad, back);
        for (std::size_t i = 0; i < lines.size(); ++i)
            ui_.text(ctx.renderer, q, x0 + pad, y0 + pad + gap * static_cast<float>(i) + ui_.ascent(fs) * 0.8f, lines[i], fs,
                     i == 0 ? gold : white);
    }
    if (!q.empty()) ctx.renderer.submitUi(q);
}

}  // namespace citysim
