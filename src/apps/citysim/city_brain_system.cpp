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
constexpr Real kRingRadius = 1.6;        // ...the ring at its foot (m)
constexpr Real kLift = 0.4;              // lines drawn this far above the surface they follow (m)
constexpr Real kLineStep = 2.0;          // a point of a drawn leg every this many metres
constexpr Real kLabelRange = 450.0;      // a marker's name is drawn within this of the camera (m)
const Vec3 kMarkerColour(1.00, 0.78, 0.16);   // the waypoint's gold (the trip lab's destination post)
const Vec3 kWantedColour(1.00, 1.00, 1.00);   // the marker the resident wants to be at

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
    ctx.commands->add("brain", "brain trait <name> on|off | day on|off | follow",
                      "Give or take one of the resident's traits (travel.lua: has_board, has_scooter), used from their "
                      "next plan; switch their day table; put the camera on them.",
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
    const std::string usage = "err usage: brain trait <name> on|off | day on|off | follow";
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
        const Vec3 colour = static_cast<int>(i) == wanted ? kWantedColour : kMarkerColour;
        const Vec3 foot = on(b.markers()[i].stand);
        ctx.debug.line(foot, foot + Vec3(0, kPostHeight, 0), colour);
        ctx.debug.circle(foot, Vec3(0, 1, 0), kRingRadius, colour);
        ctx.debug.circle(foot + Vec3(0, kPostHeight, 0), Vec3(0, 1, 0), kRingRadius * 0.5, colour);
    }
    if (!b.hasResident()) return;

    // THE PLAN, each leg in its way's colour
    const int who = b.residentAgent();
    const Agent& a = sim.agents()[static_cast<std::size_t>(who)];
    const std::vector<BrainPlanLeg> legs = b.planLegs();
    for (const BrainPlanLeg& leg : legs) {
        if (leg.done) continue;
        std::vector<Vec3> pts;
        if (leg.underWay && a.mode == Agent::Mode::Pedestrian) {
            // on foot: the pavements they will take, then straight to where the walk ends
            pts.push_back(on(a.pos));
            if (a.moving && a.route.valid())
                sim.walkPoses(who, kLineStep, [&](const Agent& c) { pts.push_back(on(c.pos)); });
            pts.push_back(on(leg.to));
        } else if (leg.underWay) {
            // at the wheel: the lanes of the route still ahead
            pts.push_back(on(a.pos));
            const driving::Driving* d = sim.drivingV2();
            const driving::CarId car(static_cast<uint32_t>(b.residentVehicle()));
            const driving::Trip* trip = d ? d->trip(d->tripOf(car)) : nullptr;
            if (trip && d->roads()) {
                const engine::LaneGraph& roads = *d->roads();
                for (std::size_t k = trip->legReached; k < trip->route.legs.size(); ++k) {
                    const driving::RouteLeg& rl = trip->route.legs[k];
                    if (rl.lane.index >= roads.paths.size()) continue;
                    const Real length = roads.paths[rl.lane.index].length;
                    Real from = rl.enterFraction * length;
                    const Real to = rl.leaveFraction * length;
                    const driving::Car* placed = d->car(car);
                    if (placed && placed->path == rl.lane) from = std::max(from, placed->along);
                    for (Real s = from; s < to; s += kLineStep) pts.push_back(on(roads.pointAt(rl.lane, s)));
                    pts.push_back(on(roads.pointAt(rl.lane, to)));
                }
            }
            pts.push_back(on(leg.to));
        } else {
            pts.push_back(on(leg.from));
            pts.push_back(on(leg.to));
        }
        for (std::size_t i = 1; i < pts.size(); ++i) ctx.debug.line(pts[i - 1], pts[i], leg.colour);
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
        const Vec3 top(m.stand.x, city_.surfaceY(-1, 0, m.stand) + kPostHeight, m.stand.y);
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
        lines.push_back("Now: " + b->stateName() + (a.travelling() ? ", leg " + std::to_string(a.legNow + 1) + " of " + std::to_string(a.legCount) : std::string()) +
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
