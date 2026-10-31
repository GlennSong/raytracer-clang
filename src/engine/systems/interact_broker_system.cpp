#include "interact_broker_system.h"

#include <cstdlib>
#include "physics_system.h"
#include "../components.h"
#include "../../log.h"
#include "../input/input_map.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

namespace {

struct SightCtx { PhysicsSystem* phys; };

bool sightBlocked(const Vec3& eye, const Vec3& anchor, void* c) {
    PhysicsSystem* phys = static_cast<SightCtx*>(c)->phys;
    if (!phys) return false;
    const Vec3 d = anchor - eye;
    const Real len = d.length();
    // Only the MIDDLE of the sight line is tested: its first 0.6 m runs through the player's own capsule (the eye
    // sits at its top, and a steep look down stays inside its radius for a while), its last 0.6 m through the
    // thing's own collider (a chair's box round its seat). Within arm's length nothing can be in the way.
    if (len < 1.4) return false;
    const Vec3 dir = d * (1.0 / len);
    Vec3 hit;
    return phys->physicsWorld().castRay(eye + dir * 0.6, dir * (len - 1.2), hit);
}

}  // namespace

void InteractBrokerSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("interact", KeyCode::E);
    ctx.actions.bindButton("interact", GamepadButton::X);
    ctx.actions.setActionContext("interact", InputContext::OnFoot);
    ctx.actions.bindButton("interact_cycle", KeyCode::Tab);
    ctx.actions.bindButton("interact_cycle", GamepadButton::Y);
    ctx.actions.setActionContext("interact_cycle", InputContext::OnFoot);
    onRequest_ = ctx.events.listen<InteractRequest>([this](const InteractRequest& r) { pendingRequest_ = r.action; });
    answers_.bind(ctx.commands, "InteractBrokerSystem", "interact?");
}

void InteractBrokerSystem::update(FrameContext& ctx) {
    show_ = marker_ = false;
    pulse_ += ctx.frameDelta;
    // An InteractRequest's one-shot (`interact`, `interact hold`, `interact stand`, `interact next`).
    const double req = pendingRequest_;
    pendingRequest_ = 0;

    Entity player;
    Vec3 pos;
    Real feetDrop = 0.7;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (player.valid()) return;
        player = e;
        pos = t.position;
        if (const CharacterController* cc = ctx.world.get<CharacterController>(e)) feetDrop = cc->halfHeight + cc->radius;
    });
    if (!player.valid()) return;
    std::vector<InteractOffer> offers;
    if (InteractOffers* o = ctx.world.get<InteractOffers>(player)) { offers.swap(o->list); }
    if (ctx.world.has<InVehicle>(player) || offers.empty()) {
        haveFocus_ = false;
        holding_ = false;
        answers_.set("interact?", "nothing in reach");
        return;
    }

    const Vec3 eye = ctx.view.camera.position;
    const Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    SightCtx sc{physics_};
    const std::vector<RankedOffer> ranked =
        rankInteractions(offers, pos - Vec3(0, feetDrop, 0), eye, fwd, &sightBlocked, &sc);
    if (ranked.empty()) {
        haveFocus_ = false;
        holding_ = false;
        // why not, for the socket: the first offer's numbers
        const InteractOffer& o0 = offers[0];
        const Vec3 feet = pos - Vec3(0, feetDrop, 0);
        Vec3 fw = fwd;
        const Real fl = fw.length();
        if (fl > 1e-6) fw = fw * (1.0 / fl);
        const Vec3 toA = o0.anchor - eye;
        const Real look = toA.length() > 1e-6 ? (toA.x * fw.x + toA.y * fw.y + toA.z * fw.z) / toA.length() : 1.0;
        char why[256];
        std::snprintf(why, sizeof why, " first %s:%s anchor %.2f %.2f %.2f feet %.2f %.2f %.2f reach %.1f look %.2f blocked %d",
                      o0.provider.c_str(), o0.name.c_str(), o0.anchor.x, o0.anchor.y, o0.anchor.z, feet.x, feet.y, feet.z,
                      o0.reach, look, sightBlocked(eye, o0.anchor, &sc) ? 1 : 0);
        answers_.set("interact?", "nothing in reach (" + std::to_string(offers.size()) +
                                                      " offered, none in view)" + why);
        return;
    }

    // THE FOCUS: the best, unless the current one is still a candidate and nothing is clearly better (or it was
    // picked by hand).
    auto same = [&](const InteractOffer& o) {
        return haveFocus_ && o.provider == fProvider_ && o.entity == fEntity_ && o.key == fKey_;
    };
    std::size_t fi = 0;
    for (std::size_t r = 0; r < ranked.size(); ++r)
        if (same(offers[ranked[r].index]) && (cycled_ || ranked[r].score < ranked[0].score + 0.6)) { fi = r; break; }
    if (fi == 0 && !(haveFocus_ && same(offers[ranked[0].index]))) cycled_ = false;
    // TAB / the wheel (first person): the next candidate.
    const bool thirdPerson = ctx.settings.getBool("playerThirdPerson", false);
    int step = 0;
    if (ctx.actions.pressed("interact_cycle") || req == 4.0) step = 1;
    if (!thirdPerson && ctx.input.scrollDelta != 0.0) step = ctx.input.scrollDelta < 0 ? 1 : -1;
    if (step != 0 && ranked.size() > 1) {
        fi = (fi + ranked.size() + static_cast<std::size_t>(step + static_cast<int>(ranked.size()))) % ranked.size();
        cycled_ = true;
    }
    const InteractOffer& f = offers[ranked[fi].index];
    haveFocus_ = true;
    fProvider_ = f.provider;
    fEntity_ = f.entity;
    fKey_ = f.key;

    // What the screen says.
    show_ = true;
    marker_ = !f.exclusive;
    anchor_ = f.anchor;
    carried_ = f.carried;
    local_ = f.local;
    player_ = player;
    name_ = f.name;
    others_ = static_cast<int>(ranked.size()) - 1;
    // THE BUTTON, on the device in use: E on the keyboard, X on a controller (InputMap::gamepadInUse).
    pad_ = ctx.actions.gamepadInUse();
    const std::string key = pad_ ? "(X)" : "E";
    prompt_.clear();
    if (!f.tap.empty()) prompt_ = key + "  " + f.tap;
    if (!f.hold.empty()) prompt_ += (prompt_.empty() ? "" : "      ") + std::string("hold ") + key + "  " + f.hold;
    {
        char at[96];
        std::snprintf(at, sizeof at, "  at %.2f %.2f %.2f", f.anchor.x, f.anchor.y, f.anchor.z);
        answers_.set("interact?", "focus [" + f.provider + ":" + f.name + "] " + prompt_ +
                                                      (others_ > 0 ? "  +" + std::to_string(others_) : "") + at);
    }

    // E: a tap, or a hold when the offer has a hold verb (then the hold fires as soon as it is long enough).
    auto send = [&](bool hold) {
        InteractCommand c;
        c.provider = f.provider;
        c.entity = f.entity;
        c.key = f.key;
        c.hold = hold;
        if (ctx.world.has<InteractCommand>(player)) ctx.world.remove<InteractCommand>(player);
        ctx.world.add<InteractCommand>(player, c);
        holding_ = false;
        holdFired_ = true;
    };
    if (req == 1.0 || req == 3.0) { send(f.tap.empty()); return; }
    if (req == 2.0) { send(!f.hold.empty()); return; }
    if (ctx.actions.pressed("interact")) { holding_ = true; holdT_ = 0; holdFired_ = false; }
    if (holding_ && ctx.actions.held("interact")) {
        holdT_ += ctx.frameDelta;
        if (!holdFired_ && holdT_ >= HOLD_S && !f.hold.empty()) send(true);
    }
    if (holding_ && ctx.actions.released("interact")) {
        holding_ = false;
        if (!holdFired_) send(f.tap.empty());
    }
}

void InteractBrokerSystem::render(FrameContext& ctx) {
    if (!show_ || ctx.actions.layerUp()) return;   // (a screen that has the controls is up: the map)
    // framebuffer pixels; sizes in window points times the display's scale
    const float W = static_cast<float>(ctx.framebufferWidth), H = static_cast<float>(ctx.framebufferHeight);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    std::vector<Renderer::UiQuad> q;
    // THE MARKER: a ring on the thing, softly pulsing, the prompt beside it.
    float px = W * 0.5f, py = H - 120.0f * k;
    if (marker_) {
        Real sx = 0, sy = 0;
        const CameraState& cam = ctx.view.camera;
        static const bool uncarried = std::getenv("RT_MARKER_UNCARRIED") != nullptr;   // A/B: the update-time anchor
        if (carried_ && !uncarried)
            if (const InteractCarrier* ic = player_.valid() ? ctx.world.get<InteractCarrier>(player_) : nullptr)
                anchor_ = ic->drawn.transformPoint(local_);
        if (projectToScreen(anchor_, cam.position, cam.target, cam.up, cam.fovDegrees, cam.aspectRatio, W, H, sx, sy)) {
            static const bool logMarker = std::getenv("RT_MARKER_LOG") != nullptr;   // the marker's screen point, each frame
            if (logMarker) LOG_INFO << "[marker] " << sx / k << " " << sy / k;
            const float r = (9.0f + 2.0f * static_cast<float>(std::sin(pulse_ * 4.0))) * k;
            const float shade[4] = {0, 0, 0, 0.55f}, gold[4] = {1.0f, 0.93f, 0.63f, 0.92f};
            const float cx = static_cast<float>(sx), cy = static_cast<float>(sy);
            ui_.ring(ctx.renderer, q, cx, cy, r + 4.0f * k, 4.0f * k, shade);
            ui_.ring(ctx.renderer, q, cx, cy, r + 1.25f * k, 2.5f * k, gold);
            ui_.disc(ctx.renderer, q, cx, cy, 2.5f * k, gold);
            px = cx + 18.0f * k;
            py = cy - 10.0f * k;
        }
    }
    std::string nm = name_;
    for (char& c : nm) if (c == '_') c = ' ';
    std::vector<std::string> lines;
    if (!nm.empty()) lines.push_back(nm);
    lines.push_back(prompt_);
    if (others_ > 0) lines.push_back("+" + std::to_string(others_) + (pad_ ? " nearby   (Y) next" : " nearby   Tab / wheel"));
    const float fs = 18.0f * k;   // readable at a glance, over a busy room
    const float lineH = (ui_.ascent(fs) + ui_.descent(fs)) * 1.05f;
    float w = 0;
    for (const std::string& l : lines) w = std::max(w, ui_.measure(l, fs));
    const float h = lineH * static_cast<float>(lines.size());
    if (!marker_) px -= w * 0.5f;
    px = std::min(px, W - w - 12.0f * k);
    py = std::min(py, H - h - 12.0f * k);
    const float back[4] = {0, 0, 0, 0.59f}, white[4] = {1, 1, 1, 0.94f};
    UiText::rect(q, px - 6.0f * k, py - 4.0f * k, px + w + 6.0f * k, py + h + 4.0f * k, back);
    for (std::size_t i = 0; i < lines.size(); ++i)
        ui_.text(ctx.renderer, q, px, py + lineH * static_cast<float>(i) + ui_.ascent(fs), lines[i], fs, white);
    ctx.renderer.submitUi(q);
}

}  // namespace engine
