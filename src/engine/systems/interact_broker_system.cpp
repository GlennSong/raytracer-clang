#include "interact_broker_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../input/input_map.h"
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif
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
    ctx.actions.bindButton("interact_cycle", GamepadButton::DpadRight);
    ctx.actions.setActionContext("interact_cycle", InputContext::OnFoot);
}

void InteractBrokerSystem::update(FrameContext& ctx) {
    show_ = marker_ = false;
    pulse_ += ctx.frameDelta;
    // The control socket's one-shots (`interact`, `interact hold`, `interact stand`, `interact next`).
    const double req = ctx.settings.getDouble("interact.request", 0.0);
    if (req > 0) ctx.settings.setDouble("interact.request", 0.0);

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
        ctx.settings.setString("interact.status", "nothing in reach");
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
        ctx.settings.setString("interact.status", "nothing in reach (" + std::to_string(offers.size()) +
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
    name_ = f.name;
    others_ = static_cast<int>(ranked.size()) - 1;
    prompt_.clear();
    if (!f.tap.empty()) prompt_ = "E  " + f.tap;
    if (!f.hold.empty()) prompt_ += (prompt_.empty() ? "" : "      ") + std::string("hold E  ") + f.hold;
    {
        char at[96];
        std::snprintf(at, sizeof at, "  at %.2f %.2f %.2f", f.anchor.x, f.anchor.y, f.anchor.z);
        ctx.settings.setString("interact.status", "focus [" + f.provider + ":" + f.name + "] " + prompt_ +
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
#ifdef RT_ENABLE_IMGUI
    if (!show_) return;
    const float W = static_cast<float>(ctx.windowWidth), H = static_cast<float>(ctx.windowHeight);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    // THE MARKER: a ring on the thing, softly pulsing, the prompt beside it.
    float px = W * 0.5f, py = H - 120.0f;
    if (marker_) {
        Real sx = 0, sy = 0;
        const CameraState& cam = ctx.view.camera;
        if (projectToScreen(anchor_, cam.position, cam.target, cam.up, cam.fovDegrees, cam.aspectRatio, W, H, sx, sy)) {
            const float r = 9.0f + 2.0f * static_cast<float>(std::sin(pulse_ * 4.0));
            dl->AddCircle(ImVec2(static_cast<float>(sx), static_cast<float>(sy)), r + 2.0f, IM_COL32(0, 0, 0, 140), 24, 4.0f);
            dl->AddCircle(ImVec2(static_cast<float>(sx), static_cast<float>(sy)), r, IM_COL32(255, 236, 160, 235), 24, 2.5f);
            dl->AddCircleFilled(ImVec2(static_cast<float>(sx), static_cast<float>(sy)), 2.5f, IM_COL32(255, 236, 160, 235));
            px = static_cast<float>(sx) + 18.0f;
            py = static_cast<float>(sy) - 10.0f;
        }
    }
    std::string nm = name_;
    for (char& c : nm) if (c == '_') c = ' ';
    std::string text = nm.empty() ? prompt_ : nm + "\n" + prompt_;
    if (others_ > 0) text += "\n+" + std::to_string(others_) + " nearby   Tab / wheel";
    ImFont* font = ImGui::GetFont();
    const float fs = ImGui::GetFontSize() * 1.4f;   // readable at a glance, over a busy room
    const ImVec2 ts = font->CalcTextSizeA(fs, 1e9f, 0.0f, text.c_str());
    if (!marker_) px -= ts.x * 0.5f;
    px = std::min(px, W - ts.x - 12.0f);
    py = std::min(py, H - ts.y - 12.0f);
    dl->AddRectFilled(ImVec2(px - 6, py - 4), ImVec2(px + ts.x + 6, py + ts.y + 4), IM_COL32(0, 0, 0, 150), 4.0f);
    dl->AddText(font, fs, ImVec2(px, py), IM_COL32(255, 255, 255, 240), text.c_str());
#else
    (void)ctx;
#endif
}

}  // namespace engine
