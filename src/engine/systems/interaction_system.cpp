#include "interaction_system.h"
#include "../components.h"
#include "../input/input_map.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/furniture_library_lua.h"
#endif
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif
#include <cmath>
#include <cstdio>

namespace engine {

void InteractionSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("interact", KeyCode::E);
    ctx.actions.bindButton("interact", GamepadButton::X);
    ctx.actions.setActionContext("interact", InputContext::OnFoot);
#ifdef RT_ENABLE_SCRIPTING
    ensureFurnitureLibraryLoaded();
#endif
}

void InteractionSystem::standUp(FrameContext& ctx, Entity player) {
    const Seated* s = ctx.world.get<Seated>(player);
    if (!s) return;
    if (s->set.valid() && ctx.world.alive(s->set)) releaseInteraction(ctx.world.get<Interactables>(s->set), *s);
    ctx.world.remove<Seated>(player);   // structural: callers are outside any each()
}

void InteractionSystem::update(FrameContext& ctx) {
    // The control socket's one-shots (`interact`, `interact hold`, `interact stand`): 1 tap, 2 hold, 3 stand.
    const double req = ctx.settings.getDouble("interact.request", 0.0);
    if (req > 0) ctx.settings.setDouble("interact.request", 0.0);
    prompt_.clear();
    choice_ = InteractChoice{};
    Entity player;
    Vec3 pos;
    Real feetDrop = 0.7;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (player.valid()) return;
        player = e;
        pos = t.position;
        if (const CharacterController* cc = ctx.world.get<CharacterController>(e)) feetDrop = cc->halfHeight + cc->radius;
    });
    if (!player.valid() || ctx.world.has<InVehicle>(player) || ctx.world.has<Passenger>(player)) return;
    const FurnitureLibrary& lib = FurnitureLibrary::global();

    // SEATED: stand up on E or on any move key (or if the furniture streamed away under them).
    if (const Seated* s = ctx.world.get<Seated>(player)) {
        const bool gone = !s->set.valid() || !ctx.world.alive(s->set) || !ctx.world.has<Interactables>(s->set);
        const bool move = std::fabs(ctx.actions.axis("cam_forward")) > 0.5 || std::fabs(ctx.actions.axis("cam_right")) > 0.5;
        {
            const Vec3 e = s->eye;
            const FurnitureAsset* fa = s->set.valid() && ctx.world.alive(s->set) && ctx.world.has<Interactables>(s->set)
                                           ? lib.find(static_cast<Piece>(ctx.world.get<Interactables>(s->set)->pieces[s->piece].piece))
                                           : nullptr;
            char buf[192];
            std::snprintf(buf, sizeof buf, "seated %s on %s eye %.2f %.2f %.2f", fa ? verbName(fa->verbs[s->verb].verb) : "?",
                          fa ? furniturePieceName(fa->piece) : "?", e.x, e.y, e.z);
            ctx.settings.setString("interact.status", buf);
        }
        if (gone || move || ctx.actions.pressed("interact") || req > 0) {
            standUp(ctx, player);
            holding_ = false;
            holdFired_ = true;   // this press is spent
            return;
        }
        prompt_ = "E / move: stand up";
        return;
    }

    // ON FOOT: the best piece within reach, and its verbs.
    std::vector<std::pair<Entity, const Interactables*>> sets;
    ctx.world.each<Interactables>([&](Entity e, Interactables& s) { sets.push_back({e, &s}); });
    if (sets.empty()) { ctx.settings.setString("interact.status", "nothing in reach"); return; }
    const Vec3 feet = pos - Vec3(0, feetDrop, 0);
    Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    choice_ = findInteraction(sets, lib, feet, fwd);
    if (!choice_.valid()) { holding_ = false; ctx.settings.setString("interact.status", "nothing in reach"); return; }
    const FurnitureAsset* a = lib.find(static_cast<Piece>(choice_.setPtr->pieces[choice_.piece].piece));
    auto label = [&](int vi) {
        const FurnVerb& v = a->verbs[static_cast<std::size_t>(vi)];
        return v.label.empty() ? std::string(verbName(v.verb)) : v.label;
    };
    const int tapVerb = choice_.primary >= 0 ? choice_.primary : choice_.secondary;
    const int holdVerb = choice_.primary >= 0 ? choice_.secondary : -1;
    prompt_ = "E  " + label(tapVerb);
    if (holdVerb >= 0) prompt_ += "      hold E  " + label(holdVerb);
    ctx.settings.setString("interact.status", std::string("prompt [") + furniturePieceName(a->piece) + "] " + prompt_);

    auto act = [&](int vi) {
        Interactables* set = ctx.world.get<Interactables>(choice_.set);
        Seated s;
        if (set && takeInteraction(*set, choice_.set, choice_.piece, vi, lib, s)) ctx.world.add<Seated>(player, s);
    };
    if (req == 1.0) { act(tapVerb); return; }
    if (req == 2.0) { if (holdVerb >= 0) act(holdVerb); return; }
    if (ctx.actions.pressed("interact")) { holding_ = true; holdT_ = 0; holdFired_ = false; }
    if (holding_ && ctx.actions.held("interact")) {
        holdT_ += ctx.frameDelta;
        if (!holdFired_ && holdT_ >= HOLD_S && holdVerb >= 0) { holdFired_ = true; holding_ = false; act(holdVerb); }
    }
    if (holding_ && ctx.actions.released("interact")) {
        holding_ = false;
        if (!holdFired_) act(tapVerb);
    }
}

void InteractionSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (prompt_.empty()) return;
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ctx.windowWidth) * 0.5f, static_cast<float>(ctx.windowHeight) - 120.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("##interact", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
                     ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted(prompt_.c_str());
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
