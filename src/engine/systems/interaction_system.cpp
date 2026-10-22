#include "interaction_system.h"
#include "../components.h"
#include "../interact_broker.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/furniture_library_lua.h"
#endif
#include <cmath>
#include <cstdio>

namespace engine {

void InteractionSystem::onStart(FrameContext& ctx) {
    (void)ctx;
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
    const Vec3 feet = pos - Vec3(0, feetDrop, 0);

    InteractCommand cmd;
    const bool commanded = takeInteractCommand(ctx.world, player, "furniture", cmd);

    // SEATED: stand up on the command, on any move key, or when the furniture streamed away under them.
    if (const Seated* s = ctx.world.get<Seated>(player)) {
        const bool gone = !s->set.valid() || !ctx.world.alive(s->set) || !ctx.world.has<Interactables>(s->set);
        const bool move = std::fabs(ctx.actions.axis("cam_forward")) > 0.5 || std::fabs(ctx.actions.axis("cam_right")) > 0.5;
        if (gone || move || commanded) { standUp(ctx, player); return; }
        const FurnitureAsset* fa = lib.find(static_cast<Piece>(ctx.world.get<Interactables>(s->set)->pieces[s->piece].piece));
        InteractOffer o;
        o.provider = "furniture";
        o.entity = s->set;
        o.key = s->piece;
        o.anchor = s->eye;
        const Verb vb = fa ? fa->verbs[s->verb].verb : Verb::Sit;
        o.tap = vb == Verb::Stand ? "step away" : "stand up";
        o.name = fa ? std::string(vb == Verb::Lie ? "lying on the " : vb == Verb::Stand ? "standing at the " : "sitting on the ") +
                          furniturePieceName(fa->piece)
                    : "seated";
        o.exclusive = true;
        offerInteraction(ctx.world, player, o);
        return;
    }

    // THE COMMAND: sit or lie on the piece the broker focused (its verbs worked out again from here).
    if (commanded && cmd.entity.valid() && ctx.world.alive(cmd.entity)) {
        if (Interactables* set = ctx.world.get<Interactables>(cmd.entity)) {
            int prim = -1, sec = -1;
            Vec3 at;
            Real d = 0;
            if (pieceVerbs(*set, static_cast<uint32_t>(cmd.key), lib, feet, REACH + 0.5, prim, sec, at, d)) {
                const int vi = cmd.hold && sec >= 0 ? sec : (prim >= 0 ? prim : sec);
                Seated seat;
                if (takeInteraction(*set, cmd.entity, static_cast<uint32_t>(cmd.key), vi, lib, seat))
                    ctx.world.add<Seated>(player, seat);
                return;
            }
        }
    }

    // THE OFFERS: every piece in reach with a free verb.
    std::vector<std::pair<Entity, const Interactables*>> sets;
    ctx.world.each<Interactables>([&](Entity e, Interactables& s) { sets.push_back({e, &s}); });
    for (const auto& [ent, set] : sets) {
        const Real pad = REACH + 2.5;
        if (feet.x < set->lo.x - pad || feet.x > set->hi.x + pad || feet.z < set->lo.z - pad ||
            feet.z > set->hi.z + pad || feet.y < set->lo.y - 3.0 || feet.y > set->hi.y + 3.0)
            continue;
        for (std::size_t pi = 0; pi < set->pieces.size(); ++pi) {
            int prim = -1, sec = -1;
            Vec3 at;
            Real d = 0;
            if (!pieceVerbs(*set, static_cast<uint32_t>(pi), lib, feet, REACH, prim, sec, at, d)) continue;
            const FurnitureAsset* a = lib.find(static_cast<Piece>(set->pieces[pi].piece));
            auto label = [&](int vi) {
                if (vi < 0) return std::string();
                const FurnVerb& v = a->verbs[static_cast<std::size_t>(vi)];
                return v.label.empty() ? std::string(verbName(v.verb)) : v.label;
            };
            InteractOffer o;
            o.provider = "furniture";
            o.entity = ent;
            o.key = pi;
            o.anchor = at + Vec3(0, 0.15, 0);
            o.tap = label(prim >= 0 ? prim : sec);
            o.hold = prim >= 0 ? label(sec) : std::string();
            o.name = furniturePieceName(a->piece);
            o.reach = REACH;
            offerInteraction(ctx.world, player, o);
        }
    }
}

}  // namespace engine
