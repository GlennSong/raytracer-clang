#include "prefab_use_system.h"

#include "../components.h"
#include "../interact_broker.h"
#include "../interaction.h"   // Seated
#include "../lights/light_board.h"
#include "../lights/light_kinds.h"
#include "../model_motion.h"
#include "../prefab_use.h"

#include <cmath>
#include <vector>

namespace engine {

namespace {

// Where someone sitting at a seat marker is: the eye above it, looking the way the marker faces, and the floor in
// front of it to stand up on.
bool seatAt(World& world, Entity instance, const std::string& marker, bool lying, Seated& out) {
    Vec3 at;
    Quat turn;
    if (!modelMarkerWorld(world, instance, marker, at, turn)) return false;
    Vec3 facing = turn.rotate(Vec3(0, 0, 1));
    facing.y = 0;
    facing = facing.lengthSquared() > 1e-8 ? normalize(facing) : Vec3(0, 0, 1);
    const Transform* place = world.get<Transform>(instance);
    if (lying) {   // on the back: the eye a head's height over the marker, looking up and along
        out.eye = at + Vec3(0, 0.25, 0);
        out.look = normalize(facing + Vec3(0, 1.6, 0));
        out.exit = Vec3(at.x, place != nullptr ? place->position.y : at.y, at.z) + turn.rotate(Vec3(1, 0, 0)) * Real(1.2);
        return true;
    }
    out.eye = at + Vec3(0, 0.72, 0) - facing * Real(0.06);
    out.look = normalize(facing + Vec3(0, -0.08, 0));
    out.exit = Vec3(at.x, place != nullptr ? place->position.y : at.y, at.z) + facing * Real(0.8);
    return true;
}

}  // namespace

void PrefabUseSystem::fixedUpdate(FrameContext& ctx) { stepPrefabUses(ctx.world, ctx.clock.fixedStep()); }

// THE LAMPS THAT ARE ON: each use with a `light` that is on publishes one candidate on the board's Prefabs channel,
// rebuilt every frame. LightSystem picks and lights them with every other light (ADR-0362); nothing here draws.
void PrefabUseSystem::render(FrameContext& ctx) {
    std::vector<LightCandidate>& lamps = ctx.view.lights.channel(LightChannel::Prefabs);
    lamps.clear();
    ctx.world.each<PrefabUses>([&](Entity e, PrefabUses& uses) {
        for (const PrefabUses::Use& u : uses.uses) {
            if (!u.on || u.use.light.empty()) continue;
            Vec3 at;
            Quat turn;
            if (!modelMarkerWorld(ctx.world, e, u.use.lightAt.empty() ? u.use.at : u.use.lightAt, at, turn)) continue;
            LightCandidate c;
            c.kind = lightKindId(u.use.light.c_str());
            if (c.kind < 0) continue;
            c.position = at;
            c.direction = turn.rotate(Vec3(0, 0, 1));
            lamps.push_back(c);
        }
    });
}

void PrefabUseSystem::update(FrameContext& ctx) {
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
    const Vec3 feet = pos - Vec3(0, feetDrop, 0);

    InteractCommand cmd;
    const bool commanded = takeInteractCommand(ctx.world, player, "prefab", cmd);

    // SEATED ON A PREFAB: stand up on the command, on any move key, or when the copy has gone.
    if (Seated* s = ctx.world.get<Seated>(player)) {
        const bool mine = s->set.valid() && ctx.world.alive(s->set) && ctx.world.has<PrefabUses>(s->set);
        if (!mine) return;   // the furniture's, or a seat whose owner has gone (InteractionSystem stands them up)
        const PrefabUses* uses = ctx.world.get<PrefabUses>(s->set);
        const bool move = std::fabs(ctx.actions.axis("cam_forward")) > 0.5 || std::fabs(ctx.actions.axis("cam_right")) > 0.5;
        if (move || commanded || s->piece >= uses->uses.size()) {
            setPrefabUse(ctx.world, s->set, s->piece, false);   // the seat is free again
            ctx.world.remove<Seated>(player);
            return;
        }
        Seated now = *s;
        if (seatAt(ctx.world, s->set, uses->uses[s->piece].use.at, uses->uses[s->piece].use.lie, now)) {   // carried with the seat
            s->eye = now.eye;
            s->exit = now.exit;
        }
        InteractOffer o;
        o.provider = "prefab";
        o.entity = s->set;
        o.key = s->piece;
        o.anchor = s->eye;
        o.tap = uses->uses[s->piece].use.lie ? "get up" : "stand up";
        o.name = uses->uses[s->piece].use.lie ? "lying down" : "seated";
        o.exclusive = true;
        offerInteraction(ctx.world, player, o);
        return;
    }

    // THE COMMAND: do the use the broker focused.
    if (commanded && cmd.entity.valid() && ctx.world.alive(cmd.entity)) {
        if (const PrefabUses* uses = ctx.world.get<PrefabUses>(cmd.entity); uses != nullptr && cmd.key < uses->uses.size()) {
            const PrefabUse use = uses->uses[static_cast<std::size_t>(cmd.key)].use;
            const bool done = doPrefabUse(ctx.world, cmd.entity, static_cast<std::size_t>(cmd.key));
            if (done && (use.sit || use.lie)) {
                Seated seat;
                seat.set = cmd.entity;
                seat.piece = static_cast<uint32_t>(cmd.key);
                if (seatAt(ctx.world, cmd.entity, use.at, use.lie, seat)) ctx.world.add<Seated>(player, seat);
            }
            return;
        }
    }

    // THE OFFERS: every use whose marker is in reach.
    struct Near {
        Entity e;
        std::size_t use;
        Vec3 at;
    };
    std::vector<Near> near;
    ctx.world.each<PrefabUses, Transform>([&](Entity e, PrefabUses& uses, Transform& t) {
        // (the copy's own point first: most copies are nowhere near)
        if (std::fabs(t.position.x - feet.x) > 12 || std::fabs(t.position.z - feet.z) > 12 || std::fabs(t.position.y - feet.y) > 6) return;
        for (std::size_t i = 0; i < uses.uses.size(); ++i) {
            Vec3 at;
            Quat turn;
            if (!prefabUseOffered(uses, i)) continue;
            if (!modelMarkerWorld(ctx.world, e, uses.uses[i].use.at, at, turn)) continue;
            if (std::hypot(at.x - feet.x, at.z - feet.z) > REACH || at.y < feet.y - 0.5 || at.y > feet.y + 2.6) continue;
            near.push_back(Near{e, i, at});
        }
    });
    for (const Near& n : near) {
        const PrefabUses* uses = ctx.world.get<PrefabUses>(n.e);
        const ModelInstance* mi = ctx.world.get<ModelInstance>(n.e);
        InteractOffer o;
        o.provider = "prefab";
        o.entity = n.e;
        o.key = n.use;
        o.anchor = n.at;
        o.tap = prefabUseWord(uses->uses[n.use]);
        o.name = mi != nullptr ? mi->prefab : std::string("prefab");
        o.reach = REACH;
        // (a seat is inside the thing's own collision shape as often as not: no sight test, as for a swing)
        o.needsSight = !(uses->uses[n.use].use.sit || uses->uses[n.use].use.lie);
        offerInteraction(ctx.world, player, o);
    }
}

}  // namespace engine
