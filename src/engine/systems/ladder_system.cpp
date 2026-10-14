#include "ladder_system.h"

#include "../components.h"
#include "../interact_broker.h"
#include "../world.h"
#include "../procgen/city/building_records.h"
#include "../procgen/city/ladders.h"
#include "../procgen/city/shape_grammar.h"   // materialFor
#include "../interaction.h"   // Seated
#include "../../log.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

int ladderEndWithinReach(const LadderSpec& l, const Vec3& feet, Real reach) {
    // the foot: standing in front of the rails, on the ground they stand on
    const Vec2 front(l.foot.x + l.out.x * 0.5, l.foot.z + l.out.y * 0.5);
    if ((Vec2(feet.x, feet.z) - front).length() < reach && std::fabs(feet.y - l.foot.y) < 0.8) return 1;
    // the top: standing on the surface it lands on, about where it lets you off
    if ((Vec2(feet.x, feet.z) - Vec2(l.exit.x, l.exit.z)).length() < reach + 0.5 && std::fabs(feet.y - l.landY) < 0.6) return 2;
    return 0;
}

void LadderSystem::onStart(FrameContext& ctx) {
    laddersEntity_ = ctx.world.create();
    ctx.world.add<Ladders>(laddersEntity_, Ladders{});
}

void LadderSystem::onStop(FrameContext& ctx) {
    for (auto& kv : drawn_) {
        if (ctx.world.alive(kv.second.e)) ctx.world.destroy(kv.second.e);
        ctx.assets.releaseMesh(kv.second.mesh);
    }
    drawn_.clear();
}

void LadderSystem::update(FrameContext& ctx) {
    World& world = ctx.world;
    Ladders* lad = world.get<Ladders>(laddersEntity_);
    if (!lad) return;
    // THE LIST, once the city is in: every building's roof ladder (records are a whole level, built at load)
    if (!lad->built) {
        const CityBuildings* cb = nullptr;
        world.each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
        if (cb && !cb->records.empty()) {
            for (const BuildingRecord& r : cb->records) {
                LadderSpec l;
                if (roofLadderFor(r.plan, r.params, r.baseY, r.baseY + r.height, l)) lad->fixed.push_back(l);
            }
            lad->built = true;
            LOG_INFO << "[ladders] " << lad->fixed.size() << " roof ladders";
            for (std::size_t i = 0; i < std::min<std::size_t>(lad->fixed.size(), 3); ++i)
                LOG_INFO << "[ladders] roof ladder: foot " << lad->fixed[i].foot.x << " " << lad->fixed[i].foot.y << " "
                         << lad->fixed[i].foot.z << " out " << lad->fixed[i].out.x << " " << lad->fixed[i].out.y
                         << " roof " << lad->fixed[i].landY;
        }
    }

    // The player.
    Entity player;
    Vec3 pos;
    Real feetDrop = 0.7;
    world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (player.valid()) return;
        player = e;
        pos = t.position;
        if (const CharacterController* cc = world.get<CharacterController>(e)) feetDrop = cc->halfHeight + cc->radius;
    });
    if (!player.valid()) return;
    const Vec3 feet = pos - Vec3(0, feetDrop, 0);

    // DRAWN: the buildings' ladders near the player (a vehicle's is part of its body)
    for (std::size_t i = 0; i < lad->fixed.size(); ++i) {
        const LadderSpec& l = lad->fixed[i];
        const Real dd = (Vec2(l.foot.x, l.foot.z) - Vec2(pos.x, pos.z)).length();
        auto it = drawn_.find(i);
        if (dd < DRAW_M && it == drawn_.end()) {
            Drawn d;
            d.mesh = ctx.assets.acquireMesh(ladderMesh(l));
            d.e = world.create();
            Transform t;
            world.add<Transform>(d.e, t);
            world.add<PrevTransform>(d.e, PrevTransform{t});
            Renderable rd;
            rd.mesh = d.mesh;
            rd.material = materialFor(PartId::Metal, Vec3(0.55, 0.56, 0.58));
            world.add<Renderable>(d.e, rd);
            drawn_.emplace(i, d);
        } else if (dd > RELEASE_M && it != drawn_.end()) {
            if (world.alive(it->second.e)) world.destroy(it->second.e);
            ctx.assets.releaseMesh(it->second.mesh);
            drawn_.erase(it);
        }
    }

    if (world.has<InVehicle>(player) || world.has<Passenger>(player) || world.has<Seated>(player)) return;
    // ON A LADDER: the PlayerSystem climbs; nothing to offer
    if (world.has<Climbing>(player)) return;

    // THE COMMAND: onto the ladder the broker focused (key: index << 2 | moving << 1 | top)
    InteractCommand cmd;
    if (takeInteractCommand(world, player, "ladder", cmd)) {
        const bool moving = (cmd.key >> 1) & 1u, top = cmd.key & 1u;
        const std::size_t i = static_cast<std::size_t>(cmd.key >> 2);
        const std::vector<LadderSpec>& from = moving ? lad->moving : lad->fixed;
        if (i < from.size()) {
            Climbing c;
            c.ladder = from[i];
            c.feetY = top ? c.ladder.landY - 1.4 : c.ladder.foot.y + 0.05;
            world.add<Climbing>(player, c);
            return;
        }
    }
    // THE OFFERS: every ladder end within reach
    auto offer = [&](const std::vector<LadderSpec>& list, bool moving) {
        for (std::size_t i = 0; i < list.size(); ++i) {
            const LadderSpec& l = list[i];
            if ((Vec2(l.foot.x, l.foot.z) - Vec2(feet.x, feet.z)).length() > 4.0) continue;
            const int end = ladderEndWithinReach(l, feet);
            if (end == 0) continue;
            InteractOffer o;
            o.provider = "ladder";
            o.key = (static_cast<uint64_t>(i) << 2) | (moving ? 2u : 0u) | (end == 2 ? 1u : 0u);
            o.anchor = end == 1 ? l.foot + Vec3(l.out.x * 0.15, 1.4, l.out.y * 0.15) : Vec3(l.foot.x, l.landY + 0.9, l.foot.z);
            o.tap = end == 1 ? "climb" : "climb down";
            o.name = "ladder";
            o.reach = REACH + 0.6;
            o.needsSight = false;   // (its rails stand off a wall: the wall would hide its own foot)
            offerInteraction(world, player, o);
        }
    };
    offer(lad->fixed, false);
    offer(lad->moving, true);
}

}  // namespace engine
