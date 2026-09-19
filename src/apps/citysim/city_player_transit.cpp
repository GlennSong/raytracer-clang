#include "city_player_transit.h"

#include "city_render.h"
#include "city_sim.h"

#include "../../engine/components.h"
#include "../../engine/input/input_map.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/procgen/city/building_records.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <cmath>

namespace citysim {

using engine::Entity;
using engine::Real;
using engine::Transform;
using engine::Vec2;
using engine::Vec3;
using engine::World;

namespace {

// How close you must be to step aboard, and where you sit once you are. A bus
// is 11.4 m long, so "near the bus" is generous; the seat rides a little above
// its pose so the capsule is not buried in the floor.
constexpr Real kBoardRadius = 9.0;
constexpr Real kSeatLift = 1.1;
constexpr Real kStepOffDistance = 3.4;

}  // namespace

void CityPlayerTransitSystem::onStart(engine::FrameContext& ctx) {
    // E IS THE INTERACT KEY, and boarding is an interaction. It only acts when
    // a bus or cab is genuinely within reach, so the elevator (which also reads
    // E) is unaffected: a hoistway is indoors and a bus is not, and the two are
    // never both in range. A separate key would have been safer and worse --
    // "press E at the thing" is the convention the level already teaches.
    ctx.actions.bindButton("transit_board", engine::KeyCode::E);
    ctx.actions.setActionContext("transit_board", engine::InputContext::OnFoot);
}

void CityPlayerTransitSystem::fixedUpdate(engine::FrameContext& ctx) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();

    // The player, the same way every other citysim system finds it.
    Entity player;
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&,
            engine::ControlledBy&) {
            if (!player.valid()) player = e;
        });
    if (!player.valid()) {
        riding_ = -1;
        return;
    }
    Transform* pt = world.get<Transform>(player);
    auto* cc = world.get<engine::CharacterController>(player);
    if (!pt || !cc) return;

    // THE RIDE YIELDS TO ANYTHING ELSE THAT MOVES YOU. This system writes the
    // player's position every step, which means it beats every OTHER way of
    // moving the player -- F's fast travel, `teleport`, T, a respawn. Glenn hit
    // exactly that: F put him at the new place and the next tick dragged him
    // back to the bus he did not know he was on.
    //
    // Rather than enumerate the ways a player can be moved (and miss one), the
    // rule is: if the player is no longer where WE put them, someone else moved
    // them, and that IS getting off.
    if (riding_ >= 0 && seated_) {
        const Real dx = pt->position.x - seat_.x;
        const Real dy = pt->position.y - seat_.y;
        const Real dz = pt->position.z - seat_.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 3.0) {
            LOG_INFO << "[transit] you left the vehicle (moved by something else)";
            riding_ = -1;
            seated_ = false;
            return;
        }
    }

    // `ride bus|taxi|any` boards, `ride off` gets off. Staged as a Setting the
    // way every other control verb reaches a system.
    double req = ctx.settings.getDouble("player.rideRequest", -1.0);
    if (req >= 0.0) ctx.settings.setDouble("player.rideRequest", -1.0);
    // E: board whatever is in reach, or get off if already aboard.
    if (ctx.actions.pressed("transit_board")) req = (riding_ >= 0) ? 0.0 : 3.0;

    const std::vector<Agent>& agents = sim.agents();

    // --- get off -------------------------------------------------------
    if (riding_ >= 0 && (req == 0.0 || riding_ >= static_cast<int>(agents.size()))) {
        const Vec3 here = pt->position;
        riding_ = -1;
        seated_ = false;
        // Step off to the SIDE, not into the road the vehicle is standing in.
        const Real gy = city_.groundHeightAt(here.x, here.z);
        const Vec3 off(here.x + kStepOffDistance, gy + 1.2, here.z);
        pt->position = off;
        if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = *pt;
        if (cc->characterId != engine::INVALID_CHARACTER)
            physics_.physicsWorld().setCharacterPosition(cc->characterId, off);
        LOG_INFO << "[transit] you got off at (" << off.x << ", " << off.z << ")";
        wasAboard_ = false;
        return;
    }

    // --- get on --------------------------------------------------------
    // NOT FROM INDOORS. E is also the lift call, and a hoistway lobby can sit
    // within 9 m of a street: pressing E to call a lift while a bus passed
    // outside the wall would put you on the bus. If the player stands inside a
    // building plan, the interaction belongs to whatever is in there.
    bool indoors = false;
    world.each<engine::CityBuildings>([&](Entity, engine::CityBuildings& cb) {
        if (indoors) return;
        if (cb.recordAt(Vec2(pt->position.x, pt->position.z), pt->position.y))
            indoors = true;
    });
    if (indoors && req >= 1.0) {
        LOG_INFO << "[transit] not from indoors -- step outside to board";
        req = -1.0;
    }
    if (riding_ < 0 && req >= 1.0) {
        const bool wantBus = (req == 1.0 || req == 3.0);
        const bool wantTaxi = (req == 2.0 || req == 3.0);
        int best = -1;
        Real bestD = kBoardRadius;
        for (std::size_t i = 0; i < agents.size(); ++i) {
            const int ai = static_cast<int>(i);
            const bool isBus = sim.isBus(ai);
            const bool isTaxi = sim.isTaxi(ai);
            if (!((isBus && wantBus) || (isTaxi && wantTaxi))) continue;
            // You board at a STOP, not by touching a moving bus. Without this,
            // pressing E for anything else while a bus drove past captured the
            // player silently -- and E is the lift key.
            if (agents[i].speed > 2.0) continue;
            const Real dx = agents[i].pos.x - pt->position.x;
            const Real dz = agents[i].pos.y - pt->position.z;
            const Real d = std::sqrt(dx * dx + dz * dz);
            if (d < bestD) { bestD = d; best = ai; }
        }
        if (best < 0) {
            LOG_INFO << "[transit] nothing to board within " << kBoardRadius
                     << " m (stand nearer a bus or a cab)";
        } else {
            riding_ = best;
            LOG_INFO << "[transit] you boarded "
                     << (sim.isBus(best) ? "a bus" : "a cab") << " (agent "
                     << best << ") " << bestD << " m away";
        }
    }

    // --- ride ----------------------------------------------------------
    if (riding_ < 0) return;
    if (riding_ >= static_cast<int>(agents.size())) { riding_ = -1; return; }
    const Agent& veh = agents[static_cast<std::size_t>(riding_)];

    // Sit on the vehicle's pose. The ground under it is the reference, not the
    // agent's own elevation, so a bus on a bridge deck carries you at deck
    // height rather than dropping you through it.
    const Real gy = city_.groundHeightAt(veh.pos.x, veh.pos.y);
    const Vec3 seat(veh.pos.x, gy + veh.elevation + kSeatLift, veh.pos.y);
    pt->position = seat;
    if (auto* prev = world.get<engine::PrevTransform>(player)) prev->value = *pt;
    if (cc->characterId != engine::INVALID_CHARACTER)
        physics_.physicsWorld().setCharacterPosition(cc->characterId, seat);
    seat_ = seat;
    seated_ = true;
    if (!wasAboard_) wasAboard_ = true;
}

}  // namespace citysim
