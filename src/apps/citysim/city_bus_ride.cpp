#include "city_bus_ride.h"

#include "city_car.h"   // CityBusRuns
#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/input/input_map.h"
#include "../../engine/interact_broker.h"
#include "../../engine/riding.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Entity;
using engine::Mat4;
using engine::Real;
using engine::Transform;
using engine::Vec3;

namespace {
constexpr Real kSeatReach = 2.0;   // m: seats offered within this of where you stand
constexpr Real kWalk = 1.6;        // m/s down the aisle
Vec3 col(const Mat4& m, int c) { return Vec3(m.m[0][c], m.m[1][c], m.m[2][c]); }
}  // namespace

// ---- The system ---------------------------------------------------------------------------------------------------

void BusRideSystem::onStart(engine::FrameContext& ctx) {
    onBoard_ = ctx.events.listen<engine::BusBoardRequest>([this](const engine::BusBoardRequest& r) {
        board_ = r;
        boardPending_ = true;
    });
}

static Entity thePlayer(engine::World& world) {
    Entity player;
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&, engine::ControlledBy&) {
            if (!player.valid()) player = e;
        });
    return player;
}

void BusRideSystem::update(engine::FrameContext& ctx) {
    if (!bus_.valid()) return;
    const Entity player = thePlayer(ctx.world);
    if (!player.valid()) return;
    engine::InteractCommand cmd;
    if (engine::takeInteractCommand(ctx.world, player, "bus", cmd)) sitRequest_ = seat_ >= 0 ? -1 : static_cast<int>(cmd.key);
    BusCabin cabin;
    Mat4 drawn;
    if (!city_.busCabinOf(ctx.world, bus_, cabin, &drawn)) return;
    engine::InteractOffer o;
    o.provider = "bus";
    o.needsSight = false;
    if (seat_ >= 0) {
        o.tap = "stand up";
        o.name = "seat";
        o.exclusive = true;
        if (const Transform* pt = ctx.world.get<Transform>(player)) o.anchor = pt->position;
        engine::offerInteraction(ctx.world, player, o);
        return;
    }
    for (std::size_t i = 0; i < cabin.seats.size(); ++i) {
        const Vec3& s = cabin.seats[i];
        const Real dx = s.x - local_.x, dz = s.z - local_.z;
        if (dx * dx + dz * dz > kSeatReach * kSeatReach) continue;
        engine::InteractOffer so = o;
        so.key = i;
        so.anchor = drawn.transformPoint(s + Vec3(0, 0.55, 0));
        so.reach = 2.2;
        so.name = "seat";
        so.tap = "sit down";
        engine::offerInteraction(ctx.world, player, so);
    }
}

void BusRideSystem::fixedUpdate(engine::FrameContext& ctx) { ride(ctx, ctx.clock.fixedStep()); }

void BusRideSystem::getOff(engine::FrameContext& ctx, Entity player) {
    if (ctx.world.has<engine::Passenger>(player)) ctx.world.remove<engine::Passenger>(player);
    if (bus_.valid() && ctx.world.alive(bus_) && ctx.world.has<CityCarCarriesPlayer>(bus_)) ctx.world.remove<CityCarCarriesPlayer>(bus_);
    city_.setPlayerBusSeat(Entity{}, -1);
    bus_ = Entity{};
    seat_ = -1;
    sitRequest_ = -2;
}

void BusRideSystem::ride(engine::FrameContext& ctx, Real dt) {
    const Entity player = thePlayer(ctx.world);
    if (!player.valid()) {
        bus_ = Entity{};
        return;
    }
    Transform* pt = ctx.world.get<Transform>(player);
    auto* cc = ctx.world.get<engine::CharacterController>(player);
    if (!pt || !cc) return;
    const Real centreUp = cc->halfHeight + cc->radius;   // the capsule's centre over its feet
    auto pin = [&](const Vec3& pos) {
        pt->position = pos;
        if (auto* prev = ctx.world.get<engine::PrevTransform>(player)) prev->value = *pt;
        if (cc->characterId != engine::INVALID_CHARACTER) physics_.physicsWorld().setCharacterPosition(cc->characterId, pos);
        pinned_ = pos;
    };

    // `bus_board <bus>`: aboard that bus, standing inside its first door (a probe's way in)
    if (boardPending_) {
        boardPending_ = false;
        const Entity b = carEntityNumbered(ctx.world, static_cast<uint32_t>(std::max(0, board_.bus)));
        BusCabin cabin;
        if (b.valid() && city_.busCabinOf(ctx.world, b, cabin, nullptr)) {
            bus_ = b;
            local_ = busInside(cabin, 0);
            seat_ = -1;
            LOG_INFO << "[bus] aboard bus " << board_.bus << " (by request)";
        } else {
            LOG_WARN << "[bus] bus_board " << board_.bus << ": no such bus drawn with a saloon";
        }
    }

    if (bus_.valid()) {
        // moved by something else (a teleport, fast travel): that is getting off; the bus gone: off where you are
        if (!ctx.world.has<CityBusRuns>(bus_) || ((pt->position - pinned_).length() > 3.0 && pinned_.lengthSquared() > 0)) {
            getOff(ctx, player);
            return;
        }
    } else {
        // ON FOOT: walking into an open doorway of a bus standing at its stop is getting on
        const Vec3 feet = pt->position - Vec3(0, centreUp, 0);
        ctx.world.each<CityBusRuns, CityCarBody>([&](Entity e, CityBusRuns& runs, CityCarBody&) {
            if (bus_.valid() || !runs.standing) return;
            BusCabin cabin;
            Mat4 drawn;
            if (!city_.busCabinOf(ctx.world, e, cabin, &drawn)) return;
            const Vec3 l = drawn.inverse().transformPoint(feet);
            const int door = busDoorMouth(cabin, l);
            if (door < 0) return;
            bus_ = e;
            local_ = busInside(cabin, door);
            seat_ = -1;
            LOG_INFO << "[bus] you got on bus " << ctx.world.get<CityCarBody>(e)->id << " by door " << door;
        });
        if (!bus_.valid()) return;
    }
    if (!ctx.world.has<engine::Passenger>(player)) ctx.world.add<engine::Passenger>(player);
    if (!ctx.world.has<CityCarCarriesPlayer>(bus_)) ctx.world.add<CityCarCarriesPlayer>(bus_, CityCarCarriesPlayer{});

    BusCabin cabin;
    Mat4 drawn;
    if (!city_.busCabinOf(ctx.world, bus_, cabin, &drawn)) return;   // (not drawn this bake: held as it was)
    const CityBusRuns& runs = *ctx.world.get<CityBusRuns>(bus_);
    city_.setPlayerBusSeat(bus_, seat_);   // (the riders drawn aboard leave your seat; `buses?` says where you are)
    // SIT / STAND (the broker's E)
    if (sitRequest_ != -2) {
        if (sitRequest_ == -1 && seat_ >= 0) {   // stand up in the aisle beside the seat
            const Vec3& s = cabin.seats[static_cast<std::size_t>(seat_)];
            local_ = Vec3(0, busFloor(cabin), s.z);
            seat_ = -1;
        } else if (sitRequest_ >= 0 && sitRequest_ < static_cast<int>(cabin.seats.size())) {
            seat_ = sitRequest_;
            LOG_INFO << "[bus] you sat down: seat " << seat_;
        }
        sitRequest_ = -2;
        city_.setPlayerBusSeat(bus_, seat_);
    }
    // HELD IN THE BUS (engine/riding.h): at `local` in its frame as drawn this bake; the rest is rideStep's
    auto hold = [&](const Vec3& local, bool sitting, const Vec3& facing) {
        engine::Passenger* ps = ctx.world.get<engine::Passenger>(player);
        if (!ps) return;
        ps->carrier = drawn;
        ps->local = local;
        ps->sitting = sitting;
        ps->carSeat = false;   // (a bus's upright seat)
        ps->eyeAbove = sitting ? engine::kRiderSeatedEye : engine::kRiderStandingEye;
        ps->hipBelow = engine::kRiderSeatedEye - engine::kEyeAboveCentre;
        engine::rideStep(ctx.world, &physics_.physicsWorld(), player, &fly_, dt);
        if (sitting) {   // facing the way the seat faces (a wall bench: the aisle), not only the bus's forward
            Vec3 f = drawn.transformDirection(facing);
            f.y = 0;
            if (f.length() > 1e-6) ps->facing = f * (Real(1) / f.length());
        }
        pinned_ = pt->position;
    };
    if (seat_ >= 0) {
        const std::size_t k = static_cast<std::size_t>(seat_);
        hold(cabin.seats[k], true, k < cabin.facing.size() ? cabin.facing[k] : Vec3(0, 0, 1));
        return;
    }
    // WALK the aisle and the door wells in the bus's frame
    const Vec3 fwdAxis = col(drawn, 2), sideAxis = col(drawn, 0);
    Vec3 move = fly_.forward() * static_cast<Real>(ctx.actions.axis("cam_forward")) +
                fly_.right() * static_cast<Real>(ctx.actions.axis("cam_right"));
    move.y = 0;
    const Real ml = move.length();
    if (ml > 1) move = move * (1 / ml);
    Vec3 next = local_;
    next.x += engine::dot(move, sideAxis) * kWalk * dt;
    next.z += engine::dot(move, fwdAxis) * kWalk * dt;
    const bool doorsOpen = runs.standing;
    // (out of a doorway is a step you may take only while the doors are open)
    const auto ok = [&](const Vec3& p) { return busStepsOff(cabin, p) >= 0 ? doorsOpen : busWalkable(cabin, p); };
    if (ok(next)) local_ = next;   // (try the step, then each axis alone: slide along a seat row or a wall)
    else if (ok(Vec3(next.x, local_.y, local_.z))) local_.x = next.x;
    else if (ok(Vec3(local_.x, local_.y, next.z))) local_.z = next.z;
    local_.y = busFloor(cabin);
    // OUT OF THE DOOR, while the bus stands at its stop: onto the pavement beside the doorway
    if (doorsOpen && busStepsOff(cabin, local_) >= 0) {
        const int door = busStepsOff(cabin, local_);
        Vec3 out = drawn.transformPoint(busOutside(cabin, door));
        out.y += centreUp + 0.05;
        LOG_INFO << "[bus] you got off bus " << ctx.world.get<CityCarBody>(bus_)->id << " by door " << door;
        getOff(ctx, player);
        pin(out);
        return;
    }
    hold(local_, false, Vec3(0, 0, 1));
}

}  // namespace citysim
