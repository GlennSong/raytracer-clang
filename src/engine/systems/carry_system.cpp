#include "carry_system.h"
#include "physics_system.h"
#include "../carry_stow.h"
#include "../components.h"
#include "../interact_broker.h"
#include "../interaction.h"   // Seated
#include "../asset_manager.h"
#include "../mesh_builder.h"
#include "../script_assets.h"
#include "../scripting/script_vm.h"
#include "../../log.h"
#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr Real kReach = 2.0;      // m from the player to a thing to lift, or to a car's cargo space
constexpr Real kHoldOut = 0.75;   // m ahead of the player's middle a held thing rides...
constexpr Real kHoldUp = 0.05;    // ...and above it

// Whether `v`'s cargo space can be reached: its panel open most of the way, or none to open (a bed).
bool cargoOpen(const Vehicle& v) {
    if (!v.hasCargo) return false;
    if (v.cargoPanel < 0 || v.cargoPanel >= static_cast<int>(v.panels.size())) return true;
    const Vehicle::Panel& p = v.panels[static_cast<std::size_t>(v.cargoPanel)];
    return p.gone || (p.open > 0.0f && p.angle >= p.open * 0.6f);
}

Vec3 cargoMiddleWorld(const Transform& t, const Vehicle& v) {
    return t.position + t.orientation.rotate((v.cargoLo + v.cargoHi) * 0.5);
}

}  // namespace

void CarrySystem::onStart(FrameContext& ctx) {
#ifdef RT_ENABLE_SCRIPTING
    const std::string code = loadScriptCode("carry_items.lua", "");
    if (!code.empty()) {
        ScriptVM vm;
        std::string err;
        if (!loadCarryCatalogue(vm, code, catalogue_, &err)) LOG_WARN << "[carry] carry_items.lua: " << err;
    }
#endif
    if (ctx.commands != nullptr)   // `carry_spawn [name]`: a carry_items.lua thing in front of the player
        spawnCmd_ = ctx.commands->bind("carry_spawn", "CarrySystem", [this](const ControlCommand& cmd) -> std::string {
            spawnStaged_.push_back(cmd.args.empty() ? std::string("box") : cmd.args[0]);
            return "ok staged " + spawnStaged_.back();
        });
}

void CarrySystem::spawn(FrameContext& ctx, const std::string& name, const Vec3& at, Real yaw) {
    const CarryItem* item = nullptr;
    for (const CarryItem& c : catalogue_.items) if (c.name == name) item = &c;
    if (!item) {
        LOG_WARN << "[carry] carry_spawn: no `" << name << "` in carry_items.lua";
        return;
    }
    const Vec3 h = item->size * 0.5;
    RenderMesh m;
    MeshBuilder::emitBox(m, h * -1.0, h, item->color);
    const Entity e = ctx.world.create();
    Transform t;
    t.position = at + Vec3(0, h.y, 0);
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, {t});
    Renderable r;
    r.mesh = ctx.assets.acquireMesh(m, "carry:" + name);
    r.material.albedo = Vec3(1, 1, 1);
    r.material.roughness = 0.8f;
    r.material.opacity = 1.0f;
    ctx.world.add<Renderable>(e, r);
    Collider col;
    col.shape = ColliderShape::Box;
    col.halfExtent = h;
    col.friction = 0.7;
    ctx.world.add<Collider>(e, col);
    ctx.world.add<RigidBody>(e, RigidBody{});
    ctx.world.add<Carryable>(e, Carryable{name, static_cast<float>(item->mass), h});
    settle_.push_back({e, static_cast<float>(item->mass), Vec3(0, 0, 0)});
    LOG_INFO << "[carry] a " << name << " put down at " << at.x << " " << at.z;
}

void CarrySystem::pickUp(FrameContext& ctx, Entity player, Entity e) {
    if (RigidBody* rb = ctx.world.get<RigidBody>(e)) {
        if (rb->bodyId != INVALID_PHYSICS_BODY) physicsSys_.physicsWorld().removeBody(rb->bodyId);
        ctx.world.remove<RigidBody>(e);
    }
    if (const Stowed* s = ctx.world.get<Stowed>(e)) {
        if (Vehicle* v = ctx.world.alive(s->vehicle) ? ctx.world.get<Vehicle>(s->vehicle) : nullptr)
            v->cargo.erase(std::remove(v->cargo.begin(), v->cargo.end(), e), v->cargo.end());
        ctx.world.remove<Stowed>(e);
    }
    ctx.world.add<CarriedBy>(e, CarriedBy{player});
    held_ = e;
    const Carryable* c = ctx.world.get<Carryable>(e);
    LOG_INFO << "[carry] picked up the " << (c ? c->name : std::string("thing"));
}

void CarrySystem::putDown(FrameContext& ctx, const Vec3& velocity) {
    const Entity e = held_;
    held_ = Entity{};
    if (!ctx.world.alive(e)) return;
    ctx.world.remove<CarriedBy>(e);
    ctx.world.add<RigidBody>(e, RigidBody{});
    const Carryable* c = ctx.world.get<Carryable>(e);
    settle_.push_back({e, c ? c->mass : 10.0f, velocity});
    LOG_INFO << "[carry] put down the " << (c ? c->name : std::string("thing"));
}

bool CarrySystem::stow(FrameContext& ctx, Entity car, const Vec3& from) {
    const Entity e = held_;
    Vehicle* v = ctx.world.alive(car) ? ctx.world.get<Vehicle>(car) : nullptr;
    const Transform* ct = v ? ctx.world.get<Transform>(car) : nullptr;
    const Carryable* c = ctx.world.alive(e) ? ctx.world.get<Carryable>(e) : nullptr;
    if (!v || !ct || !c) return false;
    std::vector<StowBox> taken;
    for (const Entity o : v->cargo) {
        const Stowed* s = ctx.world.alive(o) ? ctx.world.get<Stowed>(o) : nullptr;
        const Carryable* oc = s ? ctx.world.get<Carryable>(o) : nullptr;
        if (s && oc) taken.push_back({s->local, turnedHalf(s->turn, oc->half)});
    }
    const Vec3 near = ct->orientation.conjugate().rotate(from - ct->position);
    Vec3 at;
    Quat turn;
    if (!stowSlot(v->cargoLo, v->cargoHi, c->half, taken, near, at, turn)) {
        LOG_INFO << "[carry] the " << c->name << " does not fit";
        return false;
    }
    held_ = Entity{};
    ctx.world.remove<CarriedBy>(e);
    ctx.world.add<Stowed>(e, Stowed{car, at, turn});
    v->cargo.push_back(e);
    if (Transform* t = ctx.world.get<Transform>(e)) {
        t->position = ct->position + ct->orientation.rotate(at);
        t->orientation = ct->orientation * turn;
        if (PrevTransform* pt = ctx.world.get<PrevTransform>(e)) pt->value = *t;
    }
    LOG_INFO << "[carry] put the " << c->name << " in (" << v->cargo.size() << " in it)";
    return true;
}

void CarrySystem::update(FrameContext& ctx) {
    Entity player;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform&, ControlledBy&) { if (!player.valid()) player = e; });
    if (!player.valid()) return;
    const Vec3 me = ctx.world.get<Transform>(player)->position;
    Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    fwd.y = 0;
    fwd = fwd.length() > 1e-6 ? fwd * (1.0 / fwd.length()) : Vec3(0, 0, 1);
    const Real yaw = std::atan2(fwd.x, fwd.z);
    for (const std::string& name : spawnStaged_) spawn(ctx, name, me + fwd * 1.6 - Vec3(0, 0.9, 0), yaw);
    spawnStaged_.clear();
    if (held_.valid() && !ctx.world.alive(held_)) held_ = Entity{};
    const bool onFoot = !ctx.world.has<InVehicle>(player) && !ctx.world.has<Seated>(player);

    // the broker's commands, from last frame's offers
    InteractCommand cmd;
    if (takeInteractCommand(ctx.world, player, "carry", cmd) && !held_.valid() && ctx.world.alive(cmd.entity))
        pickUp(ctx, player, cmd.entity);
    if (takeInteractCommand(ctx.world, player, "carry_put", cmd) && held_.valid()) {
        if (cmd.hold || !cmd.entity.valid() || !stow(ctx, cmd.entity, me)) putDown(ctx, Vec3(0, 0, 0));
    }

    if (held_.valid()) {
        if (!onFoot) {   // (got into a car, sat down: it is set down where it was)
            putDown(ctx, Vec3(0, 0, 0));
        } else if (Transform* t = ctx.world.get<Transform>(held_)) {   // in front of the player, square to the view
            t->position = me + fwd * kHoldOut + Vec3(0, kHoldUp, 0);
            t->orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
            if (PrevTransform* pt = ctx.world.get<PrevTransform>(held_)) pt->value = *t;
        }
    }
    if (!onFoot) return;

    // THE OFFERS. Holding something: the open cargo space nearest (put it in; held: put it down), else put it down.
    if (held_.valid()) {
        const Carryable* hc = ctx.world.get<Carryable>(held_);
        const std::string what = hc ? hc->name : std::string("it");
        Entity car;
        Vec3 at;
        Real best = kReach * kReach;
        ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
            if (v.driver.valid() || !cargoOpen(v)) return;
            const Vec3 c = cargoMiddleWorld(t, v);
            // the reach is to the cargo space's nearest point, not its middle (a pickup's bed is 2 m long)
            const Vec3 l = t.orientation.conjugate().rotate(me - t.position);
            const Vec3 q(std::clamp(l.x, v.cargoLo.x, v.cargoHi.x), std::clamp(l.y, v.cargoLo.y, v.cargoHi.y),
                         std::clamp(l.z, v.cargoLo.z, v.cargoHi.z));
            const Real dx = l.x - q.x, dz = l.z - q.z, d2 = dx * dx + dz * dz;
            if (d2 < best) { best = d2; car = e; at = c; }
        });
        InteractOffer o;
        o.provider = "carry_put";
        o.name = what;
        o.inVolume = true;
        o.needsSight = false;
        o.anchor = car.valid() ? at : me + fwd * kHoldOut;
        o.entity = car;
        o.tap = car.valid() ? "put it in" : "put it down";
        if (car.valid()) o.hold = "put it down";
        o.exclusive = true;
        offerInteraction(ctx.world, player, o);
        return;
    }
    // Hands free: the loose things in reach light enough to lift, and what is in an open cargo space.
    ctx.world.each<Transform, Carryable>([&](Entity e, Transform& t, Carryable& c) {
        if (ctx.world.has<CarriedBy>(e) || c.mass > catalogue_.limit) return;
        if (const Stowed* s = ctx.world.get<Stowed>(e)) {
            const Vehicle* v = ctx.world.alive(s->vehicle) ? ctx.world.get<Vehicle>(s->vehicle) : nullptr;
            if (!v || !cargoOpen(*v)) return;
        }
        const Real dx = t.position.x - me.x, dz = t.position.z - me.z;
        if (dx * dx + dz * dz > kReach * kReach) return;
        InteractOffer o;
        o.provider = "carry";
        o.entity = e;
        o.name = c.name;
        o.tap = ctx.world.has<Stowed>(e) ? "take it out" : "pick up";
        o.anchor = t.position;
        o.reach = kReach;
        o.needsSight = false;
        offerInteraction(ctx.world, player, o);
    });
}

void CarrySystem::fixedUpdate(FrameContext& ctx) {
    // a thing put down: its weight, and the throw, once PhysicsSystem has made its body
    PhysicsWorld& pw = physicsSys_.physicsWorld();
    for (std::size_t i = 0; i < settle_.size();) {
        const RigidBody* rb = ctx.world.alive(settle_[i].e) ? ctx.world.get<RigidBody>(settle_[i].e) : nullptr;
        if (!rb) { settle_.erase(settle_.begin() + static_cast<std::ptrdiff_t>(i)); continue; }
        if (rb->bodyId == INVALID_PHYSICS_BODY) { ++i; continue; }
        pw.setBodyMass(rb->bodyId, settle_[i].mass, 0.2, 0.4);
        pw.setLinearVelocity(rb->bodyId, settle_[i].velocity);
        settle_.erase(settle_.begin() + static_cast<std::ptrdiff_t>(i));
    }
    // what is in a car rides it; a car gone leaves it where it was, a body again
    std::vector<Entity> orphans;
    ctx.world.each<Transform, Stowed>([&](Entity e, Transform& t, Stowed& s) {
        const Transform* ct = ctx.world.alive(s.vehicle) ? ctx.world.get<Transform>(s.vehicle) : nullptr;
        if (!ct) { orphans.push_back(e); return; }
        if (PrevTransform* pt = ctx.world.get<PrevTransform>(e)) pt->value = t;
        t.position = ct->position + ct->orientation.rotate(s.local);
        t.orientation = ct->orientation * s.turn;
    });
    for (const Entity e : orphans) {
        ctx.world.remove<Stowed>(e);
        ctx.world.add<RigidBody>(e, RigidBody{});
        const Carryable* c = ctx.world.get<Carryable>(e);
        settle_.push_back({e, c ? c->mass : 10.0f, Vec3(0, 0, 0)});
    }
}

}  // namespace engine
