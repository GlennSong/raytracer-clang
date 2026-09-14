#include "elevator_system.h"

#include "physics_system.h"
#include "../components.h"
#include "../world.h"
#include "../../log.h"
#include "../procgen/city/shape_grammar.h"
#include <algorithm>
#include <cmath>
#include <limits>
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

namespace {
constexpr Real kFeet = 0.7;          // the capsule centre (halfHeight 0.4 + radius 0.3) above the feet
constexpr Real kLeafW = 0.55;        // one leaf of the 1.1 m hoistway door
constexpr Real kLeafT = 0.04;
constexpr Real kWallMid = -0.075;    // the leaf slides inside the 0.15 m wall

Quat yawOf(const CoreShaft& hw) {
    // A box's local +Z along the shaft's v (the facing convention yaw = atan2(dir.x, dir.z)).
    return Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(hw.frame.v.x, hw.frame.v.y));
}

long long leafKey(int storey, int hoistway, bool left) {
    return (static_cast<long long>(storey) << 9) | (static_cast<long long>(hoistway) << 1) | (left ? 1 : 0);
}

RenderMaterial steel() {
    RenderMaterial m;
    m.albedo = {0.58, 0.58, 0.60};
    m.metallic = 0.85f;
    m.roughness = 0.32f;
    return m;
}
}  // namespace

void ElevatorSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("elevator_call", KeyCode::E);
    ctx.actions.bindButton("elevator_floor_up", KeyCode::Up);
    ctx.actions.bindButton("elevator_floor_down", KeyCode::Down);
}

void ElevatorSystem::update(FrameContext& ctx) {
    // Edges are frame-rate events; fixedUpdate consumes them.
    if (ctx.actions.pressed("elevator_call")) callEdge_ = true;
    if (ctx.actions.pressed("elevator_floor_up")) ++floorDelta_;
    if (ctx.actions.pressed("elevator_floor_down")) --floorDelta_;
}

void ElevatorSystem::fixedUpdate(FrameContext& ctx) {
    Vec3 player;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (found) return;
        if (ctx.world.has<InVehicle>(e)) return;   // no elevators from a car seat
        player = t.position;
        found = true;
    });
    if (!found) { callEdge_ = false; floorDelta_ = 0; status_ = Status{}; return; }
    PhysicsWorld* phys = physics_ ? &physics_->physicsWorld() : nullptr;
    step(ctx.world, phys, ctx.assets, player, ctx.clock.fixedStep(), callEdge_, floorDelta_);
    callEdge_ = false;
    floorDelta_ = 0;
}

int ElevatorSystem::storeyOf(const Bank& b, Real y) {
    const Real feet = y - kFeet + 0.4;   // a step of tolerance: mid-flight rounds up
    int k = 0;
    for (std::size_t i = 1; i < b.storeyY.size(); ++i)
        if (b.storeyY[i] <= feet) k = static_cast<int>(i);
    return k;
}

Real ElevatorSystem::cabY(std::size_t record, std::size_t i) const {
    auto it = banks_.find(record);
    if (it == banks_.end() || i >= it->second.cabs.size()) return std::numeric_limits<Real>::quiet_NaN();
    return it->second.cabs[i].y;
}

ElevatorSystem::Bank& ElevatorSystem::ensureBank(World& world, PhysicsWorld* phys, AssetManager& assets,
                                                 const BuildingRecord& r, std::size_t key) {
    auto it = banks_.find(key);
    if (it != banks_.end()) return it->second;
    Bank b;
    b.core = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
    for (const StoreyPlan& sp : storeyPlans(r.plan, r.params)) b.storeyY.push_back(r.baseY + sp.y0 + 0.05);
    for (std::size_t i = 0; i < b.core.hoistways.size(); ++i) {
        const CoreShaft& hw = b.core.hoistways[i];
        Cab cab;
        cab.y = b.storeyY.empty() ? r.baseY : b.storeyY[0];
        // Parts in the hoistway frame (u across, v in): a floor, a lit
        // ceiling, the back and the two sides; the front is open.
        struct Part { Vec3 half; Vec3 local; bool lit; bool dark; };
        const Real hw2 = CAB_W * 0.5, hd2 = CAB_D * 0.5;
        const Real u0 = hw.width * 0.5, vFront = 0.25, vMid = vFront + hd2 + 0.05;
        // The floor and ceiling reach to 5 cm off the door wall, so the sill
        // gap reads as a threshold, not a slot (the capsule could never fall
        // through a 0.25 m gap, but the eye would).
        const Real vDeck = (0.05 + vFront + CAB_D + 0.1) * 0.5, hDeck = vDeck - 0.05;
        const Part parts[5] = {
            {{hw2 + 0.05, 0.06, hDeck}, {u0, -0.06, vDeck}, false, true},                    // floor
            {{hw2 + 0.05, 0.03, hDeck}, {u0, CAB_H + 0.03, vDeck}, true, false},              // ceiling
            {{hw2 + 0.05, CAB_H * 0.5, 0.05}, {u0, CAB_H * 0.5, vFront + CAB_D + 0.05}, false, false},   // back
            {{0.05, CAB_H * 0.5, hd2 + 0.05}, {u0 - hw2 - 0.05, CAB_H * 0.5, vMid}, false, false},       // left
            {{0.05, CAB_H * 0.5, hd2 + 0.05}, {u0 + hw2 + 0.05, CAB_H * 0.5, vMid}, false, false},       // right
        };
        const Quat rot = yawOf(hw);
        for (const Part& p : parts) {
            const Vec3 pos = hw.at(p.local.x, cab.y + p.local.y, p.local.z);
            MeshHandle mh = assets.acquirePrimitive("box", p.half * 2.0);
            Entity e = world.create();
            Transform t;
            t.position = pos;
            t.orientation = rot;
            world.add<Transform>(e, t);
            world.add<PrevTransform>(e, PrevTransform{t});
            Renderable rd;
            rd.mesh = mh;
            rd.material = steel();
            if (p.dark) { rd.material.albedo = {0.22, 0.21, 0.20}; rd.material.metallic = 0.1f; rd.material.roughness = 0.6f; }
            if (p.lit) { rd.material.albedo = {0.9, 0.9, 0.9}; rd.material.metallic = 0.0f; rd.material.emission = Vec3(1.0, 0.97, 0.92) * 2.2; }
            rd.drawClass = DrawClass::Structure;
            rd.drawDistance = 200.0;
            world.add<Renderable>(e, rd);
            cab.entities.push_back(e);
            cab.meshes.push_back(mh);
            cab.local.push_back(p.local);
            cab.bodies.push_back(phys ? phys->addBox(p.half, pos, rot, BodyMotion::Kinematic, 0.0, 0.8)
                                      : INVALID_PHYSICS_BODY);
        }
        b.cabs.push_back(std::move(cab));
    }
    LOG_INFO << "[elevator] bank for record " << key << ": " << b.cabs.size() << " cabs, "
             << b.storeyY.size() << " storeys";
    return banks_.emplace(key, std::move(b)).first->second;
}

void ElevatorSystem::releaseBank(World& world, PhysicsWorld* phys, AssetManager& assets, std::size_t key) {
    auto it = banks_.find(key);
    if (it == banks_.end()) return;
    for (Cab& cab : it->second.cabs) {
        for (PhysicsBodyId id : cab.bodies) if (phys && id != INVALID_PHYSICS_BODY) phys->removeBody(id);
        for (Entity e : cab.entities) if (world.alive(e)) world.destroy(e);
        for (MeshHandle mh : cab.meshes) assets.releaseMesh(mh);
    }
    for (auto& kv : it->second.leaves) {
        if (phys && kv.second.body != INVALID_PHYSICS_BODY) phys->removeBody(kv.second.body);
        if (world.alive(kv.second.entity)) world.destroy(kv.second.entity);
        assets.releaseMesh(kv.second.mesh);
    }
    banks_.erase(it);
}

void ElevatorSystem::placeCab(World& world, PhysicsWorld* phys, const Bank&, Cab& cab, const CoreShaft& hw,
                              Real dt) {
    const Quat rot = yawOf(hw);
    for (std::size_t i = 0; i < cab.entities.size(); ++i) {
        const Vec3 pos = hw.at(cab.local[i].x, cab.y + cab.local[i].y, cab.local[i].z);
        if (Transform* t = world.get<Transform>(cab.entities[i])) {
            if (PrevTransform* pt = world.get<PrevTransform>(cab.entities[i])) pt->value = *t;
            t->position = pos;
            t->orientation = rot;
        }
        if (phys && cab.bodies[i] != INVALID_PHYSICS_BODY) phys->moveKinematic(cab.bodies[i], pos, rot, dt);
    }
}

void ElevatorSystem::syncLeaves(World& world, PhysicsWorld* phys, AssetManager& assets, Bank& b,
                                int playerStorey, Real dt) {
    const int n = static_cast<int>(b.storeyY.size());
    std::vector<long long> wanted;
    auto want = [&](int storey, int hw) {
        if (storey < 0 || storey >= n) return;
        wanted.push_back(leafKey(storey, hw, true));
        wanted.push_back(leafKey(storey, hw, false));
    };
    for (int hw = 0; hw < static_cast<int>(b.core.hoistways.size()); ++hw) {
        for (int s = playerStorey - LEAF_WINDOW; s <= playerStorey + LEAF_WINDOW; ++s) want(s, hw);
        want(b.cabs[static_cast<std::size_t>(hw)].floor, hw);
        want(b.cabs[static_cast<std::size_t>(hw)].target, hw);
    }
    std::sort(wanted.begin(), wanted.end());
    wanted.erase(std::unique(wanted.begin(), wanted.end()), wanted.end());
    // Drop the leaves outside the window.
    for (auto it = b.leaves.begin(); it != b.leaves.end();) {
        if (std::binary_search(wanted.begin(), wanted.end(), it->first)) { ++it; continue; }
        if (phys && it->second.body != INVALID_PHYSICS_BODY) phys->removeBody(it->second.body);
        if (world.alive(it->second.entity)) world.destroy(it->second.entity);
        assets.releaseMesh(it->second.mesh);
        it = b.leaves.erase(it);
    }
    // Create the missing ones and pose them all.
    for (long long key : wanted) {
        const bool left = key & 1;
        const int hwi = static_cast<int>((key >> 1) & 0xff);
        const int storey = static_cast<int>(key >> 9);
        const CoreShaft& hw = b.core.hoistways[static_cast<std::size_t>(hwi)];
        const Cab& cab = b.cabs[static_cast<std::size_t>(hwi)];
        const bool here = cab.floor == storey && cab.state != CabState::Moving &&
                          !(cab.state == CabState::Idle && cab.target != cab.floor);
        const Real open = here ? cab.doorT : 0.0;
        const Real u = hw.doorX + (left ? -1.0 : 1.0) * (kLeafW * 0.5 + kLeafW * open);
        const Real yBase = b.storeyY[static_cast<std::size_t>(storey)] - 0.05;
        const Vec3 pos = hw.at(u, yBase + hw.doorHeight * 0.5, kWallMid);
        const Quat rot = yawOf(hw);
        auto it = b.leaves.find(key);
        if (it == b.leaves.end()) {
            Leaf lf;
            lf.storey = storey;
            lf.hoistway = hwi;
            lf.left = left;
            lf.mesh = assets.acquirePrimitive("box", Vec3(kLeafW, hw.doorHeight, kLeafT));
            Entity e = world.create();
            Transform t;
            t.position = pos;
            t.orientation = rot;
            world.add<Transform>(e, t);
            world.add<PrevTransform>(e, PrevTransform{t});
            Renderable rd;
            rd.mesh = lf.mesh;
            rd.material = steel();
            rd.material.albedo = {0.5, 0.5, 0.53};
            rd.drawClass = DrawClass::Structure;
            rd.drawDistance = 150.0;
            world.add<Renderable>(e, rd);
            lf.entity = e;
            lf.body = phys ? phys->addBox(Vec3(kLeafW * 0.5, hw.doorHeight * 0.5, kLeafT * 0.5), pos, rot,
                                          BodyMotion::Kinematic, 0.0, 0.4)
                           : INVALID_PHYSICS_BODY;
            it = b.leaves.emplace(key, lf).first;
            continue;   // created in place; nothing to move this step
        }
        if (Transform* t = world.get<Transform>(it->second.entity)) {
            if (PrevTransform* pt = world.get<PrevTransform>(it->second.entity)) pt->value = *t;
            t->position = pos;
            t->orientation = rot;
        }
        if (phys && it->second.body != INVALID_PHYSICS_BODY) phys->moveKinematic(it->second.body, pos, rot, dt);
    }
}

void ElevatorSystem::step(World& world, PhysicsWorld* phys, AssetManager& assets, const Vec3& player,
                          Real dt, bool call, int floorDelta) {
    status_ = Status{};
    const CityBuildings* cb = nullptr;
    world.each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
    if (!cb || cb->records.empty()) return;
    const Vec2 xz(player.x, player.z);

    // The building the player is in, if it has a core.
    std::vector<const BuildingRecord*> nearby;
    cb->near(xz, 2.0, nearby);
    std::size_t insideKey = static_cast<std::size_t>(-1);
    for (const BuildingRecord* r : nearby) {
        if (!r->enterable || r->plan.size() < 3) continue;
        if (player.y < r->groundY - 0.5 || player.y > r->baseY + r->height + 0.5) continue;
        if (!pointInPolygon(r->plan, xz)) continue;
        if (!wantsCore(r->params)) continue;
        insideKey = static_cast<std::size_t>(r - cb->records.data());
        break;
    }
    // Free banks the player has left behind.
    std::vector<std::size_t> gone;
    for (const auto& kv : banks_) {
        if (kv.first == insideKey) continue;
        const Vec2 c = centroid(kv.second.core.rect());
        if ((c - xz).length() > RELEASE_M) gone.push_back(kv.first);
    }
    for (std::size_t key : gone) releaseBank(world, phys, assets, key);
    if (insideKey == static_cast<std::size_t>(-1)) return;
    const BuildingRecord& rec = cb->records[insideKey];
    Bank& b = ensureBank(world, phys, assets, rec, insideKey);
    if (!b.core.valid || b.cabs.empty() || b.storeyY.size() < 2) return;
    const int n = static_cast<int>(b.storeyY.size());
    const int f = storeyOf(b, player.y);

    // Where the player stands relative to the bank: in a cab, or at a door.
    int inCab = -1, atDoor = -1;
    for (std::size_t i = 0; i < b.cabs.size(); ++i) {
        const CoreShaft& hw = b.core.hoistways[i];
        const Vec2 q = hw.frame.toFrame(xz);
        const Real u0 = hw.width * 0.5 - CAB_W * 0.5, u1 = hw.width * 0.5 + CAB_W * 0.5;
        const Cab& cab = b.cabs[i];
        if (q.x >= u0 && q.x <= u1 && q.y >= 0.2 && q.y <= 0.3 + CAB_D && player.y - kFeet > cab.y - 0.6 &&
            player.y - kFeet < cab.y + 1.5)
            inCab = static_cast<int>(i);
        if (inCab < 0 && std::fabs(q.x - hw.doorX) < 1.0 && q.y < -0.1 && q.y > -CALL_M) atDoor = static_cast<int>(i);
    }

    // Verbs.
    if (inCab >= 0) {
        Cab& cab = b.cabs[static_cast<std::size_t>(inCab)];
        if (floorDelta != 0) b.selected = std::max(0, std::min(n - 1, b.selected + floorDelta));
        if (call) {
            if (b.selected != cab.floor) {
                cab.target = b.selected;
                if (cab.state == CabState::Open || cab.state == CabState::Opening) cab.state = CabState::Closing;
            } else if (cab.state == CabState::Idle || cab.state == CabState::Closing) {
                cab.state = CabState::Opening;   // let them out again
            }
        }
    } else {
        b.selected = f;
        if (call && atDoor >= 0) {
            // The nearest idle cab (or one already here) comes to this storey.
            int best = -1;
            Real bestD = 1e30;
            for (std::size_t i = 0; i < b.cabs.size(); ++i) {
                const Cab& c = b.cabs[i];
                if (c.state == CabState::Moving && c.target != f) continue;
                const Real d = std::fabs(c.y - b.storeyY[static_cast<std::size_t>(f)]) + (c.state == CabState::Moving ? 0 : 0.01);
                if (d < bestD) { bestD = d; best = static_cast<int>(i); }
            }
            if (best < 0) best = atDoor;
            Cab& c = b.cabs[static_cast<std::size_t>(best)];
            c.target = f;
            if (c.floor == f && (c.state == CabState::Idle || c.state == CabState::Closing)) c.state = CabState::Opening;
            else if (c.floor == f && c.state == CabState::Open) c.dwell = DWELL_S;
            else if (c.state == CabState::Open || c.state == CabState::Opening) c.state = CabState::Closing;
        }
    }

    // The cabs.
    bool anyMoving = false;
    for (std::size_t i = 0; i < b.cabs.size(); ++i) {
        Cab& cab = b.cabs[i];
        const Real yTarget = b.storeyY[static_cast<std::size_t>(std::max(0, std::min(n - 1, cab.target)))];
        switch (cab.state) {
            case CabState::Idle:
                cab.vel = 0;
                if (cab.target != cab.floor) cab.state = CabState::Moving;
                break;
            case CabState::Opening:
                cab.doorT = std::min(Real(1), cab.doorT + dt / DOOR_S);
                if (cab.doorT >= 1) { cab.state = CabState::Open; cab.dwell = DWELL_S; }
                break;
            case CabState::Open:
                if (static_cast<int>(i) == inCab) cab.dwell = std::max(cab.dwell, Real(1.0));
                cab.dwell -= dt;
                if (cab.dwell <= 0 || cab.target != cab.floor) cab.state = CabState::Closing;
                break;
            case CabState::Closing:
                cab.doorT = std::max(Real(0), cab.doorT - dt / DOOR_S);
                if (cab.doorT <= 0) cab.state = cab.target != cab.floor ? CabState::Moving : CabState::Idle;
                break;
            case CabState::Moving: {
                const Real d = yTarget - cab.y;
                const Real dir = d >= 0 ? 1.0 : -1.0;
                const Real allowed = std::sqrt(std::max(Real(0), 2.0 * ACCEL * std::fabs(d)));
                cab.vel = std::min({SPEED, allowed, cab.vel + ACCEL * dt});
                Real stepY = cab.vel * dt;
                if (stepY >= std::fabs(d)) {
                    cab.y = yTarget;
                    cab.vel = 0;
                    cab.floor = cab.target;
                    cab.state = CabState::Opening;
                    cab.doorT = 0;
                } else {
                    cab.y += dir * stepY;
                }
                anyMoving = anyMoving || cab.state == CabState::Moving;
                break;
            }
        }
        placeCab(world, phys, b, cab, b.core.hoistways[i], dt);
    }
    syncLeaves(world, phys, assets, b, f, dt);

    status_.inCab = inCab >= 0;
    status_.atDoor = atDoor >= 0;
    status_.floors = n;
    status_.selected = b.selected;
    if (inCab >= 0) {
        const Cab& cab = b.cabs[static_cast<std::size_t>(inCab)];
        status_.floor = cab.floor;
        status_.moving = cab.state == CabState::Moving;
    } else {
        status_.floor = f;
        status_.moving = anyMoving;
    }
}

void ElevatorSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (!status_.inCab && !status_.atDoor) return;
    ImGui::SetNextWindowPos(ImVec2(24.0f, static_cast<float>(ctx.windowHeight) - 96.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("Elevator", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
                     ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoSavedSettings);
    if (status_.inCab) {
        ImGui::Text("ELEVATOR   floor %d of %d", status_.floor, status_.floors - 1);
        if (status_.moving) ImGui::Text("moving to %d", status_.selected);
        else ImGui::Text("Up / Down: pick floor %d      E: go", status_.selected);
    } else {
        ImGui::Text("ELEVATOR   floor %d      E: call", status_.floor);
    }
    ImGui::End();
#else
    (void)ctx;
#endif
}

void ElevatorSystem::onStop(FrameContext& ctx) {
    PhysicsWorld* phys = physics_ ? &physics_->physicsWorld() : nullptr;
    std::vector<std::size_t> keys;
    for (const auto& kv : banks_) keys.push_back(kv.first);
    for (std::size_t key : keys) releaseBank(ctx.world, phys, ctx.assets, key);
    status_ = Status{};
}

}  // namespace engine
