#include "building_interior_system.h"

#include "physics_system.h"
#include "../components.h"
#include "../../log.h"
#include "../../profile.h"
#include "../world.h"
#include "../procgen/city/shape_grammar.h"
#include <algorithm>
#include <chrono>

namespace engine {

namespace {
double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
}
}  // namespace

void BuildingInteriorSystem::fixedUpdate(FrameContext& ctx) {
    RT_PROFILE_ZONE_NAMED("buildingInteriors");
    if (!physics_) return;
    // The player (InVehicle redirect — the terrain window's rule: follow the
    // CAR, not the parked character).
    Vec3 player;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>(
        [&](Entity e, Transform& t, ControlledBy&) {
            if (found) return;
            Entity tracked = e;
            if (const InVehicle* iv = ctx.world.get<InVehicle>(e))
                if (iv->vehicle.valid() && ctx.world.alive(iv->vehicle) &&
                    ctx.world.has<Transform>(iv->vehicle))
                    tracked = iv->vehicle;
            const Transform* tt =
                tracked == e ? &t : ctx.world.get<Transform>(tracked);
            player = tt->position;
            found = true;
        });
    if (!found) return;
    step(ctx.world, &physics_->physicsWorld(), ctx.assets, player);
}

void BuildingInteriorSystem::step(World& world, PhysicsWorld* phys,
                                  AssetManager& assets, const Vec3& player) {
    ++stepCount_;
    const CityBuildings* cb = nullptr;
    world.each<CityBuildings>(
        [&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
    if (!cb || cb->records.empty()) {
        // No city: still drain the free queue so onStop leftovers clear.
        if (!freeQueue_.empty() && stepCount_ - lastFree_ >= FREE_EVERY) {
            assets.releaseMesh(freeQueue_.back());
            freeQueue_.pop_back();
            lastFree_ = stepCount_;
        }
        return;
    }

    const Vec2 xz(player.x, player.z);

    // Desired residents: enterable records the player is INSIDE or
    // APPROACHING (in front of a door, within APPROACH_M).
    std::vector<const BuildingRecord*> nearby;
    cb->near(xz, RELEASE_M, nearby);
    std::vector<std::size_t> desired;   // record indices
    std::size_t insideKey = static_cast<std::size_t>(-1);
    for (const BuildingRecord* r : nearby) {
        if (!r->enterable) continue;
        const std::size_t key =
            static_cast<std::size_t>(r - cb->records.data());
        const bool inside =
            player.y > r->groundY - 0.5 &&
            player.y < r->baseY + r->height + 0.5 && r->plan.size() >= 3 &&
            pointInPolygon(r->plan, xz);
        bool approaching = false;
        for (const DoorSpec& d : r->doors) {
            const Vec2 toP = xz - d.foot;
            if (toP.length() < APPROACH_M && dot(toP, d.normal) > 0) {
                approaching = true;
                break;
            }
        }
        if (inside) insideKey = key;
        if (inside || approaching) desired.push_back(key);
    }

    // ONE build per step; the record the player is inside builds regardless.
    bool builtThisStep = false;
    for (std::size_t key : desired) {
        if (resident_.count(key)) continue;
        if (resident_.size() >= MAX_RESIDENT && key != insideKey) continue;
        if (builtThisStep && key != insideKey) continue;
        build(world, phys, assets, cb->records[key], key);
        builtThisStep = true;
    }

    // Release: resident, not desired, and beyond RELEASE_M from every door
    // (hysteresis — desired only reaches APPROACH_M).
    std::vector<std::size_t> toFree;
    for (const auto& kv : resident_) {
        if (std::find(desired.begin(), desired.end(), kv.first) !=
            desired.end())
            continue;
        const BuildingRecord& r = cb->records[kv.first];
        double nearest = 1e300;
        for (const DoorSpec& d : r.doors)
            nearest = std::min(nearest, (double)(xz - d.foot).length());
        if (r.doors.empty())
            nearest = (xz - Vec2(r.plan[0].x, r.plan[0].y)).length();
        if (nearest > RELEASE_M) toFree.push_back(kv.first);
    }
    for (std::size_t key : toFree) release(world, phys, key);

    // Rate-limited GPU frees: never on a build step (the device wait must not
    // share a step with a grow), at most one per FREE_EVERY steps.
    if (!builtThisStep && !freeQueue_.empty() &&
        stepCount_ - lastFree_ >= FREE_EVERY) {
        assets.releaseMesh(freeQueue_.back());
        freeQueue_.pop_back();
        lastFree_ = stepCount_;
    }
}

void BuildingInteriorSystem::build(World& world, PhysicsWorld* phys,
                                   AssetManager& assets,
                                   const BuildingRecord& r, std::size_t key) {
    const auto t0 = std::chrono::steady_clock::now();
    RenderMesh collider;
    BuildingMesh bm = growInterior(r.plan, r.params, r.baseY, &collider);
    const double growMs = msSince(t0);

    Resident res;
    std::size_t tris = 0, bytes = 0;
    for (const RenderMesh& part : bm.parts) {
        if (part.indices.empty()) continue;
        tris += part.indices.size() / 3;
        bytes += part.vertices.size() * sizeof(Vertex) +
                 part.indices.size() * sizeof(uint32_t);
        const PartId pid = static_cast<PartId>(part.materialIndex);
        MeshHandle mh = assets.acquireMesh(part);
        Entity e = world.create();
        Transform t;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable rd;
        rd.mesh = mh;
        rd.material = materialFor(pid, r.params.wallColor);
        rd.drawClass = DrawClass::Structure;
        rd.drawDistance = 300.0;
        world.add<Renderable>(e, rd);
        if (pid == PartId::Interior || pid == PartId::GlassLit)
            world.add<NightGlow>(e, NightGlow{Vec3(1.0, 0.92, 0.78) * 2.0});
        res.entities.push_back(e);
        res.meshes.push_back(mh);
    }

    const auto t1 = std::chrono::steady_clock::now();
    if (phys && !collider.indices.empty()) {
        std::vector<Vec3> verts;
        verts.reserve(collider.vertices.size());
        for (const Vertex& v : collider.vertices) verts.push_back(v.position);
        // Both-ways triangles: a stair must hold from above AND read from
        // below, and quad winding here follows render normals, not physics.
        std::vector<uint32_t> idx = collider.indices;
        const std::size_t oneSided = idx.size();
        idx.reserve(oneSided * 2);
        for (std::size_t i = 0; i + 2 < oneSided; i += 3) {
            idx.push_back(idx[i]);
            idx.push_back(idx[i + 2]);
            idx.push_back(idx[i + 1]);
        }
        res.body = phys->addMesh(verts, idx, Vec3(0, 0, 0), 0.85);
    }
    const double joltMs = msSince(t1);

    resident_[key] = std::move(res);
    LOG_INFO << "[interior] built " << r.recipe << " (" << r.type << ") tris="
             << tris << " grow=" << growMs << "ms jolt=" << joltMs
             << "ms bytes=" << bytes << " resident=" << resident_.size();
}

void BuildingInteriorSystem::release(World& world, PhysicsWorld* phys,
                                     std::size_t key) {
    auto it = resident_.find(key);
    if (it == resident_.end()) return;
    if (phys && it->second.body != INVALID_PHYSICS_BODY)
        phys->removeBody(it->second.body);
    for (Entity e : it->second.entities)
        if (world.alive(e)) world.destroy(e);
    for (MeshHandle mh : it->second.meshes) freeQueue_.push_back(mh);
    resident_.erase(it);
    LOG_INFO << "[interior] released record " << key
             << " resident=" << resident_.size()
             << " freeQueue=" << freeQueue_.size();
}

void BuildingInteriorSystem::onStop(FrameContext& ctx) {
    PhysicsWorld* phys = physics_ ? &physics_->physicsWorld() : nullptr;
    std::vector<std::size_t> keys;
    for (const auto& kv : resident_) keys.push_back(kv.first);
    for (std::size_t key : keys) release(ctx.world, phys, key);
    for (MeshHandle mh : freeQueue_) ctx.assets.releaseMesh(mh);
    freeQueue_.clear();
}

}  // namespace engine
