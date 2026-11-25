#include "prefab_batch.h"

#include "../log.h"
#include "model_spawn.h"
#include "model_motion.h"
#include "prefab_stamp.h"   // drawClassNamed, stampPrefab: a copy awake
#include "prefab_use.h"
#include "scripting/model_recipes.h"

#include <algorithm>
#include <cstdio>

namespace engine {

namespace {

// A matrix that draws nothing: a copy taken out of the picture keeps its place in its groups.
Mat4 nowhere(const Vec3& at) {
    Mat4 m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) m.m[r][c] = 0;
    m.m[0][3] = at.x; m.m[1][3] = at.y; m.m[2][3] = at.z; m.m[3][3] = 1;
    return m;
}

Mat4 matrixOf(const Transform& at) {
    Transform t = at;
    t.scale = Vec3(1, 1, 1);
    return t.matrix();
}

void grow(InstanceGroup& g, const Mat4& m) {
    const Vec3 o(m.m[0][3], m.m[1][3], m.m[2][3]);
    if (g.transforms.size() <= 1) {
        g.boundsCenter = o;
        g.boundsRadius = 3.0;
        return;
    }
    const Real reach = (o - g.boundsCenter).length() + 3.0;
    if (reach > g.boundsRadius) g.boundsRadius = reach;
}

// Write a copy's transforms from its pose (or the nothing-matrix when it is not shown).
void place(World& world, PrefabBatches& b, PrefabBatches::Copy& c) {
    const Mat4 owner = matrixOf(c.at);
    const std::vector<ProcModelDraw> draws = modelDraws(*c.model, c.pose.values.empty() ? nullptr : &c.pose);
    if (draws.size() != c.draws.size()) {
        LOG_WARN << "batch copy of '" << c.prefab << "': its pose changes how many things it draws; left as it was";
        return;
    }
    for (std::size_t i = 0; i < draws.size(); ++i) {
        InstanceGroup* g = world.get<InstanceGroup>(b.groups[c.draws[i].first].entity);
        if (g == nullptr || c.draws[i].second >= g->transforms.size()) continue;
        g->transforms[c.draws[i].second] = c.shown ? owner * draws[i].local : nowhere(c.at.position);
        if (c.shown) grow(*g, g->transforms[c.draws[i].second]);
    }
}

}  // namespace

uint32_t batchPrefab(World& world, Entity holder, ModelDresser* dresser, const Prefab& prefab, const Transform& at,
                     const std::string& levelDir, std::string* error, ModelStore* storeOrNull, const std::string& models) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    std::string why;
    const bool direct = !prefab.recipe.empty();
    const std::string modelName = direct ? recipeModelName(prefab.recipe, prefab.recipeArgs) : prefab.model;
    const std::shared_ptr<const ProcModel> model = direct ? obtainRecipeModel(prefab.recipe, prefab.recipeArgs, levelDir, &why, &store, models)
                                                          : obtainModel(prefab.model, levelDir, &why, &store, models);
    if (model == nullptr) {
        if (error != nullptr) *error = "prefab '" + prefab.name + "': " + why;
        return kNoBatchCopy;
    }
    if (!world.has<PrefabBatches>(holder)) world.add<PrefabBatches>(holder, PrefabBatches{});
    // (the same mesh names a stamped copy uses, so both draw from one upload: prefab_stamp.cpp)
    ModelStore::Entry kept;
    store.entry(modelName, kept);
    char build[32];
    std::snprintf(build, sizeof build, "%016llx", static_cast<unsigned long long>(kept.signature));
    const std::string keyBase = "model:" + modelName + ":" + build + ":";
    const DrawClass drawClass = drawClassNamed(prefab.drawClass);

    PrefabBatches::Copy copy;
    copy.prefab = prefab.name;
    copy.model = model;
    copy.at = at;
    const Mat4 owner = matrixOf(at);
    std::map<const ProcPart*, std::size_t> partNumber;
    const uint32_t index = static_cast<uint32_t>(world.get<PrefabBatches>(holder)->copies.size());
    for (const ProcModelDraw& d : modelDraws(*model)) {
        const std::size_t number = partNumber.emplace(d.part, partNumber.size()).first->second;
        const Real until = d.until > 0 ? (prefab.drawDistance > 0 ? std::min(d.until, prefab.drawDistance) : d.until) : prefab.drawDistance;
        char band[64];
        std::snprintf(band, sizeof band, "|%.2f|%.2f|%d", static_cast<double>(d.from), static_cast<double>(until), static_cast<int>(drawClass));
        const std::string key = keyBase + std::to_string(number) + band;
        PrefabBatches* b = world.get<PrefabBatches>(holder);
        std::map<std::string, uint32_t>::const_iterator had = b->groupOf.find(key);
        uint32_t gi = 0;
        if (had == b->groupOf.end()) {
            InstanceGroup g;
            if (dresser != nullptr) g.mesh = dresser->dress(d.owner->materialOf(*d.part), d.part->mesh, keyBase + std::to_string(number), g.material);
            g.drawClass = drawClass;
            g.minDistance = d.from;
            g.drawDistance = until;
            const Entity ge = world.create();
            world.add<InstanceGroup>(ge, std::move(g));
            b = world.get<PrefabBatches>(holder);   // (creating an entity may have moved the pool)
            gi = static_cast<uint32_t>(b->groups.size());
            b->groups.push_back(PrefabBatches::Group{ge, {}});
            b->groupOf[key] = gi;
        } else {
            gi = had->second;
        }
        InstanceGroup* g = world.get<InstanceGroup>(b->groups[gi].entity);
        const uint32_t ti = static_cast<uint32_t>(g->transforms.size());
        g->transforms.push_back(owner * d.local);
        grow(*g, g->transforms.back());
        b->groups[gi].copy.push_back(index);
        copy.draws.emplace_back(gi, ti);
    }
    copy.line = prefab;
    copy.uses = prefabUsesFor(*model, prefab);
    copy.on.assign(copy.uses.size(), 0);
    copy.lively = !prefab.script.empty() || !prefab.free.empty();   // (a door is made lively by whoever hangs it)
    for (const PrefabUse& u : copy.uses) copy.lively = copy.lively || u.near > 0;
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b->copies.empty()) { b->boundsCenter = at.position; b->boundsRadius = 2; }
    else {
        // (the ball is moved toward the new copy and grown to hold both)
        const Vec3 to = at.position - b->boundsCenter;
        const Real d = to.length();
        if (d + 2 > b->boundsRadius) {
            const Real r = (b->boundsRadius + d + 2) * Real(0.5);
            if (d > 1e-9) b->boundsCenter = b->boundsCenter + to * ((r - b->boundsRadius) / d);
            b->boundsRadius = r;
        }
    }
    b->mayLight = b->mayLight || !prefab.lights.empty();
    for (const PrefabUse& u : copy.uses) b->mayLight = b->mayLight || !u.light.empty();
    b->mayStir = b->mayStir || copy.lively;
    b->copies.push_back(std::move(copy));
    b->fixedStale = b->restingStale = true;
    return index;
}

void poseBatchCopy(World& world, Entity holder, uint32_t copy, const ProcPose& pose) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr || copy >= b->copies.size()) return;
    b->copies[copy].pose = pose;
    place(world, *b, b->copies[copy]);
}

void showBatchCopy(World& world, Entity holder, uint32_t copy, bool shown) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr || copy >= b->copies.size() || b->copies[copy].shown == shown) return;
    b->copies[copy].shown = shown;
    place(world, *b, b->copies[copy]);
}

void releaseBatches(World& world, Entity holder, const std::function<void(PhysicsBodyId)>& removeBody) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr) return;
    for (const PrefabBatches::Copy& c : std::vector<PrefabBatches::Copy>(b->copies))
        if (c.awake.valid() && world.alive(c.awake)) unstampPrefab(world, c.awake, removeBody);
    b = world.get<PrefabBatches>(holder);
    if (removeBody) {
        if (b->fixedBody != INVALID_PHYSICS_BODY) removeBody(b->fixedBody);
        if (b->restingBody != INVALID_PHYSICS_BODY) removeBody(b->restingBody);
    }
    for (const PrefabBatches::Group& g : b->groups)
        if (world.alive(g.entity)) world.destroy(g.entity);
    world.remove<PrefabBatches>(holder);
}

void removeBatchCopy(World& world, Entity holder, uint32_t copy, const std::function<void(PhysicsBodyId)>& removeBody) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr || copy >= b->copies.size() || b->copies[copy].gone) return;
    const Entity awake = b->copies[copy].awake;
    if (awake.valid() && world.alive(awake)) {
        world.remove<BatchAwake>(awake);   // (it does not go back to the batch)
        unstampPrefab(world, awake, removeBody);
    }
    b = world.get<PrefabBatches>(holder);
    b->copies[copy].awake = Entity();
    b->copies[copy].gone = true;
    b->copies[copy].lively = false;
    b->copies[copy].busy = 0;
    b->restingStale = b->fixedStale = true;
    b->copies[copy].shown = true;   // (so hiding writes its transforms)
    showBatchCopy(world, holder, copy, false);
}

bool batchCopyMarker(const PrefabBatches::Copy& copy, const std::string& marker, Vec3& position, Quat& rotation) {
    if (copy.model == nullptr) return false;
    int node = 0;
    const ProcMarker* mk = copy.model->findMarker(marker, &node);
    if (mk == nullptr) return false;
    const ProcFrame placed = ProcFrame{copy.at.position, copy.at.orientation} * copy.model->nodeFrame(node, copy.pose) * mk->frame;
    position = placed.position;
    rotation = placed.rotation;
    return true;
}

void batchCollisionTriangles(const PrefabBatches& batches, bool moving, std::vector<Vec3>& vertices, std::vector<uint32_t>& indices) {
    auto triangle = [&](uint32_t a, uint32_t b, uint32_t c) {
        indices.push_back(a); indices.push_back(b); indices.push_back(c);
        indices.push_back(a); indices.push_back(c); indices.push_back(b);
    };
    for (const PrefabBatches::Copy& c : batches.copies) {
        if (c.model == nullptr || c.gone || c.line.body != Prefab::Body::Static) continue;
        if (moving && c.awake.valid()) continue;   // (its moving shapes are its own bodies while it is awake)
        const ProcFrame at{c.at.position, c.at.orientation};
        for (const ProcCollider& pc : c.model->colliders) {
            if (pc.dynamic || c.model->nodeMoves(pc.node) != moving) continue;
            const ProcFrame frame = at * (pc.node == 0 ? pc.frame() : c.model->nodeFrame(pc.node, c.pose) * pc.frame());
            const uint32_t first = static_cast<uint32_t>(vertices.size());
            if (pc.kind == ProcCollider::Kind::Mesh || pc.kind == ProcCollider::Kind::Hull) {
                // (a mesh's points are in its node's own space; a hull's are about its centre)
                const ProcFrame of = pc.kind == ProcCollider::Kind::Mesh ? at * (pc.node == 0 ? ProcFrame{} : c.model->nodeFrame(pc.node, c.pose)) : frame;
                for (const Vertex& v : pc.mesh.vertices) vertices.push_back(of.position + of.rotation.rotate(v.position));
                for (std::size_t k = 0; k + 2 < pc.mesh.indices.size(); k += 3)
                    triangle(first + pc.mesh.indices[k], first + pc.mesh.indices[k + 1], first + pc.mesh.indices[k + 2]);
                continue;
            }
            // a box; a sphere or a capsule as the box round it (a thing at rest: nobody rolls off its corners)
            Vec3 h = pc.halfExtent;
            if (pc.kind == ProcCollider::Kind::Sphere) h = Vec3(pc.radius, pc.radius, pc.radius);
            if (pc.kind == ProcCollider::Kind::Capsule) h = Vec3(pc.radius, pc.halfHeight + pc.radius, pc.radius);
            for (int k = 0; k < 8; ++k)
                vertices.push_back(frame.position + frame.rotation.rotate(Vec3((k & 1) ? h.x : -h.x, (k & 2) ? h.y : -h.y, (k & 4) ? h.z : -h.z)));
            static const int kFaces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
            for (const int* f : kFaces) {
                triangle(first + f[0], first + f[1], first + f[2]);
                triangle(first + f[0], first + f[2], first + f[3]);
            }
        }
    }
}

Entity wakeBatchCopy(World& world, Entity holder, uint32_t copy) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr || copy >= b->copies.size() || b->copies[copy].gone) return Entity();
    if (b->copies[copy].awake.valid() && world.alive(b->copies[copy].awake)) return b->copies[copy].awake;
    const PrefabBatches::Copy c = b->copies[copy];   // (a copy: stamping makes entities, and the list may move)
    std::string why;
    const Entity e = stampPrefab(world, b->dresser.get(), c.line, c.at, std::string(), &why);
    if (!e.valid()) {
        LOG_WARN << "batch copy of '" << c.prefab << "' could not wake: " << why;
        return Entity();
    }
    // in the pose it was drawn in, with the uses that were on still on (nothing is done afresh: no sound, no motion)
    if (ModelInstance* mi = world.get<ModelInstance>(e)) {
        mi->pose = c.pose;
        mi->target = c.pose;
    }
    poseModelPieces(world, e);
    for (Entity pe : world.get<ModelInstance>(e)->pieces)
        if (PrevTransform* prev = world.get<PrevTransform>(pe))
            if (const Transform* t = world.get<Transform>(pe)) prev->value = *t;
    if (world.has<PrefabUses>(e))
        for (std::size_t i = 0; i < c.on.size(); ++i) {
            PrefabUses* uses = world.get<PrefabUses>(e);
            if (i >= uses->uses.size() || c.on[i] == 0) continue;
            // (a use that runs or sways while it is on starts doing so: a rocking chair someone is in rocks)
            if (!uses->uses[i].use.run.empty() || !uses->uses[i].use.sway.empty()) setPrefabUse(world, e, i, true);
            else uses->uses[i].on = true;
        }
    if (c.swings) world.add<SwingDoor>(e, SwingDoor{c.swingFoot, c.swingNormal, c.at.position.y});
    world.add<BatchAwake>(e, BatchAwake{holder, copy, 0});
    for (const PrefabBatches::Copy::GroupRider& r : c.groupRiders)
        if (world.alive(r.group)) rideMarkerInBatch(world, e, r.marker, r.group, r.index);
    b = world.get<PrefabBatches>(holder);
    b->copies[copy].awake = e;
    b->restingStale = true;
    showBatchCopy(world, holder, copy, false);
    // what stands on it as a copy of its own wakes with it, and goes where it goes
    for (const std::pair<uint32_t, std::string>& r : c.riders) {
        const Entity rider = wakeBatchCopy(world, holder, r.first);
        if (rider.valid()) rideMarker(world, rider, e, r.second);
    }
    return e;
}

bool batchUseOn(World& world, const PrefabBatches::Copy& copy, std::size_t use) {
    if (copy.awake.valid() && world.alive(copy.awake))
        if (const PrefabUses* uses = world.get<PrefabUses>(copy.awake)) return use < uses->uses.size() && uses->uses[use].on;
    return use < copy.on.size() && copy.on[use] != 0;
}

void batchSpots(World& world, Entity holder, std::vector<BatchSpot>& out, const std::vector<std::pair<uint32_t, uint32_t>>* mine) {
    const PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr) return;
    for (std::size_t ci = 0; ci < b->copies.size(); ++ci) {
        const PrefabBatches::Copy& c = b->copies[ci];
        if (c.gone) continue;
        for (std::size_t ui = 0; ui < c.uses.size(); ++ui) {
            const PrefabUse& use = c.uses[ui];
            if (!use.sit && !use.lie) continue;
            if (batchUseOn(world, c, ui)) {
                bool ours = false;
                if (mine != nullptr)
                    for (const std::pair<uint32_t, uint32_t>& m : *mine) ours = ours || (m.first == ci && m.second == ui);
                if (!ours) continue;
            }
            Vec3 at;
            Quat turn;
            if (!batchCopyMarker(c, use.at, at, turn)) continue;
            Vec3 facing = turn.rotate(Vec3(0, 0, 1));
            facing.y = 0;
            facing = facing.lengthSquared() > 1e-8 ? normalize(facing) : Vec3(0, 0, 1);
            const Real yaw = std::atan2(facing.x, facing.z);
            BatchSpot spot;
            spot.copy = static_cast<uint32_t>(ci);
            spot.use = static_cast<uint32_t>(ui);
            spot.unit = c.unit;
            if (use.lie) {
                // on the back: the marker is the head and its +z runs to the feet (a standing body is built about its
                // middle, 0.9 m from its feet; lying, half its depth above what it lies on)
                spot.pose = 1;
                const Vec3 middle = at + facing * Real(0.85) + Vec3(0, 0.13, 0);
                spot.at = Mat4::translate(middle.x, middle.y, middle.z) * Mat4::rotateY(yaw) * Mat4::rotateX(Real(-3.14159265358979323846 * 0.5));
            } else {
                spot.pose = 0;
                spot.at = Mat4::translate(at.x, at.y, at.z) * Mat4::rotateY(yaw);
            }
            out.push_back(spot);
        }
    }
}

void occupyBatchSpot(World& world, Entity holder, uint32_t copy, uint32_t use, bool taken) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr || copy >= b->copies.size() || use >= b->copies[copy].uses.size()) return;
    PrefabBatches::Copy& c = b->copies[copy];
    const bool was = batchUseOn(world, c, use);
    if (was == taken) return;
    if (use < c.on.size()) c.on[use] = taken ? 1 : 0;
    const bool moves = !c.uses[use].run.empty() || !c.uses[use].sway.empty();
    if (moves) c.busy = std::max(0, c.busy + (taken ? 1 : -1));
    if (moves) b->mayStir = true;
    const Entity awake = c.awake;
    if (awake.valid() && world.alive(awake)) setPrefabUse(world, awake, use, taken);
}

bool batchCopySettled(World& world, Entity awake) {
    const ModelInstance* mi = world.get<ModelInstance>(awake);
    if (mi == nullptr) return true;
    for (const std::pair<const std::string, Real>& sp : mi->speed)
        if (sp.second > 0) {
            const std::map<std::string, Real>::const_iterator want = mi->target.values.find(sp.first), is = mi->pose.values.find(sp.first);
            const Real to = want != mi->target.values.end() ? want->second : 0, at = is != mi->pose.values.end() ? is->second : 0;
            if (std::fabs(to - at) > Real(1e-6)) return false;
        }
    if (const PrefabUses* uses = world.get<PrefabUses>(awake))
        for (const PrefabUses::Use& u : uses->uses)
            if (u.moving || u.coasting > 0 || u.left > 0 || (u.on && (!u.use.run.empty() || !u.use.sway.empty() || u.use.sit || u.use.lie))) return false;
    if (const FreeTurns* turns = world.get<FreeTurns>(awake))
        for (const FreeTurns::Part& part : turns->parts)
            if (part.speed != 0) return false;
    return true;
}

void sleepBatchCopy(World& world, Entity awake, const std::function<void(PhysicsBodyId)>& removeBody) {
    const BatchAwake* from = world.get<BatchAwake>(awake);
    if (from == nullptr) return;
    const Entity holder = from->holder;
    const uint32_t copy = from->copy;
    PrefabBatches* b = world.alive(holder) ? world.get<PrefabBatches>(holder) : nullptr;
    if (b != nullptr && copy < b->copies.size()) {
        // what stood on it as a copy of its own goes back first, where it now is
        for (const std::pair<uint32_t, std::string>& r : std::vector<std::pair<uint32_t, std::string>>(b->copies[copy].riders)) {
            const Entity rider = world.get<PrefabBatches>(holder)->copies[r.first].awake;
            if (rider.valid() && world.alive(rider)) sleepBatchCopy(world, rider, removeBody);
        }
        b = world.get<PrefabBatches>(holder);
        PrefabBatches::Copy& c = b->copies[copy];
        if (const ModelInstance* mi = world.get<ModelInstance>(awake)) c.pose = mi->pose;
        if (const Transform* t = world.get<Transform>(awake)) c.at = *t;
        if (const PrefabUses* uses = world.get<PrefabUses>(awake))
            for (std::size_t i = 0; i < uses->uses.size() && i < c.on.size(); ++i) c.on[i] = uses->uses[i].on ? 1 : 0;
        c.awake = Entity();
        c.shown = false;   // (so showing it writes its transforms afresh, in the pose it was left in)
        b->restingStale = true;
    }
    unstampPrefab(world, awake, removeBody);
    if (world.alive(holder) && world.has<PrefabBatches>(holder)) showBatchCopy(world, holder, copy, true);
}

BatchCount countBatches(World& world, Entity holder) {
    BatchCount n;
    const PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr) return n;
    n.copies = b->copies.size();
    n.groups = b->groups.size();
    for (const PrefabBatches::Group& g : b->groups) n.transforms += g.copy.size();
    return n;
}

}  // namespace engine
