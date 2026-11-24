// Motion on a model instance (#191, step 8). See model_motion.h.
#include "model_motion.h"

#include "../log.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
Mat4 frameOf(const Vec3& at, const Quat& turn);   // a place and a turn as one matrix (below)
}  // namespace

namespace {

// The instance's own place: its entity's position and orientation (a scale is not carried).
ProcFrame placeOf(const Transform& t) { return ProcFrame{t.position, t.orientation}; }

}  // namespace

bool setModelTarget(World& world, Entity instance, const std::string& node, Real value, Real speed, std::string* why) {
    ModelInstance* mi = world.get<ModelInstance>(instance);
    auto fail = [&](const std::string& what) {
        if (why != nullptr) *why = what;
        return false;
    };
    if (mi == nullptr || mi->model == nullptr) return fail("not a model instance");
    const ProcNode* joint = mi->model->jointAt(node);
    if (joint == nullptr) return fail("the model has no moving node named '" + node + "'");
    const Real stopped = joint->motion.clamped(value);
    if (speed <= 0) {
        mi->pose.values[node] = stopped;
        mi->target.values.erase(node);
        mi->speed.erase(node);
        poseModelPieces(world, instance);
        return true;
    }
    mi->target.values[node] = stopped;
    mi->speed[node] = speed;
    return true;
}

Real modelValue(World& world, Entity instance, const std::string& node) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    return mi != nullptr ? mi->pose.of(node) : Real(0);
}

bool advanceModelMotion(World& world, Entity instance, Real dt) {
    ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr || mi->target.values.empty()) return false;
    bool moved = false;
    for (std::map<std::string, Real>::iterator it = mi->target.values.begin(); it != mi->target.values.end();) {
        const Real now = mi->pose.of(it->first);
        const Real step = mi->speed[it->first] * dt;
        const Real gap = it->second - now;
        if (std::abs(gap) <= step) {            // there: the target is met and forgotten
            if (gap != 0) { mi->pose.values[it->first] = it->second; moved = true; }
            mi->speed.erase(it->first);
            it = mi->target.values.erase(it);
            continue;
        }
        mi->pose.values[it->first] = now + (gap > 0 ? step : -step);
        moved = true;
        ++it;
    }
    if (moved) poseModelPieces(world, instance);
    return moved;
}

void poseModelPieces(World& world, Entity instance) {
    ModelInstance* mi = world.get<ModelInstance>(instance);
    const Transform* at = world.get<Transform>(instance);
    if (mi == nullptr || at == nullptr || mi->model == nullptr) return;
    mi->settle = true;   // (ModelMotionSystem: the pieces' previous place follows on the next step)
    const Mat4 owner = placeOf(*at).matrix();
    // The draws come in the same order in any pose (ProcModelDraw), so piece i is draw i. A model whose draws
    // merge differently when posed (two uses that coincide only at rest) keeps its pieces where they were.
    const bool posed = !mi->pose.values.empty();
    std::vector<ProcModelDraw> draws;
    if (posed) {
        draws = modelDraws(*mi->model, &mi->pose);
        if (draws.size() != mi->pieces.size()) {
            LOG_WARN << "model instance of '" << mi->prefab << "': its pose changes how many pieces it draws; pieces left as they were";
            draws.clear();
        }
    }
    for (std::size_t i = 0; i < mi->pieces.size(); ++i) {
        const Entity e = mi->pieces[i];
        ModelPiece* piece = world.get<ModelPiece>(e);
        Transform* t = world.get<Transform>(e);
        if (piece == nullptr || t == nullptr) continue;
        if (!draws.empty()) piece->local = draws[i].local;
        *t = transformFromMatrix(owner * piece->local);
        if (Renderable* r = world.get<Renderable>(e)) {
            r->lodAnchored = true;
            r->lodAnchor = at->position;
            r->lodAnchorReach = (t->position - at->position).length();
        }
    }
    for (const ModelInstance::BatchRider& ride : mi->batchRiders) {   // the copies in a batch: one transform each
        InstanceGroup* g = world.alive(ride.group) ? world.get<InstanceGroup>(ride.group) : nullptr;
        Vec3 markerAt;
        Quat markerTurn;
        if (g == nullptr || ride.index >= g->transforms.size() || !modelMarkerWorld(world, instance, ride.marker, markerAt, markerTurn)) continue;
        g->transforms[ride.index] = frameOf(markerAt, markerTurn) * ride.local;
    }
    // WHAT STANDS ON IT goes with it (rideMarker). (A copy of the list: placing a rider poses it, and a rider that
    // has gone is dropped from the list.)
    if (!mi->riders.empty()) {
        const std::vector<Entity> riders = mi->riders;
        bool gone = false;
        for (Entity rider : riders) {
            const RidesMarker* ride = world.alive(rider) ? world.get<RidesMarker>(rider) : nullptr;
            Transform* rt = ride != nullptr ? world.get<Transform>(rider) : nullptr;
            if (rt == nullptr) { gone = true; continue; }
            Vec3 markerAt;
            Quat markerTurn;
            if (!modelMarkerWorld(world, instance, ride->marker, markerAt, markerTurn)) continue;
            rt->position = markerAt + markerTurn.rotate(ride->localPosition);
            rt->orientation = markerTurn * ride->localRotation;
            if (world.has<ModelInstance>(rider)) poseModelPieces(world, rider);
        }
        if (gone) {
            ModelInstance* mine = world.get<ModelInstance>(instance);
            mine->riders.erase(std::remove_if(mine->riders.begin(), mine->riders.end(),
                                              [&](Entity r) { return !world.alive(r) || !world.has<RidesMarker>(r); }),
                               mine->riders.end());
        }
    }
}

bool rideMarker(World& world, Entity rider, Entity owner, const std::string& marker) {
    Vec3 at;
    Quat turn;
    Transform* t = world.get<Transform>(rider);
    ModelInstance* mi = world.get<ModelInstance>(owner);
    if (t == nullptr || mi == nullptr || rider == owner || !modelMarkerWorld(world, owner, marker, at, turn)) return false;
    unrideMarker(world, rider);
    const Quat back = turn.inverse();
    RidesMarker ride;
    ride.owner = owner;
    ride.marker = marker;
    ride.localPosition = back.rotate(t->position - at);
    ride.localRotation = back * t->orientation;
    world.add<RidesMarker>(rider, ride);
    world.get<ModelInstance>(owner)->riders.push_back(rider);
    return true;
}

namespace {
Mat4 frameOf(const Vec3& at, const Quat& turn) {
    Transform t;
    t.position = at;
    t.orientation = turn;
    return t.matrix();
}
}  // namespace

bool rideMarkerInBatch(World& world, Entity owner, const std::string& marker, Entity group, uint32_t index) {
    Vec3 at;
    Quat turn;
    ModelInstance* mi = world.get<ModelInstance>(owner);
    const InstanceGroup* g = world.get<InstanceGroup>(group);
    if (mi == nullptr || g == nullptr || index >= g->transforms.size() || !modelMarkerWorld(world, owner, marker, at, turn)) return false;
    ModelInstance::BatchRider ride;
    ride.group = group;
    ride.index = index;
    ride.marker = marker;
    ride.local = frameOf(at, turn).inverse() * g->transforms[index];
    world.get<ModelInstance>(owner)->batchRiders.push_back(ride);
    return true;
}

void unrideMarker(World& world, Entity rider) {
    const RidesMarker* ride = world.get<RidesMarker>(rider);
    if (ride == nullptr) return;
    if (ModelInstance* mi = world.alive(ride->owner) ? world.get<ModelInstance>(ride->owner) : nullptr)
        mi->riders.erase(std::remove(mi->riders.begin(), mi->riders.end(), rider), mi->riders.end());
    world.remove<RidesMarker>(rider);
}

bool modelBodyPlace(World& world, Entity body, Vec3& position, Quat& rotation) {
    const ModelShape* shape = world.get<ModelShape>(body);
    if (shape == nullptr) return false;
    const ModelInstance* mi = world.get<ModelInstance>(shape->owner);
    const Transform* at = world.get<Transform>(shape->owner);
    if (mi == nullptr || at == nullptr || mi->model == nullptr) return false;
    if (shape->collider < 0 || shape->collider >= static_cast<int>(mi->model->colliders.size())) return false;
    const ProcCollider& pc = mi->model->colliders[static_cast<std::size_t>(shape->collider)];
    const ProcFrame placed = placeOf(*at) * (pc.node == 0 ? pc.frame() : mi->model->nodeFrame(pc.node, mi->pose) * pc.frame());
    position = placed.position;
    rotation = placed.rotation;
    return true;
}

bool modelNodeWorld(World& world, Entity instance, const std::string& node, Vec3& position, Quat& rotation) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    const Transform* at = world.get<Transform>(instance);
    if (mi == nullptr || at == nullptr || mi->model == nullptr) return false;
    const int index = mi->model->findNode(node);
    if (index < 0) return false;
    const ProcFrame placed = placeOf(*at) * mi->model->nodeFrame(index, mi->pose);
    position = placed.position;
    rotation = placed.rotation;
    return true;
}

bool modelMarkerWorld(World& world, Entity instance, const std::string& marker, Vec3& position, Quat& rotation) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    const Transform* at = world.get<Transform>(instance);
    if (mi == nullptr || at == nullptr || mi->model == nullptr) return false;
    int node = 0;
    const ProcMarker* mk = mi->model->findMarker(marker, &node);
    if (mk == nullptr) return false;
    const ProcFrame placed = placeOf(*at) * mi->model->nodeFrame(node, mi->pose) * mk->frame;
    position = placed.position;
    rotation = placed.rotation;
    return true;
}

}  // namespace engine
