// Stamping a prefab (#191, step 7). See prefab_stamp.h.
#include "prefab_stamp.h"

#include "../log.h"
#include "asset_manager.h"
#include "model_spawn.h"
#include "scripting/model_recipes.h"
#include "scripting/prefab_list.h"

#include <cstdio>
#include <map>

namespace engine {

namespace {

DrawClass drawClassNamed(const std::string& name) {
    for (int c = 0; c < static_cast<int>(DrawClass::Count); ++c) {
        const char* known = drawClassName(static_cast<DrawClass>(c));
        if (known != nullptr && name == known) return static_cast<DrawClass>(c);
    }
    return DrawClass::Furniture;
}

}  // namespace

void placeModelPieces(World& world, Entity instance) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    const Transform* at = world.get<Transform>(instance);
    if (mi == nullptr || at == nullptr) return;
    Transform unscaled = *at;
    unscaled.scale = Vec3(1, 1, 1);
    const Mat4 owner = unscaled.matrix();
    for (Entity e : mi->pieces) {
        const ModelPiece* piece = world.get<ModelPiece>(e);
        Transform* t = world.get<Transform>(e);
        if (piece == nullptr || t == nullptr) continue;
        *t = transformFromMatrix(owner * piece->local);
        if (Renderable* r = world.get<Renderable>(e)) {
            r->lodAnchored = true;
            r->lodAnchor = at->position;
            r->lodAnchorReach = (t->position - at->position).length();
        }
    }
}

Entity stampPrefab(World& world, ModelDresser* dresser, const Prefab& prefab, const Transform& at, const std::string& levelDir, std::string* error,
                   ModelStore* storeOrNull, const std::string& models) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    std::string why;
    const std::shared_ptr<const ProcModel> model = obtainModel(prefab.model, levelDir, &why, &store, models);
    if (model == nullptr) {
        if (error != nullptr) *error = "prefab '" + prefab.name + "': " + why;
        return Entity();
    }
    // What the model was built from, as one number: a mesh's name to the asset manager carries it, so every copy of
    // one build of the model shares its uploaded meshes, and a rebuilt model does not get the old ones.
    ModelStore::Entry kept;
    store.entry(prefab.model, kept);
    char build[32];
    std::snprintf(build, sizeof build, "%016llx", static_cast<unsigned long long>(kept.signature));
    const std::string keyBase = "model:" + prefab.model + ":" + build + ":";

    const Entity entity = world.create();
    Transform place = at;
    place.scale = Vec3(1, 1, 1);
    if ((at.scale - Vec3(1, 1, 1)).length() > 1e-6) LOG_WARN << "prefab '" << prefab.name << "': a scale is not carried; stamped at its own size";
    world.add<Transform>(entity, place);
    world.add<PrevTransform>(entity, PrevTransform{place});
    ModelInstance instance;
    instance.prefab = prefab.name;
    instance.model = model;

    const DrawClass drawClass = drawClassNamed(prefab.drawClass);
    std::map<const ProcPart*, std::size_t> partNumber;   // in the order parts are first met: the same for every stamp of one build
    for (const ProcModelDraw& d : modelDraws(*model)) {
        const std::size_t number = partNumber.emplace(d.part, partNumber.size()).first->second;
        const Entity e = world.create();
        world.add<Transform>(e, Transform());
        world.add<PrevTransform>(e, PrevTransform{Transform()});
        world.add<ModelPiece>(e, ModelPiece{entity, d.local, d.from, d.until});
        if (dresser != nullptr) {
            Renderable r;
            r.mesh = dresser->dress(d.owner->materialOf(*d.part), d.part->mesh, keyBase + std::to_string(number), r.material);
            r.drawClass = drawClass;
            r.minDistance = d.from;
            // up to the next level; the last level goes as far as the prefab says, or its draw class does
            r.drawDistance = d.until > 0 ? (prefab.drawDistance > 0 ? std::min(d.until, prefab.drawDistance) : d.until) : prefab.drawDistance;
            world.add<Renderable>(e, r);
        }
        instance.pieces.push_back(e);
    }
    if (prefab.body == Prefab::Body::Static) {
        const ProcFrame frame{place.position, place.orientation};
        instance.bodies = spawnModelColliders(world, *model, &frame, prefab.friction);
    }
    world.add<ModelInstance>(entity, std::move(instance));
    placeModelPieces(world, entity);
    // A piece has not moved since the step before: its previous place is its place.
    if (const ModelInstance* mi = world.get<ModelInstance>(entity))
        for (Entity e : mi->pieces)
            if (PrevTransform* prev = world.get<PrevTransform>(e)) prev->value = *world.get<Transform>(e);
    return entity;
}

Entity stampPrefab(World& world, ModelDresser* dresser, const std::string& name, const Transform& at, const std::string& levelDir, std::string* error) {
    Prefab prefab;
    if (!findPrefab(name, prefab, levelDir, error)) return Entity();
    return stampPrefab(world, dresser, prefab, at, levelDir, error);
}

void unstampPrefab(World& world, Entity instance, const std::function<void(PhysicsBodyId)>& removeBody) {
    if (const ModelInstance* mi = world.get<ModelInstance>(instance)) {
        const std::vector<Entity> pieces = mi->pieces, bodies = mi->bodies;
        for (Entity e : pieces)
            if (world.alive(e)) world.destroy(e);
        for (Entity e : bodies) {
            if (!world.alive(e)) continue;
            if (removeBody) {
                if (const RigidBody* rb = world.get<RigidBody>(e); rb != nullptr && rb->bodyId != INVALID_PHYSICS_BODY) removeBody(rb->bodyId);
                if (const MeshCollider* mc = world.get<MeshCollider>(e); mc != nullptr && mc->bodyId != INVALID_PHYSICS_BODY) removeBody(mc->bodyId);
            }
            world.destroy(e);
        }
    }
    if (world.alive(instance)) world.destroy(instance);
}

}  // namespace engine
