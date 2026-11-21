// Stamping a prefab (#191, step 7). See prefab_stamp.h.
#include "prefab_stamp.h"

#include "../log.h"
#include "asset_manager.h"
#include "model_motion.h"
#include "model_spawn.h"
#include "prefab_use.h"
#include "script_assets.h"
#include "scripting/model_recipes.h"
#include "scripting/script_behaviour.h"
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

Entity stampPrefab(World& world, ModelDresser* dresser, const Prefab& prefab, const Transform& at, const std::string& levelDir, std::string* error,
                   ModelStore* storeOrNull, const std::string& models) {
    ModelStore& store = storeOrNull != nullptr ? *storeOrNull : modelStore();
    std::string why;
    const bool direct = !prefab.recipe.empty();
    const std::string modelName = direct ? recipeModelName(prefab.recipe, prefab.recipeArgs) : prefab.model;
    const std::shared_ptr<const ProcModel> model = direct ? obtainRecipeModel(prefab.recipe, prefab.recipeArgs, levelDir, &why, &store, models)
                                                          : obtainModel(prefab.model, levelDir, &why, &store, models);
    std::shared_ptr<const ProcModel> shown = model;
    if (shown == nullptr) {
        if (error != nullptr) *error = "prefab '" + prefab.name + "': " + why;
        // A RECIPE BEING WRITTEN (the model studio) that has a mistake in it: the last model it built is still in
        // the store, and is shown until the mistake is mended, so the thing being worked on does not vanish at every
        // slip. `error` still says what is wrong. (A prefab from the list has no such grace: a broken shipped
        // model should be seen to be missing.)
        if (direct) shown = store.find(modelName);
        if (shown == nullptr) return Entity();
    }
    // What the model was built from, as one number: a mesh's name to the asset manager carries it, so every copy of
    // one build of the model shares its uploaded meshes, and a rebuilt model does not get the old ones.
    ModelStore::Entry kept;
    store.entry(modelName, kept);
    char build[32];
    std::snprintf(build, sizeof build, "%016llx", static_cast<unsigned long long>(kept.signature));
    const std::string keyBase = "model:" + modelName + ":" + build + ":";

    const Entity entity = world.create();
    Transform place = at;
    place.scale = Vec3(1, 1, 1);
    if ((at.scale - Vec3(1, 1, 1)).length() > 1e-6) LOG_WARN << "prefab '" << prefab.name << "': a scale is not carried; stamped at its own size";
    world.add<Transform>(entity, place);
    world.add<PrevTransform>(entity, PrevTransform{place});
    ModelInstance instance;
    instance.prefab = prefab.name;
    instance.model = shown;

    const DrawClass drawClass = drawClassNamed(prefab.drawClass);
    std::map<const ProcPart*, std::size_t> partNumber;   // in the order parts are first met: the same for every stamp of one build
    for (const ProcModelDraw& d : modelDraws(*shown)) {
        const std::size_t number = partNumber.emplace(d.part, partNumber.size()).first->second;
        const Entity e = world.create();
        world.add<Transform>(e, Transform());
        world.add<PrevTransform>(e, PrevTransform{Transform()});
        ModelPiece piece{entity, d.local, d.from, d.until, d.line, std::string()};
        {
            const std::string& material = d.owner->materials[static_cast<std::size_t>(d.part->material)].name;
            const std::string& node = d.owner->nodes[static_cast<std::size_t>(d.part->node)].name;
            piece.what = (material.empty() ? std::string("plain") : material) + " on " + node + ", " + std::to_string(d.part->mesh.indices.size() / 3) + " triangles";
            if (d.owner != shown.get()) piece.what += d.owner->name.empty() ? " (a used piece)" : " (the piece '" + d.owner->name + "')";
            if (d.from > 0) piece.what += ", the level from " + std::to_string(static_cast<int>(d.from)) + " m";
        }
        world.add<ModelPiece>(e, piece);
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
        // A box, sphere or capsule on a node that can move is a kinematic body: ModelMotionSystem carries it with
        // its node. Each such body says which of the model's shapes it is (ModelShape).
        std::vector<int> colliderOf;
        instance.bodies = spawnModelColliders(world, *shown, &frame, prefab.friction, /*movingAreKinematic=*/true, &colliderOf);
        for (std::size_t i = 0; i < instance.bodies.size(); ++i)
            if (colliderOf[i] >= 0) world.add<ModelShape>(instance.bodies[i], ModelShape{entity, colliderOf[i]});
    }
    world.add<ModelInstance>(entity, std::move(instance));
#ifdef RT_ENABLE_SCRIPTING
    if (!prefab.script.empty()) {
        ScriptBehaviour behaviour;
        behaviour.source = loadScriptCode(prefab.script, levelDir);
        if (behaviour.source.empty()) LOG_WARN << "prefab '" << prefab.name << "': the script '" << prefab.script << "' cannot be read";
        else world.add<ScriptBehaviour>(entity, behaviour);
    }
#endif
    {
        std::vector<std::string> skipped;
        addPrefabUses(world, entity, prefab, &skipped);
        for (const std::string& why : skipped) LOG_WARN << "prefab '" << prefab.name << "': use " << why << " (left out)";
    }
    poseModelPieces(world, entity);
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
