#include "prefab_batch.h"

#include "../log.h"
#include "model_spawn.h"
#include "prefab_stamp.h"   // drawClassNamed
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
    world.get<PrefabBatches>(holder)->copies.push_back(std::move(copy));
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

void releaseBatches(World& world, Entity holder) {
    PrefabBatches* b = world.get<PrefabBatches>(holder);
    if (b == nullptr) return;
    for (const PrefabBatches::Group& g : b->groups)
        if (world.alive(g.entity)) world.destroy(g.entity);
    world.remove<PrefabBatches>(holder);
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
