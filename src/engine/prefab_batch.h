#ifndef RAYTRACER_ENGINE_PREFAB_BATCH_H
#define RAYTRACER_ENGINE_PREFAB_BATCH_H

// PREFAB COPIES DRAWN IN BATCHES (#315 step 10, part a; docs/model-record-design.md "How many copies would work
// near and far"). A stamped prefab (prefab_stamp.h) is an entity, an entity for each thing it draws and a body for
// each collision shape: a drinks fridge is 566 entities. Most copies of a thing are only ever looked at. A BATCH
// COPY is the same prefab at a place in a pose with no entity of its own: each thing its model draws is one
// transform in an instance group shared by every copy of that model in the same holder.
//
// A HOLDER is whoever owns a set of copies and lets them all go at once: a building's interior while it is
// streamed in, a tile of the city. It is an entity with a PrefabBatches; the groups are its too.
//
// One model, one pose record: a copy's transforms come from the same `modelDraws` a stamped copy's pieces come
// from, in the same order, so a copy that is later stamped in full (part c: it wakes) stands exactly where the
// batch drew it.
//
// This part draws. A batch copy cannot yet be used or walked into (parts b and c).

#include "components.h"
#include "procgen/model_store.h"
#include "procgen/prefab.h"
#include "world.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace engine {

class ModelDresser;

// On a holder: its groups and its copies.
struct PrefabBatches {
    struct Group {
        Entity entity;                 // has the InstanceGroup
        std::vector<uint32_t> copy;    // for each of its transforms, the copy it draws
    };
    struct Copy {
        std::string prefab;                      // its line in prefabs.lua
        std::shared_ptr<const ProcModel> model;
        Transform at;
        ProcPose pose;                           // each moving node's value (none given: at rest)
        // where each thing it draws is: a group and one of that group's transforms, in the model's draw order
        std::vector<std::pair<uint32_t, uint32_t>> draws;
        bool shown = true;                       // false: taken out of the picture (it has woken, part c)
    };
    std::vector<Group> groups;
    std::map<std::string, uint32_t> groupOf;     // by mesh key, level band and draw class
    std::vector<Copy> copies;
};

constexpr uint32_t kNoBatchCopy = 0xffffffffu;

// One more copy of `prefab` at `at`, drawn in `holder`'s batches. Returns which copy it is (an index into
// PrefabBatches::copies), or kNoBatchCopy with `error` when the prefab's model cannot be had. `dresser` gives the
// groups their meshes and materials; null makes groups with nothing to draw (a headless test). The holder gets a
// PrefabBatches if it has none.
uint32_t batchPrefab(World& world, Entity holder, ModelDresser* dresser, const Prefab& prefab, const Transform& at,
                     const std::string& levelDir = std::string(), std::string* error = nullptr, ModelStore* store = nullptr,
                     const std::string& models = "models.lua");

// Put a copy in a pose: every thing it draws is moved to where the model has it in that pose.
void poseBatchCopy(World& world, Entity holder, uint32_t copy, const ProcPose& pose);

// Take a copy out of the picture, or put it back (where its pose has it). It stays in the list either way.
void showBatchCopy(World& world, Entity holder, uint32_t copy, bool shown);

// Let every copy and group of a holder go. (The holder entity itself is the caller's.)
void releaseBatches(World& world, Entity holder);

// How many entities and transforms a holder's batches come to, for a measurement.
struct BatchCount { std::size_t copies = 0, groups = 0, transforms = 0; };
BatchCount countBatches(World& world, Entity holder);

}  // namespace engine

#endif
