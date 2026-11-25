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
// READY (part b): a copy at rest is solid and offers its uses with no entity. Its collision is its model's shapes as
// triangles, merged with every other copy's in the holder (batchCollisionTriangles: whoever has the physics makes
// the two bodies, one for shapes that never move and one for shapes on parts that can, which is made again when
// a copy wakes or sleeps). Its offers are found from its model's markers and its place (batchCopyMarker).
//
// AWAKE (part c): a copy someone uses, or a lively one (a script, a part that turns freely, a use that happens as
// someone comes near, a door that swings) with the player close, is STAMPED in full at the pose the batch drew it
// in and taken out of the picture; when it has been still a while it is put back in the pose it was left in, with
// the uses that were on still on. Nothing is remembered once the holder lets go: a floor comes back as placed.

#include "components.h"
#include "procgen/model_store.h"
#include "procgen/prefab.h"
#include "world.h"

#include <functional>
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
        Prefab line;                             // the prefab as it was placed (a door's has its opening's size in it)
        std::vector<PrefabUse> uses;             // prefabUsesFor: the uses a stamped copy would have, in its order
        std::vector<uint8_t> on;                 // which of them are on (kept while it sleeps)
        Entity awake;                            // the stamped copy while it is awake
        bool lively = false;                     // it does something by itself: awake whenever the player is close
        // A DOOR THAT SWINGS (components.h SwingDoor): given to the stamped copy when it wakes
        bool swings = false;
        Vec2 swingFoot, swingNormal;
        // WHAT STANDS ON IT, tied to one of its markers when it wakes (model_motion.h): copies in other batches (a
        // monitor in the furniture kit's groups), and other copies of this holder (a desk lamp), which wake with it
        struct GroupRider { Entity group; uint32_t index = 0; std::string marker; };
        std::vector<GroupRider> groupRiders;
        std::vector<std::pair<uint32_t, std::string>> riders;
    };
    // What dresses a copy that wakes (shared with whoever made the copies).
    std::shared_ptr<ModelDresser> dresser;
    // THE COLLISION of the copies at rest: two bodies the holder's owner makes from batchCollisionTriangles and
    // lets go with the holder. `restingStale`: a copy has woken or gone back to sleep since the second was made.
    PhysicsBodyId fixedBody = INVALID_PHYSICS_BODY, restingBody = INVALID_PHYSICS_BODY;
    bool fixedStale = true, restingStale = true;
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

// Let every copy and group of a holder go, its awake copies and its two collision bodies with them: `removeBody` is
// called with each body for the caller to hand to its PhysicsWorld. (The holder entity itself is the caller's.)
void releaseBatches(World& world, Entity holder, const std::function<void(PhysicsBodyId)>& removeBody = {});

// On a stamped copy that is a batch copy awake: where it came from, and how long it has been still.
struct BatchAwake {
    Entity holder;
    uint32_t copy = 0;
    Real still = 0;
};

// Where one of a copy's markers is in the world, in the pose it is drawn in. False: the model has no such marker.
bool batchCopyMarker(const PrefabBatches::Copy& copy, const std::string& marker, Vec3& position, Quat& rotation);

// The collision shapes of a holder's copies at rest, as triangles in the world (both windings, so a thin shape
// holds from either side). `moving` false: the shapes that never move, of every copy. `moving` true: the shapes on
// parts that can move, where each sleeping copy's pose has them (an awake copy has its own bodies for those).
void batchCollisionTriangles(const PrefabBatches& batches, bool moving, std::vector<Vec3>& vertices, std::vector<uint32_t>& indices);

// Wake a copy: stamp it in full where and as the batch drew it, its uses that were on still on, and take it out of
// the picture. Returns the stamped copy (the one already awake, if it is). What stands on it is tied to it.
Entity wakeBatchCopy(World& world, Entity holder, uint32_t copy);

// Put an awake copy back: the batch draws it in the pose it was left in, and the stamped copy goes. `removeBody`
// as for unstampPrefab.
void sleepBatchCopy(World& world, Entity awake, const std::function<void(PhysicsBodyId)>& removeBody = {});

// Whether an awake copy is doing nothing: no part on its way, nothing running, no use counting down.
bool batchCopySettled(World& world, Entity awake);

// How many entities and transforms a holder's batches come to, for a measurement.
struct BatchCount { std::size_t copies = 0, groups = 0, transforms = 0; };
BatchCount countBatches(World& world, Entity holder);

}  // namespace engine

#endif
