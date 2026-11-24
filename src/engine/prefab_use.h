#ifndef RAYTRACER_ENGINE_PREFAB_USE_H
#define RAYTRACER_ENGINE_PREFAB_USE_H

// WHAT A PERSON CAN DO TO A STAMPED PREFAB (#191, step 9). A prefab's line lists its USES (procgen/prefab.h): open
// this door, switch that on, sit here. A copy keeps which of them are on. These are the words that act on a copy;
// who may act, and when, is PrefabUseSystem's business (systems/prefab_use_system.h), which offers them to the
// player through the interaction broker. A city person, or a script, makes the same calls.
//
// Nothing here needs physics or a renderer, so a test can stamp a prefab and use it.

#include "procgen/prefab.h"
#include "world.h"

#include <cstddef>
#include <string>
#include <vector>

namespace engine {

// On a stamped prefab that has lamps (Prefab::lights): PrefabUseSystem publishes one light for each, every frame.
struct PrefabLamps {
    std::vector<PrefabLamp> lamps;
};

// On a stamped prefab with parts that turn freely (Prefab::free): each part and how fast it is turning now, in
// degrees a second. stepFreeTurns keeps them turning, slows them, and lets a walker take them round.
struct FreeTurns {
    struct Part {
        PrefabFree spec;
        Real speed = 0;
    };
    std::vector<Part> parts;
};
// One fixed step for every freely turning part. `feet` and `velocity` are someone walking (null: nobody): within a
// part's reach and outside its radius, level with it, they take it round at their own pace.
void stepFreeTurns(World& world, Real dt, const Vec3* feet = nullptr, const Vec3* velocity = nullptr);

// On a stamped prefab that has uses: each use, and whether it is on.
struct PrefabUses {
    struct Use {
        PrefabUse use;
        bool on = false;
        Real left = 0;      // seconds until it turns itself off (a use with `seconds`)
        Real time = 0;      // seconds it has been on (where a sway is in its swing)
        bool moving = false;   // its `move` parts are still on their way (kept by the step; a `stop` use reads it)
        Real coasting = 0;
        // WHAT JUST HAPPENED, for whoever makes it heard (PrefabUseSystem reads and clears these): it was done, it was
        // undone, its parts stopped moving
        enum Cue : uint8_t { Done = 1, Undone = 2, Arrived = 4 };
        uint8_t cues = 0;  // seconds of slowing down left, after a use with `coast` was turned off
    };
    std::vector<Use> uses;
};

// Give a stamped copy its prefab's uses. One whose marker, or any of whose nodes, the copy's model does not have is
// left out (and `skipped` says which and why): one line serves a model built with more or fewer of a thing.
void addPrefabUses(World& world, Entity instance, const Prefab& prefab, std::vector<std::string>* skipped = nullptr);

// The word a use offers now: its `say`, or its `back` once it is on; "stop" while a use that can be stopped is moving.
const std::string& prefabUseWord(const PrefabUses::Use& use);

// Is use `index` offered now? Not while a use it `needs` is off, nor while one it gives way to (`unless`) is on; and
// a use that is on is not offered to be turned off while another that needs it is on.
bool prefabUseOffered(const PrefabUses& uses, std::size_t index);

// Do use `index` of a copy (nothing happens, and false, when it is not offered now): a use that is off turns on (its nodes head for their values, its running parts start)
// and one that is on turns off (they return to 0, or stop). A use with no `back` is always done afresh. False when
// the entity has no such use. (`sit` is the caller's: it seats whoever asked. This does the rest of that use, if any.)
bool doPrefabUse(World& world, Entity instance, std::size_t index);

// Turn a use on or off, whatever it was.
bool setPrefabUse(World& world, Entity instance, std::size_t index, bool on);

// One step of `dt` seconds for every copy: running parts are kept turning, and a use whose time is up turns off.
void stepPrefabUses(World& world, Real dt);

}  // namespace engine

#endif
