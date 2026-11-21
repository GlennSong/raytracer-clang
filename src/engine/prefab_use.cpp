// What a person can do to a stamped prefab (#191, step 9). See prefab_use.h.
#include "prefab_use.h"

#include "components.h"
#include "model_motion.h"

#include <cmath>

namespace engine {

namespace {

// A line speaks degrees for a turn and metres for a slide; the engine keeps radians (as the script binding does).
Real unitOf(const ModelInstance& mi, const std::string& node) {
    const ProcNode* joint = mi.model != nullptr ? mi.model->jointAt(node) : nullptr;
    return joint != nullptr && joint->motion.kind == ProcMotion::Kind::Turn ? Real(3.14159265358979323846 / 180.0) : Real(1);
}

void apply(World& world, Entity instance, PrefabUses::Use& u) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr) return;
    for (const std::pair<std::string, Real>& m : u.use.move) {
        const Real unit = unitOf(*mi, m.first);
        setModelTarget(world, instance, m.first, u.on ? m.second * unit : Real(0), u.use.speed * unit);
    }
    // a running part that is switched off stops where it is
    if (!u.on)
        for (const std::pair<std::string, Real>& r : u.use.run)
            setModelTarget(world, instance, r.first, modelValue(world, instance, r.first), 0);
    u.left = u.on ? u.use.seconds : Real(0);
}

}  // namespace

void addPrefabUses(World& world, Entity instance, const Prefab& prefab, std::vector<std::string>* skipped) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr || mi->model == nullptr) return;
    PrefabUses mine;
    for (const PrefabUse& use : prefab.uses) {
        std::string why;
        if (mi->model->findMarker(use.at) == nullptr) why = "no marker '" + use.at + "'";
        for (const std::vector<std::pair<std::string, Real>>* list : {&use.move, &use.run})
            for (const std::pair<std::string, Real>& n : *list)
                if (why.empty() && mi->model->jointAt(n.first) == nullptr) why = "no moving node '" + n.first + "'";
        if (!why.empty()) {
            if (skipped != nullptr) skipped->push_back("'" + use.say + "' at '" + use.at + "': the model has " + why);
            continue;
        }
        mine.uses.push_back(PrefabUses::Use{use, false, 0});
    }
    // A SEAT IS A SEAT: every marker whose role is "seat" or "lie" offers it, unless the line's own uses already
    // name that marker.
    for (const ProcNode& node : mi->model->nodes)
        for (const ProcMarker& mk : node.markers) {
            if (mk.role != "seat" && mk.role != "lie") continue;
            bool named = false;
            for (const PrefabUse& use : prefab.uses) named = named || use.at == mk.name;
            if (named) continue;
            PrefabUse use;
            use.at = mk.name;
            use.lie = mk.role == "lie";
            use.sit = !use.lie;
            use.say = use.lie ? "lie down" : "sit";
            mine.uses.push_back(PrefabUses::Use{use, false, 0});
        }
    if (!mine.uses.empty()) world.add<PrefabUses>(instance, std::move(mine));
}

const std::string& prefabUseWord(const PrefabUses::Use& use) { return use.on && !use.use.back.empty() ? use.use.back : use.use.say; }

bool prefabUseOffered(const PrefabUses& uses, std::size_t index) {
    if (index >= uses.uses.size()) return false;
    const PrefabUses::Use& u = uses.uses[index];
    for (const PrefabUses::Use& other : uses.uses) {
        if (&other == &u) continue;
        if (!u.use.needs.empty() && other.use.id == u.use.needs && !other.on) return false;
        if (!u.use.unless.empty() && other.use.id == u.use.unless && other.on) return false;
        // turning off what another stands on
        if (u.on && !u.use.id.empty() && other.use.needs == u.use.id && other.on) return false;
    }
    return true;
}

bool setPrefabUse(World& world, Entity instance, std::size_t index, bool on) {
    PrefabUses* uses = world.get<PrefabUses>(instance);
    if (uses == nullptr || index >= uses->uses.size()) return false;
    PrefabUses::Use& u = uses->uses[index];
    u.on = on;
    apply(world, instance, u);
    // A use that cannot be undone by hand and has no time of its own (a button pressed) lets go by itself.
    if (on && u.use.back.empty() && u.use.seconds <= 0 && u.use.run.empty() && !u.use.move.empty()) u.left = Real(0.4);
    return true;
}

bool doPrefabUse(World& world, Entity instance, std::size_t index) {
    const PrefabUses* uses = world.get<PrefabUses>(instance);
    if (uses == nullptr || index >= uses->uses.size()) return false;
    const PrefabUses::Use& u = uses->uses[index];
    if (!prefabUseOffered(*uses, index)) return false;
    return setPrefabUse(world, instance, index, u.use.back.empty() ? true : !u.on);
}

void stepPrefabUses(World& world, Real dt) {
    // (collected first: turning a use off sets targets on the instance, which is not a structural change, but the
    // walk is kept simple and safe all the same)
    std::vector<Entity> with;
    world.each<PrefabUses>([&](Entity e, PrefabUses&) { with.push_back(e); });
    for (Entity e : with) {
        PrefabUses* uses = world.get<PrefabUses>(e);
        const ModelInstance* mi = world.get<ModelInstance>(e);
        if (uses == nullptr || mi == nullptr) continue;
        for (std::size_t i = 0; i < uses->uses.size(); ++i) {
            PrefabUses::Use& u = uses->uses[i];
            if (!u.on) continue;
            for (const std::pair<std::string, Real>& r : u.use.run) {
                const Real rate = r.second * unitOf(*mi, r.first);
                setModelTarget(world, e, r.first, modelValue(world, e, r.first) + rate, std::fabs(rate));   // a second ahead, always
            }
            if (u.left > 0) {
                u.left -= dt;
                if (u.left <= 0) {
                    if (prefabUseOffered(*uses, i)) setPrefabUse(world, e, i, false);
                    else u.left = Real(0.5);   // something that needs it is still on: look again shortly
                }
            }
        }
    }
}

}  // namespace engine
