// What a person can do to a stamped prefab (#191, step 9). See prefab_use.h.
#include "prefab_use.h"

#include "components.h"
#include "model_motion.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// A line speaks degrees for a turn and metres for a slide; the engine keeps radians (as the script binding does).
Real unitOf(const ModelInstance& mi, const std::string& node) {
    const ProcNode* joint = mi.model != nullptr ? mi.model->jointAt(node) : nullptr;
    return joint != nullptr && joint->motion.kind == ProcMotion::Kind::Turn ? Real(3.14159265358979323846 / 180.0) : Real(1);
}

const Real kTau = Real(6.28318530717958647692);

void apply(World& world, Entity instance, PrefabUses::Use& u) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr) return;
    for (const std::pair<std::string, Real>& m : u.use.move) {
        const Real unit = unitOf(*mi, m.first);
        setModelTarget(world, instance, m.first, u.on ? m.second * unit : Real(0), u.use.speed * unit);
    }
    if (!u.on) {
        // a running part that is switched off stops where it is (one that coasts is slowed by the step instead)
        if (u.coasting <= 0)
            for (const std::pair<std::string, Real>& r : u.use.run)
                setModelTarget(world, instance, r.first, modelValue(world, instance, r.first), 0);
        // a swaying part comes back to rest
        for (const std::pair<std::string, Real>& w : u.use.sway) {
            const Real unit = unitOf(*mi, w.first);
            setModelTarget(world, instance, w.first, 0, std::fabs(w.second) * unit * kTau / std::max(u.use.period, Real(0.1)));
        }
    }
    u.left = u.on ? u.use.seconds : Real(0);
}

// Does `name` fit `pattern`, in which one `*` stands for any text? `star` gets that text. A pattern with no `*`
// fits only itself.
bool fits(const std::string& pattern, const std::string& name, std::string& star) {
    const std::size_t at = pattern.find('*');
    if (at == std::string::npos) { star.clear(); return pattern == name; }
    const std::string head = pattern.substr(0, at), tail = pattern.substr(at + 1);
    if (name.size() < head.size() + tail.size() || name.compare(0, head.size(), head) != 0 ||
        name.compare(name.size() - tail.size(), tail.size(), tail) != 0)
        return false;
    star = name.substr(head.size(), name.size() - head.size() - tail.size());
    return !star.empty();
}

std::string withStar(std::string text, const std::string& star) {
    for (std::size_t at = text.find('*'); at != std::string::npos; at = text.find('*', at + star.size())) text.replace(at, 1, star);
    return text;
}

// MANY LIKE PARTS (procgen/prefab.h): a use with a `*` becomes the uses it stands for on this model.
std::vector<PrefabUse> expanded(const ProcModel& model, const PrefabUse& use) {
    std::vector<PrefabUse> out;
    if (use.at.find('*') != std::string::npos) {
        for (const ProcNode& node : model.nodes)
            for (const ProcMarker& mk : node.markers) {
                std::string star;
                if (!fits(use.at, mk.name, star)) continue;
                PrefabUse one = use;
                one.at = mk.name;
                one.id = withStar(use.id, star);
                one.needs = withStar(use.needs, star);
                one.unless = withStar(use.unless, star);
                one.stops = withStar(use.stops, star);
                for (std::vector<std::pair<std::string, Real>>* list : {&one.move, &one.run, &one.sway, &one.kick})
                    for (std::pair<std::string, Real>& n : *list) n.first = withStar(n.first, star);
                out.push_back(one);
            }
        return out;
    }
    PrefabUse one = use;
    for (std::vector<std::pair<std::string, Real>>* list : {&one.move, &one.run, &one.sway}) {
        std::vector<std::pair<std::string, Real>> all;
        for (const std::pair<std::string, Real>& n : *list) {
            if (n.first.find('*') == std::string::npos) { all.push_back(n); continue; }
            std::string star;
            for (const ProcNode& node : model.nodes)
                if (node.motion.kind != ProcMotion::Kind::None && fits(n.first, node.name, star)) all.emplace_back(node.name, n.second);
        }
        *list = all;
    }
    out.push_back(one);
    return out;
}

}  // namespace

std::vector<PrefabUse> prefabUsesFor(const ProcModel& model, const Prefab& prefab, std::vector<std::string>* skipped) {
    std::vector<PrefabUse> mine;
    std::vector<PrefabUse> all;
    for (const PrefabUse& written : prefab.uses) {
        const std::vector<PrefabUse> those = expanded(model, written);
        if (those.empty() && skipped != nullptr) skipped->push_back("'" + written.say + "' at '" + written.at + "': the model has no marker that fits");
        all.insert(all.end(), those.begin(), those.end());
    }
    for (const PrefabUse& use : all) {
        std::string why;
        if (model.findMarker(use.at) == nullptr) why = "no marker '" + use.at + "'";
        if (why.empty() && use.move.empty() && use.run.empty() && use.sway.empty() && use.kick.empty() && use.light.empty() && use.sound.empty() &&
            !use.sit && !use.lie)
            why = "none of the parts it names";
        if (why.empty() && !use.lightAt.empty() && model.findMarker(use.lightAt) == nullptr) why = "no marker '" + use.lightAt + "'";
        for (const std::vector<std::pair<std::string, Real>>* list : {&use.move, &use.run, &use.sway, &use.kick})
            for (const std::pair<std::string, Real>& n : *list)
                if (why.empty() && model.jointAt(n.first) == nullptr) why = "no moving node '" + n.first + "'";
        if (!why.empty()) {
            if (skipped != nullptr) skipped->push_back("'" + use.say + "' at '" + use.at + "': the model has " + why);
            continue;
        }
        mine.push_back(use);
    }
    // A SEAT IS A SEAT: every marker whose role is "seat" or "lie" offers it, unless the line's own uses already
    // name that marker.
    for (const ProcNode& node : model.nodes)
        for (const ProcMarker& mk : node.markers) {
            if (mk.role != "seat" && mk.role != "lie") continue;
            bool named = false;
            for (const PrefabUse& use : all) named = named || use.at == mk.name;
            if (named) continue;
            PrefabUse use;
            use.at = mk.name;
            use.lie = mk.role == "lie";
            use.sit = !use.lie;
            use.say = use.lie ? "lie down" : "sit";
            mine.push_back(use);
        }
    return mine;
}

void addPrefabUses(World& world, Entity instance, const Prefab& prefab, std::vector<std::string>* skipped) {
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr || mi->model == nullptr) return;
    PrefabUses mine;
    for (const PrefabUse& use : prefabUsesFor(*mi->model, prefab, skipped)) mine.uses.push_back(PrefabUses::Use{use});
    if (!mine.uses.empty()) world.add<PrefabUses>(instance, std::move(mine));
}

const std::string& prefabUseWord(const PrefabUses::Use& use) {
    static const std::string kStop = "stop";
    if (use.use.stop && use.moving) return kStop;
    return use.on && !use.use.back.empty() ? use.use.back : use.use.say;
}

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
    const bool was = u.on;
    u.on = on;
    if (on && !was) u.cues |= PrefabUses::Use::Done;
    if (!on && was) u.cues |= PrefabUses::Use::Undone;
    if (on && was) u.cues |= PrefabUses::Use::Done;   // done afresh (a button pressed again)
    if (on) { u.coasting = 0; if (!was) u.time = 0; }
    else if (was && u.use.coast > 0 && !u.use.run.empty()) u.coasting = u.use.coast;
    apply(world, instance, u);
    u.moving = !u.use.move.empty() && u.use.speed > 0;
    // A use that cannot be undone by hand and has no time of its own (a button pressed) lets go by itself.
    if (on && u.use.back.empty() && u.use.seconds <= 0 && u.use.run.empty() && !u.use.move.empty()) u.left = Real(0.4);
    // A SHOVE (a use's `kick`): more speed on a part that turns freely, up to its top speed.
    if (on && !u.use.kick.empty()) {
        if (FreeTurns* turns = world.get<FreeTurns>(instance))
            for (const std::pair<std::string, Real>& k : u.use.kick)
                for (FreeTurns::Part& part : turns->parts)
                    if (part.spec.node == k.first) part.speed = std::clamp(part.speed + k.second, -part.spec.top, part.spec.top);
        if (u.use.back.empty() && u.use.seconds <= 0 && u.use.run.empty()) u.left = Real(0.3);   // (a shove is over at once)
    }
    // DOING THIS STOPS THAT (a use's `stops`): opening the door turns the machine off.
    if (on && !was && !u.use.stops.empty()) {
        const std::string stops = u.use.stops;
        for (std::size_t i = 0; i < uses->uses.size(); ++i)
            if (i != index && uses->uses[i].on && uses->uses[i].use.id == stops) setPrefabUse(world, instance, i, false);
    }
    return true;
}

bool doPrefabUse(World& world, Entity instance, std::size_t index) {
    const PrefabUses* uses = world.get<PrefabUses>(instance);
    if (uses == nullptr || index >= uses->uses.size()) return false;
    const PrefabUses::Use& u = uses->uses[index];
    if (!prefabUseOffered(*uses, index)) return false;
    if (u.use.stop && u.moving) {   // stopped part way: the parts stay where they are
        for (const std::pair<std::string, Real>& m : u.use.move) setModelTarget(world, instance, m.first, modelValue(world, instance, m.first), 0);
        world.get<PrefabUses>(instance)->uses[index].moving = false;
        world.get<PrefabUses>(instance)->uses[index].cues |= PrefabUses::Use::Arrived;
        return true;
    }
    return setPrefabUse(world, instance, index, u.use.back.empty() ? true : !u.on);
}

void stepPrefabUses(World& world, Real dt) {
    std::vector<Entity> with;
    world.each<PrefabUses>([&](Entity e, PrefabUses&) { with.push_back(e); });
    for (Entity e : with) {
        PrefabUses* uses = world.get<PrefabUses>(e);
        const ModelInstance* mi = world.get<ModelInstance>(e);
        if (uses == nullptr || mi == nullptr) continue;
        for (std::size_t i = 0; i < uses->uses.size(); ++i) {
            PrefabUses::Use& u = uses->uses[i];
            if (u.moving) {   // still on its way? (its parts have a target until they arrive)
                bool any = false;
                for (const std::pair<std::string, Real>& m : u.use.move) any = any || mi->target.values.count(m.first) > 0;
                u.moving = any;
                if (!any) u.cues |= PrefabUses::Use::Arrived;
            }
            if (!u.on && u.coasting <= 0) continue;
            // how hard it runs: full while on, then less and less as it coasts
            const Real drive = u.on ? Real(1) : u.coasting / std::max(u.use.coast, Real(1e-6));
            for (const std::pair<std::string, Real>& r : u.use.run) {
                const Real rate = r.second * unitOf(*mi, r.first) * drive;
                setModelTarget(world, e, r.first, modelValue(world, e, r.first) + rate, std::fabs(rate));   // a second ahead, always
            }
            if (!u.on) {
                u.coasting -= dt;
                if (u.coasting <= 0) {
                    u.coasting = 0;
                    for (const std::pair<std::string, Real>& r : u.use.run) setModelTarget(world, e, r.first, modelValue(world, e, r.first), 0);
                }
                continue;
            }
            u.time += dt;
            for (const std::pair<std::string, Real>& w : u.use.sway) {
                const Real amp = w.second * unitOf(*mi, w.first), rate = kTau / std::max(u.use.period, Real(0.1));
                setModelTarget(world, e, w.first, amp * std::sin(rate * u.time), std::fabs(amp) * rate * Real(1.5));
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

void stepFreeTurns(World& world, Real dt, const Vec3* feet, const Vec3* velocity) {
    std::vector<Entity> with;
    world.each<FreeTurns>([&](Entity e, FreeTurns&) { with.push_back(e); });
    for (Entity e : with) {
        FreeTurns* turns = world.get<FreeTurns>(e);
        const ModelInstance* mi = world.get<ModelInstance>(e);
        if (turns == nullptr || mi == nullptr) continue;
        for (FreeTurns::Part& part : turns->parts) {
            // SOMEONE WALKING ROUND IT, a hand on its rail: it goes round with them. (Their pace across the line to
            // its middle, over their distance from it, is the turn they would give it.)
            Vec3 middle;
            Quat turn;
            if (feet != nullptr && velocity != nullptr && modelNodeWorld(world, e, part.spec.node, middle, turn)) {
                const Vec3 out(feet->x - middle.x, 0, feet->z - middle.z);
                const Real r = out.length();
                if (r >= part.spec.radius && r < part.spec.reach && std::fabs(feet->y - middle.y) < Real(1.2) && velocity->length() < Real(12)) {
                    const Vec3 along = cross(Vec3(0, 1, 0), out * (Real(1) / r));
                    const Real pace = dot(*velocity, along);
                    if (std::fabs(pace) > Real(0.4)) {
                        const Real want = std::clamp(pace / r * Real(180.0 / 3.14159265358979323846), -part.spec.top, part.spec.top);
                        part.speed += (want - part.speed) * std::min(Real(1), dt * Real(4));
                    }
                }
            }
            part.speed *= std::exp(-dt / std::max(part.spec.drag, Real(0.1)));
            if (std::fabs(part.speed) < Real(1.5)) {   // (under a degree and a half a second: it has stopped)
                if (part.speed != 0) setModelTarget(world, e, part.spec.node, modelValue(world, e, part.spec.node), 0);
                part.speed = 0;
                continue;
            }
            const Real rate = part.speed * unitOf(*mi, part.spec.node);
            setModelTarget(world, e, part.spec.node, modelValue(world, e, part.spec.node) + rate, std::fabs(rate));   // a second ahead, always
        }
    }
}

}  // namespace engine
