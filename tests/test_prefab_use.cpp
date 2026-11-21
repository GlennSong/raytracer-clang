// What a person can do to a stamped prefab (#191, step 9): a line's `uses`, and doing them on a copy.
#include "test_framework.h"

#include "../src/engine/components.h"
#include "../src/engine/model_motion.h"
#include "../src/engine/prefab_stamp.h"
#include "../src/engine/prefab_use.h"
#include "../src/engine/scripting/prefab_list.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace engine;

namespace {

const Real kDeg = 3.14159265358979323846 / 180.0;

PrefabUse use(const std::string& at, const std::string& say, const std::string& back) {
    PrefabUse u;
    u.at = at;
    u.say = say;
    u.back = back;
    return u;
}

// The shipped washing machine, with uses written here so the test does not hang on the shipped list's own.
Prefab washer() {
    Prefab p;
    CHECK(findPrefab("washing_machine", p));
    p.uses.clear();
    PrefabUse door = use("handle", "open", "close");
    door.move = {{"door", 110}};
    door.speed = 110;
    PrefabUse run = use("controls", "switch on", "switch off");
    run.run = {{"drum", 360}};
    run.seconds = 2;
    PrefabUse lost = use("no_such_marker", "poke", "");
    lost.move = {{"door", 5}};
    PrefabUse nonode = use("controls", "wind", "");
    nonode.move = {{"no_such_node", 5}};
    p.uses = {door, run, lost, nonode};
    return p;
}

}  // namespace

TEST_CASE(a_prefab_lines_uses_are_read_and_written_back_the_same) {
    const nlohmann::json line = nlohmann::json::parse(R"JSON({
        "model": "washing_machine", "physics": { "body": "static" },
        "uses": [ { "at": "handle", "say": "open", "back": "close", "move": { "door": 110 }, "speed": 110 },
                  { "at": "controls", "say": "switch on", "back": "switch off", "run": { "drum": 360 }, "seconds": 40 },
                  { "at": "front", "say": "sit", "sit": true } ] })JSON");
    Prefab p;
    std::string err;
    CHECK(readPrefabLine("washer", line, p, std::string(), &err));
    CHECK(err.empty());
    CHECK(p.uses.size() == 3);
    if (p.uses.size() != 3) return;
    CHECK(p.uses[0].at == "handle" && p.uses[0].say == "open" && p.uses[0].back == "close");
    CHECK(p.uses[0].move.size() == 1 && p.uses[0].move[0].first == "door");
    CHECK_APPROX(p.uses[0].move[0].second, 110.0, 1e-12);
    CHECK_APPROX(p.uses[1].seconds, 40.0, 1e-12);
    CHECK(p.uses[1].run.size() == 1 && p.uses[2].sit);
    const std::string written = prefabLine(p);
    CHECK(written.find("uses = { { at = \"handle\", say = \"open\", back = \"close\", move = { door = 110 }, speed = 110 }") != std::string::npos);
    CHECK(written.find("{ at = \"front\", say = \"sit\", sit = true }") != std::string::npos);

    // A use is refused, with its number, when it is missing a part or does nothing.
    auto refused = [&](const char* uses) {
        nlohmann::json bad = line;
        bad["uses"] = nlohmann::json::parse(uses);
        Prefab q;
        std::string why;
        const bool ok = readPrefabLine("washer", bad, q, std::string(), &why);
        return ok ? std::string() : why;
    };
    CHECK(refused(R"([ { "say": "open", "move": { "door": 1 } } ])").find("needs `at`") != std::string::npos);
    CHECK(refused(R"([ { "at": "handle", "move": { "door": 1 } } ])").find("needs `say`") != std::string::npos);
    CHECK(refused(R"([ { "at": "handle", "say": "open" } ])").find("does nothing") != std::string::npos);
    CHECK(refused(R"([ { "at": "handle", "say": "open", "moves": { "door": 1 } } ])").find("`moves`") != std::string::npos);
    CHECK(refused(R"([ { "at": "handle", "say": "open", "move": { "door": "wide" } } ])").find("is a number") != std::string::npos);
}

TEST_CASE(a_stamped_copy_keeps_the_uses_its_model_has_and_doing_one_moves_its_parts) {
    World world;
    ModelStore store;
    const Prefab p = washer();
    const Entity e = stampPrefab(world, nullptr, p, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    // The two uses that name a marker or a node the model does not have were left out.
    const PrefabUses* uses = world.get<PrefabUses>(e);
    CHECK(uses != nullptr);
    if (uses == nullptr) return;
    CHECK(uses->uses.size() == 2);
    CHECK(prefabUseWord(uses->uses[0]) == "open");

    // OPEN: the door heads for 110 degrees at 110 a second, and the word offered becomes "close".
    CHECK(doPrefabUse(world, e, 0));
    CHECK(prefabUseWord(world.get<PrefabUses>(e)->uses[0]) == "close");
    advanceModelMotion(world, e, 0.5);
    CHECK_APPROX(modelValue(world, e, "door"), 55 * kDeg, 1e-9);
    advanceModelMotion(world, e, 2.0);
    CHECK_APPROX(modelValue(world, e, "door"), 110 * kDeg, 1e-9);
    // CLOSE: back to 0.
    CHECK(doPrefabUse(world, e, 0));
    CHECK(prefabUseWord(world.get<PrefabUses>(e)->uses[0]) == "open");
    advanceModelMotion(world, e, 5.0);
    CHECK_APPROX(modelValue(world, e, "door"), 0.0, 1e-9);

    // SWITCH ON: the drum turns a full turn a second for as long as the use is on, and it turns itself off after its
    // two seconds; the drum then stays where it stopped.
    CHECK(doPrefabUse(world, e, 1));
    const Real step = 1.0 / 60.0;
    for (int i = 0; i < 60; ++i) { stepPrefabUses(world, step); advanceModelMotion(world, e, step); }
    CHECK(world.get<PrefabUses>(e)->uses[1].on);
    CHECK_APPROX(modelValue(world, e, "drum"), 360 * kDeg, 0.02);
    for (int i = 0; i < 90; ++i) { stepPrefabUses(world, step); advanceModelMotion(world, e, step); }
    CHECK(!world.get<PrefabUses>(e)->uses[1].on);
    const Real stopped = modelValue(world, e, "drum");
    CHECK(stopped > 700 * kDeg && stopped < 740 * kDeg);
    for (int i = 0; i < 30; ++i) { stepPrefabUses(world, step); advanceModelMotion(world, e, step); }
    CHECK_APPROX(modelValue(world, e, "drum"), stopped, 1e-9);

    // No such use: refused.
    CHECK(!doPrefabUse(world, e, 7));
    // A prefab whose line has no uses and whose model has no seat has none on its copy.
    Prefab plain;
    CHECK(findPrefab("hydrant", plain));
    plain.uses.clear();
    const Entity c = stampPrefab(world, nullptr, plain, Transform(), std::string(), nullptr, &store);
    CHECK(c.valid() && !world.has<PrefabUses>(c));
    // The crane's line has none either, but its model has a driver's seat, and a seat is a seat.
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    crane.uses.clear();
    const Entity k = stampPrefab(world, nullptr, crane, Transform(), std::string(), nullptr, &store);
    const PrefabUses* cab = k.valid() ? world.get<PrefabUses>(k) : nullptr;
    CHECK(cab != nullptr && cab->uses.size() == 1 && cab->uses[0].use.sit && cab->uses[0].use.at == "driver");
}

TEST_CASE(a_seat_is_a_seat_the_furniture_kits_sofa_stamped_as_a_prefab_offers_its_seats) {
    // The sofa is a kit piece made a model by prop_models.lua, which reads the furniture library's own description
    // of it into markers: three places to sit and one to lie. No line in prefabs.lua says so.
    World world;
    ModelStore store;
    Prefab sofa;
    CHECK(findPrefab("sofa", sofa));
    CHECK(sofa.uses.empty());
    const Entity e = stampPrefab(world, nullptr, sofa, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    const PrefabUses* uses = e.valid() ? world.get<PrefabUses>(e) : nullptr;
    CHECK(uses != nullptr);
    if (uses == nullptr) return;
    int sits = 0, lies = 0;
    for (const PrefabUses::Use& u : uses->uses) {
        sits += u.use.sit ? 1 : 0;
        lies += u.use.lie ? 1 : 0;
        CHECK(prefabUseWord(u) == (u.use.lie ? "lie down" : "sit"));
    }
    CHECK(sits == 3);
    CHECK(lies == 1);
    // A use of the line that names a seat marker wins over the seat's own offer: it is not offered twice.
    Prefab named = sofa;
    PrefabUse perch = use("sit_1", "perch", "");
    perch.sit = true;
    named.uses = {perch};
    const Entity f = stampPrefab(world, nullptr, named, Transform(), std::string(), nullptr, &store);
    const PrefabUses* mine = f.valid() ? world.get<PrefabUses>(f) : nullptr;
    CHECK(mine != nullptr && mine->uses.size() == 4 && mine->uses[0].use.say == "perch");
}

TEST_CASE(a_use_that_needs_another_waits_for_it_and_holds_it_open) {
    // A washing machine as a stand-in for a server rack: the drum may be run only while the door is open (`needs`),
    // and a third use gives way to the drum running (`unless`).
    World world;
    ModelStore store;
    Prefab p;
    CHECK(findPrefab("washing_machine", p));
    PrefabUse door = use("handle", "open", "close");
    door.id = "door";
    door.move = {{"door", 110}};
    PrefabUse spin = use("controls", "spin", "stop");
    spin.id = "spin";
    spin.needs = "door";
    spin.run = {{"drum", 90}};
    PrefabUse nudge = use("controls", "nudge", "");
    nudge.unless = "spin";
    nudge.move = {{"door", 20}};
    p.uses = {door, spin, nudge};
    const Entity e = stampPrefab(world, nullptr, p, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    auto offered = [&](std::size_t i) { return prefabUseOffered(*world.get<PrefabUses>(e), i); };
    CHECK(offered(0) && !offered(1) && offered(2));
    CHECK(!doPrefabUse(world, e, 1));                       // not while the door is shut
    CHECK(doPrefabUse(world, e, 0));                        // open the door
    CHECK(offered(1));
    CHECK(doPrefabUse(world, e, 1));                        // now it spins
    CHECK(!offered(0));                                     // and the door cannot be shut on it
    CHECK(!doPrefabUse(world, e, 0));
    CHECK(world.get<PrefabUses>(e)->uses[0].on);
    CHECK(!offered(2));                                     // the nudge gives way while it spins
    CHECK(doPrefabUse(world, e, 1));                        // stop
    CHECK(offered(0) && offered(2));
    CHECK(doPrefabUse(world, e, 0));                        // and shut
    CHECK(!world.get<PrefabUses>(e)->uses[0].on);

    // A line whose use names an id no use has is refused.
    nlohmann::json line = nlohmann::json::parse(R"JSON({ "model": "washing_machine",
        "uses": [ { "at": "controls", "say": "spin", "needs": "lid", "run": { "drum": 90 } } ] })JSON");
    Prefab q;
    std::string why;
    CHECK(!readPrefabLine("washer", line, q, std::string(), &why));
    CHECK(why.find("names 'lid'") != std::string::npos);
}

TEST_CASE(a_pattern_use_is_made_for_every_part_that_fits_and_a_push_coasts_to_a_stop) {
    World world;
    ModelStore store;
    // LOCKERS: one line, a door for every handle the model has.
    Prefab lockers;
    CHECK(findPrefab("lockers", lockers));
    CHECK(lockers.uses.size() == 1);
    const Entity e = stampPrefab(world, nullptr, lockers, Transform(), std::string(), nullptr, &store);
    const PrefabUses* uses = e.valid() ? world.get<PrefabUses>(e) : nullptr;
    CHECK(uses != nullptr);
    if (uses == nullptr) return;
    CHECK(uses->uses.size() >= 4);                         // the default lockers are four tall doors in a row
    std::size_t one = uses->uses.size();
    for (std::size_t i = 0; i < uses->uses.size(); ++i) {
        const PrefabUse& u = uses->uses[i].use;
        CHECK(u.move.size() == 1);
        // handle_2_1 opens door_2_1
        if (u.move.size() == 1) CHECK(u.move[0].first == "door_" + u.at.substr(std::string("handle_").size()));
        if (u.at == "handle_2_1") one = i;
    }
    CHECK(one < uses->uses.size());
    if (one >= uses->uses.size()) return;
    CHECK(doPrefabUse(world, e, one));
    advanceModelMotion(world, e, 5.0);
    CHECK(modelValue(world, e, "door_2_1") > 100 * kDeg);
    CHECK_APPROX(modelValue(world, e, "door_1_1"), 0.0, 1e-12);

    // THE ROUNDABOUT: a push runs the deck for a moment, then it slows over its coast and stops.
    Prefab round;
    CHECK(findPrefab("playground_roundabout", round));
    const Entity r = stampPrefab(world, nullptr, round, Transform(), std::string(), nullptr, &store);
    const PrefabUses* ru = r.valid() ? world.get<PrefabUses>(r) : nullptr;
    CHECK(ru != nullptr);
    if (ru == nullptr) return;
    std::size_t push = ru->uses.size();
    for (std::size_t i = 0; i < ru->uses.size(); ++i)
        if (ru->uses[i].use.say == "push") push = i;
    CHECK(push < ru->uses.size());
    if (push >= ru->uses.size()) return;
    CHECK(doPrefabUse(world, r, push));
    const Real step = 1.0 / 60.0;
    auto run = [&](Real seconds) {
        const Real before = modelValue(world, r, "deck");
        for (int i = 0; i < static_cast<int>(seconds * 60 + 0.5); ++i) { stepPrefabUses(world, step); advanceModelMotion(world, r, step); }
        return modelValue(world, r, "deck") - before;
    };
    const Real pushed = run(0.5);
    CHECK(pushed > 40 * kDeg);                              // at 110 degrees a second
    CHECK(world.get<PrefabUses>(r)->uses[push].on);
    run(0.3);                                               // its time is up: it is off, and coasting
    CHECK(!world.get<PrefabUses>(r)->uses[push].on);
    const Real early = run(1.0);
    run(5.0);
    const Real late = run(1.0);
    CHECK(early > late && late > 0);                        // slower and slower
    run(4.0);
    CHECK_APPROX(run(1.0), 0.0, 1e-9);                      // stopped
}

TEST_CASE(a_seat_that_is_sat_on_runs_and_sways_until_its_sitter_gets_up) {
    World world;
    ModelStore store;
    Prefab chair;
    CHECK(findPrefab("rocking_chair", chair));
    const Entity e = stampPrefab(world, nullptr, chair, Transform(), std::string(), nullptr, &store);
    const PrefabUses* uses = e.valid() ? world.get<PrefabUses>(e) : nullptr;
    CHECK(uses != nullptr && uses->uses.size() == 1 && uses->uses[0].use.sit);
    if (uses == nullptr || uses->uses.empty()) return;
    CHECK(doPrefabUse(world, e, 0));                        // someone sits (the system seats them; the use is on)
    const Real step = 1.0 / 60.0;
    Real lo = 0, hi = 0;
    for (int i = 0; i < 60 * 4; ++i) {
        stepPrefabUses(world, step);
        advanceModelMotion(world, e, step);
        lo = std::min(lo, modelValue(world, e, "rock"));
        hi = std::max(hi, modelValue(world, e, "rock"));
    }
    CHECK(hi > 8 * kDeg && lo < -8 * kDeg);                 // ten degrees each way
    CHECK(setPrefabUse(world, e, 0, false));                // they get up: it comes to rest
    for (int i = 0; i < 60 * 3; ++i) { stepPrefabUses(world, step); advanceModelMotion(world, e, step); }
    CHECK_APPROX(modelValue(world, e, "rock"), 0.0, 1e-6);
}

TEST_CASE(a_desk_can_be_stopped_part_way_and_sent_back) {
    World world;
    ModelStore store;
    Prefab desk;
    CHECK(findPrefab("standing_desk", desk));
    const Entity e = stampPrefab(world, nullptr, desk, Transform(), std::string(), nullptr, &store);
    const PrefabUses* uses = e.valid() ? world.get<PrefabUses>(e) : nullptr;
    CHECK(uses != nullptr && uses->uses.size() == 1 && uses->uses[0].use.stop);
    if (uses == nullptr || uses->uses.empty()) return;
    const Real step = 1.0 / 60.0;
    auto run = [&](Real seconds) {
        for (int i = 0; i < static_cast<int>(seconds * 60 + 0.5); ++i) { stepPrefabUses(world, step); advanceModelMotion(world, e, step); }
    };
    auto word = [&]() { return prefabUseWord(world.get<PrefabUses>(e)->uses[0]); };
    // RAISE, and while it rises the word is "stop".
    CHECK(word() == "raise");
    CHECK(doPrefabUse(world, e, 0));
    run(1.0);
    CHECK(word() == "stop");
    const Real rising = modelValue(world, e, "top");
    CHECK(rising > 0.05 && rising < 0.12);                  // 0.08 m a second
    // STOP: it stays where it is, and the next word sends it back down.
    CHECK(doPrefabUse(world, e, 0));
    run(1.0);
    CHECK_APPROX(modelValue(world, e, "top"), rising, 1e-9);
    CHECK(word() == "lower");
    CHECK(doPrefabUse(world, e, 0));
    run(0.5);
    CHECK(word() == "stop");
    run(3.0);
    CHECK_APPROX(modelValue(world, e, "top"), 0.0, 1e-9);
    CHECK(word() == "raise");
}

TEST_CASE(what_stands_on_a_desk_rises_with_it_whether_a_copy_in_a_batch_or_a_thing_of_its_own) {
    World world;
    ModelStore store;
    Prefab desk;
    CHECK(findPrefab("standing_desk", desk));
    Transform place;
    place.position = Vec3(10, 0, -4);
    const Entity e = stampPrefab(world, nullptr, desk, place, std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    Vec3 top;
    Quat turn;
    CHECK(modelMarkerWorld(world, e, "surface", top, turn));

    // A COPY IN A BATCH: one of three transforms of an instance group, a hand's width in from the surface's middle.
    const Entity batch = world.create();
    InstanceGroup group;
    Transform onDesk;
    onDesk.position = top + Vec3(0.2, 0, 0.1);
    Transform elsewhere;
    elsewhere.position = Vec3(-50, 0, 0);
    group.transforms = {elsewhere.matrix(), onDesk.matrix(), elsewhere.matrix()};
    world.add<InstanceGroup>(batch, group);
    CHECK(rideMarkerInBatch(world, e, "surface", batch, 1));
    CHECK(!rideMarkerInBatch(world, e, "surface", batch, 9));
    CHECK(!rideMarkerInBatch(world, e, "no_such_marker", batch, 1));

    // A THING OF ITS OWN: an entity with a place.
    const Entity mug = world.create();
    Transform mugAt;
    mugAt.position = top + Vec3(-0.3, 0, 0);
    world.add<Transform>(mug, mugAt);
    CHECK(rideMarker(world, mug, e, "surface"));

    // The desk rises 0.3 m (its stage and its top, 0.3 each... the top's own travel is what the surface is on).
    CHECK(setModelTarget(world, e, "stage", 0.3, 0));
    CHECK(setModelTarget(world, e, "top", 0.3, 0));
    Vec3 raised;
    CHECK(modelMarkerWorld(world, e, "surface", raised, turn));
    const Real rise = raised.y - top.y;
    CHECK(rise > 0.29);
    const InstanceGroup* g = world.get<InstanceGroup>(batch);
    CHECK((g->transforms[1].transformPoint(Vec3(0, 0, 0)) - (onDesk.position + Vec3(0, rise, 0))).length() < 1e-9);
    CHECK((g->transforms[0].transformPoint(Vec3(0, 0, 0)) - elsewhere.position).length() < 1e-12);   // the others stay
    CHECK((world.get<Transform>(mug)->position - (mugAt.position + Vec3(0, rise, 0))).length() < 1e-9);

    // Let go, the mug stays where it is when the desk goes down again.
    unrideMarker(world, mug);
    CHECK(setModelTarget(world, e, "top", 0.0, 0));
    CHECK((world.get<Transform>(mug)->position - (mugAt.position + Vec3(0, rise, 0))).length() < 1e-9);
    CHECK(g->transforms[1].transformPoint(Vec3(0, 0, 0)).y < onDesk.position.y + rise - 0.1);          // the copy went down
}
