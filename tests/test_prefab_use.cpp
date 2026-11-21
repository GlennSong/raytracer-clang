// What a person can do to a stamped prefab (#191, step 9): a line's `uses`, and doing them on a copy.
#include "test_framework.h"

#include "../src/engine/model_motion.h"
#include "../src/engine/prefab_stamp.h"
#include "../src/engine/prefab_use.h"
#include "../src/engine/scripting/prefab_list.h"

#include <nlohmann/json.hpp>

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
