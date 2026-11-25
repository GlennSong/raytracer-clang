// What a person can do to a stamped prefab (#191, step 9): a line's `uses`, and doing them on a copy.
#include "test_framework.h"

#include "../src/engine/audio/sfx.h"
#include "../src/engine/components.h"
#include "../src/engine/model_motion.h"
#include "../src/engine/interior_occupants.h"
#include "../src/engine/prefab_batch.h"
#include "../src/engine/prefab_stamp.h"
#include "../src/engine/prefab_use.h"
#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/furniture_library.h"
#include "../src/engine/scripting/furniture_library_lua.h"
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

// THE SHIPPED MICROWAVE (Glenn, 2026-11-23: "opening the door should stop the microwave"): it is not switched on with
// its door open (`unless`), opening the door turns it off (`stops`), and its lamp is lit while it runs or stands open.
// The fridge's freezer drawers pull out only with the freezer's door open; the automatic doors say how near.
TEST_CASE(opening_a_microwaves_door_stops_it_and_a_freezers_drawers_wait_for_its_door) {
    World world;
    ModelStore store;
    Prefab p;
    CHECK(findPrefab("microwave", p));
    const Entity e = stampPrefab(world, nullptr, p, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    const PrefabUses* uses = world.get<PrefabUses>(e);
    CHECK(uses != nullptr && uses->uses.size() >= 2);
    if (uses == nullptr || uses->uses.size() < 2) return;
    std::size_t run = 99, door = 99;
    for (std::size_t i = 0; i < uses->uses.size(); ++i) {
        if (uses->uses[i].use.id == "run") run = i;
        if (uses->uses[i].use.id == "door") door = i;
    }
    CHECK(run != 99 && door != 99);
    if (run == 99 || door == 99) return;
    CHECK(doPrefabUse(world, e, run));                      // switched on
    CHECK(world.get<PrefabUses>(e)->uses[run].on);
    CHECK(!world.get<PrefabUses>(e)->uses[run].use.light.empty());
    CHECK(doPrefabUse(world, e, door));                     // the door is opened ...
    CHECK(world.get<PrefabUses>(e)->uses[door].on);
    CHECK(!world.get<PrefabUses>(e)->uses[run].on);         // ... and it has stopped
    CHECK(!prefabUseOffered(*world.get<PrefabUses>(e), run));   // and will not start with the door open
    CHECK(doPrefabUse(world, e, door));                     // shut
    CHECK(prefabUseOffered(*world.get<PrefabUses>(e), run));

    Prefab f;
    CHECK(findPrefab("fridge", f));
    const Entity fe = stampPrefab(world, nullptr, f, Transform(), std::string(), nullptr, &store);
    CHECK(fe.valid());
    if (!fe.valid()) return;
    const PrefabUses* fu = world.get<PrefabUses>(fe);
    std::size_t freezer = 99;
    int drawers = 0;
    for (std::size_t i = 0; fu != nullptr && i < fu->uses.size(); ++i) {
        if (fu->uses[i].use.id == "freezer") freezer = i;
        if (fu->uses[i].use.say == "pull out") { ++drawers; CHECK(!prefabUseOffered(*fu, i)); }
    }
    CHECK(freezer != 99 && drawers == 2);
    if (freezer == 99) return;
    CHECK(doPrefabUse(world, fe, freezer));
    for (std::size_t i = 0; i < world.get<PrefabUses>(fe)->uses.size(); ++i)
        if (world.get<PrefabUses>(fe)->uses[i].use.say == "pull out") CHECK(doPrefabUse(world, fe, i));

    Prefab d;
    CHECK(findPrefab("automatic_doors", d));
    CHECK(!d.uses.empty() && d.uses[0].near > 2.0);
    Prefab r;
    CHECK(findPrefab("revolving_door", r));
    CHECK(r.uses.size() == 3 && r.uses[0].near > 1.0);
}

// A TUNE IS MADE FROM ITS NOTES (audio/sfx.h tune; tunes.lua): as long as its beats, never clipping, the same every
// time, and a loop without a seam -- a note held over the end rings on at the start.
TEST_CASE(a_tune_is_made_from_its_notes_and_loops_without_a_seam) {
    std::vector<sfx::TuneNote> notes;
    for (int i = 0; i < 8; ++i) { sfx::TuneNote n; n.pitch = 60 + i; n.start = i; n.length = 1; notes.push_back(n); }
    const uint32_t rate = 24000;
    for (sfx::TuneVoice voice : {sfx::TuneVoice::Piano, sfx::TuneVoice::Record}) {
        const std::vector<float> a = sfx::tune(notes, voice, 120.0, rate, 3), b = sfx::tune(notes, voice, 120.0, rate, 3);
        CHECK(a.size() == static_cast<std::size_t>(8 * 0.5 * rate));   // eight beats at 120 a minute: four seconds
        CHECK(a == b);
        float peak = 0, energy = 0;
        for (float v : a) { peak = std::max(peak, std::fabs(v)); energy += v * v; }
        CHECK(peak > 0.5f && peak <= 0.81f);
        CHECK(energy / a.size() > 1e-4f);
        // the last note still rings as the clip ends, and the start carries on from it: no jump at the seam greater
        // than between any two samples of the last note itself
        float seam = std::fabs(a.front() - a.back()), step = 0;
        for (std::size_t i = a.size() - 2000; i + 1 < a.size(); ++i) step = std::max(step, std::fabs(a[i + 1] - a[i]));
        CHECK(seam <= step * 1.5f + 0.02f);
    }
    CHECK(sfx::tune({}, sfx::TuneVoice::Piano, 120.0).empty());
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
    // IT TURNS FREELY (Glenn, 2026-11-23: "a real physics object that you could get into and it would turn"): a push
    // sets it going, a second push makes it faster, it slows by itself, and someone walking round it with a hand
    // on the rail takes it round with them.
    const Real step = 1.0 / 60.0;
    auto run = [&](Real seconds, const Vec3* feet = nullptr, const Vec3* velocity = nullptr) {
        const Real before = modelValue(world, r, "deck");
        for (int i = 0; i < static_cast<int>(seconds * 60 + 0.5); ++i) {
            stepFreeTurns(world, step, feet, velocity);
            stepPrefabUses(world, step);
            advanceModelMotion(world, r, step);
        }
        return modelValue(world, r, "deck") - before;
    };
    CHECK_APPROX(run(0.5), 0.0, 1e-9);                      // nobody has touched it
    CHECK(doPrefabUse(world, r, push));
    const Real once = run(0.5);
    CHECK(once > 30 * kDeg);                                // about 110 degrees a second
    CHECK(!world.get<PrefabUses>(r)->uses[push].on);        // (a shove is over at once: it can be pushed again)
    CHECK(doPrefabUse(world, r, push));
    const Real twice = run(0.5);
    CHECK(twice > once * 1.5);                              // faster for the second push
    const Real early = run(1.0);
    run(12.0);
    const Real late = run(1.0);
    CHECK(early > late && late > 0);                        // slower and slower
    run(60.0);
    CHECK_APPROX(run(1.0), 0.0, 1e-9);                      // stopped
    // walked round: at 1.6 m from the middle on +x, going toward -z at 2 m/s. About +y that is a turn the positive
    // way (up x out = -z), at 2 / 1.6 radians a second
    const Vec3 feet(1.6, 0.0, 0.0), going(0, 0, -2.0);
    run(2.0, &feet, &going);
    const Real led = run(0.5, &feet, &going);
    CHECK(led > 0.4 * (2.0 / 1.6) * 0.5);                   // well on its way to their pace
    const Vec3 back(0, 0, 2.0);
    run(3.0, &feet, &back);
    CHECK(run(0.5, &feet, &back) < 0);                      // and round the other way when they turn back
    const Vec3 onIt(0.5, 0.3, 0.0);                         // standing on the deck: carried, not pushing
    const FreeTurns* turns = world.get<FreeTurns>(r);
    CHECK(turns != nullptr && turns->parts.size() == 1);
    const Real before = turns != nullptr && !turns->parts.empty() ? turns->parts[0].speed : 0;
    run(0.2, &onIt, &going);
    CHECK(std::fabs(world.get<FreeTurns>(r)->parts[0].speed) <= std::fabs(before));
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

// COPIES DRAWN IN BATCHES (#315 step 10, part a; prefab_batch.h): a copy with no entity of its own is one transform
// in a group for each thing its model draws; every copy of a model shares the groups; a batch copy stands exactly
// where a stamped copy's pieces stand, at rest and in a pose; one taken out of the picture comes back where it was.
TEST_CASE(copies_drawn_in_batches_share_groups_and_stand_where_stamped_copies_do) {
    World world;
    ModelStore store;
    Prefab fridge;
    CHECK(findPrefab("fridge", fridge));
    const std::size_t before = world.entityCount();
    Transform at;
    at.position = Vec3(4, 0.05, -7);
    at.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), 0.6);
    const Entity full = stampPrefab(world, nullptr, fridge, at, std::string(), nullptr, &store);
    CHECK(full.valid());
    if (!full.valid()) return;
    const std::size_t stampedCost = world.entityCount() - before;
    const std::size_t pieces = world.get<ModelInstance>(full)->pieces.size();

    const Entity holder = world.create();
    const std::size_t mark = world.entityCount();
    const uint32_t a = batchPrefab(world, holder, nullptr, fridge, at, std::string(), nullptr, &store);
    CHECK(a == 0);
    const BatchCount one = countBatches(world, holder);
    CHECK(one.copies == 1 && one.transforms == pieces && one.groups >= 1 && one.groups <= pieces);
    const std::size_t groupCost = world.entityCount() - mark;
    Transform there = at;
    there.position = Vec3(-20, 3.1, 12);
    for (int i = 0; i < 49; ++i) { there.position.x += 1.0; CHECK(batchPrefab(world, holder, nullptr, fridge, there, std::string(), nullptr, &store) == static_cast<uint32_t>(i + 1)); }
    const BatchCount fifty = countBatches(world, holder);
    CHECK(fifty.copies == 50 && fifty.groups == one.groups && fifty.transforms == 50 * pieces);
    CHECK(world.entityCount() - mark == groupCost);          // forty-nine more copies: not one more entity
    std::printf("    [batch] a fridge stamped: %zu entities; 50 in batches: %zu entities (%zu groups, %zu transforms)\n", stampedCost,
                groupCost, fifty.groups, fifty.transforms);
    CHECK(groupCost * 10 < stampedCost * 50);                // fifty stamped would be fifty times one's entities

    // where the first batch copy draws each thing is where the stamped copy's piece is
    auto same = [&](const char* when) {
        const PrefabBatches* b = world.get<PrefabBatches>(holder);
        const ModelInstance* mi = world.get<ModelInstance>(full);
        Real worst = 0;
        for (std::size_t i = 0; i < mi->pieces.size(); ++i) {
            const Transform* piece = world.get<Transform>(mi->pieces[i]);
            const InstanceGroup* g = world.get<InstanceGroup>(b->groups[b->copies[0].draws[i].first].entity);
            const Mat4& m = g->transforms[b->copies[0].draws[i].second];
            const Mat4 want = piece->matrix();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c) worst = std::max(worst, std::fabs(m.m[r][c] - want.m[r][c]));
        }
        std::printf("    [batch] %s: a batch copy is within %.2g of the stamped copy's pieces\n", when, static_cast<double>(worst));
        return worst < 1e-6;
    };
    CHECK(same("at rest"));
    // ...and in a pose: the fridge's door open 110 degrees, both ways of serving the copy given the same pose
    CHECK(setModelTarget(world, full, "fridge_door", 110 * kDeg, 0));
    advanceModelMotion(world, full, 1.0);
    poseModelPieces(world, full);
    poseBatchCopy(world, holder, a, world.get<ModelInstance>(full)->pose);
    CHECK(same("with its door open"));
    // taken out of the picture it draws nothing, and comes back where its pose has it
    showBatchCopy(world, holder, a, false);
    {
        const PrefabBatches* b = world.get<PrefabBatches>(holder);
        const InstanceGroup* g = world.get<InstanceGroup>(b->groups[b->copies[0].draws[0].first].entity);
        const Mat4& m = g->transforms[b->copies[0].draws[0].second];
        CHECK(m.m[0][0] == 0 && m.m[1][1] == 0 && m.m[2][2] == 0);
    }
    showBatchCopy(world, holder, a, true);
    CHECK(same("shown again"));
    releaseBatches(world, holder);
    CHECK(world.entityCount() == mark);
    CHECK(!world.has<PrefabBatches>(holder));
}

// A BATCH COPY IS READY, WAKES AND SLEEPS (#315 step 10, parts b and c). At rest it is solid from its model's shapes
// and its markers are found without an entity; used, it is stamped in full where and as the batch drew it; put
// back, the batch draws it in the pose it was left in with the uses that were on still on.
// A kerb prop a car knocks off (ADR-0523): its copy leaves the batch for good, awake or asleep.
TEST_CASE(a_batch_copy_removed_is_not_drawn_solid_used_or_woken_again) {
    World world;
    ModelStore store;
    Prefab bin;
    CHECK(findPrefab("litter_bin", bin));
    const Entity holder = world.create();
    Transform at;
    at.position = Vec3(4, 0, -2);
    const uint32_t a = batchPrefab(world, holder, nullptr, bin, at, std::string(), nullptr, &store);
    Transform far = at;
    far.position.x += 900;
    const uint32_t b2 = batchPrefab(world, holder, nullptr, bin, far, std::string(), nullptr, &store);
    CHECK(a == 0 && b2 == 1);
    const PrefabBatches* b = world.get<PrefabBatches>(holder);
    // the holder knows where its copies are: near either, not near the middle
    CHECK(b->near(at.position, 5) && b->near(far.position, 5) && !b->near(Vec3(4, 0, 2000), 5));
    std::vector<Vec3> v0, v1;
    std::vector<uint32_t> i0, i1;
    batchCollisionTriangles(*b, false, v0, i0);
    // awake first, then removed: the stamped copy goes too
    const Entity woken = wakeBatchCopy(world, holder, a);
    CHECK(woken.valid());
    removeBatchCopy(world, holder, a);
    CHECK(!world.alive(woken));
    b = world.get<PrefabBatches>(holder);
    CHECK(b->copies[a].gone && !b->copies[a].awake.valid());
    CHECK(!wakeBatchCopy(world, holder, a).valid());
    batchCollisionTriangles(*b, false, v1, i1);
    CHECK(i1.size() * 2 == i0.size());                       // one bin's shapes left of two
    // every transform of the removed copy is nothing; the other's are as they were
    bool hidden = true;
    for (const std::pair<uint32_t, uint32_t>& d : b->copies[a].draws) {
        const InstanceGroup* g = world.get<InstanceGroup>(b->groups[d.first].entity);
        if (g == nullptr || (g->transforms[d.second].transformPoint(Vec3(1, 1, 1)) - g->transforms[d.second].transformPoint(Vec3(0, 0, 0))).length() > 1e-9) hidden = false;
    }
    CHECK(hidden && !b->copies[a].draws.empty());
    removeBatchCopy(world, holder, a);                       // (twice is once)
    CHECK(wakeBatchCopy(world, holder, b2).valid());          // the other is untouched
}

TEST_CASE(a_batch_copy_is_solid_wakes_where_it_was_drawn_and_sleeps_as_it_was_left) {
    World world;
    ModelStore store;
    Prefab fridge;
    CHECK(findPrefab("fridge", fridge));
    const Entity holder = world.create();
    Transform at;
    at.position = Vec3(10, 0.05, 3);
    at.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), 1.1);
    const uint32_t a = batchPrefab(world, holder, nullptr, fridge, at, std::string(), nullptr, &store);
    Transform beside = at;
    beside.position.x += 2;
    const uint32_t other = batchPrefab(world, holder, nullptr, fridge, beside, std::string(), nullptr, &store);
    CHECK(a == 0 && other == 1);
    const PrefabBatches* b = world.get<PrefabBatches>(holder);
    // its uses are the ones a stamped copy has, in the same order
    const Entity full = stampPrefab(world, nullptr, fridge, at, std::string(), nullptr, &store);
    const PrefabUses* fullUses = world.get<PrefabUses>(full);
    CHECK(fullUses != nullptr && fullUses->uses.size() == b->copies[a].uses.size() && !b->copies[a].uses.empty());
    for (std::size_t i = 0; fullUses != nullptr && i < fullUses->uses.size() && i < b->copies[a].uses.size(); ++i)
        CHECK(fullUses->uses[i].use.at == b->copies[a].uses[i].at && fullUses->uses[i].use.say == b->copies[a].uses[i].say);
    // a marker is where the stamped copy's is
    Vec3 mine, theirs;
    Quat q1, q2;
    CHECK(batchCopyMarker(b->copies[a], "fridge_door_handle", mine, q1));
    CHECK(modelMarkerWorld(world, full, "fridge_door_handle", theirs, q2));
    CHECK((mine - theirs).length() < 1e-9);
    unstampPrefab(world, full);

    // SOLID: the shapes that never move, and the shapes on parts that can, as triangles (both windings)
    std::vector<Vec3> fixedV, movingV;
    std::vector<uint32_t> fixedI, movingI;
    batchCollisionTriangles(*b, false, fixedV, fixedI);
    batchCollisionTriangles(*b, true, movingV, movingI);
    std::printf("    [batch] two fridges at rest: %zu fixed triangles, %zu on parts that move\n", fixedI.size() / 6, movingI.size() / 6);
    CHECK(fixedI.size() >= 2 * 5 * 12 * 6 && movingI.size() >= 2 * 2 * 12 * 6);   // five walls and two doors each, at least
    Real lo = 1e9, hi = -1e9;
    for (const Vec3& v : fixedV) { lo = std::min(lo, v.y); hi = std::max(hi, v.y); }
    CHECK(lo > -0.01 && lo < 0.2 && hi > 1.7 && hi < 2.0);                     // a fridge's height, on its floor

    // WAKE: stamped where it was drawn; the batch stops drawing it; its moving shapes leave the resting triangles
    const std::size_t entitiesAsleep = world.entityCount();
    const Entity woken = wakeBatchCopy(world, holder, a);
    CHECK(woken.valid() && world.has<BatchAwake>(woken) && world.has<PrefabUses>(woken));
    CHECK(wakeBatchCopy(world, holder, a) == woken);                           // (awake already: the same copy)
    CHECK(world.entityCount() > entitiesAsleep);
    b = world.get<PrefabBatches>(holder);
    CHECK(!b->copies[a].shown && b->copies[a].awake == woken && b->restingStale);
    CHECK((world.get<Transform>(woken)->position - at.position).length() < 1e-9);
    std::vector<Vec3> v2;
    std::vector<uint32_t> i2;
    batchCollisionTriangles(*b, true, v2, i2);
    CHECK(i2.size() * 2 == movingI.size());                                     // only the sleeping fridge's doors now
    // use it: open the fridge door, and let it arrive
    std::size_t door = 99;
    const PrefabUses* uses = world.get<PrefabUses>(woken);
    for (std::size_t i = 0; i < uses->uses.size(); ++i) if (uses->uses[i].use.at == "fridge_door_handle") door = i;
    CHECK(door != 99);
    if (door == 99) return;
    CHECK(doPrefabUse(world, woken, door));
    CHECK(!batchCopySettled(world, woken));                                     // its door is on its way
    for (int i = 0; i < 240; ++i) { stepPrefabUses(world, 1.0 / 60.0); advanceModelMotion(world, woken, 1.0 / 60.0); }
    CHECK(batchCopySettled(world, woken));
    const Real open = modelValue(world, woken, "fridge_door");
    CHECK(open > 100 * kDeg);

    // SLEEP: back in the batch with its door open and that use still on; the stamped copy gone
    sleepBatchCopy(world, woken);
    CHECK(!world.alive(woken));
    CHECK(world.entityCount() == entitiesAsleep);
    b = world.get<PrefabBatches>(holder);
    CHECK(b->copies[a].shown && !b->copies[a].awake.valid() && b->copies[a].on[door] != 0);
    CHECK_APPROX(b->copies[a].pose.values.at("fridge_door"), open, 1e-9);
    Vec3 handleOpen;
    CHECK(batchCopyMarker(b->copies[a], "fridge_door_handle", handleOpen, q1));
    CHECK((handleOpen - mine).length() > 0.4);                                  // the handle is where the open door has it
    // ...and woken again it is as it was left: the door open, the use on (so it offers "close")
    const Entity again = wakeBatchCopy(world, holder, a);
    CHECK(again.valid() && world.get<PrefabUses>(again)->uses[door].on);
    CHECK_APPROX(modelValue(world, again, "fridge_door"), open, 1e-9);
    CHECK(prefabUseWord(world.get<PrefabUses>(again)->uses[door]) == "close");
    // letting the holder go takes the awake copy with it
    releaseBatches(world, holder);
    CHECK(!world.alive(again));
}

// PEOPLE ON PREFAB SEATS (ADR-0522): a building's prefab copies list their free seats and beds in a fixed order; the
// city's own planner fills them as it fills the kit's; a taken seat is off the list (but still "free" to the one
// who holds it, so a plan made again takes the same seats); a seat that rocks makes its copy busy.
TEST_CASE(a_buildings_prefab_seats_take_the_citys_people_in_a_fixed_order) {
    World world;
    ModelStore store;
    Prefab chair, bed, rocker;
    CHECK(findPrefab("office_chair_adjustable", chair));
    CHECK(findPrefab("kids_bed", bed));
    CHECK(findPrefab("rocking_chair", rocker));
    const Entity holder = world.create();
    Transform at;
    for (int i = 0; i < 4; ++i) { at.position = Vec3(2.0 * i, 0, 0); CHECK(batchPrefab(world, holder, nullptr, chair, at, std::string(), nullptr, &store) == static_cast<uint32_t>(i)); }
    at.position = Vec3(0, 0, 5);
    const uint32_t bedCopy = batchPrefab(world, holder, nullptr, bed, at, std::string(), nullptr, &store);
    at.position = Vec3(4, 0, 5);
    const uint32_t rockCopy = batchPrefab(world, holder, nullptr, rocker, at, std::string(), nullptr, &store);
    CHECK(bedCopy == 4 && rockCopy == 5);

    std::vector<BatchSpot> spots;
    batchSpots(world, holder, spots);
    int seats = 0, beds = 0;
    for (std::size_t i = 0; i < spots.size(); ++i) {
        seats += spots[i].pose == 0;
        beds += spots[i].pose == 1;
        if (i > 0) CHECK(spots[i - 1].copy < spots[i].copy || (spots[i - 1].copy == spots[i].copy && spots[i - 1].use < spots[i].use));
    }
    CHECK(seats == 5 && beds == 1);
    // a seat is where its copy stands, and a bed's body lies above the bed
    CHECK(std::fabs(spots[0].at.m[0][3] - 0.0) < 0.5 && std::fabs(spots[1].at.m[0][3] - 2.0) < 0.5);
    for (const BatchSpot& sp : spots)
        if (sp.pose == 1) CHECK(sp.at.m[1][3] > 0.3 && sp.at.m[1][3] < 0.9);

    // THE PLANNER (interior_occupants.h): three people by day sit; nobody lies down. The same three seats each time.
    const Interactables none;
    const FurnitureLibrary& lib = FurnitureLibrary::global();
    OccupantPlan plan;
    plan.people = 3;
    plan.seed = 7;
    std::vector<BatchSpot> first = spots, second = spots;
    const std::vector<Occupant> a = planOccupants(none, lib, plan, &first), b = planOccupants(none, lib, plan, &second);
    CHECK(a.size() == 3 && b.size() == 3);
    for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
        CHECK(a[i].batchSpot >= 0 && a[i].batchSpot == b[i].batchSpot && a[i].pose == Occupant::Pose::Sit);
        CHECK(first[static_cast<std::size_t>(a[i].batchSpot)].taken);
    }
    // a second plan over the same list (another shop unit's people) does not share a seat with the first
    OccupantPlan more = plan;
    more.people = 2;
    more.seed = 8;
    const std::vector<Occupant> c = planOccupants(none, lib, more, &first);
    CHECK(c.size() == 2);
    for (const Occupant& o : c)
        for (const Occupant& taken : a) CHECK(o.batchSpot != taken.batchSpot);
    // at night at home the bed is taken first
    OccupantPlan night = plan;
    night.people = 1;
    night.night = true;
    std::vector<BatchSpot> forNight = spots;
    const std::vector<Occupant> asleep = planOccupants(none, lib, night, &forNight);
    CHECK(asleep.size() == 1 && asleep[0].pose == Occupant::Pose::Lie && forNight[static_cast<std::size_t>(asleep[0].batchSpot)].copy == bedCopy);

    // TAKING ONE: off the list for anyone else, still on it for whoever holds it
    const BatchSpot held = spots[static_cast<std::size_t>(a[0].batchSpot)];
    occupyBatchSpot(world, holder, held.copy, held.use, true);
    std::vector<BatchSpot> others, mineToo;
    batchSpots(world, holder, others);
    const std::vector<std::pair<uint32_t, uint32_t>> mine{{held.copy, held.use}};
    batchSpots(world, holder, mineToo, &mine);
    CHECK(others.size() == spots.size() - 1 && mineToo.size() == spots.size());
    occupyBatchSpot(world, holder, held.copy, held.use, false);
    others.clear();
    batchSpots(world, holder, others);
    CHECK(others.size() == spots.size());

    // A SEAT THAT ROCKS: taken, its copy is busy (it wakes and rocks only with the player near); woken, the seat's use
    // is on and rocking; let go, the copy is quiet again
    uint32_t rockUse = 99;
    for (const BatchSpot& sp : spots) if (sp.copy == rockCopy) rockUse = sp.use;
    CHECK(rockUse != 99);
    if (rockUse == 99) return;
    CHECK(world.get<PrefabBatches>(holder)->copies[rockCopy].busy == 0);
    occupyBatchSpot(world, holder, rockCopy, rockUse, true);
    CHECK(world.get<PrefabBatches>(holder)->copies[rockCopy].busy == 1);
    const Entity woken = wakeBatchCopy(world, holder, rockCopy);
    CHECK(woken.valid() && world.get<PrefabUses>(woken)->uses[rockUse].on && !batchCopySettled(world, woken));
    occupyBatchSpot(world, holder, rockCopy, rockUse, false);
    CHECK(world.get<PrefabBatches>(holder)->copies[rockCopy].busy == 0 && !world.get<PrefabUses>(woken)->uses[rockUse].on);
    releaseBatches(world, holder);
}

TEST_CASE(a_prefab_is_a_piece_the_floor_planner_can_place) {
    // The furniture library names some prefabs as pieces (furniture_library.lua, "PREFAB PIECES"): to the planner a
    // piece like any other, with a footprint, a family and tags; nothing of the kit's to draw.
    CHECK(ensureFurnitureLibraryLoaded());
    Piece dish = Piece::Count, desk = Piece::Count;
    CHECK(furniturePieceByName("dishwasher", dish));
    CHECK(furniturePieceByName("desk", desk));
    CHECK(pieceIsPrefab(dish) && !pieceIsPrefab(desk) && !pieceIsPrefab(Piece::Count));
    CHECK(static_cast<int>(dish) > kPieceCount && static_cast<int>(dish) < kPieceMax);
    CHECK(std::string(furniturePieceName(dish)) == "dishwasher");
    const FurniturePiece& fp = furniturePiece(dish, 0);
    CHECK_APPROX(fp.size.x, 0.60, 1e-9);
    CHECK_APPROX(fp.size.y, 0.85, 1e-9);
    CHECK(!fp.solid);
    for (const RenderMesh& m : fp.mesh) CHECK(m.vertices.empty());
    const FurnitureAsset* a = FurnitureLibrary::global().find(dish);
    CHECK(a != nullptr && a->family == "appliance" && a->verbs.empty());
    // It is a line in prefabs.lua of the same name: what is stood where the planner places it.
    Prefab p;
    CHECK(findPrefab("dishwasher", p));
    // A kit piece's name cannot be taken for a prefab piece, and a name once given keeps its number.
    CHECK(registerPrefabPiece("desk", Vec3(1, 1, 1)) == Piece::Count);
    CHECK(registerPrefabPiece("dishwasher", fp.size) == dish);
    // A kitchen's program asks for one by name.
    const RoomProgram* kitchen = FurnitureLibrary::global().program("kitchen");
    CHECK(kitchen != nullptr);
    bool asked = false;
    if (kitchen != nullptr)
        for (const FurnStep& st : kitchen->steps) asked = asked || st.pick.piece == dish;
    CHECK(asked);
}
