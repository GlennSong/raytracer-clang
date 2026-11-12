// The driving library's front door (#234): an empty world ticks; trips are begun, asked about, steered and ended;
// stale handles are refused; events come out sorted by car; the state hash is the same for the same calls; and the
// library includes nothing past its boundary (design 3).

#include "test_framework.h"

#include "apps/driving/driving.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr Real kTickSeconds = 0.1;
constexpr Real kLaneLength = 100;

// Two straight lanes, 100 m each, side by side in one section with a line a car may cross either way, open to the test
// car's tag: enough road for a destination and a hold, and a route to either lane from the first.
constexpr const char* kTestTag = "test_tag";
engine::LaneGraph twoLanes() {
    engine::LaneGraph g;
    g.vehicleTagNames = {kTestTag};
    engine::Section s;
    s.lanes.count = 2;
    s.lines.count = 1;
    for (uint32_t i = 0; i < 2; ++i) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(0, i * 4.0), engine::Vec2(kLaneLength, i * 4.0)}, {0, 0});
        p.length = kLaneLength;
        p.vehicleTags = 1;
        p.section = engine::SectionId(0);
        p.indexInSection = i;
        g.paths.push_back(p);
        g.sectionLanes.push_back(engine::PathId(i));
    }
    engine::LaneLine line;
    line.crossFromOffside = line.crossFromKerbside = true;
    g.laneLines.push_back(line);
    g.sections.push_back(s);
    return g;
}

PathPoint at(uint32_t path, Real along) {
    PathPoint p;
    p.path = engine::PathId(path);
    p.along = along;
    return p;
}

// A trip needs a placed car: gives `d` one vehicle class (an ordinary car's size) and places each of `ids` on lane 0,
// its front 10 m along.
constexpr Real kCarLength = 4.5, kCarWidth = 1.8, kCarWheelbase = 2.7, kCarSteerDegrees = 30;
constexpr Real kPlacedAlong = 10;
void placeCars(Driving& d, const std::vector<uint32_t>& ids) {
    if (d.vehicleClasses().empty()) {
        VehicleClasses classes;
        VehicleClass car = deriveVehicleClass("test_car", kCarLength, kCarWidth, kCarWheelbase, kCarSteerDegrees);
        car.tags = {kTestTag};
        classes.add(car);
        d.setVehicleClasses(classes);
    }
    for (uint32_t id : ids) CHECK(d.placeCar(CarId(id), 0, at(0, kPlacedAlong)));
}

// A fixed sequence of calls, the state hash after each tick. `variant` changes one call (another destination).
std::vector<uint64_t> runSequence(const engine::LaneGraph& g, int variant) {
    Driving d(&g);
    std::vector<uint64_t> hashes;
    placeCars(d, {3, 1});
    const TripId a = d.beginTrip(CarId(3), 1001, at(0, 50));
    const TripId b = d.beginTrip(CarId(1), 1002, at(1, 20));
    for (int t = 0; t < 30; ++t) {
        if (t == 3) d.changeDestination(a, at(1, 70));
        if (t == 5) d.changeDestination(b, at(0, variant == 0 ? 80 : 81));   // b lasts to the end
        if (t == 10) d.placeHold(b, 7, at(1, 40));
        if (t == 15) d.releaseHold(b, 7);
        if (t == 20) d.endTrip(a);
        d.tick(kTickSeconds);
        hashes.push_back(d.stateHash());
        if (t == 25) d.drainEvents();
    }
    return hashes;
}

}  // namespace

TEST_CASE(driving_empty_world_ticks) {
    Driving d;
    CHECK(d.roads() == nullptr);
    CHECK(d.tickCount() == 0u);
    for (int i = 0; i < 10; ++i) d.tick(kTickSeconds);
    CHECK(d.tickCount() == 10u);
    CHECK_APPROX(d.clockSeconds(), 10 * kTickSeconds, 1e-9);
    CHECK(d.tripCount() == 0u);
    CHECK(d.drainEvents().empty());
    // No road: nowhere to go, so every trip is refused.
    CHECK(!d.beginTrip(CarId(0), 1, at(0, 0)).valid());
}

TEST_CASE(driving_trip_begin_query_steer_end) {
    const engine::LaneGraph g = twoLanes();
    Driving d(&g);
    d.tick(kTickSeconds);
    // A car must be on the road before anyone drives it.
    CHECK(!d.beginTrip(CarId(4), 42, at(0, 30)).valid());
    placeCars(d, {4, 5});
    CHECK(!d.car(CarId(4))->driven);
    const TripId id = d.beginTrip(CarId(4), 42, at(0, 30));
    CHECK(id.valid());
    CHECK(d.car(CarId(4))->driven);
    CHECK(d.tripCount() == 1u);
    const Trip* t = d.trip(id);
    CHECK(t != nullptr);
    if (!t) return;
    CHECK(t->car == CarId(4));
    CHECK(t->person == 42u);
    CHECK(t->destination == at(0, 30));
    CHECK(t->beganAtTick == 1u);

    // One car, one trip at a time; a destination must be on the road.
    CHECK(!d.beginTrip(CarId(4), 43, at(1, 10)).valid());
    CHECK(!d.beginTrip(CarId(5), 43, at(2, 10)).valid());              // no such path
    CHECK(!d.beginTrip(CarId(5), 43, at(1, kLaneLength + 1)).valid()); // past its end
    CHECK(!d.beginTrip(CarId(5), 43, at(1, -1)).valid());              // before its start

    CHECK(d.changeDestination(id, at(1, 60)));
    CHECK(d.trip(id)->destination == at(1, 60));
    CHECK(!d.changeDestination(id, at(9, 0)));
    CHECK(d.trip(id)->destination == at(1, 60));

    CHECK(d.placeHold(id, 9, at(1, 40)));
    CHECK(d.placeHold(id, 2, at(1, 20)));
    CHECK(d.trip(id)->holds.size() == 2u);
    CHECK(d.trip(id)->holds[0].name == 2u);   // sorted by name
    CHECK(d.placeHold(id, 9, at(1, 45)));     // the same name moves the hold
    CHECK(d.trip(id)->holds.size() == 2u);
    CHECK(d.trip(id)->holds[1].at == at(1, 45));
    CHECK(d.releaseHold(id, 2));
    CHECK(!d.releaseHold(id, 2));             // already released
    CHECK(d.trip(id)->holds.size() == 1u);

    CHECK(d.endTrip(id));
    CHECK(d.tripCount() == 0u);
    CHECK(d.trip(id) == nullptr);
    const std::vector<Event> ev = d.drainEvents();
    CHECK(ev.size() == 1u);
    if (ev.size() == 1u) {
        CHECK(ev[0].kind == EventKind::EndedEarly);
        CHECK(ev[0].car == CarId(4));
        CHECK(ev[0].person == 42u);
        CHECK(ev[0].trip == id);
    }
    // The car is free again, and nobody is driving it.
    CHECK(!d.car(CarId(4))->driven);
    CHECK(d.beginTrip(CarId(4), 43, at(0, 10)).valid());
}

TEST_CASE(driving_stale_trip_handle_is_refused) {
    const engine::LaneGraph g = twoLanes();
    Driving d(&g);
    placeCars(d, {1, 2});
    const TripId old = d.beginTrip(CarId(1), 1, at(0, 10));
    CHECK(d.endTrip(old));
    // The next trip takes the freed slot; the old handle must not reach it.
    const TripId fresh = d.beginTrip(CarId(2), 2, at(0, 20));
    CHECK(fresh.valid());
    CHECK(fresh.index == old.index);
    CHECK(fresh != old);
    CHECK(d.trip(old) == nullptr);
    CHECK(!d.changeDestination(old, at(1, 5)));
    CHECK(!d.placeHold(old, 1, at(1, 5)));
    CHECK(!d.releaseHold(old, 1));
    CHECK(!d.endTrip(old));
    CHECK(!d.endTrip(TripId{}));   // the null handle too
    CHECK(d.trip(fresh) != nullptr && d.trip(fresh)->person == 2u);
}

TEST_CASE(driving_events_sorted_by_car_and_drained) {
    const engine::LaneGraph g = twoLanes();
    Driving d(&g);
    // Begun and ended in an order unlike car order.
    const uint32_t cars[] = {7, 2, 9, 0, 5};
    placeCars(d, {7, 2, 9, 0, 5});
    std::vector<TripId> trips;
    for (uint32_t c : cars) trips.push_back(d.beginTrip(CarId(c), 100 + c, at(0, 10)));
    for (std::size_t i = trips.size(); i-- > 0;) d.endTrip(trips[i]);
    d.tick(kTickSeconds);
    const std::vector<Event> ev = d.drainEvents();
    CHECK(ev.size() == trips.size());
    for (std::size_t i = 1; i < ev.size(); ++i) CHECK(ev[i - 1].car < ev[i].car);
    CHECK(d.drainEvents().empty());
}

TEST_CASE(driving_state_hash_same_calls_same_hash) {
    const engine::LaneGraph g = twoLanes();
    const std::vector<uint64_t> first = runSequence(g, 0);
    const std::vector<uint64_t> second = runSequence(g, 0);
    const std::vector<uint64_t> other = runSequence(g, 1);
    CHECK(first == second);
    CHECK(first.size() == other.size());
    // The same until the changed call, different from it on.
    for (std::size_t t = 0; t < first.size(); ++t) {
        if (t < 5) CHECK(first[t] == other[t]);
        else CHECK(first[t] != other[t]);
    }
    // Every tick moves the clock, so no two ticks share a hash.
    for (std::size_t t = 1; t < first.size(); ++t) CHECK(first[t] != first[t - 1]);
}

// THE BOUNDARY (design 3; issue #284). The library -- every file directly in src/apps/driving -- may include, outside
// itself, only the engine headers on an ALLOW-LIST, and those are FOLLOWED: everything they include in turn must be on
// the list too, so the entity world, the renderer, the city sim, the physics wrapper or Lua cannot come in through a
// header the library does include. (Before, a word list was matched against the library's own include lines only.) Its
// scripting/ folder, the one place that reads Lua, has its own list: the library's and the script VM's headers. Angle
// includes must be the standard library's (no folder, no ".h") or the JSON reader the bundle code uses.
namespace {

struct IncludeWalk {
    std::filesystem::path src;
    std::vector<std::string> allowed;        // headers outside src/apps/driving, as paths under src/
    std::vector<std::string> external;       // headers from outside src/ (a third-party include path), by name
    std::vector<std::string> seen;           // every file reached, as paths under src/
    std::vector<std::string> breaches;

    std::string underSrc(const std::filesystem::path& p) const {
        return std::filesystem::relative(std::filesystem::weakly_canonical(p), src).generic_string();
    }
    void walk(const std::filesystem::path& file) {
        const std::string me = underSrc(file);
        if (std::find(seen.begin(), seen.end(), me) != seen.end()) return;
        seen.push_back(me);
        std::ifstream in(file);
        std::string line;
        while (std::getline(in, line)) {
            const std::size_t inc = line.find("#include");
            if (inc == std::string::npos || line.find_first_not_of(" \t") != inc) continue;
            const std::size_t q = line.find('"', inc), a = line.find('<', inc);
            if (q != std::string::npos) {
                const std::string name = line.substr(q + 1, line.find('"', q + 1) - q - 1);
                std::filesystem::path found = file.parent_path() / name;
                if (!std::filesystem::exists(found)) found = src / name;
                if (!std::filesystem::exists(found)) {
                    if (std::find(external.begin(), external.end(), name) == external.end())
                        breaches.push_back(me + " includes \"" + name + "\", which is not under src/ nor allowed");
                    continue;
                }
                const std::string there = underSrc(found);
                const bool inLibrary = there.rfind("apps/driving/", 0) == 0;
                if (!inLibrary && std::find(allowed.begin(), allowed.end(), there) == allowed.end())
                    breaches.push_back(me + " includes " + there + ", which is not on the allow-list");
                else
                    walk(found);
            } else if (a != std::string::npos) {
                const std::string name = line.substr(a + 1, line.find('>', a + 1) - a - 1);
                const bool standard = name.find('/') == std::string::npos && name.find(".h") == std::string::npos;
                if (!standard && name != "nlohmann/json.hpp")
                    breaches.push_back(me + " includes <" + name + ">, which is not the standard library's");
            }
        }
    }
};

// The engine headers the library may include, and with them everything they include.
const std::vector<std::string> kLibraryAllowed = {
    "engine/procgen/city/roads/lane_graph.h",   // the road it drives on (read, never owned)
    "engine/procgen/city/polygon.h",            // ... which the lane graph's plan shapes use
    "engine/bundle/bundle.h",                   // the repo's hash (FNV) folded field by field
    "engine/bundle/binary_stream.h",            // ... which bundle.h brings
    "rt_math.h", "hash.h", "handle.h", "slot_map.h", "proc_rng.h",
};
// ... and the scripting folder's, besides: the Lua VM it reads rule sets with, and the placement policy's plain form.
const std::vector<std::string> kScriptingAllowed = {
    "engine/scripting/lua_state.h", "engine/scripting/script_vm.h", "engine/procgen/city/roads/lanes/control_plan.h",
    "engine/procgen/city/roads/lanes/road_graph_spec.h", "engine/procgen/city/road_spec.h",
    "engine/procgen/city/polyline.h",
};

std::vector<std::string> breachesOf(const std::filesystem::path& dir, const std::vector<std::string>& allowed,
                                    const std::vector<std::string>& external, std::size_t* files) {
    IncludeWalk w;
    w.src = std::filesystem::weakly_canonical(std::filesystem::path(RT_SOURCE_DIR) / "src");
    w.allowed = allowed;
    w.external = external;
    std::vector<std::filesystem::path> roots;
    for (const std::filesystem::directory_entry& e : std::filesystem::directory_iterator(dir))
        if (e.is_regular_file()) roots.push_back(e.path());
    std::sort(roots.begin(), roots.end());
    for (const std::filesystem::path& f : roots) w.walk(f);
    if (files) *files = w.seen.size();
    return w.breaches;
}

}  // namespace

TEST_CASE(driving_includes_nothing_past_its_boundary) {
    const std::filesystem::path dir = std::filesystem::path(RT_SOURCE_DIR) / "src" / "apps" / "driving";
    std::size_t files = 0;
    const std::vector<std::string> lib = breachesOf(dir, kLibraryAllowed, {}, &files);
    for (const std::string& b : lib) std::printf("    the library: %s\n", b.c_str());
    CHECK(lib.empty());
    CHECK(files > 40u);   // the library's files and the allowed headers, followed
    std::vector<std::string> scriptingAllowed = kLibraryAllowed;
    scriptingAllowed.insert(scriptingAllowed.end(), kScriptingAllowed.begin(), kScriptingAllowed.end());
    const std::vector<std::string> scripting =
        breachesOf(dir / "scripting", scriptingAllowed, {"lua.h", "lualib.h", "lauxlib.h"}, nullptr);
    for (const std::string& b : scripting) std::printf("    its scripting folder: %s\n", b.c_str());
    CHECK(scripting.empty());
    // The walk does see a breach: the entity world is not on the list.
    IncludeWalk probe;
    probe.src = std::filesystem::weakly_canonical(std::filesystem::path(RT_SOURCE_DIR) / "src");
    probe.allowed = kLibraryAllowed;
    probe.walk(probe.src / "engine" / "level_loader.h");
    CHECK(!probe.breaches.empty());
}
