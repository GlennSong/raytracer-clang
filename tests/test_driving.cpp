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

// Two straight lanes, 100 m each: enough road for a destination and a hold. Nothing here is driven on yet.
engine::LaneGraph twoLanes() {
    engine::LaneGraph g;
    for (int i = 0; i < 2; ++i) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(0, i * 4.0), engine::Vec2(kLaneLength, i * 4.0)}, {0, 0});
        p.length = kLaneLength;
        g.paths.push_back(p);
    }
    return g;
}

PathPoint at(uint32_t path, Real along) {
    PathPoint p;
    p.path = engine::PathId(path);
    p.along = along;
    return p;
}

// A fixed sequence of calls, the state hash after each tick. `variant` changes one call (another destination).
std::vector<uint64_t> runSequence(const engine::LaneGraph& g, int variant) {
    Driving d(&g);
    std::vector<uint64_t> hashes;
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
    const TripId id = d.beginTrip(CarId(4), 42, at(0, 30));
    CHECK(id.valid());
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
    // The car is free again.
    CHECK(d.beginTrip(CarId(4), 43, at(0, 10)).valid());
}

TEST_CASE(driving_stale_trip_handle_is_refused) {
    const engine::LaneGraph g = twoLanes();
    Driving d(&g);
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

// THE BOUNDARY (design 3): every file directly in src/apps/driving includes nothing from the city sim, the renderer,
// the entity world, the physics wrapper or Lua. (Its scripting/ folder, when it comes, is the one place that reads Lua.)
TEST_CASE(driving_includes_nothing_past_its_boundary) {
    const std::filesystem::path dir = std::filesystem::path(RT_SOURCE_DIR) / "src" / "apps" / "driving";
    const std::vector<std::string> forbidden = {"citysim", "renderer", "engine/world.h", "physics", "Jolt", "lua",
                                                "scripting"};
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_entry& e : std::filesystem::directory_iterator(dir))
        if (e.is_regular_file()) files.push_back(e.path());
    std::sort(files.begin(), files.end());
    CHECK(files.size() >= 2u);
    for (const std::filesystem::path& f : files) {
        std::ifstream in(f);
        std::string line;
        while (std::getline(in, line)) {
            const std::size_t inc = line.find("#include");
            if (inc == std::string::npos) continue;
            for (const std::string& word : forbidden) {
                if (line.find(word, inc) == std::string::npos) continue;
                std::printf("    %s includes past the boundary: %s\n", f.filename().string().c_str(), line.c_str());
                CHECK(false);
            }
        }
    }
}
