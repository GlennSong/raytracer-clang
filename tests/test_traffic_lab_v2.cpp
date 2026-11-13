// THE TRAFFIC LAB DRIVEN BY THE NEW MODEL (issues #255, #258, #235), and the proof that the old model is untouched.
//
// Loads real levels through the real LevelLoader and CityRenderSystem (a NullRenderer underneath, as every level test),
// steps the city the way the viewer does, and prints what it measured. Run one case at a time:
//
//   RT_TEST_FILTER=level_traffic_lab_v2_drives ./build-viewer/level_tests
//
// level_traffic_lab_v2_drives: the lab as it ships (citysim.driving "v2", rules/lab.lua) for five simulated minutes at
// the viewer's 60 steps a second. Every second it asks the wait-for graph who is waiting and why (a car waiting for no
// reason fails the test) and tracks how long each car has stood. Every tick the measurements' bodies_touching row tests
// every pair of drawn car bodies for overlap, whatever their paths, and sorts each spell by how their paths relate
// (issue #283; RT_LAB_TOUCH_LOG=1 lists every spell); none may touch. It measures queues emptying over junction lines and
// the longest wait at a line, and counts what drivers decided at junctions. Every two seconds it measures each car's
// tyres against the drawn road (CityRenderSystem::wheelGaps), sorted by where the car is: on a slope, a banked curve,
// over a crest or in a dip, or on the flat. At the end it prints the measurements table (driving/measure.h), `driving?`,
// `waiting?` and the cars that stood longest, with why. RT_LAB_LANEGRAPH_DUMP=<file> writes the lane graph the level
// drives on, as text.
//
// level_traffic_lab_v2_same_every_run: the lab loaded twice and run a minute each: the driving model's state hash is the
// same at every tick. (Across builds, the five-minute run pins its final state hash: kFiveMinuteStateHash.)
//
// level_traffic_lab_v2_soak (issue #265; only with RT_LAB_SOAK=1): thirty simulated minutes, reported per five-minute
// window -- trips arrived, mean speed by road class, the longest wait at a line and stand anywhere, loops, cars waiting
// for no reason, entries on red, overlaps of every kind, time at rest in a box by why, the tick's cost, and the model's
// working memory (room grants, junction plans, announcements, conflicts, trips) -- with throughput and memory checked
// for growth over the run; then the lab again, its state hash every minute equal to the first run's.
//
// level_traffic_lab_v2_refused_rules_leave_the_old_model_driving: a rule set that is refused (missing, or one with
// faults: RT_LAB_RULES=<file>) leaves the lab loading, the new model off and the old one driving.
//
// level_v1_state_is_unchanged: two levels driven by the OLD model alone -- the traffic lab with its citysim block set
// back to "v1", and river_town (walkers, buses, cabs, the far tiers) -- each stepped for a while, with one number
// folded over every agent's driving and walking state and every vehicle's pose, printed as it goes and CHECKED against
// the numbers pinned in the test (kV1LabHashes, kV1RiverHashes; issue #284): equal numbers say a v1 level does what it
// did when they were taken, on this machine's build.
//
// level_traffic_lab_v2_rebuild_hands_over_again: the city built a second time hands its drivers to the new model again,
// with the host's seats, counts and measurements started over (issue #284).

#include "test_framework.h"

#include "../src/apps/citysim/city_render.h"
#include "../src/apps/driving/measure.h"
#include "../src/apps/driving/traffic_state.h"
#include "../src/apps/driving/wait_graph.h"   // isWaiting
#include "../src/engine/asset_manager.h"
#include "../src/engine/bundle/bundle.h"   // fnv1a
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/world.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace engine;

namespace {

// Folds one value's bytes into the running hash.
template <typename T> void fold(uint64_t& h, const T& v) { h = bundle::fnv1a(&v, sizeof v, h); }

// ONE NUMBER OVER THE OLD MODEL'S STATE: every agent's place, heading, speed, route progress, lane, mode and goal, and
// every vehicle's pose, in index order.
uint64_t simStateHash(const citysim::CitySim& sim) {
    uint64_t h = bundle::kFnvOffset;
    for (const citysim::Agent& a : sim.agents()) {
        fold(h, a.pos.x);
        fold(h, a.pos.y);
        fold(h, a.heading.x);
        fold(h, a.heading.y);
        fold(h, a.speed);
        fold(h, a.distOnLeg);
        fold(h, a.leg);
        fold(h, a.lane);
        fold(h, a.laneF);
        fold(h, a.lateralOffset);
        fold(h, a.elevation);
        fold(h, static_cast<uint8_t>(a.moving));
        fold(h, static_cast<uint8_t>(a.mode));
        fold(h, static_cast<uint8_t>(a.state));
        fold(h, static_cast<uint8_t>(a.tier));
        fold(h, a.goal);
        fold(h, a.vehicle);
        fold(h, static_cast<uint64_t>(a.route.links.size()));
    }
    for (const citysim::SimVehicle& v : sim.vehicles()) {
        fold(h, v.pos.x);
        fold(h, v.pos.y);
        fold(h, v.heading.x);
        fold(h, v.heading.y);
        fold(h, v.driver);
    }
    return h;
}

// A LEVEL LOADED AND ITS CITY BUILT, kept together (the city reads the world).
struct LoadedCity {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    std::unique_ptr<RendererMeshUploader> uploader;
    std::unique_ptr<AssetManager> assets;
    World world;
    RenderView view;
    citysim::CityRenderSystem city;
    bool ok = false;

    // Loads `level`, sets its citysim block's driving model to `driving` and its rule set to `rules` (empty: as the
    // level says), lets `adjust` change anything else in the block (none: nothing), builds the city.
    LoadedCity(const std::string& level, const std::string& driving, const std::string& rules = std::string(),
               const std::function<void(CitySimConfig&)>& adjust = nullptr) {
        uploader = std::make_unique<RendererMeshUploader>(*renderer);
        assets = std::make_unique<AssetManager>(*uploader);
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/" + level, world, *renderer, view, *assets,
                               false))
            return;
        world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
            if (!driving.empty()) c.driving = driving;
            if (!rules.empty()) c.drivingRules = rules;
            if (adjust) adjust(c);
        });
        ok = city.build(world, assets.get(), nullptr);
    }
};

// Steps `city` `seconds` in steps of `dt`, printing the old model's state hash every `printEvery` seconds, and returns
// those hashes. `adjust`: as LoadedCity's.
std::vector<uint64_t> runAndHash(const std::string& level, const std::string& driving, Real seconds, Real dt,
                                 Real printEvery, const std::function<void(CitySimConfig&)>& adjust = nullptr) {
    std::vector<uint64_t> out;
    LoadedCity lc(level, driving, std::string(), adjust);
    CHECK(lc.ok);
    if (!lc.ok) return out;
    const int steps = static_cast<int>(seconds / dt + 0.5);
    const int every = static_cast<int>(printEvery / dt + 0.5);
    std::printf("    [v1-hash] %s (driving %s): %zu agents\n", level.c_str(), driving.empty() ? "as the level says" : driving.c_str(),
                lc.city.sim().agents().size());
    for (int i = 1; i <= steps; ++i) {
        lc.city.step(lc.world, dt);
        if (i % every == 0) {
            out.push_back(simStateHash(lc.city.sim()));
            std::printf("    [v1-hash] %s t=%6.1f s: 0x%016llxull\n", level.c_str(), i * dt,
                        static_cast<unsigned long long>(out.back()));
        }
    }
    return out;
}

// ---- The new model's lab ---------------------------------------------------------------------------------------------

constexpr Real kStep = 1.0 / 60.0;   // the viewer's fixed step, and the sim's tick on the lab (localHz 0)

// THE WAY EACH CAR CAME ONTO THE PATHS IT IS ON: for its last few front paths, the path it was on just before. Two cars
// overlapping on one path that came onto it from DIFFERENT paths met where the paths join (no merge rule yet); from
// the same path, the follower ran into the car ahead (following failed).
struct WayIn {
    static constexpr std::size_t kKept = 24;
    std::vector<std::pair<uint32_t, uint32_t>> entries;   // (path, the path before it), newest last
    void note(uint32_t path, uint32_t before) {
        if (!entries.empty() && entries.back().first == path) return;
        entries.push_back({path, before});
        if (entries.size() > kKept) entries.erase(entries.begin());
    }
    // The path it came onto `path` from; kNoIndex when unknown (it was put down there).
    uint32_t cameFrom(uint32_t path) const {
        for (std::size_t k = entries.size(); k-- > 0;)
            if (entries[k].first == path) return entries[k].second;
        return engine::kNoIndex;
    }
};

// WHAT THE FIVE-MINUTE RUN MAY SHOW, so a later change cannot make it worse unseen. Pair-ticks of overlap on the lane
// after a join: none, since first in goes first inside the junction (issue #263, ADR-0405: before it, a car that gave way
// and then went was crossed by the car it had given way to, one pair overlapped 44 ticks after a join, and junction 75's
// long connector 3010 drove through a car waiting on lane 766). Drivers that changed their minds twice on one approach:
// at most four (waits inside a junction are counted apart).
constexpr uint64_t kMostJoinPairTicks = 0;
constexpr uint64_t kMostChangedMindTwice = 4;
// BODY AGAINST BODY (issue #283): spells of two drawn bodies overlapping on the plan, any two cars, every tick, whatever
// their paths (measure.h TouchRow). None, in the five-minute run and in the soak. This replaced a once-a-second check
// here that measured bodies by the chord between their bumpers and held only crosses and joins: it saw the splits (two
// paths leaving one lane: 2 pairs in five minutes, 14 spells up to 2.8 m deep in thirty) and let them pass. Since #283 the
// car behind at a split follows the car ahead along the other path (traffic_state.h carAhead).
constexpr uint64_t kMostTouchSpells = 0;
constexpr uint64_t kMostTouchSpellsSoak = 0;
// LOOPS OF WAITING CARS (gridlock or deadlock), counted at the once-a-second look: none. Once the lights ran (#261) the
// same five cars at junctions 11 and 14 (a 6-7 m lane joins the two each way) twice entered a junction box they could not
// leave and stood in a ring; room beyond (#264) keeps a car out of a box until it has room to leave it.
constexpr uint64_t kMostLoopSamples = 0;
// AT REST IN A JUNCTION BOX (#264, the D5 brief's "done when"): car-seconds at rest with any part of the body in a box
// for any reason but a recorded give-way (or waiting inside for a far turn, or behind a car giving way in the box or
// moving off inside it: measure.h BoxReason).
constexpr double kMostBoxStopsNotAllowed = 0;
// THE LONGEST ANYONE MAY STAND (s): 100. History: 60 s before the lights ran; 100 s once they did (the longest cycle,
// 77 s, plus one green); held at 130 s by room beyond (#264) and at 180 s by #283, because a far turn at six-approach
// junction 21 gave way to the oncoming stream through two and three 7 s greens and stood 176-200 s.
// BACK TO 100 s since #284 (ADR-0410), for two reasons together:
//   - PROTECTED FAR TURNS: the rule set's `protected_when` warrants give a far turn at a junction of five or more
//     approaches its own green arrow (it never shares a green with the oncoming stream), so it is never starved of gaps.
//     Junction 21's cycle went 77 -> 93 s and 75's 77 -> 89 s; their far turns now stand at most one red (the longest
//     red-light stand measured 80 s).
//   - ANOTHER WAY: a car refused room beyond for room_refused_another_way (60 s) takes another way if one it may take
//     now exists (its turn green), and otherwise waits on. The longest stand measured is 92 s: car 257 at junction 75,
//     refused room behind a queue reaching back from the next light, then granted.
// The figure moves with the traffic (the same build with the another-way time at 30/45/60/90 s, before a driver took
// another way only onto a green, measured 169/92/118/88 s), so it is a limit with little to spare, not a target met
// with room: a change that moves the traffic may cross it and must say why.
constexpr double kLongestStand = 100.0;
// THE FIVE-MINUTE RUN'S FINAL STATE HASH (issue #284), pinned: the driving model's state after 18,000 ticks of the lab.
// Any change to what cars do changes it, so a change that moves the traffic must be acknowledged here, by updating this
// number in the same change and saying why in its report -- never left to drift. (Same every run within a build is the
// test below; this pins it across builds.) Checked only for the full five minutes (not with RT_LAB_SECONDS).
constexpr uint64_t kFiveMinuteStateHash = 0x986066aea5828fadull;   // #284 (ADR-0410)
// THE SOAK'S LIMITS (thirty minutes, issues #265 and #284):
// - THE LONGEST STAND: two of the lab's longest light cycles (kSoakStandCycles times RightOfWayCounts::longestCycle,
//   92.8 s at six-approach junction 21 since #284: 185.6 s). The arithmetic: a car that arrives just as its turn's
//   green ends waits out the rest of a cycle (at most one cycle); a queue ahead of it that does not all clear in one
//   green costs it at most one more. Refused room beyond for room_refused_another_way (60 s, under one cycle) it takes
//   another way if one is green, so that wait is inside the same two cycles. (Until #284 the limit was held at 1,130 s:
//   car 82 at junction 106 stood for good, refused room because its claim had to pass through junction 87 to junction
//   88's light, which never let a car from rest at 106 in as it got there. The router now never takes those turns one
//   after another -- the chain is closed to the class, not the turn: room.h chainsClosedForRoom -- and the measurements'
//   endless_waits row reports any wait that can never end, target none.) UNVERIFIED: the one soak run with #284's work
//   (before the fix for a chain claimed behind a car that stops between, room.cpp) formed a two-car gridlock at junctions
//   14/11 at t = 456 s and stood one car 1,346 s; that fix removes it over 500 s, but no soak has run since (it is run
//   once at the end of the milestone).
// - ENTRIES ON RED: none. Held at 1 until #284 (ADR-0409): a car committed through two close junctions (11 then 14) on
//   a prediction that the second light would let it in was held behind a slower car and arrived after the red. A claim
//   through close junctions now allows for the car ahead, needs its way seen clear of anything that could make it wait,
//   and is worked out afresh every tick until the first line.
// - DRIVERS THAT CHANGED THEIR MINDS TWICE on one approach, over the thirty minutes: its own limit (the five-minute
//   one, kMostChangedMindTwice, is for five). Measured 12 in the first soak (71e12216); see kSoakMostChangedMindTwice.
constexpr double kSoakStandCycles = 2.0;
constexpr uint64_t kSoakMostEnteredOnRed = 0;
constexpr uint64_t kSoakMostChangedMindTwice = 12;
// THE FASTEST ROADS FLOW (issue #281): on the lanes of the road class with the highest posted limit (the lab's
// freeway), the mean speed of cars in FREE FLOW -- held by nothing ahead (no car, no stop, not the end of their way),
// by no top speed of their own vehicle's (a moped or a bus that cannot do the limit), and cruising (at the top speed
// their rules give them, within kCruisingShare, not speeding up again out of a queue) -- is at least this share of the
// posted limit. Before #281 bends were measured from kinks and freeway cars crawled at about a quarter of it. The lab's
// freeway is a ring about 290 m round, which at the rule set's comfortable 2 m/s2 sideways allows 24 m/s, 83% of its
// 65 mph; cars leaving it slow for the ramps' tighter bends while still on its lanes. Measured: 24% before #281, 75%
// after (2026-11-12).
constexpr Real kFastestClassFreeFlowShare = 0.7;
constexpr Real kCruisingShare = 0.98;
constexpr Real kMetresPerSecondInAMph = 0.44704;   // for the printout

// SPEED BY WHERE A CAR IS: time-weighted over every driven car every tick.
struct SpeedTally {
    Real carSeconds = 0;
    Real metres = 0;
    void add(Real speed, Real dt) {
        carSeconds += dt;
        metres += speed * dt;
    }
    Real mean() const { return carSeconds > 0 ? metres / carSeconds : 0; }
};

// WHERE A CAR IS, for the tyre check: on a slope, a banked curve, over a crest or in a dip, or on the flat.
enum class Ground { Flat, Slope, Banked, Crest, Dip };
constexpr int kGroundCount = 5;
const char* groundWords(Ground g) {
    switch (g) {
        case Ground::Flat: return "on the flat";
        case Ground::Slope: return "on a slope (ramp)";
        case Ground::Banked: return "on a banked curve";
        case Ground::Crest: return "over a crest";
        case Ground::Dip: return "in a dip";
    }
    return "?";
}
constexpr Real kSlopeAbove = 0.02;        // a body pitched more than 2% is on a slope
constexpr Real kBankAbove = 0.01;         // ... rolled more than 1% is on a banked curve
constexpr Real kGradeChangeAbove = 0.01;  // the road's grade differs front to rear by more than 1%: a crest or a dip

// WHAT HELD A CAR, in words (its binding).
std::string bindingWords(const driving::Driving& d, const driving::Binding& b) {
    switch (b.by) {
        case driving::Limit::CarAhead: return "behind car " + std::to_string(b.car.index);
        case driving::Limit::Stop:
        case driving::Limit::TopSpeed:
            return std::string(b.by == driving::Limit::Stop ? "stopping for" : "top speed from") + " rule \"" +
                   (b.rule < d.rules().labels.rules.size() ? d.rules().labels.rules[b.rule] : std::string("?")) + "\"" +
                   (b.car.valid() ? " (car " + std::to_string(b.car.index) + ")" : std::string());
        case driving::Limit::WayEnd: return "at the end of its way";
        case driving::Limit::Nothing: return "free road";
    }
    return "?";
}

const char* pathKindWord(const engine::LaneGraph& roads, engine::PathId p) {
    return p.valid() && p.index < roads.paths.size() && roads.paths[p.index].kind == engine::PathKind::Connector
               ? "connector"
               : "lane";
}

// One side of a touch, in words.
std::string touchSideWords(const driving::Driving& d, const driving::TouchSide& s) {
    char b[256];
    std::snprintf(b, sizeof b, "car %u (%s) on %s %u at %.1f m, %.1f m/s, %s%s", s.car.index,
                  d.car(s.car) ? d.vehicleClasses().at(d.car(s.car)->vehicleClass).name.c_str() : "?",
                  pathKindWord(*d.roads(), s.path), s.path.index, s.along, s.speed,
                  s.driven ? bindingWords(d, s.binding).c_str() : "not driven", s.drawnBigger ? ", drawn bigger" : "");
    return b;
}

// BODY AGAINST BODY (issue #283): the breakdown by kind, and with RT_LAB_TOUCH_LOG set, every spell.
void printTouches(const driving::Driving& d, const driving::TouchRow& t) {
    std::printf("    [lab-v2] bodies touching on the plan, as drawn, every pair every tick: %llu spells over %llu "
                "pair-ticks, the deepest %.3f m; one car not driven in %llu, one at rest in %llu, one drawn bigger than "
                "its class in %llu; by how their paths relate (spells, pair-ticks, deepest):\n",
                static_cast<unsigned long long>(t.spells), static_cast<unsigned long long>(t.pairTicks), t.worstDepth,
                static_cast<unsigned long long>(t.spellsOneNotDriven), static_cast<unsigned long long>(t.spellsOneAtRest),
                static_cast<unsigned long long>(t.spellsDrawnBigger));
    for (uint8_t k = 0; k < driving::kTouchKindCount; ++k)
        std::printf("      %-90s %5llu %7llu %6.3f\n", driving::touchKindWords(static_cast<driving::TouchKind>(k)),
                    static_cast<unsigned long long>(t.spellsByKind[k]),
                    static_cast<unsigned long long>(t.pairTicksByKind[k]), t.worstByKind[k]);
    if (!std::getenv("RT_LAB_TOUCH_LOG")) return;
    std::vector<driving::TouchSpell> all = t.kept;
    for (const driving::TouchSpell& s : t.open()) all.push_back(s);
    for (const driving::TouchSpell& s : all)
        std::printf("      [touch] t=%.2f..%.2f (%.2f s) at (%.1f, %.1f) kind %s crossing %d junction %d band gap %.2f "
                    "depth %.3f | %s | %s\n",
                    s.firstSeconds, s.lastSeconds, s.seconds(), s.x, s.z, driving::touchKindKey(s.kind),
                    s.crossing == engine::kNoIndex ? -1 : static_cast<int>(s.crossing),
                    s.junction == engine::kNoIndex ? -1 : static_cast<int>(s.junction), s.bandGap, s.worstDepth,
                    touchSideWords(d, s.a).c_str(), touchSideWords(d, s.b).c_str());
}

struct WheelStats {
    int cars = 0;
    Real worstFloat = 0;   // highest wheel bottom above the road (m)
    Real worstSink = 0;    // lowest (negative: below the road)
    Real worstGraphOff = 0;   // the lane graph's height under the car's middle against the drawn road there (m)
};

}  // namespace

TEST_CASE(level_traffic_lab_v2_drives) {
    LoadedCity lc("traffic_lab.json", "");
    CHECK(lc.ok);
    if (!lc.ok) return;
    citysim::CityRenderSystem& city = lc.city;
    const citysim::CitySim& sim = city.sim();
    const driving::Driving* d = sim.drivingV2();
    CHECK(d != nullptr);
    if (!d) return;
    CHECK(d->hasRules());
    CHECK(sim.drivingHost().active());
    const engine::LaneGraph& roads = *d->roads();
    std::printf("    [lab-v2] at load: %s\n", sim.drivingHost().describe(*d).c_str());
    // RT_LAB_LANEGRAPH_DUMP=<file>: the lane graph the level drives on, as text (its path and crossing ids are the ones
    // every line below names; a graph built by lanes_tool from the plan alone may number them differently).
    if (const char* dumpTo = std::getenv("RT_LAB_LANEGRAPH_DUMP"))
        if (std::FILE* f = std::fopen(dumpTo, "w")) {
            const std::string text = engine::dumpText(roads);
            std::fwrite(text.data(), 1, text.size(), f);
            std::fclose(f);
        }
    // WHO GIVES WAY TO WHOM (issue #260): the table's counts, and one four-way's table in words, for checking by hand.
    std::printf("    [lab-v2] %s", driving::describeRightOfWay(d->rightOfWay()).c_str());
    for (uint32_t j = 0; j < roads.junctions.size(); ++j) {
        const driving::JunctionRules& jr = d->rightOfWay().junctions[j];
        if (jr.approaches.count != 4 || roads.junctions[j].connectors.count > 12) continue;
        std::printf("    [lab-v2] one four-way's table:\n%s", d->describeJunction(engine::JunctionId(j)).c_str());
        break;
    }
    // RT_LAB_JUNCTION=<id>: that junction's table in words too (for a person chasing one junction).
    if (const char* one = std::getenv("RT_LAB_JUNCTION"))
        if (static_cast<uint32_t>(std::atoi(one)) < roads.junctions.size())
            std::printf("    [lab-v2] junction %s's table:\n%s", one,
                        d->describeJunction(engine::JunctionId(static_cast<uint32_t>(std::atoi(one)))).c_str());
    // THE LIGHTS' PROGRAMMES (issue #261): how many phases the lit junctions have, and every programme's timing.
    std::map<uint32_t, uint32_t> byPhases;
    for (uint32_t j = 0; j < roads.junctions.size(); ++j)
        if (d->rightOfWay().junctions[j].running()) ++byPhases[d->rightOfWay().junctions[j].phases.count];
    std::printf("    [lab-v2] lights by phase count:");
    for (const std::pair<const uint32_t, uint32_t>& b : byPhases) std::printf(" %u phases x%u;", b.first, b.second);
    std::printf("\n");
    for (uint32_t j = 0; j < roads.junctions.size(); ++j)
        if (d->rightOfWay().junctions[j].running())
            std::printf("    [lab-v2] %s", driving::describeProgramme(roads, d->rightOfWay(), engine::JunctionId(j)).c_str());
    // Every lit junction, every phase and stage: no two movements whose paths meet let traffic in together, unless the rule
    // set lets one give way (a far turn on a plain green).
    // (Since #284 the model makes this check itself when the rules are handed over, and keeps what it found.)
    const std::vector<std::string> unsafe = driving::checkSignalProgrammes(roads, d->ruleSet(), d->rightOfWay());
    for (const std::string& u : unsafe) std::printf("      [unsafe programme] %s\n", u.c_str());
    CHECK(unsafe.empty());
    CHECK(d->signalProblems() == unsafe);
    // Opposing far turns at lights whose paths cross (D4's staged test 5, the shape the synthetic grid cannot make).
    uint32_t opposingCross = 0;
    for (uint32_t c = 0; c < roads.crossings.size(); ++c) {
        const engine::CrossingPoint& x = roads.crossings[c];
        if (x.otherSide != engine::OtherSide::Path || x.meeting != engine::Meeting::Cross || !x.junction.valid() ||
            !d->rightOfWay().junctions[x.junction.index].running())
            continue;
        const driving::ConnectorRules& ra = d->rightOfWay().paths[x.a.index];
        const driving::ConnectorRules& rb = d->rightOfWay().paths[x.other];
        if (ra.movement != driving::Movement::FarTurn || rb.movement != driving::Movement::FarTurn) continue;
        if (engine::dot(d->rightOfWay().approaches[ra.approach].heading, d->rightOfWay().approaches[rb.approach].heading) <
            driving::kFacingDot)
            ++opposingCross;
    }
    std::printf("    [lab-v2] opposing far turns at lights whose paths cross: %u pairs\n", opposingCross);
    // ROOM BEYOND (issue #264): the road-data report -- stretches some vehicle classes cannot stop in, and where they can
    // never pass (the brief's "report the place as a road-data problem").
    CHECK(d->roomStep());
    {
        const driving::RoomRoadReport rr = d->roomRoadReport();
        std::printf("    [lab-v2] room beyond, the road: %u stretches shorter than the longest vehicle needs; %u of them end "
                    "where everyone stops first or the road ends; %u pairs of turns through two lit junctions that let "
                    "a car arriving from rest through too seldom (closed to it):\n",
                    rr.shortStretches, rr.blockedByAStop, rr.turnPairsTooSeldom);
        for (const std::string& l : rr.lines)
            if (l.find("never") != std::string::npos || l.find("only once in") != std::string::npos)
                std::printf("      %s\n", l.c_str());
        uint64_t closed = 0, chains = 0;
        for (uint32_t k = 0; k < d->vehicleClasses().size(); ++k) {
            closed += d->routeClass(k).closed.size();
            chains += d->routeClass(k).closedChains.chains;
        }
        std::printf("    [lab-v2] turns closed to a class for want of room, summed over the classes: %llu; ways on (chains "
                    "of turns through close junctions) closed to a class: %llu\n",
                    static_cast<unsigned long long>(closed), static_cast<unsigned long long>(chains));
    }

    const Real seconds = std::getenv("RT_LAB_SECONDS") ? std::atof(std::getenv("RT_LAB_SECONDS")) : 300.0;
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    const int perSecond = static_cast<int>(1.0 / kStep + 0.5);
    std::map<uint32_t, Real> standSince;                  // car -> when it began to stand (driven cars only)
    std::map<uint32_t, std::pair<Real, std::string>> longest;   // car -> its longest stand, and why (at the end of it)
    uint64_t noReason = 0, loopSamples = 0, waitSamples = 0;
    WheelStats wheels[kGroundCount];
    Real worstGap = 0, steepestPitch = 0, steepestRoll = 0;
    std::vector<Real> gaps;
    std::vector<Vec3> bottoms;
    const auto why = [&](const driving::Car& c) -> std::string {
        const driving::Binding& b = c.binding;
        switch (b.by) {
            case driving::Limit::CarAhead: return "behind car " + std::to_string(b.car.index);
            case driving::Limit::Stop:
            case driving::Limit::TopSpeed:
                return std::string(b.by == driving::Limit::Stop ? "stopping for" : "top speed from") + " rule \"" +
                       (b.rule < d->rules().labels.rules.size() ? d->rules().labels.rules[b.rule] : std::string("?")) + "\"";
            case driving::Limit::WayEnd: return "at the end of its way";
            case driving::Limit::Nothing: return "nothing (a bug)";
        }
        return "?";
    };

    // Same-path overlaps, every tick, from each path's list of cars (as the measurements table finds them): a SPELL is a
    // pair overlapping on a path that did not on the tick before, and each is put down to a join (the two came onto the
    // path from different paths) or to following (the same way in).
    std::map<uint32_t, WayIn> wayIn;
    std::map<uint32_t, uint32_t> lastFront;
    std::set<std::pair<uint32_t, uint32_t>> overlappingBefore, overlappingNow;
    uint64_t joinPairTicks = 0, followPairTicks = 0, joinSpells = 0, followSpells = 0;
    Real worstFollowOverlap = 0;
    std::set<std::pair<uint32_t, uint32_t>> joinPairs, followPairs;
    // QUEUES EMPTYING AT JUNCTION LINES: a car's front leaving an arriving lane for a connector crosses that lane's line;
    // the queue figures come from those crossings (measure.h findQueueBursts), as the measurements table reads them.
    struct RestSeen {
        bool atRest = true;
        double restLeftAt = -1, restOdometer = 0;
    };
    std::map<uint32_t, RestSeen> rest;
    std::map<uint32_t, std::vector<driving::LineCrossing>> lineCrossings;   // by arriving lane
    std::map<uint32_t, uint32_t> frontPath;
    // AT LIGHTS (issue #261): fronts crossing a line where the lights run, by what the turn's lamp showed; and how long
    // each such car stood on the lane it arrived by (from its first standing on it to crossing; queueing included).
    uint64_t litCrossings = 0, enteredOnRed = 0, turnedOnRed = 0, enteredOnYellow = 0;
    Real litWaitSum = 0, litLongestWait = 0;
    std::map<uint32_t, std::pair<uint32_t, Real>> stoodOnLane;   // car -> (lane, seconds stood on it)
    constexpr int kRedRunnersShown = 3;
    int redRunnersShown = 0;
    // THE LONGEST WAIT AT A JUNCTION'S LINE: a car standing with its front within 2 m of the end of a lane that leads into
    // a connector, its spell from coming to rest there to moving on.
    constexpr Real kAtTheLine = 2.0;
    std::set<std::pair<uint32_t, uint64_t>> dithered;   // (car, its plan's tick) already shown
    std::map<uint32_t, Real> lineWaitSince;
    Real longestLineWait = 0;
    uint32_t longestLineWaitCar = 0;
    std::string longestLineWaitWhy;
    // SPEED BY ROAD CLASS (issue #281): lanes by their section's class, connectors as one more row; and on the class with
    // the highest posted limit, free flow as a share of the limit, and what held its cars.
    std::map<std::string, SpeedTally> byClass;
    SpeedTally everyone;
    NameIndex fastestClass = kNoName;
    Real fastestLimit = 0;
    for (const engine::Path& p : roads.paths)
        if (p.kind == engine::PathKind::Lane && p.section.valid() && p.speedLimit > fastestLimit) {
            fastestLimit = p.speedLimit;
            fastestClass = roads.sections[p.section.index].roadClass;
        }
    Real freeShareSum = 0, freeSeconds = 0, fastestSeconds = 0;
    // what held cars on the fastest class's lanes: car-seconds, and their speed over the posted limit summed over them
    std::map<std::string, std::pair<Real, Real>> fastestHeldBy;
    for (int i = 1; i <= steps; ++i) {
        city.step(lc.world, kStep);
        const Real now = i * kStep;
        for (const driving::Car& c : d->cars()) {
            if (!c.driven) continue;
            const engine::Path& p = roads.paths[c.path.index];
            const bool lane = p.kind == engine::PathKind::Lane && p.section.valid();
            const NameIndex cls = lane ? roads.sections[p.section.index].roadClass : kNoName;
            byClass[lane && cls < roads.roadClassNames.size() ? "lanes, " + roads.roadClassNames[cls]
                                                              : std::string("connectors (inside junctions)")]
                .add(c.speed, kStep);
            everyone.add(c.speed, kStep);
            if (!lane || cls != fastestClass) continue;
            if (!(p.speedLimit > 0)) continue;
            fastestSeconds += kStep;
            const driving::Limit by = c.binding.by;
            const bool ownTop = by == driving::Limit::TopSpeed && c.binding.rule == driving::kNoRule;
            const std::string held = why(c);
            std::pair<Real, Real>& h = fastestHeldBy[held.rfind("behind car", 0) == 0 ? std::string("behind a car")
                                                     : ownTop ? std::string("its own vehicle's top speed")
                                                              : held];
            h.first += kStep;
            h.second += c.speed / p.speedLimit * kStep;
            if (by == driving::Limit::CarAhead || by == driving::Limit::Stop || by == driving::Limit::WayEnd || ownTop ||
                c.speed < kCruisingShare * c.held.topSpeed)
                continue;
            freeShareSum += c.speed / p.speedLimit * kStep;
            freeSeconds += kStep;
        }
        // RT_LAB_WATCH=<car>,<from s>,<to s>: that car's card and the announcements on its junctions' crossing points,
        // every tick of the window (for a person chasing one decision).
        if (const char* watch = std::getenv("RT_LAB_WATCH")) {
            unsigned wc = 0;
            double from = 0, to = 0;
            if (std::sscanf(watch, "%u,%lf,%lf", &wc, &from, &to) == 3 && now >= from && now <= to &&
                d->car(driving::CarId(wc))) {
                std::string stale;
                const bool holds = d->waitReasonHolds(*d->car(driving::CarId(wc)), &stale);
                std::printf("---- t=%.3f%s%s\n%s", now, holds ? "" : " | its reason no longer applies: ",
                            holds ? "" : stale.c_str(), d->describeCar(driving::CarId(wc)).c_str());
                const driving::JunctionPlans* plans = d->junctionPlans(driving::CarId(wc));
                for (std::size_t k = 0; plans && k < plans->count; ++k) {
                    const engine::Path& p = roads.paths[plans->plans[k].connector.index];
                    for (uint32_t x = p.crossings.first; x < p.crossings.end(); ++x)
                        for (const driving::Announcement& a : d->board().on(roads.pathCrossings[x]))
                            std::printf("  board: crossing %u car %llu path %u arrive %.2f clear %.2f going %d fixed %d "
                                        "at tick %llu notGoing %d\n",
                                        a.crossing.index, static_cast<unsigned long long>(a.who), a.onPath.index,
                                        a.arriveIn, a.clearIn, a.going, a.arrivalFixed,
                                        static_cast<unsigned long long>(a.arrivalTick), a.notGoing);
                }
            }
        }
        for (const driving::Car& c : d->cars()) {
            if (!c.driven) continue;
            RestSeen& r = rest[c.id.index];
            const bool atRest = c.speed < driving::kStandingBelow;
            if (r.atRest && !atRest) {
                r.restLeftAt = now - kStep;
                r.restOdometer = c.odometer;
            }
            r.atRest = atRest;
            std::map<uint32_t, uint32_t>::iterator fp = frontPath.find(c.id.index);
            if (fp != frontPath.end() && fp->second != c.path.index &&
                roads.paths[c.path.index].kind == engine::PathKind::Connector &&
                roads.paths[c.path.index].from.index == fp->second) {
                driving::LineCrossing x;
                x.car = c.id;
                x.tick = static_cast<uint64_t>(i);
                x.seconds = now;
                x.restLeftAt = r.restLeftAt;
                x.restDistance = (c.odometer - c.along) - r.restOdometer;
                lineCrossings[fp->second].push_back(x);
                // AT LIGHTS (issue #261): what the car's turn showed as the tick began (what its driver decided by), and
                // how long it stood on the lane it arrived by.
                const driving::Lamp lamp =
                    driving::lampAt(roads, d->rightOfWay(), c.path, d->clockSeconds() - kStep);
                if (lamp != driving::Lamp::Dark) {
                    ++litCrossings;
                    if (lamp == driving::Lamp::Red) {
                        ++enteredOnRed;
                        if (redRunnersShown < kRedRunnersShown) {
                            ++redRunnersShown;
                            std::printf("      [red-runner] t=%.2f car %u onto connector %u at %.1f m/s:\n%s", now, c.id.index,
                                        c.path.index, c.speed, d->describeCar(c.id).c_str());
                        }
                    }
                    if (lamp == driving::Lamp::RedMayTurn) ++turnedOnRed;
                    if (lamp == driving::Lamp::Yellow) ++enteredOnYellow;
                    std::map<uint32_t, std::pair<uint32_t, Real>>::iterator st = stoodOnLane.find(c.id.index);
                    const Real stood = st != stoodOnLane.end() && st->second.first == fp->second ? st->second.second : 0;
                    litWaitSum += stood;
                    litLongestWait = std::max(litLongestWait, stood);
                }
                stoodOnLane.erase(c.id.index);
            }
            if (roads.paths[c.path.index].kind == engine::PathKind::Lane && c.speed < driving::kStandingBelow) {
                std::pair<uint32_t, Real>& st = stoodOnLane[c.id.index];
                if (st.first != c.path.index) st = {c.path.index, 0};
                st.second += kStep;
            }
            frontPath[c.id.index] = c.path.index;
            const engine::Path& on = roads.paths[c.path.index];
            const bool atLine = on.kind == engine::PathKind::Lane && on.next.count > 0 &&
                                roads.paths[roads.nextPaths[on.next.first].index].kind == engine::PathKind::Connector &&
                                on.length - c.along <= kAtTheLine && driving::isWaiting(c);
            std::map<uint32_t, Real>::iterator lw = lineWaitSince.find(c.id.index);
            if (atLine && lw == lineWaitSince.end()) lineWaitSince[c.id.index] = now;
            if (atLine && lw != lineWaitSince.end() && now - lw->second > longestLineWait) {
                longestLineWait = now - lw->second;
                longestLineWaitCar = c.id.index;
                longestLineWaitWhy = why(c);
            }
            if (!atLine && lw != lineWaitSince.end()) lineWaitSince.erase(lw);
        }
        for (const driving::Car& c : d->cars()) {
            std::map<uint32_t, uint32_t>::iterator lf = lastFront.find(c.id.index);
            if (lf != lastFront.end() && lf->second == c.path.index) continue;
            WayIn& w = wayIn[c.id.index];
            for (std::size_t k = c.trailCount; k-- > 1;) w.note(c.trail[k - 1].index, c.trail[k].index);
            w.note(c.path.index, c.trailCount > 0 ? c.trail[0].index
                                 : lf != lastFront.end() ? lf->second
                                                         : engine::kNoIndex);
            lastFront[c.id.index] = c.path.index;
        }
        overlappingNow.clear();
        for (uint32_t p = 0; p < roads.paths.size(); ++p) {
            const driving::PathList list = d->pathLists().on(engine::PathId(p));
            for (std::size_t k = 0; k + 1 < list.size(); ++k) {
                if (!(list[k + 1].rear < list[k].front - 1e-6)) continue;
                const uint32_t ca = list[k].car.index, cb = list[k + 1].car.index;
                const std::pair<uint32_t, uint32_t> key(std::min(ca, cb), std::max(ca, cb));
                if (!overlappingNow.insert(key).second) continue;
                const uint32_t fromA = wayIn[ca].cameFrom(p), fromB = wayIn[cb].cameFrom(p);
                const bool join = fromA != engine::kNoIndex && fromB != engine::kNoIndex && fromA != fromB;
                const bool fresh = overlappingBefore.count(key) == 0;
                if (join) {
                    ++joinPairTicks;
                    joinSpells += fresh;
                    joinPairs.insert(key);
                } else {
                    ++followPairTicks;
                    followSpells += fresh;
                    followPairs.insert(key);
                    worstFollowOverlap = std::max(worstFollowOverlap, list[k].front - list[k + 1].rear);
                    if (fresh && std::getenv("RT_LAB_OVERLAP_LOG"))
                        std::printf("      [follow-overlap] t=%.2f cars %u and %u on path %u, came from %u and %u\n", now, ca,
                                    cb, p, fromA, fromB);
                }
            }
        }
        overlappingBefore.swap(overlappingNow);
        // RT_LAB_CANNOT_COMPLY_LOG: every tick, each conflict whose car going second cannot stop short of the crossing
        // point it is not yet on (issue #284; ConflictCounts::secondCannotStop), with the question that decided it.
        if (std::getenv("RT_LAB_CANNOT_COMPLY_LOG"))
            for (const driving::SettledConflict& s : d->resolver().conflicts()) {
                const driving::Side& second = s.resolution.firstGoes ? s.conflict.second : s.conflict.first;
                const driving::Side& first = s.resolution.firstGoes ? s.conflict.first : s.conflict.second;
                if (!second.cannotStop || second.onIt) continue;
                std::printf("      [cannot comply] t=%.2f group %u: car %llu (on it %d, cannot stop %d, committed %d) "
                            "goes before car %llu, by %s\n",
                            now, s.group, static_cast<unsigned long long>(first.who), first.onIt, first.cannotStop,
                            first.committed, static_cast<unsigned long long>(second.who),
                            driving::questionName(s.resolution.by));
            }
        if (i % perSecond != 0) continue;

        // A DRIVER THAT CHANGED ITS MIND MORE THAN ONCE on one approach: its card, the first time, when asked.
        if (std::getenv("RT_LAB_DITHER_LOG"))
            for (const driving::Car& c : d->cars()) {
                const driving::JunctionPlans* plans = d->junctionPlans(c.id);
                for (std::size_t k = 0; plans && k < plans->count; ++k) {
                    const driving::JunctionPlan& p = plans->plans[k];
                    if (p.mindChanges < 2 || !dithered.insert({c.id.index, p.madeAtTick}).second) continue;
                    std::printf("      [dither] t=%.0f car %u at junction %u:\n%s", now, c.id.index, p.junction.index,
                                d->describeCar(c.id).c_str());
                }
            }
        // WHO IS WAITING, AND WHY: a car waiting for no reason is the fault this model exists to remove.
        const driving::WaitReport wr = d->waitReport();
        noReason += wr.noReason.size();
        loopSamples += wr.loops.size();
        waitSamples += wr.waiting;
        // A loop's first sighting: who is in it, and each car's card (for a person chasing it).
        if (!wr.loops.empty() && loopSamples == wr.loops.size()) {
            std::printf("      [loop] t=%.0f\n%s", now, d->describeWaiting(wr).c_str());
            for (const driving::WaitLoop& l : wr.loops)
                for (const driving::CarId car : l.cars) std::printf("%s", d->describeCar(car).c_str());
        }
        // HOW LONG EACH CAR HAS STOOD (waiting, as the wait-for graph means it: at rest and not moving off), and what
        // held it at the end of the stand.
        for (const driving::Car& c : d->cars()) {
            const bool standing = driving::isWaiting(c);
            std::map<uint32_t, Real>::iterator it = standSince.find(c.id.index);
            if (standing && it == standSince.end()) it = standSince.insert({c.id.index, now}).first;
            if (standing) {
                const Real stood = now - it->second;
                // RT_LAB_LONG_CARDS=<s>: the card of a car the moment it has stood that long (for a person chasing why).
                if (const char* longCards = std::getenv("RT_LAB_LONG_CARDS"))
                    if (stood >= std::atof(longCards) && stood < std::atof(longCards) + 1)
                        std::printf("      [long stand] t=%.0f car %u has stood %.0f s:\n%s", now, c.id.index, stood,
                                    d->describeCar(c.id).c_str());
                std::map<uint32_t, std::pair<Real, std::string>>::iterator l = longest.find(c.id.index);
                if (l == longest.end() || stood >= l->second.first) longest[c.id.index] = {stood, why(c)};
            }
            if (!standing && it != standSince.end()) standSince.erase(it);
        }
        // (Bodies touching on the plan, any two cars, are measured every tick by the measurements' bodies_touching row.)
        // THE TYRES AGAINST THE DRAWN ROAD, every ten seconds.
        if (i % (perSecond * 2) == 0) {
            for (std::size_t ai = 0; ai < sim.agents().size(); ++ai) {
                const int agent = static_cast<int>(ai);
                Real from = 0, to = 0;
                if (!sim.drivingHost().roadHeight(agent, from, to)) continue;
                Mat4 m;
                if (!city.wheelGaps(agent, gaps, &m, &bottoms)) continue;
                const driving::Car* c = d->car(driving::CarId(static_cast<uint32_t>(sim.agents()[ai].vehicle)));
                if (!c) continue;
                const Real pitch = m.m[1][2], roll = m.m[1][0];   // the climb of the body's forward and sideways axes
                const driving::Body body = driving::layBody(roads, *c, d->vehicleClasses().at(c->vehicleClass).length);
                const Real gradeFront = roads.gradeAt(c->path, c->along);
                const Real gradeRear = body.count > 0 ? roads.gradeAt(body.pieces[body.count - 1].path,
                                                                      std::max(Real(0), body.pieces[body.count - 1].rear))
                                                      : gradeFront;
                Ground g = Ground::Flat;
                if (gradeFront < gradeRear - kGradeChangeAbove) g = Ground::Crest;
                else if (gradeFront > gradeRear + kGradeChangeAbove) g = Ground::Dip;
                else if (std::fabs(roll) > kBankAbove) g = Ground::Banked;
                else if (std::fabs(pitch) > kSlopeAbove) g = Ground::Slope;
                WheelStats& w = wheels[static_cast<int>(g)];
                ++w.cars;
                Real drawnMid = 0;
                for (std::size_t k = 0; k < gaps.size(); ++k) {
                    w.worstFloat = std::max(w.worstFloat, gaps[k]);
                    w.worstSink = std::min(w.worstSink, gaps[k]);
                    worstGap = std::max(worstGap, std::fabs(gaps[k]));
                    drawnMid += bottoms[k].y - gaps[k];
                }
                drawnMid /= static_cast<Real>(gaps.size());
                w.worstGraphOff = std::max(w.worstGraphOff, std::fabs(to - drawnMid));
                steepestPitch = std::max(steepestPitch, std::fabs(pitch));
                steepestRoll = std::max(steepestRoll, std::fabs(roll));
                if (std::getenv("RT_LAB_WHEEL_LOG") && *std::min_element(gaps.begin(), gaps.end()) < -0.05) {
                    std::printf("      [wheels] t=%.0f agent %d car %u path %u along %.2f: road %.3f..%.3f, drawn at %.3f %.3f "
                                "%.3f;",
                                now, agent, c->id.index, c->path.index, c->along, from, to, m.m[0][3], m.m[1][3], m.m[2][3]);
                    for (std::size_t k = 0; k < gaps.size(); ++k)
                        std::printf(" (%.2f %.3f %.2f gap %+.3f)", bottoms[k].x, bottoms[k].y, bottoms[k].z, gaps[k]);
                    std::printf("\n");
                }
            }
        }
        if (i % (perSecond * 60) == 0)
            std::printf("    [lab-v2] t=%5.0f s: state hash %016llx | %s\n", now,
                        static_cast<unsigned long long>(d->stateHash()), sim.drivingHost().describe(*d).c_str());
    }

    std::printf("    [lab-v2] driving?: %s\n", sim.drivingHost().describe(*d).c_str());
    std::printf("    [lab-v2] waiting?: %s\n", d->describeWaiting(d->waitReport()).c_str());
    // THE DRIVERS' JUNCTION PLANS (issue #260): how many were made and dropped, the most crossing points on one, the caps.
    {
        const driving::JunctionPlanCounts& pc = d->junctionPlanCounts();
        const driving::CapCounts& caps = d->capCounts();
        std::printf("    [lab-v2] junction plans: %llu made; dropped %llu through, %llu as the way changed, %llu as the "
                    "route changed; the most crossing points on one plan %u (connector %u); the most junctions planned "
                    "at once %u; caps hit: %llu plans cut, %llu waiting points cut; plan hash %016llx\n",
                    static_cast<unsigned long long>(pc.made), static_cast<unsigned long long>(pc.cleared),
                    static_cast<unsigned long long>(pc.wayChanged), static_cast<unsigned long long>(pc.routeChanged),
                    pc.mostCrossings, pc.mostCrossingsConnector.index, pc.mostAtOnce,
                    static_cast<unsigned long long>(caps.junctionPlansCut),
                    static_cast<unsigned long long>(caps.waitingPointsCut), static_cast<unsigned long long>(d->planHash()));
        // GIVING WAY (issue #262): what drivers decided at junctions, and the board.
        std::printf("    [lab-v2] give way: %llu decisions to go, %llu to give way; %llu went after giving way, taking gaps "
                    "of %.2f-%.2f s (the smallest needed %.2f s); %llu plans where the driver changed its mind more than "
                    "once; %llu waits inside a junction; %llu stops on a lane to give way (#284); the board held at most "
                    "%zu announcements\n",
                    static_cast<unsigned long long>(pc.decidedGo), static_cast<unsigned long long>(pc.decidedGiveWay),
                    static_cast<unsigned long long>(pc.wentAfterGivingWay), pc.smallestGapTaken, pc.largestGapTaken,
                    pc.smallestGapNeeded, static_cast<unsigned long long>(pc.changedMindTwice),
                    static_cast<unsigned long long>(pc.waitedInside), static_cast<unsigned long long>(pc.gaveWayOnALane),
                    d->board().most());
        // ANOTHER WAY (issue #284): drivers refused room for long that asked for a way avoiding that turn, and took one.
        std::printf("    [lab-v2] another way: %llu drivers refused room for room_refused_another_way asked for one, %llu "
                    "took one\n",
                    static_cast<unsigned long long>(pc.askedAnotherWay), static_cast<unsigned long long>(pc.tookAnotherWay));
        // WHO GOES FIRST (issue #263): the conflicts the one resolver settled over the run, by the question that decided.
        const driving::ConflictCounts& cc = d->conflictCounts();
        std::printf("    [lab-v2] who goes first: %llu conflicts settled (summed over the ticks); by question:",
                    static_cast<unsigned long long>(cc.conflicts));
        for (uint8_t q = 0; q < driving::kQuestionCount; ++q)
            if (cc.byQuestion[q] > 0)
                std::printf(" %s %llu", driving::questionName(static_cast<driving::Question>(q)),
                            static_cast<unsigned long long>(cc.byQuestion[q]));
        std::printf("; rings broken %llu (untied at a queue %llu; of bodies alone, left standing %llu); conflicts whose "
                    "car going second could not stop short (#284) %llu\n",
                    static_cast<unsigned long long>(cc.rings), static_cast<unsigned long long>(cc.physicalRings),
                    static_cast<unsigned long long>(cc.bodyRings), static_cast<unsigned long long>(cc.secondCannotStop));
        // ROOM BEYOND (issue #264): asks, grants and refusals over the run.
        const driving::RoomCounts& rc = d->roomCounts();
        std::printf("    [lab-v2] room beyond: %llu asks settled, %llu granted (%llu through junctions too close to stop "
                    "between, %llu to wait inside); refused:",
                    static_cast<unsigned long long>(rc.asked), static_cast<unsigned long long>(rc.granted),
                    static_cast<unsigned long long>(rc.grantedThrough),
                    static_cast<unsigned long long>(rc.grantedToWaitInside));
        for (uint8_t k = 1; k < driving::kRoomRefusalCount; ++k)
            if (rc.refused[k] > 0)
                std::printf(" %llu %s;", static_cast<unsigned long long>(rc.refused[k]),
                            driving::roomRefusalWords(static_cast<driving::RoomRefusal>(k)));
        std::printf(" grants used on the lane %llu, given back %llu (%llu claims through close junctions checked again and "
                    "no longer holding, #284); the most held at once %u; asks of one tick put in order by the resolver "
                    "%llu; road-data places refused %zu\n",
                    static_cast<unsigned long long>(rc.usedOnLane), static_cast<unsigned long long>(rc.givenBack),
                    static_cast<unsigned long long>(rc.chainsGivenBack), rc.mostHeld,
                    static_cast<unsigned long long>(rc.orderedByResolver), rc.roadDataPlaces.size());
        for (const std::pair<uint32_t, uint32_t>& place : rc.roadDataPlaces)
            std::printf("      [road data] refused room at junction %u, lane %u beyond\n", place.first, place.second);
        // One driver's card giving way at a line, as `driver? <id>` shows it.
        for (const driving::Car& c : d->cars()) {
            if (c.binding.by != driving::Limit::Stop || c.binding.stopFor != driving::StopFor::GivingWay) continue;
            std::printf("    [lab-v2] driver? %u (giving way):\n%s", c.id.index, d->describeCar(c.id).c_str());
            break;
        }
        // One driver's card with a junction ahead, as `driver? <id>` shows it.
        for (const driving::Car& c : d->cars()) {
            const driving::JunctionPlans* plans = d->junctionPlans(c.id);
            if (!plans || plans->count == 0 || plans->plans[0].crossingCount < 4) continue;
            std::printf("    [lab-v2] driver? %u:\n%s", c.id.index, d->describeCar(c.id).c_str());
            break;
        }
    }
    std::printf("    [lab-v2] sampled each second: %llu cars waiting in all, %llu with no reason, %llu loops\n",
                static_cast<unsigned long long>(waitSamples), static_cast<unsigned long long>(noReason),
                static_cast<unsigned long long>(loopSamples));
    // The cars that stood longest, with why.
    std::vector<std::pair<Real, std::pair<uint32_t, std::string>>> stands;
    for (const std::pair<const uint32_t, std::pair<Real, std::string>>& l : longest)
        stands.push_back({l.second.first, {l.first, l.second.second}});
    std::sort(stands.begin(), stands.end(), [](const std::pair<Real, std::pair<uint32_t, std::string>>& x,
                                               const std::pair<Real, std::pair<uint32_t, std::string>>& y) {
        return x.first != y.first ? x.first > y.first : x.second.first < y.second.first;
    });
    int overLongest = 0;   // cars that stood longer than kLongestStand
    std::map<std::string, int> overLongestWhy;
    for (const std::pair<Real, std::pair<uint32_t, std::string>>& s : stands)
        if (s.first > kLongestStand) {
            ++overLongest;
            const std::string& w = s.second.second;
            ++overLongestWhy[w.rfind("behind car", 0) == 0 ? std::string("behind a car") : w];
        }
    std::printf("    [lab-v2] cars that stood more than %.0f s at a time: %d\n", kLongestStand, overLongest);
    for (const std::pair<const std::string, int>& w : overLongestWhy) std::printf("      %4d %s\n", w.second, w.first.c_str());
    std::printf("    [lab-v2] %zu cars were seen standing at a once-a-second look; the longest (stood at least this long):\n",
                stands.size());
    for (std::size_t k = 0; k < stands.size() && k < 8; ++k)
        std::printf("      car %u stood %.0f s: %s\n", stands[k].second.first, stands[k].first, stands[k].second.second.c_str());
    std::printf("    [lab-v2] bodies overlapping on one path, every tick: after coming onto it the same way (following "
                "failed) %llu pair-ticks, %llu spells, %zu pairs, worst %.3f m; after coming onto it from different paths "
                "(where paths join) %llu pair-ticks, %llu spells, %zu pairs\n",
                static_cast<unsigned long long>(followPairTicks), static_cast<unsigned long long>(followSpells),
                followPairs.size(), worstFollowOverlap, static_cast<unsigned long long>(joinPairTicks),
                static_cast<unsigned long long>(joinSpells), joinPairs.size());
    // The queues emptying over junction lines, and the longest wait at a line.
    {
        std::vector<driving::QueueBurst> bursts;
        for (const std::pair<const uint32_t, std::vector<driving::LineCrossing>>& line : lineCrossings)
            for (const driving::QueueBurst& b : driving::findQueueBursts(line.second)) bursts.push_back(b);
        const driving::QueueFigures q = driving::queueFigures(bursts);
        // The lab's queues are short: every gap from the second car on, as well as the plan's figure (from the fourth).
        double allGaps = 0;
        std::size_t gapCount = 0;
        for (const driving::QueueBurst& b : bursts)
            for (const double g : b.gaps) {
                allGaps += g;
                ++gapCount;
            }
        std::printf("    [lab-v2] queues emptying over junction lines: %zu queues; %zu gaps from the second car on, %.2f s "
                    "a car on average; %zu from the %zuth car on, %.2f s a car; the first car %.2f s from moving off to "
                    "its line\n",
                    q.bursts, gapCount, gapCount ? allGaps / static_cast<double>(gapCount) : 0.0, q.gaps,
                    driving::kSaturationFromCar, q.secondsPerCar, q.firstCarDelay);
        std::printf("    [lab-v2] the longest wait at a junction's line: %.1f s (car %u, at the end of it: %s)\n",
                    longestLineWait, longestLineWaitCar, longestLineWaitWhy.c_str());
    }
    // AT LIGHTS (issue #261).
    std::printf("    [lab-v2] at lights: %llu cars crossed a line where the lights run: %llu on red (must be 0), %llu turned "
                "on red after stopping, %llu on yellow; the mean wait at lights (standing on the lane arrived by, queueing "
                "included) %.1f s a car, the longest %.1f s\n",
                static_cast<unsigned long long>(litCrossings), static_cast<unsigned long long>(enteredOnRed),
                static_cast<unsigned long long>(turnedOnRed), static_cast<unsigned long long>(enteredOnYellow),
                litCrossings ? litWaitSum / static_cast<double>(litCrossings) : 0.0, litLongestWait);
    std::printf("    [lab-v2] tyres against the drawn road (every 2 s, every car; + floats, - sinks); the steepest body "
                "pitch seen %.3f, roll %.3f:\n",
                steepestPitch, steepestRoll);
    for (int g = 0; g < kGroundCount; ++g)
        std::printf("      %-20s %5d car-samples: highest wheel %+.4f m, lowest %+.4f m; lane graph vs drawn road under the "
                    "middle %.4f m\n",
                    groundWords(static_cast<Ground>(g)), wheels[g].cars, wheels[g].worstFloat, wheels[g].worstSink,
                    wheels[g].worstGraphOff);
    // SPEED BY ROAD CLASS (issue #281), over the whole run.
    std::printf("    [lab-v2] mean speed over the run (every driven car, every tick): %.2f m/s over %.0f car-seconds; by "
                "where the car's front is:\n", everyone.mean(), everyone.carSeconds);
    for (const std::pair<const std::string, SpeedTally>& k : byClass)
        std::printf("      %-32s %6.2f m/s (%4.1f mph) over %7.0f car-seconds\n", k.first.c_str(), k.second.mean(),
                    k.second.mean() / kMetresPerSecondInAMph, k.second.carSeconds);
    const Real freeShare = freeSeconds > 0 ? freeShareSum / freeSeconds : 0;
    std::printf("    [lab-v2] the fastest class (%s, posted %.1f m/s): free flow (held by no car, stop, way's end or its own "
                "vehicle's top speed, and cruising) %.0f of %.0f car-seconds, at %.0f%% of the posted limit on average; what held its "
                "cars (car-seconds, and their mean speed as a share of the limit):\n",
                fastestClass < roads.roadClassNames.size() ? roads.roadClassNames[fastestClass].c_str() : "none",
                fastestLimit, freeSeconds, fastestSeconds, 100.0 * freeShare);
    for (const std::pair<const std::string, std::pair<Real, Real>>& h : fastestHeldBy)
        std::printf("      %6.0f car-seconds at %3.0f%%: %s\n", h.second.first,
                    h.second.first > 0 ? 100.0 * h.second.second / h.second.first : 0.0, h.first.c_str());
    const driving::MeasurementReport report = sim.drivingHost().measurements().report();
    std::printf("    [lab-v2] the measurements table:\n%s\n", report.describe().c_str());
    // BODY AGAINST BODY (issue #283): every pair of drawn bodies every tick, sorted by how their paths relate.
    const driving::TouchRow& touches = sim.drivingHost().measurements().rows().touches;
    printTouches(*d, touches);
    CHECK(touches.spells <= kMostTouchSpells);
    // THE WHOLE TABLE PASSES (issue #284: the test used to check rows one by one and never the table), and every driver's
    // car is in the model: a car the host could not put down again stays drawn where it was, outside the model, where
    // nothing could see it (city_driving.cpp; issue #284).
    CHECK(report.passed());
    CHECK(sim.drivingHost().counts().placed == sim.drivingHost().counts().drivers);
    CHECK(sim.drivingHost().counts().noRoomYet == 0);

    CHECK(noReason == 0);
    CHECK(followPairTicks == 0);   // following holds
    CHECK(worstGap < 0.03);
    CHECK(sim.drivingHost().counts().arrivals > 0);
    // GIVING WAY AND INSIDE THE JUNCTION (issues #262 and #263, ADR-0402 and ADR-0405). No two bodies touch anywhere (the
    // bodies_touching row, checked above: crosses, joins, splits, a lane against a connector, paths with no crossing
    // point), nor overlap on the lane after a join; no loop of waiting cars; nobody stands longer than kLongestStand.
    // That includes the road-data case ADR-0400 lists (junction 75's long connector 3010 passes within touching distance
    // of the ends of lanes where cars wait at junction 90's line): a car on the connector now waits for a body on its
    // path, and a ring through that waiting is broken at an answer.
    CHECK(joinPairTicks <= kMostJoinPairTicks);
    CHECK(loopSamples <= kMostLoopSamples);
    CHECK(overLongest == 0);
    // ROOM BEYOND (issue #264): nobody at rest in a box but by a recorded give-way; the deadlock row of the measurements
    // table passes; and every grant outstanding belongs to a driven car whose front has not yet reached its lane (grants
    // never outlive their use).
    {
        const driving::BoxStopRow& box = sim.drivingHost().measurements().rows().boxStops;
        std::printf("    [lab-v2] at rest in a junction box: %.1f car-seconds; not allowed %.1f\n", box.carSeconds,
                    box.notAllowed);
        CHECK(box.notAllowed <= kMostBoxStopsNotAllowed);
        for (const driving::ReportRow& row : report.rows)
            if (row.key == "deadlock_loops" || row.key == "box_stops") CHECK(row.status == driving::RowStatus::Pass);
        std::size_t between = 0;
        for (const driving::RoomGrant& g : d->roomGrants()) {
            const driving::Car* c = d->car(driving::CarId(g.car));
            // (A front that reached the lane in the last tick's move is let go at the next settle step.)
            if (c && c->driven && (c->odometer < g.laneAt || c->path == g.lane)) {
                ++between;
                continue;
            }
            std::printf("      [grant outside] car %u lane %u at odometer %.2f, the car %s at %.2f:\n%s", g.car,
                        g.lane.index, g.laneAt, c ? (c->driven ? "driven" : "not driven") : "gone",
                        c ? c->odometer : 0.0, c ? d->describeCar(c->id).c_str() : "");
        }
        std::printf("    [lab-v2] room grants outstanding at the end: %zu; cars between their grant and its lane: %zu\n",
                    d->roomGrants().size(), between);
        CHECK(between == d->roomGrants().size());
    }
    CHECK(d->junctionPlanCounts().changedMindTwice <= kMostChangedMindTwice);
    // The fastest roads flow near their limit where nothing ahead holds a car (issue #281).
    CHECK(freeSeconds > 0);
    CHECK(freeShare >= kFastestClassFreeFlowShare);
    // THE LIGHTS RUN (issue #261): every junction the plan gave lights has a programme, and nobody drives onto a red.
    CHECK(d->lightsRun());
    CHECK(d->rightOfWay().counts.junctionsTimed > 0 && d->rightOfWay().counts.junctionsDark == 0);
    CHECK(litCrossings > 0);
    CHECK(enteredOnRed == 0);
    // NOTHING THE CARS DO HAS MOVED unseen (kFiveMinuteStateHash).
    if (steps == static_cast<int>(300.0 / kStep + 0.5)) {
        std::printf("    [lab-v2] the final state hash %016llx (pinned %016llx)\n",
                    static_cast<unsigned long long>(d->stateHash()), static_cast<unsigned long long>(kFiveMinuteStateHash));
        CHECK(d->stateHash() == kFiveMinuteStateHash);
    }
}

TEST_CASE(level_traffic_lab_v2_same_every_run) {
    std::vector<uint64_t> first, firstPlans;
    for (int run = 0; run < 2; ++run) {
        LoadedCity lc("traffic_lab.json", "");
        CHECK(lc.ok);
        if (!lc.ok) return;
        const driving::Driving* d = lc.city.sim().drivingV2();
        CHECK(d != nullptr);
        if (!d) return;
        // EVERY TICK (issue #284: it was once a second, so two runs could part and meet again between looks unseen).
        int differ = 0, plansDiffer = 0, firstDiffer = -1;
        for (int i = 1; i <= 60 * 60; ++i) {
            lc.city.step(lc.world, kStep);
            const uint64_t h = d->stateHash();
            const uint64_t p = d->planHash();   // the drivers' junction plans (#260), kept apart from the state hash
            if (run == 0) {
                first.push_back(h);
                firstPlans.push_back(p);
            } else {
                if (first[static_cast<std::size_t>(i - 1)] != h) {
                    ++differ;
                    if (firstDiffer < 0) firstDiffer = i;
                }
                if (firstPlans[static_cast<std::size_t>(i - 1)] != p) ++plansDiffer;
            }
        }
        if (run == 1) {
            std::printf("    [lab-v2] two runs of a minute: %d of %zu per-tick state hashes differ (the first at tick %d), "
                        "%d plan hashes; last %016llx\n",
                        differ, first.size(), firstDiffer, plansDiffer, static_cast<unsigned long long>(d->stateHash()));
            CHECK(differ == 0);
            CHECK(plansDiffer == 0);
        }
    }
}

// ---- The soak (issue #265) -------------------------------------------------------------------------------------------

// THE SOAK'S WINDOW: what the measurements are cut into (simulated seconds), and how long it runs by default (s).
constexpr Real kSoakWindow = 300.0;
constexpr Real kSoakSeconds = 1800.0;
// Throughput must not decay: arrivals in the last window at least this share of the mean over the windows after the
// first (the first starts from rest). A jam building over the run (gridlock spreading) shows as arrivals falling away.
constexpr double kSoakLeastThroughputShare = 0.75;
// Nothing may grow without bound: each figure of the model's memory in the second half of the run no higher than this
// share of its highest in the first half (the lab's demand is steady, so its working memory is too).
constexpr double kSoakMemoryGrowth = 1.25;

// ONE WINDOW OF THE SOAK.
struct SoakWindow {
    uint64_t arrivals = 0;
    std::map<std::string, SpeedTally> speedByClass;
    Real longestLineWait = 0;     // at a junction's line, as the five-minute test measures it (s)
    Real longestStand = 0;        // any stand, waiting as the wait-for graph means it (s)
    uint64_t loopSamples = 0;     // loops seen at the once-a-second looks
    uint64_t deadlocks = 0, gridlocks = 0;   // loops by class, at those looks
    uint64_t noReasonSamples = 0; // cars waiting for no reason, at those looks
    uint64_t enteredOnRed = 0, litCrossings = 0;
    uint64_t onePathPairTicks = 0;               // the overlaps row (bodies on one path)
    uint64_t touchSpells[driving::kTouchKindCount] = {};   // body against body, new spells by kind
    double boxSeconds[driving::kBoxReasonCount] = {};
    double boxNotAllowed = 0;
    double tickMsSum = 0, tickMsWorst = 0;
    uint64_t ticks = 0;
    // The model's working memory, the highest seen at the once-a-second looks.
    std::size_t grants = 0, plans = 0, announcements = 0, conflicts = 0, trips = 0, keptSpells = 0;
};

TEST_CASE(level_traffic_lab_v2_soak) {
    // A long run: off unless asked for (RT_LAB_SOAK=1), so the level suite stays quick. RT_LAB_SOAK_SECONDS sets the
    // length (default 30 simulated minutes); RT_LAB_SOAK_TWICE=0 skips the second run that checks the state hash.
    if (!std::getenv("RT_LAB_SOAK")) {
        std::printf("    [lab-soak] skipped: set RT_LAB_SOAK=1 to run the 30-minute soak (about 12 minutes of wall clock "
                    "with its second run)\n");
        return;
    }
    const Real seconds = std::getenv("RT_LAB_SOAK_SECONDS") ? std::atof(std::getenv("RT_LAB_SOAK_SECONDS")) : kSoakSeconds;
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    const int perSecond = static_cast<int>(1.0 / kStep + 0.5);
    const int perWindow = static_cast<int>(kSoakWindow / kStep + 0.5);
    const int perMinute = perSecond * 60;
    std::vector<uint64_t> hashes;   // the state hash every simulated minute, for the second run
    std::vector<SoakWindow> windows;
    {
        LoadedCity lc("traffic_lab.json", "");
        CHECK(lc.ok);
        if (!lc.ok) return;
        citysim::CityRenderSystem& city = lc.city;
        const citysim::CitySim& sim = city.sim();
        const driving::Driving* d = sim.drivingV2();
        CHECK(d != nullptr);
        if (!d) return;
        const engine::LaneGraph& roads = *d->roads();
        const driving::Measurements& m = sim.drivingHost().measurements();
        std::map<uint32_t, uint32_t> frontPath;
        std::map<uint32_t, Real> standSince, lineWaitSince;
        SoakWindow w;
        // What the counters stood at when the window began.
        uint64_t arrivals0 = 0, onePath0 = 0, touch0[driving::kTouchKindCount] = {};
        double box0[driving::kBoxReasonCount] = {}, notAllowed0 = 0, modelMs0 = 0;
        uint64_t boxOffendersSeen = 0;
        constexpr Real kAtTheLine = 2.0;
        for (int i = 1; i <= steps; ++i) {
            city.step(lc.world, kStep);
            const Real now = i * kStep;
            ++w.ticks;
            {
                const double ms = sim.drivingHost().cost().modelMs - modelMs0;
                modelMs0 = sim.drivingHost().cost().modelMs;
                w.tickMsSum += ms;
                w.tickMsWorst = std::max(w.tickMsWorst, ms);
            }
            for (const driving::Car& c : d->cars()) {
                if (!c.driven) continue;
                const engine::Path& p = roads.paths[c.path.index];
                const bool lane = p.kind == engine::PathKind::Lane && p.section.valid();
                const NameIndex cls = lane ? roads.sections[p.section.index].roadClass : kNoName;
                w.speedByClass[lane && cls < roads.roadClassNames.size() ? roads.roadClassNames[cls]
                                                                         : std::string("connectors")]
                    .add(c.speed, kStep);
                // A front leaving a lane for a connector where the lights run: what its lamp showed as the tick began.
                std::map<uint32_t, uint32_t>::iterator fp = frontPath.find(c.id.index);
                if (fp != frontPath.end() && fp->second != c.path.index && p.kind == engine::PathKind::Connector &&
                    p.from.index == fp->second) {
                    const driving::Lamp lamp = driving::lampAt(roads, d->rightOfWay(), c.path, d->clockSeconds() - kStep);
                    if (lamp != driving::Lamp::Dark) {
                        ++w.litCrossings;
                        if (lamp == driving::Lamp::Red) {
                            ++w.enteredOnRed;
                            // Its card, for a person chasing why (as the five-minute run prints its red runners).
                            std::printf("      [red-runner] t=%.2f car %u onto connector %u at %.1f m/s:\n%s", now,
                                        c.id.index, c.path.index, c.speed, d->describeCar(c.id).c_str());
                        }
                    }
                }
                frontPath[c.id.index] = c.path.index;
                const bool atLine = p.kind == engine::PathKind::Lane && p.next.count > 0 &&
                                    roads.paths[roads.nextPaths[p.next.first].index].kind == engine::PathKind::Connector &&
                                    p.length - c.along <= kAtTheLine && driving::isWaiting(c);
                std::map<uint32_t, Real>::iterator lw = lineWaitSince.find(c.id.index);
                if (atLine && lw == lineWaitSince.end()) lineWaitSince[c.id.index] = now;
                if (atLine && lw != lineWaitSince.end()) w.longestLineWait = std::max(w.longestLineWait, now - lw->second);
                if (!atLine && lw != lineWaitSince.end()) lineWaitSince.erase(lw);
            }
            if (i % perMinute == 0) hashes.push_back(d->stateHash());
            // A new spell of a car at rest in a box for a reason not allowed: its card, for a person chasing why.
            {
                const driving::Offenders& bad = m.rows().boxStops.offenders;
                if (bad.total > boxOffendersSeen) {
                    boxOffendersSeen = bad.total;
                    if (!bad.kept.empty()) {
                        const driving::CarId car = bad.kept.back().car;
                        std::printf("      [box offender] t=%.2f car %u:\n%s", now, car.index, d->describeCar(car).c_str());
                    }
                }
            }
            if (i % perSecond == 0) {
                const driving::WaitReport wr = d->waitReport();
                w.noReasonSamples += wr.noReason.size();
                w.loopSamples += wr.loops.size();
                w.deadlocks += wr.deadlocks;
                w.gridlocks += wr.gridlocks;
                for (const driving::Car& c : d->cars()) {
                    const bool standing = driving::isWaiting(c);
                    std::map<uint32_t, Real>::iterator it = standSince.find(c.id.index);
                    if (standing && it == standSince.end()) it = standSince.insert({c.id.index, now}).first;
                    if (standing) w.longestStand = std::max(w.longestStand, now - it->second);
                    if (!standing && it != standSince.end()) standSince.erase(it);
                }
                std::size_t plans = 0;
                for (const driving::Car& c : d->cars())
                    if (const driving::JunctionPlans* jp = d->junctionPlans(c.id)) plans += jp->count;
                w.grants = std::max(w.grants, d->roomGrants().size());
                w.plans = std::max(w.plans, plans);
                w.announcements = std::max(w.announcements, d->board().size());
                w.conflicts = std::max(w.conflicts, d->resolver().conflicts().size());
                w.trips = std::max(w.trips, d->tripCount());
                w.keptSpells = std::max(w.keptSpells, m.rows().touches.kept.size());
            }
            if (i % perWindow != 0 && i != steps) continue;
            // The window ends: the counters' growth over it.
            const citysim::DrivingCounts& counts = sim.drivingHost().counts();
            const driving::MeasureRows& rows = m.rows();
            w.arrivals = counts.arrivals - arrivals0;
            arrivals0 = counts.arrivals;
            w.onePathPairTicks = rows.overlaps.pairTicks - onePath0;
            onePath0 = rows.overlaps.pairTicks;
            for (uint8_t k = 0; k < driving::kTouchKindCount; ++k) {
                w.touchSpells[k] = rows.touches.spellsByKind[k] - touch0[k];
                touch0[k] = rows.touches.spellsByKind[k];
            }
            for (uint8_t k = 0; k < driving::kBoxReasonCount; ++k) {
                w.boxSeconds[k] = rows.boxStops.byReason[k] - box0[k];
                box0[k] = rows.boxStops.byReason[k];
            }
            w.boxNotAllowed = rows.boxStops.notAllowed - notAllowed0;
            notAllowed0 = rows.boxStops.notAllowed;
            windows.push_back(w);
            std::printf("    [lab-soak] window %zu ends t=%.0f s: state hash %016llx\n", windows.size(), now,
                        static_cast<unsigned long long>(d->stateHash()));
            w = SoakWindow();
        }
        // THE TABLE, one line per window.
        std::printf("    [lab-soak] %.0f simulated seconds in %zu windows of %.0f s:\n", seconds, windows.size(), kSoakWindow);
        std::printf("      %-6s %8s %6s %6s %6s %6s %6s %7s %7s %5s %5s %5s %4s %6s %8s %8s %6s %6s %6s %6s %5s\n", "window",
                    "arrived", "free", "art", "coll", "local", "conn", "lineW", "stand", "loops", "noRsn", "red",
                    "1path", "touch", "boxGW", "boxNot", "msMean", "msMax", "grants", "plans", "board");
        for (std::size_t k = 0; k < windows.size(); ++k) {
            const SoakWindow& x = windows[k];
            const auto speed = [&](const char* cls) {
                std::map<std::string, SpeedTally>::const_iterator it = x.speedByClass.find(cls);
                return it == x.speedByClass.end() ? 0.0 : it->second.mean();
            };
            uint64_t touchesAll = 0;
            for (uint8_t t = 0; t < driving::kTouchKindCount; ++t) touchesAll += x.touchSpells[t];
            std::printf("      %-6zu %8llu %6.2f %6.2f %6.2f %6.2f %6.2f %7.1f %7.1f %5llu %5llu %5llu %4llu %6llu %8.1f "
                        "%8.1f %6.3f %6.2f %6zu %6zu %5zu\n",
                        k + 1, static_cast<unsigned long long>(x.arrivals), speed("freeway"), speed("arterial"),
                        speed("collector"), speed("local"), speed("connectors"), x.longestLineWait, x.longestStand,
                        static_cast<unsigned long long>(x.loopSamples), static_cast<unsigned long long>(x.noReasonSamples),
                        static_cast<unsigned long long>(x.enteredOnRed), static_cast<unsigned long long>(x.onePathPairTicks),
                        static_cast<unsigned long long>(touchesAll),
                        x.boxSeconds[static_cast<int>(driving::BoxReason::GivingWay)], x.boxNotAllowed,
                        x.ticks ? x.tickMsSum / static_cast<double>(x.ticks) : 0.0, x.tickMsWorst, x.grants, x.plans,
                        x.announcements);
        }
        std::printf("      (arrived: trips ended; speeds m/s by road class: freeway, arterial, collector, local, and on "
                    "connectors; lineW: the longest wait at a junction's line, stand: the longest stand anywhere (s); "
                    "loops: waiting loops at the once-a-second looks; noRsn: cars waiting for no reason at those looks; "
                    "red: fronts onto a connector on a red; 1path: pair-ticks of bodies overlapping on one path; touch: "
                    "new spells of bodies touching, any kind; boxGW / boxNot: car-seconds at rest in a box giving way / "
                    "for a reason not allowed; ms: the model's tick on the wall clock; grants, plans, board: room "
                    "grants, junction plans and announcements, the most at once)\n");
        for (std::size_t k = 0; k < windows.size(); ++k) {
            const SoakWindow& x = windows[k];
            std::printf("      window %zu: touches by kind:", k + 1);
            for (uint8_t t = 0; t < driving::kTouchKindCount; ++t)
                if (x.touchSpells[t] > 0)
                    std::printf(" %llu %s;", static_cast<unsigned long long>(x.touchSpells[t]),
                                driving::touchKindKey(static_cast<driving::TouchKind>(t)));
            std::printf(" loops: %llu deadlock, %llu gridlock; at rest in a box by reason:",
                        static_cast<unsigned long long>(x.deadlocks), static_cast<unsigned long long>(x.gridlocks));
            for (uint8_t r = 0; r < driving::kBoxReasonCount; ++r)
                if (x.boxSeconds[r] > 0)
                    std::printf(" %s %.1f;", driving::boxReasonKey(static_cast<driving::BoxReason>(r)), x.boxSeconds[r]);
            std::printf(" memory: conflicts %zu, trips %zu, kept touch spells %zu\n", x.conflicts, x.trips, x.keptSpells);
        }
        printTouches(*d, m.rows().touches);
        std::printf("    [lab-soak] the measurements table:\n%s\n", m.report().describe().c_str());

        // WHAT THE SOAK HOLDS. Every window: nobody waits for no reason, no deadlock (a loop with a right of way in it),
        // nobody at rest in a box but by a recorded give-way, following holds on one path; over the run, no body touches
        // another, entries on red, the longest stand (kSoakStandCycles of the longest cycle) and the drivers that changed
        // their minds twice as limited (kSoakMostEnteredOnRed, kSoakMostChangedMindTwice); the measurements table passes
        // (its endless_waits and no_progress rows among it).
        const double soakLongestStand = kSoakStandCycles * d->rightOfWay().counts.longestCycle;
        const driving::JunctionPlanCounts& pc = d->junctionPlanCounts();
        std::printf("    [lab-soak] the longest stand allowed: %.0f x the longest cycle %.1f s = %.1f s; drivers that "
                    "changed their minds twice %llu; another way: %llu asked, %llu took one\n",
                    kSoakStandCycles, d->rightOfWay().counts.longestCycle, soakLongestStand,
                    static_cast<unsigned long long>(pc.changedMindTwice),
                    static_cast<unsigned long long>(pc.askedAnotherWay), static_cast<unsigned long long>(pc.tookAnotherWay));
        uint64_t touchesAll = 0, enteredOnRed = 0;
        for (const SoakWindow& x : windows) {
            CHECK(x.noReasonSamples == 0);
            CHECK(x.deadlocks == 0);
            CHECK(x.boxNotAllowed <= kMostBoxStopsNotAllowed);
            CHECK(x.onePathPairTicks == 0);
            CHECK(x.longestStand <= soakLongestStand);
            enteredOnRed += x.enteredOnRed;
            for (uint8_t t = 0; t < driving::kTouchKindCount; ++t) touchesAll += x.touchSpells[t];
        }
        CHECK(touchesAll <= kMostTouchSpellsSoak);
        CHECK(enteredOnRed <= kSoakMostEnteredOnRed);
        CHECK(pc.changedMindTwice <= kSoakMostChangedMindTwice);
        CHECK(m.report().passed());
        CHECK(sim.drivingHost().counts().placed == sim.drivingHost().counts().drivers);
        CHECK(sim.drivingHost().counts().noRoomYet == 0);
        // Throughput does not decay, and memory does not grow.
        if (windows.size() >= 3) {
            double mean = 0;
            for (std::size_t k = 1; k < windows.size(); ++k) mean += static_cast<double>(windows[k].arrivals);
            mean /= static_cast<double>(windows.size() - 1);
            std::printf("    [lab-soak] arrivals: the last window %llu, the mean after the first %.1f\n",
                        static_cast<unsigned long long>(windows.back().arrivals), mean);
            CHECK(static_cast<double>(windows.back().arrivals) >= kSoakLeastThroughputShare * mean);
            const std::size_t half = windows.size() / 2;
            const auto grows = [&](std::size_t SoakWindow::*field, const char* name) {
                std::size_t early = 0, late = 0;
                for (std::size_t k = 0; k < windows.size(); ++k)
                    (k < half ? early : late) = std::max(k < half ? early : late, windows[k].*field);
                std::printf("    [lab-soak] %s: the most at once %zu in the first half, %zu in the second\n", name, early, late);
                return static_cast<double>(late) > kSoakMemoryGrowth * static_cast<double>(std::max<std::size_t>(early, 1));
            };
            CHECK(!grows(&SoakWindow::grants, "room grants"));
            CHECK(!grows(&SoakWindow::plans, "junction plans"));
            CHECK(!grows(&SoakWindow::announcements, "announcements"));
            // (Conflicts are printed, not held: how many pairs of cars meet in one tick is the traffic's load at that
            // moment, kept in memory reused from tick to tick, and rises with the queues near junctions; 93 at most in
            // the first half of the first soak, 124 in the second.)
            grows(&SoakWindow::conflicts, "conflicts (printed only)");
            CHECK(!grows(&SoakWindow::trips, "trips"));
        }
    }
    // SAME EVERY RUN over the long run: the lab again, its state hash every simulated minute equal to the first run's.
    if (std::getenv("RT_LAB_SOAK_TWICE") && std::string(std::getenv("RT_LAB_SOAK_TWICE")) == "0") return;
    LoadedCity again("traffic_lab.json", "");
    CHECK(again.ok);
    if (!again.ok) return;
    const driving::Driving* d2 = again.city.sim().drivingV2();
    CHECK(d2 != nullptr);
    if (!d2) return;
    std::size_t differ = 0, compared = 0;
    for (int i = 1; i <= steps; ++i) {
        again.city.step(again.world, kStep);
        if (i % perMinute != 0) continue;
        const std::size_t k = static_cast<std::size_t>(i / perMinute - 1);
        if (k < hashes.size()) {
            ++compared;
            differ += hashes[k] != d2->stateHash();
        }
    }
    std::printf("    [lab-soak] the second run: %zu of %zu per-minute state hashes differ; last %016llx\n", differ, compared,
                static_cast<unsigned long long>(d2->stateHash()));
    CHECK(compared == hashes.size());
    CHECK(differ == 0);
}

// A RULE SET THAT IS REFUSED (here: one that is not there; RT_LAB_RULES names another to try) leaves the lab loading
// with the new model switched off and the old one driving every car.
TEST_CASE(level_traffic_lab_v2_refused_rules_leave_the_old_model_driving) {
    const char* other = std::getenv("RT_LAB_RULES");
    LoadedCity lc("traffic_lab.json", "", other ? other : "rules/no_such_rule_set.lua");
    CHECK(lc.ok);
    if (!lc.ok) return;
    const citysim::CitySim& sim = lc.city.sim();
    CHECK(sim.drivingV2() == nullptr);
    CHECK(!sim.drivingHost().active());
    int moving = 0;
    for (int i = 0; i < 60 * 20; ++i) lc.city.step(lc.world, kStep);
    for (const citysim::Agent& a : sim.agents()) moving += a.mode == citysim::Agent::Mode::Driver && a.moving && a.speed > 1;
    std::printf("    [lab-v2] rule set refused: the new model is %s; %d of %zu drivers under way after 20 s (the old model)\n",
                sim.drivingV2() ? "on" : "off", moving, sim.agents().size());
    CHECK(moving > 100);
}

// THE CITY BUILT AGAIN (issue #284): CitySim::build starts the new model's host over. Before, a rebuild kept the old
// seats: the host still said it drove the last build's agents, so handing the new drivers over was refused ("the new
// model already drives this level's drivers") and the level fell back to the old model -- or, built directly, the old
// model skipped agents whose indices happened to match old seats.
TEST_CASE(level_traffic_lab_v2_rebuild_hands_over_again) {
    LoadedCity lc("traffic_lab.json", "");
    CHECK(lc.ok);
    if (!lc.ok) return;
    for (int i = 0; i < 60 * 5; ++i) lc.city.step(lc.world, kStep);
    const uint32_t drivers = lc.city.sim().drivingHost().counts().drivers;
    CHECK(drivers > 0);
    CHECK(lc.city.build(lc.world, lc.assets.get(), nullptr));
    const citysim::CitySim& sim = lc.city.sim();
    CHECK(sim.drivingV2() != nullptr);
    CHECK(sim.drivingHost().active());
    if (!sim.drivingV2() || !sim.drivingHost().active()) return;
    const citysim::DrivingCounts& n = sim.drivingHost().counts();
    std::printf("    [lab-v2] built again: %u drivers handed over (%u the first time), %u placed, %llu trips begun, %zu cars "
                "in the model\n",
                n.drivers, drivers, n.placed, static_cast<unsigned long long>(n.tripsBegun), sim.drivingV2()->carCount());
    CHECK(n.drivers == drivers);
    CHECK(n.placed == n.drivers);
    CHECK(n.tripsBegun == n.placed);   // counted afresh, not carried over
    CHECK(sim.drivingV2()->carCount() == n.placed);
    // Every agent the host says it drives is a driver of this build, and the old model drives the rest.
    uint32_t handed = 0;
    for (std::size_t i = 0; i < sim.agents().size(); ++i) {
        if (!sim.drivenByV2(static_cast<int>(i))) continue;
        ++handed;
        CHECK(sim.agents()[i].mode == citysim::Agent::Mode::Driver && sim.agents()[i].vehicle >= 0);
    }
    CHECK(handed == n.drivers);
    for (int i = 0; i < 60 * 20; ++i) lc.city.step(lc.world, kStep);
    CHECK(sim.drivingHost().measurements().report().passed());
}

// THE OLD MODEL'S STATE, PINNED (issue #284: before, this printed the hashes for a person to compare by hand). Taken on
// 7021e1b8 with the #284 work, none of which reaches the old model (below). The old model's code is the same as before the rebuild began: `git diff 3e86063c..HEAD --
// src/apps/citysim/` shows only additions -- the new model's host (city_driving.*), CitySim's hand-over to it
// (startDrivingV2, setDrivingRoads, the driving v2 tick), and the seam's exclusions, each `if (drivenByV2(i)) continue`
// or a grid placement skipped for such an agent, which on a level the new model does not drive is never true; CitySim::
// build starting the new model over (#284) does nothing on such a level either. A change to the old model's code that
// changes these numbers changes what v1 levels do: it must say so and update them in the same change.
// traffic_lab as v1: t = 30, 60, 90, 120 s
constexpr uint64_t kV1LabHashes[] = {0xc986a7df6eb1a143ull, 0xf9a798b9a88a80afull, 0x9c837cdab19c3d3cull,
                                     0x12e916f1e4bc0225ull};
// river_town: t = 15, 30, 45, 60 s
constexpr uint64_t kV1RiverHashes[] = {0xf131e7b3521590c2ull, 0x97f03bee42dfacf0ull, 0xf091e2f8d51d0c6eull,
                                       0x1cea9a39a79f85f4ull};

TEST_CASE(level_v1_state_is_unchanged) {
    // The lab as v1 with the fleet it had when the hashes were taken: its own fleet now deals a city bus into its traffic
    // (#284 item 9, ADR-0412: citysim.fleet "fleet_lab", ambientBus), a change of the level's data for the new model's
    // tests, not of the old model's code, which this pin guards.
    const std::vector<uint64_t> lab = runAndHash("traffic_lab.json", "v1", 120.0, 1.0 / 60.0, 30.0, [](CitySimConfig& c) {
        c.fleet.clear();
        c.ambientBus = false;
    });
    const std::vector<uint64_t> river = runAndHash("river_town.json", "", 60.0, 1.0 / 30.0, 15.0);
    CHECK(lab == std::vector<uint64_t>(std::begin(kV1LabHashes), std::end(kV1LabHashes)));
    CHECK(river == std::vector<uint64_t>(std::begin(kV1RiverHashes), std::end(kV1RiverHashes)));
}
