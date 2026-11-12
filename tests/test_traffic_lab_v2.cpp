// THE TRAFFIC LAB DRIVEN BY THE NEW MODEL (issues #255, #258, #235), and the proof that the old model is untouched.
//
// Loads real levels through the real LevelLoader and CityRenderSystem (a NullRenderer underneath, as every level test),
// steps the city the way the viewer does, and prints what it measured. Run one case at a time:
//
//   RT_TEST_FILTER=level_traffic_lab_v2_drives ./build-viewer/level_tests
//
// level_traffic_lab_v2_drives: the lab as it ships (citysim.driving "v2", rules/lab.lua) for five simulated minutes at
// the viewer's 60 steps a second. Every second it asks the wait-for graph who is waiting and why (a car waiting for no
// reason fails the test), tracks how long each car has stood, and looks for two cars' bodies overlapping (on the same
// path, or where different paths cross or join, sorted by the kind of meeting of their paths: fails beyond what giving
// way left, issue #262). It measures queues emptying over junction lines and the longest wait at a line, and counts
// what drivers decided at junctions. Every two seconds it measures each car's tyres against the drawn road (CityRenderSystem::wheelGaps), sorted
// by where the car is: on a slope, a banked curve, over a crest or in a dip, or on the flat. At the end it prints the
// measurements table (driving/measure.h), `driving?`, `waiting?` and the cars that stood longest, with why.
//
// level_traffic_lab_v2_same_every_run: the lab loaded twice and run a minute each: the driving model's state hash is the
// same at every second.
//
// level_traffic_lab_v2_refused_rules_leave_the_old_model_driving: a rule set that is refused (missing, or one with
// faults: RT_LAB_RULES=<file>) leaves the lab loading, the new model off and the old one driving.
//
// level_v1_state_is_unchanged: two levels driven by the OLD model alone -- the traffic lab with its citysim block set
// back to "v1", and river_town (walkers, buses, cabs, the far tiers) -- each stepped for a while, with one number
// folded over every agent's driving and walking state and every vehicle's pose, printed as it goes. The numbers are
// compared by hand between a build before a change to the old model's code and after it: equal numbers are the proof
// that a v1 level behaves bit for bit as before.

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
    // level says), builds the city.
    LoadedCity(const std::string& level, const std::string& driving, const std::string& rules = std::string()) {
        uploader = std::make_unique<RendererMeshUploader>(*renderer);
        assets = std::make_unique<AssetManager>(*uploader);
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/" + level, world, *renderer, view, *assets,
                               false))
            return;
        world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
            if (!driving.empty()) c.driving = driving;
            if (!rules.empty()) c.drivingRules = rules;
        });
        ok = city.build(world, assets.get(), nullptr);
    }
};

// Steps `city` `seconds` in steps of `dt`, printing the old model's state hash every `printEvery` seconds.
void runAndHash(const std::string& level, const std::string& driving, Real seconds, Real dt, Real printEvery) {
    LoadedCity lc(level, driving);
    CHECK(lc.ok);
    if (!lc.ok) return;
    const int steps = static_cast<int>(seconds / dt + 0.5);
    const int every = static_cast<int>(printEvery / dt + 0.5);
    std::printf("    [v1-hash] %s (driving %s): %zu agents\n", level.c_str(), driving.empty() ? "as the level says" : driving.c_str(),
                lc.city.sim().agents().size());
    for (int i = 1; i <= steps; ++i) {
        lc.city.step(lc.world, dt);
        if (i % every == 0)
            std::printf("    [v1-hash] %s t=%6.1f s: %016llx\n", level.c_str(), i * dt,
                        static_cast<unsigned long long>(simStateHash(lc.city.sim())));
    }
}

// ---- The new model's lab ---------------------------------------------------------------------------------------------

constexpr Real kStep = 1.0 / 60.0;   // the viewer's fixed step, and the sim's tick on the lab (localHz 0)

// A CAR'S BODY on the plan: a rectangle from its rear bumper to its front, its width across.
struct Footprint {
    driving::CarId car;
    Vec2 rear, front;
    Real halfWidth = 0;
    Real height = 0;   // the road's height under the middle of the body
};
// Two bodies whose roads are further apart in height than this are on different decks (a street under a bridge), not
// overlapping: the road builder's own same-level figure (Rules::sameLevelDz), which the crossing points use too.
constexpr Real kSameLevelWithin = 1.0;

// Do two bodies overlap on the plan (separating axes over the two rectangles)?
bool overlap(const Footprint& a, const Footprint& b) {
    const auto corners = [](const Footprint& f, Vec2 out[4]) {
        Vec2 d = f.front - f.rear;
        const Real len = d.length();
        d = len > 1e-9 ? d * (1.0 / len) : Vec2(1, 0);
        const Vec2 side(-d.y * f.halfWidth, d.x * f.halfWidth);
        out[0] = f.rear + side;
        out[1] = f.front + side;
        out[2] = f.front - side;
        out[3] = f.rear - side;
    };
    Vec2 ca[4], cb[4];
    corners(a, ca);
    corners(b, cb);
    for (const Vec2* poly : {ca, cb})
        for (int e = 0; e < 4; ++e) {
            const Vec2 edge = poly[(e + 1) % 4] - poly[e];
            const Vec2 axis(-edge.y, edge.x);
            Real loA = 1e30, hiA = -1e30, loB = 1e30, hiB = -1e30;
            for (int k = 0; k < 4; ++k) {
                const Real pa = ca[k].x * axis.x + ca[k].y * axis.y, pb = cb[k].x * axis.x + cb[k].y * axis.y;
                loA = std::min(loA, pa);
                hiA = std::max(hiA, pa);
                loB = std::min(loB, pb);
                hiB = std::max(hiB, pb);
            }
            if (hiA <= loB || hiB <= loA) return false;
        }
    return true;
}

// Are two cars listed together on any path (bodies on the same lane or connector)?
bool sharePath(const driving::PathLists& lists, const engine::LaneGraph& roads, driving::CarId a, driving::CarId b,
               const driving::Car& ca) {
    std::vector<engine::PathId> paths{ca.path};
    for (std::size_t k = 0; k < ca.trailCount; ++k) paths.push_back(ca.trail[k]);
    (void)roads;
    for (const engine::PathId p : paths) {
        bool hasA = false, hasB = false;
        for (const driving::PathEntry& e : lists.on(p)) {
            hasA = hasA || e.car == a;
            hasB = hasB || e.car == b;
        }
        if (hasA && hasB) return true;
    }
    return false;
}

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

// WHAT THE FIVE-MINUTE RUN MAY SHOW, so a later change cannot make it worse unseen. Pairs of cars seen overlapping on
// the plan (sampled each second) where their paths cross or join inside one junction, and where they meet filed under no
// junction; pair-ticks of overlap on the lane after a join: none, since first in goes first inside the junction (issue
// #263, ADR-0405: before it, a car that gave way and then went was crossed by the car it had given way to, one pair
// overlapped 44 ticks after a join, and junction 75's long connector 3010 drove through a car waiting on lane 766).
// Drivers that changed their minds twice on one approach: at most four (waits inside a junction are counted apart).
constexpr std::size_t kMostCrossJoinPairs = 0;
constexpr uint64_t kMostJoinPairTicks = 0;
constexpr std::size_t kMostAcrossJunctionPairs = 0;
constexpr uint64_t kMostChangedMindTwice = 4;
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

// WHERE TWO CARS' PATHS MEET, for an overlap on the plan between cars on different paths: the kind of the first crossing
// point found between any path one body is on and any path the other's is on (crossings filed under a junction first,
// as each path lists them), or none.
enum class MeetKind { Cross, Join, Split, CrossAcross, JoinAcross, None };
constexpr int kMeetKindCount = 6;
const char* meetWords(MeetKind k) {
    switch (k) {
        case MeetKind::Cross: return "a cross inside one junction";
        case MeetKind::Join: return "a join inside one junction";
        case MeetKind::Split: return "a split (two paths leaving one lane)";
        case MeetKind::CrossAcross: return "a cross filed under no junction (two junctions, or a connector and a lane)";
        case MeetKind::JoinAcross: return "a join filed under no junction";
        case MeetKind::None: return "no crossing point between their paths";
    }
    return "?";
}
MeetKind meetingOf(const engine::LaneGraph& roads, const driving::Car& a, const driving::Car& b, uint32_t* crossing) {
    std::vector<engine::PathId> pa{a.path}, pb{b.path};
    for (std::size_t k = 0; k < a.trailCount; ++k) pa.push_back(a.trail[k]);
    for (std::size_t k = 0; k < b.trailCount; ++k) pb.push_back(b.trail[k]);
    for (const engine::PathId p : pa) {
        const engine::Path& path = roads.paths[p.index];
        for (uint32_t k = path.crossings.first; k < path.crossings.end(); ++k) {
            const engine::CrossingId cid = roads.pathCrossings[k];
            const engine::CrossingPoint& rec = roads.crossings[cid.index];
            if (rec.otherSide != engine::OtherSide::Path) continue;
            const engine::PathId other = rec.a == p ? rec.otherPath() : rec.a;
            if (std::find(pb.begin(), pb.end(), other) == pb.end()) continue;
            if (crossing) *crossing = cid.index;
            const bool across = !rec.junction.valid();
            if (rec.meeting == engine::Meeting::Split) return MeetKind::Split;
            if (rec.meeting == engine::Meeting::Join) return across ? MeetKind::JoinAcross : MeetKind::Join;
            return across ? MeetKind::CrossAcross : MeetKind::Cross;
        }
    }
    return MeetKind::None;
}

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
    // WHO GIVES WAY TO WHOM (issue #260): the table's counts, and one four-way's table in words, for checking by hand.
    std::printf("    [lab-v2] %s", driving::describeRightOfWay(d->rightOfWay()).c_str());
    for (uint32_t j = 0; j < roads.junctions.size(); ++j) {
        const driving::JunctionRules& jr = d->rightOfWay().junctions[j];
        if (jr.approaches.count != 4 || roads.junctions[j].connectors.count > 12) continue;
        std::printf("    [lab-v2] one four-way's table:\n%s", d->describeJunction(engine::JunctionId(j)).c_str());
        break;
    }

    const Real seconds = std::getenv("RT_LAB_SECONDS") ? std::atof(std::getenv("RT_LAB_SECONDS")) : 300.0;
    const int steps = static_cast<int>(seconds / kStep + 0.5);
    const int perSecond = static_cast<int>(1.0 / kStep + 0.5);
    std::map<uint32_t, Real> standSince;                  // car -> when it began to stand (driven cars only)
    std::map<uint32_t, std::pair<Real, std::string>> longest;   // car -> its longest stand, and why (at the end of it)
    uint64_t noReason = 0, loopSamples = 0, sameOverlapSamples = 0, crossOverlapSamples = 0, waitSamples = 0;
    std::set<std::pair<uint32_t, uint32_t>> sameOverlapPairs, crossOverlapPairs;
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
    // Overlaps on the plan between cars on different paths, by the kind of meeting of their paths (sampled each second).
    uint64_t meetSamples[kMeetKindCount] = {};
    std::set<std::pair<uint32_t, uint32_t>> meetPairs[kMeetKindCount];
    // QUEUES EMPTYING AT JUNCTION LINES: a car's front leaving an arriving lane for a connector crosses that lane's line;
    // the queue figures come from those crossings (measure.h findQueueBursts), as the measurements table reads them.
    struct RestSeen {
        bool atRest = true;
        double restLeftAt = -1, restOdometer = 0;
    };
    std::map<uint32_t, RestSeen> rest;
    std::map<uint32_t, std::vector<driving::LineCrossing>> lineCrossings;   // by arriving lane
    std::map<uint32_t, uint32_t> frontPath;
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
                std::printf("---- t=%.3f\n%s", now, d->describeCar(driving::CarId(wc)).c_str());
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
        // HOW LONG EACH CAR HAS STOOD (waiting, as the wait-for graph means it: at rest and not moving off), and what
        // held it at the end of the stand.
        for (const driving::Car& c : d->cars()) {
            const bool standing = driving::isWaiting(c);
            std::map<uint32_t, Real>::iterator it = standSince.find(c.id.index);
            if (standing && it == standSince.end()) it = standSince.insert({c.id.index, now}).first;
            if (standing) {
                const Real stood = now - it->second;
                std::map<uint32_t, std::pair<Real, std::string>>::iterator l = longest.find(c.id.index);
                if (l == longest.end() || stood >= l->second.first) longest[c.id.index] = {stood, why(c)};
            }
            if (!standing && it != standSince.end()) standSince.erase(it);
        }
        // BODIES OVERLAPPING: on a shared path (following must hold), or where different paths cross or join.
        std::vector<Footprint> bodies;
        for (const driving::Car& c : d->cars()) {
            const driving::CarPose p = driving::carPose(roads, d->vehicleClasses(), d->ruleSet(), c);
            Footprint f;
            f.car = c.id;
            f.rear = Vec2(p.rear.x, p.rear.z);
            f.front = Vec2(p.front.x, p.front.z);
            f.halfWidth = d->vehicleClasses().at(c.vehicleClass).width * 0.5;
            f.height = (p.rear.y + p.front.y) * 0.5;
            bodies.push_back(f);
        }
        for (std::size_t a = 0; a < bodies.size(); ++a)
            for (std::size_t b = a + 1; b < bodies.size(); ++b) {
                const Vec2 ma = (bodies[a].rear + bodies[a].front) * 0.5, mb = (bodies[b].rear + bodies[b].front) * 0.5;
                if ((ma - mb).length() > 25.0) continue;
                if (std::fabs(bodies[a].height - bodies[b].height) > kSameLevelWithin) continue;
                if (!overlap(bodies[a], bodies[b])) continue;
                const std::pair<uint32_t, uint32_t> key(bodies[a].car.index, bodies[b].car.index);
                if (sharePath(d->pathLists(), roads, bodies[a].car, bodies[b].car, d->cars()[a])) {
                    ++sameOverlapSamples;   // (counted per tick above, from the lists)
                    sameOverlapPairs.insert(key);
                } else {
                    ++crossOverlapSamples;
                    crossOverlapPairs.insert(key);
                    uint32_t crossing = engine::kNoIndex;
                    const MeetKind kind = meetingOf(roads, d->cars()[a], d->cars()[b], &crossing);
                    ++meetSamples[static_cast<int>(kind)];
                    const bool fresh = meetPairs[static_cast<int>(kind)].insert(key).second;
                    if (fresh && std::getenv("RT_LAB_OVERLAP_LOG")) {
                        std::printf("      [cross-overlap] t=%.0f cars %u and %u, %s (crossing %u)\n", now, key.first,
                                    key.second, meetWords(kind), crossing);
                        std::printf("%s%s", d->describeCar(bodies[a].car).c_str(), d->describeCar(bodies[b].car).c_str());
                    }
                }
            }
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
                    "once; %llu waits inside a junction; the board held at most %zu announcements\n",
                    static_cast<unsigned long long>(pc.decidedGo), static_cast<unsigned long long>(pc.decidedGiveWay),
                    static_cast<unsigned long long>(pc.wentAfterGivingWay), pc.smallestGapTaken, pc.largestGapTaken,
                    pc.smallestGapNeeded, static_cast<unsigned long long>(pc.changedMindTwice),
                    static_cast<unsigned long long>(pc.waitedInside), d->board().most());
        // WHO GOES FIRST (issue #263): the conflicts the one resolver settled over the run, by the question that decided.
        const driving::ConflictCounts& cc = d->conflictCounts();
        std::printf("    [lab-v2] who goes first: %llu conflicts settled (summed over the ticks); by question:",
                    static_cast<unsigned long long>(cc.conflicts));
        for (uint8_t q = 0; q < driving::kQuestionCount; ++q)
            if (cc.byQuestion[q] > 0)
                std::printf(" %s %llu", driving::questionName(static_cast<driving::Question>(q)),
                            static_cast<unsigned long long>(cc.byQuestion[q]));
        std::printf("; rings broken %llu (through a body %llu; of bodies and queues alone %llu)\n",
                    static_cast<unsigned long long>(cc.rings), static_cast<unsigned long long>(cc.ringThroughABody),
                    static_cast<unsigned long long>(cc.physicalRings));
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
    int over60 = 0;
    std::map<std::string, int> over60Why;
    for (const std::pair<Real, std::pair<uint32_t, std::string>>& s : stands)
        if (s.first > 60.0) {
            ++over60;
            const std::string& w = s.second.second;
            ++over60Why[w.rfind("behind car", 0) == 0 ? std::string("behind a car") : w];
        }
    std::printf("    [lab-v2] cars that stood more than 60 s at a time: %d\n", over60);
    for (const std::pair<const std::string, int>& w : over60Why) std::printf("      %4d %s\n", w.second, w.first.c_str());
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
    std::printf("    [lab-v2] bodies overlapping on the plan, sampled each second: sharing a path %llu (%zu pairs); on "
                "different paths, where they cross or join %llu (%zu pairs)\n",
                static_cast<unsigned long long>(sameOverlapSamples), sameOverlapPairs.size(),
                static_cast<unsigned long long>(crossOverlapSamples), crossOverlapPairs.size());
    std::printf("    [lab-v2] ... on different paths, by where their paths meet (samples, pairs):\n");
    for (int k = 0; k < kMeetKindCount; ++k)
        std::printf("      %-75s %4llu  %4zu\n", meetWords(static_cast<MeetKind>(k)),
                    static_cast<unsigned long long>(meetSamples[k]), meetPairs[k].size());
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

    CHECK(noReason == 0);
    CHECK(followPairTicks == 0);   // following holds
    CHECK(worstGap < 0.03);
    CHECK(sim.drivingHost().counts().arrivals > 0);
    // GIVING WAY AND INSIDE THE JUNCTION (issues #262 and #263, ADR-0402 and ADR-0405). No two cars that obeyed the rules
    // overlap where their paths cross or join, inside a junction or across two, nor on the lane after a join; no loop of
    // waiting cars; nobody stands 60 s. That includes the road-data case ADR-0400 lists (junction 75's long connector
    // 3010 passes within touching distance of the ends of lanes where cars wait at junction 90's line): a car on the
    // connector now waits for a body on its path, and a ring through that waiting is broken at an answer.
    CHECK(meetPairs[static_cast<int>(MeetKind::Cross)].size() + meetPairs[static_cast<int>(MeetKind::Join)].size() <=
          kMostCrossJoinPairs);
    CHECK(meetPairs[static_cast<int>(MeetKind::CrossAcross)].size() + meetPairs[static_cast<int>(MeetKind::JoinAcross)].size() <=
          kMostAcrossJunctionPairs);
    CHECK(joinPairTicks <= kMostJoinPairTicks);
    CHECK(loopSamples == 0);
    CHECK(over60 == 0);
    CHECK(d->junctionPlanCounts().changedMindTwice <= kMostChangedMindTwice);
    // The fastest roads flow near their limit where nothing ahead holds a car (issue #281).
    CHECK(freeSeconds > 0);
    CHECK(freeShare >= kFastestClassFreeFlowShare);
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
        int differ = 0, plansDiffer = 0;
        for (int i = 1; i <= 60 * 60; ++i) {
            lc.city.step(lc.world, kStep);
            if (i % 60 != 0) continue;
            const uint64_t h = d->stateHash();
            const uint64_t p = d->planHash();   // the drivers' junction plans (#260), kept apart from the state hash
            if (run == 0) {
                first.push_back(h);
                firstPlans.push_back(p);
            } else {
                if (first[static_cast<std::size_t>(i / 60 - 1)] != h) ++differ;
                if (firstPlans[static_cast<std::size_t>(i / 60 - 1)] != p) ++plansDiffer;
            }
        }
        if (run == 1) {
            std::printf("    [lab-v2] two runs of a minute: %d of %zu per-second state hashes differ, %d plan hashes; last "
                        "%016llx\n",
                        differ, first.size(), plansDiffer, static_cast<unsigned long long>(d->stateHash()));
            CHECK(differ == 0);
            CHECK(plansDiffer == 0);
        }
    }
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

TEST_CASE(level_v1_state_is_unchanged) {
    runAndHash("traffic_lab.json", "v1", 120.0, 1.0 / 60.0, 30.0);
    runAndHash("river_town.json", "", 60.0, 1.0 / 30.0, 15.0);
}
