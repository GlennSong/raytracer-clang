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
// path: fails; where different paths cross or join inside a junction: counted, expected until the junction rules
// exist). Every two seconds it measures each car's tyres against the drawn road (CityRenderSystem::wheelGaps), sorted
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
};

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
    for (int i = 1; i <= steps; ++i) {
        city.step(lc.world, kStep);
        const Real now = i * kStep;
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
            bodies.push_back(f);
        }
        for (std::size_t a = 0; a < bodies.size(); ++a)
            for (std::size_t b = a + 1; b < bodies.size(); ++b) {
                const Vec2 ma = (bodies[a].rear + bodies[a].front) * 0.5, mb = (bodies[b].rear + bodies[b].front) * 0.5;
                if ((ma - mb).length() > 25.0) continue;
                if (!overlap(bodies[a], bodies[b])) continue;
                const std::pair<uint32_t, uint32_t> key(bodies[a].car.index, bodies[b].car.index);
                if (sharePath(d->pathLists(), roads, bodies[a].car, bodies[b].car, d->cars()[a])) {
                    ++sameOverlapSamples;   // (counted per tick above, from the lists)
                    sameOverlapPairs.insert(key);
                } else {
                    ++crossOverlapSamples;
                    crossOverlapPairs.insert(key);
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
    std::printf("    [lab-v2] tyres against the drawn road (every 2 s, every car; + floats, - sinks); the steepest body "
                "pitch seen %.3f, roll %.3f:\n",
                steepestPitch, steepestRoll);
    for (int g = 0; g < kGroundCount; ++g)
        std::printf("      %-20s %5d car-samples: highest wheel %+.4f m, lowest %+.4f m; lane graph vs drawn road under the "
                    "middle %.4f m\n",
                    groundWords(static_cast<Ground>(g)), wheels[g].cars, wheels[g].worstFloat, wheels[g].worstSink,
                    wheels[g].worstGraphOff);
    const driving::MeasurementReport report = sim.drivingHost().measurements().report();
    std::printf("    [lab-v2] the measurements table:\n%s\n", report.describe().c_str());

    CHECK(noReason == 0);
    CHECK(followPairTicks == 0);   // following holds; overlaps where paths join wait for the junction rules
    CHECK(worstGap < 0.03);
    CHECK(sim.drivingHost().counts().arrivals > 0);
}

TEST_CASE(level_traffic_lab_v2_same_every_run) {
    std::vector<uint64_t> first;
    for (int run = 0; run < 2; ++run) {
        LoadedCity lc("traffic_lab.json", "");
        CHECK(lc.ok);
        if (!lc.ok) return;
        const driving::Driving* d = lc.city.sim().drivingV2();
        CHECK(d != nullptr);
        if (!d) return;
        int differ = 0;
        for (int i = 1; i <= 60 * 60; ++i) {
            lc.city.step(lc.world, kStep);
            if (i % 60 != 0) continue;
            const uint64_t h = d->stateHash();
            if (run == 0) first.push_back(h);
            else if (first[static_cast<std::size_t>(i / 60 - 1)] != h) ++differ;
        }
        if (run == 1) {
            std::printf("    [lab-v2] two runs of a minute: %d of %zu per-second state hashes differ; last %016llx\n", differ,
                        first.size(), static_cast<unsigned long long>(d->stateHash()));
            CHECK(differ == 0);
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
