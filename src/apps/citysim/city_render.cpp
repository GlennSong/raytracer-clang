#include "city_render.h"
#include "../../engine/lights/light_kinds.h"   // lightKindId: traffic's lamps as lights (ADR-0362)

#include "../../engine/procgen/person.h"   // buildStandingPerson (the far crowd)
#include "../../engine/interaction.h"           // Interactables: the outdoor seats (M5)
#include "../../engine/procgen/furniture_library.h"
#include "../../engine/procgen/city/shop_signs.h"
#include "../../engine/text/font.h"
#include "../../engine/systems/day_night_system.h"   // SimHourQuery: `daynight?` asks the sim's hour

#include "city_brain.h"                               // the households' cars (ADR-0444)
#include "city_car.h"                                 // ...which are entities
#include "city_near.h"                                // CityNearCar: a near car the driving sees by itself (ADR-0451)
#include "city_crowd.h"                               // a crowd made from the roads (ADR-0446)
#include "../../engine/procgen/city/polyline.h"       // distToSegment

#include "../../engine/vehicle_lamps.h"               // the lamp decision core (ADR-0065)
#include "screen_project.h"                           // worldToScreen (place labels)
#include "../../engine/asset_manager.h"
#include <cctype>
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/city/roads/road_entity.h"
#include "../../engine/procgen/city/roads/lanes/road_model_cache.h"   // the level's built road model, from its cache
#include "../../engine/procgen/city/roads/street_map.h"   // appendWalkways: the model's walkways on the street map (ADR-0476)
#include "../../engine/procgen/city/road_mesh.h"     // strokeRibbon (closed lot/block outlines)
#include "../../engine/procgen/city/street_kit.h"   // trafficSignalProto, SignalParams
#include "../../engine/procgen/noise.h"             // Noise (carved-terrain sampler)
#include "../../engine/procgen/terrain.h"            // terrainHeight, TerrainParams
#include "../../log.h"                               // LOG_WARN (place-type validation)
#include "../../renderer/event.h"                    // KeyCode (debug-widget toggle)
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>                                   // Living City debug section
#endif
#include "../../engine/procgen/vehicle/occupant.h"   // the driver seated in a see-into vehicle
#include "../../engine/player/driver_seat.h"         // seatedPose: a rider astride a bike
#include "../../engine/interior_occupants.h"         // people inside a streamed interior (campus M4)
#include "../../engine/procgen/furniture_kit.h"
#include "../../engine/furniture_draw.h"
#include "../../engine/board_meshes.h"   // the boards the riders ride   // spawnFurnitureGroups: the beach's towels and umbrellas
#include "../../engine/procgen/city/building_records.h"
#include "../../engine/procgen/city/building_uses.h"   // a home's use picks its dwellings' rule
#ifdef RT_ENABLE_SCRIPTING
#include "scripting/agent_goals.h"      // scripted goal tables (ADR-0064)
#include "scripting/activities_lua.h"   // the activity catalog (activities.lua)
#include "scripting/roles_lua.h"         // the role catalog (roles.lua)
#include "scripting/place_kinds_lua.h"   // the place kinds and dwellings (places.lua)
#include "scripting/people_lua.h"        // ages, life stages, households (people.lua)
#include "scripting/crowd_lua.h"         // a crowd made from the roads (crowd.lua, ADR-0446)
#include "scripting/vehicle_body.h"     // scripted fleet bodies (ADR-0065)
#include "../../engine/scripting/script_vm.h"
#include "../../engine/scripting/procgen_bindings.h"
#include "../../engine/scripting/script_modules.h"
#include "../../engine/script_assets.h"
#include "../../log.h"
#endif

#include <algorithm>
#include <map>
#include <unordered_map>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <numeric>   // std::gcd
#include <string>

namespace citysim {

using engine::Vec2;
using engine::Vec3;
using engine::Mat4;
using engine::Quat;
using engine::World;
using engine::Entity;
using engine::AssetManager;
using engine::InstanceGroup;
using engine::RoadGraph;
using engine::RoadNode;
using engine::RoadEdge;
using engine::RoadEntity;
using engine::MeshHandle;
using engine::MeshBuilder;

namespace {

Vec3 translationOf(const Mat4& t) { return Vec3(t.m[0][3], t.m[1][3], t.m[2][3]); }

// Debug vision-cone dimensions — MUST match the walkers' think-cadence kPedVision*
// constants (city_sim.cpp). The wedge draws what the walker can actually see.
// The player's number as a thing on the road (ADR-0451); a physics car the driving does not drive takes the next ones.
// (People on foot are numbered by their agent uid, far below.)
constexpr uint32_t kPlayerThing = 0xFFF00000u;
constexpr Real kPedConeRange = 4.5, kPedConeHalfAngle = 1.2;
// How far beside a signal head's green lamp its arrow lens is lit (metres, along the arm), clear of the head's side.
constexpr Real kArrowLensOffset = 0.34;
// Where no render view has staged the sun (headless), the cars' headlights follow the sim's own clock: off at and
// after dawn, on after dusk (in-world hours; clockIsDark).
constexpr Real kHeadlightDawn = 6.5;
constexpr Real kHeadlightDusk = 19.0;
// Debug tints, dimmer than the state rings so neither view shouts over them.
const Vec3 kNavTint(0.18, 0.32, 0.55);    // faint blue: the road/lane graph
const Vec3 kConeTint(0.55, 0.36, 0.10);   // dim amber: the sensing wedge

void refreshBounds(InstanceGroup* g) {
    if (!g) return;
    if (g->transforms.empty()) { g->boundsRadius = 0; return; }
    Vec3 c(0, 0, 0);
    for (const Mat4& t : g->transforms) c = c + translationOf(t);
    c = c / static_cast<Real>(g->transforms.size());
    Real r = 0;
    for (const Mat4& t : g->transforms)
        r = std::max(r, (translationOf(t) - c).length());
    g->boundsCenter = c;
    g->boundsRadius = r + 5.0;
}

}  // namespace

bool CityRenderSystem::clockIsDark(Real clockHours) {
    return clockHours < kHeadlightDawn || clockHours > kHeadlightDusk;
}

bool CityRenderSystem::carOccupied(int ai) const {
    if (ai < 0 || ai >= static_cast<int>(sim_.agents().size())) return false;
    const Agent& a = sim_.agents()[static_cast<std::size_t>(ai)];
    // A PERSON IN A HOUSEHOLD'S CAR IS IN IT WHENEVER THEY ARE AT ITS WHEEL (ADR-0442, ADR-0444), moving or not: the
    // brain's pulling-out and parking states are a person sitting in a car in its space, and the brain takes them out
    // of it when they leave. (For everyone else a car at rest stands empty: its person is indoors.)
    if (a.inCar >= 0) return true;
    return a.moving;
}

Vec2 CityRenderSystem::drawnAgentPos(int i, Vec2* heading) const {
    const Agent& a = sim_.agents()[static_cast<std::size_t>(i)];
    if (heading) *heading = a.heading;
    const Real period = sim_.tickPeriod();
    if (period <= 0.0 || a.released) return a.pos;
    const Vec2 d = a.pos - a.tickFromPos;
    if (d.x * d.x + d.y * d.y >= 8.0 * 8.0) return a.pos;   // a placement snaps, never slides
    const Real t = std::clamp(sim_.secondsSinceTick() / period, Real(0), Real(1));
    if (heading) {
        Real dy = std::atan2(a.heading.x, a.heading.y) - std::atan2(a.tickFromHeading.x, a.tickFromHeading.y);
        while (dy > engine::PI) dy -= 2 * engine::PI;
        while (dy < -engine::PI) dy += 2 * engine::PI;
        const Real yw = std::atan2(a.tickFromHeading.x, a.tickFromHeading.y) + dy * t;
        *heading = Vec2(std::sin(yw), std::cos(yw));
    }
    return a.tickFromPos + d * t;
}

Real CityRenderSystem::groundAt(Real x, Real z) const { return sim_.poses().ground.groundAt(x, z); }

// The mirror of CitySim's own per-agent height read (city_sim.cpp, "continuous
// carriageway height"): lerp the link's end elevations along it, absolute for a
// corridor deck/ramp, ground-relative plus the layer lift otherwise. Keeping the
// two readings identical is the whole point — a parked car placed by this and a
// moving car placed by the sim sit on the same surface (#25).
Real CityRenderSystem::deckYAt(int link, Real station, Vec2 p) const {
    if (link < 0 || link >= nav_.linkCount()) return groundAt(p.x, p.y);
    const engine::NavLink& L = nav_.links[link];
    const Real t = L.length > 1e-6
                       ? std::clamp(station / L.length, Real(0), Real(1))
                       : Real(0);
    const Real e = L.elevA + (L.elevB - L.elevA) * t;
    if (L.elevAbsolute) return deckSurfaceNear(p.x, p.y, e);
    return groundAt(p.x, p.y) + L.layer * kLayerClearance + e;
}

Real CityRenderSystem::deckSurfaceNear(Real x, Real z, Real refY) const {
    return sim_.poses().ground.deckSurfaceNear(x, z, refY);
}

bool CityRenderSystem::agentWorldPose(int agentId, Vec3& outPos,
                                      Vec2& outHeading) const {
    if (agentId < 0 || agentId >= static_cast<int>(sim_.agents().size()))
        return false;
    const Agent& a = sim_.agents()[agentId];
    if (a.released) return false;

    Real x = a.pos.x, z = a.pos.y;
    Vec2 heading = a.heading;
    // Prefer the REAL body pose when the walkers are externally owned (same source
    // the debug widgets ring): the planner ghost legitimately runs ahead/behind.
    // A walker with no reported body has no pose to follow — report failure so the caller cycles on.
    if (pedsExternallyOwned_) {
        const std::vector<ExternalAgentPose>& poses = externalPedPoses_;
        bool reported = false;
        for (const ExternalAgentPose& p : poses)
            if (p.agentId == agentId) {
                x = p.pos.x;
                z = p.pos.y;
                heading = p.heading;
                reported = true;
                break;
            }
        if (!reported) return false;
    }
    outPos = Vec3(x, groundAt(x, z) + a.elevation, z);
    outHeading = heading;
    return true;
}

bool CityRenderSystem::build(World& world, AssetManager* assets,
                             std::function<double(double, double)> ground) {
    struct StartupTotal {
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        ~StartupTotal() { LOG_INFO << "[citysim] startup: CityRenderSystem::build total " << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s"; }
    } startupTotal;
    // Level-authored settings (ADR-0063): a CitySimConfig entity — the level's
    // top-level "citysim" block — overrides the constructor params, so each level
    // picks its own population, seed, clock rate, reliability, and whether the
    // agent-state HUD starts on. The agent lab runs 1 car + 1 walker, HUD on.
    world.each<engine::CitySimConfig>([&](Entity, engine::CitySimConfig& c) {
        params_.cars = c.cars;
        params_.pedestrians = c.pedestrians;
        params_.carsPerLaneKm = c.carsPerLaneKm;
        params_.pedsPerKm = c.pedsPerKm;
        params_.maxAmbient = c.maxAmbient;
        params_.nearTarget = c.nearTarget;
        params_.farTarget = c.farTarget;
        // RT_MAX_AMBIENT=<n>: the population cap per class for this run (scale tests on a real level without
        // editing it -- the bake never sees it)
        if (const char* m = std::getenv("RT_MAX_AMBIENT")) params_.maxAmbient = std::max(6, std::atoi(m));
        // RT_DENSITY_SCALE=<k>: the level's cars per lane-km and walkers per km times k (with RT_MAX_AMBIENT: the
        // scale runs past what the streets were sized for)
        if (const char* k = std::getenv("RT_DENSITY_SCALE")) {
            const double f = std::max(0.01, std::atof(k));
            params_.carsPerLaneKm *= f;
            params_.pedsPerKm *= f;
        }
        // RT_CARS_SCALE=<k>: cars per lane-km alone times k (the busy-streets A/B: more cars, the same walkers)
        if (const char* k = std::getenv("RT_CARS_SCALE")) params_.carsPerLaneKm *= std::max(0.01, std::atof(k));
        params_.seed = c.seed;
        params_.hoursPerSecond = c.hoursPerSecond;
        params_.startHour = c.startHour;
        params_.sceneryRadius = c.sceneryRadius;
        params_.localHz = c.localHz;
        params_.adaptiveRate = c.adaptiveRate;
        params_.tieredAgents = c.tieredAgents;
        params_.dormantAgents = c.dormantAgents;
        params_.pedPromoteRadius = c.pedPromoteRadius;
        params_.pedDemoteRadius = c.pedDemoteRadius;
        params_.drivingRules = c.drivingRules;
        params_.levelPath = c.levelPath;
        params_.wander = c.wander;
        params_.schedule = c.schedule;
        params_.markers = c.markers;
        params_.residents = c.residents;
        params_.households = c.households;
        params_.crowd = c.crowd;
        params_.drivableSpacesOnly = c.drivableSpacesOnly;
        params_.ambientBus = c.ambientBus;
        params_.agentScript = c.agentScript;
        params_.vehicleScript = c.vehicleScript;
        params_.fleet = c.fleet;
        debugWidgets_ = c.debugWidgets;
        showPlan_ = showPlan_ || c.showPlan;
        authoredPlaces_ = c.places;   // level-authored destinations (ADR-0066)
        jogLoops_ = c.jogLoops;       // the tracks to run
        activityAreas_ = c.activityAreas;   // the pitches, the lawns
        sitSpots_ = c.sitSpots;             // a fountain's rim
    });

    // The road entities' look: the lift and the sidewalk width the sim draws its people and cars with.
    heightAt_ = std::move(ground);   // caller-supplied sampler (test worlds)
    roadLift_ = 0.0;
    world.each<RoadEntity>([&](Entity, RoadEntity& net) {
        roadLift_ = std::max(roadLift_, static_cast<Real>(net.look.lift));
        sidewalk_ = std::max(sidewalk_, static_cast<Real>(net.look.sidewalk));
    });
    decks_.clear();
    world.each<engine::RoadDeck>(
        [&](Entity, engine::RoadDeck& d) { decks_.push_back(d.field); });
    // THE GROUND THIS BRIDGE STANDS THINGS ON (#25). Everything this bridge
    // places sits on the finished road: parked cars, painted bay outlines,
    // crosswalk decals, at-grade traffic. The level's CDLOD terrain config
    // already folds every road's cut/fill footprint into `params.flatten`, so
    // sampling THAT is sampling the road surface (the natural pre-carve
    // sampler put them on the raw hillside — cars sunk into a cut, hovering
    // over a fill). Test worlds with no terrain keep the caller's sampler.
    world.each<engine::TerrainLodConfig>([&](Entity, engine::TerrainLodConfig& c) {
        auto params = std::make_shared<engine::TerrainParams>(c.params);
        auto noise = std::make_shared<engine::Noise>(c.seed);
        heightAt_ = [params, noise](double x, double z) {
            return engine::terrainHeight(*params, *noise, x, z);
        };
    });
    // THE LEVEL'S STREET MAP (docs/road-model-design.md step 7, ADR-0425): the one map, built at load from the road model
    // (engine::LevelRoadModel::streetMap), every link and node carrying its street's and junction's ids, its signals stated by its
    // junctions' controls -- the same map the furniture was planned on, so a pole's link is this link. The sim copies it
    // and appends its walks (below).
    {
        std::shared_ptr<const engine::NavGraph> map;
        world.each<engine::LevelRoadModel>([&](Entity, engine::LevelRoadModel& m) { if (!map && m.streetMap) map = m.streetMap; });
        if (!map || map->linkCount() == 0) return false;
        nav_ = *map;
    }
    // HEIGHTS RESOLVED ONCE, where the ground is known (review of #35 / the ramp-foot sink): every link end
    // gets its ABSOLUTE carriageway Y (elevA/B on an absolute link -- what draws the car) and its height
    // ABOVE THE GROUND (aboveA/B -- what the sim's same-level tests compare). A deck-to-street link (one
    // absolute end, one relative) was absolute as a whole: cars lerped toward y = 0 and sank, and on it the
    // sim compared an absolute Y against street cars' heights above ground, so a car at a ramp foot saw
    // street traffic tens of metres away. Now both forms are exact at both ends of every link.
    {
        int mixed = 0;
        for (engine::NavLink& L : nav_.links) {
            if (L.from < 0 || L.to < 0) continue;
            const engine::Vec2 pa = nav_.nodes[static_cast<std::size_t>(L.from)], pb = nav_.nodes[static_cast<std::size_t>(L.to)];
            const Real ga = groundAt(pa.x, pa.y), gb = groundAt(pb.x, pb.y);
            const Real lift = L.layer * kLayerClearance;
            // above the ground: a relative end's own height (+ the layer lift), an absolute end's over its ground
            L.aboveA = L.elevAbsA ? L.elevA - ga : lift + L.elevA;
            L.aboveB = L.elevAbsB ? L.elevB - gb : lift + L.elevB;
            if (L.elevAbsolute && L.elevAbsA != L.elevAbsB) {   // mixed: the relative end becomes absolute too
                if (!L.elevAbsA) L.elevA = ga + L.aboveA;
                if (!L.elevAbsB) L.elevB = gb + L.aboveB;
                L.elevAbsA = L.elevAbsB = true;
                ++mixed;
            }
        }
        if (mixed) LOG_INFO << "[citysim] " << mixed << " deck-to-street links resolved to absolute heights at both ends";
    }

    // Connectivity truth (device: "I don't see the freeway connected yet"):
    // walk the built nav and report whether a car can actually reach the
    // corridor from the streets and come back. If either direction is zero,
    // the weld failed and freeway traffic is impossible — say so loudly.
    {
        int fwLinks = 0, rampLinks = 0, onWelds = 0, offWelds = 0;
        auto street = [&](int li) { const engine::RoadClass k = nav_.links[li].klass; return k != engine::RoadClass::Ramp && k != engine::RoadClass::Freeway; };
        // A street link ENTERING a node counts as much as one leaving it: an on-ramp's street-level
        // run is a one-way link into the node where its deck begins, and a census that only looked
        // at links leaving that node called every such on-ramp unconnected.
        std::vector<char> streetIn(static_cast<std::size_t>(nav_.nodeCount()), 0);
        for (int li = 0; li < nav_.linkCount(); ++li) if (street(li)) streetIn[static_cast<std::size_t>(nav_.links[li].to)] = 1;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            const engine::NavLink& L = nav_.links[li];
            if (L.klass == engine::RoadClass::Freeway) ++fwLinks;
            if (L.klass != engine::RoadClass::Ramp) continue;
            ++rampLinks;
            bool fromStreet = streetIn[static_cast<std::size_t>(L.from)] != 0, toStreet = false;
            for (int ol : nav_.outLinks[L.from]) if (street(ol)) fromStreet = true;
            for (int ol : nav_.outLinks[L.to]) if (street(ol)) toStreet = true;
            if (fromStreet) ++onWelds;
            if (toStreet) ++offWelds;
        }
        // ...and the truth the welds only suggest: WALK it. From every street, how much of the
        // carriageway can a car reach; from the carriageway, can it get back to a street.
        auto reach = [&](auto&& seed) {
            std::vector<char> seen(static_cast<std::size_t>(nav_.linkCount()), 0);
            std::vector<int> stack;
            for (int li = 0; li < nav_.linkCount(); ++li) if (seed(li)) { seen[static_cast<std::size_t>(li)] = 1; stack.push_back(li); }
            while (!stack.empty()) {
                const int li = stack.back(); stack.pop_back();
                for (int ol : nav_.outLinks[nav_.links[li].to]) if (!seen[static_cast<std::size_t>(ol)]) { seen[static_cast<std::size_t>(ol)] = 1; stack.push_back(ol); }
            }
            return seen;
        };
        const std::vector<char> fromStreets = reach(street);
        const std::vector<char> fromFreeway = reach([&](int li) { return nav_.links[li].klass == engine::RoadClass::Freeway; });
        int fwReached = 0, streetsReached = 0;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            if (nav_.links[li].klass == engine::RoadClass::Freeway && fromStreets[static_cast<std::size_t>(li)]) ++fwReached;
            if (street(li) && fromFreeway[static_cast<std::size_t>(li)]) ++streetsReached;
        }
        // STRANDED STREETS: links a car can reach the city from and get back to — the main loop —
        // against those it cannot. A home on one of the rest routes nowhere, and assignPlaces then
        // tries every job in the city from it, a full search each.
        {
            int seedLink = -1;
            for (int li = 0; li < nav_.linkCount() && seedLink < 0; ++li) if (street(li) && nav_.outLinks[nav_.links[li].to].size() >= 3) seedLink = li;
            if (seedLink >= 0) {
                const std::vector<char> down = reach([&](int li) { return li == seedLink; });
                std::vector<std::vector<int>> inLinks(static_cast<std::size_t>(nav_.nodeCount()));
                for (int li = 0; li < nav_.linkCount(); ++li) inLinks[static_cast<std::size_t>(nav_.links[li].to)].push_back(li);
                std::vector<char> up(static_cast<std::size_t>(nav_.linkCount()), 0);
                std::vector<int> stack{seedLink};
                up[static_cast<std::size_t>(seedLink)] = 1;
                while (!stack.empty()) {
                    const int li = stack.back(); stack.pop_back();
                    for (int il : inLinks[static_cast<std::size_t>(nav_.links[li].from)]) if (!up[static_cast<std::size_t>(il)]) { up[static_cast<std::size_t>(il)] = 1; stack.push_back(il); }
                }
                int stranded = 0, streets = 0;
                std::ostringstream where;
                for (int li = 0; li < nav_.linkCount(); ++li) {
                    if (!street(li)) continue;
                    ++streets;
                    if (down[static_cast<std::size_t>(li)] && up[static_cast<std::size_t>(li)]) continue;
                    if (stranded++ < 8) {
                        const Vec2 a = nav_.nodes[static_cast<std::size_t>(nav_.links[li].from)];
                        where << " (" << static_cast<int>(a.x) << ", " << static_cast<int>(a.y) << ")" << (down[static_cast<std::size_t>(li)] ? "" : " unreachable") << (up[static_cast<std::size_t>(li)] ? "" : " no way back");
                    }
                }
                if (stranded) LOG_WARN << "[citysim] " << stranded << " of " << streets << " street links are off the main loop, e.g." << where.str();
            }
        }
        if (fwLinks > 0)
            LOG_INFO << "[citysim] freeway in nav: " << fwLinks
                     << " carriageway links, " << rampLinks << " ramp links, "
                     << onWelds << " street->ramp welds, " << offWelds
                     << " ramp->street welds; from the streets a car reaches " << fwReached << " of "
                     << fwLinks << " carriageway links, and from the carriageway " << streetsReached << " street links"
                     << ((fwReached == 0 || streetsReached == 0)
                             ? "  <-- NOT DRIVABLE"
                             : "");
    }

    // Places (ADR-0066): turn the level-authored destinations into a routable
    // PlaceMap now that the nav graph exists (each entrance snaps to a sidewalk).
    // An unrecognized type tag is warned-and-skipped so one typo can't drop the
    // level. Rebuilt fresh each build() so a reload doesn't accumulate places.
    places_ = PlaceMap{};
    placeOfAuthored_.assign(authoredPlaces_.size(), kNoPlace);
#ifdef RT_ENABLE_SCRIPTING
    {   // THE PLACE KINDS (places.lua): what each place is, its tags, and how a home splits into dwellings
        const std::string code = engine::loadScriptCode("places.lua", "");
        if (!code.empty()) {
            engine::ScriptVM vm;
            std::string err;
            PlaceKindCatalog kinds;
            if (vm.doString(code, &err) && engine::loadPlaceKindCatalog(vm, kinds, &err)) {
                LOG_INFO << "[citysim] place kinds: " << kinds.kinds.size() << ", dwelling rules: " << kinds.dwellings.size() << " (places.lua)";
                sim_.setPlaceKinds(std::move(kinds));
            } else {
                LOG_WARN << "[citysim] places.lua: " << err << " (using the built-in place kinds)";
            }
        }
    }
    {   // THE PEOPLE (people.lua): ages, life stages and the households the dwellings are filled with
        const std::string code = engine::loadScriptCode("people.lua", "");
        if (!code.empty()) {
            engine::ScriptVM vm;
            std::string err;
            PeopleCatalog people;
            if (vm.doString(code, &err) && engine::loadPeopleCatalog(vm, people, &err)) {
                LOG_INFO << "[citysim] people: " << people.bands.size() << " age bands, " << people.households.size()
                         << " household types (people.lua)";
                sim_.setPeople(std::move(people));
            } else {
                LOG_WARN << "[citysim] people.lua: " << err << " (using the built-in people)";
            }
        }
    }
#endif
    const PlaceKindCatalog& kinds = sim_.placeKinds();
    const uint32_t kBeds = kinds.tag("beds");
    int homeDwellings = 0, hallBeds = 0;
    int bySize[6] = {0};   // dwellings by bedrooms: studio, 1 ... 4, 5+
    std::vector<int> beds;
    for (const engine::AuthoredPlace& ap : authoredPlaces_) {
        PlaceType type;
        if (!parsePlaceType(ap.type, type)) {
            LOG_WARN << "citysim: unknown place type '" << ap.type << "' — skipped";
            continue;
        }
        const Vec2 doorHint(ap.ex, ap.ez);
        Real openH = ap.openHour, closeH = ap.closeHour;
        // A GENERATED shop (no authored hours) becomes a cafe, restaurant,
        // supermarket or store: reasons for people to be out (retailKindFor).
        // An authored shop keeps exactly what the level said.
        bool synthetic = false;
        if (type == PlaceType::Shop && openH <= 0.0 && closeH >= 24.0) {
            const RetailKind k = retailKindFor(Vec2(ap.x, ap.z));
            type = k.type;
            openH = k.openHour;
            closeH = k.closeHour;
            synthetic = true;
        }
        const PlaceId pid = places_.add(type, Vec2(ap.x, ap.z), nav_, openH, closeH, ap.capacity,
                                        ap.name, ap.hasEntrance ? &doorHint : nullptr);
        placeOfAuthored_[static_cast<std::size_t>(&ap - authoredPlaces_.data())] = pid;
        if (synthetic) places_.setAuthoredHours(pid, false);
        const int kind = kinds.kindFor(ap.recipe, type);
        if (kind >= 0) {
            const PlaceKind& pk = kinds.kinds[static_cast<std::size_t>(kind)];
            places_.setKind(pid, kind, pk.tags);
            // A HALL's beds: one per so much of its floor (its kind's bed_m2)
            if (pk.m2PerBed > 0 && ap.capacity == 0 && ap.floorArea > 0) {
                places_.setCapacity(pid, static_cast<int>(ap.floorArea / pk.m2PerBed));
                hallBeds += places_[pid].capacity;
            }
            // A HOME's dwellings, one household each: its floor cut by the rule for its use (house, apartments)
            if (type == PlaceType::Home && !(pk.tags & kBeds)) {
                const DwellingRule* rule = kinds.dwellingRule(engine::floorUseFor(ap.recipe, "home"));
                if (rule) {
                    beds.clear();
                    dwellingsOf(kinds, *rule, ap.floorArea, ap.units, beds);
                    places_.setDwellings(pid, beds);
                    homeDwellings += static_cast<int>(beds.size());
                    for (int b : beds) ++bySize[std::clamp(b, 0, 5)];
                }
            }
        }
        if (ap.floorArea > 0) places_.setFloorArea(pid, ap.floorArea);
        places_.setFrontage(pid, ap.frontage);   // where its door fronts, as the lot pass recorded it (ADR-0480)
    }
    if (homeDwellings > 0)
        LOG_INFO << "[citysim] homes: " << places_.countOfType(PlaceType::Home) << " places, " << homeDwellings
                 << " dwellings (studio " << bySize[0] << ", 1-bed " << bySize[1] << ", 2-bed " << bySize[2] << ", 3-bed "
                 << bySize[3] << ", 4-bed " << bySize[4] << ", 5+ " << bySize[5] << ")"
                 << (hallBeds > 0 ? ", hall beds " + std::to_string(hallBeds) : std::string());

    // DENSITY population (roads-v2.1 4c): -1 counts are computed from the
    // graph itself — cars per LANE-km of carriageway, walkers per km of
    // sidewalk (both sides of every street) — so population scales with the
    // city instead of a flat number tuned for one level.
    int carCount = params_.cars, pedCount = params_.pedestrians;
    if (carCount < 0 || pedCount < 0) {
        Real laneKm = 0, walkKm = 0;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            const engine::NavLink& L = nav_.links[li];
            laneKm += L.length * std::max(1, L.lanes) * 1e-3;
            if (L.klass != engine::RoadClass::Freeway &&
                L.klass != engine::RoadClass::Ramp)
                walkKm += L.length * 2.0 * 1e-3;
        }
        // Directed links double-count each roadway: halve to physical km.
        laneKm *= 0.5;
        walkKm *= 0.5;
        const int ceiling = std::max(6, params_.maxAmbient);
        if (carCount < 0)
            carCount = std::clamp(
                static_cast<int>(laneKm * params_.carsPerLaneKm), 6, ceiling);
        if (pedCount < 0)
            pedCount = std::clamp(
                static_cast<int>(walkKm * params_.pedsPerKm), 6, ceiling);
        LOG_INFO << "[citysim] density population: " << carCount << " cars ("
                 << laneKm << " lane-km), " << pedCount << " walkers ("
                 << walkKm << " sidewalk-km)";
        // NO MORE PEOPLE THAN HOMES (Glenn, 2026-11-01: "Do a cap for now"). The streets ask for so many; the
        // dwellings and the halls' beds hold so many (planHouseholds, the same draw assignPlaces makes). A density
        // population is cut to what is housed, cars and walkers alike, keeping walkers enough for the children and
        // the students; the shortfall is the generator's to build. An authored count is the level's word: kept, and
        // whoever it holds past the housing is counted unhoused.
        if (homeDwellings > 0) {
            const int asked = carCount + pedCount;
            const HouseholdPlan fit = planHouseholds(sim_.people(), places_, kBeds, asked, pedCount, params_.seed);
            const int housed = static_cast<int>(fit.people.size());
            if (housed < asked) {
                const int cars = std::min(carCount, std::max(0, housed - fit.walkersOnly));
                const int capped = std::min(cars, static_cast<int>(std::lround(static_cast<double>(carCount) * housed / asked)));
                LOG_INFO << "[citysim] housing holds " << housed << " of the " << asked << " the streets ask for: "
                         << capped << " cars, " << housed - capped << " walkers (" << asked - housed
                         << " short: more dwellings wanted)";
                carCount = capped;
                pedCount = housed - capped;
            }
        }
    }
    // THE FLEET CATALOGUE, loaded before the sim builds. Drivers take their body
    // at build time, so the sim has to know the real sizes first — the recipes
    // publish them (vehicle_classes.lua), and adopting them here is what keeps
    // one set of numbers behind the drawn car, its follow gaps, its parking bay
    // and its physics chassis. The meshes are built further down from these same
    // recipes; only the load is hoisted.
    // How many car variants this city has — the Lua fleet's length, and ZERO
    // when the level ships no vehicle recipes.
    //
    // Vehicles are optional CONTENT. A level with no catalogue draws no cars,
    // which is a legitimate thing for a level to be; it does NOT get a built-in
    // substitute. There used to be one — a hand-written box car in C++ — and
    // because it stood in silently, "the fleet is scripted" and "the fleet is
    // boxes" looked identical from the outside for as long as it took two
    // people to notice the city was full of blocks. Absence is a data decision;
    // say `"vehicles": "vehicles.lua"` to have cars, or `"cars": 0` not to.
    int fleetVariants = 0;
    driving::VehicleClasses drivingClasses;   // the new driving model's (#248): one per fleet class, the first slot's
    std::vector<int> drivingClassOfSlot;      // ...and each fleet slot's class among them (-1: none read)
#ifdef RT_ENABLE_SCRIPTING
    engine::ScriptVM fleetVM_;
    bool fleetScripted = false;
    if (!params_.vehicleScript.empty()) {
        // Same layering as VehicleSystem's spawn path: mesh.* procgen builders
        // plus the module loader — vehicles.lua begins with
        // `require "vehicle_classes"`, so a bare VM fails its very first line
        // and every city silently drew the retired box fleet ("why are we
        // using the old cars?").
        engine::openProcgenLibrary(fleetVM_);
        engine::openModuleLoader(fleetVM_, engine::makeModuleSource(""));
        std::string err;
        const bool scriptOk = fleetVM_.doString(params_.vehicleScript, &err);
        if (!scriptOk) {
            LOG_WARN << "citysim vehicles script: " << err
                     << " — this level draws NO cars";
        } else if (!params_.fleet.empty()) {
            // the level's named fleet (citysim.fleet): pointed at AFTER the script has run
            std::string ferr;
            if (engine::selectFleet(fleetVM_, params_.fleet, &ferr))
                LOG_INFO << "[citysim] fleet: vehicle." << params_.fleet;
            else
                LOG_WARN << "citysim fleet '" << params_.fleet << "': " << ferr << " — keeping the default fleet";
        }
        carConfigs_.clear();
        carSeatGeom_.clear();
        if (!scriptOk) {
        } else if (int slots = engine::fleetSlotCount(fleetVM_); slots > 0) {
            // THE ASSET decides how many cars the fleet has. This used to be a
            // C++ constant, so a thirteenth slot was never built and a shorter
            // fleet gave every missing slot a box car.
            // Now the asset simply says how many there are.
            fleetVariants = slots;
            std::vector<VehicleBody> catalogue;
            busVariants_.clear();
            for (int v = 0; v < slots; ++v) {
                VehicleBody body = vehicleFleetBody(v);
                Vec3 size(0, 0, 0);
                std::string className;
                std::string slotErr;
                if (!engine::loadFleetCatalogueEntry(fleetVM_, v, size, className,
                                                     &slotErr))
                    LOG_WARN << "citysim vehicles fleet[" << v << "]: " << slotErr
                             << " (using built-in dimensions)";
                // A slot with no `size` keeps the built-in one (legacy recipe).
                else if (size.x > 0 && size.y > 0 && size.z > 0) {
                    body.width = size.x;
                    body.height = size.y;
                    body.length = size.z;
                }
                // Which slot is the BUS, by the script's own class name --
                // so the transit fleet draws as a bus without a hard-coded
                // index that a reordered fleet would silently break.
                // ...and the slot IS a bus to the sim too: the type came from the built-in table by slot number
                // (slot 24 wraps to a sedan), so ambientSlotFor, which skips Bus slots, dealt ordinary cars the bus
                // body -- 1 in 25 cars a 12 m "sedan" wearing NOT IN SERVICE (Glenn: the signs stacked over the kerb)
                // ...and the FIRE ENGINE, stationed in the fire stations' bays (never dealt to traffic), by its name
                // too -- first, so the built-in table's type for its slot number (it stands where the bus did) is
                // never read
                // ...and the TAXI, dealt to the cabs alone (a yellow cab with nobody's fare is not ambient traffic)
                if (className == "fire_engine") { fireVariant_ = v; body.type = VehicleType::FireEngine; }
                else if (className == "taxi") { taxiVariant_ = v; body.type = VehicleType::Taxi; }
                else if (className.rfind("bus", 0) == 0) { busVariants_.push_back(v); body.type = VehicleType::Bus; }
                else if (body.type == VehicleType::Bus) busVariants_.push_back(v);
                else if (body.type == VehicleType::FireEngine) body.type = VehicleType::BoxTruck;
                busVariant_ = busVariants_.empty() ? -1 : busVariants_.front();
                // its handling: what it drives with once physical (item 23)
                engine::VehicleSpec handling;
                std::string hErr;
                if (!engine::loadFleetHandling(fleetVM_, v, handling, &hErr))
                    LOG_WARN << "citysim vehicles fleet[" << v << "]: " << hErr << " (it cannot be made physical)";
                carConfigs_.push_back(handling.config);
                carSeatGeom_.push_back(handling.seat);
                {   // what a car of this slot is built as once the player takes it (ADR-0355)
                    std::string recipe;
                    engine::Vec3 paint(0.5, 0.5, 0.5);
                    if (!engine::loadFleetDrivable(fleetVM_, v, recipe, paint)) recipe.clear();
                    carDrivable_.push_back(recipe);
                    carPaint_.push_back(paint);
                }
                {   // how it drives, for the new driving model (#248)
                    driving::VehicleClass dc;
                    std::string dErr;
                    int cls = -1;
                    if (!engine::loadFleetDrivingClass(fleetVM_, v, dc, &dErr))
                        LOG_WARN << "citysim vehicles fleet[" << v << "] driving class: " << dErr;
                    else if ((cls = drivingClasses.indexOf(dc.name)) < 0)
                        cls = static_cast<int>(drivingClasses.add(std::move(dc)));
                    drivingClassOfSlot.push_back(cls);
                }
                catalogue.push_back(body);
            }
            LOG_INFO << "[citysim] fleet: " << slots << " slots, bus variant "
                     << busVariant_ << (busVariant_ < 0 ? " (NO BUS MESH)" : "") << " (" << busVariants_.size()
                     << " bus bodies)";
            sim_.setFleet(std::move(catalogue));
            fleetScripted = true;
        }
    }
#endif

    // THE WALKS (campus gap pass): the parks' paths and the quad's walks join the walking network, after the
    // streets' own analyses and the places (link and node indices so far are unchanged) and before the sim reads it.
    // Footpaths are walker-only (vehicle routing skips them; nearestNode never answers with one), so a route on foot
    // simply finds the quad when it is the shorter way.
    // THEY ARE THE ROAD MODEL'S WALKWAYS (ADR-0476), read through it with where each end lands as the lot pass stated
    // it (street_map.h appendWalkways): EVERY city's model that holds walkways, as the loader published them, each city's
    // walks joined to that city's streets.
    std::vector<std::pair<int, std::shared_ptr<const engine::RoadModel>>> walkModels;
    world.each<engine::LevelRoadModel>([&](Entity, engine::LevelRoadModel& m) {
        if (!walkModels.empty()) return;
        for (std::size_t c = 0; c < m.cities.size(); ++c)
            if (m.cities[c] && !m.cities[c]->walkways.empty()) walkModels.emplace_back(static_cast<int>(c), m.cities[c]);
    });
    if (!walkModels.empty() && !std::getenv("RT_NO_FOOTPATHS")) {
        const auto tf0 = std::chrono::steady_clock::now();
        engine::NavGraph::FootpathReport fr;
        for (const std::pair<int, std::shared_ptr<const engine::RoadModel>>& wm : walkModels) {
            const engine::NavGraph::FootpathReport one = engine::appendWalkways(nav_, *wm.second, wm.first);
            fr.nodes += one.nodes;
            fr.links += one.links;
            fr.tees += one.tees;
            fr.streetJoins += one.streetJoins;
            fr.unjoined += one.unjoined;
        }
        LOG_INFO << "[citysim] footpaths joined in " << std::chrono::duration<double>(std::chrono::steady_clock::now() - tf0).count() << " s";
        LOG_INFO << "[citysim] footpaths: " << fr.links << " walk links over " << fr.nodes << " nodes, " << fr.tees
                 << " tees, " << fr.streetJoins << " ends joined to the streets, " << fr.unjoined << " ends joined to nothing (dead ends)";
        // a paseo's shops: their doors are on its walks, not the street round the block (every shopper stood at the
        // end of the paseo, by the restaurant there)
        LOG_INFO << "[citysim] places entered from a walk: " << places_.snapToWalks(nav_);
    }
    sim_.setJunctionPad(sidewalk_);
    sim_.ambientBus = params_.ambientBus;   // the households' cars' bodies (ambientSlotFor)
    // THE LEVEL'S CARS are the driving's own: entities that drive round for ever, made when it begins (startDrivingV2;
    // ADR-0453), on a level that names a rule set. The sim makes no drivers of its own (the old driving went with
    // ADR-0461): a level with no rule set has no traffic.
    const bool drives = !params_.drivingRules.empty();
    const int drivingCars = drives ? carCount : 0;
    if (!drives && carCount > 0)
        LOG_INFO << "[citysim] " << carCount << " cars asked for, but the level names no rule set (citysim.drivingRules): no traffic";
    { const auto tT0 = std::chrono::steady_clock::now();
    sim_.build(nav_, pedCount, params_.seed);
    // THE DRIVING (#234): the driving library runs on a level that names a rule set (given its road below)
    if (drives) {
        sim_.useDrivingV2(true);
        if (drivingClasses.empty()) {   // (no scripted fleet: one class a built-in slot)
            drivingClasses = builtInDrivingClasses();
            drivingClassOfSlot.clear();
            for (std::size_t s = 0; s < drivingClasses.size(); ++s) drivingClassOfSlot.push_back(static_cast<int>(s));
        }
        for (const std::string& p : driving::validate(drivingClasses)) LOG_WARN << "[citysim] driving v2 vehicle class: " << p;
        sim_.setDrivingVehicleClasses(std::move(drivingClasses));
        LOG_INFO << "[citysim] driving v2: the new driving library is on";
    }
    LOG_INFO << "[citysim] startup: sim.build " << std::chrono::duration<double>(std::chrono::steady_clock::now() - tT0).count() << " s"; }
    // THE LEVEL'S ROAD MODEL, handed to the sim on every level with a lane-built city (ADR-0480): its lane level holds the
    // walks its people walk by and, where the level names a rule set, the lanes the driving drives (ADR-0436), and what
    // the road is there is asked of the model. As the loader published it (engine::LevelRoadModel, ADR-0435): nothing is
    // made here; the model was read from its cache or built by the one function that makes it (road_model_cache.h
    // obtainLevelRoadModel), its turns checked against the level's own fleet, which that function reads from the same
    // vehicle script this host read above.
    {
        std::shared_ptr<engine::RoadModel> model;
        std::string line = "no lane graph: the level published no road model";
        bool lanesCity = false;
        world.each<engine::LevelRoadModel>([&](Entity, engine::LevelRoadModel& m) {
            if (lanesCity || m.cities.empty()) return;
            lanesCity = true;
            line = m.laneLevelReport;
            model = m.builtCity();
        });
        if (model) LOG_INFO << "[citysim] the level's road model, lane level: " << line;
        else if (lanesCity) LOG_WARN << "[citysim] the level's road model is not built: " << line;
        if (const driving::Driving* d2 = sim_.drivingV2()) {
            if (model) {
                // a class carrying a tag the road was not checked for can use no turn open to that tag: say so
                const std::vector<std::string>& roadTags = model->laneLevel.vehicleTagNames;
                for (const driving::VehicleClass& c : d2->vehicleClasses().all())
                    for (const std::string& tag : c.tags)
                        if (std::find(roadTags.begin(), roadTags.end(), tag) == roadTags.end())
                            LOG_WARN << "[citysim] driving v2: vehicle class " << c.name << " carries the tag \"" << tag
                                     << "\", which the level's roads were not checked for";
            } else {
                LOG_WARN << "[citysim] driving v2: " << line;
            }
        }
        sim_.setRoads(model, line);
    }
    // (The fire stations' engines, parked in their bays as rows of the old sim's vehicle list, went with the old driving:
    // ADR-0461. A fire engine comes back as a parked car entity. The garages' stalls, the old driving's bays, went with
    // it too: ADR-0462.)
    // THE SEATS out in the city (the furniture library, M5): every sit spot of the outdoor furniture the loader
    // laid -- park benches, plaza benches, café chairs -- for the strollers to use. Interiors are not out yet (they
    // stream in), and nobody walks into one to sit anyway.
    {
        const engine::FurnitureLibrary& flib = engine::FurnitureLibrary::global();
        std::vector<CitySim::SeatSpot> seats;
        world.each<engine::Interactables>([&](Entity, engine::Interactables& set) {
            for (const engine::InteractPiece& ip : set.pieces) {
                const engine::FurnitureAsset* fa = flib.find(static_cast<engine::Piece>(ip.piece));
                if (!fa) continue;
                bool outdoor = false;
                uint32_t tags = 0;   // (the library's tags the catalog knows: a lounger's "beach")
                for (const std::string& tg : fa->tags) {
                    outdoor = outdoor || tg == "outdoor";
                    tags |= spotTagFromName(tg);
                }
                if (!outdoor) continue;
                for (const engine::FurnVerb& v : fa->verbs) {
                    // A PLACE TO LIE (a sun lounger, a towel; Glenn, 2026-11-02): one spot for the verb, the middle of
                    // the spots it takes, the body's feet pointing away from the eye at its head
                    if (v.verb == engine::Verb::Lie) {
                        Vec3 mid(0, 0, 0);
                        int k = 0;
                        for (std::size_t q = 0; q < fa->spots.size(); ++q)
                            if (v.spots & (1u << q)) { mid = mid + fa->spots[q].at; ++k; }
                        if (k == 0) continue;
                        mid = mid * (1.0 / k);
                        const Vec3 at = engine::piecePoint(ip.xform, mid), eye = engine::piecePoint(ip.xform, v.eye);
                        const Vec2 along(at.x - eye.x, at.z - eye.z);
                        if (along.lengthSquared() < 1e-6) continue;
                        CitySim::SeatSpot s;
                        s.kind = SpotKind::Lie;
                        s.pos = Vec2(at.x, at.z);
                        s.face = normalize(along);
                        s.hip = at.y;
                        s.tags = tags;
                        seats.push_back(s);
                        continue;
                    }
                    if (v.verb != engine::Verb::Sit) continue;
                    for (std::size_t k = 0; k < fa->spots.size(); ++k) {
                        if (!(v.spots & (1u << k))) continue;
                        const Vec3 at = engine::piecePoint(ip.xform, fa->spots[k].at);
                        const Vec3 f = engine::pieceDir(ip.xform, Vec3(0, 0, 1));
                        CitySim::SeatSpot s;
                        s.pos = Vec2(at.x, at.z);
                        s.face = normalize(Vec2(f.x, f.z));
                        s.hip = at.y;
                        s.tags = tags;
                        seats.push_back(s);
                    }
                }
            }
        });
        // THE RIMS (a fountain's): seats with no furniture, the hip measured from the ground under them
        for (const auto& q : sitSpots_) {
            CitySim::SeatSpot s;
            s.pos = Vec2(q[0], q[1]);
            s.face = normalize(Vec2(q[2], q[3]));
            s.hip = (heightAt_ ? heightAt_(q[0], q[1]) : 0.0) + q[4];
            seats.push_back(s);
        }
        // THE TRACKS (activity spots, behaviour plan step 3): each loop the sim's, with eight places round it to start a
        // run from -- a jogger walks onto the track there, runs laps, and walks off
        {
            std::vector<std::vector<Vec2>> loops;
            for (const auto& lp : jogLoops_) {
                std::vector<Vec2> pts;
                for (const auto& q : lp) pts.push_back(Vec2(q[0], q[1]));
                loops.push_back(std::move(pts));
            }
            sim_.setLoops(std::move(loops));
            for (int li = 0; li < sim_.loopCount(); ++li)
                for (int k = 0; k < 8; ++k) {
                    CitySim::ActivitySpot s;
                    s.kind = SpotKind::Jog;
                    s.loop = li;
                    s.loopS = sim_.loopLength(li) * k / 8.0;
                    Vec2 tg(1, 0);
                    s.pos = sim_.loopPoint(li, s.loopS, &tg);
                    s.face = tg;
                    s.hip = 0;
                    seats.push_back(s);
                }
        }
        const std::size_t offered = seats.size();
        sim_.setSeats(std::move(seats));
        {   // THE AREAS a group plays on (a pitch) or settles on (a lawn)
            std::vector<CitySim::ActivityArea> areas;
            for (const auto& a : activityAreas_) {
                CitySim::ActivityArea ar;
                ar.kind = a.kind;
                ar.center = Vec2(a.cx, a.cz);
                ar.axis = Vec2(a.ax, a.az);
                ar.halfL = a.halfL;
                ar.halfW = a.halfW;
                areas.push_back(ar);
            }
            sim_.setAreas(std::move(areas));
            if (!sim_.areas().empty()) LOG_INFO << "[citysim] activity areas: " << sim_.areas().size() << " (pitches, lawns)";
        }
        if (offered > 0) LOG_INFO << "[citysim] seats: " << sim_.seats().size() << " of " << offered << " outdoor seats reachable from the paths";
    }
    sim_.setWander(params_.wander);
    sim_.setDaySchedule(params_.schedule);
    // Three-tier traffic (P4): the level's opt-in. The bubble only engages
    // once a player position is fed each fixed step (see step() below) — the
    // build warm-up therefore runs everything K, exactly as before.
    sim_.tieringEnabled = params_.tieredAgents;
    // RT_NEAR_TARGET=<n>: hold about n agents in the full sim round the player (the ring adapts; the GPU crowd shows
    // the rest) -- for populations past what a full ring can carry. The island at 309k: 28 -> 66 fps at 1500 (K
    // 3,501 -> 1,350; ring x0.46). NOT on by default: metro_planned's ring holds more than 1500, and the smaller
    // ring there lets a junction near (45, -195) gridlock (eastbound, northbound and southbound queues all
    // waiting), which strands the regional bus and its riders (metro_planned_regional_bus_joins_the_towns).
    // A level whose population needs it sets "nearTarget" (island_8_nature: 1500); RT_NEAR_TARGET overrides.
    sim_.nearTarget = params_.nearTarget;
    if (const char* nt = std::getenv("RT_NEAR_TARGET")) sim_.nearTarget = std::max(0, std::atoi(nt));
    sim_.farTarget = params_.farTarget;   // (the busy streets: downtown's towers fill the far ring too)
    if (const char* ft = std::getenv("RT_FAR_TARGET")) sim_.farTarget = std::max(0, std::atoi(ft));
    sim_.dormancyEnabled = params_.dormantAgents;
    sim_.pedPromoteRadius = params_.pedPromoteRadius;
    sim_.pedDemoteRadius = params_.pedDemoteRadius;
    // THE NEW MODEL BEGINS (v2 with a rule set; #255, ADR-0453): the level's cars are made, entities that drive round
    // for ever, put down on the lane graph and driven by the rules. A rule set that is refused leaves the level loading
    // with the new model off and no traffic.
    if (sim_.drivingV2() && !params_.drivingRules.empty()) {
        std::string report;
        if (sim_.startDrivingV2(world, params_.drivingRules, drivingCars, drivingClassOfSlot, &report)) {
            LOG_INFO << "[citysim] driving v2: " << report << ", by the rule set " << params_.drivingRules;
        } else {
            LOG_WARN << "[citysim] driving v2 is OFF on this level, so it has no traffic: " << report;
            sim_.useDrivingV2(false);
        }
    } else if (sim_.drivingV2()) {
        LOG_INFO << "[citysim] driving v2: no rule set named (citysim.drivingRules), so it drives no cars";
    }
    // What this LEVEL asked for, beside the density line above. Nothing used
    // to print it, so a level that silently failed to opt into tiering looked
    // exactly like one that had (2026-09-16).
    LOG_INFO << "[citysim] sim tiers: tiered=" << (params_.tieredAgents ? "on" : "off")
             << " dormancy=" << (params_.dormantAgents ? "on" : "off")
             << " localHz=" << params_.localHz
             << " maxAmbient=" << params_.maxAmbient;
    // DORMANCY MUST BE FURTHER OUT THAN ANYTHING DRAWN. A dormant agent's
    // position is rebuilt from its schedule when it wakes, so if it slept while
    // its parked car was still on screen, world-hours could pass and the wake
    // would move a car the player is looking at. Keeping the dormant radius
    // outside the parked-car draw radius makes that unobservable — and this is
    // exactly the coupling someone breaks later by tuning sceneryRadius up, so
    // it is enforced here rather than written in a comment somewhere.
    if (sim_.dormancyEnabled &&
        sim_.dormantResumeRadius <= params_.sceneryRadius) {
        const Real want = params_.sceneryRadius * 2.0;
        LOG_WARN << "[citysim] dormantResumeRadius ("
                 << sim_.dormantResumeRadius << " m) is inside the parked-car "
                    "draw radius (" << params_.sceneryRadius
                 << " m) — cars could visibly jump when their owner wakes; "
                    "raising to " << want << " m";
        sim_.dormantResumeRadius = want;
        sim_.dormantRadius = std::max(sim_.dormantRadius, want * 1.2);
    }
#ifdef RT_ENABLE_SCRIPTING
    // Scripted goal tables (ADR-0064): a level's citysim block may name an
    // agents.lua-style script (level_loader reads its text into the config);
    // its archetype tables replace the built-ins BEFORE the warm-up below, so
    // the whole visible day runs on the scripted goals. Lua runs only here, at
    // load — every per-tick transition executes in C++ from the tables.
    // THE ACTIVITY CATALOG (activities framework): what there is to go and do, and the menus that choose -- from
    // assets/scripts/activities.lua when it is there (the built-in catalog otherwise: the same, by test)
    {
        const std::string code = engine::loadScriptCode("activities.lua", "");
        if (!code.empty()) {
            engine::ScriptVM vm;
            std::string err;
            citysim::ActivityCatalog cat;
            if (vm.doString(code, &err) && engine::loadActivityCatalog(vm, cat, &err)) {
                LOG_INFO << "[citysim] activities: " << cat.defs.size() << " activities, " << cat.menus.size() << " menus (activities.lua)";
                sim_.setActivityCatalog(std::move(cat));
            } else {
                LOG_WARN << "[citysim] activities.lua: " << err << " (using the built-in catalog)";
            }
        }
    }
    {   // THE ROLES (roles.lua): every resident's day, built from its role's parts
        const std::string code = engine::loadScriptCode("roles.lua", "");
        if (!code.empty()) {
            engine::ScriptVM vm;
            std::string err;
            citysim::RoleCatalog roles;
            if (vm.doString(code, &err) && engine::loadRoleCatalog(vm, roles, &err)) {
                LOG_INFO << "[citysim] roles: " << roles.roles.size() << " (roles.lua)";
                sim_.setRoleCatalog(std::move(roles));
            } else {
                LOG_WARN << "[citysim] roles.lua: " << err
                         << " (using the built-in roles)";
            }
        }
    }
    if (!params_.agentScript.empty()) {
        engine::ScriptVM vm;
        std::string err;
        citysim::GoalTable ped, driver;
        const char* pedName = params_.wander ? "wander_pedestrian" : "schedule";
        const char* drvName = params_.wander ? "wander_driver" : "schedule";
        if (vm.doString(params_.agentScript, &err) &&
            engine::loadGoalTable(vm, pedName, ped, &err) &&
            engine::loadGoalTable(vm, drvName, driver, &err)) {
            sim_.setGoalTables(std::move(ped), std::move(driver));
            // The day OUT's own table, when the script has one (`stroller`): every role that spends the day out.
            citysim::GoalTable stroll;
            std::string serr;
            if (!params_.wander && engine::loadGoalTable(vm, "stroller", stroll, &serr))
                for (const ResidentRole& r : sim_.roles().roles)
                    if (!r.day.commute) sim_.setRoleTable(r.name, stroll);
        } else {
            LOG_WARN << "citysim agents script: " << err << " (using built-ins)";
        }
    }
#endif
    // Make the agents LIVE in the city (ADR-0066 Phase 3): pin each one's home +
    // job to real places and seed the relationship table, so the commute below
    // routes them to actual buildings. No-op when the level authored no places
    // (then the built random home/work schedule stands). After setWander so the
    // (persistent) mode is settled; before the warm-up so day one runs on places.
    // The population cache (CitySim::setPopulationCacheDir): off under RT_NOCACHE, like the level bundle.
    if (!std::getenv("RT_NOCACHE")) sim_.setPopulationCacheDir("cache/population");
    { const auto tT0 = std::chrono::steady_clock::now();
    sim_.assignPlaces(places_, nav_);
    if (!sim_.households().households.empty())
        LOG_INFO << "[citysim] population: " << sim_.populationLine()
                 << (sim_.unhousedCount() > 0 ? ", " + std::to_string(sim_.unhousedCount()) + " unhoused (more than the housing holds)" : std::string());
    {   // THE ACTIVITY SPOTS, by kind and where (tagged once the places are known)
        int byKind[static_cast<int>(SpotKind::Count)] = {0}, campus = 0, sports = 0, campusJog = 0;
        for (const CitySim::ActivitySpot& sp : sim_.spots()) {
            ++byKind[static_cast<int>(sp.kind)];
            campusJog += sp.kind == SpotKind::Jog && (sp.tags & spot_tag::kCampus) ? 1 : 0;
            campus += (sp.tags & spot_tag::kCampus) ? 1 : 0;
            sports += (sp.tags & spot_tag::kSports) ? 1 : 0;
        }
        std::string line;
        for (int k = 0; k < static_cast<int>(SpotKind::Count); ++k)
            if (byKind[k]) line += std::string(line.empty() ? "" : ", ") + std::to_string(byKind[k]) + " " + spotKindName(static_cast<SpotKind>(k));
        LOG_INFO << "[citysim] activity spots: " << (line.empty() ? std::string("none") : line) << " | " << campus << " on a campus, "
                 << sports << " by a sports field (" << campusJog << " campus jog)";
    }
    LOG_INFO << "[citysim] startup: assignPlaces " << std::chrono::duration<double>(std::chrono::steady_clock::now() - tT0).count() << " s"; }
    if (sim_.populationCache().used)
        LOG_INFO << "[citysim] population: " << (sim_.populationCache().hit ? "read from" : "decided, cached in")
                 << " cache/population/" << std::hex << sim_.populationCache().key << std::dec << ".pop";
    {
        const CitySim::VenueStaffing at10 = sim_.venueStaffing(10.0), at22 = sim_.venueStaffing(22.0);
        LOG_INFO << "[citysim] staffed places: " << at10.gated << " shops, cafes, restaurants, bars and clubs open only while"
                 << " someone is on shift (" << at10.neverStaffed << " never staffed); staffed at 10:00 " << at10.staffedNow
                 << ", at 22:00 " << at22.staffedNow;
    }

    // ONE CLOCK. A staged day/night cycle owns the world's hour and rate:
    // the sim opens at the sky's hour and its schedules run at the sky's
    // pace, so a 30-minute day is 30 minutes for the commuters too. The two
    // clocks used to be unrelated — an 8-minute sim day under a 50-second
    // sky, rush hour at any hour of the light. The level's authored
    // citysim.hoursPerSecond/startHour still rule where no cycle is staged:
    // static-sun levels, HDR environments, headless hosts and the tests.
    if (worldClockHour_ >= 0.0) {
        LOG_INFO << "[citysim] clock follows the day/night cycle: opens at "
                 << worldClockHour_ << " h, " << worldClockRate_
                 << " h/s (level authored " << params_.startHour << " h, "
                 << params_.hoursPerSecond << " h/s)";
        params_.startHour = worldClockHour_;
        params_.hoursPerSecond = worldClockRate_;
    }

    // IS THERE ENOUGH DAY TO LIVE IN? Agents move at true metres per second
    // while the clock runs at hoursPerSecond, so a journey costs
    // (distance / speed) * hoursPerSecond * 3600 times more of the day than it
    // would in reality. At the historical default of 0.05 that is a 180x
    // compression: on an 8 km city a 3 km commute eats 12.5 in-world hours, the
    // agent arrives as its shift ends, and the 50 m walk from a parking bay to
    // a door costs nearly two hours. The schedule silently becomes unlivable
    // and the only symptom is a city permanently stuck in rush hour — so say it
    // at load instead.
    {
        // (on foot: the sim's people walk since ADR-0463; the cars are the driving's)
        const Real walkHours =
            sim_.commuteSecondsWalk() * params_.hoursPerSecond;
        auto report = [&](const char* who, Real hours, Real secs) {
            if (hours <= 0) return;
            LOG_INFO << "[citysim] median " << who << " commute " << hours
                     << " in-world hours (" << secs << " s at "
                     << params_.hoursPerSecond << " h/s)";
            // A round trip plus a shift has to fit inside a 24 h day.
            if (hours > 2.0)
                LOG_ERROR << "[citysim] " << who << "S CANNOT COMPLETE A DAY: "
                          << hours << " in-world hours each way. Either lower "
                             "citysim.hoursPerSecond (try "
                          << params_.hoursPerSecond * (1.0 / hours)
                          << "), or shorten this group's journeys — for walkers "
                             "that means homes nearer their work, or transit.";
        };
        report("WALKER", walkHours, sim_.commuteSecondsWalk());
    }

    // Open the city at its authored hour, with everyone already where their own
    // day puts them. This replaces a 400-step warm-up that existed only to get
    // the population moving before the level appeared: at city scale that is
    // thousands of agents stepped at full fidelity for 40 sim-seconds, and it
    // could only ever express "a little after the build hour" — opening at
    // 22:00 would have meant simulating 27 in-world hours to reach it.
    // The rate FIRST: seeding converts commute seconds into a share of the day,
    // and step() has not run yet to supply it. Without this the sim seeded at
    // the 0.05 default while the level ran at 0.004 — a travel window 12.5x too
    // wide, which put ~3/4 of the city mid-commute the moment it loaded.
    sim_.setHoursPerSecond(params_.hoursPerSecond);
    sim_.seedFromSchedule(params_.startHour);

    MeshHandle pedMesh{}, lensMesh{};
    if (assets) {
        pedMesh = assets->acquireMesh(engine::buildStandingPerson(0.0, 0), "city:ped");   // (the crowd far off: six blocks)
        Real e = params_.signalLensSize;
        // a thin plate, turned with the head (signalLensPose), a centimetre proud of the head's own lamp -- it was a
        // 0.34 m CUBE, unturned, poking crooked through the lamp it lit (Glenn: "the stoplights are doubled up ...
        // staggered over each other and z-fighting")
        lensMesh = assets->acquireMesh(MeshBuilder::box(Vec3(e, e, 0.02)), "city:signal");
    }

    // One instance group per fleet slot; each is a body TYPE + SIZE (from the sim's
    // shared fleet table) painted its slot colour, so the traffic is a mixed fleet
    // of sedans, hatchbacks, SUVs, pickups, a van, and a box truck — every NPC car
    // built the same way as (and to scale with) the player's. The car entities draw
    // into them by their fleet slot (syncCarEntities).
    carGroups_.clear();
    sim_.poses().models.groups = 0;   // (the sim's count of drawn models: carGroups_'s, kept as they are made)
    sim_.poses().reset();             // (the fit memories are the last city's cars')
    carMidGroups_.clear();
    carLights_.clear();
    carChassis_.clear();
    carWheels_.clear();
    carGlassGroups_.clear();
    carGlassOpaqueGroups_.clear();
    carInteriorGroups_.clear();
    carSeeInto_.clear();
    carGlassOpacity_.clear();
    carOpenSeat_.clear();
    nearSwap_.clear();
    carDriverSeat_.clear();
    carRearHip_.clear();
    carHasRear_.clear();
    carHasDriver_.clear();
    {
#ifdef RT_ENABLE_SCRIPTING
        // Data-driven fleet bodies (ADR-0065). The recipes were already read
        // above — before sim_.build, because the sim adopts their dimensions —
        // so this only turns each one into an instanced mesh. Lua ran once, at
        // load, and a baked mesh is plain data — determinism is intact.
        const bool vehScript = assets && fleetScripted;
#endif
        int fleetTriangles = 0, midTriangles = 0, midCount = 0;
        for (int v = 0; v < fleetVariants; ++v) {
            MeshHandle mh{}, chassisMh{};
            std::vector<LampMarker> lights;
            std::vector<CarWheel> wheels;
            engine::RenderMesh glassMesh, interiorMesh, midMesh;
            Real midFrom = 0;
            bool seeInto = false;
            bool openSeat = false;
            float glassOpacity = 0.30f;
            Vec3 driverSeat(0, 0, 0), rearHip(0, 0, 0);
            bool hasRear = false;
            bool hasDriver = false;
            if (assets) {
                engine::RenderMesh mesh;
                bool scripted = false;
#ifdef RT_ENABLE_SCRIPTING
                // Geometry, finally: this is the call that runs the slot's
                // build() and costs a mesh.car shell. It happens only with an
                // AssetManager to upload into — the catalogue above was read
                // without it, so a headless sim gets real car sizes for free.
                engine::CarBodyRecipe recipe;
                std::string slotErr;
                if (vehScript &&
                    engine::loadFleetCarBody(fleetVM_, v, recipe, &slotErr)) {
                    mesh = std::move(recipe.mesh);
                    scripted = true;
                        // The WHEEL-LESS form of the same car, for a promoted
                        // (commandeered) one — VehicleSystem gives that car real
                        // physics wheels, so the baked pair must come off. Empty
                        // when the recipe publishes no separate wheelset.
                        if (!recipe.chassis.vertices.empty())
                            chassisMh = assets->acquireMesh(
                                recipe.chassis,
                                "city:carbody" + std::to_string(v));
                        for (const engine::WheelPlacement& w : recipe.wheels)
                            wheels.push_back({w.pos, w.radius, w.width,
                                              w.steered, w.driven, w.handBrake});
                        // RETAIN the Lua fleet's light markers (ADR-0065 follow-up):
                        // headlight_l/r + taillight_l/r become the emissive lamps
                        // baked in syncCarLamps. Previously parsed-and-dropped.
                        for (const engine::Attachment& att : recipe.lights)
                            lights.push_back({att.name, att.pos});
                        glassMesh = std::move(recipe.glass);
                        interiorMesh = std::move(recipe.interior);
                        midMesh = std::move(recipe.mid);
                        midFrom = static_cast<Real>(recipe.midFrom);
                        seeInto = recipe.seeInto;
                        glassOpacity = static_cast<float>(recipe.glassOpacity);
                        openSeat = recipe.openSeat;
                        driverSeat = recipe.driverSeat;
                        hasDriver = recipe.hasDriverSeat;
                        rearHip = recipe.rearHip;
                        hasRear = recipe.hasRearHip;
                } else if (vehScript) {
                    LOG_ERROR << "[citysim] fleet slot " << v << ": " << slotErr
                              << " — this slot draws NO car";
                }
#endif
                if (scripted) {
                    const int tris = static_cast<int>(mesh.indices.size() / 3);
                    fleetTriangles += tris;
                    // A car that is all wheels and no shell must SHOUT. This
                    // exact skew shipped once: a binary older than the asset
                    // ignored the recipe's `body` mesh, read only its `parts`
                    // (the wheels) and drew headless cars — wheels and lamps
                    // rolling down the street. A real shell is thousands of
                    // triangles; wheels alone are ~48.
                    if (tris < 100)
                        LOG_ERROR << "[citysim] fleet slot " << v << ": only "
                                  << tris << " triangles — this is not a car "
                                     "body (stale binary vs vehicles.lua, or a "
                                     "recipe with no `body` mesh)";
                    mh = assets->acquireMesh(mesh, "city:car" + std::to_string(v));
                }
            }
            carLights_.push_back(std::move(lights));
            carChassis_.push_back(chassisMh);
            carWheels_.push_back(std::move(wheels));
            // The glass, clear (a see-into vehicle always; any other car while it is near the player) and
            // opaque (traffic at large), and the cabin a near car shows through it.
            {
                Entity ge{}, go{}, gi{};
                MeshHandle glassH{}, cabinH{};
                if (assets && !glassMesh.vertices.empty()) {
                    const MeshHandle gh = assets->acquireMesh(glassMesh, "city:carglass" + std::to_string(v));
                    glassH = gh;
                    ge = world.create();
                    InstanceGroup gg;
                    gg.mesh = gh;
                    engine::RenderMaterial gm;
                    gm.albedo = Vec3(1, 1, 1);
                    gm.metallic = 0.0f;
                    gm.roughness = 0.06f;
                    gm.opacity = glassOpacity;
                    gg.material = gm;
                    gg.renderLayer = engine::LayerSim;
                    gg.drawClass = engine::DrawClass::SimBody;
                    world.add<InstanceGroup>(ge, gg);
                    if (!seeInto) {
                        // OPAQUE GLASS that reads as glass: dark, near-mirror smooth, so the sky and the street
                        // show in it (Fresnel does the rest at grazing angles) -- no transparent pass per car.
                        go = world.create();
                        InstanceGroup og;
                        og.mesh = gh;
                        engine::RenderMaterial om;
                        om.albedo = Vec3(1.0, 1.05, 1.12);   // x the glass mesh's own dark tint, a little toward the sky
                        om.metallic = 0.15f;
                        om.roughness = 0.03f;
                        og.material = om;
                        og.renderLayer = engine::LayerSim;
                        og.drawClass = engine::DrawClass::SimBody;
                        world.add<InstanceGroup>(go, og);
                    }
                }
                // (a see-into body's cabin too -- a bus's saloon, an open car's seats: drawn always, in this matte
                // material, not in the chassis's paint)
                if (assets && !interiorMesh.vertices.empty()) {
                    gi = world.create();
                    InstanceGroup ig;
                    ig.mesh = assets->acquireMesh(interiorMesh, "city:carcabin" + std::to_string(v));
                    cabinH = ig.mesh;
                    engine::RenderMaterial im;
                    im.albedo = Vec3(1, 1, 1);
                    im.metallic = 0.0f;
                    im.roughness = 0.9f;
                    ig.material = im;
                    ig.renderLayer = engine::LayerSim;
                    ig.drawClass = engine::DrawClass::SimBody;
                    world.add<InstanceGroup>(gi, ig);
                }
                carGlassGroups_.push_back(ge);
                carGlassOpaqueGroups_.push_back(go);
                carInteriorGroups_.push_back(gi);
                carGlassMesh_.push_back(glassH);   // for a commandeered car (city_vehicles.cpp): its own glass + cabin
                carCabinMesh_.push_back(cabinH);
                carSeeInto_.push_back(seeInto ? 1 : 0);
                carGlassOpacity_.push_back(glassOpacity);
                carOpenSeat_.push_back(openSeat ? 1 : 0);
                carDriverSeat_.push_back(driverSeat);
                carRearHip_.push_back(rearHip);
                carHasRear_.push_back(hasRear ? 1 : 0);
                carHasDriver_.push_back(hasDriver ? 1 : 0);
            }

            Entity e = world.create();
            InstanceGroup g;
            g.mesh = mh;
            g.material = carMaterial();
            g.material.flags |= engine::RenderMaterial::FLAG_LAMP_STATE;   // its lenses light by the mask (ADR-0342)
            g.renderLayer = engine::LayerSim;   // debug layer toggle
            g.drawClass = engine::DrawClass::SimBody;
            // THE MID-RANGE FORM: past midFrom the cheaper whole car draws instead, and the near body, its glass and
            // its cabin stop there (InstanceGroup's per-instance distances)
            Entity me{};
            if (assets && !midMesh.vertices.empty() && midFrom > 0) {
                midTriangles += static_cast<int>(midMesh.indices.size() / 3);
                ++midCount;
                g.drawDistance = midFrom;
                for (Entity ne : {carGlassGroups_.back(), carGlassOpaqueGroups_.back(), carInteriorGroups_.back()})
                    if (InstanceGroup* ng = ne.valid() ? world.get<InstanceGroup>(ne) : nullptr) ng->drawDistance = midFrom;
                me = world.create();
                InstanceGroup mg = g;
                mg.mesh = assets->acquireMesh(midMesh, "city:carmid" + std::to_string(v));
                mg.drawDistance = 0;
                mg.minDistance = midFrom;
                world.add<InstanceGroup>(me, mg);
            }
            world.add<InstanceGroup>(e, g);
            carGroups_.push_back(e);
            sim_.poses().models.groups = static_cast<int>(carGroups_.size());
            carMidGroups_.push_back(me);
        }

        // PEOPLE ON THE BUS: seated figures posed from the same SAE manikin as
        // the player's driver, at a bus's seat height. Only built when some
        // slot publishes seats, so ordinary cities pay nothing.
        busDriverGroup_ = Entity{};
        bool anySeats = false;
        for (char hd : carHasDriver_) anySeats = anySeats || hd;
        if (assets && anySeats) {
            auto person = [&](Vec3 shirt, Vec3 trousers, Vec3 skin, Real arms,
                              const std::string& key) {
                engine::OccupantParams op;
                op.hipToHeel = 0.45;      // a bus seat, not a sedan's
                op.sittingHeight = 0.86;  // a bus has the headroom a sedan lacks
                op.backAngle = 12.0;
                op.armReach = arms;
                op.shirt = shirt;
                op.trousers = trousers;
                op.skin = skin;
                Entity e = world.create();
                InstanceGroup g;
                g.mesh = assets->acquireMesh(engine::buildSeatedOccupant(op), key);
                g.material = carMaterial();
                g.renderLayer = engine::LayerSim;
                g.drawClass = engine::DrawClass::SimBody;
                g.drawDistance = 160.0;   // a face in a window: gone by a block
                world.add<InstanceGroup>(e, g);
                return e;
            };
            // The driver in uniform, hands on the wheel: a LIGHT shirt, because
            // navy on the navy moquette vanished into the seat from the kerb.
            busDriverGroup_ = person(Vec3(0.66, 0.76, 0.90), Vec3(0.10, 0.12, 0.20),
                                     Vec3(0.72, 0.54, 0.42), 72.0, "city:busdriver");
        }
        // RIDERS ASTRIDE (a motorcycle, a moped): a slot whose seat has bars draws its own rider, posed by the
        // player's seat rule (seatedPose: hands on the grips, feet on the pegs, the seat's lean) -- the same body the
        // player rides in, so an NPC bike and the one you drop agree. Nothing to see through: always drawn on a trip.
        carRiderGroups_.assign(carGroups_.size(), Entity{});
        for (std::size_t v = 0; assets && v < carGroups_.size() && v < carSeatGeom_.size(); ++v) {
            if (carSeatGeom_[v].wheel || v >= carHasDriver_.size() || !carHasDriver_[v]) continue;
            InstanceGroup g;
            g.mesh = assets->acquireMesh(engine::buildPosedPerson(engine::seatedPose(carSeatGeom_[v], {}, 0, 0, 1),
                                                                  static_cast<int>(v)),
                                         "city:rider" + std::to_string(v));
            g.material = pedMaterial();
            g.renderLayer = engine::LayerSim;
            g.drawClass = engine::DrawClass::SimBody;
            carRiderGroups_[v] = world.create();
            world.add<InstanceGroup>(carRiderGroups_[v], g);
        }

        // FLEET VERDICT — loud, unmissable, once per build. "Why is the city
        // using the OLD cars?" was asked twice while terminal probes showed the
        // script loading fine; this line settles it from the user's own console.
        int scriptedCount = 0;
        for (const auto& lights : carLights_)
            if (!lights.empty()) ++scriptedCount;   // markers only from Lua
        if (params_.vehicleScript.empty())
            LOG_WARN << "[citysim] fleet: NO CARS — this level ships no vehicle "
                        "catalogue (set citysim.vehicles, or cars: 0 to mean it)";
        else if (scriptedCount == 0)
            LOG_ERROR << "[citysim] fleet: NO CARS — a vehicles script is "
                         "configured but produced 0 bodies";
        else
            LOG_INFO << "[citysim] fleet: " << scriptedCount << "/"
                     << fleetVariants
                     << " scripted car bodies (vehicles.lua), "
                     << fleetTriangles / std::max(1, scriptedCount)
                     << " tris/car avg; " << midCount << " mid-range bodies, "
                     << midTriangles / std::max(1, midCount) << " tris avg";
    }
    pedGroup_ = world.create();
    { InstanceGroup g; g.mesh = pedMesh; g.material = pedMaterial();
      g.renderLayer = engine::LayerSim; g.drawClass = engine::DrawClass::SimBody;
      world.add<InstanceGroup>(pedGroup_, g); }
    // the mid-range band's stand-ins: the box person (RT_NO_FAR_BAND=1: none). (Its two-box cars, one per body slot,
    // stood in for the old driving's far cars: gone with it, ADR-0461.)
    farPedGroup_ = Entity{};
    if (assets && !std::getenv("RT_NO_FAR_BAND")) {
        farPedGroup_ = world.create();
        { InstanceGroup g; g.mesh = pedMesh; g.material = pedMaterial();
          g.renderLayer = engine::LayerSim; g.drawClass = engine::DrawClass::SimBody;
          world.add<InstanceGroup>(farPedGroup_, g); }
    }
    if (assets) {
        pedSeatedGroup_ = world.create();
        InstanceGroup g; g.mesh = assets->acquireMesh(buildSeatedPersonMesh(0), "city:ped_seated"); g.material = pedMaterial();
        g.renderLayer = engine::LayerSim; g.drawClass = engine::DrawClass::SimBody;
        world.add<InstanceGroup>(pedSeatedGroup_, g);
        indoorSitGroup_ = world.create();
        world.add<InstanceGroup>(indoorSitGroup_, g);
        pedGroundGroup_ = world.create();
        InstanceGroup gs = g;
        gs.mesh = assets->acquireMesh(buildGroundSittingPersonMesh(0), "city:ped_ground_sit");
        world.add<InstanceGroup>(pedGroundGroup_, gs);
        for (int v = 0; v < engine::kLiePoses; ++v) {
            pedLieGroups_[static_cast<std::size_t>(v)] = world.create();
            InstanceGroup gl = g;
            gl.mesh = assets->acquireMesh(buildLyingPersonMesh(0, v), "city:ped_lie" + std::to_string(v));
            world.add<InstanceGroup>(pedLieGroups_[static_cast<std::size_t>(v)], gl);
        }
        for (int k = 0; k < 2; ++k) {   // THE BOARDS under riders in the crowd and the far band
            crowdBoardGroups_[k] = world.create();
            InstanceGroup bg = g;
            bg.mesh = assets->acquireMesh(k == 0 ? engine::buildSkateboard() : engine::buildScooter(),
                                          k == 0 ? "city:skateboard" : "city:scooter");
            bg.material.albedo = Vec3(1, 1, 1);
            world.add<InstanceGroup>(crowdBoardGroups_[k], bg);
        }
        engine::FurnitureDrawCache beachDraw;
        for (uint32_t c = 0; c < 8; ++c)
            for (auto [pc, groups] : {std::pair<engine::Piece, std::vector<Entity>*>{engine::Piece::BeachTowel, &towelGroups_[c]},
                                      {engine::Piece::BeachUmbrella, &umbrellaGroups_[c]}}) {
                engine::PlacedPiece pp;
                pp.piece = static_cast<uint8_t>(pc);
                pp.variant = c;
                engine::spawnFurnitureGroups(world, *assets, nullptr, beachDraw, {pp}, Vec3(0.80, 0.78, 0.75), 250.0, *groups);
                for (Entity e : *groups) world.get<InstanceGroup>(e)->transforms.clear();
            }
        indoorBodyGroup_ = world.create();
        InstanceGroup b = g;
        b.mesh = pedMesh;
        world.add<InstanceGroup>(indoorBodyGroup_, b);
    }
    for (int s = 0; s < kLensColourCount; ++s) {
        signalGroups_[s] = world.create();
        InstanceGroup g;
        g.mesh = lensMesh;
        g.material = signalMaterial(static_cast<LensColour>(s));
        g.renderLayer = engine::LayerSim;   // debug layer toggle
        g.drawClass = engine::DrawClass::Effect;
        world.add<InstanceGroup>(signalGroups_[s], g);
    }

    // THE SIGNAL POLES ARE THE CITY'S (device: "place the stop lights when we build the city ... the simulation
    // should use it"): the loader's StreetFurniture plan, over the SAME street map, stands one pole and head per arm
    // of every junction whose stated control the poles stand for (the road model's: junctionLit, ADR-0421), each head
    // attached to its arm. This system adopts the sites and the already-spawned post group and lights the heads from
    // the new driving's lamps (ADR-0456); it places no pole of its own (it once did, by the old signal clock's
    // links, where the loader had placed none).
    siteByLink_.clear();
    siteHasLink_.clear();
    signalHeads_.clear();
    headsModel_ = nullptr;
    headJunctions_.clear();
    engine::Entity adoptedPosts{};
    world.each<engine::StreetFurniture>([&](Entity, engine::StreetFurniture& f) {
        // (the STREETS' links: the walks appended since are not the loader's. Comparing the whole count, every
        // level with footpaths stopped adopting the placed poles and drew a second set of its own beside them --
        // every signal on the island doubled, staggered. Glenn: "multiple traffic lights next to one another")
        if (f.navLinkCount != nav_.streetLinkCount() || f.signalPoles.empty()) {
            if (!f.signalPoles.empty())
                LOG_WARN << "[citysim] the placed signal poles were built on a different street graph (" << f.navLinkCount
                         << " links, the sim has " << nav_.streetLinkCount() << "): no signals drawn";
            return;
        }
        siteByLink_.assign(nav_.linkCount(), SignalSite{});
        siteHasLink_.assign(nav_.linkCount(), 0);
        for (const engine::StreetFurniture::Signal& s : f.signalPoles) {
            if (s.link < 0 || s.link >= nav_.linkCount()) continue;
            SignalSite st;
            st.base = s.base;
            st.face = s.face;
            st.side = Vec3(s.face.z, 0, -s.face.x);   // rightOf(face): road centre
            st.yaw = std::atan2(s.face.x, s.face.z);
            siteByLink_[s.link] = st;
            siteHasLink_[s.link] = 1;
            // the loader's post instances, in its order (a shared pole has none): one head each
            if (s.shared) continue;
            SignalHead h;
            h.link = s.link;
            h.city = s.city;
            h.junction = s.junction;
            h.arm = s.arm;
            // its arrow lens beside the head, on the side its turn goes: the far turn's toward the road's middle
            // (`side`: traffic keeps left, so the turn across the oncoming stream turns that way), the near turn's
            // the other
            const Mat4 lens = signalLensPose(st);
            h.arrowLens[0] = Mat4::trs(st.side * kArrowLensOffset, Quat(), Vec3(1, 1, 1)) * lens;
            h.arrowLens[1] = Mat4::trs(st.side * -kArrowLensOffset, Quat(), Vec3(1, 1, 1)) * lens;
            signalHeads_.push_back(h);
        }
        adoptedPosts = f.postGroup;
    });

    if (adoptedPosts.valid()) {
        signalPostGroup_ = adoptedPosts;   // loader owns the geometry
    } else {
        signalPostGroup_ = world.create();   // none placed: an empty group (nothing stands, nothing lights)
        InstanceGroup g;
        g.material = signalPostMaterial();
        g.renderLayer = engine::LayerSim;
        g.drawClass = engine::DrawClass::Furniture;
        world.add<InstanceGroup>(signalPostGroup_, g);
    }
    // ONE MODEL, ITS STATE A NUMBER: the head's own lamps light by the state each instance carries (mesh.vert,
    // FLAG_SIGNAL_STATE) -- no lens geometry rebuilt or pushed per signal per bake
    if (InstanceGroup* pg = world.get<InstanceGroup>(signalPostGroup_)) {
        pg->material.flags |= engine::RenderMaterial::FLAG_SIGNAL_STATE;
        if (pg->transforms.size() != signalHeads_.size()) {
            LOG_WARN << "[citysim] signal posts " << pg->transforms.size() << " vs " << signalHeads_.size()
                     << " heads: lamps unlit";
            signalHeads_.clear();
        }
    }

    // Curbside bay markings (R6b): one white outline instanced per bay —
    // two end ticks + the outer edge line of a 6.24 m parallel bay, riding just
    // above the asphalt. Local +Z = along the bay (like cars).
    //
    // The mesh is authored in BAND-WIDTH UNITS across (x in [-0.5, +0.5]) and the
    // instance scales x by the bay's Parking-band width, so the paint always ends
    // exactly where the band does. It used to be a fixed 2.2 m box hung 1.05 m
    // inside a hardcoded kerb line, whose outer edge lapped ~7 cm onto the
    // sidewalk — the "half on the sidewalk" Glenn saw.
    {
        engine::RenderMesh bay;
        const Vec3 white(0.85, 0.85, 0.85);
        auto stripe = [&](Real x0, Real z0, Real x1, Real z1) {
            MeshBuilder::emitQuad(bay, Vec3(x0, 0, z0), Vec3(x1, 0, z0),
                                  Vec3(x1, 0, z1), Vec3(x0, 0, z1),
                                  Vec3(0, 1, 0), white);
        };
        stripe(-0.49, 3.0, 0.49, 3.12);     // front tick
        stripe(-0.49, -3.12, 0.49, -3.0);   // back tick
        stripe(0.43, -3.12, 0.49, 3.12);    // outer (curb-side) edge line
        MeshHandle bayMesh{};
        if (assets) bayMesh = assets->acquireMesh(bay, "city:parkbay");
        parkBayGroup_ = world.create();
        InstanceGroup g;
        g.mesh = bayMesh;
        g.material.albedo = Vec3(1, 1, 1);
        g.material.roughness = 0.92f;
        g.renderLayer = 0;
        // Thousands of separate instances, so the per-instance distance cull in
        // RenderSystem actually bites here.
        g.drawClass = engine::DrawClass::GroundPaint;
        // WHICH SPACES ARE PAINTED: the ones this level's cars park in (ADR-0439): where the driving drives, THE ROAD
        // MODEL'S kerb spaces (RoadModel::kerbSpacesOf: stated once, asked here), each outline where a car in the space
        // stands (kerbSpacePose). Where it does not, none (the old driving's own bays were painted there until ADR-0461).
        const engine::RoadModel* spacesOf =
            sim_.drivingHost().active() && sim_.drivingV2() ? sim_.drivingV2()->model() : nullptr;
        // The outline mesh above is drawn for a space this long (its ticks 3.12 m either side of the middle); a space of
        // another length stretches it, so the ticks keep their place in the space.
        constexpr Real kOutlineDrawnFor = 7.0;
        if (spacesOf) {
            for (std::size_t s = 0; s < spacesOf->streets.size(); ++s)
                for (const engine::KerbSpace& space : spacesOf->kerbSpacesOf(engine::ModelStreetId(static_cast<uint32_t>(s)))) {
                    engine::KerbSpacePose pose;
                    if (!spacesOf->kerbSpacePose(space, pose)) continue;
                    // (the mesh's +x is the kerb side of a car facing +z, as for a bay: the outer line lies along the kerb)
                    const Real y = groundAt(pose.at.x, pose.at.y) + engine::kRoadMarkLift;
                    g.transforms.push_back(Mat4::trs(Vec3(pose.at.x, y, pose.at.y),
                                                     Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(pose.facing.x, pose.facing.y)),
                                                     Vec3(space.width, 1, space.length / kOutlineDrawnFor)));
                }
        }
        refreshBounds(&g);
        world.add<InstanceGroup>(parkBayGroup_, g);
    }

    // (Stop bars, crosswalks and lane arrows are the lanes builder's, painted into its deck (ADR-0108); this bridge
    // drew a second set of floating quads from the graph, angles guessed, until ADR-0293.)

    // Hand the sim the pole foot positions (XZ) as static obstacles, so pedestrians
    // on the sidewalk steer around the signal poles and never stand inside one.
    {
        // (the placed poles, by their links in link order; a shared pole once per link it serves, as before)
        std::vector<engine::Vec2> poles;
        for (std::size_t li = 0; li < siteHasLink_.size(); ++li) {
            if (!siteHasLink_[li]) continue;
            const engine::Vec3 base = siteByLink_[li].base;
            poles.push_back(engine::Vec2(base.x, base.z));
        }
        const std::size_t poleCount = poles.size();
        sim_.setStaticObstacles(std::move(poles));
        // ...and the street's FURNITURE: everything placed outdoors and drawn as furniture -- a bus stop's pole and
        // bench, lamps, signs, benches, a cafe's tables (Glenn: "the npcs probably need to recognize vehicles and
        // furniture as obstacles to avoid"). Each as a point with a body's clearance: the lean steers round it, the
        // push-out keeps a walker from standing in it. (Interiors stream in later and are not here: nobody walks the
        // pavement inside.)
        std::vector<engine::Vec2> furniture;
        world.each<engine::InstanceGroup>([&](Entity, engine::InstanceGroup& g) {
            if (g.drawClass != engine::DrawClass::Furniture) return;
            // (a MERGED mesh -- the street-name blades, the shop signs -- is one identity instance: not a thing on
            // the pavement at the world's origin)
            if (g.transforms.size() == 1 && std::fabs(g.transforms[0].m[0][3]) + std::fabs(g.transforms[0].m[2][3]) < 1e-6) return;
            for (const Mat4& m : g.transforms) furniture.push_back(engine::Vec2(m.m[0][3], m.m[2][3]));
        });
        world.each<engine::Transform, engine::Renderable>([&](Entity, engine::Transform& t, engine::Renderable& r) {
            if (r.drawClass == engine::DrawClass::Furniture) furniture.push_back(engine::Vec2(t.position.x, t.position.z));
        });
        sim_.addStaticObstacles(furniture, 0.75);
        LOG_INFO << "[citysim] walkers steer round " << sim_.staticObstacleCount() << " static obstacles ("
                 << poleCount << " signal poles, " << furniture.size() << " pieces of street furniture)";
    }

    // Crosswalks are painted into the ROAD TEXTURE now (ADR-0062): the road mesher
    // bakes a set-back "metres past the junction mouth" coordinate into the
    // carriageway UV, and the RoadMarkings shader stripes the zebra band there — so
    // it's part of the road surface, set back on the approach, and never a floating
    // decal overlapping the centreline. The decal group stays empty. (Where a crosswalk IS is the road model's,
    // ModelCrosswalk, ADR-0452: the guessed "crosswalk centres" once kept here, read by nobody, are gone.)
    crosswalkGroup_ = world.create();
    world.add<InstanceGroup>(crosswalkGroup_, InstanceGroup{});   // empty: road paints the bars

    // Debug widgets (ADR-0061): per-agent footprint ring (one group per behaviour
    // state, coloured) + a forward trajectory arrow. The groups always exist so a
    // key can toggle them at runtime; they stay empty unless debugWidgets_ is on.
    {
        MeshHandle ringMesh{}, arrowMesh{}, stripMesh{};
        MeshHandle wedgeMesh{};
        if (assets) {
            ringMesh = assets->acquireMesh(ringXZ(), "city:dbgring");
            arrowMesh = assets->acquireMesh(arrowXZ(), "city:dbgarrow");
            stripMesh = assets->acquireMesh(stripXZ(), "city:dbglane");
            // The half-angle is baked into the wedge; the instance scale carries only the range.
            wedgeMesh = assets->acquireMesh(wedgeXZ(kPedConeHalfAngle), "city:dbgwedgeped");
        }
        const int stateCount = static_cast<int>(Agent::State::Count);
        for (int s = 0; s < stateCount; ++s) {
            footprintGroups_[s] = world.create();
            InstanceGroup g;
            g.mesh = ringMesh;
            g.material = widgetMaterial(stateColor(static_cast<Agent::State>(s)));
            world.add<InstanceGroup>(footprintGroups_[s], g);
        }
        forwardGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = arrowMesh;
            g.material = widgetMaterial(Vec3(0.15, 0.85, 0.95));   // cyan trajectory
            world.add<InstanceGroup>(forwardGroup_, g);
        }
        visionGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = wedgeMesh;
            g.material = widgetMaterial(kConeTint);
            world.add<InstanceGroup>(visionGroup_, g);
        }
        navLinkGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = stripMesh;
            g.material = widgetMaterial(kNavTint);
            world.add<InstanceGroup>(navLinkGroup_, g);
        }
        navNodeGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = ringMesh;
            g.material = widgetMaterial(kNavTint);
            world.add<InstanceGroup>(navNodeGroup_, g);
        }
        // City-plan outlines (ADR-0066): BLOCKS in magenta, LOTS in amber — the
        // "blocks → lots → buildings" story drawn on the ground, same painted-
        // outline style as the rest of the widgets.
        blockGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = stripMesh;
            g.material = widgetMaterial(Vec3(0.85, 0.30, 0.85));
            world.add<InstanceGroup>(blockGroup_, g);
        }
        lotGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = stripMesh;
            g.material = widgetMaterial(Vec3(0.95, 0.80, 0.25));
            world.add<InstanceGroup>(lotGroup_, g);
        }
        // Collider prisms (device: "a physics hull visualizer"): each building's
        // plan-prism collider drawn as its EXACT volume — a rim loop at the
        // prism base and top (ground strips at explicit heights) plus vertical
        // corner posts. Hot orange so a missing/short prism reads instantly.
        colliderStripGroup_ = world.create();
        {
            InstanceGroup g;
            g.mesh = stripMesh;
            g.material = widgetMaterial(Vec3(1.0, 0.45, 0.10));
            world.add<InstanceGroup>(colliderStripGroup_, g);
        }
        colliderPostGroup_ = world.create();
        {
            InstanceGroup g;
            if (assets) g.mesh = assets->acquirePrimitive("box", Vec3(1, 1, 1));
            g.material = widgetMaterial(Vec3(1.0, 0.45, 0.10));
            world.add<InstanceGroup>(colliderPostGroup_, g);
        }
    }

    // Bake the navgraph view ONCE — it depends only on nav_, so unlike the
    // per-agent widgets it never rebakes per frame. syncGroups copies the cached
    // transforms in (HUD on) or leaves the groups empty (HUD off). The bake uses
    // no sim RNG, so determinism is untouched.
    navLinkBake_.clear();
    navNodeBake_.clear();
    for (int li = 0; li < nav_.linkCount(); ++li) {
        const engine::NavLink& L = nav_.links[li];
        int lanes = L.lanes < 1 ? 1 : L.lanes;
        // ASK the sim for the spacing rather than re-deriving it: on a street
        // with a Parking band the drivable width is the carriageway minus the
        // two parked strips, so a re-derived "half the carriageway" strip drew
        // the debug lanes under the parked cars instead of under the traffic.
        Real spacing = sim_.laneSpacingFor(li);
        for (int lane = 0; lane < lanes; ++lane) {
            Vec2 a = nav_.laneCenter(li, lane, 0.0, spacing);
            Vec2 b = nav_.laneCenter(li, lane, 1.0, spacing);
            Vec2 d(b.x - a.x, b.y - a.y);
            Real len = std::sqrt(d.x * d.x + d.y * d.y);
            if (len < 1e-6) continue;
            Real y = groundAt(a.x, a.y) + L.layer * kLayerClearance + 0.04;
            Real yaw = std::atan2(d.x, d.y);
            navLinkBake_.push_back(Mat4::trs(
                Vec3(a.x, y, a.y), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                Vec3(0.15, 1, len)));   // a thin lane-centreline strip
        }
    }
    for (int n = 0; n < nav_.nodeCount(); ++n) {
        if (!nav_.isJunction(n)) continue;
        Vec2 p = nav_.nodes[n];
        navNodeBake_.push_back(Mat4::trs(
            Vec3(p.x, groundAt(p.x, p.y) + 0.04, p.y), Quat(), Vec3(1.2, 1, 1.2)));
    }

    // The CITY-PLAN outlines bake the first time they are shown (bakePlanOutlines): a
    // debug view that is off by default should not hold ~40 MB of GPU mesh (ADR-0096).
    assets_ = assets;
    planBaked_ = false;
    blockBake_.clear();
    lotBake_.clear();

    // Bake the COLLIDER-PRISM outlines once (device: "a physics hull
    // visualizer"): the exact Jolt volumes — a rim loop at each prism's world
    // base and top plus vertical corner posts. Heights are ABSOLUTE (they're
    // the collider's own), not ground-sampled.
    colliderStripBake_.clear();
    colliderPostBake_.clear();
    {
        auto rimLoop = [&](const engine::Poly2& poly, Real y) {
            const std::size_t n = poly.size();
            for (std::size_t i = 0; i < n; ++i) {
                Vec2 a = poly[i], b = poly[(i + 1) % n];
                Vec2 d(b.x - a.x, b.y - a.y);
                Real len = std::sqrt(d.x * d.x + d.y * d.y);
                if (len < 1e-6) continue;
                Real yaw = std::atan2(d.x, d.y);
                colliderStripBake_.push_back(Mat4::trs(
                    Vec3(a.x, y, a.y), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                    Vec3(0.14, 1, len)));
            }
        };
        world.each<engine::CityPlanDebug>([&](Entity, engine::CityPlanDebug& plan) {
            for (const engine::CityPlanDebug::Prism& pr : plan.prisms) {
                if (pr.plan.size() < 3) continue;
                rimLoop(pr.plan, pr.y0);
                rimLoop(pr.plan, pr.y1);
                const Real h = pr.y1 - pr.y0;
                if (h <= 0) continue;
                for (const Vec2& v : pr.plan)
                    colliderPostBake_.push_back(Mat4::trs(
                        Vec3(v.x, pr.y0 + h * 0.5, v.y), Quat(),
                        Vec3(0.12, h, 0.12)));
            }
        });
    }

    // Per-agent speed history for the brake-light hard-decel test (ADR-0065
    // follow-up); seeded to the warmed-up speed so the first bake sees no spurious
    // deceleration.
    drawCache_.clear();   // (an agent's speed history seeds on its first look: AgentDrawCache::fresh)

    // THE MARKERS AND THE RESIDENT WITH THE AGENT BRAIN (ADR-0441; city_brain.h), on a level that states any and that
    // the new driving drives. Last, once the city is seeded: seeding puts every walker where its day has it, and the
    // resident is put at their home marker after that.
    // A CROWD MADE FROM THE ROADS (ADR-0446; city_crowd.h), added to what the level states
    std::vector<engine::CityMarker> markers = params_.markers;
    std::vector<engine::CityResident> residents = params_.residents;
    std::vector<engine::CityHousehold> households = params_.households;
#ifdef RT_ENABLE_SCRIPTING
    if (!params_.crowd.empty() && sim_.drivingV2() && sim_.drivingV2()->model()) {
        const std::string code = engine::loadScriptCode(params_.crowd, "");
        engine::ScriptVM vm;
        std::string err;
        CrowdSpec spec;
        CrowdMade made;
        // WHERE PEOPLE CAN WALK TO AND FROM: a place whose stand joins the walks (CitySim::walkJoin, the very join a
        // walk from or to it starts and ends by: the brain sends its people to their places' stands) in the largest
        // part of them a walker can get round. (A place on a walk cut off from the rest, a sidewalk ending short of its
        // junction or a sliver between two roads, is one its people could walk nowhere from.) Was the street map's
        // walking network, which walked nobody after #306 piece 6 W3 (ADR-0480).
        // ...and, for a home, where the households' cars can be parked: the kerb spaces the brain offers (the one list,
        // kerbSpacesOffered), kept to those beside lanes in the largest part of the road every vehicle class drives
        // (driving/trip_places.h; the brain, with the level's drivableSpacesOnly, keeps those of the car's own class,
        // so these are among them), at least kHomeSpaces of them for the comings and goings; and as many of them free
        // as its household can have cars, which the home then holds (makeCrowd's CrowdSpaces: the generator never
        // deals a household whose car the brain finds no free space for)
        // (12 at ADR-0446's 200 residents; at 350 a household's car found none of 12 free)
        constexpr int kHomeSpaces = 24;
        const driving::Driving& d2 = *sim_.drivingV2();
        std::vector<driving::TripPlaces> classParts;
        std::vector<uint32_t> classMain;
        for (uint32_t cls = 0; cls < d2.vehicleClasses().size(); ++cls) {
            classParts.push_back(driving::findTripPlaces(*d2.roads(), d2.routeClass(cls), kKerbLaneEndMargin, kSmallestTripPart));
            uint32_t main = driving::kNoTripPart;
            Real most = 0;
            for (uint32_t p = 0; p < classParts.back().parts.size(); ++p)
                if (classParts.back().parts[p].total > most) {
                    most = classParts.back().parts[p].total;
                    main = p;
                }
            classMain.push_back(main);
        }
        const engine::RoadModel& model = *d2.model();
        const auto everyClass = [&](const engine::KerbSpace& space) {
            const engine::KerbLanePlace lane = model.kerbLaneBeside(space);
            bool every = lane.valid();
            for (std::size_t cls = 0; cls < classParts.size() && every; ++cls)
                every = classParts[cls].partOf(lane.path) == classMain[cls];
            return every;
        };
        const CrowdSpaces carSpaces = [&](const engine::RoadFrontage& f) { return kerbSpacesOffered(model, f, everyClass); };
        const CrowdUsable onFoot = [&](Vec2 p, const engine::RoadFrontage& f, bool cars) {
            engine::WalkPoint joined;
            bool cutOff = true;
            if (!sim_.walkJoin(p, joined, &cutOff) || cutOff) return false;
            return !cars || static_cast<int>(carSpaces(f).size()) >= kHomeSpaces;
        };
        if (code.empty() || !vm.doString(code, &err) || !engine::loadCrowdSpec(vm, params_.crowd, spec, &err) ||
            !makeCrowd(spec, *sim_.drivingV2()->model(), onFoot, carSpaces, params_.seed, made, &err)) {
            LOG_WARN << "[citysim] crowd " << params_.crowd << ": " << (code.empty() ? std::string("not found under assets/scripts") : err);
        } else {
            std::string kinds;
            for (const std::pair<std::string, int>& k : made.placesOfKind) kinds += " " + std::to_string(k.second) + " " + k.first;
            LOG_INFO << "[citysim] crowd " << params_.crowd << ": " << made.residents.size() << " residents in "
                     << made.households.size() << " households, " << made.cars << " cars; places:" << kinds;
            markers.insert(markers.end(), made.markers.begin(), made.markers.end());
            residents.insert(residents.end(), made.residents.begin(), made.residents.end());
            households.insert(households.end(), made.households.begin(), made.households.end());
        }
    }
#endif
    if ((!markers.empty() || !residents.empty()) && sim_.drivingV2() && sim_.drivingHost().active()) {
        std::string brainReport;
        if (sim_.startBrain(world, markers, residents, households, drivingClassOfSlot, &brainReport, params_.drivableSpacesOnly))
            LOG_INFO << "[citysim] agent brain: " << brainReport;
        else
            LOG_WARN << "[citysim] agent brain: no markers and no resident on this level: " << brainReport;
    }

    built_ = true;
    syncGroups(world);
    return true;
}

// (The four-wheel fit and its tilt low-pass are the sim's: CityPoses::bodyOnWheels, smoothUp, city_poses.cpp.)

Mat4 CityRenderSystem::agentPose(const Agent& a) const {
    // A WALKER is the sim's walker pose (city_poses.h): the drawing calls it. (Every agent is one since ADR-0461: the
    // cars are entities, drawn by syncCarEntities where their CityCarPose is.)
    return sim_.poses().walkerPose(sim_, a, params_.pedSize.y);
}

std::vector<Vec3> CityRenderSystem::carGroupHalfExtents() const { return sim_.poses().halfExtents(sim_); }

Mat4 CityRenderSystem::signalLensPose(const SignalSite& st) {
    // Level with the green lamp of the three-lamp head (mirror of street_kit::emitTrafficSignal): the head hangs at
    // the arm end (along `side`), red on top / amber / green on the bottom, lenses on the facing side.
    engine::SignalParams sp;
    Vec3 headTop = st.base + Vec3(0, sp.armHeight - 0.1, 0) + st.side * (sp.armLength - 0.2);
    Vec3 headCenter = headTop + Vec3(0, -0.55, 0);
    constexpr Real kGreenSlot = -0.42;
    // the head's lamp discs stand 0.22 m out and are 0.06 thick (street_kit::emitTrafficSignal): the plate's back
    // face 1 cm in front of them, turned the way the head faces
    Vec3 p = headCenter + Vec3(0, kGreenSlot, 0) + st.face * 0.27;
    return Mat4::trs(p, Quat::fromAxisAngle(Vec3(0, 1, 0), st.yaw), Vec3(1, 1, 1));
}

// THE NUMBER A POST INSTANCE CARRIES FOR ITS HEAD'S FACE (mesh.vert FLAG_SIGNAL_STATE: 0 none, 1 green, 2 amber,
// 3 red), from what the new driving's lamps show there (driving::headAt).
int CityRenderSystem::headFaceNumber(driving::Lamp face) {
    switch (face) {
    case driving::Lamp::Green:
    case driving::Lamp::GreenArrow: return 1;
    case driving::Lamp::Yellow: return 2;
    case driving::Lamp::Red:
    case driving::Lamp::RedMayTurn: return 3;
    case driving::Lamp::Dark: return 0;
    }
    return 0;
}

driving::HeadShows CityRenderSystem::signalHeadShows(std::size_t head) const {
    const driving::Driving* d = sim_.drivingHost().active() ? sim_.drivingV2() : nullptr;
    if (!d || !d->lightsRun() || !d->roads() || d->model() != headsModel_ || head >= headJunctions_.size())
        return driving::HeadShows{};
    return driving::headAt(*d->roads(), d->rightOfWay(), headJunctions_[head],
                           static_cast<uint32_t>(signalHeads_[head].arm), d->clockSeconds());
}

void CityRenderSystem::tieSignalHeads(World& world) {
    // Where the new driving drives (its model is one city of the level's road model), each head attached to an arm
    // of that city's junctions shows the lamps of the lane level's junction there; elsewhere it is dark (ADR-0456).
    const driving::Driving* d = sim_.drivingHost().active() ? sim_.drivingV2() : nullptr;
    const engine::RoadModel* model = d ? d->model() : nullptr;
    if (model == headsModel_ && headJunctions_.size() == signalHeads_.size()) return;
    headsModel_ = model;
    headJunctions_.assign(signalHeads_.size(), engine::JunctionId());
    if (!model) return;
    int city = -1;
    world.each<engine::LevelRoadModel>([&](Entity, engine::LevelRoadModel& rm) {
        for (std::size_t c = 0; c < rm.cities.size() && city < 0; ++c)
            if (rm.cities[c].get() == model) city = static_cast<int>(c);
    });
    if (city < 0) return;
    int tied = 0;
    for (std::size_t k = 0; k < signalHeads_.size(); ++k) {
        const SignalHead& h = signalHeads_[k];
        if (h.city != city || h.junction < 0 || h.arm < 0) continue;
        headJunctions_[k] = model->laneJunctionOf(engine::ModelJunctionId(static_cast<uint32_t>(h.junction)));
        tied += headJunctions_[k].valid() ? 1 : 0;
    }
    LOG_INFO << "[citysim] signal heads lit by the new driving's lamps: " << tied << " of " << signalHeads_.size();
}

// Bake the emissive car lamps (ADR-0065 follow-up). Same shape as the signal
// lenses: clear each per-kind group, then for every DRAWN car push a small
// emissive box at each lit lamp marker's WORLD pose. Only the lit lamps are
// pushed, so a phase change (headlights on at dusk, brakes on braking) just moves
// transforms between batches — cheap, a handful of quads per car, rebuilt per step.
// THE LAMPS ARE THE BODY'S (ADR-0342, #186): each drawn car's lit lamps as a mask on its body instance (the bottom
// row, FLAG_LAMP_STATE), lighting the lenses the body was built with. They were emissive boxes pushed at the lamp
// markers in their own groups -- a second thing to keep in step with the body, and when it was not, headlights hung
// in the air over a parked car (ADR-0341) or glowed inside a bus.
void CityRenderSystem::syncCarLamps(World& world) {
    trafficLights_.clear();
    static const int headKind = engine::lightKindId("headlight");
    static const int tailKind = engine::lightKindId("tail_light");
    static const int brakeKind = engine::lightKindId("brake_light");

    const bool drawCars = !carGroups_.empty();

    // Turn signals blink off the SIM clock — deterministic (~1.5 Hz, 50% duty), so
    // the bake is reproducible and headless-testable. A render-time wall clock
    // would be acceptable too (a non-deterministic blink is fine per the ADR), but
    // the sim clock keeps determinism intact.
    const bool blinkOn = engine::lampBlinkOn(sim_.seconds());
    std::unordered_map<int, uint32_t> masks;   // the lit lamps of each drawn car, by its key in carAgentIds_
    // Darkness from the SUN where a render view has staged one (the
    // viewer), else from the sim's own clock (headless/offline). The two
    // clocks are NOT synced, which is why traffic used to keep its
    // headlights off all night while the player's car had them on.
    const bool dark = solarStaged_
                          ? solarElevation_ < engine::kLampDuskSunY
                          : clockIsDark(sim_.timeOfDay());
    // A CAR'S LAMPS AS LIGHTS (ADR-0362): a beam from each headlight marker of variant `v`'s body, a red glow behind
    // each tail lamp (brighter braking), the body drawn at `body`; LightSystem picks which cars are lit
    const auto lampLights = [&](const Mat4& body, int v, const engine::VehicleLamps& lamps, uint32_t group) {
        const Vec3 fwd = normalize(body.transformDirection(Vec3(0, 0, 1)));
        const Vec3 beamDir = normalize(fwd + Vec3(0, -0.14, 0));
        for (const LampMarker& lm : carLights_[static_cast<std::size_t>(v)]) {
            engine::LightCandidate c;
            c.group = group;
            if (lamps.head && lm.name.rfind("headlight", 0) == 0) {
                c.kind = headKind;
                c.position = body.transformPoint(lm.pos) + fwd * 0.25;
                c.direction = beamDir;
            } else if (lm.name.rfind("taillight", 0) == 0) {
                c.kind = lamps.brake ? brakeKind : tailKind;
                c.position = body.transformPoint(lm.pos) - fwd * 0.3;
            } else {
                continue;
            }
            trafficLights_.push_back(c);
        }
    };

    // THE CARS, which are entities (ADR-0444, ADR-0453): lit from the driving's own braking (its acceleration, or held
    // at rest; no indicators until its rules give them, task C8), while on a trip (a car at rest is dark (ADR-0341):
    // standing in its space, or left in a lane)
    if (drawCars && sim_.drivingV2()) {
        world.each<CityCarBody, CityCarLink, CityCarOnLane>([&](Entity e, CityCarBody& body, CityCarLink& link, CityCarOnLane& lane) {
            AgentDrawCache& dc = drawCache(entityCarKey(body.id));
            const Real prev = dc.fresh ? lane.speed : dc.prevSpeed;
            dc.prevSpeed = lane.speed;
            dc.fresh = false;
            const int v = parkedVariant(body.slot, body.type);
            const int key = entityCarDrawKey(body, link);
            const CityCarPose* pose = world.get<CityCarPose>(e);
            if (!pose || !lane.driven || v < 0 || v >= static_cast<int>(carLights_.size())) return;
            const engine::VehicleLamps lamps = engine::vehicleLampState(lane.braking ? engine::LampMotion::Holding : engine::LampMotion::Rolling,
                                                            lane.speed, prev, dark, 0);
            const uint32_t m = engine::lampMask(lamps, blinkOn);
            if (m) masks[key] = m;
            if (lamps.head || lamps.brake) lampLights(pose->body, v, lamps, 0x20000000u | body.id);
        });
    }
    // ...onto the body instances syncGroups drew this bake (carAgentIds_ runs beside each group's transforms)
    if (!masks.empty())
        for (std::size_t v = 0; v < carGroups_.size() && v < carAgentIds_.size(); ++v) {
            InstanceGroup* g = world.get<InstanceGroup>(carGroups_[v]);
            if (!g) continue;
            const std::vector<int>& ids = carAgentIds_[v];
            for (std::size_t k = 0; k < ids.size() && k < g->transforms.size(); ++k) {
                const auto it = masks.find(ids[k]);
                if (it != masks.end()) g->transforms[k].m[3][0] = static_cast<Real>(it->second);
            }
        }
    copyMidBodies(world);
}

// The mid-range groups draw the same cars as the near ones: their poses, ids (the renderer's interpolation) and lamp
// masks, copied after the bake is complete (the mask is written last, syncCarLamps).
void CityRenderSystem::copyMidBodies(World& world) {
    for (std::size_t v = 0; v < carMidGroups_.size() && v < carGroups_.size(); ++v) {
        if (!carMidGroups_[v].valid()) continue;
        const InstanceGroup* near = world.get<InstanceGroup>(carGroups_[v]);
        InstanceGroup* mid = world.get<InstanceGroup>(carMidGroups_[v]);
        if (!near || !mid) continue;
        mid->transforms = near->transforms;
        mid->ids = near->ids;
        mid->prevTransforms = near->prevTransforms;
        mid->prevIds = near->prevIds;
        mid->boundsCenter = near->boundsCenter;
        mid->boundsRadius = near->boundsRadius;
    }
}

std::size_t CityRenderSystem::carsLit(World& world, uint32_t bits) const {
    std::size_t n = 0;
    for (Entity e : carGroups_)
        if (const InstanceGroup* g = world.get<InstanceGroup>(e))
            for (const Mat4& t : g->transforms)
                if (static_cast<uint32_t>(t.m[3][0] + 0.5) & bits) ++n;
    return n;
}

// Bake the CITY-PLAN outlines (ADR-0066), on first show (syncGroups).
void CityRenderSystem::bakePlanOutlines(World& world) {
    // The CITY-PLAN outlines (ADR-0066): every block/lot polygon
    // (published by the loader as CityPlanDebug) is stroked as ONE CLOSED
    // RIBBON (device: "use the ribbon library ... form polygon shapes
    // properly") — a continuous mitred loop, not per-edge strips with corner
    // gaps. Draped at each polygon's local ground height (a block is small vs
    // the terrain, so one height reads flat). The group shows the merged mesh
    // via a single identity transform; the show/hide toggle stays the same.
    planBaked_ = true;
    engine::AssetManager* assets = assets_;
    if (assets) {
        engine::RenderMesh blockRib, lotRib;
        auto strokePoly = [&](const engine::Poly2& poly, double halfW,
                              double lift, engine::RenderMesh& into) {
            if (poly.size() < 3) return;
            std::vector<engine::Vec2> pts(poly.begin(), poly.end());
            const engine::Vec2 c = engine::centroid(poly);
            engine::MeshBuilder::append(
                into, engine::strokeRibbon(pts, {halfW}, groundAt(c.x, c.y) + lift,
                                           engine::Vec3(1, 1, 1), /*closed=*/true));
        };
        world.each<engine::CityPlanDebug>([&](Entity, engine::CityPlanDebug& plan) {
            for (const engine::Poly2& b : plan.blocks) strokePoly(b, 0.45, 0.06, blockRib);
            for (const engine::Poly2& l : plan.lots) strokePoly(l, 0.26, 0.05, lotRib);
        });
        if (!blockRib.vertices.empty()) {
            MeshHandle h = assets->acquireMesh(blockRib, "city:blockoutline");
            if (auto* g = world.get<InstanceGroup>(blockGroup_)) {
                g->mesh = h;
                g->boundsCenter = Vec3(0, 0, 0);
                g->boundsRadius = 6000.0;   // city-wide merged mesh: never cull
            }
            blockBake_ = {Mat4()};
        }
        if (!lotRib.vertices.empty()) {
            MeshHandle h = assets->acquireMesh(lotRib, "city:lotoutline");
            if (auto* g = world.get<InstanceGroup>(lotGroup_)) {
                g->mesh = h;
                g->boundsCenter = Vec3(0, 0, 0);
                g->boundsRadius = 6000.0;
            }
            lotBake_ = {Mat4()};
        }
    }

}

// THE BEACH'S THINGS (towelGroups_): a towel under each body lying out on the ground (its head end at the head, the
// way the body lies), and an umbrella over each group lying out on the sand, its pole beside the middle of their heads.
void CityRenderSystem::syncBeachThings(World& world) {
    std::array<std::vector<Mat4>, 8> towels, umbrellas;
    const std::vector<CitySim::Session>& sessions = sim_.sessions();
    const std::vector<CitySim::ActivityArea>& areas = sim_.areas();
    for (std::size_t si = 0; si < sessions.size(); ++si) {
        const CitySim::Session& se = sessions[si];
        if (se.state == CitySim::Session::State::Dead) continue;
        int lying = 0;
        for (int m : se.members) {
            CitySim::RestPose rp;
            if (!sim_.restPose(m, rp) || rp.kind != CitySim::RestPose::Kind::Lie || rp.raised) continue;
            ++lying;
            const Vec2 head = rp.pos - rp.face * 0.875;   // (the towel's z = 0 end)
            towels[(static_cast<uint32_t>(m) * 2654435761u) >> 29]   // (a colour each: the top bits of a Knuth hash)
               .push_back(
                Mat4::translate(head.x, groundAt(head.x, head.y) + 0.005, head.y) * Mat4::rotateY(std::atan2(rp.face.x, rp.face.y)));
        }
        if (lying == 0 || se.def < 0 || se.area < 0 || se.area >= static_cast<int>(areas.size()) ||
            areas[static_cast<std::size_t>(se.area)].kind != "beach")
            continue;
        // the pole 0.6 m up from the heads, between the middle two (an odd group: beside the middle one)
        const Vec2 side(se.axis.y, -se.axis.x);
        const Real gap = sim_.activityCatalog().defs[static_cast<std::size_t>(se.def)].radius;   // (memberPlace's spread)
        const Vec2 pole = se.center + side * ((se.members.size() % 2u) ? gap * 0.5 : 0.0) - se.axis * 0.3;
        const Vec2 origin = pole - se.axis * 1.1;   // (the umbrella's pole is 1.1 m along its +z)
        umbrellas[(static_cast<uint32_t>(si) * 2654435761u + 0x9E37u) >> 29].push_back(
            Mat4::translate(origin.x, groundAt(pole.x, pole.y), origin.y) * Mat4::rotateY(std::atan2(se.axis.x, se.axis.y)));
    }
    auto put = [&](const std::array<std::vector<Entity>, 8>& groups, const std::array<std::vector<Mat4>, 8>& xf) {
        for (std::size_t c = 0; c < 8; ++c)
            for (Entity e : groups[c])
                if (InstanceGroup* g = world.get<InstanceGroup>(e)) {
                    g->transforms = xf[c];
                    refreshBounds(g);
                }
    };
    put(towelGroups_, towels);
    put(umbrellaGroups_, umbrellas);
}

// (Where a car entity's body is each step, and a walker's, are the sim's: city_poses.h CityPoses. The drawing reads
// CityCarPose and calls the walker pose.)

// THE NEW DRIVING'S CARS ARE ENTITIES (ADR-0444, ADR-0453; city_car.h), drawn by the same means as the sim's own cars,
// into the same instanced groups, where their bodies are this step (CityCarPose). Its instance's name is its number,
// so the renderer draws it between bakes whether it is driven or parked; its key in carAgentIds_ is its driver while
// somebody sits at its wheel (the driver is drawn in the seat, the glass and cabin swapped near the player, as for any
// car with a driver), else entityCarKey.
void CityRenderSystem::syncCarEntities(World& world, std::vector<InstanceGroup*>& cars) {
    if (cars.empty()) return;
    world.each<CityCarBody, CityCarLink, CityCarPose>([&](Entity, CityCarBody& body, CityCarLink& link, CityCarPose& pose) {
        const int v = parkedVariant(body.slot, body.type);
        if (v < 0 || v >= static_cast<int>(cars.size()) || !cars[static_cast<std::size_t>(v)]) return;
        cars[static_cast<std::size_t>(v)]->push(pose.body, kCarEntityInstance | (static_cast<uint64_t>(body.id) << 8));
        carAgentIds_[static_cast<std::size_t>(v)].push_back(entityCarDrawKey(body, link));
    });
}

void CityRenderSystem::syncGroups(World& world) {
    std::vector<InstanceGroup*> cars;
    cars.reserve(carGroups_.size());
    for (Entity e : carGroups_) cars.push_back(world.get<InstanceGroup>(e));
    InstanceGroup* ped = world.get<InstanceGroup>(pedGroup_);
    InstanceGroup* pedSeated = pedSeatedGroup_.valid() ? world.get<InstanceGroup>(pedSeatedGroup_) : nullptr;
    if (pedSeated) pedSeated->transforms.clear();
    InstanceGroup* pedGround = pedGroundGroup_.valid() ? world.get<InstanceGroup>(pedGroundGroup_) : nullptr;
    std::array<InstanceGroup*, engine::kLiePoses> pedLie{};
    for (std::size_t v = 0; v < pedLie.size(); ++v) {
        pedLie[v] = pedLieGroups_[v].valid() ? world.get<InstanceGroup>(pedLieGroups_[v]) : nullptr;
        if (pedLie[v]) pedLie[v]->transforms.clear();
    }
    if (pedGround) pedGround->transforms.clear();
    InstanceGroup* boardsG[2];
    for (int k = 0; k < 2; ++k) {
        boardsG[k] = crowdBoardGroups_[k].valid() ? world.get<InstanceGroup>(crowdBoardGroups_[k]) : nullptr;
        if (boardsG[k]) boardsG[k]->transforms.clear();
    }
    // a rider's board, at its feet the way it heads (activities.lua `ride`)
    auto boardUnder = [&](int ai, Real x, Real feetY, Real z, Real yaw) {
        const Ride r = sim_.rideOf(ai);
        InstanceGroup* bg = r == Ride::Skateboard ? boardsG[0] : r == Ride::Scooter ? boardsG[1] : nullptr;
        if (bg) bg->transforms.push_back(Mat4::translate(x, feetY, z) * Mat4::rotateY(yaw));
        return CityPoses::rideDeck(r);   // the deck it stands on
    };
    InstanceGroup* sig[kLensColourCount];
    for (int s = 0; s < kLensColourCount; ++s) sig[s] = world.get<InstanceGroup>(signalGroups_[s]);

    // (moving: each instance named, so the renderer draws it between its last step and this one -- InstanceGroup::ids)
    for (InstanceGroup* c : cars) if (c) c->beginStep();
    if (ped) ped->beginStep();
    for (int s = 0; s < kLensColourCount; ++s) if (sig[s]) sig[s]->transforms.clear();

    carAgentIds_.assign(cars.size(), {});
    pedAgentIds_.assign(1, {});
    // RT_DUMP_STATS: where the sync goes, section by section (the island profile: 13 ms a bake)
    static const bool syncStats = std::getenv("RT_DUMP_STATS") != nullptr;
    static double secMs[7] = {0, 0, 0, 0, 0, 0, 0};
    static int secCalls = 0;
    auto secT = std::chrono::steady_clock::now();
    auto sec = [&](int k) {
        if (!syncStats) return;
        const auto n = std::chrono::steady_clock::now();
        secMs[k] += std::chrono::duration<double, std::milli>(n - secT).count();
        secT = n;
    };
    const auto& agents = sim_.agents();
    // the draw cache: a new bake; every 600, drop what has not been drawn for that long
    if (++drawBake_ % 600 == 0)
        for (auto it = drawCache_.begin(); it != drawCache_.end();)
            it = drawBake_ - it->second.used > 600 ? drawCache_.erase(it) : std::next(it);
    // the K tier only, ascending (agents() order): a far agent has no render membership
    for (int nearIdx : sim_.nearAgents()) {
        const std::size_t ai = static_cast<std::size_t>(nearIdx);
        const Agent& a = agents[ai];
        // P4: a far (V) agent has NO render membership — no instance, no lamp,
        // no kinematic proxy (the physics diff keys off these id lists).
        if (a.far()) continue;
        if (ped && !pedsExternallyOwned_) {   // walkers owned externally: no bake
            if (!sim_.pedVisible(static_cast<int>(ai))) continue;   // indoors / riding
            // HOW TALL (people.lua band heights by age): every pose scaled about its feet, or its hip on a seat
            const Real s = sim_.bodyScale(static_cast<int>(ai));
            const Mat4 sized = Mat4::scale(s, s, s);
            CitySim::RestPose rp;
            if (sim_.restPose(static_cast<int>(ai), rp)) {   // sitting (M5), on the grass, lying in the sun
                const Mat4 yaw = Mat4::rotateY(std::atan2(rp.face.x, rp.face.y));
                if (rp.kind == CitySim::RestPose::Kind::Seat) {
                    if (pedSeated) pedSeated->transforms.push_back(Mat4::translate(rp.pos.x, rp.hip, rp.pos.y) * yaw * sized);
                } else if (rp.kind == CitySim::RestPose::Kind::SitGround) {
                    if (pedGround)
                        pedGround->transforms.push_back(
                            Mat4::translate(rp.pos.x, groundAt(rp.pos.x, rp.pos.y) + kGroundSitHip * s, rp.pos.y) * yaw * sized);
                } else if (InstanceGroup* lie = pedLie[ai % pedLie.size()]) {   // on its back, head away from the way
                    // it faces (its feet point there), in its own pose of the few
                    const Real bed = rp.raised ? rp.hip : groundAt(rp.pos.x, rp.pos.y);   // (a lounger's, or the ground)
                    lie->transforms.push_back(Mat4::translate(rp.pos.x, bed + kLieHalfDepth * s, rp.pos.y) *
                                                 yaw * Mat4::rotateX(-engine::PI * 0.5) * sized);
                }
                continue;
            }
            Real feet = 0;
            const Mat4 pose = sim_.poses().walkerStandingPose(sim_, static_cast<int>(ai), params_.pedSize.y, &feet);
            boardUnder(static_cast<int>(ai), pose.m[0][3], feet, pose.m[2][3], std::atan2(a.heading.x, a.heading.y));
            ped->push(pose, movingId(a, 0));
            pedAgentIds_[0].push_back(static_cast<int>(ai));
        }
    }
    sec(0);
    // THE CARS, which are entities and no row of the sim's (ADR-0444, ADR-0453), into the car groups
    syncCarEntities(world, cars);

    // THE SIGNAL HEADS (ADR-0456): each shows what the new driving's lamps show for the traffic arriving by the arm
    // it is attached to (driving::headAt): its face in its post instance's bottom row (mesh.vert lights that lamp),
    // its arrow as a separate lens beside it. Dark where the new driving does not drive (the old levels): there is no
    // other clock. The pole assembly (signalPostGroup_) is static and left untouched.
    sec(1);
    tieSignalHeads(world);
    if (InstanceGroup* pg = signalPostGroup_.valid() ? world.get<InstanceGroup>(signalPostGroup_) : nullptr)
        for (std::size_t k = 0; k < signalHeads_.size() && k < pg->transforms.size(); ++k) {
            const driving::HeadShows h = signalHeadShows(k);
            pg->transforms[k].m[3][0] = static_cast<Real>(headFaceNumber(h.face));
            if (h.arrow == driving::Lamp::Dark) continue;
            InstanceGroup* lens = sig[static_cast<int>(h.arrow == driving::Lamp::GreenArrow ? LensColour::Green : LensColour::Amber)];
            if (lens) lens->transforms.push_back(signalHeads_[k].arrowLens[h.arrowFor == driving::Movement::NearTurn ? 1 : 0]);
        }

    // THE MID-RANGE BAND: far agents in reach, as stand-ins, run on from their last coarse tick
    farDrawn_ = 0;
    if (farPedGroup_.valid() && sim_.hasTierCenter() && !gpuCrowd_ && crowdInit_) {   // (the GPU crowd draws them
        // when it can -- and until feedGpuCrowd has decided whether it can, nothing: see its note)
        InstanceGroup* fp = world.get<InstanceGroup>(farPedGroup_);
        if (fp) fp->transforms.clear();
        const Vec2 c = sim_.tierCenter();
        for (int vi : sim_.farAgents()) {
            const Vec2 d = sim_.agents()[static_cast<std::size_t>(vi)].pos - c;
            if (d.x * d.x + d.y * d.y > farPedRadius * farPedRadius) continue;
            Vec2 p, h;
            if (!sim_.farDrawPose(vi, p, h)) continue;
            AgentDrawCache& dcf = drawCache(vi);
            if ((p - dcf.farYAt).lengthSquared() > 25.0) { dcf.farY = groundAt(p.x, p.y); dcf.farYAt = p; }
            const Real yaw = std::atan2(h.x, h.y);
            if (fp) {
                const Real s = sim_.bodyScale(vi);   // (a child's height: the feet stay on the ground)
                const Real deck = boardUnder(vi, p.x, dcf.farY, p.y, yaw);
                fp->transforms.push_back(
                    Mat4::trs(Vec3(p.x, dcf.farY + deck + 0.9 * s, p.y), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s, s)));
            }
            ++farDrawn_;
        }
        refreshBounds(fp);
    }

    for (InstanceGroup* c : cars) refreshBounds(c);
    refreshBounds(ped);
    refreshBounds(pedSeated);
    refreshBounds(pedGround);
    for (InstanceGroup* lie : pedLie) refreshBounds(lie);
    for (InstanceGroup* bg : boardsG) refreshBounds(bg);
    for (int s = 0; s < kLensColourCount; ++s) refreshBounds(sig[s]);
    syncBeachThings(world);

    sec(2);
    // SEE-INTO VEHICLES. The clear glass rides exactly its bodies' transforms
    // (bodies and agent ids were pushed in lockstep above), and the driver is
    // posed on the same pose, on the driver's seat. (The passengers a bus or a
    // cab carried went with them, ADR-0460.)
    {
        InstanceGroup* drv = world.get<InstanceGroup>(busDriverGroup_);
        if (drv) drv->beginStep();
        // THE NEAR SWAP (fleet v2): the kNearCount moving cars nearest the player, within kNearIn m (kept
        // until kNearOut, so a car at the edge does not flicker), draw clear glass, their cabin and driver.
        {
            constexpr Real kNearIn = 50.0, kNearOut = 58.0;
            constexpr std::size_t kNearCount = 12;
            std::vector<std::pair<Real, int>> cand;
            if (sim_.hasTierCenter()) {
                const Vec2 c = sim_.tierCenter();
                for (std::size_t v = 0; v < cars.size() && v < carSeeInto_.size(); ++v) {
                    if (!cars[v] || carSeeInto_[v]) continue;
                    for (std::size_t k = 0; k < carAgentIds_[v].size() && k < cars[v]->transforms.size(); ++k) {
                        const int ai = carAgentIds_[v][k];
                        if (ai < 0) continue;
                        if (!carOccupied(ai)) continue;   // a car left at the kerb is not swapped: nobody in it
                        // (where the car is drawn: a household car's driver is not where the car is, ADR-0444)
                        const Mat4& xf = cars[v]->transforms[k];
                        const Real d = (Vec2(xf.m[0][3], xf.m[2][3]) - c).length();
                        if (d < (nearSwap_.count(ai) ? kNearOut : kNearIn)) cand.push_back({d, ai});
                    }
                }
            }
            std::sort(cand.begin(), cand.end());
            nearSwap_.clear();
            for (std::size_t i = 0; i < cand.size() && i < kNearCount; ++i) nearSwap_.insert(cand[i].second);
        }
        for (std::size_t v = 0; v < cars.size() && v < carGlassGroups_.size(); ++v) {
            if (!cars[v]) continue;
            const bool always = carSeeInto_[v] != 0;
            const std::vector<int>& ids = carAgentIds_[v];
            InstanceGroup* clear = carGlassGroups_[v].valid() ? world.get<InstanceGroup>(carGlassGroups_[v]) : nullptr;
            InstanceGroup* opaque = carGlassOpaqueGroups_[v].valid() ? world.get<InstanceGroup>(carGlassOpaqueGroups_[v]) : nullptr;
            InstanceGroup* cabin = carInteriorGroups_[v].valid() ? world.get<InstanceGroup>(carInteriorGroups_[v]) : nullptr;
            if (clear) clear->beginStep();
            if (opaque) opaque->beginStep();
            if (cabin) cabin->beginStep();
            for (std::size_t k = 0; k < cars[v]->transforms.size(); ++k) {
                const Mat4& xf = cars[v]->transforms[k];
                const uint64_t body = k < cars[v]->ids.size() ? cars[v]->ids[k] : 0;   // its parts ride its name
                const int ai = k < ids.size() ? ids[k] : -1;
                // near-swapped only when the slot HAS a cabin to show (a kit body without one keeps its glass
                // opaque rather than showing an empty shell through clear glass)
                const bool seen = always || (ai >= 0 && nearSwap_.count(ai) && cabin);
                if (seen) {
                    if (clear) clear->push(xf, body | 1);
                    if (cabin) cabin->push(xf, body | 2);
                } else if (opaque) {
                    opaque->push(xf, body | 1);
                } else if (clear) {
                    clear->push(xf, body | 1);   // a slot with no opaque form keeps its clear glass
                }
            }
            refreshBounds(clear);
            refreshBounds(opaque);
            refreshBounds(cabin);
            const bool hasDriver = carHasDriver_[v] != 0;
            InstanceGroup* rider = v < carRiderGroups_.size() && carRiderGroups_[v].valid() ? world.get<InstanceGroup>(carRiderGroups_[v]) : nullptr;
            if (rider) rider->beginStep();
            if (!hasDriver) continue;
            for (std::size_t k = 0; k < ids.size() && k < cars[v]->transforms.size(); ++k) {
                const int ai = ids[k];
                if (ai < 0) continue;   // a car with nobody at its wheel: nobody aboard
                if (!always && (!nearSwap_.count(ai) || !carInteriorGroups_[v].valid())) continue;   // behind opaque glass: nobody to see
                const Mat4& xf = cars[v]->transforms[k];
                // NOBODY SITS IN A PARKED CAR (Glenn: "npc bodies sitting dormant in their parked cars ... They should
                // either be in a house dormant or disappeared"): a driver is in the seat only while the car is on a
                // trip (moving); at rest the person is indoors and the car stands empty
                if (!carOccupied(ai)) continue;
                const uint64_t body = k < cars[v]->ids.size() ? cars[v]->ids[k] : 0;
                if (rider || drv) (rider ? rider : drv)->push(xf * Mat4::trs(carDriverSeat_[v], Quat(), Vec3(1, 1, 1)), body | 3);
            }
            refreshBounds(rider);
        }
        refreshBounds(drv);
    }

    sec(3);
    // Debug widgets: a ground footprint (coloured by state) + a forward arrow per
    // agent, scaled to the agent's speed so it reads as the present trajectory.
    {
        constexpr int kStateCount = static_cast<int>(Agent::State::Count);
        InstanceGroup* foot[kStateCount];
        for (int s = 0; s < kStateCount; ++s) {
            foot[s] = world.get<InstanceGroup>(footprintGroups_[s]);
            if (foot[s]) foot[s]->transforms.clear();
        }
        InstanceGroup* fwd = world.get<InstanceGroup>(forwardGroup_);
        if (fwd) fwd->transforms.clear();
        InstanceGroup* cone = world.get<InstanceGroup>(visionGroup_);
        if (cone) cone->transforms.clear();
        // The navgraph view is a static bake: copy it in when the HUD is on,
        // leave the groups empty when it's off (mirrors the vanishing rings).
        InstanceGroup* navL = world.get<InstanceGroup>(navLinkGroup_);
        InstanceGroup* navN = world.get<InstanceGroup>(navNodeGroup_);
        const bool showNav = debugWidgets_ && showNavGraph_;
        if (navL) navL->transforms = showNav ? navLinkBake_ : std::vector<Mat4>{};
        if (navN) navN->transforms = showNav ? navNodeBake_ : std::vector<Mat4>{};
        // City-plan outlines (static bakes, same show-or-empty pattern).
        const bool showPlan = debugWidgets_ && showPlan_;
        if (showPlan && !planBaked_) bakePlanOutlines(world);
        InstanceGroup* blk = world.get<InstanceGroup>(blockGroup_);
        InstanceGroup* lot = world.get<InstanceGroup>(lotGroup_);
        if (blk) blk->transforms = showPlan ? blockBake_ : std::vector<Mat4>{};
        if (lot) lot->transforms = showPlan ? lotBake_ : std::vector<Mat4>{};
        // Collider prisms (static bake, same pattern).
        const bool showCol = debugWidgets_ && showColliders_;
        InstanceGroup* colS = world.get<InstanceGroup>(colliderStripGroup_);
        InstanceGroup* colP = world.get<InstanceGroup>(colliderPostGroup_);
        if (colS) colS->transforms = showCol ? colliderStripBake_ : std::vector<Mat4>{};
        if (colP) colP->transforms = showCol ? colliderPostBake_ : std::vector<Mat4>{};
        const auto& agents = sim_.agents();
        for (std::size_t ai = 0; ai < agents.size() && debugWidgets_; ++ai) {
            const Agent& a = agents[ai];
            if (a.far()) continue;   // far tier: nothing drawn
            Real x = a.pos.x, z = a.pos.y;
            Vec2 heading = a.heading;
            // Where this agent is TRYING to go (the arrow's tip): the ghost's
            // short-horizon intent by default; the bridge's pursuit target when
            // the body is external.
            Vec2 goal = a.pos + a.heading * (2.0 + a.speed);
            Agent::State ringState = a.state;   // body truth may override below
            // Externally-owned walkers (ADR-0062): the widget must ring the
            // PHYSICAL body, not the planner ghost — the ghost legitimately runs
            // ahead or behind, and a ring around it is an empty circle on the
            // ground. Use the bridge-reported real pose; a walker with no
            // reported body (not yet spawned) draws none.
            if (pedsExternallyOwned_) {
                const std::vector<ExternalAgentPose>& poses = externalPedPoses_;
                bool reported = false;
                for (const ExternalAgentPose& p : poses)
                    if (p.agentId == static_cast<int>(ai)) {
                        x = p.pos.x;
                        z = p.pos.y;
                        heading = p.heading;
                        goal = p.target;
                        if (p.stateOverride >= 0 &&
                            p.stateOverride < static_cast<int>(Agent::State::Count))
                            ringState = static_cast<Agent::State>(p.stateOverride);
                        reported = true;
                        break;
                    }
                if (!reported) continue;
            }
            // Projected onto the GROUND like painted road markings (always
            // visible the way lane paint is), just proud of the asphalt/sidewalk.
            Real y = groundAt(x, z) + a.elevation + 0.06;
            // Ring sized to the body, proud of it so the painted rim always shows
            const Real radius = params_.pedSize.x * 0.9 * 1.2;
            InstanceGroup* fg = foot[static_cast<int>(ringState)];
            if (showAgentWidgets_ && fg) fg->transforms.push_back(
                Mat4::trs(Vec3(x, y, z), Quat(), Vec3(radius, 1, radius)));
            if (showAgentWidgets_ && fwd && a.moving) {
                // The INTENT arrow: from the agent to where it's trying to go —
                // the pursuit target for external bodies, the ghost's short-
                // horizon aim otherwise. Reads as "this is my plan".
                Vec2 toGoal(goal.x - x, goal.y - z);
                Real dist = std::sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);
                Real len = std::max(Real(0.8), std::min(Real(8.0), dist));
                Real yaw = dist > 1e-4 ? std::atan2(toGoal.x, toGoal.y)
                                       : std::atan2(heading.x, heading.y);
                fwd->transforms.push_back(Mat4::trs(
                    Vec3(x, y, z), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                    Vec3(0.30, 1, len)));   // wide enough to read at street level
            }
            // The sensing wedge: what this walker can SEE, yawed to its heading
            // and scaled to its cone range (the half-angle is baked into the
            // mesh). Moving walkers only, and just below the ring so the two
            // never z-fight.
            InstanceGroup* cg = cone;
            if (showVisionCones_ && cg && a.moving) {
                const Real range = kPedConeRange;
                Real coneYaw = std::atan2(heading.x, heading.y);
                cg->transforms.push_back(Mat4::trs(
                    Vec3(x, y - 0.01, z),
                    Quat::fromAxisAngle(Vec3(0, 1, 0), coneYaw),
                    Vec3(range, 1, range)));
            }
        }
        for (int s = 0; s < kStateCount; ++s) refreshBounds(foot[s]);
        refreshBounds(fwd);
        refreshBounds(cone);
        refreshBounds(navL);
        refreshBounds(navN);
        // Block/lot outlines are ONE city-wide merged mesh drawn at a single
        // identity transform, so their extent lives in the MESH geometry, not
        // the transforms. refreshBounds derives the cull sphere from the
        // transforms, which for one identity instance collapses to a ~5 m
        // sphere at the origin — so the whole overlay popped in and out with
        // view direction (device: "they're glued together as one object and
        // the origin determines if it appears"). Give them a city-wide sphere.
        if (blk) { blk->boundsCenter = Vec3(0, 0, 0); blk->boundsRadius = 6000.0; }
        if (lot) { lot->boundsCenter = Vec3(0, 0, 0); lot->boundsRadius = 6000.0; }
    }

    sec(4);
    // Car lamps (ADR-0342): each drawn car's lit lamps onto its body instance, and the bus signs. Always baked.
    syncCarLamps(world);
    sec(5);
    if (syncStats && ++secCalls % 300 == 0) {
        LOG_INFO << "[stats] group sync ms per bake: agents " << secMs[0] / secCalls << ", parked cars " << secMs[1] / secCalls
                 << ", signals + bounds " << secMs[2] / secCalls << ", see-into vehicles " << secMs[3] / secCalls
                 << ", debug widgets " << secMs[4] / secCalls << ", car lamps " << secMs[5] / secCalls;
        for (double& m : secMs) m = 0;
        secCalls = 0;
    }
}

// PEOPLE INSIDE (campus M4). The sim rests an indoor agent at its place's door, undrawn; a streamed interior's
// furniture set says where in the building a body can be. Once a second: count who is indoors at each streamed
// building's place, and put that many on its seats and beds (planOccupants -- the same seats for the same count, so
// a refresh adds or removes people rather than shuffling them), marking the spots taken so the player is not
// offered someone's seat.
void CityRenderSystem::stepIndoors(World& world, Real dt) {
    indoorAcc_ += dt;
    if (indoorAcc_ < 1.0) return;
    indoorAcc_ = 0;
    InstanceGroup* sit = indoorSitGroup_.valid() ? world.get<InstanceGroup>(indoorSitGroup_) : nullptr;
    InstanceGroup* body = indoorBodyGroup_.valid() ? world.get<InstanceGroup>(indoorBodyGroup_) : nullptr;
    if (!sit || !body) return;
    sit->transforms.clear();
    body->transforms.clear();
    indoorDrawn_ = 0;
    const engine::CityBuildings* cb = nullptr;
    world.each<engine::CityBuildings>([&](Entity, engine::CityBuildings& c) { cb = &c; });
    // A SLOT: one place's people on its own pieces -- each shop unit's (its staff behind the counter, its customers at
    // its tables), and the building's own (the flats, the offices) on the rest
    struct Slot { PlaceId place; std::vector<uint8_t> units; Vec2 door; int people = 0, staff = 0; };
    struct Inside { engine::Interactables* set; int building; std::vector<Slot> slots; };
    std::vector<Inside> inside;
    world.each<engine::Interactables>([&](Entity, engine::Interactables& set) {
        if (set.building < 0 || !cb || set.building >= static_cast<int>(cb->records.size())) return;
        // which interior this is: its shop units' floor heights (a storey window of a tower has its own, or none)
        std::vector<double> unitsKey;
        unitsKey.reserve(set.units.size());
        for (const auto& un : set.units) unitsKey.push_back(un.y0);
        auto it = buildingPlace_.find(set.building);
        if (it != buildingPlace_.end() && it->second.units != unitsKey) {
            LOG_INFO << "[indoors] building " << set.building << ": another interior window (" << it->second.units.size()
                     << " -> " << unitsKey.size() << " shop units), matched afresh";
            buildingPlace_.erase(it);
            it = buildingPlace_.end();
        }
        if (it == buildingPlace_.end()) {
            // the places whose sites are inside this building's plan; each shop unit takes the one inside its room
            // (two storeys of a mall over the same rooms: the lower unit the lower id, as they were exported)
            const engine::BuildingRecord& r = cb->records[static_cast<std::size_t>(set.building)];
            std::vector<PlaceId> cand;
            if (r.plan.size() >= 3)
                for (const Place& p : places_.places())
                    if (engine::pointInPolygon(r.plan, p.site)) cand.push_back(p.id);
            BuildingPeople bp;
            bp.units = unitsKey;
            bp.unit.assign(set.units.size(), kNoPlace);
            std::vector<std::size_t> order(set.units.size());
            for (std::size_t u = 0; u < order.size(); ++u) order[u] = u;
            std::stable_sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return set.units[x].y0 < set.units[y].y0; });
            std::vector<uint8_t> claimed(cand.size(), 0);
            for (std::size_t u : order)
                for (std::size_t c = 0; c < cand.size(); ++c)
                    if (!claimed[c] && set.units[u].rect.size() >= 3 && engine::pointInPolygon(set.units[u].rect, places_[cand[c]].site)) {
                        bp.unit[u] = cand[c];
                        claimed[c] = 1;
                        break;
                    }
            for (std::size_t c = 0; c < cand.size() && bp.whole == kNoPlace; ++c)
                if (!claimed[c]) bp.whole = cand[c];
            it = buildingPlace_.emplace(set.building, std::move(bp)).first;
            if (std::getenv("RT_CAMPUS_DEBUG")) {
                int matched = 0;
                for (PlaceId q : it->second.unit) matched += q != kNoPlace;
                std::fprintf(stderr, "[indoors] building %d (%s, campus %d, %d floors, %zu pieces): place %d, %zu shop units (%d with a place), %zu places inside\n",
                             set.building, r.recipe.c_str(), r.params.campus, r.params.floors, set.pieces.size(),
                             it->second.whole == kNoPlace ? -1 : static_cast<int>(it->second.whole), set.units.size(), matched, cand.size());
                for (std::size_t u = 0; u < set.units.size(); ++u) {
                    const auto& rr = set.units[u].rect;
                    Vec2 c(0, 0);
                    for (const Vec2& q : rr) c = c + q * (1.0 / rr.size());
                    const PlaceId q = it->second.unit[u];
                    int pieces = 0;
                    for (const engine::InteractPiece& ip : set.pieces) pieces += ip.unit == static_cast<int>(u);
                    std::fprintf(stderr, "[indoors unit] building %d unit %zu at %.1f %.1f y %.1f (%.1f x %.1f), %d pieces: place %d %s\n", set.building, u, c.x, c.y,
                                 set.units[u].y0, rr.size() > 2 ? (rr[1] - rr[0]).length() : 0.0, rr.size() > 3 ? (rr[3] - rr[0]).length() : 0.0, pieces,
                                 q == kNoPlace ? -1 : static_cast<int>(q), q == kNoPlace ? "" : places_[q].name.c_str());
                }
                for (PlaceId q : cand) {
                    bool got = false;
                    for (PlaceId x : it->second.unit) got = got || x == q;
                    if (!got) std::fprintf(stderr, "[indoors unit] building %d place %u \"%s\" (%s) site %.1f %.1f has no unit\n", set.building,
                                           static_cast<unsigned>(q), places_[q].name.c_str(), placeTypeName(places_[q].type), places_[q].site.x, places_[q].site.y);
                }
            }
        }
        const BuildingPeople& bp = it->second;
        // the door an indoor agent rests at (CitySim::assignPlaces' doorOf)
        auto restAt = [&](PlaceId q) { const Place& p = places_[q]; return p.entrance + (p.site - p.entrance) * 0.4; };
        Inside in{&set, set.building, {}};
        Slot whole{bp.whole, std::vector<uint8_t>(set.units.size() + 1, 0), Vec2(0, 0)};
        whole.units[0] = 1;   // the building's own pieces, and any shop unit that found no place
        for (std::size_t u = 0; u < bp.unit.size(); ++u) {
            if (bp.unit[u] == kNoPlace) { whole.units[u + 1] = 1; continue; }
            Slot s{bp.unit[u], std::vector<uint8_t>(set.units.size() + 1, 0), restAt(bp.unit[u])};
            s.units[u + 1] = 1;
            in.slots.push_back(std::move(s));
        }
        if (bp.whole != kNoPlace) { whole.door = restAt(bp.whole); in.slots.push_back(std::move(whole)); }
        if (!in.slots.empty()) inside.push_back(std::move(in));
    });
    // who is in: indoors, at rest, at that place's door -- and of them, who works there and is at work
    // (the near list: interiors stream in round the player, and whoever rests inside one is near -- this was every
    // indoor agent in the city against every streamed building, once a second)
    if (!inside.empty())
        for (int ni : sim_.nearAgents()) {
            const Agent& a = sim_.agents()[static_cast<std::size_t>(ni)];
            if (!a.indoors || a.moving) continue;
            bool done = false;
            for (Inside& in : inside) {
                for (Slot& s : in.slots)
                    if ((a.pos - s.door).lengthSquared() < 0.25) {
                        ++s.people;
                        s.staff += a.workPlace == s.place && a.activity == Agent::Activity::AtWork;
                        done = true;
                        break;
                    }
                if (done) break;
            }
        }
    const engine::FurnitureLibrary& lib = engine::FurnitureLibrary::global();
    const Real h = sim_.clockHours();
    for (Inside& in : inside) {
        engine::Interactables& set = *in.set;
        std::vector<uint32_t>& held = indoorHeld_[in.building];
        held.resize(set.pieces.size(), 0);
        for (std::size_t i = 0; i < set.pieces.size(); ++i) set.pieces[i].taken &= ~held[i];   // ours back first
        std::fill(held.begin(), held.end(), 0u);
        for (Slot& s : in.slots) {
            engine::OccupantPlan plan;
            plan.people = s.people;
            plan.staff = s.staff;
            plan.units = &s.units;
            plan.night = (h >= 22.5 || h < 6.5) && places_[s.place].type == PlaceType::Home;   // asleep at home, not at work
            plan.lecturer = cb->records[static_cast<std::size_t>(in.building)].params.campus == 1 && s.people > 0 && !plan.night;
            plan.seed = static_cast<uint32_t>(in.building);
            const std::vector<engine::Occupant> occ = engine::planOccupants(set, lib, plan);
            for (const engine::Occupant& o : occ) {
                set.pieces[o.piece].taken |= o.spots;
                held[o.piece] |= o.spots;
                (o.pose == engine::Occupant::Pose::Sit ? sit : body)->transforms.push_back(o.at);
                ++indoorDrawn_;
            }
            if (std::getenv("RT_CAMPUS_DEBUG") && s.people > 0) {
                const Place& p = places_[s.place];
                int lie = 0, stand = 0;
                for (const engine::Occupant& o : occ) { lie += o.pose == engine::Occupant::Pose::Lie; stand += o.pose == engine::Occupant::Pose::Stand; }
                std::fprintf(stderr, "[indoors] %.2f h building %d place %u (%s \"%s\" at %.0f %.0f): %d inside (%d staff), %zu placed (%d lying, %d standing)\n",
                             h, in.building, static_cast<unsigned>(s.place), placeTypeName(p.type), p.name.c_str(), p.site.x, p.site.y,
                             s.people, s.staff, occ.size(), lie, stand);
                for (const engine::Occupant& o : occ)   // where to stand and look at them: 3 m in front, looking back
                    if (o.pose == engine::Occupant::Pose::Stand) {
                        const Vec3 at(o.at.m[0][3], o.at.m[1][3], o.at.m[2][3]);
                        const Vec3 f = normalize(Vec3(o.at.m[0][2], 0, o.at.m[2][2]));
                        const Vec3 eye = at + f * 3.0 + Vec3(0, 0.75, 0);
                        std::fprintf(stderr, "[indoors view] place %u staff at %.2f %.2f %.2f camera %.2f %.2f %.2f -12 %.1f\n",
                                     static_cast<unsigned>(s.place), at.x, at.y, at.z, eye.x, eye.y, eye.z,
                                     std::atan2(-f.x, f.z) * 57.29577951308232);
                        break;
                    }
            }
        }
    }
    refreshBounds(sit);   // a group with no bounds is culled whole
    refreshBounds(body);
    // a building released since: forget what we held there
    for (auto it = indoorHeld_.begin(); it != indoorHeld_.end();) {
        bool live = false;
        for (const Inside& in : inside) live = live || in.building == it->first;
        it = live ? std::next(it) : indoorHeld_.erase(it);
    }
}

void CityRenderSystem::step(World& world, Real dt) {
    if (!built_) return;
    stepIndoors(world, dt);
    bakeDt_ = dt;   // the tilt low-pass keys its gain to the bake interval
    // CADENCE IS THE SIM'S OWN (P8.2e): hand it the level's period, scaled by
    // the adaptive load multiplier, and let it decide whether this call
    // advances anything. The bridge's job is time in, poses out — deciding a
    // system's rate one layer above the state it governs is what let the far
    // tier's bucket phase silently disagree with it.
    sim_.setTickPeriod(params_.localHz > 0.0
                           ? (1.0 / params_.localHz) * static_cast<Real>(loadMul_)
                           : 0.0);
    // Three-tier traffic (P4): feed the sim the player's position each fixed
    // step — the V/K bubble's centre. No player entity (headless tests, menu
    // scenes) clears it, and everything stays K. Inert unless the level set
    // tieredAgents; deterministic for a deterministic world.
    {
        bool haveCentre = false;
        // THE PLAYER, identified by ControlledBy — not "the first entity that
        // happens to have a CharacterController". Every walker has one of
        // those, so this used to centre the whole tier bubble on an arbitrary
        // pedestrian, and which one depended on component-pool churn. The
        // bubble decides who is simulated, drawn and given a body, so it has to
        // follow the actual player.
        world.each<engine::Transform, engine::ControlledBy>(
            [&](Entity, engine::Transform& t, engine::ControlledBy&) {
                if (haveCentre) return;
                sim_.setTierCenter(engine::Vec2(t.position.x, t.position.z));
                haveCentre = true;
            });
        // Fallback for hosts that stand in a bare character for the player (the
        // headless tier tests do exactly this). A real level's player carries
        // ControlledBy, so the walker crowd can never win the branch above.
        if (!haveCentre)
            world.each<engine::CharacterController>(
                [&](Entity e, engine::CharacterController&) {
                    if (haveCentre) return;
                    if (const engine::Transform* t = world.get<engine::Transform>(e)) {
                        sim_.setTierCenter(engine::Vec2(t->position.x, t->position.z));
                        haveCentre = true;
                    }
                });
        if (!haveCentre) sim_.clearTierCenter();
    }
    // RT_DUMP_STATS: split the fixed-step bill — CitySim::step vs the group
    // bake — so the 20fps hunt names its target (8km-city plan P6).
    static const bool dumpStats = std::getenv("RT_DUMP_STATS") != nullptr;
    if (!dumpStats) {
        sim_.step(dt, params_.hoursPerSecond);
        sim_.poses().stepCarPoses(world, sim_, dt);   // (every step: the cars' bodies are the sim's, ADR-0454)
        if (bakeThisStep_) syncGroups(world);
        return;
    }
    static double simMs = 0.0, syncMs = 0.0;
    static int calls = 0;
    const CitySim::PhaseTimes ph0 = sim_.phaseTimes();
    const engine::RouteStats rs0 = engine::routeStats();
    auto t0 = std::chrono::steady_clock::now();
    sim_.step(dt, params_.hoursPerSecond);
    sim_.poses().stepCarPoses(world, sim_, dt);
    auto t1 = std::chrono::steady_clock::now();
    if (bakeThisStep_) syncGroups(world);
    auto t2 = std::chrono::steady_clock::now();
    simMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    syncMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
    // RT_SPIKE_MS=<ms>: the threshold (default 16); 0 logs every tick, what a spike is compared against
    static const double spikeMs = std::getenv("RT_SPIKE_MS") ? std::atof(std::getenv("RT_SPIKE_MS")) : 16.0;
    if (std::chrono::duration<double, std::milli>(t2 - t0).count() > spikeMs &&
        (sim_.phaseTimes().steps != ph0.steps || std::chrono::duration<double, std::milli>(t2 - t1).count() > 16.0))
        LOG_INFO << "[stats] SPIKE citysim step " << std::chrono::duration<double, std::milli>(t1 - t0).count()
                 << " ms, group sync " << std::chrono::duration<double, std::milli>(t2 - t1).count() << " ms"
                 << " | phases ms: rehash " << (sim_.phaseTimes().rehash - ph0.rehash) / 1000.0
                 << " tier " << (sim_.phaseTimes().tierPass - ph0.tierPass) / 1000.0
                 << " goals " << (sim_.phaseTimes().goals - ph0.goals) / 1000.0
                 << " sensed " << (sim_.phaseTimes().sensedBuild - ph0.sensedBuild) / 1000.0
                 << " gaps " << (sim_.phaseTimes().gaps - ph0.gaps) / 1000.0
                 << " active " << (sim_.phaseTimes().activeList - ph0.activeList) / 1000.0
                 << " move " << (sim_.phaseTimes().advMove - ph0.advMove) / 1000.0
                 << " pairs " << (sim_.phaseTimes().advPairs - ph0.advPairs) / 1000.0
                 << " pop " << (sim_.phaseTimes().advPop - ph0.advPop) / 1000.0
                 << " solver " << (sim_.phaseTimes().advSolver - ph0.advSolver) / 1000.0
                 << " tail " << (sim_.phaseTimes().advance - ph0.advance) / 1000.0
                 << " total " << (sim_.phaseTimes().total - ph0.total) / 1000.0
                 << " | routes " << engine::routeStats().calls - rs0.calls << " in "
                 << engine::routeStats().ms - rs0.ms << " ms, " << engine::routeStats().expanded - rs0.expanded
                 << " nodes expanded";
        {   // who routed: caller addresses (the viewer is not PIE: `nm -C -n` names them)
            std::vector<std::pair<const void*, long>> d;
            for (const auto& c : engine::routeStats().callers) {
                long before = 0;
                for (const auto& b : rs0.callers) if (b.first == c.first) before = b.second;
                if (c.second > before) d.push_back({c.first, c.second - before});
            }
            std::sort(d.begin(), d.end(), [](const auto& x, const auto& y) { return x.second > y.second; });
            for (std::size_t k = 0; k < d.size() && k < 4; ++k)
                LOG_INFO << "[stats]   route caller " << d[k].first << " x" << d[k].second;
        }
    if (++calls % 300 == 0) {
        // `moving` means "has an open route" — it stays true through red
        // lights, crash freezes, car-following stops and the far tier's
        // modelled junction dwells. On its own it cannot tell a jam from a
        // backlog of trips still in flight, so count the ones with actual
        // speed beside it: a big gap between the two IS the jam.
        int kTier = 0, vTier = 0, moving = 0, rolling = 0, dormant = 0;
        for (const Agent& a : sim_.agents()) {
            if (a.far()) ++vTier; else ++kTier;
            if (a.tier == Agent::Tier::D) ++dormant;
            if (a.moving) {
                ++moving;
                if (a.speed > 0.5) ++rolling;
            }
        }
        LOG_INFO << "[stats] citysim step " << (simMs / calls)
                 << " ms, group sync " << (syncMs / calls)
                 << " ms (per fixed step) | hour " << sim_.clockHours()
                 << " | active(K/P) " << kTier
                 << ", far(V) " << vTier << ", dormant(D) " << dormant
                 << ", moving " << moving << ", rolling " << rolling
                 << ", asleep " << sim_.sleepingAgents()
                 << " | far band drawn " << farDrawn_
                 << " | GPU crowd travellers " << crowdLive_ << " drawn " << crowdDrawnSeen_ << " | near ring x" << sim_.nearScale() << " far ring x" << sim_.farScale()
                 << " | stand-ins left in the CPU band's group " << [&] {
                        // (0 whenever the GPU crowd draws them: anything else is frozen boxes)
                        const InstanceGroup* g = farPedGroup_.valid() ? world.get<InstanceGroup>(farPedGroup_) : nullptr;
                        return g ? g->transforms.size() : std::size_t(0);
                    }()
                 << " | sleepers' events " << (sim_.dormantEvents() - lastDormantEvents_) << " in "
                 << (sim_.dormantEventMs() - lastDormantMs_) / calls << " ms per step";
        lastDormantEvents_ = sim_.dormantEvents();
        lastDormantMs_ = sim_.dormantEventMs();
        simMs = syncMs = 0.0;
        calls = 0;
    }
}

void CityRenderSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("agent_widgets", engine::KeyCode::J);   // toggle debug widgets
    // Semicolon, not L: L is the car's headlight toggle (VehicleSystem), and
    // switching lights used to flash the whole block/lot plan on with it.
    ctx.actions.bindButton("plan_widgets", engine::KeyCode::Semicolon);
    onSimHour_ = ctx.events.listen<engine::SimHourQuery>([this](const engine::SimHourQuery& q) {
        if (q.hours != nullptr && built_) *q.hours = sim_.clockHours();
    });
    onPin_ = ctx.events.listen<engine::PinRequest>([this](const engine::PinRequest& r) { pinRequests_.push_back(r); });
    onFindPlace_ = ctx.events.listen<engine::FindPlaceRequest>([this](const engine::FindPlaceRequest& r) {
        findPlace_ = r.name;
    });
    onLayer_ = ctx.events.listen<engine::CityLayerRequest>([this](const engine::CityLayerRequest& r) {
        layerRequests_.push_back(r);
    });
    onJitter_ = ctx.events.listen<engine::JitterRequest>([this](const engine::JitterRequest& r) {
        jitterRequests_.push_back(r);
    });
    for (const char* q : {"traffic?", "opensigns?", "findplace?"})
        answers_.bind(ctx.commands, "CityRenderSystem", q);
    answers_.bind(ctx.commands, "CityRenderSystem", "jitter?", "nothing recorded (ask with `jitter`)");
    answers_.bind(ctx.commands, "CityRenderSystem", "pin?", "none followed");
    // `ground? x z [r]` asks (r dumps the area to RT_GROUND_DUMP); `ground?` alone reads the answer.
    if (ctx.commands != nullptr)
        answerGround_ = ctx.commands->bind("ground?", "CityRenderSystem", [this](const engine::ControlCommand& cmd) -> std::string {
            if (cmd.args.size() >= 2) {
                groundQuery_ = cmd.args[0] + " " + cmd.args[1] + (cmd.args.size() >= 3 ? " " + cmd.args[2] : "");
                return "ok staged (poll `ground?` with no args)";
            }
            return "ok " + groundResult_;
        });
}

void CityRenderSystem::feedGpuCrowd(engine::FrameContext& ctx) {
    if (!built_ || !farPedGroup_.valid()) return;
    World& world = ctx.world;
    if (!crowdInit_) {
        crowdInit_ = true;
        if (!ctx.renderer.supportsGpuCrowd() || std::getenv("RT_NO_FAR_BAND")) return;
        // what a traveller is drawn as: the band's stand-in, the box person
        std::vector<engine::Renderer::CrowdKind> kinds;
        const InstanceGroup* pg = world.get<InstanceGroup>(farPedGroup_);
        if (!pg) return;
        kinds.push_back({pg->mesh, pg->material, 0.9f, static_cast<float>(farPedDistance), 60.0f});
        ctx.renderer.setCrowdKinds(kinds);
        // every walk's segments, each way, with their heights (the walks' own: a bridge's sidewalk is up on the bridge)
        const engine::RoadModel* model = sim_.roads();
        if (!model || !model->built) return;   // (no walks: the CPU band draws the far travellers)
        const engine::LaneGraph& G = model->laneLevel;
        crowdWalkSeg_.assign(G.walks.size(), 0);
        uint32_t segs = 0;
        for (std::size_t w = 0; w < G.walks.size(); ++w) {
            crowdWalkSeg_[w] = segs;
            segs += G.walks[w].curve.count > 1 ? G.walks[w].curve.count - 1 : 0;
        }
        crowdRevBase_ = segs;
        std::vector<float> L(static_cast<std::size_t>(segs) * 2 * 6);
        crowdLinkLen_.assign(static_cast<std::size_t>(segs) * 2, 0.0f);
        for (std::size_t w = 0; w < G.walks.size(); ++w) {
            const engine::Range c = G.walks[w].curve;
            for (uint32_t j = 0; j + 1 < c.count; ++j) {
                const Vec2 a = G.points[c.first + j], b = G.points[c.first + j + 1];
                const float ha = static_cast<float>(G.heights[c.first + j]), hb = static_cast<float>(G.heights[c.first + j + 1]);
                const uint32_t f = crowdWalkSeg_[w] + j, r = crowdRevBase_ + f;
                float* o = &L[static_cast<std::size_t>(f) * 6];
                o[0] = static_cast<float>(a.x); o[1] = ha; o[2] = static_cast<float>(a.y);
                o[3] = static_cast<float>(b.x); o[4] = hb; o[5] = static_cast<float>(b.y);
                float* q = &L[static_cast<std::size_t>(r) * 6];
                q[0] = o[3]; q[1] = o[4]; q[2] = o[5];
                q[3] = o[0]; q[4] = o[1]; q[5] = o[2];
                crowdLinkLen_[f] = crowdLinkLen_[r] = static_cast<float>((b - a).length());
            }
        }
        ctx.renderer.setCrowdLinks(L);
        gpuCrowd_ = true;
        // THE CPU BAND STOPS HERE, so empty what it drew. It runs in the fixed step and this in the frame's update: a
        // fixed step before the first update filled the stand-in groups with every far car round the spawn, and with
        // the band off nothing cleared them -- a frozen first-frame snapshot of box cars on the streets and the
        // freeway for the whole session, drawn by the ordinary instanced path, no collider (Glenn: "a black small
        // cube on top of a gray bigger rectilinear block. I could pass through them").
        if (InstanceGroup* g = world.get<InstanceGroup>(farPedGroup_)) g->transforms.clear();
        LOG_INFO << "[citysim] GPU crowd on: " << kinds.size() << " kinds, " << segs << " walk segments each way";
    }
    if (!gpuCrowd_) return;
    const std::vector<Agent>& agents = sim_.agents();
    bool all = false;
    sim_.takeCrowdChanges(crowdChanges_, all);
    if (crowdSlot_.size() != agents.size()) { crowdSlot_.assign(agents.size(), -1); all = true; }
    const std::size_t arenaBefore = crowdArena_.size() / 2;
    crowdDirtySlots_.clear();
    constexpr uint32_t kPedKind = 0;   // (the one kind: the box person)
    auto refresh = [&](int i) {
        const Agent& a = agents[static_cast<std::size_t>(i)];
        const engine::RoadModel* model = sim_.roads();
        const bool live = model && a.tier != Agent::Tier::K && a.moving && !a.released && !a.playerControlled &&
                          CitySim::onRoute(a);
        int& slot = crowdSlot_[static_cast<std::size_t>(i)];
        if (!live) {
            if (slot >= 0) {
                crowdGarbage_ += crowdRec_[static_cast<std::size_t>(slot)].routeLen;
                crowdRec_[static_cast<std::size_t>(slot)] = engine::Renderer::CrowdAgent{};
                crowdOwner_[static_cast<std::size_t>(slot)] = -1;
                crowdFree_.push_back(slot);
                crowdDirtySlots_.push_back(static_cast<uint32_t>(slot));
                --crowdLive_;
                slot = -1;
            }
            return;
        }
        if (slot < 0) {
            if (!crowdFree_.empty()) { slot = crowdFree_.back(); crowdFree_.pop_back(); }
            else { slot = static_cast<int>(crowdRec_.size()); crowdRec_.emplace_back(); crowdOwner_.push_back(-1); }
            crowdOwner_[static_cast<std::size_t>(slot)] = i;
            ++crowdLive_;
        }
        engine::Renderer::CrowdAgent r = crowdRec_[static_cast<std::size_t>(slot)];
        // ITS SPAN, laid from where it is (W3): the segments of the steps it has left, the first from the segment it
        // stands on; dist0 is how far into that first segment it is
        const engine::LaneGraph& G = model->laneLevel;
        if (r.live) crowdGarbage_ += r.routeLen;
        r.routeOff = static_cast<uint32_t>(crowdArena_.size() / 2);
        const auto segOf = [&](const engine::Range& c, Real at) {   // the segment of a walk's curve holding `at`
            const Real* s0 = G.along.data() + c.first;
            const Real* s1 = s0 + c.count;
            const std::ptrdiff_t k = std::upper_bound(s0, s1, at) - s0 - 1;
            return static_cast<uint32_t>(std::clamp<std::ptrdiff_t>(k, 0, static_cast<std::ptrdiff_t>(c.count) - 2));
        };
        float cum = 0;
        uint32_t laid = 0;
        Real cur = 0;
        Vec2 curDir(1, 0);
        for (std::size_t k = static_cast<std::size_t>(a.leg); k < a.route.steps.size(); ++k) {
            const engine::roads::WalkStep& s = a.route.steps[k];
            const engine::Range c = G.walks[s.walk.index].curve;
            if (c.count < 2) continue;
            Real from = s.from;
            if (k == static_cast<std::size_t>(a.leg)) {   // from where it stands
                const Real along = std::fabs(s.to - s.from);
                from = s.from + (s.forward ? 1.0 : -1.0) * std::min(a.distOnLeg, along);
                cur = from;
            }
            const uint32_t jf = segOf(c, from), jt = segOf(c, s.to);
            const uint32_t base = crowdWalkSeg_[s.walk.index];
            auto push = [&](uint32_t seg) {
                uint32_t bits;
                std::memcpy(&bits, &cum, sizeof bits);
                crowdArena_.push_back(seg);
                crowdArena_.push_back(bits);
                cum += crowdLinkLen_[seg];
                ++laid;
            };
            if (s.forward) {
                if (k == static_cast<std::size_t>(a.leg)) {
                    r.dist0 = static_cast<float>(cur - G.along[c.first + jf]);
                    curDir = G.points[c.first + jf + 1] - G.points[c.first + jf];
                }
                for (uint32_t j = jf; j <= jt; ++j) push(base + j);
            } else {
                if (k == static_cast<std::size_t>(a.leg)) {
                    r.dist0 = static_cast<float>(G.along[c.first + jf + 1] - cur);
                    curDir = G.points[c.first + jf] - G.points[c.first + jf + 1];
                }
                for (uint32_t j = jf + 1; j-- > jt;) push(crowdRevBase_ + base + j);
            }
        }
        r.routeLen = laid;
        r.t0 = static_cast<float>(a.vLastTick);
        r.speed = static_cast<float>(a.speed > 0.3 ? a.speed : 1.3 * a.speedFactor);
        const Real dl = curDir.length();
        const Vec2 dir = dl > 1e-6 ? curDir * (1.0 / dl) : Vec2(1, 0);
        const engine::roads::WalkStep& s0 = a.route.steps[static_cast<std::size_t>(a.leg)];
        r.side = static_cast<float>(dot(a.pos - G.pointAt(s0.walk, cur), Vec2(dir.y, -dir.x)));
        r.kind = kPedKind;
        r.live = 1;
        crowdRec_[static_cast<std::size_t>(slot)] = r;
        crowdDirtySlots_.push_back(static_cast<uint32_t>(slot));
    };
    if (all) for (int i = 0; i < static_cast<int>(agents.size()); ++i) refresh(i);
    else for (int i : crowdChanges_) if (i >= 0 && i < static_cast<int>(agents.size())) refresh(i);
    // the arena: compacted when its garbage outweighs what is live (every record's route re-laid, every record sent)
    bool relaid = false;
    if (crowdGarbage_ > 1000000 && crowdGarbage_ * 2 > crowdArena_.size() / 2) {
        std::vector<uint32_t> fresh;
        fresh.reserve(crowdArena_.size() / 2);
        for (engine::Renderer::CrowdAgent& r : crowdRec_) {
            if (!r.live) continue;   // (its span keeps its base leg: the entries move together)
            const uint32_t off = static_cast<uint32_t>(fresh.size() / 2);
            fresh.insert(fresh.end(), crowdArena_.begin() + r.routeOff * 2, crowdArena_.begin() + (r.routeOff + r.routeLen) * 2);
            r.routeOff = off;
        }
        crowdArena_.swap(fresh);
        crowdGarbage_ = 0;
        relaid = true;
    }
    if (relaid || crowdArena_.size() / 2 > arenaBefore) ctx.renderer.setCrowdRoutes(crowdArena_, relaid ? 0 : arenaBefore);
    if (relaid || all || !crowdDirtySlots_.empty()) ctx.renderer.setCrowdAgents(crowdRec_, crowdDirtySlots_, relaid || all);
    ctx.renderer.setCrowdClock(static_cast<float>(sim_.seconds()));
    crowdDrawnSeen_ = static_cast<int>(ctx.renderer.crowdDrawn());
}

void CityRenderSystem::listSignedStaff() {
    if (signedStaffOf_ == sim_.agents().size() && !signedStaff_.empty()) return;
    const auto& ag = sim_.agents();
    signedStaff_.clear();
    for (std::size_t i = 0; i < ag.size(); ++i) {
        const PlaceId w = ag[i].workPlace;
        if (w != kNoPlace && w < static_cast<PlaceId>(signedPlace_.size()) && w < static_cast<PlaceId>(places_.size()) && signedPlace_[w])
            signedStaff_.push_back(static_cast<int>(i));
    }
    signedAtWork_.resize(signedStaff_.size());
    for (std::size_t k = 0; k < signedStaff_.size(); ++k)   // all of them once, so the first count is whole
        signedAtWork_[k] = sim_.atWorkNow(ag[static_cast<std::size_t>(signedStaff_[k])]) ? 1 : 0;
    shiftCursor_ = 0;
    signedStaffOf_ = ag.size();
}

void CityRenderSystem::dealOpenSigns(World& world, Vec3 cam) {
    Real nearD = 1e30;
    Vec3 nearAt, nearN;
    std::vector<uint16_t> hired;
    if (signedPlace_.size() != places_.size()) {   // once: which places have a sign (the rest need nobody counted)
        signedPlace_.assign(places_.size(), 0);
        world.each<engine::OpenSigns>([&](Entity, engine::OpenSigns& os) {
            for (int ai : os.place) {
                const PlaceId pid = ai >= 0 && ai < static_cast<int>(placeOfAuthored_.size()) ? placeOfAuthored_[static_cast<std::size_t>(ai)] : kNoPlace;
                if (pid != kNoPlace && pid < static_cast<PlaceId>(signedPlace_.size())) signedPlace_[pid] = 1;
            }
        });
    }
    // WHO IS ON SHIFT, from the signed places' staff (listed once per population: places are assigned at build) and
    // their at-work flags, which update() refreshes a thirtieth at a time -- walking the island's 238k agents and
    // taking 7k schedule snapshots here, every half second, was an 8 ms frame twice a second (a stutter)
    listSignedStaff();
    std::vector<int> staff;
    hired.assign(places_.size(), 0);
    onShift_.assign(places_.size(), 0);
    for (std::size_t k = 0; k < signedStaff_.size(); ++k) {
        const int ai = signedStaff_[k];
        const PlaceId w = sim_.agents()[static_cast<std::size_t>(ai)].workPlace;
        if (hired[w] < 0xFFFF) ++hired[w];
        if (signedAtWork_[k] && onShift_[w] < 0xFFFF) ++onShift_[w];
        if (w == signWatch_) staff.push_back(ai);
    }
    PlaceId nearPid = kNoPlace;
    const Real clock = sim_.clockHours();
    int signs = 0, lit = 0, closed = 0, unstaffed = 0, noHire = 0;
    world.each<engine::OpenSigns, InstanceGroup>([&](Entity, engine::OpenSigns& os, InstanceGroup& g) {
        bool changed = false;
        for (std::size_t k = 0; k < os.at.size(); ++k) {
            const int ai = os.place[k];
            const PlaceId pid = ai >= 0 && ai < static_cast<int>(placeOfAuthored_.size()) ? placeOfAuthored_[static_cast<std::size_t>(ai)] : kNoPlace;
            bool on = false;
            if (pid != kNoPlace && pid < static_cast<PlaceId>(onShift_.size())) {
                const bool open = places_[pid].openAt(clock), staffed = onShift_[pid] > 0;
                on = open && staffed;
                closed += !open;
                unstaffed += open && !staffed;
                noHire += open && !staffed && hired[pid] == 0;
            }
            ++signs;
            lit += on;
            const uint8_t v = on ? 1 : 0;
            if (os.lit[k] != v) { os.lit[k] = v; changed = true; }
            if (on) {
                const Vec3 at(os.at[k].m[0][3], os.at[k].m[1][3], os.at[k].m[2][3]);
                if ((at - cam).length() < nearD) { nearD = (at - cam).length(); nearAt = at; nearN = Vec3(os.at[k].m[0][2], 0, os.at[k].m[2][2]); nearPid = pid; }
            }
        }
        if (!changed) return;
        InstanceGroup* dark = world.get<InstanceGroup>(os.dark);
        g.transforms.clear();
        if (dark) dark->transforms.clear();
        for (std::size_t k = 0; k < os.at.size(); ++k) {
            if (os.lit[k]) g.transforms.push_back(os.at[k]);
            else if (dark) dark->transforms.push_back(os.at[k]);
        }
    });
    // the nearest lit sign's place and its staff (as of the last deal): where each one is and whether the sim has them
    // at work -- the answer to "the sign is lit but nobody's in there"
    std::string who;
    if (signWatch_ != kNoPlace && signWatch_ < places_.size()) {
        const Place& wp = places_[signWatch_];
        int drawnFor = 0;   // stepIndoors' buildings that draw THIS place's people
        for (const auto& kv : buildingPlace_) {
            drawnFor += kv.second.whole == signWatch_;
            for (PlaceId q : kv.second.unit) drawnFor += q == signWatch_;
        }
        char h[256];
        std::snprintf(h, sizeof(h), " | place %u %s \"%s\" door %.1f %.1f, %d streamed buildings draw its people; staff:",
                      static_cast<unsigned>(signWatch_), placeTypeName(wp.type), wp.name.c_str(), wp.door.x, wp.door.y, drawnFor);
        who = h;
        const Vec2 restAt = wp.entrance + (wp.site - wp.entrance) * 0.4;
        for (int ai : staff) {
            const Agent& a = sim_.agents()[static_cast<std::size_t>(ai)];
            char b[160];
            std::snprintf(b, sizeof(b), " %d[tier %d act %d%s%s at %.0f %.0f, %.1f m from rest]", ai, static_cast<int>(a.tier),
                          static_cast<int>(a.activity), a.indoors ? " in" : "", a.moving ? " moving" : "", a.pos.x, a.pos.y, (a.pos - restAt).length());
            if (who.size() < 1500) who += b;
        }
    }
    signWatch_ = nearPid;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "nearest lit %.1f %.1f %.1f facing %.2f %.2f (%.0f m); %d OPEN signs: %d lit, %d closed for the night, %d open but nobody on shift (%d with nobody hired) (%.2f h)",
                  nearAt.x, nearAt.y, nearAt.z, nearN.x, nearN.z, nearD < 1e29 ? nearD : -1.0, signs, lit, closed, unstaffed, noHire, clock);
    answers_.set("opensigns?", buf + who);
}

void CityRenderSystem::measureJitter(engine::FrameContext& ctx) {
    for (const engine::JitterRequest& req : jitterRequests_) {
        int id = req.agent;
        const int frames = req.frames;
        const Vec3 cam = ctx.view.camera.position;
        if (id < 0) {   // the nearest moving walker (the cars are entities, which `jitter` does not follow yet)
            Real best = 80.0 * 80.0;
            const auto& ag = sim_.agents();
            for (std::size_t i = 0; i < ag.size(); ++i) {
                const Agent& a = ag[i];
                if (a.speed < 0.5 || a.released || !sim_.pedVisible(static_cast<int>(i))) continue;
                const Real dx = a.pos.x - cam.x, dz = a.pos.y - cam.z;
                if (dx * dx + dz * dz < best) { best = dx * dx + dz * dz; id = static_cast<int>(i); }
            }
        }
        jitterAgent_ = id;
        jitterFrames_ = std::max(10, frames);
        jitter_.clear();
        answers_.set("jitter?", id < 0 ? "no moving walker near" : "recording agent " + std::to_string(id));
    }
    jitterRequests_.clear();
    if (jitterAgent_ < 0 || jitterAgent_ >= static_cast<int>(sim_.agents().size())) return;
    const Agent& a = sim_.agents()[static_cast<std::size_t>(jitterAgent_)];
    // where the RENDERER draws its body this frame: the baked instance, between its last step and this one by the
    // frame's interpolation when the group names its instances (InstanceGroup::ids), else as baked
    const uint64_t want = movingId(a, 0);
    bool found = false;
    Mat4 drawnM;
    Vec3 p(a.pos.x, 0, a.pos.y);
    std::vector<Entity> drawn = carGroups_;
    drawn.push_back(pedGroup_);
    for (Entity e : drawn) {
        const InstanceGroup* g = ctx.world.get<InstanceGroup>(e);
        if (!g) continue;
        for (std::size_t i = 0; i < g->ids.size() && i < g->transforms.size(); ++i) {
            if (g->ids[i] != want) continue;
            const Mat4& cur = g->transforms[i];
            p = Vec3(cur.m[0][3], 0, cur.m[2][3]);
            drawnM = cur;
            static const bool noLerp = std::getenv("RT_NO_INSTANCE_LERP") != nullptr;   // (as the renderer draws it)
            for (std::size_t j = 0; !noLerp && j < g->prevIds.size() && j < g->prevTransforms.size(); ++j)
                if (g->prevIds[j] == want) {
                    const Mat4& pr = g->prevTransforms[j];
                    const Real al = ctx.interpolation;
                    p = Vec3(pr.m[0][3] + (cur.m[0][3] - pr.m[0][3]) * al, 0, pr.m[2][3] + (cur.m[2][3] - pr.m[2][3]) * al);
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 4; ++c) drawnM.m[r][c] = pr.m[r][c] + (cur.m[r][c] - pr.m[r][c]) * al;
                    break;
                }
            found = true;
            break;
        }
        if (found) break;
    }
    // the camera in the vehicle's own drawn frame (the rotation's columns are its axes): what a rider sees move
    const Vec3 rel = ctx.view.camera.position - Vec3(drawnM.m[0][3], drawnM.m[1][3], drawnM.m[2][3]);
    Vec3 inV;
    for (int c = 0; c < 3; ++c) {
        const Vec3 axis(drawnM.m[0][c], drawnM.m[1][c], drawnM.m[2][c]);
        const Real len2 = std::max(axis.lengthSquared(), Real(1e-12));
        (c == 0 ? inV.x : c == 1 ? inV.y : inV.z) = dot(rel, axis) / len2;
    }
    jitter_.push_back({p, ctx.view.camera.position, ctx.frameDelta, inV});
    if (static_cast<int>(jitter_.size()) < jitterFrames_) return;
    // per frame: how far it moved over how long the frame was -- an even glide is a steady speed every frame;
    // a shake is speeds jumping about (a frame that moved it nothing, then one that moved it double)
    auto stats = [&](auto pick, Real& mean, Real& cv, int& still) {
        std::vector<Real> v;
        for (std::size_t i = 1; i < jitter_.size(); ++i) {
            const Real dt = std::max(jitter_[i].dt, Real(1e-4));
            const Vec3 d = pick(jitter_[i]) - pick(jitter_[i - 1]);
            v.push_back(std::sqrt(d.x * d.x + d.z * d.z) / dt);
        }
        mean = 0;
        for (Real x : v) mean += x;
        mean /= std::max<std::size_t>(1, v.size());
        Real var = 0;
        still = 0;
        for (Real x : v) { var += (x - mean) * (x - mean); still += x < mean * 0.25 ? 1 : 0; }
        cv = mean > 1e-6 ? std::sqrt(var / std::max<std::size_t>(1, v.size())) / mean : 0;
    };
    Real vm, vcv, cm, ccv, rm, rcv;
    int vs, cs, rs;
    stats([](const JitterSample& s) { return s.at; }, vm, vcv, vs);
    stats([](const JitterSample& s) { return s.cam; }, cm, ccv, cs);
    stats([](const JitterSample& s) { return s.cam - s.at; }, rm, rcv, rs);   // the camera against the vehicle
    // THE SHAKE ITSELF: frame-to-frame jerk of the camera against the vehicle -- the second difference of their
    // offset, which a steady ride (however it turns or brakes, a frame at a time) keeps near zero, mm
    Real jerk = 0, vjerk = 0;
    for (std::size_t i = 2; i < jitter_.size(); ++i) {
        const Vec3 r0 = jitter_[i - 2].cam - jitter_[i - 2].at, r1 = jitter_[i - 1].cam - jitter_[i - 1].at,
                   r2 = jitter_[i].cam - jitter_[i].at;
        const Vec3 j = r2 - r1 * 2.0 + r0;
        jerk += j.x * j.x + j.z * j.z;
        const Vec3 vj = jitter_[i].at - jitter_[i - 1].at * 2.0 + jitter_[i - 2].at;
        vjerk += vj.x * vj.x + vj.z * vj.z;
    }
    Real inJerk = 0;   // the camera's jerk in the vehicle's own frame: the shake a rider sees, mm
    for (std::size_t i = 2; i < jitter_.size(); ++i) {
        const Vec3 j = jitter_[i].inVehicle - jitter_[i - 1].inVehicle * 2.0 + jitter_[i - 2].inVehicle;
        inJerk += j.lengthSquared();
    }
    const Real nj = static_cast<Real>(std::max<std::size_t>(1, jitter_.size() - 2));
    inJerk = std::sqrt(inJerk / nj) * 1000.0;
    jerk = std::sqrt(jerk / nj) * 1000.0;
    vjerk = std::sqrt(vjerk / nj) * 1000.0;
    Real dtMean = 0;
    for (const JitterSample& s : jitter_) dtMean += s.dt;
    dtMean /= static_cast<Real>(jitter_.size());
    char b[512];
    std::snprintf(b, sizeof b,
                  "agent %d over %zu frames (%.1f ms/frame, sim tick %.1f ms): vehicle %.2f m/s cv %.2f still %d | "
                  "camera %.2f m/s cv %.2f still %d | camera-vs-vehicle %.3f m/s, jerk %.1f mm (vehicle's own %.1f mm), "
                  "in the vehicle's frame %.2f mm",
                  jitterAgent_, jitter_.size(), dtMean * 1000.0, sim_.tickPeriod() * 1000.0, vm, vcv, vs, cm, ccv, cs, rm,
                  jerk, vjerk, inJerk);
    answers_.set("jitter?", b);
    jitterAgent_ = -1;
    jitter_.clear();
}

void CityRenderSystem::update(engine::FrameContext& ctx) {
    feedGpuCrowd(ctx);
    // the OPEN signs' staff: a thirtieth of them asked "at work?" each frame (dealOpenSigns counts the flags)
    if (built_ && !signedStaff_.empty()) {
        const std::size_t n = (signedStaff_.size() + 29) / 30;
        for (std::size_t j = 0; j < n; ++j) {
            if (shiftCursor_ >= signedStaff_.size()) shiftCursor_ = 0;
            signedAtWork_[shiftCursor_] = sim_.atWorkNow(sim_.agents()[static_cast<std::size_t>(signedStaff_[shiftCursor_])]) ? 1 : 0;
            ++shiftCursor_;
        }
    }
    // Stage the sun for the bake's car lamps (see solarElevation_).
    solarElevation_ = ctx.view.lighting.solarElevation;
    solarStaged_ = true;
    ctx.view.lights.channel(engine::LightChannel::Traffic) = trafficLights_;   // (ADR-0362; picked by LightSystem)
    // (The parked fire engines' ladders went with the fire engines, which were rows of the old driving's vehicle list:
    // ADR-0461.)
    // Per-frame so the key edge is never missed by the fixed-step tick.
    if (ctx.actions.pressed("agent_widgets")) debugWidgets_ = !debugWidgets_;
    // Semicolon flips the city-plan layer (blocks + lots) — and switches the
    // master on when it was off, so the key works standalone on web (no ImGui
    // panel there).
    if (ctx.actions.pressed("plan_widgets")) {
        showPlan_ = !showPlan_;
        if (showPlan_) debugWidgets_ = true;
    }
    // `overlay <layer>` and the web debug panel (rt_web_city): CityLayerRequests
    // set the same flags the J and ; keys flip, so the page's checkboxes and the
    // keys share them.
    for (const engine::CityLayerRequest& r : layerRequests_) {
        if (r.layer == "master") debugWidgets_ = r.on;
        else if (r.layer == "agents") showAgentWidgets_ = r.on;
        else if (r.layer == "cones") showVisionCones_ = r.on;
        else if (r.layer == "nav") showNavGraph_ = r.on;
        else if (r.layer == "plan") showPlan_ = r.on;
    }
    layerRequests_.clear();

    // Ground probe (channel `ground? x z`, the seam-diagnosis tool): answer
    // with the carved field, the lifted groundAt, and the nearest nav link's
    // deck height at the projected point — the numbers that separate "the
    // terrain is wrong" from "the road is wrong" from "the pad is wrong".
    {
        // (the counts below walk every agent on the island: kept only while a probe is asking -- see answers_: any of
        // this system's questions within the last minute. The first question after a quiet spell answers from the last
        // count. `parking?`, the old driving's bays and garage stalls, went with it: ADR-0462.)
        const bool asked = answers_.secondsSinceAsked() < 60.0;
        if (++parkingTick_ % 60 == 0 && asked) {
            const Vec2 cam(ctx.view.camera.position.x, ctx.view.camera.position.z);
            // `traffic?` (the city's next ten: "the streets look empty"): the people within 300 m of the camera -- on
            // foot, indoors, in the full sim -- and the piles. (The old driving's cars, counted here until ADR-0461, are
            // gone; the driving's cars are `driving?`'s.)
            {
                int nearPeds = 0, nearIndoors = 0, nearK = 0;
                const auto& ag = sim_.agents();
                for (std::size_t i = 0; i < ag.size(); ++i) {
                    const Agent& a = ag[i];
                    if ((a.pos - cam).lengthSquared() > 300.0 * 300.0) continue;
                    if (a.tier == Agent::Tier::K) ++nearK;
                    if (a.indoors) ++nearIndoors;
                    else ++nearPeds;
                }
                // PILES (Glenn: "all the agents and vehicles are starting in one place... on top of one another like
                // it's some spawn point"): drawn near K agents by 1.5 m cell -- how many stand in a cell with 3+ others,
                // the biggest pile, and who is in it
                std::unordered_map<int64_t, std::vector<int>> pile;
                for (int nearIdx : sim_.nearAgents()) {
                    const Agent& a = ag[static_cast<std::size_t>(nearIdx)];
                    if (a.far() || a.released || !sim_.pedVisible(nearIdx)) continue;
                    if ((a.pos - cam).lengthSquared() > 300.0 * 300.0) continue;
                    const int64_t cx = static_cast<int64_t>(std::floor(a.pos.x / 1.5)), cz = static_cast<int64_t>(std::floor(a.pos.y / 1.5));
                    pile[(cx << 32) ^ (cz & 0xffffffff)].push_back(nearIdx);
                }
                int piled = 0;
                const std::vector<int>* worst = nullptr;
                for (const auto& [k, v] : pile) {
                    if (v.size() >= 4) piled += static_cast<int>(v.size());
                    if (!worst || v.size() > worst->size()) worst = &v;
                }
                std::string pileWho;
                if (worst && worst->size() >= 4) {
                    int mov = 0;
                    std::map<int, int> homes, acts;
                    for (int i : *worst) {
                        const Agent& a = ag[static_cast<std::size_t>(i)];
                        mov += a.moving;
                        ++homes[a.home];
                        ++acts[static_cast<int>(a.activity)];
                    }
                    const Agent& a0 = ag[static_cast<std::size_t>(worst->front())];
                    char pb[256];
                    std::snprintf(pb, sizeof(pb), "; piled 4+ to a 1.5 m spot: %d agents, the worst %zu at %.1f,%.1f (%d moving, %zu homes, acts",
                                  piled, worst->size(), a0.pos.x, a0.pos.y, mov, homes.size());
                    pileWho = pb;
                    for (const auto& [k, n] : acts) pileWho += " " + std::to_string(k) + ":" + std::to_string(n);
                    pileWho += ")";
                    if (std::getenv("RT_TRAFFIC_DEBUG"))
                        for (std::size_t q = 0; q < worst->size() && q < 6; ++q) {
                            const int i = (*worst)[q];
                            const Agent& a = ag[static_cast<std::size_t>(i)];
                            const bool on = CitySim::onRoute(a);
                            const int walk = on ? static_cast<int>(a.route.steps[static_cast<std::size_t>(a.leg)].walk.index) : -1;
                            const int next = on && a.leg + 1 < static_cast<int>(a.route.steps.size())
                                                 ? static_cast<int>(a.route.steps[static_cast<std::size_t>(a.leg + 1)].walk.index) : -1;
                            char qb[260];
                            std::snprintf(qb, sizeof(qb), " {%d st %d v %.2f step %d/%zu walk %d len %.1f d %.1f next walk %d why %d gap %.2f/%.2f}",
                                          i, static_cast<int>(a.state), a.speed, a.leg, a.route.steps.size(), walk,
                                          on ? sim_.legLength(a, a.leg) : -1.0, a.distOnLeg, next,
                                          sim_.holdWhyOf(i), std::min(sim_.gapOf(i), Real(99)), sim_.minGapOf(i));
                            pileWho += qb;
                        }
                } else {
                    pileWho = "; piled 4+ to a 1.5 m spot: " + std::to_string(piled);
                }
                char tb[256];
                std::snprintf(tb, sizeof(tb), "within 300 m: %d out of doors, %d indoors, %d in tier K; sleeper waypoints %ld",
                              nearPeds, nearIndoors, nearK, sim_.dormantWaypoints());
                // ...and the riders astride (a motorcycle's): how many are drawn, the nearest one's spot -- a probe aims there
                int riders = 0; Vec3 riderAt(0, 0, 0); Real riderD = 1e30;
                for (Entity re : carRiderGroups_)
                    if (const InstanceGroup* rg = re.valid() ? ctx.world.get<InstanceGroup>(re) : nullptr)
                        for (const Mat4& m : rg->transforms) {
                            ++riders;
                            const Vec3 at(m.m[0][3], m.m[1][3], m.m[2][3]);
                            const Real d = (Vec2(at.x, at.z) - cam).length();
                            if (d < riderD) { riderD = d; riderAt = at; }
                        }
                char rb[128];
                std::snprintf(rb, sizeof(rb), "; riders astride %d drawn, nearest %.1f %.1f %.1f (%.0f m)", riders, riderAt.x, riderAt.y,
                              riderAt.z, riders ? riderD : -1.0);
                answers_.set("traffic?", std::string(tb) + pileWho + rb);
            }
        }
        // THE OPEN SIGNS (storefronts stage 3): once a second, every window's sign lit while its place is open and
        // someone who works there is at work; `opensigns?` says how many
        if (built_ && parkingTick_ % 30 == 0) dealOpenSigns(ctx.world, ctx.view.camera.position);
        // `pin <id>` / `unpin <id>` (FOLLOW an agent: in the full sim wherever it goes)
        // `findplace <name>`: the places whose name contains it -- where, what, open now, and who is in at its door
        {
            const std::string fq = findPlace_;
            if (!fq.empty()) {
                findPlace_.clear();
                std::string res;
                const Real h = sim_.clockHours();
                for (const Place& p : places_.places()) {
                    if (p.name.find(fq) == std::string::npos || res.size() > 1200) continue;
                    const Vec2 rest = p.entrance + (p.site - p.entrance) * 0.4;
                    int in = 0, staff = 0;
                    for (int ni : sim_.nearAgents()) {
                        const Agent& a = sim_.agents()[static_cast<std::size_t>(ni)];
                        if (!a.indoors || a.moving || (a.pos - rest).lengthSquared() >= 0.25) continue;
                        ++in;
                        staff += a.workPlace == p.id && a.activity == Agent::Activity::AtWork;
                    }
                    char b[256];
                    std::snprintf(b, sizeof(b), "%s%u %s \"%s\" site %.1f %.1f door %.1f %.1f %s (%.1f-%.1f h), %d in (%d staff; near only)",
                                  res.empty() ? "" : "; ", static_cast<unsigned>(p.id), placeTypeName(p.type), p.name.c_str(), p.site.x, p.site.y,
                                  p.door.x, p.door.y, p.openAt(h) ? "open" : "closed", p.openHour, p.closeHour, in, staff);
                    res += b;
                }
                answers_.set("findplace?", res.empty() ? "no place named like that" : res);
            }
        }
        {
            std::vector<engine::PinRequest> pins;
            pins.swap(pinRequests_);
            for (const engine::PinRequest& pr : pins) {
                const int id = pr.agent;
                if (id >= 0 && id < static_cast<int>(sim_.agents().size())) {
                    sim_.pinAgent(id, pr.on);
                    answers_.set("pin?", std::to_string(sim_.pinnedCount()) + " followed; last " + std::to_string(id) +
                                             (pr.on ? " pinned" : " unpinned"));
                }
            }
        }
        const std::string q = groundQuery_;
        if (!q.empty()) {
            groundQuery_.clear();
            double qx = 0, qz = 0, qr = 0;
            const int nq = std::sscanf(q.c_str(), "%lf %lf %lf", &qx, &qz, &qr);
            if (nq == 3 && qr > 0) {   // `ground? x z r`: dump nav links + deck spines within r to RT_GROUND_DUMP (JSON)
                const char* path = std::getenv("RT_GROUND_DUMP");
                if (FILE* fp = path ? std::fopen(path, "w") : nullptr) {
                    auto near = [&](engine::Vec2 p) { return (p - engine::Vec2(qx, qz)).length() < qr; };
                    std::fprintf(fp, "{\"links\":[");
                    bool first = true;
                    for (int li = 0; li < nav_.linkCount(); ++li) {
                        const engine::NavLink& L = nav_.links[static_cast<std::size_t>(li)];
                        if (L.from < 0 || L.to < 0) continue;
                        const engine::Vec2 a = nav_.nodes[static_cast<std::size_t>(L.from)], b = nav_.nodes[static_cast<std::size_t>(L.to)];
                        if (!near(a) && !near(b)) continue;
                        std::fprintf(fp, "%s{\"id\":%d,\"a\":[%.2f,%.2f],\"b\":[%.2f,%.2f],\"abs\":[%d,%d],\"elev\":[%.2f,%.2f],\"cls\":%d}",
                                     first ? "" : ",", li, a.x, a.y, b.x, b.y, L.elevAbsA ? 1 : 0, L.elevAbsB ? 1 : 0, L.elevA, L.elevB,
                                     static_cast<int>(L.klass));
                        first = false;
                    }
                    std::fprintf(fp, "],\"spines\":[");
                    first = true;
                    for (const engine::RoadDeckField& f : decks_)
                        for (const engine::UnionSpine& sp : f.spines) {
                            bool any = false;
                            for (const engine::Vec2& p : sp.points) any = any || near(p);
                            if (!any) continue;
                            std::fprintf(fp, "%s{\"layer\":%d,\"pts\":[", first ? "" : ",", sp.layer);
                            for (std::size_t i = 0; i < sp.points.size(); ++i)
                                std::fprintf(fp, "%s[%.2f,%.2f,%.2f,%.2f]", i ? "," : "", sp.points[i].x, sp.points[i].y,
                                             i < sp.yAbs.size() ? sp.yAbs[i] : 0.0, i < sp.hw.size() ? sp.hw[i] : sp.halfWidth);
                            std::fprintf(fp, "]}");
                            first = false;
                        }
                    // (the old driving's cars, drawn vs the decks under them, were listed here until ADR-0461; a car
                    // entity's tyres against the road are wheelGaps')
                    std::fprintf(fp, "]}\n");
                    std::fclose(fp);
                    groundResult_ = std::string("dumped ") + path;
                } else {
                    groundResult_ = "err set RT_GROUND_DUMP to a writable path";
                }
            } else if (nq >= 2) {
                const double carved = heightAt_ ? heightAt_(qx, qz) : 0.0;
                char buf[512];
                std::snprintf(buf, sizeof buf, "%.1f %.1f carved=%.2f lifted=%.2f drawn=%.2f", qx, qz, carved,
                              carved + roadLift_, groundAt(qx, qz));
                std::string out = buf;
                // every drawn surface over the point (which deck, at what height)
                for (const engine::RoadDeckField& f : decks_) {
                    double y;
                    if (f.heightAt(qx, qz, 0.5, &y)) { std::snprintf(buf, sizeof buf, " deckAt=%.2f", y); out += buf; }
                }
                {   // the nearest drawn deck spine within 40 m (is this road in the field at all, how wide)
                    std::size_t nSp = 0;
                    double bd = 40.0, by = 0, bhw = 0; int bl = -1;
                    for (const engine::RoadDeckField& f : decks_) {
                        nSp += f.spines.size();
                        for (const engine::UnionSpine& sp : f.spines)
                            for (std::size_t i = 0; i + 1 < sp.points.size(); ++i) {
                                const engine::Vec2 a = sp.points[i], ab = sp.points[i + 1] - a;
                                const double L2 = ab.lengthSquared();
                                double t = L2 < 1e-12 ? 0.0 : dot(engine::Vec2(qx, qz) - a, ab) / L2;
                                t = std::clamp(t, 0.0, 1.0);
                                const double d = (a + ab * t - engine::Vec2(qx, qz)).length();
                                if (d < bd && sp.yAbs.size() == sp.points.size()) {
                                    bd = d; bl = sp.layer;
                                    by = sp.yAbs[i] + (sp.yAbs[i + 1] - sp.yAbs[i]) * t;
                                    bhw = sp.hw.size() == sp.points.size() ? sp.hw[i] : sp.halfWidth;
                                }
                            }
                    }
                    std::snprintf(buf, sizeof buf, " | decks=%zu spines=%zu nearestSpine d=%.2f y=%.2f hw=%.2f layer=%d",
                                  decks_.size(), nSp, bl >= 0 ? bd : -1.0, by, bhw, bl);
                    out += buf;
                }
                // the nearest nav link, both height forms
                const int li = nav_.nearestLink(engine::Vec2(qx, qz));
                if (li >= 0) {
                    const engine::NavLink& L = nav_.links[li];
                    const engine::Vec2 a = nav_.nodes[L.from], b = nav_.nodes[L.to];
                    const engine::Vec2 ab(b.x - a.x, b.y - a.y);
                    const double len2 = ab.x * ab.x + ab.y * ab.y;
                    const double t = len2 > 1e-9 ? std::clamp(((qx - a.x) * ab.x + (qz - a.y) * ab.y) / len2, 0.0, 1.0) : 0.0;
                    std::snprintf(buf, sizeof buf,
                                  " | link=%d t=%.2f len=%.1f abs=%d(%d,%d) layer=%d elev=%.2f..%.2f above=%.2f..%.2f deckYAt=%.2f",
                                  li, t, L.length, L.elevAbsolute ? 1 : 0, L.elevAbsA ? 1 : 0, L.elevAbsB ? 1 : 0, L.layer,
                                  L.elevA, L.elevB, L.aboveA, L.aboveB,
                                  deckYAt(li, static_cast<engine::Real>(t * L.length), engine::Vec2(qx, qz)));
                    out += buf;
                }
                groundResult_ = out;
            } else {
                groundResult_ = "err parse: " + q;
            }
        }
    }
}

#ifdef RT_ENABLE_IMGUI
namespace {
// Overlay colour per place type (ADR-0066 labels): home green, shop amber, office
// blue, park teal, civic violet — the same hue keys the marker, connector, dot,
// and text so a place reads as one unit.
ImU32 placeColor(PlaceType t) {
    switch (t) {
        case PlaceType::Home:   return IM_COL32( 90, 200, 110, 255);
        case PlaceType::Shop:   return IM_COL32(240, 170,  60, 255);
        case PlaceType::Office: return IM_COL32( 90, 160, 240, 255);
        case PlaceType::Park:   return IM_COL32( 60, 210, 190, 255);
        case PlaceType::Civic:  return IM_COL32(190, 130, 240, 255);
        case PlaceType::Cafe:        return IM_COL32(200, 140,  90, 255);
        case PlaceType::Restaurant:  return IM_COL32(230,  90,  80, 255);
        case PlaceType::Bar:         return IM_COL32(170,  80, 200, 255);
        case PlaceType::Club:        return IM_COL32(240,  60, 200, 255);
        case PlaceType::Supermarket: return IM_COL32(250, 220,  90, 255);
        default:                return IM_COL32(220, 220, 220, 255);
    }
}
// Short label for an Agent::State (the reactive FSM value the ring colours use).
const char* agentStateName(Agent::State s) {
    switch (s) {
        case Agent::State::Resting:   return "Resting";
        case Agent::State::Walking:   return "Walking";
        case Agent::State::Avoiding:  return "Avoiding";
        case Agent::State::Waiting:   return "Waiting";
        case Agent::State::Cruising:  return "Cruising";
        case Agent::State::Following: return "Following";
        case Agent::State::Yielding:  return "Yielding";
        case Agent::State::Turning:   return "Turning";
        default:                      return "?";
    }
}
}  // namespace
#endif

void CityRenderSystem::render(engine::FrameContext& ctx) {
    measureJitter(ctx);   // (here: after every system's update has placed the camera and the rider)
#ifdef RT_ENABLE_IMGUI
    // No ImGui context (a backend without the debug UI): stay inert — same guard
    // the engine's DebugOverlaySystem uses.
    if (ImGui::GetCurrentContext() == nullptr) return;
    // Only draw the panel when the backtick debug overlay is UP — the shared
    // FrameContext flag every always-registered debug panel gates on, so plain
    // play looks like the shipped game. Without this the city forced the "Debug"
    // window open every frame (device: "the debug panel is always on screen").
    if (!ctx.debugOverlayActive) return;
    // Only when a living city is actually loaded: an engine app that happens to
    // register this system (or a level with no roads) shows no city section.
    if (!built_ || sim_.agents().empty()) return;

    // Append into the SHARED Debug window by matching its title (ImGui merges
    // same-titled Begin() calls in a frame). We do not create a second panel.
    ImGui::Begin(kDebugWindowTitle);
    if (ImGui::CollapsingHeader("Living City")) {
        const auto& agents = sim_.agents();
        int inCars = 0, onFoot = 0;
        for (const Agent& a : agents)
            (a.inCar >= 0 ? inCars : onFoot)++;
        ImGui::Text("Time %05.2f h   Agents %d  (%d in cars, %d on foot)",
                    sim_.timeOfDay(), static_cast<int>(agents.size()), inCars, onFoot);

        // Master toggle (mirrors the J key) + per-layer refinements.
        ImGui::Checkbox("Debug widgets (J)", &debugWidgets_);
        ImGui::BeginDisabled(!debugWidgets_);
        ImGui::Indent();
        ImGui::Checkbox("Agent rings + intent", &showAgentWidgets_);
        ImGui::Checkbox("Vision cones", &showVisionCones_);
        ImGui::Checkbox("Nav graph", &showNavGraph_);
        ImGui::Checkbox("City plan: blocks + lots (L)", &showPlan_);
        ImGui::Checkbox("Building colliders (prisms)", &showColliders_);
        ImGui::Unindent();
        ImGui::EndDisabled();

        // Render LAYERS (device: "layers for roads, buildings, simulation ...
        // turn them on or off ... so we can visually debug the terrain
        // underneath"). Each unchecked box hides that whole class of world
        // geometry via the renderer's hidden-layer mask; the terrain, sky and
        // props (layer 0) always draw, so unchecking all three bares the ground.
        ImGui::Separator();
        ImGui::TextUnformatted("Render layers");
        auto layerToggle = [&](const char* label, uint32_t bit) {
            bool shown = !(ctx.renderer.hiddenLayers & bit);
            if (ImGui::Checkbox(label, &shown)) {
                if (shown) ctx.renderer.hiddenLayers &= ~bit;
                else       ctx.renderer.hiddenLayers |= bit;
            }
        };
        ImGui::Indent();
        layerToggle("Roads", engine::LayerRoads);
        layerToggle("Buildings", engine::LayerBuildings);
        layerToggle("Simulation (cars, peds, signals)", engine::LayerSim);
        ImGui::Unindent();

        // Rebuild the road graph (device ask): reseed the road recipe and reload
        // the level, so roads, terrain grading, and buildings all regrow from the
        // fresh graph. The host (ArenaState) does the reseed+reload on the poll.
        if (ImGui::Button("Rebuild road graph (new seed)"))
            rebuildRoadsRequested_ = true;

        // Places (ADR-0066): the level-authored destinations + their type counts.
        ImGui::Separator();
        ImGui::Checkbox("Places (labels + markers)", &showPlaces_);
        if (places_.empty()) {
            ImGui::TextDisabled("  (no authored places in this level)");
        } else {
            for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
                const int n = places_.countOfType(static_cast<PlaceType>(t));
                if (n) ImGui::Text("  %-7s %d", placeTypeName(static_cast<PlaceType>(t)), n);
            }
        }

        // Selected-agent inspector: identity (UID) + schedule + live state.
        ImGui::Separator();
        const int count = static_cast<int>(agents.size());
        if (inspectAgent_ >= count) inspectAgent_ = count - 1;
        ImGui::SliderInt("Inspect agent", &inspectAgent_, -1, count - 1);
        if (inspectAgent_ >= 0) {
            const Agent& a = agents[inspectAgent_];
            ImGui::Text("UID %u   %s   %s", a.uid,
                        a.inCar >= 0 ? "In a car" : "On foot",
                        sim_.roles().nameOf(a.role));
            ImGui::Text("State: %s%s", agentStateName(a.state),
                        a.playerControlled ? "  (player)" : "");
            // Where it lives / works (ADR-0066 Phase 3): real place names.
            auto placeLabel = [&](PlaceId id) -> const char* {
                if (id == kNoPlace || id >= static_cast<PlaceId>(places_.size()))
                    return "—";
                const Place& pl = places_[id];
                return pl.name.empty() ? placeTypeName(pl.type) : pl.name.c_str();
            };
            ImGui::Text("Lives: %s", placeLabel(a.homePlace));
            ImGui::Text("Works: %s", placeLabel(a.workPlace));
            // WHO THEY ARE (people.lua): age and life stage; their household, and how each housemate is theirs
            const PeopleCatalog& folk = sim_.people();
            const RelationshipTable& rel = sim_.relationships();
            const char* stageName = a.stage >= 0 && a.stage < static_cast<int>(folk.stages.size())
                                        ? folk.stages[static_cast<std::size_t>(a.stage)].name.c_str() : "—";
            ImGui::Text("Age %d   %s", a.age, stageName);
            const HouseholdPlan& plan = sim_.households();
            if (a.household >= 0 && a.household < static_cast<int>(plan.households.size())) {
                const Household& hh = plan.households[static_cast<std::size_t>(a.household)];
                const char* typeName = hh.type < folk.households.size() ? folk.households[hh.type].name.c_str() : "hall bed";
                ImGui::Text("Household: %s of %u", typeName, hh.count);
                for (const Agent& o : sim_.agents()) {   // (a scan, only while the inspector is open)
                    if (o.household != a.household || o.uid == a.uid) continue;
                    const int t = rel.tieOf(o.uid, a.uid);   // what they are to this agent
                    ImGui::BulletText("%s: UID %u, %d", t >= 0 && t < static_cast<int>(folk.ties.size())
                                                            ? folk.ties[static_cast<std::size_t>(t)].name.c_str() : "housemate",
                                      o.uid, o.age);
                }
            } else {
                ImGui::Text("Household: none (a lodger)");
            }
            // Surface-level social graph: who else this agent knows.
            const auto& knows = rel.relationsOf(a.uid);
            ImGui::Text("Knows %d", static_cast<int>(knows.size()));
            int shown = 0;
            for (std::size_t k = 0; k < knows.size() && shown < 6; ++k) {
                if (knows[k].second == Relationship::Household) continue;
                ImGui::BulletText("UID %u (%s)", knows[k].first, relationshipName(knows[k].second));
                ++shown;
            }
            ImGui::Text("Depart work %.1f h   home %.1f h", a.departWork, a.departHome);
            ImGui::Text("Pos (%.1f, %.1f)   Speed %.1f m/s", a.pos.x, a.pos.y, a.speed);
        }
    }
    ImGui::End();

    // The place overlay (approach A): project each place to the screen and draw a
    // site ring, an entrance dot, a connector, and a type/name label on the
    // foreground draw list (independent of any window, so it reads over the 3D
    // scene). Perspective view only; an orthographic debug camera would need its
    // own projection, which the gameplay camera never uses.
    if (showPlaces_ && !places_.empty()) {
        const engine::CameraState& cam = ctx.view.camera;
        const Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
        const Mat4 proj = Mat4::perspective(cam.fovDegrees * 3.14159265358979 / 180.0,
                                            cam.aspectRatio, cam.nearPlane, cam.farPlane);
        const Mat4 vp = proj * view;
        const ImGuiIO& io = ImGui::GetIO();
        const Real w = io.DisplaySize.x, h = io.DisplaySize.y;
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        // A generated city has hundreds of places; only annotate the ones near the
        // camera so the overlay stays legible (and cheap). Text drops out sooner
        // than the marker so distant places read as dots, nearby ones as labels.
        const Real kMarkerRange = 120.0, kLabelRange = 55.0;
        for (const Place& p : places_.places()) {
            const Real dx = p.site.x - cam.position.x, dz = p.site.y - cam.position.z;
            const Real dist2 = dx * dx + dz * dz;
            if (dist2 > kMarkerRange * kMarkerRange) continue;
            const bool label = dist2 <= kLabelRange * kLabelRange;
            const Vec3 siteW(p.site.x, groundAt(p.site.x, p.site.y) + 0.1, p.site.y);
            const Vec3 entW(p.entrance.x, groundAt(p.entrance.x, p.entrance.y) + 0.1,
                            p.entrance.y);
            Vec2 sp, ep;
            const bool sv = worldToScreen(vp, siteW, w, h, sp);
            const bool ev = worldToScreen(vp, entW, w, h, ep);
            const ImU32 col = placeColor(p.type);
            if (sv && ev)
                dl->AddLine(ImVec2(sp.x, sp.y), ImVec2(ep.x, ep.y), col, 1.5f);
            if (ev) dl->AddCircleFilled(ImVec2(ep.x, ep.y), 4.0f, col);   // entrance
            if (sv) {
                dl->AddCircle(ImVec2(sp.x, sp.y), 6.0f, col, 0, 2.0f);    // site
                if (label) {
                    const char* nm = p.name.empty() ? placeTypeName(p.type)
                                                    : p.name.c_str();
                    dl->AddText(ImVec2(sp.x + 8, sp.y - 6), col, nm);
                }
            }
        }
    }
#else
    (void)ctx;
#endif
}

void CityRenderSystem::adoptWorldClock(engine::FrameContext& ctx) {
    const auto& lit = ctx.view.lighting;
    if (lit.clockHours < 0.0f) return;                       // no cycle staged
    // A jump's re-seed is drawn by this frame's last fixed step (the bake
    // cadence below); nothing to force here.
    setWorldClock(lit.clockHours, lit.clockHoursPerSecond);
}

bool CityRenderSystem::setWorldClock(Real hour, Real hoursPerSecond) {
    if (hour < 0.0) return false;
    worldClockRate_ = hoursPerSecond;
    if (!built_) {                     // build() opens the day at this hour
        worldClockHour_ = hour;
        return false;
    }
    params_.hoursPerSecond = hoursPerSecond;
    // Equal rates never drift (sky and sim advance per fixed step), so a gap
    // beyond a few in-world minutes — or half a second of clock, at silly
    // rates — IS a sky jump.
    Real gap = std::fabs(hour - sim_.clockHours());
    if (gap > 12.0) gap = 24.0 - gap;
    const Real tolerance = std::max(Real(0.05), hoursPerSecond * Real(0.5));
    if (gap <= tolerance) return false;
    LOG_INFO << "[citysim] clock jump: sky " << hour << " h, sim "
             << sim_.clockHours() << " h — re-seeding the population at "
             << hour << " h";
    sim_.setHoursPerSecond(hoursPerSecond);
    sim_.seedFromSchedule(hour);
    return true;
}

void CityRenderSystem::fixedUpdate(engine::FrameContext& ctx) {
    adoptWorldClock(ctx);   // see worldClockHour_: the sky's clock is the city's
    if (!built_) {
        build(ctx.world, &ctx.assets);   // lazy: retry until the level's roads exist
        return;
    }
    // Feed the live player to the sim so AI cars brake for (and hold short of) it,
    // whether the player is on foot or driving. The player is the entity with a
    // CharacterController under host control (PlayerSystem).
    std::vector<Vec2> obstacles;
    ctx.world.each<engine::Transform, engine::CharacterController, engine::ControlledBy>(
        [&](engine::Entity, engine::Transform& t, engine::CharacterController&, engine::ControlledBy&) {
            obstacles.push_back(Vec2(t.position.x, t.position.z));
        });
    // Every REAL physics Vehicle too (the player's car, commandeered/promoted
    // cars — driven OR abandoned): a promoted car has no planner ghost, so
    // without this ambient traffic plans straight through a car parked across
    // the lane and the kinematic proxies bulldoze the dynamic body.
    std::vector<Real> halves(obstacles.size(), Real(0));   // the player on foot: a person
    ctx.world.each<engine::Transform, engine::Vehicle>(
        [&](engine::Entity, engine::Transform& t, engine::Vehicle& v) {
            obstacles.push_back(Vec2(t.position.x, t.position.z));
            halves.push_back(std::max(Real(1.5), v.config.chassisHalfExtent.z));   // a CAR: held short by its length
        });
    // THE SAME, AS THINGS ON THE ROAD for the new driving's drivers (ADR-0451): the player on foot, and every physics
    // car the driving does not drive (the player's, one left standing) -- a near car it drives it sees by itself
    {
        std::vector<driving::RoadThing> things;
        ctx.world.each<engine::Transform, engine::CharacterController, engine::ControlledBy>(
            [&](engine::Entity, engine::Transform& t, engine::CharacterController&, engine::ControlledBy&) {
                driving::RoadThing th;
                th.id = kPlayerThing;
                th.at = Vec2(t.position.x, t.position.z);
                th.halfAcross = CitySim::kWalkerHalfAcross;
                things.push_back(th);
            });
        uint32_t next = kPlayerThing + 1;
        ctx.world.each<engine::Transform, engine::Vehicle>([&](engine::Entity e, engine::Transform& t, engine::Vehicle& v) {
            if (ctx.world.has<CityNearCar>(e)) return;
            driving::RoadThing th;
            th.id = next++;
            th.at = Vec2(t.position.x, t.position.z);
            th.halfAcross = v.config.chassisHalfExtent.x;
            things.push_back(th);
        });
        sim_.setExternalThings(std::move(things));
    }
    sim_.setExternalObstacles(std::move(obstacles), std::move(halves));

    // The pose bake exists for the RENDERER. When the clock runs several fixed
    // steps in one frame (catching up), only the last bake is ever drawn — the
    // rest are pure waste, and at piedmont scale that was ~4.5 bakes a frame.
    // ...and the step BEFORE it: the renderer draws a moving instance between its last bake and this one by the
    // frame's interpolation (InstanceGroup::ids), which is a fixed step apart only if the step before was baked too.
    // On a catch-up frame that is one bake more; on most frames (one step) nothing changes. The car entities' bodies
    // (CityCarPose: their boxes, the switch into physics) and the walkers' boxes (walkerBoxPoses) are set every step
    // regardless (ADR-0454); only the old sim's cars still have their boxes from the bake (named for later there).
    bakeThisStep_ = ctx.fixedStepIndex + 2 >= ctx.fixedStepCount;

    // ADAPTIVE DIP. When the clock is behind it runs MORE fixed steps to catch
    // up — and if the sim is what made us late, that deepens the hole (the
    // spiral that pinned piedmont at the 8-step cap). Traffic is the one
    // subsystem where catching up COARSELY is legitimate, so a sustained high
    // step count slows the sim's own rate instead. Quantised and hysteretic
    // (a streak, not a single frame) so the tick pattern doesn't flutter with
    // frame-time noise, and only on the frame's first step.
    if (params_.adaptiveRate && ctx.fixedStepIndex == 0) {
        const int steps = ctx.fixedStepCount;
        if (steps >= 5) loadStreak_ = std::min(loadStreak_ + 1, 30);
        else if (steps <= 2) loadStreak_ = std::max(loadStreak_ - 1, -30);
        if (loadStreak_ >= 8 && loadMul_ < 4) { loadMul_ *= 2; loadStreak_ = 0; }
        else if (loadStreak_ <= -8 && loadMul_ > 1) { loadMul_ /= 2; loadStreak_ = 0; }
    }

    step(ctx.world, ctx.clock.fixedStep());
    bakeThisStep_ = true;   // a direct step() call (tests) always bakes
}

bool CityRenderSystem::carEntitySeatsOf(World& world, Entity car, Mat4* pose, std::vector<Vec3>* seats) const {
    const CityCarBody* body = world.get<CityCarBody>(car);
    const CityCarPose* at = world.get<CityCarPose>(car);
    std::vector<Vec3> found;
    if (!body || !at || !seatsOfVariant(parkedVariant(body->slot, body->type), found)) return false;
    if (pose) *pose = at->body;
    if (seats) *seats = found;
    return true;
}

bool CityRenderSystem::seatsOfVariant(int slot, std::vector<Vec3>& seats) const {
    if (slot < 0 || carDriverSeat_.empty()) return false;
    const std::size_t v = static_cast<std::size_t>(slot % static_cast<int>(carDriverSeat_.size()));
    if (!carHasDriver_[v]) return false;
    const Vec3 d = carDriverSeat_[v];
    seats.clear();
    seats.push_back(d);
    seats.push_back(Vec3(-d.x, d.y, d.z));
    // the back bench, where the model has one (its recipe's rear_hip), a seat each side of its middle
    if (v < carHasRear_.size() && carHasRear_[v]) {
        const Vec3 r = carRearHip_[v];
        seats.push_back(Vec3(d.x, r.y, r.z));
        seats.push_back(Vec3(-d.x, r.y, r.z));
    }
    return true;
}

bool CityRenderSystem::wheelGaps(World& world, Entity car, std::vector<Real>& gaps, Mat4* pose,
                                 std::vector<Vec3>* bottoms) const {
    gaps.clear();
    if (bottoms) bottoms->clear();
    const CityCarBody* body = world.get<CityCarBody>(car);
    const CityCarOnLane* lane = world.get<CityCarOnLane>(car);
    const engine::Transform* t = world.get<engine::Transform>(car);
    const engine::PrevTransform* prev = world.get<engine::PrevTransform>(car);
    if (!body || !lane || !t || !prev) return false;
    const int slot = parkedVariant(body->slot, body->type);
    if (slot < 0 || slot >= static_cast<int>(carWheels_.size())) return false;
    // The four-wheel fit itself (carEntityPose without the tilt filter's memory: a pure read of the fit, not the
    // filtered pose the car's CityCarPose holds), and the surface it stands it on: the deck nearest its lane height.
    Real ref = 0;
    const Mat4 m = sim_.poses().carEntityPose(sim_, lane, *t, prev->value, slot, carGroupHalfExtents(), nullptr, bakeDt_, &ref);
    if (pose) *pose = m;
    for (const CarWheel& w : carWheels_[static_cast<std::size_t>(slot)]) {
        const Vec3 c = m.transformPoint(Vec3(w.pos.x, w.pos.y - w.radius, w.pos.z));
        const Real road = deckSurfaceNear(c.x, c.z, ref);
        gaps.push_back(c.y - road);
        if (bottoms) bottoms->push_back(c);
    }
    return !gaps.empty();
}

}  // namespace citysim
