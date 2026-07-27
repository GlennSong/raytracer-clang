#include "city_render.h"

#include "car_lamps.h"                                // lamp decision core (ADR-0065)
#include "screen_project.h"                           // worldToScreen (place labels)
#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/city/road_net.h"
#include "../../engine/procgen/city/road_mesh.h"     // strokeRibbon (closed lot/block outlines)
#include "../../engine/procgen/city/street_kit.h"   // trafficSignalProto, SignalParams
#include "../../log.h"                               // LOG_WARN (place-type validation)
#include "../../renderer/event.h"                    // KeyCode (debug-widget toggle)
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>                                   // Living City debug section
#endif
#ifdef RT_ENABLE_SCRIPTING
#include "scripting/agent_goals.h"      // scripted goal tables (ADR-0064)
#include "scripting/vehicle_body.h"     // scripted fleet bodies (ADR-0065)
#include "../../engine/scripting/script_vm.h"
#include "../../engine/scripting/procgen_bindings.h"
#include "../../engine/scripting/script_modules.h"
#include "../../engine/script_assets.h"
#include "../../log.h"
#endif

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
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
using engine::RoadNet;
using engine::MeshHandle;
using engine::MeshBuilder;

namespace {

Vec3 translationOf(const Mat4& t) { return Vec3(t.m[0][3], t.m[1][3], t.m[2][3]); }

// Debug vision-cone dimensions — MUST match the sim's sensors: senseAhead's
// per-tick driver cone and the walkers' think-cadence kPedVision* constants
// (city_sim.cpp). The wedge draws what the agent can actually see.
constexpr Real kCarConeRange = 18.0, kCarConeHalfAngle = 0.45;
constexpr Real kPedConeRange = 4.5, kPedConeHalfAngle = 1.2;
// Debug tints, dimmer than the state rings so neither view shouts over them.
const Vec3 kNavTint(0.18, 0.32, 0.55);    // faint blue: the road/lane graph
const Vec3 kConeTint(0.55, 0.36, 0.10);   // dim amber: the sensing wedge

// Synthesize a default lamp marker set for a car of body `size` (x=width,
// y=height, z=length; +Z forward) — used for the C++ fallback fleet, a
// scripting-off build, and headless tests where no Lua `lights` markers exist.
// Mirrors vehicles.lua `fleet_car`: head/taillights at the front/rear corners, so
// fallback cars still light up rather than going dark.
std::vector<CityRenderSystem::LampMarker> defaultLampMarkers(Vec3 size) {
    const Real hw = size.x * 0.5, hl = size.z * 0.5;
    const Real ly = -size.y * 0.08;   // lamp band height
    const Real lx = hw - 0.30;        // inset from the corner
    std::vector<CityRenderSystem::LampMarker> out;
    for (Real s : {Real(1), Real(-1)}) {
        const std::string side = s > 0 ? "r" : "l";
        out.push_back({"headlight_" + side, Vec3(s * lx, ly, hl - 0.05)});
        out.push_back({"taillight_" + side, Vec3(s * lx, ly, -hl + 0.05)});
    }
    return out;
}

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

Real CityRenderSystem::groundAt(Real x, Real z) const {
    return (heightAt_ ? heightAt_(x, z) : 0.0) + roadLift_;
}

bool CityRenderSystem::agentWorldPose(int agentId, Vec3& outPos,
                                      Vec2& outHeading) const {
    if (agentId < 0 || agentId >= static_cast<int>(sim_.agents().size()))
        return false;
    const Agent& a = sim_.agents()[agentId];
    if (a.released) return false;   // its physical car is player-driven now

    Real x = a.pos.x, z = a.pos.y;
    Vec2 heading = a.heading;
    // Prefer the REAL body pose when this agent is externally owned (same source
    // the debug widgets ring): the planner ghost legitimately runs ahead/behind.
    // An externally-owned agent with no reported body (released, not yet spawned)
    // has no pose to follow — report failure so the caller cycles on.
    const bool car = a.mode == Agent::Mode::Driver;
    const bool external = car ? carsExternallyOwned_ : pedsExternallyOwned_;
    if (external) {
        const std::vector<ExternalAgentPose>& poses =
            car ? externalCarPoses_ : externalPedPoses_;
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

bool CityRenderSystem::build(World& world, AssetManager* assets) {
    // Level-authored settings (ADR-0063): a CitySimConfig entity — the level's
    // top-level "citysim" block — overrides the constructor params, so each level
    // picks its own population, seed, clock rate, reliability, and whether the
    // agent-state HUD starts on. The agent lab runs 1 car + 1 walker, HUD on.
    world.each<engine::CitySimConfig>([&](Entity, engine::CitySimConfig& c) {
        params_.cars = c.cars;
        params_.pedestrians = c.pedestrians;
        params_.carsPerLaneKm = c.carsPerLaneKm;
        params_.pedsPerKm = c.pedsPerKm;
        params_.seed = c.seed;
        params_.hoursPerSecond = c.hoursPerSecond;
        params_.perceptionReliability = c.perceptionReliability;
        params_.sceneryRadius = c.sceneryRadius;
        params_.tieredAgents = c.tieredAgents;
        params_.wander = c.wander;
        params_.agentScript = c.agentScript;
        params_.vehicleScript = c.vehicleScript;
        debugWidgets_ = c.debugWidgets;
        showPlan_ = showPlan_ || c.showPlan;
        authoredPlaces_ = c.places;   // level-authored destinations (ADR-0066)
    });

    // Merge every RoadNet's constrained graph into one combined graph (a level
    // with several road entities yields one city). Matches TrafficSystem.
    RoadGraph combined;
    heightAt_ = nullptr;
    roadLift_ = 0.0;
    world.each<RoadNet>([&](Entity, RoadNet& net) {
        RoadGraph g = navRoadGraph(net);
        int base = static_cast<int>(combined.nodes.size());
        for (const RoadNode& n : g.nodes) combined.nodes.push_back(n);
        for (RoadEdge e : g.edges) {
            e.a += base;
            e.b += base;
            combined.edges.push_back(e);
        }
        if (!heightAt_ && net.heightAt) heightAt_ = net.heightAt;
        roadLift_ = std::max(roadLift_, static_cast<Real>(net.lift));
    });
    // §10: the LEVEL owns the one road graph — streets + corridor chains
    // welded at load (LevelRoadGraph). When present it replaces the private
    // RoadNet merge above wholesale; the merge remains only so loader-less
    // test worlds (bare RoadNet components) still drive.
    world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph& g) {
        if (!g.graph.edges.empty()) combined = g.graph;
    });
    if (combined.nodes.empty() || combined.edges.empty()) return false;

    nav_ = engine::buildNavGraph(combined);
    if (nav_.linkCount() == 0) return false;

    // Connectivity truth (device: "I don't see the freeway connected yet"):
    // walk the built nav and report whether a car can actually reach the
    // corridor from the streets and come back. If either direction is zero,
    // the weld failed and freeway traffic is impossible — say so loudly.
    {
        int fwLinks = 0, rampLinks = 0, onWelds = 0, offWelds = 0;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            const engine::NavLink& L = nav_.links[li];
            if (L.klass == engine::RoadClass::Freeway) ++fwLinks;
            if (L.klass != engine::RoadClass::Ramp) continue;
            ++rampLinks;
            // a ramp link leaving a node that also has street links = on-weld
            bool fromStreet = false, toStreet = false;
            for (int ol : nav_.outLinks[L.from])
                if (nav_.links[ol].klass != engine::RoadClass::Ramp &&
                    nav_.links[ol].klass != engine::RoadClass::Freeway)
                    fromStreet = true;
            for (int ol : nav_.outLinks[L.to])
                if (nav_.links[ol].klass != engine::RoadClass::Ramp &&
                    nav_.links[ol].klass != engine::RoadClass::Freeway)
                    toStreet = true;
            if (fromStreet) ++onWelds;
            if (toStreet) ++offWelds;
        }
        if (fwLinks > 0)
            LOG_INFO << "[citysim] freeway in nav: " << fwLinks
                     << " carriageway links, " << rampLinks << " ramp links, "
                     << onWelds << " street->ramp welds, " << offWelds
                     << " ramp->street welds"
                     << ((onWelds == 0 || offWelds == 0)
                             ? "  <-- NOT DRIVABLE"
                             : "");
    }

    // Places (ADR-0066): turn the level-authored destinations into a routable
    // PlaceMap now that the nav graph exists (each entrance snaps to a sidewalk).
    // An unrecognized type tag is warned-and-skipped so one typo can't drop the
    // level. Rebuilt fresh each build() so a reload doesn't accumulate places.
    places_ = PlaceMap{};
    for (const engine::AuthoredPlace& ap : authoredPlaces_) {
        PlaceType type;
        if (!parsePlaceType(ap.type, type)) {
            LOG_WARN << "citysim: unknown place type '" << ap.type << "' — skipped";
            continue;
        }
        places_.add(type, Vec2(ap.x, ap.z), nav_, ap.openHour, ap.closeHour, 0,
                    ap.name);
    }

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
        if (carCount < 0)
            carCount = std::clamp(
                static_cast<int>(laneKm * params_.carsPerLaneKm), 6, 400);
        if (pedCount < 0)
            pedCount = std::clamp(
                static_cast<int>(walkKm * params_.pedsPerKm), 6, 400);
        LOG_INFO << "[citysim] density population: " << carCount << " cars ("
                 << laneKm << " lane-km), " << pedCount << " walkers ("
                 << walkKm << " sidewalk-km)";
    }
    sim_.build(nav_, carCount, pedCount, params_.seed);
    sim_.setPerceptionReliability(params_.perceptionReliability);
    sim_.setWander(params_.wander);
    // Three-tier traffic (P4): the level's opt-in. The bubble only engages
    // once a player position is fed each fixed step (see step() below) — the
    // build warm-up therefore runs everything K, exactly as before.
    sim_.tieringEnabled = params_.tieredAgents;
#ifdef RT_ENABLE_SCRIPTING
    // Scripted goal tables (ADR-0064): a level's citysim block may name an
    // agents.lua-style script (level_loader reads its text into the config);
    // its archetype tables replace the built-ins BEFORE the warm-up below, so
    // the whole visible day runs on the scripted goals. Lua runs only here, at
    // load — every per-tick transition executes in C++ from the tables.
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
    sim_.assignPlaces(places_, nav_);

    // Warm up so the city is already ALIVE when the level appears — agents depart
    // and spread onto the roads instead of standing still for the first minute.
    for (int i = 0; i < 400; ++i) sim_.step(0.1, params_.hoursPerSecond);

    {   // Freeway usage check (companion to the weld report above): the welds
        // prove the corridor is REACHABLE; this proves cars actually ROUTE
        // onto it — count drivers whose current link is Freeway/Ramp class.
        // 0 here with welds > 0 means the router never prefers the corridor.
        int onDeck = 0, onRamp = 0, drivers = 0;
        for (const Agent& a : sim_.agents()) {
            if (a.mode != Agent::Mode::Driver || !a.moving) continue;
            ++drivers;
            if (a.leg < 0 || a.leg >= static_cast<int>(a.route.links.size())) continue;
            const engine::RoadClass k = nav_.links[a.route.links[a.leg]].klass;
            // Count DECK and RAMP separately: a driver on a ramp has entered
            // the interchange but may never reach the mainline, so lumping the
            // two hid whether cars actually ride the elevated deck.
            if (k == engine::RoadClass::Freeway) ++onDeck;
            else if (k == engine::RoadClass::Ramp) ++onRamp;
        }
        LOG_INFO << "[freeway-use] " << onDeck << " on deck, " << onRamp
                 << " on ramp, of " << drivers << " moving drivers";
    }

    MeshHandle pedMesh{}, lensMesh{};
    if (assets) {
        pedMesh = assets->acquireMesh(buildPersonMesh(0.0, 0), "city:ped");
        Real e = params_.signalLensSize;
        lensMesh = assets->acquireMesh(MeshBuilder::box(Vec3(e, e, e)), "city:signal");
    }

    // One instance group per fleet slot; each is a body TYPE + SIZE (from the sim's
    // shared fleet table) painted its slot colour, so the traffic is a mixed fleet
    // of sedans, hatchbacks, SUVs, pickups, a van, and a box truck — every NPC car
    // built the same way as (and to scale with) the player's. Drivers spread across
    // the slots by vehicle index.
    // When a CityVehicleSystem owns the cars as real physics Vehicles (ADR-0062),
    // skip the instanced kinematic car bodies entirely — otherwise every car draws
    // twice. The CitySim still runs as the planner; the bridge reads its ghosts.
    carGroups_.clear();
    carLights_.clear();
    if (!carsExternallyOwned_) {
#ifdef RT_ENABLE_SCRIPTING
        // Data-driven fleet bodies (ADR-0065): the citysim block may name a
        // vehicles.lua-style script (level_loader reads its text into the
        // config). Run it ONCE here, at load, and build each variant's instanced
        // mesh from its `vehicle.fleet[slot]` recipe (parts -> vertex-coloured
        // boxes, plus named light attachment markers). ANY failure — no script,
        // a Lua error, a malformed slot — LOG_WARNs and falls back to the C++
        // fleetCarMesh, so the streets are never left empty. Lua runs only here;
        // the baked mesh is plain data (no sim state), so determinism is intact.
        engine::ScriptVM vehVM;
        // Same layering as VehicleSystem's spawn path: mesh.* procgen builders
        // plus the module loader — vehicles.lua begins with
        // `require "vehicle_classes"`, so a bare VM fails its very first line
        // and every city silently fell back to the box fleet ("why are we
        // using the old cars?").
        engine::openProcgenLibrary(vehVM);
        engine::openModuleLoader(vehVM, engine::makeModuleSource(""));
        bool vehScript = assets && !params_.vehicleScript.empty();
        if (vehScript) {
            std::string err;
            if (!vehVM.doString(params_.vehicleScript, &err)) {
                LOG_WARN << "citysim vehicles script: " << err
                         << " (using built-in fleet meshes)";
                vehScript = false;
            }
        }
#endif
        int fleetTriangles = 0;
        for (int v = 0; v < carVariantCount(); ++v) {
            MeshHandle mh{};
            std::vector<LampMarker> lights;
            if (assets) {
                engine::RenderMesh mesh;
                bool scripted = false;
#ifdef RT_ENABLE_SCRIPTING
                if (vehScript) {
                    engine::CarBodyRecipe recipe;
                    std::string err;
                    if (engine::loadFleetCarBody(vehVM, v, recipe, &err)) {
                        mesh = std::move(recipe.mesh);
                        scripted = true;
                        // RETAIN the Lua fleet's light markers (ADR-0065 follow-up):
                        // headlight_l/r + taillight_l/r become the emissive lamps
                        // baked in syncCarLamps. Previously parsed-and-dropped.
                        for (const engine::Attachment& att : recipe.lights)
                            lights.push_back({att.name, att.pos});
                    } else {
                        LOG_WARN << "citysim vehicles fleet[" << v << "]: " << err
                                 << " (using built-in fleet mesh)";
                    }
                }
#endif
                if (!scripted) mesh = fleetCarMesh(v);
                if (scripted) {
                    const int tris = static_cast<int>(mesh.indices.size() / 3);
                    fleetTriangles += tris;
                    // A car that is all wheels and no shell must SHOUT. This
                    // exact skew shipped once: a binary older than the asset
                    // ignored the recipe's `body` mesh, read only its `parts`
                    // (the wheels) and drew headless cars — wheels and lamps
                    // rolling down the street. The retired box fleet was ~168
                    // tris; a real shell is thousands; wheels alone are ~48.
                    if (tris < 100)
                        LOG_ERROR << "[citysim] fleet slot " << v << ": only "
                                  << tris << " triangles — this is not a car "
                                     "body (stale binary vs vehicles.lua, or a "
                                     "recipe with no `body` mesh)";
                }
                mh = assets->acquireMesh(mesh, "city:car" + std::to_string(v));
            }
            // No Lua markers (C++ fallback fleet, scripting off, or headless/no
            // assets): synthesize a default set from the body size so fallback cars
            // still light up. (Skipping lamps for fallback cars was the alternative
            // — see the ADR-0065 follow-up; we synthesize instead.)
            if (lights.empty()) lights = defaultLampMarkers(fleetBodySize(v));
            carLights_.push_back(std::move(lights));

            Entity e = world.create();
            InstanceGroup g;
            g.mesh = mh;
            g.material = carMaterial();
            g.renderLayer = engine::LayerSim;   // debug layer toggle
            world.add<InstanceGroup>(e, g);
            carGroups_.push_back(e);
        }

        // FLEET VERDICT — loud, unmissable, once per build. "Why is the city
        // using the OLD cars?" was asked twice while terminal probes showed
        // the script loading fine; this line settles it from the user's own
        // console. Triangle count is the tell: the retired BOX fleet was ~170
        // tris a slot, a real mesh.car shell is thousands.
        int scriptedCount = 0;
        for (const auto& lights : carLights_)
            if (!lights.empty()) ++scriptedCount;   // markers only from Lua
        if (params_.vehicleScript.empty())
            LOG_ERROR << "[citysim] FLEET: BUILT-IN BOX CARS — no vehicles "
                         "script configured (citysim.vehicles missing?)";
        else if (scriptedCount == 0)
            LOG_ERROR << "[citysim] FLEET: BUILT-IN BOX CARS — vehicles.lua "
                         "loaded but produced 0 scripted bodies";
        else
            LOG_INFO << "[citysim] fleet: " << scriptedCount << "/"
                     << carVariantCount()
                     << " scripted car bodies (vehicles.lua), "
                     << fleetTriangles / std::max(1, scriptedCount)
                     << " tris/car avg";
    }
    pedGroup_ = world.create();
    { InstanceGroup g; g.mesh = pedMesh; g.material = pedMaterial();
      g.renderLayer = engine::LayerSim; world.add<InstanceGroup>(pedGroup_, g); }
    for (int s = 0; s < 3; ++s) {
        signalGroups_[s] = world.create();
        InstanceGroup g;
        g.mesh = lensMesh;
        g.material = signalMaterial(static_cast<SignalState>(s));
        g.renderLayer = engine::LayerSim;   // debug layer toggle
        world.add<InstanceGroup>(signalGroups_[s], g);
    }

    // Car lamp groups (ADR-0065 follow-up): one emissive instance batch per lamp
    // kind, sharing a small lens box. Created unconditionally (they stay empty
    // when cars are externally owned); syncCarLamps pushes the lit lamps each step.
    MeshHandle lampMesh{};
    if (assets)
        lampMesh = assets->acquireMesh(MeshBuilder::box(Vec3(0.28, 0.16, 0.12)),
                                       "city:carlamp");
    headlightGroup_ = world.create();
    { InstanceGroup g; g.mesh = lampMesh; g.material = lampMaterial(Vec3(1.5, 1.45, 1.2));
      g.renderLayer = engine::LayerSim; world.add<InstanceGroup>(headlightGroup_, g); }
    brakeLightGroup_ = world.create();
    { InstanceGroup g; g.mesh = lampMesh; g.material = lampMaterial(Vec3(1.7, 0.06, 0.04));
      g.renderLayer = engine::LayerSim; world.add<InstanceGroup>(brakeLightGroup_, g); }
    turnSignalGroup_ = world.create();
    { InstanceGroup g; g.mesh = lampMesh; g.material = lampMaterial(Vec3(1.7, 0.75, 0.05));
      g.renderLayer = engine::LayerSim; world.add<InstanceGroup>(turnSignalGroup_, g); }

    // Reuse the city's street-kit traffic-signal model: one pole+arm+head
    // assembly per signalled approach, placed on the near-right corner facing the
    // oncoming traffic it governs (same geometry the city generator uses). It is
    // static, so its transforms are baked once here; only the lit lens (above)
    // changes each step.
    signalLinks_.clear();
    SignalController& sc = sim_.signals();
    for (int li = 0; li < nav_.linkCount(); ++li)
        if (sc.hasSignal(li)) signalLinks_.push_back(li);

    // BUILD-TIME furniture (device: "place the stop lights when we build the
    // city ... the simulation should use it"): when the loader published a
    // StreetFurniture plan over the SAME nav build, adopt its pole sites and
    // its already-spawned post group — the sim animates lenses and holds cars,
    // but the city decided where every pole stands.
    siteByLink_.clear();
    siteHasLink_.clear();
    engine::Entity adoptedPosts{};
    world.each<engine::StreetFurniture>([&](Entity, engine::StreetFurniture& f) {
        if (f.navLinkCount != nav_.linkCount() || f.signalPoles.empty()) return;
        siteByLink_.assign(nav_.linkCount(), SignalSite{});
        siteHasLink_.assign(nav_.linkCount(), 0);
        for (const auto& s : f.signalPoles) {
            if (s.link < 0 || s.link >= nav_.linkCount()) continue;
            SignalSite st;
            st.base = s.base;
            st.face = s.face;
            st.side = Vec3(s.face.z, 0, -s.face.x);   // rightOf(face): road centre
            st.yaw = std::atan2(s.face.x, s.face.z);
            siteByLink_[s.link] = st;
            siteHasLink_[s.link] = 1;
        }
        adoptedPosts = f.postGroup;
    });

    if (adoptedPosts.valid()) {
        signalPostGroup_ = adoptedPosts;   // loader owns the geometry
    } else {
        MeshHandle postMesh{};
        if (assets) postMesh = assets->acquireMesh(engine::trafficSignalProto(), "city:signalpost");
        signalPostGroup_ = world.create();
        InstanceGroup g;
        g.mesh = postMesh;
        g.material = signalPostMaterial();
        g.renderLayer = engine::LayerSim;   // debug layer toggle
        for (int li : signalLinks_) g.transforms.push_back(signalPostPose(li));
        refreshBounds(&g);
        world.add<InstanceGroup>(signalPostGroup_, g);
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
        for (const CitySim::ParkingBay& b : sim_.parkingBays()) {
            // Bay paint is PAINT: it rides at the road mesher's own stripe
            // lift (RoadMeshParams::markLift, 2 cm), not a hand-picked 5 cm —
            // which read as a slab hovering over the asphalt.
            const Real y = groundAt(b.pos.x, b.pos.y) + engine::kRoadMarkLift;
            const Real yaw = std::atan2(b.heading.x, b.heading.y);
            const Real bw = b.width > 0 ? b.width : Real(2.2);
            g.transforms.push_back(
                Mat4::trs(Vec3(b.pos.x, y, b.pos.y),
                          Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                          Vec3(bw, 1, 1)));
        }
        refreshBounds(&g);
        world.add<InstanceGroup>(parkBayGroup_, g);
    }

    // Stop bars + lane-turn arrows (R6c, plan 4e): one white PAINT mesh for
    // every signalled approach (buildRoadMarkings), one identity instance.
    {
        engine::RenderMesh paint = buildRoadMarkings();
        MeshHandle mh{};
        if (assets) mh = assets->acquireMesh(paint, "city:roadmarks");
        roadMarkGroup_ = world.create();
        InstanceGroup g;
        g.mesh = mh;
        g.material.albedo = Vec3(1, 1, 1);
        g.material.roughness = 0.92f;
        g.transforms.push_back(Mat4());   // world-space mesh, identity instance
        refreshBounds(&g);
        world.add<InstanceGroup>(roadMarkGroup_, g);
    }

    // Hand the sim the pole foot positions (XZ) as static obstacles, so pedestrians
    // on the sidewalk steer around the signal poles and never stand inside one.
    {
        std::vector<engine::Vec2> poles;
        poles.reserve(signalLinks_.size());
        for (int li : signalLinks_) {
            engine::Vec3 base = signalSite(li).base;
            poles.push_back(engine::Vec2(base.x, base.z));
        }
        sim_.setStaticObstacles(std::move(poles));
    }

    // Crosswalks are painted into the ROAD TEXTURE now (ADR-0062): the road mesher
    // bakes a set-back "metres past the junction mouth" coordinate into the
    // carriageway UV, and the RoadMarkings shader stripes the zebra band there — so
    // it's part of the road surface, set back on the approach, and never a floating
    // decal overlapping the centreline. We keep only the crosswalk CENTRES here (a
    // handy anchor for future ped-crossing logic); the decal group stays empty.
    crosswalkCenters_.clear();
    crosswalkGroup_ = world.create();
    {
        const Real depth = 2.6;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            const engine::NavLink& L = nav_.links[li];
            if (!nav_.isJunction(L.to)) continue;
            Vec2 d = nav_.direction(li);
            Vec2 node = nav_.nodes[L.to];
            Real halfW = L.width * 0.5;
            crosswalkCenters_.push_back(node - d * (halfW + depth * 0.5 + 0.3));
        }
        world.add<InstanceGroup>(crosswalkGroup_, InstanceGroup{});   // empty: road paints the bars
    }

    // Debug widgets (ADR-0061): per-agent footprint ring (one group per behaviour
    // state, coloured) + a forward trajectory arrow. The groups always exist so a
    // key can toggle them at runtime; they stay empty unless debugWidgets_ is on.
    {
        MeshHandle ringMesh{}, arrowMesh{}, stripMesh{};
        MeshHandle wedgeMesh[2]{};
        if (assets) {
            ringMesh = assets->acquireMesh(ringXZ(), "city:dbgring");
            arrowMesh = assets->acquireMesh(arrowXZ(), "city:dbgarrow");
            stripMesh = assets->acquireMesh(stripXZ(), "city:dbglane");
            // The half-angle is baked into the wedge, so each mode gets its own
            // mesh; the instance scale carries only the range.
            wedgeMesh[static_cast<int>(Agent::Mode::Pedestrian)] =
                assets->acquireMesh(wedgeXZ(kPedConeHalfAngle), "city:dbgwedgeped");
            wedgeMesh[static_cast<int>(Agent::Mode::Driver)] =
                assets->acquireMesh(wedgeXZ(kCarConeHalfAngle), "city:dbgwedgecar");
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
        for (int mi = 0; mi < 2; ++mi) {
            visionGroups_[mi] = world.create();
            InstanceGroup g;
            g.mesh = wedgeMesh[mi];
            g.material = widgetMaterial(kConeTint);
            world.add<InstanceGroup>(visionGroups_[mi], g);
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
            Real y = groundAt(a.x, a.y) + L.layer * Real(5.8) + 0.04;
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

    // Bake the CITY-PLAN outlines once (ADR-0066): every block/lot polygon
    // (published by the loader as CityPlanDebug) is stroked as ONE CLOSED
    // RIBBON (device: "use the ribbon library ... form polygon shapes
    // properly") — a continuous mitred loop, not per-edge strips with corner
    // gaps. Draped at each polygon's local ground height (a block is small vs
    // the terrain, so one height reads flat). The group shows the merged mesh
    // via a single identity transform; the show/hide toggle stays the same.
    blockBake_.clear();
    lotBake_.clear();
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
    prevCarSpeed_.assign(sim_.agents().size(), 0.0);
    for (std::size_t i = 0; i < sim_.agents().size(); ++i)
        prevCarSpeed_[i] = sim_.agents()[i].speed;

    built_ = true;
    syncGroups(world);
    return true;
}

engine::RenderMesh CityRenderSystem::buildRoadMarkings() const {
    // Stop bars + lane-turn arrows (R6c, plan 4e): derived entirely from
    // the graph (turn options = the node's outgoing street links), thin
    // raised quads just above the asphalt — the shader's mu/mv paint has
    // no per-lane identity to hang arrows on.
    engine::RenderMesh paint;
        const Vec3 white(0.88, 0.88, 0.88);
        const Vec3 up(0, 1, 0);
        auto quad = [&](const engine::Vec2& c, const engine::Vec2& f,
                        Real halfW, Real halfL, Real y) {
            const engine::Vec2 r(f.y, -f.x);
            const engine::Vec2 a2 = c - r * halfW - f * halfL;
            const engine::Vec2 b2 = c + r * halfW - f * halfL;
            const engine::Vec2 c2 = c + r * halfW + f * halfL;
            const engine::Vec2 d2 = c - r * halfW + f * halfL;
            MeshBuilder::emitQuad(paint, Vec3(a2.x, y, a2.y), Vec3(b2.x, y, b2.y),
                                  Vec3(c2.x, y, c2.y), Vec3(d2.x, y, d2.y), up,
                                  white);
        };
        // A turn arrow: shaft along the lane + head bent toward the turn
        // (-1 left, 0 straight, +1 right).
        auto arrow = [&](const engine::Vec2& c, const engine::Vec2& f, int turn,
                         Real y) {
            quad(c, f, 0.12, 0.85, y);   // shaft
            engine::Vec2 hd = f;
            if (turn != 0) {
                const engine::Vec2 r(f.y, -f.x);
                hd = normalize(f * 0.2 + r * (Real(turn) * 0.98));
                quad(c + f * 0.75 + hd * 0.35, hd, 0.12, 0.45, y);   // bent head
            }
            // Head ticks (a simple V), pointing along hd.
            const engine::Vec2 tip =
                c + f * (turn == 0 ? Real(0.95) : Real(0.75)) +
                (turn == 0 ? engine::Vec2(0, 0) : hd * Real(0.8));
            const engine::Vec2 hr(hd.y, -hd.x);
            quad(tip + hd * 0.12 - hr * 0.22, normalize(hd - hr), 0.10, 0.32, y);
            quad(tip + hd * 0.12 + hr * 0.22, normalize(hd + hr), 0.10, 0.32, y);
        };
        for (int li : signalLinks_) {
            const engine::NavLink& L = nav_.links[li];
            const int to = L.to;
            const engine::Vec2 dir = nav_.direction(li);
            const engine::Vec2 right(dir.y, -dir.x);
            // Stop line: clear of the crossing road's mouth + the shader's
            // zebra band (~4.4 m of crosswalk paint past the mouth).
            Real mouth = 0;
            for (int ol : nav_.outLinks[to])
                mouth = std::max(mouth, nav_.links[ol].width * 0.5);
            Real back = std::min(L.length * 0.4, mouth + 6.2);
            const engine::Vec2 node = nav_.nodes[to];
            const engine::Vec2 barC =
                node - dir * back + right * (L.width * 0.25);
            const Real y = groundAt(barC.x, barC.y) + 0.06;
            quad(barC, dir, L.width * 0.24, 0.3, y);   // the bar (approach half)
            // Turn options from the node's outgoing street links.
            bool hasL = false, hasS = false, hasR = false;
            for (int ol : nav_.outLinks[to]) {
                const engine::NavLink& O = nav_.links[ol];
                if (O.to == L.from) continue;   // the U-turn back
                if (!(O.access & engine::road_access::kSignalable)) continue;   // #17/S5
                const engine::Vec2 od = nav_.direction(ol);
                const Real cross = dir.x * od.y - dir.y * od.x;
                const Real dot2 = dir.x * od.x + dir.y * od.y;
                if (dot2 > 0.7) hasS = true;
                else if (cross < -0.5) hasL = true;
                else if (cross > 0.5) hasR = true;
            }
            const int lanes = std::max(1, L.lanes);
            const Real spacing = sim_.laneSpacingFor(li);
            for (int ln = 0; ln < lanes; ++ln) {
                const engine::Vec2 laneC =
                    node - dir * (back + 2.6) +
                    right * ((0.5 + ln) * spacing);
                // One arrow per lane: leftmost leans left, rightmost right,
                // middle (or absent turns) go straight.
                int turn = 0;
                if (lanes == 1) turn = hasS ? 0 : (hasL ? -1 : (hasR ? 1 : 0));
                else if (ln == 0 && hasL) turn = -1;
                else if (ln == lanes - 1 && hasR) turn = 1;
                arrow(laneC, dir, turn,
                      groundAt(laneC.x, laneC.y) + 0.06);
            }
        }
    return paint;
}

Mat4 CityRenderSystem::agentPose(const Agent& a, int agentIdx) const {
    bool car = a.mode == Agent::Mode::Driver;
    // Tilt low-pass (4b): blend this tick's fitted up-vector toward the last
    // one before building the basis. Raw per-tick refits on noisy ground
    // vibrated the body; the ~0.5 s constant also eases the pitch snap at
    // link-grade boundaries. Returns the smoothed up (identity for peds or
    // unindexed callers).
    auto smoothedUp = [&](Vec3 up) {
        if (agentIdx < 0 || !car) return up;
        if (smoothUp_.size() <= static_cast<std::size_t>(agentIdx))
            smoothUp_.resize(sim_.agents().size(), Vec3(0, 0, 0));
        Vec3& su = smoothUp_[agentIdx];
        // Gain from the bake interval (tau ~0.7 s), so the filter behaves the
        // same at the viewer's 60 Hz and the tests' 10 Hz.
        const Real k = 1.0 - std::exp(-bakeDt_ / 0.7);
        if (su.lengthSquared() < 1e-6)
            su = up;
        else
            su = normalize(su + (up - su) * k);
        return su;
    };
    Real x = a.pos.x, z = a.pos.y;          // Vec2 maps to world XZ (.y = world z)
    // Lift the box so it rests on the ground: half its OWN body height (a tall van
    // or box truck sits higher than a sedan). Read the height from the possessed
    // SimVehicle (authoritative), falling back to the default car/ped size.
    Real bodyH = car ? params_.carSize.y : params_.pedSize.y;
    if (car && a.vehicle >= 0 && a.vehicle < static_cast<int>(sim_.vehicles().size()))
        bodyH = sim_.vehicles()[a.vehicle].height;
    Real halfH = bodyH * 0.5;
    // Absolute deck (corridor): the deck Y IS the surface; ground-relative
    // placement hovered/sank between chain nodes on hills (device).
    Real y = (a.deckY > -1e29) ? a.deckY + halfH
                               : groundAt(x, z) + a.elevation + halfH;
    Real yaw = std::atan2(a.heading.x, a.heading.y); // box local +Z -> travel heading
    // Cars sit NORMAL to the road plane (device: a world-upright box on a
    // graded street floats its nose or buries its tail). Sample the drive
    // surface a wheelbase fore/aft and a track left/right, build the tilted
    // frame from those slopes, and write the basis directly — no pitch/roll
    // sign gymnastics. Bridge traffic (elevation) rides a flat deck: skip.
    if (car && (a.deckY > -1e29 || a.elevation >= 0.5)) {
        // ELEVATED (deck/ramp): the ground sampler below would read the
        // terrain UNDER the structure, so pitch from the link's grade
        // instead — all four wheels track the ramp (device feedback).
        Vec2 f = a.heading;
        const Real fl = f.length();
        if (fl > 1e-6 && std::fabs(a.grade) > 1e-4) {
            f = f * (1.0 / fl);
            Vec3 fw = normalize(Vec3(f.x, a.grade, f.y));
            Vec3 rt(f.y, 0, -f.x);
            Vec3 up = normalize(cross(fw, rt));
            if (up.y < 0) up = up * -1;
            up = smoothedUp(up);
            fw = normalize(fw - up * dot(fw, up));
            rt = normalize(cross(up, fw));
            Mat4 m;
            m.m[0][0] = rt.x; m.m[1][0] = rt.y; m.m[2][0] = rt.z;
            m.m[0][1] = up.x; m.m[1][1] = up.y; m.m[2][1] = up.z;
            m.m[0][2] = fw.x; m.m[1][2] = fw.y; m.m[2][2] = fw.z;
            m.m[0][3] = x; m.m[1][3] = y; m.m[2][3] = z;
            return m;
        }
    }
    if (car && a.elevation < 0.5) {
        Vec2 f = a.heading;
        const Real fl = f.length();
        if (fl > 1e-6) {
            f = f * (1.0 / fl);
            const Vec2 r(f.y, -f.x);
            const Real hl = 1.3, hw = 0.7;
            const Real yF = groundAt(x + f.x * hl, z + f.y * hl);
            const Real yB = groundAt(x - f.x * hl, z - f.y * hl);
            const Real yR = groundAt(x + r.x * hw, z + r.y * hw);
            const Real yL = groundAt(x - r.x * hw, z - r.y * hw);
            Vec3 fw = normalize(Vec3(f.x, (yF - yB) / (2 * hl), f.y));
            Vec3 rt = normalize(Vec3(r.x, (yR - yL) / (2 * hw), r.y));
            Vec3 up = cross(fw, rt);
            if (up.lengthSquared() > 1e-9) {
                up = normalize(up);
                if (up.y < 0) up = up * -1;
                up = smoothedUp(up);
                fw = normalize(fw - up * dot(fw, up));
                rt = normalize(cross(up, fw));
                Mat4 m;   // columns: X = right, Y = up, Z = forward (yaw basis)
                m.m[0][0] = rt.x; m.m[1][0] = rt.y; m.m[2][0] = rt.z;
                m.m[0][1] = up.x; m.m[1][1] = up.y; m.m[2][1] = up.z;
                m.m[0][2] = fw.x; m.m[1][2] = fw.y; m.m[2][2] = fw.z;
                m.m[0][3] = x; m.m[1][3] = y; m.m[2][3] = z;
                return m;
            }
        }
    }
    if (car && agentIdx >= 0) {
        const Vec3 up = smoothedUp(Vec3(0, 1, 0));
        Vec2 f = a.heading;
        const Real fl = f.length();
        if (fl > 1e-6 && up.y < 0.99999) {
            f = f * (1.0 / fl);
            Vec3 fw(f.x, 0, f.y);
            fw = normalize(fw - up * dot(fw, up));
            const Vec3 rt = normalize(cross(up, fw));
            Mat4 m;
            m.m[0][0] = rt.x; m.m[1][0] = rt.y; m.m[2][0] = rt.z;
            m.m[0][1] = up.x; m.m[1][1] = up.y; m.m[2][1] = up.z;
            m.m[0][2] = fw.x; m.m[1][2] = fw.y; m.m[2][2] = fw.z;
            m.m[0][3] = x; m.m[1][3] = y; m.m[2][3] = z;
            return m;
        }
    }
    Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
    return Mat4::trs(Vec3(x, y, z), rot, Vec3(1, 1, 1));
}

std::vector<Vec3> CityRenderSystem::carGroupHalfExtents() const {
    std::vector<Vec3> out;
    out.reserve(carGroups_.size());
    for (std::size_t v = 0; v < carGroups_.size(); ++v) {
        Vec3 s = fleetBodySize(static_cast<int>(v));   // (width, height, length)
        out.push_back(Vec3(s.x * 0.5, s.y * 0.5, s.z * 0.5));
    }
    return out;
}

namespace {
constexpr Real kCurbGap = 0.8;   // pole stands this far beyond the kerb
}  // namespace

// The signal pole stands at the near-right curb corner of the junction the
// approach enters, scaled to THIS road's width so it never lands in the
// carriageway (a fixed setback put poles in the middle of wide roads). The mast
// arm then reaches sideways over the street toward the centre.
CityRenderSystem::SignalSite CityRenderSystem::signalSite(int link) const {
    // Build-time furniture override: the CITY placed this pole (StreetFurniture,
    // adopted in build()); everything downstream — lens poses, ped obstacles,
    // physics poles — reads the placed site.
    if (link >= 0 && link < static_cast<int>(siteHasLink_.size()) &&
        siteHasLink_[link])
        return siteByLink_[link];
    int toNode = nav_.links[link].to;
    Vec2 d = nav_.direction(link);               // approach direction (toward junction)
    Vec2 node = nav_.nodes[toNode];
    Vec2 right(d.y, -d.x);
    // Clear the pole from EVERY road at this junction, not just the approach: back
    // off along the approach by the widest crossing road's half-width (so it sits
    // beyond the perpendicular carriageway) and out to the side by this road's own
    // half-width. A fixed setback left poles in the middle of wider cross streets.
    // A KNOT-MERGED junction's drawn crossing extends nodeSpread beyond the node
    // (the node is the knot centroid) — back off by that too, or the pole stands
    // mid-carriageway on generated crossings (device feedback).
    Real thisHalf = nav_.links[link].width * 0.5;
    Real crossHalf = thisHalf;
    for (int ol : nav_.outLinks[toNode])
        crossHalf = std::max(crossHalf, nav_.links[ol].width * 0.5);
    Real spread = toNode < static_cast<int>(nav_.nodeSpread.size())
                      ? nav_.nodeSpread[toNode] : 0.0;
    Vec2 corner = node - d * (crossHalf + spread + kCurbGap) +
                  right * (thisHalf + kCurbGap);
    Real baseY = groundAt(corner.x, corner.y) + nav_.links[link].layer * Real(5.8);
    SignalSite s;
    s.base = Vec3(corner.x, baseY, corner.y);
    s.face = Vec3(-d.x, 0, -d.y);                 // head faces approaching traffic
    s.side = Vec3(-d.y, 0, d.x);                  // = rightOf(face): toward road centre
    s.yaw = std::atan2(s.face.x, s.face.z);
    return s;
}

Mat4 CityRenderSystem::signalPostPose(int link) const {
    SignalSite s = signalSite(link);
    return Mat4::trs(s.base, Quat::fromAxisAngle(Vec3(0, 1, 0), s.yaw), Vec3(1, 1, 1));
}

Mat4 CityRenderSystem::signalLensPose(int link, SignalState s) const {
    SignalSite st = signalSite(link);
    // Lamp slot on the three-lamp head (mirror of street_kit::emitTrafficSignal):
    // the head hangs at the arm end (along `side`), red on top / amber / green on
    // the bottom, lenses on the facing side.
    engine::SignalParams sp;
    Vec3 headTop = st.base + Vec3(0, sp.armHeight - 0.1, 0) + st.side * (sp.armLength - 0.2);
    Vec3 headCenter = headTop + Vec3(0, -0.55, 0);
    Real slotY = (s == SignalState::Red) ? 0.42 : (s == SignalState::Green) ? -0.42 : 0.0;
    Vec3 p = headCenter + Vec3(0, slotY, 0) + st.face * 0.22;
    return Mat4::trs(p, Quat(), Vec3(1, 1, 1));
}

// The turn side (-1 left / +1 right / 0 none) a car will take at the coming node,
// from the bend between its current and next route legs — mirrors the sim's own
// bendAhead read in labelDriverState, reused for the turn-signal decision.
int CityRenderSystem::carTurnDir(const Agent& a) const {
    if (a.mode != Agent::Mode::Driver) return 0;
    const int legCount = static_cast<int>(a.route.links.size());
    if (a.leg < 0 || a.leg + 1 >= legCount) return 0;
    Vec2 d0 = nav_.direction(a.route.links[a.leg]);
    Vec2 d1 = nav_.direction(a.route.links[a.leg + 1]);
    return carTurnSide(d0, d1);
}

// Bake the emissive car lamps (ADR-0065 follow-up). Same shape as the signal
// lenses: clear each per-kind group, then for every DRAWN car push a small
// emissive box at each lit lamp marker's WORLD pose. Only the lit lamps are
// pushed, so a phase change (headlights on at dusk, brakes on braking) just moves
// transforms between batches — cheap, a handful of quads per car, rebuilt per step.
void CityRenderSystem::syncCarLamps(World& world) {
    InstanceGroup* head = world.get<InstanceGroup>(headlightGroup_);
    InstanceGroup* brake = world.get<InstanceGroup>(brakeLightGroup_);
    InstanceGroup* turn = world.get<InstanceGroup>(turnSignalGroup_);
    if (head) head->transforms.clear();
    if (brake) brake->transforms.clear();
    if (turn) turn->transforms.clear();

    const std::vector<Agent>& agents = sim_.agents();
    if (prevCarSpeed_.size() != agents.size())
        prevCarSpeed_.assign(agents.size(), 0.0);

    // Externally-owned cars aren't drawn by this bridge (carGroups_ empty), so
    // their lamps aren't either. We still update prevCarSpeed_ below.
    const bool drawCars = !carGroups_.empty();

    // Turn signals blink off the SIM clock — deterministic (~1.5 Hz, 50% duty), so
    // the bake is reproducible and headless-testable. A render-time wall clock
    // would be acceptable too (a non-deterministic blink is fine per the ADR), but
    // the sim clock keeps determinism intact.
    const bool blinkOn = std::fmod(sim_.seconds() * kTurnBlinkHz, 1.0) < 0.5;

    for (std::size_t ai = 0; ai < agents.size(); ++ai) {
        const Agent& a = agents[ai];
        const Real prev = prevCarSpeed_[ai];
        prevCarSpeed_[ai] = a.speed;   // record for next step's decel test
        if (!drawCars) continue;
        if (a.mode != Agent::Mode::Driver) continue;
        if (a.released) continue;      // commandeered: the physical car owns its lamps
        if (a.tier == Agent::Tier::V) continue;   // far tier: no drawn car, no lamps
        const int v = (a.vehicle >= 0 ? a.vehicle : 0) % carVariantCount();
        if (v < 0 || v >= static_cast<int>(carLights_.size())) continue;
        const std::vector<LampMarker>& markers = carLights_[v];
        if (markers.empty()) continue;

        // A lane change signals the side it moves toward (device); the
        // route-bend indicator covers junction turns as before.
        int turnDir = carTurnDir(a);
        if (std::fabs(Real(a.lane) - a.laneF) > 0.12)
            turnDir = (Real(a.lane) > a.laneF) ? 1 : -1;
        CarLamps lamps = carLampState(a.state, a.speed, prev, sim_.timeOfDay(),
                                      turnDir);
        if (!lamps.head && !lamps.brake && !lamps.left && !lamps.right) continue;

        const Mat4 pose = agentPose(a);
        const Real yaw = std::atan2(a.heading.x, a.heading.y);
        const Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        for (const LampMarker& m : markers) {
            const bool isHead = m.name.rfind("headlight", 0) == 0;
            const bool isTail = m.name.rfind("taillight", 0) == 0;
            const bool leftSide = !m.name.empty() && m.name.back() == 'l';
            const bool rightSide = !m.name.empty() && m.name.back() == 'r';
            const Vec3 wpos = pose.transformPoint(m.pos);
            const Mat4 xf = Mat4::trs(wpos, rot, Vec3(1, 1, 1));
            if (isHead && lamps.head && head) head->transforms.push_back(xf);
            if (isTail && lamps.brake && brake) brake->transforms.push_back(xf);
            // The turn indicator on the signalling side flashes on BOTH the front
            // and rear corner markers of that side.
            const bool signalling =
                (leftSide && lamps.left) || (rightSide && lamps.right);
            if (signalling && blinkOn && turn) turn->transforms.push_back(xf);
        }
    }
    refreshBounds(head);
    refreshBounds(brake);
    refreshBounds(turn);
}

void CityRenderSystem::syncGroups(World& world) {
    std::vector<InstanceGroup*> cars;
    cars.reserve(carGroups_.size());
    for (Entity e : carGroups_) cars.push_back(world.get<InstanceGroup>(e));
    InstanceGroup* ped = world.get<InstanceGroup>(pedGroup_);
    InstanceGroup* sig[3];
    for (int s = 0; s < 3; ++s) sig[s] = world.get<InstanceGroup>(signalGroups_[s]);

    for (InstanceGroup* c : cars) if (c) c->transforms.clear();
    if (ped) ped->transforms.clear();
    for (int s = 0; s < 3; ++s) if (sig[s]) sig[s]->transforms.clear();

    carAgentIds_.assign(cars.size(), {});
    pedAgentIds_.assign(1, {});
    const auto& agents = sim_.agents();
    for (std::size_t ai = 0; ai < agents.size(); ++ai) {
        const Agent& a = agents[ai];
        // P4: a far (V) agent has NO render membership — no instance, no lamp,
        // no kinematic proxy (the physics diff keys off these id lists).
        if (a.tier == Agent::Tier::V) continue;
        if (a.mode == Agent::Mode::Driver) {
            if (cars.empty()) continue;   // cars owned externally (ADR-0062 bridge)
            if (a.released) continue;     // commandeered: its PHYSICAL car replaced it
            // Each driver keeps the same variant (keyed off its car index), so a
            // given car is always the same model + colour.
            int v = (a.vehicle >= 0 ? a.vehicle : 0) % carVariantCount();
            if (!cars[v]) continue;
            // R5: a physically-possessed agent renders its Jolt body's pose —
            // real suspension, pitch and roll — while the sim's kinematic
            // plan stays the brains' truth.
            const auto po = physPose_.find(static_cast<int>(ai));
            cars[v]->transforms.push_back(
                po != physPose_.end() ? po->second
                                      : agentPose(a, static_cast<int>(ai)));
            carAgentIds_[v].push_back(static_cast<int>(ai));
        } else if (ped && !pedsExternallyOwned_) {   // walkers owned externally: no bake
            ped->transforms.push_back(agentPose(a));
            pedAgentIds_[0].push_back(static_cast<int>(ai));
        }
    }
    // Scenery parked cars (R6b): bays seeded full at build render a real car
    // (variant by bay index) — and, riding the car groups, they get the same
    // kinematic collision boxes as ambient traffic for free.
    //
    // PERF (measured): a city-wide InstanceGroup gets ONE bounding sphere, so
    // the frustum cull can never reject part of it — every parked car in the
    // city was drawn every frame, in the colour AND shadow passes. At piedmont
    // scale that is ~4.4k cars; once the fleet moved from 170-triangle boxes to
    // real 1.9k-triangle bodies it became ~8M triangles a frame, and re-deriving
    // each pose (a terrain sample apiece) cost ~8ms per FIXED STEP. So: resolve
    // the poses once, then draw only what is near the player.
    if (!cars.empty()) {
        if (!sceneryBuilt_) {
            const std::vector<Vec3> he = carGroupHalfExtents();
            const auto& bays = sim_.parkingBays();
            scenery_.clear();
            for (std::size_t bi = 0; bi < bays.size(); ++bi) {
                const CitySim::ParkingBay& b = bays[bi];
                if (b.occupant != CitySim::kBayScenery) continue;
                const int v = static_cast<int>(bi) % carVariantCount();
                const Real y = groundAt(b.pos.x, b.pos.y) +
                               (v < static_cast<int>(he.size()) ? he[v].y : 0.65);
                const Real yaw = std::atan2(b.heading.x, b.heading.y);
                SceneryCar sc;
                sc.pose = Mat4::trs(Vec3(b.pos.x, y, b.pos.y),
                                    Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                                    Vec3(1, 1, 1));
                sc.pos = b.pos;
                sc.variant = v;
                sc.bay = static_cast<int>(bi);
                scenery_.push_back(sc);
            }
            sceneryBuilt_ = true;
            LOG_INFO << "[citysim] scenery parked cars: " << scenery_.size()
                     << " poses resolved once (drawn within "
                     << params_.sceneryRadius << " m)";
        }
        // Draw radius from the tier centre (the player). Beyond it a parked car
        // is a few pixels and costs a full body in two passes.
        const Vec2 centre = sim_.tierCenter();
        const bool haveCentre = sim_.hasTierCenter();
        const Real rad = params_.sceneryRadius;
        const Real rad2 = rad * rad;
        for (const SceneryCar& sc : scenery_) {
            if (haveCentre && rad > 0) {
                const Real dx = sc.pos.x - centre.x, dz = sc.pos.y - centre.y;
                if (dx * dx + dz * dz > rad2) continue;
            }
            if (!cars[sc.variant]) continue;
            cars[sc.variant]->transforms.push_back(sc.pose);
            // NEGATIVE-but-stable id: the physics proxy key must not shuffle
            // when the cull changes bake order (-1 stays "unknown").
            carAgentIds_[sc.variant].push_back(-2 - sc.bay);
        }
    }

    // Each signalled approach lights ONE lamp on its head — the lens for its
    // CURRENT state — so a phase change moves the lit lens between the red/amber/
    // green emissive batches. The pole assembly (signalPostGroup_) is static and
    // left untouched. (The lit lens overlays the head's matching housing lamp.)
    SignalController& sc = sim_.signals();
    for (int li : signalLinks_) {
        SignalState st = sc.stateForLink(li);
        int s = static_cast<int>(st);
        if (sig[s]) sig[s]->transforms.push_back(signalLensPose(li, st));
    }

    for (InstanceGroup* c : cars) refreshBounds(c);
    refreshBounds(ped);
    for (int s = 0; s < 3; ++s) refreshBounds(sig[s]);

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
        InstanceGroup* cone[2];
        for (int mi = 0; mi < 2; ++mi) {
            cone[mi] = world.get<InstanceGroup>(visionGroups_[mi]);
            if (cone[mi]) cone[mi]->transforms.clear();
        }
        // The navgraph view is a static bake: copy it in when the HUD is on,
        // leave the groups empty when it's off (mirrors the vanishing rings).
        InstanceGroup* navL = world.get<InstanceGroup>(navLinkGroup_);
        InstanceGroup* navN = world.get<InstanceGroup>(navNodeGroup_);
        const bool showNav = debugWidgets_ && showNavGraph_;
        if (navL) navL->transforms = showNav ? navLinkBake_ : std::vector<Mat4>{};
        if (navN) navN->transforms = showNav ? navNodeBake_ : std::vector<Mat4>{};
        // City-plan outlines (static bakes, same show-or-empty pattern).
        const bool showPlan = debugWidgets_ && showPlan_;
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
            if (a.tier == Agent::Tier::V) continue;   // far tier: nothing drawn
            bool car = a.mode == Agent::Mode::Driver;
            if (car && a.released) continue;   // commandeered: no ghost widget
            Real x = a.pos.x, z = a.pos.y;
            Vec2 heading = a.heading;
            // Where this agent is TRYING to go (the arrow's tip): the ghost's
            // short-horizon intent by default; the bridge's pursuit target when
            // the body is external.
            Vec2 goal = a.pos + a.heading * (2.0 + a.speed);
            Agent::State ringState = a.state;   // body truth may override below
            // Externally-owned agents (ADR-0062): the widget must ring the
            // PHYSICAL body, not the planner ghost — the ghost legitimately runs
            // ahead or behind, and a ring around it is an empty circle on the
            // ground. Use the bridge-reported real pose; an agent with no
            // reported body (released to the player, not yet spawned) draws none.
            bool external = car ? carsExternallyOwned_ : pedsExternallyOwned_;
            if (external) {
                const std::vector<ExternalAgentPose>& poses =
                    car ? externalCarPoses_ : externalPedPoses_;
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
            // Ring sized to THIS body (a box truck's footprint is bigger than a
            // sedan's), read from the possessed vehicle's dimensions.
            Real radius = params_.pedSize.x * 0.9;
            if (car) {
                Real cw = params_.carSize.x, cl = params_.carSize.z;
                if (a.vehicle >= 0 && a.vehicle < static_cast<int>(sim_.vehicles().size())) {
                    cw = sim_.vehicles()[a.vehicle].width;
                    cl = sim_.vehicles()[a.vehicle].length;
                }
                radius = std::max(cw, cl) * 0.5;
            }
            radius *= 1.2;   // proud of the body so the painted rim always shows
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
            // The sensing wedge: what this agent can SEE, yawed to its heading
            // and scaled to its mode's cone range (the half-angle is baked into
            // the mesh). Moving agents only — a parked car senses nothing worth
            // drawing — and just below the ring so the two never z-fight.
            InstanceGroup* cg = cone[static_cast<int>(a.mode)];
            if (showVisionCones_ && cg && a.moving) {
                Real range = car ? kCarConeRange : kPedConeRange;
                Real coneYaw = std::atan2(heading.x, heading.y);
                cg->transforms.push_back(Mat4::trs(
                    Vec3(x, y - 0.01, z),
                    Quat::fromAxisAngle(Vec3(0, 1, 0), coneYaw),
                    Vec3(range, 1, range)));
            }
        }
        for (int s = 0; s < kStateCount; ++s) refreshBounds(foot[s]);
        refreshBounds(fwd);
        for (int mi = 0; mi < 2; ++mi) refreshBounds(cone[mi]);
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

    // Emissive car lamps (ADR-0065 follow-up): headlights / brake / turn signals,
    // driven by each drawn car's carLampState. Always baked (not a debug widget).
    syncCarLamps(world);
}

void CityRenderSystem::step(World& world, Real dt) {
    if (!built_) return;
    bakeDt_ = dt;   // the tilt low-pass keys its gain to the bake interval
    // Three-tier traffic (P4): feed the sim the player's position each fixed
    // step — the V/K bubble's centre. No player entity (headless tests, menu
    // scenes) clears it, and everything stays K. Inert unless the level set
    // tieredAgents; deterministic for a deterministic world.
    {
        bool haveCentre = false;
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
        if (bakeThisStep_) syncGroups(world);
        return;
    }
    static double simMs = 0.0, syncMs = 0.0;
    static int calls = 0;
    auto t0 = std::chrono::steady_clock::now();
    sim_.step(dt, params_.hoursPerSecond);
    auto t1 = std::chrono::steady_clock::now();
    if (bakeThisStep_) syncGroups(world);
    auto t2 = std::chrono::steady_clock::now();
    simMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    syncMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
    if (++calls % 300 == 0) {
        LOG_INFO << "[stats] citysim step " << (simMs / calls)
                 << " ms, group sync " << (syncMs / calls) << " ms (per fixed step)";
        simMs = syncMs = 0.0;
        calls = 0;
    }
}

void CityRenderSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("agent_widgets", engine::KeyCode::J);   // toggle debug widgets
    ctx.actions.bindButton("plan_widgets", engine::KeyCode::L);    // toggle block/lot plan
}

void CityRenderSystem::update(engine::FrameContext& ctx) {
    // Per-frame so the key edge is never missed by the fixed-step tick.
    if (ctx.actions.pressed("agent_widgets")) debugWidgets_ = !debugWidgets_;
    // L flips the city-plan layer (blocks + lots) — and switches the master on
    // when it was off, so the key works standalone on web (no ImGui panel there).
    if (ctx.actions.pressed("plan_widgets")) {
        showPlan_ = !showPlan_;
        if (showPlan_) debugWidgets_ = true;
    }
    // Web debug panel (rt_web_city): ONE-SHOT settings writes — apply and clear,
    // so the page's checkboxes and the J/L keys can share the same flags without
    // a persistent setting overriding the keys every frame.
    auto pull = [&](const char* key, bool& flag) {
        const double v = ctx.settings.getDouble(key, -1.0);
        if (v >= 0.0) {
            flag = v > 0.5;
            ctx.settings.setDouble(key, -1.0);
        }
    };
    pull("citysim.master", debugWidgets_);
    pull("citysim.agents", showAgentWidgets_);
    pull("citysim.cones", showVisionCones_);
    pull("citysim.nav", showNavGraph_);
    pull("citysim.plan", showPlan_);
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
        default:                return IM_COL32(220, 220, 220, 255);
    }
}
// Short label for an agent's Role (Living City Phase 4).
const char* agentRoleName(Agent::Role r) {
    switch (r) {
        case Agent::Role::Commuter:   return "Commuter";
        case Agent::Role::Shopkeeper: return "Shopkeeper";
        case Agent::Role::Stroller:   return "Stroller";
        default:                      return "?";
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
        int drivers = 0, peds = 0;
        for (const Agent& a : agents)
            (a.mode == Agent::Mode::Driver ? drivers : peds)++;
        ImGui::Text("Time %05.2f h   Agents %d  (%d cars, %d peds)",
                    sim_.timeOfDay(), static_cast<int>(agents.size()), drivers, peds);
        ImGui::Text("Perception faults: %ld", sim_.faults());

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
                        a.mode == Agent::Mode::Driver ? "Driver" : "Pedestrian",
                        agentRoleName(a.role));
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
            // Surface-level social graph: who this agent knows.
            const RelationshipTable& rel = sim_.relationships();
            const auto& knows = rel.relationsOf(a.uid);
            ImGui::Text("Knows %d", static_cast<int>(knows.size()));
            for (std::size_t k = 0; k < knows.size() && k < 6; ++k)
                ImGui::BulletText("UID %u (%s)", knows[k].first,
                                  relationshipName(knows[k].second));
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

void CityRenderSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!built_) {
        build(ctx.world, &ctx.assets);   // lazy: retry until the level's roads exist
        return;
    }
    // Feed the live player to the sim so AI cars brake for (and hold short of) it,
    // whether the player is on foot or driving. The player is the entity with a
    // CharacterController under host control (PlayerSystem).
    std::vector<Vec2> obstacles;
    ctx.world.each<engine::Transform, engine::CharacterController, engine::ControlledBy>(
        [&](engine::Entity, engine::Transform& t, engine::CharacterController&,
            engine::ControlledBy&) {
            obstacles.push_back(Vec2(t.position.x, t.position.z));
        });
    // Every REAL physics Vehicle too (the player's car, commandeered/promoted
    // cars — driven OR abandoned): a promoted car has no planner ghost, so
    // without this ambient traffic plans straight through a car parked across
    // the lane and the kinematic proxies bulldoze the dynamic body.
    ctx.world.each<engine::Transform, engine::Vehicle>(
        [&](engine::Entity, engine::Transform& t, engine::Vehicle&) {
            obstacles.push_back(Vec2(t.position.x, t.position.z));
        });
    sim_.setExternalObstacles(std::move(obstacles));

    // The pose bake exists for the RENDERER. When the clock runs several fixed
    // steps in one frame (catching up), only the last bake is ever drawn — the
    // rest are pure waste, and at piedmont scale that was ~4.5 bakes a frame.
    // Physics proxies read the bake too, so they track the freshest step.
    bakeThisStep_ = (ctx.fixedStepIndex + 1 >= ctx.fixedStepCount);
    step(ctx.world, ctx.clock.fixedStep());
    bakeThisStep_ = true;   // a direct step() call (tests) always bakes
}

}  // namespace citysim
