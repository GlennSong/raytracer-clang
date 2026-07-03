#include "city_render.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/city/road_net.h"
#include "../../engine/procgen/city/street_kit.h"   // trafficSignalProto, SignalParams
#include "../../renderer/event.h"                    // KeyCode (debug-widget toggle)

#include <algorithm>
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

bool CityRenderSystem::build(World& world, AssetManager* assets) {
    // Level-authored settings (ADR-0063): a CitySimConfig entity — the level's
    // top-level "citysim" block — overrides the constructor params, so each level
    // picks its own population, seed, clock rate, reliability, and whether the
    // agent-state HUD starts on. The agent lab runs 1 car + 1 walker, HUD on.
    world.each<engine::CitySimConfig>([&](Entity, engine::CitySimConfig& c) {
        params_.cars = c.cars;
        params_.pedestrians = c.pedestrians;
        params_.seed = c.seed;
        params_.hoursPerSecond = c.hoursPerSecond;
        params_.perceptionReliability = c.perceptionReliability;
        params_.wander = c.wander;
        debugWidgets_ = c.debugWidgets;
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
    if (combined.nodes.empty() || combined.edges.empty()) return false;

    nav_ = engine::buildNavGraph(combined);
    if (nav_.linkCount() == 0) return false;

    sim_.build(nav_, params_.cars, params_.pedestrians, params_.seed);
    sim_.setPerceptionReliability(params_.perceptionReliability);
    sim_.setWander(params_.wander);
    // Warm up so the city is already ALIVE when the level appears — agents depart
    // and spread onto the roads instead of standing still for the first minute.
    for (int i = 0; i < 400; ++i) sim_.step(0.1, params_.hoursPerSecond);

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
    if (!carsExternallyOwned_)
        for (int v = 0; v < carVariantCount(); ++v) {
            MeshHandle mh{};
            if (assets)
                mh = assets->acquireMesh(fleetCarMesh(v),
                                         "city:car" + std::to_string(v));
            Entity e = world.create();
            InstanceGroup g;
            g.mesh = mh;
            g.material = carMaterial();
            world.add<InstanceGroup>(e, g);
            carGroups_.push_back(e);
        }
    pedGroup_ = world.create();
    { InstanceGroup g; g.mesh = pedMesh; g.material = pedMaterial(); world.add<InstanceGroup>(pedGroup_, g); }
    for (int s = 0; s < 3; ++s) {
        signalGroups_[s] = world.create();
        InstanceGroup g;
        g.mesh = lensMesh;
        g.material = signalMaterial(static_cast<SignalState>(s));
        world.add<InstanceGroup>(signalGroups_[s], g);
    }

    // Reuse the city's street-kit traffic-signal model: one pole+arm+head
    // assembly per signalled approach, placed on the near-right corner facing the
    // oncoming traffic it governs (same geometry the city generator uses). It is
    // static, so its transforms are baked once here; only the lit lens (above)
    // changes each step.
    signalLinks_.clear();
    SignalController& sc = sim_.signals();
    for (int li = 0; li < nav_.linkCount(); ++li)
        if (sc.hasSignal(li)) signalLinks_.push_back(li);

    MeshHandle postMesh{};
    if (assets) postMesh = assets->acquireMesh(engine::trafficSignalProto(), "city:signalpost");
    signalPostGroup_ = world.create();
    {
        InstanceGroup g;
        g.mesh = postMesh;
        g.material = signalPostMaterial();
        for (int li : signalLinks_) g.transforms.push_back(signalPostPose(li));
        refreshBounds(&g);
        world.add<InstanceGroup>(signalPostGroup_, g);
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
        // Mirror the sim's laneSpacing: the right half-carriageway split evenly
        // among this direction's lanes, so the strips sit under real traffic.
        Real spacing = (L.width * 0.5) / static_cast<Real>(lanes);
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

    built_ = true;
    syncGroups(world);
    return true;
}

Mat4 CityRenderSystem::agentPose(const Agent& a) const {
    bool car = a.mode == Agent::Mode::Driver;
    Real x = a.pos.x, z = a.pos.y;          // Vec2 maps to world XZ (.y = world z)
    // Lift the box so it rests on the ground: half its OWN body height (a tall van
    // or box truck sits higher than a sedan). Read the height from the possessed
    // SimVehicle (authoritative), falling back to the default car/ped size.
    Real bodyH = car ? params_.carSize.y : params_.pedSize.y;
    if (car && a.vehicle >= 0 && a.vehicle < static_cast<int>(sim_.vehicles().size()))
        bodyH = sim_.vehicles()[a.vehicle].height;
    Real halfH = bodyH * 0.5;
    Real y = groundAt(x, z) + a.elevation + halfH;   // a.elevation lifts bridge traffic
    Real yaw = std::atan2(a.heading.x, a.heading.y); // box local +Z -> travel heading
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
    int toNode = nav_.links[link].to;
    Vec2 d = nav_.direction(link);               // approach direction (toward junction)
    Vec2 node = nav_.nodes[toNode];
    Vec2 right(d.y, -d.x);
    // Clear the pole from EVERY road at this junction, not just the approach: back
    // off along the approach by the widest crossing road's half-width (so it sits
    // beyond the perpendicular carriageway) and out to the side by this road's own
    // half-width. A fixed setback left poles in the middle of wider cross streets.
    Real thisHalf = nav_.links[link].width * 0.5;
    Real crossHalf = thisHalf;
    for (int ol : nav_.outLinks[toNode])
        crossHalf = std::max(crossHalf, nav_.links[ol].width * 0.5);
    Vec2 corner = node - d * (crossHalf + kCurbGap) + right * (thisHalf + kCurbGap);
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

    for (const Agent& a : sim_.agents()) {
        if (a.mode == Agent::Mode::Driver) {
            if (cars.empty()) continue;   // cars owned externally (ADR-0062 bridge)
            if (a.released) continue;     // commandeered: its PHYSICAL car replaced it
            // Each driver keeps the same variant (keyed off its car index), so a
            // given car is always the same model + colour.
            int v = (a.vehicle >= 0 ? a.vehicle : 0) % carVariantCount();
            if (cars[v]) cars[v]->transforms.push_back(agentPose(a));
        } else if (ped && !pedsExternallyOwned_) {   // walkers owned externally: no bake
            ped->transforms.push_back(agentPose(a));
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
        if (navL) navL->transforms = debugWidgets_ ? navLinkBake_
                                                   : std::vector<Mat4>{};
        if (navN) navN->transforms = debugWidgets_ ? navNodeBake_
                                                   : std::vector<Mat4>{};
        const auto& agents = sim_.agents();
        for (std::size_t ai = 0; ai < agents.size() && debugWidgets_; ++ai) {
            const Agent& a = agents[ai];
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
            if (fg) fg->transforms.push_back(
                Mat4::trs(Vec3(x, y, z), Quat(), Vec3(radius, 1, radius)));
            if (fwd && a.moving) {
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
            if (cg && a.moving) {
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
    }
}

void CityRenderSystem::step(World& world, Real dt) {
    if (!built_) return;
    sim_.step(dt, params_.hoursPerSecond);
    syncGroups(world);
}

void CityRenderSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("agent_widgets", engine::KeyCode::J);   // toggle debug widgets
}

void CityRenderSystem::update(engine::FrameContext& ctx) {
    // Per-frame so the key edge is never missed by the fixed-step tick.
    if (ctx.actions.pressed("agent_widgets")) debugWidgets_ = !debugWidgets_;
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

    step(ctx.world, ctx.clock.fixedStep());
}

}  // namespace citysim
