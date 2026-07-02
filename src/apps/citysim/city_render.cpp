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
using engine::RenderMaterial;
using engine::InstanceGroup;
using engine::RoadGraph;
using engine::RoadNode;
using engine::RoadEdge;
using engine::RoadNet;
using engine::MeshHandle;
using engine::MeshBuilder;

namespace {

RenderMaterial carMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);            // hue carried in the car mesh's vertex colour
    m.metallic = 0.5f;
    m.roughness = 0.4f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

RenderMaterial pedMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(0.80, 0.55, 0.40);
    m.metallic = 0.0f;
    m.roughness = 0.85f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

// An emissive lens: dark body, strong self-illumination in the signal colour so
// the active phase glows even before any scene lighting.
RenderMaterial signalMaterial(SignalState s) {
    RenderMaterial m;
    m.metallic = 0.0f;
    m.roughness = 0.4f;
    m.opacity = 1.0f;
    m.flags = 0;
    switch (s) {
        case SignalState::Green:  m.albedo = Vec3(0.0, 0.2, 0.0); m.emission = Vec3(0.1, 1.6, 0.2); break;
        case SignalState::Yellow: m.albedo = Vec3(0.2, 0.18, 0.0); m.emission = Vec3(1.6, 1.3, 0.1); break;
        case SignalState::Red:    m.albedo = Vec3(0.2, 0.0, 0.0); m.emission = Vec3(1.6, 0.1, 0.1); break;
    }
    return m;
}

// Append a coloured box (centred at `c`, dimensions `size`) into `out`.
void addBox(engine::RenderMesh& out, Vec3 size, Vec3 c, Vec3 color) {
    engine::RenderMesh b = MeshBuilder::box(size);
    uint32_t base = static_cast<uint32_t>(out.vertices.size());
    for (engine::Vertex v : b.vertices) {
        v.position = v.position + c;
        v.color = color;
        out.vertices.push_back(v);
    }
    for (uint32_t i : b.indices) out.indices.push_back(base + i);
}

// A WIRE hoop of unit radius: a thin glowing band standing on edge (a short
// cylinder shell, double-sided), so from any angle it reads as a bright LINE
// circling the agent — not a flat polygon lying on the ground like a shadow.
// Instanced with scale (radius, 1, radius); the band height stays constant.
engine::RenderMesh ringXZ(int segs = 40, Real bandH = 0.10) {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    for (int i = 0; i < segs; ++i) {
        Real t0 = (Real(i) / segs) * 6.28318530718;
        Real t1 = (Real(i + 1) / segs) * 6.28318530718;
        Vec3 lo0(std::cos(t0), 0, std::sin(t0)), lo1(std::cos(t1), 0, std::sin(t1));
        Vec3 hi0 = lo0 + Vec3(0, bandH, 0), hi1 = lo1 + Vec3(0, bandH, 0);
        Vec3 n(std::cos((t0 + t1) * 0.5), 0, std::sin((t0 + t1) * 0.5));
        // Both windings so the band shows whichever side faces the camera.
        for (int side = 0; side < 2; ++side) {
            uint32_t b = static_cast<uint32_t>(m.vertices.size());
            Vec3 nn = side ? n * -1.0 : n;
            for (const Vec3& p : { lo0, lo1, hi1, hi0 }) {
                engine::Vertex v; v.position = p; v.normal = nn; v.color = white;
                m.vertices.push_back(v);
            }
            if (side == 0) {
                m.indices.push_back(b + 0); m.indices.push_back(b + 1); m.indices.push_back(b + 2);
                m.indices.push_back(b + 0); m.indices.push_back(b + 2); m.indices.push_back(b + 3);
            } else {
                m.indices.push_back(b + 0); m.indices.push_back(b + 2); m.indices.push_back(b + 1);
                m.indices.push_back(b + 0); m.indices.push_back(b + 3); m.indices.push_back(b + 2);
            }
        }
    }
    return m;
}

// A flat arrow along +Z from the origin to z=1 (a shaft + a head), facing +Y —
// the debug trajectory vector. Instanced with scale (width, 1, length) + yaw.
engine::RenderMesh arrowXZ() {
    engine::RenderMesh m;
    const Vec3 white(1, 1, 1);
    auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (const Vec3& p : { a, b, c, d }) {
            engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = white;
            m.vertices.push_back(v);
        }
        m.indices.push_back(base + 0); m.indices.push_back(base + 1); m.indices.push_back(base + 2);
        m.indices.push_back(base + 0); m.indices.push_back(base + 2); m.indices.push_back(base + 3);
    };
    quad(Vec3(-0.5, 0, 0), Vec3(0.5, 0, 0), Vec3(0.5, 0, 0.7), Vec3(-0.5, 0, 0.7));   // shaft
    // Arrow head (triangle) from z=0.6 to z=1.0.
    uint32_t b = static_cast<uint32_t>(m.vertices.size());
    for (const Vec3& p : { Vec3(-1.1, 0, 0.6), Vec3(1.1, 0, 0.6), Vec3(0, 0, 1.0) }) {
        engine::Vertex v; v.position = p; v.normal = Vec3(0, 1, 0); v.color = white;
        m.vertices.push_back(v);
    }
    m.indices.push_back(b + 0); m.indices.push_back(b + 1); m.indices.push_back(b + 2);
    return m;
}

// Debug colour for a behaviour state — unlit/emissive and marked as an OVERLAY so
// the backend draws it on top of world geometry (no depth occlusion).
RenderMaterial widgetMaterial(Vec3 color) {
    RenderMaterial m;
    // Bright HOLOGRAM look: strong emission (pops through bloom/tonemap), dark
    // base so lighting can't wash it toward a grey "shadow ring".
    m.albedo = color * 0.15; m.metallic = 0.0f; m.roughness = 1.0f; m.opacity = 1.0f;
    m.emission = color * 4.0; m.flags = RenderMaterial::FLAG_OVERLAY;
    return m;
}
// Traffic-light semantics (user-requested): GREEN = going, RED = stopped,
// AMBER = braking/avoiding, violet = turning, teal = following a leader.
Vec3 stateColor(Agent::State s) {
    switch (s) {
        // Pedestrian / shared
        case Agent::State::Walking:   return Vec3(0.15, 0.90, 0.25);   // green: go
        case Agent::State::Avoiding:  return Vec3(1.00, 0.55, 0.08);   // amber: stepping around
        case Agent::State::Waiting:   return Vec3(1.00, 0.10, 0.08);   // RED: held (signal/tether)
        // Driver FSM
        case Agent::State::Cruising:  return Vec3(0.15, 0.90, 0.25);   // green: go
        case Agent::State::Following: return Vec3(0.10, 0.65, 0.70);   // teal: pacing a leader
        case Agent::State::Yielding:  return Vec3(1.00, 0.55, 0.08);   // amber: braking for someone
        case Agent::State::Turning:   return Vec3(0.70, 0.35, 0.95);   // violet: arcing a node
        default:                      return Vec3(0.45, 0.45, 0.48);   // grey: resting/parked
    }
}

// A vertex-coloured car that mirrors the PLAYER's car body (vehicles.lua
// `car_body`): a low hull, a set-back greenhouse cabin, dark windshield + rear
// glass, and pale front / red rear corner lights — so NPC cars share the player's
// look. Wheels are part of the instanced mesh here (the player renders them as
// physics-driven entities). Body STYLE varies the hull/cabin for a mixed fleet.
// `size` is x=width, y=height, z=length (travel axis, +Z); faces +Z.
engine::RenderMesh buildCarMesh(int style, Vec3 color, Vec3 size) {
    const Real w = size.x, h = size.y, l = size.z;
    const Real hw = w * 0.5, hl = l * 0.5;
    const Vec3 glass(0.05, 0.06, 0.09);    // dark glass (matches car_body)
    const Vec3 tyre(0.04, 0.04, 0.05);
    const Vec3 head(1.0, 0.97, 0.82), tail(0.85, 0.06, 0.05);
    engine::RenderMesh m;

    switch (style) {
        case 1:  // hatchback: hull + cabin carried back to the tail, glass fore & aft
            addBox(m, Vec3(w, h * 0.46, l * 0.92), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.86, h * 0.44, l * 0.56), Vec3(0, h * 0.40, -l * 0.06), color);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.44, l * 0.12), glass);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.44, -l * 0.34), glass);
            break;
        case 2:  // SUV: tall hull, big greenhouse
            addBox(m, Vec3(w, h * 0.58, l), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.90, h * 0.46, l * 0.62), Vec3(0, h * 0.44, -l * 0.02), color);
            addBox(m, Vec3(w * 0.84, h * 0.34, l * 0.05), Vec3(0, h * 0.46, l * 0.20), glass);
            addBox(m, Vec3(w * 0.84, h * 0.34, l * 0.05), Vec3(0, h * 0.46, -l * 0.26), glass);
            break;
        case 3:  // pickup: forward cab + open bed
            addBox(m, Vec3(w, h * 0.42, l), Vec3(0, -h * 0.04, 0), color);
            addBox(m, Vec3(w * 0.90, h * 0.40, l * 0.34), Vec3(0, h * 0.34, l * 0.22), color);
            addBox(m, Vec3(w * 0.80, h * 0.26, l * 0.05), Vec3(0, h * 0.40, l * 0.38), glass);
            addBox(m, Vec3(w * 0.92, h * 0.20, l * 0.42), Vec3(0, h * 0.10, -l * 0.26), color); // bed walls
            break;
        case 4:  // van: one tall slab body, raked windshield, low nose
            addBox(m, Vec3(w, h * 0.72, l * 0.86), Vec3(0, h * 0.06, -l * 0.06), color);
            addBox(m, Vec3(w, h * 0.34, l * 0.20), Vec3(0, -h * 0.10, l * 0.40), color);       // nose
            addBox(m, Vec3(w * 0.86, h * 0.30, l * 0.05), Vec3(0, h * 0.22, l * 0.30), glass);  // windshield
            addBox(m, Vec3(w * 0.86, h * 0.24, l * 0.05), Vec3(0, h * 0.24, -l * 0.48), glass); // rear glass
            break;
        case 5:  // box truck: a small cab up front + a tall square cargo box
            addBox(m, Vec3(w, h * 0.44, l * 0.30), Vec3(0, -h * 0.04, l * 0.33), color);        // cab lower
            addBox(m, Vec3(w * 0.94, h * 0.40, l * 0.24), Vec3(0, h * 0.30, l * 0.35), color);  // cab roof
            addBox(m, Vec3(w * 0.84, h * 0.30, l * 0.05), Vec3(0, h * 0.30, l * 0.47), glass);  // windshield
            addBox(m, Vec3(w, h * 0.86, l * 0.62), Vec3(0, h * 0.12, -l * 0.17), color);        // cargo box
            break;
        default: // sedan — matches the player's car_body proportions exactly
            addBox(m, Vec3(w, h * 0.46, l), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.84, h * 0.42, l * 0.46), Vec3(0, h * 0.40, -l * 0.04), color);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.05), Vec3(0, h * 0.42, l * 0.15), glass);
            addBox(m, Vec3(w * 0.80, h * 0.26, l * 0.05), Vec3(0, h * 0.42, -l * 0.23), glass);
            break;
    }

    // Head/taillights at the corners (front +Z pale, rear -Z red) — like car_body.
    Real ly = -h * 0.08, lx = hw - 0.30;
    for (Real sx : { Real(1), Real(-1) }) {
        addBox(m, Vec3(0.34, 0.18, 0.10), Vec3(sx * lx, ly, hl - 0.05), head);
        addBox(m, Vec3(0.34, 0.18, 0.10), Vec3(sx * lx, ly, -hl + 0.05), tail);
    }

    // Four wheels (dark discs, thin along X), sat at the hull's lower edge.
    Real wr = h * 0.26, axleY = -h * 0.5 + wr, fz = l * 0.32, wxo = hw - 0.03;
    for (Real wx : { wxo, -wxo })
        for (Real wz : { fz, -fz })
            addBox(m, Vec3(0.12, wr * 2, wr * 2), Vec3(wx, axleY, wz), tyre);
    return m;
}

// The mesh style that draws each body TYPE (matches buildCarMesh's switch).
int styleForType(VehicleType t) {
    switch (t) {
        case VehicleType::Hatchback: return 1;
        case VehicleType::SUV:       return 2;
        case VehicleType::Pickup:    return 3;
        case VehicleType::Van:       return 4;
        case VehicleType::BoxTruck:  return 5;
        default:                     return 0;   // Sedan
    }
}

// A body slot's mesh size (x=width, y=height, z=length), from the shared fleet
// table (city_sim kFleet) — so the drawn car is exactly the size the sim follows
// and collides at.
Vec3 fleetBodySize(int slot) {
    const VehicleBody& b = vehicleFleetBody(slot);
    return Vec3(b.width, b.height, b.length);
}

// The car fleet's PAINT, one colour per fleet slot (mirrors city_sim kFleet slot
// for slot; body style + size come from the fleet body). Each slot is its own
// instance group (a group shares one mesh + material). Drivers spread across them
// by vehicle index, so a given car keeps its shape, size, and colour.
const Vec3 kCarColors[] = {
    Vec3(0.72, 0.10, 0.10), Vec3(0.10, 0.18, 0.52), Vec3(0.90, 0.90, 0.90),  // sedans
    Vec3(0.85, 0.78, 0.10), Vec3(0.10, 0.45, 0.30), Vec3(0.80, 0.40, 0.08),  // hatchbacks
    Vec3(0.09, 0.09, 0.11), Vec3(0.52, 0.53, 0.56), Vec3(0.30, 0.22, 0.14),  // SUVs
    Vec3(0.14, 0.30, 0.20),                                                  // pickup
    Vec3(0.62, 0.60, 0.42),                                                  // van (tan)
    Vec3(0.20, 0.42, 0.55),                                                  // box truck (teal)
};
constexpr int kNumCarVariants = static_cast<int>(sizeof(kCarColors) / sizeof(kCarColors[0]));

// The static signal assembly (pole/arm/head housing) carries its hue in vertex
// colour, like the rest of the city's street furniture.
RenderMaterial signalPostMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);
    m.metallic = 0.0f;
    m.roughness = 0.5f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

Vec3 translationOf(const Mat4& t) { return Vec3(t.m[0][3], t.m[1][3], t.m[2][3]); }

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

engine::RenderMesh fleetCarMesh(int slot) {
    int n = vehicleFleetSize();
    int s = ((slot % n) + n) % n;
    const VehicleBody& b = vehicleFleetBody(s);
    return buildCarMesh(styleForType(b.type), kCarColors[s % kNumCarVariants],
                        Vec3(b.width, b.height, b.length));
}

Real CityRenderSystem::groundAt(Real x, Real z) const {
    return (heightAt_ ? heightAt_(x, z) : 0.0) + roadLift_;
}

bool CityRenderSystem::build(World& world, AssetManager* assets) {
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
    // Warm up so the city is already ALIVE when the level appears — agents depart
    // and spread onto the roads instead of standing still for the first minute.
    for (int i = 0; i < 400; ++i) sim_.step(0.1, params_.hoursPerSecond);

    MeshHandle pedMesh{}, lensMesh{};
    if (assets) {
        pedMesh = assets->acquireMesh(MeshBuilder::box(params_.pedSize), "city:ped");
        Real e = params_.signalLensSize;
        lensMesh = assets->acquireMesh(MeshBuilder::box(Vec3(e, e, e)), "city:signal");
    }

    // One instance group per fleet slot; each is a body TYPE + SIZE (from the sim's
    // shared fleet table) painted its slot colour, so the traffic is a mixed fleet
    // of sedans, hatchbacks, SUVs, pickups, a van, and a box truck — every NPC car
    // built the same way as (and to scale with) the player's. Drivers spread across
    // the slots by vehicle index.
    // When a CityVehicleSystem owns the cars as real physics Vehicles (ADR-0061),
    // skip the instanced kinematic car bodies entirely — otherwise every car draws
    // twice. The CitySim still runs as the planner; the bridge reads its ghosts.
    carGroups_.clear();
    if (!carsExternallyOwned_)
        for (int v = 0; v < kNumCarVariants; ++v) {
            MeshHandle mh{};
            if (assets)
                mh = assets->acquireMesh(
                    buildCarMesh(styleForType(vehicleFleetBody(v).type), kCarColors[v],
                                 fleetBodySize(v)),
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

    // Crosswalks are painted into the ROAD TEXTURE now (ADR-0061): the road mesher
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

    // Debug widgets (ADR-0060): per-agent footprint ring (one group per behaviour
    // state, coloured) + a forward trajectory arrow. The groups always exist so a
    // key can toggle them at runtime; they stay empty unless debugWidgets_ is on.
    {
        MeshHandle ringMesh{}, arrowMesh{};
        if (assets) {
            ringMesh = assets->acquireMesh(ringXZ(), "city:dbgring");
            arrowMesh = assets->acquireMesh(arrowXZ(), "city:dbgarrow");
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
        InstanceGroup g;
        g.mesh = arrowMesh;
        g.material = widgetMaterial(Vec3(0.15, 0.85, 0.95));   // cyan trajectory
        world.add<InstanceGroup>(forwardGroup_, g);
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
            if (cars.empty()) continue;   // cars owned externally (ADR-0061 bridge)
            // Each driver keeps the same variant (keyed off its car index), so a
            // given car is always the same model + colour.
            int v = (a.vehicle >= 0 ? a.vehicle : 0) % kNumCarVariants;
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
        const auto& agents = sim_.agents();
        for (std::size_t ai = 0; ai < agents.size() && debugWidgets_; ++ai) {
            const Agent& a = agents[ai];
            bool car = a.mode == Agent::Mode::Driver;
            Real x = a.pos.x, z = a.pos.y;
            Vec2 heading = a.heading;
            // Where this agent is TRYING to go (the arrow's tip): the ghost's
            // short-horizon intent by default; the bridge's pursuit target when
            // the body is external.
            Vec2 goal = a.pos + a.heading * (2.0 + a.speed);
            // Externally-owned agents (ADR-0061): the widget must ring the
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
                        reported = true;
                        break;
                    }
                if (!reported) continue;
            }
            // A hologram HOOP at body height (waist for a walker, roof-line-ish
            // for a car) — clearly a debug marker circling the agent, never a
            // shadow lying on the pavement; the overlay flag draws it on top.
            Real y = groundAt(x, z) + a.elevation + (car ? 0.8 : 1.0);
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
            InstanceGroup* fg = foot[static_cast<int>(a.state)];
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
                    Vec3(0.12, 1, len)));
            }
        }
        for (int s = 0; s < kStateCount; ++s) refreshBounds(foot[s]);
        refreshBounds(fwd);
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
    sim_.setExternalObstacles(std::move(obstacles));

    step(ctx.world, ctx.clock.fixedStep());
}

}  // namespace citysim
