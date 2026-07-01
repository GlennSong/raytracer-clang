#include "city_render.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/city/road_net.h"
#include "../../engine/procgen/city/street_kit.h"   // trafficSignalProto, SignalParams

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

// A unit quad in the XZ plane (1x1, centred, facing +Y), white. Instanced per
// crosswalk bar with a translate+yaw+scale transform, so the standard
// InstanceGroup bounds/culling apply (a single baked world mesh would need
// hand-set bounds or it gets frustum-culled to nothing).
engine::RenderMesh unitQuadXZ() {
    engine::RenderMesh m;
    const Vec3 white(0.9, 0.9, 0.88);
    Vec3 corners[4] = { Vec3(-0.5, 0, -0.5), Vec3(0.5, 0, -0.5),
                        Vec3(0.5, 0, 0.5), Vec3(-0.5, 0, 0.5) };
    for (const Vec3& p : corners) {
        engine::Vertex v;
        v.position = p;
        v.normal = Vec3(0, 1, 0);
        v.color = white;
        m.vertices.push_back(v);
    }
    m.indices = { 0, 1, 2, 0, 2, 3 };
    return m;
}

// A vertex-coloured car in one of a few body STYLES (sedan / hatchback / SUV /
// pickup) with shared detailing — bumpers, head/tail lights, glass, and disc-ish
// wheels — so the traffic reads as a varied fleet rather than boxes on wheels.
// `size` is x=width, y=height, z=length (travel axis, +Z).
engine::RenderMesh buildCarMesh(int style, Vec3 color, Vec3 size) {
    const Real w = size.x, h = size.y, l = size.z;
    const Vec3 glass(0.12, 0.14, 0.18);    // dark windows
    const Vec3 trim(0.08, 0.08, 0.09);     // bumpers / dark trim
    const Vec3 tyre(0.04, 0.04, 0.05);     // near-black tyres
    const Vec3 hub(0.55, 0.56, 0.58);      // wheel hub
    const Vec3 head(0.95, 0.94, 0.80);     // headlight
    const Vec3 tail(0.75, 0.06, 0.05);     // taillight
    engine::RenderMesh m;

    // Style-specific body + greenhouse.
    switch (style) {
        case 1:  // hatchback: shorter, cabin carried back to the tail
            addBox(m, Vec3(w, h * 0.52, l * 0.92), Vec3(0, -h * 0.06, 0), color);
            addBox(m, Vec3(w * 0.94, h * 0.40, l * 0.56), Vec3(0, h * 0.24, -l * 0.06), color);
            addBox(m, Vec3(w * 0.80, h * 0.30, l * 0.48), Vec3(0, h * 0.34, -l * 0.08), glass);
            break;
        case 2:  // SUV: tall boxy body, big greenhouse
            addBox(m, Vec3(w, h * 0.60, l), Vec3(0, 0, 0), color);
            addBox(m, Vec3(w * 0.96, h * 0.44, l * 0.66), Vec3(0, h * 0.40, -l * 0.02), color);
            addBox(m, Vec3(w * 0.82, h * 0.32, l * 0.58), Vec3(0, h * 0.46, -l * 0.02), glass);
            break;
        case 3:  // pickup: forward cab, low open bed behind
            addBox(m, Vec3(w, h * 0.44, l), Vec3(0, -h * 0.12, 0), color);
            addBox(m, Vec3(w * 0.94, h * 0.40, l * 0.34), Vec3(0, h * 0.22, l * 0.22), color);
            addBox(m, Vec3(w * 0.80, h * 0.26, l * 0.28), Vec3(0, h * 0.30, l * 0.22), glass);
            addBox(m, Vec3(w * 0.94, h * 0.22, l * 0.44), Vec3(0, h * 0.02, -l * 0.26), color); // bed walls
            break;
        default: // sedan: low body, hood + boot, set-back cabin
            addBox(m, Vec3(w, h * 0.50, l), Vec3(0, -h * 0.06, 0), color);
            addBox(m, Vec3(w * 0.94, h * 0.38, l * 0.46), Vec3(0, h * 0.22, -l * 0.02), color);
            addBox(m, Vec3(w * 0.80, h * 0.28, l * 0.40), Vec3(0, h * 0.32, -l * 0.03), glass);
            break;
    }

    // Shared detailing (all styles). Front is +Z.
    addBox(m, Vec3(w * 0.98, h * 0.16, l * 0.05), Vec3(0, -h * 0.24, l * 0.49), trim);   // front bumper
    addBox(m, Vec3(w * 0.98, h * 0.16, l * 0.05), Vec3(0, -h * 0.24, -l * 0.49), trim);  // rear bumper
    addBox(m, Vec3(w * 0.16, h * 0.10, l * 0.03), Vec3(w * 0.36, -h * 0.02, l * 0.50), head);
    addBox(m, Vec3(w * 0.16, h * 0.10, l * 0.03), Vec3(-w * 0.36, -h * 0.02, l * 0.50), head);
    addBox(m, Vec3(w * 0.16, h * 0.10, l * 0.03), Vec3(w * 0.36, -h * 0.02, -l * 0.50), tail);
    addBox(m, Vec3(w * 0.16, h * 0.10, l * 0.03), Vec3(-w * 0.36, -h * 0.02, -l * 0.50), tail);

    // Wheels: a dark tyre disc (thin along X) with a lighter hub.
    Real wr = h * 0.30, wx = w * 0.5, wz = l * 0.32, wy = -h * 0.30;
    Real tw = w * 0.10;                    // tyre thickness
    const Vec3 wheelCtr[4] = { Vec3(wx, wy, wz), Vec3(-wx, wy, wz),
                               Vec3(wx, wy, -wz), Vec3(-wx, wy, -wz) };
    for (const Vec3& c : wheelCtr) {
        addBox(m, Vec3(tw, wr * 2, wr * 2), c, tyre);
        Real hx = (c.x > 0 ? -1 : 1) * tw * 0.35;   // hub set slightly inboard
        addBox(m, Vec3(tw * 0.6, wr * 0.9, wr * 0.9), Vec3(c.x + hx, c.y, c.z), hub);
    }
    return m;
}

// The car fleet: a curated set of (body style, paint colour) variants. Each is
// its own instance group (an InstanceGroup shares one mesh + material, so colour
// + shape variety means several groups). Drivers are spread across these.
struct CarVariant { int style; Vec3 color; };
const CarVariant kCarVariants[] = {
    {0, Vec3(0.72, 0.10, 0.10)},   // red sedan
    {0, Vec3(0.10, 0.18, 0.52)},   // blue sedan
    {0, Vec3(0.90, 0.90, 0.90)},   // white sedan
    {1, Vec3(0.85, 0.78, 0.10)},   // yellow hatchback
    {1, Vec3(0.10, 0.45, 0.30)},   // green hatchback
    {1, Vec3(0.80, 0.40, 0.08)},   // orange hatchback
    {2, Vec3(0.09, 0.09, 0.11)},   // black SUV
    {2, Vec3(0.52, 0.53, 0.56)},   // silver SUV
    {2, Vec3(0.30, 0.22, 0.14)},   // brown SUV
    {3, Vec3(0.14, 0.30, 0.20)},   // forest-green pickup
    {3, Vec3(0.62, 0.60, 0.42)},   // tan pickup
    {3, Vec3(0.20, 0.42, 0.55)},   // teal pickup
};
constexpr int kNumCarVariants = static_cast<int>(sizeof(kCarVariants) / sizeof(kCarVariants[0]));

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

RenderMaterial crosswalkMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);            // bright paint; vertex colour carries the white
    m.metallic = 0.0f;
    m.roughness = 0.9f;
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

    MeshHandle pedMesh{}, lensMesh{};
    if (assets) {
        pedMesh = assets->acquireMesh(MeshBuilder::box(params_.pedSize), "city:ped");
        Real e = params_.signalLensSize;
        lensMesh = assets->acquireMesh(MeshBuilder::box(Vec3(e, e, e)), "city:signal");
    }

    // One instance group per car variant (body style + paint); drivers are spread
    // across them so the traffic isn't all identical sedans.
    carGroups_.clear();
    for (int v = 0; v < kNumCarVariants; ++v) {
        MeshHandle mh{};
        if (assets)
            mh = assets->acquireMesh(
                buildCarMesh(kCarVariants[v].style, kCarVariants[v].color, params_.carSize),
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

    // Zebra crosswalks: a band of white bars across the mouth of every junction
    // approach, each bar an instance of the unit quad riding ~4 cm above the
    // asphalt. Opaque white, so it reads as a crossing and covers the painted
    // centreline where it crosses (no need to shrink the road's lane lines). Bars
    // run ALONG travel (depth), repeated ACROSS the carriageway.
    crosswalkCenters_.clear();
    MeshHandle quadMesh{};
    if (assets) quadMesh = assets->acquireMesh(unitQuadXZ(), "city:crosswalk");
    crosswalkGroup_ = world.create();
    {
        InstanceGroup g;
        g.mesh = quadMesh;
        g.material = crosswalkMaterial();
        const Real depth = 2.6, barW = 0.55, gap = 0.5, lift = 0.04;
        for (int li = 0; li < nav_.linkCount(); ++li) {
            const engine::NavLink& L = nav_.links[li];
            if (!nav_.isJunction(L.to)) continue;
            Vec2 d = nav_.direction(li);
            Vec2 node = nav_.nodes[L.to];
            Vec2 right(d.y, -d.x);
            Real halfW = L.width * 0.5;
            Vec2 center = node - d * (halfW + depth * 0.5 + 0.3);   // just outside the mouth
            crosswalkCenters_.push_back(center);
            Real y = groundAt(center.x, center.y) + L.layer * Real(5.8) + lift;
            Real yaw = std::atan2(d.x, d.y);   // unit quad local +Z -> travel direction
            Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
            for (Real lat = -halfW + barW * 0.5; lat <= halfW; lat += barW + gap) {
                Vec2 c = center + right * lat;
                // scale: local X (lateral) = bar width, local Z (travel) = band depth.
                g.transforms.push_back(Mat4::trs(Vec3(c.x, y, c.y), rot, Vec3(barW, 1, depth)));
            }
        }
        refreshBounds(&g);
        world.add<InstanceGroup>(crosswalkGroup_, g);
    }

    built_ = true;
    syncGroups(world);
    return true;
}

Mat4 CityRenderSystem::agentPose(const Agent& a) const {
    bool car = a.mode == Agent::Mode::Driver;
    Real x = a.pos.x, z = a.pos.y;          // Vec2 maps to world XZ (.y = world z)
    Real halfH = (car ? params_.carSize.y : params_.pedSize.y) * 0.5;
    Real y = groundAt(x, z) + a.elevation + halfH;   // a.elevation lifts bridge traffic
    Real yaw = std::atan2(a.heading.x, a.heading.y); // box local +Z -> travel heading
    Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
    return Mat4::trs(Vec3(x, y, z), rot, Vec3(1, 1, 1));
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
            // Each driver keeps the same variant (keyed off its car index), so a
            // given car is always the same model + colour.
            int v = (a.vehicle >= 0 ? a.vehicle : 0) % kNumCarVariants;
            if (cars[v]) cars[v]->transforms.push_back(agentPose(a));
        } else if (ped) {
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
}

void CityRenderSystem::step(World& world, Real dt) {
    if (!built_) return;
    sim_.step(dt, params_.hoursPerSecond);
    syncGroups(world);
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
