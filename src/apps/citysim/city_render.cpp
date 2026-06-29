#include "city_render.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/city/road_net.h"

#include <algorithm>
#include <cmath>

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
    m.albedo = Vec3(0.15, 0.35, 0.75);
    m.metallic = 0.6f;
    m.roughness = 0.35f;
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

    MeshHandle carMesh{}, pedMesh{}, lensMesh{};
    if (assets) {
        carMesh = assets->acquireMesh(MeshBuilder::box(params_.carSize), "city:car");
        pedMesh = assets->acquireMesh(MeshBuilder::box(params_.pedSize), "city:ped");
        Real e = params_.signalLensSize;
        lensMesh = assets->acquireMesh(MeshBuilder::box(Vec3(e, e, e)), "city:signal");
    }

    carGroup_ = world.create();
    { InstanceGroup g; g.mesh = carMesh; g.material = carMaterial(); world.add<InstanceGroup>(carGroup_, g); }
    pedGroup_ = world.create();
    { InstanceGroup g; g.mesh = pedMesh; g.material = pedMaterial(); world.add<InstanceGroup>(pedGroup_, g); }
    for (int s = 0; s < 3; ++s) {
        signalGroups_[s] = world.create();
        InstanceGroup g;
        g.mesh = lensMesh;
        g.material = signalMaterial(static_cast<SignalState>(s));
        world.add<InstanceGroup>(signalGroups_[s], g);
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

Mat4 CityRenderSystem::signalLensPose(int link) const {
    const engine::NavLink& L = nav_.links[link];
    Vec2 head = nav_.nodes[L.to];           // the junction node this approach enters
    Vec2 dir = nav_.direction(link);
    Vec2 right(dir.y, -dir.x);
    // Lens sits at the stop line, just past the right kerb, on a pole.
    Real backoff = 1.5;
    Real side = L.width * 0.5 + 1.0;
    Vec2 p = head - dir * backoff + right * side;
    Real y = groundAt(p.x, p.y) + L.layer * Real(5.8) + params_.signalHeight;
    return Mat4::trs(Vec3(p.x, y, p.y), Quat(), Vec3(1, 1, 1));
}

void CityRenderSystem::syncGroups(World& world) {
    InstanceGroup* car = world.get<InstanceGroup>(carGroup_);
    InstanceGroup* ped = world.get<InstanceGroup>(pedGroup_);
    InstanceGroup* sig[3];
    for (int s = 0; s < 3; ++s) sig[s] = world.get<InstanceGroup>(signalGroups_[s]);

    if (car) car->transforms.clear();
    if (ped) ped->transforms.clear();
    for (int s = 0; s < 3; ++s) if (sig[s]) sig[s]->transforms.clear();

    for (const Agent& a : sim_.agents()) {
        InstanceGroup* g = (a.mode == Agent::Mode::Driver) ? car : ped;
        if (g) g->transforms.push_back(agentPose(a));
    }

    // Each signalled approach gets a lens in the group for its CURRENT state, so a
    // phase change re-buckets the lens into the matching emissive batch.
    SignalController& sc = sim_.signals();
    for (int li = 0; li < nav_.linkCount(); ++li) {
        if (!sc.hasSignal(li)) continue;
        int s = static_cast<int>(sc.stateForLink(li));
        if (sig[s]) sig[s]->transforms.push_back(signalLensPose(li));
    }

    refreshBounds(car);
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
    step(ctx.world, ctx.clock.fixedStep());
}

}  // namespace citysim
