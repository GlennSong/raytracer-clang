#include "traffic_system.h"

#include "../asset_manager.h"
#include "../components.h"
#include "../mesh_builder.h"
#include "../procgen/city/road_net.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

RenderMaterial carMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(0.15, 0.35, 0.75);   // painted body
    m.metallic = 0.6f;
    m.roughness = 0.35f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

RenderMaterial pedMaterial() {
    RenderMaterial m;
    m.albedo = Vec3(0.80, 0.55, 0.40);   // clothing/skin-ish
    m.metallic = 0.0f;
    m.roughness = 0.85f;
    m.opacity = 1.0f;
    m.emission = Vec3(0, 0, 0);
    m.flags = 0;
    return m;
}

Vec3 translationOf(const Mat4& t) { return Vec3(t.m[0][3], t.m[1][3], t.m[2][3]); }

}  // namespace

bool TrafficSystem::build(World& world, AssetManager* assets) {
    // Merge every RoadNet's sampled+constrained graph into one combined graph, so
    // a level with several road entities yields one crowd. (Separate road
    // networks stay separate connected components — agents route within one.)
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

    nav_ = buildNavGraph(combined);
    if (nav_.linkCount() == 0) return false;

    sim_.build(nav_, params_.cars, params_.pedestrians, params_.seed);

    MeshHandle carMesh{};
    MeshHandle pedMesh{};
    if (assets) {
        carMesh = assets->acquireMesh(MeshBuilder::box(params_.carSize), "traffic:car");
        pedMesh = assets->acquireMesh(MeshBuilder::box(params_.pedSize), "traffic:ped");
    }

    carGroup_ = world.create();
    {
        InstanceGroup g;
        g.mesh = carMesh;
        g.material = carMaterial();
        world.add<InstanceGroup>(carGroup_, g);
    }
    pedGroup_ = world.create();
    {
        InstanceGroup g;
        g.mesh = pedMesh;
        g.material = pedMaterial();
        world.add<InstanceGroup>(pedGroup_, g);
    }

    built_ = true;
    syncGroups(world);
    return true;
}

Mat4 TrafficSystem::poseOf(const Agent& a) const {
    Real x = a.pos.x;
    Real z = a.pos.y;                       // Vec2 maps to world XZ (.y = world z)
    Real ground = heightAt_ ? heightAt_(x, z) : 0.0;
    Real halfH = (a.kind == AgentKind::Car ? params_.carSize.y : params_.pedSize.y) * 0.5;
    Real y = ground + roadLift_ + a.elevation + halfH;   // a.elevation lifts bridge traffic
    // Yaw turns the box's local +Z (its length axis) to face the travel heading.
    Real yaw = std::atan2(a.heading.x, a.heading.y);
    Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
    return Mat4::trs(Vec3(x, y, z), rot, Vec3(1, 1, 1));
}

void TrafficSystem::syncGroups(World& world) {
    InstanceGroup* car = world.get<InstanceGroup>(carGroup_);
    InstanceGroup* ped = world.get<InstanceGroup>(pedGroup_);
    if (car) car->transforms.clear();
    if (ped) ped->transforms.clear();
    for (const Agent& a : sim_.agents()) {
        InstanceGroup* g = (a.kind == AgentKind::Car) ? car : ped;
        if (g) g->transforms.push_back(poseOf(a));
    }
    // Refresh coarse cull bounds each step (the crowd is in motion). Pad by a few
    // metres so a mesh straddling the bound isn't popped.
    auto refresh = [](InstanceGroup* g) {
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
    };
    refresh(car);
    refresh(ped);
}

void TrafficSystem::step(World& world, Real dt) {
    if (!built_) return;
    sim_.step(dt, params_.hoursPerSecond);
    syncGroups(world);
}

void TrafficSystem::fixedUpdate(FrameContext& ctx) {
    if (!built_) {
        build(ctx.world, &ctx.assets);   // lazy: retry until the level's roads exist
        return;
    }
    step(ctx.world, ctx.clock.fixedStep());
}

}  // namespace engine
