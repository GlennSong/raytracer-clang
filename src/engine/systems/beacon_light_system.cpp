#include "beacon_light_system.h"

#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp, beaconBlinkGate, beaconCellPhase
#include "../world.h"
#include "../../log.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

namespace {
// The baked glow: a radial falloff with a hot core, alpha only (RGB white).
std::vector<unsigned char> bakeGlow(int n) {
    std::vector<unsigned char> px(static_cast<std::size_t>(n) * n * 4, 255);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            const double u = (x + 0.5) / n - 0.5, v = (y + 0.5) / n - 0.5;
            const double r = std::sqrt(u * u + v * v) / 0.5;   // 0 centre .. 1 edge
            double a = r >= 1.0 ? 0.0 : std::pow(1.0 - r, 1.7);
            if (r < 0.16) a = 1.0;                              // the filament
            else a = std::max(a, 0.55 * std::pow(1.0 - r, 3.0) + a * 0.45);
            px[(static_cast<std::size_t>(y) * n + x) * 4 + 3] = static_cast<unsigned char>(std::min(255.0, a * 255.0));
        }
    return px;
}
}  // namespace

void BeaconLightSystem::gather(const CityBuildings& cb) {
    lamps_.clear();
    for (const BuildingRecord& r : cb.records)
        for (const Vec3& p : r.beacons) {
            Lamp l;
            l.pos = p;
            beaconCellPhase(static_cast<int>(std::floor(p.x / 24.0)), static_cast<int>(std::floor(p.z / 24.0)),
                            l.period, l.phase);
            lamps_.push_back(l);
        }
    recordsSeen_ = cb.records.size();
    LOG_INFO << "[beacons] " << lamps_.size() << " aviation lamps on " << cb.records.size() << " records";
}

void BeaconLightSystem::ensureGroup(FrameContext& ctx) {
    if (haveGroup_) return;
    // A unit quad in the XY plane facing +Z, centred: the billboard.
    RenderMesh q;
    auto vtx = [&](Real x, Real y, float u, float v) {
        Vertex vt(Vec3(x, y, 0), Vec3(0, 0, 1), Vec3(1, 0, 0), u, v);
        vt.color = Vec3(1, 1, 1);
        q.vertices.push_back(vt);
    };
    vtx(-0.5, -0.5, 0, 0); vtx(0.5, -0.5, 1, 0); vtx(0.5, 0.5, 1, 1); vtx(-0.5, 0.5, 0, 1);
    q.indices = {0, 1, 2, 0, 2, 3};
    quad_ = ctx.assets.acquireMesh(q, "beacon:sprite");
    const std::vector<unsigned char> px = bakeGlow(GLOW_TEX);
    glow_ = ctx.renderer.uploadTexture(GLOW_TEX, GLOW_TEX, 4, px.data());
    InstanceGroup g;
    g.mesh = quad_;
    g.material.albedo = Vec3(0, 0, 0);          // nothing but the emission
    g.material.metallic = 0.0f;
    g.material.roughness = 1.0f;
    g.material.opacity = 0.98f;                 // < 1: the transparent pass; the map shapes it
    g.material.albedoMap = glow_;
    g.material.flags |= RenderMaterial::FLAG_ALPHA_FROM_MAP | RenderMaterial::FLAG_TWO_SIDED;
    g.material.emission = Vec3(0, 0, 0);
    g.drawClass = DrawClass::Effect;
    g.drawDistance = SPRITE_FAR + 100.0;
    group_ = ctx.world.create();
    ctx.world.add<InstanceGroup>(group_, std::move(g));
    haveGroup_ = true;
}

void BeaconLightSystem::update(FrameContext& ctx) {
    auto& lighting = ctx.view.lighting;
    lighting.beaconPoints.clear();
    const CityBuildings* cb = nullptr;
    ctx.world.each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
    if (!cb) {
        if (haveGroup_)
            if (InstanceGroup* g = ctx.world.get<InstanceGroup>(group_)) g->transforms.clear();
        lamps_.clear();
        recordsSeen_ = static_cast<std::size_t>(-1);
        return;
    }
    if (cb->records.size() != recordsSeen_) gather(*cb);
    if (lamps_.empty()) return;
    ensureGroup(ctx);
    InstanceGroup* g = ctx.world.get<InstanceGroup>(group_);
    if (!g) return;

    // The tier distances: the level's light LOD block, else the defaults.
    Real spriteIn = DEFAULT_SPRITE_IN, lightM = DEFAULT_LIGHT_M, lightRange = DEFAULT_LIGHT_RANGE;
    int lightCount = DEFAULT_LIGHT_COUNT;
    ctx.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
        spriteIn = c.lightSpriteIn;
        lightM = c.lightRadius;
        lightRange = c.lightRange;
        lightCount = c.lightCount;
    });
    const Real spriteFade0 = spriteIn * 0.4;   // fades in over the last 60 % of spriteIn
    const Real ramp = duskRamp(lighting.solarElevation);
    const Real adapt = std::max(1.0f, lighting.nightAdapt);
    const double seconds = lighting.beaconSeconds;
    const auto& cam = ctx.view.camera;
    Vec3 fwd = cam.target - cam.position;
    if (fwd.lengthSquared() < 1e-9) fwd = Vec3(0, 0, 1);
    fwd = normalize(fwd);
    Vec3 right = cross(fwd, cam.up);
    if (right.lengthSquared() < 1e-9) right = Vec3(1, 0, 0);
    right = normalize(right);
    const Vec3 up = normalize(cross(right, fwd));

    // The group's single emission: the beacon red at the on-screen level
    // (divided by the night adaptation like the grown lamps), on the dusk
    // ramp. Per-lamp flash and fade go into each instance's SCALE.
    g->material.emission = Vec3(1.0, 0.05, 0.02) * (2.4 * ramp / adapt);
    g->transforms.clear();
    g->boundsCenter = cam.position;
    g->boundsRadius = static_cast<Real>(SPRITE_FAR + 50.0);
    if (ramp <= 0) return;

    order_.clear();
    for (std::size_t i = 0; i < lamps_.size(); ++i) {
        const Real d2 = (lamps_[i].pos - cam.position).lengthSquared();
        if (d2 > SPRITE_FAR * SPRITE_FAR) continue;
        order_.push_back({d2, i});
    }
    // Back to front: the transparent pass blends instances in order.
    std::sort(order_.begin(), order_.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    for (const auto& [d2, i] : order_) {
        const Lamp& l = lamps_[i];
        const Real d = std::sqrt(d2);
        const Real gate = beaconBlinkGate(seconds, l.period, l.phase);
        if (gate <= 0.001) continue;
        Real nearT = (d - spriteFade0) / std::max(Real(0.5), spriteIn - spriteFade0);
        nearT = std::min(Real(1), std::max(Real(0), nearT));
        nearT = nearT * nearT * (3.0 - 2.0 * nearT);
        // 1.4 m at arm's length, growing ~1 m per 80 m so the glow holds up
        // on the skyline; the gate scales it so a lamp between flashes is gone.
        const Real size = (1.4 + d * 0.0125) * nearT * (0.55 + 0.45 * gate);
        if (size <= 0.01) continue;
        Mat4 m;
        for (int r = 0; r < 3; ++r) {
            m.m[r][0] = (r == 0 ? right.x : r == 1 ? right.y : right.z) * size;
            m.m[r][1] = (r == 0 ? up.x : r == 1 ? up.y : up.z) * size;
            m.m[r][2] = (r == 0 ? -fwd.x : r == 1 ? -fwd.y : -fwd.z) * size;   // +Z of the quad faces the camera
        }
        m.m[0][3] = l.pos.x; m.m[1][3] = l.pos.y; m.m[2][3] = l.pos.z;
        g->transforms.push_back(m);
    }

    // The near tier: red point lights on the closest lamps.
    std::sort(order_.begin(), order_.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    int lights = 0;
    for (const auto& [d2, i] : order_) {
        if (lights >= lightCount) break;
        const Real d = std::sqrt(d2);
        if (d > lightM) break;
        const Lamp& l = lamps_[i];
        const Real gate = beaconBlinkGate(seconds, l.period, l.phase);
        Real fade = (lightM - d) / (0.25 * lightM);
        fade = std::min(Real(1), std::max(Real(0), fade));
        // A tight pool: the shader's window is (1 - (d/range)^4)^2 / d^2, so a
        // short range with the intensity to match reads as a sharp red disc
        // instead of a fuzzy wash (Glenn, 2026-09-14).
        const Real k = (lightRange * lightRange) / (DEFAULT_LIGHT_RANGE * DEFAULT_LIGHT_RANGE);
        PointLight pl(l.pos, Vec3(1.0, 0.06, 0.03), static_cast<float>(64.0 * k * ramp * gate * fade));
        pl.range = static_cast<float>(lightRange);
        lighting.beaconPoints.push_back(pl);
        ++lights;
    }
}

void BeaconLightSystem::onStop(FrameContext& ctx) {
    ctx.view.lighting.beaconPoints.clear();
    if (haveGroup_ && ctx.world.alive(group_)) ctx.world.destroy(group_);
    if (haveGroup_) ctx.assets.releaseMesh(quad_);
    haveGroup_ = false;
    lamps_.clear();
    recordsSeen_ = static_cast<std::size_t>(-1);
}

}  // namespace engine
