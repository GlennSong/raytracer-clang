#include "underwater_system.h"

#include "../components.h"
#include "../procgen/hydrology.h"
#include "../world.h"

#include <cmath>

namespace engine {

UnderwaterSystem::Surface UnderwaterSystem::surfaceAt(World& world, double x, double z) {
    Surface s;
    world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& cfg) {
        if (s.kind != Water::None || !cfg.params.hydro) return;
        const Hydrology& h = *cfg.params.hydro;
        const double lake = h.lakeLevelAt(x, z);
        if (std::isfinite(lake)) { s = {Water::Lake, lake}; return; }
        double level = 0.0;
        if (h.distanceToRiver(x, z, 30.0, &level) <= 0.0 && std::isfinite(level)) s = {Water::River, level};
    });
    if (s.kind != Water::None) return s;
    world.each<Sea>([&](Entity, Sea& sea) {
        if (s.kind == Water::None && sea.contains(x, z)) s = {Water::Sea, sea.level};
    });
    return s;
}

void UnderwaterSystem::update(FrameContext& ctx) {
    UnderwaterParams& u = ctx.view.lighting.underwater;
    const Vec3 eye = ctx.view.camera.position;
    const Surface s = surfaceAt(ctx.world, eye.x, eye.z);
    u.active = s.kind != Water::None && eye.y < s.level - 0.02;
    if (!u.active) return;
    u.surfaceY = static_cast<float>(s.level);
    switch (s.kind) {
        case Water::Sea:   u.color = Vec3(0.03, 0.16, 0.18); u.visibility = 14.0f; break;   // clear blue-green
        case Water::Lake:  u.color = Vec3(0.04, 0.12, 0.08); u.visibility = 7.0f;  break;   // still, greener
        case Water::River: u.color = Vec3(0.06, 0.12, 0.08); u.visibility = 4.5f;  break;   // silty
        default: break;
    }
}

}  // namespace engine
