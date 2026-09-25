#include "grass_system.h"
#include "../components.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace engine {

namespace {
uint64_t mix(uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}
double unit(uint64_t h) { return static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0); }
constexpr double kBuildBudgetMs = 2.0;
constexpr double kKeepFar = 0.4;   // the outer ring's share of the clumps (RenderMaterial::keepFar)
}  // namespace

void GrassSystem::drop(World& world, Tile& t) {
    for (Entity e : t.groups)
        if (world.alive(e)) world.destroy(e);
    t.groups.clear();
}

void GrassSystem::update(FrameContext& ctx) {
    GrassField* field = nullptr;
    ctx.world.each<GrassField>([&](Entity, GrassField& f) { if (!field) field = &f; });
    if (!field || field->clumps.empty() || !field->ground) return;
    const GrassField& f = *field;
    const Vec3 cam = ctx.view.camera.position;
    const double T = f.tile;
    auto nearestDist = [&](int ix, int iz) {   // XZ distance from the camera to the tile's square
        const double x0 = ix * T, z0 = iz * T;
        const double dx = std::max({x0 - cam.x, 0.0, cam.x - (x0 + T)});
        const double dz = std::max({z0 - cam.z, 0.0, cam.z - (z0 + T)});
        return std::sqrt(dx * dx + dz * dz);
    };
    // what should exist, nearest first
    std::vector<std::pair<double, std::pair<int, int>>> want;
    const int i0 = static_cast<int>(std::floor((cam.x - f.radius) / T)), i1 = static_cast<int>(std::floor((cam.x + f.radius) / T));
    const int k0 = static_cast<int>(std::floor((cam.z - f.radius) / T)), k1 = static_cast<int>(std::floor((cam.z + f.radius) / T));
    for (int iz = k0; iz <= k1; ++iz)
        for (int ix = i0; ix <= i1; ++ix) {
            const double d = nearestDist(ix, iz);
            if (d <= f.radius) want.push_back({d, {ix, iz}});
        }
    std::sort(want.begin(), want.end());
    // drop what left the radius
    for (auto it = tiles_.begin(); it != tiles_.end();) {
        if (nearestDist(it->first.first, it->first.second) > f.radius + T * 0.5) {   // a little hysteresis
            drop(ctx.world, it->second);
            it = tiles_.erase(it);
        } else ++it;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::size_t nv = f.clumps.size();
    for (const auto& [d, key] : want) {
        const int lod = d < f.nearRadius ? 0 : 1;
        auto found = tiles_.find(key);
        if (found != tiles_.end() && found->second.lod == lod) continue;
        if (!first_ && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > kBuildBudgetMs)
            break;
        Tile& tile = tiles_[key];
        drop(ctx.world, tile);
        tile.lod = lod;
        // the tile's clumps: a jittered grid, thinned by density, a variant each. Both rings use
        // the SAME grid; each clump has a random rank, and the outer ring keeps only the ranks
        // under keepFar -- exactly the clumps the shader has not yet shrunk away at the boundary
        // (the thinning band ends there), so a tile changing ring changes nothing on screen.
        const double step = f.spacing;
        const int n = std::max(1, static_cast<int>(std::floor(T / step)));
        std::vector<std::vector<Mat4>> per(nv);
        double ySum = 0.0; int yCount = 0;
        const uint64_t tileHash = mix(f.seed ^ mix(static_cast<uint64_t>(static_cast<uint32_t>(key.first)) << 32 ^ static_cast<uint32_t>(key.second)));
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const uint64_t h = mix(tileHash ^ (static_cast<uint64_t>(j) << 20 | static_cast<uint64_t>(i)));
                const double x = key.first * T + (i + unit(h)) * step;
                const double z = key.second * T + (j + unit(mix(h))) * step;
                const double rank = unit(mix(h ^ 0x77ull));
                if (lod == 1 && rank >= kKeepFar) continue;
                const double dens = f.density ? f.density(x, z) : 1.0;
                if (dens <= 0.0 || unit(mix(h ^ 0x51ull)) >= dens) continue;
                const double y = f.ground(x, z);
                const double yaw = unit(mix(h ^ 0xA7ull)) * 6.283185307;
                const double s = (0.8 + 0.45 * unit(mix(h ^ 0x3Dull))) * (0.75 + 0.25 * dens);   // thinner grass is shorter
                Mat4 m = Mat4::trs(Vec3(x, y, z), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s, s));
                m.m[3][0] = static_cast<Real>(rank);   // the rank rides in the bottom row (mesh.vert reads it)
                per[mix(h ^ 0xC3ull) % nv].push_back(m);
                ySum += y; ++yCount;
            }
        const Vec3 centre(key.first * T + T * 0.5, yCount ? ySum / yCount : cam.y, key.second * T + T * 0.5);
        for (std::size_t v = 0; v < nv; ++v) {
            if (per[v].empty()) continue;
            InstanceGroup g;
            g.mesh = f.clumps[v];
            g.material = f.material;
            // the thinning band ends where tiles change ring, so the swap is invisible
            g.material.thinStart = static_cast<float>(f.nearRadius - 8.0);
            g.material.thinEnd = static_cast<float>(f.nearRadius);
            g.material.keepFar = static_cast<float>(kKeepFar);
            g.material.growFar = 1.3f;
            g.transforms = std::move(per[v]);
            g.boundsCenter = centre;
            g.boundsRadius = T * 0.75 + 2.0;
            g.drawDistance = f.fadeEnd + 2.0;
            const Entity e = ctx.world.create();
            ctx.world.add<InstanceGroup>(e, std::move(g));
            tile.groups.push_back(e);
        }
    }
    first_ = false;
}

void GrassSystem::onStop(FrameContext& ctx) {
    for (auto& [key, t] : tiles_) drop(ctx.world, t);
    tiles_.clear();
    first_ = true;
}

}  // namespace engine
