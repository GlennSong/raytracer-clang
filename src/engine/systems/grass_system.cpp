#include "grass_system.h"
#include "../components.h"
#include "../../job_system.h"

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
constexpr double kCommitBudgetMs = 1.0;   // creating a tile's groups is cheap; this caps a burst
constexpr int kMaxInFlight = 4;           // tile jobs at once: keeps ahead of a runner, leaves the pool
constexpr double kKeepFar = 0.4;          // the outer ring's share of the clumps (RenderMaterial::keepFar)
constexpr double kGroundStep = 1.0;       // the tile's ground grid (m): the drawn terrain's own resolution

// THE TILE: a jittered grid of clumps, thinned by density, a variant each. Both rings use the
// SAME grid; each clump has a random rank, and the outer ring keeps only the ranks under keepFar
// -- exactly the clumps the shader has not yet shrunk away at the boundary (the thinning band
// ends there), so a tile changing ring changes nothing on screen. The ground is sampled once
// on a 1 m grid over the tile (plus a margin for the slope) and interpolated: the ground and
// density functions are the expensive part, and the drawn terrain is 1 m cells anyway.
GrassSystem::Built buildTile(const GrassField& f, std::pair<int, int> key, int lod, uint64_t generation) {
    GrassSystem::Built b;
    b.key = key;
    b.lod = lod;
    b.generation = generation;
    const double T = f.tile;
    const std::size_t nv = f.clumps.size();
    b.perVariant.resize(nv);
    const double x0 = key.first * T, z0 = key.second * T;
    const int g = static_cast<int>(std::ceil(T / kGroundStep)) + 3;   // one sample of margin each side
    std::vector<double> hgt(static_cast<std::size_t>(g) * g);
    for (int j = 0; j < g; ++j)
        for (int i = 0; i < g; ++i)
            hgt[static_cast<std::size_t>(j) * g + i] = f.ground(x0 + (i - 1) * kGroundStep, z0 + (j - 1) * kGroundStep);
    auto H = [&](int i, int j) { return hgt[static_cast<std::size_t>(std::clamp(j, 0, g - 1)) * g + std::clamp(i, 0, g - 1)]; };
    auto sample = [&](double x, double z, double& y, double& slopeCos) {
        const double fx = (x - x0) / kGroundStep + 1.0, fz = (z - z0) / kGroundStep + 1.0;
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fz));
        const double u = fx - i, v = fz - j;
        const double h00 = H(i, j), h10 = H(i + 1, j), h01 = H(i, j + 1), h11 = H(i + 1, j + 1);
        y = (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
        const double gx = ((h10 - h00) * (1 - v) + (h11 - h01) * v) / kGroundStep;
        const double gz = ((h01 - h00) * (1 - u) + (h11 - h10) * u) / kGroundStep;
        slopeCos = 1.0 / std::sqrt(1.0 + gx * gx + gz * gz);
    };
    const double step = f.spacing;
    const int n = std::max(1, static_cast<int>(std::floor(T / step)));
    double ySum = 0.0;
    int yCount = 0;
    const uint64_t tileHash = mix(f.seed ^ mix(static_cast<uint64_t>(static_cast<uint32_t>(key.first)) << 32 ^ static_cast<uint32_t>(key.second)));
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const uint64_t h = mix(tileHash ^ (static_cast<uint64_t>(j) << 20 | static_cast<uint64_t>(i)));
            const double x = x0 + (i + unit(h)) * step;
            const double z = z0 + (j + unit(mix(h))) * step;
            const double rank = unit(mix(h ^ 0x77ull));
            if (lod == 1 && rank >= kKeepFar) continue;
            double y, slopeCos;
            sample(x, z, y, slopeCos);
            const double dens = f.density ? f.density(x, z, y, slopeCos) : 1.0;
            if (dens <= 0.0 || unit(mix(h ^ 0x51ull)) >= dens) continue;
            const double yaw = unit(mix(h ^ 0xA7ull)) * 6.283185307;
            const double s = (0.8 + 0.45 * unit(mix(h ^ 0x3Dull))) * (0.75 + 0.25 * dens);   // thinner grass is shorter
            Mat4 m = Mat4::trs(Vec3(x, y, z), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s, s));
            m.m[3][0] = static_cast<Real>(rank);   // the rank rides in the bottom row (mesh.vert reads it)
            b.perVariant[mix(h ^ 0xC3ull) % nv].push_back(m);
            ySum += y;
            ++yCount;
        }
    b.centre = Vec3(x0 + T * 0.5, yCount ? ySum / yCount : H(g / 2, g / 2), z0 + T * 0.5);
    return b;
}
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
    const double reach = f.radius + T;   // build a tile ahead of the radius: ready before it is due
    auto nearestDist = [&](int ix, int iz) {   // XZ distance from the camera to the tile's square
        const double x0 = ix * T, z0 = iz * T;
        const double dx = std::max({x0 - cam.x, 0.0, cam.x - (x0 + T)});
        const double dz = std::max({z0 - cam.z, 0.0, cam.z - (z0 + T)});
        return std::sqrt(dx * dx + dz * dz);
    };
    auto ringOf = [&](double d) { return d < f.nearRadius ? 0 : 1; };

    // 1. COMMIT what the jobs finished (dropping results for tiles that left, or from before a stop)
    std::vector<Built> done;
    {
        std::lock_guard<std::mutex> lock(inbox_->m);
        done.swap(inbox_->done);
    }
    inFlight_ -= static_cast<int>(done.size());
    std::sort(done.begin(), done.end(), [&](const Built& a, const Built& b) {
        return nearestDist(a.key.first, a.key.second) < nearestDist(b.key.first, b.key.second);
    });
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Built> later;
    for (Built& b : done) {
        auto it = tiles_.find(b.key);
        if (b.generation != generation_ || it == tiles_.end() || it->second.building != b.lod) continue;
        if (!commitAll_ && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > kCommitBudgetMs) {
            later.push_back(std::move(b));   // next frame
            continue;
        }
        Tile& tile = it->second;
        drop(ctx.world, tile);   // the old ring's clumps go only now, as the new ones arrive
        tile.lod = b.lod;
        tile.building = -1;
        for (std::size_t v = 0; v < b.perVariant.size(); ++v) {
            if (b.perVariant[v].empty()) continue;
            InstanceGroup g;
            g.mesh = f.clumps[v];
            g.material = f.material;
            // the thinning band ends where tiles change ring, so the swap is invisible
            g.material.thinStart = static_cast<float>(f.nearRadius - 8.0);
            g.material.thinEnd = static_cast<float>(f.nearRadius);
            g.material.keepFar = static_cast<float>(kKeepFar);
            g.material.growFar = 1.3f;
            g.transforms = std::move(b.perVariant[v]);
            g.boundsCenter = b.centre;
            g.boundsRadius = T * 0.75 + 2.0;
            g.drawDistance = f.fadeEnd + 2.0;
            const Entity e = ctx.world.create();
            ctx.world.add<InstanceGroup>(e, std::move(g));
            tile.groups.push_back(e);
        }
    }
    if (!later.empty()) {   // back in the inbox, still counted as in flight
        std::lock_guard<std::mutex> lock(inbox_->m);
        for (Built& b : later) inbox_->done.push_back(std::move(b));
        inFlight_ += static_cast<int>(later.size());
    }

    // 2. DROP what left the reach (a little hysteresis)
    for (auto it = tiles_.begin(); it != tiles_.end();) {
        if (nearestDist(it->first.first, it->first.second) > reach + T * 0.5) {
            drop(ctx.world, it->second);
            it = tiles_.erase(it);
        } else ++it;
    }

    // 3. START what is wanted, nearest first: tiles missing, or in the wrong ring
    std::vector<std::pair<double, std::pair<int, int>>> want;
    const int i0 = static_cast<int>(std::floor((cam.x - reach) / T)), i1 = static_cast<int>(std::floor((cam.x + reach) / T));
    const int k0 = static_cast<int>(std::floor((cam.z - reach) / T)), k1 = static_cast<int>(std::floor((cam.z + reach) / T));
    for (int iz = k0; iz <= k1; ++iz)
        for (int ix = i0; ix <= i1; ++ix) {
            const double d = nearestDist(ix, iz);
            if (d > reach) continue;
            auto it = tiles_.find({ix, iz});
            const int ring = ringOf(d);
            if (it != tiles_.end() && (it->second.building == ring || (it->second.building < 0 && it->second.lod == ring))) continue;
            want.push_back({d, {ix, iz}});
        }
    std::sort(want.begin(), want.end());
    if (first_) {   // the first frame: everything, here and now (in parallel)
        std::vector<Built> built(want.size());
        auto fieldCopy = f;
        ctx.jobs.parallelFor(0, want.size(), [&](std::size_t k) {
            built[k] = buildTile(fieldCopy, want[k].second, ringOf(want[k].first), generation_);
        });
        {
            std::lock_guard<std::mutex> lock(inbox_->m);
            for (std::size_t k = 0; k < want.size(); ++k) {
                tiles_[want[k].second].building = built[k].lod;
                inbox_->done.push_back(std::move(built[k]));
                ++inFlight_;
            }
        }
        first_ = false;
        commitAll_ = true;
        update(ctx);   // commit them this frame, all of them
        commitAll_ = false;
        return;
    }
    for (const auto& [d, key] : want) {
        if (inFlight_ >= kMaxInFlight) break;
        Tile& tile = tiles_[key];
        tile.building = ringOf(d);
        ++inFlight_;
        auto fieldCopy = std::make_shared<GrassField>(f);   // the job's own: the field may be replaced meanwhile
        ctx.jobs.run([inbox = inbox_, fieldCopy, key, lod = tile.building, gen = generation_] {
            Built b = buildTile(*fieldCopy, key, lod, gen);
            std::lock_guard<std::mutex> lock(inbox->m);
            inbox->done.push_back(std::move(b));
        });
    }
}

void GrassSystem::onStop(FrameContext& ctx) {
    for (auto& [key, t] : tiles_) drop(ctx.world, t);
    tiles_.clear();
    ++generation_;
    // jobs still running land in the old inbox: give them their own and start clean
    inbox_ = std::make_shared<Inbox>();
    inFlight_ = 0;
    first_ = true;
    commitAll_ = false;
}

}  // namespace engine
