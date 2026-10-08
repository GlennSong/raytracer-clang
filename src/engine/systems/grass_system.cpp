#include "grass_system.h"
#include "../components.h"
#include "../../job_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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
constexpr int kMaxInFlight = 4;           // tile jobs at once: keeps ahead of a runner with several fields (ADR-0133), leaves the pool
constexpr double kKeepFar = 0.4;          // the outer ring's share of the clumps (RenderMaterial::keepFar)
constexpr double kGroundStep = 1.0;       // the tile's ground grid (m): the drawn terrain's own resolution

// THE TILE: a jittered grid of clumps, thinned by density, a variant each. Both rings use the
// SAME grid; each clump has a random rank, and the outer ring keeps only the ranks under keepFar
// -- exactly the clumps the shader has not yet shrunk away at the boundary (the thinning band
// ends there), so a tile changing ring changes nothing on screen. The ground is sampled once
// on a 1 m grid over the tile (plus a margin for the slope) and interpolated: the ground and
// density functions are the expensive part, and the drawn terrain is 1 m cells anyway.
GrassSystem::Built buildTile(const GrassField& f, const GrassSystem::Key& key, int lod, uint64_t generation) {
    GrassSystem::Built b;
    b.key = key;
    b.lod = lod;
    b.generation = generation;
    const auto [layer, ix, iz] = key;   // layer = field * 2 + (1: its cards)
    const bool cards = lod == 2;
    const double T = (layer & 1) ? f.cardTile : f.tile;
    const std::size_t nv = cards ? 1 : f.clumps.size();
    b.perVariant.resize(nv);
    const double x0 = ix * T, z0 = iz * T;
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
    const double step = cards ? f.cardSpacing : f.spacing;
    const int n = std::max(1, static_cast<int>(std::floor(T / step)));
    double ySum = 0.0;
    int yCount = 0;
    const uint64_t tileHash = mix(f.seed ^ static_cast<uint64_t>(layer) * 0x9E37ull ^
                                  mix(static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32 ^ static_cast<uint32_t>(iz)));
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
            const double s = cards ? (0.8 + 0.5 * unit(mix(h ^ 0x3Dull))) * (0.7 + 0.3 * dens)
                                   : (0.8 + 0.45 * unit(mix(h ^ 0x3Dull))) * (0.75 + 0.25 * dens);   // thinner grass is shorter
            const double hs = f.height ? f.height(x, z) : 1.0;   // mown here, wild there
            if (hs < 0.04) continue;
            Mat4 m = Mat4::trs(Vec3(x, y, z), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s * hs, s));
            m.m[3][0] = static_cast<Real>(rank);   // the rank rides in the bottom row (mesh.vert reads it)
            b.perVariant[cards ? 0 : mix(h ^ 0xC3ull) % nv].push_back(m);
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
    // SEVERAL FIELDS (ADR-0130: meadow grass, tall grass, reeds...): each keeps its own tiles, the
    // tile key's layer being field * 2 + (1 for its cards)
    std::vector<GrassField*> fields;
    ctx.world.each<GrassField>([&](Entity, GrassField& f) { if (!f.clumps.empty() && f.ground) fields.push_back(&f); });
    if (fields.empty()) return;
    const Vec3 cam = ctx.view.camera.position;
    // the layers: tile size, and the band of distances a tile must reach into to be wanted
    struct Layer { double T, inner, reach; bool on; };
    std::vector<Layer> layers;
    for (const GrassField* fp : fields) {
        layers.push_back({fp->tile, -1.0, fp->radius + fp->tile, true});                                   // clumps; a tile ahead
        layers.push_back({fp->cardTile, fp->cardFadeIn - 2.0, fp->cardRadius + fp->cardTile, fp->card.valid()});   // cards
    }
    const int nLayers = static_cast<int>(layers.size());
    auto fieldOf = [&](int layer) -> const GrassField& { return *fields[static_cast<std::size_t>(layer / 2)]; };
    auto dists = [&](const Key& k, double& nearest, double& farthest) {   // XZ, camera to the tile's square
        const auto [layer, ix, iz] = k;
        const double T = layers[layer].T, x0 = ix * T, z0 = iz * T;
        const double dx = std::max({x0 - cam.x, 0.0, cam.x - (x0 + T)});
        const double dz = std::max({z0 - cam.z, 0.0, cam.z - (z0 + T)});
        nearest = std::sqrt(dx * dx + dz * dz);
        const double fx = std::max(std::fabs(cam.x - x0), std::fabs(cam.x - (x0 + T)));
        const double fz = std::max(std::fabs(cam.z - z0), std::fabs(cam.z - (z0 + T)));
        farthest = std::sqrt(fx * fx + fz * fz);
    };
    auto nearestOf = [&](const Key& k) { double n, fa; dists(k, n, fa); return n; };
    auto ringOf = [&](int layer, double d) { return (layer & 1) ? 2 : (d < fieldOf(layer).nearRadius ? 0 : 1); };

    // 1. COMMIT what the jobs finished (dropping results for tiles that left, or from before a stop)
    std::vector<Built> done;
    {
        std::lock_guard<std::mutex> lock(inbox_->m);
        done.swap(inbox_->done);
    }
    inFlight_ -= static_cast<int>(done.size());
    std::sort(done.begin(), done.end(), [&](const Built& a, const Built& b) { return nearestOf(a.key) < nearestOf(b.key); });
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Built> later;
    for (Built& b : done) {
        auto it = tiles_.find(b.key);
        if (b.generation != generation_ || it == tiles_.end() || it->second.building != b.lod) continue;
        if (!commitAll_ && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > kCommitBudgetMs) {
            later.push_back(std::move(b));   // next frame
            continue;
        }
        if (std::get<0>(b.key) >= nLayers) continue;   // a field that went away
        const GrassField& f = fieldOf(std::get<0>(b.key));
        Tile& tile = it->second;
        drop(ctx.world, tile);   // the old ring's instances go only now, as the new ones arrive
        tile.lod = b.lod;
        tile.building = -1;
        const bool cards = b.lod == 2;
        const double T = layers[std::get<0>(b.key)].T;
        for (std::size_t v = 0; v < b.perVariant.size(); ++v) {
            if (b.perVariant[v].empty()) continue;
            InstanceGroup g;
            if (cards) {
                g.mesh = f.card;
                g.material = f.cardMaterial;
                g.drawDistance = f.cardRadius + 2.0;
            } else {
                g.mesh = f.clumps[v];
                g.material = f.material;
                // the thinning band ends where tiles change ring, so the swap is invisible
                g.material.thinStart = static_cast<float>(f.nearRadius - 8.0);
                g.material.thinEnd = static_cast<float>(f.nearRadius);
                g.material.keepFar = static_cast<float>(kKeepFar);
                g.material.growFar = 1.3f;
                g.drawDistance = f.fadeEnd + 2.0;
            }
            g.transforms = std::move(b.perVariant[v]);
            g.boundsCenter = b.centre;
            g.boundsRadius = T * 0.75 + 2.0;
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

    // 2. DROP what left its layer's band (a little hysteresis)
    for (auto it = tiles_.begin(); it != tiles_.end();) {
        const int layer = std::get<0>(it->first);
        double n = 0, fa = 0;
        if (layer < nLayers) dists(it->first, n, fa);
        if (layer >= nLayers || !layers[layer].on || n > layers[layer].reach + layers[layer].T * 0.5 ||
            fa < layers[layer].inner - layers[layer].T * 0.5) {
            drop(ctx.world, it->second);
            it = tiles_.erase(it);
        } else ++it;
    }

    // 3. START what is wanted, nearest first: tiles missing, or in the wrong ring
    std::vector<std::pair<double, std::pair<Key, int>>> want;   // distance, (tile, ring)
    for (int layer = 0; layer < nLayers; ++layer) {
        const Layer& L = layers[layer];
        if (!L.on) continue;
        const int i0 = static_cast<int>(std::floor((cam.x - L.reach) / L.T)), i1 = static_cast<int>(std::floor((cam.x + L.reach) / L.T));
        const int k0 = static_cast<int>(std::floor((cam.z - L.reach) / L.T)), k1 = static_cast<int>(std::floor((cam.z + L.reach) / L.T));
        for (int iz = k0; iz <= k1; ++iz)
            for (int ix = i0; ix <= i1; ++ix) {
                const Key key{layer, ix, iz};
                double n, fa;
                dists(key, n, fa);
                if (n > L.reach || fa < L.inner) continue;
                const int ring = ringOf(layer, n);
                auto it = tiles_.find(key);
                if (it != tiles_.end() && (it->second.building == ring || (it->second.building < 0 && it->second.lod == ring))) continue;
                want.push_back({n, {key, ring}});
            }
    }
    std::sort(want.begin(), want.end());
    if (first_) {   // the first frame: everything, here and now (in parallel)
        std::vector<Built> built(want.size());
        ctx.jobs.parallelFor(0, want.size(), [&](std::size_t k) {
            built[k] = buildTile(fieldOf(std::get<0>(want[k].second.first)), want[k].second.first, want[k].second.second, generation_);
        });
        {
            std::lock_guard<std::mutex> lock(inbox_->m);
            for (std::size_t k = 0; k < want.size(); ++k) {
                tiles_[want[k].second.first].building = built[k].lod;
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
    std::vector<std::shared_ptr<GrassField>> copies(fields.size());   // the jobs' own: a field may be replaced meanwhile
    for (const auto& [d, what] : want) {
        if (inFlight_ >= kMaxInFlight) break;
        const auto& [key, ring] = what;
        Tile& tile = tiles_[key];
        tile.building = ring;
        ++inFlight_;
        std::shared_ptr<GrassField>& fieldCopy = copies[static_cast<std::size_t>(std::get<0>(key) / 2)];
        if (!fieldCopy) fieldCopy = std::make_shared<GrassField>(fieldOf(std::get<0>(key)));
        ctx.jobs.run([inbox = inbox_, fieldCopy, key = key, lod = ring, gen = generation_] {
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
