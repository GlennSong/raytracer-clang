#include "grass_system.h"
#include "../components.h"
#include "../../job_system.h"
#include "../../log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

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
constexpr int kMinInFlight = 4;           // tile jobs at once, at least (ADR-0133); half the pool where it is bigger
constexpr double kSpanCell = 64.0;        // m: the grid of the ground's range a tile's 3D distance is read off
constexpr double kSpanSlack = 6.0;        // m: the blades' height and what 3x3 samples on a 64 m cell can miss
constexpr double kLookAhead = 0.75;       // s: tiles are started nearest to where the camera is now OR will be then
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
    auto sample = [&](double x, double z, double& y, double& slopeCos, Vec3& groundN) {
        const double fx = (x - x0) / kGroundStep + 1.0, fz = (z - z0) / kGroundStep + 1.0;
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fz));
        const double u = fx - i, v = fz - j;
        const double h00 = H(i, j), h10 = H(i + 1, j), h01 = H(i, j + 1), h11 = H(i + 1, j + 1);
        y = (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
        const double gx = ((h10 - h00) * (1 - v) + (h11 - h01) * v) / kGroundStep;
        const double gz = ((h01 - h00) * (1 - u) + (h11 - h10) * u) / kGroundStep;
        slopeCos = 1.0 / std::sqrt(1.0 + gx * gx + gz * gz);
        groundN = Vec3(-gx, 1.0, -gz) * slopeCos;
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
            Vec3 groundN;
            sample(x, z, y, slopeCos, groundN);
            const double dens = f.density ? f.density(x, z, y, slopeCos) : 1.0;
            if (dens <= 0.0 || unit(mix(h ^ 0x51ull)) >= dens) continue;
            const double yaw = unit(mix(h ^ 0xA7ull)) * 6.283185307;
            const double s = cards ? (0.8 + 0.5 * unit(mix(h ^ 0x3Dull))) * (0.7 + 0.3 * dens)
                                   : (0.8 + 0.45 * unit(mix(h ^ 0x3Dull))) * (0.75 + 0.25 * dens);   // thinner grass is shorter
            const double hs = f.height ? f.height(x, z) : 1.0;   // mown here, wild there
            if (hs < 0.04) continue;
            Mat4 m = Mat4::trs(Vec3(x, y, z), Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s * hs, s));
            if (f.alignToGround) {   // (clutter) tilted to the ground under it, about cross(up, n)
                const Vec3 n = groundN * (1.0 / std::max(1e-9, groundN.length()));
                const Vec3 axis = cross(Vec3(0, 1, 0), n);
                const double al = axis.length();
                const Quat toGround = al > 1e-6 ? Quat::fromAxisAngle(axis * (1.0 / al), std::acos(std::clamp(n.y, -1.0, 1.0))) : Quat();
                m = Mat4::trs(Vec3(x, y, z), toGround * Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(s, s, s));
            }
            m.m[3][0] = static_cast<Real>(rank);   // the rank rides in the bottom row (mesh.vert reads it)...
            m.m[3][1] = static_cast<Real>(groundN.x);   // ...and the ground's normal (its x and z: it looks up), the
            m.m[3][2] = static_cast<Real>(groundN.z);   // field shaded as the slope it grows on (mesh.frag)
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
    struct SelfTime {   // RT_GRASS_LOG: this system's own time on the main thread
        LogWindow& w;
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        ~SelfTime() { const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); w.selfMs += ms; w.selfMax = std::max(w.selfMax, ms); ++w.frames; }
    } selfTime{window_};
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
    // IN 3D, as the shader fades the grass (mesh.vert: distance to the planted origin), so from 300 m up no clump
    // tile is within its 62 m and none is built. A tile's ground spans [lo, hi], read off a coarse grid of the
    // ground's range (kSpanCell squares, 3x3 samples each, once): every field grows on the level's one drawn
    // ground (level_loader), and over the city a sample of it costs ~50 us -- five per tile, per layer, was
    // 0.4 s a second at 200 m/s.
    const auto& ground = fields.front()->ground;
    auto cellSpan = [&](int cx, int cz) -> const std::pair<float, float>& {
        const std::pair<int, int> c{cx, cz};
        auto it = span_.find(c);
        if (it != span_.end()) return it->second;
        const auto s0 = std::chrono::steady_clock::now();
        double lo = 1e30, hi = -1e30;
        for (int j = 0; j < 3; ++j)
            for (int i = 0; i < 3; ++i) {
                const double y = ground((cx + 0.5 * i) * kSpanCell, (cz + 0.5 * j) * kSpanCell);
                lo = std::min(lo, y); hi = std::max(hi, y);
            }
        ++window_.spans;
        window_.spanMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count();
        return span_.emplace(c, std::pair{static_cast<float>(lo - kSpanSlack), static_cast<float>(hi + kSpanSlack)}).first->second;
    };
    auto groundSpan = [&](const Key& k) {
        const auto [layer, ix, iz] = k;
        const double T = layers[layer].T, x0 = ix * T, z0 = iz * T;
        std::pair<float, float> r{1e30f, -1e30f};
        for (int cz = static_cast<int>(std::floor(z0 / kSpanCell)); cz <= static_cast<int>(std::floor((z0 + T) / kSpanCell)); ++cz)
            for (int cx = static_cast<int>(std::floor(x0 / kSpanCell)); cx <= static_cast<int>(std::floor((x0 + T) / kSpanCell)); ++cx) {
                const auto& [lo, hi] = cellSpan(cx, cz);
                r.first = std::min(r.first, lo); r.second = std::max(r.second, hi);
            }
        return r;
    };
    auto distsFrom = [&](const Vec3& c, const Key& k, double& nearest, double& farthest) {   // c to the tile's box
        const auto [layer, ix, iz] = k;
        const double T = layers[layer].T, x0 = ix * T, z0 = iz * T;
        const double dx = std::max({x0 - c.x, 0.0, c.x - (x0 + T)});
        const double dz = std::max({z0 - c.z, 0.0, c.z - (z0 + T)});
        const double fx = std::max(std::fabs(c.x - x0), std::fabs(c.x - (x0 + T)));
        const double fz = std::max(std::fabs(c.z - z0), std::fabs(c.z - (z0 + T)));
        nearest = std::sqrt(dx * dx + dz * dz);
        farthest = std::sqrt(fx * fx + fz * fz);
        if (nearest > layers[layer].reach + T) return;   // far across the map: no need of its ground
        const auto [lo, hi] = groundSpan(k);
        const double dy = std::max({lo - c.y, 0.0, c.y - hi}), fy = std::max(std::fabs(c.y - lo), std::fabs(c.y - hi));
        nearest = std::sqrt(nearest * nearest + dy * dy);
        farthest = std::sqrt(farthest * farthest + fy * fy);
    };
    auto dists = [&](const Key& k, double& nearest, double& farthest) { distsFrom(cam, k, nearest, farthest); };
    auto nearestOf = [&](const Key& k) { double n, fa; dists(k, n, fa); return n; };
    // where the camera will be (its velocity, smoothed over a few frames; a jump resets it)
    if (haveLastCam_ && ctx.frameDelta > 1e-4) {
        const Vec3 v = (cam - lastCam_) * (1.0 / ctx.frameDelta);
        camVel_ = v.length() > 400.0 ? Vec3(0, 0, 0) : camVel_ * 0.8 + v * 0.2;
    }
    lastCam_ = cam;
    haveLastCam_ = true;
    const Vec3 ahead = cam + camVel_ * kLookAhead;
    auto priorityOf = [&](const Key& k, double nowNearest) { double n, fa; distsFrom(ahead, k, n, fa); return std::min(nowNearest, n); };
    auto ringOf = [&](int layer, double d) { return (layer & 1) ? 2 : (d < fieldOf(layer).nearRadius ? 0 : 1); };

    auto mark = [&, t = std::chrono::steady_clock::now()](double& acc) mutable {
        const auto now = std::chrono::steady_clock::now();
        acc += std::chrono::duration<double, std::milli>(now - t).count();
        t = now;
    };
    mark(window_.setupMs);
    // 1. COMMIT what the jobs finished (dropping results for tiles that left, or from before a stop)
    std::vector<Built> done;
    {
        std::lock_guard<std::mutex> lock(inbox_->m);
        done.swap(inbox_->done);
    }
    inFlight_ -= static_cast<int>(done.size());
    for (const Built& b : done) {   // RT_GRASS_LOG's window (a result put back for next frame counts once: its ms are zeroed)
        if (b.buildMs <= 0.0) continue;
        ++window_.built;
        window_.buildMs += b.buildMs;
        window_.maxMs = std::max(window_.maxMs, b.buildMs);
        const std::size_t l = static_cast<std::size_t>(std::get<0>(b.key));
        if (window_.builtByLayer.size() <= l) window_.builtByLayer.resize(l + 1, 0);
        ++window_.builtByLayer[l];
    }
    std::sort(done.begin(), done.end(), [&](const Built& a, const Built& b) { return nearestOf(a.key) < nearestOf(b.key); });
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Built> later;
    for (Built& b : done) {
        auto it = tiles_.find(b.key);
        if (b.generation != generation_ || it == tiles_.end() || it->second.building != b.lod) continue;
        if (!commitAll_ && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > kCommitBudgetMs) {
            b.buildMs = 0.0;
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

    mark(window_.commitMs);
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

    mark(window_.dropMs);
    double maxReach = 0.0;
    for (const Layer& L : layers) maxReach = std::max(maxReach, L.reach);
    if (span_.size() > 2048)   // the ground of cells long left behind (a lap of the island would keep ~50k)
        for (auto it = span_.begin(); it != span_.end();) {
            const double cx = (it->first.first + 0.5) * kSpanCell, cz = (it->first.second + 0.5) * kSpanCell;
            if (std::hypot(cx - cam.x, cz - cam.z) > maxReach + 4.0 * kSpanCell) it = span_.erase(it);
            else ++it;
        }

    // 3. START what is wanted, nearest first (now or kLookAhead from now): tiles missing, or in the wrong ring
    struct Want { double priority, nearest; Key key; int ring; };
    std::vector<Want> want;
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
                want.push_back({priorityOf(key, n), n, key, ring});
            }
    }
    std::sort(want.begin(), want.end(), [](const Want& a, const Want& b) { return a.priority < b.priority; });
    mark(window_.scanMs);
    // RT_GRASS_LOG=1: once a second, the tiles that should be drawn and have nothing yet (bare ground where grass
    // belongs: a pop when they arrive), the nearest of them, and the tile-seconds bare over the run
    static const bool grassLog = [] { const char* e = std::getenv("RT_GRASS_LOG"); return e && e[0] == '1'; }();
    if (grassLog && !first_) {
        int bare = 0;
        double nearest = -1.0;
        for (const Want& w : want) {
            const double d = w.nearest;
            const Layer& L = layers[std::get<0>(w.key)];
            auto t = tiles_.find(w.key);
            if (d > L.reach - L.T || (t != tiles_.end() && t->second.lod >= 0)) continue;   // the margin tile, or covered
            ++bare;
            if (nearest < 0.0 || d < nearest) nearest = d;
        }
        bareSeconds_ += bare * ctx.frameDelta;
        logClock_ += ctx.frameDelta;
        if (logClock_ >= 1.0) {
            logClock_ = 0.0;
            std::vector<int> wantByLayer(static_cast<std::size_t>(nLayers), 0);
            for (const Want& w : want) ++wantByLayer[static_cast<std::size_t>(std::get<0>(w.key))];
            std::string byLayer;
            for (int l = 0; l < nLayers; ++l) {
                const int built = l < static_cast<int>(window_.builtByLayer.size()) ? window_.builtByLayer[static_cast<std::size_t>(l)] : 0;
                if (!wantByLayer[static_cast<std::size_t>(l)] && !built) continue;
                byLayer += " L" + std::to_string(l) + " T" + std::to_string(static_cast<int>(layers[static_cast<std::size_t>(l)].T)) +
                           " want " + std::to_string(wantByLayer[static_cast<std::size_t>(l)]) + " built " + std::to_string(built) + ";";
            }
            LOG_INFO << "[grass] bare " << bare << " tiles, nearest " << nearest << " m, " << bareSeconds_
                     << " tile-s bare in all, " << inFlight_ << " building; last second: started " << window_.started << ", built "
                     << window_.built << " (mean " << (window_.built ? window_.buildMs / window_.built : 0.0) << " ms, max "
                     << window_.maxMs << " ms), field copies " << window_.copyMs << " ms; main thread " << (window_.frames ? window_.selfMs / window_.frames : 0.0)
                     << " ms a frame (max " << window_.selfMax << ") = setup " << window_.setupMs << " commit " << window_.commitMs << " drop "
                     << window_.dropMs << " scan " << window_.scanMs << " ms over the second, " << window_.spans << " ground spans in "
                     << window_.spanMs << " ms; tiles " << tiles_.size() << ", spans kept " << span_.size() << ";" << byLayer;
            window_ = LogWindow{};
        }
    }
    if (first_) {   // the first frame: everything, here and now (in parallel)
        std::vector<Built> built(want.size());
        ctx.jobs.parallelFor(0, want.size(), [&](std::size_t k) {
            built[k] = buildTile(fieldOf(std::get<0>(want[k].key)), want[k].key, want[k].ring, generation_);
        });
        {
            std::lock_guard<std::mutex> lock(inbox_->m);
            for (std::size_t k = 0; k < want.size(); ++k) {
                tiles_[want[k].key].building = built[k].lod;
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
    const int maxInFlight = std::max(kMinInFlight, static_cast<int>(ctx.jobs.workerCount() / 2));
    for (const Want& w : want) {
        if (inFlight_ >= maxInFlight) break;
        const Key& key = w.key;
        const int ring = w.ring;
        Tile& tile = tiles_[key];
        tile.building = ring;
        ++inFlight_;
        std::shared_ptr<GrassField>& fieldCopy = copies[static_cast<std::size_t>(std::get<0>(key) / 2)];
        if (!fieldCopy) {
            const auto c0 = std::chrono::steady_clock::now();
            fieldCopy = std::make_shared<GrassField>(fieldOf(std::get<0>(key)));
            window_.copyMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
        }
        ++window_.started;
        ctx.jobs.run([inbox = inbox_, fieldCopy, key = key, lod = ring, gen = generation_] {
            const auto b0 = std::chrono::steady_clock::now();
            Built b = buildTile(*fieldCopy, key, lod, gen);
            b.buildMs = std::max(1e-6, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count());
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
    span_.clear();
    haveLastCam_ = false;
    camVel_ = Vec3(0, 0, 0);
    first_ = true;
    commitAll_ = false;
}

}  // namespace engine
