#include "island_world.h"

#include "../noise.h"
#include "../terrain.h"
#include "../city/terrain_route.h"
#include "../city/roads/lanes/interchange.h"
#include "../city/roads/lanes/vertical_profile.h"   // profileAlong: the deck as the builder will make it   // diamondRamps: the one way on and off a freeway
#include "../city/plan/land_shape.h"             // isoLines: contours, the coastline
#include "place_names.h"
#include "../../level_params.h"   // readTerrainParams: the island's terrain block, as a level reads it

#include <tinygltf/stb_image_write.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <thread>
#include <tuple>

namespace engine {

using json = nlohmann::json;

namespace {
constexpr double kPi = 3.14159265358979;

double rnd(uint32_t seed, int k) {   // a deterministic [0, 1) per (seed, k)
    uint32_t x = seed * 747796405u + static_cast<uint32_t>(k) * 2891336453u + 0x9E3779B9u;
    x ^= x >> 16; x *= 0x7FEB352Du; x ^= x >> 15; x *= 0x846CA68Bu; x ^= x >> 16;
    return (x >> 8) / 16777216.0;
}
Vec2 fromAxis(double u, double v, double a) { return Vec2(u * std::cos(a) - v * std::sin(a), u * std::sin(a) + v * std::cos(a)); }
double wrap360(double d) { d = std::fmod(d, 360.0); return d < 0 ? d + 360.0 : d; }
// the island's roads, by kind: a freeway's grade and curvature; a mountain road's switchbacks
TerrainRouteParams freewayRouteParams() {
    TerrainRouteParams fw;
    fw.cell = 30.0; fw.maxGrade = 0.05; fw.hardGrade = 0.30; fw.gradeWeight = 800.0; fw.turnWeight = 30.0; fw.maxTurnDeg = 25.0; fw.margin = 1400.0;
    return fw;
}
// A routed road FINISHED for its kind (terrain_route.h): tightened -- no spike, no detour a straight
// line could drive -- then its corners rounded to what its traffic takes, and measured again.
void finishRoad(IslandRoad& rd, const HeightField& ground, const std::function<bool(double, double)>& blocked) {
    if (rd.points.size() < 3) return;
    TightenParams tp;
    double radius = 15.0;
    if (rd.kind == "freeway") { tp.maxGrade = 0.05; tp.maxCutFill = 12.0; tp.maxStraight = 1500.0; radius = 150.0; }
    else if (rd.kind == "pass") { tp.maxGrade = 0.07; tp.maxCutFill = 8.0; tp.maxStraight = 600.0; radius = 30.0; }
    else { tp.maxGrade = 0.08; tp.maxCutFill = 6.0; tp.maxStraight = 400.0; radius = 15.0; }
    tp.blocked = blocked;
    const TerrainRoute m = measureRoute(ground, roundRoute(tightenRoute(ground, rd.points, tp), radius));
    rd.points = m.points;
    rd.length = m.length;
    rd.climb = m.climb;
    rd.worstGrade = m.worstGrade;
}
TerrainRouteParams mountainRouteParams() {
    TerrainRouteParams mr;
    mr.cell = 12.0; mr.maxGrade = 0.08; mr.hardGrade = 0.12; mr.margin = 1500.0;   // switchbacks, not 45% ramps
    return mr;
}
}  // namespace

double IslandWorld::heightAt(double x, double z) const {
    const double fx = std::clamp((x + half) / cell, 0.0, n - 1.001), fz = std::clamp((z + half) / cell, 0.0, n - 1.001);
    const int i = static_cast<int>(fx), j = static_cast<int>(fz);
    const double u = fx - i, v = fz - j;
    auto H = [&](int a, int b) { return static_cast<double>(height[static_cast<std::size_t>(b) * n + a]); };
    return (H(i, j) * (1 - u) + H(i + 1, j) * u) * (1 - v) + (H(i, j + 1) * (1 - u) + H(i + 1, j + 1) * u) * v;
}

double IslandWorld::smoothAt(double x, double z) const {
    if (heightSmooth.size() != height.size()) return heightAt(x, z);
    const double fx = std::clamp((x + half) / cell, 0.0, n - 1.001), fz = std::clamp((z + half) / cell, 0.0, n - 1.001);
    const int i = static_cast<int>(fx), j = static_cast<int>(fz);
    const double u = fx - i, v = fz - j;
    auto H = [&](int a, int b) { return static_cast<double>(heightSmooth[static_cast<std::size_t>(b) * n + a]); };
    return (H(i, j) * (1 - u) + H(i + 1, j) * u) * (1 - v) + (H(i, j + 1) * (1 - u) + H(i + 1, j + 1) * u) * v;
}

json islandTerrainBlock(uint32_t seed, double half) {
    auto r = [&](int k) { return rnd(seed, k); };
    const double angle = r(1) * 180.0, aspect = 1.35 + 0.3 * r(2);
    const double radius = std::min(5500.0 + 700.0 * r(3), half * 0.9 / (aspect * 1.15));
    const double a = angle * kPi / 180.0;
    // the peninsula: off one end of the island (sometimes) or off a side, a little askew
    const bool offEnd = r(4) < 0.5;
    const double penBearing = wrap360(angle + (offEnd ? (r(5) < 0.5 ? 0.0 : 180.0) : (r(5) < 0.5 ? 90.0 : -90.0)) + (r(6) - 0.5) * 50.0);
    // the cliff coast: a stretch well away from the peninsula
    const double cliffMid = wrap360(penBearing + 110.0 + r(7) * 140.0), cliffSpan = 40.0 + 30.0 * r(8);
    // the range: along the long axis, gently bent, reaching most of the way to either end
    const double L = radius * aspect * 1.25;
    json spine = json::array();
    for (int k = 0; k < 5; ++k) {
        const double t = -0.5 + k / 4.0;
        const double bend = (r(20 + k) - 0.5) * radius * 0.35;
        const Vec2 p = fromAxis(t * L, bend, a);
        spine.push_back({std::round(p.x), std::round(p.y)});
    }
    json block = {
        {"seed", 100 + seed % 1000},
        {"size", 2.0 * half}, {"resolution", 1000},
        {"heightScale", 6}, {"noiseScale", 0.0006}, {"octaves", 5}, {"warp", 0.35},
        {"mountainHeight", 240 + 120 * r(9)}, {"mountainScale", 0.0009}, {"mountainAlongRange", true}, {"mountainMaskScale", 0},
        {"rangeSpine", spine}, {"rangeWidth", radius * 0.55}, {"rangeHeight", 760 + 260 * r(10)}, {"rangeVariation", 0.65},
        {"snowLine", 1300}, {"rockLine", 520},
        {"seaLevel", 0.0},
        {"island", {{"radius", std::round(radius)}, {"aspect", aspect}, {"angle", angle},
                    {"coastNoise", 0.2 + 0.08 * r(11)}, {"coastScale", 0.0003 + 0.0001 * r(12)},
                    {"peninsula", {{"bearing", penBearing}, {"length", 3000 + 1500 * r(13)}, {"width", 800 + 400 * r(14)}}},
                    {"cliffs", {{"from", wrap360(cliffMid - cliffSpan * 0.5)}, {"to", wrap360(cliffMid + cliffSpan * 0.5)}, {"height", 70 + 60 * r(15)}}},
                    {"plainHeight", 12}, {"shelfDepth", 45}}},
        // EROSION: Hawaii's look is its valleys -- deep, branching, down from the ridge -- and a road
        // over the range wants them (it climbs a valley to the saddle, not across every ridge)
        {"erode", true}, {"erodeLandOnly", true}, {"erodeRes", 1024}, {"erodeDroplets", 1500000}, {"erodeRadius", 3}, {"erodeThermal", 16},
        {"rivers", {{"region", 2.0 * half}, {"cell", 20}, {"riverArea", 2.0e6}, {"widthMin", 8}, {"widthMax", 70}, {"widthK", 0.02},
                    {"lakeMinArea", 80000}, {"bankSteep", 2.5}, {"incisionMin", 1.5}}},
    };
    return block;
}

IslandWorld planIsland(const json& blockIn, double half, double cell) {
    const auto t0 = std::chrono::steady_clock::now();
    IslandWorld w;
    w.terrain = blockIn;
    w.half = half;
    w.cell = cell;
    json block = blockIn;
    if (!block.contains("seaLevel")) block["seaLevel"] = 0.0;
    TerrainParams tp = readTerrainParams(block);
    if (block.value("erode", false)) tp.erodedBase = erodedForTerrain(block);   // as the level loader does
    const Noise nz(block.value("seed", 0u));
    w.hydro = tp.hydro;
    const double axisA = tp.island.angleDeg * kPi / 180.0;
    w.axis = Vec2(std::cos(axisA), std::sin(axisA));
    const Vec2 C(tp.island.cx, tp.island.cz);

    // 1. THE GROUND, every `cell` metres (rivers carved), across threads
    const int n = static_cast<int>(std::lround(2.0 * half / cell)) + 1;
    w.n = n;
    w.height.assign(static_cast<std::size_t>(n) * n, 0.0f);
    {
        const unsigned T = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::thread> pool;
        for (unsigned t = 0; t < T; ++t)
            pool.emplace_back([&, t] {
                for (int j = static_cast<int>(t); j < n; j += static_cast<int>(T))
                    for (int i = 0; i < n; ++i)
                        w.height[static_cast<std::size_t>(j) * n + i] =
                            static_cast<float>(terrainHeight(tp, nz, -half + i * cell, -half + j * cell));
            });
        for (auto& th : pool) th.join();
    }
    auto H = [&](int i, int j) { return static_cast<double>(w.height[static_cast<std::size_t>(std::clamp(j, 0, n - 1)) * n + std::clamp(i, 0, n - 1)]); };
    auto P = [&](int i, int j) { return Vec2(-half + i * cell, -half + j * cell); };

    // 2. BUILDABLE: above the beach, gentler than 9 %, and dry
    w.buildable.assign(static_cast<std::size_t>(n) * n, 0);
    double landArea = 0.0, peak = -1e9;
    Vec2 peakAt;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const double h = H(i, j);
            if (h > 0.0) landArea += cell * cell;
            if (h > peak) { peak = h; peakAt = P(i, j); }
            if (h < 1.5) continue;
            const double gx = (H(i + 1, j) - H(i - 1, j)) / (2 * cell), gz = (H(i, j + 1) - H(i, j - 1)) / (2 * cell);
            if (std::sqrt(gx * gx + gz * gz) > 0.09) continue;
            const Vec2 q = P(i, j);
            if (w.hydro && w.hydro->isWet(q.x, q.y, 12.0)) continue;
            w.buildable[static_cast<std::size_t>(j) * n + i] = 1;
        }
    // how deep into flat ground each cell is (m): a two-pass chamfer distance to the nearest unbuildable cell
    std::vector<float> dt(static_cast<std::size_t>(n) * n, 0.0f);
    {
        const float big = 1e9f, a1 = static_cast<float>(cell), a2 = static_cast<float>(cell * 1.41421356);
        for (std::size_t k = 0; k < dt.size(); ++k) dt[k] = w.buildable[k] ? big : 0.0f;
        auto at = [&](int i, int j) -> float& { return dt[static_cast<std::size_t>(j) * n + i]; };
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                float& d = at(i, j);
                if (d == 0.0f) continue;
                if (i > 0) d = std::min(d, at(i - 1, j) + a1);
                if (j > 0) d = std::min(d, at(i, j - 1) + a1);
                if (i > 0 && j > 0) d = std::min(d, at(i - 1, j - 1) + a2);
                if (i + 1 < n && j > 0) d = std::min(d, at(i + 1, j - 1) + a2);
            }
        for (int j = n - 1; j >= 0; --j)
            for (int i = n - 1; i >= 0; --i) {
                float& d = at(i, j);
                if (d == 0.0f) continue;
                if (i + 1 < n) d = std::min(d, at(i + 1, j) + a1);
                if (j + 1 < n) d = std::min(d, at(i, j + 1) + a1);
                if (i + 1 < n && j + 1 < n) d = std::min(d, at(i + 1, j + 1) + a2);
                if (i > 0 && j + 1 < n) d = std::min(d, at(i - 1, j + 1) + a2);
            }
    }
    auto D = [&](int i, int j) { return static_cast<double>(dt[static_cast<std::size_t>(j) * n + i]); };
    auto nearSea = [&](const Vec2& q, double within) {
        for (int k = 0; k < 16; ++k) {
            const double ang = 2 * kPi * k / 16;
            for (double rr = 200; rr <= within; rr += 200)
                if (w.heightAt(q.x + std::cos(ang) * rr, q.y + std::sin(ang) * rr) < 0.0) return true;
        }
        return false;
    };
    auto side = [&](const Vec2& q) { const Vec2 d = q - C; return w.axis.x * d.y - w.axis.y * d.x > 0 ? 1 : -1; };

    // 3. SITES. A city on each side of the range: the deepest flat ground, preferring the lowlands.
    for (int s : {1, -1}) {
        double best = -1.0;
        int bi = -1, bj = -1;
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const double d = D(i, j);
                if (d < 150.0 || side(P(i, j)) != s) continue;
                const double score = d * (H(i, j) < 120.0 ? 1.0 : 0.5);
                if (score > best) { best = score; bi = i; bj = j; }
            }
        if (bi < 0) continue;
        IslandSite c;
        c.kind = "city";
        c.at = P(bi, bj);
        c.radius = std::clamp(D(bi, bj) * 1.6, 600.0, 2000.0);
        c.elevation = H(bi, bj);
        c.coastal = nearSea(c.at, 2500.0);
        w.sites.push_back(c);
    }
    // towns: other deep flat ground, spread out; then the highest good ground, the mountain town
    auto clearOf = [&](const Vec2& q, double gap) {
        for (const IslandSite& s : w.sites)
            if ((s.at - q).length() < gap + s.radius) return false;
        return true;
    };
    for (int round = 0; round < 6; ++round) {
        double best = -1.0;
        int bi = -1, bj = -1;
        for (int j = 0; j < n; j += 2)
            for (int i = 0; i < n; i += 2) {
                const double d = D(i, j);
                if (d < 140.0 || H(i, j) > 250.0 || !clearOf(P(i, j), 2200.0)) continue;
                if (d > best) { best = d; bi = i; bj = j; }
            }
        if (bi < 0) break;
        IslandSite t;
        t.kind = "town";
        t.at = P(bi, bj);
        t.radius = std::clamp(D(bi, bj) * 1.3, 250.0, 700.0);
        t.elevation = H(bi, bj);
        t.coastal = nearSea(t.at, 1600.0);
        w.sites.push_back(t);
    }
    {
        double best = -1.0;
        int bi = -1, bj = -1;
        for (int j = 0; j < n; j += 2)
            for (int i = 0; i < n; i += 2) {
                const double d = D(i, j), h = H(i, j);
                if (d < 70.0 || h < 120.0 || !clearOf(P(i, j), 1500.0)) continue;
                const double score = h + 0.5 * d;
                if (score > best) { best = score; bi = i; bj = j; }
            }
        if (bi >= 0) {
            IslandSite m;
            m.kind = "mountain town";
            m.at = P(bi, bj);
            m.radius = std::clamp(D(bi, bj) * 1.4, 150.0, 450.0);
            m.elevation = H(bi, bj);
            w.sites.push_back(m);
        }
    }
    // each site's flat area: its patch of buildable ground (flood fill from it, capped)
    for (IslandSite& s : w.sites) {
        const int si = std::clamp(static_cast<int>(std::lround((s.at.x + half) / cell)), 0, n - 1);
        const int sj = std::clamp(static_cast<int>(std::lround((s.at.y + half) / cell)), 0, n - 1);
        std::vector<char> seen(static_cast<std::size_t>(n) * n, 0);
        std::vector<int> stack{sj * n + si};
        seen[static_cast<std::size_t>(sj) * n + si] = 1;
        long cells = 0;
        while (!stack.empty() && cells < 400000) {
            const int c = stack.back();
            stack.pop_back();
            ++cells;
            const int i = c % n, j = c / n;
            const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                const int a = i + di[k], b = j + dj[k];
                if (a < 0 || b < 0 || a >= n || b >= n) continue;
                const std::size_t nb = static_cast<std::size_t>(b) * n + a;
                if (seen[nb] || !w.buildable[nb]) continue;
                seen[nb] = 1;
                stack.push_back(static_cast<int>(nb));
            }
        }
        s.flatArea = cells * cell * cell;
    }

    // the mountain roads' ground: a separable box blur, 3 cells (60 m) each way
    {
        const int N = w.n, r = 3;
        std::vector<float> t(w.height.size());
        w.heightSmooth.assign(w.height.size(), 0.0f);
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                double sum = 0; int c = 0;
                for (int k = std::max(0, i - r); k <= std::min(N - 1, i + r); ++k) { sum += w.height[static_cast<std::size_t>(j) * N + k]; ++c; }
                t[static_cast<std::size_t>(j) * N + i] = static_cast<float>(sum / c);
            }
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                double sum = 0; int c = 0;
                for (int k = std::max(0, j - r); k <= std::min(N - 1, j + r); ++k) { sum += t[static_cast<std::size_t>(k) * N + i]; ++c; }
                w.heightSmooth[static_cast<std::size_t>(j) * N + i] = static_cast<float>(sum / c);
            }
    }
    // 4. ROADS over the ground (terrain_route.h)
    const HeightField ground = [&w](double x, double z) { return w.heightAt(x, z); };
    const HeightField smoothGround = [&w](double x, double z) { return w.smoothAt(x, z); };
    auto route = [&](const std::string& kind, int a, int b, const Vec2& from, const Vec2& to, TerrainRouteParams rp) {
        rp.blocked = [&w](double x, double z) { return w.heightAt(x, z) < 0.8; };   // never over the sea
        const HeightField& g = (kind == "pass" || kind == "mountain") ? smoothGround : ground;
        const TerrainRoute tr = routeOnTerrain(g, from, to, rp);
        IslandRoad rd;
        rd.kind = kind;
        rd.from = a;
        rd.to = b;
        rd.points = tr.points;
        rd.length = tr.length;
        rd.climb = tr.climb;
        rd.worstGrade = tr.worstGrade;
        if (kind != "pass") finishRoad(rd, g, rp.blocked);   // a pass is finished once its legs are one road
        w.roads.push_back(rd);
        return !tr.points.empty();
    };
    // the ring: every site but the mountain town, in order round the island
    std::vector<int> ring;
    for (int k = 0; k < static_cast<int>(w.sites.size()); ++k)
        if (w.sites[static_cast<std::size_t>(k)].kind != "mountain town") ring.push_back(k);
    std::sort(ring.begin(), ring.end(), [&](int a, int b) {
        const Vec2 da = w.sites[static_cast<std::size_t>(a)].at - C, db = w.sites[static_cast<std::size_t>(b)].at - C;
        return std::atan2(da.y, da.x) < std::atan2(db.y, db.x);
    });
    const TerrainRouteParams fw = freewayRouteParams();
    for (std::size_t k = 0; k < ring.size() && ring.size() >= 2; ++k) {
        const int a = ring[k], b = ring[(k + 1) % ring.size()];
        if (ring.size() == 2 && k == 1) break;
        route("freeway", a, b, w.sites[static_cast<std::size_t>(a)].at, w.sites[static_cast<std::size_t>(b)].at, fw);
    }
    // the pass: the two cities, over the range
    json saddleJson;
    {
        int c0 = -1, c1 = -1;
        for (int k = 0; k < static_cast<int>(w.sites.size()); ++k)
            if (w.sites[static_cast<std::size_t>(k)].kind == "city") (c0 < 0 ? c0 : c1) = k;
        if (c0 >= 0 && c1 >= 0) {
            // THE SADDLE: the lowest point along the middle of the range's spine -- the pass goes over it
            Vec2 saddle = (w.sites[static_cast<std::size_t>(c0)].at + w.sites[static_cast<std::size_t>(c1)].at) * 0.5;
            if (!tp.rangeSpine.empty()) {
                const std::size_t m = tp.rangeSpine.size();
                double low = 1e30;
                for (std::size_t k = m / 5; k < m - m / 5; ++k) {
                    const Vec2 q(tp.rangeSpine[k].x, tp.rangeSpine[k].z);
                    // the spine's lowest ground within 300 m across it (the saddle sits off the axis a little)
                    const Vec3 d3 = tp.rangeSpine[std::min(k + 1, m - 1)] - tp.rangeSpine[k > 0 ? k - 1 : 0];
                    Vec2 across(-d3.z, d3.x);
                    across = across.length() > 1e-9 ? across / across.length() : Vec2(1, 0);
                    for (double o = -300; o <= 300; o += 50) {
                        const Vec2 p2 = q + across * o;
                        const double h = w.heightAt(p2.x, p2.y);
                        if (h < low) { low = h; saddle = p2; }
                    }
                }
            }
            TerrainRouteParams ps;
            // 12% at the very steepest, on a 12 m grid so switchbacks fit: the 60% the pass was first allowed
            // (a 24 m grid could find nothing gentler) built as 2 km of bridge 110 m up (ADR-0112)
            ps.cell = 12.0; ps.maxGrade = 0.07; ps.hardGrade = 0.15; ps.gradeWeight = 800.0; ps.turnWeight = 12.0; ps.margin = 4000.0;
            const std::size_t before = w.roads.size();
            route("pass", c0, c1, w.sites[static_cast<std::size_t>(c0)].at, saddle, ps);
            route("pass", c0, c1, saddle, w.sites[static_cast<std::size_t>(c1)].at, ps);
            // ONE road over the saddle, not two meeting at it: forced through the saddle point, the
            // legs met in a spike (up a spur and back); tightened as one, the spike is gone
            if (w.roads.size() == before + 2 && !w.roads[before].points.empty() && !w.roads[before + 1].points.empty()) {
                IslandRoad& one = w.roads[before];
                one.points.insert(one.points.end(), w.roads[before + 1].points.begin() + 1, w.roads[before + 1].points.end());
                w.roads.pop_back();
                finishRoad(one, smoothGround, [&w](double x, double z) { return w.heightAt(x, z) < 0.8; });
            }
            saddleJson = {{"at", {std::round(saddle.x), std::round(saddle.y)}}, {"height", std::round(w.heightAt(saddle.x, saddle.y))}};
            (void)before;
        }
    }
    // the mountain road: from the nearest point on the freeway up to the mountain town
    for (int k = 0; k < static_cast<int>(w.sites.size()); ++k) {
        if (w.sites[static_cast<std::size_t>(k)].kind != "mountain town") continue;
        const Vec2 top = w.sites[static_cast<std::size_t>(k)].at;
        Vec2 foot = w.sites.empty() ? top : w.sites.front().at;
        double best = 1e30;
        for (const IslandRoad& rd : w.roads)
            for (const Vec2& q : rd.points)
                if (rd.kind == "freeway" && (q - top).length() < best) { best = (q - top).length(); foot = q; }
        route("mountain", -1, k, foot, top, mountainRouteParams());
    }

    // NAMES (place_names.h): the site shapes the name -- the coast, a river through it, the mountain
    {
        std::set<std::string> used;
        const uint32_t worldSeed = static_cast<uint32_t>(w.terrain.value("seed", 0u));
        for (std::size_t k = 0; k < w.sites.size(); ++k) {
            IslandSite& s = w.sites[k];
            PlaceTraits t;
            t.city = s.kind == "city";
            t.coastal = s.coastal;
            t.mountain = s.kind == "mountain town";
            t.river = w.hydro && w.hydro->distanceToRiver(s.at.x, s.at.y, 2000.0) < std::max(200.0, s.radius * 0.8);
            s.name = placeName(placeNameBook(), t, worldSeed, static_cast<uint32_t>(k), used);
        }
    }
    // 5. THE REPORT
    json sites = json::array(), roads = json::array();
    for (const IslandSite& s : w.sites)
        sites.push_back({{"name", s.name}, {"kind", s.kind}, {"at", {std::round(s.at.x), std::round(s.at.y)}}, {"radius", std::round(s.radius)},
                         {"elevation", std::round(s.elevation)}, {"flatKm2", std::round(s.flatArea / 1e4) / 100.0}, {"coastal", s.coastal}});
    double fwLen = 0.0;
    for (const IslandRoad& rd : w.roads) {
        if (rd.kind == "freeway") fwLen += rd.length;
        roads.push_back({{"kind", rd.kind}, {"from", rd.from}, {"to", rd.to}, {"lengthKm", std::round(rd.length / 10.0) / 100.0},
                         {"climb", std::round(rd.climb)}, {"worstGradePct", std::round(rd.worstGrade * 1000.0) / 10.0},
                         {"routed", !rd.points.empty()}});
    }
    double riverKm = 0.0;
    if (w.hydro)
        for (const River& rv : w.hydro->rivers())
            for (std::size_t k = 0; k + 1 < rv.nodes.size(); ++k) riverKm += (rv.nodes[k + 1].p - rv.nodes[k].p).length() / 1000.0;
    w.report = {{"landKm2", std::round(landArea / 1e4) / 100.0},
                {"peak", {{"height", std::round(peak)}, {"at", {std::round(peakAt.x), std::round(peakAt.y)}}}},
                {"rivers", w.hydro ? w.hydro->rivers().size() : 0}, {"riverKm", std::round(riverKm * 10.0) / 10.0},
                {"lakes", w.hydro ? w.hydro->lakes().size() : 0},
                {"sites", sites}, {"roads", roads}, {"ringFreewayKm", std::round(fwLen / 100.0) / 10.0}, {"saddle", saddleJson},
                {"seconds", std::round(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 10.0) / 10.0}};
    return w;
}

json islandSiteBrief(const IslandWorld& w, int site) {
    const IslandSite& s = w.sites.at(static_cast<std::size_t>(site));
    const bool city = s.kind == "city";
    // TOWARD THE SEA: the nearest sea sample; a site inland squares to the range's axis instead
    Vec2 toSea = w.axis;
    double best = 1e30;
    for (int j = 0; j < w.n; j += 2)
        for (int i = 0; i < w.n; i += 2) {
            if (w.height[static_cast<std::size_t>(j) * w.n + i] >= 0.0f) continue;
            const Vec2 q(-w.half + i * w.cell, -w.half + j * w.cell);
            const double d = (q - s.at).length();
            if (d < best) { best = d; toSea = q - s.at; }
        }
    if (best > 3.0 * std::max(600.0, s.radius)) toSea = w.axis;
    const double angle = std::atan2(toSea.y, toSea.x) * 180.0 / kPi;
    // what it grows to (land_shape.h): a city its flat ground's disc and a little more, a town its own;
    // the plan's square is room for that to run out along a valley or a strip of coast
    const double area = kPi * std::pow(s.radius * (city ? 1.15 : 1.0), 2.0);
    const double size = 2.0 * std::max(city ? 1400.0 : 500.0, s.radius * 2.4);
    const double core = std::sqrt(area * 0.12 / kPi), mid = std::sqrt(area * 0.35 / kPi);
    const uint32_t seed = static_cast<uint32_t>(w.terrain.value("seed", 0u) * 7919u + static_cast<uint32_t>(site) * 104729u + 17u);
    const std::string name = (city ? "city_" : s.kind == "town" ? "town_" : "mountain_town_") + std::to_string(site);
    json world = {
        {"_comment", "An island site (island_world.h islandSiteBrief): the island's terrain block; the city grown over its land to its area and laid out by depth (land_shape.h)."},
        {"base", w.terrain},
        {"seaLevel", w.terrain.value("seaLevel", 0.0)},
        {"grid", size * 0.5 + 60.0},
        // a street may be as steep as a hill town's (15%); the growth prefers the flat ground anyway
        {"land", {{"minHeight", 3.0}, {"maxSlope", 0.15}, {"shape", true}, {"area", area}, {"spokeSpacing", city ? 480.0 : 360.0}}},
    };
    return {
        {"name", name},
        {"seed", seed},
        {"size", size},
        {"center", {s.at.x, s.at.y}},
        {"relief", 3.0},   // the island is the relief; the brief's hills would only roughen it
        {"core", {{"angle", angle}, {"arterialEvery", 3}, {"block", {94.0, 64.0}}, {"radius", core}}},
        {"midtown", {{"block", {158.0, 108.0}}, {"radius", mid}, {"warp", city ? 18.0 : 10.0}}},
        {"outskirts", {{"curvature", 24.0}, {"margin", 120.0}, {"ringSpacing", city ? 140.0 : 120.0}, {"spokes", city ? 10 : 6}, {"streetSpacing", 95.0}}},
        {"freeway", {{"radius", 0.0}, {"radials", 0}, {"wobble", 0.0}}},   // no ring: the island's freeway serves it
        {"roads", {{"arterial", 22.0}, {"collector", 16.0}, {"freeway", 30.0}, {"local", 12.0}, {"sidewalk", 5.0}}},
        {"world", world},
    };
}

void joinFreewayToRing(IslandWorld& w, int site, const std::vector<Vec2>& ring, bool closed) {
    if (ring.size() < 2) return;
    // the ring's nearest point to q, along its segments
    auto nearest = [&](const Vec2& q) {
        Vec2 best = ring.front();
        double bd = 1e30;
        const std::size_t n = ring.size(), segs = closed ? n : n - 1;
        for (std::size_t i = 0; i < segs; ++i) {
            const Vec2 a = ring[i], ab = ring[(i + 1) % n] - a;
            const double L2 = ab.x * ab.x + ab.y * ab.y;
            const double t = L2 > 1e-12 ? std::clamp(((q - a).x * ab.x + (q - a).y * ab.y) / L2, 0.0, 1.0) : 0.0;
            const Vec2 f = a + ab * t;
            if ((q - f).length() < bd) { bd = (q - f).length(); best = f; }
        }
        return best;
    };
    for (IslandRoad& rd : w.roads) {
        if (rd.points.size() < 2 || (rd.from != site && rd.to != site)) continue;
        const bool atStart = rd.from == site;
        std::vector<Vec2> pts = rd.points;
        if (atStart) std::reverse(pts.begin(), pts.end());   // now it runs toward the site
        // walk in until it comes within 60 m of the ring (its corridor), densified so it stops there
        std::vector<Vec2> kept{pts.front()};
        bool met = false;
        for (std::size_t i = 0; i + 1 < pts.size() && !met; ++i) {
            const int m = std::max(1, static_cast<int>(std::ceil((pts[i + 1] - pts[i]).length() / 20.0)));
            for (int k = 1; k <= m; ++k) {
                const Vec2 q = pts[i] + (pts[i + 1] - pts[i]) * (static_cast<double>(k) / m);
                if ((q - nearest(q)).length() < 60.0) { met = true; break; }
                kept.push_back(q);
            }
        }
        if (!met || kept.size() < 2) continue;
        kept.push_back(nearest(kept.back()));
        if (atStart) std::reverse(kept.begin(), kept.end());
        rd.points = kept;
    }
}

void routeFreewayRoundCities(IslandWorld& w, const std::vector<std::pair<int, std::vector<std::vector<Vec2>>>>& cityLimits) {
    const int n = w.n;
    const std::size_t N = static_cast<std::size_t>(n) * n;
    auto cellOf = [&](const Vec2& p, int& i, int& j) {
        i = static_cast<int>(std::lround((p.x + w.half) / w.cell));
        j = static_cast<int>(std::lround((p.y + w.half) / w.cell));
        return i >= 0 && j >= 0 && i < n && j < n;
    };
    // each city's inside, rasterized (even-odd, row by row), and the no-go mask: every inside grown 40 m
    std::vector<int> owner(N, -1);
    for (const auto& [site, loops] : cityLimits) w.sites[static_cast<std::size_t>(site)].limits = loops;
    for (const auto& [site, loops] : cityLimits)
        for (int j = 0; j < n; ++j) {
            const double z = -w.half + j * w.cell;
            std::vector<double> xs;
            for (const std::vector<Vec2>& L : loops)
                for (std::size_t k = 0; k < L.size(); ++k) {
                    const Vec2 a = L[k], b = L[(k + 1) % L.size()];
                    if ((a.y > z) != (b.y > z)) xs.push_back(a.x + (z - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            std::sort(xs.begin(), xs.end());
            for (std::size_t k = 0; k + 1 < xs.size(); k += 2)
                for (int i = std::max(0, static_cast<int>(std::ceil((xs[k] + w.half) / w.cell)));
                     i < n && -w.half + i * w.cell <= xs[k + 1]; ++i)
                    owner[static_cast<std::size_t>(j) * n + i] = site;
        }
    std::vector<char> nogo(N, 0);
    const int grow = static_cast<int>(std::ceil(40.0 / w.cell));
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            if (owner[static_cast<std::size_t>(j) * n + i] < 0) continue;
            for (int dj = -grow; dj <= grow; ++dj)
                for (int di = -grow; di <= grow; ++di) {
                    const int u = i + di, v = j + dj;
                    if (u >= 0 && v >= 0 && u < n && v < n) nogo[static_cast<std::size_t>(v) * n + u] = 1;
                }
        }
    auto blocked = [&](double x, double z) {
        int i, j;
        if (!cellOf(Vec2(x, z), i, j)) return true;
        return nogo[static_cast<std::size_t>(j) * n + i] || w.heightAt(x, z) < 0.8;
    };
    // each city's WAYPOINT: the point of its limits nearest the range's axis, stepped out toward it
    const json& il = w.terrain.value("island", json::object());
    const Vec2 C(il.value("cx", 0.0), il.value("cz", 0.0));
    const Vec2 ax = w.axis;
    std::vector<Vec2> via(w.sites.size());
    for (std::size_t k = 0; k < w.sites.size(); ++k) via[k] = w.sites[k].at;
    for (const auto& [site, loops] : cityLimits) {
        double best = 1e30;
        Vec2 pick = w.sites[static_cast<std::size_t>(site)].at;
        for (const std::vector<Vec2>& L : loops)
            for (const Vec2& q : L) {
                const Vec2 d = q - C;
                const double off = std::fabs(d.x * ax.y - d.y * ax.x);   // distance from the axis line
                if (off < best) { best = off; pick = q; }
            }
        const Vec2 d = pick - C;
        const Vec2 foot = C + ax * (d.x * ax.x + d.y * ax.y);
        Vec2 in = foot - pick;
        in = in.length() > 1e-9 ? in * (1.0 / in.length()) : Vec2(0, 0);
        Vec2 q = pick;
        for (double t = 80.0; t <= 600.0 && blocked(q.x, q.y); t += 40.0) q = pick + in * t;
        via[static_cast<std::size_t>(site)] = q;
    }
    const HeightField ground = [&w](double x, double z) { return w.heightAt(x, z); };
    auto reroute = [&](IslandRoad& rd, const Vec2& from, const Vec2& to, TerrainRouteParams rp) {
        rp.blocked = blocked;
        const TerrainRoute tr = routeOnTerrain(ground, from, to, rp);
        if (tr.points.empty()) return false;
        rd.points = tr.points;
        finishRoad(rd, ground, blocked);
        return true;
    };
    double fwLen = 0.0;
    int unrouted = 0;
    for (IslandRoad& rd : w.roads) {
        if (rd.kind != "freeway" || rd.from < 0 || rd.to < 0) continue;
        if (!reroute(rd, via[static_cast<std::size_t>(rd.from)], via[static_cast<std::size_t>(rd.to)], freewayRouteParams())) ++unrouted;
        fwLen += rd.length;
    }
    // the other roads stop at the limits they reach (their own end's city)
    auto trimAt = [&](IslandRoad& rd, int site, bool atEnd) {
        // walked from the road's other end toward this one; it ends where it first enters the city
        std::vector<Vec2>& P = rd.points;
        if (!atEnd) std::reverse(P.begin(), P.end());
        std::size_t keep = 0;
        while (keep < P.size()) {
            int i, j;
            if (cellOf(P[keep], i, j) && owner[static_cast<std::size_t>(j) * n + i] == site) break;
            ++keep;
        }
        if (keep >= 2 && keep < P.size()) P.resize(keep);
        if (!atEnd) std::reverse(P.begin(), P.end());
    };
    for (IslandRoad& rd : w.roads) {
        if (rd.kind == "freeway" || rd.points.size() < 2) continue;
        if (rd.kind == "mountain" && rd.to >= 0) {
            // from the freeway where it now runs
            const Vec2 top = w.sites[static_cast<std::size_t>(rd.to)].at;
            Vec2 foot = rd.points.front();
            double best = 1e30;
            for (const IslandRoad& f : w.roads)
                if (f.kind == "freeway") for (const Vec2& q : f.points) if ((q - top).length() < best) { best = (q - top).length(); foot = q; }
            TerrainRouteParams mr = mountainRouteParams();
            const HeightField smooth = [&w](double x, double z) { return w.smoothAt(x, z); };
            const TerrainRoute tr = routeOnTerrain(smooth, foot, top, [&] { mr.blocked = [&w](double x, double z) { return w.heightAt(x, z) < 0.8; }; return mr; }());
            if (!tr.points.empty()) { rd.points = tr.points; finishRoad(rd, smooth, blocked); }
        }
        // a PASS ends ON the freeway at each side (an interchange there), not at a city's edge beside it
        if (rd.kind == "pass") {
            std::vector<Vec2>& P = rd.points;
            for (int side = 0; side < 2; ++side) {
                // walked from the saddle (the middle) out to this end
                const std::size_t mid = P.size() / 2;
                std::size_t cut = side ? P.size() : 0;
                Vec2 onto;
                for (std::size_t k = mid; side ? k < P.size() : k + 1 > 0; side ? ++k : --k) {
                    double best = 1e30;
                    for (const IslandRoad& f : w.roads)
                        if (f.kind == "freeway")
                            for (std::size_t q = 0; q + 1 < f.points.size(); ++q) {
                                const Vec2 a = f.points[q], ab = f.points[q + 1] - a;
                                const double L2 = ab.x * ab.x + ab.y * ab.y;
                                const double t = L2 > 1e-12 ? std::clamp(((P[k] - a).x * ab.x + (P[k] - a).y * ab.y) / L2, 0.0, 1.0) : 0.0;
                                const Vec2 fp = a + ab * t;
                                if ((P[k] - fp).length() < best) { best = (P[k] - fp).length(); onto = fp; }
                            }
                    if (best < 60.0) { cut = k; break; }
                    if (!side && k == 0) break;
                }
                if (side ? cut < P.size() : cut > 0) {
                    if (side) { P.resize(cut + 1); P.back() = onto; }
                    else { P.erase(P.begin(), P.begin() + static_cast<std::ptrdiff_t>(cut)); P.front() = onto; }
                    if (side) rd.to = -1; else rd.from = -1;   // it ends on the freeway now
                }
            }
            const TerrainRoute m = measureRoute([&w](double x, double z) { return w.smoothAt(x, z); }, P);
            rd.length = m.length; rd.climb = m.climb; rd.worstGrade = m.worstGrade;
        }
        if (rd.from >= 0) trimAt(rd, rd.from, false);
        if (rd.to >= 0) trimAt(rd, rd.to, true);
    }
    w.report["ringFreewayKm"] = std::round(fwLen / 100.0) / 10.0;
    w.report["freewayRoundCities"] = {{"cities", cityLimits.size()}, {"unrouted", unrouted}};
}

void linkCityToFreeway(IslandWorld& w, int site, const std::vector<Vec2>& arterialNodes, int maxLinks, double spacing, double maxLength,
                       const std::vector<std::string>& streets) {
    // every candidate's nearest freeway point
    struct C { Vec2 at, onto; double d; std::string street; };
    std::vector<C> cs;
    for (std::size_t i = 0; i < arterialNodes.size(); ++i) {
        const Vec2 p = arterialNodes[i];
        C c{p, p, 1e30, i < streets.size() ? streets[i] : std::string()};
        for (const IslandRoad& f : w.roads)
            if (f.kind == "freeway")
                for (std::size_t q = 0; q + 1 < f.points.size(); ++q) {
                    const Vec2 a = f.points[q], ab = f.points[q + 1] - a;
                    const double L2 = ab.x * ab.x + ab.y * ab.y;
                    const double t = L2 > 1e-12 ? std::clamp(((p - a).x * ab.x + (p - a).y * ab.y) / L2, 0.0, 1.0) : 0.0;
                    const Vec2 fp = a + ab * t;
                    if ((p - fp).length() < c.d) { c.d = (p - fp).length(); c.onto = fp; }
                }
        if (c.d <= maxLength) cs.push_back(c);
    }
    std::sort(cs.begin(), cs.end(), [](const C& a, const C& b) { return a.d < b.d; });
    std::vector<Vec2> chosen;
    for (const C& c : cs) {
        if (static_cast<int>(chosen.size()) >= maxLinks) break;
        bool near = false;
        for (const Vec2& q : chosen) if ((q - c.onto).length() < spacing) near = true;
        if (near) continue;
        chosen.push_back(c.onto);
        IslandRoad rd;
        rd.kind = "link";
        rd.from = site;
        rd.points = {c.at, c.onto};
        rd.length = c.d;
        rd.street = c.street;
        w.roads.push_back(rd);
    }
}

void islandInterchanges(IslandWorld& w, const std::vector<std::pair<Vec2, Vec2>>& cityStreets) {
    // the freeway's nearest point to p (and which leg, and the direction along it there)
    auto nearestFreeway = [&](const Vec2& p, Vec2& onto, Vec2& along) {
        double best = 1e30;
        for (const IslandRoad& f : w.roads)
            if (f.kind == "freeway")
                for (std::size_t q = 0; q + 1 < f.points.size(); ++q) {
                    const Vec2 a = f.points[q], ab = f.points[q + 1] - a;
                    const double L2 = ab.x * ab.x + ab.y * ab.y;
                    const double t = L2 > 1e-12 ? std::clamp(((p - a).x * ab.x + (p - a).y * ab.y) / L2, 0.0, 1.0) : 0.0;
                    const Vec2 fp = a + ab * t;
                    if ((p - fp).length() < best) { best = (p - fp).length(); onto = fp; along = ab * (1.0 / std::sqrt(std::max(L2, 1e-12))); }
                }
        return best;
    };
    // 1. EVERY ROAD THAT MEETS THE FREEWAY CROSSES IT: its end on the freeway carried on square across
    for (IslandRoad& rd : w.roads) {
        if (rd.kind == "freeway" || rd.points.size() < 2) continue;
        for (int end = 0; end < 2; ++end) {
            std::vector<Vec2>& P = rd.points;
            const Vec2 e = end ? P.back() : P.front(), prev = end ? P[P.size() - 2] : P[1];
            Vec2 onto, along;
            if (nearestFreeway(e, onto, along) > 5.0) continue;   // this end is not on the freeway
            Vec2 across(-along.y, along.x);
            if ((e - prev).x * across.x + (e - prev).y * across.y < 0) across = across * -1.0;   // on, the way it came
            // square across the freeway for the last 60 m before it, then on past it: into the city's
            // nearest arterial when one is there (a pass arriving at a town), else as far as the far
            // ramps need to land (90 m)
            std::vector<Vec2> tail;
            for (double t = 30.0; t <= 90.0; t += 30.0) {
                const Vec2 q = onto + across * t;
                if (w.heightAt(q.x, q.y) < 0.8) break;
                tail.push_back(q);
            }
            if (tail.empty()) continue;
            {
                // on into the place: to the FIRST street it meets beyond the far ramps (a T there), not on to
                // a junction further in -- that line crossed the streets between, which the builder read as
                // over- and underpasses (a 12 m mismatch in Saltwood, streets at 31%)
                double bestT = 500.0;
                Vec2 hit;
                const Vec2 from = tail.back();
                for (const auto& sg : cityStreets) {
                    const Vec2 a = sg.first, ab = sg.second - sg.first;
                    const Vec2 d = across * 500.0;
                    const double den = d.x * ab.y - d.y * ab.x;
                    if (std::fabs(den) < 1e-9) continue;
                    const Vec2 fa = a - from;
                    const double t = (fa.x * ab.y - fa.y * ab.x) / den, u = (fa.x * d.y - fa.y * d.x) / den;
                    if (t < 0 || t > 1 || u < 0 || u > 1) continue;
                    if (t * 500.0 < bestT) { bestT = t * 500.0; hit = from + d * t; }
                }
                if (bestT < 500.0) tail.push_back(hit);
            }
            const Vec2 lead = onto - across * 60.0;
            if (end) {
                while (P.size() > 1 && ((P.back() - onto).length() < 60.0)) P.pop_back();
                P.push_back(lead); P.push_back(onto);
                P.insert(P.end(), tail.begin(), tail.end());
            } else {
                while (P.size() > 1 && ((P.front() - onto).length() < 60.0)) P.erase(P.begin());
                std::vector<Vec2> head(tail.rbegin(), tail.rend());
                head.push_back(onto); head.push_back(lead);
                P.insert(P.begin(), head.begin(), head.end());
            }
        }
    }
    // ...and rounded again, so the turn onto the square crossing is a curve, not a kink; the crossing
    // itself stays square (the rounding is short against its 150 m straight)
    for (IslandRoad& rd : w.roads)
        if (rd.kind != "freeway" && rd.points.size() >= 3) rd.points = roundRoute(rd.points, rd.kind == "pass" ? 30.0 : 15.0);
    // 2. THE DIAMONDS
    std::vector<roads::lanes::RampStreet> streets;
    for (std::size_t k = 0; k < w.roads.size(); ++k)
        if (w.roads[k].kind != "freeway" && w.roads[k].points.size() >= 2) streets.push_back({w.roads[k].kind + std::to_string(k), w.roads[k].points, false});
    const HeightField ground = [&w](double x, double z) { return w.heightAt(x, z); };
    w.ramps.clear();
    w.rampEdges.clear();
    w.interchanges.clear();
    int built = 0, candidates = 0, oblique = 0, spacing = 0, terminal = 0, room = 0, conflict = 0;
    // the freeway as ONE route: its legs end to end round the island (each leg's end is the next one's
    // start, a city's waypoint -- right where that city's links reach it, so a seam there cost every
    // diamond), opened at the point farthest from any crossing
    std::vector<Vec2> ring;
    for (const IslandRoad& f : w.roads) {
        if (f.kind != "freeway" || f.points.size() < 2) continue;
        std::vector<Vec2> leg = f.points;
        if (!ring.empty() && (leg.back() - ring.back()).length() < (leg.front() - ring.back()).length()) std::reverse(leg.begin(), leg.end());
        for (const Vec2& q : leg) if (ring.empty() || (q - ring.back()).length() > 1.0) ring.push_back(q);
    }
    std::vector<Vec2> dense;
    for (std::size_t q = 0; q + 1 < ring.size(); ++q) {
        const Vec2 a2 = ring[q], b2 = ring[q + 1];
        const int m = std::max(1, static_cast<int>(std::ceil((b2 - a2).length() / 20.0)));
        for (int j = 0; j < m; ++j) dense.push_back(a2 + (b2 - a2) * (static_cast<double>(j) / m));
    }
    std::vector<std::vector<Vec2>> routes;
    if (dense.size() > 10 && (ring.front() - ring.back()).length() < 60.0) {
        // closed: open it where it is farthest from every crossing street
        std::size_t seam = 0;
        double far = -1;
        for (std::size_t q = 0; q < dense.size(); ++q) {
            double d = 1e30;
            for (const roads::lanes::RampStreet& st : streets) for (const Vec2& p : st.xy) d = std::min(d, (p - dense[q]).length());
            if (d > far) { far = d; seam = q; }
        }
        std::vector<Vec2> r(dense.begin() + static_cast<std::ptrdiff_t>(seam), dense.end());
        r.insert(r.end(), dense.begin(), dense.begin() + static_cast<std::ptrdiff_t>(seam) + 1);
        routes.push_back(r);
    } else if (dense.size() > 1) {
        dense.push_back(ring.back());
        routes.push_back(dense);
    }
    // along a route, where a point is (its station)
    auto stationOn = [](const std::vector<Vec2>& route, const Vec2& p) {
        double best = 1e30, bestS = 0.0, acc = 0.0;
        for (std::size_t q = 0; q + 1 < route.size(); ++q) {
            const Vec2 a2 = route[q], ab = route[q + 1] - a2;
            const double L2 = ab.x * ab.x + ab.y * ab.y, L = std::sqrt(L2);
            const double t = L2 > 1e-12 ? std::clamp(((p - a2).x * ab.x + (p - a2).y * ab.y) / L2, 0.0, 1.0) : 0.0;
            const double d = (p - (a2 + ab * t)).length();
            if (d < best) { best = d; bestS = acc + L * t; }
            acc += L;
        }
        return bestS;
    };
    std::set<std::string> served;   // the streets a diamond serves
    for (std::size_t k = 0; k < routes.size(); ++k) {
        const std::vector<Vec2>& route = routes[k];
        roads::lanes::DiamondOptions dop;
        dop.aId = "fw" + std::to_string(k) + "_a";
        dop.bId = "fw" + std::to_string(k) + "_b";
        // the planner's freeway section (plan_scene.h FreewaySection): four 3.75 m lanes a side
        dop.carriage = 12.0; dop.edgeReach = 22.0; dop.freewayLanes = 4; dop.freewayLaneW = 3.75;
        dop.maxDiamonds = 40;
        dop.spacing = 900.0;
        dop.clearance = 7.0;              // the crossing road passes under, the freeway on its embankment
        dop.rampHalf = 4.5 / 2 + 2.5;     // one 4.5 m lane, 2.5 m shoulders
        dop.gRamp = 0.09;
        dop.ground = ground;
        // THE DECK AS IT WILL BE BUILT: the lanes builder's own profile for this route (its freeway class:
        // a 200 m window at 6%), lifted to the underpass clearance over each road that crosses it and
        // falling away at design grade. Without it the generator assumed ground + clearance everywhere,
        // and on the foothills -- where the smoothed profile bridges the gullies -- ramps sized for 7 m had
        // 20 m to climb (island_scene.h floors the scene's freeway the same way)
        {
            const std::vector<double> zProf = roads::lanes::profileAlong(route, ground, 200.0, 0.06);
            std::vector<double> sProf{0.0};
            for (std::size_t q = 1; q < route.size(); ++q) sProf.push_back(sProf.back() + (route[q] - route[q - 1]).length());
            std::vector<std::pair<double, double>> tents;   // station, deck height
            for (const IslandRoad& rd : w.roads) {
                if (rd.kind == "freeway" || rd.points.size() < 2) continue;
                for (std::size_t i = 0; i + 1 < rd.points.size(); ++i)
                    for (std::size_t j = 0; j + 1 < route.size(); ++j) {
                        const Vec2 a = rd.points[i], b = rd.points[i + 1], c = route[j], d = route[j + 1];
                        const Vec2 r = b - a, sv = d - c;
                        const double den = r.x * sv.y - r.y * sv.x;
                        if (std::fabs(den) < 1e-12) continue;
                        const double t = ((c - a).x * sv.y - (c - a).y * sv.x) / den, u = ((c - a).x * r.y - (c - a).y * r.x) / den;
                        if (t < 0 || t > 1 || u < 0 || u > 1) continue;
                        const Vec2 x = a + r * t;
                        tents.push_back({sProf[j] + (sProf[j + 1] - sProf[j]) * u, w.heightAt(x.x, x.y) + 7.6});
                    }
            }
            const double gd = roads::lanes::kDesignGrade * 0.06;
            dop.deck = [sProf, zProf, tents, gd](double st) {
                const std::size_t k = std::min<std::size_t>(sProf.size() - 1, std::max<std::size_t>(1, static_cast<std::size_t>(std::upper_bound(sProf.begin(), sProf.end(), st) - sProf.begin())));
                const double f = (st - sProf[k - 1]) / std::max(1e-9, sProf[k] - sProf[k - 1]);
                double z = zProf[k - 1] + (zProf[k] - zProf[k - 1]) * std::clamp(f, 0.0, 1.0);
                for (const auto& [sc, zc] : tents) z = std::max(z, zc - gd * std::max(0.0, std::fabs(st - sc) - 30.0));
                return z;
            };
        }
        // ONE ATTEMPT: the diamond generator over `these` streets (the candidate, and the roads already
        // served, which a ramp must not pass over), keeping only the ramps that land on `want` (or all,
        // when it is empty). Returns whether `want` got its diamond.
        std::vector<std::pair<double, double>> laid;   // each diamond's crossing station, and its reach
        int attemptNo = 0;
        auto attempt = [&](const std::vector<roads::lanes::RampStreet>& these, const std::string& want, bool relaxed) {
            roads::lanes::DiamondOptions o = dop;
            o.idPrefix = "d" + std::to_string(k) + "_" + std::to_string(attemptNo++);
            if (relaxed) {
                o.spacing = 400.0;
                o.keepOut.clear();
                for (const auto& [c, r] : laid) o.keepOut.push_back({c - r, c + r});
            }
            const roads::lanes::DiamondResult dr = roads::lanes::diamondRamps(route, these, o);
            std::map<std::string, IslandInterchange> byDiamond;    // "<prefix>_<n>" -> its ramps
            bool got = false;
            for (const json& r : dr.ramps) {
                const bool off = r["to"].is_string();
                const std::string sid = off ? r["to"].get<std::string>() : r["from"].is_string() ? r["from"].get<std::string>() : "";
                if (!want.empty() && sid != want) continue;
                got = true;
                std::vector<Vec2> pts;
                for (const json& pt : r["path"]["points"]) pts.push_back(Vec2(pt[0].get<double>(), pt[1].get<double>()));
                w.ramps.push_back(pts);
                w.rampEdges.push_back(r);
                const json& anchor = off ? r["from"] : r["to"];
                served.insert(sid);
                // ids are <prefix>_<n>_<a|b>_<off|on>: the diamond is <prefix>_<n>
                const std::string id = r.value("id", std::string());
                const std::size_t cut = id.rfind('_', id.rfind('_') - 1);
                IslandInterchange& ic = byDiamond[id.substr(0, cut)];
                ic.road = std::atoi(sid.substr(sid.find_first_of("0123456789")).c_str());
                IslandInterchange::Ramp rp;
                rp.off = off;
                const std::string edge = anchor.value("edge", std::string());
                rp.withRoute = !edge.empty() && edge.back() == 'a';   // carriageway a runs with the route
                rp.gorePt = Vec2(anchor["at"][0].get<double>(), anchor["at"][1].get<double>());
                rp.gore = stationOn(route, rp.gorePt);
                rp.terminal = pts.empty() ? rp.gorePt : (off ? pts.back() : pts.front());
                rp.path = pts;
                ic.ramps.push_back(rp);
            }
            for (auto& [key, ic] : byDiamond) {
                double sum = 0;
                for (const IslandInterchange::Ramp& rp : ic.ramps) sum += rp.gore;
                ic.station = sum / static_cast<double>(std::max<std::size_t>(1, ic.ramps.size()));
                // where the road crosses: its point nearest the FREEWAY at that station (a point up a
                // mountain road can project onto the same station from a kilometre away)
                Vec2 onRoute = route.front();
                {
                    double acc = 0;
                    for (std::size_t q = 0; q + 1 < route.size(); ++q) {
                        const double L2 = (route[q + 1] - route[q]).length();
                        if (acc + L2 >= ic.station) { onRoute = route[q] + (route[q + 1] - route[q]) * ((ic.station - acc) / std::max(1e-9, L2)); break; }
                        acc += L2;
                    }
                }
                double best = 1e30;
                for (const Vec2& q : w.roads[static_cast<std::size_t>(ic.road)].points)
                    if ((q - onRoute).length() < best) { best = (q - onRoute).length(); ic.at = q; }
                w.interchanges.push_back(ic);
                // what it takes of the freeway, PER DIAMOND (a road crossing twice -- the pass, one at each
                // end -- is two interchanges; kept by street, their gores averaged to a "diamond" in the
                // middle of the island whose reach blocked 20 km of it)
                double lo = ic.station, hi = ic.station;
                for (const IslandInterchange::Ramp& rp : ic.ramps) { lo = std::min(lo, rp.gore); hi = std::max(hi, rp.gore); }
                dop.keepOut.push_back({ic.station - dop.spacing + 200.0, ic.station + dop.spacing - 200.0});
                laid.push_back({ic.station, std::max(ic.station - lo, hi - ic.station) + 150.0});   // its gores' reach, and the lanes beyond
                ++built;
            }
            candidates += dr.candidates; oblique += dr.rejectedOblique; spacing += dr.rejectedSpacing;
            terminal += dr.rejectedTerminal; room += dr.rejectedRoom; conflict += dr.rejectedConflict;
            return got;
        };
        auto servedStreets = [&] {
            std::vector<roads::lanes::RampStreet> out;
            for (const roads::lanes::RampStreet& st : streets) if (served.count(st.id)) out.push_back(st);
            return out;
        };
        // FIRST the pass and the mountain road -- the only way on from the hills
        {
            std::vector<roads::lanes::RampStreet> country;
            for (const roads::lanes::RampStreet& st : streets) if (st.id.rfind("link", 0) != 0) country.push_back(st);
            if (!country.empty()) attempt(country, "", false);
        }
        // THEN each place's links, one candidate at a time, best (shortest) first, until it has its
        // share (a city three, a town one) -- all of them together blocked each other's ramps
        std::vector<int> order;
        for (std::size_t si = 0; si < w.sites.size(); ++si) order.push_back(static_cast<int>(si));
        std::stable_sort(order.begin(), order.end(), [&](int a2, int b2) { return (w.sites[static_cast<std::size_t>(a2)].kind == "city") > (w.sites[static_cast<std::size_t>(b2)].kind == "city"); });
        auto candidatesOf = [&](int site) {
            std::vector<std::pair<double, std::string>> c;
            for (const roads::lanes::RampStreet& st : streets) {
                if (st.id.rfind("link", 0) != 0) continue;
                const IslandRoad& rd = w.roads[static_cast<std::size_t>(std::atoi(st.id.substr(4).c_str()))];
                if (rd.from == site) c.push_back({rd.length, st.id});
            }
            std::sort(c.begin(), c.end());
            return c;
        };
        std::map<int, int> got;
        for (int relaxedPass = 0; relaxedPass < 2; ++relaxedPass)
            for (int site : order) {
                const int want = w.sites[static_cast<std::size_t>(site)].kind == "city" ? 3 : 1;
                if (relaxedPass && got[site] > 0) continue;   // LAST: only places still cut off, spacing relaxed
                for (const auto& [len, sid] : candidatesOf(site)) {
                    if (got[site] >= (relaxedPass ? 1 : want) || served.count(sid)) continue;
                    std::vector<roads::lanes::RampStreet> these = servedStreets();
                    for (const roads::lanes::RampStreet& st : streets) if (st.id == sid) these.push_back(st);
                    if (attempt(these, sid, relaxedPass == 1)) ++got[site];
                }
            }
    }
    // THE ROUTE the interchanges are numbered along, and which way round it runs
    if (routes.size() == 1) {
        w.freewayRoute = routes.front();
        double area2 = 0.0;
        const std::vector<Vec2>& R = w.freewayRoute;
        for (std::size_t q = 0; q + 1 < R.size(); ++q) area2 += R[q].x * R[q + 1].y - R[q + 1].x * R[q].y;
        w.routeClockwise = area2 < 0.0;   // x east, y north: a negative area runs clockwise
        double L = 0.0;
        for (std::size_t q = 0; q + 1 < R.size(); ++q) L += (R[q + 1] - R[q]).length();
        std::sort(w.interchanges.begin(), w.interchanges.end(), [](const IslandInterchange& a, const IslandInterchange& b) { return a.station < b.station; });
        // EXIT NUMBERS: the km along the Inner Loop (clockwise) from the route's start; two in one km get A, B
        std::map<int, int> count;
        std::vector<int> num;
        for (const IslandInterchange& ic : w.interchanges) {
            const double d = w.routeClockwise ? ic.station : L - ic.station;
            num.push_back(static_cast<int>(std::floor(d / 1000.0)) + 1);
            ++count[num.back()];
        }
        std::map<int, int> seen;
        for (std::size_t k = 0; k < w.interchanges.size(); ++k) {
            IslandInterchange& ic = w.interchanges[k];
            ic.exit = std::to_string(num[k]);
            if (count[num[k]] > 1) ic.exit += static_cast<char>('A' + seen[num[k]]++);
            // the place it serves: the site whose limits (or centre) is nearest the crossing
            double best = 1e30;
            for (std::size_t si = 0; si < w.sites.size(); ++si) {
                double d = (w.sites[si].at - ic.at).length();
                for (const auto& loop : w.sites[si].limits) for (const Vec2& q : loop) d = std::min(d, (q - ic.at).length());
                if (d < best && d < 1500.0) { best = d; ic.site = static_cast<int>(si); }
            }
        }
    }
    // a link that got no interchange has no reason to be: it goes (a pass or mountain road without one
    // still crosses, under the freeway)
    int dropped = 0;
    for (std::size_t k = 0; k < w.roads.size(); ++k)
        if (w.roads[k].kind == "link" && !served.count("link" + std::to_string(k))) { w.roads[k].points.clear(); ++dropped; }
    w.report["linksDropped"] = dropped;
    w.report["interchanges"] = {{"built", built}, {"crossings", candidates}, {"ramps", w.ramps.size()},
                                {"refused", {{"oblique", oblique}, {"spacing", spacing}, {"noTerminal", terminal}, {"noRoom", room}, {"overAStreet", conflict}}}};
}

void planIntercityBuses(IslandWorld& w, const std::function<std::vector<Vec2>(int, const Vec2&, const Vec2&)>& streets) {
    const std::vector<Vec2>& R = w.freewayRoute;
    if (R.size() < 3 || w.centres.size() != w.sites.size()) return;
    std::vector<double> st{0.0};
    for (std::size_t k = 1; k < R.size(); ++k) st.push_back(st.back() + (R[k] - R[k - 1]).length());
    const double L = st.back();
    auto wrap = [&](double s) { return std::fmod(std::fmod(s, L) + L, L); };
    // the freeway between two stations, the shorter way round (or forward, when told)
    auto freeway = [&](double s0, double s1, int forceDir) {
        const double fwd = wrap(s1 - s0);
        const int dir = forceDir ? forceDir : (fwd <= L - fwd ? +1 : -1);
        const double len = dir > 0 ? fwd : L - fwd;
        std::vector<Vec2> out;
        for (double d = 0; d <= len; d += 40.0) {
            const double s = wrap(s0 + dir * d);
            const std::size_t k = std::min<std::size_t>(R.size() - 1, std::max<std::size_t>(1, static_cast<std::size_t>(std::upper_bound(st.begin(), st.end(), s) - st.begin())));
            const double t = (s - st[k - 1]) / std::max(1e-9, st[k] - st[k - 1]);
            out.push_back(R[k - 1] + (R[k] - R[k - 1]) * t);
        }
        return out;
    };
    auto append = [](std::vector<Vec2>& a, const std::vector<Vec2>& b) { a.insert(a.end(), b.begin(), b.end()); };
    auto reversed = [](std::vector<Vec2> v) { std::reverse(v.begin(), v.end()); return v; };
    // a road from the point on it nearest `from` to its end nearest `to`
    auto roadPart = [](const std::vector<Vec2>& P, const Vec2& from, const Vec2& toward) {
        std::size_t i = 0;
        double bd = 1e30;
        for (std::size_t k = 0; k < P.size(); ++k) if ((P[k] - from).length() < bd) { bd = (P[k] - from).length(); i = k; }
        std::vector<Vec2> out;
        if ((P.front() - toward).length() < (P.back() - toward).length()) for (std::size_t k = i + 1; k-- > 0;) out.push_back(P[k]);
        else for (std::size_t k = i; k < P.size(); ++k) out.push_back(P[k]);
        return out;
    };
    // each place's way onto the freeway: its link interchange nearest its centre
    std::vector<int> access(w.sites.size(), -1);
    for (std::size_t k = 0; k < w.interchanges.size(); ++k) {
        const IslandInterchange& ic = w.interchanges[k];
        if (ic.site < 0 || w.roads[static_cast<std::size_t>(ic.road)].kind != "link") continue;
        const std::size_t si = static_cast<std::size_t>(ic.site);
        if (access[si] < 0 || (ic.at - w.centres[si]).length() < (w.interchanges[static_cast<std::size_t>(access[si])].at - w.centres[si]).length())
            access[si] = static_cast<int>(k);
    }
    // centre -> the freeway, and back: its streets, then its link road
    auto toFreeway = [&](int site) {
        const IslandInterchange& ic = w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(site)])];
        const std::vector<Vec2> link = roadPart(w.roads[static_cast<std::size_t>(ic.road)].points, ic.at, w.centres[static_cast<std::size_t>(site)]);   // freeway -> town
        std::vector<Vec2> p = streets(site, w.centres[static_cast<std::size_t>(site)], link.empty() ? ic.at : link.back());
        append(p, reversed(link));
        return p;
    };
    auto label = [&](int site) { return w.sites[static_cast<std::size_t>(site)].name; };
    w.busLines.erase(std::remove_if(w.busLines.begin(), w.busLines.end(), [](const IslandBusLine& b) { return b.kind == "intercity"; }), w.busLines.end());
    // X1 THE ISLAND RING: every place with a way onto the freeway, in order round it
    {
        std::vector<int> order;
        for (std::size_t si = 0; si < w.sites.size(); ++si) if (access[si] >= 0) order.push_back(static_cast<int>(si));
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(a)])].station < w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(b)])].station;
        });
        if (order.size() >= 2) {
            IslandBusLine line;
            line.name = "X1 Island Ring";
            line.kind = "intercity";
            for (std::size_t k = 0; k < order.size(); ++k) {
                const int a = order[k], b = order[(k + 1) % order.size()];
                const std::vector<Vec2> out = toFreeway(a);
                append(line.path, reversed(out));   // in from the freeway to the centre...
                line.stops.push_back(w.centres[static_cast<std::size_t>(a)]);
                line.stopNames.push_back(label(a));
                append(line.path, out);             // ...and back out
                append(line.path, freeway(w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(a)])].station,
                                          w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(b)])].station, +1));
            }
            w.busLines.push_back(line);
        }
    }
    // the place at a road's end: the site whose limits (or centre) is nearest, within 1.5 km
    auto placeAt = [&](const Vec2& p) {
        int best = -1;
        double bd = 1500.0;
        for (std::size_t si = 0; si < w.sites.size(); ++si) {
            double d = (w.sites[si].at - p).length();
            for (const auto& loop : w.sites[si].limits) for (const Vec2& q : loop) d = std::min(d, (q - p).length());
            if (d < bd) { bd = d; best = static_cast<int>(si); }
        }
        return best;
    };
    for (const IslandRoad& rd : w.roads) {
        if (rd.points.size() < 2) continue;
        // X2 OVER THE PASS: a centre each end, Route 2 between, a stop at the summit
        if (rd.kind == "pass") {
            const int a = placeAt(rd.points.front()), b = placeAt(rd.points.back());
            if (a < 0 || b < 0 || a == b) continue;
            IslandBusLine line;
            line.name = "X2 Over the Pass: " + label(a) + " \u2013 " + label(b);
            line.kind = "intercity";
            line.path = streets(a, w.centres[static_cast<std::size_t>(a)], rd.points.front());
            append(line.path, rd.points);
            append(line.path, streets(b, rd.points.back(), w.centres[static_cast<std::size_t>(b)]));
            // the summit: the pass's highest point
            Vec2 top = rd.points.front();
            for (const Vec2& q : rd.points) if (w.heightAt(q.x, q.y) > w.heightAt(top.x, top.y)) top = q;
            line.stops = {w.centres[static_cast<std::size_t>(a)], top, w.centres[static_cast<std::size_t>(b)]};
            line.stopNames = {label(a), "Pass Summit", label(b)};
            w.busLines.push_back(line);
        }
        // X3 THE MOUNTAIN SHUTTLE: from the place whose way onto the freeway is nearest the road's foot,
        // along the freeway, up Route 3
        if (rd.kind == "mountain" && rd.to >= 0) {
            const Vec2 top = w.centres[static_cast<std::size_t>(rd.to)];
            const std::vector<Vec2> up = (rd.points.front() - top).length() > (rd.points.back() - top).length() ? rd.points : reversed(rd.points);
            // its interchange: the one on this road
            int icM = -1;
            for (std::size_t k = 0; k < w.interchanges.size(); ++k)
                if (&w.roads[static_cast<std::size_t>(w.interchanges[k].road)] == &rd) icM = static_cast<int>(k);
            if (icM < 0) continue;
            int from = -1;
            double bd = 1e30;
            for (std::size_t si = 0; si < w.sites.size(); ++si) {
                if (access[si] < 0) continue;
                const double s0 = w.interchanges[static_cast<std::size_t>(access[si])].station, s1 = w.interchanges[static_cast<std::size_t>(icM)].station;
                const double d = std::min(wrap(s1 - s0), L - wrap(s1 - s0));
                if (d < bd) { bd = d; from = static_cast<int>(si); }
            }
            if (from < 0) continue;
            IslandBusLine line;
            line.name = "X3 Mountain Shuttle: " + label(from) + " \u2013 " + label(rd.to);
            line.kind = "intercity";
            line.path = toFreeway(from);
            append(line.path, freeway(w.interchanges[static_cast<std::size_t>(access[static_cast<std::size_t>(from)])].station, w.interchanges[static_cast<std::size_t>(icM)].station, 0));
            append(line.path, roadPart(up, w.interchanges[static_cast<std::size_t>(icM)].at, top));
            if (std::getenv("RT_BUS_WHY")) std::printf("[bus] shuttle: road %zu pts, from ic (%.0f, %.0f), up front (%.0f,%.0f) back (%.0f,%.0f), top (%.0f,%.0f)\n", up.size(),
                w.interchanges[static_cast<std::size_t>(icM)].at.x, w.interchanges[static_cast<std::size_t>(icM)].at.y, up.front().x, up.front().y, up.back().x, up.back().y, top.x, top.y);
            line.stops = {w.centres[static_cast<std::size_t>(from)], top};
            line.stopNames = {label(from), label(rd.to)};
            w.busLines.push_back(line);
        }
    }
}

bool writeIslandMap(const IslandWorld& w, const std::string& pngPath, int px, const IslandMapView& view) {
    const double vhalf = view.half > 0.0 ? view.half : w.half;
    const Vec2 vc = view.half > 0.0 ? view.centre : Vec2(0, 0);
    const double mpp = 2.0 * vhalf / px;   // metres a pixel
    std::vector<float> img(static_cast<std::size_t>(px) * px * 3, 0.0f);
    auto put = [&](int x, int y, float r, float g, float b, float a = 1.0f) {
        if (x < 0 || y < 0 || x >= px || y >= px) return;
        float* p = &img[(static_cast<std::size_t>(y) * px + x) * 3];
        p[0] += (r - p[0]) * a; p[1] += (g - p[1]) * a; p[2] += (b - p[2]) * a;
    };
    auto toPx = [&](const Vec2& q, double& x, double& y) { x = (q.x - vc.x + vhalf) / mpp; y = (vc.y + vhalf - q.y) / mpp; };   // north (+z) up
    auto mix3 = [](const float* a, const float* b, float t, float* o) { for (int k = 0; k < 3; ++k) o[k] = a[k] + (b[k] - a[k]) * t; };
    // relief and water
    const float sand[3] = {0.86f, 0.80f, 0.60f}, low[3] = {0.40f, 0.58f, 0.28f}, mid[3] = {0.46f, 0.52f, 0.30f},
                high[3] = {0.55f, 0.48f, 0.36f}, rock[3] = {0.60f, 0.58f, 0.55f}, snow[3] = {0.93f, 0.94f, 0.96f},
                shallow[3] = {0.55f, 0.80f, 0.85f}, deep[3] = {0.10f, 0.28f, 0.48f};
    const double lx = -0.6, ly = 0.75, lz = 0.9, ll = std::sqrt(lx * lx + ly * ly + lz * lz);
    for (int y = 0; y < px; ++y)
        for (int x = 0; x < px; ++x) {
            const double wx = vc.x - vhalf + (x + 0.5) * mpp, wz = vc.y + vhalf - (y + 0.5) * mpp;
            const double h = w.heightAt(wx, wz);
            float c[3];
            if (h < 0.0) {
                mix3(shallow, deep, static_cast<float>(std::clamp(-h / 40.0, 0.0, 1.0)), c);
            } else {
                const double t = h;
                if (t < 3) mix3(sand, low, static_cast<float>(t / 3.0), c);
                else if (t < 120) mix3(low, mid, static_cast<float>((t - 3) / 117.0), c);
                else if (t < 450) mix3(mid, high, static_cast<float>((t - 120) / 330.0), c);
                else if (t < 900) mix3(high, rock, static_cast<float>((t - 450) / 450.0), c);
                else mix3(rock, snow, static_cast<float>(std::clamp((t - 900) / 300.0, 0.0, 1.0)), c);
                // hillshade from the northwest
                const double gx = (w.heightAt(wx + mpp, wz) - w.heightAt(wx - mpp, wz)) / (2 * mpp);
                const double gz = (w.heightAt(wx, wz + mpp) - w.heightAt(wx, wz - mpp)) / (2 * mpp);
                const double nx = -gx, ny = 1.0, nzz = -gz, nl = std::sqrt(nx * nx + ny * ny + nzz * nzz);
                const double shade = std::clamp(0.45 + 0.75 * (nx * lx + ny * lz + nzz * ly) / (nl * ll), 0.25, 1.25);
                for (float& k : c) k = static_cast<float>(std::min(1.0, k * shade));
                // buildable: warmed toward straw
                const int gi = std::clamp(static_cast<int>(std::lround((wx + w.half) / w.cell)), 0, w.n - 1);
                const int gj = std::clamp(static_cast<int>(std::lround((wz + w.half) / w.cell)), 0, w.n - 1);
                if (w.buildable[static_cast<std::size_t>(gj) * w.n + gi]) { c[0] = c[0] * 0.7f + 0.95f * 0.3f; c[1] = c[1] * 0.7f + 0.88f * 0.3f; c[2] = c[2] * 0.7f + 0.50f * 0.3f; }
                // contours every 100 m
                if (std::floor(h / 100.0) != std::floor(w.heightAt(wx + mpp, wz) / 100.0) ||
                    std::floor(h / 100.0) != std::floor(w.heightAt(wx, wz - mpp) / 100.0))
                    for (float& k : c) k *= 0.8f;
            }
            float* p = &img[(static_cast<std::size_t>(y) * px + x) * 3];
            p[0] = c[0]; p[1] = c[1]; p[2] = c[2];
        }
    // a thick polyline: a stamped disc every half pixel
    auto stroke = [&](const std::vector<Vec2>& pts, double widthPx, float r, float g, float b) {
        for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
            double ax, ay, bx, by;
            toPx(pts[k], ax, ay);
            toPx(pts[k + 1], bx, by);
            const int m = std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) * 2)));
            for (int s = 0; s <= m; ++s) {
                const double cx = ax + (bx - ax) * s / m, cy = ay + (by - ay) * s / m, rr = widthPx * 0.5;
                for (int yy = static_cast<int>(cy - rr - 1); yy <= static_cast<int>(cy + rr + 1); ++yy)
                    for (int xx = static_cast<int>(cx - rr - 1); xx <= static_cast<int>(cx + rr + 1); ++xx) {
                        const double d = std::hypot(xx + 0.5 - cx, yy + 0.5 - cy);
                        if (d <= rr) put(xx, yy, r, g, b, static_cast<float>(std::clamp(rr + 0.5 - d, 0.0, 1.0)));
                    }
            }
        }
    };
    if (w.hydro)
        for (const River& rv : w.hydro->rivers()) {
            std::vector<Vec2> pts;
            double wsum = 0;
            for (const RiverNode& nd : rv.nodes) { pts.push_back(nd.p); wsum += nd.width; }
            stroke(pts, std::max(1.4, wsum / std::max<std::size_t>(1, rv.nodes.size()) / mpp), 0.20f, 0.45f, 0.80f);
        }
    for (const IslandMapLayer& L : view.layers) {
        const double wpx = std::max(L.minPx, L.widthM / mpp);
        if (L.casing) for (const auto& ln : L.lines) stroke(ln, wpx + 2.0, 0.12f, 0.10f, 0.08f);   // every casing, then every fill
        for (const auto& ln : L.lines) stroke(ln, wpx, L.rgb[0], L.rgb[1], L.rgb[2]);
    }
    for (const IslandRoad& rd : w.roads) {
        if (rd.points.empty()) continue;
        // to scale when zoomed in (the freeway's pavement is 44 m kerb to kerb), never thinner than a
        // line that reads on the whole island
        auto wide = [&](double metres, double px) { return std::max(px, metres / mpp); };
        if (rd.kind == "freeway") { const double W = wide(44.0, 4.0); stroke(rd.points, W + 2.0, 0.15f, 0.10f, 0.05f); stroke(rd.points, W, 0.98f, 0.60f, 0.10f); }
        else if (rd.kind == "pass") { const double W = wide(11.0, 3.0); stroke(rd.points, W + 1.5, 0.15f, 0.10f, 0.05f); stroke(rd.points, W, 0.99f, 0.90f, 0.25f); }
        else if (rd.kind == "link") { const double W = wide(16.0, 2.6); stroke(rd.points, W + 1.5, 0.15f, 0.10f, 0.05f); stroke(rd.points, W, 0.99f, 0.80f, 0.20f); }
        else { const double W = wide(9.0, 1.8); stroke(rd.points, W + 1.4, 0.15f, 0.10f, 0.05f); stroke(rd.points, W, 1.0f, 1.0f, 1.0f); }
    }
    for (const std::vector<Vec2>& r : w.ramps) {   // the ramps, over the roads they join
        stroke(r, std::max(1.2, 9.5 / mpp) + 1.5, 0.15f, 0.10f, 0.05f);
        stroke(r, std::max(1.0, 9.5 / mpp), 0.98f, 0.72f, 0.30f);
    }
    for (const IslandSite& s : w.sites) {
        if (!view.sites) break;
        double cx, cy;
        toPx(s.at, cx, cy);
        const float r = s.kind == "city" ? 0.85f : s.kind == "town" ? 0.95f : 0.62f;
        const float g = s.kind == "city" ? 0.12f : s.kind == "town" ? 0.45f : 0.25f;
        const float b = s.kind == "city" ? 0.12f : s.kind == "town" ? 0.05f : 0.75f;
        const double R = s.radius / mpp;
        for (int k = 0; k < 720; ++k) {   // the footprint's outline
            const double a = 2 * kPi * k / 720;
            for (double t = -1.2; t <= 1.2; t += 0.4) put(static_cast<int>(cx + std::cos(a) * (R + t)), static_cast<int>(cy + std::sin(a) * (R + t)), r, g, b);
        }
        const double dot = s.kind == "city" ? 7.0 : 5.0;
        for (int yy = static_cast<int>(cy - dot - 2); yy <= static_cast<int>(cy + dot + 2); ++yy)
            for (int xx = static_cast<int>(cx - dot - 2); xx <= static_cast<int>(cx + dot + 2); ++xx) {
                const double d = std::hypot(xx + 0.5 - cx, yy + 0.5 - cy);
                if (d <= dot + 1.5) put(xx, yy, 0.05f, 0.05f, 0.05f);
                if (d <= dot) put(xx, yy, r, g, b);
            }
    }
    // a 5 km scale bar, bottom left
    {
        const double barM = vhalf > 4000.0 ? 5000.0 : vhalf > 1000.0 ? 1000.0 : 200.0;   // five ticks
        const int x0 = 30, y0 = px - 34, len = static_cast<int>(barM / mpp);
        for (int yy = y0 - 3; yy <= y0 + 3; ++yy)
            for (int xx = x0 - 3; xx <= x0 + len + 3; ++xx) put(xx, yy, 0.05f, 0.05f, 0.05f);
        for (int yy = y0 - 1; yy <= y0 + 1; ++yy)
            for (int xx = x0; xx <= x0 + len; ++xx) put(xx, yy, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f);
    }
    std::vector<unsigned char> out(img.size());
    for (std::size_t k = 0; k < img.size(); ++k) out[k] = static_cast<unsigned char>(std::clamp(img[k], 0.0f, 1.0f) * 255.0f + 0.5f);
    return stbi_write_png(pngPath.c_str(), px, px, 3, out.data(), px * 3) != 0;
}

namespace {
// hypsometric tint and northwest hillshade, `px` a side over the square (c, half): the relief alone
std::vector<unsigned char> reliefPixels(const IslandWorld& w, int px, const Vec2& c, double half) {
    const double mpp = 2.0 * half / px;
    std::vector<unsigned char> out(static_cast<std::size_t>(px) * px * 3);
    auto mix3 = [](const float* a, const float* b, float t, float* o) { for (int k = 0; k < 3; ++k) o[k] = a[k] + (b[k] - a[k]) * t; };
    const float sand[3] = {0.86f, 0.80f, 0.60f}, low[3] = {0.40f, 0.58f, 0.28f}, mid[3] = {0.46f, 0.52f, 0.30f},
                high[3] = {0.55f, 0.48f, 0.36f}, rock[3] = {0.60f, 0.58f, 0.55f}, snow[3] = {0.93f, 0.94f, 0.96f},
                shallow[3] = {0.55f, 0.80f, 0.85f}, deep[3] = {0.10f, 0.28f, 0.48f};
    const double lx = -0.6, ly = 0.75, lz = 0.9, ll = std::sqrt(lx * lx + ly * ly + lz * lz);
    for (int y = 0; y < px; ++y)
        for (int x = 0; x < px; ++x) {
            const double wx = c.x - half + (x + 0.5) * mpp, wz = c.y + half - (y + 0.5) * mpp;
            const double h = w.heightAt(wx, wz);
            float col[3];
            if (h < 0.0) mix3(shallow, deep, static_cast<float>(std::clamp(-h / 40.0, 0.0, 1.0)), col);
            else {
                if (h < 3) mix3(sand, low, static_cast<float>(h / 3.0), col);
                else if (h < 120) mix3(low, mid, static_cast<float>((h - 3) / 117.0), col);
                else if (h < 450) mix3(mid, high, static_cast<float>((h - 120) / 330.0), col);
                else if (h < 900) mix3(high, rock, static_cast<float>((h - 450) / 450.0), col);
                else mix3(rock, snow, static_cast<float>(std::clamp((h - 900) / 300.0, 0.0, 1.0)), col);
                const double e = std::max(mpp, w.cell * 0.5);
                const double gx = (w.heightAt(wx + e, wz) - w.heightAt(wx - e, wz)) / (2 * e);
                const double gz = (w.heightAt(wx, wz + e) - w.heightAt(wx, wz - e)) / (2 * e);
                const double nx = -gx, nzz = -gz, nl = std::sqrt(nx * nx + 1.0 + nzz * nzz);
                const double shade = std::clamp(0.45 + 0.75 * (nx * lx + lz + nzz * ly) / (nl * ll), 0.25, 1.25);
                for (float& k : col) k = static_cast<float>(std::min(1.0, k * shade));
            }
            unsigned char* o = &out[(static_cast<std::size_t>(y) * px + x) * 3];
            for (int k = 0; k < 3; ++k) o[k] = static_cast<unsigned char>(std::clamp(col[k], 0.0f, 1.0f) * 255.0f + 0.5f);
        }
    return out;
}
std::string base64(const std::vector<unsigned char>& in) {
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < in.size(); i += 3) {
        const unsigned v = (in[i] << 16) | ((i + 1 < in.size() ? in[i + 1] : 0) << 8) | (i + 2 < in.size() ? in[i + 2] : 0);
        out += T[(v >> 18) & 63];
        out += T[(v >> 12) & 63];
        out += i + 1 < in.size() ? T[(v >> 6) & 63] : '=';
        out += i + 2 < in.size() ? T[v & 63] : '=';
    }
    return out;
}
}  // namespace

bool writeIslandSvg(const IslandWorld& w, const std::string& svgPath, const IslandMapView& view, int reliefPx) {
    const double half = view.half > 0.0 ? view.half : w.half;
    const Vec2 c = view.half > 0.0 ? view.centre : Vec2(0, 0);
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(1);
    // user units are metres; y is -z (north up)
    auto path = [&](const std::vector<Vec2>& pts, bool closed = false) {
        for (std::size_t k = 0; k < pts.size(); ++k) o << (k ? 'L' : 'M') << pts[k].x << ' ' << -pts[k].y << ' ';
        if (closed) o << 'Z';
    };
    auto lines = [&](const std::vector<std::vector<Vec2>>& ls, const std::string& attrs) {
        if (ls.empty()) return;
        o << "<path " << attrs << " d=\"";
        for (const auto& l : ls) if (l.size() >= 2) path(l, l.size() > 2 && (l.front() - l.back()).length() < 25.0);
        o << "\"/>\n";
    };
    const double x0 = c.x - half, y0 = -(c.y + half), W = 2 * half;
    o << "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"" << x0 << ' ' << y0 << ' ' << W << ' ' << W
      << "\" width=\"2000\" height=\"2000\">\n";
    o << "<style>path{fill:none;stroke-linecap:round;stroke-linejoin:round}"
         ".hair{vector-effect:non-scaling-stroke}"
         "text{font-family:sans-serif;paint-order:stroke;stroke:#fff;stroke-width:0.25em;stroke-linejoin:round}</style>\n";
    // THE RELIEF: one image at the terrain's resolution
    {
        std::vector<unsigned char> px = reliefPixels(w, reliefPx, c, half), png;
        stbi_write_png_to_func([](void* ctx, void* data, int size) {
            auto* v = static_cast<std::vector<unsigned char>*>(ctx);
            v->insert(v->end(), static_cast<unsigned char*>(data), static_cast<unsigned char*>(data) + size);
        }, &png, reliefPx, reliefPx, 3, px.data(), reliefPx * 3);
        o << "<g id=\"relief\"><image x=\"" << x0 << "\" y=\"" << y0 << "\" width=\"" << W << "\" height=\"" << W
          << "\" preserveAspectRatio=\"none\" xlink:href=\"data:image/png;base64," << base64(png) << "\"/></g>\n";
    }
    // CONTOURS every 20 m, 100 m heavier; the COASTLINE
    {
        const Vec2 origin(-w.half, -w.half);
        o << "<g id=\"contours\" opacity=\"0.55\">\n";
        for (int lv = 20; lv <= 1400; lv += 20) {
            const auto ls = plan::isoLines(w.height, w.n, origin, w.cell, lv, 20.0, 120.0, 0.0);
            lines(ls, lv % 100 ? "class=\"hair\" stroke=\"#4a3c2c\" stroke-width=\"0.35\""
                               : "class=\"hair\" stroke=\"#3a2c1c\" stroke-width=\"0.9\"");
        }
        o << "</g>\n<g id=\"coast\">\n";
        lines(plan::isoLines(w.height, w.n, origin, w.cell, 0.0, 20.0, 150.0, 0.0), "class=\"hair\" stroke=\"#1d4f7a\" stroke-width=\"1.2\"");
        o << "</g>\n";
    }
    // RIVERS at their widths (in runs of a few nodes, each at its mean width)
    o << "<g id=\"rivers\" stroke=\"#3372c4\">\n";
    if (w.hydro)
        for (const River& rv : w.hydro->rivers())
            for (std::size_t a = 0; a + 1 < rv.nodes.size(); a += 6) {
                const std::size_t b = std::min(rv.nodes.size() - 1, a + 7);
                std::vector<Vec2> pts;
                double ws = 0;
                for (std::size_t k = a; k <= b; ++k) { pts.push_back(rv.nodes[k].p); ws += rv.nodes[k].width; }
                o << "<path stroke-width=\"" << ws / static_cast<double>(b - a + 1) << "\" d=\"";
                path(pts);
                o << "\"/>\n";
            }
    o << "</g>\n";
    // the view's layers (a city's limits and streets), to scale
    for (const IslandMapLayer& L : view.layers) {
        char rgb[8];
        std::snprintf(rgb, sizeof rgb, "#%02x%02x%02x", static_cast<int>(L.rgb[0] * 255), static_cast<int>(L.rgb[1] * 255), static_cast<int>(L.rgb[2] * 255));
        o << "<g id=\"" << (L.name.empty() ? std::string("layer") : L.name) << "\">\n";
        std::ostringstream a;
        a.setf(std::ios::fixed);
        a.precision(1);
        if (L.casing) { a << "stroke=\"#1f1a14\" stroke-width=\"" << L.widthM + 3.0 << "\""; lines(L.lines, a.str()); a.str(""); }
        a << "stroke=\"" << rgb << "\" stroke-width=\"" << L.widthM << "\"" << (L.widthM < 6.0 ? " class=\"hair\"" : "");
        lines(L.lines, a.str());
        o << "</g>\n";
    }
    // THE ISLAND'S ROADS: freeway (44 m kerb to kerb), its ramps, the pass, the links, the mountain road
    auto roadsOf = [&](const std::string& kind) {
        std::vector<std::vector<Vec2>> ls;
        for (const IslandRoad& rd : w.roads) if (rd.kind == kind && rd.points.size() >= 2) ls.push_back(rd.points);
        return ls;
    };
    o << "<g id=\"roads-country\">\n";
    for (const auto& [kind, wd, col] : std::vector<std::tuple<std::string, double, std::string>>{
             {"mountain", 9.0, "#ffffff"}, {"pass", 11.0, "#fbe43f"}, {"link", 16.0, "#fbcc33"}}) {
        lines(roadsOf(kind), "stroke=\"#1f1a14\" stroke-width=\"" + std::to_string(wd + 3.0) + "\"");
        lines(roadsOf(kind), "stroke=\"" + col + "\" stroke-width=\"" + std::to_string(wd) + "\"");
    }
    o << "</g>\n<g id=\"freeway\">\n";
    lines(roadsOf("freeway"), "stroke=\"#1f1a14\" stroke-width=\"47\"");
    lines(roadsOf("freeway"), "stroke=\"#f8961a\" stroke-width=\"44\"");
    lines(roadsOf("freeway"), "stroke=\"#ffffff\" stroke-width=\"0.6\" stroke-dasharray=\"12 18\"");   // the median line
    o << "</g>\n<g id=\"ramps\">\n";
    lines(w.ramps, "stroke=\"#1f1a14\" stroke-width=\"12\"");
    lines(w.ramps, "stroke=\"#fbb54d\" stroke-width=\"9.5\"");
    o << "</g>\n";
    // BUS LINES: local loops thin, intercity lines bold and dashed; stops as white dots
    o << "<g id=\"buses\">\n";
    {
        const char* localCols[] = {"#e6194b", "#3cb44b", "#4363d8", "#911eb4", "#f58231", "#42d4f4", "#f032e6", "#9a6324"};
        const char* interCols[] = {"#d4004f", "#1a1aff", "#7a00b3"};
        int nl = 0, ni = 0;
        for (const IslandBusLine& b : w.busLines) {
            const bool inter = b.kind == "intercity";
            const std::string col = inter ? interCols[ni++ % 3] : localCols[nl++ % 8];
            o << "<g><title>" << b.name << "</title>";
            lines({b.path}, inter ? "class=\"hair\" stroke=\"" + col + "\" stroke-width=\"3.2\" stroke-dasharray=\"14 6\" opacity=\"0.9\""
                                  : "stroke=\"" + col + "\" stroke-width=\"6\" opacity=\"0.85\"");
            for (std::size_t k = 0; k < b.stops.size(); ++k)
                o << "<circle cx=\"" << b.stops[k].x << "\" cy=\"" << -b.stops[k].y << "\" r=\"" << (inter ? 28 : 12) << "\" fill=\"#fff\" stroke=\"" << col
                  << "\" stroke-width=\"" << (inter ? 8 : 5) << "\"><title>" << b.name << ": " << (k < b.stopNames.size() ? b.stopNames[k] : std::string()) << "</title></circle>";
            o << "</g>\n";
        }
    }
    o << "</g>\n";
    // SIGNS: a marker where each stands, pointing the way its traffic reads it; its legend on hover
    o << "<g id=\"signs\">\n";
    for (const IslandSign& sg : w.signs) {
        const std::string col = sg.kind == "do-not-enter" || sg.kind == "wrong-way" ? "#c1272d" : sg.kind == "route" ? "#ffffff" : "#00693f";
        const Vec2 f = sg.facing, r(f.y, -f.x);
        const Vec2 a = sg.at + f * 9.0, b = sg.at - f * 5.0 + r * 5.0, c = sg.at - f * 5.0 - r * 5.0;
        o << "<path fill=\"" << col << "\" stroke=\"#111\" stroke-width=\"1\" d=\"M" << a.x << ' ' << -a.y << "L" << b.x << ' ' << -b.y << "L" << c.x << ' ' << -c.y
          << "Z\"><title>" << sg.kind << " (" << sg.mount << "): " << sg.legend.dump() << "</title></path>\n";
    }
    o << "</g>\n";
    // LABELS
    o << "<g id=\"labels\">\n";
    for (std::size_t k = 0; k < w.sites.size(); ++k) {
        const IslandSite& st = w.sites[k];
        const double fs = st.kind == "city" ? 260.0 : 150.0;
        o << "<text x=\"" << st.at.x << "\" y=\"" << -st.at.y << "\" font-size=\"" << fs << "\" text-anchor=\"middle\" fill=\"#3a1026\">"
          << st.name << "</text>\n";
    }
    // a scale bar, 5 km (or 1 km close in), bottom left
    const double bar = half > 4000.0 ? 5000.0 : half > 1000.0 ? 1000.0 : 200.0;
    const double bx = x0 + W * 0.03, by = y0 + W * 0.97;
    o << "<path stroke=\"#111\" stroke-width=\"" << W * 0.004 << "\" d=\"M" << bx << ' ' << by << 'L' << bx + bar << ' ' << by << "\"/>\n";
    o << "<text x=\"" << bx << "\" y=\"" << by - W * 0.008 << "\" font-size=\"" << W * 0.014 << "\" fill=\"#111\">"
      << (bar >= 1000 ? std::to_string(static_cast<int>(bar / 1000)) + " km" : std::to_string(static_cast<int>(bar)) + " m") << "</text>\n";
    o << "</g>\n</svg>\n";
    std::ofstream f(svgPath);
    if (!f) return false;
    f << o.str();
    return static_cast<bool>(f);
}

}  // namespace engine

