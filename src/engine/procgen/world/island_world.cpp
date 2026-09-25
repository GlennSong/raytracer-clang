#include "island_world.h"

#include "../noise.h"
#include "../terrain.h"
#include "../city/terrain_route.h"
#include "../../level_params.h"   // readTerrainParams: the island's terrain block, as a level reads it

#include <tinygltf/stb_image_write.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

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
}  // namespace

double IslandWorld::heightAt(double x, double z) const {
    const double fx = std::clamp((x + half) / cell, 0.0, n - 1.001), fz = std::clamp((z + half) / cell, 0.0, n - 1.001);
    const int i = static_cast<int>(fx), j = static_cast<int>(fz);
    const double u = fx - i, v = fz - j;
    auto H = [&](int a, int b) { return static_cast<double>(height[static_cast<std::size_t>(b) * n + a]); };
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

    // 4. ROADS over the ground (terrain_route.h)
    const HeightField ground = [&w](double x, double z) { return w.heightAt(x, z); };
    auto route = [&](const std::string& kind, int a, int b, const Vec2& from, const Vec2& to, TerrainRouteParams rp) {
        rp.blocked = [&w](double x, double z) { return w.heightAt(x, z) < 0.8; };   // never over the sea
        const TerrainRoute tr = routeOnTerrain(ground, from, to, rp);
        IslandRoad rd;
        rd.kind = kind;
        rd.from = a;
        rd.to = b;
        rd.points = tr.points;
        rd.length = tr.length;
        rd.climb = tr.climb;
        rd.worstGrade = tr.worstGrade;
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
    TerrainRouteParams fw;
    fw.cell = 30.0; fw.maxGrade = 0.05; fw.hardGrade = 0.30; fw.gradeWeight = 800.0; fw.turnWeight = 30.0; fw.maxTurnDeg = 25.0; fw.margin = 1400.0;
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
            ps.cell = 24.0; ps.maxGrade = 0.07; ps.hardGrade = 0.60; ps.gradeWeight = 500.0; ps.turnWeight = 12.0; ps.margin = 3000.0;
            const std::size_t before = w.roads.size();
            route("pass", c0, c1, w.sites[static_cast<std::size_t>(c0)].at, saddle, ps);
            route("pass", c0, c1, saddle, w.sites[static_cast<std::size_t>(c1)].at, ps);
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
        TerrainRouteParams mr;
        mr.cell = 12.0; mr.maxGrade = 0.08; mr.hardGrade = 0.45; mr.margin = 900.0;
        route("mountain", -1, k, foot, top, mr);
    }

    // 5. THE REPORT
    json sites = json::array(), roads = json::array();
    for (const IslandSite& s : w.sites)
        sites.push_back({{"kind", s.kind}, {"at", {std::round(s.at.x), std::round(s.at.y)}}, {"radius", std::round(s.radius)},
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

bool writeIslandMap(const IslandWorld& w, const std::string& pngPath, int px) {
    const double mpp = 2.0 * w.half / px;   // metres a pixel
    std::vector<float> img(static_cast<std::size_t>(px) * px * 3, 0.0f);
    auto put = [&](int x, int y, float r, float g, float b, float a = 1.0f) {
        if (x < 0 || y < 0 || x >= px || y >= px) return;
        float* p = &img[(static_cast<std::size_t>(y) * px + x) * 3];
        p[0] += (r - p[0]) * a; p[1] += (g - p[1]) * a; p[2] += (b - p[2]) * a;
    };
    auto toPx = [&](const Vec2& q, double& x, double& y) { x = (q.x + w.half) / mpp; y = (w.half - q.y) / mpp; };   // north (+z) up
    auto mix3 = [](const float* a, const float* b, float t, float* o) { for (int k = 0; k < 3; ++k) o[k] = a[k] + (b[k] - a[k]) * t; };
    // relief and water
    const float sand[3] = {0.86f, 0.80f, 0.60f}, low[3] = {0.40f, 0.58f, 0.28f}, mid[3] = {0.46f, 0.52f, 0.30f},
                high[3] = {0.55f, 0.48f, 0.36f}, rock[3] = {0.60f, 0.58f, 0.55f}, snow[3] = {0.93f, 0.94f, 0.96f},
                shallow[3] = {0.55f, 0.80f, 0.85f}, deep[3] = {0.10f, 0.28f, 0.48f};
    const double lx = -0.6, ly = 0.75, lz = 0.9, ll = std::sqrt(lx * lx + ly * ly + lz * lz);
    for (int y = 0; y < px; ++y)
        for (int x = 0; x < px; ++x) {
            const double wx = -w.half + (x + 0.5) * mpp, wz = w.half - (y + 0.5) * mpp;
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
    for (const IslandRoad& rd : w.roads) {
        if (rd.points.empty()) continue;
        if (rd.kind == "freeway") { stroke(rd.points, 6.0, 0.15f, 0.10f, 0.05f); stroke(rd.points, 4.0, 0.98f, 0.60f, 0.10f); }
        else if (rd.kind == "pass") { stroke(rd.points, 4.5, 0.15f, 0.10f, 0.05f); stroke(rd.points, 3.0, 0.99f, 0.90f, 0.25f); }
        else { stroke(rd.points, 3.2, 0.15f, 0.10f, 0.05f); stroke(rd.points, 1.8, 1.0f, 1.0f, 1.0f); }
    }
    for (const IslandSite& s : w.sites) {
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
        const int x0 = 30, y0 = px - 34, len = static_cast<int>(5000.0 / mpp);
        for (int yy = y0 - 3; yy <= y0 + 3; ++yy)
            for (int xx = x0 - 3; xx <= x0 + len + 3; ++xx) put(xx, yy, 0.05f, 0.05f, 0.05f);
        for (int yy = y0 - 1; yy <= y0 + 1; ++yy)
            for (int xx = x0; xx <= x0 + len; ++xx) put(xx, yy, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f, ((xx - x0) / std::max(1, len / 5)) % 2 ? 0.95f : 0.1f);
    }
    std::vector<unsigned char> out(img.size());
    for (std::size_t k = 0; k < img.size(); ++k) out[k] = static_cast<unsigned char>(std::clamp(img[k], 0.0f, 1.0f) * 255.0f + 0.5f);
    return stbi_write_png(pngPath.c_str(), px, px, 3, out.data(), px * 3) != 0;
}

}  // namespace engine
