#include "forest.h"

#include "ground_cover.h"
#include "noise.h"
#include "terrain_maps.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

namespace engine {

namespace {
double smooth(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
uint32_t mix32(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
double unit(uint32_t h) { return (mix32(h) & 0xFFFFFFu) / 16777216.0; }
}  // namespace

ForestParams forestFromJson(const nlohmann::json& j) {
    ForestParams p;
    p.spacing = j.value("spacing", p.spacing);
    p.coverage = j.value("coverage", p.coverage);
    p.standScaleM = j.value("standScale", p.standScaleM);
    p.treelineM = j.value("treeline", p.treelineM);
    p.shoreClearM = j.value("shoreClear", p.shoreClearM);
    p.maxSlopeDeg = j.value("maxSlopeDeg", p.maxSlopeDeg);
    p.cellM = j.value("cell", p.cellM);
    p.nearM = j.value("near", p.nearM);
    p.nearFadeM = j.value("nearFade", p.nearFadeM);
    p.farM = j.value("far", p.farM);
    p.seed = j.value("seed", p.seed);
    p.understorySpacing = j.value("understorySpacing", p.understorySpacing);
    p.understoryDensity = j.value("understoryDensity", p.understoryDensity);
    if (j.contains("rocks") && j["rocks"].is_array())
        for (const auto& r : j["rocks"]) {
            RockLayer L;
            L.family = r.value("family", L.family);
            L.stone = r.value("stone", L.stone);
            L.size = r.value("size", L.size);
            L.variants = std::clamp(r.value("variants", L.variants), 1, 16);
            const std::string g = r.value("ground", std::string("scree"));
            L.ground = g == "outcrop" ? 1 : g == "field" ? 2 : g == "forest" ? 3 : g == "beach" ? 4 : g == "shore" ? 5 : 0;
            L.spacing = r.value("spacing", L.spacing);
            L.density = r.value("density", L.density);
            L.moss = r.value("moss", L.moss);
            L.drawM = r.value("draw", L.drawM);
            p.rocks.push_back(L);
        }
    if (j.contains("species") && j["species"].is_array())
        for (const auto& s : j["species"]) {
            ForestSpecies fs;
            if (!realSpeciesFromName(s.value("kind", std::string("spruce")), fs.species)) continue;
            fs.variants = std::clamp(s.value("variants", fs.variants), 1, 16);
            fs.altLo = s.value("altLo", fs.altLo);
            fs.altHi = s.value("altHi", fs.altHi);
            fs.weight = s.value("weight", fs.weight);
            fs.wet = s.value("wet", fs.wet);
            fs.understory = s.value("understory", fs.understory);
            p.species.push_back(fs);
        }
    return p;
}

namespace {
const Noise& forestNoise(uint32_t seed) {
    static thread_local std::unique_ptr<Noise> nz;
    static thread_local uint32_t nzSeed = 0;
    if (!nz || nzSeed != seed) { nz = std::make_unique<Noise>(seed * 2654435761u + 5u); nzSeed = seed; }
    return *nz;
}
// the part of the density that needs no ground sample: stands and clearings, soil, scree
double forestPrefilter(const ForestParams& p, const TerrainMaps* maps, double x, double z) {
    const Noise& nz = forestNoise(p.seed);
    const double stand = 0.5 + 0.5 * nz.fbm2(x / p.standScaleM + 7.7, z / p.standScaleM - 2.9, 4) + 0.08 * nz.noise2(x * 0.03, z * 0.03);
    const double t0 = 1.0 - p.coverage - 0.1;
    double d = smooth(t0, t0 + 0.2, stand);
    if (d > 0.0 && maps) {
        const TerrainMapSample m = maps->at(x, z);
        d *= smooth(0.12, 0.45, m.soil) * (1.0 - smooth(0.2, 0.55, m.scree));
    }
    return d;
}
// ...and the part that does: the shore, the treeline, the slope, what the ground is
double forestGrounded(const ForestParams& p, double sea, const GroundCover* cover, double x, double z, double y, double slopeDeg) {
    const Noise& nz = forestNoise(p.seed);
    const double alt = y - (sea > -1e29 ? sea : 0.0);
    double d = smooth(p.shoreClearM, p.shoreClearM + 8.0, alt);
    if (d <= 0.0) return 0.0;
    const double tl = p.treelineM + 90.0 * nz.fbm2(x * 0.0017 + 3.3, z * 0.0017 - 1.1, 3);
    d *= 1.0 - smooth(tl - 70.0, tl, alt);
    d *= 1.0 - smooth(p.maxSlopeDeg - 8.0, p.maxSlopeDeg, slopeDeg);
    if (d > 0.0 && cover) {
        const Cover c = cover->at(x, z, y, std::cos(slopeDeg * 3.14159265358979 / 180.0));
        d *= std::max(0.0, 1.0 - 1.6 * (c.rock + c.snow + c.sand));
    }
    return std::clamp(d, 0.0, 1.0);
}
}  // namespace

double forestDensity(const ForestParams& p, double sea, const GroundCover* cover, const TerrainMaps* maps,
                     double x, double z, double y, double slopeDeg) {
    const double a = forestPrefilter(p, maps, x, z);
    return a <= 0.0 ? 0.0 : a * forestGrounded(p, sea, cover, x, z, y, slopeDeg);
}

double forestCanopy(const ForestParams& p, double sea, const TerrainMaps* maps, double x, double z, double y,
                    double slopeDeg) {
    const double a = forestPrefilter(p, maps, x, z);
    return a <= 0.0 ? 0.0 : a * forestGrounded(p, sea, nullptr, x, z, y, slopeDeg);
}

std::vector<ForestTree> placeForest(const ForestParams& p, double half, double sea, const GroundCover* cover,
                                    const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                    const std::function<bool(double, double)>& exclude, int vps) {
    std::vector<ForestTree> out;
    if (p.species.empty()) return out;
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    const Noise sn(p.seed * 7919u + 13u);
    bool anyUnder = false;
    for (const ForestSpecies& fs : p.species) anyUnder = anyUnder || fs.understory;
    // TWO PASSES over jittered grids: the canopy at the stand's spacing, then the understory (bushes
    // under thin canopy and thickest along the edges), each choosing among its own species
    for (int pass = 0; pass < (anyUnder ? 2 : 1); ++pass) {
        const bool under = pass == 1;
        const double spacing = under ? p.understorySpacing : p.spacing;
        const uint32_t salt = under ? 0x5EED5u : 0u;
        const int n = static_cast<int>(std::ceil(2.0 * half / spacing));
        std::vector<std::vector<ForestTree>> parts(T);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t)
            th.emplace_back([&, t] {
                for (int j = t; j < n; j += T)
                    for (int i = 0; i < n; ++i) {
                        const uint32_t h = (static_cast<uint32_t>(i) * 73856093u ^ static_cast<uint32_t>(j) * 19349663u ^ p.seed * 83492791u) + salt;
                        const double x = -half + (i + 0.1 + 0.8 * unit(h)) * spacing, z = -half + (j + 0.1 + 0.8 * unit(h ^ 0xA5A5u)) * spacing;
                        // the cheap part first: most of the island is rejected before a ground sample
                        const double roll = unit(h ^ 0x51ED27u);
                        const double pre = forestPrefilter(p, maps, x, z);
                        // the understory takes a share of the canopy's density, most at a half-covered edge
                        auto keep = [&](double d) { return under ? p.understoryDensity * (0.35 + 1.3 * 4.0 * d * (1.0 - d)) * (d > 0.05 ? 1.0 : 0.0) : d; };
                        if (roll >= keep(pre) && (!under || pre <= 0.0)) continue;
                        const double y = ground(x, z);
                        const double e = 1.5;
                        const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e), gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
                        const double slope = std::atan(std::sqrt(gx * gx + gz * gz)) * 180.0 / 3.14159265358979;
                        const double d = pre * forestGrounded(p, sea, cover, x, z, y, slope);
                        if (roll >= keep(d)) continue;
                        if (exclude && exclude(x, z)) continue;
                        // the species: its altitude band (soft), its weight, its taste for wet ground, and a
                        // patchy field of its own (stands, not a salad)
                        const double alt = y - (sea > -1e29 ? sea : 0.0);
                        const double wet = maps ? maps->at(x, z).wet : 0.0;
                        int best = -1;
                        double bw = -1e9;
                        for (std::size_t s = 0; s < p.species.size(); ++s) {
                            const ForestSpecies& fs = p.species[s];
                            if (fs.understory != under) continue;
                            const double band = smooth(fs.altLo - 60.0, fs.altLo + 20.0, alt) * (1.0 - smooth(fs.altHi - 20.0, fs.altHi + 60.0, alt));
                            if (band <= 0.0) continue;
                            const double wetK = 1.0 + fs.wet * ((0.03 + smooth(0.45, 0.8, wet)) * 4.0 - 1.0);
                            const double patch = sn.fbm2(x * 0.004 + 17.0 * s, z * 0.004 - 11.0 * s, 3);
                            const double w = std::log(band * fs.weight * wetK + 1e-6) + 1.6 * patch + 0.35 * unit(h ^ (0x1234u + static_cast<uint32_t>(s)));
                            if (w > bw) { bw = w; best = static_cast<int>(s); }
                        }
                        if (best < 0) continue;
                        const ForestSpecies& fs = p.species[best];
                        ForestTree tr;
                        tr.pos = Vec3(x, y, z);
                        tr.yaw = static_cast<float>(unit(h ^ 0x77u) * 6.2831853);
                        // stunted toward the treeline and at a stand's thin edge
                        const double tl = p.treelineM;
                        const double stunt = 1.0 - 0.5 * smooth(tl - 220.0, tl, alt);
                        tr.scale = static_cast<float>((0.72 + 0.5 * unit(h ^ 0x99u)) * stunt * (0.8 + 0.2 * d));
                        tr.variant = static_cast<uint16_t>(best * vps + static_cast<int>(unit(h ^ 0x3131u) * fs.variants) % fs.variants);
                        parts[t].push_back(tr);
                    }
            });
        for (auto& x : th) x.join();
        for (auto& v : parts) out.insert(out.end(), v.begin(), v.end());
    }
    return out;
}

std::vector<PlacedRock> placeRocks(const ForestParams& p, double half, double sea, const GroundCover* cover,
                                   const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                   const std::function<bool(double, double)>& exclude,
                                   const std::function<double(double, double, double)>& shore,
                                   const std::function<bool(double, double)>& excludeShore) {
    std::vector<PlacedRock> out;
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    for (std::size_t li = 0; li < p.rocks.size(); ++li) {
        const RockLayer& L = p.rocks[li];
        const int n = static_cast<int>(std::ceil(2.0 * half / L.spacing));
        std::vector<std::vector<PlacedRock>> parts(T);
        std::vector<std::thread> th;
        for (int t = 0; t < T; ++t)
            th.emplace_back([&, t] {
                for (int j = t; j < n; j += T)
                    for (int i = 0; i < n; ++i) {
                        const uint32_t h = (static_cast<uint32_t>(i) * 73856093u ^ static_cast<uint32_t>(j) * 19349663u ^ p.seed * 83492791u) + 0xB0u * static_cast<uint32_t>(li + 1);
                        const double roll = unit(h ^ 0x51ED27u);
                        if (roll >= L.density) continue;
                        const double x = -half + (i + 0.1 + 0.8 * unit(h)) * L.spacing, z = -half + (j + 0.1 + 0.8 * unit(h ^ 0xA5A5u)) * L.spacing;
                        // the maps first (no ground sample): most of the island is the wrong ground
                        TerrainMapSample m;
                        if (maps) m = maps->at(x, z);
                        const double pre = forestPrefilter(p, maps, x, z);
                        bool ok = false;
                        switch (L.ground) {
                            case 0: ok = maps && m.scree > 0.25 && roll < L.density * std::min(1.0, m.scree * 1.6); break;
                            case 1: ok = maps && m.convex > 0.1 && m.soil < 0.35; break;
                            case 2: ok = maps && m.soil > 0.5 && pre < 0.15; break;
                            case 3: ok = pre > 0.5; break;
                            case 5: ok = static_cast<bool>(shore); break;
                            default: ok = true; break;
                        }
                        if (!ok) continue;
                        const double y = ground(x, z);
                        const double alt = y - (sea > -1e29 ? sea : 0.0);
                        if (alt < 0.3) continue;
                        const double e = 1.5;
                        const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e), gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
                        const double slope = std::atan(std::sqrt(gx * gx + gz * gz)) * 57.2957795;
                        if (cover) {
                            const Cover c = cover->at(x, z, y, std::cos(slope / 57.2957795));
                            if (L.ground == 1 && c.rock < 0.4) continue;
                            if (L.ground == 4 && c.sand < 0.5) continue;
                            if (L.ground != 1 && L.ground != 0 && c.snow > 0.3) continue;
                        }
                        if (L.ground == 3 && forestGrounded(p, sea, cover, x, z, y, slope) < 0.3) continue;
                        if (L.ground == 5 && roll >= L.density * shore(x, z, y)) continue;
                        if (slope > 55.0) continue;
                        const auto& ex = L.ground == 5 && excludeShore ? excludeShore : exclude;
                        if (ex && ex(x, z)) continue;
                        PlacedRock r;
                        r.pos = Vec3(x, y, z);
                        r.yaw = static_cast<float>(unit(h ^ 0x77u) * 6.2831853);
                        r.scale = static_cast<float>(0.55 + 0.9 * std::pow(unit(h ^ 0x99u), 2.0));   // many small, a few big
                        r.tiltDir = static_cast<float>(unit(h ^ 0x55u) * 6.2831853);
                        r.tilt = static_cast<float>(unit(h ^ 0x66u) * (L.ground == 1 ? 0.1 : 0.25) + std::min(0.35, slope / 57.2957795 * 0.5));
                        r.layer = static_cast<uint16_t>(li);
                        r.variant = static_cast<uint16_t>(static_cast<int>(unit(h ^ 0x3131u) * L.variants) % L.variants);
                        parts[t].push_back(r);
                    }
            });
        for (auto& x : th) x.join();
        for (auto& v : parts) out.insert(out.end(), v.begin(), v.end());
    }
    return out;
}

Seat seatOnGround(const std::function<double(double, double)>& ground, double x, double z, double foot, double bed) {
    Seat st;
    const double e = std::max(0.5, 0.6 * foot);
    const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e);
    const double gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
    st.normal = normalize(Vec3(-gx, 1.0, -gz));
    double y0 = ground(x, z);
    for (int k = 0; k < 8; ++k) {
        const double a = k * 0.785398, dx = foot * std::cos(a), dz = foot * std::sin(a);
        const double rise = -(st.normal.x * dx + st.normal.z * dz) / std::max(0.2, st.normal.y);   // the plane above its centre
        y0 = std::min(y0, ground(x + dx, z + dz) - rise);
    }
    st.baseY = y0 - bed;
    return st;
}

void appendImpostor(RenderMesh& mesh, const ForestTree& t, const ImpostorSlot& s, double colour) {
    const double sc = t.scale;
    const double hw = s.halfW * sc, H = s.height * sc;
    const double cy = std::cos(t.yaw), sy = std::sin(t.yaw);
    const Vec3 base = t.pos - Vec3(0, 0.15 * sc, 0);
    const Vec3 col(colour, colour, colour);
    auto quad = [&](const Vec3 p[4], const Vec3 n[4], const double uv[4][2]) {
        const uint32_t b = static_cast<uint32_t>(mesh.vertices.size());
        for (int k = 0; k < 4; ++k) {
            Vertex v(p[k], n[k], Vec3(1, 0, 0), static_cast<float>(uv[k][0]), static_cast<float>(uv[k][1]));
            v.color = col;
            mesh.vertices.push_back(v);
        }
        for (uint32_t i : {0u, 1u, 2u, 0u, 2u, 3u}) mesh.indices.push_back(b + i);
    };
    // the side picture's rows run top (v0) to the foot (v1): image row 0 is the tree's top
    for (int k = 0; k < 2; ++k) {
        const double a = k * 1.5707963;
        const Vec3 side(cy * std::cos(a) - sy * std::sin(a), 0.0, sy * std::cos(a) + cy * std::sin(a));
        const Vec3 p[4] = {base - side * hw, base + side * hw, base + side * hw + Vec3(0, H, 0), base - side * hw + Vec3(0, H, 0)};
        // bent normals: out to each side, up toward the crown's top -- the card lights like a volume
        const Vec3 n[4] = {normalize(side * -0.7 + Vec3(0, 0.45, 0)), normalize(side * 0.7 + Vec3(0, 0.45, 0)),
                           normalize(side * 0.55 + Vec3(0, 0.85, 0)), normalize(side * -0.55 + Vec3(0, 0.85, 0))};
        const double uv[4][2] = {{s.u0, s.v1}, {s.u1, s.v1}, {s.u1, s.v0}, {s.u0, s.v0}};
        quad(p, n, uv);
    }
    // the top card, at the crown's broadest, facing up (x -> u, z -> v, turned by the yaw)
    const double yTop = (s.crownBase + 0.55 * (s.height - s.crownBase)) * sc;
    const Vec3 ax(cy, 0, sy), az(-sy, 0, cy);
    const Vec3 c = base + Vec3(0, yTop, 0);
    const Vec3 p[4] = {c - ax * hw - az * hw, c + ax * hw - az * hw, c + ax * hw + az * hw, c - ax * hw + az * hw};
    const Vec3 n[4] = {normalize(ax * -0.35 - az * 0.35 + Vec3(0, 1, 0)), normalize(ax * 0.35 - az * 0.35 + Vec3(0, 1, 0)),
                       normalize(ax * 0.35 + az * 0.35 + Vec3(0, 1, 0)), normalize(ax * -0.35 + az * 0.35 + Vec3(0, 1, 0))};
    const double uv[4][2] = {{s.tu0, s.tv0}, {s.tu1, s.tv0}, {s.tu1, s.tv1}, {s.tu0, s.tv1}};
    quad(p, n, uv);
}

}  // namespace engine
