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
    if (j.contains("species") && j["species"].is_array())
        for (const auto& s : j["species"]) {
            ForestSpecies fs;
            if (!realSpeciesFromName(s.value("kind", std::string("spruce")), fs.species)) continue;
            fs.variants = std::clamp(s.value("variants", fs.variants), 1, 16);
            fs.altLo = s.value("altLo", fs.altLo);
            fs.altHi = s.value("altHi", fs.altHi);
            fs.weight = s.value("weight", fs.weight);
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
    const int n = static_cast<int>(std::ceil(2.0 * half / p.spacing));
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    std::vector<std::vector<ForestTree>> parts(T);
    std::vector<std::thread> th;
    const Noise sn(p.seed * 7919u + 13u);
    for (int t = 0; t < T; ++t)
        th.emplace_back([&, t] {
            for (int j = t; j < n; j += T)
                for (int i = 0; i < n; ++i) {
                    const uint32_t h = static_cast<uint32_t>(i) * 73856093u ^ static_cast<uint32_t>(j) * 19349663u ^ p.seed * 83492791u;
                    const double x = -half + (i + 0.1 + 0.8 * unit(h)) * p.spacing, z = -half + (j + 0.1 + 0.8 * unit(h ^ 0xA5A5u)) * p.spacing;
                    // the cheap part first: most of the island is rejected before a ground sample
                    const double roll = unit(h ^ 0x51ED27u);
                    const double pre = forestPrefilter(p, maps, x, z);
                    if (roll >= pre) continue;
                    const double y = ground(x, z);
                    const double e = 1.5;
                    const double gx = (ground(x + e, z) - ground(x - e, z)) / (2 * e), gz = (ground(x, z + e) - ground(x, z - e)) / (2 * e);
                    const double slope = std::atan(std::sqrt(gx * gx + gz * gz)) * 180.0 / 3.14159265358979;
                    const double d = pre * forestGrounded(p, sea, cover, x, z, y, slope);
                    if (roll >= d) continue;
                    if (exclude && exclude(x, z)) continue;
                    // the species: its altitude band (soft), its weight, and a patchy field of its own
                    const double alt = y - (sea > -1e29 ? sea : 0.0);
                    int best = -1;
                    double bw = -1e9;
                    for (std::size_t s = 0; s < p.species.size(); ++s) {
                        const ForestSpecies& fs = p.species[s];
                        const double band = smooth(fs.altLo - 60.0, fs.altLo + 20.0, alt) * (1.0 - smooth(fs.altHi - 20.0, fs.altHi + 60.0, alt));
                        if (band <= 0.0) continue;
                        const double patch = sn.fbm2(x * 0.004 + 17.0 * s, z * 0.004 - 11.0 * s, 3);
                        const double w = std::log(band * fs.weight + 1e-6) + 1.6 * patch + 0.35 * unit(h ^ (0x1234u + s));
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
    return out;
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
