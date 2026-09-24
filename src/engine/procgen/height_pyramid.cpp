#include "height_pyramid.h"
#include "../../job_system.h"
#include "../bundle/bundle.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <filesystem>
#include <mutex>

namespace engine {
namespace pyramid {

PyramidSpec PyramidSpec::covering(double minX, double minZ, double maxX, double maxZ,
                                  double cell0, double tolerance) {
    PyramidSpec s;
    s.originX = minX;
    s.originZ = minZ;
    s.cell0 = cell0;
    s.tolerance = tolerance;
    const double need = std::max(maxX - minX, maxZ - minZ);
    s.levels = 1;
    while (s.extent() < need && s.levels < 24) ++s.levels;
    return s;
}

double HeightTile::sampleLocal(double gx, double gz) const {
    gx = std::clamp(gx, 0.0, static_cast<double>(kTileCells));
    gz = std::clamp(gz, 0.0, static_cast<double>(kTileCells));
    int i = std::min(static_cast<int>(gx), kTileCells - 1);
    int j = std::min(static_cast<int>(gz), kTileCells - 1);
    const double u = gx - i, v = gz - j;
    const double ha = at(i, j), hd = at(i + 1, j + 1);
    if (u >= v) return ha + u * (at(i + 1, j) - ha) + v * (hd - at(i + 1, j));
    return ha + v * (at(i, j + 1) - ha) + u * (hd - at(i, j + 1));
}

const HeightTile* Pyramid::finestAt(double x, double z) const {
    const double lx = x - spec.originX, lz = z - spec.originZ;
    if (lx < 0 || lz < 0 || lx > spec.extent() || lz > spec.extent()) return nullptr;
    const HeightTile* best = nullptr;
    for (int level = spec.levels - 1; level >= 0; --level) {
        const double ts = spec.tileSize(level);
        TileKey k{level, std::min(static_cast<int>(lx / ts), (1 << (spec.levels - 1 - level)) - 1),
                  std::min(static_cast<int>(lz / ts), (1 << (spec.levels - 1 - level)) - 1)};
        const HeightTile* t = find(k);
        if (!t) break;   // the pyramid is a tree: no tile here means none below
        best = t;
        if (!t->hasChildren) break;
    }
    return best;
}

double Pyramid::height(double x, double z) const {
    const HeightTile* t = finestAt(x, z);
    if (!t) return 0.0;
    const double c = spec.cell(t->key.level);
    const double gx = (x - spec.originX) / c - static_cast<double>(t->key.tx) * kTileCells;
    const double gz = (z - spec.originZ) / c - static_cast<double>(t->key.tz) * kTileCells;
    return t->sampleLocal(gx, gz);
}

namespace {

// Raw float samples of one tile.
std::vector<double> sampleTile(const PyramidSpec& spec, const TileKey& k, const HeightFn& h) {
    std::vector<double> out(static_cast<std::size_t>(kTileSamples) * kTileSamples);
    const double c = spec.cell(k.level);
    const double x0 = spec.originX + static_cast<double>(k.tx) * kTileCells * c;
    const double z0 = spec.originZ + static_cast<double>(k.tz) * kTileCells * c;
    for (int j = 0; j < kTileSamples; ++j)
        for (int i = 0; i < kTileSamples; ++i)
            out[static_cast<std::size_t>(j) * kTileSamples + i] = h(x0 + i * c, z0 + j * c, c);
    return out;
}

HeightTile quantize(const TileKey& k, const std::vector<double>& s) {
    HeightTile t;
    t.key = k;
    double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
    for (double v : s) { lo = std::min(lo, v); hi = std::max(hi, v); }
    // 1 mm steps where the relief allows (65 m of it); coarser only for a steeper tile.
    const double step = std::max(0.001, (hi - lo) / 65535.0);
    t.minH = static_cast<float>(lo);
    t.maxH = static_cast<float>(hi);
    t.step = static_cast<float>(step);
    t.q.resize(s.size());
    for (std::size_t n = 0; n < s.size(); ++n)
        t.q[n] = static_cast<uint16_t>(std::clamp(std::lround((s[n] - lo) / step), 0l, 65535l));
    return t;
}

// The drawn surface of a parent sampled at a child's sample (i, j): child quadrant (cx, cz)
// of the parent covers parent cells [cx*64, cx*64+64).
double parentAtChild(const std::vector<double>& parent, int cx, int cz, int i, int j) {
    const double gx = cx * (kTileCells / 2) + i * 0.5, gz = cz * (kTileCells / 2) + j * 0.5;
    int pi = std::min(static_cast<int>(gx), kTileCells - 1), pj = std::min(static_cast<int>(gz), kTileCells - 1);
    const double u = gx - pi, v = gz - pj;
    auto P = [&](int a, int b) { return parent[static_cast<std::size_t>(b) * kTileSamples + a]; };
    const double ha = P(pi, pj), hd = P(pi + 1, pj + 1);
    if (u >= v) return ha + u * (P(pi + 1, pj) - ha) + v * (hd - P(pi + 1, pj));
    return ha + v * (P(pi, pj + 1) - ha) + u * (hd - P(pi, pj + 1));
}

}  // namespace

Pyramid buildPyramid(const PyramidSpec& spec, const HeightFn& h, JobSystem* jobs) {
    Pyramid out;
    out.spec = spec;
    struct Node { TileKey key; std::vector<double> samples; };
    std::vector<Node> frontier;
    {
        const TileKey top{spec.levels - 1, 0, 0};
        frontier.push_back({top, sampleTile(spec, top, h)});
    }
    std::mutex m;
    // Level by level: sample every candidate child of the frontier, keep the four children
    // of a tile when any of them differs from the tile's drawn surface by more than the
    // tolerance, record each tile's error, and carry the kept children down.
    while (!frontier.empty()) {
        const int level = frontier.front().key.level;
        std::vector<Node> next;
        std::vector<HeightTile> done(frontier.size());
        auto work = [&](std::size_t n) {
            const Node& node = frontier[n];
            float err = 0.0f;
            std::vector<Node> kids;
            if (level > 0) {
                for (int cz = 0; cz < 2; ++cz)
                    for (int cx = 0; cx < 2; ++cx) {
                        const TileKey ck{level - 1, node.key.tx * 2 + cx, node.key.tz * 2 + cz};
                        Node kid{ck, sampleTile(spec, ck, h)};
                        for (int j = 0; j < kTileSamples; ++j)
                            for (int i = 0; i < kTileSamples; ++i) {
                                const double d = std::fabs(kid.samples[static_cast<std::size_t>(j) * kTileSamples + i] -
                                                           parentAtChild(node.samples, cx, cz, i, j));
                                err = std::max(err, static_cast<float>(d));
                            }
                        kids.push_back(std::move(kid));
                    }
            }
            HeightTile t = quantize(node.key, node.samples);
            t.error = err;
            t.hasChildren = level > 0 && err > spec.tolerance;
            done[n] = std::move(t);
            if (done[n].hasChildren) {
                std::lock_guard<std::mutex> lock(m);
                for (Node& k : kids) next.push_back(std::move(k));
            }
        };
        if (jobs) jobs->parallelFor(0, frontier.size(), work);
        else for (std::size_t n = 0; n < frontier.size(); ++n) work(n);
        for (HeightTile& t : done) out.tiles.emplace(t.key, std::move(t));
        // A deterministic order for the next level whatever the threads did.
        std::sort(next.begin(), next.end(), [](const Node& a, const Node& b) {
            return a.key.tz != b.key.tz ? a.key.tz < b.key.tz : a.key.tx < b.key.tx;
        });
        frontier = std::move(next);
    }
    return out;
}

namespace {
struct TileHeader {
    uint32_t magic, version;
    int32_t level, tx, tz;
    float minH, maxH, step, error;
    uint32_t hasChildren, samples;
};
}  // namespace

std::vector<uint8_t> encodeTile(const HeightTile& t) {
    TileHeader hdr{kTileCodecMagic, kTileCodecVersion, t.key.level, t.key.tx, t.key.tz,
                   t.minH, t.maxH, t.step, t.error, t.hasChildren ? 1u : 0u,
                   static_cast<uint32_t>(t.q.size())};
    std::vector<uint8_t> out(sizeof(hdr) + t.q.size() * sizeof(uint16_t));
    std::memcpy(out.data(), &hdr, sizeof(hdr));
    if (!t.q.empty()) std::memcpy(out.data() + sizeof(hdr), t.q.data(), t.q.size() * sizeof(uint16_t));
    return out;
}

bool decodeTile(const uint8_t* data, std::size_t size, HeightTile& out) {
    TileHeader hdr{};
    if (!data || size < sizeof(hdr)) return false;
    std::memcpy(&hdr, data, sizeof(hdr));
    if (hdr.magic != kTileCodecMagic || hdr.version != kTileCodecVersion) return false;
    if (hdr.samples != static_cast<uint32_t>(kTileSamples * kTileSamples)) return false;
    if (size != sizeof(hdr) + hdr.samples * sizeof(uint16_t)) return false;
    out.key = {hdr.level, hdr.tx, hdr.tz};
    out.minH = hdr.minH;
    out.maxH = hdr.maxH;
    out.step = hdr.step;
    out.error = hdr.error;
    out.hasChildren = hdr.hasChildren != 0;
    out.q.resize(hdr.samples);
    std::memcpy(out.q.data(), data + sizeof(hdr), hdr.samples * sizeof(uint16_t));
    return true;
}

std::string tileSectionName(const TileKey& k, const std::string& prefix) {
    return prefix + "/L" + std::to_string(k.level) + "/" + std::to_string(k.tx) + "_" + std::to_string(k.tz);
}

bool writePyramidBundle(const Pyramid& p, uint64_t key, const std::string& path, std::string* err) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (fs::path(path).has_parent_path()) fs::create_directories(fs::path(path).parent_path(), ec);
    const std::string tmp = path + ".tmp";
    bundle::BundleWriter w;
    if (!w.openFile(tmp, err)) return false;
    nlohmann::json spec = {{"originX", p.spec.originX}, {"originZ", p.spec.originZ}, {"cell0", p.spec.cell0},
                           {"levels", p.spec.levels}, {"tolerance", p.spec.tolerance},
                           {"key", bundle::hex16(key)}, {"codec", kTileCodecVersion}, {"tiles", p.tiles.size()}};
    w.addJson("terrain/spec", spec);
    // Sections in a fixed order (level, then row, then column): the same bytes every time.
    std::vector<const HeightTile*> order;
    for (const auto& kv : p.tiles) order.push_back(&kv.second);
    std::sort(order.begin(), order.end(), [](const HeightTile* a, const HeightTile* b) {
        if (a->key.level != b->key.level) return a->key.level > b->key.level;
        return a->key.tz != b->key.tz ? a->key.tz < b->key.tz : a->key.tx < b->key.tx;
    });
    for (const HeightTile* t : order) w.add(tileSectionName(t->key), encodeTile(*t));
    nlohmann::json manifest = {{"kind", "rt-height-pyramid"}, {"key", bundle::hex16(key)}};
    if (!w.finish(manifest, err)) { fs::remove(tmp, ec); return false; }
    fs::rename(tmp, path, ec);
    if (ec) { if (err) *err = "cannot move " + tmp + " into place: " + ec.message(); fs::remove(tmp, ec); return false; }
    return true;
}

bool readPyramidBundle(const std::string& path, uint64_t key, Pyramid& out, std::string* err) {
    std::unique_ptr<bundle::Bundle> b = bundle::Bundle::open(path, err);
    if (!b) return false;
    const nlohmann::json spec = b->json("terrain/spec");
    if (spec.is_null() || spec.value("key", std::string()) != bundle::hex16(key) ||
        spec.value("codec", 0u) != kTileCodecVersion) {
        if (err) *err = "built for another key or codec";
        return false;
    }
    Pyramid p;
    p.spec.originX = spec.value("originX", 0.0);
    p.spec.originZ = spec.value("originZ", 0.0);
    p.spec.cell0 = spec.value("cell0", 1.0);
    p.spec.levels = spec.value("levels", 1);
    p.spec.tolerance = spec.value("tolerance", 0.0);
    for (const std::string& name : b->sections("terrain/L")) {
        const bundle::Bundle::View v = b->section(name);
        HeightTile t;
        if (!decodeTile(v.data, v.size, t)) { if (err) *err = "bad tile section " + name; return false; }
        p.tiles.emplace(t.key, std::move(t));
    }
    if (p.tiles.size() != spec.value("tiles", std::size_t(0))) { if (err) *err = "tile count mismatch"; return false; }
    out = std::move(p);
    return true;
}

}  // namespace pyramid
}  // namespace engine
