#include "tree_cache.h"

#include "../bundle/binary_stream.h"
#include "../bundle/bundle.h"
#include "../bundle/codecs.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace engine {

namespace {

const char kMagic[4] = {'T', 'R', 'E', 'E'};

void putTexture(bundle::BinWriter& w, const TextureData& t) {
    w.put<int32_t>(t.width); w.put<int32_t>(t.height); w.put<int32_t>(t.channels);
    w.putVec(t.pixels);
}
bool getTexture(bundle::BinReader& r, TextureData& t) {
    int32_t a = 0, b = 0, c = 0;
    if (!r.get(a) || !r.get(b) || !r.get(c) || !r.getVec(t.pixels)) return false;
    t.width = a; t.height = b; t.channels = c;
    return true;
}
void putMesh(bundle::BinWriter& w, const RenderMesh& m) {
    bundle::putPackedMesh(w, bundle::packMesh(m, "", Vec3(1, 1, 1), 0, 1.0f, 0.0));
    w.put<uint8_t>(m.tangentIsData ? 1 : 0);
}
bool getMesh(bundle::BinReader& r, RenderMesh& m) {
    bundle::PackedMesh p;
    uint8_t data = 0;
    if (!bundle::getPackedMesh(r, p) || !r.get(data)) return false;
    m = bundle::unpackMesh(p);
    m.tangentIsData = data != 0;
    return true;
}

// what the forest loader reads of a level: the meshes, the leaves' kind, the bark blend and palette, the limbs (the
// trunk's girth), the atlas, the bands
void putLevel(bundle::BinWriter& w, const RecipeTreeLevel& L) {
    putMesh(w, L.tree.branches);
    putMesh(w, L.tree.leaves);
    w.put<int32_t>(L.tree.foliage);
    w.put<uint8_t>(L.tree.barkBlend ? 1 : 0);
    w.put<int32_t>(L.tree.palette);
    std::vector<float> limbs;
    limbs.reserve(L.tree.limbs.size() * 9);
    for (const TreeMesh::Limb& l : L.tree.limbs)
        for (float f : {static_cast<float>(l.a.x), static_cast<float>(l.a.y), static_cast<float>(l.a.z), static_cast<float>(l.b.x),
                        static_cast<float>(l.b.y), static_cast<float>(l.b.z), l.ra, l.rb, l.dead ? 1.0f : 0.0f})
            limbs.push_back(f);
    w.putVec(limbs);
    putTexture(w, L.atlas.albedo);
    putTexture(w, L.atlas.normal);
    w.put<int32_t>(L.atlas.tiles);
    for (float f : {L.in0, L.in1, L.out0, L.out1}) w.put<float>(f);
    w.put<double>(L.minDistance); w.put<double>(L.maxDistance);
}
bool getLevel(bundle::BinReader& r, RecipeTreeLevel& L) {
    int32_t foliage = 0, palette = -1, tiles = 1;
    uint8_t blend = 0;
    std::vector<float> limbs;
    if (!getMesh(r, L.tree.branches) || !getMesh(r, L.tree.leaves) || !r.get(foliage) || !r.get(blend) || !r.get(palette) ||
        !r.getVec(limbs) || limbs.size() % 9 != 0 || !getTexture(r, L.atlas.albedo) || !getTexture(r, L.atlas.normal) ||
        !r.get(tiles) || !r.get(L.in0) || !r.get(L.in1) || !r.get(L.out0) || !r.get(L.out1) || !r.get(L.minDistance) ||
        !r.get(L.maxDistance))
        return false;
    L.tree.foliage = foliage;
    L.tree.barkBlend = blend != 0;
    L.tree.palette = palette;
    L.atlas.tiles = tiles;
    L.tree.limbs.clear();
    for (std::size_t i = 0; i < limbs.size(); i += 9) {
        TreeMesh::Limb l;
        l.a = Vec3(limbs[i], limbs[i + 1], limbs[i + 2]);
        l.b = Vec3(limbs[i + 3], limbs[i + 4], limbs[i + 5]);
        l.ra = limbs[i + 6]; l.rb = limbs[i + 7]; l.dead = limbs[i + 8] > 0.5f;
        L.tree.limbs.push_back(l);
    }
    return true;
}

bool cacheOff() {
    const char* e = std::getenv("RT_TREE_CACHE");
    return e && std::string(e) == "0";
}

std::string pathFor(uint64_t key) { return treeCacheDir() + "/" + bundle::hex16(key) + ".tree"; }

// KEPT TO A SIZE (a commit is a new key: an island's species are ~1 GB a build, and nothing else deletes them -- the
// level bundles once reached 517 GB that way): after a write, the least recently USED files (a read touches its file)
// go until the folder is under RT_TREE_CACHE_MAX_GB (default 4)
void pruneTreeCache(const std::string& keep) {
    namespace fs = std::filesystem;
    double maxGb = 4.0;
    if (const char* e = std::getenv("RT_TREE_CACHE_MAX_GB")) maxGb = std::max(0.1, std::atof(e));
    const std::uintmax_t cap = static_cast<std::uintmax_t>(maxGb * 1024.0 * 1024.0 * 1024.0);
    struct Entry { fs::path p; fs::file_time_type t; std::uintmax_t bytes; };
    std::vector<Entry> files;
    std::uintmax_t total = 0;
    std::error_code ec;
    for (const fs::directory_entry& d : fs::directory_iterator(treeCacheDir(), ec)) {
        if (!d.is_regular_file(ec) || d.path().extension() != ".tree") continue;
        const std::uintmax_t n = d.file_size(ec);
        files.push_back({d.path(), d.last_write_time(ec), n});
        total += n;
    }
    std::sort(files.begin(), files.end(), [](const Entry& a, const Entry& b) { return a.t < b.t; });
    for (const Entry& f : files) {
        if (total <= cap) break;
        if (f.p.string() == keep) continue;
        if (fs::remove(f.p, ec)) total -= f.bytes;
    }
}

}  // namespace

std::string treeCacheDir() {
    if (const char* e = std::getenv("RT_TREE_CACHE_DIR")) if (*e) return e;
    return "cache/trees";
}

uint64_t recipeSpeciesKey(const std::string& recipeFile, const std::string& recipe, const std::string& age,
                          const std::vector<uint32_t>& seeds, const RecipeLodBands& bands) {
    if (cacheOff()) return 0;
    uint64_t file = 0;
    if (!bundle::fnv1aFile(recipeFile, file)) return 0;
    uint64_t h = bundle::fnv1aStr("rt-tree-species");
    h = bundle::fnv1a(&kTreeCacheVersion, sizeof(kTreeCacheVersion), h);
    h = bundle::fnv1aStr(bundle::engineIdentity().version, h);
    h = bundle::fnv1a(&file, sizeof(file), h);
    h = bundle::fnv1aStr(recipe, h);
    h = bundle::fnv1aStr(age, h);
    for (uint32_t s : seeds) h = bundle::fnv1a(&s, sizeof(s), h);
    for (double b : {bands.hero0, bands.hero1, bands.mid0, bands.mid1, bands.light0, bands.light1}) h = bundle::fnv1a(&b, sizeof(b), h);
    return h ? h : 1;
}

bool loadRecipeSpecies(uint64_t key, std::vector<std::vector<RecipeTreeLevel>>& out) {
    if (key == 0) return false;
    std::ifstream f(pathFor(key), std::ios::binary);
    if (!f) return false;
    const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    bundle::BinReader r(bytes.data(), bytes.size());
    uint32_t version = 0, nVar = 0;
    if (!r.magic(kMagic, &version) || version != kTreeCacheVersion || !r.get(nVar)) return false;
    std::vector<std::vector<RecipeTreeLevel>> got(nVar);
    for (auto& levels : got) {
        uint32_t n = 0;
        if (!r.get(n)) return false;
        levels.resize(n);
        for (RecipeTreeLevel& L : levels)
            if (!getLevel(r, L)) return false;
    }
    out = std::move(got);
    std::error_code ec;   // (used: the pruning keeps it)
    std::filesystem::last_write_time(pathFor(key), std::filesystem::file_time_type::clock::now(), ec);
    return true;
}

bool saveRecipeSpecies(uint64_t key, const std::vector<std::vector<RecipeTreeLevel>>& variants) {
    if (key == 0) return false;
    bundle::BinWriter w;
    w.magic(kMagic, kTreeCacheVersion);
    w.put<uint32_t>(static_cast<uint32_t>(variants.size()));
    for (const auto& levels : variants) {
        w.put<uint32_t>(static_cast<uint32_t>(levels.size()));
        for (const RecipeTreeLevel& L : levels) putLevel(w, L);
    }
    std::error_code ec;
    std::filesystem::create_directories(treeCacheDir(), ec);
    // (written whole under a name of its own, then moved into place: two loads writing the same species never mix)
    const std::string path = pathFor(key),
                      tmp = path + ".tmp" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(reinterpret_cast<const char*>(w.bytes.data()), static_cast<std::streamsize>(w.bytes.size()));
        if (!f) return false;
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) return false;
    pruneTreeCache(path);
    return true;
}

}  // namespace engine
