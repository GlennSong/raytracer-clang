#include "terrain_lod.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

std::vector<float> defaultLodRanges(int numLods, float leafRange) {
    numLods = std::max(1, numLods);
    std::vector<float> ranges;
    ranges.reserve(numLods);
    float r = std::max(1e-3f, leafRange);
    for (int i = 0; i < numLods; i++) {
        ranges.push_back(r);
        r *= 2.0f;            // each coarser level reaches twice as far
    }
    return ranges;
}

std::vector<float> lodRangesForWorld(float worldHalf, int numLods,
                                     float rangeFactor) {
    numLods = std::max(1, numLods);
    float leafNodeSize =
        (worldHalf * 2.0f) / static_cast<float>(1 << (numLods - 1));
    float leafRange = leafNodeSize * std::max(2.0f, rangeFactor);
    return defaultLodRanges(numLods, leafRange);
}

MorphRange lodMorphRange(int level, const std::vector<float>& ranges) {
    MorphRange m;
    if (ranges.empty()) return m;
    int L = std::clamp(level, 0, static_cast<int>(ranges.size()) - 1);
    m.end = ranges[L];
    m.start = (L > 0) ? ranges[L - 1] : 0.0f;
    if (m.end <= m.start) m.end = m.start + 1e-3f;   // avoid a zero-width band
    return m;
}

float distanceToBoxXZ(float px, float pz, float minX, float minZ, float size) {
    float maxX = minX + size, maxZ = minZ + size;
    float dx = std::max({minX - px, 0.0f, px - maxX});
    float dz = std::max({minZ - pz, 0.0f, pz - maxZ});
    return std::sqrt(dx * dx + dz * dz);
}

namespace {
// Recursive restricted-quadtree descent. A node at `level` covers [minX,minX+size]
// x [minZ,minZ+size]. It is emitted whole if the camera is farther than the finer
// level's range (so finer detail isn't needed here); otherwise it splits into four
// children one level finer. Using the distance to the node's NEAREST point makes a
// shared edge carry a single distance, which forces adjacent emitted nodes to
// differ by <=1 level.
void selectRec(float minX, float minZ, float size, int level,
               const std::vector<float>& ranges, float camX, float camZ,
               std::vector<LodNode>& out) {
    if (level <= 0) {
        out.push_back(LodNode{minX, minZ, size, 0});
        return;
    }
    int finer = level - 1;
    float d = distanceToBoxXZ(camX, camZ, minX, minZ, size);
    if (d > ranges[finer]) {
        out.push_back(LodNode{minX, minZ, size, level});
        return;
    }
    float h = size * 0.5f;
    selectRec(minX,     minZ,     h, finer, ranges, camX, camZ, out);
    selectRec(minX + h, minZ,     h, finer, ranges, camX, camZ, out);
    selectRec(minX,     minZ + h, h, finer, ranges, camX, camZ, out);
    selectRec(minX + h, minZ + h, h, finer, ranges, camX, camZ, out);
}
}  // namespace

std::vector<LodNode> selectLodNodes(float worldHalf, int numLods,
                                    const std::vector<float>& ranges,
                                    float camX, float camZ) {
    numLods = std::max(1, numLods);
    std::vector<LodNode> out;
    if (numLods == 1 || ranges.empty()) {
        out.push_back(LodNode{-worldHalf, -worldHalf, worldHalf * 2.0f, 0});
        return out;
    }
    selectRec(-worldHalf, -worldHalf, worldHalf * 2.0f, numLods - 1, ranges,
              camX, camZ, out);
    return out;
}

namespace {
// selectRec with the descent gated on child readiness: where the ideal cut
// would split, the parent is emitted instead until all four children report
// renderable. Coarser than ideal is the only divergence — coverage stays exact.
void selectGatedRec(float minX, float minZ, float size, int level,
                    const std::vector<float>& ranges, float camX, float camZ,
                    const std::function<bool(const LodNode&)>& ensure,
                    std::vector<LodNode>& out) {
    if (level <= 0) {
        out.push_back(LodNode{minX, minZ, size, 0});
        return;
    }
    int finer = level - 1;
    float d = distanceToBoxXZ(camX, camZ, minX, minZ, size);
    if (d > ranges[finer]) {
        out.push_back(LodNode{minX, minZ, size, level});
        return;
    }
    float h = size * 0.5f;
    const LodNode kids[4] = {LodNode{minX,     minZ,     h, finer},
                             LodNode{minX + h, minZ,     h, finer},
                             LodNode{minX,     minZ + h, h, finer},
                             LodNode{minX + h, minZ + h, h, finer}};
    bool allReady = true;
    for (const LodNode& k : kids)   // probe all four: request every absent sibling
        if (!ensure(k)) allReady = false;
    if (!allReady) {
        out.push_back(LodNode{minX, minZ, size, level});
        return;
    }
    for (const LodNode& k : kids)
        selectGatedRec(k.minX, k.minZ, k.size, finer, ranges, camX, camZ,
                       ensure, out);
}
}  // namespace

std::vector<LodNode> selectLodNodesGated(
    float worldHalf, int numLods, const std::vector<float>& ranges,
    float camX, float camZ, const std::function<bool(const LodNode&)>& ensure) {
    numLods = std::max(1, numLods);
    std::vector<LodNode> out;
    LodNode root{-worldHalf, -worldHalf, worldHalf * 2.0f, numLods - 1};
    if (!ensure(root)) return out;   // nothing renderable yet
    if (numLods == 1 || ranges.empty()) {
        out.push_back(root);
        return out;
    }
    selectGatedRec(root.minX, root.minZ, root.size, root.level, ranges,
                   camX, camZ, ensure, out);
    return out;
}

bool LodMeshStream::begin(int64_t key, uint32_t revision) {
    return inFlight_.emplace(key, revision).second;
}

void LodMeshStream::complete(Result r) {
    std::lock_guard<std::mutex> lock(doneMutex_);
    done_.push_back(std::move(r));
}

std::vector<LodMeshStream::Result> LodMeshStream::drain(uint32_t currentRevision) {
    std::vector<Result> finished;
    {
        std::lock_guard<std::mutex> lock(doneMutex_);
        finished.swap(done_);
    }
    std::vector<Result> ready;
    ready.reserve(finished.size());
    for (Result& r : finished) {
        // Clear the in-flight entry only when it belongs to THIS job: after an
        // invalidate + re-request, the entry is the new revision's and must
        // survive the old job's arrival.
        auto it = inFlight_.find(r.key);
        if (it != inFlight_.end() && it->second == r.revision) inFlight_.erase(it);
        if (r.revision == currentRevision) ready.push_back(std::move(r));
    }
    return ready;
}

void LodMeshStream::invalidate() { inFlight_.clear(); }

LodNodeMesh generateLodNodeMesh(const TerrainParams& params, const Noise& noise,
                                const LodNode& node, int gridRes, double normalEps) {
    // Even grid so the next-coarser level samples align on even indices.
    int res = std::max(2, gridRes);
    if (res % 2 != 0) res += 1;
    const int n = res + 1;                              // vertices per side
    const float step = node.size / static_cast<float>(res);
    // A world-consistent eps (passed in) makes normals identical at shared edges
    // across LOD levels (no shading seam); 0 falls back to the per-tile step.
    const double eps = (normalEps > 0.0) ? normalEps : static_cast<double>(step);

    LodNodeMesh out;
    RenderMesh& mesh = out.mesh;
    mesh.vertices.reserve(static_cast<size_t>(n) * n);

    // First pass: positions/normals/colors + the raw height grid (for morph targets).
    // The cut/fill footprints are sampled DILATED by ~half this node's cell: a
    // road corridor narrower than the grid spacing would otherwise slip between
    // two samples and the triangle spanning it would lift natural ground across
    // the carriageway (device: "the terrain poked through in some places").
    // 1.45 cells covers the full cell diagonal: at 0.75 a grid corner up to
    // ~1.4 cells from a road cut stayed on natural ground and its triangle
    // crossed UP through the deck (RT_POKE_REPORT: 9.45% of deck area at LOD0).
    const double flattenDilate = static_cast<double>(step) * 1.45;
    std::vector<float> H(static_cast<size_t>(n) * n);
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            double x = node.minX + i * step;
            double z = node.minZ + j * step;
            double y = terrainHeight(params, noise, x, z, flattenDilate);
            // FIX A corner clamp: a corner within ~1.6 cells of a road corridor
            // never exceeds the corridor's plane — an uphill lot pad's higher
            // ground starts its climb one cell later instead of tilting a
            // triangle across the sidewalk.
            if (params.flattenIndex) {
                const double rp = roadPlaneNear(*params.flattenIndex, params.flatten,
                                                x, z, static_cast<double>(step) * 1.6);
                if (rp < 1e29 &&
                    !padPlaneAbove(*params.flattenIndex, params.flatten, x, z, rp))
                    y = std::min(y, rp);
            }
            H[static_cast<size_t>(j) * n + i] = static_cast<float>(y);
            minY = std::min(minY, static_cast<float>(y));
            maxY = std::max(maxY, static_cast<float>(y));
            Vertex v(Vec3(x, y, z), terrainNormal(params, noise, x, z, eps));
            v.u = static_cast<float>(x / node.size);
            v.v = static_cast<float>(z / node.size);
            v.color = terrainColor(x, z, y, v.normal.y, noise, params);
            mesh.vertices.push_back(v);
        }
    }

    // Second pass: bake the CDLOD morph target into tangent. On a regular grid the
    // coarser level keeps even-indexed vertices; odd vertices collapse to the
    // average of their even neighbours (the coarse mesh's height at that XZ). Only
    // the height changes — the XZ of an edge midpoint is already the coarse midpoint.
    auto h = [&](int i, int j) { return H[static_cast<size_t>(j) * n + i]; };
    // Flatten-owned vertices DON'T morph (device: "pokes throughout the entire
    // city"): the morph target is a blind neighbour average, so a vertex sitting
    // on a road corridor — clamped to the deck plane in pass one — lerped back
    // UP toward its natural-ground neighbours at distance and rose through the
    // asphalt. Pin such vertices to their carved height at every LOD; they equal
    // the fine height, so there is nothing to pop.
    const FlattenGrid* fg = params.flattenIndex ? params.flattenIndex.get() : nullptr;
    static const FlattenGrid kNoGrid;
    auto corridorOwned = [&](double x, double z) {
        if (params.flatten.empty()) return false;
        return flattenCovers(fg ? *fg : kNoGrid, params.flatten, x, z, flattenDilate);
    };
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            bool ei = (i % 2 == 0), ej = (j % 2 == 0);
            float my;
            if (ei && ej)        my = h(i, j);
            else if (!ei && ej)  my = 0.5f * (h(i - 1, j) + h(i + 1, j));
            else if (ei && !ej)  my = 0.5f * (h(i, j - 1) + h(i, j + 1));
            else                 my = 0.25f * (h(i - 1, j - 1) + h(i + 1, j - 1) +
                                               h(i - 1, j + 1) + h(i + 1, j + 1));
            Vertex& v = mesh.vertices[static_cast<size_t>(j) * n + i];
            if (my != h(i, j) && corridorOwned(v.position.x, v.position.z))
                my = h(i, j);
            v.tangent = Vec3(v.position.x, my, v.position.z);   // morph-target pos
        }
    }

    mesh.indices.reserve(static_cast<size_t>(res) * res * 6);
    for (int j = 0; j < res; j++) {
        for (int i = 0; i < res; i++) {
            uint32_t a = static_cast<uint32_t>(j * n + i);
            uint32_t b = a + 1;
            uint32_t c = a + static_cast<uint32_t>(n);
            uint32_t d = c + 1;
            // Clockwise-front winding (top faces up), matching generateTerrainChunks.
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }

    out.boundsMin = Vec3(node.minX, minY, node.minZ);
    out.boundsMax = Vec3(node.minX + node.size, maxY, node.minZ + node.size);
    return out;
}

}  // namespace engine
