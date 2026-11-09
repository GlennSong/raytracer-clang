#include "light_clusters.h"

#include <algorithm>
#include <cmath>

namespace engine {

int ClusterGrid::sliceOf(float depth) const {
    if (depth <= zNear) return 0;
    const float t = std::log(depth / zNear) / std::log(zFar / zNear);
    return std::clamp(static_cast<int>(t * static_cast<float>(slices)), 0, slices - 1);
}

float ClusterGrid::sliceNear(int s) const {
    return zNear * std::pow(zFar / zNear, static_cast<float>(s) / static_cast<float>(slices));
}

void buildClusters(const ClusterGrid& grid, const Mat4& view, float tanHalfY, float aspect,
                   const std::vector<ClusterLight>& lights, ClusterLists& out, int cap) {
    const int nCells = grid.cells();
    // every (froxel, light) touch, then counted into the froxels (a counting sort: no per-froxel lists to grow)
    static thread_local std::vector<std::pair<uint32_t, uint32_t>> touches;
    touches.clear();
    const float tanHalfX = tanHalfY * aspect;
    // a tile's edges as the ratio x / depth (and y / depth) on the near-to-far rays through them
    auto edgeX = [&](int t) { return tanHalfX * (2.0f * static_cast<float>(t) / static_cast<float>(grid.tilesX) - 1.0f); };
    auto edgeY = [&](int t) { return tanHalfY * (2.0f * static_cast<float>(t) / static_cast<float>(grid.tilesY) - 1.0f); };
    auto tileOf = [](float ratio, float tanHalf, int tiles) {
        const float u = (ratio / tanHalf + 1.0f) * 0.5f;
        return std::clamp(static_cast<int>(std::floor(u * static_cast<float>(tiles))), 0, tiles - 1);
    };
    for (std::size_t li = 0; li < lights.size(); ++li) {
        const ClusterLight& L = lights[li];
        const Vec3& w = L.position;
        // view space: x right, y up, depth = -z (the view looks down -Z)
        const float x = static_cast<float>(view.m[0][0] * w.x + view.m[0][1] * w.y + view.m[0][2] * w.z + view.m[0][3]);
        const float y = static_cast<float>(view.m[1][0] * w.x + view.m[1][1] * w.y + view.m[1][2] * w.z + view.m[1][3]);
        const float d = -static_cast<float>(view.m[2][0] * w.x + view.m[2][1] * w.y + view.m[2][2] * w.z + view.m[2][3]);
        const float r = L.range;
        if (r <= 0 || d + r < grid.zNear || d - r > grid.zFar) continue;
        const float dMin = std::max(d - r, grid.zNear), dMax = std::min(d + r, grid.zFar);
        // the ratios the sphere can reach, over its depth range (conservative)
        const float rxLo = std::min((x - r) / dMin, (x - r) / dMax), rxHi = std::max((x + r) / dMin, (x + r) / dMax);
        const float ryLo = std::min((y - r) / dMin, (y - r) / dMax), ryHi = std::max((y + r) / dMin, (y + r) / dMax);
        if (rxHi < -tanHalfX || rxLo > tanHalfX || ryHi < -tanHalfY || ryLo > tanHalfY) continue;
        const int tx0 = tileOf(rxLo, tanHalfX, grid.tilesX), tx1 = tileOf(rxHi, tanHalfX, grid.tilesX);
        const int ty0 = tileOf(ryLo, tanHalfY, grid.tilesY), ty1 = tileOf(ryHi, tanHalfY, grid.tilesY);
        const int s0 = grid.sliceOf(dMin), s1 = grid.sliceOf(dMax);
        const float r2 = r * r;
        for (int s = s0; s <= s1; ++s) {
            const float zn = grid.sliceNear(s), zf = grid.sliceNear(s + 1);
            for (int ty = ty0; ty <= ty1; ++ty) {
                const float a = edgeY(ty), b = edgeY(ty + 1);
                const float yLo = std::min(a * zn, a * zf), yHi = std::max(b * zn, b * zf);
                for (int tx = tx0; tx <= tx1; ++tx) {
                    const float c = edgeX(tx), e = edgeX(tx + 1);
                    const float xLo = std::min(c * zn, c * zf), xHi = std::max(e * zn, e * zf);
                    // the sphere against the froxel's box (view space)
                    const float qx = std::clamp(x, xLo, xHi) - x, qy = std::clamp(y, yLo, yHi) - y, qz = std::clamp(d, zn, zf) - d;
                    if (qx * qx + qy * qy + qz * qz > r2) continue;
                    touches.push_back({static_cast<uint32_t>((s * grid.tilesY + ty) * grid.tilesX + tx), static_cast<uint32_t>(li)});
                }
            }
        }
    }
    std::vector<uint32_t> start(static_cast<std::size_t>(nCells) + 1, 0u);
    for (const auto& t : touches) ++start[t.first + 1];
    for (int c = 0; c < nCells; ++c) start[static_cast<std::size_t>(c) + 1] += start[static_cast<std::size_t>(c)];
    static thread_local std::vector<uint32_t> sorted;
    sorted.resize(touches.size());
    {
        std::vector<uint32_t> fill(start.begin(), start.end() - 1);
        for (const auto& t : touches) sorted[fill[t.first]++] = t.second;
    }
    out.offsetCount.assign(static_cast<std::size_t>(nCells), 0u);
    out.indices.clear();
    out.indices.reserve(sorted.size());
    out.maxInCell = 0;
    out.capped = 0;
    const int keep = std::clamp(cap, 1, 255);
    const Mat4 toWorld = view.inverse();
    for (int cIdx = 0; cIdx < nCells; ++cIdx) {
        uint32_t* b0 = sorted.data() + start[static_cast<std::size_t>(cIdx)];
        uint32_t* b1 = sorted.data() + start[static_cast<std::size_t>(cIdx) + 1];
        const int n = static_cast<int>(b1 - b0);
        out.maxInCell = std::max(out.maxInCell, n);
        int kept = n;
        if (n > keep) {
            ++out.capped;
            // the nearest to the cell's middle stay
            const int s = cIdx / (grid.tilesX * grid.tilesY), ty = (cIdx / grid.tilesX) % grid.tilesY, tx = cIdx % grid.tilesX;
            const float zm = 0.5f * (grid.sliceNear(s) + grid.sliceNear(s + 1));
            const float xm = 0.5f * (edgeX(tx) + edgeX(tx + 1)) * zm, ym = 0.5f * (edgeY(ty) + edgeY(ty + 1)) * zm;
            const Vec3 mid = toWorld.transformPoint(Vec3(xm, ym, -zm));
            std::partial_sort(b0, b0 + keep, b1, [&](uint32_t p, uint32_t q) {
                return (lights[p].position - mid).lengthSquared() < (lights[q].position - mid).lengthSquared();
            });
            kept = keep;
        }
        out.offsetCount[static_cast<std::size_t>(cIdx)] =
            (static_cast<uint32_t>(out.indices.size()) & 0xFFFFFFu) | (static_cast<uint32_t>(kept) << 24);
        out.indices.insert(out.indices.end(), b0, b0 + kept);
    }
}

}  // namespace engine
