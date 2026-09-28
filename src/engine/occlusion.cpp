#include "occlusion.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr double kMaxMove = 0.75;      // metres the camera may have moved since the depth was drawn
constexpr double kMinCosTurn = 0.9990; // ...and turned (about 2.5 degrees)
constexpr int kMaxTiles = 48 * 48;     // bigger on screen than this: just draw it
}  // namespace

bool occlusionUsable(const OcclusionDepth& o, const Vec3& eye, const Vec3& forward) {
    if (!o.valid || o.width <= 0 || o.height <= 0) return false;
    if ((eye - o.eye).length() > kMaxMove) return false;
    return dot(forward, o.forward) >= kMinCosTurn;
}

bool occludedBox(const OcclusionDepth& o, const Vec3& loIn, const Vec3& hiIn, double slack) {
    const Vec3 lo = loIn - Vec3(slack, slack, slack), hi = hiIn + Vec3(slack, slack, slack);
    const Mat4& m = o.viewProj;
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30, nearest = -1e30;
    for (int k = 0; k < 8; ++k) {
        const Vec3 p((k & 1) ? hi.x : lo.x, (k & 2) ? hi.y : lo.y, (k & 4) ? hi.z : lo.z);
        const double cx = m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3];
        const double cy = m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3];
        const double cz = m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3];
        const double cw = m.m[3][0] * p.x + m.m[3][1] * p.y + m.m[3][2] * p.z + m.m[3][3];
        if (cw <= 1e-4) return false;   // reaches behind the eye: visible
        const double iw = 1.0 / cw;
        x0 = std::min(x0, cx * iw);
        x1 = std::max(x1, cx * iw);
        y0 = std::min(y0, cy * iw);
        y1 = std::max(y1, cy * iw);
        nearest = std::max(nearest, cz * iw);   // reverse-Z: larger is nearer
    }
    if (nearest >= 1.0) return false;           // at the near plane
    // to tiles (y flipped in viewProj: row 0 at the top, as the depth buffer)
    const double W = static_cast<double>(o.width) * o.tilePixels, H = static_cast<double>(o.height) * o.tilePixels;
    auto tileX = [&](double ndc) { return static_cast<int>(std::floor((ndc * 0.5 + 0.5) * W / o.tilePixels)); };
    auto tileY = [&](double ndc) { return static_cast<int>(std::floor((ndc * 0.5 + 0.5) * H / o.tilePixels)); };
    const int tx0 = std::max(0, tileX(x0)), tx1 = std::min(o.width - 1, tileX(x1));
    const int ty0 = std::max(0, tileY(y0)), ty1 = std::min(o.height - 1, tileY(y1));
    if (tx0 > tx1 || ty0 > ty1) return false;   // off screen: the frustum decides
    if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > kMaxTiles) return false;
    for (int ty = ty0; ty <= ty1; ++ty)
        for (int tx = tx0; tx <= tx1; ++tx)
            if (nearest >= o.depth[static_cast<std::size_t>(ty) * o.width + tx]) return false;   // not behind this tile
    return true;
}

}  // namespace engine
