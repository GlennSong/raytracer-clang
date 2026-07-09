#include "structure_set.h"

#include <cmath>

namespace engine {

RenderMesh bakeWallMesh(const std::vector<WallSegment>& walls, const Vec3& color) {
    RenderMesh mesh;
    for (const WallSegment& w : walls) {
        // Bottom corners sit `drop` below each top corner (walls are vertical).
        Vec3 botA(w.topA.x, w.topA.y - static_cast<Real>(w.dropA), w.topA.z);
        Vec3 botB(w.topB.x, w.topB.y - static_cast<Real>(w.dropB), w.topB.z);
        // Face normal: horizontal, perpendicular to the run, pointing away from the
        // hill it retains. For a two-sided wall we emit both windings so it reads
        // from either side (thin structure; no volume yet).
        Vec3 run = w.topB - w.topA;
        Vec3 n(run.z, 0, -run.x);   // left normal of the run on XZ
        Real len = std::sqrt(n.x * n.x + n.z * n.z);
        if (len > 1e-6f) { n.x /= len; n.z /= len; }
        // Front + back quads (topA, topB, botB, botA), both windings.
        MeshBuilder::emitQuad(mesh, w.topA, w.topB, botB, botA, n, color);
        MeshBuilder::emitQuad(mesh, w.topB, w.topA, botA, botB, Vec3(-n.x, 0, -n.z), color);
        // Thin top cap so the crown doesn't read as a paper edge (0 width is fine
        // visually; a small inset would need a thickness param — deferred).
    }
    return mesh;
}

}  // namespace engine
