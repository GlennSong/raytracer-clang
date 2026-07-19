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
        // Backfill bench: horizontal cap from the top edge back over the
        // conform's feather, meeting natural ground at its far edge (the
        // wall top IS the natural height at the feather's end).
        if (w.bench > 0 && (w.out.x != 0 || w.out.z != 0)) {
            const Vec3 backA = w.topA + w.out * static_cast<Real>(w.bench);
            const Vec3 backB = w.topB + w.out * static_cast<Real>(w.bench);
            MeshBuilder::emitQuad(mesh, w.topA, backA, backB, w.topB,
                                  Vec3(0, 1, 0), color);
        }
    }
    return mesh;
}

}  // namespace engine
