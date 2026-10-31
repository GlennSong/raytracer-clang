#ifndef RAYTRACER_ENGINE_PROCGEN_DRAWN_CELL_H
#define RAYTRACER_ENGINE_PROCGEN_DRAWN_CELL_H

namespace engine {

// THE DRAWN CELL (ADR-0280, #117 item 22): the height inside one grid cell of a terrain mesh, interpolated across
// the triangles the meshers emit -- (a, b, d) when u >= v, else (a, d, c), split on the a-d diagonal, a the cell's
// min corner and d its max (generateLodNodeMesh, generateTerrain). `corner(di, dj)` is the height at the corner
// (di, dj) in {0, 1}^2; only the three corners of the triangle under (u, v) are asked for. One rule for every
// sampler of the drawn ground: the CDLOD leaf (lodSurfaceHeight), the static grid (terrainGridSurfaceHeight) and
// the baked pyramid (HeightTile::sampleLocal).
template <class Corner>
double drawnCellHeight(double u, double v, Corner&& corner) {
    const double ha = corner(0, 0), hd = corner(1, 1);
    if (u >= v) {
        const double hb = corner(1, 0);
        return ha + u * (hb - ha) + v * (hd - hb);
    }
    const double hc = corner(0, 1);
    return ha + v * (hc - ha) + u * (hd - hc);
}

}  // namespace engine

#endif
