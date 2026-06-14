#include "terrain.h"
#include "../mesh_builder.h"

#include <algorithm>

namespace engine {

namespace {
double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
double smoothstep(double e0, double e1, double x) {
    double t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}
Vec3 mixv(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }
}  // namespace

Vec3 terrainColor(double height, double normalUp, double noiseValue) {
    (void)height;
    const Vec3 grass(0.20, 0.38, 0.15);   // green
    const Vec3 dirt(0.42, 0.30, 0.16);    // brown
    const Vec3 rock(0.40, 0.38, 0.36);    // grey

    // Steep ground reads as rock; gentle ground is a patchy mix of grass and
    // dirt chosen by the noise term, so it's green in places and brown in others
    // rather than a uniform color.
    double slope = 1.0 - clamp01(normalUp);                 // 0 flat .. 1 vertical
    double rockFactor = smoothstep(0.32, 0.60, slope);
    double dirtFactor = smoothstep(-0.25, 0.25, noiseValue); // noise: grass<->dirt
    Vec3 ground = mixv(grass, dirt, dirtFactor);
    Vec3 c = mixv(ground, rock, rockFactor);
    return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z));
}

double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ) {
    double nx = worldX * params.noiseScale;
    double nz = worldZ * params.noiseScale;
    double h = params.warp > 0.0
                   ? noise.warpedFbm2(nx, nz, params.warp, params.octaves)
                   : noise.fbm2(nx, nz, params.octaves);
    return h * params.heightScale;
}

RenderMesh generateTerrain(const TerrainParams& params, const Noise& noise) {
    const int res = std::max(1, params.resolution);
    const int n = res + 1;                       // vertices per side
    const float half = params.size * 0.5f;
    const float step = params.size / static_cast<float>(res);

    RenderMesh mesh;
    mesh.vertices.reserve(static_cast<size_t>(n) * n);
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            double x = -half + i * step;
            double z = -half + j * step;
            double y = terrainHeight(params, noise, x, z);
            // Normal is recomputed below; UV is set by generatePlanarUVs.
            mesh.vertices.push_back(Vertex(Vec3(x, y, z), Vec3(0, 1, 0)));
        }
    }

    mesh.indices.reserve(static_cast<size_t>(res) * res * 6);
    for (int j = 0; j < res; j++) {
        for (int i = 0; i < res; i++) {
            uint32_t a = static_cast<uint32_t>(j * n + i);
            uint32_t b = a + 1;                  // +x
            uint32_t c = a + static_cast<uint32_t>(n);   // +z
            uint32_t d = c + 1;                  // +x, +z
            // Clockwise-front winding (matches the box convention) so the top
            // surface faces up — and recomputeNormals derives up-facing normals.
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }

    MeshBuilder::recomputeNormals(mesh);
    // UVs span [0,1] across the patch (offset by half, scaled by 1/size).
    MeshBuilder::generatePlanarUVs(mesh, /*axis=*/1, /*scale=*/1.0f / params.size);

    // Bake height/slope coloration into per-vertex colors (a low-frequency
    // noise term varies it). The shader multiplies these with the material.
    for (Vertex& v : mesh.vertices) {
        double nv = noise.noise2(v.position.x * 0.15, v.position.z * 0.15);
        v.color = terrainColor(v.position.y, v.normal.y, nv);
    }
    return mesh;
}

}  // namespace engine
