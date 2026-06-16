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
    // Richer, more saturated palette with a green -> olive -> earth gradient on
    // flat ground, warm-grey rock on slopes, and snow on high gentle ground.
    const Vec3 grass(0.13, 0.30, 0.07);    // deep green
    const Vec3 dryGrass(0.34, 0.36, 0.12); // olive / dry meadow
    const Vec3 dirt(0.30, 0.20, 0.10);     // rich earth brown
    const Vec3 rock(0.29, 0.27, 0.25);     // warm grey
    const Vec3 snow(0.90, 0.92, 0.96);

    double slope = 1.0 - clamp01(normalUp);                 // 0 flat .. 1 vertical
    double rockFactor = smoothstep(0.30, 0.62, slope);

    // Two-stop gradient over the noise term: green -> dry meadow -> earth, so the
    // ground varies richly instead of a flat green/brown lerp.
    double t = clamp01(noiseValue * 0.5 + 0.5);
    Vec3 ground = t < 0.5 ? mixv(grass, dryGrass, t * 2.0)
                          : mixv(dryGrass, dirt, (t - 0.5) * 2.0);
    Vec3 c = mixv(ground, rock, rockFactor);

    // Snow on high, non-steep ground (absolute altitude — a no-op on low terrain,
    // caps mountains). Snow doesn't cling to cliffs.
    double snowFactor = smoothstep(74.0, 108.0, height) *
                        (1.0 - smoothstep(0.42, 0.68, slope));
    c = mixv(c, snow, snowFactor);

    return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z));
}

double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ) {
    double nx = worldX * params.noiseScale;
    double nz = worldZ * params.noiseScale;
    double h = params.warp > 0.0
                   ? noise.warpedFbm2(nx, nz, params.warp, params.octaves)
                   : noise.fbm2(nx, nz, params.octaves);
    h *= params.heightScale;
    // Long-range ridged mountain layer (low frequency, always-positive relief).
    if (params.mountainHeight > 0.0f) {
        double m = noise.fbm2(worldX * params.mountainScale,
                              worldZ * params.mountainScale, 4);
        double ridge = 1.0 - std::abs(m);          // [0,1], peaks where m ~ 0
        h += ridge * ridge * params.mountainHeight;
    }
    return h;
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

RenderMesh generateTerrainRing(const TerrainParams& params, const Noise& noise,
                               float innerHalf, float outerHalf, int cells) {
    RenderMesh mesh;
    cells = std::max(2, cells);
    const int n = cells + 1;
    const float step = (outerHalf * 2.0f) / cells;

    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            float x = -outerHalf + i * step;
            float z = -outerHalf + j * step;
            float y = static_cast<float>(terrainHeight(params, noise, x, z));
            mesh.vertices.push_back(Vertex(Vec3(x, y, z), Vec3(0, 1, 0)));
        }
    }
    for (int j = 0; j < cells; j++) {
        for (int i = 0; i < cells; i++) {
            float x0 = -outerHalf + i * step, x1 = x0 + step;
            float z0 = -outerHalf + j * step, z1 = z0 + step;
            // Skip quads entirely inside the inner hole (left for the finer tile).
            if (std::max(std::abs(x0), std::abs(x1)) <= innerHalf &&
                std::max(std::abs(z0), std::abs(z1)) <= innerHalf)
                continue;
            uint32_t a = static_cast<uint32_t>(j * n + i);
            uint32_t b = a + 1;
            uint32_t c = a + static_cast<uint32_t>(n);
            uint32_t d = c + 1;
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }

    MeshBuilder::recomputeNormals(mesh);
    MeshBuilder::generatePlanarUVs(mesh, /*axis=*/1, /*scale=*/1.0f / (outerHalf * 2.0f));
    for (Vertex& v : mesh.vertices) {
        double nv = noise.noise2(v.position.x * 0.15, v.position.z * 0.15);
        v.color = terrainColor(v.position.y, v.normal.y, nv);
    }
    return mesh;
}

std::vector<RenderMesh> generateTerrainLOD(const TerrainParams& params,
                                           const Noise& noise, int levels, int cells) {
    std::vector<RenderMesh> rings;
    float inner = params.size * 0.5f;          // central tile edge
    for (int l = 0; l < levels; l++) {
        float outer = inner * 2.0f;            // each ring doubles the extent
        rings.push_back(generateTerrainRing(params, noise, inner, outer, cells));
        inner = outer;
    }
    return rings;
}

}  // namespace engine
