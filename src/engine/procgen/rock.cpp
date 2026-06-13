#include "rock.h"
#include "../mesh_builder.h"
#include "sdf.h"

#include <random>

namespace engine {

namespace {
// A uniformly-distributed point on the unit sphere (for placing lumps/cuts).
Vec3 randomUnitVec(std::mt19937& gen) {
    std::uniform_real_distribution<double> u(-1.0, 1.0), a(0.0, 2.0 * PI);
    double z = u(gen);
    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    double theta = a(gen);
    return Vec3(r * std::cos(theta), r * std::sin(theta), z);
}
}  // namespace

RenderMesh generateRock(const RockParams& params, const Noise& noise) {
    RenderMesh mesh = MeshBuilder::sphere(params.radius, params.stacks, params.slices);

    for (Vertex& v : mesh.vertices) {
        // The sphere's vertex normal is its radial direction; displace along it
        // by FBM so the surface gains lumps and hollows.
        Vec3 dir = normalize(v.position);
        double n = noise.fbm3(v.position.x * params.noiseScale,
                              v.position.y * params.noiseScale,
                              v.position.z * params.noiseScale,
                              params.octaves);
        v.position = v.position + dir * (n * params.displacement * params.radius);
    }

    // Displacement broke the analytic normals; rebuild them from the deformed
    // geometry (engine clockwise-front convention). Sphere seam/pole vertices
    // that no triangle references come back zero-length — fall back to the
    // radial direction so every normal is valid and outward.
    MeshBuilder::recomputeNormals(mesh);
    for (Vertex& v : mesh.vertices)
        if (v.normal.lengthSquared() < 1e-12) v.normal = normalize(v.position);
    return mesh;
}

RenderMesh generateRockSdf(const RockSdfParams& params, uint32_t seed) {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> radial(0.7, 1.0);
    std::uniform_real_distribution<double> lumpR(0.4, 1.0);

    const double base = params.baseRadius;

    // Base sphere + lumps (spheres riding the surface) fused with smooth-union.
    std::vector<Sdf> parts;
    parts.push_back(sdfSphere(Vec3(0, 0, 0), base));
    for (int i = 0; i < params.lumps; i++) {
        Vec3 dir = randomUnitVec(gen);
        double dist = base * radial(gen);
        double r = base * params.lumpScale * lumpR(gen);
        parts.push_back(sdfSphere(dir * dist, r));
    }
    Sdf body = sdfSmoothUnion(parts, params.smoothness);

    // Subtract a few spheres straddling the surface for flat-ish facets/bites.
    for (int i = 0; i < params.cuts; i++) {
        Vec3 dir = randomUnitVec(gen);
        double r = base * (0.5 + 0.4 * lumpR(gen));
        body = sdfSubtract(body, sdfSphere(dir * (base + r * 0.6), r));
    }

    double extent = base * (1.0 + params.lumpScale) + params.smoothness;
    double pad = extent * 0.15 + 2.0 * extent / std::max(1, params.resolution);
    Vec3 m(extent + pad, extent + pad, extent + pad);
    SdfBounds bounds{Vec3(0, 0, 0) - m, m};
    return polygonizeSdf(body, bounds, params.resolution);
}

}  // namespace engine
