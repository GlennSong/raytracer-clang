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
    RenderMesh mesh = polygonizeSdf(body, bounds, params.resolution);

    // Stylized option: unweld every triangle and flat-shade it, so at a low
    // polygonization resolution the rock reads as a hand-cut low-poly prop
    // (matching the city's look) instead of a smooth blob.
    if (params.faceted) {
        RenderMesh flat;
        flat.vertices.reserve(mesh.indices.size());
        flat.indices.reserve(mesh.indices.size());
        for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            Vertex a = mesh.vertices[mesh.indices[i]];
            Vertex b = mesh.vertices[mesh.indices[i + 1]];
            Vertex c = mesh.vertices[mesh.indices[i + 2]];
            Vec3 n = cross(c.position - a.position, b.position - a.position);
            if (n.lengthSquared() < 1e-14) continue;   // degenerate sliver
            n = normalize(n);
            a.normal = b.normal = c.normal = n;
            for (const Vertex& v : {a, b, c}) {
                flat.indices.push_back(static_cast<uint32_t>(flat.vertices.size()));
                flat.vertices.push_back(v);
            }
        }
        mesh = std::move(flat);
    }

    // Bake a grayscale brightness variation into the vertex color so the rock
    // reads as mottled grey (the material albedo is multiplied by this), rather
    // than a single flat tone. Faceted rocks tint per FACE (all three verts
    // sample the face centroid) so each facet is a constant plate of tone.
    Noise tint(seed + 50u);
    auto tone = [&](const Vec3& p) {
        double n = tint.fbm3(p.x * 1.5, p.y * 1.5, p.z * 1.5, 2);
        return static_cast<float>(0.78 + 0.30 * (0.5 * n + 0.5));   // ~0.78..1.08
    };
    if (params.faceted) {
        for (std::size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
            Vec3 c = (mesh.vertices[i].position + mesh.vertices[i + 1].position +
                      mesh.vertices[i + 2].position) / 3.0;
            float b = tone(c);
            mesh.vertices[i].color = mesh.vertices[i + 1].color =
                mesh.vertices[i + 2].color = Vec3(b, b, b);
        }
    } else {
        for (Vertex& v : mesh.vertices) {
            float b = tone(v.position);
            v.color = Vec3(b, b, b);
        }
    }
    return mesh;
}

}  // namespace engine
