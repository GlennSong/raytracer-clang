#include "rock.h"
#include "../mesh_builder.h"

namespace engine {

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

}  // namespace engine
