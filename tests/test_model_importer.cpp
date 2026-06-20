#include "test_framework.h"

#include "../src/engine/model_importer.h"
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

// loadCpu is the shared parse both the realtime upload and the offline path
// tracer consume — so a glTF model uses the same material/texture system on
// every backend (ADR-0039). The fixture is a textured quad with a baseColor
// texture, metallic 0, roughness 0.6.
TEST_CASE(gltf_loadcpu_parses_geometry_material_and_texture) {
    CpuModel m = ModelImporter::loadCpu("tests/assets/textured_quad.gltf");
    CHECK(m.meshes.size() == 1);
    const CpuMesh& mesh = m.meshes[0];

    // Geometry: 4 vertices, 2 triangles (6 indices), UVs present.
    CHECK(mesh.geometry.vertices.size() == 4);
    CHECK(mesh.geometry.indices.size() == 6);
    bool hasUV = false;
    for (const Vertex& v : mesh.geometry.vertices)
        if (v.u != 0.0f || v.v != 0.0f) hasUV = true;
    CHECK(hasUV);

    // Material factors round-trip from the glTF.
    CHECK_APPROX(mesh.material.metallic, 0.0, 1e-6);
    CHECK_APPROX(mesh.material.roughness, 0.6, 1e-6);

    // The baseColor texture decoded to a real RGBA image (8x8 checker).
    CHECK(mesh.material.baseColorTex.valid());
    CHECK(mesh.material.baseColorTex.width == 8);
    CHECK(mesh.material.baseColorTex.height == 8);
    CHECK(mesh.material.baseColorTex.channels >= 3);
}

TEST_CASE(gltf_loadcpu_missing_file_is_empty) {
    CpuModel m = ModelImporter::loadCpu("tests/assets/does_not_exist.gltf");
    CHECK(m.meshes.empty());
}
