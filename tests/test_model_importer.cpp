#include "test_framework.h"

#include "../src/engine/model_importer.h"
#include <cmath>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

// Resolve a repo-relative asset path. Under CMake/CTest the working directory is
// the build tree, so a bare relative path won't find the fixture; RT_SOURCE_DIR
// (set by CMake, like the gun/flora tests) anchors it. The Makefile build runs
// from the repo root and defines no such macro, so the relative path stands.
namespace {
std::string assetPath(const char* rel) {
#ifdef RT_SOURCE_DIR
    return std::string(RT_SOURCE_DIR) + "/" + rel;
#else
    return std::string(rel);
#endif
}
}  // namespace

// loadCpu is the shared parse both the realtime upload and the offline path
// tracer consume — so a glTF model uses the same material/texture system on
// every backend (ADR-0039). The fixture is a textured quad with a baseColor
// texture, metallic 0, roughness 0.6.
TEST_CASE(gltf_loadcpu_parses_geometry_material_and_texture) {
    CpuModel m = ModelImporter::loadCpu(assetPath("tests/assets/textured_quad.gltf"));
    CHECK(m.meshes.size() == 1);
    if (m.meshes.empty()) return;          // never index into an empty result
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
    CpuModel m = ModelImporter::loadCpu(assetPath("tests/assets/does_not_exist.gltf"));
    CHECK(m.meshes.empty());
}
