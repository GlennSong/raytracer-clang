#include "test_framework.h"
#include "../src/engine/anim/skin_import.h"
#include "../src/engine/anim/skinned_mesh.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace engine;
namespace anim = engine::anim;

namespace {

// A minimal rigged glTF written from scratch: two joints (root at the origin,
// tip one unit up), a 6-vertex vertical ribbon whose bottom row is welded to
// root, top row to tip, middle row blended 50/50 — the smallest mesh that
// exercises hierarchy import, inverse binds, influence remapping, and blends.
struct TempSkinGltf {
    std::string gltfPath = "test_skin_model.gltf";
    std::string binPath = "test_skin_model.bin";

    TempSkinGltf() {
        std::vector<uint8_t> bin;
        auto putF = [&](float f) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(&f);
            bin.insert(bin.end(), p, p + 4);
        };
        auto putU16 = [&](uint16_t v) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(&v);
            bin.insert(bin.end(), p, p + 2);
        };

        // positions @0 (72 B): rows y = 0, 1, 2, x = ±0.5
        const float px[6] = {-0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f};
        const float py[6] = {0, 0, 1, 1, 2, 2};
        for (int i = 0; i < 6; i++) { putF(px[i]); putF(py[i]); putF(0); }
        // normals @72 (72 B): +Z
        for (int i = 0; i < 6; i++) { putF(0); putF(0); putF(1); }
        // joints @144 (24 B): ubyte4 — row0 -> joint 0, row1 -> 0+1, row2 -> 1
        const uint8_t joints[6][4] = {{0, 0, 0, 0}, {0, 0, 0, 0},
                                      {0, 1, 0, 0}, {0, 1, 0, 0},
                                      {1, 0, 0, 0}, {1, 0, 0, 0}};
        for (auto& j : joints) bin.insert(bin.end(), j, j + 4);
        // weights @168 (96 B): float4
        const float weights[6][4] = {{1, 0, 0, 0},       {1, 0, 0, 0},
                                     {0.5f, 0.5f, 0, 0}, {0.5f, 0.5f, 0, 0},
                                     {1, 0, 0, 0},       {1, 0, 0, 0}};
        for (auto& w : weights) for (float f : w) putF(f);
        // indices @264 (24 B): ushort, CCW from +Z
        const uint16_t idx[12] = {0, 1, 3, 0, 3, 2, 2, 3, 5, 2, 5, 4};
        for (uint16_t i : idx) putU16(i);
        // inverse binds @288 (128 B): joint0 identity; joint1 translate(0,-1,0)
        const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 1};
        const float invTip[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0, -1, 0, 1};
        for (float f : identity) putF(f);
        for (float f : invTip) putF(f);

        std::ofstream(binPath, std::ios::binary)
            .write(reinterpret_cast<const char*>(bin.data()), bin.size());

        std::ofstream(gltfPath) << R"({
  "asset": {"version": "2.0"},
  "scene": 0,
  "scenes": [{"nodes": [0, 2]}],
  "nodes": [
    {"name": "root", "children": [1]},
    {"name": "tip", "translation": [0, 1, 0]},
    {"name": "body", "mesh": 0, "skin": 0}
  ],
  "skins": [{"joints": [0, 1], "inverseBindMatrices": 5}],
  "meshes": [{"primitives": [{
    "attributes": {"POSITION": 0, "NORMAL": 1, "JOINTS_0": 2, "WEIGHTS_0": 3},
    "indices": 4
  }]}],
  "buffers": [{"uri": "test_skin_model.bin", "byteLength": 416}],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 72},
    {"buffer": 0, "byteOffset": 72, "byteLength": 72},
    {"buffer": 0, "byteOffset": 144, "byteLength": 24},
    {"buffer": 0, "byteOffset": 168, "byteLength": 96},
    {"buffer": 0, "byteOffset": 264, "byteLength": 24},
    {"buffer": 0, "byteOffset": 288, "byteLength": 128}
  ],
  "accessors": [
    {"bufferView": 0, "componentType": 5126, "count": 6, "type": "VEC3",
     "min": [-0.5, 0, 0], "max": [0.5, 2, 0]},
    {"bufferView": 1, "componentType": 5126, "count": 6, "type": "VEC3"},
    {"bufferView": 2, "componentType": 5121, "count": 6, "type": "VEC4"},
    {"bufferView": 3, "componentType": 5126, "count": 6, "type": "VEC4"},
    {"bufferView": 4, "componentType": 5123, "count": 12, "type": "SCALAR"},
    {"bufferView": 5, "componentType": 5126, "count": 2, "type": "MAT4"}
  ]
})";
    }
    ~TempSkinGltf() {
        std::remove(gltfPath.c_str());
        std::remove(binPath.c_str());
    }
};

}  // namespace

TEST_CASE(skin_import_builds_skeleton_and_influences) {
    TempSkinGltf file;
    anim::SkinnedModel model = anim::loadSkinnedModel(file.gltfPath);
    CHECK(model.valid());
    CHECK(model.skeleton.jointCount() == 2);
    int root = model.skeleton.find("root");
    int tip = model.skeleton.find("tip");
    CHECK(root == 0);                                 // parent-before-child
    CHECK(tip == 1);
    CHECK(model.skeleton.joint(tip).parent == root);
    CHECK_APPROX(model.skeleton.joint(tip).bindLocal.position.y, 1.0, 1e-6);

    CHECK(model.meshes.size() == 1);
    const anim::SkinnedMesh& mesh = model.meshes[0];
    CHECK(mesh.bind.vertices.size() == 6);
    CHECK(mesh.influences.size() == 6);
    CHECK(mesh.bind.indices.size() == 12);
    // The middle row blends both joints equally.
    CHECK_APPROX(mesh.influences[2].weights[0], 0.5, 1e-6);
    CHECK_APPROX(mesh.influences[2].weights[1], 0.5, 1e-6);
    CHECK(mesh.influences[2].joints[0] == 0);
    CHECK(mesh.influences[2].joints[1] == 1);
    // The top row is welded to the tip joint.
    CHECK(mesh.influences[4].joints[0] == 1);
    CHECK_APPROX(mesh.influences[4].weights[0], 1.0, 1e-6);
}

TEST_CASE(skin_import_bind_pose_deforms_nothing) {
    TempSkinGltf file;
    anim::SkinnedModel model = anim::loadSkinnedModel(file.gltfPath);
    CHECK(model.valid());

    anim::Pose bind = anim::Pose::bind(model.skeleton);
    auto skin = anim::skinningMatrices(
        model.skeleton, anim::worldJointMatrices(model.skeleton, bind));
    RenderMesh posed = anim::skinMesh(model.meshes[0], skin);
    for (size_t i = 0; i < posed.vertices.size(); i++) {
        Vec3 delta = posed.vertices[i].position -
                     model.meshes[0].bind.vertices[i].position;
        CHECK_APPROX(delta.length(), 0.0, 1e-6);
    }
}

TEST_CASE(skin_import_bent_joint_blends_vertices) {
    TempSkinGltf file;
    anim::SkinnedModel model = anim::loadSkinnedModel(file.gltfPath);
    CHECK(model.valid());

    // Bend the tip 90 degrees about +Z; the pivot is (0,1,0).
    anim::Pose pose = anim::Pose::bind(model.skeleton);
    pose.locals[model.skeleton.find("tip")].orientation =
        Quat::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2.0);
    auto skin = anim::skinningMatrices(
        model.skeleton, anim::worldJointMatrices(model.skeleton, pose));
    RenderMesh posed = anim::skinMesh(model.meshes[0], skin);

    // Bottom row (root-welded): untouched.
    CHECK_APPROX((posed.vertices[0].position - Vec3(-0.5, 0, 0)).length(),
                 0.0, 1e-6);
    // Top-left vertex (-0.5, 2, 0): offset from pivot (-0.5, 1, 0) rotates
    // to (-1, -0.5, 0), landing at (-1, 0.5, 0).
    CHECK_APPROX((posed.vertices[4].position - Vec3(-1.0, 0.5, 0)).length(),
                 0.0, 1e-6);
    // Middle-left (-0.5, 1, 0), blended 50/50: halfway between staying put
    // and rotating to (0, 0.5, 0) -> (-0.25, 0.75, 0).
    CHECK_APPROX((posed.vertices[2].position - Vec3(-0.25, 0.75, 0)).length(),
                 0.0, 1e-6);
}

TEST_CASE(skin_import_rejects_missing_or_unskinned_files) {
    CHECK(!anim::loadSkinnedModel("no_such_model.gltf").valid());

    // A valid glTF with no skin is not a skinned model.
    const char* path = "test_unskinned.gltf";
    std::ofstream(path) << R"({"asset": {"version": "2.0"}, "scenes": [{"nodes": []}]})";
    CHECK(!anim::loadSkinnedModel(path).valid());
    std::remove(path);
}
