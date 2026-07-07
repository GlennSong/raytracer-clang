#include "skin_import.h"

#include "../../log.h"

// tinygltf's implementation TU lives in model_importer.cpp; this file only
// consumes the header (same NO_INCLUDE_JSON arrangement).
#include <nlohmann/json.hpp>
#define TINYGLTF_NO_INCLUDE_JSON
#include <tinygltf/tiny_gltf.h>

#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace engine {
namespace anim {

namespace {

const uint8_t* accessorData(const tinygltf::Model& model,
                            const tinygltf::Accessor& accessor) {
    const auto& bv = model.bufferViews[accessor.bufferView];
    return model.buffers[bv.buffer].data.data() + bv.byteOffset +
           accessor.byteOffset;
}

size_t accessorStride(const tinygltf::Model& model,
                      const tinygltf::Accessor& accessor) {
    const auto& bv = model.bufferViews[accessor.bufferView];
    if (bv.byteStride > 0) return bv.byteStride;
    int componentSize = tinygltf::GetComponentSizeInBytes(
        static_cast<uint32_t>(accessor.componentType));
    int numComponents = tinygltf::GetNumComponentsInType(
        static_cast<uint32_t>(accessor.type));
    return static_cast<size_t>(componentSize * numComponents);
}

Vec3 readVec3(const uint8_t* ptr) {
    const float* f = reinterpret_cast<const float*>(ptr);
    return Vec3(f[0], f[1], f[2]);
}

// A glTF node's local transform as the engine Transform (TRS; an explicit
// matrix decomposes through transformFromMatrix).
Transform nodeTransform(const tinygltf::Node& node) {
    if (node.matrix.size() == 16) {
        Mat4 m;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                m.m[r][c] = static_cast<Real>(node.matrix[c * 4 + r]);
        return transformFromMatrix(m);
    }
    Transform t;
    if (node.translation.size() == 3)
        t.position = Vec3(node.translation[0], node.translation[1],
                          node.translation[2]);
    if (node.rotation.size() == 4)
        t.orientation = Quat(node.rotation[0], node.rotation[1],
                             node.rotation[2], node.rotation[3]);
    if (node.scale.size() == 3)
        t.scale = Vec3(node.scale[0], node.scale[1], node.scale[2]);
    return t;
}

}  // namespace

SkinnedModel loadSkinnedModel(const std::string& path) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;
    bool binary = path.size() > 4 && path.substr(path.size() - 4) == ".glb";
    bool ok = binary ? loader.LoadBinaryFromFile(&model, &err, &warn, path)
                     : loader.LoadASCIIFromFile(&model, &err, &warn, path);
    if (!warn.empty()) LOG_WARN << "glTF skin: " << warn;
    if (!ok) {
        LOG_ERROR << "glTF skin: failed to load '" << path << "': " << err;
        return {};
    }
    if (model.skins.empty()) {
        LOG_ERROR << "glTF skin: '" << path << "' has no skin";
        return {};
    }
    const tinygltf::Skin& skin = model.skins[0];
    if (skin.joints.empty()) return {};

    // Parent of every node (glTF stores children only).
    std::vector<int> nodeParent(model.nodes.size(), -1);
    for (size_t n = 0; n < model.nodes.size(); n++)
        for (int child : model.nodes[n].children)
            if (child >= 0 && child < static_cast<int>(nodeParent.size()))
                nodeParent[child] = static_cast<int>(n);

    // The skin's joint set, and each joint's nearest ANCESTOR that is also a
    // joint (interior non-joint nodes would be unusual in a character rig;
    // their transforms compose into the child joint's bind local).
    std::unordered_map<int, int> jointSet;   // gltf node -> skin.joints index
    for (size_t j = 0; j < skin.joints.size(); j++)
        jointSet[skin.joints[j]] = static_cast<int>(j);

    auto jointAncestor = [&](int node) {
        int p = nodeParent[node];
        while (p >= 0 && !jointSet.count(p)) p = nodeParent[p];
        return p;   // -1 = a root joint
    };
    auto composedBindLocal = [&](int node) {
        // Compose node matrices from just below the joint ancestor down to
        // `node` (usually a single hop). Transforms ABOVE the root joints
        // (armature/scene nodes) are deliberately ignored — ADR-0073.
        int stop = jointAncestor(node);
        Mat4 local = nodeTransform(model.nodes[node]).matrix();
        for (int p = nodeParent[node]; p >= 0 && p != stop; p = nodeParent[p])
            local = nodeTransform(model.nodes[p]).matrix() * local;
        return transformFromMatrix(local);
    };

    // Build the skeleton parent-before-child: DFS from the root joints
    // through node children, adding the nodes that are joints.
    SkinnedModel result;
    std::vector<int> jointToSkeleton(skin.joints.size(), -1);
    std::vector<std::pair<int, int>> stack;   // (gltf node, skeleton parent)
    for (int j : skin.joints)
        if (jointAncestor(j) < 0) stack.push_back({j, -1});
    // Reverse so the first root is processed first (stack pops from the back).
    std::reverse(stack.begin(), stack.end());
    while (!stack.empty()) {
        auto [node, parent] = stack.back();
        stack.pop_back();
        int self = parent;
        auto it = jointSet.find(node);
        if (it != jointSet.end()) {
            std::string name = model.nodes[node].name.empty()
                                   ? "joint" + std::to_string(it->second)
                                   : model.nodes[node].name;
            self = result.skeleton.addJoint(name, parent,
                                            composedBindLocal(node));
            jointToSkeleton[it->second] = self;
        }
        for (int child : model.nodes[node].children)
            stack.push_back({child, self});
    }
    if (result.skeleton.jointCount() !=
        static_cast<int>(skin.joints.size())) {
        LOG_ERROR << "glTF skin: '" << path
                  << "' joint hierarchy didn't resolve (cyclic or detached)";
        return {};
    }

    // Inverse binds: the accessor is authoritative for imported rigs
    // (column-major mat4 floats), reordered into skeleton order.
    if (skin.inverseBindMatrices >= 0) {
        const auto& acc = model.accessors[skin.inverseBindMatrices];
        const uint8_t* data = accessorData(model, acc);
        size_t stride = accessorStride(model, acc);
        std::vector<Mat4> inverseBinds(result.skeleton.jointCount());
        for (size_t j = 0; j < skin.joints.size() && j < acc.count; j++) {
            const float* f = reinterpret_cast<const float*>(data + j * stride);
            Mat4 m;
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    m.m[r][c] = static_cast<Real>(f[c * 4 + r]);
            inverseBinds[jointToSkeleton[j]] = m;
        }
        result.skeleton.setInverseBinds(std::move(inverseBinds));
    } else {
        result.skeleton.finalize();   // spec default: identity-ish; compute ours
    }

    // Every skinned primitive (JOINTS_0 + WEIGHTS_0) becomes a SkinnedMesh.
    for (const auto& mesh : model.meshes) {
        for (const auto& prim : mesh.primitives) {
            if (prim.mode != -1 && prim.mode != TINYGLTF_MODE_TRIANGLES)
                continue;
            auto posIt = prim.attributes.find("POSITION");
            auto jointsIt = prim.attributes.find("JOINTS_0");
            auto weightsIt = prim.attributes.find("WEIGHTS_0");
            if (posIt == prim.attributes.end() ||
                jointsIt == prim.attributes.end() ||
                weightsIt == prim.attributes.end())
                continue;

            SkinnedMesh out;
            const auto& posAcc = model.accessors[posIt->second];
            size_t vertexCount = posAcc.count;
            out.bind.vertices.resize(vertexCount);
            out.influences.resize(vertexCount);

            const uint8_t* posData = accessorData(model, posAcc);
            size_t posStride = accessorStride(model, posAcc);
            for (size_t i = 0; i < vertexCount; i++)
                out.bind.vertices[i].position = readVec3(posData + i * posStride);

            auto normIt = prim.attributes.find("NORMAL");
            if (normIt != prim.attributes.end()) {
                const auto& acc = model.accessors[normIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++)
                    out.bind.vertices[i].normal = readVec3(data + i * stride);
            }

            // JOINTS_0: ubyte or ushort vec4, values index skin.joints —
            // remapped here to skeleton indices, so downstream never sees
            // glTF numbering.
            {
                const auto& acc = model.accessors[jointsIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++) {
                    const uint8_t* p = data + i * stride;
                    for (int k = 0; k < 4; k++) {
                        int raw =
                            acc.componentType ==
                                    TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT
                                ? reinterpret_cast<const uint16_t*>(p)[k]
                                : p[k];
                        int mapped =
                            raw >= 0 &&
                                    raw < static_cast<int>(jointToSkeleton.size())
                                ? jointToSkeleton[raw]
                                : 0;
                        out.influences[i].joints[k] =
                            static_cast<uint16_t>(std::max(mapped, 0));
                    }
                }
            }

            // WEIGHTS_0: float, or normalized ubyte/ushort. Renormalized so
            // the four weights sum to one (guards sloppy exports).
            {
                const auto& acc = model.accessors[weightsIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++) {
                    const uint8_t* p = data + i * stride;
                    float w[4];
                    for (int k = 0; k < 4; k++) {
                        switch (acc.componentType) {
                            case TINYGLTF_COMPONENT_TYPE_FLOAT:
                                w[k] = reinterpret_cast<const float*>(p)[k];
                                break;
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                                w[k] = reinterpret_cast<const uint16_t*>(p)[k] /
                                       65535.0f;
                                break;
                            default:
                                w[k] = p[k] / 255.0f;
                                break;
                        }
                    }
                    float total = w[0] + w[1] + w[2] + w[3];
                    if (total > 1e-6f)
                        for (int k = 0; k < 4; k++) w[k] /= total;
                    for (int k = 0; k < 4; k++)
                        out.influences[i].weights[k] = w[k];
                }
            }

            // Indices, reversed to the engine's CW front-face winding (glTF
            // is CCW) — same rule as ModelImporter.
            if (prim.indices >= 0) {
                const auto& acc = model.accessors[prim.indices];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                out.bind.indices.resize(acc.count);
                for (size_t i = 0; i < acc.count; i++) {
                    const uint8_t* p = data + i * stride;
                    if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
                        out.bind.indices[i] = *reinterpret_cast<const uint32_t*>(p);
                    else if (acc.componentType ==
                             TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
                        out.bind.indices[i] = *reinterpret_cast<const uint16_t*>(p);
                    else
                        out.bind.indices[i] = *p;
                }
            } else {
                out.bind.indices.resize(vertexCount);
                for (size_t i = 0; i < vertexCount; i++)
                    out.bind.indices[i] = static_cast<uint32_t>(i);
            }
            for (size_t i = 0; i + 2 < out.bind.indices.size(); i += 3)
                std::swap(out.bind.indices[i + 1], out.bind.indices[i + 2]);

            if (prim.material >= 0 &&
                prim.material < static_cast<int>(model.materials.size())) {
                const auto& pbr =
                    model.materials[prim.material].pbrMetallicRoughness;
                out.baseColor = Vec3(pbr.baseColorFactor[0],
                                     pbr.baseColorFactor[1],
                                     pbr.baseColorFactor[2]);
            }
            for (auto& v : out.bind.vertices) v.color = out.baseColor;

            result.meshes.push_back(std::move(out));
        }
    }

    if (result.meshes.empty()) {
        LOG_ERROR << "glTF skin: '" << path << "' has no skinned primitives";
        return {};
    }
    LOG_INFO << "glTF skin: '" << path << "' -> "
             << result.skeleton.jointCount() << " joints, "
             << result.meshes.size() << " skinned mesh(es)";
    return result;
}

}  // namespace anim
}  // namespace engine
