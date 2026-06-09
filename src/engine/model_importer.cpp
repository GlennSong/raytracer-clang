#include <nlohmann/json.hpp>
#define TINYGLTF_NO_INCLUDE_JSON
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include "model_importer.h"
#include <cstring>
#include <algorithm>
#include <iostream>

namespace engine {

std::unordered_map<std::string, ImportedModel> ModelImporter::cache;

static const uint8_t* accessorData(const tinygltf::Model& model,
                                   const tinygltf::Accessor& accessor) {
    const auto& bv = model.bufferViews[accessor.bufferView];
    return model.buffers[bv.buffer].data.data() + bv.byteOffset + accessor.byteOffset;
}

static size_t accessorStride(const tinygltf::Model& model,
                             const tinygltf::Accessor& accessor) {
    const auto& bv = model.bufferViews[accessor.bufferView];
    if (bv.byteStride > 0) return bv.byteStride;
    int componentSize = tinygltf::GetComponentSizeInBytes(
        static_cast<uint32_t>(accessor.componentType));
    int numComponents = tinygltf::GetNumComponentsInType(
        static_cast<uint32_t>(accessor.type));
    return static_cast<size_t>(componentSize * numComponents);
}

static Vec3 readVec3(const uint8_t* ptr) {
    const float* f = reinterpret_cast<const float*>(ptr);
    return Vec3(f[0], f[1], f[2]);
}

static void readVec2(const uint8_t* ptr, float& u, float& v) {
    const float* f = reinterpret_cast<const float*>(ptr);
    u = f[0];
    v = f[1];
}

static TextureHandle uploadGltfTexture(const tinygltf::Model& model,
                                       int textureIndex,
                                       Renderer& renderer,
                                       std::unordered_map<int, TextureHandle>& texCache) {
    if (textureIndex < 0) return TextureHandle{};

    auto it = texCache.find(textureIndex);
    if (it != texCache.end()) return it->second;

    const auto& tex = model.textures[textureIndex];
    if (tex.source < 0 || tex.source >= static_cast<int>(model.images.size()))
        return TextureHandle{};

    const auto& img = model.images[tex.source];
    if (img.image.empty() || img.width <= 0 || img.height <= 0)
        return TextureHandle{};

    TextureHandle handle = renderer.uploadTexture(
        img.width, img.height, img.component, img.image.data());
    texCache[textureIndex] = handle;
    return handle;
}

static void computeTangents(std::vector<Vertex>& vertices,
                            const std::vector<uint32_t>& indices) {
    std::vector<Vec3> tan(vertices.size());

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        uint32_t i0 = indices[i], i1 = indices[i+1], i2 = indices[i+2];
        const Vertex& v0 = vertices[i0];
        const Vertex& v1 = vertices[i1];
        const Vertex& v2 = vertices[i2];

        Vec3 edge1 = v1.position - v0.position;
        Vec3 edge2 = v2.position - v0.position;
        float du1 = v1.u - v0.u, dv1 = v1.v - v0.v;
        float du2 = v2.u - v0.u, dv2 = v2.v - v0.v;

        float det = du1 * dv2 - du2 * dv1;
        if (std::abs(det) < 1e-8f) continue;
        float invDet = 1.0f / det;

        Vec3 t = (edge1 * dv2 - edge2 * dv1) * invDet;
        tan[i0] += t;
        tan[i1] += t;
        tan[i2] += t;
    }

    for (size_t i = 0; i < vertices.size(); i++) {
        Vec3 n = vertices[i].normal;
        Vec3 t = tan[i];
        t = t - n * dot(n, t);
        if (t.lengthSquared() > 1e-8)
            vertices[i].tangent = normalize(t);
        else
            vertices[i].tangent = Vec3(1, 0, 0);
    }
}

ImportedModel ModelImporter::load(const std::string& path, Renderer& renderer) {
    auto cached = cache.find(path);
    if (cached != cache.end()) return cached->second;

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;

    bool ok = false;
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".glb") {
        ok = loader.LoadBinaryFromFile(&model, &err, &warn, path);
    } else {
        ok = loader.LoadASCIIFromFile(&model, &err, &warn, path);
    }

    if (!warn.empty()) std::cerr << "[WARN] glTF: " << warn << "\n";
    if (!err.empty()) std::cerr << "[ERROR] glTF: " << err << "\n";

    if (!ok) {
        std::cerr << "[ERROR] Failed to load: " << path << "\n";
        return {};
    }

    std::unordered_map<int, TextureHandle> texCache;
    ImportedModel result;

    for (const auto& mesh : model.meshes) {
        for (const auto& prim : mesh.primitives) {
            if (prim.mode != -1 && prim.mode != TINYGLTF_MODE_TRIANGLES)
                continue;

            RenderMesh renderMesh;

            // Indices
            if (prim.indices >= 0) {
                const auto& acc = model.accessors[prim.indices];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                renderMesh.indices.resize(acc.count);
                for (size_t i = 0; i < acc.count; i++) {
                    const uint8_t* p = data + i * stride;
                    if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
                        renderMesh.indices[i] = *reinterpret_cast<const uint32_t*>(p);
                    else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
                        renderMesh.indices[i] = *reinterpret_cast<const uint16_t*>(p);
                    else if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE)
                        renderMesh.indices[i] = *p;
                }
            }

            // Position (required)
            auto posIt = prim.attributes.find("POSITION");
            if (posIt == prim.attributes.end()) continue;
            const auto& posAcc = model.accessors[posIt->second];
            size_t vertexCount = posAcc.count;
            renderMesh.vertices.resize(vertexCount);

            const uint8_t* posData = accessorData(model, posAcc);
            size_t posStride = accessorStride(model, posAcc);
            for (size_t i = 0; i < vertexCount; i++)
                renderMesh.vertices[i].position = readVec3(posData + i * posStride);

            // Normal
            auto normIt = prim.attributes.find("NORMAL");
            if (normIt != prim.attributes.end()) {
                const auto& acc = model.accessors[normIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++)
                    renderMesh.vertices[i].normal = readVec3(data + i * stride);
            }

            // Tangent
            bool hasTangent = false;
            auto tanIt = prim.attributes.find("TANGENT");
            if (tanIt != prim.attributes.end()) {
                const auto& acc = model.accessors[tanIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++)
                    renderMesh.vertices[i].tangent = readVec3(data + i * stride);
                hasTangent = true;
            }

            // Texcoord
            auto uvIt = prim.attributes.find("TEXCOORD_0");
            if (uvIt != prim.attributes.end()) {
                const auto& acc = model.accessors[uvIt->second];
                const uint8_t* data = accessorData(model, acc);
                size_t stride = accessorStride(model, acc);
                for (size_t i = 0; i < vertexCount; i++)
                    readVec2(data + i * stride,
                             renderMesh.vertices[i].u,
                             renderMesh.vertices[i].v);
            }

            if (!hasTangent && !renderMesh.indices.empty())
                computeTangents(renderMesh.vertices, renderMesh.indices);

            // No indices — generate trivial index buffer
            if (renderMesh.indices.empty()) {
                renderMesh.indices.resize(vertexCount);
                for (size_t i = 0; i < vertexCount; i++)
                    renderMesh.indices[i] = static_cast<uint32_t>(i);
            }

            // Material
            RenderMaterial mat;
            if (prim.material >= 0 && prim.material < static_cast<int>(model.materials.size())) {
                const auto& gmat = model.materials[prim.material];
                const auto& pbr = gmat.pbrMetallicRoughness;

                mat.albedo = Vec3(pbr.baseColorFactor[0],
                                  pbr.baseColorFactor[1],
                                  pbr.baseColorFactor[2]);
                mat.opacity = static_cast<float>(pbr.baseColorFactor[3]);
                mat.metallic = static_cast<float>(pbr.metallicFactor);
                mat.roughness = static_cast<float>(pbr.roughnessFactor);

                if (!gmat.emissiveFactor.empty() && gmat.emissiveFactor.size() >= 3) {
                    mat.emission = Vec3(gmat.emissiveFactor[0],
                                        gmat.emissiveFactor[1],
                                        gmat.emissiveFactor[2]);
                }

                mat.albedoMap = uploadGltfTexture(
                    model, pbr.baseColorTexture.index, renderer, texCache);
                mat.metallicRoughnessMap = uploadGltfTexture(
                    model, pbr.metallicRoughnessTexture.index, renderer, texCache);
                mat.normalMap = uploadGltfTexture(
                    model, gmat.normalTexture.index, renderer, texCache);
                mat.emissiveMap = uploadGltfTexture(
                    model, gmat.emissiveTexture.index, renderer, texCache);
                mat.aoMap = uploadGltfTexture(
                    model, gmat.occlusionTexture.index, renderer, texCache);
            }

            MeshHandle mh = renderer.uploadMesh(renderMesh);
            result.meshes.push_back({mh, mat});
        }
    }

    std::cout << "[INFO] Loaded model: " << path
              << " (" << result.meshes.size() << " mesh(es))\n";

    cache[path] = result;
    return result;
}

ImportedModel* ModelImporter::getCached(const std::string& path) {
    auto it = cache.find(path);
    return it != cache.end() ? &it->second : nullptr;
}

void ModelImporter::clearCache() {
    cache.clear();
}

}  // namespace engine
