#include <nlohmann/json.hpp>
#define TINYGLTF_NO_INCLUDE_JSON
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include "model_importer.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <functional>
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

// A glTF node's local transform: an explicit column-major matrix, or TRS.
static Mat4 nodeMatrix(const tinygltf::Node& node) {
    if (node.matrix.size() == 16) {
        Mat4 m;
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                m.m[r][c] = static_cast<Real>(node.matrix[c * 4 + r]);
        return m;
    }
    Vec3 t = node.translation.size() == 3
                 ? Vec3(node.translation[0], node.translation[1], node.translation[2])
                 : Vec3();
    Quat q = node.rotation.size() == 4
                 ? Quat(node.rotation[0], node.rotation[1], node.rotation[2],
                        node.rotation[3])
                 : Quat::identity();
    Vec3 s = node.scale.size() == 3
                 ? Vec3(node.scale[0], node.scale[1], node.scale[2])
                 : Vec3(1, 1, 1);
    return Mat4::trs(t, q, s);
}

// Decode a glTF texture index to a CPU image (tinygltf already decoded it).
static CpuImage extractGltfImage(const tinygltf::Model& model, int textureIndex) {
    CpuImage out;
    if (textureIndex < 0) return out;
    const auto& tex = model.textures[textureIndex];
    if (tex.source < 0 || tex.source >= static_cast<int>(model.images.size()))
        return out;
    const auto& img = model.images[tex.source];
    if (img.image.empty() || img.width <= 0 || img.height <= 0) return out;
    out.width = img.width;
    out.height = img.height;
    out.channels = img.component;
    out.pixels = img.image;
    return out;
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

bool ModelImporter::validate(const std::string& path, std::string& error) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model model;
    std::string err, warn;
    bool binary = path.size() > 4 && path.substr(path.size() - 4) == ".glb";
    bool ok = binary ? loader.LoadBinaryFromFile(&model, &err, &warn, path)
                     : loader.LoadASCIIFromFile(&model, &err, &warn, path);
    if (!ok) error = err.empty() ? "unparseable glTF" : err;
    return ok;
}

CpuModel ModelImporter::loadCpu(const std::string& path) {
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

    CpuModel result;

    // Place meshes by the node graph: a glTF mesh sits where its node puts it
    // (the helmet's root node rotates it Z-up -> Y-up). Walk the scene, recording
    // each mesh's accumulated world transform; meshes with no node stay identity.
    std::vector<Mat4> meshWorld(model.meshes.size(), Mat4::identity());
    {
        std::function<void(int, const Mat4&)> walk = [&](int ni, const Mat4& parent) {
            if (ni < 0 || ni >= static_cast<int>(model.nodes.size())) return;
            const auto& node = model.nodes[ni];
            Mat4 world = parent * nodeMatrix(node);
            if (node.mesh >= 0 && node.mesh < static_cast<int>(meshWorld.size()))
                meshWorld[node.mesh] = world;
            for (int c : node.children) walk(c, world);
        };
        int sceneIdx = model.defaultScene >= 0 ? model.defaultScene : 0;
        if (sceneIdx < static_cast<int>(model.scenes.size()))
            for (int root : model.scenes[sceneIdx].nodes) walk(root, Mat4::identity());
    }

    for (size_t meshIdx = 0; meshIdx < model.meshes.size(); ++meshIdx) {
        const auto& mesh = model.meshes[meshIdx];
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
                // glTF front faces are CCW, but the renderer's front-facing winding
                // is CW (MTLWindingClockwise + cull back). Reverse each triangle so
                // imported meshes aren't backface-culled — i.e. don't render inside-out.
                for (size_t i = 0; i + 2 < renderMesh.indices.size(); i += 3)
                    std::swap(renderMesh.indices[i + 1], renderMesh.indices[i + 2]);
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

            // No indices — generate a trivial index buffer, emitting each triangle
            // reversed to match the renderer's CW front-facing winding (glTF is CCW).
            if (renderMesh.indices.empty()) {
                renderMesh.indices.resize(vertexCount);
                for (size_t i = 0; i < vertexCount; i++)
                    renderMesh.indices[i] = static_cast<uint32_t>(i);
                for (size_t i = 0; i + 2 < renderMesh.indices.size(); i += 3)
                    std::swap(renderMesh.indices[i + 1], renderMesh.indices[i + 2]);
            }

            // Material — factors + decoded texture images (CPU; uploaded later).
            CpuMaterial mat;
            if (prim.material >= 0 && prim.material < static_cast<int>(model.materials.size())) {
                const auto& gmat = model.materials[prim.material];
                const auto& pbr = gmat.pbrMetallicRoughness;

                mat.baseColor = Vec3(pbr.baseColorFactor[0],
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

                mat.baseColorTex = extractGltfImage(model, pbr.baseColorTexture.index);
                mat.metallicRoughnessTex =
                    extractGltfImage(model, pbr.metallicRoughnessTexture.index);
                mat.normalTex = extractGltfImage(model, gmat.normalTexture.index);
                mat.emissiveTex = extractGltfImage(model, gmat.emissiveTexture.index);
                mat.aoTex = extractGltfImage(model, gmat.occlusionTexture.index);
            }

            // Bake the node's world transform into the geometry (positions as
            // points, normals/tangents as directions), so every backend gets the
            // model already placed — no scene-graph handling needed downstream.
            const Mat4& W = meshWorld[meshIdx];
            for (Vertex& vtx : renderMesh.vertices) {
                vtx.position = W.transformPoint(vtx.position);
                Vec3 nn = W.transformDirection(vtx.normal);
                if (nn.lengthSquared() > 1e-12) vtx.normal = normalize(nn);
                Vec3 tt = W.transformDirection(vtx.tangent);
                if (tt.lengthSquared() > 1e-12) vtx.tangent = normalize(tt);
            }

            result.meshes.push_back({std::move(renderMesh), std::move(mat)});
        }
    }

    std::cout << "[INFO] Parsed model: " << path
              << " (" << result.meshes.size() << " mesh(es))\n";
    return result;
}

ImportedModel ModelImporter::load(const std::string& path, Renderer& renderer) {
    auto cached = cache.find(path);
    if (cached != cache.end()) return cached->second;

    CpuModel cpu = loadCpu(path);
    std::unordered_map<const std::vector<uint8_t>*, TextureHandle> texCache;
    auto upload = [&](const CpuImage& img) -> TextureHandle {
        if (!img.valid()) return TextureHandle{};
        // Dedup by the underlying pixel buffer (shared glTF images upload once).
        auto it = texCache.find(&img.pixels);
        if (it != texCache.end()) return it->second;
        TextureHandle h = renderer.uploadTexture(img.width, img.height,
                                                 img.channels, img.pixels.data());
        texCache[&img.pixels] = h;
        return h;
    };

    ImportedModel result;
    for (auto& cm : cpu.meshes) {
        RenderMaterial mat;
        mat.albedo = cm.material.baseColor;
        mat.opacity = cm.material.opacity;
        mat.metallic = cm.material.metallic;
        mat.roughness = cm.material.roughness;
        mat.emission = cm.material.emission;
        mat.albedoMap = upload(cm.material.baseColorTex);
        mat.metallicRoughnessMap = upload(cm.material.metallicRoughnessTex);
        mat.normalMap = upload(cm.material.normalTex);
        mat.emissiveMap = upload(cm.material.emissiveTex);
        mat.aoMap = upload(cm.material.aoTex);
        MeshHandle mh = renderer.uploadMesh(cm.geometry);
        result.meshes.push_back({mh, mat});
    }

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

HdrImage EnvironmentLoader::loadHdr(const std::string& path) {
    HdrImage out;
    int w = 0, h = 0, n = 0;
    // Force 3 channels (RGB); stbi_loadf returns linear float for .hdr (RGBE).
    float* data = stbi_loadf(path.c_str(), &w, &h, &n, 3);
    if (!data) {
        std::cerr << "[ERROR] Failed to load HDR environment map: " << path
                  << " (" << stbi_failure_reason() << ")\n";
        return out;
    }
    out.width = w;
    out.height = h;
    out.pixels.assign(data, data + static_cast<size_t>(w) * h * 3);
    stbi_image_free(data);

    // Log-average (geometric mean) luminance for auto-exposure. The log keeps the
    // tiny, enormously-bright sun disc from dominating, so this tracks the sky/
    // ground the eye actually adapts to. δ avoids log(0) on black pixels.
    const size_t pixelCount = static_cast<size_t>(w) * h;
    const float delta = 1e-4f;
    double logSum = 0.0;
    for (size_t i = 0; i < pixelCount; i++) {
        const float* p = out.pixels.data() + i * 3;
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        logSum += std::log(delta + std::max(lum, 0.0f));
    }
    out.avgLuminance = std::exp(static_cast<float>(logSum / static_cast<double>(pixelCount)));

    // Dominant-light extraction (ADR-0017 Phase 2): find the brightest texel,
    // then integrate direction, chromaticity, and illuminance over its
    // luminance neighborhood (≥ half the peak — the sun disc, in practice).
    // An equirect texel at row y subtends solid angle (2π/W)(π/H)·sin(θ), so
    // Σ luminance·ω over the disc is the illuminance it delivers — directly
    // the directional-light intensity in the renderer's units.
    float maxLum = 0.0f;
    for (size_t i = 0; i < pixelCount; i++) {
        const float* p = out.pixels.data() + i * 3;
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        if (lum > maxLum) maxLum = lum;
    }
    // Gate: a real sun is far brighter than the sky around it. Maps with no
    // clear peak (overcast) keep hasSun = false and the level's authored sun.
    if (maxLum > 10.0f && maxLum > 50.0f * out.avgLuminance) {
        const float discThreshold = 0.5f * maxLum;
        const double texelBase = (2.0 * M_PI / w) * (M_PI / h);
        Vec3 dirSum, colorSum;
        double illum = 0.0;
        for (int y = 0; y < h; y++) {
            // Inverse of the shader's equirect mapping: v = acos(dir.y)/π,
            // u = atan2(dir.z, dir.x)/2π + 0.5.
            double theta = (y + 0.5) / h * M_PI;
            double sinT = std::sin(theta), cosT = std::cos(theta);
            double omega = texelBase * sinT;
            for (int x = 0; x < w; x++) {
                const float* p = out.pixels.data() + (static_cast<size_t>(y) * w + x) * 3;
                float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
                if (lum < discThreshold) continue;
                double phi = ((x + 0.5) / w - 0.5) * 2.0 * M_PI;
                Vec3 dir(sinT * std::cos(phi), cosT, sinT * std::sin(phi));
                double e = lum * omega;
                dirSum   += dir * e;
                colorSum += Vec3(p[0], p[1], p[2]) * omega;
                illum    += e;
            }
        }
        if (illum > 0.0) {
            out.hasSun = true;
            out.sunDirection = normalize(dirSum);
            out.sunIntensity = static_cast<float>(illum);
            Real colorLum = 0.2126 * colorSum.x + 0.7152 * colorSum.y + 0.0722 * colorSum.z;
            out.sunColor = (colorLum > 0.0) ? colorSum / colorLum : Vec3(1, 1, 1);
        }
    }

    std::cout << "[INFO] Loaded HDR environment: " << path
              << " (" << w << "x" << h << "), avg luminance "
              << out.avgLuminance << "\n";
    if (out.hasSun) {
        std::cout << "[INFO] HDR sun extracted: dir (" << out.sunDirection.x
                  << ", " << out.sunDirection.y << ", " << out.sunDirection.z
                  << "), intensity " << out.sunIntensity << "\n";
    }
    return out;
}

TextureHandle EnvironmentLoader::loadEnvironmentMap(const std::string& path,
                                                    Renderer& renderer,
                                                    DirectionalLight* outSun) {
    HdrImage img = loadHdr(path);
    if (!img.valid()) return TextureHandle{};
    renderer.environmentAvgLuminance = img.avgLuminance;  // for auto-exposure
    if (outSun && img.hasSun) {
        outSun->direction = img.sunDirection;
        outSun->color = img.sunColor;
        outSun->intensity = img.sunIntensity;
    }
    return renderer.uploadTextureHDR(img.width, img.height, 3, img.pixels.data());
}

}  // namespace engine
