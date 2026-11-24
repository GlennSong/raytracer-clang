#include "engine/glb_export.h"
#include "engine/text/font.h"   // TextImage

#include <tinygltf/tiny_gltf.h>
#include <tinygltf/stb_image_write.h>

#include <algorithm>
#include <cstring>
#include <map>

namespace engine {

namespace {
tinygltf::Value toTinyValue(const nlohmann::json& j) {
    using tinygltf::Value;
    switch (j.type()) {
        case nlohmann::json::value_t::object: { Value::Object o; for (auto it = j.begin(); it != j.end(); ++it) o[it.key()] = toTinyValue(it.value()); return Value(o); }
        case nlohmann::json::value_t::array: { Value::Array a; for (const nlohmann::json& e : j) a.push_back(toTinyValue(e)); return Value(a); }
        case nlohmann::json::value_t::string: return Value(j.get<std::string>());
        case nlohmann::json::value_t::boolean: return Value(j.get<bool>());
        case nlohmann::json::value_t::number_integer: return Value(static_cast<int>(j.get<int64_t>()));
        case nlohmann::json::value_t::number_unsigned: { const uint64_t u = j.get<uint64_t>(); return u <= 0x7fffffffu ? Value(static_cast<int>(u)) : Value(static_cast<double>(u)); }
        case nlohmann::json::value_t::number_float: return Value(j.get<double>());
        default: return Value();
    }
}
}  // namespace

bool writeGlb(const std::vector<GlbEntry>& entries, const std::string& path, const nlohmann::json& sceneExtras, std::string* error) {
    tinygltf::Model model; model.asset.version = "2.0"; model.asset.generator = "raytracer-clang";
    tinygltf::Buffer buffer; tinygltf::Scene scene;
    auto align4 = [&]() { while (buffer.data.size() % 4) buffer.data.push_back(0); };
    auto addView = [&](const void* data, size_t bytes, int target) {
        align4(); tinygltf::BufferView bv; bv.buffer = 0; bv.byteOffset = buffer.data.size(); bv.byteLength = bytes; bv.target = target;
        const unsigned char* b = static_cast<const unsigned char*>(data); buffer.data.insert(buffer.data.end(), b, b + bytes);
        model.bufferViews.push_back(bv); return static_cast<int>(model.bufferViews.size() - 1);
    };
    auto addAccessor = [&](int view, int compType, int type, size_t count) {
        tinygltf::Accessor a; a.bufferView = view; a.byteOffset = 0; a.componentType = compType; a.type = type; a.count = count; model.accessors.push_back(a); return static_cast<int>(model.accessors.size() - 1);
    };
    // each picture once, as PNG bytes in the buffer, in the order entries first name it
    std::map<const TextImage*, int> textureOf;
    auto textureFor = [&](const TextImage* img) {
        const std::map<const TextImage*, int>::const_iterator found = textureOf.find(img);
        if (found != textureOf.end()) return found->second;
        std::vector<unsigned char> png;
        stbi_write_png_to_func([](void* ctx, void* data, int size) {
            std::vector<unsigned char>* out = static_cast<std::vector<unsigned char>*>(ctx);
            const unsigned char* b = static_cast<const unsigned char*>(data);
            out->insert(out->end(), b, b + size);
        }, &png, img->w, img->h, 4, img->rgba.data(), img->w * 4);
        tinygltf::Image image; image.name = "page" + std::to_string(model.images.size()); image.mimeType = "image/png";
        image.bufferView = addView(png.data(), png.size(), 0);
        model.images.push_back(image);
        tinygltf::Sampler sampler; sampler.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR; sampler.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR;
        sampler.wrapS = sampler.wrapT = TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE;
        model.samplers.push_back(sampler);
        tinygltf::Texture tex; tex.source = static_cast<int>(model.images.size() - 1); tex.sampler = static_cast<int>(model.samplers.size() - 1);
        model.textures.push_back(tex);
        const int id = static_cast<int>(model.textures.size() - 1);
        textureOf.emplace(img, id);
        return id;
    };
    for (const GlbEntry& e : entries) {
        if (!e.mesh) continue; const RenderMesh& m = *e.mesh; if (m.vertices.empty() || m.indices.empty()) continue;
        const size_t n = m.vertices.size();
        std::vector<float> pos(3 * n), nrm(3 * n), tan(4 * n), uv(2 * n), col; float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
        bool uniform = true; const Vec3 c0 = m.vertices[0].color;
        for (size_t i = 0; i < n; ++i) {
            const Vertex& v = m.vertices[i];
            const float p[3] = {static_cast<float>(v.position.x), static_cast<float>(v.position.y), static_cast<float>(v.position.z)};
            for (int k = 0; k < 3; ++k) { pos[3 * i + static_cast<size_t>(k)] = p[k]; mn[k] = std::min(mn[k], p[k]); mx[k] = std::max(mx[k], p[k]); }
            nrm[3 * i] = static_cast<float>(v.normal.x); nrm[3 * i + 1] = static_cast<float>(v.normal.y); nrm[3 * i + 2] = static_cast<float>(v.normal.z);
            tan[4 * i] = static_cast<float>(v.tangent.x); tan[4 * i + 1] = static_cast<float>(v.tangent.y); tan[4 * i + 2] = static_cast<float>(v.tangent.z); tan[4 * i + 3] = 1.0f;
            uv[2 * i] = v.u; uv[2 * i + 1] = v.v;
            if (uniform && (v.color.x != c0.x || v.color.y != c0.y || v.color.z != c0.z)) uniform = false;
        }
        if (!uniform) { col.resize(3 * n); for (size_t i = 0; i < n; ++i) { const Vec3& c = m.vertices[i].color; col[3 * i] = static_cast<float>(c.x); col[3 * i + 1] = static_cast<float>(c.y); col[3 * i + 2] = static_cast<float>(c.z); } }
        const int vPos = addView(pos.data(), pos.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), vNrm = addView(nrm.data(), nrm.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER);
        const int vTan = addView(tan.data(), tan.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), vUv = addView(uv.data(), uv.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER);
        const int vIdx = addView(m.indices.data(), m.indices.size() * 4, TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER);
        const int aPos = addAccessor(vPos, TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, n); model.accessors.back().minValues = {mn[0], mn[1], mn[2]}; model.accessors.back().maxValues = {mx[0], mx[1], mx[2]};
        const int aNrm = addAccessor(vNrm, TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, n);
        const int aTan = addAccessor(vTan, TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC4, n);
        const int aUv = addAccessor(vUv, TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC2, n);
        const int aIdx = addAccessor(vIdx, TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT, TINYGLTF_TYPE_SCALAR, m.indices.size());
        tinygltf::Material mat; mat.name = e.name; mat.pbrMetallicRoughness.baseColorFactor = {e.color.x, e.color.y, e.color.z, 1.0}; mat.pbrMetallicRoughness.metallicFactor = e.metallic; mat.pbrMetallicRoughness.roughnessFactor = e.roughness; mat.doubleSided = true;
        if (e.texture && e.texture->w > 0) mat.pbrMetallicRoughness.baseColorTexture.index = textureFor(e.texture);
        if (e.alphaCut) { mat.alphaMode = "MASK"; mat.alphaCutoff = 0.5; }
        if (e.materialExtras.is_object() && !e.materialExtras.empty()) mat.extras = toTinyValue(e.materialExtras);
        model.materials.push_back(mat);
        tinygltf::Primitive prim; prim.attributes["POSITION"] = aPos; prim.attributes["NORMAL"] = aNrm; prim.attributes["TANGENT"] = aTan; prim.attributes["TEXCOORD_0"] = aUv;
        if (!uniform) { const int vCol = addView(col.data(), col.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER); prim.attributes["COLOR_0"] = addAccessor(vCol, TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, n); }
        prim.indices = aIdx; prim.material = static_cast<int>(model.materials.size() - 1); prim.mode = TINYGLTF_MODE_TRIANGLES;
        tinygltf::Mesh mesh; mesh.name = e.name; mesh.primitives.push_back(prim); model.meshes.push_back(mesh);
        tinygltf::Node node; node.name = e.name; node.mesh = static_cast<int>(model.meshes.size() - 1);
        if (e.extras.is_object() && !e.extras.empty()) node.extras = toTinyValue(e.extras);
        model.nodes.push_back(node); scene.nodes.push_back(static_cast<int>(model.nodes.size() - 1));
    }
    if (sceneExtras.is_object() && !sceneExtras.empty()) scene.extras = toTinyValue(sceneExtras);
    model.buffers.push_back(buffer); model.scenes.push_back(scene); model.defaultScene = 0;
    tinygltf::TinyGLTF gltf; const bool ok = gltf.WriteGltfSceneToFile(&model, path, false, true, false, true);
    if (!ok && error) *error = "tinygltf refused to write " + path;
    return ok;
}

bool writeGlbTree(const std::vector<GlbTreeNode>& nodes, const std::vector<GlbTreeMaterial>& materials, const std::string& path,
                  const nlohmann::json& sceneExtras, std::string* error) {
    tinygltf::Model model; model.asset.version = "2.0"; model.asset.generator = "raytracer-clang";
    tinygltf::Buffer buffer; tinygltf::Scene scene;
    auto addView = [&](const void* data, size_t bytes, int target) {
        while (buffer.data.size() % 4) buffer.data.push_back(0);
        tinygltf::BufferView bv; bv.buffer = 0; bv.byteOffset = buffer.data.size(); bv.byteLength = bytes; bv.target = target;
        const unsigned char* b = static_cast<const unsigned char*>(data); buffer.data.insert(buffer.data.end(), b, b + bytes);
        model.bufferViews.push_back(bv); return static_cast<int>(model.bufferViews.size() - 1);
    };
    auto addAccessor = [&](int view, int compType, int type, size_t count) {
        tinygltf::Accessor a; a.bufferView = view; a.byteOffset = 0; a.componentType = compType; a.type = type; a.count = count;
        model.accessors.push_back(a); return static_cast<int>(model.accessors.size() - 1);
    };
    auto addImage = [&](int w, int h, int channels, const std::vector<uint8_t>& pixels) {
        std::vector<unsigned char> png;
        stbi_write_png_to_func([](void* ctx, void* data, int size) {
            std::vector<unsigned char>* out = static_cast<std::vector<unsigned char>*>(ctx);
            const unsigned char* b = static_cast<const unsigned char*>(data);
            out->insert(out->end(), b, b + size);
        }, &png, w, h, channels, pixels.data(), w * channels);
        tinygltf::Image image; image.name = "map" + std::to_string(model.images.size()); image.mimeType = "image/png";
        image.bufferView = addView(png.data(), png.size(), 0);
        model.images.push_back(image);
        if (model.samplers.empty()) {
            tinygltf::Sampler sampler; sampler.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR; sampler.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR;
            sampler.wrapS = sampler.wrapT = TINYGLTF_TEXTURE_WRAP_REPEAT;
            model.samplers.push_back(sampler);
        }
        tinygltf::Texture tex; tex.source = static_cast<int>(model.images.size() - 1); tex.sampler = 0;
        model.textures.push_back(tex);
        return static_cast<int>(model.textures.size() - 1);
    };
    for (const GlbTreeMaterial& m : materials) {
        tinygltf::Material mat; mat.name = m.name;
        mat.pbrMetallicRoughness.baseColorFactor = {m.color.x, m.color.y, m.color.z, static_cast<double>(m.opacity)};
        mat.pbrMetallicRoughness.metallicFactor = m.metallic; mat.pbrMetallicRoughness.roughnessFactor = m.roughness;
        mat.emissiveFactor = {m.emission.x, m.emission.y, m.emission.z};
        mat.doubleSided = m.twoSided;
        if (m.opacity < 1.0f) mat.alphaMode = "BLEND";
        if (m.albedo != nullptr && !m.albedo->empty()) mat.pbrMetallicRoughness.baseColorTexture.index = addImage(m.albedoW, m.albedoH, m.albedoChannels, *m.albedo);
        if (m.normal != nullptr && !m.normal->empty()) mat.normalTexture.index = addImage(m.normalW, m.normalH, m.normalChannels, *m.normal);
        if (m.extras.is_object() && !m.extras.empty()) mat.extras = toTinyValue(m.extras);
        model.materials.push_back(mat);
    }
    std::vector<int> meshOfNode(nodes.size(), -1);
    for (std::size_t ni = 0; ni < nodes.size(); ++ni) {
        const GlbTreeNode& n = nodes[ni];
        tinygltf::Node node; node.name = n.name;
        if (n.translation.x != 0 || n.translation.y != 0 || n.translation.z != 0) node.translation = {n.translation.x, n.translation.y, n.translation.z};
        if (n.rotation.x != 0 || n.rotation.y != 0 || n.rotation.z != 0 || n.rotation.w != 1) node.rotation = {n.rotation.x, n.rotation.y, n.rotation.z, n.rotation.w};
        if (n.scale != 1) node.scale = {n.scale, n.scale, n.scale};
        if (n.meshOf >= 0 && n.meshOf < static_cast<int>(ni)) {
            meshOfNode[ni] = meshOfNode[static_cast<std::size_t>(n.meshOf)];
        } else if (!n.primitives.empty()) {
            tinygltf::Mesh mesh; mesh.name = n.name;
            for (const GlbTreePrimitive& p : n.primitives) {
                if (p.mesh == nullptr || p.mesh->vertices.empty() || p.mesh->indices.empty()) continue;
                const RenderMesh& m = *p.mesh;
                const size_t count = m.vertices.size();
                std::vector<float> pos(3 * count), nrm(3 * count), tan(4 * count), uv(2 * count), col(3 * count);
                float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
                for (size_t i = 0; i < count; ++i) {
                    const Vertex& v = m.vertices[i];
                    const float q[3] = {static_cast<float>(v.position.x), static_cast<float>(v.position.y), static_cast<float>(v.position.z)};
                    for (int k = 0; k < 3; ++k) { pos[3 * i + static_cast<size_t>(k)] = q[k]; mn[k] = std::min(mn[k], q[k]); mx[k] = std::max(mx[k], q[k]); }
                    nrm[3 * i] = static_cast<float>(v.normal.x); nrm[3 * i + 1] = static_cast<float>(v.normal.y); nrm[3 * i + 2] = static_cast<float>(v.normal.z);
                    tan[4 * i] = static_cast<float>(v.tangent.x); tan[4 * i + 1] = static_cast<float>(v.tangent.y); tan[4 * i + 2] = static_cast<float>(v.tangent.z); tan[4 * i + 3] = 1.0f;
                    uv[2 * i] = v.u; uv[2 * i + 1] = v.v;
                    col[3 * i] = static_cast<float>(v.color.x); col[3 * i + 1] = static_cast<float>(v.color.y); col[3 * i + 2] = static_cast<float>(v.color.z);
                }
                // glTF's triangles are counter-clockwise; the engine's are clockwise
                std::vector<uint32_t> idx = m.indices;
                for (size_t i = 0; i + 2 < idx.size(); i += 3) std::swap(idx[i + 1], idx[i + 2]);
                tinygltf::Primitive prim;
                prim.attributes["POSITION"] = addAccessor(addView(pos.data(), pos.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, count);
                model.accessors.back().minValues = {mn[0], mn[1], mn[2]}; model.accessors.back().maxValues = {mx[0], mx[1], mx[2]};
                prim.attributes["NORMAL"] = addAccessor(addView(nrm.data(), nrm.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, count);
                prim.attributes["TANGENT"] = addAccessor(addView(tan.data(), tan.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC4, count);
                prim.attributes["TEXCOORD_0"] = addAccessor(addView(uv.data(), uv.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC2, count);
                prim.attributes["COLOR_0"] = addAccessor(addView(col.data(), col.size() * 4, TINYGLTF_TARGET_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_FLOAT, TINYGLTF_TYPE_VEC3, count);
                prim.indices = addAccessor(addView(idx.data(), idx.size() * 4, TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER), TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT, TINYGLTF_TYPE_SCALAR, idx.size());
                prim.material = p.material >= 0 && p.material < static_cast<int>(materials.size()) ? p.material : -1;
                prim.mode = TINYGLTF_MODE_TRIANGLES;
                mesh.primitives.push_back(prim);
            }
            if (!mesh.primitives.empty()) { model.meshes.push_back(mesh); meshOfNode[ni] = static_cast<int>(model.meshes.size() - 1); }
        }
        node.mesh = meshOfNode[ni];
        if (n.extras.is_object() && !n.extras.empty()) node.extras = toTinyValue(n.extras);
        model.nodes.push_back(node);
        if (n.parent >= 0 && n.parent < static_cast<int>(ni)) model.nodes[static_cast<std::size_t>(n.parent)].children.push_back(static_cast<int>(ni));
        else scene.nodes.push_back(static_cast<int>(ni));
    }
    if (sceneExtras.is_object() && !sceneExtras.empty()) scene.extras = toTinyValue(sceneExtras);
    model.buffers.push_back(buffer); model.scenes.push_back(scene); model.defaultScene = 0;
    tinygltf::TinyGLTF gltf; const bool ok = gltf.WriteGltfSceneToFile(&model, path, false, true, false, true);
    if (!ok && error) *error = "tinygltf refused to write " + path;
    return ok;
}

}  // namespace engine
