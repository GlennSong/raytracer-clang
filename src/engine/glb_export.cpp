#include "engine/glb_export.h"

#include <tinygltf/tiny_gltf.h>

#include <algorithm>
#include <cstring>

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
        tinygltf::Material mat; mat.name = e.name; mat.pbrMetallicRoughness.baseColorFactor = {e.color.x, e.color.y, e.color.z, 1.0}; mat.pbrMetallicRoughness.metallicFactor = 0.0; mat.pbrMetallicRoughness.roughnessFactor = e.roughness; mat.doubleSided = true;
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

}  // namespace engine
