#include "asset_manager.h"
#include "mesh_builder.h"
#include "../log.h"
#include "../profile.h"
#include <cstdio>

namespace engine {

std::string AssetManager::primitiveKey(const std::string& shape, Vec3 size) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s:%g,%g,%g", shape.c_str(),
                  static_cast<double>(size.x), static_cast<double>(size.y),
                  static_cast<double>(size.z));
    return std::string(buf);
}

MeshHandle AssetManager::acquirePrimitive(const std::string& shape, Vec3 size) {
    const std::string key = primitiveKey(shape, size);
    auto it = byKey_.find(key);
    if (it != byKey_.end()) {        // cache hit: share, don't rebuild or upload
        records_[it->second].refs++;
        return it->second;
    }
    RenderMesh mesh = MeshBuilder::shape(shape, size);
    if (mesh.vertices.empty()) return MeshHandle{};   // unknown shape: no upload
    return acquireMesh(mesh, key);
}

MeshHandle AssetManager::acquireMesh(const RenderMesh& mesh, const std::string& key) {
    RT_PROFILE_ZONE_NAMED("acquireMesh");
    if (!key.empty()) {
        auto it = byKey_.find(key);
        if (it != byKey_.end()) {
            records_[it->second].refs++;
            return it->second;
        }
    }
    MeshHandle handle = uploader_.uploadMesh(mesh);
    // A backend may REFUSE an upload — an empty mesh has no buffers to make
    // (see MetalRenderer::uploadMesh). Don't record the null handle: every
    // refused mesh would share the one record, so its refcount would count
    // strangers and releaseMesh would free it out from under them. Say which
    // asset it was; the key is the only thing here that names the producer.
    if (!handle.valid()) {
        LOG_WARN << "[assets] mesh '" << (key.empty() ? "<unkeyed>" : key)
                 << "' was not uploaded (" << mesh.vertices.size() << " verts, "
                 << mesh.indices.size() << " indices) — it will not draw";
        return handle;
    }
    MeshRecord rec;
    rec.handle = handle;
    rec.bounds = uploader_.getMeshBounds(handle);
    rec.refs = 1;
    rec.key = key;
    records_[handle] = rec;
    if (!key.empty()) byKey_[key] = handle;
    return handle;
}

MeshHandle AssetManager::retain(MeshHandle handle) {
    auto it = records_.find(handle);
    if (it != records_.end()) it->second.refs++;
    return handle;
}

void AssetManager::releaseMesh(MeshHandle handle) {
    if (!handle.valid()) return;
    auto it = records_.find(handle);
    if (it == records_.end()) return;
    if (--it->second.refs > 0) return;
    if (!it->second.key.empty()) byKey_.erase(it->second.key);
    uploader_.removeMesh(handle);
    records_.erase(it);
}

BoundingSphere AssetManager::meshBounds(MeshHandle handle) const {
    auto it = records_.find(handle);
    return it != records_.end() ? it->second.bounds : BoundingSphere{};
}

void AssetManager::clear() {
    for (const auto& entry : records_) uploader_.removeMesh(entry.second.handle);
    records_.clear();
    byKey_.clear();
}

int AssetManager::refCount(MeshHandle handle) const {
    auto it = records_.find(handle);
    return it != records_.end() ? it->second.refs : 0;
}

}  // namespace engine
