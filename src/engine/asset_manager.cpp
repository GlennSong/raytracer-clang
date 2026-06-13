#include "asset_manager.h"
#include "mesh_builder.h"
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
    return acquireMesh(MeshBuilder::shape(shape, size), key);
}

MeshHandle AssetManager::acquireMesh(const RenderMesh& mesh, const std::string& key) {
    if (!key.empty()) {
        auto it = byKey_.find(key);
        if (it != byKey_.end()) {
            records_[it->second].refs++;
            return it->second;
        }
    }
    MeshHandle handle = uploader_.uploadMesh(mesh);
    MeshRecord rec;
    rec.handle = handle;
    rec.bounds = uploader_.getMeshBounds(handle);
    rec.refs = 1;
    rec.key = key;
    records_[handle] = rec;
    if (!key.empty()) byKey_[key] = handle;
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
