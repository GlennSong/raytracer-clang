#include "asset_manager.h"
#include "mesh_builder.h"
#include "../profile.h"

#include <algorithm>
#include <map>
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
    MeshRecord rec;
    rec.handle = handle;
    rec.bounds = uploader_.getMeshBounds(handle);
    rec.refs = 1;
    rec.key = key;
    rec.bytes = mesh.vertices.size() * 56 + mesh.indices.size() * 4;
    records_[handle] = rec;
    if (!key.empty()) byKey_[key] = handle;
    return handle;
}

std::vector<AssetManager::MeshGroup> AssetManager::meshBytesByPrefix() const {
    std::map<std::string, MeshGroup> groups;
    for (const auto& kv : records_) {
        const std::string& k = kv.second.key;
        // every run of digits becomes '#': "road:3:deck:12" -> "road:#:deck:#"
        std::string p;
        for (std::size_t i = 0; i < k.size(); ++i) {
            if (k[i] >= '0' && k[i] <= '9') {
                if (p.empty() || p.back() != '#') p += '#';
            } else p += k[i];
        }
        if (k.empty()) p = "(unkeyed)";
        MeshGroup& g = groups[p];
        g.prefix = p;
        ++g.meshes;
        g.bytes += kv.second.bytes;
    }
    std::vector<MeshGroup> out;
    for (auto& kv : groups) out.push_back(kv.second);
    std::sort(out.begin(), out.end(), [](const MeshGroup& a, const MeshGroup& b) { return a.bytes > b.bytes; });
    return out;
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
