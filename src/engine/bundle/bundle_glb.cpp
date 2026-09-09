#include "engine/bundle/bundle_glb.h"

#include "engine/bundle/codecs.h"
#include "engine/glb_export.h"
#include "engine/mesh_builder.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>

namespace engine {
namespace bundle {

namespace {
struct MeshSection { std::string name, material; int cx = 0, cz = 0; };

// ".../cell/<cx>_<cz>/mesh/<material>" -> cell and material; false for anything else.
bool parseMeshSection(const std::string& name, MeshSection& out) {
    const size_t cell = name.find("/cell/"); const size_t mesh = name.find("/mesh/");
    if (cell == std::string::npos || mesh == std::string::npos || mesh < cell) return false;
    const std::string key = name.substr(cell + 6, mesh - (cell + 6)); const size_t us = key.find('_');
    if (us == std::string::npos) return false;
    try { out.cx = std::stoi(key.substr(0, us)); out.cz = std::stoi(key.substr(us + 1)); } catch (const std::exception&) { return false; }
    out.name = name; out.material = name.substr(mesh + 6); return !out.material.empty();
}

std::vector<MeshSection> meshSections(const Bundle& b) {
    std::vector<MeshSection> out; for (const std::string& s : b.sections("")) { MeshSection m; if (parseMeshSection(s, m)) out.push_back(m); }
    return out;
}

nlohmann::json sceneExtras(const Bundle& b) {
    nlohmann::json x; x["manifest"] = b.manifest(); nlohmann::json roads = nlohmann::json::object(), holes = nlohmann::json::object(), small = nlohmann::json::object();
    for (const std::string& s : b.sections("")) {
        const Bundle::View v = b.section(s);
        if (s.size() >= 11 && s.compare(s.size() - 11, 11, "/roads/twin") == 0) { RoadEntity e; BinReader r(v.data, v.size); if (getRoadEntity(r, e)) roads[s] = roadNetToJson(e); continue; }
        if (s.size() >= 13 && s.compare(s.size() - 13, 13, "/blocks/holes") == 0) {
            std::vector<std::vector<Vec2>> rings; BinReader r(v.data, v.size);
            if (getRings(r, rings)) { nlohmann::json arr = nlohmann::json::array(); for (const auto& ring : rings) { nlohmann::json rj = nlohmann::json::array(); for (const Vec2& q : ring) rj.push_back({q.x, q.y}); arr.push_back(rj); } holes[s] = arr; }
            continue;
        }
        if (v.size < 65536 && v.size > 0 && v.data[0] == '{') { const nlohmann::json j = b.json(s); if (!j.is_null()) small[s] = j; }
    }
    x["roads"] = roads; x["holes"] = holes; x["json"] = small; return x;
}

nlohmann::json nodeExtras(const PackedMesh& p, size_t triangles) {
    return {{"material", p.name}, {"collidable", p.collidable()}, {"paint", p.paint()}, {"friction", p.friction}, {"roughness", p.roughness}, {"triangles", triangles}};
}
}  // namespace

bool writeBundleGlb(const Bundle& b, const std::string& path, std::string* err, const GlbProgressFn* progress) {
    std::map<std::string, RenderMesh> merged; std::map<std::string, PackedMesh> first; std::map<std::string, size_t> tris;
    const std::vector<MeshSection> secs = meshSections(b); size_t done = 0;
    for (const MeshSection& s : secs) {
        if (progress && !(*progress)(0.8 * static_cast<double>(done++) / std::max<size_t>(1, secs.size()))) { if (err) *err = "cancelled"; return false; }
        const Bundle::View v = b.section(s.name); PackedMesh p; BinReader r(v.data, v.size);
        if (!getPackedMesh(r, p)) { if (err) *err = s.name + ": unreadable mesh section"; return false; }
        RenderMesh m = unpackMesh(p); tris[s.material] += p.triangleCount();
        if (!first.count(s.material)) { PackedMesh f = p; f.pos.clear(); f.nrm.clear(); f.tan.clear(); f.uv.clear(); f.col.clear(); f.idx.clear(); first[s.material] = f; }
        MeshBuilder::append(merged[s.material], m);
    }
    if (merged.empty()) { if (err) *err = "no mesh sections in " + b.path(); return false; }
    std::vector<GlbEntry> entries;
    for (const auto& kv : merged) { GlbEntry e; e.name = kv.first; e.mesh = &kv.second; const PackedMesh& f = first[kv.first]; e.color = Vec3(f.albedo[0], f.albedo[1], f.albedo[2]); e.roughness = f.roughness; e.extras = nodeExtras(f, tris[kv.first]); entries.push_back(std::move(e)); }
    if (progress) (*progress)(0.85);
    const bool ok = writeGlb(entries, path, sceneExtras(b), err); if (progress) (*progress)(1.0); return ok;
}

bool writeBundleGlbCells(const Bundle& b, const std::string& dir, std::string* err, std::vector<std::string>* files, const GlbProgressFn* progress) {
    std::error_code ec; std::filesystem::create_directories(dir, ec);
    std::map<std::pair<int, int>, std::vector<MeshSection>> cells; for (const MeshSection& s : meshSections(b)) cells[{s.cx, s.cz}].push_back(s);
    if (cells.empty()) { if (err) *err = "no mesh sections in " + b.path(); return false; }
    const nlohmann::json extras = sceneExtras(b); nlohmann::json index = nlohmann::json::array(); size_t done = 0;
    for (const auto& kv : cells) {
        if (progress && !(*progress)(static_cast<double>(done++) / static_cast<double>(cells.size()))) { if (err) *err = "cancelled"; return false; }
        std::vector<RenderMesh> meshes; std::vector<GlbEntry> entries; size_t triangles = 0; nlohmann::json materials = nlohmann::json::array();
        double mn[3] = {1e300, 1e300, 1e300}, mx[3] = {-1e300, -1e300, -1e300};
        meshes.reserve(kv.second.size());
        for (const MeshSection& s : kv.second) {
            const Bundle::View v = b.section(s.name); PackedMesh p; BinReader r(v.data, v.size);
            if (!getPackedMesh(r, p)) { if (err) *err = s.name + ": unreadable mesh section"; return false; }
            meshes.push_back(unpackMesh(p)); triangles += p.triangleCount(); materials.push_back(s.material);
            for (size_t i = 0; i < p.vertexCount(); ++i) for (int k = 0; k < 3; ++k) { mn[k] = std::min(mn[k], static_cast<double>(p.pos[3 * i + static_cast<size_t>(k)])); mx[k] = std::max(mx[k], static_cast<double>(p.pos[3 * i + static_cast<size_t>(k)])); }
            GlbEntry e; e.name = s.material; e.mesh = &meshes.back(); e.color = Vec3(p.albedo[0], p.albedo[1], p.albedo[2]); e.roughness = p.roughness; e.extras = nodeExtras(p, p.triangleCount()); entries.push_back(std::move(e));
        }
        const std::string file = std::to_string(kv.first.first) + "_" + std::to_string(kv.first.second) + ".glb";
        nlohmann::json cellExtras = extras; cellExtras["cell"] = {{"cx", kv.first.first}, {"cz", kv.first.second}};
        if (!writeGlb(entries, dir + "/" + file, cellExtras, err)) return false;
        if (files) files->push_back(dir + "/" + file);
        index.push_back({{"cx", kv.first.first}, {"cz", kv.first.second}, {"file", file}, {"triangles", triangles}, {"materials", materials}, {"min", {mn[0], mn[1], mn[2]}}, {"max", {mx[0], mx[1], mx[2]}}});
    }
    if (progress) (*progress)(1.0);
    std::ofstream f(dir + "/index.json"); f << nlohmann::json{{"cells", index}, {"count", index.size()}}.dump(1) << "\n";
    if (!f) { if (err) *err = "cannot write " + dir + "/index.json"; return false; }
    return true;
}

}  // namespace bundle
}  // namespace engine
