#include "level_scene.h"

#include "engine/mesh_builder.h"
#include "engine/procgen/terrain.h"
#include "engine/procgen/erosion.h"
#include "engine/procgen/tree.h"
#include "engine/procgen/surface_maps.h"
#include "engine/model_importer.h"
#include "engine/procgen/city/city_lots.h"   // living-city lots (offline parity)
#include "engine/procgen/city/road_net.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain_field.h"   // HeightField (level ground sampler)
#include "engine/procgen/city/water_mesh.h" // buildWaterMesh (ocean/lake surface)
#include "engine/procgen/proc_model.h"      // ProcModel (script model cache)
#include "engine/level_params.h"            // shared level-JSON -> params readers
#include "log.h"
#include <unordered_map>
#include <array>
#include <sstream>
#include <nlohmann/json.hpp>
#include <fstream>
#ifdef RT_ENABLE_SCRIPTING
#include "engine/scripting/script_vm.h"
#include "engine/scripting/procgen_bindings.h"
#include "engine/scripting/script_modules.h"
#include "engine/script_assets.h"
#include "engine/scripting/script_modules.h"
#endif

using json = nlohmann::json;

namespace engine {

namespace {

// parseVec3/parseOrientation come from level_params.h — one definition for
// both this importer and the engine loader. (This file's old parseVec3
// rejected arrays longer than 3; the shared one keeps the loader's lenient
// >=3 read.)

int addSurfaceTexture(Scene& scene, const TextureData& td) {
    Texture t;
    t.width = td.width; t.height = td.height; t.channels = td.channels;
    t.pixels = td.pixels;
    return scene.addTexture(std::move(t));
}

// One bake per surface, shared across every material that uses it (a brick wall
// and a brick building reference the same baked set). Keyed by surface id.
using SurfaceTexCache = std::unordered_map<int, std::array<int, 4>>;

// Bake (or reuse from the cache) a surface's PBR texture set into the scene and
// bind it onto `mat`, with the tiling scale for its world tile size.
void bindSurfaceTextures(Material& mat, RenderMaterial::Surface surf, Scene& scene,
                         uint32_t seed, int texSize, SurfaceTexCache* cache) {
    int id = static_cast<int>(surf);
    std::array<int, 4> idx{-1, -1, -1, -1};
    bool cached = false;
    if (cache) {
        auto it = cache->find(id);
        if (it != cache->end()) { idx = it->second; cached = true; }
    }
    if (!cached) {
        SurfaceMaps maps = surfaceMaps(surf, texSize, seed);
        idx = {addSurfaceTexture(scene, maps.albedo),
               addSurfaceTexture(scene, maps.normal),
               addSurfaceTexture(scene, maps.mr),
               addSurfaceTexture(scene, maps.ao)};
        if (cache) (*cache)[id] = idx;
    }
    mat.albedoTex = idx[0]; mat.normalTex = idx[1];
    mat.mrTex = idx[2]; mat.aoTex = idx[3];
    double tile = surfaceWorldTileSize(surf);
    mat.texScale = tile > 1e-6 ? 1.0 / tile : 1.0;
    mat.surface = 0;   // textured: the analytic path is replaced by the maps
}

// Build a Material from a material JSON block (the body shared by an inline
// "material" object and a named entry in the top-level "materials" table). A
// "surface" bakes the procedural PBR texture set (albedo/normal/MR/AO) once and
// binds it (ADR-0039 Phase B); the maps tile by the surface's world tile size.
Material materialFromJson(const json& m, Scene& scene) {
    Vec3 albedo = parseVec3(m.value("albedo", json()), Vec3(0.8, 0.8, 0.8));
    double roughness = m.value("roughness", 0.5);
    double metallic = m.value("metallic", 0.0);
    Vec3 emission = parseVec3(m.value("emission", json()), Vec3(0, 0, 0));
    bool checkerboard = m.value("checkerboard", false);
    int surface = 0;
    if (m.value("brick", false)) surface = 1;   // back-compat alias
    if (m.contains("surface"))
        surface = static_cast<int>(surfaceFromName(m["surface"].get<std::string>()));
    if (m.contains("flags"))
        for (const auto& f : m["flags"])
            checkerboard |= (f == "checkerboard");
    if (emission.lengthSquared() > 0.0)
        return Material::emissive(emission, 1.0);
    Material mat = Material::pbr(albedo, metallic, roughness);
    mat.checkerboard = checkerboard;
    mat.surface = surface;
    if (surface != 0 && m.value("textured", true)) {
        auto surf = static_cast<RenderMaterial::Surface>(surface);
        bindSurfaceTextures(mat, surf, scene, m.value("seed", 0u),
                            m.value("textureSize", 256), nullptr);
        if (m.contains("tileSize")) {
            double t = m["tileSize"].get<double>();
            mat.texScale = t > 1e-6 ? 1.0 / t : 1.0;
        }
    }
    return mat;
}

// A named material library: the level's top-level "materials" table parsed once
// into Scene material indices, so entities can reference a shared material by
// name ("material": "brickWall") instead of repeating an inline block.
using MaterialTable = std::unordered_map<std::string, int>;

MaterialTable buildMaterialTable(const json& root, Scene& scene) {
    MaterialTable table;
    if (!root.contains("materials") || !root["materials"].is_object()) return table;
    for (auto it = root["materials"].begin(); it != root["materials"].end(); ++it)
        table[it.key()] = scene.addMaterial(materialFromJson(it.value(), scene));
    return table;
}

// Resolve an entity's material to a Scene index: a string is a reference into the
// named table; an object is an inline material; absent is the default.
int importMaterial(const json& ent, Scene& scene, const MaterialTable& table) {
    if (ent.contains("material")) {
        const auto& m = ent["material"];
        if (m.is_string()) {
            auto it = table.find(m.get<std::string>());
            if (it != table.end()) return it->second;
            LOG_WARN << "Material reference '" << m.get<std::string>()
                     << "' not found in the materials table; using default";
            return scene.addMaterial(Material::pbr(Vec3(0.8, 0.8, 0.8), 0.0, 0.5));
        }
        if (m.is_object()) return scene.addMaterial(materialFromJson(m, scene));
    }
    return scene.addMaterial(materialFromJson(json::object(), scene));
}

// Tessellated mesh -> world-space triangles, carrying per-vertex normal, uv,
// and color so the path tracer shades smooth and textured (alpha-cut foliage,
// terrain slope coloring), matching the realtime renderer. The entity
// transform is applied per vertex (scale, then rotate, then translate —
// matching Transform::matrix); normals are rotated (uniform scale assumed).
void addMeshAsTriangles(const RenderMesh& mesh, const Vec3& position,
                        const Quat& orientation, const Vec3& scale,
                        int matIdx, Scene& scene) {
    auto toWorld = [&](const Vertex& v) {
        Vec3 scaled(v.position.x * scale.x, v.position.y * scale.y,
                    v.position.z * scale.z);
        return position + orientation.rotate(scaled);
    };
    auto fill = [&](Triangle& tri, int slot, const Vertex& v) {
        Vec3 n = orientation.rotate(v.normal);
        Vec3 tg = orientation.rotate(v.tangent);
        Vec3 p = toWorld(v);
        if (slot == 0) { tri.v0 = p; tri.n0 = n; tri.t0 = tg; tri.c0 = v.color; tri.uv0[0] = v.u; tri.uv0[1] = v.v; }
        else if (slot == 1) { tri.v1 = p; tri.n1 = n; tri.t1 = tg; tri.c1 = v.color; tri.uv1[0] = v.u; tri.uv1[1] = v.v; }
        else { tri.v2 = p; tri.n2 = n; tri.t2 = tg; tri.c2 = v.color; tri.uv2[0] = v.u; tri.uv2[1] = v.v; }
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Triangle tri;
        tri.materialIndex = matIdx;
        fill(tri, 0, mesh.vertices[mesh.indices[i]]);
        fill(tri, 1, mesh.vertices[mesh.indices[i + 1]]);
        fill(tri, 2, mesh.vertices[mesh.indices[i + 2]]);
        scene.addTriangle(tri);
    }
}

// A prototype's triangles in LOCAL space (identity transform), for an instanced
// proto (ADR-0041). Mirrors addMeshAsTriangles' per-vertex attribute copy.
std::vector<Triangle> meshProtoTriangles(const RenderMesh& mesh, int matIdx) {
    std::vector<Triangle> out;
    auto fill = [&](Triangle& tri, int slot, const Vertex& v) {
        if (slot == 0) { tri.v0 = v.position; tri.n0 = v.normal; tri.t0 = v.tangent; tri.c0 = v.color; tri.uv0[0] = v.u; tri.uv0[1] = v.v; }
        else if (slot == 1) { tri.v1 = v.position; tri.n1 = v.normal; tri.t1 = v.tangent; tri.c1 = v.color; tri.uv1[0] = v.u; tri.uv1[1] = v.v; }
        else { tri.v2 = v.position; tri.n2 = v.normal; tri.t2 = v.tangent; tri.c2 = v.color; tri.uv2[0] = v.u; tri.uv2[1] = v.v; }
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Triangle tri;
        tri.materialIndex = matIdx;
        fill(tri, 0, mesh.vertices[mesh.indices[i]]);
        fill(tri, 1, mesh.vertices[mesh.indices[i + 1]]);
        fill(tri, 2, mesh.vertices[mesh.indices[i + 2]]);
        out.push_back(tri);
    }
    return out;
}

bool isIdentity(const Quat& q) {
    return q.x == 0.0 && q.y == 0.0 && q.z == 0.0;
}

// (parseTerrainParams moved to engine/level_params.cpp as readTerrainParams —
// the ONE parse this importer and the engine loader share, so the city, roads,
// and the offline render all drape onto the exact same surface.)

// A procedural terrain block: regenerate the same mesh the engine does and add
// it (vertex color carries the slope/height coloring; material albedo
// multiplies it, so a white albedo lets the baked color show).
void addTerrain(const json& t, Scene& scene, const MaterialTable& materials,
                const std::vector<TerrainFlatten>& flatten = {},
                std::shared_ptr<const std::function<double(double, double)>>
                    erodedBase = nullptr) {
    TerrainParams tp = readTerrainParams(t);
    tp.erodedBase = erodedBase;   // shared eroded surface (roads conform to it)
    tp.flatten = flatten;   // city cut/fill so the ground meets the roads/blocks
    rebuildFlattenIndex(tp);   // index the cut/fill set (ADR-0075 P0)
    Noise noise(t.value("seed", 0u));
    int matIdx = importMaterial(t, scene, materials);
    // Chunked terrain (ADR-0034 Phase 1): bake every chunk's triangles. The
    // offline tracer has no far clip or culling, so this is the same surface as
    // the single mesh — the parity oracle for the viewer's chunked path.
    if (t.contains("chunks") && t["chunks"].get<int>() > 0) {
        int chunksPerSide = t.value("chunks", 1);
        float chunkSize = t.value("chunkSize", tp.size);
        int res = t.value("chunkResolution", tp.resolution);
        float colliderRadius = t.value("colliderRadius", chunkSize * 1.5f);
        for (TerrainChunk& chunk :
             generateTerrainChunks(tp, noise, chunksPerSide, chunkSize, res,
                                   colliderRadius))
            addMeshAsTriangles(chunk.mesh, Vec3(), Quat::identity(),
                               Vec3(1, 1, 1), matIdx, scene);
        return;
    }
    if (t.value("erode", false) && !tp.erodedBase) {
        // Bake the analytic base into an eroded field and hang it on the params,
        // so every terrainHeight() sample (mesh, LOD rings, flatten, normals,
        // color banding) reads the eroded surface — the same path CDLOD uses.
        // Skipped when the caller already supplied a shared eroded base.
        ErosionParams ep;
        ep.seed = t.value("seed", 0u) + 1234u;
        ep.droplets = t.value("erodeDroplets", ep.droplets);
        ep.erodeRadius = t.value("erodeRadius", ep.erodeRadius);
        ep.thermalIterations = t.value("erodeThermal", ep.thermalIterations);
        ep.talus = t.value("erodeTalus", ep.talus);
        int erodeRes = t.value("erodeRes", 512);
        bakeErodedTerrain(tp, noise, tp.size, erodeRes, ep);
    }
    addMeshAsTriangles(generateTerrain(tp, noise), Vec3(), Quat::identity(),
                       Vec3(1, 1, 1), matIdx, scene);
    // Distant LOD rings (mountains/hills out to the horizon).
    int lodRings = t.value("lodRings", 0);
    if (lodRings > 0) {
        for (RenderMesh& ring :
             generateTerrainLOD(tp, noise, lodRings, t.value("lodCells", 40)))
            addMeshAsTriangles(ring, Vec3(), Quat::identity(), Vec3(1, 1, 1),
                               matIdx, scene);
    }
}

// A hero parametric tree (shape:"tree"): grow the same mesh the viewer does,
// add bark (opaque) and leaves (alpha-cut cards) as separate materials. Color
// is baked into the vertex color, so both materials use a white albedo — the
// leaf texture's alpha drives the cutout (FLAG_ALPHA_TEST in the viewer).
void addTree(const json& ent, Scene& scene) {
    uint32_t seed = 0;
    TreeParams tp = readTreeParams(ent, seed);

    Vec3 position = parseVec3(ent.value("position", json()));
    Quat orientation = parseOrientation(ent);
    Vec3 scale = parseVec3(ent.value("scale", json()), Vec3(1, 1, 1));

    TreeMesh tm = growTree(tp, seed);
    if (tm.branches.vertices.empty()) return;

    int barkMat = scene.addMaterial(Material::pbr(Vec3(1, 1, 1), 0.0, 1.0));
    addMeshAsTriangles(tm.branches, position, orientation, scale, barkMat, scene);

    if (!tm.leaves.vertices.empty()) {
        TextureData leaf = leafTexture(128);
        Texture tex;
        tex.width = leaf.width;
        tex.height = leaf.height;
        tex.channels = leaf.channels;
        tex.pixels = std::move(leaf.pixels);
        int alphaTex = scene.addTexture(std::move(tex));

        Material leafMat = Material::pbr(Vec3(1, 1, 1), 0.0, 0.7);
        leafMat.alphaTex = alphaTex;
        int leafMatIdx = scene.addMaterial(leafMat);
        addMeshAsTriangles(tm.leaves, position, orientation, scale, leafMatIdx,
                           scene);
    }
}

int addCpuImage(Scene& scene, const CpuImage& img) {
    if (!img.valid()) return -1;
    Texture t;
    t.width = img.width; t.height = img.height; t.channels = img.channels;
    t.pixels = img.pixels;
    return scene.addTexture(std::move(t));
}

// Path-trace an imported glTF model (ADR-0039): the same CPU parse the realtime
// importer uploads, but baked into Scene triangles + textures so the model uses
// the identical material/texture system as procedural surfaces — only the UV
// source differs (the glTF's authored UVs/tangents, flagged meshUV).
void addGltfModel(const json& ent, const std::string& levelDir, Scene& scene) {
    std::string meshPath = ent["mesh"].get<std::string>();
    if (!meshPath.empty() && meshPath[0] != '/') meshPath = levelDir + "/" + meshPath;
    CpuModel model = ModelImporter::loadCpu(meshPath);
    if (model.meshes.empty()) return;

    Vec3 position = parseVec3(ent.value("position", json()));
    Vec3 scale = parseVec3(ent.value("scale", json()), Vec3(1, 1, 1));
    Quat orientation = parseOrientation(ent);

    int meshCount = 0;
    for (const CpuMesh& cm : model.meshes) {
        const CpuMaterial& gm = cm.material;
        Material mat = Material::pbr(gm.baseColor, gm.metallic, gm.roughness);
        mat.emission = gm.emission;
        mat.meshUV = true;   // sample with the glTF's own UVs (no world tiling)
        mat.albedoTex = addCpuImage(scene, gm.baseColorTex);
        mat.normalTex = addCpuImage(scene, gm.normalTex);
        mat.mrTex = addCpuImage(scene, gm.metallicRoughnessTex);
        mat.aoTex = addCpuImage(scene, gm.aoTex);
        mat.emissiveTex = addCpuImage(scene, gm.emissiveTex);
        int mi = scene.addMaterial(mat);
        addMeshAsTriangles(cm.geometry, position, orientation, scale, mi, scene);
        ++meshCount;
    }
    LOG_INFO << "glTF model " << meshPath << ": " << meshCount << " mesh(es)";
}

}  // namespace


// Add a baked TextureData to the Scene as a Texture; -1 if empty.
static int addProcTexture(const TextureData& td, Scene& scene) {
    if (td.pixels.empty()) return -1;
    Texture tex;
    tex.width = td.width; tex.height = td.height; tex.channels = td.channels;
    tex.pixels = td.pixels;
    return scene.addTexture(std::move(tex));
}

void bakeProcModel(const ProcModel& m, Scene& scene, const Vec3& offset) {
    for (const ProcPart& part : m.parts) {
        if (part.mesh.indices.empty()) continue;
        const ProcMaterial& pm = part.material;
        Material mat = Material::pbr(Vec3(1, 1, 1), pm.metallic, pm.roughness);
        if (pm.textured) {
            // World-planar tiling frame (meshUV=false): no authored UVs needed;
            // surfFrame projects at texScale = repeats per world unit (ADR-0043).
            mat.meshUV = false;
            mat.texScale = pm.tile > 1e-6 ? 1.0 / pm.tile : 1.0;
            mat.albedoTex = addProcTexture(pm.albedo, scene);
            mat.normalTex = addProcTexture(pm.normal, scene);
        } else {
            // Analytic surface (e.g. RoadMarkings) reads the mesh's own road-local UV
            // at shade time — no baked texture, paint composites over the vertex colour.
            mat.surface = pm.surface;
        }
        int mi = scene.addMaterial(mat);
        addMeshAsTriangles(part.mesh, offset, Quat::identity(), Vec3(1, 1, 1), mi, scene);
    }
    // Instance groups -> one BLAS proto each, placed by the TLAS (ADR-0041).
    for (const ProcInstanceGroup& g : m.instances) {
        if (g.proto.indices.empty() || g.transforms.empty()) continue;
        Material mat = Material::pbr(Vec3(1, 1, 1), g.metallic, g.roughness);
        if (g.alphaFoliage) {                     // alpha-cut leaf cards
            TextureData leaf = leafTexture(128);
            Texture tex;
            tex.width = leaf.width; tex.height = leaf.height;
            tex.channels = leaf.channels; tex.pixels = std::move(leaf.pixels);
            mat.alphaTex = scene.addTexture(std::move(tex));
        }
        int mi = scene.addMaterial(mat);
        int proto = scene.addProto(meshProtoTriangles(g.proto, mi));
        for (const Mat4& xf : g.transforms) {
            Mat4 placed = xf;                 // translate the instance by the offset
            placed.m[0][3] += offset.x; placed.m[1][3] += offset.y; placed.m[2][3] += offset.z;
            scene.addInstance(proto, placed);
        }
    }
}

bool LevelScene::load(const std::string& levelPath, Scene& scene,
                      std::string* outHdrPath) {
    std::ifstream file(levelPath);
    if (!file.is_open()) {
        LOG_ERROR << "Cannot open level: " << levelPath;
        return false;
    }
    json root;
    try {
        root = json::parse(file);
    } catch (const json::exception& e) {
        LOG_ERROR << "Level " << levelPath << " is malformed: " << e.what();
        return false;
    }

    // Sea level is ONE source of truth: the terrain colours its coast by it and
    // every road recipe gates buildability on it (see level_params.h).
    propagateWaterSeaLevel(root);

    // A city draped on the terrain has to be generated BEFORE the terrain mesh:
    // it computes its road/block grades from the natural ground, then hands back
    // cut/fill footprints (flatten) that the terrain is built around so the
    // ground meets the carriageways instead of poking through. Pre-generate the
    // first such city here; the entity loop reuses the cached model (keyed by
    // pointer) so it isn't generated twice.
    // Named material library: the top-level "materials" table, parsed once so
    // entities can reference shared materials by name (ADR-0039).
    MaterialTable materials = buildMaterialTable(root, scene);

    // Level directory, for resolving glTF mesh paths relative to the level file.
    std::string levelDir = ".";
    if (auto slash = levelPath.find_last_of('/'); slash != std::string::npos)
        levelDir = levelPath.substr(0, slash);

    // Level ground sampler (ADR-0044): the NATURAL terrain height (no flatten yet),
    // handed to a draped recipe as the `ground` global so it can seat its city and
    // conform the carriageways. Its cut/fill footprints come back via m:conform and
    // feed the terrain build below, so the walkable CDLOD ground meets the roads.
    // Baked EROSION (shared): if the terrain opts in with "erode": true, bake the
    // eroded height field ONCE and share the sampler across levelGround (roads/lots
    // conform to it), the lot ground, and addTerrain's mesh — so the offline render
    // matches the same eroded surface everywhere (see level_loader for the rationale).
    auto sharedEroded = readErodedBase(root);

    HeightField levelGround;
    if (root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
        tp->erodedBase = sharedEroded;
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        levelGround = [tp, noise](double x, double z) {
            return terrainHeight(*tp, *noise, x, z);
        };
    }

    int skipped = 0;
    std::vector<std::pair<const json*, ProcModel>> scriptCache;   // pre-run on-terrain
    std::vector<TerrainFlatten> scriptFlatten;
#ifdef RT_ENABLE_SCRIPTING
    std::unique_ptr<ScriptVM> scriptVm;
    auto ensureVm = [&]() -> ScriptVM& {
        if (!scriptVm) {
            scriptVm = std::make_unique<ScriptVM>();
            openProcgenLibrary(*scriptVm);
            openModuleLoader(*scriptVm, makeModuleSource(levelDir));
        }
        return *scriptVm;
    };
    // Run a script entity's recipe into `out`. On-terrain recipes get the level
    // ground injected so they drape + conform the engine terrain.
    auto runScript = [&](const json& ent, ProcModel& out) -> bool {
        std::string file = ent.value("file", std::string());
        std::string code = file.empty() ? std::string() : loadScriptCode(file, levelDir);
        if (code.empty()) { LOG_WARN << "script entity: cannot read '" << file << "'"; return false; }
        ScriptVM& vm = ensureVm();
        vm.setGlobalNumber("seed", ent.value("seed", 0.0));
        if (ent.contains("opts")) setRecipeArgs(vm, ent["opts"].dump());
        if (ent.value("onTerrain", false) && levelGround)
            setGlobalHeightField(vm, "ground", levelGround);
        std::string err;
        if (!runProcgenModelValue(vm, code, out, &err)) {
            LOG_ERROR << "script entity '" << file << "': " << err;
            return false;
        }
        return true;
    };
    // Pre-pass: run on-terrain recipes BEFORE the terrain so their cut/fill
    // footprints can grade it. Cache the model (by pointer) so the entity loop
    // bakes it rather than re-running the recipe.
    if (levelGround) {
        for (const auto& ent : root.value("entities", json::array())) {
            if (ent.value("shape", std::string()) == "script" &&
                ent.value("onTerrain", false)) {
                ProcModel m;
                if (runScript(ent, m)) {
                    scriptFlatten.insert(scriptFlatten.end(), m.flatten.begin(),
                                         m.flatten.end());
                    scriptCache.emplace_back(&ent, std::move(m));
                }
            }
        }
    }
#endif

    // Procedural terrain (top-level block, regenerated from its recipe), graded
    // flat under the city/script cut-fill footprints — and CARVED to every
    // shape:"road" net (mirroring the viewer's road pre-pass; without this the
    // offline ground buries the draped roads — device: "the road is being
    // buried by the terrain").
    if (root.contains("terrain")) {
        std::vector<TerrainFlatten> allFlatten;
        allFlatten.insert(allFlatten.end(), scriptFlatten.begin(), scriptFlatten.end());
        std::vector<RoadEntity> lotNets;
        if (levelGround)
            for (const auto& ent : root.value("entities", json::array())) {
                if (ent.value("shape", std::string()) != "road") continue;
                const json roadBlock =
                    ent.contains("road") ? ent["road"] : json::object();
                RoadEntity net = roadNetFromJson(roadBlock);
                // levelGround (natural) gates terrain-aware recipes (metro).
                if (roadBlock.contains("generate"))
                    applyGenerateRecipe(net, roadBlock["generate"], levelGround);
                std::vector<TerrainFlatten> r =
                    roadNetConformRegions(net, levelGround);
                allFlatten.insert(allFlatten.end(), r.begin(), r.end());
                lotNets.push_back(std::move(net));
            }
        // Living-city LOTS (mirrors the viewer's loader): grow the blocks'
        // buildings on the road-carved ground, stamp each building's FLAT
        // graded pad into the terrain, and bake the grown geometry — so the
        // offline render shows the same city-on-terrain the device does.
        if (levelGround && !lotNets.empty() && root.contains("citysim") &&
            root["citysim"].value("buildLots", false)) {
            const json& cs = root["citysim"];
            TerrainParams ctp = readTerrainParams(root["terrain"]);
            ctp.erodedBase = sharedEroded;   // lots grade off the eroded ground
            ctp.flatten = allFlatten;   // the road-carved ground the lots see
            rebuildFlattenIndex(ctp);   // index the cut/fill set (ADR-0075 P0)
            Noise cnoise(root["terrain"].value("seed", 0u));
            engine::EdgeBlockParams ep;
            engine::LotParams lp;
            readLotGrowParams(cs, ep, lp);
            // EDITOR/RUNTIME PARITY: the JSON-derived half is the shared reader
            // above (which now carries the parcel grain and hubRadius). What it
            // cannot carry is anything derived from the ROAD NETS — the hub list
            // and the coreness anchor — so those stay here, mirroring
            // level_loader's growCityLots exactly. Without them the editor
            // preview grows a DIFFERENT city than the game: no districts past
            // the radial rings, and coreness 0 everywhere, which means no towers.
            for (const engine::RoadEntity& n : lotNets)
                for (const engine::CityHub& h : n.plan.cityHubs)
                    lp.hubs.push_back({h.pos, h.kind});
            for (const engine::RoadEntity& n : lotNets)
                for (const engine::CityHub& h : n.plan.cityHubs) {
                    if (lp.center.x == 0 && lp.center.y == 0) lp.center = h.pos;
                    if (h.kind == 0) {
                        lp.center = h.pos;
                        break;
                    }
                }
            lp.ground = [&ctp, &cnoise](engine::Real x, engine::Real z) {
                return static_cast<engine::Real>(terrainHeight(ctp, cnoise, x, z));
            };
#ifdef RT_ENABLE_SCRIPTING
            std::unique_ptr<ScriptVM> styleVm;   // must outlive the grow
            {
                std::string sb = loadScriptCode("style_book.lua", levelDir);
                if (!sb.empty()) {
                    styleVm = std::make_unique<ScriptVM>();
                    openProcgenLibrary(*styleVm);
                    std::string err;
                    auto hook = engine::makeStyleBook(*styleVm, sb, &err);
                    if (hook) lp.styleHook = std::move(hook);
                }
            }
            // ARCHETYPE BOOK — same all-or-nothing contract as the viewer's
            // loader (level_loader.cpp); a rejected book LOG_ERRORs and the
            // compiled ladders stand, so both hosts grow the SAME city.
            {
                std::string ab = loadScriptCode("archetype_book.lua", levelDir);
                if (!ab.empty()) {
                    ScriptVM vm;   // the book is pure data once parsed
                    openProcgenLibrary(vm);
                    std::string err;
                    engine::ArchetypeBook book =
                        engine::makeArchetypeBook(vm, ab, &err);
                    if (!err.empty())
                        LOG_ERROR << "archetype_book.lua REJECTED "
                                     "(all-or-nothing): " << err;
                    else
                        lp.archetypeBook = std::move(book);
                }
            }
#endif
            engine::NetLotResult lots = engine::growLotBuildingsOnNets(
                lotNets, lp, ep, cs.value("sidewalk", 4.0) + 0.6, levelGround);
            // Building pads: flat graded ground under every footprint.
            for (const engine::LotBuilding& lb : lots.lots) {
                if (lb.type == "park" || lb.type == "green" ||
                    lb.plan.size() < 3) continue;
                std::vector<Vec3> poly;
                poly.reserve(lb.plan.size());
                for (const engine::Vec2& v : lb.plan)
                    poly.push_back(Vec3(v.x, 0, v.y));
                allFlatten.push_back(makeFlattenPad(std::move(poly), lb.groundY, 5.0));
            }
            // Bake the grown parts with the SAME per-part PBR recipes the city
            // pipeline binds (materialFor + baked surface texture sets).
            {
                using S = RenderMaterial::Surface;
                SurfaceTexCache texCache;
                for (const RenderMesh& part : lots.parts) {
                    if (part.vertices.empty()) continue;
                    RenderMaterial rm = materialFor(
                        static_cast<PartId>(part.materialIndex), Vec3(1, 1, 1));
                    Material mat = Material::pbr(Vec3(1, 1, 1), rm.metallic,
                                                 rm.roughness);
                    if (rm.surface() != S::None)
                        bindSurfaceTextures(mat, rm.surface(), scene, 1337u, 256,
                                            &texCache);
                    int mi = scene.addMaterial(mat);
                    addMeshAsTriangles(part, Vec3(), Quat::identity(),
                                       Vec3(1, 1, 1), mi, scene);
                }
                // Park / green pads: draped lot-shaped slabs, tinted like the
                // viewer (grass green; sculpted parks bake their own colours
                // and arrive white).
                for (const engine::LotBuilding& lb : lots.lots) {
                    if ((lb.type != "park" && lb.type != "green") ||
                        lb.padMesh.vertices.empty()) continue;
                    Material mat = Material::pbr(lb.color, 0.0, 1.0);
                    int mi = scene.addMaterial(mat);
                    addMeshAsTriangles(lb.padMesh, Vec3(), Quat::identity(),
                                       Vec3(1, 1, 1), mi, scene);
                }
                // Sculpted lots carry TREE SPOTS (x, scale, z): bake real
                // trees at them so parks and yards keep their landscaping in
                // the offline render too (viewer parity).
                {
                    std::vector<TreeMesh> kits;
                    int barkMi = -1, leafMi = -1;
                    for (const engine::LotBuilding& lb : lots.lots) {
                        if (lb.treeSpots.empty()) continue;
                        if (kits.empty()) {
                            TreeParams tp;
                            tp.iterations = 4;
                            tp.rootCount = 0;
                            for (uint32_t k = 0; k < 3; ++k)
                                kits.push_back(growTree(tp, 97u + k));
                            barkMi = scene.addMaterial(
                                Material::pbr(Vec3(1, 1, 1), 0.0, 1.0));
                            TextureData leaf = leafTexture(128);
                            Texture tex;
                            tex.width = leaf.width;
                            tex.height = leaf.height;
                            tex.channels = leaf.channels;
                            tex.pixels = std::move(leaf.pixels);
                            Material leafMat = Material::pbr(Vec3(1, 1, 1), 0.0, 0.7);
                            leafMat.alphaTex = scene.addTexture(std::move(tex));
                            leafMi = scene.addMaterial(leafMat);
                        }
                        uint32_t th = static_cast<uint32_t>(
                            std::llround(lb.site.x * 73.1 + lb.site.y * 37.7)) *
                            2654435761u;
                        for (const Vec3& s : lb.treeSpots) {
                            th = th * 1664525u + 1013904223u;
                            const TreeMesh& kit = kits[(th >> 8) % kits.size()];
                            const Vec3 pos(s.x, lp.ground ? lp.ground(s.x, s.z) : 0.0,
                                           s.z);
                            const Quat rot = Quat::fromAxisAngle(
                                Vec3(0, 1, 0), ((th >> 8) % 628u) / 100.0);
                            addMeshAsTriangles(kit.branches, pos, rot,
                                               Vec3(s.y, s.y, s.y), barkMi, scene);
                            if (!kit.leaves.vertices.empty())
                                addMeshAsTriangles(kit.leaves, pos, rot,
                                                   Vec3(s.y, s.y, s.y), leafMi,
                                                   scene);
                        }
                    }
                }
            }
        }
        addTerrain(root["terrain"], scene, materials, allFlatten, sharedEroded);
    }

    // Water surface (ocean / lake): a flat plane at seaLevel, emitted only where the
    // natural terrain floor dips below it (buildWaterMesh skips land cells). Offline
    // it renders as a dark, glossy dielectric — near-mirror at grazing angles (sky
    // reflection), deep blue face-on — the still-water read the depth/foam shader
    // gives on device. Needs a terrain floor, so it's gated on levelGround.
    if (root.contains("water") && levelGround) {
        const json& w = root["water"];
        engine::WaterMeshParams wp = readWaterParams(w);
        RenderMesh wmesh = engine::buildWaterMesh(levelGround, wp);
        if (!wmesh.vertices.empty()) {
            Vec3 tint(0.015, 0.05, 0.09);
            if (w.contains("color") && w["color"].is_array() && w["color"].size() == 3)
                tint = Vec3(w["color"][0], w["color"][1], w["color"][2]);
            Material wm = Material::pbr(tint, 0.0, w.value("roughness", 0.04));
            int mi = scene.addMaterial(wm);
            addMeshAsTriangles(wmesh, Vec3(), Quat::identity(), Vec3(1, 1, 1), mi,
                               scene);
        }
    }

    for (const auto& ent : root.value("entities", json::array())) {
        if (ent.contains("mesh")) {
            addGltfModel(ent, levelDir, scene);   // imported glTF, path-traced
            continue;
        }
        // Lua recipe (ADR-0042): run a script returning a composable Model and
        // bake it — the same recipe the viewer runs, so a procgen scene renders
        // offline through the same pipeline (the renderer is the only divergence).
        if (ent.value("shape", std::string()) == "script") {
#ifdef RT_ENABLE_SCRIPTING
            Vec3 scriptPos = parseVec3(ent.value("position", json()));  // world placement
            bool baked = false;
            for (auto& pre : scriptCache)            // pre-run on-terrain recipe
                if (pre.first == &ent) { bakeProcModel(pre.second, scene, scriptPos); baked = true; break; }
            if (!baked) {
                ProcModel model;
                if (runScript(ent, model)) bakeProcModel(model, scene, scriptPos);
            }
#else
            LOG_WARN << "script entity skipped (scripting disabled in this build)";
#endif
            continue;
        }
        // Hero parametric tree: bark + alpha-cut leaf cards (matches the viewer).
        if (ent.value("shape", std::string()) == "tree") {
            addTree(ent, scene);
            continue;
        }
        // Editor-authored road (shape:"road", ADR-0049): the same RoadEntity the
        // editor edits, baked to the carriageway and draped on the level terrain.
        if (ent.value("shape", std::string()) == "road") {
            const json roadBlock = ent.contains("road") ? ent["road"] : json::object();
            RoadEntity net = roadNetFromJson(roadBlock);
            // A generated network (ADR-0056 "generate" recipe — grown.json's whole
            // city) has no authored nodes: grow it exactly like the level loader
            // does, or the offline render shows bare ground where the city is.
            // levelGround (natural) gates the metro's terrain-aware layout and
            // drapes the mesh.
            if (roadBlock.contains("generate"))
                applyGenerateRecipe(net, roadBlock["generate"], levelGround);
            Material rm = Material::pbr(Vec3(1, 1, 1), 0.0, 0.93);
            if (net.look.markings)   // lane paint via the RoadMarkings surface, not geometry
                rm.surface = static_cast<int>(RenderMaterial::Surface::RoadMarkings);
            int mi = scene.addMaterial(rm);
            addMeshAsTriangles(buildRoadNetMesh(net, levelGround), Vec3(),
                               Quat::identity(), Vec3(1, 1, 1), mi, scene);
            continue;
        }
        static const char* SUPPORTED[] = {"sphere", "box", "plane", "cylinder",
                                          "cone", "wedge", "torus", "capsule"};
        std::string shape = ent.value("shape", "box");
        bool supported = false;
        for (const char* sh : SUPPORTED) supported |= (shape == sh);
        if (!supported) {
            skipped++;  // e.g. glTF "model" entities
            continue;
        }

        Vec3 size = parseVec3(ent.value("size", json()), Vec3(1, 1, 1));
        Vec3 position = parseVec3(ent.value("position", json()));
        Vec3 scale = parseVec3(ent.value("scale", json()), Vec3(1, 1, 1));
        Quat orientation = parseOrientation(ent);
        int matIdx = importMaterial(ent, scene, materials);

        // Same size semantics as the viewer's loader: spheres/cylinders/cones
        // use size.x as radius, size.y as height where applicable.
        if (shape == "sphere" && isIdentity(orientation) &&
            scale.x == scale.y && scale.y == scale.z) {
            scene.addSphere(position, size.x * scale.x, matIdx);
        } else if (shape == "sphere") {
            addMeshAsTriangles(MeshBuilder::sphere(static_cast<float>(size.x)),
                               position, orientation, scale, matIdx, scene);
        } else if (shape == "box") {
            addMeshAsTriangles(MeshBuilder::box(size), position, orientation,
                               scale, matIdx, scene);
        } else if (shape == "plane") {
            addMeshAsTriangles(
                MeshBuilder::plane(static_cast<float>(size.x),
                                   static_cast<float>(size.y)),
                position, orientation, scale, matIdx, scene);
        } else if (shape == "cylinder") {
            addMeshAsTriangles(
                MeshBuilder::cylinder(static_cast<float>(size.x),
                                      static_cast<float>(size.y)),
                position, orientation, scale, matIdx, scene);
        } else if (shape == "cone") {
            addMeshAsTriangles(MeshBuilder::cone(static_cast<float>(size.x),
                                                 static_cast<float>(size.y)),
                               position, orientation, scale, matIdx, scene);
        } else if (shape == "wedge") {
            addMeshAsTriangles(MeshBuilder::wedge(size), position, orientation,
                               scale, matIdx, scene);
        } else if (shape == "torus") {
            addMeshAsTriangles(
                MeshBuilder::torus(static_cast<float>(size.x),
                                   static_cast<float>(size.y)),
                position, orientation, scale, matIdx, scene);
        } else if (shape == "capsule") {
            addMeshAsTriangles(
                MeshBuilder::capsule(static_cast<float>(size.x),
                                     static_cast<float>(size.y)),
                position, orientation, scale, matIdx, scene);
        }
    }
    if (skipped > 0)
        LOG_WARN << "Skipped " << skipped << " unsupported entit"
                 << (skipped == 1 ? "y" : "ies") << " (e.g. glTF models)";

    // Outdoor lighting: levels are lit by sun + sky, not emissive geometry.
    // This parallels the viewer's light pass (ADR-0017): the same sun/point/
    // spot lights, sampled explicitly with shadow rays instead of shadow maps.
    scene.environment.enabled = true;
    std::string hdr;
    if (root.contains("environment") && root["environment"].is_object()) {
        const auto& env = root["environment"];
        Vec3 sky = parseVec3(env.value("skyColor", json()),
                             scene.environment.skyHorizon);
        scene.environment.skyHorizon = sky;
        scene.environment.skyZenith = sky;
        // Aerial-perspective fog (defaults its color to the sky for a seamless
        // horizon): "fog": { "density": 0.0008, "color": [r,g,b] }.
        if (env.contains("fog") && env["fog"].is_object()) {
            const auto& f = env["fog"];
            scene.fog.enabled = true;
            scene.fog.density = f.value("density", 0.0);
            // Authored by levels and honoured by the realtime renderers; the
            // offline path used to ignore it and fog uniformly (see Fog in
            // scene.h) — which over-hazed every aerial render.
            scene.fog.heightFalloff = f.value("heightFalloff", 0.0);
            scene.fog.color = parseVec3(f.value("color", json()), sky);
        }
        hdr = env.value("hdr", std::string());
        if (!hdr.empty() && outHdrPath) {
            // Resolve relative to the level file, like the viewer's loader.
            std::string dir = levelPath;
            std::size_t slash = dir.find_last_of("/\\");
            dir = (slash == std::string::npos) ? "" : dir.substr(0, slash + 1);
            *outHdrPath = dir + hdr;
        }
    }

    const json lighting = root.value("lighting", json::object());
    bool hasSun = false;
    if (lighting.contains("sun")) {
        const auto& sun = lighting["sun"];
        SceneLight l;
        l.type = SceneLight::Type::Directional;
        l.direction = parseVec3(sun.value("direction", json()), l.direction);
        l.color = parseVec3(sun.value("color", json()), l.color);
        l.intensity = sun.value("intensity", 4.0);
        scene.lights.push_back(l);
        hasSun = true;
    }
    for (const auto& pl : lighting.value("pointLights", json::array())) {
        SceneLight l;
        l.type = SceneLight::Type::Point;
        l.position = parseVec3(pl.value("position", json()));
        l.color = parseVec3(pl.value("color", json()), l.color);
        l.intensity = pl.value("intensity", 1.0);
        l.range = pl.value("range", 25.0);
        scene.lights.push_back(l);
    }
    for (const auto& sl : lighting.value("spotLights", json::array())) {
        SceneLight l;
        l.type = SceneLight::Type::Spot;
        l.position = parseVec3(sl.value("position", json()));
        l.direction = parseVec3(sl.value("direction", json()), Vec3(0, -1, 0));
        l.color = parseVec3(sl.value("color", json()), l.color);
        l.intensity = sl.value("intensity", 1.0);
        l.range = sl.value("range", 25.0);
        l.innerConeAngle = sl.value("innerConeAngle", 0.3);
        l.outerConeAngle = sl.value("outerConeAngle", 0.5);
        scene.lights.push_back(l);
    }
    if (!hasSun && hdr.empty()) {
        // The viewer derives its sun from the HDR / day-night cycle, neither
        // of which exists here — light with a default noon sun instead. With
        // an HDR, the caller extracts the dominant light from the map itself.
        SceneLight noon;
        noon.direction = Vec3(0.35, 0.8, 0.25);
        noon.color = Vec3(1.0, 0.95, 0.85);
        noon.intensity = 4.0;
        scene.lights.push_back(noon);
        LOG_INFO << "Level has no explicit sun; using a default noon sun";
    }

    scene.buildAccelerator();
    LOG_INFO << "Level scene: " << scene.triangles.size() << " triangles, "
             << scene.spheres.size() << " spheres, "
             << scene.materials.size() << " materials";
    return true;
}

bool loadSidecarCamera(const std::string& levelPath, const std::string& name,
                       SidecarCamera& out) {
    std::string path = levelPath + ".cameras.json";
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR << "No camera sidecar at " << path
                  << " — place a camera in the viewer first";
        return false;
    }
    json root;
    try {
        root = json::parse(file);
    } catch (const json::exception& e) {
        LOG_ERROR << "Camera sidecar " << path << " is malformed: " << e.what();
        return false;
    }

    for (const auto& c : root.value("cameras", json::array())) {
        std::string camName = c.value("name", std::string());
        if (!name.empty() && camName != name) continue;

        out.name = camName;
        out.position = parseVec3(c.value("position", json()));
        out.forward = parseVec3(c.value("forward", json()), Vec3(0, 0, -1));
        if (c.contains("lens")) {
            const auto& l = c["lens"];
            out.lens.focalLength = l.value("focalLength", out.lens.focalLength);
            out.lens.sensorHeight = l.value("sensorHeight", out.lens.sensorHeight);
            out.lens.fStop = l.value("fStop", out.lens.fStop);
            out.lens.focusDistance = l.value("focusDistance", out.lens.focusDistance);
            out.lens.distortionK1 = l.value("k1", out.lens.distortionK1);
            out.lens.distortionK2 = l.value("k2", out.lens.distortionK2);
            out.lens.chromaticAberration =
                l.value("chromaticAberration", out.lens.chromaticAberration);
            out.lens.vignette = l.value("vignette", out.lens.vignette);
        }
        return true;
    }
    LOG_ERROR << "Camera \"" << name << "\" not found in " << path;
    return false;
}

}  // namespace engine

namespace engine {

DayNightState applyDayNight(Scene& scene, const DayNightCycle& cycle, double lightPollution) {
    const DayNightState st = cycle.evaluate();
    // The authored sun's intensity is the truth the cycle shapes (the viewer
    // normalizes against the curve's noon anchor the same way).
    double authored = 4.0;
    std::vector<SceneLight> kept;
    for (const SceneLight& l : scene.lights) {
        if (l.type == SceneLight::Type::Directional) authored = l.intensity;
        else kept.push_back(l);
    }
    scene.lights = std::move(kept);
    if (st.lightIntensity > 1e-4f) {
        SceneLight l;
        l.type = SceneLight::Type::Directional;
        l.direction = st.lightDirection;
        l.color = st.lightColor;
        l.intensity = authored * (st.lightIntensity / kCycleNoonSunIntensity);
        scene.lights.push_back(l);
    }
    scene.environment.enabled = true;
    scene.environment.procedural = true;
    scene.environment.night = st;
    scene.environment.lightPollution = std::max(lightPollution, 0.0);
    // The haze takes the moment's horizon colour (the viewer fades its fog
    // toward the sky): an authored daylight-blue fog over a night render
    // would wash the city in a bright mist.
    if (scene.fog.enabled) scene.fog.color = st.horizonColor;
    LOG_INFO << "Day/night at " << cycle.timeOfDay * 24.0 << " h: sun elevation "
             << st.solarElevation << ", light "
             << (st.lightIntensity > 1e-4f ? (dot(st.lightDirection, st.sunDirection) > 0.999 ? "sun" : "moon") : "none")
             << " x" << st.lightIntensity / kCycleNoonSunIntensity << ", moon "
             << DayNightCycle::phaseName(st.moonAgeDays) << " (" << st.moonIllumination * 100.0f
             << "% lit), stars " << st.starVisibility << ", pollution " << lightPollution;
    return st;
}

}  // namespace engine
