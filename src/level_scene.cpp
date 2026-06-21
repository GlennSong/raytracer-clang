#include "level_scene.h"

#include "engine/mesh_builder.h"
#include "engine/procgen/terrain.h"
#include "engine/procgen/erosion.h"
#include "engine/procgen/tree.h"
#include "engine/procgen/surface_maps.h"
#include "engine/model_importer.h"
#include "engine/procgen/city/city.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain_field.h"   // HeightField (level ground sampler)
#include "engine/procgen/proc_model.h"      // ProcModel (script model cache)
#include "log.h"
#include <unordered_map>
#include <array>
#include <sstream>
#include <nlohmann/json.hpp>
#include <fstream>
#ifdef RT_ENABLE_SCRIPTING
#include "engine/scripting/script_vm.h"
#include "engine/scripting/procgen_bindings.h"
#endif

using json = nlohmann::json;

namespace engine {

namespace {

Vec3 parseVec3(const json& j, Vec3 fallback = Vec3()) {
    if (!j.is_array() || j.size() != 3) return fallback;
    return Vec3(j[0].get<Real>(), j[1].get<Real>(), j[2].get<Real>());
}

Quat parseOrientation(const json& ent) {
    if (!ent.contains("orientation")) return Quat::identity();
    const auto& o = ent["orientation"];
    Vec3 axis = parseVec3(o.value("axis", json()), Vec3(0, 1, 0));
    return Quat::fromAxisAngle(axis, degreesToRadians(o.value("angleDeg", 0.0)));
}

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

// Parse a terrain JSON block into TerrainParams (shared by addTerrain and the
// city's ground-height sampler, so the city drapes onto the exact same surface).
TerrainParams parseTerrainParams(const json& t) {
    TerrainParams tp;
    tp.size        = t.value("size", tp.size);
    tp.resolution  = t.value("resolution", tp.resolution);
    tp.heightScale = t.value("heightScale", tp.heightScale);
    tp.noiseScale  = t.value("noiseScale", tp.noiseScale);
    tp.octaves     = t.value("octaves", tp.octaves);
    tp.warp        = t.value("warp", tp.warp);
    tp.mountainHeight = t.value("mountainHeight", tp.mountainHeight);
    tp.mountainScale  = t.value("mountainScale", tp.mountainScale);
    tp.mountainMaskScale = t.value("mountainMaskScale", tp.mountainMaskScale);
    tp.mountainMaskLo = t.value("mountainMaskLo", tp.mountainMaskLo);
    tp.mountainMaskHi = t.value("mountainMaskHi", tp.mountainMaskHi);
    if (t.contains("rangeSpine") && t["rangeSpine"].is_array()) {
        std::vector<Vec3> ctl;
        for (const auto& pt : t["rangeSpine"])
            if (pt.is_array() && pt.size() >= 2)
                ctl.push_back(Vec3(pt[0].get<double>(), 0.0, pt[1].get<double>()));
        tp.rangeSpine = sampleRangeSpine(ctl);
    }
    tp.rangeWidth = t.value("rangeWidth", tp.rangeWidth);
    tp.rangeHeight = t.value("rangeHeight", tp.rangeHeight);
    tp.rangeVariation = t.value("rangeVariation", tp.rangeVariation);
    if (t.contains("range") && t["range"].is_object()) {
        const auto& r = t["range"];
        tp.rangeRidges = buildRangeRidges(
            r.value("length", 60.0f), r.value("branchAngle", 38.0f),
            r.value("falloff", 0.55f), r.value("leaderFalloff", 0.92f),
            r.value("iterations", 5), r.value("height", 130.0f),
            r.value("depthFalloff", 0.62f), r.value("angleJitter", 12.0f),
            r.value("seed", 0u));
        tp.rangeWidth = r.value("width", tp.rangeWidth);
    }
    return tp;
}

// A procedural terrain block: regenerate the same mesh the engine does and add
// it (vertex color carries the slope/height coloring; material albedo
// multiplies it, so a white albedo lets the baked color show).
void addTerrain(const json& t, Scene& scene, const MaterialTable& materials,
                const std::vector<TerrainFlatten>& flatten = {}) {
    TerrainParams tp = parseTerrainParams(t);
    tp.flatten = flatten;   // city cut/fill so the ground meets the roads/blocks
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
    if (t.value("erode", false)) {
        // Bake the analytic field to a grid, erode it (drainage detail), mesh it.
        Heightmap hm = bakeHeightmap(tp, noise);
        ErosionParams ep;
        ep.seed = t.value("seed", 0u) + 1234u;
        ep.droplets = t.value("erodeDroplets", ep.droplets);
        ep.erodeRadius = t.value("erodeRadius", ep.erodeRadius);
        ep.thermalIterations = t.value("erodeThermal", ep.thermalIterations);
        ep.talus = t.value("erodeTalus", ep.talus);
        erode(hm, ep);
        addMeshAsTriangles(generateTerrainMesh(hm), Vec3(), Quat::identity(),
                           Vec3(1, 1, 1), matIdx, scene);
    } else {
        addMeshAsTriangles(generateTerrain(tp, noise), Vec3(), Quat::identity(),
                           Vec3(1, 1, 1), matIdx, scene);
    }
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
    TreeParams tp;
    uint32_t seed = 0;
    if (ent.contains("tree")) {
        const auto& j = ent["tree"];
        tp.iterations      = j.value("iterations", tp.iterations);
        tp.trunkLength     = j.value("trunkLength", tp.trunkLength);
        tp.lengthFalloff   = j.value("lengthFalloff", tp.lengthFalloff);
        tp.leaderFalloff   = j.value("leaderFalloff", tp.leaderFalloff);
        tp.branchAngle     = j.value("branchAngle", tp.branchAngle);
        tp.angleJitter     = j.value("angleJitter", tp.angleJitter);
        tp.branchesPerNode = j.value("branchesPerNode", tp.branchesPerNode);
        tp.phyllotaxis     = j.value("phyllotaxis", tp.phyllotaxis);
        tp.terminalFraction = j.value("terminalFraction", tp.terminalFraction);
        tp.terminalForks   = j.value("terminalForks", tp.terminalForks);
        tp.droop           = j.value("droop", tp.droop);
        tp.wander          = j.value("wander", tp.wander);
        tp.rootCount       = j.value("rootCount", tp.rootCount);
        tp.rootSpread      = j.value("rootSpread", tp.rootSpread);
        tp.leafClump       = j.value("leafClump", tp.leafClump);
        tp.maxLeafCards    = j.value("maxLeafCards", tp.maxLeafCards);
        tp.tipRadius       = j.value("tipRadius", tp.tipRadius);
        tp.pipeExponent    = j.value("pipeExponent", tp.pipeExponent);
        tp.radiusScale     = j.value("radiusScale", tp.radiusScale);
        tp.ringSegments    = j.value("ringSegments", tp.ringSegments);
        tp.leaves          = j.value("leaves", tp.leaves);
        tp.leafSize        = j.value("leafSize", tp.leafSize);
        tp.leavesPerTip    = j.value("leavesPerTip", tp.leavesPerTip);
        tp.leafThickness   = j.value("leafThickness", tp.leafThickness);
        tp.barkColor       = parseVec3(j.value("barkColor", json()), tp.barkColor);
        tp.leafColor       = parseVec3(j.value("leafColor", json()), tp.leafColor);
        seed               = j.value("seed", 0u);
    }

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

// A procedural city (shape:"city", ADR-0038): regenerate the same CityModel the
// engine does and bake it. Each material part uses a white albedo so the baked
// per-vertex color shows (the tree convention); the material carries only
// metallic/roughness, so glass reads reflective. The whole city is placed at the
// entity position (its XZ centre, baseY = position.y).
// Build the CityModel from an entity recipe, draping onto the level terrain when
// the recipe asks for it. Split from the baking step so the loader can pull the
// model's terrain cut/fill footprints (model.flatten) out and feed them into the
// terrain mesh *before* it's built (the terrain has to know where the city
// levels it). isCity()/onTerrain let the loader decide whether to pre-generate.
bool cityIsOnTerrain(const json& ent) {
    return ent.value("shape", std::string()) == "city" && ent.contains("city") &&
           ent["city"].value("onTerrain", false);
}

CityModel generateCityModel(const json& ent, const json& root) {
    CityParams cp;
    Vec3 pos = parseVec3(ent.value("position", json()));
    cp.center = {pos.x, pos.z};
    cp.baseY = pos.y;
    bool onTerrain = false;
    if (ent.contains("city")) {
        const auto& j = ent["city"];
        cp.extent         = j.value("extent", cp.extent);
        cp.cellSize       = j.value("cellSize", cp.cellSize);
        cp.roadJitter     = j.value("roadJitter", cp.roadJitter);
        cp.sidewalk       = j.value("sidewalk", cp.sidewalk);
        cp.downtownRadius = j.value("downtownRadius", cp.downtownRadius);
        cp.midtownRadius  = j.value("midtownRadius", cp.midtownRadius);
        cp.parkFraction   = j.value("parkFraction", cp.parkFraction);
        cp.buildChance    = j.value("buildChance", cp.buildChance);
        cp.scatterTrees   = j.value("scatterTrees", cp.scatterTrees);
        cp.seed           = j.value("seed", cp.seed);
        onTerrain         = j.value("onTerrain", false);
    }

    // City Arena (ADR-0038 §6): drape onto the level's terrain. The shared_ptr
    // keeps the params/noise alive for the sampler closure the model may hold.
    // The sampler reads the *base* terrain (no flatten) so the city decides its
    // grades from natural ground; its flatten footprints then cut the mesh.
    if (onTerrain && root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(parseTerrainParams(root["terrain"]));
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        Real base = cp.baseY;
        cp.groundAt = [tp, noise, base](const Vec2& p) {
            return base + terrainHeight(*tp, *noise, p.x, p.y);
        };
    }
    return generateCity(cp);
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

void bakeCityModel(const CityModel& m, Scene& scene) {
    using S = RenderMaterial::Surface;
    SurfaceTexCache texCache;   // one bake per surface, shared across the city
    auto bake = [&](const RenderMesh& mesh, float metallic, float roughness,
                    S surface) {
        if (mesh.vertices.empty()) return;
        Material mat = Material::pbr(Vec3(1, 1, 1), metallic, roughness);
        if (surface != S::None)   // bake + bind the procedural PBR texture set
            bindSurfaceTextures(mat, surface, scene, 1337u, 256, &texCache);
        int mi = scene.addMaterial(mat);
        addMeshAsTriangles(mesh, Vec3(), Quat::identity(), Vec3(1, 1, 1), mi, scene);
    };
    for (const RenderMesh& part : m.parts) {
        RenderMaterial rm = materialFor(static_cast<PartId>(part.materialIndex), Vec3(1, 1, 1));
        bake(part, rm.metallic, rm.roughness, rm.surface());
    }
    bake(m.roads, 0.0f, 0.9f, S::Asphalt);
    bake(m.pavement, 0.0f, 0.95f, S::Pavement);   // sidewalk slabs
    bake(m.ground, 0.0f, 1.0f, S::None);
    bake(m.props, 0.0f, 0.85f, S::None);          // lamps, signals, tree pits

    // Instanced props (ADR-0041): each group is one BLAS proto + N placements.
    for (const CityInstanceGroup& g : m.instanceGroups) {
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
        for (const Mat4& xf : g.transforms) scene.addInstance(proto, xf);
    }
    LOG_INFO << "City: " << m.buildings.size() << " buildings, " << m.blockCount
             << " blocks, " << m.treeCount << " trees";
}

}  // namespace

#ifdef RT_ENABLE_SCRIPTING
namespace {
std::string readTextFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
// Resolve a script entity's `file`: as given, else under assets/scripts/, else
// level-relative. Returns the source, or empty if none found.
std::string loadScriptCode(const std::string& file, const std::string& levelDir) {
    for (const std::string& p : {file, std::string("assets/scripts/") + file,
                                 levelDir + "/" + file}) {
        std::string c = readTextFile(p);
        if (!c.empty()) return c;
    }
    return {};
}
}  // namespace
#endif

// Add a baked TextureData to the Scene as a Texture; -1 if empty.
static int addProcTexture(const TextureData& td, Scene& scene) {
    if (td.pixels.empty()) return -1;
    Texture tex;
    tex.width = td.width; tex.height = td.height; tex.channels = td.channels;
    tex.pixels = td.pixels;
    return scene.addTexture(std::move(tex));
}

void bakeProcModel(const ProcModel& m, Scene& scene) {
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
        }
        int mi = scene.addMaterial(mat);
        addMeshAsTriangles(part.mesh, Vec3(), Quat::identity(), Vec3(1, 1, 1), mi, scene);
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
        for (const Mat4& xf : g.transforms) scene.addInstance(proto, xf);
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

    // A city draped on the terrain has to be generated BEFORE the terrain mesh:
    // it computes its road/block grades from the natural ground, then hands back
    // cut/fill footprints (flatten) that the terrain is built around so the
    // ground meets the carriageways instead of poking through. Pre-generate the
    // first such city here; the entity loop reuses the cached model (keyed by
    // pointer) so it isn't generated twice.
    const json* cityEnt = nullptr;
    CityModel cityModel;
    std::vector<TerrainFlatten> cityFlatten;
    for (const auto& ent : root.value("entities", json::array())) {
        if (cityIsOnTerrain(ent)) {
            cityEnt = &ent;
            cityModel = generateCityModel(ent, root);
            cityFlatten = cityModel.flatten;
            break;
        }
    }

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
    HeightField levelGround;
    if (root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(parseTerrainParams(root["terrain"]));
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
    // flat under the city/script cut-fill footprints.
    if (root.contains("terrain")) {
        std::vector<TerrainFlatten> allFlatten = cityFlatten;
        allFlatten.insert(allFlatten.end(), scriptFlatten.begin(), scriptFlatten.end());
        addTerrain(root["terrain"], scene, materials, allFlatten);
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
            bool baked = false;
            for (auto& pre : scriptCache)            // pre-run on-terrain recipe
                if (pre.first == &ent) { bakeProcModel(pre.second, scene); baked = true; break; }
            if (!baked) {
                ProcModel model;
                if (runScript(ent, model)) bakeProcModel(model, scene);
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
        // Procedural city (ADR-0038): roads + buildings baked from the recipe,
        // optionally draped on the level's terrain (the City Arena).
        if (ent.value("shape", std::string()) == "city") {
            if (&ent == cityEnt) bakeCityModel(cityModel, scene);     // pre-generated
            else bakeCityModel(generateCityModel(ent, root), scene);  // flat city
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
