#include "level_scene.h"

#include "engine/mesh_builder.h"
#include "engine/procgen/terrain.h"
#include "engine/procgen/erosion.h"
#include "engine/procgen/tree.h"
#include "engine/procgen/city/city.h"
#include "engine/procgen/noise.h"
#include "log.h"
#include <nlohmann/json.hpp>
#include <fstream>

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

int importMaterial(const json& ent, Scene& scene) {
    Vec3 albedo(0.8, 0.8, 0.8);
    double roughness = 0.5, metallic = 0.0;
    Vec3 emission(0, 0, 0);
    bool checkerboard = false;
    if (ent.contains("material")) {
        const auto& m = ent["material"];
        albedo = parseVec3(m.value("albedo", json()), albedo);
        roughness = m.value("roughness", roughness);
        metallic = m.value("metallic", metallic);
        emission = parseVec3(m.value("emission", json()), emission);
        // Current documents spell the flag as a bool (property-layer JSON);
        // pre-migration ones used a "flags" array. Read both.
        checkerboard = m.value("checkerboard", false);
        if (m.contains("flags"))
            for (const auto& f : m["flags"])
                checkerboard |= (f == "checkerboard");
    }
    if (emission.lengthSquared() > 0.0)
        return scene.addMaterial(Material::emissive(emission, 1.0));
    // Full PBR parameters, shaded with the viewer's GGX model in tracePath.
    Material mat = Material::pbr(albedo, metallic, roughness);
    mat.checkerboard = checkerboard;
    return scene.addMaterial(mat);
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
        Vec3 p = toWorld(v);
        if (slot == 0) { tri.v0 = p; tri.n0 = n; tri.c0 = v.color; tri.uv0[0] = v.u; tri.uv0[1] = v.v; }
        else if (slot == 1) { tri.v1 = p; tri.n1 = n; tri.c1 = v.color; tri.uv1[0] = v.u; tri.uv1[1] = v.v; }
        else { tri.v2 = p; tri.n2 = n; tri.c2 = v.color; tri.uv2[0] = v.u; tri.uv2[1] = v.v; }
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
void addTerrain(const json& t, Scene& scene) {
    TerrainParams tp = parseTerrainParams(t);
    Noise noise(t.value("seed", 0u));
    int matIdx = importMaterial(t, scene);
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
void addCity(const json& ent, const json& root, Scene& scene) {
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
    if (onTerrain && root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(parseTerrainParams(root["terrain"]));
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        Real base = cp.baseY;
        cp.groundAt = [tp, noise, base](const Vec2& p) {
            return base + terrainHeight(*tp, *noise, p.x, p.y);
        };
    }

    CityModel m = generateCity(cp);
    auto bake = [&](const RenderMesh& mesh, float metallic, float roughness) {
        if (mesh.vertices.empty()) return;
        int mi = scene.addMaterial(Material::pbr(Vec3(1, 1, 1), metallic, roughness));
        addMeshAsTriangles(mesh, Vec3(), Quat::identity(), Vec3(1, 1, 1), mi, scene);
    };
    for (const RenderMesh& part : m.parts) {
        RenderMaterial rm = materialFor(static_cast<PartId>(part.materialIndex), Vec3(1, 1, 1));
        bake(part, rm.metallic, rm.roughness);
    }
    bake(m.roads, 0.0f, 0.9f);
    bake(m.pavement, 0.0f, 0.95f);  // flat graded aprons + curbs
    bake(m.ground, 0.0f, 1.0f);
    bake(m.props, 0.0f, 0.85f);     // trees (vertex-coloured)
    LOG_INFO << "City: " << m.buildings.size() << " buildings, " << m.blockCount
             << " blocks, " << m.treeCount << " trees";
}

}  // namespace

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

    // Procedural terrain (top-level block, regenerated from its recipe).
    if (root.contains("terrain")) addTerrain(root["terrain"], scene);

    int skipped = 0;
    for (const auto& ent : root.value("entities", json::array())) {
        if (ent.contains("mesh")) {
            skipped++;   // glTF models aren't tessellated offline yet
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
            addCity(ent, root, scene);
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
        int matIdx = importMaterial(ent, scene);

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
