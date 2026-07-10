#include "level_loader.h"
#include "mesh_builder.h"
#include "asset_manager.h"
#include "procgen/terrain.h"
#include "procgen/city/city.h"
#include "procgen/city/city_lots.h"  // grow buildings on the road net's blocks (ADR-0066)
#include "procgen/city/road_net.h"   // editor-authored roads (shape:"road")
#include "procgen/city/block_grade.h" // grade blocks to their streets (ADR-0075 P2)
#include "procgen/city/road_network.h" // extractBlocks (block grading)
#include "procgen/city/district.h"   // generated road districts (shape:"road" with "generate")
#include "procgen/city/road_constraints.h"   // applyConstraints — bake roundabouts into the graph
#include "procgen/city/road_mesh.h"   // triangulatePolygon (building prism colliders)
#include "procgen/city/water_mesh.h"  // buildWaterMesh (ocean/lake surface)
#include "procgen/erosion.h"
#include "procgen/lsystem.h"
#include "procgen/tree.h"
#include "procgen/surface_maps.h"
#include "procgen/rock.h"
#include "procgen/scatter.h"
#include "procgen/terrain_field.h"   // HeightField (level ground sampler)
#include "procgen/proc_model.h"      // ProcModel (script model cache)
#ifdef RT_ENABLE_SCRIPTING
#include "scripting/script_vm.h"
#include "scripting/procgen_bindings.h"
#include "scripting/vehicle_spec.h"
#endif
#include "model_importer.h"
#include <random>
#include <sstream>
#include "components.h"
#include "property_json.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <unordered_map>
#include <array>
#include <iterator>
#include <tuple>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace engine {

static Vec3 parseVec3(const json& j, Vec3 fallback = Vec3()) {
    if (!j.is_array() || j.size() < 3) return fallback;
    return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

// Default every road recipe's buildability `sea_level` (and beach reserve) from
// the level's top-level "water" block, so the coast's water and the city's
// land-gate share ONE number. A recipe that sets its own sea_level keeps it.
// Mutates `root` in place; a no-op when there's no water block. Shared by both
// loaders' road pre-pass and real build so they gate on an identical graph.
static void propagateWaterSeaLevel(json& root) {
    if (!root.contains("water")) return;
    const json& w = root["water"];
    double sea = w.value("seaLevel", 0.0);
    double beach = w.value("beachRise", 2.5);
    // The terrain colours its coast by this same sea level (beach/rock/sea floor).
    if (root.contains("terrain") && root["terrain"].is_object() &&
        !root["terrain"].contains("seaLevel"))
        root["terrain"]["seaLevel"] = sea;
    // Every road recipe gates buildability on it (roads/blocks stay on land).
    if (!root.contains("entities") || !root["entities"].is_array()) return;
    for (auto& ent : root["entities"]) {
        if (ent.value("shape", std::string()) != "road" || !ent.contains("road"))
            continue;
        json& road = ent["road"];
        if (!road.contains("generate") || !road["generate"].is_object()) continue;
        json& g = road["generate"];
        if (!g.contains("sea_level")) g["sea_level"] = sea;
        if (!g.contains("beach_rise")) g["beach_rise"] = beach;
    }
}

// Applies a material block onto `mat` through the property layer: described
// fields only, missing keys leave values untouched (so a partial block acts
// as an override, e.g. on a glTF's imported materials). Levels written before
// the JSON-visitor migration spelled the checkerboard as a "flags" array —
// keep reading that form.
static void applyMaterial(const json& j, RenderMaterial& mat) {
    JsonReadVisitor reader(j);
    describeProperties(mat, reader);
    if (j.contains("flags")) {
        for (auto& flag : j["flags"]) {
            if (flag.get<std::string>() == "checkerboard")
                mat.flags |= RenderMaterial::FLAG_CHECKERBOARD;
        }
    }
    // Procedural surface from the library ("brick", "concrete", "asphalt", ...).
    if (j.value("brick", false)) mat.setSurface(RenderMaterial::Surface::Brick);
    if (j.contains("surface"))
        mat.setSurface(surfaceFromName(j["surface"].get<std::string>()));
}

static RenderMaterial parseMaterial(const json& j) {
    RenderMaterial mat;
    applyMaterial(j, mat);
    return mat;
}

// Rewrite a mesh's UVs (and tangent) to a world-planar tiling frame, so a baked
// tiling texture set repeats at human scale over world-space geometry: v = height
// up a wall, u = horizontal run; horizontal faces lay it in XZ. The tangent is
// set along U so a tangent-space normal map resolves correctly. Matches the
// offline tracer's surfFrame (ADR-0039 Phase B).
static void applyWorldPlanarUVs(RenderMesh& mesh, double scale) {
    for (Vertex& vert : mesh.vertices) {
        const Vec3& p = vert.position;
        const Vec3& n = vert.normal;
        double u, v; Vec3 T;
        if (std::fabs(n.y) > 0.5) {
            u = p.x * scale; v = p.z * scale; T = Vec3(1, 0, 0);
        } else {
            double tx = n.z, tz = -n.x, tl = std::sqrt(tx * tx + tz * tz);
            if (tl < 1e-6) { tx = 1; tz = 0; tl = 1; }
            tx /= tl; tz /= tl;
            u = (p.x * tx + p.z * tz) * scale; v = p.y * scale; T = Vec3(tx, 0, tz);
        }
        vert.u = static_cast<float>(u);
        vert.v = static_cast<float>(v);
        vert.tangent = T;
    }
}

// One bake+upload per surface, shared across a level load (every brick entity
// binds the same uploaded set). The cache keys on the surface id.
using SurfaceTexCache = std::unordered_map<int, std::array<TextureHandle, 4>>;
static std::array<TextureHandle, 4> bakeSurfaceTextures(
    Renderer& renderer, RenderMaterial::Surface surf, SurfaceTexCache& cache) {
    int id = static_cast<int>(surf);
    auto it = cache.find(id);
    if (it != cache.end()) return it->second;
    SurfaceMaps mp = surfaceMaps(surf, 256, 1337u);
    auto up = [&](const TextureData& td) {
        return renderer.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };
    std::array<TextureHandle, 4> h{up(mp.albedo), up(mp.normal), up(mp.mr), up(mp.ao)};
    cache[id] = h;
    return h;
}
// Bind a baked set onto a material. The surface flag is kept as provenance (so
// it round-trips through save/load); the shader skips the analytic applySurface
// when an albedo map is present, so the textures drive the look, not the flag.
static void bindSurfaceMaps(RenderMaterial& mat, const std::array<TextureHandle, 4>& h) {
    mat.albedoMap = h[0];
    mat.normalMap = h[1];
    mat.metallicRoughnessMap = h[2];
    mat.aoMap = h[3];
}

// A named material library: the level's top-level "materials" table, so entities
// can reference a shared material by name ("material": "brickWall") instead of
// repeating an inline block (ADR-0039).
using MaterialTable = std::unordered_map<std::string, RenderMaterial>;

static MaterialTable buildMaterialTable(const json& root) {
    MaterialTable table;
    if (root.contains("materials") && root["materials"].is_object())
        for (auto it = root["materials"].begin(); it != root["materials"].end(); ++it)
            table[it.key()] = parseMaterial(it.value());
    return table;
}

// Resolve an entity's material value onto `base`: a string references the named
// table; an object is an inline material/override; absent leaves base unchanged.
static RenderMaterial resolveMaterial(const json& matJson, const MaterialTable& table,
                                      RenderMaterial base = RenderMaterial()) {
    if (matJson.is_string()) {
        auto it = table.find(matJson.get<std::string>());
        if (it != table.end()) return it->second;
        LOG_WARN << "Material reference '" << matJson.get<std::string>()
                 << "' not found in the materials table; using default";
        return base;
    }
    if (matJson.is_object()) applyMaterial(matJson, base);
    return base;
}

// Primitive meshes are deduped + refcounted by the AssetManager: identical
// shape+size share one GPU upload across the whole level (and across loads,
// since the manager persists), and the caller clears it before each load so the
// previous level's meshes are freed.
static MeshHandle getOrCreateMesh(const std::string& shape, const json& sizeJ,
                                  AssetManager& assets) {
    MeshHandle handle = assets.acquirePrimitive(shape, parseVec3(sizeJ, Vec3(1, 1, 1)));
    if (!handle.valid()) LOG_ERROR << "Unknown shape: " << shape;
    return handle;
}

static TerrainParams parseTerrainParams(const json& t);   // defined below

static Collider buildCollider(const std::string& shape, const json& sizeJ,
                               const json& physics) {
    Vec3 sz = parseVec3(sizeJ, Vec3(1, 1, 1));
    Collider c;

    if (shape == "sphere") {
        c.shape = ColliderShape::Sphere;
        c.radius = sz.x;
    } else if (shape == "capsule") {
        c.shape = ColliderShape::Capsule;
        c.radius = sz.x;
        c.halfHeight = sz.y * 0.5;
    } else {
        c.shape = ColliderShape::Box;
        c.halfExtent = sz * 0.5;
    }

    if (physics.contains("friction"))    c.friction    = physics["friction"].get<double>();
    if (physics.contains("restitution")) c.restitution = physics["restitution"].get<double>();
    return c;
}

static void createEntityCommon(Entity e, const json& ent, World& world) {
    Transform t;
    if (ent.contains("position"))
        t.position = parseVec3(ent["position"]);
    if (ent.contains("scale"))
        t.scale = parseVec3(ent["scale"], Vec3(1, 1, 1));
    if (ent.contains("orientation")) {
        auto& o = ent["orientation"];
        Vec3 axis = parseVec3(o["axis"], Vec3(0, 1, 0));
        Real angle = degreesToRadians(o.value("angleDeg", 0.0));
        t.orientation = Quat::fromAxisAngle(axis, angle);
    }
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
}

static void addPhysics(Entity e, const json& ent, const std::string& shape,
                       World& world) {
    if (!ent.contains("physics")) return;
    auto& phys = ent["physics"];

    auto sizeJ = ent.contains("size") ? ent["size"] : json::array({1, 1, 1});
    Collider c = buildCollider(shape, sizeJ, phys);
    world.add<Collider>(e, c);

    RigidBody rb;
    std::string motion = phys.value("motion", "static");
    if (motion == "dynamic")        rb.motion = BodyMotion::Dynamic;
    else if (motion == "kinematic")  rb.motion = BodyMotion::Kinematic;
    else                             rb.motion = BodyMotion::Static;
    rb.lockRotation = phys.value("lockRotation", false);
    world.add<RigidBody>(e, rb);
}

// Authoring provenance for the editor's LevelWriter (docs/edit-mode-plan.md).
static SourceSpec buildSourceSpec(const json& ent, const std::string& shape) {
    SourceSpec spec;
    spec.id = ent.value("id", 0u);
    spec.parentId = ent.value("parent", 0u);
    spec.name = ent.value("name", std::string());
    spec.shape = shape;
    spec.size = parseVec3(ent.value("size", json()), Vec3(1, 1, 1));
    if (ent.contains("material") && ent["material"].is_string())
        spec.materialName = ent["material"].get<std::string>();   // shared asset ref
    if (ent.contains("physics")) {
        const auto& phys = ent["physics"];
        spec.hasPhysics = true;
        spec.motion = phys.value("motion", "static");
        spec.friction = phys.value("friction", 0.5);
        spec.restitution = phys.value("restitution", 0.0);
        spec.lockRotation = phys.value("lockRotation", false);
    }
    return spec;
}

// Spawn a procgen "document" entity: a transform + SourceSpec carrying the
// recipe, with no mesh of its own. LevelWriter only serialises SourceSpec
// entities, so this is what lets a generated scene survive the editor's
// save-then-reload (Play): the render/instance entities a generator emits are
// runtime companions, dropped on save and regenerated from this recipe on load.
// The city and script (ADR-0042) loaders anchor on one of these; the tree folds
// the same role into its bark entity (which is also a Renderable).
static Entity spawnDocumentEntity(const json& ent, const std::string& shape,
                                  const std::string& recipe, World& world) {
    Entity doc = world.create();
    createEntityCommon(doc, ent, world);
    SourceSpec spec = buildSourceSpec(ent, shape);
    spec.recipe = recipe;
    world.add<SourceSpec>(doc, spec);
    return doc;
}

// An editor-authored road (shape:"road", ADR-0049): a RoadNet (control nodes +
// look) baked to a carriageway mesh and carried as a first-class DOCUMENT entity,
// so the inspector can widen it and the viewport can drag its nodes — the editor
// regenerates the mesh through onEdited. Drapes on the level terrain (`ground`).
static void loadRoadEntity(const json& ent, World& world, AssetManager& assets,
                           int index, const HeightField* ground) {
    const json roadBlock = ent.contains("road") ? ent["road"] : json::object();
    RoadNet net = roadNetFromJson(roadBlock);

    // A GENERATED road: instead of authored nodes, "generate" runs a procgen graph (buildDistrict)
    // and fills the RoadNet's nodes/edges from it — so the generated city IS a real, editable RoadNet
    // (the editor's node/tangent handles + buildRoadNetMesh's curves/markings/sidewalks/junctions),
    // not a baked mesh. The recipe round-trips via SourceSpec; the look comes from the road block.
    // A GENERATED road runs its recipe into the net's graph — shared with the editor's regenerate
    // (applyGenerateRecipe), so a generated city stays a real editable RoadNet whose recipe
    // round-trips instead of baking to frozen geometry (road-network-v2-plan T2.1).
    if (ground) net.heightAt = *ground;   // BEFORE generate: terrain-aware recipes (metro) gate on it
    if (roadBlock.contains("generate"))
        applyGenerateRecipe(net, roadBlock["generate"]);

    Entity e = world.create();
    createEntityCommon(e, ent, world);

    SourceSpec spec = buildSourceSpec(ent, "road");
    // Preserve the ORIGINAL authored block (a "generate" recipe, or hand-authored nodes) as the
    // saved form — NOT the baked net — so load→save is a no-op and a generated road keeps its recipe
    // instead of being frozen into geometry. The editor's regenerateRoad re-bakes this only when the
    // road is actually edited (a node drag / width change), which is when baking is correct.
    spec.recipe = roadBlock.dump();
    world.add<SourceSpec>(e, spec);
    world.add<RoadNet>(e, net);                      // the editable source of truth

    Renderable r;
    r.renderLayer = engine::LayerRoads;              // debug layer toggle
    r.material.albedo = Vec3(1, 1, 1);               // hue carried in vertex colour
    r.material.roughness = 0.93f;
    if (net.markings)                                // lane paint via the surface shader
        r.material.setSurface(RenderMaterial::Surface::RoadMarkings);
    RenderMesh mesh = buildRoadNetMesh(net);
    if (!mesh.vertices.empty())
        r.mesh = assets.acquireMesh(mesh, "road:" + std::to_string(index));
    world.add<Renderable>(e, r);

    // Static collision from the carriageway geometry (ADR-0059): without this a
    // road has no collider of its own — ground roads borrow the terrain's, but an
    // elevated bridge DECK has nothing under it, so cars fall through. Build a
    // MeshCollider from the same triangles (deck + ramps + piers) so the player's
    // car (and physics bodies) drive on roads, the overpass included.
    if (!mesh.vertices.empty()) {
        MeshCollider mc;
        mc.vertices.reserve(mesh.vertices.size());
        for (const Vertex& v : mesh.vertices) mc.vertices.push_back(v.position);
        mc.indices = mesh.indices;
        mc.friction = 0.85;
        world.add<MeshCollider>(e, mc);
    }
}

// A hero parametric tree (shape: "tree"): a real, collidable object you can
// bounce off or shoot at, distinct from the instanced vegetation scatter. The
// bark is one mesh (procedural bark texture, opaque) with a static triangle
// MeshCollider; the leaves are a second entity at the same transform (alpha-cut
// cards, no collision). docs/lsystem-botany-plan.md.
static void loadTreeEntity(const json& ent, World& world, Renderer& renderer,
                           AssetManager& assets, int index) {
    TreeParams tp;
    uint32_t seed = 0;
    if (ent.contains("tree")) {
        const auto& j = ent["tree"];
        tp.iterations     = j.value("iterations", tp.iterations);
        tp.trunkLength    = j.value("trunkLength", tp.trunkLength);
        tp.lengthFalloff  = j.value("lengthFalloff", tp.lengthFalloff);
        tp.leaderFalloff  = j.value("leaderFalloff", tp.leaderFalloff);
        tp.branchAngle    = j.value("branchAngle", tp.branchAngle);
        tp.angleJitter    = j.value("angleJitter", tp.angleJitter);
        tp.branchesPerNode = j.value("branchesPerNode", tp.branchesPerNode);
        tp.phyllotaxis    = j.value("phyllotaxis", tp.phyllotaxis);
        tp.terminalFraction = j.value("terminalFraction", tp.terminalFraction);
        tp.terminalForks  = j.value("terminalForks", tp.terminalForks);
        tp.droop          = j.value("droop", tp.droop);
        tp.wander         = j.value("wander", tp.wander);
        tp.rootCount      = j.value("rootCount", tp.rootCount);
        tp.rootSpread     = j.value("rootSpread", tp.rootSpread);
        tp.leafClump      = j.value("leafClump", tp.leafClump);
        tp.maxLeafCards   = j.value("maxLeafCards", tp.maxLeafCards);
        tp.tipRadius      = j.value("tipRadius", tp.tipRadius);
        tp.pipeExponent   = j.value("pipeExponent", tp.pipeExponent);
        tp.radiusScale    = j.value("radiusScale", tp.radiusScale);
        tp.ringSegments   = j.value("ringSegments", tp.ringSegments);
        tp.leaves         = j.value("leaves", tp.leaves);
        tp.leafSize       = j.value("leafSize", tp.leafSize);
        tp.leavesPerTip   = j.value("leavesPerTip", tp.leavesPerTip);
        tp.leafThickness  = j.value("leafThickness", tp.leafThickness);
        tp.barkColor      = parseVec3(j.value("barkColor", json()), tp.barkColor);
        tp.leafColor      = parseVec3(j.value("leafColor", json()), tp.leafColor);
        seed              = j.value("seed", 0u);
    }

    TreeMesh tm = growTree(tp, seed);
    if (tm.branches.vertices.empty()) return;

    auto upload = [&](const TextureData& td) -> TextureHandle {
        if (td.pixels.empty()) return TextureHandle{};
        return renderer.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };

    const std::string key = "tree:" + std::to_string(index) + ":" + std::to_string(seed);

    // Bark entity: textured, collidable. It is the tree's *document* entity —
    // it carries the SourceSpec (with the recipe) so the whole tree round-trips
    // through the LevelWriter; the leaf entity is a runtime companion (no
    // SourceSpec) regenerated from the recipe on load.
    {
        Entity e = world.create();
        createEntityCommon(e, ent, world);

        SourceSpec spec = buildSourceSpec(ent, "tree");
        if (ent.contains("tree")) spec.recipe = ent["tree"].dump();
        world.add<SourceSpec>(e, spec);

        Renderable r;
        r.mesh = assets.acquireMesh(tm.branches, key + ":bark");
        r.material.albedo = Vec3(1, 1, 1);   // bark color is baked into vertex color
        r.material.roughness = 1.0f;
        // Per-species bark relief: value pattern (modulates vertex color) + normal map.
        std::string styleName = ent.contains("tree")
            ? ent["tree"].value("barkStyle", std::string("oak")) : "oak";
        BarkMaps bm = barkMaps(barkStyleFromName(styleName), 256, seed);
        r.material.albedoMap = upload(bm.albedo);
        r.material.normalMap = upload(bm.normal);
        if (ent.contains("material")) applyMaterial(ent["material"], r.material);
        world.add<Renderable>(e, r);

        MeshCollider mc;
        mc.vertices = tm.collisionVertices;
        mc.indices = tm.collisionIndices;
        if (ent.contains("physics"))
            mc.friction = ent["physics"].value("friction", mc.friction);
        world.add<MeshCollider>(e, mc);
    }

    // Leaf entity: alpha-cut cards at the same transform, no collision.
    if (!tm.leaves.vertices.empty()) {
        Entity e = world.create();
        createEntityCommon(e, ent, world);

        Renderable r;
        r.mesh = assets.acquireMesh(tm.leaves, key + ":leaves");
        r.material.albedo = Vec3(1, 1, 1);   // leaf color is baked into vertex color
        r.material.roughness = 0.7f;
        r.material.albedoMap = upload(leafTexture(128));
        r.material.flags |= RenderMaterial::FLAG_ALPHA_TEST;
        world.add<Renderable>(e, r);
    }
}

// A procedural city (shape:"city", ADR-0038): spawn the generated geometry as
// renderable entities (one per material part + roads + trees + ground) and a
// static Box collider per building, so the player walks the streets and bumps
// into buildings. Optionally draped on the level's terrain (the City Arena),
// which already carries its own walk-surface MeshCollider.
// True for a city entity that drapes on the level terrain (so the loader knows
// to pre-generate it before building the terrain — its cut/fill footprints have
// to be baked into the terrain mesh).
static bool cityIsOnTerrain(const json& ent) {
    return ent.value("shape", std::string()) == "city" && ent.contains("city") &&
           ent["city"].value("onTerrain", false);
}

// Build the CityModel from an entity recipe (no ECS side effects). Separated from
// the spawn so the loader can pull model.flatten and grade the terrain to it
// before the terrain mesh is built.
static CityModel cityModelFromEntity(const json& ent, const json& root) {
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
        // District road tech (ADR-0066): real arterials + irregular streets whose
        // blocks feed the lot/building pipeline, instead of the regular grid.
        cp.districtRoads  = j.value("districtRoads", cp.districtRoads);
        cp.arterials      = j.value("arterials", cp.arterials);
        cp.blockSizeMin   = j.value("blockSizeMin", cp.blockSizeMin);
        cp.blockSizeMax   = j.value("blockSizeMax", cp.blockSizeMax);
        cp.arteryWidth    = j.value("arteryWidth", cp.arteryWidth);
        cp.streetWidth    = j.value("streetWidth", cp.streetWidth);
        cp.irregular      = j.value("irregular", cp.irregular);
    }
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

static void loadCityEntity(const json& ent, const json& root, World& world,
                           Renderer& renderer, AssetManager& assets, int index,
                           const CityModel* precomputed = nullptr) {
    (void)renderer;
    // Reuse the model the loader pre-generated for terrain cut/fill, or build it
    // now (flat cities, which need no terrain grading).
    CityModel local;
    if (!precomputed) local = cityModelFromEntity(ent, root);
    const CityModel& m = precomputed ? *precomputed : local;

    // Document entity: carries the SourceSpec (recipe) so the city round-trips
    // through the editor's save-then-load. LevelWriter only serialises entities
    // with a SourceSpec, so without this the city vanishes the instant Play
    // reloads the level — while the top-level "terrain" block survives, leaving
    // "just terrain". The render/collider entities below are runtime companions
    // (regenerated from this recipe on load), exactly like the tree's leaf entity.
    spawnDocumentEntity(ent, "city",
                        ent.contains("city") ? ent["city"].dump() : std::string(),
                        world);

    using Surface = RenderMaterial::Surface;
    const std::string key = "city:" + std::to_string(index);

    // One bake+upload per surface, shared across the whole city (every brick
    // building binds the same uploaded set). Mirrors the offline SurfaceTexCache.
    auto upload = [&](const TextureData& td) -> TextureHandle {
        return renderer.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };
    auto surfaceSet = [&, cache = std::unordered_map<int, std::array<TextureHandle, 4>>()]
                      (Surface surf) mutable -> std::array<TextureHandle, 4> {
        int id = static_cast<int>(surf);
        auto it = cache.find(id);
        if (it != cache.end()) return it->second;
        SurfaceMaps mp = surfaceMaps(surf, 256, 1337u);
        std::array<TextureHandle, 4> h{upload(mp.albedo), upload(mp.normal),
                                       upload(mp.mr), upload(mp.ao)};
        cache[id] = h;
        return h;
    };

    auto spawnMesh = [&](const RenderMesh& mesh, const std::string& tag,
                         float metallic, float roughness, Surface surface) {
        if (mesh.vertices.empty()) return;
        Entity e = world.create();
        Transform t;                       // city geometry is already world-space
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.material.albedo = Vec3(1, 1, 1);     // hue carried in vertex colour
        r.material.metallic = metallic;
        r.material.roughness = roughness;
        if (surface != Surface::None) {
            // Textured: world-planar tiling UVs (so the baked set tiles at human
            // scale, tangent aligned to U for the normal map), and bind the maps.
            RenderMesh tiled = mesh;
            applyWorldPlanarUVs(tiled, 1.0 / surfaceWorldTileSize(surface));
            r.mesh = assets.acquireMesh(tiled, key + ":" + tag);
            std::array<TextureHandle, 4> h = surfaceSet(surface);
            r.material.albedoMap = h[0];
            r.material.normalMap = h[1];
            r.material.metallicRoughnessMap = h[2];
            r.material.aoMap = h[3];
        } else {
            r.mesh = assets.acquireMesh(mesh, key + ":" + tag);
        }
        world.add<Renderable>(e, r);
    };
    for (std::size_t i = 0; i < m.parts.size(); ++i) {
        RenderMaterial rm = materialFor(static_cast<PartId>(m.parts[i].materialIndex), Vec3(1, 1, 1));
        spawnMesh(m.parts[i], "part" + std::to_string(i), rm.metallic, rm.roughness, rm.surface());
    }
    spawnMesh(m.roads, "roads", 0.0f, 0.9f, Surface::Asphalt);
    spawnMesh(m.pavement, "pavement", 0.0f, 0.95f, Surface::Pavement);
    spawnMesh(m.props, "props", 0.0f, 0.85f, Surface::None);
    spawnMesh(m.ground, "ground", 0.0f, 1.0f, Surface::None);

    // Instanced props (ADR-0041): each group becomes one InstanceGroup entity —
    // a shared mesh + material drawn over N world matrices (the existing
    // instanced-draw path), so the city's ~600 street trees are a handful of
    // batches, not baked geometry.
    for (std::size_t gi = 0; gi < m.instanceGroups.size(); ++gi) {
        const CityInstanceGroup& cg = m.instanceGroups[gi];
        if (cg.proto.vertices.empty() || cg.transforms.empty()) continue;
        InstanceGroup g;
        g.mesh = assets.acquireMesh(cg.proto, key + ":inst" + std::to_string(gi));
        g.material.albedo = Vec3(1, 1, 1);       // hue carried in vertex colour
        g.material.metallic = cg.metallic;
        g.material.roughness = cg.roughness;
        if (cg.alphaFoliage) {
            g.material.albedoMap = upload(leafTexture(128));
            g.material.flags |= RenderMaterial::FLAG_ALPHA_TEST;
        }
        g.transforms = cg.transforms;
        Vec3 centroid(0, 0, 0);
        for (const Mat4& t : cg.transforms)
            centroid = centroid + Vec3(t.m[0][3], t.m[1][3], t.m[2][3]);
        centroid = centroid / static_cast<Real>(cg.transforms.size());
        Real spread = 0;
        for (const Mat4& t : cg.transforms)
            spread = std::max(spread,
                              (Vec3(t.m[0][3], t.m[1][3], t.m[2][3]) - centroid).length());
        BoundingSphere mb = assets.meshBounds(g.mesh);
        g.boundsCenter = centroid;
        g.boundsRadius = spread + (mb.center.length() + mb.radius) * 1.5;
        world.add<InstanceGroup>(world.create(), g);
    }

    // Walk-surface collider: the flat block aprons + roads are a static triangle
    // mesh the player walks on (the curbed pads, level per block).
    {
        MeshCollider mc;
        mc.friction = 0.9;
        auto addTris = [&](const RenderMesh& rm) {
            for (std::size_t i = 0; i + 2 < rm.indices.size(); i += 3) {
                const Vec3& a = rm.vertices[rm.indices[i]].position;
                const Vec3& b = rm.vertices[rm.indices[i + 1]].position;
                const Vec3& c = rm.vertices[rm.indices[i + 2]].position;
                if (cross(b - a, c - a).length() < 1e-5) continue;   // skip degenerate
                uint32_t base = static_cast<uint32_t>(mc.vertices.size());
                mc.vertices.push_back(a);
                mc.vertices.push_back(b);
                mc.vertices.push_back(c);
                mc.indices.push_back(base);
                mc.indices.push_back(base + 1);
                mc.indices.push_back(base + 2);
            }
        };
        addTris(m.pavement);
        addTris(m.roads);
        if (!mc.indices.empty()) {
            Entity e = world.create();
            Transform t;
            world.add<Transform>(e, t);
            world.add<PrevTransform>(e, PrevTransform{t});
            world.add<MeshCollider>(e, mc);
        }
    }

    // A static Box collider per building (walk-into-it physics).
    for (const CityBuilding& b : m.buildings) {
        Entity e = world.create();
        Transform t;
        t.position = b.boxCenter;
        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), b.yaw);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Collider c;
        if (b.round) {
            // A vertical capsule is round in plan, so you can walk right up to the
            // cylinder wall instead of being held off by a circumscribing box.
            c.shape = ColliderShape::Capsule;
            c.radius = b.boxHalf.x;
            c.halfHeight = std::max(Real(0.1), b.boxHalf.y - b.boxHalf.x);
        } else {
            c.shape = ColliderShape::Box;
            c.halfExtent = b.boxHalf;
        }
        c.friction = 0.8;
        world.add<Collider>(e, c);
        RigidBody rb;
        rb.motion = BodyMotion::Static;
        world.add<RigidBody>(e, rb);
    }
    // Flat (no-terrain) city: a big static floor so the player has ground.
    if (!m.ground.vertices.empty()) {
        Vec3 pos = parseVec3(ent.value("position", json()));
        Real extent = 400, cellSize = 95;
        if (ent.contains("city")) {
            extent = ent["city"].value("extent", extent);
            cellSize = ent["city"].value("cellSize", cellSize);
        }
        Entity e = world.create();
        Transform t;
        t.position = Vec3(pos.x, pos.y - 0.5, pos.z);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Collider c;
        c.shape = ColliderShape::Box;
        c.halfExtent = Vec3(extent + cellSize, 0.5, extent + cellSize);
        c.friction = 0.9;
        world.add<Collider>(e, c);
        RigidBody rb;
        rb.motion = BodyMotion::Static;
        world.add<RigidBody>(e, rb);
    }
    LOG_INFO << "City (viewer): " << m.buildings.size() << " buildings, "
             << m.parts.size() << " draw parts, + colliders";
}

// Read a script entity's `file`: as given, else under assets/scripts/, else
// level-relative. Empty if none found.
static std::string loadScriptCode(const std::string& file, const std::string& levelDir) {
    for (const std::string& path : {file, std::string("assets/scripts/") + file,
                                    levelDir + "/" + file}) {
        std::ifstream f(path);
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            return ss.str();
        }
    }
    return {};
}

#ifdef RT_ENABLE_SCRIPTING
// Run a script entity's recipe into `out`. An on-terrain recipe gets the level's
// ground sampler injected as the `ground` global so it drapes + conforms the
// engine terrain (ADR-0044). Pure model build — no ECS side effects — so the
// loader can pre-run it for cut/fill footprints before the terrain is meshed.
static bool runScriptModel(const json& ent, const std::string& levelDir,
                           const HeightField* ground, ProcModel& out) {
    std::string file = ent.value("file", std::string());
    std::string code = file.empty() ? std::string() : loadScriptCode(file, levelDir);
    if (code.empty()) {
        LOG_WARN << "script entity: cannot read '" << file << "'";
        return false;
    }
    ScriptVM vm;
    openProcgenLibrary(vm);
    vm.setGlobalNumber("seed", ent.value("seed", 0.0));
    if (ent.contains("opts")) setRecipeArgs(vm, ent["opts"].dump());
    if (ground != nullptr) setGlobalHeightField(vm, "ground", *ground);
    std::string err;
    if (!runProcgenModelValue(vm, code, out, &err)) {
        LOG_ERROR << "script entity '" << file << "': " << err;
        return false;
    }
    return true;
}
#endif

// A Lua recipe entity (shape:"script", ADR-0042): run the recipe and spawn its
// composable model — parts as Renderable entities, instance groups as
// InstanceGroups. The realtime twin of level_scene's bakeProcModel, so the same
// city.lua that renders offline also populates the viewer/editor. `prebuilt` is
// a model the loader already ran (on-terrain pre-pass); `ground` injects the
// level terrain for an on-terrain recipe run here.
static void loadScriptEntity(const json& ent, const std::string& levelDir,
                             World& world, Renderer& renderer, AssetManager& assets,
                             int index, const ProcModel* prebuilt = nullptr,
                             const HeightField* ground = nullptr) {
#ifdef RT_ENABLE_SCRIPTING
    std::string file = ent.value("file", std::string());

    // Document entity: carries the SourceSpec (recipe) so the shape:"script"
    // entity round-trips through the editor's save-then-reload. Without it the
    // recipe is dropped on Play→save and the world comes back empty on reload —
    // the same failure the tree (bark) and city (doc) entities avoid. The
    // render/instance entities spawned below are runtime companions, regenerated
    // from this recipe on load. Created before the file read so even a recipe
    // that fails to load (transient missing file) still round-trips.
    {
        json recipe;
        recipe["file"] = file;
        if (ent.contains("seed")) recipe["seed"] = ent["seed"];
        // Carry the recipe's `opts` so they survive the editor's save→reload: the
        // script reads them via setRecipeArgs (`args`), so dropping them here makes
        // e.g. a radial city fall back to city.lua's default grid on the next load.
        if (ent.contains("opts")) recipe["opts"] = ent["opts"];
        spawnDocumentEntity(ent, "script", recipe.dump(), world);
    }

    ProcModel localModel;
    const ProcModel* modelPtr = prebuilt;
    if (modelPtr == nullptr) {
        if (!runScriptModel(ent, levelDir, ground, localModel)) return;
        modelPtr = &localModel;
    }
    const ProcModel& model = *modelPtr;
    const std::string key = "script:" + std::to_string(index);
    auto upload = [&](const TextureData& td) -> TextureHandle {
        return renderer.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };

    for (std::size_t i = 0; i < model.parts.size(); ++i) {
        const ProcPart& part = model.parts[i];
        if (part.mesh.vertices.empty()) continue;
        Entity e = world.create();
        Transform t;                          // recipe geometry is world-space
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.material.albedo = Vec3(1, 1, 1);    // hue carried in vertex colour
        r.material.metallic = part.material.metallic;
        r.material.roughness = part.material.roughness;
        // Analytic surface library (renderer.h Surface): an id evaluated in-shader
        // from the mesh's own UVs — e.g. RoadMarkings paints lane lines from the
        // road-local u/v baked onto the carriageway. Independent of texture maps,
        // so it rides whether or not the part is `textured`.
        if (part.material.surface != 0)
            r.material.setSurface(static_cast<RenderMaterial::Surface>(part.material.surface));
        if (part.material.textured) {
            RenderMesh tiled = part.mesh;
            double tile = part.material.tile > 1e-6 ? part.material.tile : 1.0;
            applyWorldPlanarUVs(tiled, 1.0 / tile);
            r.mesh = assets.acquireMesh(tiled, key + ":part" + std::to_string(i));
            if (!part.material.albedo.pixels.empty())
                r.material.albedoMap = upload(part.material.albedo);
            if (!part.material.normal.pixels.empty())
                r.material.normalMap = upload(part.material.normal);
        } else {
            r.mesh = assets.acquireMesh(part.mesh, key + ":part" + std::to_string(i));
        }
        world.add<Renderable>(e, r);
    }

    for (std::size_t gi = 0; gi < model.instances.size(); ++gi) {
        const ProcInstanceGroup& cg = model.instances[gi];
        if (cg.proto.vertices.empty() || cg.transforms.empty()) continue;
        InstanceGroup g;
        g.mesh = assets.acquireMesh(cg.proto, key + ":inst" + std::to_string(gi));
        g.material.albedo = Vec3(1, 1, 1);
        g.material.metallic = cg.metallic;
        g.material.roughness = cg.roughness;
        if (cg.alphaFoliage) {
            g.material.albedoMap = upload(leafTexture(128));
            g.material.flags |= RenderMaterial::FLAG_ALPHA_TEST;
        }
        g.transforms = cg.transforms;
        Vec3 centroid(0, 0, 0);
        for (const Mat4& tr : cg.transforms)
            centroid = centroid + Vec3(tr.m[0][3], tr.m[1][3], tr.m[2][3]);
        centroid = centroid / static_cast<Real>(cg.transforms.size());
        Real spread = 0;
        for (const Mat4& tr : cg.transforms)
            spread = std::max(spread,
                              (Vec3(tr.m[0][3], tr.m[1][3], tr.m[2][3]) - centroid).length());
        BoundingSphere mb = assets.meshBounds(g.mesh);
        g.boundsCenter = centroid;
        g.boundsRadius = spread + (mb.center.length() + mb.radius) * 1.5;
        world.add<InstanceGroup>(world.create(), g);
    }

    // Physics colliders (ADR-0042): procgen scenery is static world geometry, so
    // collision follows the actual generated mesh. Each ProcCollider becomes the
    // same Collider/MeshCollider + RigidBody the hand-authored loaders build —
    // exact triangle meshes for shells/terrain/roads, primitives where the shape
    // truly is one. (The offline path tracer has no physics and ignores these.)
    auto appendTris = [](MeshCollider& mc, const RenderMesh& rm, const Mat4* xf) {
        for (std::size_t i = 0; i + 2 < rm.indices.size(); i += 3) {
            Vec3 a = rm.vertices[rm.indices[i]].position;
            Vec3 b = rm.vertices[rm.indices[i + 1]].position;
            Vec3 c = rm.vertices[rm.indices[i + 2]].position;
            if (xf) { a = xf->transformPoint(a); b = xf->transformPoint(b); c = xf->transformPoint(c); }
            if (cross(b - a, c - a).length() < 1e-5) continue;   // skip slivers
            uint32_t base = static_cast<uint32_t>(mc.vertices.size());
            mc.vertices.push_back(a); mc.vertices.push_back(b); mc.vertices.push_back(c);
            mc.indices.push_back(base); mc.indices.push_back(base + 1); mc.indices.push_back(base + 2);
        }
    };
    auto spawnMeshCollider = [&](MeshCollider&& mc) {
        if (mc.indices.empty()) return;
        Entity e = world.create();
        Transform t;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        world.add<MeshCollider>(e, std::move(mc));
    };
    auto spawnPrimitive = [&](ColliderShape shape, const Vec3& center, Real yaw,
                              const ProcCollider& src) {
        Entity e = world.create();
        Transform t;
        t.position = center;
        if (std::abs(yaw) > 1e-9) t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Collider c;
        c.shape = shape;
        c.halfExtent = src.halfExtent;
        c.radius = src.radius;
        c.halfHeight = src.halfHeight;
        c.friction = src.friction;
        world.add<Collider>(e, c);
        RigidBody rb;
        rb.motion = src.dynamic ? BodyMotion::Dynamic : BodyMotion::Static;
        world.add<RigidBody>(e, rb);
    };
    // Static triangle-mesh colliders are MERGED by friction into one body each:
    // a city is hundreds of solids (every building, plinth, lawn, the terrain, the
    // roads), and one static mesh body per surface keeps the physics broadphase
    // cheap (a few bodies, each with its own BVH) instead of hundreds of bodies.
    // Primitives (a round tower, a lamp post) stay per-entity — there are few.
    std::vector<std::pair<Real, MeshCollider>> meshBuckets;
    auto bucketFor = [&](Real friction) -> MeshCollider& {
        for (auto& b : meshBuckets)
            if (std::abs(b.first - friction) < 1e-4) return b.second;
        meshBuckets.emplace_back(friction, MeshCollider{});
        meshBuckets.back().second.friction = friction;
        return meshBuckets.back().second;
    };
    for (const ProcCollider& pc : model.colliders) {
        switch (pc.kind) {
            case ProcCollider::Kind::Mesh:
                appendTris(bucketFor(pc.friction), pc.mesh, nullptr); break;
            case ProcCollider::Kind::Box:
                spawnPrimitive(ColliderShape::Box, pc.center, pc.yaw, pc); break;
            case ProcCollider::Kind::Sphere:
                spawnPrimitive(ColliderShape::Sphere, pc.center, 0, pc); break;
            case ProcCollider::Kind::Capsule:
                spawnPrimitive(ColliderShape::Capsule, pc.center, 0, pc); break;
        }
    }

    // Instanced colliders: stamp a collider at each placement (a lamp per verge).
    // Mesh stamps merge into the friction buckets; primitives are one body each.
    for (const ProcInstanceGroup& cg : model.instances) {
        if (cg.collision == InstanceCollision::None || cg.transforms.empty()) continue;
        if (cg.collision == InstanceCollision::Mesh) {
            const RenderMesh& proto =
                cg.collisionProto.vertices.empty() ? cg.proto : cg.collisionProto;
            MeshCollider& mc = bucketFor(cg.colliderFriction);
            for (const Mat4& xf : cg.transforms) appendTris(mc, proto, &xf);
            continue;
        }
        ColliderShape shape = cg.collision == InstanceCollision::Box ? ColliderShape::Box
                            : cg.collision == InstanceCollision::Sphere ? ColliderShape::Sphere
                                                                        : ColliderShape::Capsule;
        ProcCollider tmpl;
        tmpl.halfExtent = cg.colliderHalfExtent;
        tmpl.radius = cg.colliderRadius;
        tmpl.halfHeight = cg.colliderHalfHeight;
        tmpl.friction = cg.colliderFriction;
        // The proto stands on its origin, so lift the collider centre to wrap a
        // prop sitting on the ground rather than sinking it half-under.
        Real lift = shape == ColliderShape::Box ? cg.colliderHalfExtent.y
                  : shape == ColliderShape::Sphere ? cg.colliderRadius
                                                   : cg.colliderHalfHeight + cg.colliderRadius;
        for (const Mat4& xf : cg.transforms) {
            Vec3 pos(xf.m[0][3], xf.m[1][3], xf.m[2][3]);
            spawnPrimitive(shape, pos + Vec3(0, lift, 0), 0, tmpl);
        }
    }
    for (auto& b : meshBuckets) spawnMeshCollider(std::move(b.second));
#else
    (void)ent; (void)levelDir; (void)world; (void)renderer; (void)assets; (void)index;
    LOG_WARN << "script entity skipped (scripting disabled in this build)";
#endif
}

static void loadEntities(const json& entities, const json& root, World& world,
                         Renderer& renderer, AssetManager& assets,
                         const std::string& levelDir, bool editorMode,
                         const json* cityEnt = nullptr,
                         const CityModel* cityModel = nullptr,
                         const std::vector<std::pair<const json*, ProcModel>>*
                             scriptCache = nullptr,
                         const HeightField* ground = nullptr) {
    MaterialTable materials = buildMaterialTable(root);   // named "materials" table
    SurfaceTexCache surfaceTex;   // one bake+upload per surface across the load
    int treeIndex = 0;
    int cityIndex = 0;
    int scriptIndex = 0;
    int roadIndex = 0;
    for (auto& ent : entities) {
        // Hero parametric tree: a collidable, textured object (not scatter).
        if (ent.value("shape", std::string()) == "tree") {
            loadTreeEntity(ent, world, renderer, assets, treeIndex++);
            continue;
        }
        // Editor-authored road (ADR-0049): an editable RoadNet + its baked mesh.
        if (ent.value("shape", std::string()) == "road") {
            loadRoadEntity(ent, world, assets, roadIndex++, ground);
            continue;
        }
        // Lua recipe (ADR-0042): run the script and spawn its composable model —
        // the same shape:"script" the offline tracer renders, now in the viewer.
        // An on-terrain recipe was pre-run (for terrain grading) and is spawned
        // from the cache; others run now, with the level ground injected.
        if (ent.value("shape", std::string()) == "script") {
            const ProcModel* pre = nullptr;
            if (scriptCache != nullptr)
                for (const auto& p : *scriptCache)
                    if (p.first == &ent) { pre = &p.second; break; }
            loadScriptEntity(ent, levelDir, world, renderer, assets, scriptIndex++,
                             pre, ground);
            continue;
        }
        // Procedural city: renderable geometry + per-building colliders. Reuse the
        // model the loader pre-generated for terrain cut/fill (so it isn't built
        // twice and the geometry matches the graded terrain exactly).
        if (ent.value("shape", std::string()) == "city") {
            const CityModel* pre = (&ent == cityEnt) ? cityModel : nullptr;
            loadCityEntity(ent, root, world, renderer, assets, cityIndex++, pre);
            continue;
        }

        // Group / null object: a named transform with no mesh, for parenting.
        if (ent.value("group", false) ||
            (ent.contains("shape") && ent["shape"].get<std::string>().empty()
             && !ent.contains("mesh"))) {
            Entity e = world.create();
            createEntityCommon(e, ent, world);
            SourceSpec spec = buildSourceSpec(ent, "");
            world.add<SourceSpec>(e, spec);
            continue;
        }

        if (ent.contains("mesh")) {
            std::string meshPath = ent["mesh"].get<std::string>();
            if (!meshPath.empty() && meshPath[0] != '/')
                meshPath = levelDir + "/" + meshPath;

            ImportedModel model = ModelImporter::load(meshPath, renderer);
            if (model.meshes.empty()) continue;

            for (size_t i = 0; i < model.meshes.size(); i++) {
                Entity e = world.create();
                createEntityCommon(e, ent, world);

                Renderable r;
                r.mesh = model.meshes[i].meshHandle;
                r.material = model.meshes[i].material;
                // Present keys override the imported material; absent ones
                // keep it (JsonReadVisitor's missing-key semantics). A string
                // value swaps in a shared material from the named table.
                if (ent.contains("material"))
                    r.material = resolveMaterial(ent["material"], materials, r.material);
                world.add<Renderable>(e, r);

                if (i == 0) {
                    addPhysics(e, ent, "box", world);
                    SourceSpec spec = buildSourceSpec(ent, "");
                    spec.meshFile = ent["mesh"].get<std::string>();
                    world.add<SourceSpec>(e, spec);
                }
            }
            continue;
        }

        std::string shape = ent.value("shape", "box");
        auto sizeJ = ent.contains("size") ? ent["size"] : json::array({1, 1, 1});

        Entity e = world.create();
        createEntityCommon(e, ent, world);
        world.add<SourceSpec>(e, buildSourceSpec(ent, shape));

        Renderable r;
        if (ent.contains("material"))
            r.material = resolveMaterial(ent["material"], materials);
        RenderMaterial::Surface surf = r.material.surface();
        if (surf != RenderMaterial::Surface::None) {
            // A material with a baked surface: give this entity its own mesh with
            // planar tiling UVs (the texture tiles at human scale) and bind the
            // baked PBR set, rather than the shared, untextured primitive.
            RenderMesh mesh = MeshBuilder::shape(shape, parseVec3(sizeJ, Vec3(1, 1, 1)));
            applyWorldPlanarUVs(mesh, 1.0 / surfaceWorldTileSize(surf));
            r.mesh = assets.acquireMesh(mesh, "");   // unkeyed: per-entity UVs
            bindSurfaceMaps(r.material, bakeSurfaceTextures(renderer, surf, surfaceTex));
        } else {
            r.mesh = getOrCreateMesh(shape, sizeJ, assets);
        }
        world.add<Renderable>(e, r);

        addPhysics(e, ent, shape, world);
    }

    // Levels authored before stable ids (or with gaps) get them now, so the
    // editor's parenting always has something to reference.
    assignMissingDocumentIds(world);

    if (!editorMode) {
        // PLAY flattens the hierarchy: bake each entity's composed world
        // transform into its Transform and drop the parent link, so the
        // runtime (render, physics) never walks a hierarchy and bodies are
        // created in world space. Compute all world matrices first, then
        // assign, so the result is independent of iteration order.
        std::vector<std::pair<Entity, Mat4>> baked;
        world.each<Transform, SourceSpec>([&](Entity e, Transform&, SourceSpec& s) {
            if (s.parentId != 0) baked.emplace_back(e, worldMatrix(world, e));
        });
        for (auto& [e, m] : baked) {
            Transform flat = transformFromMatrix(m);
            *world.get<Transform>(e) = flat;
            if (auto* prev = world.get<PrevTransform>(e)) prev->value = flat;
            world.get<SourceSpec>(e)->parentId = 0;
        }
    }
}

static void loadPlayer(const json& player, World& world) {
    Entity e = world.create();

    Transform t;
    if (player.contains("position"))
        t.position = parseVec3(player["position"]);

    // The player walks on a CharacterVirtual (collide-and-slide capsule that
    // steps up curbs/stairs), not a dynamic rigid body — a dynamic capsule is
    // stopped dead by a kerb. The collider block keeps the same capsule
    // dimensions it always had.
    CharacterController cc;
    if (player.contains("collider")) {
        auto& col = player["collider"];
        cc.radius = col.value("radius", 0.3);
        cc.halfHeight = col.value("halfHeight", 0.4);
    }
    cc.stepHeight = player.value("stepHeight", 0.45);

    // CDLOD terrain: snap the spawn to just above the surface at its XZ so the
    // character settles onto the ground rather than spawning embedded (below the
    // surface) or falling from far above. Scoped to CDLOD (static-chunk levels
    // have no TerrainLodConfig and keep the authored y).
    const TerrainLodConfig* tc = nullptr;
    world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& cfg) { if (!tc) tc = &cfg; });
    if (tc) {
        Noise noise(tc->seed);
        double surface = terrainHeight(tc->params, noise, t.position.x, t.position.z);
        t.position.y = surface + cc.radius + cc.halfHeight + 1.0;   // drop ~1 m on
    }

    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    world.add<CharacterController>(e, cc);
    world.add<ControlledBy>(e, ControlledBy{0});
}

static void loadLighting(const json& lighting, RenderView& view) {
    auto& l = view.lighting;

    if (lighting.contains("sun")) {
        auto& sun = lighting["sun"];
        l.sun.direction  = parseVec3(sun["direction"], l.sun.direction);
        l.sun.color      = parseVec3(sun["color"], l.sun.color);
        l.sun.intensity  = sun.value("intensity", l.sun.intensity);
        l.sun.castsShadow = sun.value("castsShadow", true);
    }

    if (lighting.contains("pointLights")) {
        l.pointLights.clear();
        for (auto& pl : lighting["pointLights"]) {
            PointLight p;
            p.position  = parseVec3(pl["position"]);
            p.color     = parseVec3(pl["color"], Vec3(1, 1, 1));
            p.intensity = pl.value("intensity", 1.0f);
            p.range     = pl.value("range", p.range);
            l.pointLights.push_back(p);
        }
    }

    if (lighting.contains("spotLights")) {
        l.spotLights.clear();
        for (auto& sl : lighting["spotLights"]) {
            SpotLight s;
            s.position       = parseVec3(sl["position"]);
            s.direction      = parseVec3(sl["direction"]);
            s.color          = parseVec3(sl["color"], Vec3(1, 1, 1));
            s.intensity      = sl.value("intensity", 1.0f);
            s.range          = sl.value("range", s.range);
            s.innerConeAngle = sl.value("innerConeAngle", 0.3f);
            s.outerConeAngle = sl.value("outerConeAngle", 0.5f);
            s.castsShadow    = sl.value("castsShadow", false);
            l.spotLights.push_back(s);
        }
    }

    if (lighting.contains("shadow")) {
        auto& sh = lighting["shadow"];
        l.shadow.enabled    = sh.value("enabled", l.shadow.enabled);
        l.shadow.bias       = sh.value("bias", l.shadow.bias);
        l.shadow.normalBias = sh.value("normalBias", l.shadow.normalBias);
        l.shadow.pcfRadius  = sh.value("pcfRadius", l.shadow.pcfRadius);
        // Cascade-fit overrides (0 = unset): a big world needs a longer shadow range.
        l.shadow.distance     = sh.value("distance", l.shadow.distance);
        l.shadow.cascadeCount = sh.value("cascades", l.shadow.cascadeCount);
        // Artistic response (ADR-0017 Phase 2)
        l.shadowArtistic.strength        = sh.value("strength", l.shadowArtistic.strength);
        l.shadowArtistic.ambientStrength = sh.value("ambientStrength", l.shadowArtistic.ambientStrength);
        if (sh.contains("tint"))
            l.shadowArtistic.tint = parseVec3(sh["tint"], l.shadowArtistic.tint);
    }

    l.exposure          = lighting.value("exposure", l.exposure);
    l.ambientMultiplier = lighting.value("ambientMultiplier", l.ambientMultiplier);
    if (lighting.contains("ambientTint"))
        l.ambientTint = parseVec3(lighting["ambientTint"], l.ambientTint);
}

static void loadPlayerSpawn(const json& player, World& world,
                            AssetManager& assets) {
    Entity e = world.create();
    Transform t;
    if (player.contains("position"))
        t.position = parseVec3(player["position"]);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    world.add<PlayerSpawn>(e);

    Renderable gizmo;
    gizmo.mesh = assets.acquireMesh(MeshBuilder::capsule(0.3f, 0.8f),
                                    "playerspawn:capsule");
    gizmo.material.albedo = Vec3(0.2, 0.8, 0.3);   // green = "you start here"
    gizmo.material.roughness = 0.5f;
    world.add<Renderable>(e, gizmo);
}

static TerrainParams parseTerrainParams(const json& t) {
    TerrainParams p;
    p.size        = t.value("size", p.size);
    p.resolution  = t.value("resolution", p.resolution);
    p.heightScale = t.value("heightScale", p.heightScale);
    p.noiseScale  = t.value("noiseScale", p.noiseScale);
    p.octaves     = t.value("octaves", p.octaves);
    p.warp        = t.value("warp", p.warp);
    p.mountainHeight = t.value("mountainHeight", p.mountainHeight);
    p.mountainScale  = t.value("mountainScale", p.mountainScale);
    p.mountainMaskScale = t.value("mountainMaskScale", p.mountainMaskScale);
    p.mountainMaskLo = t.value("mountainMaskLo", p.mountainMaskLo);
    p.mountainMaskHi = t.value("mountainMaskHi", p.mountainMaskHi);
    p.mountainAlongRange = t.value("mountainAlongRange", p.mountainAlongRange);
    p.tiltX = t.value("tiltX", p.tiltX);
    p.tiltZ = t.value("tiltZ", p.tiltZ);
    p.seaLevel = t.value("seaLevel", p.seaLevel);   // set from the water block below
    p.snowLine = t.value("snowLine", p.snowLine);   // colour-band scaling
    p.rockLine = t.value("rockLine", p.rockLine);
    if (t.contains("rangeSpine") && t["rangeSpine"].is_array()) {
        std::vector<Vec3> ctl;
        for (const auto& pt : t["rangeSpine"])
            if (pt.is_array() && pt.size() >= 2)
                ctl.push_back(Vec3(pt[0].get<double>(), 0.0, pt[1].get<double>()));
        p.rangeSpine = sampleRangeSpine(ctl);
    }
    p.rangeWidth = t.value("rangeWidth", p.rangeWidth);
    p.rangeHeight = t.value("rangeHeight", p.rangeHeight);
    p.rangeVariation = t.value("rangeVariation", p.rangeVariation);
    if (t.contains("range") && t["range"].is_object()) {
        const auto& r = t["range"];
        p.rangeRidges = buildRangeRidges(
            r.value("length", 60.0f), r.value("branchAngle", 38.0f),
            r.value("falloff", 0.55f), r.value("leaderFalloff", 0.92f),
            r.value("iterations", 5), r.value("height", 130.0f),
            r.value("depthFalloff", 0.62f), r.value("angleJitter", 12.0f),
            r.value("seed", 0u));
        p.rangeWidth = r.value("width", p.rangeWidth);
    }
    return p;
}

// Procedural terrain (ADR-0021 persistence: the document stores the recipe —
// seed + params — and the engine regenerates the mesh at load, rather than
// serializing the geometry). The terrain entity carries no SourceSpec, so the
// LevelWriter never writes it back as a document entity (it stays a regenerated
// runtime object); its GPU mesh is owned by the AssetManager and freed on the
// next clear().
// Chunked terrain (ADR-0034 Phase 1): a grid of independently-meshed chunks, each
// with its own tight AABB so frustum culling rejects off-screen chunks, replacing
// the single origin-centred tile + concentric LOD rings. Near chunks (within the
// collider radius) carry a static collider so the player walks on them. Opt-in via
// the level's "chunks" key; without it, loadTerrain keeps the legacy single tile.
static void loadChunkedTerrain(const TerrainParams& p, const Noise& noise,
                               const json& t, World& world, AssetManager& assets) {
    int chunksPerSide = t.value("chunks", 1);
    float chunkSize = t.value("chunkSize", p.size);
    int res = t.value("chunkResolution", p.resolution);
    // Default collider coverage: the central chunk and its immediate neighbours.
    float colliderRadius = t.value("colliderRadius", chunkSize * 1.5f);

    RenderMaterial material;
    bool hasMat = t.contains("material");
    if (hasMat) applyMaterial(t["material"], material);
    else {
        material.albedo = Vec3(0.42, 0.5, 0.32);
        material.roughness = 0.95f;
    }

    auto chunks = generateTerrainChunks(p, noise, chunksPerSide, chunkSize, res,
                                        colliderRadius);
    for (TerrainChunk& chunk : chunks) {
        Entity e = world.create();
        world.add<Transform>(e, Transform{});             // mesh is world-space
        world.add<PrevTransform>(e, PrevTransform{Transform{}});
        if (chunk.collider) {
            MeshCollider mc;
            mc.vertices.reserve(chunk.mesh.vertices.size());
            for (const Vertex& v : chunk.mesh.vertices) mc.vertices.push_back(v.position);
            mc.indices = chunk.mesh.indices;
            world.add<MeshCollider>(e, mc);
        }
        Renderable r;
        r.mesh = assets.acquireMesh(chunk.mesh, "terrain_chunk_" +
                                    std::to_string(chunk.cx) + "_" +
                                    std::to_string(chunk.cz));
        r.material = material;
        world.add<Renderable>(e, r);
    }
}

// CDLOD heightfield terrain (ADR-0036, open-world Phase 1c): stamp a single
// TerrainLodConfig the TerrainLodSystem drives each frame (selection + morph +
// streaming-ready cache), instead of static chunk entities. Opt-in via the terrain
// block's "cdlod" key (an object of overrides, or `true` for defaults). The
// TerrainLodSystem also maintains a moving window of near-node colliders (ADR-0036)
// so the player walks on the surface.
static void loadCdlodTerrain(const TerrainParams& p, const json& t, World& world) {
    TerrainLodConfig cfg;
    cfg.params = p;
    cfg.seed = t.value("seed", 0u);
    const json& c = t["cdlod"];
    if (c.is_object()) {
        cfg.worldHalf = c.value("worldHalf", cfg.worldHalf);
        cfg.numLods = c.value("numLods", cfg.numLods);
        cfg.gridRes = c.value("gridRes", cfg.gridRes);
        cfg.rangeFactor = c.value("rangeFactor", cfg.rangeFactor);
        cfg.colliderRadius = c.value("colliderRadius", cfg.colliderRadius);
    }
    if (t.contains("material")) applyMaterial(t["material"], cfg.material);
    else {
        cfg.material.albedo = Vec3(0.42, 0.5, 0.32);
        cfg.material.roughness = 0.95f;
    }
    // Give the ground its natural-surface material (micro-relief + roughness) unless
    // the level authored a specific surface. The biome colour stays in the vertex
    // colour; this only adds the normal/roughness detail (Surface::TerrainGround).
    if (cfg.material.surface() == RenderMaterial::Surface::None)
        cfg.material.setSurface(RenderMaterial::Surface::TerrainGround);
    Entity e = world.create();
    world.add<TerrainLodConfig>(e, cfg);
}

static void loadTerrain(const TerrainParams& p, const Noise& noise, const json& t,
                        World& world, AssetManager& assets) {
    bool wantCdlod = t.contains("cdlod") &&
                     (t["cdlod"].is_object() ||
                      (t["cdlod"].is_boolean() && t["cdlod"].get<bool>()));
    if (wantCdlod) {
        loadCdlodTerrain(p, t, world);
        return;
    }
    if (t.contains("chunks") && t["chunks"].get<int>() > 0) {
        loadChunkedTerrain(p, noise, t, world, assets);
        return;
    }
    Entity e = world.create();
    Transform tr;   // generated directly in world space
    world.add<Transform>(e, tr);
    world.add<PrevTransform>(e, PrevTransform{tr});

    RenderMesh terrainMesh;
    if (t.value("erode", false) && !p.erodedBase) {
        // Legacy static-mesh erode path (no pre-baked field): bake -> erode ->
        // mesh, the eroded grid being the source of truth for mesh and collider.
        // When loadLevel has already baked p.erodedBase (the shared path), fall
        // through to generateTerrain — it samples that eroded field via
        // terrainHeight, so re-eroding here would double-erode.
        Heightmap hm = bakeHeightmap(p, noise);
        ErosionParams ep;
        ep.seed = t.value("seed", 0u) + 1234u;
        ep.droplets = t.value("erodeDroplets", ep.droplets);
        ep.erodeRadius = t.value("erodeRadius", ep.erodeRadius);
        ep.thermalIterations = t.value("erodeThermal", ep.thermalIterations);
        ep.talus = t.value("erodeTalus", ep.talus);
        erode(hm, ep);
        terrainMesh = generateTerrainMesh(hm);
    } else {
        terrainMesh = generateTerrain(p, noise);
    }

    // Static collision from the same geometry, so the player walks on the
    // terrain instead of falling through (PhysicsSystem makes one static mesh
    // body). Inert when physics is disabled.
    MeshCollider mc;
    mc.vertices.reserve(terrainMesh.vertices.size());
    for (const Vertex& v : terrainMesh.vertices) mc.vertices.push_back(v.position);
    mc.indices = terrainMesh.indices;
    world.add<MeshCollider>(e, mc);

    Renderable r;
    r.mesh = assets.acquireMesh(terrainMesh, "terrain");
    if (t.contains("material")) {
        applyMaterial(t["material"], r.material);
    } else {
        r.material.albedo = Vec3(0.42, 0.5, 0.32);   // muted green-brown default
        r.material.roughness = 0.95f;
    }
    if (r.material.surface() == RenderMaterial::Surface::None)
        r.material.setSurface(RenderMaterial::Surface::TerrainGround);   // ground micro-relief
    world.add<Renderable>(e, r);

    // Distant LOD rings extend the terrain to the horizon (mountains/hills) at a
    // fraction of the triangle cost. Render-only (no collider — you never reach
    // them); regenerated runtime objects, no SourceSpec.
    int lodRings = t.value("lodRings", 0);
    if (lodRings > 0) {
        int lodCells = t.value("lodCells", 40);
        std::vector<RenderMesh> rings = generateTerrainLOD(p, noise, lodRings, lodCells);
        for (std::size_t i = 0; i < rings.size(); i++) {
            Entity re = world.create();
            world.add<Transform>(re, Transform{});
            world.add<PrevTransform>(re, PrevTransform{Transform{}});
            Renderable rr;
            rr.mesh = assets.acquireMesh(rings[i], "terrain_lod" + std::to_string(i));
            rr.material = r.material;
            world.add<Renderable>(re, rr);
        }
    }
}

// Vegetation: generate a few "species" meshes (L-system trees, noise rocks)
// once, then scatter them across the terrain as individual entities sharing
// each species' GPU mesh (AssetManager dedup). Per-entity rendering for now —
// instancing (the thousands-scale path) comes later. Carries no SourceSpec, so
// these are regenerated runtime objects, not document entities.
static void loadVegetation(const json& veg, const TerrainParams& terrain,
                           const Noise& terrainNoise, World& world,
                           Renderer& renderer, AssetManager& assets,
                           const std::string& levelDir,
                           const std::string& tag = "veg") {
    if (!veg.contains("species") || !veg["species"].is_array()) return;

    // A species variant is now a multi-part model (ADR-0032): each part is one
    // {mesh, material} drawn as its own InstanceGroup over the shared transforms,
    // so bark (opaque) and leaves (alpha-cut) — or any N parts — scatter together.
    struct Part { MeshHandle mesh; RenderMaterial material; };
    struct Variant {
        std::vector<Part> parts;
        float xzRadius = 0.0f;       // canopy footprint, for scatter spacing
        float trunkHeight = 0.0f;    // measured mesh height (capsule collider)
        float trunkRadius = 0.0f;    // measured base footprint (capsule collider)
        bool collide = false;
        float colliderRadius = 0.0f; // 0 = auto from trunkRadius
        float colliderHeight = 0.0f; // 0 = auto from trunkHeight
        double colliderFriction = 0.8;
    };
    std::vector<Variant> variantList;
    uint32_t vegSeed = veg.value("seed", 0u);

#ifdef RT_ENABLE_SCRIPTING
    // Lua flora species (ADR-0023): created on first use, with the procgen
    // builders and the shared flora library loaded, so a species can be
    // `{ "kind":"script", "script":"return flora.tree(seed, {species='oak'})" }`
    // (inline) or `"script":"trees/oak.lua"` (a level-relative chunk) that
    // returns a mesh, read per variant from a `seed` global. Mixed freely with
    // the C++ tree/rock species in the same scatter.
    std::unique_ptr<ScriptVM> scriptVm;
    auto ensureScriptVm = [&]() -> ScriptVM& {
        if (!scriptVm) {
            scriptVm = std::make_unique<ScriptVM>();
            openProcgenLibrary(*scriptVm);
            std::ifstream ff("assets/scripts/flora.lua");
            if (ff) {
                std::string src((std::istreambuf_iterator<char>(ff)),
                                std::istreambuf_iterator<char>());
                std::string err;
                if (!scriptVm->doString(src, &err))
                    LOG_ERROR << "flora.lua load failed: " << err;
            } else {
                LOG_WARN << "assets/scripts/flora.lua not found; `flora.*` unavailable";
            }
        }
        return *scriptVm;
    };
#endif

    int speciesIndex = 0;
    for (const auto& s : veg["species"]) {
        std::string kind = s.value("kind", "tree");
        // Each species can emit several distinct meshes ("variants"); scatter
        // mixes them so the forest isn't one cloned model. For stochastic trees
        // the per-variant seed grows a different tree; for rocks it reshapes the
        // lumps/cuts. Each variant is one shared GPU mesh (instancing-friendly).
        int variants = std::max(1, s.value("variants", 1));

        // Tree grammar + turtle params (parsed once; the seed varies expansion).
        LSystem sys;
        if (s.contains("rules"))
            for (auto it = s["rules"].begin(); it != s["rules"].end(); ++it) {
                if (it.key().empty()) continue;
                char sym = it.key()[0];
                const auto& val = it.value();
                if (val.is_array())   // weighted: [{ "to": "...", "weight": w }, ...]
                    for (const auto& prod : val)
                        sys.rule(sym, prod.value("to", std::string()),
                                 prod.value("weight", 1.0));
                else
                    sys.rule(sym, val.get<std::string>());
            }
        TurtleParams tp;
        tp.length        = s.value("length", tp.length);
        tp.radius        = s.value("radius", tp.radius);
        tp.radiusTaper   = s.value("radiusTaper", tp.radiusTaper);
        tp.taper         = s.value("taper", tp.taper);
        tp.angleDeg      = s.value("angleDeg", tp.angleDeg);
        tp.segmentSlices = s.value("segmentSlices", tp.segmentSlices);
        tp.leafRadius    = s.value("leafRadius", tp.leafRadius);
        std::string axiom = s.value("axiom", std::string("F"));
        int iterations = s.value("iterations", 3);
        bool treeSdf = s.value("skin", std::string("cylinder")) == "sdf";
        double treeSmooth = s.value("smoothness", 0.12);
        int treeRes = s.value("sdfResolution", 40);
        // Canopy coloration: trunk color at the base fading to leaf color at the
        // top, baked into vertex colors (use a white material so it shows).
        Vec3 trunkColor = parseVec3(s.value("trunkColor", json::array({0.30, 0.22, 0.12})),
                                    Vec3(0.30, 0.22, 0.12));
        Vec3 leafColor = parseVec3(s.value("leafColor", json::array({0.18, 0.40, 0.15})),
                                   Vec3(0.18, 0.40, 0.15));

        // Rock params (parsed once).
        bool rockSdf = s.value("skin", std::string("displaced")) == "sdf";
        RockSdfParams rsp;
        rsp.baseRadius = s.value("radius", rsp.baseRadius);
        rsp.lumps      = s.value("lumps", rsp.lumps);
        rsp.cuts       = s.value("cuts", rsp.cuts);
        rsp.lumpScale  = s.value("lumpScale", rsp.lumpScale);
        rsp.smoothness = s.value("smoothness", rsp.smoothness);
        rsp.resolution = s.value("sdfResolution", rsp.resolution);
        RockParams rp;
        rp.radius       = s.value("radius", rp.radius);
        rp.displacement = s.value("displacement", rp.displacement);
        rp.noiseScale   = s.value("noiseScale", rp.noiseScale);
        rp.octaves      = s.value("octaves", rp.octaves);

        RenderMaterial material;
        if (s.contains("material")) applyMaterial(s["material"], material);

        // Optional per-trunk capsule collider for this species (forest trees you
        // bounce off). Radius/height auto-measured from the mesh unless given.
        bool spCollide = s.value("collide", false);
        float spColRadius = s.value("colliderRadius", 0.0f);
        float spColHeight = s.value("colliderHeight", 0.0f);
        double spColFriction = s.value("colliderFriction", 0.8);
        bool spWind = s.value("wind", false);   // FLAG_WIND sway for this species
        if (spWind) material.flags |= RenderMaterial::FLAG_WIND;

        // Optional: this species' mesh comes from a Lua flora script (inline
        // chunk, or a level-relative .lua path), evaluated per variant.
        std::string scriptSpec = s.value("script", std::string());
        bool hasScript = !scriptSpec.empty();
        std::string scriptSource;
        if (hasScript) {
            const bool isPath = scriptSpec.size() > 4 &&
                                scriptSpec.compare(scriptSpec.size() - 4, 4, ".lua") == 0;
            if (isPath) {
                std::string p = scriptSpec;
                if (!p.empty() && p[0] != '/') p = levelDir + "/" + p;
                std::ifstream sf(p);
                if (sf) scriptSource.assign((std::istreambuf_iterator<char>(sf)),
                                            std::istreambuf_iterator<char>());
                if (scriptSource.empty()) LOG_ERROR << "Failed to load flora script: " << p;
            } else {
                scriptSource = scriptSpec;   // inline Lua chunk
            }
            hasScript = !scriptSource.empty();
#ifndef RT_ENABLE_SCRIPTING
            if (hasScript)
                LOG_WARN << "species 'script' ignored (scripting disabled)";
            hasScript = false;
#endif
        }

        for (int v = 0; v < variants; v++) {
            uint32_t seed = vegSeed + 1000u * static_cast<uint32_t>(speciesIndex) + 1u + v;
            Variant var;
            var.collide = spCollide;
            var.colliderRadius = spColRadius;
            var.colliderHeight = spColHeight;
            var.colliderFriction = spColFriction;
            // Add one part; measure the canopy footprint (XZ radius, for scatter
            // spacing) and the trunk capsule (mesh height + base footprint).
            auto addPart = [&](const RenderMesh& m, const RenderMaterial& mat) {
                if (m.vertices.empty()) return;
                float r = 0.0f, maxY = 0.0f;
                for (const Vertex& vert : m.vertices) {
                    r = std::max(r, std::sqrt(static_cast<float>(
                            vert.position.x * vert.position.x +
                            vert.position.z * vert.position.z)));
                    maxY = std::max(maxY, static_cast<float>(vert.position.y));
                }
                var.xzRadius = std::max(var.xzRadius, r);
                var.trunkHeight = std::max(var.trunkHeight, maxY);
                // Base footprint: widest XZ in the lowest fifth (the trunk).
                float yThresh = 0.2f * maxY, baseR = 0.0f;
                for (const Vertex& vert : m.vertices)
                    if (vert.position.y <= yThresh)
                        baseR = std::max(baseR, std::sqrt(static_cast<float>(
                                vert.position.x * vert.position.x +
                                vert.position.z * vert.position.z)));
                var.trunkRadius = std::max(var.trunkRadius, baseR);
                Part p;
                p.mesh = assets.acquireMesh(
                    m, tag + ":" + std::to_string(speciesIndex) + ":" +
                           std::to_string(v) + ":" + std::to_string(var.parts.size()));
                p.material = mat;
                var.parts.push_back(p);
            };

            if (hasScript) {
#ifdef RT_ENABLE_SCRIPTING
                ScriptVM& vm = ensureScriptVm();
                vm.setGlobalNumber("seed", seed);   // the script reads `seed`
                std::vector<ScriptMeshPart> parts;
                std::string err;
                if (runProcgenModel(vm, scriptSource, parts, &err)) {
                    for (const ScriptMeshPart& sp : parts) {
                        if (!sp.mesh) continue;
                        RenderMaterial mat = material;   // species JSON default
                        if (sp.hasMaterial) {
                            mat = RenderMaterial();
                            mat.albedo = sp.albedo;
                            mat.roughness = sp.roughness;
                            mat.metallic = sp.metallic;
                            if (sp.alphaTest || sp.texture == "leaf")
                                mat.flags |= RenderMaterial::FLAG_ALPHA_TEST;
                            if (sp.texture.rfind("bark", 0) == 0) {
                                BarkMaps bm = barkMaps(barkStyleFromName(sp.texture), 256, seed);
                                if (!bm.albedo.pixels.empty())
                                    mat.albedoMap = renderer.uploadTexture(
                                        bm.albedo.width, bm.albedo.height,
                                        bm.albedo.channels, bm.albedo.pixels.data());
                                if (!bm.normal.pixels.empty())
                                    mat.normalMap = renderer.uploadTexture(
                                        bm.normal.width, bm.normal.height,
                                        bm.normal.channels, bm.normal.pixels.data());
                            } else if (sp.texture == "leaf") {
                                TextureData td = leafTexture(128);
                                if (!td.pixels.empty())
                                    mat.albedoMap = renderer.uploadTexture(
                                        td.width, td.height, td.channels, td.pixels.data());
                            }
                        }
                        if (spWind || sp.wind) mat.flags |= RenderMaterial::FLAG_WIND;
                        addPart(*sp.mesh, mat);
                    }
                } else {
                    LOG_ERROR << "flora script error: " << err;
                }
#endif
            } else if (kind == "rock") {
                addPart(rockSdf ? generateRockSdf(rsp, seed)
                                : generateRock(rp, Noise(seed)),
                        material);
            } else {
                RenderMesh mesh =
                    treeSdf ? generateTreeSdf(sys, axiom, iterations, tp,
                                              treeSmooth, treeRes, seed)
                            : generateTree(sys, axiom, iterations, tp, seed);
                MeshBuilder::bakeHeightColor(mesh, trunkColor, leafColor);
                addPart(mesh, material);
            }
            if (!var.parts.empty()) variantList.push_back(std::move(var));
        }
        speciesIndex++;
    }
    if (variantList.empty()) return;

    ScatterParams scatter;
    scatter.regionSize       = veg.value("region", 70.0f);
    scatter.count            = veg.value("count", 80);
    scatter.maxSlopeDeg      = veg.value("maxSlopeDeg", 40.0f);
    scatter.minHeight        = veg.value("minHeight", scatter.minHeight);   // altitude band
    scatter.maxHeight        = veg.value("maxHeight", scatter.maxHeight);   // (treeline etc.)
    scatter.minScale         = veg.value("minScale", 0.7f);
    scatter.maxScale         = veg.value("maxScale", 1.3f);
    scatter.densityScale     = veg.value("densityScale", 0.05);
    scatter.densityThreshold = veg.value("densityThreshold", -0.2f);
    scatter.focus            = parseVec3(veg.value("focus", json()), Vec3(0, 0, 0));
    scatter.focusRadius      = veg.value("focusRadius", 0.0f);
    scatter.focusScale       = veg.value("focusScale", 1.0f);
    scatter.focusClear       = veg.value("focusClear", 0.0f);
    scatter.clusterCount     = veg.value("clusterCount", 0);
    scatter.clusterRadius    = veg.value("clusterRadius", 6.0f);
    scatter.seed             = vegSeed;
    float vegDrawDistance    = veg.value("drawDistance", 0.0f);  // 0 = unlimited

    // Footprint spacing so big meshes don't jumble: default to the largest
    // canopy radius * max scale * a factor (<1 lets canopies overlap a little).
    // A level may override with an explicit `spacing` (world units) or tune the
    // `spacingFactor`.
    float maxXz = 0.0f;
    for (const Variant& var : variantList) maxXz = std::max(maxXz, var.xzRadius);
    float spacingFactor = veg.value("spacingFactor", 0.9f);
    scatter.minSpacing = veg.contains("spacing")
                             ? veg.value("spacing", 0.0f)
                             : maxXz * scatter.maxScale * spacingFactor;

    // Keep vegetation off everything the city graded (road decks, building
    // pads, lots): the flatten footprints ARE that set, so trees fill the map
    // right up to the streets without a hand-authored clear circle. The margin
    // keeps canopies from overhanging the kerb.
    FlattenGrid keepOut;
    if (!terrain.flatten.empty()) {
        keepOut = buildFlattenGrid(terrain.flatten);
        const double margin = veg.value("clearMargin", 3.0);
        const TerrainParams& tp = terrain;
        scatter.exclude = [&tp, &keepOut, margin](double x, double z) {
            return flattenCovers(keepOut, tp.flatten, x, z, margin);
        };
    }

    std::vector<Placement> placements = scatterOnTerrain(scatter, terrain, terrainNoise);
    LOG_INFO << "[veg] " << tag << ": " << placements.size() << " placements"
             << " (count=" << scatter.count << ", region=" << scatter.regionSize
             << ", maxSlopeDeg=" << scatter.maxSlopeDeg << ")";

    // One InstanceGroup per (variant, part): a variant's parts share the same
    // per-instance transforms (ROADMAP Phase B instancing). Plants carry no
    // SourceSpec, so they were never document entities anyway (ADR-0022).
    std::vector<std::vector<Mat4>> buckets =
        bucketPlacementsBySpecies(placements, variantList.size(), vegSeed + 7u);
    for (std::size_t si = 0; si < variantList.size(); ++si) {
        if (buckets[si].empty()) continue;
        const std::vector<Mat4>& transforms = buckets[si];

        // Coarse group bounds shared by the variant's parts: centroid of instance
        // origins + the spread (the per-part mesh extent is added below).
        Vec3 centroid(0, 0, 0);
        for (const Mat4& m : transforms)
            centroid = centroid + Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
        centroid = centroid / static_cast<Real>(transforms.size());
        Real spread = 0;
        for (const Mat4& m : transforms)
            spread = std::max(spread,
                              (Vec3(m.m[0][3], m.m[1][3], m.m[2][3]) - centroid).length());

        for (const Part& part : variantList[si].parts) {
            InstanceGroup g;
            g.mesh = part.mesh;
            g.material = part.material;
            g.transforms = transforms;
            BoundingSphere mb = assets.meshBounds(g.mesh);
            g.boundsCenter = centroid;
            g.boundsRadius = spread + (mb.center.length() + mb.radius) *
                                          static_cast<Real>(scatter.maxScale);
            g.drawDistance = vegDrawDistance;
            world.add<InstanceGroup>(world.create(), g);
        }

        // Per-trunk static capsule colliders (one body per instance) so you
        // bounce off forest trees. Cheap vs the bark triangle soup; the
        // InstanceGroups above stay render-only. No SourceSpec -> not serialized.
        const Variant& var = variantList[si];
        float colR = var.colliderRadius > 0.0f ? var.colliderRadius : var.trunkRadius;
        float colH = var.colliderHeight > 0.0f ? var.colliderHeight : var.trunkHeight;
        if (var.collide && colR > 1e-3f && colH > 1e-3f) {
            for (const Mat4& m : transforms) {
                Vec3 pos(m.m[0][3], m.m[1][3], m.m[2][3]);
                Real s = Vec3(m.m[0][0], m.m[1][0], m.m[2][0]).length();   // uniform
                float rr = colR * static_cast<float>(s);
                float hh = colH * static_cast<float>(s);

                Entity e = world.create();
                Transform t;
                t.position = Vec3(pos.x, pos.y + hh * 0.5, pos.z);
                world.add<Transform>(e, t);
                world.add<PrevTransform>(e, PrevTransform{t});

                Collider c;
                c.shape = ColliderShape::Capsule;
                c.radius = rr;
                c.halfHeight = std::max(0.0, hh * 0.5 - rr);
                c.friction = var.colliderFriction;
                world.add<Collider>(e, c);

                RigidBody rb;
                rb.motion = BodyMotion::Static;
                world.add<RigidBody>(e, rb);
            }
        }
    }
}

#ifdef RT_ENABLE_SCRIPTING
// Spawn drivable vehicles authored in Lua (ADR-0059). Each "vehicles" entry is
// either `{ "recipe": "sedan", "opts"... }` (-> vehicle.<recipe>(seed, {})) or a
// full `{ "script": "return vehicle.hatchback(seed, {...})" }`, plus a world
// `position` and optional `yaw` (degrees) and `seed`. The vehicles.lua library is
// loaded once into a procgen VM; VehicleSystem (when physics is on) then turns
// each into a Jolt vehicle the player can drive.
static void loadVehicles(const json& vehicles, World& world, AssetManager& assets,
                         const std::string& levelDir) {
    if (!vehicles.is_array() || vehicles.empty()) return;
    std::string lib = loadScriptCode("vehicles.lua", levelDir);
    if (lib.empty()) {
        LOG_WARN << "vehicles: assets/scripts/vehicles.lua not found";
        return;
    }
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::string err;
    if (!vm.doString(lib, &err)) {
        LOG_WARN << "vehicles.lua: " << err;
        return;
    }
    int index = 0;
    for (const auto& v : vehicles) {
        const int i = index++;
        std::string chunk;
        if (v.contains("script") && v["script"].is_string()) {
            chunk = v["script"].get<std::string>();
        } else {
            std::string recipe = v.value("recipe", std::string("sedan"));
            chunk = "return vehicle." + recipe + "(seed, {})";
        }
        uint32_t seed = static_cast<uint32_t>(v.value("seed", i + 1));
        VehicleSpec spec;
        if (!loadVehicleSpec(vm, chunk, seed, spec, &err)) {
            LOG_WARN << "vehicle[" << i << "]: " << err;
            continue;
        }
        Vec3 pos;
        if (v.contains("position") && v["position"].is_array() &&
            v["position"].size() == 3) {
            pos = Vec3(v["position"][0].get<double>(), v["position"][1].get<double>(),
                       v["position"][2].get<double>());
        }
        spawnVehicle(world, assets, spec, pos, v.value("yaw", 0.0));
    }
}
#endif

// Living-city lot growth (ADR-0066), shared by the TERRAIN PRE-PASS and the
// citysim build: the blocks' buildings must be grown BEFORE the terrain is
// meshed — every building stamps a flat graded pad into the ground (device:
// "the terrain should be flat under the building") — and the citysim section
// then spawns the exact same lots rather than growing them twice.
struct GrownLots {
    std::vector<engine::LotBuilding> lots;
    engine::LotPlanDebug plan;           // blocks + lots, for the debug overlay
    std::vector<RenderMesh> parts;       // grown geometry merged by PartId
    bool grown = false;
};

static GrownLots growCityLots(const std::vector<engine::RoadNet>& nets,
                              const json& cs, const std::string& levelDir,
                              const HeightField& ground) {
    GrownLots g;
    // Edge blocks (device feedback): the town RIM has no enclosed faces —
    // synthesize rectangular blocks on boundary roads' open sides so the
    // outskirts build up too. Sized by min/max length + depth knobs.
    engine::EdgeBlockParams ep;
    ep.depth = cs.value("edgeBlockDepth", ep.depth);
    ep.minLen = cs.value("edgeBlockMinLen", ep.minLen);
    ep.maxLen = cs.value("edgeBlockMaxLen", ep.maxLen);
    ep.margin = 4.0 + cs.value("sidewalk", 4.0);
    engine::LotParams lp;
    lp.seed = cs.value("seed", 1u) ^ 0x10c5u;
    lp.buildChance = cs.value("buildChance", 0.9);
    lp.roadMargin = 4.0 + cs.value("sidewalk", 4.0);   // road half + sidewalk
    lp.innerRadius = cs.value("downtownRadius", 55.0);
    lp.midRadius = cs.value("midtownRadius", 135.0);
    lp.plinth = cs.value("plinth", lp.plinth);   // base height above the pad
    // Polycentric zoning: a metro recipe leaves its hubs (with district kinds)
    // on the net — forward them so lots zone by nearest hub, not one centre.
    for (const engine::RoadNet& n : nets)
        for (const engine::CityHub& h : n.cityHubs)
            lp.hubs.push_back({h.pos, h.kind});
    lp.hubRadius = cs.value("hubRadius", 220.0);
    // TERRAIN: buildings grow from their graded pad plane, park/green pads
    // drape per-vertex (city-on-terrain; roads conform separately via
    // net.heightAt + the flatten ramps the loader carves).
    if (ground)
        lp.ground = [&ground](engine::Real x, engine::Real z) {
            return static_cast<engine::Real>(ground(x, z));
        };
    // The STYLE BOOK (the architect's Lua DATA layer): per-recipe look
    // overrides from assets/scripts/style_book.lua. The C++ architect decides
    // what stands where; the book restyles it. The vm must outlive
    // growLotBuildings below (the hook holds it).
#ifdef RT_ENABLE_SCRIPTING
    std::unique_ptr<ScriptVM> styleVm;
    {
        std::string sb = loadScriptCode("style_book.lua", levelDir);
        if (!sb.empty()) {
            styleVm = std::make_unique<ScriptVM>();
            openProcgenLibrary(*styleVm);
            std::string err;
            auto hook = engine::makeStyleBook(*styleVm, sb, &err);
            if (hook) lp.styleHook = std::move(hook);
            else if (!err.empty())
                LOG_WARN << "style_book.lua: " << err;
        }
    }
#else
    (void)levelDir;
#endif
    // Buildings keep clear of the SAMPLED road corridors by sidewalk + a
    // margin, so nothing overhangs the concrete or pokes into the street.
    const double roadClear = cs.value("sidewalk", 4.0) + 0.6;
    engine::NetLotResult r = engine::growLotBuildingsOnNets(nets, lp, ep, roadClear);
    g.lots = std::move(r.lots);
    g.plan = std::move(r.plan);
    g.parts = std::move(r.parts);
    g.grown = true;
    return g;
}

bool LevelLoader::load(const std::string& path,
                       World& world, Renderer& renderer, RenderView& view,
                       AssetManager& assets, bool editorMode) {
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR << "Failed to open level file: " << path;
        return false;
    }

    json root;
    try {
        root = json::parse(file);
    } catch (const json::parse_error& e) {
        LOG_ERROR << "JSON parse error in " << path << ": " << e.what();
        return false;
    }

    int version = root.value("version", 0);
    if (version != LEVEL_FORMAT_VERSION) {
        LOG_ERROR << "Unsupported level format version " << version
                  << " (expected " << LEVEL_FORMAT_VERSION << ")";
        return false;
    }

    std::string levelDir;
    auto lastSlash = path.find_last_of('/');
    if (lastSlash != std::string::npos)
        levelDir = path.substr(0, lastSlash);
    else
        levelDir = ".";

    // Sea level is ONE source of truth: the level's water.seaLevel gates city
    // buildability too, so a coast city's roads/blocks avoid the water instead of
    // marching into it. Inject it (and the beach reserve) as the DEFAULT sea_level
    // for every road recipe that doesn't set its own — done once, up front, so the
    // road pre-pass and the real build regenerate the SAME land-gated graph. Author
    // can still override sea_level per recipe.
    propagateWaterSeaLevel(root);

    // Baked EROSION (ADR-0043 "use the whole toolset"): if the terrain opts in with
    // "erode": true, bake the hydraulically + thermally eroded height field ONCE
    // here and share the sampler across every terrainHeight query below — the level
    // ground the roads/lots conform to, the meshed CDLOD terrain, and the carved
    // drape. Sharing is load-bearing: bake it per-sampler and the roads would
    // conform to analytic relief that the eroded mesh no longer matches, so they'd
    // sink or poke. terrainHeight reads params.erodedBase, so injecting the same
    // shared_ptr into every params copy is all it takes.
    std::shared_ptr<const std::function<double(double, double)>> sharedEroded;
    if (root.contains("terrain") && root["terrain"].value("erode", false)) {
        const json& tj = root["terrain"];
        TerrainParams eb = parseTerrainParams(tj);
        Noise en(tj.value("seed", 0u));
        ErosionParams ep;
        ep.seed = tj.value("seed", 0u) + 1234u;
        ep.droplets = tj.value("erodeDroplets", ep.droplets);
        ep.erodeRadius = tj.value("erodeRadius", ep.erodeRadius);
        ep.thermalIterations = tj.value("erodeThermal", ep.thermalIterations);
        ep.talus = tj.value("erodeTalus", ep.talus);
        bakeErodedTerrain(eb, en, eb.size, tj.value("erodeRes", 512), ep);
        sharedEroded = eb.erodedBase;
    }

    // A city draped on the terrain is generated BEFORE the terrain: it grades its
    // roads/blocks off the natural ground, then returns cut/fill footprints the
    // terrain is built around (so the ground meets the carriageways). The same
    // model is reused when the entity is spawned below.
    const json* cityEnt = nullptr;
    CityModel cityModel;
    std::vector<TerrainFlatten> cityFlatten;
    if (root.contains("entities")) {
        for (const auto& ent : root["entities"]) {
            if (cityIsOnTerrain(ent)) {
                cityEnt = &ent;
                cityModel = cityModelFromEntity(ent, root);
                cityFlatten = cityModel.flatten;
                break;
            }
        }
    }

    // Level ground sampler (ADR-0044): the NATURAL terrain height handed to an
    // on-terrain script recipe (the `ground` global) so a Lua city drapes on and
    // conforms the CDLOD terrain — the script sibling of the C++ city's groundAt.
    HeightField levelGround;
    if (root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(parseTerrainParams(root["terrain"]));
        tp->erodedBase = sharedEroded;   // roads/lots conform to the ERODED surface
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        levelGround = [tp, noise](double x, double z) {
            return terrainHeight(*tp, *noise, x, z);
        };
    }

    // Pre-pass: run on-terrain recipes BEFORE the terrain so their cut/fill
    // footprints grade it; cache the model (by pointer) so the entity loop spawns
    // it instead of re-running the recipe.
    std::vector<std::pair<const json*, ProcModel>> scriptCache;
    std::vector<TerrainFlatten> scriptFlatten;
#ifdef RT_ENABLE_SCRIPTING
    if (levelGround && root.contains("entities")) {
        for (const auto& ent : root["entities"]) {
            if (ent.value("shape", std::string()) == "script" &&
                ent.value("onTerrain", false)) {
                ProcModel m;
                if (runScriptModel(ent, levelDir, &levelGround, m)) {
                    scriptFlatten.insert(scriptFlatten.end(), m.flatten.begin(),
                                         m.flatten.end());
                    scriptCache.emplace_back(&ent, std::move(m));
                }
            }
        }
    }
#endif

    // Road pre-pass (ADR-0044 corridor conforming): grade the terrain to each editable
    // road (shape:"road") BEFORE it builds, mirroring the script pre-pass, so the ground
    // meets the road's drivable profile and no terrain pokes through.
    std::vector<TerrainFlatten> roadFlatten;
    std::vector<engine::RoadNet> preNets;   // parsed nets, for the lot pre-pass
    RenderMesh roadWallMesh;                 // ADR-0075 P1b: retaining/fill walls (world space)
    if (levelGround && root.contains("entities")) {
        for (const auto& ent : root["entities"]) {
            if (ent.value("shape", std::string()) == "road") {
                const json roadBlock =
                    ent.contains("road") ? ent["road"] : json::object();
                RoadNet net = roadNetFromJson(roadBlock);
                // A GENERATED road has no baked nodes — run its recipe here
                // exactly like the real build below does, or this pre-pass
                // sees an empty net and carves NOTHING (device: "the road is
                // being buried by the terrain — it's not conforming").
                net.heightAt = levelGround;   // BEFORE generate: metro gates on terrain
                if (roadBlock.contains("generate"))
                    applyGenerateRecipe(net, roadBlock["generate"]);
                std::vector<TerrainFlatten> r = roadNetConformRegions(net);
                roadFlatten.insert(roadFlatten.end(), r.begin(), r.end());
                // Retaining/fill walls (ADR-0075 P1b) are DISABLED: they were sized
                // to the DaylightBatter reach, which regressed the conform and was
                // reverted to the smoothstep feather — so a wall at the old batter
                // step no longer matches the ground. buildRoadWalls stays in-tree
                // for a future attempt that co-designs the walls with the carve.
                (void)roadWallMesh;
                preNets.push_back(std::move(net));
            }
        }
    }
    HeightField entityGround = levelGround;   // entities drape on the carved terrain (below)

    // Terrain is parsed once into params + noise so vegetation can scatter on
    // the same surface it generates.
    GrownLots preLots;   // lots grown by the terrain pre-pass (reused below)
    if (root.contains("terrain")) {
        TerrainParams terrainParams = parseTerrainParams(root["terrain"]);
        terrainParams.erodedBase = sharedEroded;   // eroded base for mesh + carve + drape
        terrainParams.flatten = cityFlatten;   // grade flat under the city
        terrainParams.flatten.insert(terrainParams.flatten.end(),
                                     scriptFlatten.begin(), scriptFlatten.end());
        std::vector<TerrainFlatten> baseFlatten = terrainParams.flatten;   // non-road grading
        terrainParams.flatten.insert(terrainParams.flatten.end(),
                                     roadFlatten.begin(), roadFlatten.end());   // carve to roads
        unsigned terrainSeed = root["terrain"].value("seed", 0u);
        Noise terrainNoise(terrainSeed);
        // LOT PRE-PASS (device: "some of the buildings are sunk into the
        // terrain"): grow the living city's lots on the road-carved ground
        // BEFORE the terrain is meshed, so every building stamps a FLAT graded
        // pad (at its own plane) into the flatten set. The citysim build below
        // reuses these exact lots.
        if (root.contains("citysim") &&
            root["citysim"].value("buildLots", false) && !preNets.empty()) {
            auto lotTp = std::make_shared<TerrainParams>(terrainParams);
            auto lotNoise = std::make_shared<Noise>(terrainSeed);
            HeightField lotGround = [lotTp, lotNoise](double x, double z) {
                return terrainHeight(*lotTp, *lotNoise, x, z);
            };
            // BLOCK GRADING CASCADE (ADR-0075 P2) is DISABLED pending a device-
            // verified rework: on the real city the block grader either extracted
            // no faces (a terrain-gated metro is tree-like) or the graded ground
            // interacted badly with lot/pad placement. gradeBlocks + the priority
            // mechanism stay in-tree; re-enable behind a measured device pass.
            preLots = growCityLots(preNets, root["citysim"], levelDir, lotGround);
            for (const engine::LotBuilding& lb : preLots.lots) {
                if (lb.type == "park" || lb.type == "green" ||
                    lb.plan.size() < 3) continue;
                std::vector<Vec3> poly;
                poly.reserve(lb.plan.size());
                for (const engine::Vec2& v : lb.plan)
                    poly.push_back(Vec3(v.x, 0, v.y));
                TerrainFlatten f = makeFlattenPad(std::move(poly), lb.groundY, 5.0);
                terrainParams.flatten.push_back(f);
                baseFlatten.push_back(std::move(f));   // non-road grading
            }
        }
        // Index the assembled cut/fill set (ADR-0075 Phase 0): the CDLOD mesher,
        // collider, and road drape all sample terrainHeight per vertex, so the
        // O(footprints) scan there dominates the build. Shared, so the carved
        // copies below reuse it.
        rebuildFlattenIndex(terrainParams);
        loadTerrain(terrainParams, terrainNoise, root["terrain"], world, assets);
        // Hand the CDLOD config its non-road base so the editor's re-conform action can
        // rebuild flatten = base + fresh roads without double-applying or leaving ghosts.
        world.each<TerrainLodConfig>(
            [&](Entity, TerrainLodConfig& c) { c.baseFlatten = baseFlatten; });
        // Water surface (ocean / lake): a flat plane at seaLevel emitted only where
        // the NATURAL terrain floor dips below it. Drawn with the Water surface
        // shader (waves + depth-graded colour + shoreline foam baked into UV,
        // animated on windTime; low roughness + <1 opacity for SSR reflection and
        // fresnel). Uses levelGround (the un-carved floor) so it fills real basins.
        if (root["terrain"].contains("water") || root.contains("water")) {
            const json& w = root.contains("water") ? root["water"]
                                                   : root["terrain"]["water"];
            engine::WaterMeshParams wp;
            wp.seaLevel = w.value("seaLevel", 0.0);
            if (double region = w.value("region", 0.0); region > 0.0) {
                wp.lo = {-region, -region};
                wp.hi = {region, region};
            }
            if (w.contains("lo") && w["lo"].is_array())
                wp.lo = {w["lo"][0].get<double>(), w["lo"][1].get<double>()};
            if (w.contains("hi") && w["hi"].is_array())
                wp.hi = {w["hi"][0].get<double>(), w["hi"][1].get<double>()};
            wp.cell = w.value("cell", wp.cell);
            wp.foamBand = w.value("foamBand", wp.foamBand);
            RenderMesh wmesh = engine::buildWaterMesh(levelGround, wp);
            if (!wmesh.vertices.empty()) {
                Entity we = world.create();
                world.add<Transform>(we, Transform{});
                world.add<PrevTransform>(we, PrevTransform{Transform{}});
                Renderable wr;
                wr.material.albedo = Vec3(0.05, 0.14, 0.22);   // deep-water tint
                if (w.contains("color") && w["color"].is_array() &&
                    w["color"].size() == 3)
                    wr.material.albedo = Vec3(w["color"][0], w["color"][1],
                                             w["color"][2]);
                wr.material.roughness = static_cast<float>(w.value("roughness", 0.06));
                wr.material.opacity = static_cast<float>(w.value("opacity", 0.72));
                wr.material.setSurface(RenderMaterial::Surface::Water);
                wr.mesh = assets.acquireMesh(wmesh, "water");
                world.add<Renderable>(we, wr);
            }
        }
        // Retaining/fill walls (ADR-0075 P1b): one world-space entity for every
        // road's grade-break structures — concrete-grey, with a static MeshCollider
        // so cars and pedestrians can't walk through a cut face. The terrain batter
        // grades down to each wall's top; the wall caps the residual step to natural.
        if (!roadWallMesh.vertices.empty()) {
            Entity we = world.create();
            world.add<Transform>(we, Transform{});             // mesh is world-space
            world.add<PrevTransform>(we, PrevTransform{Transform{}});
            Renderable wr;
            wr.renderLayer = engine::LayerRoads;
            wr.material.albedo = Vec3(1, 1, 1);                // grey carried in vertex colour
            wr.material.roughness = 0.9f;
            wr.mesh = assets.acquireMesh(roadWallMesh, "road_walls");
            world.add<Renderable>(we, wr);
            MeshCollider mc;
            mc.vertices.reserve(roadWallMesh.vertices.size());
            for (const Vertex& v : roadWallMesh.vertices) mc.vertices.push_back(v.position);
            mc.indices = roadWallMesh.indices;
            mc.friction = 0.9;
            world.add<MeshCollider>(we, mc);
        }
        // Entities (roads especially) drape on the CARVED terrain, so a road sits exactly
        // on its graded profile instead of the raw ground it no longer matches.
        auto carvedTp = std::make_shared<TerrainParams>(terrainParams);
        auto carvedNoise = std::make_shared<Noise>(terrainSeed);
        entityGround = [carvedTp, carvedNoise](double x, double z) {
            return terrainHeight(*carvedTp, *carvedNoise, x, z);
        };
        if (root.contains("vegetation"))
            loadVegetation(root["vegetation"], terrainParams, terrainNoise, world,
                           renderer, assets, levelDir, "veg");
        // A second, denser pass for ground cover (grass/flowers). Same scatter
        // generator with its own params — typically a low maxSlopeDeg so it lands
        // on the gentle, green ground (terrainColor reads steep slopes as rock).
        if (root.contains("foliage"))
            loadVegetation(root["foliage"], terrainParams, terrainNoise, world,
                           renderer, assets, levelDir, "foliage");
    }

    if (root.contains("entities"))
        loadEntities(root["entities"], root, world, renderer, assets, levelDir,
                     editorMode, cityEnt, &cityModel, &scriptCache,
                     entityGround ? &entityGround : nullptr);

    // Drivable vehicles (ADR-0059) — play mode only (runtime actors, like the
    // player; not editor document entities).
#ifdef RT_ENABLE_SCRIPTING
    if (!editorMode && root.contains("vehicles"))
        loadVehicles(root["vehicles"], world, assets, levelDir);
#endif

    // Level-authored city-sim settings (ADR-0063): the top-level "citysim" block
    // becomes one CitySimConfig entity the citysim render bridge reads at build —
    // so a level can choose its own population, seed, clock rate, and whether the
    // agent-state debug HUD starts on (the agent lab: 1 car, 1 walker, HUD on).
    if (root.contains("citysim") && root["citysim"].is_object()) {
        const auto& cs = root["citysim"];
        CitySimConfig cfg;
        cfg.cars = cs.value("cars", cfg.cars);
        cfg.pedestrians = cs.value("pedestrians", cfg.pedestrians);
        cfg.seed = cs.value("seed", cfg.seed);
        cfg.hoursPerSecond = cs.value("hoursPerSecond", cfg.hoursPerSecond);
        cfg.perceptionReliability =
            cs.value("perceptionReliability", cfg.perceptionReliability);
        cfg.debugWidgets = cs.value("debugWidgets", cfg.debugWidgets);
        cfg.wander = cs.value("wander", cfg.wander);
        // Scripted goal tables (ADR-0064): `"agents": "agents.lua"` names a
        // goal-table script; its TEXT rides the config so the citysim bridge
        // (scripting builds only) can install the tables at build. Missing
        // file -> warn and fall back to the built-in tables.
        std::string agentsFile = cs.value("agents", std::string());
        if (!agentsFile.empty()) {
            cfg.agentScript = loadScriptCode(agentsFile, levelDir);
            if (cfg.agentScript.empty())
                LOG_WARN << "citysim: agents script '" << agentsFile
                         << "' not found — using built-in goal tables";
        }
        // Data-driven fleet bodies (ADR-0065): `"vehicles": "vehicles.lua"`
        // names a car-body script; its TEXT rides the config so the citysim
        // bridge (scripting builds only) builds the instanced fleet meshes from
        // its `vehicle.fleet` recipes. Missing file -> warn and fall back to the
        // built-in C++ fleet meshes. Opt-in per level, exactly like `agents`.
        std::string vehiclesFile = cs.value("vehicles", std::string());
        if (!vehiclesFile.empty()) {
            cfg.vehicleScript = loadScriptCode(vehiclesFile, levelDir);
            if (cfg.vehicleScript.empty())
                LOG_WARN << "citysim: vehicles script '" << vehiclesFile
                         << "' not found — using built-in fleet meshes";
        }
        // Authored places (ADR-0066): a `"places"` array of labelled destinations
        // the citysim bridge snaps onto the sidewalk network and turns into a
        // PlaceMap. Each: {type, position:[x,y,z] (y ignored), name?, open?, close?}.
        if (cs.contains("places") && cs["places"].is_array()) {
            for (const auto& pj : cs["places"]) {
                engine::AuthoredPlace p;
                p.type = pj.value("type", std::string());
                Vec3 pos = parseVec3(pj.value("position", json()));
                p.x = static_cast<float>(pos.x);
                p.z = static_cast<float>(pos.z);
                p.name = pj.value("name", std::string());
                p.openHour = pj.value("open", 0.0f);
                p.closeHour = pj.value("close", 24.0f);
                // Optional building footprint [w, h, d] + colour: a place that
                // names a `"building"` size becomes a real solid structure.
                Vec3 bs = parseVec3(pj.value("building", json()), Vec3(0, 0, 0));
                p.buildingW = static_cast<float>(bs.x);
                p.buildingH = static_cast<float>(bs.y);
                p.buildingD = static_cast<float>(bs.z);
                p.buildingColor = parseVec3(pj.value("color", json()),
                                            Vec3(0.72, 0.70, 0.64));
                cfg.places.push_back(p);

                // Spawn the building STRUCTURE (Living City Phase 4): a static box
                // resting on the ground at the site, so the labelled place is an
                // actual structure the player and agents collide with. Regenerated
                // from the recipe each load (no SourceSpec), like the road meshes.
                if (bs.x > 0 && bs.y > 0 && bs.z > 0) {
                    Entity b = world.create();
                    Transform t;
                    t.position = Vec3(pos.x, pos.y + bs.y * 0.5, pos.z);
                    world.add<Transform>(b, t);
                    world.add<PrevTransform>(b, PrevTransform{t});
                    Renderable r;
                    r.mesh = assets.acquirePrimitive("box", bs);
                    r.material.albedo = p.buildingColor;
                    r.material.metallic = 0.0f;
                    r.material.roughness = 0.9f;
                    r.renderLayer = engine::LayerBuildings;   // debug layer toggle
                    world.add<Renderable>(b, r);
                    Collider c;
                    c.shape = ColliderShape::Box;
                    c.halfExtent = bs * 0.5;
                    c.friction = 0.9;
                    world.add<Collider>(b, c);
                    RigidBody rb;
                    rb.motion = BodyMotion::Static;
                    world.add<RigidBody>(b, rb);
                }
            }
        }
        // Grow buildings on the ROAD NETWORK's blocks (ADR-0066): the Living City
        // path — real roads (a shape:"road" `generate` recipe, the tech grown.json
        // uses) whose enclosed blocks become lots and REAL shape-grammar buildings
        // (floors/windows/roof, fitting the lot), each tagged as a place agents
        // start/end their schedules at. Runs on the RoadNet(s) already in the world.
        if (cs.value("buildLots", false)) {
            // The lots: grown by the terrain pre-pass (city-on-terrain — their
            // graded pads are already baked into the ground), or grown here for
            // a flat city. Same helper, same deterministic result.
            GrownLots grown = std::move(preLots);
            if (!grown.grown) {
                std::vector<engine::RoadNet> nets;
                world.each<engine::RoadNet>(
                    [&](Entity, engine::RoadNet& net) { nets.push_back(net); });
                grown = growCityLots(nets, cs, levelDir, entityGround);
            }
            engine::LotPlanDebug& plan = grown.plan;   // debug overlay (below)
            // The buildings' geometry, merged by shape-grammar PartId across the
            // whole district — the SAME structure as CityModel::parts, so the same
            // PBR recipes bind below.
            std::vector<RenderMesh>& lotParts = grown.parts;
            MeshHandle pad = assets.acquirePrimitive("box", Vec3(1, 1, 1));   // park pads
            // Street-tree kit for parks + unbuilt greens (device: "empty lots had
            // vegetation like trees and grass"): a few shared varieties, one mesh
            // upload each, reused across every planted lot.
            struct TreeKit { MeshHandle bark, leaves;
                             RenderMaterial barkMat, leafMat; };
            std::vector<TreeKit> treeKits;
            auto treeKit = [&](std::size_t variety) -> const TreeKit& {
                while (treeKits.size() <= variety) {
                    const std::size_t i = treeKits.size();
                    const uint32_t seed = 0xF00Du + static_cast<uint32_t>(i) * 977u;
                    TreeParams tp;
                    tp.iterations = 4;              // modest street tree, light mesh
                    tp.rootCount = 0;               // no buttress roots on a lawn
                    TreeMesh tm = growTree(tp, seed);
                    auto up = [&](const TextureData& td) -> TextureHandle {
                        if (td.pixels.empty()) return TextureHandle{};
                        return renderer.uploadTexture(td.width, td.height,
                                                      td.channels, td.pixels.data());
                    };
                    TreeKit k;
                    const std::string key = "lotTree:" + std::to_string(i);
                    k.bark = assets.acquireMesh(tm.branches, key + ":bark");
                    k.barkMat.albedo = Vec3(1, 1, 1);
                    k.barkMat.roughness = 1.0f;
                    BarkMaps bm = barkMaps(barkStyleFromName("oak"), 128, seed);
                    k.barkMat.albedoMap = up(bm.albedo);
                    k.barkMat.normalMap = up(bm.normal);
                    if (!tm.leaves.vertices.empty()) {
                        k.leaves = assets.acquireMesh(tm.leaves, key + ":leaves");
                        k.leafMat.albedo = Vec3(1, 1, 1);
                        k.leafMat.roughness = 0.7f;
                        k.leafMat.albedoMap = up(leafTexture(128));
                        k.leafMat.flags |= RenderMaterial::FLAG_ALPHA_TEST;
                    }
                    treeKits.push_back(std::move(k));
                }
                return treeKits[variety];
            };
            // Building PHYSICS (device: "I can shoot through them"): one
            // static mesh collider of PLAN PRISMS — each building's grown
            // plan polygon extruded to its height. The prism follows the
            // massing exactly, so L / courtyard / prow buildings collide
            // where their walls are, with nothing spilling onto the sidewalk
            // (the failure that got box colliders removed).
            engine::MeshCollider buildingsMc;
            std::vector<engine::CityPlanDebug::Prism> colliderPrisms;
            for (const engine::LotBuilding& lb : grown.lots) {
                const double gy = entityGround ? entityGround(lb.site.x, lb.site.y) : 0.0;
                if (lb.type != "park" && lb.type != "green" &&
                    lb.plan.size() >= 3) {
                    const double base = lb.baseY - 0.5, top = lb.baseY + lb.height;
                    const uint32_t s0 =
                        static_cast<uint32_t>(buildingsMc.vertices.size());
                    const uint32_t n = static_cast<uint32_t>(lb.plan.size());
                    for (const engine::Vec2& v : lb.plan) {
                        buildingsMc.vertices.push_back(Vec3(v.x, base, v.y));
                        buildingsMc.vertices.push_back(Vec3(v.x, top, v.y));
                    }
                    for (uint32_t i = 0; i < n; ++i) {
                        const uint32_t j = (i + 1) % n;
                        const uint32_t a0 = s0 + i * 2, a1 = a0 + 1;
                        const uint32_t b0 = s0 + j * 2, b1 = b0 + 1;
                        buildingsMc.indices.insert(buildingsMc.indices.end(),
                                                   {a0, b0, b1, a0, b1, a1});
                    }
                    for (const auto& tri : engine::triangulatePolygon(lb.plan)) {
                        const uint32_t rb =
                            static_cast<uint32_t>(buildingsMc.vertices.size());
                        for (int k = 0; k < 3; ++k)
                            buildingsMc.vertices.push_back(Vec3(
                                lb.plan[tri[k]].x, top, lb.plan[tri[k]].y));
                        buildingsMc.indices.insert(buildingsMc.indices.end(),
                                                   {rb, rb + 1, rb + 2});
                    }
                    // Record the exact prism for the collider debug layer.
                    colliderPrisms.push_back({lb.plan, base, top});
                }
                // Tag it as a place the agents can route to. An unbuilt GREEN is
                // scenery, not a schedule destination — no place tag.
                if (lb.type != "green") {
                    engine::AuthoredPlace p;
                    p.type = lb.type;
                    p.x = static_cast<float>(lb.site.x);
                    p.z = static_cast<float>(lb.site.y);
                    cfg.places.push_back(std::move(p));
                }

                if (lb.type == "park" || lb.type == "green") {
                    // A grass pad plus a few trees, not a grown building. The
                    // pad is the LOT'S OWN polygon when the pass provides one
                    // (device: "square green lots don't fit the blocks"); the
                    // oriented box is only the legacy fallback.
                    Entity e = world.create();
                    Transform t;
                    Renderable r;
                    if (!lb.padMesh.vertices.empty()) {
                        t.position = Vec3(0, 0, 0);   // heights baked (draped)
                        r.mesh = assets.acquireMesh(
                            lb.padMesh, "lotPad:" + std::to_string(lb.site.x) +
                                        ":" + std::to_string(lb.site.y));
                    } else {
                        t.position = Vec3(lb.site.x, gy + lb.height * 0.5, lb.site.y);
                        t.scale = Vec3(lb.width, lb.height, lb.depth);
                        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), lb.yaw);
                        r.mesh = pad;
                    }
                    world.add<Transform>(e, t);
                    world.add<PrevTransform>(e, PrevTransform{t});
                    r.material.albedo = lb.color;
                    r.material.roughness = 1.0f;
                    r.renderLayer = engine::LayerBuildings;   // debug layer toggle
                    world.add<Renderable>(e, r);

                    // Trees: deterministic count + spots from the lot position,
                    // scaled by the pad's real area so a block-sized park reads
                    // as a park (device: "scatter more trees around it").
                    const uint32_t h = static_cast<uint32_t>(
                        std::llround(lb.site.x * 73.1 + lb.site.y * 37.7)) * 2654435761u;
                    const double padArea = lb.pad.empty()
                        ? static_cast<double>(lb.width * lb.depth)
                        : engine::area(lb.pad);
                    const int nTrees = lb.type == "park"
                        ? std::max(3, std::min(14, static_cast<int>(padArea / 60.0)))
                        : std::max(1, std::min(4, static_cast<int>(padArea / 140.0)));
                    for (int ti = 0; ti < nTrees; ++ti) {
                        const uint32_t th = h ^ (0x9e3779b9u * static_cast<uint32_t>(ti + 1));
                        // Spread across ~90% of the pad; the point-in-polygon
                        // shrink below pulls strays back onto the grass.
                        const double fx = ((th & 0xFFu) / 255.0 - 0.5) * 0.9;
                        const double fz = (((th >> 8) & 0xFFu) / 255.0 - 0.5) * 0.9;
                        const double cy = std::cos(lb.yaw), sy = std::sin(lb.yaw);
                        double lx = fx * lb.width, lz = fz * lb.depth;
                        // Keep the tree on the pad: shrink toward the centroid
                        // until the spot is inside the lot polygon.
                        if (!lb.pad.empty())
                            for (double f = 1.0; f > 0.1; f *= 0.55) {
                                engine::Vec2 spot(lb.site.x + (lx * cy - lz * sy) * f,
                                                  lb.site.y + (lx * sy + lz * cy) * f);
                                if (engine::pointInPolygon(lb.pad, spot) || f * 0.55 <= 0.1) {
                                    lx *= f; lz *= f; break;
                                }
                            }
                        Vec3 tPos(lb.site.x + lx * cy - lz * sy, 0,
                                  lb.site.y + lx * sy + lz * cy);
                        tPos.y = entityGround ? entityGround(tPos.x, tPos.z) : 0.0;
                        const TreeKit& kit = treeKit((th >> 16) % 3u);
                        const double scale = 0.8 + ((th >> 24) & 0x3Fu) / 63.0 * 0.6;
                        Transform tt;
                        tt.position = tPos;
                        tt.scale = Vec3(scale, scale, scale);
                        tt.orientation = Quat::fromAxisAngle(
                            Vec3(0, 1, 0), (th % 628u) / 100.0);
                        Entity te = world.create();
                        world.add<Transform>(te, tt);
                        world.add<PrevTransform>(te, PrevTransform{tt});
                        Renderable tr;
                        tr.mesh = kit.bark;
                        tr.material = kit.barkMat;
                        world.add<Renderable>(te, tr);
                        if (kit.leaves.index) {
                            Entity le = world.create();
                            world.add<Transform>(le, tt);
                            world.add<PrevTransform>(le, PrevTransform{tt});
                            Renderable lr;
                            lr.mesh = kit.leaves;
                            lr.material = kit.leafMat;
                            world.add<Renderable>(le, lr);
                        }
                    }
                }
            }
            if (!buildingsMc.indices.empty()) {
                // Jolt mesh triangles are SINGLE-SIDED, and the grown plans
                // arrive with mixed winding (offset/prow/courtyard plans flip
                // orientation) — a wrong-way wall lets bullets sail through
                // from outside (device: "I can shoot through them at certain
                // angles"). Emit every triangle both ways so the prism is
                // solid regardless of plan winding.
                const std::size_t oneSided = buildingsMc.indices.size();
                buildingsMc.indices.reserve(oneSided * 2);
                for (std::size_t i = 0; i + 2 < oneSided; i += 3) {
                    buildingsMc.indices.push_back(buildingsMc.indices[i]);
                    buildingsMc.indices.push_back(buildingsMc.indices[i + 2]);
                    buildingsMc.indices.push_back(buildingsMc.indices[i + 1]);
                }
                Entity ce = world.create();
                Transform ct;
                world.add<Transform>(ce, ct);
                world.add<PrevTransform>(ce, PrevTransform{ct});
                buildingsMc.friction = 0.85;
                world.add<engine::MeshCollider>(ce, std::move(buildingsMc));
            }
            // One entity per non-empty part class, with the shape-grammar's OWN
            // material recipes: materialFor(PartId) names the procedural surface
            // (brick/concrete/stucco/metal), which gets world-scaled UVs + the
            // baked PBR texture set — identical to the shape:"city" pipeline
            // (user: "we absolutely should be using the pre-existing recipes").
            {
                using Surface = RenderMaterial::Surface;
                SurfaceTexCache lotTex;   // one bake+upload per surface class
                for (std::size_t pi = 0; pi < lotParts.size(); ++pi) {
                    RenderMesh& pm = lotParts[pi];
                    if (pm.vertices.empty()) continue;
                    Renderable r;
                    r.renderLayer = engine::LayerBuildings;   // debug layer toggle
                    r.material = materialFor(static_cast<PartId>(pi),
                                             Vec3(0.80, 0.78, 0.75));
                    const Surface surf = r.material.surface();
                    if (surf != Surface::None) {
                        applyWorldPlanarUVs(pm, 1.0 / surfaceWorldTileSize(surf));
                        bindSurfaceMaps(r.material,
                                        bakeSurfaceTextures(renderer, surf, lotTex));
                    }
                    r.mesh = assets.acquireMesh(pm, "");   // world-space, unkeyed
                    Entity e = world.create();
                    Transform t;   // identity — the merged mesh sits in world space
                    world.add<Transform>(e, t);
                    world.add<PrevTransform>(e, PrevTransform{t});
                    world.add<Renderable>(e, r);
                }
            }
            // Publish the plan (blocks + lots + collider prisms) for the
            // citysim debug overlay.
            if (!plan.blocks.empty() || !colliderPrisms.empty()) {
                engine::CityPlanDebug dbg;
                dbg.blocks = std::move(plan.blocks);
                dbg.lots = std::move(plan.lots);
                dbg.prisms = std::move(colliderPrisms);
                world.add<engine::CityPlanDebug>(world.create(), std::move(dbg));
            }
        }
        // Real buildings become a living city (ADR-0066): when the level has a
        // generated `shape:"city"` entity, drive the citysim on ITS streets and
        // tag ITS buildings as places — no hand-authoring. The generator already
        // built the road graph; spawn a nav-only RoadNet from it (no Renderable —
        // the city drew its own carriageway) so `world.each<RoadNet>` finds it,
        // and map each building's District to a place type.
        if (!cityModel.roadGraph.edges.empty()) {
            engine::RoadNet net;
            net.nodes.reserve(cityModel.roadGraph.nodes.size());
            for (const auto& n : cityModel.roadGraph.nodes) net.nodes.push_back(n.pos);
            net.edges.reserve(cityModel.roadGraph.edges.size());
            net.edgeWidths.reserve(cityModel.roadGraph.edges.size());
            for (const auto& e : cityModel.roadGraph.edges) {
                net.edges.push_back({e.a, e.b});
                net.edgeWidths.push_back(e.width);   // real per-road width (arterial vs street)
            }
            net.width = 12.0;      // fallback ribbon (grid city; district uses edgeWidths)
            net.sidewalk = 2.5;
            net.markings = false;  // the city already drew its own road surface
            net.crosswalks = false;
            // Drape on terrain (ADR-0066): an on-terrain city's roads conform to
            // the ground, so the sim must too — CityRenderSystem reads net.heightAt
            // for agent elevation. Without this, agents float over / sink under a
            // hilly city's streets.
            if (entityGround) net.heightAt = entityGround;
            world.add<engine::RoadNet>(world.create(), net);

            // District → place type. Kept as string tags (the citysim bridge
            // parses them) so engine core stays free of citysim's PlaceType.
            auto placeTag = [](engine::District d) -> const char* {
                switch (d) {
                    case engine::District::Residential: return "home";
                    case engine::District::Commercial:  return "shop";
                    case engine::District::HighRise:     return "office";
                    case engine::District::Industrial:   return "office";
                    case engine::District::Park:         return "park";
                    default:                             return "civic";
                }
            };
            for (const engine::CityBuilding& b : cityModel.buildings) {
                engine::AuthoredPlace p;
                p.type = placeTag(b.district);
                p.x = static_cast<float>(b.site.x);
                p.z = static_cast<float>(b.site.y);
                cfg.places.push_back(std::move(p));   // building already drawn — label only
            }
        }
        world.add<CitySimConfig>(world.create(), cfg);
    }

    // RT_NO_PLAYER=1 suppresses the player entirely — for headless screenshots / debug renders, so
    // the first-person gun viewmodel and a settling capsule don't intrude on an overhead frame dump.
    if (!std::getenv("RT_NO_PLAYER")) {
        if (root.contains("player")) {
            json pj = root["player"];
            // Terrain levels: never spawn under a hill — authored spawns
            // assume flat ground, so lift the point to the surface when the
            // terrain there is higher.
            if (entityGround && pj.contains("position") &&
                pj["position"].is_array() && pj["position"].size() >= 3) {
                const double px = pj["position"][0].get<double>();
                const double py = pj["position"][1].get<double>();
                const double pz = pj["position"][2].get<double>();
                const double gy = entityGround(px, pz);
                if (py < gy + 1.2) pj["position"][1] = gy + 1.2;
            }
            (editorMode ? loadPlayerSpawn(pj, world, assets)
                        : loadPlayer(pj, world));
        }
        else if (!editorMode) {
            // Every playable level gets a player. With no authored spawn, drop one in from above so
            // the character settles onto the (collidable) ground rather than the level having no actor.
            json def; def["position"] = json::array({0.0, 200.0, 0.0});
            loadPlayer(def, world);
        }
    }

    if (root.contains("lighting"))
        loadLighting(root["lighting"], view);

    // Environment map (equirectangular .hdr) — bound before probes so the bake
    // captures it for IBL (ADR-0016). Lives under the "environment" object as
    // "hdr"; path is relative to the level file.
    if (root.contains("environment") && root["environment"].is_object()) {
        const auto& env = root["environment"];
        // Aerial-perspective fog (matches the offline tracer's Scene::fog). Lives
        // under "environment" alongside the sky; pushed to the renderer via the
        // lighting block (setLights). density 0 = off.
        if (env.contains("fog") && env["fog"].is_object()) {
            const auto& f = env["fog"];
            view.lighting.fog.enabled = true;
            view.lighting.fog.density = f.value("density", 0.0f);
            view.lighting.fog.color =
                parseVec3(f.value("color", json()), view.lighting.fog.color);
        }
        // Reset FIRST: only a successful HDR load below may raise this. It is
        // the flag DayNightSystem uses to yield to a baked HDR sun — leaving a
        // previous level's value in the reused renderer silently disabled the
        // day/night cycle on every level loaded after an HDR one (device:
        // "the day/night cycle is broken in Metal" — really "after an HDR
        // level", which a fresh single-level web session never hits).
        renderer.environmentAvgLuminance = 0.0f;
        if (env.contains("hdr")) {
            std::string envPath = env["hdr"].get<std::string>();
            if (!envPath.empty() && envPath[0] != '/')
                envPath = levelDir + "/" + envPath;
            // The HDR's dominant light drives the shadow-casting sun (ADR-0017
            // Phase 2) so shadows match the sun baked into the image; set
            // "driveSun": false to keep the level's authored sun instead.
            bool driveSun = env.value("driveSun", true);
            renderer.setEnvironmentMap(EnvironmentLoader::loadEnvironmentMap(
                envPath, renderer, driveSun ? &view.lighting.sun : nullptr));
        } else {
            // No HDR in this level: clear any map a previously loaded level bound,
            // otherwise its cubemap/IBL persists (the renderer is reused across
            // loads) and you get a stale HDR sky you can't turn off.
            renderer.setEnvironmentMap(TextureHandle{});
        }
    } else {
        // No "environment" object at all — also clear any stale env map (and
        // the avg-luminance flag, or the day/night cycle stays disabled).
        renderer.environmentAvgLuminance = 0.0f;
        renderer.setEnvironmentMap(TextureHandle{});
    }

    if (root.contains("reflectionProbes")) {
        std::vector<ReflectionProbe> probes;
        for (auto& rp : root["reflectionProbes"]) {
            ReflectionProbe probe;
            probe.position        = parseVec3(rp["position"]);
            probe.influenceRadius = rp.value("radius", 10.0f);
            probe.boxMin          = parseVec3(rp["boxMin"]);
            probe.boxMax          = parseVec3(rp["boxMax"]);
            probe.priority        = rp.value("priority", 0);
            probes.push_back(probe);
        }
        renderer.setReflectionProbes(probes);
    }

    LOG_INFO << "Loaded level: " << path << " (v" << version << ", "
             << world.entityCount() << " entities)";
    return true;
}

}  // namespace engine
