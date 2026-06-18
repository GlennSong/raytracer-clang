#include "level_loader.h"
#include "mesh_builder.h"
#include "asset_manager.h"
#include "procgen/terrain.h"
#include "procgen/erosion.h"
#include "procgen/lsystem.h"
#include "procgen/tree.h"
#include "procgen/rock.h"
#include "procgen/scatter.h"
#ifdef RT_ENABLE_SCRIPTING
#include "scripting/script_vm.h"
#include "scripting/procgen_bindings.h"
#endif
#include "model_importer.h"
#include <random>
#include "components.h"
#include "property_json.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <unordered_map>
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
}

static RenderMaterial parseMaterial(const json& j) {
    RenderMaterial mat;
    applyMaterial(j, mat);
    return mat;
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

static void loadEntities(const json& entities, World& world, Renderer& renderer,
                         AssetManager& assets, const std::string& levelDir,
                         bool editorMode) {
    int treeIndex = 0;
    for (auto& ent : entities) {
        // Hero parametric tree: a collidable, textured object (not scatter).
        if (ent.value("shape", std::string()) == "tree") {
            loadTreeEntity(ent, world, renderer, assets, treeIndex++);
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
                // keep it (JsonReadVisitor's missing-key semantics).
                if (ent.contains("material"))
                    applyMaterial(ent["material"], r.material);
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

        MeshHandle meshHandle = getOrCreateMesh(shape, sizeJ, assets);
        Renderable r;
        r.mesh = meshHandle;
        if (ent.contains("material"))
            r.material = parseMaterial(ent["material"]);
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

    Collider c;
    if (player.contains("collider")) {
        auto& col = player["collider"];
        std::string shape = col.value("shape", "capsule");
        if (shape == "capsule") {
            c.shape = ColliderShape::Capsule;
            c.radius = col.value("radius", 0.3);
            c.halfHeight = col.value("halfHeight", 0.4);
        } else if (shape == "sphere") {
            c.shape = ColliderShape::Sphere;
            c.radius = col.value("radius", 0.5);
        } else {
            c.shape = ColliderShape::Box;
            c.halfExtent = parseVec3(col["halfExtent"], Vec3(0.5, 0.5, 0.5));
        }
    }
    c.friction = player.value("friction", 0.5);

    // CDLOD terrain: snap the spawn to just above the surface at its XZ. The
    // collider is a thin triangle mesh, so a tall drop (the authored y far above
    // the ground) builds up enough speed to tunnel through it in one step, and a y
    // below the surface spawns embedded — either reads as "fell through". Scoped to
    // CDLOD (static-chunk levels have no TerrainLodConfig and keep the authored y).
    const TerrainLodConfig* tc = nullptr;
    world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& cfg) { if (!tc) tc = &cfg; });
    if (tc) {
        Noise noise(tc->seed);
        double surface = terrainHeight(tc->params, noise, t.position.x, t.position.z);
        double clearance = c.radius +
                           (c.shape == ColliderShape::Capsule ? c.halfHeight : 0.0) +
                           1.0;   // drop ~1 m onto the surface, no tunnelling
        t.position.y = surface + clearance;
    }

    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    world.add<Collider>(e, c);

    RigidBody rb;
    rb.motion = BodyMotion::Dynamic;
    rb.lockRotation = true;
    rb.continuousCollision = true;   // don't tunnel through terrain on a long fall
    world.add<RigidBody>(e, rb);
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
    if (t.value("erode", false)) {
        // Bake -> erode (drainage detail) -> mesh; the eroded grid is the source
        // of truth for the mesh (and its collider) here.
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

    // Terrain is parsed once into params + noise so vegetation can scatter on
    // the same surface it generates.
    if (root.contains("terrain")) {
        TerrainParams terrainParams = parseTerrainParams(root["terrain"]);
        Noise terrainNoise(root["terrain"].value("seed", 0u));
        loadTerrain(terrainParams, terrainNoise, root["terrain"], world, assets);
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
        loadEntities(root["entities"], world, renderer, assets, levelDir, editorMode);

    if (root.contains("player"))
        (editorMode ? loadPlayerSpawn(root["player"], world, assets)
                    : loadPlayer(root["player"], world));

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
        }
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
