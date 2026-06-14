#include "level_loader.h"
#include "mesh_builder.h"
#include "asset_manager.h"
#include "procgen/terrain.h"
#include "procgen/lsystem.h"
#include "procgen/rock.h"
#include "procgen/scatter.h"
#include "procgen/node_graph.h"
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

static void loadEntities(const json& entities, World& world, Renderer& renderer,
                         AssetManager& assets, const std::string& levelDir,
                         bool editorMode) {
    for (auto& ent : entities) {
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
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});

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
    world.add<Collider>(e, c);

    RigidBody rb;
    rb.motion = BodyMotion::Dynamic;
    rb.lockRotation = true;
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
    return p;
}

// Procedural terrain (ADR-0021 persistence: the document stores the recipe —
// seed + params — and the engine regenerates the mesh at load, rather than
// serializing the geometry). The terrain entity carries no SourceSpec, so the
// LevelWriter never writes it back as a document entity (it stays a regenerated
// runtime object); its GPU mesh is owned by the AssetManager and freed on the
// next clear().
static void loadTerrain(const TerrainParams& p, const Noise& noise, const json& t,
                        World& world, AssetManager& assets) {
    Entity e = world.create();
    Transform tr;   // generated directly in world space
    world.add<Transform>(e, tr);
    world.add<PrevTransform>(e, PrevTransform{tr});

    RenderMesh terrainMesh = generateTerrain(p, noise);

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
}

// Vegetation: generate a few "species" meshes (L-system trees, noise rocks)
// once, then scatter them across the terrain as individual entities sharing
// each species' GPU mesh (AssetManager dedup). Per-entity rendering for now —
// instancing (the thousands-scale path) comes later. Carries no SourceSpec, so
// these are regenerated runtime objects, not document entities.
static void loadVegetation(const json& veg, const TerrainParams& terrain,
                           const Noise& terrainNoise, World& world,
                           AssetManager& assets, const std::string& levelDir,
                           const std::string& tag = "veg") {
    if (!veg.contains("species") || !veg["species"].is_array()) return;

    struct Species { MeshHandle mesh; RenderMaterial material; };
    std::vector<Species> species;
    uint32_t vegSeed = veg.value("seed", 0u);

    // Node-graph generators (ADR-0021 Phase C): a species may define its mesh as
    // a "graph" asset evaluated per variant with a seed, instead of the built-in
    // tree/rock generators.
    NodeRegistry nodeRegistry;
    registerBuiltinNodes(nodeRegistry);

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

        // Optional: this species' mesh is a node-graph asset, evaluated per
        // variant with a "seed" parameter.
        Graph graph;
        bool hasGraph = false;
        if (s.contains("graph")) {
            std::string graphPath = s["graph"].get<std::string>();
            if (!graphPath.empty() && graphPath[0] != '/') graphPath = levelDir + "/" + graphPath;
            std::ifstream gf(graphPath);
            if (gf) {
                std::string text((std::istreambuf_iterator<char>(gf)),
                                 std::istreambuf_iterator<char>());
                graph = graphFromJson(text);
                hasGraph = !graph.nodes.empty();
            }
            if (!hasGraph) LOG_ERROR << "Failed to load generator graph: " << graphPath;
        }

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
            RenderMesh mesh;
            if (hasScript) {
#ifdef RT_ENABLE_SCRIPTING
                ScriptVM& vm = ensureScriptVm();
                vm.doString("seed=" + std::to_string(seed));   // the script reads it
                std::shared_ptr<RenderMesh> out;
                std::string err;
                if (runProcgenMesh(vm, scriptSource, out, &err) && out) mesh = *out;
                else LOG_ERROR << "flora script error: " << err;
#endif
            } else if (hasGraph) {
                GraphValue out = graph.evaluate(
                    nodeRegistry, {{"seed", GraphValue(static_cast<double>(seed))}});
                if (auto* mp = std::get_if<MeshPtr>(&out)) { if (*mp) mesh = **mp; }
            } else if (kind == "rock") {
                mesh = rockSdf ? generateRockSdf(rsp, seed)
                               : generateRock(rp, Noise(seed));
            } else {
                mesh = treeSdf ? generateTreeSdf(sys, axiom, iterations, tp,
                                                 treeSmooth, treeRes, seed)
                               : generateTree(sys, axiom, iterations, tp, seed);
                MeshBuilder::bakeHeightColor(mesh, trunkColor, leafColor);
            }
            if (mesh.vertices.empty()) continue;
            Species sp;
            sp.mesh = assets.acquireMesh(
                mesh, tag + ":" + std::to_string(speciesIndex) + ":" + std::to_string(v));
            sp.material = material;
            species.push_back(sp);
        }
        speciesIndex++;
    }
    if (species.empty()) return;

    ScatterParams scatter;
    scatter.regionSize       = veg.value("region", 70.0f);
    scatter.count            = veg.value("count", 80);
    scatter.maxSlopeDeg      = veg.value("maxSlopeDeg", 40.0f);
    scatter.minScale         = veg.value("minScale", 0.7f);
    scatter.maxScale         = veg.value("maxScale", 1.3f);
    scatter.densityScale     = veg.value("densityScale", 0.05);
    scatter.densityThreshold = veg.value("densityThreshold", -0.2f);
    scatter.seed             = vegSeed;

    std::vector<Placement> placements = scatterOnTerrain(scatter, terrain, terrainNoise);

    // A separate stream picks which species each placement gets (deterministic).
    std::mt19937 pick(vegSeed + 7u);
    std::uniform_int_distribution<size_t> speciesPick(0, species.size() - 1);
    for (const Placement& pl : placements) {
        const Species& sp = species[speciesPick(pick)];
        Entity e = world.create();
        Transform t;
        t.position = pl.position;
        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), pl.yaw);
        t.scale = Vec3(pl.scale, pl.scale, pl.scale);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.mesh = sp.mesh;
        r.material = sp.material;
        world.add<Renderable>(e, r);
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
            loadVegetation(root["vegetation"], terrainParams, terrainNoise, world, assets,
                           levelDir, "veg");
        // A second, denser pass for ground cover (grass/flowers). Same scatter
        // generator with its own params — typically a low maxSlopeDeg so it lands
        // on the gentle, green ground (terrainColor reads steep slopes as rock).
        if (root.contains("foliage"))
            loadVegetation(root["foliage"], terrainParams, terrainNoise, world, assets,
                           levelDir, "foliage");
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
