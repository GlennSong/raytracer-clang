#include "level_loader.h"
#include "mesh_builder.h"
#include "model_importer.h"
#include "components.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <unordered_map>
#include <tuple>

using json = nlohmann::json;

namespace engine {

static Vec3 parseVec3(const json& j, Vec3 fallback = Vec3()) {
    if (!j.is_array() || j.size() < 3) return fallback;
    return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

static RenderMaterial parseMaterial(const json& j) {
    RenderMaterial mat;
    if (j.contains("albedo"))    mat.albedo    = parseVec3(j["albedo"], mat.albedo);
    if (j.contains("roughness")) mat.roughness = j["roughness"].get<float>();
    if (j.contains("metallic"))  mat.metallic  = j["metallic"].get<float>();
    if (j.contains("opacity"))   mat.opacity   = j["opacity"].get<float>();
    if (j.contains("emission"))  mat.emission  = parseVec3(j["emission"]);
    if (j.contains("flags")) {
        for (auto& flag : j["flags"]) {
            if (flag.get<std::string>() == "checkerboard")
                mat.flags |= RenderMaterial::FLAG_CHECKERBOARD;
        }
    }
    return mat;
}

struct MeshKey {
    std::string shape;
    double d1, d2, d3;
    bool operator==(const MeshKey& o) const {
        return shape == o.shape && d1 == o.d1 && d2 == o.d2 && d3 == o.d3;
    }
};

struct MeshKeyHash {
    size_t operator()(const MeshKey& k) const {
        size_t h = std::hash<std::string>{}(k.shape);
        h ^= std::hash<double>{}(k.d1) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<double>{}(k.d2) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<double>{}(k.d3) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

static MeshHandle getOrCreateMesh(
    const std::string& shape, const json& sizeJ,
    Renderer& renderer,
    std::unordered_map<MeshKey, MeshHandle, MeshKeyHash>& cache)
{
    Vec3 sz = parseVec3(sizeJ, Vec3(1, 1, 1));
    MeshKey key{shape, sz.x, sz.y, sz.z};

    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    RenderMesh mesh = MeshBuilder::shape(shape, sz);
    if (mesh.vertices.empty()) {
        LOG_ERROR << "Unknown shape: " << shape;
        return MeshHandle{};
    }

    MeshHandle handle = renderer.uploadMesh(mesh);
    cache[key] = handle;
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
                         const std::string& levelDir) {
    std::unordered_map<MeshKey, MeshHandle, MeshKeyHash> meshCache;

    for (auto& ent : entities) {
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
                if (ent.contains("material")) {
                    auto overrides = parseMaterial(ent["material"]);
                    if (ent["material"].contains("albedo"))
                        r.material.albedo = overrides.albedo;
                    if (ent["material"].contains("roughness"))
                        r.material.roughness = overrides.roughness;
                    if (ent["material"].contains("metallic"))
                        r.material.metallic = overrides.metallic;
                    if (ent["material"].contains("opacity"))
                        r.material.opacity = overrides.opacity;
                }
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

        MeshHandle meshHandle = getOrCreateMesh(shape, sizeJ, renderer, meshCache);
        Renderable r;
        r.mesh = meshHandle;
        if (ent.contains("material"))
            r.material = parseMaterial(ent["material"]);
        world.add<Renderable>(e, r);

        addPhysics(e, ent, shape, world);
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
                            Renderer& renderer) {
    Entity e = world.create();
    Transform t;
    if (player.contains("position"))
        t.position = parseVec3(player["position"]);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    world.add<PlayerSpawn>(e);

    Renderable gizmo;
    gizmo.mesh = renderer.uploadMesh(MeshBuilder::capsule(0.3f, 0.8f));
    gizmo.material.albedo = Vec3(0.2, 0.8, 0.3);   // green = "you start here"
    gizmo.material.roughness = 0.5f;
    world.add<Renderable>(e, gizmo);
}

bool LevelLoader::load(const std::string& path,
                       World& world, Renderer& renderer, RenderView& view) {
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

    if (root.contains("entities"))
        loadEntities(root["entities"], world, renderer, levelDir);

    if (root.contains("player"))
        (editorMode ? loadPlayerSpawn(root["player"], world, renderer)
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
