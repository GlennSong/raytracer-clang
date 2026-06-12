#include "level_scene.h"

#include "engine/mesh_builder.h"
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

// Tessellated shape -> world-space triangles. The entity transform is applied
// per vertex (scale, then rotate, then translate — matching Transform::matrix).
void addMeshAsTriangles(const RenderMesh& mesh, const Vec3& position,
                        const Quat& orientation, const Vec3& scale,
                        int matIdx, Scene& scene) {
    auto toWorld = [&](const Vertex& v) {
        Vec3 scaled(v.position.x * scale.x, v.position.y * scale.y,
                    v.position.z * scale.z);
        return position + orientation.rotate(scaled);
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        scene.addTriangle(toWorld(mesh.vertices[mesh.indices[i]]),
                          toWorld(mesh.vertices[mesh.indices[i + 1]]),
                          toWorld(mesh.vertices[mesh.indices[i + 2]]), matIdx);
    }
}

bool isIdentity(const Quat& q) {
    return q.x == 0.0 && q.y == 0.0 && q.z == 0.0;
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

    int skipped = 0;
    for (const auto& ent : root.value("entities", json::array())) {
        if (ent.contains("mesh")) {
            skipped++;   // glTF models aren't tessellated offline yet
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
