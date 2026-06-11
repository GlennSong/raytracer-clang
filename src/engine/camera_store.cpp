#include "camera_store.h"

#include "components.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace engine {

static json vec3ToJson(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

static Vec3 jsonToVec3(const json& j, Vec3 fallback = Vec3()) {
    if (!j.is_array() || j.size() != 3) return fallback;
    return Vec3(j[0].get<Real>(), j[1].get<Real>(), j[2].get<Real>());
}

bool CameraStore::load(const std::string& path, World& world) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    json root;
    try {
        root = json::parse(file);
    } catch (const json::exception& e) {
        LOG_WARN << "Camera store " << path << " is malformed: " << e.what();
        return false;
    }

    int count = 0;
    for (const auto& c : root.value("cameras", json::array())) {
        Transform t;
        t.position = jsonToVec3(c.value("position", json()));
        t.orientation = orientationFromForward(
            jsonToVec3(c.value("forward", json()), Vec3(0, 0, -1)));

        SceneCamera cam;
        cam.name = c.value("name", std::string("Camera"));
        if (c.contains("lens")) {
            const auto& l = c["lens"];
            cam.lens.focalLength = l.value("focalLength", cam.lens.focalLength);
            cam.lens.sensorHeight = l.value("sensorHeight", cam.lens.sensorHeight);
            cam.lens.fStop = l.value("fStop", cam.lens.fStop);
            cam.lens.focusDistance = l.value("focusDistance", cam.lens.focusDistance);
            cam.lens.nearPlane = l.value("near", cam.lens.nearPlane);
            cam.lens.farPlane = l.value("far", cam.lens.farPlane);
            cam.lens.distortionK1 = l.value("k1", cam.lens.distortionK1);
            cam.lens.distortionK2 = l.value("k2", cam.lens.distortionK2);
            cam.lens.chromaticAberration =
                l.value("chromaticAberration", cam.lens.chromaticAberration);
            cam.lens.vignette = l.value("vignette", cam.lens.vignette);
        }

        Entity e = world.create();
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, {t});
        world.add<SceneCamera>(e, cam);
        count++;
    }

    LOG_INFO << "Loaded " << count << " camera(s) from " << path;
    return true;
}

bool CameraStore::save(const std::string& path, World& world) {
    json cameras = json::array();
    world.each<Transform, SceneCamera>(
        [&](Entity, Transform& t, SceneCamera& cam) {
            json c;
            c["name"] = cam.name;
            c["position"] = vec3ToJson(t.position);
            c["forward"] = vec3ToJson(t.orientation.rotate(Vec3(0, 0, -1)));
            c["lens"] = {
                {"focalLength", cam.lens.focalLength},
                {"sensorHeight", cam.lens.sensorHeight},
                {"fStop", cam.lens.fStop},
                {"focusDistance", cam.lens.focusDistance},
                {"near", cam.lens.nearPlane},
                {"far", cam.lens.farPlane},
                {"k1", cam.lens.distortionK1},
                {"k2", cam.lens.distortionK2},
                {"chromaticAberration", cam.lens.chromaticAberration},
                {"vignette", cam.lens.vignette},
            };
            cameras.push_back(c);
        });

    json root;
    root["version"] = 1;
    root["cameras"] = cameras;

    std::ofstream file(path);
    if (!file.is_open()) {
        LOG_WARN << "Cannot write camera store: " << path;
        return false;
    }
    file << root.dump(2) << "\n";
    return true;
}

}  // namespace engine
