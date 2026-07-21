#include "camera_store.h"

#include "components.h"
#include "property_json.h"
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
        // Round-trips so a scene can author cameras that DON'T draw a body.
        // Without it every saved camera is a solid box in the world, and a
        // handful of close-in views around one prop bury the prop itself —
        // which is exactly what happened to the car lab. Defaults true, so
        // existing sidecars are unaffected.
        cam.showGizmo = c.value("show_gizmo", true);
        // The lens block is the property layer's JSON form (FieldMeta ids
        // match this file's historical keys); missing keys keep defaults.
        if (c.contains("lens")) {
            JsonReadVisitor reader(c["lens"]);
            describeProperties(cam.lens, reader);
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
            c["show_gizmo"] = cam.showGizmo;
            c["position"] = vec3ToJson(t.position);
            c["forward"] = vec3ToJson(t.orientation.rotate(Vec3(0, 0, -1)));
            json lens;
            JsonWriteVisitor writer(lens);
            describeProperties(cam.lens, writer);
            c["lens"] = std::move(lens);
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
