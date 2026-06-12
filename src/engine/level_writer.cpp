#include "level_writer.h"

#include "components.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace engine {

static json vec3ToJson(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

static bool approxOne(const Vec3& v) {
    return std::abs(v.x - 1.0) < 1e-9 && std::abs(v.y - 1.0) < 1e-9 &&
           std::abs(v.z - 1.0) < 1e-9;
}

static json materialToJson(const RenderMaterial& m) {
    json mat;
    mat["albedo"] = json::array({m.albedo.x, m.albedo.y, m.albedo.z});
    mat["roughness"] = m.roughness;
    mat["metallic"] = m.metallic;
    if (m.opacity != 1.0f) mat["opacity"] = m.opacity;
    if (m.emission.lengthSquared() > 0.0)
        mat["emission"] = vec3ToJson(m.emission);
    if (m.flags & RenderMaterial::FLAG_CHECKERBOARD)
        mat["flags"] = json::array({"checkerboard"});
    return mat;
}

static json entityToJson(const Transform& t, const SourceSpec& spec,
                         const RenderMaterial* material) {
    json ent;
    if (!spec.meshFile.empty()) {
        ent["mesh"] = spec.meshFile;
    } else {
        ent["shape"] = spec.shape;
        ent["size"] = vec3ToJson(spec.size);
    }
    ent["position"] = vec3ToJson(t.position);
    if (!approxOne(t.scale)) ent["scale"] = vec3ToJson(t.scale);

    Vec3 axis;
    Real angle;
    t.orientation.toAxisAngle(axis, angle);
    if (std::abs(angle) > 1e-9) {
        ent["orientation"] = {
            {"axis", vec3ToJson(axis)},
            {"angleDeg", radiansToDegrees(angle)},
        };
    }

    if (material) ent["material"] = materialToJson(*material);

    if (spec.hasPhysics) {
        json phys;
        phys["motion"] = spec.motion;
        phys["friction"] = spec.friction;
        phys["restitution"] = spec.restitution;
        if (spec.lockRotation) phys["lockRotation"] = true;
        ent["physics"] = phys;
    }
    return ent;
}

bool LevelWriter::save(const std::string& path, World& world) {
    // Start from the existing document so sections the editor doesn't edit
    // (environment, lighting, player, version) survive untouched.
    json root;
    {
        std::ifstream existing(path);
        if (existing.is_open()) {
            try {
                root = json::parse(existing);
            } catch (const json::exception& e) {
                LOG_ERROR << "Refusing to overwrite malformed level " << path
                          << " (" << e.what() << ") — fix or remove it first";
                return false;
            }
        } else {
            root["version"] = 1;
        }
    }

    json entities = json::array();
    world.each<Transform, SourceSpec>(
        [&](Entity e, Transform& t, SourceSpec& spec) {
            Renderable* r = world.get<Renderable>(e);
            entities.push_back(
                entityToJson(t, spec, r ? &r->material : nullptr));
        });
    root["entities"] = entities;

    std::ofstream file(path);
    if (!file.is_open()) {
        LOG_ERROR << "Cannot write level: " << path;
        return false;
    }
    file << root.dump(2) << "\n";
    LOG_INFO << "Saved " << entities.size() << " entit"
             << (entities.size() == 1 ? "y" : "ies") << " to " << path;
    return true;
}

}  // namespace engine
