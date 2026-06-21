#include "level_writer.h"

#include "components.h"
#include "property_json.h"
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

// The property layer is the format: every described material field, keyed by
// its FieldMeta id — the same walk the inspectors use, so UI and persistence
// cannot drift. (LevelLoader::parseMaterial is the JsonReadVisitor mirror.)
static json materialToJson(RenderMaterial& m) {
    json mat;
    JsonWriteVisitor writer(mat);
    describeProperties(m, writer);
    // The procedural surface isn't a described property (it's packed in flags);
    // write it explicitly so an inline textured material round-trips (ADR-0039).
    if (m.surface() != RenderMaterial::Surface::None)
        mat["surface"] = surfaceName(m.surface());
    return mat;
}

static json entityToJson(const Transform& t, const SourceSpec& spec,
                         RenderMaterial* material, json& materialsTable) {
    json ent;
    if (spec.id != 0) ent["id"] = spec.id;
    if (spec.parentId != 0) ent["parent"] = spec.parentId;
    if (!spec.name.empty()) ent["name"] = spec.name;
    const bool isRecipe = !spec.recipe.empty();
    if (spec.isGroup()) {
        ent["group"] = true;   // null object: a named transform, no mesh
    } else if (!spec.meshFile.empty()) {
        ent["mesh"] = spec.meshFile;
    } else if (isRecipe) {
        // A procedural recipe entity (e.g. shape:"tree"): emit the shape and the
        // verbatim recipe block (keyed by the shape name, as the loader reads
        // it). The generator owns size and color, so neither is written here.
        ent["shape"] = spec.shape;
        json recipe = json::parse(spec.recipe);
        if (spec.shape == "script") {
            // A Lua recipe entity (ADR-0042) reads its params (file/seed) from
            // the entity's top level, not a nested block — emit them flat so the
            // round-trip reproduces the original `{shape:"script", file, seed}`.
            for (auto it = recipe.begin(); it != recipe.end(); ++it)
                ent[it.key()] = it.value();
        } else {
            ent[spec.shape] = recipe;
        }
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

    if (material && !isRecipe) {
        if (!spec.materialName.empty()) {
            // Shared material asset: reference it by name and register the
            // definition once in the top-level table (ADR-0039).
            ent["material"] = spec.materialName;
            if (!materialsTable.contains(spec.materialName))
                materialsTable[spec.materialName] = materialToJson(*material);
        } else {
            ent["material"] = materialToJson(*material);   // inline material
        }
    }

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

    // Every saved entity gets a stable id (parenting references them); a
    // hand-authored level with none picks them up here, once.
    assignMissingDocumentIds(world);

    json entities = json::array();
    // Preserve the document's existing named-material table: its definitions
    // carry authoring-only fields the runtime RenderMaterial drops once it's
    // resolved — most importantly "surface", which binding the baked texture set
    // clears. Reconstructing from runtime state here would strip it, so a Play
    // (save-then-reload) would lose the textures (ADR-0039). We keep originals
    // and only synthesize entries for references not already defined.
    json materials = (root.contains("materials") && root["materials"].is_object())
                         ? root["materials"]
                         : json::object();
    world.each<Transform, SourceSpec>(
        [&](Entity e, Transform& t, SourceSpec& spec) {
            Renderable* r = world.get<Renderable>(e);
            entities.push_back(
                entityToJson(t, spec, r ? &r->material : nullptr, materials));
        });
    root["entities"] = entities;
    if (!materials.empty()) root["materials"] = materials;
    else root.erase("materials");

    // The editor's PlayerSpawn entity writes back into the "player" block the
    // game loader reads — moving the green capsule moves where you start.
    world.each<Transform, PlayerSpawn>([&](Entity, Transform& t, PlayerSpawn&) {
        root["player"]["position"] = vec3ToJson(t.position);
    });

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
