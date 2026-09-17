#include "level_loader.h"

#include "../profile.h"
#include "level_params.h"   // shared level-JSON -> params readers (both loaders)
#include "script_assets.h"
#include "lot_grow_setup.h"   // the lot pass's parameters from a level: one derivation for loader and bake
#include "procgen/city/lot_cache.h"   // lots read back from a level bundle (ADR-0084 B)
#ifdef RT_ENABLE_SCRIPTING
#include "scripting/script_modules.h"
#endif
#include <cstdlib>
#include <tinygltf/stb_image_write.h>   // the PNG writer the elevation maps use
                                        // (implementation lives in model_importer.cpp)
#include "procgen/city/road_rules.h"    // DesignRules: the per-class grade table the
                                        // poke report hands weldChainProfiles
#include "procgen/earthwork.h"          // the earthwork displacement field
#include "mesh_builder.h"
#include "asset_manager.h"
#include "procgen/terrain.h"
#include "procgen/city/city_lots.h"  // grow buildings on the road net's blocks (ADR-0066)
#include "procgen/city/building_collider.h"  // prism + door notches (ADR-0080)
#include "procgen/city/building_records.h"   // CityBuildings runtime records (ADR-0080)
#include "procgen/city/road_net.h"
#include "procgen/city/road_semantics.h"   // editor-authored roads (shape:"road")
#include "procgen/city/citylots_producer.h"   // the lattice city's lot pre-pass as a producer (ADR-0084 C)
#include "bundle/bake.h"
#ifdef RT_ENABLE_LANELAB
#include "procgen/lanelab/lanelab.h"     // lane-atomic road lab (ADR-0083, opt-in hook)
#include "procgen/lanelab/deck_mesh.h"
#include "procgen/lanelab/road_twin.h"   // the derived road graph: nav, furniture, map
#include "procgen/lanelab/block_audit.h"  // sceneBlocks: city blocks straight from the pavement
#include "procgen/lanelab/city_producer.h" // the city as a bundle producer (ADR-0084)
#include "procgen/lanelab/lots_producer.h" // the lot pass as the second producer (milestone B)
#include "bundle/bake.h"
#endif
#include "procgen/city/corridor_bake.h"
#include "procgen/city/corridor_plan.h"   // S3b: bake solved corridors into the net
#include "procgen/city/block_grade.h" // grade blocks to their streets (ADR-0075 P2)
#include "procgen/city/road_network.h" // extractBlocks (block grading)
#include "procgen/city/city_svg.h"
#include "procgen/city/architect.h"   // districtName (city map hub labels)
#include "procgen/city/district.h"   // generated road districts (shape:"road" with "generate")
#include "procgen/city/road_constraints.h"   // applyConstraints — bake roundabouts into the graph
#include "procgen/city/road_mesh.h"   // triangulatePolygon (building prism colliders)
#include "procgen/city/corridor_mesh.h"      // freeway corridors (plan §8)
#include "procgen/city/road_lattice.h"       // swept-lattice freeway mesher
#include "procgen/city/street_furniture.h"   // build-time signal/lamp placement
#include "procgen/city/street_kit.h"  // trafficSignalProto, streetLamp
#include "procgen/city/water_mesh.h"  // buildWaterMesh (ocean/lake surface)
#include "ai/nav_graph.h"             // buildNavGraph (street furniture plan)
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
#include "vehicle_lamps.h"   // beaconCellPhase: the flash every beacon tier shares
#include "property_json.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <unordered_map>
#include <array>
#include <iterator>
#include <tuple>
#include <utility>
#include <vector>

using json = nlohmann::json;

namespace engine {

// Files the current load has read. File-static rather than threaded through
// every helper: loading is single-threaded and the alternative is an extra
// out-param on a dozen call sites.
static std::vector<std::string> g_loadedScriptFiles;


// parseVec3/parseOrientation and propagateWaterSeaLevel come from
// level_params.h — one definition for both this loader and the offline
// importer (their local copies had already drifted: exact-3 vs >=3 arrays).

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

// Surfaces whose geometry bakes its OWN meaningful UVs in the generator —
// FanTop (centred disc), VentGrille (plate-fitted with a margin), RoofShingle
// (slope-fitted: u along the eave, v up the slope). A world-planar re-UV would
// break exactly what those parameterizations encode.
static bool surfaceBakesOwnUVs(RenderMaterial::Surface s) {
    return s == RenderMaterial::Surface::FanTop ||
           s == RenderMaterial::Surface::VentGrille ||
           s == RenderMaterial::Surface::RoofShingle;
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
    t.orientation = parseOrientation(ent);
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

// An editor-authored road (shape:"road", ADR-0049): a RoadEntity (control nodes +
// look) baked to a carriageway mesh and carried as a first-class DOCUMENT entity,
// so the inspector can widen it and the viewport can drag its nodes — the editor
// regenerates the mesh through onEdited. Drapes on the level terrain (`ground`).
// Split a WORLD-SPACE mesh into grid-cell chunks by triangle centroid, so the
// per-Renderable frustum/AABB cull (render_system) drops city blocks instead of
// treating a district-wide merged mesh as one always-visible draw (plan
// metropolis-scale P1.1/P1.3). Vertices are duplicated per chunk (cheap: a
// vertex is shared by few triangles); materialIndex carries over.
// Aviation-beacon chunks are cut this fine so a roof's lamps flash on their own phase (skyscrapers v2 M4).
static constexpr double kBeaconChunk = 24.0;
// The ROOM ATLAS for interior-mapped panes (skyscrapers v2, the faked tier): four
// rooms in a 2x2 grid of 256 px tiles, each tile a 2x2 of 128 px faces — back
// wall (0,0), ceiling (128,0), floor (0,128), side wall (128,128) — in the
// order mesh.frag's FLAG_INTERIOR_MAP samples them. Two offices, two flats:
// a bright ceiling fixture, dimmer walls, a dark floor, a desk or shelf band
// on the back wall, everything falling off with depth so the room reads deep.
// Values are LIGHT, not albedo: they multiply the pane's night emission.
static TextureHandle bakeRoomAtlas(Renderer& renderer) {
    // 16 rooms in a 4x4 grid of 256 px tiles (offices in rows 0-1, flats in
    // rows 2-3), each tile a 2x2 of 128 px faces: back wall (0,0), ceiling
    // (128,0), floor (0,128), side wall (128,128) — the order mesh.frag's
    // FLAG_INTERIOR_MAP samples them. Furniture is blobs: desks with a monitor
    // glow, shelves, a sofa, a rug, ceiling strips or a pendant, a door on the
    // side wall — everything falling off with depth. Values are LIGHT (they
    // multiply the pane's night emission), not albedo.
    const int n = 1024, tile = 256, face = 128;
    std::vector<unsigned char> img(static_cast<std::size_t>(n) * n * 4, 255);
    auto put = [&](int x, int y, const Vec3& c) {
        const std::size_t o = (static_cast<std::size_t>(y) * n + x) * 4;
        img[o] = static_cast<unsigned char>(std::min(1.0, std::max(0.0, c.x)) * 255);
        img[o + 1] = static_cast<unsigned char>(std::min(1.0, std::max(0.0, c.y)) * 255);
        img[o + 2] = static_cast<unsigned char>(std::min(1.0, std::max(0.0, c.z)) * 255);
    };
    uint32_t h = 9176u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffu) / 65535.0; };
    for (int r = 0; r < 16; ++r) {
        const bool office = r < 8;
        const int tx = (r % 4) * tile, ty = (r / 4) * tile;
        // The palette, varied per room.
        const Real wl = office ? 0.60 + 0.12 * rnd() : 0.56 + 0.14 * rnd();
        const Vec3 wall = office ? Vec3(wl, wl, wl * 0.98) : Vec3(wl, wl * 0.93, wl * 0.82);
        const Vec3 fix = office ? Vec3(1.0, 1.0, 1.0) : Vec3(1.0, 0.95, 0.86);
        const Vec3 floorC = office ? Vec3(0.22, 0.24, 0.30) * (0.8 + 0.4 * rnd()) : Vec3(0.40, 0.30, 0.21) * (0.7 + 0.5 * rnd());
        const Vec3 dark(0.12, 0.11, 0.10);
        // Furniture placement (in face UV).
        const int desks = office ? 2 + static_cast<int>(rnd() * 2.0) : 0;
        const Real deskX[3] = {0.08 + rnd() * 0.1, 0.42 + rnd() * 0.1, 0.72 + rnd() * 0.1};
        const bool shelves = !office && rnd() < 0.6;
        const bool sofa = !office && rnd() < 0.7;
        const Real sofaX = 0.15 + rnd() * 0.4;
        const bool tv = !office && rnd() < 0.5;
        const bool pendant = !office;
        const Real doorU = 0.55 + rnd() * 0.3;
        for (int y = 0; y < face; ++y)
            for (int x = 0; x < face; ++x) {
                const Real u = (x + 0.5) / face, v = (y + 0.5) / face;
                // BACK WALL (u across, v down).
                {
                    Vec3 c = wall * 0.60;
                    if (office) {
                        // A dado band, desks against it with a monitor each.
                        if (v > 0.62) c = wall * 0.48;
                        for (int k = 0; k < desks; ++k) {
                            const Real dx = deskX[k];
                            if (u > dx && u < dx + 0.2 && v > 0.58 && v < 0.66) c = dark * 1.6;      // desk top
                            if (u > dx + 0.05 && u < dx + 0.13 && v > 0.44 && v < 0.56) c = Vec3(0.55, 0.65, 0.85);   // monitor
                        }
                        if (u > 0.30 && u < 0.70 && v > 0.14 && v < 0.36) c = wall * 0.90;   // whiteboard
                    } else {
                        if (shelves && u > 0.62 && u < 0.92 && v > 0.12 && v < 0.62) {
                            const Real row = std::fmod(v * 12.0, 1.0);
                            c = row < 0.18 ? dark * 1.8 : wall * 0.35 * (0.7 + 0.6 * std::fmod(u * 31.0, 1.0));
                        }
                        if (sofa && u > sofaX && u < sofaX + 0.32 && v > 0.60 && v < 0.78) c = Vec3(0.30, 0.22, 0.20);
                        if (tv && u > 0.18 && u < 0.46 && v > 0.30 && v < 0.48) c = Vec3(0.35, 0.45, 0.75);
                        if (!tv && u > 0.20 && u < 0.44 && v > 0.16 && v < 0.40) c = wall * 0.40;   // a picture
                    }
                    put(tx + x, ty + y, c);
                }
                // CEILING (u across, v = depth in): strips or a pendant, darker deeper.
                {
                    const Real fall = 1.0 - 0.45 * v;
                    Vec3 c = wall * 0.85 * fall;
                    if (office) {
                        if (v > 0.22 && v < 0.34 && u > 0.10 && u < 0.90) c = fix;
                        if (v > 0.62 && v < 0.74 && u > 0.10 && u < 0.90) c = fix * 0.9;
                    } else if (pendant) {
                        const Real du = u - 0.5, dv = v - 0.45;
                        if (du * du + dv * dv < 0.02) c = fix;
                        else if (du * du + dv * dv < 0.06) c = wall * 1.05 * fall;
                    }
                    put(tx + face + x, ty + y, c);
                }
                // FLOOR (u across, v = depth in): a rug or desk shadows, darker deeper.
                {
                    const Real fall = 1.0 - 0.5 * v;
                    Vec3 c = floorC * fall;
                    if (office) {
                        for (int k = 0; k < desks; ++k)
                            if (u > deskX[k] && u < deskX[k] + 0.2 && v > 0.70) c = floorC * 0.45 * fall;
                    } else if (u > 0.25 && u < 0.75 && v > 0.30 && v < 0.75) {
                        c = Vec3(0.34, 0.26, 0.30) * fall;   // a rug
                    }
                    put(tx + x, ty + face + y, c);
                }
                // SIDE WALL (u = depth in, v down): a door deep in, darker deeper.
                {
                    const Real fall = 1.0 - 0.5 * u;
                    Vec3 c = wall * 0.72 * fall;
                    if (u > doorU && u < doorU + 0.22 && v > 0.22) c = Vec3(0.28, 0.22, 0.18) * fall;
                    put(tx + face + x, ty + face + y, c);
                }
            }
    }
    return renderer.uploadTexture(n, n, 4, img.data());
}

// The distant tier's lit-window map (skyscrapers v2 M4): `cells` x `cells` window
// cells, each an 8 px cell with a 5 x 5 px pane; a third of the panes lit, in the
// lit-glass tint palette (cool office whites, some fluorescent blue-white, warm
// incandescent, cream); the FIRST cell is always dark (the roof cap samples it).
// Alpha is 1 everywhere: only the emission channel reads it.
static TextureHandle bakeLitWindowMap(Renderer& renderer, int cells, uint32_t seed) {
    const int px = 8, n = cells * px;
    std::vector<unsigned char> img(static_cast<std::size_t>(n) * n * 4, 0);
    for (std::size_t i = 3; i < img.size(); i += 4) img[i] = 255;
    uint32_t h = seed;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffu) / 65535.0; };
    static const Vec3 tints[6] = {{1.0, 0.97, 0.92}, {0.92, 0.96, 1.0}, {0.80, 0.90, 1.0},
                                  {1.0, 0.85, 0.62}, {1.0, 0.92, 0.75}, {0.96, 0.98, 0.94}};
    for (int cy = 0; cy < cells; ++cy)
        for (int cx = 0; cx < cells; ++cx) {
            const double u = rnd();
            const bool lit = !(cx == 0 && cy == 0) && u < 0.34;
            if (!lit) continue;
            const Vec3 t = tints[static_cast<int>(rnd() * 5.999)];
            for (int y = 1; y < 6; ++y)
                for (int x = 2; x < 7; ++x) {
                    const std::size_t o = (static_cast<std::size_t>(cy * px + y) * n + (cx * px + x)) * 4;
                    img[o] = static_cast<unsigned char>(t.x * 255);
                    img[o + 1] = static_cast<unsigned char>(t.y * 255);
                    img[o + 2] = static_cast<unsigned char>(t.z * 255);
                }
        }
    return renderer.uploadTexture(n, n, 4, img.data());
}

static std::vector<RenderMesh> chunkMeshByCell(const RenderMesh& m, double cell) {
    std::vector<RenderMesh> out; for (MeshBuilder::CellChunk& c : MeshBuilder::chunkByCell(m, cell)) out.push_back(std::move(c.mesh)); return out;
}

static void loadRoadEntity(const json& ent, World& world, AssetManager& assets,
                           int index, const HeightField& drapeGround,
                           const RoadEntity* preNet = nullptr) {
    const json roadBlock = ent.contains("road") ? ent["road"] : json::object();
    // REUSE the terrain pre-pass net when given: the pre-pass ran the recipe
    // against the NATURAL ground and everything downstream (corridor carve,
    // lot growth, building placement) used THAT graph. Re-running the recipe
    // here against the CARVED ground made the terrain-aware gate diverge and
    // built a DIFFERENT network than the lots respect — buildings mid-road
    // (device: "buildings ... strewn about haphazardly"). One recipe run, one
    // network.
    RoadEntity net = preNet ? *preNet : roadNetFromJson(roadBlock);

    // A GENERATED road: instead of authored nodes, "generate" runs a procgen graph (buildDistrict)
    // and fills the RoadEntity's graph from it — so the generated city IS a real, editable RoadEntity
    // (the editor's node/tangent handles + buildRoadNetMesh's curves/markings/sidewalks/junctions),
    // not a baked mesh. The recipe round-trips via SourceSpec; the look comes from the road block.
    // `drapeGround` is what this road's mesh drapes on. A PRE-PASS net gets the
    // NATURAL sampler from the caller: the terrain conform computed the road's
    // height profile on natural ground, and the mesh must compute the
    // IDENTICAL profile — meshing over the already-carved terrain made the
    // weld's profiles diverge from the conform planes by up to ~1 m at
    // junctions (RT_POKE_SITE autopsy). Fresh hand-authored roads get the
    // carved ground as before.
    if (!preNet && roadBlock.contains("generate"))
        applyGenerateRecipe(net, roadBlock["generate"], drapeGround);

    Entity e = world.create();
    createEntityCommon(e, ent, world);

    SourceSpec spec = buildSourceSpec(ent, "road");
    // Preserve the ORIGINAL authored block (a "generate" recipe, or hand-authored nodes) as the
    // saved form — NOT the baked net — so load→save is a no-op and a generated road keeps its recipe
    // instead of being frozen into geometry. The editor's regenerateRoad re-bakes this only when the
    // road is actually edited (a node drag / width change), which is when baking is correct.
    spec.recipe = roadBlock.dump();
    world.add<SourceSpec>(e, spec);
    world.add<RoadEntity>(e, net);                      // the editable source of truth

    Renderable r;
    r.renderLayer = engine::LayerRoads;              // debug layer toggle
    r.material.albedo = Vec3(1, 1, 1);               // hue carried in vertex colour
    r.material.roughness = 0.93f;
    if (net.look.markings)                           // lane paint via the surface shader
        r.material.setSurface(RenderMaterial::Surface::RoadMarkings);
    RoadDeckField deck;
    // The mesher's own curb band outlines ride along for the city map: the
    // sidewalks drawn are the sidewalks built (a few thousand points).
    engine::CurbBandAudit bandAudit;
    RenderMesh mesh = buildRoadNetMesh(net, drapeGround, &bandAudit, &deck);
    if (!bandAudit.loops.empty()) {
        engine::RoadBandDebug band;
        band.loops = std::move(bandAudit.loops);
        band.mouthGaps = std::move(bandAudit.mouthGaps);
        band.sidewalkWidth = bandAudit.sidewalkWidth > 0 ? bandAudit.sidewalkWidth : net.look.sidewalk;
        world.add<engine::RoadBandDebug>(e, std::move(band));
    }
    if (!mesh.vertices.empty())
        r.mesh = assets.acquireMesh(mesh, "road:" + std::to_string(index));
    world.add<Renderable>(e, r);
    // The deck the mesh rode, for everything that must stand ON it (RoadDeck).
    if (!deck.empty()) world.add<RoadDeck>(e, RoadDeck{std::move(deck)});

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

#ifdef RT_ENABLE_LANELAB
// Lane-atomic road lab (ADR-0083, behind RT_ENABLE_LANELAB): shape:"lanelab" builds
// the lab's graph AT LOAD and spawns one entity per material mesh, each with a static
// MeshCollider from the SAME triangles (Playable Scenes rule) — the player drives the
// decks, ramps, piers and the conformed terrain it sees. Paint strips are visual only
// (a 2 mm lip reads as a step under a wheel). The authored block round-trips as a
// DOCUMENT entity like the corridor: {"lanelab": {"graph": "<path>"}} or the inline
// graph spec itself (recognised by its "edges").
// The lab's conformed terrain, published for the lot pass and the building pads when
// the level has no terrain of its own (a lab level: the lanelab grid IS the ground).
// One struct so a level load resets every member at once (a second level used to inherit the first lab's
// blocks, right-of-way and ground). Filled from the level's city bundle (ADR-0084): built now or read back.
struct LaneLabPublished {
    HeightField ground;                    // the lab's conformed terrain (a lab level's ground)
    engine::RoadGraph row;                 // freeway + ramp edges of the class-faithful twin, for the lot pass's keep-out
    double sidewalk = 4.0;                 // the citysim sidewalk, read before entities load
    std::vector<engine::Poly2> blocks;     // the lab's city blocks: the pavement's holes, inset by the sidewalk
    int ordinal = 0;                       // lanelab entities seen in this load: the bundle section namespace
    engine::bundle::LevelInputs inputs;    // the level, for the producers' keys
    std::shared_ptr<engine::bundle::Bundle> bundle;   // the level's city bundle, obtained on the first lanelab entity
    std::string bundleStatus;
};
static LaneLabPublished g_lanelab;

// Which analytic surface dresses each of the lab's material meshes. Names come from the bundle
// (city_producer splits a cell's mesh per material), so this is the one place the two vocabularies meet.
static RenderMaterial::Surface surfaceForLaneLabMaterial(const std::string& name) {
    using S = RenderMaterial::Surface;
    if (name == "asphalt" || name == "shoulder") return S::Asphalt;      // carriageway and hard shoulder
    if (name == "concrete") return S::Concrete;                          // parapets, piers, girders, slab sides
    if (name == "sidewalk") return S::Pavement;                          // scored concrete flags
    if (name == "terrain") return S::TerrainGround;                      // the lab's ground: micro-relief over the baked grass colour
    if (name == "guardrail") return S::CorrugatedMetal;                  // a W-beam is corrugated; the loader keeps it dull
    return S::None;                                                      // median grass slabs, paint_white, paint_yellow
}

static void loadLaneLabEntity(const json& ent, World& world, AssetManager& assets,
                              int index) {
    using namespace engine::lanelab;
    const json block = ent.contains("lanelab") ? ent["lanelab"] : json::object();
    spawnDocumentEntity(ent, "lanelab", block.dump(), world);
    const int ordinal = g_lanelab.ordinal++;
    const auto t0 = std::chrono::steady_clock::now();
    auto since = [](const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); };
    // ONE derivation (ADR-0084): the city's products come out of the level's bundle whether it was on disk
    // or built a moment ago — never straight out of the builder — so a cold level and a cached one are the
    // same geometry to the byte. The bundle is obtained once per load, on the first lanelab entity.
    if (!g_lanelab.bundle) {
        registerCityProducer(); registerLotsProducer();   // both before the first obtain: the bundle directory is named by every producer that applies
        engine::bundle::Obtained o = engine::bundle::obtainForLevel(g_lanelab.inputs, kCityProducerName);
        g_lanelab.bundleStatus = o.status;
        if (!o.bundle) { LOG_ERROR << "[lanelab] no city products for this level: " << o.status; return; }
        g_lanelab.bundle = o.bundle;
        LOG_INFO << "[lanelab] bundle " << o.status;
    }
    CityProducts p; std::string err;
    if (!readCityProducts(*g_lanelab.bundle, ordinal, p, &err)) { LOG_ERROR << "[lanelab] " << err; return; }
    const double tRead = since(t0);
    if (p.hasTerrain) {
        auto grid = std::make_shared<HeightGrid>(); grid->x0 = p.ground.x0; grid->y0 = p.ground.y0; grid->res = p.ground.res; grid->nx = p.ground.nx; grid->ny = p.ground.ny; grid->z = p.ground.z;
        g_lanelab.ground = [grid](double x, double z) { return grid->sample(x, z); };
    }
    {   // The CLASS-FAITHFUL twin (freeway/ramp classes intact) is the level's unified road graph — nav,
        // furniture, map — and its freeway right-of-way is the lot pass's keep-out. The twin is DERIVED: the
        // lab's pavement is the source of truth, a city block is a hole in it (Glenn, 2026-09-05).
        bool have = false; world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph&) { have = true; });
        if (!have) {
            engine::LevelRoadGraph lrg; lrg.graph = p.nav;
            LOG_INFO << "[lanelab] unified road graph from the class-faithful twin: " << lrg.graph.nodes.size() << " nodes, " << lrg.graph.edges.size() << " edges";
            world.add<engine::LevelRoadGraph>(world.create(), std::move(lrg));
        }
        g_lanelab.row = p.row;
        g_lanelab.blocks = blocksFromHoles(p.holes, 1.5, g_lanelab.sidewalk);   // pre-inset by the sidewalk (robust); the lot pass gets roadMargin 0
        LOG_INFO << "[lanelab] " << g_lanelab.blocks.size() << " city blocks from " << p.holes.size() << " pavement holes";
        if (const char* twinSvg = std::getenv("RT_LANELAB_TWIN_SVG")) {   // the twin as lines and junction dots
            const engine::RoadEntity& twin = p.twin; std::vector<int> deg(twin.graph.nodes.size(), 0);
            for (const engine::RoadEdge& e : twin.graph.edges) { ++deg[static_cast<size_t>(e.a)]; ++deg[static_cast<size_t>(e.b)]; }
            double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
            for (const engine::RoadNode& n : twin.graph.nodes) { x0 = std::min(x0, n.pos.x); y0 = std::min(y0, n.pos.y); x1 = std::max(x1, n.pos.x); y1 = std::max(y1, n.pos.y); }
            std::ofstream f(twinSvg);
            f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << x0 - 20 << " " << y0 - 20 << " " << (x1 - x0) + 40 << " " << (y1 - y0) + 40 << "'>\n";
            f << "<rect x='" << x0 - 20 << "' y='" << y0 - 20 << "' width='" << (x1 - x0) + 40 << "' height='" << (y1 - y0) + 40 << "' fill='#f4f4ee'/>\n";
            for (const engine::RoadEdge& e : twin.graph.edges) {
                const char* col = e.klass == engine::RoadClass::Freeway ? "#c33" : e.klass == engine::RoadClass::Ramp ? "#e80" : "#335";
                f << "<line x1='" << twin.graph.nodes[static_cast<size_t>(e.a)].pos.x << "' y1='" << twin.graph.nodes[static_cast<size_t>(e.a)].pos.y << "' x2='" << twin.graph.nodes[static_cast<size_t>(e.b)].pos.x << "' y2='" << twin.graph.nodes[static_cast<size_t>(e.b)].pos.y << "' stroke='" << col << "' stroke-width='1.5'/>\n";
            }
            for (size_t i = 0; i < deg.size(); ++i) if (deg[i] != 2) f << "<circle cx='" << twin.graph.nodes[i].pos.x << "' cy='" << twin.graph.nodes[i].pos.y << "' r='" << (deg[i] == 1 ? 2.5 : 3.5) << "' fill='" << (deg[i] == 1 ? "#c0392b" : "#1f4e9c") << "'/>\n";
            f << "</svg>\n";
        }
    }
    // One Renderable per (render cell, material) — the granularity the building chunks use — with a static
    // MeshCollider from the SAME triangles for everything but paint (Playable Scenes rule; a 2 mm paint lip
    // reads as a step under a wheel). Unpack one cell mesh at a time: the packed products stay float32.
    const auto t1 = std::chrono::steady_clock::now(); int collidable = 0; size_t tris = 0;
    for (const CityCellMesh& c : p.cells) {
        const engine::bundle::PackedMesh& pm = c.mesh; if (pm.vertexCount() == 0 || pm.idx.empty()) continue;
        // When the level carries a terrain block the ground is CDLOD's, built from the same conformed grid:
        // drawing the lab's own flat copy on top of it would z-fight and cost 400k triangles.
        if (pm.name == "terrain" && g_lanelab.inputs.level.contains("terrain")) continue;
        Entity e = world.create();
        createEntityCommon(e, ent, world);
        Renderable r;
        r.renderLayer = engine::LayerRoads;
        r.material.albedo = Vec3(pm.albedo[0], pm.albedo[1], pm.albedo[2]);
        r.material.roughness = pm.roughness;
        // The lab's roads were flat colour: it never asked for a surface, so nothing textured them. These are
        // ANALYTIC surfaces (renderer.h) — grain computed in the shader from the world-planar UV, no maps to
        // bake and nothing to author, the same ones the engine's own roads and plazas use. Paint strips keep
        // Surface::None: they are their own flat colour lying on the deck (Glenn, 2026-09-08).
        r.material.setSurface(surfaceForLaneLabMaterial(pm.name));
        if (pm.name == "guardrail") r.material.metallic = 0.18f;         // weathered galvanising: a sheen, not a mirror
        { const RenderMesh m = engine::bundle::unpackMesh(pm); r.mesh = assets.acquireMesh(m, "lanelab:" + std::to_string(index) + ":" + pm.name + ":" + std::to_string(c.cx) + "_" + std::to_string(c.cz)); }
        world.add<Renderable>(e, r); tris += pm.triangleCount();
        if (pm.paint() || !pm.collidable()) continue;
        MeshCollider mc; engine::bundle::colliderFromPacked(pm, mc); world.add<MeshCollider>(e, mc); ++collidable;
    }
    LOG_INFO << "[lanelab] e" << ordinal << ": " << p.cells.size() << " cell meshes, " << tris << " triangles, " << collidable << " collidable; products read in " << tRead << " s, instantiated in " << since(t1) << " s";
}
#endif

// A hero parametric tree (shape: "tree"): a real, collidable object you can
// bounce off or shoot at, distinct from the instanced vegetation scatter. The
// bark is one mesh (procedural bark texture, opaque) with a static triangle
// MeshCollider; the leaves are a second entity at the same transform (alpha-cut
// cards, no collision). docs/lsystem-botany-plan.md.
static void loadTreeEntity(const json& ent, World& world, Renderer& renderer,
                           AssetManager& assets, int index) {
    uint32_t seed = 0;
    TreeParams tp = readTreeParams(ent, seed);

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
    if (const std::string path = resolveScriptPath(file, levelDir); !path.empty())
        g_loadedScriptFiles.push_back(path);
    // A FRESH VM per script entity, deliberately. Every shipped level has one
    // such entity, so the module re-parse is once per load; and the shared-VM
    // alternative leaks `args`/`ground` from one entity into the next (see
    // level_scene.cpp's ensureVm, which sets them only when present).
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(levelDir, &g_loadedScriptFiles));
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

    // Optional world placement: recipe geometry is authored around the origin, so
    // an entity `position` [x,y,z] offsets the whole spawned model. A planet recipe
    // uses it to sit the body in front of the scene-view camera (which starts at a
    // fixed eye) instead of enclosing it. Parts are the render path; recipes that
    // also emit instances/colliders would need the same offset (none do today).
    Vec3 spawnOffset = ent.contains("position") ? parseVec3(ent["position"]) : Vec3(0, 0, 0);

    for (std::size_t i = 0; i < model.parts.size(); ++i) {
        const ProcPart& part = model.parts[i];
        if (part.mesh.vertices.empty()) continue;
        Entity e = world.create();
        Transform t;                          // recipe geometry is world-space
        t.position = spawnOffset;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.material.albedo = Vec3(1, 1, 1);    // hue carried in vertex colour
        r.material.metallic = part.material.metallic;
        r.material.roughness = part.material.roughness;
        r.material.opacity = part.material.opacity;
        r.material.emission = part.material.emission;
        if (part.material.twoSided)
            r.material.flags |= RenderMaterial::FLAG_TWO_SIDED;
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
        // Optional axial spin (procedural-planet-plan): an `angularVelocity`
        // [x,y,z] (axis · rad/s) on the entity makes MotionSystem rotate each part
        // about its origin. A planet recipe's parts are world-space centred on the
        // origin, so this spins the body on its axis. No RigidBody, so MotionSystem
        // (not physics) integrates it.
        if (ent.contains("angularVelocity")) {
            Vec3 av = parseVec3(ent["angularVelocity"]);
            if (av.lengthSquared() > 0.0) world.add<Velocity>(e, Velocity{Vec3(), av});
        }
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
                         const std::vector<std::pair<const json*, ProcModel>>*
                             scriptCache = nullptr,
                         const HeightField* ground = nullptr,
                         const std::vector<std::pair<const json*, RoadEntity>>*
                             roadCache = nullptr,
                         const HeightField* naturalGround = nullptr) {
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
        // Freeway corridor (plan §8): the geometry was built + spawned by the
        // loader's terrain pre-pass (its flatten windows must join the carve
        // set). Here it only needs its DOCUMENT entity — the SourceSpec
        // carrying the corridor block — so the editor's save-then-reload
        // (Play) round-trips it instead of dropping the freeway on the floor
        // (device: "the freeway disappeared when the simulation started").
        if (ent.value("shape", std::string()) == "corridor") {
            spawnDocumentEntity(
                ent, "corridor",
                (ent.contains("corridor") ? ent["corridor"] : json::object())
                    .dump(),
                world);
            continue;
        }
        // Editor-authored road (ADR-0049): an editable RoadEntity + its baked mesh.
        if (ent.value("shape", std::string()) == "road") {
            const RoadEntity* pre = nullptr;
            if (roadCache != nullptr)
                for (const auto& p : *roadCache)
                    if (p.first == &ent) { pre = &p.second; break; }
            // A pre-pass road meshes over the NATURAL ground (the profile the
            // conform carved to); a fresh road drapes on the carved terrain.
            static const HeightField kFlat;
            const HeightField& drape =
                pre ? (naturalGround ? *naturalGround : kFlat)
                    : (ground ? *ground : kFlat);
            loadRoadEntity(ent, world, assets, roadIndex++, drape, pre);
            continue;
        }
#ifdef RT_ENABLE_LANELAB
        // Lane-atomic road lab (ADR-0083): opt-in, drivable, apart from the road mesher.
        if (ent.value("shape", std::string()) == "lanelab") {
            loadLaneLabEntity(ent, world, assets, roadIndex++);
            continue;
        }
#endif
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
    // 0.55 default (roads-v2.1 R4): belt-and-braces with block grading — a
    // curb+lift step tops ~0.5 on slopes; the player must always mount a
    // sidewalk (drive feedback: "player can't walk up onto a sidewalk").
    cc.stepHeight = player.value("stepHeight", 0.55);

    // CDLOD terrain: snap the spawn to just above the surface at its XZ so the
    // character settles onto the ground rather than spawning embedded (below the
    // surface) or falling from far above. Scoped to CDLOD (static-chunk levels
    // have no TerrainLodConfig and keep the authored y).
    const TerrainLodConfig* tc = nullptr;
    world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& cfg) { if (!tc) tc = &cfg; });
    if (tc && !std::getenv("RT_SPAWN")) {   // RT_SPAWN is taken verbatim (a floor of a tower, say)
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
        // Static-sun truth for the dusk predicates: a level authored at dusk
        // must read as dusk. The day/night cycle overwrites this per frame
        // when enabled (and may hang the MOON in slot 0, which is why the
        // predicates read this and never sun.direction).
        l.solarElevation = static_cast<float>(l.sun.direction.y);
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

// (parseTerrainParams moved to level_params.cpp as readTerrainParams — the
// ONE parse both this loader and the offline importer use.)

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
    {
        // Height sanity sweep of the FINAL params (flattens folded) — the same
        // field every CDLOD node samples. Catches a garbage flatten plane or a
        // broken eroded base before it renders as mystery geometry.
        Noise sn(cfg.seed);
        const double half = t.contains("cdlod") && t["cdlod"].is_object()
                                ? t["cdlod"].value("worldHalf", 1024.0)
                                : 1024.0;
        double mn = 1e30, mx = -1e30;
        for (int j = -4; j <= 4; ++j)
            for (int i = -4; i <= 4; ++i) {
                double h = terrainHeight(p, sn, half * i / 4.0, half * j / 4.0);
                mn = std::min(mn, h);
                mx = std::max(mx, h);
            }
        LOG_INFO << "[terrain] final field 9x9 sweep: h [" << mn << ", " << mx
                 << "] over half-extent " << half;
    }
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
                           const std::string& tag = "veg",
                           const std::vector<engine::LotBuilding>* lots = nullptr,
                           double placeDilate = 0.0) {
    RT_PROFILE_ZONE_NAMED("loadVegetation");
    if (!veg.contains("species") || !veg["species"].is_array()) return;

    // A species variant is now a multi-part model (ADR-0032): each part is one
    // {mesh, material} drawn as its own InstanceGroup over the shared transforms,
    // so bark (opaque) and leaves (alpha-cut) — or any N parts — scatter together.
    struct Part { MeshHandle mesh; RenderMaterial material; };
    struct Variant {
        std::vector<Part> parts;
        bool isRock = false;         // rocks get squash/scale variance + deeper bedding
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
            openModuleLoader(*scriptVm, makeModuleSource(levelDir, &g_loadedScriptFiles));
            // Was a bare ifstream on "assets/scripts/flora.lua", which is why a
            // level-relative flora.lua silently never worked. The shared resolver
            // honours the same candidates as every other script.
            const std::string src = loadScriptCode("flora.lua", levelDir);
            if (!src.empty()) {
                if (const std::string path = resolveScriptPath("flora.lua", levelDir);
                    !path.empty())
                    g_loadedScriptFiles.push_back(path);
                std::string err;
                if (!scriptVm->doString(src, &err))
                    LOG_ERROR << "flora.lua load failed: " << err;
            } else {
                LOG_WARN << "flora.lua not found; `flora.*` unavailable";
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
        rsp.faceted    = s.value("faceted", rsp.faceted);
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
            if (!var.parts.empty()) {
                var.isRock = (kind == "rock");
                variantList.push_back(std::move(var));
            }
        }
        speciesIndex++;
    }
    if (variantList.empty()) return;

    ScatterParams scatter;
    scatter.placeDilate      = placeDilate;   // sample the mesh's own surface
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

    // PARK PLANTING: city park/green lots get their own trees — the open-world
    // scatter above can't be relied on to land inside a small urban park. Each
    // pad plants by area (parkTreeDensity per m^2), on the pad's own top plane.
    if (lots && veg.value("parkTrees", true)) {
        const double density = veg.value("parkTreeDensity", 0.012);   // ~1 / 85 m^2
        std::mt19937 prng(vegSeed ^ 0x51ed270bu);
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        for (const engine::LotBuilding& lb : *lots) {
            if (lb.pad.size() < 3 || (lb.type != "park" && lb.type != "green")) continue;
            // Sculpted parks planted their own treeSpots (terrain-sampled,
            // spaced against paths/furniture) — planting on top of those both
            // double-planted and, worse, used the pad plane's groundY, which
            // parks never set: every one of these trees spawned metres UNDER
            // the terrain (device: "trees are below the terrain").
            if (!lb.treeSpots.empty()) continue;
            const double a = engine::area(lb.pad);
            int want = std::min(6, static_cast<int>(a * density));
            if (want < 1) continue;
            // Pad bounding box for rejection sampling.
            double mnx = 1e30, mnz = 1e30, mxx = -1e30, mxz = -1e30;
            for (const engine::Vec2& v : lb.pad) {
                mnx = std::min(mnx, (double)v.x); mxx = std::max(mxx, (double)v.x);
                mnz = std::min(mnz, (double)v.y); mxz = std::max(mxz, (double)v.y);
            }
            for (int k = 0, placed = 0; k < want * 12 && placed < want; ++k) {
                engine::Vec2 q(mnx + uni(prng) * (mxx - mnx),
                               mnz + uni(prng) * (mxz - mnz));
                if (!engine::pointInPolygon(lb.pad, q)) continue;
                Placement pl;
                pl.position = Vec3(
                    q.x,
                    terrainHeight(terrain, terrainNoise, q.x, q.y, placeDilate) + 0.1,
                    q.y);   // the ground under its OWN feet (mesh-matched dilate)
                pl.yaw = static_cast<float>(uni(prng) * 6.2831853);
                pl.scale = static_cast<float>(0.55 + 0.35 * uni(prng));
                placements.push_back(pl);
                ++placed;
            }
        }
    }
    LOG_INFO << "[veg] " << tag << ": " << placements.size() << " placements"
             << " (count=" << scatter.count << ", region=" << scatter.regionSize
             << ", maxSlopeDeg=" << scatter.maxSlopeDeg << ")";

    // One InstanceGroup per (variant, part, GRID CELL): a variant's parts share
    // the same per-instance transforms (ROADMAP Phase B instancing), and the
    // cell split gives every group LOCAL bounds — frustum and draw-distance
    // culling then drop whole forest chunks instead of treating the entire
    // map's scatter as one huge always-visible group (device: 2 fps whenever
    // trees were on screen). Plants carry no SourceSpec (ADR-0022).
    const Real vegCell = veg.value("cullCell", 280.0);
    std::vector<std::vector<Mat4>> buckets =
        bucketPlacementsBySpecies(placements, variantList.size(), vegSeed + 7u);
    for (std::size_t si = 0; si < variantList.size(); ++si) {
        if (buckets[si].empty()) continue;

        std::map<std::pair<int, int>, std::vector<Mat4>> cells;
        const bool isRock = variantList[si].isRock;
        for (const Mat4& mIn : buckets[si]) {
            Mat4 m = mIn;
            // PER-INSTANCE VARIANCE + BEDDING (device: fields read uniform;
            // trees/rocks float or sink). Deterministic from the position hash.
            // Rocks: independent XZ/Y scale (squat boulders to tall outcrops)
            // and bed a third of their height into the ground. Trees: mild
            // extra scale jitter and root the trunk slightly below grade.
            const uint32_t hsh = static_cast<uint32_t>(
                (static_cast<int64_t>(m.m[0][3] * 8) * 73856093LL) ^
                (static_cast<int64_t>(m.m[2][3] * 8) * 19349663LL) ^ vegSeed);
            auto unit = [&](uint32_t salt) {
                uint32_t x = hsh ^ (salt * 2654435761u);
                x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu;
                x ^= x >> 16;
                return (x & 0xFFFFFF) / 16777215.0;
            };
            if (isRock) {
                // ROCK FORMATIONS (device: "tall slats, boulders, or smaller
                // groupings ... around the foothills"): each accepted placement
                // becomes a FORMATION — a deterministic cluster whose members
                // share a family look. Rocks live in the FOOTHILLS; inside the
                // city shelf only the occasional park boulder survives.
                const double baseY = m.m[1][3];
                if (baseY < 20.0 && unit(9) > 0.12) continue;   // shelf: rare
                const double kindRoll = unit(11);
                int members; double sxzLo, sxzHi, syLo, syHi, spread;
                if (kindRoll < 0.22) {          // tall slats in a rough row
                    members = 3 + (int)(unit(12) * 3); sxzLo = 0.35; sxzHi = 0.6;
                    syLo = 1.8; syHi = 2.9; spread = 1.0;
                } else if (kindRoll < 0.55) {   // chunky boulder cluster
                    members = 2 + (int)(unit(12) * 3); sxzLo = 1.3; sxzHi = 2.5;
                    syLo = 0.8; syHi = 1.5; spread = 2.2;
                } else if (kindRoll < 0.80) {   // small scatter grouping
                    members = 4 + (int)(unit(12) * 3); sxzLo = 0.4; sxzHi = 0.8;
                    syLo = 0.4; syHi = 0.8; spread = 1.4;
                } else {                        // lone stone
                    members = 1; sxzLo = 0.7; sxzHi = 1.6; syLo = 0.6; syHi = 1.4;
                    spread = 0;
                }
                const double rowAng = unit(13) * 6.2831853;
                const Vec2 rowDir(std::cos(rowAng), std::sin(rowAng));
                for (int mi = 0; mi < members; ++mi) {
                    Mat4 mm = m;
                    const uint32_t ms = 100 + (uint32_t)mi * 17u;
                    const double sxz = sxzLo + (sxzHi - sxzLo) * unit(ms + 1);
                    const double sy = syLo + (syHi - syLo) * unit(ms + 2);
                    for (int r = 0; r < 3; ++r) {
                        mm.m[r][0] *= sxz; mm.m[r][2] *= sxz;
                        mm.m[r][1] *= sy;
                    }
                    // slats march along the row; clusters spread radially
                    Vec2 off(0, 0);
                    if (members > 1) {
                        if (kindRoll < 0.22)
                            off = rowDir * (spread * (mi - members * 0.5) +
                                            (unit(ms + 3) - 0.5) * 0.5);
                        else {
                            const double oa = unit(ms + 4) * 6.2831853;
                            const double orr = spread * (0.4 + 0.6 * unit(ms + 5));
                            off = Vec2(std::cos(oa), std::sin(oa)) * orr;
                        }
                    }
                    mm.m[0][3] += off.x;
                    mm.m[2][3] += off.y;
                    const double gy = terrainHeight(terrain, terrainNoise,
                                                    mm.m[0][3], mm.m[2][3],
                                                    placeDilate);
                    mm.m[1][3] = gy - 0.33 * sy * variantList[si].trunkHeight;
                    const int cx2 = (int)std::floor(mm.m[0][3] / vegCell);
                    const int cz2 = (int)std::floor(mm.m[2][3] / vegCell);
                    cells[{cx2, cz2}].push_back(mm);
                }
                continue;   // formation members already binned
            } else {
                const double s = 0.85 + 0.3 * unit(3);
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) m.m[r][c] *= s;
                m.m[1][3] -= 0.12;   // bed the root ball just below grade
                // (was -0.35: tuned when placement sampled a DIFFERENT surface
                // than the mesh — with the dilate-matched sample that much
                // bedding buried every trunk on flat ground)
            }
            const int cx = static_cast<int>(std::floor(m.m[0][3] / vegCell));
            const int cz = static_cast<int>(std::floor(m.m[2][3] / vegCell));
            cells[{cx, cz}].push_back(m);
        }
        for (const auto& [key, transforms] : cells) {
            // Coarse group bounds: centroid of instance origins + the spread
            // (the per-part mesh extent is added below).
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
                g.drawClass = engine::DrawClass::Scenery;
                g.renderLayer = engine::LayerFoliage;   // debug layer toggle
                world.add<InstanceGroup>(world.create(), g);
            }
        }
        const std::vector<Mat4>& transforms = buckets[si];

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
    if (const std::string p = resolveScriptPath("vehicles.lua", levelDir); !p.empty())
        g_loadedScriptFiles.push_back(p);
    if (lib.empty()) {
        LOG_WARN << "vehicles: assets/scripts/vehicles.lua not found";
        return;
    }
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(levelDir, &g_loadedScriptFiles));
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
    std::vector<RenderMesh> flatParts;   // the LOD1 twin (city-render-perf R2)
    std::vector<engine::TerrainFlatten> gradeFlatten;   // in-pass block grades
    bool grown = false;
    // Lots read from a bundle whose parts are stored per render cell (ADR-0084 B): parts/flatParts stay
    // empty and the spawner instantiates these sections one chunk at a time. `bundle` keeps the mapping alive.
    std::shared_ptr<engine::bundle::Bundle> bundle;
    std::vector<engine::lotcache::LotCellPart> cellParts;
};

// The authored player spawn (world XZ): engine::authoredSpawnXZ in lot_grow_setup.h — read from the RAW
// level json BEFORE any spawn-safety relocation (the lot pass flags the building beside it as enterable).

// `ground` grades the lot pads; `netGround` is what the roads themselves drape
// on (the pre-pass nets use the NATURAL terrain — the same sampler their
// conform profiles were computed against), for the sampled clearance graph.
static GrownLots growCityLots(
    const engine::bundle::LevelInputs& inputs,
    const std::vector<engine::RoadEntity>& nets, const json& cs,
    const std::string& levelDir, const HeightField& ground,
    const HeightField& netGround,
    const engine::RoadGraph* freewayROW = nullptr,
    std::function<std::function<engine::Real(engine::Real, engine::Real, engine::Real)>(
        const std::vector<engine::TerrainFlatten>&)> groundWith = nullptr,
    double groundMeshCell = 0.0,
    const engine::Vec2* enterableAt = nullptr) {
    RT_PROFILE_ZONE_NAMED("growCityLots");
    GrownLots g;
    // ONE derivation (ADR-0084, milestone B): the parameters come from lotGrowSetupForLevel — the function
    // the `lots` bundle producer runs headlessly — so a grow here and a grow in rt_bake are the same city.
    // The setup owns the style book's VM for as long as the grow runs.
    engine::LotGrowSetup s = engine::lotGrowSetupForLevel(cs, levelDir, ground, nets, std::move(groundWith), groundMeshCell, enterableAt);
    for (const std::string& p : s.scriptFiles) g_loadedScriptFiles.push_back(p);
#ifdef RT_ENABLE_LANELAB
    if (!g_lanelab.blocks.empty()) {
        // The lane lab's blocks are exact to the kerb and already inset by the sidewalk (Clipper): the same
        // parceller and grammar, no road graph, and no miter inset to reject them.
        s.lp.roadMargin = 0;
        s.lp.sidewalkRise = engine::lanelab::lanelabSidewalkRise();   // paving meets the lab's sidewalk (ADR-0086)
        engine::NetLotResult r; bool fromBundle = false;
        const auto tl = std::chrono::steady_clock::now();
        auto since = [](const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); };
        // The lot pass's products come out of the level's bundle when the `lots` producer applies: the bundle
        // the city came from already holds them when both were baked together (one obtain per load, also under
        // RT_NOCACHE); otherwise they are obtained now — read, or baked with the city copied forward.
        if (const engine::bundle::BundleProducer* lotsProducer = engine::bundle::findProducer(engine::lanelab::kLotsProducerName);
            lotsProducer && lotsProducer->applies(g_lanelab.inputs)) {
            std::string err, status;
            const std::string want = engine::bundle::hex16(lotsProducer->identity(g_lanelab.inputs).key);
            // Per-cell parts stay in the bundle (no whole-part materialisation): the spawner reads each cell's
            // section when it makes the Renderable. A whole-part layout is read as before.
            auto readLots = [&](const std::shared_ptr<engine::bundle::Bundle>& b) {
                r = engine::NetLotResult(); g.cellParts.clear(); g.bundle.reset();
                const bool perCell = engine::lotcache::lotCellSize(*b, engine::lanelab::kLotsSectionPrefix) > 0.0;
                if (!engine::lotcache::readLotResult(*b, engine::lanelab::kLotsSectionPrefix, r, &err, /*withParts=*/!perCell)) return false;
                if (perCell && !engine::lotcache::listLotCellParts(*b, engine::lanelab::kLotsSectionPrefix, g.cellParts, &err)) return false;
                g.bundle = b; return true;
            };
            if (g_lanelab.bundle && engine::bundle::manifestProducer(g_lanelab.bundle->manifest(), engine::lanelab::kLotsProducerName).value("key", std::string()) == want) {
                fromBundle = readLots(g_lanelab.bundle);
                status = "from the city's bundle";
            }
            if (!fromBundle) {
                engine::bundle::Obtained o = engine::bundle::obtainForLevel(g_lanelab.inputs, engine::lanelab::kLotsProducerName);
                status = o.status;
                if (o.bundle) fromBundle = readLots(o.bundle);
            }
            if (fromBundle) {
                LOG_INFO << "[lanelab] lots " << status << ": " << r.lots.size() << " buildings, " << r.plan.lots.size() << " lots, " << (g.cellParts.empty() ? std::to_string(r.parts.size()) + " whole parts" : std::to_string(g.cellParts.size()) + " cell parts") << ", read in " << since(tl) << " s";
                // A warm load grows nothing, so the grow's skyline lines never
                // print; the census is computed from the records instead.
                const engine::SkylineCensus sc = engine::skylineCensus(r.lots);
                LOG_INFO << sc.line();
                LOG_INFO << sc.districtsLine();
            }
            else LOG_WARN << "[lanelab] lots bundle unusable (" << (err.empty() ? status : err) << "); growing in place";
        }
        if (!fromBundle) {
            r = engine::NetLotResult(); g.cellParts.clear(); g.bundle.reset();
            r.lots = engine::growLotBuildings(g_lanelab.blocks, s.lp, &r.plan, s.planOnly ? nullptr : &r.parts, nullptr, 0.0,
                                              (s.wantFlat && !s.planOnly) ? &r.flatParts : nullptr, &r.gradeFlatten);
            LOG_INFO << "[lanelab] lots on " << g_lanelab.blocks.size() << " scene blocks: " << r.lots.size() << " buildings, " << r.plan.lots.size() << " lots, grown in " << since(tl) << " s";
        }
        g.lots = std::move(r.lots); g.plan = std::move(r.plan); g.parts = std::move(r.parts); g.flatParts = std::move(r.flatParts); g.gradeFlatten = std::move(r.gradeFlatten); g.grown = true;
        (void)netGround; (void)freewayROW;
        return g;
    }
#endif
    // The lattice city's lot pre-pass comes out of the level's bundle when the `citylots` producer
    // applies (ADR-0084 milestone C) — the same grow, done once and stored with its parts already
    // split per render cell. A miss, an unreadable section or RT_NOCACHE falls through to growing here.
    engine::NetLotResult r;
    {
        const auto tl = std::chrono::steady_clock::now();
        auto since2 = [](const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); };
        engine::registerCityLotsProducer();
        bool fromBundle = false;
        if (const engine::bundle::BundleProducer* p = engine::bundle::findProducer(engine::kCityLotsProducerName);
            p && !inputs.levelPath.empty() && p->applies(inputs)) {
            std::string err;
            engine::bundle::Obtained o = engine::bundle::obtainForLevel(inputs, engine::kCityLotsProducerName);
            if (o.bundle) {
                const bool perCell = engine::lotcache::lotCellSize(*o.bundle, engine::kCityLotsSectionPrefix) > 0.0;
                if (engine::lotcache::readLotResult(*o.bundle, engine::kCityLotsSectionPrefix, r, &err, /*withParts=*/!perCell) &&
                    (!perCell || engine::lotcache::listLotCellParts(*o.bundle, engine::kCityLotsSectionPrefix, g.cellParts, &err))) {
                    g.bundle = o.bundle; fromBundle = true;
                    LOG_INFO << "[citylots] lots " << o.status << ": " << r.lots.size() << " buildings, " << r.plan.lots.size()
                             << " lots, " << (g.cellParts.empty() ? std::to_string(r.parts.size()) + " whole parts" : std::to_string(g.cellParts.size()) + " cell parts")
                             << ", read in " << since2(tl) << " s";
                    // A warm load grows nothing, so the grow's skyline lines
                    // never print; the census is computed from the records.
                    const engine::SkylineCensus sc = engine::skylineCensus(r.lots);
                    LOG_INFO << sc.line();
                    LOG_INFO << sc.districtsLine();
                } else {
                    r = engine::NetLotResult(); g.cellParts.clear(); g.bundle.reset();
                    LOG_WARN << "[citylots] lots bundle unusable (" << (err.empty() ? o.status : err) << "); growing in place";
                }
            }
        }
        if (!fromBundle)
            r = engine::growLotBuildingsOnNets(
                nets, s.lp, s.ep, s.roadClear, netGround, freewayROW, s.wantFlat, !s.planOnly);
    }
    g.lots = std::move(r.lots);
    g.plan = std::move(r.plan);
    g.parts = std::move(r.parts);
    g.flatParts = std::move(r.flatParts);
    g.gradeFlatten = std::move(r.gradeFlatten);
    g.grown = true;
    return g;
}

// THE CITY MAP (procgen/city/city_svg.h): everything the generators built,
// layered. Assembled here once the streets, lots, furniture plan and scatter
// exist; stored on the world (CityMap) so the viewer can write it any time
// (`citymap <path> [layers]`), and written at load for RT_CITY_SVG (all
// layers, or RT_CITY_SVG_LAYERS=roads,sidewalks,...) and RT_FURNITURE_SVG
// (the furniture-centric preset).
static std::shared_ptr<engine::CityMapData> assembleCityMap(
        World& world, const engine::RoadGraph& combined, const engine::NavGraph& nav,
        const engine::StreetFurniturePlan& plan, double hubRadius) {
    auto m = std::make_shared<engine::CityMapData>();
    m->roads = combined;
    m->nav = nav;
    m->furniture = plan;
    m->hubRadius = hubRadius;
    world.each<engine::RoadEntity>([&](Entity, engine::RoadEntity& net) {
        for (const engine::CityHub& h : net.plan.cityHubs) {
            engine::CityMapData::Hub hub;
            hub.pos = h.pos;
            hub.kind = h.kind;
            hub.name = engine::districtName(static_cast<engine::DistrictTag>(h.kind));
            m->hubs.push_back(hub);
        }
    });
    world.each<engine::RoadBandDebug>([&](Entity, engine::RoadBandDebug& b) {
        m->curbLoops.insert(m->curbLoops.end(), b.loops.begin(), b.loops.end());
        m->mouthGaps.insert(m->mouthGaps.end(), b.mouthGaps.begin(), b.mouthGaps.end());
        m->sidewalkWidth = std::max(m->sidewalkWidth, static_cast<double>(b.sidewalkWidth));
    });
    world.each<engine::CityPlanDebug>([&](Entity, engine::CityPlanDebug& d) {
        m->blocks.insert(m->blocks.end(), d.blocks.begin(), d.blocks.end());
        m->lots.insert(m->lots.end(), d.lots.begin(), d.lots.end());
        for (const engine::CityPlanDebug::Prism& pr : d.prisms)
            m->buildings.push_back({pr.plan, pr.district, pr.type});
    });
    world.each<engine::AuthoredPlace>([&](Entity, engine::AuthoredPlace& p) {
        m->places.push_back({engine::Vec2(p.x, p.z), p.type, p.name});
    });
    world.each<engine::CityBuildings>([&](Entity, engine::CityBuildings& cb) {
        for (const engine::BuildingRecord& r : cb.records)
            for (const engine::DoorSpec& d : r.doors)
                m->doors.push_back({d.foot, d.normal, r.enterable});
    });
    world.each<engine::InstanceGroup>([&](Entity, engine::InstanceGroup& g) {
        engine::CityMapData::ObjectKind kind;
        if (g.drawClass == engine::DrawClass::Scenery) kind = engine::CityMapData::ObjectKind::Scenery;
        else if (g.drawClass == engine::DrawClass::Furniture) kind = engine::CityMapData::ObjectKind::Furniture;
        else return;
        for (const Mat4& t : g.transforms)
            m->objects.push_back({engine::Vec2(t.m[0][3], t.m[2][3]), kind});
    });
    return m;
}

const std::vector<std::string>& LevelLoader::lastLoadedScriptFiles() {
    return g_loadedScriptFiles;
}

static LevelLoader::GroundProbeReport g_groundProbeReport;
const LevelLoader::GroundProbeReport& LevelLoader::lastGroundProbeReport() {
    return g_groundProbeReport;
}
static LevelLoader::PokeReport g_pokeReport;
const LevelLoader::PokeReport& LevelLoader::lastPokeReport() {
    return g_pokeReport;
}

// ELEVATION MAPS (RT_ELEVATION_MAP=<prefix>): top-down heightmaps from an
// INDEPENDENT probe grid — not the flatten set, not the road samples, not the
// CDLOD vertices — so "how the terrain changed" can be SEEN, not inferred.
// Four surfaces per probe: natural (the base relief the roads were solved
// against), the final analytic ground (every stamp applied), the finest tile's
// own bilinear interpolation (what is drawn), and — once the earthwork field
// exists — the displacement alone. Written as PNGs sharing the citymap's
// world extent (x right, z down, same as the SVG viewBox) so they overlay
// pixel-exact, plus a JSON sidecar with the extent, cell and scales. Device:
// "something akin to an elevation map so we can see the base terrain and how
// the terrain changed ... a top down way to debug the landscape."
namespace {
struct ElevationSurface {
    const char* name;
    std::function<double(double, double)> at;
};
void writeElevationMaps(const std::string& prefix, double half, double cell,
                        const std::vector<ElevationSurface>& surfaces,
                        const std::vector<std::vector<Vec2>>& polylines,
                        bool hasMark, double markX, double markZ) {
    const int n = std::max(2, static_cast<int>(std::ceil(2.0 * half / cell)) + 1);
    const double minX = -half, minZ = -half;
    const std::size_t N = static_cast<std::size_t>(n) * n;
    std::vector<std::vector<double>> H(surfaces.size(), std::vector<double>(N));
    for (std::size_t s = 0; s < surfaces.size(); ++s)
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i)
                H[s][static_cast<std::size_t>(j) * n + i] =
                    surfaces[s].at(minX + i * cell, minZ + j * cell);
    auto idx = [&](int i, int j) {
        i = std::max(0, std::min(n - 1, i));
        j = std::max(0, std::min(n - 1, j));
        return static_cast<std::size_t>(j) * n + i;
    };
    // Shared height range across the absolute surfaces so they compare.
    double lo = 1e300, hi = -1e300;
    for (std::size_t s = 0; s < surfaces.size(); ++s)
        for (double v : H[s]) { lo = std::min(lo, v); hi = std::max(hi, v); }
    const double span = std::max(1e-6, hi - lo);
    auto overlay = [&](std::vector<unsigned char>& rgb) {
        auto plot = [&](int i, int j, unsigned char r, unsigned char g, unsigned char b) {
            if (i < 0 || j < 0 || i >= n || j >= n) return;
            unsigned char* p = &rgb[(static_cast<std::size_t>(j) * n + i) * 3];
            p[0] = r; p[1] = g; p[2] = b;
        };
        for (const auto& pl : polylines)
            for (std::size_t k = 0; k + 1 < pl.size(); ++k) {
                const double x0 = (pl[k].x - minX) / cell, z0 = (pl[k].y - minZ) / cell;
                const double x1 = (pl[k + 1].x - minX) / cell, z1 = (pl[k + 1].y - minZ) / cell;
                const int steps = std::max(1, static_cast<int>(std::ceil(
                    std::max(std::fabs(x1 - x0), std::fabs(z1 - z0)))));
                for (int t = 0; t <= steps; ++t) {
                    const double f = static_cast<double>(t) / steps;
                    plot(static_cast<int>(std::lround(x0 + (x1 - x0) * f)),
                         static_cast<int>(std::lround(z0 + (z1 - z0) * f)), 20, 20, 20);
                }
            }
        if (hasMark) {
            const int mi = static_cast<int>(std::lround((markX - minX) / cell));
            const int mj = static_cast<int>(std::lround((markZ - minZ) / cell));
            for (int d = -6; d <= 6; ++d) {
                plot(mi + d, mj, 255, 0, 255);
                plot(mi, mj + d, 255, 0, 255);
            }
        }
    };
    // Absolute surfaces: greyscale height with a NW hillshade so relief reads.
    for (std::size_t s = 0; s < surfaces.size(); ++s) {
        std::vector<unsigned char> rgb(N * 3);
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const double h = H[s][idx(i, j)];
                const double dx = (H[s][idx(i + 1, j)] - H[s][idx(i - 1, j)]) / (2 * cell);
                const double dz = (H[s][idx(i, j + 1)] - H[s][idx(i, j - 1)]) / (2 * cell);
                // normal (-dx, 1, -dz), light from the NW and above
                const double nl = (dx * 0.5 + 1.0 * 0.7 + dz * 0.5) /
                                  std::sqrt(dx * dx + 1.0 + dz * dz);
                const double shade = 0.55 + 0.45 * std::max(0.0, nl);
                const double g = (40.0 + 200.0 * (h - lo) / span) * shade;
                const unsigned char v = static_cast<unsigned char>(
                    std::max(0.0, std::min(255.0, g)));
                unsigned char* p = &rgb[idx(i, j) * 3];
                p[0] = v; p[1] = v; p[2] = v;
            }
        overlay(rgb);
        const std::string path = prefix + "_" + surfaces[s].name + ".png";
        stbi_write_png(path.c_str(), n, n, 3, rgb.data(), n * 3);
    }
    // Differences against the FIRST surface (natural): signed, diverging —
    // blue = lowered (cut), red = raised (fill) — scaled to the max |d|.
    std::vector<double> diffScale(surfaces.size(), 0.0);
    std::vector<std::pair<double, double>> diffWorstAt(surfaces.size(), {0, 0});
    for (std::size_t s = 1; s < surfaces.size(); ++s) {
        double mx = 0; int mi = 0, mj = 0;
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const double d = H[s][idx(i, j)] - H[0][idx(i, j)];
                if (std::fabs(d) > mx) { mx = std::fabs(d); mi = i; mj = j; }
            }
        diffScale[s] = mx;
        diffWorstAt[s] = {minX + mi * cell, minZ + mj * cell};
        const double sc = std::max(1e-6, mx);
        std::vector<unsigned char> rgb(N * 3);
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const double d = (H[s][idx(i, j)] - H[0][idx(i, j)]) / sc;   // -1..1
                const double a = std::min(1.0, std::fabs(d));
                unsigned char* p = &rgb[idx(i, j) * 3];
                if (d >= 0) {   // raised: white -> red
                    p[0] = 255; p[1] = static_cast<unsigned char>(255 * (1 - a));
                    p[2] = static_cast<unsigned char>(255 * (1 - a));
                } else {        // lowered: white -> blue
                    p[0] = static_cast<unsigned char>(255 * (1 - a));
                    p[1] = static_cast<unsigned char>(255 * (1 - a)); p[2] = 255;
                }
            }
        overlay(rgb);
        const std::string path = prefix + "_" + surfaces[s].name + "_minus_" +
                                 surfaces[0].name + ".png";
        stbi_write_png(path.c_str(), n, n, 3, rgb.data(), n * 3);
    }
    std::ofstream js(prefix + ".json");
    js << "{ \"minX\": " << minX << ", \"minZ\": " << minZ << ", \"cell\": " << cell
       << ", \"size\": " << n << ", \"heightLo\": " << lo << ", \"heightHi\": " << hi
       << ", \"surfaces\": [";
    for (std::size_t s = 0; s < surfaces.size(); ++s)
        js << (s ? ", " : "") << "{ \"name\": \"" << surfaces[s].name
           << "\", \"diffScale\": " << diffScale[s] << ", \"diffWorstX\": "
           << diffWorstAt[s].first << ", \"diffWorstZ\": " << diffWorstAt[s].second << " }";
    js << "] }\n";
    // THE BANK CENSUS: how steep is the ground beside the roads? Cells within
    // 20 m of any road centreline whose slope exceeds 40% — the "cliff" gate
    // for the earthwork field (a fixed 8 m feather absorbing a metres-high
    // disagreement is exactly a >40% bank). Per surface, so natural / final /
    // drawn / earthworked can be compared on the same probes.
    {
        const int reachCells = std::max(1, static_cast<int>(std::ceil(20.0 / cell)));
        std::vector<unsigned char> nearRoad(N, 0);
        for (const auto& pl : polylines)
            for (std::size_t k = 0; k + 1 < pl.size(); ++k) {
                const double x0 = (pl[k].x - minX) / cell, z0 = (pl[k].y - minZ) / cell;
                const double x1 = (pl[k + 1].x - minX) / cell, z1 = (pl[k + 1].y - minZ) / cell;
                const int steps = std::max(1, static_cast<int>(std::ceil(
                    std::max(std::fabs(x1 - x0), std::fabs(z1 - z0)))));
                for (int t = 0; t <= steps; ++t) {
                    const double f = static_cast<double>(t) / steps;
                    const int ci = static_cast<int>(std::lround(x0 + (x1 - x0) * f));
                    const int cj = static_cast<int>(std::lround(z0 + (z1 - z0) * f));
                    for (int dj = -reachCells; dj <= reachCells; ++dj)
                        for (int di = -reachCells; di <= reachCells; ++di) {
                            if (di * di + dj * dj > reachCells * reachCells) continue;
                            const int i = ci + di, j = cj + dj;
                            if (i >= 0 && j >= 0 && i < n && j < n) nearRoad[idx(i, j)] = 1;
                        }
                }
            }
        std::ostringstream bank;
        bank << "[bank-census] within 20 m of a road, cells over 40% slope:";
        for (std::size_t s = 0; s < surfaces.size(); ++s) {
            long over = 0, total = 0;
            double worst = 0, wx = 0, wz = 0;
            for (int j = 1; j + 1 < n; ++j)
                for (int i = 1; i + 1 < n; ++i) {
                    if (!nearRoad[idx(i, j)]) continue;
                    ++total;
                    const double dx = (H[s][idx(i + 1, j)] - H[s][idx(i - 1, j)]) / (2 * cell);
                    const double dz = (H[s][idx(i, j + 1)] - H[s][idx(i, j - 1)]) / (2 * cell);
                    const double slope = std::sqrt(dx * dx + dz * dz);
                    if (slope > 0.40) ++over;
                    if (slope > worst) { worst = slope; wx = minX + i * cell; wz = minZ + j * cell; }
                }
            bank << " " << surfaces[s].name << " " << over << "/" << total
                 << " (worst " << static_cast<int>(worst * 100) << "% at " << wx << "," << wz << ")";
        }
        LOG_INFO << bank.str();
    }
    std::ostringstream summary;
    summary << "[elevation-map] cell=" << cell << " size=" << n << "x" << n
            << " heights [" << lo << ", " << hi << "]";
    for (std::size_t s = 1; s < surfaces.size(); ++s)
        summary << " | " << surfaces[s].name << "-" << surfaces[0].name
                << " max |d| " << diffScale[s] << " m at (" << diffWorstAt[s].first
                << "," << diffWorstAt[s].second << ")";
    summary << " -> " << prefix << "_*.png";
    LOG_INFO << summary.str();
}
}  // namespace

bool LevelLoader::load(const std::string& path,
                       World& world, Renderer& renderer, RenderView& view,
                       AssetManager& assets, bool editorMode) {
    RT_PROFILE_ZONE_NAMED("levelLoad");
    g_loadedScriptFiles.clear();
    const auto tLoad0 = std::chrono::steady_clock::now();
    // Per-stage load timing (metropolis-scale-plan P4.3). One line at the end, so the cost of a
    // cold city is attributable without a profiler — which is what sizing a bundle producer needs.
    auto tStagePrev = tLoad0; std::string loadStages;
    auto loadStage = [&](const char* name) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - tStagePrev).count();
        tStagePrev = now;
        if (dt < 0.05) return;                     // only stages worth seeing
        char buf[96]; std::snprintf(buf, sizeof buf, "%s%s %.2f s", loadStages.empty() ? "" : ", ", name, dt);
        loadStages += buf;
    };
    g_groundProbeReport = {};
    g_pokeReport = {};
#ifdef RT_ENABLE_LANELAB
    g_lanelab = LaneLabPublished();
#endif
    std::ifstream file(path);
    if (!file.is_open()) {
        LOG_ERROR << "Failed to open level file: " << path;
        return false;
    }

    json root;
    try {
        root = json::parse(file);
#ifdef RT_ENABLE_LANELAB
        g_lanelab.inputs.levelPath = path; g_lanelab.inputs.level = root;
        { const size_t slash = path.find_last_of('/'); g_lanelab.inputs.levelDir = slash == std::string::npos ? "." : path.substr(0, slash); }
#endif
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
    auto sharedEroded = readErodedBase(root);
#ifdef RT_ENABLE_LANELAB
    // THE LANE LAB'S GROUND, RENDERED BY CDLOD (2026-09-08). The lab bakes the source level's terrain to a
    // 5 m grid and conforms it to the roads, then meshed that grid itself: no LOD, no morphing, no material
    // blending, and 400k triangles of it. CDLOD builds its nodes from TerrainParams, so it cannot take a
    // height callback — but `terrainHeight` reads `params.erodedBase` INSTEAD of the analytic relief, and
    // that is a std::function. Point it at the conformed grid and CDLOD renders the lab's surface with
    // everything the real terrain path has. The grid only covers the roads plus a margin, so outside it the
    // sampler falls back to the level's own terrain and blends across a band, or the edge value would smear
    // over the whole world.
    if (root.contains("terrain") && !engine::lanelab::cityEntities(root).empty()) {
        engine::lanelab::registerCityProducer(); engine::lanelab::registerLotsProducer();
        engine::bundle::Obtained o = engine::bundle::obtainForLevel(g_lanelab.inputs, engine::lanelab::kCityProducerName);
        engine::lanelab::CityProducts cp; std::string cperr;
        if (o.bundle && engine::lanelab::readCityProducts(*o.bundle, 0, cp, &cperr) && cp.hasTerrain) {
            g_lanelab.bundle = o.bundle; g_lanelab.bundleStatus = o.status;   // loadLaneLabEntity reuses it
            auto grid = std::make_shared<engine::lanelab::HeightGrid>();
            grid->x0 = cp.ground.x0; grid->y0 = cp.ground.y0; grid->res = cp.ground.res;
            grid->nx = cp.ground.nx; grid->ny = cp.ground.ny; grid->z = cp.ground.z;
            // Publish the lab's BLOCKS and GROUND now, before the terrain pre-pass grows the lots. The
            // pre-pass is where building pads and block grades are stamped into the terrain; a lab level
            // whose lots grew only at entity time (after the terrain) had every pad miss the CDLOD ground —
            // 706 of 1284 lots buried, the reason this block stayed parked on 2026-09-08. loadLaneLabEntity
            // re-derives the same blocks from the same products (deterministic), so nothing disagrees.
            g_lanelab.ground = [grid](double x, double z) { return grid->sample(x, z); };
            g_lanelab.sidewalk = root.contains("citysim") && root["citysim"].is_object() ? root["citysim"].value("sidewalk", 4.0) : 4.0;
            g_lanelab.blocks = engine::lanelab::blocksFromHoles(cp.holes, 1.5, g_lanelab.sidewalk);
            g_lanelab.row = cp.row;
            LOG_INFO << "[lanelab] " << g_lanelab.blocks.size() << " city blocks published for the terrain pre-pass";
            auto fbTp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
            fbTp->erodedBase = sharedEroded;          // the fallback keeps whatever base the level had; no recursion
            auto fbNoise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
            const double bx1 = grid->x0 + grid->res * (grid->nx - 1), by1 = grid->y0 + grid->res * (grid->ny - 1);
            sharedEroded = std::make_shared<const std::function<double(double, double)>>(
                [grid, fbTp, fbNoise, bx1, by1](double x, double z) {
                    const double band = 60.0;   // blend to the level's own terrain over the last 60 m of the grid
                    const double inset = std::min(std::min(x - grid->x0, bx1 - x), std::min(z - grid->y0, by1 - z));
                    if (inset <= 0.0) return terrainHeight(*fbTp, *fbNoise, x, z);
                    const double lab = grid->sample(x, z);
                    if (inset >= band) return lab;
                    const double u = inset / band, w = u * u * (3 - 2 * u);
                    return terrainHeight(*fbTp, *fbNoise, x, z) * (1 - w) + lab * w;
                });
            LOG_INFO << "[lanelab] CDLOD terrain from the lab's conformed grid: " << grid->nx << " x " << grid->ny
                     << " @ " << grid->res << " m, " << (bx1 - grid->x0) << " x " << (by1 - grid->y0) << " m";
        } else if (!cperr.empty()) LOG_WARN << "[lanelab] no conformed ground for CDLOD: " << cperr;
    }
#endif

    // A city draped on the terrain is generated BEFORE the terrain: it grades its
    // roads/blocks off the natural ground, then returns cut/fill footprints the
    // terrain is built around (so the ground meets the carriageways). The same
    // model is reused when the entity is spawned below.

    // Level ground sampler (ADR-0044): the NATURAL terrain height handed to an
    // on-terrain script recipe (the `ground` global) so a Lua city drapes on and
    // conforms the CDLOD terrain — the script sibling of the C++ city's groundAt.
    HeightField levelGround;
    // Finest rendered CDLOD cell (leaf node / gridRes) — the walkway sculptors
    // sample ground through the tile's own interpolation on this grid so
    // ribbons sit on the MESH, not on the analytic function between samples.
    double lotMeshCell = 0.0;
    if (root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
        tp->erodedBase = sharedEroded;   // roads/lots conform to the ERODED surface
        auto noise = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        levelGround = [tp, noise](double x, double z) {
            return terrainHeight(*tp, *noise, x, z);
        };
        const json& tj = root["terrain"];
        if (tj.contains("cdlod")) {
            const json& cj = tj["cdlod"];
            const double worldHalf =
                cj.is_object() ? cj.value("worldHalf", 1024.0) : 1024.0;
            const int numLods = cj.is_object() ? cj.value("numLods", 6) : 6;
            const int gridRes = cj.is_object() ? cj.value("gridRes", 32) : 32;
            lotMeshCell = (worldHalf * 2.0 / double(1 << (numLods - 1))) /
                          std::max(1, gridRes);
        }
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

    loadStage("terrain parse");
    // Road pre-pass (ADR-0044 corridor conforming): grade the terrain to each editable
    // road (shape:"road") BEFORE it builds, mirroring the script pre-pass, so the ground
    // meets the road's drivable profile and no terrain pokes through.
    std::vector<TerrainFlatten> roadFlatten;
    std::vector<engine::RoadEntity> preNets;   // parsed nets, for the lot pre-pass
    std::vector<const json*> preNetEnts;    // matching entity per pre-pass net
    RenderMesh roadWallMesh;                 // ADR-0075 P1b: retaining/fill walls (world space)
    if (levelGround && root.contains("entities")) {
        for (const auto& ent : root["entities"]) {
            if (ent.value("shape", std::string()) == "road") {
                const json roadBlock =
                    ent.contains("road") ? ent["road"] : json::object();
                RoadEntity net = roadNetFromJson(roadBlock);
                // A GENERATED road has no baked nodes — run its recipe here
                // exactly like the real build below does, or this pre-pass
                // sees an empty net and carves NOTHING (device: "the road is
                // being buried by the terrain — it's not conforming").
                // levelGround (natural) gates the metro's terrain-aware layout.
                if (roadBlock.contains("generate"))
                    applyGenerateRecipe(net, roadBlock["generate"], levelGround);
                std::vector<TerrainFlatten> r =
                    roadNetConformRegions(net, levelGround);
                roadFlatten.insert(roadFlatten.end(), r.begin(), r.end());
                preNets.push_back(std::move(net));
                preNetEnts.push_back(&ent);
            }
        }
    }
    // CORRIDORS (plan §8): freeway-grade alignments. The mesh is built here
    // against the BASE terrain so its at-grade flatten windows join the same
    // carve set the roads use; the entities spawn after the terrain does.
    // Roads-v2.1 2e: there is no corridor renderer. corridorAuthor SOLVES
    // (ramp centrelines + at-grade flatten windows) and bakeCorridorIntoNet
    // turns the solve into ordinary graph edges; THE road mesher builds the
    // freeway from the graph like any street. Flatten flows straight into
    // roadFlatten; nav routes the baked edges natively via navRoadGraph.
    std::vector<CorridorDef> corridorDefs;
    // Roads-v2 S3b: which preNet each SYNTH corridor came from (-1 = authored /
    // rules-lab). The solved corridor BAKES into that net (bakeCorridorIntoNet)
    // so freeways+ramps land in the editable street graph before the road
    // entities spawn from roadCache.
    std::vector<int> corridorSrcNet;
    if (levelGround && root.contains("entities")) {
        for (const auto& ent : root["entities"]) {
            if (ent.value("shape", std::string()) != "corridor") continue;
            const json cb = ent.contains("corridor") ? ent["corridor"] : json::object();
            CorridorDef def;
            std::vector<Vec2> control;
            for (const auto& p : cb.value("points", json::array()))
                if (p.is_array() && p.size() >= 2)
                    control.emplace_back(p[0].get<double>(), p[1].get<double>());
            if (control.size() < 2) continue;
            def.horizontal = Alignment::fromPolyline(
                control, cb.value("radius", 220.0), cb.value("spiral", 60.0));
            def.lanes.throughLanes = cb.value("lanes", 4);
            def.laneWidth = cb.value("laneWidth", 3.6);
            def.medianWidth = cb.value("median", 1.4);
            def.designSpeed = cb.value("designSpeed", 30.0);
            if (cb.contains("profile") && cb["profile"].is_array()) {
                for (const auto& pv : cb["profile"])
                    if (pv.is_array() && pv.size() >= 2)
                        def.vertical.pvis.push_back(
                            {pv[0].get<double>(), pv[1].get<double>(),
                             pv.size() > 2 ? pv[2].get<double>() : 0.0});
            } else {
                // No authored profile: follow the terrain at a smoothed grade
                // (PVIs every 80 m at ground height + a small embankment).
                const Real len = def.horizontal.length();
                for (Real s = 0; s <= len; s += 80.0) {
                    const Vec2 p = def.horizontal.pos(std::min(s, len));
                    def.vertical.pvis.push_back(
                        {std::min(s, len), levelGround(p.x, p.y) + 0.4, 50.0});
                }
            }
            // (exits parse used to hide inside the profile branch — a
            // profile-less corridor silently lost its ramps)
            for (const auto& ex : cb.value("exits", json::array())) {
                engine::ExitDef e;
                e.station = ex.value("station", 0.0);
                e.upStation = ex.value("upStation", true);
                if (ex.contains("target") && ex["target"].is_array() &&
                    ex["target"].size() >= 2)
                    e.target = Vec2(ex["target"][0].get<double>(),
                                    ex["target"][1].get<double>());
                // landing grade defaults to the street's own terrain height
                e.targetY = ex.value("targetY",
                                     levelGround(e.target.x, e.target.y) + 0.12);
                e.decelLength = ex.value("decel", 220.0);
                e.onRamp = ex.value("onRamp", false);
                e.rampRadius = ex.value("radius", 70.0);
                e.rampSpiral = ex.value("spiral", 30.0);
                def.exits.push_back(e);
            }
            // 2e: authored corridors bake too — into the level's first street
            // net (creating an empty one when the level has none), so the one
            // mesher builds them exactly like the metro-planned corridors.
            if (preNets.empty()) {
                preNets.emplace_back();
                preNetEnts.push_back(&ent);
            }
            corridorSrcNet.push_back(0);
            corridorDefs.push_back(std::move(def));
            // One-mesher P8: still OPT-IN for authored corridors. Flipping this
            // to true is a prerequisite for deleting the corridor mesher, but
            // the weld renders freeway_lab's gores differently from the geometry
            // it would replace (unexplained dark wedges), and the deletion gate
            // is "no regression" — so this stays false until that is understood.
        }
    }
    std::vector<std::vector<Vec2>> corridorGuides;   // §12: per built def, its
                                                     // own dense centreline
                                                     // (empty for authored)
    // §10.6/§12: metro-planned corridors with NETWORK RULES (device: "It
    // needs to have rules about elevation and not having the freeways
    // intersect. on and off ramps should not criss-cross"):
    //   rule 1 (generator): one route per hub — routes never touch.
    //   rule 2 (here): residual geometric crossings force the SHORTER route
    //          OVER on a bridge bump solved into its profile.
    //   rule 3 (here): one GLOBAL landing/ramp registry — a ramp that would
    //          crowd a landing, cross another ramp, or graze another
    //          corridor at grade is never stamped.
    if (levelGround) {
        // The route inputs: rules-lab plans authored straight in the level
        // JSON ("freewayPlans": [[[x,z],...], ...] — every planner rule
        // exercisable in ISOLATION, docs/freeway-rules.md), then the plans
        // each metro net grew (srcNet = the bake target, S3b).
        std::vector<engine::CorridorRouteInput> routeIns;
        if (root.contains("freewayPlans") && root["freewayPlans"].is_array()) {
            for (const auto& jp : root["freewayPlans"]) {
                engine::CorridorRouteInput in;
                in.spacing = root.value("interchangeSpacing", 700.0);
                for (const auto& q : jp)
                    if (q.is_array() && q.size() >= 2)
                        in.anchors.emplace_back(q[0].get<double>(),
                                                q[1].get<double>());
                if (in.anchors.size() < 2) continue;
                routeIns.push_back(std::move(in));
            }
        }
        for (std::size_t ni = 0; ni < preNets.size(); ++ni) {
            const json& rootEnt = *preNetEnts[ni];
            const json gen = rootEnt.contains("road") &&
                                     rootEnt["road"].contains("generate")
                                 ? rootEnt["road"]["generate"]
                                 : json::object();
            const Real spacing = gen.value("interchange_spacing", 700.0);
            for (const std::vector<Vec2>& plan : preNets[ni].plan.freewayPlans) {
                if (plan.size() < 2) continue;
                engine::CorridorRouteInput in;
                in.anchors = plan;
                in.spacing = spacing;
                in.srcNet = static_cast<int>(ni);   // S3b: bake target
                routeIns.push_back(std::move(in));
            }
        }
        // §12 R3f: street-anchor snapshot for FEASIBILITY-driven placement
        std::vector<engine::CorridorStreetAnchor> synthAnchors;
        for (const engine::RoadEntity& net2 : preNets) {
            const engine::RoadGraph& g2 = net2.graph;
            std::vector<int> deg(g2.nodes.size(), 0);
            for (const engine::RoadEdge& ed : g2.edges) {
                if (ed.a >= 0 && ed.a < static_cast<int>(deg.size())) ++deg[ed.a];
                if (ed.b >= 0 && ed.b < static_cast<int>(deg.size())) ++deg[ed.b];
            }
            for (std::size_t k = 0; k < g2.nodes.size(); ++k)
                synthAnchors.push_back({g2.nodes[k].pos, deg[k]});
        }
        // The NETWORK-RULES pipeline lives in corridor_plan.cpp now, SHARED
        // with the editor's recipe-Regenerate (rebakeNetCorridors) — the
        // regen path must run the same rules or baked freeways vanish on a
        // Regenerate (the S3 known gap).
        for (engine::PlannedCorridor& pc :
             engine::planCorridorRoutes(routeIns, synthAnchors, levelGround)) {
            while (corridorGuides.size() < corridorDefs.size())
                corridorGuides.emplace_back();       // authored defs: no guide
            corridorGuides.push_back(std::move(pc.guide));
            while (corridorSrcNet.size() < corridorDefs.size())
                corridorSrcNet.push_back(-1);
            corridorSrcNet.push_back(pc.srcNet);
            corridorDefs.push_back(std::move(pc.def));
        }
    }
    // §10.6: streets may pass UNDER a viaduct, never THROUGH an at-grade
    // corridor — cut street edges that cross a low span of any corridor
    // (corridor_plan.cpp, shared with the editor's recipe-Regenerate).
    if (levelGround && !corridorDefs.empty()) {
        std::vector<engine::RoadEntity*> netPtrs;
        for (engine::RoadEntity& net : preNets) netPtrs.push_back(&net);
        engine::cutStreetsUnderCorridors(netPtrs, corridorDefs, levelGround);
    }
    if (levelGround && !corridorDefs.empty()) {
        for (CorridorDef& def : corridorDefs) {
            // §10.3/§12 landing resolution (corridor_plan.cpp, shared with
            // the editor's recipe-Regenerate): each ramp claims a real street
            // JUNCTION node on its own side (or splits the nearest street
            // edge into a new T-junction), the unreachable are dropped
            // (station = -1), and the criss-cross guard prunes any pair
            // whose gore->landing runs still X.
            std::vector<engine::RoadEntity*> landNets;
            for (engine::RoadEntity& n2 : preNets) landNets.push_back(&n2);
            std::vector<std::pair<int, int>> rampAnchors =
                engine::resolveCorridorLandings(def, landNets, levelGround);
            // Roads-v2.1 2e: the corridor SOLVES and BAKES — nothing here
            // draws. corridorAuthor engineers the ramp centrelines and the
            // at-grade flatten windows; the bake turns the solved corridor
            // into ordinary graph edges of its street net, and THE one road
            // mesher (buildRoadNetMesh, via roadCache below) builds deck,
            // ramps, parapets, undersides and portal bents from that graph.
            // The corridor renderer (sweepCorridor + its entity) is deleted;
            // signage returns with the R6 furniture pass.
            engine::CorridorAuthoring au =
                engine::corridorAuthor(def, levelGround, 3.0);
            {
                const std::size_t di =
                    static_cast<std::size_t>(&def - corridorDefs.data());
                if (di < corridorSrcNet.size() && corridorSrcNet[di] >= 0 &&
                    corridorSrcNet[di] < static_cast<int>(preNets.size())) {
                    engine::RoadEntity& src = preNets[corridorSrcNet[di]];
                    const std::size_t e0 = src.graph.edges.size();
                    engine::bakeCorridorIntoNet(src, def, au.rampPaths, {},
                                                levelGround);
                    LOG_INFO << "[bake] corridor -> net " << corridorSrcNet[di]
                             << ": +" << (src.graph.edges.size() - e0)
                             << " edges (freeway+ramps in the editable graph)";
                }
            }
            roadFlatten.insert(roadFlatten.end(), au.flatten.begin(),
                               au.flatten.end());
        }
    }
    loadStage("roads + corridors");
    // §10: ONE derived road graph. Streets and corridor chains weld HERE, at
    // build — the citysim nav, street furniture, and editor all read this
    // single component instead of merging graphs privately.
    if (!preNets.empty()) {
        engine::LevelRoadGraph lrg;
        for (const engine::RoadEntity& net : preNets) {
            engine::RoadGraph g = engine::navRoadGraph(net, levelGround);
            const int base = static_cast<int>(lrg.graph.nodes.size());
            for (const engine::RoadNode& n : g.nodes)
                lrg.graph.nodes.push_back(n);
            for (engine::RoadEdge e : g.edges) {
                e.a += base; e.b += base;
                lrg.graph.edges.push_back(e);
            }
        }
        if (!lrg.graph.edges.empty()) {
            LOG_INFO << "[roadgraph] unified: " << lrg.graph.nodes.size()
                     << " nodes, " << lrg.graph.edges.size()
                     << " edges (freeway+ramps native from the baked graph)";
            // PROOF DUMP (RT_DUMP_ROADGRAPH=path): write the ONE unified graph —
            // every street, arterial, freeway carriageway and ramp — so its
            // connectivity and cross-class reachability can be checked and drawn.
            if (const char* dp = std::getenv("RT_DUMP_ROADGRAPH")) {
                std::ofstream f(dp);
                f << "{\"nodes\":[";
                for (std::size_t i = 0; i < lrg.graph.nodes.size(); ++i) {
                    const engine::RoadNode& n = lrg.graph.nodes[i];
                    f << (i ? "," : "") << "[" << n.pos.x << "," << n.pos.y
                      << "," << n.elev << "," << static_cast<int>(n.kind) << "]";
                }
                f << "],\"edges\":[";
                for (std::size_t i = 0; i < lrg.graph.edges.size(); ++i) {
                    const engine::RoadEdge& e = lrg.graph.edges[i];
                    f << (i ? "," : "") << "[" << e.a << "," << e.b << ","
                      << static_cast<int>(e.klass) << "," << e.layer << ","
                      << static_cast<int>(e.provenance) << "]";
                }
                f << "]}";
                LOG_INFO << "[roadgraph] dumped " << lrg.graph.nodes.size()
                         << " nodes to " << dp;
            }
            // Load-time PLANARITY REPORT (#18): the bake already REJECTS
            // crossing ramp chains; this catches anything else (authored
            // nets, multi-net overlaps) loudly instead of silently meshing
            // roads through each other.
            {
                const auto viols =
                    engine::auditRoadGraph(lrg.graph, levelGround);
                // Corridor crossings are reported individually below (each is
                // its own LOG_ERROR); only the street-street count is
                // summarised, so there is nothing to tally for them.
                int street = 0;
                for (const auto& v : viols) {
                    const bool corr =
                        lrg.graph.edges[v.edgeA].klass == engine::RoadClass::Freeway ||
                        lrg.graph.edges[v.edgeA].klass == engine::RoadClass::Ramp ||
                        lrg.graph.edges[v.edgeB].klass == engine::RoadClass::Freeway ||
                        lrg.graph.edges[v.edgeB].klass == engine::RoadClass::Ramp;
                    if (corr) {
                        LOG_ERROR << "[roadgraph] corridor edges " << v.edgeA
                                  << " and " << v.edgeB << " cross at ("
                                  << v.at.x << ", " << v.at.y << ") with only "
                                  << v.dY << " m clearance";
                    } else {
                        ++street;
                    }
                }
                if (street)
                    LOG_WARN << "[roadgraph] " << street
                             << " street-street curve crossing(s) — the metro "
                                "generator's own planarity debt (tracked "
                                "separately from #18)";
            }
            world.add<engine::LevelRoadGraph>(world.create(), std::move(lrg));
        }
    }

    HeightField entityGround = levelGround;   // entities drape on the carved terrain (below)

    // The routed freeway RIGHT-OF-WAY for the lot pass to build (and
    // re-zone) around — from the BAKED graph now (2e): navRoadGraph carries
    // the corridor's freeway/ramp edges natively, so blocks under a deck
    // become open/utility space instead of clipped buildings.
    engine::RoadGraph freewayROW;
    for (const engine::RoadEntity& net2 : preNets) {
        engine::RoadGraph g2 = engine::navRoadGraph(net2, levelGround);
        const int base = static_cast<int>(freewayROW.nodes.size());
        for (const engine::RoadNode& n : g2.nodes)
            freewayROW.nodes.push_back(n);
        for (engine::RoadEdge e : g2.edges) {
            if (e.klass != engine::RoadClass::Freeway &&
                e.klass != engine::RoadClass::Ramp)
                continue;
            e.a += base; e.b += base;
            freewayROW.edges.push_back(e);
        }
    }
    const engine::RoadGraph* freewayROWp =
        freewayROW.edges.empty() ? nullptr : &freewayROW;

    loadStage("road graph + ROW");
    // Terrain is parsed once into params + noise so vegetation can scatter on
    // the same surface it generates.
    GrownLots preLots;   // lots grown by the terrain pre-pass (reused below)
    if (root.contains("terrain")) {
        TerrainParams terrainParams = readTerrainParams(root["terrain"]);
        terrainParams.erodedBase = sharedEroded;   // eroded base for mesh + carve + drape
        // APPEND the script pre-pass records — assign() here silently wiped any
        // level-AUTHORED terrain.flatten records (walkway-lab round; the lab's
        // probe field caught it: perfect function/mesh agreement on a slope
        // with no stages in it).
        terrainParams.flatten.insert(terrainParams.flatten.end(),
                                     scriptFlatten.begin(), scriptFlatten.end());
        std::vector<TerrainFlatten> baseFlatten = terrainParams.flatten;   // non-road grading
        terrainParams.flatten.insert(terrainParams.flatten.end(),
                                     roadFlatten.begin(), roadFlatten.end());   // carve to roads
        unsigned terrainSeed = root["terrain"].value("seed", 0u);
        Noise terrainNoise(terrainSeed);
        // THE EARTHWORK FIELD (procgen/earthwork.h): the ground reshaped to
        // carry the road network, fitted to the road carve regions against the
        // NATURAL ground and installed into THESE params — the ones the lots,
        // pads, CDLOD, colliders and walls are built on — and NOT into the
        // shared `tp` behind levelGround, so the road solve and the load-time
        // road mesh keep reading natural ground (deck/carve parity). Sea-floor
        // cells are pinned so the shore never moves.
        {
            EarthworkStats es;
            const double sea = root.contains("water")
                                   ? root["water"].value("seaLevel", terrainParams.seaLevel)
                                   : terrainParams.seaLevel;
            terrainParams.earthwork = buildEarthworkField(
                roadFlatten, levelGround, terrainParams.earthworkParams, sea, &es);
            if (terrainParams.earthwork)
                LOG_INFO << "[earthwork] " << es.cells << " cells at " << es.cell
                         << " m (" << es.fixed << " fixed), reach "
                         << terrainParams.earthworkParams.reach << " m, max |D| "
                         << es.maxAbsD << " m at (" << es.maxAbsDX << "," << es.maxAbsDZ
                         << "), last sweep residual " << es.residual << " m, extent x["
                         << es.minX << "," << es.maxX << "] z[" << es.minZ << "," << es.maxZ << "]";
            else
                LOG_INFO << "[earthwork] off ("
                         << (terrainParams.earthworkParams.enabled ? "no road regions"
                                                                   : "disabled")
                         << ")";
        }
        // LOT PRE-PASS (device: "some of the buildings are sunk into the
        // terrain"): grow the living city's lots on the road-carved ground
        // BEFORE the terrain is meshed, so every building stamps a FLAT graded
        // pad (at its own plane) into the flatten set. The citysim build below
        // reuses these exact lots.
        loadStage("terrain field");
        // A lane-lab level has no road entities but publishes its BLOCKS before this point (from the
        // city bundle, above), and its lots must grow here too, or their pads never reach the terrain.
        const bool labBlocks =
#ifdef RT_ENABLE_LANELAB
            !g_lanelab.blocks.empty();
#else
            false;
#endif
        if (root.contains("citysim") &&
            (root["citysim"].value("buildLots", false) ||
             root["citysim"].value("planOnly", false)) &&
            (!preNets.empty() || labBlocks)) {
            auto lotTp = std::make_shared<TerrainParams>(terrainParams);
            auto lotNoise = std::make_shared<Noise>(terrainSeed);
            HeightField lotGround = [lotTp, lotNoise](double x, double z) {
                return terrainHeight(*lotTp, *lotNoise, x, z);
            };
            // Priority-correct rebind hook for the in-pass block grades
            // (LotParams::groundWith): fold extras into the SAME region list
            // as the roads so priorities resolve as the final terrain will.
            auto lotGroundWith = [lotTp, lotNoise](
                                     const std::vector<TerrainFlatten>& extra) {
                auto tp = std::make_shared<TerrainParams>(*lotTp);
                tp->flatten.insert(tp->flatten.end(), extra.begin(),
                                   extra.end());
                rebuildFlattenIndex(*tp);
                // Dilate-aware (third arg): the mesh-conforming walkway
                // sampler reproduces a CDLOD corner query exactly.
                return [tp, lotNoise](Real x, Real z, Real dilate) {
                    return terrainHeight(*tp, *lotNoise, x, z,
                                         static_cast<double>(dilate));
                };
            };
            engine::Vec2 spawnXZ;
            const bool haveSpawn = authoredSpawnXZ(root, spawnXZ);
            engine::bundle::LevelInputs lotInputs;
            lotInputs.levelPath = path; lotInputs.level = root; lotInputs.levelDir = levelDir;
            preLots = growCityLots(lotInputs, preNets, root["citysim"], levelDir, lotGround,
                                   levelGround, freewayROWp, lotGroundWith,
                                   lotMeshCell, haveSpawn ? &spawnXZ : nullptr);
            // BLOCK GRADING CASCADE (ADR-0075 P2, re-enabled roads-v2.1 R4):
            // the old attempt extracted faces from the GRAPH (none on a
            // tree-like terrain-gated metro); the LOT PLAN's own block
            // polygons are the real thing. Each block, DILATED to overlap
            // the road conform band (the road's higher priority wins inside
            // its own footprint; everywhere else the block plane owns the
            // ground, so no raw-terrain notch survives between the two — the
            // notch was the 'fell in the hole, can't get out' pit), blends
            // to the plane of its road-carved boundary. Kills three drive
            // findings at once: corralled pits, the sidewalk-outer-face
            // wall, and buildings floating off sloped interiors.
            // The grades were computed INSIDE the lot grower, between
            // parcelling and building growth, so every pad plane sampled the
            // terraced ground (the buried-buildings fix) — consume that one
            // derivation instead of re-fitting here.
            if (!preLots.gradeFlatten.empty()) {
                const std::vector<TerrainFlatten>& blockGrades =
                    preLots.gradeFlatten;
                LOG_INFO << "[grade] " << blockGrades.size()
                         << " block planes/terraces from "
                         << preLots.plan.blocks.size() << " blocks";
                std::vector<TerrainFlatten> grades = blockGrades;
#ifdef RT_ENABLE_LANELAB
                if (!g_lanelab.blocks.empty()) grades = engine::lanelab::lanelabTerraces(blockGrades, g_lanelab.blocks, g_lanelab.sidewalk);   // shrunk by the feather and clipped to the block: a terrace ends at the block line, not in the road
#endif
                terrainParams.flatten.insert(terrainParams.flatten.end(),
                                             grades.begin(),
                                             grades.end());
                // ...and into the NON-ROAD base too: the editor's Conform
                // Terrain action rebuilds flatten = baseFlatten + fresh roads,
                // and block grades missing from the base meant one press of G
                // silently reverted every block plane/terrace to raw noise
                // under the buildings.
                baseFlatten.insert(baseFlatten.end(), grades.begin(), grades.end());
            }
            // Pads OUTRANK roads (kPadFlattenPriority, terrain.h): the road conform half-width overlaps the
            // first metres of building depth, and priority is a hard override — below roads, every frontage
            // facade stood on the ROAD's plane. Road-graph levels keep pads off the carriageway by roadClear;
            // a lanelab level has no road graph here, so its pads are CLIPPED to their block and feathered
            // no further than the sidewalk (Glenn found a pad plane standing a metre proud inside a junction).
            {
                std::vector<TerrainFlatten> pads;
#ifdef RT_ENABLE_LANELAB
                if (!g_lanelab.blocks.empty()) pads = engine::lanelab::clipPadsToBlocks(preLots.lots, g_lanelab.blocks, g_lanelab.sidewalk);
                else
#endif
                for (const engine::LotBuilding& lb : preLots.lots) {
                    if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3) continue;
                    pads.push_back(engine::lotPadFlatten(lb));
                }
                for (TerrainFlatten& f : pads) {
                    f.priority = kPadFlattenPriority;
                    terrainParams.flatten.push_back(f);
                    baseFlatten.push_back(std::move(f));   // non-road grading
                }
            }
        }
        // Index the assembled cut/fill set (ADR-0075 Phase 0): the CDLOD mesher,
        // collider, and road drape all sample terrainHeight per vertex, so the
        // O(footprints) scan there dominates the build. Shared, so the carved
        // copies below reuse it.
        rebuildFlattenIndex(terrainParams);
        loadTerrain(terrainParams, terrainNoise, root["terrain"], world, assets);

        // ELEVATION MAPS (RT_ELEVATION_MAP=<prefix>, see writeElevationMaps):
        // natural vs final vs drawn, from an independent probe grid over the
        // CDLOD extent. RT_ELEVATION_MAP_CELL (default 4 m) sets the spacing;
        // RT_ELEVATION_MARK="x,z" draws a crosshair (a reported site).
        if (const char* prefix = std::getenv("RT_ELEVATION_MAP")) {
            const double half = root["terrain"].contains("cdlod")
                                    ? root["terrain"]["cdlod"].value("worldHalf", 1024.0)
                                    : 200.0;
            double cell = 4.0;
            if (const char* c = std::getenv("RT_ELEVATION_MAP_CELL")) cell = std::max(0.5, std::atof(c));
            const double pcell = lotMeshCell > 0.5 ? lotMeshCell : 2.0;
            auto drawn = [&](double x, double z) {
                auto corner = [&](double cx, double cz) {
                    return terrainHeight(terrainParams, terrainNoise, cx, cz, pcell * 1.45);
                };
                const double gx = std::floor(x / pcell) * pcell;
                const double gz = std::floor(z / pcell) * pcell;
                const double fx = (x - gx) / pcell, fz = (z - gz) / pcell;
                return corner(gx, gz) * (1 - fx) * (1 - fz) +
                       corner(gx + pcell, gz) * fx * (1 - fz) +
                       corner(gx, gz + pcell) * (1 - fx) * fz +
                       corner(gx + pcell, gz + pcell) * fx * fz;
            };
            std::vector<ElevationSurface> surfaces = {
                {"natural", [&](double x, double z) { return levelGround(x, z); }},
                {"final", [&](double x, double z) {
                     return terrainHeight(terrainParams, terrainNoise, x, z); }},
                {"drawn", drawn},
            };
            // The earthwork field alone (natural + D), so its diff against
            // natural IS the field — the heat map Glenn asked to see.
            if (terrainParams.earthwork)
                surfaces.push_back({"earthworked", [&](double x, double z) {
                    return levelGround(x, z) + (*terrainParams.earthwork)(x, z); }});
            // Road centrelines, so "where the land moved" reads against the roads.
            std::vector<std::vector<Vec2>> lines;
            for (const engine::RoadEntity& net : preNets)
                for (const UnionSpine& sp : engine::roadNetWeldSpines(
                         engine::roadNetConstrainedGraph(net, levelGround)))
                    lines.push_back(sp.points);
            double mx = 0, mz = 0; bool hasMark = false;
            if (const char* m = std::getenv("RT_ELEVATION_MARK"))
                hasMark = std::sscanf(m, "%lf,%lf", &mx, &mz) == 2;
            writeElevationMaps(prefix, half, cell, surfaces, lines, hasMark, mx, mz);
        }

        // GROUND PROBES (RT_GROUND_PROBES=1, walkway-lab instrument): a grid of
        // thin posts whose BASE sits exactly at terrainHeight — the analytic
        // claim, planted in the world. Mesh above the claim = post half-buried;
        // mesh below = post floats. Disagreement is visible at a glance and its
        // direction names the culprit. Emitted from the FINAL params (every
        // flatten in), the same field the CDLOD mesher samples.
        if (std::getenv("RT_GROUND_PROBES")) {
            const double half = root["terrain"].contains("cdlod")
                                    ? root["terrain"]["cdlod"].value("worldHalf", 1024.0)
                                    : 200.0;
            const double step = std::max(4.0, half * 2.0 / 96.0);
            // Self-measuring probes: each post also computes its delta against
            // the FINEST tile's interpolation (the surface the player stands
            // on) and wears the verdict — green flush, orange within a metre,
            // red beyond. The loader logs the histogram + worst offender, so
            // any level launch with probes prints a numeric adherence verdict.
            const double pcell = lotMeshCell > 0.5 ? lotMeshCell : 2.0;
            auto probeTile = [&](double x, double z) {
                auto corner = [&](double cx, double cz) {
                    return terrainHeight(terrainParams, terrainNoise, cx, cz,
                                         pcell * 1.45);
                };
                const double gx = std::floor(x / pcell) * pcell;
                const double gz = std::floor(z / pcell) * pcell;
                const double fx = (x - gx) / pcell, fz = (z - gz) / pcell;
                return corner(gx, gz) * (1 - fx) * (1 - fz) +
                       corner(gx + pcell, gz) * fx * (1 - fz) +
                       corner(gx, gz + pcell) * (1 - fx) * fz +
                       corner(gx + pcell, gz + pcell) * fx * fz;
            };
            int nOk = 0, nNear = 0, nOff = 0;
            double worstD = 0, worstX = 0, worstZ = 0;
            RenderMesh probes;
            for (double px = -half; px <= half; px += step)
                for (double pz = -half; pz <= half; pz += step) {
                    const double h = terrainHeight(terrainParams, terrainNoise, px, pz);
                    const double d = h - probeTile(px, pz);
                    if (std::fabs(d) > std::fabs(worstD)) {
                        worstD = d; worstX = px; worstZ = pz;
                    }
                    Vec3 verdictC;
                    if (std::fabs(d) <= 0.3) { verdictC = Vec3(0.15, 0.7, 0.2); ++nOk; }
                    else if (std::fabs(d) <= 1.0) { verdictC = Vec3(0.95, 0.6, 0.1); ++nNear; }
                    else { verdictC = Vec3(0.85, 0.12, 0.10); ++nOff; }
                    const Real s2 = 0.12, tall = 1.1;
                    const Vec3 red = verdictC, white(0.92, 0.92, 0.9);
                    // base half red (the judgment zone), top half white — four
                    // side quads per band (tops skipped; you read the BASE).
                    auto band = [&](Real y0, Real y1, const Vec3& c) {
                        const Vec3 p00(px - s2, y0, pz - s2), p10(px + s2, y0, pz - s2);
                        const Vec3 p11(px + s2, y0, pz + s2), p01(px - s2, y0, pz + s2);
                        const Vec3 q00(px - s2, y1, pz - s2), q10(px + s2, y1, pz - s2);
                        const Vec3 q11(px + s2, y1, pz + s2), q01(px - s2, y1, pz + s2);
                        MeshBuilder::emitQuad(probes, p00, p10, q10, q00, Vec3(0, 0, -1), c);
                        MeshBuilder::emitQuad(probes, p10, p11, q11, q10, Vec3(1, 0, 0), c);
                        MeshBuilder::emitQuad(probes, p11, p01, q01, q11, Vec3(0, 0, 1), c);
                        MeshBuilder::emitQuad(probes, p01, p00, q00, q01, Vec3(-1, 0, 0), c);
                    };
                    band(h, h + tall * 0.5, red);
                    band(h + tall * 0.5, h + tall, white);
                }
            if (!probes.vertices.empty()) {
                Entity pe = world.create();
                world.add<Transform>(pe, Transform{});
                world.add<PrevTransform>(pe, PrevTransform{Transform{}});
                Renderable pr;
                pr.material.albedo = Vec3(1, 1, 1);
                pr.material.roughness = 0.9f;
                pr.mesh = assets.acquireMesh(probes, "ground_probes");
                world.add<Renderable>(pe, pr);
                g_groundProbeReport = {nOk + nNear + nOff, nOk, nNear, nOff,
                                       worstD, worstX, worstZ};
                LOG_INFO << "[probes] " << probes.vertices.size() / 32
                         << " ground probes planted: " << nOk << " flush (<=0.3m), "
                         << nNear << " near (<=1m), " << nOff
                         << " off (>1m); worst " << worstD << " m at (" << worstX
                         << ", " << worstZ << ")";
            }
        }

        // AUTHORED WALKWAYS (walkway-lab): {"shape":"walkway","walkway":
        // {"a":[x,z],"b":[x,z],"width":W}} — a paved band that is a PURE
        // READER of the final ground, emitted here (after every flatten) and
        // sampled through the tile's own interpolation (finest cell, the
        // mesher's step*1.45 dilation) so it sits on the RENDERED mesh
        // wherever the mesh is continuous. The lab's S5 stages ride this.
        if (root.contains("entities")) {
            RenderMesh walks;
            const double cell = lotMeshCell > 0.5 ? lotMeshCell : 2.0;
            auto tileGy = [&](double x, double z) {
                auto corner = [&](double cx, double cz) {
                    return terrainHeight(terrainParams, terrainNoise, cx, cz,
                                         cell * 1.45);
                };
                const double gx = std::floor(x / cell) * cell;
                const double gz = std::floor(z / cell) * cell;
                const double fx = (x - gx) / cell, fz = (z - gz) / cell;
                return corner(gx, gz) * (1 - fx) * (1 - fz) +
                       corner(gx + cell, gz) * fx * (1 - fz) +
                       corner(gx, gz + cell) * (1 - fx) * fz +
                       corner(gx + cell, gz + cell) * fx * fz;
            };
            for (const auto& ent : root["entities"]) {
                if (ent.value("shape", std::string()) != "walkway") continue;
                const json wb = ent.contains("walkway") ? ent["walkway"] : json::object();
                // Either a single span {a, b} or a polyline {points: [[x,z]..]}.
                std::vector<Vec2> pts;
                if (wb.contains("points"))
                    for (const auto& q : wb["points"])
                        if (q.is_array() && q.size() >= 2)
                            pts.emplace_back(q[0].get<double>(), q[1].get<double>());
                if (pts.size() < 2 && wb.contains("a") && wb.contains("b")) {
                    pts = {Vec2(wb["a"][0].get<double>(), wb["a"][1].get<double>()),
                           Vec2(wb["b"][0].get<double>(), wb["b"][1].get<double>())};
                }
                if (pts.size() < 2) continue;
                const double hw = wb.value("width", 2.0) * 0.5;
                const Vec3 pave(0.62, 0.61, 0.58);
                for (std::size_t pi = 0; pi + 1 < pts.size(); ++pi) {
                const Vec2 A = pts[pi], B = pts[pi + 1];
                const Vec2 d = B - A;
                const double L = d.length();
                if (L < 1.0) continue;
                const Vec2 dir = d * (1.0 / L);
                const Vec2 perp(-dir.y, dir.x);
                const int segs = std::max(1, static_cast<int>(L / std::min(3.0, cell)));
                for (int si = 0; si < segs; ++si) {
                    const Vec2 q0 = A + dir * (L * si / segs);
                    const Vec2 q1 = A + dir * (L * (si + 1) / segs);
                    const Vec2 l0 = q0 - perp * hw, r0 = q0 + perp * hw;
                    const Vec2 l1 = q1 - perp * hw, r1 = q1 + perp * hw;
                    MeshBuilder::emitQuad(
                        walks,
                        Vec3(l0.x, tileGy(l0.x, l0.y) + 0.05, l0.y),
                        Vec3(r0.x, tileGy(r0.x, r0.y) + 0.05, r0.y),
                        Vec3(r1.x, tileGy(r1.x, r1.y) + 0.05, r1.y),
                        Vec3(l1.x, tileGy(l1.x, l1.y) + 0.05, l1.y),
                        Vec3(0, 1, 0), pave);
                }
                }
            }
            if (!walks.vertices.empty()) {
                Entity we2 = world.create();
                world.add<Transform>(we2, Transform{});
                world.add<PrevTransform>(we2, PrevTransform{Transform{}});
                Renderable wr2;
                wr2.material.albedo = Vec3(1, 1, 1);
                wr2.material.roughness = 0.92f;
                wr2.mesh = assets.acquireMesh(walks, "lab_walkways");
                world.add<Renderable>(we2, wr2);
                LOG_INFO << "[walkway-lab] authored walkway bands emitted";
            }
        }
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
            engine::WaterMeshParams wp = readWaterParams(w);
            // The water's floor is the EARTHWORKED ground: the field can lift a
            // shore cell or (pinned at the sea floor, never below it) — the
            // mesh must read the ground the player sees, not the pre-earthwork
            // natural. Sea-floor cells are fixed at D = 0, so basins that were
            // wet stay wet.
            HeightSampler waterFloor = levelGround;
            if (terrainParams.earthwork) {
                auto ew = terrainParams.earthwork;
                auto nat = levelGround;
                waterFloor = [ew, nat](double x, double z) { return nat(x, z) + (*ew)(x, z); };
            }
            RenderMesh wmesh = engine::buildWaterMesh(waterFloor, wp);
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
        // Retaining walls (roads-v2.1 R6a): built against the FULLY CARVED
        // ground — base terrain + road conform + block grading — never the
        // raw hill. Sampling the base terrain stood walls against blocks the
        // city had already graded level (hillcity smoke: a wall fencing a
        // flat lawn). Co-designed with the smoothstep conform: a wall fronts
        // each DEEP cut face the feather leaves near-vertical, with a
        // backfill bench capping the feather dip behind it; minWall 3.5
        // keeps shallow grassy cuts open.
        {
            auto wallTp = std::make_shared<TerrainParams>(terrainParams);
            auto wallNoise = std::make_shared<Noise>(terrainSeed);
            StructureParams wp;
            wp.minWall = 3.5;
            const HeightField wallGround = [wallTp, wallNoise](double x, double z) {
                return terrainHeight(*wallTp, *wallNoise, x, z);
            };
            for (const engine::RoadEntity& pn : preNets) {
                StructureSet ws = buildRoadWalls(pn, wallGround, wp);
                if (!ws.empty()) {
                    LOG_INFO << "[walls] " << ws.walls.size()
                             << " retaining segments";
                    MeshBuilder::append(roadWallMesh, ws.mesh);
                }
            }
        }
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
        //
        // PLACEMENT SAMPLES THE MESH'S SURFACE, not the raw field: the CDLOD
        // mesher grows every flatten footprint by its per-node step*1.45
        // (terrain_lod.cpp) — sampling with dilate 0 planted trees and poles on
        // a surface up to metres away from the one actually drawn near any
        // road/pad/grade edge (Glenn: "trees and stop lights float off the
        // ground, especially on hilly areas"). Match the LEAF step's dilate —
        // the finest mesh and the collider both use it, so what stands on this
        // sample stands on what the player sees and drives on. (Coarse-LOD
        // tiles dilate wider still; that residual is a far-field-only drift.)
        double placeDilate = 0.0;
        world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) {
            const double leafSize =
                (2.0 * c.worldHalf) / static_cast<double>(1 << (c.numLods - 1));
            placeDilate = (leafSize / std::max(1, c.gridRes)) * 1.45;
        });
        auto carvedTp = std::make_shared<TerrainParams>(terrainParams);
        auto carvedNoise = std::make_shared<Noise>(terrainSeed);
        entityGround = [carvedTp, carvedNoise, placeDilate](double x, double z) {
            return terrainHeight(*carvedTp, *carvedNoise, x, z, placeDilate);
        };
        if (root.contains("vegetation"))
            loadVegetation(root["vegetation"], terrainParams, terrainNoise, world,
                           renderer, assets, levelDir, "veg",
                           preLots.grown ? &preLots.lots : nullptr, placeDilate);
        // A second, denser pass for ground cover (grass/flowers). Same scatter
        // generator with its own params — typically a low maxSlopeDeg so it lands
        // on the gentle, green ground (terrainColor reads steep slopes as rock).
        if (root.contains("foliage"))
            loadVegetation(root["foliage"], terrainParams, terrainNoise, world,
                           renderer, assets, levelDir, "foliage", nullptr,
                           placeDilate);
    }

    if (root.contains("entities"))
    // RT_POKE_REPORT=1: measure the REAL renderer's ground against the decks.
    // The headless probe approximated the CDLOD sample as "terrainHeight at the
    // test point with a dilated footprint" — but the device interpolates BETWEEN
    // GRID CORNERS, and a corner just past the dilation sits on natural ground,
    // tilting its triangle up through the deck. This report uses the loader's
    // FINAL flatten set and the exact per-LOD grid formula (size/step/dilate as
    // generateLodNodeMesh), so its numbers are the game's numbers.
    if (std::getenv("RT_POKE_REPORT") && root.contains("terrain")) {
        TerrainParams tpFull;   // the FINAL carved params (with index)
        int pNumLods = 6, pGridRes = 32;
        world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) {
            tpFull = c.params;
            pNumLods = c.numLods;
            pGridRes = c.gridRes;
        });
        Noise pnNoise(root["terrain"].value("seed", 0u));
        float pWorldHalf = root["terrain"]["cdlod"].value("worldHalf", 1024.0);
        for (int lvl = 0; lvl < 3; ++lvl) {
            const double nodeSize = pWorldHalf * 2.0 / std::pow(2.0, pNumLods - 1 - lvl);
            const double step = nodeSize / pGridRes;
            const double dil = step * 1.45;   // mirror generateLodNodeMesh
            auto gridH = [&](int gi, int gj) {
                double y = terrainHeight(tpFull, pnNoise, gi * step, gj * step, dil);
                if (tpFull.flattenIndex) {   // mirror generateLodNodeMesh's Fix A clamp
                    const double rp = roadPlaneNear(*tpFull.flattenIndex, tpFull.flatten,
                                                    gi * step, gj * step, step * 1.6);
                    if (rp < 1e29 &&
                        !padPlaneAbove(*tpFull.flattenIndex, tpFull.flatten,
                                       gi * step, gj * step, rp))
                        y = std::min(y, rp);
                }
                return y;
            };
            // DENSE poke map (device feedback: point samples between verts miss
            // pokes — cover the whole deck area; heightfield-vs-heightfield on a
            // dense grid IS the ray-cast-down). Deck height comes from the SAME
            // reconciled chain profiles the mesher rides, not the carve proxy.
            long n = 0, poke = 0, pokeCovered = 0, pokeHole = 0;
            double worst = 0, wx = 0, wz = 0;
            for (const engine::RoadEntity& net : preNets) {
                std::vector<UnionSpine> spines = engine::roadNetWeldSpines(
                    engine::roadNetConstrainedGraph(net, levelGround));
                // The SAME arguments the carve passes (road_net.cpp
                // roadNetConformRegions), or this report measures a different
                // deck than the one the terrain was cut to.
                const engine::DesignRules pokeRules;
                std::vector<std::vector<double>> profs = engine::weldChainProfiles(
                    spines, levelGround, 0.0, /*maxGrade=*/0.08,
                    net.look.sidewalk + 4.0,
                    net.look.perClassGrade ? &pokeRules : nullptr);
                if (std::getenv("RT_POKE_SITE") || std::getenv("RT_JUNCTION_DUMP")) {
                    RoadGraph gFp =
                        engine::roadNetConstrainedGraph(net, levelGround);
                    double fp = 0;
                    for (std::size_t si2 = 0; si2 < spines.size(); ++si2)
                        fp += spines[si2].points.front().x * (si2 + 1) * 1e-3;
                    LOG_INFO << "[report-fingerprint] nodes=" << gFp.nodes.size()
                             << " edges=" << gFp.edges.size()
                             << " spines=" << spines.size() << " fp=" << fp;
                }
                // Segment spatial hash: the DECK at a point is the NEAREST
                // spine's profile (weldSolid::heightOf) — comparing a sample
                // against its OWN chain's profile miscounted junction overlaps
                // where a closer, higher chain owns the surface (the immortal
                // 0.9432 m "poke" was this instrument error, not terrain).
                struct SegRef { int si, i; };
                std::map<std::pair<int, int>, std::vector<SegRef>> segGrid;
                const double segCell = 25.0;
                auto cellOf = [&](double x, double z) {
                    return std::make_pair((int)std::floor(x / segCell),
                                          (int)std::floor(z / segCell));
                };
                for (int si2 = 0; si2 < (int)spines.size(); ++si2) {
                    if (profs[si2].size() < 2) continue;
                    const auto& pp = spines[si2].points;
                    for (int i2 = 0; i2 + 1 < (int)pp.size(); ++i2) {
                        const auto c0 = cellOf(std::min(pp[i2].x, pp[i2 + 1].x),
                                               std::min(pp[i2].y, pp[i2 + 1].y));
                        const auto c1 = cellOf(std::max(pp[i2].x, pp[i2 + 1].x),
                                               std::max(pp[i2].y, pp[i2 + 1].y));
                        for (int gx = c0.first; gx <= c1.first; ++gx)
                            for (int gz = c0.second; gz <= c1.second; ++gz)
                                segGrid[{gx, gz}].push_back({si2, i2});
                    }
                }
                auto deckNearest = [&](const Vec2& q, double fallback) {
                    double bestD2 = 1e30, h = fallback;
                    const auto cq = cellOf(q.x, q.y);
                    for (int gx = cq.first - 1; gx <= cq.first + 1; ++gx)
                        for (int gz = cq.second - 1; gz <= cq.second + 1; ++gz) {
                            auto it = segGrid.find({gx, gz});
                            if (it == segGrid.end()) continue;
                            for (const SegRef& sr : it->second) {
                                const auto& pp = spines[sr.si].points;
                                Vec2 ab = pp[sr.i + 1] - pp[sr.i];
                                double L2 = ab.lengthSquared();
                                double tt = L2 < 1e-12 ? 0.0
                                    : std::max(0.0, std::min(1.0, dot(q - pp[sr.i], ab) / L2));
                                double d2 = (q - (pp[sr.i] + ab * tt)).lengthSquared();
                                if (d2 < bestD2) {
                                    bestD2 = d2;
                                    h = profs[sr.si][sr.i] +
                                        (profs[sr.si][sr.i + 1] - profs[sr.si][sr.i]) * tt;
                                }
                            }
                        }
                    return h;
                };
                // JUNCTION DUMP (RT_JUNCTION_DUMP="x,z,radius,path"): every
                // surface that could carry a step at one junction, at the
                // SAME stations, so the step is attributed to ONE of them
                // instead of argued about. Per chain with an endpoint within
                // `radius` of (x,z), 1 m stations from the node outward to
                // radius+40: natural ground, the solved deck P, the nearest-
                // spine deck the pad rides, the carve target (road regions
                // only), the final analytic terrain, and the LOD0 tile's own
                // bilinear interpolation. Plus `nodeSpread`: the max arm-to-arm
                // difference of the solved endpoints sharing that node key —
                // the deck profile is grade-clamped per pair (lower-only ease,
                // weldChainProfiles), so a cliff at a junction can only be arms
                // DISAGREEING at the node and the pad chording across it.
                // Written once, at LOD 0. Device: "that road T-junction is
                // really bad ... it creates this bad dip in the road".
                if (lvl == 0 && std::getenv("RT_JUNCTION_DUMP")) {
                    double jx = 0, jz = 0, jr = 30;
                    char jpath[512] = {0};
                    if (std::sscanf(std::getenv("RT_JUNCTION_DUMP"), "%lf,%lf,%lf,%511s",
                                    &jx, &jz, &jr, jpath) == 4) {
                        auto nodeKey = [](const Vec2& v) {
                            return std::make_pair(
                                static_cast<long long>(std::llround(v.x * 8)),
                                static_cast<long long>(std::llround(v.y * 8)));
                        };
                        // Arm endpoint heights per node key, for nodeSpread.
                        std::map<std::pair<long long, long long>,
                                 std::vector<std::pair<int, double>>> arms;
                        for (int s2 = 0; s2 < (int)spines.size(); ++s2) {
                            if (profs[s2].size() < 2 || spines[s2].closed) continue;
                            const auto& pp = spines[s2].points;
                            arms[nodeKey(pp.front())].push_back({s2, profs[s2].front()});
                            arms[nodeKey(pp.back())].push_back({s2, profs[s2].back()});
                        }
                        std::ofstream jf(jpath, std::ios::app);
                        jf << "# junction dump at (" << jx << "," << jz << ") r=" << jr
                           << " net.sidewalk=" << net.look.sidewalk << "\n";
                        const Vec2 J(jx, jz);
                        for (const auto& kv : arms) {
                            const Vec2 nv(kv.first.first / 8.0, kv.first.second / 8.0);
                            if ((nv - J).length() > jr) continue;
                            double lo = 1e30, hi = -1e30;
                            for (const auto& a : kv.second) {
                                lo = std::min(lo, a.second);
                                hi = std::max(hi, a.second);
                            }
                            // Per arm: the flags that EXEMPT a chain from the
                            // node fold and from the carve (authored deck /
                            // layer) — an at-grade road wearing them floats.
                            std::ostringstream armsTxt;
                            for (const auto& a : kv.second) {
                                const UnionSpine& sp2 = spines[a.first];
                                armsTxt << " chain" << a.first << "=" << a.second
                                        << "{klass=" << static_cast<int>(sp2.klass)
                                        << " yAbs=" << sp2.yAbs.size()
                                        << " authoredDeck=" << sp2.authoredDeck
                                        << " layer=" << sp2.layer
                                        << " closed=" << sp2.closed
                                        << " pts=" << sp2.points.size() << "}";
                            }
                            jf << "# node (" << nv.x << "," << nv.y << ") arms=" << kv.second.size()
                               << " nodeSpread=" << (hi - lo) << " [" << armsTxt.str() << " ]\n";
                            LOG_INFO << "[junction-dump] node (" << nv.x << "," << nv.y
                                     << ") arms=" << kv.second.size()
                                     << " nodeSpread=" << (hi - lo) << " m ["
                                     << armsTxt.str() << " ]";
                        }
                        jf << "chain,fromFront,s,x,z,natural,deckP,deckNearest,"
                              "carveTarget,finalAnalytic,lod0Tile,klass\n";
                        // The LOD0 tile's bilinear interpolation of gridH.
                        auto tile0 = [&](double x, double z) {
                            const double gx = std::floor(x / step), gz = std::floor(z / step);
                            const double fx = x / step - gx, fz = z / step - gz;
                            const int gi = (int)gx, gj = (int)gz;
                            return gridH(gi, gj) * (1 - fx) * (1 - fz) +
                                   gridH(gi + 1, gj) * fx * (1 - fz) +
                                   gridH(gi, gj + 1) * (1 - fx) * fz +
                                   gridH(gi + 1, gj + 1) * fx * fz;
                        };
                        for (int s2 = 0; s2 < (int)spines.size(); ++s2) {
                            if (profs[s2].size() < 2) continue;
                            const auto& pp = spines[s2].points;
                            const int n2 = (int)pp.size();
                            for (int fromFront = 1; fromFront >= 0; --fromFront) {
                                const Vec2& end = fromFront ? pp.front() : pp.back();
                                if ((end - J).length() > jr) continue;
                                // Walk 1 m stations from this end along the chain.
                                double sAcc = 0.0;
                                int seg = fromFront ? 0 : n2 - 2;
                                double segT = 0.0;
                                for (double s = 0.0; s <= jr + 40.0; s += 1.0) {
                                    // advance to station s
                                    while (true) {
                                        const Vec2 a = fromFront ? pp[seg] : pp[seg + 1];
                                        const Vec2 b = fromFront ? pp[seg + 1] : pp[seg];
                                        const double L = (b - a).length();
                                        if (sAcc + L * (1.0 - segT) >= s || L < 1e-9) {
                                            const double need = s - sAcc;
                                            const double t2 = L < 1e-9 ? 0.0
                                                : std::min(1.0, segT + need / L);
                                            const Vec2 q = a + (b - a) * t2;
                                            const double pa = fromFront ? profs[s2][seg]
                                                                        : profs[s2][seg + 1];
                                            const double pb = fromFront ? profs[s2][seg + 1]
                                                                        : profs[s2][seg];
                                            const double P = pa + (pb - pa) * t2;
                                            const double nat = levelGround(q.x, q.y);
                                            const double carve =
                                                applyFlatten(roadFlatten, q.x, q.y, nat);
                                            const double fin =
                                                terrainHeight(tpFull, pnNoise, q.x, q.y);
                                            jf << s2 << "," << fromFront << "," << s << ","
                                               << q.x << "," << q.y << "," << nat << ","
                                               << P << "," << deckNearest(q, P) << ","
                                               << carve << "," << fin << ","
                                               << tile0(q.x, q.y) << ","
                                               << static_cast<int>(spines[s2].klass) << "\n";
                                            break;
                                        }
                                        sAcc += L * (1.0 - segT);
                                        segT = 0.0;
                                        if (fromFront ? (++seg >= n2 - 1) : (--seg < 0)) {
                                            seg = -1; break;
                                        }
                                    }
                                    if (seg < 0) break;
                                }
                            }
                        }
                        LOG_INFO << "[junction-dump] wrote " << jpath;
                    }
                }
                for (std::size_t si = 0; si < spines.size(); ++si) {
                    const auto& pts = spines[si].points;
                    if (profs[si].size() < 2) continue;
                    const double hw = spines[si].halfWidth + net.look.sidewalk - 0.3;
                    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                        Vec2 d2v = pts[i + 1] - pts[i];
                        const double L = d2v.length();
                        if (L < 1e-9) continue;
                        d2v = d2v * (1.0 / L);
                        const Vec2 nrm(-d2v.y, d2v.x);
                        const int segs = std::max(1, (int)(L / 2.0));
                        const int lats = std::max(2, (int)(hw / 1.5));
                        for (int s = 0; s <= segs; ++s) {
                            const double t = (double)s / segs;
                            const Vec2 qc = pts[i] + (pts[i + 1] - pts[i]) * t;
                            const double ownDeck =
                                profs[si][i] + (profs[si][i + 1] - profs[si][i]) * t +
                                0.0;
                            for (int li = -lats; li <= lats; ++li) {
                                const Vec2 q = qc + nrm * (hw * li / (double)lats);
                                const double deck = deckNearest(q, ownDeck) + net.look.lift;
                                const double fx = q.x / step, fz = q.y / step;
                                const int gi = (int)std::floor(fx), gj = (int)std::floor(fz);
                                const double u = fx - gi, v = fz - gj;
                                const double gh = gridH(gi, gj) * (1 - u) * (1 - v) +
                                                  gridH(gi + 1, gj) * u * (1 - v) +
                                                  gridH(gi, gj + 1) * (1 - u) * v +
                                                  gridH(gi + 1, gj + 1) * u * v;
                                ++n;
                                const double dd = gh - deck;
                                // RT_POKE_SITE="x,z": full autopsy of samples
                                // within 3 m of the given point — deck source,
                                // corner heights, and what the corner clamp saw.
                                if (const char* site = std::getenv("RT_POKE_SITE")) {
                                    double sx = 0, sz = 0;
                                    if (sscanf(site, "%lf,%lf", &sx, &sz) == 2 &&
                                        std::hypot(q.x - sx, q.y - sz) < 3.0 && lvl == 0) {
                                        LOG_INFO << "[poke-site] q=(" << q.x << "," << q.y
                                                 << ") gh=" << gh << " deck=" << deck
                                                 << " ownDeck=" << ownDeck
                                                 << " dd=" << dd << " chain=" << si
                                                 << " hw=" << spines[si].halfWidth;
                                        // every priority-1 region covering q (dilated):
                                        for (std::size_t ri = 0; ri < tpFull.flatten.size(); ++ri) {
                                            const TerrainFlatten& r = tpFull.flatten[ri];
                                            if (r.priority != kRoadFlattenPriority || r.polygon.size() < 3) continue;
                                            if (q.x < r.minX - dil || q.x > r.maxX + dil ||
                                                q.y < r.minZ - dil || q.y > r.maxZ + dil) continue;
                                            auto inPoly = [&](double px, double pz) {
                                                bool in = false;
                                                size_t np = r.polygon.size();
                                                for (size_t a = 0, b = np - 1; a < np; b = a++) {
                                                    double xi = r.polygon[a].x, zi = r.polygon[a].z;
                                                    double xj = r.polygon[b].x, zj = r.polygon[b].z;
                                                    if (((zi > pz) != (zj > pz)) &&
                                                        (px < (xj - xi) * (pz - zi) / (zj - zi) + xi))
                                                        in = !in;
                                                }
                                                return in;
                                            };
                                            auto distPoly = [&](double px, double pz) {
                                                double bd = 1e30;
                                                size_t np = r.polygon.size();
                                                for (size_t a = 0, b = np - 1; a < np; b = a++) {
                                                    double ax = r.polygon[b].x, az = r.polygon[b].z;
                                                    double ex = r.polygon[a].x - ax,
                                                           ez = r.polygon[a].z - az;
                                                    double l2 = ex * ex + ez * ez;
                                                    double tt = l2 > 1e-12
                                                        ? std::max(0.0, std::min(1.0,
                                                              ((px - ax) * ex + (pz - az) * ez) / l2))
                                                        : 0.0;
                                                    double ddx = px - (ax + ex * tt),
                                                           ddz = pz - (az + ez * tt);
                                                    bd = std::min(bd, std::sqrt(ddx * ddx + ddz * ddz));
                                                }
                                                return bd;
                                            };
                                            const bool insideR = inPoly(q.x, q.y);
                                            if (!insideR && distPoly(q.x, q.y) > dil) continue;
                                            LOG_INFO << "[poke-site]   region#" << ri
                                                     << " owner=" << r.owner
                                                     << " plane=" << r.planeY(q.x, q.y)
                                                     << (insideR ? " INSIDE" : " (dilated)");
                                        }
                                        int mine = 0;
                                        for (std::size_t ri = 0; ri < tpFull.flatten.size(); ++ri) {
                                            const TerrainFlatten& r = tpFull.flatten[ri];
                                            if (r.owner != (int)si) continue;
                                            ++mine;
                                            if (std::abs((r.minX + r.maxX) * 0.5 - q.x) < 40 &&
                                                std::abs((r.minZ + r.maxZ) * 0.5 - q.y) < 40)
                                                LOG_INFO << "[poke-site]   my-chain region#"
                                                         << ri << " bbox x["
                                                         << r.minX << "," << r.maxX << "] z["
                                                         << r.minZ << "," << r.maxZ
                                                         << "] plane@q=" << r.planeY(q.x, q.y);
                                        }
                                        LOG_INFO << "[poke-site]   chain " << si
                                                 << " owns " << mine << " regions total";
                                        for (int cj = 0; cj <= 1; ++cj)
                                            for (int ci = 0; ci <= 1; ++ci) {
                                                const double cxw = (gi + ci) * step;
                                                const double czw = (gj + cj) * step;
                                                const double raw = terrainHeight(
                                                    tpFull, pnNoise, cxw, czw, dil);
                                                double rp = 1e30;
                                                if (tpFull.flattenIndex)
                                                    rp = roadPlaneNear(*tpFull.flattenIndex,
                                                                       tpFull.flatten, cxw,
                                                                       czw, step * 1.6);
                                                const bool cov = tpFull.flattenIndex &&
                                                    flattenCovers(*tpFull.flattenIndex,
                                                                  tpFull.flatten, cxw, czw,
                                                                  dil);
                                                LOG_INFO << "[poke-site]   corner("
                                                         << cxw << "," << czw
                                                         << ") raw=" << raw
                                                         << " roadPlane="
                                                         << (rp < 1e29 ? rp : -999)
                                                         << " covered=" << cov;
                                            }
                                    }
                                }
                                if (dd > 0.05) {
                                    ++poke;
                                    // RT_POKE_DUMP=<path>: every poke site with
                                    // the owning chain's sample count, so a
                                    // count change can be attributed by place
                                    // and by chain shape (e.g. two-point
                                    // chains that were never graded before).
                                    if (const char* pd = std::getenv("RT_POKE_DUMP")) {
                                        static std::ofstream pf(pd);
                                        // endl: the viewer is usually killed,
                                        // not exited, so the tail must be on
                                        // disk already (a buffered run lost the
                                        // last ~125 LOD2 sites).
                                        pf << lvl << "," << q.x << "," << q.y << ","
                                           << dd << "," << si << ","
                                           << spines[si].points.size() << std::endl;
                                    }
                                    if (dd > worst) { worst = dd; wx = q.x; wz = q.y; }
                                    if (dd > 0.3) {
                                        const bool covered = tpFull.flattenIndex &&
                                            flattenCovers(*tpFull.flattenIndex,
                                                          tpFull.flatten, q.x, q.y, dil);
                                        if (covered) ++pokeCovered; else ++pokeHole;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            g_pokeReport.lods = lvl + 1;
            g_pokeReport.samples[lvl] = n;
            g_pokeReport.pokes[lvl] = poke;
            g_pokeReport.worst[lvl] = worst;
            g_pokeReport.worstX[lvl] = wx;
            g_pokeReport.worstZ[lvl] = wz;
            LOG_INFO << "[poke-report] LOD " << lvl << " (step " << step << "m): "
                     << poke << "/" << n << " samples poke ("
                     << (n ? 100.0 * poke / n : 0.0) << "%), worst " << worst
                     << "m @ (" << wx << "," << wz << ")  [>0.3m: "
                     << pokeCovered << " inside a footprint, " << pokeHole
                     << " coverage holes]";
        }
    }
    {
        // Hand the pre-pass road nets to the entity pass so the recipe runs
        // ONCE per road: the entity reuses the exact network the lots and the
        // terrain carve were built against (see loadRoadEntity).
        std::vector<std::pair<const json*, RoadEntity>> roadCache;
        roadCache.reserve(preNets.size());
        for (std::size_t i = 0; i < preNets.size(); ++i)
            roadCache.emplace_back(preNetEnts[i], std::move(preNets[i]));
#ifdef RT_ENABLE_LANELAB
        g_lanelab.sidewalk = root.contains("citysim") && root["citysim"].is_object() ? root["citysim"].value("sidewalk", 4.0) : 4.0;
#endif
        loadStage("terrain + lot pre-pass");
        loadEntities(root["entities"], root, world, renderer, assets, levelDir,
                     editorMode, &scriptCache,
                     entityGround ? &entityGround : nullptr,
                     roadCache.empty() ? nullptr : &roadCache,
                     levelGround ? &levelGround : nullptr);
#ifdef RT_ENABLE_LANELAB
        // A lab level has no terrain entity: the lanelab grid is the ground the lots grade
        // on and the building pads sample (published by loadLaneLabEntity).
        if (!entityGround && g_lanelab.ground) entityGround = g_lanelab.ground;
        // The ONE derived road graph (§10) for a lab level: the lane lab's twin, sampled the way
        // every road entity is, so the city map (`citymap`), street furniture and the citysim nav
        // read the lab's streets through the same component as any other level's. Only when the
        // level published none of its own.
        // ONLY for a level that actually loaded a lanelab entity: this walks every RoadEntity in the
        // world, so without the ordinal test it also fires on a plain shape:"road" level that published
        // no graph of its own — handing it street furniture and a city map it does not have on a build
        // with the hook off. agent_lab and small_town caught this.
        if (g_lanelab.ordinal > 0) {
            bool have = false; world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph&) { have = true; });
            if (!have) {
                engine::LevelRoadGraph lrg;
                world.each<engine::RoadEntity>([&](Entity, engine::RoadEntity& net) {
                    engine::RoadGraph g = engine::navRoadGraph(net, entityGround);
                    const int base = static_cast<int>(lrg.graph.nodes.size());
                    for (const engine::RoadNode& n : g.nodes) lrg.graph.nodes.push_back(n);
                    for (engine::RoadEdge e : g.edges) { e.a += base; e.b += base; lrg.graph.edges.push_back(e); }
                });
                if (!lrg.graph.edges.empty()) {
                    LOG_INFO << "[lanelab] unified road graph from the road twin: " << lrg.graph.nodes.size() << " nodes, " << lrg.graph.edges.size() << " edges";
                    world.add<engine::LevelRoadGraph>(world.create(), std::move(lrg));
                }
            }
        }
#endif
    // (2e: the corridor document entity carries no mesh — the freeway IS the
    // road entity's mesh, built from the baked graph by the one mesher.)

    }

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
        // How far parked scenery cars still draw (0 = never cull).
        cfg.sceneryRadius = cs.value("sceneryRadius", cfg.sceneryRadius);
        cfg.maxWalkerBodies = cs.value("maxWalkerBodies", cfg.maxWalkerBodies);
        cfg.localHz = cs.value("localHz", cfg.localHz);
        cfg.adaptiveRate = cs.value("adaptiveRate", cfg.adaptiveRate);
        cfg.carsPerLaneKm = cs.value("carsPerLaneKm", cfg.carsPerLaneKm);
        cfg.pedsPerKm = cs.value("pedsPerKm", cfg.pedsPerKm);
        cfg.maxAmbient = cs.value("maxAmbient", cfg.maxAmbient);
        cfg.seed = cs.value("seed", cfg.seed);
        cfg.hoursPerSecond = cs.value("hoursPerSecond", cfg.hoursPerSecond);
        // ONE CLOCK, ONE AUTHOR (Glenn, 2026-09-16: "use the same tick for
        // day/night and city simulation so it all syncs up"). At RUNTIME they
        // already share the sky's tick: DayNightSystem stages the hour and the
        // rate, and CityRenderSystem::setWorldClock pushes both into the sim.
        // But the LEVEL could author the rate twice -- "dayMinutes" for the sky
        // and "hoursPerSecond" for the city -- and only the sky's was honoured.
        // Worse, a headless context that runs no DayNightSystem (level_tests,
        // the commute census) fell back to the citysim default and reported a
        // day length the game never runs.
        //
        // So the authored day is now the ONE number: derive the sim's rate from
        // it, and say so when a level still carries the old knob.
        {
            const bool cycleRuns = !root.contains("dayNight") ||
                                   !root["dayNight"].is_object() ||
                                   root["dayNight"].value("enabled", true);
            double dayMinutes = -1.0;
            if (root.contains("dayNight") && root["dayNight"].is_object())
                dayMinutes = root["dayNight"].value("dayMinutes", -1.0);
            if (cycleRuns && dayMinutes > 0.0) {
                const float derived = static_cast<float>(24.0 / (dayMinutes * 60.0));
                if (cs.contains("hoursPerSecond") &&
                    std::fabs(derived - cfg.hoursPerSecond) > 1e-6f)
                    LOG_WARN << "[citysim] \"hoursPerSecond\" " << cfg.hoursPerSecond
                             << " is OVERRIDDEN by the authored day: dayMinutes "
                             << dayMinutes << " -> " << derived
                             << " h/s. One clock, one author -- drop the citysim key.";
                cfg.hoursPerSecond = derived;
            } else if (cycleRuns && cs.contains("hoursPerSecond")) {
                LOG_WARN << "[citysim] \"hoursPerSecond\" " << cfg.hoursPerSecond
                         << " (a " << (cfg.hoursPerSecond > 0 ? 24.0 / (cfg.hoursPerSecond * 60.0) : 0.0)
                         << " minute day) is IGNORED while a day/night cycle runs: the sky's "
                            "clock drives the city's schedules. Author \"dayNight\": "
                            "{\"dayMinutes\": N} instead.";
            }
        }
        cfg.lightSpriteIn = cs.value("lightSpriteIn", cfg.lightSpriteIn);
        cfg.lightSphereOut = cs.value("lightSphereOut", cfg.lightSphereOut);
        cfg.lightRadius = cs.value("lightRadius", cfg.lightRadius);
        cfg.lightRange = cs.value("lightRange", cfg.lightRange);
        cfg.lightCount = cs.value("lightCount", cfg.lightCount);
        cfg.lampLightRadius = cs.value("lampLightRadius", cfg.lampLightRadius);
        cfg.lampLightCount = cs.value("lampLightCount", cfg.lampLightCount);
        cfg.lampLightRange = cs.value("lampLightRange", cfg.lampLightRange);
        cfg.lampGlowDistance = cs.value("lampGlowDistance", cfg.lampGlowDistance);
        cfg.startHour = cs.value("startHour", cfg.startHour);
        cfg.perceptionReliability =
            cs.value("perceptionReliability", cfg.perceptionReliability);
        cfg.debugWidgets = cs.value("debugWidgets", cfg.debugWidgets);
        cfg.tieredAgents = cs.value("tiered", cfg.tieredAgents);
        cfg.dormantAgents = cs.value("dormancy", cfg.dormantAgents);
        cfg.showPlan = cs.value("showPlan", false);
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
#ifdef RT_ENABLE_LANELAB
        // The lane lab's freeway and ramps as the routed right-of-way (ADR-0083): its RoadEntity
        // twin carries them by class, so the lot pass gets the per-class keep-out band and the
        // under-deck re-zoning instead of the mainline-proxy fallback. Ramps keep their own
        // class (declaring them Freeway re-zoned every block inside the frontage roads as
        // freeway shadow); the embankment they sit on is carried by the twin's edge WIDTH,
        // which the building-clearance check reads.
        if (freewayROW.edges.empty() && !g_lanelab.row.edges.empty()) {
            freewayROW = g_lanelab.row;
            if (!freewayROW.edges.empty()) {
                freewayROWp = &freewayROW;
                LOG_INFO << "[lanelab] freeway right-of-way for the lot pass: " << freewayROW.edges.size() << " carriageway + ramp segments from the road twin";
            }
        }
#endif
        // Grow buildings on the ROAD NETWORK's blocks (ADR-0066): the Living City
        // path — real roads (a shape:"road" `generate` recipe, the tech grown.json
        // uses) whose enclosed blocks become lots and REAL shape-grammar buildings
        // (floors/windows/roof, fitting the lot), each tagged as a place agents
        // start/end their schedules at. Runs on the RoadEntity(s) already in the world.
        if (!cs.value("buildLots", false) && cs.value("planOnly", false)) {
            // PLAN-ONLY (device: "show me the city blocks and then the
            // individual lots — no buildings, just the demarcation lines"):
            // grow the full block/lot plan and publish ONLY the outlines.
            GrownLots grown = std::move(preLots);
            if (!grown.grown) {
                std::vector<engine::RoadEntity> nets;
                world.each<engine::RoadEntity>(
                    [&](Entity, engine::RoadEntity& net) { nets.push_back(net); });
                engine::Vec2 spawnXZ2;
                const bool haveSpawn2 = authoredSpawnXZ(root, spawnXZ2);
                engine::bundle::LevelInputs lotInputs2;
                lotInputs2.levelPath = path; lotInputs2.level = root; lotInputs2.levelDir = levelDir;
                grown = growCityLots(lotInputs2, nets, cs, levelDir, entityGround,
                                     entityGround, freewayROWp, nullptr,
                                     lotMeshCell,
                                     haveSpawn2 ? &spawnXZ2 : nullptr);
            }
            if (!grown.plan.blocks.empty() || !grown.plan.lots.empty()) {
                engine::CityPlanDebug dbg;
                dbg.blocks = std::move(grown.plan.blocks);
                dbg.lots = std::move(grown.plan.lots);
                world.add<engine::CityPlanDebug>(world.create(), std::move(dbg));
                LOG_INFO << "[plan-only] " << dbg.blocks.size() << " blocks, "
                         << dbg.lots.size() << " lots (outlines only)";
            }
        }
        if (cs.value("buildLots", false)) {
            // The lots: grown by the terrain pre-pass (city-on-terrain — their
            // graded pads are already baked into the ground), or grown here for
            // a flat city. Same helper, same deterministic result.
            GrownLots grown = std::move(preLots);
            if (!grown.grown) {
                std::vector<engine::RoadEntity> nets;
                world.each<engine::RoadEntity>(
                    [&](Entity, engine::RoadEntity& net) { nets.push_back(net); });
                engine::Vec2 spawnXZ2;
                const bool haveSpawn2 = authoredSpawnXZ(root, spawnXZ2);
                engine::bundle::LevelInputs lotInputs2;
                lotInputs2.levelPath = path; lotInputs2.level = root; lotInputs2.levelDir = levelDir;
                grown = growCityLots(lotInputs2, nets, cs, levelDir, entityGround,
                                     entityGround, freewayROWp, nullptr,
                                     lotMeshCell,
                                     haveSpawn2 ? &spawnXZ2 : nullptr);
            }
            // Stage timings for the tail after the grow (what a bundle does not yet cover), one log line.
            const auto tLots0 = std::chrono::steady_clock::now(); auto tLotsPrev = tLots0; std::string lotStages;
            auto lotStage = [&](const char* name) {
                const auto now = std::chrono::steady_clock::now(); char buf[64];
                std::snprintf(buf, sizeof buf, "%s%s %.2f s", lotStages.empty() ? "" : ", ", name, std::chrono::duration<double>(now - tLotsPrev).count());
                lotStages += buf; tLotsPrev = now;
            };
            // RT_LOT_PLAN_SVG=<path>: the parceller's plan as a plain SVG (block
            // interiors, every lot, every built plan) for levels without a city
            // map — the lanelab lab levels — so "why is this block empty" is a
            // picture, not a log line.
            if (const char* planSvg = std::getenv("RT_LOT_PLAN_SVG")) {
                double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
                auto grow = [&](const engine::Poly2& poly) { for (const engine::Vec2& q : poly) { x0 = std::min(x0, q.x); y0 = std::min(y0, q.y); x1 = std::max(x1, q.x); y1 = std::max(y1, q.y); } };
                for (const auto& b : grown.plan.blocks) grow(b);
                for (const auto& l : grown.plan.lots) grow(l);
                if (x1 > x0 && y1 > y0) {
                    std::ofstream f(planSvg);
                    f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << x0 - 20 << " " << y0 - 20 << " " << (x1 - x0) + 40 << " " << (y1 - y0) + 40 << "'>\n";
                    f << "<rect x='" << x0 - 20 << "' y='" << y0 - 20 << "' width='" << (x1 - x0) + 40 << "' height='" << (y1 - y0) + 40 << "' fill='#eef0e6'/>\n";
                    auto poly = [&](const engine::Poly2& pl, const char* fill, const char* stroke, double w) {
                        f << "<polygon fill='" << fill << "' stroke='" << stroke << "' stroke-width='" << w << "' points='";
                        for (const engine::Vec2& q : pl) f << q.x << "," << q.y << " ";
                        f << "'/>\n";
                    };
                    for (const auto& b : grown.plan.blocks) poly(b, "#cfd8c0", "#44557a", 0.8);
                    for (const auto& l : grown.plan.lots) poly(l, "none", "#c0392b", 0.4);
                    for (const auto& lb : grown.lots) poly(lb.plan, lb.type == "park" ? "#7fb069" : "#333333", "none", 0);
                    f << "</svg>\n";
                    LOG_INFO << "[citylots] plan svg -> " << planSvg << " (" << grown.plan.blocks.size() << " blocks, " << grown.plan.lots.size() << " lots)";
                }
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
            engine::CityBuildings cityB;
            std::size_t blockedDoors = 0;
            std::vector<engine::CityPlanDebug::Prism> colliderPrisms;
            for (const engine::LotBuilding& lb : grown.lots) {
                const double gy = entityGround ? entityGround(lb.site.x, lb.site.y) : 0.0;
                // Park FENCES + TREE TRUNKS are solid (drive feedback: "Parks
                // also have no collision detection") — thin walls / posts in
                // the same static collider mesh the buildings use.
                auto wall = [&](const Vec2& a2, const Vec2& b2, double h2,
                                double thick) {
                    engine::Vec2 d2 = b2 - a2;
                    const double l2 = d2.length();
                    if (l2 < 1e-3) return;
                    d2 = d2 * (1.0 / l2);
                    const engine::Vec2 n2(-d2.y, d2.x);
                    const double y0 =
                        (entityGround ? entityGround(a2.x, a2.y) : 0.0) - 0.3;
                    const uint32_t s0 =
                        static_cast<uint32_t>(buildingsMc.vertices.size());
                    for (int c4 = 0; c4 < 4; ++c4) {
                        const engine::Vec2 p2 =
                            (c4 < 2 ? a2 : b2) +
                            n2 * ((c4 % 2 == 0 ? 1.0 : -1.0) * thick * 0.5);
                        buildingsMc.vertices.push_back(Vec3(p2.x, y0, p2.y));
                        buildingsMc.vertices.push_back(
                            Vec3(p2.x, y0 + 0.3 + h2, p2.y));
                    }
                    const uint32_t q[4] = { s0, s0 + 2, s0 + 6, s0 + 4 };
                    for (int f2 = 0; f2 < 4; ++f2) {
                        const uint32_t A2 = q[f2], B2 = q[(f2 + 1) % 4];
                        buildingsMc.indices.insert(buildingsMc.indices.end(),
                                                   { A2, B2, B2 + 1, A2, B2 + 1,
                                                     A2 + 1 });
                    }
                };
                for (const auto& fs : lb.fenceSegs)
                    wall(fs.first, fs.second, 0.95, 0.14);
                for (const Vec3& ts : lb.treeSpots) {
                    const engine::Vec2 t2(ts.x, ts.z);
                    wall(t2 + engine::Vec2(-0.16, 0), t2 + engine::Vec2(0.16, 0),
                         2.2, 0.32);
                }
                if (lb.type != "park" && lb.type != "green" &&
                    lb.plan.size() >= 3) {
                    const double base = lb.baseY - 0.5, top = lb.baseY + lb.height;
                    // ADR-0080: prism extrusion extracted to building_collider
                    // (door notches for ENTERABLE units, roof cap, and a floor
                    // cap at the drawn Ground slab so whoever is inside stands
                    // on the floor they see -- not the pad 0.5 m below it).
                    // EXIT AUDIT (citywide enable): a door whose exit point
                    // (foot + 1.5 n) lands inside ANY building plan opens
                    // into a neighbour -- stepping out would embed the
                    // player in that neighbour's collider. Such units demote
                    // to non-enterable BEFORE the notch is cut (the aperture
                    // stays drawn; TECH_DEBT: the lot pass should re-pick
                    // the entrance edge instead).
                    std::vector<engine::BuildingUnit> units = lb.units;
                    for (engine::BuildingUnit& u : units) {
                        if (!u.enterable) continue;
                        bool clear = true;
                        for (const engine::DoorSpec& d : u.doors) {
                            const engine::Vec2 outP = d.foot + d.normal * 1.5;
                            for (const engine::LotBuilding& ob : grown.lots) {
                                if (ob.type == "park" ||
                                    ob.type == "green" || ob.plan.size() < 3)
                                    continue;
                                if (engine::pointInPolygon(ob.plan, outP)) {
                                    clear = false;
                                    if (std::getenv("RT_PRINT_DEMOTED"))
                                        LOG_INFO << "[demoted] " << lb.recipe << " at (" << lb.site.x << ", "
                                                 << lb.site.y << ") floors " << u.params.floors
                                                 << " envelope " << static_cast<int>(u.params.envelope)
                                                 << " door (" << d.foot.x << ", " << d.foot.y << ") n ("
                                                 << d.normal.x << ", " << d.normal.y << ") exits into "
                                                 << ob.recipe << " at (" << ob.site.x << ", " << ob.site.y
                                                 << ")" << (&ob == &lb ? " [ITSELF]" : "");
                                    break;
                                }
                            }
                            if (!clear) break;
                        }
                        if (!clear) {
                            u.enterable = false;
                            ++blockedDoors;
                        }
                    }
                    std::vector<engine::DoorSpec> doorCuts;
                    for (const engine::BuildingUnit& u : units)
                        if (u.enterable)
                            doorCuts.insert(doorCuts.end(), u.doors.begin(),
                                            u.doors.end());
                    engine::appendBuildingPrism(
                        buildingsMc.vertices, buildingsMc.indices, lb.plan,
                        base, top, lb.baseY + 0.05, doorCuts,
                        lb.baseY - lb.groundY);
                    // A PAVED lot (ADR-0086) is a walkable slab to its lot
                    // line: the plate the lot pass drew at paveY gets the
                    // same prism treatment (top cap at the plate, sides down
                    // into the ground), else whoever steps off the building
                    // sinks 0.3 m to the terrain under the concrete.
                    if (lb.pavedLot.size() >= 3)
                        engine::appendBuildingPrism(
                            buildingsMc.vertices, buildingsMc.indices, lb.pavedLot,
                            lb.paveY - 1.0, lb.paveY, lb.paveY - 0.5, {}, 0.0);
                    // Runtime records: one per grown unit (ADR-0080).
                    for (const engine::BuildingUnit& u : units) {
                        cityB.records.push_back({u.plan, u.baseY, lb.groundY,
                                                 lb.height, u.params, u.doors,
                                                 u.enterable, lb.recipe,
                                                 lb.type, lb.district});
                        cityB.records.back().beacons = u.beacons;
                    }
                    // Record the exact prism for the collider debug layer.
                    colliderPrisms.push_back({lb.plan, base, top, lb.district, lb.type});
                }
                // Tag it as a place the agents can route to. An unbuilt GREEN is
                // scenery, not a schedule destination — no place tag.
                if (lb.type != "green") {
                    engine::AuthoredPlace p;
                    p.type = lb.type;
                    p.x = static_cast<float>(lb.site.x);
                    p.z = static_cast<float>(lb.site.y);
                    // The real door (ADR-0080): first unit that has one --
                    // the citysim snaps this place's entrance from a step
                    // outside it instead of from the centroid.
                    for (const engine::BuildingUnit& u : lb.units) {
                        if (u.doors.empty()) continue;
                        const engine::DoorSpec& d0 = u.doors.front();
                        p.hasEntrance = true;
                        p.ex = static_cast<float>(d0.foot.x + d0.normal.x * 1.5);
                        p.ez = static_cast<float>(d0.foot.y + d0.normal.y * 1.5);
                        break;
                    }
                    cfg.places.push_back(std::move(p));
                }

                // One tree (bark + leaf entities) planted at a world spot —
                // shared by the legacy scatter and the sculpted treeSpots.
                auto plantTreeAt = [&](double px, double pz, double scale,
                                       uint32_t th) {
                    Vec3 tPos(px, 0, pz);
                    tPos.y = entityGround ? entityGround(tPos.x, tPos.z) : 0.0;
                    const TreeKit& kit = treeKit(th % 3u);
                    Transform tt;
                    tt.position = tPos;
                    tt.scale = Vec3(scale, scale, scale);
                    tt.orientation =
                        Quat::fromAxisAngle(Vec3(0, 1, 0), (th % 628u) / 100.0);
                    Entity te = world.create();
                    world.add<Transform>(te, tt);
                    world.add<PrevTransform>(te, PrevTransform{tt});
                    Renderable tr;
                    tr.mesh = kit.bark;
                    tr.material = kit.barkMat;
                    tr.renderLayer = engine::LayerFoliage;
                    tr.drawClass = engine::DrawClass::Scenery;
                    world.add<Renderable>(te, tr);
                    if (kit.leaves.index) {
                        Entity le = world.create();
                        world.add<Transform>(le, tt);
                        world.add<PrevTransform>(le, PrevTransform{tt});
                        Renderable lr;
                        lr.mesh = kit.leaves;
                        lr.material = kit.leafMat;
                        lr.renderLayer = engine::LayerFoliage;
                        lr.drawClass = engine::DrawClass::Scenery;
                        world.add<Renderable>(le, lr);
                    }
                };
                // SCULPTED lots (parks, house yards) carry their own
                // deterministic tree spots: (x, trunk scale, z).
                if (!lb.treeSpots.empty()) {
                    uint32_t th = static_cast<uint32_t>(
                        std::llround(lb.site.x * 73.1 + lb.site.y * 37.7)) *
                        2654435761u;
                    for (const Vec3& s : lb.treeSpots) {
                        th = th * 1664525u + 1013904223u;
                        plantTreeAt(s.x, s.z, s.y, th >> 8);
                    }
                }
                if (lb.type == "park" || lb.type == "green") {
                    // The lot's GROUND is the terrain (device: "remove the
                    // green pads ... make sure they adhere to the ground").
                    // Sculpted parks still carry a padMesh — but it holds only
                    // the plaza + walking paths now; a green has none. The
                    // oriented-box pad survives solely for legacy grid levels
                    // whose lots have no polygon.
                    if (!lb.padMesh.vertices.empty() || lb.pad.empty()) {
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
                    }

                    // Trees: deterministic count + spots from the lot position,
                    // scaled by the pad's real area so a block-sized park reads
                    // as a park (device: "scatter more trees around it").
                    // Sculpted lots already planted their own spots above.
                    const uint32_t h = static_cast<uint32_t>(
                        std::llround(lb.site.x * 73.1 + lb.site.y * 37.7)) * 2654435761u;
                    const double padArea = lb.pad.empty()
                        ? static_cast<double>(lb.width * lb.depth)
                        : engine::area(lb.pad);
                    const int nTrees = lb.type == "park"
                        ? std::max(3, std::min(14, static_cast<int>(padArea / 60.0)))
                        : std::max(1, std::min(4, static_cast<int>(padArea / 140.0)));
                    for (int ti = 0; lb.treeSpots.empty() && ti < nTrees; ++ti) {
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
                        const double scale = 0.8 + ((th >> 24) & 0x3Fu) / 63.0 * 0.6;
                        plantTreeAt(lb.site.x + lx * cy - lz * sy,
                                    lb.site.y + lx * sy + lz * cy, scale,
                                    th >> 16);
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
                engine::mirrorTriangles(buildingsMc.indices);
                Entity ce = world.create();
                Transform ct;
                world.add<Transform>(ce, ct);
                world.add<PrevTransform>(ce, PrevTransform{ct});
                buildingsMc.friction = 0.85;
                world.add<engine::MeshCollider>(ce, std::move(buildingsMc));
            }
            lotStage("buildings+trees+collider");
            if (!cityB.records.empty()) {
                cityB.buildIndex();
                std::size_t doorCount = 0, enterableCount = 0;
                for (const engine::BuildingRecord& r : cityB.records) {
                    doorCount += r.doors.size();
                    if (r.enterable) ++enterableCount;
                }
                LOG_INFO << "[buildings] " << cityB.records.size()
                         << " records, " << doorCount << " doors, "
                         << enterableCount << " enterable (" << blockedDoors
                         << " demoted: exit into a neighbour)";
                {   // Finish census (device: "most buildings ... still
                    // white"): the classes actually DEALT across enterable
                    // units, so dead seed bits or a skewed palette shows
                    // in one line instead of a walkthrough.
                    int hf[5] = {0, 0, 0, 0, 0}, hs2[4] = {0, 0, 0, 0},
                        hp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                    for (const auto& r : cityB.records) {
                        if (!r.enterable) continue;
                        ++hf[((r.params.seed >> 6) & 7u) % 5u];
                        ++hs2[(r.params.seed >> 9) & 3u];
                        ++hp[(r.params.seed >> 11) & 7u];
                    }
                    char fin[192];
                    std::snprintf(fin, sizeof fin,
                                  "[interiors] floors %d/%d/%d/%d/%d "
                                  "stairs %d/%d/%d/%d "
                                  "paints %d/%d/%d/%d/%d/%d/%d/%d",
                                  hf[0], hf[1], hf[2], hf[3], hf[4], hs2[0],
                                  hs2[1], hs2[2], hs2[3], hp[0], hp[1],
                                  hp[2], hp[3], hp[4], hp[5], hp[6], hp[7]);
                    LOG_INFO << fin;
                }
                world.add<engine::CityBuildings>(world.create(),
                                                std::move(cityB));
            }
            // One entity per non-empty part class, with the shape-grammar's OWN
            // material recipes: materialFor(PartId) names the procedural surface
            // (brick/concrete/stucco/metal), which gets world-scaled UVs + the
            // baked PBR texture set — identical to the shape:"city" pipeline
            // (user: "we absolutely should be using the pre-existing recipes").
            {
                using Surface = RenderMaterial::Surface;
                SurfaceTexCache lotTex;   // one bake+upload per surface class
                // Chunked per grid cell (plan P1.1): the whole-district merged
                // mesh defeated frustum culling — any visible corner drew the
                // entire city. One Renderable per (cell, part) gives the AABB
                // cull real granularity. renderCell 0 restores the old merge.
                const double renderCell = cs.value("renderCell", 250.0);
                // HLOD (P1.2): full-detail chunks draw to detailDistance; past
                // it each cell's MASS-BOX proxy takes over (baked below from
                // the lots' oriented boxes). 0 disables the swap.
                const double detailDistance = cs.value("detailDistance", 700.0);
                // R2 (city-render-perf): the optional MIDDLE tier. When
                // facadeDistance > detailDistance, full facades stop at
                // detailDistance, the FLAT LOD1 facades carry the ring out to
                // facadeDistance, and the mass boxes take over past that.
                // Absent (0), everything behaves exactly as before.
                const double facadeDistance = cs.value("facadeDistance", 0.0);
                const bool haveFlat = !grown.flatParts.empty() ||
                    std::any_of(grown.cellParts.begin(), grown.cellParts.end(), [](const engine::lotcache::LotCellPart& c) { return c.flat; });
                const bool threeTier =
                    facadeDistance > detailDistance && detailDistance > 0 && haveFlat;
                // Per part: the material, its surface textures (one bake per surface class) and the
                // draw-distance scale — derived once and applied to every chunk of the part, whichever
                // tier and wherever the chunk came from (grown here or read per cell from the bundle).
                struct PartProto { Renderable proto; Surface surf = Surface::None; bool reUV = false; double ddScale = 1.0; bool ready = false; };
                std::map<std::size_t, PartProto> protos;
                TextureHandle roomAtlas{};   // baked on first sight of a lit-glass part (interior mapping)
                auto protoFor = [&](std::size_t pi, bool scaleSmallParts) -> PartProto& {
                    PartProto& pp = protos[pi * 2 + (scaleSmallParts ? 1 : 0)];
                    if (pp.ready) return pp;
                    pp.proto.renderLayer = engine::LayerBuildings;   // debug layer toggle
                    pp.proto.material = materialFor(static_cast<PartId>(pi), Vec3(0.80, 0.78, 0.75));
                    if (static_cast<PartId>(pi) == PartId::GlassLit) {
                        // The faked tier: every lit pane shows a virtual room (interior mapping),
                        // the atlas in the albedo slot; the glass albedo stays for the day look.
                        if (roomAtlas.index == 0) roomAtlas = bakeRoomAtlas(renderer);
                        pp.proto.material.albedoMap = roomAtlas;
                        pp.proto.material.flags |= RenderMaterial::FLAG_INTERIOR_MAP;
                    }
                    pp.surf = pp.proto.material.surface();
                    if (pp.surf != Surface::None) {
                        // FanTop/VentGrille/RoofShingle bake their own UVs in the grammar (centred disc /
                        // plate-fitted / slope-fitted) — a world-planar re-UV would break them. InteriorFloor
                        // is exempt BY PART, not by surface: it authors plank-direction UVs (u along the
                        // room's long axis) but shares WoodSiding with the water tanks, whose world-planar
                        // re-UV must stay.
                        pp.reUV = !surfaceBakesOwnUVs(pp.surf) &&
                                  static_cast<PartId>(pi) != PartId::InteriorFloor &&
                                  static_cast<PartId>(pi) != PartId::InteriorFloorTile &&
                                  static_cast<PartId>(pi) != PartId::InteriorFloorMarble &&
                                  static_cast<PartId>(pi) != PartId::InteriorFloorCarpet;
                        bindSurfaceMaps(pp.proto.material, bakeSurfaceTextures(renderer, pp.surf, lotTex));
                    }
                    // MID TIER: small dressing (HVAC, tanks, trim, doors, hedges) is subpixel long before
                    // the shell swaps to its proxy — cull those parts earlier so the far half of the detail
                    // ring draws bare shells. Superseded in three-tier mode, where EVERY part swaps to LOD1
                    // together at detailDistance (a clean lockstep swap beats a ring of buildings missing
                    // their trim).
                    if (scaleSmallParts) {
                        switch (static_cast<PartId>(pi)) {
                            case PartId::Vent: case PartId::Utility: case PartId::Fan:
                            case PartId::Wood: case PartId::Detail: case PartId::Trim:
                            case PartId::Door: case PartId::Foliage: case PartId::Path:
                                pp.ddScale = 0.55; break;
                            default: break;
                        }
                    }
                    pp.ready = true; return pp;
                };
                // One chunk (a render cell's share of a part) → one Renderable. World-planar UVs are a
                // per-vertex function of position and normal, so a chunk gets the UVs the whole part would.
                auto spawnChunk = [&](std::size_t pi, RenderMesh& chunk, double minDist, double drawDist, bool scaleSmallParts) {
                    if (chunk.vertices.empty()) return;
                    PartProto& pp = protoFor(pi, scaleSmallParts);
                    if (pp.reUV) applyWorldPlanarUVs(chunk, 1.0 / surfaceWorldTileSize(pp.surf));
                    Renderable r = pp.proto;
                    if (drawDist > 0) r.drawDistance = drawDist * pp.ddScale;
                    r.minDistance = minDist;
                    r.drawClass = engine::DrawClass::Structure;
                    r.mesh = assets.acquireMesh(chunk, "");   // world-space, unkeyed
                    Entity e = world.create();
                    Transform t;   // identity — the mesh sits in world space
                    world.add<Transform>(e, t);
                    world.add<PrevTransform>(e, PrevTransform{t});
                    world.add<Renderable>(e, r);
                    // The lit-window part (WS3): warm interior glow, raised after dusk by the day/night
                    // NightGlow pass — dark at noon by construction (emission starts 0; the material
                    // equals Glass by day).
                    if (static_cast<PartId>(pi) == PartId::LitBand)
                        // Crown bands, signage, podium uplights: the vertex colour is the tint.
                        world.add<engine::NightGlow>(e, engine::NightGlow{Vec3(1.0, 1.0, 1.0) * 1.3});
                    if (static_cast<PartId>(pi) == PartId::GlassLit)
                        // White: the pane's vertex colour is its tint (litTint, FLAG_EMISSIVE_VERTEX_TINT);
                        // 0.5, not 1.3: the room atlas carries the contrast now (a fixture at 1, walls
                        // at ~0.6, floor ~0.25) and the night exposure lifts it, so the fixture clips
                        // to white and the walls keep their tint instead of the whole pane clipping.
                        world.add<engine::NightGlow>(e, engine::NightGlow{Vec3(1.0, 1.0, 1.0) * 0.22});
                    const bool beaconFamily = static_cast<PartId>(pi) == PartId::Beacon ||
                                              static_cast<PartId>(pi) == PartId::BeaconGlow ||
                                              static_cast<PartId>(pi) == PartId::BeaconHaze;
                    if (beaconFamily) {
                        // Aviation beacons: FLASHING on a phase hashed from the 24 m cell the chunk
                        // stands in (beacon chunks are cut that small, so neighbouring towers differ;
                        // the lamp and its halo share the cell, so they share the phase).
                        Vec3 c(0, 0, 0);
                        for (const Vertex& v : chunk.vertices) c += v.position;
                        c = c * (1.0 / static_cast<double>(chunk.vertices.size()));
                        engine::BeaconBlink bb;
                        engine::beaconCellPhase(static_cast<int>(std::floor(c.x / kBeaconChunk)),
                                                static_cast<int>(std::floor(c.z / kBeaconChunk)), bb.period, bb.phase);
                        bb.baseOpacity = r.material.opacity;   // the halos fade out with the flash
                        if (bb.baseOpacity < 1.0f) r.material.opacity = 0.0f;   // dark at noon by construction
                        // ON SCREEN (the beacon pass divides by the night exposure adaptation, up
                        // to 6x, because a red brighter than ~1 tonemaps to orange, then white —
                        // Glenn: "white and red tinged"): the lamp itself 1.3, a saturated red; its
                        // translucent halo hotter, so the blended haze crosses the bloom threshold
                        // and the light GLOWS (Glenn: "missing an emissive quality").
                        // On screen through their opacity: bulb 0.55 x 3.0 = 1.65, haze 0.22 x 4.5 = 1.0.
                        const Real glow = static_cast<PartId>(pi) == PartId::BeaconGlow ? 3.0
                                        : static_cast<PartId>(pi) == PartId::BeaconHaze ? 4.5 : 1.3;
                        world.add<engine::NightGlow>(e, engine::NightGlow{Vec3(1.0, 1.0, 1.0) * glow});
                        world.add<engine::BeaconBlink>(e, bb);
                    }
                };
                // Whole parts (grown here, or a whole-part bundle): split per render cell now. One spawner
                // for both tiers, so material binding and chunking cannot diverge between LOD0 and LOD1.
                auto spawnPartChunks = [&](std::vector<RenderMesh>& partsVec, double minDist, double drawDist, bool scaleSmallParts) {
                    for (std::size_t pi = 0; pi < partsVec.size(); ++pi) {
                        RenderMesh& pm = partsVec[pi];
                        if (pm.vertices.empty()) continue;
                        // Beacons chunk FINE (one tower, not one cell), so each roof can flash on its own phase.
                        const bool beaconPart = static_cast<PartId>(pi) == PartId::Beacon ||
                                                static_cast<PartId>(pi) == PartId::BeaconGlow ||
                                                static_cast<PartId>(pi) == PartId::BeaconHaze;
                        const double cellFor = beaconPart ? kBeaconChunk : renderCell;
                        for (RenderMesh& chunk : chunkMeshByCell(pm, cellFor)) spawnChunk(pi, chunk, minDist, drawDist, scaleSmallParts);
                    }
                };
                if (!grown.cellParts.empty() && grown.bundle) {
                    // Parts already split per render cell in the bundle (ADR-0084 B): one section → one
                    // Renderable, unpacked one chunk at a time; nothing is chunked at load.
                    std::size_t spawned = 0;
                    for (const engine::lotcache::LotCellPart& cp : grown.cellParts) {
                        if (cp.flat && !threeTier) continue;   // the LOD1 tier is drawn only in three-tier mode
                        RenderMesh chunk;
                        if (!engine::lotcache::readLotPart(*grown.bundle, cp.section, chunk)) { LOG_WARN << "[lots] " << cp.section << ": unreadable, skipped"; continue; }
                        if (static_cast<PartId>(cp.part) == PartId::Beacon || static_cast<PartId>(cp.part) == PartId::BeaconGlow ||
                            static_cast<PartId>(cp.part) == PartId::BeaconHaze) {
                            // The bundle cut beacons per render cell; re-cut them fine so each roof flashes alone.
                            for (RenderMesh& sub : chunkMeshByCell(chunk, kBeaconChunk)) {
                                if (cp.flat) spawnChunk(static_cast<std::size_t>(cp.part), sub, detailDistance, facadeDistance, false);
                                else spawnChunk(static_cast<std::size_t>(cp.part), sub, 0.0, detailDistance, !threeTier);
                            }
                            ++spawned;
                            continue;
                        }
                        if (cp.flat) spawnChunk(static_cast<std::size_t>(cp.part), chunk, detailDistance, facadeDistance, false);
                        else spawnChunk(static_cast<std::size_t>(cp.part), chunk, 0.0, detailDistance, !threeTier);
                        ++spawned;
                    }
                    LOG_INFO << "[lots] " << spawned << " cell parts instantiated from the bundle";
                } else {
                    spawnPartChunks(lotParts, 0.0, detailDistance, !threeTier);
                    if (threeTier) spawnPartChunks(grown.flatParts, detailDistance, facadeDistance, false);
                }
            }
            lotStage("part chunks");
            // HLOD PROXIES (P1.2): one mass-box mesh per render cell, baked
            // from the lots' oriented boxes in their own colours — a distant
            // chunk becomes a handful of boxes, visually the same silhouette
            // for a fraction of the triangles. Drawn only PAST detailDistance.
            if (cs.value("detailDistance", 700.0) > 0) {
                const double cell = cs.value("renderCell", 250.0);
                const double fd = cs.value("facadeDistance", 0.0);
                const double dd = std::max(cs.value("detailDistance", 700.0), fd);
                // Two proxy meshes per cell: MASONRY (matte, the wall colour) and GLASS
                // (a curtain-wall tower's proxy is metallic like its facade — a matte box in
                // the wall colour its facade never shows read as a pale slab by day and
                // night beside the mirror-dark real towers; Glenn's skyline shot).
                std::map<std::tuple<int, int, int>, RenderMesh> proxies;
                for (const engine::LotBuilding& lb : grown.lots) {
                    if (lb.height < 1.5 || lb.pad.size() >= 3) continue;   // parks/greens: skip
                    const bool curtain = !lb.units.empty() && lb.units.front().params.curtainWall;
                    RenderMesh& pmesh =
                        proxies[{(int)std::floor(lb.site.x / cell),
                                 (int)std::floor(lb.site.y / cell), curtain ? 1 : 0}];
                    // Value-match the detail look (device: pop at the swap):
                    // real facades read darker than their wall colour because
                    // of the window grid, and every roof deck is near-charcoal
                    // — sides bake that window duty cycle in, the cap takes
                    // the roof material's tone.
                    // The distant tier's foundation reach: min ground along
                    // the plan perimeter (the same carved+dilated sampler
                    // placement uses), bedded in — so the far building can't
                    // hover where its LOD0 twin stands on concrete.
                    Real boxBottom = std::numeric_limits<Real>::quiet_NaN();
                    if (entityGround && lb.plan.size() >= 3) {
                        double lo = 1e30;
                        for (std::size_t pi = 0; pi < lb.plan.size(); ++pi) {
                            const engine::Vec2& a2 = lb.plan[pi];
                            const engine::Vec2& b2 =
                                lb.plan[(pi + 1) % lb.plan.size()];
                            lo = std::min(lo, entityGround(a2.x, a2.y));
                            const engine::Vec2 m2 = (a2 + b2) * 0.5;
                            lo = std::min(lo, entityGround(m2.x, m2.y));
                        }
                        boxBottom = static_cast<Real>(lo - 0.5);
                    }
                    // Glass carries the pane colour; masonry the wall colour, a touch darker
                    // than the day match so the far blocks do not float pale at night.
                    const Vec3 sideCol = curtain ? Vec3(0.10, 0.14, 0.18) : lb.color * 0.74;
                    engine::appendLotMassBox(pmesh, lb, sideCol,
                                             Vec3(0.20, 0.20, 0.22), boxBottom);
                }
                // The distant tier's NIGHT: a baked lit-window emissive map — one window
                // per bay and storey (the mass box's UVs are in those cells), a third of
                // them lit in the same tints the real panes wear — under a NightGlow tag,
                // so past detailDistance a tower still shows its windows after dusk instead
                // of going grey while its beacons flash (Glenn's skyline shot, 2026-09-14).
                // Mipmapping averages the grid to a soft glow at a kilometre, which is what
                // the real chunk collapses to as well.
                const TextureHandle litWindows = bakeLitWindowMap(renderer, engine::kMassBoxTile * 8, 4242u);
                for (auto& [key, pmesh] : proxies) {
                    if (pmesh.vertices.empty()) continue;
                    const bool glass = std::get<2>(key) == 1;
                    Renderable r;
                    r.renderLayer = engine::LayerBuildings;
                    r.material.albedo = Vec3(1, 1, 1);   // colour rides the verts
                    // Glass proxies are MATTE and dark, not metallic: a metallic box mirrors
                    // the sky — pale by day, and pale at night under the six-fold exposure,
                    // whatever its albedo — where the real tower's panes break the sky up
                    // and read dark. (Measured: a red-painted metallic proxy still drew white.)
                    r.material.roughness = glass ? 0.8f : 0.9f;
                    r.material.metallic = 0.0f;
                    r.material.emissiveMap = litWindows;
                    r.minDistance = dd;
                    r.mesh = assets.acquireMesh(pmesh, "");
                    Entity e = world.create();
                    Transform t;
                    world.add<Transform>(e, t);
                    world.add<PrevTransform>(e, PrevTransform{t});
                    world.add<Renderable>(e, r);
                    // 0.4, not the panes' 1.3: mipmapping has already averaged the grid to a
                    // third, and at the night exposure a box glowing evenly at 1.3 read as a
                    // pale slab — the far city should be a soft, tinted glow, not lit boxes.
                    engine::NightGlow ng;
                    ng.fullEmission = Vec3(1.0, 1.0, 1.0) * 1.0;
                    ng.nightAlbedo = 0.22f;   // the body goes dark with dusk; the window glow carries it
                    ng.dayAlbedo = r.material.albedo;
                    world.add<engine::NightGlow>(e, ng);
                }
            }
            lotStage("hlod proxies");
            // Publish the plan (blocks + lots + collider prisms) for the
            // citysim debug overlay.
            if (!plan.blocks.empty() || !colliderPrisms.empty()) {
                engine::CityPlanDebug dbg;
                dbg.blocks = std::move(plan.blocks);
                dbg.lots = std::move(plan.lots);
                dbg.prisms = std::move(colliderPrisms);
                world.add<engine::CityPlanDebug>(world.create(), std::move(dbg));
            }
            lotStage("plan debug");
            LOG_INFO << "[lots] load tail: " << lotStages << " (" << std::chrono::duration<double>(std::chrono::steady_clock::now() - tLots0).count() << " s after the grow)";
        }
        world.add<CitySimConfig>(world.create(), cfg);
    }

    // Day/night policy (device: "the scene loads bright but everything gets
    // dark when the level starts" — the cycle was overwriting the authored
    // sun + ambient every frame). "dayNight": {"enabled": false} pins the
    // level's static lighting; {"timeOfDay": .., "dayMinutes": ..,
    // "latitude": .., "dayOfYear": ..} seeds the cycle (real minutes per
    // loop, and where/when on Earth the sun's arc is drawn for).
    if (root.contains("dayNight") && root["dayNight"].is_object()) {
        const auto& dn = root["dayNight"];
        DayNightConfig dc;
        dc.enabled = dn.value("enabled", dc.enabled);
        dc.timeOfDay = dn.value("timeOfDay", dc.timeOfDay);
        dc.dayMinutes = dn.value("dayMinutes", dc.dayMinutes);
        dc.speed = dn.value("speed", dc.speed);   // legacy days/sec
        dc.latitude = dn.value("latitude", dc.latitude);
        dc.dayOfYear = dn.value("dayOfYear", dc.dayOfYear);
        dc.newMoonDay = dn.value("newMoonDay", dc.newMoonDay);
        dc.lightPollution = dn.value("lightPollution", dc.lightPollution);
        dc.pollutionFalloff = dn.value("pollutionFalloff", dc.pollutionFalloff);
        {
            const std::string w = dn.value("weather", std::string());
            if (w == "off") dc.weather = 0;
            else if (w == "auto") dc.weather = 1;
            else if (w == "clear") dc.weather = 2;
            else if (w == "fair") dc.weather = 3;
            else if (w == "overcast") dc.weather = 4;
            else if (w == "storm") dc.weather = 5;
        }
        world.add<DayNightConfig>(world.create(), dc);
    }

    // STREET FURNITURE at BUILD time (device: "place the stop lights when we
    // build the city instead of during the simulation ... the simulation
    // should use it but it shouldn't be responsible for where they are").
    // Plan every signal pole + street lamp from the roads' deterministic nav
    // graph (the same one the citysim bridge will derive), spawn them as city
    // geometry, and publish a StreetFurniture component: the sim animates the
    // lenses and reacts to phases, but never invents a pole.
    if (root.contains("citysim") &&
        root["citysim"].value("streetFurniture", true)) {
        engine::RoadGraph combined;
        std::function<Real(Real, Real)> furnGround;
        // §10: furniture plans on THE unified graph (class filters keep
        // lamps/signals off the corridor); ground is the level's own sampler
        // (the entity no longer stores one).
        world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph& g) {
            if (combined.nodes.empty()) combined = g.graph;
        });
        // The CARVED sampler, not levelGround: furniture grounded against the
        // NATURAL field floated over road fills and sank into cuts on every
        // graded hillside — poles must stand on the same surface the road and
        // sidewalk were draped on (entityGround also carries the mesh-matching
        // leaf dilate).
        if (entityGround)
            furnGround = [g = entityGround](Real x, Real z) { return g(x, z); };
        else if (levelGround)
            furnGround = [g = levelGround](Real x, Real z) { return g(x, z); };
        // ...and on the DECK where there is one: the carved sampler sits 0.22 m
        // under the drawn asphalt (road_net.cpp, "Carve a step BELOW"), and a
        // pole planted there stood with its foot inside the sidewalk band. The
        // band rides the deck + curb, which is where a kerb pole's foot goes.
        {
            auto decks = std::make_shared<std::vector<engine::RoadDeckField>>();
            Real curb = 0, margin = 0;
            world.each<engine::RoadDeck>(
                [&](Entity, engine::RoadDeck& d) { decks->push_back(d.field); });
            world.each<engine::RoadEntity>([&](Entity, engine::RoadEntity& net) {
                curb = std::max(curb, static_cast<Real>(net.look.curb));
                margin = std::max(margin, static_cast<Real>(net.look.sidewalk) + 2.0);
            });
            if (!decks->empty() && furnGround) {
                auto base = furnGround;
                furnGround = [base, decks, curb, margin](Real x, Real z) {
                    for (const engine::RoadDeckField& f : *decks) {
                        double y;
                        if (f.heightAt(x, z, margin, &y)) return static_cast<Real>(y) + curb;
                    }
                    return base(x, z);
                };
            }
        }
        if (!combined.edges.empty()) {
            const engine::NavGraph nav = engine::buildNavGraph(combined);
            const engine::StreetFurniturePlan fplan =
                [&] {
                    engine::StreetFurnitureParams fp;
                    world.each<engine::RoadEntity>(
                        [&](Entity, engine::RoadEntity& net) {
                            fp.sidewalkWidth = std::max(
                                fp.sidewalkWidth,
                                static_cast<Real>(net.look.sidewalk));
                        });
                    return engine::planStreetFurniture(nav, furnGround, fp);
                }();
            engine::StreetFurniture sf;
            sf.navLinkCount = nav.linkCount();
            sf.lampHeads = fplan.lampHeads;
            for (const engine::SignalSpot& s : fplan.signals)
                sf.signalPoles.push_back({s.base, s.face, s.link});
            {
                const double hubRadius =
                    root.contains("citysim") && root["citysim"].is_object()
                        ? root["citysim"].value("hubRadius", 220.0) : 220.0;
                engine::CityMap map{assembleCityMap(world, combined, nav, fplan, hubRadius)};
                std::vector<const engine::RoadDeckField*> decks;
                world.each<engine::RoadDeck>([&](Entity, engine::RoadDeck& d) { decks.push_back(&d.field); });
                if (const char* svgPath = std::getenv("RT_CITY_SVG")) {
                    const char* sel = std::getenv("RT_CITY_SVG_LAYERS");
                    engine::writeCityMapSvg(svgPath, *map.data,
                                            engine::CityMapLayers::fromList(sel ? sel : "all"), decks);
                }
                if (const char* svgPath = std::getenv("RT_FURNITURE_SVG"))
                    engine::writeCityMapSvg(svgPath, *map.data, engine::furnitureMapLayers(), decks);
                // The census itself, for the Teleport panel's place list.
                map.conflicts = std::make_shared<const std::vector<engine::SidewalkCrossing>>(
                    engine::findSidewalkRoadCrossings(*map.data, decks));
                if (!map.conflicts->empty())
                    LOG_INFO << "[citymap] " << map.conflicts->size()
                             << " sidewalk-on-asphalt places (Debug > Teleport lists them)";
                world.add<engine::CityMap>(world.create(), std::move(map));
            }

            auto groupBounds = [&](InstanceGroup& g, Real meshReach) {
                if (g.transforms.empty()) return;
                Vec3 c(0, 0, 0);
                for (const Mat4& m : g.transforms)
                    c = c + Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
                c = c / static_cast<Real>(g.transforms.size());
                Real spread = 0;
                for (const Mat4& m : g.transforms)
                    spread = std::max(spread,
                        (Vec3(m.m[0][3], m.m[1][3], m.m[2][3]) - c).length());
                g.boundsCenter = c;
                g.boundsRadius = spread + meshReach;
            };

            // Signal posts: ONE static group — the citysim bridge adopts this
            // entity (lens animation, pedestrian obstacles, physics poles).
            if (!sf.signalPoles.empty()) {
                InstanceGroup g;
                g.mesh = assets.acquireMesh(engine::trafficSignalProto(),
                                            "city:signalpost");
                g.material.albedo = Vec3(1, 1, 1);   // colour rides the verts
                g.material.roughness = 0.6f;
                for (const auto& s : sf.signalPoles) {
                    const Real yaw = std::atan2(s.face.x, s.face.z);
                    g.transforms.push_back(Mat4::trs(
                        s.base, Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                        Vec3(1, 1, 1)));
                }
                groupBounds(g, 8.0);
                Entity pg = world.create();
                world.add<InstanceGroup>(pg, g);
                sf.postGroup = pg;
            }

            // Street lamps: per-cell groups so the frustum/draw-distance cull
            // works street by street, plus an emissive HEAD shell per lamp so
            // the fixture reads lit (it also feeds the night point lights —
            // see RenderSystem).
            if (!fplan.lampBases.empty()) {
                const Real cellSz = 280.0;
                std::map<std::pair<int, int>, std::vector<Mat4>> cells;
                for (const Vec3& b : fplan.lampBases)
                    cells[{(int)std::floor(b.x / cellSz),
                           (int)std::floor(b.z / cellSz)}]
                        .push_back(Mat4::translate(b.x, b.y, b.z));
                MeshHandle poleMesh =
                    assets.acquireMesh(engine::streetLamp(), "city:streetlamp");
                engine::LampParams lp;
                RenderMesh glowBox = MeshBuilder::box(
                    Vec3(lp.headSize.x + 0.05, lp.headSize.y + 0.04,
                         lp.headSize.z + 0.05));
                for (Vertex& v : glowBox.vertices)
                    v.position.y += lp.height;   // shell wraps the head
                MeshHandle glowMesh = assets.acquireMesh(glowBox, "city:lampglow");
                // The light LOD block: poles and glow shells draw to lampGlowDistance,
                // beyond which BeaconLightSystem's sprite tier carries every bulb.
                const double lampGlowDist = root.contains("citysim")
                                                ? root["citysim"].value("lampGlowDistance", 650.0)
                                                : 650.0;
                for (auto& [key, transforms] : cells) {
                    InstanceGroup g;
                    g.mesh = poleMesh;
                    g.material.albedo = Vec3(1, 1, 1);
                    g.material.roughness = 0.7f;
                    g.transforms = transforms;
                    g.drawDistance = lampGlowDist;
                    g.drawClass = engine::DrawClass::Furniture;
                    groupBounds(g, lp.height + 1.0);
                    world.add<InstanceGroup>(world.create(), g);
                    InstanceGroup glow;
                    glow.mesh = glowMesh;
                    glow.material.albedo = Vec3(0.25, 0.21, 0.12);
                    // Emission starts DARK: lamps glowed 24/7 before (WS3).
                    // The NightGlow tag hands the full emission to the
                    // day/night pass, which scales it on the shared dusk ramp.
                    glow.material.emission = Vec3(0, 0, 0);
                    glow.material.roughness = 0.4f;
                    glow.transforms = std::move(transforms);
                    glow.drawDistance = lampGlowDist;
                    glow.drawClass = engine::DrawClass::Effect;
                    groupBounds(glow, lp.height + 1.0);
                    Entity glowE = world.create();
                    world.add<InstanceGroup>(glowE, glow);
                    world.add<engine::NightGlow>(
                        glowE, engine::NightGlow{Vec3(1.0, 0.85, 0.55) * 4.5});
                }
            }
            LOG_INFO << "[furniture] " << sf.signalPoles.size() << " signals, "
                     << fplan.lampBases.size() << " street lamps";
            world.add<engine::StreetFurniture>(world.create(), std::move(sf));
        }
    }

    // RT_NO_PLAYER=1 suppresses the player entirely — for headless screenshots / debug renders, so
    // the first-person gun viewmodel and a settling capsule don't intrude on an overhead frame dump.
    if (!std::getenv("RT_NO_PLAYER")) {
        if (root.contains("player")) {
            json pj = root["player"];
            // RT_SPAWN=x,y,z: a debug spawn, taken verbatim — no walk-out of
            // prisms, no terrain lift — so a headless shot can start a walker
            // on the 20th floor of a tower and the interior streams around it.
            bool spawnOverride = false;
            if (const char* sp = std::getenv("RT_SPAWN")) {
                double sx = 0, sy = 0, sz = 0;
                if (std::sscanf(sp, "%lf,%lf,%lf", &sx, &sy, &sz) == 3) {
                    pj["position"] = json::array({sx, sy, sz});
                    spawnOverride = true;
                    LOG_INFO << "[player] RT_SPAWN override: (" << sx << ", " << sy << ", " << sz << ")";
                }
            }
            // Terrain levels: never spawn under a hill — authored spawns
            // assume flat ground, so lift the point to the surface when the
            // terrain there is higher.
            if (entityGround && !spawnOverride && pj.contains("position") &&
                pj["position"].is_array() && pj["position"].size() >= 3) {
                double px = pj["position"][0].get<double>();
                const double py = pj["position"][1].get<double>();
                double pz = pj["position"][2].get<double>();
                // NEVER SPAWN INSIDE A BUILDING (device: "I'm stuck inside of
                // a building for the player start"). Every lot building is a
                // solid plan prism (the district collider, above), so a spawn
                // inside a footprint is a capsule inside a static mesh with
                // no way out. Nothing checked this: the terrain lift below
                // is the only adjustment the spawn ever got, and the
                // playable-levels test's CDLOD branch cannot see a prism
                // (it has no bottom face). metro_v2's authored spawn sat
                // inside a 417 m2 building, 17.6 m from its street.
                //
                // Walk the spawn out: to the nearest edge of the offending
                // prism, 2.5 m past it along the outward normal, and re-test
                // (a neighbour's prism may be there too). Loud, not silent.
                std::vector<const CityPlanDebug::Prism*> prisms;
                world.each<CityPlanDebug>([&](Entity, CityPlanDebug& plan) {
                    for (const CityPlanDebug::Prism& pr : plan.prisms)
                        prisms.push_back(&pr);
                });
                const double ox = px, oz = pz;
                // Prisms near the spawn (the sweep below stays within 24 m).
                std::vector<const CityPlanDebug::Prism*> nearby;
                for (const CityPlanDebug::Prism* pr : prisms) {
                    if (pr->plan.size() < 3) continue;
                    double x0 = 1e300, x1 = -1e300, z0 = 1e300, z1 = -1e300;
                    for (const engine::Vec2& v : pr->plan) {
                        x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
                        z0 = std::min(z0, v.y); z1 = std::max(z1, v.y);
                    }
                    if (px > x0 - 26.0 && px < x1 + 26.0 &&
                        pz > z0 - 26.0 && pz < z1 + 26.0)
                        nearby.push_back(pr);
                }
                const auto insideAny =
                    [&](double x, double z) -> const CityPlanDebug::Prism* {
                    for (const CityPlanDebug::Prism* pr : nearby)
                        if (engine::pointInPolygon(pr->plan, engine::Vec2(x, z)))
                            return pr;
                    return nullptr;
                };
                bool moved = false;
                if (const CityPlanDebug::Prism* hit = insideAny(px, pz)) {
                    // A building plan can be CONCAVE (metro_v2's civic
                    // spawn building is one L-shaped polygon — measured: 1
                    // containing prism): from inside one wing the nearest
                    // edge is the notch, and stepping 2.5 m "outward" past
                    // it crosses into the other wing, still inside. The
                    // first cut of this guard did exactly that, ping-ponged,
                    // and logged success after its attempts ran out. Sweep
                    // rings around the authored point instead: the nearest
                    // deterministic candidate outside EVERY prism wins. The
                    // containing count in the warn keeps overlap measurable.
                    int containing = 0;
                    for (const CityPlanDebug::Prism* pr : nearby)
                        if (engine::pointInPolygon(pr->plan,
                                                   engine::Vec2(px, pz)))
                            ++containing;
                    for (double r = 3.0; r <= 24.0 && !moved; r += 1.5) {
                        for (int h = 0; h < 16 && !moved; ++h) {
                            const double a = h * (3.14159265358979323846 / 8.0);
                            const double cx = ox + std::cos(a) * r;
                            const double cz = oz + std::sin(a) * r;
                            if (!insideAny(cx, cz)) {
                                px = cx;
                                pz = cz;
                                moved = true;
                            }
                        }
                    }
                    if (moved) {
                        pj["position"][0] = px;
                        pj["position"][2] = pz;
                        LOG_WARN << "[spawn] authored spawn (" << ox << ","
                                 << oz << ") is INSIDE a building ("
                                 << hit->type << ", " << containing
                                 << " containing prisms) -> moved to (" << px
                                 << "," << pz
                                 << "); re-author player.position";
                    } else {
                        LOG_WARN << "[spawn] authored spawn (" << ox << ","
                                 << oz << ") is INSIDE a building ("
                                 << hit->type << ", " << containing
                                 << " containing prisms) and NO clear point "
                                    "within 24 m -- left as authored";
                    }
                }
                const double gy = entityGround(px, pz);
                if (py < gy + 1.2 || moved) pj["position"][1] = gy + 1.2;
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

    // Per-level instance-buffer capacities (8km-city plan P0.2). Values below
    // the backend defaults are clamped up by the renderer.
    if (root.contains("render") && root["render"].is_object()) {
        const auto& r = root["render"];
        renderer.setInstanceCapacities(r.value("maxInstances", 0u),
                                       r.value("maxShadowInstances", 0u),
                                       r.value("maxFoliageInstances", 0u));

        // "render.drawDistances": the level's draw-distance policy, by content
        // class (metres; omitted or 0 = unlimited). Stamped as a singleton that
        // RenderSystem resolves every Renderable/InstanceGroup through, so the
        // distance for e.g. every tree in the world lives HERE and not at the
        // dozen sites that create drawables. Absent block = no policy entity =
        // everything unlimited, which is what levels did before this existed.
        if (r.contains("drawDistances") && r["drawDistances"].is_object()) {
            const auto& d = r["drawDistances"];
            DrawPolicy p;
            auto set = [&](DrawClass c, const char* key) {
                p.distance[static_cast<int>(c)] = d.value(key, 0.0);
            };
            set(DrawClass::Terrain,     "terrain");
            set(DrawClass::Structure,   "structure");
            set(DrawClass::Scenery,     "scenery");
            set(DrawClass::Furniture,   "furniture");
            set(DrawClass::GroundPaint, "groundPaint");
            set(DrawClass::SimBody,     "simBody");
            set(DrawClass::Effect,      "effect");
            world.add<DrawPolicy>(world.create(), p);
        }
    }

    // Report drawables that never declared a content class. They resolve to
    // unlimited (the old default), so this is not an error — it is the thing
    // that used to be invisible. A site added later without a DrawClass shows
    // up here instead of quietly drawing to the far plane forever.
    {
        size_t unsetR = 0, totalR = 0, unsetG = 0, totalG = 0;
        world.each<Renderable>([&](Entity, Renderable& r) {
            ++totalR;
            if (r.drawClass == DrawClass::Unset) ++unsetR;
        });
        world.each<InstanceGroup>([&](Entity, InstanceGroup& g) {
            ++totalG;
            if (g.drawClass == DrawClass::Unset) ++unsetG;
        });
        if (unsetR || unsetG)
            LOG_INFO << "[draw-policy] " << unsetR << "/" << totalR
                     << " renderables and " << unsetG << "/" << totalG
                     << " instance groups have no DrawClass (drawing unlimited)";
    }

    // Environment map (equirectangular .hdr) — bound before probes so the bake
    // captures it for IBL (ADR-0016). Lives under the "environment" object as
    // "hdr"; path is relative to the level file.
    //
    // Cinematic-sky opt-ins (reset first — the RenderView/renderer are reused
    // across loads, so a previous level's sky must not leak):
    //   "environment.sky"    {"model": "scattering", turbidity?, groundAlbedo?,
    //                         brightness?, mieG?, multiScatter?, aerial?,
    //                         sunAngularRadius?} -> LUT scattering sky + aerial
    //                         perspective (replaces exp fog while active).
    //   "environment.clouds" {coverage?, bottom?, top?, density?, noiseScale?,
    //                         wind?, steps?, lightSteps?, phaseG?, far?,
    //                         ambient?, detailStrength?, enabled?} -> volumetric
    //                         cloud slab (retires the 2D FBM overlay while
    //                         active).
    view.lighting.skyScattering = SkyScatteringParams{};
    view.lighting.volumetricClouds = VolumetricCloudParams{};
    // Fog too: it was the one environment block with no reset, so a level
    // without an "environment.fog" object inherited the previous level's haze
    // for the rest of the session.
    view.lighting.fog = FogParams{};
    if (root.contains("environment") && root["environment"].is_object()) {
        const auto& env = root["environment"];
        if (env.contains("sky") && env["sky"].is_object()) {
            const auto& s = env["sky"];
            if (s.value("model", std::string()) == "scattering") {
                auto& ss = view.lighting.skyScattering;
                ss.enabled = true;
                ss.turbidity        = s.value("turbidity", ss.turbidity);
                ss.brightness       = s.value("brightness", ss.brightness);
                ss.mieG             = s.value("mieG", ss.mieG);
                ss.multiScatter     = s.value("multiScatter", ss.multiScatter);
                ss.aerialDensity    = s.value("aerial", ss.aerialDensity);
                ss.sunAngularRadius = s.value("sunAngularRadius", ss.sunAngularRadius);
                if (s.contains("groundAlbedo"))
                    ss.groundAlbedo = parseVec3(s["groundAlbedo"], ss.groundAlbedo);
            }
        }
        if (env.contains("clouds") && env["clouds"].is_object()) {
            const auto& c = env["clouds"];
            auto& vc = view.lighting.volumetricClouds;
            vc.enabled     = c.value("enabled", true);
            vc.coverage    = c.value("coverage", vc.coverage);
            vc.bottom      = c.value("bottom", vc.bottom);
            vc.top         = c.value("top", vc.top);
            vc.density     = c.value("density", vc.density);
            vc.noiseScale  = c.value("noiseScale", vc.noiseScale);
            vc.wind        = c.value("wind", vc.wind);
            vc.steps       = c.value("steps", vc.steps);
            vc.lightSteps  = c.value("lightSteps", vc.lightSteps);
            vc.phaseG      = c.value("phaseG", vc.phaseG);
            vc.farDistance = c.value("far", vc.farDistance);
            vc.ambient     = c.value("ambient", vc.ambient);
            vc.detailStrength = c.value("detailStrength", vc.detailStrength);
        }
        // Aerial-perspective fog (matches the offline tracer's Scene::fog). Lives
        // under "environment" alongside the sky; pushed to the renderer via the
        // lighting block (setLights). density 0 = off.
        if (env.contains("fog") && env["fog"].is_object()) {
            const auto& f = env["fog"];
            view.lighting.fog.enabled = true;
            view.lighting.fog.density = f.value("density", 0.0f);
            view.lighting.fog.heightFalloff = f.value("heightFalloff", 0.0f);
            view.lighting.fog.color =
                parseVec3(f.value("color", json()), view.lighting.fog.color);
        }
        // Bloom, level-authored (device: "the bloom is really big"): the
        // scattering sky pushed far more radiance over the default threshold
        // than the analytic sky ever did, and until now the only dials were
        // the debug HUD's. Absent keys keep the renderer defaults.
        if (env.contains("bloom") && env["bloom"].is_object()) {
            const auto& b = env["bloom"];
            renderer.bloomEnabled = b.value("enabled", renderer.bloomEnabled);
            renderer.bloomParams.threshold =
                b.value("threshold", renderer.bloomParams.threshold);
            renderer.bloomParams.knee = b.value("knee", renderer.bloomParams.knee);
            renderer.bloomParams.intensity =
                b.value("intensity", renderer.bloomParams.intensity);
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

    // Scene gravity: a top-level "gravity" [x,y,z] (m/s²) singleton PhysicsSystem
    // applies to the world. A space level sets [0,0,0] so the player floats instead
    // of falling. Absent leaves the engine default (Earth).
    if (root.contains("gravity")) {
        Entity g = world.create();
        world.add<SceneGravity>(g, SceneGravity{parseVec3(root["gravity"], Vec3(0, -9.81, 0))});
    }

    // Opt in this scene to the live Planet Lab editor panel (PlanetLabSystem draws
    // its floating window only when this singleton is present).
    if (root.value("planetLab", false)) {
        Entity e = world.create();
        world.add<ScenePlanetLab>(e, ScenePlanetLab{});
    }

    // Planetary atmosphere glow (procedural-planet-plan P3): a top-level "atmosphere"
    // block { center:[x,y,z], radius, thickness?, density?, intensity?, enabled? }
    // drives the renderer's scattering pass (Metal today; other backends ignore it).
    // thickness = shell height as a fraction of radius, density scales the scattering,
    // intensity the glow strength — turn them up for an unmistakable limb halo. Absent
    // clears it.
    if (root.contains("atmosphere")) {
        const json& at = root["atmosphere"];
        AtmosphereRenderParams ap = atmosphereParamsFor(
            parseVec3(at.value("center", json())),
            at.value("radius", 20.0f),
            at.value("thickness", 0.08f),
            at.value("density", 1.0f),
            at.value("intensity", 32.0f));
        ap.enabled = at.value("enabled", true);
        renderer.setAtmosphere(ap);
    } else {
        AtmosphereRenderParams off;
        renderer.setAtmosphere(off);   // enabled=false by default
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

    loadStage("entities + spawn");
    LOG_INFO << "Loaded level: " << path << " (v" << version << ", "
             << world.entityCount() << " entities) in " << std::chrono::duration<double>(std::chrono::steady_clock::now() - tLoad0).count() << " s";
    if (!loadStages.empty()) LOG_INFO << "[load] " << loadStages;
    return true;
}

}  // namespace engine
