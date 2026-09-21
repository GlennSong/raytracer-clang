// Every SHIPPED level is playable by construction — AGENTS.md § Playable Scenes
// ("Collidable by default", "Always a player start"), made executable.
//
// Why this file exists (docs/knowledge-retention-plan.md): the rule was written
// down, in the file everyone reads, and violated anyway — a roadlab bridge
// shipped as a Renderable with no collider. It survived a green suite because
// nothing in that suite takes a shipped level as INPUT. 18 test files assert
// things about colliders; every one of them asserts over geometry it generated
// in-process. This one opens assets/levels/*.json.
//
// It drives LevelLoader::load — the real, and only, path from a level file to a
// running world — rather than re-deriving what the loader would do. A test that
// mirrors the loader (see test_city_generated.cpp's "Mirror level_loader") is a
// second implementation, and cannot detect drift in the first.
//
// Rendering is a NullRenderer: mesh handles are fake, so nothing here may assert
// on GPU state or mesh BOUNDS (NullRenderer returns unit bounds for everything).
// Colliders are CPU-side components, which is exactly what the rule is about.

#include "test_framework.h"

#include <nlohmann/json.hpp>

#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/terrain.h"
#include "../src/engine/procgen/city/roads/road_entity.h"   // RoadEntity (signal census)
#include "../src/engine/procgen/city/building_records.h"
#include "../src/engine/procgen/city/core_plan.h"  // CityBuildings doors (ADR-0080)
#include "../src/engine/procgen/city/city_svg.h"   // CityMapData (the in-road census)
#include "../src/apps/citysim/city_render.h"        // CityRenderSystem (traffic census)
#include "../src/apps/citysim/city_map_raster.h"    // the map tool's picture
#include "../src/apps/citysim/bus_stop_props.h"     // routeColour
#include "../src/engine/procgen/city/street_signs.h"   // street name signs
#include "../src/engine/text/font.h"
#include "../src/engine/system.h"
#include "../src/engine/world.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <set>
#include <chrono>
#include <cstdlib>   // setenv (the marker test)
#include <cmath>
#include <fstream>
#include <functional>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace engine;
using json = nlohmann::json;

namespace {

std::string levelsDir() {
#ifdef RT_SOURCE_DIR
    return std::string(RT_SOURCE_DIR) + "/assets/levels";
#else
    return "assets/levels";
#endif
}

// WHICH CITY THE SIM CASES DRIVE. These cases measure BEHAVIOUR — traffic on the
// road, buses that serve the city, cars that clear a junction — against the shipped
// metro, and they say "metro" rather than "this one particular file". A second city
// now exists that is built by the other road builder (metro_lanes, ADR-0089/0090),
// whose road graph is the lanes twin rather than the lattice's sampler: a quarter
// the nodes, and every number here was tuned on the other one. RT_CITYSIM_LEVEL
// points them at it, so "does the sim work on a lane-built city" is a measurement
// rather than an opinion.
std::string simLevelPath() {
    const char* only = std::getenv("RT_CITYSIM_LEVEL");
    const std::string name = only && *only ? std::string(only) : std::string("metro_v2_test");
    return levelsDir() + "/" + name + (name.size() > 5 && name.compare(name.size() - 5, 5, ".json") == 0
                                           ? "" : ".json");
}

// Every shipped level, sorted so a failure names the same file run to run.
// `*.json.cameras.json` sidecars are camera bookmarks, not levels.
std::vector<std::string> shippedLevels() {
    std::vector<std::string> out;
    std::error_code ec;
    // Every directory of shipped levels, as paths relative to levelsDir(). The lab levels live
    // apart from the game's (ADR-0085) but are held to the same gates when the loader knows the
    // shape — a lanelab level with no collider or no ground under its spawn is as broken as any.
    std::vector<std::string> dirs = {""};
#ifdef RT_ROADS_LANES
    dirs.push_back("../lanelab/levels/");
#endif
    for (const std::string& dir : dirs)
    for (const auto& entry : std::filesystem::directory_iterator(levelsDir() + "/" + dir, ec)) {
        const std::string name = dir + entry.path().filename().string();
        if (name.size() < 5 || name.compare(name.size() - 5, 5, ".json") != 0) continue;
        if (name.find(".cameras.json") != std::string::npos) continue;
        // RT_LEVELS=a,b: only levels whose file name contains one of the substrings (timing one level).
        if (const char* only = std::getenv("RT_LEVELS"); only && *only) {
            bool keep = false; std::string list = only; size_t start = 0;
            while (start <= list.size()) { const size_t comma = list.find(',', start); const std::string sub = list.substr(start, comma == std::string::npos ? std::string::npos : comma - start); if (!sub.empty() && name.find(sub) != std::string::npos) keep = true; if (comma == std::string::npos) break; start = comma + 1; }
            if (!keep) continue;
        }
        out.push_back(name);
    }
    std::sort(out.begin(), out.end());
    return out;
}

// --- ground probe -----------------------------------------------------------
// A downward ray from the spawn: does anything COLLIDABLE sit under the player?
// Not a Jolt cast — this binary is the Jolt-free one — but the same question
// asked of the collider components the loader produced, which is where the rule
// bites ("gets a collider in the SAME recipe that makes its geometry").

// Ground is allowed to sit slightly above the spawn point: an authored spawn can
// be a few cm inside a kerb or a sidewalk slab without the level being broken.
constexpr double GROUND_ABOVE_SPAWN_TOLERANCE = 1.0;

bool pointInTriangleXZ(double px, double pz, const Vec3& a, const Vec3& b,
                       const Vec3& c) {
    const double d1 = (px - b.x) * (a.z - b.z) - (a.x - b.x) * (pz - b.z);
    const double d2 = (px - c.x) * (b.z - c.z) - (b.x - c.x) * (pz - c.z);
    const double d3 = (px - a.x) * (c.z - a.z) - (c.x - a.x) * (pz - a.z);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

// The probe accumulates every collidable surface over the spawn's XZ and sorts
// them into two buckets. The distinction matters: a level can have a tree canopy
// or an overpass above the spawn AND solid ground below it, and only the ground
// answers the question. Taking the highest surface and testing that one would
// fail such a level — the ground is there, it is just not the top hit.
struct Probe {
    bool ground = false;         // a surface at or below the spawn: real footing
    double groundTop = 0;        // the highest of those (what you land on)
    bool ceiling = false;        // a surface ABOVE the spawn and nothing below
    double lowestCeiling = 0;    // the closest one, for the failure message
};

void addSurface(Probe& p, double top, double spawnY) {
    if (top <= spawnY + GROUND_ABOVE_SPAWN_TOLERANCE) {
        if (!p.ground || top > p.groundTop) p.groundTop = top;
        p.ground = true;
    } else {
        if (!p.ceiling || top < p.lowestCeiling) p.lowestCeiling = top;
        p.ceiling = true;
    }
}

// A MeshCollider owns CPU triangles in WORLD space (components.h), so no
// transform is applied here. The surface height is the triangle INTERPOLATED at
// the spawn's XZ, not the triangle's highest vertex: on steep terrain one
// triangle can span several metres vertically, and taking its peak would report
// ground metres above where the actor would actually stand.
void meshColliderProbe(const MeshCollider& mc, const Vec3& spawn, Probe& p) {
    for (std::size_t i = 0; i + 2 < mc.indices.size(); i += 3) {
        const Vec3& a = mc.vertices[mc.indices[i]];
        const Vec3& b = mc.vertices[mc.indices[i + 1]];
        const Vec3& c = mc.vertices[mc.indices[i + 2]];
        if (!pointInTriangleXZ(spawn.x, spawn.z, a, b, c)) continue;
        const double den = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
        if (std::fabs(den) < 1e-12) continue;   // edge-on in XZ: no surface here
        const double w0 = ((b.z - c.z) * (spawn.x - c.x) +
                           (c.x - b.x) * (spawn.z - c.z)) / den;
        const double w1 = ((c.z - a.z) * (spawn.x - c.x) +
                           (a.x - c.x) * (spawn.z - c.z)) / den;
        addSurface(p, w0 * a.y + w1 * b.y + (1.0 - w0 - w1) * c.y, spawn.y);
    }
}

// Primitive colliders. Orientation is ignored: a rotated box's AABB is a
// superset, so the XZ overlap test can only ever be GENEROUS — it will not fail
// a level that really does have ground, which is the failure mode that matters
// for a gate that must stay honest.
void primitiveColliderProbe(const Collider& col, const Transform& t,
                            const Vec3& spawn, Probe& p) {
    double halfX = 0, halfZ = 0, top = 0;
    switch (col.shape) {
        case ColliderShape::Box:
            halfX = col.halfExtent.x * t.scale.x;
            halfZ = col.halfExtent.z * t.scale.z;
            top = t.position.y + col.halfExtent.y * t.scale.y;
            break;
        case ColliderShape::Sphere:
            halfX = halfZ = col.radius * t.scale.x;
            top = t.position.y + col.radius * t.scale.y;
            break;
        case ColliderShape::Capsule:
            halfX = halfZ = col.radius * t.scale.x;
            top = t.position.y + (col.halfHeight + col.radius) * t.scale.y;
            break;
    }
    if (std::fabs(spawn.x - t.position.x) > halfX) return;
    if (std::fabs(spawn.z - t.position.z) > halfZ) return;
    addSurface(p, top, spawn.y);
}

// --- one loaded level -------------------------------------------------------

struct LevelFacts {
    std::string name;
    bool loaded = false;
    int players = 0;           // CharacterController entities (the actor)
    int colliders = 0;         // Collider + MeshCollider components
    bool groundUnderSpawn = false;
    std::string groundSource;  // what answered the probe, for the failure message
    double loadSeconds = 0;

    // FLOORPLAN CONFORMANCE CENSUS (device: "prove that the entire floorplan
    // of the building is conformed to the surface"): walked along every
    // building prism's perimeter against the mesh-equivalent ground (leaf
    // dilate + FIX-A clamp replay). Burial = ground swallowing a wall above
    // its plinth; gap = ground falling below the pad by more than the
    // foundation block's guaranteed reach (drape bound + bed-in).
    int censusLots = 0;
    int censusBurials = 0;
    int censusGaps = 0;
    double censusWorst = 0;    // metres, worst |deviation|
    double censusWorstX = 0, censusWorstZ = 0;

    // --- the two conditions under which the rule does not apply -------------
    // Neither is an allowlist of level names. Both are properties of the level
    // itself, so a level cannot drift into an exemption: turn gravity back on,
    // or drop the CDLOD block, and the requirement returns.

    // A scene with zero gravity has no "down" for an actor to fall in. The five
    // planet levels are deliberate zero-g space scenes ("gravity": [0,0,0]) —
    // nothing to stand on is the design, not a missing collider.
    bool weightless = false;
    // CDLOD terrain streams its chunk colliders in around the player, so at load
    // time the collider under the spawn does not exist yet
    // (test_terrain_lod.cpp `terrain_chunk_collider_flag_by_radius`). The
    // surface is real and analytic; `groundUnderSpawn` still answers from it.
    bool streamsTerrainColliders = false;

    // Diagnostics for the failure message. `surfaceTopAtSpawn` is the ground
    // surface when there is one, and otherwise the LOWEST surface above the
    // spawn — which is what tells you the actor is buried rather than floating.
    Vec3 spawn;
    bool anyColliderOverlapsSpawnXZ = false;
    double surfaceTopAtSpawn = 0;

    // THE SPAWN IS NOT INSIDE A BUILDING. Every lot building is a solid plan
    // prism in the district collider; a spawn inside one is a capsule inside
    // a static mesh (device: "I'm stuck inside of a building for the player
    // start"). The CDLOD branch above cannot see this — a prism has no bottom
    // face, so the collider probe finds only a ceiling — so it is asked of
    // the plan prisms directly.
    bool spawnInsidePrism = false;
    std::string spawnInsideType;
    // ADR-0080 doors: every enterable record's aperture must open OUTWARD
    // onto clear ground (foot + 1.5n outside every prism) from a real wall
    // (foot - 0.5n inside its own plan), and the spawn belongs at the door.
    int enterableDoors = 0;
    int enterableDoorsOutsideOk = 0;
    int enterableDoorsInsideOk = 0;
    double spawnToEnterableDoor = 1e300;   // XZ metres, nearest enterable foot
    double enterableDoorX = 0, enterableDoorZ = 0, enterableDoorNX = 0,
           enterableDoorNZ = 0;            // first enterable foot + normal
};

LevelFacts inspect(const std::string& name) {
    LevelFacts f;
    f.name = name;

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;

    const auto t0 = std::chrono::steady_clock::now();
    f.loaded = LevelLoader::load(levelsDir() + "/" + name, world, *renderer, view,
                                 assets, /*editorMode=*/false);
    f.loadSeconds = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
    if (!f.loaded) return f;

    Vec3 spawn(0, 0, 0);
    world.each<CharacterController, Transform>(
        [&](Entity, CharacterController&, Transform& t) {
            if (f.players == 0) spawn = t.position;
            ++f.players;
        });
    f.spawn = spawn;

    world.each<Collider>([&](Entity, Collider&) { ++f.colliders; });
    world.each<MeshCollider>([&](Entity, MeshCollider&) { ++f.colliders; });

    world.each<SceneGravity>([&](Entity, SceneGravity& g) {
        if (g.value.length() < 1e-6) f.weightless = true;
    });
    world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig&) { f.streamsTerrainColliders = true; });

    if (f.players == 0) return f;

    Probe probe;
    world.each<MeshCollider>([&](Entity, MeshCollider& mc) {
        const bool had = probe.ground;
        meshColliderProbe(mc, spawn, probe);
        if (!had && probe.ground) f.groundSource = "MeshCollider";
    });
    world.each<Collider, Transform>([&](Entity, Collider& c, Transform& t) {
        const bool had = probe.ground;
        primitiveColliderProbe(c, t, spawn, probe);
        if (!had && probe.ground) f.groundSource = "Collider";
    });
    f.groundUnderSpawn = probe.ground;
    f.anyColliderOverlapsSpawnXZ = probe.ground || probe.ceiling;
    f.surfaceTopAtSpawn = probe.ground ? probe.groundTop : probe.lowestCeiling;

    world.each<CityPlanDebug>([&](Entity, CityPlanDebug& plan) {
        for (const CityPlanDebug::Prism& pr : plan.prisms) {
            if (pr.plan.size() < 3) continue;
            if (spawn.y < pr.y0 - 0.5 || spawn.y > pr.y1 + 0.5) continue;
            if (pointInPolygon(pr.plan, Vec2(spawn.x, spawn.z))) {
                f.spawnInsidePrism = true;
                f.spawnInsideType = pr.type;
                return;
            }
        }
    });

    // ADR-0080: audit every enterable record's doors against the prisms.
    world.each<CityBuildings>([&](Entity, CityBuildings& cb) {
        for (const BuildingRecord& r : cb.records) {
            if (!r.enterable) continue;
            for (const DoorSpec& d : r.doors) {
                if (f.enterableDoors == 0) {
                    f.enterableDoorX = d.foot.x;
                    f.enterableDoorZ = d.foot.y;
                    f.enterableDoorNX = d.normal.x;
                    f.enterableDoorNZ = d.normal.y;
                }
                ++f.enterableDoors;
                const Vec2 outP = d.foot + d.normal * 1.5;
                const Vec2 inP = d.foot - d.normal * 0.5;
                bool outClear = true;
                world.each<CityPlanDebug>([&](Entity, CityPlanDebug& plan) {
                    for (const CityPlanDebug::Prism& pr : plan.prisms)
                        if (pr.plan.size() >= 3 &&
                            pointInPolygon(pr.plan, outP)) {
                            outClear = false;
                            return;
                        }
                });
                if (outClear) ++f.enterableDoorsOutsideOk;
                if (r.plan.size() >= 3 && pointInPolygon(r.plan, inP))
                    ++f.enterableDoorsInsideOk;
                const double dx = spawn.x - d.foot.x, dz = spawn.z - d.foot.y;
                f.spawnToEnterableDoor = std::min(
                    f.spawnToEnterableDoor, std::sqrt(dx * dx + dz * dz));
            }
        }
    });

    // CDLOD terrain: the chunk collider under the spawn has not streamed in yet,
    // but the surface is analytic and the loader snaps the spawn to it. So the
    // question becomes the one that can actually regress — is the spawn ABOVE
    // the terrain surface rather than buried inside a hill?
    if (!f.groundUnderSpawn) {
        world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& cfg) {
            if (f.groundUnderSpawn) return;
            Noise noise(cfg.seed);
            const double surface =
                terrainHeight(cfg.params, noise, spawn.x, spawn.z);
            if (spawn.y >= surface - GROUND_ABOVE_SPAWN_TOLERANCE) {
                f.groundUnderSpawn = true;
                f.groundSource = "CDLOD terrain";
            }
        });
    }
    // --- the census ---------------------------------------------------------
    world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& cfg) {
        world.each<CityPlanDebug>([&](Entity, CityPlanDebug& plan) {
            if (plan.prisms.empty()) return;
            Noise noise(cfg.seed);
            const double leafSize =
                (2.0 * cfg.worldHalf) /
                static_cast<double>(1 << (cfg.numLods - 1));
            const double leafStep = leafSize / std::max(1, cfg.gridRes);
            const double dilate = leafStep * 1.45;
            auto meshGround = [&](double x, double z) {
                double y = terrainHeight(cfg.params, noise, x, z, dilate);
                if (cfg.params.flattenIndex) {
                    const double rp =
                        roadPlaneNear(*cfg.params.flattenIndex,
                                      cfg.params.flatten, x, z, leafStep * 1.6);
                    if (rp < 1e29 &&
                        !padPlaneAbove(*cfg.params.flattenIndex,
                                       cfg.params.flatten, x, z, rp, dilate))
                        y = std::min(y, rp);
                }
                return y;
            };
            // plinth from the level JSON (the prism base = baseY - 0.5).
            double plinth = 0.15;
            {
                std::ifstream jf(levelsDir() + "/" + name);
                if (jf) {
                    json root = json::parse(jf, nullptr, false);
                    if (root.is_object() && root.contains("citysim"))
                        plinth = root["citysim"].value("plinth", 0.15);
                }
            }
            for (const auto& prism : plan.prisms) {
                if (prism.plan.size() < 3) continue;
                ++f.censusLots;
                const double baseY = prism.y0 + 0.5;
                const double groundY = baseY - plinth;
                bool buried = false, gapped = false;
                for (std::size_t i = 0; i < prism.plan.size(); ++i) {
                    const Vec2& a = prism.plan[i];
                    const Vec2& b = prism.plan[(i + 1) % prism.plan.size()];
                    const int steps = std::max(
                        1, static_cast<int>(std::ceil((b - a).length())));
                    for (int st = 0; st <= steps; ++st) {
                        const Vec2 q =
                            a + (b - a) * (static_cast<double>(st) / steps);
                        const double g = meshGround(q.x, q.y);
                        const double burial = g - baseY;
                        // The foundation block drapes to min perimeter ground
                        // - 0.5; anything deeper than the relief gate + drape
                        // means the field diverged from what growth saw.
                        const double gap = (groundY - g) - 9.0;
                        if (burial > 0.55) {
                            buried = true;
                            if (burial > f.censusWorst) {
                                f.censusWorst = burial;
                                f.censusWorstX = q.x;
                                f.censusWorstZ = q.y;
                            }
                        }
                        if (gap > 0) {
                            gapped = true;
                            if (gap > f.censusWorst) {
                                f.censusWorst = gap;
                                f.censusWorstX = q.x;
                                f.censusWorstZ = q.y;
                            }
                        }
                    }
                }
                // NAME EVERY OFFENDER, not just the worst. The census used to
                // report a count and one position, which says a defect exists
                // somewhere but not WHICH building -- so a walk that finds a
                // broken lot cannot be matched against it (Glenn flew to one
                // and the worst-case coordinate was 600 m away). Each is
                // printed with a teleport-ready position.
                if (buried || gapped) {
                    const Vec2 c = centroid(prism.plan);
                    std::printf("    [census]   %s %s at (%.1f, %.1f)"
                                "  ->  teleport %.2f %.2f\n",
                                f.name.c_str(),
                                buried && gapped ? "BURIED+GAP" : (buried ? "BURIED" : "GAP"),
                                c.x, c.y, c.x, c.y);
                }
                if (buried) ++f.censusBurials;
                if (gapped) ++f.censusGaps;
            }
        });
    });
    return f;
}

// Loading 50-odd levels means running their procgen, which is the expensive part
// of this file. Do it ONCE and let each case read the facts.
const std::vector<LevelFacts>& allLevels() {
    static const std::vector<LevelFacts> facts = [] {
        std::vector<LevelFacts> v;
        double total = 0;
        for (const std::string& name : shippedLevels()) {
            v.push_back(inspect(name));
            total += v.back().loadSeconds;
        }
        for (const LevelFacts& f : v) std::printf("    [levels] %s: %.1fs\n", f.name.c_str(), f.loadSeconds);
        std::printf("    [levels] loaded %zu levels in %.1fs\n", v.size(), total);
        return v;
    }();
    return facts;
}

}  // namespace

TEST_CASE(every_shipped_level_loads) {
    const std::vector<LevelFacts>& levels = allLevels();
    CHECK(!levels.empty());   // the glob itself must find the levels
    for (const LevelFacts& f : levels) {
        if (!f.loaded) std::printf("    level '%s' failed to load\n", f.name.c_str());
        CHECK(f.loaded);
    }
}

// "Always a player start": the loader guarantees an actor even when the level
// authors no "player" block (it drops a default in from above). Exactly one —
// two actors means two things are reading input.
TEST_CASE(every_shipped_level_has_exactly_one_player) {
    for (const LevelFacts& f : allLevels()) {
        if (!f.loaded) continue;
        if (f.players != 1)
            std::printf("    level '%s' has %d players\n", f.name.c_str(), f.players);
        CHECK(f.players == 1);
    }
}

// "Collidable by default": a level with no collider at all is a diorama.
TEST_CASE(every_shipped_level_has_a_collider) {
    for (const LevelFacts& f : allLevels()) {
        if (!f.loaded) continue;
        if (f.weightless) continue;                // nothing falls; see LevelFacts
        if (f.streamsTerrainColliders) continue;   // chunks arrive around the player
        if (f.colliders == 0)
            std::printf("    level '%s' has NO colliders\n", f.name.c_str());
        CHECK(f.colliders > 0);
    }
}

// The rule's own predicted symptom: "flying over it hides that the actor is
// falling through everything." This is that flight, taken away.
TEST_CASE(every_shipped_level_has_collidable_ground_under_the_spawn) {
    for (const LevelFacts& f : allLevels()) {
        if (!f.loaded || f.players == 0) continue;
        if (f.weightless) continue;
        if (!f.groundUnderSpawn) {
            if (f.anyColliderOverlapsSpawnXZ)
                std::printf("    level '%s': spawn y=%.2f is BELOW the collider "
                            "over it (top y=%.2f) — spawned inside the geometry\n",
                            f.name.c_str(), f.spawn.y, f.surfaceTopAtSpawn);
            else
                std::printf("    level '%s': nothing collidable under the spawn "
                            "(%.1f, %.1f, %.1f)\n", f.name.c_str(), f.spawn.x,
                            f.spawn.y, f.spawn.z);
        }
        CHECK(f.groundUnderSpawn);
    }
}


// The spawn stands OUTSIDE every building (device: "I'm stuck inside of a
// building for the player start"). metro_v2's authored spawn sat inside a
// 417 m2 building 17.6 m from its street; the loader now walks such a spawn
// out to the street and warns, and this asks the plan prisms directly — the
// only probe that can see a prism, which has no bottom face.
TEST_CASE(every_shipped_level_spawns_outside_every_building) {
    for (const LevelFacts& f : allLevels()) {
        if (!f.loaded || f.players == 0) continue;
        if (f.spawnInsidePrism)
            std::printf("    level '%s': spawn (%.1f, %.1f, %.1f) is INSIDE a %s building\n",
                        f.name.c_str(), f.spawn.x, f.spawn.y, f.spawn.z,
                        f.spawnInsideType.c_str());
        CHECK(!f.spawnInsidePrism);
    }
}

// ADR-0080: the metro's spawn building is ENTERABLE — it has a real door
// whose aperture opens outward onto clear ground from a real wall, and the
// spawn is authored AT that door (the door foot is printed so re-authoring
// is a copy-paste).
TEST_CASE(metro_spawn_building_has_a_walkable_door) {
    bool sawMetro = false;
    for (const LevelFacts& f : allLevels()) {
        if (f.name != "metro_v2_test.json" || !f.loaded) continue;
        sawMetro = true;
        std::printf("    [doors] enterable doors=%d outsideOk=%d insideOk=%d "
                    "spawn->door=%.2f m; door foot (%.2f, %.2f) n (%.2f, %.2f)\n",
                    f.enterableDoors, f.enterableDoorsOutsideOk,
                    f.enterableDoorsInsideOk, f.spawnToEnterableDoor,
                    f.enterableDoorX, f.enterableDoorZ, f.enterableDoorNX,
                    f.enterableDoorNZ);
        CHECK(f.enterableDoors >= 1);
        CHECK(f.enterableDoorsOutsideOk == f.enterableDoors);
        CHECK(f.enterableDoorsInsideOk == f.enterableDoors);
        // The spawn stands at its building's door (AGENTS.md: place it
        // deliberately). Re-author player.position when this trips.
        CHECK(f.spawnToEnterableDoor < 3.5);
    }
    CHECK(sawMetro);
}

// THE GATE (floorplan-conformance round): every building's floorplan meets
// the drawn ground — no wall buried past its plinth, no daylight beyond the
// foundation's reach — on every level that grows buildings on terrain.
TEST_CASE(level_census_every_floorplan_conforms_to_the_drawn_ground) {
    int lotLevels = 0;
    for (const LevelFacts& f : allLevels()) {
        if (f.censusLots == 0) continue;
        ++lotLevels;
        if (f.censusBurials || f.censusGaps)
            std::printf(
                "    [census] %s: lots=%d burials=%d gaps=%d worst=%.2f m at "
                "(%.1f, %.1f)\n",
                f.name.c_str(), f.censusLots, f.censusBurials, f.censusGaps,
                f.censusWorst, f.censusWorstX, f.censusWorstZ);
        CHECK(f.censusBurials == 0);
        CHECK(f.censusGaps == 0);
    }
    std::printf("    [census] %d levels carry lot buildings\n", lotLevels);
    CHECK(lotLevels >= 5);
}

// THE MARKER TEST (ground-probes round, device: "if we place these markers in
// metro_v2_test they should all adhere to the ground — is there some way we
// could do that as a test?"): RT_GROUND_PROBES plants a post at every ~29 m
// grid point whose base is the analytic terrainHeight, and the loader scores
// each one against the finest rendered tile's own bilinear interpolation —
// the surface the player stands on. This case flips the env on, loads the
// shipped metro through the REAL loader, and asserts the histogram the run
// prints. Measured at adoption: 9409 probes, 95.5% flush, worst 10.5 m — the
// off probes are a thin seam along freeway bench cuts, where dilation smears
// the cut edge across one cell (docs/TECH_DEBT.md, "map vs the territory").
// The bounds hold that seam where it is; the flush floor rises if placement
// ever regresses to reading a stale surface again.
// A LOCATOR for headless frames (print-only, RT_PRINT_CORES=1): the three
// tallest cored buildings of each selected level with the world points a
// shot needs — hoistway 0's door foot and normal, the lobby in front of
// the bank, the same spot on the top storey — so a tower interior can be
// framed with RT_SPAWN and lanelab_shots.py without a walk.
// BUILDINGS IN THE ROAD (Glenn, 2026-09-17: "one lot being built in the middle
// of a street which is placing trees in the road"). The lot pass has a road
// clearance test, but every consumer downstream reads `planOk ? plan : site`,
// and `site` is the raw parcel polygon that clearance never touched -- so a lot
// whose plan fails for ANY reason can still build from an unvetted footprint.
// This asks the finished city the question directly: does any building's plan
// stand inside a carriageway? Print-only for now: it names offenders so the
// path that produced them can be traced, before it becomes a gate.
TEST_CASE(level_print_buildings_in_the_carriageway) {
    for (const LevelFacts& f : allLevels()) {
        if (!f.loaded) continue;
        std::unique_ptr<Renderer> renderer = Renderer::create();
        RendererMeshUploader uploader(*renderer);
        AssetManager assets(uploader);
        World world;
        RenderView view;
        if (!LevelLoader::load(levelsDir() + "/" + f.name, world, *renderer, view, assets, false))
            continue;
        const CityBuildings* cb = nullptr;
        world.each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
        const CityMap* cm = nullptr;
        world.each<CityMap>([&](Entity, CityMap& m) { if (!cm) cm = &m; });
        if (!cb || !cm || !cm->data) continue;
        const RoadGraph& rg = cm->data->roads;
        auto segDist = [](const Vec2& p, const Vec2& a, const Vec2& b) {
            const Vec2 ab = b - a;
            const Real l2 = ab.x * ab.x + ab.y * ab.y;
            if (l2 < 1e-9) return (p - a).length();
            Real t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;
            t = std::max(Real(0), std::min(Real(1), t));
            return (p - (a + ab * t)).length();
        };
        int offenders = 0;
        Real worst = 0; Vec2 worstAt(0, 0); std::size_t worstRec = 0;
        for (std::size_t i = 0; i < cb->records.size(); ++i) {
            const BuildingRecord& r = cb->records[i];
            if (r.plan.size() < 3) continue;
            bool bad = false;
            for (const Vec2& v : r.plan) {
                for (std::size_t e = 0; e < rg.edges.size(); ++e) {
                    const RoadEdge& ed = rg.edges[e];
                    if (ed.a < 0 || ed.b < 0) continue;
                    const Real d = segDist(v, rg.nodes[ed.a].pos, rg.nodes[ed.b].pos);
                    const Real into = ed.width * Real(0.5) - d;   // >0 = inside the lane
                    if (into > Real(0.25)) {
                        bad = true;
                        if (into > worst) { worst = into; worstAt = v; worstRec = i; }
                    }
                }
            }
            if (bad) ++offenders;
        }
        std::printf("    [in-road] %s: %d of %zu buildings stand in a carriageway; worst %.2f m "
                    "into the lane at (%.1f, %.1f), record %zu\n",
                    f.name.c_str(), offenders, cb->records.size(), worst, worstAt.x, worstAt.y, worstRec);
    }
}

TEST_CASE(level_print_tower_cores) {
    if (!std::getenv("RT_PRINT_CORES")) return;
    for (const std::string& name : shippedLevels()) {
        std::unique_ptr<Renderer> renderer = Renderer::create();
        RendererMeshUploader uploader(*renderer);
        AssetManager assets(uploader);
        World world;
        RenderView view;
        if (!LevelLoader::load(levelsDir() + "/" + name, world, *renderer, view, assets, false)) continue;
        struct Tower { std::size_t idx; int floors; };
        std::vector<Tower> towers;
        const CityBuildings* cbp = nullptr;
        world.each<CityBuildings>([&](Entity, CityBuildings& cb) { cbp = &cb; });
        if (!cbp) continue;
        for (std::size_t i = 0; i < cbp->records.size(); ++i) {
            const BuildingRecord& r = cbp->records[i];
            if (!r.enterable || !wantsCore(r.params)) continue;
            if (!coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params)).valid) continue;
            towers.push_back({i, r.params.floors});
        }
        // PODIUM + TOWER lots: an enterable cored record with a taller record
        // rooted above it inside its plan (Glenn's second walk: "very tall but
        // only 4 floors on the elevator").
        for (std::size_t i = 0; i < cbp->records.size(); ++i) {
            const BuildingRecord& r = cbp->records[i];
            if (!r.enterable) continue;
            for (std::size_t j = 0; j < cbp->records.size(); ++j) {
                const BuildingRecord& t = cbp->records[j];
                if (j == i || t.baseY < r.baseY + 2.0 || t.plan.size() < 3) continue;
                if (!pointInPolygon(r.plan, centroid(t.plan))) continue;
                const CorePlan pc = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
                std::printf("[podium] %s record %zu (%d floors, baseY %.2f, core %d) under record %zu (%d floors, baseY %.2f); door foot (%.2f, %.2f) normal (%.2f, %.2f)\n",
                            name.c_str(), i, r.params.floors, r.baseY, pc.valid ? 1 : 0, j, t.params.floors, t.baseY,
                            r.doors.empty() ? 0.0 : r.doors[0].foot.x, r.doors.empty() ? 0.0 : r.doors[0].foot.y,
                            r.doors.empty() ? 0.0 : r.doors[0].normal.x, r.doors.empty() ? 0.0 : r.doors[0].normal.y);
                if (pc.valid) {
                    const Vec2 lob = pc.hoistways[0].doorFoot() + pc.hoistways[0].doorNormal() * 3.0;
                    std::printf("[podium]   lobby spot (%.2f, %.2f) y %.2f; hoistway door foot (%.2f, %.2f)\n", lob.x, lob.y, r.baseY + 0.9,
                                pc.hoistways[0].doorFoot().x, pc.hoistways[0].doorFoot().y);
                }
            }
        }
        // Two enterable RESIDENTIAL (masonry) buildings of 3+ floors, for
        // window-alignment frames (RT_SPAWN on an upper storey).
        int resPrinted = 0;
        for (std::size_t i = 0; i < cbp->records.size() && resPrinted < 2; ++i) {
            const BuildingRecord& r = cbp->records[i];
            if (!r.enterable || r.params.curtainWall || r.params.solidFacade || r.params.floors < 3 || r.doors.empty()) continue;
            if (r.plan.size() != 4) continue;
            const std::vector<StoreyPlan> st = storeyPlans(r.plan, r.params);
            if (st.size() < 3) continue;
            const Vec2 c = centroid(r.plan);
            std::printf("[resi] %s record %zu (%s): %d floors baseY %.2f; door foot (%.2f, %.2f) normal (%.2f, %.2f); centroid (%.2f, %.2f); storey 1 floor y %.2f h %.2f\n",
                        name.c_str(), i, r.recipe.c_str(), r.params.floors, r.baseY, r.doors[0].foot.x, r.doors[0].foot.y,
                        r.doors[0].normal.x, r.doors[0].normal.y, c.x, c.y, r.baseY + st[1].y0, st[1].h);
            ++resPrinted;
        }
        for (std::size_t i = 0; i < cbp->records.size(); ++i) {
            const BuildingRecord& r = cbp->records[i];
            if (r.recipe != "podium_tower" || r.params.floors < 20) continue;
            const CorePlan pc = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
            const std::vector<MassTier> tiers = massStack(r.plan, r.params);
            Real minSide = 1e9;
            for (const MassTier& t : tiers) {
                const OBB2 ob = orientedBoundingBox(t.plan);
                minSide = std::min(minSide, 2.0 * std::min(ob.half[0], ob.half[1]));
            }
            std::printf("[podium-one] %s record %zu: %d floors enterable %d envelope %d towerFrac %.2f tiers %zu minSide %.1f core %d doors %zu at (%.1f, %.1f)\n",
                        name.c_str(), i, r.params.floors, r.enterable ? 1 : 0, static_cast<int>(r.params.envelope),
                        r.params.towerFrac, tiers.size(), minSide, pc.valid ? 1 : 0, r.doors.size(),
                        centroid(r.plan).x, centroid(r.plan).y);
            if (pc.valid) {
                const Vec2 lob = pc.hoistways[0].doorFoot() + pc.hoistways[0].doorNormal() * 3.0;
                const std::vector<StoreyPlan> st = storeyPlans(r.plan, r.params);
                std::printf("[podium-one]   lobby spot (%.2f, %.2f) y %.2f; hoistway door foot (%.2f, %.2f) normal (%.2f, %.2f); top storey y %.2f\n",
                            lob.x, lob.y, r.baseY + 0.9, pc.hoistways[0].doorFoot().x, pc.hoistways[0].doorFoot().y,
                            pc.hoistways[0].doorNormal().x, pc.hoistways[0].doorNormal().y,
                            r.baseY + st[static_cast<std::size_t>(r.params.floors - 1)].y0);
            }
        }
        std::sort(towers.begin(), towers.end(), [](const Tower& a, const Tower& b) { return a.floors > b.floors; });
        for (std::size_t k = 0; k < towers.size() && k < 3; ++k) {
            const BuildingRecord& r = cbp->records[towers[k].idx];
            const CorePlan core = coreFor(r.plan, r.params, entranceEdgeFor(r.plan, r.params));
            const CoreShaft& hw = core.hoistways[0];
            const Vec2 foot = hw.doorFoot(), n = hw.doorNormal();
            const Vec2 lobby = foot + n * 3.0;
            const std::vector<StoreyPlan> st = storeyPlans(r.plan, r.params);
            const int top = r.params.floors - 1;
            const Real yTop = r.baseY + st[static_cast<std::size_t>(top)].y0;
            std::printf("[cores] %s record %zu: %d floors baseY %.2f groundH %.2f floorH %.2f hoistways %zu\n",
                        name.c_str(), towers[k].idx, r.params.floors, r.baseY, r.params.groundHeight,
                        r.params.floorHeight, core.hoistways.size());
            std::printf("[cores]   door foot (%.2f, %.2f) normal (%.2f, %.2f); lobby spot (%.2f, %.2f) y %.2f; top storey y %.2f (storey %d)\n",
                        foot.x, foot.y, n.x, n.y, lobby.x, lobby.y, r.baseY + 0.9, yTop, top);
            const Vec2 c = core.frame.toWorld({core.length * 0.5, core.depth * 0.5});
            std::printf("[cores]   core centre (%.2f, %.2f) length %.1f depth %.1f; plan centroid (%.2f, %.2f)\n",
                        c.x, c.y, core.length, core.depth, centroid(r.plan).x, centroid(r.plan).y);
        }
    }
}

TEST_CASE(metro_ground_probes_adhere_between_the_seams) {
    setenv("RT_GROUND_PROBES", "1", 1);
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    unsetenv("RT_GROUND_PROBES");   // never leaks into the census cases
    CHECK(loaded);
    if (!loaded) return;

    const LevelLoader::GroundProbeReport& r = LevelLoader::lastGroundProbeReport();
    std::printf("    [probes] %d planted: %d flush, %d near, %d off; "
                "worst %.2f m at (%.1f, %.1f)\n",
                r.total, r.flush, r.nearMiss, r.off, r.worst, r.worstX, r.worstZ);
    CHECK(r.total > 5000);                          // the fixture must bite
    CHECK(r.flush >= r.total * 94 / 100);           // adherence is the norm
    CHECK(r.off <= r.total * 3 / 100);              // seams stay seams
    CHECK(std::fabs(r.worst) < 20.0);               // bounded by bench depth
}

// THE POKE GATE. RT_POKE_REPORT=1 builds the dense deck-vs-drawn-terrain map at
// LOD 0/1/2 through the loader's FINAL flatten set and the exact per-LOD grid
// formula, so its numbers are the game's numbers. It is the measurement that
// killed the DaylightBatter earthwork (road_net.cpp: 0.00% -> 0.14% poke) —
// and until now it only PRINTED; nothing failed when it regressed. The terrain-
// earthwork plan changes the ground under every road, so this is the number
// that must not move.
//
// MEASURED 2026-08-29 on the shipped metro_v2 (241,530 samples per level):
// LOD0 126 pokes (0.052%, worst 3.51 m at -501,-585), LOD1 1424 (0.59%, worst
// 4.73 m at -841.6,-388.1), LOD2 2433 (1.0%, worst 6.61 m at -861.9,-401.3).
// docs/metropolis-scale-plan.md's "CLOSED: 0/184,688" is from an older level
// state and is STALE — and the LOD1/LOD2 worst sites sit 30-40 m from the
// T-junction the device reported as a cliff (-877,-423). The gate is therefore
// "no worse than today", to be tightened toward zero as the earthwork lands;
// it must not be loosened.
TEST_CASE(metro_road_decks_are_never_poked_by_the_drawn_terrain) {
    setenv("RT_POKE_REPORT", "1", 1);
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    unsetenv("RT_POKE_REPORT");
    CHECK(loaded);
    if (!loaded) return;

    const LevelLoader::PokeReport& r = LevelLoader::lastPokeReport();
    for (int l = 0; l < r.lods; ++l)
        std::printf("    [poke] LOD %d: %ld/%ld poke, worst %.2f m at (%.1f, %.1f)\n",
                    l, r.pokes[l], r.samples[l], r.worst[l], r.worstX[l], r.worstZ[l]);
    CHECK(r.lods == 3);                 // the report ran for every level
    CHECK(r.samples[0] > 100000);       // the fixture must bite
    // The measured baseline (above). Tighten these, never loosen them.
    CHECK(r.pokes[0] <= 126);
    CHECK(r.pokes[1] <= 1424);
    CHECK(r.pokes[2] <= 2433);
    CHECK(r.worst[0] <= 3.6);
}

// THE SIGNAL CENSUS (device: "why do stoplights show up in the middle of the
// street and not on the corners?"). planStreetFurniture backs each pole off
// its nav node by (widest half-width + sidewalk + knot spread + curbGap), and
// test_street_furniture proves that clears the pad disc on a synthetic cross.
// This case asks the SHIPPED metro instead: for every pole the loader planted,
// is its foot inside any drawn junction pad or carriageway ribbon? The pad
// model here is the mesher's own (road_net.cpp buildRoadNetLattice): disc
// radius rad[node] = max incident (width/2 + look.sidewalk), and two junctions
// whose connecting chain is shorter than rad[a]+rad[b]+0.25 fuse into ONE
// compound pad that also owns the asphalt between them. The nav graph fuses
// only within NavBuildParams::junctionMergeRadius (7 m) — so a pole planned
// for the connector of a mesher-merged pair stands in the pad's middle.
TEST_CASE(metro_signal_poles_stand_off_the_asphalt) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;

    Real sidewalk = 0;
    world.each<engine::RoadEntity>([&](Entity, engine::RoadEntity& net) {
        sidewalk = std::max(sidewalk, static_cast<Real>(net.look.sidewalk));
    });
    const RoadGraph* graph = nullptr;
    world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph& g) {
        if (!graph) graph = &g.graph;
    });
    const StreetFurniture* sf = nullptr;
    world.each<engine::StreetFurniture>([&](Entity, engine::StreetFurniture& f) {
        if (!sf) sf = &f;
    });
    CHECK(graph != nullptr);
    CHECK(sf != nullptr);
    if (!graph || !sf) return;
    const RoadGraph& g = *graph;
    const int N = static_cast<int>(g.nodes.size());

    // The mesher's junction model.
    std::vector<int> deg(N, 0);
    std::vector<double> rad(N, 0.0);
    std::vector<std::vector<int>> adj(N);
    for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (e.a < 0 || e.b < 0 || e.a >= N || e.b >= N) continue;
        ++deg[e.a]; ++deg[e.b];
        adj[e.a].push_back(static_cast<int>(ei));
        adj[e.b].push_back(static_cast<int>(ei));
        const double r = e.width * 0.5 + sidewalk;
        rad[e.a] = std::max(rad[e.a], r);
        rad[e.b] = std::max(rad[e.b], r);
    }
    // Chains between junction nodes (walk through degree-2 curve samples);
    // pairs shorter than rad[a]+rad[b]+0.25 are one COMPOUND pad.
    struct Pair { int a, b; double len; };
    std::vector<Pair> compound;
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 3) continue;
        for (int ei : adj[v]) {
            int prev = v;
            const RoadEdge* e = &g.edges[ei];
            int cur = e->a == v ? e->b : e->a;
            double len = (g.nodes[cur].pos - g.nodes[v].pos).length();
            int guard = 0;
            while (deg[cur] == 2 && guard++ < 10000) {
                int nextEdge = -1;
                for (int ej : adj[cur]) {
                    const RoadEdge& f = g.edges[ej];
                    const int other = f.a == cur ? f.b : f.a;
                    if (other != prev) { nextEdge = ej; break; }
                }
                if (nextEdge < 0) break;
                const RoadEdge& f = g.edges[nextEdge];
                const int nxt = f.a == cur ? f.b : f.a;
                len += (g.nodes[nxt].pos - g.nodes[cur].pos).length();
                prev = cur; cur = nxt;
            }
            if (deg[cur] >= 3 && cur > v && len <= rad[v] + rad[cur] + 0.25)
                compound.push_back({v, cur, len});
        }
    }

    auto distSeg = [](const Vec2& p, const Vec2& a, const Vec2& b) {
        const Vec2 ab = b - a;
        const double l2 = ab.lengthSquared();
        double t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        return (a + ab * t - p).length();
    };

    int total = 0, inPad = 0, inCompound = 0, inBody = 0;
    double worst = 0; Vec2 worstAt(0, 0); const char* worstKind = "";
    int printed = 0;
    for (const StreetFurniture::Signal& s : sf->signalPoles) {
        ++total;
        const Vec2 p(s.base.x, s.base.z);
        // (1) inside a junction's own pad disc
        double padDepth = 0;   // how far INSIDE (positive = in the asphalt)
        for (int v = 0; v < N; ++v) {
            if (deg[v] < 3) continue;
            padDepth = std::max(padDepth, rad[v] - (g.nodes[v].pos - p).length());
        }
        // (2) inside the asphalt a compound pad owns between its members
        double compDepth = 0;
        for (const Pair& c : compound)
            compDepth = std::max(compDepth,
                                 std::min(rad[c.a], rad[c.b]) -
                                     distSeg(p, g.nodes[c.a].pos, g.nodes[c.b].pos));
        // (3) inside a body ribbon (carriageway only — the sidewalk band is
        //     exactly where a pole belongs)
        double bodyDepth = 0;
        for (const RoadEdge& e : g.edges) {
            if (e.a < 0 || e.b < 0 || e.a >= N || e.b >= N) continue;
            bodyDepth = std::max(bodyDepth,
                                 e.width * 0.5 - distSeg(p, g.nodes[e.a].pos, g.nodes[e.b].pos));
        }
        const char* kind = nullptr;
        double depth = 0;
        if (padDepth > 0.05) { kind = "pad"; depth = padDepth; ++inPad; }
        else if (compDepth > 0.05) { kind = "compound"; depth = compDepth; ++inCompound; }
        else if (bodyDepth > 0.05) { kind = "carriageway"; depth = bodyDepth; ++inBody; }
        if (kind) {
            if (depth > worst) { worst = depth; worstAt = p; worstKind = kind; }
            if (printed < 8) {
                ++printed;
                std::printf("    pole at (%.1f, %.1f): %.2f m inside %s\n",
                            p.x, p.y, depth, kind);
            }
        }
    }
    std::printf("    [signals] %d poles: %d in a pad disc, %d in compound asphalt, "
                "%d in a carriageway; %zu compound pairs; worst %.2f m (%s) at (%.1f, %.1f)\n",
                total, inPad, inCompound, inBody, compound.size(), worst, worstKind,
                worstAt.x, worstAt.y);
    CHECK(total > 50);                                 // the fixture must bite
    CHECK(inPad + inCompound + inBody == 0);           // every pole on a kerb

    // LAMPS, same question (device, metro map: "some of the street lights are
    // actually sitting right in the middle of the road"): every lamp foot
    // against every carriageway ribbon. Measured at adoption: 75 of 1739
    // inside, worst 5.84 m inside a 12 m road — kerb lamps of one street
    // planted where the neighbouring street's ribbon reaches past the
    // node-radius junction clear.
    int lampTotal = 0, lampInBody = 0;
    double lampWorst = 0; Vec2 lampWorstAt(0, 0);
    for (const Vec3& h : sf->lampHeads) {
        ++lampTotal;
        const Vec2 q(h.x, h.z);
        double depth = 0;
        for (const RoadEdge& e : g.edges) {
            if (e.a < 0 || e.b < 0 || e.a >= N || e.b >= N) continue;
            depth = std::max(depth,
                             e.width * 0.5 - distSeg(q, g.nodes[e.a].pos, g.nodes[e.b].pos));
        }
        if (depth > 0.05) {
            ++lampInBody;
            if (depth > lampWorst) { lampWorst = depth; lampWorstAt = q; }
        }
    }
    std::printf("    [lamps] %d lamps: %d inside a carriageway; worst %.2f m at (%.1f, %.1f)\n",
                lampTotal, lampInBody, lampWorst, lampWorstAt.x, lampWorstAt.y);
    CHECK(lampTotal > 500);
    CHECK(lampInBody == 0);
}

// THE TRAFFIC CENSUS (device: "the simulated cars ... sometimes they are below
// the road ... the cars should appear above the road ... come up with some
// tests to ensure the vehicles are on the road properly"). Loads the shipped
// metro through the real loader, stands up the citysim bridge exactly as the
// game does (default params, CitySimConfig from the level), drives the sim for
// a while, and compares every drawn DRIVEN car's wheel-bottom against the
// COLLIDER surface under it — the road mesh the player's own car drives on.
// Measured at adoption on the deck fixture: cars 0.62 m under the deck on
// average (the bridge placed them on the terrain, which the mesher carves
// 0.22 m under the deck, minus a vestigial 0.08 lift, plus the carve's own
// hillside error); after RoadDeck: within 3 cm.
namespace {
// Collider triangles bucketed by XZ cell so a probe touches a few dozen
// triangles, not the city's millions.
struct ColliderGrid {
    struct Tri { Vec3 a, b, c; };
    std::vector<Tri> tris;
    std::unordered_map<long long, std::vector<int>> cells;
    static constexpr double kCell = 12.0;
    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^ (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void add(const MeshCollider& mc) {
        for (std::size_t i = 0; i + 2 < mc.indices.size(); i += 3) {
            Tri t{mc.vertices[mc.indices[i]], mc.vertices[mc.indices[i + 1]],
                  mc.vertices[mc.indices[i + 2]]};
            const int id = static_cast<int>(tris.size());
            tris.push_back(t);
            const double x0 = std::min({t.a.x, t.b.x, t.c.x}), x1 = std::max({t.a.x, t.b.x, t.c.x});
            const double z0 = std::min({t.a.z, t.b.z, t.c.z}), z1 = std::max({t.a.z, t.b.z, t.c.z});
            for (int cz = static_cast<int>(std::floor(z0 / kCell)); cz <= static_cast<int>(std::floor(z1 / kCell)); ++cz)
                for (int cx = static_cast<int>(std::floor(x0 / kCell)); cx <= static_cast<int>(std::floor(x1 / kCell)); ++cx)
                    cells[key(cx, cz)].push_back(id);
        }
    }
    // Every collider surface under (x, z), for the offender listing.
    std::vector<double> surfacesAt(double x, double z) const {
        std::vector<double> out;
        auto it = cells.find(key(static_cast<int>(std::floor(x / kCell)),
                                 static_cast<int>(std::floor(z / kCell))));
        if (it == cells.end()) return out;
        for (int id : it->second) {
            const Tri& t = tris[id];
            if (!pointInTriangleXZ(x, z, t.a, t.b, t.c)) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            out.push_back(w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y);
        }
        std::sort(out.begin(), out.end());
        return out;
    }
    // Highest collider surface under (x, z) that is not above `ceiling`.
    bool surfaceAt(double x, double z, double ceiling, double& top) const {
        auto it = cells.find(key(static_cast<int>(std::floor(x / kCell)),
                                 static_cast<int>(std::floor(z / kCell))));
        if (it == cells.end()) return false;
        bool found = false;
        for (int id : it->second) {
            const Tri& t = tris[id];
            if (!pointInTriangleXZ(x, z, t.a, t.b, t.c)) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            const double y = w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y;
            if (y > ceiling) continue;
            if (!found || y > top) { top = y; found = true; }
        }
        return found;
    }
};
}  // namespace

TEST_CASE(metro_traffic_drives_on_the_road_deck) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;

    ColliderGrid grid;
    world.each<MeshCollider>([&](Entity, MeshCollider& mc) { grid.add(mc); });
    CHECK(!grid.tris.empty());

    citysim::CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));
    const std::vector<Vec3> he = city.carGroupHalfExtents();

    int checked = 0, seen = 0, noSurface = 0;
    double sum = 0, lo = 1e9, hi = -1e9;
    Vec2 worstAt(0, 0);
    double worst = 0;
    // Offender context: what the bridge could have used at that spot.
    std::vector<const engine::RoadDeckField*> decks;
    world.each<engine::RoadDeck>([&](Entity, engine::RoadDeck& d) { decks.push_back(&d.field); });
    std::function<double(double, double)> terrain;
    world.each<engine::TerrainLodConfig>([&](Entity, engine::TerrainLodConfig& c) {
        auto params = std::make_shared<engine::TerrainParams>(c.params);
        auto noise = std::make_shared<engine::Noise>(c.seed);
        terrain = [params, noise](double x, double z) {
            return engine::terrainHeight(*params, *noise, x, z);
        };
    });
    struct Offender { double d, x, z, y, surface, deck, terr; int link; double linkHalf; int layer; double speed; int padHits; };
    std::vector<Offender> offenders;
    std::size_t padTris = 0;
    for (const engine::RoadDeckField* f : decks) padTris += f->pads.size();
    std::printf("    [traffic] decks=%zu pad triangles=%zu\n", decks.size(), padTris);
    auto sample = [&]() {
        const auto& ids = city.carAgentIds();
        for (std::size_t v = 0; v < city.carGroups().size(); ++v) {
            InstanceGroup* g = world.get<InstanceGroup>(city.carGroups()[v]);
            if (!g || v >= ids.size()) continue;
            for (std::size_t i = 0; i < g->transforms.size() && i < ids[v].size(); ++i) {
                if (ids[v][i] < 0) continue;                 // parked scenery
                // MOVING traffic only: a driver resting at a destination may
                // have pulled off the carriageway (a dead end, a place's
                // entrance) — that is a different question from "cars drive
                // on the road".
                const auto& agents = city.sim().agents();
                const int aid = ids[v][i];
                if (aid >= static_cast<int>(agents.size()) || agents[aid].speed < 1.0) continue;
                ++seen;
                const Mat4& m = g->transforms[i];
                const double x = m.m[0][3], y = m.m[1][3], z = m.m[2][3];
                const double bottom = y - (v < he.size() ? he[v].y : 0.65);
                double top;
                // The surface the car is ON: the highest collider under its
                // origin (a bridge deck over a street resolves to the deck).
                if (!grid.surfaceAt(x, z, y, top)) { ++noSurface; continue; }
                const double d = bottom - top;
                ++checked;
                sum += d;
                lo = std::min(lo, d);
                hi = std::max(hi, d);
                if (std::fabs(d) > worst) { worst = std::fabs(d); worstAt = Vec2(x, z); }
                if (std::fabs(d) > 0.25) {
                    double deckY = std::nan("");
                    for (const engine::RoadDeckField* f : decks) {
                        double yy;
                        if (f->heightAt(x, z, 4.5, &yy)) { deckY = yy; break; }
                    }
                    const int link = city.nav().nearestLink(Vec2(x, z));
                    int padHits = 0;
                    for (const engine::RoadDeckField* f : decks)
                        for (const auto& t : f->pads)
                            if (x >= std::min({t.a.x, t.b.x, t.c.x}) - 0.01 && x <= std::max({t.a.x, t.b.x, t.c.x}) + 0.01 &&
                                z >= std::min({t.a.z, t.b.z, t.c.z}) - 0.01 && z <= std::max({t.a.z, t.b.z, t.c.z}) + 0.01)
                                ++padHits;
                    offenders.push_back({d, x, z, y, top, deckY,
                                         terrain ? terrain(x, z) : std::nan(""), link,
                                         link >= 0 ? city.nav().links[link].width * 0.5 : 0.0,
                                         link >= 0 ? city.nav().links[link].layer : -1,
                                         agents[aid].speed, padHits});
                }
            }
        }
    };
    for (int i = 0; i < 600; ++i) {                       // 60 s of city time
        city.step(world, 0.1);
        if (i % 20 == 0) sample();
    }
    std::printf("    [traffic] %d samples (%d seen, %d off any collider): wheel-bottom minus "
                "surface mean=%.3fm min=%.3fm max=%.3fm worst=%.3fm at (%.1f, %.1f)\n",
                checked, seen, noSurface, checked ? sum / checked : 0.0, lo, hi, worst,
                worstAt.x, worstAt.y);
    std::sort(offenders.begin(), offenders.end(),
              [](const Offender& a, const Offender& b) { return std::fabs(a.d) > std::fabs(b.d); });
    std::printf("    [traffic] %zu samples off by > 0.25 m; worst distinct spots:\n", offenders.size());
    std::vector<std::pair<int, int>> shown;
    int printed = 0;
    for (const Offender& o : offenders) {
        const std::pair<int, int> cellKey{static_cast<int>(std::floor(o.x / 3.0)),
                                          static_cast<int>(std::floor(o.z / 3.0))};
        if (std::find(shown.begin(), shown.end(), cellKey) != shown.end()) continue;
        shown.push_back(cellKey);
        std::printf("      d=%+.2f at (%.1f, %.1f) y=%.2f surface=%.2f deck=%.2f terrain+0.22=%.2f "
                    "link=%d half=%.1f layer=%d speed=%.1f padTrisOver=%d surfaces:",
                    o.d, o.x, o.z, o.y, o.surface, o.deck, o.terr + 0.22, o.link, o.linkHalf,
                    o.layer, o.speed, o.padHits);
        for (double h : grid.surfacesAt(o.x, o.z)) std::printf(" %.2f", h);
        std::printf("\n");
        if (++printed >= 10) break;
    }
    // Measured at adoption (RoadDeck + pads, moving cars only): 2562 samples,
    // mean 0.000 m, max +0.012 m, min -0.269 m — ONE sample, at
    // (-899.9, 106.6) on the western foothill, where a second collider
    // surface stands 0.26 m above the deck under a driving car (three stacked
    // surfaces there: 44.43 / 44.83 deck / 45.09). That is a road-side
    // geometry question, not a placement one; the bound holds it at its
    // current size and the mean/max bounds keep placement honest.
    CHECK(checked > 200);                                  // the fixture must bite
    CHECK(lo > -0.30);                                     // never IN the road
    CHECK(hi < 0.10);                                      // never hovering
    CHECK(std::fabs(checked ? sum / checked : 0.0) < 0.03);   // no systematic term
}


// THE BUSES SERVE THE CITY (Glenn, 2026-09-18: "The bus routes do seem sparse.
// I couldn't find one and I walked around for a while"). Built exactly as the
// game builds it -- the loader's terrain-gated street network, the level's own
// busRoutes / busStops / busMaxWalk, the sim's own seed -- because a headless
// metro grown without terrain is a different, rounder city, and measuring that
// one once already produced a wrong answer.
TEST_CASE(metro_bus_network_serves_the_city) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;
    citysim::CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));

    const citysim::BusNetwork& net = city.sim().buses();
    CHECK(net.routeCount() >= 3);
    int stops = 0;
    for (int r = 0; r < net.routeCount(); ++r) {
        const citysim::BusRoute& route = net.route(r);
        Real loop = 0;
        for (std::size_t k = 1; k < route.path.size(); ++k) {
            const Real dx = route.path[k].x - route.path[k - 1].x;
            const Real dy = route.path[k].y - route.path[k - 1].y;
            loop += std::sqrt(dx * dx + dy * dy);
        }
        stops += static_cast<int>(route.stops.size());
        std::printf("    [buses] route %d: %zu stops over a %.0f m loop (%.0f m apart)\n",
                    r, route.stops.size(), loop, loop / std::max<std::size_t>(1, route.stops.size()));
    }
    const Real walk = 220.0;   // metro_v2_test.json busMaxWalk
    const Real served = net.coverage(city.nav(), walk);
    const Real block = net.coverage(city.nav(), 120.0);
    const Real share = net.streetShare(city.nav());
    std::printf("    [buses] %d stops; %.0f%% of walkable street within %.0f m of one, "
                "%.0f%% within 120 m; %.0f%% of streets have a bus on them\n",
                stops, 100.0 * served, walk, 100.0 * block, 100.0 * share);
    // A walker outside the served area is never offered a bus at all.
    CHECK(served >= 0.90);

    // WHAT THE RIDER'S RULE ACCEPTS: walking trips across the city,
    // scored by door-to-door TIME -- walk to the stop, wait half a headway,
    // ride forward round the loop, walk on -- against walking the whole way
    // (straight line x1.25 for the street network).
    {
        const auto& nav = city.nav();
        uint32_t h = 12345u;
        auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return h; };
        int trips = 0, accepted = 0, slowerThanWalking = 0, backwardsRide = 0, changes = 0;
        double sumSaved = 0;
        for (int k = 0; k < 3000; ++k) {
            const Vec2 o = nav.nodes[rnd() % nav.nodes.size()];
            const Vec2 d = nav.nodes[rnd() % nav.nodes.size()];
            const Real direct = std::sqrt((d.x - o.x) * (d.x - o.x) + (d.y - o.y) * (d.y - o.y));
            if (direct < 300 || direct > 2500) continue;
            ++trips;
            const citysim::BusTrip t = net.planTrip(o, d, walk);
            if (!t.valid()) continue;
            ++accepted;
            const auto& r = net.route(t.route);
            const Vec2 a = r.stops[static_cast<std::size_t>(t.fromStop)].pos;
            const Vec2 b = r.stops[static_cast<std::size_t>(t.toStop)].pos;
            const Real wa = std::sqrt((a.x - o.x) * (a.x - o.x) + (a.y - o.y) * (a.y - o.y));
            const Real wb = std::sqrt((d.x - b.x) * (d.x - b.x) + (d.y - b.y) * (d.y - b.y));
            const Real walkT = direct * 1.25 / 1.4;
            Real busT = wa * 1.25 / 1.4 + net.waitSeconds(t.route) +
                        net.rideSeconds(t.route, t.fromStop, t.toStop);
            // A TRANSFER is planned as its first leg; stepping off at the change
            // stop the rider plans again, exactly as here.
            const citysim::BusTrip t2 = net.planTrip(b, d, walk);
            if (t2.valid() && t2.route != t.route) {
                ++changes;
                const auto& r2 = net.route(t2.route);
                const Vec2 a2 = r2.stops[static_cast<std::size_t>(t2.fromStop)].pos;
                const Vec2 b2 = r2.stops[static_cast<std::size_t>(t2.toStop)].pos;
                busT += std::sqrt((a2.x - b.x) * (a2.x - b.x) + (a2.y - b.y) * (a2.y - b.y)) * 1.25 / 1.4 +
                        60.0 + net.waitSeconds(t2.route) +
                        net.rideSeconds(t2.route, t2.fromStop, t2.toStop) +
                        std::sqrt((d.x - b2.x) * (d.x - b2.x) + (d.y - b2.y) * (d.y - b2.y)) * 1.25 / 1.4;
            } else {
                busT += wb * 1.25 / 1.4;
            }
            if (busT > walkT) ++slowerThanWalking;
            if (net.rideMetres(t.route, t.fromStop, t.toStop) > 0.5 * r.loopLength) ++backwardsRide;
            sumSaved += walkT - busT;
        }
        std::printf("    [rider] %d trips 300-2500 m: bus taken for %d (%d with a change); "
                    "%d of those SLOWER than walking, %d riding over half a loop; "
                    "mean saving %.0f s\n",
                    trips, accepted, changes, slowerThanWalking, backwardsRide,
                    accepted ? sumSaved / accepted : 0.0);
        // The old walk-only rule, measured: 2378 of 2552 accepted, 1382 of them
        // SLOWER than walking, a mean 132 s LOST. Chosen by time now.
        CHECK(accepted > 500);
        CHECK(changes > 0);                     // transfers are used
        CHECK(slowerThanWalking == 0);
        CHECK(accepted && sumSaved / accepted > 120.0);
    }

    // And a stop should be about a block away, on a street you might walk
    // down: the old network had 57% and 28% here, which is what "I walked
    // around for a while and couldn't find one" measured as.
    CHECK(block >= 0.80);
    CHECK(share >= 0.50);

    // EVERY BUS STARTS AT A STOP ON ITS OWN ROUTE, through the whole build --
    // setBuses seated them, and assignPlaces (which runs after it) then sent
    // every one home, so a route's first bus was a kilometre off its loop.
    int buses = 0, offRoute = 0;
    Real worst = 0;
    for (int i = 0; i < static_cast<int>(city.sim().agents().size()); ++i) {
        const int r = city.sim().busRouteOf(i);
        if (r < 0) continue;
        ++buses;
        const citysim::BusRoute& route = net.route(r);
        const int n = static_cast<int>(route.stops.size());
        const int at = (city.sim().busNextStopOf(i) - 1 + n) % n;
        const Vec2 d = city.sim().agents()[static_cast<std::size_t>(i)].pos -
                       route.stops[static_cast<std::size_t>(at)].pos;
        const Real dd = std::sqrt(d.x * d.x + d.y * d.y);
        worst = std::max(worst, dd);
        if (dd > 30.0) ++offRoute;
    }
    std::printf("    [buses] %d buses; %d not at their starting stop (worst %.0f m)\n",
                buses, offRoute, worst);
    CHECK(buses == 24);
    CHECK(offRoute == 0);

    // And in the running city, walkers choose it: three minutes, boardings.
    {
        const long before = city.sim().busBoardAttempts();
        const auto ps0 = city.sim().buses().planStats();
        for (int i = 0; i < 1800; ++i) city.step(world, 0.1);
        long aboard = 0;
        for (std::size_t ai = 0; ai < city.sim().agents().size(); ++ai)
            if (city.sim().isBus(static_cast<int>(ai)))
                aboard += city.sim().rides().load(static_cast<int>(ai));
        const auto& ps = city.sim().buses().planStats();
        std::printf("    [rider] 3 min of metro: %ld boardings, %ld aboard now, %ld waiting; "
                    "plans asked %ld, bus %ld (%ld with a change), not worth it %ld\n",
                    city.sim().busBoardAttempts() - before, aboard,
                    static_cast<long>(city.sim().buses().waitingCount()), ps.asked - ps0.asked,
                    ps.ok - ps0.ok,
                    ps.transfers - ps0.transfers, ps.noSaving - ps0.noSaving);
        CHECK(city.sim().busBoardAttempts() - before > 0);
    }
}

// PARKING (Glenn, 2026-09-18: "They park at the corner in a pile, but they
// should be using the parallel parking spaces or maybe the parking garages, but
// they need to park in spaces and not in a heap on the corner"). A census of
// every DRAWN parked car in the loader-built metro: in a marked bay, or on the
// verge; how many sit on top of another parked car; how many inside a junction.
namespace {
struct ParkCensus {
    int drivers = 0, bays = 0, parked = 0, inBay = 0, verge = 0, stacked = 0,
        atJunction = 0, offStreet = 0;
};
ParkCensus parkCensus(const citysim::CityRenderSystem& city) {
    ParkCensus c;
    const auto& sim = city.sim();
    const auto& nav = city.nav();
    c.bays = static_cast<int>(sim.parkingBays().size());
    std::vector<Vec2> parkedAt;
    for (std::size_t ai = 0; ai < sim.agents().size(); ++ai) {
        const auto& a = sim.agents()[ai];
        if (a.archetype != citysim::Agent::Mode::Driver) continue;
        if (sim.isBus(static_cast<int>(ai))) continue;   // at a stop, not parked
        ++c.drivers;
        if (a.moving) continue;                      // on the road
        if (a.car < 0 || a.car >= static_cast<int>(sim.vehicles().size())) continue;
        const auto& v = sim.vehicles()[static_cast<std::size_t>(a.car)];
        if (v.offStreet) { ++c.offStreet; continue; }
        ++c.parked;
        if (a.parkedBay >= 0) ++c.inBay; else ++c.verge;
        // A driver resting IN its car is drawn at its own pose; one that got
        // out left the car where it parked.
        parkedAt.push_back(a.vehicle >= 0 ? a.pos : v.pos);
    }
    for (std::size_t i = 0; i < parkedAt.size(); ++i) {
        for (std::size_t j = 0; j < parkedAt.size(); ++j) {
            if (i == j) continue;
            const Vec2 d = parkedAt[i] - parkedAt[j];
            if (d.x * d.x + d.y * d.y < 3.0 * 3.0) { ++c.stacked; break; }
        }
        for (int n = 0; n < nav.nodeCount(); ++n) {
            if (!nav.isJunction(n)) continue;
            const Vec2 d = parkedAt[i] - nav.nodes[static_cast<std::size_t>(n)];
            if (d.x * d.x + d.y * d.y < 10.0 * 10.0) { ++c.atJunction; break; }
        }
    }
    return c;
}
void printPark(const char* when, const ParkCensus& c) {
    std::printf("    [park] %s: %d drivers, %d bays | %d parked on street: %d in bays, "
                "%d on the verge | %d stacked within 3 m of another | %d inside a "
                "junction | %d off-street\n",
                when, c.drivers, c.bays, c.parked, c.inBay, c.verge, c.stacked,
                c.atJunction, c.offStreet);
}
}  // namespace

TEST_CASE(metro_cars_park_in_spaces_not_heaps) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;
    citysim::CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));
    const ParkCensus atLoad = parkCensus(city);
    printPark("at load", atLoad);
    for (int i = 0; i < 1800; ++i) city.step(world, 0.1);   // 3 minutes
    // AND TRAFFIC STILL FLOWS. Two ways parking in bays first slowed metro:
    // cars resting in a bay sat just inside the car-ahead corridor of the kerb
    // lane (passing cars braked behind them), and routes ended in U-turns to
    // reach a bay's street, or left one by turning straight back. Measured:
    // mean speed of moving cars after 3 min 5.6 m/s before the allocator, 4.1
    // with those two bugs, 6.1 fixed; U-turn routes 0 / 67 / ~11.
    int moving = 0, uturns = 0;
    double speed = 0;
    for (std::size_t ai = 0; ai < city.sim().agents().size(); ++ai) {
        const auto& a = city.sim().agents()[ai];
        if (a.archetype != citysim::Agent::Mode::Driver || !a.moving ||
            city.sim().isBus(static_cast<int>(ai)))
            continue;
        ++moving;
        speed += a.speed;
        const auto& rl = a.route.links;
        for (std::size_t q = 1; q < rl.size(); ++q) {
            const auto& l0 = city.nav().links[static_cast<std::size_t>(rl[q - 1])];
            const auto& l1 = city.nav().links[static_cast<std::size_t>(rl[q])];
            if (l0.from == l1.to && l0.to == l1.from) { ++uturns; break; }
        }
    }
    const double meanSpeed = moving ? speed / moving : 0.0;
    std::printf("    [park] traffic after 3 min: %d moving, mean %.2f m/s, %d routes "
                "with a U-turn\n", moving, meanSpeed, uturns);
    CHECK(moving > 100);
    CHECK(meanSpeed > 5.0);
    CHECK(uturns < 25);
    const ParkCensus later = parkCensus(city);
    printPark("after 3 min", later);
    // Measured before the allocator: 2 of 1257 parked cars in a bay, 857
    // within 3 m of another, 23 inside a junction -- with 4646 bays free.
    for (const ParkCensus* c : {&atLoad, &later}) {
        CHECK(c->parked > 500);                       // the fixture must bite
        CHECK(c->inBay >= c->parked * 95 / 100);      // in marked spaces
        CHECK(c->stacked == 0);                       // no heaps
        CHECK(c->atJunction == 0);                    // nothing in the box
    }
}

// CARS WAIT BEFORE THE CROSSWALK AND DO NOT DRIVE THROUGH EACH OTHER (Glenn,
// 2026-09-19: "they should not stop in the middle of the intersection. That
// creates an instant traffic jam. They should stop before the cross walk and
// then wait for the light ... The cars should not go through each other").
//
// Metro's streets are chains of ~4 m links, and every junction rule looked at
// the current link only: the stop line was clamped to 40% of a 4 m link (inside
// the box), the car one link back did not see the junction at all, car-
// following saw two links (~8 m) ahead, and far-tier cars were promoted into
// the live sim at cruise on top of stopped queues. Measured by this test on
// the code before the fix: 3732 car-seconds stuck in a box and 5535
// overlapping pairs; after it, ~100 and ~80-270 (the busy ring where far cars
// join the live sim still makes the odd pile). The gates sit well between.
//
// Sampled once a second after a 20 s warm-up, over the live (non-far) cars:
//  - STUCK IN A BOX: a car stopped > 5 s with its centre inside a junction's
//    drawn mouth (the widest arm's half-width + the 3.5 m sidewalk band --
//    where the road mesher trims the streets and the zebras start).
//  - OVERLAPS: two car bodies interpenetrating by more than a 10 cm skin.
TEST_CASE(metro_junctions_stay_clear) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;
    citysim::CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));
    const auto& nav = city.nav();
    std::vector<int> junctions;
    std::vector<Real> mouth;
    for (int n = 0; n < nav.nodeCount(); ++n) {
        if (!nav.isJunction(n)) continue;
        Real r = 0;
        for (int li : nav.outLinks[static_cast<std::size_t>(n)])
            r = std::max(r, nav.links[static_cast<std::size_t>(li)].width * 0.5);
        junctions.push_back(n);
        mouth.push_back(r + 3.5);
    }
    // Rectangle overlap by separating axes, bodies shrunk by a 10 cm skin.
    auto overlap = [](Vec2 pa, Vec2 ha, Real la, Vec2 pb, Vec2 hb, Real lb) {
        const Vec2 axes[4] = {ha, Vec2(-ha.y, ha.x), hb, Vec2(-hb.y, hb.x)};
        const Vec2 d = pb - pa;
        for (const Vec2& ax : axes) {
            auto reach = [&](Vec2 h, Real l) {
                return std::fabs(h.x * ax.x + h.y * ax.y) * (l * 0.5 - 0.1) +
                       std::fabs(-h.y * ax.x + h.x * ax.y) * 0.8;
            };
            if (std::fabs(d.x * ax.x + d.y * ax.y) > reach(ha, la) + reach(hb, lb)) return false;
        }
        return true;
    };
    auto bodyLength = [&](const citysim::Agent& a) {
        return a.vehicle >= 0 ? city.sim().vehicles()[static_cast<std::size_t>(a.vehicle)].length
                              : Real(4.2);
    };
    std::vector<Real> stoppedFor(city.sim().agents().size(), 0);
    long samples = 0, carsSeen = 0, inBox = 0, stuckBox = 0, overlaps = 0;
    for (int i = 0; i < 1800; ++i) {
        city.step(world, 0.1);
        const auto& ag = city.sim().agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            if (ag[k].mode == citysim::Agent::Mode::Driver && ag[k].moving && ag[k].speed < 0.3)
                stoppedFor[k] += 0.1;
            else
                stoppedFor[k] = 0;
        }
        if (i < 200 || i % 10) continue;
        ++samples;
        std::vector<std::size_t> cars;
        for (std::size_t k = 0; k < ag.size(); ++k)
            if (ag[k].mode == citysim::Agent::Mode::Driver && ag[k].moving && !ag[k].far())
                cars.push_back(k);
        carsSeen += static_cast<long>(cars.size());
        for (std::size_t k : cars) {
            if (ag[k].speed >= 0.3) continue;
            for (std::size_t j = 0; j < junctions.size(); ++j) {
                const Vec2 d = ag[k].pos - nav.nodes[static_cast<std::size_t>(junctions[j])];
                const Real r = mouth[j];
                if (d.x * d.x + d.y * d.y < r * r) {
                    ++inBox;
                    if (stoppedFor[k] > 5.0) ++stuckBox;
                    break;
                }
            }
        }
        for (std::size_t x = 0; x < cars.size(); ++x)
            for (std::size_t y = x + 1; y < cars.size(); ++y) {
                const auto& A = ag[cars[x]];
                const auto& B = ag[cars[y]];
                const Vec2 d = A.pos - B.pos;
                if (d.x * d.x + d.y * d.y > 16.0 * 16.0) continue;
                if (std::fabs(A.elevation - B.elevation) > 2.5) continue;
                if (overlap(A.pos, A.heading, bodyLength(A), B.pos, B.heading, bodyLength(B)))
                    ++overlaps;
            }
    }
    std::printf("    [junction] %ld samples, %.0f live cars each: %ld stopped in a box "
                "(%ld for > 5 s), %ld overlapping pairs\n",
                samples, samples ? static_cast<double>(carsSeen) / samples : 0.0, inBox,
                stuckBox, overlaps);
    CHECK(samples > 0 && carsSeen / samples > 40);   // the city is actually driving
    CHECK(stuckBox < 300);
    CHECK(overlaps < 800);
}

// PEOPLE ARE OUT DOING THINGS, AND INSIDE WHEN THEY ARE NOT (Glenn, 2026-09-19:
// "npcs stand around and don't walk and also overlap each other ... More
// movement in the city would be great", then "agents who are at work or home
// should ... go inside the building. We should have non workers and pedestrians
// who are out for a stroll or going to public spaces").
//
// Measured on metro before the change, within 200 m of the player at 7:45:
// 2.4 walkers moving, ~193 standing on the pavement by the node of their home
// or job (every pedestrian between trips was drawn where it rested), and 427
// overlapping pairs among them (one idle pose per node, 8 slots). After: the
// resting are indoors, about a third of walkers have the day off and go out
// (parks, cafes, stores, a walk round the block), commuters walk out to lunch,
// and ~21 are moving at any hour sampled (7:45 .. 18:30).
//
// Sampled after a minute's warm-up at the level's own hour, with the sky's
// real-time clock the game runs:
//  - MOVING: walkers under way within 200 m.
//  - STANDING OUTSIDE: drawn (pedVisible) but not moving and not waiting for a
//    bus -- the idle crowd.
//  - OVERLAPS: two drawn walkers closer than 0.45 m (bodies are 0.5 m wide).
TEST_CASE(metro_pedestrians_walk_and_keep_apart) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded =
        LevelLoader::load(simLevelPath(), world, *renderer,
                          view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;
    citysim::CityRenderSystem city;
    city.setWorldClock(7.75, 1.0 / 3600.0);   // the level's hour, the sky's pace
    CHECK(city.build(world, &assets, nullptr));
    for (int i = 0; i < 600; ++i) city.step(world, 0.1);
    long samples = 0, moving = 0, standing = 0, indoors = 0, overlaps = 0;
    for (int i = 0; i < 600; ++i) {
        city.step(world, 0.1);
        if (i % 20) continue;
        ++samples;
        const auto& ag = city.sim().agents();
        const Vec2 c = city.sim().tierCenter();
        std::vector<std::size_t> drawn;
        for (std::size_t k = 0; k < ag.size(); ++k) {
            const auto& a = ag[k];
            if (a.mode != citysim::Agent::Mode::Pedestrian || a.far()) continue;
            const Vec2 d = a.pos - c;
            if (d.x * d.x + d.y * d.y > 200.0 * 200.0) continue;
            const int ki = static_cast<int>(k);
            if (city.sim().riding(ki)) continue;
            if (!city.sim().pedVisible(ki)) { ++indoors; continue; }
            if (a.moving) ++moving;
            else if (!city.sim().awaitingRide(ki)) ++standing;
            drawn.push_back(k);
        }
        for (std::size_t x = 0; x < drawn.size(); ++x)
            for (std::size_t y = x + 1; y < drawn.size(); ++y) {
                const Vec2 d = ag[drawn[x]].pos - ag[drawn[y]].pos;
                if (d.x * d.x + d.y * d.y < 0.45 * 0.45) ++overlaps;
            }
    }
    const double n = samples ? static_cast<double>(samples) : 1.0;
    std::printf("    [peds] within 200 m per sample: moving %.1f, standing outside %.1f, "
                "indoors %.1f | overlapping pairs %ld\n",
                moving / n, standing / n, indoors / n, overlaps);
    CHECK(samples > 0);
    CHECK(indoors / n > 50);          // the neighbourhood is populated...
    CHECK(moving / n > 12);           // ...and people are out walking (was 2.4)
    CHECK(standing / n < 5);          // nobody loitering by their front door (was ~193)
    CHECK(overlaps == 0);             // and nobody standing inside anybody (was 427/sample)
}


// THE MAP IS THE CITY SVG (Glenn, 2026-09-19: "make a minimap using the svg map
// we have and use it to pan and zoom around ... bus stops and bus lines ... It
// can't be imgui. But should show the svg map"). The map tool rasterizes the
// level's own city-map SVG (what the generators built) plus the bus lines; this
// pins that the picture is the city, the lines are on it in their colours, and
// that a close view only pays for what it shows.
TEST_CASE(metro_map_shows_the_city_and_its_bus_lines) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded = LevelLoader::load(simLevelPath(), world, *renderer,
                                          view, assets, false);
    CHECK(loaded);
    if (!loaded) return;
    citysim::CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));
    const engine::CityMap* map = nullptr;
    world.each<engine::CityMap>([&](Entity, engine::CityMap& m) { if (!map) map = &m; });
    CHECK(map && map->data);
    if (!map || !map->data) return;
    const std::string svg = (std::filesystem::temp_directory_path() / "rt_metro_map_test.svg").string();
    CHECK(engine::writeCityMapSvg(svg, *map->data,
                                  engine::CityMapLayers::fromList(citysim::kCityMapLayers)));
    citysim::CityMapRaster r;
    CHECK(r.loadCity(svg));
    std::vector<citysim::TransitLine> lines;
    const auto& net = city.sim().buses();
    for (int ri = 0; ri < net.routeCount(); ++ri) {
        citysim::TransitLine l;
        for (const auto& pt : net.route(ri).path) l.path.push_back({pt.x, pt.y});
        const engine::Vec3 c = citysim::routeColour(ri);
        l.r = static_cast<float>(c.x); l.g = static_cast<float>(c.y); l.b = static_cast<float>(c.z);
        lines.push_back(l);
    }
    CHECK(r.loadTransit(citysim::transitSvg(lines, r.minX(), r.minZ(), r.width(), r.height())));

    // The whole city in 1000 px.
    citysim::CityMapRaster::View v;
    v.cx = r.minX() + r.width() * 0.5;
    v.cz = r.minZ() + r.height() * 0.5;
    v.metresPerPixel = std::max(r.width(), r.height()) / 1000.0;
    v.w = v.h = 1000;
    std::vector<uint8_t> px;
    const auto t0 = std::chrono::steady_clock::now();
    r.rasterize(v, px);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const int cityShapes = r.lastShapesDrawn();
    // Streets and buildings are grey-brown; the paper and the block ground are
    // near-white or green-grey. Count pixels that are clearly INK.
    long ink = 0;
    for (std::size_t k = 0; k < px.size(); k += 4) {
        const int lum = (px[k] * 3 + px[k + 1] * 6 + px[k + 2]) / 10;
        if (lum < 190) ++ink;
    }
    const double inkShare = static_cast<double>(ink) / (v.w * v.h);
    // Each route's line, in its colour, at points along its path.
    int hit = 0, tried = 0, worstRoute = 100;
    for (int ri = 0; ri < net.routeCount(); ++ri) {
        const auto& path = net.route(ri).path;
        const engine::Vec3 c = citysim::routeColour(ri);
        int rh = 0, rt = 0;
        for (std::size_t k = 0; k < path.size(); k += std::max<std::size_t>(1, path.size() / 40)) {
            const int x = static_cast<int>((path[k].x - (v.cx - v.w * 0.5 * v.metresPerPixel)) / v.metresPerPixel);
            const int y = static_cast<int>((path[k].y - (v.cz - v.h * 0.5 * v.metresPerPixel)) / v.metresPerPixel);
            ++rt;
            bool found = false;
            for (int dy = -2; dy <= 2 && !found; ++dy)
                for (int dx = -2; dx <= 2 && !found; ++dx) {
                    const int xx = x + dx, yy = y + dy;
                    if (xx < 0 || yy < 0 || xx >= v.w || yy >= v.h) continue;
                    const std::size_t o = (static_cast<std::size_t>(yy) * v.w + xx) * 4;
                    const double dr = px[o] - c.x * 255, dg = px[o + 1] - c.y * 255, db = px[o + 2] - c.z * 255;
                    if (dr * dr + dg * dg + db * db < 60.0 * 60.0) found = true;
                }
            if (found) ++rh;
        }
        hit += rh;
        tried += rt;
        worstRoute = std::min(worstRoute, rt ? rh * 100 / rt : 0);
    }
    // A street-level view (0.3 m/px) round the player.
    citysim::CityMapRaster::View s;
    s.cx = city.sim().tierCenter().x;
    s.cz = city.sim().tierCenter().y;
    s.metresPerPixel = 0.3;
    s.w = 1400;
    s.h = 900;
    r.rasterize(s, px);
    const int streetShapes = r.lastShapesDrawn();
    std::printf("    [map] city view %.0f ms: %d shapes, %.0f%% ink; bus lines found at %d/%d "
                "path points (worst route %d%%); street view drew %d shapes\n",
                ms, cityShapes, inkShare * 100.0, hit, tried, worstRoute, streetShapes);
    CHECK(inkShare > 0.08);                    // it is a city, not a blank sheet
    CHECK(worstRoute >= 30);                   // every route's line is on it (routes sharing a street paint over each other)
    CHECK(streetShapes < cityShapes / 5);      // a close view only rasterizes what it shows
}

// STREET NAME SIGNS ON METRO (Glenn, 2026-09-19: "store a list of all the
// street sign textures and map them to the right signs and make sure they fit
// and are legible"). The loader names the streets and stands a post at a
// corner of every intersection; this rebuilds the same plan and atlas from
// the level's graph and holds the whole city to it: every blade's lettering
// fits its blade and its capitals are no smaller than 40% of the blade, every
// post names exactly the streets that meet at its junction, and no post
// stands in a carriageway.
TEST_CASE(metro_street_signs_fit_and_name_their_corners) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    const bool loaded = LevelLoader::load(simLevelPath(), world, *renderer,
                                          view, assets, false);
    CHECK(loaded);
    if (!loaded) return;
    const engine::RoadGraph* graph = nullptr;
    world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph& g) { if (!graph) graph = &g.graph; });
    const engine::StreetDirectory* dir = nullptr;
    world.each<engine::StreetDirectory>([&](Entity, engine::StreetDirectory& d) { if (!dir) dir = &d; });
    CHECK(graph && dir && dir->naming);
    if (!graph || !dir || !dir->naming) return;
    const engine::StreetNaming& names = *dir->naming;
    const engine::Font* font = engine::signFont();
    CHECK(font != nullptr);
    if (!font) return;
    engine::StreetSignParams sp;
    sp.sidewalkWidth = 3.5;
    const auto posts = engine::planStreetSigns(*graph, names, nullptr, sp);
    const engine::SignAtlas atlas = engine::buildSignAtlas(*font, names, posts, sp);
    // Intersections where two or more named streets meet.
    std::vector<std::vector<int>> at(graph->nodes.size());
    for (int e = 0; e < static_cast<int>(graph->edges.size()); ++e) {
        at[static_cast<std::size_t>(graph->edges[static_cast<std::size_t>(e)].a)].push_back(e);
        at[static_cast<std::size_t>(graph->edges[static_cast<std::size_t>(e)].b)].push_back(e);
    }
    int corners = 0;
    for (std::size_t n = 0; n < graph->nodes.size(); ++n) {
        if (at[n].size() < 3) continue;
        const auto k = graph->nodes[n].kind;
        if (k != engine::JunctionKind::Intersection && k != engine::JunctionKind::Auto) continue;
        std::set<int> s;
        for (int e : at[n]) if (names.streetOf(e) >= 0) s.insert(names.streetOf(e));
        if (s.size() >= 2) ++corners;
    }
    int fits = 0, legible = 0, abbreviated = 0, condensed = 0, wrongNames = 0, inRoad = 0;
    float minCap = 1e9f;
    const int pad = static_cast<int>(std::lround(sp.bladePx * 0.28));
    for (const auto& [s, b] : atlas.blades) {
        if (b.textPx + 2 * pad <= b.wPx + 1) ++fits;
        if (b.capPx >= 0.40f * sp.bladePx) ++legible;
        abbreviated += b.abbreviated ? 1 : 0;
        condensed += b.xScale < 0.999f ? 1 : 0;
        minCap = std::min(minCap, b.capPx);
    }
    for (const auto& post : posts) {
        std::set<int> want, have;
        for (int e : at[static_cast<std::size_t>(post.node)])
            if (names.streetOf(e) >= 0) want.insert(names.streetOf(e));
        for (const auto& b : post.blades) have.insert(b.street);
        // Up to three blades: the widest three of the streets here.
        if (!(have.size() == std::min<std::size_t>(3, want.size()) &&
              std::includes(want.begin(), want.end(), have.begin(), have.end())))
            ++wrongNames;
        // In ANY carriageway (not just this junction's): distance to every
        // edge's centreline segment against its half-width.
        const Vec2 q(post.base.x, post.base.z);
        for (const auto& ed : graph->edges) {
            const Vec2 a = graph->nodes[static_cast<std::size_t>(ed.a)].pos;
            const Vec2 b = graph->nodes[static_cast<std::size_t>(ed.b)].pos;
            const Vec2 ab = b - a;
            const Real l2 = ab.x * ab.x + ab.y * ab.y;
            const Real t = l2 > 1e-12 ? std::clamp(((q.x - a.x) * ab.x + (q.y - a.y) * ab.y) / l2, 0.0, 1.0) : 0.0;
            const Vec2 c(a.x + ab.x * t, a.y + ab.y * t);
            if ((q - c).length() < ed.width * 0.5 + 0.3) { ++inRoad; break; }
        }
    }
    std::printf("    [signs] %zu named streets; %d intersections, %zu posts; %zu blades on %zu page(s): "
                "%d fit, %d legible (smallest capitals %.1f px = %.2f m), %d abbreviated, %d condensed; "
                "%d posts with wrong names, %d in a carriageway\n",
                names.streets.size(), corners, posts.size(), atlas.blades.size(), atlas.pages.size(),
                fits, legible, minCap, minCap / sp.bladePx * sp.bladeHeight, abbreviated, condensed,
                wrongNames, inRoad);
    if (const char* dump = std::getenv("RT_SIGN_ATLAS_DUMP")) {
        const auto& pg = atlas.pages.front();
        FILE* f = std::fopen(dump, "wb");
        if (f) {
            std::fprintf(f, "P6 %d %d 255\n", pg.w, pg.h);
            for (std::size_t k = 0; k < pg.rgba.size(); k += 4) std::fwrite(&pg.rgba[k], 1, 3, f);
            std::fclose(f);
        }
    }
    CHECK(posts.size() >= static_cast<std::size_t>(corners * 9 / 10));   // nearly every corner signed
    CHECK(fits == static_cast<int>(atlas.blades.size()));
    CHECK(legible == static_cast<int>(atlas.blades.size()));
    CHECK(wrongNames == 0);
    CHECK(inRoad == 0);
    CHECK(dir->signPosts == static_cast<int>(posts.size()));   // the loader stood the same plan
}

// ============================================================================
// THE CITY COMPLETENESS GATE (Glenn, 2026-09-20: "These systems we've built seem
// incredibly brittle. If you can change one thing and the whole thing collapses
// like this and regresses I think that means systems need to be built more
// resiliently.")
//
// He was right, and the shape of it is measurable. `RoadBuilder` declares THREE
// products — build / ground / navGraph. The engine reads road facts from EIGHT
// channels across ~76 sites: RoadEntity (28), terrain.flatten (21), the spec's
// parking band (11), freewayROW (10), look.sidewalk (3), RoadDeck (2),
// LevelRoadGraph (1), RoadBandDebug. The lattice fills all eight as a SIDE
// EFFECT of being a RoadEntity that carries a graph. So a second builder can
// satisfy 100% of the declared interface and still produce a city where cars
// sink into the asphalt, nobody can park, signal poles stand in the road and
// trees grow on the carriageway — which is exactly what happened, four times,
// each discovered by a different downstream test weeks apart.
//
// With one implementation an implicit contract is invisible. It only becomes
// visible when there are two. So this gate enumerates the contract ONCE and
// holds every builder to it: load a shipped level built by each, and assert the
// city published what the engine is going to read. It is deliberately about
// PRESENCE and SELF-CONSISTENCY, not quality — the quality gates already exist
// and are per-subsystem; this one exists so a missing channel cannot reach them.
// ============================================================================

namespace {

struct CityFacts {
    std::string level;
    bool loaded = false;
    // channel 1: the graph everything routes on
    std::size_t navNodes = 0, navEdges = 0;
    // channel 2: the surface things stand on
    std::size_t deckFields = 0, deckSpines = 0;
    long navSamples = 0, navOnDeck = 0;         // does the deck cover the graph?
    // channel 3: the road's own look (the sidewalk band furniture measures off)
    double sidewalk = 0, curb = 0;
    // channel 4: the kerbside parking band the sim lays bays in
    std::size_t streetEdges = 0, edgesWithParking = 0;
    // channel 5: the keep-out the vegetation scatter (and anything else that
    // asks "did the city grade here") reads
    std::size_t flattenRegions = 0;
    long carriagewaySamples = 0, carriagewayKeptOut = 0;
    // channel 6: the freeway right-of-way the lot pass zones around
    std::size_t freewayEdges = 0;
    // channel 7: something to build on
    std::size_t lotBuildings = 0;
};

CityFacts gatherCityFacts(const std::string& levelName) {
    CityFacts f;
    f.level = levelName;
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    f.loaded = LevelLoader::load(levelsDir() + "/" + levelName, world, *renderer, view,
                                 assets, /*editorMode=*/false);
    if (!f.loaded) return f;

    engine::RoadGraph nav;
    world.each<engine::LevelRoadGraph>([&](Entity, engine::LevelRoadGraph& g) {
        if (!g.graph.edges.empty()) nav = g.graph;
    });
    f.navNodes = nav.nodes.size();
    f.navEdges = nav.edges.size();

    std::vector<const engine::RoadDeckField*> decks;
    world.each<engine::RoadDeck>([&](Entity, engine::RoadDeck& d) {
        decks.push_back(&d.field);
        f.deckSpines += d.field.spines.size();
    });
    f.deckFields = decks.size();

    world.each<engine::RoadEntity>([&](Entity, engine::RoadEntity& net) {
        f.sidewalk = std::max(f.sidewalk, static_cast<double>(net.look.sidewalk));
        f.curb = std::max(f.curb, static_cast<double>(net.look.curb));
    });

    // The terrain's final flatten set: what the city told the ground it had graded.
    std::vector<engine::TerrainFlatten> flatten;
    world.each<engine::TerrainLodConfig>([&](Entity, engine::TerrainLodConfig& c) {
        if (c.params.flatten.size() > flatten.size()) flatten = c.params.flatten;
    });
    f.flattenRegions = flatten.size();
    engine::FlattenGrid keepOut;
    if (!flatten.empty()) keepOut = engine::buildFlattenGrid(flatten);

    // Walk the graph: sample the middle of every street edge and ask the two
    // questions a consumer asks — is there a DECK under this point, and did the
    // city mark it as graded so nothing scatters onto it?
    for (const engine::RoadEdge& e : nav.edges) {
        if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(nav.nodes.size()) ||
            e.b >= static_cast<int>(nav.nodes.size()))
            continue;
        const engine::Vec2 a = nav.nodes[static_cast<std::size_t>(e.a)].pos;
        const engine::Vec2 b = nav.nodes[static_cast<std::size_t>(e.b)].pos;
        const bool street = e.klass != engine::RoadClass::Freeway &&
                            e.klass != engine::RoadClass::Ramp;
        if (street) {
            ++f.streetEdges;
            if (e.parkWidth > 0.0) ++f.edgesWithParking;
        } else {
            ++f.freewayEdges;
        }
        for (int k = 1; k <= 3; ++k) {
            const double t = k / 4.0;
            const engine::Vec2 q(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            ++f.navSamples;
            double y = 0;
            for (const engine::RoadDeckField* d : decks)
                if (d->heightAt(q.x, q.y, 1.0, &y)) { ++f.navOnDeck; break; }
            // The keep-out question, asked exactly the way the vegetation scatter asks
            // it: is this point excluded from scatter? Two sources answer it — the
            // graded `flatten` set a carving builder fills, and the DECK a replacing
            // builder publishes instead. Testing one implementation rather than the
            // consumer's question is how this went unnoticed in the first place.
            ++f.carriagewaySamples;
            bool kept = !flatten.empty() && engine::flattenCovers(keepOut, flatten, q.x, q.y, 0.0);
            if (!kept)
                for (const engine::RoadDeckField* d : decks) {
                    double dy = 0;
                    if (d->heightAt(q.x, q.y, 0.0, &dy)) { kept = true; break; }
                }
            if (kept) ++f.carriagewayKeptOut;
        }
    }

    world.each<CityBuildings>([&](Entity, CityBuildings& cb) { f.lotBuildings += cb.records.size(); });
    return f;
}

void printCityFacts(const CityFacts& f) {
    std::printf("    [city] %-22s nav %zu/%zu  deck %zu fields/%zu spines  "
                "on-deck %ld/%ld  sidewalk %.2f curb %.2f\n",
                f.level.c_str(), f.navNodes, f.navEdges, f.deckFields, f.deckSpines,
                f.navOnDeck, f.navSamples, f.sidewalk, f.curb);
    std::printf("    [city] %-22s parking %zu/%zu street edges  flatten %zu regions, "
                "carriageway kept out %ld/%ld  freeway %zu edges\n",
                f.level.c_str(), f.edgesWithParking, f.streetEdges, f.flattenRegions,
                f.carriagewayKeptOut, f.carriagewaySamples, f.freewayEdges);
}

}  // namespace

// Every shipped city, whichever builder paved it, publishes what the engine reads.
TEST_CASE(every_builder_publishes_a_complete_city) {
    // One level per builder. Adding a builder means adding a level here.
    const char* kCities[] = {"metro_v2_test.json", "metro_lanes.json"};
    for (const char* name : kCities) {
        const CityFacts f = gatherCityFacts(name);
        CHECK(f.loaded);
        if (!f.loaded) { std::printf("    [city] %s FAILED TO LOAD\n", name); continue; }
        printCityFacts(f);

        // 1. THE GRAPH. Nav, traffic, furniture, signs and the map all route on it.
        CHECK(f.navEdges > 100);
        CHECK(f.navNodes > 100);

        // 2. THE DECK. Everything the sim stands on a road reads this; without it
        //    traffic falls back to the terrain and sinks into the asphalt.
        CHECK(f.deckFields > 0);
        CHECK(f.deckSpines > 0);
        //    ...and it must COVER the graph it belongs to, not merely exist.
        CHECK(f.navSamples > 0);
        CHECK(f.navOnDeck * 100 >= f.navSamples * 90);

        // 3. THE LOOK. Furniture measures its lateral placement off this band;
        //    a city built from a baked graph used to report a default 3.5 m beside
        //    its real 5 m one.
        CHECK(f.sidewalk > 0.5);
        CHECK(f.curb > 0.0);

        // 4. THE PARKING BAND. No band, no bays: the sim parked 2249 cars on the
        //    verge and stacked 2065 of them on each other.
        CHECK(f.streetEdges > 0);
        CHECK(f.edgesWithParking * 4 >= f.streetEdges);

        // 5. THE KEEP-OUT. `terrain.flatten` means two things — "grade this" and
        //    "do not scatter here" — and a builder that replaces the ground instead
        //    of carving it satisfied the first and silently dropped the second, so
        //    the vegetation scatter planted trees down the middle of the roads.
        CHECK(f.flattenRegions > 0);
        CHECK(f.carriagewaySamples > 0);
        CHECK(f.carriagewayKeptOut * 100 >= f.carriagewaySamples * 90);

        // 6. THE FREEWAY, IF THERE IS ONE, is in the same graph as the streets — a
        //    separate corridor nobody routes on was the legacy failure (ADR-0089).
        //    The right-of-way itself is consumed inside the load and is not a world
        //    component, so it is the lot pass's own gate that owns it, not this one.

        // 7. SOMETHING TO BUILD ON.
        CHECK(f.lotBuildings > 100);
    }
}

// ============================================================================
// NOTHING THE CITY DRESSES ITSELF WITH STANDS IN A ROAD
//
// Glenn, 2026-09-21, looking at metro_lanes: "the blocks, lots, buildings look
// fine. Things related to the road are broken. Cars aren't parked on the side of
// the street properly. Trees are in the middle of the road. Bus stops aren't on
// the sidewalks ... Please write validation tests to verify that trees, rocks,
// stop signs, stop lights, street signs, bus stops, benches -- really any
// obstacle or furniture doesn't intersect with the roads."
//
// One predicate, asked of every class of thing the city plants: is this point on
// the DRIVING SURFACE? The deck field answers it for whichever builder paved the
// road (`depthInside` > 0 = inside the built asphalt, and it returns HOW FAR in,
// which is what makes a failure actionable instead of a yes/no).
//
// Why a single test over all of them rather than one per subsystem: every one of
// these placements measures off its own idea of where the kerb is — a width on a
// graph edge, a `look.sidewalk`, a nav link's half-width — and those ideas drift
// apart silently. The asphalt is the only thing that cannot be wrong about where
// it is.
// ============================================================================

namespace {

struct RoadIntrusion {
    std::string what;
    int count = 0;
    int total = 0;
    double worst = 0;
    engine::Vec2 worstAt{0, 0};
    std::vector<std::pair<double, engine::Vec2>> sites;   // the deepest few, for the dump
};

// How far INSIDE the built driving surface this point lies, over every deck in
// the world. 0 = not on a road.
double depthOnAnyDeck(const std::vector<const engine::RoadDeckField*>& decks, double x, double z) {
    double worst = 0;
    for (const engine::RoadDeckField* d : decks) worst = std::max(worst, d->depthInside(x, z));
    return worst;
}

void note(RoadIntrusion& r, const std::vector<const engine::RoadDeckField*>& decks, double x,
          double z, double allow) {
    ++r.total;
    const double d = depthOnAnyDeck(decks, x, z);
    if (d <= allow) return;
    ++r.count;
    if (d > r.worst) { r.worst = d; r.worstAt = engine::Vec2(x, z); }
    r.sites.push_back({d, engine::Vec2(x, z)});
}

}  // namespace

TEST_CASE(city_furniture_never_stands_in_a_road) {
    const char* kCities[] = {"metro_v2_test.json", "metro_lanes.json"};
    for (const char* name : kCities) {
        std::unique_ptr<Renderer> renderer = Renderer::create();
        RendererMeshUploader uploader(*renderer);
        AssetManager assets(uploader);
        World world;
        RenderView view;
        const bool loaded = LevelLoader::load(levelsDir() + "/" + name, world, *renderer, view,
                                              assets, /*editorMode=*/false);
        CHECK(loaded);
        if (!loaded) continue;

        std::vector<const engine::RoadDeckField*> decks;
        world.each<engine::RoadDeck>([&](Entity, engine::RoadDeck& d) { decks.push_back(&d.field); });
        CHECK(!decks.empty());
        if (decks.empty()) continue;

        // SCATTER — trees and rocks. Instanced, so the transforms ARE the trunks.
        // A trunk is allowed nothing: a tree in the road is the thing Glenn saw.
        RoadIntrusion scatter{"trees + rocks"};
        world.each<InstanceGroup>([&](Entity, InstanceGroup& g) {
            // SCENERY only. Furniture, paint and structure ride in InstanceGroups too, and
            // some of those carry LOCAL transforms whose translation is the origin — 198 of
            // them read as "at (0,0)", which sits in a road on this city and turned the
            // scatter count into a fiction. A measurement that cannot say WHICH thing it
            // measured is not a measurement.
            if (g.drawClass != DrawClass::Scenery) return;
            for (const Mat4& m : g.transforms) note(scatter, decks, m.m[0][3], m.m[2][3], 0.0);
        });

        // STREET FURNITURE — signal poles and lamp posts. The pole FOOT is what
        // must be clear; a mast arm reaching over the near lane is the point of it.
        RoadIntrusion poles{"signal poles"}, lamps{"street lamps"};
        world.each<engine::StreetFurniture>([&](Entity, engine::StreetFurniture& sf) {
            for (const engine::StreetFurniture::Signal& s : sf.signalPoles)
                note(poles, decks, s.base.x, s.base.z, 0.0);
            // A lamp HEAD legitimately overhangs; its post is directly below it.
            for (const Vec3& h : sf.lampHeads) note(lamps, decks, h.x, h.z, 0.0);
        });

        // BUS STOPS — the pole, the sign and the bench. A BusStop::pos is a nav
        // NODE and sits on the centreline by construction, so asking about THAT
        // measures nothing; what must be clear is where the furniture LANDED,
        // which buildBusStopProps reports for exactly this audit.
        RoadIntrusion stops{"bus stop furniture"};
        {
            citysim::CityRenderSystem city;
            if (city.build(world, &assets, nullptr)) {
                std::vector<Vec3> at;
                // Height does not matter to this audit — only WHERE the prop landed
                // in plan — so the deck (or 0) is a sufficient ground for the call.
                citysim::buildBusStopProps(
                    world, assets, city.sim().buses(), city.nav(),
                    [&](Real x, Real z) {
                        double y = 0;
                        for (const engine::RoadDeckField* d : decks)
                            if (d->heightAt(x, z, 4.0, &y)) return static_cast<Real>(y);
                        return Real(0);
                    },
                    nullptr, &at, decks.front());
                for (const Vec3& p : at) note(stops, decks, p.x, p.z, 0.0);
            }
        }

        const RoadIntrusion* all[] = {&scatter, &poles, &lamps, &stops};
        for (const RoadIntrusion* r : all) {
            std::printf("    [furniture] %-14s %-20s %d of %d in a road%s\n", name, r->what.c_str(),
                        r->count, r->total,
                        r->count ? (" — worst " + std::to_string(r->worst) + " m in at (" +
                                    std::to_string(r->worstAt.x) + ", " +
                                    std::to_string(r->worstAt.y) + ")")
                                       .c_str()
                                 : "");
        }
        for (const RoadIntrusion* r : all) {
            if (r->count == 0) continue;
            std::vector<std::pair<double, engine::Vec2>> worst = r->sites;
            std::sort(worst.begin(), worst.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            for (std::size_t i = 0; i < worst.size() && i < 4; ++i)
                std::printf("        %s deepest %zu: %.2f m in at (%.1f, %.1f)\n", r->what.c_str(),
                            i, worst[i].first, worst[i].second.x, worst[i].second.y);
        }
        for (const RoadIntrusion* r : all) CHECK(r->count == 0);
    }
}
