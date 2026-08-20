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
#include "../src/engine/system.h"
#include "../src/engine/world.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
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

// Every shipped level, sorted so a failure names the same file run to run.
// `*.json.cameras.json` sidecars are camera bookmarks, not levels.
std::vector<std::string> shippedLevels() {
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(levelsDir(), ec)) {
        const std::string name = entry.path().filename().string();
        if (name.size() < 5 || name.compare(name.size() - 5, 5, ".json") != 0) continue;
        if (name.find(".cameras.json") != std::string::npos) continue;
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
