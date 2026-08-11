#include "test_framework.h"

#include "../src/engine/procgen/city/lot_mesh.h"
#include "../src/engine/procgen/city/lot_city.h"
#include "../src/engine/procgen/city/material_set.h"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {

// Build one recipe on a plain w x d envelope, the way the city pass does.
BuiltBuilding build(const char* recipe, Real w, Real d, int storeys,
                    std::uint32_t seed = 5) {
    static PaletteLibrary palettes = stockPalettes();
    static RecipeBook book = stockRecipes();
    const LotRecipe* rec = book.byName(recipe);
    if (!rec) return BuiltBuilding{};
    Shape2 env = rectShape(0, 0, w, d);
    for (Edge2& e : env.outer.edges) e.tag = EdgeTag::Side;
    env.outer.edges[0].tag = EdgeTag::Street;
    LotTags tags = measureLot(env, {}, StreetClass::Street, true, 0.7);
    tags.maxStoreys = storeys;
    return buildFromRecipe(*rec, env, tags, palettes, 3, seed);
}

int triangles(const BuildingMesh& m) {
    int t = 0;
    for (const RenderMesh& p : m.parts) t += static_cast<int>(p.indices.size() / 3);
    return t;
}

}  // namespace

// THE BUG THIS EXISTS FOR: the engine's winding convention is not the obvious
// one. MeshBuilder::emitQuad orders indices so that cross(b - a, c - a) points
// AGAINST the shading normal, and every mesh in the engine is built that way.
// A hand-wound triangle that disagrees is backfacing against the entire rest of
// the world — and it does not look like a winding bug, it looks like a hole in
// the roof. (Written the wrong way round first; the roofs vanished.)
TEST_CASE(lotmesh_winding_matches_the_engine_convention) {
    const BuiltBuilding b = build("office_block", 34, 30, 9);
    const BuildingMesh m = meshBuilding(b);
    CHECK(triangles(m) > 200);
    int agree = 0, disagree = 0;
    for (const RenderMesh& p : m.parts)
        for (std::size_t i = 0; i + 2 < p.indices.size(); i += 3) {
            const Vertex& v = p.vertices[p.indices[i]];
            const Vec3 a = v.position;
            const Vec3 bb = p.vertices[p.indices[i + 1]].position;
            const Vec3 c = p.vertices[p.indices[i + 2]].position;
            const Vec3 gn = cross(bb - a, c - a);
            if (gn.lengthSquared() < 1e-12) continue;
            (dot(normalize(gn), v.normal) < 0 ? agree : disagree)++;
        }
    CHECK(agree > 200);
    CHECK(disagree == 0);
}

// A wall has an outside, and it is the side away from the building. Getting it
// backwards builds every building inside-out, which the facing debug view
// cannot show you — emitQuad winds geometry to agree with whatever normal it is
// handed, so the near wall is culled and you see the far wall's flipped face.
TEST_CASE(lotmesh_walls_face_outward) {
    const BuiltBuilding b = build("apartment_slab", 26, 16, 8);
    const BuildingMesh m = meshBuilding(b);
    const Shape2& foot = b.surfaces.levels[0].plans[0];
    const Vec2 c = centroid(foot);

    int out = 0, in = 0;
    for (const RenderMesh& p : m.parts) {
        // Walls only: the reveals of a window face along the wall, and a slab
        // faces up or down.
        if (p.materialIndex != static_cast<int>(b.materials.forTier(0).wall)) continue;
        for (std::size_t i = 0; i + 2 < p.indices.size(); i += 3) {
            const Vertex& v = p.vertices[p.indices[i]];
            if (std::fabs(v.normal.y) > 0.5) continue;
            const Vec3 mid = (v.position + p.vertices[p.indices[i + 1]].position +
                              p.vertices[p.indices[i + 2]].position) * (1.0 / 3.0);
            const Vec2 away(mid.x - c.x, mid.z - c.y);
            if (away.lengthSquared() < 1e-6) continue;
            const Vec2 n(v.normal.x, v.normal.z);
            (dot(n, normalize(away)) > 0 ? out : in)++;
        }
    }
    CHECK(out > 100);
    // A plain slab has no courtyard, so nothing legitimately faces inward.
    CHECK(in == 0);
}

// A courtyard is a real hole, and its walls face INTO the court. This is the
// first thing in the codebase to hand `triangulateWithHoles` an actual hole —
// its only other caller passes an empty list.
TEST_CASE(lotmesh_a_courtyard_is_a_hole_with_inward_walls) {
    BuiltBuilding b;
    for (std::uint32_t s = 1; s < 40 && b.surfaces.levels.empty(); ++s) {
        BuiltBuilding t = build("office_block", 40, 36, 8, s);
        if (!t.surfaces.levels.empty() && !t.surfaces.levels[0].plans.empty() &&
            !t.surfaces.levels[0].plans[0].holes.empty())
            b = t;
    }
    if (b.surfaces.levels.empty()) return;   // no courtyard drawn at this size
    const BuildingMesh m = meshBuilding(b);
    const Shape2& foot = b.surfaces.levels[0].plans[0];
    const Vec2 c = centroid(foot);
    int inward = 0;
    for (const RenderMesh& p : m.parts)
        for (std::size_t i = 0; i + 2 < p.indices.size(); i += 3) {
            const Vertex& v = p.vertices[p.indices[i]];
            if (std::fabs(v.normal.y) > 0.5) continue;
            const Vec3 mid = (v.position + p.vertices[p.indices[i + 1]].position +
                              p.vertices[p.indices[i + 2]].position) * (1.0 / 3.0);
            const Vec2 away(mid.x - c.x, mid.z - c.y);
            if (away.lengthSquared() < 1e-6) continue;
            if (dot(Vec2(v.normal.x, v.normal.z), normalize(away)) < -0.5) ++inward;
        }
    CHECK(inward > 0);
}

// The distance LODs have to be dramatically cheaper or they are not LODs, and
// they have to read off the same blueprint or the windows move when the swap
// happens.
TEST_CASE(lotmesh_lods_are_cheaper_and_the_proxy_keeps_the_footprint) {
    const BuiltBuilding b = build("glass_tower", 46, 44, 34);
    LotMeshParams full;
    LotMeshParams flat;
    flat.detail = FacadeDetail::Flat;
    const BuildingMesh mf = meshBuilding(b, full);
    const BuildingMesh ml = meshBuilding(b, flat);
    CHECK(triangles(mf) > 2000);
    CHECK(triangles(ml) * 2 < triangles(mf));
    // The proxy is the footprint extruded, not a bounding box: its own bounds
    // must match the building's.
    CHECK(mf.proxy.indices.size() / 3 > 4);
    CHECK(static_cast<int>(mf.proxy.indices.size() / 3) < triangles(mf) / 8);
    Real lo = 1e30, hi = -1e30;
    for (const Vertex& v : mf.proxy.vertices) {
        lo = std::min(lo, static_cast<Real>(v.position.y));
        hi = std::max(hi, static_cast<Real>(v.position.y));
    }
    CHECK(std::fabs(hi - mf.height) < 0.5);
    CHECK(std::fabs(lo) < 0.5);
}

// A tower must not cost a million triangles. It did: the mass stack's taper
// runs through the sampled field, and a marching-squares contour repeated once
// per storey put 8 684 edges in a 24-storey plan and 13.8 MILLION triangles in
// its mesh. The field refits its results now; this is the guard.
TEST_CASE(lotmesh_a_tower_has_a_sane_triangle_budget) {
    for (const char* r : {"office_tower", "glass_tower", "stepped_tower",
                          "profile_tower", "hotel"}) {
        const BuiltBuilding b = build(r, 44, 40, 30, 91);
        if (b.surfaces.levels.empty()) continue;
        int edges = 0;
        for (const Level& L : b.surfaces.levels)
            for (const Shape2& s : L.plans) edges += static_cast<int>(s.outer.size());
        CHECK(edges < 40 * static_cast<int>(b.surfaces.levels.size()));
        CHECK(triangles(meshBuilding(b)) < 60000);
    }
}

TEST_CASE(lotmesh_is_deterministic) {
    const BuiltBuilding b = build("walkup", 20, 22, 5);
    const BuildingMesh a = meshBuilding(b);
    const BuildingMesh c = meshBuilding(b);
    CHECK(a.parts.size() == c.parts.size());
    CHECK(triangles(a) == triangles(c));
    CHECK(std::fabs(a.height - c.height) < 1e-9);
}

// Every vertex has to be a number. A NaN reaches the GPU as a hole in the world
// and there is no message when it does.
TEST_CASE(lotmesh_emits_no_degenerate_geometry) {
    for (const char* r : {"cottage_house", "shopfront", "civic_hall", "warehouse",
                          "university_campus"}) {
        const BuiltBuilding b = build(r, 40, 34, 4, 17);
        if (b.surfaces.levels.empty()) continue;
        const BuildingMesh m = meshBuilding(b);
        for (const RenderMesh& p : m.parts) {
            for (const Vertex& v : p.vertices) {
                CHECK(std::isfinite(v.position.x) && std::isfinite(v.position.y) &&
                      std::isfinite(v.position.z));
                CHECK(std::isfinite(v.normal.x) && std::isfinite(v.normal.y) &&
                      std::isfinite(v.normal.z));
                CHECK(std::fabs(v.normal.lengthSquared() - 1.0) < 0.05);
            }
            for (uint32_t i : p.indices) CHECK(i < p.vertices.size());
        }
    }
}

// The city pass: blocks in, the SAME outputs the old pass produces out. If this
// diverges, every consumer downstream — colliders, places, the chunker, the
// HLOD — has to learn which pass built the city, and the migration stops being
// a flag.
TEST_CASE(lotmesh_city_pass_fills_the_same_outputs) {
    std::vector<Poly2> blocks;
    for (int i = 0; i < 3; ++i) {
        const Real x = i * 160.0;
        blocks.push_back({{x, 0}, {x + 120, 0}, {x + 120, 110}, {x, 110}});
    }
    LotParams lp;
    lp.seed = 4;
    lp.lotSystem = true;
    lp.center = Vec2(60, 55);
    lp.hubs = {{Vec2(60, 55), 0}, {Vec2(340, 55), 2}};

    LotPlanDebug debug;
    std::vector<RenderMesh> parts, flat;
    std::vector<LotBuilding> lots;
    growLotSystemBlocks(blocks, lp, &debug, &parts, &flat, &lots);

    CHECK(parts.size() == static_cast<std::size_t>(PartId::Count));
    for (std::size_t i = 0; i < parts.size(); ++i)
        CHECK(parts[i].materialIndex == static_cast<int>(i));
    int tris = 0;
    for (const RenderMesh& p : parts) tris += static_cast<int>(p.indices.size() / 3);
    CHECK(tris > 1000);
    CHECK(!lots.empty());
    CHECK(!debug.lots.empty());

    // A downtown hub and a residential one, on identical blocks, must not build
    // the same city — that is the whole point of asking the land what it is for.
    int downtown = 0, suburb = 0;
    for (const LotBuilding& l : lots) {
        if (l.type == "green") continue;
        (l.site.x < 200 ? downtown : suburb)++;
    }
    CHECK(downtown > 0 && suburb > 0);
    CHECK(suburb > downtown);        // houses parcel finer than tower plates

    // Every building carries its real footprint for the collider, never an
    // empty polygon — an L-plan extruded from a box spills onto the sidewalk.
    for (const LotBuilding& l : lots)
        if (l.type != "green") {
            CHECK(l.plan.size() >= 3);
            CHECK(l.height > 1.0);
        }
}
