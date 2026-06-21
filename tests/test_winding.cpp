// Winding audit: every RenderMesh the engine generates must wind front faces
// clockwise — a triangle (a,b,c)'s outward geometric normal cross(c-a, b-a)
// agrees with its shading normals (see MeshBuilder::recomputeNormals and the
// Metal viewer's clockwise-front back-face cull). The offline path tracer is
// two-sided so it never catches an inverted builder; this test does, by
// checking the convention empirically across every builder rather than reading
// index order by hand. Double-sided foliage (leaf cards) is excluded — it is
// deliberately wound both ways.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/terrain.h"
#include "../src/engine/procgen/terrain_field.h"
#include "../src/engine/procgen/noise.h"
#include "../src/engine/procgen/sdf.h"
#include "../src/engine/procgen/rock.h"
#include "../src/engine/procgen/tree.h"
#include "../src/engine/procgen/city/polygon.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/street_kit.h"

using namespace engine;

namespace {

// How many of `mesh`'s triangles are wound against the engine convention: the
// geometric normal cross(c-a, b-a) points materially *opposite* the triangle's
// averaged shading normal. We test the angle, not just the sign: a true
// inversion is ~180° (cos ≈ -1, e.g. a flipped cap), whereas a sliver in a
// dual-contoured SDF mesh — where a vertex's gradient normal lies nearly in the
// (near-zero-area) triangle's plane — is ~90° (cos ≈ 0) and is harmless, since
// such meshes are globally winding-consistent by construction. The -0.3 cut
// (>107° apart) sits safely between the two. Degenerate faces/normals are
// skipped. `total` receives the number of triangles actually tested.
int backfaceCount(const RenderMesh& mesh, int& total) {
    total = 0;
    int bad = 0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const Vertex& a = mesh.vertices[mesh.indices[i]];
        const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
        const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
        Vec3 geo = cross(c.position - a.position, b.position - a.position);
        Vec3 shade = a.normal + b.normal + c.normal;
        if (geo.lengthSquared() < 1e-16 || shade.lengthSquared() < 1e-12)
            continue;   // degenerate face or normals — orientation undefined
        ++total;
        if (dot(normalize(geo), normalize(shade)) < -0.3) ++bad;
    }
    return bad;
}

// Assert a solid (single-sided) mesh is consistently wound front-out: no
// triangle faces against its shading normal, and there is something to test.
void checkSolid(const RenderMesh& mesh, const char* what) {
    int total = 0;
    int bad = backfaceCount(mesh, total);
    if (total == 0 || bad != 0)
        std::printf("    winding: %s -> %d of %d triangles inside-out\n", what,
                    bad, total);
    CHECK(total > 0);
    CHECK(bad == 0);
}

}  // namespace

// --- MeshBuilder primitives -------------------------------------------------
TEST_CASE(winding_meshbuilder_primitives_face_out) {
    checkSolid(MeshBuilder::box(Vec3(2, 1, 3)), "box");
    checkSolid(MeshBuilder::sphere(1.0f), "sphere");
    checkSolid(MeshBuilder::cylinder(1.0f, 2.0f), "cylinder");
    checkSolid(MeshBuilder::plane(4.0f, 4.0f), "plane");
    checkSolid(MeshBuilder::cone(1.0f, 2.0f), "cone");
    checkSolid(MeshBuilder::wedge(Vec3(2, 1, 3)), "wedge");
    checkSolid(MeshBuilder::torus(2.0f, 0.5f), "torus");
    checkSolid(MeshBuilder::capsule(0.5f, 2.0f), "capsule");
}

// --- Terrain (heightfield + noise) ------------------------------------------
TEST_CASE(winding_terrain_faces_up) {
    Noise noise(7);
    TerrainParams tp;
    checkSolid(generateTerrain(tp, noise), "generateTerrain");
    checkSolid(generateTerrainRing(tp, noise, tp.size * 0.5f, tp.size, 16),
               "generateTerrainRing");
    auto chunks = generateTerrainChunks(tp, noise, 2, 64.0f, 16, 0.0f);
    CHECK(!chunks.empty());
    checkSolid(chunks.front().mesh, "generateTerrainChunks");

    HeightField h = heightFbm(7, 0.02, 12.0, 4);
    checkSolid(bakeHeightMesh(h, 200.0, 48, Vec3(0.3, 0.4, 0.25)),
               "bakeHeightMesh");
}

// --- SDF polygonization + rocks ---------------------------------------------
TEST_CASE(winding_sdf_and_rock_face_out) {
    Sdf blob = sdfUnion(sdfSphere(Vec3(-0.4, 0, 0), 0.8),
                        sdfSphere(Vec3(0.4, 0, 0), 0.8));
    SdfBounds b{Vec3(-1.6, -1.6, -1.6), Vec3(1.6, 1.6, 1.6)};
    checkSolid(polygonizeSdf(blob, b, 28), "polygonizeSdf");

    Noise rockNoise(11);
    RockParams rp;
    checkSolid(generateRock(rp, rockNoise), "generateRock");
    RockSdfParams rsp;
    checkSolid(generateRockSdf(rsp, 11), "generateRockSdf");
}

// --- Trees (branch tubes; leaves are double-sided, excluded) ----------------
TEST_CASE(winding_tree_branches_face_out) {
    TreeParams tp;
    TreeMesh tm = growTree(tp, 3);
    checkSolid(tm.branches, "tree branches");
}

// --- City: shape grammar, road mesh, street furniture -----------------------
TEST_CASE(winding_city_geometry_faces_out) {
    Poly2 footprint = {{0, 0}, {18, 0}, {18, 18}, {0, 18}};
    Scope s = scopeFromFootprint(footprint, 0.0, 24.0);
    BuildingParams bp;
    bp.floors = 6;
    bp.seed = 3;
    BuildingMesh bm = growBuilding(s, bp);
    CHECK(!bm.parts.empty());
    for (size_t i = 0; i < bm.parts.size(); ++i)
        checkSolid(bm.parts[i], "building part");
    checkSolid(bm.proxy, "building proxy");

    GridRoadParams grp;
    grp.extent = 180;
    grp.cellSize = 70;
    grp.seed = 2;
    RoadGraph g = planarize(gridRoads(grp));
    RoadMeshParams rmp;
    rmp.color = Vec3(0.08, 0.08, 0.09);
    checkSolid(buildRoadMesh(g, rmp), "buildRoadMesh");

    checkSolid(streetLamp(), "streetLamp");
    checkSolid(trafficSignalProto(), "trafficSignalProto");
}
