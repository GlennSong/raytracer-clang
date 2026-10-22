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
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/ladders.h"
#include "../src/engine/procgen/city/shop_signs.h"
#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/real_tree.h"
#include "../src/engine/procgen/stylized_tree.h"
#include "../src/engine/procgen/stylized_rock.h"
#include "../src/engine/procgen/lsystem.h"
#include "../src/engine/procgen/grass.h"
#include "../src/engine/procgen/planet.h"
#include "../src/engine/procgen/terrain_lod.h"
#include "../src/engine/procgen/vehicle_mesh.h"
#include "../src/engine/procgen/vehicle/car_mesh.h"
#include "../src/engine/procgen/vehicle/train_mesh.h"
#include "../src/engine/procgen/vehicle/occupant.h"
#include "../src/engine/anim/mannequin.h"
#include "../src/apps/citysim/city_meshes.h"
#include "../src/apps/citysim/station_boards.h"
#include "../src/apps/citysim/scripting/vehicle_body.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/script_assets.h"
#include "../src/engine/procgen/polymesh.h"
#include "../src/engine/procgen/city/structure_set.h"
#include "../src/engine/procgen/city/water_mesh.h"
#include "mesh_invariants.h"

#include <fstream>
#include <sstream>

#include <cstdlib>
#include <map>
#include <string>

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
    const mesh_invariants::Audit a = mesh_invariants::audit(mesh);
    total = static_cast<int>(a.tris - a.degenerate - a.nonFinite);
    return static_cast<int>(a.inward);
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

    // Roads are NOT checked here any more. buildRoadMesh is gone (roads-v2: the
    // swept lattice is the one mesher), and `checkSolid` is a per-face test against
    // each face's own vertex normals — valid for soup meshes like these, but it
    // false-positives on a shared-vertex lattice, where a crease vertex carries the
    // average of the two faces it folds between (it flags exactly half the curb
    // risers on a plain street and none of the deck). Road winding is gated in
    // tests/test_road_lattice.cpp: `lattice_winding_matches_the_engine_convention`
    // pins it against MeshBuilder::box, and
    // `per_face_normal_test_is_invalid_for_a_shared_vertex_lattice` records why
    // this check cannot live here.
    checkSolid(streetLamp(), "streetLamp");
    checkSolid(trafficSignalProto(), "trafficSignalProto");
}

// ============================================================================
// THE WHOLE-ENGINE AUDIT (Glenn, 2026-10-13: "the entire procedural generation sits on this ... Everything, and I mean
// everything in this engine should use the same tooling ... this way we know that every polygon ends up being right
// when it's created"). mesh_invariants::audit over every producer -- inward triangles, degenerate ones, non-finite
// positions, non-unit normals -- counted per producer so a regression names its maker. A producer that is clean
// CHECKs it; one that is not yet prints its counts, which the mesh library work takes to zero.
// ============================================================================
using mesh_invariants::Audit;
using mesh_invariants::audit;
using mesh_invariants::printAudit;

namespace {
Poly2 rectPlan(Real w, Real d) { return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}}; }
Poly2 lPlan(Real w, Real d) {
    return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, 0}, {0, 0}, {0, d / 2}, {-w / 2, d / 2}};
}
// Lots as the city cuts them: a U (two concave corners facing each other), a flatiron wedge (an acute corner), a
// trapezoid with a 0.4 m edge where a corner was clipped, and an irregular seven-sided lot with a shallow notch and a
// near-collinear vertex. The island's bake had flipped cornice and wall faces the rect and L never show.
std::vector<Poly2> lotPlans() {
    return {
        {{-14, -10}, {14, -10}, {14, 10}, {6, 10}, {6, -2}, {-6, -2}, {-6, 10}, {-14, 10}},
        {{-16, -9}, {16, -9}, {-10, 9}},
        {{-12, -10}, {12, -10}, {12.4, -9.6}, {9, 10}, {-11, 10}},
        {{-13, -11}, {4, -12}, {14, -4}, {13.9, 3}, {12, 11}, {-2, 9}, {-3, 11.5}, {-12, 10}},
    };
}

// Audit a building mesh, by part, into `byPart`; returns the whole.
Audit auditBuilding(const BuildingMesh& bm, std::map<int, Audit>& byPart) {
    Audit all;
    for (const RenderMesh& p : bm.parts) {
        const Audit a = audit(p);
        byPart[p.materialIndex].add(a);
        all.add(a);
    }
    return all;
}

// The parts drawn from BOTH sides (the clear glass, the beacon halos): their back faces are seen, so a reversed
// triangle there is not a hole -- still counted, listed apart.
bool twoSided(int part) {
    return part == static_cast<int>(PartId::GlassClear) || part == static_cast<int>(PartId::BeaconGlow) ||
           part == static_cast<int>(PartId::BeaconHaze);
}
}  // namespace

// EVERY RECIPE, outside and in: the architect's whole registry grown on a rect and an L, exterior and interior.
TEST_CASE(mesh_audit_every_building_recipe) {
    std::map<int, Audit> byPart;
    std::map<std::string, Audit> byRecipe;
    Audit total;
    for (int i = 0; i < architectRecipeCount(); ++i) {
        const char* name = architectRecipeName(i);
        for (uint32_t seed : {3u, 17u}) {
            BuildingRecipe r;
            if (!architectRecipeByName(name, seed, 0.6, 24, 600, r)) continue;
            if (r.massing == BuildingRecipe::Massing::Park || r.massing == BuildingRecipe::Massing::Plaza) continue;
            std::vector<Poly2> plans = {rectPlan(24, 20), lPlan(28, 24)};
            for (const Poly2& lp : lotPlans()) plans.push_back(lp);
            for (const Poly2& plan : plans) {
                Audit a = auditBuilding(growPlanBuilding(plan, r.params, 0.0), byPart);
                a.add(auditBuilding(growInterior(plan, r.params, 0.0), byPart));
                byRecipe[name].add(a);
                total.add(a);
            }
            if (r.massing == BuildingRecipe::Massing::BoxMass) {
                Scope s;
                s.origin = Vec3(-10, 0, -8);
                s.size = Vec3(20, r.params.floors * 3.5, 16);
                const Audit a = auditBuilding(growBuilding(s, r.params), byPart);
                byRecipe[name].add(a);
                total.add(a);
            }
        }
    }
    for (const auto& [n, a] : byRecipe)
        if (!a.clean()) printAudit(("recipe " + n).c_str(), a);
    for (const auto& [p, a] : byPart)
        printAudit(("part " + std::to_string(p) + (twoSided(p) ? " (two-sided)" : "")).c_str(), a);
    printAudit("ALL BUILDINGS", total);
    CHECK(total.tris > 100000);
    CHECK(total.clean());
}

// EVERY FURNITURE PIECE, in a spread of variants.
TEST_CASE(mesh_audit_every_furniture_piece) {
    Audit total;
    for (int i = 0; i < kPieceCount; ++i) {
        Audit piece;
        for (uint32_t variant : {0u, 9u, 42u, 101u}) {
            const FurniturePiece& fp = furniturePiece(static_cast<Piece>(i), variant);
            for (const RenderMesh& m : fp.mesh) piece.add(audit(m));
        }
        if (!piece.clean()) printAudit(furniturePieceName(static_cast<Piece>(i)), piece);
        total.add(piece);
    }
    printAudit("ALL FURNITURE", total);
    CHECK(total.tris > 1000);
    CHECK(total.clean());
}

// THE STREET: lamps, signals, bins, racks, news boxes, tree pits, signs, ladders.
TEST_CASE(mesh_audit_street_kit_signs_ladders) {
    Audit total;
    const std::pair<const char*, RenderMesh> meshes[] = {
        {"streetLamp", streetLamp()},
        {"trafficSignalProto", trafficSignalProto()},
        {"litterBinProto", litterBinProto()},
        {"bikeRackProto", bikeRackProto()},
        {"newsBoxesProto", newsBoxesProto()},
        {"treePitProto", treePitProto()},
        {"openSignMesh", openSignMesh()},
        {"busSignMesh", busSignMesh(0, 2)},
        {"ladderMesh", ladderMesh([] { LadderSpec l; l.out = Vec2(0, 1); l.topY = 6; l.landY = 6; l.cage = true; return l; }())},
    };
    for (const auto& [n, m] : meshes) {
        const Audit a = audit(m);
        printAudit(n, a);
        total.add(a);
    }
    printAudit("ALL STREET KIT", total);
    CHECK(total.clean());
}

// NATURE: every real species, the stylized trees, an L-system tree, rocks, grass, terrain LOD nodes, a planet. Leaf
// cards, fronds and grass blades are drawn two-sided, and a stylized canopy's normals are LEANED toward its crown's
// centre (MeshBuilder::leanNormals: the crown lights as one volume) -- so on a clump's inner side they point away
// from its face on purpose. Those are listed, not gated.
TEST_CASE(mesh_audit_nature) {
    Audit solid, cards, leaned;
    for (int i = 0; i < static_cast<int>(RealSpecies::Count); ++i) {
        const RealTree t = realTree(static_cast<RealSpecies>(i), 5, 12.0);
        const Audit b = audit(t.bark);
        if (!b.clean()) printAudit((std::string("real bark ") + realSpeciesName(static_cast<RealSpecies>(i))).c_str(), b);
        solid.add(b);
        cards.add(audit(t.foliage));
    }
    for (int i = 0; i <= static_cast<int>(StylizedShape::FloweringShrub); ++i) {
        StylizedTreeParams p;
        p.shape = static_cast<StylizedShape>(i);
        const StylizedTree t = stylizedTree(7, p);
        const Audit b = audit(t.bark), c = audit(t.canopy);
        if (!b.clean()) printAudit(("stylized bark " + std::to_string(i)).c_str(), b);
        solid.add(b);
        leaned.add(c);
    }
    {
        LSystem sys;
        sys.rule('F', "F[+F]F[-F][&F]");
        TurtleParams tp;
        const Audit a = audit(buildTurtleMesh(sys.expand("F", 3), tp));
        printAudit("lsystem turtle", a);
        solid.add(a);
    }
    {
        const TreeMesh tm = growTree(TreeParams{}, 3);
        cards.add(audit(tm.leaves));
    }
    for (uint32_t seed : {1u, 2u, 3u}) {
        const Audit a = audit(stylizedRock(seed, StylizedRockParams{}));
        if (!a.clean()) printAudit(("stylized rock " + std::to_string(seed)).c_str(), a);
        solid.add(a);
    }
    {
        Noise noise(1);
        TerrainParams p;
        Audit lod;
        for (const LodNode& n : {LodNode{0.0f, 0.0f, 100.0f, 0}, LodNode{200.0f, -100.0f, 50.0f, 1}})
            lod.add(audit(generateLodNodeMesh(p, noise, n, 32).mesh));
        printAudit("terrain lod nodes", lod);
        solid.add(lod);
    }
    {
        const Audit a = audit(generatePlanet(PlanetParams{}, 4));
        printAudit("planet", a);
        solid.add(a);
    }
    const Audit grass = audit(grassClump(3, GrassClumpParams{}));
    printAudit("grass clump (two-sided)", grass);
    printAudit("grass card (two-sided)", audit(grassCardMesh(GrassClumpParams{})));
    printAudit("leaf cards (two-sided)", cards);
    printAudit("stylized canopies (leaned normals)", leaned);
    printAudit("ALL NATURE (solid)", solid);
    CHECK(solid.tris > 10000);
    CHECK(solid.clean());
}

// VEHICLES AND PEOPLE: the C++ car, its shells, the trains, the occupant, the mannequin, the citysim's people and
// widgets, the station boards, and the Lua fleet (assets/scripts/vehicles.lua) slot by slot.
TEST_CASE(mesh_audit_vehicles_and_people) {
    using namespace citysim;
    Audit total;
    auto one = [&](const std::string& n, const RenderMesh& m) {
        const Audit a = audit(m);
        printAudit(n.c_str(), a);
        total.add(a);
    };
    {
        const CarMesh car = buildCarMesh(defaultSedanParams());
        Audit a;
        for (const RenderMesh& m : car.parts) a.add(audit(m));
        printAudit("buildCarMesh (sedan)", a);
        total.add(a);
    }
    for (int s = 0; s <= static_cast<int>(CarShellStyle::BoxTruck); ++s)
        one("buildCarShell " + std::to_string(s), buildCarShell(static_cast<CarShellStyle>(s), Vec3(0.5, 0.1, 0.1),
                                                                 Vec3(1.8, 1.5, 4.5)));
    for (const char* kind : {"intercity", "subway", "light_rail"}) {
        TrainBodySpec spec;
        spec.kind = kind;
        const TrainBodyMesh c = buildTrainCarriage(spec), l = buildTrainLocomotive(spec);
        one(std::string("train carriage ") + kind, c.body);
        one(std::string("train carriage glass ") + kind, c.glass);
        one(std::string("train door leaf ") + kind, c.doorLeaf);
        one(std::string("train locomotive ") + kind, l.body);
    }
    one("buildSeatedOccupant", buildSeatedOccupant());
    {
        const anim::Mannequin m = anim::buildMannequin(1.8);
        one("bakeMannequinMesh (bind)", anim::bakeMannequinMesh(m, anim::Pose::bind(m.skeleton)));
    }
    for (int o = 0; o < personOutfitCount(); ++o) {
        Audit a = audit(buildPersonMesh(0.3, o));
        a.add(audit(buildSeatedPersonMesh(o)));
        a.add(audit(buildGroundSittingPersonMesh(o)));
        if (!a.clean()) printAudit(("person outfit " + std::to_string(o)).c_str(), a);
        total.add(a);
    }
    one("ringXZ", ringXZ());
    one("arrowXZ", arrowXZ());
    one("stripXZ", stripXZ());
    one("wedgeXZ", wedgeXZ(0.6));
    one("stationBoardFrame", stationBoardFrame());
    one("stationBoardFaces", stationBoardFaces());

    // the Lua fleet
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/vehicles.lua");
    std::stringstream ss;
    ss << in.rdbuf();
    std::string err;
    CHECK(vm.doString(ss.str(), &err));
    for (int slot = 0; slot < fleetSlotCount(vm); ++slot) {
        CarBodyRecipe body;
        if (!loadFleetCarBody(vm, slot, body, &err)) continue;
        Audit a = audit(body.mesh);
        a.add(audit(body.chassis));
        printAudit(("lua fleet " + std::to_string(slot) + " " + body.className).c_str(), a);
        total.add(a);
    }
    printAudit("ALL VEHICLES AND PEOPLE", total);
    CHECK(total.tris > 10000);
    CHECK(total.clean());
}

// NOTHING IN A BUILDING IS SEE-THROUGH FROM BELOW (Glenn, 2026-10-13: "in the skyscraper stairwells there's issues
// too"): engine::auditUndersides over towers grown outside and in -- every floor, landing and tread over 2 m up has
// something facing down under it (a ceiling, a flight's soffit).
TEST_CASE(mesh_audit_building_undersides) {
    size_t hanging = 0, open = 0;
    for (const char* name : {"glass_tower", "art_deco_tower", "sky_exposure_tower", "stepped_tower", "condo_tower",
                             "office_slab", "apartments", "hotel", "hospital", "mixed_use"}) {
        BuildingRecipe r;
        if (!architectRecipeByName(name, 7, 0.8, 26, 700, r)) continue;
        const Poly2 plan = rectPlan(26, 22);
        const BuildingMesh out = growPlanBuilding(plan, r.params, 0.0), in = growInterior(plan, r.params, 0.0);
        std::vector<NamedMeshRef> refs;
        std::vector<std::string> names;
        for (const BuildingMesh* bm : {&out, &in})
            for (const RenderMesh& p : bm->parts) names.push_back("part " + std::to_string(p.materialIndex));
        size_t k = 0;
        for (const BuildingMesh* bm : {&out, &in})
            for (const RenderMesh& p : bm->parts) refs.push_back({names[k++], &p});
        const UndersideAudit u = auditUndersides(
            refs,
            [](const std::string& n) {
                return n != "part " + std::to_string(static_cast<int>(PartId::Glass)) &&
                       n != "part " + std::to_string(static_cast<int>(PartId::GlassClear)) &&
                       n != "part " + std::to_string(static_cast<int>(PartId::GlassLit));
            },
            [](double, double) { return 0.0; });
        if (u.open > 0) {
            std::printf("    %s (%d floors):\n", name, r.params.floors);
            printUndersideAudit(u, 4);
            std::printf("    [undersides]   first at %.2f %.2f %.2f\n", u.firstOpen.x, u.firstOpen.y, u.firstOpen.z);
            if (std::getenv("RT_UNDERSIDE_COLUMN")) printColumn(refs, u.firstOpen.x, u.firstOpen.z, 0.6);
        }
        hanging += u.hanging;
        open += u.open;
    }
    std::printf("    [undersides] buildings: %zu hanging, %zu open\n", hanging, open);
    CHECK(hanging > 1000);
    CHECK(open == 0);
}

// THE MODELLING LAYER (PolyMesh -> toParts, the Lua poly.*): lofts of an L and a star section, capped -- concave caps,
// which toParts used to fan from their first corner (triangles outside the face, some reversed) -- extruded and
// subdivided too.
TEST_CASE(mesh_audit_polymesh_concave_faces) {
    const std::vector<Vec2> ell = {{-1, -1}, {1, -1}, {1, 0}, {0, 0}, {0, 1}, {-1, 1}};
    std::vector<Vec2> star;
    for (int i = 0; i < 10; ++i) {
        const double a = 6.283185307179586 * i / 10, r = i % 2 ? 0.45 : 1.0;
        star.push_back(Vec2(r * std::cos(a), r * std::sin(a)));
    }
    Audit total;
    for (const std::vector<Vec2>& sec : {ell, star}) {
        PolyMesh m = loft({sec, sec}, {0.0, 1.5}, true, true);
        const std::vector<int> caps = selectGroup(m, "cap_end");
        for (const PolyMesh& pm : {m, subdivide(m, 1)}) {
            for (const auto& [mat, rm] : toParts(pm)) total.add(audit(rm));
        }
        PolyMesh e = m;
        extrude(e, caps, 0.4);
        for (const auto& [mat, rm] : toParts(e)) total.add(audit(rm));
    }
    printAudit("polymesh (L, star; loft, subdivide, extrude)", total);
    CHECK(total.tris > 100);
    CHECK(total.clean());
}

// THE SHAPE KIT'S OTHER WORDS and the small producers: rounded boxes (radius 0 to fat), lathes (a vase, a bowl, a
// plate, a closed knob), tubes (straight, bent, through vertical), ribbons, icospheres, the shape grammar's faceted
// latheMesh, a retaining wall with its bench, the sea.
TEST_CASE(mesh_audit_shape_kit_and_small_producers) {
    Audit total;
    auto one = [&](const std::string& n, const RenderMesh& m) {
        const Audit a = audit(m);
        if (!a.clean()) printAudit(n.c_str(), a);
        total.add(a);
    };
    for (double r : {0.0, 0.004, 0.05, 0.3})
        for (int segs : {1, 2, 4}) one("roundedBox r" + std::to_string(r), MeshBuilder::roundedBox(Vec3(0.6, 0.4, 0.8), r, segs));
    const std::vector<std::vector<std::pair<double, double>>> profiles = {
        {{0.0, 0.0}, {0.05, 0.0}, {0.075, 0.10}, {0.05, 0.22}, {0.035, 0.26}, {0.045, 0.28}, {0.0, 0.28}},
        {{0.0, 0.0}, {0.06, 0.0}, {0.14, 0.07}, {0.145, 0.075}, {0.0, 0.03}},
        {{0.0, 0.0}, {0.10, 0.0}, {0.14, 0.02}, {0.0, 0.02}},
        {{0.0, 0.0}, {0.03, 0.01}, {0.038, 0.035}, {0.03, 0.06}, {0.0, 0.07}}};
    for (const auto& pr : profiles) {
        one("lathe", MeshBuilder::lathe(pr, 16));
        std::vector<Vec2> p2;
        for (const auto& [r, y] : pr) p2.push_back(Vec2(r, y));
        one("latheMesh", latheMesh(Vec3(0, 0, 0), p2, 12, Vec3(1, 1, 1)));
    }
    one("tube straight", MeshBuilder::tube({Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(0, 2, 0)}, {0.1, 0.08, 0.05}, 8));
    one("tube bent", MeshBuilder::tube({Vec3(0, 0, 0), Vec3(0.2, 1, 0), Vec3(1, 1.6, 0.3), Vec3(2, 1.5, 1)}, {0.1, 0.08, 0.06, 0.03}, 6));
    one("tube through vertical", MeshBuilder::tube({Vec3(0, 0, 0), Vec3(0.05, 1, 0), Vec3(0.1, 2, 0.05), Vec3(1, 2.5, 0)}, {0.1, 0.1, 0.1, 0.1}, 8));
    one("ribbon", MeshBuilder::ribbon({Vec3(0, 0, 0), Vec3(5, 0.2, 1), Vec3(9, 0.1, 4)}, {1.0, 1.2, 0.8}));
    for (int sub : {0, 1, 2}) one("icosphere " + std::to_string(sub), MeshBuilder::icosphere(sub));
    {
        WallSegment w;
        w.topA = Vec3(0, 3, 0); w.topB = Vec3(20, 3.5, 2); w.dropA = 2.5; w.dropB = 3.0; w.out = Vec3(0, 0, -1); w.bench = 2.0;
        WallSegment f = w;
        f.retaining = false; f.bench = 0;
        one("bakeWallMesh", bakeWallMesh({w, f}, Vec3(0.5, 0.5, 0.5)));
    }
    {
        WaterMeshParams wp;
        wp.lo = Vec2(-200, -200); wp.hi = Vec2(200, 200);
        one("buildWaterMesh", buildWaterMesh([](double x, double z) { return 0.02 * x - 2.0 + 0.01 * z; }, wp));
    }
    printAudit("ALL SHAPE KIT + SMALL PRODUCERS", total);
    CHECK(total.tris > 1000);
    CHECK(total.clean());
}

// THE MESH LIBRARY'S SOLIDS AND FILLS (mesh_builder.h): every one clean by the audit, every solid CLOSED (each edge
// in exactly two triangles), whatever way round its corners come -- a box on a sheared frame, a block given its
// corners clockwise, a prism with a hole and its outline either winding, a cylinder faceted and smooth, a polygon in
// a tilted plane with a hole.
TEST_CASE(mesh_library_solids_are_clean_and_closed) {
    auto check = [](const char* what, const RenderMesh& m, bool closed) {
        const Audit a = audit(m);
        const size_t open = openEdges(m);
        if (!a.clean() || (closed && open)) {
            printAudit(what, a);
            std::printf("    %s: %zu open edges\n", what, open);
        }
        CHECK(a.tris > 0);
        CHECK(a.clean());
        if (closed) CHECK(open == 0);
    };
    {
        RenderMesh m;
        MeshBuilder::emitBox(m, Vec3(-1, 0, -2), Vec3(1, 3, 2), Vec3(1, 1, 1));
        check("emitBox aabb", m, true);
    }
    {
        RenderMesh m;
        MeshFrame f;
        f.o = Vec3(2, 1, 0); f.x = Vec3(1.0, 0.0, 0.3); f.y = Vec3(0.1, 2.0, 0.0); f.z = Vec3(-0.2, 0.0, 1.5);
        MeshBuilder::emitBox(m, f, Vec3(1, 1, 1));
        check("emitBox sheared", m, true);
        RenderMesh mirrored;
        f.x = f.x * -1.0;   // a left-handed frame: the faces must still face out
        MeshBuilder::emitBox(mirrored, f, Vec3(1, 1, 1));
        check("emitBox left-handed", mirrored, true);
    }
    {
        RenderMesh m;
        const Vec3 foot[4] = {{0, 0, 0}, {0, 0, 2}, {3, 0, 2.5}, {3.2, 0, -0.4}};   // clockwise round
        const Vec3 head[4] = {{0.2, 2, 0.1}, {0.2, 2.2, 1.8}, {2.8, 1.6, 2.3}, {3.0, 1.5, -0.2}};
        MeshBuilder::emitBlock(m, foot, head, Vec3(1, 1, 1));
        check("emitBlock", m, true);
    }
    for (bool reverse : {false, true}) {
        std::vector<Vec2> outline = {{-5, -4}, {5, -4}, {5, 0}, {1, 0}, {1, 4}, {-5, 4}};
        std::vector<Vec2> hole = {{-3, -2}, {-1, -2}, {-1, 2}, {-3, 2}};
        if (reverse) { std::reverse(outline.begin(), outline.end()); }
        else std::reverse(hole.begin(), hole.end());
        RenderMesh m;
        MeshBuilder::emitPlanPrism(m, outline, {hole}, 1.0, 1.3, Vec3(1, 1, 1));
        check(reverse ? "emitPlanPrism (cw outline)" : "emitPlanPrism (ccw outline)", m, true);
    }
    for (bool faceted : {true, false}) {
        RenderMesh m;
        MeshBuilder::emitCylinder(m, Vec3(1, 0, 1), 0.4, 2.0, 12, Vec3(1, 1, 1), faceted);
        check(faceted ? "emitCylinder faceted" : "emitCylinder smooth", m, !faceted ? false : true);
    }
    {
        RenderMesh m;
        const Vec3 n = normalize(Vec3(0.3, 1, -0.2));
        const Vec3 u = normalize(cross(Vec3(0, 0, 1), n)), v = cross(n, u);
        std::vector<Vec3> ring, hole;
        for (int i = 0; i < 7; ++i) {
            const double a = 6.283185307179586 * i / 7, r = i % 2 ? 2.0 : 3.0;
            ring.push_back(u * (r * std::cos(a)) + v * (r * std::sin(a)));
            hole.push_back(u * (0.5 * std::cos(-a)) + v * (0.5 * std::sin(-a)));
        }
        MeshBuilder::emitPolygon(m, ring, {hole}, n, Vec3(1, 1, 1));
        check("emitPolygon tilted, holed", m, false);
    }
    for (bool faceted : {true, false}) {
        LatheStyle st;
        st.faceted = faceted;
        st.centre = Vec3(3, 0, -2);
        check(faceted ? "lathe faceted" : "lathe smooth",
              MeshBuilder::lathe({{0.0, 0.0}, {0.3, 0.0}, {0.35, 0.5}, {0.2, 0.9}, {0.25, 1.0}, {0.0, 1.0}}, 14, st), false);
    }
}

// THE LIBRARY FROM LUA (Glenn: "lua should be able to access it"): mesh.block, box_frame, polygon, prism, column,
// lathe (both looks), flat_shade, autosmooth -- each checked by mesh.audit inside the script, as a script author would.
TEST_CASE(mesh_library_from_lua_is_clean) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    const char* code = R"LUA(
        local function ok(name, m, closed)
            local a = mesh.audit(m)
            assert(a.tris > 0, name .. ": empty")
            assert(a.inward == 0 and a.degenerate == 0, name .. ": " .. a.inward .. " inward, " .. a.degenerate .. " degenerate")
            if closed then assert(a.open_edges == 0, name .. ": " .. a.open_edges .. " open edges") end
        end
        ok("block", mesh.block({{0,0,0},{0,0,2},{3,0,2},{3,0,0}}, {{0,2,0},{0,2,2},{3,1.5,2},{3,1.5,0}}, {0.5,0.5,0.5}), true)
        ok("box_frame", mesh.box_frame{origin={1,0,1}, x={1,0,0.2}, y={0,2,0}, z={-0.1,0,1.5}}, true)
        ok("polygon", mesh.polygon({{0,0,0},{2,0,0},{2,0,2},{1,0,1},{0,0,2}}, {}, {0,1,0}), false)
        ok("prism", mesh.prism({{-5,-4},{5,-4},{5,4},{-5,4}}, {{{-1,-1},{1,-1},{1,1},{-1,1}}}, 0, 0.3), true)
        ok("column", mesh.column({0,0,0}, 0.3, 2, 10), true)
        ok("lathe faceted", mesh.lathe{profile={{0,0},{0.3,0},{0.35,0.5},{0,1}}, segments=12}, false)
        ok("lathe smooth", mesh.lathe{profile={{0,0},{0.3,0},{0.35,0.5},{0,1}}, segments=12, smooth=true}, false)
        ok("flat_shade", mesh.flat_shade(mesh.icosphere(1)), false)
        ok("autosmooth", mesh.autosmooth(mesh.box({1,1,1}), 40), false)
    )LUA";
    std::string err;
    const bool ran = vm.doString(code, &err);
    if (!ran) std::printf("    lua: %s\n", err.c_str());
    CHECK(ran);
}
