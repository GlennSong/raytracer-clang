// THE MESH MASS (mesh_mass.h, issue #105 step 6): a closed mesh sliced into storeys, each slice resampled corner
// for corner; the artist's hatch through the grammar.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/mesh_mass.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
// A vase: a lathe of a profile r(y) = 10 + 4 sin(pi y / H) over H = 60 m, 24 around, closed top and bottom.
RenderMesh vase(Real H, int around, int rings) {
    RenderMesh m;
    std::vector<std::vector<Vec3>> ringPts(static_cast<std::size_t>(rings + 1));
    for (int j = 0; j <= rings; ++j) {
        const Real y = H * j / rings;
        const Real r = 10.0 + 4.0 * std::sin(3.14159265358979323846 * y / H);
        for (int i = 0; i < around; ++i) {
            const Real a = 2.0 * 3.14159265358979323846 * i / around;
            ringPts[static_cast<std::size_t>(j)].push_back(Vec3(std::cos(a) * r, y, std::sin(a) * r));
        }
    }
    for (int j = 0; j < rings; ++j)
        for (int i = 0; i < around; ++i) {
            const Vec3 a = ringPts[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)], b = ringPts[static_cast<std::size_t>(j)][static_cast<std::size_t>((i + 1) % around)];
            const Vec3 c = ringPts[static_cast<std::size_t>(j + 1)][static_cast<std::size_t>((i + 1) % around)], d = ringPts[static_cast<std::size_t>(j + 1)][static_cast<std::size_t>(i)];
            MeshBuilder::emitQuad(m, a, b, c, d, normalize(cross(b - a, d - a)), Vec3(0.5, 0.5, 0.5));
        }
    for (int i = 0; i < around; ++i) {   // the caps
        MeshBuilder::emitTri(m, Vec3(0, 0, 0), ringPts[0][static_cast<std::size_t>((i + 1) % around)], ringPts[0][static_cast<std::size_t>(i)], Vec3(0, -1, 0), Vec3(0.5, 0.5, 0.5));
        MeshBuilder::emitTri(m, Vec3(0, H, 0), ringPts[static_cast<std::size_t>(rings)][static_cast<std::size_t>(i)], ringPts[static_cast<std::size_t>(rings)][static_cast<std::size_t>((i + 1) % around)], Vec3(0, 1, 0), Vec3(0.5, 0.5, 0.5));
    }
    return m;
}
}  // namespace

TEST_CASE(a_mesh_slices_to_its_section_resampled_corner_for_corner) {
    const RenderMesh m = vase(60.0, 24, 30);
    const Poly2 s10 = sliceMesh(m, 10.0, 0.2, 32);
    const Poly2 s30 = sliceMesh(m, 30.0, 0.2, 32);
    CHECK(s10.size() == 32 && s30.size() == 32);
    CHECK(signedArea(s10) > 0 && signedArea(s30) > 0);
    // the radius the profile gives at each height, within the chords' tolerance
    const Real r10 = 10.0 + 4.0 * std::sin(3.14159265358979323846 * 10.0 / 60.0), r30 = 14.0;
    for (const Vec2& q : s10) CHECK(std::fabs(q.length() - r10) < 0.35);
    for (const Vec2& q : s30) CHECK(std::fabs(q.length() - r30) < 0.35);
    // corner for corner: the first corner EXACTLY on the +x bearing on both, the k-th corners at the same bearing
    CHECK(std::fabs(s10[0].y) < 1e-6 && s10[0].x > 0 && std::fabs(s30[0].y) < 1e-6 && s30[0].x > 0);
    for (std::size_t k = 0; k < 32; ++k) {
        const Real a0 = std::atan2(s10[k].y, s10[k].x), a1 = std::atan2(s30[k].y, s30[k].x);
        CHECK(std::fabs(std::remainder(a0 - a1, 2.0 * 3.14159265358979323846)) < 0.15);
    }
    CHECK(sliceMesh(m, 70.0).empty());   // above the mesh: nothing
    std::printf("    [mesh] slices at 10 and 30 m: %zu corners each, r %.2f and %.2f\n", s10.size(), s10[0].length(), s30[0].length());
}

TEST_CASE(a_mesh_mass_runs_through_the_grammar_fitted_to_its_lot) {
    const Poly2 plan = {{-15, -15}, {15, -15}, {15, 15}, {-15, 15}};
    BuildingParams p = towerParams(16);
    p.baseFloors = 1;
    MeshMass mm;
    mm.base = plan;
    mm.baseStoreys = 1;
    mm.groundH = p.groundHeight;
    mm.floorH = p.floorHeight;
    mm.floors = p.floors;
    mm.mesh = vase(60.0, 24, 30);
    fitMeshToPlan(mm.mesh, plan, mm.height());   // the vase's 28 m waist onto the 30 m lot, its 60 m onto the height
    const std::vector<StoreyPlan> storeys = sliceStoreys(mm, p);
    CHECK(storeys.size() == 17);
    CHECK(storeys[0].plan.size() == 4 && storeys[1].plan.size() == 4);
    Real maxR = 0;
    for (int k = 2; k <= 16; ++k) {
        const Poly2& sp = storeys[static_cast<std::size_t>(k)].plan;
        CHECK(sp.size() == 32);
        CHECK(storeys[static_cast<std::size_t>(k)].tier == 1);
        Vec2 lo, hi;
        bounds(sp, lo, hi);
        CHECK(lo.x >= -15.1 && hi.x <= 15.1 && lo.y >= -15.1 && hi.y <= 15.1);   // on the lot
        maxR = std::max(maxR, (hi.x - lo.x) * 0.5);
    }
    CHECK(maxR > 14.0);   // the waist fills the lot
    const MassBounds b = mm.bounds();
    CHECK(b.y1 == mm.height());
    const RenderMesh shell = mm.shell(0, Vec3(0.5, 0.5, 0.5));
    CHECK(shell.vertices.size() > 32 * 4 * 10);
    std::printf("    [mesh] %zu storeys, widest half %.2f m, loft %zu vertices\n", storeys.size(), maxR, shell.vertices.size());
}

TEST_CASE(a_mesh_too_narrow_for_its_core_is_widened_until_every_storey_holds_it) {
    // A vase whose neck would be 6 m across on a 30 m lot: the core (kCoreTierMin + 3 = 17 m) could not stand in
    // it, so the footprint is pushed toward the lot's box until the narrowest storey is wide enough; the belly,
    // already on the box, stays where it is.
    const Poly2 plan = {{-15, -15}, {15, -15}, {15, 15}, {-15, 15}};
    BuildingParams p = towerParams(16);
    MeshMass mm;
    mm.base = plan;
    mm.baseStoreys = 1;
    mm.groundH = p.groundHeight;
    mm.floorH = p.floorHeight;
    mm.floors = p.floors;
    RenderMesh m = vase(60.0, 24, 30);
    for (Vertex& v : m.vertices) {   // pinch the neck: radius 3 at the top, the 14 m belly kept
        const Real t = v.position.y / 60.0;
        const Real r0 = 10.0 + 4.0 * std::sin(3.14159265358979323846 * t);
        const Real r1 = t < 0.5 ? r0 : 14.0 + (3.0 - 14.0) * (t - 0.5) * 2.0;
        const Real k = r0 > 1e-9 ? r1 / r0 : 1;
        v.position.x *= k;
        v.position.z *= k;
    }
    fitMeshToPlan(m, plan, mm.height());
    auto narrowest = [&](const RenderMesh& mesh) {
        Real n = 1e9;
        for (int k = 2; k <= 16; ++k) n = std::min(n, tierShortSide(sliceMesh(mesh, mm.groundH + (k - 1) * mm.floorH + 0.5 * mm.floorH, mm.tol, mm.corners)));
        return n;
    };
    const Real before = narrowest(m);
    CHECK(before < 10.0);
    widenMeshForCore(m, plan, mm);
    const Real after = narrowest(m);
    CHECK(after >= kCoreTierMin + 3.0 - 0.1);
    Real widest = 0;
    for (int k = 2; k <= 16; ++k) {
        const Poly2 s = sliceMesh(m, mm.groundH + (k - 1) * mm.floorH + 0.5 * mm.floorH, mm.tol, mm.corners);
        Vec2 lo, hi;
        bounds(s, lo, hi);
        CHECK(lo.x >= -15.1 && hi.x <= 15.1 && lo.y >= -15.1 && hi.y <= 15.1);   // still on the lot
        widest = std::max(widest, hi.x - lo.x);
    }
    CHECK(widest > 29.0);   // the belly still fills the lot
    // and through the grammar: the storeys, the core, every cell reached
    mm.mesh = m;
    const std::vector<StoreyPlan> storeys = sliceStoreys(mm, p);
    CHECK(storeys.size() == 17);
    std::printf("    [mesh] narrowest storey %.1f m -> %.1f m (needs %.1f), widest %.1f m\n", before, after, kCoreTierMin + 3.0, widest);
}

TEST_CASE(a_mass_on_a_lot_too_tight_for_its_core_stands_as_the_plan) {
    // The library's first mass on an L whose arms are 12 and 14 m wide: no box inside the plan holds the core
    // (kCoreTierMin + 3 m), so the storeys are the plan -- the twist's rule, the gate's L lot.
    CHECK(massLibrary().size() >= 1);   // assets/masses/sculpted_01.glb (tools/make_mass.py)
    const Poly2 L = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    BuildingParams p = towerParams(20);
    p.envelope = BuildingParams::Envelope::Mesh;
    p.massMesh = 1;
    p.baseFloors = 1;
    const MeshMass onL = meshOf(L, p);
    CHECK(onL.mesh.indices.empty());
    for (const StoreyPlan& sp : storeyPlans(L, p)) CHECK(sp.plan.size() == 6);
    // and on a 24 x 20 rectangle the same mass is carried: every storey above the base its own slice
    const Poly2 rect = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const MeshMass onRect = meshOf(rect, p);
    CHECK(!onRect.mesh.indices.empty());
    const std::vector<StoreyPlan> storeys = storeyPlans(rect, p);
    CHECK(storeys.size() == 21);
    int sliced = 0;
    for (std::size_t k = 2; k < storeys.size(); ++k) if (storeys[k].plan.size() == onRect.corners) ++sliced;
    CHECK(sliced == 19);
    CHECK(unreachableCells(spaceGraphOf(rect, p, 0.0)).empty());
    std::printf("    [mesh] on the L: the plan; on 24 x 20: %d sliced storeys of %zu\n", sliced, storeys.size() - 2);
}

TEST_CASE(the_mesh_envelope_without_a_library_entry_is_the_plan) {
    // Envelope::Mesh with no file behind it: the storeys are the plan, the building stands as drawn.
    const Poly2 plan = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    BuildingParams p = towerParams(8);
    p.envelope = BuildingParams::Envelope::Mesh;
    p.massMesh = 200;   // past any library
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 9);
    for (const StoreyPlan& sp : storeys) CHECK(sp.plan.size() == 4);
    CHECK(massStack(plan, p).size() == 1);
    CHECK(unreachableCells(spaceGraphOf(plan, p, 0.0)).empty());
    std::printf("    [mesh] library holds %zu masses\n", massLibrary().size());
}
