// THE SECTION AND THE FACE (architectural toolkit, issue #105, foundation B, step 2a): lines only, and the proof
// that nothing downstream can tell -- a lot's section tessellates to the lot corner for corner, the n-gon is the
// drum's circle, the offset is the grammar's offsetPlan, a flat face is the plan edge's rectangle, and a building
// grown from a section hashes to the building grown from its plan.
#include "test_framework.h"
#include "../src/engine/procgen/city/face.h"
#include "../src/engine/procgen/city/section.h"
#include "../src/engine/procgen/city/shape_grammar.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace engine;

namespace {

bool samePoly(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].x != b[i].x || a[i].y != b[i].y) return false;   // bit for bit
    return true;
}

uint64_t hashOf(const BuildingMesh& bm) {
    uint64_t h = 1469598103934665603ull;
    for (const RenderMesh& part : bm.parts)
        for (const Vertex& vx : part.vertices) {
            const double q[3] = {vx.position.x, vx.position.y, vx.position.z};
            for (double d : q) {
                h ^= static_cast<uint64_t>(static_cast<int64_t>(std::llround(d * 4096.0)));
                h *= 1099511628211ull;
            }
        }
    return h;
}

}  // namespace

TEST_CASE(section_from_lot_tessellates_to_the_lot) {
    const Poly2 rect = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    for (const Poly2& p : {rect, lPlan}) {
        const Section s = Section::fromLot(p);
        CHECK(s.outer.size() == p.size());
        CHECK(samePoly(s.tessellate(), p));
        for (std::size_t i = 0; i < s.outer.size(); ++i) {
            CHECK(s.outer[i].kind == Edge::Kind::Line);
            CHECK(s.outer[i].b.x == s.outer[(i + 1) % s.outer.size()].a.x && s.outer[i].b.y == s.outer[(i + 1) % s.outer.size()].a.y);
        }
    }
    // A clockwise lot comes out CCW, as every grower insists.
    Poly2 cw = rect;
    std::reverse(cw.begin(), cw.end());
    CHECK(isCCW(Section::fromLot(cw).tessellate()));
}

TEST_CASE(section_rect_and_ngon_are_the_grammars_own) {
    CHECK(samePoly(Section::rect(24, 20).tessellate(), Poly2{{-12, -10}, {12, -10}, {12, 10}, {-12, 10}}));
    // The drum's circle as city_lots draws it: sixteen corners from angle 0, counter-clockwise.
    const Vec2 cc(3.5, -2.0);
    const Real rad = 11.0;
    Poly2 circ;
    for (int k = 0; k < 16; ++k) {
        const Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
        circ.push_back(cc + Vec2(std::cos(a2), std::sin(a2)) * rad);
    }
    CHECK(samePoly(Section::ngon(16, rad, cc).tessellate(), circ));
}

TEST_CASE(section_offset_is_the_grammars_offset_plan) {
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    for (const Real d : {Real(0.55), Real(2.0), Real(-0.75)})
        CHECK(samePoly(Section::fromLot(lPlan).offset(d).tessellate(), offsetPlanPublic(lPlan, d)));
    const Section holed = Section::fromLot(lPlan).withHole(Section::rect(4, 4));
    CHECK(holed.holes.size() == 1);
    CHECK(holed.holesTessellated().front().size() == 4);
    CHECK(holed.offset(1.0).holes.size() == 1);
}

TEST_CASE(flat_face_is_the_plan_edges_rectangle) {
    const Poly2 plan = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Face f = Face::ofEdge(plan, 1, 4.5, 3.2);   // the +x edge, (12,-10) -> (12,10)
    CHECK(f.kind == Face::Kind::Flat);
    CHECK_APPROX(f.width, 20.0, 1e-12);
    CHECK_APPROX(f.height, 3.2, 1e-12);
    const Vec3 bl = f.at(0, 0), tr = f.at(f.width, f.height);
    CHECK_APPROX(bl.x, 12.0, 1e-12); CHECK_APPROX(bl.y, 4.5, 1e-12); CHECK_APPROX(bl.z, -10.0, 1e-12);
    CHECK_APPROX(tr.x, 12.0, 1e-12); CHECK_APPROX(tr.y, 7.7, 1e-12); CHECK_APPROX(tr.z, 10.0, 1e-12);
    CHECK_APPROX(f.normal(1, 1).x, 1.0, 1e-12);   // outward: to the right of a->b on a CCW plan
    CHECK_APPROX(f.normal(1, 1).z, 0.0, 1e-12);
}

TEST_CASE(a_section_grows_what_its_plan_grows) {
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    BuildingParams p;
    p.floors = 9;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    p.setbackFloors = 4;
    p.setbackEvery = 1.5;
    const uint64_t fromPlan = hashOf(growPlanBuilding(lPlan, p, 1.5));
    const uint64_t fromSection = hashOf(growSectionBuilding(Section::fromLot(lPlan), p, 1.5));
    CHECK(fromPlan == fromSection);
    CHECK(hashOf(growSectionBuilding(Section::rect(24, 20), p, 1.5)) == hashOf(growPlanBuilding(Section::rect(24, 20).tessellate(), p, 1.5)));
}
