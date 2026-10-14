// LADDERS (Glenn, 2026-10-13: "Can we make ladders as interactable? And then use it on the fire engine and on
// buildings like auxiliary access ways to roofs?"). The roof-access ladders are pure in the building records: which
// buildings carry one, where it stands, where it lets you off; and which end of a ladder a player can reach.
#include "test_framework.h"
#include "../src/engine/ladder.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/ladders.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/systems/ladder_system.h"
#include <cmath>
#include <cstdio>

using namespace engine;

// Low flat-roofed buildings carry a roof ladder (two in three); towers, pitched roofs and big boxes do not. Each stands
// on its building's REAR wall, off it on its brackets, from the base to over the roof; it lets you off on the roof,
// inside the plan; a tall one is caged.
TEST_CASE(low_flat_roofed_buildings_carry_a_roof_ladder_on_their_back_wall) {
    const char* recipes[] = {"commercial_block", "office_midrise", "brick_shop", "corner_shop", "industrial_office", "police",
                             "bank", "apartments", "glass_tower", "cottage", "villa", "supermarket"};
    const Real sizes[][2] = {{20, 14}, {30, 20}};
    int buildings = 0, ladders = 0, onRear = 0, exitInside = 0, overRoof = 0, caged = 0, tallOnes = 0, wrong = 0, overGlass = 0;
    for (const char* rn : recipes)
        for (uint32_t seed = 1; seed <= 6; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                const BuildingParams& p = rc.params;
                ++buildings;
                const Real roof = 2.0 + p.groundHeight + p.floors * p.floorHeight;   // (base 2.0)
                LadderSpec l;
                if (!roofLadderFor(plan, p, 2.0, roof, l)) continue;
                ++ladders;
                const bool eligible = p.roofStyle == BuildingParams::RoofStyle::Flat && p.floors >= 1 && p.floors <= 4 &&
                                      !p.curtainWall && !p.bigBox;
                if (!eligible) ++wrong;
                Poly2 ccw = plan;
                ensureCCW(ccw);
                const std::size_t re = rearEdgeOf(ccw, p);
                const Vec2 a = ccw[re], b = ccw[(re + 1) % ccw.size()];
                const Vec2 d = normalize(b - a), n(d.y, -d.x);
                const Real off = dot(Vec2(l.foot.x, l.foot.z) - a, n), along = dot(Vec2(l.foot.x, l.foot.z) - a, d);
                if (off > 0.1 && off < 0.4 && along > 0 && along < (b - a).length() && dot(l.out, n) > 0.99) ++onRear;
                // never across a window: no opening of the rear face, on any storey, within its cage's half-width
                const std::vector<StoreyPlan> sps = storeyPlans(ccw, p);
                bool glass = false;
                for (std::size_t k = 0; k < sps.size(); ++k)
                    for (const WallOpening& o : storeyOpenings(sps[k].plan, p, static_cast<int>(k), sps[k].y0, sps[k].h, entranceEdgeFor(ccw, p))) {
                        if (std::fabs(dot(o.a - a, n) + std::max(p.wallThickness, Real(0.55))) > 0.4) continue;
                        const Real t0 = std::min(dot(o.a - a, d), dot(o.b - a, d)), t1 = std::max(dot(o.a - a, d), dot(o.b - a, d));
                        if (along + 0.48 > t0 && along - 0.48 < t1) glass = true;
                    }
                overGlass += glass;
                if (pointInPolygon(plan, Vec2(l.exit.x, l.exit.z)) && std::fabs(l.exit.y - roof) < 1e-9) ++exitInside;
                if (l.topY > roof + 0.9 && std::fabs(l.foot.y - 2.0) < 1e-9) ++overRoof;
                if (roof - 2.0 > 4.5) { ++tallOnes; caged += l.cage; }
            }
    std::printf("    [roof ladders] %d buildings, %d ladders: %d on the rear wall, %d letting off on the roof, %d over the "
                "parapet, %d of %d tall ones caged, %d where none belongs, %d across a window\n",
                buildings, ladders, onRear, exitInside, overRoof, caged, tallOnes, wrong, overGlass);
    CHECK(ladders > 20 && ladders < buildings);
    CHECK(onRear == ladders && exitInside == ladders && overRoof == ladders);
    CHECK(caged == tallOnes);
    CHECK(wrong == 0);
    CHECK(overGlass == 0);
    // the mesh: rails, rungs, a cage
    LadderSpec l;
    l.foot = Vec3(0, 0, 0); l.out = Vec2(0, 1); l.width = 0.46; l.landY = 7.0; l.topY = 8.1; l.cage = true;
    const RenderMesh m = ladderMesh(l);
    CHECK(m.indices.size() / 3 > 200);
    Real top = -1e9;
    for (const Vertex& v : m.vertices) top = std::max(top, static_cast<Real>(v.position.y));
    CHECK(std::fabs(top - 8.1) < 0.1);
}

// Which end a player can use: standing in front of its foot (climb), standing on its landing by where it comes over
// (climb down); neither from the side of the wall, nor from a floor away.
TEST_CASE(a_ladder_is_reached_from_its_foot_or_its_top) {
    LadderSpec l;
    l.foot = Vec3(10, 2, 5); l.out = Vec2(0, 1); l.landY = 9; l.topY = 10; l.exit = Vec3(10, 9, 4);
    CHECK(ladderEndWithinReach(l, Vec3(10, 2, 5.6)) == 1);     // in front of the foot
    CHECK(ladderEndWithinReach(l, Vec3(10.3, 2.2, 5.9)) == 1);
    CHECK(ladderEndWithinReach(l, Vec3(10, 9, 4.3)) == 2);     // on the roof by the top
    CHECK(ladderEndWithinReach(l, Vec3(10, 9, 3.2)) == 2);     // ...where it let you off
    CHECK(ladderEndWithinReach(l, Vec3(10, 5.5, 5.6)) == 0);   // halfway up in the air
    CHECK(ladderEndWithinReach(l, Vec3(14, 2, 5.6)) == 0);     // along the wall
    CHECK(ladderEndWithinReach(l, Vec3(10, 9, 8.0)) == 0);     // far out on the roof
}
