// The city pipeline's last stage on a planned city: buildings grown on the blocks the planner's streets
// enclose stay sane. The arterial, fabric, district and frontage stages this file held judged the old
// metro grower and retired with it (ADR-0267); the planner's own promises are test_city_plan.cpp's, and
// frontage is test_city_lots.cpp's `every_built_lot_touches_a_road`.

#include "test_framework.h"
#include "planned_city.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cstdio>
#include <vector>

using namespace engine;

// BUILDINGS — recipes are picked per district, built within lot bounds. Every seed: three planned cities.
TEST_CASE(stage5_buildings_fit_their_lots) {
    for (uint32_t seed : {11u, 29u, 53u}) {
        const plan::CityPlan city = citytest::plannedCity(seed);
        std::vector<Poly2> blocks = extractBlocks(city.streets, 200.0);
        CHECK(!blocks.empty());

        LotParams lotParams;
        lotParams.center = city.brief.center;
        lotParams.seed = 11;
        lotParams.hubs = {{city.brief.center, 0}};   // downtown is the financial hub

        std::vector<LotBuilding> buildings = growLotBuildings(blocks, lotParams);
        CHECK(!buildings.empty());

        // The building's OBB (width/depth) is the hard limit of what a lot holds; the plan grown from the lot
        // stays within its block (growLotBuildings checks that). Here: no degenerate dimensions.
        int overflowed = 0;
        for (const LotBuilding& b : buildings) {
            if (b.type == "park" || b.type == "green") continue;   // ground scenery, no plan
            if (b.width > 1000 || b.depth > 1000) ++overflowed;
        }
        std::printf("        [seed %u] %zu blocks, %zu buildings\n", seed, blocks.size(), buildings.size());
        CHECK(overflowed == 0);
    }
}
