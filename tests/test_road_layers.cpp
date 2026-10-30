#include "test_framework.h"

#include "../src/engine/procgen/city/road_rules.h"
#include <cmath>

using namespace engine;

// --- the design-rules registry (ADR-0052): one class-keyed source for road parameters ---
TEST_CASE(design_rules_rank_classes_sensibly) {
    const DesignRules& d = defaultDesign();
    CHECK(d.minRadius(RoadClass::Freeway) > d.minRadius(RoadClass::Local));   // freeway curves gently
    CHECK(d.maxGrade(RoadClass::Freeway) < d.maxGrade(RoadClass::Local));     // and climbs gently
    CHECK(d.forClass(RoadClass::Freeway).dividedMedian);                      // freeway is divided
    CHECK(!d.forClass(RoadClass::Local).dividedMedian);
    CHECK(d.forClass(RoadClass::Ramp).lanes == 1);                            // a ramp is one lane
    CHECK(d.forClass(RoadClass::Freeway).fullWidth() > d.forClass(RoadClass::Local).fullWidth());
    CHECK(d.clearance > 4.0 && d.rampGrade > 0.0);                            // grade-sep numbers present
}
