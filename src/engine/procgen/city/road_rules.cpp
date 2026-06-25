#include "road_rules.h"

namespace engine {

ClassRules DesignRules::forClass(RoadClass c) const {
    switch (c) {
        // class         minRadius  maxGrade  laneW  lanes  divided
        case RoadClass::Freeway:   return { 300.0, 0.05, 3.6, 6, true  };
        case RoadClass::Arterial:  return {  80.0, 0.08, 3.5, 4, false };
        case RoadClass::Collector: return {  40.0, 0.10, 3.4, 2, false };
        case RoadClass::Local:     return {  15.0, 0.12, 3.2, 2, false };
        case RoadClass::Ramp:      return {  25.0, 0.06, 4.2, 1, false };
    }
    return { 15.0, 0.12, 3.2, 2, false };       // unreachable; keeps the compiler happy
}

const DesignRules& defaultDesign() {
    static const DesignRules rules;             // member defaults above
    return rules;
}

}  // namespace engine
