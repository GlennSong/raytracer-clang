#include "road_rules.h"

#include <algorithm>

namespace engine {

ClassRules DesignRules::forClass(RoadClass c) const {
    switch (c) {
        // class         minRadius  maxGrade  laneW  lanes  divided  sidewalk
        case RoadClass::Freeway:   return { 300.0, 0.05, 3.6, 6, true,  0.0 };
        case RoadClass::Arterial:  return {  80.0, 0.08, 3.5, 4, false, 4.5 };
        case RoadClass::Collector: return {  40.0, 0.10, 3.4, 2, false, 3.5 };
        case RoadClass::Local:     return {  15.0, 0.12, 3.2, 2, false, 3.0 };
        case RoadClass::Ramp:      return {  25.0, 0.06, 4.2, 1, false, 0.0 };
    }
    return { 15.0, 0.12, 3.2, 2, false, 3.0 };  // unreachable; keeps the compiler happy
}

CrossSection DesignRules::crossSectionFor(RoadClass c) const {
    const ClassRules r = forClass(c);
    const double lw = r.laneWidth;
    CrossSection xs;

    // A one-way ramp: outer(left) shoulder + one lane + wider outer(right)
    // shoulder (AASHTO asymmetric ramp section).
    if (c == RoadClass::Ramp) {
        xs.push_back({LaneType::Shoulder, LaneDir::None, 1.2});
        xs.push_back({LaneType::Driving,  LaneDir::Fwd,  lw});
        xs.push_back({LaneType::Shoulder, LaneDir::None, 2.4});
        return xs;
    }

    const bool freeway = (c == RoadClass::Freeway);
    const int  total   = std::max(1, r.lanes);
    const int  left    = total / 2;             // oncoming (Back) side
    const int  right   = total - left;          // with-travel (Fwd) side

    // Left edge: sidewalk (surface) or paved shoulder (freeway).
    if (r.sidewalk > 0.0) xs.push_back({LaneType::Sidewalk, LaneDir::None, r.sidewalk});
    else if (freeway)     xs.push_back({LaneType::Shoulder, LaneDir::None, 3.0});

    for (int i = 0; i < left; ++i) xs.push_back({LaneType::Driving, LaneDir::Back, lw});
    if (r.dividedMedian)
        xs.push_back({LaneType::Median, LaneDir::None, freeway ? 1.4 : 3.35});
    for (int i = 0; i < right; ++i) xs.push_back({LaneType::Driving, LaneDir::Fwd, lw});

    if (r.sidewalk > 0.0) xs.push_back({LaneType::Sidewalk, LaneDir::None, r.sidewalk});
    else if (freeway)     xs.push_back({LaneType::Shoulder, LaneDir::None, 3.0});
    return xs;
}

double carriagewayWidth(const CrossSection& xs) {
    double w = 0.0;
    for (const LaneSpec& l : xs)
        if (l.type != LaneType::Sidewalk) w += l.width;
    return w;
}

double totalWidth(const CrossSection& xs) {
    double w = 0.0;
    for (const LaneSpec& l : xs) w += l.width;
    return w;
}

int drivingLaneCount(const CrossSection& xs) {
    int n = 0;
    for (const LaneSpec& l : xs)
        if (l.type == LaneType::Driving) ++n;
    return n;
}

const DesignRules& defaultDesign() {
    static const DesignRules rules;             // member defaults above
    return rules;
}

}  // namespace engine
