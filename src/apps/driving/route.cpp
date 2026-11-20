#include "apps/driving/route.h"

#include <string>
#include <vector>

namespace driving {

RouteClass bindRouteClass(const engine::LaneGraph& roads, const VehicleClass& cls, std::vector<std::string>* notes) {
    RouteClass rc;
    rc.tightestTurnRadius = cls.tightestTurnRadius;
    rc.topSpeed = cls.topSpeed;
    for (const std::string& tag : cls.tags) {
        const engine::NameIndex bit = engine::LaneGraph::nameIndex(roads.vehicleTagNames, tag);
        if (bit == engine::kNoName || bit >= engine::kMaxTagNames) {
            if (notes)
                notes->push_back("vehicle class '" + cls.name + "': the road knows no vehicle tag '" + tag +
                                 "', so the class matches no path by it");
            continue;
        }
        rc.tags |= engine::TagMask(1) << bit;
    }
    return rc;
}

}  // namespace driving
