#ifndef RAYTRACER_APPS_DRIVING_ROUTE_H
#define RAYTRACER_APPS_DRIVING_ROUTE_H

// THE DRIVING'S SIDE OF ROUTING. The router is the engine's one route-finder (engine/procgen/city/roads/route.h,
// ADR-0467), which routes cars and walkers over the road model's one lane level; the driving calls it from there under
// the names below. What stays here is what only the driving knows: its vehicle classes, bound to the road as the router
// sees them (bindRouteClass). The ways on closed to a class are the driving's too (room.h chainsClosedForRoom, Driving's
// closed turns); it fills them into the RouteClass it passes in.

#include "engine/procgen/city/roads/route.h"
#include "apps/driving/notice.h"          // WayAhead
#include "apps/driving/traffic_state.h"   // PathPoint
#include "apps/driving/vehicle_class.h"

#include <string>
#include <vector>

namespace driving {

using engine::roads::kLaneChangeCostSeconds;
using engine::roads::kCannotFinish;
using engine::roads::kChainClosed;
using engine::roads::ClosedChains;
using engine::roads::buildClosedChains;
using engine::roads::RouteClass;
using engine::roads::mayUse;
using engine::roads::freeFlowSpeed;
using engine::roads::RouteBounds;
using engine::roads::studyRoads;
using engine::roads::SectionSeconds;
using engine::roads::NoRouteWhy;
using engine::roads::kNoRouteWhyCount;
using engine::roads::RouteLeg;
using engine::roads::RouteTurn;
using engine::roads::RouteLane;
using engine::roads::Route;
using engine::roads::findRoute;
using engine::roads::RoutePart;
using engine::roads::kRoutePartCount;
using engine::roads::RoutePlace;
using engine::roads::placeOnRoute;
using engine::roads::Across;
using engine::roads::kAcrossCount;
using engine::roads::LanesToCross;
using engine::roads::lanesToCross;
using engine::roads::LastChance;
using engine::roads::lastChance;
using engine::roads::wayAhead;
using engine::roads::Remaining;
using engine::roads::remainingFrom;
using engine::roads::routeHash;

// BINDS A CLASS TO A GRAPH: each of its tag names to the graph's bit for it. A tag the graph does not know is not an
// error -- the class simply matches no path by that tag -- and is reported as a plain line in `notes` when given.
RouteClass bindRouteClass(const engine::LaneGraph& roads, const VehicleClass& cls, std::vector<std::string>* notes = nullptr);

}  // namespace driving

#endif
