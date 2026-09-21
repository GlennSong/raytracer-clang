#ifndef RAYTRACER_APPS_CITYSIM_BUS_STOP_PROPS_H
#define RAYTRACER_APPS_CITYSIM_BUS_STOP_PROPS_H

// VISIBLE BUS STOPS (Glenn, 2026-09-18: "are there bus stops? We should have
// stops with benches and signs for bus stops so that we know where the route
// is").
//
// There were none. BusNetwork derives its stops as NAV NODES — logical points
// with no geometry — which was a deliberate simplification when the routes were
// built ("stops are existing nav nodes, so no stop geometry has to be placed").
// The cost of it is that the whole network is invisible: a rider standing at a
// stop reads as somebody loitering on a pavement, and there is no way to see
// where a route runs.
//
// So: a pole, a route-coloured sign and a bench at every stop. The SIGN COLOUR
// is keyed to the route, because "so that we know where the route is" is a
// question about telling one route from another, not just about seeing a post.
//
// Takes its ground sampler as a CALLBACK, for the same reason Dispatch takes
// its cost function: this then depends on no terrain, no renderer and no level
// loader, and can be dropped into any host that can answer "how high is the
// ground here".

#include "city_bus.h"

#include "../../engine/asset_manager.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/procgen/city/road_mesh.h"   // RoadDeckField: where the asphalt really is
#include "../../engine/world.h"

#include <functional>
#include <vector>

namespace citysim {


// Build the furniture for every stop in `net`. Returns how many stops got it —
// a stop whose node carries no usable street link is skipped rather than having
// a pole dropped in a carriageway. Entities are appended to `out` when given,
// so a caller can tear them down on a rebuild.
// The route's colour, as painted on its stop signs, and its name -- so the
// HUD, the map and the street all call route 2 the same thing.
engine::Vec3 routeColour(int route);
const char* routeColourName(int route);

int buildBusStopProps(
    engine::World& world, engine::AssetManager& assets, const BusNetwork& net,
    const engine::NavGraph& nav,
    const std::function<engine::Real(engine::Real, engine::Real)>& groundAt,
    std::vector<engine::Entity>* out = nullptr,
    // Where each stop's furniture actually LANDED (the kerbside point, not the
    // nav node it was derived from). For framing and for anyone auditing that
    // a stop ended up on a pavement rather than in a lane.
    std::vector<engine::Vec3>* outPositions = nullptr,
    // THE ASPHALT THAT WAS DRAWN. The kerbside point below is derived from the chosen
    // street's own WIDTH, so a stop beside a narrow street at a junction with a wide one
    // lands in the wide one — measured: 14 of 68 stops inside a carriageway on the
    // lattice metro, 7 of 50 on the lane-built city, worst 9.8 m in. Given the deck, the
    // furniture is stepped further out along the same kerb normal until it is clear.
    // Null = width-only, as before.
    const engine::RoadDeckField* deck = nullptr);

}  // namespace citysim

#endif
