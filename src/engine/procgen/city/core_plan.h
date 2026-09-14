#ifndef ENGINE_PROCGEN_CITY_CORE_PLAN_H
#define ENGINE_PROCGEN_CITY_CORE_PLAN_H

// THE CORE (skyscrapers v2 M5, ADR-0086 point 12): a tall building's vertical
// circulation — a bank of elevator hoistways flanked by two enclosed dog-leg
// stairwells — as pure geometry from the plan, the mass stack and the params,
// so the exterior grow (the ground ceiling's shaft holes), the streamed
// interior (walls, flights, landings, colliders) and the runtime (the
// elevator cab, the hoistway doors) all derive the SAME core by construction.
//
// The core is a rectangle seated at the centre of the TOP tier (the smallest
// plan, which every tier below contains), its door wall facing the entrance:
//
//        v (into the core, away from the lobby)
//        ^  +-------+----+----+----+-------+
//        |  | stair | ho | is | tw | stair |   hoistways 2.4 x 2.6
//        |  |  A    | ay |  s |    |  B    |   stairs   2.6 x (1.5 + run + 1.5)
//        |  |       |----service----|       |   service  the closed block behind
//        |  +--=----+-=--+-=--+-=--+----=--+
//        +----------------------------------> u (along the bank)
//                 = doors, all in the v = 0 wall, facing the lobby
//
// Every shaft is a hole in every slab. A stair is two flights of 1.2 m
// either side of a 0.2 m spine wall: flight A climbs away from the door to a
// half landing at the far end, flight B climbs back to the next floor's
// landing at the door (riser <= 0.20, tread 0.26). The ground storey's
// taller flight sizes the shaft; upper storeys get a longer half landing.
// Walls are 0.15 m thick (two skins; the skin between two shafts is drawn
// once). The service block has no door.

#include "polygon.h"
#include "site_plan.h"       // SiteFrame
#include "shape_grammar.h"   // MassTier, StoreyPlan, BuildingParams, RenderMesh
#include <cstddef>
#include <vector>

namespace engine {

// One shaft of the core. `frame.origin` is the shaft's door-wall left corner;
// u runs along the door wall, v into the shaft. The door (when doorWidth > 0)
// sits in the v = 0 wall, centred at `doorX`, on every storey.
struct CoreShaft {
    SiteFrame frame;
    Real width = 0;        // along u
    Real depth = 0;        // along v
    Real doorX = 0;
    Real doorWidth = 0;    // 0 = no door (the service block)
    Real doorHeight = 2.1;
    Poly2 rect() const;                                        // world XZ, CCW
    Vec2 doorFoot() const { return frame.toWorld({doorX, 0}); }
    Vec2 doorNormal() const { return frame.v * -1.0; }         // out of the shaft
    Vec3 at(Real u, Real v, Real y) const {
        const Vec2 w = frame.toWorld({u, v});
        return {w.x, y, w.y};
    }
};

struct CoreStair {
    CoreShaft shaft;
    Real flightWidth = 1.2;
    Real spine = 0.2;      // the wall between the two flights
    Real landing = 1.5;    // the floor landing's depth (v in [0, landing])
    Real tread = 0.26;
};

struct CorePlan {
    bool valid = false;
    SiteFrame frame;                 // u along the bank, v away from the lobby
    Real length = 0, depth = 0;      // along u / v
    std::vector<CoreShaft> hoistways;
    std::vector<CoreStair> stairs;   // two, at the bank's ends
    CoreShaft service;               // the closed block behind the hoistways
    bool hasService = false;
    Poly2 rect() const;              // the whole core, world XZ, CCW
};

// Hoistways by height: 1 up to 8 floors, 2 to 20, 3 to 40, 4 beyond.
int hoistwaysFor(int floors);
// Risers in one half flight (riser <= 0.20) and its horizontal run.
int halfFlightRisers(Real storeyHeight);
Real halfFlightRun(Real storeyHeight, Real tread = 0.26);
// The core policy: `params.core` 0 = auto (six floors and up), 1 = never,
// 2 = always (a Lua override).
bool wantsCore(const BuildingParams& params);

// The core for a plan and its tiers (massStack), the door wall facing the
// entrance edge. `valid` is false when no bank — down to one hoistway, in
// either orientation — fits inside every tier with a 1.8 m corridor around
// it; the building then keeps the straight stair of ADR-0080 (or no stair).
CorePlan corePlan(const Poly2& plan, const std::vector<MassTier>& tiers,
                  const BuildingParams& params, std::size_t entranceEdge);
// wantsCore ? corePlan(plan, massStack(plan, params), ...) : invalid — the
// one call the grammar, the lot pass and the runtime share.
CorePlan coreFor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge);

// The holes every slab (and the ground ceiling) carries: hoistways + stairs.
std::vector<Poly2> coreSlabHoles(const CorePlan& core);

// One storey's share of the core: shaft walls with their doors, the two
// flights, the half landing and (for storeys above ground) the floor landing,
// the spine wall and the soffits. `drywall` takes the painted walls (the
// Interior part), `floor` the landings (the floor finish), `stair` the treads,
// risers and soffits (the stair finish). `colliderOut` receives everything a
// walker touches. `flightsUp` is false on the top storey (nowhere to climb);
// `landing` is false on the ground storey (the ground slab is already there).
struct CoreMeshes {
    RenderMesh drywall;
    RenderMesh floor;
    RenderMesh stair;
};
void emitCoreStorey(CoreMeshes& out, RenderMesh* colliderOut, const CorePlan& core,
                    const StoreyPlan& sp, Real baseY, const BuildingParams& params,
                    bool flightsUp, bool landing);

}  // namespace engine

#endif
