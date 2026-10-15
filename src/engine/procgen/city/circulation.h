#ifndef ENGINE_PROCGEN_CITY_CIRCULATION_H
#define ENGINE_PROCGEN_CITY_CIRCULATION_H

// CIRCULATION (architectural toolkit, issue #105, step 3; ADR-0174): how a building is climbed, as words of its
// own. A SHAFT is one vertical element -- a hoistway, a stairwell, a closed service block, an escalator's slot, a
// ramp -- standing in its own frame with a door in its front wall on the storeys it serves. A BANK is the hoistways
// that answer one set of call buttons, facing one lobby. A ZONE is the run of storeys a bank serves: a low-rise
// bank stops at every storey up to the zone's top, a high-rise bank runs express from the lobby to its zone, and
// where zones meet, a SKY LOBBY is the transfer floor. The plan of all of them is a CirculationPlan.
//
// Today's core (core_plan.h: two dog-leg stairwells either side of a bank of hoistways with a service block
// behind, seated in the centre of the top tier facing the entrance) is the first PRESET of this plan: corePreset
// lays it out, and corePlan reads the preset back into the CorePlan the emitters, the elevator and the streamed
// interior take, so every cored building today is byte-identical to what it was before this header existed. The
// shafts become cells of the SpaceGraph (addTo), the doors become portals: that is where the gate reads them.

#include "polygon.h"
#include "site_plan.h"       // SiteFrame
#include "shape_grammar.h"   // MassTier, StoreyPlan, BuildingParams
#include <cstddef>
#include <cstdint>
#include <vector>

namespace engine {

struct SpaceGraph;
struct CorePlan;

// The core's dimensions (core_plan.h's drawing): what the preset lays out and the emitters draw.
constexpr Real kShaftWall = 0.15;     // shaft wall thickness
constexpr Real kHoistwayW = 2.4;      // a hoistway along the bank
constexpr Real kHoistwayD = 2.6;      // a hoistway into the core
constexpr Real kStairShaftW = 2.6;    // two 1.2 m flights + the 0.2 m spine
constexpr Real kStairLanding = 1.5;   // the floor landing's depth
constexpr Real kStairTread = 0.26;
constexpr Real kCoreCorridor = 2.2;   // the ring the core needs inside every tier: 1.5 m clear past the inner skin

// One vertical element. `frame.origin` is the front (door) wall's left corner; u runs along the front wall, v into
// the shaft. The door, when `doorW` > 0, sits in the v = 0 wall centred at `doorU` on every storey the shaft serves.
struct Shaft {
    enum class Kind : uint8_t { Hoistway, Stair, Service, Scenic, Escalator, Ramp, Vomitory };
    Kind kind = Kind::Hoistway;
    SiteFrame frame;
    Real w = 0, d = 0;             // along u / v
    int k0 = 0, k1 = 0;            // the storeys it runs through, inclusive
    Real doorU = 0, doorW = 0;     // doorW 0 = closed (the service block)
    Real doorH = 2.1;
    int bank = -1;                 // a hoistway's bank
    int cell = -1;                 // the SpaceGraph cell addTo made for it
    Poly2 rect() const;            // world XZ, CCW
    Vec2 doorFoot() const { return frame.toWorld({doorU, 0}); }
    Vec2 doorNormal() const { return frame.v * -1.0; }   // out of the shaft
};

// The hoistways that answer one set of buttons. `lobbyEdge` is the plan edge their doors face on the ground storey.
struct Bank {
    std::vector<int> shafts;
    std::size_t lobbyEdge = 0;
};

// The storeys bank `bank` serves, inclusive. A bank with doors on every storey of the building is one zone.
struct Zone {
    int bank = 0;
    int k0 = 0, k1 = 0;
};

struct CirculationPlan {
    bool valid = false;
    // The block the shafts stand in when they stand together (the core): u along the bank, v away from the lobby.
    SiteFrame frame;
    Real length = 0, depth = 0;
    std::vector<Shaft> shafts;
    std::vector<Bank> banks;
    std::vector<Zone> zones;
    int skyLobby = -1;             // the transfer storey between two zones, -1 for none
    Poly2 rect() const;            // the block, world XZ, CCW
    bool serves(const Shaft& s, int k) const;   // a door on storey k: the shaft runs through it and its bank's zone holds it
    const Shaft* stairs(std::size_t i) const;   // the i-th stair shaft, nullptr past the last
    // THE SHAFTS AS CELLS: one cell per stair and hoistway up the building, a Stair / Lift portal from every hall
    // it has a door on; the stairs on into the roof cell when `roofExit` (the bulkhead). `hallOf` is the hall cell
    // per storey; the roof cell is `g.roof`. Sets every shaft's `cell`.
    void addTo(SpaceGraph& g, const std::vector<StoreyPlan>& storeys, Real baseY, const std::vector<int>& hallOf, bool roofExit);
};

// THE CORE PRESET: two stairwells either side of hoistwaysFor(floors) hoistways (fewer when the plan is tight),
// the service block behind, the whole seated at the centre of the top tier with its doors facing the entrance
// edge, inside every tier with a corridor ring around it. `valid` false when not even one hoistway fits. One bank,
// one zone over every storey.
CirculationPlan corePreset(const Poly2& plan, const std::vector<MassTier>& tiers, const BuildingParams& params,
                           std::size_t entranceEdge);
// wantsCore ? corePreset(plan, massStack(plan, params), ...) : invalid -- what coreFor reads.
CirculationPlan circulationFor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge);
// The core_plan.h reading of a plan whose shafts stand together: the stairs, the hoistways, the service block.
CorePlan asCorePlan(const CirculationPlan& c);

}  // namespace engine

#endif
