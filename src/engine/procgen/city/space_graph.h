#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SPACE_GRAPH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SPACE_GRAPH_H

#include "polygon.h"
#include "room_plan.h"       // RoomKind
#include "shape_grammar.h"   // BuildingParams, StoreyPlan
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// THE SPACE GRAPH (architectural toolkit, issue #105, foundation A; ADR-0174). A building is a set of CELLS -- the
// volumes a person can be in: a room, a storey's hall, a stair shaft, a hoistway, the roof deck -- joined by PORTALS
// -- the openings a person passes through: a door, a stair door, a lift door, an escalator's foot. A cell is
// three-dimensional (a stair shaft spans every storey it serves; an atrium will span the storeys it cuts); a storey
// plan is a VIEW of the graph (storeyView), not the model. Reachability over the graph is the gate every building
// must pass before it ships: from the street, every cell, with the walker's means (WalkerCaps).
//
// Step 1a of the brief on #105: the graph is built FROM today's growers -- the storey stack, the room planner, the
// core and the facade's doors -- by spaceGraphOf, an adapter that changes no mesh; later steps grow from it.

struct FloorSurface {
    enum class Kind : uint8_t { Level, Sloped, Stepped, Helical };
    Kind kind = Kind::Level;
    Real y0 = 0;              // world, at `origin`
    Vec2 origin{0, 0};        // where y0 is measured (Sloped: the grade runs from here)
    Vec2 grade{0, 0};         // Sloped: dy per metre in plan
    Real rise = 0, run = 0;   // Stepped: one row (a flight's riser and tread), climbing along `grade`'s direction
    Real yAt(const Vec2& p) const;
};
// A flight's riser and tread as a stair climbs (halfFlightRisers): the surface of a stair shaft cell.
void steppedFloor(FloorSurface& f, Real storeyH, Real tread, const Vec2& up);

enum class CellKind : uint8_t { Hall, Room, Stair, Lift, Roof, Pad };   // Pad: a helipad, a floor over the roof deck

struct Cell {
    CellKind kind = CellKind::Hall;
    RoomKind use = RoomKind::Hall;    // a Room's use (every other kind: Hall)
    uint8_t style = 0;                // a shop's trade
    Poly2 plan;                       // world XZ, the cell's outline
    std::vector<Poly2> holes;         // what is cut out of it (a hall's rooms and core; an upper mall level's atria)
    FloorSurface floor;
    Real height = 0;                  // floor to ceiling
    Real freeArea = 0;                // the plan's area less its holes: a hall its rooms cover whole has none
    int k0 = 0, k1 = 0;               // the storeys it spans, inclusive (the roof deck: the storey count)
    int tier = 0;                     // the massing tier of storey k0
};

struct Portal {
    enum class Kind : uint8_t { Door, Passage, Void, Stair, Escalator, Lift, Ramp, Vomitory };
    int a = -1, b = -1;       // the cells it joins; b < 0 = outside (the street)
    Kind kind = Kind::Door;
    Vec2 p0, p1;              // the opening's span, world XZ
    Real sill = 0, head = 0;  // above the storey's floor
    int storey = 0;           // where a walker crosses it (a lift door's: its landing's storey)
};

struct SpaceGraph {
    std::vector<Cell> cells;
    std::vector<Portal> portals;
    int entrance = -1;      // the ground hall (what the street doors open into)
    int roof = -1;          // the roof deck cell, when the core reaches it
    int storeys = 0;        // the storey count
    bool modelled = true;   // false: this building's inside is not a graph yet (a garage's decks and ramps)
};

// What a walker can use. The default is everyone: stairs, lifts, escalators, ramps.
struct WalkerCaps {
    bool stairs = true;
    bool lift = true;
    bool escalator = true;
    bool ramp = true;
};

bool passable(const Portal& p, const WalkerCaps& caps);
// Which cells a walker reaches from `from` (a cell index; -1 = the street, through every portal onto it).
std::vector<bool> reachableFrom(const SpaceGraph& g, int from, const WalkerCaps& caps);
bool reachable(const SpaceGraph& g, int from, int to, const WalkerCaps& caps);
// THE GATE: the cells a walker cannot reach from the street. Empty = the building passes. A hall its rooms cover
// whole (freeArea under kCoveredHall: a house whose plate is all rooms, a church that is its nave) is no place to
// stand and is not counted.
constexpr Real kCoveredHall = 2.0;
std::vector<int> unreachableCells(const SpaceGraph& g, const WalkerCaps& caps = WalkerCaps{});

// THE DERIVED VIEW: one StoreyPlan per storey, read off the Hall cells (plan = the hall's outline, y0 relative to
// `baseY`, h, tier). Equal to storeyPlans(plan, params) for every building spaceGraphOf builds: the step-1 guard.
std::vector<StoreyPlan> storeyView(const SpaceGraph& g, Real baseY);

// THE ADAPTER: the graph of the building today's growers grow on `plan` (world XZ) with `params`, wall base `baseY`.
// Cells: every storey's hall (its outline, holed by its rooms and the core), every walled room, each core stair and
// hoistway as one shaft spanning every storey, the roof deck when the core reaches it. Portals: the room planner's
// doors, the facade's street doors (storey 0), the shaft doors on every storey (Stair / Lift), the bulkhead's stair
// doors onto the roof, the straight stair's flights (hall to hall), a mall's escalators and glass lift.
// `roof`: what stands on the roof (a record's RoofItems), cut from the roof cell's free floor when given.
SpaceGraph spaceGraphOf(const Poly2& plan, const BuildingParams& params, Real baseY,
                        const std::vector<RoofItem>* roof = nullptr);

// THE BLUEPRINT SHEET: every storey drawn -- halls, rooms by use, shafts, doors, stair and lift doors -- the
// unreachable cells hatched red, as one SVG document.
std::string blueprintSvg(const SpaceGraph& g, const std::string& title, const std::vector<int>& unreachable);

const char* cellKindName(CellKind k);
const char* portalKindName(Portal::Kind k);
const char* cellUseName(RoomKind k);

}  // namespace engine

#endif
