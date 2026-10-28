#ifndef ENGINE_PROCGEN_CITY_FLOOR_ZONES_H
#define ENGINE_PROCGEN_CITY_FLOOR_ZONES_H

// THE FLOOR AS A BLOCK (Glenn, 2026-10-16: "having a full rectangle, trying to fit a floor and then being thrown out
// because I can't seems pretty absolute ... similar to how we build lots out of city blocks we should come up with
// some scheme to partition the open space of an office floor ... use the tagging system that we use for city lots
// for furniture inside of a floor ... meeting rooms and a cubicle space and offices, a water cooler and so on").
//
// A storey's plate is cut the way the lot pass cuts a block (parcel.h): frontage first, then the court.
//
//   +----+-----+-----+-----+-----+-----+----+
//   | c  |  w  |  w  |  w  |  w  |  w  | c  |   WINDOW LOTS along the glass, cut at the facade's bays; CORNER
//   +----+-----+-----+-----+-----+-----+----+   lots where it turns (a rounded corner's arc is its own lot)
//   |              the aisle                |
//   |   +-------+------+  +------+-------+  |   INTERIOR LOTS: the court left between the aisle and the
//   |   |  i    |  i   |  |  i   |   i   |  |   corridor round the core, bisected like a block's remainder
//   |===+=======+======+==+======+=======+==|   the corridor's long lanes run on to the aisle: the floor's
//   |   |       [ core + corridor ]      |  |   STREETS, so every lot fronts circulation and the floor walks
//   ...                                         by construction
//
// Any plate: a rectangle, a chamfered or notched one, a rounded or stadium section, a drum, a wave -- a curve is
// cut into chords of at most a metre's bulge and each chord fronts its own lots. Each lot learns what the lot pass
// tells a lot (its site, area, frontage, depth, how far from the lifts), and a FLOOR PROGRAM (Lua,
// assets/scripts/floor_programs.lua, the way the archetype book picks buildings) tags it: reception, meeting
// rooms, private offices, a pantry, open plan. The tag decides the walls (an enclosed room gets partitions and a
// door onto circulation; an open zone none) and the furniture (the tag's RoomKind runs its furniture_rooms.lua
// program). The output is an ordinary RoomPlan, so the walls, the space graph and the furniture read it unchanged;
// its zones are real rooms that do not overlap (the old office floor's OpenPlan overlapped the rooms set into it).

#include "core_plan.h"
#include "polygon.h"
#include "room_plan.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace engine {

enum class ZoneSite : uint8_t { Window = 0, Corner = 1, Interior = 2 };
constexpr uint8_t zoneSiteBit(ZoneSite s) { return static_cast<uint8_t>(1u << static_cast<unsigned>(s)); }

// How a plate is cut (the program's numbers, metres).
struct FloorZoneParams {
    Real windowDepth = 4.5;   // a window lot's depth, from the facade's chord
    Real aisle = 1.6;         // between the window lots and the court
    Real street = 1.8;        // the corridor's long lanes run on to the aisle this wide
    Real lotWidth = 4.8;      // a window lot's width along the glass (grouped to the facade's bays)
    Real interiorLot = 24.0;  // the court is bisected down to lots of about this many m2
    Real minSide = 2.6;       // ...no side shorter than this
    Real minAccess = 0;       // a window lot with less than this onto circulation joins its neighbour along the glass
                              // (a flat needs its hall and bath on the corridor: a tight curve's wedges pair up)
    Real minLotArea = 0;      // ...and so does one smaller than this (m2: no flat in a leftover bay)
    // LOBBY MODE (a SHALLOW floor, a few metres from the glass to the core, #117 item 35): no corridor round the core,
    // which would leave the lots slivers -- the core keeps clear only its LOBBY, the strip in front of its lift doors
    // out to the glass. The lots take the band round the core whole; those off the lobby are reached through the open
    // zones beside them. zonedFloorPlan cuts this way when the corridor ring leaves no lots.
    bool lobbyOnly = false;
};

struct FloorZone {
    Poly2 poly;              // CCW, world XZ
    Poly2 frame;             // a rectangle inside it for the furniture (CCW); frame[0] -> frame[1] its side 0: the
                             // glass for a window or corner lot, else the side away from its door
    ZoneSite site = ZoneSite::Window;
    Real area = 0;
    Real frontage = 0;       // its glass, along the chord (window and corner lots)
    Real depth = 0;          // the frame's depth
    bool curved = false;     // its glass is an arc
    int group = -1;          // window lots: the chord they front; interior lots: the court piece they were cut from
    Real toLobby = 1e9;      // its centre's distance to the lift lobby's mouth
    Real toCore = 1e9;       // ...and to the corridor round the core
    Real accessLen = 0;      // the longest run of its edge onto the floor's circulation (corridor, streets, aisle)
    bool onStub = false;     // one of the two lots either side of a stub of corridor (a stadium's end), as it stays
};

struct FloorPartition {
    std::vector<FloorZone> zones;
    Real fieldArea = 0;      // the plate inside its walls, less the core and its corridor ring (m2)
    Vec2 lobby;              // the lift lobby's mouth
    // what no zone covers: the corridor ring, the streets, the aisle -- each piece its outline then its holes (the
    // zones it rings)
    std::vector<std::vector<Poly2>> circulation;
    bool lobbyMode = false;  // cut in lobby mode (FloorZoneParams::lobbyOnly)...
    std::vector<Poly2> lobbyArea;   // ...and its lobby, the floor's way in (the core's own walls are not)
};

// Cut a storey plate (CCW world XZ, the facade's outside line) round `core` (valid: the core; else none -- a
// stair-only building passes its well as a core) with the walls' inner face `inset` in. `keepOut` (a stair's well)
// is left out of every lot. Deterministic in its inputs.
FloorPartition partitionFloor(const Poly2& plate, const CorePlan& core, Real inset, const BuildingParams& params,
                              const FloorZoneParams& zp, const Poly2& keepOut = Poly2{});

// --------------------------------------------------------------------------------------------- the program
// ONE TAG of a floor program: which lots may take it and how many do, and what it builds. Rules are applied in
// order; a lot tagged by one is not offered to the next. Every lot no rule takes gets the program's `fill`, or
// `shallow` where it is too shallow for that.
struct ZoneRule {
    std::string tag;
    RoomKind kind = RoomKind::OpenPlan;
    uint8_t sites = 7;            // bits: zoneSiteBit(Window | Corner | Interior)
    int count = 0;                // exactly this many a floor (where lots allow) ...
    Real perM2 = 0;               // ... or one per this many m2 of floor ...
    Real share = 0;               // ... or this share of the lots it may take
    Real minArea = 0, maxArea = 1e9, minDepth = 0;
    Real minFloor = 0;            // only on a floor of at least this many m2 (a small floor has no copy room)
    bool enclosed = false;        // partitions round it, a door onto circulation
    uint8_t walls = 2;            // 0 drywall, 1 glass, 2 glass behind a curtain wall else drywall
    uint8_t near = 0;             // 0 spread out, 1 the lift lobby, 2 the core
    bool lifts = false;           // only on a floor with lifts (a stair-only office has no reception)
};

// A BUILDING'S HABIT (picked per building by seed and weight): numbers the rules take instead of their own --
// a company of corner offices, one of private offices along the glass, an all-open one.
struct ZoneStyle {
    std::string name;
    Real weight = 1;
    struct Set { std::string tag, field; Real value = 0; };
    std::vector<Set> sets;
};

struct FloorProgram {
    std::string name;
    FloorZoneParams zones;
    std::string fill = "open_plan", shallow = "desk_row";
    Real shallowDepth = 3.2;
    // HOMES (an apartment floor): its `flat` lots are laid out inside as flats -- living room and bedrooms on the
    // glass, the hall, bath and kitchen behind (room_plan.h layoutApartment), a studio where that will not walk; the
    // floor is no workplace (no office lighting), and one that will not walk is not opened up but laid another way
    // (zonedFloorPlan).
    bool residential = false;
    std::vector<ZoneRule> rules;   // `fill` and `shallow` are rules too (looked up by tag; their quotas unused)
    std::vector<ZoneStyle> styles;
};

// The programs by name, loaded once from assets/scripts/floor_programs.lua by the scripting layer
// (furniture_library_lua.h, the same VM as the furniture). Empty until then; floorProgram() falls back to the
// built-in program of that name (office, apartments: what the Lua file ships with), so procgen and tests need no Lua.
class FloorProgramBook {
public:
    void clear() { programs_.clear(); }
    void set(FloorProgram p);
    const FloorProgram* find(const std::string& name) const;
    std::size_t size() const { return programs_.size(); }
    static FloorProgramBook& global();
private:
    std::vector<FloorProgram> programs_;
};
FloorProgram builtinOfficeProgram();
FloorProgram builtinApartmentProgram();
const FloorProgram& floorProgram(const std::string& name);

// The RoomKind a program names ("open_plan", "meeting", ... -- space_graph's cellUseName with underscores).
bool roomKindFromName(const std::string& name, RoomKind& out);

// Tag `fp`'s zones by `prog` for one building (`seed` picks its style): the RoomKind index per zone, and whether it
// is enclosed and in glass. Exposed for the tests; zonedFloorPlan is the one call the planner makes.
struct ZoneTags {
    std::vector<int> rule;   // per zone: the rule index in prog.rules
    std::string style;
};
ZoneTags tagZones(const FloorPartition& fp, const FloorProgram& prog, uint32_t seed, bool lifts);

// THE FLOOR: partition, tag, walls and doors, walked from `entry` (the lift lobby, the stair's foot) before it is
// shipped -- an enclosed room it cannot reach is opened up, and a floor that still fails is left open. The ONE planner
// of an office or apartment floor (#117 item 35): a plate too shallow for a corridor round its core is cut in lobby
// mode (FloorZoneParams::lobbyOnly); a floor of homes that lays no flat that way is two flats wrapped round the core
// off its lobby (room_plan.h wrapFlatsLayout), else the `residents` program's lounge. Empty when the plate leaves no
// lots at all.
RoomPlan zonedFloorPlan(const Poly2& plate, const BuildingParams& params, const CorePlan& core, Real inset,
                        int storey, bool lifts, const Vec2& entry, const Poly2& stairWell, const FloorProgram& prog);

}  // namespace engine

#endif
