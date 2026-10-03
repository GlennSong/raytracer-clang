#ifndef ENGINE_PROCGEN_FURNITURE_LIBRARY_H
#define ENGINE_PROCGEN_FURNITURE_LIBRARY_H

// THE FURNITURE LIBRARY (Glenn, 2026-10-01: "an asset library of furniture items ... tag the furniture for use in
// different places ... the furniture could have interaction points like sitting or operating it"). Each kit piece
// (furniture_kit.h) may carry a DESCRIPTION, authored as data in assets/scripts/furniture_library.lua: its family
// and tags, and how a body uses it.
//
// SPOTS AND VERBS. A spot is a physical place a body goes -- a sofa's three seats, a bed's mattress and its two
// edges. A verb ("sit", "lie") names the spots it uses, where the body goes (the eye, the way it faces) and where
// it stands up again. Occupancy is per SPOT: a verb is offered only while every spot it uses is free, so one person
// sitting in the middle of the sofa leaves the other seats to sit on but no room to lie down. The first verb is
// the piece's primary (tap the interact key), the second its secondary (hold it).
//
// Everything is in PIECE SPACE: x across the piece, y up from the floor, z out from the wall it backs onto -- a
// seat faces +z. A placed piece's transform (PlacedPiece::xform) takes it to the world.

#include "furniture_kit.h"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

enum class Verb : uint8_t { Sit, Lie, Count };
const char* verbName(Verb v);
bool verbByName(const std::string& name, Verb& out);

struct FurnSpot {
    std::string id;
    Vec3 at{0, 0, 0};   // where the body's weight goes (a seat's centre, the mattress's middle)
};

struct FurnVerb {
    Verb verb = Verb::Sit;
    uint32_t spots = 0;       // bit i = spot i
    Vec3 eye{0, 1.2, 0};      // the user's eye
    Vec3 look{0, 0, 1};       // which way they face (a lying user looks up and along the bed)
    Vec3 exit{0, 0, 1};       // where they stand up again, on the floor
    std::string label;        // the prompt's word ("sit", "lie down"); the verb's name if empty
};

// AN ANCHOR (M3): a surface on the piece that takes small things -- a nightstand's top, a desk's corner, a
// shelf -- its centre `at` (piece space), the footprint it offers (w across, d deep), and what it takes: goods
// whose tags include one of `accepts`. `chance` of being dressed at all (a bare surface reads lived-in too).
struct FurnAnchor {
    std::string id;
    Vec3 at{0, 0, 0};
    Real w = 0.3, d = 0.3;
    std::vector<std::string> accepts;
    Real chance = 1.0;
    Real yaw = 0;   // turn the good about +y (radians): a lamp at a bed's left faces right
};

struct FurnitureAsset {
    Piece piece = Piece::Count;
    std::string family;                // seating, sleeping, storage, surface, goods, ...
    std::vector<std::string> tags;     // office, living, bedroom, outdoor, public, ... (goods: lamp, plant, books, ...)
    std::vector<FurnSpot> spots;       // at most 32
    std::vector<FurnVerb> verbs;       // [0] the primary
    // CLEARANCE (M3): the floor kept free in front of the piece (+z, metres) and beside it -- a chair's pull-out,
    // a wardrobe's doors, the side of a bed you get in from. The placer reserves it with the footprint.
    Real clearFront = 0, clearSide = 0;
    std::vector<FurnAnchor> anchors;
    // VARIETY (the campus): > 0, each placement takes one of this many designs by where it stands (the style bits),
    // not the room's -- a library's bookcases are not all the same bookcase
    int variety = 0;
};

// ROOM PROGRAMS (M3b, Glenn 2026-10-03: "go ahead with the furniture tags"): what a room of each kind holds, as
// DATA (assets/scripts/furniture_rooms.lua), each piece asked for by what it is -- a family and tags -- rather
// than by name, so a new piece described with the right tags turns up in the rooms it suits with no code.
//
// A PICK is either a named piece, or a family plus tags: every described piece of that family carrying ALL the
// tags and fitting the slot (its footprint within `fit`, the slot it goes in), the largest that fits winning
// (ties by the room's hash). A wardrobe and a nightstand are both bedroom storage; the slot decides which.
struct FurnPick {
    Piece piece = Piece::Count;          // a named piece (Count: pick by family + tags)
    std::string family;
    std::vector<std::string> tags;
    bool valid() const { return piece != Piece::Count || !family.empty(); }
};

// One piece of a step's SET, in the footprint's frame: x along the wall from the footprint's left end, z out from
// the wall, y up; `facing` false turns it back toward the wall (a chair at its desk). `fitW`/`fitD` the slot a
// tag pick must fit (0: the step's footprint).
struct FurnMember {
    FurnPick pick;
    Real x = 0, z = 0, y = 0;
    bool facing = true;
    Real fitW = 0, fitD = 0;
};

struct FurnStep {
    enum class Kind : uint8_t { Wall, Grid, Counter, Hang, OneOf };
    Kind kind = Kind::Wall;
    FurnPick pick;                       // Wall without a set, Grid, Hang
    Real w = 0, d = 0;                   // the footprint against the wall (along x out)
    std::vector<int> sides;              // walls to try, in order (0 the window wall, 2 opposite, 1/3 the ends)
    std::vector<int> ringSides;          // ...in a ring floor's room, when different
    bool longFirst = false;              // sides = the room's long walls first
    bool opposite = false;               // sides = opposite the previous step's wall (and only if it placed)
    bool tall = false;                   // stands up the wall: no picture over it
    Real clear = 0;                      // floor kept free in front (the piece's own clearance first, if described)
    int countMin = 1, countMax = 1;      // how many; `countPer` > 0: one per that many metres of the room's W
    Real countPer = 0;
    Real minW = 0;                       // only in a room wider (W) than this
    std::vector<FurnMember> set;         // the pieces placed at each footprint (empty: `pick`, centred)
    // Grid: cells w x d in rows from the window wall, aisles between; d2 > 0 a shorter cell where a full one does
    // not fit, built with style2 (the piece's style bits) -- an office pod of four where six will not go
    Real aisleX = 0, aisleZ = 0, d2 = 0;
    int style2 = -1;
    // Grid TIERS: each row's style bits its tier counted from the front (the far side from the window wall), so a
    // lecture hall's rows step up toward the back (LectureRow models its own riser)
    bool tiers = false;
    // Grid RUNS: after every `runN` cells along a row, a cross aisle `crossW` wide (a library's stack rows)
    int runN = 0;
    Real crossW = 0;
    // Counter: a run of 0.6 m modules on the longest wall: the roles' pieces and the pattern placing them
    std::string pattern;                 // "kitchen" | "kitchenette"
    FurnPick base, sink, hob, tallUnit, wallUnit;
    std::vector<FurnStep> alternatives;  // OneOf: one per building, by its hash
};

struct RoomProgram {
    std::string name;
    std::vector<FurnStep> steps;
};

class FurnitureLibrary {
public:
    void clear();
    void set(FurnitureAsset a);
    const FurnitureAsset* find(Piece p) const;   // nullptr: no description (the piece is scenery)
    bool interactive(Piece p) const { const FurnitureAsset* a = find(p); return a && !a->verbs.empty(); }
    // GOODS (M3): the described pieces of family "goods" carrying any of `tags`, in piece order.
    std::vector<Piece> goodsFor(const std::vector<std::string>& tags) const;
    std::size_t size() const;
    // THE PICK: the piece `p` names, or the best-fitting described piece of its family and tags for a w x d slot.
    // Count when nothing qualifies (the step is skipped).
    Piece pick(const FurnPick& p, Real w, Real d, uint32_t hash) const;
    // The room programs, by name (office, bedroom, flat, living, kitchen, open_plan, meeting, kitchenette, closet,
    // bath). nullptr: none loaded -- the room is left empty.
    void setProgram(RoomProgram prog);
    const RoomProgram* program(const std::string& name) const;
    std::size_t programCount() const { return programs_.size(); }

    // The process-wide library the runtime reads (the interaction system, the interior system); loaded once from
    // assets/scripts/furniture_library.lua by the scripting layer (furniture_library_lua.h).
    static FurnitureLibrary& global();

private:
    std::array<FurnitureAsset, kPieceCount> assets_{};
    std::array<bool, kPieceCount> has_{};
    std::vector<RoomProgram> programs_;
};

// May `v` start, with `taken` the spots in use?
inline bool verbFree(const FurnVerb& v, uint32_t taken) { return (v.spots & taken) == 0; }

}  // namespace engine

#endif
