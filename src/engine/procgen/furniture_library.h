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

struct FurnitureAsset {
    Piece piece = Piece::Count;
    std::string family;                // seating, sleeping, storage, surface, ...
    std::vector<std::string> tags;     // office, living, bedroom, outdoor, public, ...
    std::vector<FurnSpot> spots;       // at most 32
    std::vector<FurnVerb> verbs;       // [0] the primary
};

class FurnitureLibrary {
public:
    void clear();
    void set(FurnitureAsset a);
    const FurnitureAsset* find(Piece p) const;   // nullptr: no description (the piece is scenery)
    bool interactive(Piece p) const { const FurnitureAsset* a = find(p); return a && !a->verbs.empty(); }
    std::size_t size() const;

    // The process-wide library the runtime reads (the interaction system, the interior system); loaded once from
    // assets/scripts/furniture_library.lua by the scripting layer (furniture_library_lua.h).
    static FurnitureLibrary& global();

private:
    std::array<FurnitureAsset, kPieceCount> assets_{};
    std::array<bool, kPieceCount> has_{};
};

// May `v` start, with `taken` the spots in use?
inline bool verbFree(const FurnVerb& v, uint32_t taken) { return (v.spots & taken) == 0; }

}  // namespace engine

#endif
