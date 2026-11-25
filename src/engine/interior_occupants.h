#ifndef RAYTRACER_ENGINE_INTERIOR_OCCUPANTS_H
#define RAYTRACER_ENGINE_INTERIOR_OCCUPANTS_H

// PEOPLE INSIDE (campus milestone 4; Glenn: "students ... sitting in lectures ... living in the dorms"). The city
// sim knows how many people are inside a building -- an agent indoors rests at its door, undrawn -- but not where
// in it. When a building's interior streams in, its furniture set says where a body can be: the seats, the beds,
// the lectern. planOccupants puts that many people on them, the same way every time for the same building and
// hour, as poses to draw (a sitting, lying or standing person) and the spots they hold, so the player is not
// offered a seat someone is in.

#include "interaction.h"
#include <cstdint>
#include <vector>

namespace engine {

struct BatchSpot;   // a place on a prefab copy (prefab_batch.h)

struct Occupant {
    enum class Pose : uint8_t { Sit, Lie, Stand };
    Pose pose = Pose::Sit;
    // Where the body is drawn: Sit -- the seated mesh's origin (the hip on the seat, facing +z); Lie and Stand --
    // the standing mesh's origin (its middle, 0.9 m above its feet, facing +z), a lying body turned onto its back
    // along the bed.
    Mat4 at;
    uint32_t piece = 0;   // index into the set's pieces
    uint32_t spots = 0;   // the spots held on it
    // ...or a place on a prefab copy (prefab_batch.h BatchSpot): its index in the list the plan was given, and then
    // `piece` and `spots` mean nothing. -1: on the furniture kit's own piece, as above.
    int batchSpot = -1;
};

struct OccupantPlan {
    int people = 0;        // how many the building has inside
    bool night = false;    // asleep: the beds first
    bool lecturer = false; // someone at the lectern when anyone is in (a class is on)
    uint32_t seed = 0;     // the building's own (its record index): the same seats each time
    // only these pieces: `units[piece.unit + 1]` non-zero (InteractPiece::unit, -1 the building's own); null: all
    const std::vector<uint8_t>* units = nullptr;
    int staff = 0;         // of `people`, how many WORK there: they go to the staff places (a counter, the bar, the
                           // pass) first, and nobody else does; staff with no place left are out the back
};

// `count` people on the free spots of `set` (spots already taken -- the player -- are left alone). Beds take the
// sleepers at night; by day people sit, a piece at a time with a gap here and there, never on a fixture (a
// toilet); a lectern takes its lecturer first, and while one is there the lecture hall's rows fill first. A place to
// STAND that is not a lectern (a lab bench) takes a standing worker like a seat takes a sitter. Fewer places than people: the rest are elsewhere in the building
// (a storey not streamed, the corridor).
// `prefabSpots`: the free places on the building's prefab copies (prefab_batch.h batchSpots), which take people as
// the kit's seats and beds do, in the same draw: a chair is a chair whichever way it is drawn. The planner marks
// each one it fills `taken`, so the next plan over the same list (another shop unit's) leaves it.
std::vector<Occupant> planOccupants(const Interactables& set, const FurnitureLibrary& lib, const OccupantPlan& plan,
                                    std::vector<BatchSpot>* prefabSpots = nullptr);

}  // namespace engine

#endif
