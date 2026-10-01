#ifndef RAYTRACER_ENGINE_INTERACTION_H
#define RAYTRACER_ENGINE_INTERACTION_H

// INTERACTIONS (the furniture library, M1; Glenn, 2026-10-01: "sit for chair type objects and lay for bed ... lay
// on sofas and benches"). What a body can do with a piece is DATA on the piece's library entry
// (procgen/furniture_library.h: spots and verbs); here are the runtime's two halves.
//
//   Interactables  ONE ECS entity per SET of placed pieces -- a streamed interior's furniture (the interior system
//                  makes it with the building and destroys it on release), later a city cell's benches. A set holds
//                  the pieces' world transforms and which spots are taken; nothing per piece is an entity, so
//                  thousands of chairs cost nothing until someone stands next to one.
//   Seated         on the player while they sit or lie: which piece and verb, where the eye is. PlayerSystem
//                  leaves the character alone and puts the camera at the eye; standing up removes it.
//
// findInteraction is the one query (pure, headless-tested): from where someone stands and looks, the best piece
// within reach and, on it, the primary verb (tap) and the secondary verb (hold) still free.

#include "../rt_math.h"
#include "procgen/furniture_library.h"
#include "world.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace engine {

struct InteractPiece {
    uint8_t piece = 0;   // Piece
    Mat4 xform;          // piece space -> world
    uint32_t taken = 0;  // spots in use (bit i = the library entry's spot i)
};

struct Interactables {
    std::vector<InteractPiece> pieces;
    Vec3 lo{0, 0, 0}, hi{0, 0, 0};   // world bounds of the pieces' origins (the cheap first test)
    void refreshBounds();
};

struct Seated {
    Entity set;              // the Interactables entity
    uint32_t piece = 0;      // index into its pieces
    uint8_t verb = 0;        // index into the library entry's verbs
    uint32_t spots = 0;      // the spots this user holds
    Vec3 eye{0, 0, 0};       // world
    Vec3 look{0, 0, 1};      // world, unit
    Vec3 exit{0, 0, 0};      // world, on the floor
    bool aimed = false;      // the camera has been turned to `look` once
};

// Piece space -> world.
Vec3 piecePoint(const Mat4& m, const Vec3& p);
Vec3 pieceDir(const Mat4& m, const Vec3& d);

struct InteractChoice {
    Entity set;
    const Interactables* setPtr = nullptr;
    uint32_t piece = 0;
    int primary = -1;     // verb index for a tap, -1 none
    int secondary = -1;   // verb index for a hold, -1 none
    Real distance = 0;
    bool valid() const { return setPtr && (primary >= 0 || secondary >= 0); }
};

// The best piece within `reach` (horizontal metres from `feet`, the user's floor point) that `forward` roughly
// faces, among `sets`. On it: the PRIMARY verb is the nearest free use of the entry's first verb kind, the SECONDARY
// the nearest free use of any other kind (a sofa: sit on the nearest free cushion / lie along it).
InteractChoice findInteraction(const std::vector<std::pair<Entity, const Interactables*>>& sets,
                               const FurnitureLibrary& lib, const Vec3& feet, const Vec3& forward,
                               Real reach = 1.8);

// Take verb `vi` on piece `pi` of `set` for a user: marks its spots and fills `out`. False if taken meanwhile.
bool takeInteraction(Interactables& set, Entity setEntity, uint32_t pi, int vi, const FurnitureLibrary& lib,
                     Seated& out);
// Give the spots back (the set may already be gone: then nothing to do).
void releaseInteraction(Interactables* set, const Seated& s);

}  // namespace engine

#endif
