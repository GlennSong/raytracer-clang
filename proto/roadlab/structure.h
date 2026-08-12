#ifndef ROADLAB_STRUCTURE_H
#define ROADLAB_STRUCTURE_H

// How a road is CARRIED. Elevation already lives on the spine, so stacking two
// carriageways twenty metres apart vertically costs nothing geometrically —
// what a structure adds is everything around the surface: whether the ground
// follows the road or passes under it, the deck and piers that hold it up, the
// bore that encloses it, and the clearance rule the lint pass checks.
//
// Barriers and parapets are swept profiles in the road's own (t, h) frame, so
// they bank and climb with the carriageway without anyone writing a special
// case for a barrier on a superelevated curve.

#include "network.h"
#include "tessellate.h"
#include <string>
#include <vector>

namespace roadlab {

// Convenience constructors for the spans a Road carries.
StructureSpan bridgeSpan(double s0, double s1, double pierSpacing = 32.0,
                         double clearance = 5.1);
StructureSpan tunnelSpan(double s0, double s1, double headroom = 5.5);
StructureSpan embankmentSpan(double s0, double s1);
StructureSpan cutSpan(double s0, double s1);

// The carrier at a station (AtGrade when no span covers it).
CarrierKind carrierAt(const Road& r, double s);

// Deck, parapets, piers, abutments, bores and portals for one road, plus the
// swept barriers its cross-section calls for anywhere along its length.
void tessellateStructures(const Network& net, const Road& road, Mesh& out);

// --- terrain --------------------------------------------------------------

struct TerrainParams {
    double base = 0.0;
    double amplitude = 6.0;
    double frequency = 1.0 / 260.0;
    uint32_t seed = 5;
    bool enabled = true;

    // Earthworks. The ground meets a road at the road's own edge height and
    // then runs away from it on a BATTER — a slope with an authored gradient —
    // until it reaches natural ground, where it daylights. That is what a real
    // cut or embankment is, and it is the reason these are gradients rather
    // than a width: a fixed-width feather makes the slope depend on how deep
    // the earthwork happens to be, so a shallow one is a lazy ramp and a deep
    // one is a cliff. Measured before this existed: 83 degrees.
    //
    // Rise:run. A cut stands steeper than a fill because it is undisturbed
    // material; 1:1 is a firm soil or soft rock cut, 1:2 a normal embankment.
    double cutBatter = 1.0;
    double fillBatter = 0.5;
    // How far a batter may run before we stop and leave the step. Real projects
    // answer this with a retaining wall; roadlab has no walls, so the residual
    // is counted in the fallback census rather than quietly smoothed away.
    double batterReach = 45.0;
};

double terrainBaseHeight(const TerrainParams& p, double x, double z);

// Ground height with roads conformed into it. At-grade, embankment and cut
// stretches pull the terrain to their own surface and batter out to daylight;
// bridge and tunnel stretches deliberately do NOT, which is the whole
// difference between a road on the ground and a road over it.
double terrainHeightAt(const Network& net, const TerrainParams& p, double x, double z);

// True where two earthworks claim the ground at heights their batters cannot
// reconcile — two roads at different levels closer together than their slopes
// can resolve. The real answer there is a retaining wall, which roadlab does not
// build, so the ground takes a step. Exposed so a caller (and the test) can tell
// a wall apart from a batter that has gone wrong.
bool terrainConflictAt(const Network& net, const TerrainParams& p, double x, double z);

// The steepest ground gradient the conform produces near `plan`, sampled in
// four directions at `step` metres. Exposed because "the earthwork never
// exceeds its batter" is the property worth testing, and it cannot be read off
// a height.
double terrainSlopeAt(const Network& net, const TerrainParams& p, Vec2 plan, double step);

// A grid, with cells that fall on a carriageway skipped so the ground never
// fights the road surface for the same pixels.
void tessellateTerrain(const Network& net, const TerrainParams& p, Vec2 lo, Vec2 hi, double cell,
                       Mesh& out);

}  // namespace roadlab

#endif
