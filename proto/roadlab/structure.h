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
    double slopeWidth = 13.0;   // how far a cut/fill batter runs out from the road
    bool enabled = true;
};

double terrainBaseHeight(const TerrainParams& p, double x, double z);

// Ground height with roads conformed into it. At-grade, embankment and cut
// stretches pull the terrain to their own surface and batter out over
// `slopeWidth`; bridge and tunnel stretches deliberately do NOT, which is the
// whole difference between a road on the ground and a road over it.
double terrainHeightAt(const Network& net, const TerrainParams& p, double x, double z);

// A grid, with cells that fall on a carriageway skipped so the ground never
// fights the road surface for the same pixels.
void tessellateTerrain(const Network& net, const TerrainParams& p, Vec2 lo, Vec2 hi, double cell,
                       Mesh& out);

}  // namespace roadlab

#endif
