#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_FOOTPRINT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_FOOTPRINT_H

// CITY FOOTPRINT (8km-city plan P8, Glenn's masterplan step 1): before any
// road exists, derive the POLYGON of the whole buildable city area from the
// terrain — never into water or over steep ground, favoring the flat-to-hilly
// land, non-degenerate and never skinny. The footprint is the contract every
// later stage builds against: its rim carries the gates (and optionally a
// perimeter arterial), its interior is subdivided into district cells whose
// cut lines ARE the arterials, and its shape is the single hand-editable
// object that steers the entire city (planner override, P8-G).
//
// Terrain adaptation happens HERE, once — roads inherit it instead of each
// dodging terrain independently (the space-colonization tangles this round
// retires).

#include "buildability.h"
#include "polygon.h"

#include <cstdint>
#include <vector>

namespace engine {

// A station on the footprint boundary where the outside world connects:
// spokes start here, connectors to neighboring sites leave here.
struct FootprintGate {
    Vec2 pos;              // on the polygon boundary
    double s = 0;          // arc-length station along the boundary (CCW)
    Vec2 outward;          // unit boundary normal pointing out of the polygon
    int toSite = -1;       // >= 0: connector endpoint toward that site index
};

struct Footprint {
    Poly2 polygon;         // simple, CCW, resampled ~evenly; empty only on
                           // a hard failure (callers treat empty as "no city")
    Vec2 center;           // deepest interior point (downtown anchor)
    double coreDepth = 0;  // distance from center to the boundary (m)
    std::vector<FootprintGate> gates;   // ordered by s
    bool degraded = false; // fell back to an inscribed disc (hostile terrain)
};

struct FootprintParams {
    double cell = 80.0;        // flood-fill grid pitch (m)
    double simplifyEps = 0.0;  // Douglas-Peucker tolerance; 0 = 0.6 * cell
    double minWidth = 0.0;     // kill lobes narrower than this (m); 0 = keep all
    int smoothPasses = 2;      // Chaikin corner-cut rounds
    double resample = 60.0;    // final boundary vertex spacing (m)
    // Radial wobble of the SITE CLIP: 0 = a hard disc (open terrain yields a
    // circle), 0.1-0.2 = the clip radius varies by seeded low-frequency
    // harmonics so even a city on a featureless plain gets an organic
    // boundary. Terrain still carves the polygon regardless — the wobble
    // only shapes the outer limit.
    double wobble = 0.0;
    std::uint32_t seed = 5u;   // wobble phases (gates hash their own seed)
};

// Step 1 (F0): seed + radius + terrain -> one regularized polygon.
// Deterministic: integer grid, fixed-order flood fills, no RNG. The seed is
// snapped to buildable ground if it isn't already; hostile terrain degrades
// to an inscribed disc (degraded = true) rather than failing.
Footprint buildFootprint(const Vec2& seed, double maxRadius,
                         const HeightSampler& ground,
                         const BuildabilityConfig& build,
                         const FootprintParams& params);

// F1: gates at even arc-length spacing along the boundary (count = max(3,
// round(perimeter / gateSpacing))); the phase is hashed from the seed so gate
// placement is deterministic and independent of any shared RNG stream.
void placeFootprintGates(Footprint& f, double gateSpacing, std::uint32_t seed);

// Cross-site connector pairing: MST over site centers; for each link, the
// best-facing gate of each endpoint gets toSite set (ties by gate index;
// already-assigned gates are passed over while unassigned ones remain).
void pairConnectorGates(std::vector<Footprint>& sites);

}  // namespace engine

#endif
