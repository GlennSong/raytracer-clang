#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_METRO_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_METRO_H

#include "polygon.h"
#include "road_network.h"   // RoadGraph
#include "buildability.h"   // HeightSampler, BuildabilityConfig (terrain-aware layout)
#include "city_footprint.h" // Footprint (P8 footprint-first skeleton)
#include <cstdint>
#include <string>

namespace engine {

// A whole-metro road generator (the "kind":"metro" recipe). Unlike buildDistrict
// — which subdivides ONE footprint — this grows a network of ORGANIC ARTERIALS
// between a handful of hotspots by multi-source space colonization, stitches the
// separately-grown trees into one graph, and closes a few loops so the arterials
// actually enclose blocks (a tree encloses none). Each enclosed block (a face of
// the arterial graph, via extractBlocks) is then filled by the SAME OBB
// subdivision buildDistrict uses — so local streets are cuts of the arterial-
// bounded block and come off the arterials naturally — or by concentric
// rings+spokes for a radial hub. The result is one connected, planarized graph
// that flows through the normal planarize -> weld -> conform pipeline.
// A hub with a district flavor: the metro's polycentric zoning unit. `kind`
// mirrors DistrictTag by value (0 financial, 1 commercial, 2 residential,
// 3 oldtown, 4 industrial) without depending on architect.h.
struct CityHub {
    Vec2 pos{0, 0};
    int  kind = 0;
    bool radial = false;
    int  site = 0;   // which MetroSite grew this hub (0 = the primary city)
};

// A settlement footprint for multi-site metros (8km-city plan P2): the primary
// city plus satellite towns grow in ONE graph — one planarize, one mesher, and
// the hub-to-hub backbone MST spans every site, so town connectivity is
// structural rather than stitched.
struct MetroSite {
    Vec2   center{0, 0};
    double radius    = 400.0;  // footprint half-extent (m)
    int    hotspots  = 3;      // >=1; the first is the site's central hub
    double blockSize = 0.0;    // 0 = inherit MetroParams::blockSize
    double density   = 1.0;    // scales this site's ambient attractor field
    int    kindBias  = -1;     // central-hub district kind (-1 = default cycle)
};

struct MetroParams {
    // §10.6 corridor freeways are OPT-IN (generate: "corridor_freeways").
    // Device verdict on the first integration: "a complete disaster, lol" —
    // route chaining kinked, routes overlapped, ramps hit water. The system
    // goes back to the LAB until a car provably drives an on-ramp; until
    // then the metro keeps its legacy freeway-width street edges.
    bool corridorFreeways = false;

    Vec2   center{0, 0};
    double radius      = 700.0;   // footprint half-extent (m)
    int    hotspots    = 6;       // number of hubs (>=2); one central, rest ringed
    double blockSize   = 70.0;    // target block edge for the street subdivision (m)
    double arteryWidth = 13.0;    // width tagged on arterial edges
    double streetWidth = 7.0;     // width tagged on local streets
    bool   ringRoad    = false;   // add a deliberate ring road around the core
    std::uint32_t seed = 5u;

    // METROPOLIS tier (2 km-class cities): a FREEWAY backbone is laid hub-to-hub
    // FIRST (MST + nearest extras, gently curved), and arterial growth seeds at
    // the hubs AND at interchange points spaced along the freeways — so the city
    // grows along its corridors the way real metros do. Collector streets are
    // the first (widest) cuts of the block subdivision, before the local grid,
    // and each block takes its size from the nearest hub's district flavor.
    bool   freeways           = false;  // lay the hub-to-hub freeway backbone
    double freewayWidth       = 22.0;   // ~6 lanes (DesignRules::Freeway)
    // Class stamped on the legacy backbone edges. Freeway (the historical
    // behavior) strips sidewalks/crosswalks/frontage and blocks foot routing;
    // a no-freeway metro sets Arterial so the spine stays a street.
    RoadClass backboneClass   = RoadClass::Freeway;
    double collectorWidth     = 9.5;    // 2 lanes + parking
    double collectorSpan      = 0.0;    // faces wider than this get collector
                                        // cuts first (0 = 3x blockSize)
    double interchangeSpacing = 520.0;  // arterial seed spacing along freeways

    // GROWTH SPACING (the "room to breathe" dials, device feedback: blocks need
    // space for lots + landscaping, roads should run LONGER between junctions).
    // Defaults are the proven small-metro values; a 2 km metro roughly doubles
    // them so the arterial mesh is sparser and every enclosed face is big
    // enough to parcel properly.
    double segLength       = 18.0;   // colonization step (arterial segment, m)
    double influence       = 240.0;  // attractor influence radius (m)
    double killRadius      = 48.0;   // attractor consumed within this (m)
    double mergeRadius     = 34.0;   // growth tips fuse across trees within this (m)
    double corridorSpacing = 55.0;   // corridor attractor spacing + jitter (m)
    double ambientPer500   = 90.0;   // ambient attractors per (500 m)^2
    double loopMin         = 80.0;   // loop-closing link length range (m)
    double loopMax         = 190.0;

    // Floor on the fabric-fill cell edge (m). The per-district cell caps
    // (kindCellCap, ~70-156 m) clamp fabric to small-metro spacing no matter
    // how big blockSize is; a big-block city (min intersection spacing 150 m)
    // floors them here instead of fighting mergeShortEdges afterwards. 0 = the
    // legacy caps apply unchanged.
    double minBlockEdge = 0.0;

    // P7 PATCH-CONFORMING FABRIC (8km-city plan): how city-site faces are
    // subdivided. "" = legacy gridFill (every shipped level). "chords" =
    // stationed opposite-side chords blended toward the Coons iso-curves;
    // "bisect" = recursive near-midpoint bisection with node hygiene;
    // "court" = one perimeter ring + ribs, big-block center; "mix" = seeded
    // per-face choice weighted by the nearest hub's district kind
    // (financial/commercial -> chords, oldtown/industrial -> bisect,
    // residential -> 60/40 chords/court). Towns always keep gridFill. The
    // fabric blockLen is clamped >= 150 m this round (Phase-0 gate
    // reconciliation — regional 110 m grading is a later change).
    std::string fabric;
    // Core fabric block edge (m). 0 = flat max(150, minBlockEdge). Non-zero
    // (typically 110) requires the REGION-AWARE consolidation floor in the
    // recipe tail — the density unlock that makes downtown fabric legal.
    double fabricCoreLen = 0;
    double fabricConform = 0.15;        // 0 chord .. 1 Coons, per line
    double fabricJitter = 0.12;         // station jitter (fraction of a gap)
    double fabricSoftCollapse = 0.8;    // bisect soft-band fuse probability

    // Satellite settlements. Empty = single-site legacy behavior driven by
    // center/radius/hotspots/blockSize above. Non-empty REPLACES them: sites[0]
    // is the primary city (keeps the financial-core hub cycle), later sites are
    // towns whose central hub takes kindBias.
    std::vector<MetroSite> sites;

    // ARTERIALS-ONLY (city-pipeline v2 stage 1): emit ONLY the arterial
    // skeleton + freeway seeds; skip the per-face local/collector fabric fill.
    // The two-tier rebuild fills blocks from district templates instead of
    // colonization, so the local grid is generated downstream, not here.
    bool   arterialsOnly = false;

    // P8 FOOTPRINT-FIRST skeleton (Glenn's masterplan; city_footprint.h).
    // "" = legacy colonization skeleton (every shipped level). "footprint" =
    // derive a terrain-aware footprint polygon per site, gates on its rim,
    // and (P8-C) build the arterials by recursive bisection of the polygon —
    // no space colonization at the arterial tier. Stage B wiring: footprints
    // are computed and exported for the planner overlay while the roads
    // still come from the legacy growth; P8-C swaps the skeleton itself.
    std::string skeleton;
    double footprintCell = 80.0;    // F0 flood-fill grid pitch (m)
    double footprintWobble = 0.12;  // radial clip wobble (0 = compass circle)
    double districtLen   = 1500.0;  // bisection stop: target district cell (m)
    double gateSpacing   = 1100.0;  // rim gate arc spacing (city; towns derive)
    bool   rimRoad       = true;    // perimeter arterial on the boundary
    bool   spineRoad     = true;    // founding road between opposite gates
    double skeletonSway  = 0.05;    // spoke/cut meander amplitude
    double arterialSpan  = 0.0;     // min arterial junction span; 0 = derived
    // P8-D pipeline stepper (footprint mode only): "" = full build,
    // "footprint" = polygons+gates only (empty graph), "skeleton" = arterials
    // only, "collectors" = arterials + collector cuts, no street fabric.
    std::string stopAfter;

    // Terrain-aware layout (optional). When `ground` is set, hotspots, arterial
    // growth and blocks are gated on the buildability of the ground: the city
    // hugs buildable land and avoids water / steep mountain, instead of marching
    // over them. Unset `ground` = the old terrain-blind 2D layout.
    HeightSampler      ground;    // terrain height sampler (null = no gating)
    BuildabilityConfig build;     // slope/water thresholds for the gate

    // When non-null, receives the hubs (with district kinds) so the caller can
    // drive polycentric zoning (DistrictMap::hubs) from the same layout.
    std::vector<CityHub>* outHubs = nullptr;

    // When non-null (and skeleton == "footprint"), receives the per-site
    // footprints (polygon + gates) — the planner's Footprint overlay and the
    // editor's future hand-edit surface read these.
    std::vector<Footprint>* outFootprints = nullptr;
};

// Grow the metro and return its planarized RoadGraph (arterials
// RoadClass::Arterial, collector cuts RoadClass::Collector, local streets
// RoadClass::Local). §10.6: the freeway backbone is NO LONGER street edges —
// each hub-to-hub route is returned in `freewayPlans` (anchor polylines) for
// the loader to build as a real CORRIDOR (clothoid alignment, profile,
// stamped interchanges). Interchange seeds still feed street growth so the
// city grows TOWARD the future ramps.
RoadGraph buildMetro(const MetroParams& p,
                     std::vector<std::vector<Vec2>>* freewayPlans = nullptr);

}  // namespace engine

#endif
