#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H

#include "road_network.h"
#include "../terrain.h"                   // TerrainFlatten (road -> terrain cut/fill)
#include "../../../renderer/renderer.h"   // RenderMesh
#include "../../../rt_math.h"             // Vec3
#include <array>
#include <functional>

namespace engine {

// Turn a road graph into a connected road *surface* (ADR-0044). The naive
// approach — one full-width ribbon per edge, run to the node centre — overlaps
// badly where roads meet (worst at a radial hub). This builds it properly:
//   * at each junction (degree >= 3) the incident roads are trimmed back to their
//     curb corners — the point where neighbouring roads' edges cross — and the
//     gap is filled with one triangulated junction pad, so roads meet *at* the
//     intersection instead of through it;
//   * the trimmed ribbons span the ground between junctions.
// Every patch of asphalt belongs to exactly one element, so nothing overlaps.
// Draped on `heightAt` (world XZ -> height; null = flat). Vertex-coloured.
struct RoadMeshParams {
    double lift = 0.25;                 // raise above the terrain to avoid z-fight
    double minSetback = 1.0;            // floor on the junction trim distance (m)
    Vec3   color{0.08, 0.08, 0.09};     // asphalt
    // Plaza: a node with many converging arms (a radial hub) trims them all back
    // to at least `plazaRadius`, so the pad fills a clean circular plaza instead
    // of a cramped fan of overlapping corners. 0 radius = off (ordinary junction).
    int    plazaMinArms = 6;
    double plazaRadius = 0.0;
    // Hairpins: a degree-2 bend sharper than this deflection (radians) is too tight
    // for the carriageway to round without the widened ribbon folding (ADR-0048), so
    // it is built as a junction-style turning pad (a switchback bulb) instead of a
    // simple bend. 0 disables. (pi*0.6 ~ 108-degree turn.)
    double hairpinDeflection = 0.0;
    // Rounded curb returns: a junction corner (between two adjacent arms) is filled
    // with a circular arc of this radius instead of a straight chamfer, so the
    // intersection corners read as real rounded kerbs and the sidewalk wraps them
    // with no notch. The arms are set back an extra `cornerRadius` to make room.
    // 0 = the plain straight chamfer (no extra setback).
    double cornerRadius = 0.0;
    // Sidewalks: a raised skirt along the carriageway edges (and around the
    // junction corners) — a curb lip facing the street, a concrete slab top, and
    // an outer face dropping back to the ground. 0 width = no sidewalks.
    double sidewalkWidth = 0.0;         // slab width beyond the carriageway (m)
    double curbHeight = 0.15;           // how far the lip stands above the road (m)
    Vec3   sidewalkColor{0.62, 0.62, 0.60};   // concrete slab
    Vec3   curbColor{0.48, 0.48, 0.47};       // curb faces
    // Lane markings: thin raised stripes painted on the carriageway — solid edge
    // lines, a double-yellow centreline between opposing directions, and dashed
    // white lane dividers. The lane count is derived from the road width
    // (width / laneWidth), so wider arterials read as multi-lane for free.
    bool   laneMarkings = false;
    // Paint lane markings via the RoadMarkings surface shader (road-local UV baked on
    // the carriageway) instead of stripe geometry (road-geometry-plan Problem 3). When
    // set, the carriageway ribbon still bakes UV but the geometric paintLine pass is
    // skipped; the caller sets Surface::RoadMarkings on the road material.
    bool   shaderMarkings = false;
    double laneWidth = 3.5;             // nominal lane width -> lane count (m)
    double markWidth = 0.16;            // painted stripe width (m)
    double markLift = 0.02;             // raise stripes above the asphalt (m)
    double dashLength = 3.0, dashGap = 3.0;   // lane-divider dash pattern (m)
    Vec3   laneColor{0.85, 0.85, 0.82};       // white lines
    Vec3   centerColor{0.80, 0.70, 0.12};     // yellow centreline
    // Crosswalks: a zebra band laid across each junction arm, just outside the
    // intersection (at the curb-corner trim where the ribbon meets the pad), so a
    // crossing appears exactly where two roads meet. Off by default.
    bool   crosswalks = false;
    double crosswalkDepth = 2.6;        // band length along the road (m)
    // No zebra across a FREEWAY: an arm/chain at or above this full width keeps
    // its paint but loses the pedestrian band (grade-separated roads don't have
    // street crossings). Wide enough that 4-lane arterials (~14 m) still get one.
    double crosswalkMaxWidth = 18.0;
    double crosswalkBar = 0.55, crosswalkGap = 0.5;   // zebra bar/gap (m)
    Vec3   crosswalkColor{0.85, 0.85, 0.82};
    // Terrain conformance: a ribbon is split along its length only where the ground
    // bends, so flat ground stays a single stable quad and polygons appear where the
    // surface actually needs to follow the terrain. `conformTol` is the chord sag (m)
    // allowed before another split; `conformStep` caps the longest flat span (m).
    double conformTol = 0.08;
    double conformStep = 24.0;
    std::function<double(double, double)> heightAt;
};

RenderMesh buildRoadMesh(const RoadGraph& graph, const RoadMeshParams& params);

// A circular curb-return fillet for an intersection corner (ADR-0044 corner fix). The two
// kerb lines cross at `corner`; `dirA`/`dirB` point FROM the corner back along each kerb
// toward the carriageway (i.e. -armDirection for the arm whose edge that kerb is). Returns
// the arc as a polyline from the tangent point on line A to the tangent point on line B —
// a TRUE fixed-radius arc tangent to both lines, so it reads as a real rounded kerb at ANY
// corner angle (acute or obtuse), unlike a mouth-to-mouth chord. `radius` is the desired
// kerb radius; `maxTangent` caps how far back the tangent points may sit (so the fillet
// shrinks to fit a tight corner instead of overrunning the trim) — the radius is reduced
// to honour it. Returns empty for a degenerate corner (near-straight or folded, or no
// room), where the caller should fall back to a straight chamfer. `maxStep` is the arc
// flattening tolerance in radians.
std::vector<Vec2> curbReturnFillet(const Vec2& corner, const Vec2& dirA, const Vec2& dirB,
                                   double radius, double maxTangent, double maxStep = 0.4);

// Sample a Hermite spline (endpoints p0,p1 with tangents m0,m1) into `segs+1` points,
// relaxing the tangent magnitudes if the curve would otherwise bend tighter than
// `minRadius` (ADR-0044 fold prevention — a road that folds on itself is a generation
// problem, fixed at the source by giving the curve a real minimum radius, like a road
// design speed). Endpoints and tangent DIRECTIONS are preserved (the road still leaves
// each node heading the right way); only over-tight curvature is eased toward the chord,
// which has infinite radius — so the relaxation always converges. A curve already gentler
// than minRadius is returned untouched. Pure + headless.
std::vector<Vec2> fairHermite(const Vec2& p0, const Vec2& m0, const Vec2& p1, const Vec2& m1,
                              int segs, double minRadius);

// Turn the raw terrain heights sampled under a road centerline into a DRIVABLE vertical
// profile (corridor model, ADR-0044 terrain conforming). `ground[i]` is the terrain height
// at centerline sample i and `s[i]` its arc-length position. The profile is smoothed (a
// 3-point moving average kills bumps so the road isn't a roller-coaster) then slope-limited
// to |grade| <= maxGrade (rise/run) by iterated forward/backward clamping. The terrain is
// later cut/filled to meet this profile (roadConformRegions), so a road follows the natural
// slope where it can and the ground is graded to it where it's too steep. Pure + headless.
std::vector<double> roadProfile(const std::vector<double>& ground,
                                const std::vector<double>& s, double maxGrade);

// Vertical profile for a road that must CLEAR what it crosses (ADR-0054 grade separation).
// `minHeight[i]` is the lowest the deck may sit at arc-length `s[i]` — the ground there, or,
// at a crossing, the lower road's deck + clearance. Returns the MINIMAL profile that stays
// >= minHeight everywhere and never exceeds |grade| <= maxGrade: the slope-limited upper
// envelope of the constraints (each point casts a downward "cone" of slope maxGrade; the
// profile is their max). So a road hugs the ground, ramps up at maxGrade to clear an
// obstacle, and eases back down — a bridge approach falls out for free. Where the approach
// length is too short to return to ground at grade, the ends ride high (the honest answer:
// the ramp needs more room). Pure + headless.
std::vector<double> clearanceProfile(const std::vector<double>& s,
                                     const std::vector<double>& minHeight, double maxGrade);

// Cut/fill footprints that grade the terrain to a road corridor (ADR-0044 terrain
// conforming). One flatten ramp per centerline segment, set to the road's vertical profile
// `profileY` (from roadProfile), with half-width = carriageway half-width + `shoulder` and a
// `falloff` feather back to natural ground (the embankment). Fed to the terrain via
// applyFlatten so the ground meets the road exactly — cut where land is higher, fill where
// lower — so no terrain pokes through. Pure; returns the regions for the caller to apply.
std::vector<TerrainFlatten> roadConformRegions(const std::vector<Vec2>& centerline,
                                               const std::vector<double>& profileY,
                                               double halfWidth, double shoulder,
                                               double falloff);

// Ear-clipping triangulation of a simple polygon (any winding; non-convex OK) into index
// triples into `poly`. For the junction pad (ADR-0044): fanning from the node centre
// assumes the ring is star-convex from it, which T-junctions, mixed-width arms and notched
// rings violate — the fan then self-overlaps. Triangulating the ring boundary instead is
// robust for any simple polygon. Returns no triangles for n < 3, or stops early on a
// degenerate / self-intersecting ring it can't fully ear (covers what it can).
std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<Vec2>& poly);

// Stroke a centerline polyline into a flat filled ribbon — path stroking (ADR-0048),
// the back-to-basics primitive. Robust for ANY curve at ANY width including bends
// tighter than the width: per-segment trapezoids (variable half-width per point) plus
// a round join that fans the OUTER wedge at each vertex. The inside of a bend has the
// trapezoids overlap — a coplanar fill, not a fold — and a 180-degree vertex gets a
// semicircular turning cap for free. `halfW` is per-point (clamped/repeated if short);
// the ribbon lies flat at height `y`. `closed` strokes a loop (a ring/circle).
RenderMesh strokeRibbon(const std::vector<Vec2>& centerline,
                        const std::vector<double>& halfW, double y,
                        const Vec3& color, bool closed = false);

// A bridge DECK ribbon that rides a vertical profile (ADR-0054). Unlike strokeRibbon (one
// flat height), the deck top sits at `deckY[i]` per centerline point — feed it the output of
// `clearanceProfile` and the deck lifts onto the bridge and ramps back to grade. A slab of
// `thickness` is given a vertical fascia along both edges so it reads as a real deck edge,
// not a painted stripe. Pure + headless; vertex-coloured. (Piers/abutments are a later pass.)
RenderMesh bridgeDeck(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                      double halfWidth, const Vec3& deckColor, double thickness = 0.6,
                      const Vec3& sideColor = Vec3(0.30, 0.30, 0.32));
// Per-point half-width overload (ADR-0053): the deck can TAPER along its length, so a ramp can
// narrow to a nose where it meets the mainline (a merge/diverge gore) instead of ending blunt.
RenderMesh bridgeDeck(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                      const std::vector<double>& halfWidths, const Vec3& deckColor,
                      double thickness = 0.6, const Vec3& sideColor = Vec3(0.30, 0.30, 0.32));

// Support columns under a bridge deck (ADR-0054): for each centerline index in `atSamples`, a
// box pier rising from the ground (`ground`, flat if unset) up to the deck underside
// (deckY[i] - deckThk) at that point, `width` across the deck and `depth` along the road. The
// structure that sells a flat overpass as carried on abutments straddling the road below.
RenderMesh bridgePiers(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                       const std::vector<int>& atSamples, double width, double depth,
                       double deckThk, const Vec3& color,
                       const std::function<double(double, double)>& ground = {});

// Parapet barriers along both edges of a deck (ADR-0054): a low wall standing `height` above
// the deck on each verge, the full length — the safety rail of an overpass/viaduct.
RenderMesh deckBarriers(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                        double halfWidth, double height, const Vec3& color);
// Per-point half-width overload: barriers follow a tapering deck edge.
RenderMesh deckBarriers(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                        const std::vector<double>& halfWidths, double height, const Vec3& color);

// Lane markings painted on a deck/road that rides a vertical profile: solid edge lines, a
// solid centreline (`centerColor`, e.g. yellow — set `center=false` for a one-way ramp), and
// dashed lane dividers. Lane count = round(2*halfWidth / laneWidth). Stripes ride `lift` above
// the deck so they sit on the surface at any height. The finishing detail that makes a deck
// read as a multilane road.
struct DeckMarkParams {
    double laneWidth = 3.6;
    double markWidth = 0.18;
    double lift = 0.04;
    bool   center = true;
    double dashLength = 4.0, dashGap = 5.0;
    Vec3   laneColor{0.85, 0.85, 0.82};
    Vec3   centerColor{0.80, 0.70, 0.12};
};
RenderMesh deckMarkings(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                        double halfWidth, const DeckMarkParams& params);

// Union a set of centerline spines into ONE non-overlapping filled mesh (ADR-0048).
// Where ribbons cross or overlap, the result is a single merged surface, not stacked
// ribbons. Method: the signed distance to a polyline IS the Minkowski sum (so round
// joins/caps fall out), `min` over the spines is the union, and the region {sdf < 0}
// is meshed by marching-squares *filled cells* on a `cell`-spaced grid — so the
// triangles are non-overlapping by construction (cells are disjoint) and holes (a
// ring's centre, a roundabout) appear for free. Crisp to ~`cell`; finer = sharper.
struct UnionSpine {
    std::vector<Vec2> points;
    double halfWidth = 4.0;
    bool   closed = false;
};
RenderMesh unionRibbons(const std::vector<UnionSpine>& spines, double cell,
                        double y, const Vec3& color);

// WELD spines into ONE polygonal road surface via the unified join engine (unified-road-plan):
// each spine -> exact ribbon outline (offsetPolyline), boolean-unioned (polygonUnion), holes
// bridged in (bridgeHoles), then triangulated — so enclosed blocks stay open. The sharp, exact,
// light counterpart to unionRibbons' SDF grid — same welded result, but real polygons (no grid
// blur) and UV-ready. `cornerRadius` > 0 fillets the welded outline's convex corners (the ONE
// rounding pass: curb returns at junctions, rounded bends and road ends — concave notches and
// near-straight/near-spike corners are left as-is). Flat at height `y`, vertex-coloured.
RenderMesh weldRibbons(const std::vector<UnionSpine>& spines, double y, const Vec3& color,
                       double cornerRadius = 0.0);

// EXTRUDE the welded outline into a closed SOLID with real thickness — top deck, vertical side
// walls, and a bottom — instead of a one-sided floating ribbon (unified-road-plan task 3). The
// road rides a smoothed, grade-limited vertical profile: terrain height is sampled along each
// spine's centerline and ironed flat by roadProfile (follows hills, irons out bumps), then any
// surface point takes the nearest spine's smoothed height. Block-interior holes get inner walls
// too, so an open block is a real shaft. `cornerRadius` fillets junctions as in weldRibbons.
struct WeldSolidParams {
    double topY = 0.06;                         // flat fallback height when no terrain is given
    double thickness = 0.5;                     // top-to-bottom extrude depth (m)
    double cornerRadius = 0.0;                  // junction curb-return fillet
    double maxGrade = 0.08;                     // profile slope limit (rise/run) for smoothing
    Vec3   topColor{0.10, 0.10, 0.11};          // asphalt deck
    Vec3   sideColor{0.18, 0.18, 0.19};         // curb / road edge
    Vec3   bottomColor{0.05, 0.05, 0.05};       // underside
    // Raised sidewalk: a curbed concrete band run along the welded boundary loops (the
    // outer carriageway edge + every block-interior/island hole), offset outward by
    // sidewalkWidth and standing curbHeight above the deck. 0 width = no sidewalks.
    double sidewalkWidth = 0.0;                  // slab width beyond the carriageway (m)
    double curbHeight = 0.15;                    // curb lip above the deck (m)
    Vec3   sidewalkColor{0.55, 0.55, 0.52};      // concrete slab top
    Vec3   curbColor{0.42, 0.42, 0.40};          // curb riser faces
    std::function<double(double, double)> heightAt;  // terrain (world XZ -> height); null = flat
    // Zebra crosswalks painted into the road TEXTURE (not overlaid geometry): when
    // true, the carriageway UV `mv` is baked as metres PAST the junction mouth, so
    // the RoadMarkings shader stripes a set-back crosswalk band on each approach.
    // When false, mv is a large sentinel so no band is painted (ADR-0062).
    bool   crosswalks = false;
    // No zebra on FREEWAY-width chains (>= this full width): grade-separated
    // roads have no street crossings. Mirrors RoadMeshParams::crosswalkMaxWidth.
    double crosswalkMaxWidth = 18.0;
    // Junction pads (device: mesh holes at skewed junctions): every chain ENDS
    // square to its own direction at a junction node, so where the arms meet
    // off-square (a bent through-road at a T, arms at 170°) the caps disagree and
    // a knife-wedge of ground shows through between them. A disc of the widest
    // incident arm's half-width per junction node — unioned into the outline and
    // laid as a plain deck fan — covers every such wedge at any arm angle.
    std::vector<Vec2>   padCenters;
    std::vector<double> padRadii;
};
RenderMesh weldSolid(const std::vector<UnionSpine>& spines, const WeldSolidParams& p);

// The deck's smoothed per-spine profiles, junction-RECONCILED (chains sharing
// an endpoint agree on its height) — weldSolid rides these, and the terrain
// conform pass carves to the same surface. heightAt null = flat topY.
std::vector<std::vector<double>> weldChainProfiles(
    const std::vector<UnionSpine>& spines,
    const std::function<double(double, double)>& heightAt, double topY,
    double maxGrade);

// Union spines into a full ROADBED — carriageway + raised sidewalk + curb — and drape
// it on terrain (ADR-0048). The SDF gives the bands for free: carriageway is {sdf<0},
// sidewalk is {0<=sdf<sidewalkWidth}, with a curb step between, all from one distance
// field, so junctions merge and sidewalks wrap the network with no special cases.
// Each vertex is seated on `heightAt` (the terrain), so the dense union grid drapes
// smoothly over 3D ground. Per-band vertex colours (road/sidewalk/curb) carry a small
// grain so it reads as a surface; pass the result to add_solid(mesh, material) for a
// procedural asphalt/concrete texture on top.
struct RoadbedParams {
    double cell = 0.6;
    double sidewalkWidth = 2.0;
    double curbHeight = 0.18;
    double lift = 0.05;
    double grain = 0.06;                 // per-cell colour jitter (texture-ish)
    Vec3   roadColor{0.09, 0.09, 0.10};
    Vec3   sidewalkColor{0.58, 0.58, 0.55};
    Vec3   curbColor{0.50, 0.50, 0.48};
    std::function<double(double, double)> heightAt;   // terrain drape (flat if unset)
    // Junction interiors (centre + radius) where lane markings are SUPPRESSED — a crossing reads as
    // plain asphalt, and the road-local UV is ambiguous there anyway (two roads meet), so this is also
    // the fix for the marking glitch in the middle of an intersection.
    std::vector<Vec2>   noPaintCenters;
    std::vector<double> noPaintRadii;
};
RenderMesh unionRoadbed(const std::vector<UnionSpine>& spines, const RoadbedParams& params);

// Convert a road GRAPH's edges to spines (one per edge, half-width from its width) and
// build the roadbed. The bridge from a generated network (city.layout / gridRoads /
// radialRoads / tensorRoads) to a draped, merged, non-overlapping ribbon surface — so
// the engine's road generators feed straight into the union/roadbed pipeline.
RenderMesh unionRoadbed(const RoadGraph& graph, const RoadbedParams& params);
std::vector<UnionSpine> graphToSpines(const RoadGraph& graph);

// Trace a road graph into its road CHAINS: maximal polylines of degree-2 nodes between
// junctions (degree != 2), plus any pure loops. The unit a centerline marking follows.
std::vector<std::vector<Vec2>> traceChains(const RoadGraph& graph);

// Lane markings (ADR-0048): a thin centerline stripe down each road chain, pulled back
// from the junctions (markings break at intersections) and draped on the terrain a
// hair above the asphalt. The finishing detail that makes a ribbon read as a road.
struct LaneMarkParams {
    double markWidth = 0.25;
    double trim = 4.0;                       // pull the stripe back from each junction (m)
    double lift = 0.10;                      // height above the road surface (m)
    double dashLength = 0.0, dashGap = 0.0;  // 0 => solid centerline
    Vec3   color{0.86, 0.72, 0.16};          // yellow
    std::function<double(double, double)> heightAt;
};
RenderMesh laneMarkings(const RoadGraph& graph, const LaneMarkParams& params);

}  // namespace engine

#endif
