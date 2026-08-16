#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H

#include "road_network.h"
#include "../terrain.h"                   // TerrainFlatten (road -> terrain cut/fill)
#include "../../../renderer/renderer.h"   // RenderMesh
#include "../../../rt_math.h"             // Vec3
#include <array>
#include <functional>

namespace engine {

// How far ANY road paint floats above the asphalt (m). One constant so every
// decal agrees: the mesher's own stripes/crosswalks and the sim's parking-bay
// markings, which drifted to 5 cm and read as a slab hovering over the road.
constexpr double kRoadMarkLift = 0.02;

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

// Support columns under a bridge deck (ADR-0054): for each centerline index in `atSamples`, a
// box pier rising from the ground (`ground`, flat if unset) up to the deck underside
// (deckY[i] - deckThk) at that point, `width` across the deck and `depth` along the road. The
// structure that sells a flat overpass as carried on abutments straddling the road below.
RenderMesh bridgePiers(const std::vector<Vec2>& centerline, const std::vector<double>& deckY,
                       const std::vector<int>& atSamples, double width, double depth,
                       double deckThk, const Vec3& color,
                       const std::function<double(double, double)>& ground = {});

// One road CHAIN as a swept centreline: the polyline, its half-width (constant or
// per-point), optional absolute heights and per-point cross-slope. This is the unit
// the road mesher sweeps — `road_lattice.h` includes this header for it, and
// `weldChainProfiles` below reconciles a set of them into agreeing deck profiles.
struct UnionSpine {
    std::vector<Vec2> points;
    double halfWidth = 4.0;
    bool   closed = false;
    // The road class this chain carries (from the graph edge). Lets the welder's
    // markings, crosswalk gate, and lane dashes vary by CLASS instead of guessing
    // from width — the single lanesForClass() source (road-unification-plan P1).
    RoadClass klass = RoadClass::Local;
    // Semantic access bits (road_access::k*): PER-END, from the chain's
    // first and last edges (a chain's two ends can differ — an Intersection
    // at one, a ramp-approach Landing at the other). The mesher's zebra
    // reads the end-adjacent bits so it doesn't drop a crossable end just
    // because the chain was welded from the other end (roads-v2.2 #17/#21).
    uint8_t access = road_access::kAllStreet;      // first edge (front end)
    uint8_t accessBack = road_access::kAllStreet;  // last edge (back end)
    // ABSOLUTE per-point deck Y (parallel to `points`; EMPTY = drape on terrain
    // as before). This is the 3-D channel that lets ONE welder carry a ramp or an
    // elevated deck through the same flat-street pipeline: a chain with authored
    // heights (corridor decks/ramps) rides them; a plain street leaves it empty
    // and the profile is derived from the ground (road-unification: welder→3D).
    std::vector<double> yAbs;
    // Per-point HALF-WIDTH (parallel to `points`; EMPTY = constant `halfWidth`).
    // Lets ONE deck WIDEN along its length — an aux-lane / gore flare at a
    // freeway diverge (road-unification one-mesher P5). The weld strokes a
    // variable-offset ribbon from it and the lateral UV normalises against it.
    std::vector<double> hw;
    // Per-point CROSS-SLOPE (superelevation, dy per metre of lateral offset,
    // + = deck rises to the LEFT; parallel to `points`; EMPTY = laterally flat).
    // Banks the deck through a curve (one-mesher P6): the height sampler adds
    // signedLateral * crossSlope so the whole deck/wall/underside tilts.
    std::vector<double> crossSlope;
};
// (WeldSolidParams / weldSolid deleted — roads-v2 S6, one mesher.)


// The deck's smoothed per-spine profiles, junction-RECONCILED (chains sharing
// an endpoint agree on its height) — weldSolid rides these, and the terrain
// conform pass carves to the same surface. heightAt null = flat topY.
// `overlapReach` > 0 additionally reconciles MID-SPAN overlaps: wherever two
// chains' corridors come within (halfWidth_j + overlapReach) of each other —
// junction interiors, slip roads, close parallels — every deck is pulled to
// the LOWEST overlapping profile and the approach is re-eased at maxGrade.
// This is what lets the terrain conform carve to the same surface the decks
// ride: with one-sided reconciliation the higher deck floated up to 2.6 m
// above the carved ground (road_poke_probe metropolis, plan P3.1/P3.2).
std::vector<std::vector<double>> weldChainProfiles(
    const std::vector<UnionSpine>& spines,
    const std::function<double(double, double)>& heightAt, double topY,
    double maxGrade, double overlapReach = 0.0);


}  // namespace engine

#endif
