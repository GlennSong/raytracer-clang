#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H

#include "road_network.h"
#include "../../../renderer/renderer.h"   // RenderMesh
#include "../../../rt_math.h"             // Vec3
#include <array>
#include <functional>
#include <unordered_map>

namespace engine {

// How far ANY road paint floats above the asphalt (m). One constant so every
// decal agrees: the mesher's own stripes/crosswalks and the sim's parking-bay
// markings, which drifted to 5 cm and read as a slab hovering over the road.
constexpr double kRoadMarkLift = 0.02;

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

// One road CHAIN as a swept centreline: the polyline, its half-width (constant or
// per-point), optional absolute heights and per-point cross-slope -- a lane-built
// city's published deck is a list of these (RoadDeck / RoadDeckField).
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
    // STRUCTURAL elevation: the chain carries authored absolute heights (a
    // corridor deck / ramp — any node elevAbsolute) or rides a layer above
    // grade. The builder fills yAbs for EVERY chain, so yAbs
    // alone cannot say "this is a bridge"; and comparing a plain street's
    // reconciled profile against the raw terrain cannot either — a street
    // graded across a dip rides a fill the conform pass will carve the
    // ground up to. Only these two flags make a chain a bridge.
    bool authoredDeck = false;
    int  layer = 0;
    // WHAT RUNS ON IT: a road (traffic, parked cars, the street a walker is on), a TRACK or a FOOTWAY (a station's
    // way in). The last two are drawn decks with their own profile, but nothing on a street ever stands on them: the
    // deck field's queries skip them (they are not indexed), so a car under a viaduct never reads the rail above it
    // (Glenn, 2026-10-12: "cars weirdly stuck in the tracks"). The earthwork still rides them -- they are structure.
    enum class Use : uint8_t { Road = 0, Track = 1, Footway = 2 };
    Use use = Use::Road;
    // Where the TRAVEL lanes end, as a fraction of the half-width (1.0 = at
    // the kerb, no kerbside parking). A 12 m street with 2.5 m parking bays
    // carries travel lanes only out to 3.5 m of its 6 m half-width, so this is
    // 0.583 — and both the painted lane dividers and the white edge line
    // belong at THAT boundary, not at a share of the whole carriageway.
    double travelEdgeFrac = 1.0;
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
    // THE DRAWN SECTION (a lane-built deck, ADR-0223): absolute deck Y at the carriageway's LEFT and RIGHT edges
    // (left = fwd rotated +90 in XZ, as crossSlope's; parallel to `points`; EMPTY = level across, yAbs alone). The
    // lanes builder draws each lane at its own blend toward the roads it meets, so a carriageway is not level
    // across: one centreline height left 3364 of 151k metro samples 25 cm off the drawn pavement, this 139.
    std::vector<double> yLeft, yRight;
    // Deck Y on segment (i, i + 1) at parameter t, `lateral` metres to the left of the centreline: the profile,
    // the superelevation, and the drawn section lerped centre -> edge on that side (held at the edge beyond it).
    double sectionY(std::size_t i, double t, double lateral) const;
};
// (WeldSolidParams / weldSolid deleted — roads-v2 S6, one mesher.)

// THE DECK THE MESHER DREW, as a queryable field. The lattice rides each
// chain's reconciled, grade-limited profile (`yAbs` below); the terrain is
// carved 0.22 m UNDER that profile on purpose so the deck stays proud of the
// interpolating grid (road_net.cpp, "Carve a step BELOW"). Anything placed on
// a road by sampling the terrain therefore sat 0.22 m (minus a vestigial
// 0.08 "lift") inside the asphalt — every sim car, parked car, painted bay
// and signal lens (device: "the simulated cars ... sometimes they are below
// the road"). This field is the road surface itself, exported by
// the road builder from the very surface it built, so a consumer that asks it
// stands where the mesh is by construction.
struct RoadDeckField {
    std::vector<UnionSpine> spines;   // yAbs filled: the per-point deck Y the lattice rode
    // JUNCTION PADS, as the drawn triangles (upward faces only). A pad is an
    // earcut of its boundary loop with heights only at the loop — a chordal
    // surface between the mouths — while the chain profile through the node
    // curves; on a steep hill the two differ by the sagitta at the pad's
    // edge (metro traffic census: +0.83 m hover on a crest, -0.67 m sink in a
    // sag). Inside a pad the field answers from these, not the profile.
    struct Tri { Vec3 a, b, c; };
    std::vector<Tri> pads;
    // Deck Y at (x, z): the pad triangle under it if any, else the nearest
    // spine's lerped profile when that spine's centreline passes within
    // (half-width + margin). False off every road — the caller falls back
    // to its terrain.
    bool heightAt(double x, double z, double margin, double* outY) const;
    // The drawn surface at (x, z) for traffic that knows its own level roughly (`refY`, a lerp between nav
    // ends, #35): of the surfaces within `window` of refY (never a stacked level), the junction pad under
    // the point, else the road it is most inside in plan -- the ramp, not the freeway it runs beside.
    // False when no deck here lies within the window.
    bool heightNear(double x, double z, double margin, double refY, double window, double* outY) const;
    // heightNear, preferring spines of road class `*klass` (null: any): the road a car of that class is on.
    // Pass a huge window when the caller has no height to go by (a street car: its class picks the road).
    bool heightOn(double x, double z, double margin, const RoadClass* klass, double refY, double window,
                  double* outY) const;
    // How far INSIDE the built asphalt (x, z) lies, in metres: the distance in
    // from the nearest spine's edge — interior segments only, so a point past
    // a chain's end (a cul-de-sac cap) is outside — or, inside a junction
    // pad, the distance to that triangle's nearest edge (a lower bound).
    // 0 = not on the deck. The city map's "sidewalk cuts across the road"
    // census measures the band against THIS, not the graph.
    double depthInside(double x, double z, int* spineOut = nullptr) const;   // spineOut: -1 = a pad
    // Any drawn deck within half-width + margin of (x, z) -- tracks and footways included, which heightAt, heightOn
    // and depthInside leave out (they answer for things on a street): what a tree or a rock stays off.
    bool coversAny(double x, double z, double margin) const;
    // Spatial hash of pad triangles and spine segments; the builder
    // calls it after filling both (call again after editing by hand).
    void buildIndex();
    bool empty() const { return spines.empty() && pads.empty(); }

private:
    // Every drawn surface over (x, z): pad triangles (pad = true) and spines within half-width + margin
    // (their layer, centreline distance d, half-width hw and road class). heightAt and heightNear choose among these.
    // (`anyUse`: tracks and footways too; otherwise roads only)
    void surfacesAt(double x, double z, double margin,
                    const std::function<void(double y, bool pad, int layer, double d, double hw, RoadClass klass)>& fn,
                    bool anyUse = false) const;
    struct Seg { int spine; int i; };        // spine segment (i, i+1)
    std::unordered_map<long long, std::vector<Seg>> cells_;
    std::unordered_map<long long, std::vector<int>> padCells_;
    double cell_ = 24.0;   // >= widest half-width + sidewalk + margin: 3x3 covers
};

}  // namespace engine

#endif
