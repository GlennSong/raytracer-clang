#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_PAVEMENT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_PAVEMENT_H

// The 2-D pavement and its 3-D surfaces (ADR-0083). Footprints: one polygon per lane.
// Closing fills gores and kerb returns only where two same-level footprints actually meet.
// Layers (shoulder, sidewalk, median) are ring differences around the roads. Then ONE
// constrained Delaunay over every lane outline: each triangle is classified by the lanes
// that cover its centroid, those lanes cluster by height into LEVELS (one surface per level:
// an overpass is two levels, a merge is one), each level's owner is its highest-ranked lane,
// and vertices weld across triangles by plan position and height into connected surfaces.

#include "engine/procgen/city/roads/lanes/deck_height.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include <array>
#include <functional>
#include <map>
#include <vector>

namespace engine {
namespace roads::lanes {

struct DeckVertex { Vec2 xy; double z; };

struct Surface {
    std::vector<DeckVertex> verts;
    std::vector<std::array<int, 3>> tris;   // CCW in plan
    std::vector<int> triOwner;              // lane index per triangle
    std::vector<std::pair<int, int>> boundary;   // edges used by exactly one triangle, in CCW triangle order (interior on the left)
    std::vector<int> owners;                // distinct lanes
    double area = 0, thick = 0.5;
    int nonManifold = 0, cracks = 0, boundaryOdd = 0;
    struct CrackSample { Vec2 xy; double dz; std::vector<int> owners; };
    std::vector<CrackSample> crackSamples;   // the worst few, for diagnosis
};

struct PairStats { double sameLevelArea = 0, separatedArea = 0, dz = 0; double minDz = 1e9; Vec2 minAt; };   // minDz: smallest deck-to-deck gap over the SEPARATED triangles

struct Pavement {
    std::vector<PolySet> footprints;        // per lane, after closing pieces were merged in
    PolySet surface, shoulder, sidewalk, median;
    std::vector<std::vector<int>> partners; // per lane
    std::map<std::pair<int, int>, PairStats> pairs;
    std::vector<Surface> decks;
    std::vector<double> islands;            // holes 1..2000 m²
    int enclosedBlocks = 0, triangles = 0;
    double closingAdded = 0;
};

// A KINK (ADR-0472): two road ends the road file states meet at one place -- a street carrying on as another, or two
// roads ending at the same junction point or spread-junction node -- at the same level (within the rules' sameLevelDz),
// with nothing between them on one side, which is the outside of the bend. Each road's bands (asphalt, shoulder,
// sidewalk) are drawn from its own spine and end square there, so on the outside of the bend they leave a wedge of bare
// ground; the build closes it with a MITRE: each band runs on past its end to the line that halves the bend (the
// bisector), where it meets the other's. Which ends meet is read only from the file's statements (ModelStreet::carriesOn,
// ModelJunction::arms and nodes), never found by searching for nearby ends. `edgeA`/`edgeB` index the graph's edges;
// an end is 0 at the spine's first point and 1 at its last; B lies counter-clockwise of A across the open side.
// `turn`: how far the road turns there, in radians (0: straight on, pi: back on itself).
struct BandKink {
    int edgeA = -1, endA = 0, edgeB = -1, endB = 0;
    double turn = 0;
};
std::vector<BandKink> bandKinks(const RoadLabGraph& g);
// A mitre's point reaches at most this many times the wider band's half width out from where the two ends meet (it does
// past a turn of 120 degrees); beyond that it is cut off square across the bisector, so a hairpin's outside is not drawn
// out to a spike.
constexpr double kMitreLimit = 2.0;
// One layer's mitre at a kink, as two pieces: A's (from its end to the bisector) and B's, each band `halfWidthA` /
// `halfWidthB` out from its spine. Past a turn of 120 degrees (kMitreLimit) the point is cut off square across the
// bisector, twice the wider half width out from where the ends meet. Where the two bands' corners on the open side are
// already one point (a straight carry-on) both pieces are empty, and nothing is added.
std::array<Ring, 2> mitrePieces(const RoadLabGraph& g, const BandKink& k, double halfWidthA, double halfWidthB);

// Footprints + closing + layers (needs the lanes' own heights for the same-level test).
void buildFootprints(const RoadLabGraph& g, const LaneSet& L, const DeckHeight& H, Pavement& out);
// The arrangement, levels, partners and welded surfaces. Sets H's partners as a side effect.
// `threads` 0 = default; `progress` (0..1 of this pass, on the calling thread; false = stop) drives a build's bar.
void buildSurfaces(const RoadLabGraph& g, const LaneSet& L, DeckHeight& H, Pavement& out, unsigned threads = 0, const std::function<bool(double)>* progress = nullptr);

// Triangulate one polygon (with holes) of a layer; z from the nearest contributing road.
struct FlatMesh { std::vector<DeckVertex> verts; std::vector<std::array<int, 3>> tris; std::vector<std::pair<int, int>> boundary; };
// `spanning`: centroids of layer triangles that bridged two LEVELS (an elevated ramp's shoulder and the
// ground street's, one 2D polygon) and were dropped — a layer is a 2D ring set and cannot hold two levels;
// the drop leaves a gap under the upper deck's edge instead of a wall-sided block (Glenn's ramp cube).
// `band`: this layer's width per edge (layerBands): a vertex stands at the level of the road whose band of THIS
// layer reaches it, not of whatever lane is nearest in plan (a deck overhead).
std::vector<FlatMesh> layerMeshes(const RoadLabGraph& g, const DeckHeight& H, const PolySet& layer, const std::vector<int>& roads, std::vector<Vec2>* spanning = nullptr, const std::vector<double>* band = nullptr);
// Lane bounding boxes binned on a coarse grid: a point tests only the lanes whose box covers its cell, in
// ascending lane order (so "first lane found" answers match a full scan). Shared by the cover pass, the
// mesher's seam/junction/paint queries and the surface-step audit.
struct LaneGrid {
    double x0 = 0, y0 = 0, cell = 16; int nx = 1, ny = 1; std::vector<std::vector<int>> cells;
    LaneGrid(const std::vector<Box2>& boxes, const std::vector<PolySet>& footprints);
    const std::vector<int>& at(const Vec2& p) const;
};
std::vector<Box2> laneBoxes(const std::vector<PolySet>& footprints);   // bounds() of every footprint, once

// The roads a layer may borrow heights from: those with the layer's ring (sidewalk/shoulder/median > 0), or every paved road.
std::vector<int> layerRoads(const RoadLabGraph& g, bool anyRoad);
// Each edge's width of one layer (0 where it has none): the reach of its sidewalk, shoulder or median band.
enum LayerKind { kSidewalkLayer = 0, kShoulderLayer = 1, kMedianLayer = 2 };
std::vector<double> layerBands(const RoadLabGraph& g, LayerKind kind);

}  // namespace roads::lanes
}  // namespace engine

#endif
