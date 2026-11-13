#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_DECK_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_DECK_MESH_H

// lanelab -> engine RenderMesh, through MeshBuilder's winding rule. World is (x, height, z):
// lanelab's plan Vec2 (x, y) lands on (x, z). Decks carry lane-local UVs (u = lateral metres
// from the owning lane's centreline, v = station along its parent spine).

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "renderer/renderer.h"
#include "engine/procgen/city/rail_track.h"
#include <map>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// The sidewalk slab's top above the lane deck — the kerb. Named so the lot
// pass's paving datum (block_audit.h lanesSidewalkRise) is the mesher's own number.
constexpr double kSidewalkLift = 0.12;

// ROAD PAINT on the drawn road (MeshBuilder::drapeOnMesh): every stripe, bar and arrow lies this far above the road
// triangle under it, so the depth test does not let the asphalt show through it. (The cure for that is a depth bias
// when paint is drawn; until the renderer has one, the lift stays.)
constexpr double kPaintLift = 0.02;
// ...and goes on the road surface nearest the height it was laid at, never on one further than this (a bridge over
// the street, or the street under a bridge).
constexpr double kPaintReach = 1.0;
// A paint triangle the road under it folds by less than this (metres off the plane through its corners) is kept whole
// rather than cut along the road's triangles: half a millimetre is far under what the 2 cm lift hides.
constexpr double kPaintFlush = 0.0005;

struct NamedMesh { std::string name; RenderMesh mesh; Vec3 color; };

// One mesh per material: asphalt (deck tops and bottoms), concrete (walls, piers), sidewalk,
// shoulder, median, paint white, paint yellow, terrain. Empty meshes are omitted.
std::vector<NamedMesh> buildMeshes(const Result& r);

// WHICH LANES' FOOTPRINTS HOLD A POINT, asked many times: every lane footprint's bounding box once, a coarse grid of
// them, and the footprints prepared for point tests. Built once per result by whoever asks (the mesher, the lane graph).
struct LaneLookup {
    std::vector<Box2> box;                 // per lane
    LaneGrid grid;
    std::vector<PreparedSet> footprint;    // per lane
    explicit LaneLookup(const Pavement& pavement);
};

// JUNCTION BOXES, as the road paint finds them: lane paint stops inside them, and each street approaching one gets a
// zebra crosswalk, lane arrows and (where it must stop) a stop bar. The lane graph cuts its lanes at the same boxes.
// Where an approach's marks stand, in metres back from the box along the approaching street's spine: the crosswalk
// from kCrosswalkNear to kCrosswalkFar, the stop line at kStopLineBack (its bar is painted only where the street
// stops for the crossing road; the line is where every approach ends).
constexpr double kCrosswalkNear = 1.0;
constexpr double kCrosswalkFar = 4.0;
constexpr double kStopLineBack = 5.2;
// Two stretches of one street inside crossing roads closer than this along it are ONE junction: a divided cross road's
// carriageways sit a median apart.
constexpr double kJunctionMergeGap = 12.0;

// (JunctionSpan, one junction box along a street, is in lanes.h: the build keeps the paint's boxes in its Result.)
// The paint's PARALLEL TEST: a crossing lane whose direction there is within this (|cos| of the angle between them) of
// the street's runs beside it, not across it (0.7: closer than about 46 degrees). The paint's number; a caller with its
// own reason passes another (the lane graph, lane_graph_build.h kJunctionParallelForGraph).
constexpr double kJunctionParallel = 0.7;
// Whether a point on street lane `li` (heading `t` there) is inside an intersection: inside a CROSSING road's lane at
// the same level. Parallel overlaps (a gore, an aux lane beside its host) and grade-separated ones (a street under a
// viaduct) are not. Returns the crossing road's rank, or -1; `crossEdges` (optional) gets the crossing lanes' roads.
// `parallelCos`: the parallel test's threshold (above: beside, not across).
int junctionCrossRank(const Result& r, const LaneLookup& look, size_t li, const Vec2& p, const Vec2& t,
                      std::vector<int>* crossEdges = nullptr, double parallelCos = kJunctionParallel);
// Whether the paint finds junction boxes along a road of its own: streets only -- not ramps, freeways, tracks, nor classes
// ranked below 1.
bool paintsJunctionBoxes(const RoadLabGraph& g, const EdgeSpec& e);
// Every street's junction boxes (graph edge index -> its boxes, in station order, a divided cross road's two merged
// into one). Streets only (paintsJunctionBoxes).
std::map<int, std::vector<JunctionSpan>> junctionSpans(const Result& r, const LaneLookup& look,
                                                       double parallelCos = kJunctionParallel);

// Every track lane's railhead line, in running order, with the platforms beside it (rail_track.h).
std::vector<RailTrack> railTracks(const Result& r);

// LANE ARROWS: what each of an approach's lanes may do at a junction, given the moves the junction
// offers (hasLeft/Straight/Right, in plan terms) and whether its leftmost / rightmost lane is a turn
// pocket. Lanes are listed plan-left first. The usual assignment: the leftmost lane turns left (and
// goes straight unless it is a pocket), the rightmost turns right (likewise), the middle ones go
// straight -- or, with no straight on offer, split left and right. buildMeshes paints these.
struct LaneMoves { bool left = false, straight = false, right = false; };
std::vector<LaneMoves> assignLaneMoves(std::size_t lanes, bool hasLeft, bool hasStraight, bool hasRight,
                                       bool leftPocket = false, bool rightPocket = false);

// The parapet runs the mesher sweeps: a chain of deck-boundary points on an elevated freeway or ramp lane.
// Exposed so a diagnostic can ask where a wall ENDS — a run whose cap sits on same-level pavement is a wall
// standing in the middle of drivable road ("the freeway walls end up blocking the car", Glenn, 2026-09-07).
// What a freeway or ramp deck edge FACES (2026-09-08). It began as one geometric rule — a 0.9 m parapet
// wherever the deck stood 1.5 m clear of the ground — which left an at-grade freeway bare. Naming the role
// is the edge grammar: the class table says what to build for each (barrierFor in deck_mesh.cpp; Glenn,
// 2026-09-08: "if it's a lane on the interior or exterior edge it should build some kind of wall/barrier/fence").
// Freeway and ramp classes by default, plus any class that authors a barrier; streets are the lot pass's business.
enum class EdgeRole : uint8_t {
    Seam = 0,    // same-level pavement just beyond: not an edge of the structure at all, never walled
    Median,      // faces the other carriageway of the same freeway across the median
    VsStreet,    // faces a street, collector or arterial within reach — the freeway/city separation
    Elevated,    // outward edge of a deck standing clear of the ground: structure, needs a parapet
    AtGrade,     // outward edge sitting on the ground with nothing beyond: a guardrail by default (ADR-0220)
    Count_
};
const char* edgeRoleName(EdgeRole role);
// Metres of freeway/ramp deck edge per role, and how much of it carries a barrier today.
struct ParapetCensus { double metres[static_cast<size_t>(EdgeRole::Count_)] = {}; double built[static_cast<size_t>(EdgeRole::Count_)] = {}; };

struct ParapetRun {
    std::vector<Vec2> pts;
    std::vector<double> z, offOuter, offInner;
    int lane = -1;          // the owning lane of the run's first edge
    bool closed = false;
    bool capOnPavement[2] = {false, false};   // start cap, end cap: same-level pavement just beyond it
    EdgeRole role = EdgeRole::Elevated;       // the role of the run's first edge
    BarrierKind kind = BarrierKind::Wall;     // what to build: one kind per run
    double height = 0.9;
};
std::vector<ParapetRun> parapetRuns(const Result& r, ParapetCensus* census = nullptr);

}  // namespace roads::lanes
}  // namespace engine

#endif
