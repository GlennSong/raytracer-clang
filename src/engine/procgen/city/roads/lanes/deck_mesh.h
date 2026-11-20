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
// `paint` (optional, out): what the paint decided (RoadPaint, below) -- the list the lines, arrows and stop bars are drawn from.
struct RoadPaint;
std::vector<NamedMesh> buildMeshes(const Result& r, RoadPaint* paint = nullptr);

// WHICH LANES' FOOTPRINTS HOLD A POINT, asked many times: every lane footprint's bounding box once, a coarse grid of
// them, and the footprints prepared for point tests. Built once per result by whoever asks (the mesher, the lane graph).
struct LaneLookup {
    std::vector<Box2> box;                 // per lane
    LaneGrid grid;
    std::vector<PreparedSet> footprint;    // per lane
    explicit LaneLookup(const Pavement& pavement);
};

// THE STATED JUNCTIONS' AREAS AND STOP LINES (step 9, ADR-0428) -- THE ONE PLACE a stop line is decided. Each arm of each
// junction the road file states (RoadLabGraph::junctions; build() derives the layout once, first, for a file that states
// none) has, along its road's spine:
//   - its MOUTH: the stated station `at`;
//   - its BOX EDGE, where the junction's area begins, and its STOP LINE, where traffic arriving by it stops: back from
//     the mouth as the one rule says (road_model_parts.h boxEdgeBack, stopLineBack: the widest crossing road's half
//     width, or where they meet at a sharp angle until the arm's kerbs clear the crossing roads, within the room the
//     next junction facing it along its road leaves (ADR-0471); then kStopLineBack).
// A crosswalk's band lies kCrosswalkNear to kCrosswalkFar back from an arm's box edge: the ONE source of those two numbers,
// read only by the road model's one function that states the crosswalks (road_model.cpp stateCrosswalks, ADR-0452); the
// paint draws each marked crosswalk from the model's record, with its lane arrows and stop bar where the road model states
// them (RoadPaint, below); the road model's ModelArm::boxEdge and ::stopLine ARE these; the lane level ends its lanes at
// these. Nothing finds or groups a junction from overlapping lanes.
constexpr double kCrosswalkNear = 1.0;
constexpr double kCrosswalkFar = 4.0;
// One arm of a stated junction, as the paint, the model and the lane level read it.
struct ArmStop {
    int edge = -1;              // its road (graph edge index; -1: a road the graph does not have)
    double station = 0;         // its mouth, along the road's spine
    bool spineIn = true;        // the spine runs into the junction here (the approach lies at lower stations)
    double boxEdge = 0;         // where the junction's area begins, along the spine
    double stopLine = 0;        // where traffic arriving by it stops, along the spine
    bool painted = false;       // its road is a street the paint marks (paintsJunctionBoxes)
    // THE PLANNING FAULT (ADR-0471, road_model_parts.h boxEdgeSqueeze): metres short of where its kerbs clear the roads
    // crossing it that the room to `facing` (the next stated junction along its road, facing it; -1: none) leaves its box
    // edge -- two junctions too close for the angle their roads meet at. 0: clear.
    double squeeze = 0;
    int facing = -1;
};
// One stated junction's arms (in their list order) and its area along each road it lies on (JunctionSpan: from the
// lowest to the highest of its arms' box edges and mouths on that road, and any stretch the file states inside it).
struct JunctionArea {
    std::vector<ArmStop> arms;
    std::vector<JunctionSpan> spans;
};
// Every stated junction's, in the graph's list order. `g` with its ramps composed (an arm's station is a composed one).
std::vector<JunctionArea> junctionAreas(const RoadLabGraph& g);
// The paint's PARALLEL TEST: a crossing lane whose direction there is within this (|cos| of the angle between them) of
// the street's runs beside it, not across it (0.7: closer than about 46 degrees). The paint's number. (The ramp-landing
// check has its own, looser one: lane_graph_build.h kJunctionParallelForGraph.)
constexpr double kJunctionParallel = 0.7;
// Whether a point on street lane `li` (heading `t` there) is inside an intersection: inside a CROSSING road's lane at
// the same level. Parallel overlaps (a gore, an aux lane beside its host) and grade-separated ones (a street under a
// viaduct) are not. Returns the crossing road's rank, or -1; `crossEdges` (optional) gets the crossing lanes' roads.
// `parallelCos`: the parallel test's threshold (above: beside, not across).
int junctionCrossRank(const Result& r, const LaneLookup& look, size_t li, const Vec2& p, const Vec2& t,
                      std::vector<int>* crossEdges = nullptr, double parallelCos = kJunctionParallel);
// Whether the paint marks a road's junction approaches (crosswalk, arrows, stop bar): what its class states
// (road_graph_spec.h RoadClassSpec::junctionMarkings, "junction_markings" in a road file's `classes`), and never an
// anchored road's (a ramp's end is a gore). No class name or rank is asked (issue #292).
bool paintsJunctionBoxes(const RoadLabGraph& g, const EdgeSpec& e);

// Every track lane's railhead line, in running order, with the platforms beside it (rail_track.h).
std::vector<RailTrack> railTracks(const Result& r);

// WHAT THE PAINT SAYS ABOUT THE RULES OF THE ROAD, AS A LIST (what, where) that buildMeshes draws from and a test
// reads. Every entry is read off the built road model of the same build (road_model.h builtRoadModel), so the paint
// cannot say something the model does not:
//   - a LINE along a lane's edge. Between two lanes it is Dashed where the lane table lets it be crossed both ways and
//     Solid where not (lane_turns.h laneLineBetween, the rule the lane level fills its LaneLine from), drawn only where
//     both lanes run along a stated street outside a junction's stop lines; the road's Edge line where no lane lies
//     beyond; the Centre line of a two-way road with no median stated;
//   - an ARROW in a lane arriving at a junction: the kinds of turn RoadModel::turns states from that lane there, laid
//     out as the arrows' marking row of road_attachments.json says (when it is painted, how far before the stop line);
//   - a STOP BAR across the lanes arriving by an arm: a marking row of road_attachments.json holding for the arm's stated
//     signs and its junction's stated control (control_signs.h markingsOnArm).
// Stations are along the lane's road's spine. Lanes are indices into Result::lanes; junctions and arms into the road
// file's list (RoadLabGraph::junctions).
enum class PaintedLineKind : uint8_t { None, Edge, Centre, Dashed, Solid };
struct PaintedLine {
    int lane = -1;              // the lane whose edge it is drawn along
    int other = -1;             // the lane beyond it, toward the kerb (Dashed, Solid); -1: none
    bool kerbSide = true;       // along the lane's kerb-side edge (false: its centre-line-side edge)
    PaintedLineKind kind = PaintedLineKind::None;
    double from = 0, to = 0;    // from <= to
    double metres = 0;          // its drawn length
};
struct PaintedArrow {
    int junction = -1, arm = -1, lane = -1;
    bool straight = false, nearTurn = false, farTurn = false;   // near turn: toward the kerb; far turn: across the oncoming traffic
    double station = 0;         // its tail
    Vec2 at, heading;           // its tail in plan, and the way it points (the lane's direction of travel there)
};
struct PaintedStopBar {
    int junction = -1, arm = -1;
    std::string marking;        // the row that called for it (its kind in road_attachments.json)
    double station = 0;         // the arm's stop line
    std::vector<int> lanes;     // the arriving lanes it lies across
    Vec2 from, to;              // its two ends in plan
    int fromLane = -1, toLane = -1;   // the lanes its ends lie in (for their heights)
};
// A ZEBRA painted across an arm (ADR-0452): the model's crosswalk it is the paint of, and where its stripes run across
// the road -- from the first stripe's middle to the last's, at the band's middle (the paint's own measure of the lanes'
// width there).
struct PaintedZebra {
    uint32_t crosswalk = kNoIndex;   // the road model's
    Vec2 from, to;
    int stripes = 0;
};
struct RoadPaint {
    std::vector<PaintedLine> lines;
    std::vector<PaintedArrow> arrows;
    std::vector<PaintedStopBar> stopBars;
    std::vector<PaintedZebra> zebras;
    std::vector<std::string> problems;   // what was not painted because nothing states it, once per kind of gap
};
// WHAT THE PAINT READS BESIDES THE ROADS THEMSELVES, AS ONE NUMBER FOR THE BAKE KEY (city_producer.cpp): of a road file
// (`spec`, its JSON) each junction's control, arms (with their streets, headings and signs) and through pairs, the mark
// that the arms are in ring order, the streets (without their names) and the lane rules; and of the data tables, the
// turn table (assets/data/lane_turns.json, its bytes' hash) and the marking rows and control names of
// assets/data/road_attachments.json. Changing any of them re-bakes the city, as the paint is baked into its meshes.
uint64_t paintFactsKey(const nlohmann::json& spec);
// The shapes a marking row can ask for with "draws" -- the ones the paint has: "bar", a bar across the arriving lanes at
// the stop line; "turn_arrows", in each arriving lane the turns the road model states from it.
constexpr const char* kPaintDrawsBar = "bar";
constexpr const char* kPaintDrawsTurnArrows = "turn_arrows";
// A marking is painted no nearer than this to either end of its road (metres): a crosswalk or an arrow that would lie
// nearer is left out (the road ends inside the junction's area).
constexpr double kPaintRoadEndMargin = 2.0;

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
