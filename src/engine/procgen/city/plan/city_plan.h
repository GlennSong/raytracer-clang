#ifndef RAYTRACER_ENGINE_CITY_PLAN_H
#define RAYTRACER_ENGINE_CITY_PLAN_H

// THE CITY PLAN: design the city as DATA, score it, and only then build it
// (Glenn, 2026-09-22: "we need some way to design the city we want and build a
// road graph and block layout and evaluate it as data before it gets built by
// the city generator"; "Can you build maps before trying to construct anything
// so we can see potential city layouts?").
//
//   Brief (JSON, a few numbers)  ->  generatePlan  ->  CityPlan (roads + blocks)
//                                                        |
//                                   evaluatePlan (the engine's own parcel walk
//                                   and router)  ->  PlanScore  ->  map + report
//
// The core of the city is a GRID (rectilinear blocks, denser toward the centre);
// midtown keeps the grid but lets it warp; the outskirts are curving ring roads,
// radial spokes and wedge blocks between them. A freeway ring with radial spurs
// and interchanges runs through it. Every block gets a USE — parcelled into lots,
// one landmark on the whole block, or a park — decided here, as data, where it
// can be seen and changed, instead of inside the lot pass.
//
// Nothing here meshes, grades terrain or grows a building: a plan evaluates in
// seconds so layouts can be compared before any of that is paid for.

#include "../polygon.h"
#include "../road_network.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace engine {
namespace plan {

struct Brief {
    std::string name = "city";
    uint32_t seed = 1;
    Real size = 3000;                  // side of the square map (m)
    Vec2 center{0, 0};
    // The CORE grid: block size (along u, along v) at the centre, the grid's angle.
    Real coreRadius = 450;
    Real coreBlockU = 110, coreBlockV = 75;
    Real gridAngleDeg = 12;
    int arterialEvery = 3;             // every Nth grid line is an arterial
    // MIDTOWN: the same grid, blocks growing to this size at its rim, warped.
    Real midRadius = 800;
    Real midBlockU = 140, midBlockV = 95;
    Real warp = 30;                    // metres of displacement at the rim
    // OUTSKIRTS: ring roads, radial spokes, wedge blocks between them.
    Real ringSpacing = 170;
    int spokes = 12;
    Real curvature = 45;               // ring/spoke meander (m)
    Real wedgeStreetSpacing = 95;      // local streets across each ring band
    Real outerMargin = 120;            // keep the city this far inside the map edge
    // FREEWAY: a ring, radial spurs toward downtown, interchanges on arterials.
    Real freewayRadius = 1000;
    int freewayRadials = 2;
    Real freewayWobble = 50;
    // GROUND. The plan is drawn flat — blocks, lots and routes are plan-view — but the city is
    // built on this: rolling relief, peak to trough, over the map. Roads conform to it within
    // their class grades and lots grade their pads off it.
    Real relief = 18;
    // THE WORLD AROUND IT (optional): a level `terrain` block as the ground under everything —
    // the tilt toward a coast, the mountain range — with the relief above laid over it inside the
    // city and faded out before the edge of the city's ground grid, so the grid and the level's own
    // terrain beyond it are one surface. {"base", "seaLevel", "grid", "reliefFade", "calm", "water"};
    // `city_plan level-world` prints the level blocks that must match it. Null: the old flat world.
    nlohmann::json world;
    // Road widths (carriageway) and the sidewalk every street carries (m).
    Real localWidth = 12, collectorWidth = 16, arterialWidth = 22, freewayWidth = 30;
    Real sidewalk = 5;
};

Brief briefFromJson(const nlohmann::json& j);
nlohmann::json briefToJson(const Brief& b);
// Variant `k` of a brief: k = 0 is the brief itself; others spread the grid angle,
// block size, spokes, curvature and freeway around it (deterministic in k).
Brief variantOf(const Brief& b, int k);

enum class BlockUse : uint8_t { Lots, Landmark, Park, RightOfWay };
const char* blockUseName(BlockUse u);

struct PlanBlock {
    Poly2 face;              // the street-centreline face
    Poly2 buildable;         // inset by each street's half-width + sidewalk
    int district = 0;        // 0 core, 1 midtown, 2 outskirts
    BlockUse use = BlockUse::Lots;
    // Scores (evaluatePlan).
    Real area = 0;           // buildable area (m²)
    Real rectangularity = 0; // area / oriented-box area (1 = a rectangle)
    Real narrow = 0;         // oriented-box short side (m)
    int edges = 0;           // corners of the simplified buildable polygon
    int lots = 0;            // a dry-run of the engine's parcel walk at the district grain
    int predictedBuildings = 0;
};

struct CityPlan {
    Brief brief;
    RoadGraph streets;                 // local / collector / arterial, planar
    RoadGraph freeway;                 // ring + radials (grade-separated from streets)
    std::vector<Vec2> ring;            // the freeway ring's centreline, closed (empty: no ring)
    std::vector<std::vector<Vec2>> limits;   // world.land.shape: the city's limits, grown over its land (closed outlines)
    std::vector<Vec2> ringArc;         // world.land: the ring opened into a C where it met the sea (ring is then empty)
    std::vector<std::vector<Vec2>> spurs;   // the radial expressways, drawn from their far end in to midtown's boulevard
    // A TOWN at an expressway's far end, one per spur: a small grid square to the expressway, whose
    // main street crosses it at the GATE — where the expressway's boulevard pair meets it, as the
    // inner end meets midtown's boulevard. Not built where no site out there stands clear of the sea.
    struct Town { bool built = false; Vec2 gate, axis, centre; };
    std::vector<Town> towns;
    // THE OUTER LOOP (optional, brief.world.loop): a freeway that carries the two expressways on
    // round the mountain side of the city, following the mountain front — so the route is one
    // road, city -> spur `loopFrom` outward -> loop -> spur `loopTo` inward -> city, with no
    // freeway-to-freeway junction. `loop` runs from the first spur's outer end to the second's.
    // PLACES stand along it: strip towns laid out in its frame, their main street on the city
    // side and a back road on the mountain side, cross streets under the (elevated) freeway.
    std::vector<Vec2> loop;
    int loopFrom = -1, loopTo = -1;
    // `hubs` are district hubs for the level: (at, kind). A strip place has one; an organic town has
    // its gridded centre and its curvy residential cells.
    struct Place { std::string name, kind; double s0 = 0, s1 = 0; Vec2 hub; std::vector<std::pair<Vec2, std::string>> hubs; };
    std::vector<Place> places;
    std::vector<Vec2> interchanges;
    std::vector<PlanBlock> blocks;
};

struct PlanScore {
    int blocks = 0, lotBlocks = 0, landmarkBlocks = 0, parkBlocks = 0, rowBlocks = 0;
    int predictedBuildings = 0;
    int lots = 0;
    Real rectilinearShare = 0;         // share of buildable area in blocks with rectangularity >= 0.85
    Real coreRectilinearShare = 0;     // the same, core + midtown only
    Real gridRectilinearShare = 0;     // the same, the CORE grid alone (midtown is allowed to warp)
    Real meanLotsPerLotBlock = 0;
    int streetComponents = 0;          // 1 = one connected street network
    // Roads whose CORRIDORS (carriageway + sidewalk) overlap: two streets drawn so close
    // that the builder paves one over the other. The lanes builder reports it as lanes that
    // do not own their footprint; it is a design fault and belongs here, before anything is
    // built. Pairs that share a junction are not counted — that is what a junction is.
    int corridorOverlaps = 0;
    Real worstOverlap = 0;             // deepest intrusion (m)
    // STREETS THAT MERGE INTO A MESS (2026-10-09, Glenn: "many roads that merged together"). The corridor test
    // above counts only roads sharing a line; these catch the rest -- a street ending inside another's roadway
    // without joining it, a junction that collected half a dozen streets, two streets leaving one a few degrees
    // apart. Streets only, measured on the graph that is handed to the builder.
    int worstJunction = 0;             // most streets at one junction
    int crowdedJunctions = 0;          // junctions of 5 or more streets
    int tightTurns = 0;                // pairs of streets leaving a junction under 15 degrees apart
    Real pavedOverlapM = 0;            // roadway inside another road's roadway, away from where the two meet (m)
    std::vector<Vec2> messAt;          // the worst places (most overlap, then the most crowded junctions), up to 5
    int commutesSampled = 0;
    Real freewayCommuteShare = 0;      // of sampled outskirts->downtown commutes routed on the freeway
    Real streetKm = 0, freewayKm = 0;
    std::vector<std::string> notes;
};

// A ROAD, not a graph edge: the longest run of one class between junctions. A curve the
// planner laid down as forty short edges is one chain — which is what the builder has to be
// handed (forty stubs is what the frontage walk rejected), and what "two roads overlap"
// has to be measured between (consecutive edges of one curve are always within a lane of
// each other, and that is not an overlap).
struct PlanChain {
    std::vector<Vec2> pts;
    std::vector<int> nodes;            // the graph nodes along it
    RoadClass klass = RoadClass::Local;
    Real width = 12;
};
std::vector<PlanChain> chainsOf(const RoadGraph& g);

CityPlan generatePlan(const Brief& brief);
PlanScore evaluatePlan(CityPlan& plan);

nlohmann::json planToJson(const CityPlan& plan, const PlanScore& score);
std::string planToSvg(const CityPlan& plan, const PlanScore& score);

}  // namespace plan
}  // namespace engine

#endif
