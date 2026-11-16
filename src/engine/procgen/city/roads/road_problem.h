#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_PROBLEM_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_PROBLEM_H

// A PROBLEM THE ROAD BUILD FOUND, AS A RECORD (ADR-0438; issue #292 "problems are prose"). Building the lane level
// reports where the roads do not come out as stated: a lane with a fold, a turn that fits no vehicle, two lanes joined
// across a gap. Each used to be a sentence written where it was found, so a tool or a test could only read the words.
// Now each is a record:
//   - its KIND, one of the list below (each kind has a name and a one-line meaning: problemKindRow);
//   - WHAT IT CONCERNS, by the road model's ids: junction, street, lane, road (their names), an arm (its place in its
//     junction's list), a lane path or connector of the lane level (its number);
//   - WHERE: a plan point, a station along a road's centre line;
//   - its NUMBERS, in the order its kind's sentence reads them (a gap in metres, an angle in degrees, a count);
//   - its WORDS: the parts of the sentence that are a choice, not an id or a number (which end of a street, "leaves" or
//     "merges into", the corner fit's own reason).
// THE SENTENCE is made from the record by one function, problemLine, from the kind's own template: nothing else writes
// a problem's words. The tools print those sentences as before.
//
// Covered so far: everything building the lane level reports (lanes/lane_graph_build.h LaneGraphBuild::problems, the
// crossing points' among them). The road model's own list (RoadModel::problems) and the paint's are still sentences.

#include "engine/procgen/city/polygon.h"             // Vec2, Real
#include "engine/procgen/city/roads/graph_index.h"   // kNoIndex
#include "engine/procgen/city/roads/road_model_parts.h"   // kNotStated

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace engine {

enum class RoadProblemKind : uint8_t {
    LaneNotBesideSpine,
    LaneFoldTidied,
    LaneNothingLeft,
    ArmNoStopLine,
    StreetEndNoStopLine,
    NoRoomBetweenStopLines,
    LaneOnNoStreet,
    StreetEndUnstated,
    CarryOnNotNamedBack,
    CarryOnAmbiguous,
    CarryOnNoLaneLeaves,
    CarryOnOtherSlot,
    CarryOnGap,
    ConnectionNotJoined,
    ConnectionGap,
    TurnNoJunction,
    JunctionNoLanePath,
    ArmWaysCancel,
    OutlineFewCorners,
    OutlineCrossesItself,
    SignOnArmNoLane,
    TurnNoLanePath,
    TurnFitsNoVehicle,
    TurnsNotAllPlaced,
    LaneHoleAtStations,
    LaneUncovered,
    LaneKink,
    ConnectorKink,
    JoinKink,
    MergeNotCarried,
    LaneGraphCheck,
    ManyCrossingPoints,
    PathsNearlyOverEachOther,
    CrossingOutsideJunction,
    Count_
};
constexpr std::size_t kRoadProblemKindCount = static_cast<std::size_t>(RoadProblemKind::Count_);

// ONE KIND: its name (what a tool prints a count against), what it means in one plain line, and its sentence. In the
// sentence, {junction} {street} {street2} {lane} {lane2} {road} {arm} {path} {path2} stand for the record's ids, {x:N}
// {y:N} {station:N} for its place and {n0:N}..{n9:N} for its numbers, each written with N decimals, and {w0}..{w3}
// for its words.
struct RoadProblemKindRow {
    RoadProblemKind kind;
    const char* name;
    const char* meaning;
    const char* line;
};
const RoadProblemKindRow& problemKindRow(RoadProblemKind kind);

struct RoadProblem {
    RoadProblemKind kind = RoadProblemKind::LaneGraphCheck;
    // what it concerns (empty, -1 or kNoIndex: not that)
    std::string junction, street, street2, lane, lane2, road;
    int arm = -1;
    uint32_t path = kNoIndex, path2 = kNoIndex;   // lane paths or connectors of the lane level
    // where (kNotStated: nowhere in particular)
    Vec2 at = Vec2(kNotStated, kNotStated);
    Real station = kNotStated;
    std::vector<Real> numbers;
    std::vector<std::string> words;

    RoadProblem() = default;
    explicit RoadProblem(RoadProblemKind k) : kind(k) {}
    // (each returns the record, so one is written in a line where it is found)
    RoadProblem& atJunction(std::string id) { junction = std::move(id); return *this; }
    RoadProblem& onStreet(std::string id) { street = std::move(id); return *this; }
    RoadProblem& andStreet(std::string id) { street2 = std::move(id); return *this; }
    RoadProblem& onLane(std::string id) { lane = std::move(id); return *this; }
    RoadProblem& andLane(std::string id) { lane2 = std::move(id); return *this; }
    RoadProblem& onRoad(std::string id) { road = std::move(id); return *this; }
    RoadProblem& onArm(uint32_t a) { arm = static_cast<int>(a); return *this; }
    RoadProblem& onPath(uint32_t p) { path = p; return *this; }
    RoadProblem& andPath(uint32_t p) { path2 = p; return *this; }
    RoadProblem& where(const Vec2& p) { at = p; return *this; }
    RoadProblem& atStation(Real s) { station = s; return *this; }
    RoadProblem& number(Real v) { numbers.push_back(v); return *this; }
    RoadProblem& word(std::string w) { words.push_back(std::move(w)); return *this; }
};

// THE SENTENCE OF A PROBLEM: its kind's line with the record's ids, place, numbers and words put in. The one function
// that writes a problem's words.
std::string problemLine(const RoadProblem& p);
// ...of each of a list, in order.
std::vector<std::string> problemLines(const std::vector<RoadProblem>& problems);
// HOW MANY OF EACH KIND a list holds: (kind, count) for each kind present, in the kinds' own order.
std::vector<std::pair<RoadProblemKind, uint32_t>> problemsByKind(const std::vector<RoadProblem>& problems);

}  // namespace engine

#endif
