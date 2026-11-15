#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_PARTS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_PARTS_H

// THE ROAD MODEL'S JUNCTIONS, ARMS AND STREETS (road_model.h says what the model is): ONE set of types. The road file is
// read into them (lanes/road_graph_spec.h RoadLabGraph::fromJson), the layout and the control plan write them
// (lanes/road_layout.h, lanes/control_plan.h), and the road model holds the same objects with what only it knows added
// (road_model.h plannedRoadModel, assembleRoadModel). Kept apart from road_model.h so the road file's reader need not
// include the lane graph's types.

#include "engine/procgen/city/polygon.h"             // Vec2, Real
#include "engine/procgen/city/road_network.h"        // RoadClass: the engine's class of a street
#include "engine/procgen/city/roads/graph_index.h"   // GraphIndex, Range, kNoIndex

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace engine {

struct ModelJunctionTag {};
struct ModelStreetTag {};
struct ModelLaneTag {};
using ModelJunctionId = GraphIndex<ModelJunctionTag>;
using ModelStreetId = GraphIndex<ModelStreetTag>;
using ModelLaneId = GraphIndex<ModelLaneTag>;

// No station or height stated.
constexpr Real kNotStated = std::numeric_limits<Real>::quiet_NaN();
inline bool stated(Real v) { return !std::isnan(v); }

// ONE ARM: a road's way out of a junction. THE ROAD FILE STATES IT (its junction's "arms": [{"road": "s4", "at": 87.3,
// "spine": "in", "regulations": ["stop"], "street": "s4/1", "heading": 90}]): the road, the station along its spine where
// it meets the junction ("at"), whether the spine runs INto the junction there (its forward lanes arrive) or OUT of it, the
// signs standing where it arrives, the street it is and the way it leaves. A road that passes through (a ramp ending on a
// street's side) is two arms, one each way. A junction spread over several nodes lists only the arms that lead out of it.
struct ModelArm {
    ModelStreetId street;            // the street this arm is, an index into the same list's streets (none: not stated)
    std::string road;                // the road-file edge
    Real station = 0;                // where the arm meets the junction, along the road's spine
    bool spineIn = true;             // the road's spine runs INTO the junction here (the street ends here)
    Real heading = kNotStated;       // degrees, the way the arm leaves the junction, plan coordinates
    std::vector<std::string> regulations;   // signs standing at the end of each lane arriving by it (the control plan)
    // where the junction's area begins and where traffic arriving by this arm stops, along its road's spine: the one rule
    // from the stated junction (deck_mesh.h junctionAreas, ADR-0428) -- what the paint draws and the lane level reads.
    // Not in the road file: the road model adds them (plannedRoadModel).
    Real boxEdge = kNotStated;
    Real stopLine = kNotStated;
};

// What a street is made of across its width (design section 1: the cross-section), from its road's class and the road
// file's lanes. The lanes each way are the TRAVEL lanes: the one lane count the builder lays and the city sim drives
// (ADR-0418). KERB PARKING is a band along each kerb, over the kerb side of the kerb lane each way (roads/lanes
// LaneLayout::parking): `parking` its width where the road states one that fits (streetSection), 0 none; `parkingOffset`
// where the band's middle lies, out from the carriageway's middle toward the kerb (the same reading streetSection gives
// the city sim's bays).
struct ModelSection {
    std::string roadClass;
    RoadClass klass = RoadClass::Local;        // the engine's class for it (roads/lanes classOf: the one mapping)
    int lanesForward = 0, lanesBackward = 0;   // forward: running along the spine
    Real laneWidth = 0, medianGap = 0, median = 0, shoulder = 0, sidewalk = 0;
    Real parking = 0;                          // the kerb parking band's width along each kerb (m); 0: none
    Real parkingOffset = 0;                    // its middle, from the carriageway's middle toward the kerb (m)
    bool track = false, footway = false;
};

// A STRETCH OF ROAD INSIDE A JUNCTION (between nodes the planner grouped into it; the road file's "inside": [{"road",
// "from", "to"}]), or one of the paint's boxes along a road: a road and stations from <= to. The road model adds to a
// stretch inside a junction what the street map needs to drive it (ADR-0425): its road's index, its centre line between
// the two stations (`line`, into RoadModel::linePoints, with their stations in RoadModel::lineStations), its road's
// cross-section and whether it is one-way. A paint box carries none of them.
struct ModelStretch {
    ModelStretch() = default;
    ModelStretch(std::string r, Real f, Real t) : road(std::move(r)), from(f), to(t) {}
    std::string road;
    Real from = 0, to = 0;
    uint32_t edge = kNoIndex;        // the road's index (inside stretches; the road model's)
    Range line;
    ModelSection section;
    bool oneWay = false;
};

// ONE POINT OF A JUNCTION SPREAD OVER SEVERAL (ADR-0425): where roads meet inside it -- the road ends that meet there, a
// road's station where another ends on its side, or where two roads cross at grade. Each of its points is a road and a
// station along that road's spine, written to the centimetre as the arms' are. The road file states them on the junction:
//   "nodes": [{"at": [x, y], "points": [{"road": "s21", "at": 90.89}, {"road": "s313", "at": 24.13}]}, ...]
// A junction with no stretch of road inside it is one point (its `at`) and lists none. A junction with stretches inside it
// lists every one of its points, so a reader joins its arms and its inside stretches by these stations, never by where
// they lie (the street map). Part of the layout: the bake key and the lane graph's key do not see it.
struct ModelNodePoint {
    std::string road;                // the road-file edge (empty in a model read back from its saved bytes)
    Real station = 0;
    uint32_t edge = kNoIndex;        // the road's index (the road model's)
};
struct ModelNode {
    Vec2 at;
    std::vector<ModelNodePoint> points;
};

// A JUNCTION. THE ROAD FILE STATES IT, in its one "junctions" list (control_plan.h writes the controls, road_layout.h the
// rest):
//   "junctions": [{"id": "j12", "at": [x, y], "control": "stop", "why": "a local street meets a collector",
//                  "arms": [...], "through": [[0, 2]], "inside": [...], "nodes": [...], "hand": true}]
// `control` (traffic lights, an all-way stop, stop or give-way signs on its minor roads, or nothing) and each arm's
// regulations are the city planner's, from the rule set's placement policy, written as INTENT: the builder carries them
// through and checks them, never works a control out ("roads: no guessing"). A list that carries no control at all is the
// layout alone; one where some junctions name a control and some do not is refused. A junction the planner spread over
// several nodes names the stretches of road lying between them as `inside`. The layout's members (each arm's street and
// heading, `through`, and the "streets" list) are not part of the city's bake key (road_layout.h keyedRoadFileText).
struct ModelJunction {
    std::string id;
    Vec2 at;
    Real height = kNotStated;        // the drawn deck at `at`, read off the builder's heights along its first arm's road (built layer)
    std::vector<ModelArm> arms;      // in the order the file lists them: their ring where it says so (ADR-0427)
    std::vector<std::array<uint32_t, 2>> through;   // pairs of arms (indices into arms) that are one street carrying on
    std::string control, why;        // empty control: the road file plans none
    bool hand = false;               // written by a person, so re-planning keeps it as it is (not in the saved bytes)
    std::vector<ModelStretch> inside;   // stretches of road inside it
    std::vector<ModelNode> nodes;       // its points, when stretches lie inside it (ADR-0425)
    std::vector<ModelStretch> boxes;    // its area along each road it lies on (deck_mesh.h junctionAreas, ADR-0428; the road model's)
};
// Is this junction's stated control one the signal poles stand for (road_attachments.json "signal_pole": lights)?
bool junctionLit(const ModelJunction& J);

// A STREET: a stretch of one road-file edge from one junction to the next, or to a dead end. THE ROAD FILE STATES IT:
//   "streets": [{"id": "s12/1", "road": "s12", "from": 40.2, "to": 131.7, "junctions": ["j3", "j9"],
//                "carries_on": [null, "s13"], "name": "Oak Street"}]
// An edge that passes through junctions is several streets; the stretches of it inside a junction belong to no street.
// WHERE A STREET ONLY CARRIES ON (ADR-0419): two road ends that meet with nothing else there (one road running on as
// another, or a bend) are no junction -- no junction area, no stop line -- and each of the two streets names the other at
// that end. ITS NAME (ADR-0420) is the planner's (or the one-off layout tool's); every street of one RUN (road_layout.h
// streetRuns) carries the same name. The file names junctions and streets by id; here they are places in the same list
// (the reader resolves them: RoadLabGraph::fromJson). Everything from `edge` down is the road model's (plannedRoadModel).
struct ModelStreet {
    std::string id, road;
    Real from = 0, to = 0;           // stations along the road's spine
    std::array<ModelJunctionId, 2> junctions;   // at `from`, at `to` (none: a dead end, or a ramp's anchored end)
    // at `from`, at `to`: the street this one carries on into where no junction is (one road running on as another, or a
    // bend: ADR-0419); none: a junction, a dead end or a ramp's anchored end
    std::array<ModelStreetId, 2> carriesOn;
    std::string name;                // as the road file states it (empty: none; not in the saved bytes -- the model's
                                     // readers ask streetName, the attachment made from it)
    uint32_t edge = kNoIndex;        // the road-file edge's index
    ModelSection section;
    bool oneWay = false;
    Real speedLimit = 0;             // metres per second; 0: none posted
    Real lotsFrom = -1, lotsTo = -1; // where buildings may front it (the road's lots_range); -1: all of it
    std::string frontageOf;          // a frontage road: the freeway carriageway it runs beside
    Range line;                      // into linePoints: its centre line, `from` to `to` (planned: the road file's spine)
    std::vector<ModelLaneId> lanes;  // its lanes, forward then backward, each by slot from the centre line (built layer)
    // The width the travel lanes take across, both ways (lanes x width + the median gap): what the map draws.
    Real travelWidth() const {
        return (section.lanesForward + section.lanesBackward) * section.laneWidth + section.medianGap;
    }
};

}  // namespace engine

#endif
