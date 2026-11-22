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

// "THESE TWO STATIONS ARE THE SAME PLACE" (ADR-0432): two stations along one road, each read from a road file or worked
// out from one, are one place when they are within what the file can say -- it writes stations to the centimetre
// (rt_math.h kWrittenStation). The one answer for every join made by station: an arm and its street's end, a street's
// end and a junction's point, a re-planned arm and the listed one. (Stated facts on one road are never this close: the
// shortest street is half a metre.)
inline bool sameWrittenStation(Real a, Real b) { return approxEqual(a, b, kWrittenStation); }

// THE FACTS OF ONE LANE, DECLARED ONCE (ADR-0435). A lane exists in two forms: the builder's lane while a road is being
// laid and drawn (lanes/lane_expand.h Lane: these facts plus its edges and working data) and the road
// model's lane, the record kept (road_model.h ModelLane: these facts plus where it lies on the streets). Both ARE this
// struct, so a fact has one name and one type, and the model takes the builder's whole (road_model.cpp builtRoadModel),
// never field by field. Readers that have the model read the model's.
//
// THE LANES' CENTRE LINES, HELD ONCE (ADR-0438): every lane's points one after another, and the distance along its own
// lane to each. A lane names its stretch of these arrays (LaneFacts::curve). The road build makes the object as it lays
// the lanes (lanes/lane_expand.h expand) and the road model made from that build holds THE SAME OBJECT
// (RoadModel::curves), not a copy: the builder's lane and the model's lane read one set of points. Nothing changes it
// once the lanes are laid. A model read back from its saved bytes holds its own.
struct LaneCurves {
    std::vector<Vec2> points;
    std::vector<Real> along;
};
struct LaneFacts {
    std::string id, kind;            // kind: through | turn | slip | aux | ramp | connector
    uint32_t edge = kNoIndex;        // its road, an index into the road file's edges (none: a connector)
    int dir = 1;                     // +1 along its road's spine, -1 against it
    // Its place across its road, counted from the centre-line side to the kerb side among the lanes running its way
    // (f0 / b0 is 0); a turn pocket born on the kerb side is one beyond the kerb lane, one born on the centre-line
    // side is -1; a ramp's lanes 0 and 1; a connector -1.
    int slot = -1;
    Real width = 0;                  // metres
    Real spineFrom = 0, spineTo = 0; // the stretch of its road's spine it is laid along (stations, from <= to)
    Range curve;                     // its centre line, the way it is laid (spine order): its stretch of LaneCurves

    bool onRoad() const { return edge != kNoIndex; }
    // Its road's index as a signed number, -1 for none: for code that keeps road indices as int.
    int road() const { return onRoad() ? static_cast<int>(edge) : -1; }
};

// THE WAY A STATED HEADING POINTS: a heading in degrees counter-clockwise from +x in plan coordinates (an arm's
// `heading`, as the layout writes it: lanes/control_plan.h degreesOf), as a unit vector. The one reading of a heading.
inline Vec2 directionOf(Real degrees) { return Vec2(std::cos(degrees * DEG_TO_RAD), std::sin(degrees * DEG_TO_RAD)); }

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
//
// THE QUESTIONS EVERY READER ASKS OF A CROSS-SECTION are answered here once (ADR-0432), over the lanes each way:
//   ONE-WAY: its traffic runs one way only, whichever way along the spine (lanes one way and none the other). WHICH way
//     is a separate fact: `lanesArriving` at an end of the road.
//   THE TRAVEL WIDTH: every lane's width, and the median gap where the road has lanes both ways (the gap lies BETWEEN the
//     two ways, so a one-way road has none) -- what the builder lays (roads/lanes bandsFor), and so what is drawn.
//   THE CARRIAGEWAY'S HALF WIDTH: half the travel width and one shoulder -- the spine to the edge of the paved road.
//   WHETHER CARS DRIVE ON IT: not on a track (the trains') nor on a footway (people on foot). Asked by the lane level,
//     the street map, the junction grouping, the kerb parking, the street names and the ramp landings (issue #292: each
//     once wrote the two flags out for itself).
// The road file's lanes and classes (roads/lanes LaneLayout, RoadClassSpec) and the model's section all answer through
// these functions.
inline bool lanesRunOneWay(int forward, int backward) { return (forward > 0) != (backward > 0); }
// (the median gap a road really has: the stated one where it has lanes both ways)
inline Real lanesMedianGap(int forward, int backward, Real medianGap) { return forward > 0 && backward > 0 ? medianGap : Real(0); }
inline Real lanesTravelWidth(int forward, int backward, Real laneWidth, Real medianGap) {
    return (forward + backward) * laneWidth + lanesMedianGap(forward, backward, medianGap);
}
inline Real carriagewayHalfWidthOf(Real travelWidth, Real shoulder) { return travelWidth * Real(0.5) + shoulder; }
inline bool sectionCarriesCars(bool track, bool footway) { return !track && !footway; }
struct ModelSection {
    std::string roadClass;
    RoadClass klass = RoadClass::Local;        // the engine's class for it (roads/lanes classOf: the one mapping)
    int lanesForward = 0, lanesBackward = 0;   // forward: running along the spine
    Real laneWidth = 0, medianGap = 0, median = 0, shoulder = 0, sidewalk = 0;
    Real parking = 0;                          // the kerb parking band's width along each kerb (m); 0: none
    Real parkingOffset = 0;                    // its middle, from the carriageway's middle toward the kerb (m)
    bool track = false, footway = false;
    // WHICH KERBS HAVE THE SIDEWALK (ADR-0464): `sidewalkAlong` the kerb of the traffic running along the spine (+1),
    // `sidewalkAgainst` the other (-1). The road file's edge may state it ("sidewalks": "both" | "along" | "against" |
    // "none"); stated nowhere, both, as before. `sidewalk` is the width of each one there is.
    bool sidewalkAlong = true, sidewalkAgainst = true;
    // Is there a sidewalk along kerb `kerb` (+1 / -1, as above)?
    bool sidewalkOnKerb(int kerb) const { return sidewalk > 0 && (kerb > 0 ? sidewalkAlong : sidewalkAgainst); }
    // CAN PEOPLE ON FOOT WALK ALONG KERB `kerb` OF IT: a sidewalk there on a road cars drive, or the road is a footway
    // (all of it is the walk). A track: never.
    bool pavementOnKerb(int kerb) const { return footway || (carriesCars() && sidewalkOnKerb(kerb)); }
    // Does its traffic run one way only (either way along the spine)?
    bool oneWay() const { return lanesRunOneWay(lanesForward, lanesBackward); }
    // The lanes whose traffic ARRIVES at an end of the road: the forward lanes where the spine runs INTO that end
    // (`spineIn`, as an arm states it), the backward lanes where it runs out of it. 0: nothing arrives there (the
    // entry end of a one-way street).
    int lanesArriving(bool spineIn) const { return spineIn ? lanesForward : lanesBackward; }
    // The width the travel lanes take across, both ways: what the map draws.
    Real travelWidth() const { return lanesTravelWidth(lanesForward, lanesBackward, laneWidth, medianGap); }
    // The spine to the carriageway's edge: where the kerb (or the verge) begins.
    Real carriagewayHalfWidth() const { return carriagewayHalfWidthOf(travelWidth(), shoulder); }
    // Do cars drive on it (not a track, not a footway)?
    bool carriesCars() const { return sectionCarriesCars(track, footway); }
};

// A STRETCH OF ROAD INSIDE A JUNCTION (between nodes the planner grouped into it; the road file's "inside": [{"road",
// "from", "to"}]), or one of the paint's boxes along a road: a road and stations from <= to. The road model adds to a
// stretch inside a junction what the street map needs to drive it (ADR-0425): its road's index, its centre line between
// the two stations (`line`, into RoadModel::linePoints, with their stations in RoadModel::lineStations) and its
// road's cross-section. A paint box carries none of them.
struct ModelStretch {
    ModelStretch() = default;
    ModelStretch(std::string r, Real f, Real t) : road(std::move(r)), from(f), to(t) {}
    std::string road;
    Real from = 0, to = 0;
    uint32_t edge = kNoIndex;        // the road's index (inside stretches; the road model's)
    Range line;
    ModelSection section;
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
//
// THE ARM THAT IS THE SAME ROAD CARRYING ON THROUGH from `arm`, as a junction's `through` pairs state it; kNoIndex: none
// is stated. The one reading of the pairs (ADR-0432): which turn is straight on, which approaches face each other, which
// road is the major one, whose width a stop line stands back by.
using ThroughPairs = std::vector<std::array<uint32_t, 2>>;
inline uint32_t throughPartner(const ThroughPairs& through, uint32_t arm) {
    for (const std::array<uint32_t, 2>& t : through) {
        if (t[0] == arm) return t[1];
        if (t[1] == arm) return t[0];
    }
    return kNoIndex;
}

// HOW FAR BACK FROM ITS MOUTH AN ARM'S STOP LINE STANDS -- THE ONE RULE (ADR-0419, ADR-0428, ADR-0471; issue #292). The
// junction's area begins back from the arm's mouth (away from the junction): that is the arm's BOX EDGE, where the
// crossing roads' lanes begin. Its STOP LINE stands kStopLineBack further back. A road CROSSING an arm is one of the
// junction's arms on a road other than its own and its stated through partner's (the same street carrying on is not
// crossing it). The box edge stands back by:
// - THE SQUARE RULE (ADR-0428, squareBoxEdgeBack): the half width of the widest road crossing it at the same point of
//   the junction (mouths within kSameJunctionPoint), else anywhere in the junction. Right where roads meet square;
// - or, where it is more, THE KERBS CLEAR (ADR-0471, crossingClearance): far enough that both of the arm's kerb lines
//   are out of every crossing arm's carriageway, anywhere in the junction, at the angle they actually meet. Each arm is
//   read as a straight strip of carriageway from its mouth along its stated heading, its half width either side. Where
//   two arms leave one point at a sharp angle a, the arm's kerb on that side runs on beside the other's carriageway
//   before it parts from it, and is clear at W / sin a + w / tan a (W: the crossing road's half width, w: the arm's own);
//   at a right angle or wider apart that is no more than the square rule's W. An arm that leaves beside it but apart (a
//   divided road's other carriageway) never overlaps it, and adds nothing.
// TWO CAPS on what the kerbs-clear rule adds:
// - THE ANGLE: no arm sets another back further than it would meeting it at kSharpestMeet from one point (W / sin 20 + w /
//   tan 20, about 2.9 W + 2.7 w), so arms leaving almost together (a fork, a slip road) or overlapping set each other
//   back a few road widths, not without bound;
// - THE ROOM (ArmMouth::room): at most half the room the square rule leaves on the arm's road between its stop line and
//   the next junction's facing one, less the storage a car needs between them (armRoom). So the two stop lines never
//   pass each other or the next junction, and the grouping of road ends into junctions, which asks this rule whether a
//   car has room between two nodes, groups exactly as the square rule did. An arm the room stops short of clear is a
//   planning fault -- two junctions too close for the angle their roads meet at (boxEdgeSqueeze) -- and its kerb end
//   still lies out in the paved junction.
// Asked by the stated junctions' areas (lanes/deck_mesh.h junctionAreas: the paint, the model's ModelArm::boxEdge /
// stopLine, the lane level) and by the grouping of road ends into junctions (lanes/control_plan.h findJunctions).
constexpr Real kStopLineBack = 5.2;
constexpr Real kSameJunctionPoint = 2.0;   // metres: two arms' mouths this near meet at one point of a junction
constexpr Real kSharpestMeet = 20.0;       // degrees: the angle cap (above)
// Two headings whose directions' dot product is no more than this are square or wider apart: a heading written to the
// hundredth of a degree (directionOf(90)) reads back about 6e-17 off square, which is square.
constexpr Real kSquareDot = 1e-9;
// A kerb line whose slope across a strip's edge is under this runs along that edge (never crosses it).
constexpr Real kAlongEdge = 1e-12;
// One arm as the rule reads it: its road (any number that tells roads apart; below zero: a road nobody knows, which
// crosses nothing), where its mouth is in plan, its road's half width (half the travel width), the way it leaves the
// junction (unit: its stated heading, directionOf; zero: not known, the square rule alone), and the room cap (above;
// metres; infinite: no junction faces it along its road).
struct ArmMouth {
    int road = -1;
    Vec2 mouth;
    Real halfWidth = 0;
    Vec2 dir;
    Real room = std::numeric_limits<Real>::infinity();
};
// The square rule's metres from arm k's mouth back to its box edge. `through`: the junction's stated through pairs, by
// place in `arms`.
inline Real squareBoxEdgeBack(const std::vector<ArmMouth>& arms, const ThroughPairs& through, std::size_t k) {
    const uint32_t ownStreet = throughPartner(through, static_cast<uint32_t>(k));
    Real atPoint = -1, anywhere = 0;
    for (std::size_t o = 0; o < arms.size(); ++o) {
        const ArmMouth& b = arms[o];
        if (b.road < 0 || b.road == arms[k].road || o == ownStreet) continue;
        anywhere = std::max(anywhere, b.halfWidth);
        if (atOrBefore((b.mouth - arms[k].mouth).length(), kSameJunctionPoint, kSamePoint)) atPoint = std::max(atPoint, b.halfWidth);
    }
    return atPoint >= 0 ? atPoint : anywhere;
}
// Metres from arm a's mouth to where its kerbs are clear of crossing arm b's strip, with the angle cap (above: THE KERBS
// CLEAR); 0 where a heading is not known or the two leave square or wider apart (the square rule answers there).
inline Real crossingClearance(const ArmMouth& a, const ArmMouth& b) {
    const Real W = b.halfWidth, w = a.halfWidth;
    if (a.dir.length() < 0.5 || b.dir.length() < 0.5 || dot(a.dir, b.dir) <= kSquareDot) return 0;   // (a zero dir: not known)
    const Real sinCap = std::sin(kSharpestMeet * DEG_TO_RAD), cosCap = std::cos(kSharpestMeet * DEG_TO_RAD);
    const Real cap = W / sinCap + w * cosCap / sinCap;
    const Vec2 n = perp(a.dir), nb = perp(b.dir);
    Real clear = 0;
    for (const Real side : {Real(1), Real(-1)}) {
        // the kerb line p(t) = o + t * a.dir, t >= 0, against b's strip: b.dir . q >= 0 and |nb . q| <= W (q = p - b.mouth),
        // each a half line in t (slope * t + at0 >= 0); where they all hold is one interval [lo, hi]
        const Vec2 o = a.mouth + n * (side * w) - b.mouth;
        const Real slope[3] = {dot(b.dir, a.dir), -dot(nb, a.dir), dot(nb, a.dir)};
        const Real at0[3] = {dot(b.dir, o), W - dot(nb, o), W + dot(nb, o)};
        Real lo = 0, hi = std::numeric_limits<Real>::infinity();
        for (int c = 0; c < 3; ++c) {
            if (std::fabs(slope[c]) < kAlongEdge) {
                if (at0[c] < 0) hi = -1;   // never inside
            } else if (slope[c] > 0) {
                lo = std::max(lo, -at0[c] / slope[c]);
            } else {
                hi = std::min(hi, -at0[c] / slope[c]);
            }
        }
        if (hi > lo) clear = std::max(clear, hi);
    }
    return std::min(clear, cap);
}
// Metres arm k's kerbs need beyond the square rule to be clear of every crossing arm (angle-capped, not room-capped).
inline Real kerbsClearBeyondSquare(const std::vector<ArmMouth>& arms, const ThroughPairs& through, std::size_t k) {
    const uint32_t ownStreet = throughPartner(through, static_cast<uint32_t>(k));
    Real clear = 0;
    for (std::size_t o = 0; o < arms.size(); ++o) {
        const ArmMouth& b = arms[o];
        if (b.road < 0 || b.road == arms[k].road || o == ownStreet) continue;
        clear = std::max(clear, crossingClearance(arms[k], b));
    }
    return std::max(Real(0), clear - squareBoxEdgeBack(arms, through, k));
}
// Metres from arm k's mouth back to its box edge: the square rule, or the kerbs clear, within the room.
inline Real boxEdgeBack(const std::vector<ArmMouth>& arms, const ThroughPairs& through, std::size_t k) {
    return squareBoxEdgeBack(arms, through, k) + std::min(kerbsClearBeyondSquare(arms, through, k), std::max(Real(0), arms[k].room));
}
// How many metres short of its kerbs clear the room cap leaves arm k's box edge (0: clear): the planning fault of two
// junctions too close for the angle their roads meet at.
inline Real boxEdgeSqueeze(const std::vector<ArmMouth>& arms, const ThroughPairs& through, std::size_t k) {
    return std::max(Real(0), kerbsClearBeyondSquare(arms, through, k) - std::max(Real(0), arms[k].room));
}
// ... and back to its stop line.
inline Real stopLineBack(const std::vector<ArmMouth>& arms, const ThroughPairs& through, std::size_t k) {
    return boxEdgeBack(arms, through, k) + kStopLineBack;
}
// THE ROOM CAP for two arms facing each other along one road, `gap` metres apart (mouth to mouth), whose square-rule stop
// lines stand `squareStopA` and `squareStopB` back: half what is left over once both stop lines and `storage` are taken
// out, less kSamePoint so the two enlarged stop lines never leave a hair under `storage` between them (0: no room left).
inline Real armRoom(Real gap, Real squareStopA, Real squareStopB, Real storage) {
    return std::max(Real(0), (gap - squareStopA - squareStopB - storage) * Real(0.5) - kSamePoint);
}

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
    Real speedLimit = 0;            // metres per second; 0: none posted
    Real lotsFrom = -1, lotsTo = -1; // where buildings may front it (the road's lots_range); -1: all of it
    std::string frontageOf;          // a frontage road: the freeway carriageway it runs beside
    Range line;                      // into linePoints: its centre line, `from` to `to` (planned: the road file's spine)
    std::vector<ModelLaneId> lanes;  // its lanes, forward then backward, each by slot from the centre line (built layer)
};

// A BUS STOP (the buses plan, step 1; ~/.claude/plans/buses-on-road-model.md): where a bus pulls in to the kerb and
// people get on and off. STATED in the road file, as a door's frontage is (ADR-0474): a street by its id, the station
// along its road's spine and the kerb (+1: the kerb of the traffic running along the spine, so a bus running that way
// serves it; -1: the other) -- never worked out from a position. The road model answers where its bus pulls in
// (RoadModel::busStopLane) and where people wait (RoadModel::busStopWalk).
struct ModelBusStop {
    std::string id;                  // unique in the road file ("R1-03")
    std::string name;                // what the sign says ("Mill Street"); may be empty
    std::string streetId;            // the street as the road file names it
    ModelStreetId street;            // ...resolved by the model (none: not resolved)
    Real station = 0;                // metres along the street's road's spine
    int kerb = 1;                    // +1 or -1
};
// A BUS LINE: its stops in the order a bus calls at them, round and round: from the last it drives on to the first. A
// line that goes there and back is a loop too, its way back calling at stops on the other kerb (a stop serves the
// traffic on its own kerb only).
struct ModelBusLine {
    std::string id;                  // unique in the road file ("R1")
    std::string name;                // "River Loop"
    std::vector<ModelBusStop> stops;
};

}  // namespace engine

#endif
