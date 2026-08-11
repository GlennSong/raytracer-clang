#ifndef ROADLAB_JUNCTION_H
#define ROADLAB_JUNCTION_H

// Junctions are the ONLY special case in the whole model, and they are special
// only because the road surface stops being a ribbon there. Everything else that
// people normally hard-code — roundabouts, on-ramps, turn bays — is composition
// over roads and junctions, built in builders.h.
//
// A junction owns: which roads arrive (arms), how far each was pulled back, the
// pad polygon that fills the middle, the CONNECTOR roads that carry each turning
// movement, the conflict points between those connectors, and the control policy
// that arbitrates them. The conflict table is computed once and read by both the
// simulator (right of way) and the signal generator (which movements can be
// green together) — the same fact serving both.

#include "network.h"
#include <string>
#include <vector>

namespace roadlab {

enum class JunctionControl : uint8_t {
    Uncontrolled,   // priority to the right
    Yield,          // minor arms yield to the major road
    PriorityStop,   // minor arms stop
    AllWayStop,
    Signalized,
    RoundaboutEntry,   // circulating traffic has absolute priority
    Merge,             // free-flowing merge/diverge; no pad, gap acceptance only
};

enum class TurnKind : uint8_t { Through, Left, Right, UTurn, Merge, Diverge };

enum class ConflictKind : uint8_t { Crossing, Merging, Diverging };

struct JunctionArm {
    int road = -1;
    bool atEnd = true;         // the junction touches the road's high-s end
    double sContact = 0;       // station on the road where the pad begins
    Vec2 contact{0, 0};        // plan point of that station on the reference line
    double headingIn = 0;      // plan heading pointing INTO the junction
    double leftExtent = 0;     // +t edge of the carriageway at the contact
    double rightExtent = 0;    // -t edge (negative)
    double trim = 0;           // how much of the road the junction claimed
    double priority = 0;       // higher wins at unsignalised junctions
    bool crosswalk = true;
};

struct Connection {
    int connectorRoad = -1;    // the generated Road carrying this movement
    int fromArm = -1, toArm = -1;
    LaneRef from, to;          // the lane entering and the lane leaving
    TurnKind turn = TurnKind::Through;
    int signalGroup = -1;      // index into Junction::phases, -1 = unsignalised
    double priority = 0;       // resolved right-of-way rank
    bool yields = false;       // must give way regardless of priority (ramp/roundabout)
};

struct ConflictPoint {
    int connA = -1, connB = -1;      // indices into Junction::connections
    double sA = 0, sB = 0;           // station on each connector where they meet
    double angle = 0;                // crossing angle, radians
    ConflictKind kind = ConflictKind::Crossing;
};

struct SignalPhase {
    std::vector<int> connections;    // indices into Junction::connections
    double green = 12.0;
    double yellow = 3.5;
    double allRed = 1.5;
};

struct Junction {
    int id = -1;
    std::string name;
    Vec2 center{0, 0};
    double elevation = 0;
    JunctionControl control = JunctionControl::Uncontrolled;
    double cornerRadius = 6.0;
    bool generateCrosswalks = true;

    std::vector<JunctionArm> arms;
    std::vector<Connection> connections;
    std::vector<ConflictPoint> conflicts;
    std::vector<SignalPhase> phases;
    std::vector<Vec2> boundary;      // the pad polygon, CCW

    double cycleLength() const;
    // Which phase is green at time `t`, and how long it has left.
    int phaseAt(double t, double& remaining) const;
    bool isGreen(int connectionIndex, double t) const;
};

// Resolve a junction in place: measure the arms, choose trim distances, move the
// arms' [sBegin,sEnd] windows back, build the pad polygon, generate a connector
// road per turning movement, find the conflict points and — if signalised —
// colour the conflict graph into phases.
void buildJunction(Network& net, Junction& j);

// Classify a movement from one arm to another by their headings.
TurnKind classifyTurn(double headingInFrom, double headingOutTo);

const char* turnName(TurnKind t);
const char* controlName(JunctionControl c);

// Crosswalk bands generated for a junction arm: a rectangle in the arm's own
// (s,t) frame that the surface shader paints and the pedestrian graph walks.
struct Crosswalk {
    int road = -1;
    double s = 0;            // centre station on the arm
    double depth = 3.0;      // along s
    double tMin = 0, tMax = 0;
    bool signalised = false;
};
std::vector<Crosswalk> junctionCrosswalks(const Network& net, const Junction& j);

}  // namespace roadlab

#endif
