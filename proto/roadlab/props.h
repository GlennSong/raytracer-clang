#ifndef ROADLAB_PROPS_H
#define ROADLAB_PROPS_H

// Nothing here is placed by hand. Every lamp, sign, signal and tree is the
// output of a RULE evaluated against the parametric road, which is what makes
// the furniture correct by construction rather than correct by diligence:
//
//   * a lamp is "every 30 m at the outer kerb, alternating sides, not inside a
//     junction";
//   * a STOP sign exists because the junction's control policy says that arm
//     stops — the same fact the simulator arbitrates with;
//   * a LANE ENDS sign exists because a strip's width function reaches zero, and
//     it stands one taper length upstream because that is what the taper length
//     is for;
//   * an advisory speed plaque carries the number the connector's own curvature
//     produced.
//
// Get the road data right and the signage cannot contradict it.

#include "network.h"
#include "junction.h"
#include "tessellate.h"
#include <string>
#include <vector>

namespace roadlab {

enum class PropKind : uint8_t {
    StreetLight,
    TrafficSignal,
    PedSignal,
    SignStop,
    SignYield,
    SignSpeed,
    SignAdvisory,
    SignGuide,
    SignMerge,
    SignLaneEnds,
    SignCurve,
    SignNoEntry,
    Tree,
    Bench,
    Hydrant,
    ParkingMeter,
    BusStop,
    Bollard,
    CrashAttenuator,
    MilePost,
};

struct Prop {
    PropKind kind = PropKind::StreetLight;
    Vec3 pos{0, 0, 0};
    double yaw = 0;          // facing, plan radians
    double scale = 1.0;
    int road = -1;
    double s = 0, t = 0;
    int junction = -1;
    double value = 0;        // speed limit, distance, whatever the sign says
    char label[12] = {0};
};

struct PropRules {
    double lightSpacing = 30.0;
    double lightHeight = 8.5;
    bool alternateLights = true;
    double treeSpacing = 11.0;
    double meterSpacing = 6.5;
    double hydrantSpacing = 85.0;
    double junctionClearance = 14.0;   // no furniture this close to a pad
    uint32_t seed = 11;
    bool lights = true;
    bool signs = true;
    bool signals = true;
    bool vegetation = true;
    bool streetFurniture = true;
};

void generateProps(const Network& net, std::vector<Prop>& out, const PropRules& rules = {});
void tessellateProps(const std::vector<Prop>& props, Mesh& out);

const char* propKindName(PropKind k);

}  // namespace roadlab

#endif
