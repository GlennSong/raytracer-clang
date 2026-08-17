#ifndef ROADLAB_BUILDERS_H
#define ROADLAB_BUILDERS_H

// Composition, not primitives. Everything a road system is usually asked to
// special-case is built here out of the same three ingredients — a spine, a
// cross-section, and a junction:
//
//   roundabout  = a ring split into arcs + one junction per arm
//   on-ramp     = an auxiliary lane that grows from zero + one lane-level link
//   off-ramp    = the same in reverse
//   lane drop   = a width function reaching zero
//   turn bay    = a strip that only exists near a junction
//   overpass    = an elevation profile + a bridge span
//   tunnel      = an elevation profile + a tunnel span
//   multi-tier  = two roads whose elevation profiles differ; nothing else
//
// If any of these had needed a new type, the model underneath would be wrong.

#include "network.h"
#include "junction.h"
#include "structure.h"
#include <string>
#include <vector>

namespace roadlab {

struct RoadDesc {
    std::string name;
    std::string preset = "street2";
    std::vector<Vec2> points;
    double cornerRadius = 45.0;
    bool spirals = true;
    double baseHeight = 0.0;
    std::vector<Vec2> elevationKnots;   // (s, y); empty = flat at baseHeight
    double designSpeed = -1.0;          // < 0 = the preset's
    double crossfall = 0.02;
    bool autoSuperelevation = true;
    RoadKind kind = RoadKind::Normal;
};

int buildRoad(Network& net, const RoadDesc& desc);

// A junction from road ends. `arms` is (roadId, atEnd).
int buildIntersection(Network& net, const std::string& name,
                      const std::vector<std::pair<int, bool>>& arms, JunctionControl control,
                      double cornerRadius = 7.0);

// --- roundabout -----------------------------------------------------------

struct RoundaboutDesc {
    std::string name = "roundabout";
    Vec2 center{0, 0};
    // Radius of the ring's REFERENCE LINE, which the preset puts at the outer
    // edge of the truck apron. Circulating lanes sit outside it, the island
    // inside.
    double radius = 14.0;
    std::string ringPreset = "roundabout2";
    double height = 0.0;
    std::vector<std::pair<int, bool>> arms;   // (roadId, atEnd)
};

// Splits the ring into one arc per gap between arms and makes a junction at each
// arm, where circulating traffic has priority. Nothing about a roundabout is a
// special case except that its arms happen to be arcs of the same circle.
void buildRoundabout(Network& net, const RoundaboutDesc& desc);

// --- lane count and bays --------------------------------------------------

// Rebuild a road's cross-section as: base, then `delta` lanes added/removed on
// `side` starting at `s`, over the speed-derived taper, held for `runLength`,
// then back to base (runLength < 0 keeps the change to the end of the road).
void applyLaneChange(Network& net, int roadId, double s, int side, int delta,
                     double runLength = -1.0);

// Replay a road's recorded cross-section edits onto its base section. Called
// automatically by the apply* helpers; exposed so an editor could re-run it after
// changing an edit in place.
void rebuildProfile(Network& net, int roadId);

// A turn bay that exists only for `length` metres before `sJunction`.
void applyTurnBay(Network& net, int roadId, double sJunction, bool atEnd, int side,
                  double length = 60.0, double width = 3.3);

// Swap a road's median for a two-way left-turn lane over its whole length.
void applyTwltl(Network& net, int roadId);

// --- ramps ----------------------------------------------------------------

// How a ramp joins its mainline. Both are standard; they differ in which piece
// of geometry does the merging, and roadlab can express either because a
// merge is a width function, not a special case.
//
//   Parallel  an auxiliary lane at FULL width from the gore nose, running the
//             acceleration length before tapering out. The ramp delivers a whole
//             lane. Standard at freeway speeds.
//   Taper     no auxiliary lane; the ramp's own lane narrows into the mainline.
//             Shorter and simpler, and what a rural entrance looks like.
enum class RampEntry : uint8_t { Parallel, Taper };

// How long the auxiliary lane takes to reach full width at a parallel-type gore
// nose. Not zero: a lane that appears with no transition at all is a step in the
// cross-section, and the mesher needs somewhere to put the ring pair. One metre
// is below anything the eye resolves at road scale and keeps every downstream
// invariant (lane-section seams, the marking bake's ring rule) intact.
constexpr double kGoreNoseTaper = 1.0;

// How wide the painted gore is where the ramp first comes alongside. It closes
// to nothing at the nose, so this is the widest the neutral area ever gets;
// 3 m over the ~60 m the two run alongside is about a 1:20 taper, which is what
// the wedge on a real parallel entrance looks like.
constexpr double kGoreAreaWidth = 3.0;

struct RampDesc {
    std::string name = "ramp";
    std::string preset = "ramp1";
    Vec2 outerPoint{0, 0};       // where the ramp meets the surface street end
    double outerHeading = 0;     // heading at that end, pointing along travel
    double outerHeight = 0;
    double auxLength = 180.0;    // parallel length of the auxiliary lane
    double designSpeed = 60.0;
    // Which side of the mainline the ramp is on. 0 DERIVES it from outerPoint,
    // which is the only answer that cannot be wrong: the ramp is on whichever
    // side its far end is. Authoring it was how the metro generator ended up
    // running every ramp diagonally across its own bypass — the ring circulates
    // counter-clockwise, so its interchanges are on the left, and the default
    // said right. -1 and +1 still override.
    int side = 0;
    RampEntry entry = RampEntry::Parallel;
};

// Grows an auxiliary lane on the mainline, splits it at the merge point, and
// links the ramp's lane INTO that auxiliary lane. Returns the ramp road id, and
// writes the mainline's downstream road id to `mainlineTail`.
int buildOnRamp(Network& net, int mainRoad, double sMerge, const RampDesc& desc,
                int* mainlineTail = nullptr);

// The mirror: an auxiliary deceleration lane that the ramp peels out of.
int buildOffRamp(Network& net, int mainRoad, double sExit, const RampDesc& desc,
                 int* mainlineTail = nullptr);

// --- vertical -------------------------------------------------------------

// Raise a road over [s0, s1] by `rise`, with approach grades either side, and
// declare the span a bridge. The clearance lint then checks it against whatever
// passes underneath.
void makeOverpass(Network& net, int roadId, double s0, double s1, double rise,
                  double approach = 90.0, double pierSpacing = 30.0);

// Drop a road under the ground and declare the span a tunnel; the terrain
// deliberately keeps its own height over the top.
void makeTunnel(Network& net, int roadId, double s0, double s1, double drop,
                double approach = 90.0);

void makeEmbankment(Network& net, int roadId, double s0, double s1);

}  // namespace roadlab

#endif
