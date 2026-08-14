#ifndef ROADLAB_PROFILE_H
#define ROADLAB_PROFILE_H

// The cross-section: what the road looks like laterally, as a function of s.
//
// A road is an ordered stack of STRIPS either side of the reference line, each
// with a width that is a cubic in s. That single idea covers most of the road
// vocabulary: a lane that appears is a strip whose width ramps 0 -> W, a lane
// that drops ramps W -> 0, a shoulder is a strip, a median is a strip, parallel
// parking is a strip, the sidewalk is a strip. Nothing is special-cased, and
// because the boundary offsets are known functions of s, the lane markings drawn
// on them follow every taper for free.
//
// Topology (which strips exist) changes only at LANE SECTION boundaries. Inside
// a section, only widths vary. That split is what keeps the model both
// continuous enough to render and discrete enough to reason about.

#include "rl_math.h"
#include "spine.h"
#include <string>
#include <vector>

namespace roadlab {

enum class StripKind : uint8_t {
    Shoulder,   // paved or gravel; drivable in emergencies
    Travel,     // an ordinary through lane
    Turn,       // a turn bay / auxiliary lane
    Bus,        // bus or HOV lane
    Bike,       // cycle lane
    Parking,    // kerbside parking (parallel or angled)
    Median,     // separation: painted, raised, planted or barrier
    Apron,      // roundabout truck apron (mountable)
    Curb,       // the kerb face: the ramp between two heights
    Gutter,     // drainage channel
    Verge,      // grass / planting strip
    Sidewalk,   // raised pedestrian way
    Barrier,    // parapet, jersey barrier, guardrail zone
    Slope,      // cut/fill batter running out to terrain
};

enum class SurfaceKind : uint8_t { Asphalt, Concrete, Gravel, Dirt, Grass, Brick, Steel };

// Marking style. The style is BOTH the picture the shader draws and the rule the
// simulator obeys — one field, so paint and legality cannot drift apart.
enum class MarkStyle : uint8_t {
    None,
    Solid,        // no crossing
    Dashed,       // crossing allowed both ways
    Double,       // two solid stripes; no crossing
    SolidDashed,  // solid on the left of the pair, dashed on the right
    DashedSolid,
    WideDashed,   // ramp/lane-drop dotted extension; crossing allowed
    Botts,        // raised markers
    Hatch,        // diagonal hatching (painted median / gore)
    Chevron,      // gore chevrons pointing back up the road
};

enum class PaintColor : uint8_t { White, Yellow, Red, Blue, Green };

struct Marking {
    MarkStyle style = MarkStyle::None;
    PaintColor color = PaintColor::White;
    float width = 0.12f;      // single-stripe width (m)
    float gap = 0.10f;        // separation for Double / SolidDashed (m)
    float dashOn = 3.0f;      // dash pattern along s (m)
    float dashOff = 9.0f;
    float wear = 0.0f;        // 0 = fresh paint, 1 = barely there

    // Legality, derived from the style unless overridden. `fromLeft` means a
    // vehicle on the +t side crossing toward -t.
    bool crossableFromLeft() const;
    bool crossableFromRight() const;
};

Marking markSolid(PaintColor c = PaintColor::White, float w = 0.12f);
Marking markDashed(PaintColor c = PaintColor::White, float on = 3.0f, float off = 9.0f);
Marking markDouble(PaintColor c = PaintColor::Yellow);
Marking markEdge();          // the solid white edge line at the shoulder
Marking markNone();

enum : uint32_t {
    kAccessCar = 1u << 0,
    kAccessTruck = 1u << 1,
    kAccessBus = 1u << 2,
    kAccessBike = 1u << 3,
    kAccessPed = 1u << 4,
    kAccessEmergency = 1u << 5,
    kAccessAll = 0xFFFFFFFFu,
};

constexpr int kNoLane = 0;   // 0 is the reference line, never a lane id

struct Strip {
    StripKind kind = StripKind::Travel;
    SurfaceKind surface = SurfaceKind::Asphalt;
    Poly3 width = Poly3::constant(3.6);

    // Travel direction relative to the reference line: +1 with increasing s,
    // -1 against, 0 not drivable. Direction lives per strip because that is the
    // only place it is actually true — a one-way pair and a two-way street are
    // the same structure with different per-strip signs.
    int8_t dir = 0;

    // The marking on this strip's OUTER boundary (the edge further from the
    // reference line). The innermost boundary of all is the centre marking on
    // the section itself.
    Marking outerMark;

    float speedLimit = 0.0f;      // km/h; 0 = inherit the road's
    uint32_t access = kAccessCar | kAccessTruck | kAccessBus | kAccessEmergency;
    float height = 0.0f;          // top surface above the pavement plane (m)

    int id = 0;                   // signed ordinal: +ve left of the reference line
    int predecessor = kNoLane;    // lane id in the previous section
    int successor = kNoLane;      // lane id in the next section

    bool drivable() const;
    bool isLane() const;          // counts toward "how many lanes is this road"
};

// The shortest run a lane section may have. A section shorter than this carries
// no lane-graph nodes, so leaving one in the list puts a hole in the middle of a
// road: the lanes either side link to it instead of to each other, and traffic
// cannot pass. Both the section list and the lane graph measure against this
// same number — they are two halves of one rule, and they drifted apart once.
constexpr double kMinLaneSectionRun = 0.25;

struct LaneSection {
    double s0 = 0;
    double length = 0;            // filled in by CrossSection::finalize
    std::vector<Strip> left;      // index 0 adjacent to the reference line, outward
    std::vector<Strip> right;
    Marking centerMark;           // the marking sitting on the reference line

    int laneCount() const;
    double totalWidthAt(double s) const;
    double carriagewayHalfWidthAt(double s, int side) const;   // side +1 left, -1 right
    const Strip* strip(int id) const;
    Strip* strip(int id);
    void assignIds();             // (re)number strips outward from the reference line
};

// One lateral boundary between two strips, as the shader and the sim see it.
struct Boundary {
    double t = 0;
    Marking mark;
    StripKind innerKind = StripKind::Travel;   // strip on the reference-line side
    StripKind outerKind = StripKind::Travel;
};

class CrossSection {
public:
    std::vector<LaneSection> sections;
    Profile1D laneOffset;         // lateral shift of the road centre from the spine

    void finalize(double roadLength);
    bool empty() const { return sections.empty(); }

    int sectionIndexAt(double s) const;
    const LaneSection& sectionAt(double s) const;

    // Every boundary at station s, ordered from the most negative t to the most
    // positive. This is the one query the marking shader makes.
    void boundariesAt(double s, std::vector<Boundary>& out) const;

    double laneCenterT(int sectionIdx, int laneId, double s) const;
    double laneWidthAt(int sectionIdx, int laneId, double s) const;
    // Inner/outer boundary offsets of a lane, in t.
    bool laneSpanAt(int sectionIdx, int laneId, double s, double& tInner, double& tOuter) const;

    double totalWidthAt(double s) const;
    double leftExtentAt(double s) const;    // +t edge of the whole section
    double rightExtentAt(double s) const;   // -t edge (negative)
    // Height of the surface at (s,t) above the pavement plane: flat across most
    // strips, ramped across Curb strips so kerbs and sidewalks read in 3D.
    double heightAt(double s, double t) const;
    const Strip* stripAtT(double s, double t, int* laneId = nullptr) const;

    // Link lanes across every section boundary by matching kind and ordinal, so
    // the lane graph knows that lane -2 here continues as lane -2 there even when
    // a neighbouring lane appeared or vanished between them.
    void linkSections();
};

// --- presets --------------------------------------------------------------

// A road "type" is not a class, it is a named cross-section plus a design speed.
// Freeway, arterial, alley and country lane are all the same structure.
struct RoadPreset {
    std::string name;
    LaneSection section;
    double designSpeedKph = 50.0;
    bool allowsPedestrians = true;
};

// Known names: street2, street2_park, street1_oneway, alley1, collector4,
// arterial4, arterial6_median, boulevard_twltl, freeway2, freeway3, freeway4,
// ramp1, ramp2, country2, roundabout1, roundabout2, bridge2, tunnel2, service1.
RoadPreset roadPreset(const std::string& name);
std::vector<std::string> roadPresetNames();

// --- transitions ----------------------------------------------------------

// Radius and design speed are two views of the same constraint:
// R = v^2 / (127 (e + f)). Both directions live here so the lint, the ramp
// generator and the junction connector builder all quote the same number — a
// generator that used a different constant would produce roads its own lint
// rejects, which is exactly the failure this avoids.
double minRadiusForSpeed(double speedKph, double sideFriction = 0.18);
double designSpeedForRadius(double radius, double sideFriction = 0.18);

// MUTCD-style taper length for a lateral shift of `widthChange` metres at
// `speedKph`. Nobody should be typing taper lengths by hand: the number follows
// from the geometry and the design speed, and getting it wrong is what makes a
// generated merge look and drive wrong.
double taperLength(double widthChange, double speedKph);

// The shortest transition that can legally get from A to B at this speed.
double minTransitionLength(const LaneSection& a, const LaneSection& b, double speedKph);

// Blend A into B over `length`, starting at `s0`. Strips are aligned by role
// (a Needleman-Wunsch pass over the kind sequences), matched strips blend width,
// unmatched strips taper in or out, and a lane that is being dropped has its
// outer marking forced solid — which is exactly how a real taper is painted.
LaneSection blendSection(const LaneSection& a, const LaneSection& b, double s0, double length);

// Author a road's cross-section as keyframes: "this template from here", "become
// that one over L metres starting here". The timeline emits the lane sections.
class ProfileTimeline {
public:
    // `transitionLength < 0` asks the solver for the minimum legal length.
    void at(double s, const LaneSection& section, double transitionLength = -1.0);
    CrossSection build(double roadLength, double designSpeedKph) const;
    bool empty() const { return keys_.empty(); }

private:
    struct Key {
        double s = 0;
        LaneSection section;
        double transition = -1.0;
    };
    std::vector<Key> keys_;
};

// --- cross-section edits --------------------------------------------------

// Drop `count` lanes from `side` (+1 left, -1 right) starting at `s`, over a
// speed-derived taper. Returns the section list to hand to a ProfileTimeline.
LaneSection sectionWithLanesChanged(const LaneSection& base, int side, int delta,
                                    double laneWidth = 3.6);
// The same section with its outermost Shoulder narrowed to nothing.
//
// What a gore actually does. An auxiliary lane is inserted INBOARD of the
// shoulder, so a ramp converging from outside has to cross the shoulder to reach
// it — there is no station at which the two pavements simply meet, and no choice
// of taper length avoids that. The shoulder has to get out of the way: it
// narrows to zero ahead of the nose, the ramp's own pavement occupies that
// ground, and the shoulder reappears outboard of the new lane. That is what the
// paint on a real gore is drawn around.
LaneSection sectionWithShoulderRetracted(const LaneSection& base, int side);
// Insert a turn bay: a Turn strip on `side` that exists only near a junction.
LaneSection sectionWithTurnBay(const LaneSection& base, int side, double width = 3.3);
// Swap the median for a two-way left-turn lane.
LaneSection sectionWithTwltl(const LaneSection& base, double width = 3.6);

const char* stripKindName(StripKind k);
const char* surfaceName(SurfaceKind s);
const char* markStyleName(MarkStyle s);

}  // namespace roadlab

#endif
