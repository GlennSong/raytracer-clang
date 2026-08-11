#ifndef ROADLAB_NETWORK_H
#define ROADLAB_NETWORK_H

// The topology layer: roads, how they link, and the LANE GRAPH that everything
// downstream drives on.
//
// Two graph resolutions on purpose. The ROAD graph (roads + junctions) is what
// routing searches. The LANE graph (one node per lane per section, edges through
// junction connectors) is what vehicles occupy and what lane-choice planning
// reads. Keeping them separate means route search stays cheap while the driving
// model still knows which of five lanes it has to be in.
//
// A road is never trimmed destructively. A junction claims a window of each arm
// by moving the road's [sBegin, sEnd], and the parametric source stays whole —
// so a junction can be rebuilt, resized or deleted without the road losing
// information it can't get back. Splitting a road likewise just makes two
// windows over the same geometry.

#include "profile.h"
#include "spine.h"
#include <string>
#include <vector>

namespace roadlab {

struct Junction;   // junction.h

enum class RoadKind : uint8_t {
    Normal,
    Ramp,
    Connector,        // generated inside a junction
    RoundaboutRing,   // one arc of a circulating carriageway
};

enum class LinkType : uint8_t { None, Road, Junction };

struct RoadLink {
    LinkType type = LinkType::None;
    int id = -1;
    // Which end of the linked road we attach to. Meaningless for junctions.
    bool toStart = true;
    // Explicit lane pairing (fromLaneId, toLaneId). Empty = match by ordinal and
    // kind. A merge uses this to say "the ramp's lane becomes the mainline's new
    // auxiliary lane", which is the whole content of an on-ramp.
    std::vector<std::pair<int, int>> laneLinks;
};

// How a stretch of road is carried. Elevation already lives on the spine, so a
// two-tier interchange is free geometrically; what a structure adds is whether
// the ground follows the road, what gets built underneath, and what the
// clearance lint has to check. The generators live in structure.h.
enum class CarrierKind : uint8_t {
    AtGrade,      // terrain conforms to the road (cut / fill)
    Embankment,   // raised on fill with side slopes
    Cut,          // below grade, walls or batters either side
    Bridge,       // deck on piers; the ground passes underneath untouched
    Viaduct,      // a long bridge; piers on a regular rhythm
    Tunnel,       // bored; the ground closes over the top
};

struct StructureSpan {
    CarrierKind kind = CarrierKind::AtGrade;
    double s0 = 0, s1 = 0;
    double deckThickness = 1.2;    // bridge/viaduct
    double pierSpacing = 32.0;
    double clearanceRequired = 5.1;   // what must fit underneath (m)
    double boreClearance = 5.5;       // tunnel headroom above the surface
    double parapetHeight = 1.05;
    bool generatePiers = true;
    bool contains(double s) const { return s >= s0 - 1e-6 && s <= s1 + 1e-6; }
};

struct Road {
    int id = -1;
    std::string name;
    RoadKind kind = RoadKind::Normal;
    Spine spine;
    CrossSection xs;
    double designSpeed = 50.0;      // km/h
    bool allowsPedestrians = true;
    int junctionId = -1;            // >= 0 when this road is a junction connector

    RoadLink pred, succ;

    // The window of the spine this road actually occupies. Junction trimming and
    // road splitting both work by moving these, never by editing the geometry.
    double sBegin = 0.0;
    double sEnd = -1.0;             // < 0 means "to the end of the spine"

    std::vector<StructureSpan> structures;

    // Cross-section edits, recorded rather than applied destructively. Each is
    // replayed in station order against whatever the section has already become,
    // so "drop a lane at 150 m" and "add a turn bay at 400 m" compose instead of
    // the second silently undoing the first.
    struct ProfileEdit {
        enum class Kind : uint8_t { Lanes, TurnBay, Twltl, Auxiliary };
        Kind kind = Kind::Lanes;
        double s = 0;
        double runLength = -1;   // < 0 = to the end of the road
        double taper = -1;       // < 0 = solve it
        int side = -1;
        int delta = -1;
        double width = 3.3;
        bool atEnd = true;       // TurnBay: which side of `s` the bay sits on
    };
    std::vector<ProfileEdit> profileEdits;

    double spineLength() const { return spine.length(); }
    double begin() const { return sBegin; }
    double end() const { return sEnd < 0 ? spine.length() : std::min(sEnd, spine.length()); }
    double activeLength() const { return std::max(0.0, end() - begin()); }
    bool contains(double s) const { return s >= begin() - 1e-6 && s <= end() + 1e-6; }

    // Surface point at (s, t): the spine frame, plus the cross-section's own
    // height so kerbs and footways stand proud of the carriageway.
    Vec3 surfacePoint(double s, double t) const;
    Vec3 surfaceNormal(double s, double t) const;
    Frame frameAt(double s) const { return spine.frameAt(s); }
    Vec2 planPoint(double s, double t) const { return spine.toPlan(s, t); }

    // Pose of a lane at a station, in the lane's own travel direction.
    bool lanePose(int laneId, double s, Vec2& pos, double& heading) const;
    double laneCenterT(int laneId, double s) const;
    double speedLimitOf(int laneId, double s) const;
};

// --- lane graph -----------------------------------------------------------

struct LaneRef {
    int road = -1;
    int section = 0;
    int lane = 0;
    bool valid() const { return road >= 0 && lane != 0; }
    bool operator==(const LaneRef& o) const {
        return road == o.road && section == o.section && lane == o.lane;
    }
};

struct LaneNode {
    LaneRef ref;
    double sStart = 0;      // in travel order: where the vehicle enters
    double sEnd = 0;        // where it leaves (sEnd < sStart when dir == -1)
    double length = 0;
    int8_t dir = 1;
    StripKind kind = StripKind::Travel;
    float speedLimit = 50.0f;
    uint32_t access = kAccessCar;
    int junctionId = -1;

    std::vector<int> successors;
    std::vector<int> predecessors;

    // Neighbours in the TRAVEL frame, not the t frame: `left` is the lane a
    // driver would see out of their left window regardless of which way the
    // reference line runs.
    int leftNeighbor = -1;
    int rightNeighbor = -1;
    bool leftCrossable = false;
    bool rightCrossable = false;

    // Station in travel order -> (s, t) on the road.
    double roadS(double travelled) const { return dir > 0 ? sStart + travelled : sStart - travelled; }
};

class LaneGraph {
public:
    std::vector<LaneNode> nodes;
    void clear() { nodes.clear(); index_.clear(); }
    int find(const LaneRef& ref) const;
    void addIndex(const LaneRef& ref, int node);
    // Lanes reachable from `node` that eventually reach `targetRoad`, within
    // `maxDepth` hops. This is what turns "I need to turn right in 400 m" into
    // "I must be in one of these lanes".
    std::vector<int> lanesReaching(int fromNode, int targetRoad, int maxDepth = 6) const;
    // The same question for a single lane, which is what a driver actually asks
    // every time they check whether they are still in the right one.
    bool reaches(int fromNode, int targetRoad, int maxDepth = 6) const;

private:
    struct Key {
        int road, section, lane;
        bool operator<(const Key& o) const {
            if (road != o.road) return road < o.road;
            if (section != o.section) return section < o.section;
            return lane < o.lane;
        }
    };
    std::vector<std::pair<Key, int>> index_;
};

// An explicit lane-to-lane link that the road graph cannot express. A road has
// one predecessor and one successor, but an on-ramp needs a THIRD attachment:
// its lane becomes the mainline's new auxiliary lane while the mainline itself
// carries straight on. Rather than inventing a junction with no pad, the merge
// is stated for what it is — one lane continuing into another.
struct ExtraLaneLink {
    int fromRoad = -1;
    bool fromAtEnd = true;
    int fromLane = 0;
    int toRoad = -1;
    bool toAtStart = true;
    int toLane = 0;
};

// --- network --------------------------------------------------------------

struct RoadHit {
    int road = -1;
    double s = 0, t = 0;
    double distance = 0;   // lateral distance outside the road surface, 0 if on it
};

class Network {
public:
    int addRoad(Road r);
    Road& road(int id) { return roads_[size_t(id)]; }
    const Road& road(int id) const { return roads_[size_t(id)]; }
    std::vector<Road>& roads() { return roads_; }
    const std::vector<Road>& roads() const { return roads_; }
    int roadCount() const { return int(roads_.size()); }

    int addJunction(const Junction& j);
    Junction& junction(int id);
    const Junction& junction(int id) const;
    std::vector<Junction>& junctions() { return junctions_; }
    const std::vector<Junction>& junctions() const { return junctions_; }
    int junctionCount() const { return int(junctions_.size()); }

    // Split a road at station s. The original keeps [begin, s]; the returned road
    // is [s, end] over a copy of the same geometry. Links are rewired.
    int splitRoad(int roadId, double s);

    // Resolve every junction (trim arms, build connectors, find conflicts,
    // generate signal phases) and then build the lane graph.
    void build();
    const LaneGraph& lanes() const { return lanes_; }
    LaneGraph& lanes() { return lanes_; }

    // Design lint. Returns human-readable problems: sub-minimum radii for the
    // design speed, over-steep grades, tapers that are too short, and vertical
    // clearance violations between stacked roads.
    std::vector<std::string> validate() const;

    // Nearest road surface to a plan point. Used by the sim for re-localisation
    // and by the renderer's top-down mode.
    bool sample(Vec2 planPoint, RoadHit& hit, double maxDistance = 5.0) const;

    void planBounds(Vec2& lo, Vec2& hi) const;

    std::vector<ExtraLaneLink> extraLinks;

private:
    std::vector<Road> roads_;
    std::vector<Junction> junctions_;
    LaneGraph lanes_;

    void buildLaneGraph();
    void linkRoadToRoad(int roadA, bool aAtEnd, int roadB, bool bAtStart,
                        const std::vector<std::pair<int, int>>& overrides);
};

// Lane ordinals a road offers at one of its ends, split by whether traffic is
// entering or leaving through that end. Junction building and road linking both
// need this and must agree, so it lives here.
struct EndLanes {
    std::vector<int> incoming;   // lanes whose travel direction reaches this end
    std::vector<int> outgoing;   // lanes that leave the road through this end
};
EndLanes endLanes(const Road& r, bool atEnd);

}  // namespace roadlab

#endif
