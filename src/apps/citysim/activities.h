#ifndef RAYTRACER_CITYSIM_ACTIVITIES_H
#define RAYTRACER_CITYSIM_ACTIVITIES_H

// ACTIVITY SPOTS (the behaviour plan, step 2; Glenn: "do they use existing tech? Or do you need to build a system to
// define npc behavior?"). The world ADVERTISES what a body can do where -- a bench to sit on, a stand to watch from,
// a lab bench to stand at, a track to run, a pitch to play on -- and a goal state asks for "an activity of these kinds,
// with these tags" instead of naming a C++ target. The seats people already sit on (the furniture library, M5) are the
// first kind; new behaviours are new kinds and new spots, data on top of one reservation registry.
//
//   kind   what the body does there (and so how it is drawn)
//   tags   where it is, for a goal to filter on: on the campus, in a park, by the sports field
//
// One registry, one reservation per spot: nobody shares a seat, and a spot is released on every way out of it.

#include <cstdint>
#include <string>
#include <vector>

namespace citysim {

enum class SpotKind : uint8_t {
    Sit,     // a seat: walk to it, sit facing its way, walk back
    Lie,     // a bed, a lawn
    Stand,   // at a lectern, a lab bench, a rail
    Jog,     // a loop to run (a track)
    Play,    // a place to play (a pitch)
    Watch,   // a place to stand and watch from
    Count
};
const char* spotKindName(SpotKind k);
bool spotKindFromName(const std::string& name, SpotKind& out);
inline uint32_t spotKindBit(SpotKind k) { return 1u << static_cast<unsigned>(k); }

// Where a spot is (bits), for a goal to require.
namespace spot_tag {
constexpr uint32_t kCampus = 1;   // on a university campus: its quad, its courtyard, by its sports field
constexpr uint32_t kPark = 2;     // in a park
constexpr uint32_t kSports = 4;   // at the sports field
}  // namespace spot_tag
uint32_t spotTagFromName(const std::string& name);   // 0 for an unknown name

// What a goal asks for: any of `kinds`, carrying ALL of `tags`, between `minDist` and `maxDist` of where the agent
// stands (straight line), one of the `nearest` closest free ones (so neighbours do not all take the same bench).
struct ActivityQuery {
    uint32_t kinds = 1u;   // spotKindBit(SpotKind::Sit)
    uint32_t tags = 0;
    double minDist = 0, maxDist = 600;
    int nearest = 6;
};

// ---- THE CATALOG (activities framework, step A; ~/.claude/plans/activities-framework.md) ----------------------------
//
// WHAT a body can go and do (an ActivityDef: which kinds of SITE it happens at, when, how long, how far it is worth
// walking) and WHO picks what WHEN (a Menu: per hour band, activities tried first with a chance, then a weighted pick).
// Data -- assets/scripts/activities.lua, with these C++ defaults when there is no script -- so a new errand for the
// town's residents is a definition and a menu line, not a new chooser in C++.
//
// SITE KINDS (strings, so Lua names them): the city's places -- "cafe", "restaurant", "shop", "supermarket", "civic",
// "park", "library", "teaching", "quad", "field" -- its activity spots -- "seat" (Sit), "bed" (Lie), "stand" (Stand),
// "loop" (Jog), "watch" (Watch) -- its activity AREAS -- "pitch", "lawn" -- and "street" (a street corner to walk to).

// What the body does once there.
enum class Perform : uint8_t {
    Inside,    // in through the door, out of sight (a cafe, a shop)
    Outside,   // stays out at the site (a park, the quad)
    Spot,      // the spot's own use: sit, lie, stand at it, run its loop (stepSeats)
    Wander,    // a walk to a street corner and a look about
    Roam,      // a GROUP on an area (a pitch, a lawn): a session, its members placed by the activity's formation
};

// How a group's members stand on its area (perform Roam).
enum class Formation : uint8_t {
    Roam,     // each role runs to point after point in its zone (a kickabout); one session to an area
    Circle,   // a ring facing its middle, sized to who is in it (a chat, a picnic); joinable, several to a lawn
    Pair,     // two facing each other `radius` apart, stepping about (a game of catch)
    Spread,   // side by side, a body's width and a bit apart (sunbathers)
};
// How a member's body is held once it is in its place.
enum class Pose : uint8_t { Stand, SitGround, Lie };
const char* formationName(Formation f);
const char* poseName(Pose p);
// Area site kinds: a session on one of the city's areas rather than a spot or a place.
inline bool isAreaSiteKind(const std::string& k) { return k == "pitch" || k == "lawn"; }

// A ROLE in a group activity: how many take it, how fast they move, and which part of the area is theirs (-1 the
// whole of it, 0 / 1 its first / second half along its length -- swapped at the session's half time).
struct RoleDef {
    std::string name;
    int n = 1;
    double speedLo = 1.2, speedHi = 1.6;
    int zone = -1;
    Pose pose = Pose::Stand;   // held in its place (formations other than Roam)
};

struct ActivityDef {
    std::string name;
    std::vector<std::string> sites;   // site kinds it happens at (any of)
    uint32_t tags = 0;                // spot tags the site must carry (spot sites only)
    double hourLo = 0, hourHi = 24;   // when it is offered (a band that may wrap midnight)
    double minutesLo = 0, minutesHi = 0;   // how long; 0 = the site kind's own (a coffee, a meal)
    double distLo = 0, distHi = 650;  // straight-line band from where the agent stands (m)
    int nearest = 0;                  // >0: one of the k nearest candidates; 0: every candidate in the band counts
    bool perSite = false;             // its menu weight is per candidate site (three cafes: three times a coffee)
    bool walkersOnly = false;         // not for someone in their car
    bool inShift = false;             // only inside the agent's working window (lunch)
    int bringOwnEighths = 0;          // this many eighths of agents (by their own bits) never go (brought lunch)
    bool fromHome = false;            // the distance band is measured from HOME, not from here (the errand on the
                                      // way home: a shop near home)
    Perform perform = Perform::Inside;
    // A GROUP (perform Roam, an area site): its roles; it starts once `minPlayers` are there, and a session that has
    // not filled within `gatherMinutes` is given up (its players go); `swapAt` (0..1 of the run) swaps the zones.
    std::vector<RoleDef> roles;
    int minPlayers = 0;
    double gatherMinutes = 20;
    double swapAt = 0;
    // How the members stand (Roam: zones; Circle, Pair, Spread: places round the session's middle) and the size of it
    // (a ring's least radius, the pair's distance apart, the spread's spacing; 0 the formation's own).
    Formation formation = Formation::Roam;
    double radius = 0;
    // Offered only while a session of this activity is running within `distHi` (watching a game).
    std::string during;
};

struct MenuEntry {
    std::string activity;
    double chance = 0;   // in `first`: tried, in order, with this probability
    double weight = 0;   // in `pick`: its share of the weighted choice
};
struct MenuBand {
    double hourLo = 0, hourHi = 24;
    std::vector<MenuEntry> first;
    std::vector<MenuEntry> pick;
};
struct Menu {
    std::string name;
    std::vector<MenuBand> bands;
};

struct ActivityCatalog {
    std::vector<ActivityDef> defs;
    std::vector<Menu> menus;
    int find(const std::string& name) const;   // -1 when unknown
    const Menu* menu(const std::string& name) const;
    // Canonical text: two catalogs that describe identically behave identically (the Lua one against these defaults).
    std::string describe() const;
};

// The built-in catalog: today's outings, lunch and students' breaks exactly (menus "outing", "lunch", "student_break").
ActivityCatalog defaultActivityCatalog();
// The minutes a site kind keeps a visitor by default: {lo, hi}.
void siteKindMinutes(const std::string& kind, double& lo, double& hi);
// Is the hour inside [lo, hi) (wrapping midnight when lo > hi)?
inline bool hourIn(double h, double lo, double hi) { return lo <= hi ? (h >= lo && h < hi) : (h >= lo || h < hi); }

}  // namespace citysim

#endif
