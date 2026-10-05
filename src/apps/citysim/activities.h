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

}  // namespace citysim

#endif
