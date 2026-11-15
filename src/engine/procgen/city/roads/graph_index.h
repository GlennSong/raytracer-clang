#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GRAPH_INDEX_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GRAPH_INDEX_H

// The index types the lane graph (lane_graph.h) and the road model (road_model_parts.h) share.

#include <cstdint>

namespace engine {

// AN INDEX INTO ONE OF THE GRAPH'S ARRAYS, typed by what it points at, so a path's index cannot be passed where a
// junction's is wanted. kNone is "no such thing" (a lane's junction, a connector's section). (handle.h's Handle is
// for slots that are freed and reused, and carries a generation; the graph never frees, so it would only double the
// size of every reference.)
constexpr uint32_t kNoIndex = 0xFFFFFFFFu;

template <typename Tag>
struct GraphIndex {
    static constexpr uint32_t kNone = kNoIndex;
    uint32_t index = kNone;

    GraphIndex() = default;
    explicit GraphIndex(uint32_t i) : index(i) {}
    bool valid() const { return index != kNone; }
    bool operator==(const GraphIndex& o) const { return index == o.index; }
    bool operator!=(const GraphIndex& o) const { return index != o.index; }
    bool operator<(const GraphIndex& o) const { return index < o.index; }
};

// A stretch of one of the graph's shared arrays: `count` entries from `first`.
struct Range {
    uint32_t first = 0;
    uint32_t count = 0;
    uint32_t end() const { return first + count; }
    bool operator==(const Range& o) const { return first == o.first && count == o.count; }
    bool operator!=(const Range& o) const { return !(*this == o); }
};

}  // namespace engine

#endif
