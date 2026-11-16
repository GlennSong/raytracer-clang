#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GRAPH_INDEX_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GRAPH_INDEX_H

// The index types the lane graph (lane_graph.h) and the road model (road_model_parts.h) share, and the view of a
// stretch of an array (Span).

#include <cstddef>
#include <cstdint>
#include <vector>

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

// THE ENTRIES A RANGE NAMES, READ IN PLACE: a pointer to the first and how many, with the reading half of a vector's
// interface (size, [], front, back, begin, end). It owns nothing and copies nothing: it is valid while the array it
// looks into is alive and is not appended to. A whole vector reads as one too, so a function that takes a Span accepts
// either. (C++20's std::span; this build is C++17.) What a lane's centre line is handed out as (lanes/lane_expand.h
// Lane::xy, s), so the builder and the road model read the same points (ADR-0438).
template <typename T>
struct Span {
    const T* first = nullptr;
    std::size_t count = 0;

    Span() = default;
    Span(const T* f, std::size_t n) : first(f), count(n) {}
    Span(const std::vector<T>& v) : first(v.data()), count(v.size()) {}   // (not explicit: a vector is passed as it is)
    Span(const std::vector<T>& v, const Range& r) : first(v.data() + r.first), count(r.count) {}
    std::size_t size() const { return count; }
    bool empty() const { return count == 0; }
    const T* data() const { return first; }
    const T* begin() const { return first; }
    const T* end() const { return first + count; }
    const T& operator[](std::size_t i) const { return first[i]; }
    const T& front() const { return first[0]; }
    const T& back() const { return first[count - 1]; }
    std::vector<T> copied() const { return std::vector<T>(first, first + count); }
};

}  // namespace engine

#endif
