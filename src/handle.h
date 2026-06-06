#ifndef RAYTRACER_HANDLE_H
#define RAYTRACER_HANDLE_H

#include <cstdint>

namespace engine {

// A type-safe, recyclable identity into a SlotMap. The Tag parameter makes
// Handle<Mesh> a distinct type from Handle<Entity> at compile time. A handle is
// null when generation == 0; a non-null handle goes stale once its slot is
// freed and reused (SlotMap bumps the generation on erase), so the staleness is
// detectable rather than a silent alias.
template <typename Tag>
struct Handle {
    uint32_t index = 0;
    uint32_t generation = 0;

    bool valid() const { return generation != 0; }

    bool operator==(const Handle& other) const {
        return index == other.index && generation == other.generation;
    }
    bool operator!=(const Handle& other) const { return !(*this == other); }
    bool operator<(const Handle& other) const {
        return index < other.index ||
               (index == other.index && generation < other.generation);
    }
};

}  // namespace engine

#endif
