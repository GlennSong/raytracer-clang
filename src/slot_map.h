#ifndef RAYTRACER_SLOT_MAP_H
#define RAYTRACER_SLOT_MAP_H

#include "handle.h"
#include <vector>
#include <cstddef>
#include <cstdint>

namespace engine {

// Stable handles into recycling storage. insert() returns a Handle valid until
// erase(); freed slots are reused by later inserts with a bumped generation, so
// a handle to a freed (and possibly reused) slot is reported stale rather than
// silently aliasing a new value. T must be a value type (copyable). Tag
// defaults to T so each element type gets its own distinct Handle type.
template <typename T, typename Tag = T>
class SlotMap {
public:
    using HandleType = Handle<Tag>;

    HandleType insert(const T& value) {
        uint32_t index;
        if (freeHead != NO_SLOT) {
            index = freeHead;
            freeHead = slots[index].nextFree;
            slots[index].value = value;
            slots[index].occupied = true;
        } else {
            index = static_cast<uint32_t>(slots.size());
            slots.push_back(Slot{1, NO_SLOT, true, value});
        }
        count++;

        HandleType handle;
        handle.index = index;
        handle.generation = slots[index].generation;
        return handle;
    }

    bool erase(HandleType handle) {
        Slot* slot = slotFor(handle);
        if (!slot) return false;
        slot->occupied = false;
        slot->generation++;                              // invalidate handles
        if (slot->generation == 0) slot->generation = 1; // never wrap to null
        slot->nextFree = freeHead;
        freeHead = handle.index;
        count--;
        return true;
    }

    T* get(HandleType handle) {
        Slot* slot = slotFor(handle);
        return slot ? &slot->value : nullptr;
    }
    const T* get(HandleType handle) const {
        const Slot* slot = slotFor(handle);
        return slot ? &slot->value : nullptr;
    }

    bool contains(HandleType handle) const { return slotFor(handle) != nullptr; }
    std::size_t size() const { return count; }

    // Drop all elements. Existing handles become stale (their slots no longer
    // exist), so access through them is reported null rather than aliasing.
    void clear() {
        slots.clear();
        freeHead = NO_SLOT;
        count = 0;
    }

    // The live handle occupying a slot, or a null handle if the slot is empty.
    // Lets callers that track raw indices (e.g. an ECS) recover a full handle.
    HandleType handleAt(uint32_t index) const {
        HandleType handle;
        if (index < slots.size() && slots[index].occupied) {
            handle.index = index;
            handle.generation = slots[index].generation;
        }
        return handle;
    }

    // Visit every live element as fn(HandleType, T&).
    template <typename Fn>
    void forEach(Fn fn) {
        for (uint32_t i = 0; i < slots.size(); i++) {
            if (!slots[i].occupied) continue;
            HandleType handle;
            handle.index = i;
            handle.generation = slots[i].generation;
            fn(handle, slots[i].value);
        }
    }

private:
    static constexpr uint32_t NO_SLOT = 0xFFFFFFFFu;

    struct Slot {
        uint32_t generation;
        uint32_t nextFree;
        bool occupied;
        T value;
    };

    Slot* slotFor(HandleType handle) {
        if (handle.generation == 0 || handle.index >= slots.size()) return nullptr;
        Slot& slot = slots[handle.index];
        if (!slot.occupied || slot.generation != handle.generation) return nullptr;
        return &slot;
    }
    const Slot* slotFor(HandleType handle) const {
        if (handle.generation == 0 || handle.index >= slots.size()) return nullptr;
        const Slot& slot = slots[handle.index];
        if (!slot.occupied || slot.generation != handle.generation) return nullptr;
        return &slot;
    }

    std::vector<Slot> slots;
    uint32_t freeHead = NO_SLOT;
    std::size_t count = 0;
};

}  // namespace engine

#endif
