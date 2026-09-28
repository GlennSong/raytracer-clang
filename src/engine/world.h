#ifndef RAYTRACER_ENGINE_WORLD_H
#define RAYTRACER_ENGINE_WORLD_H

#include "../handle.h"
#include "../slot_map.h"
#include "sparse_set.h"
#include <memory>
#include <unordered_map>
#include <tuple>
#include <cstdint>
#include <cstddef>

namespace engine {

struct EntityTag {};
using Entity = Handle<EntityTag>;

// Entity-component registry. Entities are recyclable handles (backed by a
// SlotMap, so stale handles are detected by generation); components live in
// per-type sparse sets. Systems operate over them via each().
class World {
public:
    Entity create();
    void destroy(Entity entity);
    bool alive(Entity entity) const;
    // Destroy every live entity (e.g. before loading a different level).
    // Goes through destroy() so generations bump: handles held across the
    // clear are detectably stale, never silently reused.
    void destroyAll();
    std::size_t entityCount() const { return entities.size(); }

    template <typename T>
    T& add(Entity entity, const T& value = T()) {
        return pool<T>().insert(entity.index, value);
    }

    template <typename T>
    void remove(Entity entity) {
        if (auto* p = poolIfExists<T>()) p->removeIndex(entity.index);
    }

    template <typename T>
    T* get(Entity entity) {
        if (!alive(entity)) return nullptr;
        auto* p = poolIfExists<T>();
        return p ? p->get(entity.index) : nullptr;
    }

    template <typename T>
    bool has(Entity entity) {
        if (!alive(entity)) return false;
        auto* p = poolIfExists<T>();
        return p && p->containsIndex(entity.index);
    }

    // Invoke fn(Entity, Ts&...) for each live entity having all of Ts.
    // Contract: do NOT create/destroy entities or add/remove any of the
    // iterated component types inside the callback — collect and apply after.
    template <typename... Ts, typename Fn>
    void each(Fn fn) {
        static_assert(sizeof...(Ts) > 0, "each requires at least one component type");
        using First = typename std::tuple_element<0, std::tuple<Ts...>>::type;
        SparseSet<First>* firstPool = poolIfExists<First>();
        if (!firstPool) return;

        // Drive the walk from the SMALLEST pool: each<Transform, ControlledBy> used to walk every
        // Transform in the city (tens of thousands) to find the one player -- ~0.2 ms in each of
        // the half-dozen systems that ask, every fixed step.
        const std::vector<uint32_t>* drive = &firstPool->entityIndices();
        bool missing = false;
        auto consider = [&](const auto* p) {
            if (!p) { missing = true; return; }
            if (p->entityIndices().size() < drive->size()) drive = &p->entityIndices();
        };
        (consider(poolIfExists<Ts>()), ...);
        if (missing) return;
        if (drive == &firstPool->entityIndices()) {
            const std::vector<uint32_t>& indices = *drive;
            for (std::size_t i = 0; i < indices.size(); i++) {
                uint32_t index = indices[i];
                if (!(poolHas<Ts>(index) && ...)) continue;
                fn(entities.handleAt(index), *poolIfExists<Ts>()->get(index)...);
            }
            return;
        }
        // Another type's pool: walk a copy, so `fn` may add or remove that component safely.
        const std::vector<uint32_t> indices = *drive;
        for (uint32_t index : indices) {
            if (!(poolHas<Ts>(index) && ...)) continue;
            fn(entities.handleAt(index), *poolIfExists<Ts>()->get(index)...);
        }
    }

private:
    struct EntityMeta {};
    using ComponentId = uint32_t;

    static ComponentId nextComponentId() {
        static ComponentId id = 0;
        return id++;
    }
    template <typename T>
    static ComponentId componentId() {
        static ComponentId id = nextComponentId();
        return id;
    }

    template <typename T>
    SparseSet<T>& pool() {
        ComponentId id = componentId<T>();
        auto it = pools.find(id);
        if (it == pools.end()) {
            auto set = std::make_unique<SparseSet<T>>();
            SparseSet<T>& ref = *set;
            pools.emplace(id, std::move(set));
            return ref;
        }
        return *static_cast<SparseSet<T>*>(it->second.get());
    }

    template <typename T>
    SparseSet<T>* poolIfExists() {
        auto it = pools.find(componentId<T>());
        return it == pools.end() ? nullptr
                                 : static_cast<SparseSet<T>*>(it->second.get());
    }

    template <typename T>
    bool poolHas(uint32_t index) {
        auto* p = poolIfExists<T>();
        return p && p->containsIndex(index);
    }

    SlotMap<EntityMeta, EntityTag> entities;
    std::unordered_map<ComponentId, std::unique_ptr<ISparseSet>> pools;
};


}  // namespace engine

#endif
