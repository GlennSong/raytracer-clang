#ifndef RAYTRACER_ENGINE_COMPONENT_REGISTRY_H
#define RAYTRACER_ENGINE_COMPONENT_REGISTRY_H

#include "properties.h"
#include "world.h"
#include <functional>
#include <string>
#include <vector>

namespace engine {

// Runtime enumeration of component types — what C++ can't do by itself and a
// Unity-style inspector needs: "which components does this entity have, and
// walk each one's fields". One registration line per component type; the
// inspector, and eventually generic serialization, never name concrete
// components again. Game code can register its own types alongside the
// engine's (registerEngineComponents).
class ComponentRegistry {
public:
    struct Entry {
        std::string name;
        std::function<bool(World&, Entity)> has;
        std::function<void(World&, Entity, PropertyVisitor&)> visit;
    };

    // T must have a describeProperties(T&, PropertyVisitor&) overload (or
    // pass a custom accessor for components whose editable data is nested,
    // e.g. Renderable -> its material).
    template <typename T>
    void add(std::string name) {
        addWithAccessor<T>(std::move(name),
                           [](T& component, PropertyVisitor& v) {
                               describeProperties(component, v);
                           });
    }

    template <typename T, typename Fn>
    void addWithAccessor(std::string name, Fn accessor) {
        Entry entry;
        entry.name = std::move(name);
        entry.has = [](World& world, Entity e) { return world.has<T>(e); };
        entry.visit = [accessor](World& world, Entity e, PropertyVisitor& v) {
            if (T* component = world.get<T>(e)) accessor(*component, v);
        };
        entries_.push_back(std::move(entry));
    }

    // Marker components (PlayerSpawn): present in the list, nothing to edit.
    template <typename T>
    void addMarker(std::string name) {
        addWithAccessor<T>(std::move(name), [](T&, PropertyVisitor&) {});
    }

    const std::vector<Entry>& entries() const { return entries_; }

private:
    std::vector<Entry> entries_;
};

// The engine's built-in component set, in inspector display order.
void registerEngineComponents(ComponentRegistry& registry);

}  // namespace engine

#endif
