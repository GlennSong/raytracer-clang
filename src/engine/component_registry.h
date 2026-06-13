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
        // Optional thunks; null means the operation isn't offered in UIs.
        // `addTo` default-constructs the component; `removeFrom` detaches it.
        std::function<void(World&, Entity)> addTo;
        std::function<void(World&, Entity)> removeFrom;
        // Post-edit hook, invoked by inspectors AFTER a field write goes
        // through. `label` is the edited FieldMeta label, or nullptr when the
        // whole component was restored at once (undo, paste). Installed by
        // whoever owns the needed services (e.g. the editor wires a mesh
        // rebuild for Shape size — registration time has no renderer).
        std::function<void(World&, Entity, const char* label)> onEdited;
    };

    // T must have a describeProperties(T&, PropertyVisitor&) overload (or
    // pass a custom accessor for components whose editable data is nested,
    // e.g. Renderable -> its material).
    template <typename T>
    Entry& add(std::string name) {
        return addWithAccessor<T>(std::move(name),
                                  [](T& component, PropertyVisitor& v) {
                                      describeProperties(component, v);
                                  });
    }

    template <typename T, typename Fn>
    Entry& addWithAccessor(std::string name, Fn accessor) {
        Entry entry;
        entry.name = std::move(name);
        entry.has = [](World& world, Entity e) { return world.has<T>(e); };
        entry.visit = [accessor](World& world, Entity e, PropertyVisitor& v) {
            if (T* component = world.get<T>(e)) accessor(*component, v);
        };
        entries_.push_back(std::move(entry));
        return entries_.back();
    }

    // Marker components (PlayerSpawn): present in the list, nothing to edit.
    template <typename T>
    Entry& addMarker(std::string name) {
        return addWithAccessor<T>(std::move(name), [](T&, PropertyVisitor&) {});
    }

    // Default add/remove thunks for components an "Add Component" menu may
    // attach or detach freely (no construction-time services needed).
    template <typename T>
    Entry& allowAddRemove(Entry& entry) {
        entry.addTo = [](World& world, Entity e) {
            if (!world.has<T>(e)) world.add<T>(e);
        };
        entry.removeFrom = [](World& world, Entity e) { world.remove<T>(e); };
        return entry;
    }

    const std::vector<Entry>& entries() const { return entries_; }
    Entry* find(const std::string& name) {
        for (Entry& e : entries_)
            if (e.name == name) return &e;
        return nullptr;
    }

private:
    std::vector<Entry> entries_;
};

// The engine's built-in component set, in inspector display order.
void registerEngineComponents(ComponentRegistry& registry);

}  // namespace engine

#endif
