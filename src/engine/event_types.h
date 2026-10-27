#ifndef RAYTRACER_ENGINE_EVENT_TYPES_H
#define RAYTRACER_ENGINE_EVENT_TYPES_H

#include "event_bus.h"
#include "property_json.h"

#include <nlohmann/json.hpp>

namespace engine {

// Name an event type on a bus (ADR-0253): E is any copyable, default-constructible struct with a
// describeProperties(E&, PropertyVisitor&) overload, its fields described once through the property layer. The
// name then reaches it from JSON: Lua's events.publish / events.on, the control channel's `watch` and `events?`.
// Fields missing from the JSON keep E's defaults.
template <typename E>
bool describeEvent(EventBus& bus, const char* name, const char* summary) {
    EventBus::NamedType t;
    t.name = name;
    t.summary = summary;
    t.type = std::type_index(typeid(E));
    t.fields = [] {
        E e;
        nlohmann::json j = nlohmann::json::object();
        JsonWriteVisitor w(j);
        describeProperties(e, w);
        return j.dump();
    };
    t.publish = [](EventBus& b, const std::string& text, bool queued, std::string* error) {
        E e;
        if (!text.empty()) {
            const nlohmann::json fields = nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false);
            if (fields.is_discarded() || !fields.is_object()) {
                *error = "the fields are not a JSON object";
                return false;
            }
            try {
                JsonReadVisitor r(fields);
                describeProperties(e, r);
            } catch (const nlohmann::json::exception& ex) {   // a field of the wrong kind
                *error = ex.what();
                return false;
            }
        }
        if (queued) b.enqueue(e);
        else b.publish(e);
        return true;
    };
    t.subscribe = [](EventBus& b, EventBus::JsonHandler handler) {
        return b.subscribe<E>([fn = std::move(handler)](const E& event) {
            E e = event;   // the visitor takes a reference it could write through
            nlohmann::json j = nlohmann::json::object();
            JsonWriteVisitor w(j);
            describeProperties(e, w);
            fn(j.dump());
        });
    };
    return bus.addNamedType(std::move(t));
}

struct PlaySound;
void describeProperties(PlaySound& e, PropertyVisitor& v);

// The engine's own named events (PlaySound, ...), described on `bus`. Application calls it once.
void describeEngineEvents(EventBus& bus);

}  // namespace engine

#endif
