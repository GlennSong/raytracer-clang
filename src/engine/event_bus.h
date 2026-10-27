#ifndef RAYTRACER_ENGINE_EVENT_BUS_H
#define RAYTRACER_ENGINE_EVENT_BUS_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine {

// Typed publish/subscribe bus (ADR-0066): the decoupling seam for "system A
// announces, system B reacts" — sound cues, gameplay signals, UI notifications.
// An event is any copyable struct; subscribers register a callback per event
// type. Delivery is synchronous and in subscription order (deterministic, fits
// ADR-0002); enqueue() defers a publish to the next dispatchQueued(), which the
// Application calls once per frame after the fixed steps.
//
// Reentrancy rules (enforced, tested):
// - unsubscribe() during a publish is safe — the handler slot is nulled and
//   swept after the dispatch completes.
// - subscribe() during a publish is safe — the new handler sees the NEXT
//   publish, not the one in flight.
// - publish() from inside a handler is allowed (nested dispatch); enqueue()
//   during dispatchQueued() lands in the next frame's batch.
//
// Not thread-safe by design: publish/subscribe from the sim/main thread only
// (same rule as World). Cross-thread producers enqueue via their own staging
// and hand over on the main thread.
//
// Named types (ADR-0253): an event type described with describeEvent<E>()
// (event_types.h) also has a name and its fields, through the property layer,
// so Lua (events.publish / events.on), the control channel (`watch`, `events?`)
// and tools reach it by name with its fields as JSON. C++ keeps the typed calls. The JSON crosses this header
// as text (the vendored JSON library has no forward header); event_types.h does the parsing.
class EventBus {
public:
    using SubscriptionId = uint64_t;
    static constexpr SubscriptionId INVALID_SUBSCRIPTION = 0;
    using JsonHandler = std::function<void(const std::string& fieldsJson)>;

    // A named type: its name, one line on what it means, and the type-erased doors describeEvent<E> fills.
    struct NamedType {
        std::string name;
        std::string summary;
        std::type_index type = std::type_index(typeid(void));
        // The fields of a default event, as a JSON object (the keys and their defaults, for `events?`).
        std::function<std::string()> fields;
        // Build an E from a JSON object (missing fields keep their defaults) and publish it now, or enqueue it.
        // False with `error` set when the text is not an object or a field has the wrong kind.
        std::function<bool(EventBus&, const std::string& fieldsJson, bool queued, std::string* error)> publish;
        // Subscribe to E, handing each one over as JSON.
        std::function<SubscriptionId(EventBus&, JsonHandler)> subscribe;
    };

    template <typename E>
    SubscriptionId subscribe(std::function<void(const E&)> handler) {
        SubscriptionId id = nextId++;
        handlers[std::type_index(typeid(E))].push_back(
            {id, [fn = std::move(handler)](const void* event) {
                 fn(*static_cast<const E*>(event));
             }});
        return id;
    }

    // Remove a subscription by id. Unknown/expired ids are ignored, so a
    // subscriber can unsubscribe unconditionally in its teardown.
    void unsubscribe(SubscriptionId id);

    // A subscription that ends when it is destroyed: a system keeps these as members, so it stops hearing
    // events when it goes (the bus must outlive it; Application declares it before the state stack).
    class Subscription {
    public:
        Subscription() = default;
        Subscription(EventBus* bus, SubscriptionId id) : bus_(bus), id_(id) {}
        ~Subscription() { reset(); }
        Subscription(Subscription&& o) noexcept : bus_(o.bus_), id_(o.id_) { o.bus_ = nullptr; }
        Subscription& operator=(Subscription&& o) noexcept {
            if (this != &o) { reset(); bus_ = o.bus_; id_ = o.id_; o.bus_ = nullptr; }
            return *this;
        }
        Subscription(const Subscription&) = delete;
        Subscription& operator=(const Subscription&) = delete;
        void reset() {
            if (bus_ != nullptr) bus_->unsubscribe(id_);
            bus_ = nullptr;
        }

    private:
        EventBus* bus_ = nullptr;
        SubscriptionId id_ = INVALID_SUBSCRIPTION;
    };
    template <typename E>
    Subscription listen(std::function<void(const E&)> handler) {
        return Subscription(this, subscribe<E>(std::move(handler)));
    }

    // Deliver `event` to every current subscriber of E, in subscription order.
    template <typename E>
    void publish(const E& event) {
        auto it = handlers.find(std::type_index(typeid(E)));
        if (it == handlers.end()) return;
        dispatchTo(it->second, &event);
    }

    // Defer a publish to the next dispatchQueued(). The event is copied into
    // the queue, so the caller's instance can die immediately.
    template <typename E>
    void enqueue(E event) {
        queued.push_back([e = std::move(event)](EventBus& bus) { bus.publish(e); });
    }

    // Deliver everything enqueued since the last call. Events enqueued while
    // dispatching (a handler chaining a new event) run in the NEXT call — one
    // batch per frame, no unbounded cascades.
    void dispatchQueued();

    template <typename E>
    std::size_t subscriberCount() const {
        return countLive(std::type_index(typeid(E)));
    }

    std::size_t queuedCount() const { return queued.size(); }

    // Name a type (describeEvent<E> calls this). Describing a name again for the same type is a no-op; for a
    // different type it is refused (false, logged).
    bool addNamedType(NamedType type);
    // Every named type, sorted by name.
    std::vector<const NamedType*> namedTypes() const;
    const NamedType* namedType(const std::string& name) const;
    // Publish (or enqueue) a named event from JSON. False with `error` set when the name is unknown.
    bool publishNamed(const std::string& name, const std::string& fieldsJson, bool queued, std::string* error = nullptr);
    // Subscribe to a named event as JSON. INVALID_SUBSCRIPTION when the name is unknown.
    SubscriptionId subscribeNamed(const std::string& name, JsonHandler handler);
    // How many live subscribers a named type has (0 when unknown).
    std::size_t namedSubscriberCount(const std::string& name) const;

private:
    struct Handler {
        SubscriptionId id;
        std::function<void(const void*)> fn;   // null = unsubscribed mid-dispatch
    };

    void dispatchTo(std::vector<Handler>& list, const void* event);
    void sweep(std::vector<Handler>& list);

    std::size_t countLive(std::type_index type) const;

    std::unordered_map<std::type_index, std::vector<Handler>> handlers;
    std::vector<NamedType> named;
    std::vector<std::function<void(EventBus&)>> queued;
    SubscriptionId nextId = 1;
    int dispatchDepth = 0;
};


}  // namespace engine

#endif
