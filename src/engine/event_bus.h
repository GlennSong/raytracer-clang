#ifndef RAYTRACER_ENGINE_EVENT_BUS_H
#define RAYTRACER_ENGINE_EVENT_BUS_H

#include <cstddef>
#include <cstdint>
#include <functional>
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
class EventBus {
public:
    using SubscriptionId = uint64_t;
    static constexpr SubscriptionId INVALID_SUBSCRIPTION = 0;

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
        auto it = handlers.find(std::type_index(typeid(E)));
        if (it == handlers.end()) return 0;
        std::size_t count = 0;
        for (const auto& handler : it->second)
            if (handler.fn) ++count;
        return count;
    }

    std::size_t queuedCount() const { return queued.size(); }

private:
    struct Handler {
        SubscriptionId id;
        std::function<void(const void*)> fn;   // null = unsubscribed mid-dispatch
    };

    void dispatchTo(std::vector<Handler>& list, const void* event);
    void sweep(std::vector<Handler>& list);

    std::unordered_map<std::type_index, std::vector<Handler>> handlers;
    std::vector<std::function<void(EventBus&)>> queued;
    SubscriptionId nextId = 1;
    int dispatchDepth = 0;
};


}  // namespace engine

#endif
