#include "event_bus.h"

#include <algorithm>

namespace engine {

void EventBus::unsubscribe(SubscriptionId id) {
    if (id == INVALID_SUBSCRIPTION) return;
    for (auto& [type, list] : handlers) {
        (void)type;
        for (auto& handler : list) {
            if (handler.id != id) continue;
            if (dispatchDepth > 0) {
                handler.fn = nullptr;   // swept once the dispatch unwinds
            } else {
                list.erase(list.begin() + (&handler - list.data()));
            }
            return;
        }
    }
}

void EventBus::dispatchQueued() {
    // Swap the batch out first: enqueues from inside a handler land in the
    // fresh `queued` and run next call, so a chain can't spin forever.
    std::vector<std::function<void(EventBus&)>> batch;
    batch.swap(queued);
    for (auto& deliver : batch) deliver(*this);
}

void EventBus::dispatchTo(std::vector<Handler>& list, const void* event) {
    ++dispatchDepth;
    // Snapshot the size: handlers subscribed during this dispatch see the next
    // publish. Index access (not iterators) survives the reallocation an
    // append causes, and the copy keeps the callable alive if the list moves
    // while it runs.
    const std::size_t count = list.size();
    for (std::size_t i = 0; i < count; ++i) {
        auto fn = list[i].fn;
        if (fn) fn(event);
    }
    --dispatchDepth;
    if (dispatchDepth == 0) sweep(list);
}

void EventBus::sweep(std::vector<Handler>& list) {
    list.erase(std::remove_if(list.begin(), list.end(),
                              [](const Handler& h) { return !h.fn; }),
               list.end());
}

}  // namespace engine
