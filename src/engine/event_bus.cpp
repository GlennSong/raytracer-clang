#include "event_bus.h"
#include "../log.h"

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

std::size_t EventBus::countLive(std::type_index type) const {
    std::unordered_map<std::type_index, std::vector<Handler>>::const_iterator it = handlers.find(type);
    if (it == handlers.end()) return 0;
    std::size_t count = 0;
    for (const auto& handler : it->second)
        if (handler.fn) ++count;
    return count;
}

bool EventBus::addNamedType(NamedType type) {
    for (const NamedType& t : named) {
        if (t.name != type.name) continue;
        if (t.type == type.type) return true;
        LOG_ERROR << "[events] '" << type.name << "' already names another event type";
        return false;
    }
    named.push_back(std::move(type));
    return true;
}

std::vector<const EventBus::NamedType*> EventBus::namedTypes() const {
    std::vector<const NamedType*> out;
    for (const NamedType& t : named) out.push_back(&t);
    std::sort(out.begin(), out.end(), [](const NamedType* a, const NamedType* b) { return a->name < b->name; });
    return out;
}

const EventBus::NamedType* EventBus::namedType(const std::string& name) const {
    for (const NamedType& t : named)
        if (t.name == name) return &t;
    return nullptr;
}

bool EventBus::publishNamed(const std::string& name, const std::string& fieldsJson, bool queued, std::string* error) {
    const NamedType* t = namedType(name);
    if (t == nullptr) {
        if (error) *error = "unknown event: " + name + " (try events?)";
        return false;
    }
    // Copy the door first: a handler may describe a new type, which can move `named`.
    const std::function<bool(EventBus&, const std::string&, bool, std::string*)> publish = t->publish;
    std::string why;
    if (publish(*this, fieldsJson, queued, &why)) return true;
    if (error) *error = name + ": " + why;
    return false;
}

EventBus::SubscriptionId EventBus::subscribeNamed(const std::string& name, JsonHandler handler) {
    const NamedType* t = namedType(name);
    return t == nullptr ? INVALID_SUBSCRIPTION : t->subscribe(*this, std::move(handler));
}

std::size_t EventBus::namedSubscriberCount(const std::string& name) const {
    const NamedType* t = namedType(name);
    return t == nullptr ? 0 : countLive(t->type);
}

}  // namespace engine
