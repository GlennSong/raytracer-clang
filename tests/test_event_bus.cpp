#include "test_framework.h"
#include "../src/engine/event_bus.h"

#include <string>
#include <vector>

using engine::EventBus;

namespace {

struct Ping { int value = 0; };
struct Chat { std::string text; };

}  // namespace

TEST_CASE(event_bus_delivers_to_subscriber) {
    EventBus bus;
    int received = 0;
    bus.subscribe<Ping>([&](const Ping& p) { received = p.value; });
    bus.publish(Ping{42});
    CHECK(received == 42);
}

TEST_CASE(event_bus_types_are_isolated) {
    EventBus bus;
    int pings = 0, chats = 0;
    bus.subscribe<Ping>([&](const Ping&) { ++pings; });
    bus.subscribe<Chat>([&](const Chat&) { ++chats; });
    bus.publish(Ping{});
    bus.publish(Ping{});
    bus.publish(Chat{"hi"});
    CHECK(pings == 2);
    CHECK(chats == 1);
}

TEST_CASE(event_bus_publish_without_subscribers_is_safe) {
    EventBus bus;
    bus.publish(Ping{7});
    CHECK(bus.subscriberCount<Ping>() == 0);
}

TEST_CASE(event_bus_handlers_run_in_subscription_order) {
    EventBus bus;
    std::vector<int> order;
    bus.subscribe<Ping>([&](const Ping&) { order.push_back(1); });
    bus.subscribe<Ping>([&](const Ping&) { order.push_back(2); });
    bus.subscribe<Ping>([&](const Ping&) { order.push_back(3); });
    bus.publish(Ping{});
    CHECK(order.size() == 3);
    CHECK(order[0] == 1);
    CHECK(order[1] == 2);
    CHECK(order[2] == 3);
}

TEST_CASE(event_bus_unsubscribe_stops_delivery) {
    EventBus bus;
    int count = 0;
    auto id = bus.subscribe<Ping>([&](const Ping&) { ++count; });
    bus.publish(Ping{});
    bus.unsubscribe(id);
    bus.publish(Ping{});
    CHECK(count == 1);
    CHECK(bus.subscriberCount<Ping>() == 0);
}

TEST_CASE(event_bus_unknown_unsubscribe_is_ignored) {
    EventBus bus;
    bus.unsubscribe(EventBus::INVALID_SUBSCRIPTION);
    bus.unsubscribe(9999);
    bus.subscribe<Ping>([](const Ping&) {});
    bus.unsubscribe(9999);
    CHECK(bus.subscriberCount<Ping>() == 1);
}

TEST_CASE(event_bus_self_unsubscribe_during_publish_is_safe) {
    EventBus bus;
    int first = 0, second = 0;
    EventBus::SubscriptionId firstId = EventBus::INVALID_SUBSCRIPTION;
    firstId = bus.subscribe<Ping>([&](const Ping&) {
        ++first;
        bus.unsubscribe(firstId);   // remove myself mid-dispatch
    });
    bus.subscribe<Ping>([&](const Ping&) { ++second; });
    bus.publish(Ping{});
    bus.publish(Ping{});
    CHECK(first == 1);    // gone after the first delivery
    CHECK(second == 2);   // later handler unaffected both times
    CHECK(bus.subscriberCount<Ping>() == 1);
}

TEST_CASE(event_bus_subscribe_during_publish_sees_next_event) {
    EventBus bus;
    int lateCount = 0;
    bool subscribed = false;
    bus.subscribe<Ping>([&](const Ping&) {
        if (!subscribed) {
            subscribed = true;
            bus.subscribe<Ping>([&](const Ping&) { ++lateCount; });
        }
    });
    bus.publish(Ping{});          // late handler added here, must not fire
    CHECK(lateCount == 0);
    bus.publish(Ping{});          // fires now
    CHECK(lateCount == 1);
}

TEST_CASE(event_bus_nested_publish_from_handler) {
    EventBus bus;
    std::vector<std::string> log;
    bus.subscribe<Ping>([&](const Ping&) {
        log.push_back("ping");
        bus.publish(Chat{"from-ping"});
    });
    bus.subscribe<Chat>([&](const Chat& c) { log.push_back(c.text); });
    bus.publish(Ping{});
    CHECK(log.size() == 2);
    CHECK(log[0] == "ping");
    CHECK(log[1] == "from-ping");
}

TEST_CASE(event_bus_enqueue_defers_until_dispatch) {
    EventBus bus;
    int received = 0;
    bus.subscribe<Ping>([&](const Ping& p) { received = p.value; });
    bus.enqueue(Ping{5});
    CHECK(received == 0);
    CHECK(bus.queuedCount() == 1);
    bus.dispatchQueued();
    CHECK(received == 5);
    CHECK(bus.queuedCount() == 0);
}

TEST_CASE(event_bus_enqueue_during_dispatch_runs_next_batch) {
    EventBus bus;
    int count = 0;
    bus.subscribe<Ping>([&](const Ping&) {
        ++count;
        if (count == 1) bus.enqueue(Ping{});   // chain one follow-up
    });
    bus.enqueue(Ping{});
    bus.dispatchQueued();
    CHECK(count == 1);              // the chained event did NOT run this batch
    CHECK(bus.queuedCount() == 1);
    bus.dispatchQueued();
    CHECK(count == 2);
    CHECK(bus.queuedCount() == 0);  // and it didn't chain again
}
