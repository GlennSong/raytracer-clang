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

// --- named types (ADR-0253): an event described through the property layer reaches JSON callers by name ---

#include "../src/engine/event_types.h"
#include <nlohmann/json.hpp>

namespace {
struct Hop {
    engine::Vec3 to{1, 2, 3};
    int agent = -1;
    bool run = false;
    std::string why = "test";
};
// Found by argument-dependent lookup, beside the type, as an engine event's is beside it in namespace engine.
void describeProperties(Hop& h, engine::PropertyVisitor& v) {
    using engine::FieldMeta;
    v.field(FieldMeta("to"), h.to);
    v.field(FieldMeta("agent"), h.agent);
    v.field(FieldMeta("run"), h.run);
    v.field(FieldMeta("why"), h.why);
}
void describeProperties(Ping& p, engine::PropertyVisitor& v) {
    v.field(engine::FieldMeta("value"), p.value);
}
}  // namespace

TEST_CASE(event_bus_named_type_publishes_from_json_and_keeps_defaults) {
    EventBus bus;
    CHECK(engine::describeEvent<Hop>(bus, "Hop", "Jump somewhere."));
    Hop got;
    int n = 0;
    bus.subscribe<Hop>([&](const Hop& h) { got = h; ++n; });
    std::string error;
    CHECK(bus.publishNamed("Hop", R"({"agent": 7, "run": true})", false, &error));
    CHECK(n == 1);
    CHECK(got.agent == 7 && got.run);
    CHECK(got.to.y == 2 && got.why == "test");   // not given: E's defaults
    // Queued: delivered at the frame's dispatch, not before.
    CHECK(bus.publishNamed("Hop", "{}", true, &error));
    CHECK(n == 1);
    bus.dispatchQueued();
    CHECK(n == 2);
}

TEST_CASE(event_bus_named_type_hands_subscribers_json) {
    EventBus bus;
    engine::describeEvent<Hop>(bus, "Hop", "Jump somewhere.");
    nlohmann::json seen;
    const EventBus::SubscriptionId id = bus.subscribeNamed("Hop", [&](const std::string& j) { seen = nlohmann::json::parse(j); });
    CHECK(id != EventBus::INVALID_SUBSCRIPTION);
    CHECK(bus.namedSubscriberCount("Hop") == 1);
    Hop h;
    h.agent = 12;
    h.to = engine::Vec3(4, 5, 6);
    bus.publish(h);
    CHECK(seen["agent"] == 12);   // an integer, not 12.0
    CHECK(seen["agent"].is_number_integer());
    CHECK(seen["to"][2] == 6.0);
    CHECK(seen["why"] == "test");
    CHECK(bus.subscribeNamed("Nope", [](const std::string&) {}) == EventBus::INVALID_SUBSCRIPTION);
}

TEST_CASE(event_bus_named_type_refuses_a_second_type_and_bad_fields) {
    EventBus bus;
    CHECK(engine::describeEvent<Hop>(bus, "Hop", "Jump somewhere."));
    CHECK(engine::describeEvent<Hop>(bus, "Hop", "Again, the same type: fine."));
    CHECK(!engine::describeEvent<Ping>(bus, "Hop", "Another type under the name."));
    std::string error;
    CHECK(!bus.publishNamed("Hop", R"({"agent": "seven"})", false, &error));
    CHECK(!error.empty());
    CHECK(!bus.publishNamed("Hop", "[1, 2]", false, &error));
    CHECK(!bus.publishNamed("Nope", "{}", false, &error));
    CHECK(error.find("unknown event") != std::string::npos);
    CHECK(bus.namedTypes().size() == 1);
}
