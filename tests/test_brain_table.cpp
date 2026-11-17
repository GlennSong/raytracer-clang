// THE AGENT BRAIN'S DATA (ADR-0440, src/apps/citysim/brain): the record's size; the shipped brain.lua and travel.lua
// read, compile and bind; data that does not agree with the C++ is refused with the file and the key; a state that is
// not built refuses entry; and the library includes nothing past its boundary.

#include "test_framework.h"

#include "brain_fakes.h"
#include "include_walk.h"

#include "apps/citysim/brain/scripting/brain_lua.h"
#include "apps/citysim/scripting/travel_lua.h"
#include "engine/scripting/script_vm.h"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using namespace brain_test;

namespace {

std::string fileText(const std::string& underSource) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/" + underSource);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

StateSpec* stateNamed(BrainSpec& spec, const std::string& name) {
    for (StateSpec& s : spec.states)
        if (s.name == name) return &s;
    return nullptr;
}

// Compiles `spec` against the real behaviours (or `set`), expecting a refusal whose message holds every one of `says`.
void refused(const BrainSpec& spec, const std::vector<std::string>& says, const BehaviourSet* set = nullptr) {
    BehaviourSet real;
    registerBehaviours(real);
    BrainTable table;
    std::string err;
    const bool ok = compileBrainTable(spec, set ? *set : real, table, &err);
    CHECK(!ok);
    for (const std::string& s : says) {
        if (err.find(s) == std::string::npos) std::printf("    wanted \"%s\" in: %s\n", s.c_str(), err.c_str());
        CHECK(err.find(s) != std::string::npos);
    }
}

}  // namespace

TEST_CASE(brain_record_is_small_plain_data) {
    // What one person costs: 120 bytes, packed on 4, copied and hashed as bytes. (The target was 96; ADR-0440 says
    // where the other 24 went: each leg's planned seconds, and where the plan began.)
    CHECK(sizeof(AgentBrain) == 120u);
    CHECK(alignof(AgentBrain) == 4u);
    CHECK(std::is_trivially_copyable<AgentBrain>::value);
    CHECK(std::is_standard_layout<AgentBrain>::value);
    CHECK(!std::is_polymorphic<AgentBrain>::value);
    CHECK(sizeof(BrainEvent) == 16u);
    CHECK(sizeof(StatePayload) == 4u);
    // a leg's way and its vehicle share four bytes
    AgentBrain a;
    std::memset(&a, 0, sizeof(a));
    a.legUse[3] = packLegUse(5, 123456);
    CHECK(a.legWay(3) == 5 && a.legHandle(3) == 123456u);
    a.legUse[0] = packLegUse(1, kNoHandle);
    CHECK(a.legWay(0) == 1 && a.legHandle(0) == kNoHandle);
    a.legCount = 8;
    a.legNow = 8;
    CHECK(a.legCount == 8 && a.legNow == 8 && !a.travelling());
}

TEST_CASE(brain_shipped_data_reads_compiles_and_binds) {
    BrainSpec spec;
    CHECK(shippedBrainSpec(spec));
    BehaviourSet behaviours;
    registerBehaviours(behaviours);
    BrainTable table;
    std::string err;
    const bool compiled = compileBrainTable(spec, behaviours, table, &err);
    if (!compiled) std::printf("    %s\n", err.c_str());
    CHECK(compiled);
    if (!compiled) return;
    // every state of the design's section 5, and which are built
    // (20 events and 2 intents since ADR-0441: `space_taken`, and `be_outside_at` for a place with no building)
    // (22 states and 22 events since ADR-0443: `no_way`, a person with no way there who tries again; its clock's
    // `try_again`; and `things_changed`, sent when something a person plans with changes)
    CHECK(table.states.size() == 22u && table.events.size() == 22u && table.intents.size() == 2u);
    CHECK(table.intents[0].name == "be_at" && table.intents[1].name == "be_outside_at");
    CHECK(table.states[table.intents[1].arrive].name == "standing");
    CHECK(table.on[table.state("parking")][table.event("space_taken")].verb == Verb::Go);
    CHECK(table.on[table.state("parking")][table.event("space_taken")].state == table.state("driving"));
    std::string built, notBuilt;
    for (const StateRow& s : table.states) (s.fns.built ? built : notBuilt) += s.name + " ";
    CHECK(built == "walking on_small_wheels pulling_out driving parking indoors standing no_way knocked_down stuck crashed "
                   "possessed released ");
    CHECK(notBuilt == "waiting boarding riding alighting hailing using_spot group_activity working_post asleep ");
    CHECK(table.states[table.start].name == "indoors" && table.states[table.noPlan].name == "no_way");
    // ADR-0443: a person with no way there waits a minute (the data's `seconds`) and plans again with every way allowed;
    // they plan again at once when something they plan with changes; everyone else ignores that
    {
        const int noWay = table.state("no_way");
        CHECK(noWay >= 0);
        if (noWay < 0) return;
        CHECK(table.states[noWay].ends == Ends::Clock && table.states[noWay].fixedMillis == 60000u);
        CHECK(table.events[table.states[noWay].clock] == "try_again");
        CHECK(table.changedEvent != kNoId && table.events[table.changedEvent] == "things_changed");
        CHECK(table.on[noWay][table.event("try_again")].verb == Verb::ReplanAfresh);
        CHECK(table.on[noWay][table.changedEvent].verb == Verb::ReplanAfresh);
        CHECK(table.on[noWay][table.intentEvent].verb == Verb::Replan);
        CHECK(table.on[table.state("walking")][table.changedEvent].verb == Verb::Ignore);
        CHECK(table.on[table.state("standing")][table.changedEvent].verb == Verb::Ignore);
        CHECK(table.on[table.state("possessed")][table.changedEvent].verb == Verb::Ignore);
    }
    CHECK(table.events[table.intentEvent] == "new_intent");
    // a few answers, read back: the row's own, an event's `otherwise`, and a holding state answering only its row
    const int walking = table.state("walking"), driving = table.state("driving"), possessed = table.state("possessed");
    const int indoors = table.state("indoors");
    CHECK(walking >= 0 && driving >= 0 && possessed >= 0 && indoors >= 0);
    if (walking < 0 || driving < 0 || possessed < 0 || indoors < 0) return;
    CHECK(table.on[walking][table.event("arrived")].verb == Verb::NextLeg);
    CHECK(table.on[driving][table.event("parked")].verb == Verb::Go);
    CHECK(table.on[driving][table.event("parked")].state == table.state("parking"));
    CHECK(table.on[driving][table.event("no_route")].verb == Verb::ReplanWithout);    // the rule for a failed leg
    CHECK(table.on[driving][table.event("cannot_park")].verb == Verb::ReplanWithout);
    CHECK(table.on[walking][table.event("blocked")].verb == Verb::ReplanWithout);
    CHECK(table.on[walking][table.event("vehicle_gone")].verb == Verb::Replan);
    CHECK(table.on[walking][table.event("possessed")].state == possessed);            // anyone can be taken over
    CHECK(table.on[possessed][table.event("no_route")].verb == Verb::Ignore);         // ...and then nothing moves them
    CHECK(table.on[possessed][table.event("arrived")].verb == Verb::Ignore);
    CHECK(table.on[indoors][table.event("arrived")].verb == Verb::Refuse);
    CHECK(table.states[walking].ends == Ends::Skill && table.states[walking].skill == "walk");
    CHECK(table.states[table.state("parking")].ends == Ends::Clock);
    CHECK(table.states[indoors].ends == Ends::Never && table.states[indoors].skill == "none");

    // THE SHIPPED travel.lua: walk and car as before, and the two small wheels (estimates) for people who have them
    engine::ScriptVM vm;
    CHECK(vm.doString(fileText("assets/scripts/travel.lua"), &err));
    TravelCatalog travel;
    const bool read = engine::loadTravelCatalog(vm, travel, &err);
    if (!read) std::printf("    %s\n", err.c_str());
    CHECK(read);
    CHECK(travel.modes.size() == 4u && travel.find("skateboard") == 2 && travel.find("scooter") == 3);
    if (travel.modes.size() != 4u) return;
    CHECK(travel.traits.size() == 2u && travel.trait("has_board") == 0 && travel.trait("has_scooter") == 1);
    CHECK(travel.modes[2].needsTrait == 0 && travel.modes[3].needsTrait == 1 && travel.modes[0].needsTrait == -1);
    CHECK(!travel.modes[2].roads && !travel.modes[2].ownVehicle && !travel.modes[3].roads && !travel.modes[3].ownVehicle);
    CHECK(travel.modes[2].speed > travel.modes[0].speed && travel.modes[3].speed > travel.modes[0].speed);
    CHECK(travel.modes[2].longestLegMinutes == 15.0 && travel.modes[3].longestLegMinutes == 20.0);
    CHECK(travel.modes[0].longestLegMinutes == 0.0);
    CHECK(travel.modes[1].access == -1);   // the car is reached by the cheapest way on foot, not a fixed one
    const bool bound = bindWays(travel, table, &err);
    if (!bound) std::printf("    %s\n", err.c_str());
    CHECK(bound);
    CHECK(table.ways.size() == 4u);
    if (table.ways.size() != 4u) return;
    CHECK(table.states[table.ways[0].begins].name == "walking");
    CHECK(table.states[table.ways[1].begins].name == "pulling_out");
    CHECK(table.states[table.ways[2].begins].name == "on_small_wheels");
    CHECK(table.states[table.ways[3].begins].name == "on_small_wheels");
    CHECK(table.ways[1].startMillis == 120000u && table.ways[1].endMillis == 180000u);   // 2 and 3 minutes
    // ADR-0442: a car waits at most a minute and a half for a clear lane to leave its space into (a way on foot waits
    // for nothing); a new intent at the wheel redirects the drive, and is left alone while pulling out and parking;
    // `blocked` at the wheel is the failed-leg rule; every state has a display name
    CHECK(table.ways[1].setOffWaitMillis == 90000u && table.ways[0].setOffWaitMillis == 0u);
    CHECK(table.on[driving][table.intentEvent].verb == Verb::Redirect);
    CHECK(table.on[table.state("pulling_out")][table.intentEvent].verb == Verb::Ignore);
    CHECK(table.on[table.state("parking")][table.intentEvent].verb == Verb::Ignore);
    CHECK(table.on[walking][table.intentEvent].verb == Verb::Replan);
    CHECK(table.on[driving][table.event("blocked")].verb == Verb::ReplanWithout);
    CHECK(table.states[driving].fns.retarget != nullptr && table.states[walking].fns.retarget == nullptr);
    CHECK(table.states[table.state("pulling_out")].label == "getting into the car");
    CHECK(table.states[table.state("standing")].label == "standing outside");
    for (const StateRow& s : table.states) CHECK(!s.label.empty());
}

TEST_CASE(brain_data_that_disagrees_with_the_code_is_refused_by_file_and_key) {
    BrainSpec shipped;
    CHECK(shippedBrainSpec(shipped));
    if (shipped.states.empty()) return;

    // a state in the data with no behaviour in C++
    {
        BrainSpec s = shipped;
        StateSpec flying;
        flying.name = "flying";
        s.states.push_back(flying);
        refused(s, {"brain.lua: states.flying", "no behaviour registered"});
    }
    // a behaviour in C++ with no state in the data
    {
        BehaviourSet set;
        registerBehaviours(set);
        set.add("teleporting", StateFns{});
        refused(shipped, {"brain.lua: states", "no row `teleporting`", "registered a behaviour"}, &set);
    }
    // a row's event leading to a state that does not exist
    {
        BrainSpec s = shipped;
        for (std::pair<std::string, std::string>& row : stateNamed(s, "walking")->on)
            if (row.first == "arrived") row.second = "flying";
        refused(s, {"brain.lua: states.walking.on.arrived", "names `flying`", "neither a state nor a verb"});
    }
    // ...and an event's `otherwise` doing the same
    {
        BrainSpec s = shipped;
        for (EventSpec& e : s.events)
            if (e.name == "possessed") e.otherwise = "abducted";
        refused(s, {"brain.lua: events.possessed.otherwise", "names `abducted`"});
    }
    // a row answering an event the data does not list
    {
        BrainSpec s = shipped;
        stateNamed(s, "walking")->on.emplace_back("rained_on", "indoors");
        refused(s, {"brain.lua: states.walking.on.rained_on", "is not an event"});
    }
    // an event a behaviour raises itself, gone from the data
    {
        BrainSpec s = shipped;
        for (std::size_t i = 0; i < s.events.size(); ++i)
            if (s.events[i].name == "vehicle_gone") { s.events.erase(s.events.begin() + static_cast<long>(i)); break; }
        refused(s, {"brain.lua: events", "no `vehicle_gone`", "behaviour of `pulling_out` raises"});
    }
    // a skill C++ has no interface for; a clock event that is not one; resume where nothing is held
    {
        BrainSpec s = shipped;
        stateNamed(s, "walking")->skill = "teleport";
        refused(s, {"brain.lua: states.walking.skill", "names `teleport`"});
        s = shipped;
        stateNamed(s, "parking")->clock = "ding";
        refused(s, {"brain.lua: states.parking.clock", "names `ding`"});
        s = shipped;
        stateNamed(s, "walking")->on.emplace_back("got_up", "resume");
        refused(s, {"brain.lua: states.walking.on.got_up", "resume", "does not hold"});
    }
    // redirect in a state whose behaviour cannot be sent elsewhere, and as an event's `otherwise`
    {
        BrainSpec s = shipped;
        stateNamed(s, "parking")->on.emplace_back("new_intent", "redirect");
        refused(s, {"brain.lua: states.parking.on.new_intent", "redirect", "cannot be sent elsewhere"});
        s = shipped;
        for (EventSpec& e : s.events)
            if (e.name == "new_intent") e.otherwise = "redirect";
        refused(s, {"brain.lua: events.new_intent.otherwise", "is redirect"});
    }
    // the named roles
    {
        BrainSpec s = shipped;
        s.start = "limbo";
        refused(s, {"brain.lua: start", "names `limbo`"});
        s = shipped;
        s.intents[0].arrive = "nowhere";
        refused(s, {"brain.lua: intents.be_at.arrive", "names `nowhere`"});
        s = shipped;
        s.intentEvent = "";
        refused(s, {"brain.lua: intent_event", "is missing"});
    }
    // a state listed twice, and one named like a verb
    {
        BrainSpec s = shipped;
        s.states.push_back(*stateNamed(s, "indoors"));
        refused(s, {"brain.lua: states.indoors", "listed twice"});
        s = shipped;
        StateSpec verb;
        verb.name = "replan";
        s.states.push_back(verb);
        refused(s, {"brain.lua: states.replan", "named like a verb"});
    }
    // THE FILE ITSELF: a key a row does not have, and a value of the wrong kind, each named by the reader
    {
        engine::ScriptVM vm;
        std::string err;
        CHECK(vm.doString(fileText("assets/scripts/brain.lua") +
                              "\nbrain.states[1].colour = 'red'\nbrain.states[2].on.arrived = 7\n", &err));
        BrainSpec s;
        CHECK(!loadBrainSpec(vm, s, &err));
        CHECK(err.find("brain.lua: states[1] has the key \"colour\"") != std::string::npos);
        CHECK(err.find("brain.lua: states.on_small_wheels.on.arrived is 7") != std::string::npos);
    }
    // A WAY OF TRAVELLING that begins in no state of brain.lua
    {
        BehaviourSet set;
        registerBehaviours(set);
        BrainTable table;
        std::string err;
        CHECK(compileBrainTable(shipped, set, table, &err));
        TravelCatalog travel = testTravel();
        travel.modes[1].beginsIn = "flying";
        CHECK(!bindWays(travel, table, &err));
        CHECK(err == "travel.lua: modes `car`: begins_in names `flying`, which is not a state of brain.lua");
        travel.modes[1].beginsIn = "";
        CHECK(!bindWays(travel, table, &err));
        CHECK(err == "travel.lua: modes `car`: begins_in is missing");
    }
}

TEST_CASE(brain_a_plan_over_the_cap_is_refused_and_the_next_cheapest_taken) {
    const engine::NavGraph nav = longStreet();
    const TravelCatalog travel = testTravel();
    // the fixed form holds 8 legs: a 9-leg way is refused, by the file and the row
    TripRequest ask;
    ask.from = Vec2(0, 0);
    ask.to = Vec2(1500, 0);
    ask.vehicles.push_back(OwnedVehicle{kCar, Vec2(100, 0), Vec2(1300, 0)});
    const TripPlan plan = planTrip(nav, travel, ask);
    CHECK(plan.valid() && plan.choice()->mode == kCar && plan.choice()->legs.size() == 3u);
    if (!plan.valid()) return;
    FixedLegs fixed;
    std::string err;
    CHECK(fixedLegs(travel, *plan.choice(), fixed, &err));
    CHECK(fixed.count == 3 && fixed.legs[0].way == kWalk && fixed.legs[1].way == kCar && fixed.legs[2].way == kWalk);
    CHECK(fixed.legs[0].end == LegEnd::Vehicle && fixed.legs[1].end == LegEnd::VehicleLeft &&
          fixed.legs[2].end == LegEnd::Door);
    CHECK(fixed.legs[0].seconds == 72 && fixed.legs[1].seconds == 300 && fixed.legs[2].seconds == 144);
    TripOption nine = *plan.choice();
    while (nine.legs.size() < 9u) nine.legs.push_back(nine.legs.back());
    CHECK(!fixedLegs(travel, nine, fixed, &err));
    CHECK(err == "travel.lua: modes `car`: a plan of 9 legs; a plan holds at most 8 (kMaxPlanLegs)");
    // ...and in the planner: a way needing more legs than the plan holds is not possible, and the next cheapest wins
    ask.maxLegs = 2;
    const TripPlan capped = planTrip(nav, travel, ask);
    CHECK(capped.valid() && capped.choice()->mode == kWalk);
    CHECK(!capped.options[kCar].possible);
    CHECK(capped.options[kCar].why == "3 legs, and a plan holds at most 2");
    // asking for more than the fixed form holds changes nothing: the cap is 8
    ask.maxLegs = 99;
    CHECK(planTrip(nav, travel, ask).choice()->mode == kCar);
}

TEST_CASE(brain_a_state_that_is_not_built_refuses_entry_and_says_so) {
    BrainSpec spec;
    CHECK(shippedBrainSpec(spec));
    if (spec.states.empty()) return;
    // a row sending someone to a state that is not built: they stay where they are, and the machine says why
    stateNamed(spec, "indoors")->on.emplace_back("knocked_down", "asleep");
    {
        Lab lab(true, &spec);
        CHECK(lab.ok);
        if (!lab.ok) return;
        const AgentId who = lab.person(lab.door(0), 0, CarRef(), Detail::Near);
        CHECK(lab.machine->post(who, "knocked_down"));
        lab.tick();
        CHECK(lab.stateName(who) == "indoors");
        CHECK(lab.machine->counts().errors == 1u);
        CHECK(std::string(lab.machine->lastError()) == "brain: agent 0: `asleep` is not built yet");
        CHECK(lab.trace.empty());
    }
    // a way of travelling that begins in a state that is not built cannot be travelled: it is ruled out, and the
    // person goes another way
    {
        TravelCatalog travel = testTravel();
        travel.modes[kSkateboard].beginsIn = "riding";
        Lab lab(true, nullptr, &travel);
        CHECK(lab.ok);
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, kHasBoard, CarRef(), Detail::Near);
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        CHECK(lab.planOf(who) == "walk>1");
        CHECK(lab.stateName(who) == "walking");
        CHECK(lab.machine->agent(who).ruledOut == (1u << kSkateboard));
        CHECK(lab.machine->counts().errors == 1u && lab.machine->counts().plans == 2u);
        CHECK(std::string(lab.machine->lastError()) == "brain: agent 0: `riding` is not built yet, so its way is ruled out");
    }
}

// THE BOUNDARY. The library -- every file directly in src/apps/citysim/brain -- may include, outside itself and the
// driving library it is built on, only the headers on this list, and those are followed. The city sim (city_sim.h),
// the renderer, the entity world and the physics wrapper are not on it.
TEST_CASE(brain_includes_nothing_past_its_boundary) {
    const std::vector<std::string> allowed = {
        "apps/citysim/trip_plan.h",                 // THE trip planner, which the brain plans with
        "engine/ai/nav_graph.h",                    // ... the street map it plans on
        "engine/ai/pathfind.h",                     // ... and its router
        "engine/procgen/city/road_network.h", "engine/procgen/city/road_spec.h", "engine/procgen/city/polygon.h",
        "engine/procgen/city/roads/road_model.h", "engine/procgen/city/roads/road_model_parts.h",
        "engine/procgen/city/roads/lane_graph.h", "engine/procgen/city/roads/graph_index.h",
        "engine/procgen/city/roads/vehicle_size.h",
        "engine/bundle/bundle.h", "engine/bundle/binary_stream.h",   // the repo's hash
        "rt_math.h", "hash.h", "handle.h", "slot_map.h", "proc_rng.h",
    };
    const std::filesystem::path src = std::filesystem::weakly_canonical(std::filesystem::path(RT_SOURCE_DIR) / "src");
    const std::filesystem::path dir = src / "apps" / "citysim" / "brain";
    auto walkOf = [&](const std::filesystem::path& folder, const std::vector<std::string>& more,
                      const std::vector<std::string>& external) {
        IncludeWalk w;
        w.src = src;
        w.library = "apps/citysim/brain/";
        w.builtOn = {"apps/driving/"};
        w.allowed = allowed;
        w.allowed.insert(w.allowed.end(), more.begin(), more.end());
        w.external = external;
        std::vector<std::filesystem::path> roots;
        for (const std::filesystem::directory_entry& e : std::filesystem::directory_iterator(folder))
            if (e.is_regular_file()) roots.push_back(e.path());
        std::sort(roots.begin(), roots.end());
        for (const std::filesystem::path& f : roots) w.walk(f);
        return w;
    };
    const IncludeWalk lib = walkOf(dir, {}, {});
    for (const std::string& b : lib.breaches) std::printf("    the library: %s\n", b.c_str());
    CHECK(lib.breaches.empty());
    CHECK(lib.seen.size() > 20u);   // the library's files and what they include, followed
    for (const std::string& f : lib.seen) CHECK(f.find("city_sim") == std::string::npos);
    // its scripting folder, the one place that reads Lua: the script VM's headers besides
    const IncludeWalk scripting =
        walkOf(dir / "scripting", {"engine/scripting/lua_state.h", "engine/scripting/script_vm.h"},
               {"lua.h", "lualib.h", "lauxlib.h"});
    for (const std::string& b : scripting.breaches) std::printf("    its scripting folder: %s\n", b.c_str());
    CHECK(scripting.breaches.empty());
    // the walk does see a breach: the city sim is not on the list
    IncludeWalk probe;
    probe.src = src;
    probe.library = "apps/citysim/brain/";
    probe.allowed = allowed;
    probe.walk(src / "apps" / "citysim" / "city_trip.h");
    CHECK(!probe.breaches.empty());
}
