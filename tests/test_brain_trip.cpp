// ONE PERSON'S TRIPS THROUGH THE AGENT BRAIN (ADR-0440), with stand-in skills (brain_fakes.h) and the real trip planner,
// on a straight 2 km street with a node every 100 m. Every expected time is worked out by hand in milliseconds:
//   walking 100 m takes 72 s (5 km/h); a skateboard 36 s (10 km/h);
//   driving 100 m takes 12.5 s (a local street, 8 m/s); the car takes 60 s to pull out and 90 s to park.
// The clock steps half a second at a time, and every time below falls on a step.

#include "test_framework.h"

#include "brain_fakes.h"

#include <string>
#include <vector>

using namespace brain_test;
using engine::approxEqual;

namespace {

// How close two fractions of a leg are to count as the same (they are worked out from whole milliseconds).
constexpr Real kSameFraction = 1e-9;

// THE USUAL SCENE: home at 0 m, the household's car in a kerb space at 100 m, a free space at 1300 m, the door wanted
// at 1500 m. Places are numbered as they are made: home 0, door 1, the car's space 2, the far space 3.
struct Scene {
    Lab lab;
    PlaceRef home, door, carSpace, farSpace;
    CarRef car;
    AgentId who = kNoAgent;

    explicit Scene(Detail detail, bool skills = true, uint8_t traits = 0) : lab(skills) {
        if (!lab.ok) return;
        home = lab.door(0);
        door = lab.door(1500);
        carSpace = lab.space(100);
        farSpace = lab.space(1300);
        car = lab.world.addCar(carSpace);
        who = lab.person(home, traits, car, detail);
    }
    void setOut() {
        lab.machine->setIntent(who, 0, door);
        lab.tick();
    }
    // An event that is answered at the tick at `at` (posted just before it).
    void postFor(BrainTime at, const char* event) {
        lab.runTo(at - kTick);
        CHECK(lab.machine->post(who, event));
        lab.runTo(at);
    }
};

const char* const kWholeTrip = "walking@0 pulling_out@72000 driving@132000 parking@282000 walking@372000 indoors@516000";

}  // namespace

TEST_CASE(brain_trip_walk_drive_walk_states_and_times) {
    Scene s(Detail::Near);
    CHECK(s.lab.ok);
    if (!s.lab.ok) return;
    CHECK(s.lab.stateName(s.who) == "indoors");
    s.setOut();
    // THE PLAN: walk to the car's space, drive to the far space, walk to the door; 72 s, 300 s (60 + 150 + 90), 144 s
    const AgentBrain& a = s.lab.machine->agent(s.who);
    CHECK(s.lab.planOf(s.who) == "walk>2 car>3 walk>1");
    CHECK(a.legCount == 3 && a.legNow == 0);
    CHECK(a.legSeconds[0] == 72 && a.legSeconds[1] == 300 && a.legSeconds[2] == 144);
    CHECK(a.legHandle(0) == kNoHandle && a.legHandle(1) == s.car.id && a.legHandle(2) == kNoHandle);
    CHECK(s.lab.planner->lastReason() ==
          "walk 18.0 min, car 8.6 min (1.2 walk + 5.0 car + 2.4 walk): car");   // (no board: the board rows are not his)
    CHECK(a.intentTarget == s.door.id && a.intentBegan == 0u && a.intentKind == 0);

    // ONE LINE, part-way through the drive
    s.lab.runTo(200000);
    CHECK(s.lab.machine->describe(s.who) ==
          "agent 0: wants be_at place 1 since 0.0 s | plan: walk to 2, car to 3 (vehicle 0), walk to 1 | leg 2 of 3 | "
          "driving since 132.0 s, due 282.0 s, waiting for parked from the drive skill | near");
    CHECK(s.lab.world.cars[0].heldBy == s.who);

    s.lab.runTo(600000);
    // THE STATES, each with the time it was entered
    CHECK(s.lab.traceOf(s.who) == kWholeTrip);
    std::printf("    %s\n", s.lab.traceOf(s.who).c_str());
    CHECK(s.lab.stateName(s.who) == "indoors" && a.stateEntered == 516000u && a.stateDue == kNever);
    CHECK(a.legNow == 3 && !a.travelling());
    CHECK(s.lab.machine->counts().plans == 1u && s.lab.machine->counts().arrivals == 1u);
    CHECK(s.lab.machine->counts().refused == 0u && s.lab.machine->counts().errors == 0u);
    // the skills were asked once each: two walks, one pull-out, one drive, one parking
    CHECK(s.lab.walk->started == 2 && s.lab.walk->cancelled == 0);
    CHECK(s.lab.drive->pulledOut == 1 && s.lab.drive->begun == 1 && s.lab.drive->parked == 1 && s.lab.drive->ended == 0);
    // the car is where its driver left it, free, for whoever comes next
    CHECK(s.lab.world.cars[0].standing == s.farSpace && s.lab.world.cars[0].heldBy == kNoAgent);
    CHECK(s.lab.world.spaceCar[s.farSpace.id] == s.car.id && s.lab.world.spaceCar[s.carSpace.id] == CarRef::kNone);
    CHECK(s.lab.machine->describe(s.who) ==
          "agent 0: wants be_at place 1 since 0.0 s | plan: walk to 2, car to 3 (vehicle 0), walk to 1 | plan done | "
          "indoors since 516.0 s, waiting for an event | near");
}

TEST_CASE(brain_trip_a_board_is_one_leg_near_and_not_used_beyond_its_longest_leg) {
    // NEAR: 500 m. On the board 180 s (minded as 3.6 min) against 6 min on foot: one skateboard leg.
    {
        Lab lab;
        CHECK(lab.ok);
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, kHasBoard, CarRef(), Detail::Near);
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        CHECK(lab.planOf(who) == "skateboard>1");
        CHECK(lab.machine->agent(who).legSeconds[0] == 180);
        CHECK(lab.planner->lastReason() == "walk 6.0 min, car -- no car, skateboard 3.0 min: skateboard");
        lab.runTo(300000);
        CHECK(lab.traceOf(who) == "on_small_wheels@0 indoors@180000");
        CHECK(lab.walk->walks[who].way == kSkateboard);   // the walk skill was told which way, so it moved at its speed
    }
    // THE SAME TRIP FOR SOMEONE WITH NO BOARD: the row is not theirs, and the reason does not mention it
    {
        Lab lab;
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        CHECK(lab.planOf(who) == "walk>1");
        CHECK(lab.planner->lastReason() == "walk 6.0 min, car -- no car: walk");
    }
    // FAR: 1500 m is 9 minutes on the board, and a board suits a leg of 6 at most: on foot, 1080 s
    {
        Lab lab;
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), office = lab.door(1500);
        const AgentId who = lab.person(home, kHasBoard, CarRef(), Detail::Near);
        lab.machine->setIntent(who, 0, office);
        lab.tick();
        CHECK(lab.planOf(who) == "walk>1");
        CHECK(lab.machine->agent(who).legSeconds[0] == 1080);
        CHECK(lab.planner->lastReason() ==
              "walk 18.0 min, car -- no car, skateboard -- too far for skateboard (9.0 min, at most 6.0): walk");
    }
    // FAR, WITH THE CAR: the board is how the car is reached and left (36 s and 72 s where walking is 72 and 144),
    // with no rule saying so: each end of the drive is made by the cheapest way on foot the person has
    {
        Scene s(Detail::Near, true, kHasBoard);
        if (!s.lab.ok) return;
        s.setOut();
        CHECK(s.lab.planOf(s.who) == "skateboard>2 car>3 skateboard>1");
        s.lab.runTo(500000);
        CHECK(s.lab.traceOf(s.who) ==
              "on_small_wheels@0 pulling_out@36000 driving@96000 parking@246000 on_small_wheels@336000 indoors@408000");
    }
}

TEST_CASE(brain_trip_the_household_car_taken_by_another_is_gone_at_the_kerb) {
    Scene s(Detail::Near);
    if (!s.lab.ok) return;
    // someone else of the household, who lives beside the car and wants the same door
    const PlaceRef theirHome = s.lab.door(100);
    const AgentId other = s.lab.person(theirHome, 0, s.car, Detail::Near);
    s.setOut();
    CHECK(s.lab.planOf(s.who) == "walk>2 car>3 walk>1");   // planned while the car was free
    s.lab.runTo(20000);
    s.lab.machine->setIntent(other, 0, s.door);   // 20 s later the other sets out: at the car in no time, and takes it
    s.lab.runTo(1200000);
    CHECK(s.lab.traceOf(other) == "walking@20500 pulling_out@21000 driving@81000 parking@231000 walking@321000 indoors@465000");
    // THE FIRST PERSON reaches the kerb at 72 s: the car is there and someone has it. Planned again, with the car not
    // priced because it is not free; on foot from the kerb, 1400 m, 1008 s.
    CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 walking@72000 indoors@1080000");
    const AgentBrain& a = s.lab.machine->agent(s.who);
    CHECK(s.lab.planOf(s.who) == "walk>1");
    CHECK(a.legSeconds[0] == 1008);
    CHECK(a.ruledOut == 0);   // nothing failed: the car was simply not there to price
    CHECK(s.lab.machine->counts().plans == 3u && s.lab.machine->counts().arrivals == 2u);
    CHECK(s.lab.machine->counts().refused == 0u && s.lab.machine->counts().errors == 0u);
    // the car stayed where its driver parked it
    CHECK(s.lab.world.cars[0].standing == s.farSpace && s.lab.world.cars[0].heldBy == kNoAgent);
}

TEST_CASE(brain_trip_no_route_and_cannot_park_plan_again_without_the_car) {
    // NO ROUTE, reported part-way through the drive, at 194.5 s: the car is 500 m on, at 600 m. The person gets out
    // there, the car is ruled out, and they walk the 900 m left: 648 s.
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.setOut();
        s.postFor(194500, "no_route");
        const AgentBrain& a = s.lab.machine->agent(s.who);
        CHECK(a.ruledOut == (1u << kCar));
        CHECK(s.lab.planOf(s.who) == "walk>1" && a.legSeconds[0] == 648);
        CHECK(s.lab.planner->lastReason() == "walk 10.8 min, car -- ruled out (a leg of it could not be done): walk");
        CHECK(s.lab.drive->ended == 1);
        // the car is left where it stopped, free, at a place of its own (the fifth place made)
        CHECK(s.lab.world.cars[0].heldBy == kNoAgent && s.lab.world.cars[0].standing.id == 4u);
        CHECK(approxEqual(s.lab.world.places[4], Vec2(600, 0), engine::kSamePoint));
        s.lab.runTo(900000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 walking@194500 indoors@842500");
        CHECK(s.lab.machine->counts().plans == 2u && s.lab.machine->counts().arrivals == 1u);
    }
    // NO ROUTE AT THE START of the drive (the skill refuses): the same rule, from the car's space; 1400 m, 1008 s
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.lab.drive->noRoute = true;
        s.setOut();
        s.lab.runTo(1200000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 walking@132000 indoors@1140000");
        CHECK(s.lab.machine->agent(s.who).ruledOut == (1u << kCar));
        CHECK(s.lab.world.cars[0].heldBy == kNoAgent);
    }
    // CANNOT PARK: another car takes the far space during the drive. Beside it at 282 s the space is refused; the
    // person leaves the car there and walks the last 200 m: 144 s.
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.runTo(200000);
        s.lab.world.addCar(s.farSpace);
        s.lab.runTo(500000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 parking@282000 walking@282000 indoors@426000");
        CHECK(s.lab.machine->agent(s.who).ruledOut == (1u << kCar));
        CHECK(s.lab.world.spaceCar[s.farSpace.id] == 1u);   // the other car's
        CHECK(s.lab.world.cars[0].heldBy == kNoAgent && s.lab.world.cars[0].standing.id == 4u);
        CHECK(approxEqual(s.lab.world.places[4], Vec2(1300, 0), engine::kSamePoint));
    }
    // CANNOT PARK reported by the driving itself, part-way: the table's rule for a failed leg, with no row of its own
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.setOut();
        s.postFor(194500, "cannot_park");
        CHECK(s.lab.stateName(s.who) == "walking" && s.lab.machine->agent(s.who).ruledOut == (1u << kCar));
        s.lab.runTo(900000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 walking@194500 indoors@842500");
    }
}

// A TAKEN SPACE, AND A WORLD THAT NAMES THE NEXT (ADR-0441). Another car takes the far space (1300 m) during the drive,
// and there is a free one at 1200 m, the next nearest to the door. Beside the taken space at 282 s the person is told
// the one at 1200 m: the leg now ends there, and they drive to it, 100 m in 12.5 s, park (90 s), and walk the 300 m to
// the door (216 s). The car is not ruled out and nothing is planned again.
TEST_CASE(brain_trip_a_taken_space_is_answered_with_the_next_when_the_world_names_one) {
    Scene s(Detail::Near);
    if (!s.lab.ok) return;
    const PlaceRef nextSpace = s.lab.space(1200);
    s.lab.world.offersNextSpace = true;
    s.setOut();
    CHECK(s.lab.planOf(s.who) == "walk>2 car>3 walk>1");
    s.lab.runTo(200000);
    s.lab.world.addCar(s.farSpace);
    s.lab.runTo(700000);
    CHECK(s.lab.traceOf(s.who) ==
          "walking@0 pulling_out@72000 driving@132000 parking@282000 driving@282000 parking@294500 walking@384500 indoors@600500");
    const AgentBrain& a = s.lab.machine->agent(s.who);
    CHECK(a.ruledOut == 0 && s.lab.machine->counts().plans == 1u);
    CHECK(s.lab.planOf(s.who) == "walk>2 car>" + std::to_string(nextSpace.id) + " walk>1");
    CHECK(s.lab.world.spaceCar[nextSpace.id] == s.car.id);       // their car stands in the next space
    CHECK(s.lab.world.spaceCar[s.farSpace.id] == 1u);             // ...and the other car's in the one it took
    CHECK(s.lab.world.cars[0].heldBy == kNoAgent && s.lab.world.cars[0].standing == nextSpace);
}

TEST_CASE(brain_trip_possessed_suspends_the_intent_and_release_resumes_it) {
    Lab lab;
    CHECK(lab.ok);
    if (!lab.ok) return;
    const PlaceRef home = lab.door(0), shop = lab.door(500);   // 360 s on foot
    const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
    lab.machine->setIntent(who, 0, shop);
    lab.tick();
    // TAKEN OVER at 72 s, 100 m along
    lab.runTo(71500);
    CHECK(lab.machine->post(who, "possessed"));
    lab.runTo(72000);
    const AgentBrain& a = lab.machine->agent(who);
    CHECK(lab.stateName(who) == "possessed");
    CHECK(a.heldState == lab.state("walking") && a.payload.remaining == 288000u && a.stateDue == kNever);
    CHECK(lab.walk->cancelled == 1 && !lab.walk->walks[who].active);
    CHECK(lab.machine->describe(who) ==
          "agent 0: wants be_at place 1 since 0.0 s | plan: walk to 1 | leg 1 of 1 | possessed since 72.0 s, waiting "
          "for an event, holding walking with 288.0 s left | near");
    // NOTHING ADVANCES while held: the record is the same, byte for byte, two minutes on; the walk that would have
    // ended at 360 s does not
    const uint64_t held = lab.machine->stateHash();
    const uint64_t transitions = lab.machine->counts().transitions;
    // ...and reports that would move anyone else on (a leg failed, a walk ended, a new intent's event) do nothing
    lab.runTo(100000);
    CHECK(lab.machine->post(who, "blocked") && lab.machine->post(who, "arrived") && lab.machine->post(who, "no_route"));
    lab.runTo(400000);
    CHECK(lab.stateName(who) == "possessed" && lab.machine->counts().plans == 1u);
    CHECK(lab.machine->stateHash() == held);
    CHECK(lab.machine->counts().transitions == transitions && lab.machine->counts().arrivals == 0u);
    CHECK(a.intentTarget == shop.id && a.intentBegan == 0u);   // the intent is kept
    CHECK(lab.machine->nextAttention(who) == kNever);
    // LET GO at 500 s: the intent takes over, planned from where they stand (100 m along): 400 m, 288 s
    lab.runTo(499500);
    CHECK(lab.machine->post(who, "released"));
    lab.runTo(500000);
    CHECK(lab.stateName(who) == "walking" && a.stateEntered == 500000u && a.stateDue == 788000u);
    CHECK(a.originX == 10000 && a.originZ == 0);   // planned from 100.00 m, in the record's whole centimetres
    CHECK(a.legSeconds[0] == 288);
    lab.runTo(900000);
    CHECK(lab.traceOf(who) == "walking@0 possessed@72000 released@500000 walking@500000 indoors@788000");
    CHECK(lab.machine->counts().plans == 2u && lab.machine->counts().arrivals == 1u);
    CHECK(lab.machine->counts().refused == 0u && lab.machine->counts().errors == 0u);
}

TEST_CASE(brain_trip_knocked_down_then_got_up_carries_on_the_same_leg) {
    Lab lab;
    CHECK(lab.ok);
    if (!lab.ok) return;
    const PlaceRef home = lab.door(0), shop = lab.door(500);
    const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
    lab.machine->setIntent(who, 0, shop);
    lab.tick();
    lab.runTo(71500);
    CHECK(lab.machine->post(who, "knocked_down"));
    lab.runTo(72000);
    const AgentBrain& a = lab.machine->agent(who);
    CHECK(lab.stateName(who) == "knocked_down" && a.payload.remaining == 288000u);
    lab.runTo(99500);
    CHECK(lab.machine->post(who, "got_up"));
    lab.runTo(100000);
    // back in the same state, on the same leg of the same plan, with the 28 s on the ground added to its end
    CHECK(lab.stateName(who) == "walking" && a.legNow == 0 && a.legCount == 1);
    CHECK(a.stateEntered == 28000u && a.stateDue == 388000u && a.heldState == kNoId);
    CHECK(approxEqual(lab.walk->walks[who].fraction, 0.2, kSameFraction));   // the walk skill told how far they had got
    CHECK(lab.machine->counts().plans == 1u);
    lab.runTo(500000);
    CHECK(lab.traceOf(who) == "walking@0 knocked_down@72000 walking@100000 indoors@388000");
    CHECK(lab.machine->counts().plans == 1u && lab.machine->counts().arrivals == 1u);
    CHECK(lab.walk->started == 2 && lab.walk->cancelled == 1);
}

TEST_CASE(brain_trip_stuck_carries_on_and_a_crash_goes_on_without_the_car) {
    // STUCK part-way through a walk, then freed: the same as knocked down, by the other pair of events
    {
        Lab lab;
        CHECK(lab.ok);
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        lab.runTo(71500);
        CHECK(lab.machine->post(who, "stuck"));
        lab.runTo(99500);
        CHECK(lab.stateName(who) == "stuck");
        CHECK(lab.machine->post(who, "got_up"));   // not this state's way out: nothing happens
        CHECK(lab.machine->post(who, "freed"));
        lab.runTo(500000);
        CHECK(lab.traceOf(who) == "walking@0 stuck@72000 walking@100000 indoors@388000");
        CHECK(lab.machine->counts().plans == 1u && lab.machine->counts().refused == 0u);
    }
    // CRASHED part-way through the drive, at 194.5 s and 600 m: out of the car, which stays where it stopped. Up again
    // at 250 s, the trip goes on without the car: 900 m on foot, 648 s.
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.setOut();
        s.postFor(194500, "crashed");
        const AgentBrain& a = s.lab.machine->agent(s.who);
        CHECK(s.lab.stateName(s.who) == "crashed" && a.heldState == s.lab.state("driving") && a.stateDue == kNever);
        CHECK(s.lab.drive->ended == 1 && s.lab.world.cars[0].heldBy == kNoAgent);
        s.postFor(250000, "got_up");
        CHECK(s.lab.stateName(s.who) == "walking" && a.ruledOut == (1u << kCar) && a.legSeconds[0] == 648);
        s.lab.runTo(1000000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 crashed@194500 walking@250000 indoors@898000");
        CHECK(s.lab.machine->counts().plans == 2u && s.lab.machine->counts().arrivals == 1u);
    }
}

TEST_CASE(brain_trip_far_by_the_clock_alone_keeps_the_near_times) {
    // FAR, WITH NO SKILL AT ALL: the same trip, the same states at the same times
    {
        Scene s(Detail::Far, false);
        CHECK(s.lab.ok);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.runTo(600000);
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
        CHECK(s.lab.walk->started == 0 && s.lab.drive->begun == 0 && s.lab.drive->pulledOut == 0);
        CHECK(s.lab.world.cars[0].standing == s.farSpace && s.lab.world.cars[0].heldBy == kNoAgent);
        CHECK(s.lab.machine->describe(s.who) ==
              "agent 0: wants be_at place 1 since 0.0 s | plan: walk to 2, car to 3 (vehicle 0), walk to 1 | plan done | "
              "indoors since 516.0 s, waiting for an event | far");
    }
    // VISITED ONLY WHEN IT SAYS IT NEEDS IT (a host's clock queue): five visits, at the five times
    {
        Scene s(Detail::Far, false);
        if (!s.lab.ok) return;
        CHECK(s.lab.machine->nextAttention(s.who) == kNever);
        s.setOut();
        std::vector<BrainTime> visits;
        for (int i = 0; i < 10 && s.lab.machine->nextAttention(s.who) != kNever; ++i) {
            visits.push_back(s.lab.machine->nextAttention(s.who));
            s.lab.world.clock = visits.back();
            s.lab.machine->step(s.who, Detail::Far);
        }
        CHECK(visits == std::vector<BrainTime>({72000, 132000, 282000, 372000, 516000}));
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
    }
    // VISITED LATE, once, an hour on: each state is entered at the time it was due, not at the time of the visit
    {
        Scene s(Detail::Far, false);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.world.clock = 3600000;
        s.lab.machine->step(s.who, Detail::Far);
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
        CHECK(s.lab.machine->agent(s.who).stateEntered == 516000u);
    }
    // RAISED TO NEAR PART-WAY THROUGH THE DRIVE, at 200 s: 68 of the drive's 150 s are behind them, and the drive
    // skill is told so; the rest of the trip runs on skills and ends at the same time
    {
        Scene s(Detail::Far);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.runTo(200000);
        CHECK(s.lab.stateName(s.who) == "driving" && s.lab.drive->begun == 0 && s.lab.walk->started == 0);
        s.lab.machine->step(s.who, Detail::Near);
        CHECK(s.lab.drive->begun == 1);
        CHECK(approxEqual(s.lab.drive->trips[0].fraction, 68.0 / 150.0, kSameFraction));
        CHECK(s.lab.drive->trips[0].start == 132000u && s.lab.drive->trips[0].end == 282000u);
        Vec2 at;
        CHECK(s.lab.world.bodyPosition(s.who, at) && approxEqual(at, Vec2(644, 0), engine::kSamePoint));
        s.lab.runTo(600000);
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
        CHECK(s.lab.walk->started == 1 && s.lab.drive->parked == 1);
    }
    // ...AND PART-WAY THROUGH A WALK, at 400 s: 28 of the last walk's 144 s are behind them
    {
        Scene s(Detail::Far);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.runTo(400000);
        s.lab.machine->step(s.who, Detail::Near);
        CHECK(s.lab.walk->started == 1);
        CHECK(approxEqual(s.lab.walk->walks[s.who].fraction, 28.0 / 144.0, kSameFraction));
        s.lab.runTo(600000);
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
    }
    // LOWERED TO FAR PART-WAY THROUGH THE FIRST WALK: the skill stops, the clock carries on, the same times
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.setOut();
        s.lab.runTo(30000);
        s.lab.machine->step(s.who, Detail::Far);
        s.lab.world.bodyKnown[s.who] = 0;   // (a far person has no body)
        CHECK(s.lab.walk->cancelled == 1);
        s.lab.runTo(600000);
        CHECK(s.lab.traceOf(s.who) == kWholeTrip);
        CHECK(s.lab.walk->started == 1 && s.lab.drive->begun == 0);
    }
}

TEST_CASE(brain_trip_an_intent_changed_on_the_way_is_taken_up_on_arriving) {
    // A state that does not act on a new intent (driving) keeps going; on arriving, the person is not where they now
    // want to be, and goes on from there. From the door at 1500 m to one at 1900 m: 400 m on foot (4.8 min) beats the
    // car parked 200 m back, 288 s.
    Scene s(Detail::Near);
    if (!s.lab.ok) return;
    const PlaceRef further = s.lab.door(1900);
    s.setOut();
    s.lab.runTo(200000);
    s.lab.machine->setIntent(s.who, 0, further);
    s.lab.runTo(1000000);
    CHECK(s.lab.traceOf(s.who) ==
          "walking@0 pulling_out@72000 driving@132000 parking@282000 walking@372000 walking@516000 indoors@804000");
    CHECK(s.lab.machine->counts().arrivals == 1u && s.lab.machine->counts().plans == 2u);
    CHECK(s.lab.machine->agent(s.who).intentBegan == 200000u);
    // ...and a new intent while walking is acted on at once: back home from 100 m out, 72 s
    Lab lab;
    if (!lab.ok) return;
    const PlaceRef home = lab.door(0), shop = lab.door(500);
    const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
    lab.machine->setIntent(who, 0, shop);
    lab.tick();
    lab.runTo(71500);
    lab.machine->setIntent(who, 0, home);
    lab.runTo(300000);
    CHECK(lab.traceOf(who) == "walking@0 walking@72000 indoors@144000");
}
