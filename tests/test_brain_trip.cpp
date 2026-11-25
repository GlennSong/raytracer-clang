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

// A NEW INTENT DURING A CAR LEG (ADR-0442). The usual scene, with a second door at 1900 m and a free kerb space beside it
// at 1800 m (places 4 and 5). The person is told to be at the second door:
//   AT THE WHEEL, at 200 s: the car is 68 s into the drive, 544 m on, at 644 m. The drive is redirected: the plan
//     becomes "car to the space at 1800 m, walk to the door". 1156 m at 8 m/s is 144.5 s, so beside the space at
//     344.5 s; parked 90 s later; the last 100 m on foot take 72 s.
//   WHILE PULLING OUT, at 100 s: nothing happens until the drive begins at 132 s, and it is redirected as it begins: the
//     car sets off for the space at 1800 m, 1700 m and 212.5 s away. The same times from there.
//   WHILE PARKING, at 300 s: parking is finished (372 s). As they step out, the walk to the old door is dropped and
//     the trip planned again from beside the car at 1300 m: on foot 600 m is 7.2 min; by car 62.5 s of driving, 150 s
//     of pulling out and parking and 72 s on foot is 4.7 min, so they get back in. The walk to the car is no distance
//     and is reported at the next tick (372.5 s); out of the space at 432.5 s, beside the new one at 495 s, parked at
//     585 s, at the door at 657 s.
TEST_CASE(brain_trip_a_new_intent_at_the_wheel_redirects_the_drive) {
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        const PlaceRef further = s.lab.door(1900), spaceBeside = s.lab.space(1800);
        CHECK(further.id == 4u && spaceBeside.id == 5u);
        s.setOut();
        s.lab.runTo(199500);
        s.lab.machine->setIntent(s.who, 0, further);
        s.lab.runTo(200000);
        // redirected at once: still driving, the same car, a new plan of two legs
        CHECK(s.lab.stateName(s.who) == "driving" && s.lab.planOf(s.who) == "car>5 walk>4");
        CHECK(s.lab.machine->agent(s.who).legNow == 0 && s.lab.machine->agent(s.who).intentBegan == 199500u);
        // (the planner prices the drive from where the car is, at 644 m, along the street's walk to the space at
        // 1800 m: 1156 m is 144.5 s, and the way's 150 s of pulling out and parking -- the first minute of which is
        // already behind a driver at the wheel, a known over-count: 294.5 s is 4.9 min. On foot is not priced: nobody
        // leaves a car in a lane to walk.) RE-PINNED (#306, the street map's deletion) from "car 6.3 min (5.1 car +
        // 1.2 walk)": the street map priced it from its node nearest the car, at 600 m, 44 m off.
        CHECK(s.lab.planner->lastReason() ==
              "walk -- at the wheel of the car, car 6.1 min (4.9 car + 1.2 walk): car");
        CHECK(s.lab.drive->begun == 1 && s.lab.drive->redirected == 1 && s.lab.drive->ended == 0);
        Vec2 at;
        CHECK(s.lab.drive->where(s.car, at) && approxEqual(at, Vec2(644, 0), engine::kSamePoint));
        s.lab.runTo(700000);
        CHECK(s.lab.traceOf(s.who) ==
              "walking@0 pulling_out@72000 driving@132000 driving@200000 parking@344500 walking@434500 indoors@506500");
        const BrainCounts& c = s.lab.machine->counts();
        CHECK(c.plans == 2u && c.redirects == 1u && c.redirectsLeft == 0u && c.arrivals == 1u && c.refused == 0u && c.errors == 0u);
        CHECK(s.lab.world.cars[0].standing == spaceBeside && s.lab.world.cars[0].heldBy == kNoAgent);
        CHECK(s.lab.world.spaceCar[s.farSpace.id] == CarRef::kNone);   // the space first planned for was never taken
    }
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        const PlaceRef further = s.lab.door(1900);
        s.lab.space(1800);
        s.setOut();
        s.lab.runTo(100000);
        s.lab.machine->setIntent(s.who, 0, further);
        s.lab.runTo(131500);
        CHECK(s.lab.stateName(s.who) == "pulling_out" && s.lab.planOf(s.who) == "walk>2 car>3 walk>1");
        CHECK(s.lab.machine->counts().plans == 1u);
        s.lab.runTo(700000);
        CHECK(s.lab.traceOf(s.who) ==
              "walking@0 pulling_out@72000 driving@132000 driving@132000 parking@344500 walking@434500 indoors@506500");
        CHECK(s.lab.machine->counts().plans == 2u && s.lab.machine->counts().redirects == 1u);
        CHECK(s.lab.drive->begun == 1 && s.lab.drive->redirected == 1);
    }
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        const PlaceRef further = s.lab.door(1900), spaceBeside = s.lab.space(1800);
        s.setOut();
        s.lab.runTo(300000);
        s.lab.machine->setIntent(s.who, 0, further);
        s.lab.runTo(371500);
        CHECK(s.lab.stateName(s.who) == "parking" && s.lab.machine->counts().plans == 1u);
        s.lab.runTo(372000);
        CHECK(s.lab.planOf(s.who) == "walk>3 car>5 walk>4");
        CHECK(s.lab.planner->lastReason() == "walk 7.2 min, car 4.7 min (0.0 walk + 3.5 car + 1.2 walk): car");
        s.lab.runTo(800000);
        CHECK(s.lab.traceOf(s.who) ==
              "walking@0 pulling_out@72000 driving@132000 parking@282000 walking@372000 walking@372000 pulling_out@372500 "
              "driving@432500 parking@495000 walking@585000 indoors@657000");
        const BrainCounts& c = s.lab.machine->counts();
        CHECK(c.plans == 2u && c.redirects == 0u && c.arrivals == 1u && c.refused == 0u && c.errors == 0u);
        CHECK(s.lab.world.cars[0].standing == spaceBeside);
    }
}

// THE LANE BESIDE THE SPACE IS BUSY (ADR-0442): a car given a trip waits in its space until the lane is clear, and its
// way says how long it waits (the test ways: 2 minutes). The usual scene.
//   BUSY FOR GOOD: the drive begins at 132 s and the car does not move. At 252 s, two minutes on, the driving state
//     raises `blocked`; it is answered at the next tick, 252.5 s: the car is ruled out, they get out beside it at 100 m
//     and walk the 1400 m, 1008 s.
//   CLEARED AT 180 s: the car sets off at the next tick, 180.5 s; 1200 m at 8 m/s is 150 s, so beside its space at
//     330.5 s, parked at 420.5 s, and the 200 m on foot end at 564.5 s. Nothing is planned again.
TEST_CASE(brain_trip_a_car_that_cannot_set_off_is_given_up_as_blocked_after_its_ways_limit) {
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.lab.drive->laneBusy = true;
        s.setOut();
        s.lab.runTo(252000);
        CHECK(s.lab.stateName(s.who) == "driving" && s.lab.drive->waitingToSetOff(s.car));
        s.lab.runTo(252500);
        const AgentBrain& a = s.lab.machine->agent(s.who);
        CHECK(s.lab.stateName(s.who) == "walking" && a.ruledOut == (1u << kCar));
        CHECK(s.lab.planOf(s.who) == "walk>1" && a.legSeconds[0] == 1008);
        CHECK(s.lab.drive->ended == 1 && !s.lab.drive->waitingToSetOff(s.car));
        Vec2 at;
        CHECK(s.lab.drive->where(s.car, at) && approxEqual(at, Vec2(100, 0), engine::kSamePoint));   // it never moved
        CHECK(s.lab.world.cars[0].heldBy == kNoAgent);
        s.lab.runTo(1300000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 walking@252500 indoors@1260500");
        CHECK(s.lab.machine->counts().plans == 2u && s.lab.machine->counts().refused == 0u);
    }
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        s.lab.drive->laneBusy = true;
        s.setOut();
        s.lab.runTo(180000);
        CHECK(s.lab.drive->waitingToSetOff(s.car));
        s.lab.drive->laneBusy = false;
        s.lab.runTo(700000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 pulling_out@72000 driving@132000 parking@330500 walking@420500 indoors@564500");
        CHECK(s.lab.machine->counts().plans == 1u && s.lab.machine->agent(s.who).ruledOut == 0);
    }
    // A way with no limit waits for as long as it takes: an hour on, still in its space
    {
        TravelCatalog ways = testTravel();
        ways.modes[kCar].setOffWaitMinutes = 0;
        Lab lab(true, nullptr, &ways);
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), door = lab.door(1500), carSpace = lab.space(100);
        lab.space(1300);
        const CarRef car = lab.world.addCar(carSpace);
        const AgentId who = lab.person(home, 0, car, Detail::Near);
        lab.drive->laneBusy = true;
        lab.machine->setIntent(who, 0, door);
        lab.tick();
        lab.runTo(3600000);
        CHECK(lab.traceOf(who) == "walking@0 pulling_out@72000 driving@132000");
        CHECK(lab.drive->waitingToSetOff(car));
    }
}

// A WALK THAT CANNOT REACH ITS END, AND A PERSON WITH NO WAY THERE (ADR-0443). The stand-in walking has a gap in the
// pavements at one x: a leg by a way the gap stops, whose ends lie on either side of it, cannot be begun, which the
// walking state reports as `blocked`.
TEST_CASE(brain_trip_a_blocked_walk_goes_another_way_and_no_way_there_tries_again) {
    // ANOTHER WAY: with a board, 500 m is a board's leg (180 s). The board cannot pass 250 m; the feet can. The board is
    // ruled out and the trip walked, 360 s.
    {
        Lab lab;
        CHECK(lab.ok);
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, kHasBoard, CarRef(), Detail::Near);
        lab.walk->gapAt = 250;
        lab.walk->gapWays = 1u << kSkateboard;
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        CHECK(lab.planOf(who) == "walk>1");
        CHECK(lab.machine->agent(who).ruledOut == (1u << kSkateboard));
        lab.runTo(400000);
        CHECK(lab.traceOf(who) == "on_small_wheels@0 walking@0 indoors@360000");
        CHECK(lab.machine->counts().plans == 2u && lab.machine->counts().noPlans == 0u && lab.walk->refusedAtGap == 1);
    }
    // NO WAY THERE: no board, and nobody passes 250 m. They stand with the reason kept, and try again every 60 s (the
    // data's `seconds`) with every way allowed again; when the gap is mended at 150 s the try at 180 s walks, 360 s.
    {
        Lab lab;
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
        lab.walk->gapAt = 250;
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        const AgentBrain& a = lab.machine->agent(who);
        CHECK(lab.stateName(who) == "no_way" && a.legCount == 0 && a.stateEntered == 0u && a.stateDue == 60000u);
        CHECK(a.ruledOut == (1u << kWalk));
        CHECK(lab.planner->lastError() ==
              "no way there (walk -- ruled out (a leg of it could not be done), car -- no car: no way there)");
        CHECK(lab.machine->nextAttention(who) == 60000u);   // a host with a clock queue wakes them then
        CHECK(lab.machine->describe(who) ==
              "agent 0: wants be_at place 1 since 0.0 s | plan: none | no_way since 0.0 s, due 60.0 s, waiting for the "
              "clock (try_again) | near");
        lab.runTo(150000);
        CHECK(lab.traceOf(who) == "walking@0 no_way@0 walking@60000 no_way@60000 walking@120000 no_way@120000");
        CHECK(lab.machine->counts().noPlans == 3u && lab.machine->counts().plans == 3u && a.stateDue == 180000u);
        lab.walk->gapAt = -1;   // mended
        lab.runTo(600000);
        CHECK(lab.traceOf(who) ==
              "walking@0 no_way@0 walking@60000 no_way@60000 walking@120000 no_way@120000 walking@180000 indoors@540000");
        CHECK(lab.machine->counts().noPlans == 3u && lab.machine->counts().plans == 4u && lab.machine->counts().arrivals == 1u);
        CHECK(lab.machine->counts().refused == 0u && lab.machine->counts().errors == 0u);
    }
    // SOMETHING CHANGES, AND THEY DO NOT WAIT THE MINUTE: given a board at 20 s (the gap stops feet only), they are on
    // it at once: 500 m, 180 s
    {
        Lab lab;
        if (!lab.ok) return;
        const PlaceRef home = lab.door(0), shop = lab.door(500);
        const AgentId who = lab.person(home, 0, CarRef(), Detail::Near);
        lab.walk->gapAt = 250;
        lab.walk->gapWays = 1u << kWalk;
        lab.machine->setIntent(who, 0, shop);
        lab.tick();
        CHECK(lab.stateName(who) == "no_way");
        lab.runTo(19500);
        lab.machine->setTraits(who, kHasBoard);
        lab.runTo(300000);
        CHECK(lab.traceOf(who) == "walking@0 no_way@0 on_small_wheels@20000 indoors@200000");
        CHECK(lab.machine->counts().noPlans == 1u);
    }
    // ...AND WHEN THE HOUSEHOLD'S CAR BECOMES FREE. The usual scene, the feet stopped at 700 m (between the car at 100 m
    // and the far space at 1300 m), and someone else has the car: no way. At 30 s they give the car up and the person
    // is told: walk to the car (72 s), drive (60 + 150 + 90 s), walk to the door (144 s).
    {
        Scene s(Detail::Near);
        if (!s.lab.ok) return;
        const AgentId other = s.lab.person(s.lab.door(100), 0, s.car, Detail::Near);
        CHECK(s.lab.world.claimCar(s.car, other));
        s.lab.walk->gapAt = 700;
        s.lab.walk->gapWays = 1u << kWalk;
        s.setOut();
        CHECK(s.lab.stateName(s.who) == "no_way" && s.lab.traceOf(s.who) == "walking@0 no_way@0");
        CHECK(s.lab.planner->lastError() ==
              "no way there (walk -- ruled out (a leg of it could not be done), car -- the car is in use: no way there)");
        s.lab.runTo(29500);
        s.lab.world.releaseCar(s.car, other);
        s.lab.machine->notifyChanged(s.who);
        s.lab.runTo(40000);
        CHECK(s.lab.planOf(s.who) == "walk>2 car>3 walk>1" && s.lab.stateName(s.who) == "walking");
        // (told again while walking: a trip under way is not planned again for it)
        const uint64_t plans = s.lab.machine->counts().plans;
        s.lab.machine->notifyChanged(s.who);
        s.lab.runTo(600000);
        CHECK(s.lab.machine->counts().plans == plans);
        CHECK(s.lab.traceOf(s.who) ==
              "walking@0 no_way@0 walking@30000 pulling_out@102000 driving@162000 parking@312000 walking@402000 indoors@546000");
        CHECK(s.lab.machine->counts().refused == 0u && s.lab.machine->counts().errors == 0u);
    }
}

TEST_CASE(brain_trip_a_new_intent_on_foot_is_acted_on_at_once) {
    // back home from 100 m out, 72 s
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

// ---- BY BUS (ADR-0484): walk to a stop, wait for the line's bus, ride it round to a stop near the door, walk on ----------
namespace {
// testTravel() with a bus row (its mean wait the line's: half its 8-minute headway, 4 minutes; nothing at either end)
TravelCatalog withBus() {
    TravelCatalog c = testTravel();
    TravelMode bus;
    bus.name = "bus";
    bus.transit = "bus";
    bus.speed = 0;
    bus.beginsIn = "waiting";
    c.modes.push_back(bus);
    return c;
}
// One line with stops at 200, 800, 1400 and 1900 m; 100 s from each to the next, 300 s from the last round to the first
struct BusScene {
    TravelCatalog travel = withBus();
    Lab lab;
    PlaceRef home, door, s200, s800, s1400, s1900;
    AgentId who = kNoAgent;
    explicit BusScene(Detail detail) : lab(true, nullptr, &travel) {
        if (!lab.ok) return;
        home = lab.door(100);
        door = lab.door(1500);
        s200 = lab.world.addStop(0, 0, Vec2(200, 0));
        s800 = lab.world.addStop(0, 1, Vec2(800, 0));
        s1400 = lab.world.addStop(0, 2, Vec2(1400, 0));
        s1900 = lab.world.addStop(0, 3, Vec2(1900, 0));
        TransitTimetable::Line line;
        line.network = "bus";
        line.source = 0;
        line.headwaySeconds = 480;
        for (const Real x : {200.0, 800.0, 1400.0, 1900.0}) line.stops.push_back(TransitTimetable::Stop{Vec2(x, 0), x == 1900.0 ? 300.0 : 100.0, 0});
        lab.street.transit.lines = {line};
        who = lab.person(home, 0, CarRef(), detail);
    }
};
}  // namespace

TEST_CASE(brain_trip_by_bus_walk_wait_ride_walk) {
    // FAR: the clock alone. Walk 100 m to the stop at 200 m (72 s), wait the mean 4 minutes, ride two stops (200 s),
    // walk 100 m from the stop at 1400 m (72 s): 9.7 minutes against 16.8 on foot
    {
        BusScene s(Detail::Far);
        CHECK(s.lab.ok);
        if (!s.lab.ok) return;
        s.lab.machine->setIntent(s.who, 0, s.door);
        s.lab.tick();
        std::printf("    [bus] plan %s (%s)\n", s.lab.planOf(s.who).c_str(), s.lab.planner->lastReason().c_str());
        CHECK(s.lab.planOf(s.who) == "walk>" + std::to_string(s.s200.id) + " bus>" + std::to_string(s.s1400.id) + " walk>" +
                                         std::to_string(s.door.id));
        CHECK(s.lab.planner->lastReason() == "walk 16.8 min, car -- no car, bus 9.7 min (1.2 walk + 7.3 bus + 1.2 walk): bus");
        s.lab.runTo(700000);
        CHECK(s.lab.traceOf(s.who) == "walking@0 waiting@72000 riding@312000 walking@512000 indoors@584000");
    }
    // NEAR: the skills. The ride skill is asked to wait at the 200 m stop for line 0, to get off at 1400 m; nothing moves
    // until a bus comes; the bus at a stop that is not theirs changes nothing
    {
        BusScene s(Detail::Near);
        if (!s.lab.ok) return;
        s.lab.machine->setIntent(s.who, 0, s.door);
        s.lab.runTo(80000);
        CHECK(s.lab.stateName(s.who) == "waiting");
        CHECK(s.lab.ride->riders.size() == 1u && s.lab.ride->riders[0].board == s.s200 && s.lab.ride->riders[0].alight == s.s1400 &&
              s.lab.ride->riders[0].line == 0u);
        s.lab.ride->busAt(*s.lab.machine, 0, s.s800);   // (not their stop)
        s.lab.runTo(400000);
        CHECK(s.lab.stateName(s.who) == "waiting");    // a near person waits for the bus, not the clock
        s.lab.ride->busAt(*s.lab.machine, 0, s.s200);
        s.lab.runTo(400500);
        CHECK(s.lab.stateName(s.who) == "riding");
        s.lab.ride->busAt(*s.lab.machine, 0, s.s800);   // (still aboard)
        s.lab.runTo(401000);
        CHECK(s.lab.stateName(s.who) == "riding");
        s.lab.ride->busAt(*s.lab.machine, 0, s.s1400);
        s.lab.runTo(401500);
        CHECK(s.lab.stateName(s.who) == "walking");
        s.lab.runTo(500000);
        CHECK(s.lab.stateName(s.who) == "indoors");
        CHECK(s.lab.ride->riders.empty() && s.lab.ride->waits == 1 && s.lab.ride->cancels == 0);
        std::printf("    [bus] near: %s\n", s.lab.traceOf(s.who).c_str());
    }
}
