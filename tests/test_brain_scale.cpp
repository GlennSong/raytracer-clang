// A THOUSAND PEOPLE THROUGH THE AGENT BRAIN (ADR-0440): two runs give the same state hash whatever order the skills
// post their reports in, and stepping the machine allocates nothing.
//
// The allocation count is real: this program replaces the global operator new with one that counts while a test asks it
// to. (That is why the brain's tests are their own program.) What is counted is every tick of a run AFTER the plans are
// made: answering events, entering and leaving states, the clock, and the stand-in skills. Planning is not in it, and
// does allocate: it calls the trip planner, whose routes are vectors.

#include "test_framework.h"

#include "brain_fakes.h"

#include "hash.h"

#include <cstdlib>
#include <new>
#include <vector>

namespace {
long allocationsCounted = 0;
bool countingAllocations = false;
}  // namespace

void* operator new(std::size_t size) {
    if (countingAllocations) ++allocationsCounted;
    void* p = std::malloc(size ? size : 1);
    if (!p) throw std::bad_alloc();
    return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using namespace brain_test;

namespace {

constexpr int kPeople = 1000;
constexpr int kNodes = 21;            // the street's nodes, 100 m apart: a door at each
constexpr int kSpacesPerNode = 60;    // kerb spaces at each
constexpr BrainTime kRunFor = 1700000;   // the longest trip is the whole street on foot, 1440 s

struct Crowd {
    std::vector<PlaceRef> doors;
    std::vector<PlaceRef> wants;      // by person: the door they will ask for
    std::vector<BrainTime> setsOut;   // by person
    std::vector<int> spacesUsed = std::vector<int>(kNodes, 0);
};

// A kerb space at `node` nobody stands in yet (not valid: all taken).
PlaceRef freeSpaceAt(const Crowd& crowd, int node, int used) {
    PlaceRef p;
    if (used < kSpacesPerNode) p.id = crowd.doors.back().id + 1 + static_cast<uint32_t>(node * kSpacesPerNode + used);
    return p;
}

// The crowd, made the same way every time from hashes of each person's number. `households`: people come in pairs
// who live at one door, share one car and set out at the same moment (so who reaches the car first is decided by the
// machine's order alone). Otherwise nobody shares or competes: the first 21 people have a car each, live one to a door
// and each want a different door (so no two cars are sent to one space), and everyone sets out at time 0.
constexpr int kLoneDrivers = kNodes;
constexpr int kLoneDriverHop = 7;
Crowd populate(Lab& lab, bool households) {
    Crowd crowd;
    for (int n = 0; n < kNodes; ++n) crowd.doors.push_back(lab.door(100.0 * n));
    for (int n = 0; n < kNodes; ++n)
        for (int k = 0; k < kSpacesPerNode; ++k) lab.space(100.0 * n);
    for (int i = 0; i < kPeople; ++i) {
        const bool loneDriver = !households && i < kLoneDrivers;
        const uint32_t home = households && i < 600 ? static_cast<uint32_t>(i / 2) + 5000u : static_cast<uint32_t>(i);
        const int homeNode = loneDriver ? i : static_cast<int>(engine::lowbias32(home) % kNodes);
        const bool carOwner = households ? (i < 600 && i % 2 == 0) : loneDriver;
        const bool sharesCar = households && i < 600 && i % 2 == 1;
        CarRef car;
        if (carOwner) {
            const PlaceRef space = freeSpaceAt(crowd, homeNode, crowd.spacesUsed[static_cast<std::size_t>(homeNode)]);
            if (space.valid()) {
                ++crowd.spacesUsed[static_cast<std::size_t>(homeNode)];
                car = lab.world.addCar(space);
            }
        } else if (sharesCar) {
            car.id = lab.machine->agent(static_cast<AgentId>(i - 1)).car;   // the one their housemate was just given
        }
        const uint8_t traits = static_cast<uint8_t>(engine::lowbias32(static_cast<uint32_t>(i) + 99u) % 4);
        const Detail detail = engine::lowbias32(home + 31u) % 3 == 0 ? Detail::Far : Detail::Near;
        lab.person(crowd.doors[static_cast<std::size_t>(homeNode)], traits, car, detail);
        const int hop = loneDriver ? kLoneDriverHop
                                   : 1 + static_cast<int>(engine::lowbias32(static_cast<uint32_t>(i) + 7777u) % (kNodes - 1));
        crowd.wants.push_back(crowd.doors[static_cast<std::size_t>((homeNode + hop) % kNodes)]);
        crowd.setsOut.push_back(households ? static_cast<BrainTime>(engine::lowbias32(home + 5u) % 8) * kTick : 0);
    }
    lab.drive->trips.resize(lab.world.cars.size());
    return crowd;
}

struct RunResult {
    uint64_t hash = 0;
    BrainCounts counts;
    int vehicleGone = 0;   // people who found the car taken (entered pulling out and left it at the same instant)
    int spaceTaken = 0;    // drivers who found their space taken (entered parking and left it at the same instant)
};

// One run of the households. `reversed`: the stand-in skills post their reports, and the test sets intents, highest
// person first instead of lowest.
RunResult runHouseholds(bool reversed) {
    RunResult out;
    Lab lab;
    if (!lab.ok) return out;
    const Crowd crowd = populate(lab, true);
    lab.walk->reversed = reversed;
    lab.drive->reversed = reversed;
    for (BrainTime t = 0; t <= kRunFor; t += kTick) {
        lab.world.clock = t;
        for (int k = 0; k < kPeople; ++k) {
            const int i = reversed ? kPeople - 1 - k : k;
            if (crowd.setsOut[static_cast<std::size_t>(i)] == t)
                lab.machine->setIntent(static_cast<AgentId>(i), 0, crowd.wants[static_cast<std::size_t>(i)]);
        }
        lab.tick();
    }
    out.hash = lab.machine->stateHash();
    out.counts = lab.machine->counts();
    // (a state entered and left at one instant is followed in the trace by the same person's next state at that time)
    const StateId pullingOut = lab.state("pulling_out"), parking = lab.state("parking");
    for (std::size_t i = 0; i + 1 < lab.trace.size(); ++i) {
        if (lab.trace[i + 1].agent != lab.trace[i].agent || lab.trace[i + 1].at != lab.trace[i].at) continue;
        if (lab.trace[i].state == pullingOut) ++out.vehicleGone;
        if (lab.trace[i].state == parking) ++out.spaceTaken;
    }
    return out;
}

}  // namespace

TEST_CASE(brain_scale_two_runs_of_a_thousand_people_give_the_same_hash) {
    const RunResult first = runHouseholds(false);
    const RunResult again = runHouseholds(false);
    const RunResult reversed = runHouseholds(true);
    std::printf("    hash %016llx, %llu plans, %llu transitions, %llu events; %d found the car taken, %d their space\n",
                static_cast<unsigned long long>(first.hash), static_cast<unsigned long long>(first.counts.plans),
                static_cast<unsigned long long>(first.counts.transitions),
                static_cast<unsigned long long>(first.counts.events), first.vehicleGone, first.spaceTaken);
    CHECK(first.hash != 0 && first.hash == again.hash);
    // the skills' reports and the intents posted in the opposite order: the machine takes events lowest person first,
    // so the answer is the same
    CHECK(first.hash == reversed.hash);
    CHECK(first.counts.transitions == reversed.counts.transitions && first.counts.plans == reversed.counts.plans);
    // everybody got there, and the run did exercise the shared car: some found it taken and planned again
    CHECK(first.counts.arrivals == static_cast<uint64_t>(kPeople));
    CHECK(first.counts.noPlans == 0u && first.counts.errors == 0u && first.counts.refused == 0u);
    CHECK(first.vehicleGone > 50);
    CHECK(first.counts.plans == static_cast<uint64_t>(kPeople + first.vehicleGone + first.spaceTaken));
}

TEST_CASE(brain_scale_stepping_the_machine_allocates_nothing) {
    Lab lab;
    CHECK(lab.ok);
    if (!lab.ok) return;
    const Crowd crowd = populate(lab, false);
    lab.machine->traceTo(nullptr);   // (a trace is the caller's list, and grows)
    for (int i = 0; i < kPeople; ++i) lab.machine->setIntent(static_cast<AgentId>(i), 0, crowd.wants[static_cast<std::size_t>(i)]);
    lab.tick();   // every plan is made here, by the trip planner, which allocates
    CHECK(lab.machine->counts().plans == static_cast<uint64_t>(kPeople));
    const uint64_t transitionsBefore = lab.machine->counts().transitions;
    allocationsCounted = 0;
    countingAllocations = true;
    for (BrainTime t = kTick; t <= kRunFor; t += kTick) {
        lab.world.clock = t;
        lab.tick();
    }
    countingAllocations = false;
    const uint64_t transitions = lab.machine->counts().transitions - transitionsBefore;
    std::printf("    %llu transitions and %llu events with %ld allocations\n", static_cast<unsigned long long>(transitions),
                static_cast<unsigned long long>(lab.machine->counts().events), allocationsCounted);
    CHECK(allocationsCounted == 0);
    // ...over a run that did the work: everyone's arrival, and for each driver at least driving, parking and the last
    // walk (a far driver's first walk of no metres and pulling out fall in the first tick), with no plan made after
    // the first tick (so nothing in the count was a plan)
    CHECK(transitions >= static_cast<uint64_t>(kPeople + kLoneDrivers * 3));
    CHECK(lab.machine->counts().plans == static_cast<uint64_t>(kPeople));
    CHECK(lab.machine->counts().arrivals == static_cast<uint64_t>(kPeople));
    CHECK(lab.machine->counts().errors == 0u);
    // the counter does count: one vector made while it is on
    allocationsCounted = 0;
    countingAllocations = true;
    std::vector<int>* probe = new std::vector<int>(8, 1);
    countingAllocations = false;
    CHECK(allocationsCounted == 2);
    delete probe;
}
