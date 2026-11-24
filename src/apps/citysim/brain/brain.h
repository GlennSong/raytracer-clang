#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_H

// THE AGENT BRAIN'S MACHINE (docs/agent-brain-design.md; ADR-0440). It holds every person's record (brain_record.h),
// takes events in, and moves each person from state to state as the table says (brain_table.h). It names no state, no
// event and no way of travelling: those are rows of brain.lua and travel.lua.
//
// ONE TICK (tick()): the events posted since the last tick are put in order -- by person, lowest first, then in the
// order they came -- and each is answered by its person's current state. Then every person is stepped at their level
// of detail. A NEAR person's skills are running, and report by events. A FAR person has no skill: the state's planned
// end time passing stands for the skill's report. A person whose state ends by the clock at every level (pulling out,
// parking) is moved on by the clock at every level. A state entered because the clock ran out is entered AT the time
// it ran out, not at the time the machine noticed, so a far person visited late has the same history as one visited
// on time.
//
// A NEW INTENT THAT A STATE DID NOT ACT ON IS NOT LOST (ADR-0442): the plan under way then no longer leads where the
// person wants to be (its last leg ends somewhere else). On entering each later state of that trip the intent's event
// is delivered again, and that state's row says what happens: a driver who was told while pulling out is redirected as
// the drive begins; one told while parking plans again as they step out of the car. A trip whose every later state
// ignores it is finished, and the person goes on from where it ends.
//
// A HOST WITH A CLOCK QUEUE need not call tick() for far people: nextAttention() says when a person next needs a
// visit, and step() visits one. tick() itself visits everybody, which is simple and is what the tests use.
//
// NOT IN THIS CLASS: Lua (the table is compiled before it is handed over), routes (a skill's), bodies (a skill's).
// Nothing is allocated per person, and a transition allocates nothing; planning again calls the planner, which may.

#include "brain_record.h"
#include "brain_skills.h"
#include "brain_table.h"

#include <cstdint>
#include <string>
#include <vector>

namespace citysim {
namespace brain {

// What the machine has done, for tests and for a host's own checks.
struct BrainCounts {
    uint64_t transitions = 0;    // states entered
    uint64_t plans = 0;          // plans made
    uint64_t noPlans = 0;        // times there was no way there
    uint64_t arrivals = 0;       // intents satisfied
    uint64_t refused = 0;        // events that should not have come (the table's `refuse`)
    uint64_t errors = 0;         // things the machine could not do (lastError says the latest)
    uint64_t events = 0;         // events answered
    uint64_t redirects = 0;      // legs under way sent somewhere else (the verb `redirect`)
    uint64_t redirectsLeft = 0;  // ...asked for and not made (no plan in the vehicle): the old trip carries on
};

// One state entered: who, which, and at what time. What a trace is a list of.
struct Transition {
    AgentId agent = kNoAgent;
    StateId state = kNoId;
    BrainTime at = 0;
};

class Machine {
public:
    // Every state entered from now on is added to `out` (null: stop). For tests and for watching one person; the
    // list is the caller's and grows as the caller lets it.
    void traceTo(std::vector<Transition>* out) { trace_ = out; }

    // The table must already be compiled and its ways bound. The skills may be null for a machine whose people are all
    // far (no skill is ever asked).
    Machine(const BrainTable& table, World& world, Planner& planner, WalkSkill* walk, DriveSkill* drive);
    // THE RIDE SKILL (ADR-0484): who carries people on buses. Null (the default): nobody near rides; a near person given a
    // bus leg is told it cannot be done.
    void setRide(RideSkill* ride) { ride_ = ride; }

    // A NEW PERSON at `home`, wanting to be there (intent `kind`) and there already: in that intent's arriving state
    // (indoors at a building, standing at a place with none), or the table's start state where that is not built.
    AgentId addAgent(PlaceRef home, IntentKind kind, uint8_t traits, CarRef car, Detail detail);
    std::size_t agentCount() const { return agents_.size(); }
    const AgentBrain& agent(AgentId id) const { return agents_[id]; }
    // WHAT A PERSON HAS, changed (they were given a board, or lost it): bit i for travel.lua traits[i]. Read when they
    // next plan; a trip under way is not planned again for it. They are told something changed (notifyChanged).
    void setTraits(AgentId who, uint8_t traits);
    // SOMETHING `who` PLANS WITH HAS CHANGED (ADR-0443: their household's car became free or was moved; a trait). The
    // table's changed event is posted to them; their state's row says what follows (a person with no way there plans
    // again; anyone else carries on). Nothing when the data names no such event.
    void notifyChanged(AgentId who);

    // A NEW INTENT: `who` now wants `kind` at `target`. The ways ruled out for the last intent are allowed again. It
    // arrives as the table's intent event, so the person's current state says what happens.
    void setIntent(AgentId who, IntentKind kind, PlaceRef target);
    // AN EVENT for `who`, answered at the next tick.
    void post(AgentId who, EventId kind, uint32_t payload = 0);
    // The same by the event's name in the data. False: no such event.
    bool post(AgentId who, const std::string& event, uint32_t payload = 0);

    // Answer the posted events, then step everybody, at the world's time.
    void tick();
    // A HOST WITH ITS OWN ATTENTION (a clock queue): answer the posted events only; the host then steps whom it will.
    // Returns everyone an event was answered for, lowest first, each once (their next attention may have moved).
    const std::vector<AgentId>& answerEvents();
    // Visit one person at `level`. A change of level is carried out first: raised to near, the state is taken up
    // part-way with the fraction of its planned time already gone; lowered, its skill is stopped.
    void step(AgentId who, Detail level);
    // When `who` next needs a visit if no event comes first: the time a clock-ended state is due. kNever: not until an
    // event. (A near person in a state a skill ends is waiting for that skill's event.)
    BrainTime nextAttention(AgentId who) const;
    // WHERE `who` IS at `at`, as far as the brain can say (a far person part-way along a leg: on the straight line
    // between its ends, by the share of its planned time gone; otherwise their place, or a leg's end).
    Vec2 whereIs(AgentId who, BrainTime at) const;

    // ONE LINE about a person: intent, plan, the leg under way, state, since when, waiting for what.
    std::string describe(AgentId who) const;
    // Every record, hashed (the repo's FNV-1a): the same calls give the same number.
    uint64_t stateHash() const;

    const BrainCounts& counts() const { return counts_; }
    const char* lastError() const { return lastError_; }
    const BrainTable& table() const { return table_; }

    // FOR THE STATES' FUNCTIONS (brain_states.cpp): where the leg under way starts and ends.
    bool legStart(const AgentBrain& a, Vec2& out) const;
    bool legEnd(const AgentBrain& a, Vec2& out) const;
    // The place the leg under way starts at, when a leg before it says so (not valid for the first leg, which starts
    // at the plan's origin).
    PlaceRef legStartPlace(const AgentBrain& a) const;

private:
    StepContext contextFor(AgentId who, BrainTime at) const;
    void answer(AgentId who, EventId kind, uint32_t payload, BrainTime at, int depth);
    // Enter `target` afresh at `at`. False: it is not built.
    bool enterState(AgentId who, StateId target, BrainTime at, int depth);
    void leaveState(AgentId who, ExitWhy why, BrainTime at);
    void beginLeg(AgentId who, BrainTime at, int depth);
    void arrive(AgentId who, BrainTime at, int depth);
    void replan(AgentId who, bool ruleOutThisWay, bool leave, BrainTime at, int depth);
    void redirect(AgentId who, BrainTime at, int depth);
    // `legs` become the person's plan, from its first leg; the space a vehicle leg ends in is held for them.
    void takePlan(AgentId who, const PlanLegs& legs);
    void resume(AgentId who, BrainTime at, int depth);
    void runClock(AgentId who, bool nearNow);
    BrainTime lastsOf(const StepContext& ctx, const AgentBrain& a) const;
    Real fractionDone(const StepContext& ctx, const AgentBrain& a) const;
    void fail(AgentId who, const char* what, const char* name);
    void noteEntered(AgentId who, StateId state, BrainTime at);

    const BrainTable& table_;
    World& world_;
    Planner& planner_;
    WalkSkill* walk_;
    DriveSkill* drive_;
    RideSkill* ride_ = nullptr;
    std::vector<AgentBrain> agents_;
    std::vector<BrainEvent> incoming_, draining_;
    std::vector<AgentId> answered_;
    uint32_t posted_ = 0;
    BrainCounts counts_;
    char lastError_[160] = {0};
    std::vector<Transition>* trace_ = nullptr;
};

// A place's plan point in the record's whole centimetres, and back.
int32_t toCentimetres(Real metres);
Real fromCentimetres(int32_t centimetres);

}  // namespace brain
}  // namespace citysim

#endif
