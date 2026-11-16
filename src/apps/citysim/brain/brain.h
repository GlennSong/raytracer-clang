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

    // A NEW PERSON at `home`, wanting to be there (intent `kind`), in the table's start state.
    AgentId addAgent(PlaceRef home, IntentKind kind, uint8_t traits, CarRef car, Detail detail);
    std::size_t agentCount() const { return agents_.size(); }
    const AgentBrain& agent(AgentId id) const { return agents_[id]; }

    // A NEW INTENT: `who` now wants `kind` at `target`. The ways ruled out for the last intent are allowed again. It
    // arrives as the table's intent event, so the person's current state says what happens.
    void setIntent(AgentId who, IntentKind kind, PlaceRef target);
    // AN EVENT for `who`, answered at the next tick.
    void post(AgentId who, EventId kind, uint32_t payload = 0);
    // The same by the event's name in the data. False: no such event.
    bool post(AgentId who, const std::string& event, uint32_t payload = 0);

    // Answer the posted events, then step everybody, at the world's time.
    void tick();
    // Visit one person at `level`. A change of level is carried out first: raised to near, the state is taken up
    // part-way with the fraction of its planned time already gone; lowered, its skill is stopped.
    void step(AgentId who, Detail level);
    // When `who` next needs a visit if no event comes first: the time a clock-ended state is due. kNever: not until an
    // event. (A near person in a state a skill ends is waiting for that skill's event.)
    BrainTime nextAttention(AgentId who) const;

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
    void resume(AgentId who, BrainTime at, int depth);
    void runClock(AgentId who, bool nearNow);
    Vec2 whereIs(AgentId who, BrainTime at) const;
    BrainTime lastsOf(const StepContext& ctx, const AgentBrain& a) const;
    Real fractionDone(const StepContext& ctx, const AgentBrain& a) const;
    void fail(AgentId who, const char* what, const char* name);
    void noteEntered(AgentId who, StateId state, BrainTime at);

    const BrainTable& table_;
    World& world_;
    Planner& planner_;
    WalkSkill* walk_;
    DriveSkill* drive_;
    std::vector<AgentBrain> agents_;
    std::vector<BrainEvent> incoming_, draining_;
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
