#include "brain.h"

#include "../../../engine/bundle/bundle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace citysim {
namespace brain {

namespace {

// How many transitions one event may set off at one instant (a leg that cannot begin plans again, whose first leg
// cannot begin either, ...). Each planning again rules a way out or starts from a real state, so a healthy table stays
// far under it; reaching it is reported and the person stays where they are.
constexpr int kMaxChain = 16;
constexpr Real kCentimetresPerMetre = 100;

std::string secondsText(BrainTime t) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f s", static_cast<double>(t) / kMillisPerSecond);
    return b;
}

}  // namespace

int32_t toCentimetres(Real metres) { return static_cast<int32_t>(std::lround(metres * kCentimetresPerMetre)); }
Real fromCentimetres(int32_t centimetres) { return static_cast<Real>(centimetres) / kCentimetresPerMetre; }

Machine::Machine(const BrainTable& table, World& world, Planner& planner, WalkSkill* walk, DriveSkill* drive)
    : table_(table), world_(world), planner_(planner), walk_(walk), drive_(drive) {}

void Machine::fail(AgentId who, const char* what, const char* name) {
    std::snprintf(lastError_, sizeof(lastError_), "brain: agent %u: `%s` %s", who, name, what);
    ++counts_.errors;
}

void Machine::noteEntered(AgentId who, StateId state, BrainTime at) {
    ++counts_.transitions;
    if (!trace_) return;
    Transition t;
    t.agent = who;
    t.state = state;
    t.at = at;
    trace_->push_back(t);
}

AgentId Machine::addAgent(PlaceRef home, IntentKind kind, uint8_t traits, CarRef car, Detail detail) {
    AgentBrain a;
    std::memset(&a, 0, sizeof(a));
    const BrainTime now = world_.now();
    a.intentKind = kind;
    a.intentTarget = home.id;
    a.intentBegan = now;
    // they are where they want to be: in their intent's arriving state (the table's start state when that is not built)
    const StateId there = kind < table_.intents.size() ? table_.intents[kind].arrive : kNoId;
    a.state = there != kNoId && table_.states[there].fns.built ? there : table_.start;
    a.stateEntered = now;
    a.stateDue = kNever;
    a.heldState = kNoId;
    a.traits = traits;
    a.car = car.id;
    a.detail = static_cast<uint8_t>(detail);
    Vec2 at;
    if (world_.positionOf(home, at)) {
        a.originX = toCentimetres(at.x);
        a.originZ = toCentimetres(at.y);
    }
    agents_.push_back(a);
    // room for a few events a person a tick in both lists, so answering events does not allocate once the crowd is in
    const std::size_t room = agents_.capacity() * 4;
    if (incoming_.capacity() < room) incoming_.reserve(room);
    if (draining_.capacity() < room) draining_.reserve(room);
    return static_cast<AgentId>(agents_.size() - 1);
}

void Machine::setIntent(AgentId who, IntentKind kind, PlaceRef target) {
    AgentBrain& a = agents_[who];
    a.intentKind = kind;
    a.intentTarget = target.id;
    a.intentBegan = world_.now();
    a.ruledOut = 0;
    post(who, table_.intentEvent);
}

void Machine::post(AgentId who, EventId kind, uint32_t payload) {
    BrainEvent e;
    e.agent = who;
    e.kind = kind;
    e.payload = payload;
    e.order = posted_++;
    incoming_.push_back(e);
}

bool Machine::post(AgentId who, const std::string& event, uint32_t payload) {
    const int e = table_.event(event);
    if (e < 0) return false;
    post(who, static_cast<EventId>(e), payload);
    return true;
}

StepContext Machine::contextFor(AgentId who, BrainTime at) const {
    StepContext ctx;
    ctx.machine = const_cast<Machine*>(this);
    ctx.table = &table_;
    ctx.world = &world_;
    ctx.walk = walk_;
    ctx.drive = drive_;
    ctx.agent = who;
    ctx.now = at;
    ctx.skillsRun = static_cast<Detail>(agents_[who].detail) != Detail::Far;
    return ctx;
}

BrainTime Machine::lastsOf(const StepContext& ctx, const AgentBrain& a) const {
    const StateFns& fns = table_.states[a.state].fns;
    return fns.lasts ? fns.lasts(ctx, a) : kNever;
}

Real Machine::fractionDone(const StepContext& ctx, const AgentBrain& a) const {
    const BrainTime lasts = lastsOf(ctx, a);
    if (lasts == kNever || lasts == 0 || ctx.now <= a.stateEntered) return 0;
    const BrainTime elapsed = std::min(ctx.now - a.stateEntered, lasts);
    return static_cast<Real>(elapsed) / static_cast<Real>(lasts);
}

PlaceRef Machine::legStartPlace(const AgentBrain& a) const {
    PlaceRef p;
    if (a.legNow > 0 && a.legNow <= a.legCount) p.id = a.legEnd[a.legNow - 1];
    return p;
}

bool Machine::legStart(const AgentBrain& a, Vec2& out) const {
    if (a.legNow == 0) {
        out = Vec2(fromCentimetres(a.originX), fromCentimetres(a.originZ));
        return true;
    }
    return world_.positionOf(legStartPlace(a), out);
}

bool Machine::legEnd(const AgentBrain& a, Vec2& out) const {
    if (!a.travelling()) return false;
    PlaceRef p;
    p.id = a.legEnd[a.legNow];
    return world_.positionOf(p, out);
}

// Where the person is at `at`: their body when they have one, or else worked out from the plan and the state.
Vec2 Machine::whereIs(AgentId who, BrainTime at) const {
    const AgentBrain& a = agents_[who];
    Vec2 p;
    const StepContext ctx = contextFor(who, at);
    if (ctx.skillsRun && world_.bodyPosition(who, p)) return p;
    const Vec2 origin(fromCentimetres(a.originX), fromCentimetres(a.originZ));
    if (!a.travelling()) return origin;
    // a held person is where the state that was interrupted had got to
    const StateRow& now = table_.states[a.state];
    const bool held = now.holds && a.heldState != kNoId;
    AgentBrain as = a;
    Real fraction = 0;
    if (held) {
        as.state = a.heldState;
        const BrainTime lasts = lastsOf(ctx, as);
        if (lasts != kNever && lasts > 0 && a.payload.remaining != kNever)
            fraction = Real(1) - static_cast<Real>(std::min(a.payload.remaining, lasts)) / static_cast<Real>(lasts);
    } else {
        fraction = fractionDone(ctx, a);
    }
    Vec2 from = origin, to = origin;
    legStart(a, from);
    if (!legEnd(a, to)) to = from;
    switch (table_.states[as.state].where) {
        case Where::Place: return origin;
        case Where::LegStart: return from;
        case Where::LegEnd: return to;
        case Where::AlongLeg: return from + (to - from) * fraction;
    }
    return origin;
}

void Machine::leaveState(AgentId who, ExitWhy why, BrainTime at) {
    AgentBrain& a = agents_[who];
    const StateFns& fns = table_.states[a.state].fns;
    if (!fns.exit) return;
    StepContext ctx = contextFor(who, at);
    fns.exit(ctx, a, why);
}

bool Machine::enterState(AgentId who, StateId target, BrainTime at, int depth) {
    const StateRow& row = table_.states[target];
    if (!row.fns.built) return false;
    AgentBrain& a = agents_[who];
    a.state = target;
    a.stateEntered = at;
    if (!row.holds) {
        a.heldState = kNoId;
        a.payload.word = 0;
    }
    StepContext ctx = contextFor(who, at);
    const BrainTime lasts = lastsOf(ctx, a);
    a.stateDue = lasts == kNever ? kNever : at + lasts;
    noteEntered(who, target, at);
    const EventId raised = row.fns.enter ? row.fns.enter(ctx, a) : kNoId;
    if (raised != kNoId) {
        answer(who, raised, 0, at, depth + 1);
        return true;
    }
    // A NEW INTENT THAT AN EARLIER STATE OF THIS TRIP DID NOT ACT ON (the plan no longer leads where the person wants to
    // be) is delivered again to the state just entered, whose row says what happens (the top of brain.h).
    const AgentBrain& now = agents_[who];
    if (now.state == target && now.travelling() && now.legEnd[now.legCount - 1] != now.intentTarget) {
        const Verb verb = table_.on[target][table_.intentEvent].verb;
        if (verb != Verb::Ignore && verb != Verb::Refuse) answer(who, table_.intentEvent, 0, at, depth + 1);
    }
    return true;
}

void Machine::beginLeg(AgentId who, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    const WayId way = a.legWay(a.legNow);
    if (way >= table_.ways.size()) {
        fail(who, "is a way the table does not have", "leg");
        enterState(who, table_.noPlan, at, depth);
        return;
    }
    const WayRow& row = table_.ways[way];
    if (enterState(who, row.begins, at, depth)) return;
    // the way begins in a state that is not built: it cannot be travelled, so it is ruled out and the trip planned again
    fail(who, "is not built yet, so its way is ruled out", table_.states[row.begins].name.c_str());
    a.ruledOut = static_cast<uint16_t>(a.ruledOut | (1u << way));
    replan(who, false, false, at, depth + 1);
}

void Machine::arrive(AgentId who, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    PlaceRef last;
    if (a.legCount > 0) last.id = a.legEnd[a.legCount - 1];
    a.legNow = a.legCount;
    Vec2 p;
    if (last.valid() && world_.positionOf(last, p)) {
        a.originX = toCentimetres(p.x);
        a.originZ = toCentimetres(p.y);
    }
    // the intent changed on the way (in a state that does not act on a new intent): go on from here to the new place
    if (last.valid() && last.id != a.intentTarget) {
        replan(who, false, false, at, depth + 1);
        return;
    }
    ++counts_.arrivals;
    const StateId there = table_.intents[a.intentKind].arrive;
    if (enterState(who, there, at, depth)) return;
    fail(who, "is not built yet (an intent arrives in it)", table_.states[there].name.c_str());
    enterState(who, table_.noPlan, at, depth);
}

// Plan again from where the person stands. With `leave` the state they are in is left (its skill stopped); without,
// the caller has already left it.
void Machine::replan(AgentId who, bool ruleOutThisWay, bool leave, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    if (depth > kMaxChain) {
        fail(who, "planned again too many times at one instant", table_.states[a.state].name.c_str());
        return;
    }
    if (ruleOutThisWay) a.ruledOut = static_cast<uint16_t>(a.ruledOut | (1u << a.legWay(a.legNow)));
    // where they stand is asked BEFORE the state is left: leaving a car changes what "their body" means
    const Vec2 from = whereIs(who, at);
    if (leave) leaveState(who, ExitWhy::Abandoned, at);
    PlanAsk ask;
    ask.agent = who;
    ask.from = from;
    ask.to.id = a.intentTarget;
    ask.traits = a.traits;
    ask.ruledOut = a.ruledOut;
    ask.car.id = a.car;
    a.originX = toCentimetres(from.x);
    a.originZ = toCentimetres(from.y);
    a.legCount = 0;
    a.legNow = 0;
    a.heldState = kNoId;
    // the space the dropped plan held for its car is let go before the new plan is priced, so it can be chosen again
    world_.dropHold(who);
    PlanLegs legs;
    if (!planner_.plan(ask, legs) || legs.count > kMaxLegs) {
        ++counts_.noPlans;
        enterState(who, table_.noPlan, at, depth);
        return;
    }
    ++counts_.plans;
    takePlan(who, legs);
    if (a.legCount == 0) arrive(who, at, depth);
    else beginLeg(who, at, depth);
}

void Machine::takePlan(AgentId who, const PlanLegs& legs) {
    AgentBrain& a = agents_[who];
    for (int i = 0; i < kMaxLegs; ++i) {
        a.legEnd[i] = i < legs.count ? legs.end[i] : 0;
        a.legUse[i] = i < legs.count ? legs.use[i] : 0;
        a.legSeconds[i] = i < legs.count ? legs.seconds[i] : 0;
    }
    a.legCount = legs.count;
    a.legNow = 0;
    // A LEG THAT USES A VEHICLE ENDS WHERE THE VEHICLE IS LEFT: that space is held for this person from now until
    // they park in it or drop the plan (ADR-0441), so no other plan is told it meanwhile. (The last such leg: a person
    // holds one space.)
    for (int i = 0; i < a.legCount; ++i)
        if (a.legHandle(i) != kNoHandle) {
            PlaceRef space;
            space.id = a.legEnd[i];
            world_.holdSpace(space, who);
        }
}

// STAY IN THIS STATE AND THIS VEHICLE, AND GO SOMEWHERE ELSE IN IT (the verb `redirect`, ADR-0442). The plan is made
// again with the person at the wheel: its first leg must be the leg under way (the same way, the same vehicle) with a
// new end. Then the old plan's space is let go and the new one held, the state begins its time afresh, and its
// behaviour tells its skill where the leg now ends. When there is no such plan (no free space near the new place, no
// vehicle on this leg) nothing changes: the old trip carries on, and the new intent is taken up when it ends.
void Machine::redirect(AgentId who, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    const StateRow& row = table_.states[a.state];
    if (!a.travelling() || !row.fns.retarget) {
        ++counts_.refused;
        return;
    }
    const uint32_t use = a.legUse[a.legNow];
    PlanLegs legs;
    bool planned = false;
    const Vec2 from = whereIs(who, at);
    if (a.legHandle(a.legNow) != kNoHandle) {
        PlanAsk ask;
        ask.agent = who;
        ask.from = from;
        ask.to.id = a.intentTarget;
        ask.traits = a.traits;
        ask.ruledOut = a.ruledOut;
        ask.car.id = a.legHandle(a.legNow);
        ask.atTheWheel = true;
        // (the space the old plan holds is let go before the new one is priced, so it can be chosen again)
        world_.dropHold(who);
        planned = planner_.plan(ask, legs) && legs.count > 0 && legs.count <= kMaxLegs && legs.use[0] == use;
    }
    if (!planned) {
        ++counts_.redirectsLeft;
        // the old plan stands, and so does its hold
        for (int i = a.legNow; i < a.legCount; ++i)
            if (a.legHandle(i) != kNoHandle) {
                PlaceRef space;
                space.id = a.legEnd[i];
                world_.holdSpace(space, who);
            }
        return;
    }
    ++counts_.plans;
    ++counts_.redirects;
    a.originX = toCentimetres(from.x);
    a.originZ = toCentimetres(from.y);
    takePlan(who, legs);
    // the state's planned time runs from here, to the new end
    a.stateEntered = at;
    StepContext ctx = contextFor(who, at);
    const BrainTime lasts = lastsOf(ctx, a);
    a.stateDue = lasts == kNever ? kNever : at + lasts;
    noteEntered(who, a.state, at);
    const EventId raised = row.fns.retarget(ctx, a);
    if (raised != kNoId) answer(who, raised, 0, at, depth + 1);
}

// Leave a state that holds and carry on with what was interrupted, from where it had got to.
void Machine::resume(AgentId who, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    const StateId back = a.heldState;
    if (back == kNoId) {
        fail(who, "has nothing to carry on with", table_.states[a.state].name.c_str());
        return;
    }
    const BrainTime remaining = a.payload.remaining;
    leaveState(who, ExitWhy::Done, at);
    a.state = back;
    a.heldState = kNoId;
    a.payload.word = 0;
    StepContext ctx = contextFor(who, at);
    const BrainTime lasts = lastsOf(ctx, a);
    Real fraction = 0;
    if (lasts == kNever || remaining == kNever) {
        a.stateEntered = at;
        a.stateDue = kNever;
    } else {
        const BrainTime left = std::min(remaining, lasts);
        const BrainTime gone = lasts - left;
        a.stateEntered = at >= gone ? at - gone : 0;
        a.stateDue = at + left;
        fraction = lasts > 0 ? static_cast<Real>(gone) / static_cast<Real>(lasts) : Real(1);
    }
    noteEntered(who, back, at);
    const StateFns& fns = table_.states[back].fns;
    const EventId raised = ctx.skillsRun && fns.enterMidLeg ? fns.enterMidLeg(ctx, a, fraction) : kNoId;
    if (raised != kNoId) answer(who, raised, 0, at, depth + 1);
}

void Machine::answer(AgentId who, EventId kind, uint32_t payload, BrainTime at, int depth) {
    AgentBrain& a = agents_[who];
    const StateRow& row = table_.states[a.state];
    if (depth > kMaxChain) {
        fail(who, "set off too many transitions at one instant", row.name.c_str());
        return;
    }
    if (kind >= table_.events.size()) {
        fail(who, "was sent an event the table does not have", row.name.c_str());
        return;
    }
    ++counts_.events;
    if (row.fns.onEvent) {
        StepContext ctx = contextFor(who, at);
        BrainEvent e;
        e.agent = who;
        e.kind = kind;
        e.payload = payload;
        row.fns.onEvent(ctx, a, e);
    }
    const Answer ans = table_.on[a.state][kind];
    switch (ans.verb) {
        case Verb::Refuse:
            ++counts_.refused;
            return;
        case Verb::Ignore:
            return;
        case Verb::Go: {
            const StateRow& target = table_.states[ans.state];
            if (!target.fns.built) {
                fail(who, "is not built yet", target.name.c_str());
                return;
            }
            if (target.holds && !row.holds) {
                // what is interrupted, and how much of its planned time was left
                const StateId interrupted = a.state;
                const BrainTime left = a.stateDue == kNever ? kNever : (a.stateDue > at ? a.stateDue - at : 0);
                leaveState(who, ExitWhy::Held, at);
                a.heldState = interrupted;
                a.payload.remaining = left;
            } else {
                // (from one holding state to another, what was interrupted stays as it was noted)
                leaveState(who, ExitWhy::Done, at);
            }
            enterState(who, ans.state, at, depth);
            return;
        }
        case Verb::NextLeg:
            if (!a.travelling()) {
                ++counts_.refused;
                return;
            }
            leaveState(who, ExitWhy::Done, at);
            a.legNow = static_cast<uint8_t>(a.legNow + 1);
            if (a.travelling()) beginLeg(who, at, depth);
            else arrive(who, at, depth);
            return;
        case Verb::Replan:
            replan(who, false, true, at, depth);
            return;
        case Verb::ReplanWithout:
            // with no leg under way there is no way to rule out: the event should not have come
            if (!a.travelling()) {
                ++counts_.refused;
                return;
            }
            replan(who, true, true, at, depth);
            return;
        case Verb::Resume:
            resume(who, at, depth);
            return;
        case Verb::Redirect:
            redirect(who, at, depth);
            return;
    }
}

// The clock: while the person's state is one the clock ends (for a far person, one a skill would end too) and its
// time is up, raise the state's clock event AT that time.
void Machine::runClock(AgentId who, bool nearNow) {
    const BrainTime now = world_.now();
    for (int i = 0; i < kMaxChain; ++i) {
        AgentBrain& a = agents_[who];
        const StateRow& row = table_.states[a.state];
        const bool byClock = row.ends == Ends::Clock || (row.ends == Ends::Skill && !nearNow);
        if (!byClock || a.stateDue == kNever || a.stateDue > now) return;
        const StateId stateBefore = a.state;
        const BrainTime dueBefore = a.stateDue, enteredBefore = a.stateEntered;
        answer(who, row.clock, 0, a.stateDue, 0);
        if (a.state == stateBefore && a.stateDue == dueBefore && a.stateEntered == enteredBefore) {
            // the table did nothing with the state's own clock event: stop the clock so it is not raised for ever
            fail(who, "did not answer its own clock event", table_.states[stateBefore].name.c_str());
            a.stateDue = kNever;
            return;
        }
    }
}

void Machine::step(AgentId who, Detail level) {
    AgentBrain& a = agents_[who];
    const BrainTime now = world_.now();
    const bool wasNear = static_cast<Detail>(a.detail) != Detail::Far;
    const bool nearNow = level != Detail::Far;   // Middle is not built: it runs as Near
    if (nearNow && !wasNear) {
        // RAISED: brought up to date by the clock as a far person, then the state is taken up part-way
        runClock(who, false);
        agents_[who].detail = static_cast<uint8_t>(level);
        AgentBrain& r = agents_[who];
        StepContext ctx = contextFor(who, now);
        const StateFns& fns = table_.states[r.state].fns;
        if (fns.enterMidLeg) {
            const EventId raised = fns.enterMidLeg(ctx, r, fractionDone(ctx, r));
            if (raised != kNoId) answer(who, raised, 0, now, 0);
        }
    } else if (!nearNow && wasNear) {
        // LOWERED: the skill stops; the state carries on by the clock, from its planned end (or from now, when the
        // skill was running late)
        leaveState(who, ExitWhy::Lowered, now);
        a.detail = static_cast<uint8_t>(level);
        if (table_.states[a.state].ends == Ends::Skill && a.stateDue != kNever && a.stateDue < now) a.stateDue = now;
    }
    agents_[who].detail = static_cast<uint8_t>(level);
    const StateFns& fns = table_.states[agents_[who].state].fns;
    if (nearNow && fns.step) {
        StepContext ctx = contextFor(who, now);
        fns.step(ctx, agents_[who]);
    }
    runClock(who, nearNow);
}

void Machine::tick() {
    const BrainTime now = world_.now();
    draining_.swap(incoming_);
    std::sort(draining_.begin(), draining_.end(), [](const BrainEvent& x, const BrainEvent& y) {
        return x.agent != y.agent ? x.agent < y.agent : x.order < y.order;
    });
    for (const BrainEvent& e : draining_)
        if (e.agent < agents_.size()) answer(e.agent, e.kind, e.payload, now, 0);
    draining_.clear();
    for (std::size_t i = 0; i < agents_.size(); ++i)
        step(static_cast<AgentId>(i), static_cast<Detail>(agents_[i].detail));
}

BrainTime Machine::nextAttention(AgentId who) const {
    const AgentBrain& a = agents_[who];
    const StateRow& row = table_.states[a.state];
    const bool nearNow = static_cast<Detail>(a.detail) != Detail::Far;
    const bool byClock = row.ends == Ends::Clock || (row.ends == Ends::Skill && !nearNow);
    return byClock ? a.stateDue : kNever;
}

std::string Machine::describe(AgentId who) const {
    const AgentBrain& a = agents_[who];
    const StateRow& row = table_.states[a.state];
    std::string s = "agent " + std::to_string(who) + ": wants " + table_.intents[a.intentKind].name + " place " +
                    std::to_string(a.intentTarget) + " since " + secondsText(a.intentBegan) + " | plan: ";
    if (a.legCount == 0) s += "none";
    for (int i = 0; i < a.legCount; ++i) {
        const WayId way = a.legWay(i);
        s += (i ? ", " : "") + (way < table_.ways.size() ? table_.ways[way].name : std::string("?")) + " to " +
             std::to_string(a.legEnd[i]);
        if (a.legHandle(i) != kNoHandle) s += " (vehicle " + std::to_string(a.legHandle(i)) + ")";
    }
    if (a.travelling()) s += " | leg " + std::to_string(a.legNow + 1) + " of " + std::to_string(a.legCount);
    else if (a.legCount > 0) s += " | plan done";
    s += " | " + row.name + " since " + secondsText(a.stateEntered);
    if (a.stateDue != kNever) s += ", due " + secondsText(a.stateDue);
    const bool nearNow = static_cast<Detail>(a.detail) != Detail::Far;
    if (row.ends == Ends::Skill && nearNow)
        s += ", waiting for " + table_.events[row.clock] + " from the " + row.skill + " skill";
    else if (row.ends != Ends::Never)
        s += ", waiting for the clock (" + table_.events[row.clock] + ")";
    else
        s += ", waiting for an event";
    if (row.holds && a.heldState != kNoId) {
        s += ", holding " + table_.states[a.heldState].name;
        if (a.payload.remaining != kNever) s += " with " + secondsText(a.payload.remaining) + " left";
    }
    s += nearNow ? " | near" : " | far";
    return s;
}

uint64_t Machine::stateHash() const {
    return agents_.empty() ? engine::bundle::kFnvOffset
                           : engine::bundle::fnv1a(agents_.data(), agents_.size() * sizeof(AgentBrain));
}

}  // namespace brain
}  // namespace citysim
