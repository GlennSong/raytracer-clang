#include "apps/driving/trace.h"

#include "apps/driving/driving.h"
#include "apps/driving/notice.h"   // kStandingBelow
#include "engine/bundle/bundle.h"  // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <cstdio>
#include <vector>

namespace driving {

namespace {

// The card calls a car speeding up or slowing only when its acceleration is beyond this (m/s2); within it, steady.
constexpr Real kSteadyWithin = 0.05;

bool sameValue(const ReadValue& a, const ReadValue& b) {
    return a.present == b.present && a.number == b.number && a.yes == b.yes && a.word == b.word;
}

bool sameReason(const TracedReason& a, const TracedReason& b) {
    if (a.rule != b.rule || a.kept != b.kept || a.read != b.read) return false;
    for (std::size_t i = 0; i < a.kept && i < kMaxTracedValues; ++i)
        if (!sameValue(a.values[i], b.values[i])) return false;
    return true;
}

void foldReason(engine::bundle::Fnv& h, const TracedReason& r) {
    h.pod(r.rule);
    h.pod(r.kept);
    h.pod(r.read);
    for (std::size_t i = 0; i < r.kept && i < kMaxTracedValues; ++i) {
        h.pod(static_cast<uint8_t>(r.values[i].present));
        h.pod(r.values[i].number);
        h.pod(static_cast<uint8_t>(r.values[i].yes));
        h.pod(r.values[i].word);
    }
}

// A number with one decimal.
std::string one(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}
// A number with none.
std::string whole(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.0f", v);
    return b;
}

std::string carWords(CarId c) { return "car " + std::to_string(c.index); }

// "lane 93" or "connector 12", as the road calls the path.
std::string pathWords(const engine::LaneGraph* roads, engine::PathId p) {
    if (!roads || !p.valid() || p.index >= roads->paths.size()) return "path " + std::to_string(p.index);
    return (roads->paths[p.index].kind == engine::PathKind::Lane ? "lane " : "connector ") + std::to_string(p.index);
}

std::string placeWords(const engine::LaneGraph* roads, engine::PathId p, Real along) {
    return pathWords(roads, p) + " at " + one(along) + " m";
}

// A rule's name in quotes, or its number when there are no names.
std::string ruleName(const Driving& d, RuleIndex rule) {
    if (d.hasRules() && rule < d.rules().labels.rules.size()) return "\"" + d.rules().labels.rules[rule] + "\"";
    return "rule " + std::to_string(rule);
}

// Why, in brackets: the kept reason explained, else the rule's name.
std::string whyWords(const Driving& d, const TracedReason& why, RuleIndex rule) {
    if (why.rule == rule && rule != kNoRule && d.hasRules()) {
        const std::string line = explainKept(d.rules(), why);
        if (!line.empty()) return " (" + line + ")";
    }
    return rule == kNoRule ? std::string() : " (" + ruleName(d, rule) + ")";
}

// One recent decision, in words.
std::string entryWords(const Driving& d, const TraceEntry& e) {
    std::string what;
    switch (e.binding.by) {
    case Limit::Nothing: what = "free road"; break;
    case Limit::CarAhead: what = "following " + carWords(e.binding.car); break;
    case Limit::Stop:
        what = "a stop" + whyWords(d, e.why, e.binding.rule);
        if (e.binding.stopFor == StopFor::GivingWay && e.binding.car.valid())
            what += ", giving way to " + carWords(e.binding.car);
        if (e.binding.stopFor == StopFor::NoRoom)
            what += e.binding.car.valid() ? ", waiting for room beyond behind " + carWords(e.binding.car)
                                          : std::string(", waiting for room beyond");
        break;
    case Limit::TopSpeed:
        what = e.binding.rule == kNoRule ? std::string("its vehicle class's top speed")
                                         : "a top speed" + whyWords(d, e.why, e.binding.rule);
        break;
    case Limit::WayEnd: what = "the end of its way"; break;
    }
    return what + "; " + one(e.speed) + " m/s on " + placeWords(d.roads(), e.path, e.along);
}

}  // namespace

// ---- The reasons kept ------------------------------------------------------------------------------------------------

TracedReason keepReason(const Decision& decision, const Reason& why) {
    TracedReason r;
    r.rule = why.rule;
    r.read = why.valueCount;
    for (uint32_t i = 0; i < why.valueCount && r.kept < kMaxTracedValues; ++i) {
        const uint32_t v = why.firstValue + i;
        if (v >= decision.values.size()) break;
        r.values[r.kept++] = decision.values[v];
    }
    return r;
}

std::string explainKept(const CompiledRules& rules, const TracedReason& why) {
    if (why.rule == kNoRule) return std::string();
    if (why.kept == 0 && why.read > 0) {
        if (why.rule < rules.labels.rules.size()) return "\"" + rules.labels.rules[why.rule] + "\"";
        return "rule " + std::to_string(why.rule);
    }
    // explain() reads its values out of a Decision: lend it one holding just these.
    Decision lent;
    lent.values.assign(why.values.begin(), why.values.begin() + why.kept);
    Reason r;
    r.rule = why.rule;
    r.firstValue = 0;
    r.valueCount = why.kept;
    std::string line = explain(rules, lent, r);
    if (why.read > why.kept) line += "; and " + std::to_string(why.read - why.kept) + " more not kept";
    return line;
}

// ---- The ring ---------------------------------------------------------------------------------------------------------

bool operator==(const TraceEntry& a, const TraceEntry& b) {
    return a.tick == b.tick && a.seconds == b.seconds && a.binding == b.binding && a.speed == b.speed &&
           a.path == b.path && a.along == b.along && sameReason(a.why, b.why);
}

void CarTrace::add(const TraceEntry& e) {
    ring[written % kTraceLength] = e;
    ++written;
}

std::size_t CarTrace::size() const { return std::min<std::size_t>(written, kTraceLength); }

const TraceEntry& CarTrace::oldestFirst(std::size_t i) const {
    const std::size_t first = written > kTraceLength ? written % kTraceLength : 0;
    return ring[(first + i) % kTraceLength];
}

const TraceEntry* CarTrace::latest() const { return written == 0 ? nullptr : &ring[(written - 1) % kTraceLength]; }

void CarTrace::addPlanNote(const PlanNote& n) {
    planNotes[planNotesWritten % kPlanNoteLength] = n;
    ++planNotesWritten;
}

std::size_t CarTrace::planNoteCount() const { return std::min<std::size_t>(planNotesWritten, kPlanNoteLength); }

const PlanNote& CarTrace::planNoteOldestFirst(std::size_t i) const {
    const std::size_t first = planNotesWritten > kPlanNoteLength ? planNotesWritten % kPlanNoteLength : 0;
    return planNotes[(first + i) % kPlanNoteLength];
}

bool operator==(const GiveWayNote& a, const GiveWayNote& b) {
    return a.tick == b.tick && a.seconds == b.seconds && a.junction == b.junction && a.connector == b.connector &&
           a.decision == b.decision && a.judged == b.judged;
}

void CarTrace::addGiveWayNote(const GiveWayNote& n) {
    giveWayNotes[giveWayNotesWritten % kGiveWayNoteLength] = n;
    ++giveWayNotesWritten;
}

std::size_t CarTrace::giveWayNoteCount() const { return std::min<std::size_t>(giveWayNotesWritten, kGiveWayNoteLength); }

const GiveWayNote& CarTrace::giveWayNoteOldestFirst(std::size_t i) const {
    const std::size_t first = giveWayNotesWritten > kGiveWayNoteLength ? giveWayNotesWritten % kGiveWayNoteLength : 0;
    return giveWayNotes[(first + i) % kGiveWayNoteLength];
}

uint64_t traceHash(const CarTrace& trace) {
    engine::bundle::Fnv h;
    h.pod(trace.written);
    for (std::size_t i = 0; i < trace.size(); ++i) {
        const TraceEntry& e = trace.oldestFirst(i);
        h.pod(e.tick);
        h.pod(e.seconds);
        h.pod(static_cast<uint8_t>(e.binding.by));
        h.pod(e.binding.car.index);
        h.pod(e.binding.rule);
        h.pod(e.speed);
        h.pod(e.path.index);
        h.pod(e.along);
        foldReason(h, e.why);
    }
    foldReason(h, trace.heldStop);
    foldReason(h, trace.heldTopSpeed);
    h.pod(trace.planNotesWritten);
    for (std::size_t i = 0; i < trace.planNoteCount(); ++i) {
        const PlanNote& n = trace.planNoteOldestFirst(i);
        h.pod(n.tick);
        h.pod(n.seconds);
        h.pod(static_cast<uint8_t>(n.change));
        h.pod(n.junction.index);
        h.pod(n.connector.index);
    }
    h.pod(trace.giveWayNotesWritten);
    for (std::size_t i = 0; i < trace.giveWayNoteCount(); ++i) {
        const GiveWayNote& n = trace.giveWayNoteOldestFirst(i);
        h.pod(n.tick);
        h.pod(n.seconds);
        h.pod(n.junction.index);
        h.pod(n.connector.index);
        h.pod(static_cast<uint8_t>(n.decision));
        h.pod(static_cast<uint8_t>(n.judged.cause));
        h.pod(n.judged.other.index);
        h.pod(n.judged.crossing.index);
        h.pod(n.judged.need);
        h.pod(n.judged.have);
        h.pod(static_cast<uint8_t>(n.judged.by));
        h.pod(n.judged.stopAlong);
    }
    return h.h;
}

// ---- The driver card --------------------------------------------------------------------------------------------------

std::string describeCar(const Driving& d, CarId id) {
    const Car* c = d.car(id);
    if (!c) return "no such car: " + std::to_string(id.index) + " is not on the road in the new driving model\n";
    const engine::LaneGraph* roads = d.roads();
    const std::string className = c->vehicleClass < d.vehicleClasses().size()
                                      ? "class \"" + d.vehicleClasses().at(c->vehicleClass).name + "\""
                                      : "an unknown class (" + std::to_string(c->vehicleClass) + ")";
    const Trip* t = d.trip(d.tripOf(id));
    if (!c->driven || !t)
        return carWords(id) + ", " + className + ", parked on " + placeWords(roads, c->path, c->along) +
               ": nobody is driving it\n";

    // Who, and where to.
    std::string out = carWords(id) + ", " + className + ", driven by person " + std::to_string(t->person) +
                      ", trip to " + placeWords(roads, t->destination.path, t->destination.along);
    PathPoint front;
    front.path = c->path;
    front.along = c->along;
    const Remaining toGo = roads ? remainingFrom(t->route, *roads, front, t->legReached) : Remaining();
    out += toGo.onRoute ? " (" + whole(toGo.metres) + " m, about " + whole(toGo.seconds) + " s to go)\n"
                        : " (it is off its route)\n";

    // Where, and how it is moving.
    std::string motion = "steady";
    if (c->speed < kStandingBelow && !(c->acceleration > 0)) motion = "at rest";
    else if (c->acceleration > kSteadyWithin) motion = "speeding up";
    else if (c->acceleration < -kSteadyWithin) motion = "slowing";
    out += "on " + placeWords(roads, c->path, c->along) + ", " + one(c->speed) + " m/s, " + motion + "\n";

    // What limits it now. The way from its front on, as the move step reads it.
    std::vector<engine::PathId> after;
    Real toEnd = 0;
    if (roads && c->path.valid() && c->path.index < roads->paths.size()) {
        toEnd = roads->paths[c->path.index].length - c->along;
        const std::vector<engine::PathId>& way = t->way.paths;
        if (t->wayAt < way.size() && way[t->wayAt] == c->path) {
            for (std::size_t k = t->wayAt + 1; k < way.size(); ++k) {
                after.push_back(way[k]);
                toEnd += roads->paths[way[k].index].length;
            }
        }
    }
    const Binding& b = c->binding;
    out += "now: ";
    if (!c->held.decided) {
        out += "not decided yet (its driver decides on its next tick)";
    } else {
        switch (b.by) {
        case Limit::Nothing: out += "free road: nothing holds it back"; break;
        case Limit::CarAhead: {
            out += "following " + carWords(b.car);
            const Neighbour n = roads ? carAhead(*roads, d.cars(), d.pathLists(), id, after, toEnd) : Neighbour();
            const Car* ahead = d.car(b.car);
            if (n.car == b.car) out += ", " + one(n.gap) + " m ahead";
            if (ahead) out += ", doing " + one(ahead->speed) + " m/s";
            break;
        }
        case Limit::Stop:
            out += "a stop " + one(static_cast<Real>(c->held.stopAt - c->odometer)) + " m ahead, by rule " +
                   ruleName(d, b.rule);
            // Giving way: to whom, where, the gap it has and the gap it needs (the plan it gives way at says).
            if (b.stopFor == StopFor::GivingWay && roads)
                for (std::size_t k = 0; k < t->junctions.count; ++k) {
                    const JunctionPlan& p = t->junctions.plans[k];
                    if (p.decision != GiveWayDecision::GiveWay) continue;
                    out += "; " + describeGiveWay(*roads, d.rightOfWay(), p, d.clockSeconds());
                    break;
                }
            // Waiting for room beyond (#264): why it was refused, at the plan it was refused at.
            if (b.stopFor == StopFor::NoRoom)
                for (std::size_t k = 0; k < t->junctions.count; ++k) {
                    const JunctionPlan& p = t->junctions.plans[k];
                    if (p.room.state != RoomState::Refused) continue;
                    out += "; room beyond junction " + std::to_string(p.junction.index) + " " + describeRoom(p);
                    break;
                }
            break;
        case Limit::TopSpeed:
            out += "its top speed of " + one(c->held.topSpeed) + " m/s, " +
                   (b.rule == kNoRule ? std::string("its vehicle class's own") : "by rule " + ruleName(d, b.rule));
            break;
        case Limit::WayEnd: out += "the end of its way, " + one(toEnd) + " m ahead"; break;
        }
    }
    out += "\n";

    // What its driver holds, and why.
    out += "held: ";
    if (!c->held.decided) {
        out += "nothing yet\n";
    } else {
        out += c->held.hasStop ? "stop in " + one(static_cast<Real>(c->held.stopAt - c->odometer)) + " m" +
                                     whyWords(d, t->trace.heldStop, c->held.stopRule)
                               : std::string("no stop");
        out += "; top speed " + one(c->held.topSpeed) + " m/s" +
               (c->held.topSpeedRule == kNoRule ? std::string(" (its vehicle class's own)")
                                                : whyWords(d, t->trace.heldTopSpeed, c->held.topSpeedRule));
        out += "\n";
    }

    // The junctions ahead, as its driver plans for them.
    if (roads) out += describeJunctionPlans(*roads, d.rightOfWay(), *c, t->junctions, d.clockSeconds());

    // What it decided recently, oldest first.
    const CarTrace& trace = t->trace;
    if (trace.written == 0) {
        out += d.settings().trace ? "recent: nothing yet\n" : "recent: not kept (tracing is off)\n";
    } else {
        out += "recent";
        if (trace.written > trace.size())
            out += " (the last " + std::to_string(trace.size()) + " of " + std::to_string(trace.written) + ")";
        out += ":\n";
        for (std::size_t i = 0; i < trace.size(); ++i) {
            const TraceEntry& e = trace.oldestFirst(i);
            out += "  t=" + one(e.seconds) + " s  " + entryWords(d, e) + "\n";
        }
    }
    // Its junction plans made and dropped, oldest first.
    if (trace.planNotesWritten > 0) {
        out += "junction plans";
        if (trace.planNotesWritten > trace.planNoteCount())
            out += " (the last " + std::to_string(trace.planNoteCount()) + " of " +
                   std::to_string(trace.planNotesWritten) + ")";
        out += ":\n";
        for (std::size_t i = 0; i < trace.planNoteCount(); ++i) {
            const PlanNote& n = trace.planNoteOldestFirst(i);
            out += "  t=" + one(n.seconds) + " s  junction " + std::to_string(n.junction.index) + " by connector " +
                   std::to_string(n.connector.index) + ": " + planChangeWords(n.change) + "\n";
        }
    }
    // What it decided at junctions, oldest first: each change of go / give way, or of whom it gives way to.
    if (trace.giveWayNotesWritten > 0) {
        out += "give-way decisions";
        if (trace.giveWayNotesWritten > trace.giveWayNoteCount())
            out += " (the last " + std::to_string(trace.giveWayNoteCount()) + " of " +
                   std::to_string(trace.giveWayNotesWritten) + ")";
        out += ":\n";
        for (std::size_t i = 0; i < trace.giveWayNoteCount(); ++i) {
            const GiveWayNote& n = trace.giveWayNoteOldestFirst(i);
            JunctionPlan shown;
            shown.decision = n.decision;
            shown.judged = n.judged;
            out += "  t=" + one(n.seconds) + " s  junction " + std::to_string(n.junction.index) + " by connector " +
                   std::to_string(n.connector.index) + ": " +
                   (roads ? describeGiveWay(*roads, d.rightOfWay(), shown) : std::string(giveWayDecisionWords(n.decision))) +
                   "\n";
        }
    }
    return out;
}

}  // namespace driving
