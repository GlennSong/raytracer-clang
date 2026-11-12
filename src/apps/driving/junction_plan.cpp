#include "apps/driving/junction_plan.h"

#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <cstdio>

namespace driving {

namespace {

using engine::LaneGraph;
using engine::PathId;

std::string one(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

// Is any part of the car's body on `p`: its front's path or one of its trail?
bool underBody(const Car& car, PathId p) {
    if (car.path == p) return true;
    for (std::size_t i = 0; i < car.trailCount && i < kMaxTrail; ++i)
        if (car.trail[i] == p) return true;
    return false;
}

void note(PlanUpdate& out, uint64_t tick, double seconds, PlanChange change, const JunctionPlan& plan) {
    if (out.noteCount >= out.notes.size()) return;   // cannot happen: at most every old plan dropped and as many made
    PlanNote& n = out.notes[out.noteCount++];
    n.tick = tick;
    n.seconds = seconds;
    n.change = change;
    n.junction = plan.junction;
    n.connector = plan.connector;
}

}  // namespace

bool operator==(const GiveWayJudgement& a, const GiveWayJudgement& b) {
    return a.cause == b.cause && a.other == b.other && a.crossing == b.crossing && a.otherPath == b.otherPath &&
           a.need == b.need && a.have == b.have && a.by == b.by && a.stopAlong == b.stopAlong;
}

bool operator==(const JunctionPlan& a, const JunctionPlan& b) {
    if (a.junction != b.junction || a.connector != b.connector || a.turn != b.turn ||
        a.startOdometer != b.startOdometer || a.madeAtTick != b.madeAtTick || a.crossingCount != b.crossingCount ||
        a.waitingCount != b.waitingCount)
        return false;
    for (std::size_t i = 0; i < a.waitingCount && i < kMaxWaitingPoints; ++i)
        if (a.waitingAt[i] != b.waitingAt[i]) return false;
    return a.arrivalFixed == b.arrivalFixed && a.arrivalTick == b.arrivalTick && a.arrivalAt == b.arrivalAt &&
           a.cameToRest == b.cameToRest && a.restAt == b.restAt && a.committed == b.committed &&
           a.committedTick == b.committedTick && a.decision == b.decision && a.mindChanges == b.mindChanges &&
           a.judged == b.judged;
}

const char* giveWayDecisionWords(GiveWayDecision d) {
    switch (d) {
    case GiveWayDecision::Undecided: return "undecided";
    case GiveWayDecision::Go: return "go";
    case GiveWayDecision::GiveWay: return "give way";
    }
    return "";
}

bool operator==(const PlanNote& a, const PlanNote& b) {
    return a.tick == b.tick && a.seconds == b.seconds && a.change == b.change && a.junction == b.junction &&
           a.connector == b.connector;
}

const JunctionPlan* JunctionPlans::find(PathId connector) const {
    for (std::size_t i = 0; i < count && i < kMaxJunctionPlans; ++i)
        if (plans[i].connector == connector) return &plans[i];
    return nullptr;
}

const char* planChangeWords(PlanChange c) {
    switch (c) {
    case PlanChange::Made: return "made";
    case PlanChange::Cleared: return "dropped: through it";
    case PlanChange::WayChanged: return "dropped: its way no longer goes through it";
    case PlanChange::RouteChanged: return "dropped: the trip was routed again";
    }
    return "";
}

const char* turnWords(engine::Turn t) {
    switch (t) {
    case engine::Turn::Straight: return "straight on";
    case engine::Turn::NearTurn: return "near turn";
    case engine::Turn::FarTurn: return "far turn";
    case engine::Turn::UTurn: return "U-turn";
    case engine::Turn::None: break;
    }
    return "no turn";
}

bool findWaitingPoints(const LaneGraph& roads, const RightOfWay& row, PathId connector, Real carLength,
                       JunctionPlan& plan) {
    plan.waitingAt.fill(0);
    plan.waitingAt[0] = 0;   // the line
    plan.waitingCount = 1;
    if (!connector.valid() || connector.index >= roads.paths.size() || connector.index >= row.paths.size()) return true;
    const engine::Range gaps = row.paths[connector.index].gaps;
    bool fitted = true;
    for (uint32_t g = gaps.first; g < gaps.end() && g < row.gaps.size(); ++g) {
        const ClearGap& gap = row.gaps[g];
        if (gap.to - gap.from < carLength) continue;   // the body does not fit between the two stretches
        if (plan.waitingCount >= kMaxWaitingPoints) {
            fitted = false;
            break;
        }
        plan.waitingAt[plan.waitingCount++] = gap.to;
    }
    return fitted;
}

void updateJunctionPlans(const LaneGraph& roads, const RightOfWay& row, const Car& car, Real carLength,
                         const WayAhead& way, uint64_t tick, double seconds, JunctionPlans& plans, PlanUpdate& out) {
    const JunctionPlans old = plans;
    std::array<uint8_t, kMaxJunctionPlans> kept{};
    plans.count = 0;

    // First the plans for connectors the car's body is still on but its way no longer runs along (it is leaving them).
    const bool wayIsMine = !way.paths.empty() && way.paths[0] == car.path;
    const std::size_t wayCount = wayIsMine ? way.paths.size() : 1;
    const auto onWay = [&](PathId p) {
        for (std::size_t k = 0; k < wayCount; ++k)
            if ((wayIsMine ? way.paths[k] : car.path) == p) return true;
        return false;
    };
    for (std::size_t i = 0; i < old.count; ++i)
        if (!onWay(old.plans[i].connector) && underBody(car, old.plans[i].connector)) {
            plans.plans[plans.count++] = old.plans[i];
            kept[i] = 1;
        }

    // Then each connector along the way, in order: its plan kept, or a new one.
    double startAt = -static_cast<double>(car.along);   // where path k starts, metres ahead of the front
    for (std::size_t k = 0; k < wayCount; ++k) {
        const PathId p = wayIsMine ? way.paths[k] : car.path;
        if (!p.valid() || p.index >= roads.paths.size()) break;
        const engine::Path& path = roads.paths[p.index];
        if (path.kind == engine::PathKind::Connector && path.junction.valid()) {
            std::size_t match = old.count;
            for (std::size_t i = 0; i < old.count; ++i)
                if (!kept[i] && old.plans[i].connector == p) {
                    match = i;
                    break;
                }
            if (plans.count >= kMaxJunctionPlans) {
                out.plansCut = true;
                break;
            }
            if (match < old.count) {
                plans.plans[plans.count++] = old.plans[match];
                kept[match] = 1;
            } else {
                JunctionPlan plan;
                plan.junction = path.junction;
                plan.connector = p;
                plan.turn = path.turn;
                plan.startOdometer = car.odometer + startAt;
                plan.madeAtTick = tick;
                plan.crossingCount = path.crossings.count;
                if (!findWaitingPoints(roads, row, p, carLength, plan)) out.waitingPointsCut = true;
                plans.plans[plans.count++] = plan;
                ++out.made;
                note(out, tick, seconds, PlanChange::Made, plan);
            }
        }
        startAt += path.length;
    }

    // The old plans not kept are dropped: through them, or turned away from them before reaching them.
    for (std::size_t i = 0; i < old.count; ++i) {
        if (kept[i]) continue;
        ++out.dropped;
        const PlanChange why = old.plans[i].startOdometer <= car.odometer ? PlanChange::Cleared : PlanChange::WayChanged;
        note(out, tick, seconds, why, old.plans[i]);
    }
    for (std::size_t i = plans.count; i < kMaxJunctionPlans; ++i) plans.plans[i] = JunctionPlan();
}

void dropJunctionPlans(uint64_t tick, double seconds, PlanChange why, JunctionPlans& plans, PlanUpdate& out) {
    for (std::size_t i = 0; i < plans.count; ++i) {
        ++out.dropped;
        note(out, tick, seconds, why, plans.plans[i]);
        plans.plans[i] = JunctionPlan();
    }
    plans.count = 0;
}

uint64_t junctionPlansHash(const JunctionPlans& plans) {
    engine::bundle::Fnv h;
    h.pod(plans.count);
    for (std::size_t i = 0; i < plans.count; ++i) {
        const JunctionPlan& p = plans.plans[i];
        h.pod(p.junction.index);
        h.pod(p.connector.index);
        h.pod(static_cast<uint8_t>(p.turn));
        h.pod(p.startOdometer);
        h.pod(p.madeAtTick);
        h.pod(p.crossingCount);
        h.pod(p.waitingCount);
        for (std::size_t k = 0; k < p.waitingCount; ++k) h.pod(p.waitingAt[k]);
        h.pod(static_cast<uint8_t>(p.arrivalFixed));
        h.pod(p.arrivalTick);
        h.pod(p.arrivalAt);
        h.pod(static_cast<uint8_t>(p.cameToRest));
        h.pod(p.restAt);
        h.pod(static_cast<uint8_t>(p.committed));
        h.pod(p.committedTick);
        h.pod(static_cast<uint8_t>(p.decision));
        h.pod(p.mindChanges);
        h.pod(static_cast<uint8_t>(p.judged.cause));
        h.pod(p.judged.other.index);
        h.pod(p.judged.crossing.index);
        h.pod(p.judged.otherPath.index);
        h.pod(p.judged.need);
        h.pod(p.judged.have);
        h.pod(static_cast<uint8_t>(p.judged.by));
        h.pod(p.judged.stopAlong);
    }
    return h.h;
}

std::string describeGiveWay(const LaneGraph& roads, const RightOfWay& row, const JunctionPlan& p, double now) {
    std::string out;
    const GiveWayJudgement& j = p.judged;
    const std::string other = j.otherPath.valid() ? pathTurnWords(roads, row, j.otherPath) : std::string("a path");
    switch (p.decision) {
    case GiveWayDecision::Undecided: out = "not decided yet (not near enough to judge)"; break;
    case GiveWayDecision::Go:
        out = "go";
        if (j.cause == GiveWayCause::Gap)
            out += " (the tightest gap: car " + std::to_string(j.other.index) + " on " + other + ", at crossing point " +
                   std::to_string(j.crossing.index) + ", " + one(j.have) + " s; I need " + one(j.need) + " s)";
        break;
    case GiveWayDecision::GiveWay:
        out = "giving way";
        if (j.cause == GiveWayCause::Gap) {
            out += " to car " + std::to_string(j.other.index) + " on " + other + ", at crossing point " +
                   std::to_string(j.crossing.index) + " (" + questionWords(j.by) + "): a gap of " + one(j.have) +
                   " s; I need " + one(j.need) + " s";
            out += j.stopAlong > 0 ? ", waiting inside the junction " + one(j.stopAlong) + " m along" : ", at the line";
        } else if (j.cause == GiveWayCause::NextJunction)
            out += ": I give way at the next junction and my body would not fit between the two";
        break;
    }
    if (p.mindChanges > 0) out += "; changed its mind " + std::to_string(p.mindChanges) + (p.mindChanges == 1 ? " time" : " times");
    if (p.arrivalFixed) out += "; its arrival at the line, for who goes first: t=" + one(p.arrivalAt) + " s";
    if (p.cameToRest) {
        out += ", came to rest there at t=" + one(p.restAt) + " s";
        if (now >= p.restAt) out += " (" + one(now - p.restAt) + " s ago)";
    }
    if (p.committed) out += "; committed (past its last waiting point)";
    return out;
}

std::string describeJunctionPlans(const LaneGraph& roads, const RightOfWay& row, const Car& car,
                                  const JunctionPlans& plans, double now) {
    if (plans.count == 0) return "junctions ahead: none in view\n";
    std::string out = "junctions ahead (" + std::to_string(plans.count) + " planned, nearest first):\n";
    for (std::size_t i = 0; i < plans.count; ++i) {
        const JunctionPlan& p = plans.plans[i];
        const double ahead = p.startOdometer - car.odometer;
        out += "  junction " + std::to_string(p.junction.index) + ": " + turnWords(p.turn) + " by connector " +
               std::to_string(p.connector.index) + ", ";
        out += ahead >= 0 ? "its start " + one(ahead) + " m ahead" : "its start " + one(-ahead) + " m behind my front";
        out += " (planned at tick " + std::to_string(p.madeAtTick) + ")\n";
        out += "    decided: " + describeGiveWay(roads, row, p, now) + "\n";
        if (!p.connector.valid() || p.connector.index >= roads.paths.size()) continue;
        const engine::Path& path = roads.paths[p.connector.index];
        const uint32_t phases =
            p.junction.valid() && p.junction.index < row.junctions.size() ? row.junctions[p.junction.index].phaseCount() : 1;
        out += "    crossing points (" + std::to_string(path.crossings.count) + ", in order along it" +
               (phases > 1 ? "; who gives way at phase 0 of " + std::to_string(phases) + ", the lights' clock is task D2"
                           : std::string()) +
               "):\n";
        for (uint32_t k = path.crossings.first; k < path.crossings.end() && k < roads.pathCrossings.size(); ++k) {
            const engine::CrossingId cid = roads.pathCrossings[k];
            const engine::CrossingPoint& rec = roads.crossings[cid.index];
            const bool iAmA = rec.a == p.connector;
            const Real from = iAmA ? rec.aFrom : rec.otherFrom, to = iAmA ? rec.aTo : rec.otherTo;
            out += "      " + std::to_string(k - path.crossings.first + 1) + ". ";
            if (rec.otherSide == engine::OtherSide::Crosswalk) out += "crosswalk " + std::to_string(rec.other);
            else if (rec.otherSide == engine::OtherSide::Track) out += "rail track " + std::to_string(rec.other);
            else {
                const PathId other = iAmA ? rec.otherPath() : rec.a;
                const char* kind = rec.meeting == engine::Meeting::Cross  ? "cross"
                                   : rec.meeting == engine::Meeting::Join ? "join"
                                                                          : "split";
                out += std::string(kind) + " with " + pathTurnWords(roads, row, other);
            }
            out += ", " + one(from) + "-" + one(to) + " m along, " + one(ahead + from) + " m ahead: ";
            const MeetingAnswer* a = row.answer(cid, 0);
            if (!a) {
                out += "whoever is on it announces (people and trains are task E5)\n";
                continue;
            }
            switch (myTurn(roads, *a, cid, p.connector)) {
            case MyTurn::GiveWay: out += std::string("I give way, ") + settledByWords(a->by); break;
            case MyTurn::GoFirst: out += std::string("it gives way to me, ") + settledByWords(a->by); break;
            case MyTurn::Neither: out += std::string("nobody gives way, ") + settledByWords(a->by); break;
            case MyTurn::Ask: {
                const bool iGive = (a->fallback == GivesWay::A) == iAmA;
                out += std::string("asked while driving (") + settledByWords(a->by) + "); if that ties, " +
                       (iGive ? "I give way, " : "it gives way to me, ") + settledByWords(a->fallbackBy);
                break;
            }
            }
            out += "\n";
        }
        out += "    waiting points:";
        for (std::size_t w = 0; w < p.waitingCount; ++w)
            out += (w == 0 ? " the line (" : ", ") + one(p.waitingAt[w]) + (w == 0 ? " m)" : " m along");
        out += "; committed once past " + one(p.lastWaitingPoint()) + " m along it\n";
    }
    return out;
}

}  // namespace driving
