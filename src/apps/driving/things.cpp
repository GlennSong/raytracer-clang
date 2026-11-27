#include "apps/driving/things.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace driving {

namespace {

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

FactSpec spec(const char* name, FactKind kind, UnitFamily unit, NoticeFn fn, const char* meaning) {
    FactSpec s;
    s.name = name;
    s.kind = kind;
    s.unit = unit;
    s.notice = fn;
    s.meaning = meaning;
    return s;
}

// (what each fact is about: the thing's path, and how far ahead it is)
void about(const ThingView& t, NoticedFact& out) {
    out.aboutPath = t.thing.path;
    out.hasMetres = true;
    out.metres = t.thing.distance;
}

void noticeThingAhead(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things) return;
    out.present = true;
    out.yes = v.things->thing.found;
    if (out.yes) about(*v.things, out);
}

void noticeThingGap(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    out.present = true;
    out.number = std::max(Real(0), v.things->thing.distance);
    about(*v.things, out);
}

void noticeThingStop(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    const ThingView& t = *v.things;
    Real at = t.thing.distance - t.standOff;
    // A thing just beyond a junction, leaving me no room to stand clear of the box behind it: I stop at the line, where
    // I can still choose another way, not in the box.
    if (t.lineAt > 0 && t.thing.distance > t.lineAt && t.thing.distance - t.boxEndsAt < t.myLength + t.standOff)
        at = std::min(at, t.lineAt);
    out.present = true;
    out.number = std::max(Real(0), at);
    about(t, out);
}

void noticeThingSpeed(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    out.present = true;
    out.number = v.things->thing.speed;
    about(*v.things, out);
}

void noticeThingStood(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    out.present = true;
    out.number = v.things->stood;
    out.hasSeconds = true;
    out.seconds = v.things->stood;
    about(*v.things, out);
}

void noticeWaited(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    out.present = true;
    out.number = v.things->waited;
    out.hasSeconds = true;
    out.seconds = v.things->waited;
    about(*v.things, out);
}

void noticeRoomToPass(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->thing.found) return;
    out.present = true;
    out.yes = v.things->thing.roomToPass;
    about(*v.things, out);
}

// (what a crosswalk fact is about: the path its stretch is on, and how far ahead it begins)
void aboutCrosswalk(const ThingView& t, NoticedFact& out) {
    out.aboutPath = t.crosswalk.path;
    out.hasMetres = true;
    out.metres = t.crosswalk.stopAt;
}

void noticeCrosswalkPerson(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things) return;
    out.present = true;
    out.yes = v.things->crosswalk.found;
    if (out.yes) aboutCrosswalk(*v.things, out);
}

void noticeCrosswalkStop(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.things || !v.things->crosswalk.found) return;
    out.present = true;
    out.number = std::max(Real(0), v.things->crosswalk.stopAt - v.things->crosswalkStandOff);
    aboutCrosswalk(*v.things, out);
}

constexpr const char* kThingReaderName = "the things in a driver's way";

}  // namespace

void CrosswalkPeople::fill(const std::vector<RoadThing>& things, std::size_t crosswalks) {
    on.assign(crosswalks, 0);
    waiting.assign(crosswalks, 0);
    for (const RoadThing& t : things) {
        if (t.crosswalk >= crosswalks) continue;
        (t.onCrosswalk ? on : waiting)[t.crosswalk] = 1;
    }
}

CrosswalkAhead findCrosswalkAhead(const engine::RoadModel& model, engine::PathId path, Real along,
                                  const std::vector<engine::PathId>& pathsAfter, Real reach, Real mySpeed, Real comfortDecel,
                                  const CrosswalkPeople& people) {
    CrosswalkAhead best;
    const engine::LaneGraph& roads = model.laneLevel;
    // (how far I need to stop braking no harder than that, for a person who is only waiting)
    const Real stopping = comfortDecel > 0 ? mySpeed * mySpeed / (2 * comfortDecel) : Real(0);
    Real start = -along;   // where the path begins, from my front
    for (std::size_t k = 0; k <= pathsAfter.size(); ++k) {
        const engine::PathId pid = k == 0 ? path : pathsAfter[k - 1];
        if (!pid.valid() || pid.index >= roads.paths.size()) break;
        if (start > reach) break;
        const engine::Path& P = roads.paths[pid.index];
        for (uint32_t i = P.crossings.first; i < P.crossings.end() && i < roads.pathCrossings.size(); ++i) {
            const engine::CrossingPoint& x = roads.crossings[roads.pathCrossings[i].index];
            if (x.otherSide != engine::OtherSide::Crosswalk || x.a != pid) continue;
            const Real begins = start + x.aFrom;
            if (begins < 0 || begins > reach) continue;   // already over it (committed), or beyond my look
            if (best.found && begins >= best.stopAt) continue;
            const uint32_t id = model.crosswalkOfLane(x.other);
            const engine::ModelCrosswalk* cw = model.crosswalk(id);
            if (!cw || id >= people.on.size()) continue;
            const bool on = people.on[id] != 0;
            const bool waiting = people.waiting[id] != 0 && cw->control != engine::CrosswalkControl::Lights &&
                                 begins >= stopping;
            if (!on && !waiting) continue;
            best.found = true;
            best.crosswalk = id;
            best.stopAt = begins;
            best.path = pid;
            best.someoneOn = on;
        }
        start += P.length;
    }
    return best;
}

OnLine nearestOn(const engine::LaneGraph& roads, engine::PathId path, const engine::Vec2& p) {
    OnLine best;
    best.away = -1;
    const engine::Path& P = roads.paths[path.index];
    for (uint32_t k = P.curve.first; k + 1 < P.curve.end(); ++k) {
        const engine::Vec2 a = roads.points[k], b = roads.points[k + 1];
        const engine::Vec2 ab = b - a;
        const Real len2 = ab.lengthSquared();
        const Real s = len2 > 0 ? std::clamp(((p - a).x * ab.x + (p - a).y * ab.y) / len2, Real(0), Real(1)) : Real(0);
        const engine::Vec2 q = a + ab * s;
        const Real away = (p - q).length();
        if (best.away >= 0 && away >= best.away) continue;
        const Real len = std::sqrt(len2);
        const engine::Vec2 dir = len > 0 ? ab * (1 / len) : engine::Vec2(1, 0);
        best.away = away;
        best.along = roads.along[k] + s * len;
        // (facing +z the driver's left is +x: the left of a heading (x, z) is (z, -x))
        best.side = (p - q).x * dir.y - (p - q).y * dir.x;
    }
    return best;
}

void ThingGrid::build(const std::vector<RoadThing>& things, const engine::LaneGraph& roads) {
    if (roads_ != &roads || pathCount_ != roads.paths.size()) {
        roads_ = &roads;
        pathCount_ = roads.paths.size();
        pathBoxes_.resize(pathCount_);
        for (std::size_t i = 0; i < pathCount_; ++i) {
            const engine::Path& P = roads.paths[i];
            PathBox b;
            for (uint32_t k = P.curve.first; k < P.curve.end(); ++k) {
                const engine::Vec2 q = roads.points[k];
                b.lo = engine::Vec2(std::min(b.lo.x, q.x), std::min(b.lo.y, q.y));
                b.hi = engine::Vec2(std::max(b.hi.x, q.x), std::max(b.hi.y, q.y));
            }
            pathBoxes_[i] = b;
        }
    }
    for (std::pair<const uint64_t, std::vector<uint32_t>>& c : cells_) c.second.clear();
    count_ = things.size();
    widest_ = 0;
    for (uint32_t i = 0; i < things.size(); ++i) {
        const RoadThing& th = things[i];
        widest_ = std::max(widest_, th.halfAcross);
        cells_[key(static_cast<int64_t>(std::floor(th.at.x / kCell)), static_cast<int64_t>(std::floor(th.at.y / kCell)))].push_back(i);
    }
}

bool ThingGrid::beyond(engine::PathId path, engine::Vec2 p, Real reach) const {
    if (path.index >= pathBoxes_.size()) return false;
    const PathBox& b = pathBoxes_[path.index];
    if (b.lo.x > b.hi.x) return false;
    const Real dx = std::max(Real(0), std::max(b.lo.x - p.x, p.x - b.hi.x));
    const Real dy = std::max(Real(0), std::max(b.lo.y - p.y, p.y - b.hi.y));
    // (a hair of slack, so a point the line itself would measure exactly at `reach` is never passed over)
    const Real r = reach * (Real(1) + Real(1e-9)) + Real(1e-9);
    return dx * dx + dy * dy > r * r;
}

bool ThingGrid::near(engine::Vec2 centre, Real radius, std::vector<uint32_t>& out) const {
    const int64_t x0 = static_cast<int64_t>(std::floor((centre.x - radius) / kCell));
    const int64_t x1 = static_cast<int64_t>(std::floor((centre.x + radius) / kCell));
    const int64_t z0 = static_cast<int64_t>(std::floor((centre.y - radius) / kCell));
    const int64_t z1 = static_cast<int64_t>(std::floor((centre.y + radius) / kCell));
    if (static_cast<double>(x1 - x0 + 1) * static_cast<double>(z1 - z0 + 1) > static_cast<double>(count_)) return false;
    out.clear();
    for (int64_t x = x0; x <= x1; ++x)
        for (int64_t z = z0; z <= z1; ++z) {
            const std::unordered_map<uint64_t, std::vector<uint32_t>>::const_iterator c = cells_.find(key(x, z));
            if (c != cells_.end()) out.insert(out.end(), c->second.begin(), c->second.end());
        }
    std::sort(out.begin(), out.end());
    return true;
}

ThingAhead findThingAhead(const engine::LaneGraph& roads, engine::PathId path, Real along,
                          const std::vector<engine::PathId>& pathsAfter, Real reach, Real myHalfWidth, Real myOffset,
                          Real clearance, const std::vector<RoadThing>& things, Real mySpeed, const ThingGrid* grid) {
    ThingAhead best;
    std::vector<uint32_t> nearby;
    if (things.empty() || !path.valid() || path.index >= roads.paths.size()) return best;
    Real start = -along;   // my front to the start of the path looked along
    for (std::size_t k = 0; k <= pathsAfter.size(); ++k) {
        const engine::PathId pid = k == 0 ? path : pathsAfter[k - 1];
        if (!pid.valid() || pid.index >= roads.paths.size()) break;
        if (start > reach) break;
        const engine::Path& P = roads.paths[pid.index];
        if (P.curve.count < 2) {
            start += P.length;
            continue;
        }
        const engine::Vec2 first = roads.points[P.curve.first];
        // (every point of the path lies within its length of its start: a thing farther from it than that and its reach
        // across is beside no part of the path, so only the things near its start need be looked at, ADR-0534)
        const bool someOnly = grid && grid->near(first, P.length + myHalfWidth + clearance + grid->widest(), nearby);
        const std::size_t looked = someOnly ? nearby.size() : things.size();
        for (std::size_t n = 0; n < looked; ++n) {
            const RoadThing& th = things[someOnly ? nearby[n] : n];
            const Real across = myHalfWidth + th.halfAcross + clearance;
            if ((th.at - first).length() > P.length + across) continue;
            // (farther from the path's box than across and the coming widening: nearestOn would put it beyond them too)
            if (grid && grid->beyond(pid, th.at, across + kComingWiden)) continue;
            const OnLine on = nearestOn(roads, pid, th.at);
            if (on.away < 0 || on.away > across + kComingWiden) continue;
            // beside the path's stretch, not off one of its ends
            if (on.away > std::fabs(on.side) + clearance) continue;
            const Real lateral = on.side - myOffset;
            const Real d = start + on.along - th.halfAcross;
            if (std::fabs(lateral) > across) {
                // not in my way yet: is it coming into it? Moving toward my line, reaching my side of it before I reach
                // it, and a little after (kComingMargin)
                if (std::fabs(lateral) > across + kComingWiden) continue;
                const engine::Vec2 dir = roads.directionAt(pid, on.along);
                const Real toward = th.velocity.x * dir.y - th.velocity.y * dir.x;   // (+ toward the kerb side)
                if (!(lateral * toward < 0)) continue;
                const Real theirs = (std::fabs(lateral) - across) / std::fabs(toward);
                const Real mine = std::max(Real(0), d) / std::max(mySpeed, kComingSlowest);
                if (theirs > mine + kComingMargin) continue;
            }
            if (start + on.along + th.halfAcross < 0) continue;   // behind my front
            if (d > reach) continue;
            if (best.found && d >= best.distance) continue;
            const engine::Vec2 dir = roads.directionAt(pid, on.along);
            best.found = true;
            best.id = th.id;
            best.distance = d;
            best.speed = th.velocity.x * dir.x + th.velocity.y * dir.y;
            best.moves = th.velocity.length();
            best.side = on.side;
            best.halfAcross = th.halfAcross;
            best.path = pid;
            // room to go round it, inside my lane: my body clear of it and of the lane's edges by the clearance
            const Real lane = P.width * Real(0.5);
            const Real left = on.side + th.halfAcross + clearance + myHalfWidth;
            const Real right = on.side - th.halfAcross - clearance - myHalfWidth;
            const bool leftFits = left + myHalfWidth <= lane;
            const bool rightFits = right - myHalfWidth >= -lane;
            best.roomToPass = leftFits || rightFits;
            best.passOffset = !best.roomToPass ? Real(0)
                              : leftFits && (!rightFits || std::fabs(left) <= std::fabs(right)) ? left
                                                                                                  : right;
        }
        start += P.length;
    }
    return best;
}

bool registerThingFacts(FactRegistry& registry, std::vector<std::string>* problems) {
    std::vector<FactSpec> specs;
    specs.push_back(spec(kFactThingAhead, FactKind::YesNo, UnitFamily::Ratio, noticeThingAhead,
                         "something is in my way, along my way ahead within my look-ahead (about its path, and how far)"));
    specs.push_back(spec(kFactThingGap, FactKind::Number, UnitFamily::Length, noticeThingGap,
                         "how far ahead the thing in my way is, my front to its near side (m)"));
    FactSpec stop = spec(kFactThingStop, FactKind::Place, UnitFamily::Length, noticeThingStop,
                         "where I stop short of the thing in my way: thing_stand_off before it, or the line of a junction "
                         "beyond which it leaves me no room");
    stop.numbers.push_back(need(kNumberThingStandOff, UnitFamily::Length));
    specs.push_back(stop);
    specs.push_back(spec(kFactThingSpeed, FactKind::Number, UnitFamily::Speed, noticeThingSpeed,
                         "how fast the thing in my way moves along my way (m/s; below zero toward me)"));
    specs.push_back(spec(kFactThingStood, FactKind::Number, UnitFamily::Time, noticeThingStood,
                         "how long the thing in my way has stood still there (s; 0 while it moves)"));
    specs.push_back(spec(kFactWaited, FactKind::Number, UnitFamily::Time, noticeWaited,
                         "how long I have stood behind the thing in my way (s; 0 while I move)"));
    FactSpec room = spec(kFactRoomToPass, FactKind::YesNo, UnitFamily::Ratio, noticeRoomToPass,
                         "there is room to go round the thing in my way, pass_clearance beside it");
    room.numbers.push_back(need(kNumberPassClearance, UnitFamily::Length));
    specs.push_back(room);
    // PEOPLE ON CROSSWALKS (ADR-0452). Their numbers are read where the rule set states them (a rule set without them
    // gives no crosswalk facts: nobody is seen at a crosswalk).
    FactSpec person = spec(kFactCrosswalkPerson, FactKind::YesNo, UnitFamily::Ratio, noticeCrosswalkPerson,
                           "someone is on a crosswalk ahead of me that my way crosses, or waits at its kerb plainly about to "
                           "cross where no lights run it and I can still stop for them comfortably");
    person.numbers.push_back(need(kNumberCrosswalkDecel, UnitFamily::Acceleration));
    specs.push_back(person);
    FactSpec cwStop = spec(kFactCrosswalkStop, FactKind::Place, UnitFamily::Length, noticeCrosswalkStop,
                           "where I stop for the people at the crosswalk ahead: crosswalk_stand_off before my way reaches it");
    cwStop.numbers.push_back(need(kNumberCrosswalkStandOff, UnitFamily::Length));
    specs.push_back(cwStop);
    bool ok = true;
    for (FactSpec& s : specs) {
        std::string why;
        if (registry.add(std::move(s), &why) == kNoFact) {
            ok = false;
            if (problems) problems->push_back(why);
        }
    }
    ReaderSpec reader;
    reader.name = kThingReaderName;
    reader.meaning = "looks for the things in each driver's way and carries out going round them";
    reader.numbers.push_back(need(kNumberThingStandOff, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberPassClearance, UnitFamily::Length));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        ok = false;
        if (problems) problems->push_back(why);
    }
    return ok;
}

bool registerThingAnswers(AnswerVocabulary& vocabulary, std::string* why) {
    return vocabulary.addAction(kActionHorn, "sound the horn at what stands in my way", why) != kNoAction &&
           vocabulary.addIntention(kIntentionPass, {kWordSwerve}, "go round what is in my way, to the side of my line",
                                   why) != kNoIntentionKind &&
           vocabulary.addIntention(kIntentionWay, {kWordAnother},
                                   "find another way from where I am, round the stretch where something stands", why) !=
               kNoIntentionKind;
}

bool hasThingStep(const FactRegistry& registry) { return registry.findReader(kThingReaderName) != nullptr; }

}  // namespace driving
