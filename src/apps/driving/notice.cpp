#include "apps/driving/notice.h"

#include "apps/driving/road_rules.h"

#include <cmath>
#include <limits>
#include <utility>

namespace driving {

namespace {

constexpr Real kNoSpeedYet = std::numeric_limits<Real>::infinity();
constexpr Real kTwo = 2;   // the 2 in v^2 = u^2 + 2 a s

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

// ---- Reading the way ahead -----------------------------------------------------------------------------------------

// The way as the facts read it: the supplied paths when they start on my own path, else my own path alone.
bool wayStartsOnMine(const DriverView& v) { return !v.way.paths.empty() && v.way.paths[0] == v.me.path; }
std::size_t wayCount(const DriverView& v) { return wayStartsOnMine(v) ? v.way.paths.size() : 1; }
engine::PathId wayPath(const DriverView& v, std::size_t i) { return i == 0 ? v.me.path : v.way.paths[i]; }

// Whether a regulation means "the top speed is my own value", through the device its kind is bound to.
bool setsTopSpeed(const DriverView& v, const engine::Regulation& reg) {
    const Device* d = v.roadRules.device(v.rules, reg.kind);
    return d && d->topSpeedIsValue;
}

// The limit in force at `along` on `path`: the path's own limit, replaced by the latest regulation at or before that
// point that sets the top speed.
Real limitAt(const DriverView& v, engine::PathId path, Real along) {
    const engine::Path& p = v.roads.paths[path.index];
    Real limit = p.speedLimit;
    for (uint32_t r = p.regulations.first; r < p.regulations.end() && r < v.roads.regulations.size(); ++r) {
        const engine::Regulation& reg = v.roads.regulations[r];
        if (reg.at > along) break;
        if (setsTopSpeed(v, reg)) limit = reg.value;
    }
    return limit;
}

// ---- Work the facts share (through the scratch) --------------------------------------------------------------------

// The car ahead, looked up once per driver.
const Neighbour& lookCarAhead(const DriverView& v, NoticeScratch& s) {
    if (!s.haveCarAhead) {
        s.pathsAfterMine.clear();
        for (std::size_t i = 1; i < wayCount(v); ++i) s.pathsAfterMine.push_back(wayPath(v, i));
        s.carAhead = carAhead(v.roads, v.cars, v.lists, v.me.id, s.pathsAfterMine, v.lookAhead);
        ++s.carAheadLookups;
        s.haveCarAhead = true;
    }
    return s.carAhead;
}

// The limit where my front is, worked out once per driver. False on an unknown path.
bool lookRoadLimit(const DriverView& v, NoticeScratch& s, Real& limit) {
    if (!knownPath(v.roads, v.me.path)) return false;
    if (!s.haveRoadLimit) {
        s.roadLimit = limitAt(v, v.me.path, v.me.along);
        s.haveRoadLimit = true;
    }
    limit = s.roadLimit;
    return true;
}

// This person's speed habit (a ratio), through personalValue. False when the rule set has no such setting.
bool speedHabit(const DriverView& v, Real& habit) {
    const PersonalSetting* s = v.rules.findPersonal(kSettingSpeedHabit);
    if (!s) return false;
    habit = static_cast<Real>(personalValue(*s, v.person));
    return true;
}

// The speed I may do now so that, braking comfortably at `brake`, I am down to `speedThere` after `distance` metres.
Real speedNowFor(Real speedThere, Real brake, Real distance) {
    return std::sqrt(speedThere * speedThere + kTwo * brake * distance);
}

// ---- Filling a fact -----------------------------------------------------------------------------------------------

void setNumber(NoticedFact& out, Real value) {
    out.present = true;
    out.number = value;
}
void setYes(NoticedFact& out, bool yes) {
    out.present = true;
    out.yes = yes;
}
void setMetres(NoticedFact& out, Real metres) {
    out.hasMetres = true;
    out.metres = metres;
}

// ---- The built-in facts -------------------------------------------------------------------------------------------

// The car-ahead family: four facts from one lookup.
void noticeCarAhead(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    const Neighbour& n = lookCarAhead(v, s);
    setYes(out, n.found());
    if (n.found()) {
        out.aboutCar = n.car;
        setMetres(out, n.gap);
    }
}
void noticeGapAhead(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    const Neighbour& n = lookCarAhead(v, s);
    if (!n.found()) return;
    setNumber(out, n.gap);
    out.aboutCar = n.car;
    setMetres(out, n.gap);
}
void noticeSpeedAhead(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    const Neighbour& n = lookCarAhead(v, s);
    if (!n.found()) return;
    setNumber(out, n.speed);
    out.aboutCar = n.car;
    setMetres(out, n.gap);
}
void noticeClosingSpeed(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    const Neighbour& n = lookCarAhead(v, s);
    if (!n.found()) return;
    setNumber(out, v.me.speed - n.speed);
    out.aboutCar = n.car;
    setMetres(out, n.gap);
}

void noticeRoadLimit(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    Real limit = 0;
    if (!lookRoadLimit(v, s, limit)) return;
    setNumber(out, limit);
    out.aboutPath = v.me.path;
}

void noticeMyLimit(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    Real limit = 0, habit = 0;
    if (!lookRoadLimit(v, s, limit) || !speedHabit(v, habit)) return;
    setNumber(out, limit * habit);
    out.aboutPath = v.me.path;
}

// The lowest "speed I may do now" over every limit ahead lower than the one I am under.
void noticeSlowerAhead(const DriverView& v, NoticeScratch& s, NoticedFact& out) {
    Real current = 0, habit = 0;
    if (!lookRoadLimit(v, s, current) || !speedHabit(v, habit)) return;
    const Real brake = v.myClass.brake;
    Real best = kNoSpeedYet;
    engine::PathId bestPath;
    Real bestMetres = 0;
    const auto consider = [&](Real limit, engine::PathId path, Real distance) {
        if (!(limit < current) || distance > v.lookAhead) return;
        const Real now = speedNowFor(limit * habit, brake, distance);
        if (now < best) {
            best = now;
            bestPath = path;
            bestMetres = distance;
        }
    };
    // Ahead of me on my own path.
    const engine::Path& mine = v.roads.paths[v.me.path.index];
    for (uint32_t r = mine.regulations.first; r < mine.regulations.end() && r < v.roads.regulations.size(); ++r) {
        const engine::Regulation& reg = v.roads.regulations[r];
        if (reg.at > v.me.along && setsTopSpeed(v, reg)) consider(reg.value, v.me.path, reg.at - v.me.along);
    }
    // On each later path: the limit in force as I enter it, then its regulations further along.
    Real toStart = mine.length - v.me.along;
    for (std::size_t i = 1; i < wayCount(v); ++i) {
        const engine::PathId id = wayPath(v, i);
        if (!knownPath(v.roads, id) || toStart > v.lookAhead) break;
        consider(limitAt(v, id, 0), id, toStart);
        const engine::Path& p = v.roads.paths[id.index];
        for (uint32_t r = p.regulations.first; r < p.regulations.end() && r < v.roads.regulations.size(); ++r) {
            const engine::Regulation& reg = v.roads.regulations[r];
            if (reg.at > 0 && setsTopSpeed(v, reg)) consider(reg.value, id, toStart + reg.at);
        }
        toStart += p.length;
    }
    if (best == kNoSpeedYet) return;
    setNumber(out, best);
    out.aboutPath = bestPath;
    setMetres(out, bestMetres);
}

// The lowest "speed I may do now" over every bend on the paths in view; my own path's counts from where I am.
void noticeCornerSpeed(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!knownPath(v.roads, v.me.path)) return;
    const Number* side = v.rules.findNumber(kNumberComfortableSideAccel);
    if (!side || !(side->value > 0)) return;
    const Real sideAccel = static_cast<Real>(side->value);
    const Real brake = v.myClass.brake;
    Real best = kNoSpeedYet;
    engine::PathId bestPath;
    Real bestMetres = 0;
    Real toStart = 0;
    for (std::size_t i = 0; i < wayCount(v); ++i) {
        const engine::PathId id = wayPath(v, i);
        if (!knownPath(v.roads, id) || toStart > v.lookAhead) break;
        const engine::Path& p = v.roads.paths[id.index];
        // A radius that is not a positive finite number is no bend (kNoBend, or a path whose radius was never set).
        if (std::isfinite(p.tightestRadius) && p.tightestRadius > 0) {
            const Real now = speedNowFor(std::sqrt(sideAccel * p.tightestRadius), brake, toStart);
            if (now < best) {
                best = now;
                bestPath = id;
                bestMetres = toStart;
            }
        }
        toStart += (i == 0) ? p.length - v.me.along : p.length;
    }
    if (best == kNoSpeedYet) return;
    setNumber(out, best);
    out.aboutPath = bestPath;
    setMetres(out, bestMetres);
}

// Where my way runs out: the first path on it with nothing after it, if its end is within my look-ahead.
struct WayEnd {
    bool found = false;
    engine::PathId path;
    Real metres = 0;
};
WayEnd findWayEnd(const DriverView& v) {
    WayEnd end;
    if (!knownPath(v.roads, v.me.path)) return end;
    Real toEnd = -v.me.along;
    for (std::size_t i = 0; i < wayCount(v); ++i) {
        const engine::PathId id = wayPath(v, i);
        if (!knownPath(v.roads, id)) break;
        const engine::Path& p = v.roads.paths[id.index];
        toEnd += p.length;
        if (toEnd > v.lookAhead) break;
        if (p.next.count == 0) {
            end.found = true;
            end.path = id;
            end.metres = toEnd;
            break;
        }
    }
    return end;
}
void noticeLaneEndsAhead(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const WayEnd end = findWayEnd(v);
    setYes(out, end.found);
    if (end.found) {
        out.aboutPath = end.path;
        setMetres(out, end.metres);
    }
}
void noticeLaneEnd(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const WayEnd end = findWayEnd(v);
    if (!end.found) return;
    setNumber(out, end.metres);
    out.aboutPath = end.path;
    setMetres(out, end.metres);
}

void noticeMySpeed(const DriverView& v, NoticeScratch&, NoticedFact& out) { setNumber(out, v.me.speed); }
void noticeStanding(const DriverView& v, NoticeScratch&, NoticedFact& out) { setYes(out, v.me.speed < kStandingBelow); }

FactSpec spec(const char* name, FactKind kind, NoticeFn fn, const char* meaning) {
    FactSpec s;
    s.name = name;
    s.kind = kind;
    s.notice = fn;
    s.meaning = meaning;
    return s;
}
FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

}  // namespace

// ---- NoticedFact, WayAhead, scratch -------------------------------------------------------------------------------

bool operator==(const NoticedFact& a, const NoticedFact& b) {
    if (a.kind != b.kind || a.present != b.present) return false;
    if (!a.present) return true;
    return a.number == b.number && a.yes == b.yes && a.word == b.word && a.aboutCar == b.aboutCar &&
           a.aboutPath == b.aboutPath && a.hasSeconds == b.hasSeconds && (!a.hasSeconds || a.seconds == b.seconds) &&
           a.hasMetres == b.hasMetres && (!a.hasMetres || a.metres == b.metres);
}

WayAhead followOnlyWay(const engine::LaneGraph& roads, engine::PathId path, Real along, Real lookAhead) {
    WayAhead way;
    if (!knownPath(roads, path)) return way;
    way.paths.push_back(path);
    Real reach = roads.paths[path.index].length - along;
    engine::PathId at = path;
    while (reach < lookAhead && way.paths.size() < kMaxFollowedPaths) {
        const engine::Path& p = roads.paths[at.index];
        if (p.next.count != 1 || p.next.first >= roads.nextPaths.size()) break;
        const engine::PathId next = roads.nextPaths[p.next.first];
        if (!knownPath(roads, next)) break;
        way.paths.push_back(next);
        reach += roads.paths[next.index].length;
        at = next;
    }
    return way;
}

void NoticeScratch::clearForDriver() {
    haveCarAhead = false;
    carAhead = Neighbour();
    haveRoadLimit = false;
    roadLimit = 0;
    pathsAfterMine.clear();
    worked.clear();
    carAheadLookups = 0;
}

// ---- The registry ------------------------------------------------------------------------------------------------

FactSlot FactRegistry::add(FactSpec spec, std::string* why) {
    if (spec.name.empty()) {
        if (why) *why = "a fact needs a name";
        return kNoFact;
    }
    if (find(spec.name) != kNoFact) {
        if (why) *why = "fact \"" + spec.name + "\" is already registered";
        return kNoFact;
    }
    if (!spec.notice) {
        if (why) *why = "fact \"" + spec.name + "\" has no function to work it out";
        return kNoFact;
    }
    facts_.push_back(std::move(spec));
    return static_cast<FactSlot>(facts_.size() - 1);
}

FactSlot FactRegistry::find(const std::string& name) const {
    for (std::size_t i = 0; i < facts_.size(); ++i)
        if (facts_[i].name == name) return static_cast<FactSlot>(i);
    return kNoFact;
}

void noticeFacts(const DriverView& view, const FactRegistry& registry, const std::vector<FactSlot>& wanted,
                 std::vector<NoticedFact>& out, NoticeScratch& scratch) {
    scratch.clearForDriver();
    scratch.worked.assign(registry.size(), 0);
    out.resize(registry.size());
    for (std::size_t i = 0; i < registry.size(); ++i) {
        out[i] = NoticedFact();
        out[i].kind = registry.at(static_cast<FactSlot>(i)).kind;
    }
    for (const FactSlot slot : wanted) {
        if (slot >= registry.size() || scratch.worked[slot]) continue;
        scratch.worked[slot] = 1;
        registry.at(slot).notice(view, scratch, out[slot]);
    }
}

void noticeFacts(const DriverView& view, const FactRegistry& registry, const std::vector<FactSlot>& wanted,
                 std::vector<NoticedFact>& out) {
    NoticeScratch scratch;
    noticeFacts(view, registry, wanted, out, scratch);
}

bool registerBuiltInFacts(FactRegistry& registry, std::vector<std::string>* problems) {
    std::vector<FactSpec> specs;
    specs.push_back(spec(kFactCarAhead, FactKind::YesNo, noticeCarAhead,
                         "there is a car ahead of me on my way, within how far I look"));
    specs.push_back(spec(kFactGapAhead, FactKind::Number, noticeGapAhead,
                         "metres from my front bumper to the rear of the car ahead"));
    specs.push_back(spec(kFactSpeedAhead, FactKind::Number, noticeSpeedAhead, "how fast the car ahead is going, m/s"));
    specs.push_back(spec(kFactClosingSpeed, FactKind::Number, noticeClosingSpeed,
                         "how much faster I am going than the car ahead, m/s (below zero: it is pulling away)"));
    specs.push_back(spec(kFactRoadLimit, FactKind::Number, noticeRoadLimit,
                         "the speed limit in force where my front bumper is, m/s"));
    FactSpec myLimit = spec(kFactMyLimit, FactKind::Number, noticeMyLimit,
                            "the speed I like to drive here: the limit times my own speed habit, m/s");
    myLimit.settings.push_back(need(kSettingSpeedHabit, UnitFamily::Ratio));
    specs.push_back(myLimit);
    FactSpec slower = spec(kFactSlowerAhead, FactKind::Number, noticeSlowerAhead,
                           "the speed I may do now and still slow comfortably to a lower limit ahead, m/s");
    slower.settings.push_back(need(kSettingSpeedHabit, UnitFamily::Ratio));
    specs.push_back(slower);
    FactSpec corner = spec(kFactCornerSpeed, FactKind::Number, noticeCornerSpeed,
                           "the speed I may do now and still slow comfortably for the tightest bend ahead, m/s");
    corner.numbers.push_back(need(kNumberComfortableSideAccel, UnitFamily::Acceleration));
    specs.push_back(corner);
    specs.push_back(spec(kFactLaneEndsAhead, FactKind::YesNo, noticeLaneEndsAhead,
                         "my way runs out within how far I look"));
    specs.push_back(spec(kFactLaneEnd, FactKind::Place, noticeLaneEnd, "where my way runs out, metres ahead"));
    specs.push_back(spec(kFactMySpeed, FactKind::Number, noticeMySpeed, "how fast I am going, m/s"));
    specs.push_back(spec(kFactStanding, FactKind::YesNo, noticeStanding, "I am standing still, or as good as"));

    bool ok = true;
    for (FactSpec& s : specs) {
        std::string why;
        if (registry.add(std::move(s), &why) == kNoFact) {
            ok = false;
            if (problems) problems->push_back(why);
        }
    }
    return ok;
}

}  // namespace driving
