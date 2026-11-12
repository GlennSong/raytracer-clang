#include "apps/driving/right_of_way.h"

#include "apps/driving/traffic_state.h"   // kerbwardPerpSign

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace driving {

namespace {

using engine::CrossingId;
using engine::CrossingPoint;
using engine::JunctionId;
using engine::LaneGraph;
using engine::PathId;
using engine::Range;
using engine::Vec2;

constexpr Real kPi = 3.14159265358979323846;
constexpr Real kDegreesPerRadian = 180.0 / kPi;
// The compass has eight points, each this many degrees wide.
constexpr Real kCompassPointDegrees = 45.0;
constexpr int kCompassPoints = 8;
// The compass points, anticlockwise from east (seen from above, north up).
const char* const kCompassWords[kCompassPoints] = {"east",      "north-east", "north",      "north-west",
                                                   "west",      "south-west", "south",      "south-east"};
// How many of the loader's notes describeRightOfWay prints (the rest are counted).
constexpr std::size_t kNotesShown = 20;

// The connector's own stated turn as a movement word's value (B2 states it; it is never worked out again here).
Movement movementOfTurn(engine::Turn t) {
    switch (t) {
    case engine::Turn::NearTurn: return Movement::NearTurn;
    case engine::Turn::FarTurn: return Movement::FarTurn;
    case engine::Turn::UTurn: return Movement::UTurn;
    case engine::Turn::Straight:
    case engine::Turn::None: break;
    }
    return Movement::Straight;
}

bool facing(const Vec2& a, const Vec2& b) { return engine::dot(a, b) < kFacingDot; }
bool alongside(const Vec2& a, const Vec2& b) { return engine::dot(a, b) > kAlongsideDot; }

// How far round from b's direction of travel, toward b's kerb, lies the way approach a's traffic comes from: above zero
// (up to a half turn) when it is on b's kerb half.
Real kerbwardAngle(const Vec2& a, const Vec2& b, DrivesOn drivesOn) {
    const Vec2 from = a * Real(-1);
    const Vec2 kerb = engine::perp(b) * kerbwardPerpSign(drivesOn);
    return std::atan2(engine::dot(from, kerb), engine::dot(from, b));
}

// The sign an approach carries: the last regulation along its arriving lane whose device has a rank. None: rank 0
// (an approach with no sign shares the rank listed first).
int signRankOf(const LaneGraph& roads, const RuleSet& rules, const RoadRules& roadRules, PathId lane) {
    int rank = 0;
    const engine::Range regs = roads.paths[lane.index].regulations;
    for (uint32_t r = regs.first; r < regs.end() && r < roads.regulations.size(); ++r) {
        const Device* d = roadRules.device(rules, roads.regulations[r].kind);
        if (d && d->rank != kNoRank) rank = d->rank;
    }
    return rank;
}

// The stretch of crossing record `rec` on path `p` (its side a, or its other side when that is p).
void stretchOn(const CrossingPoint& rec, PathId p, Real& from, Real& to) {
    if (rec.a == p) {
        from = rec.aFrom;
        to = rec.aTo;
    } else {
        from = rec.otherFrom;
        to = rec.otherTo;
    }
}

// The clear gaps along connector `p` between its crossing stretches (splits left out: there the car behind follows).
void findGaps(const LaneGraph& roads, PathId p, RightOfWay& out) {
    ConnectorRules& cr = out.paths[p.index];
    const engine::Path& path = roads.paths[p.index];
    std::vector<ClearGap> stretches;
    for (uint32_t i = path.crossings.first; i < path.crossings.end() && i < roads.pathCrossings.size(); ++i) {
        const CrossingPoint& rec = roads.crossings[roads.pathCrossings[i].index];
        if (rec.meeting == engine::Meeting::Split) continue;
        ClearGap s;
        stretchOn(rec, p, s.from, s.to);
        stretches.push_back(s);
    }
    std::sort(stretches.begin(), stretches.end(), [](const ClearGap& x, const ClearGap& y) {
        return x.from != y.from ? x.from < y.from : x.to < y.to;
    });
    cr.firstStretchAt = stretches.empty() ? path.length : stretches.front().from;
    cr.gaps.first = static_cast<uint32_t>(out.gaps.size());
    Real reach = 0;
    bool any = false;
    for (const ClearGap& s : stretches) {
        if (any && s.from > reach) {
            ClearGap g;
            g.from = reach;
            g.to = s.from;
            out.gaps.push_back(g);
        }
        reach = any ? std::max(reach, s.to) : s.to;
        any = true;
    }
    cr.gaps.count = static_cast<uint32_t>(out.gaps.size()) - cr.gaps.first;
}

// ONE JUNCTION, while its part of the table is built: its connectors in order, each one's approach (an index into
// the junction's own approaches) and movement, which pairs meet, and which conflict for the lights.
struct JunctionWork {
    std::vector<PathId> conn;
    std::vector<uint32_t> app;        // local approach of each connector
    std::vector<Movement> mov;
    std::vector<Vec2> heading;        // per local approach
    std::vector<uint8_t> meet;        // n * n: a cross or a join between the two
    uint32_t n = 0;

    bool meets(uint32_t i, uint32_t k) const { return meet[i * n + k] != 0; }
};

// May turns i and k of a junction with control `ck` be green together? Yes when they don't meet, come from one approach,
// or come from approaches facing each other and one of them may go on a plain green, giving way.
bool conflict(const JunctionWork& w, const ControlKind& ck, uint32_t i, uint32_t k) {
    if (i == k || w.app[i] == w.app[k] || !w.meets(i, k)) return false;
    const bool shared = facing(w.heading[w.app[i]], w.heading[w.app[k]]) &&
                        (hasMovement(ck.givesWayOnGreen, w.mov[i]) || hasMovement(ck.givesWayOnGreen, w.mov[k]));
    return !shared;
}

// THE PHASES of a junction with lights, each a list of which connectors are green (in local order).
std::vector<std::pair<bool, std::vector<uint8_t>>> buildPhases(const JunctionWork& w, const ControlKind& ck) {
    const uint32_t n = w.n;
    const uint32_t m = static_cast<uint32_t>(w.heading.size());
    // Whole approaches first, coloured greedily in order: an approach joins the first phase none of whose approaches has
    // a turn conflicting with one of its own.
    std::vector<std::vector<uint8_t>> appConflict(m, std::vector<uint8_t>(m, 0));
    for (uint32_t i = 0; i < n; ++i)
        for (uint32_t k = 0; k < n; ++k)
            if (conflict(w, ck, i, k)) appConflict[w.app[i]][w.app[k]] = 1;
    std::vector<uint32_t> colour(m, 0);
    uint32_t colours = 0;
    for (uint32_t a = 0; a < m; ++a) {
        uint32_t c = 0;
        for (; c < colours; ++c) {
            bool ok = true;
            for (uint32_t b = 0; b < a && ok; ++b)
                if (colour[b] == c && appConflict[a][b]) ok = false;
            if (ok) break;
        }
        colour[a] = c;
        if (c == colours) ++colours;
    }
    std::vector<std::vector<uint8_t>> main(colours, std::vector<uint8_t>(n, 0));
    for (uint32_t i = 0; i < n; ++i) main[colour[w.app[i]]][i] = 1;
    // Then single turns: any turn that conflicts with nothing green in a phase joins it.
    for (std::vector<uint8_t>& green : main)
        for (uint32_t i = 0; i < n; ++i) {
            if (green[i]) continue;
            bool ok = true;
            for (uint32_t k = 0; k < n && ok; ++k)
                if (green[k] && conflict(w, ck, i, k)) ok = false;
            if (ok) green[i] = 1;
        }
    // Arrows: before each phase, the arrow movements that would there share their green with what they meet from the
    // approach facing them get phases of their own (coloured among themselves by the same test).
    std::vector<std::pair<bool, std::vector<uint8_t>>> out;
    for (const std::vector<uint8_t>& green : main) {
        std::vector<uint32_t> candidates;
        for (uint32_t i = 0; i < n; ++i) {
            if (!green[i] || !hasMovement(ck.arrow, w.mov[i])) continue;
            bool givesWayHere = false;
            for (uint32_t k = 0; k < n && !givesWayHere; ++k)
                if (green[k] && w.app[k] != w.app[i] && w.meets(i, k) && facing(w.heading[w.app[i]], w.heading[w.app[k]]))
                    givesWayHere = true;
            if (givesWayHere) candidates.push_back(i);
        }
        std::vector<std::vector<uint8_t>> arrows;
        for (const uint32_t i : candidates) {
            std::size_t c = 0;
            for (; c < arrows.size(); ++c) {
                bool ok = true;
                for (uint32_t k = 0; k < n && ok; ++k)
                    if (arrows[c][k] && conflict(w, ck, i, k)) ok = false;
                if (ok) break;
            }
            if (c == arrows.size()) arrows.push_back(std::vector<uint8_t>(n, 0));
            arrows[c][i] = 1;
        }
        for (const std::vector<uint8_t>& a : arrows) out.push_back({true, a});
        out.push_back({false, green});
    }
    return out;
}

// What one question says about turns a and b (local indexes) at `phase`: true when it tells them apart, with who gives
// way in `who` (A for a, Other for b).
bool ask(SettleQuestion q, const JunctionWork& w, const RightOfWay& row, const JunctionRules& jr, const RuleSet& rules,
         uint32_t a, uint32_t b, uint32_t phase, GivesWay& who) {
    switch (q) {
    case SettleQuestion::Light: {
        if (jr.phases.count == 0 || phase >= jr.phases.count) return false;
        const Phase& ph = row.phases[jr.phases.first + phase];
        const Light la = row.lights[ph.lights.first + a], lb = row.lights[ph.lights.first + b];
        if (la == Light::Green && lb != Light::Green) who = GivesWay::Other;
        else if (lb == Light::Green && la != Light::Green) who = GivesWay::A;
        else if (la == Light::RedMayTurn && lb == Light::Red) who = GivesWay::A;
        else if (lb == Light::RedMayTurn && la == Light::Red) who = GivesWay::Other;
        else return false;
        return true;
    }
    case SettleQuestion::SignRank: {
        const int ra = row.approaches[jr.approaches.first + w.app[a]].signRank;
        const int rb = row.approaches[jr.approaches.first + w.app[b]].signRank;
        if (ra == rb) return false;
        who = ra > rb ? GivesWay::A : GivesWay::Other;
        return true;
    }
    case SettleQuestion::Movement: {
        const uint32_t ra = rules.movementRankOf(w.mov[a]), rb = rules.movementRankOf(w.mov[b]);
        if (ra == rb) return false;
        who = ra > rb ? GivesWay::A : GivesWay::Other;
        return true;
    }
    case SettleQuestion::KerbSide: {
        const bool aOnB = onKerbSideOf(w.heading, w.app[a], w.app[b], rules.drivesOn);
        const bool bOnA = onKerbSideOf(w.heading, w.app[b], w.app[a], rules.drivesOn);
        if (aOnB == bOnA) return false;
        who = aOnB ? GivesWay::Other : GivesWay::A;   // the one on the other's kerb side goes first
        return true;
    }
    case SettleQuestion::ArrivedFirst: return true;   // asked while driving
    }
    return false;
}

}  // namespace

bool operator==(const MeetingAnswer& a, const MeetingAnswer& b) {
    return a.who == b.who && a.by == b.by && a.fallback == b.fallback && a.fallbackBy == b.fallbackBy;
}

bool onKerbSideOf(const std::vector<Vec2>& all, std::size_t a, std::size_t b, DrivesOn drivesOn) {
    if (a == b || a >= all.size() || b >= all.size()) return false;
    if (facing(all[a], all[b]) || alongside(all[a], all[b])) return false;
    const Real mine = kerbwardAngle(all[a], all[b], drivesOn);
    if (!(mine > 0)) return false;
    for (std::size_t x = 0; x < all.size(); ++x) {
        if (x == a || x == b || facing(all[x], all[b]) || alongside(all[x], all[b])) continue;
        const Real t = kerbwardAngle(all[x], all[b], drivesOn);
        if (t > 0 && (t < mine || (t == mine && x < a))) return false;   // another is found first
    }
    return true;
}

const MeetingAnswer* RightOfWay::answer(CrossingId crossing, uint32_t phase) const {
    if (!crossing.valid() || crossing.index >= answersOf.size()) return nullptr;
    const Range r = answersOf[crossing.index];
    return phase < r.count ? &answers[r.first + phase] : nullptr;
}

Light RightOfWay::light(const LaneGraph& roads, PathId connector, uint32_t phase) const {
    if (!connector.valid() || connector.index >= roads.paths.size()) return Light::Green;
    const engine::Path& p = roads.paths[connector.index];
    if (p.kind != engine::PathKind::Connector || !p.junction.valid() || p.junction.index >= junctions.size())
        return Light::Green;
    const JunctionRules& jr = junctions[p.junction.index];
    if (jr.phases.count == 0) return Light::Green;
    if (phase >= jr.phases.count) return Light::Red;
    return lights[phases[jr.phases.first + phase].lights.first + this->paths[connector.index].indexInJunction];
}

RightOfWay buildRightOfWay(const LaneGraph& roads, const RuleSet& rules, const RoadRules& roadRules) {
    RightOfWay out;
    out.junctions.resize(roads.junctions.size());
    out.paths.resize(roads.paths.size());
    out.answersOf.assign(roads.crossings.size(), Range());
    out.counts.junctions = static_cast<uint32_t>(roads.junctions.size());

    // Where a body can stand along each connector.
    for (uint32_t p = 0; p < roads.paths.size(); ++p)
        if (roads.paths[p].kind == engine::PathKind::Connector) findGaps(roads, PathId(p), out);

    // Each junction's approaches, connectors and lights.
    std::vector<JunctionWork> work(roads.junctions.size());
    for (uint32_t j = 0; j < roads.junctions.size(); ++j) {
        const engine::Junction& jn = roads.junctions[j];
        JunctionRules& jr = out.junctions[j];
        JunctionWork& w = work[j];
        if (jn.control < roadRules.controlOfName.size() && roadRules.controlOfName[jn.control] < rules.controls.size()) {
            jr.control = roadRules.controlOfName[jn.control];
            if (jn.control < roadRules.darkOfName.size() && roadRules.darkOfName[jn.control]) ++out.counts.junctionsDark;
        } else {
            ++out.counts.junctionsWithoutControl;
            out.notes.push_back("junction " + std::to_string(j) + ": rule set \"" + rules.name +
                                "\" has no row for its control: every pair there falls to the lower path id");
        }
        jr.approaches.first = static_cast<uint32_t>(out.approaches.size());
        w.n = jn.connectors.count;
        for (uint32_t i = 0; i < jn.connectors.count; ++i) {
            const PathId c = roads.junctionConnectors[jn.connectors.first + i];
            const engine::Path& cp = roads.paths[c.index];
            const PathId from = cp.from;
            const engine::SectionId section =
                from.valid() && from.index < roads.paths.size() ? roads.paths[from.index].section : engine::SectionId();
            // The approach: the one with this lane's section (or this lane, when it lies in none), or a new one.
            uint32_t local = static_cast<uint32_t>(w.heading.size());
            for (uint32_t a = 0; a < w.heading.size(); ++a) {
                const Approach& ap = out.approaches[jr.approaches.first + a];
                if (section.valid() ? ap.section == section : ap.firstLane == from) {
                    local = a;
                    break;
                }
            }
            if (local == w.heading.size()) {
                Approach ap;
                ap.section = section;
                ap.firstLane = from;
                ap.heading = from.valid() && from.index < roads.paths.size()
                                 ? roads.directionAt(from, roads.paths[from.index].length)
                                 : Vec2(1, 0);
                ap.signRank = from.valid() && from.index < roads.paths.size()
                                  ? signRankOf(roads, rules, roadRules, from)
                                  : 0;
                out.approaches.push_back(ap);
                w.heading.push_back(ap.heading);
            } else {
                Approach& ap = out.approaches[jr.approaches.first + local];
                if (from < ap.firstLane) ap.firstLane = from;
            }
            w.conn.push_back(c);
            w.app.push_back(local);
            w.mov.push_back(movementOfTurn(cp.turn));
            ConnectorRules& cr = out.paths[c.index];
            cr.approach = jr.approaches.first + local;
            cr.indexInJunction = i;
            cr.movement = w.mov.back();
        }
        jr.approaches.count = static_cast<uint32_t>(w.heading.size());
        // Which pairs meet (a cross or a join filed under this junction).
        w.meet.assign(static_cast<std::size_t>(w.n) * w.n, 0);
        for (uint32_t c = jn.crossings.first; c < jn.crossings.end() && c < roads.crossings.size(); ++c) {
            const CrossingPoint& rec = roads.crossings[c];
            if (rec.otherSide != engine::OtherSide::Path || rec.meeting == engine::Meeting::Split) continue;
            const PathId o = rec.otherPath();
            if (roads.paths[rec.a.index].junction.index != j || roads.paths[o.index].junction.index != j) continue;
            const uint32_t ia = out.paths[rec.a.index].indexInJunction, io = out.paths[o.index].indexInJunction;
            w.meet[ia * w.n + io] = w.meet[io * w.n + ia] = 1;
        }
        // The lights.
        if (jr.control == kNoControl || !rules.controls[jr.control].hasLights()) continue;
        const ControlKind& ck = rules.controls[jr.control];
        const std::vector<std::pair<bool, std::vector<uint8_t>>> phases = buildPhases(w, ck);
        jr.phases.first = static_cast<uint32_t>(out.phases.size());
        for (const std::pair<bool, std::vector<uint8_t>>& ph : phases) {
            Phase p;
            p.arrow = ph.first;
            p.lights.first = static_cast<uint32_t>(out.lights.size());
            p.lights.count = w.n;
            for (uint32_t i = 0; i < w.n; ++i) {
                Light l = Light::Red;
                if (ph.second[i]) l = Light::Green;
                else if (ck.turnOnRed && w.mov[i] == ck.turnOnRedMovement) l = Light::RedMayTurn;
                out.lights.push_back(l);
            }
            out.phases.push_back(p);
        }
        jr.phases.count = static_cast<uint32_t>(phases.size());
        ++out.counts.junctionsWithLights;
        out.counts.phases += jr.phases.count;
    }

    // Every meeting of two paths, in id order.
    for (uint32_t c = 0; c < roads.crossings.size(); ++c) {
        const CrossingPoint& rec = roads.crossings[c];
        if (rec.otherSide != engine::OtherSide::Path) continue;
        ++out.counts.meetings;
        Range& mine = out.answersOf[c];
        mine.first = static_cast<uint32_t>(out.answers.size());
        mine.count = rec.junction.valid() && rec.junction.index < out.junctions.size()
                         ? out.junctions[rec.junction.index].phaseCount()
                         : 1;
        const PathId o = rec.otherPath();
        if (rec.meeting == engine::Meeting::Split) {
            MeetingAnswer a;
            a.who = GivesWay::Neither;
            a.by = SettledBy::Split;
            for (uint32_t ph = 0; ph < mine.count; ++ph) out.answers.push_back(a);
            ++out.counts.splits;
            continue;
        }
        if (!rec.junction.valid()) {
            MeetingAnswer a;
            a.who = GivesWay::Ask;
            a.by = SettledBy::AcrossJunctions;
            a.fallback = GivesWay::A;
            a.fallbackBy = SettledBy::LowerPathId;
            out.answers.push_back(a);
            ++out.counts.acrossJunctions;
            ++out.counts.askWhileDriving;
            ++out.counts.byLowerPathId;
            continue;
        }
        const uint32_t j = rec.junction.index;
        const JunctionRules& jr = out.junctions[j];
        const JunctionWork& w = work[j];
        const uint32_t ia = out.paths[rec.a.index].indexInJunction, io = out.paths[o.index].indexInJunction;
        const std::vector<SettleQuestion> none;
        const std::vector<SettleQuestion>& settle = jr.control != kNoControl ? rules.controls[jr.control].settle : none;
        bool anyAsk = false, anyPathId = false;
        for (uint32_t ph = 0; ph < jr.phaseCount(); ++ph) {
            MeetingAnswer a;
            bool asking = false, decided = false;
            for (const SettleQuestion q : settle) {
                GivesWay who = GivesWay::A;
                if (!ask(q, w, out, jr, rules, ia, io, ph, who)) continue;
                if (q == SettleQuestion::ArrivedFirst) {
                    if (!asking) {
                        asking = true;
                        a.who = GivesWay::Ask;
                        a.by = SettledBy::ArrivedFirst;
                    }
                    continue;
                }
                if (asking) {
                    a.fallback = who;
                    a.fallbackBy = static_cast<SettledBy>(q);
                } else {
                    a.who = who;
                    a.by = static_cast<SettledBy>(q);
                }
                decided = true;
                break;
            }
            if (!decided) {
                // Nothing told them apart: the lower path id (side A: a < other for two paths) gives way.
                if (asking) {
                    a.fallback = GivesWay::A;
                    a.fallbackBy = SettledBy::LowerPathId;
                } else {
                    a.who = GivesWay::A;
                    a.by = SettledBy::LowerPathId;
                }
                anyPathId = true;
            }
            anyAsk = anyAsk || asking;
            out.answers.push_back(a);
        }
        if (anyAsk) ++out.counts.askWhileDriving;
        else ++out.counts.settledAtLoad;
        if (anyPathId) {
            ++out.counts.byLowerPathId;
            out.notes.push_back("junction " + std::to_string(j) + ": nothing in its settle list tells connector " +
                                std::to_string(rec.a.index) + " from connector " + std::to_string(o.index) +
                                " apart; the lower path id (" + std::to_string(rec.a.index) + ") gives way");
        }
    }
    return out;
}

MyTurn myTurn(const LaneGraph& roads, const MeetingAnswer& a, CrossingId crossing, PathId path, PathId* otherPath) {
    if (!crossing.valid() || crossing.index >= roads.crossings.size()) return MyTurn::Neither;
    const CrossingPoint& rec = roads.crossings[crossing.index];
    const bool iAmA = rec.a == path;
    if (otherPath) *otherPath = iAmA ? rec.otherPath() : rec.a;
    switch (a.who) {
    case GivesWay::Neither: return MyTurn::Neither;
    case GivesWay::Ask: return MyTurn::Ask;
    case GivesWay::A: return iAmA ? MyTurn::GiveWay : MyTurn::GoFirst;
    case GivesWay::Other: return iAmA ? MyTurn::GoFirst : MyTurn::GiveWay;
    }
    return MyTurn::Neither;
}

// ---- Words -------------------------------------------------------------------------------------------------------------

const char* movementPhrase(Movement m) {
    switch (m) {
    case Movement::Straight: return "straight on";
    case Movement::NearTurn: return "the near turn";
    case Movement::FarTurn: return "the far turn";
    case Movement::UTurn: return "the U-turn";
    }
    return "a turn";
}

std::string fromWords(const Vec2& heading) {
    // Where the traffic comes from is behind its direction of travel. On the plan, east is +x and north is -z (the
    // plan's second coordinate).
    const Real east = -heading.x, north = heading.y;
    if (east == 0 && north == 0) return "nowhere";
    const Real degrees = std::atan2(north, east) * kDegreesPerRadian;
    int point = static_cast<int>(std::lround(degrees / kCompassPointDegrees)) % kCompassPoints;
    if (point < 0) point += kCompassPoints;
    return std::string("the ") + kCompassWords[point];
}

const char* settledByWords(SettledBy by) {
    switch (by) {
    case SettledBy::Light: return "by the lights";
    case SettledBy::SignRank: return "by the signs";
    case SettledBy::Movement: return "by movement";
    case SettledBy::KerbSide: return "by who comes from the other's kerb side";
    case SettledBy::ArrivedFirst: return "by who reached the line first";
    case SettledBy::LowerPathId: return "by the lower path id (nothing else told them apart)";
    case SettledBy::Split: return "by shape: they leave one lane, and the car behind follows";
    case SettledBy::AcrossJunctions: return "while driving: they meet across two junctions, with no control to ask";
    }
    return "";
}

std::string pathTurnWords(const LaneGraph& roads, const RightOfWay& row, PathId p) {
    if (!p.valid() || p.index >= roads.paths.size() || p.index >= row.paths.size())
        return "path " + std::to_string(p.index);
    const engine::Path& path = roads.paths[p.index];
    if (path.kind != engine::PathKind::Connector) return "lane " + std::to_string(p.index);
    const ConnectorRules& cr = row.paths[p.index];
    std::string out = movementPhrase(cr.movement);
    if (cr.approach < row.approaches.size()) out += " from " + fromWords(row.approaches[cr.approach].heading);
    return out + " (connector " + std::to_string(p.index) + ")";
}

namespace {

std::string one(Real v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

const char* meetingWord(engine::Meeting m) {
    switch (m) {
    case engine::Meeting::Cross: return "cross";
    case engine::Meeting::Join: return "join";
    case engine::Meeting::Split: return "split";
    }
    return "meeting";
}

std::string answerWords(const LaneGraph& roads, const RightOfWay& row, const CrossingPoint& rec, const MeetingAnswer& a) {
    const std::string ta = pathTurnWords(roads, row, rec.a), to = pathTurnWords(roads, row, rec.otherPath());
    switch (a.who) {
    case GivesWay::Neither: return std::string("nobody gives way, ") + settledByWords(a.by);
    case GivesWay::A: return ta + " gives way to " + to + ", " + settledByWords(a.by);
    case GivesWay::Other: return to + " gives way to " + ta + ", " + settledByWords(a.by);
    case GivesWay::Ask: {
        std::string out = std::string("asked while driving (") + settledByWords(a.by) + "); if that ties, ";
        out += a.fallback == GivesWay::A ? ta + " gives way to " + to : to + " gives way to " + ta;
        return out + ", " + settledByWords(a.fallbackBy);
    }
    }
    return "";
}

}  // namespace

std::string describeJunctionRules(const LaneGraph& roads, const RuleSet& rules, const RightOfWay& row, JunctionId j) {
    if (!j.valid() || j.index >= roads.junctions.size() || j.index >= row.junctions.size())
        return "no such junction: " + std::to_string(j.index) + "\n";
    const engine::Junction& jn = roads.junctions[j.index];
    const JunctionRules& jr = row.junctions[j.index];
    std::string out = "junction " + std::to_string(j.index) + ": ";
    if (jr.control == kNoControl) {
        out += "no control the rule set knows";
    } else {
        const ControlKind& ck = rules.controls[jr.control];
        const std::string planned = jn.control < roads.controlNames.size() ? roads.controlNames[jn.control] : std::string();
        if (!planned.empty() && planned != ck.name) out += "\"" + planned + "\", its lights dark: treated as ";
        out += "control \"" + ck.name + "\" (settle:";
        for (const SettleQuestion q : ck.settle) out += std::string(" ") + settleQuestionWord(q);
        out += ck.everyoneStops ? "; everyone stops first)" : ")";
    }
    out += ", " + std::to_string(jr.approaches.count) + " approaches, " + std::to_string(jn.connectors.count) +
           " connectors, " + std::to_string(jr.phaseCount()) + (jr.phaseCount() == 1 ? " phase\n" : " phases\n");
    for (uint32_t a = 0; a < jr.approaches.count; ++a) {
        const Approach& ap = row.approaches[jr.approaches.first + a];
        out += "  approach " + std::to_string(a) + ": from " + fromWords(ap.heading) + ", lane " +
               std::to_string(ap.firstLane.index) + (ap.signRank > 0 && static_cast<std::size_t>(ap.signRank) <
                                                                            rules.ranks.size()
                                                         ? ", sign \"" + rules.ranks[static_cast<std::size_t>(ap.signRank)] + "\""
                                                         : std::string()) +
               "\n";
    }
    for (uint32_t ph = 0; ph < jr.phases.count; ++ph) {
        const Phase& p = row.phases[jr.phases.first + ph];
        out += "  phase " + std::to_string(ph) + (p.arrow ? " (arrow)" : "") + ": green:";
        std::string redTurn;
        for (uint32_t i = 0; i < jn.connectors.count; ++i) {
            const Light l = row.lights[p.lights.first + i];
            const PathId c = roads.junctionConnectors[jn.connectors.first + i];
            if (l == Light::Green) out += " " + std::to_string(c.index);
            else if (l == Light::RedMayTurn) redTurn += " " + std::to_string(c.index);
        }
        if (!redTurn.empty()) out += "; red, may turn:" + redTurn;
        if (jr.running())
            out += "; green " + one(p.green) + " s, then yellow " + one(p.yellow) + " s and all-red " + one(p.allRed) + " s";
        out += "\n";
    }
    if (jr.running()) out += "  cycle " + one(jr.cycle) + " s, offset " + one(jr.offset) + " s\n";
    for (uint32_t c = jn.crossings.first; c < jn.crossings.end() && c < roads.crossings.size(); ++c) {
        const CrossingPoint& rec = roads.crossings[c];
        if (rec.otherSide != engine::OtherSide::Path || row.answersOf[c].count == 0) continue;
        out += "  " + std::string(meetingWord(rec.meeting)) + " " + std::to_string(c) + ", connectors " +
               std::to_string(rec.a.index) + " (" + one(rec.aFrom) + "-" + one(rec.aTo) + " m) and " +
               std::to_string(rec.other) + " (" + one(rec.otherFrom) + "-" + one(rec.otherTo) + " m):";
        const Range answers = row.answersOf[c];
        if (answers.count == 1) {
            out += " " + answerWords(roads, row, rec, row.answers[answers.first]) + "\n";
            continue;
        }
        out += "\n";
        for (uint32_t ph = 0; ph < answers.count; ++ph)
            out += "    phase " + std::to_string(ph) + ": " +
                   answerWords(roads, row, rec, row.answers[answers.first + ph]) + "\n";
    }
    return out;
}

std::string describeRightOfWay(const RightOfWay& row) {
    const RightOfWayCounts& n = row.counts;
    std::string out = "right of way: " + std::to_string(n.junctions) + " junctions (" +
                      std::to_string(n.junctionsWithLights) + " with lights, " + std::to_string(n.phases) +
                      " phases among them; " + std::to_string(n.junctionsDark) +
                      " with lights treated as another control while dark; " + std::to_string(n.junctionsWithoutControl) +
                      " whose control the rule set lacks); " + std::to_string(n.meetings) + " meetings of two paths: " +
                      std::to_string(n.settledAtLoad) + " settled at load, " + std::to_string(n.askWhileDriving) +
                      " asked while driving (" + std::to_string(n.acrossJunctions) + " of them across two junctions), " +
                      std::to_string(n.splits) + " splits; " + std::to_string(n.byLowerPathId) +
                      " left in some part to the lower path id\n";
    if (n.junctionsTimed > 0)
        out += "  lights running a programme: " + std::to_string(n.junctionsTimed) + " junctions, cycles " + one(n.shortestCycle) +
               "-" + one(n.longestCycle) + " s; " + std::to_string(n.waitInsidePoints) +
               " far turns may wait inside on their green (" + std::to_string(n.waitInsideDropped) +
               " places the builder marked refused at load)\n";
    for (std::size_t i = 0; i < row.notes.size() && i < kNotesShown; ++i) out += "  " + row.notes[i] + "\n";
    if (row.notes.size() > kNotesShown)
        out += "  ... and " + std::to_string(row.notes.size() - kNotesShown) + " more notes\n";
    return out;
}

}  // namespace driving
