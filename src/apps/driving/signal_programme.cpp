#include "apps/driving/signal_programme.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace driving {

namespace {

constexpr Real kTwo = 2;   // the 2s in the yellow formula

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

std::string one(Real v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

Lamp lampOf(Light l) {
    switch (l) {
    case Light::Green: return Lamp::Green;
    case Light::Red: return Lamp::Red;
    case Light::RedMayTurn: return Lamp::RedMayTurn;
    }
    return Lamp::Red;
}

// The light connector `i` (its place in the junction) shows in phase `ph` of `jr`.
Light lightIn(const RightOfWay& row, const JunctionRules& jr, uint32_t ph, uint32_t i) {
    const Phase& p = row.phases[jr.phases.first + ph];
    return i < p.lights.count ? row.lights[p.lights.first + i] : Light::Red;
}

// How far along connector `c` it has crossed everything it crosses: where its last crossing stretch ends -- the ITE
// formula's "far side of the farthest conflicting lane" (joins and splits left out: past them the car is one following
// another in a lane). 0 when it crosses nothing.
Real clearDistance(const engine::LaneGraph& roads, engine::PathId c) {
    const engine::Path& p = roads.paths[c.index];
    Real reach = 0;
    for (uint32_t k = p.crossings.first; k < p.crossings.end() && k < roads.pathCrossings.size(); ++k) {
        const engine::CrossingPoint& x = roads.crossings[roads.pathCrossings[k].index];
        if (x.meeting != engine::Meeting::Cross) continue;
        reach = std::max(reach, x.a == c ? x.aTo : x.otherTo);
    }
    return reach;
}

// Where a junction lies on the plan: the middle of its outline's points (the origin for one with none).
engine::Vec2 junctionMiddle(const engine::LaneGraph& roads, const engine::Junction& jn) {
    engine::Vec2 sum(0, 0);
    uint32_t n = 0;
    for (uint32_t k = jn.box.first; k < jn.box.end() && k < roads.planPoints.size(); ++k, ++n) sum = sum + roads.planPoints[k];
    return n > 0 ? sum * (Real(1) / static_cast<Real>(n)) : sum;
}

}  // namespace

// ---- The rule set ----------------------------------------------------------------------------------------------------

bool registerLightClock(FactRegistry& registry, std::vector<std::string>* problems) {
    ReaderSpec reader;
    reader.name = kLightClockReaderName;
    reader.meaning = "times every junction's traffic lights at load (green, yellow, all-red, cycle, offset), and tells a "
                     "driver at a yellow whether it can stop";
    reader.numbers.push_back(need(kNumberSignalReaction, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberSignalDeceleration, UnitFamily::Acceleration));
    reader.numbers.push_back(need(kNumberGravity, UnitFamily::Acceleration));
    reader.numbers.push_back(need(kNumberYellowShortest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberYellowLongest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberAllRedVehicleLength, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberAllRedShortest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberAllRedLongest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGreenShortest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGreenLongest, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberSignalCycle, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberWaveSpeed, UnitFamily::Speed));
    reader.numbers.push_back(need(kNumberWaveEast, UnitFamily::Ratio));
    reader.numbers.push_back(need(kNumberWaveNorth, UnitFamily::Ratio));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        if (problems) problems->push_back(why);
        return false;
    }
    return true;
}

bool hasLightClock(const FactRegistry& registry) { return registry.findReader(kLightClockReaderName) != nullptr; }

bool readLightTiming(const RuleSet& rules, LightTiming& out, std::string* why) {
    const std::pair<const char*, Real*> numbers[] = {
        {kNumberSignalReaction, &out.reaction},
        {kNumberSignalDeceleration, &out.deceleration},
        {kNumberGravity, &out.gravity},
        {kNumberYellowShortest, &out.yellowShortest},
        {kNumberYellowLongest, &out.yellowLongest},
        {kNumberAllRedVehicleLength, &out.vehicleLength},
        {kNumberAllRedShortest, &out.allRedShortest},
        {kNumberAllRedLongest, &out.allRedLongest},
        {kNumberGreenShortest, &out.greenShortest},
        {kNumberGreenLongest, &out.greenLongest},
        {kNumberSignalCycle, &out.cycle},
        {kNumberWaveSpeed, &out.waveSpeed},
        {kNumberWaveEast, &out.waveEast},
        {kNumberWaveNorth, &out.waveNorth},
    };
    for (const std::pair<const char*, Real*>& n : numbers) {
        const Number* found = rules.findNumber(n.first);
        if (!found) {
            if (why)
                *why = "rule set \"" + rules.name + "\" lacks the number \"" + n.first + "\", which " +
                       kLightClockReaderName + " reads";
            return false;
        }
        *n.second = static_cast<Real>(found->value);
    }
    return true;
}

// ---- The formulas ----------------------------------------------------------------------------------------------------

Real yellowSeconds(const LightTiming& t, Real approachSpeed, Real grade) {
    const Real bottom = kTwo * t.deceleration + kTwo * t.gravity * grade;
    const Real y = bottom > 0 ? t.reaction + std::max(Real(0), approachSpeed) / bottom : t.yellowLongest;
    return std::clamp(y, t.yellowShortest, std::max(t.yellowShortest, t.yellowLongest));
}

Real allRedSeconds(const LightTiming& t, Real clearDistance, Real speed) {
    const Real r = speed > 0 ? (std::max(Real(0), clearDistance) + t.vehicleLength) / speed : t.allRedLongest;
    return std::clamp(r, t.allRedShortest, std::max(t.allRedShortest, t.allRedLongest));
}

// ---- At load ---------------------------------------------------------------------------------------------------------

void timeSignalProgrammes(const engine::LaneGraph& roads, const RoadRules& roadRules, const LightTiming& t,
                          RightOfWay& row) {
    RightOfWayCounts& counts = row.counts;
    counts.junctionsTimed = 0;
    counts.shortestCycle = counts.longestCycle = 0;
    counts.waitInsidePoints = counts.waitInsideDropped = 0;
    for (ConnectorRules& cr : row.paths) cr.waitInsideAt = engine::kNoWaitInside;
    // The green wave's direction on the plan (east is +x, north is -y), as a unit vector; none when it has no length.
    const Real waveLength = std::sqrt(t.waveEast * t.waveEast + t.waveNorth * t.waveNorth);
    const bool wave = waveLength > 0 && t.waveSpeed > 0;

    for (uint32_t j = 0; j < roads.junctions.size() && j < row.junctions.size(); ++j) {
        JunctionRules& jr = row.junctions[j];
        jr.cycle = jr.offset = 0;
        const uint32_t n = jr.phases.count;
        if (n == 0) continue;
        const engine::Junction& jn = roads.junctions[j];
        // Each movement's yellow and all-red, should it lose its green.
        std::vector<Real> yellowOf(jn.connectors.count, 0), allRedOf(jn.connectors.count, 0);
        for (uint32_t i = 0; i < jn.connectors.count; ++i) {
            const engine::PathId c = roads.junctionConnectors[jn.connectors.first + i];
            const engine::PathId from = roads.paths[c.index].from;
            Real approach = knownPath(roads, from) ? roadRules.baseLimit(roads, from) : Real(0);
            if (!(approach > 0)) approach = roadRules.baseLimit(roads, c);
            const Real grade = knownPath(roads, from) ? roads.gradeAt(from, roads.paths[from.index].length) : Real(0);
            yellowOf[i] = yellowSeconds(t, approach, grade);
            allRedOf[i] = allRedSeconds(t, clearDistance(roads, c), approach);
        }
        Real lost = 0;
        for (uint32_t ph = 0; ph < n; ++ph) {
            Phase& p = row.phases[jr.phases.first + ph];
            const uint32_t next = (ph + 1) % n;
            p.yellow = p.allRed = 0;
            for (uint32_t i = 0; i < jn.connectors.count; ++i) {
                if (lightIn(row, jr, ph, i) != Light::Green || lightIn(row, jr, next, i) == Light::Green) continue;
                p.yellow = std::max(p.yellow, yellowOf[i]);
                p.allRed = std::max(p.allRed, allRedOf[i]);
            }
            lost += p.yellow + p.allRed;
        }
        // The greens share what the target cycle leaves, equally.
        const Real share = (t.cycle - lost) / static_cast<Real>(n);
        const Real green = std::clamp(share, t.greenShortest, std::max(t.greenShortest, t.greenLongest));
        for (uint32_t ph = 0; ph < n; ++ph) {
            Phase& p = row.phases[jr.phases.first + ph];
            p.green = green;
            jr.cycle += p.length();
        }
        if (wave) {
            // The phase the wave times: the first serving the approach that runs most nearly along the wave's line (either
            // way). Every junction then starts that phase as the wave passes, so two lights close together serve the
            // same road at the same time (their phases are numbered in no particular order of direction).
            const engine::Vec2 along(t.waveEast / waveLength, -t.waveNorth / waveLength);
            uint32_t waved = 0;
            Real best = -1;
            for (uint32_t ph = 0; ph < n; ++ph)
                for (uint32_t i = 0; i < jn.connectors.count; ++i) {
                    if (lightIn(row, jr, ph, i) != Light::Green) continue;
                    const engine::PathId c = roads.junctionConnectors[jn.connectors.first + i];
                    const uint32_t a = row.paths[c.index].approach;
                    if (a >= row.approaches.size()) continue;
                    const Real lined = std::fabs(engine::dot(row.approaches[a].heading, along));
                    if (lined > best) {
                        best = lined;
                        waved = ph;
                    }
                }
            Real wavedStarts = 0;
            for (uint32_t ph = 0; ph < waved; ++ph) wavedStarts += row.phases[jr.phases.first + ph].length();
            const engine::Vec2 mid = junctionMiddle(roads, jn);
            const Real reached = (mid.x * along.x + mid.y * along.y) / t.waveSpeed;
            const Real shift = std::fmod(reached - wavedStarts, jr.cycle);
            jr.offset = shift < 0 ? shift + jr.cycle : shift;
        }
        ++counts.junctionsTimed;
        counts.shortestCycle = counts.junctionsTimed == 1 ? jr.cycle : std::min(counts.shortestCycle, jr.cycle);
        counts.longestCycle = std::max(counts.longestCycle, jr.cycle);

        // Each far turn's place to wait inside: kept when, in every phase where it is green, it goes first at every
        // crossing point its body has reached by then.
        for (uint32_t i = 0; i < jn.connectors.count; ++i) {
            const engine::PathId c = roads.junctionConnectors[jn.connectors.first + i];
            const engine::Path& p = roads.paths[c.index];
            if (!(p.waitInsideAt > 0) || p.waitInsideAt > p.length) continue;
            bool ok = true;
            std::string why;
            for (uint32_t ph = 0; ph < n && ok; ++ph) {
                if (lightIn(row, jr, ph, i) != Light::Green) continue;
                for (uint32_t k = p.crossings.first; k < p.crossings.end() && k < roads.pathCrossings.size() && ok; ++k) {
                    const engine::CrossingId cid = roads.pathCrossings[k];
                    const engine::CrossingPoint& x = roads.crossings[cid.index];
                    if (x.otherSide != engine::OtherSide::Path || x.meeting == engine::Meeting::Split || x.junction.index != j)
                        continue;
                    if (!((x.a == c ? x.aFrom : x.otherFrom) < p.waitInsideAt)) continue;
                    const MeetingAnswer* a = row.answer(cid, ph);
                    engine::PathId other;
                    if (a && myTurn(roads, *a, cid, c, &other) == MyTurn::GoFirst) continue;
                    ok = false;
                    why = "junction " + std::to_string(j) + ": connector " + std::to_string(c.index) + " may not wait inside at " +
                          one(p.waitInsideAt) + " m: in phase " + std::to_string(ph) + " it does not go first at crossing point " +
                          std::to_string(cid.index) + " (with connector " + std::to_string(other.index) + "), which its body reaches";
                }
            }
            if (ok) {
                row.paths[c.index].waitInsideAt = p.waitInsideAt;
                ++counts.waitInsidePoints;
            } else {
                ++counts.waitInsideDropped;
                row.notes.push_back(why);
            }
        }
    }
}

// ---- At any moment ---------------------------------------------------------------------------------------------------

SignalNow signalAt(const RightOfWay& row, engine::JunctionId junction, double now) {
    SignalNow s;
    if (!junction.valid() || junction.index >= row.junctions.size()) return s;
    const JunctionRules& jr = row.junctions[junction.index];
    if (!jr.running()) return s;
    double tau = std::fmod(now - static_cast<double>(jr.offset), static_cast<double>(jr.cycle));
    if (tau < 0) tau += jr.cycle;
    s.running = true;
    for (uint32_t ph = 0; ph < jr.phases.count; ++ph) {
        const Phase& p = row.phases[jr.phases.first + ph];
        const bool last = ph + 1 == jr.phases.count;
        if (!(tau < p.length()) && !last) {
            tau -= p.length();
            continue;
        }
        s.phase = ph;
        if (tau < p.green) {
            s.stage = Stage::Green;
            s.into = static_cast<Real>(tau);
            s.remaining =p.green - s.into;
        } else if (tau < p.green + p.yellow) {
            s.stage = Stage::Yellow;
            s.into = static_cast<Real>(tau) - p.green;
            s.remaining =p.yellow - s.into;
        } else {
            s.stage = Stage::AllRed;
            s.into = static_cast<Real>(tau) - p.green - p.yellow;
            s.remaining =std::max(Real(0), p.allRed - s.into);
        }
        break;
    }
    return s;
}

uint32_t currentPhase(const RightOfWay& row, engine::JunctionId junction, double now) {
    const SignalNow s = signalAt(row, junction, now);
    return s.running ? s.phase : 0;
}

Lamp lampAt(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId connector, double now) {
    if (!knownPath(roads, connector) || connector.index >= row.paths.size()) return Lamp::Dark;
    const engine::Path& p = roads.paths[connector.index];
    if (p.kind != engine::PathKind::Connector || !p.junction.valid() || p.junction.index >= row.junctions.size())
        return Lamp::Dark;
    const JunctionRules& jr = row.junctions[p.junction.index];
    const SignalNow s = signalAt(row, p.junction, now);
    if (!s.running) return Lamp::Dark;
    const uint32_t i = row.paths[connector.index].indexInJunction;
    const Light mine = lightIn(row, jr, s.phase, i);
    if (s.stage == Stage::Green || mine != Light::Green) return lampOf(mine);
    const Light next = lightIn(row, jr, (s.phase + 1) % jr.phases.count, i);
    if (next == Light::Green) return Lamp::Green;   // green in the next phase too: it stays green
    if (s.stage == Stage::Yellow) return Lamp::Yellow;
    return next == Light::RedMayTurn ? Lamp::RedMayTurn : Lamp::Red;
}

std::vector<std::string> lampWords() { return {"green", "yellow", "red", "red_may_turn"}; }

const char* lampWord(Lamp l) {
    switch (l) {
    case Lamp::Dark: return "dark";
    case Lamp::Green: return "green";
    case Lamp::Yellow: return "yellow";
    case Lamp::Red: return "red";
    case Lamp::RedMayTurn: return "red_may_turn";
    }
    return "dark";
}

uint32_t lampWordIndex(Lamp l) {
    switch (l) {
    case Lamp::Green: return 0;
    case Lamp::Yellow: return 1;
    case Lamp::Red: return 2;
    case Lamp::RedMayTurn: return 3;
    case Lamp::Dark: break;
    }
    return kNoWord;
}

std::vector<std::string> checkSignalProgrammes(const engine::LaneGraph& roads, const RuleSet& rules, const RightOfWay& row) {
    std::vector<std::string> out;
    const auto letsIn = [](Lamp l) { return l == Lamp::Green || l == Lamp::Yellow; };
    for (uint32_t j = 0; j < roads.junctions.size() && j < row.junctions.size(); ++j) {
        const JunctionRules& jr = row.junctions[j];
        if (!jr.running()) continue;
        const ControlKind* ck = jr.control < rules.controls.size() ? &rules.controls[jr.control] : nullptr;
        const engine::Junction& jn = roads.junctions[j];
        double phaseStart = jr.offset;
        for (uint32_t ph = 0; ph < jr.phases.count; ++ph) {
            const Phase& p = row.phases[jr.phases.first + ph];
            const Real stageFrom[3] = {0, p.green, p.green + p.yellow};
            const Real stageLength[3] = {p.green, p.yellow, p.allRed};
            for (int s = 0; s < 3; ++s) {
                if (!(stageLength[s] > 0)) continue;
                const double at = phaseStart + stageFrom[s] + stageLength[s] / 2;
                for (uint32_t c = jn.crossings.first; c < jn.crossings.end() && c < roads.crossings.size(); ++c) {
                    const engine::CrossingPoint& x = roads.crossings[c];
                    if (x.otherSide != engine::OtherSide::Path || x.meeting == engine::Meeting::Split) continue;
                    const engine::PathId a = x.a, b = x.otherPath();
                    if (!letsIn(lampAt(roads, row, a, at)) || !letsIn(lampAt(roads, row, b, at))) continue;
                    const ConnectorRules& ra = row.paths[a.index];
                    const ConnectorRules& rb = row.paths[b.index];
                    if (ra.approach == rb.approach) continue;   // one approach's turns are green together by design
                    const bool facingEach = ra.approach < row.approaches.size() && rb.approach < row.approaches.size() &&
                                            engine::dot(row.approaches[ra.approach].heading,
                                                        row.approaches[rb.approach].heading) < kFacingDot;
                    const bool permitted = facingEach && ck &&
                                           (hasMovement(ck->givesWayOnGreen, ra.movement) ||
                                            hasMovement(ck->givesWayOnGreen, rb.movement));
                    if (permitted) continue;
                    static const char* const kStageWords[3] = {"green", "yellow", "all-red"};
                    out.push_back("junction " + std::to_string(j) + ", phase " + std::to_string(ph) + " (its " +
                                  kStageWords[s] + "): " + pathTurnWords(roads, row, a) + " and " +
                                  pathTurnWords(roads, row, b) + " both let traffic in, and their paths meet (crossing point " +
                                  std::to_string(c) + ")");
                }
            }
            phaseStart += p.length();
        }
    }
    return out;
}

std::string describeProgramme(const engine::LaneGraph& roads, const RightOfWay& row, engine::JunctionId junction) {
    if (!junction.valid() || junction.index >= row.junctions.size() || junction.index >= roads.junctions.size())
        return "no such junction: " + std::to_string(junction.index) + "\n";
    const JunctionRules& jr = row.junctions[junction.index];
    std::string out = "junction " + std::to_string(junction.index) + ": ";
    if (!jr.running()) return out + "no lights running a programme\n";
    out += std::to_string(jr.phases.count) + " phases, cycle " + one(jr.cycle) + " s, offset " + one(jr.offset) + " s\n";
    for (uint32_t ph = 0; ph < jr.phases.count; ++ph) {
        const Phase& p = row.phases[jr.phases.first + ph];
        out += "  phase " + std::to_string(ph) + (p.arrow ? " (arrow)" : "") + ": green " + one(p.green) + " s, yellow " +
               one(p.yellow) + " s, all-red " + one(p.allRed) + " s\n";
    }
    const engine::Junction& jn = roads.junctions[junction.index];
    for (uint32_t i = 0; i < jn.connectors.count; ++i) {
        const engine::PathId c = roads.junctionConnectors[jn.connectors.first + i];
        if (c.index < row.paths.size() && row.paths[c.index].waitInsideAt > 0)
            out += "  " + pathTurnWords(roads, row, c) + " may wait inside at " + one(row.paths[c.index].waitInsideAt) + " m\n";
    }
    return out;
}

}  // namespace driving
