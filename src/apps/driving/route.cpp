#include "apps/driving/route.h"

#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace driving {

namespace {

using engine::LaneGraph;
using engine::Path;
using engine::PathId;
using engine::PathKind;
using engine::Section;
using engine::SectionId;

constexpr Real kForever = std::numeric_limits<Real>::infinity();
constexpr uint32_t kNoNode = 0xFFFFFFFFu;
// How far a share of a lane's length may stray past a leg's ends and still count as inside it (rounding only).
constexpr Real kShareTolerance = 1e-9;
// How far a distance along may stray past the start or the destination and still count (rounding only).
constexpr Real kAlongTolerance = 1e-6;

bool knownPath(const LaneGraph& g, PathId p) { return p.valid() && p.index < g.paths.size(); }

bool inRange(const engine::Range& r, std::size_t size) { return r.first <= size && r.count <= size - r.first; }

// Whether a lane lies in a section that lists it where it says it is, so its neighbours can be read.
bool inSoundSection(const LaneGraph& g, PathId lane) {
    if (!knownPath(g, lane)) return false;
    const Path& p = g.paths[lane.index];
    if (p.kind != PathKind::Lane || !p.section.valid() || p.section.index >= g.sections.size()) return false;
    const Section& s = g.sections[p.section.index];
    return inRange(s.lanes, g.sectionLanes.size()) && p.indexInSection < s.lanes.count &&
           g.sectionLanes[s.lanes.first + p.indexInSection] == lane;
}

// The lane beside `lane`, toward the kerb or toward the offside, when the line between may be crossed from this lane's
// side; none otherwise (no such lane, a solid line, or no sound section). Line i lies between lane i (offside) and lane
// i + 1 (kerb side).
PathId sidestep(const LaneGraph& g, PathId lane, bool towardKerb) {
    if (!inSoundSection(g, lane)) return PathId();
    const Path& p = g.paths[lane.index];
    const Section& s = g.sections[p.section.index];
    if (!inRange(s.lines, g.laneLines.size()) || s.lines.count + 1 != s.lanes.count) return PathId();
    const uint32_t k = p.indexInSection;
    if (towardKerb) {
        if (k + 1 >= s.lanes.count || !g.laneLines[s.lines.first + k].crossFromOffside) return PathId();
        return g.sectionLanes[s.lanes.first + k + 1];
    }
    if (k == 0 || !g.laneLines[s.lines.first + k - 1].crossFromKerbside) return PathId();
    return g.sectionLanes[s.lanes.first + k - 1];
}

// The paths leaving a path's end, in its next list's order (only known ones).
template <typename Fn> void forEachNext(const LaneGraph& g, PathId p, Fn fn) {
    const engine::Range next = g.paths[p.index].next;
    if (!inRange(next, g.nextPaths.size())) return;
    for (uint32_t i = next.first; i < next.end(); ++i)
        if (knownPath(g, g.nextPaths[i])) fn(g.nextPaths[i]);
}

// The one lane a connector joins, or none.
PathId laneAfter(const LaneGraph& g, PathId connector) {
    PathId lane;
    forEachNext(g, connector, [&](PathId q) {
        if (!lane.valid() && g.paths[q.index].kind == PathKind::Lane) lane = q;
    });
    return lane;
}

// The seconds driving the whole of a path takes: a learned time for its section when the table has one, else its
// length at the free-flow speed. Below zero: it cannot be driven (no speed at all).
Real pathSeconds(const LaneGraph& g, const RouteClass& cls, PathId p, const SectionSeconds* learned) {
    const Path& path = g.paths[p.index];
    if (learned && path.kind == PathKind::Lane && path.section.valid() && path.section.index < learned->size() &&
        (*learned)[path.section.index] > 0)
        return (*learned)[path.section.index];
    const Real speed = freeFlowSpeed(g, cls, p);
    return speed > 0 ? path.length / speed : Real(-1);
}

std::string pointWords(const PathPoint& p) {
    return "path " + std::to_string(p.path.index) + " at " + std::to_string(p.along) + " m";
}

bool onRoad(const LaneGraph& g, const PathPoint& p) {
    return knownPath(g, p.path) && p.along >= 0 && p.along <= g.paths[p.path.index].length;
}

// Where in a leg's lanes `lane` is, or -1.
int laneSlot(const Route& r, const RouteLeg& leg, PathId lane) {
    for (uint32_t k = 0; k < leg.laneCount; ++k)
        if (r.lanes[leg.firstLane + k].lane == lane) return static_cast<int>(k);
    return -1;
}

// The changes still needed from `lane` as leg `leg` lists it; kCannotFinish when the leg does not have it.
uint32_t changesIn(const Route& r, uint32_t leg, PathId lane) {
    const int k = laneSlot(r, r.legs[leg], lane);
    return k < 0 ? kCannotFinish : r.lanes[r.legs[leg].firstLane + static_cast<uint32_t>(k)].changes;
}

// The first leg at or after `fromLeg` that has `lane`, or -1.
int legWithLane(const Route& r, PathId lane, uint32_t fromLeg) {
    for (std::size_t i = fromLeg; i < r.legs.size(); ++i)
        if (laneSlot(r, r.legs[i], lane) >= 0) return static_cast<int>(i);
    return -1;
}

// How a car leaving `lane` (in leg `leg`, not the last) leads on into the next leg: the step it takes out of the
// lane's end (a connector, or the next leg's lane itself) whose lane in the next leg needs the fewest changes still
// (the lower path id on a tie). None when the lane does not lead on.
PathId bestStepOn(const Route& r, const LaneGraph& g, uint32_t leg, PathId lane, uint32_t* changes) {
    PathId best;
    uint32_t bestChanges = kCannotFinish;
    forEachNext(g, lane, [&](PathId q) {
        PathId into = q;
        if (g.paths[q.index].kind == PathKind::Connector) {
            if (!mayUse(g, r.vehicle, q)) return;
            into = laneAfter(g, q);
            if (!into.valid()) return;
        }
        const uint32_t c = changesIn(r, leg + 1, into);
        if (c == kCannotFinish) return;
        if (c < bestChanges || (c == bestChanges && q < best)) {
            bestChanges = c;
            best = q;
        }
    });
    if (changes) *changes = bestChanges;
    return best;
}

// ---- the search ------------------------------------------------------------------------------------------------

// One entry of the search's heap: a node, the time to reach it, and that plus the lower bound from it.
struct HeapItem {
    Real estimate = 0;
    Real cost = 0;
    uint32_t node = 0;
};
// The heap keeps the LOWEST estimate on top; a tie goes to the lower node number (for a lane, its path id).
bool heapAfter(const HeapItem& a, const HeapItem& b) {
    if (a.estimate != b.estimate) return a.estimate > b.estimate;
    return a.node > b.node;
}

Route noRoute(Route r, NoRouteWhy why, const std::string& words) {
    r.found = false;
    r.why = why;
    r.whyWords = words;
    return r;
}

}  // namespace

// ---- classes and paths -------------------------------------------------------------------------------------------

RouteClass bindRouteClass(const LaneGraph& roads, const VehicleClass& cls, std::vector<std::string>* notes) {
    RouteClass rc;
    rc.tightestTurnRadius = cls.tightestTurnRadius;
    rc.topSpeed = cls.topSpeed;
    for (const std::string& tag : cls.tags) {
        const engine::NameIndex bit = LaneGraph::nameIndex(roads.vehicleTagNames, tag);
        if (bit == engine::kNoName || bit >= engine::kMaxTagNames) {
            if (notes)
                notes->push_back("vehicle class '" + cls.name + "': the road knows no vehicle tag '" + tag +
                                 "', so the class matches no path by it");
            continue;
        }
        rc.tags |= engine::TagMask(1) << bit;
    }
    return rc;
}

bool mayUse(const LaneGraph& roads, const RouteClass& cls, PathId path) {
    if (!knownPath(roads, path)) return false;
    const Path& p = roads.paths[path.index];
    return (p.vehicleTags & cls.tags) != 0 && p.tightestRadius >= cls.tightestTurnRadius;
}

Real freeFlowSpeed(const LaneGraph& roads, const RouteClass& cls, PathId path) {
    if (!knownPath(roads, path)) return 0;
    const Real limit = roads.paths[path.index].speedLimit;
    if (limit <= 0) return cls.topSpeed;
    return cls.topSpeed > 0 ? std::min(limit, cls.topSpeed) : limit;
}

RouteBounds studyRoads(const LaneGraph& roads) {
    RouteBounds b;
    if (roads.paths.empty()) return b;
    b.shortestPath = kForever;
    const std::size_t points = std::min(roads.points.size(), roads.along.size());
    for (uint32_t i = 0; i < roads.paths.size(); ++i) {
        const Path& p = roads.paths[i];
        b.fastestLimit = std::max(b.fastestLimit, p.speedLimit);
        if (p.speedLimit <= 0) b.anyUnlimited = true;
        b.shortestPath = std::min(b.shortestPath, p.length);
        if (p.curve.count == 0 || !inRange(p.curve, points)) continue;
        const engine::Vec2 end = roads.points[p.curve.end() - 1];
        forEachNext(roads, PathId(i), [&](PathId q) {
            const engine::Range c = roads.paths[q.index].curve;
            if (c.count == 0 || !inRange(c, points)) return;
            b.widestGap = std::max(b.widestGap, engine::distance(end, roads.points[c.first]));
        });
    }
    for (const Section& s : roads.sections) {
        if (!inRange(s.lanes, roads.sectionLanes.size())) continue;
        for (uint32_t k = 1; k < s.lanes.count; ++k) {
            const PathId a = roads.sectionLanes[s.lanes.first + k - 1];
            const PathId c = roads.sectionLanes[s.lanes.first + k];
            if (!knownPath(roads, a) || !knownPath(roads, c)) continue;
            b.widestSidestep = std::max(b.widestSidestep, engine::distance(roads.pointAt(a, 0), roads.pointAt(c, 0)));
        }
    }
    b.usable = b.shortestPath > 0 && std::isfinite(b.shortestPath);
    return b;
}

// ---- finding a route ---------------------------------------------------------------------------------------------

Route findRoute(const LaneGraph& roads, const RouteBounds& bounds, const RouteClass& cls, const PathPoint& from,
                const PathPoint& to, const SectionSeconds* sectionSeconds) {
    Route r;
    r.from = from;
    r.to = to;
    r.vehicle = cls;
    if (!onRoad(roads, from))
        return noRoute(r, NoRouteWhy::StartNotOnRoad, "the car's place (" + pointWords(from) + ") is not on the road");
    if (!onRoad(roads, to))
        return noRoute(r, NoRouteWhy::DestinationNotOnRoad, "the destination (" + pointWords(to) + ") is not on the road");
    if (to.path != from.path && !mayUse(roads, cls, to.path))
        return noRoute(r, NoRouteWhy::DestinationClosed,
                       "the destination (" + pointWords(to) +
                           ") is on a path this vehicle may not use: its tags do not match, or it bends tighter than "
                           "the vehicle can turn");

    // THE NODES. Node i < n is "path i, from its start". After them come the START nodes: the car's own path from
    // where its front is, and (on a lane in a section) each lane beside it from the same share of its length, since a
    // car may change lanes before it has driven anywhere. Last comes the destination.
    const uint32_t n = static_cast<uint32_t>(roads.paths.size());
    const Path& startPath = roads.paths[from.path.index];
    const Real startShare = startPath.length > 0 ? from.along / startPath.length : Real(0);
    std::vector<PathId> startNodes;
    uint32_t startSlot = 0;
    const bool startInSection = inSoundSection(roads, from.path);
    if (startInSection) {
        const Section& s = roads.sections[startPath.section.index];
        for (uint32_t k = 0; k < s.lanes.count; ++k) startNodes.push_back(roads.sectionLanes[s.lanes.first + k]);
        startSlot = startPath.indexInSection;
    } else {
        startNodes.push_back(from.path);
    }
    const uint32_t startCount = static_cast<uint32_t>(startNodes.size());
    const uint32_t destNode = n + startCount;
    const uint32_t nodeCount = destNode + 1;
    auto nodePath = [&](uint32_t u) { return u < n ? PathId(u) : (u < destNode ? startNodes[u - n] : to.path); };
    auto nodeEntry = [&](uint32_t u) -> Real {
        if (u < n) return 0;
        if (u >= destNode) return to.along;
        const PathId p = startNodes[u - n];
        return p == from.path ? from.along : startShare * roads.paths[p.index].length;
    };

    // THE LOWER BOUND (A*): the straight distance to the destination, less the widest gap between joined paths, over
    // the fastest any step can go, slowed by the share a gap can add to each path's length. Every step a route takes
    // either drives along a path (at most `fastest` metres a second, and a path is at least shortestPath long) or
    // jumps a gap between paths (at most widestGap, once per path), or changes lanes (kLaneChangeCostSeconds for a
    // shift no wider than `fastest` times that), so the bound never exceeds what remains. The first step, from part
    // way along the start path, may be short; the subtracted gap covers it.
    Real fastest = 0;
    if (bounds.usable) {
        fastest = cls.topSpeed > 0 ? std::min(bounds.fastestLimit, cls.topSpeed) : bounds.fastestLimit;
        if (bounds.anyUnlimited) fastest = std::max(fastest, cls.topSpeed);
        if (sectionSeconds) {
            for (uint32_t s = 0; s < sectionSeconds->size() && s < roads.sections.size(); ++s) {
                const Real secs = (*sectionSeconds)[s];
                const Section& sec = roads.sections[s];
                if (!(secs > 0) || !inRange(sec.lanes, roads.sectionLanes.size())) continue;
                for (uint32_t k = 0; k < sec.lanes.count; ++k) {
                    const PathId l = roads.sectionLanes[sec.lanes.first + k];
                    if (knownPath(roads, l)) fastest = std::max(fastest, roads.paths[l.index].length / secs);
                }
            }
        }
        Real sidestepWidth = bounds.widestSidestep;
        for (uint32_t k = 1; k < startCount; ++k)
            sidestepWidth = std::max(sidestepWidth, engine::distance(roads.pointAt(startNodes[k - 1], nodeEntry(n + k - 1)),
                                                                     roads.pointAt(startNodes[k], nodeEntry(n + k))));
        fastest = std::max(fastest, sidestepWidth / kLaneChangeCostSeconds);
    }
    const bool useBound = bounds.usable && fastest > 0;
    const Real boundScale = useBound ? 1 / (fastest * (1 + bounds.widestGap / bounds.shortestPath)) : 0;
    const engine::Vec2 destPoint = roads.pointAt(to.path, to.along);
    auto bound = [&](uint32_t u) -> Real {
        if (!useBound || u == destNode) return 0;
        const Real d = engine::distance(roads.pointAt(nodePath(u), nodeEntry(u)), destPoint) - bounds.widestGap;
        return d > 0 ? d * boundScale : Real(0);
    };

    std::vector<Real> best(nodeCount, kForever);
    std::vector<uint32_t> pred(nodeCount, kNoNode);
    std::vector<uint8_t> bySidestep(nodeCount, 0);
    std::vector<HeapItem> heap;
    auto reach = [&](uint32_t v, uint32_t from_, Real cost, bool sideways) {
        if (!(cost < best[v])) return;
        best[v] = cost;
        pred[v] = from_;
        bySidestep[v] = sideways ? 1 : 0;
        HeapItem item;
        item.cost = cost;
        item.estimate = cost + bound(v);
        item.node = v;
        heap.push_back(item);
        std::push_heap(heap.begin(), heap.end(), heapAfter);
    };
    reach(n + startSlot, kNoNode, 0, false);

    bool arrived = false;
    while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), heapAfter);
        const HeapItem item = heap.back();
        heap.pop_back();
        if (item.cost > best[item.node]) continue;   // a stale entry: the node was reached cheaper since
        const uint32_t u = item.node;
        if (u == destNode) {
            arrived = true;
            break;
        }
        const PathId p = nodePath(u);
        const Path& path = roads.paths[p.index];
        const Real entry = nodeEntry(u);
        const Real whole = pathSeconds(roads, cls, p, sectionSeconds);
        if (whole >= 0 && path.length > 0) {
            const Real perMetre = whole / path.length;
            // The destination, when it lies ahead on this path.
            if (p == to.path && to.along + kAlongTolerance >= entry)
                reach(destNode, u, item.cost + perMetre * std::max(Real(0), to.along - entry), false);
            // On along the path's end.
            const Real toEnd = item.cost + perMetre * (path.length - entry);
            forEachNext(roads, p, [&](PathId q) {
                if (mayUse(roads, cls, q)) reach(q.index, u, toEnd, false);
            });
        }
        // Sideways, into the lane beside (a start node goes to the start node beside it), when the class may.
        if (cls.laneChanges && path.kind == PathKind::Lane) {
            for (const bool towardKerb : {false, true}) {
                const PathId beside = sidestep(roads, p, towardKerb);
                if (!beside.valid() || !mayUse(roads, cls, beside)) continue;
                const uint32_t v = u < n ? beside.index : n + roads.paths[beside.index].indexInSection;
                if (u >= n && !startInSection) continue;
                reach(v, u, item.cost + kLaneChangeCostSeconds, true);
            }
        }
    }
    if (!arrived)
        return noRoute(r, NoRouteWhy::NothingLeadsThere,
                       "nothing this vehicle may use leads from " + pointWords(from) + " to " + pointWords(to));
    r.cost = best[destNode];

    // THE NODES TAKEN, start to destination (not counting the destination node itself).
    std::vector<uint32_t> taken;
    for (uint32_t u = pred[destNode]; u != kNoNode; u = pred[u]) taken.push_back(u);
    std::reverse(taken.begin(), taken.end());

    // INTO LEGS AND TURNS: lanes of one section joined by lane changes are one leg; a connector (or a lane leading
    // straight into the next section's lane) is the turn between two legs; a connector first or last is the start or
    // end connector.
    PathId pendingConnector;
    for (std::size_t i = 0; i < taken.size(); ++i) {
        const uint32_t u = taken[i];
        const PathId p = nodePath(u);
        const Path& path = roads.paths[p.index];
        if (path.kind == PathKind::Connector) {
            if (i == 0) r.startConnector = p;
            if (i + 1 == taken.size()) r.endConnector = p;
            else if (i > 0) pendingConnector = p;
            continue;
        }
        if (i > 0 && bySidestep[u]) {
            r.legs.back().lane = p;
            continue;
        }
        RouteLeg leg;
        leg.section = inSoundSection(roads, p) ? path.section : SectionId();
        leg.lane = p;
        leg.enterFraction = path.length > 0 ? nodeEntry(u) / path.length : Real(0);
        if (!r.legs.empty()) {
            RouteTurn t;
            t.fromSection = r.legs.back().section;
            t.toSection = leg.section;
            if (pendingConnector.valid()) {
                const Path& c = roads.paths[pendingConnector.index];
                t.turn = c.turn;
                t.junction = c.junction;
                t.connector = pendingConnector;
                t.metres = c.length;
                const Real speed = freeFlowSpeed(roads, cls, pendingConnector);
                t.seconds = speed > 0 ? c.length / speed : Real(0);
            }
            r.turns.push_back(t);
            pendingConnector = PathId();
        }
        r.legs.push_back(leg);
    }

    // Each leg's lanes, where the route comes out of it, and its length.
    for (std::size_t i = 0; i < r.legs.size(); ++i) {
        RouteLeg& leg = r.legs[i];
        leg.firstLane = static_cast<uint32_t>(r.lanes.size());
        if (leg.section.valid()) {
            const Section& s = roads.sections[leg.section.index];
            for (uint32_t k = 0; k < s.lanes.count; ++k) {
                RouteLane rl;
                rl.lane = roads.sectionLanes[s.lanes.first + k];
                rl.length = knownPath(roads, rl.lane) ? roads.paths[rl.lane.index].length : Real(0);
                r.lanes.push_back(rl);
            }
        } else {
            RouteLane rl;
            rl.lane = leg.lane;
            rl.length = roads.paths[leg.lane.index].length;
            r.lanes.push_back(rl);
        }
        leg.laneCount = static_cast<uint32_t>(r.lanes.size()) - leg.firstLane;
        const Real length = roads.paths[leg.lane.index].length;
        if (i + 1 == r.legs.size() && !r.endConnector.valid())
            leg.leaveFraction = length > 0 ? to.along / length : Real(1);
        leg.metres = std::max(Real(0), leg.leaveFraction - leg.enterFraction) * length;
        const Real speed = freeFlowSpeed(roads, cls, leg.lane);
        leg.seconds = speed > 0 ? leg.metres / speed : Real(0);
    }

    // The start and end connectors' share, then each leg's "after it", from the destination back.
    Real startMetres = 0, startSeconds = 0, endMetres = 0, endSeconds = 0;
    if (r.startConnector.valid()) {
        const bool alsoEnd = r.endConnector == r.startConnector && r.legs.empty();
        startMetres = (alsoEnd ? to.along : roads.paths[r.startConnector.index].length) - from.along;
        const Real speed = freeFlowSpeed(roads, cls, r.startConnector);
        startSeconds = speed > 0 ? startMetres / speed : Real(0);
    }
    if (r.endConnector.valid() && !r.legs.empty()) {
        endMetres = to.along;
        const Real speed = freeFlowSpeed(roads, cls, r.endConnector);
        endSeconds = speed > 0 ? endMetres / speed : Real(0);
    }
    Real afterMetres = endMetres, afterSeconds = endSeconds;
    for (std::size_t i = r.legs.size(); i-- > 0;) {
        r.legs[i].metresAfter = afterMetres;
        r.legs[i].secondsAfter = afterSeconds;
        afterMetres += r.legs[i].metres;
        afterSeconds += r.legs[i].seconds;
        if (i > 0) {
            afterMetres += r.turns[i - 1].metres;
            afterSeconds += r.turns[i - 1].seconds;
        }
    }
    r.metres = startMetres + afterMetres;
    r.seconds = startSeconds + afterSeconds;

    // THE LANE-CHANGES TABLE, worked out backwards from the destination. In the last leg a lane finishes when the
    // destination is on it (or it leads into the end connector); in any other leg, when it leads into the next leg at
    // a lane that can finish, and then it needs what that lane needs. Then, within the leg, a lane needs one more
    // change than a lane beside it that it may cross into.
    for (std::size_t i = r.legs.size(); i-- > 0;) {
        const RouteLeg& leg = r.legs[i];
        const bool last = i + 1 == r.legs.size();
        for (uint32_t k = 0; k < leg.laneCount; ++k) {
            RouteLane& rl = r.lanes[leg.firstLane + k];
            const bool usable = mayUse(roads, cls, rl.lane) || rl.lane == from.path;
            uint32_t staying = kCannotFinish;
            if (usable && knownPath(roads, rl.lane)) {
                if (last) {
                    const bool finishes = r.endConnector.valid() ? roads.paths[r.endConnector.index].from == rl.lane
                                                                 : rl.lane == to.path;
                    if (finishes) staying = 0;
                } else {
                    bestStepOn(r, roads, static_cast<uint32_t>(i), rl.lane, &staying);
                }
            }
            rl.changesIfStaying = staying;
            rl.changes = staying;
        }
        bool changed = true;
        while (changed) {
            changed = false;
            for (uint32_t k = 0; k < leg.laneCount; ++k) {
                RouteLane& rl = r.lanes[leg.firstLane + k];
                for (const bool towardKerb : {false, true}) {
                    const PathId beside = sidestep(roads, rl.lane, towardKerb);
                    if (!beside.valid()) continue;
                    const int slot = laneSlot(r, leg, beside);
                    if (slot < 0) continue;
                    const uint32_t there = r.lanes[leg.firstLane + static_cast<uint32_t>(slot)].changes;
                    if (there != kCannotFinish && there + 1 < rl.changes) {
                        rl.changes = there + 1;
                        changed = true;
                    }
                }
            }
        }
    }

    r.found = true;
    return r;
}

// ---- questions ---------------------------------------------------------------------------------------------------

RoutePlace placeOnRoute(const Route& route, const LaneGraph& roads, const PathPoint& at, uint32_t fromLeg) {
    RoutePlace place;
    if (!route.found || !knownPath(roads, at.path)) return place;
    const Path& p = roads.paths[at.path.index];
    if (p.kind == PathKind::Lane) {
        const Real share = p.length > 0 ? at.along / p.length : Real(0);
        for (std::size_t i = fromLeg; i < route.legs.size(); ++i) {
            const RouteLeg& leg = route.legs[i];
            if (laneSlot(route, leg, at.path) < 0) continue;
            if (share + kShareTolerance < leg.enterFraction || share > leg.leaveFraction + kShareTolerance) continue;
            place.part = RoutePart::Leg;
            place.index = static_cast<uint32_t>(i);
            return place;
        }
        return place;
    }
    const bool onlyConnector = route.legs.empty();
    if (fromLeg == 0 && at.path == route.startConnector && at.along + kAlongTolerance >= route.from.along &&
        (!onlyConnector || at.along <= route.to.along + kAlongTolerance)) {
        place.part = RoutePart::StartConnector;
        return place;
    }
    if (!onlyConnector && at.path == route.endConnector && at.along <= route.to.along + kAlongTolerance) {
        place.part = RoutePart::EndConnector;
        return place;
    }
    const PathId into = laneAfter(roads, at.path);
    for (std::size_t i = fromLeg; i < route.turns.size(); ++i) {
        if (laneSlot(route, route.legs[i], p.from) < 0 || laneSlot(route, route.legs[i + 1], into) < 0) continue;
        place.part = RoutePart::Turn;
        place.index = static_cast<uint32_t>(i);
        return place;
    }
    return place;
}

LanesToCross lanesToCross(const Route& route, PathId lane, uint32_t fromLeg) {
    LanesToCross out;
    if (!route.found) return out;
    const int legIndex = legWithLane(route, lane, fromLeg);
    if (legIndex < 0) return out;
    out.onRoute = true;
    const RouteLeg& leg = route.legs[static_cast<std::size_t>(legIndex)];
    const uint32_t k = static_cast<uint32_t>(laneSlot(route, leg, lane));
    const uint32_t mine = route.lanes[leg.firstLane + k].changes;
    for (uint32_t d = 1; d < leg.laneCount; ++d) {
        const uint32_t kerbward = k + d < leg.laneCount ? route.lanes[leg.firstLane + k + d].changes : kCannotFinish;
        const uint32_t offward = k >= d ? route.lanes[leg.firstLane + k - d].changes : kCannotFinish;
        const bool kerbBetter = kerbward < mine;
        const bool offBetter = offward < mine;
        if (!kerbBetter && !offBetter) continue;
        out.lanes = d;
        out.toward = kerbBetter && (!offBetter || kerbward <= offward) ? Across::TowardKerb : Across::TowardOffside;
        return out;
    }
    return out;
}

LastChance lastChance(const Route& route, PathId lane, Real along, uint32_t fromLeg) {
    LastChance out;
    if (!route.found) return out;
    const int legIndex = legWithLane(route, lane, fromLeg);
    if (legIndex < 0) return out;
    const RouteLeg& leg = route.legs[static_cast<std::size_t>(legIndex)];
    const RouteLane& rl = route.lanes[leg.firstLane + static_cast<uint32_t>(laneSlot(route, leg, lane))];
    if (rl.changesIfStaying != kCannotFinish) return out;
    out.needed = true;
    out.metres = std::max(Real(0), leg.leaveFraction * rl.length - along);
    return out;
}

WayAhead wayAhead(const Route& route, const LaneGraph& roads, PathId path, Real along, Real lookAhead,
                  uint32_t fromLeg) {
    WayAhead way;
    if (!knownPath(roads, path)) return way;
    way.paths.push_back(path);
    PathPoint here;
    here.path = path;
    here.along = along;
    RoutePlace place = placeOnRoute(route, roads, here, fromLeg);
    if (!place.onRoute()) return way;
    Real reach = roads.paths[path.index].length - along;
    PathId at = path;
    while (reach < lookAhead && way.paths.size() < kMaxFollowedPaths) {
        PathId next;
        RoutePlace nextPlace;
        if (place.part == RoutePart::StartConnector || place.part == RoutePart::Turn) {
            const uint32_t leg = place.part == RoutePart::Turn ? place.index + 1 : 0;
            if (leg >= route.legs.size()) break;   // the destination is on this connector
            next = laneAfter(roads, at);
            if (!next.valid() || laneSlot(route, route.legs[leg], next) < 0) break;
            nextPlace.part = RoutePart::Leg;
            nextPlace.index = leg;
        } else if (place.part == RoutePart::Leg) {
            const uint32_t leg = place.index;
            if (leg + 1 == route.legs.size()) {
                // The last leg: on to the end connector when the destination is on it, else the destination is here.
                if (!route.endConnector.valid() || roads.paths[route.endConnector.index].from != at) break;
                next = route.endConnector;
                nextPlace.part = RoutePart::EndConnector;
            } else {
                next = bestStepOn(route, roads, leg, at, nullptr);
                if (!next.valid()) break;   // this lane does not lead on: the way ends with it
                nextPlace.part = roads.paths[next.index].kind == PathKind::Connector ? RoutePart::Turn : RoutePart::Leg;
                nextPlace.index = nextPlace.part == RoutePart::Turn ? leg : leg + 1;
            }
        } else {
            break;   // on the end connector: the destination is on it
        }
        way.paths.push_back(next);
        reach += roads.paths[next.index].length;
        at = next;
        place = nextPlace;
    }
    return way;
}

Remaining remainingFrom(const Route& route, const LaneGraph& roads, const PathPoint& at, uint32_t fromLeg) {
    Remaining out;
    const RoutePlace place = placeOnRoute(route, roads, at, fromLeg);
    if (!place.onRoute()) return out;
    out.onRoute = true;
    const Path& p = roads.paths[at.path.index];
    const Real speed = freeFlowSpeed(roads, route.vehicle, at.path);
    auto add = [&](Real metres, Real seconds) {
        out.metres += metres;
        out.seconds += seconds;
    };
    auto driveHere = [&](Real metres) { add(metres, speed > 0 ? metres / speed : Real(0)); };
    switch (place.part) {
    case RoutePart::Leg: {
        const RouteLeg& leg = route.legs[place.index];
        const Real share = p.length > 0 ? at.along / p.length : Real(0);
        driveHere(std::max(Real(0), leg.leaveFraction - share) * p.length);
        add(leg.metresAfter, leg.secondsAfter);
        break;
    }
    case RoutePart::Turn: {
        const RouteLeg& next = route.legs[place.index + 1];
        driveHere(std::max(Real(0), p.length - at.along));
        add(next.metres + next.metresAfter, next.seconds + next.secondsAfter);
        break;
    }
    case RoutePart::StartConnector:
        if (route.legs.empty()) {
            driveHere(std::max(Real(0), route.to.along - at.along));
        } else {
            driveHere(std::max(Real(0), p.length - at.along));
            add(route.legs[0].metres + route.legs[0].metresAfter, route.legs[0].seconds + route.legs[0].secondsAfter);
        }
        break;
    case RoutePart::EndConnector:
        driveHere(std::max(Real(0), route.to.along - at.along));
        break;
    case RoutePart::Off:
        break;
    }
    return out;
}

uint64_t routeHash(const Route& route) {
    engine::bundle::Fnv h;
    h.pod(static_cast<uint8_t>(route.found));
    h.pod(static_cast<uint8_t>(route.why));
    h.str(route.whyWords);
    h.pod(route.from.path.index);
    h.pod(route.from.along);
    h.pod(route.to.path.index);
    h.pod(route.to.along);
    h.pod(route.vehicle.tags);
    h.pod(route.vehicle.tightestTurnRadius);
    h.pod(route.vehicle.topSpeed);
    h.pod(static_cast<uint8_t>(route.vehicle.laneChanges));
    h.pod(route.startConnector.index);
    h.pod(route.endConnector.index);
    h.pod(static_cast<uint64_t>(route.legs.size()));
    for (const RouteLeg& l : route.legs) {
        h.pod(l.section.index);
        h.pod(l.lane.index);
        h.pod(l.enterFraction);
        h.pod(l.leaveFraction);
        h.pod(l.firstLane);
        h.pod(l.laneCount);
        h.pod(l.metres);
        h.pod(l.seconds);
        h.pod(l.metresAfter);
        h.pod(l.secondsAfter);
    }
    h.pod(static_cast<uint64_t>(route.turns.size()));
    for (const RouteTurn& t : route.turns) {
        h.pod(static_cast<uint8_t>(t.turn));
        h.pod(t.junction.index);
        h.pod(t.fromSection.index);
        h.pod(t.toSection.index);
        h.pod(t.connector.index);
        h.pod(t.metres);
        h.pod(t.seconds);
    }
    h.pod(static_cast<uint64_t>(route.lanes.size()));
    for (const RouteLane& l : route.lanes) {
        h.pod(l.lane.index);
        h.pod(l.length);
        h.pod(l.changes);
        h.pod(l.changesIfStaying);
    }
    h.pod(route.metres);
    h.pod(route.seconds);
    h.pod(route.cost);
    return h.h;
}

}  // namespace driving
