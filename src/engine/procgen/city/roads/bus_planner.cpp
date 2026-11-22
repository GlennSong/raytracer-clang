#include "engine/procgen/city/roads/bus_planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

namespace engine::roads {

namespace {

// A piece of a loop the bus drives: a lane from `a` to `b` metres along it, or a turn through a junction (no lane).
struct Piece {
    PathId lane;     // not valid: a turn
    Real a = 0, b = 0;
    Real metres = 0;
};

}  // namespace

BusPlan planBusLines(const RoadModel& m, const RouteClass& bus, const BusPlanParams& params) {
    BusPlan plan;
    const LaneGraph& g = m.laneLevel;
    if (!m.built || g.paths.empty()) {
        plan.notes.push_back("the road model has no lane level: no line can be laid");
        return plan;
    }
    // A STREET A BUS SERVES: carries cars, is no freeway or ramp, and has a sidewalk on a kerb
    const auto servedStreet = [&](uint32_t s) {
        if (s >= m.streets.size()) return false;
        const ModelSection& sec = m.streets[s].section;
        return sec.carriesCars() && sec.klass != RoadClass::Freeway && sec.klass != RoadClass::Ramp && sec.sidewalk > 0;
    };
    // A LANE A STOP MAY STAND BESIDE: on such a street, along its spine, with a sidewalk on the kerb of its way, and open
    // to the bus
    const auto kerbOf = [&](const Path& p) { return p.alongSpine() ? 1 : -1; };
    // ...and the KERB LANE of its section (offside to kerb: the last), the one a bus pulls in on (RoadModel::busStopLane):
    // a bus that may not change lanes (the driving cannot yet: RouteClass::laneChanges) must already be in it
    const auto kerbLane = [&](const Path& p) {
        if (!p.section.valid() || p.section.index >= g.sections.size()) return false;
        const Range lanes = g.sections[p.section.index].lanes;
        return lanes.count > 0 && p.indexInSection + 1 == lanes.count;
    };
    const auto stopLane = [&](PathId id) {
        if (!id.valid() || id.index >= g.paths.size()) return false;
        const Path& p = g.paths[id.index];
        return p.kind == PathKind::Lane && p.onSpine && kerbLane(p) && servedStreet(p.street) &&
               m.streets[p.street].section.sidewalkOnKerb(kerbOf(p)) && mayUse(g, bus, id);
    };

    // ---- 1. HUBS ------------------------------------------------------------------------------------------------------
    Real servedLength = 0;
    for (uint32_t s = 0; s < m.streets.size(); ++s)
        if (servedStreet(s)) servedLength += m.streets[s].to - m.streets[s].from;
    // THE PART OF THE ROADS A BUS CAN DRIVE ROUND (lane_graph.h reachableParts, over the paths open to it): a hub
    // anywhere else could be driven into and not out of (a car turns round nowhere: a dead end's lanes are a part of
    // their own), so the hubs are all in the largest part
    ReachRules openToBus;
    openToBus.laneChanges = bus.laneChanges;
    openToBus.open.assign(g.paths.size(), 0);
    for (uint32_t i = 0; i < g.paths.size(); ++i) openToBus.open[i] = mayUse(g, bus, PathId(i)) ? 1 : 0;
    const ReachableParts parts = reachableParts(g, openToBus);
    // where a bus can stand near a junction: the lanes a stop may stand beside, at a junctionClear from their end
    std::vector<PathPoint> standing;
    std::size_t outside = 0;
    for (uint32_t i = 0; i < g.paths.size(); ++i) {
        if (stopLane(PathId(i)) && i < parts.partOf.size() && parts.partOf[i] != parts.largest) ++outside;
        if (stopLane(PathId(i)) && g.paths[i].length > 2 * params.junctionClear && i < parts.partOf.size() &&
            parts.partOf[i] == parts.largest) {
            PathPoint pp;
            pp.path = PathId(i);
            pp.along = g.paths[i].length - params.junctionClear;
            standing.push_back(pp);
        }
    }
    if (outside > 0)
        plan.notes.push_back(std::to_string(outside) + " lanes a stop could stand beside lie outside the largest part of the roads "
                             "a bus can drive round (of " + std::to_string(parts.sizes.empty() ? 0u : parts.sizes[parts.largest]) +
                             " paths): no hub is put there");
    if (standing.size() < 2) {
        plan.notes.push_back("fewer than two lanes a bus could stand beside: no line can be laid");
        return plan;
    }
    Vec2 middle(0, 0);
    for (const PathPoint& p : standing) middle = middle + g.pointAt(p.path, p.along);
    middle = middle * (Real(1) / static_cast<Real>(standing.size()));
    const int want = std::clamp(static_cast<int>(std::lround(servedLength / std::max(Real(1), params.hubEvery))), params.fewestHubs,
                                params.mostHubs);
    std::vector<PathPoint> hubs;
    {
        // the first: nearest the middle; each next: the farthest from those already picked
        std::size_t first = 0;
        Real best = 1e30;
        for (std::size_t i = 0; i < standing.size(); ++i) {
            const Real d = (g.pointAt(standing[i].path, standing[i].along) - middle).length();
            if (d < best) {
                best = d;
                first = i;
            }
        }
        hubs.push_back(standing[first]);
        while (static_cast<int>(hubs.size()) < want) {
            std::size_t far = 0;
            Real farD = -1;
            for (std::size_t i = 0; i < standing.size(); ++i) {
                Real nearest = 1e30;
                const Vec2 p = g.pointAt(standing[i].path, standing[i].along);
                for (const PathPoint& h : hubs) nearest = std::min(nearest, (p - g.pointAt(h.path, h.along)).length());
                if (nearest > farD) {
                    farD = nearest;
                    far = i;
                }
            }
            if (farD < params.spacing) break;   // (a small town: no hub that far from the rest)
            hubs.push_back(standing[far]);
        }
    }
    // in their order round the town's middle
    std::sort(hubs.begin(), hubs.end(), [&](const PathPoint& x, const PathPoint& y) {
        const Vec2 a = g.pointAt(x.path, x.along) - middle, b = g.pointAt(y.path, y.along) - middle;
        const Real ta = std::atan2(a.y, a.x), tb = std::atan2(b.y, b.x);
        if (ta != tb) return ta < tb;
        return x.path.index < y.path.index;
    });
    plan.hubs = static_cast<int>(hubs.size());

    // ---- 2. LINES: rings through the hubs, each leaving one out, routes spread by the reuse price --------------------------
    const RouteBounds bounds = studyRoads(m);
    std::vector<int> uses(g.sections.size(), 0);
    SectionSeconds priced(g.sections.size(), 0);
    const auto reprice = [&]() {
        for (std::size_t s = 0; s < g.sections.size(); ++s) {
            if (uses[s] == 0) {
                priced[s] = 0;
                continue;
            }
            // the section's time at its first lane's free-flow speed, times the reuse price
            Real secs = 0;
            for (uint32_t i = 0; i < g.paths.size() && secs <= 0; ++i)
                if (g.paths[i].kind == PathKind::Lane && g.paths[i].section.index == s) {
                    const Real v = freeFlowSpeed(m, bus, PathId(i));
                    secs = v > 0 ? g.paths[i].length / v : 0;
                }
            priced[s] = secs * (1 + params.reusePrice * uses[s]);
        }
    };
    const int lineCount = params.lines > 0 ? params.lines : std::min(3, std::max(1, plan.hubs - 1));
    std::set<uint32_t> drivenStreets;
    std::map<std::string, int> stopNames;   // (a name already on the line gets a number)
    for (int k = 0; k < lineCount; ++k) {
        std::vector<PathPoint> ring;
        for (int h = 0; h < plan.hubs; ++h)
            if (plan.hubs < 3 || h != k % plan.hubs) ring.push_back(hubs[static_cast<std::size_t>(h)]);
        if (ring.size() < 2) continue;
        // the loop, leg by leg (a hub no route reaches from the one before is left out)
        std::vector<Piece> loop;
        std::vector<Route> legs;
        std::vector<PathPoint> kept{ring.front()};
        for (std::size_t i = 1; i <= ring.size(); ++i) {
            const PathPoint& to = ring[i % ring.size()];
            reprice();
            Route r = findRoute(m, bounds, bus, kept.back(), to, &priced);
            if (!r.found) {
                if (i < ring.size()) {
                    plan.notes.push_back("line " + std::to_string(k + 1) + ": no route from one hub to the next (" + r.whyWords +
                                         "); that hub is left out");
                    continue;
                }
                plan.notes.push_back("line " + std::to_string(k + 1) + ": no route round to its first hub (" + r.whyWords +
                                     "); the line is not laid");
                legs.clear();
                break;
            }
            for (const RouteLeg& L : r.legs)
                if (L.section.valid() && L.section.index < uses.size()) ++uses[L.section.index];
            legs.push_back(std::move(r));
            if (i < ring.size()) kept.push_back(to);
        }
        if (legs.empty() || kept.size() < 2) continue;
        for (const Route& r : legs) {
            for (std::size_t i = 0; i < r.legs.size(); ++i) {
                const RouteLeg& L = r.legs[i];
                if (L.lane.valid() && L.lane.index < g.paths.size()) {
                    const Real len = g.paths[L.lane.index].length;
                    loop.push_back({L.lane, L.enterFraction * len, L.leaveFraction * len, L.metres});
                    if (g.paths[L.lane.index].street != kNoIndex) drivenStreets.insert(g.paths[L.lane.index].street);
                }
                if (i < r.turns.size()) loop.push_back({PathId(), 0, 0, r.turns[i].metres});
            }
        }
        // ---- 3. STOPS along the loop ----------------------------------------------------------------------------------
        ModelBusLine line;
        line.id = params.idPrefix + std::to_string(k + 1);
        line.name = "Line " + std::to_string(k + 1);
            Real since = params.spacing;   // (the loop's first good place is its first stop)
        for (const Piece& pc : loop) {
            if (!pc.lane.valid() || !stopLane(pc.lane)) {
                since += pc.metres;
                continue;
            }
            const Path& p = g.paths[pc.lane.index];
            const Real lo = std::max(pc.a, params.junctionClear), hi = std::min(pc.b, p.length - params.junctionClear);
            Real s = pc.a;
            const Real step = 5;
            for (; s <= pc.b; s += step) {
                if (since >= params.spacing && s >= lo && s <= hi) {
                    ModelBusStop stop;
                    const Real station = p.stationFrom + (p.stationTo - p.stationFrom) * (s / std::max(p.length, kSamePoint));
                    stop.street = ModelStreetId(p.street);
                    stop.streetId = m.streets[p.street].id;
                    stop.station = station;
                    stop.kerb = kerbOf(p);
                    std::string name = streetName(m, stop.street);
                    if (name.empty()) name = stop.streetId;
                    const int n = ++stopNames[line.id + "|" + name];
                    stop.name = n > 1 ? name + " " + std::to_string(n) : name;
                    char id[32];
                    std::snprintf(id, sizeof(id), "%s-%02zu", line.id.c_str(), line.stops.size() + 1);
                    stop.id = id;
                    line.stops.push_back(stop);
                    since = 0;
                }
                since += step;
            }
            since -= (s - pc.b);   // (the last step ran past the piece's end)
        }
        // the last stop too near the first round the loop: left out
        if (line.stops.size() > 2 && since < params.spacing * Real(0.5)) {
            line.stops.pop_back();
        }
        // ---- 4. CHECKED: each stop to the next, from the kerb lane beside it, routes for the bus ------------------------
        const auto legDrives = [&](const ModelBusStop& a, const ModelBusStop& b) {
            const KerbLanePlace ka = m.busStopLane(a), kb = m.busStopLane(b);
            if (!ka.valid() || !kb.valid()) return false;
            PathPoint pa, pb;
            pa.path = ka.path;
            pa.along = ka.along;
            pb.path = kb.path;
            pb.along = kb.along;
            return findRoute(m, bounds, bus, pa, pb).found;
        };
        // the first leg that cannot be driven loses its far stop; again until every leg drives
        for (bool dropped = true; dropped && line.stops.size() >= 2;) {
            dropped = false;
            for (std::size_t i = 0; i < line.stops.size(); ++i) {
                const std::size_t j = (i + 1) % line.stops.size();
                if (legDrives(line.stops[i], line.stops[j])) continue;
                plan.notes.push_back("line " + line.id + ": stop " + line.stops[j].id + " cannot be driven to from " +
                                     line.stops[i].id + "; left out");
                line.stops.erase(line.stops.begin() + static_cast<std::ptrdiff_t>(j));
                dropped = true;
                break;
            }
        }
        if (line.stops.size() < 2) {
            plan.notes.push_back("line " + line.id + ": fewer than two stops left; not laid");
            continue;
        }
        // renumbered, in order
        for (std::size_t i = 0; i < line.stops.size(); ++i) {
            char id[32];
            std::snprintf(id, sizeof(id), "%s-%02zu", line.id.c_str(), i + 1);
            line.stops[i].id = id;
        }
        plan.lines.push_back(std::move(line));
    }

    // ---- THE NUMBERS: coverage and street share over the served streets ------------------------------------------------
    std::vector<Vec2> stopsAt;
    for (const ModelBusLine& L : plan.lines)
        for (const ModelBusStop& s : L.stops) {
            const KerbLanePlace k = m.busStopLane(s);
            if (k.valid()) stopsAt.push_back(k.at);
        }
    Real covered = 0, driven = 0, total = 0;
    for (uint32_t s = 0; s < m.streets.size(); ++s) {
        if (!servedStreet(s)) continue;
        const ModelStreet& S = m.streets[s];
        const Real len = S.to - S.from;
        total += len;
        if (drivenStreets.count(s)) driven += len;
        if (S.line.count < 2) continue;
        for (uint32_t i = S.line.first; i + 1 < S.line.end(); ++i) {
            const Vec2 a = m.linePoints[i], b = m.linePoints[i + 1];
            const Real seg = (b - a).length();
            const int n = std::max(1, static_cast<int>(std::ceil(seg / 10.0)));
            for (int j = 0; j < n; ++j) {
                const Vec2 p = a + (b - a) * ((j + Real(0.5)) / n);
                bool near = false;
                for (const Vec2& q : stopsAt)
                    if ((p - q).length() <= params.reach) { near = true; break; }
                if (near) covered += seg / n;
            }
        }
    }
    plan.coverage = total > 0 ? covered / total : 0;
    plan.streetShare = total > 0 ? driven / total : 0;
    return plan;
}

nlohmann::json busLinesJson(const std::vector<ModelBusLine>& lines) {
    nlohmann::json out = nlohmann::json::array();
    for (const ModelBusLine& L : lines) {
        nlohmann::json stops = nlohmann::json::array();
        for (const ModelBusStop& s : L.stops) {
            const double station = std::round(s.station * 100.0) / 100.0;   // (to the centimetre, as the file states stations)
            stops.push_back({{"id", s.id}, {"name", s.name}, {"street", s.streetId}, {"station", station}, {"kerb", s.kerb}});
        }
        out.push_back({{"id", L.id}, {"name", L.name}, {"stops", stops}});
    }
    return out;
}

}  // namespace engine::roads
