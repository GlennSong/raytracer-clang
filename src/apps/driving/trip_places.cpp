#include "apps/driving/trip_places.h"

#include "hash.h"

#include <algorithm>
#include <cmath>

namespace driving {

namespace {

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

// The lane holding the `metres`-th usable metre of `part`, and the place on it.
PathPoint placeAtMetres(const TripPart& part, Real margin, Real metres) {
    PathPoint p;
    if (part.lanes.empty() || !(part.total > 0)) return p;
    // the last lane whose stretch begins at or before `metres`
    const std::vector<Real>::const_iterator it = std::upper_bound(part.before.begin(), part.before.end(), metres);
    const std::size_t i = it == part.before.begin() ? 0 : static_cast<std::size_t>(it - part.before.begin()) - 1;
    p.path = part.lanes[i];
    p.along = margin + (metres - part.before[i]);
    return p;
}

Real usableLength(const engine::LaneGraph& roads, engine::PathId lane, Real margin) {
    return roads.paths[lane.index].length - 2 * margin;
}

}  // namespace

uint32_t TripPlaces::partOf(engine::PathId path) const {
    return path.valid() && path.index < partOfPath.size() ? partOfPath[path.index] : kNoTripPart;
}

TripPlaces findTripPlaces(const engine::LaneGraph& roads, const RouteClass& cls, Real margin, std::size_t smallestPart) {
    TripPlaces out;
    out.margin = margin;
    const std::size_t n = roads.paths.size();
    out.partOfPath.assign(n, kNoTripPart);
    engine::ReachRules rules;
    rules.laneChanges = cls.laneChanges;
    rules.open.assign(n, 0);
    for (std::size_t p = 0; p < n; ++p) rules.open[p] = mayUse(roads, cls, engine::PathId(static_cast<uint32_t>(p))) ? 1 : 0;
    const engine::ReachableParts parts = engine::reachableParts(roads, rules);

    // Each part's usable lanes, in path id order (parts are numbered by their lowest path, so walking the paths in
    // order meets the parts in order too).
    std::vector<std::vector<engine::PathId>> lanesOf(parts.sizes.size());
    for (std::size_t p = 0; p < n; ++p) {
        const engine::PathId id(static_cast<uint32_t>(p));
        if (!rules.open[p] || roads.paths[p].kind != engine::PathKind::Lane) continue;
        if (!(usableLength(roads, id, margin) > 0)) continue;
        lanesOf[parts.partOf[p]].push_back(id);
    }
    std::vector<uint32_t> indexOfPart(parts.sizes.size(), kNoTripPart);
    for (std::size_t k = 0; k < lanesOf.size(); ++k) {
        if (lanesOf[k].size() < smallestPart || lanesOf[k].empty()) continue;
        // A part of one path goes round only when that path leads into itself; otherwise it is a dead end, or a road
        // nothing leads into.
        if (parts.sizes[k] < 2) {
            const std::vector<engine::PathId> steps = engine::stepsFrom(roads, lanesOf[k][0], rules);
            if (std::find(steps.begin(), steps.end(), lanesOf[k][0]) == steps.end()) continue;
        }
        TripPart tp;
        tp.lanes = lanesOf[k];
        for (const engine::PathId lane : tp.lanes) {
            tp.before.push_back(tp.total);
            tp.total += usableLength(roads, lane, margin);
        }
        indexOfPart[k] = static_cast<uint32_t>(out.parts.size());
        out.partBefore.push_back(out.total);
        out.total += tp.total;
        out.laneCount += tp.lanes.size();
        out.parts.push_back(std::move(tp));
    }
    // Every path of a kept part (its connectors too) knows its part, so a car anywhere in it is sent within it.
    for (std::size_t p = 0; p < n; ++p)
        if (rules.open[p]) out.partOfPath[p] = indexOfPart[parts.partOf[p]];
    return out;
}

PathPoint placeAnywhere(const TripPlaces& places, Real unit) {
    if (places.empty()) return PathPoint();
    const Real metres = std::clamp(unit, Real(0), Real(1)) * places.total;
    const std::vector<Real>::const_iterator it = std::upper_bound(places.partBefore.begin(), places.partBefore.end(), metres);
    const std::size_t k = it == places.partBefore.begin() ? 0 : static_cast<std::size_t>(it - places.partBefore.begin()) - 1;
    return placeAtMetres(places.parts[k], places.margin, std::min(metres - places.partBefore[k], places.parts[k].total));
}

PathPoint placeInPart(const TripPlaces& places, uint32_t part, Real unit) {
    if (part >= places.parts.size()) return PathPoint();
    const TripPart& tp = places.parts[part];
    return placeAtMetres(tp, places.margin, std::clamp(unit, Real(0), Real(1)) * tp.total);
}

Real placeDraw(uint32_t seed, PersonId person, PlaceQuestion question, uint32_t draw) {
    const uint32_t who = engine::hashPair(static_cast<uint32_t>(person), static_cast<uint32_t>(person >> 32));
    const uint32_t what = engine::hashPair(seed, static_cast<uint32_t>(question));
    return static_cast<Real>(engine::unit24(engine::hashCell(who, draw, what)));
}

bool roomFor(const engine::LaneGraph& roads, const PathLists& lists, const PathPoint& front, Real length, Real gap) {
    if (!knownPath(roads, front.path)) return false;
    const Real rear = front.along - length;
    if (rear < gap || front.along > roads.paths[front.path.index].length - gap) return false;
    return carsOnStretch(lists, front.path, rear - gap, front.along + gap).empty();
}

TripPlacing putCarSomewhere(Driving& driving, const TripPlaces& places, CarId car, uint32_t vehicleClass, Real gap,
                            uint32_t seed, PersonId person, uint32_t& draw, uint32_t maxTries) {
    TripPlacing r;
    const engine::LaneGraph* roads = driving.roads();
    if (!roads || places.empty() || vehicleClass >= driving.vehicleClasses().size() || driving.car(car)) return r;
    const Real length = driving.vehicleClasses().at(vehicleClass).length;
    while (r.tries < maxTries) {
        ++r.tries;
        const PathPoint at = placeAnywhere(places, placeDraw(seed, person, PlaceQuestion::Start, draw++));
        if (!roomFor(*roads, driving.pathLists(), at, length, gap)) continue;
        if (driving.placeCar(car, vehicleClass, at)) {
            r.done = true;
            break;
        }
    }
    return r;
}

TripPlacing sendCarSomewhere(Driving& driving, const TripPlaces& places, CarId car, PersonId person, uint32_t seed,
                             uint32_t& draw, uint32_t maxTries) {
    TripPlacing r;
    const Car* c = driving.car(car);
    if (!c || driving.tripOf(car).valid()) return r;
    const uint32_t part = places.partOf(c->path);
    if (part == kNoTripPart) return r;
    const PathPoint here{c->path, c->along};
    while (r.tries < maxTries) {
        ++r.tries;
        const PathPoint to = placeInPart(places, part, placeDraw(seed, person, PlaceQuestion::Destination, draw++));
        // Not where the car already stands: that trip would be over before it began.
        if (to.path == here.path && std::fabs(to.along - here.along) < places.margin) continue;
        if (driving.beginTrip(car, person, to).valid()) {
            r.done = true;
            break;
        }
        ++r.noRoute;   // the car is placed and free and the place is on the road: only a missing route refuses it
    }
    return r;
}

}  // namespace driving
