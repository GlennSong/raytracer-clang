#include "apps/driving/traffic_state.h"

#include <algorithm>
#include <limits>
#include <tuple>

namespace driving {

namespace {

// Shorter than this, a car's body has no direction of its own (its rear and front are one point).
constexpr Real kShortestBody = 1e-9;

bool knownPath(const engine::LaneGraph& roads, engine::PathId path) {
    return path.valid() && path.index < roads.paths.size();
}

// The order of a path's entries: by front, then rear, then car id, then why -- whole, so it never depends on the order
// the records were stored in.
bool entryBefore(const PathEntry& a, const PathEntry& b) {
    return std::make_tuple(a.front, a.rear, a.car.index, static_cast<uint8_t>(a.why)) <
           std::make_tuple(b.front, b.rear, b.car.index, static_cast<uint8_t>(b.why));
}

// A list entry and the path it belongs to, while the lists are built.
struct Listed {
    uint32_t path = 0;
    PathEntry entry;
};

// A candidate nearer than `best` (a smaller gap, or the same gap and a lower car id)?
bool nearer(const PathEntry& e, Real gap, const Neighbour& best) {
    if (!best.found()) return true;
    return gap < best.gap || (gap == best.gap && e.car < best.car);
}

engine::Vec3 lift(const engine::Vec2& plan, Real height) { return engine::Vec3(plan.x, height, plan.y); }

}  // namespace

bool operator==(const PathEntry& a, const PathEntry& b) {
    return a.car == b.car && a.why == b.why && a.front == b.front && a.rear == b.rear;
}

PathList PathLists::on(engine::PathId path) const {
    PathList list;
    if (!path.valid() || static_cast<std::size_t>(path.index) + 1 >= firstOfPath.size()) return list;
    list.first = entries.data() + firstOfPath[path.index];
    list.count = firstOfPath[path.index + 1] - firstOfPath[path.index];
    return list;
}

bool operator==(const PathLists& a, const PathLists& b) {
    return a.entries == b.entries && a.firstOfPath == b.firstOfPath;
}

Body layBody(const engine::LaneGraph& roads, const Car& car, Real length) {
    Body body;
    if (!knownPath(roads, car.path)) {
        body.unplaced = length;
        return body;
    }
    Real remaining = length;
    // Lays the rest of the body back from `front` on `path`; what does not fit stays in `remaining`.
    const auto lay = [&](engine::PathId path, Real front) {
        BodyPiece piece;
        piece.path = path;
        piece.front = front;
        if (front >= remaining) {
            piece.rear = front - remaining;
            remaining = 0;
        } else {
            piece.rear = 0;
            remaining -= front;
        }
        body.pieces[body.count++] = piece;
    };
    lay(car.path, car.along);
    for (std::size_t k = 0; k < car.trailCount && k < kMaxTrail && remaining > 0; ++k) {
        if (!knownPath(roads, car.trail[k])) break;
        lay(car.trail[k], roads.paths[car.trail[k].index].length);
    }
    body.unplaced = remaining;
    return body;
}

void rebuildPathLists(const engine::LaneGraph& roads, const VehicleClasses& classes, const std::vector<Car>& cars,
                      PathLists& out) {
    std::vector<Listed> listed;
    listed.reserve(cars.size());
    for (const Car& car : cars) {
        if (car.vehicleClass >= classes.size() || !knownPath(roads, car.path)) continue;
        const Real length = classes.at(car.vehicleClass).length;
        const Body body = layBody(roads, car, length);
        for (std::size_t i = 0; i < body.count; ++i) {
            Listed l;
            l.path = body.pieces[i].path.index;
            l.entry.car = car.id;
            l.entry.why = i == 0 ? OnPath::Front : OnPath::Tail;
            l.entry.front = body.pieces[i].front;
            l.entry.rear = body.pieces[i].rear;
            listed.push_back(l);
        }
        if (knownPath(roads, car.besidePath) && car.besidePath != car.path) {
            const Real laneLength = roads.paths[car.besidePath.index].length;
            Listed l;
            l.path = car.besidePath.index;
            l.entry.car = car.id;
            l.entry.why = OnPath::Beside;
            l.entry.front = std::clamp(car.along, Real(0), laneLength);
            l.entry.rear = std::clamp(car.along - length, Real(0), laneLength);
            listed.push_back(l);
        }
    }
    std::sort(listed.begin(), listed.end(), [](const Listed& a, const Listed& b) {
        if (a.path != b.path) return a.path < b.path;
        return entryBefore(a.entry, b.entry);
    });

    const std::size_t pathCount = roads.paths.size();
    out.entries.clear();
    out.entries.reserve(listed.size());
    out.firstOfPath.assign(pathCount + 1, 0);
    for (const Listed& l : listed) {
        out.entries.push_back(l.entry);
        ++out.firstOfPath[l.path + 1];
    }
    for (std::size_t i = 0; i < pathCount; ++i) out.firstOfPath[i + 1] += out.firstOfPath[i];

    out.recordOfCar.resize(cars.size());
    for (std::size_t i = 0; i < cars.size(); ++i) out.recordOfCar[i] = static_cast<uint32_t>(i);
    std::sort(out.recordOfCar.begin(), out.recordOfCar.end(),
              [&cars](uint32_t a, uint32_t b) { return cars[a].id < cars[b].id; });
}

const Car* findCar(const std::vector<Car>& cars, const PathLists& lists, CarId id) {
    const std::vector<uint32_t>& order = lists.recordOfCar;
    std::vector<uint32_t>::const_iterator it =
        std::lower_bound(order.begin(), order.end(), id, [&cars](uint32_t record, CarId want) {
            return record < cars.size() && cars[record].id < want;
        });
    if (it == order.end() || *it >= cars.size() || cars[*it].id != id) return nullptr;
    return &cars[*it];
}

Neighbour carAhead(const engine::LaneGraph& roads, const std::vector<Car>& cars, const PathLists& lists, CarId me,
                   const std::vector<engine::PathId>& pathsAhead, Real lookAhead) {
    Neighbour best;
    const Car* mine = findCar(cars, lists, me);
    if (!mine || !knownPath(roads, mine->path)) return best;

    // On my own path: a car reaching past my front; its gap from my front to its rear on this path.
    for (const PathEntry& e : lists.on(mine->path)) {
        if (e.car == me || e.front <= mine->along) continue;
        const Real gap = e.rear - mine->along;
        if (gap <= lookAhead && nearer(e, gap, best)) {
            best.car = e.car;
            best.gap = gap;
        }
    }
    // On the paths ahead, each starting `toStart` metres past my front.
    Real toStart = roads.paths[mine->path.index].length - mine->along;
    for (const engine::PathId path : pathsAhead) {
        if (!knownPath(roads, path) || toStart > lookAhead || (best.found() && toStart > best.gap)) break;
        for (const PathEntry& e : lists.on(path)) {
            if (e.car == me) continue;
            const Real gap = toStart + e.rear;
            if (gap <= lookAhead && nearer(e, gap, best)) {
                best.car = e.car;
                best.gap = gap;
            }
        }
        toStart += roads.paths[path.index].length;
    }
    if (best.found()) {
        const Car* other = findCar(cars, lists, best.car);
        best.speed = other ? other->speed : 0;
    }
    return best;
}

Neighbour carBehind(const std::vector<Car>& cars, const PathLists& lists, engine::PathId lane, Real along,
                    CarId except) {
    Neighbour best;
    for (const PathEntry& e : lists.on(lane)) {
        if (e.why == OnPath::Tail || e.car == except || e.front > along) continue;
        const Real gap = along - e.front;
        if (nearer(e, gap, best)) {
            best.car = e.car;
            best.gap = gap;
        }
    }
    if (best.found()) {
        const Car* other = findCar(cars, lists, best.car);
        best.speed = other ? other->speed : 0;
    }
    return best;
}

std::vector<PathEntry> carsOnStretch(const PathLists& lists, engine::PathId path, Real from, Real to) {
    std::vector<PathEntry> out;
    for (const PathEntry& e : lists.on(path))
        if (e.rear <= to && e.front >= from) out.push_back(e);
    return out;
}

Real kerbwardPerpSign(DrivesOn drivesOn) {
    // Keeping to the side DrivesOn::Right names, the kerb is a quarter turn clockwise of the heading (seen from above):
    // perp's way. Keeping to the other side, it is the other way.
    return drivesOn == DrivesOn::Right ? Real(1) : Real(-1);
}

CarPose carPose(const engine::LaneGraph& roads, const VehicleClasses& classes, const RuleSet& rules, const Car& car) {
    CarPose pose;
    const Real length = car.vehicleClass < classes.size() ? classes.at(car.vehicleClass).length : Real(0);
    const Real kerbward = kerbwardPerpSign(rules.drivesOn) * car.offset;

    const engine::Vec2 heading = roads.directionAt(car.path, car.along);
    const engine::Vec2 front = roads.pointAt(car.path, car.along) + engine::perp(heading) * kerbward;
    pose.front = lift(front, roads.heightAt(car.path, car.along));
    pose.travel = engine::normalize(engine::Vec3(heading.x, roads.gradeAt(car.path, car.along), heading.y));

    const Body body = layBody(roads, car, length);
    if (body.count == 0) {
        pose.rear = pose.front;
        pose.body = pose.travel;
        return pose;
    }
    const BodyPiece& last = body.pieces[body.count - 1];
    const engine::Vec2 rearHeading = roads.directionAt(last.path, last.rear);
    const engine::Vec2 rear = roads.pointAt(last.path, last.rear) + engine::perp(rearHeading) * kerbward;
    pose.rear = lift(rear, roads.heightAt(last.path, last.rear));
    const engine::Vec3 span = pose.front - pose.rear;
    pose.body = span.length() > kShortestBody ? engine::normalize(span) : pose.travel;
    return pose;
}

}  // namespace driving
