#include "city_buses.h"

#include "city_car.h"       // CityBusRuns, carPosition
#include "city_driving.h"   // busService
#include "city_sim.h"       // Agent
#include "../driving/driving.h"
#include "../../engine/procgen/city/roads/road_model.h"

#include <algorithm>

namespace citysim {

void BusRiders::reset() {
    riders_.clear();
    timetable_ = BusTimetable();
    world_ = nullptr;
    counts_ = Counts();
}

void BusRiders::build(const engine::RoadModel& model, const driving::Driving& d, const std::vector<uint8_t>& openWalks,
                      engine::World& world) {
    reset();
    world_ = &world;
    const int bus = d.vehicleClasses().indexOf("bus");
    const BusService& service = busService();
    if (bus < 0 || model.busLines.empty() || !service.usable) return;
    for (const engine::ModelBusLine& line : model.busLines) {
        std::vector<BusTimetable::Stop> stops;
        const std::size_t n = line.stops.size();
        for (std::size_t k = 0; k < n; ++k) {
            BusTimetable::Stop st;
            const engine::WalkJoin w = model.busStopWalk(line.stops[k], openWalks);
            const engine::KerbLanePlace a = model.busStopLane(line.stops[k]), b = model.busStopLane(line.stops[(k + 1) % n]);
            st.wait = w.valid() ? w.point : a.at;
            Real drive = 600;   // (no route: a leg nobody would wait for; Driving::busLineProblems said so at load)
            if (a.valid() && b.valid()) {
                driving::PathPoint pa, pb;
                pa.path = a.path;
                pa.along = a.along;
                pb.path = b.path;
                pb.along = b.along;
                const engine::roads::Route r = d.routeBetween(static_cast<uint32_t>(bus), pa, pb);
                if (r.found) drive = r.seconds * service.trafficFactor;
            }
            st.toNextSeconds = drive + service.dwellSeconds;
            stops.push_back(st);
        }
        timetable_.lines.push_back(std::move(stops));
    }
}

bool BusRiders::wait(int agent, uint32_t line, uint32_t board, uint32_t alight) {
    if (line >= timetable_.lines.size() || board >= timetable_.lines[line].size() || alight >= timetable_.lines[line].size())
        return false;
    riders_.erase(std::remove_if(riders_.begin(), riders_.end(), [agent](const Rider& r) { return r.agent == agent; }), riders_.end());
    Rider r;
    r.agent = agent;
    r.line = line;
    r.board = board;
    r.alight = alight;
    riders_.push_back(r);
    ++counts_.waits;
    return true;
}

bool BusRiders::aboard(int agent, uint32_t line, uint32_t alight) {
    if (line >= timetable_.lines.size() || alight >= timetable_.lines[line].size()) return false;
    riders_.erase(std::remove_if(riders_.begin(), riders_.end(), [agent](const Rider& r) { return r.agent == agent; }), riders_.end());
    Rider r;
    r.agent = agent;
    r.line = line;
    r.alight = alight;
    r.onBoard = true;
    riders_.push_back(r);
    return true;
}

void BusRiders::cancel(int agent, std::vector<Agent>& agents) {
    for (const Rider& r : riders_)
        if (r.agent == agent && r.onBoard && agent >= 0 && static_cast<std::size_t>(agent) < agents.size())
            agents[static_cast<std::size_t>(agent)].indoors = false;   // put down where the bus is
    riders_.erase(std::remove_if(riders_.begin(), riders_.end(), [agent](const Rider& r) { return r.agent == agent; }), riders_.end());
}

bool BusRiders::riding(int agent) const {
    for (const Rider& r : riders_)
        if (r.agent == agent) return true;
    return false;
}

int BusRiders::aboardOn(engine::Entity bus) const {
    int n = 0;
    for (const Rider& r : riders_) n += r.onBoard && r.bus == bus ? 1 : 0;
    return n;
}

std::vector<BusRiders::Event> BusRiders::tick(std::vector<Agent>& agents) {
    std::vector<Event> out;
    if (!world_ || riders_.empty()) return out;
    std::vector<std::pair<uint32_t, engine::Entity>> buses;
    world_->each<CityBusRuns, CityCarBody>(
        [&](engine::Entity e, CityBusRuns&, CityCarBody& body) { buses.push_back({body.id, e}); });
    std::sort(buses.begin(), buses.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    const int capacity = busService().capacity;
    for (const std::pair<uint32_t, engine::Entity>& b : buses) {
        const engine::Entity bus = b.second;
        const CityBusRuns& runs = *world_->get<CityBusRuns>(bus);
        const Vec2 at = carPosition(*world_->get<engine::Transform>(bus));
        int load = 0;
        for (const Rider& r : riders_) load += r.onBoard && r.bus == bus ? 1 : 0;
        std::vector<Rider> keep;
        keep.reserve(riders_.size());
        for (Rider& r : riders_) {
            if (r.agent < 0 || static_cast<std::size_t>(r.agent) >= agents.size() || r.line != runs.line) {
                keep.push_back(r);
                continue;
            }
            Agent& a = agents[static_cast<std::size_t>(r.agent)];
            if (r.onBoard && (r.bus == bus || r.bus == engine::Entity{})) {
                a.pos = at;   // carried where the bus is
                if (runs.standing && runs.stop == r.alight) {
                    // OFF at their stop: on the pavement where people wait for it
                    a.pos = timetable_.lines[r.line][r.alight].wait;
                    a.indoors = false;
                    if (r.bus == bus) --load;
                    ++counts_.alightings;
                    out.push_back(Event{r.agent, false});
                    continue;
                }
                if (r.bus == engine::Entity{}) r.bus = bus;   // (the first bus of the line it is seen on: theirs)
                keep.push_back(r);
                continue;
            }
            if (!r.onBoard && runs.standing && runs.stop == r.board) {
                if (load >= capacity) {
                    ++counts_.leftFull;
                    keep.push_back(r);
                    continue;
                }
                // ON: the bus they waited for stands at their stop, with room
                r.onBoard = true;
                r.bus = bus;
                ++load;
                a.indoors = true;
                a.moving = false;
                a.pos = at;
                ++counts_.boardings;
                out.push_back(Event{r.agent, true});
            }
            keep.push_back(r);
        }
        riders_.swap(keep);
    }
    return out;
}

}  // namespace citysim
