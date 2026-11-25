#include "city_rail_riders.h"

#include "city_sim.h"   // Agent

#include <algorithm>
#include <cmath>
#include <queue>

using engine::Real;
using engine::Vec2;
using engine::Vec3;

namespace citysim {

namespace {

constexpr Real kWalkPace = 1.4;          // m/s along a way in (the stairs included)
constexpr Real kRidePace = 25.0;         // m/s, a train's average between stops (accelerating, curves)
constexpr int kSeatsPerCar = 72;         // a coach's seats (train_mesh's coach reports 72)
constexpr Real kPlatformBand0 = 0.8, kPlatformBand1 = 2.3;   // the band waiting riders stand in: metres in from the
                                                             // platform's edge (the stair runs along its back)

Real dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

// where the platform is: the line's railhead at `s`, the way it runs, and the platform side's plan direction
void frameAt(const RailLine& L, double s, int side, Vec2& p, Vec2& fwd, Vec2& out) {
    const Vec3 a = L.at(s - 2.0), b = L.at(s + 2.0), c = L.at(s);
    fwd = Vec2(b.x - a.x, b.z - a.z);
    const Real fl = std::hypot(fwd.x, fwd.y);
    fwd = fl > 1e-9 ? Vec2(fwd.x / fl, fwd.y / fl) : Vec2(0, 1);
    const Vec2 left(fwd.y, -fwd.x);   // cross(up, f) in plan: rail_sim.cpp's car frame
    out = Vec2(left.x * side, left.y * side);
    p = Vec2(c.x, c.z);
}

}  // namespace

void TrainRiders::clear() {
    rail_ = nullptr;
    comp_.clear();
    stations_.clear();
    rides_.clear();
    counts_ = Counts{};
}

void TrainRiders::build(const RailSim* rail, const WalkPointMap& spots, Real reach, Real minNetworkStreet) {
    clear();
    if (!rail || rail->lines().empty() || spots.empty()) return;
    rail_ = rail;

    // STREET NETWORKS: the parts of the walks (a town is one; city_walk_points.h), each as long as its street points
    // space out
    comp_ = spots.part;
    std::vector<Real> street;
    for (int s = 0; s < spots.streetNodeCount(); ++s) {
        const int c = comp_[static_cast<std::size_t>(s)];
        if (c < 0) continue;
        if (c >= static_cast<int>(street.size())) street.resize(static_cast<std::size_t>(c) + 1, 0);
        street[static_cast<std::size_t>(c)] += kWalkPointSpacing;
    }
    const int biggest = street.empty() ? -1 : static_cast<int>(std::max_element(street.begin(), street.end()) - street.begin());

    // THE STATIONS, by name across the lines (a double track's two lines both stop at each)
    const auto& lines = rail->lines();
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const RailLine& L = lines[li];
        for (std::size_t si = 0; si < L.stops.size(); ++si) {
            const engine::RailStop& st = L.stops[si];
            int k = -1;
            for (std::size_t j = 0; j < stations_.size(); ++j) if (stations_[j].name == st.name) k = static_cast<int>(j);
            if (k < 0) {
                RailStation S;
                S.name = st.name;
                S.stopOnLine.assign(lines.size(), -1);
                // the way in; a station without one (a lab's) is reached from beside its platform
                Vec2 p, f, o;
                frameAt(L, 0.5 * (st.s0 + st.s1), st.side, p, f, o);
                const Vec2 mid(p.x + o.x * (engine::kPlatformGap + 2.25), p.y + o.y * (engine::kPlatformGap + 2.25));
                S.access = st.hasAccess ? st.access : mid;
                S.walkIn = std::max<Real>(20.0, dist(S.access, mid) * 1.4 / kWalkPace);
                stations_.push_back(S);
                k = static_cast<int>(stations_.size()) - 1;
            }
            stations_[static_cast<std::size_t>(k)].stopOnLine[li] = static_cast<int>(si);
        }
    }
    // each pinned to a street spot
    for (RailStation& S : stations_) {
        Real bd = reach;
        for (int s = 0; s < spots.streetNodeCount(); ++s) {
            const int c = comp_[static_cast<std::size_t>(s)];
            if (c < 0 || (c != biggest && street[static_cast<std::size_t>(c)] < minNetworkStreet)) continue;
            const Real d = dist(spots.nodes[static_cast<std::size_t>(s)], S.access);
            if (d < bd) { bd = d; S.accessNode = s; S.network = c; }
        }
    }
}

namespace {
// a line's trains: how many, and the pace and dwell they keep between stops (the first one's kind; a tram is no intercity)
struct LineService {
    int trains = 0;
    Real pace = kRidePace, dwell = 30;
};
LineService serviceOf(const RailSim& rail, std::size_t li) {
    LineService out;
    for (const Train& t : rail.trains()) {
        if (t.line != static_cast<int>(li)) continue;
        if (out.trains++ == 0) {
            out.pace = std::min<Real>(kRidePace, 0.75 * t.kind.vmax);
            out.dwell = t.kind.dwell;
        }
    }
    return out;
}
// metres along line `L` from stop `a` to stop `b`, the way it runs (an open line: -1 when b is not ahead)
double arcBetween(const RailLine& L, int a, int b) {
    double arc = L.stops[static_cast<std::size_t>(b)].s1 - L.stops[static_cast<std::size_t>(a)].s1;
    if (arc <= 0) {
        if (!L.loop) return -1;
        arc += L.length;
    }
    return arc;
}
}  // namespace

int TrainRiders::stationOf(int line, int stop) const {
    for (std::size_t k = 0; k < stations_.size(); ++k) {
        const std::vector<int>& on = stations_[k].stopOnLine;
        if (line >= 0 && line < static_cast<int>(on.size()) && on[static_cast<std::size_t>(line)] == stop) return static_cast<int>(k);
    }
    return -1;
}

void TrainRiders::addTimetable(TransitTimetable& tt) const {
    if (!rail_) return;
    const std::vector<RailLine>& lines = rail_->lines();
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const RailLine& L = lines[li];
        const LineService sv = serviceOf(*rail_, li);
        const int n = static_cast<int>(L.stops.size());
        if (sv.trains == 0 || n < 2) continue;
        TransitTimetable::Line tl;
        tl.network = "rail";
        tl.source = static_cast<int>(li);
        tl.loop = L.loop;
        tl.headwaySeconds = (L.length / sv.pace + sv.dwell * static_cast<Real>(n)) / static_cast<Real>(sv.trains);
        for (int k = 0; k < n; ++k) {
            TransitTimetable::Stop st;
            const int station = stationOf(static_cast<int>(li), k);
            if (station >= 0) {
                st.wait = stations_[static_cast<std::size_t>(station)].access;
                st.accessSeconds = stations_[static_cast<std::size_t>(station)].walkIn;
            }
            const double arc = (k + 1 < n || L.loop) ? arcBetween(L, k, (k + 1) % n) : -1;
            st.toNextSeconds = arc > 0 ? arc / sv.pace + sv.dwell : 0;
            tl.stops.push_back(st);
        }
        tt.lines.push_back(std::move(tl));
    }
}

std::vector<int> TrainRiders::passengers() const {
    std::vector<int> out;
    out.reserve(rides_.size());
    for (const TrainRide& r : rides_) out.push_back(r.agent);
    return out;
}

const TrainRide* TrainRiders::rideOf(int agent) const {
    const std::vector<TrainRide>::const_iterator it =
        std::lower_bound(rides_.begin(), rides_.end(), agent, [](const TrainRide& r, int a) { return r.agent < a; });
    return it != rides_.end() && it->agent == agent ? &*it : nullptr;
}

TrainRide* TrainRiders::find(int agent) { return const_cast<TrainRide*>(static_cast<const TrainRiders*>(this)->rideOf(agent)); }

bool TrainRiders::doorToDoor(int fromNode, Vec2 fromPos, int toNode, Vec2 toPos, Real* seconds) const {
    if (!active()) return false;
    const int na = networkOf(fromNode), nb = networkOf(toNode);
    if (na < 0 || nb < 0) return false;
    // every station in the start's town against every one in the end's, on the line that takes it quickest -- a town's
    // light-rail stop round the corner beats its main-line station across town
    constexpr Real kStreetFactor = 1.3;
    const std::vector<RailLine>& lines = rail_->lines();
    Real bestT = 1e30;
    bool found = false;
    for (const RailStation& A : stations_) {
        if (A.accessNode < 0 || A.network != na) continue;
        for (const RailStation& B : stations_) {
            if (&B == &A || B.accessNode < 0 || B.network != nb) continue;
            const Real walk = (dist(fromPos, A.access) + dist(B.access, toPos)) * kStreetFactor / kWalkPace + A.walkIn + B.walkIn;
            for (std::size_t li = 0; li < lines.size(); ++li) {
                const RailLine& L = lines[li];
                const int sa = A.stopOnLine[li], sb = B.stopOnLine[li];
                if (sa < 0 || sb < 0) continue;
                const double arc = arcBetween(L, sa, sb);
                const LineService sv = serviceOf(*rail_, li);
                if (arc <= 0 || sv.trains == 0) continue;
                const int nStops = static_cast<int>(L.stops.size());
                const int between = L.loop ? (sb - sa + nStops) % nStops - 1 : sb - sa - 1;
                const Real lap = L.length / sv.pace + sv.dwell * static_cast<Real>(nStops);
                const Real t = walk + 0.5 * lap / sv.trains + arc / sv.pace + sv.dwell * std::max(0, between);
                if (t < bestT) { bestT = t; found = true; }
            }
        }
    }
    if (found && seconds) *seconds = bestT;
    return found;
}

bool TrainRiders::platformSpot(int st, int line, int k, Vec2& pos, Real& top, Vec2& face) const {
    if (!rail_ || st < 0 || st >= static_cast<int>(stations_.size())) return false;
    const auto& lines = rail_->lines();
    if (line < 0 || line >= static_cast<int>(lines.size())) return false;
    const int si = stations_[static_cast<std::size_t>(st)].stopOnLine[static_cast<std::size_t>(line)];
    if (si < 0) return false;
    const RailLine& L = lines[static_cast<std::size_t>(line)];
    const engine::RailStop& S = L.stops[static_cast<std::size_t>(si)];
    // spots spread over the platform's middle 70 %, a hash of k so a crowd fills it unevenly, two rows deep
    const uint32_t h = static_cast<uint32_t>(k) * 2654435761u;
    const Real u = static_cast<Real>((h >> 8) % 1000) / 1000.0;
    const Real s = S.s0 + (S.s1 - S.s0) * (0.15 + 0.7 * u);
    const Real band = kPlatformBand0 + (kPlatformBand1 - kPlatformBand0) * static_cast<Real>((h >> 20) % 100) / 100.0;
    Vec2 p, f, o;
    frameAt(L, s, S.side, p, f, o);
    const Real lat = engine::kPlatformGap + band;
    pos = Vec2(p.x + o.x * lat, p.y + o.y * lat);
    top = S.top;
    face = Vec2(-o.x, -o.y);
    return true;
}

bool TrainRiders::wait(int agent, int line, int board, int alight) {
    const int from = stationOf(line, board), to = stationOf(line, alight);
    if (!rail_ || from < 0 || to < 0 || board == alight) return false;
    TrainRide r;
    r.agent = agent;
    r.line = line;
    r.board = board;
    r.alight = alight;
    r.from = from;
    r.to = to;
    r.phase = TrainRide::Phase::Entering;
    r.timer = stations_[static_cast<std::size_t>(from)].walkIn;
    if (TrainRide* had = find(agent)) *had = r;
    else rides_.insert(std::lower_bound(rides_.begin(), rides_.end(), agent, [](const TrainRide& x, int a) { return x.agent < a; }), r);
    ++counts_.waits;
    return true;
}

bool TrainRiders::aboard(int agent, int line, int alight) {
    const int to = stationOf(line, alight);
    if (!rail_ || to < 0) return false;
    // the line's train with the least running left to the stop (on an open line, only one short of it)
    const RailLine& L = rail_->lines()[static_cast<std::size_t>(line)];
    const double at = L.stops[static_cast<std::size_t>(alight)].s1;
    int best = -1;
    double bestLeft = 0;
    const std::vector<Train>& trains = rail_->trains();
    for (std::size_t ti = 0; ti < trains.size(); ++ti) {
        if (trains[ti].line != line) continue;
        double left = at - trains[ti].s;
        if (left <= 0) {
            if (!L.loop) continue;
            left += L.length;
        }
        if (best < 0 || left < bestLeft) { best = static_cast<int>(ti); bestLeft = left; }
    }
    if (best < 0) return false;
    TrainRide r;
    r.agent = agent;
    r.line = line;
    r.alight = alight;
    r.to = to;
    r.phase = TrainRide::Phase::Aboard;
    r.train = best;
    if (TrainRide* had = find(agent)) *had = r;
    else rides_.insert(std::lower_bound(rides_.begin(), rides_.end(), agent, [](const TrainRide& x, int a) { return x.agent < a; }), r);
    return true;
}

void TrainRiders::cancel(int agent, std::vector<Agent>& agents) {
    const TrainRide* r = rideOf(agent);
    if (!r) return;
    if (agent >= 0 && agent < static_cast<int>(agents.size())) {
        Agent& a = agents[static_cast<std::size_t>(agent)];
        // out on the street at the station they were in: the way in they came by, or the one they were heading for
        const int st = r->phase == TrainRide::Phase::Aboard || r->phase == TrainRide::Phase::Leaving || r->from < 0 ? r->to : r->from;
        if (st >= 0) a.pos = stations_[static_cast<std::size_t>(st)].access;
        a.tickFromPos = a.pos;
        a.indoors = false;
    }
    rides_.erase(rides_.begin() + (r - rides_.data()));
}

std::vector<RideEvent> TrainRiders::tick(Real dt, std::vector<Agent>& agents) {
    std::vector<RideEvent> out;
    if (!rail_ || rides_.empty()) return out;
    const std::vector<Train>& trains = rail_->trains();
    const std::vector<RailLine>& lines = rail_->lines();
    std::vector<int> aboardOn(trains.size(), 0);
    std::vector<int> done;   // riders out on the street this tick
    for (TrainRide& r : rides_) {
        if (r.agent < 0 || r.agent >= static_cast<int>(agents.size())) { done.push_back(r.agent); continue; }
        Agent& a = agents[static_cast<std::size_t>(r.agent)];
        if (r.phase == TrainRide::Phase::Entering) {
            a.indoors = true;   // (on the footway and the stairs: not drawn)
            r.timer -= dt;
            if (r.timer > 0) continue;
            Vec2 pos, face;
            Real top = 0;
            if (platformSpot(r.from, r.line, r.agent, pos, top, face)) {
                a.pos = pos;
                a.tickFromPos = pos;
                a.heading = face;
                r.spot = pos;
                r.top = top;
            }
            a.indoors = false;
            a.moving = false;
            r.phase = TrainRide::Phase::Waiting;
        } else if (r.phase == TrainRide::Phase::Leaving) {
            a.indoors = true;
            r.timer -= dt;
            if (r.timer > 0) continue;
            a.pos = stations_[static_cast<std::size_t>(r.to)].access;   // out on the street at the far way in
            a.tickFromPos = a.pos;
            a.indoors = false;
            done.push_back(r.agent);
            out.push_back(RideEvent{r.agent, false, true});
        } else if (r.phase == TrainRide::Phase::Aboard && r.train >= 0 && r.train < static_cast<int>(trains.size())) {
            const Train& T = trains[static_cast<std::size_t>(r.train)];
            const engine::Vec3 h = lines[static_cast<std::size_t>(T.line)].at(T.s);
            a.pos = Vec2(h.x, h.z);   // (where the train is: the brain's attention reads it)
            a.tickFromPos = a.pos;
            a.indoors = true;         // (drawn on a seat by the train system, not on the street)
            ++aboardOn[static_cast<std::size_t>(r.train)];
        }
    }
    // AT THE PLATFORMS: off at their stop, then on, while a train stands with its doors open
    for (std::size_t ti = 0; ti < trains.size(); ++ti) {
        const Train& T = trains[ti];
        if (!T.dwelling || T.doorSide == 0) continue;
        const int stop = T.nextStop;
        const int seats = T.kind.cars * kSeatsPerCar;
        for (TrainRide& r : rides_) {
            if (r.phase == TrainRide::Phase::Aboard && r.train == static_cast<int>(ti) && r.alight == stop) {
                r.phase = TrainRide::Phase::Leaving;
                r.timer = stations_[static_cast<std::size_t>(r.to)].walkIn;
                r.train = -1;
                --aboardOn[ti];
                ++counts_.alightings;
            } else if (r.phase == TrainRide::Phase::Waiting && r.line == T.line && r.board == stop) {
                if (aboardOn[ti] >= seats) {
                    if (r.leftBy != static_cast<int>(ti)) ++counts_.leftFull;
                    r.leftBy = static_cast<int>(ti);
                    continue;
                }
                r.phase = TrainRide::Phase::Aboard;
                r.train = static_cast<int>(ti);
                ++aboardOn[ti];
                ++counts_.boardings;
                if (r.agent >= 0 && r.agent < static_cast<int>(agents.size())) agents[static_cast<std::size_t>(r.agent)].indoors = true;
                out.push_back(RideEvent{r.agent, true, true});
            }
        }
    }
    if (!done.empty())
        rides_.erase(std::remove_if(rides_.begin(), rides_.end(),
                                    [&](const TrainRide& r) { return std::find(done.begin(), done.end(), r.agent) != done.end(); }),
                     rides_.end());
    return out;
}

}  // namespace citysim
