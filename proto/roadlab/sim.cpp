#include "sim.h"

#include "diag.h"

#include <algorithm>
#include <queue>

namespace roadlab {

Simulation::Simulation(const Network& net, const SimParams& params)
    : net_(net), params_(params), rng_(params.seed) {
    laneOccupants_.assign(net_.lanes().nodes.size(), {});
    connectorOf_.assign(size_t(net_.roadCount()), {-1, -1});
    junctionState_.resize(size_t(net_.junctionCount()));
    for (const Junction& j : net_.junctions()) {
        junctionState_[size_t(j.id)].occupied.assign(j.connections.size(), 0);
        junctionState_[size_t(j.id)].arrival.assign(j.connections.size(), 1e9);
        for (size_t ci = 0; ci < j.connections.size(); ++ci) {
            int cr = j.connections[ci].connectorRoad;
            if (cr >= 0 && cr < net_.roadCount()) connectorOf_[size_t(cr)] = {j.id, int(ci)};
        }
    }
    buildWalkGraph();
}

namespace {

// The centre of a road's footway on one side, if it has one there.
bool footwayOffset(const Road& r, double s, int side, double& tOut) {
    const LaneSection& sec = r.xs.sectionAt(s);
    const std::vector<Strip>& stack = side > 0 ? sec.left : sec.right;
    double ds = s - sec.s0, acc = 0;
    for (const Strip& st : stack) {
        double w = std::max(0.0, st.width.eval(ds));
        if (st.kind == StripKind::Sidewalk && w > 0.5) {
            tOut = (side > 0 ? 1.0 : -1.0) * (acc + w * 0.5);
            return true;
        }
        acc += w;
    }
    return false;
}

}  // namespace

int Simulation::findWalkNode(int road, int side, bool atEnd) const {
    for (size_t i = 0; i < walkNodes_.size(); ++i) {
        const WalkNode& n = walkNodes_[i];
        if (n.road == road && n.side == side && n.atEnd == atEnd) return int(i);
    }
    return -1;
}

void Simulation::buildWalkGraph() {
    walkNodes_.clear();
    walkLinks_.clear();

    // A node per (road, side, end) that actually has a footway there.
    for (const Road& r : net_.roads()) {
        if (!r.allowsPedestrians || r.kind == RoadKind::Connector) continue;
        if (r.activeLength() < 6.0) continue;
        for (int side : {-1, 1}) {
            double t = 0;
            if (!footwayOffset(r, r.begin() + 1.0, side, t)) continue;
            for (bool atEnd : {false, true}) {
                double s = atEnd ? r.end() - 0.5 : r.begin() + 0.5;
                double tt = 0;
                if (!footwayOffset(r, s, side, tt)) tt = t;
                WalkNode n;
                n.road = r.id;
                n.side = side;
                n.atEnd = atEnd;
                n.pos = r.surfacePoint(s, tt);
                walkNodes_.push_back(n);
            }
        }
    }

    // CROSSINGS, straight from the paint generator: a pedestrian can only cross
    // where a zebra was actually drawn.
    for (const Junction& j : net_.junctions()) {
        for (const Crosswalk& cw : junctionCrosswalks(net_, j)) {
            const Road& r = net_.road(cw.road);
            bool atEnd = std::fabs(cw.s - r.end()) < std::fabs(cw.s - r.begin());
            int a = findWalkNode(cw.road, +1, atEnd);
            int b = findWalkNode(cw.road, -1, atEnd);
            if (a < 0 || b < 0) continue;
            WalkLink link;
            link.a = a;
            link.b = b;
            link.crossing = true;
            link.road = cw.road;
            link.s = cw.s;
            link.junction = j.id;
            link.length = std::max(3.0, cw.tMax - cw.tMin);
            walkLinks_.push_back(link);
            walkNodes_[size_t(a)].junction = j.id;
            walkNodes_[size_t(b)].junction = j.id;
        }
    }

    // CORNERS: walking round an intersection. Nodes belonging to a junction are
    // ordered by bearing and consecutive ones joined, which is exactly the ring
    // of footway that wraps the kerb returns. Consecutive nodes on the SAME road
    // are skipped — that pair is the crossing, not a corner.
    for (const Junction& j : net_.junctions()) {
        std::vector<int> ring;
        for (size_t i = 0; i < walkNodes_.size(); ++i) {
            const WalkNode& n = walkNodes_[i];
            for (const JunctionArm& arm : j.arms) {
                if (arm.road == n.road && arm.atEnd == n.atEnd) {
                    ring.push_back(int(i));
                    walkNodes_[i].junction = j.id;
                    break;
                }
            }
        }
        if (ring.size() < 2) continue;
        std::sort(ring.begin(), ring.end(), [&](int a, int b) {
            Vec2 pa = planOf(walkNodes_[size_t(a)].pos) - j.center;
            Vec2 pb = planOf(walkNodes_[size_t(b)].pos) - j.center;
            return std::atan2(pa.y, pa.x) < std::atan2(pb.y, pb.x);
        });
        for (size_t i = 0; i < ring.size(); ++i) {
            int a = ring[i];
            int b = ring[(i + 1) % ring.size()];
            if (a == b) continue;
            if (walkNodes_[size_t(a)].road == walkNodes_[size_t(b)].road) continue;
            WalkLink link;
            link.a = a;
            link.b = b;
            link.crossing = false;
            link.junction = j.id;
            link.length = length(walkNodes_[size_t(a)].pos - walkNodes_[size_t(b)].pos);
            walkLinks_.push_back(link);
        }
    }

    // Plain road-to-road continuations, where the footway just carries on.
    for (const Road& r : net_.roads()) {
        if (r.succ.type != LinkType::Road || r.succ.id < 0) continue;
        for (int side : {-1, 1}) {
            int a = findWalkNode(r.id, side, true);
            int b = findWalkNode(r.succ.id, side, !r.succ.toStart);
            if (a < 0 || b < 0) continue;
            WalkLink link;
            link.a = a;
            link.b = b;
            link.length = length(walkNodes_[size_t(a)].pos - walkNodes_[size_t(b)].pos);
            walkLinks_.push_back(link);
        }
    }

    walkLinksOf_.assign(walkNodes_.size(), {});
    for (size_t i = 0; i < walkLinks_.size(); ++i) {
        walkLinksOf_[size_t(walkLinks_[i].a)].push_back(int(i));
        walkLinksOf_[size_t(walkLinks_[i].b)].push_back(int(i));
    }
}

double Simulation::laneLength(int node) const {
    if (node < 0 || size_t(node) >= net_.lanes().nodes.size()) return 0;
    return net_.lanes().nodes[size_t(node)].length;
}

double Simulation::roadStationOf(const Vehicle& v) const {
    const LaneNode& n = net_.lanes().nodes[size_t(v.lane)];
    return n.roadS(v.travelled);
}

int Simulation::randomStartLane() {
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    if (nodes.empty()) return -1;
    for (int attempt = 0; attempt < 200; ++attempt) {
        int i = rng_.rangeI(0, int(nodes.size()) - 1);
        const LaneNode& n = nodes[size_t(i)];
        if (n.junctionId >= 0) continue;          // don't start inside a junction
        if (n.length < 20.0) continue;
        if (!(n.access & kAccessCar)) continue;
        if (n.kind != StripKind::Travel) continue;
        return i;
    }
    return -1;
}

void Simulation::seedVehicles(int count) {
    for (int i = 0; i < count; ++i) {
        int lane = randomStartLane();
        if (lane < 0) break;
        Vehicle v;
        v.id = nextId_++;
        v.lane = lane;
        v.travelled = rng_.range(2.0, std::max(3.0, laneLength(lane) - 2.0));
        const LaneNode& n = net_.lanes().nodes[size_t(lane)];
        v.speed = n.speedLimit / 3.6 * rng_.range(0.6, 0.95);
        v.desiredFactor = rng_.range(0.88, 1.12);
        v.length = rng_.range(4.1, 5.4);
        if (rng_.chance(0.12)) {
            v.length = rng_.range(9.0, 14.0);   // a few trucks, to see gaps matter
            v.width = 2.5;
            v.desiredFactor *= 0.86;
        }
        v.color = {rng_.range(0.10, 0.75), rng_.range(0.10, 0.70), rng_.range(0.10, 0.75)};
        vehicles_.push_back(v);
    }
    for (Vehicle& v : vehicles_) assignRoute(v);
    rebuildOccupancy();
}

void Simulation::seedPedestrians(int count) {
    if (walkNodes_.empty()) return;
    for (int i = 0; i < count; ++i) {
        // Start on a footway, heading for one of its ends.
        int node = rng_.rangeI(0, int(walkNodes_.size()) - 1);
        const WalkNode& n = walkNodes_[size_t(node)];
        const Road& r = net_.road(n.road);
        Pedestrian p;
        p.node = node;
        p.road = n.road;
        p.side = n.side;
        p.s = rng_.range(r.begin() + 1.0, r.end() - 1.0);
        // Walk toward the end this node sits at.
        p.dir = n.atEnd ? 1.0 : -1.0;
        p.speed = rng_.range(1.05, 1.65);
        p.color = {rng_.range(0.2, 0.8), rng_.range(0.2, 0.7), rng_.range(0.2, 0.8)};
        peds_.push_back(p);
    }
}

void Simulation::seedParking(double occupancy) {
    slots_.clear();
    parked_.clear();
    for (const Road& r : net_.roads()) {
        if (r.kind == RoadKind::Connector) continue;
        for (size_t si = 0; si < r.xs.sections.size(); ++si) {
            const LaneSection& sec = r.xs.sections[si];
            double s0 = std::max(sec.s0, r.begin());
            double s1 = std::min(sec.s0 + sec.length, r.end());
            for (const std::vector<Strip>* stack : {&sec.left, &sec.right}) {
                for (const Strip& st : *stack) {
                    if (st.kind != StripKind::Parking) continue;
                    // Slots are a rule over the strip, exactly like the tee
                    // markings the shader paints on the same strip.
                    for (double s = s0 + 3.0; s + 5.6 < s1 - 3.0; s += 6.5) {
                        ParkingSlot slot;
                        slot.road = r.id;
                        slot.s = s + 2.8;
                        slot.t = r.xs.laneCenterT(int(si), st.id, s + 2.8);
                        slot.occupied = rng_.next01() < occupancy;
                        slots_.push_back(slot);
                        if (!slot.occupied) continue;
                        ParkedCar pc;
                        pc.pos = r.surfacePoint(slot.s, slot.t) + Vec3{0, 0.72, 0};
                        pc.yaw = r.spine.frameAt(slot.s).heading;
                        pc.color = {rng_.range(0.1, 0.7), rng_.range(0.1, 0.6),
                                    rng_.range(0.1, 0.7)};
                        parked_.push_back(pc);
                    }
                }
            }
        }
    }
}

void Simulation::rebuildOccupancy() {
    laneOccupants_.assign(net_.lanes().nodes.size(), {});
    for (size_t i = 0; i < vehicles_.size(); ++i) {
        const Vehicle& v = vehicles_[i];
        if (!v.active || v.lane < 0) continue;
        laneOccupants_[size_t(v.lane)].push_back(int(i));
    }
    for (std::vector<int>& list : laneOccupants_) {
        std::sort(list.begin(), list.end(), [&](int a, int b) {
            return vehicles_[size_t(a)].travelled < vehicles_[size_t(b)].travelled;
        });
    }
}

void Simulation::updateJunctionState() {
    for (JunctionState& js : junctionState_) {
        std::fill(js.occupied.begin(), js.occupied.end(), char(0));
        std::fill(js.arrival.begin(), js.arrival.end(), 1e9);
    }
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    for (const Vehicle& v : vehicles_) {
        if (!v.active || v.lane < 0) continue;
        const LaneNode& n = nodes[size_t(v.lane)];
        int roadId = n.ref.road;
        if (roadId < 0 || size_t(roadId) >= connectorOf_.size()) continue;
        auto [jid, ci] = connectorOf_[size_t(roadId)];
        if (jid >= 0) {
            junctionState_[size_t(jid)].occupied[size_t(ci)] = 1;
            junctionState_[size_t(jid)].arrival[size_t(ci)] = 0.0;
            continue;
        }
        // Not on a connector: is the vehicle about to enter one?
        for (int succ : n.successors) {
            const LaneNode& sn = nodes[size_t(succ)];
            int sr = sn.ref.road;
            if (sr < 0 || size_t(sr) >= connectorOf_.size()) continue;
            auto [sjid, sci] = connectorOf_[size_t(sr)];
            if (sjid < 0) continue;
            double dist = std::max(0.0, n.length - v.travelled);
            double t = dist / std::max(1.0, v.speed);
            double& slot = junctionState_[size_t(sjid)].arrival[size_t(sci)];
            slot = std::min(slot, t);
        }
    }
}

double Simulation::desiredSpeedFor(const Vehicle& v) const {
    const LaneNode& n = net_.lanes().nodes[size_t(v.lane)];
    double limit = n.speedLimit / 3.6 * v.desiredFactor;
    // Curvature caps the speed independently of the sign: the same geometry that
    // set the design speed sets what is comfortable here and now.
    const Road& r = net_.road(n.ref.road);
    double k = std::fabs(r.spine.curvatureAt(roadStationOf(v)));
    if (k > 1e-5) limit = std::min(limit, std::sqrt(params_.lateralAccelLimit / k));
    return std::max(2.0, limit);
}

double Simulation::gapToLeader(const Vehicle& v, double& leaderSpeed, double horizon) const {
    leaderSpeed = 1e3;
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    // Same lane first: the ordered occupant list makes this a single step.
    const std::vector<int>& here = laneOccupants_[size_t(v.lane)];
    for (int idx : here) {
        const Vehicle& o = vehicles_[size_t(idx)];
        if (o.id == v.id || !o.active) continue;
        if (o.travelled <= v.travelled) continue;
        leaderSpeed = o.speed;
        return o.travelled - v.travelled - 0.5 * (v.length + o.length);
    }
    // Then downstream, following the lane graph until the horizon runs out. This
    // is what stops a queue at a junction from being invisible to the car
    // approaching it.
    double base = laneLength(v.lane) - v.travelled;
    int node = v.lane;
    for (int depth = 0; depth < 4 && base < horizon; ++depth) {
        const LaneNode& n = nodes[size_t(node)];
        if (n.successors.empty()) {
            // A lane that simply ends is itself an obstacle; the driver must be
            // out of it before the taper closes.
            return base;
        }
        int next = n.successors.front();
        const std::vector<int>& list = laneOccupants_[size_t(next)];
        if (!list.empty()) {
            const Vehicle& o = vehicles_[size_t(list.front())];
            leaderSpeed = o.speed;
            return base + o.travelled - 0.5 * (v.length + o.length);
        }
        base += laneLength(next);
        node = next;
    }
    return horizon;
}

double Simulation::gapBehind(int laneNode, double travelled, double& followerSpeed) const {
    followerSpeed = 0;
    if (laneNode < 0) return 1e9;
    const std::vector<int>& list = laneOccupants_[size_t(laneNode)];
    for (size_t i = list.size(); i-- > 0;) {
        const Vehicle& o = vehicles_[size_t(list[i])];
        if (o.travelled < travelled) {
            followerSpeed = o.speed;
            return travelled - o.travelled;
        }
    }
    return 1e9;
}

bool Simulation::blockedByJunction(const Vehicle& v, double& distance) const {
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    const LaneNode& n = nodes[size_t(v.lane)];
    int roadId = n.ref.road;
    // Already inside the junction: committed, keep going.
    if (roadId >= 0 && size_t(roadId) < connectorOf_.size() &&
        connectorOf_[size_t(roadId)].first >= 0)
        return false;

    double toEnd = std::max(0.0, n.length - v.travelled);
    if (toEnd > 55.0) return false;

    for (int succ : n.successors) {
        const LaneNode& sn = nodes[size_t(succ)];
        int sr = sn.ref.road;
        if (sr < 0 || size_t(sr) >= connectorOf_.size()) continue;
        auto [jid, ci] = connectorOf_[size_t(sr)];
        if (jid < 0) continue;
        const Junction& j = net_.junction(jid);
        const Connection& conn = j.connections[size_t(ci)];

        // Signals: the phase the conflict graph was coloured into is the phase
        // the driver obeys. No second source of truth.
        if (j.control == JunctionControl::Signalized) {
            if (!j.isGreen(int(ci), time_)) {
                // Amber-to-red dilemma zone: too close to stop, so go.
                if (toEnd > v.speed * 1.2) {
                    distance = toEnd;
                    return true;
                }
            }
            continue;
        }
        if (!conn.yields) continue;

        const JunctionState& js = junctionState_[size_t(jid)];
        for (const ConflictPoint& cp : j.conflicts) {
            int other = -1;
            if (cp.connA == int(ci)) other = cp.connB;
            else if (cp.connB == int(ci)) other = cp.connA;
            if (other < 0) continue;
            if (cp.kind == ConflictKind::Diverging) continue;
            const Connection& oc = j.connections[size_t(other)];
            // Two movements that both yield sort themselves out by arrival; a
            // movement with priority does not wait for one without.
            if (oc.yields && conn.priority >= oc.priority) continue;
            if (js.occupied[size_t(other)] || js.arrival[size_t(other)] < params_.yieldHorizon) {
                distance = toEnd;
                return true;
            }
        }
    }
    return false;
}

std::vector<int> Simulation::planRoute(int fromNode) {
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    std::vector<int> path;
    RL_CALLED("planRoute");
    if (fromNode < 0 || size_t(fromNode) >= nodes.size()) {
        RL_FALLBACK("planRoute called with no start node");
        return path;
    }

    // Dijkstra by travel TIME, not distance: a driver routing by distance would
    // happily take a residential street over the parallel arterial.
    std::vector<double> dist(nodes.size(), 1e300);
    std::vector<int> prev(nodes.size(), -1);
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    dist[size_t(fromNode)] = 0.0;
    pq.push({0.0, fromNode});
    double furthest = 0;
    while (!pq.empty()) {
        auto [d, n] = pq.top();
        pq.pop();
        if (d > dist[size_t(n)] + 1e-9) continue;
        furthest = std::max(furthest, d);
        for (int nx : nodes[size_t(n)].successors) {
            const LaneNode& t = nodes[size_t(nx)];
            if (!(t.access & kAccessCar)) continue;
            double cost = t.length / std::max(3.0, double(t.speedLimit) / 3.6);
            // A small per-hop penalty keeps routes from threading a hundred tiny
            // junction connectors when a straight run would do.
            cost += 0.6;
            if (dist[size_t(n)] + cost < dist[size_t(nx)] - 1e-9) {
                dist[size_t(nx)] = dist[size_t(n)] + cost;
                prev[size_t(nx)] = n;
                pq.push({dist[size_t(nx)], nx});
            }
        }
    }

    // Pick a destination among the genuinely-far reachable lanes, so trips are
    // long enough to require real lane planning.
    // A destination has to be somewhere a trip can legitimately end, and far
    // enough away that the trip needs planning.
    auto usableDestination = [&](size_t i) {
        if (dist[i] > 1e299) return false;
        if (nodes[i].junctionId >= 0) return false;   // don't end inside a junction
        return nodes[i].length >= 15.0;
    };

    std::vector<int> candidates;
    double floorTime = std::max(15.0, furthest * 0.45);
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!usableDestination(i) || dist[i] < floorTime) continue;
        candidates.push_back(int(i));
    }
    if (candidates.empty()) {
        // Nothing far enough. That used to mean no route at all, which quietly
        // switched routing OFF for 38% of all traffic: a vehicle starting on a
        // lane that runs off the edge of the map, or into a short cul-de-sac, has
        // nowhere 45% of the way to "furthest" because furthest is itself tiny.
        // Those drivers still have somewhere to go — just not far — and a short
        // route is a much better model of them than none.
        double best = -1;
        int bestNode = -1;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!usableDestination(i)) continue;
            if (dist[i] > best) {
                best = dist[i];
                bestNode = int(i);
            }
        }
        if (bestNode < 0 || bestNode == fromNode) {
            // Genuinely nowhere to go: an isolated road with no successors, which
            // is what the `tiers` demo is by construction.
            RL_FALLBACK("planRoute no reachable destination at all -> vehicle drives unrouted");
            return path;
        }
        RL_FALLBACK("planRoute nothing past the distance floor -> shortest available trip");
        candidates.push_back(bestNode);
    }
    int dest = candidates[size_t(rng_.rangeI(0, int(candidates.size()) - 1))];
    for (int n = dest; n >= 0; n = prev[size_t(n)]) path.push_back(n);
    std::reverse(path.begin(), path.end());
    ++replans_;
    return path;
}

void Simulation::assignRoute(Vehicle& v) {
    v.routeCheckIn = 0;
    v.cachedDemandRoad = -1;
    v.cachedDemandDist = 1e9;
    if (!params_.routing) {
        v.route.clear();
        v.routePos = 0;
        v.targetRoad = -1;
        return;
    }
    v.route = planRoute(v.lane);
    v.routePos = 0;
    v.targetRoad = v.route.empty() ? -1 : net_.lanes().nodes[size_t(v.route.back())].ref.road;
}

int Simulation::routeDemand(const Vehicle& v, double& distance) const {
    distance = 1e9;
    if (v.route.empty() || v.routePos + 1 >= v.route.size()) return -1;
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    const LaneNode& cur = nodes[size_t(v.lane)];

    // Walk forward along the plan accumulating distance. The interesting road is
    // the first one the CURRENT lane cannot reach — that is the thing the driver
    // has to do something about, and how far away it is sets the urgency.
    double ahead = std::max(0.0, cur.length - v.travelled);
    for (size_t i = v.routePos + 1; i < v.route.size(); ++i) {
        int node = v.route[i];
        int road = nodes[size_t(node)].ref.road;
        if (road != cur.ref.road &&
            !net_.lanes().reaches(v.lane, road, params_.routeMaxDepth)) {
            distance = ahead;
            return road;
        }
        ahead += nodes[size_t(node)].length;
        if (ahead > params_.routeLookahead) break;
    }
    return -1;
}

bool Simulation::pedestrianMayCross(int linkIndex, double patience) const {
    if (linkIndex < 0 || size_t(linkIndex) >= walkLinks_.size()) return false;
    return crossingClear(walkLinks_[size_t(linkIndex)], patience);
}

bool Simulation::crossingClear(const WalkLink& link, double patience) const {
    if (!link.crossing) return true;

    // Signalised: the pedestrian reads the SAME phase table the drivers obey. A
    // crossing is walkable when no movement using this arm's carriageway is
    // green — which is derived, not a separate pedestrian signal plan someone
    // had to keep in sync.
    if (link.junction >= 0 && link.junction < net_.junctionCount()) {
        const Junction& j = net_.junction(link.junction);
        if (j.control == JunctionControl::Signalized && !j.phases.empty()) {
            for (size_t ci = 0; ci < j.connections.size(); ++ci) {
                const Connection& c = j.connections[ci];
                if (c.from.road != link.road && c.to.road != link.road) continue;
                if (j.isGreen(int(ci), time_)) return false;
            }
            return true;
        }
    }

    // Otherwise: gap acceptance against the traffic that would actually hit
    // them. Patience shrinks the gap they will take, which is why a pedestrian
    // at a busy kerb eventually steps out rather than waiting forever.
    double need = params_.pedCrossGap * (1.0 - 0.7 * saturate(patience / params_.pedMaxWait));
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    for (const Vehicle& v : vehicles_) {
        if (!v.active || v.lane < 0) continue;
        const LaneNode& n = nodes[size_t(v.lane)];
        if (n.ref.road != link.road) continue;
        double station = n.roadS(v.travelled);
        // Only traffic approaching the crossing matters.
        double ahead = (link.s - station) * (n.dir > 0 ? 1.0 : -1.0);
        if (ahead < -1.0 || ahead > 90.0) continue;
        double eta = ahead / std::max(1.0, v.speed);
        if (eta < need) return false;
    }
    return true;
}

bool Simulation::blockedByCrossing(const Vehicle& v, double& distance) const {
    if (!params_.vehiclesYieldToPeds || occupiedCrossings_.empty()) return false;
    const LaneNode& n = net_.lanes().nodes[size_t(v.lane)];
    double station = n.roadS(v.travelled);
    for (const ActiveCrossing& c : occupiedCrossings_) {
        if (c.road != n.ref.road) continue;
        double ahead = (c.s - station) * (n.dir > 0 ? 1.0 : -1.0);
        // Stop short of the zebra, not on it.
        if (ahead < 0.5 || ahead > 60.0) continue;
        distance = ahead - 2.0;
        return true;
    }
    return false;
}

int Simulation::chooseSuccessor(Vehicle& v) {
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    const LaneNode& n = nodes[size_t(v.lane)];
    if (n.successors.empty()) return -1;

    // Follow the plan when the plan is still followable.
    if (!v.route.empty()) {
        // Re-sync: the vehicle may have changed lanes since the route was made.
        for (size_t i = v.routePos; i < v.route.size(); ++i) {
            if (v.route[i] == v.lane) {
                v.routePos = i;
                break;
            }
        }
        if (v.routePos + 1 < v.route.size()) {
            int want = v.route[v.routePos + 1];
            for (int s : n.successors) {
                if (s == want) {
                    ++v.routePos;
                    return s;
                }
            }
        }
        // The plan no longer connects from here — lane changes have taken the
        // vehicle off it. Re-plan from where it actually is.
        v.route = planRoute(v.lane);
        v.routePos = 0;
        v.targetRoad = v.route.empty() ? -1 : nodes[size_t(v.route.back())].ref.road;
        if (v.route.size() > 1) {
            for (int s : n.successors)
                if (s == v.route[1]) {
                    v.routePos = 1;
                    return s;
                }
        }
    }
    return n.successors[size_t(rng_.rangeI(0, int(n.successors.size()) - 1))];
}

void Simulation::considerLaneChange(Vehicle& v) {
    if (v.changingTo >= 0) return;
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    const LaneNode& n = nodes[size_t(v.lane)];
    if (n.junctionId >= 0) return;

    double toEnd = n.length - v.travelled;

    // MANDATORY, reason one: this lane goes nowhere. The taper the shader draws
    // and the "successors is empty" the sim reads are the same fact, so the
    // driver is never surprised by paint that says merge while the graph says
    // continue.
    bool laneEnds = n.successors.empty() && toEnd < 220.0;

    // MANDATORY, reason two: the lane still goes somewhere, just not where this
    // trip is going. THIS is the query the two-resolution graph exists for —
    // "I need the third exit, so I must be in one of these lanes by then" — and
    // it is answered by walking the plan, not by a special case per junction.
    // Refresh the (graph-searching) route demand on a timer rather than every
    // tick; in between, the distance simply shrinks as the vehicle drives.
    if (v.routeCheckIn <= 0.0) {
        v.cachedDemandRoad = routeDemand(v, v.cachedDemandDist);
        v.routeCheckIn = 0.4;
    }
    double demandDistance = v.cachedDemandDist;
    int demandRoad = v.cachedDemandRoad;
    bool routeForces = demandRoad >= 0 && demandDistance < params_.routeLookahead;
    v.routeUrgency = routeForces ? saturate(1.0 - demandDistance / params_.routeLookahead) : 0.0;

    bool mustLeave = laneEnds || routeForces;

    struct Option {
        int node;
        bool crossable;
    };
    Option options[2] = {{n.leftNeighbor, n.leftCrossable}, {n.rightNeighbor, n.rightCrossable}};

    double bestScore = mustLeave ? -1e9 : 0.6;   // discretionary needs a real win
    int best = -1;
    for (const Option& opt : options) {
        if (opt.node < 0) continue;
        // A solid line is not crossable. Same field as the paint. Even a driver
        // who NEEDS the lane will not cross a solid line here; that is what
        // makes a missed exit possible, which is correct behaviour.
        if (!opt.crossable) continue;
        const LaneNode& tgt = nodes[size_t(opt.node)];
        if (!(tgt.access & kAccessCar)) continue;
        if (laneEnds && tgt.successors.empty()) continue;
        // A neighbour that cannot serve the plan either is no help.
        if (routeForces && !net_.lanes().reaches(opt.node, demandRoad, params_.routeMaxDepth))
            continue;

        double leadSpeed = 0, followSpeed = 0;
        double gapAhead = 1e9;
        for (int idx : laneOccupants_[size_t(opt.node)]) {
            const Vehicle& o = vehicles_[size_t(idx)];
            if (o.travelled > v.travelled) {
                gapAhead = o.travelled - v.travelled - 0.5 * (v.length + o.length);
                leadSpeed = o.speed;
                break;
            }
        }
        double gapBack = gapBehind(opt.node, v.travelled, followSpeed);
        gapBack -= 0.5 * v.length + 2.5;

        // Gap acceptance: room in front, and room behind measured against how
        // fast the follower is closing. A driver running out of road accepts a
        // tighter gap than one merely looking for a faster lane.
        double squeeze = 1.0 - 0.55 * std::max(v.routeUrgency, laneEnds ? 0.8 : 0.0);
        double needBack = (4.0 + std::max(0.0, followSpeed - v.speed) * 1.6) * squeeze;
        if (gapAhead < (6.0 + v.speed * 0.6) * squeeze || gapBack < needBack) continue;

        double myGapSpeed = 0;
        double myGap = gapToLeader(v, myGapSpeed);
        double score = (gapAhead - myGap) * 0.02 + (leadSpeed - myGapSpeed) * 0.35;
        if (laneEnds) score += 1000.0 / std::max(8.0, toEnd);
        if (routeForces) score += 1000.0 * (0.2 + v.routeUrgency);
        if (score > bestScore) {
            bestScore = score;
            best = opt.node;
        }
    }
    if (best >= 0) {
        v.changingTo = best;
        v.changeTime = 0;
        ++laneChanges_;
        if (mustLeave) ++mandatory_;
    }
}

void Simulation::respawn(Vehicle& v) {
    if (!params_.spawnRespawn) {
        v.active = false;
        return;
    }
    // Find a start that is actually empty; dropping a car on top of another is
    // the easiest way to make the whole queue model nonsense.
    int lane = -1;
    double at = 0.5;
    for (int attempt = 0; attempt < 24 && lane < 0; ++attempt) {
        int cand = randomStartLane();
        if (cand < 0) break;
        double want = 1.0;
        bool clear = true;
        for (int idx : laneOccupants_[size_t(cand)]) {
            const Vehicle& o = vehicles_[size_t(idx)];
            if (o.active && std::fabs(o.travelled - want) < 0.5 * (v.length + o.length) + 6.0)
                clear = false;
        }
        if (clear) {
            lane = cand;
            at = want;
        }
    }
    if (lane < 0) {
        RL_FALLBACK("respawn found no empty start lane -> vehicle retired");
        v.active = false;
        return;
    }
    v.lane = lane;
    v.travelled = at;
    v.speed = net_.lanes().nodes[size_t(lane)].speedLimit / 3.6 * 0.6;
    v.changingTo = -1;
    v.lateral = 0;
    v.waited = 0;
    v.routeUrgency = 0;
    v.routeCheckIn = 0;
    v.cachedDemandRoad = -1;
    v.cachedDemandDist = 1e9;
    assignRoute(v);
}

void Simulation::step(double dt) {
    if (net_.lanes().nodes.empty()) return;
    rebuildOccupancy();
    updateJunctionState();
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;

    for (Vehicle& v : vehicles_) {
        if (!v.active || v.lane < 0) continue;

        double v0 = desiredSpeedFor(v);
        double leaderSpeed = 1e3;
        double gap = gapToLeader(v, leaderSpeed);

        double stopDist = 0;
        if (blockedByJunction(v, stopDist)) {
            if (stopDist < gap) {
                gap = stopDist;
                leaderSpeed = 0.0;
            }
        }
        // Someone is on the zebra. The crossing the driver stops for is the same
        // crossing the shader painted and the pedestrian graph walked.
        double pedDist = 0;
        if (blockedByCrossing(v, pedDist) && pedDist < gap) {
            gap = std::max(0.4, pedDist);
            leaderSpeed = 0.0;
        }

        // IDM.
        double a = params_.idmMaxAccel;
        double b = params_.idmComfortDecel;
        double dv = v.speed - std::min(leaderSpeed, 1e3);
        double sStar = params_.idmMinGap + std::max(0.0, v.speed * params_.idmHeadway +
                                                            v.speed * dv /
                                                                (2.0 * std::sqrt(a * b)));
        double sGap = std::max(0.4, gap);
        double accel = a * (1.0 - std::pow(v.speed / v0, 4.0) - (sStar / sGap) * (sStar / sGap));
        accel = clampd(accel, -6.5, a);

        v.speed = std::max(0.0, v.speed + accel * dt);
        if (v.speed < 0.5) v.waited += dt;
        else v.waited = std::max(0.0, v.waited - dt * 0.5);

        v.routeCheckIn -= dt;
        v.cachedDemandDist -= v.speed * dt;
        considerLaneChange(v);

        if (v.changingTo >= 0) {
            v.changeTime += dt;
            double u = saturate(v.changeTime / params_.laneChangeDuration);
            // Lateral offset for the render; the vehicle commits to the new lane
            // at the halfway point so leader/follower stays consistent.
            const LaneNode& from = nodes[size_t(v.lane)];
            const LaneNode& to = nodes[size_t(v.changingTo)];
            double tFrom = net_.road(from.ref.road).laneCenterT(from.ref.lane, roadStationOf(v));
            double tTo = net_.road(to.ref.road).laneCenterT(to.ref.lane, roadStationOf(v));
            double delta = tTo - tFrom;
            if (u >= 0.5 && v.lane != v.changingTo) {
                // The gap was checked when the manoeuvre started, over a second
                // ago. Re-check before committing: the target lane may have
                // closed up since, and a driver would abort rather than merge
                // into the back of someone.
                bool clear = true;
                for (int idx : laneOccupants_[size_t(v.changingTo)]) {
                    const Vehicle& o = vehicles_[size_t(idx)];
                    if (!o.active || o.id == v.id) continue;
                    double gap = std::fabs(o.travelled - v.travelled) -
                                 0.5 * (v.length + o.length);
                    if (gap < 0.5) {
                        clear = false;
                        break;
                    }
                }
                if (!clear) {
                    v.changingTo = -1;
                    v.lateral = 0;
                } else {
                    double travelled = v.travelled;
                    v.lane = v.changingTo;
                    v.travelled = clampd(travelled, 0.0, std::max(0.5, laneLength(v.lane)));
                    v.lateral = -delta * (1.0 - u);
                }
            } else {
                v.lateral = delta * u;
            }
            if (u >= 1.0) {
                v.changingTo = -1;
                v.lateral = 0;
            }
        }

        v.travelled += v.speed * dt;
        double len = laneLength(v.lane);
        while (v.travelled > len) {
            // Arrived: the plan is spent. A real trip ends here; this one starts
            // a new leg from where it is, which keeps the population steady
            // without teleporting anybody.
            if (!v.route.empty() && v.routePos + 1 >= v.route.size()) {
                ++completed_;
                assignRoute(v);
                if (v.route.size() < 2) {
                    respawn(v);
                    break;
                }
            }
            int next = chooseSuccessor(v);
            if (next < 0) {
                // Ran out of lane. In a real game this is a despawn or a
                // network-boundary handoff; here it recycles.
                ++completed_;
                respawn(v);
                break;
            }
            v.travelled -= len;
            v.lane = next;
            v.changingTo = -1;
            v.lateral = 0;
            len = laneLength(v.lane);
            if (len < 1e-3) {
                respawn(v);
                break;
            }
        }
    }

    stepPedestrians(dt);
    time_ += dt;
}

void Simulation::stepPedestrians(double dt) {
    occupiedCrossings_.clear();
    if (walkNodes_.empty()) return;

    for (Pedestrian& p : peds_) {
        if (!p.active) continue;

        // Mid-crossing: keep going, and stay visible to drivers while doing it.
        if (p.crossingLink >= 0) {
            const WalkLink& link = walkLinks_[size_t(p.crossingLink)];
            p.crossProgress += p.speed * dt / std::max(1.0, link.length);
            occupiedCrossings_.push_back({link.road, link.s});
            if (p.crossProgress >= 1.0) {
                ++crossingsMade_;
                int arrive = (p.node == link.a) ? link.b : link.a;
                const WalkNode& n = walkNodes_[size_t(arrive)];
                p.node = arrive;
                p.road = n.road;
                p.side = n.side;
                p.s = n.atEnd ? net_.road(n.road).end() - 0.5 : net_.road(n.road).begin() + 0.5;
                // Arriving at a footway end means turning to walk back down it.
                p.dir = n.atEnd ? -1.0 : 1.0;
                p.crossingLink = -1;
                p.crossProgress = 0;
                p.waiting = 0;
            }
            continue;
        }

        const Road& r = net_.road(p.road);
        double target = p.dir > 0 ? r.end() - 0.5 : r.begin() + 0.5;
        double before = p.s;
        p.s += p.dir * p.speed * dt;
        bool arrived = (p.dir > 0) ? (p.s >= target) : (p.s <= target);
        if (!arrived) {
            (void)before;
            continue;
        }
        p.s = target;

        // At a footway end: pick a link out. Corners are free; a crossing has to
        // be safe first, and until it is the pedestrian stands at the kerb.
        int node = findWalkNode(p.road, p.side, p.dir > 0);
        if (node < 0) {
            p.dir = -p.dir;
            continue;
        }
        p.node = node;
        const std::vector<int>& links = walkLinksOf_[size_t(node)];
        if (links.empty()) {
            p.dir = -p.dir;
            continue;
        }

        // A pedestrian who has decided to cross COMMITS to it and waits at the
        // kerb. Re-rolling the choice every tick would mean nobody ever waits,
        // which quietly removes the interesting half of the behaviour.
        int chosen = p.pendingLink;
        if (chosen < 0) {
            for (int attempt = 0; attempt < 6 && chosen < 0; ++attempt) {
                int cand = links[size_t(rng_.rangeI(0, int(links.size()) - 1))];
                const WalkLink& cl = walkLinks_[size_t(cand)];
                if (cl.crossing && !rng_.chance(0.5)) continue;
                chosen = cand;
            }
            if (chosen < 0) chosen = links[0];
        }

        const WalkLink& link = walkLinks_[size_t(chosen)];
        if (link.crossing) {
            if (!crossingClear(link, p.waiting)) {
                p.pendingLink = chosen;
                p.waiting += dt;
                pedWaitTotal_ += dt;
                continue;   // stand at the kerb
            }
            p.crossingLink = chosen;
            p.pendingLink = -1;
            p.crossProgress = 0;
            p.waiting = 0;
            continue;
        }
        p.pendingLink = -1;

        int other = (link.a == node) ? link.b : link.a;
        const WalkNode& n = walkNodes_[size_t(other)];
        p.node = other;
        p.road = n.road;
        p.side = n.side;
        p.s = n.atEnd ? net_.road(n.road).end() - 0.5 : net_.road(n.road).begin() + 0.5;
        p.dir = n.atEnd ? -1.0 : 1.0;
        p.waiting = 0;
    }
}

void Simulation::run(double seconds, double dt) {
    int steps = std::max(1, int(seconds / dt));
    for (int i = 0; i < steps; ++i) step(dt);
}

bool Simulation::vehiclePose(const Vehicle& v, Vec3& pos, double& yaw) const {
    if (!v.active || v.lane < 0) return false;
    const LaneNode& n = net_.lanes().nodes[size_t(v.lane)];
    const Road& r = net_.road(n.ref.road);
    double s = clampd(n.roadS(v.travelled), r.begin(), r.end());
    double t = r.laneCenterT(n.ref.lane, s) + v.lateral;
    pos = r.surfacePoint(s, t) + Vec3{0, 0.72, 0};
    Frame f = r.spine.frameAt(s);
    yaw = n.dir > 0 ? f.heading : wrapPi(f.heading + kPi);
    return true;
}

bool Simulation::pedestrianPose(const Pedestrian& p, Vec3& pos, double& yaw) const {
    if (!p.active || p.road < 0) return false;
    if (p.crossingLink >= 0) {
        const WalkLink& link = walkLinks_[size_t(p.crossingLink)];
        Vec3 a = walkNodes_[size_t(link.a)].pos;
        Vec3 b = walkNodes_[size_t(link.b)].pos;
        if (p.node == link.b) std::swap(a, b);
        pos = lerp(a, b, saturate(p.crossProgress)) + Vec3{0, 0.88, 0};
        Vec3 d = b - a;
        yaw = std::atan2(d.z, d.x);
        return true;
    }
    const Road& r = net_.road(p.road);
    const LaneSection& sec = r.xs.sectionAt(p.s);
    const std::vector<Strip>& stack = p.side > 0 ? sec.left : sec.right;
    double ds = p.s - sec.s0, acc = 0, t = 0;
    bool found = false;
    for (const Strip& st : stack) {
        double w = std::max(0.0, st.width.eval(ds));
        if (st.kind == StripKind::Sidewalk && w > 0.5) {
            t = (p.side > 0 ? 1.0 : -1.0) * (acc + w * 0.5);
            found = true;
            break;
        }
        acc += w;
    }
    if (!found) return false;
    pos = r.surfacePoint(p.s, t) + Vec3{0, 0.88, 0};
    Frame f = r.spine.frameAt(p.s);
    yaw = p.dir > 0 ? f.heading : wrapPi(f.heading + kPi);
    return true;
}

SimStats Simulation::stats() const {
    SimStats st;
    double sum = 0, wait = 0;
    for (const Vehicle& v : vehicles_) {
        if (!v.active) continue;
        ++st.vehicles;
        sum += v.speed;
        wait += v.waited;
        if (v.speed < 0.5) ++st.stopped;
        else ++st.moving;
    }
    st.meanSpeedKph = st.vehicles ? sum / st.vehicles * 3.6 : 0.0;
    st.meanWait = st.vehicles ? wait / st.vehicles : 0.0;
    st.laneChanges = laneChanges_;
    st.mandatoryChanges = mandatory_;
    st.completedTrips = completed_;
    st.replans = replans_;
    for (const Vehicle& v : vehicles_) {
        if (!v.active) continue;
        double d = 0;
        if (routeDemand(v, d) >= 0 && d < params_.routeLookahead) ++st.offRoute;
    }
    st.pedestrians = int(peds_.size());
    for (const Pedestrian& p : peds_) {
        if (!p.active) continue;
        if (p.crossingLink >= 0) ++st.pedsCrossing;
        else if (p.pendingLink >= 0) ++st.pedsWaiting;
    }
    st.meanPedWait = peds_.empty() ? 0.0 : pedWaitTotal_ / double(peds_.size());
    st.crossingsMade = crossingsMade_;
    st.parkedCars = int(parked_.size());
    return st;
}

void tessellateAgents(const Simulation& sim, Mesh& out) {
    for (const Vehicle& v : sim.vehicles()) {
        Vec3 pos;
        double yaw = 0;
        if (!sim.vehiclePose(v, pos, yaw)) continue;
        emitBox(out, pos, {v.length * 0.5, 0.62, v.width * 0.5}, yaw, v.color, 0.35);
        // A slab of cabin, so direction of travel reads in a top-down shot.
        emitBox(out, pos + Vec3{0, 0.62, 0}, {v.length * 0.28, 0.30, v.width * 0.42}, yaw,
                v.color * 0.7, 0.25);
    }
    for (const ParkedCar& p : sim.parkedCars()) {
        emitBox(out, p.pos, {2.35, 0.60, 0.92}, p.yaw, p.color, 0.4);
        emitBox(out, p.pos + Vec3{0, 0.58, 0}, {1.3, 0.28, 0.82}, p.yaw, p.color * 0.7, 0.3);
    }
    for (const Pedestrian& p : sim.pedestrians()) {
        Vec3 pos;
        double yaw = 0;
        if (!sim.pedestrianPose(p, pos, yaw)) continue;
        emitBox(out, pos, {0.22, 0.42, 0.22}, yaw, p.color, 0.8);
        emitBox(out, pos + Vec3{0, 0.52, 0}, {0.16, 0.14, 0.16}, yaw, {0.6, 0.48, 0.40}, 0.8);
    }
}

}  // namespace roadlab
