#include "sim.h"

#include <algorithm>

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
    rebuildOccupancy();
}

void Simulation::seedPedestrians(int count) {
    std::vector<int> footwayRoads;
    for (const Road& r : net_.roads()) {
        if (!r.allowsPedestrians || r.kind == RoadKind::Connector) continue;
        const LaneSection& sec = r.xs.sectionAt(r.begin() + 1.0);
        bool has = false;
        for (const std::vector<Strip>* st : {&sec.left, &sec.right})
            for (const Strip& x : *st)
                if (x.kind == StripKind::Sidewalk) has = true;
        if (has && r.activeLength() > 15.0) footwayRoads.push_back(r.id);
    }
    if (footwayRoads.empty()) return;
    for (int i = 0; i < count; ++i) {
        Pedestrian p;
        p.road = footwayRoads[size_t(rng_.rangeI(0, int(footwayRoads.size()) - 1))];
        const Road& r = net_.road(p.road);
        p.side = rng_.chance(0.5) ? 1 : -1;
        p.s = rng_.range(r.begin() + 1.0, r.end() - 1.0);
        p.dir = rng_.chance(0.5) ? 1.0 : -1.0;
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

int Simulation::chooseSuccessor(int node) {
    const LaneNode& n = net_.lanes().nodes[size_t(node)];
    if (n.successors.empty()) return -1;
    return n.successors[size_t(rng_.rangeI(0, int(n.successors.size()) - 1))];
}

void Simulation::considerLaneChange(Vehicle& v) {
    if (v.changingTo >= 0) return;
    const std::vector<LaneNode>& nodes = net_.lanes().nodes;
    const LaneNode& n = nodes[size_t(v.lane)];
    if (n.junctionId >= 0) return;

    double toEnd = n.length - v.travelled;
    // MANDATORY: this lane goes nowhere. The taper the shader draws and the
    // "successors is empty" the sim reads are the same fact, so the driver is
    // never surprised by paint that says merge while the graph says continue.
    bool mustLeave = n.successors.empty() && toEnd < 220.0;

    struct Option {
        int node;
        bool crossable;
    };
    Option options[2] = {{n.leftNeighbor, n.leftCrossable}, {n.rightNeighbor, n.rightCrossable}};

    double bestScore = mustLeave ? -1e9 : 0.6;   // discretionary needs a real win
    int best = -1;
    for (const Option& opt : options) {
        if (opt.node < 0) continue;
        // A solid line is not crossable. Same field as the paint.
        if (!opt.crossable && !mustLeave) continue;
        const LaneNode& tgt = nodes[size_t(opt.node)];
        if (!(tgt.access & kAccessCar)) continue;
        if (mustLeave && tgt.successors.empty()) continue;

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
        // fast the follower is closing.
        double needBack = 4.0 + std::max(0.0, followSpeed - v.speed) * 1.6;
        if (gapAhead < 6.0 + v.speed * 0.6 || gapBack < needBack) continue;

        double myGapSpeed = 0;
        double myGap = gapToLeader(v, myGapSpeed);
        double score = (gapAhead - myGap) * 0.02 + (leadSpeed - myGapSpeed) * 0.35;
        if (mustLeave) score += 1000.0 / std::max(8.0, toEnd);
        if (score > bestScore) {
            bestScore = score;
            best = opt.node;
        }
    }
    if (best >= 0) {
        v.changingTo = best;
        v.changeTime = 0;
        ++laneChanges_;
    }
}

void Simulation::respawn(Vehicle& v) {
    ++completed_;
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
        v.active = false;
        return;
    }
    v.lane = lane;
    v.travelled = at;
    v.speed = net_.lanes().nodes[size_t(lane)].speedLimit / 3.6 * 0.6;
    v.changingTo = -1;
    v.lateral = 0;
    v.waited = 0;
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
            int next = chooseSuccessor(v.lane);
            if (next < 0) {
                // Ran out of lane. In a real game this is a despawn or a
                // network-boundary handoff; here it recycles.
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

    for (Pedestrian& p : peds_) {
        if (!p.active) continue;
        const Road& r = net_.road(p.road);
        p.s += p.dir * p.speed * dt;
        if (p.s > r.end() - 0.5 || p.s < r.begin() + 0.5) {
            p.dir = -p.dir;
            p.s = clampd(p.s, r.begin() + 0.5, r.end() - 0.5);
        }
    }

    time_ += dt;
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
    st.completedTrips = completed_;
    st.pedestrians = int(peds_.size());
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
