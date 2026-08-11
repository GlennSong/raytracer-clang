#include "network.h"

#include "junction.h"
#include <cstdio>
#include <map>
#include <functional>
#include <queue>

namespace roadlab {

// --- Road -----------------------------------------------------------------

Vec3 Road::surfacePoint(double s, double t) const {
    return spine.toWorld(s, t, xs.heightAt(s, t));
}

Vec3 Road::surfaceNormal(double s, double t) const {
    // Differenced rather than analytic: the cross-section height is piecewise
    // (kerb faces are ramps between plateaus), so a closed-form normal would
    // have to special-case every strip kind. Differencing gets kerbs right for
    // free.
    const double dt = 0.05, ds = 0.5;
    Vec3 a = surfacePoint(s, t - dt), b = surfacePoint(s, t + dt);
    Vec3 c = surfacePoint(std::max(begin(), s - ds), t);
    Vec3 d = surfacePoint(std::min(end(), s + ds), t);
    Vec3 n = normalize(cross(d - c, b - a));
    return n.y < 0 ? -n : n;
}

double Road::laneCenterT(int laneId, double s) const {
    return xs.laneCenterT(xs.sectionIndexAt(s), laneId, s);
}

bool Road::lanePose(int laneId, double s, Vec2& pos, double& heading) const {
    int si = xs.sectionIndexAt(s);
    const LaneSection* sec = si >= 0 ? &xs.sections[size_t(si)] : nullptr;
    if (!sec) return false;
    const Strip* st = sec->strip(laneId);
    if (!st) return false;
    double t = xs.laneCenterT(si, laneId, s);
    Frame f = spine.frameAt(s);
    pos = f.planPos + perpLeft(dirOf(f.heading)) * t;
    heading = st->dir >= 0 ? f.heading : wrapPi(f.heading + kPi);
    return true;
}

double Road::speedLimitOf(int laneId, double s) const {
    int si = xs.sectionIndexAt(s);
    if (si < 0) return designSpeed;
    const Strip* st = xs.sections[size_t(si)].strip(laneId);
    if (st && st->speedLimit > 0.5f) return st->speedLimit;
    return designSpeed;
}

// --- end lanes ------------------------------------------------------------

EndLanes endLanes(const Road& r, bool atEnd) {
    EndLanes out;
    double s = atEnd ? r.end() : r.begin();
    int si = r.xs.sectionIndexAt(atEnd ? std::max(r.begin(), s - 1e-3) : s + 1e-3);
    if (si < 0) return out;
    const LaneSection& sec = r.xs.sections[size_t(si)];

    struct Entry {
        int id;
        double t;
    };
    std::vector<Entry> in, outg;
    auto consider = [&](const Strip& st) {
        if (!st.isLane() || st.dir == 0) return;
        // Zero-width here means the lane has already tapered away; it cannot
        // carry traffic through this end.
        if (r.xs.laneWidthAt(si, st.id, s) < 0.5) return;
        double t = r.xs.laneCenterT(si, st.id, s);
        bool reachesEnd = atEnd ? (st.dir > 0) : (st.dir < 0);
        (reachesEnd ? in : outg).push_back({st.id, t});
    };
    for (const Strip& st : sec.left) consider(st);
    for (const Strip& st : sec.right) consider(st);

    // Order right-to-left IN THE TRAVEL FRAME, so "rightmost lane" means the
    // same thing to the junction builder no matter which way the reference line
    // happens to run. A driver arriving at the high-s end travelling +s has
    // their right hand toward -t.
    auto sortTravelRightToLeft = [](std::vector<Entry>& v, bool rightIsNegativeT) {
        std::sort(v.begin(), v.end(), [&](const Entry& a, const Entry& b) {
            return rightIsNegativeT ? a.t < b.t : a.t > b.t;
        });
    };
    sortTravelRightToLeft(in, atEnd);
    sortTravelRightToLeft(outg, !atEnd);
    for (const Entry& e : in) out.incoming.push_back(e.id);
    for (const Entry& e : outg) out.outgoing.push_back(e.id);
    return out;
}

// --- lane graph -----------------------------------------------------------

int LaneGraph::find(const LaneRef& ref) const {
    Key k{ref.road, ref.section, ref.lane};
    auto it = std::lower_bound(
        index_.begin(), index_.end(), k,
        [](const std::pair<Key, int>& a, const Key& b) { return a.first < b; });
    if (it != index_.end() && !(k < it->first) && !(it->first < k)) return it->second;
    return -1;
}

void LaneGraph::addIndex(const LaneRef& ref, int node) {
    Key k{ref.road, ref.section, ref.lane};
    auto it = std::lower_bound(
        index_.begin(), index_.end(), k,
        [](const std::pair<Key, int>& a, const Key& b) { return a.first < b; });
    index_.insert(it, {k, node});
}

std::vector<int> LaneGraph::lanesReaching(int fromNode, int targetRoad, int maxDepth) const {
    std::vector<int> result;
    if (fromNode < 0 || size_t(fromNode) >= nodes.size()) return result;
    const LaneNode& start = nodes[size_t(fromNode)];

    // Which lanes of the same road+section can get to targetRoad? That is the
    // question a driver is actually asking 400 m before their exit.
    auto reaches = [&](int node) {
        std::vector<char> seen(nodes.size(), 0);
        std::queue<std::pair<int, int>> q;
        q.push({node, 0});
        seen[size_t(node)] = 1;
        while (!q.empty()) {
            auto [n, d] = q.front();
            q.pop();
            if (nodes[size_t(n)].ref.road == targetRoad) return true;
            if (d >= maxDepth) continue;
            for (int nx : nodes[size_t(n)].successors) {
                if (!seen[size_t(nx)]) {
                    seen[size_t(nx)] = 1;
                    q.push({nx, d + 1});
                }
            }
        }
        return false;
    };

    for (size_t i = 0; i < nodes.size(); ++i) {
        const LaneNode& n = nodes[i];
        if (n.ref.road != start.ref.road || n.ref.section != start.ref.section) continue;
        if (n.dir != start.dir) continue;
        if (reaches(int(i))) result.push_back(int(i));
    }
    return result;
}

// --- network --------------------------------------------------------------

int Network::addRoad(Road r) {
    r.id = int(roads_.size());
    if (!r.spine.finalized()) r.spine.finalize();
    if (r.xs.sections.empty()) {
        LaneSection sec = roadPreset("street2").section;
        sec.s0 = 0;
        r.xs.sections.push_back(sec);
    }
    r.xs.finalize(r.spine.length());
    roads_.push_back(std::move(r));
    return roads_.back().id;
}

int Network::addJunction(const Junction& j) {
    Junction copy = j;
    copy.id = int(junctions_.size());
    junctions_.push_back(copy);
    return copy.id;
}

Junction& Network::junction(int id) { return junctions_[size_t(id)]; }
const Junction& Network::junction(int id) const { return junctions_[size_t(id)]; }

int Network::splitRoad(int roadId, double s) {
    if (roadId < 0 || size_t(roadId) >= roads_.size()) return -1;
    double b0 = roads_[size_t(roadId)].begin();
    double b1 = roads_[size_t(roadId)].end();
    if (s <= b0 + 1e-3 || s >= b1 - 1e-3) return -1;

    Road tail = roads_[size_t(roadId)];   // copies the geometry; both keep whole s
    tail.id = int(roads_.size());
    tail.name = roads_[size_t(roadId)].name + ".b";
    tail.sBegin = s;
    tail.sEnd = b1;
    tail.pred = RoadLink{LinkType::Road, roadId, false, {}};
    tail.succ = roads_[size_t(roadId)].succ;

    // Anything that used to attach to the head's far end now attaches to the
    // tail's far end.
    RoadLink oldSucc = roads_[size_t(roadId)].succ;
    if (oldSucc.type == LinkType::Road && oldSucc.id >= 0) {
        Road& far = roads_[size_t(oldSucc.id)];
        if (far.pred.type == LinkType::Road && far.pred.id == roadId) far.pred.id = tail.id;
        if (far.succ.type == LinkType::Road && far.succ.id == roadId) far.succ.id = tail.id;
    }
    for (Junction& j : junctions_) {
        for (JunctionArm& a : j.arms) {
            if (a.road == roadId && a.atEnd) a.road = tail.id;
        }
    }

    roads_[size_t(roadId)].sEnd = s;
    roads_[size_t(roadId)].succ = RoadLink{LinkType::Road, tail.id, true, {}};
    roads_.push_back(std::move(tail));
    return roads_.back().id;
}

void Network::build() {
    for (Junction& j : junctions_) buildJunction(*this, j);
    buildLaneGraph();
}

void Network::linkRoadToRoad(int roadA, bool aAtEnd, int roadB, bool bAtStart,
                             const std::vector<std::pair<int, int>>& overrides) {
    const Road& A = roads_[size_t(roadA)];
    const Road& B = roads_[size_t(roadB)];
    EndLanes ea = endLanes(A, aAtEnd);
    EndLanes eb = endLanes(B, !bAtStart ? true : false);

    double sa = aAtEnd ? A.end() : A.begin();
    double sb = bAtStart ? B.begin() : B.end();
    int seca = A.xs.sectionIndexAt(aAtEnd ? std::max(A.begin(), sa - 1e-3) : sa + 1e-3);
    int secb = B.xs.sectionIndexAt(bAtStart ? sb + 1e-3 : std::max(B.begin(), sb - 1e-3));

    auto connect = [&](int laneA, int laneB) {
        int na = lanes_.find({roadA, seca, laneA});
        int nb = lanes_.find({roadB, secb, laneB});
        if (na < 0 || nb < 0) return;
        lanes_.nodes[size_t(na)].successors.push_back(nb);
        lanes_.nodes[size_t(nb)].predecessors.push_back(na);
    };

    if (!overrides.empty()) {
        for (const auto& pr : overrides) connect(pr.first, pr.second);
        return;
    }
    // Default pairing: rightmost to rightmost. Lanes are already ordered
    // right-to-left in the travel frame, so the pairing is a zip — and a lane
    // that has no partner is a lane that ends, which the sim reads as a
    // mandatory lane change rather than a dead end it discovers too late.
    size_t n = std::min(ea.incoming.size(), eb.outgoing.size());
    for (size_t i = 0; i < n; ++i) connect(ea.incoming[i], eb.outgoing[i]);
    size_t m = std::min(eb.incoming.size(), ea.outgoing.size());
    for (size_t i = 0; i < m; ++i) {
        int nb = lanes_.find({roadB, secb, eb.incoming[i]});
        int na = lanes_.find({roadA, seca, ea.outgoing[i]});
        if (na < 0 || nb < 0) continue;
        lanes_.nodes[size_t(nb)].successors.push_back(na);
        lanes_.nodes[size_t(na)].predecessors.push_back(nb);
    }
}

void Network::buildLaneGraph() {
    lanes_.clear();

    // 1. A node per (road, section, lane) that carries traffic.
    for (const Road& r : roads_) {
        for (size_t si = 0; si < r.xs.sections.size(); ++si) {
            const LaneSection& sec = r.xs.sections[si];
            double s0 = std::max(sec.s0, r.begin());
            double s1 = std::min(sec.s0 + sec.length, r.end());
            if (s1 - s0 < 0.25) continue;
            auto emit = [&](const Strip& st) {
                if (!st.isLane() || st.dir == 0) return;
                LaneNode n;
                n.ref = {r.id, int(si), st.id};
                n.dir = st.dir;
                n.sStart = st.dir > 0 ? s0 : s1;
                n.sEnd = st.dir > 0 ? s1 : s0;
                n.length = s1 - s0;
                n.kind = st.kind;
                n.speedLimit = st.speedLimit > 0.5f ? st.speedLimit : float(r.designSpeed);
                n.access = st.access;
                n.junctionId = r.junctionId;
                lanes_.addIndex(n.ref, int(lanes_.nodes.size()));
                lanes_.nodes.push_back(n);
            };
            for (const Strip& st : sec.left) emit(st);
            for (const Strip& st : sec.right) emit(st);
        }
    }

    // 2. Neighbours inside a section, in the travel frame.
    for (const Road& r : roads_) {
        for (size_t si = 0; si < r.xs.sections.size(); ++si) {
            const LaneSection& sec = r.xs.sections[si];
            double mid = clampd(sec.s0 + sec.length * 0.5, r.begin(), r.end());
            struct Entry {
                int node;
                double t;
                int8_t dir;
                int id;
            };
            std::vector<Entry> lane;
            auto collect = [&](const Strip& st) {
                int n = lanes_.find({r.id, int(si), st.id});
                if (n < 0) return;
                lane.push_back({n, r.xs.laneCenterT(int(si), st.id, mid), st.dir, st.id});
            };
            for (const Strip& st : sec.left) collect(st);
            for (const Strip& st : sec.right) collect(st);
            std::sort(lane.begin(), lane.end(),
                      [](const Entry& a, const Entry& b) { return a.t < b.t; });

            // Crossing legality comes from the marking on the shared boundary —
            // the same field the shader paints, so a solid line is uncrossable in
            // the sim for exactly the reason it looks uncrossable on screen.
            for (size_t i = 0; i + 1 < lane.size(); ++i) {
                const Entry& lo = lane[i];
                const Entry& hi = lane[i + 1];
                if (lo.dir != hi.dir) continue;
                const Strip* inner = sec.strip(lo.id);
                const Strip* outer = sec.strip(hi.id);
                if (!inner || !outer) continue;
                const Marking* m = nullptr;
                if (lo.id < 0 && hi.id > 0) {
                    m = &sec.centerMark;
                } else if (lo.id < 0) {
                    m = &outer->outerMark;   // the higher-t of two right lanes
                } else {
                    m = &inner->outerMark;
                }
                bool crossable = m->crossableFromLeft() && m->crossableFromRight();
                LaneNode& a = lanes_.nodes[size_t(lo.node)];
                LaneNode& b = lanes_.nodes[size_t(hi.node)];
                if (a.dir > 0) {
                    a.leftNeighbor = hi.node;
                    a.leftCrossable = crossable;
                    b.rightNeighbor = lo.node;
                    b.rightCrossable = crossable;
                } else {
                    a.rightNeighbor = hi.node;
                    a.rightCrossable = crossable;
                    b.leftNeighbor = lo.node;
                    b.leftCrossable = crossable;
                }
            }
        }
    }

    // 3. Section-to-section continuity inside a road.
    for (const Road& r : roads_) {
        for (size_t si = 0; si + 1 < r.xs.sections.size(); ++si) {
            const LaneSection& a = r.xs.sections[si];
            for (const std::vector<Strip>* stack : {&a.left, &a.right}) {
                for (const Strip& st : *stack) {
                    if (!st.isLane() || st.dir == 0 || st.successor == kNoLane) continue;
                    int na = lanes_.find({r.id, int(si), st.id});
                    int nb = lanes_.find({r.id, int(si) + 1, st.successor});
                    if (na < 0 || nb < 0) continue;
                    if (st.dir > 0) {
                        lanes_.nodes[size_t(na)].successors.push_back(nb);
                        lanes_.nodes[size_t(nb)].predecessors.push_back(na);
                    } else {
                        lanes_.nodes[size_t(nb)].successors.push_back(na);
                        lanes_.nodes[size_t(na)].predecessors.push_back(nb);
                    }
                }
            }
        }
    }

    // 4. Road-to-road links.
    for (const Road& r : roads_) {
        if (r.succ.type == LinkType::Road && r.succ.id >= 0)
            linkRoadToRoad(r.id, true, r.succ.id, r.succ.toStart, r.succ.laneLinks);
        if (r.pred.type == LinkType::Road && r.pred.id >= 0)
            linkRoadToRoad(r.id, false, r.pred.id, r.pred.toStart, r.pred.laneLinks);
    }

    // 5. Explicit lane links (ramp merges and diverges).
    for (const ExtraLaneLink& el : extraLinks) {
        if (el.fromRoad < 0 || el.toRoad < 0) continue;
        const Road& A = roads_[size_t(el.fromRoad)];
        const Road& B = roads_[size_t(el.toRoad)];
        double sa = el.fromAtEnd ? A.end() : A.begin();
        double sb = el.toAtStart ? B.begin() : B.end();
        int seca = A.xs.sectionIndexAt(el.fromAtEnd ? std::max(A.begin(), sa - 1e-3) : sa + 1e-3);
        int secb = B.xs.sectionIndexAt(el.toAtStart ? sb + 1e-3 : std::max(B.begin(), sb - 1e-3));
        int na = lanes_.find({el.fromRoad, seca, el.fromLane});
        int nb = lanes_.find({el.toRoad, secb, el.toLane});
        if (na < 0 || nb < 0) continue;
        lanes_.nodes[size_t(na)].successors.push_back(nb);
        lanes_.nodes[size_t(nb)].predecessors.push_back(na);
    }

    // 6. Junction connectors: arm lane -> connector -> arm lane.
    for (const Junction& j : junctions_) {
        for (const Connection& c : j.connections) {
            if (c.connectorRoad < 0) continue;
            const Road& conn = roads_[size_t(c.connectorRoad)];
            int connSection = conn.xs.sectionIndexAt(conn.begin() + 1e-3);
            int connLane = -1;
            for (const Strip& st : conn.xs.sections[size_t(std::max(0, connSection))].right) {
                if (st.isLane()) {
                    connLane = st.id;
                    break;
                }
            }
            if (connLane == 0) continue;
            int nc = lanes_.find({conn.id, connSection, connLane});
            int nfrom = lanes_.find(c.from);
            int nto = lanes_.find(c.to);
            if (nc < 0) continue;
            if (nfrom >= 0) {
                lanes_.nodes[size_t(nfrom)].successors.push_back(nc);
                lanes_.nodes[size_t(nc)].predecessors.push_back(nfrom);
            }
            if (nto >= 0) {
                lanes_.nodes[size_t(nc)].successors.push_back(nto);
                lanes_.nodes[size_t(nto)].predecessors.push_back(nc);
            }
        }
    }
}

// --- validation -----------------------------------------------------------

std::vector<std::string> Network::validate() const {
    std::vector<std::string> out;
    char buf[320];

    for (const Road& r : roads_) {
        // Minimum radius from the design speed: R = v^2 / (127 (e + f)). It is a
        // rule for open road, not for turning paths inside a junction — those are
        // designed to a different standard and are deliberately tight.
        double v = r.designSpeed;
        double rmin = minRadiusForSpeed(v);
        if (r.kind == RoadKind::Connector || r.kind == RoadKind::RoundaboutRing) rmin = 0;
        for (const GeomPrim& g : r.spine.prims()) {
            double k = std::max(std::fabs(g.curv0), std::fabs(g.curv1));
            if (k < 1e-6) continue;
            double R = 1.0 / k;
            if (R < rmin * 0.98) {
                std::snprintf(buf, sizeof buf,
                              "road %d (%s): radius %.0f m at s=%.0f is below the %.0f m "
                              "minimum for %.0f km/h",
                              r.id, r.name.c_str(), R, g.s0, rmin, v);
                out.push_back(buf);
                break;
            }
        }
        // Grade.
        double worstGrade = 0, worstAt = 0;
        for (double s = r.begin(); s <= r.end(); s += 5.0) {
            double gr = std::fabs(r.spine.elevationConst().slope(s));
            if (gr > worstGrade) {
                worstGrade = gr;
                worstAt = s;
            }
        }
        double gradeLimit = r.designSpeed >= 90 ? 0.06 : 0.12;
        if (worstGrade > gradeLimit + 1e-6) {
            std::snprintf(buf, sizeof buf,
                          "road %d (%s): grade %.1f%% at s=%.0f exceeds the %.0f%% limit",
                          r.id, r.name.c_str(), worstGrade * 100.0, worstAt, gradeLimit * 100.0);
            out.push_back(buf);
        }
        // Taper rate: a lane whose width changes faster than 1:15 reads as a
        // kink and drives like one.
        for (const LaneSection& sec : r.xs.sections) {
            // A sliver section carries no useful taper information; the geometry
            // lint below would report a meaningless 1:0 rate for it.
            if (sec.length < 2.0) continue;
            for (const std::vector<Strip>* stack : {&sec.left, &sec.right}) {
                for (const Strip& st : *stack) {
                    double w0 = st.width.eval(0), w1 = st.width.eval(sec.length);
                    double rate = std::fabs(w1 - w0) / sec.length;
                    // A turn bay is allowed a sharper entry taper than a lane
                    // shift: drivers are decelerating into it, not tracking
                    // through it at speed.
                    double limit = st.kind == StripKind::Turn ? 1.0 / 8.0 : 1.0 / 12.0;
                    if (rate > limit * 1.02) {
                        std::snprintf(buf, sizeof buf,
                                      "road %d (%s): %s taper 1:%.0f at s=%.0f is sharper than "
                                      "the limit (needs %.0f m, has %.0f m)",
                                      r.id, r.name.c_str(), stripKindName(st.kind), 1.0 / rate,
                                      sec.s0, taperLength(std::fabs(w1 - w0), r.designSpeed),
                                      sec.length);
                        out.push_back(buf);
                    }
                }
            }
        }
    }

    // Roads joined by road links or ramp lane links form ONE carriageway system:
    // split pieces of a motorway, plus the ramps that merge into them. Clearance
    // between members of the same system is meaningless — they are supposed to
    // converge. (A road that genuinely passes over itself would be missed; that
    // is a known limitation of using connectivity as the proxy.)
    std::vector<int> carriageway(roads_.size());
    for (size_t i = 0; i < carriageway.size(); ++i) carriageway[i] = int(i);
    std::function<int(int)> findRoot = [&](int x) {
        while (carriageway[size_t(x)] != x) {
            carriageway[size_t(x)] = carriageway[size_t(carriageway[size_t(x)])];
            x = carriageway[size_t(x)];
        }
        return x;
    };
    auto unite = [&](int a, int b) {
        if (a < 0 || b < 0) return;
        int ra = findRoot(a), rb = findRoot(b);
        if (ra != rb) carriageway[size_t(ra)] = rb;
    };
    for (const Road& r : roads_) {
        if (r.succ.type == LinkType::Road) unite(r.id, r.succ.id);
        if (r.pred.type == LinkType::Road) unite(r.id, r.pred.id);
    }
    for (const ExtraLaneLink& el : extraLinks) unite(el.fromRoad, el.toRoad);

    // Vertical clearance between stacked roads. This is the check that makes
    // multi-tier authoring safe: two roads crossing in plan are fine, two roads
    // crossing in plan with 3 m of air between them are not.
    for (size_t i = 0; i < roads_.size(); ++i) {
        for (size_t k = i + 1; k < roads_.size(); ++k) {
            const Road& A = roads_[i];
            const Road& B = roads_[k];
            // Junction internals overlap each other and their arms by design.
            if (A.kind == RoadKind::Connector || B.kind == RoadKind::Connector) continue;
            // Two windows over the same split geometry, or a ramp lane feeding a
            // mainline lane, are the SAME carriageway. They are supposed to touch.
            auto directlyLinked = [&](const Road& x, const Road& y) {
                if ((x.succ.type == LinkType::Road && x.succ.id == y.id) ||
                    (x.pred.type == LinkType::Road && x.pred.id == y.id))
                    return true;
                for (const ExtraLaneLink& el : extraLinks) {
                    if ((el.fromRoad == x.id && el.toRoad == y.id) ||
                        (el.fromRoad == y.id && el.toRoad == x.id))
                        return true;
                }
                return false;
            };
            if (directlyLinked(A, B)) continue;
            if (findRoot(A.id) == findRoot(B.id)) continue;
            // A junction's connectors overlap its own arms by construction —
            // that is what a junction IS. Only unrelated roads can violate
            // clearance.
            auto sharesJunction = [&](const Road& x, const Road& y) {
                for (int jid : {x.junctionId, y.junctionId}) {
                    if (jid < 0) continue;
                    if (x.junctionId == y.junctionId) return true;
                    for (const JunctionArm& arm : junctions_[size_t(jid)].arms)
                        if (arm.road == x.id || arm.road == y.id) return true;
                }
                return false;
            };
            if (sharesJunction(A, B)) continue;
            Vec2 loA, hiA, loB, hiB;
            A.spine.planBounds(loA, hiA);
            B.spine.planBounds(loB, hiB);
            const double pad = 30.0;
            if (hiA.x + pad < loB.x || hiB.x + pad < loA.x || hiA.y + pad < loB.y ||
                hiB.y + pad < loA.y)
                continue;
            double worst = 1e9, worstS = 0;
            bool overlap = false;
            for (double s = A.begin(); s <= A.end(); s += 2.0) {
                Vec2 p = A.spine.toPlan(s, 0);
                double sb = 0, tb = 0;
                if (!B.spine.toST(p, sb, tb)) continue;
                if (sb < B.begin() || sb > B.end()) continue;
                double halfB = std::max(std::fabs(B.xs.leftExtentAt(sb)),
                                        std::fabs(B.xs.rightExtentAt(sb)));
                double halfA = std::max(std::fabs(A.xs.leftExtentAt(s)),
                                        std::fabs(A.xs.rightExtentAt(s)));
                if (std::fabs(tb) > halfA + halfB) continue;
                // Roads running roughly parallel are not crossing — they are a
                // merge, a frontage road or a split carriageway. Clearance is a
                // question about things passing OVER each other.
                double ha = A.spine.frameAt(s).heading;
                double hb = B.spine.frameAt(sb).heading;
                double sep = std::fabs(std::sin(angleDiff(ha, hb)));
                if (sep < 0.26) continue;
                overlap = true;
                double ya = A.surfacePoint(s, 0).y;
                double yb = B.surfacePoint(sb, tb).y;
                double gap = std::fabs(ya - yb);
                if (gap < worst) {
                    worst = gap;
                    worstS = s;
                }
            }
            if (!overlap) continue;
            double required = 5.1;
            for (const StructureSpan& sp : A.structures)
                if (sp.contains(worstS)) required = std::max(required, sp.clearanceRequired);
            if (worst < required - 1e-3) {
                if (worst < 0.35) {
                    // Coincident surfaces are an at-grade crossing, not a stack;
                    // it needs a junction, and the absence of one is the bug.
                    bool haveJunction = false;
                    for (const Junction& j : junctions_) {
                        for (const JunctionArm& arm : j.arms) {
                            if (arm.road == A.id || arm.road == B.id) haveJunction = true;
                        }
                    }
                    if (!haveJunction) {
                        std::snprintf(buf, sizeof buf,
                                      "roads %d (%s) and %d (%s) cross at grade near s=%.0f with "
                                      "no junction declared",
                                      A.id, A.name.c_str(), B.id, B.name.c_str(), worstS);
                        out.push_back(buf);
                    }
                } else {
                    std::snprintf(buf, sizeof buf,
                                  "roads %d (%s) and %d (%s): vertical clearance %.2f m at s=%.0f "
                                  "is below the %.2f m requirement",
                                  A.id, A.name.c_str(), B.id, B.name.c_str(), worst, worstS,
                                  required);
                    out.push_back(buf);
                }
            }
        }
    }

    for (const Junction& j : junctions_) {
        if (j.arms.size() < 2) {
            std::snprintf(buf, sizeof buf, "junction %d (%s) has %zu arm(s)", j.id,
                          j.name.c_str(), j.arms.size());
            out.push_back(buf);
        }
        if (j.control == JunctionControl::Signalized && j.phases.empty()) {
            std::snprintf(buf, sizeof buf, "junction %d (%s) is signalised but has no phases",
                          j.id, j.name.c_str());
            out.push_back(buf);
        }
    }
    return out;
}

bool Network::sample(Vec2 planPoint, RoadHit& hit, double maxDistance) const {
    bool found = false;
    double best = 1e300;
    for (const Road& r : roads_) {
        Vec2 lo, hi;
        r.spine.planBounds(lo, hi);
        double pad = 40.0;
        if (planPoint.x < lo.x - pad || planPoint.x > hi.x + pad || planPoint.y < lo.y - pad ||
            planPoint.y > hi.y + pad)
            continue;
        double s = 0, t = 0;
        // toST clamps s to the spine, so a point off the END of a road would
        // otherwise report as being on it — which punched holes in the terrain
        // just past every road's tip.
        if (!r.spine.toST(planPoint, s, t)) continue;
        if (s < r.begin() - 1e-6 || s > r.end() + 1e-6) continue;
        double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
        double outside = 0;
        if (t > le) outside = t - le;
        if (t < re) outside = re - t;
        if (outside > maxDistance) continue;
        // Prefer the road we are actually ON; among those, the one whose surface
        // is highest, so an overpass wins over the road beneath it.
        double score = outside > 1e-6 ? 1000.0 + outside : -r.surfacePoint(s, t).y;
        if (score < best) {
            best = score;
            hit.road = r.id;
            hit.s = s;
            hit.t = t;
            hit.distance = outside;
            found = true;
        }
    }
    return found;
}

void Network::planBounds(Vec2& lo, Vec2& hi) const {
    lo = {1e300, 1e300};
    hi = {-1e300, -1e300};
    for (const Road& r : roads_) {
        Vec2 a, b;
        r.spine.planBounds(a, b);
        double pad = std::max(std::fabs(r.xs.leftExtentAt(r.begin())),
                              std::fabs(r.xs.rightExtentAt(r.begin()))) +
                     2.0;
        lo.x = std::min(lo.x, a.x - pad);
        lo.y = std::min(lo.y, a.y - pad);
        hi.x = std::max(hi.x, b.x + pad);
        hi.y = std::max(hi.y, b.y + pad);
    }
    if (roads_.empty()) {
        lo = {-10, -10};
        hi = {10, 10};
    }
}

}  // namespace roadlab
