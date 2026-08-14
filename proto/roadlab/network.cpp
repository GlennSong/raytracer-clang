#include "network.h"

#include "diag.h"

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

int endSectionIndex(const Road& r, bool atEnd) {
    double s = atEnd ? r.end() : r.begin();
    int si = r.xs.sectionIndexAt(atEnd ? std::max(r.begin(), s - 1e-3) : s + 1e-3);
    while (si >= 0 && si < int(r.xs.sections.size())) {
        const LaneSection& sec = r.xs.sections[size_t(si)];
        double s0 = std::max(sec.s0, r.begin());
        double s1 = std::min(sec.s0 + sec.length, r.end());
        if (s1 - s0 >= kMinLaneSectionRun) return si;
        si += atEnd ? -1 : +1;
    }
    return -1;
}

EndLanes endLanes(const Road& r, bool atEnd) {
    EndLanes out;
    double s = atEnd ? r.end() : r.begin();
    int si = endSectionIndex(r, atEnd);
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

std::vector<std::pair<int, int>> pairLanesAcross(const Road& a, bool aAtEnd, const Road& b,
                                                 bool bAtStart, double tolerance) {
    std::vector<std::pair<int, int>> pairs;
    double sa = aAtEnd ? a.end() : a.begin();
    double sb = bAtStart ? b.begin() : b.end();
    EndLanes ea = endLanes(a, aAtEnd);
    EndLanes eb = endLanes(b, !bAtStart);
    // Positions must come from the SAME section the ids came from. Road::
    // laneCenterT resolves the section by station, which lands in the sliver
    // whenever a road window ends just past a section boundary — so the ids
    // would be one section's and the geometry another's, and the match is then
    // between lanes that were never in the same stack.
    int seca = endSectionIndex(a, aAtEnd);
    int secb = endSectionIndex(b, !bAtStart);
    RL_CALLED("pairLanesAcross");
    if (seca < 0 || secb < 0) {
        RL_FALLBACK("pairLanesAcross miss (an end carries no lane section)");
        return pairs;
    }
    if (ea.incoming.empty() || eb.outgoing.empty()) {
        // Not a failure: linkRoadToRoad pairs BOTH directions of every link, and
        // a one-way road — every connector, every ramp — has no lanes to offer in
        // one of them. Counted so the number is visible rather than assumed.
        RL_FALLBACK("pairLanesAcross miss (one side is one-way against this direction)");
        return pairs;
    }

    // Compare WORLD positions rather than t values: the two roads have different
    // reference lines and possibly opposite senses, and matching in the plane
    // sidesteps every sign question that raises.
    std::vector<char> used(eb.outgoing.size(), 0);
    for (int la : ea.incoming) {
        Vec2 pa = a.planPoint(sa, a.xs.laneCenterT(seca, la, sa));
        double best = tolerance;
        int bestIdx = -1;
        for (size_t k = 0; k < eb.outgoing.size(); ++k) {
            if (used[k]) continue;
            int lb = eb.outgoing[k];
            Vec2 pb = b.planPoint(sb, b.xs.laneCenterT(secb, lb, sb));
            double d = length(pa - pb);
            if (d < best) {
                best = d;
                bestIdx = int(k);
            }
        }
        if (bestIdx < 0) {
            // Either a genuine lane drop, or two carriageways that do not line up
            // as well as the author thought. The count separates those.
            RL_FALLBACK("pairLanesAcross lane found no partner within tolerance");
            continue;
        }
        used[size_t(bestIdx)] = 1;
        pairs.push_back({la, eb.outgoing[size_t(bestIdx)]});
    }
    return pairs;
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

bool LaneGraph::reaches(int fromNode, int targetRoad, int maxDepth) const {
    if (fromNode < 0 || size_t(fromNode) >= nodes.size()) return false;
    // Small, bounded, and called per vehicle per step, so it uses a scratch mark
    // buffer rather than allocating a visited set every time.
    static thread_local std::vector<int> stamp;
    static thread_local int epoch = 0;
    if (stamp.size() != nodes.size()) {
        stamp.assign(nodes.size(), 0);
        epoch = 0;
    }
    ++epoch;
    static thread_local std::vector<std::pair<int, int>> queue;
    queue.clear();
    queue.push_back({fromNode, 0});
    stamp[size_t(fromNode)] = epoch;
    for (size_t head = 0; head < queue.size(); ++head) {
        auto [n, d] = queue[head];
        if (nodes[size_t(n)].ref.road == targetRoad) return true;
        if (d >= maxDepth) continue;
        for (int nx : nodes[size_t(n)].successors) {
            if (stamp[size_t(nx)] == epoch) continue;
            stamp[size_t(nx)] = epoch;
            queue.push_back({nx, d + 1});
        }
    }
    return false;
}

std::vector<int> LaneGraph::lanesReaching(int fromNode, int targetRoad, int maxDepth) const {
    std::vector<int> result;
    if (fromNode < 0 || size_t(fromNode) >= nodes.size()) return result;
    const LaneNode& start = nodes[size_t(fromNode)];

    // Which lanes of the same road+section can get to targetRoad? That is the
    // question a driver is actually asking 400 m before their exit.
    for (size_t i = 0; i < nodes.size(); ++i) {
        const LaneNode& n = nodes[i];
        if (n.ref.road != start.ref.road || n.ref.section != start.ref.section) continue;
        if (n.dir != start.dir) continue;
        if (reaches(int(i), targetRoad, maxDepth)) result.push_back(int(i));
    }
    return result;
}

// --- spatial index --------------------------------------------------------

void RoadIndex::build(const std::vector<Road>& roads, double pad, double cellSize) {
    cells_.clear();
    empty_.clear();
    cell_ = std::max(8.0, cellSize);
    if (roads.empty()) {
        nx_ = nz_ = 0;
        return;
    }
    Vec2 hi{-1e300, -1e300};
    lo_ = {1e300, 1e300};
    for (const Road& r : roads) {
        Vec2 a, b;
        r.spine.planBounds(a, b);
        lo_.x = std::min(lo_.x, a.x - pad);
        lo_.y = std::min(lo_.y, a.y - pad);
        hi.x = std::max(hi.x, b.x + pad);
        hi.y = std::max(hi.y, b.y + pad);
    }
    nx_ = std::max(1, std::min(2048, int((hi.x - lo_.x) / cell_) + 1));
    nz_ = std::max(1, std::min(2048, int((hi.y - lo_.y) / cell_) + 1));
    cells_.assign(size_t(nx_) * size_t(nz_), {});
    for (const Road& r : roads) {
        Vec2 a, b;
        r.spine.planBounds(a, b);
        int i0 = std::max(0, int((a.x - pad - lo_.x) / cell_));
        int i1 = std::min(nx_ - 1, int((b.x + pad - lo_.x) / cell_));
        int k0 = std::max(0, int((a.y - pad - lo_.y) / cell_));
        int k1 = std::min(nz_ - 1, int((b.y + pad - lo_.y) / cell_));
        for (int k = k0; k <= k1; ++k)
            for (int i = i0; i <= i1; ++i) cells_[size_t(k) * size_t(nx_) + size_t(i)].push_back(r.id);
    }
}

const std::vector<int>& RoadIndex::near(Vec2 p) const {
    if (cells_.empty()) return empty_;
    int i = int((p.x - lo_.x) / cell_);
    int k = int((p.y - lo_.y) / cell_);
    if (i < 0 || k < 0 || i >= nx_ || k >= nz_) return empty_;
    return cells_[size_t(k) * size_t(nx_) + size_t(i)];
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
    // An imported junction arrives with its arms already trimmed and its
    // connectors already built; re-resolving it would trim twice and duplicate
    // every connector. Adopt takes it as given and only derives what the file
    // does not carry: the pad, the conflict table, priority and phases.
    for (Junction& j : junctions_) {
        if (j.imported) {
            adoptJunction(*this, j);
        } else {
            buildJunction(*this, j);
        }
    }
    buildLaneGraph();
    index_.build(roads_);
}

void Network::linkRoadToRoad(int roadA, bool aAtEnd, int roadB, bool bAtStart,
                             const std::vector<std::pair<int, int>>& overrides) {
    const Road& A = roads_[size_t(roadA)];
    const Road& B = roads_[size_t(roadB)];
    // The section that carries nodes, not merely the one the end station lands
    // in — see endSectionIndex. endLanes() resolves the same way, so the lane
    // ids the pairing produces and the nodes looked up here are from one section.
    int seca = endSectionIndex(A, aAtEnd);
    int secb = endSectionIndex(B, !bAtStart);
    if (seca < 0 || secb < 0) return;

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
    // Default pairing is by position, not ordinal: a lane that has no partner is
    // a lane that ends, which the sim reads as a mandatory lane change rather
    // than a dead end it discovers too late — but a lane that DOES continue must
    // continue as itself, not as its neighbour.
    for (const auto& pr : pairLanesAcross(A, aAtEnd, B, bAtStart)) connect(pr.first, pr.second);
    for (const auto& pr : pairLanesAcross(B, !bAtStart, A, !aAtEnd)) {
        int nb = lanes_.find({roadB, secb, pr.first});
        int na = lanes_.find({roadA, seca, pr.second});
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
            if (s1 - s0 < kMinLaneSectionRun) continue;
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
        // Resolve to the section that carries nodes, exactly as the road-to-road
        // linking does. Resolving by station instead finds the sliver a ramp
        // split leaves behind, which has no nodes, and the link is then dropped
        // by the `continue` below — an on-ramp that renders perfectly and that
        // no vehicle can ever use to join the mainline.
        int seca = endSectionIndex(A, el.fromAtEnd);
        int secb = endSectionIndex(B, !el.toAtStart);
        int na = seca >= 0 ? lanes_.find({el.fromRoad, seca, el.fromLane}) : -1;
        int nb = secb >= 0 ? lanes_.find({el.toRoad, secb, el.toLane}) : -1;
        if (na < 0 || nb < 0) {
            RL_FALLBACK("extra lane link dropped (no node at one end)");
            continue;
        }
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
            // Only what this road actually IS. A cross-section is built over the
            // whole spine, but a road is the window it occupies — splitting is
            // how ramps work, and every piece carries the full timeline. A
            // section outside the window belongs to a sibling piece, which is
            // judged on its own, and reporting it here says the same thing about
            // the same metre of asphalt once per piece that shares the spine.
            if (sec.s0 + sec.length < r.begin() - 1e-6 || sec.s0 > r.end() + 1e-6) continue;
            // A sliver section carries no useful taper information; the geometry
            // lint below would report a meaningless 1:0 rate for it.
            if (sec.length < 2.0) continue;
            // A gore nose is exempt: see ProfileEdit::mergeNose. The lane is
            // handed over by a ramp rather than tapered out of nothing, so the
            // surface a driver sees does not narrow even though this one road's
            // share of it does.
            bool atNose = false;
            for (const Road::ProfileEdit& e : r.profileEdits)
                if (e.mergeNose && std::fabs(e.s - sec.s0) < 2.0) atNose = true;
            // ...and the same exemption has to survive a round trip through
            // OpenDRIVE, which stores geometry rather than authoring: the edits
            // are gone on the far side, and the export has split the merge into
            // stub roads whose links read as plain road-to-road.
            //
            // What survives is the SHAPE, and a gore has a distinctive one: a
            // lane that is born at zero width and is at full width a few metres
            // later, right where the road ends and hands it on. Nothing else in
            // the system looks like that — a real lane drop also reaches zero,
            // but over the hundred-plus metres the lint is asking for, which is
            // what the length bound separates.
            bool goreShaped = sec.length < 25.0 &&
                              (std::fabs(sec.s0 - r.begin()) < 1e-3 ||
                               std::fabs(sec.s0 + sec.length - r.end()) < 1e-3);
            if (atNose || goreShaped) continue;
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

        // The arms of a junction have to actually meet. Declaring one whose arms
        // stand a hundred metres apart is an authoring slip — a mistyped split
        // station, a road moved without moving the junction — and it is nearly
        // invisible downstream: trimming still runs, a pad is still built, and
        // the result is a plausible-looking asphalt sheet spanning the gap with a
        // self-intersecting outline. Two demos shipped with exactly this bug.
        //
        // The budget is generous on purpose. A wide skewed junction legitimately
        // puts its contacts well apart, so this only catches arms that are not
        // plausibly the same place.
        double widest = 0;
        for (const JunctionArm& a : j.arms)
            widest = std::max(widest, std::max(std::fabs(a.leftExtent), std::fabs(a.rightExtent)));
        double budget = 4.0 * widest + 2.0 * j.cornerRadius + 20.0;
        for (const JunctionArm& a : j.arms) {
            double d = length(a.contact - j.center);
            if (d <= budget) continue;
            std::snprintf(buf, sizeof buf,
                          "junction %d (%s): arm on road %d (%s) contacts %.0f m from the "
                          "junction centre, past the %.0f m the geometry allows — the arms do "
                          "not meet",
                          j.id, j.name.c_str(), a.road, roads_[size_t(a.road)].name.c_str(), d,
                          budget);
            out.push_back(buf);
            break;   // one report per junction is enough to find it
        }
    }
    return out;
}

bool Network::sample(Vec2 planPoint, RoadHit& hit, double maxDistance) const {
    RL_CALLED("Network::sample");
    bool found = false;
    double best = 1e300;
    // The index narrows this to the handful of roads that could possibly matter;
    // without it every query walks the whole network.
    static thread_local std::vector<int> all;
    const std::vector<int>* candidates = &roadsNear(planPoint);
    if (index_.empty()) {
        all.clear();
        for (const Road& r : roads_) all.push_back(r.id);
        candidates = &all;
    }
    for (int rid : *candidates) {
        const Road& r = roads_[size_t(rid)];
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
    if (!found) RL_FALLBACK("Network::sample miss (no road under the point)");
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
