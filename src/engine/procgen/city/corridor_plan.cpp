#include "corridor_plan.h"

#include "../../../log.h"
#include "corridor_bake.h"
#include "corridor_mesh.h"   // corridorAuthor

#include <algorithm>
#include <array>
#include <cmath>

namespace engine {

namespace {

// Distance from p to segment ab (shared by the planning rule 3 registries).
Real segDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real l2 = ab.lengthSquared();
    Real u = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
    u = u < 0 ? 0 : (u > 1 ? 1 : u);
    return (a + ab * u - p).length();
}

// Do segments p1p2 and p3p4 properly intersect? (landing selection + the
// criss-cross guard share this)
bool segsCross(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4) {
    auto ori = [](const Vec2& a, const Vec2& b, const Vec2& c) {
        const Real v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return v > 1e-6 ? 1 : (v < -1e-6 ? -1 : 0);
    };
    return ori(p1, p2, p3) != ori(p1, p2, p4) &&
           ori(p3, p4, p1) != ori(p3, p4, p2);
}

}  // namespace

// §10.6/§12: metro-planned corridors with NETWORK RULES (device: "It needs to
// have rules about elevation and not having the freeways intersect. on and off
// ramps should not criss-cross"):
//   rule 1 (generator): one route per hub — routes never touch.
//   rule 2 (here): residual geometric crossings force the SHORTER route OVER
//          on a bridge bump solved into its profile.
//   rule 3 (here): one GLOBAL landing/ramp registry — a ramp that would crowd
//          a landing, cross another ramp, or graze another corridor at grade
//          is never stamped.
std::vector<PlannedCorridor> planCorridorRoutes(
    const std::vector<CorridorRouteInput>& routes,
    const std::vector<CorridorStreetAnchor>& anchors,
    const std::function<Real(Real, Real)>& ground) {
    std::vector<PlannedCorridor> out;
    if (!ground) return out;
    struct PlannedRoute {
        std::vector<Vec2> anchors;   // generator anchor polyline
        std::vector<Vec2> dense;     // 20 m samples (crossing tests)
        Real len = 0;
        Real spacing = 700;
        std::vector<Real> bridgeAt;  // stations that must fly over
        bool dropped = false;        // parallel-overlap loser
        int srcNet = -1;             // which net grew this route (S3b bake)
    };
    std::vector<PlannedRoute> planned;
    for (const CorridorRouteInput& in : routes) {
        if (in.anchors.size() < 2) continue;
        PlannedRoute pr;
        pr.anchors = in.anchors;
        pr.spacing = in.spacing;
        pr.srcNet = in.srcNet;
        for (std::size_t k = 0; k + 1 < in.anchors.size(); ++k) {
            const Vec2 A = in.anchors[k], B = in.anchors[k + 1];
            const Real seg = (B - A).length();
            for (Real s2 = 0; s2 < seg; s2 += 20.0)
                pr.dense.push_back(A + (B - A) * (s2 / seg));
            pr.len += seg;
        }
        pr.dense.push_back(in.anchors.back());
        planned.push_back(std::move(pr));
    }
    // §12 R1.2b: a route may not approach ITSELF — enough compound curvature
    // makes a teardrop. Truncate at the first self-approach (non-adjacent
    // samples within 24 m), keeping the prefix.
    for (PlannedRoute& pr : planned) {
        std::size_t cut = pr.dense.size();
        for (std::size_t i = 0; i < pr.dense.size() && cut == pr.dense.size(); ++i)
            for (std::size_t j = i + 8; j < pr.dense.size(); ++j)
                if ((pr.dense[j] - pr.dense[i]).length() < 24.0) {
                    cut = j;
                    break;
                }
        if (cut < pr.dense.size()) {
            LOG_WARN << "[corridor] route curls back on itself near ("
                     << pr.dense[cut].x << ", " << pr.dense[cut].y
                     << ") — truncated (R1.2)";
            pr.dense.resize(cut);
            // rebuild anchors from the surviving samples (every ~160 m)
            pr.anchors.clear();
            for (std::size_t k = 0; k < pr.dense.size(); k += 8)
                pr.anchors.push_back(pr.dense[k]);
            if (pr.anchors.size() < 2 ||
                (pr.anchors.back() - pr.dense.back()).length() > 1.0)
                pr.anchors.push_back(pr.dense.back());
            pr.len = (pr.dense.size() - 1) * 20.0;
            if (pr.anchors.size() < 2) pr.dropped = true;
        }
    }
    // rule 2: a short contact is a CROSSING (shorter route bridges over); a
    // long contact is a PARALLEL OVERLAP (no vertical fix can save it — the
    // shorter route is dropped, loudly).
    for (std::size_t a2 = 0; a2 < planned.size(); ++a2)
        for (std::size_t b2 = a2 + 1; b2 < planned.size(); ++b2) {
            PlannedRoute& flyer = planned[a2].len <= planned[b2].len
                                      ? planned[a2] : planned[b2];
            const PlannedRoute& other = planned[a2].len <= planned[b2].len
                                            ? planned[b2] : planned[a2];
            if (flyer.dropped || other.dropped) continue;
            std::vector<char> hit(flyer.dense.size(), 0);
            for (std::size_t i = 0; i < flyer.dense.size(); ++i)
                for (const Vec2& q : other.dense)
                    if ((q - flyer.dense[i]).length() < 24.0) {
                        hit[i] = 1;
                        break;
                    }
            Real lastBump = -1e9;
            for (std::size_t i = 0; i < hit.size(); ++i) {
                if (!hit[i]) continue;
                std::size_t j = i;
                while (j + 1 < hit.size() && hit[j + 1]) ++j;
                const Real runLen = (j - i + 1) * 20.0;
                const Real st = (i + j) * 0.5 * 20.0;
                if (runLen > 140.0) {
                    flyer.dropped = true;
                    LOG_WARN << "[corridor] routes run PARALLEL for " << runLen
                             << " m near (" << flyer.dense[i].x << ", "
                             << flyer.dense[i].y << ") — shorter route dropped";
                    break;
                }
                if (st - lastBump >= 260.0) {
                    flyer.bridgeAt.push_back(st);
                    lastBump = st;
                    LOG_INFO << "[corridor] route crossing near ("
                             << flyer.dense[i].x << ", " << flyer.dense[i].y
                             << ") — shorter route bridges over";
                }
                i = j;
            }
        }
    // §12 R1.4 (device: "1-2 freeways crossing the city"): a metro earns FEW,
    // GOOD freeways — twisted routes die to a straightness test (endpoint
    // distance / arc length), and only the longest survivor builds. More
    // freeway is not better freeway.
    {
        for (PlannedRoute& pr : planned) {
            if (pr.dropped || pr.dense.size() < 2) continue;
            const Real chord = (pr.dense.back() - pr.dense.front()).length();
            if (chord / std::max(Real(1), pr.len) < 0.55) {
                pr.dropped = true;
                LOG_WARN << "[corridor] route len=" << pr.len
                         << " is twisted (straightness " << chord / pr.len
                         << ") — dropped (R1.4)";
            }
        }
        std::vector<std::size_t> alive;
        for (std::size_t i = 0; i < planned.size(); ++i)
            if (!planned[i].dropped) alive.push_back(i);
        std::sort(alive.begin(), alive.end(),
                  [&](std::size_t a2, std::size_t b2) {
                      return planned[a2].len > planned[b2].len;
                  });
        for (std::size_t k = 1; k < alive.size(); ++k) {
            planned[alive[k]].dropped = true;
            LOG_INFO << "[corridor] route len=" << planned[alive[k]].len
                     << " dropped: ONE spine freeway (R1.4, device)";
        }
    }
    // rule 3 registries, shared by every stamped diamond
    std::vector<Vec2> globalLandings;
    std::vector<std::pair<Vec2, Vec2>> rampSegs;
    for (std::size_t pi2 = 0; pi2 < planned.size(); ++pi2) {
        const PlannedRoute& pr = planned[pi2];
        if (pr.dropped) continue;
        CorridorDef def;
        // R 300 desired, floor 200 — the plan was already bend-limited
        // (chamfered) in the generator, so this rarely bites; it just
        // guarantees no residual corner becomes a hairpin.
        def.horizontal = Alignment::fromPolyline(pr.anchors, 300.0, 90.0,
                                                 2.0, 200.0);
        const Real len = def.horizontal.length();
        if (len < 320.0) continue;
        def.lanes.throughLanes = 3;
        def.laneWidth = 3.6;
        def.medianWidth = 1.4;
        // eased profile + rule-2 bridge bumps (5% tent, 60 m curves)
        std::vector<Real> ss, zs;
        for (Real s = 0;; s += 120.0) {
            const Real sv = std::min(s, len);
            const Vec2 p = def.horizontal.pos(sv);
            ss.push_back(sv);
            zs.push_back(ground(p.x, p.y));
            if (sv >= len) break;
        }
        // §12 R2d (device: "freeways don't conform to the terrain — they
        // should be elevated over it ... smooth, cars go fast"): the profile
        // is ENGINEERED, not draped. Heavy smoothing fits a stiff line; then a
        // clearance-and-grade loop RAISES it wherever the ground pokes near it
        // and re-caps grades at 3.5% — bumps become underpasses beneath
        // structure, dips become viaducts.
        const std::vector<Real> tz = zs;   // raw terrain line
        for (int pass = 0; pass < 8; ++pass) {
            std::vector<Real> sm = zs;
            for (std::size_t i = 1; i + 1 < zs.size(); ++i)
                sm[i] = (zs[i - 1] + zs[i] * 2.0 + zs[i + 1]) * 0.25;
            zs.swap(sm);
        }
        // §12 R2e (device: "one elevated highway that heads toward the ground
        // at either end" + "the elevation is way too short ... no clearance"):
        // the spine RIDES STRUCTURE through the city and descends to grade
        // over its last ~300 m each side. The floor is sized for REAL
        // headroom: deck surface = ground + girder depth (2.75 m) + the 5 m
        // roadway clearance + a 1.25 m margin = +9 m, so a city street passes
        // cleanly UNDER the deck's underside.
        const Real kViaductFloor = 2.75 + 5.0 + 1.25;   // = 9.0 m
        for (std::size_t i = 0; i < ss.size(); ++i)
            if (ss[i] > 300.0 && ss[i] < len - 300.0)
                zs[i] = std::max(zs[i], tz[i] + kViaductFloor);
        for (int it2 = 0; it2 < 3; ++it2) {
            for (std::size_t i = 0; i < zs.size(); ++i)
                zs[i] = std::max(zs[i], tz[i] + 0.6);   // clear the ground
            for (std::size_t i = 1; i < zs.size(); ++i)   // grade cap fwd
                zs[i] = std::max(zs[i], zs[i - 1] - 0.035 * 120.0);
            for (std::size_t i = zs.size() - 1; i-- > 0;)  // and back
                zs[i] = std::max(zs[i], zs[i + 1] - 0.035 * 120.0);
        }
        for (const Real sb : pr.bridgeAt)
            for (std::size_t i = 0; i < ss.size(); ++i) {
                // clearance 9 m at the crossing, falling at 5% away
                std::size_t bi = std::min(ss.size() - 1,
                    static_cast<std::size_t>(sb / 120.0));
                const Real tent = zs[bi] + 9.0 - 0.05 * std::fabs(ss[i] - sb);
                zs[i] = std::max(zs[i], std::min(tent, zs[bi] + 9.0));
            }
        for (std::size_t i = 0; i < ss.size(); ++i)
            def.vertical.pvis.push_back({ss[i], zs[i] + 0.6, 60.0});
        // §12 R3f FEASIBILITY-DRIVEN placement (device: "the road graph could
        // be built around it using the exit and entrance points"): scan for
        // stations where a diamond is PROVABLY possible — outside bridge
        // zones, with a street anchor available on each carriageway's side
        // within ramp reach — and place diamonds at the BEST such stations.
        // Even-spacing stamp-then-veto left whole routes rampless.
        struct Site { Real s; int feasible; };
        std::vector<Site> sites;
        for (Real sC = 90.0; sC <= len - 90.0; sC += 60.0) {
            bool nearBridge = false;
            for (const Real sb : pr.bridgeAt)
                if (std::fabs(sb - sC) < 320.0) nearBridge = true;
            if (nearBridge) continue;
            // per-RAMP feasibility with the SAME filters resolution applies
            // (side, flow direction, 70-340 m from the gore) — checking only
            // the site centre still left ramps unresolvable
            auto rampFeasible = [&](Real st, bool up, bool on) {
                if (st < 60.0 || st > len - 60.0) return false;
                const Vec2 gc = def.horizontal.pos(st);
                const Vec2 gn = def.horizontal.normal(st);
                const Real sideSign = up ? -1.0 : 1.0;
                const Vec2 travelDir =
                    def.horizontal.tangent(st) * (up ? 1.0 : -1.0);
                // ANY street node nearby (not just a junction) makes the site
                // feasible — the resolution below connects via the nearest
                // junction OR by splitting the nearest edge into a new
                // T-junction (device: "connect to the closest road even if
                // they have to make a new road graph node").
                for (const CorridorStreetAnchor& an : anchors) {
                    if (dot(an.pos - gc, gn) * sideSign < 12.0) continue;
                    const Real along = dot(an.pos - gc, travelDir);
                    if (on ? along > -20.0 : along < 20.0) continue;
                    const Real d = (an.pos - gc).length();
                    if (d >= 70.0 && d <= 340.0) return true;
                }
                return false;
            };
            int feas = 0;
            if (rampFeasible(sC - 80.0, true, false)) ++feas;
            if (rampFeasible(sC + 110.0, true, true)) ++feas;
            if (rampFeasible(sC + 80.0, false, false)) ++feas;
            if (rampFeasible(sC - 110.0, false, true)) ++feas;
            if (feas >= 2) sites.push_back({sC, feas});
        }
        std::sort(sites.begin(), sites.end(),
                  [](const Site& a, const Site& b) {
                      return a.feasible > b.feasible;
                  });
        std::vector<Real> chosen;
        const int maxD = std::max(1, static_cast<int>(
            std::floor(len / std::max(Real(300), pr.spacing * 0.6))));
        for (const Site& st : sites) {
            if (static_cast<int>(chosen.size()) >= maxD) break;
            bool clear2 = true;
            for (const Real cs : chosen)
                if (std::fabs(cs - st.s) < pr.spacing * 0.55) clear2 = false;
            if (clear2) chosen.push_back(st.s);
        }
        if (sites.empty() && len >= 420.0)
            LOG_WARN << "[corridor] route len=" << len
                     << " has NO feasible interchange site (streets out of "
                        "reach everywhere)";
        for (const Real sI : chosen) {
            auto stamp = [&](Real st, bool up, bool on, Real side) {
                if (st < 60.0 || st > len - 60.0) return;
                const Vec2 gpos = def.horizontal.pos(st);
                const Vec2 gn2 = def.horizontal.normal(st);
                ExitDef e;
                e.station = st;
                e.upStation = up;
                e.onRamp = on;
                // Landing at THIS ramp's own station (not the shared
                // interchange centre) — a carriageway's exit and on-ramp sit
                // ~190 m apart along the corridor, so each claims a DISTINCT
                // street landing. Sharing the centre made the on-ramp land on
                // the exit's spot, which rule 3a (60 m apart) then rejected —
                // the audit's "0 on-ramps survive" bug.
                e.target = gpos + gn2 * (side * 95.0);
                // rule 3a: landings keep 60 m apart across ALL corridors
                for (const Vec2& u2 : globalLandings)
                    if ((u2 - e.target).length() < 60.0) return;
                // rule 3b: the ramp run must not cross another ramp
                for (const auto& rs2 : rampSegs)
                    if (segDist(gpos, rs2.first, rs2.second) < 30.0 ||
                        segDist(e.target, rs2.first, rs2.second) < 30.0)
                        return;
                // rule 3c: nor graze ANOTHER corridor at grade
                for (std::size_t pj = 0; pj < planned.size(); ++pj) {
                    if (pj == pi2 || planned[pj].dropped) continue;
                    for (const Vec2& q : planned[pj].dense)
                        if (segDist(q, gpos, e.target) < 26.0) return;
                }
                e.targetY = ground(e.target.x, e.target.y) + 0.12;
                e.decelLength = std::min(Real(200), len * 0.28);
                e.rampRadius = 65.0;
                e.rampSpiral = 28.0;
                globalLandings.push_back(e.target);
                rampSegs.push_back({gpos, e.target});
                def.exits.push_back(e);
            };
            stamp(sI - 80.0, true, false, -1.0);    // up exit
            stamp(sI + 110.0, true, true, -1.0);    // up on-ramp
            stamp(sI + 80.0, false, false, 1.0);    // down exit
            stamp(sI - 110.0, false, true, 1.0);    // down on-ramp
        }
        // R1.3 REVISED (device): the mainline is ITS OWN SYSTEM — it runs
        // across the map and ENDS, full-width, as a dead end. No taper into
        // surface streets, no terminus graft: ramps are the only couplings
        // between the two systems. (taperEnds stays available for
        // hand-authored corridors that want the funnel.)
        def.taperEnds = false;
        LOG_INFO << "[corridor] metro plan: len=" << len
                 << " bridges=" << pr.bridgeAt.size()
                 << " exits=" << def.exits.size();
        PlannedCorridor pc;
        pc.def = std::move(def);
        pc.guide = pr.dense;
        pc.srcNet = pr.srcNet;
        out.push_back(std::move(pc));
    }
    return out;
}

// §10.6: streets may pass UNDER a viaduct, never THROUGH an at-grade corridor
// — cut street edges that cross a low span of any corridor.
void cutStreetsUnderCorridors(const std::vector<RoadNet*>& nets,
                              const std::vector<CorridorDef>& defs,
                              const std::function<Real(Real, Real)>& ground) {
    if (!ground || defs.empty()) return;
    // A street passes UNDER only with real headroom: the clearance is the deck
    // UNDERSIDE (deck surface minus the box girder depth), not the deck
    // surface, and it must clear the ~5 m highway standard — the old check
    // used the deck surface at 4.5 m, so a street rammed the girder (device:
    // "literally no clearance with the things on the ground").
    const Real kDeckStruct = 2.75;    // deckDepth 1.9 + girder band 0.85
    const Real kMinUnderClear = 5.0;  // AASHTO ~16 ft over the roadway
    struct CorSample { Vec2 pos; Real clear; Real reach; };
    std::vector<CorSample> cs;
    for (const CorridorDef& def : defs) {
        const Real len = def.horizontal.length();
        for (Real s = 0; s <= len; s += 20.0) {
            const Vec2 p = def.horizontal.pos(s);
            cs.push_back({p,
                          def.vertical.elevation(s) - kDeckStruct -
                              ground(p.x, p.y),
                          def.halfWidthAt(s) + 7.0});
        }
    }
    int cut = 0;
    for (RoadNet* netp : nets) {
        RoadNet& net = *netp;
        std::vector<std::array<int, 2>> keptE;
        std::vector<double> keptW;
        std::vector<int> keptL;
        std::vector<RoadClass> keptC;
        std::vector<int> keptS;
        std::vector<uint8_t> keptB;
        const bool hasW = !net.edgeWidths.empty();
        const bool hasL = !net.edgeLayers.empty();
        const bool hasC = !net.edgeClasses.empty();
        const bool hasS = !net.edgeSpecs.empty();
        const bool hasB = !net.edgeBaked.empty();
        for (std::size_t ei = 0; ei < net.edges.size(); ++ei) {
            const auto& ed = net.edges[ei];
            bool blocked = false;
            if (ed[0] >= 0 && ed[1] >= 0 &&
                ed[0] < static_cast<int>(net.nodes.size()) &&
                ed[1] < static_cast<int>(net.nodes.size())) {
                const Vec2 A = net.nodes[ed[0]], B = net.nodes[ed[1]];
                for (int k = 0; k <= 8 && !blocked; ++k) {
                    const Vec2 q = A + (B - A) * (k / 8.0);
                    for (const CorSample& sm2 : cs) {
                        if (sm2.clear >= kMinUnderClear) continue;   // clears: ok
                        if ((q - sm2.pos).length() < sm2.reach) {
                            blocked = true;
                            break;
                        }
                    }
                }
            }
            if (blocked) { ++cut; continue; }
            keptE.push_back(ed);
            if (hasW)
                keptW.push_back(ei < net.edgeWidths.size()
                                    ? net.edgeWidths[ei] : 0.0);
            if (hasL)
                keptL.push_back(ei < net.edgeLayers.size()
                                    ? net.edgeLayers[ei] : 0);
            if (hasC)
                keptC.push_back(ei < net.edgeClasses.size()
                                    ? net.edgeClasses[ei] : RoadClass::Local);
            if (hasS)
                keptS.push_back(ei < net.edgeSpecs.size()
                                    ? net.edgeSpecs[ei] : -1);
            if (hasB)
                keptB.push_back(ei < net.edgeBaked.size()
                                    ? net.edgeBaked[ei] : 0);
        }
        net.edges.swap(keptE);
        if (hasW) net.edgeWidths.swap(keptW);
        if (hasL) net.edgeLayers.swap(keptL);
        if (hasC) net.edgeClasses.swap(keptC);
        if (hasS) net.edgeSpecs.swap(keptS);
        if (hasB) net.edgeBaked.swap(keptB);
    }
    if (cut) LOG_INFO << "[corridor] cut " << cut
                      << " street edges crossing at-grade spans";
}

// Ramps land ON city roads (device): each ramp claims a street JUNCTION node
// on its own side of the deck, nearest its gore — shortest usable ramp by
// construction. Dead-end stubs are last resort (device: "the road that it
// connects to is a dead end"), no two ramps may share or crowd a landing
// (device: "two of them overlap"), and a wrong-side node is never taken (it
// made an on-ramp dive under the mainline).
// §10.3: candidates are AUTHORED street-net nodes (not curve samples) — the
// landing must be a node the street net can grow a junction STUB from. The
// return records (net, node) per exit.
std::vector<std::pair<int, int>> resolveCorridorLandings(
    CorridorDef& def, const std::vector<RoadNet*>& nets,
    const std::function<Real(Real, Real)>& ground) {
    struct StreetAnchor { int net; int node; Vec2 pos; int degree; };
    std::vector<StreetAnchor> anchors;
    for (std::size_t ni = 0; ni < nets.size(); ++ni) {
        const RoadNet& net = *nets[ni];
        std::vector<int> deg(net.nodes.size(), 0);
        for (const auto& ed : net.edges) {
            if (ed[0] >= 0 && ed[0] < static_cast<int>(deg.size())) ++deg[ed[0]];
            if (ed[1] >= 0 && ed[1] < static_cast<int>(deg.size())) ++deg[ed[1]];
        }
        for (std::size_t k = 0; k < net.nodes.size(); ++k)
            anchors.push_back({static_cast<int>(ni), static_cast<int>(k),
                               net.nodes[k], deg[k]});
    }
    std::vector<std::pair<int, int>> rampAnchors(def.exits.size(), {-1, -1});
    std::vector<Vec2> used;
    for (std::size_t xi = 0; xi < def.exits.size(); ++xi) {
        ExitDef& e = def.exits[xi];
        const Real sg = std::max(
            Real(1), std::min(e.station, def.horizontal.length() - 1));
        const Vec2 gc = def.horizontal.pos(sg);
        const Vec2 gn = def.horizontal.normal(sg);
        const Real sideSign = e.upStation ? -1.0 : 1.0;
        const Vec2 travelDir = def.horizontal.tangent(sg) *
                               (e.upStation ? 1.0 : -1.0);
        auto usable = [&](const StreetAnchor& an, bool needJunction) {
            if (dot(an.pos - gc, gn) * sideSign < 12.0) return false;
            // flow direction: an exit unloads DOWNSTREAM, an on-ramp is fed
            // from UPSTREAM — a feeder on the wrong side makes the ramp
            // double back (device: "weirdly making cars do a U-turn")
            const Real along = dot(an.pos - gc, travelDir);
            if (e.onRamp ? along > -20.0 : along < 20.0) return false;
            const Real dg = (an.pos - gc).length();
            // the gore band + a turnable clothoid need ~130 m
            if (dg < 130.0 || dg > 360.0) return false;
            if (needJunction && an.degree < 2) return false;
            for (const Vec2& u : used)
                if ((u - an.pos).length() < 45.0) return false;
            return true;
        };
        bool found = false;
        for (const bool needJ : {true, false}) {
            Real best = 1e30;
            for (const StreetAnchor& an : anchors) {
                if (!usable(an, needJ)) continue;
                // Don't pick a target whose gore->landing run would cross an
                // ALREADY-resolved ramp's run — otherwise the criss-cross
                // guard drops one of them later, costing an on-ramp.
                // Re-target to a non-crossing node instead of losing the ramp.
                bool crossesResolved = false;
                for (std::size_t xj = 0; xj < xi && !crossesResolved; ++xj) {
                    if (def.exits[xj].station < 0) continue;
                    const Real sj = std::max(Real(1),
                        std::min(def.exits[xj].station,
                                 def.horizontal.length() - 1));
                    if (segsCross(gc, an.pos, def.horizontal.pos(sj),
                                  def.exits[xj].target))
                        crossesResolved = true;
                }
                if (crossesResolved) continue;
                // Rank by the TURN the ramp must make to reach the node, not
                // raw distance: the nearest node on a sparse (e.g. coastal)
                // grid is often near-perpendicular, forcing the ramp into a
                // ~90-116 deg cul-de-sac loop (measured). A target aligned
                // with the flow lets the ramp peel off gently. Keep a light
                // distance term so a marginally-smoother but far node isn't
                // chased across the map.
                const Vec2 rel = an.pos - gc;
                const Real along = std::fabs(dot(rel, travelDir));
                const Real lateral = std::fabs(dot(rel, gn));
                const Real turn = std::atan2(lateral, along);
                // Keep the nearest-node ranking (which preserves connectivity
                // on a sparse grid) but add a steep penalty for EXTREME
                // peel-off angles (> ~60 deg): those are the ~90-116 deg
                // cul-de-sac loops. This straightens the worst landings when a
                // better-angled node is not much farther, without aggressively
                // re-targeting every ramp (which caused crossings and dropped
                // on-ramps).
                const Real dist = rel.length();
                const Real loopPenalty =
                    std::max(Real(0), turn - Real(1.05)) * 260.0;
                const Real cost = dist + loopPenalty;
                if (cost < best) {
                    best = cost;
                    e.target = an.pos;
                    rampAnchors[xi] = {an.net, an.node};
                    found = true;
                }
            }
            if (found) break;
        }
        if (!found) {
            // T-JUNCTION FALLBACK (device: "they should connect to the
            // closest road even if they have to make a new road graph node"):
            // no existing junction is reachable, so project the ideal landing
            // onto the nearest street EDGE on the ramp's own side + flow and
            // SPLIT it, inserting a new node there (a real T-junction the
            // stub then grafts to).
            const Vec2 ideal = gc + gn * (sideSign * 165.0);
            Real best = 1e30;
            int bNet = -1, bEdge = -1;
            Vec2 bProj;
            for (std::size_t ni = 0; ni < nets.size(); ++ni) {
                const RoadNet& net = *nets[ni];
                for (std::size_t ei = 0; ei < net.edges.size(); ++ei) {
                    const auto& ed = net.edges[ei];
                    if (ed[0] < 0 || ed[1] < 0 ||
                        ed[0] >= (int)net.nodes.size() ||
                        ed[1] >= (int)net.nodes.size()) continue;
                    const Vec2 A = net.nodes[ed[0]];
                    const Vec2 B = net.nodes[ed[1]];
                    const Vec2 AB = B - A;
                    const Real l2 = AB.lengthSquared();
                    if (l2 < 400.0) continue;   // skip short stubs
                    Real t = dot(ideal - A, AB) / l2;
                    if (t < 0.12 || t > 0.88) continue;   // not at a node
                    const Vec2 q = A + AB * t;
                    if (dot(q - gc, gn) * sideSign < 12.0) continue;
                    const Real along = dot(q - gc, travelDir);
                    if (e.onRamp ? along > -20.0 : along < 20.0) continue;
                    const Real dg = (q - gc).length();
                    if (dg < 110.0 || dg > 380.0) continue;
                    bool clash = false;
                    for (const Vec2& u : used)
                        if ((u - q).length() < 45.0) { clash = true; break; }
                    if (clash) continue;
                    const Real d = (ideal - q).length();
                    if (d < best) { best = d; bNet = (int)ni; bEdge = (int)ei; bProj = q; }
                }
            }
            if (bNet >= 0) {
                RoadNet& net = *nets[bNet];
                if (net.tangents.size() < net.nodes.size())
                    net.tangents.resize(net.nodes.size(), Vec2(0, 0));
                if (net.edgeWidths.size() < net.edges.size())
                    net.edgeWidths.resize(net.edges.size(), 0.0);
                if (net.edgeLayers.size() < net.edges.size())
                    net.edgeLayers.resize(net.edges.size(), 0);
                if (!net.edgeClasses.empty() &&
                    net.edgeClasses.size() < net.edges.size())
                    net.edgeClasses.resize(net.edges.size(), RoadClass::Local);
                if (!net.edgeSpecs.empty() &&
                    net.edgeSpecs.size() < net.edges.size())
                    net.edgeSpecs.resize(net.edges.size(), -1);
                if (!net.edgeBaked.empty() &&
                    net.edgeBaked.size() < net.edges.size())
                    net.edgeBaked.resize(net.edges.size(), 0);
                const std::array<int, 2> ab = net.edges[bEdge];
                const double ew = net.edgeWidths[bEdge];
                const int el = net.edgeLayers[bEdge];
                const RoadClass ek = (bEdge < (int)net.edgeClasses.size())
                                         ? net.edgeClasses[bEdge]
                                         : RoadClass::Local;
                const int N = (int)net.nodes.size();
                net.nodes.push_back(bProj);
                net.tangents.push_back(Vec2(0, 0));
                net.edges[bEdge] = {ab[0], N};    // (a -> new)
                net.edges.push_back({N, ab[1]});  // (new -> b)
                net.edgeWidths.push_back(ew);
                net.edgeLayers.push_back(el);
                if (!net.edgeClasses.empty())
                    net.edgeClasses.push_back(ek);   // both halves keep the class
                if (!net.edgeSpecs.empty())
                    net.edgeSpecs.push_back(
                        bEdge < (int)net.edgeSpecs.size() ? net.edgeSpecs[bEdge]
                                                          : -1);
                if (!net.edgeBaked.empty()) net.edgeBaked.push_back(0);
                rampAnchors[xi] = {bNet, N};
                e.target = bProj;
                found = true;   // the final else records `used`
                LOG_INFO << "[corridor] ramp at s=" << e.station
                         << ": split nearest street edge -> new T-junction at ("
                         << bProj.x << ", " << bProj.y << ")";
            }
        }
        if (!found) {
            LOG_WARN << "[corridor] ramp at s=" << e.station
                     << ": no street edge reachable on its side — DROPPED";
            e.station = -1;   // sentinel: mesh + nav skip it
        } else {
            used.push_back(e.target);
        }
        if (ground) e.targetY = ground(e.target.x, e.target.y) + 0.12;
    }
    // §12 CRISS-CROSS GUARD (device: "I see criss-crossing onramps"): the
    // independent landing snap can leave two ramps' gore->landing runs
    // crossing each other. Test the RESOLVED segments pairwise and drop the
    // lower-priority ramp (on-ramps before exits, then later station) so no
    // two ramp runs X. (segsCross shared with landing selection — which
    // already avoids most crossings at pick time.)
    for (std::size_t xi = 0; xi < def.exits.size(); ++xi) {
        if (def.exits[xi].station < 0) continue;
        const Real si = std::max(Real(1),
            std::min(def.exits[xi].station, def.horizontal.length() - 1));
        const Vec2 gi2 = def.horizontal.pos(si);
        for (std::size_t xj = 0; xj < xi; ++xj) {
            if (def.exits[xj].station < 0) continue;
            const Real sj = std::max(Real(1),
                std::min(def.exits[xj].station, def.horizontal.length() - 1));
            const Vec2 gj = def.horizontal.pos(sj);
            if (!segsCross(gi2, def.exits[xi].target,
                           gj, def.exits[xj].target)) continue;
            // drop the lower priority: on-ramp over exit, else later
            std::size_t drop = def.exits[xi].onRamp && !def.exits[xj].onRamp
                                   ? xi
                               : (!def.exits[xi].onRamp && def.exits[xj].onRamp
                                      ? xj : xi);
            LOG_INFO << "[corridor] ramp at s=" << def.exits[drop].station
                     << " crosses another ramp — dropped";
            def.exits[drop].station = -1;
            if (drop == xi) break;
        }
    }
    return rampAnchors;
}

int rebakeNetCorridors(RoadNet& net, Real interchangeSpacing) {
    if (!net.heightAt || net.freewayPlans.empty()) return 0;
    const std::function<Real(Real, Real)> ground =
        [h = net.heightAt](Real x, Real z) { return h(x, z); };
    std::vector<CorridorRouteInput> routes;
    for (const std::vector<Vec2>& plan : net.freewayPlans) {
        if (plan.size() < 2) continue;
        routes.push_back({plan, interchangeSpacing, 0});
    }
    if (routes.empty()) return 0;
    std::vector<CorridorStreetAnchor> anchors;
    {
        std::vector<int> deg(net.nodes.size(), 0);
        for (const auto& ed : net.edges) {
            if (ed[0] >= 0 && ed[0] < static_cast<int>(deg.size())) ++deg[ed[0]];
            if (ed[1] >= 0 && ed[1] < static_cast<int>(deg.size())) ++deg[ed[1]];
        }
        for (std::size_t k = 0; k < net.nodes.size(); ++k)
            anchors.push_back({net.nodes[k], deg[k]});
    }
    std::vector<PlannedCorridor> planned =
        planCorridorRoutes(routes, anchors, ground);
    if (planned.empty()) return 0;
    const std::vector<RoadNet*> nets{&net};
    {
        std::vector<CorridorDef> defs;
        defs.reserve(planned.size());
        for (const PlannedCorridor& pc : planned) defs.push_back(pc.def);
        cutStreetsUnderCorridors(nets, defs, ground);
    }
    int baked = 0;
    for (PlannedCorridor& pc : planned) {
        resolveCorridorLandings(pc.def, nets, ground);
        const CorridorAuthoring au = corridorAuthor(pc.def, ground, 3.0);
        bakeCorridorIntoNet(net, pc.def, au.rampPaths);
        ++baked;
        LOG_INFO << "[bake] regenerate: corridor re-baked (+"
                 << pc.def.exits.size() << " exits planned)";
    }
    return baked;
}

}  // namespace engine
