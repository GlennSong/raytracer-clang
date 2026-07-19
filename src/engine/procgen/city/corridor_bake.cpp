#include "corridor_bake.h"

#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

namespace {

// Find-or-add a named preset in the net's spec table.
int ensureSpec(RoadNet& net, const std::string& preset) {
    for (int i = 0; i < static_cast<int>(net.specs.size()); ++i)
        if (net.specs[i].name == preset) return i;
    net.specs.push_back(roadSpecPreset(preset));
    return static_cast<int>(net.specs.size()) - 1;
}

// Pad every parallel-to-edges array to the current edge count, then push one
// entry for a new edge. Baking must never leave the arrays ragged — a short
// parallel array silently means "default" for every edge past its end.
void pushEdge(RoadNet& net, int a, int b, double width, RoadClass klass,
              int layer, int spec) {
    const std::size_t n = net.edges.size();
    net.edgeWidths.resize(n, 0.0);
    net.edgeLayers.resize(n, 0);
    net.edgeClasses.resize(n, RoadClass::Local);
    net.edgeSpecs.resize(n, -1);
    net.edgeBaked.resize(n, 0);
    net.edges.push_back({ a, b });
    net.edgeWidths.push_back(width);
    net.edgeLayers.push_back(layer);
    net.edgeClasses.push_back(klass);
    net.edgeSpecs.push_back(spec);
    net.edgeBaked.push_back(1);        // street passes skip; corridor draws itself
}

// Add a node with an (optionally absolute) deck height. nodeElev is parallel
// to nodes; NaN = at grade (drape).
int pushNode(RoadNet& net, const Vec2& p, double elev) {
    const std::size_t n = net.nodes.size();
    net.nodeElev.resize(n, std::numeric_limits<double>::quiet_NaN());
    net.nodes.push_back(p);
    net.nodeElev.push_back(elev);
    return static_cast<int>(net.nodes.size()) - 1;
}

// Stamp a semantic-layer hint (#17) on a net node: the bake KNOWS which
// node is a gore (and whether the ramp joins or leaves) and which street
// node is a landing — geometry alone cannot tell a Merge from a mirrored
// Diverge. classifyRoadGraph honours these at degree >= 3.
void stampKind(RoadNet& net, int ni, JunctionKind k) {
    if (ni < 0) return;
    if (net.nodeKinds.size() < net.nodes.size())
        net.nodeKinds.resize(net.nodes.size(),
                             static_cast<uint8_t>(JunctionKind::Auto));
    net.nodeKinds[ni] = static_cast<uint8_t>(k);
}

// Store an authored tangent for a baked node (parallel array padded). The
// magnitude follows the Catmull-Rom convention netGraph's Hermite uses
// (~half the span between neighbouring nodes), so sparse nodes reproduce
// the solved curve instead of chording it.
void setTangent(RoadNet& net, int ni, const Vec2& t) {
    if (net.tangents.size() < net.nodes.size())
        net.tangents.resize(net.nodes.size(), Vec2(0, 0));
    net.tangents[ni] = t;
}

}  // namespace

// The bake-time PLANARITY audit (#18): does any edge in [e0, end) cross an
// EARLIER edge in XZ — sharing no endpoint position — with less than
// `clearance` metres of vertical gap at the crossing? Node Y = authored
// nodeElev where finite, else terrain (net.heightAt). This is the check the
// planner's §12 guard never did: it tested the straight gore->target CHORD,
// never the authored clothoid the ramp actually drives.
bool chainCrossesNet(const RoadNet& net, std::size_t e0, double clearance) {
    auto nodeY = [&](int v) {
        if (v < static_cast<int>(net.nodeElev.size()) &&
            std::isfinite(net.nodeElev[v]))
            return net.nodeElev[v];
        return net.heightAt ? net.heightAt(net.nodes[v].x, net.nodes[v].y) : 0.0;
    };
    for (std::size_t i = e0; i < net.edges.size(); ++i) {
        const Vec2 p = net.nodes[net.edges[i][0]];
        const Vec2 q = net.nodes[net.edges[i][1]];
        for (std::size_t j = 0; j < e0; ++j) {
            const Vec2 r = net.nodes[net.edges[j][0]];
            const Vec2 s2 = net.nodes[net.edges[j][1]];
            // Shared endpoint (by position): adjacency, not a crossing.
            const double tol2 = 1.0;
            if ((p - r).lengthSquared() < tol2 || (p - s2).lengthSquared() < tol2 ||
                (q - r).lengthSquared() < tol2 || (q - s2).lengthSquared() < tol2)
                continue;
            const Vec2 d1 = q - p, d2 = s2 - r;
            const double den = d1.x * d2.y - d1.y * d2.x;
            if (std::fabs(den) < 1e-9) continue;
            const Vec2 pr = r - p;
            const double t = (pr.x * d2.y - pr.y * d2.x) / den;
            const double u = (pr.x * d1.y - pr.y * d1.x) / den;
            if (t <= 0.02 || t >= 0.98 || u <= 0.02 || u >= 0.98) continue;
            const double ya = nodeY(net.edges[i][0]) +
                              (nodeY(net.edges[i][1]) - nodeY(net.edges[i][0])) * t;
            const double yb = nodeY(net.edges[j][0]) +
                              (nodeY(net.edges[j][1]) - nodeY(net.edges[j][0])) * u;
            if (std::fabs(ya - yb) < clearance) {
                LOG_ERROR << "[bake] ramp chain crosses an existing road at ("
                          << (p + d1 * t).x << ", " << (p + d1 * t).y
                          << ") with only " << std::fabs(ya - yb)
                          << " m clearance — ramp dropped";
                return true;
            }
        }
    }
    return false;
}

std::vector<int> bakeCorridorIntoNet(RoadNet& net, const CorridorDef& def,
                                     const std::vector<RampPath>& rampPaths,
                                     const CorridorBakeParams& params) {
    std::vector<int> mainline;
    const Real L = def.horizontal.length();
    if (L < 1.0) return mainline;
    const int streetNodes = static_cast<int>(net.nodes.size());
    const std::size_t firstMainlineEdge = net.edges.size();

    // The SAMPLED existing net (#18): streets are Catmull-Rom curves in the
    // derived graph — a chord-level test misses a ramp slicing through a
    // street's BOW (the metro audit caught exactly that at dY = 0). The
    // authored ramp polylines are tested against these curve segments; the
    // net grows corridor-by-corridor, so earlier corridors are included.
    const RoadGraph sampled = navRoadGraph(net);
    auto sampledY = [&](int v) {
        const RoadNode& nd = sampled.nodes[v];
        if (nd.elevAbsolute) return static_cast<double>(nd.elev);
        return (net.heightAt ? net.heightAt(nd.pos.x, nd.pos.y) : 0.0) +
               static_cast<double>(nd.elev);
    };
    // Does the authored polyline cross a sampled segment at like height,
    // away from its own two ends (gore + landing adjacency)?
    auto rampPathCrosses = [&](const std::vector<Vec3>& path) {
        if (path.size() < 2) return false;
        const Vec2 endA(path.front().x, path.front().z);
        const Vec2 endB(path.back().x, path.back().z);
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            const Vec2 p(path[i].x, path[i].z), q(path[i + 1].x, path[i + 1].z);
            for (const RoadEdge& e : sampled.edges) {
                const Vec2 r = sampled.nodes[e.a].pos;
                const Vec2 s2 = sampled.nodes[e.b].pos;
                const Vec2 d1 = q - p, d2 = s2 - r;
                const double den = d1.x * d2.y - d1.y * d2.x;
                if (std::fabs(den) < 1e-9) continue;
                const Vec2 pr = r - p;
                const double t = (pr.x * d2.y - pr.y * d2.x) / den;
                const double u = (pr.x * d1.y - pr.y * d1.x) / den;
                if (t <= 0.0 || t >= 1.0 || u <= 0.02 || u >= 0.98) continue;
                const Vec2 at = p + d1 * t;
                if ((at - endA).length() < 6.0 || (at - endB).length() < 6.0)
                    continue;   // its own gore / landing node adjacency
                const double yr = path[i].y + (path[i + 1].y - path[i].y) * t;
                const double ys = sampledY(e.a) + (sampledY(e.b) - sampledY(e.a)) * u;
                if (std::fabs(yr - ys) < 4.5) {
                    LOG_ERROR << "[bake] authored ramp crosses a road curve at ("
                              << at.x << ", " << at.y << ") with only "
                              << std::fabs(yr - ys)
                              << " m clearance — ramp dropped";
                    return true;
                }
            }
        }
        return false;
    };

    const int fwySpec = ensureSpec(net, "freeway3");
    const int rampSpec = ensureSpec(net, "ramp1");
    const double fwyWidth = net.specs[fwySpec].carriagewayWidth();
    const double rampWidth = net.specs[rampSpec].carriagewayWidth();

    // --- Mainline stations: 0, L, every gore (exactly — the gore must BE a
    // node so the ramp can attach by id), and even fill at ~mainlineStep.
    std::vector<double> gores;                     // parallel to exits, <0 = none
    std::vector<double> stations{ 0.0, static_cast<double>(L) };
    for (std::size_t xi = 0; xi < def.exits.size(); ++xi) {
        double sg = -1.0;
        if (def.exits[xi].station >= 0 &&
            xi < rampPaths.size() && rampPaths[xi].pts.size() >= 2) {
            sg = std::min<double>(std::max(1.0, def.exits[xi].station), L - 1.0);
            stations.push_back(sg);
        }
        gores.push_back(sg);
    }
    std::sort(stations.begin(), stations.end());
    stations.erase(std::unique(stations.begin(), stations.end(),
                               [](double a, double b) { return std::fabs(a - b) < 1e-3; }),
                   stations.end());
    // Even fill between the pinned stations.
    std::vector<double> filled;
    for (std::size_t i = 0; i + 1 < stations.size(); ++i) {
        filled.push_back(stations[i]);
        const double gap = stations[i + 1] - stations[i];
        const int div = std::max(1, static_cast<int>(std::ceil(gap / params.mainlineStep)));
        for (int k = 1; k < div; ++k) filled.push_back(stations[i] + gap * k / div);
    }
    filled.push_back(stations.back());

    // --- Bake the mainline chain, tangents from the solved alignment (2a).
    for (std::size_t fi = 0; fi < filled.size(); ++fi) {
        const double s = filled[fi];
        const Vec2 p = def.horizontal.pos(static_cast<Real>(s));
        const double y = def.vertical.elevation(static_cast<Real>(s));
        const int ni = pushNode(net, p, y);
        Vec2 tan = def.horizontal.tangent(static_cast<Real>(s));
        const double tl = tan.length();
        if (tl > 1e-9) {
            const double prevS = fi > 0 ? filled[fi - 1] : s;
            const double nextS = fi + 1 < filled.size() ? filled[fi + 1] : s;
            const double span = std::max(1.0, (nextS - prevS) * 0.5);
            setTangent(net, ni, tan * (span / tl));
        }
        if (!mainline.empty())
            pushEdge(net, mainline.back(), ni, fwyWidth, RoadClass::Freeway,
                     /*layer=*/1, fwySpec);
        mainline.push_back(ni);
    }

    // MAINLINE PLANARITY (#18, review S4b): the ramp loop audits each ramp
    // against everything already baked, but nothing audited the MAINLINE
    // itself against the pre-existing streets. An authored freeway that dips
    // toward the surface (a sag in def.vertical) could slice a street at
    // like height. The mainline is the backbone — rolling it back would
    // orphan the whole interchange — so this WARNS loudly instead of
    // dropping: an authoring error the level must fix.
    if (chainCrossesNet(net, static_cast<std::size_t>(streetNodes) == 0
                                 ? 0
                                 : firstMainlineEdge,
                        4.5)) {
        LOG_ERROR << "[bake] freeway MAINLINE crosses a surface road at like "
                     "height — check the corridor's vertical profile";
    }

    // --- Bake each surviving ramp: gore node -> authored descent -> landing.
    for (std::size_t xi = 0; xi < gores.size(); ++xi) {
        if (gores[xi] < 0) continue;
        // The gore node: the mainline node at this station (exact by construction).
        int gore = -1;
        double best = 1e18;
        const Vec2 gp = def.horizontal.pos(static_cast<Real>(gores[xi]));
        for (int ni : mainline) {
            const double d = (net.nodes[ni] - gp).lengthSquared();
            if (d < best) { best = d; gore = ni; }
        }
        if (gore < 0 || best > 1.0) continue;      // must be exact, not near
        if (rampPathCrosses(rampPaths[xi].pts)) continue;   // #18: author-curve audit
        // Snapshot every parallel array: if this ramp's authored chain turns
        // out to CROSS the net (#18), the whole chain rolls back and the
        // gore/landing kinds are never stamped.
        const std::size_t snapNodes = net.nodes.size();
        const std::size_t snapEdges = net.edges.size();
        const std::size_t snapTang = net.tangents.size();
        const std::size_t snapKinds = net.nodeKinds.size();

        // 2a RIBBON EXTRACTION: drop the gore band (junction surface, owned
        // by the 2c gore treatment) and orient the walk GORE -> LANDING. An
        // on-ramp's authored points run street -> merge with the band at the
        // BACK, so its ribbon is walked in reverse — the old bake walked
        // every ramp forward and snap-searched the DECK end for a street
        // landing: every on-ramp baked backwards with a floating landing.
        const RampPath& rp0 = rampPaths[xi];
        const bool onRamp = def.exits[xi].onRamp;
        std::vector<Vec3> rp;
        if (onRamp) {
            const int hi = static_cast<int>(rp0.pts.size()) - rp0.bandBack;
            for (int k = hi - 1; k >= 0; --k) rp.push_back(rp0.pts[k]);
        } else {
            for (std::size_t k = rp0.bandFront; k < rp0.pts.size(); ++k)
                rp.push_back(rp0.pts[k]);
        }
        if (rp.size() < 2) continue;

        std::vector<double> arc(rp.size(), 0.0);
        for (std::size_t k = 1; k < rp.size(); ++k)
            arc[k] = arc[k - 1] + (Vec2(rp[k].x, rp[k].z) -
                                   Vec2(rp[k - 1].x, rp[k - 1].z)).length();
        // SHAPE-DRIVEN node selection (Douglas-Peucker, in 3D): keep nodes
        // where the ribbon bends in PLAN (clothoid onsets) or in PROFILE
        // (the descent's vertical knees) — a 2D DP left plan-straight steep
        // sections chorded 0.45 m proud, which the drive probe read as a
        // 0.17 m washboard. Bounded by rampStep*1.8 so no span outruns
        // local editability.
        std::vector<std::size_t> keep{ 0, rp.size() - 1 };
        {
            const double tol = 0.18;
            std::vector<std::pair<std::size_t, std::size_t>> stack{
                { 0, rp.size() - 1 } };
            while (!stack.empty()) {
                const auto seg = stack.back();
                stack.pop_back();
                if (seg.second <= seg.first + 1) continue;
                const Vec3& A = rp[seg.first];
                const Vec3& B = rp[seg.second];
                const Vec3 AB = B - A;
                const double l2 = dot(AB, AB);
                double worst = -1.0;
                std::size_t at = seg.first;
                for (std::size_t k = seg.first + 1; k < seg.second; ++k) {
                    const Vec3& P3 = rp[k];
                    double t = l2 > 1e-12 ? dot(P3 - A, AB) / l2 : 0.0;
                    t = t < 0 ? 0 : (t > 1 ? 1 : t);
                    const Vec3 cp = A + AB * t - P3;
                    const double d = std::sqrt(dot(cp, cp));
                    if (d > worst) { worst = d; at = k; }
                }
                const bool tooLong =
                    arc[seg.second] - arc[seg.first] > params.rampStep * 1.8;
                if (worst > tol || tooLong) {
                    const std::size_t mid =
                        worst > tol ? at : (seg.first + seg.second) / 2;
                    keep.push_back(mid);
                    stack.push_back({ seg.first, mid });
                    stack.push_back({ mid, seg.second });
                }
            }
            std::sort(keep.begin(), keep.end());
            keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
        }
        auto authoredDir = [&](std::size_t k) {
            const std::size_t k0 = k > 0 ? k - 1 : k;
            const std::size_t k1 = k + 1 < rp.size() ? k + 1 : k;
            Vec2 d(rp[k1].x - rp[k0].x, rp[k1].z - rp[k0].z);
            const double l = d.length();
            return l > 1e-9 ? d * (1.0 / l) : Vec2(0, 0);
        };
        int prev = gore;
        int landing = -1;
        for (std::size_t ki = 0; ki < keep.size(); ++ki) {
            const std::size_t k = keep[ki];
            const Vec2 q(rp[k].x, rp[k].z);
            const int ni = pushNode(net, q, rp[k].y);   // rides authored heights
            // Tangent: authored direction, Catmull-Rom magnitude (half the
            // arc span between the neighbouring KEPT nodes).
            const double prevA = ki > 0 ? arc[keep[ki - 1]] : arc[k];
            const double nextA = ki + 1 < keep.size() ? arc[keep[ki + 1]] : arc[k];
            const Vec2 dir = authoredDir(k);
            if (!(dir.x == 0.0 && dir.y == 0.0))
                setTangent(net, ni, dir * std::max(1.0, (nextA - prevA) * 0.5));
            if (ni != prev)
                pushEdge(net, prev, ni, rampWidth, RoadClass::Ramp, /*layer=*/0, rampSpec);
            prev = ni;
        }
        // LANDING GRAFT: the ribbon's authored end stays a real node (the
        // chain is faithful to the solved curve to its last metre); the final
        // edge grafts it onto the street's own node — "the ramp ends on a
        // street node", as an EDGE, not by substituting the ribbon end (the
        // substitution cut the last ~20 m corner 3.7 m wide).
        {
            const Vec2 end(rp.back().x, rp.back().z);
            int snap = -1;
            double sd = params.landingSnap * params.landingSnap;
            for (int v = 0; v < streetNodes; ++v) {
                const double d = (net.nodes[v] - end).lengthSquared();
                if (d < sd) { sd = d; snap = v; }
            }
            if (snap >= 0 && snap != prev)
                pushEdge(net, prev, snap, rampWidth, RoadClass::Ramp, /*layer=*/0,
                         rampSpec);
            landing = snap;
        }

        // PLANARITY AUDIT (#18): the authored chain against everything
        // already in the net (streets + mainline + earlier ramps). On a
        // violation the chain rolls back whole — a criss-crossed ramp never
        // enters the graph.
        if (chainCrossesNet(net, snapEdges, 4.5)) {
            net.nodes.resize(snapNodes);
            net.nodeElev.resize(std::min(net.nodeElev.size(), snapNodes));
            net.edges.resize(snapEdges);
            net.edgeWidths.resize(std::min(net.edgeWidths.size(), snapEdges));
            net.edgeLayers.resize(std::min(net.edgeLayers.size(), snapEdges));
            net.edgeClasses.resize(std::min(net.edgeClasses.size(), snapEdges));
            net.edgeSpecs.resize(std::min(net.edgeSpecs.size(), snapEdges));
            net.edgeBaked.resize(std::min(net.edgeBaked.size(), snapEdges));
            net.tangents.resize(snapTang);
            net.nodeKinds.resize(std::min(net.nodeKinds.size(), snapKinds));
            continue;
        }
        stampKind(net, gore,
                  def.exits[xi].onRamp ? JunctionKind::Merge
                                       : JunctionKind::Diverge);
        if (landing >= 0) stampKind(net, landing, JunctionKind::Landing);
    }
    // Tangent array stays parallel even if the last pushes were snaps.
    if (net.tangents.size() < net.nodes.size())
        net.tangents.resize(net.nodes.size(), Vec2(0, 0));
    return mainline;
}

}  // namespace engine
