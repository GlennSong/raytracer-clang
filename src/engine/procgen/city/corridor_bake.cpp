#include "corridor_bake.h"

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

std::vector<int> bakeCorridorIntoNet(RoadNet& net, const CorridorDef& def,
                                     const std::vector<RampPath>& rampPaths,
                                     const CorridorBakeParams& params) {
    std::vector<int> mainline;
    const Real L = def.horizontal.length();
    if (L < 1.0) return mainline;
    const int streetNodes = static_cast<int>(net.nodes.size());

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
        }
    }
    // Tangent array stays parallel even if the last pushes were snaps.
    if (net.tangents.size() < net.nodes.size())
        net.tangents.resize(net.nodes.size(), Vec2(0, 0));
    return mainline;
}

}  // namespace engine
