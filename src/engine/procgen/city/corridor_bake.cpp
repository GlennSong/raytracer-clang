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
    net.edges.push_back({ a, b });
    net.edgeWidths.push_back(width);
    net.edgeLayers.push_back(layer);
    net.edgeClasses.push_back(klass);
    net.edgeSpecs.push_back(spec);
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

    // --- Bake the mainline chain.
    for (double s : filled) {
        const Vec2 p = def.horizontal.pos(static_cast<Real>(s));
        const double y = def.vertical.elevation(static_cast<Real>(s));
        const int ni = pushNode(net, p, y);
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

        const std::vector<Vec3>& pts = rampPaths[xi].pts;
        int prev = gore;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const Vec2 q(pts[k].x, pts[k].z);
            const bool last = (k + 1 == pts.size());
            int ni;
            if (last) {
                // Landing: REUSE the street's own node when one is close — the
                // graft that makes "the ramp ends on a street node" literal.
                int snap = -1;
                double sd = params.landingSnap * params.landingSnap;
                for (int v = 0; v < streetNodes; ++v) {
                    const double d = (net.nodes[v] - q).lengthSquared();
                    if (d < sd) { sd = d; snap = v; }
                }
                ni = (snap >= 0)
                         ? snap
                         : pushNode(net, q, std::numeric_limits<double>::quiet_NaN());
            } else {
                ni = pushNode(net, q, pts[k].y);   // descent rides authored heights
            }
            if (ni != prev)
                pushEdge(net, prev, ni, rampWidth, RoadClass::Ramp, /*layer=*/0, rampSpec);
            prev = ni;
        }
    }
    return mainline;
}

}  // namespace engine
