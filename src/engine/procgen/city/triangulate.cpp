#include "triangulate.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// A compact port of mapbox/earcut.hpp (ISC license). Vertices live in a doubly-
// linked ring; holes are bridged into the outer ring, then ears are clipped with
// a validity test over the whole ring. Computation is in double regardless of
// Real for robustness on near-degenerate junction boundaries.
struct Node {
    int i;              // index into the flat vertex list
    double x, y;
    int prev = -1, next = -1;
    bool steiner = false;
};

struct Earcut {
    std::vector<Node> nodes;
    std::vector<std::array<int, 3>> tris;

    int makeNode(int i, double x, double y) {
        nodes.push_back({i, x, y, -1, -1, false});
        return static_cast<int>(nodes.size()) - 1;
    }
    double area(int p, int q, int r) const {
        return (nodes[q].y - nodes[p].y) * (nodes[r].x - nodes[q].x) -
               (nodes[q].x - nodes[p].x) * (nodes[r].y - nodes[q].y);
    }
    bool equals(int a, int b) const {
        return nodes[a].x == nodes[b].x && nodes[a].y == nodes[b].y;
    }

    // Build a circular doubly-linked ring from a list of flat vertex indices.
    // Matches mapbox: signedArea = sum of (x_j - x_i)(y_i + y_j), j = prev;
    // insert forward when clockwise == (signedArea > 0). The outer ring is built
    // with clockwise=true and holes with clockwise=false, so they end up counter-
    // oriented as the ear clip needs, independent of the caller's winding.
    int linkedList(const std::vector<int>& ring, const std::vector<Vec2>& pts,
                   bool clockwise) {
        int n = static_cast<int>(ring.size());
        double a = 0;
        for (int i = 0, j = n - 1; i < n; j = i++)
            a += (static_cast<double>(pts[ring[j]].x) - pts[ring[i]].x) *
                 (static_cast<double>(pts[ring[i]].y) + pts[ring[j]].y);
        int last = -1;
        auto insert = [&](int idx) {
            int nd = makeNode(ring[idx], pts[ring[idx]].x, pts[ring[idx]].y);
            if (last == -1) { nodes[nd].prev = nd; nodes[nd].next = nd; }
            else {
                nodes[nd].prev = last;
                nodes[nd].next = nodes[last].next;
                nodes[nodes[last].next].prev = nd;
                nodes[last].next = nd;
            }
            last = nd;
        };
        if (clockwise == (a > 0)) for (int i = 0; i < n; i++) insert(i);
        else for (int i = n - 1; i >= 0; i--) insert(i);
        return last;
    }

    bool pointInTriangle(double ax, double ay, double bx, double by, double cx,
                         double cy, double px, double py) const {
        return (cx - px) * (ay - py) - (ax - px) * (cy - py) >= 0 &&
               (ax - px) * (by - py) - (bx - px) * (ay - py) >= 0 &&
               (bx - px) * (cy - py) - (cx - px) * (by - py) >= 0;
    }
    bool isEar(int ear) const {
        int a = nodes[ear].prev, b = ear, c = nodes[ear].next;
        if (area(a, b, c) >= 0) return false;   // reflex — not an ear
        int p = nodes[c].next;
        while (p != a) {
            if (pointInTriangle(nodes[a].x, nodes[a].y, nodes[b].x, nodes[b].y,
                                nodes[c].x, nodes[c].y, nodes[p].x, nodes[p].y) &&
                area(nodes[p].prev, p, nodes[p].next) >= 0)
                return false;
            p = nodes[p].next;
        }
        return true;
    }
    int removeNode(int p) {   // unlink p, return its prev
        nodes[nodes[p].prev].next = nodes[p].next;
        nodes[nodes[p].next].prev = nodes[p].prev;
        return nodes[p].prev;
    }
    void earcutLinked(int ear, int pass = 0) {
        if (ear == -1) return;
        int stop = ear, prev, next;
        while (nodes[ear].prev != nodes[ear].next) {
            prev = nodes[ear].prev; next = nodes[ear].next;
            if (isEar(ear)) {
                tris.push_back({nodes[prev].i, nodes[ear].i, nodes[next].i});
                removeNode(ear);
                ear = nodes[next].next; stop = nodes[next].next;
                continue;
            }
            ear = next;
            if (ear == stop) {
                // No ear this round; escalate through mapbox's recovery passes.
                if (pass == 0) { earcutLinked(filterPoints(ear), 1); return; }
                else if (pass == 1) { ear = cureLocalIntersections(ear); earcutLinked(ear, 2); return; }
                else { splitEarcut(ear); return; }
            }
        }
    }
    int filterPoints(int start, int end = -1) {
        if (start == -1) return start;
        if (end == -1) end = start;
        int p = start; bool again;
        do {
            again = false;
            if (!nodes[p].steiner && (equals(p, nodes[p].next) ||
                area(nodes[p].prev, p, nodes[p].next) == 0)) {
                p = removeNode(p); end = p;
                if (p == nodes[p].next) break;
                again = true;
            } else p = nodes[p].next;
        } while (again || p != end);
        return end;
    }
    int cureLocalIntersections(int start) {
        int p = start;
        do {
            int a = nodes[p].prev, b = nodes[nodes[p].next].next;
            if (!equals(a, b) && intersects(a, p, nodes[p].next, b) &&
                locallyInside(a, b) && locallyInside(b, a)) {
                tris.push_back({nodes[a].i, nodes[p].i, nodes[b].i});
                removeNode(p); removeNode(nodes[p].next);
                p = start = b;
            }
            p = nodes[p].next;
        } while (p != start);
        return filterPoints(p);
    }
    bool intersects(int p1, int q1, int p2, int q2) const {
        auto sign = [&](double v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); };
        int o1 = sign(area(p1, q1, p2)), o2 = sign(area(p1, q1, q2)),
            o3 = sign(area(p2, q2, p1)), o4 = sign(area(p2, q2, q1));
        return (o1 != o2 && o3 != o4);
    }
    bool locallyInside(int a, int b) const {
        return area(nodes[a].prev, a, nodes[a].next) < 0
            ? area(a, b, nodes[a].next) >= 0 && area(a, nodes[a].prev, b) >= 0
            : area(a, b, nodes[a].prev) < 0 || area(a, nodes[a].next, b) < 0;
    }
    void splitEarcut(int start) {
        int a = start;
        do {
            int b = nodes[nodes[a].next].next;
            while (b != nodes[a].prev) {
                if (nodes[a].i != nodes[b].i && isValidDiagonal(a, b)) {
                    int c = splitPolygon(a, b);
                    a = filterPoints(a, nodes[a].next);
                    c = filterPoints(c, nodes[c].next);
                    earcutLinked(a); earcutLinked(c);
                    return;
                }
                b = nodes[b].next;
            }
            a = nodes[a].next;
        } while (a != start);
    }
    bool isValidDiagonal(int a, int b) const {
        return nodes[nodes[a].next].i != nodes[b].i &&
               nodes[nodes[a].prev].i != nodes[b].i &&
               !intersectsPolygon(a, b) && locallyInside(a, b) &&
               locallyInside(b, a) && middleInside(a, b);
    }
    bool intersectsPolygon(int a, int b) const {
        int p = a;
        do {
            if (nodes[p].i != nodes[a].i && nodes[nodes[p].next].i != nodes[a].i &&
                nodes[p].i != nodes[b].i && nodes[nodes[p].next].i != nodes[b].i &&
                intersects(p, nodes[p].next, a, b)) return true;
            p = nodes[p].next;
        } while (p != a);
        return false;
    }
    bool middleInside(int a, int b) const {
        int p = a; bool inside = false;
        double px = (nodes[a].x + nodes[b].x) / 2, py = (nodes[a].y + nodes[b].y) / 2;
        do {
            if (((nodes[p].y > py) != (nodes[nodes[p].next].y > py)) &&
                nodes[nodes[p].next].y != nodes[p].y &&
                (px < (nodes[nodes[p].next].x - nodes[p].x) * (py - nodes[p].y) /
                      (nodes[nodes[p].next].y - nodes[p].y) + nodes[p].x))
                inside = !inside;
            p = nodes[p].next;
        } while (p != a);
        return inside;
    }
    int splitPolygon(int a, int b) {
        int a2 = makeNode(nodes[a].i, nodes[a].x, nodes[a].y);
        int b2 = makeNode(nodes[b].i, nodes[b].x, nodes[b].y);
        int an = nodes[a].next, bp = nodes[b].prev;
        nodes[a].next = b; nodes[b].prev = a;
        nodes[a2].next = an; nodes[an].prev = a2;
        nodes[b2].next = a2; nodes[a2].prev = b2;
        nodes[bp].next = b2; nodes[b2].prev = bp;
        return b2;
    }
    void eliminateHoles(int& outerNode, std::vector<int>& holeNodes) {
        std::sort(holeNodes.begin(), holeNodes.end(),
                  [&](int a, int b) { return nodes[a].x < nodes[b].x; });
        for (int h : holeNodes) outerNode = eliminateHole(h, outerNode);
    }
    int getLeftmost(int start) {
        int p = start, left = start;
        do {
            if (nodes[p].x < nodes[left].x ||
                (nodes[p].x == nodes[left].x && nodes[p].y < nodes[left].y)) left = p;
            p = nodes[p].next;
        } while (p != start);
        return left;
    }
    int eliminateHole(int hole, int outerNode) {
        hole = getLeftmost(hole);
        int b = findHoleBridge(hole, outerNode);
        if (b == -1) return outerNode;   // give up on this hole (keeps it open)
        int m = splitPolygon(b, hole);
        filterPoints(m, nodes[m].next);
        filterPoints(b, nodes[b].next);
        return outerNode;
    }
    int findHoleBridge(int hole, int outerNode) {
        int p = outerNode; double hx = nodes[hole].x, hy = nodes[hole].y;
        double qx = -1e30; int m = -1;
        // Ray-cast to the left; find the outer edge just left of the hole point.
        do {
            int n2 = nodes[p].next;
            if (hy <= nodes[p].y && hy >= nodes[n2].y && nodes[n2].y != nodes[p].y) {
                double x = nodes[p].x + (hy - nodes[p].y) *
                           (nodes[n2].x - nodes[p].x) / (nodes[n2].y - nodes[p].y);
                if (x <= hx && x > qx) {
                    qx = x;
                    m = nodes[p].x < nodes[n2].x ? p : n2;
                    if (x == hx) return m;
                }
            }
            p = nodes[p].next;
        } while (p != outerNode);
        if (m == -1) return -1;
        // Refine against reflex verts inside the (hole, I, m) triangle (mapbox).
        int stop = m; double mx = nodes[m].x, my = nodes[m].y, tanMin = 1e30;
        p = m;
        do {
            double px = nodes[p].x, py = nodes[p].y;
            if (hx >= px && px >= mx && hx != px &&
                pointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, px, py)) {
                double tan = std::fabs(hy - py) / (hx - px);
                if (locallyInside(p, hole) && (tan < tanMin ||
                    (tan == tanMin && px > nodes[m].x))) { m = p; tanMin = tan; }
            }
            p = nodes[p].next;
        } while (p != stop);
        return m;
    }
};

}  // namespace

std::vector<std::array<Vec2, 3>>
triangulateWithHoles(const Poly2& outer, const std::vector<Poly2>& holes) {
    std::vector<std::array<Vec2, 3>> out;
    if (outer.size() < 3) return out;

    Earcut E;
    std::vector<Vec2> pts;
    std::vector<int> oring;
    for (const Vec2& v : outer) { oring.push_back(static_cast<int>(pts.size())); pts.push_back(v); }
    int outerNode = E.linkedList(oring, pts, /*clockwise=*/true);

    std::vector<int> holeStarts;
    for (const Poly2& h : holes) {
        if (h.size() < 3) continue;
        std::vector<int> ring;
        for (const Vec2& v : h) { ring.push_back(static_cast<int>(pts.size())); pts.push_back(v); }
        holeStarts.push_back(E.linkedList(ring, pts, /*clockwise=*/false));
    }
    if (!holeStarts.empty()) E.eliminateHoles(outerNode, holeStarts);
    E.earcutLinked(E.filterPoints(outerNode));

    out.reserve(E.tris.size());
    for (const std::array<int, 3>& t : E.tris)
        out.push_back({pts[t[0]], pts[t[1]], pts[t[2]]});
    return out;
}

}  // namespace engine
