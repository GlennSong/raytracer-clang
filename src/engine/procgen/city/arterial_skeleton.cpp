#include "arterial_skeleton.h"

#include "../../../log.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

std::uint32_t mix32(std::uint32_t x) {
    x += 0x9e3779b9u;
    x ^= x >> 16; x *= 0x21f0aaadu;
    x ^= x >> 15; x *= 0x735a2d97u;
    x ^= x >> 15;
    return x;
}
double hash01(std::uint32_t x) { return mix32(x) / 4294967296.0; }
std::uint32_t hashVec(std::uint32_t h, const Vec2& v) {
    h = mix32(h ^ static_cast<std::uint32_t>(std::llround(v.x * 8.0)));
    h = mix32(h ^ static_cast<std::uint32_t>(std::llround(v.y * 8.0)));
    return h;
}

// Open-polyline Chaikin (endpoints pinned) + even resample (endpoints exact).
std::vector<Vec2> chaikinOpen(const std::vector<Vec2>& pts, int passes) {
    std::vector<Vec2> cur = pts;
    for (int p = 0; p < passes && cur.size() >= 3; ++p) {
        std::vector<Vec2> next;
        next.reserve(cur.size() * 2);
        next.push_back(cur.front());
        for (std::size_t i = 0; i + 1 < cur.size(); ++i) {
            next.push_back(cur[i] * 0.75 + cur[i + 1] * 0.25);
            next.push_back(cur[i] * 0.25 + cur[i + 1] * 0.75);
        }
        next.push_back(cur.back());
        cur = std::move(next);
    }
    return cur;
}

std::vector<Vec2> resampleOpen(const std::vector<Vec2>& pts, double spacing) {
    if (pts.size() < 2) return pts;
    std::vector<double> cum(pts.size(), 0);
    for (std::size_t i = 1; i < pts.size(); ++i)
        cum[i] = cum[i - 1] + (pts[i] - pts[i - 1]).length();
    const double L = cum.back();
    const int m = std::max<int>(1, static_cast<int>(std::llround(L / spacing)));
    std::vector<Vec2> out;
    out.reserve(m + 1);
    out.push_back(pts.front());
    std::size_t edge = 0;
    for (int k = 1; k < m; ++k) {
        const double want = L * k / m;
        while (edge + 1 < cum.size() && cum[edge + 1] < want) ++edge;
        const double span = cum[edge + 1] - cum[edge];
        const double t = span > 1e-12 ? (want - cum[edge]) / span : 0.0;
        out.push_back(lerp(pts[edge], pts[edge + 1], t));
    }
    out.push_back(pts.back());
    return out;
}

// Glenn's line -> curve: recursive midpoint displacement, displacement
// biased toward buildable ground and (optionally) clamped inside a cell,
// endpoints pinned. Candidates try both signs at shrinking amplitude so the
// curve DODGES hostile ground where it can and stays a road where it can't.
void displaceSpan(std::vector<Vec2>& out, const Vec2& a, const Vec2& b,
                  int depth, double sway, std::uint32_t seed,
                  const std::function<bool(const Vec2&)>& inside,
                  const std::function<bool(double, double)>& buildable) {
    if (depth <= 0) {
        out.push_back(b);
        return;
    }
    const Vec2 mid = (a + b) * 0.5;
    const double len = (b - a).length();
    const Vec2 n = normalize(perp(b - a));
    const double amp =
        (hash01(hashVec(seed, mid)) * 2.0 - 1.0) * sway * len;
    Vec2 pick = mid;
    const double scales[7] = {1.0, -1.0, 0.5, -0.5, 0.25, -0.25, 0.0};
    for (double s : scales) {
        const Vec2 q = mid + n * (amp * s);
        if (inside && !inside(q)) continue;
        if (buildable && !buildable(q.x, q.y)) continue;
        pick = q;
        break;
    }
    if (inside && !inside(pick)) pick = mid;   // never leave the cell
    displaceSpan(out, a, pick, depth - 1, sway, mix32(seed), inside, buildable);
    displaceSpan(out, pick, b, depth - 1, sway, mix32(seed ^ 0x5bf03635u),
                 inside, buildable);
}

std::vector<Vec2> displacedRaw(
    const Vec2& a, const Vec2& b, int octaves, double sway,
    std::uint32_t seed, const std::function<bool(const Vec2&)>& inside,
    const std::function<bool(double, double)>& buildable) {
    std::vector<Vec2> raw;
    raw.push_back(a);
    displaceSpan(raw, a, b, octaves, sway, seed, inside, buildable);
    return raw;
}

std::vector<Vec2> displacedCurve(
    const Vec2& a, const Vec2& b, int octaves, double sway, double spacing,
    std::uint32_t seed, const std::function<bool(const Vec2&)>& inside,
    const std::function<bool(double, double)>& buildable) {
    return resampleOpen(
        chaikinOpen(displacedRaw(a, b, octaves, sway, seed, inside, buildable),
                    2),
        spacing);
}

void addChain(RoadGraph& Ga, const std::vector<Vec2>& pts, double width,
              RoadClass klass, bool closed = false) {
    if (pts.size() < 2) return;
    int prev = Ga.addNode(pts[0], 0.5);
    const int first = prev;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const int id = Ga.addNode(pts[i], 0.5);
        if (id != prev) Ga.addEdge(prev, id, width, klass);
        prev = id;
    }
    if (closed && prev != first) Ga.addEdge(prev, first, width, klass);
}

// A district cell during bisection: its boundary ring rides ON the emitted
// geometry (rim samples / cut samples), so every new cut terminates on a
// real road. Parallel flags mark junction vertices (gates, cut endpoints).
struct Cell {
    Poly2 ring;
    std::vector<char> junc;   // vertex is an emitted junction
    std::vector<char> gate;   // vertex is a rim gate (stronger snap pull)
    int depth = 0;
};

void makeCCW(Cell& c) {
    if (signedArea(c.ring) >= 0) return;
    std::reverse(c.ring.begin(), c.ring.end());
    std::reverse(c.junc.begin(), c.junc.end());
    std::reverse(c.gate.begin(), c.gate.end());
}

// Split `cell` along cutPts (front/back are EXISTING ring vertices ia/ib):
// child A = ring path ia..ib + reversed cut interior; child B = the rest.
void splitCell(const Cell& cell, int ia, int ib,
               const std::vector<Vec2>& cutPts, Cell& outA, Cell& outB) {
    const int n = static_cast<int>(cell.ring.size());
    auto walk = [&](int from, int to, Cell& dst) {
        for (int k = from;; k = (k + 1) % n) {
            dst.ring.push_back(cell.ring[k]);
            dst.junc.push_back(cell.junc[k]);
            dst.gate.push_back(cell.gate[k]);
            if (k == to) break;
        }
    };
    outA.depth = outB.depth = cell.depth + 1;
    walk(ia, ib, outA);
    for (int k = static_cast<int>(cutPts.size()) - 2; k >= 1; --k) {
        outA.ring.push_back(cutPts[k]);
        outA.junc.push_back(0);
        outA.gate.push_back(0);
    }
    walk(ib, ia, outB);
    for (std::size_t k = 1; k + 1 < cutPts.size(); ++k) {
        outB.ring.push_back(cutPts[k]);
        outB.junc.push_back(0);
        outB.gate.push_back(0);
    }
    makeCCW(outA);
    makeCCW(outB);
}

// Insert a vertex at `pos` on ring edge `edge` (between edge and edge+1).
// Returns the new index; `other` (another pending index) is shifted when it
// sits at/after the insertion point.
int insertRingVertex(Cell& cell, int edge, const Vec2& pos, int* other) {
    const int at = edge + 1;
    cell.ring.insert(cell.ring.begin() + at, pos);
    cell.junc.insert(cell.junc.begin() + at, 1);
    cell.gate.insert(cell.gate.begin() + at, 0);
    if (other && *other >= at) ++*other;
    return at;
}

}  // namespace

void emitConnectorRoad(RoadGraph& Ga, const Vec2& a, const Vec2& b,
                       double width, RoadClass klass,
                       const std::function<bool(double, double)>& buildable,
                       double sway, std::uint32_t seed) {
    addChain(Ga,
             displacedCurve(a, b, /*octaves=*/3, sway, /*spacing=*/60.0,
                            hashVec(hashVec(seed, a), b), nullptr, buildable),
             width, klass);
}

void buildFootprintSkeleton(RoadGraph& Ga, const Footprint& f, int siteIndex,
                            int hotspots, int kindBias,
                            const SkeletonParams& params,
                            const std::function<bool(double, double)>& buildable,
                            std::vector<CityHub>& hubsOut) {
    if (f.polygon.size() < 3) return;
    const double dl = std::max(300.0, params.districtLen);
    if (!params.rimRoad)
        LOG_WARN << "[skeleton] rim_road=false unsupported (P8-C v1): cuts "
                    "must terminate on roads — rim emitted anyway";

    // --- the root cell: footprint ring with gates as exact vertices ---------
    Cell root;
    {
        const std::size_t n = f.polygon.size();
        std::vector<double> cum(n + 1, 0);
        for (std::size_t i = 0; i < n; ++i)
            cum[i + 1] =
                cum[i] + (f.polygon[(i + 1) % n] - f.polygon[i]).length();
        std::size_t nextGate = 0;
        for (std::size_t i = 0; i < n; ++i) {
            root.ring.push_back(f.polygon[i]);
            bool isGate = false;
            // A gate landing within a meter of the vertex OWNS the vertex.
            while (nextGate < f.gates.size() &&
                   f.gates[nextGate].s < cum[i] + 1.0) {
                isGate = true;
                ++nextGate;
            }
            root.junc.push_back(isGate ? 1 : 0);
            root.gate.push_back(isGate ? 1 : 0);
            // Gates strictly inside this edge become their own vertices.
            while (nextGate < f.gates.size() &&
                   f.gates[nextGate].s < cum[i + 1] - 1.0) {
                root.ring.push_back(f.gates[nextGate].pos);
                root.junc.push_back(1);
                root.gate.push_back(1);
                ++nextGate;
            }
        }
        makeCCW(root);
    }
    addChain(Ga, root.ring, params.arteryWidth, RoadClass::Arterial,
             /*closed=*/true);

    const auto insideFootprint = [&](const Vec2& q) {
        return pointInPolygon(f.polygon, q);
    };
    int cutCounter = 0;
    auto cutSeed = [&]() {
        return mix32(params.seed * 2654435761u +
                     static_cast<std::uint32_t>(siteIndex) * 97u +
                     static_cast<std::uint32_t>(++cutCounter));
    };

    // GLOBAL junction registry: gates + every cut endpoint, across ALL cells.
    // Sibling cells share boundary chains but not ring bookkeeping — without
    // this, two branches plant feet a few hundred metres apart on the same
    // chain and only the consolidation backstop can fix it, by dragging
    // junctions (measured: 64-deg kinks). Snapping here is the constructive
    // fix; the backstop goes idle.
    struct Junction {
        Vec2 pos;
        bool gate;
    };
    std::vector<Junction> registry;
    for (const FootprintGate& gate : f.gates)
        registry.push_back({gate.pos, true});

    std::vector<Cell> queue;
    std::vector<Cell> leaves;

    // --- cut #1: the spine — founding road between the two most-opposite
    // gates, THROUGH the center (two displaced legs sharing the center).
    bool spineDone = false;
    if (params.spineRoad && f.gates.size() >= 2) {
        const double P = perimeter(f.polygon);
        int gi = -1, gj = -1;
        double best = -1;
        for (std::size_t i = 0; i < f.gates.size(); ++i)
            for (std::size_t j = i + 1; j < f.gates.size(); ++j) {
                const double ds =
                    std::fabs(f.gates[i].s - f.gates[j].s) * 2.0 * 3.14159265358979323846 / P;
                const double score =
                    (1.0 - std::cos(ds)) *
                    (f.gates[i].pos - f.gates[j].pos).length();
                if (score > best) {
                    best = score;
                    gi = static_cast<int>(i);
                    gj = static_cast<int>(j);
                }
            }
        // Ring indices of the two gates (exact vertices by construction).
        auto ringIndexOf = [&](const Vec2& p) {
            for (std::size_t k = 0; k < root.ring.size(); ++k)
                if ((root.ring[k] - p).length() < 1.0)
                    return static_cast<int>(k);
            return -1;
        };
        const int ia = ringIndexOf(f.gates[gi].pos);
        const int ib = ringIndexOf(f.gates[gj].pos);
        if (ia >= 0 && ib >= 0 && ia != ib) {
            // Two displaced legs through the center, smoothed as ONE
            // polyline — smoothing per leg would pin the center and mint a
            // kink there whenever the gates aren't perfectly opposite (the
            // center is an anchor, not a law; the road rounds through it).
            std::vector<Vec2> raw = displacedRaw(
                root.ring[ia], f.center, 2, params.sway, cutSeed(),
                insideFootprint, buildable);
            std::vector<Vec2> rawB = displacedRaw(
                f.center, root.ring[ib], 2, params.sway, cutSeed(),
                insideFootprint, buildable);
            raw.insert(raw.end(), rawB.begin() + 1, rawB.end());
            std::vector<Vec2> spine =
                resampleOpen(chaikinOpen(raw, 3), 55.0);
            addChain(Ga, spine, params.arteryWidth, RoadClass::Arterial);
            Cell a, b;
            splitCell(root, ia, ib, spine, a, b);
            queue.push_back(std::move(a));
            queue.push_back(std::move(b));
            spineDone = true;
        }
    }
    if (!spineDone) queue.push_back(std::move(root));

    // --- recursive bisection to district cells ------------------------------
    const double stopArea = 1.8 * dl * dl;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        Cell cell = std::move(queue[head]);   // FIFO: deterministic order
        const double cellArea = area(cell.ring);
        const OBB2 obb = orientedBoundingBox(cell.ring);
        const int longAx = obb.longAxis();
        const bool bisect = cellArea > stopArea &&
                            obb.half[1 - longAx] * 2.0 > 0.7 * dl &&
                            cell.depth < 12 && cell.ring.size() >= 8 &&
                            queue.size() + leaves.size() < 96;
        if (!bisect) {
            leaves.push_back(std::move(cell));
            continue;
        }

        // Cut chord: the line perpendicular to the long axis, stationed near
        // the midpoint (0.4..0.6 seeded — the fabricBisect character).
        const std::uint32_t seed = cutSeed();
        const double t = 0.4 + 0.2 * hash01(hashVec(seed, obb.center));
        const Vec2 c0 =
            obb.center + obb.axis[longAx] * ((t - 0.5) * 2.0 * obb.half[longAx]);
        const Vec2 dir = obb.axis[1 - longAx];

        // Ring crossings of the infinite line, parameterized along `dir`.
        struct Crossing {
            int edge;
            Vec2 pos;
            double u;
        };
        std::vector<Crossing> crossings;
        const int n = static_cast<int>(cell.ring.size());
        for (int k = 0; k < n; ++k) {
            const Vec2& p = cell.ring[k];
            const Vec2& q = cell.ring[(k + 1) % n];
            const double sp = cross(dir, p - c0), sq = cross(dir, q - c0);
            if ((sp > 0) == (sq > 0)) continue;
            const double tt = sp / (sp - sq);
            const Vec2 x = lerp(p, q, tt);
            crossings.push_back({k, x, dot(x - c0, dir)});
        }
        if (crossings.size() < 2) {
            leaves.push_back(std::move(cell));
            continue;
        }
        // The pair bracketing the OBB center along the cut direction.
        const Crossing* neg = nullptr;
        const Crossing* pos = nullptr;
        for (const Crossing& x : crossings) {
            if (x.u <= 0 && (!neg || x.u > neg->u)) neg = &x;
            if (x.u > 0 && (!pos || x.u < pos->u)) pos = &x;
        }
        if (!neg || !pos ||
            (pos->pos - neg->pos).length() < 0.6 * dl ||
            !pointInPolygon(cell.ring, (pos->pos + neg->pos) * 0.5)) {
            leaves.push_back(std::move(cell));
            continue;
        }

        // Endpoints: snap to an existing junction when one is close — gates
        // pull harder — so cuts CONTINUE through crossroads instead of
        // minting offset stubs and near-junction runts. The registry sees
        // junctions across sibling cells; a registry hit landing on this
        // cell's ring (shared chains) is mapped to a ring vertex or inserted
        // at its exact position.
        auto resolveEndpoint = [&](Vec2 want, int* other) {
            const double juncReach = params.continuationSnap * dl;
            const double gateReach = 2.0 * juncReach;
            const Junction* best = nullptr;
            double bestD = 1e30;
            for (const Junction& j : registry) {
                const double d = (j.pos - want).length();
                if (d < (j.gate ? gateReach : juncReach) && d < bestD) {
                    bestD = d;
                    best = &j;
                }
            }
            // Map the pick onto THIS ring: an existing vertex, or an exact
            // point on an edge (a sibling's foot on the shared chain). The
            // hosting edge is located by projection AT CALL TIME — earlier
            // inserts already shifted any precomputed edge index.
            for (int attempt = 0; attempt < 2; ++attempt) {
                const Vec2 target = (attempt == 0 && best) ? best->pos : want;
                if (attempt == 1 || best) {
                    for (int k = 0; k < static_cast<int>(cell.ring.size());
                         ++k)
                        if ((cell.ring[k] - target).length() < 1.0) {
                            cell.junc[k] = 1;
                            return k;
                        }
                    const int n2 = static_cast<int>(cell.ring.size());
                    int hostEdge = -1;
                    double hostD = 2.0;
                    for (int k = 0; k < n2; ++k) {
                        const Vec2& a = cell.ring[k];
                        const Vec2& b = cell.ring[(k + 1) % n2];
                        const Vec2 ab = b - a;
                        const double len2 = ab.lengthSquared();
                        if (len2 < 1e-12) continue;
                        const double t = std::clamp(
                            dot(target - a, ab) / len2, 0.0, 1.0);
                        const double d = (target - (a + ab * t)).length();
                        if (d < hostD) {
                            hostD = d;
                            hostEdge = k;
                        }
                    }
                    if (hostEdge >= 0) {
                        if (attempt == 1) registry.push_back({target, false});
                        return insertRingVertex(cell, hostEdge, target, other);
                    }
                }
                // Registry pick wasn't on this ring — use the raw crossing.
            }
            return -1;   // crossing lost its host edge (degenerate ring)
        };
        int ia = resolveEndpoint(neg->pos, nullptr);
        int ib = ia < 0 ? -1 : resolveEndpoint(pos->pos, &ia);
        if (ia < 0 || ib < 0) {
            leaves.push_back(std::move(cell));
            continue;
        }
        if (ia == ib) {
            leaves.push_back(std::move(cell));
            continue;
        }

        const auto insideCell = [&](const Vec2& q) {
            return pointInPolygon(cell.ring, q);
        };
        std::vector<Vec2> cut = displacedCurve(
            cell.ring[ia], cell.ring[ib], 2, params.sway, 55.0, seed,
            insideCell, buildable);
        addChain(Ga, cut, params.arteryWidth, RoadClass::Arterial);
        Cell a, b;
        splitCell(cell, ia, ib, cut, a, b);
        queue.push_back(std::move(a));
        queue.push_back(std::move(b));
    }

    // --- hubs: derived from the district cells (legacy kind cadence) --------
    const int kindCycle[6] = {1, 2, 4, 1, 2, 3};
    const int townCycle[2] = {2, 1};
    const bool city = siteIndex == 0;
    hubsOut.push_back(
        {f.center, city ? 0 : (kindBias >= 0 ? kindBias : 2), city, siteIndex});
    int made = 1;
    if (city) {
        for (std::size_t c = 0; c < leaves.size() && made < hotspots; ++c) {
            const Vec2 hub = centroid(leaves[c].ring);
            bool clash = false;
            for (const CityHub& h : hubsOut)
                if (h.site == siteIndex && (h.pos - hub).length() < 0.35 * dl)
                    clash = true;
            if (clash) continue;
            hubsOut.push_back({hub, kindCycle[(made - 1) % 6], made % 3 == 0,
                               siteIndex});
            ++made;
        }
    } else {
        for (std::size_t g = 0; g < f.gates.size() && made < hotspots; ++g) {
            hubsOut.push_back({lerp(f.gates[g].pos, f.center, 0.35),
                               townCycle[(made - 1) % 2], false, siteIndex});
            ++made;
        }
    }
    LOG_INFO << "[skeleton] site " << siteIndex << ": "
             << leaves.size() << " district cells, " << cutCounter
             << " cuts, " << made << " hubs";
}

}  // namespace engine
