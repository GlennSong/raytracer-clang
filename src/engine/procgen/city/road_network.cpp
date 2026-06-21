#include "road_network.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace engine {
namespace {

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x6c078965u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};

// Proper interior intersection of segments p1p2 and p3p4. Excludes shared/near
// endpoints (eps), so only true crossings split an edge. Returns t along p1p2.
bool segCross(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4,
              Vec2& out, Real& t) {
    Vec2 d1 = p2 - p1, d2 = p4 - p3;
    Real denom = cross(d1, d2);
    if (std::abs(denom) < 1e-9) return false;        // parallel/collinear
    Vec2 d13 = p3 - p1;
    t = cross(d13, d2) / denom;
    Real u = cross(d13, d1) / denom;
    const Real eps = 1e-6;
    if (t <= eps || t >= 1 - eps || u <= eps || u >= 1 - eps) return false;
    out = p1 + d1 * t;
    return true;
}

}  // namespace

int RoadGraph::addNode(const Vec2& p, Real tol) {
    Real tol2 = tol * tol;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if ((nodes[i].pos - p).lengthSquared() <= tol2) return static_cast<int>(i);
    nodes.push_back({p});
    return static_cast<int>(nodes.size() - 1);
}

void RoadGraph::addEdge(int a, int b, Real width, RoadClass klass) {
    if (a == b) return;
    for (const RoadEdge& e : edges)                  // no duplicate undirected edge
        if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) return;
    edges.push_back({a, b, width, klass});
}

Vec2 RoadArc::at(Real t) const {
    Real a = a0 + (a1 - a0) * t;
    return center + Vec2(std::cos(a), std::sin(a)) * radius;
}

void sampleArc(RoadGraph& g, int n0, int n1, const RoadArc& arc, Real width,
               RoadClass klass, Real maxErr, int minSegs) {
    Real sweep = std::abs(arc.a1 - arc.a0);
    int segs = std::max(1, minSegs);
    if (arc.radius > 1e-6 && maxErr > 0.0) {
        // Chord error e = R(1 - cos(dTheta/2)); pick the step so e <= maxErr.
        Real dThetaMax = 2.0 * std::acos(std::max(Real(0), Real(1) - maxErr / arc.radius));
        if (dThetaMax > 1e-6)
            segs = std::max(segs, static_cast<int>(std::ceil(sweep / dThetaMax)));
    }
    int prev = n0;
    for (int i = 1; i < segs; ++i) {
        int mid = g.addNode(arc.at(static_cast<Real>(i) / segs), 0.01);
        g.addEdge(prev, mid, width, klass);
        prev = mid;
    }
    g.addEdge(prev, n1, width, klass);
}

RoadGraph gridRoads(const GridRoadParams& p) {
    RoadGraph g;
    Rng rng(p.seed);
    int n = std::max(1, static_cast<int>(std::lround(p.extent * 2 / p.cellSize)));
    int dim = n + 1;
    Real step = p.extent * 2 / n;
    Vec2 o = p.center - Vec2(p.extent, p.extent);

    // Node grid with interior jitter.
    std::vector<std::vector<int>> id(dim, std::vector<int>(dim, -1));
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            Vec2 pos = o + Vec2(i * step, j * step);
            bool interior = (i > 0 && i < dim - 1 && j > 0 && j < dim - 1);
            if (interior) {
                Real jr = p.jitter * step;
                pos += Vec2(rng.range(-jr, jr), rng.range(-jr, jr));
            }
            id[j][i] = g.addNode(pos, 0.01);
        }

    auto classOf = [&](int line) -> RoadClass {
        if (line == 0 || line == dim - 1) return RoadClass::Arterial;
        return (line % 3 == 0) ? RoadClass::Collector : RoadClass::Local;
    };
    auto widthOf = [&](RoadClass c) {
        return c == RoadClass::Arterial ? p.arterialWidth
             : c == RoadClass::Collector ? p.collectorWidth : p.localWidth;
    };

    // Horizontal + vertical segments. Local streets may drop out, but border
    // arterials always stay (so the region stays enclosed).
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            if (i + 1 < dim) {
                RoadClass c = classOf(j);
                if (!(c == RoadClass::Local && rng.unit() < p.dropout))
                    g.addEdge(id[j][i], id[j][i + 1], widthOf(c), c);
            }
            if (j + 1 < dim) {
                RoadClass c = classOf(i);
                if (!(c == RoadClass::Local && rng.unit() < p.dropout))
                    g.addEdge(id[j][i], id[j + 1][i], widthOf(c), c);
            }
        }
    return g;
}

RoadGraph radialRoads(const RadialParams& p) {
    RoadGraph g;
    Rng rng(p.seed);
    int rings = std::max(1, static_cast<int>(std::lround(p.extent / p.ringSpacing)));
    int spokes = std::max(3, p.spokes);
    int subdiv = std::max(1, p.ringSubdiv);   // minimum chords per avenue gap

    auto widthOf = [&](RoadClass c) {
        return c == RoadClass::Arterial ? p.arterialWidth
             : c == RoadClass::Collector ? p.collectorWidth : p.localWidth;
    };

    // Avenue angles: evenly spaced, slightly angle-jittered but kept ON the circle
    // so the rings stay true arcs. One jitter per avenue, shared across rings, so
    // the avenues are straight radials. There is NO centre node — avenues meet the
    // inner roundabout (ring 1), and the disc inside it is an island.
    Real aj = p.jitter * (2 * PI / spokes) * 0.5;
    std::vector<Real> theta(spokes);
    for (int s = 0; s < spokes; ++s)
        theta[s] = 2 * PI * s / spokes + rng.range(-aj, aj);

    std::vector<std::vector<int>> sp(rings + 1);   // sp[ring][avenue] node on the circle
    for (int r = 1; r <= rings; ++r) {
        sp[r].resize(spokes);
        Real radius = r * p.ringSpacing;
        for (int s = 0; s < spokes; ++s)
            sp[r][s] = g.addNode(
                p.center + Vec2(std::cos(theta[s]), std::sin(theta[s])) * radius, 0.01);
    }

    // Ring roads: each avenue gap is a true ARC, sampled to a fine polyline (chord
    // error bounded), so the ring reads as a smooth circle and lots line the arc.
    // The roundabout (ring 1) and the outer ring are arterial.
    for (int r = 1; r <= rings; ++r) {
        RoadClass c = (r == 1 || r == rings) ? RoadClass::Arterial
                    : (r % 2 == 0) ? RoadClass::Collector : RoadClass::Local;
        Real radius = r * p.ringSpacing;
        for (int s = 0; s < spokes; ++s) {
            Real a1 = (s + 1 < spokes) ? theta[s + 1] : theta[0] + 2 * PI;
            RoadArc arc{p.center, radius, theta[s], a1};
            sampleArc(g, sp[r][s], sp[r][(s + 1) % spokes], arc, widthOf(c), c,
                      0.15, subdiv);
        }
    }
    // Avenues: straight, from the roundabout (ring 1) outward.
    for (int s = 0; s < spokes; ++s) {
        RoadClass c = (s % 2 == 0) ? RoadClass::Arterial : RoadClass::Collector;
        for (int r = 1; r < rings; ++r)
            g.addEdge(sp[r][s], sp[r + 1][s], widthOf(c), c);
    }
    return g;
}

// --- Tensor-field generator -------------------------------------------------
namespace {

// A symmetric, traceless 2nd-order tensor stored by its two free components:
// the matrix is [[a, b], [b, -a]]. The major eigenvector is at angle
// 0.5*atan2(b, a); the minor is perpendicular. Summing tensors is just summing
// (a, b) — that linearity is the whole point: basis fields blend cleanly.
struct Tensor { Real a = 0, b = 0; };

// Evaluate the field at q: a constant grid basis plus a radial singularity whose
// influence decays with distance, so orientation is radial in the core and grid
// at the rim. Encoding a direction theta as (cos 2theta, sin 2theta) is what
// makes a road's 180-degree ambiguity vanish, so opposing fields reinforce
// instead of cancelling.
Tensor fieldAt(const TensorRoadParams& p, const Vec2& q) {
    Tensor t;
    t.a += p.gridStrength * std::cos(2 * p.gridAngle);
    t.b += p.gridStrength * std::sin(2 * p.gridAngle);

    Vec2 v = q - p.center;
    Real r = v.length();
    if (r > 1e-3) {
        Real phi = std::atan2(v.y, v.x);          // major eigenvector = radial
        Real w = p.radialStrength *
                 std::exp(-(r * r) / (p.radialDecay * p.radialDecay));
        t.a += w * std::cos(2 * phi);
        t.b += w * std::sin(2 * phi);
    }
    return t;
}

// Unit major-eigenvector direction; zero where the field is degenerate (the two
// eigenvalues coincide and orientation is undefined — a singular point).
Vec2 majorDir(const Tensor& t) {
    Real mag = std::sqrt(t.a * t.a + t.b * t.b);
    if (mag < 1e-6) return Vec2(0, 0);
    Real ang = 0.5 * std::atan2(t.b, t.a);
    return Vec2(std::cos(ang), std::sin(ang));
}

// Uniform-grid point index for evenly-spaced streamlines: lets a candidate ask
// "is any committed streamline of my family within d?" in O(1). Cell == the
// separation distance, so the answer lives in the 3x3 neighbourhood.
struct PointHash {
    Real cell;
    std::unordered_map<long long, std::vector<Vec2>> buckets;
    explicit PointHash(Real c) : cell(c > 1e-3 ? c : 1.0) {}
    static long long key(int i, int j) {
        return (static_cast<long long>(i) << 32) ^ (static_cast<unsigned>(j));
    }
    void insert(const Vec2& p) {
        buckets[key(static_cast<int>(std::floor(p.x / cell)),
                    static_cast<int>(std::floor(p.y / cell)))].push_back(p);
    }
    bool nearAny(const Vec2& p, Real d) const {
        Real d2 = d * d;
        int ci = static_cast<int>(std::floor(p.x / cell));
        int cj = static_cast<int>(std::floor(p.y / cell));
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                auto it = buckets.find(key(ci + di, cj + dj));
                if (it == buckets.end()) continue;
                for (const Vec2& q : it->second)
                    if ((q - p).lengthSquared() <= d2) return true;
            }
        return false;
    }
};

// Integrate one streamline from `seed` along eigenvector family `fam` (0 = major,
// 1 = minor), in one direction (sign). A midpoint (RK2) step keeps curved rings
// smooth. The line runs its full span — stopping at the region edge, a
// degenerate point, when it closes back on its own seed (a ring), or when it
// bunches to within `dStop` of an ALREADY-traced line of the *same* family.
// That last test is the Jobard & Lefebvre separation rule, but kept tight: same
// family lines are parallel and only approach when crowding (e.g. radial spokes
// converging on the core), so a small `dStop` curbs near-duplicate slivers
// without truncating a line at the cross-family intersections that form blocks.
// Returns the ordered points walked.
std::vector<Vec2> integrate(const TensorRoadParams& p, const Vec2& seed, int fam,
                            Real sign, const PointHash& hash, Real dStop,
                            Real dClose, Real extent, int maxSteps) {
    std::vector<Vec2> pts;
    Vec2 cur = seed;
    Vec2 prevDir(0, 0);
    auto eigenAt = [&](const Vec2& q) {
        Vec2 d = majorDir(fieldAt(p, q));
        return fam == 0 ? d : perp(d);
    };
    auto inRegion = [&](const Vec2& q) {
        Vec2 d = q - p.center;
        return std::abs(d.x) <= extent && std::abs(d.y) <= extent;
    };
    for (int s = 0; s < maxSteps; ++s) {
        Vec2 dir = eigenAt(cur);
        if (dir.lengthSquared() < 1e-9) break;               // degenerate point
        // Eigenvectors have no sign; pick the one continuing our heading.
        if (prevDir.lengthSquared() > 1e-9) {
            if (dot(dir, prevDir) < 0) dir = -dir;
        } else {
            dir = dir * sign;
        }
        Vec2 mid = cur + dir * (p.step * 0.5);               // RK2 midpoint
        Vec2 dir2 = eigenAt(mid);
        if (dir2.lengthSquared() < 1e-9) break;
        if (dot(dir2, dir) < 0) dir2 = -dir2;
        Vec2 next = cur + dir2 * p.step;

        if (!inRegion(next)) break;
        // Closed orbit (ring): we've travelled a bit and come home.
        if (s > 4 && (next - seed).length() < dClose) { pts.push_back(seed); break; }
        // Bunched into an existing same-family line: stop (anti-sliver).
        if (s > 1 && hash.nearAny(next, dStop)) { pts.push_back(next); break; }
        pts.push_back(next);
        prevDir = normalize(next - cur);
        cur = next;
    }
    return pts;
}

// Trace an evenly-spaced set of streamlines of one family, growing from a seed
// queue and self-propagating: every committed sample spawns offset seeds a
// `spacing` to either side, so streamlines fill the region at a uniform pitch
// (Jobard & Lefebvre). Commits each line into the graph and the family hash.
void traceFamily(RoadGraph& g, const TensorRoadParams& p, int fam,
                 PointHash& hash, std::queue<Vec2>& seeds, Real width,
                 RoadClass klass) {
    const Real dReject = p.spacing * 0.7;    // reject a seed this close to a line
    const Real dStop   = p.spacing * 0.4;    // stop a line bunching this close
    const Real dClose  = p.spacing * 0.5;    // ring closes back on its seed
    const Real dSeed   = p.spacing;          // perpendicular offset for new seeds
    const Real snap    = std::max(Real(0.25), p.step * 0.4);
    const int  maxSteps = static_cast<int>(p.extent * 8 / p.step) + 16;
    int guard = 0, guardMax = 20000;

    while (!seeds.empty() && guard++ < guardMax) {
        Vec2 seed = seeds.front();
        seeds.pop();
        Vec2 d = seed - p.center;
        if (std::abs(d.x) > p.extent || std::abs(d.y) > p.extent) continue;
        if (hash.nearAny(seed, dReject)) continue;           // already covered here

        std::vector<Vec2> fwd =
            integrate(p, seed, fam, +1, hash, dStop, dClose, p.extent, maxSteps);
        std::vector<Vec2> bwd =
            integrate(p, seed, fam, -1, hash, dStop, dClose, p.extent, maxSteps);

        // Stitch backward (reversed) + seed + forward into one polyline.
        std::vector<Vec2> line;
        line.reserve(bwd.size() + fwd.size() + 1);
        for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) line.push_back(*it);
        line.push_back(seed);
        for (const Vec2& q : fwd) line.push_back(q);
        if (line.size() < 2) continue;

        // Commit to the graph and the separation hash.
        int prev = g.addNode(line[0], snap);
        hash.insert(line[0]);
        for (std::size_t i = 1; i < line.size(); ++i) {
            int cur = g.addNode(line[i], snap);
            g.addEdge(prev, cur, width, klass);
            hash.insert(line[i]);
            prev = cur;
        }

        // Self-propagate: drop seeds offset to each side along the local normal.
        for (std::size_t i = 0; i < line.size(); i += 2) {
            Vec2 t = (i + 1 < line.size()) ? line[i + 1] - line[i]
                   : (i > 0)               ? line[i] - line[i - 1]
                                           : Vec2(1, 0);
            Vec2 n = normalize(perp(t));
            if (n.lengthSquared() < 1e-9) continue;
            seeds.push(line[i] + n * dSeed);
            seeds.push(line[i] - n * dSeed);
        }
    }
}

}  // namespace

RoadGraph tensorRoads(const TensorRoadParams& p) {
    RoadGraph g;
    Rng rng(p.seed);

    // Major family = avenues/spokes; minor family = the cross streets and rings.
    PointHash majorHash(p.spacing), minorHash(p.spacing);
    std::queue<Vec2> majorSeeds, minorSeeds;

    // Seed the core (where the radial field is strongest) plus a coarse scatter
    // so disconnected pockets at the grid rim still get traced. A little jitter
    // keeps rim seeds off the field's symmetry lines.
    majorSeeds.push(p.center);
    minorSeeds.push(p.center);
    Real jr = p.spacing * 0.15;
    for (Real y = -p.extent; y <= p.extent; y += p.spacing * 1.5)
        for (Real x = -p.extent; x <= p.extent; x += p.spacing * 1.5) {
            Vec2 s = p.center + Vec2(x + rng.range(-jr, jr), y + rng.range(-jr, jr));
            majorSeeds.push(s);
            minorSeeds.push(s);
        }

    // Trace avenues first, then cross streets seeded onto them by the scatter.
    traceFamily(g, p, 0, majorHash, majorSeeds, p.collectorWidth, RoadClass::Collector);
    traceFamily(g, p, 1, minorHash, minorSeeds, p.localWidth, RoadClass::Local);
    return g;
}

RoadGraph planarize(const RoadGraph& in, Real tol) {
    RoadGraph out;
    // Re-key nodes into the output (dedup by tol).
    std::vector<int> remap(in.nodes.size());
    for (std::size_t i = 0; i < in.nodes.size(); ++i)
        remap[i] = out.addNode(in.nodes[i].pos, tol);

    for (const RoadEdge& e : in.edges) {
        Vec2 a = in.nodes[e.a].pos, b = in.nodes[e.b].pos;
        // Collect crossing points with every other edge.
        std::vector<std::pair<Real, Vec2>> cuts;   // (t, point)
        for (const RoadEdge& o : in.edges) {
            if (&o == &e) continue;
            Vec2 c = in.nodes[o.a].pos, d = in.nodes[o.b].pos;
            Vec2 x; Real t;
            if (segCross(a, b, c, d, x, t)) cuts.emplace_back(t, x);
        }
        std::sort(cuts.begin(), cuts.end(),
                  [](const auto& l, const auto& r) { return l.first < r.first; });
        // Chain: a -> cut0 -> cut1 -> ... -> b.
        int prev = remap[e.a];
        for (const auto& cut : cuts) {
            int mid = out.addNode(cut.second, tol);
            out.addEdge(prev, mid, e.width, e.klass);
            prev = mid;
        }
        out.addEdge(prev, remap[e.b], e.width, e.klass);
    }
    return out;
}

std::vector<Poly2> extractBlocks(const RoadGraph& g, Real minArea) {
    std::vector<Poly2> blocks;
    const int nNodes = static_cast<int>(g.nodes.size());
    if (nNodes < 3 || g.edges.empty()) return blocks;

    // Directed half-edges: each undirected edge -> two, twins adjacent (2k, 2k+1).
    struct Half { int tail, head, twin; Real angle; };
    std::vector<Half> he;
    he.reserve(g.edges.size() * 2);
    for (const RoadEdge& e : g.edges) {
        int i = static_cast<int>(he.size());
        Vec2 va = g.nodes[e.a].pos, vb = g.nodes[e.b].pos;
        he.push_back({e.a, e.b, i + 1, std::atan2(vb.y - va.y, vb.x - va.x)});
        he.push_back({e.b, e.a, i,     std::atan2(va.y - vb.y, va.x - vb.x)});
    }

    // Outgoing half-edges per node, sorted CCW by angle.
    std::vector<std::vector<int>> outAt(nNodes);
    for (int i = 0; i < static_cast<int>(he.size()); ++i) outAt[he[i].tail].push_back(i);
    for (auto& list : outAt)
        std::sort(list.begin(), list.end(),
                  [&](int x, int y) { return he[x].angle < he[y].angle; });

    // next(h): at head(h), take the half-edge clockwise-adjacent to the twin —
    // the previous entry in the CCW-sorted outgoing list. Traces interior faces
    // CCW (positive area); the unbounded outer face comes out CW (negative).
    auto nextHalf = [&](int h) {
        int v = he[h].head;
        int twin = he[h].twin;
        const std::vector<int>& list = outAt[v];
        int idx = 0;
        for (int k = 0; k < static_cast<int>(list.size()); ++k)
            if (list[k] == twin) { idx = k; break; }
        int deg = static_cast<int>(list.size());
        return list[(idx + deg - 1) % deg];
    };

    std::vector<char> used(he.size(), 0);
    for (int start = 0; start < static_cast<int>(he.size()); ++start) {
        if (used[start]) continue;
        Poly2 face;
        int h = start;
        int guard = 0;
        do {
            used[h] = 1;
            face.push_back(g.nodes[he[h].tail].pos);
            h = nextHalf(h);
            if (++guard > nNodes * 4 + 8) break;     // malformed: bail
        } while (h != start && !used[h]);

        if (face.size() < 3) continue;
        Real sa = signedArea(face);
        if (sa > 0 && sa >= minArea) blocks.push_back(std::move(face));
    }
    return blocks;
}

}  // namespace engine
