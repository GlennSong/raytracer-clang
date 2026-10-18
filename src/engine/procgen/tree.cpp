#include "tree.h"
#include "../mesh_builder.h"
#include "lsystem.h"
#include "skeleton.h"
#include "../../rt_math.h"
#include "../../curve.h"
#include "sdf.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <functional>
#include <unordered_map>
#include <map>
#include <random>
#include <sstream>

namespace engine {

namespace {

std::string num(float v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

// Build a parametric grammar from the params. The apex A(l) lays one internode
// F(l), spawns `branchesPerNode` side branches each rolled by the golden angle
// (so they spiral around the trunk in 3D, not in a plane), and continues a
// central leader A(l*falloff). Rolls are applied *outside* the brackets so the
// phyllotactic spin accumulates into the leader; the pitch '&' is inside, so
// only the branch bends away.
//
// With terminal ramification on, a second guarded production fires once the
// internode shrinks below terminalFraction*trunkLength: a denser spray of
// `terminalForks` short twigs (wider pitch, faster shrink), so branch ends fork
// repeatedly into many short tipped twigs — the crown density real trees have.
// Side branches start shorter, so they hit the threshold sooner and ramify
// earlier than the leader. Unexpanded apices after `iterations` are the tips.
ParametricLSystem buildGrammar(const TreeParams& p) {
    const std::string f = num(p.lengthFalloff);        // side branches
    const std::string lf = num(p.leaderFalloff);       // central leader (height)
    const std::string roll = num(p.phyllotaxis);
    const std::string pitch = num(p.branchAngle);

    auto structural = [&]() {
        std::string s = "F(l)";
        for (int i = 0; i < p.branchesPerNode; i++)
            s += "/(" + roll + ")[&(" + pitch + ")A(l*" + f + ")]";
        s += "/(" + roll + ")A(l*" + lf + ")";   // tall central leader continues
        return s;
    };

    ParametricLSystem g;
    if (p.terminalForks <= 0 || p.terminalFraction <= 0.0f) {
        g.rule("A(l)", structural());
        return g;
    }

    const std::string thresh = num(p.trunkLength * p.terminalFraction);
    g.rule("A(l):l>" + thresh, structural());

    // Terminal spray: more forks, wider pitch, shorter twigs (faster shrink so it
    // stays terminal and keeps forking for the remaining orders).
    const std::string ft = num(0.7f);
    const std::string pitchT = num(p.branchAngle * 1.3f);
    std::string term = "F(l)";
    for (int i = 0; i < p.terminalForks; i++)
        term += "/(" + roll + ")[&(" + pitchT + ")A(l*" + ft + ")]";
    term += "/(" + roll + ")A(l*" + ft + ")";
    g.rule("A(l):l<=" + thresh, term);
    return g;
}

// --- branch skeleton -------------------------------------------------------
// The turtle interpretation now lives in the shared, domain-agnostic
// skeleton.{h,cpp} (buildSkeleton -> Skeleton/SkeletonNode); the tree-specific
// steps below (pipe-model radii, droop, skinning, leaves) are consumers of it.

// Pipe model: a node carrying no children gets the tip radius; an internal node
// combines its children's cross-sections, r = (sum r_child^n)^(1/n). Nodes are
// in DFS pre-order, so a reverse pass sees every child before its parent.
void assignRadii(std::vector<SkeletonNode>& nodes, const TreeParams& p) {
    std::vector<float> sumPow(nodes.size(), 0.0f);
    const float n = p.pipeExponent;
    for (int i = static_cast<int>(nodes.size()) - 1; i >= 0; i--) {
        float r = nodes[i].childCount == 0
                      ? p.tipRadius
                      : std::pow(sumPow[i], 1.0f / n);
        r *= p.radiusScale;
        if (r < p.tipRadius * p.radiusScale) r = p.tipRadius * p.radiusScale;
        nodes[i].radius = r;
        if (nodes[i].parent >= 0)
            sumPow[nodes[i].parent] += std::pow(r / p.radiusScale, n);
    }
}

// Smooth 3D value noise in [0,1) (the gnarl's meander and the bark's irregular cross-section).
float gnarlHash(int x, int y, int z, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u ^ static_cast<uint32_t>(y) * 668265263u ^
                 static_cast<uint32_t>(z) * 2246822519u ^ seed * 3266489917u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (h ^ (h >> 16)) / 4294967296.0f;
}
float gnarlNoise(const Vec3& p, uint32_t seed) {
    const int xi = static_cast<int>(std::floor(p.x)), yi = static_cast<int>(std::floor(p.y)), zi = static_cast<int>(std::floor(p.z));
    const float fx = static_cast<float>(p.x - xi), fy = static_cast<float>(p.y - yi), fz = static_cast<float>(p.z - zi);
    const float sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy), sz = fz * fz * (3 - 2 * fz);
    auto L = [](float a, float b, float t) { return a + (b - a) * t; };
    float c[2][2];
    for (int j = 0; j < 2; ++j)
        for (int k = 0; k < 2; ++k)
            c[j][k] = L(gnarlHash(xi, yi + j, zi + k, seed), gnarlHash(xi + 1, yi + j, zi + k, seed), sx);
    return L(L(c[0][0], c[1][0], sy), L(c[0][1], c[1][1], sy), sz);
}

// Organic shaping: bend branches under gravity (droop) and add a lateral
// meander (wander), thin branches bending most. The key is that the bend
// *accumulates* down a limb — each apical segment bends relative to its parent's
// already-bent direction, so a straight rod of skeleton nodes curls into a
// cantilever arc (more sag toward the tip), not just a uniform tilt. At a fork a
// lateral child bends from its *own* direction, preserving the branch angle.
// Runs in parent-before-child order, so children inherit the bent positions and
// joints stay connected; the thick ~vertical trunk (thin~0) barely moves.
//
// THE GNARL's centreline terms ride the same pass: the thin ends turn back up to the light (light: a sagging limb's
// S), a limb now and then KINKS (crookChance: a lost leader, a lateral taking over -- the kink persists, since the
// next segment bends from this one's direction), and a smooth multi-octave MEANDER (coherent noise of position, so
// a limb wanders in curves, not per-node jitter).
void shapeBranches(std::vector<SkeletonNode>& nodes, const TreeParams& p, std::mt19937& rng, uint32_t seed) {
    if (nodes.empty() || (p.droop <= 0.0f && p.wander <= 0.0f && p.light <= 0.0f && p.crookChance <= 0.0f &&
                          p.meander <= 0.0f)) return;

    // Apical (straightest) child per node, from the original geometry.
    std::vector<int> cont(nodes.size(), -1);
    std::vector<double> best(nodes.size(), -2.0);
    for (size_t i = 1; i < nodes.size(); i++) {
        int par = nodes[i].parent;
        if (par < 0) continue;
        Vec3 inDir = nodes[par].parent >= 0
                         ? normalize(nodes[par].pos - nodes[nodes[par].parent].pos)
                         : nodes[par].heading;
        Vec3 d = nodes[i].pos - nodes[par].pos;
        double L = d.length();
        if (L < 1e-9) continue;
        double dp = dot(d * (1.0 / L), inDir);
        if (dp > best[par]) { best[par] = dp; cont[par] = static_cast<int>(i); }
    }

    std::vector<Vec3> orig(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++) orig[i] = nodes[i].pos;

    float trunkR = std::max(nodes[0].radius, p.tipRadius);
    std::uniform_real_distribution<float> jit(-1.0f, 1.0f);
    for (size_t i = 1; i < nodes.size(); i++) {
        int par = nodes[i].parent;
        if (par < 0) continue;
        Vec3 off = orig[i] - orig[par];
        float len = static_cast<float>(off.length());
        if (len < 1e-6f) { nodes[i].pos = nodes[par].pos; continue; }

        // Base direction: the parent's already-bent direction when this is the
        // apical continuation (bends accumulate into a curve), else the segment's
        // own direction at a fork (keep the branch angle).
        Vec3 base = off * (1.0f / len);
        if (cont[par] == static_cast<int>(i) && nodes[par].parent >= 0)
            base = normalize(nodes[par].pos - nodes[nodes[par].parent].pos);

        float thin = std::clamp(1.0f - nodes[i].radius / trunkR, 0.0f, 1.0f);
        if (p.crookChance > 0.0f && cont[par] == static_cast<int>(i)) {
            std::uniform_real_distribution<float> u01(0.0f, 1.0f);
            if (u01(rng) < p.crookChance) {   // a kink: the limb carries on from a new direction
                Vec3 r0 = std::abs(base.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                Vec3 s0 = normalize(cross(base, r0)), u0 = normalize(cross(s0, base));
                const double az = u01(rng) * 2.0 * PI, ang = p.crookAngle * (0.5 + u01(rng)) * PI / 180.0;
                base = normalize(base * std::cos(ang) + (s0 * std::cos(az) + u0 * std::sin(az)) * std::sin(ang));
            }
        }
        Vec3 bent = normalize(base + Vec3(0, -1, 0) * static_cast<double>(p.droop * thin) +
                              Vec3(0, 1, 0) * static_cast<double>(p.light * thin * thin));
        Vec3 ref = std::abs(bent.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        Vec3 side = normalize(cross(bent, ref));
        Vec3 up = normalize(cross(side, bent));
        Vec3 jitter = p.wander > 0.0f ? (side * jit(rng) + up * jit(rng)) * static_cast<double>(p.wander * thin) : Vec3(0, 0, 0);
        if (p.meander > 0.0f) {   // two octaves of coherent noise at the node's place: 3 m and 1 m wavelengths
            const Vec3 q = orig[i];
            const double m1 = gnarlNoise(q * 0.33, seed) - 0.5 + 0.5 * (gnarlNoise(q * 1.0 + Vec3(17, 3, 5), seed) - 0.5);
            const double m2 = gnarlNoise(q * 0.33 + Vec3(41, 7, 13), seed) - 0.5 + 0.5 * (gnarlNoise(q * 1.0 + Vec3(5, 29, 11), seed) - 0.5);
            jitter = jitter + (side * m1 + up * m2) * static_cast<double>(2.0f * p.meander);
        }
        nodes[i].pos = nodes[par].pos + normalize(bent + jitter) * len;
    }
}

// --- generalized-cylinder skinning ----------------------------------------

// A branch as a polyline of node samples: centerline points, per-point radius,
// and distance-from-root (for bark v). Skinned as one continuous swept tube.
struct BranchPath {
    std::vector<Vec3>  points;
    std::vector<float> radius;
    std::vector<float> dist;
};

// Linear-interpolate a per-knot attribute at spline parameter u in [0, segs].
float lerpAt(const std::vector<float>& a, double u) {
    if (a.empty()) return 0.0f;
    int segs = static_cast<int>(a.size()) - 1;
    if (segs <= 0) return a[0];
    double cu = std::clamp(u, 0.0, static_cast<double>(segs));
    int i = std::min(static_cast<int>(std::floor(cu)), segs - 1);
    float t = static_cast<float>(cu - i);
    return a[i] * (1.0f - t) + a[i + 1] * t;
}

// A SURFACE'S BARK MAPPING, kept to look up again (Glenn, 2026-10-18, of the limbs' seams: "we need to blend those
// textures better"): a limb welded on takes its PARENT's bark where it meets it. A tube's ring centres, frames, radii and
// bark v (addCurvedBranch fills one), or a trunk-zone stretch's ends; u round it as the surface itself has it.
struct BarkMap {
    std::vector<Vec3> c, right, up;
    std::vector<float> r, v;
    float uRep = 1.0f, uOff = 0.0f;
    Vec3 color{0.3, 0.25, 0.2};
    // the nearest point of the centre line: segment k, t along it
    void nearest(const Vec3& p, int& k, double& t) const {
        double best = 1e30;
        k = 0; t = 0.0;
        for (std::size_t i = 0; i + 1 < c.size(); ++i) {
            const Vec3 ab = c[i + 1] - c[i];
            const double tt = std::clamp(dot(p - c[i], ab) / std::max(1e-12, dot(ab, ab)), 0.0, 1.0);
            const double d = (p - (c[i] + ab * tt)).lengthSquared();
            if (d < best) { best = d; k = static_cast<int>(i); t = tt; }
        }
    }
    // how far p is out of the surface (the radial distance less the radius there)
    double dist(const Vec3& p) const {
        if (c.size() < 2) return 1e30;
        int k; double t;
        nearest(p, k, t);
        const Vec3 q = c[k] + (c[k + 1] - c[k]) * t;
        return (p - q).length() - (r[k] + (r[k + 1] - r[k]) * t);
    }
    Vec3 normal(const Vec3& p) const {
        int k; double t;
        nearest(p, k, t);
        const Vec3 q = p - (c[k] + (c[k + 1] - c[k]) * t);
        return q.length() > 1e-9 ? normalize(q) : Vec3(0, 1, 0);
    }
    void uv(const Vec3& p, float& u, float& vv) const {
        int k; double t;
        nearest(p, k, t);
        const Vec3 axis = normalize(c[k + 1] - c[k]);
        Vec3 R = right[k] * (1.0 - t) + right[k + 1] * t;
        R = R - axis * dot(R, axis);
        R = R.length() > 1e-9 ? normalize(R) : right[k];
        Vec3 U = up[k] * (1.0 - t) + up[k + 1] * t;
        U = U - axis * dot(U, axis) - R * dot(U, R);
        U = U.length() > 1e-9 ? normalize(U) : cross(axis, R);
        const Vec3 q = p - (c[k] + (c[k + 1] - c[k]) * t);
        double phi = std::atan2(dot(q, U), dot(q, R));
        if (phi < 0) phi += 2.0 * PI;
        u = uRep * static_cast<float>(phi / (2.0 * PI)) + uOff;
        vv = v[k] + (v[k + 1] - v[k]) * static_cast<float>(t);
    }
};

// BARK VARIETY: the bark's colour at `p` -- soft darker patches (barkMottle), a dark foot rising in tongues
// (barkFootDark) -- for every surface of bark (a limb's tube, the trunk zone)
Vec3 barkTint(const TreeParams& g, const Vec3& p, const Vec3& color, float footY, uint32_t seed) {
    if (g.barkMottle <= 0.0f && g.barkFootDark <= 0.0f) return color;
    float m = 1.0f;
    if (g.barkMottle > 0.0f) {   // patches with soft edges, two sizes
        const float n2 = 0.6f * gnarlNoise(p * 1.1, seed ^ 0x3b1du) + 0.4f * gnarlNoise(p * 3.7, seed ^ 0x51d7u);
        const float t = std::clamp((n2 - 0.52f) / 0.10f, 0.0f, 1.0f);
        m -= g.barkMottle * t * t * (3.0f - 2.0f * t);
    }
    Vec3 c = color * m;
    if (g.barkFootDark > 0.0f) {
        const float hy = static_cast<float>(p.y) - footY;
        // the dark base rises unevenly round the trunk, in tongues
        const float reach = g.barkFootHeight * (0.6f + 0.8f * gnarlNoise(Vec3(p.x * 3.0, 0.0, p.z * 3.0), seed ^ 0x77u));
        // (whole up to two fifths of its reach, fading out above)
        const float w = g.barkFootDark * std::clamp((reach - hy) / std::max(0.05f, 0.6f * reach), 0.0f, 1.0f);
        c = c * (1.0f - w) + g.barkFootColor * w;
    }
    return c;
}

// THE SOCKET a side branch is welded into (Glenn: "the tree trunks and branches were not seamless"): its parent's
// axis (through `c` along `d`) and radius `r` at the fork, and the collar's flare.
struct Socket {
    Vec3 c, d;
    float r = 0.0f;
    float collar = 0.0f;
};

// Skin a branch as a continuous generalized cylinder: fit a Catmull-Rom curve
// through the centerline, sample it at even arc length (so curvature shows),
// orient each ring with a rotation-minimizing frame (no twist), and sweep a
// tapered ring strip with bark UVs flowing along the length. Ring winding/normal
// layout matches MeshBuilder::cylinder.
//
// THE GNARL's cross-section (`g`, when given): the ring is not a circle -- the trunk (`trunk`) flutes into `lobes`
// ridges that fade up from the foot (`footY`) and swells into its root flare; every limb bulges at its nodes and
// pinches between, and its outline is knotty (barkNoise, coherent round the ring and along the limb); the lobes and
// the knots spiral with the grain (twist). The normals then come from the displaced surface, not the radial.
//
// THE WELD (`sock`, a side branch): the branch does not start at the fork's centre, buried in its parent with a hard
// crease where the two tubes cross -- it starts where its axis leaves the parent, its first ring projected back along
// it onto the parent's surface (the contact curve), and FLARES into it (a collar: the radius swells toward the
// parent, so the branch meets it at a shallow angle, a fillet); its normals there are the parent's, blending to its
// own along the collar, so the shading runs across the joint without a seam.
void addCurvedBranch(RenderMesh& mesh, const BranchPath& path, const Vec3& color,
                     int segs, float vScale, int ringsPerSegment,
                     const TreeParams* g = nullptr, bool trunk = false, float footY = 0.0f, uint32_t seed = 0,
                     const Socket* sock = nullptr, bool pointTip = false, BarkMap* map = nullptr) {
    if (path.points.size() < 2) return;
    Spline<Vec3> spline = Spline<Vec3>::catmullRom(path.points);
    int rings = std::max(2, spline.segments() * std::max(1, ringsPerSegment) + 1);

    std::vector<double> us = spline.uniformArcLengthParams(rings);
    if (g && g->ringAngle > 0.0f) {
        // RINGS BY NEED (TreeParams::ringAngle): from fine samples along the curve, a ring where the limb has turned by
        // ringAngle since the last, gone ringSpacing of its radii, or thinned by two fifths -- a straight run of limb is
        // one long band, a bend as many as it takes. (Near the start of a welded limb, its collar's flare: every
        // 0.7 radius for two radii -- a slanting limb's first band from its contact on the parent runs long.)
        const std::vector<double> fine = spline.uniformArcLengthParams(std::max(2, spline.segments() * 8 + 1));
        const double cosTurn = std::cos(static_cast<double>(g->ringAngle) * PI / 180.0);
        std::vector<double> keep{fine.front()};
        Vec3 tLast = normalize(spline.derivative(fine.front())), pPrev = spline.eval(fine.front());
        double sLast = 0.0, s = 0.0;
        float rLast = lerpAt(path.radius, fine.front());
        const float r0w = rLast;
        for (std::size_t i = 1; i < fine.size(); ++i) {
            const Vec3 p = spline.eval(fine[i]);
            s += (p - pPrev).length();
            pPrev = p;
            if (i + 1 == fine.size()) { keep.push_back(fine[i]); break; }
            const Vec3 t = normalize(spline.derivative(fine[i]));
            const float r = lerpAt(path.radius, fine[i]);
            const double gap = sock && s < 2.0 * r0w ? 0.7 * r0w : static_cast<double>(g->ringSpacing) * std::max(r, 0.5f * rLast);
            if (dot(t, tLast) < cosTurn || s - sLast >= gap || std::fabs(r - rLast) > std::max(0.4f * rLast, 0.008f)) {
                keep.push_back(fine[i]);
                tLast = t; sLast = s; rLast = r;
            }
        }
        us = keep;
        rings = static_cast<int>(us.size());
    }
    std::vector<Vec3> centers(rings), tangents(rings);
    for (int k = 0; k < rings; k++) {
        centers[k] = spline.eval(us[k]);
        tangents[k] = spline.derivative(us[k]);
    }
    std::vector<CurveFrame> frames = rotationMinimizingFrames(centers, tangents);

    // the socket: the first ring whose centre is out of the parent (k0), the ring before it the contact ring (k0 - 1)
    int k0 = -1;
    float r0 = 0.0f;
    std::vector<float> sFrom(static_cast<std::size_t>(rings), 0.0f);
    auto parentDist = [&](const Vec3& p) { const Vec3 q = p - sock->c; return static_cast<float>((q - sock->d * dot(q, sock->d)).length()); };
    if (sock && sock->collar > 0.0f && sock->r > 0.0f) {
        for (int k = 0; k < rings; k++)
            if (parentDist(centers[k]) >= sock->r) { k0 = k; break; }
        if (k0 < 1 || k0 > rings - 2) k0 = -1;   // (out at once, or never: no weld)
        if (k0 > 0) {
            r0 = lerpAt(path.radius, us[k0]);
            for (int k = k0; k < rings; k++) sFrom[static_cast<std::size_t>(k)] = static_cast<float>((centers[k] - centers[k0]).length());
        }
    }
    const bool shaped = g && (g->nodeSwell > 0.0f || g->barkNoise > 0.0f || (trunk && (g->flare > 0.0f || (g->lobes > 0 && g->lobeDepth > 0.0f))));
    const float twistRad = g ? g->twist * static_cast<float>(PI) / 180.0f : 0.0f;
    const float footR = path.radius.empty() ? 0.0f : path.radius.front();
    // the bark tile ROUND the limb a whole number of times by its girth (a tile is 1 / vScale metres, as along it):
    // not stretched round a trunk, not squeezed round a twig
    const float uRep = std::max(1.0f, std::round(2.0f * static_cast<float>(PI) * footR * vScale));
    // (each limb starts the bark tile at its own place, round and along: neighbouring limbs do not line up)
    const uint32_t tubeHash = static_cast<uint32_t>(std::lround(path.points.front().x * 1000.0)) * 73856093u ^
                              static_cast<uint32_t>(std::lround(path.points.front().y * 1000.0)) * 19349663u ^
                              static_cast<uint32_t>(std::lround(path.points.front().z * 1000.0)) * 83492791u ^ seed;
    const float uOff = static_cast<float>((tubeHash & 1023u) / 1024.0), vOff = static_cast<float>(((tubeHash >> 10) & 1023u) / 1024.0);
    if (map) {   // (its mapping, to look up again: every ring's centre, frame, radius and v)
        map->c = centers;
        map->right.clear(); map->up.clear(); map->r.clear(); map->v.clear();
        for (int k = 0; k < rings; ++k) {
            map->right.push_back(frames[k].right);
            map->up.push_back(frames[k].up);
            map->r.push_back(lerpAt(path.radius, us[k]));
            map->v.push_back(lerpAt(path.dist, us[k]) * vScale + vOff);
        }
        map->uRep = uRep;
        map->uOff = uOff;
        map->color = color;
    }
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int k = 0; k < rings; k++) {
        const Vec3 right = frames[k].right;
        const Vec3 up = frames[k].up;
        float rad = lerpAt(path.radius, us[k]);
        float v = lerpAt(path.dist, us[k]) * vScale;
        const float sAlong = lerpAt(path.dist, us[k]);
        float ringScale = 1.0f, lobeA = 0.0f;
        if (shaped) {
            const float hy = static_cast<float>(centers[k].y) - footY;
            if (trunk && g->flare > 0.0f) ringScale *= 1.0f + g->flare * std::exp(-std::max(0.0f, hy) / std::max(0.3f, 2.5f * footR));
            if (trunk && g->lobes > 0) lobeA = g->lobeDepth * std::exp(-std::max(0.0f, hy) / std::max(0.2f, g->lobeHeight));
            if (g->nodeSwell > 0.0f) {   // the knots of the path (its nodes) at whole u: a bulge there, a pinch between
                const double fu = us[k] - std::floor(us[k]), d = std::min(fu, 1.0 - fu);
                ringScale *= 1.0f + g->nodeSwell * static_cast<float>(std::exp(-(d / 0.14) * (d / 0.14)) - 0.35);
            }
        }
        for (int i = 0; i <= segs; i++) {
            float phi = 2.0f * static_cast<float>(PI) * i / segs;
            float c = std::cos(phi), s = std::sin(phi);
            Vec3 dir = right * c + up * s;        // outward radial = normal
            Vec3 tan = right * (-s) + up * c;     // around the circumference
            float r = rad;
            if (k0 > 0 && k >= k0)   // the collar: swelling into the parent over about a radius and a half
                r *= 1.0f + sock->collar * std::exp(-sFrom[static_cast<std::size_t>(k)] / std::max(1e-3f, 1.3f * r0));
            if (shaped) {
                float m = ringScale;
                const float ph = phi + twistRad * sAlong;
                if (lobeA > 0.0f) m *= 1.0f + lobeA * (0.5f + 0.5f * std::cos(g->lobes * ph + static_cast<float>(seed % 97)));
                if (g->barkNoise > 0.0f) {   // round the ring (periodic) and along the limb, at a scale set by its girth
                    const float k2 = 2.2f, kz = 1.4f / std::max(0.05f, rad * 4.0f);
                    const Vec3 q(std::cos(ph) * k2, std::sin(ph) * k2, sAlong * kz);
                    m *= 1.0f + g->barkNoise * (2.0f * gnarlNoise(q + Vec3(static_cast<double>(seed % 251), 0, 0), seed) - 1.0f);
                }
                r *= m;
            }
            Vertex vert(centers[k] + dir * r, dir, tan,
                        uRep * static_cast<float>(i) / segs + uOff, v + vOff);
            vert.color = g ? barkTint(*g, centers[k] + dir * r, color, footY, seed) : color;
            mesh.vertices.push_back(vert);
        }
    }
    const int stride = segs + 1;
    if (k0 > 0) {
        // the contact ring: ring k0 carried back along the branch onto the parent (a hair inside it, so the parent's
        // knots and lobes do not open a gap); the rings before it (inside the parent) dropped
        const Vec3 T = normalize(tangents[k0]);
        // (a hair inside the parent -- by as much as its knots and node pinches can dent it, or the contact ring stands
        // proud of it and the tube's open start shows: a slit of sky under a birch's limb)
        const float dent = g ? 1.3f * g->barkNoise + 0.4f * g->nodeSwell : 0.0f;
        const float R = sock->r * std::max(0.6f, 0.94f - dent);
        for (int i = 0; i <= segs; i++) {
            const Vec3 P = mesh.vertices[base + static_cast<uint32_t>(k0 * stride + i)].position;
            const Vec3 q = P - sock->c, a = cross(T, sock->d), b = cross(q, sock->d);
            const double A = dot(a, a), B = dot(a, b), C = dot(b, b) - double(R) * R;
            double t = 0.0;
            if (C > 0.0 && A > 1e-12) {
                const double disc = B * B - A * C;
                t = disc >= 0.0 ? (B - std::sqrt(disc)) / A : B / A;
                if (t < 0.0) t = 0.0;
            }
            Vertex& vx = mesh.vertices[base + static_cast<uint32_t>((k0 - 1) * stride + i)];
            vx.position = P - T * t;
        }
        mesh.vertices.erase(mesh.vertices.begin() + base, mesh.vertices.begin() + base + static_cast<std::ptrdiff_t>((k0 - 1) * stride));
        centers.erase(centers.begin(), centers.begin() + (k0 - 1));
        sFrom.erase(sFrom.begin(), sFrom.begin() + (k0 - 1));
        rings -= k0 - 1;
    }
    if (shaped || k0 > 0) {   // the normals of the displaced surface: across the ring x along the limb, turned outward
        auto P = [&](int k, int i) { return mesh.vertices[base + static_cast<uint32_t>(k * stride + i)].position; };
        std::vector<Vec3> nrm(static_cast<std::size_t>(rings * stride));
        for (int k = 0; k < rings; k++)
            for (int i = 0; i <= segs; i++) {
                const int ip = i == segs ? 1 : i + 1, im = i == 0 ? segs - 1 : i - 1;
                const Vec3 around = P(k, ip) - P(k, im);
                const Vec3 along = P(std::min(rings - 1, k + 1), i) - P(std::max(0, k - 1), i);
                Vec3 n = cross(around, along);
                const Vertex& vx = mesh.vertices[base + static_cast<uint32_t>(k * stride + i)];
                if (n.length() < 1e-12) n = vx.normal;
                n = normalize(n);
                if (dot(n, vx.normal) < 0) n = n * -1.0;
                nrm[static_cast<std::size_t>(k * stride + i)] = n;
            }
        for (int k = 0; k < rings; k++)
            for (int i = 0; i <= segs; i++) {
                Vertex& vx = mesh.vertices[base + static_cast<uint32_t>(k * stride + i)];
                Vec3 n = nrm[static_cast<std::size_t>(k * stride + i)];
                if (k0 > 0) {   // across the weld: the parent's own normal at the contact, the branch's along the collar
                    const float w = k == 0 ? 1.0f : std::exp(-sFrom[static_cast<std::size_t>(k)] / std::max(1e-3f, 0.8f * r0)) * 0.8f;
                    const Vec3 q = vx.position - sock->c;
                    const Vec3 radial = q - sock->d * dot(q, sock->d);
                    if (w > 0.01f && radial.length() > 1e-9) n = normalize(n * (1.0 - w) + normalize(radial) * w);
                }
                vx.normal = n;
                const Vec3 t = vx.tangent - vx.normal * dot(vx.tangent, vx.normal);
                if (t.length() > 1e-9) vx.tangent = normalize(t);
            }
    }
    if (pointTip && rings >= 2) {   // the end closed in a short point: one apex, a fan to the last ring
        const uint32_t last = base + static_cast<uint32_t>((rings - 1) * stride);
        Vec3 c(0, 0, 0);
        for (int i = 0; i < segs; ++i) c = c + mesh.vertices[last + i].position;
        c = c * (1.0 / segs);
        const Vec3 d = normalize(c - centers[rings - 2]);
        const float r = (mesh.vertices[last].position - c).length();
        Vertex apex(c + d * (2.5 * r), d, mesh.vertices[last].tangent, 0.5f * uRep, mesh.vertices[last].v + 2.5f * r * vScale);
        apex.color = mesh.vertices[last].color;
        mesh.vertices.push_back(apex);
        const uint32_t ap = static_cast<uint32_t>(mesh.vertices.size() - 1);
        for (int i = 0; i < segs; ++i) {
            const Vec3 mid = (mesh.vertices[last + i].position + mesh.vertices[last + i + 1].position) * 0.5;
            MeshBuilder::triFacing(mesh, last + static_cast<uint32_t>(i), last + static_cast<uint32_t>(i) + 1, ap, mid - c + d * (0.3 * r));
        }
    }
    for (int k = 0; k < rings - 1; k++) {
        for (int i = 0; i < segs; i++) {
            uint32_t a = base + k * stride + i;            // ring k, vert i
            uint32_t cc = a + 1;                           // ring k, vert i+1
            uint32_t b = base + (k + 1) * stride + i;      // ring k+1, vert i
            uint32_t d = b + 1;
            // wound so front faces point along the outward radial (the vertex normals)
            MeshBuilder::quadFacing(mesh, a, b, d, cc, mesh.vertices[a].normal + mesh.vertices[d].normal);
        }
    }
}

// --- leaf cards ------------------------------------------------------------

// One double-sided card from `at` along `forward` for `len`, `w` wide along
// `side`, shaded with `normal`.
void addLeafQuad(RenderMesh& mesh, const Vec3& at, const Vec3& forward,
                 const Vec3& side, const Vec3& normal, float w, float len,
                 const Vec3& color, int tiles = 1, int tile = 0) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 corners[4] = {at - side * w, at + side * w,
                       at + side * w + forward * len, at - side * w + forward * len};
    // (an atlas of tiles x tiles cards: this card's cell)
    const float ts = 1.0f / static_cast<float>(std::max(1, tiles));
    const float u0 = (tile % std::max(1, tiles)) * ts, v0 = (tile / std::max(1, tiles)) * ts;
    float uv[4][2] = {{u0, v0}, {u0 + ts, v0}, {u0 + ts, v0 + ts}, {u0, v0 + ts}};
    for (int i = 0; i < 4; i++) {
        Vertex v(corners[i], normal, side, uv[i][0], uv[i][1]);
        v.color = color;
        mesh.vertices.push_back(v);
    }
    MeshBuilder::quadFacing(mesh, base, base + 1, base + 2, base + 3, normal);           // double-sided:
    MeshBuilder::quadFacing(mesh, base, base + 1, base + 2, base + 3, normal * -1.0);    // both windings
}

// A leaf-cluster card: two perpendicular crossed quads, so a cluster has volume
// from any viewing angle instead of vanishing edge-on. The texture is a clump of
// many leaves, so one cross reads as a dense tuft.
void addLeaf(RenderMesh& mesh, const Vec3& at, const Vec3& dir, float size,
             const Vec3& color, int tiles = 1, int tile = 0) {
    Vec3 forward = normalize(dir);
    Vec3 ref = std::abs(forward.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 side = normalize(cross(forward, ref));
    Vec3 normal = normalize(cross(forward, side));
    float w = size * 0.5f, len = size * 1.7f;
    addLeafQuad(mesh, at, forward, side, normal, w, len, color, tiles, tile);
    addLeafQuad(mesh, at, forward, normal, side, w, len, color, tiles, tile);   // crossed 90°
}

// THE TRUNK ZONE (TreeParams::trunkZone; ADR-0183): the zone's limbs and the buttress roots as ONE implicit surface.
//   the FIELD: each limb (zone node to zone node, and its buttress roots) a tapered capsule -- the trunk's flaring and
//     fluted toward the foot (flare, lobes, spiralling with twist) -- smooth-unioned thickest first, each blended in by a
//     fillet 0.3 of its radius; a limb LEAVING the zone ends in its node's round cap at 0.92 of its radius, so the tube
//     that carries it on (built from the node) comes out of it; the bark's knots (barkNoise) displace the union at the
//     scale of the nearest limb
//   the MESH: Surface Nets at trunkZoneCell (sparse: near the surface only), simplified to trunkZoneTris, every vertex
//     settled back onto the field and its normal the field's
//   the BARK: each vertex mapped by its NEAREST limb -- round it (a frame carried from the foot limb to limb, so a limb's
//     bark carries on round into the next), along it (the distance from the foot, as the tubes' v) -- each triangle in
//     one limb's mapping (a vertex shared across two is duplicated: the bark meets along the crotch, as a tree's own
//     branch-bark ridge does); darker in a CROTCH (two limbs at a real angle both near)
void buildTrunkZone(TreeMesh& out, const std::vector<SkeletonNode>& nodes, const std::vector<std::vector<int>>& children,
                    const std::vector<char>& inZone, const TreeParams& params, uint32_t seed, float footY,
                    const std::function<BranchPath(int, std::mt19937&, Vec3&)>& rootPath, std::vector<BarkMap>* maps,
                    const std::function<bool(int)>& stubbed) {
    struct Piece {
        Vec3 a, b;
        float ra = 0, rb = 0;
        bool trunk = false;          // (the foot's limbs: flare and lobes)
        float tallA = 1, tallB = 1;  // a buttress root's height over its width at each end (it stands up off the trunk as a fin)
        int chain = 0;               // the limb it is a stretch of: a limb's stretches meet hard (no fillet bulging each node)
        float distA = 0, distB = 0;  // distance from the foot at each end (the bark's v)
        Vec3 right, up, axis;        // its frame (right x up = axis)
        Vec3 rightA, rightB;         // the frame carried to each end node: round the limb, the bark's angle is measured from
                                     // between them, so two stretches of a limb agree where they meet
        float uRep = 1;              // the bark tile round it
        Vec3 color;
        Vec3 lo, hi;                 // its box, padded by its widest
    };
    std::vector<Piece> pieces;
    const Vec3 foot = nodes[0].pos;
    const float footR = nodes[0].radius;
    const float vScale = params.barkVScale;
    // frames carried from the foot: each node's, along the limb into it
    std::vector<Vec3> nodeRight(nodes.size(), Vec3(1, 0, 0)), nodeAxis(nodes.size(), Vec3(0, 1, 0));
    std::mt19937 shadeRng(seed ^ 0x2b07e5u);
    std::uniform_real_distribution<float> shade(0.85f, 1.0f);
    auto carry = [](const Vec3& right, const Vec3& from, const Vec3& to) {   // minimal rotation of `right` from -> to
        Vec3 r = right - to * dot(right, to);
        if (r.length() < 1e-6) r = cross(to, std::fabs(to.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0));
        (void)from;
        return normalize(r);
    };
    const float trunkMul = 1.0f + std::max(0.0f, params.flare) + std::max(0.0f, params.lobeDepth);
    auto finish = [&](Piece& pc) {
        pc.axis = normalize(pc.b - pc.a);
        pc.up = normalize(cross(pc.axis, pc.right));   // (right x up = axis)
        pc.right = normalize(cross(pc.up, pc.axis));
        pc.uRep = std::max(1.0f, std::round(2.0f * static_cast<float>(PI) * pc.ra * vScale));
        const float pad = std::max(pc.ra, pc.rb) * (pc.trunk ? trunkMul : 1.0f) * std::max(pc.tallA, pc.tallB) * (1.0f + params.barkNoise) + 0.05f;
        pc.lo = Vec3(std::min(pc.a.x, pc.b.x) - pad, std::min(pc.a.y, pc.b.y) - pad, std::min(pc.a.z, pc.b.z) - pad);
        pc.hi = Vec3(std::max(pc.a.x, pc.b.x) + pad, std::max(pc.a.y, pc.b.y) + pad, std::max(pc.a.z, pc.b.z) + pad);
    };
    std::map<int, Vec3> chainShade;
    // the limbs: a node carries on its parent's limb if it is the parent's child most in line with the limb (thick ones)
    std::vector<int> chainOf(nodes.size(), -1);
    int chains = 0;
    std::vector<int> lineChild(nodes.size(), -1);
    for (size_t n = 0; n < nodes.size(); ++n) {
        if (!inZone[n]) continue;
        const Vec3 in = nodes[n].parent >= 0 ? normalize(nodes[n].pos - nodes[nodes[n].parent].pos) : Vec3(0, 1, 0);
        double best = -2.0;
        for (int c : children[n]) {
            if (!inZone[c]) continue;
            const Vec3 d = nodes[c].pos - nodes[n].pos;
            if (d.length() < 1e-9) continue;
            const double al = dot(normalize(d), in) + 0.5 * nodes[c].radius / std::max(1e-6f, nodes[n].radius);
            if (al > best) { best = al; lineChild[n] = c; }
        }
    }
    for (size_t n = 1; n < nodes.size(); ++n) {
        const int par = nodes[n].parent;
        if (par < 0 || !inZone[par]) continue;
        chainOf[n] = lineChild[par] == static_cast<int>(n) && chainOf[par] >= 0 ? chainOf[par] : chains++;
        if (par == 0 && lineChild[0] == static_cast<int>(n)) chainOf[n] = chainOf[0] = chainOf[0] >= 0 ? chainOf[0] : chainOf[n];
        const Vec3 d = nodes[n].pos - nodes[par].pos;
        if (d.length() < 1e-6) continue;
        const Vec3 ax = normalize(d);
        nodeAxis[n] = ax;
        nodeRight[n] = carry(par == 0 ? Vec3(1, 0, 0) : nodeRight[par], par == 0 ? Vec3(0, 1, 0) : nodeAxis[par], ax);
        if (!inZone[n]) continue;   // (a limb leaving: its tube, from the node's cap)
        Piece pc;
        pc.a = nodes[par].pos; pc.b = nodes[n].pos;
        pc.ra = nodes[par].radius; pc.rb = nodes[n].radius;
        pc.trunk = par == 0 || nodes[par].pos.y - footY < 2.0f * params.lobeHeight;
        pc.distA = nodes[par].distFromRoot; pc.distB = nodes[n].distFromRoot;
        pc.right = nodeRight[n];
        pc.rightA = par == 0 ? carry(Vec3(1, 0, 0), Vec3(0, 1, 0), ax) : nodeRight[par];
        pc.rightB = nodeRight[n];
        pc.chain = chainOf[n];
        // (one shade a limb, as a tube has: per stretch, the bark stepped in brightness at every node)
        auto cs = chainShade.find(pc.chain);
        if (cs == chainShade.end()) cs = chainShade.emplace(pc.chain, params.barkColor * shade(shadeRng)).first;
        pc.color = cs->second;
        finish(pc);
        pieces.push_back(pc);
    }
    // THE STUBS: every limb leaving the zone (a tube from here on) starts as a short cone in it, from the node out a
    // little past the parent's surface, a shade wider than the limb and tapering under it -- the zone fillets it like
    // any limb, and the tube comes out of the cone at a shallow angle (Glenn, 2026-10-18: "a hole where the limb
    // attaches to the main trunk" -- a collar's contact ring, projected onto a straight cylinder the zone is not,
    // stood off it as a strip with sky behind)
    for (size_t z = 0; z < nodes.size(); ++z) {
        if (!inZone[z]) continue;
        for (int c : children[z]) {
            if (inZone[c] || !stubbed(c)) continue;
            const Vec3 d = nodes[c].pos - nodes[z].pos;
            const double L = d.length();
            if (L < 1e-6) continue;
            const Vec3 ax = d * (1.0 / L);
            const float rc = nodes[c].radius;
            Piece pc;
            pc.a = nodes[z].pos;
            pc.b = nodes[z].pos + ax * std::min(0.8 * L, static_cast<double>(nodes[z].radius) + 1.2 * rc);
            pc.ra = 1.08f * rc; pc.rb = 0.88f * rc;
            pc.distA = nodes[z].distFromRoot; pc.distB = nodes[z].distFromRoot + static_cast<float>((pc.b - pc.a).length());
            pc.right = carry(nodeRight[z], nodeAxis[z], ax);
            pc.rightA = pc.rightB = pc.right;
            pc.chain = chains++;
            auto cs = chainShade.find(chainOf[z]);
            pc.color = cs != chainShade.end() ? cs->second : params.barkColor;
            finish(pc);
            pieces.push_back(pc);
        }
    }
    out.zoneLimbs = static_cast<int>(pieces.size());
    // THE ROOTS (Glenn, 2026-10-18: "the roots look very tepid. There should be bigger roots so we can sink the tree
    // into the ground"): each a FLARE off the trunk -- as thick as the trunk's flared foot where it leaves it, a broad
    // hump (wider than high) -- that runs out at least six trunk radii (rootSpread) along the ground, thinning and
    // sinking, and DIVES under it at its end, with a smaller one between each two: the trunk stands IN the ground, held
    // by them. Its azimuth and reach the
    // tube roots' own draws (a tree grown both ways roots the same way).
    std::mt19937 rootRng(seed ^ 0x700757u);
    const Vec3 up(0, 1, 0);
    const int mainRoots = params.rootCount;
    for (int k = 0; k < 2 * mainRoots; ++k) {
        // (the main roots, then a smaller one between each two: 0.55 of the size, half way round)
        const bool minor = k >= mainRoots;
        Vec3 color;
        const BranchPath tube = rootPath(minor ? k - mainRoots : k, rootRng, color);
        Vec3 dir = tube.points.back() - foot;
        if (minor) {   // half way to the next main root's bearing
            const double a = std::atan2(dir.z, dir.x) + PI / std::max(1, mainRoots);
            dir = Vec3(std::cos(a), 0.0, std::sin(a)) * dir.length();
        }
        const double size = minor ? 0.55 : 1.0;
        dir.y = 0.0;
        dir = dir.length() > 1e-6 ? normalize(dir) : Vec3(1, 0, 0);
        const Vec3 side = normalize(cross(up, dir));
        const double reach = size * std::max(6.0 * footR, static_cast<double>((tube.points.back() - foot).length()) * 1.4);
        std::uniform_real_distribution<double> wob(-0.15, 0.15);
        // (along it: out, up off the ground, radius over the trunk's, height over width)
        // (sized from the trunk's flared foot, not its bare radius: beside the flare a root of the bare one is a spike)
        const double foot2 = footR * (1.0 + 0.5 * std::max(0.0f, params.flare));
        const double prof[6][4] = {{0.10, 0.85, 1.00, 1.15}, {0.26, 0.40, 0.80, 1.0}, {0.44, 0.14, 0.58, 0.85},
                                   {0.63, 0.02, 0.40, 0.8}, {0.82, -0.10, 0.26, 0.8}, {1.00, -0.40, 0.15, 0.8}};
        BranchPath rp;
        std::vector<float> tall;
        double lateral = 0.0;
        for (int q = 0; q < 6; ++q) {
            const double out = prof[q][0] * reach;
            if (q >= 2) lateral += wob(rootRng) * reach * 0.2;   // (a root wanders a little)
            const Vec3 p = foot + dir * std::max(out, static_cast<double>(footR) * (q == 0 ? 0.2 : 1.0)) + side * lateral +
                           up * (prof[q][1] * foot2 * (q < 2 ? 1.0 : size));
            rp.points.push_back(p);
            rp.radius.push_back(static_cast<float>(prof[q][2] * foot2 * (q == 0 ? 1.0 : size)));
            rp.dist.push_back(static_cast<float>((p - foot).length()));
            tall.push_back(static_cast<float>(prof[q][3]));
        }
        out.limbs.push_back({foot, rp.points.front(), footR, rp.radius.front()});
        for (std::size_t q = 0; q + 1 < rp.points.size(); ++q) {
            out.limbs.push_back({rp.points[q], rp.points[q + 1], rp.radius[q], rp.radius[q + 1]});
            Piece pc;
            pc.a = rp.points[q]; pc.b = rp.points[q + 1];
            const float minR = 1.0f * params.trunkZoneCell;
            pc.ra = std::max(minR, rp.radius[q]); pc.rb = std::max(minR, rp.radius[q + 1]);
            pc.tallA = tall[q]; pc.tallB = tall[q + 1];
            pc.chain = chains + k;
            pc.distA = rp.dist[q]; pc.distB = rp.dist[q + 1];
            pc.right = up;
            pc.rightA = pc.rightB = up;
            pc.color = color;
            finish(pc);
            pieces.push_back(pc);
        }
    }
    if (pieces.empty()) return;
    // (one bark tile count round a limb, from its thickest: per stretch, the bark jumped where the limb tapered)
    {
        std::map<int, float> rep;
        for (const Piece& pc : pieces) rep[pc.chain] = std::max(rep[pc.chain], pc.uRep);
        for (Piece& pc : pieces) pc.uRep = rep[pc.chain];
    }
    // the limbs thickest first, each blended into those before it by a fillet 0.3 of its radius
    const int chainCount = chains + 2 * params.rootCount;
    std::vector<double> chainR(static_cast<std::size_t>(chainCount), 0.0), chainK(static_cast<std::size_t>(chainCount), 0.01);
    for (const Piece& pc : pieces) chainR[static_cast<std::size_t>(pc.chain)] = std::max(chainR[static_cast<std::size_t>(pc.chain)], static_cast<double>(std::max(pc.ra, pc.rb)));
    std::vector<int> chainOrder;
    for (int c = 0; c < chainCount; ++c)
        if (chainR[static_cast<std::size_t>(c)] > 0.0) { chainOrder.push_back(c); chainK[static_cast<std::size_t>(c)] = 0.3 * chainR[static_cast<std::size_t>(c)]; }
    std::stable_sort(chainOrder.begin(), chainOrder.end(), [&](int x, int y) { return chainR[static_cast<std::size_t>(x)] > chainR[static_cast<std::size_t>(y)]; });
    thread_local std::vector<double> chainScratch;
    chainScratch.assign(static_cast<std::size_t>(chainCount), 1e30);
    const float twistRad = params.twist * static_cast<float>(PI) / 180.0f;
    // a piece's own distance at p (and the t along it)
    auto pieceDist = [&](const Piece& pc, const Vec3& p, double& t) {
        const Vec3 ab = pc.b - pc.a;
        t = std::clamp(dot(p - pc.a, ab) / std::max(1e-12, dot(ab, ab)), 0.0, 1.0);
        const Vec3 c = pc.a + ab * t;
        double r = pc.ra + (pc.rb - pc.ra) * t;
        const double tall = pc.tallA + (pc.tallB - pc.tallA) * t;
        if (tall != 1.0) {   // (a root's section: `tall` times as high as wide -- scaled back so it never says more than the
                             // true distance, which the sparse mesher's skipping relies on)
            const Vec3 q = p - c;
            return (std::sqrt(q.x * q.x + q.z * q.z + (q.y / tall) * (q.y / tall)) - r) * std::min(1.0, tall);
        }
        if (pc.trunk) {
            const double hy = std::max(0.0, p.y - footY);
            r *= 1.0 + params.flare * std::exp(-hy / std::max(0.3, 2.5 * footR));
            if (params.lobes > 0 && params.lobeDepth > 0.0f) {
                const double phi = std::atan2(p.z - foot.z, p.x - foot.x) + twistRad * hy;
                const double lobeA = params.lobeDepth * std::exp(-hy / std::max(0.2f, params.lobeHeight));
                r *= 1.0 + lobeA * (0.5 + 0.5 * std::cos(params.lobes * phi + static_cast<double>(seed % 97)));
            }
        }
        return (p - c).length() - r;
    };
    // the field (and the nearest limb, and how near the next limb at an angle to it is: the crotch)
    struct Near { int i = -1, j = -1; double d1 = 1e30, d2 = 1e30; };
    SdfBounds bounds{Vec3(1e30, 1e30, 1e30), Vec3(-1e30, -1e30, -1e30)};
    for (const Piece& pc : pieces) {
        bounds.min = Vec3(std::min(bounds.min.x, pc.lo.x), std::min(bounds.min.y, pc.lo.y), std::min(bounds.min.z, pc.lo.z));
        bounds.max = Vec3(std::max(bounds.max.x, pc.hi.x), std::max(bounds.max.y, pc.hi.y), std::max(bounds.max.z, pc.hi.z));
    }
    // THE PIECES NEAR A POINT, by a coarse grid (the field is the mesher's cost: every sample looked at every limb and
    // root): each cell lists the pieces whose boxes, padded by their fillet and 0.6 m, reach it -- near the surface the
    // field is exact; far from it a sample may miss a piece, but a point INSIDE a piece is inside its box, so the side
    // it is on is always right
    const double gridCell = 0.25;
    const int gx = std::max(1, static_cast<int>(std::ceil((bounds.max.x - bounds.min.x) / gridCell))),
              gy = std::max(1, static_cast<int>(std::ceil((bounds.max.y - bounds.min.y) / gridCell))),
              gz = std::max(1, static_cast<int>(std::ceil((bounds.max.z - bounds.min.z) / gridCell)));
    std::vector<std::vector<int>> grid(static_cast<std::size_t>(gx) * gy * gz);
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        const Piece& pc = pieces[i];
        const double pad = chainK[static_cast<std::size_t>(pc.chain)] + 0.6;
        const int x0 = std::max(0, static_cast<int>((pc.lo.x - pad - bounds.min.x) / gridCell)), x1 = std::min(gx - 1, static_cast<int>((pc.hi.x + pad - bounds.min.x) / gridCell));
        const int y0 = std::max(0, static_cast<int>((pc.lo.y - pad - bounds.min.y) / gridCell)), y1 = std::min(gy - 1, static_cast<int>((pc.hi.y + pad - bounds.min.y) / gridCell));
        const int z0 = std::max(0, static_cast<int>((pc.lo.z - pad - bounds.min.z) / gridCell)), z1 = std::min(gz - 1, static_cast<int>((pc.hi.z + pad - bounds.min.z) / gridCell));
        for (int z = z0; z <= z1; ++z)
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) grid[(static_cast<std::size_t>(z) * gy + y) * gx + x].push_back(static_cast<int>(i));
    }
    std::vector<int> allPieces(pieces.size());
    for (std::size_t i = 0; i < pieces.size(); ++i) allPieces[i] = static_cast<int>(i);
    auto field = [&](const Vec3& p, Near* nr) {
        Near best;
        // each limb's distance (its stretches' least: they meet hard), then the limbs smooth-unioned, thickest first
        const std::vector<int>* list = &allPieces;
        if (!nr) {
            const int x = static_cast<int>((p.x - bounds.min.x) / gridCell), y = static_cast<int>((p.y - bounds.min.y) / gridCell),
                      z = static_cast<int>((p.z - bounds.min.z) / gridCell);
            if (x < 0 || y < 0 || z < 0 || x >= gx || y >= gy || z >= gz) return 1.0;   // (out of every piece's reach)
            list = &grid[(static_cast<std::size_t>(z) * gy + y) * gx + x];
            if (list->empty()) return 1.0;
        }
        std::vector<double>& per = chainScratch;
        std::fill(per.begin(), per.end(), 1e30);
        double dmin = 1e30, rN = footR;   // (the nearest piece's radius: the knots' scale)
        for (int i : *list) {
            const Piece& pc = pieces[static_cast<std::size_t>(i)];
            double t;
            const double d = pieceDist(pc, p, t);
            per[static_cast<std::size_t>(pc.chain)] = std::min(per[static_cast<std::size_t>(pc.chain)], d);
            if (d < dmin) { dmin = d; rN = pc.ra + (pc.rb - pc.ra) * t; }
            if (nr) {
                if (d < best.d1) {
                    if (best.i >= 0 && std::fabs(dot(pieces[best.i].axis, pc.axis)) < 0.9) { best.j = best.i; best.d2 = best.d1; }
                    best.i = i; best.d1 = d;
                } else if (d < best.d2 && std::fabs(dot(pieces[best.i].axis, pc.axis)) < 0.9) {
                    best.j = i; best.d2 = d;
                }
            }
        }
        double acc = 1e30;
        for (int c : chainOrder) {
            const double d = per[static_cast<std::size_t>(c)];
            if (d >= 1e29) continue;
            if (acc >= 1e29) { acc = d; continue; }
            const double k = chainK[static_cast<std::size_t>(c)];
            const double h = std::clamp(0.5 + 0.5 * (d - acc) / k, 0.0, 1.0);   // (polynomial smooth min)
            acc = d + (acc - d) * h - k * h * (1.0 - h);
        }
        if (params.barkNoise > 0.0f) {   // the knots, at the scale of the limb they are on
            const double kz = 1.4 / std::max(0.05, rN * 4.0);
            acc -= params.barkNoise * rN * (2.0 * gnarlNoise(p * kz + Vec3(static_cast<double>(seed % 251), 0, 0), seed) - 1.0);
        }
        // (cut a little under the ground -- no round foot below it -- softly: a root grazing a hard cut leaves a sheet
        // thinner than a cell)
        {
            const double cut = static_cast<double>(footY) - 0.03 - p.y, k = 0.04;
            const double h = std::clamp(0.5 - 0.5 * (cut - acc) / k, 0.0, 1.0);   // (polynomial smooth max)
            acc = cut + (acc - cut) * h + k * h * (1.0 - h);
        }
        if (nr) *nr = best;
        return acc;
    };
    const Sdf sdf = [&](const Vec3& p) { return field(p, nullptr); };
    const double cell = std::max(0.005, static_cast<double>(params.trunkZoneCell));
    RenderMesh raw = polygonizeSdfSparse(sdf, bounds, cell);
    out.zoneRawTris = static_cast<int>(raw.indices.size() / 3);
    // onto the field: a few Newton steps
    const double eps = 0.25 * cell;
    auto grad = [&](const Vec3& p) {
        return Vec3(sdf(p + Vec3(eps, 0, 0)) - sdf(p - Vec3(eps, 0, 0)), sdf(p + Vec3(0, eps, 0)) - sdf(p - Vec3(0, eps, 0)),
                    sdf(p + Vec3(0, 0, eps)) - sdf(p - Vec3(0, 0, eps))) * (0.5 / eps);
    };
    auto project = [&](Vec3 p) {
        for (int it = 0; it < 4; ++it) {
            const double f = sdf(p);
            const Vec3 g = grad(p);
            const double g2 = dot(g, g);
            if (g2 < 1e-12 || std::fabs(f) < 1e-4 * cell) break;
            Vec3 step = g * (f / g2);
            if (step.length() > cell) step = normalize(step) * cell;
            p = p - step;
        }
        return p;
    };
    // EVEN TRIANGLES (Glenn, 2026-10-18, of the simplified trunk: "those triangles look really stretched"): the pinches
    // split, then remeshed isotropically (MeshBuilder::remesh), each edge about k times the radius of the limb it is on
    // -- finer up a thin limb, coarser round the trunk -- k from the budget (the raw surface's area over each part's
    // triangle size), no finer than a cell
    MeshBuilder::splitNonManifold(raw);
    MeshBuilder::fillHoles(raw, 24, project);   // (splitting where two sheets crossed at an edge can open a few)
    // (the limb's radius near p, looked up once a 6 cm cell: the remesher asks for it at every edge)
    std::unordered_map<uint64_t, double> radiusCache;
    auto radiusAt = [&](const Vec3& p) {
        const double c = 0.06;
        const uint64_t key = (static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.x / c)) & 0x1fffff) << 42) |
                             (static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.y / c)) & 0x1fffff) << 21) |
                             (static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.z / c)) & 0x1fffff));
        auto it = radiusCache.find(key);
        if (it != radiusCache.end()) return it->second;
        Near nr;
        field(p, &nr);
        const double r = nr.i >= 0 ? static_cast<double>(std::max(pieces[nr.i].ra, pieces[nr.i].rb)) : static_cast<double>(footR);
        radiusCache.emplace(key, r);
        return r;
    };
    double want = 0.0;   // the triangles at k = 1: each face's area over an equilateral's of side its limb's radius
    for (std::size_t k = 0; k + 2 < raw.indices.size(); k += 9) {
        const Vec3 &a = raw.vertices[raw.indices[k]].position, &b = raw.vertices[raw.indices[k + 1]].position,
                   &c = raw.vertices[raw.indices[k + 2]].position;
        const double r = radiusAt((a + b + c) * (1.0 / 3.0));
        want += 3.0 * 0.5 * cross(b - a, c - a).length() / (0.433 * r * r);
    }
    // (x 1.18: the remesher's edges run 4/5 to 4/3 of the target, its triangles fewer than equilateral ones of it --
    // measured on the oak, 4174 for 3000 asked)
    const double kEdge = 1.18 * std::sqrt(want / std::max(64, params.trunkZoneTris));
    // (most of the way there by quadric simplification -- fast, but long thin triangles -- then evened out)
    MeshBuilder::simplify(raw, static_cast<std::size_t>(2 * std::max(64, params.trunkZoneTris)));
    MeshBuilder::remesh(raw, [&](const Vec3& p) { return std::clamp(kEdge * radiusAt(p), cell, 0.6); }, project, 5);
    // (and once more at the size the count says, if it came out over: the budget is the budget)
    const std::size_t got = raw.indices.size() / 3, budget = static_cast<std::size_t>(std::max(64, params.trunkZoneTris));
    if (got > budget + budget / 10) {
        const double k2 = kEdge * std::sqrt(static_cast<double>(got) / budget);
        MeshBuilder::remesh(raw, [&](const Vec3& p) { return std::clamp(k2 * radiusAt(p), cell, 0.6); }, project, 3);
    }
    for (Vertex& v : raw.vertices) {
        const Vec3 g = grad(v.position);
        v.normal = g.length() > 1e-9 ? normalize(g) : Vec3(0, 1, 0);
    }
    // each vertex's nearest limb and crotch
    std::vector<Near> near(raw.vertices.size());
    for (std::size_t i = 0; i < raw.vertices.size(); ++i) field(raw.vertices[i].position, &near[i]);
    // the bark's angle round a piece at q (from its a end): measured from the frame between its ends' frames at q's t
    auto angleOf = [&](const Piece& pc, const Vec3& q, double t) {
        Vec3 r = pc.rightA * (1.0 - t) + pc.rightB * t;
        r = r - pc.axis * dot(r, pc.axis);
        r = r.length() > 1e-9 ? normalize(r) : pc.right;
        const Vec3 u = cross(pc.axis, r);
        double phi = std::atan2(dot(q, u), dot(q, r));
        if (phi < 0) phi += 2.0 * PI;
        return phi;
    };
    // THE BARK, BLENDED (Glenn, 2026-10-18: "we need to blend those textures better"): each triangle in the mapping of
    // its nearest limb A (by its middle) AND of the nearest limb of another chain B; each of its vertices a weight for
    // B by how much nearer A's surface is (half at the crease where they are as near, nothing a blend width out) --
    // the second UV and the weight ride the tangent slot (RenderMaterial::FLAG_UV_BLEND), so the shader crossfades the
    // two barks over the crotch instead of cutting from one to the other. A vertex shared by triangles of other pairs
    // or wraps is duplicated.
    RenderMesh& m = out.branches;
    out.zoneIndexStart = m.indices.size();
    out.zoneVertexStart = m.vertices.size();
    std::unordered_map<uint64_t, uint32_t> made;
    auto uvOf = [&](const Piece& pc, const Vec3& p, bool wrap, float& u, float& v) {
        const Vec3 q = p - pc.a;
        const double t = std::clamp(dot(q, pc.b - pc.a) / std::max(1e-12, (pc.b - pc.a).lengthSquared()), 0.0, 1.0);
        u = pc.uRep * static_cast<float>(angleOf(pc, q, t) / (2.0 * PI));
        if (wrap && u < 0.5f * pc.uRep) u += pc.uRep;
        v = (pc.distA + (pc.distB - pc.distA) * static_cast<float>(t)) * vScale;
    };
    auto mapped = [&](uint32_t vi, int A, int Bp, bool wrapA, bool wrapB) -> uint32_t {
        const uint64_t key = (static_cast<uint64_t>(vi) << 28) ^ (static_cast<uint64_t>(A) << 14) ^ (static_cast<uint64_t>(Bp + 1) << 2) ^
                             (wrapA ? 1u : 0u) ^ (wrapB ? 2u : 0u);
        auto it = made.find(key);
        if (it != made.end()) return it->second;
        const Piece& pa = pieces[static_cast<std::size_t>(A)];
        const Vertex& src = raw.vertices[vi];
        float u, v, u2 = 0, v2 = 0, w = 0;
        uvOf(pa, src.position, wrapA, u, v);
        if (Bp >= 0) {
            const Piece& pb = pieces[static_cast<std::size_t>(Bp)];
            uvOf(pb, src.position, wrapB, u2, v2);
            double t;
            const double dA = pieceDist(pa, src.position, t), dB = pieceDist(pb, src.position, t);
            // (logistic in how much nearer A is: half at the crease, falling away exponentially over a fifth of the
            // thinner limb's radius -- a straight band either cut as a line (narrow) or let a far limb's mapping in
            // where neighbouring triangles chose another second limb (wide), a saw-tooth along their edges)
            const double sigma = std::max(0.015, 0.2 * std::min(std::max(pa.ra, pa.rb), std::max(pb.ra, pb.rb)));
            w = static_cast<float>(1.0 / (1.0 + std::exp(std::clamp((dB - dA) / sigma, -30.0, 30.0))));
        }
        Vertex outV(src.position, src.normal, Vec3(u2, v2, w), u, v);
        // the crotch: a second limb, at an angle, nearly as near
        const Near& nr = near[vi];
        double crotch = 0.0;
        if (nr.j >= 0) {
            const double rr = std::max(0.02f, std::max(pieces[nr.i].ra, pieces[nr.i].rb));
            crotch = std::clamp(1.0 - (nr.d2 - nr.d1) / (0.5 * rr), 0.0, 1.0);
        }
        const Vec3 colA = barkTint(params, src.position, pa.color, footY, seed);
        const Vec3 colB = Bp >= 0 ? barkTint(params, src.position, pieces[static_cast<std::size_t>(Bp)].color, footY, seed) : colA;
        outV.color = (colA * (1.0 - w) + colB * w) * (1.0 - 0.35 * crotch);
        m.vertices.push_back(outV);
        const uint32_t id = static_cast<uint32_t>(m.vertices.size() - 1);
        made[key] = id;
        return id;
    };
    for (std::size_t k = 0; k + 2 < raw.indices.size(); k += 3) {
        const uint32_t vs[3] = {raw.indices[k], raw.indices[k + 1], raw.indices[k + 2]};
        const Vec3 mid = (raw.vertices[vs[0]].position + raw.vertices[vs[1]].position + raw.vertices[vs[2]].position) * (1.0 / 3.0);
        // its limbs: the nearest, and the nearest of another chain
        int A = -1, Bp = -1;
        double dA = 1e30, dB = 1e30;
        for (std::size_t i = 0; i < pieces.size(); ++i) {
            double t;
            const double d = pieceDist(pieces[i], mid, t);
            if (d < dA) {
                if (A >= 0 && pieces[static_cast<std::size_t>(A)].chain != pieces[i].chain) { Bp = A; dB = dA; }
                A = static_cast<int>(i); dA = d;
            } else if (d < dB && pieces[i].chain != pieces[static_cast<std::size_t>(A)].chain) {
                Bp = static_cast<int>(i); dB = d;
            }
        }
        if (A < 0) continue;
        if (Bp >= 0 && pieces[static_cast<std::size_t>(Bp)].chain == pieces[static_cast<std::size_t>(A)].chain) Bp = -1;
        // (round each limb's back, the tile's seam: the low side taken past it)
        auto wraps = [&](int P, bool w3[3]) {
            float us[3];
            for (int e = 0; e < 3; ++e) {
                float u, v;
                uvOf(pieces[static_cast<std::size_t>(P)], raw.vertices[vs[e]].position, false, u, v);
                us[e] = u / pieces[static_cast<std::size_t>(P)].uRep;
            }
            const bool wrap = std::max({us[0], us[1], us[2]}) - std::min({us[0], us[1], us[2]}) > 0.5f;
            for (int e = 0; e < 3; ++e) w3[e] = wrap && us[e] < 0.5f;
        };
        bool wa[3] = {false, false, false}, wb[3] = {false, false, false};
        wraps(A, wa);
        if (Bp >= 0) wraps(Bp, wb);
        const uint32_t ia = mapped(vs[0], A, Bp, wa[0], wb[0]), ib = mapped(vs[1], A, Bp, wa[1], wb[1]),
                       ic = mapped(vs[2], A, Bp, wa[2], wb[2]);
        const Vec3 nOut = raw.vertices[vs[0]].normal + raw.vertices[vs[1]].normal + raw.vertices[vs[2]].normal;
        MeshBuilder::triFacing(m, ia, ib, ic, nOut);
    }
    out.zoneVertexEnd = m.vertices.size();
    out.barkBlend = true;
    // (its stretches' bark mappings, for the limbs welded into it to take where they meet it)
    if (maps)
        for (const Piece& pc : pieces) {
            BarkMap bm;
            bm.c = {pc.a, pc.b};
            for (const Vec3& rr : {pc.rightA, pc.rightB}) {
                Vec3 r = rr - pc.axis * dot(rr, pc.axis);
                r = r.length() > 1e-9 ? normalize(r) : pc.right;
                bm.right.push_back(r);
                bm.up.push_back(cross(pc.axis, r));
            }
            bm.r = {pc.ra, pc.rb};
            bm.v = {pc.distA * vScale, pc.distB * vScale};
            bm.uRep = pc.uRep;
            bm.uOff = 0.0f;
            bm.color = pc.color;
            maps->push_back(bm);
        }
    out.zoneIndexEnd = m.indices.size();
}

}  // namespace

TreeMesh skinTree(const ModuleString& s, const TreeParams& params, uint32_t seed) {
    std::mt19937 rng(seed);

    Skeleton skel = buildSkeleton(s, params.angleJitter, rng);
    std::vector<SkeletonNode>& nodes = skel.nodes;
    assignRadii(nodes, params);
    shapeBranches(nodes, params, rng, seed);
    // THE LEAN: the whole tree tipped off vertical about its foot, toward a seeded azimuth
    if (params.lean > 0.0f && !nodes.empty()) {
        std::uniform_real_distribution<float> az(0.0f, 2.0f * static_cast<float>(PI));
        const float a = az(rng);
        const Quat q = Quat::fromAxisAngle(Vec3(std::cos(a), 0, std::sin(a)), params.lean * PI / 180.0);
        const Vec3 foot = nodes[0].pos;
        for (SkeletonNode& n : nodes) {
            n.pos = foot + q.rotate(n.pos - foot);
            n.heading = normalize(q.rotate(n.heading));
        }
    }

    // CLIPPED: everything beyond the box goes, and a node that lost its children is a tip (its leaves on the face)
    if (params.clipX > 0.0f && params.clipY > 0.0f && params.clipZ > 0.0f && !nodes.empty()) {
        const Vec3 foot = nodes[0].pos;
        std::vector<int> remap(nodes.size(), -1);
        std::vector<SkeletonNode> kept;
        for (size_t i = 0; i < nodes.size(); ++i) {   // (parent before child)
            const SkeletonNode& n = nodes[i];
            const Vec3 d = n.pos - foot;
            const bool inside = std::fabs(d.x) <= params.clipX && d.y >= -0.05 && d.y <= params.clipY &&
                                std::fabs(d.z) <= params.clipZ;
            if (!inside || (n.parent >= 0 && remap[n.parent] < 0)) continue;
            remap[i] = static_cast<int>(kept.size());
            SkeletonNode c = n;
            c.parent = n.parent >= 0 ? remap[n.parent] : -1;
            c.childCount = 0;
            kept.push_back(c);
        }
        for (SkeletonNode& c : kept)
            if (c.parent >= 0) kept[c.parent].childCount++;
        for (SkeletonNode& c : kept)
            if (c.childCount == 0) c.isTip = true;
        nodes.swap(kept);
    }

    // CROWDED FORKS STAGGERED (Glenn, 2026-10-18: "some trees look like branching messes"): where more than two limbs
    // of wood leave one node (a maple's opposite pair beside its leader, a pine's whorl), one joint must join them all
    // and its hull tangles. Real limbs leave a stem a little apart: the leader and the thickest side keep the node, and
    // each further side limb moves up the leader onto a new node of its own, three times its and the stem's radii together
    // apart -- every fork then two limbs out. (Where the leader's internode has no room for that, the fork stays.)
    if ((params.joints == 1 || params.trunkZone > 0.0f) && nodes.size() > 2) {
        std::vector<std::vector<int>> kids(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i)
            if (nodes[i].parent >= 0) kids[static_cast<std::size_t>(nodes[i].parent)].push_back(static_cast<int>(i));
        const std::size_t before = nodes.size();
        for (size_t n = 0; n < before; ++n) {
            const float rN = nodes[n].radius;
            if (rN < params.jointMinRadius || kids[n].size() < 3) continue;
            std::vector<int> wood;   // (the joint planner's arms)
            for (int c : kids[n])
                if (nodes[c].radius >= 0.4f * params.jointMinRadius && nodes[c].radius >= 0.3f * rN) wood.push_back(c);
            if (wood.size() < 3) continue;
            // the leader: the child best in line with the stem
            const Vec3 inDir = nodes[n].parent >= 0 ? normalize(nodes[n].pos - nodes[nodes[n].parent].pos) : nodes[n].heading;
            int lead = -1;
            double best = -2.0;
            for (int c : wood) {
                const Vec3 d = nodes[c].pos - nodes[n].pos;
                if (d.length() < 1e-9) continue;
                const double al = dot(normalize(d), inDir) + 0.5 * nodes[c].radius / rN;
                if (al > best) { best = al; lead = c; }
            }
            if (lead < 0) continue;
            std::vector<int> sides;
            for (int c : wood) if (c != lead) sides.push_back(c);
            std::sort(sides.begin(), sides.end(), [&](int a, int b) { return nodes[a].radius > nodes[b].radius; });
            const Vec3 a = nodes[n].pos, b = nodes[lead].pos;
            const double L = (b - a).length();
            if (L < 1e-6) continue;
            constexpr double kStagger = 3.0;   // (closer, their joints' cut-backs overlap and fall back to the plain hull)
            double need = 0.0;
            for (std::size_t k = 1; k < sides.size(); ++k) need += kStagger * (rN + nodes[sides[k]].radius);
            if (need > 0.8 * L) continue;   // (no room up the leader: crowded forks there are better than overlapping ones)
            double t = 0.0;
            int prev = static_cast<int>(n);
            for (std::size_t k = 1; k < sides.size(); ++k) {
                t += kStagger * (rN + nodes[sides[k]].radius);
                const double u = t / L;
                SkeletonNode m = nodes[n];
                m.pos = a + (b - a) * u;
                m.radius = nodes[n].radius + (nodes[lead].radius - nodes[n].radius) * static_cast<float>(u);
                m.distFromRoot = nodes[n].distFromRoot + static_cast<float>(u * L);
                m.parent = prev;
                m.isTip = false;
                nodes.push_back(m);
                const int mi = static_cast<int>(nodes.size()) - 1;
                kids.emplace_back();
                nodes[static_cast<std::size_t>(sides[k])].parent = mi;
                prev = mi;
            }
            if (prev != static_cast<int>(n)) nodes[static_cast<std::size_t>(lead)].parent = prev;
        }
        if (nodes.size() > before) {   // parent before child again (depth first from the root), the counts redone
            std::vector<std::vector<int>> ch(nodes.size());
            int root = -1;
            for (size_t i = 0; i < nodes.size(); ++i) {
                if (nodes[i].parent >= 0) ch[static_cast<std::size_t>(nodes[i].parent)].push_back(static_cast<int>(i));
                else if (root < 0) root = static_cast<int>(i);
            }
            std::vector<int> order, stack{root};
            while (!stack.empty()) {
                const int x = stack.back();
                stack.pop_back();
                order.push_back(x);
                for (auto it = ch[static_cast<std::size_t>(x)].rbegin(); it != ch[static_cast<std::size_t>(x)].rend(); ++it) stack.push_back(*it);
            }
            std::vector<int> remap(nodes.size(), -1);
            for (std::size_t i = 0; i < order.size(); ++i) remap[static_cast<std::size_t>(order[i])] = static_cast<int>(i);
            std::vector<SkeletonNode> sorted;
            sorted.reserve(order.size());
            for (int x : order) {
                SkeletonNode c = nodes[static_cast<std::size_t>(x)];
                c.parent = c.parent >= 0 ? remap[static_cast<std::size_t>(c.parent)] : -1;
                c.childCount = static_cast<int>(ch[static_cast<std::size_t>(x)].size());
                sorted.push_back(c);
            }
            nodes.swap(sorted);
        }
    }

    TreeMesh out;

    // Decompose the skeleton into branch chains: from each node, the child whose
    // direction best continues the incoming direction is the "apical" leader
    // (one smooth limb); every other child starts a new chain anchored at the
    // fork, so side branches socket into their parent. Each chain is then swept
    // as one continuous curved tube.
    std::vector<std::vector<int>> children(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++)
        if (nodes[i].parent >= 0) children[nodes[i].parent].push_back(static_cast<int>(i));

    auto incomingDir = [&](int n) -> Vec3 {
        if (nodes[n].parent < 0) return nodes[n].heading;
        Vec3 d = nodes[n].pos - nodes[nodes[n].parent].pos;
        double L = d.length();
        return L > 1e-9 ? d * (1.0 / L) : nodes[n].heading;
    };
    // (the limb carried on is the best-aligned of the THICK children: a twig lined up with the limb does not take
    // over from the limb that goes on the other way)
    std::vector<int> cont(nodes.size(), -1);
    for (size_t n = 0; n < nodes.size(); n++) {
        Vec3 inDir = incomingDir(static_cast<int>(n));
        double best = -2.0;
        float thickest = 0.0f;
        for (int c : children[n]) thickest = std::max(thickest, nodes[c].radius);
        for (int c : children[n]) {
            if (nodes[c].radius < 0.6f * thickest) continue;
            Vec3 d = nodes[c].pos - nodes[n].pos;
            double L = d.length();
            if (L < 1e-9) continue;
            double dp = dot(d * (1.0 / L), inDir);
            if (dp > best) { best = dp; cont[n] = c; }
        }
    }

    // THE SPRIGS (TreeParams::sprigSize; branch_cards.h): every node's REACH -- the farthest node of its subtree from
    // where it leaves its parent -- and, parent before child, a node within reach of sprigSize whose parent is not
    // starts a sprig, its whole subtree with it. (A tree whose trunk is within reach is all one: no sprigs.)
    std::vector<int> sprigOf(nodes.size(), -1);
    const int trunkTop = cont.empty() ? -1 : cont[0];
    if (params.sprigSize > 0.0f && trunkTop > 0) {
        std::vector<float> reach(nodes.size(), 0.0f);
        for (size_t d = 1; d < nodes.size(); ++d)
            for (int a = static_cast<int>(d); a > 0 && nodes[a].parent >= 0; a = nodes[a].parent)
                reach[a] = std::max(reach[a], static_cast<float>((nodes[d].pos - nodes[nodes[a].parent].pos).length()));
        if (reach[trunkTop] > params.sprigSize) {
            for (size_t n = 1; n < nodes.size(); ++n) {
                const int par = nodes[n].parent;
                if (par < 0) continue;
                if (sprigOf[par] >= 0) sprigOf[n] = sprigOf[par];
                else if (reach[n] <= params.sprigSize) {
                    sprigOf[n] = static_cast<int>(out.sprigs.size());
                    TreeMesh::Sprig sg;
                    sg.base = nodes[par].pos;
                    sg.radius = nodes[n].radius;
                    out.sprigs.push_back(sg);
                }
            }
            // each sprig's frame: out toward its nodes' middle, across their widest spread, the face skyward
            std::vector<std::vector<int>> members(out.sprigs.size());
            for (size_t n = 0; n < nodes.size(); ++n)
                if (sprigOf[n] >= 0) members[static_cast<std::size_t>(sprigOf[n])].push_back(static_cast<int>(n));
            // (its leaves reach past its nodes: a shoot and a leaf, a spray, a card)
            const double leafReach = params.foliage == 2 && params.leafKit.set ? params.leafKit.shootLength * 1.3 + params.leafKit.size
                                     : params.foliage == 1 ? params.leafSize : params.leafSize * 1.7;
            for (std::size_t q = 0; q < out.sprigs.size(); ++q) {
                TreeMesh::Sprig& sg = out.sprigs[q];
                Vec3 mid(0, 0, 0);
                for (int n : members[q]) mid = mid + nodes[n].pos;
                mid = mid * (1.0 / members[q].size());
                Vec3 ax = mid - sg.base;
                if (ax.length() < 1e-6) ax = nodes[members[q].front()].heading;
                ax = normalize(ax);
                const Vec3 ref = std::fabs(ax.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                const Vec3 e1 = normalize(cross(ax, ref)), e2 = cross(ax, e1);
                double cxx = 0, cxy = 0, cyy = 0;
                for (int n : members[q]) {
                    const Vec3 d = nodes[n].pos - sg.base;
                    const double x = dot(d, e1), y = dot(d, e2);
                    cxx += x * x; cxy += x * y; cyy += y * y;
                }
                const double th = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);   // (the 2 x 2 covariance's major axis)
                Vec3 side = normalize(e1 * std::cos(th) + e2 * std::sin(th));
                Vec3 nrm = normalize(cross(ax, side));
                if (nrm.y < 0.0) { side = side * -1.0; nrm = nrm * -1.0; }
                sg.axis = ax; sg.side = side; sg.normal = nrm;
                sg.nodes = static_cast<int>(members[q].size());
                for (int n : members[q]) {
                    const Vec3 d = nodes[n].pos - sg.base;
                    sg.length = std::max(sg.length, static_cast<float>(dot(d, ax)));
                    sg.halfWidth = std::max(sg.halfWidth, static_cast<float>(std::fabs(dot(d, side))));
                    sg.depth = std::max(sg.depth, static_cast<float>(std::fabs(dot(d, nrm))));
                }
                sg.length += static_cast<float>(leafReach);
                sg.halfWidth += static_cast<float>(leafReach);
                sg.depth += static_cast<float>(0.5 * leafReach);
            }
        }
    }
    const bool donor = params.sprigDonor != 0 && !out.sprigs.empty();
    if (!out.sprigs.empty() && !donor)   // (the fine branching the sprigs stand for: a twig card along each internode)
        for (size_t n = 1; n < nodes.size(); ++n) {
            const int par = nodes[n].parent;
            if (par < 0 || sprigOf[n] < 0) continue;
            TreeMesh::Twig tw;
            tw.a = nodes[static_cast<std::size_t>(par)].pos;
            tw.b = nodes[n].pos;
            tw.r = nodes[n].radius;
            // (leafy toward the ends -- a tip, or a twig forking no further; inside, where it forks, bare wood)
            tw.leafy = nodes[n].isTip || children[n].size() <= 1 ||
                       (params.leafThickness > 0.0f && nodes[n].radius <= 0.5f * params.leafThickness);
            tw.forks = static_cast<int>(children[n].size());
            out.twigs.push_back(tw);
        }
    // (a donor's every vertex is owned: -1 the tree's wood and leaves, else its sprig's)
    std::vector<int> barkOwner, leafOwner;
    auto own = [&](int sprig) {
        if (!donor) return;
        barkOwner.resize(out.branches.vertices.size(), sprig);
        leafOwner.resize(out.leaves.vertices.size(), sprig);
    };
    // THE SIDES a limb is built on (TreeParams::woodEdge): by its girth
    auto sidesFor = [&](float r) {
        if (params.woodEdge <= 0.0f) return params.ringSegments;
        const int n = static_cast<int>(std::ceil(2.0 * PI * r / params.woodEdge));
        return std::clamp(n, std::min(4, params.ringSegments), params.ringSegments);
    };

    // Walk chains from a worklist of (anchor, first) starts; the anchor is the
    // fork point, included so the curve passes through the joint.
    // A FORK INTO NEAR-EQUAL HALVES (the weld's crotch): a side child as thick as the continuing one -- neither is the
    // limb carried on; the limb ends there and both weld into it
    // THE JOINTS (ADR-0181): at a fork of wood at least jointMinRadius thick, every limb meeting there -- the one
    // coming in and the ones going out -- is CUT BACK, just far enough that its end ring clears the other limbs and
    // lies on the near side of every other end ring's plane; the end rings are then joined by one surface, the convex
    // hull of the rings found on a sphere round the fork (so a ring's own bumps cannot unseat it) less each ring's own
    // cap. A twig off a thick limb is still welded on (its tube starts buried in the joint). Planned here, before the
    // limbs are swept; the joints are built after, from the rings the limbs actually ended in.
    struct Arm { int child = -1; Vec3 u; float r = 0.0f; double d = 0.0, maxD = 0.0; };
    std::vector<char> isJoint(nodes.size(), 0), cutStart(nodes.size(), 0);
    std::vector<double> cutIn(nodes.size(), 0.0), cutOut(nodes.size(), 0.0);
    std::vector<std::vector<Arm>> jointArms(nodes.size());
    // (with the trunk zone the joints are its; outside it limbs weld -- Glenn, 2026-10-18: no hull joins, blend seams)
    if (params.joints == 1 && params.trunkZone <= 0.0f) {
        auto candidate = [&](int n) { return children[n].size() >= 2 && nodes[n].radius >= params.jointMinRadius; };
        if (std::getenv("RT_JOINT_DEBUG")) {
            int cands = 0, multi = 0;
            float rmax = 0;
            for (size_t n = 0; n < nodes.size(); n++) { cands += candidate(static_cast<int>(n)); multi += children[n].size() >= 2; rmax = std::max(rmax, nodes[n].radius); }
            std::fprintf(stderr, "[joint] %zu nodes, %d forks, %d candidates, max radius %.3f\n", nodes.size(), multi, cands, rmax);
        }
        for (size_t n = 0; n < nodes.size(); n++) {
            if (!candidate(static_cast<int>(n))) continue;
            const int par = nodes[n].parent;
            const float rJ = nodes[n].radius;
            std::vector<Arm> arms;
            if (par >= 0) {
                const Vec3 seg = nodes[n].pos - nodes[par].pos;
                const double L = seg.length();
                if (L < 1e-6) continue;
                arms.push_back({-1, seg * (-1.0 / L), rJ, 0.0, L * (candidate(par) ? 0.45 : 0.7)});
            }
            for (int c : children[n]) {
                if (sprigOf[c] >= 0) continue;   // (a sprig: its card hangs there)
                // a twig, or a limb far thinner than the fork (a spruce's whorl off its trunk): welded with a collar --
                // a joint's triangles cannot resolve a collar that small without thousands of them
                if (nodes[c].radius < 0.4f * params.jointMinRadius || nodes[c].radius < 0.3f * rJ) continue;
                const Vec3 seg = nodes[c].pos - nodes[n].pos;
                const double L = seg.length();
                if (L < 1e-6) continue;
                arms.push_back({c, seg * (1.0 / L), nodes[c].radius * 1.05f, 0.0, L * (candidate(c) ? 0.45 : 0.7)});
            }
            bool ok = false;
            for (;;) {
                int outs = 0;
                for (const Arm& a : arms) outs += a.child >= 0;
                if (outs < 2) break;   // (a bend, not a fork)
                for (Arm& a : arms) a.d = std::min(a.maxD, 0.75 * std::max(static_cast<double>(a.r), 0.5 * rJ));
                // each end ring a face of the joint's hull: every other ring on the near side of its plane (with a
                // margin), and clear of every other limb's tube
                for (int it = 0; it < 12; ++it)
                    for (Arm& a : arms) {
                        for (const Arm& b : arms) {
                            if (&a == &b) continue;
                            const double c = std::clamp(dot(a.u, b.u), -1.0, 1.0), sn = std::sqrt(std::max(0.0, 1.0 - c * c));
                            if (c > 0.0) a.d = std::max(a.d, (1.06 * b.r + a.r * c) / std::max(sn, 0.15));   // clear of b's limb
                            a.d = std::max(a.d, b.d * c + b.r * sn + 0.12 * a.r);                            // past b's ring
                        }
                        a.d = std::min(a.d, a.maxD);
                    }
                int worst = -1;
                for (std::size_t i = 0; i < arms.size(); ++i)
                    for (std::size_t j = 0; j < arms.size(); ++j) {
                        if (i == j) continue;
                        const Arm &a = arms[i], &b = arms[j];
                        const double c = std::clamp(dot(a.u, b.u), -1.0, 1.0), sn = std::sqrt(std::max(0.0, 1.0 - c * c));
                        // (the joint is built from the rings the limbs really end in, retreating them until it is
                        // whole -- only a limb running nearly alongside another is beyond that)
                        const bool bad = c > std::cos(2.0 * PI / 180.0);   // (only limbs on top of each other)
                        if (!bad) continue;
                        // drop the thinner of the two going out (never the limb coming in)
                        const int pick = a.child < 0 ? static_cast<int>(j) : b.child < 0 ? static_cast<int>(i)
                                       : (a.r < b.r ? static_cast<int>(i) : static_cast<int>(j));
                        if (worst < 0 || arms[pick].r < arms[worst].r) worst = pick;
                    }
                if (std::getenv("RT_JOINT_DEBUG")) {
                    std::fprintf(stderr, "[joint] node %zu rJ %.3f arms:", n, rJ);
                    for (const Arm& a : arms) std::fprintf(stderr, " (%d r %.3f d %.3f max %.3f)", a.child, a.r, a.d, a.maxD);
                    std::fprintf(stderr, " -> %s\n", worst < 0 ? "ok" : "drop");
                }
                if (worst < 0) { ok = true; break; }
                arms.erase(arms.begin() + worst);
            }
            if (!ok) continue;
            isJoint[n] = 1;
            for (const Arm& a : arms) {
                if (a.child < 0) cutIn[n] = a.d;
                else { cutOut[a.child] = a.d; cutStart[a.child] = 1; }
            }
            jointArms[n] = arms;
        }
    }

    std::vector<char> equalFork(nodes.size(), 0);
    if (params.collar > 0.0f)
        for (size_t n = 0; n < nodes.size(); n++) {
            if (isJoint[n]) continue;
            if (cont[n] < 0 || nodes[n].radius < params.forkMinRadius) continue;
            for (int c : children[n])
                if (c != cont[n] && nodes[c].radius >= params.forkRatio * nodes[cont[n]].radius) equalFork[n] = 1;
        }
    std::vector<std::pair<int, int>> work;
    auto pushStarts = [&](int node) {
        for (int c : children[node])
            if (c != cont[node] || equalFork[node] || isJoint[node]) work.push_back({node, c});
    };
    // a buttress root (k of rootCount): its path from the foot and its bark colour (the draws: azimuth, length, colour)
    auto buttressRoot = [&](int k, std::mt19937& r, Vec3& color) {
        const float baseR = std::max(nodes[0].radius, params.tipRadius * params.radiusScale);
        const Vec3 baseP = nodes[0].pos;
        const float rootLen = baseR * params.rootSpread, tipR = params.tipRadius * params.radiusScale;
        std::uniform_real_distribution<float> jit(-0.3f, 0.3f), lenv(0.7f, 1.2f), shade(0.85f, 1.0f);
        const float ang = (params.phyllotaxis * k) * static_cast<float>(PI) / 180.0f + jit(r);
        const Vec3 dir(std::cos(ang), 0.0f, std::sin(ang));
        const float L = rootLen * lenv(r);
        BranchPath rp;
        auto add = [&](const Vec3& p, float rad) {
            rp.points.push_back(p);
            rp.radius.push_back(rad);
            rp.dist.push_back(static_cast<float>((p - baseP).length()));
        };
        add(baseP + dir * (baseR * 0.3f) + Vec3(0, baseR * 0.7f, 0), baseR * 0.55f);
        add(baseP + dir * (baseR * 1.5f) + Vec3(0, baseR * 0.15f, 0), baseR * 0.4f);
        add(baseP + dir * (L * 0.6f), tipR * 2.0f);
        add(baseP + dir * L + Vec3(0, -0.02f, 0), tipR);
        color = params.barkColor * shade(r);
        return rp;
    };

    std::vector<BarkMap> zoneMaps;   // (the zone's bark mappings, stretch by stretch)
    // THE TRUNK ZONE (TreeParams::trunkZone): which wood it is -- from the foot, every node as thick, not a sprig
    std::vector<char> inZone(nodes.size(), 0);
    if (params.trunkZone > 0.0f && !nodes.empty() && nodes[0].radius >= params.trunkZone) {
        inZone[0] = 1;
        for (size_t n = 1; n < nodes.size(); ++n) {
            const int par = nodes[n].parent;
            inZone[n] = par >= 0 && inZone[par] && sprigOf[n] < 0 && nodes[n].radius >= params.trunkZone;
        }
    }
    const bool zoneOn = trunkTop > 0 && inZone[static_cast<std::size_t>(trunkTop)];
    if (zoneOn) {
        buildTrunkZone(out, nodes, children, inZone, params, seed, static_cast<float>(nodes[0].pos.y),
                       [&](int k, std::mt19937& r, Vec3& c) { return buttressRoot(k, r, c); }, &zoneMaps,
                       [&](int c) { return sprigOf[static_cast<std::size_t>(c)] < 0 &&
                                           (params.woodMinRadius <= 0.0f || nodes[static_cast<std::size_t>(c)].radius >= params.woodMinRadius); });
        own(-1);
        // the limbs leaving it: tubes from its nodes (a side one welded on its collar), the rest as before
        for (size_t z = 0; z < nodes.size(); ++z)
            if (inZone[z])
                for (int c : children[z])
                    if (!inZone[c]) work.push_back({static_cast<int>(z), c});
    } else {
        if (cont[0] != -1 && !isJoint[0]) work.push_back({0, cont[0]});
        pushStarts(0);
    }
    // each joint's end rings, as the limbs leave them: which limb's tube, and which end of it
    struct Tube { uint32_t vBase = 0, rings = 0; std::size_t iBase = 0, iEnd = 0; int startBack = 0, endBack = 0, segs = 0; };
    struct EndRing { int tube = -1; bool start = true; };
    std::vector<Tube> tubes;
    std::vector<std::vector<EndRing>> jointRings(nodes.size());

    float vScale = params.barkVScale;
    std::uniform_real_distribution<float> vary(0.85f, 1.0f);
    const float footY = nodes.empty() ? 0.0f : static_cast<float>(nodes[0].pos.y);
    const int trunkFirst = cont.empty() ? -1 : cont[0];
    // THE WELDS BLENDED (Glenn, 2026-10-18: "we need to blend those textures better ... let's get up to the thin
    // limbs"): every tube keeps its bark mapping (BarkMap), and a limb welded into a tube or the trunk zone takes its
    // PARENT's bark, normals and colour where it meets it -- the parent's mapping in its second UV, weighted 1 on the
    // parent's surface and falling away over its own radius out of it (RenderMaterial::FLAG_UV_BLEND), so at the
    // seam both sides wear the same bark and light the same way (SpeedTree's seam ring, by the same blend as the zone's)
    std::vector<BarkMap> tubeMaps;
    std::vector<int> tubeOfNode(nodes.size(), -1);
    std::vector<std::pair<uint32_t, Vec3>> weldBlend;   // (a vertex, its second UV and weight)
    while (!work.empty()) {
        auto [anchor, first] = work.back();
        work.pop_back();
        const int sprig = sprigOf[first];
        if (sprig >= 0 && !donor) continue;   // a sprig (all of it): its card stands in
        const bool trunk = anchor == 0 && first == trunkFirst;
        if (!trunk && params.woodMinRadius > 0.0f && nodes[first].radius < params.woodMinRadius && !isJoint[anchor]) {
            for (int cur = first; cur != -1; cur = cont[cur]) pushStarts(cur);   // (its own side limbs: thinner still)
            continue;
        }
        BranchPath path;
        auto append = [&](int idx) {
            path.points.push_back(nodes[idx].pos);
            path.radius.push_back(nodes[idx].radius);
            path.dist.push_back(nodes[idx].distFromRoot);
        };
        const bool cut = isJoint[anchor] && cutStart[first];
        int endsAt = -1, lastNode = -1;
        bool intoSprig = false;
        if (cut) {   // out from the joint by its cut-back
            const Vec3 u = normalize(nodes[first].pos - nodes[anchor].pos);
            path.points.push_back(nodes[anchor].pos + u * cutOut[first]);
            path.radius.push_back(nodes[first].radius * 1.05f);
            path.dist.push_back(nodes[anchor].distFromRoot + static_cast<float>(cutOut[first]));
        } else {
            append(anchor);
            // (out of the trunk zone: its first ring narrower than the zone's limb there, so its open start is buried in
            // the zone -- a ring as wide stood on the zone's surface, and the tube's open end showed as a slit of sky)
            if (inZone[static_cast<std::size_t>(anchor)])
                path.radius.back() = std::min(0.8f * path.radius.back(), 0.85f * nodes[first].radius);
        }
        for (int cur = first; cur != -1; cur = cont[cur]) {
            if (sprig < 0 && sprigOf[cur] >= 0) {   // the limb runs on into a sprig: the wood ends here
                if (donor) work.push_back({lastNode >= 0 ? lastNode : anchor, cur});
                intoSprig = true;
                // (with twig cards on the sprig, the wood TAPERS on along its first internode to a twig's girth -- the
                // card's drawn twig takes it from there; a cone or a sawn end at the node stood out as a horn or stump)
                if (!donor && params.foliage == 2) {   // (on to the sprig's first node, down to a twig's girth)
                    const Vec3 from = path.points.back();
                    path.points.push_back(nodes[cur].pos);
                    path.radius.push_back(std::max(params.tipRadius * params.radiusScale * 1.2f, 0.3f * path.radius.back()));
                    path.dist.push_back(path.dist.back() + static_cast<float>((nodes[cur].pos - from).length()));
                }
                break;
            }
            if (isJoint[cur]) {   // ending short of the joint by its cut-back
                const int par = nodes[cur].parent;
                const Vec3 u = normalize(nodes[cur].pos - nodes[par].pos);
                path.points.push_back(nodes[cur].pos - u * cutIn[cur]);
                path.radius.push_back(nodes[cur].radius);
                path.dist.push_back(nodes[cur].distFromRoot - static_cast<float>(cutIn[cur]));
                pushStarts(cur);
                endsAt = cur;
                break;
            }
            append(cur);
            pushStarts(cur);
            lastNode = cur;
            if (equalFork[cur]) {
                // THE STUB the halves weld into: on along the limb as far as they leave its surface, then capped round
                const int par = nodes[cur].parent;
                const Vec3 dIn = par >= 0 ? normalize(nodes[cur].pos - nodes[par].pos) : nodes[cur].heading;
                const float r = nodes[cur].radius;
                double ls = 0.6 * r;
                for (int ch : children[cur]) {
                    const Vec3 dc = normalize(nodes[ch].pos - nodes[cur].pos);
                    const double cs = std::clamp(dot(dc, dIn), -1.0, 1.0), sn = std::sqrt(std::max(1e-6, 1.0 - cs * cs));
                    ls = std::max(ls, (r + 1.7 * nodes[ch].radius) * cs / sn);
                }
                ls = std::min(ls, 4.0 * static_cast<double>(r));
                const Vec3 p0 = nodes[cur].pos;
                const float d0 = nodes[cur].distFromRoot;
                auto stub = [&](double t, float rad) {
                    path.points.push_back(p0 + dIn * t);
                    path.radius.push_back(rad);
                    path.dist.push_back(d0 + static_cast<float>(t));
                };
                stub(ls, r * 0.98f);
                stub(ls + 0.5 * r, r * 0.12f);
                break;
            }
        }
        // A TIP ends in a point, not a sawn-off end (near foliage only -- a card hides it)
        const bool pointTip = (params.foliage != 0 && endsAt < 0 && lastNode >= 0 && children[lastNode].empty()) ||
                              (intoSprig && path.points.size() >= 2 && params.foliage != 2);
        Vec3 color = params.barkColor * vary(rng);
        Socket sock;
        const Socket* sp = nullptr;
        // (out of the trunk zone, a limb comes out of its STUB in the zone instead -- buildTrunkZone)
        if (!trunk && !cut && params.collar > 0.0f && !inZone[static_cast<std::size_t>(anchor)]) {   // a side branch: welded into its parent at the fork
            const int par = nodes[anchor].parent;
            const Vec3 dIn = par >= 0 ? normalize(nodes[anchor].pos - nodes[par].pos) : nodes[anchor].heading;
            const Vec3 dOut = cont[anchor] >= 0 ? normalize(nodes[cont[anchor]].pos - nodes[anchor].pos) : dIn;
            sock.c = nodes[anchor].pos;
            sock.d = equalFork[anchor] ? dIn : normalize(dIn + dOut);   // (a crotch: the stub runs on along the limb)
            sock.r = nodes[anchor].radius;
            sock.collar = params.collar;
            // (only a limb that LEAVES its parent at an angle: one running near along the parent's axis stays "inside" it
            // for metres, and its rings up to there were dropped -- a stem of a fan floating above its fork. It starts
            // buried instead, as a fork's halves do)
            const Vec3 dC = normalize(nodes[first].pos - nodes[anchor].pos);
            const double sn = std::sqrt(std::max(0.0, 1.0 - dot(dC, sock.d) * dot(dC, sock.d)));
            // (a near-equal fork's halves weld into its stub -- but in the trunk zone there is no stub: the zone is the
            // fork, and a half run along the limb's axis is the same floating stem)
            if ((equalFork[anchor] && !inZone[static_cast<std::size_t>(anchor)]) || sn > 0.35) sp = &sock;
        }
        if (std::getenv("RT_JOINT_DEBUG") && endsAt >= 0 && !cut && !trunk)
            std::fprintf(stderr, "[joint] limb into joint %d starts welded at node %d (joint %d, %zu children, r %.3f, first %d r %.3f, stub %d)\n",
                         endsAt, anchor, isJoint[anchor], children[anchor].size(), nodes[anchor].radius, first,
                         nodes[first].radius, equalFork[anchor]);
        Tube tb;
        tb.vBase = static_cast<uint32_t>(out.branches.vertices.size());
        tb.iBase = out.branches.indices.size();
        tb.segs = sidesFor(trunk ? nodes[0].radius : nodes[first].radius);
        own(-1);
        BarkMap map;
        if (path.points.size() >= 2)
            addCurvedBranch(out.branches, path, color, tb.segs, vScale, 3, &params, trunk, footY, seed, sp, pointTip, &map);
        own(sprig);
        // the weld into its parent, blended
        const bool fromZone = inZone[static_cast<std::size_t>(anchor)] != 0;
        const BarkMap* parentTube = !fromZone && tubeOfNode[static_cast<std::size_t>(anchor)] >= 0
                                        ? &tubeMaps[static_cast<std::size_t>(tubeOfNode[static_cast<std::size_t>(anchor)])] : nullptr;
        if (!trunk && !cut && path.points.size() >= 2 && (sp || fromZone) && (parentTube || (fromZone && !zoneMaps.empty()))) {
            // the parent at p: the zone's nearest stretch, or the tube
            auto parentAt = [&](const Vec3& p) -> const BarkMap* {
                if (!fromZone) return parentTube;
                const BarkMap* best = nullptr;
                double bd = 1e30;
                for (const BarkMap& zm : zoneMaps) {
                    const double d = zm.dist(p);
                    if (d < bd) { bd = d; best = &zm; }
                }
                return best;
            };
            const double sigma = std::max(0.01, 1.0 * static_cast<double>(nodes[first].radius));
            // (the parent's u near where the limb leaves it: every second UV unwrapped to within half a tile of it)
            const Vec3 leave = path.points.size() >= 2 ? path.points[0] + normalize(path.points[1] - path.points[0]) * (nodes[anchor].radius)
                                                       : path.points[0];
            const BarkMap* pRef = parentAt(leave);
            float uRef = 0, vRef = 0;
            if (pRef) pRef->uv(leave, uRef, vRef);
            for (uint32_t vi = tb.vBase; vi < out.branches.vertices.size(); ++vi) {
                Vertex& vx = out.branches.vertices[vi];
                const BarkMap* pm = parentAt(vx.position);
                if (!pm) continue;
                const double d = pm->dist(vx.position);
                const double w = d <= 0.0 ? 1.0 : std::exp(-d / sigma);
                if (w < 0.01) continue;
                float u2, v2;
                pm->uv(vx.position, u2, v2);
                if (pm == pRef) u2 -= pm->uRep * std::round((u2 - uRef) / pm->uRep);
                weldBlend.push_back({vi, Vec3(u2, v2, w)});
                vx.normal = normalize(vx.normal * (1.0 - 0.75 * w) + pm->normal(vx.position) * (0.75 * w));
                vx.color = vx.color * (1.0 - w) + barkTint(params, vx.position, pm->color, footY, seed) * w;
            }
        }
        if (map.c.size() >= 2) {   // (this tube's mapping, for the limbs welded into it)
            tubeMaps.push_back(std::move(map));
            const int ti = static_cast<int>(tubeMaps.size()) - 1;
            for (int cur = first; cur != -1 && lastNode >= 0; cur = cont[cur]) {   // (the nodes it ran through)
                tubeOfNode[static_cast<std::size_t>(cur)] = ti;
                if (cur == lastNode) break;
            }
            if (!cut && tubeOfNode[static_cast<std::size_t>(anchor)] < 0 && !inZone[static_cast<std::size_t>(anchor)])
                tubeOfNode[static_cast<std::size_t>(anchor)] = ti;
        }
        // (a point tip's apex is one vertex past its rings)
        tb.rings = (static_cast<uint32_t>(out.branches.vertices.size()) - tb.vBase) / static_cast<uint32_t>(tb.segs + 1);
        tb.iEnd = out.branches.indices.size();
        if (tb.rings >= 2 && (cut || endsAt >= 0)) {
            tubes.push_back(tb);
            if (cut) jointRings[anchor].push_back({static_cast<int>(tubes.size()) - 1, true});
            if (endsAt >= 0) jointRings[endsAt].push_back({static_cast<int>(tubes.size()) - 1, false});
        }
    }

    // THE JOINTS, from the rings the limbs ended in. Each ring is taken as the circle it would be (its vertices' own
    // bearings round its centre, at its mean radius, in its own plane -- so a knotty ring's dents cannot leave a vertex
    // off the hull) with an apex a little out along its limb (so its cap is a cone the hull cannot cut across); the
    // hull of all of them, less the apexes' faces, joins ring to ring. A joint is WHOLE when every ring edge lies on
    // exactly one remaining face. Where it is not, a limb is in the way of a ring -- the plan's ideal limbs were not
    // the gnarled ones swept -- and that ring RETREATS one ring further along its own tube (the band it leaves goes),
    // as a road is cut back from a junction until its end is clear; up to a few rings. A joint still not whole is
    // closed by the plain hull of its rings (covered, if not as tidy). Wound outward; a ring vertex at the u seam takes
    // its duplicate where its face reaches round from the far side, so the bark does not smear back across the tile.
    int jointsBuilt = 0, jointsPlain = 0;
    own(-1);
    const std::size_t jointIdxStart = out.branches.indices.size();
    auto ringStart = [&](const EndRing& e) {
        const Tube& t = tubes[static_cast<std::size_t>(e.tube)];
        const uint32_t k = e.start ? static_cast<uint32_t>(t.startBack) : t.rings - 1 - static_cast<uint32_t>(t.endBack);
        return t.vBase + k * static_cast<uint32_t>(t.segs + 1);
    };
    if (std::getenv("RT_JOINT_DEBUG")) {
        int short_ = 0;
        for (size_t j = 0; j < nodes.size(); ++j)
            if (isJoint[j] && jointRings[j].size() != jointArms[j].size()) {
                ++short_;
                if (short_ <= 5) std::fprintf(stderr, "[joint] node %zu: %zu arms, %zu rings\n", j, jointArms[j].size(), jointRings[j].size());
            }
        std::fprintf(stderr, "[joint] %d joints missing rings\n", short_);
    }
    for (size_t j = 0; j < nodes.size(); ++j) {
        if (!isJoint[j] || jointRings[j].size() < 3) continue;
        const std::vector<EndRing>& ends = jointRings[j];
        const Vec3 C = nodes[j].pos;
        std::vector<Vec3> pts;
        std::vector<int> ringOf, idxOf;   // per hull point: its ring (-1 an apex) and its index round the ring
        std::vector<std::array<uint32_t, 3>> keep;
        std::vector<uint32_t> rings(ends.size());
        std::vector<int> segsOf(ends.size());   // (each ring its own limb's sides)
        for (std::size_t a = 0; a < ends.size(); ++a) segsOf[a] = tubes[static_cast<std::size_t>(ends[a].tube)].segs;
        bool whole = false;
        for (int attempt = 0; attempt < 8 && !whole; ++attempt) {
            pts.clear(); ringOf.clear(); idxOf.clear(); keep.clear();
            std::vector<Vec3> cen(ends.size()), nrmOf(ends.size());
            std::vector<double> rhoOf(ends.size(), 0.0);
            for (std::size_t a = 0; a < ends.size(); ++a) {
                rings[a] = ringStart(ends[a]);
                const int segs = segsOf[a];
                Vec3 centroid(0, 0, 0);
                for (int i = 0; i < segs; ++i) centroid = centroid + out.branches.vertices[rings[a] + i].position;
                centroid = centroid * (1.0 / segs);
                Vec3 nrm(0, 0, 0);
                for (int i = 0; i < segs; ++i)
                    nrm = nrm + cross(out.branches.vertices[rings[a] + i].position - centroid,
                                      out.branches.vertices[rings[a] + (i + 1) % segs].position - centroid);
                nrm = nrm.length() > 1e-12 ? normalize(nrm) : normalize(centroid - C);
                if (dot(nrm, centroid - C) < 0.0) nrm = nrm * -1.0;
                cen[a] = centroid;
                nrmOf[a] = nrm;
                const Vec3 ref = std::fabs(nrm.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                const Vec3 e1 = normalize(cross(nrm, ref)), e2 = cross(nrm, e1);
                double rho = 0.0;
                for (int i = 0; i < segs; ++i) {
                    const Vec3 v = out.branches.vertices[rings[a] + i].position - centroid;
                    rho += (v - nrm * dot(v, nrm)).length();
                }
                rho /= segs;
                pts.push_back(centroid + nrm * (0.03 * rho));
                rhoOf[a] = rho;
                ringOf.push_back(-1);
                idxOf.push_back(static_cast<int>(a));
                for (int i = 0; i < segs; ++i) {
                    const Vec3 v = out.branches.vertices[rings[a] + i].position - centroid;
                    const double phi = std::atan2(dot(v, e2), dot(v, e1));
                    pts.push_back(centroid + (e1 * std::cos(phi) + e2 * std::sin(phi)) * rho);
                    ringOf.push_back(static_cast<int>(a));
                    idxOf.push_back(i);
                }
            }
            const auto faces = MeshBuilder::convexHull(pts);
            for (const auto& f : faces)
                if (ringOf[f[0]] >= 0 && ringOf[f[1]] >= 0 && ringOf[f[2]] >= 0) keep.push_back(f);
            whole = !faces.empty();
            for (std::size_t a = 0; a < ends.size() && whole; ++a)
                for (int i = 0, segs = segsOf[a]; i < segs && whole; ++i) {
                    int found = 0;
                    for (const auto& f : keep)
                        for (int e = 0; e < 3; ++e) {
                            const uint32_t x = f[e], y = f[(e + 1) % 3];
                            const auto same = [&](uint32_t q, int ii) { return ringOf[q] == static_cast<int>(a) && idxOf[q] == ii; };
                            if ((same(x, i) && same(y, (i + 1) % segs)) || (same(y, i) && same(x, (i + 1) % segs))) ++found;
                        }
                    whole = found == 1;
                }
            if (!whole) {
                if (const char* dump = std::getenv("RT_JOINT_DUMP")) {
                    static int dumped = 0;
                    if (dumped < 3) {
                        std::string fn = std::string(dump) + "/joint" + std::to_string(dumped++) + ".txt";
                        if (FILE* fp = std::fopen(fn.c_str(), "w")) {
                            for (std::size_t q = 0; q < pts.size(); ++q)
                                std::fprintf(fp, "p %d %d %.9f %.9f %.9f\n", ringOf[q], idxOf[q], pts[q].x, pts[q].y, pts[q].z);
                            for (const auto& f : faces) std::fprintf(fp, "f %u %u %u\n", f[0], f[1], f[2]);
                            std::fclose(fp);
                        }
                    }
                }
            }
            if (whole) break;
            // retreat each ring another limb's ring reaches past (on the far side of its plane)
            bool moved = false;
            for (std::size_t a = 0; a < ends.size(); ++a) {
                bool crossed = false;
                for (std::size_t b2 = 0; b2 < ends.size() && !crossed; ++b2) {
                    if (b2 == a) continue;
                    for (int i = 0; i < segsOf[b2] && !crossed; ++i)
                        crossed = dot(out.branches.vertices[rings[b2] + i].position - cen[a], nrmOf[a]) > -0.06 * rhoOf[a];
                }
                if (!crossed) continue;
                Tube& t = tubes[static_cast<std::size_t>(ends[a].tube)];
                if (static_cast<uint32_t>(t.startBack + t.endBack) + 2 > t.rings) continue;   // (keep a band of the limb)
                (ends[a].start ? t.startBack : t.endBack) += 1;
                moved = true;
                if (std::getenv("RT_JOINT_DEBUG"))
                    std::fprintf(stderr, "[retreat] joint %zu ring %zu (%s, r %.3f) attempt %d\n", j, a, ends[a].start ? "out" : "in", rhoOf[a], attempt);
            }
            if (!moved) break;
        }
        if (std::getenv("RT_JOINT_DEBUG") && !whole)
            std::fprintf(stderr, "[joint] node %zu PLAIN: %zu rings\n", j, ends.size());
        auto vertexOf = [&](uint32_t q, const std::array<uint32_t, 3>& f) -> uint32_t {
            const int a = ringOf[q], i = idxOf[q];
            const int segs = segsOf[static_cast<std::size_t>(a)];
            int idx = i;
            if (i == 0)   // the seam: the duplicate (u = 1) when this face reaches round from the far side
                for (uint32_t o : f)
                    if (ringOf[o] == a && idxOf[o] > segs / 2) idx = segs;
            return rings[static_cast<std::size_t>(a)] + static_cast<uint32_t>(idx);
        };
        {
            // THE JOINT'S SHAPE: the hull is only the joint's topology -- its spans are straight and it bulges out to
            // every ring (a diamond round a thin limb, a web across a crotch). Refined (every edge but the rings' own
            // split until it is about a ring segment long) and its new vertices PROJECTED onto the limbs' smooth
            // union -- each limb a capsule from the fork to its ring, blended in by a fillet a fifth of its radius --
            // it becomes the crotch's saddle and a twig's collar.
            struct Cap { Vec3 a, b; double r, k; int segs; };
            std::vector<Cap> caps;
            double rhoMin = 1e9;
            for (std::size_t a = 0; a < ends.size(); ++a) {
                const int segs = segsOf[a];
                Vec3 cen(0, 0, 0);
                for (int i = 0; i < segs; ++i) cen = cen + out.branches.vertices[rings[a] + i].position;
                cen = cen * (1.0 / segs);
                double rho = 0.0;
                for (int i = 0; i < segs; ++i) rho += (out.branches.vertices[rings[a] + i].position - cen).length();
                rho /= segs;
                rhoMin = std::min(rhoMin, rho);
                const Vec3 ax = normalize(cen - C);
                caps.push_back({C, cen + ax * (0.5 * rho), rho, 0.2 * rho, segs});
            }
            // the thickest first, the rest blended into it
            std::sort(caps.begin(), caps.end(), [](const Cap& x, const Cap& y) { return x.r > y.r; });
            auto field = [&](const Vec3& p) {
                double acc = 1e9;
                for (std::size_t q = 0; q < caps.size(); ++q) {
                    const Cap& c = caps[q];
                    const Vec3 ab = c.b - c.a;
                    const double t = std::clamp(dot(p - c.a, ab) / std::max(1e-12, dot(ab, ab)), 0.0, 1.0);
                    const double d = (p - (c.a + ab * t)).length() - c.r;
                    if (q == 0) { acc = d; continue; }
                    const double h = std::clamp(0.5 + 0.5 * (d - acc) / c.k, 0.0, 1.0);   // (polynomial smooth min)
                    acc = d + (acc - d) * h - c.k * h * (1.0 - h);
                }
                return acc;
            };
            const double eps = 0.02 * rhoMin;
            auto grad = [&](const Vec3& p) {
                return Vec3(field(p + Vec3(eps, 0, 0)) - field(p - Vec3(eps, 0, 0)),
                            field(p + Vec3(0, eps, 0)) - field(p - Vec3(0, eps, 0)),
                            field(p + Vec3(0, 0, eps)) - field(p - Vec3(0, 0, eps))) * (0.5 / eps);
            };
            // the joint's triangles, as vertex indices into out.branches (the rings' own vertices): the whole hull's,
            // or (not whole) the plain hull of the rings themselves less each ring's own cap
            std::vector<std::array<uint32_t, 3>> tris;
            if (whole) {
                for (const auto& f : keep) tris.push_back({vertexOf(f[0], f), vertexOf(f[1], f), vertexOf(f[2], f)});
            } else {
                std::vector<Vec3> real;
                std::vector<uint32_t> realIdx;
                std::vector<int> realRing;
                for (std::size_t a = 0; a < rings.size(); ++a)
                    for (int i = 0; i < segsOf[a]; ++i) {
                        real.push_back(out.branches.vertices[rings[a] + i].position);
                        realIdx.push_back(rings[a] + i);
                        realRing.push_back(static_cast<int>(a));
                    }
                for (const auto& f : MeshBuilder::convexHull(real))
                    if (!(realRing[f[0]] == realRing[f[1]] && realRing[f[1]] == realRing[f[2]]))
                        tris.push_back({realIdx[f[0]], realIdx[f[1]], realIdx[f[2]]});
            }
            auto ringEdge = [&](uint32_t x, uint32_t y) {   // two neighbours round one ring: never split
                for (std::size_t a = 0; a < rings.size(); ++a) {
                    const uint32_t r = rings[a];
                    const int segs = segsOf[a];
                    const bool inX = x >= r && x <= r + static_cast<uint32_t>(segs), inY = y >= r && y <= r + static_cast<uint32_t>(segs);
                    if (inX && inY) {
                        const int dx = static_cast<int>(x - r), dy = static_cast<int>(y - r);
                        const int dd = std::abs(dx - dy);
                        return dd == 1 || dd == segs - 1;
                    }
                }
                return false;
            };
            // (an edge is split to the ring spacing of the limb it lies on: coarse round the trunk, fine at a twig)
            // (near a thin limb, its fine spacing, easing back to the coarse with the distance from it)
            constexpr double kGrow = 1.8, kFloor = 0.35;
            constexpr int kLevels = 4;
            // (the wood built by need, woodEdge: the joint's triangles as coarse as the tubes beside them -- twice the
            // spacing, the same smooth union; Glenn, 2026-10-18: "it makes a lot of triangles")
            const double kSpan = params.woodEdge > 0.0f ? 3.5 : 1.7;
            auto targetAt = [&](const Vec3& p) {
                double best = 1e9;
                for (const Cap& c : caps) {
                    const Vec3 ab = c.b - c.a;
                    const double t = std::clamp(dot(p - c.a, ab) / std::max(1e-12, dot(ab, ab)), 0.0, 1.0);
                    const double d = std::max(0.0, (p - (c.a + ab * t)).length() - c.r);
                    best = std::min(best, 2.0 * PI * c.r / c.segs * kSpan + kGrow * d);
                }
                return std::max(best, kFloor * 2.0 * PI * caps.front().r / caps.front().segs * kSpan);   // (no finer than this share of the joint's own)
            };
            // (onto the limbs' union: a few Newton steps)
            auto project = [&](Vec3 p) {
                for (int it = 0; it < 6; ++it) {
                    const double fv = field(p);
                    const Vec3 g = grad(p);
                    const double g2 = dot(g, g);
                    if (g2 < 1e-12 || std::fabs(fv) < 1e-5 * rhoMin) break;
                    Vec3 step = g * (fv / g2);
                    if (step.length() > 0.5 * rhoMin) step = normalize(step) * (0.5 * rhoMin);
                    p = p - step;
                }
                return p;
            };
            const std::size_t firstNew = out.branches.vertices.size();
            for (int level = 0; level < kLevels; ++level) {
                std::map<std::pair<uint32_t, uint32_t>, uint32_t> mids;
                bool any = false;
                auto midOf = [&](uint32_t x, uint32_t y) -> int64_t {
                    if (ringEdge(x, y)) return -1;
                    const Vertex &vx = out.branches.vertices[x], &vy = out.branches.vertices[y];
                    if ((vx.position - vy.position).length() < targetAt((vx.position + vy.position) * 0.5)) return -1;
                    const auto key = std::make_pair(std::min(x, y), std::max(x, y));
                    auto it = mids.find(key);
                    if (it != mids.end()) return it->second;
                    Vertex m = vx;
                    m.position = (vx.position + vy.position) * 0.5;
                    float uy = vy.u;   // (u unwrapped toward x's: the bark tile does not run back across the seam)
                    if (uy - vx.u > 0.5f * std::max(1.0f, std::fabs(vx.u) + std::fabs(vy.u))) uy -= std::round(uy - vx.u);
                    m.u = 0.5f * (vx.u + uy);
                    m.v = 0.5f * (vx.v + vy.v);
                    m.color = (vx.color + vy.color) * 0.5;
                    out.branches.vertices.push_back(m);
                    const uint32_t id = static_cast<uint32_t>(out.branches.vertices.size() - 1);
                    mids[key] = id;
                    any = true;
                    return id;
                };
                std::vector<std::array<uint32_t, 3>> next;
                for (const auto& t : tris) {
                    int64_t m[3];
                    for (int e = 0; e < 3; ++e) m[e] = midOf(t[e], t[(e + 1) % 3]);
                    const int nSplit = (m[0] >= 0) + (m[1] >= 0) + (m[2] >= 0);
                    if (nSplit == 0) { next.push_back(t); continue; }
                    if (nSplit == 3) {
                        const uint32_t a = t[0], b = t[1], c = t[2];
                        const uint32_t ab = static_cast<uint32_t>(m[0]), bc = static_cast<uint32_t>(m[1]), ca = static_cast<uint32_t>(m[2]);
                        next.push_back({a, ab, ca}); next.push_back({ab, b, bc}); next.push_back({ca, bc, c}); next.push_back({ab, bc, ca});
                        continue;
                    }
                    // one or two split: turn the triangle so its first edge is split
                    int r0 = 0;
                    while (m[r0] < 0) ++r0;
                    if (nSplit == 2 && m[(r0 + 2) % 3] >= 0) r0 = (r0 + 2) % 3;   // (the two split edges r0, r0+1)
                    const uint32_t a = t[r0], b = t[(r0 + 1) % 3], c = t[(r0 + 2) % 3];
                    const uint32_t m1 = static_cast<uint32_t>(m[r0]);
                    if (nSplit == 1) { next.push_back({a, m1, c}); next.push_back({m1, b, c}); continue; }
                    const uint32_t m2 = static_cast<uint32_t>(m[(r0 + 1) % 3]);
                    next.push_back({a, m1, m2}); next.push_back({m1, b, m2}); next.push_back({a, m2, c});
                }
                tris.swap(next);
                // the new vertices onto the limbs' union (a few Newton steps), as they come
                for (std::size_t v = firstNew; v < out.branches.vertices.size(); ++v)
                    out.branches.vertices[v].position = project(out.branches.vertices[v].position);
                if (!any) break;
            }
            // NO SLIVERS (Glenn: "some of those joins look like they have a lot of degenerate triangles"): the hull and
            // the midpoint splits leave long thin triangles fanned off the ring vertices. Edges FLIP where the two
            // triangles either side are fatter the other way (the smaller of their least angles grows), ring edges
            // never; the new vertices RELAX toward their neighbours' middle and go back onto the surface; three rounds.
            auto minAngle = [&](uint32_t a, uint32_t b, uint32_t c) {
                const Vec3 &A = out.branches.vertices[a].position, &B = out.branches.vertices[b].position, &Cc = out.branches.vertices[c].position;
                auto ang = [](const Vec3& p, const Vec3& q, const Vec3& r) {
                    const Vec3 u = q - p, w = r - p;
                    const double lu = u.length(), lw = w.length();
                    if (lu < 1e-12 || lw < 1e-12) return 0.0;
                    return std::acos(std::clamp(dot(u, w) / (lu * lw), -1.0, 1.0));
                };
                return std::min({ang(A, B, Cc), ang(B, Cc, A), ang(Cc, A, B)});
            };
            for (int pass = 0; pass < 3; ++pass) {
                for (int sweep = 0; sweep < 4; ++sweep) {   // flips
                    std::map<std::pair<uint32_t, uint32_t>, std::vector<std::pair<std::size_t, int>>> edgeTris;
                    for (std::size_t q = 0; q < tris.size(); ++q)
                        for (int e = 0; e < 3; ++e) {
                            const uint32_t x = tris[q][e], y = tris[q][(e + 1) % 3];
                            edgeTris[{std::min(x, y), std::max(x, y)}].push_back({q, e});
                        }
                    std::vector<char> touched(tris.size(), 0);
                    int flips = 0;
                    for (const auto& [edge, users] : edgeTris) {
                        if (users.size() != 2 || ringEdge(edge.first, edge.second)) continue;
                        const auto [q1, e1] = users[0];
                        const auto [q2, e2] = users[1];
                        if (touched[q1] || touched[q2]) continue;
                        const uint32_t a = tris[q1][e1], b = tris[q1][(e1 + 1) % 3], c = tris[q1][(e1 + 2) % 3];
                        uint32_t d = tris[q2][0] + tris[q2][1] + tris[q2][2] - a - b;   // q2's third vertex
                        if (d == c || d == a || d == b) continue;
                        // (a duplicate seam vertex can stand for one of a b: then the sum trick fails -- check it)
                        if (!((tris[q2][0] == d) || (tris[q2][1] == d) || (tris[q2][2] == d))) continue;
                        const double before = std::min(minAngle(a, b, c), minAngle(b, a, d));
                        const double after = std::min(minAngle(c, d, a), minAngle(d, c, b));
                        // (only where the quad is convex enough that the new diagonal stays on the surface)
                        const Vec3 nOld = cross(out.branches.vertices[b].position - out.branches.vertices[a].position,
                                                out.branches.vertices[c].position - out.branches.vertices[a].position);
                        const Vec3 n1 = cross(out.branches.vertices[d].position - out.branches.vertices[c].position,
                                              out.branches.vertices[a].position - out.branches.vertices[c].position);
                        const Vec3 n2 = cross(out.branches.vertices[c].position - out.branches.vertices[d].position,
                                              out.branches.vertices[b].position - out.branches.vertices[d].position);
                        if (after > before * 1.05 + 1e-3 && dot(n1, nOld) * dot(n2, nOld) > 0.0) {
                            tris[q1] = {c, d, a};
                            tris[q2] = {d, c, b};
                            touched[q1] = touched[q2] = 1;
                            ++flips;
                        }
                    }
                    if (!flips) break;
                }
                if (pass == 2) break;
                // relax the new vertices: toward their neighbours' middle, back onto the surface
                std::vector<Vec3> sum(out.branches.vertices.size() - firstNew, Vec3(0, 0, 0));
                std::vector<int> cnt(sum.size(), 0);
                for (const auto& t : tris)
                    for (int e = 0; e < 3; ++e) {
                        const uint32_t x = t[e];
                        if (x < firstNew) continue;
                        for (int o = 1; o < 3; ++o) { sum[x - firstNew] = sum[x - firstNew] + out.branches.vertices[t[(e + o) % 3]].position; cnt[x - firstNew] += 1; }
                    }
                for (std::size_t v = 0; v < sum.size(); ++v)
                    if (cnt[v]) {
                        Vertex& vx = out.branches.vertices[firstNew + v];
                        vx.position = project(vx.position * 0.4 + sum[v] * (0.6 / cnt[v]));
                    }
            }
            for (std::size_t v = firstNew; v < out.branches.vertices.size(); ++v) {
                Vertex& vx = out.branches.vertices[v];
                const Vec3 g = grad(vx.position);
                if (g.length() > 1e-9) vx.normal = normalize(g);
                const Vec3 tg = vx.tangent - vx.normal * dot(vx.tangent, vx.normal);
                if (tg.length() > 1e-9) vx.tangent = normalize(tg);
            }
            for (const auto& t : tris) {
                const Vec3 mid = (out.branches.vertices[t[0]].position + out.branches.vertices[t[1]].position +
                                  out.branches.vertices[t[2]].position) * (1.0 / 3.0);
                Vec3 outward = grad(mid);
                if (outward.length() < 1e-9) outward = mid - C;
                MeshBuilder::triFacing(out.branches, t[0], t[1], t[2], outward);
            }
            ++(whole ? jointsBuilt : jointsPlain);
        }
        for (uint32_t r : rings) out.jointRingStarts.push_back(r);
    }
    out.jointIndexEnd = out.branches.indices.size();
    out.jointIndexStart = jointIdxStart;
    // the bands the retreating rings left: gone from their tubes
    {
        std::vector<char> drop(out.branches.indices.size() / 3, 0);
        bool any = false;
        for (const Tube& t : tubes) {
            if (t.startBack == 0 && t.endBack == 0) continue;
            for (std::size_t k = t.iBase; k + 2 < t.iEnd; k += 3) {
                uint32_t lo = ~0u, hi = 0;
                for (int e = 0; e < 3; ++e) {
                    const uint32_t ring = (out.branches.indices[k + e] - t.vBase) / static_cast<uint32_t>(t.segs + 1);
                    lo = std::min(lo, ring);
                    hi = std::max(hi, ring);
                }
                if (hi <= static_cast<uint32_t>(t.startBack) || lo >= t.rings - 1 - static_cast<uint32_t>(t.endBack)) {
                    drop[k / 3] = 1;
                    any = true;
                }
            }
        }
        if (any) {
            std::vector<uint32_t> kept;
            kept.reserve(out.branches.indices.size());
            for (std::size_t k = 0; k + 2 < out.branches.indices.size(); k += 3)
                if (!drop[k / 3]) kept.insert(kept.end(), {out.branches.indices[k], out.branches.indices[k + 1], out.branches.indices[k + 2]});
            const std::size_t gone = out.branches.indices.size() - kept.size();   // (all of them tube bands, before the joints)
            out.jointIndexStart -= gone;
            out.jointIndexEnd -= gone;
            out.branches.indices.swap(kept);
        }
    }
    out.joints = jointsBuilt;
    out.jointsPlain = jointsPlain;

    // Buttress roots: from the base, swept out and down, flattening to run along
    // the ground (local y~0). Widens the flare and grounds the tree. (With the trunk zone they are part of it.)
    if (params.rootCount > 0 && !nodes.empty() && !zoneOn) {
        const float baseR = std::max(nodes[0].radius, params.tipRadius * params.radiusScale);
        const Vec3 baseP = nodes[0].pos;
        for (int k = 0; k < params.rootCount; k++) {
            Vec3 color;
            const BranchPath rp = buttressRoot(k, rng, color);
            // (the bark's variety only -- the dark foot and patches -- not the gnarl, which is for limbs)
            TreeParams rootLook;
            rootLook.barkMottle = params.barkMottle;
            rootLook.barkFootDark = params.barkFootDark;
            rootLook.barkFootHeight = params.barkFootHeight;
            rootLook.barkFootColor = params.barkFootColor;
            rootLook.ringAngle = params.ringAngle;
            rootLook.ringSpacing = params.ringSpacing;
            addCurvedBranch(out.branches, rp, color, sidesFor(rp.radius.front()), vScale, 3, &rootLook, false, footY, seed);
            out.limbs.push_back({baseP, rp.points.front(), baseR, rp.radius.front()});   // (the root's own skeleton)
            for (std::size_t q = 0; q + 1 < rp.points.size(); ++q)
                out.limbs.push_back({rp.points[q], rp.points[q + 1], rp.radius[q], rp.radius[q + 1]});
        }
    }

    own(-1);
    // Leaves on twig tips and (to fill out the ends) any thin enough branch.
    // Each card is jittered off the node into the surrounding gap, so clusters
    // fill the crown volume rather than sitting only on the skeleton. A hard
    // card budget keeps a single tree's triangle count bounded.
    // A PALM'S CROWN (foliage = 4; foliage_kit.h FrondSpec): fronds at the top of each stem, no leaves elsewhere
    if (params.foliage == 4 && params.fronds.set) {
        out.foliage = 4;
        for (const SkeletonNode& n : nodes) {
            if (!n.isTip) continue;
            const Vec3 up = n.parent >= 0 ? normalize(n.pos - nodes[n.parent].pos) : n.heading;
            addFrondCrown(params.fronds, n.pos, up, n.radius, seed ^ static_cast<uint32_t>(&n - nodes.data()), out.leaves, out.branches);
        }
    } else if (params.leaves) {
        std::uniform_real_distribution<float> tilt(-0.5f, 0.5f);
        std::uniform_real_distribution<float> vary(0.8f, 1.15f);
        std::uniform_real_distribution<float> off(-1.0f, 1.0f);
        int cards = 0;
        // THE PALETTE: a smooth noise over the crown walks each card's colour along it, so colour comes in patches
        // (a limb that has turned, its neighbour not yet), not card by card
        const std::vector<Vec3>& pal = params.leafPalette;
        auto leafColourAt = [&](const Vec3& at) -> Vec3 {
            if (pal.empty()) return params.leafColor;
            if (pal.size() == 1) return pal[0];
            const Vec3 q = at * (1.0 / std::max(0.2f, params.paletteScale));
            const float nz = 0.65f * gnarlNoise(q, seed ^ 0x9a1e77u) + 0.35f * gnarlNoise(q * 2.3, seed ^ 0x51c3u);
            const float g = std::clamp((nz - 0.5f) * 2.2f + 0.5f, 0.0f, 0.9999f) * static_cast<float>(pal.size() - 1);
            const int i = static_cast<int>(g);
            return pal[i] + (pal[i + 1] - pal[i]) * static_cast<double>(g - i);
        };
        // THE BUDGET thins the cards evenly over the crown: a tree that wants more than maxLeafCards keeps each card
        // at the same odds, not the first maxLeafCards in turtle order (which left the last-grown limbs -- the top --
        // bare)
        auto leafyNode = [&](const SkeletonNode& n) {
            return n.isTip || (params.leafThickness > 0.0f && n.radius <= params.leafThickness);
        };
        // SPRAYS (foliage_kit.h): the species' variants, laid along their twigs, the fan level -- their own count a
        // node and their own budget
        std::vector<RenderMesh> sprays;
        if (params.foliage == 1) sprays = buildSprays(params.spray, params.cardTiles, seed ^ 0x5b7au);
        out.foliage = sprays.empty() ? 0 : 1;
        const bool kit = params.foliage == 2 && params.leafKit.set;
        if (kit) out.foliage = 2;
        const int perNode = kit ? params.leafKit.perTwig : sprays.empty() ? params.leavesPerTip : params.spray.perNode;
        const int budget = kit ? params.leafKit.budget : sprays.empty() ? params.maxLeafCards : params.spray.budget;
        std::uniform_real_distribution<double> sprayRoll(-0.35, 0.35), sprayColour(0.0, 1.0);
        std::size_t wanted = 0, wantedBlossoms = 0;
        for (const SkeletonNode& n : nodes)
            if (leafyNode(n)) {
                wanted += static_cast<std::size_t>(std::max(0, kit ? params.leafKit.shoots * params.leafKit.perTwig / std::max(1, static_cast<int>(params.leafLod * params.leafLod)) : perNode));
                if (kit) wantedBlossoms += static_cast<std::size_t>(std::max(0, params.leafKit.shoots * params.leafKit.clusters * params.leafKit.perCluster));
            }
        const double blossomKeep = kit && wantedBlossoms > static_cast<std::size_t>(params.leafKit.blossomBudget)
                                       ? static_cast<double>(params.leafKit.blossomBudget) / static_cast<double>(wantedBlossoms) : 1.0;
        int blossoms = 0;
        const double keep = wanted > static_cast<std::size_t>(budget)
                                ? static_cast<double>(budget) / static_cast<double>(wanted) : 1.0;
        std::uniform_real_distribution<double> odds(0.0, 1.0);
        // (the leaf kit's shoots, thinned to whichever budget -- leaves or blossoms -- binds first)
        const double coarse = std::max(1.0, static_cast<double>(params.leafLod));
        const double shootKeep = std::min(keep, blossomKeep) / coarse;
        int pendingOwner = -1;
        for (const SkeletonNode& n : nodes) {
            if (!leafyNode(n)) continue;
            const int nSprig = sprigOf[static_cast<std::size_t>(&n - nodes.data())];
            // (a sprig's leaves are in its card; with cards, the wood's own are left off too -- a limb longer than a
            // card is bare inside, its foliage where its sprigs are, as a pine's limb is)
            if (!donor && (nSprig >= 0 || !out.sprigs.empty())) continue;
            // (what a node adds -- its leaves, its shoots' twigs -- is its sprig's: owned as the next node starts)
            own(pendingOwner);
            pendingOwner = nSprig;
            // the twig the cards hang on: this node's internode (parent to node), or for the root its heading
            const Vec3 a = n.parent >= 0 ? nodes[n.parent].pos : n.pos - n.heading * 0.1;
            Vec3 h = n.pos - a;
            const double segLen = h.length();
            h = segLen > 1e-9 ? h * (1.0 / segLen) : n.heading;
            const Vec3 ref = std::abs(h.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
            const Vec3 s1 = normalize(cross(h, ref)), s2 = normalize(cross(h, s1));
            std::uniform_real_distribution<float> along(1.0f - std::clamp(params.leafClump, 0.0f, 1.0f), 1.0f);
            // LEAFY SHOOTS (the leaf kit): a real tree has more orders of fine twigs than the skeleton grows -- each
            // leafy twig puts out a few thin SHOOTS, fanned off it and lifted toward the light, and the leaves (and a
            // sakura's blossom clusters) ride along them, alternate or in opposite pairs, a leaf at the tip
            if (kit) {
                const LeafKitSpec& lk = params.leafKit;
                for (int sh = 0; sh < lk.shoots; ++sh) {
                    if (cards >= budget) break;
                    if (shootKeep < 1.0 && odds(rng) >= shootKeep) continue;   // (and 1 / leafLod of them: coarse)
                    const double sroll = (params.phyllotaxis * sh) * PI / 180.0 + off(rng) * 0.6;
                    const Vec3 so = s1 * std::cos(sroll) + s2 * std::sin(sroll);
                    const double sang = (lk.shootAngle + tilt(rng) * 30.0) * PI / 180.0;
                    Vec3 sd = normalize(h * std::cos(sang) + so * std::sin(sang));
                    sd = normalize(sd + Vec3(0, lk.lift, 0));
                    const double slen = lk.shootLength * (0.7 + 0.6 * odds(rng));
                    const float u = along(rng);
                    const float rr = (n.parent >= 0 ? nodes[n.parent].radius * (1.0f - u) : 0.0f) + n.radius * u;
                    const Vec3 s0 = a + (n.pos - a) * u + so * (rr * 0.5);
                    // the shoot: a thin tapered twig, sagging a little under its leaves
                    BranchPath tw;
                    Vec3 tp = s0, td = sd;
                    for (int q = 0; q <= 2; ++q) {
                        tw.points.push_back(tp);
                        tw.radius.push_back(std::max(0.0015f, rr * 0.5f * (1.0f - 0.35f * q)));
                        tw.dist.push_back(n.distFromRoot + static_cast<float>(slen * q / 2.0));
                        td = normalize(td + Vec3(0, -1.5 * lk.sag, 0));
                        tp = tp + td * (slen / 2.0);
                    }
                    if (coarse < 1.5) addCurvedBranch(out.branches, tw, params.barkColor * vary(rng), 3, vScale, 1);
                    auto shootAt = [&](double t, Vec3& p, Vec3& d) {   // a point and heading along the shoot
                        const double x = std::clamp(t, 0.0, 1.0) * 2.0;
                        const int i = std::min(1, static_cast<int>(x));
                        p = tw.points[i] + (tw.points[i + 1] - tw.points[i]) * (x - i);
                        d = normalize(tw.points[i + 1] - tw.points[i]);
                    };
                    const Vec3 refs = std::abs(sd.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                    const Vec3 t1 = normalize(cross(sd, refs)), t2 = normalize(cross(sd, t1));
                    // its leaves
                    const int nl = std::max(1, static_cast<int>(std::lround(lk.perTwig / coarse)));
                    for (int j = 0; j < nl; ++j) {
                        const double tpos = lk.pairs ? 0.25 + 0.75 * ((j / 2) * 2.0 + 1.0) / nl : 0.2 + 0.8 * (j + 1.0) / nl;
                        const double kroll = lk.pairs ? (j / 2) * (PI * 0.5) + (j % 2) * PI + off(rng) * 0.25
                                                      : (params.phyllotaxis * j) * PI / 180.0 + off(rng) * 0.3;
                        Vec3 at, hd;
                        shootAt(tpos, at, hd);
                        const Vec3 ok = normalize((t1 * std::cos(kroll) + t2 * std::sin(kroll)) - hd * dot(t1 * std::cos(kroll) + t2 * std::sin(kroll), hd));
                        const double ang = (lk.angle + tilt(rng) * 20.0) * PI / 180.0;
                        Vec3 ld = normalize(hd * std::cos(ang) + ok * std::sin(ang));
                        ld = normalize(ld * (1.0 - lk.droop) + Vec3(0, -1, 0) * lk.droop);
                        const Vec3 hang = normalize(ok - ld * dot(ok, ld));
                        Vec3 skyward = Vec3(0, 1, 0) - ld * dot(Vec3(0, 1, 0), ld);
                        skyward = skyward.length() > 1e-6 ? normalize(skyward) : hang;
                        const Vec3 face = normalize(hang * (1.0 - lk.face) + skyward * lk.face);
                        std::uniform_int_distribution<int> leafTile(0, lk.blossomSize > 0.0 ? 1 : 3);
                        addKitLeaf(out.leaves, at, ld, face, lk.size * coarse * vary(rng), lk.fold, lk.curl * (0.7 + 0.6 * odds(rng)),
                                   2, leafTile(rng), leafColourAt(at) * vary(rng));
                        ++cards;
                    }
                    // its blossom clusters (a sakura): umbels on spurs along it, the cups opening outward and up
                    const int nc = std::max(1, static_cast<int>(std::lround(lk.clusters / coarse)));
                    for (int c = 0; c < nc && lk.blossomSize > 0.0; ++c) {
                        if (blossoms >= lk.blossomBudget) break;
                        Vec3 sp, hd;
                        shootAt(0.15 + 0.85 * (c + 0.5) / nc, sp, hd);
                        const double croll = (params.phyllotaxis * (c + 2)) * PI / 180.0 + off(rng) * 0.5;
                        const Vec3 oc = normalize(t1 * std::cos(croll) + t2 * std::sin(croll));
                        const Vec3 clusterAxis = normalize(oc + hd * 0.5 + Vec3(0, 0.5, 0));
                        const Vec3 ref2 = std::fabs(clusterAxis.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                        const Vec3 c1 = normalize(cross(clusterAxis, ref2)), c2 = cross(clusterAxis, c1);
                        for (int bl = 0; bl < lk.perCluster; ++bl) {
                            const double phi = 2.0 * PI * bl / lk.perCluster + off(rng) * 0.4;
                            const double spread = 0.5 + 0.35 * odds(rng);
                            const Vec3 stalk = normalize(clusterAxis * std::cos(spread) + (c1 * std::cos(phi) + c2 * std::sin(phi)) * std::sin(spread));
                            const double sz = lk.blossomSize * coarse * (0.85 + 0.3 * odds(rng));
                            const Vec3 centre = sp + stalk * (0.8 * sz);
                            std::uniform_int_distribution<int> flowerTile(2, 3);
                            addKitBlossom(out.leaves, centre, normalize(stalk + Vec3(0, 0.3, 0)), sz, lk.cup, odds(rng) * 2.0 * PI,
                                          2, flowerTile(rng), leafColourAt(centre) * vary(rng));
                            ++blossoms;
                        }
                    }
                }
                if (cards >= budget) break;
                continue;
            }
            for (int j = 0; j < perNode; j++) {
                if (cards >= budget) break;
                if (keep < 1.0 && odds(rng) >= keep) continue;
                // spiralled round the twig, each card's FOOT on the twig's bark somewhere along its last stretch, the
                // card tilted out from the twig by leafPitch (about the twig's own axis: a sideways limb's cards lie
                // along it, not across it)
                const double roll = (params.phyllotaxis * j) * PI / 180.0 + off(rng) * 0.4;
                const Vec3 o = s1 * std::cos(roll) + s2 * std::sin(roll);
                const double pitch = (params.leafPitch * PI / 180.0) + tilt(rng) * 0.6;
                const Vec3 dir = normalize(h * std::cos(pitch) + o * std::sin(pitch));
                const float u = along(rng);
                const float rad = (n.parent >= 0 ? nodes[n.parent].radius * (1.0f - u) : 0.0f) + n.radius * u;
                const Vec3 jpos = a + (n.pos - a) * u + o * (rad * 0.9);
                int tile = 0;
                if (params.cardTiles > 1) {
                    const int nt = params.cardTiles;
                    if (params.tipCards) {   // the first row at the tips, the rest along the stems
                        std::uniform_int_distribution<int> col(0, nt - 1), row(1, nt - 1);
                        tile = n.isTip ? col(rng) : row(rng) * nt + col(rng);
                    } else {
                        std::uniform_int_distribution<int> pick(0, nt * nt - 1);
                        tile = pick(rng);
                    }
                }
                if (!sprays.empty()) {
                    const RenderMesh& sp = sprays[static_cast<std::size_t>(tile + cards) % sprays.size()];
                    const Vec3 Y = dir;
                    Vec3 Z = Vec3(0, 1, 0) - Y * dot(Y, Vec3(0, 1, 0));
                    Z = Z.length() > 1e-3 ? normalize(Z) : normalize(cross(Y, Vec3(1, 0, 0)));
                    const Vec3 X0 = normalize(cross(Y, Z));
                    const double rr = sprayRoll(rng);   // the fan turned a little about its shoot
                    const Vec3 X = X0 * std::cos(rr) + Z * std::sin(rr), Zr = normalize(cross(X, Y));
                    const double sc = params.leafSize * vary(rng);
                    const Vec3 tint = params.leafPalette.empty()
                                          ? params.spray.color + (params.spray.colorHi - params.spray.color) * sprayColour(rng)
                                          : leafColourAt(jpos);
                    const uint32_t base = static_cast<uint32_t>(out.leaves.vertices.size());
                    for (const Vertex& v : sp.vertices) {
                        Vertex w = v;
                        w.position = jpos + (X * v.position.x + Y * v.position.y + Zr * v.position.z) * sc;
                        w.normal = normalize(X * v.normal.x + Y * v.normal.y + Zr * v.normal.z);
                        w.tangent = normalize(X * v.tangent.x + Y * v.tangent.y + Zr * v.tangent.z);
                        w.color = tint;
                        out.leaves.vertices.push_back(w);
                    }
                    for (uint32_t i : sp.indices) out.leaves.indices.push_back(base + i);
                } else {
                    addLeaf(out.leaves, jpos, dir, params.leafSize,
                            leafColourAt(jpos) * vary(rng), params.cardTiles, tile);
                }
                cards++;
            }
            if (cards >= budget) break;
        }
        own(pendingOwner);
    }
    own(-1);

    // A DONOR's sprigs, each into its own meshes (every triangle goes with its first vertex's owner)
    if (donor) {
        auto split = [&](const RenderMesh& src, const std::vector<int>& owner, bool bark) {
            std::vector<uint32_t> remap(src.vertices.size(), ~0u);
            for (std::size_t k = 0; k + 2 < src.indices.size(); k += 3) {
                const int o = owner[src.indices[k]];
                if (o < 0) continue;
                RenderMesh& dst = bark ? out.sprigs[static_cast<std::size_t>(o)].bark : out.sprigs[static_cast<std::size_t>(o)].leaves;
                for (int e = 0; e < 3; ++e) {
                    const uint32_t v = src.indices[k + e];
                    if (remap[v] == ~0u) {
                        remap[v] = static_cast<uint32_t>(dst.vertices.size());
                        dst.vertices.push_back(src.vertices[v]);
                    }
                    dst.indices.push_back(remap[v]);
                }
            }
        };
        split(out.branches, barkOwner, true);
        split(out.leaves, leafOwner, false);
    }

    for (const SkeletonNode& n : nodes)
        if (n.parent >= 0) out.limbs.push_back({nodes[n.parent].pos, n.pos, nodes[n.parent].radius, n.radius});

    // (the bark blends where the trunk zone meets itself: every vertex's tangent slot then its second UV and weight --
    // a tube's its own UV, weight 0 -- and the mesh says so: RenderMaterial::FLAG_UV_BLEND)
    if (!weldBlend.empty()) out.barkBlend = true;
    if (out.barkBlend) {
        for (std::size_t v = 0; v < out.branches.vertices.size(); ++v) {
            if (v >= out.zoneVertexStart && v < out.zoneVertexEnd) continue;   // (the zone's: its own pair)
            Vertex& vx = out.branches.vertices[v];
            vx.tangent = Vec3(vx.u, vx.v, 0.0);
        }
        for (const auto& [vi, data] : weldBlend) out.branches.vertices[vi].tangent = data;   // (a weld's: its parent's bark)
        out.branches.tangentIsData = true;
    }
    // Collision = the bark geometry (positions + indices), branches only.
    out.collisionVertices.reserve(out.branches.vertices.size());
    for (const Vertex& v : out.branches.vertices)
        out.collisionVertices.push_back(v.position);
    out.collisionIndices = out.branches.indices;

    return out;
}

// Grow a tree from the standard parametric grammar (the C++ convenience): build
// the grammar from `params`, expand it, then skin. Lua can instead author its
// own grammar and call skinTree directly (ADR-0030).
TreeMesh growTree(const TreeParams& params, uint32_t seed) {
    ParametricLSystem grammar = buildGrammar(params);
    ModuleString axiom = parseModuleLiterals("A(" + num(params.trunkLength) + ")");
    ModuleString s = grammar.expand(axiom, params.iterations, seed);
    return skinTree(s, params, seed);
}

// --- procedural textures ---------------------------------------------------

namespace {

// A cheap deterministic hash -> [0,1), and value noise from it.
float hash2(int x, int y, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u +
                 static_cast<uint32_t>(y) * 668265263u + seed * 362437u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (h ^ (h >> 16)) / 4294967296.0f;
}
float valueNoise(float x, float y, uint32_t seed) {
    int xi = (int)std::floor(x), yi = (int)std::floor(y);
    float fx = x - xi, fy = y - yi;
    float sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    float a = hash2(xi, yi, seed),       b = hash2(xi + 1, yi, seed);
    float c = hash2(xi, yi + 1, seed),   d = hash2(xi + 1, yi + 1, seed);
    return (a + (b - a) * sx) + ((c - a) + (d - c) * sx - (b - a) * sx) * sy;
}

float fbm(float x, float y, uint32_t seed, int octaves) {
    float n = 0.0f, amp = 0.5f, f = 1.0f;
    for (int o = 0; o < octaves; o++) {
        n += amp * valueNoise(x * f, y * f, seed + o * 17u);
        amp *= 0.5f; f *= 2.0f;
    }
    return n;
}
float ridged(float x, float y, uint32_t seed, int octaves) {
    return 1.0f - std::abs(fbm(x, y, seed, octaves) * 2.0f - 1.0f);
}

// Bark relief height in [0,1] per species. Furrows run vertically (texture V is
// along the branch length), so the across-trunk axis (U) carries the ridges.
float barkHeight(BarkStyle style, float u, float v, uint32_t seed) {
    if (style == BarkStyle::Birch) {
        // Smooth pale bark with short horizontal lenticel dashes + peeling bands.
        float base = 0.6f + 0.08f * fbm(u * 5.0f, v * 5.0f, seed, 3);
        float lent = fbm(u * 22.0f, v * 60.0f, seed + 3u, 2);   // stretched across U
        if (lent > 0.72f) base -= 0.45f;                        // dark lenticel
        float band = valueNoise(u * 1.5f, v * 9.0f, seed + 7u);
        if (band > 0.8f) base -= 0.12f;                         // peeling band
        return std::clamp(base, 0.0f, 1.0f);
    }
    if (style == BarkStyle::Pine) {
        // Blocky scale plates separated by cracks.
        float warp = fbm(u * 4.0f, v * 4.0f, seed + 5u, 2) * 0.4f;
        float plate = fbm(u * 6.0f + warp, v * 6.0f, seed, 2);
        float crack = ridged(u * 7.0f, v * 7.0f, seed + 2u, 2);
        return std::clamp(plate * 0.7f + 0.3f - (crack > 0.85f ? 0.5f : 0.0f),
                          0.0f, 1.0f);
    }
    // Oak: deep vertical furrows, wavy. High U-frequency ridges, slow V warp.
    float warp = fbm(u * 3.0f, v * 2.0f, seed + 5u, 2) * 0.35f;
    float furrow = ridged(u * 9.0f + warp, v * 1.6f, seed, 4);
    return std::clamp(furrow * 0.85f + 0.1f * fbm(u * 16.0f, v * 16.0f, seed + 9u, 2),
                      0.0f, 1.0f);
}

}  // namespace

TextureData barkTexture(int size, uint32_t seed) {
    TextureData t;
    t.width = t.height = size;
    t.channels = 3;
    t.pixels.resize((size_t)size * size * 3);
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float u = (float)x / size, v = (float)y / size;
            // Vertical fibres: stretched in v, several octaves; ridges in u.
            float n = 0.0f, amp = 0.5f, fu = 8.0f, fv = 2.0f;
            for (int o = 0; o < 4; o++) {
                n += amp * valueNoise(u * fu, v * fv, seed + o * 17u);
                amp *= 0.5f; fu *= 2.0f; fv *= 2.0f;
            }
            // Sharpen into bark ridges and keep it bright enough to modulate.
            float ridge = 0.55f + 0.45f * std::abs(std::sin((u * 6.2832f) * 3.0f + n * 4.0f));
            float g = std::min(1.0f, 0.5f * ridge + 0.6f * n);
            uint8_t b = (uint8_t)(std::max(0.4f, std::min(1.0f, g)) * 255.0f);
            size_t i = ((size_t)y * size + x) * 3;
            t.pixels[i] = t.pixels[i + 1] = t.pixels[i + 2] = b;
        }
    }
    return t;
}

TextureData leafTexture(int size) {
    TextureData t;
    t.width = t.height = size;
    t.channels = 4;
    t.pixels.assign((size_t)size * size * 4, 0);

    // A CLUSTER of many small pointed-oval leaves (not one leaf): so a single
    // card reads as a dense tuft of foliage. Density here is ~free (fixed texture
    // cost), unlike adding cards — so pack it full. Deterministic layout.
    struct Leaf { float cx, cy, ca, sa, halfLen, halfW, shade; };
    const int K = 34;
    std::vector<Leaf> leaves;
    uint32_t h = 0x9e3779b9u;
    auto rnd = [&]() {
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        return (h & 0xffffff) / static_cast<float>(0x1000000);
    };
    for (int i = 0; i < K; i++) {
        float ang = rnd() * 6.2831853f;
        Leaf lf;
        lf.cx = 0.5f + (rnd() - 0.5f) * 0.82f;
        lf.cy = 0.5f + (rnd() - 0.5f) * 0.92f;
        lf.ca = std::cos(ang); lf.sa = std::sin(ang);
        lf.halfLen = 0.15f + rnd() * 0.09f;
        lf.halfW = 0.045f + rnd() * 0.03f;
        lf.shade = 0.62f + rnd() * 0.38f;     // per-leaf value variation
        leaves.push_back(lf);
    }

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float u = (float)x / (size - 1), v = (float)y / (size - 1);
            float alpha = 0.0f, shade = 1.0f;
            for (const Leaf& lf : leaves) {
                float dx = u - lf.cx, dy = v - lf.cy;
                // into the leaf's local frame (lx along length, ly across width)
                float lx = dx * lf.ca + dy * lf.sa;
                float ly = -dx * lf.sa + dy * lf.ca;
                float ty = (lx + lf.halfLen) / (2.0f * lf.halfLen);   // 0..1 base->tip
                if (ty < 0.0f || ty > 1.0f) continue;
                float w = lf.halfW * std::sin(ty * 3.14159f);         // pointed oval
                if (std::abs(ly) <= w) {
                    alpha = 1.0f;
                    float vein = std::abs(ly) < w * 0.12f ? 0.65f : 1.0f;   // midrib
                    shade = lf.shade * vein;
                }
            }
            uint8_t c = (uint8_t)(std::min(1.0f, shade) * 255.0f);
            size_t i = ((size_t)y * size + x) * 4;
            t.pixels[i] = t.pixels[i + 1] = t.pixels[i + 2] = c;
            t.pixels[i + 3] = (uint8_t)(alpha * 255.0f);
        }
    }
    return t;
}

BarkStyle barkStyleFromName(const std::string& name) {
    if (name.find("birch") != std::string::npos) return BarkStyle::Birch;
    if (name.find("pine") != std::string::npos) return BarkStyle::Pine;
    return BarkStyle::Oak;
}

BarkMaps barkMaps(BarkStyle style, int size, uint32_t seed) {
    BarkMaps m;
    m.albedo.width = m.albedo.height = size;
    m.albedo.channels = 3;
    m.albedo.pixels.resize((size_t)size * size * 3);
    m.normal.width = m.normal.height = size;
    m.normal.channels = 3;
    m.normal.pixels.resize((size_t)size * size * 3);

    auto H = [&](int x, int y) {
        x = ((x % size) + size) % size;            // wrap (seamless tiling)
        y = ((y % size) + size) % size;
        return barkHeight(style, (float)x / size, (float)y / size, seed);
    };
    const float strength = size * 0.012f;          // bump intensity
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float h = H(x, y);
            // Albedo: a grayscale value pattern (furrows darker) that modulates
            // the bark vertex color — keep it bright enough to read as bark.
            float val = std::clamp(0.55f + 0.45f * h, 0.35f, 1.0f);
            size_t i = ((size_t)y * size + x) * 3;
            uint8_t a = (uint8_t)(val * 255.0f);
            m.albedo.pixels[i] = m.albedo.pixels[i + 1] = m.albedo.pixels[i + 2] = a;

            // Tangent-space normal from the height gradient (central difference).
            float dx = (H(x + 1, y) - H(x - 1, y)) * strength;
            float dy = (H(x, y + 1) - H(x, y - 1)) * strength;
            Vec3 n = normalize(Vec3(-dx, -dy, 1.0));
            m.normal.pixels[i + 0] = (uint8_t)((n.x * 0.5 + 0.5) * 255.0);
            m.normal.pixels[i + 1] = (uint8_t)((n.y * 0.5 + 0.5) * 255.0);
            m.normal.pixels[i + 2] = (uint8_t)((n.z * 0.5 + 0.5) * 255.0);
        }
    }
    return m;
}

}  // namespace engine
