#include "tree.h"
#include "../mesh_builder.h"
#include "lsystem.h"
#include "skeleton.h"
#include "../../rt_math.h"
#include "../../curve.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
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
                     const Socket* sock = nullptr, bool pointTip = false) {
    if (path.points.size() < 2) return;
    Spline<Vec3> spline = Spline<Vec3>::catmullRom(path.points);
    int rings = std::max(2, spline.segments() * std::max(1, ringsPerSegment) + 1);

    std::vector<double> us = spline.uniformArcLengthParams(rings);
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
            vert.color = color;
            if (g && (g->barkMottle > 0.0f || g->barkFootDark > 0.0f)) {   // BARK VARIETY: patches, a dark foot
                const Vec3 p = centers[k] + dir * r;
                float m = 1.0f;
                if (g->barkMottle > 0.0f) {   // patches with soft edges, two sizes
                    const float n2 = 0.6f * gnarlNoise(p * 1.1, seed ^ 0x3b1du) + 0.4f * gnarlNoise(p * 3.7, seed ^ 0x51d7u);
                    const float t = std::clamp((n2 - 0.52f) / 0.10f, 0.0f, 1.0f);
                    m -= g->barkMottle * t * t * (3.0f - 2.0f * t);
                }
                Vec3 c = color * m;
                if (g->barkFootDark > 0.0f) {
                    const float hy = static_cast<float>(p.y) - footY;
                    // the dark base rises unevenly round the trunk, in tongues
                    const float reach = g->barkFootHeight * (0.6f + 0.8f * gnarlNoise(Vec3(p.x * 3.0, 0.0, p.z * 3.0), seed ^ 0x77u));
                    // (whole up to two fifths of its reach, fading out above)
                    const float w = g->barkFootDark * std::clamp((reach - hy) / std::max(0.05f, 0.6f * reach), 0.0f, 1.0f);
                    c = c * (1.0f - w) + g->barkFootColor * w;
                }
                vert.color = c;
            }
            mesh.vertices.push_back(vert);
        }
    }
    const int stride = segs + 1;
    if (k0 > 0) {
        // the contact ring: ring k0 carried back along the branch onto the parent (a hair inside it, so the parent's
        // knots and lobes do not open a gap); the rings before it (inside the parent) dropped
        const Vec3 T = normalize(tangents[k0]);
        const float R = sock->r * 0.94f;
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
    if (params.joints == 1) {
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
    if (cont[0] != -1 && !isJoint[0]) work.push_back({0, cont[0]});
    pushStarts(0);
    // each joint's end rings, as the limbs leave them: which limb's tube, and which end of it
    struct Tube { uint32_t vBase = 0, rings = 0; std::size_t iBase = 0, iEnd = 0; int startBack = 0, endBack = 0; };
    struct EndRing { int tube = -1; bool start = true; };
    std::vector<Tube> tubes;
    std::vector<std::vector<EndRing>> jointRings(nodes.size());
    const uint32_t ringStride = static_cast<uint32_t>(params.ringSegments + 1);

    float vScale = params.barkVScale;
    std::uniform_real_distribution<float> vary(0.85f, 1.0f);
    const float footY = nodes.empty() ? 0.0f : static_cast<float>(nodes[0].pos.y);
    const int trunkFirst = cont.empty() ? -1 : cont[0];
    while (!work.empty()) {
        auto [anchor, first] = work.back();
        work.pop_back();
        const bool trunk = anchor == 0 && first == trunkFirst;
        BranchPath path;
        auto append = [&](int idx) {
            path.points.push_back(nodes[idx].pos);
            path.radius.push_back(nodes[idx].radius);
            path.dist.push_back(nodes[idx].distFromRoot);
        };
        const bool cut = isJoint[anchor] && cutStart[first];
        int endsAt = -1, lastNode = -1;
        if (cut) {   // out from the joint by its cut-back
            const Vec3 u = normalize(nodes[first].pos - nodes[anchor].pos);
            path.points.push_back(nodes[anchor].pos + u * cutOut[first]);
            path.radius.push_back(nodes[first].radius * 1.05f);
            path.dist.push_back(nodes[anchor].distFromRoot + static_cast<float>(cutOut[first]));
        } else {
            append(anchor);
        }
        for (int cur = first; cur != -1; cur = cont[cur]) {
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
        const bool pointTip = params.foliage != 0 && endsAt < 0 && lastNode >= 0 && children[lastNode].empty();
        Vec3 color = params.barkColor * vary(rng);
        Socket sock;
        const Socket* sp = nullptr;
        if (!trunk && !cut && params.collar > 0.0f) {   // a side branch: welded into its parent at the fork
            const int par = nodes[anchor].parent;
            const Vec3 dIn = par >= 0 ? normalize(nodes[anchor].pos - nodes[par].pos) : nodes[anchor].heading;
            const Vec3 dOut = cont[anchor] >= 0 ? normalize(nodes[cont[anchor]].pos - nodes[anchor].pos) : dIn;
            sock.c = nodes[anchor].pos;
            sock.d = equalFork[anchor] ? dIn : normalize(dIn + dOut);   // (a crotch: the stub runs on along the limb)
            sock.r = nodes[anchor].radius;
            sock.collar = params.collar;
            sp = &sock;
        }
        if (std::getenv("RT_JOINT_DEBUG") && endsAt >= 0 && !cut && !trunk)
            std::fprintf(stderr, "[joint] limb into joint %d starts welded at node %d (joint %d, %zu children, r %.3f, first %d r %.3f, stub %d)\n",
                         endsAt, anchor, isJoint[anchor], children[anchor].size(), nodes[anchor].radius, first,
                         nodes[first].radius, equalFork[anchor]);
        Tube tb;
        tb.vBase = static_cast<uint32_t>(out.branches.vertices.size());
        tb.iBase = out.branches.indices.size();
        addCurvedBranch(out.branches, path, color, params.ringSegments, vScale, 3, &params, trunk, footY, seed, sp, pointTip);
        tb.rings = (static_cast<uint32_t>(out.branches.vertices.size()) - tb.vBase) / ringStride;
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
    const int segs = params.ringSegments;
    const std::size_t jointIdxStart = out.branches.indices.size();
    auto ringStart = [&](const EndRing& e) {
        const Tube& t = tubes[static_cast<std::size_t>(e.tube)];
        const uint32_t k = e.start ? static_cast<uint32_t>(t.startBack) : t.rings - 1 - static_cast<uint32_t>(t.endBack);
        return t.vBase + k * ringStride;
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
        bool whole = false;
        for (int attempt = 0; attempt < 8 && !whole; ++attempt) {
            pts.clear(); ringOf.clear(); idxOf.clear(); keep.clear();
            std::vector<Vec3> cen(ends.size()), nrmOf(ends.size());
            std::vector<double> rhoOf(ends.size(), 0.0);
            for (std::size_t a = 0; a < ends.size(); ++a) {
                rings[a] = ringStart(ends[a]);
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
                for (int i = 0; i < segs && whole; ++i) {
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
                    for (int i = 0; i < segs && !crossed; ++i)
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
            struct Cap { Vec3 a, b; double r, k; };
            std::vector<Cap> caps;
            double rhoMin = 1e9;
            for (std::size_t a = 0; a < ends.size(); ++a) {
                Vec3 cen(0, 0, 0);
                for (int i = 0; i < segs; ++i) cen = cen + out.branches.vertices[rings[a] + i].position;
                cen = cen * (1.0 / segs);
                double rho = 0.0;
                for (int i = 0; i < segs; ++i) rho += (out.branches.vertices[rings[a] + i].position - cen).length();
                rho /= segs;
                rhoMin = std::min(rhoMin, rho);
                const Vec3 ax = normalize(cen - C);
                caps.push_back({C, cen + ax * (0.5 * rho), rho, 0.2 * rho});
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
                    for (int i = 0; i < segs; ++i) {
                        real.push_back(out.branches.vertices[rings[a] + i].position);
                        realIdx.push_back(rings[a] + i);
                        realRing.push_back(static_cast<int>(a));
                    }
                for (const auto& f : MeshBuilder::convexHull(real))
                    if (!(realRing[f[0]] == realRing[f[1]] && realRing[f[1]] == realRing[f[2]]))
                        tris.push_back({realIdx[f[0]], realIdx[f[1]], realIdx[f[2]]});
            }
            auto ringEdge = [&](uint32_t x, uint32_t y) {   // two neighbours round one ring: never split
                for (uint32_t r : rings) {
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
            auto targetAt = [&](const Vec3& p) {
                double best = 1e9;
                for (const Cap& c : caps) {
                    const Vec3 ab = c.b - c.a;
                    const double t = std::clamp(dot(p - c.a, ab) / std::max(1e-12, dot(ab, ab)), 0.0, 1.0);
                    const double d = std::max(0.0, (p - (c.a + ab * t)).length() - c.r);
                    best = std::min(best, 2.0 * PI * c.r / segs * 1.7 + kGrow * d);
                }
                return std::max(best, kFloor * 2.0 * PI * caps.front().r / segs * 1.7);   // (no finer than this share of the joint's own)
            };
            const std::size_t firstNew = out.branches.vertices.size();
            for (int level = 0; level < 4; ++level) {
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
                for (std::size_t v = firstNew; v < out.branches.vertices.size(); ++v) {
                    Vec3 p = out.branches.vertices[v].position;
                    for (int it = 0; it < 6; ++it) {
                        const double fv = field(p);
                        const Vec3 g = grad(p);
                        const double g2 = dot(g, g);
                        if (g2 < 1e-12 || std::fabs(fv) < 1e-5 * rhoMin) break;
                        Vec3 step = g * (fv / g2);
                        if (step.length() > 0.5 * rhoMin) step = normalize(step) * (0.5 * rhoMin);
                        p = p - step;
                    }
                    out.branches.vertices[v].position = p;
                }
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
                    const uint32_t ring = (out.branches.indices[k + e] - t.vBase) / ringStride;
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
    // the ground (local y~0). Widens the flare and grounds the tree.
    if (params.rootCount > 0 && !nodes.empty()) {
        float baseR = std::max(nodes[0].radius, params.tipRadius * params.radiusScale);
        Vec3 baseP = nodes[0].pos;
        float rootLen = baseR * params.rootSpread;
        float tipR = params.tipRadius * params.radiusScale;
        std::uniform_real_distribution<float> jit(-0.3f, 0.3f);
        std::uniform_real_distribution<float> lenv(0.7f, 1.2f);
        std::uniform_real_distribution<float> vary(0.85f, 1.0f);
        for (int k = 0; k < params.rootCount; k++) {
            float ang = (params.phyllotaxis * k) * static_cast<float>(PI) / 180.0f + jit(rng);
            Vec3 dir(std::cos(ang), 0.0f, std::sin(ang));
            float L = rootLen * lenv(rng);
            BranchPath rp;
            auto add = [&](const Vec3& p, float r) {
                rp.points.push_back(p);
                rp.radius.push_back(r);
                rp.dist.push_back(static_cast<float>((p - baseP).length()));
            };
            add(baseP + dir * (baseR * 0.3f) + Vec3(0, baseR * 0.7f, 0), baseR * 0.55f);
            add(baseP + dir * (baseR * 1.5f) + Vec3(0, baseR * 0.15f, 0), baseR * 0.4f);
            add(baseP + dir * (L * 0.6f), tipR * 2.0f);
            add(baseP + dir * L + Vec3(0, -0.02f, 0), tipR);
            Vec3 color = params.barkColor * vary(rng);
            // (the bark's variety only -- the dark foot and patches -- not the gnarl, which is for limbs)
            TreeParams rootLook;
            rootLook.barkMottle = params.barkMottle;
            rootLook.barkFootDark = params.barkFootDark;
            rootLook.barkFootHeight = params.barkFootHeight;
            rootLook.barkFootColor = params.barkFootColor;
            addCurvedBranch(out.branches, rp, color, params.ringSegments, vScale, 3, &rootLook, false, footY, seed);
            out.limbs.push_back({baseP, rp.points.front(), baseR, rp.radius.front()});   // (the root's own skeleton)
            for (std::size_t q = 0; q + 1 < rp.points.size(); ++q)
                out.limbs.push_back({rp.points[q], rp.points[q + 1], rp.radius[q], rp.radius[q + 1]});
        }
    }

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
                wanted += static_cast<std::size_t>(std::max(0, kit ? params.leafKit.shoots * params.leafKit.perTwig : perNode));
                if (kit) wantedBlossoms += static_cast<std::size_t>(std::max(0, params.leafKit.shoots * params.leafKit.clusters * params.leafKit.perCluster));
            }
        const double blossomKeep = kit && wantedBlossoms > static_cast<std::size_t>(params.leafKit.blossomBudget)
                                       ? static_cast<double>(params.leafKit.blossomBudget) / static_cast<double>(wantedBlossoms) : 1.0;
        int blossoms = 0;
        const double keep = wanted > static_cast<std::size_t>(budget)
                                ? static_cast<double>(budget) / static_cast<double>(wanted) : 1.0;
        std::uniform_real_distribution<double> odds(0.0, 1.0);
        // (the leaf kit's shoots, thinned to whichever budget -- leaves or blossoms -- binds first)
        const double shootKeep = std::min(keep, blossomKeep);
        for (const SkeletonNode& n : nodes) {
            if (!leafyNode(n)) continue;
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
                    if (shootKeep < 1.0 && odds(rng) >= shootKeep) continue;
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
                    addCurvedBranch(out.branches, tw, params.barkColor * vary(rng), 3, vScale, 1);
                    auto shootAt = [&](double t, Vec3& p, Vec3& d) {   // a point and heading along the shoot
                        const double x = std::clamp(t, 0.0, 1.0) * 2.0;
                        const int i = std::min(1, static_cast<int>(x));
                        p = tw.points[i] + (tw.points[i + 1] - tw.points[i]) * (x - i);
                        d = normalize(tw.points[i + 1] - tw.points[i]);
                    };
                    const Vec3 refs = std::abs(sd.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                    const Vec3 t1 = normalize(cross(sd, refs)), t2 = normalize(cross(sd, t1));
                    // its leaves
                    const int nl = std::max(1, lk.perTwig);
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
                        addKitLeaf(out.leaves, at, ld, face, lk.size * vary(rng), lk.fold, lk.curl * (0.7 + 0.6 * odds(rng)),
                                   2, leafTile(rng), leafColourAt(at) * vary(rng));
                        ++cards;
                    }
                    // its blossom clusters (a sakura): umbels on spurs along it, the cups opening outward and up
                    for (int c = 0; c < lk.clusters && lk.blossomSize > 0.0; ++c) {
                        if (blossoms >= lk.blossomBudget) break;
                        Vec3 sp, hd;
                        shootAt(0.15 + 0.85 * (c + 0.5) / lk.clusters, sp, hd);
                        const double croll = (params.phyllotaxis * (c + 2)) * PI / 180.0 + off(rng) * 0.5;
                        const Vec3 oc = normalize(t1 * std::cos(croll) + t2 * std::sin(croll));
                        const Vec3 clusterAxis = normalize(oc + hd * 0.5 + Vec3(0, 0.5, 0));
                        const Vec3 ref2 = std::fabs(clusterAxis.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
                        const Vec3 c1 = normalize(cross(clusterAxis, ref2)), c2 = cross(clusterAxis, c1);
                        for (int bl = 0; bl < lk.perCluster; ++bl) {
                            const double phi = 2.0 * PI * bl / lk.perCluster + off(rng) * 0.4;
                            const double spread = 0.5 + 0.35 * odds(rng);
                            const Vec3 stalk = normalize(clusterAxis * std::cos(spread) + (c1 * std::cos(phi) + c2 * std::sin(phi)) * std::sin(spread));
                            const double sz = lk.blossomSize * (0.85 + 0.3 * odds(rng));
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
    }

    for (const SkeletonNode& n : nodes)
        if (n.parent >= 0) out.limbs.push_back({nodes[n.parent].pos, n.pos, nodes[n.parent].radius, n.radius});

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
