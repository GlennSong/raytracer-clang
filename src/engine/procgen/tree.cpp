#include "tree.h"
#include "lsystem.h"
#include "../../rt_math.h"
#include "../../curve.h"

#include <algorithm>
#include <cmath>
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

struct Node {
    Vec3 pos;
    Vec3 heading{0, 1, 0};   // turtle heading at this node (for leaf orientation)
    int  parent = -1;
    int  childCount = 0;
    float radius = 0.0f;     // filled by the pipe model
    float distFromRoot = 0.0f;
    bool isTip = false;
};

// Walk the expanded module string into a node tree, applying angle jitter from
// the seeded RNG (so each tree differs but reproducibly).
std::vector<Node> walkSkeleton(const ModuleString& s, const TreeParams& p,
                               std::mt19937& rng) {
    std::vector<Node> nodes;
    nodes.push_back(Node{});   // root at origin, heading +Y

    struct State { int node; Mat4 orient; };
    State st{0, Mat4::identity()};
    std::vector<State> stack;

    std::uniform_real_distribution<float> jit(-p.angleJitter, p.angleJitter);
    auto turn = [&](char axis, float deg) {
        double a = (deg + jit(rng)) * PI / 180.0;
        switch (axis) {
            case 'Z': st.orient = st.orient * Mat4::rotateZ(a); break;
            case 'X': st.orient = st.orient * Mat4::rotateX(a); break;
            case 'Y': st.orient = st.orient * Mat4::rotateY(a); break;
        }
    };

    for (const Module& m : s) {
        float arg = m.params.empty() ? 0.0f : m.params[0];
        switch (m.symbol) {
            case 'F': {
                Vec3 heading = st.orient.transformDirection(Vec3(0, 1, 0));
                Node n;
                n.parent = st.node;
                n.pos = nodes[st.node].pos + heading * arg;
                n.heading = heading;
                n.distFromRoot = nodes[st.node].distFromRoot + arg;
                nodes[st.node].childCount++;
                st.node = static_cast<int>(nodes.size());
                nodes.push_back(n);
                break;
            }
            case 'A':   // an unexpanded apex => a twig tip (leaf attachment)
            case 'L':
                nodes[st.node].isTip = true;
                break;
            case '+': turn('Z',  arg); break;
            case '-': turn('Z', -arg); break;
            case '&': turn('X',  arg); break;
            case '^': turn('X', -arg); break;
            case '/': turn('Y',  arg); break;
            case '\\': turn('Y', -arg); break;
            case '[': stack.push_back(st); break;
            case ']': if (!stack.empty()) { st = stack.back(); stack.pop_back(); } break;
            default: break;
        }
    }
    return nodes;
}

// Pipe model: a node carrying no children gets the tip radius; an internal node
// combines its children's cross-sections, r = (sum r_child^n)^(1/n). Nodes are
// in DFS pre-order, so a reverse pass sees every child before its parent.
void assignRadii(std::vector<Node>& nodes, const TreeParams& p) {
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

// Organic shaping: bend branches under gravity (droop) and add a lateral
// meander (wander), thin branches bending most. The key is that the bend
// *accumulates* down a limb — each apical segment bends relative to its parent's
// already-bent direction, so a straight rod of skeleton nodes curls into a
// cantilever arc (more sag toward the tip), not just a uniform tilt. At a fork a
// lateral child bends from its *own* direction, preserving the branch angle.
// Runs in parent-before-child order, so children inherit the bent positions and
// joints stay connected; the thick ~vertical trunk (thin~0) barely moves.
void shapeBranches(std::vector<Node>& nodes, const TreeParams& p, std::mt19937& rng) {
    if (nodes.empty() || (p.droop <= 0.0f && p.wander <= 0.0f)) return;

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
        Vec3 bent = normalize(base + Vec3(0, -1, 0) * static_cast<double>(p.droop * thin));
        Vec3 ref = std::abs(bent.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        Vec3 side = normalize(cross(bent, ref));
        Vec3 up = normalize(cross(side, bent));
        Vec3 jitter = (side * jit(rng) + up * jit(rng)) *
                      static_cast<double>(p.wander * thin);
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

// Skin a branch as a continuous generalized cylinder: fit a Catmull-Rom curve
// through the centerline, sample it at even arc length (so curvature shows),
// orient each ring with a rotation-minimizing frame (no twist), and sweep a
// tapered ring strip with bark UVs flowing along the length. Ring winding/normal
// layout matches MeshBuilder::cylinder.
void addCurvedBranch(RenderMesh& mesh, const BranchPath& path, const Vec3& color,
                     int segs, float vScale, int ringsPerSegment) {
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

    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int k = 0; k < rings; k++) {
        const Vec3 right = frames[k].right;
        const Vec3 up = frames[k].up;
        float rad = lerpAt(path.radius, us[k]);
        float v = lerpAt(path.dist, us[k]) * vScale;
        for (int i = 0; i <= segs; i++) {
            float phi = 2.0f * static_cast<float>(PI) * i / segs;
            float c = std::cos(phi), s = std::sin(phi);
            Vec3 dir = right * c + up * s;        // outward radial = normal
            Vec3 tan = right * (-s) + up * c;     // around the circumference
            Vertex vert(centers[k] + dir * rad, dir, tan,
                        static_cast<float>(i) / segs, v);
            vert.color = color;
            mesh.vertices.push_back(vert);
        }
    }
    const int stride = segs + 1;
    for (int k = 0; k < rings - 1; k++) {
        for (int i = 0; i < segs; i++) {
            uint32_t a = base + k * stride + i;            // ring k, vert i
            uint32_t cc = a + 1;                           // ring k, vert i+1
            uint32_t b = base + (k + 1) * stride + i;      // ring k+1, vert i
            uint32_t d = b + 1;
            // Wound so front faces point along the outward radial (= the vertex
            // normal). The RMF frame has cross(right, up) == +tangent, the
            // opposite chirality to the old per-segment frame, so the winding is
            // reversed from a plain cylinder to keep faces facing out.
            mesh.indices.insert(mesh.indices.end(), {a, b, cc, cc, b, d});
        }
    }
}

// --- leaf cards ------------------------------------------------------------

// One double-sided card from `at` along `forward` for `len`, `w` wide along
// `side`, shaded with `normal`.
void addLeafQuad(RenderMesh& mesh, const Vec3& at, const Vec3& forward,
                 const Vec3& side, const Vec3& normal, float w, float len,
                 const Vec3& color) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 corners[4] = {at - side * w, at + side * w,
                       at + side * w + forward * len, at - side * w + forward * len};
    float uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    for (int i = 0; i < 4; i++) {
        Vertex v(corners[i], normal, side, uv[i][0], uv[i][1]);
        v.color = color;
        mesh.vertices.push_back(v);
    }
    mesh.indices.insert(mesh.indices.end(),
                        {base, base + 1, base + 2, base, base + 2, base + 3,
                         base, base + 2, base + 1, base, base + 3, base + 2});
}

// A leaf-cluster card: two perpendicular crossed quads, so a cluster has volume
// from any viewing angle instead of vanishing edge-on. The texture is a clump of
// many leaves, so one cross reads as a dense tuft.
void addLeaf(RenderMesh& mesh, const Vec3& at, const Vec3& dir, float size,
             const Vec3& color) {
    Vec3 forward = normalize(dir);
    Vec3 ref = std::abs(forward.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 side = normalize(cross(forward, ref));
    Vec3 normal = normalize(cross(forward, side));
    float w = size * 0.5f, len = size * 1.7f;
    addLeafQuad(mesh, at, forward, side, normal, w, len, color);
    addLeafQuad(mesh, at, forward, normal, side, w, len, color);   // crossed 90°
}

}  // namespace

TreeMesh skinTree(const ModuleString& s, const TreeParams& params, uint32_t seed) {
    std::mt19937 rng(seed);

    std::vector<Node> nodes = walkSkeleton(s, params, rng);
    assignRadii(nodes, params);
    shapeBranches(nodes, params, rng);

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
    std::vector<int> cont(nodes.size(), -1);
    for (size_t n = 0; n < nodes.size(); n++) {
        Vec3 inDir = incomingDir(static_cast<int>(n));
        double best = -2.0;
        for (int c : children[n]) {
            Vec3 d = nodes[c].pos - nodes[n].pos;
            double L = d.length();
            if (L < 1e-9) continue;
            double dp = dot(d * (1.0 / L), inDir);
            if (dp > best) { best = dp; cont[n] = c; }
        }
    }

    // Walk chains from a worklist of (anchor, first) starts; the anchor is the
    // fork point, included so the curve passes through the joint.
    std::vector<std::pair<int, int>> work;
    auto pushStarts = [&](int node) {
        for (int c : children[node])
            if (c != cont[node]) work.push_back({node, c});
    };
    if (cont[0] != -1) work.push_back({0, cont[0]});
    pushStarts(0);

    float vScale = params.barkVScale;
    std::uniform_real_distribution<float> vary(0.85f, 1.0f);
    while (!work.empty()) {
        auto [anchor, first] = work.back();
        work.pop_back();
        BranchPath path;
        auto append = [&](int idx) {
            path.points.push_back(nodes[idx].pos);
            path.radius.push_back(nodes[idx].radius);
            path.dist.push_back(nodes[idx].distFromRoot);
        };
        append(anchor);
        for (int cur = first; cur != -1; cur = cont[cur]) {
            append(cur);
            pushStarts(cur);
        }
        Vec3 color = params.barkColor * vary(rng);
        addCurvedBranch(out.branches, path, color, params.ringSegments, vScale, 3);
    }

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
            addCurvedBranch(out.branches, rp, color, params.ringSegments, vScale, 3);
        }
    }

    // Leaves on twig tips and (to fill out the ends) any thin enough branch.
    // Each card is jittered off the node into the surrounding gap, so clusters
    // fill the crown volume rather than sitting only on the skeleton. A hard
    // card budget keeps a single tree's triangle count bounded.
    if (params.leaves) {
        std::uniform_real_distribution<float> tilt(-0.5f, 0.5f);
        std::uniform_real_distribution<float> vary(0.8f, 1.15f);
        std::uniform_real_distribution<float> off(-1.0f, 1.0f);
        int cards = 0;
        for (const Node& n : nodes) {
            bool leafy = n.isTip ||
                         (params.leafThickness > 0.0f && n.radius <= params.leafThickness);
            if (!leafy) continue;
            for (int j = 0; j < params.leavesPerTip; j++) {
                if (cards >= params.maxLeafCards) break;
                // Spiral the leaves around the heading and pitch them outward.
                double roll = (params.phyllotaxis * j) * PI / 180.0;
                Mat4 r = Mat4::rotateY(roll) * Mat4::rotateX((0.7 + tilt(rng)));
                Vec3 dir = r.transformDirection(n.heading);
                Vec3 jpos = n.pos + Vec3(off(rng), off(rng), off(rng)) *
                                        (params.leafSize * params.leafClump);
                addLeaf(out.leaves, jpos, dir, params.leafSize,
                        params.leafColor * vary(rng));
                cards++;
            }
            if (cards >= params.maxLeafCards) break;
        }
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
