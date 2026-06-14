#include "tree.h"
#include "lsystem.h"
#include "../../rt_math.h"

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
// only the branch bends away. Unexpanded apices left after `iterations` are the
// twig tips where leaves attach.
ParametricLSystem buildGrammar(const TreeParams& p) {
    const std::string f = num(p.lengthFalloff);
    const std::string roll = num(p.phyllotaxis);
    const std::string pitch = num(p.branchAngle);

    std::string succ = "F(l)";
    for (int i = 0; i < p.branchesPerNode; i++)
        succ += "/(" + roll + ")[&(" + pitch + ")A(l*" + f + ")]";
    succ += "/(" + roll + ")A(l*" + f + ")";   // central leader continues

    ParametricLSystem g;
    g.rule("A(l)", succ);
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

// --- generalized-cylinder skinning ----------------------------------------

// Two perpendiculars (right, up) to `axis` with cross(right, up) == -axis, so a
// ring built as right*cos + up*sin winds outward-front exactly like
// MeshBuilder::cylinder (engine winds front faces clockwise).
void frameFor(const Vec3& axis, Vec3& right, Vec3& up) {
    Vec3 ref = std::abs(axis.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    right = normalize(cross(axis, ref));
    up = normalize(cross(right, axis));
}

// Append one tapered tube segment from A (radius rA) to B (radius rB), with bark
// v running vA..vB. Mirrors the cylinder side-wall layout/winding.
void addTube(RenderMesh& mesh, const Vec3& A, const Vec3& B, float rA, float rB,
             float vA, float vB, const Vec3& color, int segs) {
    Vec3 axis = B - A;
    float len = static_cast<float>(axis.length());
    if (len < 1e-6f) return;
    axis = axis * (1.0f / len);
    Vec3 right, up;
    frameFor(axis, right, up);

    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    for (int i = 0; i <= segs; i++) {
        float phi = 2.0f * static_cast<float>(PI) * i / segs;
        float c = std::cos(phi), s = std::sin(phi);
        Vec3 dir = right * c + up * s;          // outward radial = normal
        Vec3 tan = right * (-s) + up * c;       // around the circumference
        float u = static_cast<float>(i) / segs;
        Vertex va(A + dir * rA, dir, tan, u, vA);
        Vertex vb(B + dir * rB, dir, tan, u, vB);
        va.color = color;
        vb.color = color;
        mesh.vertices.push_back(va);
        mesh.vertices.push_back(vb);
    }
    for (int i = 0; i < segs; i++) {
        uint32_t a = base + i * 2, b = a + 1, cc = a + 2, d = a + 3;
        mesh.indices.insert(mesh.indices.end(), {a, cc, b, b, cc, d});
    }
}

// --- leaf cards ------------------------------------------------------------

void addLeaf(RenderMesh& mesh, const Vec3& at, const Vec3& dir, float size,
             const Vec3& color) {
    Vec3 forward = normalize(dir);
    Vec3 ref = std::abs(forward.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 side = normalize(cross(forward, ref));
    Vec3 normal = normalize(cross(forward, side));
    float w = size * 0.5f, len = size * 1.7f;

    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 corners[4] = {at - side * w, at + side * w,
                       at + side * w + forward * len, at - side * w + forward * len};
    float uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    for (int i = 0; i < 4; i++) {
        Vertex v(corners[i], normal, side, uv[i][0], uv[i][1]);
        v.color = color;
        mesh.vertices.push_back(v);
    }
    // Double-sided so the card shows from either face.
    mesh.indices.insert(mesh.indices.end(),
                        {base, base + 1, base + 2, base, base + 2, base + 3,
                         base, base + 2, base + 1, base, base + 3, base + 2});
}

}  // namespace

TreeMesh growTree(const TreeParams& params, uint32_t seed) {
    std::mt19937 rng(seed);

    ParametricLSystem grammar = buildGrammar(params);
    ModuleString axiom = parseModuleLiterals("A(" + num(params.trunkLength) + ")");
    ModuleString s = grammar.expand(axiom, params.iterations, seed);

    std::vector<Node> nodes = walkSkeleton(s, params, rng);
    assignRadii(nodes, params);

    TreeMesh out;

    // Skin each edge (parent -> node) as a tapered tube.
    float vScale = params.barkVScale;
    for (size_t i = 1; i < nodes.size(); i++) {
        const Node& n = nodes[i];
        if (n.parent < 0) continue;
        const Node& par = nodes[n.parent];
        // Slight per-branch color variation so the bark isn't a flat slab.
        std::uniform_real_distribution<float> vary(0.85f, 1.0f);
        Vec3 color = params.barkColor * vary(rng);
        addTube(out.branches, par.pos, n.pos, par.radius, n.radius,
                par.distFromRoot * vScale, n.distFromRoot * vScale, color,
                params.ringSegments);
    }

    // Leaves at twig tips.
    if (params.leaves) {
        std::uniform_real_distribution<float> tilt(-0.5f, 0.5f);
        std::uniform_real_distribution<float> vary(0.8f, 1.15f);
        for (const Node& n : nodes) {
            if (!n.isTip) continue;
            for (int j = 0; j < params.leavesPerTip; j++) {
                // Spiral the leaves around the heading and pitch them outward.
                double roll = (params.phyllotaxis * j) * PI / 180.0;
                Mat4 r = Mat4::rotateY(roll) * Mat4::rotateX((0.7 + tilt(rng)));
                Vec3 dir = r.transformDirection(n.heading);
                addLeaf(out.leaves, n.pos, dir, params.leafSize,
                        params.leafColor * vary(rng));
            }
        }
    }

    // Collision = the bark geometry (positions + indices), branches only.
    out.collisionVertices.reserve(out.branches.vertices.size());
    for (const Vertex& v : out.branches.vertices)
        out.collisionVertices.push_back(v.position);
    out.collisionIndices = out.branches.indices;

    return out;
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
    t.pixels.resize((size_t)size * size * 4);
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float u = (float)x / (size - 1), v = (float)y / (size - 1);
            // Pointed-oval leaf: half-width tapers to 0 at the tip and base.
            float halfW = 0.42f * std::sin(v * 3.14159f);
            float dist = std::abs(u - 0.5f);
            float alpha = dist < halfW ? 1.0f : 0.0f;
            // A faint central vein darkens the RGB a touch (still mostly white).
            float vein = dist < 0.03f ? 0.7f : 1.0f;
            uint8_t c = (uint8_t)(vein * 255.0f);
            size_t i = ((size_t)y * size + x) * 4;
            t.pixels[i] = t.pixels[i + 1] = t.pixels[i + 2] = c;
            t.pixels[i + 3] = (uint8_t)(alpha * 255.0f);
        }
    }
    return t;
}

}  // namespace engine
