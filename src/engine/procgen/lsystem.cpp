#include "lsystem.h"
#include "../mesh_builder.h"
#include "sdf.h"
#include "../../rt_math.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <random>
#include <sstream>
#include <vector>

namespace engine {

void LSystem::rule(char symbol, const std::string& replacement, double weight) {
    rules[symbol].push_back({replacement, weight});
}

std::string LSystem::expand(const std::string& axiom, int iterations,
                            uint32_t seed) const {
    std::mt19937 gen(seed);
    std::string current = axiom;
    for (int it = 0; it < iterations; it++) {
        std::string next;
        next.reserve(current.size() * 2);
        for (char c : current) {
            auto rule = rules.find(c);
            if (rule == rules.end() || rule->second.empty()) {
                next += c;                       // no production: copy verbatim
                continue;
            }
            const std::vector<Production>& prods = rule->second;
            if (prods.size() == 1) {
                next += prods[0].text;           // deterministic
                continue;
            }
            // Weighted random pick (seeded, so deterministic per seed).
            double total = 0.0;
            for (const Production& p : prods) total += p.weight;
            double pick = std::uniform_real_distribution<double>(0.0, total)(gen);
            for (const Production& p : prods) {
                pick -= p.weight;
                if (pick <= 0.0) { next += p.text; break; }
            }
        }
        current.swap(next);
    }
    return current;
}

// ---------------------------------------------------------------------------
// Parametric L-system

namespace {

// A tiny recursive-descent compiler for successor-parameter expressions. It
// turns "l*0.7" into a ParamExpr closure over the bound predecessor params
// (looked up by formal name -> index in `vars`). Lenient: a malformed
// expression sets ok=false and evaluates to 0 rather than throwing, which suits
// a generation engine (a bad grammar yields a degenerate plant, not a crash).
struct ExprCompiler {
    const std::string& s;
    const std::unordered_map<std::string, int>& vars;
    size_t i = 0;
    bool ok = true;

    void skip() { while (i < s.size() && std::isspace((unsigned char)s[i])) i++; }
    char peek() { skip(); return i < s.size() ? s[i] : '\0'; }

    ParamExpr expr() {                       // term (('+'|'-') term)*
        ParamExpr left = term();
        for (;;) {
            char c = peek();
            if (c != '+' && c != '-') break;
            i++;
            ParamExpr right = term();
            left = [c, left, right](const std::vector<float>& b) {
                return c == '+' ? left(b) + right(b) : left(b) - right(b);
            };
        }
        return left;
    }
    ParamExpr term() {                       // factor (('*'|'/') factor)*
        ParamExpr left = factor();
        for (;;) {
            char c = peek();
            if (c != '*' && c != '/') break;
            i++;
            ParamExpr right = factor();
            left = [c, left, right](const std::vector<float>& b) {
                float r = right(b);
                if (c == '*') return left(b) * r;
                return r != 0.0f ? left(b) / r : 0.0f;
            };
        }
        return left;
    }
    ParamExpr factor() {                     // '-'factor | '('expr')' | num | var
        char c = peek();
        if (c == '-') { i++; ParamExpr f = factor();
            return [f](const std::vector<float>& b) { return -f(b); }; }
        if (c == '(') { i++; ParamExpr e = expr();
            if (peek() == ')') i++; else ok = false; return e; }
        if (std::isdigit((unsigned char)c) || c == '.') return number();
        if (std::isalpha((unsigned char)c) || c == '_') return variable();
        ok = false;
        return [](const std::vector<float>&) { return 0.0f; };
    }
    ParamExpr number() {
        skip();
        size_t start = i;
        while (i < s.size() && (std::isdigit((unsigned char)s[i]) || s[i] == '.')) i++;
        float v = 0.0f;
        try { v = std::stof(s.substr(start, i - start)); } catch (...) { ok = false; }
        return [v](const std::vector<float>&) { return v; };
    }
    ParamExpr variable() {
        skip();
        size_t start = i;
        while (i < s.size() && (std::isalnum((unsigned char)s[i]) || s[i] == '_')) i++;
        auto it = vars.find(s.substr(start, i - start));
        int idx = it != vars.end() ? it->second : -1;
        if (idx < 0) ok = false;
        return [idx](const std::vector<float>& b) {
            return (idx >= 0 && idx < (int)b.size()) ? b[idx] : 0.0f;
        };
    }
};

ParamExpr compileExpr(const std::string& text,
                      const std::unordered_map<std::string, int>& vars) {
    ExprCompiler c{text, vars};
    ParamExpr e = c.expr();
    return e;
}

// Read the balanced parenthesised group starting at text[pos]=='(' and return
// its inner contents, advancing pos past the matching ')'.
std::string readParens(const std::string& text, size_t& pos) {
    int depth = 0;
    size_t start = pos + 1;
    for (; pos < text.size(); pos++) {
        if (text[pos] == '(') depth++;
        else if (text[pos] == ')') { depth--; if (depth == 0) { size_t e = pos; pos++;
            return text.substr(start, e - start); } }
    }
    return text.substr(start);   // unbalanced: take the rest
}

// Split "a, b*2, c" on top-level commas (respecting nested parentheses).
std::vector<std::string> splitArgs(const std::string& s) {
    std::vector<std::string> out;
    int depth = 0;
    size_t start = 0;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') depth--;
        else if (s[i] == ',' && depth == 0) { out.push_back(s.substr(start, i - start)); start = i + 1; }
    }
    std::string last = s.substr(start);
    // A trailing/empty arg list ("F()") yields no parameters.
    bool allSpace = last.find_first_not_of(" \t") == std::string::npos;
    if (!(out.empty() && allSpace)) out.push_back(last);
    return out;
}

bool isModuleSymbol(char c) { return !std::isspace((unsigned char)c) && c != '(' && c != ')'; }

}  // namespace

void ParametricLSystem::rule(const std::string& predecessor,
                             const std::string& successor, double weight) {
    // Predecessor: a symbol with optional formal parameter names "A(l,w)".
    size_t p = 0;
    while (p < predecessor.size() && std::isspace((unsigned char)predecessor[p])) p++;
    if (p >= predecessor.size()) return;
    char symbol = predecessor[p++];
    std::vector<std::string> formals;
    std::unordered_map<std::string, int> vars;
    if (p < predecessor.size() && predecessor[p] == '(') {
        for (std::string name : splitArgs(readParens(predecessor, p))) {
            size_t a = name.find_first_not_of(" \t");
            size_t b = name.find_last_not_of(" \t");
            if (a == std::string::npos) continue;
            name = name.substr(a, b - a + 1);
            vars[name] = (int)formals.size();
            formals.push_back(name);
        }
    }

    // Successor: a sequence of module templates with parameter expressions.
    Production prod;
    prod.formals = formals;
    prod.weight = weight;
    for (size_t i = 0; i < successor.size();) {
        if (std::isspace((unsigned char)successor[i])) { i++; continue; }
        ModuleTemplate m;
        m.symbol = successor[i++];
        if (i < successor.size() && successor[i] == '(')
            for (const std::string& arg : splitArgs(readParens(successor, i)))
                m.params.push_back(compileExpr(arg, vars));
        prod.successor.push_back(std::move(m));
    }
    rules_[symbol].push_back(std::move(prod));
}

ModuleString ParametricLSystem::expand(const ModuleString& axiom, int iterations,
                                       uint32_t seed) const {
    std::mt19937 gen(seed);
    ModuleString current = axiom;
    for (int it = 0; it < iterations; it++) {
        ModuleString next;
        next.reserve(current.size() * 2);
        for (const Module& m : current) {
            auto rule = rules_.find(m.symbol);
            if (rule == rules_.end()) { next.push_back(m); continue; }
            // Match by symbol *and* parameter arity (ABoP module matching).
            std::vector<const Production*> matches;
            for (const Production& pr : rule->second)
                if (pr.formals.size() == m.params.size()) matches.push_back(&pr);
            if (matches.empty()) { next.push_back(m); continue; }

            const Production* chosen = matches.front();
            if (matches.size() > 1) {
                double total = 0.0;
                for (const Production* pr : matches) total += pr->weight;
                double pick = std::uniform_real_distribution<double>(0.0, total)(gen);
                for (const Production* pr : matches) {
                    pick -= pr->weight;
                    if (pick <= 0.0) { chosen = pr; break; }
                }
            }
            for (const ModuleTemplate& mt : chosen->successor) {
                Module nm;
                nm.symbol = mt.symbol;
                nm.params.reserve(mt.params.size());
                for (const ParamExpr& e : mt.params) nm.params.push_back(e(m.params));
                next.push_back(std::move(nm));
            }
        }
        current.swap(next);
    }
    return current;
}

ModuleString ParametricLSystem::expand(const std::string& axiom, int iterations,
                                       uint32_t seed) const {
    return expand(parseModuleLiterals(axiom), iterations, seed);
}

ModuleString parseModuleLiterals(const std::string& text) {
    static const std::unordered_map<std::string, int> noVars;
    ModuleString out;
    for (size_t i = 0; i < text.size();) {
        if (!isModuleSymbol(text[i])) { i++; continue; }
        Module m;
        m.symbol = text[i++];
        if (i < text.size() && text[i] == '(')
            for (const std::string& arg : splitArgs(readParens(text, i)))
                m.params.push_back(compileExpr(arg, noVars)({}));   // literal eval
        out.push_back(std::move(m));
    }
    return out;
}

std::string moduleString(const ModuleString& s) {
    auto fmt = [](float v) {
        std::ostringstream os;
        os << v;                       // default formatting trims trailing zeros
        return os.str();
    };
    std::string out;
    for (const Module& m : s) {
        out += m.symbol;
        if (!m.params.empty()) {
            out += '(';
            for (size_t i = 0; i < m.params.size(); i++) {
                if (i) out += ',';
                out += fmt(m.params[i]);
            }
            out += ')';
        }
    }
    return out;
}

namespace {

// The turtle state machine, shared by every interpretation. `onForward` is
// called for each F with the segment's start position, the turtle orientation
// (local +Y is the heading), the radius to use, and whether this is a leaf
// blob (a sphere, no advance) vs a branch segment (a capsule, advances). The
// caller turns that into geometry.
template <typename Fn>
void walkTurtle(const std::string& symbols, const TurtleParams& params, Fn onElement) {
    struct State {
        Vec3 position;
        Mat4 orientation;   // rotates local +Y onto the heading
        float radius;
    };
    State st{Vec3(0, 0, 0), Mat4::identity(), params.radius};
    std::vector<State> stack;
    const double angle = params.angleDeg * PI / 180.0;

    for (char c : symbols) {
        switch (c) {
            case 'F': {
                onElement(st.position, st.orientation, st.radius, /*isLeaf=*/false);
                Vec3 heading = st.orientation.transformDirection(Vec3(0, 1, 0));
                st.position = st.position + heading * params.length;
                st.radius *= params.taper;   // thin continuously up the path
                break;
            }
            case 'L':   // a leaf blob at the current position (no advance)
                if (params.leafRadius > 0.0f)
                    onElement(st.position, st.orientation, params.leafRadius, /*isLeaf=*/true);
                break;
            case '+': st.orientation = st.orientation * Mat4::rotateZ(angle);  break;
            case '-': st.orientation = st.orientation * Mat4::rotateZ(-angle); break;
            case '&': st.orientation = st.orientation * Mat4::rotateX(angle);  break;
            case '^': st.orientation = st.orientation * Mat4::rotateX(-angle); break;
            case '/': st.orientation = st.orientation * Mat4::rotateY(angle);  break;
            case '\\': st.orientation = st.orientation * Mat4::rotateY(-angle); break;
            case '[': stack.push_back(st); st.radius *= params.radiusTaper;    break;
            case ']': if (!stack.empty()) { st = stack.back(); stack.pop_back(); } break;
            default: break;   // letters with no turtle meaning (e.g. rule symbols)
        }
    }
}

}  // namespace

RenderMesh buildTurtleMesh(const std::string& symbols, const TurtleParams& params) {
    RenderMesh mesh;
    walkTurtle(symbols, params, [&](const Vec3& pos, const Mat4& orient, float radius,
                                    bool isLeaf) {
        if (isLeaf) {
            RenderMesh blob = MeshBuilder::sphere(radius, 6, 8);
            MeshBuilder::transform(blob, Mat4::translate(pos.x, pos.y, pos.z));
            MeshBuilder::append(mesh, blob);
            return;
        }
        // A cylinder is centered on its local Y over [-h/2, h/2]; place its
        // center half a length along the heading so the segment runs from pos to
        // pos + heading*length.
        RenderMesh seg = MeshBuilder::cylinder(radius, params.length, params.segmentSlices);
        Vec3 heading = orient.transformDirection(Vec3(0, 1, 0));
        Vec3 center = pos + heading * (params.length * 0.5);
        Mat4 xform = Mat4::translate(center.x, center.y, center.z) * orient;
        MeshBuilder::appendTransformed(mesh, seg, xform);
    });
    return mesh;
}

std::vector<BranchSegment> turtleSegments(const std::string& symbols,
                                          const TurtleParams& params) {
    std::vector<BranchSegment> segments;
    walkTurtle(symbols, params, [&](const Vec3& pos, const Mat4& orient, float radius,
                                    bool isLeaf) {
        if (isLeaf) {
            segments.push_back({pos, pos, radius});   // a==b capsule = sphere blob
            return;
        }
        Vec3 heading = orient.transformDirection(Vec3(0, 1, 0));
        segments.push_back({pos, pos + heading * params.length, radius});
    });
    return segments;
}

std::vector<LeafPlacement> turtleLeaves(const std::string& symbols,
                                        const TurtleParams& params) {
    // Force leaf emission regardless of the blob radius — a real-leaf caller
    // wants the attachment points, not the blob path.
    TurtleParams p = params;
    if (p.leafRadius <= 0.0f) p.leafRadius = 1.0f;

    std::vector<LeafPlacement> leaves;
    walkTurtle(symbols, p, [&](const Vec3& pos, const Mat4& orient, float,
                               bool isLeaf) {
        if (!isLeaf) return;
        leaves.push_back({pos, orient.transformDirection(Vec3(0, 1, 0))});
    });
    return leaves;
}

RenderMesh buildTurtleMeshSdf(const std::string& symbols, const TurtleParams& params,
                              double smoothness, int resolution) {
    std::vector<BranchSegment> segments = turtleSegments(symbols, params);
    if (segments.empty()) return RenderMesh{};

    // Bounds + sizing first, so we can keep branches above a minimum radius.
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    double maxRadius = 0.0;
    auto expand = [&](const Vec3& p) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    };
    for (const BranchSegment& s : segments) {
        expand(s.a);
        expand(s.b);
        maxRadius = std::max(maxRadius, static_cast<double>(s.radius));
    }

    double margin = maxRadius + smoothness;
    Vec3 size = (hi - lo) + Vec3(2 * margin, 2 * margin, 2 * margin);
    double cell = std::max({size.x, size.y, size.z}) / std::max(1, resolution);
    // A capsule thinner than ~1.5 cells can't be captured by Surface Nets (it
    // falls between samples), producing the "weird geometry" on thin branches.
    // Clamp each branch radius up to that floor so every branch stays solid.
    double minRadius = 1.5 * cell;

    std::vector<Sdf> capsules;
    capsules.reserve(segments.size());
    for (const BranchSegment& s : segments)
        capsules.push_back(sdfCapsule(s.a, s.b, std::max(static_cast<double>(s.radius), minRadius)));

    // Pad the sampling box so the full radius + blend fits with a cell to spare.
    Vec3 pad(margin + cell, margin + cell, margin + cell);
    SdfBounds bounds{lo - pad, hi + pad};

    Sdf field = sdfSmoothUnion(capsules, smoothness);
    return polygonizeSdf(field, bounds, resolution);
}

RenderMesh generateTree(const LSystem& system, const std::string& axiom,
                        int iterations, const TurtleParams& params, uint32_t seed) {
    return buildTurtleMesh(system.expand(axiom, iterations, seed), params);
}

RenderMesh generateTreeSdf(const LSystem& system, const std::string& axiom, int iterations,
                           const TurtleParams& params, double smoothness, int resolution,
                           uint32_t seed) {
    return buildTurtleMeshSdf(system.expand(axiom, iterations, seed), params,
                              smoothness, resolution);
}

}  // namespace engine
