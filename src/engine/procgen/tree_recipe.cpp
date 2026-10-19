#include <algorithm>
#include "tree_recipe.h"

#include "lsystem.h"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace engine {

namespace {

// Each TreeParams field by name: a float, an int (rounded) or a bool (non-zero).
struct Field {
    const char* name;
    float TreeParams::*f = nullptr;
    int TreeParams::*i = nullptr;
    bool TreeParams::*b = nullptr;
};
const std::vector<Field>& fields() {
    static const std::vector<Field> v = {
        {"iterations", nullptr, &TreeParams::iterations}, {"trunkLength", &TreeParams::trunkLength},
        {"lengthFalloff", &TreeParams::lengthFalloff}, {"leaderFalloff", &TreeParams::leaderFalloff},
        {"branchAngle", &TreeParams::branchAngle}, {"angleJitter", &TreeParams::angleJitter},
        {"branchesPerNode", nullptr, &TreeParams::branchesPerNode}, {"phyllotaxis", &TreeParams::phyllotaxis},
        {"terminalFraction", &TreeParams::terminalFraction}, {"terminalForks", nullptr, &TreeParams::terminalForks},
        {"droop", &TreeParams::droop}, {"wander", &TreeParams::wander},
        {"light", &TreeParams::light}, {"crookChance", &TreeParams::crookChance}, {"crookAngle", &TreeParams::crookAngle},
        {"meander", &TreeParams::meander}, {"lean", &TreeParams::lean}, {"flare", &TreeParams::flare},
        {"lobes", nullptr, &TreeParams::lobes}, {"lobeDepth", &TreeParams::lobeDepth}, {"lobeHeight", &TreeParams::lobeHeight},
        {"nodeSwell", &TreeParams::nodeSwell}, {"barkNoise", &TreeParams::barkNoise}, {"twist", &TreeParams::twist},
        {"collar", &TreeParams::collar}, {"forkRatio", &TreeParams::forkRatio}, {"forkMinRadius", &TreeParams::forkMinRadius},
        {"rootCount", nullptr, &TreeParams::rootCount}, {"rootSpread", &TreeParams::rootSpread},
        {"tipRadius", &TreeParams::tipRadius}, {"pipeExponent", &TreeParams::pipeExponent},
        {"radiusScale", &TreeParams::radiusScale}, {"ringSegments", nullptr, &TreeParams::ringSegments},
        {"barkVScale", &TreeParams::barkVScale}, {"leaves", nullptr, nullptr, &TreeParams::leaves},
        {"leafSize", &TreeParams::leafSize}, {"leavesPerTip", nullptr, &TreeParams::leavesPerTip},
        {"leafThickness", &TreeParams::leafThickness}, {"leafClump", &TreeParams::leafClump}, {"leafPitch", &TreeParams::leafPitch},
        {"maxLeafCards", nullptr, &TreeParams::maxLeafCards}, {"cardTiles", nullptr, &TreeParams::cardTiles},
        {"paletteScale", &TreeParams::paletteScale}, {"tipCards", nullptr, nullptr, &TreeParams::tipCards},
        {"foliage", nullptr, &TreeParams::foliage},
        {"joints", nullptr, &TreeParams::joints}, {"jointMinRadius", &TreeParams::jointMinRadius},
        {"woodMinRadius", &TreeParams::woodMinRadius}, {"leafLod", &TreeParams::leafLod},
        {"sprigSize", &TreeParams::sprigSize}, {"sprigDonor", nullptr, &TreeParams::sprigDonor},
        {"woodEdge", &TreeParams::woodEdge}, {"ringAngle", &TreeParams::ringAngle}, {"ringSpacing", &TreeParams::ringSpacing},
        {"trunkZone", &TreeParams::trunkZone}, {"deadChance", &TreeParams::deadChance},
        {"colonize", nullptr, &TreeParams::colonize}, {"colonizePoints", nullptr, &TreeParams::colonizePoints},
        {"colonizeStep", &TreeParams::colonizeStep}, {"colonizeShell", &TreeParams::colonizeShell},
        {"crownWiden", &TreeParams::crownWiden}, {"crownHeighten", &TreeParams::crownHeighten}, {"trunkZoneCell", &TreeParams::trunkZoneCell},
        {"trunkZoneTris", nullptr, &TreeParams::trunkZoneTris},
        {"barkMottle", &TreeParams::barkMottle}, {"barkFootDark", &TreeParams::barkFootDark},
        {"barkFootHeight", &TreeParams::barkFootHeight},
        {"clipX", &TreeParams::clipX}, {"clipY", &TreeParams::clipY}, {"clipZ", &TreeParams::clipZ},
    };
    return v;
}

uint32_t mix32(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
uint32_t nameHash(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) { h ^= c; h *= 16777619u; }
    return h;
}
// A number in [0,1) for this seed and this name: each range draws on its own, so adding one leaves the others alone
double draw01(uint32_t seed, const std::string& name) {
    return mix32(seed * 0x9E3779B9u ^ nameHash(name)) / 4294967296.0;
}

}  // namespace

void TreeRecipeBook::set(TreeRecipe r) {
    for (TreeRecipe& q : recipes_)
        if (q.name == r.name) { q = std::move(r); return; }
    recipes_.push_back(std::move(r));
}
const TreeRecipe* TreeRecipeBook::find(const std::string& name) const {
    for (const TreeRecipe& q : recipes_)
        if (q.name == name) return &q;
    return nullptr;
}
std::vector<std::string> TreeRecipeBook::names() const {
    std::vector<std::string> v;
    for (const TreeRecipe& q : recipes_) v.push_back(q.name);
    return v;
}

bool setTreeParam(TreeParams& p, const std::string& name, double value) {
    for (const Field& f : fields()) {
        if (name != f.name) continue;
        if (f.f) p.*(f.f) = static_cast<float>(value);
        else if (f.i) p.*(f.i) = static_cast<int>(std::lround(value));
        else if (f.b) p.*(f.b) = value != 0.0;
        return true;
    }
    return false;
}

const std::vector<std::string>& treeParamNames() {
    static const std::vector<std::string> v = [] {
        std::vector<std::string> out;
        for (const Field& f : fields()) out.push_back(f.name);
        return out;
    }();
    return v;
}

TreeParams drawTreeParams(const TreeRecipe& r, uint32_t seed, std::map<std::string, double>* vars) {
    TreeParams p;
    p.iterations = r.iterations;
    for (const TreeRecipe::Range& g : r.params)
        setTreeParam(p, g.name, g.lo + (g.hi - g.lo) * draw01(seed, "p:" + g.name));
    if (vars)
        for (const TreeRecipe::Range& g : r.vars) (*vars)[g.name] = g.lo + (g.hi - g.lo) * draw01(seed, "v:" + g.name);
    const double tb = draw01(seed, "c:bark"), tl = draw01(seed, "c:leaf");
    p.barkColor = r.barkColor + (r.barkColorHi - r.barkColor) * tb;
    p.leafColor = r.leafColor + (r.leafColorHi - r.leafColor) * tl;
    if (!r.leafPalettes.empty()) {
        const std::size_t k = static_cast<std::size_t>(draw01(seed, "c:palette") * r.leafPalettes.size());
        p.leafPalette = r.leafPalettes[std::min(k, r.leafPalettes.size() - 1)];
    }
    return p;
}

std::string substituteVars(const std::string& text, const std::map<std::string, double>& vars, std::string* missing) {
    std::string out;
    for (std::size_t i = 0; i < text.size();) {
        if (text[i] != '$') { out += text[i++]; continue; }
        std::size_t j = i + 1;
        while (j < text.size() && (std::isalnum(static_cast<unsigned char>(text[j])) || text[j] == '_')) ++j;
        const std::string name = text.substr(i + 1, j - i - 1);
        auto it = vars.find(name);
        if (it == vars.end()) {
            if (missing && missing->empty()) *missing = name;
            out += text.substr(i, j - i);
        } else {
            char buf[32];
            std::snprintf(buf, sizeof buf, "%.6g", it->second);
            out += buf;
        }
        i = j;
    }
    return out;
}

TreeRecipe agedRecipe(const TreeRecipe& r, const std::string& age) {
    auto it = r.ages.find(age);
    if (it == r.ages.end()) return r;
    const TreeAge& a = it->second;
    TreeRecipe out = r;
    if (!a.axiom.empty()) out.axiom = a.axiom;
    if (a.iterations >= 0) out.iterations = a.iterations;
    auto over = [](std::vector<RecipeRange>& base, const std::vector<RecipeRange>& top) {
        for (const RecipeRange& g : top) {
            auto f = std::find_if(base.begin(), base.end(), [&](const RecipeRange& b) { return b.name == g.name; });
            if (f != base.end()) *f = g; else base.push_back(g);
        }
    };
    over(out.params, a.params);
    over(out.vars, a.vars);
    if (a.sprig >= 0) out.branchCards.sprig = a.sprig;
    if (a.nearLeaves >= 0) out.branchCards.nearLeaves = static_cast<int>(a.nearLeaves);
    if (a.nearTwigTris >= 0) out.branchCards.nearTwigTris = static_cast<int>(a.nearTwigTris);
    out.ages.clear();
    return out;
}

TreeMesh growRecipeTree(const TreeRecipe& r, uint32_t seed, const std::map<std::string, double>& overrides) {
    if (r.blades.set) {   // a BLADE PLANT: no grammar, no wood -- its blades and the crown they rise from
        TreeMesh t;
        buildBladePlant(r.blades, seed, t.leaves, t.branches);
        t.foliage = 3;
        t.collisionVertices.reserve(t.branches.vertices.size());
        for (const Vertex& v : t.branches.vertices) t.collisionVertices.push_back(v.position);
        t.collisionIndices = t.branches.indices;
        return t;
    }
    std::map<std::string, double> vars;
    TreeParams p = drawTreeParams(r, seed, &vars);
    for (const auto& [name, v] : overrides)
        if (!setTreeParam(p, name, v)) vars[name] = v;   // (a var's name: the lab may pin a grammar number too)
    if (r.card.set) p.cardTiles = std::max(1, r.card.variants);   // (the card atlas the recipe draws)
    p.spray = r.spray;
    p.leafKit = r.leafKit;
    p.fronds = r.fronds;
    if (p.foliage == 4 && !r.fronds.set) p.foliage = 0;
    if (p.foliage == 1 && (!r.spray.set || !r.strand.set)) p.foliage = 0;   // (no spray kit: cards)
    if (p.foliage == 2 && (!r.leafKit.set || !r.card.set)) p.foliage = 0;   // (no leaf kit: cards)
    if (p.foliage == 1) p.cardTiles = std::max(1, r.strand.variants);      // (the strand atlas)
    if (p.foliage == 2) p.cardTiles = 2;                                    // (the element atlas: 2 x 2)
    if (r.rules.empty()) return growTree(p, seed);
    ParametricLSystem grammar;
    for (const TreeRecipe::Rule& rule : r.rules)
        grammar.rule(substituteVars(rule.predecessor, vars), substituteVars(rule.successor, vars), rule.weight);
    const ModuleString axiom = parseModuleLiterals(substituteVars(r.axiom, vars));
    const ModuleString s = grammar.expand(axiom, p.iterations, seed);
    return skinTree(s, p, seed);
}

}  // namespace engine
