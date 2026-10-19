#include "tree_recipes_lua.h"
#include "lua_state.h"
#include "lua_helpers.h"
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"

#include <algorithm>
#include <map>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

// A number or a {lo, hi} range at the top of the stack.
bool readRange(lua_State* L, double& lo, double& hi) {
    if (lua_isnumber(L, -1)) { lo = hi = lua_tonumber(L, -1); return true; }
    if (lua_isboolean(L, -1)) { lo = hi = lua_toboolean(L, -1) ? 1.0 : 0.0; return true; }
    if (!lua_istable(L, -1)) return false;
    lua_rawgeti(L, -1, 1);
    lua_rawgeti(L, -2, 2);
    const bool ok = lua_isnumber(L, -2) && lua_isnumber(L, -1);
    if (ok) { lo = lua_tonumber(L, -2); hi = lua_tonumber(L, -1); }
    lua_pop(L, 2);
    return ok;
}

// A colour {r, g, b} or a range {{r, g, b}, {r, g, b}} at field `key` of table t.
void readColour(lua_State* L, int t, const char* key, Vec3& lo, Vec3& hi) {
    lua_getfield(L, t, key);
    if (lua_istable(L, -1)) {
        lua_rawgeti(L, -1, 1);
        const bool range = lua_istable(L, -1);
        lua_pop(L, 1);
        auto vec = [&](int idx) {
            Vec3 v;
            for (int k = 0; k < 3; ++k) {
                lua_rawgeti(L, idx, k + 1);
                const double x = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0.0;
                lua_pop(L, 1);
                (k == 0 ? v.x : k == 1 ? v.y : v.z) = x;
            }
            return v;
        };
        if (range) {
            lua_rawgeti(L, -1, 1); lo = vec(lua_gettop(L)); lua_pop(L, 1);
            lua_rawgeti(L, -1, 2); hi = vec(lua_gettop(L)); lua_pop(L, 1);
        } else {
            lo = hi = vec(lua_gettop(L));
        }
    }
    lua_pop(L, 1);
}

// A list of colours {{r,g,b}, ...} at the top of the stack.
std::vector<Vec3> readColourList(lua_State* L) {
    std::vector<Vec3> out;
    const int lt = lua_gettop(L), n = static_cast<int>(luaL_len(L, lt));
    for (int i = 1; i <= n; ++i) {
        lua_rawgeti(L, lt, i);
        if (lua_istable(L, -1)) {
            Vec3 v;
            for (int k = 0; k < 3; ++k) {
                lua_rawgeti(L, -1, k + 1);
                (k == 0 ? v.x : k == 1 ? v.y : v.z) = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0.0;
                lua_pop(L, 1);
            }
            out.push_back(v);
        }
        lua_pop(L, 1);
    }
    return out;
}

// LEAF PALETTES at field `key`: one palette {{r,g,b}, ...} or several {{{r,g,b}, ...}, ...}.
bool readPalettes(lua_State* L, int t, const char* key, std::vector<std::vector<Vec3>>& out, const std::string& where,
                  std::string* err) {
    lua_getfield(L, t, key);
    if (lua_istable(L, -1)) {
        lua_rawgeti(L, -1, 1);
        bool nested = false;
        if (lua_istable(L, -1)) {
            lua_rawgeti(L, -1, 1);
            nested = lua_istable(L, -1);
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
        if (nested) {
            const int pt = lua_gettop(L), n = static_cast<int>(luaL_len(L, pt));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, pt, i);
                if (lua_istable(L, -1)) out.push_back(readColourList(L));
                lua_pop(L, 1);
            }
        } else {
            out.push_back(readColourList(L));
        }
        for (const auto& pal : out)
            if (pal.empty()) { lua_pop(L, 1); return fail(err, where + "." + key + ": an empty palette"); }
    }
    lua_pop(L, 1);
    return true;
}

// Every name/value of the table at field `key` of t as ranges, in name order (deterministic whatever the walk).
bool readRanges(lua_State* L, int t, const char* key, std::vector<TreeRecipe::Range>& out, const std::string& where,
                std::string* err) {
    lua_getfield(L, t, key);
    if (lua_istable(L, -1)) {
        const int rt = lua_gettop(L);
        lua_pushnil(L);
        while (lua_next(L, rt) != 0) {
            if (!lua_isstring(L, -2)) { lua_pop(L, 3); return fail(err, where + "." + key + ": a key is not a name"); }
            TreeRecipe::Range g;
            g.name = lua_tostring(L, -2);
            if (!readRange(L, g.lo, g.hi)) {
                lua_pop(L, 3);
                return fail(err, where + "." + key + "." + g.name + ": not a number or {lo, hi}");
            }
            out.push_back(g);
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    std::sort(out.begin(), out.end(), [](const TreeRecipe::Range& a, const TreeRecipe::Range& b) { return a.name < b.name; });
    return true;
}

// The `rules` list at field `key` of t: {predecessor, successor [, weight]} each.
bool readRules(lua_State* L, int t, std::vector<RecipeRule>& out, const std::string& where, std::string* err) {
    lua_getfield(L, t, "rules");
    if (lua_istable(L, -1)) {
        const int rt = lua_gettop(L), n = static_cast<int>(luaL_len(L, rt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, rt, i);
            const int e = lua_gettop(L);
            RecipeRule rule;
            if (lua_istable(L, e)) {
                lua_rawgeti(L, e, 1);
                if (lua_isstring(L, -1)) rule.predecessor = lua_tostring(L, -1);
                lua_rawgeti(L, e, 2);
                if (lua_isstring(L, -1)) rule.successor = lua_tostring(L, -1);
                lua_rawgeti(L, e, 3);
                rule.weight = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : numField(L, e, "weight", 1.0);
                lua_pop(L, 3);
            }
            lua_pop(L, 1);
            if (rule.predecessor.empty() || rule.successor.empty()) {
                lua_pop(L, 1);
                return fail(err, where + ".rules[" + std::to_string(i) + "]: not {predecessor, successor [, weight]}");
            }
            out.push_back(rule);
        }
    }
    lua_pop(L, 1);
    return true;
}

// Every $name a grammar uses has a var.
bool checkVars(const std::string& axiom, const std::vector<RecipeRule>& rules, const std::vector<RecipeRange>& vars,
               const std::string& where, std::string* err) {
    std::map<std::string, double> mid;
    mid["row"] = 0.0;   // (the card grammar's own: the atlas row a card is drawn in)
    for (const RecipeRange& g : vars) mid[g.name] = 0.5 * (g.lo + g.hi);
    std::string missing;
    substituteVars(axiom, mid, &missing);
    for (const RecipeRule& rule : rules) {
        substituteVars(rule.predecessor, mid, &missing);
        substituteVars(rule.successor, mid, &missing);
    }
    if (!missing.empty()) return fail(err, where + ": $" + missing + " has no var");
    return true;
}

// A LEAF CARD (leaf_card.h) table at the top of the stack: a 2D twig grammar, its leaf and flower. (The recipe's
// `card`, and its `strand` -- the near foliage's shoot.)
bool readCard(lua_State* L, LeafCardSpec& cs, const std::string& cw, std::string* err) {
    const int ct = lua_gettop(L);
    cs.set = true;
    if (const std::string a = strField(L, ct, "axiom"); !a.empty()) cs.axiom = a;
    cs.iterations = static_cast<int>(numField(L, ct, "iterations", cs.iterations));
    cs.variants = std::clamp(static_cast<int>(numField(L, ct, "variants", cs.variants)), 1, 4);
    cs.stemWidth = numField(L, ct, "stem_width", cs.stemWidth);
    cs.cup = numField(L, ct, "cup", cs.cup);
    cs.shadeJitter = numField(L, ct, "shade_jitter", cs.shadeJitter);
    cs.tipBright = numField(L, ct, "tip_bright", cs.tipBright);
    if (!readRules(L, ct, cs.rules, cw, err) || !readRanges(L, ct, "vars", cs.vars, cw, err) ||
        !checkVars(cs.axiom, cs.rules, cs.vars, cw, err))
        return false;
    lua_getfield(L, ct, "leaf");
    if (lua_istable(L, -1)) {
        const int lt = lua_gettop(L);
        if (const std::string s = strField(L, lt, "shape"); !s.empty()) cs.shape = s;
        cs.leafLength = numField(L, lt, "length", cs.leafLength);
        cs.leafWidth = numField(L, lt, "width", cs.leafWidth);
        cs.lobes = static_cast<int>(numField(L, lt, "lobes", cs.lobes));
        cs.teeth = static_cast<int>(numField(L, lt, "teeth", cs.teeth));
        Vec3 hi;
        readColour(L, lt, "color", cs.leafTint, hi);
    }
    lua_pop(L, 1);
    { Vec3 hi; readColour(L, ct, "stem_color", cs.stemTint, hi); }
    static const char* kShapes[] = {"oval", "lobed", "serrate", "needle", "leaflet", "palmate"};
    if (std::find(std::begin(kShapes), std::end(kShapes), cs.shape) == std::end(kShapes)) {
        return fail(err, cw + ".leaf: unknown shape '" + cs.shape + "'");
    }
    // THE FLOWER (drawn at every W)
    lua_getfield(L, ct, "flower");
    if (lua_istable(L, -1)) {
        const int ft = lua_gettop(L);
        auto& fl = cs.flower;
        fl.set = true;
        if (const std::string sh = strField(L, ft, "shape"); !sh.empty()) fl.shape = sh;
        fl.size = numField(L, ft, "size", fl.size);
        fl.petals = static_cast<int>(numField(L, ft, "petals", fl.petals));
        fl.notch = numField(L, ft, "notch", fl.notch);
        fl.eye = numField(L, ft, "eye", fl.eye);
        Vec3 hi;
        readColour(L, ft, "color", fl.tint, hi);
        fl.eyeTint = fl.tint;
        readColour(L, ft, "eye_color", fl.eyeTint, hi);
        static const char* kFlowers[] = {"blossom", "daisy", "floret"};
        if (std::find(std::begin(kFlowers), std::end(kFlowers), fl.shape) == std::end(kFlowers)) {
            return fail(err, cw + ".flower: unknown shape '" + fl.shape + "'");
        }
    }
    lua_pop(L, 1);
    return true;
}

}  // namespace

bool readTreeRecipes(ScriptVM& vm, TreeRecipeBook& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "tree_recipes");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "no global `tree_recipes`"); }
    const int book = lua_gettop(L);
    std::vector<TreeRecipe> got;
    lua_pushnil(L);
    while (lua_next(L, book) != 0) {
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "a recipe is not name = {...}"); }
        TreeRecipe r;
        r.name = lua_tostring(L, -2);
        const std::string where = "tree_recipes." + r.name;
        const int t = lua_gettop(L);
        if (const std::string a = strField(L, t, "axiom"); !a.empty()) r.axiom = a;
        r.iterations = static_cast<int>(numField(L, t, "iterations", r.iterations));
        if (const std::string b = strField(L, t, "bark"); !b.empty()) r.bark = b;
        if (const std::string k = strField(L, t, "kind"); !k.empty()) {
            if (k != "tree" && k != "shrub" && k != "blades") {
                lua_settop(L, base);
                return fail(err, where + ".kind: not \"tree\", \"shrub\" or \"blades\"");
            }
            r.shrub = k != "tree";
        }
        readColour(L, t, "bark_color", r.barkColor, r.barkColorHi);
        readColour(L, t, "leaf_color", r.leafColor, r.leafColorHi);
        if (!readPalettes(L, t, "leaf_palette", r.leafPalettes, where, err)) { lua_settop(L, base); return false; }
        if (!readRanges(L, t, "params", r.params, where, err) || !readRanges(L, t, "vars", r.vars, where, err)) {
            lua_settop(L, base);
            return false;
        }
        for (const TreeRecipe::Range& g : r.params) {
            const auto& names = treeParamNames();
            if (std::find(names.begin(), names.end(), g.name) == names.end()) {
                lua_settop(L, base);
                return fail(err, where + ".params: unknown param '" + g.name + "'");
            }
        }
        if (!readRules(L, t, r.rules, where, err) || !checkVars(r.axiom, r.rules, r.vars, where, err)) {
            lua_settop(L, base);
            return false;
        }
        // THE LEAF CARD (leaf_card.h) and the near foliage's STRAND and SPRAY (foliage_kit.h)
        lua_getfield(L, t, "card");
        if (lua_istable(L, -1) && !readCard(L, r.card, where + ".card", err)) { lua_settop(L, base); return false; }
        lua_pop(L, 1);
        lua_getfield(L, t, "strand");
        if (lua_istable(L, -1) && !readCard(L, r.strand, where + ".strand", err)) { lua_settop(L, base); return false; }
        lua_pop(L, 1);
        lua_getfield(L, t, "fronds");
        if (lua_istable(L, -1)) {   // A PALM'S CROWN (foliage_kit.h FrondSpec)
            const int ft = lua_gettop(L);
            FrondSpec& f = r.fronds;
            f.set = true;
            f.fan = numField(L, ft, "fan", 0.0) != 0.0;
            f.count = std::max(1, static_cast<int>(numField(L, ft, "count", f.count)));
            f.length = numField(L, ft, "length", f.length);
            f.lengthJitter = numField(L, ft, "length_jitter", f.lengthJitter);
            f.arch = numField(L, ft, "arch", f.arch);
            f.rachisRadius = numField(L, ft, "rachis", f.rachisRadius);
            f.leaflets = std::max(1, static_cast<int>(numField(L, ft, "leaflets", f.leaflets)));
            f.leafletLength = numField(L, ft, "leaflet_length", f.leafletLength);
            f.leafletWidth = numField(L, ft, "leaflet_width", f.leafletWidth);
            f.leafletAngle = numField(L, ft, "leaflet_angle", f.leafletAngle);
            f.leafletDroop = numField(L, ft, "leaflet_droop", f.leafletDroop);
            f.fanSpread = numField(L, ft, "fan_spread", f.fanSpread);
            f.petiole = numField(L, ft, "petiole", f.petiole);
            f.pitchYoung = numField(L, ft, "pitch_young", f.pitchYoung);
            f.pitchOld = numField(L, ft, "pitch_old", f.pitchOld);
            f.skirt = std::max(0, static_cast<int>(numField(L, ft, "skirt", f.skirt)));
            readColour(L, ft, "color", f.color, f.colorHi);
            Vec3 hi;
            readColour(L, ft, "rachis_color", f.rachisColor, hi);
            readColour(L, ft, "skirt_color", f.skirtColor, hi);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "blades");
        if (lua_istable(L, -1)) {   // A BLADE PLANT (foliage_kit.h BladeSpec)
            const int bt = lua_gettop(L);
            BladeSpec& b = r.blades;
            b.set = true;
            b.count = std::max(1, static_cast<int>(numField(L, bt, "count", b.count)));
            b.length = numField(L, bt, "length", b.length);
            b.lengthJitter = numField(L, bt, "length_jitter", b.lengthJitter);
            b.width = numField(L, bt, "width", b.width);
            b.rise = numField(L, bt, "rise", b.rise);
            b.spread = numField(L, bt, "spread", b.spread);
            b.fan = numField(L, bt, "fan", b.fan);
            b.arch = numField(L, bt, "arch", b.arch);
            b.fold = numField(L, bt, "fold", b.fold);
            b.twist = numField(L, bt, "twist", b.twist);
            b.taper = numField(L, bt, "taper", b.taper);
            b.crown = numField(L, bt, "crown", b.crown);
            readColour(L, bt, "color", b.color, b.colorHi);
            Vec3 hi;
            readColour(L, bt, "edge_color", b.edgeColor, hi);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "leaves");
        if (lua_istable(L, -1)) {   // NEAR LEAVES (foliage = 2; foliage_kit.h LeafKitSpec)
            const int lt = lua_gettop(L);
            LeafKitSpec& lk = r.leafKit;
            lk.set = true;
            lk.size = numField(L, lt, "size", lk.size);
            lk.shoots = std::max(1, static_cast<int>(numField(L, lt, "shoots", lk.shoots)));
            lk.shootLength = numField(L, lt, "shoot_length", lk.shootLength);
            lk.shootAngle = numField(L, lt, "shoot_angle", lk.shootAngle);
            lk.lift = numField(L, lt, "lift", lk.lift);
            lk.sag = numField(L, lt, "sag", lk.sag);
            lk.perTwig = std::max(1, static_cast<int>(numField(L, lt, "per_shoot", lk.perTwig)));
            lk.pairs = numField(L, lt, "pairs", lk.pairs ? 1.0 : 0.0) != 0.0;
            lk.angle = numField(L, lt, "angle", lk.angle);
            lk.fold = numField(L, lt, "fold", lk.fold);
            lk.curl = numField(L, lt, "curl", lk.curl);
            lk.droop = numField(L, lt, "droop", lk.droop);
            lk.face = numField(L, lt, "face", lk.face);
            lk.budget = std::max(1, static_cast<int>(numField(L, lt, "budget", lk.budget)));
            lua_getfield(L, lt, "blossom");
            if (lua_istable(L, -1)) {
                const int bt = lua_gettop(L);
                lk.blossomSize = numField(L, bt, "size", 0.035);
                lk.perCluster = std::max(1, static_cast<int>(numField(L, bt, "per_cluster", lk.perCluster)));
                lk.clusters = std::max(0, static_cast<int>(numField(L, bt, "clusters", lk.clusters)));
                lk.cup = numField(L, bt, "cup", lk.cup);
                lk.blossomBudget = std::max(1, static_cast<int>(numField(L, bt, "budget", lk.blossomBudget)));
            }
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "spray");
        if (lua_istable(L, -1)) {
            const int st = lua_gettop(L);
            SpraySpec& sp = r.spray;
            sp.set = true;
            sp.sideShoots = static_cast<int>(numField(L, st, "side_shoots", sp.sideShoots));
            sp.sideAngle = numField(L, st, "side_angle", sp.sideAngle);
            sp.sideLength = numField(L, st, "side_length", sp.sideLength);
            sp.sideFalloff = numField(L, st, "side_falloff", sp.sideFalloff);
            sp.lift = numField(L, st, "lift", sp.lift);
            sp.fins = std::clamp(static_cast<int>(numField(L, st, "fins", sp.fins)), 1, 6);
            sp.aspect = numField(L, st, "aspect", sp.aspect);
            sp.droop = numField(L, st, "droop", sp.droop);
            sp.variants = std::clamp(static_cast<int>(numField(L, st, "variants", sp.variants)), 1, 16);
            sp.perNode = std::max(1, static_cast<int>(numField(L, st, "per_node", sp.perNode)));
            sp.budget = std::max(1, static_cast<int>(numField(L, st, "budget", sp.budget)));
            readColour(L, st, "color", sp.color, sp.colorHi);
        }
        lua_pop(L, 1);
        // AGES and FORMS (tree_recipe.h TreeAge): { young = {...}, old = {...} }, { stand = {...} } -- each its axiom,
        // iterations, params and vars set, `scale = { params = {...}, vars = {...} }` factors, branch-card budgets
        for (const char* table : {"ages", "forms"}) {
            lua_getfield(L, t, table);
            if (lua_istable(L, -1)) {
                const int at = lua_gettop(L);
                lua_pushnil(L);
                while (lua_next(L, at) != 0) {
                    if (!lua_isstring(L, -2) || !lua_istable(L, -1)) {
                        lua_settop(L, base);
                        return fail(err, where + "." + table + ": not name = {...}");
                    }
                    const std::string an = lua_tostring(L, -2);
                    const std::string aw = where + "." + table + "." + an;
                    const int a = lua_gettop(L);
                    TreeAge age;
                    age.axiom = strField(L, a, "axiom");
                    age.axiomPrefix = strField(L, a, "axiom_prefix");
                    age.iterations = static_cast<int>(numField(L, a, "iterations", -1));
                    age.sprig = numField(L, a, "sprig", -1);
                    age.nearLeaves = numField(L, a, "near_leaves", -1);
                    age.nearTwigTris = numField(L, a, "near_twig_tris", -1);
                    if (!readRanges(L, a, "params", age.params, aw, err) || !readRanges(L, a, "vars", age.vars, aw, err)) {
                        lua_settop(L, base);
                        return false;
                    }
                    lua_getfield(L, a, "scale");
                    if (lua_istable(L, -1)) {
                        const int sc = lua_gettop(L);
                        if (!readRanges(L, sc, "params", age.paramsScale, aw + ".scale", err) ||
                            !readRanges(L, sc, "vars", age.varsScale, aw + ".scale", err)) {
                            lua_settop(L, base);
                            return false;
                        }
                    }
                    lua_pop(L, 1);
                    for (const std::vector<TreeRecipe::Range>* list : {&age.params, &age.paramsScale})
                        for (const TreeRecipe::Range& g : *list) {
                            const auto& names = treeParamNames();
                            if (std::find(names.begin(), names.end(), g.name) == names.end()) {
                                lua_settop(L, base);
                                return fail(err, aw + ".params: unknown param '" + g.name + "'");
                            }
                        }
                    (std::string(table) == "ages" ? r.ages : r.forms)[an] = age;
                    lua_pop(L, 1);
                }
            }
            lua_pop(L, 1);
        }
        lua_getfield(L, t, "needles");
        if (lua_istable(L, -1)) {   // A CONIFER'S NEEDLES (procgen/branch_cards.h)
            const int nt = lua_gettop(L);
            NeedleSpec& nd = r.needles;
            nd.set = true;
            lua_getfield(L, nt, "kind");
            if (lua_isstring(L, -1)) nd.kind = lua_tostring(L, -1);
            lua_pop(L, 1);
            if (nd.kind != "brush" && nd.kind != "flat" && nd.kind != "tuft") {
                lua_settop(L, base);
                return fail(err, where + ".needles: kind must be brush, flat or tuft (not " + nd.kind + ")");
            }
            nd.length = numField(L, nt, "length", nd.length);
            nd.width = numField(L, nt, "width", nd.width);
            nd.density = numField(L, nt, "density", nd.density);
            nd.forward = numField(L, nt, "forward", nd.forward);
            nd.shootLength = numField(L, nt, "shoot_length", nd.shootLength);
            nd.sideShoots = std::max(0, static_cast<int>(numField(L, nt, "side_shoots", nd.sideShoots)));
            nd.sideAngle = numField(L, nt, "side_angle", nd.sideAngle);
            nd.reach = std::clamp(numField(L, nt, "reach", nd.reach), 0.05, 1.0);
            nd.limbRadius = numField(L, nt, "limb_radius", nd.limbRadius);
            nd.hang = std::clamp(numField(L, nt, "hang", nd.hang), 0.0, 1.0);
            nd.frondSpacing = std::max(0.0, numField(L, nt, "frond_spacing", nd.frondSpacing));
            nd.frondLength = std::max(0.05, numField(L, nt, "frond_length", nd.frondLength));
            readColour(L, nt, "color", nd.color, nd.colorHi);
            Vec3 hi;
            readColour(L, nt, "tip_color", nd.tipColor, hi);
            readColour(L, nt, "twig_color", nd.twigColor, hi);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "branch_cards");
        if (lua_istable(L, -1)) {   // THE BRANCH CARDS (procgen/branch_cards.h)
            const int ct = lua_gettop(L);
            BranchCardSpec& bc = r.branchCards;
            bc.set = true;
            bc.sprig = numField(L, ct, "sprig", bc.sprig);
            bc.woodEdge = numField(L, ct, "wood_edge", bc.woodEdge);
            bc.ringAngle = numField(L, ct, "ring_angle", bc.ringAngle);
            bc.pictures = std::clamp(static_cast<int>(numField(L, ct, "pictures", bc.pictures)), 1, 16);
            bc.twigs = numField(L, ct, "twigs", bc.twigs ? 1.0 : 0.0) != 0.0;
            bc.variants = std::clamp(static_cast<int>(numField(L, ct, "variants", bc.variants)), 1, 6);
            bc.fillRadius = numField(L, ct, "fill_radius", bc.fillRadius);
            bc.fillSpacing = numField(L, ct, "fill_spacing", bc.fillSpacing);
            bc.nearLeaves = std::max(0, static_cast<int>(numField(L, ct, "near_leaves", bc.nearLeaves)));
            bc.nearTwigTris = std::max(0, static_cast<int>(numField(L, ct, "near_twig_tris", bc.nearTwigTris)));
            bc.views = std::clamp(static_cast<int>(numField(L, ct, "views", bc.views)), 1, 4);
            bc.cell = std::clamp(static_cast<int>(numField(L, ct, "cell", bc.cell)), 64, 512);
            bc.fold = numField(L, ct, "fold", bc.fold);
            bc.bend = std::clamp(numField(L, ct, "bend", bc.bend), 0.0, 1.0);
        }
        lua_pop(L, 1);
        got.push_back(std::move(r));
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    std::sort(got.begin(), got.end(), [](const TreeRecipe& a, const TreeRecipe& b) { return a.name < b.name; });
    for (TreeRecipe& r : got) out.set(std::move(r));
    return true;
}

bool loadTreeRecipesSource(const std::string& source, TreeRecipeBook& out, std::string* err) {
    ScriptVM vm;
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "tree_recipes.lua: " + e);
    return readTreeRecipes(vm, out, err);
}

TreeRecipeBook loadShippedTreeRecipes(std::string* path) {
    TreeRecipeBook book;
    if (path) *path = resolveScriptPath("tree_recipes.lua", "");
    const std::string src = loadScriptCode("tree_recipes.lua", "");
    if (src.empty()) { LOG_WARN << "[trees] no tree_recipes.lua: recipe trees grow nothing"; return book; }
    std::string err;
    if (!loadTreeRecipesSource(src, book, &err)) { LOG_WARN << "[trees] " << err; return TreeRecipeBook{}; }
    LOG_INFO << "[trees] " << book.size() << " tree recipes";
    return book;
}

const TreeRecipeBook& shippedTreeRecipes(std::string* path) {
    static std::string lastSrc;
    static TreeRecipeBook book;
    if (path) *path = resolveScriptPath("tree_recipes.lua", "");
    const std::string src = loadScriptCode("tree_recipes.lua", "");
    if (src != lastSrc) {
        lastSrc = src;
        book = TreeRecipeBook{};
        std::string err;
        if (src.empty()) LOG_WARN << "[trees] no tree_recipes.lua: recipe trees grow nothing";
        else if (!loadTreeRecipesSource(src, book, &err)) { LOG_WARN << "[trees] " << err; book = TreeRecipeBook{}; }
        else LOG_INFO << "[trees] " << book.size() << " tree recipes";
    }
    return book;
}

}  // namespace engine
