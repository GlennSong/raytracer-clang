#include "furniture_library_lua.h"
#include "lua_state.h"
#include "lua_helpers.h"
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"

#include <algorithm>
#include <mutex>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

}  // namespace

bool readFurnitureLibrary(ScriptVM& vm, FurnitureLibrary& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "furniture_library");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "no global `furniture_library`"); }
    const int lib = lua_gettop(L);
    out.clear();
    lua_pushnil(L);
    while (lua_next(L, lib) != 0) {
        // key at -2, entry at -1
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "an entry is not name = {...}"); }
        const std::string name = lua_tostring(L, -2);
        const int t = lua_gettop(L);
        FurnitureAsset a;
        if (!furniturePieceByName(name, a.piece)) { lua_settop(L, base); return fail(err, "unknown piece '" + name + "'"); }
        a.family = strField(L, t, "family");
        lua_getfield(L, t, "tags");
        if (lua_istable(L, -1)) {
            const int n = static_cast<int>(luaL_len(L, -1));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, -1, i);
                if (lua_isstring(L, -1)) a.tags.push_back(lua_tostring(L, -1));
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "spots");
        if (lua_istable(L, -1)) {
            const int st = lua_gettop(L), n = static_cast<int>(luaL_len(L, st));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, st, i);
                if (lua_istable(L, -1)) {
                    FurnSpot s;
                    s.id = strField(L, lua_gettop(L), "id");
                    s.at = vec3Field(L, lua_gettop(L), "at", Vec3(0, 0, 0));
                    a.spots.push_back(s);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        if (a.spots.size() > 32) { lua_settop(L, base); return fail(err, name + ": more than 32 spots"); }
        a.variety = static_cast<int>(numField(L, t, "variety", 0.0));
        a.rise = numField(L, t, "rise", 0.0);
        lua_getfield(L, t, "clearance");   // {front = m, side = m}
        if (lua_istable(L, -1)) {
            a.clearFront = numField(L, lua_gettop(L), "front", 0.0);
            a.clearSide = numField(L, lua_gettop(L), "side", 0.0);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "anchors");
        if (lua_istable(L, -1)) {
            const int at = lua_gettop(L), n = static_cast<int>(luaL_len(L, at));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, at, i);
                if (lua_istable(L, -1)) {
                    const int e = lua_gettop(L);
                    FurnAnchor an;
                    an.id = strField(L, e, "id");
                    an.at = vec3Field(L, e, "at", an.at);
                    an.w = numField(L, e, "w", an.w);
                    an.d = numField(L, e, "d", an.d);
                    an.chance = numField(L, e, "chance", 1.0);
                    an.yaw = numField(L, e, "yaw", 0.0);
                    lua_getfield(L, e, "accepts");
                    if (lua_istable(L, -1)) {
                        const int k = static_cast<int>(luaL_len(L, -1));
                        for (int j = 1; j <= k; ++j) {
                            lua_rawgeti(L, -1, j);
                            if (lua_isstring(L, -1)) an.accepts.push_back(lua_tostring(L, -1));
                            lua_pop(L, 1);
                        }
                    }
                    lua_pop(L, 1);
                    if (an.accepts.empty()) { lua_settop(L, base); return fail(err, name + ": anchor '" + an.id + "' accepts nothing"); }
                    a.anchors.push_back(an);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "verbs");
        if (lua_istable(L, -1)) {
            const int vt = lua_gettop(L), n = static_cast<int>(luaL_len(L, vt));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, vt, i);
                if (lua_istable(L, -1)) {
                    const int e = lua_gettop(L);
                    FurnVerb v;
                    const std::string vn = strField(L, e, "verb");
                    if (!verbByName(vn, v.verb)) { lua_settop(L, base); return fail(err, name + ": unknown verb '" + vn + "'"); }
                    v.label = strField(L, e, "label");
                    v.eye = vec3Field(L, e, "eye", v.eye);
                    v.look = vec3Field(L, e, "look", v.look);
                    v.exit = vec3Field(L, e, "exit", v.exit);
                    lua_getfield(L, e, "spots");
                    if (lua_istable(L, -1)) {
                        const int sn = static_cast<int>(luaL_len(L, -1));
                        for (int k = 1; k <= sn; ++k) {
                            lua_rawgeti(L, -1, k);
                            const std::string sid = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
                            lua_pop(L, 1);
                            bool found = false;
                            for (std::size_t j = 0; j < a.spots.size(); ++j)
                                if (a.spots[j].id == sid) { v.spots |= 1u << j; found = true; }
                            if (!found) { lua_settop(L, base); return fail(err, name + ": verb uses unknown spot '" + sid + "'"); }
                        }
                    }
                    lua_pop(L, 1);
                    if (v.spots == 0) { lua_settop(L, base); return fail(err, name + ": a verb with no spots"); }
                    a.verbs.push_back(v);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        out.set(std::move(a));
        lua_pop(L, 1);   // the entry; the key stays for lua_next
    }
    lua_settop(L, base);
    return true;
}

namespace {

// a pick: "piece_name" or {family = "...", tags = {...}}
bool readPick(lua_State* L, int idx, FurnPick& out, const std::string& where, std::string* err) {
    if (lua_isstring(L, idx)) {
        const std::string n = lua_tostring(L, idx);
        if (!furniturePieceByName(n, out.piece)) return fail(err, where + ": unknown piece '" + n + "'");
        return true;
    }
    if (!lua_istable(L, idx)) return fail(err, where + ": a pick is a piece name or {family =, tags =}");
    const int t = lua_absindex(L, idx);
    out.family = strField(L, t, "family");
    if (out.family.empty()) return fail(err, where + ": a pick by tags needs a family");
    lua_getfield(L, t, "tags");
    if (lua_istable(L, -1)) {
        const int n = static_cast<int>(luaL_len(L, -1));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_isstring(L, -1)) out.tags.push_back(lua_tostring(L, -1));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    return true;
}

bool readPickField(lua_State* L, int t, const char* key, FurnPick& out, const std::string& where, std::string* err,
                   bool required) {
    lua_getfield(L, t, key);
    bool ok = true;
    if (!lua_isnil(L, -1)) ok = readPick(L, -1, out, where + "." + key, err);
    else if (required) ok = fail(err, where + ": no `" + key + "`");
    lua_pop(L, 1);
    return ok;
}

// "long" or {0, 1, 3}
bool readSides(lua_State* L, int t, const char* key, std::vector<int>& out, bool* longFirst) {
    lua_getfield(L, t, key);
    if (lua_isstring(L, -1) && std::string(lua_tostring(L, -1)) == "long") {
        if (longFirst) *longFirst = true;
    } else if (lua_istable(L, -1)) {
        const int n = static_cast<int>(luaL_len(L, -1));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, -1, i);
            if (lua_isnumber(L, -1)) out.push_back(static_cast<int>(lua_tointeger(L, -1)));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    return true;
}

bool readPair(lua_State* L, int t, const char* key, Real& a, Real& b) {
    lua_getfield(L, t, key);
    const bool ok = lua_istable(L, -1);
    if (ok) {
        lua_rawgeti(L, -1, 1); a = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : a; lua_pop(L, 1);
        lua_rawgeti(L, -1, 2); b = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : b; lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return ok;
}

bool readStep(lua_State* L, int t, FurnStep& st, const std::string& where, std::string* err) {
    lua_getfield(L, t, "one_of");
    if (lua_istable(L, -1)) {
        st.kind = FurnStep::Kind::OneOf;
        const int ot = lua_gettop(L), n = static_cast<int>(luaL_len(L, ot));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, ot, i);
            FurnStep alt;
            if (!lua_istable(L, -1) || !readStep(L, lua_gettop(L), alt, where + ".one_of[" + std::to_string(i) + "]", err)) {
                lua_pop(L, 2);
                return err && !err->empty() ? false : fail(err, where + ": a one_of entry is not a step");
            }
            st.alternatives.push_back(std::move(alt));
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
        st.choices = st.alternatives.size();
        lua_getfield(L, t, "fallback");   // { one_of = {...}, fallback = step }: where none of them fits
        if (lua_istable(L, -1)) {
            FurnStep fb;
            if (!readStep(L, lua_gettop(L), fb, where + ".fallback", err)) { lua_pop(L, 1); return false; }
            st.alternatives.push_back(std::move(fb));
        }
        lua_pop(L, 1);
        return st.choices == 0 ? fail(err, where + ": an empty one_of") : true;
    }
    lua_pop(L, 1);
    const bool counter = [&] { lua_getfield(L, t, "counter"); const bool c = lua_isstring(L, -1); if (c) st.pattern = lua_tostring(L, -1); lua_pop(L, 1); return c; }();
    if (readPair(L, t, "grid", st.w, st.d)) st.kind = FurnStep::Kind::Grid;
    else if (counter) st.kind = FurnStep::Kind::Counter;
    else if ([&] { lua_getfield(L, t, "hang"); const bool h = !lua_isnil(L, -1); lua_pop(L, 1); return h; }()) st.kind = FurnStep::Kind::Hang;
    else if (readPair(L, t, "wall", st.w, st.d)) st.kind = FurnStep::Kind::Wall;
    else return fail(err, where + ": a step is wall =, grid =, counter =, hang = or one_of =");
    if (counter) {
        readPair(L, t, "wall", st.w, st.d);
        if (st.pattern != "kitchen" && st.pattern != "kitchenette") return fail(err, where + ": unknown counter '" + st.pattern + "'");
        if (!readPickField(L, t, "base", st.base, where, err, true) || !readPickField(L, t, "sink", st.sink, where, err, true) ||
            !readPickField(L, t, "hob", st.hob, where, err, false) || !readPickField(L, t, "tall_unit", st.tallUnit, where, err, false) ||
            !readPickField(L, t, "wall_unit", st.wallUnit, where, err, false))
            return false;
    }
    if (st.kind == FurnStep::Kind::Hang && !readPickField(L, t, "hang", st.pick, where, err, true)) return false;
    if (!readPickField(L, t, "pick", st.pick, where, err, false)) return false;
    readSides(L, t, "sides", st.sides, &st.longFirst);
    readSides(L, t, "ring_sides", st.ringSides, nullptr);
    lua_getfield(L, t, "sides");
    st.opposite = lua_isstring(L, -1) && std::string(lua_tostring(L, -1)) == "opposite";
    lua_pop(L, 1);
    st.tall = boolField(L, t, "tall", false);
    st.clear = numField(L, t, "clear", 0.0);
    st.minW = numField(L, t, "min_w", 0.0);
    lua_getfield(L, t, "count");
    if (lua_isnumber(L, -1)) st.countMin = st.countMax = static_cast<int>(lua_tointeger(L, -1));
    else if (lua_istable(L, -1)) {
        const int c = lua_gettop(L);
        st.countPer = numField(L, c, "per", 0.0);
        st.countMin = static_cast<int>(numField(L, c, "min", 1.0));
        st.countMax = static_cast<int>(numField(L, c, "max", 1.0));
    }
    lua_pop(L, 1);
    readPair(L, t, "aisle", st.aisleX, st.aisleZ);
    st.d2 = numField(L, t, "short", 0.0);
    st.from = numField(L, t, "from", 0.6);
    st.rows = static_cast<int>(numField(L, t, "rows", 0.0));
    st.to = numField(L, t, "to", 0.8);
    st.anyWall = boolField(L, t, "any_wall", false);
    st.tiers = boolField(L, t, "tiers", false);
    {
        Real rn = 0, cw = 0;
        if (readPair(L, t, "run", rn, cw)) { st.runN = static_cast<int>(rn); st.crossW = cw; }
    }
    st.style2 = static_cast<int>(numField(L, t, "short_style", numField(L, t, "style", -1.0)));
    lua_getfield(L, t, "set");
    if (lua_istable(L, -1)) {
        const int sv = lua_gettop(L), n = static_cast<int>(luaL_len(L, sv));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, sv, i);
            const int e = lua_gettop(L);
            FurnMember m;
            const std::string mw = where + ".set[" + std::to_string(i) + "]";
            if (!lua_istable(L, e) || !readPickField(L, e, "pick", m.pick, mw, err, true)) { lua_pop(L, 2); return false; }
            m.x = numField(L, e, "x", 0.0);
            m.z = numField(L, e, "z", 0.0);
            m.y = numField(L, e, "y", 0.0);
            m.facing = strField(L, e, "faces") != "wall";
            readPair(L, e, "fit", m.fitW, m.fitD);
            st.set.push_back(std::move(m));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    if ((st.kind == FurnStep::Kind::Wall || st.kind == FurnStep::Kind::Grid) && st.set.empty() && !st.pick.valid())
        return fail(err, where + ": nothing to place (no pick, no set)");
    return true;
}

}  // namespace

bool readRoomPrograms(ScriptVM& vm, FurnitureLibrary& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "furniture_rooms");
    if (lua_isnil(L, -1)) { lua_settop(L, base); return true; }   // no programs: rooms stay empty
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "`furniture_rooms` is not a table"); }
    const int rooms = lua_gettop(L);
    lua_pushnil(L);
    while (lua_next(L, rooms) != 0) {
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "a room program is not name = {...}"); }
        RoomProgram prog;
        prog.name = lua_tostring(L, -2);
        const int pt = lua_gettop(L), n = static_cast<int>(luaL_len(L, pt));
        for (int i = 1; i <= n; ++i) {
            lua_rawgeti(L, pt, i);
            FurnStep st;
            const std::string where = "furniture_rooms." + prog.name + "[" + std::to_string(i) + "]";
            if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, where + ": not a table"); }
            if (!readStep(L, lua_gettop(L), st, where, err)) { lua_settop(L, base); return false; }
            prog.steps.push_back(std::move(st));
            lua_pop(L, 1);
        }
        out.setProgram(std::move(prog));
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    return true;
}

bool loadFurnitureLibrarySource(const std::string& source, FurnitureLibrary& out, std::string* err) {
    ScriptVM vm;
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "furniture_library.lua: " + e);
    return readFurnitureLibrary(vm, out, err) && readRoomPrograms(vm, out, err);
}

bool readFloorPrograms(ScriptVM& vm, FloorProgramBook& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "floor_programs");
    if (lua_isnil(L, -1)) { lua_settop(L, base); return true; }   // none: the built-in office program
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "`floor_programs` is not a table"); }
    const int progs = lua_gettop(L);
    auto bad = [&](const std::string& m) { lua_settop(L, base); return fail(err, m); };
    lua_pushnil(L);
    while (lua_next(L, progs) != 0) {
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) return bad("a floor program is not name = {...}");
        FloorProgram prog;
        prog.name = lua_tostring(L, -2);
        const std::string where = "floor_programs." + prog.name;
        const int pt = lua_gettop(L);
        lua_getfield(L, pt, "lots");
        if (lua_istable(L, -1)) {
            const int t = lua_gettop(L);
            FloorZoneParams& z = prog.zones;
            z.windowDepth = numField(L, t, "window_depth", z.windowDepth);
            z.aisle = numField(L, t, "aisle", z.aisle);
            z.street = numField(L, t, "street", z.street);
            z.lotWidth = numField(L, t, "lot_width", z.lotWidth);
            z.interiorLot = numField(L, t, "interior_lot", z.interiorLot);
            z.minSide = numField(L, t, "min_side", z.minSide);
        }
        lua_pop(L, 1);
        if (const std::string f = strField(L, pt, "fill"); !f.empty()) prog.fill = f;
        if (const std::string f = strField(L, pt, "shallow"); !f.empty()) prog.shallow = f;
        prog.shallowDepth = numField(L, pt, "shallow_depth", prog.shallowDepth);
        lua_getfield(L, pt, "zones");
        if (!lua_istable(L, -1)) return bad(where + ": no zones");
        {
            const int zt = lua_gettop(L), n = static_cast<int>(luaL_len(L, zt));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, zt, i);
                const int t = lua_gettop(L);
                const std::string w = where + ".zones[" + std::to_string(i) + "]";
                if (!lua_istable(L, t)) return bad(w + ": not a table");
                ZoneRule r;
                r.tag = strField(L, t, "tag");
                if (r.tag.empty()) return bad(w + ": no tag");
                const std::string kind = strField(L, t, "kind");
                if (!roomKindFromName(kind.empty() ? r.tag : kind, r.kind)) return bad(w + ": unknown kind '" + kind + "'");
                lua_getfield(L, t, "sites");
                if (lua_istable(L, -1)) {
                    r.sites = 0;
                    const int st = lua_gettop(L), sn = static_cast<int>(luaL_len(L, st));
                    for (int k = 1; k <= sn; ++k) {
                        lua_rawgeti(L, st, k);
                        const std::string site = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
                        lua_pop(L, 1);
                        if (site == "window") r.sites |= zoneSiteBit(ZoneSite::Window);
                        else if (site == "corner") r.sites |= zoneSiteBit(ZoneSite::Corner);
                        else if (site == "interior") r.sites |= zoneSiteBit(ZoneSite::Interior);
                        else return bad(w + ": unknown site '" + site + "'");
                    }
                }
                lua_pop(L, 1);
                r.count = static_cast<int>(numField(L, t, "count", 0));
                r.perM2 = numField(L, t, "per_m2", 0);
                r.share = numField(L, t, "share", 0);
                r.minDepth = numField(L, t, "min_depth", 0);
                lua_getfield(L, t, "area");   // {min, max}
                if (lua_istable(L, -1)) {
                    lua_rawgeti(L, -1, 1);
                    if (lua_isnumber(L, -1)) r.minArea = lua_tonumber(L, -1);
                    lua_pop(L, 1);
                    lua_rawgeti(L, -1, 2);
                    if (lua_isnumber(L, -1)) r.maxArea = lua_tonumber(L, -1);
                    lua_pop(L, 1);
                }
                lua_pop(L, 1);
                r.enclosed = boolField(L, t, "enclosed", false);
                r.lifts = boolField(L, t, "lifts", false);
                const std::string walls = strField(L, t, "walls");
                if (walls.empty() || walls == "facade") r.walls = 2;
                else if (walls == "glass") r.walls = 1;
                else if (walls == "drywall") r.walls = 0;
                else return bad(w + ": walls must be glass, drywall or facade");
                const std::string near = strField(L, t, "near");
                if (near.empty()) r.near = 0;
                else if (near == "lobby") r.near = 1;
                else if (near == "core") r.near = 2;
                else if (near == "far") r.near = 3;
                else return bad(w + ": near must be lobby, core or far");
                prog.rules.push_back(r);
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        bool haveFill = false;
        for (const ZoneRule& r : prog.rules) haveFill = haveFill || r.tag == prog.fill;
        if (!haveFill) return bad(where + ": the fill tag '" + prog.fill + "' has no zone");
        lua_getfield(L, pt, "styles");
        if (lua_istable(L, -1)) {
            const int sl = lua_gettop(L), n = static_cast<int>(luaL_len(L, sl));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, sl, i);
                const int t = lua_gettop(L);
                if (!lua_istable(L, t)) return bad(where + ".styles: not a table");
                ZoneStyle st;
                st.name = strField(L, t, "name");
                st.weight = numField(L, t, "weight", 1.0);
                lua_getfield(L, t, "set");   // { tag = { field = value, ... }, ... }
                if (lua_istable(L, -1)) {
                    const int setT = lua_gettop(L);
                    lua_pushnil(L);
                    while (lua_next(L, setT) != 0) {
                        if (lua_isstring(L, -2) && lua_istable(L, -1)) {
                            const std::string tag = lua_tostring(L, -2);
                            const int ft = lua_gettop(L);
                            lua_pushnil(L);
                            while (lua_next(L, ft) != 0) {
                                if (lua_isstring(L, -2)) {
                                    ZoneStyle::Set s;
                                    s.tag = tag;
                                    s.field = lua_tostring(L, -2);
                                    s.value = lua_isboolean(L, -1) ? (lua_toboolean(L, -1) ? 1.0 : 0.0) : lua_tonumber(L, -1);
                                    st.sets.push_back(s);
                                }
                                lua_pop(L, 1);
                            }
                        }
                        lua_pop(L, 1);
                    }
                }
                lua_pop(L, 1);
                // (deterministic whatever order the table walked in)
                std::sort(st.sets.begin(), st.sets.end(), [](const ZoneStyle::Set& a, const ZoneStyle::Set& b) {
                    return a.tag != b.tag ? a.tag < b.tag : a.field < b.field;
                });
                prog.styles.push_back(st);
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        out.set(std::move(prog));
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    return true;
}

bool loadFloorProgramsSource(const std::string& source, FloorProgramBook& out, std::string* err) {
    ScriptVM vm;
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "floor_programs.lua: " + e);
    return readFloorPrograms(vm, out, err);
}

bool ensureFurnitureLibraryLoaded() {
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, [] {
        std::string src = loadScriptCode("furniture_library.lua", "");
        if (src.empty()) { LOG_WARN << "[furniture] no furniture_library.lua: nothing is interactive"; return; }
        // the room programs ride along: one VM, both globals (furniture_rooms.lua, M3b)
        const std::string rooms = loadScriptCode("furniture_rooms.lua", "");
        if (rooms.empty()) LOG_WARN << "[furniture] no furniture_rooms.lua: rooms stay unfurnished";
        else src += "\n" + rooms;
        std::string err;
        FurnitureLibrary lib;
        if (!loadFurnitureLibrarySource(src, lib, &err)) { LOG_WARN << "[furniture] " << err; return; }
        FurnitureLibrary::global() = lib;
        ok = true;
        // the floor programs (floor_zones.h): absent, the built-in office program stands
        const std::string floors = loadScriptCode("floor_programs.lua", "");
        if (!floors.empty()) {
            FloorProgramBook book;
            std::string ferr;
            if (loadFloorProgramsSource(floors, book, &ferr)) {
                FloorProgramBook::global() = book;
                LOG_INFO << "[floors] " << book.size() << " floor programs";
            } else {
                LOG_WARN << "[floors] " << ferr << " (the built-in office program stands)";
            }
        }
        LOG_INFO << "[furniture] library: " << lib.size() << " described pieces, " << lib.programCount() << " room programs";
    });
    return ok;
}

}  // namespace engine
