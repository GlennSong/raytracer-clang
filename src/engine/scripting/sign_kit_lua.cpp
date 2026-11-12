#include "sign_kit_lua.h"

#include "lua_state.h"
#include "lua_json.h"
#include "procgen_bindings.h"
#include "procgen_mesh.h"   // luaToMesh
#include "script_modules.h"
#include "script_vm.h"
#include "../script_assets.h"
#include "../procgen/world/road_sign_build.h"   // rasterizeSignFace, packSignImages
#include "../text/font.h"

#include <cmath>
#include <map>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

double num(const nlohmann::json& j, const char* key, double def) {
    const nlohmann::json::const_iterator it = j.find(key);
    return it != j.end() && it->is_number() ? it->get<double>() : def;
}
std::string str(const nlohmann::json& j, const char* key, const std::string& def) {
    const nlohmann::json::const_iterator it = j.find(key);
    return it != j.end() && it->is_string() ? it->get<std::string>() : def;
}

}  // namespace

SignFace signFaceFromJson(const nlohmann::json& j) {
    SignFace f;
    f.w = num(j, "w", 1.0);
    f.h = num(j, "h", 1.0);
    f.background = str(j, "background", "none");
    const nlohmann::json::const_iterator elems = j.find("elems");
    if (elems == j.end() || !elems->is_array()) return f;
    for (const nlohmann::json& e : *elems) {
        SignElem s;
        s.kind = str(e, "kind", "");
        s.x = num(e, "x", 0); s.y = num(e, "y", 0); s.w = num(e, "w", 0); s.h = num(e, "h", 0);
        s.text = str(e, "text", "");
        s.cap = num(e, "cap", 0);
        s.color = str(e, "color", "#ffffff");
        s.anchor = str(e, "anchor", "start");
        s.angle = num(e, "angle", 0);
        s.radius = num(e, "radius", 0);
        s.xScale = num(e, "xscale", 1.0);
        const nlohmann::json::const_iterator rings = e.find("rings");
        if (rings != e.end() && rings->is_array())
            for (const nlohmann::json& ring : *rings) {
                s.rings.emplace_back();
                if (ring.is_array())
                    for (const nlohmann::json& p : ring)
                        if (p.is_array() && p.size() >= 2) s.rings.back().push_back({p[0].get<double>(), p[1].get<double>()});
            }
        f.elems.push_back(std::move(s));
    }
    return f;
}

struct SignKit::Impl {
    ScriptVM vm;
    bool loaded = false;
    const Font* font = nullptr;   // what sign_text measures with (set for each build)
};

namespace {

// sign_text.width(text, cap, x_scale) -> metres: how wide `text` is set with capitals `cap` metres tall
int l_sign_text_width(lua_State* L) {
    const Font* const* font = static_cast<const Font* const*>(lua_touserdata(L, lua_upvalueindex(1)));
    const char* text = luaL_checkstring(L, 1);
    const double cap = luaL_checknumber(L, 2);
    const double xs = luaL_optnumber(L, 3, 1.0);
    if (!font || !*font) return luaL_error(L, "sign_text: no font");
    const double capRatio = (*font)->capHeight(100.0f) / 100.0;
    lua_pushnumber(L, (*font)->measure(text, 100.0f, static_cast<float>(xs)) / 100.0 * cap / capRatio);
    return 1;
}

// Call catalogue.<fn>(args...) with `nargs` values already pushed; leaves one result on the stack.
bool callCatalogue(lua_State* L, const char* fn, int nargs, std::string* err) {
    lua_getglobal(L, "__sign_catalogue");
    if (!lua_istable(L, -1)) { lua_pop(L, 1 + nargs); return fail(err, "no sign catalogue loaded"); }
    lua_getfield(L, -1, fn);
    lua_remove(L, -2);
    if (!lua_isfunction(L, -1)) { lua_pop(L, 1 + nargs); return fail(err, std::string("the catalogue has no ") + fn + "()"); }
    lua_insert(L, -1 - nargs);
    if (lua_pcall(L, nargs, 1, 0) != LUA_OK) {
        const std::string msg = lua_tostring(L, -1) ? lua_tostring(L, -1) : "error";
        lua_pop(L, 1);
        return fail(err, std::string(fn) + ": " + msg);
    }
    return true;
}

nlohmann::json catalogueField(lua_State* L, const char* key) {
    lua_getglobal(L, "__sign_catalogue");
    nlohmann::json out;
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, key);
        out = luaToJson(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return out;
}

}  // namespace

SignKit::SignKit() : impl_(std::make_unique<Impl>()) {}
SignKit::~SignKit() = default;

bool SignKit::load(const std::string& file, const std::string& levelDir, std::string* err) {
    const std::string src = loadScriptCode(file, levelDir);
    if (src.empty()) return fail(err, file + " not found");
    return loadSource(src, levelDir, err);
}

bool SignKit::loadSource(const std::string& source, const std::string& levelDir, std::string* err) {
    impl_ = std::make_unique<Impl>();
    openProcgenLibrary(impl_->vm);
    openModuleLoader(impl_->vm, makeModuleSource(levelDir));
    lua_State* L = luaState(impl_->vm);
    impl_->font = signFont();
    lua_newtable(L);
    lua_pushlightuserdata(L, &impl_->font);
    lua_pushcclosure(L, l_sign_text_width, 1);
    lua_setfield(L, -2, "width");
    lua_setglobal(L, "sign_text");
    if (!impl_->vm.doString("__sign_catalogue = (function() " + source + "\nend)()", err)) return false;
    lua_getglobal(L, "__sign_catalogue");
    const bool ok = lua_istable(L, -1);
    lua_pop(L, 1);
    if (!ok) return fail(err, "the sign catalogue returned no table");
    impl_->loaded = true;
    return true;
}

std::vector<std::string> SignKit::names() {
    std::vector<std::string> out;
    lua_State* L = luaState(impl_->vm);
    if (!impl_->loaded || !callCatalogue(L, "names", 0, nullptr)) return out;
    const nlohmann::json j = luaToJson(L, -1);
    lua_pop(L, 1);
    if (j.is_array())
        for (const nlohmann::json& n : j) if (n.is_string()) out.push_back(n.get<std::string>());
    return out;
}

std::vector<nlohmann::json> SignKit::examples() {
    std::vector<nlohmann::json> out;
    lua_State* L = luaState(impl_->vm);
    if (!impl_->loaded || !callCatalogue(L, "examples", 0, nullptr)) return out;
    const nlohmann::json j = luaToJson(L, -1);
    lua_pop(L, 1);
    if (j.is_array()) out.assign(j.begin(), j.end());
    return out;
}

bool SignKit::build(const std::vector<nlohmann::json>& requests, const Font& font, SignKitBuild& out, std::string* err,
                    const SignKitOptions& opt) {
    if (!impl_->loaded) return fail(err, "no sign catalogue loaded");
    impl_->font = &font;
    lua_State* L = luaState(impl_->vm);
    const int base = lua_gettop(L);
    out = SignKitBuild();
    out.sheeting = catalogueField(L, "sheeting");
    SignWearLook look;
    {
        const nlohmann::json w = catalogueField(L, "wear_look");
        look.fade = num(w, "fade", look.fade);
        look.fadeGrey = num(w, "fade_grey", look.fadeGrey);
        look.fadeLift = num(w, "fade_lift", look.fadeLift);
        look.fadeCell = num(w, "fade_cell", look.fadeCell);
        look.dirt = num(w, "dirt", look.dirt);
        look.dirtCell = num(w, "dirt_cell", look.dirtCell);
        look.streak = num(w, "streak", look.streak);
        look.dirtLow = num(w, "dirt_low", look.dirtLow);
        look.shineLoss = num(w, "shine_loss", look.shineLoss);
        const nlohmann::json::const_iterator c = w.find("dirt_color");
        if (c != w.end() && c->is_array() && c->size() >= 3) {
            look.dirtR = (*c)[0].get<double>(); look.dirtG = (*c)[1].get<double>(); look.dirtB = (*c)[2].get<double>();
        }
    }
    // 1. every request's faces, the same face (content, wear and seed) once
    std::map<std::string, int> faceIndex;
    std::vector<nlohmann::json> faceJson;
    std::vector<std::vector<int>> facesOf(requests.size());
    for (std::size_t r = 0; r < requests.size(); ++r) {
        pushJson(L, requests[r]);
        if (!callCatalogue(L, "faces", 1, err)) { lua_settop(L, base); return fail(err, requests[r].dump() + ": " + (err ? *err : "")); }
        const nlohmann::json fs = luaToJson(L, -1);
        lua_pop(L, 1);
        if (!fs.is_array()) { lua_settop(L, base); return fail(err, requests[r].dump() + ": faces() returned no list"); }
        for (const nlohmann::json& f : fs) {
            const std::string key = f.dump();
            std::map<std::string, int>::const_iterator it = faceIndex.find(key);
            if (it == faceIndex.end()) {
                it = faceIndex.emplace(key, static_cast<int>(faceJson.size())).first;
                faceJson.push_back(f);
            }
            facesOf[r].push_back(it->second);
        }
    }
    // 2. drawn, worn and packed: colour and shine share one layout
    const double k = opt.pxPerMetre;
    std::vector<TextImage> colour, shine;
    std::vector<double> tabs;
    for (const nlohmann::json& f : faceJson) {
        out.faces.push_back(signFaceFromJson(f));
        out.faceLabels.push_back(str(f, "label", ""));
        double tab = 0;
        colour.push_back(rasterizeSignFace(out.faces.back(), font, k, &tab));
        tabs.push_back(tab);
        nlohmann::json s = f.contains("shine") ? f["shine"] : nlohmann::json::object();
        s["w"] = out.faces.back().w;
        s["h"] = out.faces.back().h;
        shine.push_back(rasterizeSignFace(signFaceFromJson(s), font, k));
        const double wear = num(f, "wear", 0.0);
        const uint32_t seed = static_cast<uint32_t>(static_cast<long long>(num(f, "seed", 0.0)));
        wearSignFace(colour.back(), &shine.back(), wear, seed, look, k);
    }
    std::vector<SignImagePlace> places, shinePlaces;
    out.pages = packSignImages(colour, opt.pageSize, places);
    out.shinePages = packSignImages(shine, opt.pageSize, shinePlaces);
    nlohmann::json slots = nlohmann::json::array();
    const double P = opt.pageSize;
    for (std::size_t i = 0; i < faceJson.size(); ++i) {
        const SignImagePlace& p = places[i];
        if (p.page < 0 || shinePlaces[i].page != p.page || shinePlaces[i].x != p.x || shinePlaces[i].y != p.y) {
            lua_settop(L, base);
            return fail(err, "face " + out.faceLabels[i] + " does not fit a page (or its shine landed elsewhere)");
        }
        // the panel inside its canvas (the canvas sits at x + 1, y + 1 with a pixel of margin; an exit tab above it)
        const double v0 = (p.y + 2 + tabs[i] * k) / P;
        slots.push_back({{"page", p.page}, {"u0", (p.x + 2) / P}, {"u1", (p.x + 2 + out.faces[i].w * k) / P}, {"v0", v0},
                         {"v1", v0 + out.faces[i].h * k / P}, {"tab_v0", (p.y + 2) / P}});
    }
    // 3. the models, each on its own faces' slots
    for (std::size_t r = 0; r < requests.size(); ++r) {
        nlohmann::json mine = nlohmann::json::array();
        for (int f : facesOf[r]) mine.push_back(slots[static_cast<std::size_t>(f)]);
        pushJson(L, requests[r]);
        pushJson(L, mine);
        if (!callCatalogue(L, "model", 2, err)) { lua_settop(L, base); return fail(err, requests[r].dump() + ": " + (err ? *err : "")); }
        const int t = lua_gettop(L);
        if (!lua_istable(L, t)) { lua_settop(L, base); return fail(err, requests[r].dump() + ": model() returned no table"); }
        SignModel m;
        lua_getfield(L, t, "label");
        if (lua_isstring(L, -1)) m.label = lua_tostring(L, -1);
        lua_pop(L, 1);
        lua_getfield(L, t, "info");
        m.info = luaToJson(L, -1);
        lua_pop(L, 1);
        lua_getfield(L, t, "parts");
        if (lua_istable(L, -1)) {
            const lua_Integer n = luaL_len(L, -1);
            for (lua_Integer i = 1; i <= n; ++i) {
                lua_geti(L, -1, i);
                const int pt = lua_gettop(L);
                SignModelPart part;
                lua_getfield(L, pt, "name");
                if (lua_isstring(L, -1)) part.name = lua_tostring(L, -1);
                lua_pop(L, 1);
                lua_getfield(L, pt, "face");
                if (lua_isinteger(L, -1)) {
                    const lua_Integer fi = lua_tointeger(L, -1);
                    if (fi >= 1 && static_cast<std::size_t>(fi) <= facesOf[r].size())
                        part.page = places[static_cast<std::size_t>(facesOf[r][static_cast<std::size_t>(fi - 1)])].page;
                }
                lua_pop(L, 1);
                // its material, as the catalogue resolved it: {name, metallic, roughness, color}
                lua_getfield(L, pt, "material");
                const nlohmann::json mat = luaToJson(L, -1);
                lua_pop(L, 1);
                if (!mat.is_object()) {
                    lua_settop(L, base);
                    return fail(err, requests[r].dump() + ": part " + part.name + " names no material");
                }
                part.materialName = str(mat, "name", "");
                part.material.metallic = static_cast<float>(num(mat, "metallic", part.material.metallic));
                part.material.roughness = static_cast<float>(num(mat, "roughness", part.material.roughness));
                const nlohmann::json::const_iterator col = mat.find("color");
                if (col != mat.end() && col->is_array() && col->size() >= 3)
                    part.color = Vec3((*col)[0].get<double>(), (*col)[1].get<double>(), (*col)[2].get<double>());
                lua_getfield(L, pt, "mesh");
                const std::shared_ptr<RenderMesh> mesh = luaToMesh(L, -1);
                lua_pop(L, 1);
                if (mesh) {
                    part.mesh = *mesh;
                    m.parts.push_back(std::move(part));
                }
                lua_pop(L, 1);
            }
        }
        lua_settop(L, base);
        out.models.push_back(std::move(m));
    }
    lua_settop(L, base);
    return true;
}

}  // namespace engine
