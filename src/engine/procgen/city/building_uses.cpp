#include "building_uses.h"

#include "architect.h"   // civicRoomsFor
#include "../../script_assets.h"
#include "../../../log.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../../scripting/lua_state.h"
#include "../../scripting/script_vm.h"
#endif

#include <chrono>
#include <filesystem>
#include <map>
#include <mutex>

namespace engine {

namespace {

struct UseBook {
    std::map<std::string, std::string> byRecipe, byPlaceType;
    bool loaded = false;
    std::string path;
    std::filesystem::file_time_type stamp{};
    std::chrono::steady_clock::time_point checked{};
    int generation = 0;
};

std::mutex& bookMutex() { static std::mutex m; return m; }
UseBook& book() { static UseBook b; return b; }

#ifdef RT_ENABLE_SCRIPTING
// building_uses = { defaults = { placeType = use, ... }, recipes = { name = use, ... } }
bool readBook(const std::string& source, UseBook& out, std::string& err) {
    ScriptVM vm;
    if (!vm.doString(source, &err)) return false;
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "building_uses");
    if (!lua_istable(L, -1)) { err = "`building_uses` is not a table"; lua_settop(L, base); return false; }
    auto readMap = [&](const char* field, std::map<std::string, std::string>& into) {
        lua_getfield(L, -1, field);
        if (lua_istable(L, -1)) {
            lua_pushnil(L);
            while (lua_next(L, -2)) {
                if (lua_type(L, -2) == LUA_TSTRING && lua_type(L, -1) == LUA_TSTRING)
                    into[lua_tostring(L, -2)] = lua_tostring(L, -1);
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
    };
    readMap("defaults", out.byPlaceType);
    readMap("recipes", out.byRecipe);
    lua_settop(L, base);
    return true;
}
#endif

// Load the book on first use, and again whenever the file changes (looked at no more than once a second): an edit
// while the game runs reaches the next interior planned.
void refresh(UseBook& b) {
    const auto now = std::chrono::steady_clock::now();
    if (b.loaded && now - b.checked < std::chrono::seconds(1)) return;
    b.checked = now;
    const std::string path = resolveScriptPath("building_uses.lua", "");
    std::error_code ec;
    const auto stamp = path.empty() ? std::filesystem::file_time_type{} : std::filesystem::last_write_time(path, ec);
    if (b.loaded && path == b.path && stamp == b.stamp) return;
    const bool first = !b.loaded;
    b.loaded = true;
    b.path = path;
    b.stamp = stamp;
    if (path.empty()) {
        if (first) LOG_WARN << "[uses] no building_uses.lua: every building takes its place type's use";
        return;
    }
#ifdef RT_ENABLE_SCRIPTING
    UseBook next;
    std::string err;
    if (!readBook(loadScriptCode("building_uses.lua", ""), next, err)) {
        LOG_WARN << "[uses] building_uses.lua: " << err << (first ? "" : " (the uses before the edit stand)");
        return;
    }
    b.byRecipe = std::move(next.byRecipe);
    b.byPlaceType = std::move(next.byPlaceType);
    ++b.generation;
    LOG_INFO << "[uses] " << b.byRecipe.size() << " recipes' uses" << (first ? "" : " (reloaded)");
#else
    if (first) LOG_WARN << "[uses] built without Lua: every building takes its place type's use";
#endif
}

}  // namespace

std::string floorUseFor(const std::string& recipe, const std::string& placeType) {
    std::lock_guard<std::mutex> lock(bookMutex());
    UseBook& b = book();
    refresh(b);
    if (auto it = b.byRecipe.find(recipe); it != b.byRecipe.end()) return it->second;
    if (auto it = b.byPlaceType.find(placeType); it != b.byPlaceType.end()) return it->second;
    // (no book: the place type alone)
    return placeType == "home" ? "apartments" : placeType == "depot" ? "none" : "office";
}

bool floorUseListed(const std::string& recipe) {
    std::lock_guard<std::mutex> lock(bookMutex());
    UseBook& b = book();
    refresh(b);
    return b.byRecipe.count(recipe) > 0;
}

int buildingUsesGeneration() {
    std::lock_guard<std::mutex> lock(bookMutex());
    UseBook& b = book();
    refresh(b);
    return b.generation;
}

void applyFloorUse(BuildingParams& p, const std::string& recipe, const std::string& placeType) {
    const std::string use = floorUseFor(recipe, placeType);
    p.house = use == "house";
    p.officeUse = use == "office" && !p.residential;   // (homes over a shop are homes: the recipe says so)
}

BuildingParams interiorParamsFor(const BuildingRecord& r) {
    BuildingParams params = r.params;
    applyFloorUse(params, r.recipe, r.type);
    params.civicRooms = civicRoomsFor(r.recipe);
    return params;
}

}  // namespace engine
