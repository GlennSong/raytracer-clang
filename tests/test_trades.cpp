// THE TRADES AS DATA (ADR-0360): assets/scripts/trades.lua is the built-in table, field for field, in its order --
// so moving the table to Lua changed no shop on any street -- and a broken file is refused, not half-read.

#include "test_framework.h"
#include "../src/engine/procgen/city/trades.h"
#include "../src/engine/scripting/trades_lua.h"
#include <fstream>
#include <sstream>

using namespace engine;

namespace {
std::string readTradesFile() {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/trades.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
bool sameVec(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool sameRoom(const TradeInterior& a, const TradeInterior& b) {
    for (int i = 0; i < 3; ++i) if (!sameVec(a.walls[i], b.walls[i])) return false;
    return a.dadoH == b.dadoH && (a.dadoH == 0 || sameVec(a.dado, b.dado)) && sameVec(a.floorA, b.floorA) &&
           a.tile == b.tile && (a.tile == 0 || sameVec(a.floorB, b.floorB)) && sameVec(a.ceiling, b.ceiling) &&
           a.lights == b.lights;
}
}  // namespace

TEST_CASE(trades_lua_is_the_built_in_trades) {
    TradeBook lua;
    std::string err;
    CHECK(loadTradesSource(readTradesFile(), lua, &err));
    if (!err.empty()) std::printf("    %s\n", err.c_str());
    const TradeBook& c = builtinTradeBook();
    CHECK(lua.trades.size() == c.trades.size());
    for (std::size_t i = 0; i < std::min(lua.trades.size(), c.trades.size()); ++i) {
        const TradeInfo &a = lua.trades[i], &b = c.trades[i];
        const bool same = a.id == b.id && a.name == b.name && a.placeType == b.placeType && a.openHour == b.openHour &&
                          a.closeHour == b.closeHour && a.weight[0] == b.weight[0] && a.weight[1] == b.weight[1] &&
                          a.weight[2] == b.weight[2] && a.weight[3] == b.weight[3] && a.minBays == b.minBays &&
                          a.maxBays == b.maxBays && sameVec(a.fascia, b.fascia) && a.terrace == b.terrace;
        if (!same) std::printf("    trade %zu: lua `%s` != built-in `%s`\n", i, a.name.c_str(), b.name.c_str());
        CHECK(same);
    }
    CHECK(lua.rooms.size() == c.rooms.size());
    for (const TradeBook::Room& r : c.rooms) {   // (by id: the file lists a trade's room with the trade)
        const TradeBook::Room* m = nullptr;
        for (const TradeBook::Room& q : lua.rooms) if (q.id == r.id) m = &q;
        if (!m || !sameRoom(m->in, r.in)) std::printf("    room %d differs\n", r.id);
        CHECK(m && sameRoom(m->in, r.in));
    }
    CHECK(sameRoom(lua.plain, c.plain));
}

TEST_CASE(a_broken_trades_file_is_refused) {
    TradeBook b;
    b.trades.push_back(TradeInfo{});
    std::string err;
    CHECK(!loadTradesSource("return { plain_room = { walls = {} }, trades = {} }", b, &err));
    CHECK(!err.empty());
    CHECK(b.trades.size() == 1);   // untouched
    err.clear();
    const std::string twice = "return { plain_room = { walls = { {1,1,1},{1,1,1},{1,1,1} } }, trades = {"
                              " { id = 1, name = 'a', place = 'shop', hours = {9, 17}, bays = {2, 2} },"
                              " { id = 1, name = 'b', place = 'shop', hours = {9, 17}, bays = {2, 2} } } }";
    CHECK(!loadTradesSource(twice, b, &err));   // an id taken twice
    std::printf("    [trades] refused: %s\n", err.c_str());
}
