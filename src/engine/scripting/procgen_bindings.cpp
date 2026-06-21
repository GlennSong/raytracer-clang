#include "procgen_bindings.h"

#include "procgen_mesh.h"
#include "lua_state.h"
#include "../procgen/sdf.h"
#include "../procgen/noise.h"
#include "../procgen/lsystem.h"
#include "../procgen/tree.h"
#include "../procgen/terrain.h"
#include "../procgen/scatter.h"
#include "../procgen/city/shape_grammar.h"
#include "../procgen/city/street_kit.h"
#include "../procgen/city/road_network.h"
#include "../procgen/city/road_mesh.h"
#include "../procgen/city/polygon.h"
#include "../procgen/city/parcel.h"
#include "../procgen/proc_model.h"
#include "../procgen/texture_field.h"
#include "../procgen/terrain_field.h"
#include "../procgen/erosion.h"
#include "../mesh_builder.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <memory>
#include <new>
#include <utility>
#include <vector>

namespace engine {
namespace {

// Wrapped C++ values cross the C boundary as full userdata, each with a
// metatable whose __gc destroys the held object. The names are unique registry
// keys (luaL_newmetatable / luaL_checkudata).
constexpr const char* kSdfMt = "engine.procgen.Sdf";
constexpr const char* kMeshMt = "engine.procgen.Mesh";

using MeshPtr = std::shared_ptr<RenderMesh>;

// --- Sdf userdata (a Field) ---

void pushSdf(lua_State* L, Sdf sdf) {
    void* mem = lua_newuserdatauv(L, sizeof(Sdf), 0);
    new (mem) Sdf(std::move(sdf));
    luaL_setmetatable(L, kSdfMt);
}

Sdf& checkSdf(lua_State* L, int idx) {
    return *static_cast<Sdf*>(luaL_checkudata(L, idx, kSdfMt));
}

int sdfGc(lua_State* L) {
    static_cast<Sdf*>(lua_touserdata(L, 1))->~Sdf();
    return 0;
}

// --- Mesh userdata ---

void pushMesh(lua_State* L, MeshPtr mesh) {
    void* mem = lua_newuserdatauv(L, sizeof(MeshPtr), 0);
    new (mem) MeshPtr(std::move(mesh));
    luaL_setmetatable(L, kMeshMt);
}

int meshGc(lua_State* L) {
    static_cast<MeshPtr*>(lua_touserdata(L, 1))->~MeshPtr();
    return 0;
}

RenderMesh& checkMesh(lua_State* L, int idx) {
    return **static_cast<MeshPtr*>(luaL_checkudata(L, idx, kMeshMt));
}

// --- LSystem userdata (the grammar; ADR-0021 Phase B.1) ---

constexpr const char* kLSystemMt = "engine.procgen.LSystem";

LSystem& checkLSystem(lua_State* L, int idx) {
    return *static_cast<LSystem*>(luaL_checkudata(L, idx, kLSystemMt));
}

int lsystemGc(lua_State* L) {
    static_cast<LSystem*>(lua_touserdata(L, 1))->~LSystem();
    return 0;
}

// --- ParametricLSystem + ModuleString userdata (grammar in Lua; ADR-0030) ---

constexpr const char* kPLSystemMt = "engine.procgen.ParametricLSystem";
constexpr const char* kModulesMt = "engine.procgen.ModuleString";

ParametricLSystem& checkPLSystem(lua_State* L, int idx) {
    return *static_cast<ParametricLSystem*>(luaL_checkudata(L, idx, kPLSystemMt));
}
int plsystemGc(lua_State* L) {
    static_cast<ParametricLSystem*>(lua_touserdata(L, 1))->~ParametricLSystem();
    return 0;
}

void pushModules(lua_State* L, ModuleString m) {
    void* mem = lua_newuserdatauv(L, sizeof(ModuleString), 0);
    new (mem) ModuleString(std::move(m));
    luaL_setmetatable(L, kModulesMt);
}
ModuleString& checkModules(lua_State* L, int idx) {
    return *static_cast<ModuleString*>(luaL_checkudata(L, idx, kModulesMt));
}
int modulesGc(lua_State* L) {
    static_cast<ModuleString*>(lua_touserdata(L, 1))->~ModuleString();
    return 0;
}

// --- argument helpers ---

// Read an optional numeric field `key` from the table at `idx` (default if
// absent). Used to turn a Lua params table into a C++ params struct.
double optField(lua_State* L, int idx, const char* key, double fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    double v = luaL_optnumber(L, -1, fallback);
    lua_pop(L, 1);
    return v;
}

// A Vec3 is a 3-element Lua array, e.g. {x, y, z}.
Vec3 checkVec3(lua_State* L, int idx) {
    idx = lua_absindex(L, idx);  // we push while reading; pin the table index
    luaL_checktype(L, idx, LUA_TTABLE);
    Vec3 v;
    lua_geti(L, idx, 1); v.x = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 2); v.y = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 3); v.z = luaL_checknumber(L, -1); lua_pop(L, 1);
    return v;
}

void pushVec3(lua_State* L, const Vec3& v) {
    lua_createtable(L, 3, 0);
    lua_pushnumber(L, v.x); lua_seti(L, -2, 1);
    lua_pushnumber(L, v.y); lua_seti(L, -2, 2);
    lua_pushnumber(L, v.z); lua_seti(L, -2, 3);
}

// --- sdf.* ---

int l_sdf_sphere(lua_State* L) {
    Vec3 c = checkVec3(L, 1);
    double r = luaL_checknumber(L, 2);
    pushSdf(L, sdfSphere(c, r));
    return 1;
}

int l_sdf_box(lua_State* L) {
    Vec3 c = checkVec3(L, 1);
    Vec3 h = checkVec3(L, 2);
    pushSdf(L, sdfBox(c, h));
    return 1;
}

int l_sdf_capsule(lua_State* L) {
    Vec3 a = checkVec3(L, 1);
    Vec3 b = checkVec3(L, 2);
    double r = luaL_checknumber(L, 3);
    pushSdf(L, sdfCapsule(a, b, r));
    return 1;
}

int l_sdf_union(lua_State* L) {
    pushSdf(L, sdfUnion(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_intersect(lua_State* L) {
    pushSdf(L, sdfIntersect(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_subtract(lua_State* L) {
    pushSdf(L, sdfSubtract(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_smooth_union(lua_State* L) {
    Sdf a = checkSdf(L, 1);
    Sdf b = checkSdf(L, 2);
    double k = luaL_checknumber(L, 3);
    pushSdf(L, sdfSmoothUnion(a, b, k));
    return 1;
}
// smooth_union_all({f1, f2, ...}, k) — fold a list (avoids deep closure nesting).
int l_sdf_smooth_union_all(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    double k = luaL_checknumber(L, 2);
    std::vector<Sdf> parts;
    lua_Integer n = luaL_len(L, 1);
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_geti(L, 1, i);
        parts.push_back(checkSdf(L, -1));
        lua_pop(L, 1);
    }
    luaL_argcheck(L, !parts.empty(), 1, "need at least one field");
    pushSdf(L, sdfSmoothUnion(parts, k));
    return 1;
}

// --- noise.* (seed-first so the surface stays a pure function) ---

int l_noise_value2(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    lua_pushnumber(L, Noise(seed).noise2(x, y));
    return 1;
}

int l_noise_fbm2(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    int octaves = static_cast<int>(luaL_optinteger(L, 4, 4));
    lua_pushnumber(L, Noise(seed).fbm2(x, y, octaves));
    return 1;
}

int l_noise_fbm3(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    double z = luaL_checknumber(L, 4);
    int octaves = static_cast<int>(luaL_optinteger(L, 5, 4));
    lua_pushnumber(L, Noise(seed).fbm3(x, y, z, octaves));
    return 1;
}

// --- polygonize(field, {min=, max=}, resolution) -> Mesh ---

int l_polygonize(lua_State* L) {
    Sdf field = checkSdf(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    lua_getfield(L, 2, "min"); Vec3 mn = checkVec3(L, -1); lua_pop(L, 1);
    lua_getfield(L, 2, "max"); Vec3 mx = checkVec3(L, -1); lua_pop(L, 1);
    int res = static_cast<int>(luaL_checkinteger(L, 3));
    luaL_argcheck(L, res >= 2, 3, "resolution must be >= 2");

    RenderMesh mesh = polygonizeSdf(field, SdfBounds{mn, mx}, res);
    pushMesh(L, std::make_shared<RenderMesh>(std::move(mesh)));
    return 1;
}

// --- mesh.* : primitives + assembly (the Mesh value type; ROADMAP 3.3) ---

int l_mesh_box(lua_State* L) {
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::box(checkVec3(L, 1))));
    return 1;
}
int l_mesh_sphere(lua_State* L) {
    pushMesh(L, std::make_shared<RenderMesh>(
                    MeshBuilder::sphere(static_cast<float>(luaL_checknumber(L, 1)))));
    return 1;
}
int l_mesh_cylinder(lua_State* L) {
    auto r = static_cast<float>(luaL_checknumber(L, 1));
    auto h = static_cast<float>(luaL_checknumber(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::cylinder(r, h)));
    return 1;
}
int l_mesh_cone(lua_State* L) {
    auto r = static_cast<float>(luaL_checknumber(L, 1));
    auto h = static_cast<float>(luaL_checknumber(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::cone(r, h)));
    return 1;
}
int l_mesh_plane(lua_State* L) {
    auto w = static_cast<float>(luaL_checknumber(L, 1));
    auto d = static_cast<float>(luaL_checknumber(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::plane(w, d)));
    return 1;
}
int l_mesh_torus(lua_State* L) {
    auto bigR = static_cast<float>(luaL_checknumber(L, 1));
    auto smallR = static_cast<float>(luaL_checknumber(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::torus(bigR, smallR)));
    return 1;
}
int l_mesh_capsule(lua_State* L) {
    auto r = static_cast<float>(luaL_checknumber(L, 1));
    auto h = static_cast<float>(luaL_checknumber(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::capsule(r, h)));
    return 1;
}

// Assembly ops each return a NEW mesh (copy + apply), so scripts compose by
// dataflow — the same shape the node graph's Mesh wires have.
int l_mesh_merge(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    std::vector<RenderMesh> parts;
    lua_Integer n = luaL_len(L, 1);
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_geti(L, 1, i);
        parts.push_back(checkMesh(L, -1));
        lua_pop(L, 1);
    }
    pushMesh(L, std::make_shared<RenderMesh>(MeshBuilder::merged(parts)));
    return 1;
}
int l_mesh_translate(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    Vec3 t = checkVec3(L, 2);
    MeshBuilder::transform(*m, Mat4::translate(t.x, t.y, t.z));
    pushMesh(L, m);
    return 1;
}
// mesh.place(mesh, {x,y,z}, yaw) -> mesh : yaw (radians, about +Y) then translate,
// for seating an oriented part — e.g. a building turned to face its lot's street.
int l_mesh_place(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    Vec3 t = checkVec3(L, 2);
    double yaw = luaL_optnumber(L, 3, 0.0);
    MeshBuilder::transform(*m, Mat4::translate(t.x, t.y, t.z) *
                                   Mat4::rotateY(static_cast<Real>(yaw)));
    pushMesh(L, m);
    return 1;
}
int l_mesh_scale(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    Vec3 s;
    if (lua_isnumber(L, 2)) {           // scalar -> uniform scale
        double k = lua_tonumber(L, 2);
        s = Vec3(k, k, k);
    } else {
        s = checkVec3(L, 2);
    }
    MeshBuilder::transform(*m, Mat4::scale(s.x, s.y, s.z));
    pushMesh(L, m);
    return 1;
}
int l_mesh_rotate_y(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    MeshBuilder::transform(*m, Mat4::rotateY(luaL_checknumber(L, 2)));
    pushMesh(L, m);
    return 1;
}
int l_mesh_rotate_x(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    MeshBuilder::transform(*m, Mat4::rotateX(luaL_checknumber(L, 2)));
    pushMesh(L, m);
    return 1;
}
int l_mesh_rotate_z(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    MeshBuilder::transform(*m, Mat4::rotateZ(luaL_checknumber(L, 2)));
    pushMesh(L, m);
    return 1;
}
// Rotate a mesh so its local +Y aligns with `dir` (places oriented leaf cards,
// petals, grass blades along a heading).
int l_mesh_orient(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    Vec3 y = normalize(checkVec3(L, 2));
    Vec3 ref = (std::fabs(y.y) > 0.99) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    Vec3 x = normalize(cross(ref, y));
    Vec3 z = cross(x, y);
    Mat4 r;   // identity; basis as columns
    r.m[0][0] = x.x; r.m[1][0] = x.y; r.m[2][0] = x.z;
    r.m[0][1] = y.x; r.m[1][1] = y.y; r.m[2][1] = y.z;
    r.m[0][2] = z.x; r.m[1][2] = z.y; r.m[2][2] = z.z;
    MeshBuilder::transform(*m, r);
    pushMesh(L, m);
    return 1;
}
int l_mesh_recompute_normals(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    MeshBuilder::recomputeNormals(*m);
    pushMesh(L, m);
    return 1;
}
int l_mesh_bake_height_color(lua_State* L) {
    auto m = std::make_shared<RenderMesh>(checkMesh(L, 1));
    Vec3 low = checkVec3(L, 2);
    Vec3 high = checkVec3(L, 3);
    MeshBuilder::bakeHeightColor(*m, low, high);
    pushMesh(L, m);
    return 1;
}

// --- lsystem.* : the grammar generator (ADR-0021 Phase B.1) ---

int l_lsystem_create(lua_State* L) {
    void* mem = lua_newuserdatauv(L, sizeof(LSystem), 0);
    new (mem) LSystem();
    luaL_setmetatable(L, kLSystemMt);
    return 1;
}
int l_lsystem_rule(lua_State* L) {              // sys:rule("F", "FF", weight?)
    LSystem& sys = checkLSystem(L, 1);
    const char* symbol = luaL_checkstring(L, 2);
    const char* replacement = luaL_checkstring(L, 3);
    double weight = luaL_optnumber(L, 4, 1.0);
    luaL_argcheck(L, symbol[0] != '\0', 2, "symbol must be a non-empty string");
    sys.rule(symbol[0], replacement, weight);
    lua_settop(L, 1);                           // return self, for chaining
    return 1;
}
int l_lsystem_expand(lua_State* L) {            // sys:expand(axiom, iters, seed?)
    LSystem& sys = checkLSystem(L, 1);
    const char* axiom = luaL_checkstring(L, 2);
    int iterations = static_cast<int>(luaL_checkinteger(L, 3));
    auto seed = static_cast<uint32_t>(luaL_optinteger(L, 4, 0));
    std::string out = sys.expand(axiom, iterations, seed);
    lua_pushlstring(L, out.data(), out.size());
    return 1;
}

// --- parametric L-system: scripts author the grammar (ADR-0030) ---

int l_lsystem_parametric(lua_State* L) {        // lsystem.parametric()
    void* mem = lua_newuserdatauv(L, sizeof(ParametricLSystem), 0);
    new (mem) ParametricLSystem();
    luaL_setmetatable(L, kPLSystemMt);
    return 1;
}
// sys:rule("A(l,w)", "F(l,w)[+(30)A(l*0.7,w*0.6)]", weight?) — successor params
// are arithmetic expressions over the predecessor's formals (+ - * /, parens).
int l_plsystem_rule(lua_State* L) {
    ParametricLSystem& sys = checkPLSystem(L, 1);
    const char* pred = luaL_checkstring(L, 2);
    const char* succ = luaL_checkstring(L, 3);
    double weight = luaL_optnumber(L, 4, 1.0);
    sys.rule(pred, succ, weight);
    lua_settop(L, 1);                           // return self, for chaining
    return 1;
}
// sys:expand("A(1.0,0.1)", iterations, seed?) -> module string (feeds tree.skin).
int l_plsystem_expand(lua_State* L) {
    ParametricLSystem& sys = checkPLSystem(L, 1);
    const char* axiom = luaL_checkstring(L, 2);
    int iters = static_cast<int>(luaL_checkinteger(L, 3));
    auto seed = static_cast<uint32_t>(luaL_optinteger(L, 4, 0));
    pushModules(L, sys.expand(axiom, iters, seed));
    return 1;
}

bool optBoolField(lua_State* L, int idx, const char* key, bool fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    bool v = lua_isnil(L, -1) ? fallback : (lua_toboolean(L, -1) != 0);
    lua_pop(L, 1);
    return v;
}

// --- building.* : the split/shape grammar (ADR-0038 §2, ADR-0028 L1 grammar) ---

// Defined further down; needed here for the building style/colour fields.
Vec3 optVec3Field(lua_State* L, int idx, const char* key, Vec3 fallback);
std::string optStrField(lua_State* L, int idx, const char* key, const char* def);

BuildingParams readBuildingParams(lua_State* L, int idx) {
    BuildingParams p;
    if (lua_isnoneornil(L, idx)) return p;
    luaL_checktype(L, idx, LUA_TTABLE);
    p.floors        = static_cast<int>(optField(L, idx, "floors", p.floors));
    p.groundRetail  = optBoolField(L, idx, "ground_retail", p.groundRetail);
    p.floorHeight   = static_cast<Real>(optField(L, idx, "floor_height", p.floorHeight));
    p.groundHeight  = static_cast<Real>(optField(L, idx, "ground_height", p.groundHeight));
    p.bayWidth      = static_cast<Real>(optField(L, idx, "bay_width", p.bayWidth));
    p.windowInset   = static_cast<Real>(optField(L, idx, "window_inset", p.windowInset));
    p.walkableGround= optBoolField(L, idx, "walkable_ground", p.walkableGround);
    p.setbackEvery  = static_cast<Real>(optField(L, idx, "setback_every", p.setbackEvery));
    p.setbackFloors = static_cast<int>(optField(L, idx, "setback_floors", p.setbackFloors));
    p.parapet       = static_cast<Real>(optField(L, idx, "parapet", p.parapet));
    uint32_t seed   = static_cast<uint32_t>(optField(L, idx, "seed", 0));
    p.seed          = seed;

    // Massing: box | cylinder (round tower) | pagoda (tiered) — the "not every
    // building is a box" axis.
    std::string shp = optStrField(L, idx, "shape", "box");
    if (shp == "cylinder") p.shape = BuildingShape::Cylinder;
    else if (shp == "pagoda") p.shape = BuildingShape::Pagoda;
    p.sides     = static_cast<int>(optField(L, idx, "sides", p.sides));
    p.tiers     = static_cast<int>(optField(L, idx, "tiers", p.tiers));
    p.pilasters = optBoolField(L, idx, "pilasters", p.pilasters);

    // Facade style: a named look picks the wall material + a seeded colour
    // (brick reds, concrete greys, glass curtain, ...). Fine knobs override it.
    std::string style = optStrField(L, idx, "style", "");
    if (style == "glass")          p.curtainWall = true;
    else if (style == "brick")    { p.wallPart = PartId::Brick;    p.wallColor = facadeColor(FacadeStyle::Brick, seed); }
    else if (style == "concrete") { p.wallPart = PartId::Concrete; p.wallColor = facadeColor(FacadeStyle::Concrete, seed); }
    else if (style == "stucco")   { p.wallPart = PartId::Stucco;   p.wallColor = facadeColor(FacadeStyle::Stucco, seed); }
    else if (style == "metal")    { p.wallPart = PartId::Metal;    p.wallColor = facadeColor(FacadeStyle::Metal, seed); }
    else if (style == "painted")    p.wallColor = facadeColor(FacadeStyle::Painted, seed);
    p.curtainWall = optBoolField(L, idx, "curtain_wall", p.curtainWall);
    p.wallColor = optVec3Field(L, idx, "wall_color", p.wallColor);
    return p;
}

// building.grow{ floors=, width=, depth=, seed=, ... } -> mesh
// Grows the split/shape-grammar building over a rectangular footprint and returns
// the merged mesh (per-vertex colours baked, so wall/glass/roof read on a single
// material). Lua authors the grammar params; the rewrite/emit loop stays in C++.
int l_building_grow(lua_State* L) {
    BuildingParams p = readBuildingParams(L, 1);
    Real width = (lua_istable(L, 1)) ? static_cast<Real>(optField(L, 1, "width", 18.0)) : 18.0;
    Real depth = (lua_istable(L, 1)) ? static_cast<Real>(optField(L, 1, "depth", 14.0)) : 14.0;
    Poly2 foot = {{-width * 0.5, -depth * 0.5}, {width * 0.5, -depth * 0.5},
                  {width * 0.5, depth * 0.5}, {-width * 0.5, depth * 0.5}};
    Scope s = scopeFromFootprint(foot, 0.0, 10.0);
    BuildingMesh bm = growBuilding(s, p);
    pushMesh(L, std::make_shared<RenderMesh>(bm.merged()));
    return 1;
}

// building.height{...} -> number : the grown height in metres (no mesh). Cheap
// query for layout code that places a building before committing geometry.
int l_building_height(lua_State* L) {
    BuildingParams p = readBuildingParams(L, 1);
    Poly2 foot = {{-9, -7}, {9, -7}, {9, 7}, {-9, 7}};
    Scope s = scopeFromFootprint(foot, 0.0, 10.0);
    lua_pushnumber(L, growBuilding(s, p).height);
    return 1;
}

// Read an optional {x,y,z} field; fall back if absent/not a table.
Vec3 optVec3Field(lua_State* L, int idx, const char* key, Vec3 fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    Vec3 v = lua_istable(L, -1) ? checkVec3(L, -1) : fallback;
    lua_pop(L, 1);
    return v;
}

// --- streetfurniture.* : the street-kit props as tunable Lua recipes (ADR-0042).
// Each wraps the SAME C++ builder the city uses, so exposing a part never forks
// its geometry — Lua only sets the params; the build stays in C++.

// streetfurniture.lamp{ height=, pole_radius=, head_size={x,y,z},
//   pole_color={r,g,b}, head_color={r,g,b} } -> mesh (built at the origin).
int l_furniture_lamp(lua_State* L) {
    LampParams p;
    if (lua_istable(L, 1)) {
        p.height     = static_cast<Real>(optField(L, 1, "height", p.height));
        p.poleRadius = static_cast<Real>(optField(L, 1, "pole_radius", p.poleRadius));
        p.headSize   = optVec3Field(L, 1, "head_size", p.headSize);
        p.poleColor  = optVec3Field(L, 1, "pole_color", p.poleColor);
        p.headColor  = optVec3Field(L, 1, "head_color", p.headColor);
    }
    pushMesh(L, std::make_shared<RenderMesh>(streetLamp(p)));
    return 1;
}

// streetfurniture.traffic_signal{ pole_height=, arm_length=, arm_height=,
//   pole_color={r,g,b}, housing_color={r,g,b} } -> mesh (origin, facing +Z).
int l_furniture_traffic_signal(lua_State* L) {
    SignalParams p;
    if (lua_istable(L, 1)) {
        p.poleHeight   = static_cast<Real>(optField(L, 1, "pole_height", p.poleHeight));
        p.armLength    = static_cast<Real>(optField(L, 1, "arm_length", p.armLength));
        p.armHeight    = static_cast<Real>(optField(L, 1, "arm_height", p.armHeight));
        p.poleColor    = optVec3Field(L, 1, "pole_color", p.poleColor);
        p.housingColor = optVec3Field(L, 1, "housing_color", p.housingColor);
    }
    pushMesh(L, std::make_shared<RenderMesh>(trafficSignalProto(p)));
    return 1;
}

// --- scope.* : the split/shape-grammar op-vocabulary (ADR-0042 Phase 2) -------
// A Scope is an oriented box; split/divide/inset subdivide it and box/shell emit
// geometry. This is the spatial-subdivision core of the grammar (shape_grammar.h)
// exposed so Lua can author arbitrary props and facade details, not just whole
// buildings. The ops wrap the same C++ the building grammar uses.
constexpr const char* kScopeMt = "engine.procgen.Scope";

void pushScope(lua_State* L, const Scope& s) {
    void* mem = lua_newuserdatauv(L, sizeof(Scope), 0);
    new (mem) Scope(s);
    luaL_setmetatable(L, kScopeMt);
}
Scope& checkScope(lua_State* L, int idx) {
    return *static_cast<Scope*>(luaL_checkudata(L, idx, kScopeMt));
}
int scopeGc(lua_State* L) {
    static_cast<Scope*>(lua_touserdata(L, 1))->~Scope();
    return 0;
}

// An axis argument: a number 0/1/2, or "x"/"y"/"z" (== right/up/forward, also
// spelled "right"/"up"/"forward"). Matches the C++ scope frame.
int checkAxis(lua_State* L, int idx) {
    if (lua_type(L, idx) == LUA_TSTRING) {
        const char* s = lua_tostring(L, idx);
        switch (s[0]) {
            case 'x': case 'r': return 0;
            case 'y': case 'u': return 1;
            case 'z': case 'f': return 2;
            default: return luaL_error(L, "axis must be x/y/z (right/up/forward)");
        }
    }
    int a = static_cast<int>(luaL_checkinteger(L, idx));
    luaL_argcheck(L, a >= 0 && a <= 2, idx, "axis must be 0..2");
    return a;
}

// Push a vector of scopes as a 1-based Lua array.
void pushScopeArray(lua_State* L, const std::vector<Scope>& parts) {
    lua_createtable(L, static_cast<int>(parts.size()), 0);
    for (std::size_t i = 0; i < parts.size(); ++i) {
        pushScope(L, parts[i]);
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
}

Vec3 optColor(lua_State* L, int idx) {
    return lua_istable(L, idx) ? checkVec3(L, idx) : Vec3(0.8, 0.8, 0.8);
}

// scope{ origin = {x,y,z}, size = {x,y,z} } -> Scope (axis-aligned; the axes
// default to world right/up/forward, origin is the box's min corner).
int l_scope(lua_State* L) {
    Scope s;
    if (lua_istable(L, 1)) {
        s.origin = optVec3Field(L, 1, "origin", s.origin);
        s.size   = optVec3Field(L, 1, "size", s.size);
    }
    pushScope(L, s);
    return 1;
}

int l_scope_corner(lua_State* L) {
    Scope& s = checkScope(L, 1);
    pushVec3(L, s.corner(luaL_checknumber(L, 2), luaL_checknumber(L, 3),
                         luaL_checknumber(L, 4)));
    return 1;
}
int l_scope_center(lua_State* L) {
    pushVec3(L, checkScope(L, 1).center());
    return 1;
}
int l_scope_size(lua_State* L) {
    pushVec3(L, checkScope(L, 1).size);
    return 1;
}
// s:split(axis, {a, b, ...}) -> {Scope}. A negative size means "repeat |size| to
// fill" (the grammar's repeat-divide), matching splitScope.
int l_scope_split(lua_State* L) {
    Scope& s = checkScope(L, 1);
    int axis = checkAxis(L, 2);
    luaL_checktype(L, 3, LUA_TTABLE);
    std::vector<Real> sizes;
    lua_Integer n = luaL_len(L, 3);
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_geti(L, 3, i);
        sizes.push_back(static_cast<Real>(luaL_checknumber(L, -1)));
        lua_pop(L, 1);
    }
    pushScopeArray(L, splitScope(s, axis, sizes));
    return 1;
}
// s:divide(axis, target) -> {Scope}: N≈(extent/target) equal cells (repeatScope).
int l_scope_divide(lua_State* L) {
    Scope& s = checkScope(L, 1);
    int axis = checkAxis(L, 2);
    Real target = static_cast<Real>(luaL_checknumber(L, 3));
    pushScopeArray(L, repeatScope(s, axis, target));
    return 1;
}
int l_scope_inset(lua_State* L) {
    Scope& s = checkScope(L, 1);
    pushScope(L, insetScope(s, static_cast<Real>(luaL_checknumber(L, 2))));
    return 1;
}
// s:box(color) -> mesh : the six faces of the scope as a solid box.
int l_scope_box(lua_State* L) {
    Scope& s = checkScope(L, 1);
    BuildingMesh bm;
    emitBox(bm, s, PartId::Wall, optColor(L, 2));
    pushMesh(L, std::make_shared<RenderMesh>(bm.merged()));
    return 1;
}
// s:shell(color [, floor, ceiling]) -> mesh : the four walls (a hollow storey).
int l_scope_shell(lua_State* L) {
    Scope& s = checkScope(L, 1);
    bool floor = lua_toboolean(L, 3);
    bool ceiling = lua_toboolean(L, 4);
    BuildingMesh bm;
    emitShell(bm, s, PartId::Wall, optColor(L, 2), floor, ceiling);
    pushMesh(L, std::make_shared<RenderMesh>(bm.merged()));
    return 1;
}

// mesh.quad(a, b, c, d, normal, color) -> mesh : one panel from four corners.
int l_mesh_quad(lua_State* L) {
    Vec3 a = checkVec3(L, 1), b = checkVec3(L, 2), c = checkVec3(L, 3),
         d = checkVec3(L, 4), nrm = checkVec3(L, 5);
    auto m = std::make_shared<RenderMesh>();
    emitQuad(*m, a, b, c, d, nrm, optColor(L, 6));
    pushMesh(L, std::move(m));
    return 1;
}

// --- city.* : the road→block layout solver, exposed for decoration (ADR-0042
// Phase 3). A recipe asks for the street layout and places props along it; the
// solver (an algorithm, not a recipe) stays in C++.

// Push a ground-plane point as a { x =, z = } table.
void pushPoint(lua_State* L, const Vec2& p) {
    lua_createtable(L, 0, 2);
    lua_pushnumber(L, p.x); lua_setfield(L, -2, "x");
    lua_pushnumber(L, p.y); lua_setfield(L, -2, "z");   // Vec2.y == world z
}

// Read an optional string field (default if absent / not a string).
std::string optStrField(lua_State* L, int idx, const char* key, const char* def) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    const char* s = lua_tostring(L, -1);
    std::string r = s ? s : def;
    lua_pop(L, 1);
    return r;
}

// city.layout{ pattern=, extent=, cell_size=, ring_spacing=, spokes=, jitter=,
//   dropout=, seed= } -> { nodes = {{x,z}...}, edges = {{a,b,width}...},
//   blocks = {{{x,z}...}...} }. pattern = "grid" (default) | "radial" (ADR-0044).
// Node indices in edges are 1-based (Lua convention).
int l_city_layout(lua_State* L) {
    std::string pattern = lua_istable(L, 1) ? optStrField(L, 1, "pattern", "grid") : "grid";
    RoadGraph raw;
    if (pattern == "radial") {
        RadialParams rp;
        rp.extent      = static_cast<Real>(optField(L, 1, "extent", rp.extent));
        rp.ringSpacing = static_cast<Real>(optField(L, 1, "ring_spacing", rp.ringSpacing));
        rp.spokes      = static_cast<int>(optField(L, 1, "spokes", rp.spokes));
        rp.jitter      = static_cast<Real>(optField(L, 1, "jitter", rp.jitter));
        rp.seed        = static_cast<uint32_t>(optField(L, 1, "seed", 0));
        raw = radialRoads(rp);
    } else {
        GridRoadParams gp;
        if (lua_istable(L, 1)) {
            gp.extent   = static_cast<Real>(optField(L, 1, "extent", gp.extent));
            gp.cellSize = static_cast<Real>(optField(L, 1, "cell_size", gp.cellSize));
            gp.jitter   = static_cast<Real>(optField(L, 1, "jitter", gp.jitter));
            gp.dropout  = static_cast<Real>(optField(L, 1, "dropout", gp.dropout));
            gp.seed     = static_cast<uint32_t>(optField(L, 1, "seed", 0));
        }
        raw = gridRoads(gp);
    }
    RoadGraph g = planarize(raw);
    std::vector<Poly2> blocks = extractBlocks(g);

    lua_createtable(L, 0, 3);

    lua_createtable(L, static_cast<int>(g.nodes.size()), 0);
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        pushPoint(L, g.nodes[i].pos);
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    lua_setfield(L, -2, "nodes");

    lua_createtable(L, static_cast<int>(g.edges.size()), 0);
    for (std::size_t i = 0; i < g.edges.size(); ++i) {
        const RoadEdge& e = g.edges[i];
        lua_createtable(L, 0, 3);
        lua_pushinteger(L, e.a + 1); lua_setfield(L, -2, "a");
        lua_pushinteger(L, e.b + 1); lua_setfield(L, -2, "b");
        lua_pushnumber(L, e.width);  lua_setfield(L, -2, "width");
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    lua_setfield(L, -2, "edges");

    lua_createtable(L, static_cast<int>(blocks.size()), 0);
    for (std::size_t i = 0; i < blocks.size(); ++i) {
        const Poly2& poly = blocks[i];
        lua_createtable(L, static_cast<int>(poly.size()), 0);
        for (std::size_t j = 0; j < poly.size(); ++j) {
            pushPoint(L, poly[j]);
            lua_seti(L, -2, static_cast<lua_Integer>(j + 1));
        }
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    lua_setfield(L, -2, "blocks");

    return 1;
}

// city.lots(block, { inset=, target_area=, min_area=, min_edge=, jitter=, seed= })
// -> array of lots. `inset` first pulls the block in off the road centrelines (so
// lots clear the carriageway + sidewalk) — if that collapses a small block, no
// lots come back, so the recipe simply leaves it. Then it partitions the buildable
// interior into parcels (parcel.h) and returns each oriented to ITS street:
// { cx, cz, w (along the street = facade width), d (toward the street = depth),
//   yaw (turn a building's +Z to face the street), area }.
int l_city_lots(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    Poly2 block;
    lua_Integer n = luaL_len(L, 1);
    block.reserve(static_cast<std::size_t>(n));
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_geti(L, 1, i);
        block.push_back(Vec2(optField(L, -1, "x", 0.0), optField(L, -1, "z", 0.0)));
        lua_pop(L, 1);
    }
    ParcelParams pp;
    double insetD = 0.0;
    if (lua_istable(L, 2)) {
        pp.targetArea = static_cast<Real>(optField(L, 2, "target_area", pp.targetArea));
        pp.minArea    = static_cast<Real>(optField(L, 2, "min_area", pp.minArea));
        pp.minEdge    = static_cast<Real>(optField(L, 2, "min_edge", pp.minEdge));
        pp.jitter     = static_cast<Real>(optField(L, 2, "jitter", pp.jitter));
        pp.seed       = static_cast<uint32_t>(optField(L, 2, "seed", 0));
        insetD        = optField(L, 2, "inset", 0.0);
    }
    if (insetD > 0.0) block = inset(block, static_cast<Real>(insetD));

    std::vector<Lot> lots;
    if (block.size() >= 3) lots = subdivideBlock(block, pp);
    lua_createtable(L, static_cast<int>(lots.size()), 0);
    for (std::size_t i = 0; i < lots.size(); ++i) {
        const Lot& lot = lots[i];
        OBB2 obb = orientedBoundingBox(lot.footprint);
        const Vec2& f = lot.frontage;                 // outward, toward the street
        // The OBB axis most aligned with the frontage runs toward the street
        // (depth); the other runs along it (the facade width).
        int depthAxis = std::abs(dot(obb.axis[0], f)) >= std::abs(dot(obb.axis[1], f)) ? 0 : 1;
        // Square the building to the lot box (so a grid building fronts the road
        // perpendicular) — the yaw is the depth axis turned to point outward, NOT
        // the raw radial frontage (which sent corner lots off at 45°).
        Vec2 sd = obb.axis[depthAxis];
        if (dot(sd, f) < 0) sd = sd * -1.0;
        lua_createtable(L, 0, 5);
        lua_pushnumber(L, obb.center.x);                       lua_setfield(L, -2, "cx");
        lua_pushnumber(L, obb.center.y);                       lua_setfield(L, -2, "cz");
        lua_pushnumber(L, obb.half[1 - depthAxis] * 2.0);      lua_setfield(L, -2, "w");
        lua_pushnumber(L, obb.half[depthAxis] * 2.0);          lua_setfield(L, -2, "d");
        lua_pushnumber(L, std::atan2(sd.x, sd.y));             lua_setfield(L, -2, "yaw");
        lua_pushnumber(L, lot.area);                           lua_setfield(L, -2, "area");
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    return 1;
}

// --- texture.* : compose procedural textures from primitives (ADR-0043) -------
// A Field is a 2D scalar field over (u,v) — the same closure substrate as Sdf.
// Build a brick texture from a brick lattice × noise, then bake two colours by
// the mask — rather than asking for a baked preset. Bake -> an Image (RGB).
constexpr const char* kFieldMt = "engine.procgen.Field";
constexpr const char* kImageMt = "engine.procgen.Image";
using ImagePtr = std::shared_ptr<TextureData>;

void pushField(lua_State* L, Field2 f) {
    void* mem = lua_newuserdatauv(L, sizeof(Field2), 0);
    new (mem) Field2(std::move(f));
    luaL_setmetatable(L, kFieldMt);
}
Field2& checkField(lua_State* L, int idx) {
    return *static_cast<Field2*>(luaL_checkudata(L, idx, kFieldMt));
}
int fieldGc(lua_State* L) {
    static_cast<Field2*>(lua_touserdata(L, 1))->~Field2();
    return 0;
}
void pushImage(lua_State* L, ImagePtr img) {
    void* mem = lua_newuserdatauv(L, sizeof(ImagePtr), 0);
    new (mem) ImagePtr(std::move(img));
    luaL_setmetatable(L, kImageMt);
}
int imageGc(lua_State* L) {
    static_cast<ImagePtr*>(lua_touserdata(L, 1))->~ImagePtr();
    return 0;
}

int l_tex_constant(lua_State* L) {
    pushField(L, fieldConstant(luaL_checknumber(L, 1)));
    return 1;
}
int l_tex_noise(lua_State* L) {
    auto seed = static_cast<uint32_t>(optField(L, 1, "seed", 0));
    double scale = optField(L, 1, "scale", 8.0);
    pushField(L, fieldNoise(seed, scale));
    return 1;
}
int l_tex_fbm(lua_State* L) {
    auto seed = static_cast<uint32_t>(optField(L, 1, "seed", 0));
    double scale = optField(L, 1, "scale", 8.0);
    int oct = static_cast<int>(optField(L, 1, "octaves", 4));
    pushField(L, fieldFbm(seed, scale, oct));
    return 1;
}
int l_tex_checker(lua_State* L) {
    pushField(L, fieldChecker(optField(L, 1, "cols", 4), optField(L, 1, "rows", 4)));
    return 1;
}
int l_tex_brick(lua_State* L) {
    pushField(L, fieldBrick(optField(L, 1, "cols", 6), optField(L, 1, "rows", 12),
                            optField(L, 1, "mortar", 0.1),
                            optField(L, 1, "variation", 0.35),
                            static_cast<uint32_t>(optField(L, 1, "seed", 0))));
    return 1;
}
int l_tex_gradient_y(lua_State* L) {
    pushField(L, fieldGradientY());
    return 1;
}
// Field methods (return new fields — immutable composition).
int l_field_add(lua_State* L) { pushField(L, fieldAdd(checkField(L, 1), checkField(L, 2))); return 1; }
int l_field_mul(lua_State* L) { pushField(L, fieldMul(checkField(L, 1), checkField(L, 2))); return 1; }
int l_field_mix(lua_State* L) {
    pushField(L, fieldMix(checkField(L, 1), checkField(L, 2), luaL_checknumber(L, 3)));
    return 1;
}
int l_field_scale_bias(lua_State* L) {
    pushField(L, fieldScaleBias(checkField(L, 1), luaL_checknumber(L, 2),
                                luaL_checknumber(L, 3)));
    return 1;
}
int l_field_clamp(lua_State* L) {
    pushField(L, fieldClamp(checkField(L, 1), luaL_optnumber(L, 2, 0.0),
                            luaL_optnumber(L, 3, 1.0)));
    return 1;
}
// texture.bake_gray(field, size) -> Image (roughness/height/AO map).
int l_tex_bake_gray(lua_State* L) {
    int size = static_cast<int>(luaL_optinteger(L, 2, 256));
    pushImage(L, std::make_shared<TextureData>(bakeFieldGray(checkField(L, 1), size)));
    return 1;
}
// texture.bake_color(mask, colorA, colorB, size) -> Image (albedo map).
int l_tex_bake_color(lua_State* L) {
    Field2& mask = checkField(L, 1);
    Vec3 a = checkVec3(L, 2), b = checkVec3(L, 3);
    int size = static_cast<int>(luaL_optinteger(L, 4, 256));
    pushImage(L, std::make_shared<TextureData>(bakeFieldColor(mask, a, b, size)));
    return 1;
}
// texture.bake_normal(height, strength, size) -> Image (tangent-space normal).
int l_tex_bake_normal(lua_State* L) {
    Field2& h = checkField(L, 1);
    double strength = luaL_optnumber(L, 2, 1.0);
    int size = static_cast<int>(luaL_optinteger(L, 3, 256));
    pushImage(L, std::make_shared<TextureData>(bakeFieldNormal(h, strength, size)));
    return 1;
}

// --- terrain.* : compose a heightfield from primitives (ADR-0043) -------------
// A HeightField is a world-space function (x,z)->height — the 2.5D sibling of the
// texture Field. Build terrain from fbm + ridged ridges + domain warp + terraces,
// then bake to a mesh. `terrain` is a callable table: terrain(params, seed) still
// runs the C++ preset (generateTerrain); terrain.fbm{...} etc. compose.
constexpr const char* kHeightMt = "engine.procgen.Height";

void pushHeight(lua_State* L, HeightField f) {
    void* mem = lua_newuserdatauv(L, sizeof(HeightField), 0);
    new (mem) HeightField(std::move(f));
    luaL_setmetatable(L, kHeightMt);
}
HeightField& checkHeight(lua_State* L, int idx) {
    return *static_cast<HeightField*>(luaL_checkudata(L, idx, kHeightMt));
}
int heightGc(lua_State* L) {
    static_cast<HeightField*>(lua_touserdata(L, 1))->~HeightField();
    return 0;
}

int l_terr_flat(lua_State* L) { pushHeight(L, heightConstant(luaL_checknumber(L, 1))); return 1; }
int l_terr_noise(lua_State* L) {
    pushHeight(L, heightNoise(static_cast<uint32_t>(optField(L, 1, "seed", 0)),
                              optField(L, 1, "freq", 0.01), optField(L, 1, "amp", 10.0)));
    return 1;
}
int l_terr_fbm(lua_State* L) {
    pushHeight(L, heightFbm(static_cast<uint32_t>(optField(L, 1, "seed", 0)),
                            optField(L, 1, "freq", 0.01), optField(L, 1, "amp", 20.0),
                            static_cast<int>(optField(L, 1, "octaves", 5))));
    return 1;
}
int l_terr_ridged(lua_State* L) {
    pushHeight(L, heightRidged(static_cast<uint32_t>(optField(L, 1, "seed", 0)),
                               optField(L, 1, "freq", 0.01), optField(L, 1, "amp", 20.0),
                               static_cast<int>(optField(L, 1, "octaves", 5))));
    return 1;
}
int l_terr_warp(lua_State* L) {
    pushHeight(L, heightWarp(checkHeight(L, 1), checkHeight(L, 2),
                             luaL_optnumber(L, 3, 30.0)));
    return 1;
}
int l_terr_terrace(lua_State* L) {
    pushHeight(L, heightTerrace(checkHeight(L, 1), luaL_checknumber(L, 2)));
    return 1;
}
// terrain.erode(field, { size=, resolution=, droplets=, talus=, seed=, ... }) ->
// HeightField. A bake pass: hydraulic + thermal erosion carves drainage networks.
int l_terr_erode(lua_State* L) {
    HeightField& f = checkHeight(L, 1);
    double size = optField(L, 2, "size", 400.0);
    int res = static_cast<int>(optField(L, 2, "resolution", 384));
    ErosionParams p;
    p.droplets    = static_cast<int>(optField(L, 2, "droplets", p.droplets));
    p.erodeSpeed  = static_cast<float>(optField(L, 2, "erode_speed", p.erodeSpeed));
    p.depositSpeed = static_cast<float>(optField(L, 2, "deposit_speed", p.depositSpeed));
    p.evaporation = static_cast<float>(optField(L, 2, "evaporation", p.evaporation));
    p.thermalIterations = static_cast<int>(optField(L, 2, "thermal_iterations", p.thermalIterations));
    p.talus       = static_cast<float>(optField(L, 2, "talus", p.talus));
    p.seed        = static_cast<uint32_t>(optField(L, 2, "seed", 0));
    pushHeight(L, erodeField(f, size, res, p));
    return 1;
}
// terrain.conform(field, layout, { margin=, falloff= }) -> HeightField. Cut/fill
// the terrain to a road network: each edge becomes a constant-grade ramp corridor
// (its endpoints at the field's natural height, flat across the road, linear
// along its length), easing back to the natural ground across `falloff`. So urban
// roads stay flat (or a single incline) while the terrain conforms *around* them.
// Build the road mesh on the RESULT (height = conformed field) so the ribbon is
// flat too.
int l_terr_conform(lua_State* L) {
    HeightField base = checkHeight(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    double margin = lua_istable(L, 3) ? optField(L, 3, "margin", 2.0) : 2.0;
    double falloff = lua_istable(L, 3) ? optField(L, 3, "falloff", 8.0) : 8.0;

    // Nodes ({x=,z=}, 1-based), each seated at the terrain's natural height so the
    // road meets the ground at the junctions.
    lua_getfield(L, 2, "nodes");
    luaL_checktype(L, -1, LUA_TTABLE);
    lua_Integer nn = luaL_len(L, -1);
    std::vector<Vec3> node(static_cast<std::size_t>(nn));
    for (lua_Integer i = 1; i <= nn; ++i) {
        lua_geti(L, -1, i);
        double x = optField(L, -1, "x", 0.0), z = optField(L, -1, "z", 0.0);
        node[static_cast<std::size_t>(i - 1)] = Vec3(x, base(x, z), z);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    // Edges ({a, b, width}, 1-based) -> a ramp corridor between the two nodes.
    std::vector<TerrainFlatten> regions;
    lua_getfield(L, 2, "edges");
    if (lua_istable(L, -1)) {
        lua_Integer ne = luaL_len(L, -1);
        regions.reserve(static_cast<std::size_t>(ne));
        for (lua_Integer i = 1; i <= ne; ++i) {
            lua_geti(L, -1, i);
            int a = static_cast<int>(optField(L, -1, "a", 0));
            int b = static_cast<int>(optField(L, -1, "b", 0));
            double w = optField(L, -1, "width", 8.0);
            lua_pop(L, 1);
            if (a < 1 || a > nn || b < 1 || b > nn) continue;
            const Vec3& pa = node[static_cast<std::size_t>(a - 1)];
            const Vec3& pb = node[static_cast<std::size_t>(b - 1)];
            regions.push_back(
                makeFlattenRamp(pa, pb, pa.y, pb.y, w * 0.5 + margin, falloff));
        }
    }
    lua_pop(L, 1);

    // Optional block pads: opts.pads = { { y=, poly={ {x=,z=}, ... } }, ... }.
    // Each becomes a flat pad (makeFlattenPad) at height y — the recipe decides
    // which blocks are flat enough to develop and seats a level plot under them.
    if (lua_istable(L, 3)) {
        double padFalloff = optField(L, 3, "pad_falloff", falloff);
        lua_getfield(L, 3, "pads");
        if (lua_istable(L, -1)) {
            int pads = lua_absindex(L, -1);
            lua_Integer np = luaL_len(L, pads);
            for (lua_Integer i = 1; i <= np; ++i) {
                lua_geti(L, pads, i);
                int pad = lua_absindex(L, -1);
                double y = optField(L, pad, "y", 0.0);
                std::vector<Vec3> poly;
                lua_getfield(L, pad, "poly");
                if (lua_istable(L, -1)) {
                    int pp = lua_absindex(L, -1);
                    lua_Integer pn = luaL_len(L, pp);
                    poly.reserve(static_cast<std::size_t>(pn));
                    for (lua_Integer k = 1; k <= pn; ++k) {
                        lua_geti(L, pp, k);
                        poly.push_back(Vec3(optField(L, -1, "x", 0.0), 0.0,
                                            optField(L, -1, "z", 0.0)));
                        lua_pop(L, 1);
                    }
                }
                lua_pop(L, 1);   // poly
                if (poly.size() >= 3)
                    regions.push_back(makeFlattenPad(std::move(poly), y, padFalloff));
                lua_pop(L, 1);   // pad
            }
        }
        lua_pop(L, 1);   // pads
    }

    pushHeight(L, conformField(std::move(base), std::move(regions)));
    return 1;
}
// HeightField methods (immutable composition).
int l_height_add(lua_State* L) { pushHeight(L, heightAdd(checkHeight(L, 1), checkHeight(L, 2))); return 1; }
int l_height_mul(lua_State* L) { pushHeight(L, heightMul(checkHeight(L, 1), checkHeight(L, 2))); return 1; }
int l_height_max(lua_State* L) { pushHeight(L, heightMax(checkHeight(L, 1), checkHeight(L, 2))); return 1; }
int l_height_min(lua_State* L) { pushHeight(L, heightMin(checkHeight(L, 1), checkHeight(L, 2))); return 1; }
int l_height_scale(lua_State* L) { pushHeight(L, heightScale(checkHeight(L, 1), luaL_checknumber(L, 2))); return 1; }
int l_height_mix(lua_State* L) {
    pushHeight(L, heightMix(checkHeight(L, 1), checkHeight(L, 2), luaL_checknumber(L, 3)));
    return 1;
}
int l_height_clamp(lua_State* L) {
    pushHeight(L, heightClamp(checkHeight(L, 1), luaL_checknumber(L, 2), luaL_checknumber(L, 3)));
    return 1;
}
// h:at(x, z) -> height : sample the field (for draping a city on terrain).
int l_height_at(lua_State* L) {
    HeightField& h = checkHeight(L, 1);
    lua_pushnumber(L, h(luaL_checknumber(L, 2), luaL_checknumber(L, 3)));
    return 1;
}
// terrain.mesh(field, { size=, resolution=, color={r,g,b} }) -> mesh.
int l_terr_mesh(lua_State* L) {
    HeightField& h = checkHeight(L, 1);
    double size = optField(L, 2, "size", 200.0);
    int res = static_cast<int>(optField(L, 2, "resolution", 128));
    Vec3 color = optVec3Field(L, 2, "color", Vec3(0.34, 0.40, 0.28));
    pushMesh(L, std::make_shared<RenderMesh>(bakeHeightMesh(h, size, res, color)));
    return 1;
}
// terrain(params, seed) via __call (index 1 is the terrain table itself).
TerrainParams readTerrainParams(lua_State* L, int idx);   // defined below
int l_terrain_preset(lua_State* L) {
    TerrainParams p = readTerrainParams(L, 2);
    auto seed = static_cast<uint32_t>(luaL_optinteger(L, 3, 0));
    pushMesh(L, std::make_shared<RenderMesh>(generateTerrain(p, Noise(seed))));
    return 1;
}

// city.road_mesh(layout, { height=HeightField, lift=, color={r,g,b} }) -> mesh.
// Builds a connected road *surface* from a layout table (the one city.layout
// returns, or one a recipe edited — added spine edges/nodes), with trimmed
// ribbons + junction pads (road_mesh.h). `height` drapes it on terrain.
int l_city_road_mesh(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    RoadGraph g;
    // Nodes (1-based; the table is {x=,z=} points).
    lua_getfield(L, 1, "nodes");
    lua_Integer nn = lua_istable(L, -1) ? luaL_len(L, -1) : 0;
    std::vector<int> remap(static_cast<std::size_t>(nn));
    for (lua_Integer i = 1; i <= nn; ++i) {
        lua_geti(L, -1, i);
        double x = optField(L, -1, "x", 0.0), z = optField(L, -1, "z", 0.0);
        remap[static_cast<std::size_t>(i - 1)] = g.addNode(Vec2(x, z), 0.01);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    // Edges ({a, b, width}, 1-based node indices).
    lua_getfield(L, 1, "edges");
    lua_Integer ne = lua_istable(L, -1) ? luaL_len(L, -1) : 0;
    for (lua_Integer i = 1; i <= ne; ++i) {
        lua_geti(L, -1, i);
        int a = static_cast<int>(optField(L, -1, "a", 0));
        int b = static_cast<int>(optField(L, -1, "b", 0));
        double w = optField(L, -1, "width", 8.0);
        if (a >= 1 && a <= nn && b >= 1 && b <= nn)
            g.addEdge(remap[a - 1], remap[b - 1], static_cast<Real>(w));
        lua_pop(L, 1);
    }
    lua_pop(L, 1);

    RoadMeshParams rp;
    if (lua_istable(L, 2)) {
        rp.lift = optField(L, 2, "lift", rp.lift);
        rp.minSetback = optField(L, 2, "min_setback", rp.minSetback);
        rp.color = optVec3Field(L, 2, "color", rp.color);
        rp.sidewalkWidth = optField(L, 2, "sidewalk", rp.sidewalkWidth);
        rp.curbHeight = optField(L, 2, "curb", rp.curbHeight);
        rp.sidewalkColor = optVec3Field(L, 2, "sidewalk_color", rp.sidewalkColor);
        rp.curbColor = optVec3Field(L, 2, "curb_color", rp.curbColor);
        rp.laneMarkings = optBoolField(L, 2, "markings", rp.laneMarkings);
        rp.laneWidth = optField(L, 2, "lane_width", rp.laneWidth);
        rp.markWidth = optField(L, 2, "mark_width", rp.markWidth);
        rp.laneColor = optVec3Field(L, 2, "lane_color", rp.laneColor);
        rp.centerColor = optVec3Field(L, 2, "center_color", rp.centerColor);
        rp.plazaRadius = optField(L, 2, "plaza", rp.plazaRadius);
        rp.plazaMinArms = static_cast<int>(optField(L, 2, "plaza_min_arms", rp.plazaMinArms));
        lua_getfield(L, 2, "height");
        if (auto* hf = static_cast<HeightField*>(luaL_testudata(L, -1, kHeightMt))) {
            HeightField h = *hf;
            rp.heightAt = [h](double x, double z) { return h(x, z); };
        }
        lua_pop(L, 1);
    }
    pushMesh(L, std::make_shared<RenderMesh>(buildRoadMesh(g, rp)));
    return 1;
}

// --- material.* : a baked material bundle for a part (ADR-0043) ---------------
// Applied to a part by world-planar projection — no authored UVs needed.
constexpr const char* kMaterialMt = "engine.procgen.Material";
using MaterialPtr = std::shared_ptr<ProcMaterial>;

void pushMaterial(lua_State* L, MaterialPtr m) {
    void* mem = lua_newuserdatauv(L, sizeof(MaterialPtr), 0);
    new (mem) MaterialPtr(std::move(m));
    luaL_setmetatable(L, kMaterialMt);
}
int materialGc(lua_State* L) {
    static_cast<MaterialPtr*>(lua_touserdata(L, 1))->~MaterialPtr();
    return 0;
}

// material.new{ albedo=Image, normal=Image, roughness=, metallic=, tile= }.
int l_material_new(lua_State* L) {
    auto m = std::make_shared<ProcMaterial>();
    if (lua_istable(L, 1)) {
        lua_getfield(L, 1, "albedo");
        if (auto* img = static_cast<ImagePtr*>(luaL_testudata(L, -1, kImageMt))) {
            m->albedo = **img; m->textured = true;
        }
        lua_pop(L, 1);
        lua_getfield(L, 1, "normal");
        if (auto* img = static_cast<ImagePtr*>(luaL_testudata(L, -1, kImageMt))) {
            m->normal = **img; m->textured = true;
        }
        lua_pop(L, 1);
        m->roughness = static_cast<float>(optField(L, 1, "roughness", m->roughness));
        m->metallic = static_cast<float>(optField(L, 1, "metallic", m->metallic));
        m->tile = optField(L, 1, "tile", m->tile);
    }
    pushMaterial(L, std::move(m));
    return 1;
}

// --- model.* : the composable procedural model value (ADR-0042 Phase 3) --------
// A Model accumulates part meshes + instance groups, so a recipe can return a
// whole block/scene, not just one mesh. Models compose via :merge, so parts nest
// into collections nest into a city. :flatten bridges to single-mesh consumers.
constexpr const char* kModelMt = "engine.procgen.Model";
using ModelPtr = std::shared_ptr<ProcModel>;

void pushModel(lua_State* L, ModelPtr m) {
    void* mem = lua_newuserdatauv(L, sizeof(ModelPtr), 0);
    new (mem) ModelPtr(std::move(m));
    luaL_setmetatable(L, kModelMt);
}
ProcModel& checkModel(lua_State* L, int idx) {
    return **static_cast<ModelPtr*>(luaL_checkudata(L, idx, kModelMt));
}
int modelGc(lua_State* L) {
    static_cast<ModelPtr*>(lua_touserdata(L, 1))->~ModelPtr();
    return 0;
}

// model.new() -> an empty Model.
int l_model_new(lua_State* L) {
    pushModel(L, std::make_shared<ProcModel>());
    return 1;
}

// m:add(mesh [, material]) -> m : append a part. With a material it is textured
// (world-planar), else baked vertex-coloured. Chainable.
int l_model_add(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    ProcPart part;
    part.mesh = checkMesh(L, 2);
    if (auto* mat = static_cast<MaterialPtr*>(luaL_testudata(L, 3, kMaterialMt)))
        part.material = **mat;
    m.parts.push_back(std::move(part));
    lua_pushvalue(L, 1);
    return 1;
}

// m:add_instances(mesh, placements [, opts]) -> m : place a prototype many times.
// placements is an array of { pos = {x,y,z}, yaw = radians, scale = number };
// opts may set { roughness=, metallic=, alpha_foliage= }. Chainable.
int l_model_add_instances(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    ProcInstanceGroup g;
    g.proto = checkMesh(L, 2);
    luaL_checktype(L, 3, LUA_TTABLE);
    lua_Integer n = luaL_len(L, 3);
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_geti(L, 3, i);
        Vec3 pos = optVec3Field(L, -1, "pos", Vec3(0, 0, 0));
        double yaw = optField(L, -1, "yaw", 0.0);
        double scale = optField(L, -1, "scale", 1.0);
        g.transforms.push_back(Mat4::trs(pos,
            Quat::fromAxisAngle(Vec3(0, 1, 0), static_cast<Real>(yaw)),
            Vec3(scale, scale, scale)));
        lua_pop(L, 1);
    }
    if (lua_istable(L, 4)) {
        g.roughness = static_cast<float>(optField(L, 4, "roughness", g.roughness));
        g.metallic = static_cast<float>(optField(L, 4, "metallic", g.metallic));
        g.alphaFoliage = optBoolField(L, 4, "alpha_foliage", g.alphaFoliage);
        // opts.collide: solidify every instance. "mesh" stamps the proto's
        // triangles at each transform; a { shape="capsule", radius=, height= }
        // (or box/sphere) table places one primitive collider per instance —
        // the right shape for a lamp post (a thin capsule), exactly.
        lua_getfield(L, 4, "collide");
        if (lua_isstring(L, -1) && std::string(lua_tostring(L, -1)) == "mesh") {
            g.collision = InstanceCollision::Mesh;
        } else if (lua_istable(L, -1)) {
            int ct = lua_absindex(L, -1);
            std::string shape = optStrField(L, ct, "shape", "capsule");
            g.colliderFriction =
                static_cast<Real>(optField(L, ct, "friction", g.colliderFriction));
            if (shape == "box") {
                g.collision = InstanceCollision::Box;
                g.colliderHalfExtent = optVec3Field(L, ct, "size", Vec3(1, 1, 1)) * 0.5;
            } else if (shape == "sphere") {
                g.collision = InstanceCollision::Sphere;
                g.colliderRadius = static_cast<Real>(optField(L, ct, "radius", g.colliderRadius));
            } else {
                g.collision = InstanceCollision::Capsule;
                g.colliderRadius = static_cast<Real>(optField(L, ct, "radius", g.colliderRadius));
                g.colliderHalfHeight =
                    static_cast<Real>(optField(L, ct, "height", g.colliderHalfHeight * 2.0) * 0.5);
            }
        }
        lua_pop(L, 1);
    }
    m.instances.push_back(std::move(g));
    lua_pushvalue(L, 1);
    return 1;
}

// m:collide(mesh [, {friction=}]) -> m : an exact static triangle-mesh collider
// from `mesh` (world space). The faithful collider for terrain, roads, and any
// solid shell — collision is the actual surface, not an approximation. Chainable.
int l_model_collide(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    ProcCollider c;
    c.kind = ProcCollider::Kind::Mesh;
    c.mesh = checkMesh(L, 2);
    if (lua_istable(L, 3))
        c.friction = static_cast<Real>(optField(L, 3, "friction", c.friction));
    m.colliders.push_back(std::move(c));
    lua_pushvalue(L, 1);
    return 1;
}

// m:add_solid(mesh [, material]) -> m : render the mesh AND collide with the
// same triangles. The default for built world geometry (a building shell, a
// foundation): what you see is exactly what you hit. Chainable.
int l_model_add_solid(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    RenderMesh mesh = checkMesh(L, 2);
    ProcPart part;
    part.mesh = mesh;
    if (auto* mat = static_cast<MaterialPtr*>(luaL_testudata(L, 3, kMaterialMt)))
        part.material = **mat;
    m.parts.push_back(std::move(part));
    ProcCollider c;
    c.kind = ProcCollider::Kind::Mesh;
    c.mesh = std::move(mesh);
    m.colliders.push_back(std::move(c));
    lua_pushvalue(L, 1);
    return 1;
}

// m:add_collider(spec) -> m : a primitive collider, for a part whose true shape
// IS a primitive (a round tower is exactly a capsule). spec.shape = "box"
// (size={w,h,d}, yaw), "sphere" (radius), or "capsule" (radius, height); with
// center={x,y,z}, friction, motion ("static"|"dynamic"). Chainable.
int l_model_add_collider(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    ProcCollider c;
    std::string shape = optStrField(L, 2, "shape", "box");
    c.center = optVec3Field(L, 2, "center", c.center);
    c.yaw = static_cast<Real>(optField(L, 2, "yaw", 0.0));
    c.friction = static_cast<Real>(optField(L, 2, "friction", c.friction));
    c.dynamic = optStrField(L, 2, "motion", "static") == std::string("dynamic");
    if (shape == "sphere") {
        c.kind = ProcCollider::Kind::Sphere;
        c.radius = static_cast<Real>(optField(L, 2, "radius", c.radius));
    } else if (shape == "capsule") {
        c.kind = ProcCollider::Kind::Capsule;
        c.radius = static_cast<Real>(optField(L, 2, "radius", c.radius));
        c.halfHeight = static_cast<Real>(optField(L, 2, "height", c.halfHeight * 2.0) * 0.5);
    } else {
        c.kind = ProcCollider::Kind::Box;
        c.halfExtent = optVec3Field(L, 2, "size", Vec3(1, 1, 1)) * 0.5;
    }
    m.colliders.push_back(std::move(c));
    lua_pushvalue(L, 1);
    return 1;
}

int l_model_collider_count(lua_State* L) {
    lua_pushinteger(L, static_cast<lua_Integer>(checkModel(L, 1).colliders.size()));
    return 1;
}

// m:merge(other) -> m : fold another Model's parts + instances into this one.
int l_model_merge(lua_State* L) {
    ProcModel& m = checkModel(L, 1);
    m.merge(checkModel(L, 2));
    lua_pushvalue(L, 1);
    return 1;
}

int l_model_part_count(lua_State* L) {
    lua_pushinteger(L, static_cast<lua_Integer>(checkModel(L, 1).parts.size()));
    return 1;
}
int l_model_instance_count(lua_State* L) {
    lua_pushinteger(L, checkModel(L, 1).instanceCount());
    return 1;
}

// Flatten a model to a single mesh: every part, plus each instance group's proto
// stamped at each transform. Bridges a Model to single-mesh consumers/preview.
RenderMesh flattenModel(const ProcModel& m) {
    RenderMesh out;
    for (const ProcPart& p : m.parts) MeshBuilder::append(out, p.mesh);
    for (const ProcInstanceGroup& g : m.instances)
        for (const Mat4& xf : g.transforms)
            MeshBuilder::appendTransformed(out, g.proto, xf);
    return out;
}

// m:flatten() -> mesh.
int l_model_flatten(lua_State* L) {
    pushMesh(L, std::make_shared<RenderMesh>(flattenModel(checkModel(L, 1))));
    return 1;
}

// The skin-side TreeParams (grammar fields are unused; the grammar is the
// module string passed to tree.skin).
TreeParams readTreeParams(lua_State* L, int idx) {
    TreeParams p;
    if (lua_isnoneornil(L, idx)) return p;
    luaL_checktype(L, idx, LUA_TTABLE);
    p.angleJitter  = static_cast<float>(optField(L, idx, "angle_jitter", p.angleJitter));
    p.phyllotaxis  = static_cast<float>(optField(L, idx, "phyllotaxis", p.phyllotaxis));
    p.tipRadius    = static_cast<float>(optField(L, idx, "tip_radius", p.tipRadius));
    p.pipeExponent = static_cast<float>(optField(L, idx, "pipe_exponent", p.pipeExponent));
    p.radiusScale  = static_cast<float>(optField(L, idx, "radius_scale", p.radiusScale));
    p.ringSegments = static_cast<int>(optField(L, idx, "ring_segments", p.ringSegments));
    p.barkVScale   = static_cast<float>(optField(L, idx, "bark_v_scale", p.barkVScale));
    p.droop        = static_cast<float>(optField(L, idx, "droop", p.droop));
    p.wander       = static_cast<float>(optField(L, idx, "wander", p.wander));
    p.rootCount    = static_cast<int>(optField(L, idx, "root_count", p.rootCount));
    p.rootSpread   = static_cast<float>(optField(L, idx, "root_spread", p.rootSpread));
    p.leafSize     = static_cast<float>(optField(L, idx, "leaf_size", p.leafSize));
    p.leavesPerTip = static_cast<int>(optField(L, idx, "leaves_per_tip", p.leavesPerTip));
    p.leafThickness = static_cast<float>(optField(L, idx, "leaf_thickness", p.leafThickness));
    p.leafClump = static_cast<float>(optField(L, idx, "leaf_clump", p.leafClump));
    p.maxLeafCards = static_cast<int>(optField(L, idx, "max_leaf_cards", p.maxLeafCards));
    lua_getfield(L, lua_absindex(L, idx), "leaves");
    if (!lua_isnil(L, -1)) p.leaves = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    p.barkColor = optVec3Field(L, idx, "bark_color", p.barkColor);
    p.leafColor = optVec3Field(L, idx, "leaf_color", p.leafColor);
    return p;
}

// tree.skin(modules, params?, seed?) -> bark Mesh, leaf Mesh (leaf may be nil).
// Two meshes because bark is opaque and leaves are alpha-cut (different
// materials); the caller places/instances them separately.
int l_tree_skin(lua_State* L) {
    ModuleString& modules = checkModules(L, 1);
    TreeParams params = readTreeParams(L, 2);
    auto seed = static_cast<uint32_t>(luaL_optinteger(L, 3, 0));
    TreeMesh tm = skinTree(modules, params, seed);
    pushMesh(L, std::make_shared<RenderMesh>(std::move(tm.branches)));
    if (tm.leaves.vertices.empty()) {
        lua_pushnil(L);
    } else {
        pushMesh(L, std::make_shared<RenderMesh>(std::move(tm.leaves)));
    }
    return 2;
}

TurtleParams readTurtleParams(lua_State* L, int idx) {
    TurtleParams p;
    if (lua_isnoneornil(L, idx)) return p;
    luaL_checktype(L, idx, LUA_TTABLE);
    p.length = static_cast<float>(optField(L, idx, "length", p.length));
    p.radius = static_cast<float>(optField(L, idx, "radius", p.radius));
    p.radiusTaper = static_cast<float>(optField(L, idx, "radius_taper", p.radiusTaper));
    p.taper = static_cast<float>(optField(L, idx, "taper", p.taper));
    p.angleDeg = static_cast<float>(optField(L, idx, "angle_deg", p.angleDeg));
    p.segmentSlices = static_cast<int>(optField(L, idx, "segment_slices", p.segmentSlices));
    p.leafRadius = static_cast<float>(optField(L, idx, "leaf_radius", p.leafRadius));
    return p;
}
int l_turtle_mesh(lua_State* L) {               // kit-bashed cylinders
    const char* symbols = luaL_checkstring(L, 1);
    pushMesh(L, std::make_shared<RenderMesh>(
                    buildTurtleMesh(symbols, readTurtleParams(L, 2))));
    return 1;
}
int l_turtle_mesh_sdf(lua_State* L) {           // one welded surface (SDF skin)
    const char* symbols = luaL_checkstring(L, 1);
    TurtleParams p = readTurtleParams(L, 2);
    double smoothness = luaL_checknumber(L, 3);
    int res = static_cast<int>(luaL_checkinteger(L, 4));
    pushMesh(L, std::make_shared<RenderMesh>(
                    buildTurtleMeshSdf(symbols, p, smoothness, res)));
    return 1;
}
// segments(symbols, params) -> array of {a={x,y,z}, b={x,y,z}, radius} (branches
// only; tapered radius reflects the `taper` param). For building/inspecting.
int l_lsystem_segments(lua_State* L) {
    const char* symbols = luaL_checkstring(L, 1);
    std::vector<BranchSegment> segs = turtleSegments(symbols, readTurtleParams(L, 2));
    lua_newtable(L);
    lua_Integer out = 0;
    for (const BranchSegment& s : segs) {
        if (s.a.x == s.b.x && s.a.y == s.b.y && s.a.z == s.b.z) continue;  // leaf
        lua_createtable(L, 0, 3);
        pushVec3(L, s.a); lua_setfield(L, -2, "a");
        pushVec3(L, s.b); lua_setfield(L, -2, "b");
        lua_pushnumber(L, s.radius); lua_setfield(L, -2, "radius");
        lua_seti(L, -2, ++out);
    }
    return 1;
}
// leaves(symbols, params) -> array of {position={x,y,z}, direction={x,y,z}} —
// where to place real leaf cards (and which way they point).
int l_lsystem_leaves(lua_State* L) {
    const char* symbols = luaL_checkstring(L, 1);
    std::vector<LeafPlacement> leaves = turtleLeaves(symbols, readTurtleParams(L, 2));
    lua_createtable(L, static_cast<int>(leaves.size()), 0);
    for (std::size_t i = 0; i < leaves.size(); ++i) {
        lua_createtable(L, 0, 2);
        pushVec3(L, leaves[i].position); lua_setfield(L, -2, "position");
        pushVec3(L, leaves[i].direction); lua_setfield(L, -2, "direction");
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    return 1;
}

// --- terrain / scatter (the Field->Mesh and Frame generators) ---

TerrainParams readTerrainParams(lua_State* L, int idx) {
    TerrainParams p;
    if (lua_isnoneornil(L, idx)) return p;
    luaL_checktype(L, idx, LUA_TTABLE);
    p.size = static_cast<float>(optField(L, idx, "size", p.size));
    p.resolution = static_cast<int>(optField(L, idx, "resolution", p.resolution));
    p.heightScale = static_cast<float>(optField(L, idx, "height_scale", p.heightScale));
    p.noiseScale = optField(L, idx, "noise_scale", p.noiseScale);
    p.octaves = static_cast<int>(optField(L, idx, "octaves", p.octaves));
    p.warp = optField(L, idx, "warp", p.warp);
    return p;
}
ScatterParams readScatterParams(lua_State* L, int idx) {
    ScatterParams p;
    if (lua_isnoneornil(L, idx)) return p;
    luaL_checktype(L, idx, LUA_TTABLE);
    p.regionSize = static_cast<float>(optField(L, idx, "region_size", p.regionSize));
    p.count = static_cast<int>(optField(L, idx, "count", p.count));
    p.minScale = static_cast<float>(optField(L, idx, "min_scale", p.minScale));
    p.maxScale = static_cast<float>(optField(L, idx, "max_scale", p.maxScale));
    p.maxSlopeDeg = static_cast<float>(optField(L, idx, "max_slope_deg", p.maxSlopeDeg));
    p.minHeight = static_cast<float>(optField(L, idx, "min_height", p.minHeight));
    p.maxHeight = static_cast<float>(optField(L, idx, "max_height", p.maxHeight));
    p.densityScale = optField(L, idx, "density_scale", p.densityScale);
    p.densityThreshold =
        static_cast<float>(optField(L, idx, "density_threshold", p.densityThreshold));
    p.seed = static_cast<uint32_t>(optField(L, idx, "seed", p.seed));
    return p;
}
// scatter(scatterParams, terrainParams, terrainSeed) -> array of Frames, each
// {position = {x,y,z}, yaw = , scale = } (the Frame value type; ADR-0021).
int l_scatter(lua_State* L) {
    ScatterParams sp = readScatterParams(L, 1);
    TerrainParams tp = readTerrainParams(L, 2);
    auto terrainSeed = static_cast<uint32_t>(luaL_optinteger(L, 3, 0));
    std::vector<Placement> places = scatterOnTerrain(sp, tp, Noise(terrainSeed));

    lua_createtable(L, static_cast<int>(places.size()), 0);
    for (std::size_t i = 0; i < places.size(); ++i) {
        const Placement& pl = places[i];
        lua_createtable(L, 0, 3);
        lua_createtable(L, 3, 0);
        lua_pushnumber(L, pl.position.x); lua_seti(L, -2, 1);
        lua_pushnumber(L, pl.position.y); lua_seti(L, -2, 2);
        lua_pushnumber(L, pl.position.z); lua_seti(L, -2, 3);
        lua_setfield(L, -2, "position");
        lua_pushnumber(L, pl.yaw); lua_setfield(L, -2, "yaw");
        lua_pushnumber(L, pl.scale); lua_setfield(L, -2, "scale");
        lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    return 1;
}

void registerMetatable(lua_State* L, const char* name, lua_CFunction gc) {
    if (luaL_newmetatable(L, name)) {
        lua_pushcfunction(L, gc);
        lua_setfield(L, -2, "__gc");
    }
    lua_pop(L, 1);
}

}  // namespace

std::shared_ptr<RenderMesh> luaToMesh(lua_State* L, int idx) {
    void* ud = luaL_testudata(L, idx, kMeshMt);
    if (ud == nullptr) return nullptr;
    return *static_cast<MeshPtr*>(ud);
}

void openProcgenLibrary(ScriptVM& vm) {
    lua_State* L = luaState(vm);

    registerMetatable(L, kSdfMt, sdfGc);
    registerMetatable(L, kMeshMt, meshGc);

    // The LSystem metatable also carries an __index method table (rule/expand),
    // so a script writes `sys:rule(...)` / `sys:expand(...)`.
    if (luaL_newmetatable(L, kLSystemMt)) {
        lua_pushcfunction(L, lsystemGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_lsystem_rule);
        lua_setfield(L, -2, "rule");
        lua_pushcfunction(L, l_lsystem_expand);
        lua_setfield(L, -2, "expand");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);

    // ParametricLSystem: rule/expand methods; ModuleString is opaque (just __gc).
    if (luaL_newmetatable(L, kPLSystemMt)) {
        lua_pushcfunction(L, plsystemGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_plsystem_rule);
        lua_setfield(L, -2, "rule");
        lua_pushcfunction(L, l_plsystem_expand);
        lua_setfield(L, -2, "expand");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);
    registerMetatable(L, kModulesMt, modulesGc);

    // The Scope metatable carries the split-grammar op methods (ADR-0042 Phase 2).
    if (luaL_newmetatable(L, kScopeMt)) {
        lua_pushcfunction(L, scopeGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_scope_corner); lua_setfield(L, -2, "corner");
        lua_pushcfunction(L, l_scope_center); lua_setfield(L, -2, "center");
        lua_pushcfunction(L, l_scope_size);   lua_setfield(L, -2, "size");
        lua_pushcfunction(L, l_scope_split);  lua_setfield(L, -2, "split");
        lua_pushcfunction(L, l_scope_divide); lua_setfield(L, -2, "divide");
        lua_pushcfunction(L, l_scope_inset);  lua_setfield(L, -2, "inset");
        lua_pushcfunction(L, l_scope_box);    lua_setfield(L, -2, "box");
        lua_pushcfunction(L, l_scope_shell);  lua_setfield(L, -2, "shell");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);

    // The Field metatable carries the texture-compose methods (ADR-0043).
    if (luaL_newmetatable(L, kFieldMt)) {
        lua_pushcfunction(L, fieldGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_field_add);        lua_setfield(L, -2, "add");
        lua_pushcfunction(L, l_field_mul);        lua_setfield(L, -2, "mul");
        lua_pushcfunction(L, l_field_mix);        lua_setfield(L, -2, "mix");
        lua_pushcfunction(L, l_field_scale_bias); lua_setfield(L, -2, "scale_bias");
        lua_pushcfunction(L, l_field_clamp);      lua_setfield(L, -2, "clamp");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);
    registerMetatable(L, kImageMt, imageGc);      // Image is opaque (just __gc)
    registerMetatable(L, kMaterialMt, materialGc);  // Material is opaque (just __gc)

    // The HeightField metatable carries the terrain-compose methods (ADR-0043).
    if (luaL_newmetatable(L, kHeightMt)) {
        lua_pushcfunction(L, heightGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_height_add);   lua_setfield(L, -2, "add");
        lua_pushcfunction(L, l_height_mul);   lua_setfield(L, -2, "mul");
        lua_pushcfunction(L, l_height_max);   lua_setfield(L, -2, "max");
        lua_pushcfunction(L, l_height_min);   lua_setfield(L, -2, "min");
        lua_pushcfunction(L, l_height_scale); lua_setfield(L, -2, "scale");
        lua_pushcfunction(L, l_height_mix);   lua_setfield(L, -2, "mix");
        lua_pushcfunction(L, l_height_clamp); lua_setfield(L, -2, "clamp");
        lua_pushcfunction(L, l_height_at);    lua_setfield(L, -2, "at");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);

    // The Model metatable carries the compose methods (ADR-0042 Phase 3).
    if (luaL_newmetatable(L, kModelMt)) {
        lua_pushcfunction(L, modelGc);
        lua_setfield(L, -2, "__gc");
        lua_newtable(L);
        lua_pushcfunction(L, l_model_add);            lua_setfield(L, -2, "add");
        lua_pushcfunction(L, l_model_add_solid);      lua_setfield(L, -2, "add_solid");
        lua_pushcfunction(L, l_model_add_instances);  lua_setfield(L, -2, "add_instances");
        lua_pushcfunction(L, l_model_collide);        lua_setfield(L, -2, "collide");
        lua_pushcfunction(L, l_model_add_collider);   lua_setfield(L, -2, "add_collider");
        lua_pushcfunction(L, l_model_merge);          lua_setfield(L, -2, "merge");
        lua_pushcfunction(L, l_model_part_count);     lua_setfield(L, -2, "part_count");
        lua_pushcfunction(L, l_model_instance_count); lua_setfield(L, -2, "instance_count");
        lua_pushcfunction(L, l_model_collider_count); lua_setfield(L, -2, "collider_count");
        lua_pushcfunction(L, l_model_flatten);        lua_setfield(L, -2, "flatten");
        lua_setfield(L, -2, "__index");
    }
    lua_pop(L, 1);

    static const luaL_Reg kSdfFns[] = {
        {"sphere", l_sdf_sphere},
        {"box", l_sdf_box},
        {"capsule", l_sdf_capsule},
        {"union", l_sdf_union},
        {"intersect", l_sdf_intersect},
        {"subtract", l_sdf_subtract},
        {"smooth_union", l_sdf_smooth_union},
        {"smooth_union_all", l_sdf_smooth_union_all},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kSdfFns);
    lua_setglobal(L, "sdf");

    static const luaL_Reg kNoiseFns[] = {
        {"value2", l_noise_value2},
        {"fbm2", l_noise_fbm2},
        {"fbm3", l_noise_fbm3},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kNoiseFns);
    lua_setglobal(L, "noise");

    static const luaL_Reg kMeshFns[] = {
        {"box", l_mesh_box},
        {"sphere", l_mesh_sphere},
        {"cylinder", l_mesh_cylinder},
        {"cone", l_mesh_cone},
        {"plane", l_mesh_plane},
        {"torus", l_mesh_torus},
        {"capsule", l_mesh_capsule},
        {"merge", l_mesh_merge},
        {"translate", l_mesh_translate},
        {"place", l_mesh_place},
        {"scale", l_mesh_scale},
        {"rotate_x", l_mesh_rotate_x},
        {"rotate_y", l_mesh_rotate_y},
        {"rotate_z", l_mesh_rotate_z},
        {"orient", l_mesh_orient},
        {"recompute_normals", l_mesh_recompute_normals},
        {"bake_height_color", l_mesh_bake_height_color},
        {"quad", l_mesh_quad},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kMeshFns);
    lua_setglobal(L, "mesh");

    lua_pushcfunction(L, l_scope);
    lua_setglobal(L, "scope");

    static const luaL_Reg kLSystemFns[] = {
        {"create", l_lsystem_create},
        {"parametric", l_lsystem_parametric},
        {"turtle_mesh", l_turtle_mesh},
        {"turtle_mesh_sdf", l_turtle_mesh_sdf},
        {"segments", l_lsystem_segments},
        {"leaves", l_lsystem_leaves},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kLSystemFns);
    lua_setglobal(L, "lsystem");

    // tree.skin(modules, params, seed) -> bark, leaves — the grammar-agnostic
    // skinner, so Lua authors the grammar (lsystem.parametric) and skins it.
    static const luaL_Reg kTreeFns[] = {
        {"skin", l_tree_skin},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kTreeFns);
    lua_setglobal(L, "tree");

    static const luaL_Reg kBuildingFns[] = {
        {"grow", l_building_grow},
        {"height", l_building_height},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kBuildingFns);
    lua_setglobal(L, "building");

    static const luaL_Reg kFurnitureFns[] = {
        {"lamp", l_furniture_lamp},
        {"traffic_signal", l_furniture_traffic_signal},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kFurnitureFns);
    lua_setglobal(L, "streetfurniture");

    static const luaL_Reg kModelFns[] = {
        {"new", l_model_new},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kModelFns);
    lua_setglobal(L, "model");

    static const luaL_Reg kCityFns[] = {
        {"layout", l_city_layout},
        {"lots", l_city_lots},
        {"road_mesh", l_city_road_mesh},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kCityFns);
    lua_setglobal(L, "city");

    static const luaL_Reg kTextureFns[] = {
        {"constant", l_tex_constant},
        {"noise", l_tex_noise},
        {"fbm", l_tex_fbm},
        {"checker", l_tex_checker},
        {"brick", l_tex_brick},
        {"gradient_y", l_tex_gradient_y},
        {"bake_gray", l_tex_bake_gray},
        {"bake_color", l_tex_bake_color},
        {"bake_normal", l_tex_bake_normal},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kTextureFns);
    lua_setglobal(L, "texture");

    static const luaL_Reg kMaterialFns[] = {
        {"new", l_material_new},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kMaterialFns);
    lua_setglobal(L, "material");

    lua_pushcfunction(L, l_polygonize);
    lua_setglobal(L, "polygonize");
    // `terrain` is a table of heightfield ops (ADR-0043) that is also CALLABLE:
    // terrain(params, seed) runs the C++ preset via __call; terrain.fbm{...} etc.
    // compose a heightfield, terrain.mesh(field, {...}) tessellates it.
    static const luaL_Reg kTerrainFns[] = {
        {"flat", l_terr_flat},     {"noise", l_terr_noise},
        {"fbm", l_terr_fbm},       {"ridged", l_terr_ridged},
        {"warp", l_terr_warp},     {"terrace", l_terr_terrace},
        {"erode", l_terr_erode},   {"conform", l_terr_conform},
        {"mesh", l_terr_mesh},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kTerrainFns);
    lua_newtable(L);                                  // its metatable
    lua_pushcfunction(L, l_terrain_preset);
    lua_setfield(L, -2, "__call");
    lua_setmetatable(L, -2);
    lua_setglobal(L, "terrain");
    lua_pushcfunction(L, l_scatter);
    lua_setglobal(L, "scatter");
}

bool runProcgenMesh(ScriptVM& vm, const std::string& code,
                    std::shared_ptr<RenderMesh>& out, std::string* error) {
    lua_State* L = luaState(vm);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);
        return false;
    }

    if (auto* mesh = static_cast<MeshPtr*>(luaL_testudata(L, -1, kMeshMt))) {
        out = *mesh;
        lua_pop(L, 1);
        return true;
    }
    // A Model return flattens to a single mesh (parts + stamped instances), so a
    // composed recipe works with single-mesh consumers / preview (ADR-0042).
    if (auto* mdl = static_cast<ModelPtr*>(luaL_testudata(L, -1, kModelMt))) {
        out = std::make_shared<RenderMesh>(flattenModel(**mdl));
        lua_pop(L, 1);
        return true;
    }
    if (error != nullptr) *error = "procgen script did not return a Mesh or Model";
    lua_pop(L, 1);
    return false;
}

namespace {
// Recursively push a JSON value onto the Lua stack (objects -> keyed tables,
// arrays -> 1-based tables, primitives map across).
void pushJson(lua_State* L, const nlohmann::json& j) {
    if (j.is_object()) {
        lua_newtable(L);
        for (auto it = j.begin(); it != j.end(); ++it) {
            pushJson(L, it.value());
            lua_setfield(L, -2, it.key().c_str());
        }
    } else if (j.is_array()) {
        lua_newtable(L);
        lua_Integer i = 1;
        for (const auto& e : j) {
            pushJson(L, e);
            lua_seti(L, -2, i++);
        }
    } else if (j.is_boolean()) {
        lua_pushboolean(L, j.get<bool>());
    } else if (j.is_number()) {
        lua_pushnumber(L, j.get<double>());
    } else if (j.is_string()) {
        lua_pushstring(L, j.get<std::string>().c_str());
    } else {
        lua_pushnil(L);
    }
}
}  // namespace

void setRecipeArgs(ScriptVM& vm, const std::string& optsJson) {
    nlohmann::json opts = nlohmann::json::parse(optsJson, nullptr, false);
    if (opts.is_discarded() || !opts.is_object()) return;   // ignore malformed opts
    lua_State* L = luaState(vm);
    pushJson(L, opts);
    lua_setglobal(L, "args");
}

bool runProcgenModelValue(ScriptVM& vm, const std::string& code, ProcModel& out,
                          std::string* error) {
    lua_State* L = luaState(vm);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);
        return false;
    }
    if (auto* mdl = static_cast<ModelPtr*>(luaL_testudata(L, -1, kModelMt))) {
        out = **mdl;                     // copy out the parts + instance groups
        lua_pop(L, 1);
        return true;
    }
    if (auto* mesh = static_cast<MeshPtr*>(luaL_testudata(L, -1, kMeshMt))) {
        { ProcPart p; p.mesh = **mesh; out.parts.push_back(std::move(p)); }  // bare mesh = one part
        lua_pop(L, 1);
        return true;
    }
    if (error != nullptr) *error = "procgen script did not return a Model or Mesh";
    lua_pop(L, 1);
    return false;
}

bool runProcgenTexture(ScriptVM& vm, const std::string& code, TextureData& out,
                       std::string* error) {
    lua_State* L = luaState(vm);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);
        return false;
    }
    if (auto* img = static_cast<ImagePtr*>(luaL_testudata(L, -1, kImageMt))) {
        out = **img;
        lua_pop(L, 1);
        return true;
    }
    if (error != nullptr) *error = "procgen script did not return an Image";
    lua_pop(L, 1);
    return false;
}

namespace {
// Read one model part from the value at `idx`: either a Mesh userdata, or a
// table { mesh=, texture=, alpha_test=, albedo=, roughness=, metallic= }.
bool readModelPart(lua_State* L, int idx, ScriptMeshPart& part) {
    idx = lua_absindex(L, idx);
    if (auto* m = static_cast<MeshPtr*>(luaL_testudata(L, idx, kMeshMt))) {
        part.mesh = *m;
        part.hasMaterial = true;   // table/list parts describe themselves
        return true;
    }
    if (!lua_istable(L, idx)) return false;
    lua_getfield(L, idx, "mesh");
    auto* m = static_cast<MeshPtr*>(luaL_testudata(L, -1, kMeshMt));
    if (m == nullptr) { lua_pop(L, 1); return false; }
    part.mesh = *m;
    lua_pop(L, 1);
    part.hasMaterial = true;
    part.roughness = static_cast<float>(optField(L, idx, "roughness", part.roughness));
    part.metallic = static_cast<float>(optField(L, idx, "metallic", part.metallic));
    lua_getfield(L, idx, "alpha_test");
    part.alphaTest = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    lua_getfield(L, idx, "wind");
    part.wind = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    lua_getfield(L, idx, "texture");
    if (lua_isstring(L, -1)) part.texture = lua_tostring(L, -1);
    lua_pop(L, 1);
    part.albedo = optVec3Field(L, idx, "albedo", part.albedo);
    return true;
}
}  // namespace

bool runProcgenModel(ScriptVM& vm, const std::string& code,
                     std::vector<ScriptMeshPart>& out, std::string* error) {
    lua_State* L = luaState(vm);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);
        return false;
    }

    out.clear();
    // A single Mesh return -> one part using the caller's default material.
    if (luaL_testudata(L, -1, kMeshMt) != nullptr) {
        ScriptMeshPart part;
        readModelPart(L, -1, part);
        part.hasMaterial = false;
        out.push_back(std::move(part));
        lua_pop(L, 1);
        return true;
    }
    // Otherwise a list (array) of parts (Mesh userdata or part tables).
    if (lua_istable(L, -1)) {
        lua_Integer n = luaL_len(L, -1);
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_geti(L, -1, i);
            ScriptMeshPart part;
            if (readModelPart(L, -1, part)) out.push_back(std::move(part));
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
    if (out.empty()) {
        if (error != nullptr) *error = "procgen script did not return a model";
        return false;
    }
    return true;
}

}  // namespace engine
