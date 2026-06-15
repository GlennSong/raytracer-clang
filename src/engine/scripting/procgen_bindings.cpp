#include "procgen_bindings.h"

#include "procgen_mesh.h"
#include "lua_state.h"
#include "../procgen/sdf.h"
#include "../procgen/noise.h"
#include "../procgen/lsystem.h"
#include "../procgen/tree.h"
#include "../procgen/terrain.h"
#include "../procgen/scatter.h"
#include "../mesh_builder.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"

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

// Read an optional {x,y,z} field; fall back if absent/not a table.
Vec3 optVec3Field(lua_State* L, int idx, const char* key, Vec3 fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    Vec3 v = lua_istable(L, -1) ? checkVec3(L, -1) : fallback;
    lua_pop(L, 1);
    return v;
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
    p.leafSize     = static_cast<float>(optField(L, idx, "leaf_size", p.leafSize));
    p.leavesPerTip = static_cast<int>(optField(L, idx, "leaves_per_tip", p.leavesPerTip));
    p.leafThickness = static_cast<float>(optField(L, idx, "leaf_thickness", p.leafThickness));
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
int l_terrain(lua_State* L) {                   // terrain(params, seed) -> Mesh
    TerrainParams p = readTerrainParams(L, 1);
    auto seed = static_cast<uint32_t>(luaL_optinteger(L, 2, 0));
    pushMesh(L, std::make_shared<RenderMesh>(generateTerrain(p, Noise(seed))));
    return 1;
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
        {"scale", l_mesh_scale},
        {"rotate_x", l_mesh_rotate_x},
        {"rotate_y", l_mesh_rotate_y},
        {"rotate_z", l_mesh_rotate_z},
        {"orient", l_mesh_orient},
        {"recompute_normals", l_mesh_recompute_normals},
        {"bake_height_color", l_mesh_bake_height_color},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kMeshFns);
    lua_setglobal(L, "mesh");

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

    lua_pushcfunction(L, l_polygonize);
    lua_setglobal(L, "polygonize");
    lua_pushcfunction(L, l_terrain);
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

    auto* mesh = static_cast<MeshPtr*>(luaL_testudata(L, -1, kMeshMt));
    if (mesh == nullptr) {
        if (error != nullptr) *error = "procgen script did not return a Mesh";
        lua_pop(L, 1);
        return false;
    }
    out = *mesh;
    lua_pop(L, 1);
    return true;
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
