#include "vehicle_body.h"

#include "../../../engine/scripting/lua_state.h"   // luaState() + the Lua C API (scripting-internal)
#include "../../../engine/scripting/procgen_mesh.h"   // luaToMesh (mesh.car shells)
#include "../../../engine/mesh_builder.h"          // MeshBuilder::box (same primitive as addBox)

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

// String field of the table at absolute stack index `t` ("" when absent).
std::string strField(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    const char* s = lua_isstring(L, -1) ? lua_tostring(L, -1) : nullptr;
    std::string v = s ? s : "";
    lua_pop(L, 1);
    return v;
}

// A {x,y,z} array-field of the table at absolute stack index `t` (like
// vehicle_spec's reader): missing components keep `def`.
Vec3 vec3Field(lua_State* L, int t, const char* key, Vec3 def) {
    lua_getfield(L, t, key);
    Vec3 r = def;
    if (lua_istable(L, -1)) {
        int st = lua_gettop(L);
        lua_rawgeti(L, st, 1);
        if (lua_isnumber(L, -1)) r.x = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 2);
        if (lua_isnumber(L, -1)) r.y = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 3);
        if (lua_isnumber(L, -1)) r.z = lua_tonumber(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return r;
}

// Boolean field of the table at absolute stack index `t` (`def` when absent).
bool boolField(lua_State* L, int t, const char* key, bool def) {
    lua_getfield(L, t, key);
    bool v = lua_isnil(L, -1) ? def : lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return v;
}

// Concatenate `src` onto `dst`, rebasing indices.
void appendMesh(RenderMesh& dst, const RenderMesh& src) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (uint32_t i : src.indices) dst.indices.push_back(base + i);
}

// Read an optional Mesh-valued field into `out`; false only when the field is
// present but is not a usable Mesh (absent is fine, and leaves `out` empty).
bool optMeshField(lua_State* L, int t, const char* key, RenderMesh& out,
                  bool& present) {
    lua_getfield(L, t, key);
    present = !lua_isnil(L, -1);
    bool ok = true;
    if (present) {
        std::shared_ptr<RenderMesh> m = luaToMesh(L, -1);
        if (!m || m->vertices.empty()) ok = false;
        else out = *m;
    }
    lua_pop(L, 1);
    return ok;
}

// Push `vehicle.fleet[slot+1]` onto the stack. On failure the stack is restored
// to `base` and false is returned with `err` filled.
bool pushFleetSlot(lua_State* L, int base, int slot, const std::string& where,
                   std::string* err) {
    lua_getglobal(L, "vehicle");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "vehicles.lua: no global `vehicle` table (load it first)");
    }
    lua_getfield(L, -1, "fleet");
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, "vehicles.lua: no `vehicle.fleet` array");
    }
    const int fleet = lua_gettop(L);
    const int n = static_cast<int>(luaL_len(L, fleet));
    if (slot < 0 || slot >= n) {
        lua_settop(L, base);
        return fail(err, where + ": slot out of range (fleet has " +
                             std::to_string(n) + ")");
    }
    lua_rawgeti(L, fleet, slot + 1);
    if (!lua_istable(L, -1)) {
        lua_settop(L, base);
        return fail(err, where + " is not a table");
    }
    return true;
}

// Append a vertex-coloured box (centred at `c`, dimensions `size`) into `out` —
// the EXACT primitive citysim's addBox uses (MeshBuilder::box, offset, tint), so
// a Lua-authored body winds identically to the box car it replaced.
void addColoredBox(RenderMesh& out, Vec3 size, Vec3 c, Vec3 color) {
    RenderMesh b = MeshBuilder::box(size);
    uint32_t base = static_cast<uint32_t>(out.vertices.size());
    for (Vertex v : b.vertices) {
        v.position = v.position + c;
        v.color = color;
        out.vertices.push_back(v);
    }
    for (uint32_t i : b.indices) out.indices.push_back(base + i);
}

}  // namespace

bool loadFleetCarBody(ScriptVM& vm, int slot, CarBodyRecipe& out,
                      std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const std::string where =
        "vehicle.fleet[" + std::to_string(slot + 1) + "]";

    if (!pushFleetSlot(L, base, slot, where, err)) return false;
    int rec = lua_gettop(L);
    CarBodyRecipe body;

    // The catalogue half rides on the SLOT, not on whatever build() returns —
    // read it before the geometry so a slot keeps its declared size either way.
    body.size = vec3Field(L, rec, "size", Vec3(0, 0, 0));
    body.className = strField(L, rec, "class");

    // GEOMETRY ON DEMAND. A slot may publish `build()` instead of a body: call
    // it now and read the recipe it returns. This is the expensive half (a
    // mesh.car shell per slot), which is exactly why it is behind a function —
    // loadFleetCatalogueEntry never calls it.
    lua_getfield(L, rec, "build");
    if (lua_isfunction(L, -1)) {
        if (lua_pcall(L, 0, 1, 0) != 0) {
            const char* msg = lua_tostring(L, -1);
            const std::string detail = msg ? msg : "unknown Lua error";
            lua_settop(L, base);
            return fail(err, where + ": build() failed: " + detail);
        }
        if (!lua_istable(L, -1)) {
            lua_settop(L, base);
            return fail(err, where + ": build() did not return a table");
        }
        rec = lua_gettop(L);   // read the built recipe from here on
    } else {
        lua_pop(L, 1);         // no build(): the slot itself is the recipe
    }

    // The CHASSIS accumulates first — everything that is not a wheel. The
    // wheelset joins it afterwards to make the instanced mesh, so the two forms
    // the city needs (with wheels for ambient traffic, without for a promoted
    // physics car) are cut from one piece of geometry.
    RenderMesh chassis;

    // body = a procgen Mesh (mesh.car's shell — the SAME generator the car lab
    // and the drivable spec use). Optional: a slot may be mesh-only, boxes-only
    // (the legacy box fleet), or a mesh shell PLUS box parts. Vertex colours
    // ride the mesh, matching carMaterial's white albedo exactly as the box path
    // did.
    bool hasBody = false;
    if (!optMeshField(L, rec, "body", chassis, hasBody)) {
        lua_settop(L, base);
        return fail(err, where + ": `body` is not a non-empty Mesh");
    }

    // parts = { { pos={x,y,z}, size={w,h,l}, color={r,g,b} }, ... }
    // Required only when there is no `body` mesh to carry the slot.
    lua_getfield(L, rec, "parts");
    const bool hasParts = lua_istable(L, -1);
    if (!hasParts && !hasBody) {
        lua_settop(L, base);
        return fail(err, where + ": missing `parts` array (and no `body` mesh)");
    }
    const int parts = lua_gettop(L);
    const int nParts = hasParts ? static_cast<int>(luaL_len(L, parts)) : 0;
    if (nParts <= 0 && !hasBody) {
        lua_settop(L, base);
        return fail(err, where + ": empty `parts` array (and no `body` mesh)");
    }
    for (int i = 1; i <= nParts; ++i) {
        lua_rawgeti(L, parts, i);
        const int pi = lua_gettop(L);
        if (!lua_istable(L, pi)) {
            lua_settop(L, base);
            return fail(err, where + ": parts[" + std::to_string(i) +
                                 "] is not a table");
        }
        Vec3 size = vec3Field(L, pi, "size", Vec3(0, 0, 0));
        Vec3 pos = vec3Field(L, pi, "pos", Vec3(0, 0, 0));
        Vec3 color = vec3Field(L, pi, "color", Vec3(1, 1, 1));
        lua_pop(L, 1);
        if (size.x <= 0 || size.y <= 0 || size.z <= 0) {
            lua_settop(L, base);
            return fail(err, where + ": parts[" + std::to_string(i) +
                                 "] has a non-positive size");
        }
        addColoredBox(chassis, size, pos, color);
    }
    lua_pop(L, 1);   // parts

    // wheels = the baked wheelset as its own Mesh, so the chassis above stays
    // wheel-less. Absent on a legacy recipe (its wheels, if any, were boxes in
    // `parts` and are already part of the chassis) — in which case there is no
    // honest wheel-less form to offer and `chassis` is left empty below.
    RenderMesh wheelMesh;
    bool hasWheels = false;
    if (!optMeshField(L, rec, "wheels", wheelMesh, hasWheels)) {
        lua_settop(L, base);
        return fail(err, where + ": `wheels` is not a non-empty Mesh");
    }

    // wheel_layout = { { pos=, radius=, width=, steered=, driven=, hand_brake= } }
    lua_getfield(L, rec, "wheel_layout");
    if (lua_istable(L, -1)) {
        const int layout = lua_gettop(L);
        const int nWheels = static_cast<int>(luaL_len(L, layout));
        for (int i = 1; i <= nWheels; ++i) {
            lua_rawgeti(L, layout, i);
            const int wi = lua_gettop(L);
            if (!lua_istable(L, wi)) {
                lua_settop(L, base);
                return fail(err, where + ": wheel_layout[" + std::to_string(i) +
                                     "] is not a table");
            }
            WheelPlacement w;
            w.pos = vec3Field(L, wi, "pos", Vec3(0, 0, 0));
            lua_getfield(L, wi, "radius");
            w.radius = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0.0;
            lua_pop(L, 1);
            lua_getfield(L, wi, "width");
            w.width = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : 0.0;
            lua_pop(L, 1);
            w.steered = boolField(L, wi, "steered", false);
            w.driven = boolField(L, wi, "driven", true);
            w.handBrake = boolField(L, wi, "hand_brake", false);
            lua_pop(L, 1);
            if (w.radius <= 0 || w.width <= 0) {
                lua_settop(L, base);
                return fail(err, where + ": wheel_layout[" + std::to_string(i) +
                                     "] has a non-positive radius/width");
            }
            body.wheels.push_back(w);
        }
    }
    lua_pop(L, 1);   // wheel_layout (or non-table)

    // lights = { { name=, pos={x,y,z} }, ... } — named lamp attachment markers.
    lua_getfield(L, rec, "lights");
    if (lua_istable(L, -1)) {
        const int lights = lua_gettop(L);
        const int nLights = static_cast<int>(luaL_len(L, lights));
        for (int i = 1; i <= nLights; ++i) {
            lua_rawgeti(L, lights, i);
            const int li = lua_gettop(L);
            if (!lua_istable(L, li)) {
                lua_settop(L, base);
                return fail(err, where + ": lights[" + std::to_string(i) +
                                     "] is not a table");
            }
            std::string name = strField(L, li, "name");
            Vec3 pos = vec3Field(L, li, "pos", Vec3(0, 0, 0));
            lua_pop(L, 1);
            if (name.empty()) {
                lua_settop(L, base);
                return fail(err, where + ": lights[" + std::to_string(i) +
                                     "] has no `name`");
            }
            body.lights.push_back({name, pos});
        }
    }
    lua_pop(L, 1);   // lights (or non-table)

    // Compose the two forms the city needs. `mesh` is always the whole car, so
    // every existing consumer is unaffected. `chassis` is filled ONLY when the
    // recipe published a separate wheelset — otherwise there is no honest
    // wheel-less form to hand out, and an empty chassis says exactly that.
    body.mesh = chassis;
    if (hasWheels) {
        appendMesh(body.mesh, wheelMesh);
        body.chassis = std::move(chassis);
    }

    lua_settop(L, base);
    out = std::move(body);
    return true;
}

int fleetSlotCount(ScriptVM& vm) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    int n = 0;
    lua_getglobal(L, "vehicle");
    if (lua_istable(L, -1)) {
        lua_getfield(L, -1, "fleet");
        if (lua_istable(L, -1)) n = static_cast<int>(luaL_len(L, lua_gettop(L)));
    }
    lua_settop(L, base);
    return n;
}

bool loadFleetCatalogueEntry(ScriptVM& vm, int slot, Vec3& size,
                             std::string& className, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    const std::string where = "vehicle.fleet[" + std::to_string(slot + 1) + "]";
    if (!pushFleetSlot(L, base, slot, where, err)) return false;
    const int rec = lua_gettop(L);

    // Deliberately does NOT touch `build` — reading a car's measurements must
    // not cost a car's geometry.
    size = vec3Field(L, rec, "size", Vec3(0, 0, 0));
    className = strField(L, rec, "class");
    lua_settop(L, base);

    if (size.x < 0 || size.y < 0 || size.z < 0)
        return fail(err, where + ": `size` has a negative component");
    return true;
}

}  // namespace engine
