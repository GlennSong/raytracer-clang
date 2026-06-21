#include "test_framework.h"

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/procgen/texture_field.h"   // TextureData
#include "../src/engine/procgen/sdf.h"
#include "../src/engine/mesh_builder.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

// --- the VM seal (ADR-0023) ---

TEST_CASE(script_vm_runs_and_reads_globals) {
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString("answer = 6 * 7", &err));
    double a = 0;
    CHECK(vm.getGlobalNumber("answer", a));
    CHECK_APPROX(a, 42.0, 1e-12);
}

TEST_CASE(script_vm_reports_errors) {
    ScriptVM vm;
    std::string err = "unset";
    CHECK(!vm.doString("this is not valid lua", &err));
    CHECK(!err.empty());
    CHECK(err != "unset");
}

TEST_CASE(script_vm_sandbox_blocks_io_os_require) {
    ScriptVM vm;
    // The procgen sandbox opens no io/os/package, so those globals are nil and a
    // script cannot reach the filesystem, the clock, or native loading.
    bool b = false;
    CHECK(vm.doString("ok_io = (io == nil)"));
    CHECK(vm.getGlobalBool("ok_io", b)); CHECK(b);
    b = false;
    CHECK(vm.doString("ok_os = (os == nil)"));
    CHECK(vm.getGlobalBool("ok_os", b)); CHECK(b);
    b = false;
    CHECK(vm.doString("ok_req = (require == nil)"));
    CHECK(vm.getGlobalBool("ok_req", b)); CHECK(b);

    // ...but the pure libraries ARE available.
    double pi = 0;
    CHECK(vm.doString("p = math.pi"));
    CHECK(vm.getGlobalNumber("p", pi));
    CHECK_APPROX(pi, 3.14159265358979, 1e-10);
}

// --- the procgen binding surface ---

TEST_CASE(procgen_script_builds_an_sdf_sphere_mesh) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local s = sdf.sphere({0, 0, 0}, 1.0)
        return polygonize(s, {min = {-1.5,-1.5,-1.5}, max = {1.5,1.5,1.5}}, 24)
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 50);
    CHECK(mesh->indices.size() % 3 == 0);

    // Same property test_sdf.cpp asserts for the C++ path: vertices sit on the
    // unit sphere (within a grid cell).
    const double cell = 3.0 / 24;
    bool onSurface = true;
    for (const Vertex& v : mesh->vertices)
        if (std::fabs(v.position.length() - 1.0) > 1.5 * cell) onSurface = false;
    CHECK(onSurface);
}

TEST_CASE(procgen_script_matches_the_cpp_substrate) {
    // ADR-0023: the Lua front-end and the C++ generators are the SAME substrate.
    // An identical recipe must yield an identical mesh.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> viaScript;
    const char* code = R"LUA(
        local a = sdf.sphere({-0.6, 0, 0}, 0.7)
        local b = sdf.sphere({ 0.6, 0, 0}, 0.7)
        return polygonize(sdf.smooth_union(a, b, 0.4),
                          {min = {-2,-2,-2}, max = {2,2,2}}, 32)
    )LUA";
    CHECK(runProcgenMesh(vm, code, viaScript, nullptr));
    if (!viaScript) { CHECK(false); return; }

    Sdf blob = sdfSmoothUnion(sdfSphere(Vec3(-0.6, 0, 0), 0.7),
                              sdfSphere(Vec3(0.6, 0, 0), 0.7), 0.4);
    RenderMesh viaCpp =
        polygonizeSdf(blob, SdfBounds{Vec3(-2, -2, -2), Vec3(2, 2, 2)}, 32);

    CHECK(viaScript->vertices.size() == viaCpp.vertices.size());
    CHECK(viaScript->indices.size() == viaCpp.indices.size());
}

TEST_CASE(procgen_script_grows_a_building) {
    // ADR-0038/0028: the split/shape grammar is an L1 sibling exposed to Lua, the
    // same way the L-system is. building.grow returns a mesh; taller floors -> more
    // geometry; building.height is a cheap query.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        return building.grow{ floors = 6, width = 18, depth = 14,
                              ground_retail = true, walkable_ground = true, seed = 3 }
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 100);
    CHECK(mesh->indices.size() % 3 == 0);

    double h6 = 0, h12 = 0;
    CHECK(vm.doString("h6  = building.height{ floors = 6 }"));
    CHECK(vm.doString("h12 = building.height{ floors = 12 }"));
    CHECK(vm.getGlobalNumber("h6", h6));
    CHECK(vm.getGlobalNumber("h12", h12));
    CHECK(h12 > h6);
    CHECK_APPROX(h6, 6 * 3.2 + 4.5 + 1.1, 1e-6);   // matches the C++ grammar
}

TEST_CASE(procgen_script_builds_street_furniture) {
    // ADR-0042: the street kit is exposed as tunable Lua recipes wrapping the
    // same C++ builder the city uses. A lamp and a signal each return a mesh, and
    // a taller lamp yields a taller mesh (the params reach the builder).
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> lamp, signal;
    std::string err;
    CHECK(runProcgenMesh(vm, "return streetfurniture.lamp{ height = 4.6 }", lamp, &err));
    CHECK(runProcgenMesh(vm, "return streetfurniture.traffic_signal{}", signal, &err));
    if (!lamp || !signal) { CHECK(false); return; }
    CHECK(lamp->vertices.size() > 12);
    CHECK(signal->indices.size() % 3 == 0);
    CHECK(signal->vertices.size() > lamp->vertices.size());   // signal is richer

    // A taller lamp reaches higher (the height param flows to the C++ builder).
    auto topY = [](const RenderMesh& m) {
        double y = -1e30;
        for (const Vertex& v : m.vertices) y = std::max(y, double(v.position.y));
        return y;
    };
    std::shared_ptr<RenderMesh> tall;
    CHECK(runProcgenMesh(vm, "return streetfurniture.lamp{ height = 8.0 }", tall, nullptr));
    if (tall) CHECK(topY(*tall) > topY(*lamp));
}

TEST_CASE(procgen_script_authors_with_scope_ops) {
    // ADR-0042 Phase 2: the split/shape-grammar op-vocabulary is exposed, so Lua
    // can subdivide a scope and emit geometry per cell — author props/details,
    // not just whole buildings. The ops wrap the same C++ grammar.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local s = scope{ origin = {0, 0, 0}, size = {4, 3, 2} }
        local bays = s:split('x', { 1, 1, 1, 1 })      -- four equal bays
        local parts = {}
        for i = 1, #bays do parts[i] = bays[i]:box{0.2, 0.6, 0.8} end
        return mesh.merge(parts)
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 0);
    CHECK(mesh->indices.size() % 3 == 0);

    // split returns the requested number of cells; divide sizes by target.
    double n = 0, halved = 0, divided = 0;
    CHECK(vm.doString("n = #(scope{origin={0,0,0},size={4,3,2}}:split('x',{1,1,1,1}))"));
    CHECK(vm.getGlobalNumber("n", n));
    CHECK(n == 4);
    CHECK(vm.doString("divided = #(scope{origin={0,0,0},size={10,3,2}}:divide('x', 2.0))"));
    CHECK(vm.getGlobalNumber("divided", divided));
    CHECK(divided == 5);                                  // 10 / 2 = 5 cells

    // inset shrinks the footprint but keeps height; corner/center read back.
    CHECK(vm.doString("halved = (scope{origin={0,0,0},size={4,3,4}}:inset(1.0)):size()[1]"));
    CHECK(vm.getGlobalNumber("halved", halved));
    CHECK_APPROX(halved, 2.0, 1e-6);                     // 4 - 2*1 = 2 wide

    // mesh.quad emits a single panel (two triangles).
    std::shared_ptr<RenderMesh> quad;
    CHECK(runProcgenMesh(vm,
        "return mesh.quad({0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,1,1})",
        quad, nullptr));
    if (quad) CHECK(quad->indices.size() == 6);
}

TEST_CASE(procgen_script_composes_a_model) {
    // ADR-0042 Phase 3: a recipe returns a composable Model — parts + instance
    // groups — not just one mesh. It flattens for single-mesh consumers, and
    // models merge so parts nest into collections.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local m = model.new()
        m:add(mesh.box{8, 0.2, 8})                 -- a ground slab part
        local lamp = streetfurniture.lamp{}
        m:add_instances(lamp, {
            { pos = {-3, 0, 0} },
            { pos = { 3, 0, 0} },
            { pos = { 0, 0, 3}, yaw = 1.57, scale = 1.2 },
        })
        return m
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));   // a Model flattens to a mesh
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 0);
    CHECK(mesh->indices.size() % 3 == 0);

    // Structure + composition, inspected from Lua.
    double parts = 0, insts = 0;
    CHECK(vm.doString(R"LUA(
        local a = model.new(); a:add(mesh.box{1,1,1})
        local b = model.new(); b:add(mesh.box{1,1,1}); b:add(mesh.box{1,1,1})
        a:merge(b)                                 -- 1 + 2 parts
        mparts = a:part_count()
        local c = model.new()
        c:add_instances(streetfurniture.lamp{}, { {pos={0,0,0}}, {pos={1,0,0}}, {pos={2,0,0}} })
        minsts = c:instance_count()
    )LUA"));
    CHECK(vm.getGlobalNumber("mparts", parts));
    CHECK(parts == 3);
    CHECK(vm.getGlobalNumber("minsts", insts));
    CHECK(insts == 3);
}

TEST_CASE(procgen_script_decorates_the_road_network) {
    // ADR-0042 Phase 3: the road→block solver is exposed (city.layout) so a Lua
    // recipe places props along the real street network and returns a model. The
    // solver stays C++; the recipe just decorates its output.
    ScriptVM vm;
    openProcgenLibrary(vm);
    double nodes = 0, edges = 0, blocks = 0, placed = 0;
    CHECK(vm.doString(R"LUA(
        local L = city.layout{ extent = 200, cell_size = 95, seed = 5 }
        nnodes = #L.nodes
        nedges = #L.edges
        nblocks = #L.blocks
        -- the layout is real geometry: a node has world x/z, an edge indexes nodes
        ok = (L.nodes[1].x ~= nil) and (L.edges[1].a >= 1) and (#L.blocks[1] >= 3)
    )LUA"));
    bool ok = false;
    CHECK(vm.getGlobalBool("ok", ok));
    CHECK(ok);
    CHECK(vm.getGlobalNumber("nnodes", nodes)); CHECK(nodes > 0);
    CHECK(vm.getGlobalNumber("nedges", edges)); CHECK(edges > 0);
    CHECK(vm.getGlobalNumber("nblocks", blocks)); CHECK(blocks > 0);

    // A recipe places one lamp per road segment and returns a model; the instance
    // count equals the edge count (every segment got a lamp).
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local L = city.layout{ extent = 200, cell_size = 95, seed = 5 }
        local p = {}
        for _, e in ipairs(L.edges) do
            local a, b = L.nodes[e.a], L.nodes[e.b]
            p[#p + 1] = { pos = { (a.x + b.x) * 0.5, 0, (a.z + b.z) * 0.5 } }
        end
        local m = model.new()
        m:add_instances(streetfurniture.lamp{}, p)
        placed = m:instance_count()
        return m
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    CHECK(mesh && !mesh->vertices.empty());
    CHECK(vm.getGlobalNumber("placed", placed));
    CHECK(placed == edges);                        // one lamp per road segment
}

TEST_CASE(procgen_model_value_extracts_parts_and_instances) {
    // ADR-0042 Phase 4: runProcgenModelValue pulls the structured ProcModel
    // (parts + instance groups) out of a recipe, so a loader can spawn it with
    // real instancing — not just a flattened mesh.
    ScriptVM vm;
    openProcgenLibrary(vm);
    ProcModel m;
    std::string err;
    const char* code = R"LUA(
        local lay = city.layout{ extent = 200, cell_size = 95, seed = 5 }
        local p = {}
        for _, e in ipairs(lay.edges) do
            local a, b = lay.nodes[e.a], lay.nodes[e.b]
            p[#p + 1] = { pos = { (a.x + b.x) * 0.5, 0, (a.z + b.z) * 0.5 } }
        end
        local m = model.new()
        m:add(mesh.box{4, 0.2, 4})                 -- one baked part
        m:add_instances(streetfurniture.lamp{}, p) -- one instance group
        return m
    )LUA";
    CHECK(runProcgenModelValue(vm, code, m, &err));
    CHECK(m.parts.size() == 1);
    CHECK(m.instances.size() == 1);
    CHECK(m.instanceCount() > 0);              // a lamp per road segment

    // A bare mesh return is accepted as a single part.
    ProcModel single;
    CHECK(runProcgenModelValue(vm, "return mesh.box{1,1,1}", single, nullptr));
    CHECK(single.parts.size() == 1);
    CHECK(single.instances.empty());
}

TEST_CASE(procgen_model_carries_colliders_following_geometry) {
    // ADR-0042: procgen scenery is static world geometry, so a recipe declares
    // colliders that follow the actual mesh — exact trimesh for shells/ground,
    // primitives where the shape truly is one, and per-instance for props.
    ScriptVM vm;
    openProcgenLibrary(vm);
    ProcModel m;
    std::string err;
    const char* code = R"LUA(
        local m = model.new()
        m:add_solid(mesh.box{20, 12, 16})            -- render + exact trimesh
        m:collide(mesh.box{200, 1, 200})             -- ground walk surface
        m:add_collider{ shape = "capsule", center = {0, 5, 0},
                        radius = 4, height = 10 }     -- a round tower, exactly
        local places = { { pos = {10, 0, 0} }, { pos = {-10, 0, 0} } }
        m:add_instances(streetfurniture.lamp{}, places,
                        { collide = { shape = "capsule", radius = 0.3, height = 4 } })
        return m
    )LUA";
    CHECK(runProcgenModelValue(vm, code, m, &err));
    // add_solid contributes one part AND one collider; collide adds another;
    // add_collider a third. The instance group carries its own collision spec.
    CHECK(m.parts.size() == 1);
    CHECK(m.colliders.size() == 3);
    CHECK(m.instances.size() == 1);
    CHECK(m.instances[0].collision == InstanceCollision::Capsule);

    int meshK = 0, capsuleK = 0;
    for (const ProcCollider& c : m.colliders) {
        if (c.kind == ProcCollider::Kind::Mesh) {
            ++meshK;
            CHECK(!c.mesh.vertices.empty());          // real triangles, not a box
        } else if (c.kind == ProcCollider::Kind::Capsule) {
            ++capsuleK;
            CHECK_APPROX(c.radius, 4.0, 1e-9);
            CHECK_APPROX(c.halfHeight, 5.0, 1e-9);    // height 10 -> halfHeight 5
        }
    }
    CHECK(meshK == 2);       // the solid shell + the ground
    CHECK(capsuleK == 1);
}

TEST_CASE(procgen_script_builds_a_brick_texture) {
    // ADR-0043: a brick texture is composed from primitives in Lua (brick lattice
    // × fbm grit, two colours mixed by the mask), then baked — not a preset.
    ScriptVM vm;
    openProcgenLibrary(vm);
    TextureData tex;
    std::string err;
    const char* code = R"LUA(
        local lattice = texture.brick{ cols=6, rows=12, mortar=0.12, variation=0.4, seed=7 }
        local grit = texture.fbm{ seed=9, scale=20, octaves=4 }:scale_bias(0.3, 0.7)
        return texture.bake_color(lattice:mul(grit), {0.62,0.6,0.57}, {0.6,0.2,0.14}, 96)
    )LUA";
    CHECK(runProcgenTexture(vm, code, tex, &err));
    CHECK(tex.width == 96 && tex.height == 96 && tex.channels == 3);
    // Brick faces show (red channel clearly above blue somewhere).
    bool sawBrick = false;
    for (std::size_t i = 0; i + 2 < tex.pixels.size(); i += 3)
        if (tex.pixels[i] > tex.pixels[i + 2] + 40) { sawBrick = true; break; }
    CHECK(sawBrick);
    // A non-Image return is an error.
    TextureData none;
    CHECK(!runProcgenTexture(vm, "return 5", none, nullptr));
}

TEST_CASE(procgen_script_composes_terrain_heightfield) {
    // ADR-0043: terrain composes from primitives, then bakes to a mesh. `terrain`
    // is a callable table — terrain(params, seed) still runs the C++ preset.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local base   = terrain.fbm{ seed=7, freq=0.01, amp=20, octaves=5 }
        local ridges = terrain.ridged{ seed=7, freq=0.006, amp=30, octaves=5 }
        local land   = terrain.warp(base:max(ridges), terrain.fbm{seed=3, freq=0.02, amp=1}, 25)
        return terrain.mesh(land, { size = 256, resolution = 64, color = {0.32, 0.40, 0.26} })
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() == 65u * 65u);
    CHECK(mesh->indices.size() % 3 == 0);

    // The preset path still works through __call (back-compat).
    std::shared_ptr<RenderMesh> preset;
    CHECK(runProcgenMesh(vm,
        "return terrain({size = 50, resolution = 16, height_scale = 5}, 99)",
        preset, &err));
    CHECK(preset && !preset->vertices.empty());
}

TEST_CASE(procgen_terrain_conform_binding_runs) {
    // terrain.conform reads a road layout and cuts/fills the field to it so urban
    // roads sit flat. Smoke-test the binding end to end (city.layout -> conform ->
    // mesh); the cut/fill math itself is covered in test_terrain_field.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local base = terrain.fbm{ seed=5, freq=0.02, amp=25, octaves=5 }
        local lay  = city.layout{ extent=120, cell_size=60, seed=5 }
        local land = terrain.conform(base, lay, { margin=3, falloff=8 })
        return terrain.mesh(land, { size=300, resolution=80, color={0.3,0.4,0.25} })
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() == 81u * 81u);
    CHECK(mesh->indices.size() % 3 == 0);
}

TEST_CASE(procgen_terrain_conform_block_pads_run) {
    // terrain.conform also flattens block pads: the recipe passes a `pads` list of
    // { y, poly } and the ground is cut/filled to a level plot under each. Smoke-
    // test that path end to end (the pad math is in test_terrain_field).
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local base = terrain.fbm{ seed=5, freq=0.02, amp=25, octaves=5 }
        local lay  = city.layout{ extent=120, cell_size=60, seed=5 }
        local pads = {}
        for _, b in ipairs(lay.blocks) do
            local cx, cz, n = 0, 0, 0
            for _, p in ipairs(b) do cx = cx + p.x; cz = cz + p.z; n = n + 1 end
            cx, cz = cx / n, cz / n
            pads[#pads + 1] = { y = base:at(cx, cz), poly = {
                {x=cx-8,z=cz-8}, {x=cx+8,z=cz-8}, {x=cx+8,z=cz+8}, {x=cx-8,z=cz+8} } }
        end
        local land = terrain.conform(base, lay, { pads = pads, pad_falloff = 6 })
        return terrain.mesh(land, { size=300, resolution=64, color={0.3,0.4,0.25} })
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() == 65u * 65u);
}

TEST_CASE(procgen_recipe_args_from_opts) {
    // A level's `opts` is marshalled into the global `args` table so one recipe is
    // parameterizable from the level (numbers, bools, and arrays survive).
    ScriptVM vm;
    openProcgenLibrary(vm);
    setRecipeArgs(vm, R"({"w": 4, "tall": true, "color": [0.1, 0.2, 0.3]})");
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local h = args.tall and args.w * 3 or args.w     -- bool + number
        return mesh.box{ args.w, h, args.color[1] + 4 }  -- array element
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    CHECK(mesh != nullptr);
    // Malformed / non-object opts are ignored, not fatal.
    setRecipeArgs(vm, "not json");
    setRecipeArgs(vm, "[1,2,3]");
}

TEST_CASE(procgen_city_lots_partition_a_block) {
    // city.lots subdivides a block into parcels sized near target_area, each an
    // oriented box small enough to seat a building that FITS the block.
    ScriptVM vm;
    openProcgenLibrary(vm);
    ProcModel m;
    std::string err;
    // Build a model of one small box per lot, and report lot stats via globals.
    const char* code = R"LUA(
        local block = { {x=-30,z=-20}, {x=30,z=-20}, {x=30,z=20}, {x=-30,z=20} }  -- 60x40 = 2400 m^2
        local lots = city.lots(block, { target_area = 500, seed = 3 })
        n_lots = #lots
        max_w, max_d = 0, 0
        local m = model.new()
        for _, lot in ipairs(lots) do
            max_w = math.max(max_w, lot.w); max_d = math.max(max_d, lot.d)
            m:add(mesh.place(mesh.box{ lot.w - 2, 1, lot.d - 2 }, { lot.cx, 0, lot.cz }, lot.yaw))
        end
        return m
    )LUA";
    CHECK(runProcgenModelValue(vm, code, m, &err));
    double nLots = 0, maxW = 0, maxD = 0;
    CHECK(vm.getGlobalNumber("n_lots", nLots));
    CHECK(nLots >= 2.0);                               // a 2400 m^2 block splits up
    CHECK(vm.getGlobalNumber("max_w", maxW));
    CHECK(vm.getGlobalNumber("max_d", maxD));
    CHECK(maxW <= 60.0 && maxD <= 60.0);              // no lot exceeds the block
    CHECK(static_cast<int>(m.parts.size()) == static_cast<int>(nLots));
}

TEST_CASE(procgen_building_style_and_shape) {
    // The variety knobs are exposed: a glass box and a round cylinder tower both
    // grow, and the cylinder's mass differs from the box's.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> box, cyl;
    std::string err;
    CHECK(runProcgenMesh(vm,
        "return building.grow{ floors=8, width=16, depth=14, style='glass', seed=1 }",
        box, &err));
    CHECK(box && !box->vertices.empty());
    CHECK(runProcgenMesh(vm,
        "return building.grow{ floors=8, width=16, depth=14, shape='cylinder', sides=24, seed=1 }",
        cyl, &err));
    CHECK(cyl && !cyl->vertices.empty());
    CHECK(cyl->vertices.size() != box->vertices.size());   // a round mass, not a box
}

TEST_CASE(procgen_non_mesh_return_is_an_error) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    CHECK(!runProcgenMesh(vm, "return 42", mesh, &err));
    CHECK(!err.empty());
    CHECK(mesh == nullptr);
}

TEST_CASE(procgen_noise_is_seed_deterministic) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    double v1 = 0, v2 = 0, v3 = 0;
    CHECK(vm.doString("a = noise.fbm2(1234, 1.5, 2.5)"));
    CHECK(vm.doString("b = noise.fbm2(1234, 1.5, 2.5)"));   // same seed+coords
    CHECK(vm.doString("c = noise.fbm2(9999, 1.5, 2.5)"));   // different seed
    CHECK(vm.getGlobalNumber("a", v1));
    CHECK(vm.getGlobalNumber("b", v2));
    CHECK(vm.getGlobalNumber("c", v3));
    CHECK_APPROX(v1, v2, 1e-12);   // deterministic
    CHECK(std::fabs(v1 - v3) > 1e-9);  // seed actually matters
}

// --- the deepened surface: a whole generator written in Lua ---

TEST_CASE(procgen_script_builds_a_full_lsystem_tree) {
    // A complete grammar generator authored in Lua: define rules, expand the
    // axiom, then skin the turtle string into one welded SDF surface. This is
    // the "how deep can you go" proof — the same pipeline as the C++ generators.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> tree;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.create()
        sys:rule("X", "F[+X][-X]FX")
        sys:rule("F", "FF")
        local symbols = sys:expand("X", 3, 7)
        return lsystem.turtle_mesh_sdf(symbols,
            {length = 0.6, radius = 0.10, angle_deg = 28, leaf_radius = 0.0},
            0.08, 48)
    )LUA";
    CHECK(runProcgenMesh(vm, code, tree, &err));
    if (!tree) { CHECK(false); return; }
    CHECK(tree->vertices.size() > 100);     // a real welded canopy of branches
    CHECK(tree->indices.size() % 3 == 0);
}

TEST_CASE(procgen_script_skins_a_parametric_tree) {
    // The grammar-first path (ADR-0030): a script defines a *parametric* grammar
    // (successor params are expressions over the predecessor's), expands it, and
    // skins the modules into the real tree (curved generalized-cylinder bark +
    // alpha-cut leaf cards) via tree.skin — the same generator the C++ growTree
    // uses, now driven entirely from Lua.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> bark;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.parametric()
        sys:rule("A(l,w)",
          "F(l,w)/(137.5)[&(38)A(l*0.8,w*0.6)]/(137.5)[&(38)A(l*0.8,w*0.6)]/(137.5)A(l*0.85,w*0.7)")
        local modules = sys:expand("A(1.4,0.12)", 5, 7)
        local bark, leaves = tree.skin(modules,
            { ring_segments = 6, droop = 0.2, leaves_per_tip = 4,
              bark_color = {0.3, 0.2, 0.13}, leaf_color = {0.2, 0.45, 0.13} }, 7)
        assert(leaves ~= nil, "expected leaf cards")
        return bark
    )LUA";
    CHECK(runProcgenMesh(vm, code, bark, &err));
    if (!bark) { CHECK(false); return; }
    CHECK(bark->vertices.size() > 100);     // a swept canopy of branches
    CHECK(bark->indices.size() % 3 == 0);
}

TEST_CASE(procgen_model_returns_multiple_parts) {
    // Multi-part model (ADR-0032): a script returns a list of {mesh, material}
    // parts — here bark (opaque, bark texture) + leaves (alpha-cut, leaf texture)
    // from one tree. The loader turns each into its own InstanceGroup.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::vector<ScriptMeshPart> parts;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.parametric()
        sys:rule("A(l,w)", "F(l,w)/(137.5)[&(40)A(l*0.8,w*0.6)]/(137.5)A(l*0.85,w*0.7)")
        local modules = sys:expand("A(1.2,0.1)", 4, 5)
        local bark, leaves = tree.skin(modules, { droop = 0.2, leaves_per_tip = 3 }, 5)
        return {
            { mesh = bark,   texture = "bark" },
            { mesh = leaves, texture = "leaf", alpha_test = true, roughness = 0.6 },
        }
    )LUA";
    CHECK(runProcgenModel(vm, code, parts, &err));
    CHECK(parts.size() == 2u);
    if (parts.size() == 2) {
        CHECK(parts[0].texture == "bark");
        CHECK(parts[0].mesh && !parts[0].mesh->vertices.empty());
        CHECK(parts[0].hasMaterial);
        CHECK(parts[1].texture == "leaf");
        CHECK(parts[1].alphaTest);
        CHECK(parts[1].mesh && !parts[1].mesh->vertices.empty());
    }
}

TEST_CASE(procgen_model_single_mesh_is_one_default_part) {
    // Back-compat: a bare single-mesh return is one part with no material of its
    // own (the caller applies the species' default).
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::vector<ScriptMeshPart> parts;
    std::string err;
    CHECK(runProcgenModel(vm, "return mesh.box({1, 1, 1})", parts, &err));
    CHECK(parts.size() == 1u);
    if (!parts.empty()) CHECK(!parts[0].hasMaterial);
}

TEST_CASE(procgen_script_kitbashes_meshes) {
    // mesh primitives + transform + merge: two boxes welded into one buffer.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> combined;
    const char* code = R"LUA(
        local a = mesh.box({1, 1, 1})
        local b = mesh.translate(mesh.box({1, 1, 1}), {2, 0, 0})
        return mesh.merge({a, b})
    )LUA";
    CHECK(runProcgenMesh(vm, code, combined, nullptr));
    if (!combined) { CHECK(false); return; }

    RenderMesh oneBox = MeshBuilder::box(Vec3(1, 1, 1));
    CHECK(combined->vertices.size() == oneBox.vertices.size() * 2);
    CHECK(combined->indices.size() == oneBox.indices.size() * 2);
    // The second box was shifted +2 in x: some vertex must sit out there.
    bool shifted = false;
    for (const Vertex& v : combined->vertices)
        if (v.position.x > 1.5) shifted = true;
    CHECK(shifted);
}

TEST_CASE(procgen_script_builds_terrain) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> ground;
    const char* code = R"LUA(
        return terrain({size = 50, resolution = 16, height_scale = 5}, 99)
    )LUA";
    CHECK(runProcgenMesh(vm, code, ground, nullptr));
    if (!ground) { CHECK(false); return; }
    CHECK(ground->vertices.size() == 17 * 17);   // (resolution + 1)^2
    CHECK(ground->indices.size() % 3 == 0);
}

TEST_CASE(procgen_scatter_returns_frames) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // scatter yields a plain Lua array of {position, yaw, scale} — the Frame
    // value type. A script consumes it like any table.
    CHECK(vm.doString(R"LUA(
        local frames = scatter({region_size = 80, count = 200, seed = 3},
                               {size = 80, resolution = 32}, 99)
        n = #frames
        ok = (n > 0) and (frames[1].position ~= nil)
              and (type(frames[1].yaw) == "number")
    )LUA"));
    double n = 0;
    bool ok = false;
    CHECK(vm.getGlobalNumber("n", n));
    CHECK(n > 0);
    CHECK(vm.getGlobalBool("ok", ok));
    CHECK(ok);
}
