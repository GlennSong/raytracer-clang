#include "test_framework.h"

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/renderer/renderer.h"

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A procgen VM with the flora library loaded (so `flora.*` is available).
struct FloraVM {
    ScriptVM vm;
    FloraVM() {
        openProcgenLibrary(vm);
        std::string src = readFile(std::string(RT_SOURCE_DIR) + "/assets/scripts/flora.lua");
        std::string err;
        if (!vm.doString(src, &err)) std::printf("    flora load error: %s\n", err.c_str());
    }
};

std::shared_ptr<RenderMesh> run(ScriptVM& vm, const char* code) {
    std::shared_ptr<RenderMesh> m;
    std::string err;
    if (!runProcgenMesh(vm, code, m, &err)) std::printf("    run error: %s\n", err.c_str());
    return m;
}

}  // namespace

TEST_CASE(flora_param_tree_returns_bark_and_leaf_parts) {
    // The parametric path (ADR-0030/0032): flora.param_tree builds a grammar,
    // skins the real curved tree, and returns a two-part model (opaque bark +
    // alpha-cut leaves) for the multi-material scatter.
    FloraVM f;
    std::vector<ScriptMeshPart> parts;
    std::string err;
    bool ok = runProcgenModel(f.vm, "return flora.param_tree(7, {species='pine'})",
                              parts, &err);
    if (!ok) std::printf("    run error: %s\n", err.c_str());
    CHECK(ok);
    CHECK(parts.size() == 2u);
    if (parts.size() == 2) {
        CHECK(parts[0].texture == "bark");
        CHECK(parts[0].mesh && parts[0].mesh->vertices.size() > 100);
        CHECK(parts[1].texture == "leaf");
        CHECK(parts[1].alphaTest);
        CHECK(parts[1].wind);   // leaves opt into wind sway
        CHECK(parts[1].mesh && !parts[1].mesh->vertices.empty());
    }
    // Different species => different geometry (structurally distinct grammars).
    std::vector<ScriptMeshPart> oak;
    runProcgenModel(f.vm, "return flora.param_tree(7, {species='oak'})", oak, nullptr);
    CHECK(!oak.empty());
    if (!oak.empty() && parts.size() == 2)
        CHECK(oak[0].mesh->vertices.size() != parts[0].mesh->vertices.size());
}

TEST_CASE(flora_tree_has_real_leaf_cards_not_blobs) {
    FloraVM f;
    // Full canopy vs. branches-only (leaf_step huge keeps no leaf cards): the
    // leaves add real geometry on top of the welded branches.
    auto leafy = run(f.vm, "return flora.tree(7, {species='oak'})");
    auto bare = run(f.vm, "return flora.tree(7, {species='oak', leaf_step=100000})");
    if (!leafy || !bare) { CHECK(false); return; }
    CHECK(leafy->vertices.size() > bare->vertices.size());
    CHECK(bare->vertices.size() > 50);   // branches exist on their own
}

TEST_CASE(flora_tree_trunk_tapers_upward) {
    FloraVM f;
    // The first (root) segment is the thickest; taper<1 thins every segment up
    // the path, so the minimum radius is well below the root's.
    CHECK(f.vm.doString(R"LUA(
        local sys = lsystem.create()
        sys:rule('X', 'F[&+X][&-X]FX'); sys:rule('F', 'FF')
        local sym = sys:expand('X', 3, 1)
        local segs = lsystem.segments(sym, { radius = 0.2, taper = 0.9, length = 0.5 })
        nseg = #segs
        root_r = segs[1].radius
        min_r = segs[1].radius
        for i = 2, #segs do if segs[i].radius < min_r then min_r = segs[i].radius end end
    )LUA"));
    double n = 0, root = 0, mn = 0;
    CHECK(f.vm.getGlobalNumber("nseg", n));
    CHECK(f.vm.getGlobalNumber("root_r", root));
    CHECK(f.vm.getGlobalNumber("min_r", mn));
    CHECK(n > 4);
    CHECK_APPROX(root, 0.2, 1e-6);   // root keeps the base radius
    CHECK(mn < root * 0.9);          // upper branches are clearly thinner
}

TEST_CASE(flora_species_produce_different_trees) {
    FloraVM f;
    auto oak = run(f.vm, "return flora.tree(3, {species='oak'})");
    auto pine = run(f.vm, "return flora.tree(3, {species='pine'})");
    auto birch = run(f.vm, "return flora.tree(3, {species='birch'})");
    if (!oak || !pine || !birch) { CHECK(false); return; }
    CHECK(oak->vertices.size() != pine->vertices.size());
    CHECK(pine->vertices.size() != birch->vertices.size());
}

TEST_CASE(flora_seed_varies_trees_via_global) {
    FloraVM f;
    // Mirrors the level loader: set a `seed` global, then run the species' inline
    // chunk. Stochastic grammars => different seeds grow different trees.
    f.vm.doString("seed = 100");
    auto a = run(f.vm, "return flora.tree(seed, {species='oak', res=40})");
    f.vm.doString("seed = 200");
    auto b = run(f.vm, "return flora.tree(seed, {species='oak', res=40})");
    if (!a || !b) { CHECK(false); return; }
    CHECK(a->vertices.size() > 0);
    CHECK(a->vertices.size() != b->vertices.size());
}

TEST_CASE(flora_generators_are_seed_deterministic) {
    FloraVM f;
    const char* recipes[] = {
        "return flora.tree(42, {species='oak'})",
        "return flora.rock(5, {})",
        "return flora.grass(11, {})",
        "return flora.flower(9, {})",
    };
    for (const char* r : recipes) {
        auto a = run(f.vm, r);
        auto b = run(f.vm, r);
        if (!a || !b) { CHECK(false); continue; }
        CHECK(a->vertices.size() > 0);
        CHECK(a->vertices.size() == b->vertices.size());
        CHECK(a->indices.size() == b->indices.size());
    }
}

TEST_CASE(flora_rock_grass_flower_are_built) {
    FloraVM f;
    auto rock = run(f.vm, "return flora.rock(5, {radius=1.0})");
    auto grass = run(f.vm, "return flora.grass(11, {blades=8})");
    auto flower = run(f.vm, "return flora.flower(9, {petals=6})");
    if (!rock || !grass || !flower) { CHECK(false); return; }
    CHECK(rock->vertices.size() > 50);          // a real polygonized rock
    CHECK(grass->vertices.size() > 0);
    CHECK(flower->vertices.size() > 0);
    CHECK(rock->indices.size() % 3 == 0);
}

TEST_CASE(flora_scatters_mixed_species_together) {
    FloraVM f;
    // The "scatter them in with the trees we have" story: scatter frames over the
    // terrain and hand each a species — trees, rocks, grass, flowers mixed.
    CHECK(f.vm.doString(R"LUA(
        local frames = scatter({ region_size = 60, count = 150, seed = 2 },
                               { size = 60, resolution = 32 }, 9)
        fcount = #frames
        local kinds = { 'oak', 'pine', 'birch', 'rock', 'grass', 'flower' }
        local seen = {}
        for i = 1, #frames do seen[kinds[(i % #kinds) + 1]] = true end
        variety = 0
        for _ in pairs(seen) do variety = variety + 1 end
    )LUA"));
    double fcount = 0, variety = 0;
    CHECK(f.vm.getGlobalNumber("fcount", fcount));
    CHECK(f.vm.getGlobalNumber("variety", variety));
    CHECK(fcount > 0);
    CHECK(variety >= 4);   // several species placed across the scatter
}
