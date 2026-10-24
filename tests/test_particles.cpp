// PARTICLES (engine/particles.h, ADR-0239): the effects as data, the simulation, asking for them from anywhere.

#include "test_framework.h"

#include "../src/engine/particles.h"
#include "../src/engine/components.h"
#include "../src/engine/world.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../src/engine/scripting/particles_lua.h"
#endif

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace engine;

// Each built-in effect does what it says: dust spreads and settles, smoke and bubbles rise, sparks fall, and every
// one is gone by the end of its longest life.
TEST_CASE(particle_effects_move_as_named_and_die_on_time) {
    const ParticleBook book;
    auto run = [&](const char* name, Real seconds, Vec3& meanPos, std::size_t& alive) {
        ParticlePool pool(book, 7);
        const int fx = book.find(name);
        CHECK(fx >= 0);
        pool.burst(fx, Vec3(0, 10, 0), Vec3(1, 0, 0), 40);
        for (Real t = 0; t < seconds; t += 1.0 / 60.0) pool.step(1.0 / 60.0);
        meanPos = Vec3();
        for (const Particle& p : pool.particles()) meanPos = meanPos + p.pos;
        alive = pool.size();
        if (alive) meanPos = meanPos * (1.0 / static_cast<Real>(alive));
    };
    Vec3 m;
    std::size_t n = 0;
    run("smoke", 1.0, m, n);
    std::printf("    [smoke] 1 s: %zu alive, mean y %.2f\n", n, m.y);
    CHECK(n == 40 && m.y > 10.0);   // all still there, risen
    run("bubble", 1.0, m, n);
    std::printf("    [bubble] 1 s: mean y %.2f\n", m.y);
    CHECK(m.y > 10.4 && m.y < 11.2);   // rising at well under a metre a second (the water holds it back)
    run("spark", 0.15, m, n);
    std::printf("    [spark] 0.15 s: %zu alive, mean y %.2f x %.2f\n", n, m.y, m.x);
    CHECK(n == 40 && m.x > 0.2);   // thrown the way asked
    run("dust", 0.6, m, n);
    CHECK(n == 40 && std::fabs(m.y - 10.0) < 0.6);   // low: kicked up, settling
    for (std::size_t i = 0; i < book.size(); ++i) {
        ParticlePool pool(book, 3);
        pool.burst(static_cast<int>(i), Vec3(), Vec3(), 20);
        for (Real t = 0; t < book.at(static_cast<int>(i)).lifeMax + 0.05; t += 1.0 / 60.0) pool.step(1.0 / 60.0);
        CHECK(pool.size() == 0);
    }
}

// Faded in and out, never past the effect's alpha; a bubble ends at its ceiling (the water's surface).
TEST_CASE(particles_fade_within_their_alpha_and_bubbles_pop_at_the_surface) {
    const ParticleBook book;
    ParticlePool pool(book, 11);
    const int dust = book.find("dust"), bubble = book.find("bubble");
    pool.emit(dust, Vec3(), Vec3());
    Real most = 0, first = -1, last = 0;
    while (pool.size()) {
        const Real a = pool.drawnAlpha(pool.particles()[0]);
        if (first < 0) first = a;
        most = std::max(most, a);
        last = a;
        pool.step(1.0 / 60.0);
    }
    CHECK(first < 0.05 && last < 0.05);
    CHECK(most <= book.at(dust).alpha + 1e-9 && most > book.at(dust).alpha * 0.9);
    pool.emit(bubble, Vec3(0, -1.0, 0), Vec3(), 1, /*ceiling*/ -0.5);
    Real t = 0;
    while (pool.size() && t < 5) { pool.step(1.0 / 60.0); t += 1.0 / 60.0; }
    std::printf("    [bubble] 0.5 m to the surface in %.2f s\n", t);
    CHECK(t < book.at(bubble).lifeMin);   // popped at the surface, not old age
}

// Asked for from anywhere: requests queue on one entity; an emitter's rate is what it emits.
TEST_CASE(particles_are_requested_by_name_from_any_system) {
    World world;
    requestParticles(world, "spark", Vec3(1, 2, 3), Vec3(0, 1, 0));
    requestParticles(world, "dust", Vec3(), Vec3(), 5);
    int holders = 0;
    std::size_t queued = 0;
    world.each<ParticleRequests>([&](Entity, ParticleRequests& q) { ++holders; queued += q.queue.size(); });
    CHECK(holders == 1 && queued == 2);
}

#ifdef RT_ENABLE_SCRIPTING
// THE SHIPPED BOOK (assets/scripts/particles.lua) loads, and is what the built-ins say (the two kept alike); a typo
// in a field is an error, not a setting quietly lost; an entry naming a built-in edits only what it gives.
TEST_CASE(the_shipped_particle_book_loads_and_matches_the_builtins) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/particles.lua");
    std::stringstream ss;
    ss << in.rdbuf();
    ParticleBook fresh;
    ParticleBook book;
    std::string err;
    const bool ok = loadParticleBookSource(ss.str(), book, &err);
    if (!ok) std::printf("    [particles] %s\n", err.c_str());
    CHECK(ok);
    CHECK(book.size() == fresh.size());   // the file adds nothing the built-ins lack, and the other way round
    for (std::size_t i = 0; i < fresh.size(); ++i) {
        const ParticleEffect& a = fresh.at(static_cast<int>(i));
        const int j = book.find(a.name);
        CHECK(j >= 0);
        if (j < 0) continue;
        const ParticleEffect& b = book.at(j);
        const bool same = a.gravity == b.gravity && a.drag == b.drag && a.wobble == b.wobble &&
                          a.lifeMin == b.lifeMin && a.lifeMax == b.lifeMax && a.shape == b.shape &&
                          (a.colour - b.colour).length() < 1e-9 && (a.glow - b.glow).length() < 1e-9 &&
                          a.sizeStart == b.sizeStart && a.sizeEnd == b.sizeEnd && a.alpha == b.alpha &&
                          a.fadeIn == b.fadeIn && a.fadeOut == b.fadeOut && a.stretch == b.stretch &&
                          a.count == b.count && a.speed == b.speed && a.up == b.up && a.spread == b.spread &&
                          a.jitter == b.jitter;
        if (!same) std::printf("    [particles] %s differs between particles.lua and the built-ins\n", a.name.c_str());
        CHECK(same);
    }
    ParticleBook typo;
    CHECK(!loadParticleBookSource("particles = { dust = { gravty = 2 } }", typo, &err));
    std::printf("    [particles] a typo: %s\n", err.c_str());
    ParticleBook edit;
    CHECK(loadParticleBookSource("particles = { dust = { alpha = 0.9 }, embers = { life = {1, 2}, glow = {3, 1, 0} } }",
                                 edit, &err));
    const ParticleEffect& d = edit.at(edit.find("dust"));
    CHECK(d.alpha == 0.9 && d.gravity == fresh.at(fresh.find("dust")).gravity);
    CHECK(edit.find("embers") >= 0 && edit.size() == fresh.size() + 1);
}
#endif
