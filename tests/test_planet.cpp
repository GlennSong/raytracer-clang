#include "test_framework.h"

#include "../src/engine/procgen/planet.h"
#include "../src/engine/procgen/cellular.h"
#include "../src/engine/procgen/color_ramp.h"
#include "mesh_invariants.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <utility>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// Count edges of a triangle mesh that are NOT part of a closed, consistently-wound
// manifold: a directed edge seen more than once, or one whose reverse is absent.
// Zero => watertight with a single coherent winding (seams welded).
int manifoldDefects(const RenderMesh& m) {
    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uint32_t tri[3] = {m.indices[t], m.indices[t + 1], m.indices[t + 2]};
        for (int e = 0; e < 3; e++) directed[{tri[e], tri[(e + 1) % 3]}]++;
    }
    int defects = 0;
    for (const auto& kv : directed) {
        if (kv.second != 1) ++defects;
        if (directed.find({kv.first.second, kv.first.first}) == directed.end()) ++defects;
    }
    return defects;
}

bool sameMesh(const RenderMesh& a, const RenderMesh& b) {
    if (a.vertices.size() != b.vertices.size() || a.indices.size() != b.indices.size())
        return false;
    for (std::size_t i = 0; i < a.indices.size(); i++)
        if (a.indices[i] != b.indices[i]) return false;
    for (std::size_t i = 0; i < a.vertices.size(); i++)
        if ((a.vertices[i].position - b.vertices[i].position).lengthSquared() != 0.0)
            return false;
    return true;
}

}  // namespace

// --- Worley / cellular ------------------------------------------------------

TEST_CASE(worley_orders_f1_f2_and_is_deterministic) {
    for (int i = 0; i < 200; i++) {
        double x = i * 0.37, y = i * 0.11 - 3.0, z = i * -0.29 + 1.5;
        WorleyResult w = worley3(7, x, y, z);
        CHECK(w.f1 <= w.f2 + 1e-12);
        CHECK(w.f1 >= 0.0);
        CHECK(w.f1 < 2.0);   // a jittered-grid nearest feature is always close
        WorleyResult w2 = worley3(7, x, y, z);
        CHECK(w.f1 == w2.f1);
        CHECK(w.cellId == w2.cellId);
    }
}

TEST_CASE(worley_reseeds_to_a_different_field) {
    int differ = 0;
    for (int i = 0; i < 100; i++) {
        double x = i * 0.5, y = 2.0, z = -1.0;
        if (worley3(1, x, y, z).cellId != worley3(2, x, y, z).cellId) ++differ;
    }
    CHECK(differ > 50);   // most samples land in a differently-hashed cell
}

// --- Colour ramp ------------------------------------------------------------

TEST_CASE(color_ramp_hits_stops_interpolates_and_clamps) {
    ColorRamp r{{0.0, Vec3(0, 0, 0)}, {1.0, Vec3(1, 1, 1)}, {0.5, Vec3(1, 0, 0)}};
    // Sorted on construction, so the red stop is the middle one.
    CHECK((r.eval(0.0) - Vec3(0, 0, 0)).lengthSquared() < 1e-9);
    CHECK((r.eval(0.5) - Vec3(1, 0, 0)).lengthSquared() < 1e-9);
    CHECK((r.eval(1.0) - Vec3(1, 1, 1)).lengthSquared() < 1e-9);
    // Midway between stop 0 (black) and stop 0.5 (red) => half red.
    CHECK_APPROX(r.eval(0.25).x, 0.5, 1e-6);
    CHECK_APPROX(r.eval(0.25).y, 0.0, 1e-6);
    // Clamp beyond the ends.
    CHECK((r.eval(-3.0) - Vec3(0, 0, 0)).lengthSquared() < 1e-9);
    CHECK((r.eval(9.0) - Vec3(1, 1, 1)).lengthSquared() < 1e-9);
}

// --- Crater morphology ------------------------------------------------------

TEST_CASE(crater_profile_bowl_below_rim_above) {
    CHECK(craterProfile(0.0) < -0.5);          // deep bowl at the centre
    CHECK(craterProfile(1.0) > 0.1);           // raised rim at the radius
    CHECK(craterProfile(0.0) < craterProfile(1.0));
    CHECK(craterProfile(2.0) == 0.0);          // nothing past the ejecta reach
    CHECK_APPROX(craterProfile(5.0), 0.0, 1e-12);
}

// --- Height field -----------------------------------------------------------

TEST_CASE(planet_height_is_finite_bounded_and_deterministic) {
    PlanetParams p = planetMars();
    for (int i = 0; i < 500; i++) {
        double a = i * 0.257, b = i * 0.131;
        Vec3 dir = normalize(Vec3(std::cos(a) * std::cos(b), std::sin(b),
                                  std::sin(a) * std::cos(b)));
        double h = planetHeight(p, 1234, dir);
        CHECK(std::isfinite(h));
        CHECK(std::fabs(h) < 4.0);
        CHECK(h == planetHeight(p, 1234, dir));   // deterministic
    }
}

// --- Full surface mesh ------------------------------------------------------

TEST_CASE(planet_mesh_is_watertight_after_displacement) {
    PlanetParams p = planetMars();
    p.faceRes = 24;   // keep the test fast
    RenderMesh m = generatePlanet(p, 42);

    CHECK(mesh_invariants::triangleCount(m) == 6 * 24 * 24 * 2);
    CHECK(!mesh_invariants::hasNonFinite(m));
    CHECK(mesh_invariants::indicesInRange(m));
    CHECK(mesh_invariants::degenerateTriangles(m) == 0);
    CHECK(manifoldDefects(m) == 0);   // radial displacement preserved the manifold
}

TEST_CASE(planet_mesh_radii_stay_within_the_relief_band) {
    PlanetParams p = planetMars();
    p.faceRes = 20;
    RenderMesh m = generatePlanet(p, 7);
    const double R = p.radius;
    const double band = R * p.reliefFraction * 4.0;   // generous: |h| < 4
    for (const Vertex& v : m.vertices) {
        double r = v.position.length();
        CHECK(r > R - band);
        CHECK(r < R + band);
        // Vertex colour is a valid, in-range RGB.
        CHECK(v.color.x >= 0.0 && v.color.x <= 1.0);
        CHECK(v.color.y >= 0.0 && v.color.y <= 1.0);
        CHECK(v.color.z >= 0.0 && v.color.z <= 1.0);
    }
}

TEST_CASE(planet_generation_is_deterministic_and_seed_varies) {
    PlanetParams p = planetMoon();
    p.faceRes = 16;
    CHECK(sameMesh(generatePlanet(p, 5), generatePlanet(p, 5)));
    CHECK(!sameMesh(generatePlanet(p, 5), generatePlanet(p, 6)));
}

TEST_CASE(planet_polar_caps_colour_the_poles) {
    PlanetParams p = planetEarthlike();
    p.faceRes = 24;
    RenderMesh m = generatePlanet(p, 3);
    // The vertex nearest the +Y pole should read as cap (near-white), well above the
    // green/brown biome colours near the equator.
    const Vertex* pole = nullptr;
    double best = -1.0;
    for (const Vertex& v : m.vertices) {
        double y = normalize(v.position).y;
        if (y > best) { best = y; pole = &v; }
    }
    CHECK(pole != nullptr);
    if (pole) {
        CHECK(pole->color.x > 0.7);
        CHECK(pole->color.y > 0.7);
        CHECK(pole->color.z > 0.7);
    }
}

// --- Gas giant --------------------------------------------------------------

TEST_CASE(gas_giant_texture_is_well_formed_and_deterministic) {
    GasGiantParams p = gasGiantJupiter();
    p.texWidth = 128;
    p.texHeight = 64;
    TextureData t = generateGasGiantTexture(p, 5);
    CHECK(t.width == 128);
    CHECK(t.height == 64);
    CHECK(t.channels == 3);
    CHECK(t.pixels.size() == static_cast<size_t>(128) * 64 * 3);

    // Not a flat fill — the bands give real variation.
    uint8_t lo = 255, hi = 0;
    for (uint8_t px : t.pixels) { lo = std::min(lo, px); hi = std::max(hi, px); }
    CHECK(hi - lo > 40);

    // Deterministic in the seed.
    TextureData t2 = generateGasGiantTexture(p, 5);
    CHECK(t.pixels == t2.pixels);
    TextureData t3 = generateGasGiantTexture(p, 6);
    CHECK(t.pixels != t3.pixels);
}

TEST_CASE(gas_giant_texture_wraps_seamlessly_in_longitude) {
    GasGiantParams p = gasGiantJupiter();
    p.texWidth = 256;
    p.texHeight = 128;
    p.stormCount = 0;   // storms could straddle the seam; test the band field itself
    TextureData t = generateGasGiantTexture(p, 9);

    // The wrap gap between column 0 and column W-1 is exactly one texel width — the
    // same spacing as any interior neighbour — because the field is sampled from the
    // 3D direction. So the seam step must be no larger than a normal neighbour step:
    // seamlessness means "no discontinuity", not "identical pixels".
    auto colDiff = [&](int a, int b) {
        int mx = 0;
        for (int j = 0; j < t.height; j++) {
            size_t ia = (static_cast<size_t>(j) * t.width + a) * 3;
            size_t ib = (static_cast<size_t>(j) * t.width + b) * 3;
            for (int ch = 0; ch < 3; ch++)
                mx = std::max(mx, std::abs(static_cast<int>(t.pixels[ia + ch]) -
                                           static_cast<int>(t.pixels[ib + ch])));
        }
        return mx;
    };
    int seamDiff = colDiff(0, t.width - 1);
    int interiorMax = 0;
    for (int i = 0; i < t.width - 1; i++) interiorMax = std::max(interiorMax, colDiff(i, i + 1));
    CHECK(seamDiff <= interiorMax + 2);   // the seam is just another neighbour step
}

TEST_CASE(gas_giant_texture_is_banded_by_latitude) {
    GasGiantParams p = gasGiantNeptune();
    p.texWidth = 128;
    p.texHeight = 128;
    p.stormCount = 0;
    TextureData t = generateGasGiantTexture(p, 3);
    // Average red per row; rows at different latitudes should differ (bands), so the
    // spread across rows is well above zero.
    double lo = 1e9, hi = -1e9;
    for (int j = 0; j < t.height; j++) {
        double sum = 0;
        for (int i = 0; i < t.width; i++)
            sum += t.pixels[(static_cast<size_t>(j) * t.width + i) * 3];
        double avg = sum / t.width;
        lo = std::min(lo, avg);
        hi = std::max(hi, avg);
    }
    CHECK(hi - lo > 20.0);   // clear latitudinal banding
}

TEST_CASE(gas_giant_mesh_is_a_watertight_smooth_sphere) {
    GasGiantParams p = gasGiantJupiter();
    p.faceRes = 20;
    RenderMesh m = generateGasGiantMesh(p);
    CHECK(mesh_invariants::triangleCount(m) == 6 * 20 * 20 * 2);
    CHECK(manifoldDefects(m) == 0);
    // Smooth: every vertex is exactly on the sphere (no displacement).
    for (const Vertex& v : m.vertices) CHECK_APPROX(v.position.length(), p.radius, 1e-2);
}

TEST_CASE(ocean_shell_only_when_enabled_and_sits_near_sea_radius) {
    PlanetParams mars = planetMars();
    CHECK(generateOceanShell(mars, 1).vertices.empty());   // Mars: no ocean

    PlanetParams earth = planetEarthlike();
    earth.faceRes = 24;
    RenderMesh ocean = generateOceanShell(earth, 1);
    CHECK(!ocean.vertices.empty());
    double amp = earth.radius * earth.reliefFraction;
    double seaR = earth.radius + amp * earth.seaLevel;
    for (const Vertex& v : ocean.vertices)
        CHECK_APPROX(v.position.length(), seaR, 1.0);
    CHECK(manifoldDefects(ocean) == 0);
}
