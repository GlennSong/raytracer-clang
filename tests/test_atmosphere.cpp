#include "test_framework.h"

#include "../src/engine/procgen/atmosphere.h"

#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(ray_sphere_hits_and_misses) {
    double t0, t1;
    // Ray from outside, straight at the origin sphere of radius 1.
    CHECK(raySphere(Vec3(0, 0, 5), normalize(Vec3(0, 0, -1)), 1.0, t0, t1));
    CHECK_APPROX(t0, 4.0, 1e-6);
    CHECK_APPROX(t1, 6.0, 1e-6);
    // A ray pointing away misses.
    CHECK(!raySphere(Vec3(0, 0, 5), normalize(Vec3(0, 0, 1)), 1.0, t0, t1));
    // A ray offset past the radius misses.
    CHECK(!raySphere(Vec3(0, 2, 5), normalize(Vec3(0, 0, -1)), 1.0, t0, t1));
    // From inside: t0 negative, t1 positive.
    CHECK(raySphere(Vec3(0, 0, 0), normalize(Vec3(0, 0, 1)), 1.0, t0, t1));
    CHECK(t0 < 0.0);
    CHECK(t1 > 0.0);
}

TEST_CASE(transmittance_is_in_unit_range_and_decreases_with_distance) {
    AtmosphereParams p = atmosphereEarth();
    Vec3 top = Vec3(0, p.atmosphereRadius, 0);
    Vec3 mid = Vec3(0, p.planetRadius + (p.atmosphereRadius - p.planetRadius) * 0.5, 0);
    Vec3 ground = Vec3(0, p.planetRadius, 0);

    Vec3 tShort = transmittance(p, top, mid);     // upper half
    Vec3 tLong = transmittance(p, top, ground);   // whole column (denser, longer)
    for (double c : {tShort.x, tShort.y, tShort.z}) { CHECK(c > 0.0); CHECK(c <= 1.0); }
    // A longer, denser path transmits less on every channel.
    CHECK(tLong.x < tShort.x);
    CHECK(tLong.y < tShort.y);
    CHECK(tLong.z < tShort.z);
}

TEST_CASE(rayleigh_scatters_blue_more_than_red) {
    AtmosphereParams p = atmosphereEarth();
    // Blue extinction coefficient exceeds red, so blue transmits less along a column.
    CHECK(p.rayleighCoeff.z > p.rayleighCoeff.x);
    Vec3 t = transmittance(p, Vec3(0, p.atmosphereRadius, 0), Vec3(0, p.planetRadius, 0));
    CHECK(t.z < t.x);   // blue attenuated more than red
}

TEST_CASE(atmosphere_inscatter_is_finite_nonnegative_and_blue_looking_up) {
    AtmosphereParams p = atmosphereEarth();
    Vec3 sun = normalize(Vec3(0.3, 1.0, 0.2));
    // A viewer just above the ground looking straight up sees a blue sky: the
    // in-scattered radiance is positive on all channels and bluest.
    Vec3 eye(0, p.planetRadius + 1e-4, 0);
    ScatterResult s = atmosphereScatter(p, eye, Vec3(0, 1, 0), sun, 1e9);
    CHECK(std::isfinite(s.inScatter.x));
    CHECK(std::isfinite(s.inScatter.y));
    CHECK(std::isfinite(s.inScatter.z));
    CHECK(s.inScatter.x >= 0.0);
    CHECK(s.inScatter.y >= 0.0);
    CHECK(s.inScatter.z >= 0.0);
    CHECK(s.inScatter.z > s.inScatter.x);   // sky is blue
    // Transmittance stays in range.
    CHECK(s.transmittance.x > 0.0 && s.transmittance.x <= 1.0);
}

TEST_CASE(atmosphere_shadowed_night_side_barely_scatters) {
    AtmosphereParams p = atmosphereEarth();
    Vec3 sun = normalize(Vec3(0, 1, 0));
    // Look up from the far (anti-sun) side of the planet: the light rays into the
    // atmosphere are blocked by the planet, so in-scatter is far dimmer than the
    // sun-lit side.
    Vec3 dayEye(0, p.planetRadius + 1e-4, 0);
    Vec3 nightEye(0, -(p.planetRadius + 1e-4), 0);
    double day = atmosphereScatter(p, dayEye, Vec3(0, 1, 0), sun, 1e9).inScatter.length();
    double night = atmosphereScatter(p, nightEye, Vec3(0, -1, 0), sun, 1e9).inScatter.length();
    CHECK(night < day * 0.25);
}

TEST_CASE(atmosphere_presets_differ_earth_blue_mars_reddish) {
    AtmosphereParams e = atmosphereEarth();
    AtmosphereParams m = atmosphereMars();
    // Earth's dominant scattering channel is blue; Mars's dust makes red dominant.
    CHECK(e.rayleighCoeff.z > e.rayleighCoeff.x);
    CHECK(m.rayleighCoeff.x > m.rayleighCoeff.z);
    // Mars's atmosphere shell is thinner.
    CHECK((m.atmosphereRadius - m.planetRadius) < (e.atmosphereRadius - e.planetRadius));
}
