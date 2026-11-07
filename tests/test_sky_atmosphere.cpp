// THE PHYSICAL SKY (ADR-0367): the CPU atmosphere is the reference the GPU's sky follows, so it is held to what is
// known about Earth's: the air's transmittance straight up (a closed form), the tables against the exact march, a
// blue noon sky of a few thousand cd/m^2 with ~100,000 lux of sun at the ground, a reddened sun and horizon at
// sunset, multiple scattering lighting the twilight, and a black sky once the sun is well down.

#include "test_framework.h"
#include "../src/engine/sky/sky_atmosphere.h"
#include <cmath>

using namespace engine;

namespace {
const AtmosphereTables& tables() {
    static const AtmosphereTables t = [] {
        AtmosphereTables out;
        buildAtmosphereTables(Atmosphere{}, out);
        return out;
    }();
    return t;
}
Vec3 sunAt(double elevationDeg) {
    const double e = elevationDeg * 3.14159265358979 / 180.0;
    return Vec3(std::cos(e), std::sin(e), 0.0);
}
double nits(const Vec3& radianceUnits) { return (0.2126 * radianceUnits.x + 0.7152 * radianceUnits.y + 0.0722 * radianceUnits.z) * kLuxPerUnit; }
}  // namespace

TEST_CASE(sky_transmittance_straight_up_is_the_closed_form) {
    const Atmosphere a;
    // optical depth straight up: beta * scale height for each exponential layer, ozone's tent integrates to 15 km
    const double odR = 5.802e-6 * 8000 + 4.40e-6 * 1200 + 0.650e-6 * 15000;
    const double odB = 33.1e-6 * 8000 + 4.40e-6 * 1200 + 0.085e-6 * 15000;
    const Vec3 t = atmosphereTransmittance(a, 0.0, 1.0);
    CHECK(std::fabs(t.x - std::exp(-odR)) < 0.01);
    CHECK(std::fabs(t.z - std::exp(-odB)) < 0.01);
    CHECK(t.z < t.y && t.y < t.x);   // blue is lost first
    CHECK(atmosphereTransmittance(a, 10.0, -0.5).x == 0.0);   // into the ground: none
}

TEST_CASE(sky_tables_agree_with_the_exact_march) {
    const AtmosphereTables& t = tables();
    const double rg = t.atm.groundRadius;
    for (double h : {0.0, 500.0, 3000.0, 20000.0})
        for (double mu : {1.0, 0.5, 0.15, 0.03}) {
            const Vec3 exact = atmosphereTransmittance(t.atm, h, mu), table = sampleTransmittance(t, rg + h, mu);
            CHECK(std::fabs(exact.x - table.x) < 0.02 && std::fabs(exact.z - table.z) < 0.03);
        }
}

TEST_CASE(sky_noon_is_blue_and_daylight_bright) {
    const AtmosphereTables& t = tables();
    const double top = kSunTopLux / kLuxPerUnit;
    const SkyReading s = readSky(t, 2.0, sunAt(60), top);
    const double sunLux = s.lightIntensity * kLuxPerUnit * std::sin(60.0 * 3.14159265358979 / 180.0);
    std::printf("    noon: sun %.0f lux on the ground, zenith %.0f cd/m2 (%.3f %.3f %.3f), sky on the ground %.0f lux\n",
                sunLux, nits(s.zenith), s.zenith.x, s.zenith.y, s.zenith.z, s.irradianceUp.y * kLuxPerUnit);
    CHECK(sunLux > 80000 && sunLux < 120000);
    CHECK(nits(s.zenith) > 1500 && nits(s.zenith) < 12000);
    CHECK(s.zenith.z > s.zenith.y && s.zenith.y > s.zenith.x);
    CHECK(s.irradianceUp.y * kLuxPerUnit > 8000 && s.irradianceUp.y * kLuxPerUnit < 40000);
    CHECK(s.lightColor.z > 0.75);   // a high sun is near white
}

TEST_CASE(sky_sunset_reddens_the_sun_and_the_horizon_toward_it) {
    const AtmosphereTables& t = tables();
    const Vec3 sun = sunAt(2);
    const SkyReading s = readSky(t, 2.0, sun, kSunTopLux / kLuxPerUnit);
    CHECK(s.lightColor.x == 1.0 && s.lightColor.z < 0.5);   // the sun itself goes orange-red
    const Vec3 toward = atmosphereSkyRadiance(t, 2.0, normalize(Vec3(1.0, 0.05, 0.0)), sun);
    // (low on the far side it is pink -- the belt of Venus, sunlit air seen along the horizon -- but above it, blue)
    const Vec3 away = atmosphereSkyRadiance(t, 2.0, normalize(Vec3(-1.0, 0.6, 0.0)), sun);
    std::printf("    sunset: horizon toward the sun %.4f %.4f %.4f, away %.4f %.4f %.4f\n", toward.x, toward.y, toward.z, away.x,
                away.y, away.z);
    CHECK(toward.x > toward.z);   // warm toward the sun
    CHECK(away.z > away.x);       // still blue up the other way
}

TEST_CASE(sky_twilight_glows_and_night_is_dark) {
    const AtmosphereTables& t = tables();
    const double top = kSunTopLux / kLuxPerUnit;
    const double noon = nits(atmosphereSkyRadiance(t, 2.0, Vec3(0, 1, 0), sunAt(60)) * top);
    const double civil = nits(atmosphereSkyRadiance(t, 2.0, Vec3(0, 1, 0), sunAt(-4)) * top);
    const double night = nits(atmosphereSkyRadiance(t, 2.0, Vec3(0, 1, 0), sunAt(-20)) * top);
    std::printf("    zenith: noon %.0f, sun -4 deg %.2f, sun -20 deg %.6f cd/m2\n", noon, civil, night);
    CHECK(civil > 0.5 && civil < noon * 0.05);   // civil twilight: still a lit sky, well under day
    CHECK(night < 1e-3);                         // astronomical night: the sun lights nothing
    const Vec3 ms = sampleMultiScatter(t, t.atm.groundRadius + 100.0, 0.5);
    CHECK(ms.z > ms.x && ms.x > 0.0);   // the multiple scattering is real and blue
}
