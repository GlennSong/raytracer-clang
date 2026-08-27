#include "test_framework.h"

#include "../src/scene.h"
#include "../src/level_scene.h"
#include "../src/engine/day_night_cycle.h"

#include <cmath>
#include <cstdio>

using namespace engine;

// THE TRACER'S SKY (device: "what about the ray tracer?"): the offline miss
// shader at an hour of the cycle carries what the viewer's sky carries —
// the moon disc where the moon is, the star field at night and not by day,
// the city's glow toward downtown — and applyDayNight lights the level from
// the same moment.

namespace {
// Deterministic directions on the upper hemisphere.
Vec3 hemiDir(int i) {
    uint32_t h = static_cast<uint32_t>(i) * 0x9e3779b1u;
    h ^= h >> 15; h *= 0x85ebca6bu; h ^= h >> 13;
    const double u = (h & 0xffffu) / 65535.0, v = ((h >> 16) & 0xffffu) / 65535.0;
    const double z = 0.05 + 0.95 * u;             // altitude sin, away from the horizon
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double a = 6.283185307179586 * v;
    return Vec3(r * std::cos(a), z, r * std::sin(a));
}
// A STAR is a point that stands out from the smooth sky beside it: the
// radiance minus the radiance 0.1 deg away. (A plain brightness threshold
// counts the night palette itself — the first cut found 10837 "stars".)
int starPoints(const EnvironmentLight& env, double contrast, const Vec3* avoid, int n = 20000) {
    int stars = 0;
    for (int i = 0; i < n; ++i) {
        const Vec3 d = hemiDir(i);
        if (avoid && dot(d, *avoid) > 0.95) continue;   // not the moon / sun glow
        const Vec3 r = env.radiance(d);
        const Vec3 r2 = env.radiance(normalize(d + Vec3(0.002, 0.0, 0.0)));
        if ((r - r2).length() > contrast) ++stars;
    }
    return stars;
}
}

TEST_CASE(tracer_sky_at_night_carries_the_moon_and_the_stars) {
    EnvironmentLight env;
    env.enabled = true;
    env.procedural = true;
    DayNightCycle c;   // 40 N, day 172
    c.timeOfDay = 0.0;
    c.moonLock = kSynodicMonthDays * 0.5;   // full moon, up at midnight
    env.night = c.evaluate();
    const Vec3 m = env.night.moonDirection;
    CHECK(m.y > 0.0);
    CHECK(env.radiance(m).length() > 1.0);                       // the disc
    const Vec3 side = normalize(m + Vec3(0.16, 0.0, 0.0));       // ~9 deg off: past the glow lobe
    CHECK(env.radiance(side).length() < 0.25);                    // dark sky beside it
    // Stars: a new-moon midnight; a few hundred of 20k directions land on one.
    c.moonLock = 0.0;
    env.night = c.evaluate();
    const int night = starPoints(env, 0.2, nullptr);
    // Noon: none beyond the sky itself (away from the sun's glow).
    c.timeOfDay = 0.5;
    env.night = c.evaluate();
    const Vec3 sun = env.night.sunDirection;
    int day = 0;
    for (int i = 0; i < 20000; ++i) {
        const Vec3 d = hemiDir(i);
        if (dot(d, sun) > 0.95) continue;
        const Vec3 r = env.radiance(d);
        // The day sky is smooth: a "bright point" is one that stands out
        // from its neighbour by more than the palette ever does.
        const Vec3 r2 = env.radiance(normalize(d + Vec3(0.002, 0.0, 0.0)));
        if ((r - r2).length() > 0.2) ++day;
    }
    std::printf("    [tracer sky] bright points: night %d / 20000, noon %d\n", night, day);
    CHECK(night > 30);
    CHECK(night < 2000);
    CHECK(day == 0);
}

TEST_CASE(tracer_sky_light_pollution_glows_toward_the_city_and_hides_faint_stars) {
    EnvironmentLight env;
    env.enabled = true;
    env.procedural = true;
    DayNightCycle c;
    c.timeOfDay = 0.0;
    c.moonLock = 0.0;
    env.night = c.evaluate();
    env.cityDirection = Vec3(0, 0, 1);
    // A low contrast so the faint stars count: pollution cuts that end.
    const int dark = starPoints(env, 0.02, nullptr);
    env.lightPollution = 0.9;
    const int city = starPoints(env, 0.02, nullptr);
    const Vec3 lowToward = normalize(Vec3(0.0, 0.05, 1.0)), lowAway = normalize(Vec3(0.0, 0.05, -1.0));
    std::printf("    [tracer sky] stars dark %d, city %d; horizon toward %.4f away %.4f\n", dark, city,
                env.radiance(lowToward).length(), env.radiance(lowAway).length());
    CHECK(city * 10 < dark * 7);                                         // the faint end is gone
    CHECK(env.radiance(lowToward).length() > env.radiance(lowAway).length() * 1.1);
}

TEST_CASE(tracer_day_night_replaces_the_authored_sun_with_the_moment) {
    Scene scene;
    SceneLight sun;
    sun.type = SceneLight::Type::Directional;
    sun.direction = Vec3(0.3, 0.9, 0.2);
    sun.intensity = 8.0;   // metro authors 8
    scene.lights.push_back(sun);
    SceneLight lamp;
    lamp.type = SceneLight::Type::Point;
    scene.lights.push_back(lamp);

    DayNightCycle c;
    c.timeOfDay = 0.5;   // noon: the sun, at the authored strength
    DayNightState st = applyDayNight(scene, c, 0.0);
    CHECK(scene.lights.size() == 2);   // the lamp kept, the authored sun replaced
    const SceneLight* dir = nullptr;
    for (const SceneLight& l : scene.lights) if (l.type == SceneLight::Type::Directional) dir = &l;
    CHECK(dir != nullptr);
    if (dir) {
        CHECK_APPROX(dir->intensity, 8.0 * (st.sunIntensity / kCycleNoonSunIntensity), 1e-9);
        CHECK(dir->intensity > 7.5);
        CHECK(dir->direction.y > 0.9);
    }
    CHECK(scene.environment.procedural);
    CHECK(scene.environment.enabled);
    // Midnight, full moon: the light is the moon — up, blue, a fraction of the sun.
    c.timeOfDay = 0.0;
    c.moonLock = kSynodicMonthDays * 0.5;
    st = applyDayNight(scene, c, 0.0);
    dir = nullptr;
    for (const SceneLight& l : scene.lights) if (l.type == SceneLight::Type::Directional) dir = &l;
    CHECK(dir != nullptr);
    if (dir) {
        CHECK(dir->direction.y > 0.0);
        CHECK(dir->color.z > dir->color.x);
        CHECK(dir->intensity < 0.5);
        CHECK(dir->intensity > 0.05);
    }
    // Midnight, new moon: no directional light at all — lamps carry the night.
    c.moonLock = 0.0;
    applyDayNight(scene, c, 0.0);
    int dirs = 0;
    for (const SceneLight& l : scene.lights) if (l.type == SceneLight::Type::Directional) ++dirs;
    CHECK(dirs == 0);
    CHECK(scene.lights.size() == 1);
}
