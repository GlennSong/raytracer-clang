#ifndef RAYTRACER_ENGINE_SKY_SKY_ATMOSPHERE_H
#define RAYTRACER_ENGINE_SKY_SKY_ATMOSPHERE_H

// THE PHYSICAL SKY (ADR-0367; renderer-realism phase 2). Hillaire's atmosphere ("A Scalable and Production Ready Sky
// and Atmosphere Rendering Technique", 2020): Rayleigh, Mie and ozone over a spherical planet, a transmittance table,
// a multiple-scattering table, and the sky's radiance marched from them. The CPU model here is the reference: it gives
// the sun's colour and strength at the ground (through the air between), the sky colours the engine's other readers
// take (clouds, underwater, the cycle), and the two tables the GPU's sky-view pass samples -- so both sides are one
// model. Radiance is per unit of illuminance arriving at the top of the atmosphere; times the sun's (or the moon's),
// in engine light units (ADR-0366: one unit = kLuxPerUnit lux, or that many cd/m^2).

#include "../../rt_math.h"
#include <vector>

namespace engine {

struct SkyScatteringParams;

// ENGINE LIGHT UNITS (ADR-0366): an illuminance of one unit is this many lux, a luminance of one unit this many cd/m^2.
// Chosen so the cycle's noon sun (5 units) is 100,000 lux -- clear-day sunlight at the ground -- and a level's typical
// exposure of 0.7 is EV100 14.5, a camera in daylight.
constexpr double kLuxPerUnit = 20000.0;
// The sun's illuminance above the atmosphere (the solar constant in lux), and the full moon's (it is lit by the sun:
// its albedo ~0.12, so about 2.5e-6 of the sun's): times the transmittance on the way down.
constexpr double kSunTopLux = 128000.0;
constexpr double kFullMoonTopLux = 0.32;

struct Atmosphere {
    double groundRadius = 6360000.0, topRadius = 6460000.0;   // m
    Vec3 rayleighScatter{5.802e-6, 13.558e-6, 33.1e-6};       // /m at sea level
    double rayleighHeight = 8000.0;
    double mieScatter = 3.996e-6, mieExtinction = 4.40e-6;    // /m
    double mieHeight = 1200.0, mieG = 0.8;
    Vec3 ozoneAbsorb{0.650e-6, 1.881e-6, 0.085e-6};          // /m at the layer's peak (25 km, 15 km either side)
    Vec3 groundAlbedo{0.3, 0.3, 0.3};
    double multiScatter = 1.0;   // scales the multiple-scattering term (1: the model's own)
};
// The level's (SkyScatteringParams: turbidity scales the Mie terms).
Atmosphere atmosphereFrom(const SkyScatteringParams& p);

// The precomputed tables, CPU-side and uploaded as they are: transmittance (Bruneton's r, mu mapping) and multiple
// scattering (Hillaire's: u the sun's cosine, v the height). RGB, row-major, row 0 first.
struct AtmosphereTables {
    static constexpr int kTransW = 256, kTransH = 64, kMsW = 32, kMsH = 32;
    std::vector<float> transmittance;   // kTransW * kTransH * 3
    std::vector<float> multiScatter;    // kMsW * kMsH * 3
    Atmosphere atm;
};
void buildAtmosphereTables(const Atmosphere& atm, AtmosphereTables& out);

// Transmittance from a height (m above the ground) along a direction with cosine-zenith `mu` to the top of the
// atmosphere; zero when the ray meets the ground (the planet's shadow). Exact (marched), not the table.
Vec3 atmosphereTransmittance(const Atmosphere& atm, double height, double mu);
// ...and read from the tables (bilinear), as the GPU reads it.
Vec3 sampleTransmittance(const AtmosphereTables& t, double r, double mu);
Vec3 sampleMultiScatter(const AtmosphereTables& t, double r, double muS);

// The sky's radiance seen from `height` toward `view` (unit, world: +Y up), lit from `light` (unit, toward the sun or
// moon): single scattering plus the multiple-scattering term, and the ground's bounce for rays that meet it. Per unit
// of illuminance at the top of the atmosphere.
Vec3 atmosphereSkyRadiance(const AtmosphereTables& t, double height, const Vec3& view, const Vec3& light, int steps = 32);
// The illuminance the whole sky gives a surface facing `normal` (cosine-weighted, `samples` directions), per unit
// illuminance at the top.
Vec3 atmosphereSkyIrradiance(const AtmosphereTables& t, double height, const Vec3& normal, const Vec3& light, int samples = 64);

// THE SKY AS THE ENGINE'S OTHER READERS WANT IT, in engine units for a light of `topUnits` at the top: the light's own
// colour and strength at the camera (transmittance), and the dome's zenith, horizon (the average round it), ground and
// up-facing irradiance.
struct SkyReading {
    Vec3 lightColor{1, 1, 1};   // normalised (max channel 1)
    double lightIntensity = 0;  // units at the camera
    Vec3 zenith, horizon, ground;
    Vec3 irradianceUp;          // units
};
SkyReading readSky(const AtmosphereTables& t, double height, const Vec3& light, double topUnits);

}  // namespace engine

#endif
