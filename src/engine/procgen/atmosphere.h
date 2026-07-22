#ifndef RAYTRACER_ENGINE_PROCGEN_ATMOSPHERE_H
#define RAYTRACER_ENGINE_PROCGEN_ATMOSPHERE_H

#include "../../rt_math.h"   // Vec3

namespace engine {

// Physically-based atmospheric scattering (procedural-planet-plan P3). A CPU
// reference of the single-scattering Rayleigh + Mie model (Nishita/O'Neil core):
// the same maths the realtime shader will port, but here it is pure and testable,
// and drives the headless planet preview so the blue limb / reddened terminator can
// be seen without a GPU. Multiple scattering (Hillaire) is the device-side upgrade;
// single scattering already gives a blue sky, an atmosphere halo, and sunset colours.
//
// Distances are in the same world units as the planet (the preview uses radius 1).
struct AtmosphereParams {
    double planetRadius = 1.0;
    double atmosphereRadius = 1.025;      // top of the atmosphere shell

    // Density falls off exponentially with altitude at these scale heights.
    double rayleighScaleHeight = 0.006;
    double mieScaleHeight = 0.0018;

    // Scattering coefficients (per length). Rayleigh is per-channel and ∝ λ⁻⁴, so
    // blue scatters far more than red — the reason the sky is blue and sunsets red.
    Vec3   rayleighCoeff{20.0, 47.0, 115.0};   // ~(5.8,13.5,33.1) scaled for unit radius
    double mieCoeff = 42.0;
    double mieG = 0.76;                   // Mie asymmetry (forward-scattering)

    Vec3   sunColor{1.0, 1.0, 1.0};
    double sunIntensity = 22.0;

    int viewSamples = 24;                 // raymarch steps along the view ray
    int lightSamples = 8;                 // steps along the ray toward the sun
};

// Earth-like blue atmosphere; thin, dusty (brownish, high Mie) Mars atmosphere.
AtmosphereParams atmosphereEarth(double planetRadius = 1.0);
AtmosphereParams atmosphereMars(double planetRadius = 1.0);

// Entry/exit parameters (t0 < t1) where the ray `origin + t·dir` intersects the
// sphere of `radius` centred at the origin. Returns false if it misses (or is fully
// behind the origin). t0 may be negative when `origin` is inside the sphere.
bool raySphere(const Vec3& origin, const Vec3& dir, double radius, double& t0, double& t1);

// Transmittance (per channel, in (0,1]) of the atmosphere between two points along a
// straight segment — exp(−optical depth) for combined Rayleigh + Mie extinction.
Vec3 transmittance(const AtmosphereParams& p, const Vec3& a, const Vec3& b);

// The result of marching a view ray through the atmosphere: the in-scattered light
// added along the ray, and the transmittance of the whole marched segment (so the
// caller composites `background · transmittance + inScatter`).
struct ScatterResult {
    Vec3 inScatter{0, 0, 0};
    Vec3 transmittance{1, 1, 1};
};

// Single-scattering raymarch of `origin + t·dir` for t in [0, tMax] (clip tMax to
// the ground hit, or pass a large value for a ray that exits to space). `sunDir`
// points toward the sun. Samples in shadow of the planet contribute no in-scatter.
ScatterResult atmosphereScatter(const AtmosphereParams& p, const Vec3& origin,
                                const Vec3& dir, const Vec3& sunDir, double tMax);

}  // namespace engine

#endif
