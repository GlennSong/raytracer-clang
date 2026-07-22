#include "atmosphere.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Density (relative to sea level) at world point `q` for a given scale height.
inline double densityAt(const Vec3& q, double planetRadius, double scaleHeight) {
    double h = q.length() - planetRadius;
    if (h < 0.0) h = 0.0;
    return std::exp(-h / scaleHeight);
}

// Optical depth (Rayleigh, Mie) along the segment a→b, by trapezoid-ish midpoint
// sampling. Returned as {odRayleigh, odMie} — the ∫ density ds each extinction term
// integrates against its coefficient.
void opticalDepth(const AtmosphereParams& p, const Vec3& a, const Vec3& b, int steps,
                  double& odR, double& odM) {
    odR = 0.0;
    odM = 0.0;
    steps = std::max(1, steps);
    Vec3 seg = b - a;
    double len = seg.length();
    if (len < 1e-12) return;
    Vec3 dir = seg / len;
    double ds = len / steps;
    for (int i = 0; i < steps; i++) {
        Vec3 q = a + dir * (ds * (i + 0.5));
        odR += densityAt(q, p.planetRadius, p.rayleighScaleHeight) * ds;
        odM += densityAt(q, p.planetRadius, p.mieScaleHeight) * ds;
    }
}

}  // namespace

bool raySphere(const Vec3& origin, const Vec3& dir, double radius, double& t0, double& t1) {
    // |origin + t·dir|² = radius²  →  t² + 2b t + c = 0 with dir assumed normalized.
    double b = dot(origin, dir);
    double c = dot(origin, origin) - radius * radius;
    double disc = b * b - c;
    if (disc < 0.0) return false;
    double s = std::sqrt(disc);
    t0 = -b - s;
    t1 = -b + s;
    if (t1 < 0.0) return false;   // sphere entirely behind the ray
    return true;
}

Vec3 transmittance(const AtmosphereParams& p, const Vec3& a, const Vec3& b) {
    double odR, odM;
    opticalDepth(p, a, b, p.lightSamples, odR, odM);
    // Mie extinction ≈ scattering here (no separate absorption term in this reference).
    Vec3 tau = p.rayleighCoeff * odR + Vec3(1, 1, 1) * (p.mieCoeff * odM);
    return Vec3(std::exp(-tau.x), std::exp(-tau.y), std::exp(-tau.z));
}

ScatterResult atmosphereScatter(const AtmosphereParams& p, const Vec3& origin,
                                const Vec3& dir, const Vec3& sunDir, double tMax) {
    ScatterResult out;

    double a0, a1;
    if (!raySphere(origin, dir, p.atmosphereRadius, a0, a1)) return out;  // misses the shell
    double tEnter = std::max(0.0, a0);
    double tExit = std::min(tMax, a1);
    if (tExit <= tEnter) return out;

    const int steps = std::max(2, p.viewSamples);
    const double ds = (tExit - tEnter) / steps;

    // Rayleigh + Mie phase functions (θ = angle between view and sun directions).
    double mu = dot(dir, sunDir);
    double phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    double g = p.mieG;
    double phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                    ((2.0 + g * g) * std::pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    // Accumulate view-ray optical depth as we march (transmittance camera→sample).
    double odViewR = 0.0, odViewM = 0.0;
    Vec3 inscatR(0, 0, 0), inscatM(0, 0, 0);

    for (int i = 0; i < steps; i++) {
        double t = tEnter + ds * (i + 0.5);
        Vec3 q = origin + dir * t;

        double dR = densityAt(q, p.planetRadius, p.rayleighScaleHeight) * ds;
        double dM = densityAt(q, p.planetRadius, p.mieScaleHeight) * ds;
        odViewR += dR;
        odViewM += dM;

        // Light ray from the sample toward the sun. If it hits the planet, the sample
        // is in the planet's shadow — no direct sunlight to in-scatter.
        double g0, g1;
        bool shadowed = raySphere(q, sunDir, p.planetRadius, g0, g1) && g1 > 0.0 && g0 > 0.0;
        if (shadowed) continue;

        double la0, la1;
        raySphere(q, sunDir, p.atmosphereRadius, la0, la1);   // always hits from inside
        Vec3 lightExit = q + sunDir * std::max(0.0, la1);
        double odLightR, odLightM;
        opticalDepth(p, q, lightExit, p.lightSamples, odLightR, odLightM);

        // Combined transmittance: camera→sample (accumulated) + sample→sun.
        Vec3 tau = p.rayleighCoeff * (odViewR + odLightR) +
                   Vec3(1, 1, 1) * (p.mieCoeff * (odViewM + odLightM));
        Vec3 tr(std::exp(-tau.x), std::exp(-tau.y), std::exp(-tau.z));

        inscatR += tr * dR;
        inscatM += tr * dM;
    }

    Vec3 rayleigh = (p.rayleighCoeff * inscatR) * phaseR;   // Vec3*Vec3 is componentwise
    Vec3 mie = inscatM * (p.mieCoeff * phaseM);
    out.inScatter = (p.sunColor * (rayleigh + mie)) * p.sunIntensity;

    Vec3 tauView = p.rayleighCoeff * odViewR + Vec3(1, 1, 1) * (p.mieCoeff * odViewM);
    out.transmittance = Vec3(std::exp(-tauView.x), std::exp(-tauView.y), std::exp(-tauView.z));
    return out;
}

AtmosphereParams atmosphereEarth(double planetRadius) {
    AtmosphereParams p;
    p.planetRadius = planetRadius;
    p.atmosphereRadius = planetRadius * 1.025;
    double T = p.atmosphereRadius - p.planetRadius;
    p.rayleighScaleHeight = T * 0.25;
    p.mieScaleHeight = T * 0.07;
    // Coefficients ∝ λ⁻⁴; divided by planetRadius so the optical depth (β·scaleHeight,
    // and scaleHeight ∝ radius) is scale-invariant. Tuned for a vertical blue OD ≈ 0.7.
    p.rayleighCoeff = Vec3(20.0, 47.0, 115.0) / p.planetRadius;
    p.mieCoeff = 42.0 / p.planetRadius;
    p.mieG = 0.76;
    p.sunColor = Vec3(1.0, 1.0, 1.0);
    p.sunIntensity = 22.0;
    return p;
}

AtmosphereParams atmosphereMars(double planetRadius) {
    AtmosphereParams p;
    p.planetRadius = planetRadius;
    p.atmosphereRadius = planetRadius * 1.012;        // thin
    double T = p.atmosphereRadius - p.planetRadius;
    p.rayleighScaleHeight = T * 0.30;
    p.mieScaleHeight = T * 0.12;
    // Dusty: weak Rayleigh, strong forward Mie, warm/brown tint from the coefficients.
    p.rayleighCoeff = Vec3(10.0, 7.0, 5.0) / p.planetRadius;   // reddish (dust), not blue
    p.mieCoeff = 70.0 / p.planetRadius;
    p.mieG = 0.6;
    p.sunColor = Vec3(1.0, 0.92, 0.82);
    p.sunIntensity = 14.0;
    return p;
}

}  // namespace engine
