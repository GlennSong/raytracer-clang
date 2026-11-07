#include "sky_atmosphere.h"
#include "../../renderer/renderer.h"   // SkyScatteringParams

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp01(double v) { return std::min(1.0, std::max(0.0, v)); }

double ozoneDensity(double h) { return std::max(0.0, 1.0 - std::fabs(h - 25000.0) / 15000.0); }

// What scatters and what is lost at a height: Rayleigh and Mie scattering, and the extinction (scattering, Mie's
// absorption, ozone's)
struct Medium { Vec3 rayleigh; double mie = 0; Vec3 extinction; };
Medium mediumAt(const Atmosphere& a, double h) {
    Medium m;
    const double rR = std::exp(-h / a.rayleighHeight), rM = std::exp(-h / a.mieHeight);
    m.rayleigh = a.rayleighScatter * rR;
    m.mie = a.mieScatter * rM;
    m.extinction = m.rayleigh + Vec3(a.mieExtinction * rM, a.mieExtinction * rM, a.mieExtinction * rM) + a.ozoneAbsorb * ozoneDensity(h);
    m.extinction = Vec3(std::max(m.extinction.x, 1e-12), std::max(m.extinction.y, 1e-12), std::max(m.extinction.z, 1e-12));
    return m;
}

double distToTop(double r, double mu, double rt) {
    const double d = r * r * (mu * mu - 1.0) + rt * rt;
    return std::max(-r * mu + std::sqrt(std::max(d, 0.0)), 0.0);
}
// distance to the ground along (r, mu), or -1 when the ray misses it
double distToGround(double r, double mu, double rg) {
    const double d = r * r * (mu * mu - 1.0) + rg * rg;
    if (d < 0.0 || mu >= 0.0) return -1.0;
    const double t = -r * mu - std::sqrt(d);
    return t > 0.0 ? t : -1.0;
}

// Bruneton's transmittance mapping (the GPU's sky-view pass uses the same, sky_view.comp)
void transUv(double r, double mu, double rg, double rt, double& u, double& v) {
    const double H = std::sqrt(std::max(rt * rt - rg * rg, 0.0));
    const double rho = std::sqrt(std::max(r * r - rg * rg, 0.0));
    const double d = distToTop(r, mu, rt), dMin = rt - r, dMax = rho + H;
    u = clamp01((d - dMin) / std::max(dMax - dMin, 1e-4));
    v = clamp01(rho / std::max(H, 1e-4));
}
void transRMu(double u, double v, double rg, double rt, double& r, double& mu) {
    const double H = std::sqrt(std::max(rt * rt - rg * rg, 0.0));
    const double rho = H * v;
    r = std::sqrt(rho * rho + rg * rg);
    const double dMin = rt - r, dMax = rho + H, d = dMin + u * (dMax - dMin);
    mu = d <= 0.0 ? 1.0 : std::clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
}

Vec3 bilinear(const std::vector<float>& tex, int w, int h, double u, double v) {
    const double x = clamp01(u) * w - 0.5, y = clamp01(v) * h - 0.5;
    const int x0 = std::clamp(static_cast<int>(std::floor(x)), 0, w - 1), y0 = std::clamp(static_cast<int>(std::floor(y)), 0, h - 1);
    const int x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
    const double fx = clamp01(x - x0), fy = clamp01(y - y0);
    auto at = [&](int xi, int yi) {
        const std::size_t i = (static_cast<std::size_t>(yi) * w + xi) * 3;
        return Vec3(tex[i], tex[i + 1], tex[i + 2]);
    };
    return (at(x0, y0) * (1 - fx) + at(x1, y0) * fx) * (1 - fy) + (at(x0, y1) * (1 - fx) + at(x1, y1) * fx) * fy;
}

Vec3 expV(const Vec3& v) { return Vec3(std::exp(v.x), std::exp(v.y), std::exp(v.z)); }
Vec3 mulV(const Vec3& a, const Vec3& b) { return Vec3(a.x * b.x, a.y * b.y, a.z * b.z); }
Vec3 divV(const Vec3& a, const Vec3& b) { return Vec3(a.x / b.x, a.y / b.y, a.z / b.z); }

// a well-spread direction on the unit sphere (spherical Fibonacci), i of n
Vec3 fibonacciDir(int i, int n) {
    const double golden = kPi * (3.0 - std::sqrt(5.0));
    const double y = 1.0 - 2.0 * (i + 0.5) / n, rad = std::sqrt(std::max(0.0, 1.0 - y * y)), th = golden * i;
    return Vec3(std::cos(th) * rad, y, std::sin(th) * rad);
}

}  // namespace

Atmosphere atmosphereFrom(const SkyScatteringParams& p) {
    Atmosphere a;
    a.groundRadius = p.planetRadius;
    a.topRadius = p.planetRadius + p.atmosphereHeight;
    a.rayleighScatter = p.rayleighBeta;
    a.rayleighHeight = p.rayleighScaleHeight;
    a.mieScatter = p.mieBeta * p.turbidity;
    a.mieExtinction = p.mieBeta * 1.11 * p.turbidity;
    a.mieHeight = p.mieScaleHeight;
    a.mieG = p.mieG;
    a.groundAlbedo = p.groundAlbedo;
    a.multiScatter = p.multiScatter;
    return a;
}

Vec3 atmosphereTransmittance(const Atmosphere& a, double height, double mu) {
    const double r = a.groundRadius + std::max(height, 0.0);
    if (distToGround(r, mu, a.groundRadius) > 0.0) return Vec3(0, 0, 0);
    const double dist = distToTop(r, mu, a.topRadius);
    const int steps = 64;
    const double ds = dist / steps;
    Vec3 od(0, 0, 0);
    for (int i = 0; i < steps; ++i) {
        const double t = (i + 0.5) * ds;
        const double rp = std::sqrt(std::max(r * r + t * t + 2.0 * r * mu * t, 1.0));
        od = od + mediumAt(a, std::max(rp - a.groundRadius, 0.0)).extinction * ds;
    }
    return expV(od * -1.0);
}

Vec3 sampleTransmittance(const AtmosphereTables& t, double r, double mu) {
    const double rg = t.atm.groundRadius, rt = t.atm.topRadius;
    r = std::clamp(r, rg, rt);
    const double horizonMu = -std::sqrt(std::max(1.0 - (rg * rg) / (r * r), 0.0));
    if (mu < horizonMu) return Vec3(0, 0, 0);
    double u, v;
    transUv(r, mu, rg, rt, u, v);
    return bilinear(t.transmittance, AtmosphereTables::kTransW, AtmosphereTables::kTransH, u, v);
}

Vec3 sampleMultiScatter(const AtmosphereTables& t, double r, double muS) {
    const double u = clamp01(muS * 0.5 + 0.5), v = clamp01((r - t.atm.groundRadius) / (t.atm.topRadius - t.atm.groundRadius));
    return bilinear(t.multiScatter, AtmosphereTables::kMsW, AtmosphereTables::kMsH, u, v);
}

void buildAtmosphereTables(const Atmosphere& a, AtmosphereTables& out) {
    out.atm = a;
    const double rg = a.groundRadius, rt = a.topRadius;
    // TRANSMITTANCE: each texel's (r, mu) marched to the top
    out.transmittance.assign(static_cast<std::size_t>(AtmosphereTables::kTransW) * AtmosphereTables::kTransH * 3, 0.0f);
    for (int y = 0; y < AtmosphereTables::kTransH; ++y)
        for (int x = 0; x < AtmosphereTables::kTransW; ++x) {
            double r, mu;
            transRMu((x + 0.5) / AtmosphereTables::kTransW, (y + 0.5) / AtmosphereTables::kTransH, rg, rt, r, mu);
            const double dist = distToTop(r, mu, rt);
            const int steps = 40;
            const double ds = dist / steps;
            Vec3 od(0, 0, 0);
            for (int i = 0; i < steps; ++i) {
                const double tt = (i + 0.5) * ds;
                const double rp = std::sqrt(std::max(r * r + tt * tt + 2.0 * r * mu * tt, 1.0));
                od = od + mediumAt(a, std::max(rp - rg, 0.0)).extinction * ds;
            }
            const Vec3 tr = expV(od * -1.0);
            const std::size_t i = (static_cast<std::size_t>(y) * AtmosphereTables::kTransW + x) * 3;
            out.transmittance[i] = static_cast<float>(tr.x);
            out.transmittance[i + 1] = static_cast<float>(tr.y);
            out.transmittance[i + 2] = static_cast<float>(tr.z);
        }
    // MULTIPLE SCATTERING (Hillaire 2020, sec. 5.5): at each (height, sun cosine), the second-order light arriving from
    // every direction (isotropic phase) and the fraction f of light the surroundings scatter back; the infinite series
    // of higher orders sums to G / (1 - f)
    out.multiScatter.assign(static_cast<std::size_t>(AtmosphereTables::kMsW) * AtmosphereTables::kMsH * 3, 0.0f);
    const int dirs = 64, steps = 20;
    const double iso = 1.0 / (4.0 * kPi);
    for (int y = 0; y < AtmosphereTables::kMsH; ++y)
        for (int x = 0; x < AtmosphereTables::kMsW; ++x) {
            const double muS = std::clamp(2.0 * (x + 0.5) / AtmosphereTables::kMsW - 1.0, -1.0, 1.0);
            const double r = rg + std::max((y + 0.5) / AtmosphereTables::kMsH * (rt - rg), 1.0);
            const Vec3 sun(std::sqrt(std::max(0.0, 1.0 - muS * muS)), muS, 0.0);
            const Vec3 x0(0, r, 0);
            Vec3 G(0, 0, 0), F(0, 0, 0);
            for (int d = 0; d < dirs; ++d) {
                const Vec3 w = fibonacciDir(d, dirs);
                const double tg = distToGround(r, w.y, rg);
                const double tMax = tg > 0.0 ? tg : distToTop(r, w.y, rt);
                const double ds = tMax / steps;
                Vec3 T(1, 1, 1), L(0, 0, 0), f(0, 0, 0);
                for (int i = 0; i < steps; ++i) {
                    const Vec3 p = x0 + w * ((i + 0.5) * ds);
                    const double rp = p.length();
                    const Medium m = mediumAt(a, std::max(rp - rg, 0.0));
                    const Vec3 sigS = m.rayleigh + Vec3(m.mie, m.mie, m.mie);
                    const Vec3 Ts = sampleTransmittance(out, rp, dot(p * (1.0 / rp), sun));
                    const Vec3 stepT = expV(m.extinction * -ds);
                    const Vec3 S = mulV(sigS, Ts) * iso;
                    const Vec3 one(1, 1, 1);
                    L = L + mulV(T, divV(mulV(S, one - stepT), m.extinction));
                    f = f + mulV(T, divV(mulV(sigS, one - stepT), m.extinction));
                    T = mulV(T, stepT);
                }
                if (tg > 0.0) {   // the ground's bounce (Lambertian)
                    const Vec3 p = x0 + w * tg;
                    const Vec3 n = p * (1.0 / p.length());
                    const double muG = dot(n, sun);
                    const Vec3 Ts = sampleTransmittance(out, rg, muG);
                    L = L + mulV(mulV(T, Ts), a.groundAlbedo) * (std::max(muG, 0.0) / kPi);
                }
                G = G + L * (1.0 / dirs);
                F = F + f * (iso * 4.0 * kPi / dirs);
            }
            const Vec3 psi(G.x / std::max(1e-6, 1.0 - F.x) * a.multiScatter, G.y / std::max(1e-6, 1.0 - F.y) * a.multiScatter,
                           G.z / std::max(1e-6, 1.0 - F.z) * a.multiScatter);
            const std::size_t i = (static_cast<std::size_t>(y) * AtmosphereTables::kMsW + x) * 3;
            out.multiScatter[i] = static_cast<float>(psi.x);
            out.multiScatter[i + 1] = static_cast<float>(psi.y);
            out.multiScatter[i + 2] = static_cast<float>(psi.z);
        }
}

Vec3 atmosphereSkyRadiance(const AtmosphereTables& t, double height, const Vec3& view, const Vec3& light, int steps) {
    const Atmosphere& a = t.atm;
    const double rg = a.groundRadius, rt = a.topRadius;
    const double r = rg + std::max(height, 0.5);
    const Vec3 x0(0, r, 0);
    const double tg = distToGround(r, view.y, rg);
    const double tMax = tg > 0.0 ? tg : distToTop(r, view.y, rt);
    if (tMax <= 0.0) return Vec3(0, 0, 0);
    const double nu = dot(view, light);
    const double phaseR = 3.0 / (16.0 * kPi) * (1.0 + nu * nu);
    const double g = a.mieG, g2 = g * g;
    const double phaseM = 3.0 / (8.0 * kPi) * ((1.0 - g2) * (1.0 + nu * nu)) / ((2.0 + g2) * std::pow(1.0 + g2 - 2.0 * g * nu, 1.5));
    const double ds = tMax / steps;
    Vec3 T(1, 1, 1), L(0, 0, 0);
    const Vec3 one(1, 1, 1);
    for (int i = 0; i < steps; ++i) {
        const Vec3 p = x0 + view * ((i + 0.5) * ds);
        const double rp = p.length();
        const Medium m = mediumAt(a, std::max(rp - rg, 0.0));
        const double muS = dot(p * (1.0 / rp), light);
        const Vec3 Ts = sampleTransmittance(t, rp, muS);
        const Vec3 single = mulV(Ts, m.rayleigh * phaseR + Vec3(m.mie, m.mie, m.mie) * phaseM);
        const Vec3 multi = mulV(m.rayleigh + Vec3(m.mie, m.mie, m.mie), sampleMultiScatter(t, rp, muS));
        const Vec3 S = single + multi;
        const Vec3 stepT = expV(m.extinction * -ds);
        L = L + mulV(T, divV(mulV(S, one - stepT), m.extinction));
        T = mulV(T, stepT);
    }
    if (tg > 0.0) {
        const Vec3 p = x0 + view * tg;
        const Vec3 n = p * (1.0 / p.length());
        const double muG = dot(n, light);
        L = L + mulV(mulV(T, sampleTransmittance(t, rg, muG)), a.groundAlbedo) * (std::max(muG, 0.0) / kPi);
    }
    return L;
}

Vec3 atmosphereSkyIrradiance(const AtmosphereTables& t, double height, const Vec3& normal, const Vec3& light, int samples) {
    // cosine-weighted directions about the normal (a Fibonacci hemisphere, mapped by Malley's method)
    Vec3 up = std::fabs(normal.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    const Vec3 tx = normalize(cross(up, normal)), ty = cross(normal, tx);
    Vec3 sum(0, 0, 0);
    const double golden = kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < samples; ++i) {
        const double rr = std::sqrt((i + 0.5) / samples), th = golden * i;
        const double x = rr * std::cos(th), y = rr * std::sin(th), z = std::sqrt(std::max(0.0, 1.0 - rr * rr));
        const Vec3 d = normalize(tx * x + ty * y + normal * z);
        sum = sum + atmosphereSkyRadiance(t, height, d, light, 24);
    }
    return sum * (kPi / samples);   // E = pi * the cosine-weighted mean radiance
}

namespace {
void shBasis(const Vec3& d, double y[9]) {
    y[0] = 0.282095;
    y[1] = 0.488603 * d.y;
    y[2] = 0.488603 * d.z;
    y[3] = 0.488603 * d.x;
    y[4] = 1.092548 * d.x * d.y;
    y[5] = 1.092548 * d.y * d.z;
    y[6] = 0.315392 * (3.0 * d.z * d.z - 1.0);
    y[7] = 1.092548 * d.x * d.z;
    y[8] = 0.546274 * (d.x * d.x - d.y * d.y);
}
}  // namespace

void atmosphereIrradianceSH(const AtmosphereTables& t, double height, const Vec3& light, double topUnits, Vec3 out[9],
                            int samples) {
    for (int k = 0; k < 9; ++k) out[k] = Vec3(0, 0, 0);
    double y[9];
    for (int i = 0; i < samples; ++i) {
        const Vec3 d = fibonacciDir(i, samples);
        const Vec3 L = atmosphereSkyRadiance(t, height, d, light, 20) * topUnits;
        shBasis(d, y);
        for (int k = 0; k < 9; ++k) out[k] = out[k] + L * y[k];
    }
    // the radiance's coefficients (4 pi / N each), convolved with the clamped cosine (pi, 2 pi / 3, pi / 4 by band) and
    // over pi: the radiance-like E / pi the shader's ambient takes
    const double band[9] = {1.0, 2.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0, 0.25, 0.25, 0.25, 0.25, 0.25};
    for (int k = 0; k < 9; ++k) out[k] = out[k] * (4.0 * kPi / samples * band[k]);
}

Vec3 evalIrradianceSH(const Vec3 sh[9], const Vec3& n) {
    double y[9];
    shBasis(n, y);
    Vec3 e(0, 0, 0);
    for (int k = 0; k < 9; ++k) e = e + sh[k] * y[k];
    return Vec3(std::max(0.0, e.x), std::max(0.0, e.y), std::max(0.0, e.z));
}

SkyReading readSky(const AtmosphereTables& t, double height, const Vec3& light, double topUnits) {
    SkyReading s;
    const double r = t.atm.groundRadius + std::max(height, 0.5);
    const Vec3 tr = sampleTransmittance(t, r, light.y);
    const double peak = std::max(tr.x, std::max(tr.y, tr.z));
    s.lightColor = peak > 1e-9 ? tr * (1.0 / peak) : Vec3(1, 1, 1);
    s.lightIntensity = topUnits * peak;
    s.zenith = atmosphereSkyRadiance(t, height, Vec3(0, 1, 0), light) * topUnits;
    Vec3 hz(0, 0, 0);
    const double el = 0.035;   // two degrees up: the horizon band
    for (int k = 0; k < 8; ++k) {
        const double az = k * kPi / 4.0;
        hz = hz + atmosphereSkyRadiance(t, height, normalize(Vec3(std::cos(az), el, std::sin(az))), light);
    }
    s.horizon = hz * (topUnits / 8.0);
    s.ground = atmosphereSkyRadiance(t, height, normalize(Vec3(0.3, -1.0, 0.0)), light) * topUnits;
    s.irradianceUp = atmosphereSkyIrradiance(t, height, Vec3(0, 1, 0), light, 48) * topUnits;
    return s;
}

}  // namespace engine
