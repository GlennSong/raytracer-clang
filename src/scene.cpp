#include "scene.h"

#include <algorithm>

namespace engine {

// --- BRDF + light falloff, mirroring the realtime renderer ----------------
// These are line-for-line ports of lighting.metal so direct light, shadows,
// and materials match the viewer (ADR-0017): GGX NDF, height-correlated Smith
// visibility (with the 1/(4 NdotL NdotV) folded in), Schlick Fresnel with
// f0 = lerp(0.04, albedo, metallic), and UE-style windowed inverse-square
// falloff. Roughness is perceptual (squared into alpha, floored at 0.002).
namespace {

double luminanceOf(const Vec3& c) {
    return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
}

double distributionGGX(double NdotH, double a2) {
    double d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / std::max(PI * d * d, 1e-9);
}

double visibilitySmithGGX(double NdotV, double NdotL, double a2) {
    double gv = NdotL * std::sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    double gl = NdotV * std::sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / std::max(gv + gl, 1e-7);
}

Vec3 fresnelSchlick(double cosTheta, const Vec3& f0) {
    double f = std::pow(std::clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
    return f0 + (Vec3(1, 1, 1) - f0) * f;
}

double distanceAttenuation(double dist, double range) {
    double ratio2 = (dist * dist) / std::max(range * range, 1e-4);
    double window = std::clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
    return window * window / std::max(dist * dist, 1e-4);
}

Vec3 f0For(const Vec3& albedo, double metallic) {
    return Vec3(0.04, 0.04, 0.04) * (1.0 - metallic) + albedo * metallic;
}

// lighting.metal's per-light evaluation: specular D*Vis*F plus an
// energy-balanced Lambert term ((1-F)(1-metallic) albedo / pi).
Vec3 evalBRDF(const Vec3& n, const Vec3& v, const Vec3& l,
              const Vec3& albedo, double metallic, double roughness) {
    double a = std::max(roughness * roughness, 0.002);
    double a2 = a * a;
    double NdotV = std::max(dot(n, v), 1e-4);
    double NdotL = std::max(dot(n, l), 0.0);
    Vec3 h = normalize(l + v);
    double NdotH = std::max(dot(n, h), 0.0);
    double VdotH = std::max(dot(v, h), 0.0);

    Vec3 F = fresnelSchlick(VdotH, f0For(albedo, metallic));
    Vec3 specular = F * (distributionGGX(NdotH, a2) *
                         visibilitySmithGGX(NdotV, NdotL, a2));
    Vec3 diffuse = (Vec3(1, 1, 1) - F) * ((1.0 - metallic) / PI) * albedo;
    return diffuse + specular;
}

void buildBasis(const Vec3& n, Vec3& t, Vec3& b) {
    Vec3 up = std::abs(n.y) < 0.99 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    t = normalize(cross(up, n));
    b = cross(n, t);
}

// common.metal's applyCheckerboard: 1m world-space squares, dark = 0.3x.
Vec3 applyCheckerboard(const Vec3& albedo, const Vec3& worldPos) {
    int cx = static_cast<int>(std::floor(worldPos.x));
    int cz = static_cast<int>(std::floor(worldPos.z));
    return (((cx + cz) & 1) != 0) ? albedo * 0.3 : albedo;
}

// --- Procedural surface library --------------------------------------------
// Analytic, world-space materials a city needs (brick, concrete, roof tiles,
// asphalt, ...), evaluated at the hit point. Each is kept byte-for-byte in step
// with common.metal's applySurface so the offline tracer previews exactly what
// the Metal viewer draws. All take a base albedo (the material/vertex colour)
// and return the patterned albedo. See material.h Surface ids.
double surfHash(double a, double b) {   // == common.metal hash21
    double s = std::sin(a * 12.9898 + b * 78.233) * 43758.5453;
    return s - std::floor(s);
}
double surfTile(double x, double m) { return x - m * std::floor(x / m); }  // [0,m)
double surfStep(double e0, double e1, double x) {
    double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
double surfVNoise(double x, double y) {            // value noise, bilinear
    double xi = std::floor(x), yi = std::floor(y), xf = x - xi, yf = y - yi;
    double a = surfHash(xi, yi), b = surfHash(xi + 1, yi);
    double c = surfHash(xi, yi + 1), d = surfHash(xi + 1, yi + 1);
    double ux = xf * xf * (3 - 2 * xf), uy = yf * yf * (3 - 2 * yf);
    return a * (1 - ux) * (1 - uy) + b * ux * (1 - uy) +
           c * (1 - ux) * uy + d * ux * uy;
}
double surfFbm(double x, double y) {               // ~[0,1]
    double v = 0, amp = 0.5, f = 1;
    for (int i = 0; i < 4; ++i) { v += amp * surfVNoise(x * f, y * f); f *= 2; amp *= 0.5; }
    return v;
}
// Facade coords: v = height up a wall, u = horizontal run; horizontal surfaces
// (roofs/ground) lay the pattern in the XZ plane.
void surfPlane(const Vec3& p, const Vec3& n, double& u, double& v) {
    if (std::fabs(n.y) > 0.5) { u = p.x; v = p.z; return; }
    double tx = n.z, tz = -n.x, tl = std::sqrt(tx * tx + tz * tz);
    if (tl < 1e-6) { tx = 1; tz = 0; tl = 1; }
    u = p.x * (tx / tl) + p.z * (tz / tl);
    v = p.y;
}

// The world-planar tiling frame for texture sampling: tile UV (scaled) plus the
// tangent (T, along u) and bitangent (B, along v) so a tangent-space normal map
// can be rotated into world space. Matches surfPlane's u/v axes.
void surfFrame(const Vec3& p, const Vec3& n, double scale, double& u, double& v,
               Vec3& T, Vec3& B) {
    if (std::fabs(n.y) > 0.5) {
        u = p.x * scale; v = p.z * scale;
        T = Vec3(1, 0, 0); B = Vec3(0, 0, 1);
    } else {
        double tx = n.z, tz = -n.x, tl = std::sqrt(tx * tx + tz * tz);
        if (tl < 1e-6) { tx = 1; tz = 0; tl = 1; }
        tx /= tl; tz /= tl;
        u = (p.x * tx + p.z * tz) * scale; v = p.y * scale;
        T = Vec3(tx, 0, tz); B = Vec3(0, 1, 0);
    }
}

Vec3 surfBrick(const Vec3& base, double u, double v) {
    const double courseH = 0.075, brickL = 0.20, mortar = 0.011;
    double row = std::floor(v / courseH);
    double off = (std::fmod(std::fabs(row), 2.0) < 1.0) ? 0.0 : brickL * 0.5;
    double uu = u + off, col = std::floor(uu / brickL);
    double fy = v - row * courseH, fx = uu - col * brickL;
    double joint = std::min(std::min(fy, courseH - fy), std::min(fx, brickL - fx));
    double h = surfHash(col, row), h2 = surfHash(col * 1.7 + 3.1, row * 0.9 + 5.7);
    double shade = 0.74 + 0.46 * h;
    if (h2 < 0.12) shade *= 0.6;
    shade *= 0.94 + 0.12 * (fx / brickL);
    Vec3 mortarCol(0.30, 0.29, 0.27);
    double t = std::clamp((joint - mortar) / 0.004, 0.0, 1.0);
    return mortarCol + (base * shade - mortarCol) * t;
}
Vec3 surfConcrete(const Vec3& base, double u, double v) {
    double n = surfFbm(u * 0.6, v * 0.6), fine = surfVNoise(u * 9.0, v * 9.0);
    double shade = 0.84 + 0.22 * n + 0.06 * (fine - 0.5);
    double gu = surfTile(u, 3.0); gu = std::min(gu, 3.0 - gu);
    double gv = surfTile(v, 3.0); gv = std::min(gv, 3.0 - gv);
    double jt = surfStep(0.015, 0.04, std::min(gu, gv));   // control joints
    return base * shade * (0.74 + 0.26 * jt);
}
Vec3 surfStucco(const Vec3& base, double u, double v) {
    double n = surfFbm(u * 3.0, v * 3.0), fine = surfVNoise(u * 22.0, v * 22.0);
    return base * (0.90 + 0.12 * (n - 0.5) + 0.10 * (fine - 0.5));
}
Vec3 surfRoofTile(const Vec3& base, double u, double v) {
    const double tileW = 0.18, rowH = 0.32;
    double row = std::floor(v / rowH);
    double off = (std::fmod(std::fabs(row), 2.0) < 1.0) ? 0.0 : tileW * 0.5;
    double uu = u + off, col = std::floor(uu / tileW);
    double fx = uu - col * tileW, fy = v - row * rowH;
    double curve = std::sin(PI * (fx / tileW));               // barrel crown
    double valley = surfStep(0.0, 0.02, std::min(fx, tileW - fx));
    double lap = surfStep(0.0, 0.05, fy);                     // head-lap shadow
    double h = surfHash(col, row);
    double shade = (0.55 + 0.5 * curve) * (0.85 + 0.30 * h) * valley * (0.6 + 0.4 * lap);
    return base * shade;
}
Vec3 surfShingle(const Vec3& base, double u, double v) {
    const double tabW = 0.30, rowH = 0.14;
    double row = std::floor(v / rowH);
    double off = (std::fmod(std::fabs(row), 2.0) < 1.0) ? 0.0 : tabW * 0.5;
    double uu = u + off, col = std::floor(uu / tabW);
    double fx = uu - col * tabW, fy = v - row * rowH;
    double h = surfHash(col, row);
    double key = surfStep(0.0, 0.012, std::min(fx, tabW - fx));
    double shadow = surfStep(0.0, 0.03, fy);
    return base * (0.82 + 0.32 * h) * (0.55 + 0.45 * key) * (0.5 + 0.5 * shadow);
}
Vec3 surfCorrugated(const Vec3& base, double u, double v) {
    const double ribW = 0.12;
    double rib = std::cos(2.0 * PI * u / ribW);
    double shade = 0.72 + 0.28 * rib;
    double rust = surfFbm(u * 1.5, v * 0.6);
    double rmask = std::clamp((rust - 0.62) / 0.18, 0.0, 1.0) * 0.45;
    Vec3 rustCol(0.40, 0.22, 0.12);
    return base * shade * (1 - rmask) + rustCol * rmask;
}
Vec3 surfAsphalt(const Vec3& base, double u, double v) {
    double spk = surfVNoise(u * 30.0, v * 30.0), patch = surfFbm(u * 0.4, v * 0.4);
    double shade = std::clamp(0.92 + 0.46 * (spk - 0.5) + 0.12 * (patch - 0.5), 0.5, 1.4);
    return base * shade;
}
Vec3 surfPavement(const Vec3& base, double u, double v) {
    const double slab = 1.2;
    double su = surfTile(u, slab), sv = surfTile(v, slab);
    double joint = std::min(std::min(su, slab - su), std::min(sv, slab - sv));
    double h = surfHash(std::floor(u / slab), std::floor(v / slab));
    double spk = surfVNoise(u * 26.0, v * 26.0);
    double shade = 0.90 + 0.12 * (h - 0.5) + 0.06 * (spk - 0.5);
    double jt = surfStep(0.02, 0.05, joint);
    return base * shade * (0.6 + 0.4 * jt);
}
Vec3 surfCobble(const Vec3& base, double u, double v) {
    const double cell = 0.18;
    double cu = u / cell, cv = v / cell, iu = std::floor(cu), iv = std::floor(cv);
    double best = 1e9, bh = 0;
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            double ci = iu + di, cj = iv + dj;
            double jx = surfHash(ci, cj), jy = surfHash(ci + 5.2, cj + 1.7);
            double px = ci + 0.5 + (jx - 0.5) * 0.7, py = cj + 0.5 + (jy - 0.5) * 0.7;
            double dx = cu - px, dy = cv - py, d = dx * dx + dy * dy;
            if (d < best) { best = d; bh = surfHash(ci + 9.1, cj + 4.3); }
        }
    double stone = surfStep(0.0, 0.12, 0.62 - std::sqrt(best));
    Vec3 mortar(0.32, 0.30, 0.27);
    return mortar + (base * (0.7 + 0.6 * bh) - mortar) * stone;
}
Vec3 surfWood(const Vec3& base, double u, double v) {
    const double boardH = 0.18;
    double row = std::floor(v / boardH), fy = v - row * boardH;
    double h = surfHash(row, 3.0), grain = surfVNoise(u * 40.0, row * 9.0 + v * 2.0);
    double shadow = surfStep(0.0, 0.02, fy);
    return base * (0.85 + 0.20 * h + 0.12 * (grain - 0.5)) * (0.55 + 0.45 * shadow);
}

// Dispatch a Surface id (material.h / renderer.h Surface) to its pattern. The
// result is clamped to [0,1] per channel: an albedo above 1 would make the path
// tracer gain energy each bounce (throughput never decays, Russian roulette
// never terminates — a pathological slowdown), so surfaces stay conserving.
// Road lane paint composited over the asphalt from road-local mesh UV (ADR-0044 /
// road-geometry-plan Problem 3), not stripe geometry. `mu` encodes lateral position
// (1 = left edge, 2 = centre, 3 = right edge; < 0.5 means this isn't carriageway — a
// sidewalk or junction pad — so it stays plain). Normalised across the width, so it
// tracks any road width and conforms to terrain for free. `mv` (arc-length) is baked
// for future dashed dividers; v1 paints a double-yellow centreline + white edge lines.
Vec3 surfRoadMarkings(const Vec3& base, double mu, double mv, double wu, double wv) {
    // The welded road wears ONE surface id, split by its road-local mu: the
    // carriageway sits at mu in [1,3] (lat = mu-2); the raised sidewalk/curb band
    // carries mu in [0,1]. Texture BOTH (device: roads/sidewalks were flat
    // colour): concrete pavement on the band, asphalt grain under the lane
    // paint. wu/wv is the world-planar UV the other surfaces tile by.
    if (mu < 0.98) return surfPavement(base, wu, wv);   // sidewalk / curb band
    Vec3 deck = surfAsphalt(base, wu, wv);              // grained asphalt deck
    double lat = mu - 2.0;                            // [-1, 1], 0 = centreline, ±1 = curb
    auto band = [](double x, double c, double hw) {  // ~1 inside a line of half-width hw
        double d = std::fabs(x - c), t = std::clamp((d - hw) / 0.006, 0.0, 1.0);
        return 1.0 - t * t * (3.0 - 2.0 * t);        // smoothstep falloff (soft edge)
    };
    const Vec3 yellow(0.82, 0.68, 0.13), white(0.86, 0.86, 0.83);
    double y = std::max(band(lat, 0.030, 0.013), band(lat, -0.030, 0.013));  // double yellow centre
    double w = std::max(band(lat, 0.92, 0.016), band(lat, -0.92, 0.016));    // solid white edges
    // The centreline ENDS before a crosswalk (device: the double yellow cut
    // through the zebra bars): mv = metres past the junction mouth and the band
    // ends at ~3.6, so the yellow fades in just past it. Without crosswalks mv
    // is a large sentinel and the line runs the full segment as before.
    {
        double t = std::clamp((mv - 4.0) / 0.8, 0.0, 1.0);
        y *= t * t * (3.0 - 2.0 * t);
    }
    Vec3 c = deck * (1.0 - y) + yellow * y;
    c = c * (1.0 - w) + white * w;
    // Zebra crosswalk painted into the road texture (ADR-0062): mv = metres PAST the
    // junction mouth (baked by the road mesher), so the band sits set back on the
    // approach, not in the intersection. Bars run across the carriageway (mu). This
    // mirrors the Metal/Vulkan RoadMarkings shader for realtime<->offline parity.
    auto sstep = [](double a, double b, double x) {
        double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    };
    // Only the carriageway (mu in [~1,3]) — the raised sidewalk/curb shares this
    // surface but carries a 0..1 UV, so gate on mu > 1.05 to keep paint off the curb.
    double cwEdge = sstep(0.5, 0.8, mv) * (1.0 - sstep(3.3, 3.6, mv));
    double barPhase = (mu - 1.0) * 4.0; barPhase -= std::floor(barPhase);
    double cw = cwEdge * (barPhase < 0.5 ? 1.0 : 0.0) * (mu > 1.05 ? 1.0 : 0.0);
    c = c * (1.0 - cw) + white * cw;
    return c;
}

Vec3 applySurface(int id, const Vec3& base, const Vec3& worldPos, const Vec3& n,
                  double mu, double mv) {
    double u, v; surfPlane(worldPos, n, u, v);
    Vec3 c;
    switch (id) {
        case 1:  c = surfBrick(base, u, v); break;
        case 2:  c = surfConcrete(base, u, v); break;
        case 3:  c = surfStucco(base, u, v); break;
        case 4:  c = surfRoofTile(base, u, v); break;
        case 5:  c = surfShingle(base, u, v); break;
        case 6:  c = surfCorrugated(base, u, v); break;
        case 7:  c = surfAsphalt(base, u, v); break;
        case 8:  c = surfPavement(base, u, v); break;
        case 9:  c = surfCobble(base, u, v); break;
        case 10: c = surfWood(base, u, v); break;
        case 11: c = surfRoadMarkings(base, mu, mv, u, v); break;
        default: return base;
    }
    return Vec3(std::clamp(c.x, 0.0, 1.0), std::clamp(c.y, 0.0, 1.0),
                std::clamp(c.z, 0.0, 1.0));
}

}  // namespace

Vec3 EnvironmentMap::sample(const Vec3& unitDir) const {
    // Inverse of the loader's equirect mapping (model_importer.cpp):
    // dir = (sin(theta) cos(phi), cos(theta), sin(theta) sin(phi)).
    double theta = std::acos(std::clamp(unitDir.y, -1.0, 1.0));
    double phi = std::atan2(unitDir.z, unitDir.x);
    double x = (phi / (2.0 * PI) + 0.5) * width - 0.5;
    double y = (theta / PI) * height - 0.5;

    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    double fx = x - x0, fy = y - y0;
    auto texel = [&](int xi, int yi) {
        xi = ((xi % width) + width) % width;          // wrap in azimuth
        yi = std::clamp(yi, 0, height - 1);           // clamp at the poles
        const float* p = &pixels[(static_cast<size_t>(yi) * width + xi) * 3];
        return Vec3(p[0], p[1], p[2]);
    };
    return texel(x0, y0) * ((1 - fx) * (1 - fy)) +
           texel(x0 + 1, y0) * (fx * (1 - fy)) +
           texel(x0, y0 + 1) * ((1 - fx) * fy) +
           texel(x0 + 1, y0 + 1) * (fx * fy);
}

void EnvironmentMap::suppressSunDisc() {
    if (!valid()) return;
    const size_t count = static_cast<size_t>(width) * height;

    float maxLum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        const float* p = &pixels[i * 3];
        maxLum = std::max(maxLum,
                          0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2]);
    }
    const float disc = 0.5f * maxLum;   // the extraction's own disc threshold

    // Fill with the circumsolar ring (bright sky just below disc level), so
    // the patched region blends instead of leaving a dark hole.
    Vec3 ring;
    size_t ringCount = 0;
    for (size_t i = 0; i < count; i++) {
        const float* p = &pixels[i * 3];
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        if (lum >= 0.1f * disc && lum < disc) {
            ring += Vec3(p[0], p[1], p[2]);
            ringCount++;
        }
    }
    if (ringCount == 0) return;   // no meaningful ring: leave the map alone
    Vec3 fill = ring / static_cast<double>(ringCount);

    for (size_t i = 0; i < count; i++) {
        float* p = &pixels[i * 3];
        float lum = 0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2];
        if (lum >= disc) {
            p[0] = static_cast<float>(fill.x);
            p[1] = static_cast<float>(fill.y);
            p[2] = static_cast<float>(fill.z);
        }
    }
}

Vec3 EnvironmentLight::radiance(const Vec3& unitDir) const {
    if (map.valid()) return map.sample(unitDir) * skyIntensity;
    // Gradient sky fallback: horizon -> zenith above, horizon color below
    // (ground-bounce stand-in).
    double t = std::max(unitDir.y, 0.0);
    return ((1.0 - t) * skyHorizon + t * skyZenith) * skyIntensity;
}

Vec3 Scene::sampleDirectLight(const Vec3& point, const Vec3& normal,
                              const Vec3& viewDir, const Vec3& albedo,
                              double metallic, double roughness) const {
    if (lights.empty()) return Vec3(0, 0, 0);

    // One light per sample, picked uniformly (estimator scales by count).
    size_t idx = std::min(static_cast<size_t>(randomDouble() * lights.size()),
                          lights.size() - 1);
    const SceneLight& light = lights[idx];

    Vec3 lightDir;
    double attenuation;
    double maxDist = 1e19;

    if (light.type == SceneLight::Type::Directional) {
        // Soft sun: jitter the shadow ray within the angular radius, the
        // path-traced analog of the viewer's PCF penumbra.
        Vec3 d = normalize(light.direction);
        Vec3 t, b;
        buildBasis(d, t, b);
        double r = light.angularRadius * std::sqrt(randomDouble());
        double phi = 2.0 * PI * randomDouble();
        lightDir = normalize(d + (t * std::cos(phi) + b * std::sin(phi)) *
                                     std::tan(r));
        attenuation = light.intensity;
    } else {
        Vec3 toLight = light.position - point;
        double dist = toLight.length();
        if (dist <= 1e-6) return Vec3(0, 0, 0);
        lightDir = toLight / dist;
        attenuation = light.intensity * distanceAttenuation(dist, light.range);

        if (light.type == SceneLight::Type::Spot) {
            // smoothstep(outerCos, innerCos, theta), as in the shader.
            double theta = dot(-lightDir, normalize(light.direction));
            double innerCos = std::cos(light.innerConeAngle);
            double outerCos = std::cos(light.outerConeAngle);
            double e = std::clamp((theta - outerCos) /
                                      std::max(innerCos - outerCos, 1e-6),
                                  0.0, 1.0);
            attenuation *= e * e * (3.0 - 2.0 * e);
        }
        maxDist = dist - 1e-3;
    }

    double NdotL = dot(normal, lightDir);
    if (NdotL <= 0.0 || attenuation <= 0.0) return Vec3(0, 0, 0);

    // Shadow ray; opaque occluders count, alpha-cut leaves let light through
    // (dappled shade), like the viewer's alpha-tested shadow pass.
    HitRecord shadowRec;
    if (intersectVisible(Ray(point, lightDir), 1e-3, maxDist, shadowRec))
        return Vec3(0, 0, 0);

    Vec3 brdf = evalBRDF(normal, viewDir, lightDir, albedo, metallic, roughness);
    return brdf * light.color *
           (attenuation * NdotL * static_cast<double>(lights.size()));
}

double Texture::sampleAlpha(double u, double v) const {
    if (pixels.empty() || channels < 1) return 1.0;
    int x = std::clamp(static_cast<int>(u * width), 0, width - 1);
    int y = std::clamp(static_cast<int>(v * height), 0, height - 1);
    size_t i = (static_cast<size_t>(y) * width + x) * channels + (channels - 1);
    return pixels[i] / 255.0;
}

namespace {
// Bilinear fetch with wrapping (for tiling textures); reads up to 3 channels.
void texFetchWrapped(const Texture& t, double u, double v, double out[3]) {
    double fx = (u - std::floor(u)) * t.width - 0.5;
    double fy = (v - std::floor(v)) * t.height - 0.5;
    int x0 = static_cast<int>(std::floor(fx)), y0 = static_cast<int>(std::floor(fy));
    double tx = fx - x0, ty = fy - y0;
    auto px = [&](int x, int y, int c) {
        x = ((x % t.width) + t.width) % t.width;
        y = ((y % t.height) + t.height) % t.height;
        int ch = std::min(c, t.channels - 1);
        return t.pixels[(static_cast<size_t>(y) * t.width + x) * t.channels + ch] / 255.0;
    };
    for (int c = 0; c < 3; ++c) {
        double a = px(x0, y0, c) * (1 - tx) + px(x0 + 1, y0, c) * tx;
        double b = px(x0, y0 + 1, c) * (1 - tx) + px(x0 + 1, y0 + 1, c) * tx;
        out[c] = a * (1 - ty) + b * ty;
    }
}
}  // namespace

Vec3 Texture::sampleRGB(double u, double v) const {
    if (pixels.empty() || channels < 1) return Vec3(1, 1, 1);
    double c[3];
    texFetchWrapped(*this, u, v, c);
    return Vec3(c[0], c[1], c[2]);
}

double Texture::sampleR(double u, double v) const {
    if (pixels.empty() || channels < 1) return 1.0;
    double c[3];
    texFetchWrapped(*this, u, v, c);
    return c[0];
}

int Scene::addMaterial(const Material& mat) {
    materials.push_back(mat);
    return static_cast<int>(materials.size()) - 1;
}

int Scene::addTexture(Texture tex) {
    textures.push_back(std::move(tex));
    return static_cast<int>(textures.size()) - 1;
}

void Scene::addSphere(const Vec3& center, double radius, int matIdx) {
    spheres.push_back(Sphere(center, radius, matIdx));
}

void Scene::addTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matIdx) {
    triangles.push_back(Triangle(v0, v1, v2, matIdx));
}

void Scene::addQuad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx) {
    quads.push_back(Quad(corner, edge1, edge2, matIdx));
}

void Scene::addMeshSphere(const Vec3& center, double radius, int matIdx,
                          int stacks, int slices) {
    for (int i = 0; i < stacks; i++) {
        double theta0 = PI * i / stacks;
        double theta1 = PI * (i + 1) / stacks;
        for (int j = 0; j < slices; j++) {
            double phi0 = 2.0 * PI * j / slices;
            double phi1 = 2.0 * PI * (j + 1) / slices;

            Vec3 p00 = center + radius * Vec3(
                std::sin(theta0) * std::cos(phi0),
                std::cos(theta0),
                std::sin(theta0) * std::sin(phi0));
            Vec3 p10 = center + radius * Vec3(
                std::sin(theta1) * std::cos(phi0),
                std::cos(theta1),
                std::sin(theta1) * std::sin(phi0));
            Vec3 p01 = center + radius * Vec3(
                std::sin(theta0) * std::cos(phi1),
                std::cos(theta0),
                std::sin(theta0) * std::sin(phi1));
            Vec3 p11 = center + radius * Vec3(
                std::sin(theta1) * std::cos(phi1),
                std::cos(theta1),
                std::sin(theta1) * std::sin(phi1));

            if (i != 0) addTriangle(p00, p10, p01, matIdx);
            if (i != stacks - 1) addTriangle(p10, p11, p01, matIdx);
        }
    }
}

int Scene::addProto(std::vector<Triangle> tris) {
    protos.emplace_back();
    protos.back().triangles = std::move(tris);
    protos.back().build();
    return static_cast<int>(protos.size()) - 1;
}

void Scene::addInstance(int proto, const Mat4& toWorld) {
    if (proto < 0 || proto >= static_cast<int>(protos.size())) return;
    instances.push_back(makeInstance(proto, toWorld, protos[proto]));
}

void Scene::buildAccelerator() {
    if (!triangles.empty()) {
        kdTree.build(triangles);
    }
    if (!instances.empty()) {
        tlas.build(instances);
    }
}

bool Scene::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    HitRecord tempRec;
    bool hitAnything = false;
    double closest = tMax;

    for (const auto& sphere : spheres) {
        if (sphere.intersect(ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }

    if (!kdTree.isEmpty()) {
        if (kdTree.intersect(triangles, ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    } else {
        for (const auto& tri : triangles) {
            if (tri.intersect(ray, tMin, closest, tempRec)) {
                hitAnything = true;
                closest = tempRec.t;
                rec = tempRec;
            }
        }
    }

    for (const auto& quad : quads) {
        if (quad.intersect(ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }

    // Instanced geometry (ADR-0041): the TLAS, gated by the running nearest hit.
    if (!tlas.isEmpty()) {
        if (tlas.intersect(instances, protos, ray, tMin, closest, tempRec)) {
            hitAnything = true;
            closest = tempRec.t;
            rec = tempRec;
        }
    }

    return hitAnything;
}

bool Scene::intersectVisible(const Ray& ray, double tMin, double tMax,
                             HitRecord& rec) const {
    double start = tMin;
    for (int i = 0; i < 64; i++) {   // cap: bounded leaf overdraw per ray
        if (!intersect(ray, start, tMax, rec)) return false;
        const Material& mat = materials[rec.materialIndex];
        if (mat.alphaTex >= 0 &&
            textures[mat.alphaTex].sampleAlpha(rec.u, rec.v) < mat.alphaCutoff) {
            start = rec.t + 1e-4;   // transparent texel: continue along the ray
            continue;
        }
        return true;
    }
    return false;
}

Vec3 Scene::tracePath(const Ray& ray, int maxBounces) const {
    Vec3 throughput(1.0, 1.0, 1.0);
    Vec3 radiance(0.0, 0.0, 0.0);
    Ray currentRay = ray;
    double firstDist = -1.0;

    for (int bounce = 0; bounce < maxBounces; bounce++) {
        HitRecord rec;
        if (!intersectVisible(currentRay, 0.001, 1e20, rec)) {
            if (environment.enabled)
                radiance += throughput *
                            environment.radiance(normalize(currentRay.direction));
            break;
        }
        if (bounce == 0) firstDist = rec.t;

        const Material& mat = materials[rec.materialIndex];

        // Flat emission; an emissive map (glTF) modulates it in the PBR block.
        if (mat.emissiveTex < 0) radiance += throughput * mat.emission;

        if (mat.type == MaterialType::EMISSIVE) {
            break;
        }

        if (mat.type == MaterialType::DIFFUSE) {
            // Delta lights can't be hit by chance, so explicit sampling here
            // never double-counts the scattered path below.
            Vec3 albedo = mat.albedo * rec.color;
            radiance += throughput *
                        sampleDirectLight(rec.point, rec.normal,
                                          -normalize(currentRay.direction),
                                          albedo, 0.0, 1.0);
            Vec3 scatterDir = randomCosineHemisphere(rec.normal);
            currentRay = Ray(rec.point, scatterDir);
            throughput = throughput * albedo;
        } else if (mat.type == MaterialType::PBR) {
            // The viewer's surface model (ADR-0017), path-traced: explicit
            // light sampling with the same GGX BRDF for direct light, then a
            // one-sample lobe pick (GGX specular vs cosine diffuse) for the
            // indirect bounce.
            Vec3 n = rec.normal;
            Vec3 v = -normalize(currentRay.direction);
            double rough = mat.roughness, metal = mat.metallic;
            Vec3 albedo = (mat.checkerboard
                               ? applyCheckerboard(mat.albedo, rec.point)
                               : mat.albedo) *
                          rec.color;
            if (mat.albedoTex >= 0 || mat.normalTex >= 0 || mat.mrTex >= 0 ||
                mat.aoTex >= 0 || mat.emissiveTex >= 0) {
                // A PBR texture set. glTF models sample with their own UVs +
                // tangent; the procedural bake uses a world-planar tiling frame.
                // Same slots either way — the source is invisible here.
                double tu, tv; Vec3 T, B;
                if (mat.meshUV) {
                    tu = rec.u * mat.texScale; tv = rec.v * mat.texScale;
                    T = rec.tangent;
                    B = T.lengthSquared() > 1e-12 ? normalize(cross(rec.normal, T))
                                                  : Vec3();
                } else {
                    surfFrame(rec.point, rec.normal, mat.texScale, tu, tv, T, B);
                }
                // glTF base-colour/emissive are sRGB-encoded; decode to linear.
                // Normal/MR/AO are linear data and sampled as-is.
                auto toLinear = [&](Vec3 c) {
                    return mat.meshUV ? Vec3(std::pow(c.x, 2.2), std::pow(c.y, 2.2),
                                             std::pow(c.z, 2.2))
                                      : c;
                };
                if (mat.albedoTex >= 0)
                    albedo = albedo * toLinear(textures[mat.albedoTex].sampleRGB(tu, tv));
                if (mat.mrTex >= 0) {
                    Vec3 mr = textures[mat.mrTex].sampleRGB(tu, tv);
                    rough = std::clamp(mr.y, 0.04, 1.0);
                    metal = mr.z;
                }
                if (mat.aoTex >= 0)
                    albedo = albedo * textures[mat.aoTex].sampleR(tu, tv);  // cheap AO
                if (mat.emissiveTex >= 0)
                    radiance += throughput * mat.emission *
                                toLinear(textures[mat.emissiveTex].sampleRGB(tu, tv));
                if (mat.normalTex >= 0 && T.lengthSquared() > 1e-12) {
                    Vec3 tn = textures[mat.normalTex].sampleRGB(tu, tv) * 2.0 -
                              Vec3(1, 1, 1);
                    n = normalize(T * tn.x + B * tn.y + rec.normal * tn.z);
                }
            } else if (mat.surface) {
                albedo = applySurface(mat.surface, albedo, rec.point, rec.normal,
                                      rec.u, rec.v);
            }

            radiance += throughput * sampleDirectLight(rec.point, n, v, albedo,
                                                       metal, rough);

            Vec3 f0 = f0For(albedo, metal);
            double specWeight = luminanceOf(f0);
            double diffWeight = luminanceOf(albedo) * (1.0 - metal);
            double pSpec = std::clamp(
                specWeight / std::max(specWeight + diffWeight, 1e-6), 0.05, 1.0);

            double a = std::max(rough * rough, 0.002);
            double a2 = a * a;
            if (randomDouble() < pSpec) {
                // Sample the GGX NDF for a half-vector around the normal.
                double u1 = randomDouble(), u2 = randomDouble();
                double cosTheta =
                    std::sqrt((1.0 - u1) / (1.0 + (a2 - 1.0) * u1));
                double sinTheta =
                    std::sqrt(std::max(1.0 - cosTheta * cosTheta, 0.0));
                double phi = 2.0 * PI * u2;
                Vec3 t, b;
                buildBasis(n, t, b);
                Vec3 h = normalize(t * (sinTheta * std::cos(phi)) +
                                   b * (sinTheta * std::sin(phi)) +
                                   n * cosTheta);
                Vec3 l = reflect(-v, h);
                double NdotL = dot(n, l);
                if (NdotL <= 0.0) break;
                double NdotV = std::max(dot(n, v), 1e-4);
                double NdotH = std::max(dot(n, h), 1e-6);
                double VdotH = std::max(dot(v, h), 1e-6);
                // NDF-sampling estimator: f*cos/pdf = F * Vis * 4 VdotH NdotL
                // / NdotH; clamped to tame fireflies at grazing angles.
                double w = std::min(visibilitySmithGGX(NdotV, NdotL, a2) * 4.0 *
                                        VdotH * NdotL / NdotH,
                                    8.0);
                throughput = throughput * fresnelSchlick(VdotH, f0) * (w / pSpec);
                currentRay = Ray(rec.point, l);
            } else {
                // (1-F) is folded into the direct term only — the indirect
                // diffuse keeps the plain albedo weight, like most realtime-
                // parity tracers.
                Vec3 l = randomCosineHemisphere(n);
                throughput = throughput * albedo *
                             ((1.0 - metal) / (1.0 - pSpec));
                currentRay = Ray(rec.point, l);
            }
        } else if (mat.type == MaterialType::METAL) {
            Vec3 reflected = reflect(normalize(currentRay.direction), rec.normal);
            if (mat.roughness > 0) {
                reflected = normalize(reflected + mat.roughness * randomInUnitSphere());
            }
            if (dot(reflected, rec.normal) <= 0) break;
            currentRay = Ray(rec.point, reflected);
            throughput = throughput * mat.albedo;
        } else if (mat.type == MaterialType::GLASS) {
            Vec3 unitDir = normalize(currentRay.direction);
            double etaRatio = rec.frontFace ? (1.0 / mat.ior) : mat.ior;

            double cosTheta = std::min(dot(-unitDir, rec.normal), 1.0);
            double sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);

            bool cannotRefract = etaRatio * sinTheta > 1.0;
            Vec3 direction;

            if (cannotRefract || schlick(cosTheta, mat.ior) > randomDouble()) {
                direction = reflect(unitDir, rec.normal);
            } else {
                direction = refract(unitDir, rec.normal, etaRatio);
            }

            currentRay = Ray(rec.point, direction);
            throughput = throughput * mat.albedo;
        }

        // Russian roulette after 3 bounces
        if (bounce > 3) {
            double p = std::max({throughput.x, throughput.y, throughput.z});
            if (randomDouble() > p) break;
            throughput = throughput / p;
        }
    }

    // Aerial-perspective fog: fade the shaded result toward the fog color with
    // primary-ray distance (sky rays already are the horizon color, so skip them).
    if (fog.enabled && fog.density > 0.0 && firstDist > 0.0) {
        double f = 1.0 - std::exp(-fog.density * firstDist);
        radiance = radiance * (1.0 - f) + fog.color * f;
    }

    return radiance;
}

}  // namespace engine

