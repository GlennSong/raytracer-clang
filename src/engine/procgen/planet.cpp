#include "planet.h"

#include "noise.h"
#include "cellular.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace engine {

namespace {

inline double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

inline double smoothstep(double edge0, double edge1, double x) {
    if (edge0 == edge1) return x < edge0 ? 0.0 : 1.0;
    double t = clampd((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// 3D ridged multifractal: sum (1-|noise|)² octaves — the sharp irregular ridgelines
// terrain.cpp builds in 2D, lifted to the sphere's 3D sample direction.
double ridged3(const Noise& n, Vec3 p, double freq, int octaves) {
    double sum = 0.0, amp = 0.5, totalAmp = 0.0;
    p = p * freq;
    for (int o = 0; o < octaves; o++) {
        double v = n.noise3(p.x, p.y, p.z);
        v = 1.0 - std::fabs(v);
        v *= v;
        sum += v * amp;
        totalAmp += amp;
        p = p * 2.0;
        amp *= 0.5;
    }
    return totalAmp > 0.0 ? sum / totalAmp : 0.0;   // ~[0,1]
}

// The impact-crater field: read the nearest Worley feature as a crater centre,
// keep it with probability craterDensity (seeded by the cell id), size it in cell
// units, and apply the crater profile. Nearest-feature only (craters bounded below
// one cell), which is ample from orbit.
double craterField(const PlanetParams& p, uint32_t seed, const Vec3& dir) {
    WorleyResult w = worley3(seed, dir.x * p.craterFreq, dir.y * p.craterFreq,
                             dir.z * p.craterFreq);
    // Two independent per-crater randoms from the cell id.
    double keep = (w.cellId & 0xffff) / 65535.0;
    if (keep > p.craterDensity) return 0.0;
    double r01 = ((w.cellId >> 16) & 0xffff) / 65535.0;
    double radius = p.craterMinRadius + r01 * (p.craterMaxRadius - p.craterMinRadius);
    if (radius < 1e-4) return 0.0;
    double depthScale = 0.6 + 0.4 * r01;   // vary rim/bowl strength per crater
    return craterProfile(w.f1 / radius) * depthScale;
}

}  // namespace

double craterProfile(double t) {
    if (t >= 1.6) return 0.0;
    const double rimPos = 1.0, rimWidth = 0.16, rimHeight = 0.28;
    const double rr = t - rimPos;
    double rim = rimHeight * std::exp(-(rr * rr) / (2.0 * rimWidth * rimWidth));
    double h;
    if (t < 1.0) {
        // Parabolic bowl: -1 at the centre rising to 0 at the rim, plus the rim ridge.
        h = -(1.0 - t * t) + rim;
    } else {
        // Beyond the rim: the ejecta blanket decays outward.
        h = rim * std::exp(-(t - 1.0) * 2.5);
    }
    return h;
}

double planetHeight(const PlanetParams& p, uint32_t seed, const Vec3& dir) {
    Noise base(seed);
    Noise warp(seed + 101u);
    Noise mnt(seed + 202u);

    // Domain warp the sample direction for organic coastlines (not computery fbm).
    Vec3 d = dir;
    if (p.warpStrength != 0.0) {
        double wx = warp.fbm3(dir.x * 2.0 + 11.3, dir.y * 2.0, dir.z * 2.0, 3);
        double wy = warp.fbm3(dir.x * 2.0, dir.y * 2.0 + 47.1, dir.z * 2.0, 3);
        double wz = warp.fbm3(dir.x * 2.0, dir.y * 2.0, dir.z * 2.0 + 23.7, 3);
        d = normalize(dir + Vec3(wx, wy, wz) * p.warpStrength);
    }

    double cont = base.fbm3(d.x * p.continentFreq, d.y * p.continentFreq,
                            d.z * p.continentFreq, p.continentOctaves) * p.continentAmp;

    // Mountains ride the land only: mask by how far above sea the continents sit, so
    // ranges rise on land and the sea floor stays smooth.
    double landMask = smoothstep(p.seaLevel - 0.05, p.seaLevel + 0.25, cont);
    double mountains = (ridged3(mnt, d, p.mountainFreq, p.mountainOctaves) - 0.5) *
                       2.0 * p.mountainAmp * landMask;

    double craters = craterField(p, seed + 303u, dir) * p.craterAmp;

    return cont + mountains + craters;
}

Vec3 planetSurfaceColor(const PlanetParams& p, uint32_t seed, const Vec3& dir) {
    double h = planetHeight(p, seed, dir);
    Vec3 c;
    if (p.hasOcean && h < p.seaLevel) {
        // Water: bright coastal shelves fading to deep blue with depth, so seas read
        // as water rather than the lowest land tone. (generatePlanet flattens the
        // matching geometry to a single sea surface.)
        double depthN = clampd((p.seaLevel - h) / 0.15, 0.0, 1.0);
        Vec3 shallow = clampVec(p.oceanColor * 1.7, 0.0, 1.0);
        c = lerp(shallow, p.oceanColor, depthN);
    } else {
        // A grey fallback ramp so a bare PlanetParams still colours to something sane.
        ColorRamp ramp = p.landRamp;
        if (ramp.stops.empty()) {
            ramp = ColorRamp{{0.0, Vec3(0.32, 0.30, 0.28)}, {0.6, Vec3(0.55, 0.52, 0.48)},
                             {1.0, Vec3(0.80, 0.78, 0.75)}};
        }
        double elev = clampd((h - p.seaLevel) / std::max(1e-3, 1.0 - p.seaLevel), 0.0, 1.0);
        c = ramp.eval(elev);
    }
    if (p.polarCaps) {                          // ice caps over land AND frozen sea
        double capMask = smoothstep(p.capLatitude - 0.06, p.capLatitude + 0.06,
                                    std::fabs(dir.y));
        c = lerp(c, p.capColor, capMask);
    }
    return clampVec(c, 0.0, 1.0);
}

RenderMesh generatePlanet(const PlanetParams& p, uint32_t seed) {
    RenderMesh mesh = MeshBuilder::cubeSphere(p.radius, p.faceRes);

    const double amp = static_cast<double>(p.radius) * p.reliefFraction;
    for (Vertex& v : mesh.vertices) {
        Vec3 dir = normalize(v.position);
        double h = planetHeight(p, seed, dir);
        // Ocean worlds flatten the sea floor up to one water surface at sea level, so
        // continents read as land rising from an ocean, not uniform terrain. Colour
        // (land ramp or depth-tinted water) comes from planetSurfaceColor.
        double surfaceH = (p.hasOcean && h < p.seaLevel) ? p.seaLevel : h;
        v.position = dir * (static_cast<double>(p.radius) + amp * surfaceH);
        v.color = planetSurfaceColor(p, seed, dir);
    }

    // Radial displacement moved each welded vertex once, so the manifold stays
    // watertight; rebuild normals from the deformed geometry (engine clockwise-front).
    MeshBuilder::recomputeNormals(mesh);
    for (Vertex& v : mesh.vertices)
        if (v.normal.lengthSquared() < 1e-12) v.normal = normalize(v.position);
    return mesh;
}

RenderMesh generateOceanShell(const PlanetParams& p, uint32_t seed) {
    (void)seed;
    if (!p.hasOcean) return RenderMesh{};
    const double amp = static_cast<double>(p.radius) * p.reliefFraction;
    float seaRadius = static_cast<float>(static_cast<double>(p.radius) + amp * p.seaLevel);
    RenderMesh mesh = MeshBuilder::cubeSphere(seaRadius, std::max(24, p.faceRes / 2));
    for (Vertex& v : mesh.vertices) v.color = clampVec(p.oceanColor, 0.0, 1.0);
    return mesh;
}

// --------------------------------------------------------------------------
// Presets
// --------------------------------------------------------------------------

PlanetParams planetMars() {
    PlanetParams p;
    p.continentFreq = 1.1;
    p.mountainFreq = 2.6;
    p.mountainAmp = 0.9;            // Tharsis-scale relief
    p.craterFreq = 8.0;
    p.craterAmp = 0.55;
    p.craterDensity = 0.6;
    p.reliefFraction = 0.05;
    p.seaLevel = -0.15;            // no ocean; low datum so most of the disc is land
    p.hasOcean = false;
    p.polarCaps = true;
    p.capLatitude = 0.86;         // thin caps
    p.capColor = Vec3(0.92, 0.93, 0.95);
    p.landRamp = ColorRamp{
        {0.00, Vec3(0.35, 0.16, 0.10)},   // dark iron basin
        {0.35, Vec3(0.60, 0.28, 0.16)},   // rusty dust
        {0.65, Vec3(0.74, 0.44, 0.28)},   // lighter oxide
        {1.00, Vec3(0.86, 0.66, 0.48)},   // dusty highlands
    };
    return p;
}

PlanetParams planetMoon() {
    PlanetParams p;
    p.continentFreq = 1.4;
    p.continentAmp = 0.7;
    p.mountainFreq = 3.2;
    p.mountainAmp = 0.35;
    p.craterFreq = 6.0;
    p.craterAmp = 0.8;            // craters dominate
    p.craterDensity = 0.75;
    p.craterMinRadius = 0.18;
    p.craterMaxRadius = 0.72;
    p.reliefFraction = 0.045;
    p.seaLevel = -0.4;
    p.hasOcean = false;
    p.polarCaps = false;
    p.landRamp = ColorRamp{
        {0.00, Vec3(0.16, 0.16, 0.17)},   // mare basalt
        {0.55, Vec3(0.42, 0.42, 0.43)},
        {1.00, Vec3(0.72, 0.72, 0.73)},   // bright highlands
    };
    return p;
}

// --------------------------------------------------------------------------
// Gas giant (headless slice)
// --------------------------------------------------------------------------

namespace {

// Equirectangular (u, v) -> unit sphere direction, matching cubeSphere's UVs
// (u = 0.5 + atan2(z,x)/2π, v = 0.5 - asin(y)/π), so the baked texture lands right.
Vec3 equirectDir(double u, double v) {
    double lon = (u - 0.5) * 2.0 * PI;
    double lat = (0.5 - v) * PI;
    double cl = std::cos(lat);
    return Vec3(cl * std::cos(lon), std::sin(lat), cl * std::sin(lon));
}

}  // namespace

RenderMesh generateGasGiantMesh(const GasGiantParams& p) {
    return MeshBuilder::cubeSphere(p.radius, p.faceRes);
}

TextureData generateGasGiantTexture(const GasGiantParams& p, uint32_t seed) {
    ColorRamp ramp = p.bandRamp;
    if (ramp.stops.empty()) {
        ramp = ColorRamp{{0.0, Vec3(0.35, 0.32, 0.30)}, {0.5, Vec3(0.62, 0.58, 0.52)},
                         {1.0, Vec3(0.88, 0.85, 0.80)}};
    }

    Noise flow(seed + 11u);       // domain warp of the band latitude
    Noise detail(seed + 29u);     // fine cloud turbulence

    // Storm centres + per-storm tint, placed deterministically on the sphere.
    std::mt19937 gen(seed + 991u);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    struct Storm { Vec3 dir; double size; double tint; };
    std::vector<Storm> storms;
    for (int s = 0; s < p.stormCount; s++) {
        double lat = (uni(gen) - 0.5) * 1.4;             // avoid the very poles
        double lon = uni(gen) * 2.0 * PI;
        double cl = std::cos(lat);
        storms.push_back({Vec3(cl * std::cos(lon), std::sin(lat), cl * std::sin(lon)),
                          p.stormSize * (0.6 + 0.8 * uni(gen)), 0.5 + 0.5 * uni(gen)});
    }

    const int W = std::max(2, p.texWidth), H = std::max(2, p.texHeight);
    TextureData tex;
    tex.width = W;
    tex.height = H;
    tex.channels = 3;
    tex.pixels.resize(static_cast<size_t>(W) * H * 3);

    for (int j = 0; j < H; j++) {
        double v = (j + 0.5) / H;
        for (int i = 0; i < W; i++) {
            double u = (i + 0.5) / W;
            Vec3 dir = equirectDir(u, v);

            // Warp the band latitude by turbulent flow (sampled in 3D → seamless in
            // longitude), then read alternating belts/zones out of a sine.
            double warp = flow.fbm3(dir.x * 1.5, dir.y * 1.5, dir.z * 1.5, p.warpOctaves);
            double bandCoord = dir.y + warp * p.flowWarp;
            double phase = 0.5 + 0.5 * std::sin(bandCoord * p.bandFreq * PI);
            // Fine turbulence breaks the bands up.
            double d = detail.fbm3(dir.x * p.detailFreq, dir.y * p.detailFreq,
                                   dir.z * p.detailFreq, 4);
            double t = clampd(phase + d * p.detailAmp, 0.0, 1.0);
            Vec3 c = ramp.eval(t);

            // Vortex storms: a smooth angular falloff toward the storm tint.
            for (const Storm& st : storms) {
                double ang = std::acos(clampd(dot(dir, st.dir), -1.0, 1.0));  // radians
                double reach = st.size * PI;
                if (ang < reach) {
                    double m = smoothstep(reach, reach * 0.2, ang) * st.tint;
                    c = lerp(c, p.stormColor, m);
                }
            }

            c = clampVec(c, 0.0, 1.0);
            size_t idx = (static_cast<size_t>(j) * W + i) * 3;
            tex.pixels[idx + 0] = static_cast<uint8_t>(c.x * 255.0 + 0.5);
            tex.pixels[idx + 1] = static_cast<uint8_t>(c.y * 255.0 + 0.5);
            tex.pixels[idx + 2] = static_cast<uint8_t>(c.z * 255.0 + 0.5);
        }
    }
    return tex;
}

GasGiantParams gasGiantJupiter() {
    GasGiantParams p;
    p.bandFreq = 13.0;
    p.flowWarp = 0.4;
    p.detailFreq = 6.0;
    p.detailAmp = 0.18;
    p.stormCount = 3;
    p.stormSize = 0.14;
    p.stormColor = Vec3(0.72, 0.34, 0.22);   // the Great Red Spot family
    p.bandRamp = ColorRamp{
        {0.00, Vec3(0.45, 0.30, 0.20)},   // dark belt
        {0.30, Vec3(0.72, 0.56, 0.40)},   // tan
        {0.60, Vec3(0.86, 0.78, 0.64)},   // cream zone
        {1.00, Vec3(0.94, 0.90, 0.82)},   // bright zone
    };
    return p;
}

GasGiantParams gasGiantNeptune() {
    GasGiantParams p;
    p.bandFreq = 8.0;
    p.flowWarp = 0.3;
    p.detailFreq = 4.0;
    p.detailAmp = 0.12;
    p.stormCount = 2;
    p.stormSize = 0.1;
    p.stormColor = Vec3(0.10, 0.14, 0.30);   // dark storm spot
    p.bandRamp = ColorRamp{
        {0.00, Vec3(0.12, 0.24, 0.52)},   // deep blue belt
        {0.50, Vec3(0.22, 0.40, 0.70)},   // mid blue
        {1.00, Vec3(0.44, 0.62, 0.86)},   // light azure zone
    };
    return p;
}

PlanetParams planetEarthlike() {
    PlanetParams p;
    p.continentFreq = 1.5;
    p.continentAmp = 1.0;
    p.mountainFreq = 3.4;
    p.mountainAmp = 0.7;
    p.craterFreq = 9.0;
    p.craterAmp = 0.05;          // barely any craters (weathered/active world)
    p.craterDensity = 0.15;
    p.reliefFraction = 0.035;
    p.seaLevel = 0.02;
    p.hasOcean = true;
    p.oceanColor = Vec3(0.04, 0.12, 0.26);
    p.polarCaps = true;
    p.capLatitude = 0.74;
    p.capColor = Vec3(0.95, 0.96, 0.98);
    p.landRamp = ColorRamp{
        {0.00, Vec3(0.20, 0.34, 0.16)},   // coastal green
        {0.30, Vec3(0.26, 0.40, 0.18)},   // forest
        {0.55, Vec3(0.45, 0.42, 0.26)},   // arid highland
        {0.80, Vec3(0.42, 0.33, 0.24)},   // rock
        {1.00, Vec3(0.72, 0.70, 0.66)},   // bare peak
    };
    return p;
}

}  // namespace engine
