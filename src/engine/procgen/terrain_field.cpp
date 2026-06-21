#include "terrain_field.h"

#include "noise.h"
#include "erosion.h"
#include "../mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <memory>

namespace engine {

HeightField heightConstant(double h) {
    return [h](double, double) { return h; };
}

HeightField heightNoise(uint32_t seed, double freq, double amp) {
    auto n = std::make_shared<Noise>(seed);
    return [n, freq, amp](double x, double z) {
        return n->noise2(x * freq, z * freq) * amp;
    };
}

HeightField heightFbm(uint32_t seed, double freq, double amp, int octaves) {
    auto n = std::make_shared<Noise>(seed);
    int oc = std::max(1, octaves);
    return [n, freq, amp, oc](double x, double z) {
        return n->fbm2(x * freq, z * freq, oc) * amp;
    };
}

HeightField heightRidged(uint32_t seed, double freq, double amp, int octaves) {
    auto n = std::make_shared<Noise>(seed);
    int oc = std::max(1, octaves);
    return [n, freq, amp, oc](double x, double z) {
        double sum = 0, f = freq, a = amp;
        for (int i = 0; i < oc; ++i) {
            double v = 1.0 - std::fabs(n->noise2(x * f, z * f));
            sum += v * v * a;
            f *= 2.0; a *= 0.5;
        }
        return sum;
    };
}

HeightField heightWarp(HeightField base, HeightField by, double strength) {
    return [base, by, strength](double x, double z) {
        double ox = by(x, z) * strength;
        double oz = by(x + 5.2, z - 1.3) * strength;   // decorrelated offset
        return base(x + ox, z + oz);
    };
}

HeightField heightTerrace(HeightField base, double step) {
    double s = step > 1e-6 ? step : 1.0;
    return [base, s](double x, double z) {
        return std::round(base(x, z) / s) * s;
    };
}

HeightField heightAdd(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return a(x, z) + b(x, z); };
}
HeightField heightMul(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return a(x, z) * b(x, z); };
}
HeightField heightScale(HeightField a, double s) {
    return [a, s](double x, double z) { return a(x, z) * s; };
}
HeightField heightMax(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return std::max(a(x, z), b(x, z)); };
}
HeightField heightMin(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return std::min(a(x, z), b(x, z)); };
}
HeightField heightMix(HeightField a, HeightField b, double t) {
    return [a, b, t](double x, double z) {
        double u = a(x, z), v = b(x, z);
        return u + (v - u) * t;
    };
}
HeightField heightClamp(HeightField a, double lo, double hi) {
    return [a, lo, hi](double x, double z) {
        double v = a(x, z);
        return v < lo ? lo : (v > hi ? hi : v);
    };
}

HeightField erodeField(const HeightField& f, double worldSize, int resolution,
                       const ErosionParams& params) {
    int n = std::max(2, resolution) + 1;
    auto hm = std::make_shared<Heightmap>();
    hm->n = n;
    hm->worldSize = static_cast<float>(worldSize);
    hm->h.resize(static_cast<std::size_t>(n) * n);
    double half = worldSize * 0.5;
    double step = worldSize / (n - 1);
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x)            // same index convention as bakeHeightmap
            hm->set(x, z, static_cast<float>(f(-half + x * step, -half + z * step)));
    erode(*hm, params);
    return [hm](double x, double z) {
        return static_cast<double>(
            hm->sampleWorld(static_cast<float>(x), static_cast<float>(z)));
    };
}

RenderMesh bakeHeightMesh(const HeightField& h, double size, int resolution,
                          const Vec3& color) {
    RenderMesh mesh;
    int res = std::max(1, resolution);
    int n = res + 1;
    double half = size * 0.5;
    double cell = size / res;
    double e = cell;   // finite-difference step for normals

    mesh.vertices.reserve(static_cast<std::size_t>(n) * n);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            double x = -half + i * cell;
            double z = -half + j * cell;
            double y = h(x, z);
            // Normal from the height gradient (central differences).
            double hl = h(x - e, z), hr = h(x + e, z);
            double hd = h(x, z - e), hu = h(x, z + e);
            Vec3 nrm = normalize(Vec3(hl - hr, 2.0 * e, hd - hu));
            Vec3 tan = normalize(Vec3(2.0 * e, hr - hl, 0.0));
            Vertex v(Vec3(x, y, z), nrm, tan, (x + half) / size, (z + half) / size);
            v.color = color;
            mesh.vertices.push_back(v);
        }
    }
    // Clockwise-front winding for an up-facing grid (the engine's convention —
    // see MeshBuilder::gridIndices). The previous hand-rolled order here was
    // inverted, so the terrain top was culled as a back face in the viewer.
    MeshBuilder::gridIndices(mesh, n, n);
    return mesh;
}

}  // namespace engine
