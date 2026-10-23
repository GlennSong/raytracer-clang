#ifndef RAYTRACER_ENGINE_VERTEX_PACK_H
#define RAYTRACER_ENGINE_VERTEX_PACK_H

// The standard vertex's compact encodings (ADR-0096), shared by the Vulkan upload and the bundle's compact
// meshes (ADR-0224), so what a bake stores is exactly what the GPU is given:
// - a unit vector as an octahedral snorm16 pair (~0.003 deg); octDecode is the inverse in mesh.vert;
// - a tint in [0, 1] as unorm8.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {

// Unit vector -> octahedral snorm16 pair. A zero vector encodes as (0, 0).
inline void octEncode(double x, double y, double z, int16_t out[2]) {
    const double l1 = std::fabs(x) + std::fabs(y) + std::fabs(z);
    double u = 0.0, v = 0.0;
    if (l1 > 1e-30) {
        u = x / l1; v = y / l1;
        if (z < 0.0) {
            const double fu = (1.0 - std::fabs(v)) * (u >= 0.0 ? 1.0 : -1.0);
            const double fv = (1.0 - std::fabs(u)) * (v >= 0.0 ? 1.0 : -1.0);
            u = fu; v = fv;
        }
    }
    auto snorm16 = [](double c) {
        const double s = std::clamp(c, -1.0, 1.0) * 32767.0;
        return static_cast<int16_t>(s >= 0.0 ? s + 0.5 : s - 0.5);   // round half away from zero
    };
    out[0] = snorm16(u);
    out[1] = snorm16(v);
}

// The pair back to a unit vector (mesh.vert's octDecode, in double). (0, 0) decodes to +z.
inline void octDecode(const int16_t in[2], double& x, double& y, double& z) {
    double u = std::max(in[0] / 32767.0, -1.0), v = std::max(in[1] / 32767.0, -1.0);
    z = 1.0 - std::fabs(u) - std::fabs(v);
    if (z < 0.0) {
        const double fu = (1.0 - std::fabs(v)) * (u >= 0.0 ? 1.0 : -1.0);
        const double fv = (1.0 - std::fabs(u)) * (v >= 0.0 ? 1.0 : -1.0);
        u = fu; v = fv;
    }
    x = u; y = v;
    const double l = std::sqrt(x * x + y * y + z * z);
    x /= l; y /= l; z /= l;
}

inline uint8_t unorm8(double c) { return static_cast<uint8_t>(std::clamp(c, 0.0, 1.0) * 255.0 + 0.5); }

}  // namespace engine

#endif
