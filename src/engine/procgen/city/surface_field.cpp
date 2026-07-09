#include "surface_field.h"

#include <cmath>

namespace engine {

Vec3 SurfaceField::normal(double x, double z, double eps) const {
    // Central differences on the authored height: the gradient of the surface,
    // matching terrainNormal's convention (y up, cross of the tangents).
    double hL = height(x - eps, z), hR = height(x + eps, z);
    double hD = height(x, z - eps), hU = height(x, z + eps);
    Vec3 n(static_cast<Real>(hL - hR), static_cast<Real>(2.0 * eps),
           static_cast<Real>(hD - hU));
    Real len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    if (len > 1e-9f) { n.x /= len; n.y /= len; n.z /= len; }
    return n;
}

}  // namespace engine
