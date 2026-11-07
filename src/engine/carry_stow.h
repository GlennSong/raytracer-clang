#ifndef RAYTRACER_ENGINE_CARRY_STOW_H
#define RAYTRACER_ENGINE_CARRY_STOW_H

// WHERE A THING GOES IN A CAR'S CARGO SPACE (ADR-0358): pure, so the tests can hold it. The space is a box in the
// chassis frame; what is already in it are boxes too (their middles and half extents, chassis-aligned). A new thing
// is tried flat first -- its thinnest side down, a door laid on its face -- then on its other sides, each also turned
// a quarter about the vertical; the first turn it fits in wins, packed from a corner: on the floor before on top of
// what is there, the row nearest `near` (where the player stands, in the chassis frame) first, across from one side.

#include "../rt_math.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

struct StowBox {
    Vec3 centre, half;
};

// The thing's half extents once turned by `q` (the box round it, chassis-aligned).
inline Vec3 turnedHalf(const Quat& q, const Vec3& h) {
    const Vec3 ax = q.rotate(Vec3(1, 0, 0)), ay = q.rotate(Vec3(0, 1, 0)), az = q.rotate(Vec3(0, 0, 1));
    return Vec3(std::fabs(ax.x) * h.x + std::fabs(ay.x) * h.y + std::fabs(az.x) * h.z,
                std::fabs(ax.y) * h.x + std::fabs(ay.y) * h.y + std::fabs(az.y) * h.z,
                std::fabs(ax.z) * h.x + std::fabs(ay.z) * h.y + std::fabs(az.z) * h.z);
}

// True with `at` (its middle) and `turn` when the thing of half extents `half` fits in [lo, hi] beside `taken`.
inline bool stowSlot(const Vec3& lo, const Vec3& hi, const Vec3& half, const std::vector<StowBox>& taken, const Vec3& near,
                     Vec3& at, Quat& turn) {
    const Real kPi = 3.14159265358979323846;
    const Vec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
    // the turns, flattest first: the axis the thing is thinnest along stood up, then the others
    Quat lay[3] = {Quat(), Quat::fromAxisAngle(Z, kPi * 0.5), Quat::fromAxisAngle(X, kPi * 0.5)};   // y, x, z up
    const Real up[3] = {half.y, half.x, half.z};
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int a, int b) { return up[a] < up[b]; });
    constexpr Real gap = 0.01, step = 0.05;
    for (const int o : order)
        for (const Real spin : {Real(0), kPi * 0.5}) {
            const Quat q = Quat::fromAxisAngle(Y, spin) * lay[o];
            const Vec3 h = turnedHalf(q, half);
            if (2 * h.x > hi.x - lo.x + 1e-6 || 2 * h.y > hi.y - lo.y + 1e-6 || 2 * h.z > hi.z - lo.z + 1e-6) continue;
            bool found = false;
            Real best = 1e30;
            Vec3 bestAt;
            for (Real x = lo.x + h.x; x <= hi.x - h.x + 1e-6; x += step)
                for (Real z = lo.z + h.z; z <= hi.z - h.z + 1e-6; z += step) {
                    // the floor here: the space's, or the top of whatever is under this footprint
                    Real y0 = lo.y;
                    for (const StowBox& b : taken)
                        if (std::fabs(x - b.centre.x) < h.x + b.half.x - gap && std::fabs(z - b.centre.z) < h.z + b.half.z - gap)
                            y0 = std::max(y0, b.centre.y + b.half.y);
                    const Real y = y0 + h.y + gap;
                    if (y + h.y > hi.y + 1e-6) continue;
                    // packed from a corner: the row nearest the player first, across it from one side, the floor
                    // before a stack (a box centred in the boot leaves no room for one either side of it)
                    const Real d = std::fabs(z - near.z) * 10.0 + (x - lo.x) + (y - lo.y) * 40.0;
                    if (d < best) { best = d; bestAt = Vec3(x, y, z); found = true; }
                }
            if (found) {
                at = bestAt;
                turn = q;
                return true;
            }
        }
    return false;
}

}  // namespace engine

#endif
