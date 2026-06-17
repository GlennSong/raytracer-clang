#include "curve.h"

namespace engine {

std::vector<CurveFrame> rotationMinimizingFrames(const std::vector<Vec3>& pts,
                                                 const std::vector<Vec3>& tangents) {
    std::vector<CurveFrame> frames(pts.size());
    if (pts.empty()) return frames;

    // Seed frame: any perpendicular to the first tangent.
    Vec3 t0 = normalize(tangents[0]);
    Vec3 ref = std::abs(t0.y) < 0.95 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 r0 = normalize(cross(ref, t0));
    Vec3 u0 = normalize(cross(t0, r0));
    frames[0] = {t0, r0, u0};

    // Double-reflection transport (Wang, Jüttler, Zheng & Liu 2008): reflect the
    // frame across the chord, then across the tangent change — minimal twist.
    for (size_t i = 0; i + 1 < pts.size(); i++) {
        Vec3 ti = frames[i].tangent;
        Vec3 ri = frames[i].right;
        Vec3 t1 = normalize(tangents[i + 1]);

        Vec3 v1 = pts[i + 1] - pts[i];
        double c1 = dot(v1, v1);
        if (c1 < 1e-12) {                       // coincident points: carry frame
            Vec3 up = normalize(cross(t1, ri));
            frames[i + 1] = {t1, normalize(cross(up, t1)), up};
            continue;
        }
        Vec3 rL = ri - v1 * (2.0 / c1 * dot(v1, ri));
        Vec3 tL = ti - v1 * (2.0 / c1 * dot(v1, ti));

        Vec3 v2 = t1 - tL;
        double c2 = dot(v2, v2);
        Vec3 r1 = c2 < 1e-12 ? rL : rL - v2 * (2.0 / c2 * dot(v2, rL));

        Vec3 up = normalize(cross(t1, r1));
        Vec3 right = normalize(cross(up, t1));   // re-orthonormalize
        frames[i + 1] = {t1, right, up};
    }
    return frames;
}

}  // namespace engine
