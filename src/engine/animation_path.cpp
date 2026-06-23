#include "animation_path.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
double vlen(const Vec3& v) { return std::sqrt(v.lengthSquared()); }

// A rotation whose local +Z points along `fwd` and local +Y near `up`.
Quat lookRotation(const Vec3& fwd, const Vec3& upHint) {
    double fl = vlen(fwd);
    if (fl < 1e-9) return Quat::identity();
    Vec3 f = fwd * (1.0 / fl);
    Vec3 up = upHint;
    Vec3 right = cross(up, f);
    double rl = vlen(right);
    if (rl < 1e-6) {                       // up parallel to forward: pick another reference
        up = (std::fabs(f.y) < 0.9) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        right = cross(up, f);
        rl = vlen(right);
    }
    right = right * (1.0 / rl);
    up = cross(f, right);                  // re-orthonormalize
    Mat4 r = Mat4::identity();             // columns = world dirs of local X, Y, Z
    r.m[0][0] = right.x; r.m[1][0] = right.y; r.m[2][0] = right.z;
    r.m[0][1] = up.x;    r.m[1][1] = up.y;    r.m[2][1] = up.z;
    r.m[0][2] = f.x;     r.m[1][2] = f.y;     r.m[2][2] = f.z;
    return Quat::fromRotationMatrix(r);
}
}  // namespace

PathPose animationPose(const AnimationPath& path, double t) {
    PathPose pose;
    double phase = (path.duration > 1e-9) ? t / path.duration : 0.0;
    double u;
    switch (path.loop) {
        case LoopMode::Once:
            u = std::max(0.0, std::min(1.0, phase));
            break;
        case LoopMode::Loop:
            u = phase - std::floor(phase);            // wrap to [0,1)
            break;
        case LoopMode::PingPong: {
            double m = std::fmod(phase, 2.0);
            if (m < 0) m += 2.0;
            u = (m <= 1.0) ? m : 2.0 - m;             // triangle wave
            break;
        }
        default: u = 0;
    }
    pose.u = u;

    double L = path.curve.length();
    pose.position = path.curve.atDistance(u * L);
    pose.orientation = Quat::identity();
    if (path.faceTangent && L > 1e-6) {
        // Direction of travel from a short arc-length finite difference (so it matches
        // the constant-speed motion, not the uniform parameterization).
        double d = u * L, e = std::max(0.01, L * 0.001);
        Vec3 a = path.curve.atDistance(std::max(0.0, d - e));
        Vec3 b = path.curve.atDistance(std::min(L, d + e));
        pose.orientation = lookRotation(b - a, path.upHint);
    }
    return pose;
}

}  // namespace engine
