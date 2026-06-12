#ifndef RAYTRACER_MATH_H
#define RAYTRACER_MATH_H

#include <cmath>
#include <algorithm>
#include <random>

namespace engine {

// Scalar precision for all engine math. Centralized here so float/double is a
// single switch point (see docs/decisions.md, ADR-0005). Defaults to double.
using Real = double;

struct Vec3 {
    Real x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(Real x, Real y, Real z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(Real t) const { return {x * t, y * t, z * t}; }
    Vec3 operator/(Real t) const { return {x / t, y / t, z / t}; }
    Vec3 operator*(const Vec3& v) const { return {x * v.x, y * v.y, z * v.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(Real t) { x *= t; y *= t; z *= t; return *this; }
    Vec3& operator/=(Real t) { x /= t; y /= t; z /= t; return *this; }

    Real length() const { return std::sqrt(x * x + y * y + z * z); }
    Real lengthSquared() const { return x * x + y * y + z * z; }
};

inline Vec3 operator*(Real t, const Vec3& v) { return v * t; }

inline Real dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline Vec3 normalize(const Vec3& v) {
    Real len = v.length();
    if (len > 0) return v / len;
    return v;
}

inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0 * dot(v, n) * n;
}

inline Vec3 refract(const Vec3& uv, const Vec3& n, Real etaRatio) {
    Real cosTheta = std::min(dot(-uv, n), Real(1.0));
    Vec3 perpendicular = etaRatio * (uv + cosTheta * n);
    Vec3 parallel = -std::sqrt(std::abs(1.0 - perpendicular.lengthSquared())) * n;
    return perpendicular + parallel;
}

inline Real schlick(Real cosine, Real ior) {
    Real r0 = (1.0 - ior) / (1.0 + ior);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - cosine, 5.0);
}

inline Vec3 clampVec(const Vec3& v, Real lo, Real hi) {
    return {
        std::clamp(v.x, lo, hi),
        std::clamp(v.y, lo, hi),
        std::clamp(v.z, lo, hi)
    };
}

inline Vec3 lerp(const Vec3& a, const Vec3& b, Real t) {
    return a + (b - a) * t;
}

inline Real distance(const Vec3& a, const Vec3& b) { return (a - b).length(); }
inline Real distanceSquared(const Vec3& a, const Vec3& b) {
    return (a - b).lengthSquared();
}

inline Vec3 minVec(const Vec3& a, const Vec3& b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 maxVec(const Vec3& a, const Vec3& b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

inline bool approxEqual(Real a, Real b, Real eps = 1e-6) {
    return std::abs(a - b) <= eps;
}
inline bool approxEqual(const Vec3& a, const Vec3& b, Real eps = 1e-6) {
    return approxEqual(a.x, b.x, eps) && approxEqual(a.y, b.y, eps) &&
           approxEqual(a.z, b.z, eps);
}

struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray() {}
    Ray(const Vec3& origin, const Vec3& direction)
        : origin(origin), direction(direction) {}

    Vec3 at(Real t) const { return origin + direction * t; }
};

struct Quat;

struct Mat4 {
    Real m[4][4];

    Mat4() {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = (i == j) ? 1.0 : 0.0;
    }

    static Mat4 identity() { return Mat4(); }

    static Mat4 translate(Real tx, Real ty, Real tz) {
        Mat4 result;
        result.m[0][3] = tx;
        result.m[1][3] = ty;
        result.m[2][3] = tz;
        return result;
    }

    static Mat4 scale(Real sx, Real sy, Real sz) {
        Mat4 result;
        result.m[0][0] = sx;
        result.m[1][1] = sy;
        result.m[2][2] = sz;
        return result;
    }

    static Mat4 rotateX(Real radians) {
        Mat4 result;
        Real c = std::cos(radians);
        Real s = std::sin(radians);
        result.m[1][1] = c;  result.m[1][2] = -s;
        result.m[2][1] = s;  result.m[2][2] = c;
        return result;
    }

    static Mat4 rotateY(Real radians) {
        Mat4 result;
        Real c = std::cos(radians);
        Real s = std::sin(radians);
        result.m[0][0] = c;  result.m[0][2] = s;
        result.m[2][0] = -s; result.m[2][2] = c;
        return result;
    }

    static Mat4 rotateZ(Real radians) {
        Mat4 result;
        Real c = std::cos(radians);
        Real s = std::sin(radians);
        result.m[0][0] = c;  result.m[0][1] = -s;
        result.m[1][0] = s;  result.m[1][1] = c;
        return result;
    }

    // Right-handed perspective projection mapping clip-space depth to [0, 1]
    // (Metal/Vulkan/D3D convention), with the camera looking down -z. Built
    // engine-side so the math is backend-neutral and testable (see ADR-0009);
    // the backend transposes to its column-major layout when uploading.
    static Mat4 perspective(Real fovYRadians, Real aspect, Real nearPlane, Real farPlane) {
        Real yScale = 1.0 / std::tan(fovYRadians * 0.5);
        Real xScale = yScale / aspect;
        Real zRange = farPlane - nearPlane;

        Mat4 result;
        result.m[0][0] = xScale;
        result.m[1][1] = yScale;
        result.m[2][2] = -farPlane / zRange;
        result.m[2][3] = -(farPlane * nearPlane) / zRange;
        result.m[3][2] = -1.0;
        result.m[3][3] = 0.0;
        // Off-diagonal entries left from identity construction are all zero
        // except the ones set above; the remaining identity 1s we override here.
        return result;
    }

    // Orthographic projection (centered, given full height and aspect) mapping
    // clip-space depth to [0, 1]. Same conventions as perspective().
    static Mat4 orthographic(Real height, Real aspect, Real nearPlane, Real farPlane) {
        Real h = height;
        Real w = height * aspect;
        Real zRange = farPlane - nearPlane;

        Mat4 result;
        result.m[0][0] = 2.0 / w;
        result.m[1][1] = 2.0 / h;
        result.m[2][2] = -1.0 / zRange;
        result.m[2][3] = -nearPlane / zRange;
        return result;
    }

    // Right-handed view matrix (world -> view), camera at eye looking toward
    // center; matches the renderer convention of looking down -z. Built engine-
    // side so it is backend-neutral and testable (companion to the projection
    // helpers; see ADR-0009).
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = normalize(center - eye);
        Vec3 s = normalize(cross(f, up));
        Vec3 u = cross(s, f);

        Mat4 m;
        m.m[0][0] = s.x;  m.m[0][1] = s.y;  m.m[0][2] = s.z;  m.m[0][3] = -dot(s, eye);
        m.m[1][0] = u.x;  m.m[1][1] = u.y;  m.m[1][2] = u.z;  m.m[1][3] = -dot(u, eye);
        m.m[2][0] = -f.x; m.m[2][1] = -f.y; m.m[2][2] = -f.z; m.m[2][3] = dot(f, eye);
        return m;
    }

    // Compose translation * rotation * scale into a model matrix. Defined after
    // Quat below.
    static Mat4 trs(const Vec3& translation, const Quat& rotation,
                    const Vec3& scale);

    Mat4 operator*(const Mat4& b) const {
        Mat4 result;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++)
                    result.m[i][j] += m[i][k] * b.m[k][j];
            }
        return result;
    }

    Vec3 transformPoint(const Vec3& p) const {
        Real w = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];
        return {
            (m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3]) / w,
            (m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3]) / w,
            (m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3]) / w
        };
    }

    Vec3 transformDirection(const Vec3& d) const {
        return {
            m[0][0] * d.x + m[0][1] * d.y + m[0][2] * d.z,
            m[1][0] * d.x + m[1][1] * d.y + m[1][2] * d.z,
            m[2][0] * d.x + m[2][1] * d.y + m[2][2] * d.z
        };
    }

    Mat4 transpose() const {
        Mat4 r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r.m[i][j] = m[j][i];
        return r;
    }

    // 4×4 matrix inverse via cofactor expansion. Returns identity if singular.
    Mat4 inverse() const {
        // Compute cofactors for each element of the result
        Real c[16]; // cofactors laid out row-major
        const Real* s = &m[0][0];

        c[0]  =  s[5]*(s[10]*s[15]-s[11]*s[14]) - s[9]*(s[6]*s[15]-s[7]*s[14]) + s[13]*(s[6]*s[11]-s[7]*s[10]);
        c[1]  = -(s[4]*(s[10]*s[15]-s[11]*s[14]) - s[8]*(s[6]*s[15]-s[7]*s[14]) + s[12]*(s[6]*s[11]-s[7]*s[10]));
        c[2]  =  s[4]*(s[9]*s[15]-s[11]*s[13]) - s[8]*(s[5]*s[15]-s[7]*s[13]) + s[12]*(s[5]*s[11]-s[7]*s[9]);
        c[3]  = -(s[4]*(s[9]*s[14]-s[10]*s[13]) - s[8]*(s[5]*s[14]-s[6]*s[13]) + s[12]*(s[5]*s[10]-s[6]*s[9]));

        c[4]  = -(s[1]*(s[10]*s[15]-s[11]*s[14]) - s[9]*(s[2]*s[15]-s[3]*s[14]) + s[13]*(s[2]*s[11]-s[3]*s[10]));
        c[5]  =  s[0]*(s[10]*s[15]-s[11]*s[14]) - s[8]*(s[2]*s[15]-s[3]*s[14]) + s[12]*(s[2]*s[11]-s[3]*s[10]);
        c[6]  = -(s[0]*(s[9]*s[15]-s[11]*s[13]) - s[8]*(s[1]*s[15]-s[3]*s[13]) + s[12]*(s[1]*s[11]-s[3]*s[9]));
        c[7]  =  s[0]*(s[9]*s[14]-s[10]*s[13]) - s[8]*(s[1]*s[14]-s[2]*s[13]) + s[12]*(s[1]*s[10]-s[2]*s[9]);

        c[8]  =  s[1]*(s[6]*s[15]-s[7]*s[14]) - s[5]*(s[2]*s[15]-s[3]*s[14]) + s[13]*(s[2]*s[7]-s[3]*s[6]);
        c[9]  = -(s[0]*(s[6]*s[15]-s[7]*s[14]) - s[4]*(s[2]*s[15]-s[3]*s[14]) + s[12]*(s[2]*s[7]-s[3]*s[6]));
        c[10] =  s[0]*(s[5]*s[15]-s[7]*s[13]) - s[4]*(s[1]*s[15]-s[3]*s[13]) + s[12]*(s[1]*s[7]-s[3]*s[5]);
        c[11] = -(s[0]*(s[5]*s[14]-s[6]*s[13]) - s[4]*(s[1]*s[14]-s[2]*s[13]) + s[12]*(s[1]*s[6]-s[2]*s[5]));

        c[12] = -(s[1]*(s[6]*s[11]-s[7]*s[10]) - s[5]*(s[2]*s[11]-s[3]*s[10]) + s[9]*(s[2]*s[7]-s[3]*s[6]));
        c[13] =  s[0]*(s[6]*s[11]-s[7]*s[10]) - s[4]*(s[2]*s[11]-s[3]*s[10]) + s[8]*(s[2]*s[7]-s[3]*s[6]);
        c[14] = -(s[0]*(s[5]*s[11]-s[7]*s[9]) - s[4]*(s[1]*s[11]-s[3]*s[9]) + s[8]*(s[1]*s[7]-s[3]*s[5]));
        c[15] =  s[0]*(s[5]*s[10]-s[6]*s[9]) - s[4]*(s[1]*s[10]-s[2]*s[9]) + s[8]*(s[1]*s[6]-s[2]*s[5]);

        Real det = s[0]*c[0] + s[1]*c[1] + s[2]*c[2] + s[3]*c[3];
        if (std::abs(det) < 1e-12) return Mat4(); // singular → identity

        Real invDet = 1.0 / det;
        Mat4 r;
        // Transpose of cofactor matrix divided by determinant
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r.m[j][i] = c[i * 4 + j] * invDet;
        return r;
    }
};

// Unit quaternion for rotations. Stored (x, y, z, w) with w the scalar part;
// identity is (0, 0, 0, 1). Preferred over Euler angles for orientation: no
// gimbal lock, clean composition, and slerp for wobble-free interpolation (the
// limitation ADR-0006 flagged). Bridges cleanly to physics quaternions.
struct Quat {
    Real x, y, z, w;

    Quat() : x(0), y(0), z(0), w(1) {}
    Quat(Real x, Real y, Real z, Real w) : x(x), y(y), z(z), w(w) {}

    static Quat identity() { return Quat(); }

    static Quat fromAxisAngle(const Vec3& axis, Real radians) {
        Vec3 n = normalize(axis);
        Real half = radians * 0.5;
        Real s = std::sin(half);
        return Quat(n.x * s, n.y * s, n.z * s, std::cos(half));
    }

    // Orientation from the (orthonormal) upper-3x3 of a column-vector-
    // convention matrix — e.g. a gizmo-manipulated transform with the scale
    // divided out. Shepperd's method: pick the largest diagonal pivot for
    // numerical stability.
    static Quat fromRotationMatrix(const Mat4& m) {
        Real trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        Quat q;
        if (trace > 0.0) {
            Real s = std::sqrt(trace + 1.0) * 2.0;
            q = Quat((m.m[2][1] - m.m[1][2]) / s, (m.m[0][2] - m.m[2][0]) / s,
                     (m.m[1][0] - m.m[0][1]) / s, 0.25 * s);
        } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
            Real s = std::sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0;
            q = Quat(0.25 * s, (m.m[0][1] + m.m[1][0]) / s,
                     (m.m[0][2] + m.m[2][0]) / s, (m.m[2][1] - m.m[1][2]) / s);
        } else if (m.m[1][1] > m.m[2][2]) {
            Real s = std::sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0;
            q = Quat((m.m[0][1] + m.m[1][0]) / s, 0.25 * s,
                     (m.m[1][2] + m.m[2][1]) / s, (m.m[0][2] - m.m[2][0]) / s);
        } else {
            Real s = std::sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0;
            q = Quat((m.m[0][2] + m.m[2][0]) / s, (m.m[1][2] + m.m[2][1]) / s,
                     0.25 * s, (m.m[1][0] - m.m[0][1]) / s);
        }
        return q.normalized();
    }

    // Inverse of fromAxisAngle (for serialization). Identity yields the +Y
    // axis with angle 0 so the output is always a valid axis.
    void toAxisAngle(Vec3& axis, Real& radians) const {
        Quat q = normalized();
        Real s = std::sqrt(std::max(1.0 - q.w * q.w, 0.0));
        if (s < 1e-9) {
            axis = Vec3(0, 1, 0);
            radians = 0.0;
            return;
        }
        axis = Vec3(q.x / s, q.y / s, q.z / s);
        radians = 2.0 * std::acos(std::clamp(q.w, Real(-1.0), Real(1.0)));
    }

    // Euler radians applied X then Y then Z, matching the old Transform matrix
    // order (rotateZ * rotateY * rotateX), so existing poses round-trip.
    static Quat fromEuler(Real ex, Real ey, Real ez) {
        return fromAxisAngle(Vec3(0, 0, 1), ez) *
               fromAxisAngle(Vec3(0, 1, 0), ey) *
               fromAxisAngle(Vec3(1, 0, 0), ex);
    }
    static Quat fromEuler(const Vec3& e) { return fromEuler(e.x, e.y, e.z); }

    // Hamilton product: composing rotations (this applied after b on a vector).
    Quat operator*(const Quat& b) const {
        return Quat(
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w,
            w * b.w - x * b.x - y * b.y - z * b.z);
    }

    Real dot(const Quat& b) const { return x * b.x + y * b.y + z * b.z + w * b.w; }
    Real length() const { return std::sqrt(x * x + y * y + z * z + w * w); }

    Quat normalized() const {
        Real len = length();
        if (len > 0) return Quat(x / len, y / len, z / len, w / len);
        return identity();
    }

    Quat conjugate() const { return Quat(-x, -y, -z, w); }

    Quat inverse() const {
        Real lenSq = x * x + y * y + z * z + w * w;
        if (lenSq <= 0) return identity();
        return Quat(-x / lenSq, -y / lenSq, -z / lenSq, w / lenSq);
    }

    // Rotate a vector by this (assumed unit) quaternion.
    Vec3 rotate(const Vec3& v) const {
        Vec3 u(x, y, z);
        return u * (2.0 * dot3(u, v)) + v * (w * w - dot3(u, u)) +
               cross(u, v) * (2.0 * w);
    }

    Vec3 toEuler() const;
    Mat4 toMat4() const;

    // Shortest-arc spherical interpolation, robust as the inputs become parallel.
    static Quat slerp(const Quat& a, Quat b, Real t) {
        Real d = a.dot(b);
        if (d < 0) {                       // take the shorter path
            b = Quat(-b.x, -b.y, -b.z, -b.w);
            d = -d;
        }
        if (d > 0.9995) {                  // nearly identical: lerp + normalize
            return Quat(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                        a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t).normalized();
        }
        Real theta0 = std::acos(d);
        Real theta = theta0 * t;
        Real sinTheta0 = std::sin(theta0);
        Real s1 = std::sin(theta) / sinTheta0;
        Real s0 = std::cos(theta) - d * s1;
        return Quat(a.x * s0 + b.x * s1, a.y * s0 + b.y * s1,
                    a.z * s0 + b.z * s1, a.w * s0 + b.w * s1);
    }

private:
    static Real dot3(const Vec3& a, const Vec3& b) { return engine::dot(a, b); }
};

inline Mat4 Quat::toMat4() const {
    Real xx = x * x, yy = y * y, zz = z * z;
    Real xy = x * y, xz = x * z, yz = y * z;
    Real wx = w * x, wy = w * y, wz = w * z;

    Mat4 m;  // identity; fill the rotation 3x3 (column-vector convention)
    m.m[0][0] = 1 - 2 * (yy + zz); m.m[0][1] = 2 * (xy - wz);     m.m[0][2] = 2 * (xz + wy);
    m.m[1][0] = 2 * (xy + wz);     m.m[1][1] = 1 - 2 * (xx + zz); m.m[1][2] = 2 * (yz - wx);
    m.m[2][0] = 2 * (xz - wy);     m.m[2][1] = 2 * (yz + wx);     m.m[2][2] = 1 - 2 * (xx + yy);
    return m;
}

inline Vec3 Quat::toEuler() const {
    Mat4 r = toMat4();
    Real ex = std::atan2(r.m[2][1], r.m[2][2]);
    Real ey = std::asin(std::clamp(-r.m[2][0], Real(-1), Real(1)));
    Real ez = std::atan2(r.m[1][0], r.m[0][0]);
    return Vec3(ex, ey, ez);
}

inline Mat4 Mat4::trs(const Vec3& translation, const Quat& rotation,
                      const Vec3& scale) {
    Mat4 m = rotation.toMat4();
    // Scale the rotated basis columns, then set the translation column.
    m.m[0][0] *= scale.x; m.m[1][0] *= scale.x; m.m[2][0] *= scale.x;
    m.m[0][1] *= scale.y; m.m[1][1] *= scale.y; m.m[2][1] *= scale.y;
    m.m[0][2] *= scale.z; m.m[1][2] *= scale.z; m.m[2][2] *= scale.z;
    m.m[0][3] = translation.x;
    m.m[1][3] = translation.y;
    m.m[2][3] = translation.z;
    return m;
}

// Minimal bounding volume for frustum culling. Center is in model space;
// radius is the max distance from center to any vertex.
struct BoundingSphere {
    Vec3 center;
    Real radius = 0;
};

struct Frustum {
    Vec3 normals[6];
    Real distances[6];

    static Frustum fromViewProjection(const Mat4& vp);
    bool containsSphere(const Vec3& center, Real radius) const;
};

constexpr Real PI = 3.14159265358979323846;

inline Real degreesToRadians(Real degrees) {
    return degrees * PI / 180.0;
}

inline Real radiansToDegrees(Real radians) {
    return radians * 180.0 / PI;
}

inline double randomDouble() {
    thread_local std::mt19937 gen(std::random_device{}());
    thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

inline double randomDouble(double min, double max) {
    return min + (max - min) * randomDouble();
}

inline Vec3 randomInUnitSphere() {
    while (true) {
        Vec3 p(randomDouble(-1, 1), randomDouble(-1, 1), randomDouble(-1, 1));
        if (p.lengthSquared() < 1.0) return p;
    }
}

inline Vec3 randomHemisphere(const Vec3& normal) {
    Vec3 inSphere = randomInUnitSphere();
    if (dot(inSphere, normal) > 0.0)
        return normalize(inSphere);
    else
        return normalize(-inSphere);
}

inline Vec3 randomCosineHemisphere(const Vec3& normal) {
    Vec3 dir = normalize(randomInUnitSphere() + normal);
    return dir;
}

// Gribb-Hartmann frustum plane extraction from a view-projection matrix.
// Each plane's normal points inward; a point is inside when dot(n,p)+d >= 0.
inline Frustum Frustum::fromViewProjection(const Mat4& vp) {
    Frustum f;
    // Row accessors: vp.m[row][col] (row-major)
    auto extractPlane = [&](int index, Real r0, Real r1, Real r2, Real r3) {
        Real len = Vec3(r0, r1, r2).length();
        if (len > 0) {
            f.normals[index] = Vec3(r0, r1, r2) / len;
            f.distances[index] = r3 / len;
        }
    };
    // Left:   row3 + row0
    extractPlane(0,
        vp.m[3][0] + vp.m[0][0], vp.m[3][1] + vp.m[0][1],
        vp.m[3][2] + vp.m[0][2], vp.m[3][3] + vp.m[0][3]);
    // Right:  row3 - row0
    extractPlane(1,
        vp.m[3][0] - vp.m[0][0], vp.m[3][1] - vp.m[0][1],
        vp.m[3][2] - vp.m[0][2], vp.m[3][3] - vp.m[0][3]);
    // Bottom: row3 + row1
    extractPlane(2,
        vp.m[3][0] + vp.m[1][0], vp.m[3][1] + vp.m[1][1],
        vp.m[3][2] + vp.m[1][2], vp.m[3][3] + vp.m[1][3]);
    // Top:    row3 - row1
    extractPlane(3,
        vp.m[3][0] - vp.m[1][0], vp.m[3][1] - vp.m[1][1],
        vp.m[3][2] - vp.m[1][2], vp.m[3][3] - vp.m[1][3]);
    // Near:   row3 + row2
    extractPlane(4,
        vp.m[3][0] + vp.m[2][0], vp.m[3][1] + vp.m[2][1],
        vp.m[3][2] + vp.m[2][2], vp.m[3][3] + vp.m[2][3]);
    // Far:    row3 - row2
    extractPlane(5,
        vp.m[3][0] - vp.m[2][0], vp.m[3][1] - vp.m[2][1],
        vp.m[3][2] - vp.m[2][2], vp.m[3][3] - vp.m[2][3]);
    return f;
}

inline bool Frustum::containsSphere(const Vec3& center, Real radius) const {
    for (int i = 0; i < 6; i++) {
        Real dist = dot(normals[i], center) + distances[i];
        if (dist < -radius) return false;
    }
    return true;
}

}  // namespace engine

#endif
