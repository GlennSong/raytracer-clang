#ifndef RAYTRACER_MATH_H
#define RAYTRACER_MATH_H

#include <cmath>
#include <algorithm>
#include <random>

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
    Vec3& operator*=(Real t) { x *= t; y *= t; z *= t; return *this; }

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

struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray() {}
    Ray(const Vec3& origin, const Vec3& direction)
        : origin(origin), direction(direction) {}

    Vec3 at(Real t) const { return origin + direction * t; }
};

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
};

constexpr Real PI = 3.14159265358979323846;

inline Real degreesToRadians(Real degrees) {
    return degrees * PI / 180.0;
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

#endif
