#include "face.h"

namespace engine {

Face Face::flat(const Vec3& bl, const Vec3& h, const Vec3& v, const Vec3& n, Real width, Real height) {
    Face f;
    f.bl = bl;
    f.h = h;
    f.v = v;
    f.n = n;
    f.width = width;
    f.height = height;
    return f;
}

Face Face::ofEdge(const Poly2& pl, std::size_t i, Real y, Real h) {
    const Vec2 a = pl[i], b = pl[(i + 1) % pl.size()];
    const Vec2 d = normalize(b - a);
    return flat(Vec3(a.x, y, a.y), Vec3(d.x, 0, d.y), Vec3(0, 1, 0), Vec3(d.y, 0, -d.x), (b - a).length(), h);
}

Face Face::ofEdgeRuled(const Poly2& lower, const Poly2& upper, std::size_t i, Real y, Real h) {
    Face fr = ofEdge(lower, i, y, h);
    const std::size_t n = lower.size();
    if (upper.size() != n) return fr;
    const Vec2 a = upper[i % n], b = upper[(i + 1) % n];
    fr.ruled = true;
    fr.tl = Vec3(a.x, y + h, a.y);
    fr.tr = Vec3(b.x, y + h, b.y);
    const Vec3 mid0 = fr.at(0, 0), mid1 = fr.at(fr.width, 0), mid2 = fr.at(fr.width, h), mid3 = fr.at(0, h);
    const Vec3 nrm = cross(mid1 - mid0, mid3 - mid0) + cross(mid2 - mid1, mid3 - mid2);
    if (nrm.lengthSquared() > 1e-12) {
        const Vec3 nn = normalize(nrm);
        fr.n = dot(nn, fr.n) < 0 ? nn * -1.0 : nn;
    }
    return fr;
}

}  // namespace engine
