#include "face.h"

namespace engine {

Face Face::flat(const Vec3& bl, const Vec3& h, const Vec3& v, const Vec3& n, Real width, Real height) {
    Face f;
    f.kind = Kind::Flat;
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

}  // namespace engine
