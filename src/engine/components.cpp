#include "components.h"

Mat4 Transform::matrix() const {
    return Mat4::trs(position, orientation, scale);
}

Transform lerp(const Transform& a, const Transform& b, Real t) {
    Transform result;
    result.position = lerp(a.position, b.position, t);
    result.orientation = Quat::slerp(a.orientation, b.orientation, t);
    result.scale = lerp(a.scale, b.scale, t);
    return result;
}
