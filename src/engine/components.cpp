#include "components.h"

Mat4 Transform::matrix() const {
    return Mat4::translate(position.x, position.y, position.z) *
           Mat4::rotateZ(rotation.z) *
           Mat4::rotateY(rotation.y) *
           Mat4::rotateX(rotation.x) *
           Mat4::scale(scale.x, scale.y, scale.z);
}

Transform lerp(const Transform& a, const Transform& b, Real t) {
    Transform result;
    result.position = a.position + (b.position - a.position) * t;
    result.rotation = a.rotation + (b.rotation - a.rotation) * t;
    result.scale = a.scale + (b.scale - a.scale) * t;
    return result;
}
