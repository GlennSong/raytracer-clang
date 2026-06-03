#include "world.h"

Mat4 Transform::matrix() const {
    return Mat4::translate(position.x, position.y, position.z) *
           Mat4::rotateZ(rotation.z) *
           Mat4::rotateY(rotation.y) *
           Mat4::rotateX(rotation.x) *
           Mat4::scale(scale.x, scale.y, scale.z);
}

Transform lerp(const Transform& a, const Transform& b, double t) {
    Transform result;
    result.position = a.position + (b.position - a.position) * t;
    result.rotation = a.rotation + (b.rotation - a.rotation) * t;
    result.scale = a.scale + (b.scale - a.scale) * t;
    return result;
}

Entity& World::add(const Entity& entity) {
    entityList.push_back(entity);
    Entity& added = entityList.back();
    added.previousTransform = added.transform;
    return added;
}

void World::step(double dt) {
    for (Entity& e : entityList) {
        e.previousTransform = e.transform;
        if (!e.simulated) continue;
        e.transform.position += e.velocity * dt;
        e.transform.rotation += e.angularVelocity * dt;
    }
}

Transform World::renderTransform(const Entity& entity, double alpha) const {
    return lerp(entity.previousTransform, entity.transform, alpha);
}
