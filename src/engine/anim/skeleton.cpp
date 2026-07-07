#include "skeleton.h"

#include "../debug_draw.h"

namespace engine {
namespace anim {

int Skeleton::addJoint(const std::string& name, int parent,
                       const Transform& bindLocal) {
    if (parent >= static_cast<int>(joints.size()) || parent < -1) return -1;
    joints.push_back({name, parent, bindLocal});
    return static_cast<int>(joints.size()) - 1;
}

int Skeleton::find(const std::string& name) const {
    for (int i = 0; i < static_cast<int>(joints.size()); i++)
        if (joints[i].name == name) return i;
    return -1;
}

std::vector<Mat4> Skeleton::bindWorld() const {
    std::vector<Mat4> world(joints.size());
    for (std::size_t i = 0; i < joints.size(); i++) {
        Mat4 local = joints[i].bindLocal.matrix();
        world[i] = joints[i].parent < 0 ? local : world[joints[i].parent] * local;
    }
    return world;
}

void Skeleton::finalize() {
    std::vector<Mat4> world = bindWorld();
    inverseBinds.resize(world.size());
    for (std::size_t i = 0; i < world.size(); i++)
        inverseBinds[i] = world[i].inverse();
}

Pose Pose::bind(const Skeleton& skeleton) {
    Pose pose;
    pose.locals.reserve(skeleton.jointCount());
    for (int i = 0; i < skeleton.jointCount(); i++)
        pose.locals.push_back(skeleton.joint(i).bindLocal);
    return pose;
}

Pose lerpPose(const Pose& a, const Pose& b, Real t) {
    Pose out;
    std::size_t count = std::min(a.locals.size(), b.locals.size());
    out.locals.reserve(count);
    for (std::size_t i = 0; i < count; i++)
        out.locals.push_back(lerp(a.locals[i], b.locals[i], t));
    return out;
}

std::vector<Mat4> worldJointMatrices(const Skeleton& skeleton,
                                     const Pose& pose) {
    std::vector<Mat4> world(skeleton.jointCount());
    for (int i = 0; i < skeleton.jointCount(); i++) {
        // A short pose (e.g. from a mismatched clip) falls back to bind locals.
        const Transform& local = i < static_cast<int>(pose.locals.size())
                                     ? pose.locals[i]
                                     : skeleton.joint(i).bindLocal;
        Mat4 localMat = local.matrix();
        int parent = skeleton.joint(i).parent;
        world[i] = parent < 0 ? localMat : world[parent] * localMat;
    }
    return world;
}

std::vector<Mat4> skinningMatrices(const Skeleton& skeleton,
                                   const std::vector<Mat4>& world) {
    const std::vector<Mat4>& inverseBind = skeleton.inverseBind();
    std::vector<Mat4> skin(world.size());
    for (std::size_t i = 0; i < world.size() && i < inverseBind.size(); i++)
        skin[i] = world[i] * inverseBind[i];
    return skin;
}

void debugDrawSkeleton(DebugDraw& debug, const Skeleton& skeleton,
                       const std::vector<Mat4>& world, const Vec3& color,
                       Real seconds) {
    for (int i = 0; i < skeleton.jointCount() &&
                    i < static_cast<int>(world.size()); i++) {
        int parent = skeleton.joint(i).parent;
        if (parent < 0) continue;
        debug.line(world[parent].transformPoint(Vec3(0, 0, 0)),
                   world[i].transformPoint(Vec3(0, 0, 0)), color, seconds);
    }
}

}  // namespace anim
}  // namespace engine
