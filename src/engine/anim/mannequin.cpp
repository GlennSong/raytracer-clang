#include "mannequin.h"

#include "../mesh_builder.h"

namespace engine {
namespace anim {

namespace {

// Palette: a neutral figure that reads at a glance (skin / shirt / pants /
// shoes). Baked into vertex colors so a single white material shows it.
const Vec3 SKIN(0.87, 0.67, 0.53);
const Vec3 SHIRT(0.30, 0.45, 0.62);
const Vec3 PANTS(0.28, 0.28, 0.34);
const Vec3 SHOES(0.16, 0.13, 0.11);

void tint(RenderMesh& mesh, const Vec3& color) {
    for (auto& v : mesh.vertices) v.color = color;
}

// A capsule whose cylinder spans from the joint origin ALONG `axis` for
// `length` — limbs hang off their joint, they aren't centred on it.
RenderMesh limb(Real radius, Real length, const Vec3& axis) {
    RenderMesh mesh = MeshBuilder::capsule(static_cast<float>(radius),
                                           static_cast<float>(length));
    // Capsule is Y-centred at the origin: shift so it starts at the joint,
    // then rotate +Y onto the limb axis.
    Quat rot;
    if (axis.x > 0.5) rot = Quat::fromAxisAngle(Vec3(0, 0, 1), -M_PI / 2.0);
    else if (axis.x < -0.5) rot = Quat::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2.0);
    else if (axis.y > 0.5) rot = Quat();
    else rot = Quat::fromAxisAngle(Vec3(0, 0, 1), M_PI);   // -Y (legs)
    MeshBuilder::transform(
        mesh, Mat4::trs(axis * (length * 0.5), rot, Vec3(1, 1, 1)));
    return mesh;
}

RenderMesh boxAt(const Vec3& size, const Vec3& offset) {
    RenderMesh mesh = MeshBuilder::box(size);
    MeshBuilder::transform(mesh, Mat4::trs(offset, Quat(), Vec3(1, 1, 1)));
    return mesh;
}

}  // namespace

Mannequin buildMannequin(Real height) {
    // Proportions as fractions of a 1.8 m reference figure.
    const Real s = height / 1.8;
    auto at = [&](Real x, Real y, Real z) {
        Transform t;
        t.position = Vec3(x * s, y * s, z * s);
        return t;
    };

    Mannequin m;
    Skeleton& sk = m.skeleton;
    // Core, pelvis-rooted; feet land at y=0 (hip 0.92 - knee 0.44 - shin 0.40
    // leaves the ankle at 0.08, the foot box reaches the floor).
    int pelvis = sk.addJoint("pelvis", -1, at(0, 0.98, 0));
    int spine = sk.addJoint("spine", pelvis, at(0, 0.14, 0));
    int chest = sk.addJoint("chest", spine, at(0, 0.22, 0));
    int neck = sk.addJoint("neck", chest, at(0, 0.18, 0));
    int head = sk.addJoint("head", neck, at(0, 0.08, 0));
    // Arms in T-pose along ±X.
    int shoulderL = sk.addJoint("shoulder.L", chest, at(0.20, 0.14, 0));
    int elbowL = sk.addJoint("elbow.L", shoulderL, at(0.30, 0, 0));
    int wristL = sk.addJoint("wrist.L", elbowL, at(0.26, 0, 0));
    int shoulderR = sk.addJoint("shoulder.R", chest, at(-0.20, 0.14, 0));
    int elbowR = sk.addJoint("elbow.R", shoulderR, at(-0.30, 0, 0));
    int wristR = sk.addJoint("wrist.R", elbowR, at(-0.26, 0, 0));
    // Legs straight down.
    int hipL = sk.addJoint("hip.L", pelvis, at(0.10, -0.06, 0));
    int kneeL = sk.addJoint("knee.L", hipL, at(0, -0.44, 0));
    int ankleL = sk.addJoint("ankle.L", kneeL, at(0, -0.40, 0));
    int hipR = sk.addJoint("hip.R", pelvis, at(-0.10, -0.06, 0));
    int kneeR = sk.addJoint("knee.R", hipR, at(0, -0.44, 0));
    int ankleR = sk.addJoint("ankle.R", kneeR, at(0, -0.40, 0));
    sk.finalize();

    auto part = [&](RenderMesh mesh, int joint, const Vec3& color) {
        tint(mesh, color);
        m.parts.push_back({std::move(mesh), joint, color});
    };
    const Vec3 X(1, 0, 0), NX(-1, 0, 0), NY(0, -1, 0);

    part(boxAt(Vec3(0.28, 0.16, 0.18) * s, Vec3(0, 0.02, 0) * s), pelvis, PANTS);
    part(boxAt(Vec3(0.26, 0.24, 0.16) * s, Vec3(0, 0.11, 0) * s), spine, SHIRT);
    part(boxAt(Vec3(0.32, 0.22, 0.18) * s, Vec3(0, 0.08, 0) * s), chest, SHIRT);
    part(boxAt(Vec3(0.17, 0.21, 0.19) * s, Vec3(0, 0.12, 0) * s), head, SKIN);

    part(limb(0.050 * s, 0.28 * s, X), shoulderL, SHIRT);
    part(limb(0.043 * s, 0.24 * s, X), elbowL, SKIN);
    part(boxAt(Vec3(0.11, 0.05, 0.09) * s, Vec3(0.06, 0, 0) * s), wristL, SKIN);
    part(limb(0.050 * s, 0.28 * s, NX), shoulderR, SHIRT);
    part(limb(0.043 * s, 0.24 * s, NX), elbowR, SKIN);
    part(boxAt(Vec3(0.11, 0.05, 0.09) * s, Vec3(-0.06, 0, 0) * s), wristR, SKIN);

    part(limb(0.070 * s, 0.40 * s, NY), hipL, PANTS);
    part(limb(0.055 * s, 0.36 * s, NY), kneeL, PANTS);
    part(boxAt(Vec3(0.10, 0.08, 0.25) * s, Vec3(0, -0.04, 0.06) * s), ankleL, SHOES);
    part(limb(0.070 * s, 0.40 * s, NY), hipR, PANTS);
    part(limb(0.055 * s, 0.36 * s, NY), kneeR, PANTS);
    part(boxAt(Vec3(0.10, 0.08, 0.25) * s, Vec3(0, -0.04, 0.06) * s), ankleR, SHOES);

    return m;
}

RenderMesh bakeMannequinMesh(const Mannequin& mannequin, const Pose& pose,
                             const Mat4& model) {
    std::vector<Mat4> world = worldJointMatrices(mannequin.skeleton, pose);
    RenderMesh baked;
    for (const MannequinPart& part : mannequin.parts) {
        if (part.joint < 0 || part.joint >= static_cast<int>(world.size()))
            continue;
        MeshBuilder::appendTransformed(baked, part.mesh,
                                       model * world[part.joint]);
    }
    return baked;
}

}  // namespace anim
}  // namespace engine
