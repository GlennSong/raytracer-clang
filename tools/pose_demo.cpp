// Pose demo (ADR-0072, character-posing-plan P0), headless: three mannequins
// struck into different poses purely by joint rotations on the Skeleton, each
// baked to world-space part meshes and path-traced into pose_demo.png — the
// first proto comic panel: posed characters, a camera, a still.
#include "camera.h"
#include "engine/anim/mannequin.h"
#include "engine/mesh_builder.h"
#include "image.h"
#include "job_system.h"
#include "material.h"
#include "path_tracer.h"
#include "scene.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>

using namespace engine;
namespace anim = engine::anim;   // Skeleton/Pose live here (procgen owns engine::Skeleton)

namespace {

Quat rx(Real deg) { return Quat::fromAxisAngle(Vec3(1, 0, 0), deg * M_PI / 180.0); }
Quat ry(Real deg) { return Quat::fromAxisAngle(Vec3(0, 1, 0), deg * M_PI / 180.0); }
Quat rz(Real deg) { return Quat::fromAxisAngle(Vec3(0, 0, 1), deg * M_PI / 180.0); }

void set(anim::Pose& pose, const anim::Skeleton& sk, const char* joint, const Quat& q) {
    int i = sk.find(joint);
    if (i >= 0) pose.locals[i].orientation = q;
}

// Add a mannequin's parts to the scene, posed and placed by `model`, with a
// flat material per part color.
void addPosed(Scene& scene, const anim::Mannequin& m, const anim::Pose& pose,
              const Mat4& model,
              std::map<std::tuple<float, float, float>, int>& materials) {
    auto world = anim::worldJointMatrices(m.skeleton, pose);
    for (const auto& part : m.parts) {
        auto key = std::make_tuple(static_cast<float>(part.color.x),
                                   static_cast<float>(part.color.y),
                                   static_cast<float>(part.color.z));
        auto it = materials.find(key);
        if (it == materials.end())
            it = materials.emplace(key, scene.addMaterial(Material::diffuse(part.color)))
                     .first;
        RenderMesh posed;
        MeshBuilder::appendTransformed(posed, part.mesh, model * world[part.joint]);
        for (size_t i = 0; i + 2 < posed.indices.size(); i += 3)
            scene.addTriangle(posed.vertices[posed.indices[i]].position,
                              posed.vertices[posed.indices[i + 1]].position,
                              posed.vertices[posed.indices[i + 2]].position,
                              it->second);
    }
}

}  // namespace

int main() {
    anim::Mannequin m = anim::buildMannequin();
    const anim::Skeleton& sk = m.skeleton;

    // Pose 1: the T-pose (bind) — the reference figure.
    anim::Pose tpose = anim::Pose::bind(sk);

    // Rotation cheat sheet (T-pose): the LEFT arm points +X — rz(+90) raises
    // it straight up, rz(-90) hangs it down. The RIGHT arm points -X, so the
    // signs flip: rz(+90) hangs it DOWN. Legs point -Y; rx(-) swings a leg
    // forward, rx(+) back.

    // Pose 2: waving — left arm up at a friendly diagonal with a bent
    // forearm, right arm relaxed down, a little head tilt.
    anim::Pose wave = anim::Pose::bind(sk);
    set(wave, sk, "shoulder.L", rz(60));
    set(wave, sk, "elbow.L", rz(48));
    set(wave, sk, "shoulder.R", rz(72));
    set(wave, sk, "elbow.R", rz(8));
    set(wave, sk, "head", rz(-10));

    // Pose 3: mid-stride — arms hang and counter-swing, legs scissored with
    // bent knees, a lean into the walk, pelvis dropped so the front heel
    // plants instead of hovering.
    anim::Pose stride = anim::Pose::bind(sk);
    set(stride, sk, "shoulder.L", rx(-28) * rz(-72));
    set(stride, sk, "shoulder.R", rx(28) * rz(72));
    set(stride, sk, "elbow.L", rz(-12));
    set(stride, sk, "elbow.R", rz(12));
    set(stride, sk, "hip.L", rx(-30));
    set(stride, sk, "knee.L", rx(10));
    set(stride, sk, "hip.R", rx(24));
    set(stride, sk, "knee.R", rx(48));
    set(stride, sk, "spine", rx(-6));
    stride.locals[sk.find("pelvis")].position.y -= 0.09;

    Scene scene;
    int ground = scene.addMaterial(Material::diffuse(Vec3(0.45, 0.44, 0.42)));
    scene.addQuad(Vec3(-40, 0, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), ground);
    int sky = scene.addMaterial(Material::emissive(Vec3(0.75, 0.82, 0.95), 0.9));
    scene.addQuad(Vec3(-40, 12, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), sky);
    int lamp = scene.addMaterial(Material::emissive(Vec3(1.0, 0.9, 0.75), 6.0));
    scene.addQuad(Vec3(-6, 7, 6), Vec3(3, 0, 0), Vec3(0, 0, -3), lamp);

    std::map<std::tuple<float, float, float>, int> materials;
    addPosed(scene, m, tpose, Mat4::trs(Vec3(-1.6, 0, 0), ry(12), Vec3(1, 1, 1)),
             materials);
    addPosed(scene, m, wave, Mat4::trs(Vec3(0.0, 0, 0.4), ry(-6), Vec3(1, 1, 1)),
             materials);
    addPosed(scene, m, stride, Mat4::trs(Vec3(1.6, 0, 0), ry(-25), Vec3(1, 1, 1)),
             materials);
    scene.buildAccelerator();

    Camera camera(Vec3(0.2, 1.5, 5.6), Vec3(0, 1.0, 0), Vec3(0, 1, 0), 40.0, 1.0);
    JobSystem jobs;
    RenderConfig cfg;
    cfg.imageSize = 640;
    cfg.samplesPerPixel = 96;
    Image img = renderImage(scene, camera, cfg, jobs);
    img.writePng("pose_demo.png");
    std::printf("Wrote pose_demo.png (%dx%d): %d joints, %zu parts, 3 poses\n",
                img.width, img.height, sk.jointCount(), m.parts.size());
    return 0;
}
