// Skinning demo (ADR-0073, character-posing-plan P2), headless: the same
// two-bone arm bent 80 degrees, twice. LEFT: rigid per-bone parts (the P0
// mannequin technique) — the joint gaps and interpenetration are the honest
// limitation. RIGHT: one continuous skinned tube deformed by CPU linear-blend
// skinning — the elbow creases smoothly. Path-traced to skin_demo.png: why P2
// exists, in one image.
#include "camera.h"
#include "engine/anim/skinned_mesh.h"
#include "engine/mesh_builder.h"
#include "image.h"
#include "job_system.h"
#include "material.h"
#include "path_tracer.h"
#include "scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
namespace anim = engine::anim;

namespace {

// Two joints: root at the origin, elbow one unit up. The "arm" spans y 0..2.
anim::Skeleton twoBoneArm() {
    anim::Skeleton sk;
    Transform t;
    sk.addJoint("root", -1, t);
    t.position = Vec3(0, 1, 0);
    sk.addJoint("elbow", 0, t);
    sk.finalize();
    return sk;
}

// A continuous tube around the arm with smoothly blended weights across the
// elbow: below y=0.7 all root, above y=1.3 all elbow, smoothstep between.
anim::SkinnedMesh skinnedTube(Real radius, int rings, int segments) {
    anim::SkinnedMesh mesh;
    auto ringPoint = [&](Real angle, Real y) {
        return Vec3(std::cos(angle) * radius, y, std::sin(angle) * radius);
    };
    for (int r = 0; r < rings; r++) {
        Real y0 = 2.0 * r / rings, y1 = 2.0 * (r + 1) / rings;
        for (int s = 0; s < segments; s++) {
            Real a0 = 2.0 * M_PI * s / segments;
            Real a1 = 2.0 * M_PI * (s + 1) / segments;
            Vec3 outward = normalize(Vec3(std::cos(a0), 0, std::sin(a0)) +
                                     Vec3(std::cos(a1), 0, std::sin(a1)));
            MeshBuilder::emitQuad(mesh.bind, ringPoint(a0, y0), ringPoint(a1, y0),
                                  ringPoint(a1, y1), ringPoint(a0, y1), outward,
                                  Vec3(0.85, 0.62, 0.45));
        }
    }

    mesh.influences.resize(mesh.bind.vertices.size());
    for (size_t i = 0; i < mesh.bind.vertices.size(); i++) {
        Real y = mesh.bind.vertices[i].position.y;
        Real t = std::min(std::max((y - 0.7) / 0.6, 0.0), 1.0);
        t = t * t * (3 - 2 * t);   // smoothstep
        anim::VertexWeights& vw = mesh.influences[i];
        vw.joints[0] = 0;
        vw.weights[0] = static_cast<float>(1.0 - t);
        vw.joints[1] = 1;
        vw.weights[1] = static_cast<float>(t);
    }
    return mesh;
}

void addMesh(Scene& scene, const RenderMesh& mesh, const Mat4& model, int mat) {
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        scene.addTriangle(
            model.transformPoint(mesh.vertices[mesh.indices[i]].position),
            model.transformPoint(mesh.vertices[mesh.indices[i + 1]].position),
            model.transformPoint(mesh.vertices[mesh.indices[i + 2]].position),
            mat);
}

}  // namespace

int main() {
    anim::Skeleton sk = twoBoneArm();

    anim::Pose bent = anim::Pose::bind(sk);
    bent.locals[1].orientation =
        Quat::fromAxisAngle(Vec3(0, 0, 1), -80.0 * M_PI / 180.0);
    auto world = anim::worldJointMatrices(sk, bent);
    auto skin = anim::skinningMatrices(sk, world);

    const Real radius = 0.22;

    // LEFT: rigid per-bone segments following the joints (P0 style). Flat
    // cylinder ends make the failure legible: a wedge-shaped gap opens on the
    // elbow's outside while the inside interpenetrates.
    RenderMesh rigid;
    {
        RenderMesh lower = MeshBuilder::cylinder(radius, 1.0);
        MeshBuilder::transform(lower, world[0] * Mat4::translate(0, 0.5, 0));
        RenderMesh upper = MeshBuilder::cylinder(radius, 1.0);
        MeshBuilder::transform(upper, world[1] * Mat4::translate(0, 0.5, 0));
        MeshBuilder::append(rigid, lower);
        MeshBuilder::append(rigid, upper);
    }

    // RIGHT: the continuous tube through CPU skinning (P2).
    anim::SkinnedMesh tube = skinnedTube(radius, 24, 20);
    RenderMesh smooth = anim::skinMesh(tube, skin);

    Scene scene;
    int ground = scene.addMaterial(Material::diffuse(Vec3(0.42, 0.42, 0.4)));
    scene.addQuad(Vec3(-40, 0, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), ground);
    int sky = scene.addMaterial(Material::emissive(Vec3(0.75, 0.82, 0.95), 0.9));
    scene.addQuad(Vec3(-40, 10, -40), Vec3(80, 0, 0), Vec3(0, 0, 80), sky);
    int lamp = scene.addMaterial(Material::emissive(Vec3(1.0, 0.9, 0.75), 6.0));
    scene.addQuad(Vec3(-5, 6, 5), Vec3(2.5, 0, 0), Vec3(0, 0, -2.5), lamp);
    int fleshRigid = scene.addMaterial(Material::diffuse(Vec3(0.62, 0.66, 0.78)));
    int fleshSkinned = scene.addMaterial(Material::diffuse(Vec3(0.85, 0.62, 0.45)));

    addMesh(scene, rigid, Mat4::translate(-1.7, 0.02, 0), fleshRigid);
    addMesh(scene, smooth, Mat4::translate(0.5, 0.02, 0), fleshSkinned);
    scene.buildAccelerator();

    Camera camera(Vec3(0.0, 1.6, 5.6), Vec3(-0.1, 0.9, 0), Vec3(0, 1, 0), 40.0, 1.0);
    JobSystem jobs;
    RenderConfig cfg;
    cfg.imageSize = 640;
    cfg.samplesPerPixel = 96;
    Image img = renderImage(scene, camera, cfg, jobs);
    img.writePng("skin_demo.png");
    std::printf("Wrote skin_demo.png: rigid (%zu tris) vs skinned (%zu tris), "
                "elbow bent 80 deg\n",
                rigid.indices.size() / 3, smooth.indices.size() / 3);
    return 0;
}
