#include "board_meshes.h"

#include "mesh_builder.h"

namespace engine {

namespace {
void addBlock(RenderMesh& m, const Vec3& centre, const Vec3& size, const Vec3& colour) {
    MeshBuilder::emitBox(m, MeshFrame::centred(centre, size), colour);
}
}  // namespace

RenderMesh buildSkateboard() {
    RenderMesh m;
    const Vec3 deck(0.16, 0.1, 0.12), wheel(0.92, 0.9, 0.82), truck(0.6, 0.6, 0.62);
    addBlock(m, Vec3(0, 0.1, 0), Vec3(0.21, 0.025, 0.62), deck);
    addBlock(m, Vec3(0, 0.125, 0.36), Vec3(0.21, 0.025, 0.14), deck);    // nose
    addBlock(m, Vec3(0, 0.125, -0.36), Vec3(0.21, 0.025, 0.14), deck);   // tail
    for (Real z : {-0.24, 0.24}) {
        addBlock(m, Vec3(0, 0.065, z), Vec3(0.16, 0.04, 0.04), truck);
        for (Real x : {-0.085, 0.085}) addBlock(m, Vec3(x, 0.03, z), Vec3(0.035, 0.06, 0.06), wheel);
    }
    return m;
}

RenderMesh buildScooter() {
    RenderMesh m;
    const Vec3 deck(0.15, 0.16, 0.18), metal(0.7, 0.72, 0.75), grip(0.08, 0.08, 0.09), wheel(0.2, 0.2, 0.22);
    addBlock(m, Vec3(0, 0.07, 0), Vec3(0.13, 0.035, 0.6), deck);
    addBlock(m, Vec3(0, 0.55, 0.34), Vec3(0.04, 0.95, 0.04), metal);    // the stem
    addBlock(m, Vec3(0, 1.02, 0.34), Vec3(0.5, 0.035, 0.035), metal);   // the bar
    for (Real x : {-0.22, 0.22}) addBlock(m, Vec3(x, 1.02, 0.34), Vec3(0.09, 0.045, 0.045), grip);
    for (Real z : {-0.3, 0.34}) addBlock(m, Vec3(0, 0.05, z), Vec3(0.03, 0.1, 0.1), wheel);
    return m;
}

}  // namespace engine
