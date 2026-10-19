#include "person.h"

#include "../mesh_builder.h"

namespace engine {

namespace {

// Deterministic outfits: shirt / pants / skin.
const PersonOutfit kOutfits[] = {
    {{0.62, 0.35, 0.25}, {0.22, 0.24, 0.30}, {0.87, 0.68, 0.55}},
    {{0.30, 0.42, 0.58}, {0.16, 0.16, 0.18}, {0.55, 0.38, 0.28}},
    {{0.38, 0.50, 0.32}, {0.30, 0.26, 0.22}, {0.76, 0.56, 0.42}},
    {{0.55, 0.48, 0.30}, {0.24, 0.28, 0.36}, {0.42, 0.29, 0.22}},
    {{0.45, 0.32, 0.48}, {0.20, 0.20, 0.24}, {0.87, 0.68, 0.55}},
    {{0.60, 0.58, 0.55}, {0.28, 0.22, 0.20}, {0.64, 0.46, 0.34}},
    {{0.72, 0.62, 0.28}, {0.18, 0.22, 0.28}, {0.55, 0.38, 0.28}},
    {{0.35, 0.55, 0.55}, {0.26, 0.24, 0.22}, {0.76, 0.56, 0.42}},
    {{0.66, 0.30, 0.34}, {0.22, 0.26, 0.24}, {0.87, 0.68, 0.55}},
    {{0.42, 0.38, 0.60}, {0.20, 0.18, 0.16}, {0.64, 0.46, 0.34}},
    {{0.52, 0.56, 0.62}, {0.30, 0.30, 0.34}, {0.42, 0.29, 0.22}},
    {{0.58, 0.44, 0.36}, {0.24, 0.20, 0.26}, {0.76, 0.56, 0.42}},
    // LAST SLOT: the PLAYER's outfit -- a vivid red jacket over near-black trousers, deliberately absent from the muted
    // walker palette above so the player reads instantly in a crowd (citysim's personOutfitCount() excludes it)
    {{0.88, 0.12, 0.10}, {0.10, 0.10, 0.12}, {0.87, 0.68, 0.55}},
};
constexpr int kNumOutfits = static_cast<int>(sizeof(kOutfits) / sizeof(kOutfits[0]));

// A coloured box centred at `c`, PITCHED about the X-axis line y = pivotY (a limb about its hip or shoulder)
void addBoxPitched(RenderMesh& out, const Vec3& size, const Vec3& c, const Vec3& color, Real pivotY, Real angle) {
    MeshBuilder::emitBox(out, MeshFrame::centred(c, size).rotatedAbout(Vec3(0, pivotY, 0), Vec3(1, 0, 0), angle), color);
}

}  // namespace

int personOutfitTotal() { return kNumOutfits; }

const PersonOutfit& personOutfit(int outfit) { return kOutfits[((outfit % kNumOutfits) + kNumOutfits) % kNumOutfits]; }

RenderMesh buildStandingPerson(Real swing, int outfit) {
    const PersonOutfit& o = personOutfit(outfit);
    RenderMesh m;
    // hips at y = -0.05, shoulders at y = 0.50; legs swing opposed, arms counter-swing at ~70%
    const Real hipY = -0.05, shoulderY = 0.50;
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(-0.115, -0.475, 0), o.pants, hipY, swing);         // left leg
    addBoxPitched(m, Vec3(0.17, 0.85, 0.22), Vec3(0.115, -0.475, 0), o.pants, hipY, -swing);         // right leg
    MeshBuilder::emitBox(m, MeshFrame::centred(Vec3(0, 0.25, 0), Vec3(0.44, 0.60, 0.26)), o.shirt);  // torso
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(-0.29, 0.19, 0), o.shirt, shoulderY, -swing * 0.7);  // left arm
    addBoxPitched(m, Vec3(0.12, 0.62, 0.16), Vec3(0.29, 0.19, 0), o.shirt, shoulderY, swing * 0.7);    // right arm
    MeshBuilder::emitBox(m, MeshFrame::centred(Vec3(0, 0.70, 0), Vec3(0.26, 0.26, 0.26)), o.skin);   // head
    return m;
}

}  // namespace engine
