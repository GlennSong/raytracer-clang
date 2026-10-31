#ifndef RAYTRACER_ENGINE_BODY_DRAW_H
#define RAYTRACER_ENGINE_BODY_DRAW_H

// THE JOINTED BODY AS DRAWN, for anything that poses one every step (the player in third person, a ragdoll): the
// torso and head as their blocks, each limb as ONE skinned tube (block_body.h skinnedLimb) placed at its shoulder
// or hip, chosen for its fold from meshes made once per 2.5 degrees -- the skin is exact at each step and the step
// is under what a fold moves in a frame. The forearm and shin blocks are not drawn: the tube covers them.
//
// A body baked once (a walker's stride) welds its tubes at their exact folds instead: procgen/person.h
// buildPosedPerson.

#include "block_body.h"
#include "../renderer/renderer.h"

#include <array>
#include <unordered_map>

namespace engine {

class AssetManager;

struct BodyPiece {
    bool drawn = true;   // false: the forearm and shin (the tube covers them)
    MeshHandle mesh;
    Vec3 position;
    Quat orientation;
    Vec3 scale{1, 1, 1};
};

class BodyMeshes {
public:
    static constexpr Real kFoldStep = 2.5 * 3.14159265358979 / 180.0;
    // the pieces for a posed body's blocks (poseBlocks, or a ragdoll's parts with their sizes) at this scale
    std::array<BodyPiece, BlockBody::kParts> pieces(AssetManager& assets, const std::array<PartTransform, BlockBody::kParts>& blocks,
                                                    Real scale);
    // `girth` 1 is the body as built; the first-person view draws its arms at kFirstPersonGirth
    MeshHandle limb(AssetManager& assets, int limb, Real fold, Real girth = 1);
    // seen from 0.3 m the body's 12 x 16 cm arm fills the view's lower edge: the first person draws it as thick as
    // the old block sleeve (7.5 cm), the same bones and bend
    static constexpr Real kFirstPersonGirth = 0.6;
    // Make every fold's tube now (an arm's and a leg's: ~120 small meshes), so a body that starts moving never
    // builds one mid-stride -- the walking hitch the walker's lazily made poses had (TECH_DEBT).
    // (girth below 1: the first-person view's arms only)
    void warm(AssetManager& assets, Real girth = 1);

private:
    MeshHandle box_;
    std::unordered_map<int, MeshHandle> limbs_;   // (arm or leg) x fold step
};

}  // namespace engine

#endif
