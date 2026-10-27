#include "body_draw.h"

#include "asset_manager.h"
#include "mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace engine {

MeshHandle BodyMeshes::limb(AssetManager& assets, int limb, Real fold, Real girth) {
    // a left and a right limb are the same tube in the limb's own frame: one set for the arms, one for the legs
    const int kind = limb >= 2 ? 1 : 0;
    const int step = static_cast<int>(std::lround(std::max(Real(0), fold) / kFoldStep));
    const int thin = girth < 1 ? 1 : 0;   // (two girths: the body's, and the first-person view's)
    const int key = thin * 10000 + kind * 1000 + step;
    auto it = limbs_.find(key);
    if (it != limbs_.end()) return it->second;
    const MeshHandle h = assets.acquireMesh(skinnedLimb(limb, step * kFoldStep, Vec3(1, 1, 1), girth),
                                            std::string("body:limb:") + (thin ? "fp:" : "") + (kind ? "leg:" : "arm:") +
                                                std::to_string(step));
    limbs_[key] = h;
    return h;
}

void BodyMeshes::warm(AssetManager& assets, Real girth) {
    for (Real f = 0; f <= kElbowMax + kFoldStep; f += kFoldStep) limb(assets, 0, f, girth);   // (limb 0: an arm)
    if (girth < 1) return;
    for (Real f = 0; f <= kKneeMax + kFoldStep; f += kFoldStep) limb(assets, 2, f);   // (limb 2: a leg)
}

std::array<BodyPiece, BlockBody::kParts> BodyMeshes::pieces(AssetManager& assets,
                                                            const std::array<PartTransform, BlockBody::kParts>& blocks,
                                                            Real scale) {
    if (!box_.valid()) box_ = assets.acquireMesh(MeshBuilder::box(Vec3(1, 1, 1)), "player:arm_block");
    std::array<BodyPiece, BlockBody::kParts> out;
    for (int i : {BlockBody::Torso, BlockBody::Head}) {
        const PartTransform& b = blocks[static_cast<std::size_t>(i)];
        out[static_cast<std::size_t>(i)] = {true, box_, b.position, b.orientation, b.size};
    }
    for (int k = 0; k < BlockBody::kLimbs; ++k) {
        const BlockBody::Part up = BlockBody::upper(k), lo = BlockBody::lower(k);
        const PartTransform& u = blocks[static_cast<std::size_t>(up)];
        const PartTransform& l = blocks[static_cast<std::size_t>(lo)];
        const Real fold = foldOf(u.orientation, l.orientation);
        out[static_cast<std::size_t>(up)] = {true, limb(assets, k, fold), jointOf(up, u, scale), u.orientation,
                                             Vec3(scale, scale, scale)};
        out[static_cast<std::size_t>(lo)].drawn = false;
    }
    return out;
}

}  // namespace engine
