#ifndef RAYTRACER_ENGINE_PROCGEN_STYLIZED_TREE_H
#define RAYTRACER_ENGINE_PROCGEN_STYLIZED_TREE_H

// STYLIZED TREES (the flora plan; Breath of the Wild is the reference). A full crown from very
// few polygons: a simple skeleton -- trunk and a few limbs, no twigs -- and a canopy of puffy,
// noise-lumped CLUMPS whose normals lean toward the crown's own centre, so the tree lights as
// one soft volume of foliage instead of a thousand leaves; colour is baked darker underneath
// and inside, lighter on top. About 1-1.5k triangles a tree. Pines are stacked jagged cone
// tiers; palms a ringed, curved trunk under a crown of drooping sawtooth fronds (no alpha).

#include "../../renderer/renderer.h"   // RenderMesh

#include <cstdint>
#include <string>

namespace engine {

enum class StylizedShape { Round, Spreading, Columnar, Flowering, Pine, Palm };
bool stylizedShapeFromName(const std::string& name, StylizedShape& out);

struct StylizedTreeParams {
    StylizedShape shape = StylizedShape::Round;
    double height = 7.0;        // to the crown top (m); each seed varies it +-15%
    double crownRadius = 0.0;   // 0 = from the shape and height
    int    clumps = 0;          // canopy clumps (0 = the shape's default)
    Vec3   barkColor{0.10, 0.065, 0.04};                 // linear albedo
    Vec3   leafDark{0.014, 0.045, 0.010}, leafLight{0.08, 0.19, 0.03};
    Vec3   leafTint{1, 1, 1};   // multiplies the foliage after the shape picks its palette (a variant's shift)
};

struct StylizedTree {
    RenderMesh bark;     // opaque, vertex-coloured
    RenderMesh canopy;   // opaque (palm fronds want two-sided drawing), vertex-coloured
};

StylizedTree stylizedTree(uint32_t seed, const StylizedTreeParams& p);

}  // namespace engine

#endif
