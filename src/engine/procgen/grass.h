#ifndef RAYTRACER_ENGINE_PROCGEN_GRASS_H
#define RAYTRACER_ENGINE_PROCGEN_GRASS_H

// STYLIZED GRASS CLUMPS (the flora plan, step "grass field"). A clump is a small tuft of
// real geometry blades -- tapered, bent, dark at the root and light at the tip -- not an
// alpha-cut card: cards read flat up close and pay for their alpha test in fill. Every
// normal points straight up, so a field lights like one soft carpet instead of glittering
// blade by blade (the Breath of the Wild look). GrassSystem instances a few variants of it
// on a jittered grid around the camera.

#include "../../renderer/renderer.h"   // RenderMesh
#include "texture_field.h"            // TextureData, the blade fields

#include <cstdint>

namespace engine {

struct GrassClumpParams {
    int    blades = 18;         // per clump
    double radius = 0.22;       // blades spread over a disc this wide (m)
    double height = 0.55;       // mean blade height (m); each varies 0.65-1.35x
    double width = 0.045;       // blade width at the root (m)
    double lean = 0.35;         // how far a tip leans out, as a fraction of its height
    Vec3   rootColor{0.012, 0.035, 0.006};   // linear albedo: field grass, darker at the root
    Vec3   tipColor{0.075, 0.16, 0.022};
};

// Three triangles a blade: a tapered quad to the bend, a triangle to the tip.
RenderMesh grassClump(uint32_t seed, const GrassClumpParams& p);

// THE FAR FIELD: a grass CARD -- three crossed quads (MeshBuilder::crossCards) carrying a baked
// texture of blades (fieldBlades), coloured root to tip like the clumps so the two blend where
// one fades into the other. RGBA, linear (the renderer does not decode sRGB), alpha the cut-out.
RenderMesh grassCardMesh(const GrassClumpParams& p);
TextureData grassCardTexture(uint32_t seed, const GrassClumpParams& p);

}  // namespace engine

#endif
