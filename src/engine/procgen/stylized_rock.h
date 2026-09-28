#ifndef RAYTRACER_ENGINE_PROCGEN_STYLIZED_ROCK_H
#define RAYTRACER_ENGINE_PROCGEN_STYLIZED_ROCK_H

// THE ROCK LIBRARY (the flora plan): stylized stones in a few FAMILIES and MATERIALS, so any
// biome gets rocks that belong there. Low-poly and semi-faceted (crisp facets, softened a
// little), colour varied per facet, MOSS on the faces that look up, and the base darkened
// where it meets the ground (with the bedding, the seam reads as contact, not a cut).
//   boulder  a squat lumpy mass           slab     flat and broad, a cut top
//   pebbles  a cluster of small stones     outcrop  tall and jagged, planar breaks
// Materials: granite, sandstone, basalt, mossy (granite, heavily mossed). The mesh carries only
// light (a per-facet shade, a dark foot); the stone's colour and moss are its MATERIAL's -- a
// triplanar texture from material_recipes.h and the top-layer feature (ADR-0098).

#include "../../renderer/renderer.h"   // RenderMesh

#include <cstdint>
#include <string>

namespace engine {

enum class RockFamily { Boulder, Slab, Pebbles, Outcrop };
enum class RockMaterial { Granite, Sandstone, Basalt, Mossy };
bool rockFamilyFromName(const std::string& n, RockFamily& out);
bool rockMaterialFromName(const std::string& n, RockMaterial& out);

struct StylizedRockParams {
    RockFamily family = RockFamily::Boulder;
    RockMaterial material = RockMaterial::Granite;
    double size = 1.5;    // across (m); each seed varies it
    double moss = -1.0;   // 0..1 moss on upward faces; < 0 = the material's own (mossy 0.8, else 0.2)
};

// Origin at the base centre, +Y up; the stone rises from y = 0 (bed it in by ~a quarter).
RenderMesh stylizedRock(uint32_t seed, const StylizedRockParams& p);
// The stone's default moss (top-layer amount): mossy 0.9, others light.
double rockDefaultMoss(RockMaterial m);

}  // namespace engine

#endif
