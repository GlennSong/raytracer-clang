#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_SIGN_KIT_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_SIGN_KIT_H

// THE SIGN KIT (ahead of #241: signs planted from the regulations). Every road sign the driving model's rules need --
// STOP, YIELD, speed limits, no-turn signs, lane-use, warnings, parking, street blades, freeway guide signs -- as DATA:
// assets/scripts/signs_us.lua lists them (their plate shape and size, colours, legend, and the regulation each stands
// for) over the recipes in assets/scripts/sign_kit.lua (plate outlines, symbols, plates, posts, brackets, mast arms).
// A new sign is a new entry in the Lua; nothing here names one.
//
// How a sign is made:
//   1. THE FACE is laid out in Lua as a SignFace (road_signs.h: the same layout the road-sign system draws its guide
//      signs from, ADR-0110), with "poly" and "line" elements for the plate shape, borders and symbols, and the sign
//      font (Overpass, text/font.h) for lettering. It is rasterized by rasterizeSignFace into an ATLAS of pages, the
//      way the road-sign system's faces are: the face is a texture, not geometry and not vertex colour (see
//      docs/adr-draft-sign-kit.md for why).
//   2. NIGHT SHINE: each face is rasterized a second time with every colour replaced by how strongly that sheeting
//      returns headlight (the catalogue's `sheeting` table): a grey SHINE page per colour page, same layout. The
//      renderer does not use it yet; it is the input a retroreflection shader will read.
//   3. WEAR (wearSignFace): fade and grime painted into both pages from a 0..1 amount and a seed; the plate's bend and
//      the post's lean are done to the mesh in Lua. Same amount and seed, same bytes.
//   4. THE MODEL comes from the Lua recipe over the mesh words: textured plate faces (one part per face, on its atlas
//      page), and the metal (plate backs and rims, posts, clamps, arms), near or far.

#include "road_signs.h"                 // SignFace
#include "../../text/font.h"            // TextImage
#include "../../../renderer/renderer.h" // RenderMesh
#include "../proc_model.h"              // ProcMaterial: the engine's material for a procgen part

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// How a face wears (the catalogue's `wear_look`; every number is data, a zero here does nothing).
struct SignWearLook {
    double fade = 0;          // at wear 1, how far each colour moves toward its faded tone
    double fadeGrey = 0;      // the faded tone: this share of the colour's saturation lost...
    double fadeLift = 0;      // ...and this share of the way to white
    double fadeCell = 0.2;    // metres: the faded patches' size (sun hits a sign unevenly)
    double dirt = 0;          // at wear 1, the most grime covering a texel (0..1)
    double dirtCell = 0.1;    // metres: a grime blotch's width
    double streak = 1;        // grime runs this many times taller than wide (rain runs down)
    double dirtLow = 0;       // grime gathers toward the bottom: this much more at the bottom edge than the top
    double shineLoss = 0;     // at wear 1, the share of night shine the sheeting has lost with age
    double dirtR = 0.3, dirtG = 0.28, dirtB = 0.24;   // the grime's colour
};

// Wear `colour` (and its `shine` page, when given) in place: `wear` 0..1, `seed` any number, `pxPerMetre` the face's
// resolution (so blotches are a size in metres, not pixels). Transparent texels stay untouched.
void wearSignFace(TextImage& colour, TextImage* shine, double wear, uint32_t seed, const SignWearLook& look,
                  double pxPerMetre);

// One piece of a built sign.
struct SignModelPart {
    std::string name;   // "face" (a textured plate front), "plate_back" (backs and rims), "post" (posts, clamps, arms)
    int page = -1;      // a face: the atlas page its texture is on; -1: no texture (vertex-coloured)
    RenderMesh mesh;    // in the sign's own frame: x the reader's right, y up, z toward the reader; the post's foot at 0
    // WHAT IT IS MADE OF, from the catalogue's `materials`, aged by the request's wear: the engine's procgen material
    // (proc_model.h ProcMaterial; its metallic and roughness are set) and the base colour, which a procgen part also
    // carries as its vertex colour. `materialName` is the catalogue's name for it ("face", "plate_back", "post").
    std::string materialName;
    ProcMaterial material;
    Vec3 color{1, 1, 1};
};

struct SignModel {
    std::string label;                // what the catalogue calls it, with its parameters ("NO LEFT TURN (R3-2)")
    std::vector<SignModelPart> parts;
    nlohmann::json info;              // the catalogue's record: code, regulation, sizes, mounting, triangles
};

struct SignKitOptions {
    double pxPerMetre = 256;   // face texels per metre (a 30-inch STOP is 195 px across)
    int pageSize = 2048;
};

struct SignKitBuild {
    std::vector<TextImage> pages;        // the faces' colours (alpha: the plate's shape)
    std::vector<TextImage> shinePages;   // the same layout, grey: 255 = the brightest sheeting, 0 = none (black)
    std::vector<SignModel> models;       // one per request, in order
    std::vector<SignFace> faces;         // every distinct face, as laid out (the contact sheet draws these)
    std::vector<std::string> faceLabels; // ...and what each is
    nlohmann::json sheeting;             // the catalogue's night-shine table (ra_white, per colour)
};

}  // namespace engine

#endif
