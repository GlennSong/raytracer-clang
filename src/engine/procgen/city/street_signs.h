#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STREET_SIGNS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STREET_SIGNS_H

// STREET NAME SIGNS (Glenn, 2026-09-19: "It would be nice to have in world signs
// for streets. I think you'd have to composite textures and store a list of all
// the street sign textures and map them to the right signs and make sure they
// fit and are legible").
//
// Three steps, all pure (no renderer, no ECS -- the level loader uploads):
//   1. PLAN: at every junction of the road model where two or more named streets
//      meet (SignCorner: the model's junction, its arms and their names, ADR-0420;
//      nothing here decides what a junction or a street is), one post on the most
//      open kerb corner (clear of the carriageways and of the signal poles), with a
//      blade per street -- up to three, stacked and crossed -- each blade parallel
//      to its street. Where the post stands is placement: it keeps clear of the
//      level's carriageways as drawn (the road graph's edges and widths).
//   2. ATLAS: each signed street's name composited onto a green blade (white
//      border, white Overpass lettering) and shelf-packed into 2048 px pages.
//      The suffix is abbreviated as on real blades (Boulevard -> Blvd); a name
//      that still will not fit the longest blade is condensed, then -- within
//      a floor -- set smaller; every
//      blade records what it did and how tall its capitals came out, so a
//      test can hold the whole city to "fits and is legible".
//   3. MESHES: blade quads (the name reads correctly from both sides) merged
//      per 280 m cell and atlas page, so signs cull street by street; posts as
//      instance transforms.

#include "../../../rt_math.h"
#include "../../text/font.h"
#include "road_network.h"
#include "street_names.h"
#include "../../../renderer/renderer.h"   // RenderMesh

#include <nlohmann/json.hpp>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace engine {

// The blade's shape and colours are CONTENT: assets/data/streets.json "signs"
// (Glenn, 2026-09-20). streetSignParams() reads them; the level supplies only
// what it alone knows (its sidewalk width).
struct StreetSignParams {
    Real sidewalkWidth = 3.5;     // the band the post stands in
    Real kerbGap = 1.0;           // post centre this far past the carriageway edge
    Real bladeHeight = 0.30;      // metres: a 12-inch street-name blade
    Real minBladeWidth = 0.75;
    Real maxBladeWidth = 1.8;     // 6 ft: the longest blade a pole carries
    Real topBladeY = 3.05;        // upper blade centre above the post foot
    Real bladeStep = 0.34;        // the next blade down
    int bladePx = 80;             // atlas pixels per blade height
    int pagePx = 2048;            // atlas page size
    Real cellSize = 280.0;        // mesh cells (as the street lamps)
    // Lettering: capitals this tall as a fraction of the blade, never set
    // smaller than the floor to make a long name fit.
    float capFraction = 0.56f;
    float capFloorFraction = 0.40f;
    uint8_t face[4] = {0, 104, 64, 255};        // US street-name green
    uint8_t legend[4] = {246, 246, 240, 255};   // the lettering and border
};

// Read the "signs" object of the streets asset over `p`.
void applyStreetSignData(const nlohmann::json& signs, StreetSignParams& p);
// The shipped blade (assets/data/streets.json), read once.
const StreetSignParams& streetSignParams();

// A JUNCTION TO SIGN, as the road model states it (the loader fills these from the level's model: road_model.h
// namedArmsAt): which city and junction, and each arm of it that carries a name -- where the arm meets the junction,
// the way it leaves, its carriageway's half width (the street's stated travel lanes) and its name. Arms with no name
// (a ramp, a freeway) are not here: their carriageways are cleared as the level's roads are.
struct SignCorner {
    int city = -1, junction = -1;
    struct Arm {
        int arm = -1;                 // the junction's arm (its index in the model)
        Vec2 point;                   // where it meets the junction
        Vec2 dir{1, 0};               // unit: the way it leaves the junction
        Real halfWidth = 0;
        std::string name;
    };
    std::vector<Arm> arms;
};

struct StreetSignBlade {
    std::string name;             // the street it names (as the road model states it)
    int arm = -1;                 // an arm of the junction that street is (the first, round the junction)
    Vec2 along{1, 0};             // unit XZ: the street's line at the junction
    Real y = 0;                   // centre height above the post foot
};

struct StreetSignPost {
    int city = -1, junction = -1; // the road model's junction (SignCorner)
    Vec3 base;                    // foot, on the sidewalk
    std::vector<StreetSignBlade> blades;
};

// `carriageways`: the level's roads as drawn (edges with their widths), which a post keeps clear of.
std::vector<StreetSignPost> planStreetSigns(
    const std::vector<SignCorner>& corners, const RoadGraph& carriageways,
    const std::function<Real(Real, Real)>& ground, const StreetSignParams& p,
    const std::vector<Vec3>& avoid = {});

// One street's blade in the atlas.
struct SignBlade {
    std::string text;             // what is lettered (maybe abbreviated)
    int page = 0;
    float u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    int wPx = 0, hPx = 0;         // the blade's own pixels (no padding)
    float textPx = 0;             // lettering width
    float capPx = 0;              // capital height, pixels (legibility)
    float xScale = 1.0f;          // condensing applied (1 = none)
    bool abbreviated = false;
    Real widthM() const;          // world width, from the pixel aspect
    Real heightM = 0.30;
};

struct SignAtlas {
    std::vector<TextImage> pages;
    std::map<std::string, SignBlade> blades;   // the street's name -> its blade
};

SignAtlas buildSignAtlas(const Font& font, const std::vector<StreetSignPost>& posts,
                         const StreetSignParams& p);

struct StreetSignMeshes {
    struct Cell {
        int page = 0;
        RenderMesh mesh;          // blades, world coordinates
        Vec3 centre;
        Real radius = 0;
    };
    std::vector<Cell> blades;
    std::vector<Mat4> posts;      // post transforms (foot at origin, +Y up, Y scaled to height)
    RenderMesh postMesh;          // one post, unit height
};

StreetSignMeshes buildStreetSignMeshes(const std::vector<StreetSignPost>& posts,
                                       const SignAtlas& atlas, const StreetSignParams& p);

}  // namespace engine

#endif
