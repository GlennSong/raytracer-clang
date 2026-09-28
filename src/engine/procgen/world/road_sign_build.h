#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_ROAD_SIGN_BUILD_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_ROAD_SIGN_BUILD_H

// ROAD SIGNS IN 3D (ADR-0110): the sign plan (road_signs.h) built -- each face rasterized from the
// SAME layout the SVG sign sheet draws (layoutSign), packed into atlas pages that are CACHED on disk
// by their content (Glenn: "These signs would be built and cached offline?" -- yes: a level loads the
// pages, it does not letter them), and the structures that carry them:
//   roadside  -- one post, or two under a panel wider than 1.6 m; a guide sign's bottom 2.1 m up, a
//                regulatory sign's (DO NOT ENTER, WRONG WAY) 1.5 m;
//   overhead  -- a gantry: two uprights either side of the carriageway and a truss across, the panel
//                hung on it 5.6 m over the road.
// A panel's texture reads left to right as its reader sees it in the WORLD (u runs along
// cross(forward, up)), so the plan/world mirror never reverses the lettering.

#include "road_signs.h"
#include "../../mesh_builder.h"
#include "../../text/font.h"

#include <functional>
#include <string>
#include <vector>

namespace engine {

// One face, drawn at `pxPerMetre`; `tabAbove` gets the metres of exit tab over the panel's top edge.
TextImage rasterizeSignFace(const SignFace& f, const Font& font, double pxPerMetre, double* tabAbove = nullptr);

struct RoadSignAtlas {
    std::vector<TextImage> pages;
    struct Slot {
        int page = -1;
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0;   // the face (tab included) on its page
        double w = 0, h = 0, tab = 0, tabW = 0; // metres: the panel, and the tab above it (its height, its width)
    };
    std::vector<Slot> slots;                    // per sign, in the plan's order
    std::string key;                            // the content hash it is cached under
    bool fromCache = false;
};

// Bake every sign's face into pages (shelf-packed). With a `cacheDir`, the pages are looked up there
// first by a hash of the signs' faces, and written there when baked.
RoadSignAtlas bakeRoadSignAtlas(const std::vector<IslandSign>& signs, const Font& font, const std::string& cacheDir = {},
                                double pxPerMetre = 64.0, int pageSize = 2048);

struct RoadSignMeshes {
    struct Panels { int page = 0; RenderMesh mesh; Vec3 centre; double radius = 0; };
    struct Steel { RenderMesh mesh; Vec3 centre; double radius = 0; };
    std::vector<Panels> panels;   // textured faces, a mesh per atlas page per 400 m cell
    std::vector<Steel> steel;     // posts, gantries, the panels' backs (vertex-coloured), per cell
};

// `ground(x, z)`: the height a post stands on. `carriageHalf`: an overhead sign's gantry spans this
// either side of its sign (the carriageway and its shoulder).
RoadSignMeshes buildRoadSignMeshes(const std::vector<IslandSign>& signs, const RoadSignAtlas& atlas,
                                   const std::function<double(double, double)>& ground, double carriageHalf = 11.0);

// A sign plan as JSON ([{kind, at, facing, mount, legend}]) and back.
nlohmann::json roadSignsToJson(const std::vector<IslandSign>& signs);
std::vector<IslandSign> roadSignsFromJson(const nlohmann::json& j);

}  // namespace engine

#endif
