#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_LOT_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_LOT_MESH_H

#include "building_recipe.h"
#include "site_plan.h"
#include "shape_grammar.h"   // BuildingMesh, PartId, FacadeDetail

namespace engine {

// THE BRIDGE (docs/lot-system-plan.md §15): the Lot System describes a building
// — plans, levels, bay grids, elements, materials — and this turns that
// description into triangles.
//
// It emits into `BuildingMesh`, the type the existing city path already speaks,
// and that choice is the whole point. The loader, the renderer, the procedural
// surface bake, the box colliders, the HLOD swap and the editor's save/load
// round trip all consume `BuildingMesh`/`CityModel` already, so a building
// described the new way needs no new code anywhere downstream. The seam is one
// module wide.
//
// What it reads, and from where:
//   * `surfaces.levels`  — a Shape2 floor plate per level, with y0/y1 and tier
//   * `surfaces.grids`   — the bay grid per level per edge, computed once
//   * `fenestrate()`     — the openings for a wall, from its grid, tag and tier
//                          style. The BLUEPRINT: full and flat detail read the
//                          same list, so the two can never disagree about where
//                          a building's windows are.
//   * `placements`       — everything that is not an opening (cornices, base
//                          courses, parapets, balconies)
//   * `materials`        — the palette per tier
//
// Arcs are tessellated HERE and nowhere earlier, at the tolerance the consumer
// asks for. A curved wall stays one true arc through the whole grammar and
// becomes chords once, at the end, which is what the kernel's bulge exists for.

struct LotMeshParams {
    Real baseY = 0;
    // Full punches real openings and reveals; Flat emits a spandrel/glass band
    // per storey — the R2 distance LOD, from the same opening list.
    FacadeDetail detail = FacadeDetail::Full;
    Real chordTol = 0.12;      // arc -> chords
    Real reveal = 0.14;        // how far the glass sits behind the wall face
    bool proxy = true;         // also build the coarse HLOD mass
    bool retail = false;       // ground floor is a shopfront
    std::uint32_t seed = 1;
};

// One building, from its description. Deterministic for `params.seed`.
BuildingMesh meshBuilding(const BuiltBuilding& b, const LotMeshParams& params = {});

// The ground the building stands on: every zone the site plan allocated, as
// flat triangles with the zone's own material. Appended, so a caller can mesh a
// whole block into one BuildingMesh and get a handful of draws for it.
void meshSiteInto(BuildingMesh& out, const SitePlan& site,
                  const SiteFurnishing& furn, Real y);

}  // namespace engine

#endif
