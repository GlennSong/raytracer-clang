#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_LOT_CITY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_LOT_CITY_H

#include "city_lots.h"       // LotParams, LotBuilding, LotPlanDebug
#include "lot_mesh.h"

namespace engine {

// THE LOT SYSTEM AS A CITY PASS (docs/lot-system-plan.md §15.3): blocks in,
// the same outputs `growLotBuildings` produces out.
//
// This is deliberately shaped as a DROP-IN for the old pass rather than as a
// parallel universe. It fills the same `LotBuilding` records (so colliders,
// places and the HLOD read it unchanged), the same PartId-merged part list (so
// the loader binds the same materials and chunks it the same way) and the same
// LOD1 twin. A level turns it on with one flag and everything downstream is
// none the wiser — which is the only honest way to migrate a pipeline that a
// shipping level depends on.
//
// The whole chain runs here: parcelBlock -> lot_program -> layoutSite ->
// buildFromRecipe -> meshBuilding. The district's program mix comes from the
// nearest hub's kind, exactly as the road grain does, so "what this land is
// for" is answered once and the streets and the buildings agree about it.
// `enclosedCount`: how many leading entries of `blocks` are real graph FACES,
// bounded by streets on every side. Anything past it is a synthesized rim strip,
// open on its far side — and open ground is what admits the campus-scale
// programs (university, office park, big-box, works), because their parcels are
// larger than any enclosed block can offer. Whether a block is enclosed is a
// GEOMETRIC fact; deriving it from the district kind instead meant a rim strip
// beside a residential hub was parcelled at house grain.
void growLotSystemBlocks(const std::vector<Poly2>& blocks, std::size_t enclosedCount,
                         const LotParams& params,
                         LotPlanDebug* debug, std::vector<RenderMesh>* outParts,
                         std::vector<RenderMesh>* outFlatParts,
                         std::vector<LotBuilding>* outLots);

// The stock program mix for a metro district kind (CityHub::kind: 0 financial,
// 1 commercial, 2 residential, 3 old town, 4 industrial). Shared with the road
// layer's grain derivation, so a district cannot lay one kind of street and
// build another kind of building.
ProgramSet programsForDistrict(int kind);

}  // namespace engine

#endif
