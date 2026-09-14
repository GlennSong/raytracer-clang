#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_LOT_CACHE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_LOT_CACHE_H

// Bundle codecs for the lot pass's products (ADR-0084, milestone B): what growLotBuildings returns —
// the lots with their plans, pads, units and building parameters, the plan debug record, the block
// terraces, and the grown part meshes (LOD0 and the LOD1 twin) — so a level can skip the grower and run
// everything after it (chunking, HLOD, colliders, trees) unchanged on data read back from disk.
//
// BuildingParams and OpeningStyle have no JSON or property description, so they are written field by
// field here. A new field in either struct must be added to putBuildingParams/getBuildingParams AND
// kLotsFormatVersion bumped; tests/test_lot_cache.cpp sets every field and compares.

#include "engine/bundle/bundle.h"
#include "engine/bundle/binary_stream.h"
#include "engine/procgen/city/city_lots.h"
#include "engine/procgen/terrain.h"

#include <string>
#include <vector>

namespace engine {
namespace lotcache {

constexpr int kLotsFormatVersion = 3;   // 2: grown parts stored per render cell when written with cell > 0

void putBuildingParams(bundle::BinWriter& w, const BuildingParams& p);
bool getBuildingParams(bundle::BinReader& r, BuildingParams& p);
void putLots(bundle::BinWriter& w, const std::vector<LotBuilding>& lots);
bool getLots(bundle::BinReader& r, std::vector<LotBuilding>& lots);
void putLotPlan(bundle::BinWriter& w, const LotPlanDebug& plan);
bool getLotPlan(bundle::BinReader& r, LotPlanDebug& plan);
void putFlattens(bundle::BinWriter& w, const std::vector<TerrainFlatten>& f);
bool getFlattens(bundle::BinReader& r, std::vector<TerrainFlatten>& f);

// Sections under `prefix` (e.g. "lots/"): meta, lots, plan, grade, and the grown parts either whole
// (`parts/<i>`, `flat/<i>`; cell == 0) or split by render cell (`cell/<cx>_<cz>/parts/<i>`,
// `cell/<cx>_<cz>/flat/<i>`; cell > 0, MeshBuilder::chunkByCell) so the loader instantiates a cell's parts
// straight from the sections, one chunk at a time, and never chunks at load. `meta` carries `cell` and `cells`.
void writeLotResult(bundle::BundleWriter& w, const std::string& prefix, const NetLotResult& r, double cell = 0.0);
// `withParts` false leaves parts/flatParts empty (sized from meta) — the loader spawns per-cell sections instead.
// A per-cell layout is reassembled into whole parts when parts are asked for.
bool readLotResult(const bundle::Bundle& b, const std::string& prefix, NetLotResult& r, std::string* err, bool withParts = true);

struct LotCellPart { int cx = 0, cz = 0, part = 0; bool flat = false; std::string section; };
double lotCellSize(const bundle::Bundle& b, const std::string& prefix);   // meta.cell; 0 = whole parts (or no meta)
// The per-cell part sections, cell-major, LOD0 parts before LOD1 within a cell; empty for a whole-part layout.
bool listLotCellParts(const bundle::Bundle& b, const std::string& prefix, std::vector<LotCellPart>& out, std::string* err);
bool readLotPart(const bundle::Bundle& b, const std::string& section, RenderMesh& m);

}  // namespace lotcache
}  // namespace engine

#endif
