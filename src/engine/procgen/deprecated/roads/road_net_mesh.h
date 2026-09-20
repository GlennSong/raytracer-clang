#ifndef RAYTRACER_ENGINE_PROCGEN_DEPRECATED_ROADS_ROAD_NET_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_DEPRECATED_ROADS_ROAD_NET_MESH_H

// DEPRECATED (2026-09-20): the lattice road builder — the road surface the city
// has been meshed with since roads-v2. It is still what every shipped level
// builds, so it still compiles and is still selectable; see the roads module
// (procgen/city/roads/road_builder.h), which reaches it through the "lattice"
// builder. New work belongs in the lanes builder, not here. This half of the
// old road_net.{h,cpp} is the MESHER; the road entity, its recipe and the
// graphs derived from it are shared road model and live in city/roads.

#include "../../city/roads/road_entity.h"   // RoadEntity, RoadLook, CurbBandAudit
#include "../../city/structure_set.h"       // StructureSet, StructureParams (buildRoadWalls)

namespace engine {

// Build the road surface for `road` (its graph swept by buildRoadNetLattice).
// `heightAt` drapes it on the level terrain (null = flat).
// `deckOut` (optional) receives the DECK the mesh rode — see RoadDeckField —
// indexed and ready to query; the loader stores it beside the road entity.
RenderMesh buildRoadNetMesh(const RoadEntity& road, const RoadGroundFn& heightAt,
                            CurbBandAudit* auditOut = nullptr,
                            RoadDeckField* deckOut = nullptr);

// Swept-lattice street mesher (street-lattice-plan.md, stage 3): sweep each chain
// as a lattice body trimmed to the junction boundary, and fill each deg>=3 node
// with a Coons junction patch that shares the bodies' mouth rings — so the
// surface is quads with interior vertices (conforms to terrain) and junctions
// interpolate height (no medial-axis step). `heightAt` drapes the streets
// (null = flat). `chainTriEndsOut` (optional, diagnostics): index-buffer
// position after each swept chain BODY, in order; back() is where bodies end
// and junction PADS begin. The surface-scan tests use it to classify which
// surfaces stack (same-chain self-fold / cross-chain / body-pad / pad-pad).
RenderMesh buildRoadNetLattice(const RoadGraph& g,
                               const std::function<Real(Real, Real)>& heightAt,
                               std::vector<std::size_t>* chainTriEndsOut = nullptr,
                               double sidewalkWidth = 3.0, double curbHeight = 0.15,
                               bool crosswalks = true, CurbBandAudit* auditOut = nullptr,
                               // 0 = the old radius-less quadratic corner. The real
                               // value arrives from RoadLook::cornerRadius via
                               // buildRoadNetMesh; hand-built graphs keep the old shape.
                               double cornerRadius = 0.0,
                               // The deck: final chains (yAbs) + pad triangles.
                               RoadDeckField* deckOut = nullptr,
                               // Grade limit per class (RoadLook::perClassGrade) or
                               // the single kRoadMaxGrade. Must match what the
                               // terrain carve is told, or deck and ground disagree.
                               bool perClassGrade = true);

// The terrain cut/fill footprints that grade the ground to this road (ADR-0044 corridor
// conforming). Traces the graph's chains, gives each a smoothed, grade-limited vertical
// profile (roadProfile, over `heightAt`), and emits a flatten ramp per segment at the
// profile, half-width = carriageway + `shoulder`, feathered over `falloff`. The loader
// folds these into the level terrain before it builds, so the ground meets the road and no
// terrain pokes through. Empty if `heightAt` is null (a flat road needs no carving).
std::vector<TerrainFlatten> roadNetConformRegions(const RoadEntity& road,
                                                  const RoadGroundFn& heightAt,
                                                  double shoulder = 1.5,
                                                  double falloff = 8.0, double maxGrade = 0.10);

// Retaining / fill walls for a road whose `heightAt` is the NATURAL (pre-carve)
// ground (ADR-0075 Phase 1). Reuses the SAME spine + grade-limited profile as
// roadNetConformRegions, so a wall stands exactly where the terrain batter clamps
// at `p.reach` — capping the residual step a steep cut/fill can't daylight, never
// double-counting it. The 3-D grade-break geometry the 2.5-D ground defers to a
// StructureSet. Empty on flat ground or when `heightAt` is null.
StructureSet buildRoadWalls(const RoadEntity& road, const RoadGroundFn& heightAt,
                            const StructureParams& p = {});

}  // namespace engine

#endif