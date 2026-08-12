#include "lot_city.h"

#include "parcel_block.h"
#include "material_set.h"
#include "rng.h"
#include "../../../log.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// Merge a building's parts into the city-wide per-PartId list. The old pass
// merges the same way, so a chunker downstream cannot tell which pass built a
// given triangle — which is the point.
void mergeParts(std::vector<RenderMesh>& dst, const BuildingMesh& src,
                const Vec2& offset, Real baseY) {
    for (const RenderMesh& p : src.parts) {
        if (p.indices.empty()) continue;
        const std::size_t idx = static_cast<std::size_t>(p.materialIndex);
        if (idx >= dst.size()) continue;
        RenderMesh& out = dst[idx];
        const uint32_t base = static_cast<uint32_t>(out.vertices.size());
        out.vertices.reserve(out.vertices.size() + p.vertices.size());
        for (Vertex v : p.vertices) {
            v.position = Vec3(v.position.x + offset.x, v.position.y + baseY,
                              v.position.z + offset.y);
            out.vertices.push_back(v);
        }
        out.indices.reserve(out.indices.size() + p.indices.size());
        for (uint32_t i : p.indices) out.indices.push_back(base + i);
    }
}

// The nearest hub's district kind — the same question the road grain asks, so
// the two cannot disagree about what a piece of land is for.
int districtKindAt(const LotParams& p, const Vec2& q) {
    if (p.hubs.empty()) return 2;                 // no hubs: an ordinary suburb
    int best = 0;
    Real bd = 1e30;
    for (std::size_t i = 0; i < p.hubs.size(); ++i) {
        const Real d = (p.hubs[i].first - q).lengthSquared();
        if (d < bd) { bd = d; best = static_cast<int>(i); }
    }
    return std::max(0, std::min(4, p.hubs[best].second));
}

// The recipe library names more place types than the SIM knows: it has hotels,
// schools, hospitals, works and parking decks, while citysim's PlaceType is
// home/shop/office/park/civic. An unrecognised tag is dropped with a warning, so
// those buildings would stand there with nobody able to route to them — present
// but not part of the city. Mapping them to the nearest kind the sim models is
// the honest stopgap until PlaceType grows.
std::string placeTypeFor(const std::string& recipeType) {
    if (recipeType == "hotel") return "home";
    if (recipeType == "school" || recipeType == "hospital") return "civic";
    if (recipeType == "work" || recipeType == "parking") return "office";
    return recipeType;   // home, shop, office, civic, park pass through
}

}  // namespace

ProgramSet programsForDistrict(int kind) {
    switch (std::max(0, std::min(4, kind))) {
        case 0:  return downtownPrograms();
        case 1:  return commercialPrograms();
        case 3:  return residentialPrograms();     // old town: houses, tight grain
        case 4:  return rimPrograms();
        default: return residentialPrograms();
    }
}

void growLotSystemBlocks(const std::vector<Poly2>& blocks, std::size_t enclosedCount,
                         const LotParams& params,
                         LotPlanDebug* debug, std::vector<RenderMesh>* outParts,
                         std::vector<RenderMesh>* outFlatParts,
                         std::vector<LotBuilding>* outLots) {
    PaletteLibrary palettes = stockPalettes();
    RecipeBook book = stockRecipes();

    if (outParts && outParts->size() < static_cast<std::size_t>(PartId::Count)) {
        outParts->resize(static_cast<std::size_t>(PartId::Count));
        for (int i = 0; i < static_cast<int>(PartId::Count); ++i)
            (*outParts)[i].materialIndex = i;
    }
    if (outFlatParts &&
        outFlatParts->size() < static_cast<std::size_t>(PartId::Count)) {
        outFlatParts->resize(static_cast<std::size_t>(PartId::Count));
        for (int i = 0; i < static_cast<int>(PartId::Count); ++i)
            (*outFlatParts)[i].materialIndex = i;
    }

    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& face = blocks[bi];
        if (face.size() < 3) continue;
        Shape2 block = shapeFromPoly(face, EdgeTag::Street);
        orient(block);
        if (area(block) < 120.0) continue;

        const Vec2 centre = centroid(block);
        const int kind = districtKindAt(params, centre);
        ProgramSet mix = programsForDistrict(kind);

        // A FACE IS NOT ALWAYS A BLOCK. When the road fill fails — a recipe with
        // no local streets, a site the fabric skipped — extractBlocks hands back
        // a face a kilometre across, and parcelling that at house grain is
        // thousands of buildings and gigabytes of triangles. (It OOM-killed the
        // first end-to-end run.) A block that could hold dozens of real blocks
        // is unbuilt land, and saying so out loud beats dying or, worse,
        // silently building a slum the size of a district.
        const BlockGrain grain = blockGrainFor(mix);
        const Real sane = grain.depth * grain.length * 6.0;
        if (area(block) > sane) {
            LOG_WARN << "[lotcity] face " << bi << " is " << area(block)
                     << " m2, over " << sane
                     << " for this district's grain — no streets reach it, left "
                        "unbuilt";
            continue;
        }
        // Coreness is distance from downtown, normalized the same way the old
        // pass does it — it is what lets the skyline rise toward the centre
        // rather than by dice.
        const Real dist = (centre - params.center).length();
        const Real coreness =
            std::max(Real(0), std::min(Real(1), 1.0 - dist / std::max(params.midRadius *
                                                                     2.2, Real(1))));
        // Enclosed means "streets on every side", which is a fact about the
        // BLOCK, not about the district it sits in. Reading it off the district
        // kind (`kind != 4`) meant a rim strip next to a residential hub was
        // treated as an enclosed block and parcelled at house grain, so
        // unconstrained ground never got the campus-scale programs it exists
        // for. The caller knows which blocks came out of extractBlocks.
        const bool enclosed = bi < enclosedCount;

        BlockParcelParams pp;
        // A BLOCK FACE IS A ROAD CENTRELINE, not a kerb. extractBlocks walks the
        // road graph, so the face runs down the middle of the carriageway, and
        // the buildable interior starts a half-carriageway plus its sidewalk in
        // from it — which is exactly what LotParams::roadMargin measures
        // (city_lots.h:69) and exactly what the shipping pass insets by before
        // it parcels (city_lots.cpp:1223).
        //
        // This used to subtract a flat 8 m, converting to the parceller's own
        // margin — a different quantity (parcel_block.h: the verge PAST the
        // kerb, default 2 m). The result was a 3 m inset from the centre of a
        // 12 m road, so the first lot began inside the carriageway and its
        // building stood in the street. Pass the distance the block face
        // actually needs and let the two pipelines agree on it.
        pp.roadMargin = std::max(Real(0.5), params.roadMargin);
        const std::uint32_t seed =
            params.seed * 2654435761u + static_cast<std::uint32_t>(bi) * 40503u + 7u;
        ParcelledBlock pb = parcelBlock(block, mix, pp, enclosed, coreness,
                                        kind == 1 ? StreetClass::Arterial
                                                  : StreetClass::Street,
                                        seed);
        if (debug) {
            debug->blocks.push_back(tessellate(pb.parcellable.outer, 0.3));
            for (const ParcelledLot& l : pb.lots)
                debug->lots.push_back(tessellate(l.shape.outer, 0.3));
        }

        for (std::size_t li = 0; li < pb.lots.size(); ++li) {
            const ParcelledLot& lot = pb.lots[li];
            if (lot.program < 0 ||
                lot.program >= static_cast<int>(mix.programs.size()))
                continue;
            const LotProgram& prog = mix.programs[lot.program];
            const std::uint32_t lotSeed =
                seed + static_cast<std::uint32_t>(li) * 2246822519u;

            SitePlan site = layoutSite(lot.shape, lot.tags, prog, lotSeed);
            const ZoneArea* envelope = nullptr;
            for (const ZoneArea& z : site.zones)
                if (z.zone == Zone::Building) envelope = &z;
            if (!envelope || envelope->parts.empty()) continue;

            // The program's declared height range binds as much as the plate
            // does: the recipe is asked for the smaller of the two.
            LotTags tags = lot.tags;
            tags.maxStoreys = std::min(tags.maxStoreys, static_cast<Real>(prog.maxStoreys));
            // WEIGHTED, and hashed. This picked `prog.recipes[lotSeed % n]`:
            // uniform over the program's recipes, so LotRecipe::weight — which
            // is authored on all 37 of them and says a glass tower is rarer than
            // a walk-up — was read by nothing at all. `%` on a raw seed is also
            // a poor hash: lotSeed advances by a fixed stride per lot, so a
            // block's recipes cycled in lockstep instead of varying.
            const LotRecipe* rec = nullptr;
            if (!prog.recipes.empty()) {
                Real total = 0;
                for (const std::string& name : prog.recipes)
                    if (const LotRecipe* r = book.byName(name))
                        total += std::max(Real(0), r->weight);
                Rng pick(lotSeed ^ 0x5bf03635u);
                Real r = pick.unit() * total;
                for (const std::string& name : prog.recipes) {
                    const LotRecipe* cand = book.byName(name);
                    if (!cand) continue;
                    rec = cand;                       // last valid, as the fallback
                    r -= std::max(Real(0), cand->weight);
                    if (r <= 0) break;
                }
            }
            if (!rec) continue;

            // A LONG ENVELOPE IS A ROW, NOT A LONG BUILDING. Sixty metres of
            // frontage is a terrace everywhere in the world; building it as one
            // mass gives the sixty-metre house nobody lives in. Cut it at the
            // program's OWN declared unit width, so a cottage row and an office
            // plate each terrace at their own grain from one rule.
            const std::vector<Shape2> units =
                terraceEnvelope(envelope->parts[0], prog.targetWidth());

            for (std::size_t ui = 0; ui < units.size(); ++ui) {
                const Shape2& unit = units[ui];
                if (area(unit) < 20.0) continue;
                // Each unit is its own building: its own seed, so a terrace is a
                // row of related houses rather than one house stamped N times.
                const std::uint32_t unitSeed =
                    lotSeed + static_cast<std::uint32_t>(ui) * 2654435761u;

                BuiltBuilding bb =
                    buildFromRecipe(*rec, unit, tags, palettes, seed, unitSeed);
                if (bb.surfaces.levels.empty()) continue;

                const Real baseY =
                    params.ground ? params.ground(centroid(unit).x, centroid(unit).y)
                                  : Real(0);

                LotMeshParams mp;
                mp.seed = unitSeed;
                mp.retail = rec->placeType == "shop";
                mp.baseY = baseY;
                if (outParts) {
                    BuildingMesh full = meshBuilding(bb, mp);
                    // The SITE is the lot's, not the unit's — furnish it once,
                    // with the first unit, or a terrace gets N copies of the
                    // same front path stacked on each other.
                    if (ui == 0) {
                        SiteFurnishing furn = furnishSite(site, prog, lotSeed);
                        meshSiteInto(full, site, furn, baseY);
                    }
                    mergeParts(*outParts, full, Vec2(0, 0), 0);
                }
                if (outFlatParts) {
                    LotMeshParams fp = mp;
                    fp.detail = FacadeDetail::Flat;
                    fp.proxy = false;
                    mergeParts(*outFlatParts, meshBuilding(bb, fp), Vec2(0, 0), 0);
                }

                if (outLots) {
                    // The collider/HLOD record. The PLAN polygon is the real
                    // footprint, so a courtyard or an L keeps its shape instead
                    // of spilling onto the sidewalk as a box would.
                    LotBuilding lb;
                    const Shape2& foot = bb.surfaces.levels[0].plans[0];
                    lb.plan = tessellate(foot.outer, 0.25);
                    lb.site = centroid(foot);
                    Vec2 lo, hi;
                    bounds(foot, lo, hi);
                    lb.width = hi.x - lo.x;
                    lb.depth = hi.y - lo.y;
                    lb.height = bb.surfaces.levels.back().y1;
                    lb.yaw = 0;
                    lb.type = placeTypeFor(rec->placeType);
                    lb.recipe = rec->name;
                    lb.baseY = baseY;
                    lb.groundY = baseY;
                    lb.color = bb.materials.forTier(0).wallColor;
                    outLots->push_back(std::move(lb));
                }
            }
        }

        // Open space the parceller kept BY DESIGN becomes a green lot, exactly
        // as the old pass reports its unbuilt lots — so the host plants it.
        if (outLots)
            for (const Shape2& g : pb.openSpace) {
                if (area(g) < 40.0) continue;
                LotBuilding lb;
                lb.pad = tessellate(g.outer, 0.4);
                lb.site = centroid(g);
                lb.type = "green";
                lb.height = 0;
                outLots->push_back(std::move(lb));
            }
    }
}

}  // namespace engine
