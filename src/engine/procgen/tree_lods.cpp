#include "tree_lods.h"

#include "branch_cards.h"
#include "foliage_kit.h"
#include "material_recipes.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
uint32_t nameHash(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char ch : s) h = (h ^ ch) * 16777619u;
    return h;
}
}  // namespace

LeafCardAtlas recipeLeafAtlas(const TreeRecipe& r, const TreeMesh& tm) {
    const uint32_t h = nameHash(r.name);
    if (tm.foliage == 3 || tm.foliage == 4) {   // BLADES, FRONDS: the strap leaf's veins (material_recipes.h)
        const BarkTextures bt = bladeTextures(h);
        LeafCardAtlas a;
        a.albedo = bt.albedo;
        a.normal = bt.normal;
        return a;
    }
    if (tm.foliage == 2)   // NEAR LEAVES: each leaf / blossom one whole picture of the species' (foliage_kit.h)
        return foliageElementAtlas(r.card, r.leafKit.blossomSize > 0.0, h);
    if (tm.foliage == 1)   // NEAR FOLIAGE: sprays wearing the species' strands (foliage_kit.h), tiles its shape
        return leafCardAtlas(r.strand, h, std::max(32, static_cast<int>(std::lround(436.0 * r.spray.aspect))), 436);
    if (r.card.set)        // the species' own leaf cards, drawn by its card grammar (leaf_card.h)
        return leafCardAtlas(r.card, h);
    return LeafCardAtlas{};
}

std::vector<RecipeTreeLevel> growRecipeLevels(const TreeRecipe& r, uint32_t seed, const std::map<std::string, double>& overrides,
                                              const RecipeLodBands& bands, bool lod, BranchCardSet* cardCache) {
    std::vector<RecipeTreeLevel> out;
    const bool nearKit = r.leafKit.set || (r.spray.set && r.strand.set);
    // THE BRANCH CARDS (branch_cards.h): every level the wood to sprigs and a card a sprig, the cards baked once
    // (or taken from the caller's cache: a forest's variants of a species wear one atlas) from a donor grown whole
    const BranchCardSpec& bc = r.branchCards;
    if (bc.set && lod && !overrides.count("foliage") && !overrides.count("sprigSize")) {
        // (the pictures of the nearest foliage the species has: leaves one by one, sprays, else its own cards)
        const double foliage = r.leafKit.set ? 2.0 : nearKit ? 1.0 : 0.0;
        BranchCardSet local;
        BranchCardSet& set = cardCache ? *cardCache : local;
        if (set.empty()) {
            std::map<std::string, double> o = overrides;
            o["foliage"] = foliage;
            o["sprigSize"] = bc.sprig;
            o["sprigDonor"] = 1.0;
            const TreeMesh donor = growRecipeTree(r, seed, o);
            set = bakeBranchCards(donor, recipeLeafAtlas(r, donor), bc);
        }
        if (!set.empty()) {
            // each level: its wood's coarseness (sides apart, degrees a ring), joints, and the range it draws over
            auto level = [&](double edge, double angle, bool joints, bool roots, double woodMin) {
                std::map<std::string, double> o = overrides;
                o["foliage"] = foliage;
                o["sprigSize"] = bc.sprig;
                if (!o.count("woodEdge")) o["woodEdge"] = edge;
                if (!o.count("ringAngle")) o["ringAngle"] = angle;
                if (!joints) o["joints"] = 0.0;
                else if (!o.count("jointMinRadius") && edge > bc.woodEdge) o["jointMinRadius"] = 0.10;   // (the trunk and main limbs)
                if (!roots && !o.count("rootCount")) o["rootCount"] = 0.0;
                if (woodMin > 0.0 && !o.count("woodMinRadius")) o["woodMinRadius"] = woodMin;
                RecipeTreeLevel L;
                L.tree = growRecipeTree(r, seed, o);
                wearBranchCards(L.tree, set, bc, seed ^ 0x9e3779b9u);
                L.atlas = set.atlas;
                return L;
            };
            auto fadeInto = [&](RecipeTreeLevel& next, double b0, double b1) {
                RecipeTreeLevel& prev = out.back();
                prev.out0 = static_cast<float>(b0); prev.out1 = static_cast<float>(b1);
                prev.maxDistance = b1 + 4.0;
                next.in0 = static_cast<float>(b0); next.in1 = static_cast<float>(b1);
                next.minDistance = std::max(0.0, b0 - 4.0);
                out.push_back(std::move(next));
            };
            out.push_back(level(bc.woodEdge, bc.ringAngle, true, true, 0.0));                       // hero
            if (bands.hero1 > bands.hero0) {
                RecipeTreeLevel M = level(bc.woodEdge * 1.6, bc.ringAngle * 1.5, true, true, 0.0);   // mid
                fadeInto(M, bands.hero0, bands.hero1);
            }
            if (bands.mid1 > bands.mid0) {
                RecipeTreeLevel Lt = level(bc.woodEdge * 2.5, bc.ringAngle * 2.0, false, false, 0.0);   // light
                fadeInto(Lt, bands.mid0, bands.mid1);
                if (bands.light1 > bands.light0) {
                    RecipeTreeLevel F = level(0.25, 30.0, false, false, 0.06);                       // far
                    fadeInto(F, bands.light0, bands.light1);
                }
            }
            return out;
        }
    }
    const bool heroMid = lod && nearKit && !overrides.count("foliage") && bands.hero1 > bands.hero0;
    const bool light = bands.mid1 > bands.mid0 && !r.blades.set;
    auto grow = [&](std::map<std::string, double> o) {
        RecipeTreeLevel L;
        L.tree = growRecipeTree(r, seed, o);
        L.atlas = recipeLeafAtlas(r, L.tree);
        return L;
    };
    if (heroMid) {
        std::map<std::string, double> hero = overrides, mid = overrides;
        hero["foliage"] = r.leafKit.set ? 2.0 : 1.0;
        // the mid wears the same leaves, coarser (TreeParams::leafLod): crossed cards showed as cards even at 70 m
        // (Glenn, 2026-10-17); a spray species keeps its cards for now
        mid["foliage"] = r.leafKit.set ? 2.0 : 0.0;
        if (r.leafKit.set && !mid.count("leafLod")) mid["leafLod"] = 2.2;
        // ...and its finest twigs (under a pixel from 20 m) go, the rest on 0.6 of the sides: the mid's cost was its bark
        if (!mid.count("woodMinRadius")) mid["woodMinRadius"] = 0.015;
        if (!mid.count("ringSegments"))
            mid["ringSegments"] = std::max(5.0, std::round(0.6 * drawTreeParams(r, seed).ringSegments));
        if (!mid.count("jointMinRadius")) mid["jointMinRadius"] = 0.10;   // (the trunk and main limbs)
        RecipeTreeLevel H = grow(hero), M = grow(mid);
        H.out0 = static_cast<float>(bands.hero0); H.out1 = static_cast<float>(bands.hero1); H.maxDistance = bands.hero1 + 4.0;
        M.in0 = static_cast<float>(bands.hero0); M.in1 = static_cast<float>(bands.hero1); M.minDistance = std::max(0.0, bands.hero0 - 4.0);
        out.push_back(std::move(H));
        out.push_back(std::move(M));
    } else {
        out.push_back(grow(overrides));
    }
    // the LIGHT tree (and past it the FAR one): the same skeleton (the same seed, the same draw) without joints, twigs or
    // most of its sides, the leaves coarser still (or fewer cards)
    auto lighter = [&](double leafLod, double wood, double woodShrub, double sides, double cards, double cardsShrub,
                       double b0, double b1) {
        std::map<std::string, double> o = overrides;
        if (!r.fronds.set) o["foliage"] = r.leafKit.set ? 2.0 : 0.0;
        if (r.leafKit.set && !o.count("leafLod")) o["leafLod"] = leafLod;
        o["joints"] = 0.0;
        if (!o.count("woodMinRadius")) o["woodMinRadius"] = r.shrub ? woodShrub : wood;
        if (!o.count("ringSegments")) o["ringSegments"] = sides;
        if (!o.count("rootCount")) o["rootCount"] = 0.0;
        if (!o.count("maxLeafCards") && !r.leafKit.set) o["maxLeafCards"] = r.shrub ? cardsShrub : cards;   // (cards)
        RecipeTreeLevel F = grow(o);
        RecipeTreeLevel& prev = out.back();
        prev.out0 = static_cast<float>(b0); prev.out1 = static_cast<float>(b1);
        prev.maxDistance = b1 + 4.0;
        F.in0 = static_cast<float>(b0); F.in1 = static_cast<float>(b1);
        F.minDistance = std::max(0.0, b0 - 4.0);
        out.push_back(std::move(F));
    };
    if (light) lighter(3.5, 0.035, 0.012, 5.0, 4000.0, 500.0, bands.mid0, bands.mid1);
    if (light && bands.light1 > bands.light0) lighter(6.0, 0.07, 0.025, 4.0, 1500.0, 200.0, bands.light0, bands.light1);
    return out;
}

}  // namespace engine
