#include "material_set.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

MaterialSet mk(const char* name, PartId wall, Vec3 wallC, PartId roof, Vec3 roofC,
               Vec3 trimC, PartId base, Vec3 accentC, FacadeStyle style,
               int minS, int maxS) {
    MaterialSet m;
    m.name = name;
    m.wall = wall;
    m.wallColor = wallC;
    m.roof = roof;
    m.roofColor = roofC;
    m.trim = PartId::Trim;
    m.trimColor = trimC;
    m.base = base;
    m.accent = PartId::Metal;
    m.accentColor = accentC;
    m.style = style;
    m.minStoreys = minS;
    m.maxStoreys = maxS;
    return m;
}

// A small, bounded shift in a colour — enough that two neighbours are not
// identical, never enough to leave the family.
Vec3 jitter(const Vec3& c, Rng& rng, Real amount) {
    const Real v = 1.0 + rng.range(-0.075, 0.075) * amount;
    const Real h = rng.range(-0.028, 0.028) * amount;
    Vec3 out(c.x * v + h, c.y * v, c.z * v - h * 0.6);
    out.x = std::max(Real(0), std::min(Real(1), out.x));
    out.y = std::max(Real(0), std::min(Real(1), out.y));
    out.z = std::max(Real(0), std::min(Real(1), out.z));
    return out;
}

}  // namespace

const char* eraName(Era e) {
    switch (e) {
        case Era::Victorian:    return "victorian";
        case Era::Edwardian:    return "edwardian";
        case Era::Interwar:     return "interwar";
        case Era::PostWar:      return "post-war";
        case Era::Modern:       return "modern";
        case Era::Contemporary: return "contemporary";
    }
    return "?";
}

const char* characterName(Character c) {
    switch (c) {
        case Character::Suburban:   return "suburban";
        case Character::OldTown:    return "old-town";
        case Character::Terraced:   return "terraced";
        case Character::Commercial: return "commercial";
        case Character::Downtown:   return "downtown";
        case Character::Civic:      return "civic";
        case Character::Industrial: return "industrial";
        case Character::Upscale:    return "upscale";
        case Character::RunDown:    return "run-down";
    }
    return "?";
}

PaletteLibrary stockPalettes() {
    PaletteLibrary lib;
    auto add = [&](MaterialSet m, Character c, Era e) {
        lib.sets.push_back(std::move(m));
        lib.character.push_back(c);
        lib.era.push_back(e);
    };

    // --- suburban: timber and painted render, low and pitched ---------------
    add(mk("clapboard_white", PartId::Siding, Vec3(0.88, 0.87, 0.83),
           PartId::Shingle, Vec3(0.26, 0.24, 0.24), Vec3(0.96, 0.95, 0.93),
           PartId::Concrete, Vec3(0.42, 0.36, 0.30), FacadeStyle::Wood, 1, 3),
        Character::Suburban, Era::PostWar);
    add(mk("clapboard_sage", PartId::Siding, Vec3(0.62, 0.67, 0.57),
           PartId::Shingle, Vec3(0.28, 0.26, 0.24), Vec3(0.93, 0.93, 0.90),
           PartId::Concrete, Vec3(0.40, 0.35, 0.28), FacadeStyle::Wood, 1, 3),
        Character::Suburban, Era::PostWar);
    add(mk("render_cream", PartId::Stucco, Vec3(0.87, 0.82, 0.71),
           PartId::Shingle, Vec3(0.32, 0.26, 0.22), Vec3(0.94, 0.92, 0.88),
           PartId::Concrete, Vec3(0.46, 0.40, 0.33), FacadeStyle::Stucco, 1, 4),
        Character::Suburban, Era::Interwar);

    // --- old town / terraced: brick, close-grained -------------------------
    add(mk("london_stock_brick", PartId::Brick, Vec3(0.67, 0.58, 0.47),
           PartId::Shingle, Vec3(0.24, 0.22, 0.23), Vec3(0.91, 0.89, 0.85),
           PartId::Concrete, Vec3(0.34, 0.30, 0.26), FacadeStyle::Brick, 2, 6),
        Character::Terraced, Era::Victorian);
    add(mk("red_stock_brick", PartId::Brick, Vec3(0.61, 0.34, 0.28),
           PartId::Shingle, Vec3(0.27, 0.20, 0.18), Vec3(0.93, 0.91, 0.87),
           PartId::Concrete, Vec3(0.36, 0.28, 0.24), FacadeStyle::Brick, 2, 6),
        Character::Terraced, Era::Victorian);
    add(mk("edwardian_buff", PartId::Brick, Vec3(0.76, 0.68, 0.55),
           PartId::Shingle, Vec3(0.30, 0.26, 0.24), Vec3(0.95, 0.94, 0.90),
           PartId::Concrete, Vec3(0.40, 0.34, 0.28), FacadeStyle::Brick, 2, 5),
        Character::OldTown, Era::Edwardian);
    add(mk("old_town_lime", PartId::Stucco, Vec3(0.83, 0.79, 0.70),
           PartId::Shingle, Vec3(0.29, 0.25, 0.24), Vec3(0.92, 0.90, 0.86),
           PartId::Concrete, Vec3(0.38, 0.33, 0.28), FacadeStyle::Stucco, 2, 5),
        Character::OldTown, Era::Victorian);

    // --- commercial high street --------------------------------------------
    add(mk("high_street_brick", PartId::Brick, Vec3(0.58, 0.42, 0.36),
           PartId::Concrete, Vec3(0.28, 0.27, 0.27), Vec3(0.90, 0.88, 0.84),
           PartId::Concrete, Vec3(0.32, 0.34, 0.38), FacadeStyle::Brick, 2, 8),
        Character::Commercial, Era::Interwar);
    add(mk("deco_limestone", PartId::Concrete, Vec3(0.82, 0.79, 0.72),
           PartId::Concrete, Vec3(0.30, 0.29, 0.28), Vec3(0.90, 0.88, 0.82),
           PartId::Concrete, Vec3(0.55, 0.45, 0.28), FacadeStyle::Sandstone, 3, 14),
        Character::Commercial, Era::Interwar);
    add(mk("post_war_panel", PartId::Concrete, Vec3(0.68, 0.68, 0.66),
           PartId::Concrete, Vec3(0.26, 0.26, 0.27), Vec3(0.84, 0.84, 0.82),
           PartId::Concrete, Vec3(0.40, 0.42, 0.45), FacadeStyle::Concrete, 3, 16),
        Character::Commercial, Era::PostWar);

    // --- downtown towers ----------------------------------------------------
    add(mk("blue_curtain_glass", PartId::Glass, Vec3(0.36, 0.46, 0.58),
           PartId::Metal, Vec3(0.22, 0.24, 0.27), Vec3(0.62, 0.66, 0.70),
           PartId::Concrete, Vec3(0.30, 0.34, 0.40), FacadeStyle::GlassCurtain,
           8, 80),
        Character::Downtown, Era::Contemporary);
    add(mk("bronze_curtain_glass", PartId::Glass, Vec3(0.44, 0.38, 0.29),
           PartId::Metal, Vec3(0.25, 0.22, 0.19), Vec3(0.60, 0.55, 0.46),
           PartId::Concrete, Vec3(0.36, 0.32, 0.26), FacadeStyle::GlassCurtain,
           8, 80),
        Character::Downtown, Era::Modern);
    add(mk("granite_shaft", PartId::Concrete, Vec3(0.46, 0.45, 0.47),
           PartId::Metal, Vec3(0.24, 0.24, 0.26), Vec3(0.72, 0.71, 0.70),
           PartId::Concrete, Vec3(0.34, 0.34, 0.36), FacadeStyle::Concrete,
           6, 45),
        Character::Downtown, Era::Modern);

    // --- civic --------------------------------------------------------------
    add(mk("civic_ashlar", PartId::Concrete, Vec3(0.84, 0.81, 0.74),
           PartId::Metal, Vec3(0.32, 0.38, 0.36), Vec3(0.93, 0.92, 0.88),
           PartId::Concrete, Vec3(0.52, 0.46, 0.30), FacadeStyle::Sandstone, 1, 8),
        Character::Civic, Era::Edwardian);
    add(mk("civic_portland", PartId::Concrete, Vec3(0.88, 0.86, 0.80),
           PartId::Metal, Vec3(0.30, 0.36, 0.34), Vec3(0.95, 0.94, 0.91),
           PartId::Concrete, Vec3(0.48, 0.44, 0.32), FacadeStyle::Sandstone, 1, 6),
        Character::Civic, Era::Victorian);

    // --- industrial ---------------------------------------------------------
    add(mk("dark_works_brick", PartId::Brick, Vec3(0.34, 0.28, 0.26),
           PartId::Metal, Vec3(0.30, 0.31, 0.32), Vec3(0.66, 0.64, 0.60),
           PartId::Concrete, Vec3(0.40, 0.42, 0.44), FacadeStyle::DarkBrick, 1, 5),
        Character::Industrial, Era::Victorian);
    add(mk("clad_shed", PartId::Metal, Vec3(0.62, 0.64, 0.66),
           PartId::Metal, Vec3(0.44, 0.46, 0.48), Vec3(0.72, 0.74, 0.76),
           PartId::Concrete, Vec3(0.36, 0.38, 0.40), FacadeStyle::Metal, 1, 3),
        Character::Industrial, Era::Modern);

    // --- upscale / run-down: the same axis, opposite ends -------------------
    add(mk("upscale_ashlar", PartId::Concrete, Vec3(0.86, 0.83, 0.76),
           PartId::Shingle, Vec3(0.26, 0.25, 0.26), Vec3(0.97, 0.96, 0.93),
           PartId::Concrete, Vec3(0.58, 0.50, 0.32), FacadeStyle::Sandstone, 2, 10),
        Character::Upscale, Era::Edwardian);
    add(mk("rundown_render", PartId::Stucco, Vec3(0.62, 0.60, 0.55),
           PartId::Shingle, Vec3(0.24, 0.23, 0.22), Vec3(0.70, 0.68, 0.64),
           PartId::Concrete, Vec3(0.32, 0.30, 0.28), FacadeStyle::Stucco, 1, 6),
        Character::RunDown, Era::PostWar);

    // Window styles follow the palette rather than being chosen separately —
    // a deco limestone block and a timber cottage do not share a window.
    for (std::size_t i = 0; i < lib.sets.size(); ++i) {
        MaterialSet& m = lib.sets[i];
        switch (m.style) {
            case FacadeStyle::GlassCurtain:
                m.window.preferredBay = 4.6;
                m.window.windowWidthFrac = 0.92;
                m.window.sill = 0.5;
                m.window.head = 2.95;
                break;
            case FacadeStyle::Wood:
                m.window.preferredBay = 2.9;
                m.window.windowWidthFrac = 0.46;
                m.window.sill = 0.95;
                m.window.head = 2.15;
                break;
            case FacadeStyle::Sandstone:
                m.window.preferredBay = 3.8;
                m.window.windowWidthFrac = 0.5;
                m.window.sill = 1.05;
                m.window.head = 2.6;
                m.window.corniceBand = true;
                break;
            case FacadeStyle::Metal:
                m.window.preferredBay = 5.2;
                m.window.windowWidthFrac = 0.35;
                m.window.sill = 2.4;
                m.window.head = 3.6;
                break;
            default:
                m.window.preferredBay = 3.4;
                m.window.windowWidthFrac = 0.55;
                break;
        }
    }
    return lib;
}

std::vector<int> PaletteLibrary::candidates(Character c, int storeys) const {
    std::vector<int> out;
    for (std::size_t i = 0; i < sets.size(); ++i) {
        if (character[i] != c) continue;
        // ADR-0040: cladding follows structure follows height. A set declares
        // the band it is credible in, so a timber cottage palette can never
        // land on a forty-storey tower.
        if (storeys < sets[i].minStoreys || storeys > sets[i].maxStoreys) continue;
        out.push_back(static_cast<int>(i));
    }
    if (!out.empty()) return out;
    // Nothing in this character fits the height — fall back to any set that
    // does, so a building is never left unclad.
    for (std::size_t i = 0; i < sets.size(); ++i)
        if (storeys >= sets[i].minStoreys && storeys <= sets[i].maxStoreys)
            out.push_back(static_cast<int>(i));
    if (out.empty() && !sets.empty()) out.push_back(0);
    return out;
}

int PaletteLibrary::districtFamily(Character c, Era preferred, int storeys,
                                   std::uint32_t districtSeed) const {
    std::vector<int> cand = candidates(c, storeys);
    if (cand.empty()) return -1;
    // Era is a PREFERENCE, not a filter: if nothing in the district's era fits
    // the height, the district still gets dressed.
    std::vector<int> preferredEra;
    for (int i : cand)
        if (era[i] == preferred) preferredEra.push_back(i);
    const std::vector<int>& pool = preferredEra.empty() ? cand : preferredEra;
    Rng dr(districtSeed ? districtSeed : 3u);
    return pool[dr.irange(0, static_cast<int>(pool.size()) - 1)];
}

int PaletteLibrary::pick(Character c, Era preferred, int storeys,
                         std::uint32_t districtSeed,
                         std::uint32_t buildingSeed) const {
    std::vector<int> cand = candidates(c, storeys);
    if (cand.empty()) return -1;
    const int family = districtFamily(c, preferred, storeys, districtSeed);

    // A minority of buildings step outside the district's family, which is what
    // a real street looks like. An outlier must actually DIFFER from the family
    // — drawing from the whole candidate list would silently return the family
    // itself much of the time, so the "variation" would be invisible and the
    // rate meaningless.
    Rng br(buildingSeed ? buildingSeed : 5u);
    if (br.unit() < 0.22 && cand.size() > 1) {
        std::vector<int> others;
        for (int i : cand)
            if (i != family) others.push_back(i);
        if (!others.empty())
            return others[br.irange(0, static_cast<int>(others.size()) - 1)];
    }
    return family;
}

MaterialSet perturb(const MaterialSet& base, std::uint32_t seed, Real amount) {
    Rng rng(seed ? seed : 11u);
    MaterialSet m = base;
    // Colours shift; PARTS never do. Changing which parts a building uses is
    // what would break the bundle's coherence.
    m.wallColor = jitter(base.wallColor, rng, amount);
    m.roofColor = jitter(base.roofColor, rng, amount * 0.6);
    m.trimColor = jitter(base.trimColor, rng, amount * 0.4);
    m.accentColor = jitter(base.accentColor, rng, amount * 0.5);
    return m;
}

int basePaletteFor(const PaletteLibrary& lib, int shaftSet, Character c,
                   std::uint32_t seed) {
    if (shaftSet < 0 || shaftSet >= static_cast<int>(lib.sets.size()))
        return shaftSet;
    // ADR-0040's bound: one family per mass, with a legitimate SECOND treatment
    // at the base. A glass tower on a stone podium is the canonical case, so
    // look for a masonry set in the same character to stand underneath.
    std::vector<int> masonry;
    for (std::size_t i = 0; i < lib.sets.size(); ++i) {
        if (lib.character[i] != c && lib.character[i] != Character::Civic) continue;
        const FacadeStyle s = lib.sets[i].style;
        if (s == FacadeStyle::GlassCurtain || s == FacadeStyle::Metal) continue;
        masonry.push_back(static_cast<int>(i));
    }
    if (masonry.empty()) return shaftSet;
    Rng rng(seed ? seed : 17u);
    return masonry[rng.irange(0, static_cast<int>(masonry.size()) - 1)];
}

const MaterialSet& BuildingMaterials::forTier(int tier) const {
    if (perTier.empty()) return siteSet;
    if (tier < 0) return perTier.front();
    if (tier >= static_cast<int>(perTier.size())) return perTier.back();
    return perTier[tier];
}

SidedPart BuildingMaterials::partFor(int tier, PartId logical, Side side) const {
    const MaterialSet& m = forTier(tier);
    PartId p = logical;
    switch (logical) {
        case PartId::Wall:   p = m.wall; break;
        case PartId::Trim:   p = m.trim; break;
        case PartId::Roof:   p = m.roof; break;
        case PartId::Ground: p = m.base; break;
        default: break;      // Glass, Door and the rest are not re-mapped
    }
    return SidedPart{static_cast<std::uint8_t>(p), side};
}

BuildingMaterials assignMaterials(const PaletteLibrary& lib, Character c, Era era,
                                  const std::vector<int>& tierStoreys,
                                  std::uint32_t districtSeed,
                                  std::uint32_t buildingSeed) {
    BuildingMaterials out;
    if (tierStoreys.empty()) {
        out.siteSet = lib.sets.empty() ? MaterialSet{} : lib.sets[0];
        return out;
    }
    int total = 0;
    for (int n : tierStoreys) total += n;

    // The SHAFT's palette is chosen for the building's whole height, so a tall
    // building gets a tall building's cladding.
    const int shaft = lib.pick(c, era, std::max(1, total), districtSeed, buildingSeed);
    const MaterialSet shaftSet =
        shaft >= 0 ? perturb(lib.sets[shaft], buildingSeed) : MaterialSet{};

    for (std::size_t i = 0; i < tierStoreys.size(); ++i) {
        if (i == 0 && tierStoreys.size() > 1) {
            // The base tier may re-clad — the one legitimate second treatment.
            const int b = basePaletteFor(lib, shaft, c, buildingSeed * 31u + 7u);
            out.perTier.push_back(
                b >= 0 ? perturb(lib.sets[b], buildingSeed * 13u + 3u) : shaftSet);
        } else {
            out.perTier.push_back(shaftSet);
        }
    }
    out.siteSet = shaftSet;
    return out;
}

}  // namespace engine
