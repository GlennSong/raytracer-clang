#include "ground_cover.h"

#include <algorithm>
#include <cmath>

namespace engine {

bool biomeFromName(const std::string& n, Biome& out) {
    static const std::pair<const char*, Biome> kNames[] = {
        {"sea", Biome::Sea}, {"beach", Biome::Beach}, {"lowland", Biome::Lowland},
        {"upland", Biome::Upland}, {"mountain", Biome::Mountain}};
    for (const auto& [name, b] : kNames)
        if (n == name) { out = b; return true; }
    return false;
}

namespace {
double smooth(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
Vec3 mix3(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }
}  // namespace

GroundCover::GroundCover(const GroundCoverParams& p) : p_(p), noise_(p.seed * 2654435761u + 17u) {}

Cover GroundCover::at(double x, double z, double height, double normalUp) const {
    const bool sea = p_.seaLevel > -1e29;
    const double h = sea ? height - p_.seaLevel : height;             // height above the sea
    const double slope = std::acos(std::clamp(normalUp, -1.0, 1.0)) * (180.0 / 3.14159265358979);
    // the noise that makes every edge ragged: a macro field and a fine one
    const double macro = noise_.fbm2(x * 0.018 + 11.3, z * 0.018 - 4.1, 3);   // ~-1..1
    const double fine = noise_.noise2(x * 0.22 - 3.7, z * 0.22 + 8.9);
    const double jag = 0.65 * macro + 0.35 * fine;
    // raw coverage of each layer, sharpened after its noise
    // snow above the snowline -- but it slides off faces much past 50 degrees: ledges and gullies
    // hold it, cliffs stay bare rock (every alpine reference reads that way)
    const double snowRaw = p_.snowHeight < 1e29
                               ? smooth(-0.1, 0.1, (h + 15.0 * jag - p_.snowHeight) / 20.0) *
                                     (1.0 - smooth(-0.15, 0.15, (slope + 6.0 * jag - 50.0) / 12.0))
                               : 0.0;
    const double rockRaw = std::max(smooth(-0.15, 0.15, (slope + 7.0 * jag - p_.rockSlopeDeg) / 10.0),
                                    0.55 * smooth(-0.2, 0.2, (h + 12.0 * jag - p_.mountainHeight * 1.25) / 30.0));
    const double sandRaw = sea ? 1.0 - smooth(-0.12, 0.12, (h + 1.2 * jag - p_.beachHeight) / p_.beachHeight) : 0.0;
    const double patch = noise_.fbm2(x * 0.045 + 7.0, z * 0.045 - 3.0, 3) * 0.5 + 0.5 + 0.18 * fine;
    const double dryUp = smooth(p_.uplandHeight, p_.mountainHeight, h);
    const double dirtRaw = std::clamp(smooth(0.66 - 0.4 * p_.dirtPatches, 0.74 - 0.4 * p_.dirtPatches, patch) *
                                          std::min(1.0, 2.0 * p_.dirtPatches) + 0.35 * dryUp * smooth(0.45, 0.6, patch),
                                      0.0, 1.0);
    Cover c;
    double rem = 1.0;
    c.snow = snowRaw * rem; rem -= c.snow;
    c.rock = rockRaw * rem; rem -= c.rock;
    c.sand = sandRaw * rem; rem -= c.sand;
    c.dirt = dirtRaw * rem; rem -= c.dirt;
    c.grass = std::max(0.0, rem);
    // biome: the band the point stands in (the sea floor counts as sea)
    if (sea && h < 0.0) c.biome = Biome::Sea;
    else if (sea && h < p_.beachHeight * 1.8 + 1.5 * jag) c.biome = Biome::Beach;
    else if (h > p_.mountainHeight * 0.85 + 8.0 * jag) c.biome = Biome::Mountain;
    else if (h > p_.uplandHeight + 6.0 * jag) c.biome = Biome::Upland;
    else c.biome = Biome::Lowland;
    // colour: grass greener in the lowland, drier uphill; each layer mottled a little
    const Vec3 grassCol = mix3(p_.grass, p_.grassDry, std::clamp(0.25 * dryUp + 0.12 * (macro + 1.0) * dryUp, 0.0, 1.0)) *
                          (1.0 + 0.10 * macro);
    const Vec3 col = grassCol * c.grass + p_.dirt * (c.dirt * (1.0 + 0.12 * fine)) + p_.sand * (c.sand * (1.0 + 0.06 * fine)) +
                     p_.rock * (c.rock * (1.0 + 0.15 * macro)) + p_.snow * c.snow;
    c.colour = col;
    return c;
}

}  // namespace engine
