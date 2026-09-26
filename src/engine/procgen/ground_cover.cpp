#include "ground_cover.h"

#include "terrain_maps.h"

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

Cover GroundCover::at(double x, double z, double height, double normalUp, double normalX, double normalZ) const {
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
    // THE SNOWLINE WANDERS (ADR-0118, Glenn: "the mountain snow line is very regular which makes it look
    // odd"). A +-15 m ripple on a 470 m line read as a contour. Real lines move by hundreds of metres:
    //   lobes    -- broad fields (~800 m and ~170 m) that push the line up a shoulder and down a basin;
    //   aspect   -- shaded, north-facing (+z is north here) faces hold it lower, sunny faces melt higher;
    //   tongues  -- snow runs down the fall line in gullies: noise stretched ALONG the downslope direction.
    // All in proportion to the snow height, so a low island and a high range wander alike.
    const bool mapped = p_.maps != nullptr;
    const TerrainMapSample ms = mapped ? p_.maps->at(x, z) : TerrainMapSample{};
    double snowLine = p_.snowHeight;
    if (p_.snowHeight < 1e29) {
        const double S = std::max(50.0, p_.snowHeight);
        const double lobe = noise_.fbm2(x * 0.0012 - 21.7, z * 0.0012 + 13.3, 3), mid = noise_.fbm2(x * 0.006 + 4.4, z * 0.006 - 9.2, 3);
        snowLine += S * (0.14 * lobe + 0.06 * mid);
        const double hz = std::hypot(normalX, normalZ);   // how much the face tilts, and which way
        if (hz > 1e-4) {
            const double north = normalZ / hz;            // +1 facing north (shaded), -1 facing south
            snowLine -= S * 0.13 * north * std::min(1.0, hz * 2.5);
            // tongues: along the fall line (dx, dz), across it (-dz, dx); long along, narrow across
            const double dx = normalX / hz, dz = normalZ / hz;
            const double along = x * dx + z * dz, across = -x * dz + z * dx;
            const double streak = noise_.noise2(along * 0.004 + 2.1, across * 0.045 - 6.3);   // ~250 m long, ~22 m wide
            snowLine -= S * 0.09 * std::max(0.0, streak) * std::min(1.0, hz * 3.0);
        }
        // the ground's own shape (ADR-0128): gullies and hollows hold snow far below the line, ridges and
        // spurs are blown bare above it
        if (mapped) snowLine += S * (0.12 * ms.convex - 0.08 * ms.wet);
    }
    const double snowRaw = p_.snowHeight < 1e29
                               ? smooth(-0.1, 0.1, (h + 15.0 * jag - snowLine) / 20.0) *
                                     (1.0 - smooth(-0.15, 0.15, (slope + 6.0 * jag - 50.0) / 12.0))
                               : 0.0;
    const double high = smooth(-0.2, 0.2, (h + 12.0 * jag - p_.mountainHeight * 1.25) / 30.0);
    // mapped: bare rock where the ground cannot hold soil (steep, convex); up high, thin soil is enough to bare it
    const double rockRaw = mapped
        ? std::max(smooth(-0.15, 0.15, (slope + 7.0 * jag - p_.rockSlopeDeg - 6.0 * (ms.soil - 0.5)) / 10.0),
                   high * smooth(0.25, 0.65, 1.0 - ms.soil + 0.15 * jag))
        : std::max(smooth(-0.15, 0.15, (slope + 7.0 * jag - p_.rockSlopeDeg) / 10.0), 0.55 * high);
    const double sandRaw = sea ? 1.0 - smooth(-0.12, 0.12, (h + 1.2 * jag - p_.beachHeight) / p_.beachHeight) : 0.0;
    const double patch = noise_.fbm2(x * 0.045 + 7.0, z * 0.045 - 3.0, 3) * 0.5 + 0.5 + 0.18 * fine;
    const double dryUp = smooth(p_.uplandHeight, p_.mountainHeight, h);
    const double dirtRaw = std::clamp(smooth(0.66 - 0.4 * p_.dirtPatches, 0.74 - 0.4 * p_.dirtPatches, patch) *
                                          std::min(1.0, 2.0 * p_.dirtPatches) + 0.35 * dryUp * smooth(0.45, 0.6, patch),
                                      0.0, 1.0);
    // mapped: scree below the cliffs and washed gravel down the steeper channels read as bare earth
    const double dirtAll = mapped ? std::max({dirtRaw, smooth(0.2, 0.5, ms.scree + 0.15 * jag),
                                              0.8 * ms.wet * dryUp * smooth(8.0, 20.0, slope)})
                                  : dirtRaw;
    Cover c;
    double rem = 1.0;
    c.snow = snowRaw * rem; rem -= c.snow;
    c.rock = rockRaw * rem; rem -= c.rock;
    c.sand = sandRaw * rem; rem -= c.sand;
    c.dirt = dirtAll * rem; rem -= c.dirt;
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
