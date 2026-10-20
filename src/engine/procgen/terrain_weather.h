#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_WEATHER_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_WEATHER_H

// THE WEATHERED GROUND (ADR-0126): a level's terrain grown and weathered offline, the pipeline rt_erode
// proved (ADR-0122..0125), for the engine. A terrain block's "weather" asks for it:
//
//   "weather": { "res": 4096,                                  // the fine grid (cells a side)
//                "grow":  { "res": 1024, "iterations": 250,   // stream power (ADR-0125)
//                           "blur": 600, "K": 2e-5, "diffusion": 0.01 },
//                "water": { "steps": 20000, "breach": 25,     // droplet-free Mei water + sediment on the GPU
//                           "deposit": 0.15, "rockHardness": 0.15, "creep": 1e-4 },
//                "roughness": 2.5,                             // m of multi-scale roughness before the water
//                "shapeKeep": 1.0, "shapeScale": 1500 }        // the original's form below 1.5 km, kept
//
// The terrain's own relief says WHERE the ranges are (blurred into an uplift map); the physics decides
// their shape; the result is mapped back onto the land's height range. The sea is held as it was.
// Baked once and cached (content hash of the terrain block + a code tag) -- minutes, not a load cost.

#include "erosion.h"
#include "stream_power.h"
#include "terrain.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace engine {

struct WeatherParams {
    int res = 4096;
    int growRes = 1024;
    int growIterations = 250;
    double growBlurM = 600.0;
    StreamPowerParams grow;
    ErosionParams water;
    double roughnessM = 2.5;
    // KEEP THE SHAPE (ADR-0126, Glenn: "I feel like we lose some of the shape?"): stream power grows a narrow
    // divide over wide low foothills -- real, but not the island the level drew. The result keeps the
    // ORIGINAL terrain below `shapeScaleM` (its broad masses and plateaus) and takes the grown terrain's
    // structure above it (ridges, spurs, valleys): final = low(original) + (grown - low(grown)), blended by
    // `shapeKeep` (0 = the grown shape, 1 = the original's).
    double shapeKeep = 1.0;
    double shapeScaleM = 1500.0;
    // ...and on the original's low ground the grown structure fades out: none below plainHeightM above the
    // sea, all of it reliefRampM higher. Coastal plains stay flat (real alluvium, and where cities stand).
    double plainHeightM = 15.0;
    double reliefRampM = 120.0;
    // MASSIFS: the kept form's height above the plains varies by 1 +- massifAmount over ~massifScaleM, so a
    // range rises into high massifs and falls to saddles instead of topping out at one height
    double massifAmount = 0.35;
    double massifScaleM = 3000.0;
    // PEAKS: separate summits placed along the range (a jittered cell of peakSpacingM each, where the range is
    // high), up to peakHeightM over the kept form, heights skewed low; peakUplift lifts the grow's uplift under
    // them too, so the valleys radiate from the summits. "peaks": { "height", "spacing", "uplift" }
    double peakHeightM = 0.0;
    double peakSpacingM = 2200.0;
    double peakUplift = 1.0;
    // REFINE (ADR-0127): the grown grid doubled level by level to `res` -- upscale, blur, detail, drain.
    // "refine": { "detail" (m at 10 m cells), "roughness" (amplitude ~ cell^roughness), "wavelength" (cells),
    //             "slopeDamp" (detail / (1 + k |grad|^2)), "incise" (stream-power K a level), "iterations",
    //             "stochasticFrom" (cells a side: the levels this fine and finer drain by stochastic routing, ADR-0206;
    //             0 = none, so the coarser levels -- the ground a city was planned on -- stay as they were) }
    bool refine = false;
    double refineDetailM = 8.0;
    double refineRoughness = 0.8;
    double refineWavelengthCells = 8.0;
    double refineSlopeDamp = 1.5;
    double refineIncise = 2.0e-3;
    int refineInciseIterations = 10;
    int refineStochasticFrom = 0;
    // ROCK LAYERS in the grow (ADR-0209, procgen/rock_layers.h): "grow": { "layers": { ... } } -- the beds make the
    // erosion uneven while the mountains grow (hard beds stand as cliff bands and slab ridges); absent: the grow as
    // it was
    bool growLayers = false;
    RockLayersParams layers;
    // FACETS (ADR-0210; Glenn's reference, a limestone peak of big flat planes meeting at angles): the finished grid's
    // steep mountain rock cut into irregular cells ~`facetCellM` across, each replaced by ONE PLANE -- its own slope
    // (fitted over the cell) tilted at random up to `facetTiltDeg` -- so a face breaks into flat planes with real
    // edges, steps and creases where neighbours disagree; the shape catches light and shadow the way fractured rock
    // does, which lines painted on the colour never did. "facets": { "cell", "tilt", "slopeLo", "slopeHi", "strength",
    // and their VARIETY round a mountain (Glenn: "different levels of this result around the mountain"): "cellFine" (a
    // second, finer set of cells the coarse one blends to), "vary" (m: the scale of the regions that differ), "strengthLo",
    // "tiltLo" (the least a region gets) }
    double facetCellM = 0.0;   // 0 = off
    double facetTiltDeg = 8.0, facetSlopeLoDeg = 30.0, facetSlopeHiDeg = 45.0, facetStrength = 1.0;
    double facetCellFineM = 0.0;   // 0: one set of cells only
    double facetVaryM = 1500.0, facetStrengthLo = 1.0, facetTiltLoDeg = -1.0;   // (tiltLo < 0: the tilt everywhere)
};

WeatherParams weatherFromJson(const nlohmann::json& weather, const nlohmann::json& terrainBlock);

// Grow and weather `tp`'s land. `grownOut` (optional) gets the fine grid before the water.
Heightmap weatherTerrain(const TerrainParams& tp, uint32_t seed, const WeatherParams& w, Heightmap* grownOut = nullptr);

// WHAT SHAPES THE WEATHERED GROUND -- one answer for every cache downstream of it (review of ADR-0126..0134):
// the terrain block minus what is computed ON the ground (rivers) or only paints or stands on it (groundCover,
// material, cdlod, forest, trails).
nlohmann::json groundShapingBlock(const nlohmann::json& terrainBlock);
// The weathered ground's CONTENT KEY: the shaping block, the weathering and stream-power code tags, the erosion
// backend the bake would run on, and a fingerprint of the relief the block produces (so a change to the relief
// code is caught without a tag bump). The weather cache, the ground maps and the baked-ground pyramid all fold
// this one key -- a downstream cache folds the upstream key, it does not re-list upstream tags.
std::string weatheredGroundKey(const nlohmann::json& terrainBlock);

// The same, through the disk cache (cache/terrain/weather_<hash>.bin). Returns the fine grid.
Heightmap weatheredTerrainCached(const nlohmann::json& terrainBlock);

}  // namespace engine

#endif
