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
    double cragM = 0.0;   // "crags": m of ridged rock detail on steep mountain ground (see the fine grid)
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
    //             "slopeDamp" (detail / (1 + k |grad|^2)), "incise" (stream-power K a level), "iterations" }
    bool refine = false;
    double refineDetailM = 8.0;
    double refineRoughness = 0.8;
    double refineWavelengthCells = 8.0;
    double refineSlopeDamp = 1.5;
    double refineIncise = 2.0e-3;
    int refineInciseIterations = 10;
};

WeatherParams weatherFromJson(const nlohmann::json& weather, const nlohmann::json& terrainBlock);

// Grow and weather `tp`'s land. `grownOut` (optional) gets the fine grid before the water.
Heightmap weatherTerrain(const TerrainParams& tp, uint32_t seed, const WeatherParams& w, Heightmap* grownOut = nullptr);

// The pipeline's code tag (the maps' cache key folds it too: a regrown ground must not keep old maps).
const char* weatherCodeTag();

// The same, through the disk cache (cache/terrain/weather_<hash>.bin). Returns the fine grid.
Heightmap weatheredTerrainCached(const nlohmann::json& terrainBlock);

}  // namespace engine

#endif
