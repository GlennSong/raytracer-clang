#ifndef RAYTRACER_ENGINE_PROCGEN_ROCK_LAYERS_H
#define RAYTRACER_ENGINE_PROCGEN_ROCK_LAYERS_H

// ROCK LAYERS (ADR-0209; Glenn, 2026-10-19: "slabs or sheets of stone making ridges" -- the Flatirons, Yamnuska,
// Banded Peak). A mountain's rock is a stack of beds, tilted by the mountain building, some hard (limestone,
// sandstone) and some soft (shale). Erosion takes the soft ones faster: the hard ones stand as cliff bands, ledges
// and, steeply tilted, as slab ridges; the soft ones wear into the slopes and valleys between. The look is in the
// scale -- beds tens to hundreds of metres thick, the structure a whole face is made of -- so the beds belong in the
// stage that grows the mountains (stream power, ADR-0125), where they make the erosion itself uneven, not stamped
// onto the finished ground (three tries at that, ADR-0209, read as nothing or as clay).
//
// The field: at any point and height, which bed it is in and how hard. Beds are planes dipping at `dipDeg` toward a
// direction that wanders over `dipScaleM` (folded rock), warped by noise, `thicknessM` thick on average (varying
// 0.6x..1.5x over `thicknessScaleM`); a share `hard` of them is hard, by their index, and a hard bed softens in
// patches along its outcrop (`lateralScaleM`) so no ledge runs forever.

#include <cstdint>
#include <nlohmann/json.hpp>

#include "noise.h"

namespace engine {

struct RockLayersParams {
    double thicknessM = 120.0;
    double dipDeg = 25.0;
    double dipScaleM = 6000.0;
    double thicknessScaleM = 2500.0;
    double warpM = 0.0;          // the beds' warp amplitude (m); 0 = half the thickness
    double hard = 0.4;           // share of beds that are hard
    double lateralScaleM = 900.0;
    double hardK = 0.15;         // a hard bed's erodibility, times the soft beds'
    double hardDiffusion = 0.2;  // ...and its hillslope creep
    double toneThicknessM = 25.0;   // the BANDS a face shows (its colour, the macro map's): beds this thick
    // ...only in the HIGH ground (an island's trial: layered rock all over moved the rivers the towns stand on --
    // 35% of the old channels kept -- and up to 95 m of ground in a hilly town): none below fromHeight, all from
    // fullHeight (m above the sea); both 0: everywhere
    double fromHeightM = 0.0, fullHeightM = 0.0;
};

// from a "layers" block (weather.grow.layers)
RockLayersParams rockLayersFromJson(const nlohmann::json& j);

class RockLayers {
public:
    RockLayers(const RockLayersParams& p, uint32_t seed);
    // how hard the rock is at (x, z) at height h: 0 soft .. 1 hard
    double hardness(double x, double z, double h) const;
    // the rock's TONE there, 0 dark .. 1 pale: each band of toneThicknessM its own (Banded Peak's light and dark
    // beds), graded at the contacts
    double tone(double x, double z, double h) const;
    const RockLayersParams& params() const { return p_; }

private:
    double bedCoordinate(double x, double z, double h, double thickness) const;   // u: the bed index + how far through
    RockLayersParams p_;
    Noise n_;
    double tanDip_;
    uint32_t seed_;
};

}  // namespace engine

#endif
