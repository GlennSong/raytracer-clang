#ifndef RAYTRACER_ENGINE_PROCGEN_STREAM_POWER_H
#define RAYTRACER_ENGINE_PROCGEN_STREAM_POWER_H

// MOUNTAINS FROM UPLIFT AND RIVERS (ADR-0125). Instead of drawing mountains with noise (ridged noise read
// as pyramids, and its combed striping survived every later erosion pass), GROW them: the crust is pushed
// up by an uplift map while rivers cut down in proportion to their drainage area and slope -- the stream
// power law, dh/dt = U - K A^m S^n, with n = 1 -- until the two balance. Ridges, spurs and branching
// valleys come out of the physics. Solver: Braun & Willett 2013 ("FastScape": receivers, drainage area,
// an implicit sweep up the drainage tree, stable at any time step), as Cordonnier et al. 2016 use it for
// terrain; depressions are filled each iteration (with a tiny gradient) so every cell drains to the sea
// or the grid's edge. Plus linear hillslope diffusion (soil creep: rounded tops, smooth footslopes).
//
// Units are relative: U is a 0..1 map scaled by `uplift`, and the result is rescaled so the land spans
// the height range it started with (`rescale`). CPU: the sweep is sequential down the tree.

#include "erosion.h"   // Heightmap
#include "rock_layers.h"

#include <cstdint>
#include <vector>

namespace engine {

// Bump when the solver's output changes for the same inputs: every weathered ground folds it (ADR-0126).
constexpr const char* kStreamPowerCodeTag = "2026-09-26.1";

struct StreamPowerParams {
    int    iterations = 250;
    double dt = 5.0e4;         // years a step (the implicit sweep is stable at any dt)
    double K = 2.0e-5;         // erodibility
    double m = 0.5;            // area exponent (n = 1)
    double uplift = 1.0e-3;    // m/yr where the uplift map is 1
    double diffusion = 0.01;   // hillslope diffusivity, m^2/yr (0 = off)
    double seaLevel = -1e30;   // cells below it (initially) are sea: fixed outlets
    bool   rescale = true;     // map the result onto [targetLo, targetHi] (unset: the land's starting range)
    double targetLo = 0.0, targetHi = -1.0;
    // STOCHASTIC ROUTING (ADR-0206; Glenn's 8192 island ground: "gullies in parallel diagonal lines"): a cell drains
    // to one of its downhill neighbours drawn in proportion to the slope toward it, not always the steepest. On an
    // even slope between two of the grid's eight directions the steepest is the same one for every cell, and the
    // drainage cut parallel channels along it; drawn, the flow lines wander about the true downhill direction and
    // follow it on average. Seeded: the same ground every bake. Off by default (every existing ground as it was).
    bool   stochastic = false;
    uint32_t seed = 0;
    // ROCK LAYERS (ADR-0209): when set, each cell's erodibility and creep are its rock's at its height NOW (re-read
    // every iteration, so cutting down through a hard bed into a soft one speeds up): times hardK / hardDiffusion
    // where the rock is hard. Cell (x, z) is at (layerOriginX + x cell, layerOriginZ + z cell). Unset: as it was.
    const RockLayers* layers = nullptr;
    double layerOriginX = 0.0, layerOriginZ = 0.0;
};

// Grow `hm` under `uplift` (n x n, 0..1; the grid's edge and the sea are the outlets, held fixed).
// Returns the number of iterations run.
// `erodibility` (optional, n x n, around 1): K's multiplier per cell -- softer and harder rock, so the
// valleys do not all come out alike.
// `areaOut` (optional) gets the last iteration's drainage area per cell (m^2), `receiversOut` its receivers (the cell
// each drains to; -1 at an outlet).
int streamPowerErode(Heightmap& hm, const std::vector<float>& uplift, const StreamPowerParams& p,
                     const std::vector<float>* erodibility = nullptr, std::vector<float>* areaOut = nullptr,
                     std::vector<int32_t>* receiversOut = nullptr);

}  // namespace engine

#endif
