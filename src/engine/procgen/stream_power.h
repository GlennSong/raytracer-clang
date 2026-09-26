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

#include <vector>

namespace engine {

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
};

// Grow `hm` under `uplift` (n x n, 0..1; the grid's edge and the sea are the outlets, held fixed).
// Returns the number of iterations run.
// `erodibility` (optional, n x n, around 1): K's multiplier per cell -- softer and harder rock, so the
// valleys do not all come out alike.
// `areaOut` (optional) gets the last iteration's drainage area per cell (m^2).
int streamPowerErode(Heightmap& hm, const std::vector<float>& uplift, const StreamPowerParams& p,
                     const std::vector<float>* erodibility = nullptr, std::vector<float>* areaOut = nullptr);

}  // namespace engine

#endif
