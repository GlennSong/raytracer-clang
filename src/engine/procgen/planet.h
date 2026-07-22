#ifndef RAYTRACER_ENGINE_PROCGEN_PLANET_H
#define RAYTRACER_ENGINE_PROCGEN_PLANET_H

#include "../../rt_math.h"            // Vec3
#include "../../renderer/renderer.h"  // RenderMesh
#include "color_ramp.h"
#include "tree.h"                     // TextureData

#include <cstdint>

namespace engine {

// A procedural planet seen from space (procedural-planet-plan, ADR-0076). P1: the
// rocky surface — a cube-sphere displaced by a composed radial height field
// (fbm continents + ridged mountains + impact craters + domain warp), with biome
// colour baked to vertex colours so it rides the existing PBR path with no custom
// shader. Deterministic in `seed` (ADR-0021) and fully headless — the shading
// upgrades (atmosphere, ocean, clouds) are later, device-side phases.
struct PlanetParams {
    float radius = 1000.0f;
    int   faceRes = 96;              // cube-sphere grid per face (6·faceRes²·2 tris)

    // --- height field (normalised units; displacement scales these by radius) ---
    double continentFreq = 1.3;      // broad landmass frequency (fbm)
    double continentAmp = 1.0;
    int    continentOctaves = 5;
    double mountainFreq = 3.0;       // ridged mountain-belt frequency
    double mountainAmp = 0.7;
    int    mountainOctaves = 5;
    double warpStrength = 0.25;      // domain warp on the sample direction

    // --- craters (Worley impact field) ---
    double craterFreq = 7.0;         // cells per unit direction (bigger = smaller/more)
    double craterAmp = 0.5;
    double craterDensity = 0.55;     // fraction of cells that actually crater
    double craterMinRadius = 0.22;   // crater radius range, in Worley cell units
    double craterMaxRadius = 0.62;

    // --- displacement + sea ---
    double reliefFraction = 0.04;    // peak displacement = radius·reliefFraction
                                     // (already vertically exaggerated — see plan)
    double seaLevel = 0.0;           // normalised height; land where h > seaLevel

    // --- colour ---
    ColorRamp landRamp;              // by normalised elevation 0..1 (empty = grey)
    bool polarCaps = true;
    double capLatitude = 0.8;        // |dir.y| threshold where caps take over
    Vec3 capColor{0.90, 0.92, 0.95};

    // --- ocean shell (a separate translucent body at the sea-level radius) ---
    bool hasOcean = false;
    Vec3 oceanColor{0.05, 0.11, 0.22};
    double oceanOpacity = 0.72;
};

// Tuned presets. Mars: oxide dust, thin caps, cratered, low relief, no ocean.
// Moon: greyscale, heavily cratered, no caps, no ocean. Earth-like: biomes + big
// caps + an ocean.
PlanetParams planetMars();
PlanetParams planetMoon();
PlanetParams planetEarthlike();

// The composed radial height field at a unit direction, roughly in [-1, 1]. Exposed
// so callers (and tests) can query relief without meshing.
double planetHeight(const PlanetParams& p, uint32_t seed, const Vec3& dir);

// One crater's radial profile: `t` = distance/craterRadius. Negative in the bowl
// (down to ~-1 at the centre), positive at the raised rim near t=1, fading to 0 by
// t≈1.6 through the ejecta blanket. Exposed for testing the crater morphology.
double craterProfile(double t);

// The surface albedo at a unit direction: the biome ramp by elevation above sea
// level, blended toward the cap colour past the cap latitude. The single source of
// truth for surface colour — shared by the mesh baker and any equirect/preview
// bake so they always agree.
Vec3 planetSurfaceColor(const PlanetParams& p, uint32_t seed, const Vec3& dir);

// Generate the rocky surface mesh: a displaced cube-sphere with per-vertex biome
// colour and recomputed normals. Watertight (radial displacement moves welded seam
// vertices identically). Pair with a white-ish PBR material so the vertex colour
// shows.
RenderMesh generatePlanet(const PlanetParams& p, uint32_t seed);

// Generate the ocean shell (a plain cube-sphere at the sea-level radius) when
// `hasOcean` is set; an empty mesh otherwise. Give it a low-roughness, <1-opacity
// material tinted `oceanColor`.
RenderMesh generateOceanShell(const PlanetParams& p, uint32_t seed);

// --------------------------------------------------------------------------
// Gas giant (procedural-planet-plan P5, headless slice). A gas giant has no solid
// surface — it is a SMOOTH sphere whose look is entirely the top of the cloud deck:
// latitudinal belts/zones warped by turbulent flow, plus vortex storms. Here we
// bake that as a seamless equirectangular albedo texture (rides the existing
// albedoMap slot on a plain cube-sphere, whose UVs are already equirectangular).
// The evolving-fluid animation + limb darkening + atmosphere halo are the later
// device-side upgrades; the static banded texture is fully headless + testable.
// --------------------------------------------------------------------------
struct GasGiantParams {
    float radius = 1000.0f;
    int   faceRes = 64;              // smooth sphere (NO displacement)
    int   texWidth = 512;            // equirectangular albedo (2:1)
    int   texHeight = 256;

    double bandFreq = 11.0;          // belt/zone pairs from pole to pole
    double flowWarp = 0.35;          // domain-warp strength → turbulent belts
    int    warpOctaves = 4;
    double detailFreq = 5.0;         // fine cloud turbulence frequency
    double detailAmp = 0.15;

    int    stormCount = 3;           // vortex spots (e.g. the Great Red Spot)
    double stormSize = 0.13;         // angular radius (fraction of pi)
    Vec3   stormColor{0.72, 0.34, 0.24};

    ColorRamp bandRamp;              // by warped latitude phase 0..1 (empty = greys)
};

// Preset gas giants: warm banded Jupiter, deep-blue Neptune.
GasGiantParams gasGiantJupiter();
GasGiantParams gasGiantNeptune();

// The smooth sphere a gas giant is painted on (a plain cube-sphere, no relief).
RenderMesh generateGasGiantMesh(const GasGiantParams& p);

// Bake the seamless equirectangular albedo (RGB8, texWidth×texHeight). Sampled on
// the 3D sphere direction so the longitude wrap is seamless; deterministic in seed.
TextureData generateGasGiantTexture(const GasGiantParams& p, uint32_t seed);

}  // namespace engine

#endif
