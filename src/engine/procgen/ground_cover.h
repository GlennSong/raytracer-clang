#ifndef RAYTRACER_ENGINE_PROCGEN_GROUND_COVER_H
#define RAYTRACER_ENGINE_PROCGEN_GROUND_COVER_H

// THE GROUND-COVER MAP (the flora plan): what the ground is at any point -- grass, dirt, sand,
// rock, snow -- and which BIOME it belongs to (beach, lowland, upland, mountain). One answer
// for everything that asks: the terrain's colour (terrainColor, when TerrainParams::cover is
// set), the grass field's density (GrassSystem) and which tree species may grow there
// (loadVegetation's "biome"). It is a pure function of the point, its height and its slope,
// with noise that makes every boundary ragged and patchy rather than a contour line.
//
// Transitions are SHARPENED: each layer's raw coverage is pushed through a narrow smoothstep
// after its noise is added, so grass meets dirt at a ragged edge a metre or two wide, not a
// hundred-metre fade. Grass's colour sits just under the grass field's own (procgen/grass.h),
// so where blades part the ground reads as shaded grass, not a pale gap.

#include "../../rt_math.h"
#include "noise.h"

#include <cstdint>
#include <memory>
#include <string>

namespace engine {

struct TerrainMaps;
struct ForestParams;

enum class Biome : uint8_t { Sea = 0, Beach, Lowland, Upland, Mountain, Count };
bool biomeFromName(const std::string& name, Biome& out);

struct GroundCoverParams {
    double seaLevel = -1e30;       // world Y; -1e30 = no sea (no beach band)
    double beachHeight = 2.5;      // sand to about this far above the sea
    double uplandHeight = 40.0;    // above sea level: the dry upland (meadow, more bare earth)
    double mountainHeight = 85.0;  // above sea level: the mountain (pines, stone)
    double snowHeight = 1e30;      // above sea level: snow (off by default)
    double rockSlopeDeg = 36.0;    // bare rock on ground steeper than this
    double dirtPatches = 0.22;     // bare-earth patches in the lowland, 0..1
    uint32_t seed = 1;
    // linear albedo
    Vec3 grass{0.030, 0.068, 0.013};     // just under the grass field's blades
    Vec3 grassDry{0.075, 0.085, 0.030};  // upland meadow
    Vec3 dirt{0.105, 0.072, 0.040};
    Vec3 sand{0.40, 0.34, 0.22};
    Vec3 rock{0.085, 0.080, 0.072};      // dark enough to sit beside stylized grass in full sun
    Vec3 snow{0.85, 0.88, 0.92};
    // THE GROUND'S MAPS (ADR-0128, procgen/terrain_maps.h), when the terrain is a weathered one: rock where
    // the ground sheds soil (steep, convex, high), scree below cliffs, meadow in the hollows, snow held in
    // gullies and blown off ridges, washed gravel down the channels
    std::shared_ptr<const TerrainMaps> maps;
    // THE FOREST FLOOR (ADR-0129): under the terrain's "forest" canopy the ground is leaf and needle
    // litter (the dirt layer) with a little grass, not meadow -- and the grass field thins with it
    std::shared_ptr<const ForestParams> forest;
};

struct Cover {
    double grass = 1, dirt = 0, sand = 0, rock = 0, snow = 0;   // sum to 1
    Biome biome = Biome::Lowland;
    Vec3 colour;
};

class GroundCover {
public:
    explicit GroundCover(const GroundCoverParams& p);
    // `height` is the ground's world Y at (x, z), `normalUp` its normal's y (1 = flat).
    // normalX/Z: the horizontal part of the surface normal, when the caller has it -- snow lies lower on
    // shaded (north-facing) faces and runs down the fall line in tongues (ADR-0118); 0,0 = slope only
    Cover at(double x, double z, double height, double normalUp, double normalX = 0.0, double normalZ = 0.0) const;
    const GroundCoverParams& params() const { return p_; }

private:
    GroundCoverParams p_;
    Noise noise_;
};

}  // namespace engine

#endif
