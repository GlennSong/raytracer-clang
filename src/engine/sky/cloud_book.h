#ifndef RAYTRACER_ENGINE_SKY_CLOUD_BOOK_H
#define RAYTRACER_ENGINE_SKY_CLOUD_BOOK_H

// THE CLOUDS AS DATA (ADR-0386): assets/scripts/clouds.lua read into a CloudBook -- the four cloud types along the
// weather's type axis (where each stands, how big its cells are, its shape), and the weather map's size. The map
// (cloud_map.h) places cells of these types; the renderer's march shapes them (VolumetricCloudParams::types).

#include <array>
#include <string>

namespace engine {

struct CloudTypeSpec {
    std::string name;
    float base = 1200.0f, top = 2700.0f;                  // metres above the sea
    std::array<float, 4> profile{0.0f, 0.1f, 0.5f, 1.0f};   // density by height, fractions of base..top
    float density = 1.0f, erosion = 1.0f, anvil = 0.0f;
    float radiusMin = 300.0f, radiusMax = 1200.0f;         // a cell's footprint, half across
    float aspect = 1.2f;                                    // long against wide
    float low = 0.4f;                                       // the smallest cell's top, as a fraction of the height
};

struct CloudBook {
    std::array<CloudTypeSpec, 4> types{};   // along the type axis: 0, 1/3, 2/3, 1
    float shapeScale = 3.0f;                // the shape noise against the level's noiseScale
    float shadow = 0.85f;                   // how much of the sun a cloud's heart takes on the ground (ADR-0387)
    float mapExtent = 80000.0f;             // metres across
    int mapSize = 1024;                     // texels across
    float rebuildMetres = 8000.0f;          // the camera's drift in the air that rebakes the map
    bool loaded = false;                    // false: clouds.lua was not read (the built-in cumulus everywhere)
};

void setCloudBook(const CloudBook& book);
// The book in use: clouds.lua, read once (with scripting); without it, four cumulus slots
const CloudBook& cloudBook();

}  // namespace engine

#endif
