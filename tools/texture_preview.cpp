// Bakes every procedural surface and writes a contact sheet of its PBR maps
// (albedo / normal / roughness / AO), each tiled 2x2 to show the seamless tiling
// the engine uploads to Metal. Build: see the one-liner in the chat.
#include "engine/procgen/surface_maps.h"
#include <vector>
#include <cstdint>
#include <cstdio>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tinygltf/stb_image_write.h"

using namespace engine;
using S = RenderMaterial::Surface;

int main() {
    std::vector<S> surfs = {S::Brick, S::Concrete, S::Stucco, S::RoofTile,
                            S::RoofShingle, S::CorrugatedMetal, S::Asphalt,
                            S::Pavement, S::Cobblestone, S::WoodSiding};
    const int cell = 128, tiles = 2;
    const int rows = static_cast<int>(surfs.size()), cols = 4;
    const int W = cols * cell, H = rows * cell;
    std::vector<uint8_t> sheet(static_cast<size_t>(W) * H * 3, 24);

    // mode: 0 = RGB, 1..n = single channel (gray)
    auto draw = [&](const TextureData& t, int cx, int cy, int mode) {
        if (t.pixels.empty()) return;
        for (int y = 0; y < cell; ++y)
            for (int x = 0; x < cell; ++x) {
                int sx = (x * tiles * t.width / cell) % t.width;
                int sy = (y * tiles * t.height / cell) % t.height;
                size_t si = (static_cast<size_t>(sy) * t.width + sx) * t.channels;
                uint8_t r, g, b;
                if (mode == 0) {
                    r = t.pixels[si];
                    g = t.pixels[si + (t.channels > 1 ? 1 : 0)];
                    b = t.pixels[si + (t.channels > 2 ? 2 : 0)];
                } else {
                    r = g = b = t.pixels[si + (mode - 1)];
                }
                int px = cx * cell + x, py = cy * cell + y;
                size_t di = (static_cast<size_t>(py) * W + px) * 3;
                sheet[di] = r; sheet[di + 1] = g; sheet[di + 2] = b;
            }
    };

    const char* names[] = {"brick", "concrete", "stucco", "rooftile", "shingle",
                           "corrugated", "asphalt", "pavement", "cobble", "wood"};
    for (int i = 0; i < rows; ++i) {
        SurfaceMaps m = surfaceMaps(surfs[i], 128, 1337u);
        draw(m.albedo, 0, i, 0);   // albedo (RGB)
        draw(m.normal, 1, i, 0);   // normal map (RGB)
        draw(m.mr, 2, i, 2);       // roughness (MR green channel)
        draw(m.ao, 3, i, 1);       // ambient occlusion
        std::printf("row %2d: %s\n", i, names[i]);
    }
    stbi_write_png("/tmp/proc_textures.png", W, H, 3, sheet.data(), W * 3);
    std::printf("wrote /tmp/proc_textures.png (%dx%d) cols: albedo | normal | roughness | AO\n", W, H);
    return 0;
}
