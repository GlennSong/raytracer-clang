// rt_trees -- the real-tree gallery (ADR-0129): every species' impostor pictures (side + top) and its
// foliage texture, written as raw RGBA for tools/erosion_preview-style viewing, plus triangle counts.
//
//   rt_trees OUT [variants]      -> OUT/<species>_<v>_side.rgba (256x512), _top.rgba (256x256),
//                                   OUT/<species>_foliage.rgba (512x512), OUT/meta.txt
#include "engine/procgen/real_tree.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: rt_trees OUT [variants]\n"); return 2; }
    const std::string out = argv[1];
    const int variants = argc > 2 ? std::atoi(argv[2]) : 3;
    std::filesystem::create_directories(out);
    std::ofstream meta(out + "/meta.txt");
    for (int s = 0; s < static_cast<int>(engine::RealSpecies::Count); ++s) {
        const auto sp = static_cast<engine::RealSpecies>(s);
        const std::string name = engine::realSpeciesName(sp);
        const engine::TextureData fol = engine::realFoliageTexture(sp, 512, 7u);
        std::ofstream(out + "/" + name + "_foliage.rgba", std::ios::binary).write(reinterpret_cast<const char*>(fol.pixels.data()), static_cast<std::streamsize>(fol.pixels.size()));
        for (int v = 0; v < variants; ++v) {
            const engine::RealTree t = engine::realTree(sp, 1000u + v * 17u, 0.0);
            std::vector<uint8_t> side(256 * 512 * 4), top(256 * 256 * 4);
            engine::renderImpostor(t, fol, false, 256, 512, 3.0, side.data(), 256 * 4);
            engine::renderImpostor(t, fol, true, 256, 256, 3.0, top.data(), 256 * 4);
            std::ofstream(out + "/" + name + "_" + std::to_string(v) + "_side.rgba", std::ios::binary).write(reinterpret_cast<const char*>(side.data()), static_cast<std::streamsize>(side.size()));
            std::ofstream(out + "/" + name + "_" + std::to_string(v) + "_top.rgba", std::ios::binary).write(reinterpret_cast<const char*>(top.data()), static_cast<std::streamsize>(top.size()));
            meta << name << " " << v << " h=" << t.height << " crownR=" << t.crownRadius << " barkTris=" << t.bark.indices.size() / 3
                 << " foliageTris=" << t.foliage.indices.size() / 3 << "\n";
        }
    }
    return 0;
}
