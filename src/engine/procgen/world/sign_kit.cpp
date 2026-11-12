#include "sign_kit.h"

#include "../noise.h"          // valueNoise2
#include "../../../hash.h"     // lowbias32
#include "../../../rt_math.h"  // smoothstep01

#include <algorithm>
#include <cmath>

namespace engine {

void wearSignFace(TextImage& colour, TextImage* shine, double wear, uint32_t seed, const SignWearLook& look, double k) {
    wear = std::clamp(wear, 0.0, 1.0);
    if (wear <= 0 || colour.w <= 0 || k <= 0) return;
    // three independent noise fields from the one seed: where it faded, and the grime's coarse and fine blotches
    const uint32_t sFade = lowbias32(seed ^ 0x51u), sDirt = lowbias32(seed ^ 0xD1u), sFine = lowbias32(seed ^ 0xF1u);
    const double hMetres = colour.h / k;
    const double cellX = std::max(1e-3, look.dirtCell), cellY = cellX * std::max(1.0, look.streak);
    const double shineKeep = 1.0 - wear * std::clamp(look.shineLoss, 0.0, 1.0);
    for (int j = 0; j < colour.h; ++j) {
        const double y = j / k;
        const double low = look.dirtLow * (y / std::max(1e-6, hMetres));   // more grime toward the bottom edge
        for (int i = 0; i < colour.w; ++i) {
            uint8_t* px = &colour.rgba[(static_cast<std::size_t>(j) * colour.w + i) * 4];
            if (px[3] == 0) continue;
            const double x = i / k;
            double r = px[0] / 255.0, g = px[1] / 255.0, b = px[2] / 255.0;
            // FADE: toward a greyer, lighter tone, patchily
            const double patch = 0.6 + 0.8 * valueNoise2(x / look.fadeCell, y / look.fadeCell, sFade);
            const double f = std::clamp(wear * look.fade * patch, 0.0, 1.0);
            const double lum = 0.2126 * r + 0.7152 * g + 0.0722 * b;
            auto faded = [&](double c) {
                const double grey = c + (lum - c) * look.fadeGrey;
                return grey + (1.0 - grey) * look.fadeLift;
            };
            r += (faded(r) - r) * f;
            g += (faded(g) - g) * f;
            b += (faded(b) - b) * f;
            // GRIME: blotches stretched downward, a fine grain over them, gathering low
            const double n = 0.7 * valueNoise2(x / cellX, y / cellY, sDirt) + 0.3 * valueNoise2(x / (cellX * 0.3), y / (cellY * 0.3), sFine);
            const double d = std::clamp(wear * look.dirt * smoothstep01(std::clamp((n + low - 0.35) / 0.5, 0.0, 1.0)), 0.0, 1.0);
            r += (look.dirtR - r) * d;
            g += (look.dirtG - g) * d;
            b += (look.dirtB - b) * d;
            px[0] = static_cast<uint8_t>(std::lround(std::clamp(r, 0.0, 1.0) * 255.0));
            px[1] = static_cast<uint8_t>(std::lround(std::clamp(g, 0.0, 1.0) * 255.0));
            px[2] = static_cast<uint8_t>(std::lround(std::clamp(b, 0.0, 1.0) * 255.0));
            // the shine: aged sheeting returns less light, and grime covers it
            if (shine && i < shine->w && j < shine->h) {
                uint8_t* s = &shine->rgba[(static_cast<std::size_t>(j) * shine->w + i) * 4];
                const double keep = shineKeep * (1.0 - d);
                for (int c = 0; c < 3; ++c) s[c] = static_cast<uint8_t>(std::lround(s[c] * keep));
            }
        }
    }
}

}  // namespace engine
