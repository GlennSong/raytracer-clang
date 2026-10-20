#include "rock_layers.h"

#include <algorithm>
#include <cmath>

namespace engine {

RockLayersParams rockLayersFromJson(const nlohmann::json& j) {
    RockLayersParams p;
    p.thicknessM = j.value("thickness", p.thicknessM);
    p.dipDeg = j.value("dip", p.dipDeg);
    p.dipScaleM = j.value("dipScale", p.dipScaleM);
    p.thicknessScaleM = j.value("thicknessScale", p.thicknessScaleM);
    p.warpM = j.value("warp", p.warpM);
    p.hard = j.value("hard", p.hard);
    p.lateralScaleM = j.value("lateralScale", p.lateralScaleM);
    p.hardK = j.value("hardK", p.hardK);
    p.hardDiffusion = j.value("hardDiffusion", p.hardDiffusion);
    p.toneThicknessM = j.value("toneThickness", p.toneThicknessM);
    return p;
}

RockLayers::RockLayers(const RockLayersParams& p, uint32_t seed)
    : p_(p), n_(seed * 2654435761u + 91u), tanDip_(std::tan(p.dipDeg * 3.14159265358979 / 180.0)), seed_(seed) {}

double RockLayers::bedCoordinate(double x, double z, double h, double thickness) const {
    const double dipA = 6.2831853 * n_.fbm2(x / p_.dipScaleM + 4.1, z / p_.dipScaleM - 2.7, 2);
    const double thick = thickness * (0.6 + 0.9 * (0.5 + 0.5 * n_.fbm2(x / p_.thicknessScaleM - 1.3, z / p_.thicknessScaleM + 8.2, 2)));
    const double warpAmp = p_.warpM > 0.0 ? p_.warpM : 0.5 * thickness;
    const double warp = warpAmp * n_.fbm2(x / (3.0 * thickness) + 3.3, z / (3.0 * thickness) - 6.1, 3);
    return (h + warp - tanDip_ * (x * std::cos(dipA) + z * std::sin(dipA))) / std::max(1.0, thick);
}

double RockLayers::tone(double x, double z, double h) const {
    const double u = bedCoordinate(x, z, h, p_.toneThicknessM);
    auto bedTone = [&](double f) {
        uint32_t hs = static_cast<uint32_t>(static_cast<int64_t>(f)) * 0x27D4EB2Fu ^ seed_ * 0x165667B1u;
        hs ^= hs >> 15; hs *= 0x85EBCA77u; hs ^= hs >> 13; hs *= 0xC2B2AE3Du; hs ^= hs >> 16;
        // (pushed toward pale or dark: beds read as alternating bands, not a soft grey ramp)
        const double r = std::clamp(((hs & 0xFFFFFFu) / 16777216.0 - 0.2) / 0.6, 0.0, 1.0);
        return r * r * (3.0 - 2.0 * r);
    };
    const double f = std::floor(u), t = u - f;
    // graded over the last 15% of a band into the next (a contact reads as a line, not a step in the texture)
    const double k = std::clamp((t - 0.85) / 0.15, 0.0, 1.0);
    return bedTone(f) + (bedTone(f + 1.0) - bedTone(f)) * k * k * (3.0 - 2.0 * k);
}

double RockLayers::hardness(double x, double z, double h) const {
    const double u = bedCoordinate(x, z, h, p_.thicknessM);
    const double f = std::floor(u), t = u - f;
    uint32_t hs = static_cast<uint32_t>(static_cast<int64_t>(f)) * 0x9E3779B1u ^ seed_ * 0x85EBCA77u;
    hs ^= hs >> 16; hs *= 0x7FEB352Du; hs ^= hs >> 15; hs *= 0x846CA68Bu; hs ^= hs >> 16;
    // hard by its index (a share `hard`), softened at its top and bottom contacts (a bed's edges grade into the next)
    // and in patches along its outcrop
    const double bed = std::clamp(((hs & 0xFFFFFFu) / 16777216.0 - (1.0 - p_.hard)) / 0.12, 0.0, 1.0);
    const double contact = std::clamp(std::min(t, 1.0 - t) / 0.08, 0.0, 1.0);
    const double lateral = std::clamp((0.5 + 0.5 * n_.fbm2(x / p_.lateralScaleM + 1.7, z / p_.lateralScaleM - 3.1, 2) - 0.3) / 0.25, 0.0, 1.0);
    return bed * contact * lateral;
}

}  // namespace engine
