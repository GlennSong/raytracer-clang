#ifndef RAYTRACER_ENGINE_PROCGEN_COLOR_RAMP_H
#define RAYTRACER_ENGINE_PROCGEN_COLOR_RAMP_H

#include "../../rt_math.h"   // Vec3, lerp

#include <algorithm>
#include <initializer_list>
#include <vector>

namespace engine {

// A multi-stop colour gradient (procedural-planet-plan, the colour-ramp gap
// ADR-0076 flags): sorted (position, colour) stops, linearly interpolated between,
// clamped to the end stops outside their range. The biome "wardrobe" for a planet
// surface — elevation (or latitude, or any 0..1 mask) maps through it to an RGB.
struct ColorRamp {
    struct Stop {
        double t;
        Vec3 color;
    };
    std::vector<Stop> stops;

    ColorRamp() = default;
    ColorRamp(std::initializer_list<Stop> s) : stops(s) { sort(); }

    void add(double t, const Vec3& c) {
        stops.push_back({t, c});
        sort();
    }

    Vec3 eval(double t) const {
        if (stops.empty()) return Vec3(0, 0, 0);
        if (t <= stops.front().t) return stops.front().color;
        if (t >= stops.back().t) return stops.back().color;
        for (std::size_t i = 1; i < stops.size(); i++) {
            if (t <= stops[i].t) {
                const double span = stops[i].t - stops[i - 1].t;
                const double f = span > 1e-9 ? (t - stops[i - 1].t) / span : 0.0;
                return lerp(stops[i - 1].color, stops[i].color, f);
            }
        }
        return stops.back().color;
    }

private:
    void sort() {
        std::stable_sort(stops.begin(), stops.end(),
                         [](const Stop& a, const Stop& b) { return a.t < b.t; });
    }
};

}  // namespace engine

#endif
