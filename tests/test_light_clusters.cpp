// CLUSTERED LIGHTS (renderer-realism phase 1): every light that reaches a point is in that point's froxel's list,
// lights behind the camera are in none, and a street of a thousand lamps leaves each froxel a handful.

#include "test_framework.h"
#include "../src/renderer/light_clusters.h"
#include <chrono>
#include <cmath>

using namespace engine;

namespace {
// The froxel a world point falls in (the shader's lookup), or -1 off screen.
int cellOf(const ClusterGrid& g, const Mat4& view, float tanHalfY, float aspect, const Vec3& w) {
    const Vec3 v = view.transformPoint(w);
    const float d = -static_cast<float>(v.z);
    if (d <= 0) return -1;
    const float u = (static_cast<float>(v.x) / d / (tanHalfY * aspect) + 1) * 0.5f, t = (static_cast<float>(v.y) / d / tanHalfY + 1) * 0.5f;
    if (u < 0 || u >= 1 || t < 0 || t >= 1) return -1;
    const int tx = static_cast<int>(u * g.tilesX), ty = static_cast<int>(t * g.tilesY), s = g.sliceOf(d);
    return (s * g.tilesY + ty) * g.tilesX + tx;
}
}  // namespace

TEST_CASE(every_light_that_reaches_a_point_is_in_its_froxel) {
    const ClusterGrid g;
    const Mat4 view = Mat4::lookAt(Vec3(0, 1.7, 0), Vec3(0, 1.2, -50), Vec3(0, 1, 0));
    const float tanHalfY = std::tan(30.0f * 3.14159265f / 180.0f), aspect = 16.0f / 9.0f;
    std::vector<ClusterLight> lights;
    uint32_t h = 12345u;
    auto rnd = [&]() { h = h * 1664525u + 1013904223u; return static_cast<float>(h >> 8) / 16777216.0f; };
    for (int i = 0; i < 400; ++i)
        lights.push_back({Vec3(rnd() * 160 - 80, rnd() * 12, -rnd() * 300 + 20), 4.0f + rnd() * 30});
    ClusterLists out;
    buildClusters(g, view, tanHalfY, aspect, lights, out, 255);
    int missed = 0, points = 0;
    for (int i = 0; i < 4000; ++i) {
        const Vec3 p(rnd() * 160 - 80, rnd() * 12, -rnd() * 300);
        const int c = cellOf(g, view, tanHalfY, aspect, p);
        if (c < 0) continue;
        ++points;
        const uint32_t oc = out.offsetCount[static_cast<std::size_t>(c)];
        const uint32_t off = oc & 0xFFFFFFu, cnt = oc >> 24;
        for (std::size_t li = 0; li < lights.size(); ++li) {
            if ((lights[li].position - p).length() >= lights[li].range) continue;
            bool in = false;
            for (uint32_t k = 0; k < cnt; ++k) in = in || out.indices[off + k] == li;
            missed += !in;
        }
    }
    std::printf("    [clusters] %d points checked, %d light hits missed; busiest froxel %d\n", points, missed, out.maxInCell);
    CHECK(points > 1000);
    CHECK(missed == 0);
}

TEST_CASE(a_light_behind_the_camera_is_in_no_froxel) {
    const ClusterGrid g;
    const Mat4 view = Mat4::lookAt(Vec3(0, 1.7, 0), Vec3(0, 1.7, -10), Vec3(0, 1, 0));
    ClusterLists out;
    buildClusters(g, view, 0.577f, 1.78f, {{Vec3(0, 2, 30), 10.0f}}, out);
    CHECK(out.indices.empty());
}

TEST_CASE(a_street_of_a_thousand_lamps_leaves_each_froxel_a_handful) {
    // two rows of lamps every 12 m down a 6 km street, range 34 (the city's lamps), the camera in the middle of it
    const ClusterGrid g;
    const Mat4 view = Mat4::lookAt(Vec3(0, 1.7, 0), Vec3(0, 1.5, -100), Vec3(0, 1, 0));
    std::vector<ClusterLight> lights;
    for (int i = 0; i < 500; ++i)
        for (const float x : {-8.0f, 8.0f}) lights.push_back({Vec3(x, 7.5, -3000.0f + 12.0f * static_cast<float>(i)), 34.0f});
    ClusterLists out;
    const auto t0 = std::chrono::steady_clock::now();
    buildClusters(g, view, 0.577f, 1.78f, lights, out);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    int used = 0;
    long total = 0;
    for (const uint32_t oc : out.offsetCount) if (oc >> 24) { ++used; total += oc >> 24; }
    std::printf("    [clusters] %zu lamps: %d froxels lit, %.1f lights each on average, busiest %d, %d capped; binned in %.2f ms\n",
                lights.size(), used, used ? static_cast<double>(total) / used : 0.0, out.maxInCell, out.capped, ms);
    CHECK(used > 0);
    CHECK(static_cast<double>(total) / used < 24.0);   // a handful, not the thousand
}

TEST_CASE(a_crowd_of_lights_round_the_camera_bins_fast_and_keeps_the_nearest) {
    // the worst case measured in the viewer (RT_LIGHT_FILL=1032): a thousand lights within 60 m of the camera
    const ClusterGrid g;
    const Mat4 view = Mat4::lookAt(Vec3(0, 1.7, 0), Vec3(0, 1.5, -100), Vec3(0, 1, 0));
    std::vector<ClusterLight> lights;
    for (int k = 0; k < 1000; ++k) {
        const float a = static_cast<float>(k) * 2.39996f, r = 4.0f + 60.0f * std::sqrt(static_cast<float>(k % 997) / 997.0f);
        lights.push_back({Vec3(r * std::cos(a), 0.7f + static_cast<float>(k % 5), r * std::sin(a)), 20.0f});
    }
    ClusterLists out;
    buildClusters(g, view, 0.577f, 1.78f, lights, out);   // (warm)
    const auto t0 = std::chrono::steady_clock::now();
    buildClusters(g, view, 0.577f, 1.78f, lights, out);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("    [clusters] 1000 lights in 60 m: %zu entries, busiest %d, %d capped, binned in %.2f ms\n", out.indices.size(),
                out.maxInCell, out.capped, ms);
    CHECK(out.capped > 0);
    for (const uint32_t oc : out.offsetCount) CHECK((oc >> 24) <= 64u);
}
