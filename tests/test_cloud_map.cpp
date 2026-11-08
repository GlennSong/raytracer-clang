// THE WEATHER MAP THAT PLACES THE CLOUDS (ADR-0386; engine/sky/cloud_map.h, assets/scripts/clouds.lua): the book
// reads, the coverage the weather asks for is the sky's share under cloud on every seed, an overcast closes into a
// deck, and a cell's fate never depends on the map's window (two bakes agree where they meet).
#include "test_framework.h"

#include "../src/engine/scripting/clouds_lua.h"
#include "../src/engine/sky/cloud_map.h"

#include <cmath>
#include <fstream>
#include <sstream>

using namespace engine;

namespace {
std::string readCloudsFile() {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/clouds.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
// the shipped book on a smaller map (40 km, 78 m texels: the same texel, a quarter of the bake)
CloudBook labBook() {
    CloudBook b;
    std::string err;
    if (!loadCloudsSource(readCloudsFile(), b, &err)) return b;
    b.mapExtent = 40000.0f;
    b.mapSize = 512;
    return b;
}
float coverOf(const CloudBook& b, float coverage, float lo, float hi, uint32_t seed) {
    CloudMapInputs in;
    in.coverage = coverage;
    in.typeLo = lo;
    in.typeHi = hi;
    in.seed = seed;
    CloudMapImage m;
    bakeCloudMap(b, in, m);
    return cloudMapCover(m);
}
}  // namespace

TEST_CASE(clouds_lua_reads_four_types_mild_to_severe) {
    CloudBook b;
    std::string err;
    CHECK(loadCloudsSource(readCloudsFile(), b, &err));
    CHECK(err.empty());
    CHECK(b.loaded);
    CHECK(b.types[0].name == "stratus" && b.types[3].name == "cumulonimbus");
    for (const CloudTypeSpec& t : b.types) CHECK(t.top > t.base && t.radiusMax >= t.radiusMin);
    CHECK(b.types[3].top > b.types[2].top);   // the towers stand tallest
    CHECK(!loadCloudsSource("return { types = { {}, {} } }", b, &err));
    CHECK(!loadCloudsSource("return { types = { { base = 900, top = 500 }, {}, {}, {} } }", b, &err));
}

TEST_CASE(cloud_type_weights_spread_the_range) {
    const std::array<float, 4> one = cloudTypeWeights(2.0f / 3.0f, 2.0f / 3.0f);
    CHECK(one[2] == 1.0f);
    const std::array<float, 4> fair = cloudTypeWeights(0.40f, 0.75f);   // stratocumulus to cumulus
    CHECK(fair[0] == 0.0f && fair[3] == 0.0f);
    CHECK(fair[1] > 0.0f && fair[2] > fair[1]);
    CHECK(std::fabs(fair[0] + fair[1] + fair[2] + fair[3] - 1.0f) < 1e-5f);
}

TEST_CASE(cloud_map_cover_is_what_the_weather_asks_on_every_seed) {
    const CloudBook b = labBook();
    CHECK(b.loaded);
    // clear, fair, broken: the share of the sky under cloud within a few points of the asked, seed after seed
    const float asks[3] = {0.05f, 0.22f, 0.45f};
    for (float ask : asks)
        for (uint32_t seed = 1; seed <= 6; ++seed) {
            const float got = coverOf(b, ask, 0.40f, 0.75f, seed);
            CHECK(std::fabs(got - ask) < 0.05f + 0.25f * ask);
        }
    // none asked, none there
    CHECK(coverOf(b, 0.0f, 0.40f, 0.75f, 3) == 0.0f);
}

TEST_CASE(cloud_map_overcast_closes_into_a_deck) {
    const CloudBook b = labBook();
    const float got = coverOf(b, 0.85f, 0.0f, 0.40f, 7);   // stratus to stratocumulus
    CHECK(got > 0.75f);
}

TEST_CASE(cloud_map_bakes_agree_where_they_meet) {
    const CloudBook b = labBook();
    CloudMapInputs in;
    in.coverage = 0.3f;
    in.seed = 11;
    CloudMapImage a, c;
    bakeCloudMap(b, in, a);
    in.centreX = 9000.0;   // the camera drifted 9 km: the same air, another window
    in.centreZ = -4500.0;
    bakeCloudMap(b, in, c);
    const double texel = b.mapExtent / b.mapSize;
    const int di = static_cast<int>(std::lround((c.originX - a.originX) / texel));
    const int dj = static_cast<int>(std::lround((c.originZ - a.originZ) / texel));
    int same = 0, compared = 0;
    for (int j = 0; j < c.size; j += 7)
        for (int i = 0; i < c.size; i += 7) {
            const int ai = i + di, aj = j + dj;
            if (ai < 0 || aj < 0 || ai >= a.size || aj >= a.size) continue;
            ++compared;
            const std::size_t pa = (static_cast<std::size_t>(aj) * a.size + ai) * 4, pc = (static_cast<std::size_t>(j) * c.size + i) * 4;
            if (a.rgba[pa] == c.rgba[pc] && a.rgba[pa + 1] == c.rgba[pc + 1]) ++same;
        }
    CHECK(compared > 1000);
    CHECK(same == compared);
    // ...and another seed is another sky
    in.seed = 12;
    CloudMapImage d;
    bakeCloudMap(b, in, d);
    CHECK(d.rgba != c.rgba);
}
