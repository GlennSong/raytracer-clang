#include "paint_texture.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <unordered_map>

namespace roadlab {

namespace {

inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// One station's style set as a lookup key. Byte-exact rather than approximate:
// two rows that differ in the last bit of a dash length are two rows, and
// merging them would be a silent edit to the road. The bytes come straight from
// the same floats the texture will hold, so equal keys are pixels a GPU could
// not tell apart either.
std::string styleKey(const float* row) {
    return std::string(reinterpret_cast<const char*>(row),
                       size_t(kStyleTexelsPerRow) * 4 * sizeof(float));
}

}  // namespace

double PaintProfile::rowCoord(double s) const {
    if (stations.size() < 2) return double(rowBase);
    if (s <= stations.front()) return double(rowBase);
    if (s >= stations.back()) return double(rowBase + int(stations.size()) - 1);
    size_t hi = size_t(std::lower_bound(stations.begin(), stations.end(), s) - stations.begin());
    if (hi == 0) hi = 1;
    size_t lo = hi - 1;
    double span = stations[hi] - stations[lo];
    double u = span > 1e-12 ? (s - stations[lo]) / span : 0.0;
    return double(rowBase) + double(lo) + u;
}

int PaintAtlas::sample(int roadIndex, double row, RlBoundary* out, int maxCount) const {
    if (roadIndex < 0 || size_t(roadIndex) >= profiles.size()) return 0;
    const PaintProfile& prof = profiles[size_t(roadIndex)];
    int n = prof.rows();
    if (n == 0) return 0;

    double local = row - double(prof.rowBase);
    if (local < 0) local = 0;
    if (local > double(n - 1)) local = double(n - 1);

    int lo = clampi(int(std::floor(local)), 0, n - 1);
    int hi = clampi(lo + 1, 0, n - 1);
    float u = float(local - double(lo));
    // Nearest, matched to the hardware rule: a texel's footprint runs from its
    // centre minus half to its centre plus half, so the switch is at .5.
    int near = clampi(int(std::floor(local + 0.5)), 0, n - 1);

    const size_t pw = size_t(kProfileTexelsPerRow) * 4;
    const float* pLo = &prof.profile[size_t(lo) * pw];
    const float* pHi = &prof.profile[size_t(hi) * pw];
    const float* pNear = &prof.profile[size_t(near) * pw];

    int styleRow = clampi(int(pNear[size_t(kOffsetTexels) * 4]), 0, styleRows - 1);
    const float* sty = &styleTex[size_t(styleRow) * size_t(kStyleTexelsPerRow) * 4];

    int count = 0;
    for (int k = 0; k < kMaxBakedBoundaries && count < maxCount; ++k) {
        const float* a = &sty[size_t(k) * 8];
        if (a[0] < -0.5f) continue;   // padding: no boundary in this slot
        RlBoundary b;
        b.t = pLo[k] + (pHi[k] - pLo[k]) * u;
        b.style = a[0];
        b.width = a[1];
        b.gap = a[2];
        b.color = a[3];
        b.dashOn = a[4];
        b.dashOff = a[5];
        b.wear = a[6];
        out[count++] = b;
    }
    return count;
}

namespace {

// Build one road's rows and fold its style sets into the shared table.
void appendProfile(const Road& road, double step, PaintAtlas& atlas,
                   std::unordered_map<std::string, int>& styleIndex) {
    BoundaryStrip strip = bakeBoundaryStrip(road, step);
    PaintProfile prof;
    prof.stations = strip.stations;
    prof.rowBase = atlas.rows;
    size_t n = prof.stations.size();
    const size_t pw = size_t(kProfileTexelsPerRow) * 4;
    prof.profile.assign(n * pw, 0.0f);

    std::vector<float> scratch(size_t(kStyleTexelsPerRow) * 4, 0.0f);
    for (size_t i = 0; i < n; ++i) {
        const RlBoundary* row = &strip.rows[i * size_t(strip.stride)];
        float* p = &prof.profile[i * pw];

        // Padding is style -1 here too, for the same reason it is in the strip:
        // a slot that holds no boundary has to be distinguishable from one that
        // holds an unmarked boundary, or the shader renumbers every lane past
        // the first.
        for (int k = 0; k < kMaxBakedBoundaries; ++k) {
            p[k] = k < strip.stride ? row[k].t : 0.0f;
            float* a = &scratch[size_t(k) * 8];
            if (k < strip.stride) {
                a[0] = row[k].style;
                a[1] = row[k].width;
                a[2] = row[k].gap;
                a[3] = row[k].color;
                a[4] = row[k].dashOn;
                a[5] = row[k].dashOff;
                a[6] = row[k].wear;
            } else {
                a[0] = -1.0f;
                for (int j = 1; j < 7; ++j) a[j] = 0.0f;
            }
            a[7] = 0.0f;
        }

        std::string key = styleKey(scratch.data());
        auto it = styleIndex.find(key);
        int idx;
        if (it == styleIndex.end()) {
            idx = atlas.styleRows++;
            styleIndex.emplace(std::move(key), idx);
            atlas.styleTex.insert(atlas.styleTex.end(), scratch.begin(), scratch.end());
        } else {
            idx = it->second;
        }
        p[size_t(kOffsetTexels) * 4] = float(idx);
    }

    atlas.rows += int(n);
    atlas.profileTex.insert(atlas.profileTex.end(), prof.profile.begin(), prof.profile.end());
    atlas.profiles.push_back(std::move(prof));
}

}  // namespace

PaintAtlas bakePaintAtlas(const Network& net, double step) {
    PaintAtlas atlas;
    std::unordered_map<std::string, int> styleIndex;
    atlas.profiles.reserve(net.roads().size());
    for (const Road& r : net.roads()) appendProfile(r, step, atlas, styleIndex);
    return atlas;
}

PaintAtlas bakePaintAtlas(const Road& road, double step) {
    PaintAtlas atlas;
    std::unordered_map<std::string, int> styleIndex;
    appendProfile(road, step, atlas, styleIndex);
    return atlas;
}

}  // namespace roadlab
