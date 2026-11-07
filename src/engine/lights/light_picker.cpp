#include "light_picker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace engine {

float lightEdgeFade(float d, float edge) {
    if (edge <= 0.0f) return 1.0f;
    const float e = std::max(1.0f, edge);
    float t = (e - d) / (0.25f * e);
    t = std::min(1.0f, std::max(0.0f, t));
    return t * t * (3.0f - 2.0f * t);
}

float lightImportance(float intensity, float range, float d) {
    if (d >= range) return 0.0f;
    const float dd = std::max(1.0f, d);
    return intensity / (dd * dd);
}

void pickLights(const LightBook& book, const std::vector<LightPool>& pools, const std::vector<LightCandidate>& cands,
                const Vec3& eye, float dusk, std::vector<PointLight>& points, std::vector<SpotLight>& spots, int cap) {
    const int nKinds = static_cast<int>(book.kinds.size());
    // each candidate's fade: -1 not picked; pool-less kinds are always 1
    std::vector<float> fade(cands.size(), -1.0f);
    std::vector<float> dist(cands.size(), 0.0f);
    for (std::size_t i = 0; i < cands.size(); ++i) {
        dist[i] = static_cast<float>((cands[i].position - eye).length());
        const int k = cands[i].kind;
        if (k >= 0 && k < nKinds && book.kinds[static_cast<std::size_t>(k)].pool < 0) fade[i] = 1.0f;
    }
    // per pool: the units (a candidate, or a whole group) in reach, ranked mine first then nearest
    struct Unit { bool mine; float d; uint32_t group; std::size_t first; };
    std::vector<Unit> units;
    std::unordered_map<uint32_t, std::size_t> unitOf;   // a group's unit
    std::unordered_set<uint32_t> picked;                 // the groups picked
    for (std::size_t p = 0; p < pools.size(); ++p) {
        const LightPool& pool = pools[p];
        units.clear();
        unitOf.clear();
        for (std::size_t i = 0; i < cands.size(); ++i) {
            const int k = cands[i].kind;
            if (k < 0 || k >= nKinds || book.kinds[static_cast<std::size_t>(k)].pool != static_cast<int>(p)) continue;
            if (pool.within > 0.0f && dist[i] >= pool.within && !cands[i].mine) continue;
            const uint32_t g = pool.groups ? cands[i].group : 0u;
            if (g != 0u) {
                const std::unordered_map<uint32_t, std::size_t>::iterator it = unitOf.find(g);
                if (it != unitOf.end()) {
                    Unit& u = units[it->second];
                    u.mine = u.mine || cands[i].mine;
                    u.d = std::min(u.d, dist[i]);
                    continue;
                }
                unitOf[g] = units.size();
            }
            units.push_back({cands[i].mine, dist[i], g, i});
        }
        const std::size_t keep = std::min(units.size(), static_cast<std::size_t>(std::max(0, pool.count)));
        auto before = [](const Unit& a, const Unit& b) { return a.mine != b.mine ? a.mine : a.d < b.d; };
        std::partial_sort(units.begin(), units.begin() + static_cast<std::ptrdiff_t>(keep), units.end(), before);
        float edge = pool.within;
        if (units.size() > keep && keep > 0) {   // the count binds: the farthest picked is the edge that moves
            float farthest = 0.0f;
            for (std::size_t u = 0; u < keep; ++u) farthest = std::max(farthest, units[u].d);
            edge = edge > 0.0f ? std::min(edge, farthest) : farthest;
        }
        // the picked: single candidates directly, groups by id in one pass over the candidates
        picked.clear();
        for (std::size_t u = 0; u < keep; ++u) {
            if (units[u].group != 0u) { picked.insert(units[u].group); continue; }
            const std::size_t i = units[u].first;
            fade[i] = (cands[i].mine || !pool.fade) ? 1.0f : lightEdgeFade(dist[i], edge);
        }
        if (!picked.empty())
            for (std::size_t i = 0; i < cands.size(); ++i) {
                const int k = cands[i].kind;
                if (cands[i].group == 0u || k < 0 || k >= nKinds || book.kinds[static_cast<std::size_t>(k)].pool != static_cast<int>(p) ||
                    picked.count(cands[i].group) == 0)
                    continue;
                fade[i] = (cands[i].mine || !pool.fade) ? 1.0f : lightEdgeFade(dist[i], edge);
            }
    }
    // the lit, and (over the cap) only the most important of them
    std::vector<float> lit(cands.size(), 0.0f);
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < cands.size(); ++i) {
        if (fade[i] <= 0.0f) continue;
        const LightKind& k = book.kinds[static_cast<std::size_t>(cands[i].kind)];
        lit[i] = k.intensity * cands[i].scale * fade[i] * (k.dusk ? dusk : 1.0f);
        if (lit[i] > 0.0f) order.push_back(i);
    }
    if (static_cast<int>(order.size()) > std::max(0, cap)) {
        std::vector<float> score(cands.size(), 0.0f);
        for (std::size_t i : order) {
            const LightKind& k = book.kinds[static_cast<std::size_t>(cands[i].kind)];
            score[i] = cands[i].mine ? std::numeric_limits<float>::max()
                                     : lightImportance(lit[i], cands[i].range > 0.0f ? cands[i].range : k.range, dist[i]);
        }
        const std::size_t keepN = static_cast<std::size_t>(std::max(0, cap));
        std::nth_element(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(keepN), order.end(),
                         [&](std::size_t a, std::size_t b) { return score[a] > score[b]; });
        order.resize(keepN);
        std::sort(order.begin(), order.end());   // back in the candidates' order
    }
    for (std::size_t i : order) {
        const LightCandidate& c = cands[i];
        const LightKind& k = book.kinds[static_cast<std::size_t>(c.kind)];
        const float intensity = lit[i];
        const float range = c.range > 0.0f ? c.range : k.range;
        if (k.spot) {
            SpotLight s(c.position + k.offset, c.direction, k.color, intensity, k.inner, k.outer);
            s.range = range;
            spots.push_back(s);
        } else {
            PointLight pl(c.position + k.offset, k.color, intensity);
            pl.range = range;
            points.push_back(pl);
        }
    }
}

}  // namespace engine
