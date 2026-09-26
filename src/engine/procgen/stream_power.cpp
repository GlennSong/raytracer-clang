#include "stream_power.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

namespace engine {

int streamPowerErode(Heightmap& hm, const std::vector<float>& uplift, const StreamPowerParams& p,
                     const std::vector<float>* erodibility) {
    if (erodibility && erodibility->size() != hm.h.size()) erodibility = nullptr;
    const int n = hm.n;
    if (n < 3 || uplift.size() != hm.h.size()) return 0;
    const std::size_t N = static_cast<std::size_t>(n) * n;
    const double cell = hm.worldSize > 0.0f ? hm.worldSize / (n - 1) : 1.0;
    const double cellArea = cell * cell;
    std::vector<double> h(hm.h.begin(), hm.h.end());

    // outlets: the grid's edge and the sea as it stands now (held at their height throughout)
    std::vector<uint8_t> outlet(N, 0);
    double landLo = 1e30, landHi = -1e30;
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) {
            const std::size_t i = static_cast<std::size_t>(z) * n + x;
            if (x == 0 || z == 0 || x == n - 1 || z == n - 1 || h[i] < p.seaLevel) outlet[i] = 1;
            else { landLo = std::min(landLo, h[i]); landHi = std::max(landHi, h[i]); }
        }

    const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1}, dz[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    const double dl[8] = {cell, cell, cell, cell, cell * std::sqrt(2.0), cell * std::sqrt(2.0), cell * std::sqrt(2.0), cell * std::sqrt(2.0)};
    std::vector<double> hf(N);
    std::vector<int32_t> rec(N), order;
    std::vector<double> area(N), rlen(N);
    std::vector<uint8_t> seen(N);
    order.reserve(N);
    using Item = std::pair<double, int32_t>;

    int it = 0;
    for (; it < p.iterations; ++it) {
        // 1. uplift
        for (std::size_t i = 0; i < N; ++i) if (!outlet[i]) h[i] += p.uplift * uplift[i] * p.dt;

        // 2. every cell drains: a priority flood from the outlets fills hollows with a tiny rise (so flats
        //    still slope), and each cell's receiver is the neighbour it was flooded FROM -- a tree rooted at
        //    the outlets, visited in `order` outlets first
        std::fill(seen.begin(), seen.end(), 0);
        order.clear();
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
        for (std::size_t i = 0; i < N; ++i)
            if (outlet[i]) { seen[i] = 1; hf[i] = h[i]; rec[i] = -1; q.push({h[i], static_cast<int32_t>(i)}); }
        while (!q.empty()) {
            const auto [lv, c] = q.top();
            q.pop();
            order.push_back(c);
            const int cx = c % n, cz = c / n;
            for (int k = 0; k < 8; ++k) {
                const int nx = cx + dx[k], nz = cz + dz[k];
                if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
                const std::size_t nb = static_cast<std::size_t>(nz) * n + nx;
                if (seen[nb]) continue;
                seen[nb] = 1;
                hf[nb] = std::max(h[nb], lv + 1e-4 * dl[k]);
                rec[nb] = c;
                rlen[nb] = dl[k];
                q.push({hf[nb], static_cast<int32_t>(nb)});
            }
        }
        // ...but a cell drains to its STEEPEST downhill neighbour on the filled surface where it has one (the
        // flood's tree otherwise follows the queue's order across open slopes)
        for (std::size_t i = 0; i < N; ++i) {
            if (outlet[i]) continue;
            const int cx = static_cast<int>(i % n), cz = static_cast<int>(i / n);
            double best = 0.0;
            for (int k = 0; k < 8; ++k) {
                const int nx = cx + dx[k], nz = cz + dz[k];
                if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
                const std::size_t nb = static_cast<std::size_t>(nz) * n + nx;
                const double s = (hf[i] - hf[nb]) / dl[k];
                if (s > best) { best = s; rec[i] = static_cast<int32_t>(nb); rlen[i] = dl[k]; }
            }
        }
        // the order must put every receiver before its donors: rebuild it from the tree (donor lists, outlets
        // first, breadth-first)
        {
            std::vector<int32_t> head(N, -1), next(N, -1);
            for (std::size_t i = 0; i < N; ++i)
                if (rec[i] >= 0) { next[i] = head[static_cast<std::size_t>(rec[i])]; head[static_cast<std::size_t>(rec[i])] = static_cast<int32_t>(i); }
            order.clear();
            for (std::size_t i = 0; i < N; ++i) if (rec[i] < 0) order.push_back(static_cast<int32_t>(i));
            for (std::size_t o = 0; o < order.size(); ++o)
                for (int32_t d = head[static_cast<std::size_t>(order[o])]; d >= 0; d = next[static_cast<std::size_t>(d)]) order.push_back(d);
        }
        // 3. drainage area, accumulated down the tree (donors before receivers: the order reversed)
        std::fill(area.begin(), area.end(), cellArea);
        for (std::size_t o = order.size(); o-- > 0;) {
            const int32_t c = order[o];
            if (rec[static_cast<std::size_t>(c)] >= 0) area[static_cast<std::size_t>(rec[static_cast<std::size_t>(c)])] += area[static_cast<std::size_t>(c)];
        }
        // 4. implicit incision, receivers first: h = (h + f h_r) / (1 + f), f = K dt A^m / L (n = 1)
        for (const int32_t c : order) {
            const std::size_t i = static_cast<std::size_t>(c);
            if (outlet[i] || rec[i] < 0) continue;
            const double hr = h[static_cast<std::size_t>(rec[i])];
            const double f = p.K * (erodibility ? (*erodibility)[i] : 1.0) * p.dt * std::pow(area[i], p.m) / rlen[i];
            h[i] = (h[i] + f * hr) / (1.0 + f);   // between h and h_r: a hollow below its outlet silts up toward it
        }
        // 5. hillslope diffusion (explicit, sub-stepped for stability)
        if (p.diffusion > 0.0) {
            const double lim = 0.2 * cellArea / p.diffusion;   // stable explicit step
            const int sub = std::max(1, static_cast<int>(std::ceil(p.dt / lim)));
            const double sdt = p.dt / sub;
            std::vector<double> lap(N, 0.0);
            for (int s = 0; s < std::min(sub, 16); ++s) {   // capped: creep is the gentle term here
                for (int z = 1; z < n - 1; ++z)
                    for (int x = 1; x < n - 1; ++x) {
                        const std::size_t i = static_cast<std::size_t>(z) * n + x;
                        lap[i] = (h[i - 1] + h[i + 1] + h[i - static_cast<std::size_t>(n)] + h[i + static_cast<std::size_t>(n)] - 4.0 * h[i]) / cellArea;
                    }
                for (std::size_t i = 0; i < N; ++i) if (!outlet[i]) h[i] += p.diffusion * sdt * lap[i];
            }
        }
    }

    // back onto the land's starting range: the physics decides the SHAPE, the level decides how high
    if (p.targetHi > p.targetLo) { landLo = p.targetLo; landHi = p.targetHi; }
    if (p.rescale && landHi > landLo) {
        double lo = 1e30, hi = -1e30;
        for (std::size_t i = 0; i < N; ++i) if (!outlet[i]) { lo = std::min(lo, h[i]); hi = std::max(hi, h[i]); }
        const double s = hi > lo ? (landHi - landLo) / (hi - lo) : 1.0;
        for (std::size_t i = 0; i < N; ++i) if (!outlet[i]) h[i] = landLo + (h[i] - lo) * s;
    }
    for (std::size_t i = 0; i < N; ++i) hm.h[i] = static_cast<float>(h[i]);
    return it;
}

}  // namespace engine
