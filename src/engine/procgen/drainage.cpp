#include "drainage.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>

namespace engine {

void floodDrainage(const std::vector<double>& h, int n, double cell, const std::vector<uint8_t>& outlet,
                   double risePerMetre, DrainageFlood& out) {
    const std::size_t N = static_cast<std::size_t>(n) * n;
    out.filled.resize(N);
    out.receiver.resize(N);
    out.receiverLength.resize(N);
    out.seen.assign(N, 0);
    out.order.clear();
    out.order.reserve(N);
    const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1}, dz[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    const double diag = cell * std::sqrt(2.0);
    const double dl[8] = {cell, cell, cell, cell, diag, diag, diag, diag};
    using Item = std::pair<double, int32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    for (std::size_t i = 0; i < N; ++i)
        if (outlet[i]) {
            out.seen[i] = 1;
            out.filled[i] = h[i];
            out.receiver[i] = -1;
            q.push({h[i], static_cast<int32_t>(i)});
        }
    while (!q.empty()) {
        const auto [lv, c] = q.top();
        q.pop();
        out.order.push_back(c);
        const int cx = c % n, cz = c / n;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dx[k], nz = cz + dz[k];
            if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
            const std::size_t nb = static_cast<std::size_t>(nz) * n + nx;
            if (out.seen[nb]) continue;
            out.seen[nb] = 1;
            out.filled[nb] = std::max(h[nb], lv + risePerMetre * dl[k]);
            out.receiver[nb] = c;
            out.receiverLength[nb] = dl[k];
            q.push({out.filled[nb], static_cast<int32_t>(nb)});
        }
    }
}

}  // namespace engine
