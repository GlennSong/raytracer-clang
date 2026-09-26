#include "trails.h"

#include <algorithm>
#include <cmath>

namespace engine {

TrailNetwork::TrailNetwork(const std::vector<std::vector<Vec2>>& lines) {
    double lo[2] = {1e30, 1e30}, hi[2] = {-1e30, -1e30};
    for (const auto& L : lines)
        for (std::size_t k = 0; k + 1 < L.size(); ++k) {
            segs_.push_back({L[k], L[k + 1]});
            for (const Vec2& p : {L[k], L[k + 1]}) {
                lo[0] = std::min(lo[0], p.x); lo[1] = std::min(lo[1], p.y);
                hi[0] = std::max(hi[0], p.x); hi[1] = std::max(hi[1], p.y);
            }
        }
    if (segs_.empty()) return;
    x0_ = lo[0] - bin_; z0_ = lo[1] - bin_;
    nx_ = static_cast<int>((hi[0] - x0_) / bin_) + 2;
    nz_ = static_cast<int>((hi[1] - z0_) / bin_) + 2;
    bins_.assign(static_cast<std::size_t>(nx_) * nz_, {});
    for (std::size_t s = 0; s < segs_.size(); ++s) {
        const Seg& g = segs_[s];
        const int i0 = static_cast<int>((std::min(g.a.x, g.b.x) - x0_) / bin_), i1 = static_cast<int>((std::max(g.a.x, g.b.x) - x0_) / bin_);
        const int j0 = static_cast<int>((std::min(g.a.y, g.b.y) - z0_) / bin_), j1 = static_cast<int>((std::max(g.a.y, g.b.y) - z0_) / bin_);
        for (int j = std::max(0, j0); j <= std::min(nz_ - 1, j1); ++j)
            for (int i = std::max(0, i0); i <= std::min(nx_ - 1, i1); ++i) bins_[static_cast<std::size_t>(j) * nx_ + i].push_back(static_cast<int>(s));
    }
}

TrailNetwork TrailNetwork::fromJson(const nlohmann::json& trails) {
    std::vector<std::vector<Vec2>> lines;
    if (trails.is_array())
        for (const auto& L : trails) {
            std::vector<Vec2> pts;
            if (L.is_array())
                for (const auto& p : L)
                    if (p.is_array() && p.size() >= 2) pts.emplace_back(p[0].get<double>(), p[1].get<double>());
            if (pts.size() >= 2) lines.push_back(std::move(pts));
        }
    return TrailNetwork(lines);
}

double TrailNetwork::distance(double x, double z, double maxD) const {
    if (segs_.empty()) return maxD;
    const int r = static_cast<int>(std::ceil(maxD / bin_));
    const int ci = static_cast<int>((x - x0_) / bin_), cj = static_cast<int>((z - z0_) / bin_);
    double best = maxD;
    const Vec2 q(x, z);
    for (int j = std::max(0, cj - r); j <= std::min(nz_ - 1, cj + r); ++j)
        for (int i = std::max(0, ci - r); i <= std::min(nx_ - 1, ci + r); ++i)
            for (int s : bins_[static_cast<std::size_t>(j) * nx_ + i]) {
                const Seg& g = segs_[static_cast<std::size_t>(s)];
                const Vec2 ab = g.b - g.a;
                const double L2 = dot(ab, ab);
                const double t = L2 > 1e-12 ? std::clamp(dot(q - g.a, ab) / L2, 0.0, 1.0) : 0.0;
                best = std::min(best, (q - (g.a + ab * t)).length());
            }
    return best;
}

}  // namespace engine
