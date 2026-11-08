#include "probe_sites.h"
#include "../procgen/city/road_network.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace engine {

namespace {
uint64_t mergeKey(int64_t cx, int64_t cz) { return (static_cast<uint64_t>(cx) << 32) ^ static_cast<uint64_t>(static_cast<uint32_t>(cz)); }
}  // namespace

void placeProbeSites(const RoadGraph& g, const ProbeGroundFn& ground, const ProbeRules& rules, std::vector<ProbeSite>& out) {
    const double spacing = std::max(rules.spacing, 5.0);
    const double merge = std::max(rules.merge, 1.0);
    // the sites so far, by merge cell, so a junction's arms share one
    std::unordered_map<uint64_t, std::vector<std::size_t>> cells;
    auto nearOne = [&](const Vec3& p) {
        const int64_t cx = static_cast<int64_t>(std::floor(p.x / merge)), cz = static_cast<int64_t>(std::floor(p.z / merge));
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dz = -1; dz <= 1; ++dz) {
                const auto it = cells.find(mergeKey(cx + dx, cz + dz));
                if (it == cells.end()) continue;
                for (std::size_t i : it->second) {
                    const Vec3 d = out[i].position - p;
                    if (d.x * d.x + d.z * d.z < merge * merge && std::fabs(d.y) < 6.0) return true;
                }
            }
        return false;
    };
    // a node's carriageway height (absolute)
    auto nodeY = [&](const RoadNode& n) {
        return n.elevAbsolute ? static_cast<double>(n.elev) : ground(n.pos.x, n.pos.y) + n.elev;
    };
    for (const RoadEdge& e : g.edges) {
        if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size()) || e.b >= static_cast<int>(g.nodes.size())) continue;
        const RoadNode& na = g.nodes[static_cast<std::size_t>(e.a)];
        const RoadNode& nb = g.nodes[static_cast<std::size_t>(e.b)];
        const double dx = nb.pos.x - na.pos.x, dz = nb.pos.y - na.pos.y;
        const double len = std::sqrt(dx * dx + dz * dz);
        if (len < 1e-3) continue;
        const double ux = dx / len, uz = dz / len;
        const int n = std::max(1, static_cast<int>(std::lround(len / spacing)));
        // at grade both ends: follow the ground between them; a deck or a ramp: the line between its ends
        const bool atGrade = !na.elevAbsolute && !nb.elevAbsolute;
        const double ya = nodeY(na), yb = nodeY(nb);
        // the box: the piece of street this site stands for (with a little overlap), past the kerbs to the frontages
        const double along = len / n * 0.5 + 5.0;
        const double across = e.width * 0.5 + rules.side;
        const double hx = std::fabs(ux) * along + std::fabs(uz) * across;
        const double hz = std::fabs(uz) * along + std::fabs(ux) * across;
        for (int k = 0; k < n; ++k) {
            const double t = (k + 0.5) / n;
            const double x = na.pos.x + dx * t, z = na.pos.y + dz * t;
            const double road = atGrade ? ground(x, z) + na.elev + (nb.elev - na.elev) * t : ya + (yb - ya) * t;
            ProbeSite s;
            s.position = Vec3(x, road + rules.height, z);
            if (nearOne(s.position)) continue;
            s.boxMin = Vec3(x - hx, road - rules.below, z - hz);
            s.boxMax = Vec3(x + hx, road + rules.above, z + hz);
            const int64_t cx = static_cast<int64_t>(std::floor(x / merge)), cz = static_cast<int64_t>(std::floor(z / merge));
            cells[mergeKey(cx, cz)].push_back(out.size());
            out.push_back(s);
        }
    }
}

bool pickResidentProbes(const std::vector<ProbeSite>& sites, const Vec3& eye, double hysteresis, std::vector<int>& resident) {
    const std::size_t slots = resident.size();
    if (slots == 0) return false;
    bool changed = false;
    // the nearest `slots` sites (horizontal distance: a probe serves its street, whatever the camera's height)
    std::vector<std::pair<double, int>> byDist;
    byDist.reserve(sites.size());
    for (std::size_t i = 0; i < sites.size(); ++i) {
        const double dx = sites[i].position.x - eye.x, dz = sites[i].position.z - eye.z;
        byDist.emplace_back(dx * dx + dz * dz, static_cast<int>(i));
    }
    const std::size_t want = std::min(slots, byDist.size());
    if (want < byDist.size())
        std::nth_element(byDist.begin(), byDist.begin() + static_cast<std::ptrdiff_t>(want), byDist.end());
    std::sort(byDist.begin(), byDist.begin() + static_cast<std::ptrdiff_t>(want));
    const double edge = want > 0 ? std::sqrt(byDist[want - 1].first) : 0.0;
    std::vector<char> isResident(sites.size(), 0);
    // leave: a site gone (the level changed) or drifted past the edge + hysteresis
    for (int& r : resident) {
        if (r < 0) continue;
        if (r >= static_cast<int>(sites.size())) { r = -1; changed = true; continue; }
        const double dx = sites[static_cast<std::size_t>(r)].position.x - eye.x, dz = sites[static_cast<std::size_t>(r)].position.z - eye.z;
        if (std::sqrt(dx * dx + dz * dz) > edge + hysteresis) { r = -1; changed = true; continue; }
        isResident[static_cast<std::size_t>(r)] = 1;
    }
    // join: the nearest wanted sites not yet in, into the free slots
    std::size_t slot = 0;
    for (std::size_t k = 0; k < want; ++k) {
        const int s = byDist[k].second;
        if (isResident[static_cast<std::size_t>(s)]) continue;
        while (slot < slots && resident[slot] >= 0) ++slot;
        if (slot == slots) break;
        resident[slot] = s;
        isResident[static_cast<std::size_t>(s)] = 1;
        changed = true;
    }
    return changed;
}

}  // namespace engine
