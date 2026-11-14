#include "probe_sites.h"
#include "../ai/nav_graph.h"   // the street map the sites stand along

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace engine {

namespace {
uint64_t mergeKey(int64_t cx, int64_t cz) { return (static_cast<uint64_t>(cx) << 32) ^ static_cast<uint64_t>(static_cast<uint32_t>(cz)); }
}  // namespace

void placeProbeSites(const NavGraph& map, const ProbeGroundFn& ground, const ProbeRules& rules, std::vector<ProbeSite>& out) {
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
    // the street map's links as pieces of road, each once (a two-way street's two links are one piece), each end's height
    // as its link carries it: an absolute deck height, or a height above the ground
    std::unordered_map<uint64_t, char> done;
    for (int li = 0; li < map.streetLinkCount(); ++li) {
        const NavLink& e = map.links[static_cast<std::size_t>(li)];
        if (e.footpath || e.from < 0 || e.to < 0) continue;
        if (!done.emplace(mergeKey(std::min(e.from, e.to), std::max(e.from, e.to)), 1).second) continue;
        const Vec2 pa = map.nodes[static_cast<std::size_t>(e.from)], pb = map.nodes[static_cast<std::size_t>(e.to)];
        const double dx = pb.x - pa.x, dz = pb.y - pa.y;
        const double len = std::sqrt(dx * dx + dz * dz);
        if (len < 1e-3) continue;
        const double ux = dx / len, uz = dz / len;
        const int n = std::max(1, static_cast<int>(std::lround(len / spacing)));
        // at grade both ends: follow the ground between them; a deck or a ramp: the line between its ends
        const bool atGrade = !e.elevAbsA && !e.elevAbsB;
        const double ya = e.elevAbsA ? e.elevA : ground(pa.x, pa.y) + e.elevA;
        const double yb = e.elevAbsB ? e.elevB : ground(pb.x, pb.y) + e.elevB;
        // the box: the piece of street this site stands for (with a little overlap), past the kerbs to the frontages
        const double along = len / n * 0.5 + 5.0;
        const double across = e.width * 0.5 + rules.side;
        const double hx = std::fabs(ux) * along + std::fabs(uz) * across;
        const double hz = std::fabs(uz) * along + std::fabs(ux) * across;
        for (int k = 0; k < n; ++k) {
            const double t = (k + 0.5) / n;
            const double x = pa.x + dx * t, z = pa.y + dz * t;
            const double road = atGrade ? ground(x, z) + e.elevA + (e.elevB - e.elevA) * t : ya + (yb - ya) * t;
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

void placeStoreyProbeSites(const Vec2& centre, const Vec2 axis[2], const Real half[2], double floorY, double ceilY,
                           double spacing, double height, std::vector<ProbeSite>& out) {
    const double sp = std::max(spacing, 2.0);
    const int nu = std::max(1, static_cast<int>(std::lround(2.0 * half[0] / sp)));
    const int nv = std::max(1, static_cast<int>(std::lround(2.0 * half[1] / sp)));
    const double cu = 2.0 * half[0] / nu, cv = 2.0 * half[1] / nv;   // the cells as laid
    // the box: the turned cell's axis-aligned bounds, half a metre over so neighbours meet
    const double hx = std::fabs(axis[0].x) * cu * 0.5 + std::fabs(axis[1].x) * cv * 0.5 + 0.5;
    const double hz = std::fabs(axis[0].y) * cu * 0.5 + std::fabs(axis[1].y) * cv * 0.5 + 0.5;
    const double y = std::min(floorY + height, ceilY - 0.3);
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const double u = -half[0] + (i + 0.5) * cu, v = -half[1] + (j + 0.5) * cv;
            const double x = centre.x + axis[0].x * u + axis[1].x * v, z = centre.y + axis[0].y * u + axis[1].y * v;
            ProbeSite s;
            s.position = Vec3(x, y, z);
            s.boxMin = Vec3(x - hx, floorY - 0.2, z - hz);
            s.boxMax = Vec3(x + hx, ceilY + 0.2, z + hz);
            s.indoor = true;
            out.push_back(s);
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
