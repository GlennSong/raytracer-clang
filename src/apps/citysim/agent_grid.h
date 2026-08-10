#ifndef RAYTRACER_APPS_CITYSIM_AGENT_GRID_H
#define RAYTRACER_APPS_CITYSIM_AGENT_GRID_H

// P4.1 (three-tier traffic): a uniform spatial grid over the city's XZ plane.
// The sim's neighbour senses, the physics bridge's possess ring, and the V/K
// tier bubble all used to LINEAR-SCAN every agent; this grid turns each of
// those into a few-bucket lookup. It deliberately stays a dumb candidate
// filter: query() returns a SUPERSET of the ids inside the circle (everything
// hashed into the cells the circle's bounding box touches), and every caller
// re-applies its exact predicates — so a grid-backed scan produces bit-for-bit
// the same result as the linear scan it replaced, just over fewer candidates.
//
// Determinism (ADR-0002): query() output is sorted ascending, so consumers
// iterate candidates in agent-index order — the same order the linear scans
// used — no matter what history the buckets carry. Bucket-internal order is
// therefore free to be swap-erase churned. place() early-outs when the id's
// cell is unchanged, which is what makes "V agents re-hash only on their
// coarse tick" free: their position only moves when their tick runs.
//
// Header-only, standard library only, no ECS/render/physics dependencies —
// like agent_id.h it drops into every headless test build.

#include "../../engine/procgen/city/polygon.h"   // engine::Vec2, engine::Real

#include <algorithm>
#include <cmath>
#include <vector>

namespace citysim {

class AgentGrid {
public:
    // Cover [lo, hi] with square cells of `cellSize` metres and make room for
    // ids 0..count-1. Positions outside the bounds clamp into the border cells
    // (correct, just coarser there), so the margin only needs to cover where
    // bodies usually live. Reconfiguring drops all placements.
    void configure(engine::Vec2 lo, engine::Vec2 hi, engine::Real cellSize,
                   int count) {
        lo_ = lo;
        cell_ = cellSize > 1.0 ? cellSize : 1.0;
        nx_ = std::max(1, static_cast<int>(std::ceil((hi.x - lo.x) / cell_)));
        ny_ = std::max(1, static_cast<int>(std::ceil((hi.y - lo.y) / cell_)));
        buckets_.assign(static_cast<std::size_t>(nx_) * ny_, {});
        cellOf_.assign(count > 0 ? count : 0, -1);
    }
    bool configured() const { return !buckets_.empty(); }

    // Insert or move id to the cell holding `pos`. O(1) when the cell is
    // unchanged (the common case), O(bucket) on a move.
    void place(int id, engine::Vec2 pos) {
        if (id < 0 || id >= static_cast<int>(cellOf_.size())) return;
        const int c = cellAt(pos);
        int& cur = cellOf_[id];
        if (cur == c) return;
        if (cur >= 0) {
            std::vector<int>& b = buckets_[cur];
            for (std::size_t k = 0; k < b.size(); ++k)
                if (b[k] == id) {
                    b[k] = b.back();
                    b.pop_back();
                    break;
                }
        }
        buckets_[c].push_back(id);
        cur = c;
    }

    // All ids hashed into cells the circle's bounding box touches — a SUPERSET
    // of the ids within `radius` of `pos`, sorted ascending. Callers apply
    // their own exact distance/state predicates, exactly as their old linear
    // scans did.
    void query(engine::Vec2 pos, engine::Real radius,
               std::vector<int>& out) const {
        out.clear();
        if (buckets_.empty()) return;
        const int cx0 = clampX(gridX(pos.x - radius));
        const int cx1 = clampX(gridX(pos.x + radius));
        const int cy0 = clampY(gridY(pos.y - radius));
        const int cy1 = clampY(gridY(pos.y + radius));
        for (int cy = cy0; cy <= cy1; ++cy)
            for (int cx = cx0; cx <= cx1; ++cx) {
                const std::vector<int>& b =
                    buckets_[static_cast<std::size_t>(cy) * nx_ + cx];
                out.insert(out.end(), b.begin(), b.end());
            }
        std::sort(out.begin(), out.end());
    }

private:
    int gridX(engine::Real x) const {
        return static_cast<int>(std::floor((x - lo_.x) / cell_));
    }
    int gridY(engine::Real y) const {
        return static_cast<int>(std::floor((y - lo_.y) / cell_));
    }
    int clampX(int cx) const { return cx < 0 ? 0 : (cx >= nx_ ? nx_ - 1 : cx); }
    int clampY(int cy) const { return cy < 0 ? 0 : (cy >= ny_ ? ny_ - 1 : cy); }
    int cellAt(engine::Vec2 pos) const {
        return clampY(gridY(pos.y)) * nx_ + clampX(gridX(pos.x));
    }

    engine::Vec2 lo_;
    engine::Real cell_ = 64.0;
    int nx_ = 0, ny_ = 0;
    std::vector<std::vector<int>> buckets_;
    std::vector<int> cellOf_;   // per id: current cell, -1 = not placed
};

}  // namespace citysim

#endif
