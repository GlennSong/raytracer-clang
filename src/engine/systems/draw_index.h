#ifndef RAYTRACER_ENGINE_DRAW_INDEX_H
#define RAYTRACER_ENGINE_DRAW_INDEX_H

// THE DRAW INDEX (RenderSystem's encode walk; Glenn: "see what kind of performance you can eke out of the system").
// The island visited ~27,000 entities and ~50,000 instance groups every frame to draw ~340 and ~410 of them: 6 ms of
// encode, nearly all of it finding things off screen.
//
// Static drawables -- by their DrawClass: terrain, structure, scenery, furniture, ground paint -- are bucketed into
// square world cells by where they stand; a frame tests each cell's box against the view and visits only the members
// of the visible ones. Everything else (bodies, effects, the unclassed) sits on a short list visited every frame.
//
// It keeps itself right with no help from whoever creates, moves or destroys drawables: each frame re-classifies a
// slice of the pools (new ones, moved ones, reclassed ones); a member whose entity index no longer holds a drawable is
// dropped on its visit. A wholesale change (a level load, a big stream) is re-classified at once. So a static thing
// that moves is in its new cell within a few frames, and nothing is lost. Keyed by ENTITY INDEX: a reused index is
// whatever drawable holds it now, the pools being the truth.

#include "../components.h"
#include "../world.h"
#include "../../renderer/renderer.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

class DrawIndex {
public:
    static constexpr Real kCell = 200.0;          // metres
    static constexpr std::size_t kSlice = 4000;   // re-classified per frame, per pool

    static bool isStatic(DrawClass c) {
        return c == DrawClass::Terrain || c == DrawClass::Structure || c == DrawClass::Scenery ||
               c == DrawClass::Furniture || c == DrawClass::GroundPaint;
    }

    // policy: the level's DrawPolicy (nullptr: every class unlimited) -- each cell remembers the farthest any member
    // of it can be seen from, and is skipped beyond that (a view down an avenue reaches the horizon; scenery does not)
    void refresh(SparseSet<Transform>& tPool, SparseSet<Renderable>& rPool, SparseSet<InstanceGroup>* gPool,
                 const Renderer& renderer, const DrawPolicy* policy) {
        policy_ = policy;
        const std::vector<uint32_t>& ents = rPool.entityIndices();
        if (eCount_ < ents.size() * 9 / 10 || ents.size() < kSlice) {
            for (uint32_t i : ents) classifyEntity(i, tPool, rPool, renderer);
        } else {
            for (std::size_t k = 0; k < kSlice; ++k) {
                if (eCursor_ >= ents.size()) eCursor_ = 0;
                classifyEntity(ents[eCursor_++], tPool, rPool, renderer);
            }
        }
        if (!gPool) return;
        const std::vector<uint32_t>& grps = gPool->entityIndices();
        if (gCount_ < grps.size() * 9 / 10 || grps.size() < kSlice) {
            for (uint32_t i : grps) classifyGroup(i, *gPool);
        } else {
            for (std::size_t k = 0; k < kSlice; ++k) {
                if (gCursor_ >= grps.size()) gCursor_ = 0;
                classifyGroup(grps[gCursor_++], *gPool);
            }
        }
    }

    // fe(index) / fg(index): draw that entity / group; false = nothing drawable holds the index any more (dropped).
    // tEntities: stamped between the two halves (the encode stats).
    // groupDistance: the live vegetation slider (> 0 overrides every group's distance, as the walk does);
    // cellOk(lo, hi, distance): a last say per visible cell (the occlusion test, for cells too far to shadow anything)
    template <typename FE, typename FG, typename FC>
    void visit(const Frustum& frustum, const Vec3& eye, Real groupDistance, FE&& fe, FG&& fg, FC&& cellOk,
               std::chrono::steady_clock::time_point& tEntities) {
        visible_.clear();
        for (int c = 0; c < static_cast<int>(cells_.size()); ++c) {
            const Cell& cell = cells_[static_cast<std::size_t>(c)];
            if ((cell.ents.empty() && cell.groups.empty()) || !frustum.containsAABB(cell.lo, cell.hi)) continue;
            // beyond the farthest any member draws: none of them would (the walk's own distance tests, cell-wide)
            const Real reach = std::max(cell.unlimited ? Real(1e30) : cell.reach, groupDistance > 0 && !cell.groups.empty() ? groupDistance : Real(0));
            const Real dx = std::max({cell.lo.x - eye.x, Real(0), eye.x - cell.hi.x});
            const Real dy = std::max({cell.lo.y - eye.y, Real(0), eye.y - cell.hi.y});
            const Real dz = std::max({cell.lo.z - eye.z, Real(0), eye.z - cell.hi.z});
            const Real d2 = dx * dx + dy * dy + dz * dz;
            if (reach < 1e29 && d2 > reach * reach) continue;
            if (!cellOk(cell.lo, cell.hi, std::sqrt(d2))) continue;
            visible_.push_back(c);
        }
        drop_.clear();
        for (int c : visible_)
            for (uint32_t i : cells_[static_cast<std::size_t>(c)].ents)
                if (!fe(i)) drop_.push_back(i);
        for (uint32_t i : dynEnts_)
            if (!fe(i)) drop_.push_back(i);
        for (uint32_t i : drop_) removeFrom(eSlot_, i, true);
        tEntities = std::chrono::steady_clock::now();
        drop_.clear();
        for (int c : visible_)
            for (uint32_t i : cells_[static_cast<std::size_t>(c)].groups)
                if (!fg(i)) drop_.push_back(i);
        for (uint32_t i : dynGroups_)
            if (!fg(i)) drop_.push_back(i);
        for (uint32_t i : drop_) removeFrom(gSlot_, i, false);
    }

    std::size_t cellCount() const { return cells_.size(); }
    std::size_t visibleCells() const { return visible_.size(); }

private:
    struct Cell {
        std::vector<uint32_t> ents, groups;
        Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};   // grows with what is put in it
        Real reach = 0;          // the farthest any member draws from (grows)
        bool unlimited = false;  // ...or one draws at any distance
    };
    // slack: a member measured from elsewhere than its own box (a lodCell centre) can be that much nearer
    void note(Cell& c, Real explicitDist, DrawClass cls, Real slack = 0) {
        const Real d = explicitDist > 0 ? explicitDist : (policy_ ? policy_->distanceFor(cls) : Real(0));
        if (d <= 0) c.unlimited = true;
        else c.reach = std::max(c.reach, d + slack);
    }
    struct Slot {
        int32_t where = -2;   // -2 nowhere, -1 the dynamic list, else a cell
        int32_t pos = -1;
    };

    int cellFor(const Vec3& p) {
        const int64_t cx = static_cast<int64_t>(std::floor(p.x / kCell)), cz = static_cast<int64_t>(std::floor(p.z / kCell));
        const int64_t key = (cx << 32) ^ (cz & 0xffffffff);
        auto it = cellOf_.find(key);
        if (it != cellOf_.end()) return it->second;
        cells_.emplace_back();
        cellOf_.emplace(key, static_cast<int>(cells_.size()) - 1);
        return static_cast<int>(cells_.size()) - 1;
    }
    std::vector<uint32_t>& listOf(int where, bool entity) {
        if (where == -1) return entity ? dynEnts_ : dynGroups_;
        Cell& c = cells_[static_cast<std::size_t>(where)];
        return entity ? c.ents : c.groups;
    }
    void removeFrom(std::vector<Slot>& slots, uint32_t i, bool entity) {
        if (i >= slots.size() || slots[i].where == -2) return;
        std::vector<uint32_t>& v = listOf(slots[i].where, entity);
        const std::size_t pos = static_cast<std::size_t>(slots[i].pos);
        const uint32_t moved = v.back();
        v[pos] = moved;
        v.pop_back();
        if (moved != i) slots[moved].pos = static_cast<int32_t>(pos);
        slots[i] = Slot{};
        --(entity ? eCount_ : gCount_);
    }
    void placeIn(std::vector<Slot>& slots, uint32_t i, int where, bool entity) {
        if (i >= slots.size()) slots.resize(i + 1);
        if (slots[i].where == where) return;
        removeFrom(slots, i, entity);
        std::vector<uint32_t>& v = listOf(where, entity);
        slots[i].where = where;
        slots[i].pos = static_cast<int32_t>(v.size());
        v.push_back(i);
        ++(entity ? eCount_ : gCount_);
    }
    static void grow(Cell& c, const Vec3& lo, const Vec3& hi) {
        c.lo = Vec3(std::min(c.lo.x, lo.x), std::min(c.lo.y, lo.y), std::min(c.lo.z, lo.z));
        c.hi = Vec3(std::max(c.hi.x, hi.x), std::max(c.hi.y, hi.y), std::max(c.hi.z, hi.z));
    }

    void classifyEntity(uint32_t i, SparseSet<Transform>& tPool, SparseSet<Renderable>& rPool, const Renderer& renderer) {
        const Transform* t = tPool.get(i);
        const Renderable* r = rPool.get(i);
        if (!t || !r) { removeFrom(eSlot_, i, true); return; }
        if (!isStatic(r->drawClass)) { placeIn(eSlot_, i, -1, true); return; }
        const BoundingSphere b = renderer.getMeshBounds(r->mesh);
        Vec3 lo, hi;
        transformedAABB(t->matrix(), b.boxMin, b.boxMax, lo, hi);
        const int c = cellFor((lo + hi) * 0.5);
        grow(cells_[static_cast<std::size_t>(c)], lo, hi);
        note(cells_[static_cast<std::size_t>(c)], r->drawDistance, r->drawClass, r->lodCell);
        placeIn(eSlot_, i, c, true);
    }
    void classifyGroup(uint32_t i, SparseSet<InstanceGroup>& gPool) {
        const InstanceGroup* g = gPool.get(i);
        if (!g) { removeFrom(gSlot_, i, false); return; }
        if (!isStatic(g->drawClass)) { placeIn(gSlot_, i, -1, false); return; }
        const Vec3 r(g->boundsRadius, g->boundsRadius, g->boundsRadius);
        const int c = cellFor(g->boundsCenter);
        grow(cells_[static_cast<std::size_t>(c)], g->boundsCenter - r, g->boundsCenter + r);
        note(cells_[static_cast<std::size_t>(c)], g->drawDistance, g->drawClass);
        placeIn(gSlot_, i, c, false);
    }

    std::unordered_map<int64_t, int> cellOf_;
    std::vector<Cell> cells_;
    std::vector<Slot> eSlot_, gSlot_;
    std::vector<uint32_t> dynEnts_, dynGroups_, drop_;
    std::vector<int> visible_;
    std::size_t eCursor_ = 0, gCursor_ = 0, eCount_ = 0, gCount_ = 0;
    const DrawPolicy* policy_ = nullptr;
};

}  // namespace engine

#endif
