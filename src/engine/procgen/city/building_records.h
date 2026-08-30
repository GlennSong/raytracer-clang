#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_RECORDS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_RECORDS_H

#include "city_lots.h"   // DoorSpec, BuildingParams, Poly2, pointInPolygon
#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

// The runtime identity of every grown building unit (ADR-0080). Before this,
// a building dissolved at load into ~20 city-wide part meshes; the only
// survivor was CityPlanDebug's prism (debug-only). These records carry what
// the RUNTIME needs: the plan (inside tests), the doors (approach tests,
// leaves, entrance hints), and the verbatim BuildingParams regen key —
// growPlanBuilding/growInterior are pure in (plan, params, baseY), so ~250 B
// here rebuilds a whole interior on demand. Kept out of components.h so
// engine core does not include the grammar; the loader stores one
// CityBuildings component on its own entity.
struct BuildingRecord {
    Poly2 plan;
    Real baseY = 0;    // wall base (pad + plinth)
    Real groundY = 0;  // graded pad plane
    Real height = 0;   // massing height above baseY
    BuildingParams params;
    std::vector<DoorSpec> doors;
    bool enterable = false;
    std::string recipe, type, district;
};

struct CityBuildings {
    std::vector<BuildingRecord> records;

    // 24 m spatial hash over plan bboxes (the RoadDeckField pattern).
    void buildIndex() {
        cells_.clear();
        for (uint32_t i = 0; i < records.size(); ++i) {
            Real x0, x1, z0, z1;
            if (!bbox(records[i], x0, x1, z0, z1)) continue;
            for (int cx = cellOf(x0); cx <= cellOf(x1); ++cx)
                for (int cz = cellOf(z0); cz <= cellOf(z1); ++cz)
                    cells_[key(cx, cz)].push_back(i);
        }
    }

    // The record whose plan contains xz, with y inside its prism band
    // (generously: half a metre under the pad to half a metre over the top).
    const BuildingRecord* recordAt(const Vec2& xz, Real y) const {
        auto it = cells_.find(key(cellOf(xz.x), cellOf(xz.y)));
        if (it == cells_.end()) return nullptr;
        for (uint32_t i : it->second) {
            const BuildingRecord& r = records[i];
            if (y < r.groundY - 0.5 || y > r.baseY + r.height + 0.5) continue;
            if (r.plan.size() >= 3 && pointInPolygon(r.plan, xz)) return &r;
        }
        return nullptr;
    }

    // Every record whose plan bbox dilated by r covers xz (callers refine
    // by door distance / dot products themselves).
    void near(const Vec2& xz, Real r, std::vector<const BuildingRecord*>& out) const {
        out.clear();
        std::vector<uint32_t> idx;
        for (int cx = cellOf(xz.x - r); cx <= cellOf(xz.x + r); ++cx)
            for (int cz = cellOf(xz.y - r); cz <= cellOf(xz.y + r); ++cz) {
                auto it = cells_.find(key(cx, cz));
                if (it == cells_.end()) continue;
                idx.insert(idx.end(), it->second.begin(), it->second.end());
            }
        std::sort(idx.begin(), idx.end());
        idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
        for (uint32_t i : idx) {
            Real x0, x1, z0, z1;
            if (!bbox(records[i], x0, x1, z0, z1)) continue;
            if (xz.x >= x0 - r && xz.x <= x1 + r && xz.y >= z0 - r &&
                xz.y <= z1 + r)
                out.push_back(&records[i]);
        }
    }

  private:
    static constexpr Real kCell = 24.0;
    static int cellOf(Real v) {
        return static_cast<int>(std::floor(v / kCell));
    }
    static int64_t key(int cx, int cz) {
        return (static_cast<int64_t>(cx) << 32) ^
               static_cast<uint32_t>(cz);
    }
    static bool bbox(const BuildingRecord& r, Real& x0, Real& x1, Real& z0,
                     Real& z1) {
        if (r.plan.size() < 3) return false;
        x0 = z0 = 1e300;
        x1 = z1 = -1e300;
        for (const Vec2& v : r.plan) {
            x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
            z0 = std::min(z0, v.y); z1 = std::max(z1, v.y);
        }
        return true;
    }
    std::unordered_map<int64_t, std::vector<uint32_t>> cells_;
};

}  // namespace engine

#endif
