#include "envelope.h"
#include "implicit.h"
#include "mesh_mass.h"
#include <algorithm>
#include <cmath>

namespace engine {

namespace {
bool samePlanLoose(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if ((a[i] - b[i]).lengthSquared() > 1e-8) return false;
    return true;
}
}  // namespace

uint8_t skinOf(const BuildingParams& params) {
    if (params.skin != 0) return params.skin & 3u;
    return params.envelope == BuildingParams::Envelope::Wave ? kSkinPlates : 0;
}

std::vector<StoreyPlan> formStoreys(const Poly2& plan, const BuildingParams& params) {
    BuildingParams q = params;
    q.skin = 0;
    if (q.envelope == BuildingParams::Envelope::Mesh) return sliceStoreys(meshOf(plan, q), q);
    if (q.envelope == BuildingParams::Envelope::Wave) return sliceStoreys(implicitOf(plan, q), q);
    return sliceStoreys(stackOf(plan, q), q);
}

Poly2 skinFloorsPlan(const Poly2& planIn, const BuildingParams& params) {
    Poly2 none;
    const uint8_t skin = skinOf(params);
    if (skin == 0 || planIn.size() < 3) return none;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const Real gap = (skin & kSkinDouble) ? std::max(Real(0), params.skinGap) : Real(0);
    if (params.envelope == BuildingParams::Envelope::Wave) {
        // the wave runs past the plan: the plates' floors are the plan; a veil's floors sit inside its trough
        if (gap <= 0) return plan;
        const Poly2 inner = offsetPlanPublic(plan, std::max(Real(0), params.wave) + gap);
        return inner.size() >= 3 && tierShortSide(inner) >= kCoreTierMin ? inner : none;
    }
    const std::vector<StoreyPlan> form = formStoreys(plan, params);
    const int f0 = std::max(1, params.baseFloors);
    if (static_cast<int>(form.size()) <= f0 + 1) return none;
    const Vec2 c = centroid(plan);
    auto scaled = [&](Real s) {
        Poly2 p;
        for (const Vec2& q : plan) p.push_back(c + (q - c) * s);
        return p;
    };
    auto fits = [&](Real s) {
        const Poly2 inner = scaled(s);
        const Poly2 outer = gap > 0 ? offsetPlanPublic(inner, -gap) : inner;
        if (outer.size() < 3) return false;
        for (std::size_t k = static_cast<std::size_t>(f0) + 1; k < form.size(); ++k) {
            const Poly2& sl = form[k].plan;
            if (sl.size() < 3) return false;
            const Vec2 sc = centroid(sl);
            for (const Vec2& q : outer)
                if (!pointInPolygon(sl, q + (sc - q) * 0.01)) return false;
        }
        return true;
    };
    Real lo = 0.2, hi = 1.0;
    if (fits(hi)) lo = hi;
    else {
        if (!fits(lo)) return none;
        for (int i = 0; i < 14; ++i) {
            const Real mid = (lo + hi) * 0.5;
            if (fits(mid)) lo = mid; else hi = mid;
        }
    }
    const Poly2 inner = scaled(lo);
    return tierShortSide(inner) >= kCoreTierMin ? inner : none;
}

bool skinFloorsStack(const Poly2& planIn, const BuildingParams& params, Stack& out) {
    const Poly2 inner = skinFloorsPlan(planIn, params);
    if (inner.empty()) return false;
    Poly2 plan = planIn;
    ensureCCW(plan);
    if (samePlanLoose(inner, plan)) return false;   // the floors are the plan: today's path (the Wave's plates)
    BuildingParams q = params;
    q.envelope = BuildingParams::Envelope::None;
    q.skin = 0;
    q.setbackFloors = 0;
    q.setbackEvery = 0;
    Stack st = stackOf(plan, q);
    st.entries.clear();
    const int f0 = std::max(1, params.baseFloors);
    StackEntry base;
    base.section = Section::fromLot(plan);
    base.k0 = 0;
    base.k1 = std::min(f0, params.floors);
    st.entries.push_back(base);
    if (f0 < params.floors) {
        StackEntry tower;
        tower.section = Section::fromLot(inner);
        tower.k0 = f0 + 1;
        tower.k1 = params.floors;
        tower.rel = Relation::Nest;
        st.entries.push_back(tower);
    }
    out = std::move(st);
    return true;
}

std::vector<Poly2> skinSlices(const Poly2& plan, const BuildingParams& params) {
    std::vector<Poly2> out;
    if (!(skinOf(params) & kSkinDouble)) return out;
    if (skinFloorsPlan(plan, params).empty()) return out;
    const std::vector<StoreyPlan> form = formStoreys(plan, params);
    const int f0 = std::max(1, params.baseFloors);
    if (static_cast<int>(form.size()) <= f0 + 1) return out;
    out.resize(form.size());
    std::size_t maxN = 0;
    bool same = true;
    for (std::size_t k = static_cast<std::size_t>(f0) + 1; k < form.size(); ++k) {
        maxN = std::max(maxN, form[k].plan.size());
        if (form[k].plan.size() != form[static_cast<std::size_t>(f0) + 1].plan.size()) same = false;
    }
    const int n = same ? 0 : static_cast<int>(std::max<std::size_t>(32, maxN));
    for (std::size_t k = static_cast<std::size_t>(f0) + 1; k < form.size(); ++k) {
        if (form[k].plan.size() < 3) continue;
        out[k] = n ? resampleLoop(form[k].plan, n) : form[k].plan;
    }
    return out;
}

std::vector<Poly2> skinPlates(const Poly2& plan, const BuildingParams& params) {
    std::vector<Poly2> out;
    if (!(skinOf(params) & kSkinPlates)) return out;
    if (params.envelope == BuildingParams::Envelope::Wave) return wavePlates(plan, params);
    if (skinFloorsPlan(plan, params).empty()) return out;
    const std::vector<StoreyPlan> form = formStoreys(plan, params);
    const int f0 = std::max(1, params.baseFloors);
    out.resize(form.size());
    for (std::size_t k = static_cast<std::size_t>(f0) + 1; k < form.size(); ++k) out[k] = form[k].plan;
    return out;
}

}  // namespace engine
