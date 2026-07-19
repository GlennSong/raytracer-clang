#include "street_furniture.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {

StreetFurniturePlan planStreetFurniture(
    const NavGraph& nav, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& p) {
    StreetFurniturePlan out;
    auto gy = [&](Real x, Real z) { return ground ? ground(x, z) : Real(0); };
    // Grade-separated links ride a bridge deck this far up per layer — the same
    // constant CityRenderSystem uses, so poles on a bridge stand on its deck.
    const Real kLayerLift = 5.8;

    // SIGNALS: one per (junction, approach BEARING) — matching the sim's
    // SignalController for real this time. Two mismatches used to break the
    // "poles and phases agree" claim:
    //   - the controller only signalises 4+-approach junctions (T-junctions
    //     stay uncontrolled, by device feedback), but poles were planted at
    //     EVERY junction — a dark, dead head on every T;
    //   - the unified graph can hold OVERLAPPING COLLINEAR edges (a long
    //     edge plus a stub riding it), and per-LINK placement stood two heads
    //     on one arm (device: "double stoplights which is bizarre").
    // So: skip junctions the controller leaves uncontrolled, quantise each
    // approach bearing to a 15-degree bin per node, and keep ONE pole per
    // bin — the widest approach, then the longest, then the lowest link
    // index (deterministic). Phases stay per-LINK in the sim (stateForLink),
    // so a deduped twin still resolves its phase; only its redundant pole is
    // gone. The pole stands on the near-right curb corner, backed off by the
    // widest crossing road (and a knot-merged junction's spread) so it never
    // lands in a carriageway.
    std::map<std::pair<int, int>, int> armBest;   // (node, bearing bin) -> link
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        // Semantic signal predicate (#17/S5), shared with the controller and
        // the render markings so all three agree exactly.
        if (!(L.access & road_access::kSignalable)) continue;
        if (!nav.signalControlled(L.to)) continue;
        Vec2 d = nav.direction(li);
        int bin = static_cast<int>(std::lround(std::atan2(d.y, d.x) /
                                               (PI / 12.0)));
        bin = ((bin % 24) + 24) % 24;
        const auto key = std::make_pair(L.to, bin);
        auto it = armBest.find(key);
        if (it != armBest.end()) {
            const NavLink& W = nav.links[it->second];
            const bool better =
                L.width > W.width ||
                (L.width == W.width &&
                 (L.length > W.length ||
                  (L.length == W.length && li < it->second)));
            if (!better) continue;
            it->second = li;
        } else {
            armBest.emplace(key, li);
        }
    }
    for (const auto& kv : armBest) {
        const int li = kv.second;
        const NavLink& L = nav.links[li];
        Vec2 d = nav.direction(li);
        Vec2 node = nav.nodes[L.to];
        Vec2 right(d.y, -d.x);
        Real thisHalf = L.width * 0.5;
        Real crossHalf = thisHalf;
        for (int ol : nav.outLinks[L.to])
            crossHalf = std::max(crossHalf, nav.links[ol].width * 0.5);
        Real spread = L.to < static_cast<int>(nav.nodeSpread.size())
                          ? nav.nodeSpread[L.to] : Real(0);
        Vec2 corner = node - d * (crossHalf + spread + p.curbGap) +
                      right * (thisHalf + p.curbGap);
        SignalSpot s;
        s.base = Vec3(corner.x, gy(corner.x, corner.y) + L.layer * kLayerLift,
                      corner.y);
        s.face = Vec3(-d.x, 0, -d.y);   // head faces its approaching traffic
        s.link = li;
        out.signals.push_back(s);
    }

    // LAMPS: march each link's RIGHT sidewalk — a two-way road contributes one
    // directed link per direction, so both sides light up and the pattern
    // alternates naturally. Freeway-width carriageways have no sidewalk to
    // stand on; junction mouths are left to the signal poles.
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        if (L.klass == RoadClass::Freeway || L.klass == RoadClass::Ramp)
            continue;   // §10: corridor lighting is its own pass, not lamps
        if (L.width > p.maxLampRoadWidth) continue;
        Vec2 a = nav.nodes[L.from], b = nav.nodes[L.to];
        const Real len = (b - a).length();
        if (len < p.lampSpacing * 0.6) continue;
        const int n = std::max(1, static_cast<int>(std::floor(len / p.lampSpacing)));
        for (int k = 0; k < n; ++k) {
            const Real t = (k + 0.5) / n;
            Vec2 sp = nav.sidewalkPoint(li, t, p.lampVerge);
            if ((sp - nav.nodes[L.to]).length() < p.junctionClear ||
                (sp - nav.nodes[L.from]).length() < p.junctionClear)
                continue;
            const Real y = gy(sp.x, sp.y) + L.layer * kLayerLift;
            out.lampBases.push_back(Vec3(sp.x, y, sp.y));
            out.lampHeads.push_back(Vec3(sp.x, y + p.lampHeight, sp.y));
        }
    }
    return out;
}

}  // namespace engine
