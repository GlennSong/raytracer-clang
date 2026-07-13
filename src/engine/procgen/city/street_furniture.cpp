#include "street_furniture.h"

#include <algorithm>

namespace engine {

StreetFurniturePlan planStreetFurniture(
    const NavGraph& nav, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& p) {
    StreetFurniturePlan out;
    auto gy = [&](Real x, Real z) { return ground ? ground(x, z) : Real(0); };
    // Grade-separated links ride a bridge deck this far up per layer — the same
    // constant CityRenderSystem uses, so poles on a bridge stand on its deck.
    const Real kLayerLift = 5.8;

    // SIGNALS: one per junction-entering link — the exact criterion the sim's
    // SignalController uses, so placed poles and simulated phases agree
    // one-to-one by link index. The pole stands on the near-right curb corner,
    // backed off by the widest crossing road (and a knot-merged junction's
    // spread) so it never lands in a carriageway.
    for (int li = 0; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[li];
        // §10: the unified graph includes the corridor — merges/gores are
        // junctions in the graph but NEVER signalised street corners.
        if (L.klass == RoadClass::Freeway || L.klass == RoadClass::Ramp)
            continue;
        if (!nav.isJunction(L.to)) continue;
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
