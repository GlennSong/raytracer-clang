#include "site_plan.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

Shape2 largest(const std::vector<Shape2>& v) {
    if (v.empty()) return Shape2{};
    std::size_t best = 0;
    for (std::size_t i = 1; i < v.size(); ++i)
        if (area(v[i]) > area(v[best])) best = i;
    return v[best];
}

Real totalArea(const std::vector<Shape2>& v) {
    Real a = 0;
    for (const Shape2& s : v) a += area(s);
    return a;
}

// Take `want` out of `free`, returning EVERY region that was taken and
// shrinking `free` by all of them. Every zone is allocated through this one
// function, which is what makes overlap structurally impossible rather than
// merely unlikely.
//
// Returning all the regions rather than the biggest one matters: a circulation
// zone that a building splits into a front path and a rear driveway is two
// regions, and keeping only the larger silently deletes the path the lot needs
// to work.
std::vector<Shape2> carve(std::vector<Shape2>& free, const std::vector<Shape2>& want) {
    if (want.empty() || free.empty()) return {};
    std::vector<Shape2> got = shapeBool(want, free, BoolOp::Intersect);
    std::vector<Shape2> taken;
    for (Shape2& s : got)
        if (area(s) >= 0.5) taken.push_back(std::move(s));
    if (taken.empty()) return {};
    free = shapeBool(free, taken, BoolOp::Subtract);
    return taken;
}

// A rectangle in the lot's own oriented frame, given fractional extents. Lets
// the allocator say "the rear 40%" without caring how the lot is rotated.
Shape2 frameRect(const OBB2& box, Real u0, Real u1, Real v0, Real v1) {
    auto pt = [&](Real u, Real v) {
        return box.center + box.axis[0] * ((u * 2 - 1) * box.half[0]) +
               box.axis[1] * ((v * 2 - 1) * box.half[1]);
    };
    Poly2 ring = {pt(u0, v0), pt(u1, v0), pt(u1, v1), pt(u0, v1)};
    return shapeFromPoly(ring);
}

// A swept ribbon along a polyline — the path/driveway primitive. The same
// profile sweep the road mesher already does along centrelines, at lot scale.
Shape2 ribbon(const std::vector<Vec2>& pts, Real halfWidth) {
    if (pts.size() < 2) return Shape2{};
    std::vector<Shape2> segs;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2 d = pts[i + 1] - pts[i];
        const Real len = d.length();
        if (len < 1e-6) continue;
        const Vec2 t = d / len;
        const Vec2 n(-t.y, t.x);
        Poly2 quad = {pts[i] + n * halfWidth, pts[i] - n * halfWidth,
                      pts[i + 1] - n * halfWidth, pts[i + 1] + n * halfWidth};
        segs.push_back(shapeFromPoly(quad));
    }
    if (segs.empty()) return Shape2{};
    std::vector<Shape2> acc{segs[0]};
    for (std::size_t i = 1; i < segs.size(); ++i)
        acc = shapeBool(acc, {segs[i]}, BoolOp::Unite);
    return largest(acc);
}

Real distToSeg(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const Real len2 = d.lengthSquared();
    if (len2 < 1e-12) return (p - a).length();
    Real t = dot(p - a, d) / len2;
    t = std::max(Real(0), std::min(Real(1), t));
    return (p - (a + d * t)).length();
}

}  // namespace

const char* zoneName(Zone z) {
    switch (z) {
        case Zone::Building:    return "building";
        case Zone::Frontage:    return "frontage";
        case Zone::Circulation: return "circulation";
        case Zone::Open:        return "open";
        case Zone::Parking:     return "parking";
        case Zone::Service:     return "service";
    }
    return "?";
}

const char* propKindName(PropKind k) {
    switch (k) {
        case PropKind::Tree:    return "tree";
        case PropKind::Shrub:   return "shrub";
        case PropKind::Hedge:   return "hedge";
        case PropKind::Planter: return "planter";
        case PropKind::Bench:   return "bench";
        case PropKind::Bin:     return "bin";
        case PropKind::Lamp:    return "lamp";
        case PropKind::Bollard: return "bollard";
        case PropKind::Car:     return "car";
        case PropKind::Sign:    return "sign";
        case PropKind::Table:   return "table";
    }
    return "?";
}

Real ZoneArea::totalArea() const {
    Real a = 0;
    for (const Shape2& s : parts) a += area(s);
    return a;
}

bool ZoneArea::contains(const Vec2& p) const {
    for (const Shape2& s : parts)
        if (engine::contains(s, p)) return true;
    return false;
}

const ZoneArea* SitePlan::zoneOf(Zone z) const {
    for (const ZoneArea& a : zones)
        if (a.zone == z) return &a;
    return nullptr;
}

Real SitePlan::zoneArea(Zone z) const {
    Real a = 0;
    for (const ZoneArea& s : zones)
        if (s.zone == z) a += s.totalArea();
    return a;
}

Real SitePlan::allocatedArea() const {
    Real a = 0;
    for (const ZoneArea& s : zones) a += s.totalArea();
    return a;
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

SitePlan layoutSite(const Shape2& lot, const LotTags& tags,
                    const LotProgram& program, std::uint32_t seed) {
    SitePlan site;
    site.lot = lot;
    if (lot.outer.size() < 3) return site;
    std::vector<Shape2> free{lot};
    Rng rng(seed ? seed : 1u);

    const Poly2 ring = tessellate(lot.outer, 0.1);
    const OBB2 box = orientedBoundingBox(ring);
    const Vec2 lotCentre = centroid(lot);

    // --- 1. ACCESS: where the lot meets the street ---------------------------
    for (std::size_t i = 0; i < lot.outer.size(); ++i) {
        if (lot.outer.edges[i].tag != EdgeTag::Street) continue;
        const Vec2 a = lot.outer.start(i), b = lot.outer.end(i);
        const Vec2 d = b - a;
        if (d.lengthSquared() < 1e-6) continue;
        AccessPoint ap;
        ap.at = a + d * 0.5;
        ap.inward = normalize(Vec2(-d.y, d.x));      // interior is left of CCW
        ap.edge = static_cast<int>(i);
        site.access.push_back(ap);
    }
    if (site.access.empty()) {
        // No tagged street edge: fall back to the edge nearest the lot's
        // longest side so a lot always has a way in. Landscaping a lot with no
        // access at all would be worse than guessing.
        std::size_t best = 0;
        Real bestLen = 0;
        for (std::size_t i = 0; i < lot.outer.size(); ++i) {
            const Real len = (lot.outer.end(i) - lot.outer.start(i)).length();
            if (len > bestLen) { bestLen = len; best = i; }
        }
        const Vec2 a = lot.outer.start(best), b = lot.outer.end(best);
        AccessPoint ap;
        ap.at = (a + b) * 0.5;
        const Vec2 d = b - a;
        ap.inward = normalize(Vec2(-d.y, d.x));
        ap.edge = static_cast<int>(best);
        site.access.push_back(ap);
    }

    // --- 2. SETBACKS: the buildable envelope ---------------------------------
    // Setbacks are PROGRAM properties, not one global number per district: a
    // villa wants 8 m of front garden, a shopfront wants zero.
    TagOffsets setbacks;
    setbacks.street = -program.frontSetback;
    setbacks.side = -program.sideSetback;
    setbacks.rear = -program.rearSetback;
    setbacks.court = -program.sideSetback;
    setbacks.party = 0;                       // a party wall stands ON the line
    setbacks.none = -program.sideSetback;
    Shape2 buildable = offsetByTag(lot, setbacks);
    if (buildable.outer.size() < 3 || area(buildable) < program.minArea * 0.25) {
        // The setbacks consumed the lot. Fall back to a uniform inset rather
        // than emitting a site with no building at all.
        std::vector<Shape2> shrunk = offsetShape(lot, -program.sideSetback);
        buildable = shrunk.empty() ? lot : largest(shrunk);
    }

    // --- 3. BUILDING ENVELOPE ------------------------------------------------
    // Reserve coverage x lotArea inside the buildable region, BIASED by the
    // program: toward the rear for a garden, toward the street for a shopfront,
    // toward the corner for a tower with a plaza.
    const Real wantArea = std::max(program.minArea,
                                   area(lot) * std::min(Real(0.95), program.coverage));
    Real bias = 0.5;                       // 0 = hard against the street
    switch (program.frontage) {
        case FrontageKind::Direct:    bias = 0.12; break;
        case FrontageKind::Patio:     bias = 0.22; break;
        case FrontageKind::Forecourt: bias = 0.55; break;
        case FrontageKind::Plaza:     bias = 0.58; break;
        case FrontageKind::Garden:    bias = 0.68; break;
        case FrontageKind::Yard:      bias = 0.40; break;
    }
    // Which way is "toward the street" in the box's frame?
    const Vec2 toStreet = normalize(site.access[0].at - lotCentre);
    const int axis = std::fabs(dot(toStreet, box.axis[0])) >
                             std::fabs(dot(toStreet, box.axis[1]))
                         ? 0 : 1;
    const bool streetIsLow = dot(toStreet, box.axis[axis]) < 0;

    Shape2 envelope;
    {
        // Grow a band from the biased position until it holds the wanted area.
        const Real full = area(buildable);
        Real frac = full > 1e-6 ? std::min(Real(1), wantArea / full) : 1;
        frac = std::max(frac, Real(0.05));
        const Real half = frac * 0.5;
        Real c = streetIsLow ? bias : 1 - bias;
        c = std::max(half, std::min(1 - half, c));
        Shape2 band = axis == 0 ? frameRect(box, c - half, c + half, 0, 1)
                                : frameRect(box, 0, 1, c - half, c + half);
        std::vector<Shape2> got = shapeBool({buildable}, {band}, BoolOp::Intersect);
        envelope = got.empty() ? buildable : largest(got);
        // If the band undershot (a concave lot), widen it once.
        if (area(envelope) < wantArea * 0.8 && frac < 0.95) {
            const Real h2 = std::min(Real(0.5), half * 1.6);
            Real c2 = std::max(h2, std::min(1 - h2, c));
            Shape2 wide = axis == 0 ? frameRect(box, c2 - h2, c2 + h2, 0, 1)
                                    : frameRect(box, 0, 1, c2 - h2, c2 + h2);
            std::vector<Shape2> g2 = shapeBool({buildable}, {wide}, BoolOp::Intersect);
            if (!g2.empty() && area(largest(g2)) > area(envelope))
                envelope = largest(g2);
        }
    }
    site.buildEnvelope = envelope;
    {
        std::vector<Shape2> taken = carve(free, {envelope});
        if (!taken.empty()) site.zones.push_back({Zone::Building, taken});
    }

    // The entrance: the point on the envelope nearest the access point, which
    // is where circulation has to arrive.
    if (!site.access.empty() && site.buildEnvelope.outer.size() >= 3) {
        const Poly2 env = tessellate(site.buildEnvelope.outer, 0.15);
        Real best = 1e30;
        for (std::size_t i = 0; i < env.size(); ++i) {
            const Vec2 a = env[i], b = env[(i + 1) % env.size()];
            const Vec2 m = (a + b) * 0.5;
            const Real dd = (m - site.access[0].at).length();
            if (dd < best) { best = dd; site.entrance = m; site.hasEntrance = true; }
        }
    }

    // --- 4. CIRCULATION: every access point reaches the entrance -------------
    if (site.hasEntrance) {
        std::vector<Shape2> paths;
        for (const AccessPoint& ap : site.access) {
            // A dog-leg rather than a straight line, so the path reads as a
            // designed walk instead of a diagonal scar.
            const Vec2 mid(ap.at.x + (site.entrance.x - ap.at.x) * 0.5,
                           ap.at.y + (site.entrance.y - ap.at.y) * 0.35);
            std::vector<Vec2> pts{ap.at, mid, site.entrance};
            Shape2 p = ribbon(pts, 0.75);
            if (p.outer.size() >= 3) paths.push_back(p);
        }
        if (program.parkingStalls > 0) {
            // A driveway to the rear, routed AROUND the building rather than
            // through it. Driving straight at the far side of the lot puts the
            // ribbon through the envelope, and since the envelope is already
            // carved out of `free` the driveway would come back as two
            // disconnected stubs — a driveway that does not reach the parking.
            const AccessPoint& ap = site.access[0];
            const Vec2 n = ap.inward;
            const Vec2 along(-n.y, n.x);
            // How far the envelope reaches to each side of the access point,
            // and how far the lot does: pass down whichever side has room.
            Real envLeft = 0, envRight = 0;
            for (const Edge2& e : site.buildEnvelope.outer.edges) {
                const Real s = dot(e.a - ap.at, along);
                envLeft = std::min(envLeft, s);
                envRight = std::max(envRight, s);
            }
            Real lotLeft = 0, lotRight = 0, lotDeep = 0;
            for (const Edge2& e : lot.outer.edges) {
                const Real s = dot(e.a - ap.at, along);
                lotLeft = std::min(lotLeft, s);
                lotRight = std::max(lotRight, s);
                lotDeep = std::max(lotDeep, dot(e.a - ap.at, n));
            }
            const Real roomRight = lotRight - envRight;
            const Real roomLeft = envLeft - lotLeft;
            const bool goRight = roomRight >= roomLeft;
            const Real room = goRight ? roomRight : roomLeft;
            if (room > 2.4) {
                const Real lane = (goRight ? envRight + room * 0.5
                                           : envLeft - room * 0.5);
                const Vec2 p1 = ap.at + along * lane;
                const Vec2 p2 = p1 + n * (lotDeep * 0.86);
                std::vector<Vec2> pts{ap.at, ap.at + n * 1.5,
                                      p1 + n * 1.5, p2};
                Shape2 dv = ribbon(pts, std::min(Real(1.6), room * 0.42));
                if (dv.outer.size() >= 3) paths.push_back(dv);
            }
        }
        if (!paths.empty()) {
            // Fold the ribbons together PAIRWISE before intersecting with the
            // lot. shapeBool treats each operand as one well-formed region set,
            // so handing it a list whose members overlap each other (the front
            // walk and the driveway share the ground at the gate) is outside
            // its contract — the classification never compares A against A, and
            // one of the two paths silently disappears.
            std::vector<Shape2> acc{paths[0]};
            for (std::size_t i = 1; i < paths.size(); ++i)
                acc = shapeBool(acc, {paths[i]}, BoolOp::Unite);
            std::vector<Shape2> merged = shapeBool(acc, {lot}, BoolOp::Intersect);
            std::vector<Shape2> circ = carve(free, merged);
            if (!circ.empty())
                site.zones.push_back({Zone::Circulation, circ});
        }
    }

    // --- 5. FRONTAGE / PARKING / SERVICE / OPEN fill what remains ------------
    // Frontage: the strip between the street and the building.
    if (program.frontage != FrontageKind::Direct && totalArea(free) > 2) {
        const Real depth = std::max(Real(1.5), program.frontSetback);
        const Vec2 n = site.access[0].inward;
        const Vec2 a = site.access[0].at;
        Poly2 strip = {a - Vec2(-n.y, n.x) * 60 - n * 5,
                       a + Vec2(-n.y, n.x) * 60 - n * 5,
                       a + Vec2(-n.y, n.x) * 60 + n * depth,
                       a - Vec2(-n.y, n.x) * 60 + n * depth};
        std::vector<Shape2> front = carve(free, {shapeFromPoly(strip)});
        if (!front.empty()) site.zones.push_back({Zone::Frontage, front});
    }

    if (program.parkingStalls > 0 && totalArea(free) > 8) {
        // Parking goes to the far side from the street.
        const Real want = std::min(totalArea(free) * 0.6,
                                   program.parkingStalls * 13.0);
        const Real full = totalArea(free);
        const Real frac = full > 1e-6 ? std::min(Real(0.85), want / full) : 0.4;
        Shape2 band = axis == 0
                          ? frameRect(box, streetIsLow ? 1 - frac : 0,
                                      streetIsLow ? 1 : frac, 0, 1)
                          : frameRect(box, 0, 1, streetIsLow ? 1 - frac : 0,
                                      streetIsLow ? 1 : frac);
        std::vector<Shape2> park = carve(free, {band});
        if (!park.empty()) site.zones.push_back({Zone::Parking, park});
    }

    if (program.service != ServiceKind::None && totalArea(free) > 4) {
        // Service takes a small bite at the rear corner.
        const Real s = program.service == ServiceKind::Loading ? 0.30 : 0.16;
        Shape2 corner = frameRect(box, streetIsLow ? 1 - s : 0,
                                  streetIsLow ? 1 : s, 0, s * 1.4);
        std::vector<Shape2> svc = carve(free, {corner});
        if (!svc.empty()) site.zones.push_back({Zone::Service, svc});
    }

    if (program.open != OpenKind::None && totalArea(free) > 1) {
        site.zones.push_back({Zone::Open, free});
        free.clear();
    }
    site.free = free.empty() ? Shape2{} : largest(free);

    // --- 6. BOUNDARIES, and the gates the paths demand -----------------------
    BoundaryKind kind = BoundaryKind::None;
    Real bh = 1.2;
    switch (program.frontage) {
        case FrontageKind::Garden:    kind = BoundaryKind::Hedge; bh = 1.1; break;
        case FrontageKind::Patio:     kind = BoundaryKind::Railing; bh = 0.95; break;
        case FrontageKind::Forecourt: kind = BoundaryKind::Fence; bh = 1.3; break;
        case FrontageKind::Yard:      kind = BoundaryKind::Fence; bh = 2.1; break;
        case FrontageKind::Plaza:
        case FrontageKind::Direct:    kind = BoundaryKind::None; break;
    }
    if (kind != BoundaryKind::None) {
        const ZoneArea* circ = site.zoneOf(Zone::Circulation);
        for (std::size_t i = 0; i < lot.outer.size(); ++i) {
            const EdgeTag tag = lot.outer.edges[i].tag;
            if (tag == EdgeTag::Party) continue;         // a party wall IS the boundary
            const Vec2 a = lot.outer.start(i), b = lot.outer.end(i);
            if ((b - a).lengthSquared() < 1e-6) continue;
            BoundaryRun run;
            run.a = a;
            run.b = b;
            run.kind = kind;
            run.height = tag == EdgeTag::Street ? bh : bh * 0.9;
            run.onEdge = tag;
            site.boundaries.push_back(run);

            // A GATE where circulation crosses this run. Emitted because the
            // path has to get through the fence — geometry demanding an object,
            // not a die roll producing one.
            if (!circ) continue;
            Real bestT = -1, bestDist = 1e30;
            for (const Shape2& part : circ->parts) {
                for (const Vec2& p : tessellate(part.outer, 0.2)) {
                    const Vec2 d = b - a;
                    const Real len2 = d.lengthSquared();
                    Real t = len2 > 1e-12 ? dot(p - a, d) / len2 : 0;
                    if (t < 0.02 || t > 0.98) continue;
                    const Real dist = distToSeg(p, a, b);
                    if (dist < bestDist) { bestDist = dist; bestT = t; }
                }
            }
            if (bestT >= 0 && bestDist < 0.9) {
                GateSpec g;
                g.centre = a + (b - a) * bestT;
                g.axis = normalize(b - a);
                g.width = 1.4;
                g.height = run.height;
                g.style = kind;
                g.hingeLeft = rng.chance(0.5);
                site.gates.push_back(g);
            }
        }
    }
    return site;
}

// ---------------------------------------------------------------------------
// Furnishing
// ---------------------------------------------------------------------------

std::vector<PropInstance> furnishZone(const ZoneArea& zone,
                                      const LotProgram& program,
                                      std::uint32_t seed) {
    std::vector<PropInstance> out;
    if (zone.parts.empty()) return out;
    const Real a = zone.totalArea();
    if (a < 1.5) return out;
    Rng rng(seed ? seed : 7u);

    // What lives in each zone, and how densely.
    struct Choice { PropKind kind; Real radius; Real weight; };
    std::vector<Choice> palette;
    Real density = 0.02;          // props per square metre
    switch (zone.zone) {
        case Zone::Open:
            palette = {{PropKind::Tree, 1.7, 1.0},
                       {PropKind::Shrub, 0.6, 2.4},
                       {PropKind::Planter, 0.6, 0.5}};
            density = program.open == OpenKind::Park ? 0.030 : 0.045;
            break;
        case Zone::Frontage:
            palette = {{PropKind::Shrub, 0.55, 2.0},
                       {PropKind::Planter, 0.6, 1.0},
                       {PropKind::Tree, 1.6, 0.5}};
            density = 0.055;
            break;
        case Zone::Parking:
            palette = {{PropKind::Car, 1.5, 3.0}, {PropKind::Lamp, 0.35, 0.4}};
            density = 0.020;
            break;
        case Zone::Service:
            palette = {{PropKind::Bin, 0.6, 3.0}, {PropKind::Bollard, 0.3, 0.8}};
            density = 0.045;
            break;
        case Zone::Circulation:
            palette = {{PropKind::Lamp, 0.35, 1.0}, {PropKind::Bollard, 0.3, 0.6}};
            density = 0.012;
            break;
        case Zone::Building:
            return out;               // the building furnishes itself
    }
    if (palette.empty()) return out;

    const int target = std::max(1, static_cast<int>(a * density));
    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const Shape2& part : zone.parts) {
        Vec2 l, h;
        bounds(part, l, h);
        lo.x = std::min(lo.x, l.x); lo.y = std::min(lo.y, l.y);
        hi.x = std::max(hi.x, h.x); hi.y = std::max(hi.y, h.y);
    }

    // Poisson-disk dart throwing: each placed prop subtracts its own footprint,
    // so props cannot overlap each other any more than zones can overlap each
    // other. The same invariant as the zone allocator, one level down.
    const int maxDarts = target * 40 + 60;
    for (int dart = 0; dart < maxDarts && static_cast<int>(out.size()) < target;
         ++dart) {
        const Vec2 p(rng.range(lo.x, hi.x), rng.range(lo.y, hi.y));
        if (!zone.contains(p)) continue;
        Real total = 0;
        for (const Choice& c : palette) total += c.weight;
        Real r = rng.unit() * total;
        Choice pick = palette.back();
        for (const Choice& c : palette) {
            r -= c.weight;
            if (r <= 0) { pick = c; break; }
        }
        const Real radius = pick.radius * rng.range(0.85, 1.2);
        // Keep clear of the zone's own boundary, so a tree does not straddle
        // the edge into the neighbouring zone.
        Real edgeDist = 1e30;
        for (const Shape2& part : zone.parts) {
            const Poly2 ring = tessellate(part.outer, 0.25);
            for (std::size_t i = 0; i < ring.size(); ++i)
                edgeDist = std::min(
                    edgeDist, distToSeg(p, ring[i], ring[(i + 1) % ring.size()]));
        }
        if (edgeDist < radius * 0.8) continue;
        bool clash = false;
        for (const PropInstance& q : out)
            if ((q.at - p).length() < q.radius + radius) { clash = true; break; }
        if (clash) continue;
        PropInstance pi;
        pi.kind = pick.kind;
        pi.at = p;
        pi.radius = radius;
        pi.yaw = rng.range(0, kTau);
        pi.scale = rng.range(0.85, 1.15);
        out.push_back(pi);
    }
    return out;
}

SiteFurnishing furnishSite(const SitePlan& site, const LotProgram& program,
                           std::uint32_t seed) {
    SiteFurnishing f;
    f.boundaries = site.boundaries;
    f.gates = site.gates;
    std::uint32_t s = seed ? seed : 13u;
    for (const ZoneArea& z : site.zones) {
        std::vector<PropInstance> p = furnishZone(z, program, s);
        s = s * 1664525u + 1013904223u;
        f.props.insert(f.props.end(), p.begin(), p.end());
    }
    return f;
}

}  // namespace engine
