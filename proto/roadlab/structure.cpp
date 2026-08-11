#include "structure.h"

namespace roadlab {

StructureSpan bridgeSpan(double s0, double s1, double pierSpacing, double clearance) {
    StructureSpan sp;
    sp.kind = (s1 - s0) > 120.0 ? CarrierKind::Viaduct : CarrierKind::Bridge;
    sp.s0 = s0;
    sp.s1 = s1;
    sp.pierSpacing = pierSpacing;
    sp.clearanceRequired = clearance;
    return sp;
}

StructureSpan tunnelSpan(double s0, double s1, double headroom) {
    StructureSpan sp;
    sp.kind = CarrierKind::Tunnel;
    sp.s0 = s0;
    sp.s1 = s1;
    sp.boreClearance = headroom;
    sp.generatePiers = false;
    return sp;
}

StructureSpan embankmentSpan(double s0, double s1) {
    StructureSpan sp;
    sp.kind = CarrierKind::Embankment;
    sp.s0 = s0;
    sp.s1 = s1;
    sp.generatePiers = false;
    return sp;
}

StructureSpan cutSpan(double s0, double s1) {
    StructureSpan sp;
    sp.kind = CarrierKind::Cut;
    sp.s0 = s0;
    sp.s1 = s1;
    sp.generatePiers = false;
    return sp;
}

CarrierKind carrierAt(const Road& r, double s) {
    for (const StructureSpan& sp : r.structures)
        if (sp.contains(s)) return sp.kind;
    return CarrierKind::AtGrade;
}

namespace {

const Vec3 kConcrete{0.36, 0.355, 0.34};
const Vec3 kConcreteDark{0.24, 0.24, 0.235};
const Vec3 kSteel{0.30, 0.31, 0.33};

// A jersey-barrier section, in (t, h), for a barrier strip of the given width
// standing at tCentre. Positive `facing` puts the sloped face toward +t.
std::vector<Vec2> jerseyProfile(double tCentre, double width, double height, double facing) {
    double hw = width * 0.5;
    double f = facing >= 0 ? 1.0 : -1.0;
    return {
        {tCentre - f * hw, 0.0},
        {tCentre - f * hw * 0.55, height * 0.28},
        {tCentre - f * hw * 0.18, height},
        {tCentre + f * hw * 0.35, height},
        {tCentre + f * hw * 0.72, height * 0.28},
        {tCentre + f * hw, 0.0},
    };
}

// Bridge parapet: a slab wall with a slight batter.
std::vector<Vec2> parapetProfile(double tEdge, double height, double inward) {
    double f = inward >= 0 ? 1.0 : -1.0;
    return {
        {tEdge, 0.0},
        {tEdge, height},
        {tEdge + f * 0.28, height},
        {tEdge + f * 0.34, height * 0.35},
        {tEdge + f * 0.34, 0.0},
    };
}

}  // namespace

void tessellateStructures(const Network& net, const Road& road, Mesh& out) {
    if (road.kind == RoadKind::Connector) return;

    // --- barriers the cross-section itself asks for -----------------------
    // Any Barrier strip, anywhere, becomes a swept profile. That covers freeway
    // medians, ramp edges and bridge parapets with one rule.
    for (size_t si = 0; si < road.xs.sections.size(); ++si) {
        const LaneSection& sec = road.xs.sections[si];
        double s0 = std::max(sec.s0, road.begin());
        double s1 = std::min(sec.s0 + sec.length, road.end());
        if (s1 - s0 < 0.5) continue;
        for (int side = 0; side < 2; ++side) {
            const std::vector<Strip>& stack = side == 0 ? sec.right : sec.left;
            for (const Strip& st : stack) {
                if (st.kind != StripKind::Barrier) continue;
                double mid = 0.5 * (s0 + s1);
                double a = 0, b = 0;
                if (!road.xs.laneSpanAt(int(si), st.id, mid, a, b)) continue;
                double centre = 0.5 * (a + b);
                double width = std::max(0.25, std::fabs(b - a));
                if (width < 0.05) continue;
                // Face the carriageway: the sloped side looks toward the
                // reference line, which is where the traffic is.
                double facing = centre > 0 ? -1.0 : 1.0;
                sweepProfile(road, s0, s1,
                             jerseyProfile(centre, width, std::max(0.5, double(st.height)), facing),
                             kConcrete, out, 3.0, true);
            }
        }
    }

    for (const StructureSpan& sp : road.structures) {
        double s0 = std::max(sp.s0, road.begin());
        double s1 = std::min(sp.s1, road.end());
        if (s1 - s0 < 1.0) continue;
        double mid = 0.5 * (s0 + s1);
        double tL = road.xs.leftExtentAt(mid);
        double tR = road.xs.rightExtentAt(mid);

        switch (sp.kind) {
            case CarrierKind::Bridge:
            case CarrierKind::Viaduct: {
                // Deck: the soffit box under the carriageway. Swept in the road
                // frame so it banks with the superelevation.
                std::vector<Vec2> deck = {
                    {tL + 0.35, 0.02},
                    {tL + 0.35, -sp.deckThickness},
                    {tR - 0.35, -sp.deckThickness},
                    {tR - 0.35, 0.02},
                };
                sweepProfile(road, s0, s1, deck, kConcrete, out, 4.0, true);

                // Parapets, unless the cross-section already carries barriers at
                // the edges (freeway decks do; a street bridge does not).
                bool haveEdgeBarrier = false;
                const LaneSection& sec = road.xs.sectionAt(mid);
                for (const std::vector<Strip>* stack : {&sec.left, &sec.right})
                    if (!stack->empty() && stack->back().kind == StripKind::Barrier)
                        haveEdgeBarrier = true;
                if (!haveEdgeBarrier) {
                    sweepProfile(road, s0, s1, parapetProfile(tL + 0.35, sp.parapetHeight, -1.0),
                                 kConcrete, out, 3.0, true);
                    sweepProfile(road, s0, s1, parapetProfile(tR - 0.35, sp.parapetHeight, +1.0),
                                 kConcrete, out, 3.0, true);
                }

                if (!sp.generatePiers) break;
                // Piers on a regular rhythm, with abutment walls at the ends.
                // Spacing is snapped so the spans come out equal rather than
                // leaving a stub bay at the far end.
                int bays = std::max(1, int(std::round((s1 - s0) / sp.pierSpacing)));
                double spacing = (s1 - s0) / double(bays);
                for (int i = 0; i <= bays; ++i) {
                    double s = s0 + spacing * double(i);
                    Frame f = road.spine.frameAt(s);
                    double deckY = road.surfacePoint(s, 0).y - sp.deckThickness;
                    bool abutment = (i == 0 || i == bays);
                    double groundY = terrainBaseHeight(TerrainParams{}, f.planPos.x, f.planPos.y);
                    // Never build a pier that would stand in another road; the
                    // clearance lint reports the ones that cannot be avoided.
                    RoadHit hit;
                    if (!abutment && net.sample(f.planPos, hit, 0.0) && hit.road != road.id)
                        continue;
                    double h = deckY - groundY;
                    if (h < 0.5) continue;
                    if (abutment) {
                        Vec2 nrm = perpLeft(dirOf(f.heading));
                        Vec3 centre = worldOf(f.planPos + nrm * (0.5 * (tL + tR)),
                                              groundY + h * 0.5);
                        emitBox(out, centre, {1.1, h * 0.5, std::fabs(tL - tR) * 0.5 + 0.3},
                                f.heading, kConcreteDark, 0.9);
                    } else {
                        // A pier per carriageway half reads better than one wide
                        // wall and is what a real viaduct does.
                        for (double frac : {0.42, -0.42}) {
                            Vec2 nrm = perpLeft(dirOf(f.heading));
                            double toff = 0.5 * (tL + tR) + frac * std::fabs(tL - tR);
                            Vec2 p = f.planPos + nrm * toff;
                            double gy = terrainBaseHeight(TerrainParams{}, p.x, p.y);
                            emitCylinder(out, worldOf(p, gy), 0.75, deckY - gy - 0.4, 12,
                                         kConcreteDark, 0.85);
                            emitBox(out, worldOf(p, deckY - 0.2), {1.0, 0.2, 1.0}, f.heading,
                                    kConcrete, 0.85);
                        }
                    }
                }
                break;
            }
            case CarrierKind::Tunnel: {
                // The bore: an arch profile swept along the road, plus a portal
                // head wall at each end. Rendered as geometry so a camera at the
                // portal sees into a real hole rather than at a decal.
                std::vector<Vec2> arch;
                double half = std::max(std::fabs(tL), std::fabs(tR)) + 0.6;
                double springing = 1.8;
                arch.push_back({tR - 0.6, 0.0});
                arch.push_back({tR - 0.6, springing});
                const int kSegs = 10;
                for (int i = 0; i <= kSegs; ++i) {
                    double a = kPi * double(i) / double(kSegs);
                    arch.push_back({-half * std::cos(a) + 0.5 * (tL + tR),
                                    springing + (sp.boreClearance - springing) * std::sin(a)});
                }
                arch.push_back({tL + 0.6, springing});
                arch.push_back({tL + 0.6, 0.0});
                sweepProfile(road, s0, s1, arch, kConcreteDark, out, 4.0, false);
                for (double s : {s0, s1}) {
                    Frame f = road.spine.frameAt(s);
                    Vec3 centre = road.surfacePoint(s, 0.5 * (tL + tR)) +
                                  Vec3{0, sp.boreClearance * 0.5 + 1.2, 0};
                    emitBox(out, centre, {0.9, sp.boreClearance * 0.5 + 1.2, half + 1.4},
                            f.heading, kConcrete, 0.9);
                }
                break;
            }
            case CarrierKind::AtGrade:
            case CarrierKind::Embankment:
            case CarrierKind::Cut:
                // Handled by the terrain conform, not by built geometry.
                break;
        }
    }
}

// --- terrain --------------------------------------------------------------

double terrainBaseHeight(const TerrainParams& p, double x, double z) {
    if (!p.enabled) return p.base;
    double n = fbm(x * p.frequency + double(p.seed), z * p.frequency - double(p.seed), 5);
    double ridge = 0.35 * fbm(x * p.frequency * 3.1, z * p.frequency * 3.1, 3);
    return p.base + p.amplitude * (n - 0.5) * 2.0 + p.amplitude * 0.4 * (ridge - 0.5);
}

namespace {

// Signed-ish distance to a polygon: negative inside, positive outside.
double polygonDistance(const std::vector<Vec2>& poly, Vec2 q) {
    if (poly.size() < 3) return 1e9;
    double best = 1e9;
    bool inside = false;
    for (size_t i = 0, k = poly.size() - 1; i < poly.size(); k = i++) {
        Vec2 a = poly[k], b = poly[i];
        Vec2 ab = b - a, aq = q - a;
        double h = clampd(dot(aq, ab) / std::max(1e-9, dot(ab, ab)), 0.0, 1.0);
        best = std::min(best, length(aq - ab * h));
        if (((a.y > q.y) != (b.y > q.y)) &&
            (q.x < (b.x - a.x) * (q.y - a.y) / (b.y - a.y) + a.x))
            inside = !inside;
    }
    return inside ? -best : best;
}

}  // namespace

double terrainHeightAt(const Network& net, const TerrainParams& p, double x, double z) {
    double base = terrainBaseHeight(p, x, z);
    if (!p.enabled) return base;
    Vec2 plan{x, z};
    double bestWeight = 0.0, hsum = 0.0, wsum = 0.0;

    // Junction pads conform too. They are road surface as much as a carriageway
    // is; leaving them out left a cliff of unconformed ground around every
    // intersection, which read as a black wedge in the render.
    for (const Junction& j : net.junctions()) {
        if (j.boundary.size() < 3) continue;
        // Bounding-box reject first. polygonDistance walks every edge, and this
        // runs per terrain vertex per junction — without the cheap test in front
        // of it, ground generation is the slowest thing in the whole system.
        Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
        for (Vec2 q : j.boundary) {
            lo.x = std::min(lo.x, q.x);
            lo.y = std::min(lo.y, q.y);
            hi.x = std::max(hi.x, q.x);
            hi.y = std::max(hi.y, q.y);
        }
        if (x < lo.x - p.slopeWidth || x > hi.x + p.slopeWidth || z < lo.y - p.slopeWidth ||
            z > hi.y + p.slopeWidth)
            continue;
        double d = polygonDistance(j.boundary, plan);
        if (d > p.slopeWidth) continue;
        double w = d <= 0 ? 1.0 : 1.0 - smoothstepd(0.0, p.slopeWidth, d);
        // Weighted accumulation, not a winner-takes-all pick. Choosing the single
        // strongest influence makes the height field jump wherever the winner
        // changes, and a jump in a height field is a cliff — which is exactly
        // what turned the ground around every junction into black wedges.
        wsum += w * w;
        hsum += w * w * junctionElevationAt(net, j, plan);
        bestWeight = std::max(bestWeight, w);
    }
    for (int rid : net.roadsNear(plan)) {
        const Road& r = net.road(rid);
        if (r.kind == RoadKind::Connector) continue;
        Vec2 lo, hi;
        r.spine.planBounds(lo, hi);
        double pad = 60.0;
        if (x < lo.x - pad || x > hi.x + pad || z < lo.y - pad || z > hi.y + pad) continue;
        double s = 0, t = 0;
        if (!r.spine.toST(plan, s, t)) continue;
        if (s < r.begin() - 1e-6 || s > r.end() + 1e-6) continue;
        // A bridge or a tunnel deliberately does not move the ground. That one
        // branch is the entire difference between an overpass and a causeway.
        CarrierKind carrier = carrierAt(r, s);
        if (carrier == CarrierKind::Bridge || carrier == CarrierKind::Viaduct ||
            carrier == CarrierKind::Tunnel)
            continue;
        double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
        double edge = t > 0 ? le : re;
        double over = std::fabs(t) - std::fabs(edge);
        if (over > p.slopeWidth) continue;
        double tc = clampd(t, re, le);
        double roadY = r.surfacePoint(s, tc).y;
        double w = over <= 0 ? 1.0 : 1.0 - smoothstepd(0.0, p.slopeWidth, over);
        wsum += w * w;
        hsum += w * w * roadY;
        bestWeight = std::max(bestWeight, w);
    }
    if (wsum <= 1e-9) return base;
    return lerp(base, hsum / wsum, bestWeight);
}

void tessellateTerrain(const Network& net, const TerrainParams& p, Vec2 lo, Vec2 hi, double cell,
                       Mesh& out) {
    int nx = std::max(2, int((hi.x - lo.x) / cell) + 1);
    int nz = std::max(2, int((hi.y - lo.y) / cell) + 1);
    nx = std::min(nx, 900);
    nz = std::min(nz, 900);
    double dx = (hi.x - lo.x) / double(nx - 1);
    double dz = (hi.y - lo.y) / double(nz - 1);

    std::vector<double> height(size_t(nx) * size_t(nz));
    std::vector<char> onRoad(size_t(nx) * size_t(nz), 0);
    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            double x = lo.x + dx * i, z = lo.y + dz * j;
            height[size_t(j) * size_t(nx) + size_t(i)] = terrainHeightAt(net, p, x, z);
            RoadHit hit;
            bool covered = net.sample({x, z}, hit, 0.0);
            for (const Junction& jn : net.junctions()) {
                if (covered) break;
                if (jn.boundary.size() >= 3 && polygonDistance(jn.boundary, {x, z}) < -0.4)
                    covered = true;
            }
            onRoad[size_t(j) * size_t(nx) + size_t(i)] = covered ? char(1) : char(0);
        }
    }

    std::vector<uint32_t> vid(size_t(nx) * size_t(nz), 0xFFFFFFFFu);
    auto vertexAt = [&](int i, int j) -> uint32_t {
        size_t k = size_t(j) * size_t(nx) + size_t(i);
        if (vid[k] != 0xFFFFFFFFu) return vid[k];
        double x = lo.x + dx * i, z = lo.y + dz * j;
        double h = height[k];
        // The gradient comes from the neighbours already in the grid. Re-sampling
        // the height field four more times per vertex would be five times the
        // work for an answer the grid already contains.
        int im = std::max(0, i - 1), ip = std::min(nx - 1, i + 1);
        int jm = std::max(0, j - 1), jp = std::min(nz - 1, j + 1);
        double hx = (height[size_t(j) * size_t(nx) + size_t(ip)] -
                     height[size_t(j) * size_t(nx) + size_t(im)]) /
                    std::max(1e-6, double(ip - im) * dx) * 1.5;
        double hz = (height[size_t(jp) * size_t(nx) + size_t(i)] -
                     height[size_t(jm) * size_t(nx) + size_t(i)]) /
                    std::max(1e-6, double(jp - jm) * dz) * 1.5;
        Vertex v;
        v.pos = {x, h, z};
        // Clamp the shading slope. A batter steeper than 45 degrees is an
        // authoring problem, and letting its normal go horizontal renders it as a
        // black hole rather than as the steep bank it is.
        double gx = clampd(hx / 1.5, -1.0, 1.0);
        double gz = clampd(hz / 1.5, -1.0, 1.0);
        v.normal = normalize(Vec3{-gx, 1.0, -gz});
        double g = fbm(x * 0.06, z * 0.06, 4);
        double rock = smoothstepd(0.55, 0.9, 1.0 - v.normal.y);
        v.color = lerp(Vec3{0.055 + 0.05 * g, 0.10 + 0.075 * g, 0.035 + 0.025 * g},
                       Vec3{0.20, 0.19, 0.17}, rock);
        v.roughness = 0.95;
        v.material = MatKind::Flat;
        vid[k] = out.push(v);
        return vid[k];
    };

    for (int j = 0; j + 1 < nz; ++j) {
        for (int i = 0; i + 1 < nx; ++i) {
            size_t k = size_t(j) * size_t(nx) + size_t(i);
            // A cell entirely under a carriageway is dropped: the road surface is
            // already there, and two surfaces at the same height is how you get a
            // shimmering mess.
            if (onRoad[k] && onRoad[k + 1] && onRoad[k + size_t(nx)] &&
                onRoad[k + size_t(nx) + 1])
                continue;
            uint32_t a = vertexAt(i, j), b = vertexAt(i + 1, j);
            uint32_t c = vertexAt(i + 1, j + 1), d = vertexAt(i, j + 1);
            out.quad(a, b, c, d);
        }
    }
}

}  // namespace roadlab
