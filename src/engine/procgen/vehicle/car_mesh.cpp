#include "car_mesh.h"

#include "car_interior.h"
#include "../../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Which run of the chamfered profile an edge belongs to. The run is what decides
// whether a cell can be a window: windshields and backlights live on the TOP run
// (the surface facing up-and-forward), side glass on the SIDE runs, and the
// BOTTOM run is the floor pan, which is never cut.
enum class Run : std::uint8_t { Side, Top, Bottom, Chamfer };

struct ProfilePt {
    Real u = 0;   // half-width fraction, [-1, 1]
    Real v = 0;   // height fraction within the ring, [-1, 1]; used when anchor < 0
    Run run = Run::Side;
    // Side points take their height from a per-station ANCHOR instead of a fixed
    // v. Stations go at the silhouette breaks; anchors are the same idea applied
    // to the vertical axis. Sampling the sides uniformly instead puts the belt
    // line mid-cell, and then the daylight opening can only ever be one cell
    // tall — which is exactly what a uniform profile produced.
    int anchor = -1;
};

// Per-station side anchors, bottom to top.
// Two EXTRA levels between the sill and the belt. The wheel arch lives entirely
// in that band, and with only sill+lowerMid it was one cell tall and one cell
// wide — a single rectangle, which is why a round wheel sat in a square hole.
// A curve needs cells to step through; these are them.
enum SideAnchor { kAnchorSill = 0, kAnchorArchLow, kAnchorLowerMid, kAnchorBelt,
                  kAnchorGlassMid, kAnchorRail, kAnchorRoof, kSideAnchorCount };

// Piecewise smoothstep through keys ordered NOSE FIRST (descending z).
Real keyEval(const std::vector<CarKey>& keys, Real z) {
    if (keys.empty()) return 0;
    if (z >= keys.front().z) return keys.front().v;
    if (z <= keys.back().z) return keys.back().v;
    for (std::size_t i = 0; i + 1 < keys.size(); ++i) {
        if (z <= keys[i].z && z >= keys[i + 1].z) {
            const Real span = keys[i].z - keys[i + 1].z;
            if (span <= Real(1e-9)) return keys[i].v;
            Real t = (keys[i].z - z) / span;
            t = t * t * (Real(3) - Real(2) * t);
            return keys[i].v + (keys[i + 1].v - keys[i].v) * t;
        }
    }
    return keys.back().v;
}

// Sample n points along a straight run, inclusive of both ends.
void runPoints(std::vector<ProfilePt>& out, Real u0, Real v0, Real u1, Real v1,
               int n, Run run) {
    n = std::max(n, 2);
    for (int i = 0; i < n; ++i) {
        const Real t = static_cast<Real>(i) / (n - 1);
        out.push_back({u0 + (u1 - u0) * t, v0 + (v1 - v0) * t, run});
    }
}

// The ring cross-section: a chamfered rectangle in unit (u, v). Counter-clockwise
// from the bottom of the right side, so consecutive points always walk the
// perimeter in one direction and cell corners come out in perimeter order.
//
// The chamfer is the whole low-poly read: it is what turns a box into something
// with a shoulder, and it costs four edges.
std::vector<ProfilePt> buildProfile(Real c, int topPts, int bottomPts) {
    c = std::min(std::max(c, Real(0.02)), Real(0.45));
    std::vector<ProfilePt> p;
    // Right side, bottom to top, on the named anchors.
    for (int a = 0; a < kSideAnchorCount; ++a) p.push_back({1, 0, Run::Side, a});
    runPoints(p, 1 - c, 1, -1 + c, 1, topPts, Run::Top);            // top
    // Left side, top to bottom — the reverse anchor order keeps the whole ring
    // walking one way, so cell corners come out in perimeter order.
    for (int a = kSideAnchorCount - 1; a >= 0; --a) p.push_back({-1, 0, Run::Side, a});
    runPoints(p, -1 + c, -1, 1 - c, -1, bottomPts, Run::Bottom);    // floor
    return p;
}

// The run of the EDGE between two profile points: the shared run when they agree,
// otherwise the chamfer that joins them.
Run edgeRun(const ProfilePt& a, const ProfilePt& b) {
    return (a.run == b.run) ? a.run : Run::Chamfer;
}

bool inBand(Real z, const CarBand& b) { return z >= b.z0 && z <= b.z1; }

// Emit one triangle facing AWAY from `axis`, or nothing if it is degenerate.
//
// Both the guard and the per-triangle normal matter. Over the hood the roofline
// falls below the belt, so the greenhouse anchors collapse onto each other and
// produce zero-area slivers; emitTri cannot orient those (its cross product is
// zero), and they would ship as faces whose stored normal matches no geometry.
// And a cell spanning two stations is generally NON-PLANAR, so one normal for
// the whole quad is wrong for at least one half — splitting first and computing
// each normal from its own three corners makes "stored normal == geometric
// normal" hold exactly rather than approximately.
void emitOriented(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
                  const Vec3& axis, const Vec3& color, Real sign) {
    Vec3 n = cross(b - a, c - a);
    if (n.lengthSquared() < Real(1e-12)) return;
    n = normalize(n);
    const Vec3 centroid = (a + b + c) * (Real(1) / Real(3));
    if (dot(n, centroid - axis) * sign < 0) n = n * Real(-1);
    MeshBuilder::emitTri(m, a, b, c, n, color);
}

void emitOutward(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
                 const Vec3& axis, const Vec3& color) {
    emitOriented(m, a, b, c, axis, color, Real(1));
}

// The inner skin faces the other way. Culling is unconditional, so this is not
// cosmetic: an inward face is the only thing a viewer outside the car can see
// THROUGH a window, and an outward-wound one would simply vanish.
void emitInward(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
                const Vec3& axis, const Vec3& color) {
    emitOriented(m, a, b, c, axis, color, Real(-1));
}

Real signOf(Real x) { return x < 0 ? Real(-1) : Real(1); }

}  // namespace

RenderMesh CarMesh::merged() const {
    RenderMesh out;
    for (int i = 0; i < kCarPartCount; ++i) MeshBuilder::append(out, parts[i]);
    return out;
}

CarMesh buildCarMesh(const CarParams& p) {
    CarMesh out;
    const Real W = p.width, H = p.height, L = p.length;
    if (W <= 0 || H <= 0 || L <= 0 || p.stations.size() < 2) return out;

    RenderMesh& body = out.parts[static_cast<int>(CarPart::Body)];
    RenderMesh& interior = out.parts[static_cast<int>(CarPart::Interior)];
    RenderMesh& glass = out.parts[static_cast<int>(CarPart::Glass)];

    // --- stations -----------------------------------------------------------
    // The recipe authors the STYLE stations (nose, hood, cowl, roof, deck, tail).
    // The generator inserts the FUNCTIONAL ones — the four wheel-arch boundaries
    // — so an arch opening always lands exactly on a station edge instead of
    // being approximated by whichever cells happen to straddle it.
    const Real frontAxleZ01 = (L * Real(0.5) - p.frontOverhang) / L;
    const Real rearAxleZ01 = -(L * Real(0.5) - p.rearOverhang) / L;

    std::vector<Real> st = p.stations;
    for (Real az : {frontAxleZ01, rearAxleZ01}) {
        // Both edges AND intermediate stations. One cell spanning the whole arch
        // can only ever be a rectangle; the curve is drawn by the steps between
        // these. Spaced by ARC angle rather than evenly in z, so the steps are
        // finest near the ends where the ellipse turns hardest.
        for (int k = 0; k <= p.archSegments; ++k) {
            const Real t = static_cast<Real>(k) / p.archSegments;
            const Real u = std::cos(t * Real(3.14159265358979));
            st.push_back(az + p.archHalfSpan * u);
        }
    }
    // Every APERTURE EDGE is a station too. A cell is classified by its midpoint,
    // so a band edge that falls between stations is simply not seen: the B-pillar
    // gap between the two door windows is ~0.18 m, the stations either side of it
    // were 0.5 m apart, and the pillar vanished — one continuous pane from A to C.
    // Snapping edges to stations makes an opening land where it was authored
    // rather than where the sampling happened to allow.
    st.push_back(p.windshield.z0);
    st.push_back(p.windshield.z1);
    st.push_back(p.backlight.z0);
    st.push_back(p.backlight.z1);
    for (const CarBand& b : p.sideWindows) {
        st.push_back(b.z0);
        st.push_back(b.z1);
    }
    // RAKE-SWEEP STATIONS. A leaning pillar occupies a different z at every
    // height, so it can only be drawn where stations exist across the band it
    // sweeps. Without them a raked edge does not lean — it vanishes from the
    // upper cells and leaves a floating stub, which is exactly what happened the
    // first time this was switched on.
    //
    // Targeted, not uniform: only the neighbourhood of each pane EDGE is
    // subdivided, because that is the only place the sweep matters. Subdividing
    // the whole daylight opening would roughly double the body for no gain in
    // the middle of a pane.
    if (p.dloRake > Real(0.01) && !p.sideWindows.empty()) {
        const Real glassH = std::max((Real(0.90) - keyEval(p.roof, p.windshield.z1)),
                                     Real(0.10)) * H;
        const Real sweep = glassH * std::tan(degreesToRadians(p.dloRake)) / L;
        const int kSteps = 1;
        for (const CarBand& b : p.sideWindows)
            for (Real edge : {b.z0, b.z1})
                for (int k = 1; k <= kSteps; ++k)
                    st.push_back(edge - sweep * k / kSteps);
    }
    st.erase(std::remove_if(st.begin(), st.end(),
                            [](Real z) { return z <= Real(-0.5) || z >= Real(0.5); }),
             st.end());
    st.push_back(Real(0.5));
    st.push_back(Real(-0.5));
    std::sort(st.begin(), st.end(), [](Real a, Real b) { return a > b; });
    st.erase(std::unique(st.begin(), st.end(),
                         [](Real a, Real b) { return std::fabs(a - b) < Real(1e-4); }),
             st.end());
    const int S = static_cast<int>(st.size());

    const std::vector<ProfilePt> prof =
        buildProfile(p.chamfer, p.topPts, p.bottomPts);
    const int K = static_cast<int>(prof.size());
    if (S < 2 || K < 4) return out;

    // --- the belt line is DERIVED, not authored -----------------------------
    // Orsborn et al. found the belt line has no rules of its own: it emerges from
    // the hood, side windows and trunk. So we take it as the roofline height at
    // the cowl — which is precisely the hood line carried aft, which is what a
    // belt line physically is.
    const Real beltBase = keyEval(p.roof, p.windshield.z1);

    // The belt as a CURVE, not a constant: flat ahead of the B-pillar, then
    // smoothly climbing. Combined with the falling roofline this closes the
    // daylight opening toward the rear — the wedge that a flat belt cannot make.
    auto beltAt = [&](Real z) {
        if (z >= p.beltKickStart) return beltBase;
        const Real span = std::max(p.beltKickStart - p.beltKickEnd, Real(1e-6));
        Real t = std::min((p.beltKickStart - z) / span, Real(1));
        t = t * t * (Real(3) - Real(2) * t);
        return beltBase + p.beltKick * t;
    };
    const Real belt01 = beltBase;

    const Real tumbleTan = std::tan(degreesToRadians(p.tumblehome));

    // --- rings --------------------------------------------------------------
    std::vector<Vec3> ring(static_cast<std::size_t>(S) * K);
    std::vector<Real> ringY01(static_cast<std::size_t>(S) * K);
    std::vector<Real> centreY(S), halfW(S);

    for (int i = 0; i < S; ++i) {
        const Real z01 = st[i];
        const Real zw = z01 * L;
        const Real roof01 = keyEval(p.roof, z01);
        const Real sill01 = keyEval(p.sill, z01);
        const Real rx = Real(0.5) * W * keyEval(p.plan, z01);
        const Real cy01 = (sill01 + roof01) * Real(0.5);
        const Real ry01 = (roof01 - sill01) * Real(0.5);
        centreY[i] = (cy01 - Real(0.5)) * H;
        halfW[i] = rx;

        // Side anchors for this station. The belt and rail are clamped into the
        // ring: over the hood the roofline drops BELOW the belt, and there the
        // greenhouse simply collapses to nothing rather than inverting.
        const Real vTop = Real(1) - std::min(std::max(p.chamfer, Real(0.02)), Real(0.45));
        const Real yLo = cy01 - vTop * ry01;
        const Real yHi = cy01 + vTop * ry01;
        const Real eps = std::max((yHi - yLo) * Real(0.02), Real(1e-4));
        const Real yBelt = std::min(std::max(beltAt(z01), yLo + eps), yHi - eps);
        const Real yRail = std::min(std::max(roof01 - p.railDrop, yBelt + eps), yHi - eps);
        Real anchorY[kSideAnchorCount];
        anchorY[kAnchorSill] = yLo;
        anchorY[kAnchorArchLow] = yLo + (yBelt - yLo) * Real(0.34);
        anchorY[kAnchorLowerMid] = yLo + (yBelt - yLo) * Real(0.67);
        anchorY[kAnchorBelt] = yBelt;
        anchorY[kAnchorGlassMid] = (yBelt + yRail) * Real(0.5);
        anchorY[kAnchorRail] = yRail;
        anchorY[kAnchorRoof] = yHi;

        for (int j = 0; j < K; ++j) {
            const Real y01 = prof[j].anchor >= 0 ? anchorY[prof[j].anchor]
                                                 : cy01 + prof[j].v * ry01;
            Real x = prof[j].u * rx;
            // Tumblehome: lean the sides inward above the belt, by angle from
            // vertical. This is the front-view taper that stops a low-poly car
            // reading as a shoebox.
            if (y01 > belt01 && std::fabs(prof[j].u) > Real(0.5)) {
                const Real inset = tumbleTan * (y01 - belt01) * H;
                x = signOf(x) * std::max(std::fabs(x) - inset, rx * Real(0.05));
            }
            const std::size_t idx = static_cast<std::size_t>(i) * K + j;
            ring[idx] = Vec3(x, (y01 - Real(0.5)) * H, zw);
            ringY01[idx] = y01;
        }
    }

    // --- cell classification + emission -------------------------------------
    // Glass and arch cells emit NOTHING. They are real holes in the skin; P1's
    // inner skin and reveal quads close the cavity behind them.
    const bool cutApertures = (p.lod != CarLod::Low && p.lod != CarLod::Proxy);

    const Real dloTan = std::tan(degreesToRadians(p.dloRake));

    auto isGlass = [&](Real zc, Run run, Real midU, int a1, int a2) {
        if (run == Run::Top) {
            // Leave the outboard strips solid: those are the A-pillar tops and
            // the roof rails. Without them the windshield spans the full width
            // and the greenhouse reads as a hole punched clean through the car.
            //
            // NOTE the pillar test deliberately does NOT apply here. Pillars are
            // SIDE-view members that separate one side window from the next; the
            // A and C pillars sit at z01 0.115 and -0.305, both of which fall
            // inside the windscreen and backlight bands. Testing them against
            // top-run glass chopped each of those windows into two strips with a
            // painted bar across the middle.
            if (std::fabs(midU) > p.glassHalfU) return false;
            return inBand(zc, p.windshield) || inBand(zc, p.backlight);
        }
        if (run == Run::Side) {
            for (Real pz : p.pillars)
                if (std::fabs(zc - pz) < p.pillarHalfWidth) return false;
            // Exactly the cells between the belt and the roof rail — no
            // threshold arithmetic, because the anchors ARE those lines.
            if (std::min(a1, a2) < kAnchorBelt || std::max(a1, a2) > kAnchorRail)
                return false;
            for (const CarBand& b : p.sideWindows)
                if (inBand(zc, b)) return true;
        }
        return false;
    };

    // The arch follows the WHEEL: a semi-ellipse peaking over the axle and
    // closing down to the sill at either end, rather than a flat ceiling across
    // a flat z-window. The old rule was a rectangle in both axes, so the opening
    // was square no matter how round the wheel in it was.
    auto archTopAt = [&](Real zc) {
        Real best = 0;
        for (Real axle : {frontAxleZ01, rearAxleZ01}) {
            const Real d = std::fabs(zc - axle) / p.archHalfSpan;
            if (d >= Real(1)) continue;
            best = std::max(best, p.archTop * std::sqrt(Real(1) - d * d));
        }
        return best;
    };
    auto isArch = [&](Real zc, Run run, Real maxY01) {
        if (run != Run::Side) return false;
        return maxY01 <= archTopAt(zc);
    };

    // --- classify every cell up front ---------------------------------------
    // The hole map is needed three more times after the outer skin: to decide
    // which stations get an inner skin, to walk the aperture boundaries for
    // reveals, and to place glass. Classifying once keeps those four passes
    // from drifting apart.
    enum Cell : std::uint8_t { kSolid, kGlassCell, kArchCell };
    std::vector<std::uint8_t> cell(static_cast<std::size_t>(S - 1) * K, kSolid);

    for (int i = 0; i + 1 < S; ++i) {
        const Real zc = (st[i] + st[i + 1]) * Real(0.5);
        for (int j = 0; j < K; ++j) {
            const int j2 = (j + 1) % K;
            const Run run = edgeRun(prof[j], prof[j2]);
            const std::size_t ia = static_cast<std::size_t>(i) * K + j;
            const std::size_t ib = static_cast<std::size_t>(i) * K + j2;
            const std::size_t ic = static_cast<std::size_t>(i + 1) * K + j2;
            const std::size_t id = static_cast<std::size_t>(i + 1) * K + j;
            const Real maxY01 = std::max(std::max(ringY01[ia], ringY01[ib]),
                                         std::max(ringY01[ic], ringY01[id]));
            const Real midU = (prof[j].u + prof[j2].u) * Real(0.5);
            if (!cutApertures) continue;
            // Rake: a cell higher up the glass is tested against a z shifted
            // FORWARD, which is the same as sloping the pane's edges backward as
            // they rise. Without it every pillar is dead vertical.
            const Real meanY01 = (ringY01[ia] + ringY01[ib] + ringY01[ic] +
                                  ringY01[id]) * Real(0.25);
            const Real hAbove = std::max(meanY01 - beltAt(zc), Real(0)) * H;
            const Real zRaked = zc + hAbove * dloTan / L;
            if (isGlass(zRaked, run, midU, prof[j].anchor, prof[j2].anchor))
                cell[static_cast<std::size_t>(i) * K + j] = kGlassCell;
            else if (isArch(zc, run, maxY01))
                cell[static_cast<std::size_t>(i) * K + j] = kArchCell;
        }
    }
    auto cellAt = [&](int i, int j) -> std::uint8_t {
        return cell[static_cast<std::size_t>(i) * K + ((j % K) + K) % K];
    };
    auto isHole = [&](int i, int j) { return cellAt(i, j) != kSolid; };

    // --- inner skin positions ------------------------------------------------
    // Offset each ring vertex inward along the ring's own 2D outward direction,
    // leaving z alone: the cabin is lined laterally and vertically, and keeping
    // the offset in-plane means the inner skin stays parallel to the outer one
    // through the rake of the windscreen instead of shearing.
    std::vector<Vec3> inner(static_cast<std::size_t>(S) * K);
    for (int i = 0; i < S; ++i) {
        for (int j = 0; j < K; ++j) {
            const std::size_t idx = static_cast<std::size_t>(i) * K + j;
            const Vec3& prev = ring[static_cast<std::size_t>(i) * K + (j + K - 1) % K];
            const Vec3& next = ring[static_cast<std::size_t>(i) * K + (j + 1) % K];
            Vec3 t(next.x - prev.x, next.y - prev.y, 0);
            Vec3 n2 = (t.lengthSquared() > Real(1e-12))
                          ? normalize(Vec3(t.y, -t.x, 0))
                          : Vec3(0, 1, 0);
            const Vec3 fromCentre(ring[idx].x, ring[idx].y - centreY[i], 0);
            if (dot(n2, fromCentre) < 0) n2 = n2 * Real(-1);
            inner[idx] = ring[idx] - n2 * p.shellThickness;
        }
    }

    // Two different questions, and conflating them doubles the mesh.
    //
    //   `lined`  — stations that need REVEALS, i.e. touching any hole including
    //              the wheel arches. Reveals only need inner POSITIONS, which
    //              exist everywhere, so this can be generous.
    //   `walled` — stations that need an inner SKIN, i.e. a surface you can
    //              actually see through a window. Only the greenhouse qualifies.
    //              Lining the arches too would wrap the whole underbody in a
    //              second hull nobody will ever look at.
    auto markSpan = [&](std::vector<bool>& flag, bool glassOnly) {
        for (int i = 0; i + 1 < S; ++i)
            for (int j = 0; j < K; ++j) {
                const std::uint8_t c = cellAt(i, j);
                if (c == kSolid) continue;
                if (glassOnly && c != kGlassCell) continue;
                flag[i] = true;
                flag[i + 1] = true;
            }
        std::vector<bool> grown = flag;
        for (int i = 0; i < S; ++i)
            if (flag[i]) {
                if (i > 0) grown[i - 1] = true;
                if (i + 1 < S) grown[i + 1] = true;
            }
        flag = grown;
    };
    std::vector<bool> lined(S, false), walled(S, false);
    markSpan(lined, false);
    markSpan(walled, true);

    // Nothing below the cabin floor is ever visible — the floor closes it — so
    // the inner skin stops there instead of lining the sills and underbody.
    const Real floorY = (p.cabinFloor - Real(0.5)) * H;

    // --- outer skin, inner skin, reveals -------------------------------------
    // The daylight opening's span, so the blackout band covers the pillars
    // BETWEEN the panes without spilling along the whole flank.
    Real dloFront = -1e9, dloRear = 1e9;
    for (const CarBand& b : p.sideWindows) {
        dloFront = std::max(dloFront, b.z1);
        dloRear = std::min(dloRear, b.z0);
    }

    for (int i = 0; i + 1 < S; ++i) {
        const Real zc = (st[i] + st[i + 1]) * Real(0.5);
        const Real cy = (centreY[i] + centreY[i + 1]) * Real(0.5);
        const Real cz = (st[i] + st[i + 1]) * Real(0.5) * L;
        const Vec3 axis(0, cy, cz);

        for (int j = 0; j < K; ++j) {
            const int j2 = (j + 1) % K;
            const std::size_t ia = static_cast<std::size_t>(i) * K + j;
            const std::size_t ib = static_cast<std::size_t>(i) * K + j2;
            const std::size_t ic = static_cast<std::size_t>(i + 1) * K + j2;
            const std::size_t id = static_cast<std::size_t>(i + 1) * K + j;

            // Solid side cells INSIDE the greenhouse band are pillars, and they
            // get blacked out rather than painted. This is what fuses the two
            // door panes and the post between them into a single dark graphic;
            // painted pillars chop the opening into separate holes and read as
            // bulk. Cheap — a vertex colour — and the biggest single change to
            // how the flank reads after the beltline itself.
            const bool inDlo =
                edgeRun(prof[j], prof[j2]) == Run::Side &&
                std::min(prof[j].anchor, prof[j2].anchor) >= kAnchorBelt &&
                std::max(prof[j].anchor, prof[j2].anchor) <= kAnchorRail &&
                zc <= dloFront + Real(0.03) && zc >= dloRear - Real(0.03);
            const Vec3 skinColor = inDlo ? p.pillarColor : p.paint;

            if (!isHole(i, j)) {
                // Outer skin. TWO TRIANGLES, each with its OWN normal — not one
                // quad with one normal. A cell spanning two stations is
                // generally NON-PLANAR, and a single quad normal is then wrong
                // for at least one half; the two triangles can even face
                // opposite ways. Splitting first makes every face planar by
                // construction, so "stored normal == geometric normal" holds
                // exactly. (The winding test caught this the moment the profile
                // started producing warped cells.)
                emitOutward(body, ring[ia], ring[ib], ring[ic], axis, skinColor);
                emitOutward(body, ring[ia], ring[ic], ring[id], axis, skinColor);

                // Inner skin: the same cell offset inward, wound to face INTO
                // the cabin. This is the headliner and the door cards — the only
                // reason a window shows an interior rather than the sky.
                // Line the whole cabin side, sill to roof — not just the
                // greenhouse.
                //
                // This used to stop at the belt line, on the reasoning that
                // nothing below it "can ever be looked at". That was true only
                // while the glass was accidentally OPAQUE: `material.new`
                // silently dropped `opacity`, so the windows were solid panels
                // and the door cards behind them genuinely were invisible. With
                // real transparency you look straight through a window, past the
                // belt, and out the far side of the car — because the far side
                // below the belt has no inward face and back faces are culled.
                //
                // The floor run is still skipped: the cabin floor is emitted
                // separately below and closes that off.
                const Run cellRun = edgeRun(prof[j], prof[j2]);
                const bool linable = cellRun == Run::Top || cellRun != Run::Bottom;
                if (walled[i] && walled[i + 1] && linable) {
                    emitInward(interior, inner[ia], inner[ib], inner[ic], axis,
                               p.interiorColor);
                    emitInward(interior, inner[ia], inner[ic], inner[id], axis,
                               p.interiorColor);
                }
            } else if (cellAt(i, j) == kArchCell) {
                // The wheel well's back wall. Without it the arch is a cut whose
                // reveal has nothing to terminate against: the lip's inner edge
                // is a boundary, and since culling is unconditional it simply
                // disappears from one side, so the car is see-through at the
                // arch. Emitting the same cell at the inner offset turns the
                // hole into a closed pocket. Faces OUTWARD, because you look at
                // this from outside the car, unlike the cabin liner.
                emitOutward(body, inner[ia], inner[ib], inner[ic], axis, p.archColor);
                emitOutward(body, inner[ia], inner[ic], inner[id], axis, p.archColor);
            }

            // Reveals: wherever a solid cell borders a hole, bridge outer to
            // inner along the shared edge. Without these the opening is a
            // zero-width slit and you see the skin's back face — which culling
            // removes, so the car appears to have a gap in it.
            if (!lined[i] || !lined[i + 1]) continue;
            auto reveal = [&](std::size_t o0, std::size_t o1) {
                emitOutward(body, ring[o0], ring[o1], inner[o1], axis, p.paint);
                emitOutward(body, ring[o0], inner[o1], inner[o0], axis, p.paint);
            };
            const bool here = isHole(i, j);
            // Around the ring: the edge between cell j and cell j-1.
            if (here != isHole(i, j - 1)) reveal(ia, id);
            // Along the car: the edge between station-pair i and i-1.
            if (i > 0 && here != isHole(i - 1, j)) reveal(ia, ib);
        }
    }

    // --- floor, bulkheads, glass --------------------------------------------
    int firstLined = -1, lastLined = -1;
    for (int i = 0; i < S; ++i)
        if (lined[i]) { if (firstLined < 0) firstLined = i; lastLined = i; }

    if (firstLined >= 0 && lastLined > firstLined) {
        // Half-width of the cabin at a station, taken from the inner skin at the
        // rocker so the floor meets the door cards instead of poking through.
        int sillIdx = 0;
        for (int j = 0; j < K; ++j)
            if (prof[j].anchor == kAnchorSill && prof[j].u > 0) { sillIdx = j; break; }
        auto innerHalfW = [&](int i) {
            return std::fabs(inner[static_cast<std::size_t>(i) * K + sillIdx].x);
        };
        auto innerTopY = [&](int i) {
            Real top = -1e30;
            for (int j = 0; j < K; ++j)
                top = std::max(top, inner[static_cast<std::size_t>(i) * K + j].y);
            return top;
        };

        // Floor: up-facing, one strip per station pair. Closes the cavity from
        // below so a window shows a cabin rather than a view out of the sills.
        for (int i = firstLined; i < lastLined; ++i) {
            const Real w0 = innerHalfW(i), w1 = innerHalfW(i + 1);
            const Real z0 = st[i] * L, z1 = st[i + 1] * L;
            const Vec3 a(-w0, floorY, z0), b(w0, floorY, z0);
            const Vec3 c(w1, floorY, z1), d(-w1, floorY, z1);
            const Vec3 below(0, floorY - Real(1), (z0 + z1) * Real(0.5));
            emitOutward(interior, a, b, c, below, p.interiorColor);
            emitOutward(interior, a, c, d, below, p.interiorColor);
        }

        // Firewall and rear bulkhead, each facing INTO the cabin. These are what
        // stop you seeing along the length of the car and out the far end.
        auto bulkhead = [&](int i, Real facing) {
            const Real w = innerHalfW(i);
            const Real top = std::max(innerTopY(i), floorY + Real(0.05));
            const Real z = st[i] * L;
            const Vec3 a(-w, floorY, z), b(w, floorY, z), c(w, top, z), d(-w, top, z);
            const Vec3 behind(0, (floorY + top) * Real(0.5), z - facing);
            emitOutward(interior, a, b, c, behind, p.interiorColor);
            emitOutward(interior, a, c, d, behind, p.interiorColor);
        };
        // Skip a bulkhead that lands on the body's own end CAP. When the lined
        // cabin reaches the nose (firstLined == 0) or the tail (lastLined ==
        // S-1) — which a van does, its greenhouse running the full length — the
        // bulkhead quad ends up coplanar with the cap FAN at that end, both
        // facing the same way. The two triangulations then z-fight, and because
        // one is a radial fan it reads as a dark starburst on the rear (the van
        // showed this plainly). The cap already closes the cavity there, so the
        // bulkhead is redundant exactly when it would fight.
        if (firstLined > 0) bulkhead(firstLined, L);        // firewall
        if (lastLined < S - 1) bulkhead(lastLined, -L);     // rear bulkhead

        // --- the grammar's half ----------------------------------------------
        // The sweep has now closed a cavity; its bounds become a Scope and the
        // building grammar fills it. This is the seam between the two machines:
        // everything above makes SURFACE, everything below subdivides VOLUME.
        // The cabin is the GREENHOUSE, and specifically the UNPADDED glass range.
        // Two mistakes lived here: `lined` includes the wheel arches, which reach
        // far past the doors (that put the seats in the engine bay), and even
        // `walled` is padded a station either side so the cavity closes behind
        // the glass — padding that belongs to the skin, not to the furniture.
        // Taken literally it stretched the cabin from the bumper to the boot.
        int firstW = -1, lastW = -1;
        for (int i = 0; i + 1 < S; ++i)
            for (int j = 0; j < K; ++j)
                if (cellAt(i, j) == kGlassCell) {
                    if (firstW < 0) firstW = i;
                    lastW = i + 1;
                }

        // The cabin Scope is a RECTANGULAR PRISM, but the cavity it stands in is
        // not: under the raked screen the roof is barely above the floor, and a
        // box sized by mid-cabin headroom burst straight through the bonnet.
        // Walk both ends inward until the local headroom can actually hold the
        // furniture, then take the roof as the LOWEST point over what is left —
        // so the box fits everywhere, not just in the middle.
        const Real kMinHeadroom = Real(0.62);
        while (firstW < lastW && innerTopY(firstW) - floorY < kMinHeadroom) ++firstW;
        while (lastW > firstW && innerTopY(lastW) - floorY < kMinHeadroom) --lastW;

        if (p.interior && firstW >= 0 && lastW > firstW) {
            CabinSpace cabin;
            cabin.frontZ = st[firstW] * L;
            cabin.rearZ = st[lastW] * L;
            cabin.floorY = floorY;
            cabin.roofY = Real(1e30);
            cabin.halfWidth = Real(1e30);
            for (int i = firstW; i <= lastW; ++i) {
                cabin.roofY = std::min(cabin.roofY, innerTopY(i));
                cabin.halfWidth = std::min(cabin.halfWidth, innerHalfW(i));
            }

            InteriorParams ip;
            ip.rows = p.rows;
            ip.seatPitch = p.seatPitch;
            ip.backAngle = p.backAngle;
            ip.steerDiameter = p.steerDiameter;
            ip.trimColor = p.interiorColor;
            ip.seatColor = p.seatColor;

            Vec3 sgrp(0, 0, 0);
            MeshBuilder::append(interior, buildCarInterior(cabin, ip, &sgrp));
            out.attaches.push_back({sgrp, Vec3(0, 0, 1), "driver_seat"});
        }

        // Glass: set into the aperture, and emitted TWICE with opposite winding.
        // There is no two-sided material flag, so a single pane would disappear
        // from whichever side it was not wound for — including from the driver's
        // seat looking out.
        const Real t = (p.shellThickness > Real(1e-6))
                           ? std::min(p.glassInset / p.shellThickness, Real(1))
                           : Real(0);
        for (int i = 0; i + 1 < S; ++i) {
            const Real cy = (centreY[i] + centreY[i + 1]) * Real(0.5);
            const Real cz = (st[i] + st[i + 1]) * Real(0.5) * L;
            const Vec3 axis(0, cy, cz);
            for (int j = 0; j < K; ++j) {
                if (cellAt(i, j) != kGlassCell) continue;
                const int j2 = (j + 1) % K;
                const std::size_t ia = static_cast<std::size_t>(i) * K + j;
                const std::size_t ib = static_cast<std::size_t>(i) * K + j2;
                const std::size_t ic = static_cast<std::size_t>(i + 1) * K + j2;
                const std::size_t id = static_cast<std::size_t>(i + 1) * K + j;
                auto pane = [&](std::size_t k) {
                    return ring[k] + (inner[k] - ring[k]) * t;
                };
                const Vec3 a = pane(ia), b = pane(ib), c = pane(ic), d = pane(id);
                // ONE pane, not two. Emitting it a second time facing inward is
                // how it used to be visible from both sides, but the transparent
                // pass does not write depth, so both coincident faces blended:
                // 0.32 authored read as 0.54 per pane and ~79% through the whole
                // car, which is why the windows looked like panels and the
                // interior was invisible. The glass material now carries
                // FLAG_TWO_SIDED and the backend draws both faces of this ONE
                // pane instead. Also halves the glass and removes the coincident
                // faces that made this part non-manifold.
                emitOutward(glass, a, b, c, axis, p.glassTint);
                emitOutward(glass, a, c, d, axis, p.glassTint);
            }
        }
    }

    // --- end caps -----------------------------------------------------------
    // The cap fan faces away from a point pushed back along the car's axis, so
    // "outward" means "out of the nose/tail" rather than "away from the ring".
    auto cap = [&](int i, Real inward) {
        const Vec3 ctr(0, centreY[i], st[i] * L);
        const Vec3 axis(0, centreY[i], st[i] * L + inward);
        for (int j = 0; j < K; ++j) {
            const int j2 = (j + 1) % K;
            emitOutward(body, ctr, ring[static_cast<std::size_t>(i) * K + j],
                        ring[static_cast<std::size_t>(i) * K + j2], axis, p.paint);
        }
    };
    cap(0, -L * Real(0.1));
    cap(S - 1, L * Real(0.1));

    // --- lamps ---------------------------------------------------------------
    // Both a LENS you can see and a MARKER the caller lights.
    //
    // The lens geometry is its own part with its own material, so the caller
    // decides whether it glows — the generator bakes no emission. citysim already
    // works this way: it draws the body's lamp boxes AND overlays emissive
    // instances at the markers per lamp state (headlights by clock, brakes on
    // deceleration, indicators blinking), so a lamp that baked its own glow would
    // be lit twice and could never switch off.
    //
    // Marker names are the contract citysim's syncCarLamps parses: a "headlight"
    // or "taillight" prefix, and a trailing 'l'/'r' which is what selects the
    // side for turn signals.
    const Real lampY01 = std::max(belt01 - Real(0.14), Real(0.30));
    const Real lampY = (lampY01 - Real(0.5)) * H;
    RenderMesh& lamp = out.parts[static_cast<int>(CarPart::Lamp)];

    // Lamps sit on the NOSE and TAIL end rings (station 0 / S-1). Those rings are
    // closed by a FLAT cap in their own z plane — not a rounded, forward-bulging
    // nose — so the frontmost bodywork at the lamp is that flat face, and a lens
    // set just proud of it is visible and reads as flush-mounted.
    //
    // Two earlier attempts, for the next person: recessing DEEPER buries the lens
    // behind the cap (invisible), and moving the station INBOARD buries it behind
    // the nose bodywork that is now in front of it (also invisible). The lens must
    // be at the tip and slightly proud — the only free knob is HOW proud. 15 mm
    // read as "sticking out"; this leaves ~5 mm, a flush lens.
    constexpr Real kLampProud = 0.005;   // metres the lens stands off the cap
    for (int side = 0; side < 2; ++side) {
        const Real sx = side == 0 ? Real(1) : Real(-1);
        const char* tag = side == 0 ? "_r" : "_l";
        struct End { int station; Real dir; Vec3 tint; const char* name; };
        const End ends[2] = {
            {0, Real(1), Vec3(1.0, 0.97, 0.82), "headlight"},
            {S - 1, Real(-1), Vec3(0.85, 0.06, 0.05), "taillight"},
        };
        for (const End& e : ends) {
            const Real z = st[e.station] * L;
            const Vec3 at(sx * halfW[e.station] * Real(0.60), lampY, z);
            out.attaches.push_back({at, Vec3(0, 0, e.dir),
                                    std::string(e.name) + tag});

            // Position the lens so its OUTER face stands kLampProud beyond the cap
            // plane: centre = face - dir * halfDepth. The box's other 95% sits
            // inside the body, so it reads as a lens set into the panel.
            RenderMesh lens = MeshBuilder::box(
                Vec3(p.lampWidth, p.lampHeight, p.lampDepth));
            for (Vertex& v : lens.vertices) v.color = e.tint;
            const Real faceZ = z + e.dir * kLampProud;
            const Real centreZ = faceZ - e.dir * p.lampDepth * Real(0.5);
            MeshBuilder::appendTransformed(
                lamp, lens,
                Mat4::trs(Vec3(at.x, at.y, centreZ), Quat(), Vec3(1, 1, 1)));
        }
    }

    out.halfExtent = Vec3(W * Real(0.5), H * Real(0.5), L * Real(0.5));
    return out;
}

CarParams defaultSedanParams() {
    CarParams p;
    // Mirrors vehicle_classes.lua `sedan`: 4.59 x 1.82 x 1.45 m on a 2.70 m
    // wheelbase. Kept in sync by tests/test_car_mesh.cpp.
    p.width = 1.82;
    p.height = 1.45;
    p.length = 4.59;
    p.wheelDiameter = 0.62;
    p.frontOverhang = 0.90;
    p.rearOverhang = 1.00;
    p.groundClearance = 0.14;

    // Style stations, nose first. The four arch boundaries are inserted by the
    // generator, so this list is only the silhouette breaks a designer names.
    // ONLY the style breaks. Every aperture edge and wheel-arch boundary is
    // inserted by the generator, so listing them here too just produced pairs of
    // stations a few millimetres apart — each one a full extra ring around the
    // car. Dropping the five duplicates costs nothing visually.
    p.stations = {0.50, 0.44, 0.30, 0.19, 0.02, -0.28, -0.44, -0.50};

    // Cowl at 0.25 — just behind the front axle (0.304). A FWD car has a SHORT
    // dash-to-axle: the engine sits over the front wheels, so the screen starts
    // close behind them. Pushing the cowl to 0.15 gave a 0.71 m dash-to-axle and
    // the car read as a 1970s rear-drive coupe with an enormous bonnet.
    p.roof = {{0.50, 0.42}, {0.44, 0.46}, {0.30, 0.52}, {0.25, 0.55}, {0.125, 0.95},
              {-0.24, 0.96}, {-0.40, 0.62}, {-0.46, 0.58}, {-0.50, 0.54}};
    p.sill = {{0.50, 0.22}, {0.44, 0.14}, {0.30, 0.10}, {-0.40, 0.10},
              {-0.46, 0.14}, {-0.50, 0.22}};
    p.plan = {{0.50, 0.62}, {0.44, 0.82}, {0.30, 0.97}, {0.02, 1.00},
              {-0.22, 1.00}, {-0.40, 0.97}, {-0.46, 0.88}, {-0.50, 0.70}};

    p.chamfer = 0.16;
    // Six across the top so the windshield can be inset from the roof rails and
    // still be more than one cell wide.
    p.topPts = 6;
    p.bottomPts = 2;   // the floor pan is never cut; it needs no detail
    p.tumblehome = 7.0;

    // Windshield: 0.58 m of horizontal run, from trade glass dimensions (~1.5 x
    // 0.7 m panel) at ~58 deg rake. The first pass spanned 1.10 m — nearly double
    // a real screen — which pushed the roof back and made the whole greenhouse
    // read wrong.
    p.windshield = {0.125, 0.25};
    p.backlight = {-0.40, -0.24};

    // The DLO — the strongest single graphic on a car's side.
    //
    // The GAPS between these bands ARE the pillars, which is why `pillars` is now
    // empty: running both mechanisms at once meant the pillar list punched holes
    // through bands it was never meant to touch. Front door glass ~0.90 m, rear
    // ~0.70 m, B-pillar ~0.16 m between them — all measured figures, and all much
    // longer than the 0.41 m stub the first pass produced.
    // Front door glass ~0.90 m, rear ~0.70 m, B-pillar ~0.16 m between them.
    // The rear pane was 1.15 m — 64% oversized — which INVERTED the ratio (real
    // cars run front/rear 1.00-1.25) and made the front window look small next
    // to it even though the front was already the right size.
    //
    // Four panes, not two: each door carries a QUARTER LIGHT separated from its
    // main pane by a slim bar. The rear one is not decoration — that section of
    // glass physically cannot roll down, because the rear wheel housing cuts into
    // the door, so real cars fix it in place. The front one is the "mirror sail"
    // pane left over when the mirror moved off the A-pillar onto the door skin.
    //
    // Unlike the pillar rake, these cost almost nothing: a band's edges BECOME
    // stations, so a vertical-edged pane is exactly what this grid draws well.
    p.sideWindows = {
        {0.082, 0.110},      // front door quarter light (leading)
        {-0.086, 0.072},     // front door main pane
        {-0.235, -0.121},    // rear door main pane
        {-0.273, -0.245},    // rear door quarter light (trailing, fixed)
    };
    p.pillars = {};
    p.pillarHalfWidth = 0.026;

    // A-pillar / roof rail width. UMTRI-2006-29 measured A-pillars across 21
    // vehicles at 0.085-0.138 m; 0.12 m is the mean. The top run reaches u=0.84
    // (the chamfer), so leaving 0.12 m of body puts the glass edge at ~0.71 —
    // against the 0.55 of the first pass, which left a pillar four times too wide
    // and a windshield that looked like a letterbox.
    p.glassHalfU = 0.71;
    // Side glass ~0.42 m tall on a 1.45 m car; the rest of the greenhouse is roof
    // rail and header, not glass (greenhouse != DLO).
    p.railDrop = 0.07;
    p.archHalfSpan = 0.080;
    p.archTop = 0.42;

    p.paint = Vec3(0.72, 0.10, 0.10);
    return p;
}

}  // namespace engine
