#include "lot_mesh.h"

#include "shape_ops.h"
#include "roof_plant.h"      // the shared rooftop arrangement (audit §18.3)
#include "shape_grammar.h"   // offsetPlan — the ONE mitred ring offset (audit §18.3)
#include "../../mesh_builder.h"
#include "triangulate.h"
#include "rng.h"
#include "../../../log.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace engine {
namespace {

constexpr Real kTau = 6.28318530717958647692;

// Vec2 is world XZ (polygon.h's convention, which the whole city speaks), so a
// plan point at height y is (x, y, z).
Vec3 at3(const Vec2& p, Real y) { return Vec3(p.x, y, p.y); }

// The outward normal of a CCW edge, lifted to 3D. Same formula the 2D ops use
// (right of a CCW edge), so a wall's outside is the same outside everywhere.
Vec3 outward3(const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const Real len = d.length();
    if (len < 1e-9) return Vec3(1, 0, 0);
    return Vec3(d.y / len, 0, -d.x / len);
}

// Geometry is accumulated into one slot PER PART, not appended to a growing
// vector of parts. shape_grammar's equivalent returns a reference into
// `out.parts` and warns that the next call invalidates it; a fixed array cannot
// be invalidated at all, so a wall reference stays live across the glass and
// trim writes that happen in the middle of emitting it. (Written the other way
// first, and it segfaulted on the first tower.)
using PartAcc = std::array<RenderMesh, static_cast<std::size_t>(PartId::Count)>;

RenderMesh& part(PartAcc& acc, PartId id) {
    return acc[static_cast<std::size_t>(id)];
}

// Flatten into the BuildingMesh's part list. `materialIndex` IS the PartId
// ordinal — the contract city_lots relies on when mapping a part back to a
// RenderMaterial.
void flatten(PartAcc& acc, BuildingMesh& out) {
    for (std::size_t i = 0; i < acc.size(); ++i) {
        if (acc[i].indices.empty()) continue;
        RenderMesh* dst = nullptr;
        for (RenderMesh& p : out.parts)
            if (p.materialIndex == static_cast<int>(i)) { dst = &p; break; }
        if (!dst) {
            out.parts.push_back(std::move(acc[i]));
            out.parts.back().materialIndex = static_cast<int>(i);
            continue;
        }
        const uint32_t base = static_cast<uint32_t>(dst->vertices.size());
        dst->vertices.insert(dst->vertices.end(), acc[i].vertices.begin(),
                             acc[i].vertices.end());
        for (uint32_t idx : acc[i].indices) dst->indices.push_back(base + idx);
    }
}

// How many chords an arc needs to stay within `chordTol` of the true curve —
// the same inversion `tessellate` does, applied to a SUB-span of an edge so a
// wall between two windows still curves.
int arcSteps(const Vec2& a, const Vec2& b, Real bulge, Real span, Real chordTol) {
    if (std::fabs(bulge) < 1e-9) return 1;
    const ArcGeom g = arcGeom(a, b, bulge);
    if (g.straight || g.radius < 1e-6) return 1;
    const Real ratio = std::max(Real(-1), std::min(Real(1), 1.0 - chordTol / g.radius));
    const Real maxStep = 2.0 * std::acos(ratio);
    const Real sweep = std::fabs(g.sweep) * std::max(Real(0), span);
    return std::max(1, static_cast<int>(std::ceil(sweep / std::max(maxStep, Real(1e-4)))));
}

// A vertical band of wall between two parameters along one plan edge, from y0
// to y1. Curved edges are subdivided here — the only place in the pipeline that
// turns an arc into chords.
void emitBand(RenderMesh& mesh, const Vec2& a, const Vec2& b, Real bulge,
              Real t0, Real t1, Real y0, Real y1, const Vec3& color,
              Real chordTol, bool faceOut = true) {
    if (t1 <= t0 + 1e-6 || y1 <= y0 + 1e-6) return;
    const int steps = arcSteps(a, b, bulge, t1 - t0, chordTol);
    Vec2 prev = arcPoint(a, b, bulge, t0);
    for (int s = 1; s <= steps; ++s) {
        const Real t = t0 + (t1 - t0) * (static_cast<Real>(s) / steps);
        const Vec2 cur = arcPoint(a, b, bulge, t);
        Vec3 n = outward3(prev, cur);
        if (!faceOut) n = Vec3(-n.x, -n.y, -n.z);
        // emitQuad orients the triangles from the normal, so the corner order
        // only has to be consistent, never correct.
        MeshBuilder::emitQuad(mesh, at3(prev, y0), at3(cur, y0), at3(cur, y1),
                              at3(prev, y1), n, color);
        prev = cur;
    }
}

// A horizontal slab from a region, at height y. Uses the hole-aware
// triangulator — the first caller in the codebase to pass it a hole, which is
// what a courtyard plan or an atrium ring finally needs.
void emitSlab(RenderMesh& mesh, const Shape2& region, Real y, bool up,
              const Vec3& color, Real chordTol) {
    if (region.outer.size() < 3) return;
    const TessShape t = tessellate(region, chordTol);
    if (t.outer.size() < 3) return;
    const std::vector<std::array<Vec2, 3>> tris = triangulateWithHoles(t.outer, t.holes);
    const Vec3 n(0, up ? 1 : -1, 0);
    for (const std::array<Vec2, 3>& tri : tris) {
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int k = 0; k < 3; ++k) {
            Vertex v(at3(tri[k], y), n, static_cast<float>(tri[k].x * 0.25),
                     static_cast<float>(tri[k].y * 0.25));
            v.tangent = Vec3(1, 0, 0);
            v.color = color;
            mesh.vertices.push_back(v);
        }
        // The engine's winding convention, which is not the obvious one: look at
        // MeshBuilder::emitQuad and it orders indices so that
        // cross(b - a, c - a) points AGAINST the shading normal. Every mesh in
        // the engine is built that way, so a hand-wound triangle has to match or
        // it is backfacing against everything else.
        //
        // triangulateWithHoles returns CCW triangles in (x, z); mapped to 3D
        // that cross product already points DOWN, so an up-facing slab keeps the
        // triangulator's order and a down-facing one reverses it.
        if (up)
            mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2});
        else
            mesh.indices.insert(mesh.indices.end(), {base, base + 2, base + 1});
    }
}

// A vertical face swept around a closed ring, facing AWAY FROM `reference` — the
// ring paired with it (a band's other edge, a parapet's other side).
//
// Taking the direction from the pairing rather than from the winding is the
// point. "Outward is the right of a CCW edge" is only true in one orientation
// convention, and the ring here has been through tessellate() and offsetPlan()
// (which normalizes its own copy), so its winding is not the caller's to assume
// — getting it backwards silently turns every base course inside out. Two
// mitred rings, on the other hand, always know which of them is outside.
void emitRingFace(RenderMesh& mesh, const Poly2& ring, const Poly2& reference,
                  Real y0, Real y1, const Vec3& color) {
    const std::size_t n = ring.size();
    if (reference.size() != n) return;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        const Vec2& a = ring[i];
        const Vec2& b = ring[j];
        Vec2 d = b - a;
        if (d.length() < 1e-6) continue;
        const Vec2 mid = (a + b) * Real(0.5);
        const Vec2 ref = (reference[i] + reference[j]) * Real(0.5);
        Vec2 nrm = mid - ref;
        if (nrm.length() < 1e-9) {                  // degenerate pairing
            d = normalize(d);
            nrm = Vec2(d.y, -d.x);
        }
        nrm = normalize(nrm);
        MeshBuilder::emitQuad(mesh, at3(a, y0), at3(b, y0), at3(b, y1), at3(a, y1),
                              Vec3(nrm.x, 0, nrm.y), color);
    }
}

// The flat annulus between two rings that share a vertex count — the top of a
// parapet, the soffit of a cornice. This is what closes a corner: the two rings
// are MITRED, so o[i]..i[i] meet exactly.
void emitRingCap(RenderMesh& mesh, const Poly2& o, const Poly2& in, Real y,
                 bool up, const Vec3& color) {
    if (o.size() != in.size()) return;
    const std::size_t n = o.size();
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        MeshBuilder::emitQuad(mesh, at3(o[i], y), at3(o[j], y), at3(in[j], y),
                              at3(in[i], y), Vec3(0, up ? 1 : -1, 0), color);
    }
}

// A projecting band swept around a loop — a cornice or a base course.
//
// This used to offset every SEGMENT along its own normal, which is not an offset
// ring: at a corner the two neighbouring segments move in different directions
// and never meet, leaving an open wedge at every corner of every moulding. It
// reads exactly like the mesher giving up half way round. Offset the tessellated
// ring properly instead (shape_grammar's `offsetPlan`, mitred and
// vertex-count-preserving, which is why the two rings can be walked in
// parallel), then the band is two faces and two caps between rings that close.
void emitSweptBand(PartAcc& acc, const Loop2& loop, Real y, Real height,
                   Real project, PartId id, const Vec3& color, Real chordTol) {
    if (loop.size() < 3 || height <= 1e-4 || project <= 1e-4) return;
    Poly2 inner = tessellate(loop, chordTol);
    if (inner.size() < 3) return;
    // offsetPlan normalizes its own copy's winding, so both rings must be put in
    // ITS convention or the index correspondence the caps rely on is reversed.
    ensureCCW(inner);
    const Poly2 outer = offsetPlan(inner, -project);   // d < 0 grows outward
    if (outer.size() != inner.size()) return;
    RenderMesh& mesh = part(acc, id);
    emitRingFace(mesh, outer, inner, y, y + height, color);       // fascia
    emitRingCap(mesh, outer, inner, y + height, true, color);     // top
    emitRingCap(mesh, outer, inner, y, false, color);             // soffit
}

// A PARAPET is a wall, not a ribbon: an upstand with an inner and an outer face
// and a coping that oversails both. Sweeping it as a zero-projection band gave a
// single-sided sheet with no thickness, no top and no inside — visible as a roof
// edge you can see straight through. Mirrors the shipping pipeline's
// `emitPlanParapet`, which already had this right (audit §18.2).
void emitParapetRing(PartAcc& acc, const Loop2& loop, Real roofY, Real height,
                     PartId wallId, const Vec3& wallColor, PartId copingId,
                     const Vec3& copingColor, Real chordTol) {
    if (loop.size() < 3 || height <= 1e-4) return;
    const Real th = 0.24;    // upstand thickness
    const Real lip = 0.05;   // coping oversail, inboard and outboard
    const Real ch = 0.09;    // coping height
    Poly2 O = tessellate(loop, chordTol);              // upstand outer: on the plan line
    if (O.size() < 3) return;
    ensureCCW(O);   // offsetPlan normalizes its copy; the rings must agree

    const Poly2 I  = offsetPlan(O, th);                // upstand inner
    const Poly2 Oc = offsetPlan(O, -lip);              // coping outer
    const Poly2 Ic = offsetPlan(O, th + lip);          // coping inner
    if (I.size() != O.size() || Oc.size() != O.size() || Ic.size() != O.size())
        return;
    const Real top = roofY + height;
    RenderMesh& wall = part(acc, wallId);
    emitRingFace(wall, O, I, roofY, top, wallColor);   // outer: away from the inside
    emitRingFace(wall, I, O, roofY, top, wallColor);   // inner: away from the street
    RenderMesh& cop = part(acc, copingId);
    emitRingFace(cop, Oc, Ic, top, top + ch, copingColor);
    emitRingFace(cop, Ic, Oc, top, top + ch, copingColor);
    emitRingCap(cop, Oc, Ic, top + ch, true, copingColor);   // coping top
    emitRingCap(cop, Oc, Ic, top, false, copingColor);       // and its underside
}

// An axis-aligned-in-its-own-frame box, six faces, seated on the roof.
void emitPlantBox(RenderMesh& mesh, const Vec2& o, const Vec2& u, const Vec2& v,
                  Real w, Real d, Real y0, Real y1, const Vec3& color) {
    const Vec2 c[4] = {o, o + u * w, o + u * w + v * d, o + v * d};
    for (int i = 0; i < 4; ++i) {
        const Vec2& a = c[i];
        const Vec2& b = c[(i + 1) % 4];
        Vec2 dd = b - a;
        if (dd.length() < 1e-6) continue;
        dd = normalize(dd);
        const Vec2 nrm(dd.y, -dd.x);
        MeshBuilder::emitQuad(mesh, at3(a, y0), at3(b, y0), at3(b, y1), at3(a, y1),
                              Vec3(nrm.x, 0, nrm.y), color);
    }
    MeshBuilder::emitQuad(mesh, at3(c[0], y1), at3(c[1], y1), at3(c[2], y1),
                          at3(c[3], y1), Vec3(0, 1, 0), color);
}

// A vertical tube + its lid — water tanks and fan cowls.
void emitPlantTube(RenderMesh& side, RenderMesh& lid, const Vec2& c, Real radius,
                   Real y0, Real y1, int segments, const Vec3& sideCol,
                   const Vec3& lidCol) {
    const Real tau = 6.28318530717958647692;
    for (int i = 0; i < segments; ++i) {
        const Real a0 = tau * i / segments, a1 = tau * (i + 1) / segments;
        const Vec2 p0(c.x + std::cos(a0) * radius, c.y + std::sin(a0) * radius);
        const Vec2 p1(c.x + std::cos(a1) * radius, c.y + std::sin(a1) * radius);
        Vec2 n = (p0 + p1) * Real(0.5) - c;
        if (n.length() < 1e-9) continue;
        n = normalize(n);
        MeshBuilder::emitQuad(side, at3(p0, y0), at3(p1, y0), at3(p1, y1),
                              at3(p0, y1), Vec3(n.x, 0, n.y), sideCol);
        MeshBuilder::emitQuad(lid, at3(c, y1), at3(p0, y1), at3(p1, y1), at3(c, y1),
                              Vec3(0, 1, 0), lidCol);
    }
}

// Which levels belong to which tier's material set, and the fenestration style
// that goes with it.
const MaterialSet& setFor(const BuiltBuilding& b, int tier) {
    return b.materials.forTier(tier);
}

// Does the recipe want openings on this level/edge at all? An Opening placement
// is the recipe's PERMISSION ("this wall is fenestrated"); the geometry comes
// from `fenestrate`, which knows about sills and heads. Reading the permission
// from the registry is what keeps a blank-walled works shed blank.
bool fenestrated(const std::vector<ElementPlacement>& placements, int level, int edge) {
    for (const ElementPlacement& p : placements)
        if (p.kind == ElementKind::Opening && p.level == level && p.edge == edge)
            return true;
    return false;
}

}  // namespace

BuildingMesh meshBuilding(const BuiltBuilding& b, const LotMeshParams& params) {
    BuildingMesh out;
    const BuildingSurfaces& S = b.surfaces;
    if (S.levels.empty()) return out;
    PartAcc acc;

    const Real tol = std::max(Real(0.02), params.chordTol);
    Real top = params.baseY;
    // Tally of element kinds the recipe asked for and this mesher cannot draw,
    // reported once per building rather than swallowed per placement.
    std::array<int, static_cast<std::size_t>(ElementKind::RoofPlant) + 1> unbuiltKinds{};

    // Which level index starts each tier, so "floor 0" means the ground floor of
    // its own tier — the same convention resolveElements uses.
    std::vector<int> tierStart(S.topTier + 2, 0);
    for (std::size_t i = 0; i < S.levels.size(); ++i) {
        const int t = S.levels[i].tier;
        if (t + 1 < static_cast<int>(tierStart.size()) && tierStart[t] == 0 &&
            (i == 0 || S.levels[i - 1].tier != t))
            tierStart[t] = static_cast<int>(i);
    }

    for (std::size_t li = 0; li < S.levels.size(); ++li) {
        const Level& L = S.levels[li];
        const MaterialSet& ms = setFor(b, L.tier);
        const Real y0 = params.baseY + L.y0, y1 = params.baseY + L.y1;
        top = std::max(top, y1);
        const int floorInTier =
            static_cast<int>(li) -
            (L.tier < static_cast<int>(tierStart.size()) ? tierStart[L.tier] : 0);
        const bool ground = (li == 0);

        // ONE FLAT EDGE INDEX per level, walking plans then holes in exactly the
        // order surfacesFor built the bay grids and tagOfEdge reads the tags. It
        // is also the index resolveElements stamps into ElementPlacement::edge.
        // The wall loop used the per-plan `e` and guarded with `pi == 0`, which
        // meant every plan after the first, and EVERY COURTYARD, was addressing
        // the wrong grid or none at all.
        std::size_t gridIdx = 0;

        // One wall, fenestrated or not. A courtyard wall is the same wall facing
        // the other way: its grid was always built (facade_plan.cpp), its tag is
        // Court, and the resolver already places Openings on it. Court walls used
        // to be a blank band per level instead, which is why a courtyard read as
        // an empty shaft rather than as somewhere a building looks into.
        auto emitWall = [&](const Vec2& a, const Vec2& c, Real bulge, EdgeTag tag,
                            std::size_t gi, bool faceOut) {
                const Vec3 wallCol = ms.wallColor;

                // A party wall is blank BY CONSTRUCTION — the payoff of tagging
                // an edge once, at plan time, instead of guessing with a dot
                // product at draw time.
                const WallRole role =
                    roleForTag(tag, floorInTier, params.retail && ground);
                const bool wantGlass =
                    params.detail == FacadeDetail::Full && role != WallRole::Blank &&
                    gi < S.grids[li].size() &&
                    fenestrated(b.placements, static_cast<int>(li), static_cast<int>(gi));

                RenderMesh& wall = part(acc, ms.wall);
                if (!wantGlass) {
                    if (params.detail == FacadeDetail::Flat && role != WallRole::Blank) {
                        // FLAT LOD: a spandrel band and a glass band per storey,
                        // read off the same blueprint the full detail punches.
                        // Two quads instead of a dozen, and the windows land in
                        // the same place, so the swap is invisible.
                        const Real h = y1 - y0;
                        const Real sill = y0 + h * 0.30, head = y0 + h * 0.82;
                        emitBand(wall, a, c, bulge, 0, 1, y0, sill, wallCol, tol, faceOut);
                        emitBand(wall, a, c, bulge, 0, 1, head, y1, wallCol, tol, faceOut);
                        emitBand(part(acc, PartId::Glass), a, c, bulge, 0, 1,
                                 sill, head, Vec3(0.42, 0.52, 0.60), tol, faceOut);
                    } else {
                        emitBand(wall, a, c, bulge, 0, 1, y0, y1, wallCol, tol, faceOut);
                    }
                    return;
                }

                // Punched: the openings for this wall, on this floor.
                const std::vector<Opening> holes =
                    fenestrate(S.grids[li][gi], role, ms.window, floorInTier,
                               ground && tag == EdgeTag::Street);
                Real cursor = 0;
                for (const Opening& o : holes) {
                    const Real o0 = std::max(Real(0), std::min(Real(1), o.t0));
                    const Real o1 = std::max(o0, std::min(Real(1), o.t1));
                    const Real sill = std::min(y1, y0 + o.sill);
                    const Real head = std::min(y1, y0 + std::max(o.sill, o.head));
                    if (o1 <= o0 + 1e-5 || head <= sill + 1e-5) continue;
                    // The pier before the opening, then the spandrel under it
                    // and the lintel over it.
                    emitBand(wall, a, c, bulge, cursor, o0, y0, y1, wallCol, tol, faceOut);
                    emitBand(wall, a, c, bulge, o0, o1, y0, sill, wallCol, tol, faceOut);
                    emitBand(wall, a, c, bulge, o0, o1, head, y1, wallCol, tol, faceOut);
                    // The glass, set back in its reveal so the wall reads as
                    // having thickness.
                    const Vec2 g0 = arcPoint(a, c, bulge, o0);
                    const Vec2 g1 = arcPoint(a, c, bulge, o1);
                    Vec3 n3 = outward3(g0, g1);
                    if (!faceOut) n3 = Vec3(-n3.x, -n3.y, -n3.z);
                    const Vec2 back(-n3.x * params.reveal, -n3.z * params.reveal);
                    const bool door = o.kind == OpeningKind::Door;
                    RenderMesh& glass =
                        part(acc, door ? PartId::Door : PartId::Glass);
                    const Vec3 col = door ? ms.trimColor : Vec3(0.40, 0.50, 0.58);
                    MeshBuilder::emitQuad(glass, at3(g0 + back, sill),
                                          at3(g1 + back, sill), at3(g1 + back, head),
                                          at3(g0 + back, head), n3, col);
                    // The reveal itself: jambs, sill and head. Cheap, and it is
                    // what makes a window look punched rather than painted on.
                    RenderMesh& trim = part(acc, ms.trim);
                    MeshBuilder::emitQuad(trim, at3(g0, sill), at3(g0 + back, sill),
                                          at3(g0 + back, head), at3(g0, head),
                                          Vec3(-n3.z, 0, n3.x), ms.trimColor);
                    MeshBuilder::emitQuad(trim, at3(g1, sill), at3(g1 + back, sill),
                                          at3(g1 + back, head), at3(g1, head),
                                          Vec3(n3.z, 0, -n3.x), ms.trimColor);
                    MeshBuilder::emitQuad(trim, at3(g0, sill), at3(g1, sill),
                                          at3(g1 + back, sill), at3(g0 + back, sill),
                                          Vec3(0, 1, 0), ms.trimColor);
                    MeshBuilder::emitQuad(trim, at3(g0, head), at3(g1, head),
                                          at3(g1 + back, head), at3(g0 + back, head),
                                          Vec3(0, -1, 0), ms.trimColor);
                    cursor = o1;
                }
                emitBand(wall, a, c, bulge, cursor, 1, y0, y1, wallCol, tol, faceOut);
        };

        for (std::size_t pi = 0; pi < L.plans.size(); ++pi) {
            const Shape2& plan = L.plans[pi];
            if (plan.outer.size() < 3) {
                // Still consume this plan's indices, or every later plan and
                // hole reads someone else's grid.
                gridIdx += plan.outer.size();
                for (const Loop2& h : plan.holes) gridIdx += h.size();
                continue;
            }

            // --- walls ------------------------------------------------------
            for (std::size_t e = 0; e < plan.outer.size(); ++e, ++gridIdx)
                emitWall(plan.outer.start(e), plan.outer.end(e), plan.outer.bulge(e),
                         plan.outer.edges[e].tag, gridIdx, true);

            // --- court walls ------------------------------------------------
            // A hole in the plan is a real courtyard: the same wall, looked at
            // from the inside, with the same windows the street facade gets.
            for (const Loop2& h : plan.holes)
                for (std::size_t e = 0; e < h.size(); ++e, ++gridIdx)
                    emitWall(h.start(e), h.end(e), h.bulge(e), h.edges[e].tag,
                             gridIdx, false);

            // --- slabs ------------------------------------------------------
            // The part of this plate that nothing above covers is a roof or a
            // terrace; the part of it that nothing below carries is a soffit.
            // Expressed as booleans rather than as "is this the top floor",
            // which is what makes a setback terrace and a cantilever fall out of
            // the same two lines.
            std::vector<Shape2> above, below;
            if (li + 1 < S.levels.size())
                for (const Shape2& s : S.levels[li + 1].plans) above.push_back(s);
            if (li > 0)
                for (const Shape2& s : S.levels[li - 1].plans) below.push_back(s);

            RenderMesh& roof = part(acc, ms.roof);
            const std::vector<Shape2> cap =
                above.empty() ? std::vector<Shape2>{plan}
                              : shapeBool({plan}, above, BoolOp::Subtract);
            for (const Shape2& s : cap) emitSlab(roof, s, y1, true, ms.roofColor, tol);

            const std::vector<Shape2> soffit =
                below.empty() ? std::vector<Shape2>{plan}
                              : shapeBool({plan}, below, BoolOp::Subtract);
            RenderMesh& base = part(acc, li == 0 ? ms.base : ms.wall);
            for (const Shape2& s : soffit)
                emitSlab(base, s, y0, false,
                         li == 0 ? ms.accentColor : ms.wallColor, tol);

            // --- the court FLOOR --------------------------------------------
            // A courtyard is a hole in every plate of the stack, so without this
            // it is a shaft you see the sky through from below and the terrain
            // through from above — an absence rather than a place. The ground
            // floor's holes get a paved plate, laid at the same level the
            // building's own ground sits on.
            if (li == 0)
                for (const Loop2& h : plan.holes) {
                    if (h.size() < 3) continue;
                    Shape2 court;
                    court.outer = h;
                    orient(court);
                    if (area(court) < 1.0) continue;
                    emitSlab(part(acc, PartId::Path), court, y0, true,
                             ms.accentColor, tol);
                }
        }
    }

    // --- attached elements --------------------------------------------------
    // Everything that is not an opening. Each is a band swept around the level
    // it belongs to, which is the shape all four of these actually are.
    if (params.detail == FacadeDetail::Full) {
        for (const ElementPlacement& p : b.placements) {
            if (p.level < 0 || p.level >= static_cast<int>(S.levels.size())) continue;
            const Level& L = S.levels[p.level];
            if (L.plans.empty()) continue;
            const MaterialSet& ms = setFor(b, L.tier);
            const Real y0 = params.baseY + L.y0, y1 = params.baseY + L.y1;
            switch (p.kind) {
                case ElementKind::Cornice:
                    emitSweptBand(acc, L.plans[0].outer, y1 - 0.32,
                                  std::max(Real(0.28), p.height),
                                  std::max(Real(0.18), p.depth), ms.trim,
                                  ms.trimColor, tol);
                    break;
                case ElementKind::BaseCourse:
                    emitSweptBand(acc, L.plans[0].outer, y0,
                                  std::max(Real(0.6), p.height), 0.10, ms.base,
                                  ms.accentColor, tol);
                    break;
                case ElementKind::Parapet:
                    emitParapetRing(acc, L.plans[0].outer, y1,
                                    std::max(Real(0.9), p.height), ms.wall,
                                    ms.wallColor, ms.trim, ms.trimColor, tol);
                    break;
                case ElementKind::RoofDeck: {
                    // style 1 = a CHIMNEY, which is what addHouseDressing asks
                    // for and the single most-requested element the mesher could
                    // not build. A house without one reads as a box with a lid;
                    // it is also the only thing standing on most residential
                    // roofs, since roof plant is for buildings with lifts.
                    if (p.style != 1) break;
                    const Poly2 ring = tessellate(L.plans[0].outer, 0.3);
                    if (ring.size() < 3) break;
                    const OBB2 box = orientedBoundingBox(ring);
                    // Seated off-centre along the ridge, on the plan — a stack
                    // through the middle of the roof is where nobody builds one.
                    Rng cr(params.seed ^ 0x7f4a7c15u);
                    const int longAxis = box.half[0] >= box.half[1] ? 0 : 1;
                    const Vec2 along = box.axis[longAxis];
                    const Vec2 across = box.axis[1 - longAxis];
                    const Real w = std::min(Real(0.9), box.half[1 - longAxis] * 0.5);
                    if (w < 0.25) break;
                    const Vec2 c = box.center +
                                   along * (box.half[longAxis] * cr.range(-0.55, 0.55));
                    if (!pointInPolygon(ring, c)) break;
                    const Real h = cr.range(1.1, 1.8);
                    const Vec2 o = c - along * (w * 0.5) - across * (w * 0.5);
                    emitPlantBox(part(acc, ms.base), o, along, across, w, w, y1,
                                 y1 + h, ms.accentColor * 0.8);
                    // The cap: a thin oversailing slab, which is what makes a
                    // stack read as a chimney rather than as a post.
                    emitPlantBox(part(acc, ms.trim), o - along * 0.08 - across * 0.08,
                                 along, across, w + 0.16, w + 0.16, y1 + h,
                                 y1 + h + 0.12, ms.trimColor * 0.7);
                    break;
                }
                case ElementKind::RoofPlant: {
                    // A flat roof is not empty: the stair overrun, the tank and
                    // the air handling are what stop a building reading as an
                    // extruded box. The ARRANGEMENT comes from the shared
                    // planner (roof_plant.h) that the shipping grammar also
                    // uses, so the two pipelines cannot drift about how a roof
                    // is laid out; only the drawing below is ours.
                    const Poly2 ring = tessellate(L.plans[0].outer, 0.25);
                    if (ring.size() < 3) break;
                    const OBB2 box = orientedBoundingBox(ring);
                    const Vec2 u = box.axis[0], v = box.axis[1];
                    const Real bw = box.half[0] * 2, bd = box.half[1] * 2;
                    const Vec2 origin = box.center - u * box.half[0] - v * box.half[1];
                    Rng rng(params.seed ^ 0x9E3779B9u);
                    // style 1 = a SEALED facade (a glass curtain-wall tower),
                    // which the planner reads as "no timber water tank". The
                    // recipe declares that; the mesher does not infer it from
                    // window widths, which would be a guess dressed as a rule.
                    const std::vector<RoofPlantItem> plant = planRoofPlant(
                        &ring, origin, u, v, bw, bd,
                        static_cast<int>(S.levels.size()), p.style == 1,
                        [&rng] { return rng.unit(); });
                    for (const RoofPlantItem& it : plant) {
                        const Vec2 o = origin + u * it.u + v * it.v;
                        const Vec2 c = o + u * (it.w * 0.5) + v * (it.d * 0.5);
                        switch (it.kind) {
                            case RoofPlantItem::Kind::Access: {
                                const Real h = 3.0;
                                emitPlantBox(part(acc, ms.trim), o, u, v, it.w, it.d,
                                             y1, y1 + h, ms.trimColor * 0.85);
                                // The capped lid slab that reads as a roof over
                                // the overrun rather than an open shaft.
                                emitPlantBox(part(acc, ms.trim),
                                             o - u * 0.15 - v * 0.15, u, v,
                                             it.w + 0.3, it.d + 0.3, y1 + h,
                                             y1 + h + 0.14, ms.trimColor * 0.6);
                                break;
                            }
                            case RoofPlantItem::Kind::WaterTank: {
                                const Real tr = std::max(Real(0.9),
                                                         (std::min(it.w, it.d) - 1.0) * 0.5);
                                const Real legH = 2.0, th = 3.0;
                                const Vec3 woodCol(0.42, 0.31, 0.22);
                                for (int lx = -1; lx <= 1; lx += 2)
                                    for (int lz = -1; lz <= 1; lz += 2) {
                                        const Vec2 lc = c + u * (lx * tr * 0.7) +
                                                        v * (lz * tr * 0.7);
                                        emitPlantBox(part(acc, PartId::Wood),
                                                     lc - u * 0.08 - v * 0.08, u, v,
                                                     0.16, 0.16, y1, y1 + legH,
                                                     woodCol * 0.85);
                                    }
                                emitPlantTube(part(acc, PartId::Wood),
                                              part(acc, PartId::Wood), c, tr,
                                              y1 + legH, y1 + legH + th, 14,
                                              woodCol, woodCol * 1.05);
                                break;
                            }
                            case RoofPlantItem::Kind::Hvac: {
                                const Real ah = 1.8;
                                const Vec3 casing(0.55, 0.57, 0.60);
                                emitPlantBox(part(acc, PartId::Utility), o, u, v,
                                             it.w, it.d, y1, y1 + ah, casing);
                                const Real frad =
                                    std::min(Real(0.6), std::min(it.w, it.d) * 0.22);
                                emitPlantTube(part(acc, PartId::Utility),
                                              part(acc, PartId::Fan), c, frad,
                                              y1 + ah, y1 + ah + 0.22, 12,
                                              casing * 0.9, Vec3(0.16, 0.17, 0.18));
                                break;
                            }
                        }
                    }
                    break;
                }
                case ElementKind::Balcony: {
                    // A tray on one bay: floor, front and two returns.
                    if (p.edge < 0 || p.edge >= static_cast<int>(L.plans[0].outer.size()))
                        break;
                    const Loop2& l = L.plans[0].outer;
                    const Vec2 a = l.start(p.edge), c = l.end(p.edge);
                    const Real bulge = l.bulge(p.edge);
                    const Vec2 q0 = arcPoint(a, c, bulge, p.t0);
                    const Vec2 q1 = arcPoint(a, c, bulge, p.t1);
                    const Vec3 n3 = outward3(q0, q1);
                    const Real d = std::max(Real(0.9), p.depth);
                    const Vec2 off(n3.x * d, n3.z * d);
                    RenderMesh& m = part(acc, ms.accent);
                    const Real fy = y0 + 0.06;
                    MeshBuilder::emitQuad(m, at3(q0, fy), at3(q1, fy),
                                          at3(q1 + off, fy), at3(q0 + off, fy),
                                          Vec3(0, 1, 0), ms.accentColor);
                    MeshBuilder::emitQuad(m, at3(q0 + off, fy), at3(q1 + off, fy),
                                          at3(q1 + off, fy + 1.05),
                                          at3(q0 + off, fy + 1.05), n3, ms.accentColor);
                    break;
                }
                case ElementKind::Opening:
                    // Handled, but not here: an Opening placement is the
                    // recipe's PERMISSION to fenestrate a wall, read by
                    // `fenestrated()` while the wall is built. Not a gap.
                    break;
                default:
                    // NOT SILENTLY. Fifteen of the nineteen declared kinds fell
                    // through here, so a recipe could ask for a portico, a
                    // steeple or a shopfront sign and get nothing back with
                    // nothing to say it was dropped — which is how a whole roof
                    // kit stayed lost across a pipeline (docs §18). A registry
                    // that advertises what it cannot build must at least admit
                    // it; the shipping grammar HAS emitters for most of these
                    // and they are the next thing to extract.
                    unbuiltKinds[static_cast<std::size_t>(p.kind)]++;
                    break;
            }
        }
    }

    for (std::size_t k = 0; k < unbuiltKinds.size(); ++k)
        if (unbuiltKinds[k])
            LOG_WARN << "[lotmesh] " << unbuiltKinds[k] << " x "
                     << elementKindName(static_cast<ElementKind>(k))
                     << " requested by the recipe but this mesher has no emitter "
                        "for it — the element is silently missing from the building";

    flatten(acc, out);
    out.height = top - params.baseY;

    // --- the HLOD mass ------------------------------------------------------
    // The ground plate extruded to full height: an order of magnitude cheaper
    // than the detailed parts and still the building's actual footprint, which
    // a bounding box would not be.
    if (params.proxy && !S.levels.empty() && !S.levels[0].plans.empty()) {
        const Shape2& foot = S.levels[0].plans[0];
        out.proxy.materialIndex = static_cast<int>(setFor(b, 0).wall);
        const Vec3 col = setFor(b, 0).wallColor;
        for (std::size_t e = 0; e < foot.outer.size(); ++e)
            emitBand(out.proxy, foot.outer.start(e), foot.outer.end(e),
                     foot.outer.bulge(e), 0, 1, params.baseY, top, col, 0.6);
        emitSlab(out.proxy, foot, top, true, col, 0.6);
    }
    return out;
}

// ---------------------------------------------------------------------------
// The ground
// ---------------------------------------------------------------------------

namespace {

// What a zone is made of. The site layer decides WHERE things are; this is the
// one place that decides what they look like, so a path is the same grey on
// every lot in the city.
void zoneLook(Zone z, PartId& part, Vec3& color) {
    switch (z) {
        case Zone::Frontage:    part = PartId::Path;     color = Vec3(0.70, 0.68, 0.64); break;
        case Zone::Circulation: part = PartId::Path;     color = Vec3(0.62, 0.60, 0.58); break;
        case Zone::Parking:     part = PartId::Ground;   color = Vec3(0.28, 0.28, 0.30); break;
        case Zone::Service:     part = PartId::Concrete; color = Vec3(0.46, 0.45, 0.44); break;
        case Zone::Open:        part = PartId::Foliage;  color = Vec3(0.30, 0.45, 0.24); break;
        case Zone::Building:    part = PartId::Concrete; color = Vec3(0.42, 0.41, 0.40); break;
    }
}

}  // namespace

void meshSiteInto(BuildingMesh& out, const SitePlan& site,
                  const SiteFurnishing& furn, Real y) {
    PartAcc acc;
    for (const ZoneArea& z : site.zones) {
        // The building's own footprint is covered by the building; paving it as
        // well would z-fight with the ground slab.
        if (z.zone == Zone::Building) continue;
        PartId id = PartId::Ground;
        Vec3 color(0.5, 0.5, 0.5);
        zoneLook(z.zone, id, color);
        RenderMesh& mesh = part(acc, id);
        for (const Shape2& s : z.parts)
            emitSlab(mesh, s, y + 0.02, true, color, 0.15);
    }

    // Boundaries: a fence, wall or hedge run is a thin upright slab. Height and
    // kind come from the site layer; only the look is decided here.
    for (const BoundaryRun& r : furn.boundaries) {
        const bool hedge = r.kind == BoundaryKind::Hedge;
        const PartId id = hedge ? PartId::Foliage
                                : (r.kind == BoundaryKind::Wall ? PartId::Concrete
                                                                : PartId::Wood);
        const Vec3 col = hedge ? Vec3(0.24, 0.40, 0.22) : Vec3(0.55, 0.50, 0.44);
        const Real half = hedge ? 0.35 : 0.06;
        RenderMesh& mesh = part(acc, id);
        const Vec2 d = r.b - r.a;
        const Real len = d.length();
        if (len < 0.2) continue;
        const Vec2 n(d.y / len * half, -d.x / len * half);
        const Vec2 c0 = r.a + n, c1 = r.b + n, c2 = r.b - n, c3 = r.a - n;
        const Real top = y + r.height;
        MeshBuilder::emitQuad(mesh, at3(c0, y), at3(c1, y), at3(c1, top), at3(c0, top),
                              Vec3(n.x / half, 0, n.y / half), col);
        MeshBuilder::emitQuad(mesh, at3(c2, y), at3(c3, y), at3(c3, top), at3(c2, top),
                              Vec3(-n.x / half, 0, -n.y / half), col);
        MeshBuilder::emitQuad(mesh, at3(c0, top), at3(c1, top), at3(c2, top),
                              at3(c3, top), Vec3(0, 1, 0), col);
    }
    flatten(acc, out);
}

}  // namespace engine
