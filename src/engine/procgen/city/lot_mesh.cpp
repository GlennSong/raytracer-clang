#include "lot_mesh.h"

#include "shape_ops.h"
#include "../../mesh_builder.h"
#include "triangulate.h"
#include "rng.h"
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

// A projecting band swept around a loop — a cornice, a base course, a parapet.
// Three quads per edge: the fascia, the top and the soffit, so it reads as a
// real moulding from above and below rather than as a painted line.
void emitSweptBand(PartAcc& acc, const Loop2& loop, Real y, Real height,
                   Real project, PartId id, const Vec3& color, Real chordTol) {
    if (loop.size() < 3 || height <= 1e-4) return;
    RenderMesh& mesh = part(acc, id);
    const std::size_t n = loop.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = loop.start(i), b = loop.end(i);
        const Real bulge = loop.bulge(i);
        const int steps = arcSteps(a, b, bulge, 1.0, chordTol);
        Vec2 prev = a;
        for (int s = 1; s <= steps; ++s) {
            const Vec2 cur = arcPoint(a, b, bulge, static_cast<Real>(s) / steps);
            const Vec3 n3 = outward3(prev, cur);
            const Vec2 off(n3.x * project, n3.z * project);
            const Vec2 p0 = prev + off, p1 = cur + off;
            MeshBuilder::emitQuad(mesh, at3(p0, y), at3(p1, y), at3(p1, y + height),
                                  at3(p0, y + height), n3, color);           // fascia
            MeshBuilder::emitQuad(mesh, at3(prev, y + height), at3(cur, y + height),
                                  at3(p1, y + height), at3(p0, y + height),
                                  Vec3(0, 1, 0), color);                     // top
            if (project > 1e-4)
                MeshBuilder::emitQuad(mesh, at3(prev, y), at3(cur, y), at3(p1, y),
                                      at3(p0, y), Vec3(0, -1, 0), color);    // soffit
            prev = cur;
        }
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

        for (std::size_t pi = 0; pi < L.plans.size(); ++pi) {
            const Shape2& plan = L.plans[pi];
            if (plan.outer.size() < 3) continue;

            // --- walls ------------------------------------------------------
            for (std::size_t e = 0; e < plan.outer.size(); ++e) {
                const Vec2 a = plan.outer.start(e), c = plan.outer.end(e);
                const Real bulge = plan.outer.bulge(e);
                const EdgeTag tag = plan.outer.edges[e].tag;
                const Vec3 wallCol = ms.wallColor;

                // A party wall is blank BY CONSTRUCTION — the payoff of tagging
                // an edge once, at plan time, instead of guessing with a dot
                // product at draw time.
                const WallRole role =
                    roleForTag(tag, floorInTier, params.retail && ground);
                const bool wantGlass =
                    params.detail == FacadeDetail::Full && role != WallRole::Blank &&
                    pi == 0 && e < S.grids[li].size() &&
                    fenestrated(b.placements, static_cast<int>(li), static_cast<int>(e));

                RenderMesh& wall = part(acc, ms.wall);
                if (!wantGlass) {
                    if (params.detail == FacadeDetail::Flat && role != WallRole::Blank) {
                        // FLAT LOD: a spandrel band and a glass band per storey,
                        // read off the same blueprint the full detail punches.
                        // Two quads instead of a dozen, and the windows land in
                        // the same place, so the swap is invisible.
                        const Real h = y1 - y0;
                        const Real sill = y0 + h * 0.30, head = y0 + h * 0.82;
                        emitBand(wall, a, c, bulge, 0, 1, y0, sill, wallCol, tol);
                        emitBand(wall, a, c, bulge, 0, 1, head, y1, wallCol, tol);
                        emitBand(part(acc, PartId::Glass), a, c, bulge, 0, 1,
                                 sill, head, Vec3(0.42, 0.52, 0.60), tol);
                    } else {
                        emitBand(wall, a, c, bulge, 0, 1, y0, y1, wallCol, tol);
                    }
                    continue;
                }

                // Punched: the openings for this wall, on this floor.
                const std::vector<Opening> holes =
                    fenestrate(S.grids[li][e], role, ms.window, floorInTier,
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
                    emitBand(wall, a, c, bulge, cursor, o0, y0, y1, wallCol, tol);
                    emitBand(wall, a, c, bulge, o0, o1, y0, sill, wallCol, tol);
                    emitBand(wall, a, c, bulge, o0, o1, head, y1, wallCol, tol);
                    // The glass, set back in its reveal so the wall reads as
                    // having thickness.
                    const Vec2 g0 = arcPoint(a, c, bulge, o0);
                    const Vec2 g1 = arcPoint(a, c, bulge, o1);
                    const Vec3 n3 = outward3(g0, g1);
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
                emitBand(wall, a, c, bulge, cursor, 1, y0, y1, wallCol, tol);
            }

            // --- court walls ------------------------------------------------
            // A hole in the plan is a real courtyard, and its walls face INWARD.
            for (const Loop2& h : plan.holes) {
                RenderMesh& wall = part(acc, ms.wall);
                for (std::size_t e = 0; e < h.size(); ++e)
                    emitBand(wall, h.start(e), h.end(e), h.bulge(e), 0, 1, y0, y1,
                             ms.wallColor, tol, false);
            }

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
                    emitSweptBand(acc, L.plans[0].outer, y1,
                                  std::max(Real(0.9), p.height), 0.0, ms.wall,
                                  ms.wallColor, tol);
                    break;
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
                default:
                    break;   // porches, steeples, signs: their own emitters, later
            }
        }
    }

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
