#include "shape_grammar.h"

#include "road_mesh.h"            // triangulatePolygon (floorplan roof/slab fill)
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// Deterministic per-building RNG (seeded; ADR-0002). Small xorshift — we only
// need cheap, reproducible jitter, not statistical quality.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }   // [0,1)
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};

int materialIndexFor(PartId id) { return static_cast<int>(id); }

// Indexed access to a Vec3 component (avoids pointer arithmetic across members,
// which is UB under -Wpedantic).
Real axisComp(const Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
void setAxisComp(Vec3& v, int i, Real val) {
    if (i == 0) v.x = val; else if (i == 1) v.y = val; else v.z = val;
}

// A rectangle on a building face: bottom-left corner + in-plane axes + outward
// normal. The facade subdivision works entirely in this 2D frame.
struct FaceRect {
    Vec3 bl;            // bottom-left corner (world)
    Vec3 h;             // unit horizontal axis
    Vec3 v;             // unit vertical axis (== scope up)
    Vec3 n;             // outward normal
    Real width = 0;
    Real height = 0;

    Vec3 at(Real x, Real y) const { return bl + h * x + v * y; }
};

// The four vertical faces of a storey scope, outward-facing.
FaceRect faceOf(const Scope& s, int side) {
    const Vec3 r = s.axis[0], u = s.axis[1], f = s.axis[2];
    const Real W = s.size.x, H = s.size.y, D = s.size.z;
    const Vec3 O = s.origin;
    FaceRect fr;
    fr.v = u; fr.height = H;
    switch (side) {
        case 0: fr.n = f;      fr.bl = O + f * D;          fr.h = r;       fr.width = W; break; // front
        case 1: fr.n = f * -1; fr.bl = O + r * W;          fr.h = r * -1;  fr.width = W; break; // back
        case 2: fr.n = r;      fr.bl = O + r * W + f * D;  fr.h = f * -1;  fr.width = D; break; // right
        default: fr.n = r * -1; fr.bl = O;                 fr.h = f;       fr.width = D; break; // left
    }
    return fr;
}

}  // namespace

Vec3 facadeColor(FacadeStyle style, uint32_t seed) {
    Rng rng(seed ? seed : 1u);
    Real t = rng.unit();
    switch (style) {
        case FacadeStyle::Brick:
            // Warm reds/browns, some buff.
            return lerp(Vec3(0.50, 0.22, 0.16), Vec3(0.62, 0.40, 0.28), t);
        case FacadeStyle::Stucco:
            return lerp(Vec3(0.86, 0.82, 0.72), Vec3(0.80, 0.74, 0.60), t);
        case FacadeStyle::Painted:
            // Muted pastels (residential).
            return lerp(Vec3(0.74, 0.78, 0.78), Vec3(0.80, 0.72, 0.66), t) +
                   Vec3(rng.range(-0.04, 0.04), rng.range(-0.04, 0.04), rng.range(-0.04, 0.04));
        case FacadeStyle::GlassCurtain:
            return lerp(Vec3(0.58, 0.62, 0.66), Vec3(0.66, 0.68, 0.70), t);
        case FacadeStyle::Metal:
            // Cool steel / corrugated siding (industrial).
            return lerp(Vec3(0.46, 0.50, 0.54), Vec3(0.56, 0.58, 0.60), t);
        case FacadeStyle::Concrete:
        default:
            return lerp(Vec3(0.62, 0.62, 0.60), Vec3(0.74, 0.73, 0.70), t);
    }
}

RenderMaterial materialFor(PartId id, const Vec3& wallColor) {
    RenderMaterial m;
    switch (id) {
        case PartId::Glass:
            m.albedo = {0.18, 0.27, 0.34}; m.metallic = 0.9f; m.roughness = 0.08f; break;
        case PartId::Trim:
            m.albedo = wallColor * 0.55; m.metallic = 0.0f; m.roughness = 0.7f; break;
        case PartId::Roof:
            m.albedo = {0.18, 0.18, 0.20}; m.metallic = 0.0f; m.roughness = 0.85f; break;
        case PartId::Door:
            m.albedo = {0.12, 0.12, 0.13}; m.metallic = 0.3f; m.roughness = 0.4f; break;
        case PartId::Ground:
            m.albedo = {0.30, 0.30, 0.32}; m.metallic = 0.0f; m.roughness = 0.9f; break;
        case PartId::Detail:
            m.albedo = wallColor * 0.8; m.metallic = 0.1f; m.roughness = 0.6f; break;
        case PartId::Brick:
            // A wall, but shaded with a world-space procedural material from the
            // library. Albedo stays the wall colour (which rides in vertex colour
            // for the merged city mesh); the shader/tracer add the surface detail.
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.88f;
            m.setSurface(RenderMaterial::Surface::Brick); break;
        case PartId::Concrete:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.92f;
            m.setSurface(RenderMaterial::Surface::Concrete); break;
        case PartId::Stucco:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.85f;
            m.setSurface(RenderMaterial::Surface::Stucco); break;
        case PartId::Metal:
            m.albedo = wallColor; m.metallic = 0.55f; m.roughness = 0.45f;
            m.setSurface(RenderMaterial::Surface::CorrugatedMetal); break;
        case PartId::Wood:
            m.albedo = {0.52, 0.40, 0.27}; m.metallic = 0.0f; m.roughness = 0.85f;
            m.setSurface(RenderMaterial::Surface::WoodSiding); break;
        case PartId::Wall:
        default:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.75f; break;
    }
    return m;
}

// The shape grammar's quad emitter is the engine's winding-aware MeshBuilder
// helper (kept as a free function here so the grammar's many call sites read
// tersely). Centralising the winding rule means the grammar, terrain and roads
// can't drift apart on which way a front face points.
void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& normal, const Vec3& color) {
    MeshBuilder::emitQuad(mesh, a, b, c, d, normal, color);
}

// Append a part's geometry, creating the part lazily and keeping materialIndex.
// NOTE: the returned reference is invalidated by any later partMesh() that grows
// out.parts — use it immediately, never hold it across another partMesh() call.
static RenderMesh& partMesh(BuildingMesh& out, PartId id) {
    for (RenderMesh& p : out.parts)
        if (p.materialIndex == materialIndexFor(id)) return p;
    out.parts.emplace_back();
    RenderMesh& p = out.parts.back();
    p.materialIndex = materialIndexFor(id);
    return p;
}

// Append a whole mesh's geometry into a part (offsetting indices). Safe to call
// in sequence — the part reference is used only for this one append.
static void appendToPart(BuildingMesh& out, PartId id, const RenderMesh& src) {
    if (src.vertices.empty()) return;
    RenderMesh& dst = partMesh(out, id);
    uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

void emitBox(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color) {
    RenderMesh& mesh = partMesh(out, part);
    // Eight corners.
    Vec3 c000 = s.corner(0, 0, 0), c100 = s.corner(1, 0, 0);
    Vec3 c110 = s.corner(1, 1, 0), c010 = s.corner(0, 1, 0);
    Vec3 c001 = s.corner(0, 0, 1), c101 = s.corner(1, 0, 1);
    Vec3 c111 = s.corner(1, 1, 1), c011 = s.corner(0, 1, 1);
    const Vec3 r = s.axis[0], u = s.axis[1], f = s.axis[2];
    emitQuad(mesh, c000, c100, c110, c010, f * -1, color);   // back  (-f)
    emitQuad(mesh, c001, c101, c111, c011, f,      color);   // front (+f)
    emitQuad(mesh, c000, c001, c011, c010, r * -1, color);   // left  (-r)
    emitQuad(mesh, c100, c101, c111, c110, r,      color);   // right (+r)
    emitQuad(mesh, c000, c100, c101, c001, u * -1, color);   // bottom(-u)
    emitQuad(mesh, c010, c110, c111, c011, u,      color);   // top   (+u)
}

void emitShell(BuildingMesh& out, const Scope& s, PartId part, const Vec3& color,
               bool floor, bool ceiling) {
    RenderMesh& mesh = partMesh(out, part);
    for (int side = 0; side < 4; ++side) {
        FaceRect fr = faceOf(s, side);
        emitQuad(mesh, fr.at(0, 0), fr.at(fr.width, 0), fr.at(fr.width, fr.height),
                 fr.at(0, fr.height), fr.n, color);
    }
    const Vec3 u = s.axis[1];
    if (floor)
        emitQuad(mesh, s.corner(0, 0, 0), s.corner(1, 0, 0), s.corner(1, 0, 1),
                 s.corner(0, 0, 1), u, color);
    if (ceiling)
        emitQuad(mesh, s.corner(0, 1, 0), s.corner(1, 1, 0), s.corner(1, 1, 1),
                 s.corner(0, 1, 1), u * -1, color);
}

// A solid parapet: a closed ring of four thin boxes around a footprint
// perimeter, so the roof edge reads as a real wall with thickness from every
// angle (emitShell gives single-sided quads that vanish edge-on / from inside).
// `footOrigin` is the min corner, extents `width` (along r) × `depth` (along f),
// rising `height` from `y`, wall `thick` metres.
void emitParapet(BuildingMesh& out, const Vec3& footOrigin, Real width, Real depth,
                 const Vec3& r, const Vec3& f, Real y, Real height, Real thick,
                 PartId part, const Vec3& color) {
    const Vec3 up(0, 1, 0);
    Real t = std::min(thick, std::min(width, depth) * 0.45);
    Vec3 base(footOrigin.x, y, footOrigin.z);
    // Back (-f) and front (+f) run the full width; left/right fit between them.
    emitBox(out, Scope{base, {r, up, f}, Vec3(width, height, t)}, part, color);
    emitBox(out, Scope{base + f * (depth - t), {r, up, f}, Vec3(width, height, t)},
            part, color);
    emitBox(out, Scope{base + f * t, {r, up, f}, Vec3(t, height, depth - 2 * t)},
            part, color);
    emitBox(out, Scope{base + r * (width - t) + f * t, {r, up, f},
                       Vec3(t, height, depth - 2 * t)}, part, color);
}

std::vector<Scope> splitScope(const Scope& s, int axis, const std::vector<Real>& sizes) {
    std::vector<Scope> out;
    Real total = 0;
    for (Real sz : sizes) total += std::abs(sz);
    Real extent = axisComp(s.size, axis);
    Real cursor = 0;
    for (Real sz : sizes) {
        Real len = (total > 0) ? std::abs(sz) / total * extent : 0;
        Scope child = s;
        child.origin = s.origin + s.axis[axis] * cursor;
        setAxisComp(child.size, axis, len);
        out.push_back(child);
        cursor += len;
    }
    return out;
}

std::vector<Scope> repeatScope(const Scope& s, int axis, Real target) {
    Real extent = axisComp(s.size, axis);
    int n = std::max(1, static_cast<int>(std::lround(extent / std::max(target, Real(0.01)))));
    std::vector<Real> sizes(static_cast<std::size_t>(n), 1.0);
    return splitScope(s, axis, sizes);
}

Scope insetScope(const Scope& s, Real d) {
    Scope o = s;
    Real dx = std::min(d, s.size.x * 0.49);
    Real dz = std::min(d, s.size.z * 0.49);
    o.origin = s.origin + s.axis[0] * dx + s.axis[2] * dz;
    o.size.x = s.size.x - 2 * dx;
    o.size.z = s.size.z - 2 * dz;
    return o;
}

Scope scopeFromFootprint(const Poly2& footprint, Real baseY, Real height,
                         const std::function<bool(const Vec2&)>& cornerOk) {
    OBB2 obb = orientedBoundingBox(footprint);
    // Fit the box INSIDE the footprint: a non-rectangular lot (wedge/trapezoid or
    // corner piece) has an OBB that bulges past the polygon, which would seat the
    // building proud of its lot. Shrink about an interior anchor until all four
    // corners sit inside, so rectangular lots keep full size and skew lots pull in.
    // `cornerOk` folds in the caller's extra constraint (road clearance).
    Vec2 anchor = pointInPolygon(footprint, obb.center) ? obb.center
                                                        : centroid(footprint);
    auto cornersInside = [&](Real s) {
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2) {
                Vec2 c = anchor + obb.axis[0] * (obb.half[0] * s * sx)
                                + obb.axis[1] * (obb.half[1] * s * sy);
                if (!pointInPolygon(footprint, c)) return false;
                if (cornerOk && !cornerOk(c)) return false;
            }
        return true;
    };
    Real fit = 1.0;
    if (!cornersInside(1.0)) {
        Real lo = 0.0, hi = 1.0;
        for (int it = 0; it < 16; ++it) {
            Real mid = (lo + hi) * 0.5;
            (cornersInside(mid) ? lo : hi) = mid;
        }
        fit = lo;
    }
    Scope s;
    // Long OBB axis -> forward (facades face the long sides); short -> right.
    int la = obb.longAxis(), sa = 1 - la;
    Vec2 fwd = obb.axis[la], rgt = obb.axis[sa];
    Real fwdHalf = obb.half[la] * fit, rgtHalf = obb.half[sa] * fit;
    s.axis[0] = normalize(Vec3(rgt.x, 0, rgt.y));
    s.axis[1] = Vec3(0, 1, 0);
    s.axis[2] = normalize(Vec3(fwd.x, 0, fwd.y));
    s.size = Vec3(rgtHalf * 2, height, fwdHalf * 2);
    Vec3 centerXZ(anchor.x, baseY, anchor.y);
    s.origin = centerXZ - s.axis[0] * rgtHalf - s.axis[2] * fwdHalf;
    return s;
}

// --- Facade subdivision -----------------------------------------------------

namespace {

enum class FacadeMode { Residential, Retail, Entrance, Solid };

// A curtain-wall storey (ADR-0040 Pass B): a continuous glass skin, not punched
// windows — an opaque spandrel band hiding the floor slab, vision glass above,
// and a proud steel mullion/transom grid. This is what a glass tower actually
// is (a skin hung off a frame), and it reads far better than flat panels.
void emitCurtainWallRect(BuildingMesh& out, const FaceRect& fr,
                         const Vec3& wallColor) {
    Real fh = fr.height, W = fr.width;
    if (W < 0.5 || fh < 0.5) return;
    RenderMesh glass, mull;
    Vec3 glassCol = materialFor(PartId::Glass, wallColor).albedo;
    Vec3 spandrelCol = glassCol * 0.45;          // opaque shadow-box band
    Vec3 mullCol(0.34, 0.36, 0.40);              // steel mullions
    Real spandrelH = std::min(Real(0.9), fh * 0.30);

    // Glass sits INSET behind the frame plane; the mullion grid is SOLID
    // geometry — front face + side returns back to the glass — so up close it
    // reads as a frame the panes sit in, not a decal (device feedback).
    const Real glassIn = 0.10;                   // glass plane behind the grid
    Vec3 gin = fr.n * (-glassIn);
    emitQuad(glass, fr.at(0, 0) + gin, fr.at(W, 0) + gin,
             fr.at(W, spandrelH) + gin, fr.at(0, spandrelH) + gin,
             fr.n, spandrelCol);                 // spandrel (floor-slab band)
    emitQuad(glass, fr.at(0, spandrelH) + gin, fr.at(W, spandrelH) + gin,
             fr.at(W, fh) + gin, fr.at(0, fh) + gin, fr.n, glassCol);   // vision glass

    const Real proud = 0.06;                     // grid stands proud of the wall
    Vec3 outv = fr.n * proud;
    const Real mw = 0.09;
    // A solid BAR on the facade: front face + both side returns down to the
    // glass plane (vertical bars get left/right cheeks, horizontal get top/
    // bottom), so the lattice has real depth from any angle.
    auto bar = [&](Real a0, Real b0, Real a1, Real b1, bool vertical) {
        emitQuad(mull, fr.at(a0, b0) + outv, fr.at(a1, b0) + outv,
                 fr.at(a1, b1) + outv, fr.at(a0, b1) + outv, fr.n, mullCol);
        if (vertical) {
            emitQuad(mull, fr.at(a0, b0) + gin, fr.at(a0, b0) + outv,
                     fr.at(a0, b1) + outv, fr.at(a0, b1) + gin, fr.h * -1, mullCol);
            emitQuad(mull, fr.at(a1, b0) + gin, fr.at(a1, b0) + outv,
                     fr.at(a1, b1) + outv, fr.at(a1, b1) + gin, fr.h, mullCol);
        } else {
            emitQuad(mull, fr.at(a0, b1) + gin, fr.at(a1, b1) + gin,
                     fr.at(a1, b1) + outv, fr.at(a0, b1) + outv, fr.v, mullCol);
            emitQuad(mull, fr.at(a0, b0) + outv, fr.at(a1, b0) + outv,
                     fr.at(a1, b0) + gin, fr.at(a0, b0) + gin, fr.v * -1, mullCol);
        }
    };
    int bays = std::max(1, static_cast<int>(std::lround(W / 1.6)));
    for (int b = 0; b <= bays; ++b) {            // vertical mullions
        Real x = std::min(std::max(b * W / bays, mw * 0.5), W - mw * 0.5);
        bar(x - mw * 0.5, 0, x + mw * 0.5, fh, true);
    }
    for (Real ty : {spandrelH, fh - 0.04}) {     // transoms (spandrel line + head)
        Real t0 = std::max(Real(0), ty - mw * 0.5), t1 = std::min(fh, ty + mw * 0.5);
        bar(0, t0, W, t1, false);
    }
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::Detail, mull);     // mullions read as metal detail
}
void emitCurtainWall(BuildingMesh& out, const Scope& storey, int side,
                     const Vec3& wallColor) {
    emitCurtainWallRect(out, faceOf(storey, side), wallColor);
}

// Subdivide one face into window bays. Wall margins around each window read as
// mullions/piers; the window is recessed by `inset` (Glass). In Entrance mode the
// centre bay is a real door-height opening (no fill) so the shell is enterable.
// Works on a bare FaceRect so BOTH massing paths share it: the box grammar
// (faceOf a storey scope) and the floorplan grammar (one rect per plan edge).
void emitFacadeRect(BuildingMesh& out, const FaceRect& fr, FacadeMode mode,
                    const BuildingParams& p, const Vec3& wallColor) {
    // Accumulate into locals, then append once each — never hold a part reference
    // across a partMesh() that could reallocate out.parts.
    // surround = sill/hood trim courses; frame = window frames + muntin lights.
    RenderMesh wall, glass, door, surround, frame;

    int bays = std::max(1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    Real bw = fr.width / bays;
    Real fh = fr.height;

    Real sill, head, margin;
    // The entrance face's non-door bays match the OTHER ground faces (retail
    // storefronts when groundRetail) — the front used to wear small residential
    // windows while the other three sides had tall shopfronts (device feedback).
    const bool retailish = (mode == FacadeMode::Retail) ||
                           (mode == FacadeMode::Entrance && p.groundRetail);
    if (mode == FacadeMode::Solid) {           // warehouse: small high clerestory
        sill = fh * 0.66; head = fh * 0.84; margin = std::min(bw * 0.36, Real(1.4));
    } else {
        sill = retailish ? 0.4 : human::WINDOW_SILL;
        head = std::min(fh - 0.4, retailish ? fh - 0.4 : human::WINDOW_HEAD);
        // A CONSTANT window module across every face (the piers absorb the slack),
        // so a wide face and a narrow one show the same window size, not different
        // ones (ADR-0040). Width is the bay minus piers, clamped.
        Real winW = std::min(Real(1.6), std::max(Real(0.8), bw - 0.8));
        margin = (bw - winW) * 0.5;
    }
    if (head <= sill) { head = fh * 0.75; sill = fh * 0.2; }

    int centreBay = bays / 2;
    for (int b = 0; b < bays; ++b) {
        Real x0 = b * bw, x1 = (b + 1) * bw;
        bool entrance = (mode == FacadeMode::Entrance && b == centreBay);

        Real wx0 = x0 + margin, wx1 = x1 - margin;          // window/opening span
        Real openSill = entrance ? 0.0 : sill;
        Real openHead = entrance ? std::min(human::DOOR_HEIGHT, fh - 0.3) : head;
        if (entrance) {
            Real dw = std::min(human::DOOR_WIDTH, bw - 0.4);
            Real cx = (x0 + x1) * 0.5;
            wx0 = cx - dw * 0.5; wx1 = cx + dw * 0.5;
        }

        // Wall surround: bottom band, top band, left pier, right pier.
        auto wallQuad = [&](Real a0, Real a1, Real b0, Real b1) {
            if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
            emitQuad(wall, fr.at(a0, b0), fr.at(a1, b0), fr.at(a1, b1), fr.at(a0, b1),
                     fr.n, wallColor);
        };
        // The opening ELEMENT (building-grammar-plan.md P2): its head may be an
        // ARCH — a real arc cut into the wall, not a square hole. Arches apply
        // to punched residential windows only (storefronts, clerestories and
        // doors stay flat) and only when the opening can carry the rise.
        OpeningStyle st = p.window;
        if (entrance || mode != FacadeMode::Residential)
            st.head = OpeningStyle::Head::Flat;
        const Real span = wx1 - wx0;
        Real rise = 0;
        if (st.head == OpeningStyle::Head::Round) rise = span * 0.5;
        else if (st.head == OpeningStyle::Head::Segmental)
            rise = span * std::min(Real(0.5), std::max(Real(0.12), st.archRise));
        if (rise > 0 && openHead - rise < openSill + 0.35) rise = 0;   // too squat
        const Real ysp = openHead - rise;                    // springline
        const Real cx = (wx0 + wx1) * 0.5;
        // Arc samples, left springer -> right springer (face space).
        const int NARC = 8;
        Real R = 0, Cy = 0;
        Vec2 arc[NARC + 1];
        if (rise > 0) {
            R = (rise * rise + span * span * 0.25) / (2 * rise);
            Cy = openHead - R;
            const Real thL = std::atan2(ysp - Cy, wx0 - cx);
            const Real thR = std::atan2(ysp - Cy, wx1 - cx);
            for (int k = 0; k <= NARC; ++k) {
                Real th = thL + (thR - thL) * (Real(k) / NARC);
                arc[k] = Vec2(cx + R * std::cos(th), Cy + R * std::sin(th));
            }
        }

        // Wall surround: below, piers to the springline, above the apex — and
        // for an arch, the SPANDRELS between the arc and the apex line.
        wallQuad(x0, x1, 0, openSill);                       // below opening
        wallQuad(x0, x1, openHead, fh);                      // above apex
        wallQuad(x0, wx0, openSill, ysp);                    // left pier
        wallQuad(wx1, x1, openSill, ysp);                    // right pier
        if (rise > 0) {
            wallQuad(x0, wx0, ysp, openHead);                // pier strips beside the arch
            wallQuad(wx1, x1, ysp, openHead);
            for (int k = 0; k < NARC; ++k) {                 // spandrel fill over the arc
                // At the APEX one arc point touches the openHead line — a quad
                // there has a degenerate first triangle, which breaks emitQuad's
                // winding pick (the zero-area cross can't vote). Emit the
                // surviving piece as a triangle instead.
                const Real h0 = openHead - arc[k].y, h1 = openHead - arc[k + 1].y;
                Vec3 A = fr.at(arc[k].x, arc[k].y), B = fr.at(arc[k + 1].x, arc[k + 1].y);
                Vec3 TB = fr.at(arc[k + 1].x, openHead), TA = fr.at(arc[k].x, openHead);
                if (h0 < 1e-4 && h1 < 1e-4) continue;
                if (h1 < 1e-4)      MeshBuilder::emitTri(wall, A, B, TA, fr.n, wallColor);
                else if (h0 < 1e-4) MeshBuilder::emitTri(wall, A, B, TB, fr.n, wallColor);
                else                emitQuad(wall, A, B, TB, TA, fr.n, wallColor);
            }
        }

        if (entrance) {
            // The DOOR element: a recessed doorway, closed like the windows —
            // jambs + lintel + threshold connect the wall opening back to the
            // door leaf, so you can't see through the gap into the hollow shell.
            Vec3 in = fr.n * -0.18;
            Vec3 oBL = fr.at(wx0, 0), oBR = fr.at(wx1, 0);
            Vec3 oTL = fr.at(wx0, openHead), oTR = fr.at(wx1, openHead);
            Vec3 dBL = oBL + in, dBR = oBR + in, dTL = oTL + in, dTR = oTR + in;
            Vec3 rev = wallColor * 0.7;
            emitQuad(wall, oTL, oTR, dTR, dTL, fr.v * -1, rev);   // lintel (faces down)
            emitQuad(wall, oBL, oBR, dBR, dBL, fr.v, rev);        // threshold (faces up)
            emitQuad(wall, oBL, oTL, dTL, dBL, fr.h, rev);        // left jamb
            emitQuad(wall, oBR, oTR, dTR, dBR, fr.h * -1, rev);   // right jamb
            emitQuad(door, dBL, dBR, dTR, dTL, fr.n,
                     materialFor(PartId::Door, wallColor).albedo);   // the door leaf
            // DOORFRAME (device feedback): painted stiles + head rail seated in
            // the recess around the leaf — the same joinery the windows wear.
            {
                const Vec3 fp = fr.n * -0.09;
                const Real dfw = 0.10;
                auto dfQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                    if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
                    emitQuad(frame, fr.at(a0, b0) + fp, fr.at(a1, b0) + fp,
                             fr.at(a1, b1) + fp, fr.at(a0, b1) + fp,
                             fr.n, p.window.frameColor);
                };
                dfQuad(wx0, 0, wx0 + dfw, openHead);              // left stile
                dfQuad(wx1 - dfw, 0, wx1, openHead);              // right stile
                dfQuad(wx0 + dfw, openHead - dfw, wx1 - dfw, openHead);   // head rail
            }
            // ARCHITRAVE: a proud trim surround on the wall face around the
            // opening — jamb casings + a head band.
            {
                RenderMesh& srd = surround;
                auto caseQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                    Vec3 ov = fr.n * 0.05;
                    emitQuad(srd, fr.at(a0, b0) + ov, fr.at(a1, b0) + ov,
                             fr.at(a1, b1) + ov, fr.at(a0, b1) + ov, fr.n, p.trimColor);
                };
                const Real cw = 0.12;
                caseQuad(wx0 - cw, 0, wx0, openHead + cw);        // left casing
                caseQuad(wx1, 0, wx1 + cw, openHead + cw);        // right casing
                caseQuad(wx0, openHead, wx1, openHead + cw);      // head band
            }
            // AWNING over the DOOR (device: it was centred on the face, not the
            // door — it belongs to the door grammar): a projecting ledge just
            // above the opening, spanning a little wider than the leaf.
            if (p.awning) {
                const Real aw = std::min((wx1 - wx0) + 1.2, fr.width - 0.4);
                const Real ac = (wx0 + wx1) * 0.5;
                Vec3 c0 = fr.at(ac - aw * 0.5, openHead + 0.22);
                Vec3 across = normalize(fr.h);
                Scope a;
                a.axis[0] = across; a.axis[1] = Vec3(0, 1, 0); a.axis[2] = fr.n;
                a.size = Vec3(aw, 0.16, 1.25);
                a.origin = c0;
                emitBox(out, a, PartId::Detail, p.trimColor);
            }
            // The entrance attach point sits at the DOOR's foot (not the face
            // centre — off by half a bay on even bay counts).
            out.attaches.push_back({fr.at((wx0 + wx1) * 0.5, 0), fr.n, "entrance"});
        } else {
            const Vec3 in = fr.n * (-p.windowInset);
            const Vec3 rev = wallColor * 0.82;
            const Vec3 gcol = materialFor(PartId::Glass, wallColor).albedo;
            // Reveals: close the recess between the wall opening and the inset
            // glass — sill, jambs to the springline, then a flat lintel or the
            // arc SOFFIT (per-segment quads whose normals point at the arc
            // centre, so the underside of the arch shades correctly).
            Vec3 oBL = fr.at(wx0, openSill), oBR = fr.at(wx1, openSill);
            Vec3 oTL = fr.at(wx0, ysp), oTR = fr.at(wx1, ysp);
            emitQuad(wall, oBL, oBR, oBR + in, oBL + in, fr.v, rev);      // sill reveal
            emitQuad(wall, oBL, oTL, oTL + in, oBL + in, fr.h, rev);      // left jamb
            emitQuad(wall, oBR, oTR, oTR + in, oBR + in, fr.h * -1, rev); // right jamb
            if (rise <= 0) {
                emitQuad(wall, oTL, oTR, oTR + in, oTL + in, fr.v * -1, rev);   // lintel
            } else {
                for (int k = 0; k < NARC; ++k) {
                    Vec3 A = fr.at(arc[k].x, arc[k].y), B = fr.at(arc[k + 1].x, arc[k + 1].y);
                    Real mx = (arc[k].x + arc[k + 1].x) * 0.5;
                    Real my = (arc[k].y + arc[k + 1].y) * 0.5;
                    Vec3 nrm = normalize(fr.h * (cx - mx) + fr.v * (Cy - my));
                    emitQuad(wall, A, B, B + in, A + in, nrm, rev);       // arc soffit
                }
            }

            // FRAME: a painted border seated partway into the reveal — the glass
            // sits INSIDE it (device: "a frame around it and then in that frame
            // sits the actual window"). Rails + stiles, and on an arch a curved
            // head rail following the arc. Muntins split the frame into lights.
            const Vec3 fp = fr.n * (-p.windowInset * 0.45);
            const Real fw = std::max(Real(0.04), st.frameWidth);
            auto frameQuad = [&](Real a0, Real b0, Real a1, Real b1) {
                if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
                emitQuad(frame, fr.at(a0, b0) + fp, fr.at(a1, b0) + fp,
                         fr.at(a1, b1) + fp, fr.at(a0, b1) + fp, fr.n, st.frameColor);
            };
            frameQuad(wx0, openSill, wx1, openSill + fw);                 // bottom rail
            frameQuad(wx0, openSill + fw, wx0 + fw, ysp);                 // left stile
            frameQuad(wx1 - fw, openSill + fw, wx1, ysp);                 // right stile
            if (rise <= 0) {
                frameQuad(wx0 + fw, ysp - fw, wx1 - fw, ysp);             // top rail
            } else {
                const Real innerK = (R - fw) / R;
                for (int k = 0; k < NARC; ++k) {                          // curved head rail
                    Vec2 iA(cx + (arc[k].x - cx) * innerK, Cy + (arc[k].y - Cy) * innerK);
                    Vec2 iB(cx + (arc[k + 1].x - cx) * innerK, Cy + (arc[k + 1].y - Cy) * innerK);
                    emitQuad(frame, fr.at(iA.x, iA.y) + fp, fr.at(iB.x, iB.y) + fp,
                             fr.at(arc[k + 1].x, arc[k + 1].y) + fp,
                             fr.at(arc[k].x, arc[k].y) + fp, fr.n, st.frameColor);
                }
            }
            // Muntins: the pane grid. On an arch, a TRANSOM bar crosses at the
            // springline (sash below, lunette above — the classic layout) and
            // the vertical muntins CONTINUE into the lunette up to the arc
            // itself (device: "the vertical doesn't reach the top of the arch").
            const Real mw = 0.032;
            const Real pz0 = openSill + fw;
            const Real pz1 = (rise > 0) ? ysp : ysp - fw;
            if (rise > 0) frameQuad(wx0, ysp - 0.045, wx1, ysp + 0.02);   // transom
            for (int k = 1; k < st.lightsX; ++k) {
                Real xk = wx0 + span * (Real(k) / st.lightsX);
                frameQuad(xk - mw, pz0, xk + mw, pz1);
                if (rise > 0) {
                    // Up into the lunette: stop at the arc above this x.
                    Real dx = xk - cx;
                    Real yArc = Cy + std::sqrt(std::max(Real(0), R * R - dx * dx));
                    frameQuad(xk - mw, ysp, xk + mw, yArc - 0.01);
                }
            }
            for (int k = 1; k < st.lightsY; ++k) {
                Real yk = pz0 + (pz1 - pz0) * (Real(k) / st.lightsY);
                frameQuad(wx0 + fw, yk - mw, wx1 - fw, yk + mw);
            }

            // GLASS: the full opening behind the frame — a rectangle to the
            // springline plus (for an arch) a lunette fan to the arc.
            emitQuad(glass, oBL + in, oBR + in, oTR + in, oTL + in, fr.n, gcol);
            if (rise > 0) {
                Vec3 S = fr.at(cx, ysp) + in;
                for (int k = 0; k < NARC; ++k)
                    MeshBuilder::emitTri(glass, S, fr.at(arc[k].x, arc[k].y) + in,
                                         fr.at(arc[k + 1].x, arc[k + 1].y) + in,
                                         fr.n, gcol);
            }

            // Surrounds (Trim): the projecting SILL course, and a HOOD — a flat
            // header band, or a voussoir band that FOLLOWS the arch (device:
            // "window arches ... should follow the arch to look natural").
            if (!p.curtainWall && mode != FacadeMode::Solid) {
                RenderMesh& srd = surround;
                auto ledge = [&](Real a0, Real b0, Real a1, Real b1, Real proud) {
                    Vec3 ov = fr.n * proud;
                    Vec3 BL = fr.at(a0, b0), BR = fr.at(a1, b0);
                    Vec3 TL = fr.at(a0, b1), TR = fr.at(a1, b1);
                    emitQuad(srd, BL + ov, BR + ov, TR + ov, TL + ov, fr.n, p.trimColor);
                    emitQuad(srd, TL, TR, TR + ov, TL + ov, fr.v, p.trimColor);        // top ledge
                    emitQuad(srd, BL + ov, BR + ov, BR, BL, fr.v * -1, p.trimColor);   // underside
                    emitQuad(srd, BL, BL + ov, TL + ov, TL, fr.h * -1, p.trimColor);   // left end
                    emitQuad(srd, BR + ov, BR, TR, TR + ov, fr.h, p.trimColor);        // right end
                };
                const Real over = 0.14;                        // oversail past the jambs
                if (st.sill)
                    ledge(std::max(x0, wx0 - over), openSill - 0.13,
                          std::min(x1, wx1 + over), openSill, 0.16);      // sill course
                if (st.hood == OpeningStyle::Hood::Band && rise <= 0)
                    ledge(std::max(x0, wx0 - over), openHead,
                          std::min(x1, wx1 + over), openHead + 0.12, 0.06);   // header
                if (st.hood == OpeningStyle::Hood::Arch && rise > 0) {
                    // Voussoir band: a proud arc course from the extrados out.
                    const Real bw = 0.15, proud = 0.07;
                    const Vec3 hp = fr.n * proud;
                    const Real outerK = (R + bw) / R;
                    auto outerAt = [&](int k) {
                        return Vec2(cx + (arc[k].x - cx) * outerK,
                                    Cy + (arc[k].y - Cy) * outerK);
                    };
                    for (int k = 0; k < NARC; ++k) {
                        Vec3 A = fr.at(arc[k].x, arc[k].y);
                        Vec3 B = fr.at(arc[k + 1].x, arc[k + 1].y);
                        Vec2 oA = outerAt(k), oB = outerAt(k + 1);
                        Vec3 OA = fr.at(oA.x, oA.y), OB = fr.at(oB.x, oB.y);
                        emitQuad(srd, A + hp, B + hp, OB + hp, OA + hp, fr.n, p.trimColor);
                        Real mx = (arc[k].x + arc[k + 1].x) * 0.5;
                        Real my = (arc[k].y + arc[k + 1].y) * 0.5;
                        Vec3 dn = normalize(fr.h * (cx - mx) + fr.v * (Cy - my));
                        emitQuad(srd, A, B, B + hp, A + hp, dn, p.trimColor);      // intrados lip
                        emitQuad(srd, OA + hp, OB + hp, OB, OA, dn * -1, p.trimColor); // extrados
                    }
                    // End CAPS at the springers (device: "the arches ... are
                    // missing the end caps"): close the band's cut face where it
                    // dies onto the wall, on both sides.
                    for (int e : {0, NARC}) {
                        Vec3 A = fr.at(arc[e].x, arc[e].y);
                        Vec2 oE = outerAt(e);
                        Vec3 OE = fr.at(oE.x, oE.y);
                        emitQuad(srd, A, OE, OE + hp, A + hp, fr.v * -1, p.trimColor);
                    }
                }
            }
        }
    }

    // Pilasters: thin vertical piers proud of the wall at each bay boundary. Per
    // storey they stack into continuous full-height pillars. A box projecting
    // outward by `proud`, under Trim.
    if (p.pilasters && mode != FacadeMode::Entrance) {
        RenderMesh trim;
        Real proud = 0.18, pw = 0.5;
        Vec3 up = fr.v;
        for (int b = 0; b <= bays; ++b) {
            Real x = std::min(std::max(b * bw, pw * 0.5), fr.width - pw * 0.5);
            Vec3 c0 = fr.at(x, 0), c1 = fr.at(x, fr.height);
            Vec3 along = normalize(fr.h) * (pw * 0.5);
            Vec3 outv = fr.n * proud;
            // Front quad + two returns of the pier (a shallow box face).
            emitQuad(trim, c0 - along + outv, c0 + along + outv,
                     c1 + along + outv, c1 - along + outv, fr.n, p.trimColor);
            emitQuad(trim, c0 + along, c0 + along + outv, c1 + along + outv,
                     c1 + along, fr.h, p.trimColor);
            emitQuad(trim, c0 - along + outv, c0 - along, c1 - along,
                     c1 - along + outv, fr.h * -1, p.trimColor);
            (void)up;
        }
        appendToPart(out, PartId::Trim, trim);
    }

    // The wall surface goes to the building's chosen facade part (procedural
    // brick/concrete/stucco/metal, or the flat Wall).
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Glass, glass);
    appendToPart(out, PartId::Door, door);
    appendToPart(out, PartId::Trim, surround);
    appendToPart(out, PartId::Detail, frame);   // frames/muntins read as joinery
}

// The box-grammar wrapper: pick the storey scope's face, then share the
// element machinery above with the floorplan path.
void emitFacade(BuildingMesh& out, const Scope& storey, int side, FacadeMode mode,
                const BuildingParams& p, const Vec3& wallColor) {
    emitFacadeRect(out, faceOf(storey, side), mode, p, wallColor);
}

}  // namespace

// --- Curved (cylindrical) tower -------------------------------------------

// A vertical tube (cylinder wall) y0..y1 at `radius`, `sides` facets, outward
// normals, into part `pid` with vertex colour `col`.
static void emitTube(BuildingMesh& out, const Vec3& cXZ, Real radius, Real y0,
                     Real y1, int sides, PartId pid, const Vec3& col) {
    RenderMesh m;
    for (int i = 0; i < sides; ++i) {
        Real a0 = 2 * PI * i / sides, a1 = 2 * PI * (i + 1) / sides;
        Vec3 d0(std::cos(a0), 0, std::sin(a0)), d1(std::cos(a1), 0, std::sin(a1));
        Vec3 n = normalize(d0 + d1);
        emitQuad(m, cXZ + d0 * radius + Vec3(0, y0, 0),
                    cXZ + d1 * radius + Vec3(0, y0, 0),
                    cXZ + d1 * radius + Vec3(0, y1, 0),
                    cXZ + d0 * radius + Vec3(0, y1, 0), n, col);
    }
    appendToPart(out, pid, m);
}

// A horizontal disc cap at height y (normal up or down).
static void emitDisc(BuildingMesh& out, const Vec3& cXZ, Real radius, Real y,
                     int sides, PartId pid, const Vec3& col, bool up) {
    RenderMesh m;
    Vec3 c = cXZ + Vec3(0, y, 0);
    Vec3 n(0, up ? 1 : -1, 0);
    for (int i = 0; i < sides; ++i) {
        Real a0 = 2 * PI * i / sides, a1 = 2 * PI * (i + 1) / sides;
        Vec3 p0 = c + Vec3(std::cos(a0), 0, std::sin(a0)) * radius;
        Vec3 p1 = c + Vec3(std::cos(a1), 0, std::sin(a1)) * radius;
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto v = [&](const Vec3& p) { Vertex vt(p, n, Vec3(1, 0, 0), 0, 0); vt.color = col; return vt; };
        m.vertices.push_back(v(c));
        m.vertices.push_back(v(up ? p0 : p1));
        m.vertices.push_back(v(up ? p1 : p0));
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
    }
    appendToPart(out, pid, m);
}

static BuildingMesh growCylinder(const Scope& scope, const BuildingParams& p) {
    BuildingMesh out;
    Vec3 cXZ = scope.corner(0.5, 0, 0.5); cXZ.y = 0;
    Real baseY = scope.origin.y;
    Real R = std::min(scope.size.x, scope.size.z) * 0.5 * 0.96;
    int sides = std::max(20, p.sides);
    Vec3 wall = p.wallColor;
    Vec3 glass = materialFor(PartId::Glass, wall).albedo;
    Real y = baseY;
    Real gh = p.groundHeight;

    // Round towers are stout, not needle-thin (Marina City / Torre Agbar read at
    // roughly height ≈ 5x diameter). Cap the storey count to that slenderness so a
    // small round lot doesn't become a pencil (ADR-0040).
    Real diameter = 2 * R;
    int maxFloors = std::max(4, static_cast<int>((5.0 * diameter - gh) / p.floorHeight));
    int floors = std::min(p.floors, maxFloors);

    // All bands share one radius — a continuous shell, so there are no radial
    // gaps between the spandrel and window rings to see through (the old inset
    // left open slots and lit the hollow interior).
    emitTube(out, cXZ, R, y, y + 0.5, sides, PartId::Trim, p.trimColor);          // base ring
    emitTube(out, cXZ, R, y + 0.5, y + gh - 0.3, sides, PartId::Glass, glass);    // lobby glass
    emitTube(out, cXZ, R, y + gh - 0.3, y + gh, sides, PartId::Trim, p.trimColor); // cornice ring
    y += gh;
    for (int i = 0; i < floors; ++i) {
        Real fh = p.floorHeight;
        emitTube(out, cXZ, R, y, y + 0.9, sides, PartId::Wall, wall);             // spandrel band
        emitTube(out, cXZ, R, y + 0.9, y + fh, sides, PartId::Glass, glass);      // window band
        y += fh;
    }
    emitDisc(out, cXZ, R, y, sides, PartId::Roof, materialFor(PartId::Roof, wall).albedo, true);
    emitTube(out, cXZ, R, y, y + p.parapet * 0.7, sides, PartId::Trim, p.trimColor);  // parapet
    out.height = (y + p.parapet * 0.7) - baseY;

    BuildingMesh sc;
    emitBox(sc, Scope{scope.origin, {scope.axis[0], Vec3(0, 1, 0), scope.axis[2]},
                      Vec3(scope.size.x, out.height, scope.size.z)}, PartId::Wall, wall);
    if (!sc.parts.empty()) out.proxy = sc.parts.front();
    out.attaches.push_back({cXZ + Vec3(0, y, 0), Vec3(0, 1, 0), "roof"});
    return out;
}

// --- Pagoda (tiered, flared upturned roofs) --------------------------------

// A flared hip roof over a square of half-width `halfW` centred at cXZ, eave at
// `eaveY`: deep eaves (overhang), a concave sweep up to the apex, and the corners
// lifted (`cornerLift`) for the iconic upturned-corner silhouette.
static void emitFlaredRoof(BuildingMesh& out, const Vec3& cXZ, Real eaveY, Real halfW,
                           Real overhang, Real rise, Real cornerLift, const Vec3& tile) {
    RenderMesh m;
    Real e = halfW + overhang;
    auto ring = [&](int k, Real radial, Real frac) {       // k: 0..7 around the square
        Real ang = PI * 0.25 * k;
        Real cx = std::cos(ang), cz = std::sin(ang);
        Real s = 1.0 / std::max(std::fabs(cx), std::fabs(cz));   // project dir onto square
        Vec3 p = cXZ + Vec3(cx, 0, cz) * (e * radial * s);
        bool corner = (k % 2 == 1);
        p.y = eaveY + (radial >= 0.99 ? (corner ? cornerLift : 0.0) : rise * frac);
        return p;
    };
    Vec3 apex = cXZ + Vec3(0, eaveY + rise, 0);
    for (int k = 0; k < 8; ++k) {
        Vec3 e0 = ring(k, 1.0, 0), e1 = ring((k + 1) % 8, 1.0, 0);
        Vec3 m0 = ring(k, 0.42, 0.62), m1 = ring((k + 1) % 8, 0.42, 0.62);
        Vec3 n = normalize(cross(e1 - e0, m0 - e0)); if (n.y < 0) n = n * -1;
        emitQuad(m, e0, e1, m1, m0, n, tile);              // eave -> mid (concave skirt)
        Vec3 tn = normalize(cross(m1 - m0, apex - m0)); if (tn.y < 0) tn = tn * -1;
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto v = [&](const Vec3& p) { Vertex vt(p, tn, Vec3(1, 0, 0), 0, 0); vt.color = tile; return vt; };
        m.vertices.push_back(v(m0)); m.vertices.push_back(v(m1)); m.vertices.push_back(v(apex));
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});   // mid -> apex
    }
    appendToPart(out, PartId::Roof, m);
}

static BuildingMesh growPagoda(const Scope& scope, const BuildingParams& p) {
    BuildingMesh out;
    Rng rng(p.seed);
    Vec3 cXZ = scope.corner(0.5, 0, 0.5); cXZ.y = 0;
    Real baseY = scope.origin.y;
    const Vec3 r(1, 0, 0), f(0, 0, 1);             // world-aligned (grid city)
    Real w0 = std::min(scope.size.x, scope.size.z) * 0.5 * 0.78;

    Vec3 wallCol(0.62, 0.13, 0.11);                // vermillion
    Vec3 colCol(0.42, 0.08, 0.07);                 // darker red columns/trim
    Vec3 tile = (rng.unit() < 0.4) ? Vec3(0.16, 0.28, 0.46)   // imperial blue
                                   : Vec3(0.13, 0.40, 0.25);  // jade green
    Vec3 gold(0.80, 0.64, 0.20);

    auto boxAt = [&](Real cy, Real h, Real hw, PartId pid, const Vec3& col) {
        emitBox(out, Scope{cXZ + Vec3(0, cy, 0) - r * hw - f * hw,
                           {r, Vec3(0, 1, 0), f}, Vec3(hw * 2, h, hw * 2)}, pid, col);
    };

    int tiers = std::max(2, p.tiers);
    Real shrink = 0.80, tierH = std::max(Real(2.8), p.floorHeight);
    Real y = baseY;

    boxAt(y, 0.7, w0 + 0.6, PartId::Ground, Vec3(0.52, 0.50, 0.46));   // stone podium
    y += 0.7;

    Real w = w0;
    for (int t = 0; t < tiers; ++t) {
        boxAt(y, tierH, w, PartId::Wall, wallCol);                     // red tier body
        // Corner columns (the temple posts) + a dark lattice screen per face.
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                emitBox(out, Scope{cXZ + Vec3(sx * w - 0.16, y, sz * w - 0.16),
                                   {r, Vec3(0, 1, 0), f}, Vec3(0.32, tierH, 0.32)},
                        PartId::Trim, colCol);
        boxAt(y + tierH * 0.18, tierH * 0.6, w * 0.82, PartId::Glass, Vec3(0.18, 0.10, 0.06));
        y += tierH;
        // Flared roof at the top of this tier (deep eaves, upturned corners).
        emitFlaredRoof(out, cXZ, y, w, w * 0.55, w * 0.62, w * 0.16, tile);
        w *= shrink;
    }
    // Gold finial: a stacked post + bead spire.
    boxAt(y, 1.6, 0.16, PartId::Detail, gold);
    emitBox(out, Scope{cXZ + Vec3(-0.35, y + 1.6, -0.35), {r, Vec3(0, 1, 0), f},
                       Vec3(0.7, 0.7, 0.7)}, PartId::Detail, gold);
    out.height = (y + 2.3) - baseY;

    BuildingMesh sc;
    emitBox(sc, Scope{Vec3(cXZ.x - w0, baseY, cXZ.z - w0), {r, Vec3(0, 1, 0), f},
                      Vec3(w0 * 2, out.height, w0 * 2)}, PartId::Wall, wallCol);
    if (!sc.parts.empty()) out.proxy = sc.parts.front();
    out.attaches.push_back({cXZ + Vec3(0, y, 0), Vec3(0, 1, 0), "roof"});
    return out;
}

// Rooftop crown (ADR-0040 Pass B): a mechanical penthouse/bulkhead set back on
// the roof, plus the iconic timber water tank on a leg frame — what makes a top
// read as a real building, not an extruded box. `roofY` is the roof slab level;
// footOrigin/width/depth/r/f describe the (possibly set-back) roof footprint.
static void emitCrown(BuildingMesh& out, const Vec3& footOrigin, Real width,
                      Real depth, const Vec3& r, const Vec3& f, Real roofY,
                      const BuildingParams& p, Rng& rng,
                      const Poly2* topPlan = nullptr) {
    const Vec3 up(0, 1, 0);
    // The roof may be an L / U / courtyard PLAN, not its oriented box — a crown
    // element seated by box coordinates alone can hang over the notch (device:
    // "a giant block that doesn't fit properly with the rooftop"). When the
    // caller passes the top plan, every element proves its corners are ON it.
    auto onRoof = [&](const Vec3& o, Real w2, Real d2) {
        if (!topPlan) return true;
        for (int cx = 0; cx <= 1; ++cx)
            for (int cz = 0; cz <= 1; ++cz) {
                Vec3 q = o + r * (w2 * cx) + f * (d2 * cz);
                if (!pointInPolygon(*topPlan, Vec2(q.x, q.z))) return false;
            }
        return true;
    };
    // ROOF PLANT subgrammar (device: "what is that gray block supposed to
    // represent? ... it would help if it had some definition"). The old mute
    // penthouse box is two readable pieces of equipment now:
    //  - a roof-access BULKHEAD — the stair/elevator overrun: human-scaled,
    //    a capped lid slab, and a dark door facing the open roof;
    //  - a louvred HVAC unit — metal casing, intake bands, fan discs on top.
    // Every element proves its seat is ON the roof polygon before emitting.
    const Vec3 darkPanel(0.16, 0.17, 0.18);
    Real bu = -1, bv = 0, bw = 0, bd = 0;   // bulkhead seat (HVAC keeps clear)
    if (p.floors >= 4 && width > 5 && depth > 5) {
        bw = std::min(std::max(width * 0.28, Real(3.0)), Real(5.0));
        bd = std::min(std::max(depth * 0.24, Real(2.4)), Real(3.8));
        const Real bh = rng.range(2.8, 3.3);
        for (int attempt = 0; attempt < 4; ++attempt) {
            const Real u = (width - bw) * rng.range(0.15, 0.6);
            const Real v = (depth - bd) * rng.range(0.15, 0.6);
            Vec3 po = footOrigin + r * u + f * v;
            if (!onRoof(po, bw, bd)) {
                bw *= 0.8; bd *= 0.8;   // shrink toward a seat; give up quietly
                if (bw < 2.4 || bd < 2.0) break;
                continue;
            }
            bu = u; bv = v;
            emitBox(out, Scope{Vec3(po.x, roofY, po.z), {r, up, f},
                               Vec3(bw, bh, bd)},
                    PartId::Trim, p.trimColor * 0.85);
            // The lid: a slightly oversailing cap slab, darker — a roof on
            // the roof, so the box stops reading as an unfinished block.
            emitBox(out, Scope{Vec3(po.x, roofY + bh, po.z) - r * 0.15 - f * 0.15,
                               {r, up, f}, Vec3(bw + 0.3, 0.14, bd + 0.3)},
                    PartId::Trim, p.trimColor * 0.6);
            // The access DOOR faces the open roof centre.
            Vec3 bc = po + r * (bw * 0.5) + f * (bd * 0.5);
            Vec3 rc = footOrigin + r * (width * 0.5) + f * (depth * 0.5);
            Vec3 to = rc - bc;
            const Real dr = dot(to, r), df = dot(to, f);
            const bool alongR = std::fabs(dr) >= std::fabs(df);
            Vec3 n2 = alongR ? r * (dr > 0 ? 1.0 : -1.0)
                             : f * (df > 0 ? 1.0 : -1.0);
            Vec3 tang = alongR ? f : r;
            Vec3 fc = bc + n2 * ((alongR ? bw : bd) * 0.5);
            Vec3 doorO = fc - tang * 0.5;
            emitBox(out, Scope{Vec3(doorO.x, roofY, doorO.z), {tang, up, n2},
                               Vec3(1.0, 2.1, 0.06)},
                    PartId::Detail, darkPanel);
            break;
        }
    }
    // The HVAC unit: lower and wider than the bulkhead, kept clear of it.
    if (p.floors >= 4 && width > 6 && depth > 5 && rng.unit() < 0.85) {
        const Real aw = std::min(std::max(width * 0.30, Real(2.6)), Real(6.0));
        const Real ad = std::min(std::max(depth * 0.22, Real(2.0)), Real(3.4));
        const Real ah = rng.range(1.5, 2.1);
        const Vec3 casing = materialFor(PartId::Metal, p.wallColor).albedo;
        const Vec3 louvre(0.30, 0.31, 0.33);
        for (int attempt = 0; attempt < 5; ++attempt) {
            const Real u = (width - aw) * rng.range(0.1, 0.9);
            const Real v = (depth - ad) * rng.range(0.1, 0.9);
            if (bu >= 0 && u < bu + bw + 0.6 && u + aw + 0.6 > bu &&
                v < bv + bd + 0.6 && v + ad + 0.6 > bv) continue;
            Vec3 po = footOrigin + r * u + f * v;
            if (!onRoof(po, aw, ad)) continue;
            emitBox(out, Scope{Vec3(po.x, roofY, po.z), {r, up, f},
                               Vec3(aw, ah, ad)},
                    PartId::Metal, casing);
            // Intake louvres: three dark bands along both long faces.
            for (int band = 0; band < 3; ++band) {
                const Real ly = roofY + ah * (0.22 + 0.24 * band);
                for (int side = 0; side < 2; ++side) {
                    Vec3 lo = po + r * 0.2 + f * (side ? ad : Real(-0.05));
                    emitBox(out, Scope{Vec3(lo.x, ly, lo.z), {r, up, f},
                                       Vec3(aw - 0.4, 0.12, 0.05)},
                            PartId::Detail, louvre);
                }
            }
            // Fan cowls + dark blade discs on top.
            const int nf = aw > 4.2 ? 2 : 1;
            for (int k2 = 0; k2 < nf; ++k2) {
                const Real fu = aw * (nf == 1 ? 0.5 : (k2 == 0 ? 0.3 : 0.7));
                Vec3 fcen = po + r * fu + f * (ad * 0.5);
                fcen.y = 0;
                const Real frad = std::min(Real(0.6), std::min(aw, ad) * 0.22);
                emitTube(out, fcen, frad, roofY + ah, roofY + ah + 0.22, 12,
                         PartId::Metal, casing * 0.9);
                emitDisc(out, fcen, frad, roofY + ah + 0.22, 12,
                         PartId::Detail, darkPanel, true);
            }
            break;
        }
    }
    // Timber water tank: a tube on four legs with a flat top — the NYC rooftop
    // tank, on masonry mid-rises. The wood surface makes it read as timber.
    if (p.floors >= 3 && p.floors <= 20 && !p.curtainWall && width > 4 && depth > 4 &&
        rng.unit() < 0.65) {
        Real tr = std::min(rng.range(1.2, 1.7), std::min(width, depth) * 0.22);
        Real th = rng.range(2.6, 3.4), legH = rng.range(1.6, 2.4);
        Real ix = tr + 0.6, iz = tr + 0.6;   // keep the tank fully on the roof
        Vec3 tc = footOrigin + r * (ix + (width - 2 * ix) * rng.unit()) +
                  f * (iz + (depth - 2 * iz) * rng.unit());
        tc.y = 0;
        for (int attempt = 0;
             attempt < 3 && !onRoof(tc - r * tr - f * tr, 2 * tr, 2 * tr);
             ++attempt) {
            tc = footOrigin + r * (ix + (width - 2 * ix) * rng.unit()) +
                 f * (iz + (depth - 2 * iz) * rng.unit());
            tc.y = 0;
        }
        if (!onRoof(tc - r * tr - f * tr, 2 * tr, 2 * tr)) return;
        Real tankBase = roofY + legH;
        Vec3 woodCol = materialFor(PartId::Wood, p.wallColor).albedo;
        for (int lx = -1; lx <= 1; lx += 2)
            for (int lz = -1; lz <= 1; lz += 2) {
                Vec3 lc = tc + r * (lx * tr * 0.7) + f * (lz * tr * 0.7);
                emitBox(out, Scope{Vec3(lc.x, roofY, lc.z) - r * 0.08 - f * 0.08,
                                   {r, up, f}, Vec3(0.16, legH, 0.16)},
                        PartId::Wood, woodCol * 0.85);
            }
        emitTube(out, tc, tr, tankBase, tankBase + th, 14, PartId::Wood, woodCol);
        emitDisc(out, tc, tr, tankBase + th, 14, PartId::Wood, woodCol * 1.05, true);
    }
}

BuildingMesh growBuilding(const Scope& scope, const BuildingParams& params) {
    if (params.shape == BuildingShape::Cylinder) return growCylinder(scope, params);
    if (params.shape == BuildingShape::Pagoda)   return growPagoda(scope, params);
    BuildingMesh out;
    Rng rng(params.seed);

    const Real baseY = scope.origin.y;
    const Vec3 wallColor =
        params.wallColor * (0.92 + 0.08 * rng.unit());      // slight per-building tint

    // Running footprint (origin XZ + extents along right/forward); shrinks at setbacks.
    Vec3 footOrigin = scope.origin;
    Real width = scope.size.x, depth = scope.size.z;
    const Vec3 r = scope.axis[0], f = scope.axis[2];

    auto storeyScope = [&](Real y, Real h) {
        Scope s;
        s.axis[0] = r; s.axis[1] = Vec3(0, 1, 0); s.axis[2] = f;
        s.origin = Vec3(footOrigin.x, y, footOrigin.z);
        s.size = Vec3(width, h, depth);
        return s;
    };

    Real y = baseY;

    // Entrance side: the face whose outward normal points most toward the street
    // (params.faceDir), so the door faces the road, not an alley (ADR-0040). Side
    // normals follow faceOf: 0=+f, 1=-f, 2=+r, 3=-r.
    Vec3 sideNormal[4] = {f, f * -1, r, r * -1};
    int entranceSide = 0;
    Real bestDot = -1e30;
    for (int s = 0; s < 4; ++s) {
        Real dp = sideNormal[s].x * params.faceDir.x + sideNormal[s].z * params.faceDir.z;
        if (dp > bestDot) { bestDot = dp; entranceSide = s; }
    }

    // Ground floor: taller, glassy retail (or lobby), walkable shell with a real
    // entrance on the street-facing face (ADR-0038 §4).
    Real gh = params.groundHeight;
    Scope ground = storeyScope(y, gh);
    FacadeMode groundMode = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    for (int side = 0; side < 4; ++side) {
        FacadeMode mode = (side == entranceSide && params.walkableGround)
                              ? FacadeMode::Entrance : groundMode;
        emitFacade(out, ground, side, mode, params, wallColor);
    }
    // Ground slab you can stand on.
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.1, depth)},
            PartId::Ground, materialFor(PartId::Ground, wallColor).albedo);

    // Base course / water-table: a low band the building rises from. Emitted
    // per face and skipped on the entrance side (ADR-0040) so it steps around
    // the doorway instead of clipping it ("the foundation eats the base").
    if (params.baseCourse) {
        const Real bh = std::min(Real(0.45), gh * 0.12);   // water-table height
        const Real proud = 0.1;
        Vec3 col = params.trimColor * 0.8;
        RenderMesh band;
        for (int side = 0; side < 4; ++side) {
            if (side == entranceSide && params.walkableGround) continue;   // entrance breaks it
            FaceRect fr = faceOf(ground, side);
            Vec3 out = fr.n * proud;
            Vec3 b0 = fr.at(0, 0), b1 = fr.at(fr.width, 0);
            Vec3 t0 = fr.at(0, bh), t1 = fr.at(fr.width, bh);
            emitQuad(band, b0 + out, b1 + out, t1 + out, t0 + out, fr.n, col);  // face
            emitQuad(band, t0 + out, t1 + out, t1, t0, fr.v, col);              // top ledge
        }
        appendToPart(out, PartId::Trim, band);
    }
    // (The entrance attach point + awning + doorframe are emitted by the DOOR
    // element inside emitFacade, so they sit on the actual door bay — a face-
    // centred awning was off by half a bay on even bay counts.)
    // BASE CORNICE: a real stepped profile capping the base (device: "I don't
    // see a cornice at the top of the building's base" — the old single band
    // read as nothing). Three courses stepping outward — bed mould, corona,
    // and a thin cap slab — wrapped around the whole perimeter.
    auto emitCornice = [&](Real yTop, Real scale) {
        struct Tier { Real grow, h; };
        const Tier tiers[3] = {{0.10, 0.16}, {0.24, 0.18}, {0.34, 0.08}};
        Real yb = yTop;
        for (const Tier& t : tiers) {
            Real g2 = t.grow * scale, h2 = t.h * scale;
            Scope sc{Vec3(footOrigin.x, yb, footOrigin.z) - r * g2 - f * g2,
                     {r, Vec3(0, 1, 0), f}, Vec3(width + 2 * g2, h2, depth + 2 * g2)};
            emitBox(out, sc, PartId::Trim, params.trimColor);
            yb += h2;
        }
    };
    if (params.stringCourse) emitCornice(y - 0.32, 1.0);
    y += gh;

    // Upper floors carry no pilasters — base piers belong to the base only
    // (ADR-0040), capped by the string course above.
    BuildingParams upper = params;
    upper.pilasters = false;

    // Upper residential floors, with optional setbacks.
    bool didSetback = false;
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0 &&
            width - 2 * params.setbackEvery >= 8.0 &&
            depth - 2 * params.setbackEvery >= 8.0) {
            didSetback = true;
            Real d = params.setbackEvery;
            Real dx = std::min(d, width * 0.4), dz = std::min(d, depth * 0.4);
            // Cap the lower (wider) mass with a roof slab so the exposed setback
            // ledge is a real surface, not an open shelf (ADR-0040: stepped
            // towers terrace at each setback). Slab spans the old footprint; the
            // new mass rises from it and hides all but the ledge ring.
            emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                               {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
                    PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
            footOrigin = footOrigin + r * dx + f * dz;
            width -= 2 * dx; depth -= 2 * dz;
            // A low terrace parapet around the stepped-back mass.
            emitParapet(out, footOrigin, width, depth, r, f, y, 0.6, 0.24,
                        PartId::Trim, materialFor(PartId::Trim, wallColor).albedo);
        }
        Real fh = params.floorHeight;
        Scope storey = storeyScope(y, fh);
        FacadeMode mode = params.solidFacade ? FacadeMode::Solid
                                             : FacadeMode::Residential;
        for (int side = 0; side < 4; ++side) {
            if (params.curtainWall) emitCurtainWall(out, storey, side, wallColor);
            else emitFacade(out, storey, side, mode, upper, wallColor);
        }
        if (i == params.floors / 2) {
            FaceRect ff = faceOf(storey, 0);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }

    // QUOINS (element, P2): alternating cast-stone blocks up every corner
    // arris — the masonry corner treatment (and it hides the thin-texture edge
    // where two brick faces meet, device feedback). Skipped on stepped towers
    // (the corners move) and clean-skin facades.
    if (params.quoins && !params.curtainWall && !params.solidFacade && !didSetback) {
        const Real qh = 0.42, gap = 0.03, proud = 0.045;
        const Real longL = 0.62, shortL = 0.30;
        const Vec3 qcol = params.trimColor;
        for (int cxi = 0; cxi <= 1; ++cxi)
            for (int czi = 0; czi <= 1; ++czi) {
                int k = 0;
                for (Real qy = baseY + 0.06; qy + qh <= y - 0.45; qy += qh, ++k) {
                    const bool alongR = ((k + cxi + czi) & 1) == 0;
                    const Real lr = alongR ? longL : shortL;    // extent along r
                    const Real lf = alongR ? shortL : longL;    // extent along f
                    const Real x0q = cxi ? width - lr : -proud;
                    const Real z0q = czi ? depth - lf : -proud;
                    Vec3 o = footOrigin + r * x0q + f * z0q;
                    o.y = qy;
                    emitBox(out, Scope{o, {r, Vec3(0, 1, 0), f},
                                       Vec3(lr + proud, qh - gap, lf + proud)},
                            PartId::Trim, qcol);
                }
            }
    }

    // Top cornice: the stepped crown capping the shaft (mirrors the base
    // cornice, a touch larger), so the shaft reads as framed between base and
    // crown — the tripartite base/shaft/capital line a masonry building always
    // has. Glass towers get a clean cap instead.
    if (params.stringCourse && !params.curtainWall) emitCornice(y - 0.45, 1.25);
    // Roof: a flat slab + a parapet railing around the perimeter (ADR-0038 §4).
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
            PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
    if (params.parapet > 0) {
        emitParapet(out, footOrigin, width, depth, r, f, y, params.parapet, 0.28,
                    PartId::Trim, materialFor(PartId::Trim, wallColor).albedo);
    }
    // Crown: mechanical penthouse + rooftop water tank (ADR-0040 Pass B).
    emitCrown(out, footOrigin, width, depth, r, f, y, params, rng);
    out.attaches.push_back({Vec3(footOrigin.x, y, footOrigin.z) +
                                r * (width * 0.5) + f * (depth * 0.5),
                            Vec3(0, 1, 0), "roof"});

    out.height = (y + params.parapet) - baseY;

    // Coarse proxy mass (single box, ground footprint to roof) for HLOD/impostor
    // baking (ADR-0038 §6). Built into a scratch BuildingMesh so it stays separate
    // from the detailed parts.
    {
        BuildingMesh scratch;
        emitBox(scratch, Scope{scope.origin, {scope.axis[0], Vec3(0, 1, 0), scope.axis[2]},
                               Vec3(scope.size.x, out.height, scope.size.z)},
                PartId::Wall, wallColor);
        if (!scratch.parts.empty()) out.proxy = scratch.parts.front();
    }

    return out;
}

RenderMesh BuildingMesh::merged() const {
    RenderMesh m;
    for (const RenderMesh& p : parts) MeshBuilder::append(m, p);
    return m;
}


// --- Floorplan buildings (building-grammar-plan.md P3) ----------------------

namespace {

// Offset a CCW plan polygon: d > 0 shrinks (inset), d < 0 grows (outset) — the
// swept-cornice / setback-tier primitive. Same line-intersection construction
// as polygon.h's inset, without its d > 0 guard (cornices need the outset).
Poly2 offsetPlan(const Poly2& poly, Real d) {
    const std::size_t n = poly.size();
    if (n < 3 || d == 0) return poly;
    Poly2 p = poly;
    ensureCCW(p);
    struct Line { Vec2 pt, dir; };
    std::vector<Line> lines(n);
    for (std::size_t i = 0; i < n; ++i) {
        Vec2 a = p[i], b = p[(i + 1) % n];
        Vec2 dir = normalize(b - a);
        Vec2 inward(-dir.y, dir.x);          // interior is LEFT of a CCW edge
        lines[i] = {a + inward * d, dir};
    }
    Poly2 out(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Line& l0 = lines[(i + n - 1) % n];
        const Line& l1 = lines[i];
        Real denom = cross(l0.dir, l1.dir);
        if (std::abs(denom) < 1e-9) { out[i] = l1.pt; continue; }
        Real t = cross(l1.pt - l0.pt, l1.dir) / denom;
        Vec2 q = l0.pt + l0.dir * t;
        // MITER LIMIT: at a near-parallel joint (an arc prow's chords) the
        // line intersection flies arbitrarily far and the ring grows spikes
        // (device: the haywire tower top). Fall back to a bevel-ish offset —
        // the vertex translated by its own edge normal.
        if ((q - p[i]).length() > std::fabs(d) * 4.0 + 0.5) q = l1.pt;
        out[i] = q;
    }
    return out;
}

// One plan edge as a facade rectangle: outward for CCW is the RIGHT of a->b.
FaceRect planEdgeRect(const Poly2& pl, std::size_t i, Real y, Real h) {
    Vec2 a = pl[i], b = pl[(i + 1) % pl.size()];
    Vec2 d = normalize(b - a);
    FaceRect fr;
    fr.bl = Vec3(a.x, y, a.y);
    fr.h = Vec3(d.x, 0, d.y);
    fr.v = Vec3(0, 1, 0);
    fr.n = Vec3(d.y, 0, -d.x);
    fr.width = (b - a).length();
    fr.height = h;
    return fr;
}

// A horizontal SLAB filling the plan: triangulated top + underside, plus a
// side band around the outline — roof decks, cornice tiers, ground pads.
void emitPlanSlab(BuildingMesh& out, const Poly2& pl, Real yTop, Real thick,
                  PartId part, const Vec3& col) {
    if (pl.size() < 3) return;
    RenderMesh m;
    for (const auto& t : triangulatePolygon(pl)) {
        Vec3 a(pl[t[0]].x, yTop, pl[t[0]].y);
        Vec3 b(pl[t[1]].x, yTop, pl[t[1]].y);
        Vec3 c(pl[t[2]].x, yTop, pl[t[2]].y);
        MeshBuilder::emitTri(m, a, b, c, Vec3(0, 1, 0), col);
        Vec3 a2 = a, b2 = b, c2 = c;
        a2.y = b2.y = c2.y = yTop - thick;
        MeshBuilder::emitTri(m, a2, b2, c2, Vec3(0, -1, 0), col);
    }
    for (std::size_t i = 0; i < pl.size(); ++i) {
        FaceRect fr = planEdgeRect(pl, i, yTop - thick, thick);
        emitQuad(m, fr.at(0, 0), fr.at(fr.width, 0), fr.at(fr.width, thick),
                 fr.at(0, thick), fr.n, col);
    }
    appendToPart(out, part, m);
}

// A real LIP around the roof (device: "the roof tops don't really have a lip
// around them"): an upstand RING following the plan outline — outer face on
// the plan line, 0.24 m thick, usually in the facade's own material — capped
// by a slightly oversailing darker coping so the edge reads from the street.
// Each edge's boxes extend half a thickness past their corners, closing the
// ring without mitring (convex corners butt, concave overlap harmlessly).
void emitPlanParapet(BuildingMesh& out, const Poly2& pl, Real roofY, Real h,
                     const Vec3& wallCol, PartId wallPart,
                     const Vec3& copingCol) {
    if (h <= 0 || pl.size() < 3) return;
    const Vec3 up(0, 1, 0);
    const Real th = 0.24;    // upstand thickness
    const Real lip = 0.05;   // coping oversail, in and out
    for (std::size_t i = 0; i < pl.size(); ++i) {
        Vec2 a = pl[i], b = pl[(i + 1) % pl.size()];
        Vec2 dv = b - a;
        const Real len = dv.length();
        if (len < 1e-6) continue;
        Vec2 d = dv * (1.0 / len);
        Vec2 nOut(d.y, -d.x);   // outward for a CCW plan
        Vec3 d3(d.x, 0, d.y), n3(nOut.x, 0, nOut.y);
        Vec3 o = Vec3(a.x, roofY, a.y) - n3 * th - d3 * (th * 0.5);
        emitBox(out, Scope{o, {d3, up, n3}, Vec3(len + th, h, th)},
                wallPart, wallCol);
        Vec3 co = Vec3(a.x, roofY + h, a.y) - n3 * (th + lip) -
                  d3 * (th * 0.5 + lip);
        emitBox(out, Scope{co, {d3, up, n3},
                           Vec3(len + th + 2 * lip, 0.09, th + 2 * lip)},
                PartId::Trim, copingCol);
    }
}

// The VEHICLE BAY front (fire stations, loading docks, parking entries): the
// entrance edge's ground floor as a row of wide segmented roller doors. Each
// bay is a real recessed opening — jamb + head reveals connect the wall plane
// back to the door plane — and the door panel carries horizontal SLATS so it
// reads as a roller door, with a lintel band across the whole front.
void emitBayFront(BuildingMesh& out, const FaceRect& fr,
                  const BuildingParams& p, const Vec3& wallColor) {
    RenderMesh wall, detail, trim;
    const Real W = fr.width, H = fr.height;
    int n = std::max(1, p.groundBays);
    Real margin = 1.0;
    const Real gap = 0.8;
    Real bayW = (W - 2 * margin - (n - 1) * gap) / n;
    while (n > 1 && bayW < 3.2) {   // cramped front: fewer, proper-width bays
        --n;
        bayW = (W - 2 * margin - (n - 1) * gap) / n;
    }
    bayW = std::min(bayW, Real(4.8));
    if (bayW < 2.6) {   // no room for even one bay: plain wall face
        emitQuad(wall, fr.at(0, 0), fr.at(W, 0), fr.at(W, H), fr.at(0, H),
                 fr.n, wallColor);
        appendToPart(out, p.wallPart, wall);
        return;
    }
    margin = (W - (n * bayW + (n - 1) * gap)) * 0.5;
    const Real bayH = std::min(H - 0.8, Real(3.8));
    auto wallQuad = [&](Real a0, Real a1, Real b0, Real b1) {
        if (a1 - a0 < 1e-4 || b1 - b0 < 1e-4) return;
        emitQuad(wall, fr.at(a0, b0), fr.at(a1, b0), fr.at(a1, b1),
                 fr.at(a0, b1), fr.n, wallColor);
    };
    wallQuad(0, margin, 0, H);                 // end piers
    wallQuad(W - margin, W, 0, H);
    wallQuad(margin, W - margin, bayH, H);     // over the doors
    const Vec3 in = fr.n * -0.35;              // door plane, well recessed
    const Vec3 panel(0.36, 0.37, 0.39), slat(0.28, 0.29, 0.31);
    for (int b = 0; b < n; ++b) {
        const Real x0 = margin + b * (bayW + gap), x1 = x0 + bayW;
        if (b + 1 < n) wallQuad(x1, x1 + gap, 0, bayH);   // pier between bays
        Vec3 tL = fr.at(x0, bayH), tR = fr.at(x1, bayH);
        Vec3 bL = fr.at(x0, 0), bR = fr.at(x1, 0);
        emitQuad(wall, bL, bL + in, tL + in, tL, fr.h, wallColor);        // jamb
        emitQuad(wall, bR + in, bR, tR, tR + in, fr.h * -1, wallColor);   // jamb
        emitQuad(wall, tL, tL + in, tR + in, tR, fr.v * -1, wallColor);   // head
        emitQuad(detail, bL + in, bR + in, tR + in, tL + in, fr.n, panel);
        const Vec3 proud = fr.n * 0.03;
        for (Real sy = 0.5; sy < bayH - 0.3; sy += 0.55) {
            Vec3 sL = fr.at(x0 + 0.12, sy) + in + proud;
            Vec3 sR = fr.at(x1 - 0.12, sy) + in + proud;
            emitQuad(detail, sL, sR, sR + fr.v * 0.16, sL + fr.v * 0.16,
                     fr.n, slat);
        }
    }
    // Lintel band across the whole front above the doors.
    const Vec3 ov = fr.n * 0.10;
    const Real ly0 = bayH + 0.05, ly1 = std::min(H - 0.1, bayH + 0.5);
    if (ly1 > ly0)
        emitQuad(trim, fr.at(margin * 0.4, ly0) + ov,
                 fr.at(W - margin * 0.4, ly0) + ov,
                 fr.at(W - margin * 0.4, ly1) + ov,
                 fr.at(margin * 0.4, ly1) + ov, fr.n, p.trimColor);
    appendToPart(out, p.wallPart, wall);
    appendToPart(out, PartId::Detail, detail);
    appendToPart(out, PartId::Trim, trim);
}

}  // namespace

BuildingMesh growPlanBuilding(const Poly2& planIn, const BuildingParams& params,
                              Real baseY) {
    BuildingMesh out;
    Poly2 plan = planIn;
    if (plan.size() < 3) return out;
    ensureCCW(plan);
    Rng rng(params.seed);
    const Vec3 wallColor = params.wallColor * (0.92 + 0.08 * rng.unit());

    // Street-facing edge: the longest edge whose outward normal points most
    // toward faceDir — the door (and its awning/architrave) lands there.
    std::size_t entranceEdge = 0;
    Real bestScore = -1e30;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
        Vec2 d = b - a;
        Real len = d.length();
        if (len < human::DOOR_WIDTH + 1.2) continue;
        Vec2 nrm(d.y / len, -d.x / len);
        Real score = nrm.x * params.faceDir.x + nrm.y * params.faceDir.z + len * 0.01;
        if (score > bestScore) { bestScore = score; entranceEdge = i; }
    }

    // Swept cornice: three stepped courses following the CURRENT plan outline.
    auto sweptCornice = [&](const Poly2& pl, Real yTop, Real scale) {
        struct Tier { Real grow, h; };
        const Tier tiers[3] = {{0.10, 0.16}, {0.24, 0.18}, {0.34, 0.08}};
        Real yb = yTop;
        for (const Tier& t : tiers) {
            emitPlanSlab(out, offsetPlan(pl, -t.grow * scale), yb + t.h * scale,
                         t.h * scale, PartId::Trim, params.trimColor);
            yb += t.h * scale;
        }
    };
    // Corner POSTS at every plan vertex: a square pier hiding the wall miter
    // (the floorplan counterpart of the box path's quoined arris). With quoins
    // on, the post alternates block heights like a quoin stack.
    auto cornerPosts = [&](const Poly2& pl, Real y0, Real h) {
        if (params.curtainWall || params.solidFacade) return;
        for (std::size_t i = 0; i < pl.size(); ++i) {
            Vec2 P = pl[i];
            Vec2 pPrev = pl[(i + pl.size() - 1) % pl.size()];
            Vec2 pNext = pl[(i + 1) % pl.size()];
            Vec2 d0 = normalize(P - pPrev), d1 = normalize(pNext - P);
            Vec2 n0(d0.y, -d0.x), n1(d1.y, -d1.x);
            Vec2 bis = n0 + n1;
            Real bl = bis.length();
            if (bl < 1e-6) continue;                       // straight-through vertex
            // Only REAL corners get a post: chord joints of a tessellated
            // CURVED plan edge (turn < ~33 deg) stay smooth, so a round tower
            // reads as a curve, not a ribbed drum.
            if (std::fabs(cross(d0, d1)) < 0.55) continue;
            bis = bis * (1.0 / bl);
            Vec2 side(-bis.y, bis.x);
            const Real half = 0.20, proud = 0.05;
            Vec3 r3(side.x, 0, side.y), f3(bis.x, 0, bis.y);
            if (params.quoins) {
                const Real qh = 0.42, gap = 0.03;
                int k = 0;
                for (Real qy = y0; qy + qh <= y0 + h; qy += qh, ++k) {
                    Real w = (k & 1) ? half * 1.5 : half * 2.3;
                    Scope s{Vec3(P.x, qy, P.y) - r3 * (w * 0.5) - f3 * (half - proud),
                            {r3, Vec3(0, 1, 0), f3}, Vec3(w, qh - gap, half * 2)};
                    emitBox(out, s, PartId::Trim, params.trimColor);
                }
            } else {
                Scope s{Vec3(P.x, y0, P.y) - r3 * half - f3 * (half - proud),
                        {r3, Vec3(0, 1, 0), f3}, Vec3(half * 2, h, half * 2)};
                emitBox(out, s, PartId::Trim, params.trimColor * 0.92);
            }
        }
    };

    Real y = baseY;
    const Real gh = params.groundHeight;
    FacadeMode groundMode = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    // Ground storey: one facade rect per plan edge; the door on the street edge.
    // retailStreetOnly (P3.c): storefronts only where the edge FACES the street
    // (normal within ~70 deg of faceDir); side/rear edges wear plain walls.
    for (std::size_t i = 0; i < plan.size(); ++i) {
        // VEHICLE BAYS claim the street edge outright (fire station, depot).
        if (i == entranceEdge && params.groundBays > 0) {
            emitBayFront(out, planEdgeRect(plan, i, y, gh), params, wallColor);
            continue;
        }
        FacadeMode mode = (i == entranceEdge && params.walkableGround)
                              ? FacadeMode::Entrance : groundMode;
        if (mode == FacadeMode::Retail && params.retailStreetOnly) {
            Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
            Vec2 d = normalize(b - a);
            Vec2 nrm(d.y, -d.x);
            if (nrm.x * params.faceDir.x + nrm.y * params.faceDir.z < 0.35)
                mode = FacadeMode::Residential;
        }
        if (params.curtainWall && mode != FacadeMode::Entrance)
            emitCurtainWallRect(out, planEdgeRect(plan, i, y, gh), wallColor);
        else
            emitFacadeRect(out, planEdgeRect(plan, i, y, gh), mode, params, wallColor);
    }
    emitPlanSlab(out, plan, y + 0.05, 0.1, PartId::Ground,
                 materialFor(PartId::Ground, wallColor).albedo);
    // Base course wraps the plan (skipping the door edge).
    if (params.baseCourse) {
        const Real bh = std::min(Real(0.45), gh * 0.12);
        RenderMesh band;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            if (i == entranceEdge && params.walkableGround) continue;
            FaceRect fr = planEdgeRect(plan, i, y, bh);
            Vec3 ov = fr.n * 0.1;
            emitQuad(band, fr.at(0, 0) + ov, fr.at(fr.width, 0) + ov,
                     fr.at(fr.width, bh) + ov, fr.at(0, bh) + ov, fr.n,
                     params.trimColor * 0.8);
            emitQuad(band, fr.at(0, bh) + ov, fr.at(fr.width, bh) + ov,
                     fr.at(fr.width, bh), fr.at(0, bh), fr.v, params.trimColor * 0.8);
        }
        appendToPart(out, PartId::Trim, band);
    }
    if (params.stringCourse) sweptCornice(plan, y + gh - 0.32, 1.0);
    cornerPosts(plan, y, gh);
    y += gh;

    // Upper floors; setbacks shrink the plan per tier (base/shaft/capital),
    // each transition capped by a roof slab + a swept cornice.
    BuildingParams upper = params;
    upper.pilasters = false;
    Poly2 cur = plan;
    Real tierY0 = y;
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0) {
            // A tier inset that EXPLODES must not become the next tier: on an
            // acute prow the line intersections fly off, or the ring self-
            // crosses, while still passing an area check (device: "one of the
            // triangle skyscrapers went haywire when building the top"). The
            // tier is valid only if it truly shrank, every vertex stayed
            // inside the tier below, and no edge flipped direction. An
            // invalid inset just means the tower rises straight instead.
            auto insetOk = [](const Poly2& outer, const Poly2& inner) {
                if (inner.size() != outer.size()) return false;
                const Real ai = area(inner);
                if (ai < 60.0 || ai >= area(outer)) return false;
                for (std::size_t k = 0; k < inner.size(); ++k) {
                    if (!pointInPolygon(outer, inner[k])) return false;
                    Vec2 d0 = outer[(k + 1) % outer.size()] - outer[k];
                    Vec2 d1 = inner[(k + 1) % inner.size()] - inner[k];
                    if (dot(d0, d1) <= 0) return false;   // edge flipped
                }
                return true;
            };
            Poly2 next = offsetPlan(cur, params.setbackEvery);
            if (insetOk(cur, next)) {
                emitPlanSlab(out, cur, y - 0.05, 0.2, PartId::Roof,
                             materialFor(PartId::Roof, wallColor).albedo);
                if (params.stringCourse && !params.curtainWall)
                    sweptCornice(cur, y - 0.4, 1.0);
                cornerPosts(cur, tierY0, y - tierY0);
                emitPlanParapet(out, offsetPlan(cur, 0.02), y, 0.55,
                                materialFor(PartId::Trim, wallColor).albedo,
                                PartId::Trim,
                                materialFor(PartId::Trim, wallColor).albedo * 0.9);
                cur = next;
                tierY0 = y;
            }
        }
        const Real fh = params.floorHeight;
        for (std::size_t e = 0; e < cur.size(); ++e) {
            if (params.curtainWall)
                emitCurtainWallRect(out, planEdgeRect(cur, e, y, fh), wallColor);
            else
                emitFacadeRect(out, planEdgeRect(cur, e, y, fh),
                               params.solidFacade ? FacadeMode::Solid
                                                  : FacadeMode::Residential,
                               upper, wallColor);
        }
        if (i == params.floors / 2) {
            FaceRect ff = planEdgeRect(cur, entranceEdge % cur.size(), y, fh);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }
    cornerPosts(cur, tierY0, y - tierY0);

    // ROOF (P3.c): a Gable/Hip pitched roof over a rect-ish top plan — the
    // residential silhouette — else the flat deck + parapet + crown.
    OBB2 topObb = orientedBoundingBox(cur);
    const bool pitched =
        params.roofStyle != BuildingParams::RoofStyle::Flat &&
        area(cur) > 0.85 * (4 * topObb.half[0] * topObb.half[1]);
    Real roofRise = 0;
    if (pitched) {
        const bool hip = params.roofStyle == BuildingParams::RoofStyle::Hip;
        const int la = topObb.longAxis(), sa = 1 - la;
        Vec3 r3(topObb.axis[la].x, 0, topObb.axis[la].y);
        Vec3 f3(topObb.axis[sa].x, 0, topObb.axis[sa].y);
        const Real ov = 0.45;                            // eaves overhang
        const Real hw = topObb.half[la] + (hip ? ov : Real(0));
        const Real hd = topObb.half[sa] + ov;
        const Real rise = std::max(Real(0.8), params.roofPitch * hd);
        roofRise = rise + 0.03;
        // A modest eaves cornice band, then a thin ceiling deck under the roof.
        if (params.stringCourse && !params.curtainWall) sweptCornice(cur, y - 0.30, 0.7);
        emitPlanSlab(out, cur, y + 0.03, 0.15, PartId::Roof,
                     materialFor(PartId::Roof, wallColor).albedo);
        Vec3 C(topObb.center.x, y + 0.03, topObb.center.y);
        const Real rh = hip ? std::max(Real(0.6), hw - hd) : hw;   // ridge half-length
        Vec3 A0 = C - r3 * hw - f3 * hd, A1 = C + r3 * hw - f3 * hd;
        Vec3 B0 = C - r3 * hw + f3 * hd, B1 = C + r3 * hw + f3 * hd;
        Vec3 up(0, 1, 0);
        Vec3 Rg0 = C - r3 * rh + up * rise, Rg1 = C + r3 * rh + up * rise;
        RenderMesh roof, gableW;
        const Vec3 roofCol = materialFor(PartId::Roof, wallColor).albedo;
        Vec3 nNear = normalize(f3 * (-rise) + up * hd);
        Vec3 nFar = normalize(f3 * rise + up * hd);
        emitQuad(roof, A0, A1, Rg1, Rg0, nNear, roofCol);   // near slope
        emitQuad(roof, B1, B0, Rg0, Rg1, nFar, roofCol);    // far slope
        if (hip) {
            const Real run = std::max(Real(0.2), hw - rh);
            MeshBuilder::emitTri(roof, A0, B0, Rg0,
                                 normalize(r3 * (-rise) + up * run), roofCol);
            MeshBuilder::emitTri(roof, A1, B1, Rg1,
                                 normalize(r3 * rise + up * run), roofCol);
        } else {
            // Gable END walls rise to the ridge in the wall material.
            MeshBuilder::emitTri(gableW, A0, B0, Rg0, r3 * -1, wallColor);
            MeshBuilder::emitTri(gableW, A1, B1, Rg1, r3, wallColor);
        }
        // SOFFIT: a down-facing deck across the whole eave rectangle. The
        // slopes are single-sided, so wherever they overhang the walls the
        // roof was see-through from below (device: "you can see through the
        // bottom of them"). Sits a hair under the slope base so it never
        // fights the ceiling slab.
        {
            RenderMesh soffit;
            const Vec3 dn(0, -1, 0), sc = materialFor(PartId::Trim, wallColor).albedo;
            Vec3 drop = up * -0.01;
            emitQuad(soffit, A0 + drop, B0 + drop, B1 + drop, A1 + drop, dn, sc);
            appendToPart(out, PartId::Trim, soffit);
        }
        appendToPart(out, PartId::Roof, roof);
        appendToPart(out, params.wallPart, gableW);
    } else {
        if (params.stringCourse && !params.curtainWall) sweptCornice(cur, y - 0.45, 1.25);
        emitPlanSlab(out, cur, y + 0.05, 0.2, PartId::Roof,
                     materialFor(PartId::Roof, wallColor).albedo);
        if (params.parapet > 0) {
            // The lip continues the FACADE material (a brick building has a
            // brick parapet) with a trim coping; glass/solid facades keep the
            // whole lip in trim so the ring doesn't read as floating cladding.
            const bool plainLip = params.curtainWall || params.solidFacade;
            emitPlanParapet(out, cur, y + 0.05, params.parapet,
                            plainLip ? materialFor(PartId::Trim, wallColor).albedo
                                     : wallColor,
                            plainLip ? PartId::Trim : params.wallPart,
                            materialFor(PartId::Trim, wallColor).albedo * 0.9);
        }
        // Crown (penthouse + tank) seated on the top tier's oriented frame.
        Vec3 r3(topObb.axis[0].x, 0, topObb.axis[0].y);
        Vec3 f3(topObb.axis[1].x, 0, topObb.axis[1].y);
        Vec3 fo = Vec3(topObb.center.x, 0, topObb.center.y) -
                  r3 * topObb.half[0] - f3 * topObb.half[1];
        emitCrown(out, fo, topObb.half[0] * 2, topObb.half[1] * 2, r3, f3,
                  y + 0.05, params, rng, &cur);
    }
    out.attaches.push_back({Vec3(centroid(cur).x, y + roofRise, centroid(cur).y),
                            Vec3(0, 1, 0), "roof"});
    out.height = (y + (pitched ? roofRise : params.parapet)) - baseY;

    // Coarse HLOD proxy: the plan's oriented box, ground to roof.
    {
        OBB2 obb = orientedBoundingBox(plan);
        Vec3 r3(obb.axis[0].x, 0, obb.axis[0].y), f3(obb.axis[1].x, 0, obb.axis[1].y);
        BuildingMesh scratch;
        emitBox(scratch, Scope{Vec3(obb.center.x, baseY, obb.center.y) -
                                   r3 * obb.half[0] - f3 * obb.half[1],
                               {r3, Vec3(0, 1, 0), f3},
                               Vec3(obb.half[0] * 2, out.height, obb.half[1] * 2)},
                PartId::Wall, wallColor);
        out.proxy = scratch.merged();
    }
    return out;
}

}  // namespace engine
