#include "shape_grammar.h"

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
        case PartId::Wall:
        default:
            m.albedo = wallColor; m.metallic = 0.0f; m.roughness = 0.75f; break;
    }
    return m;
}

void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& normal, const Vec3& color) {
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    Vec3 tan = normalize(b - a);
    auto vtx = [&](const Vec3& p, float u, float vv) {
        Vertex v(p, normal, tan, u, vv);
        v.color = color;
        return v;
    };
    mesh.vertices.push_back(vtx(a, 0, 0));
    mesh.vertices.push_back(vtx(b, 1, 0));
    mesh.vertices.push_back(vtx(c, 1, 1));
    mesh.vertices.push_back(vtx(d, 0, 1));
    // Engine winding convention (matches MeshBuilder::box, which renders correctly
    // under the viewer's back-face culling): the triangle's geometric normal points
    // OPPOSITE the outward shading `normal` (geo·normal < 0). The offline path
    // tracer is two-sided so it never caught this; the Metal viewer culls, so a
    // flipped winding renders the building inside-out.
    Vec3 geo = cross(b - a, c - a);
    if (dot(geo, normal) <= 0)
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    else
        mesh.indices.insert(mesh.indices.end(), {base, base + 2, base + 1, base, base + 3, base + 2});
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

Scope scopeFromFootprint(const Poly2& footprint, Real baseY, Real height) {
    OBB2 obb = orientedBoundingBox(footprint);
    Scope s;
    // Long OBB axis -> forward (facades face the long sides); short -> right.
    int la = obb.longAxis(), sa = 1 - la;
    Vec2 fwd = obb.axis[la], rgt = obb.axis[sa];
    Real fwdHalf = obb.half[la], rgtHalf = obb.half[sa];
    s.axis[0] = normalize(Vec3(rgt.x, 0, rgt.y));
    s.axis[1] = Vec3(0, 1, 0);
    s.axis[2] = normalize(Vec3(fwd.x, 0, fwd.y));
    s.size = Vec3(rgtHalf * 2, height, fwdHalf * 2);
    Vec3 centerXZ(obb.center.x, baseY, obb.center.y);
    s.origin = centerXZ - s.axis[0] * rgtHalf - s.axis[2] * fwdHalf;
    return s;
}

// --- Facade subdivision -----------------------------------------------------

namespace {

enum class FacadeMode { Residential, Retail, Entrance, Solid };

// Subdivide one face into window bays. Wall margins around each window read as
// mullions/piers; the window is recessed by `inset` (Glass). In Entrance mode the
// centre bay is a real door-height opening (no fill) so the shell is enterable.
void emitFacade(BuildingMesh& out, const Scope& storey, int side, FacadeMode mode,
                const BuildingParams& p, const Vec3& wallColor) {
    FaceRect fr = faceOf(storey, side);
    // Accumulate into locals, then append once each — never hold a part reference
    // across a partMesh() that could reallocate out.parts.
    RenderMesh wall, glass, door;

    int bays = std::max(1, static_cast<int>(std::lround(fr.width / std::max(p.bayWidth, Real(0.5)))));
    Real bw = fr.width / bays;
    Real fh = fr.height;

    Real sill, head, margin;
    if (mode == FacadeMode::Solid) {           // warehouse: small high clerestory
        sill = fh * 0.66; head = fh * 0.84; margin = std::min(bw * 0.36, Real(1.4));
    } else {
        sill = (mode == FacadeMode::Retail) ? 0.4 : human::WINDOW_SILL;
        head = std::min(fh - 0.4, (mode == FacadeMode::Retail) ? fh - 0.4 : human::WINDOW_HEAD);
        margin = std::min(bw * 0.22, Real(0.6));
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
        wallQuad(x0, x1, 0, openSill);                       // below opening
        wallQuad(x0, x1, openHead, fh);                      // above opening
        wallQuad(x0, wx0, openSill, openHead);               // left pier
        wallQuad(wx1, x1, openSill, openHead);               // right pier

        if (entrance) {
            // A real opening: a recessed dark threshold so it reads as a doorway
            // you can pass through (ADR-0038 §4).
            Vec3 in = fr.n * -0.15;
            emitQuad(door, fr.at(wx0, 0) + in, fr.at(wx1, 0) + in,
                     fr.at(wx1, openHead) + in, fr.at(wx0, openHead) + in, fr.n,
                     materialFor(PartId::Door, wallColor).albedo);
        } else {
            Vec3 in = fr.n * (-p.windowInset);
            // Window reveals (jambs/sill/lintel): close the recess between the wall
            // opening and the inset glass, so you don't see straight through the
            // (hollow) building around the glass. Four quads from the opening edge
            // back to the glass, normals facing into the recess.
            Vec3 oBL = fr.at(wx0, openSill), oBR = fr.at(wx1, openSill);
            Vec3 oTL = fr.at(wx0, openHead), oTR = fr.at(wx1, openHead);
            Vec3 gBL = oBL + in, gBR = oBR + in, gTL = oTL + in, gTR = oTR + in;
            Vec3 rev = wallColor * 0.82;
            emitQuad(wall, oTL, oTR, gTR, gTL, fr.v * -1, rev);   // lintel (faces down)
            emitQuad(wall, oBL, oBR, gBR, gBL, fr.v,      rev);   // sill   (faces up)
            emitQuad(wall, oBL, oTL, gTL, gBL, fr.h,      rev);   // left jamb (faces +h)
            emitQuad(wall, oBR, oTR, gTR, gBR, fr.h * -1, rev);   // right jamb
            emitQuad(glass, gBL, gBR, gTR, gTL, fr.n,
                     materialFor(PartId::Glass, wallColor).albedo);
        }
    }

    // Pilasters: thin vertical piers proud of the wall at each bay boundary. Per
    // storey they stack into continuous full-height pillars. A box projecting
    // outward by `proud`, under Trim.
    if (p.pilasters && mode != FacadeMode::Entrance) {
        RenderMesh trim;
        Real proud = 0.18, pw = 0.5;
        Vec3 up = storey.axis[1];
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
    int sides = std::max(16, p.sides);
    Vec3 wall = p.wallColor;
    Vec3 glass = materialFor(PartId::Glass, wall).albedo;
    Real y = baseY;

    Real gh = p.groundHeight;
    emitTube(out, cXZ, R, y, y + 0.5, sides, PartId::Trim, p.trimColor);          // base ring
    emitTube(out, cXZ, R * 0.99, y + 0.5, y + gh - 0.3, sides, PartId::Glass, glass);  // lobby glass
    emitTube(out, cXZ, R, y + gh - 0.3, y + gh, sides, PartId::Trim, p.trimColor);     // cornice ring
    y += gh;
    for (int i = 0; i < p.floors; ++i) {
        Real fh = p.floorHeight;
        emitTube(out, cXZ, R, y, y + 0.9, sides, PartId::Wall, wall);             // spandrel band
        emitTube(out, cXZ, R * 0.985, y + 0.9, y + fh, sides, PartId::Glass, glass);  // window band
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

    // Ground floor: taller, glassy retail (or lobby), walkable shell with a real
    // entrance on the front face (ADR-0038 §4).
    Real gh = params.groundHeight;
    Scope ground = storeyScope(y, gh);
    FacadeMode groundMode = params.solidFacade ? FacadeMode::Solid
                          : params.groundRetail ? FacadeMode::Retail
                                                : FacadeMode::Residential;
    for (int side = 0; side < 4; ++side) {
        FacadeMode mode = (side == 0 && params.walkableGround) ? FacadeMode::Entrance : groundMode;
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
            if (side == 0 && params.walkableGround) continue;   // entrance breaks it
            FaceRect fr = faceOf(ground, side);
            Vec3 out = fr.n * proud;
            Vec3 b0 = fr.at(0, 0), b1 = fr.at(fr.width, 0);
            Vec3 t0 = fr.at(0, bh), t1 = fr.at(fr.width, bh);
            emitQuad(band, b0 + out, b1 + out, t1 + out, t0 + out, fr.n, col);  // face
            emitQuad(band, t0 + out, t1 + out, t1, t0, fr.v, col);              // top ledge
        }
        appendToPart(out, PartId::Trim, band);
    }
    {
        FaceRect ef = faceOf(ground, 0);
        out.attaches.push_back({ef.at(ef.width * 0.5, 0), ef.n, "entrance"});
        // Awning: a projecting ledge over the entrance (ground floor).
        if (params.awning) {
            Real dw = std::min(human::DOOR_WIDTH + 1.6, ef.width * 0.5);
            Vec3 c = ef.at(ef.width * 0.5, human::DOOR_HEIGHT + 0.15);
            Vec3 across = ef.h, out_n = ef.n;
            Scope a;
            a.axis[0] = across; a.axis[1] = Vec3(0, 1, 0); a.axis[2] = out_n;
            a.size = Vec3(dw, 0.18, 1.3);
            a.origin = c - across * (dw * 0.5);
            emitBox(out, a, PartId::Detail, params.trimColor);
        }
    }
    // String course / cornice: an oversailing band capping the ground floor
    // (separates the taller retail/lobby base from the floors above, and caps the
    // base piers). Wraps the whole perimeter — a cornice runs over the entrance.
    if (params.stringCourse) {
        Real grow = 0.14, h = 0.36, yb = y - 0.18;
        Scope sc{Vec3(footOrigin.x, yb, footOrigin.z) - r * grow - f * grow,
                 {r, Vec3(0, 1, 0), f}, Vec3(width + 2 * grow, h, depth + 2 * grow)};
        emitBox(out, sc, PartId::Trim, params.trimColor);
    }
    y += gh;

    // Upper floors carry no pilasters — base piers belong to the base only
    // (ADR-0040), capped by the string course above.
    BuildingParams upper = params;
    upper.pilasters = false;

    // Upper residential floors, with optional setbacks.
    for (int i = 0; i < params.floors; ++i) {
        if (params.setbackFloors > 0 && params.setbackEvery > 0 && i > 0 &&
            i % params.setbackFloors == 0) {
            Real d = params.setbackEvery;
            Real dx = std::min(d, width * 0.4), dz = std::min(d, depth * 0.4);
            footOrigin = footOrigin + r * dx + f * dz;
            width -= 2 * dx; depth -= 2 * dz;
        }
        Real fh = params.floorHeight;
        Scope storey = storeyScope(y, fh);
        FacadeMode mode = params.solidFacade ? FacadeMode::Solid
                        : params.curtainWall ? FacadeMode::Retail
                                             : FacadeMode::Residential;
        for (int side = 0; side < 4; ++side)
            emitFacade(out, storey, side, mode, upper, wallColor);
        if (i == params.floors / 2) {
            FaceRect ff = faceOf(storey, 0);
            out.attaches.push_back({ff.at(ff.width * 0.5, fh * 0.5), ff.n, "facade"});
        }
        y += fh;
    }

    // Roof: a flat slab + a parapet railing around the perimeter (ADR-0038 §4).
    emitBox(out, Scope{Vec3(footOrigin.x, y - 0.05, footOrigin.z),
                       {r, Vec3(0, 1, 0), f}, Vec3(width, 0.2, depth)},
            PartId::Roof, materialFor(PartId::Roof, wallColor).albedo);
    if (params.parapet > 0) {
        Scope para = storeyScope(y, params.parapet);
        emitShell(out, para, PartId::Trim, materialFor(PartId::Trim, wallColor).albedo,
                  false, false);
    }
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

}  // namespace engine
