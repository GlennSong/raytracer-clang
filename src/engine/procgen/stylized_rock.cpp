#include "stylized_rock.h"
#include "noise.h"
#include "proc_rng.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace engine {

bool rockFamilyFromName(const std::string& n, RockFamily& out) {
    if (n == "boulder") out = RockFamily::Boulder;
    else if (n == "slab") out = RockFamily::Slab;
    else if (n == "pebbles") out = RockFamily::Pebbles;
    else if (n == "outcrop") out = RockFamily::Outcrop;
    else return false;
    return true;
}
bool rockMaterialFromName(const std::string& n, RockMaterial& out) {
    if (n == "granite") out = RockMaterial::Granite;
    else if (n == "sandstone") out = RockMaterial::Sandstone;
    else if (n == "basalt") out = RockMaterial::Basalt;
    else if (n == "mossy") out = RockMaterial::Mossy;
    else return false;
    return true;
}

namespace {

struct Palette { Vec3 base, dark; double moss; };
Palette paletteFor(RockMaterial m) {
    switch (m) {
        // linear albedo, warm-neutral: stylized stone reads best a little lighter than real rock
        case RockMaterial::Sandstone: return {{0.30, 0.215, 0.135}, {0.17, 0.115, 0.07}, 0.15};
        case RockMaterial::Basalt:    return {{0.085, 0.080, 0.080}, {0.045, 0.043, 0.043}, 0.25};
        case RockMaterial::Mossy:     return {{0.19, 0.185, 0.165}, {0.10, 0.097, 0.088}, 0.9};
        default:                      return {{0.21, 0.200, 0.180}, {0.11, 0.105, 0.095}, 0.25};
    }
}

// One stone into `m`, as a recipe over the shape kit (MeshBuilder): an icosphere lumped by noise,
// cut by a few planes into crisp breaks (and a flat foot), scaled, FACETED, coloured per facet
// (moss on the faces that look up, a dark foot), its normals leaned a little toward the stone's
// own volume -- semi-faceted.
void stone(RenderMesh& m, const Vec3& at, const Vec3& scale, int subdiv, int cuts, double lumps, ProcRng& rng,
           const Noise& noise, const Palette& pal, double moss, double footY, bool flatTop = false) {
    RenderMesh st = MeshBuilder::icosphere(subdiv);
    const Vec3 o(0, 0, 0);
    MeshBuilder::displaceNoise(st, o, lumps, 1.7, static_cast<uint32_t>(rng.next() * 4294967295.0));
    for (int c = 0; c < cuts; ++c) {
        const double a = rng.in(0, 6.2831853), el = rng.in(-0.3, 1.2);
        MeshBuilder::cutByPlane(st, o, Vec3(std::cos(a) * std::cos(el), std::sin(el), std::sin(a) * std::cos(el)), rng.in(0.55, 0.85));
    }
    if (flatTop)   // a weathered, near-level top instead of a spire
        MeshBuilder::cutByPlane(st, o, Vec3(rng.in(-0.25, 0.25), 1.0, rng.in(-0.25, 0.25)), rng.in(0.45, 0.7));
    MeshBuilder::cutByPlane(st, o, Vec3(0, -1, 0), 0.35);   // the flat foot it stands on
    MeshBuilder::deform(st, [&](const Vec3& q) { return at + Vec3(q.x * scale.x, (q.y + 0.35) * scale.y, q.z * scale.z); });
    const Vec3 c0 = at + Vec3(0, 0.5 * scale.y, 0);
    MeshBuilder::facet(st, c0);
    (void)noise; (void)pal; (void)moss;
    // The colour is the MATERIAL's (a triplanar stone texture) and its moss the material's top
    // layer (ADR-0098); the mesh carries only light: a slight shade per facet, a darker foot.
    MeshBuilder::colorBy(st, [&](const Vertex& v) {
        const uint32_t hf = static_cast<uint32_t>(std::llround(v.normal.x * 977.0) * 73856093LL ^ std::llround(v.normal.y * 977.0) * 19349663LL ^
                                                  std::llround(v.normal.z * 977.0) * 83492791LL);
        const double shade = 0.94 + 0.12 * ((hf * 2654435761u) >> 8) / 16777215.0;
        const double foot = std::clamp((v.position.y - footY) / std::max(0.05, 0.25 * scale.y), 0.0, 1.0);
        return Vec3(1, 1, 1) * (shade * (0.55 + 0.45 * foot));
    });
    // smooth-ish shading over a low-poly silhouette: the facets still break the outline
    MeshBuilder::leanNormals(st, c0, scale * 0.5, 0.55);
    MeshBuilder::append(m, st);
}

}  // namespace

double rockDefaultMoss(RockMaterial m) { return paletteFor(m).moss; }

RenderMesh stylizedRock(uint32_t seed, const StylizedRockParams& p) {
    RenderMesh m;
    ProcRng rng(seed);
    const Noise noise(seed * 31u + 7u);
    const Palette pal = paletteFor(p.material);
    const double moss = p.moss >= 0 ? p.moss : pal.moss;
    const double s = p.size * rng.in(0.8, 1.2);
    switch (p.family) {
        case RockFamily::Slab:
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.8 * rng.in(0.9, 1.3), s * 0.28 * rng.in(0.8, 1.2), s * 0.6 * rng.in(0.9, 1.2)), 1,
                  3, 0.18, rng, noise, pal, moss * 1.2, 0.0);
            break;
        case RockFamily::Pebbles: {
            const int n = 5 + static_cast<int>(rng.next() * 5);
            for (int i = 0; i < n; ++i) {
                const double a = rng.in(0, 6.2831853), r = s * 0.5 * std::sqrt(rng.next());
                const double ps = s * rng.in(0.12, 0.24);
                stone(m, Vec3(std::cos(a) * r, 0, std::sin(a) * r), Vec3(ps, ps * rng.in(0.5, 0.8), ps * rng.in(0.8, 1.1)), 0, 1,
                      0.15, rng, noise, pal, moss * 0.5, 0.0);
            }
            break;
        }
        case RockFamily::Outcrop: {
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.5, s * 0.85 * rng.in(0.85, 1.2), s * 0.45), 1, 5, 0.22, rng, noise, pal, moss, 0.0, true);
            if (rng.next() < 0.7)   // a leaning second shard
                stone(m, Vec3(s * rng.in(0.25, 0.4), 0, s * rng.in(-0.2, 0.2)), Vec3(s * 0.34, s * 0.6 * rng.in(0.8, 1.2), s * 0.32), 1, 4,
                      0.2, rng, noise, pal, moss, 0.0, true);
            break;
        }
        default:
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.55 * rng.in(0.9, 1.2), s * 0.42 * rng.in(0.8, 1.2), s * 0.5 * rng.in(0.9, 1.2)), 1,
                  2, 0.25, rng, noise, pal, moss, 0.0);
            break;
    }
    // THE SEAM: a skirt of small stones around a boulder's or an outcrop's foot, so where the
    // rock meets the ground reads as rubble settling against it, not a mesh cut by a plane.
    if (p.family == RockFamily::Boulder || p.family == RockFamily::Outcrop) {
        const double reach = p.family == RockFamily::Outcrop ? 0.5 : 0.62;
        const int n = 3 + static_cast<int>(rng.next() * 4);
        for (int i = 0; i < n; ++i) {
            const double a = rng.in(0, 6.2831853), r = s * reach * rng.in(0.85, 1.25);
            const double ps = s * rng.in(0.07, 0.14);
            stone(m, Vec3(std::cos(a) * r, -ps * 0.15, std::sin(a) * r), Vec3(ps, ps * rng.in(0.5, 0.8), ps * rng.in(0.8, 1.1)), 0, 1,
                  0.15, rng, noise, pal, moss * 0.4, 0.0);
        }
    }
    return m;
}

}  // namespace engine
