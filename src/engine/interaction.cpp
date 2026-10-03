#include "interaction.h"
#include <algorithm>
#include <cmath>

namespace engine {

void Interactables::refreshBounds() {
    lo = Vec3(1e30, 1e30, 1e30);
    hi = Vec3(-1e30, -1e30, -1e30);
    for (const InteractPiece& p : pieces) {
        const Vec3 o(p.xform.m[0][3], p.xform.m[1][3], p.xform.m[2][3]);
        lo = Vec3(std::min(lo.x, o.x), std::min(lo.y, o.y), std::min(lo.z, o.z));
        hi = Vec3(std::max(hi.x, o.x), std::max(hi.y, o.y), std::max(hi.z, o.z));
    }
}

Vec3 piecePoint(const Mat4& m, const Vec3& p) {
    return Vec3(m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3],
                m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3],
                m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3]);
}

Vec3 pieceDir(const Mat4& m, const Vec3& d) {
    return Vec3(m.m[0][0] * d.x + m.m[0][1] * d.y + m.m[0][2] * d.z,
                m.m[1][0] * d.x + m.m[1][1] * d.y + m.m[1][2] * d.z,
                m.m[2][0] * d.x + m.m[2][1] * d.y + m.m[2][2] * d.z);
}

namespace {

// Where a verb puts the body: the mean of the spots it takes.
Vec3 verbCentre(const FurnitureAsset& a, const FurnVerb& v) {
    Vec3 s(0, 0, 0);
    int n = 0;
    for (std::size_t i = 0; i < a.spots.size(); ++i)
        if (v.spots & (1u << i)) { s = s + a.spots[i].at; ++n; }
    return n ? s * (1.0 / n) : Vec3(0, 0, 0);
}

}  // namespace

bool pieceVerbs(const Interactables& set, uint32_t pi, const FurnitureLibrary& lib, const Vec3& feet, Real reach,
                int& primary, int& secondary, Vec3& anchor, Real& distance) {
    primary = secondary = -1;
    if (pi >= set.pieces.size()) return false;
    const InteractPiece& ip = set.pieces[pi];
    const FurnitureAsset* a = lib.find(static_cast<Piece>(ip.piece));
    if (!a || a->verbs.empty()) return false;
    const Vec3 o(ip.xform.m[0][3], ip.xform.m[1][3], ip.xform.m[2][3]);
    if (std::fabs(o.y - feet.y) > 1.2) return false;   // another floor
    Real primD = 1e30, secD = 1e30;
    Vec3 primAt, secAt;
    for (std::size_t vi = 0; vi < a->verbs.size(); ++vi) {
        const FurnVerb& v = a->verbs[vi];
        if (!verbFree(v, ip.taken)) continue;
        const Vec3 c = piecePoint(ip.xform, verbCentre(*a, v));
        const Real d = Vec3(c.x - feet.x, 0, c.z - feet.z).length();
        if (d > reach) continue;
        if (v.verb == a->verbs[0].verb) { if (d < primD) { primD = d; primary = static_cast<int>(vi); primAt = c; } }
        else if (d < secD) { secD = d; secondary = static_cast<int>(vi); secAt = c; }
    }
    if (primary < 0 && secondary < 0) return false;
    anchor = primary >= 0 ? primAt : secAt;
    distance = std::min(primD, secD);
    return true;
}

InteractChoice findInteraction(const std::vector<std::pair<Entity, const Interactables*>>& sets,
                               const FurnitureLibrary& lib, const Vec3& feet, const Vec3& forward, Real reach) {
    InteractChoice best;
    Real bestScore = 1e30;
    Vec3 fw(forward.x, 0, forward.z);
    const Real fl = fw.length();
    fw = fl > 1e-6 ? fw * (1.0 / fl) : Vec3(0, 0, 0);
    for (const auto& [ent, set] : sets) {
        if (!set) continue;
        const Real pad = reach + 2.5;
        if (feet.x < set->lo.x - pad || feet.x > set->hi.x + pad || feet.z < set->lo.z - pad ||
            feet.z > set->hi.z + pad || feet.y < set->lo.y - 3.0 || feet.y > set->hi.y + 3.0)
            continue;
        for (std::size_t pi = 0; pi < set->pieces.size(); ++pi) {
            int prim = -1, sec = -1;
            Vec3 at;
            Real d = 0;
            if (!pieceVerbs(*set, static_cast<uint32_t>(pi), lib, feet, reach, prim, sec, at, d)) continue;
            Vec3 to(at.x - feet.x, 0, at.z - feet.z);
            const Real facing = d > 1e-6 ? (to.x * fw.x + to.z * fw.z) / std::max(Real(1e-6), to.length()) : 1.0;
            if (d > 0.7 && facing < 0.25) continue;   // behind you: not what you're reaching for
            const Real score = d - 0.5 * facing;
            if (score < bestScore) {
                bestScore = score;
                best.set = ent;
                best.setPtr = set;
                best.piece = static_cast<uint32_t>(pi);
                best.primary = prim;
                best.secondary = sec;
                best.distance = d;
            }
        }
    }
    return best;
}

bool takeInteraction(Interactables& set, Entity setEntity, uint32_t pi, int vi, const FurnitureLibrary& lib,
                     Seated& out) {
    if (pi >= set.pieces.size() || vi < 0) return false;
    InteractPiece& ip = set.pieces[pi];
    const FurnitureAsset* a = lib.find(static_cast<Piece>(ip.piece));
    if (!a || static_cast<std::size_t>(vi) >= a->verbs.size()) return false;
    const FurnVerb& v = a->verbs[static_cast<std::size_t>(vi)];
    if (!verbFree(v, ip.taken)) return false;
    ip.taken |= v.spots;
    out = Seated{};
    out.set = setEntity;
    out.piece = pi;
    out.verb = static_cast<uint8_t>(vi);
    out.spots = v.spots;
    out.eye = piecePoint(ip.xform, v.eye);
    Vec3 lk = pieceDir(ip.xform, v.look);
    const Real ll = lk.length();
    out.look = ll > 1e-6 ? lk * (1.0 / ll) : Vec3(0, 0, 1);
    out.exit = piecePoint(ip.xform, v.exit);
    return true;
}

void releaseInteraction(Interactables* set, const Seated& s) {
    if (!set || s.piece >= set->pieces.size()) return;
    set->pieces[s.piece].taken &= ~s.spots;
}

}  // namespace engine
