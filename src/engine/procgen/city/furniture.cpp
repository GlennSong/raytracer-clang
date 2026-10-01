#include "furniture.h"
#include "../furniture_kit.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

namespace {

uint32_t mix32(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}

// THE ROOM FRAME: origin at rect[0], u along rect[0]->rect[1] (the ring room's window wall), v toward rect[3].
struct RoomFrame {
    Vec2 o, u, v;     // unit axes
    Real W = 0, D = 0;
    Vec2 world(Real a, Real b) const { return o + u * a + v * b; }
    Vec2 local(const Vec2& p) const { const Vec2 d = p - o; return Vec2(dot(d, u), dot(d, v)); }
    Vec2 worldDir(Real a, Real b) const { return u * a + v * b; }
};

struct Box2 { Real a0, b0, a1, b1; };   // a local rectangle in the room frame
bool overlaps(const Box2& x, const Box2& y, Real gap) {
    return x.a0 < y.a1 + gap && y.a0 < x.a1 + gap && x.b0 < y.b1 + gap && y.b0 < x.b1 + gap;
}

// A spot against a wall: `side` 0 the v = 0 wall, 1 the u = W wall, 2 the v = D wall, 3 the u = 0 wall; `t` the
// footprint's left end along that wall (seen from inside the room), `w` x `d` its size (along the wall x out).
struct Placement {
    const RoomFrame* f = nullptr;
    int side = 0;
    Real t = 0, w = 0, d = 0;
    Vec2 roomAt(Real x, Real z) const {   // footprint point (x along the wall, z out from it) -> room coords
        switch (side) {
            case 0: return Vec2(t + x, z);
            case 1: return Vec2(f->W - z, t + x);
            case 2: return Vec2(f->W - (t + x), f->D - z);
            default: return Vec2(z, f->D - (t + x));
        }
    }
    Box2 footprint() const {
        const Vec2 a = roomAt(0, 0), b = roomAt(w, d);
        return {std::min(a.x, b.x), std::min(a.y, b.y), std::max(a.x, b.x), std::max(a.y, b.y)};
    }
};

struct Furnisher {
    std::vector<PlacedPiece>& out;
    RenderMesh* collider;
    RoomFrame f;
    Real y0;
    uint32_t variant;
    std::vector<Box2> taken;   // door swings and placed footprints

    // A kit piece in the footprint's frame: its centre `x` along the wall, `z` out, turned to face +z (into the
    // room) or, with `facing` false, back toward the wall (a chair at its desk).
    void put(const Placement& p, Piece pc, Real x, Real z, bool facing = true, Real yOff = 0) {
        const Vec2 a = f.world(p.roomAt(x, z).x, p.roomAt(x, z).y);
        const Vec2 ax = f.world(p.roomAt(x + 1, z).x, p.roomAt(x + 1, z).y) - a;
        const Vec2 az = f.world(p.roomAt(x, z + 1).x, p.roomAt(x, z + 1).y) - a;
        Vec3 X(ax.x, 0, ax.y), Z(az.x, 0, az.y);
        if (!facing) { X = X * -1.0; Z = Z * -1.0; }
        // Right-handed: a mirrored frame would turn the piece inside out.
        if (dot(cross(X, Vec3(0, 1, 0)), Z) < 0) X = X * -1.0;
        PlacedPiece pp;
        pp.piece = static_cast<uint8_t>(pc);
        pp.variant = variant;
        Mat4& M = pp.xform;
        M.m[0][0] = X.x; M.m[1][0] = 0; M.m[2][0] = X.z;
        M.m[0][1] = 0;   M.m[1][1] = 1; M.m[2][1] = 0;
        M.m[0][2] = Z.x; M.m[1][2] = 0; M.m[2][2] = Z.z;
        M.m[0][3] = a.x; M.m[1][3] = y0 + yOff; M.m[2][3] = a.y;
        out.push_back(pp);
        const FurniturePiece& kit = furniturePiece(pc, variant);
        if (!kit.solid || !collider) return;
        // The footprint box, the piece's height.
        const Real hw = kit.size.x * 0.5;
        auto W = [&](Real px, Real pz) {
            return Vec3(a.x + X.x * px + Z.x * pz, 0, a.y + X.z * px + Z.z * pz);
        };
        const Vec3 c[4] = {W(-hw, 0), W(hw, 0), W(hw, kit.size.z), W(-hw, kit.size.z)};
        const Real ya = y0 + yOff, yb = y0 + yOff + kit.size.y;
        auto V = [&](int i, Real y) { return Vec3(c[i].x, y, c[i].z); };
        auto q = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3) {
            MeshBuilder::emitTri(*collider, p0, p1, p2, normalize(cross(p1 - p0, p2 - p0)), Vec3(1, 1, 1));
            MeshBuilder::emitTri(*collider, p0, p2, p3, normalize(cross(p2 - p0, p3 - p0)), Vec3(1, 1, 1));
        };
        q(V(0, ya), V(1, ya), V(1, yb), V(0, yb)); q(V(1, ya), V(2, ya), V(2, yb), V(1, yb));
        q(V(2, ya), V(3, ya), V(3, yb), V(2, yb)); q(V(3, ya), V(0, ya), V(0, yb), V(3, yb));
        q(V(0, yb), V(1, yb), V(2, yb), V(3, yb));
    }

    // Find a spot against a wall for a w x d footprint: the preferred sides in order, each tried centred, then
    // at its ends and quarters. False when it fits nowhere.
    bool place(Real w, Real d, const std::vector<int>& sides, Placement& p) {
        for (int s : sides) {
            const Real len = (s == 0 || s == 2) ? f.W : f.D;
            const Real depth = (s == 0 || s == 2) ? f.D : f.W;
            if (w > len - 0.1 || d > depth - 0.7) continue;
            const Real free = len - w - 0.1;
            for (Real k : {0.5, 0.0, 1.0, 0.25, 0.75}) {
                Placement c{&f, s, 0.05 + free * k, w, d};
                const Box2 fp = c.footprint();
                bool clear = true;
                for (const Box2& b : taken)
                    if (overlaps(fp, b, 0.08)) { clear = false; break; }
                if (!clear) continue;
                p = c;
                taken.push_back(fp);
                return true;
            }
        }
        return false;
    }
};

}  // namespace

void emitFurniture(std::vector<PlacedPiece>& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, uint32_t seed) {
    std::vector<Vec2> doors;
    for (const RoomWall& w : rp.walls)
        if (w.doorAt >= 0) doors.push_back(w.a + (w.b - w.a) * w.doorAt);
    const uint32_t hb = mix32(seed ^ 0x6a09e667u);
    // The building's wood and kitchen style; the fabric varies a little room to room within its palette.
    const uint32_t wood = (hb >> 3) & 3u, style = (hb >> 9) & 3u;

    for (const Room& room : rp.rooms) {
        if (room.rect.size() != 4 || room.kind == RoomKind::Hall) continue;
        const Vec2 rc = (room.rect[0] + room.rect[2]) * 0.5;
        const uint32_t hr = mix32(hb ^ static_cast<uint32_t>(std::lround(rc.x * 7.0)) ^
                                  (static_cast<uint32_t>(std::lround(rc.y * 13.0)) << 8));
        const uint32_t fabric = (hb + ((hr >> 5) % 3u)) & 7u;
        Furnisher F{out, colliderOut, {}, y0, fabric | (wood << 3) | (style << 5), {}};
        F.f.o = room.rect[0];
        const Vec2 du = room.rect[1] - room.rect[0], dv = room.rect[3] - room.rect[0];
        F.f.W = du.length(); F.f.D = dv.length();
        if (F.f.W < 1.8 || F.f.D < 1.8) continue;
        F.f.u = du * (1.0 / F.f.W); F.f.v = dv * (1.0 / F.f.D);
        for (const Vec2& dp : doors) {
            const Vec2 l = F.f.local(dp);
            if (l.x < -0.4 || l.x > F.f.W + 0.4 || l.y < -0.4 || l.y > F.f.D + 0.4) continue;
            F.taken.push_back({l.x - 0.6, l.y - 0.6, l.x + 0.6, l.y + 0.6});
        }
        // Sides by preference: a ring room's window wall is side 0, its front (door) side 2; a house room goes
        // longest wall first.
        const bool ring = rp.topology == PlateTopology::Ring;
        const std::vector<int> longFirst = F.f.W >= F.f.D ? std::vector<int>{0, 2, 1, 3} : std::vector<int>{1, 3, 0, 2};
        Placement p;
        switch (room.kind) {
            case RoomKind::Office: {
                // The DESK backs onto the window with its chair in front, facing it, and the monitor on top.
                const int desks = F.f.W > 6.8 ? 2 : 1;
                for (int k = 0; k < desks; ++k) {
                    if (!F.place(1.6, 1.6, ring ? std::vector<int>{0, 1, 3} : longFirst, p)) break;
                    F.put(p, Piece::Desk, 0.8, 0.0);
                    F.put(p, Piece::Monitor, 0.8, 0.12, true, 0.75);
                    F.put(p, Piece::OfficeChair, 0.8, 1.55, false);
                }
                if (F.place(0.46, 0.6, {1, 3, 2}, p)) F.put(p, Piece::FilingCabinet, 0.23, 0.0);
                if (F.f.W > 5.5 && F.place(0.9, 0.9, {2, 1, 3}, p)) F.put(p, Piece::Planter, 0.45, 0.0);
                break;
            }
            case RoomKind::Flat:
            case RoomKind::Bed: {
                // The BED, head to a wall, between two nightstands.
                if (F.place(2.6, 2.12, ring ? std::vector<int>{1, 3, 0} : longFirst, p)) {
                    F.put(p, Piece::Nightstand, 0.25, 0.0);
                    F.put(p, Piece::Bed, 1.3, 0.0);
                    F.put(p, Piece::Nightstand, 2.35, 0.0);
                }
                if (room.kind == RoomKind::Flat) {
                    if (F.place(2.16, 1.75, {2, 0, 1, 3}, p)) {
                        F.put(p, Piece::Sofa, 1.08, 0.0);
                        F.put(p, Piece::CoffeeTable, 1.08, 1.15);
                    }
                } else if (F.place(1.2, 0.6, {1, 3, 0, 2}, p)) {
                    F.put(p, Piece::Wardrobe, 0.6, 0.0);
                }
                break;
            }
            case RoomKind::Living: {
                Placement sofa;
                if (F.place(2.16, 1.75, longFirst, sofa)) {
                    F.put(sofa, Piece::Sofa, 1.08, 0.0);
                    F.put(sofa, Piece::CoffeeTable, 1.08, 1.15);
                    const int opp = (sofa.side + 2) % 4;
                    if (F.place(1.6, 0.42, {opp}, p)) F.put(p, Piece::TvUnit, 0.8, 0.0);
                }
                if (F.place(0.84, 0.84, {1, 3, 0, 2}, p)) F.put(p, Piece::LoungeChair, 0.42, 0.0);
                break;
            }
            case RoomKind::Kitchen: {
                // The COUNTER RUN: as many 0.6 m modules as the longest free wall takes (2 to 6), the fridge tower
                // at one end, the sink and the hob among the base units, a wall cupboard over each base.
                const Real wall = std::max(F.f.W, F.f.D);
                const int mods = std::clamp(static_cast<int>((wall - 1.0) / 0.6), 2, 6);
                if (F.place(mods * 0.6, 0.64, longFirst, p)) {
                    for (int i = 0; i < mods; ++i) {
                        const Real x = 0.3 + i * 0.6;
                        Piece pc = Piece::KitchenBase;
                        if (i == 0 && mods >= 4) pc = Piece::KitchenTall;
                        else if (i == mods / 2) pc = Piece::KitchenSink;
                        else if (i == mods - 1 || (mods == 2 && i == 0)) pc = Piece::KitchenHob;
                        F.put(p, pc, x, 0.0);
                        if (pc != Piece::KitchenTall) F.put(p, Piece::KitchenWall, x, 0.0);
                    }
                }
                if (F.place(1.4, 1.85, {0, 1, 2, 3}, p)) {
                    F.put(p, Piece::DiningTable, 0.7, 0.5);
                    F.put(p, Piece::DiningChair, 0.7, 0.0 + 0.0, true);
                    F.put(p, Piece::DiningChair, 0.7, 1.85, false);
                }
                break;
            }
            case RoomKind::Bath: {
                if (F.place(1.7, 0.75, longFirst, p)) F.put(p, Piece::Bathtub, 0.85, 0.0);
                if (F.place(0.4, 0.7, {1, 3, 0, 2}, p)) F.put(p, Piece::Toilet, 0.2, 0.0);
                if (F.place(0.8, 0.5, {0, 1, 2, 3}, p)) F.put(p, Piece::Vanity, 0.4, 0.0);
                break;
            }
            default: break;
        }
    }
}

}  // namespace engine
