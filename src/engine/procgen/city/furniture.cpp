#include "furniture.h"
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
};

struct Box2 { Real a0, b0, a1, b1; };   // a local rectangle in the room frame
bool overlaps(const Box2& x, const Box2& y, Real gap) {
    return x.a0 < y.a1 + gap && y.a0 < x.a1 + gap && x.b0 < y.b1 + gap && y.b0 < x.b1 + gap;
}

// A piece's own space: x along the wall it backs onto (0..w), z out from that wall (0..d), y up. `side` says
// which wall: 0 the v = 0 wall, 1 the u = W wall, 2 the v = D wall, 3 the u = 0 wall; `t` the piece's left end
// along that wall, measured so the piece faces into the room.
struct Placement {
    const RoomFrame* f = nullptr;
    int side = 0;
    Real t = 0, w = 0, d = 0;
    // local room coordinates of piece point (x, z)
    Vec2 roomAt(Real x, Real z) const {
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
    FurnitureMeshes& out;
    RenderMesh* collider;
    RoomFrame f;
    Real y0;
    std::vector<Box2> taken;   // door swings and placed pieces

    // A box in the piece's space -> the room, five faces (no underside); `solid` also adds it to the collider.
    void box(const Placement& p, Real x0, Real z0, Real x1, Real z1, Real ya, Real yb, const Vec3& col, bool wood,
             bool solid = false) {
        RenderMesh& m = wood ? out.wood : out.soft;
        const Vec2 c[4] = {f.world(p.roomAt(x0, z0).x, p.roomAt(x0, z0).y), f.world(p.roomAt(x1, z0).x, p.roomAt(x1, z0).y),
                           f.world(p.roomAt(x1, z1).x, p.roomAt(x1, z1).y), f.world(p.roomAt(x0, z1).x, p.roomAt(x0, z1).y)};
        auto V = [&](int i, Real y) { return Vec3(c[i].x, y0 + y, c[i].y); };
        // Orientation-safe: each face's normal from its own corners, pointing away from the box centre.
        const Vec2 cc = (c[0] + c[1] + c[2] + c[3]) * 0.25;
        auto face = [&](int i, int j) {
            Vec3 a = V(i, ya), b = V(j, ya), cU = V(j, yb), d = V(i, yb);
            const Vec2 mid = (c[i] + c[j]) * 0.5;
            Vec3 n(mid.x - cc.x, 0, mid.y - cc.y);
            const Real l = std::sqrt(n.x * n.x + n.z * n.z);
            if (l < 1e-9) return;
            n = n * (1.0 / l);
            if (dot(cross(b - a, d - a), n) < 0) { std::swap(a, b); std::swap(cU, d); }
            MeshBuilder::emitQuad(m, a, b, cU, d, n, col);
        };
        face(0, 1); face(1, 2); face(2, 3); face(3, 0);
        Vec3 t0 = V(0, yb), t1 = V(1, yb), t2 = V(2, yb), t3 = V(3, yb);
        if (dot(cross(t1 - t0, t3 - t0), Vec3(0, 1, 0)) < 0) { std::swap(t1, t3); }
        MeshBuilder::emitQuad(m, t0, t1, t2, t3, Vec3(0, 1, 0), col);
        if (solid && collider) {
            Vec3 b0 = V(0, ya), b1 = V(1, ya), b2 = V(2, ya), b3 = V(3, ya);
            Vec3 u0 = V(0, yb), u1 = V(1, yb), u2 = V(2, yb), u3 = V(3, yb);
            auto q = [&](const Vec3& a, const Vec3& b, const Vec3& cq, const Vec3& d) {
                MeshBuilder::emitTri(*collider, a, b, cq, normalize(cross(b - a, cq - a)), Vec3(1, 1, 1));
                MeshBuilder::emitTri(*collider, a, cq, d, normalize(cross(cq - a, d - a)), Vec3(1, 1, 1));
            };
            q(b0, b1, u1, u0); q(b1, b2, u2, u1); q(b2, b3, u3, u2); q(b3, b0, u0, u3); q(u0, u1, u2, u3); q(b0, b3, b2, b1);
        }
    }

    // Find a spot against a wall for a w x d piece: the preferred sides in order, each tried centred, then at its
    // quarters and ends. Returns false when the piece fits nowhere.
    bool place(Real w, Real d, std::initializer_list<int> sides, Placement& p, bool centreFirst = true) {
        for (int s : sides) {
            const Real len = (s == 0 || s == 2) ? f.W : f.D;
            const Real depth = (s == 0 || s == 2) ? f.D : f.W;
            if (w > len - 0.1 || d > depth - 0.6) continue;
            const Real free = len - w - 0.1;
            std::vector<Real> ts;
            if (centreFirst) ts.push_back(0.05 + free * 0.5);
            for (Real k : {0.0, 1.0, 0.25, 0.75}) ts.push_back(0.05 + free * k);
            for (Real t : ts) {
                Placement c{&f, s, t, w, d};
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

void emitFurniture(FurnitureMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, uint32_t seed) {
    // Door points: the centre of every doorway in the storey's walls.
    std::vector<Vec2> doors;
    for (const RoomWall& w : rp.walls)
        if (w.doorAt >= 0) doors.push_back(w.a + (w.b - w.a) * w.doorAt);

    // The building's palette: one wood, one fabric family.
    const uint32_t hb = mix32(seed ^ 0x6a09e667u);
    static const Vec3 kWood[4] = {{0.62, 0.46, 0.30}, {0.38, 0.25, 0.16}, {0.80, 0.70, 0.55}, {0.30, 0.22, 0.18}};
    static const Vec3 kFabric[6] = {{0.42, 0.45, 0.50}, {0.30, 0.38, 0.52}, {0.40, 0.48, 0.38},
                                    {0.62, 0.36, 0.28}, {0.66, 0.60, 0.50}, {0.25, 0.25, 0.27}};
    const Vec3 wood = kWood[hb % 4];
    const Vec3 dark(0.12, 0.12, 0.13), white(0.92, 0.92, 0.90), steel(0.55, 0.57, 0.60), linen(0.94, 0.93, 0.90);

    for (const Room& room : rp.rooms) {
        if (room.rect.size() != 4 || room.kind == RoomKind::Hall) continue;
        Furnisher F{out, colliderOut, {}, y0, {}};
        F.f.o = room.rect[0];
        const Vec2 du = room.rect[1] - room.rect[0], dv = room.rect[3] - room.rect[0];
        F.f.W = du.length(); F.f.D = dv.length();
        if (F.f.W < 1.8 || F.f.D < 1.8) continue;
        F.f.u = du * (1.0 / F.f.W); F.f.v = dv * (1.0 / F.f.D);
        // Door swings: a 1.2 m square inside the room at each doorway on (or near) its edge.
        for (const Vec2& dp : doors) {
            const Vec2 l = F.f.local(dp);
            if (l.x < -0.4 || l.x > F.f.W + 0.4 || l.y < -0.4 || l.y > F.f.D + 0.4) continue;
            F.taken.push_back({l.x - 0.6, l.y - 0.6, l.x + 0.6, l.y + 0.6});
        }
        const Vec2 rc = (room.rect[0] + room.rect[2]) * 0.5;
        const uint32_t hr = mix32(hb ^ static_cast<uint32_t>(std::lround(rc.x * 7.0)) ^
                                  (static_cast<uint32_t>(std::lround(rc.y * 13.0)) << 8));
        const Vec3 fabric = kFabric[(hr >> 4) % 6];
        Placement p;
        // The sides by preference. A ring room's window wall is side 0 and its front (door) side 2; a house room
        // has no such order, so its sides go longest first.
        const bool ring = rp.topology == PlateTopology::Ring;
        const bool wideU = F.f.W >= F.f.D;
        auto longFirst = [&](std::initializer_list<int> fallback) {
            return wideU ? std::vector<int>{0, 2, 1, 3} : std::vector<int>{1, 3, 0, 2};
            (void)fallback;
        };
        auto placeAny = [&](Real w, Real d, const std::vector<int>& sides, Placement& pp) {
            for (int s : sides) if (F.place(w, d, {s}, pp)) return true;
            return false;
        };
        switch (room.kind) {
            case RoomKind::Office: {
                // DESK + chair + monitor, its back to the window wall (the desk faces into the room).
                const int desks = F.f.W > 6.8 ? 2 : 1;
                for (int k = 0; k < desks; ++k) {
                    if (!placeAny(1.6, 1.55, ring ? std::vector<int>{0, 1, 3} : longFirst({}), p)) break;
                    F.box(p, 0, 0, 1.6, 0.8, 0.70, 0.75, wood, true, true);           // top
                    F.box(p, 0.05, 0.02, 1.55, 0.06, 0.0, 0.70, steel, false);         // modesty panel
                    F.box(p, 0.55, 0.12, 1.05, 0.16, 0.75, 1.10, dark, false);         // monitor
                    F.box(p, 0.55, 1.0, 1.05, 1.5, 0.42, 0.48, fabric * 0.6, false);   // chair seat
                    F.box(p, 0.55, 1.45, 1.05, 1.52, 0.48, 1.0, fabric * 0.6, false);  // chair back
                }
                if (placeAny(0.9, 0.5, {1, 3, 2}, p)) F.box(p, 0, 0, 0.9, 0.5, 0.0, 1.1, steel * 0.9, false, true);
                break;
            }
            case RoomKind::Flat:
            case RoomKind::Bed: {
                // BED, head to a wall, a nightstand either side.
                const bool flat = room.kind == RoomKind::Flat;
                if (placeAny(2.5, 2.1, ring ? std::vector<int>{1, 3, 0} : longFirst({}), p)) {
                    F.box(p, 0.45, 0, 2.05, 2.05, 0.0, 0.30, wood, true, true);       // frame
                    F.box(p, 0.5, 0.08, 2.0, 2.0, 0.30, 0.52, linen, false);           // mattress
                    F.box(p, 0.5, 0.75, 2.0, 2.02, 0.52, 0.58, fabric, false);         // duvet
                    F.box(p, 0.45, 0, 2.05, 0.08, 0.30, 1.05, wood * 0.85, true);      // headboard
                    F.box(p, 0.0, 0.0, 0.42, 0.4, 0.0, 0.5, wood, true);               // nightstands
                    F.box(p, 2.08, 0.0, 2.5, 0.4, 0.0, 0.5, wood, true);
                }
                if (flat) {
                    if (placeAny(2.0, 0.9, {2, 0, 1, 3}, p)) {
                        F.box(p, 0, 0, 2.0, 0.9, 0.0, 0.42, fabric, false, true);      // sofa seat
                        F.box(p, 0, 0, 2.0, 0.22, 0.42, 0.85, fabric * 0.9, false);    // back
                    }
                    if (placeAny(0.9, 0.9, {0, 1, 3}, p)) F.box(p, 0, 0, 0.9, 0.9, 0.0, 0.74, wood, true, true);
                } else if (placeAny(1.2, 0.6, {1, 3, 0, 2}, p)) {
                    F.box(p, 0, 0, 1.2, 0.6, 0.0, 2.0, wood * 0.9, true, true);         // wardrobe
                }
                break;
            }
            case RoomKind::Living: {
                // SOFA with its coffee table, the TV on the wall opposite.
                Placement sofa;
                if (placeAny(2.2, 1.9, longFirst({}), sofa)) {
                    F.box(sofa, 0, 0, 2.2, 0.9, 0.0, 0.42, fabric, false, true);
                    F.box(sofa, 0, 0, 2.2, 0.22, 0.42, 0.85, fabric * 0.9, false);
                    F.box(sofa, 0, 0, 0.2, 0.9, 0.42, 0.62, fabric * 0.9, false);
                    F.box(sofa, 2.0, 0, 2.2, 0.9, 0.42, 0.62, fabric * 0.9, false);
                    F.box(sofa, 0.55, 1.3, 1.65, 1.9, 0.0, 0.42, wood, true);          // coffee table
                    const int opp = (sofa.side + 2) % 4;
                    if (F.place(1.6, 0.45, {opp}, p)) {
                        F.box(p, 0, 0, 1.6, 0.45, 0.0, 0.5, wood * 0.8, true, true);    // stand
                        F.box(p, 0.15, 0.15, 1.45, 0.21, 0.55, 1.3, dark, false);       // TV
                    }
                }
                break;
            }
            case RoomKind::Kitchen: {
                // The COUNTER RUN along a wall, the hob and sink in its top; a table and two chairs.
                const Real run = std::min(Real(3.6), std::max(F.f.W, F.f.D) - 0.8);
                if (run >= 1.2 && placeAny(run, 0.6, longFirst({}), p)) {
                    F.box(p, 0, 0, run, 0.58, 0.0, 0.86, (hr & 1u) ? white : fabric * 0.8, false, true);
                    F.box(p, 0, 0, run, 0.62, 0.86, 0.90, Vec3(0.70, 0.70, 0.68), false);
                    F.box(p, run * 0.25, 0.1, run * 0.25 + 0.6, 0.5, 0.90, 0.91, dark, false);   // hob
                }
                if (placeAny(1.2, 1.6, {0, 1, 2, 3}, p)) {
                    F.box(p, 0, 0.4, 1.2, 1.2, 0.71, 0.75, wood, true, true);
                    F.box(p, 0.35, 0.0, 0.85, 0.35, 0.0, 0.46, wood * 0.8, true);       // two chairs
                    F.box(p, 0.35, 1.25, 0.85, 1.6, 0.0, 0.46, wood * 0.8, true);
                }
                break;
            }
            case RoomKind::Bath: {
                if (placeAny(1.7, 0.75, longFirst({}), p)) F.box(p, 0, 0, 1.7, 0.75, 0.0, 0.55, white, false, true);
                if (placeAny(0.42, 0.7, {1, 3, 0, 2}, p)) {
                    F.box(p, 0.0, 0.18, 0.42, 0.7, 0.0, 0.40, white, false);           // bowl
                    F.box(p, 0.0, 0.0, 0.42, 0.18, 0.0, 0.80, white, false);           // cistern
                }
                if (placeAny(0.8, 0.5, {0, 1, 2, 3}, p)) F.box(p, 0, 0, 0.8, 0.5, 0.0, 0.85, white * 0.95, false, true);
                break;
            }
            default: break;
        }
    }
}

}  // namespace engine
