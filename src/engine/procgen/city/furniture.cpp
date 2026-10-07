#include "furniture.h"
#include "trades.h"
#include "../furniture_library.h"   // descriptions: anchors to dress, clearances to keep (M3)
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
    Real off = 0;   // side 0 only: how far out from the wall the footprint starts (an open-plan grid's row)
    Vec2 roomAt(Real x, Real z) const {   // footprint point (x along the wall, z out from it) -> room coords
        switch (side) {
            case 0: return Vec2(t + x, off + z);
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
    std::vector<Box2> doorways;   // the door swings alone (a picture keeps clear of them)
    std::vector<Box2> tall;       // footprints of pieces that stand up the wall (no picture over them)
    // THE ROOM'S REAL WALLS, per side, in that side's own coordinate (the Placement t): the stretches an interior
    // wall actually runs along, and its doorways. A picture hangs only on a wall (Glenn, 2026-10-03: "they need to
    // be hung in places on the wall that have room. Some ... were hung in doorways or over windows"): the window wall
    // is the facade, not a partition, and an open plan's side has no wall at all.
    std::vector<std::pair<Real, Real>> wallSpan[4], doorGap[4];

    void learnWalls(const std::vector<RoomWall>& walls) {
        const Real tol = 0.25;
        for (const RoomWall& w : walls) {
            const Vec2 a = f.local(w.a), b = f.local(w.b);
            for (int sd = 0; sd < 4; ++sd) {
                // the side's line and its t coordinate (left end seen from inside), as Placement::roomAt lays it
                auto onSide = [&](const Vec2& q) {
                    switch (sd) {
                        case 0: return std::fabs(q.y) < tol;
                        case 1: return std::fabs(q.x - f.W) < tol;
                        case 2: return std::fabs(q.y - f.D) < tol;
                        default: return std::fabs(q.x) < tol;
                    }
                };
                auto tOf = [&](const Vec2& q) {
                    switch (sd) {
                        case 0: return q.x;
                        case 1: return q.y;
                        case 2: return f.W - q.x;
                        default: return f.D - q.y;
                    }
                };
                if (!onSide(a) || !onSide(b)) continue;
                const Real ta = tOf(a), tb = tOf(b);
                wallSpan[sd].push_back({std::min(ta, tb), std::max(ta, tb)});
                if (w.doorAt >= 0) {
                    const Real td = ta + (tb - ta) * w.doorAt;
                    doorGap[sd].push_back({td - kRoomDoorW * 0.5 - 0.25, td + kRoomDoorW * 0.5 + 0.25});
                }
            }
        }
    }
    // Is [t0, t1] on side `sd` solid wall, clear of every doorway?
    bool onWall(int sd, Real t0, Real t1) const {
        bool covered = false;
        for (const auto& sp : wallSpan[sd]) covered = covered || (sp.first <= t0 + 1e-6 && sp.second >= t1 - 1e-6);
        if (!covered) return false;
        for (const auto& g : doorGap[sd]) if (g.first < t1 && t0 < g.second) return false;
        return true;
    }

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
        if (const FurnitureAsset* fa = FurnitureLibrary::global().find(pc); fa && fa->variety > 0) {
            const uint32_t h = static_cast<uint32_t>(std::lround(a.x * 13.0)) * 73856093u ^
                               static_cast<uint32_t>(std::lround(a.y * 13.0)) * 19349663u;
            pp.variant = (variant & 31u) | ((mix32(h) % static_cast<uint32_t>(fa->variety)) << 5);
        }
        Mat4& M = pp.xform;
        M.m[0][0] = X.x; M.m[1][0] = 0; M.m[2][0] = X.z;
        M.m[0][1] = 0;   M.m[1][1] = 1; M.m[2][1] = 0;
        M.m[0][2] = Z.x; M.m[1][2] = 0; M.m[2][2] = Z.z;
        M.m[0][3] = a.x; M.m[1][3] = y0 + yOff; M.m[2][3] = a.y;
        out.push_back(pp);
        dress(pc, M);
        const FurniturePiece& kit = furniturePiece(pc, pp.variant);
        if (!kit.solid || !collider) return;
        // The footprint box, the piece's height.
        const Real hw = kit.size.x * 0.5;
        auto W = [&](Real px, Real pz) {
            return Vec3(a.x + X.x * px + Z.x * pz, 0, a.y + X.z * px + Z.z * pz);
        };
        const Vec3 c[4] = {W(-hw, 0), W(hw, 0), W(hw, kit.size.z), W(-hw, kit.size.z)};
        const Real ya = y0 + yOff, yb = y0 + yOff + (kit.colliderH > 0 ? kit.colliderH : kit.size.y);
        auto V = [&](int i, Real y) { return Vec3(c[i].x, y, c[i].z); };
        auto q = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3) {
            MeshBuilder::emitTri(*collider, p0, p1, p2, normalize(cross(p1 - p0, p2 - p0)), Vec3(1, 1, 1));
            MeshBuilder::emitTri(*collider, p0, p2, p3, normalize(cross(p2 - p0, p3 - p0)), Vec3(1, 1, 1));
        };
        q(V(0, ya), V(1, ya), V(1, yb), V(0, yb)); q(V(1, ya), V(2, ya), V(2, yb), V(1, yb));
        q(V(2, ya), V(3, ya), V(3, yb), V(2, yb)); q(V(3, ya), V(0, ya), V(0, yb), V(3, yb));
        q(V(0, yb), V(1, yb), V(2, yb), V(3, yb));
    }

    // THE DRESSING (the furniture library, M3): every anchor the placed piece's description carries -- a desk's
    // corner, a nightstand's top -- may take one of the goods it accepts (a lamp, a plant, a stack of books),
    // chosen and turned by a hash of where it stands. Goods are pieces too: instanced, never solid.
    void dress(Piece pc, const Mat4& M) {
        const FurnitureLibrary& lib = FurnitureLibrary::global();
        const FurnitureAsset* a = lib.find(pc);
        if (!a || a->anchors.empty()) return;
        uint32_t h = static_cast<uint32_t>(std::lround(M.m[0][3] * 31.0)) * 73856093u ^
                     static_cast<uint32_t>(std::lround(M.m[2][3] * 17.0)) * 19349663u ^ static_cast<uint32_t>(M.m[1][3] * 7.0);
        for (const FurnAnchor& an : a->anchors) {
            h = h * 1664525u + 1013904223u;
            if ((h >> 8) % 1000u >= static_cast<uint32_t>(an.chance * 1000.0)) continue;
            std::vector<Piece> fits;
            for (Piece g : lib.goodsFor(an.accepts)) {
                const Vec3 sz = furniturePiece(g, 0).size;
                if (sz.x <= an.w + 0.05 && sz.z <= an.d + 0.05) fits.push_back(g);
            }
            if (fits.empty()) continue;
            h = h * 1664525u + 1013904223u;
            const Piece g = fits[(h >> 9) % fits.size()];
            const uint32_t gv = (h >> 13) & 63u;
            const Vec3 sz = furniturePiece(g, gv).size;
            // the good, centred on the anchor, turned a little (nothing on a real desk is square to it)
            const Real turn = an.yaw + (static_cast<Real>((h >> 19) % 61u) - 30.0) * 0.01;
            const Real c = std::cos(turn), sn = std::sin(turn);
            Mat4 L;   // good -> piece: rotate about y, then move its footprint's centre onto the anchor
            L.m[0][0] = c;   L.m[0][2] = sn;
            L.m[2][0] = -sn; L.m[2][2] = c;
            // its footprint's centre (0, 0, size.z / 2) turned lands at (sn, 0, c) * size.z / 2; move that onto the anchor
            L.m[0][3] = an.at.x - sn * sz.z * 0.5;
            L.m[1][3] = an.at.y;
            L.m[2][3] = an.at.z - c * sz.z * 0.5;
            PlacedPiece gp;
            gp.piece = static_cast<uint8_t>(g);
            gp.variant = gv;
            gp.xform = M * L;
            out.push_back(gp);
        }
    }
    // The clearance a piece's description keeps in front of it (`fallback` with no library loaded).
    static Real clearOf(Piece pc, Real fallback) {
        const FurnitureAsset* a = FurnitureLibrary::global().find(pc);
        return a && a->clearFront > 0 ? a->clearFront : fallback;
    }

    // Find a spot against a wall for a w x d footprint: the preferred sides in order, each tried centred, then
    // at its ends and quarters. False when it fits nowhere. `clear`: floor to keep free in front of it too (the
    // piece's CLEARANCE: a wardrobe's doors, a counter's standing room) -- reserved, never furnished.
    bool place(Real w, Real d, const std::vector<int>& sides, Placement& p, bool isTall = false, Real clear = 0) {
        for (int s : sides) {
            const Real len = (s == 0 || s == 2) ? f.W : f.D;
            const Real depth = (s == 0 || s == 2) ? f.D : f.W;
            if (w > len - 0.1 || d > depth - 0.7) continue;
            const Real reach = std::min(d + clear, depth - 0.5);   // the piece and the floor in front of it
            const Real free = len - w - 0.1;
            for (Real k : {0.5, 0.0, 1.0, 0.25, 0.75}) {
                Placement c{&f, s, 0.05 + free * k, w, d};
                Placement zone{&f, s, 0.05 + free * k, w, reach};
                const Box2 fp = c.footprint(), zp = zone.footprint();
                bool clearOk = true;
                for (const Box2& b : taken)
                    if (overlaps(zp, b, 0.08)) { clearOk = false; break; }
                if (!clearOk) continue;
                p = c;
                taken.push_back(zp);
                if (isTall) tall.push_back(fp);
                return true;
            }
        }
        return false;
    }

    // THE OPEN PLAN's grid (buildings C): `pc` footprints (w across x, d out from the window wall, side 0) in rows
    // from the windows, aisles between, every one clear of the rooms set into the zone and of the doorways.
    // `d2` > 0: a shorter fallback footprint (variant `variant2`) tried where the full one does not fit.
    int fill(Piece pc, Real w, Real d, Real aisleX, Real aisleZ, Real d2 = 0, uint32_t variant2 = 0) {
        int placed = 0;
        for (Real z = 0.6; z + std::min(d, d2 > 0 ? d2 : d) <= f.D - 0.8 + 1e-6; z += d + aisleZ)
            for (Real x = 0.6; x + w <= f.W - 0.6 + 1e-6; x += w + aisleX) {
                for (int attempt = 0; attempt < 2; ++attempt) {
                    const Real dd = attempt == 0 ? d : d2;
                    if (dd <= 0 || z + dd > f.D - 0.8 + 1e-6) continue;
                    // the row's offset from the window wall (it used to be dropped: every row landed in the first)
                    Placement c{&f, 0, x, w, dd, z};
                    const Box2 fp = c.footprint();
                    bool clear = true;
                    for (const Box2& b : taken)
                        if (overlaps(fp, b, 0.1)) { clear = false; break; }
                    if (!clear) continue;
                    taken.push_back(fp);
                    const uint32_t keep = variant;
                    if (attempt == 1) variant = variant2;
                    put(c, pc, w * 0.5, 0.0);
                    variant = keep;
                    ++placed;
                    break;
                }
            }
        return placed;
    }

    // WALL ART (Glenn: "the walls are bare. So some pictures hanging on there would be nice"): a picture on the
    // first of `sides` with a clear stretch -- not over a doorway or a tall piece (a wardrobe, a TV, a cupboard);
    // over a sofa or a bed is where pictures go.
    bool hang(const std::vector<int>& sides, uint32_t artVariant, Piece what = Piece::Picture) {
        const FurniturePiece& kit = furniturePiece(what, artVariant);
        const Real w = kit.size.x;
        for (int s : sides) {
            const Real len = (s == 0 || s == 2) ? f.W : f.D;
            if (w > len - 0.6) continue;
            const Real free = len - w - 0.2;
            for (Real k : {0.5, 0.3, 0.7, 0.15, 0.85}) {
                Placement c{&f, s, 0.1 + free * k, w, 0.3};
                if (!onWall(s, c.t - 0.1, c.t + w + 0.1)) continue;
                const Box2 fp = c.footprint();
                bool clear = true;
                for (const Box2& b : doorways) if (overlaps(fp, b, 0.05)) clear = false;
                for (const Box2& b : tall) if (overlaps(fp, b, 0.05)) clear = false;
                if (!clear) continue;
                const uint32_t keep = variant;
                variant = artVariant;
                // on the wall's FACE, not its centre line (a whiteboard was hung inside the partition)
                put(c, what, w * 0.5, kRoomWallT * 0.5 + 0.005);
                variant = keep;
                return true;
            }
        }
        return false;
    }
};

// The program a room of `kind` runs (furniture_rooms.lua); "" for the kinds still furnished in code (shops: by trade).
const char* programFor(RoomKind kind) {
    switch (kind) {
        case RoomKind::Office: return "office";
        case RoomKind::Bed: return "bedroom";
        case RoomKind::Flat: return "flat";
        case RoomKind::Living: return "living";
        case RoomKind::Kitchen: return "kitchen";
        case RoomKind::OpenPlan: return "open_plan";
        case RoomKind::Meeting: return "meeting";
        case RoomKind::Kitchenette: return "kitchenette";
        case RoomKind::Closet: return "closet";
        case RoomKind::Bath: return "bath";
        case RoomKind::Classroom: return "classroom";
        case RoomKind::Lecture: return "lecture_hall";
        case RoomKind::Lab: return "lab";
        case RoomKind::Reading: return "reading_room";
        case RoomKind::Stacks: return "stacks";
        case RoomKind::Dorm: return "dorm";
        default: return "";
    }
}

// RUN A ROOM PROGRAM (M3b): each step in order, every piece picked from the library by name or by family + tags.
struct ProgramRun {
    Furnisher& F;
    const FurnitureLibrary& lib;
    bool ring;
    std::vector<int> longFirst;
    uint32_t hb, hr;
    bool prevPlaced = false;
    int prevSide = 0;

    std::vector<int> sidesOf(const FurnStep& st) const {
        if (st.opposite) return {(prevSide + 2) % 4};
        if (ring && !st.ringSides.empty()) return st.ringSides;
        if (st.longFirst) return longFirst;
        if (!st.sides.empty()) return st.sides;
        return {0, 1, 2, 3};
    }
    Piece pickOf(const FurnPick& p, Real w, Real d) const { return lib.pick(p, w, d, hr); }

    // A GRID whose cells hold a SET (a classroom's desk and its chair) or step up in TIERS (a lecture hall's rows):
    // rows out from the window wall, every cell clear of what is already placed; a row's tier counted from the
    // front, the side away from the windows, where the lectern stands.
    void gridOfSets(const FurnStep& st) {
        const Real zStart = 0.6, zEnd = F.f.D - 0.8, xEnd = F.f.W - 0.6;
        int rows = 0;
        for (Real z = zStart; z + st.d <= zEnd + 1e-6; z += st.d + st.aisleZ) ++rows;
        int r = 0, placed = 0;
        // the clearance kept to what is already placed -- but a grid's own cells may touch (a stack row's bookcases
        // end to end, aisle 0): the margin is never wider than the grid's own aisles, or every other cell drops
        const Real gap = std::min(Real(0.1), std::max(Real(0), std::min(st.aisleX, st.aisleZ) - Real(0.01)));
        const std::size_t ownFrom = F.taken.size();
        for (Real z = zStart; z + st.d <= zEnd + 1e-6; z += st.d + st.aisleZ, ++r) {
            int inRun = 0;
            for (Real x = 0.6; x + st.w <= xEnd + 1e-6; x += st.w + st.aisleX) {
                if (st.runN > 0 && inRun == st.runN) { x += st.crossW; inRun = 0; if (x + st.w > xEnd + 1e-6) break; }
                ++inRun;
                Placement c{&F.f, 0, x, st.w, st.d, z};
                const Box2 fp = c.footprint();
                bool clear = true;
                for (std::size_t bi = 0; bi < F.taken.size(); ++bi)
                    if (overlaps(fp, F.taken[bi], bi >= ownFrom ? gap : Real(0.1))) { clear = false; break; }
                if (!clear) continue;
                F.taken.push_back(fp);
                const uint32_t keep = F.variant;
                if (st.tiers) F.variant = (F.variant & ~(7u << 5)) | (static_cast<uint32_t>(std::min(rows - 1 - r, 7)) << 5);
                if (st.set.empty()) {
                    const Piece pc = pickOf(st.pick, st.w, st.d);
                    if (pc != Piece::Count) F.put(c, pc, st.w * 0.5, 0.0);
                } else {
                    for (const FurnMember& m : st.set) {
                        const Piece pc = pickOf(m.pick, m.fitW > 0 ? m.fitW : st.w, m.fitD > 0 ? m.fitD : st.d);
                        if (pc != Piece::Count) F.put(c, pc, m.x, m.z, m.facing, m.y);
                    }
                }
                F.variant = keep;
                ++placed;
            }
        }
        prevPlaced = placed > 0;
    }

    void run(const FurnStep& st) {
        Placement p;
        switch (st.kind) {
            case FurnStep::Kind::OneOf:
                run(st.alternatives[(hb >> 13) % st.alternatives.size()]);
                return;
            case FurnStep::Kind::Wall: {
                if (st.opposite && !prevPlaced) return;
                if (st.minW > 0 && !(F.f.W > st.minW)) { prevPlaced = false; return; }
                const Piece single = st.set.empty() ? pickOf(st.pick, st.w, st.d) : Piece::Count;
                if (st.set.empty() && single == Piece::Count) { prevPlaced = false; return; }
                const Piece lead = st.set.empty() ? single : pickOf(st.set.front().pick,
                    st.set.front().fitW > 0 ? st.set.front().fitW : st.w, st.set.front().fitD > 0 ? st.set.front().fitD : st.d);
                const Real clear = st.clear > 0 ? Furnisher::clearOf(lead, st.clear) : 0;
                int n = st.countMin;
                if (st.countPer > 0)
                    n = std::clamp(static_cast<int>(std::floor((F.f.W - 1e-6) / st.countPer)), st.countMin, st.countMax);
                const std::vector<int> sides = sidesOf(st);
                bool any = false;
                for (int k = 0; k < n; ++k) {
                    if (!F.place(st.w, st.d, sides, p, st.tall, clear)) break;
                    any = true;
                    prevSide = p.side;
                    if (st.set.empty()) { F.put(p, single, st.w * 0.5, 0.0); continue; }
                    for (const FurnMember& m : st.set) {
                        const Piece pc = pickOf(m.pick, m.fitW > 0 ? m.fitW : st.w, m.fitD > 0 ? m.fitD : st.d);
                        if (pc != Piece::Count) F.put(p, pc, m.x, m.z, m.facing, m.y);
                    }
                }
                prevPlaced = any;
                return;
            }
            case FurnStep::Kind::Grid: {
                if (!st.set.empty() || st.tiers) { gridOfSets(st); return; }
                const Piece pc = pickOf(st.pick, st.w, std::max(st.d, st.d2));
                if (pc == Piece::Count) return;
                const uint32_t v2 = st.style2 >= 0 ? (F.variant & ~(7u << 5)) | (static_cast<uint32_t>(st.style2) << 5) : 0u;
                prevPlaced = F.fill(pc, st.w, st.d, st.aisleX, st.aisleZ, st.d2, v2) > 0;
                return;
            }
            case FurnStep::Kind::Hang: {
                const Piece pc = pickOf(st.pick, 0, 0);
                const uint32_t v = st.style2 >= 0 ? (F.variant & ~(7u << 5)) | (static_cast<uint32_t>(st.style2) << 5) : F.variant;
                if (pc != Piece::Count) prevPlaced = F.hang(sidesOf(st), v, pc);
                return;
            }
            case FurnStep::Kind::Counter: {
                const bool kitchen = st.pattern == "kitchen";
                const Real wall = kitchen ? std::max(F.f.W, F.f.D) : F.f.W;
                const int mods = std::clamp(static_cast<int>((wall - (kitchen ? 1.0 : 0.6)) / 0.6), 2, 6);
                const Piece base = pickOf(st.base, 0.6, st.d), sink = pickOf(st.sink, 0.6, st.d);
                const Piece hob = st.hob.valid() ? pickOf(st.hob, 0.6, st.d) : Piece::Count;
                const Piece tallU = st.tallUnit.valid() ? pickOf(st.tallUnit, 0.6, st.d) : Piece::Count;
                const Piece wallU = st.wallUnit.valid() ? pickOf(st.wallUnit, 0.6, st.d) : Piece::Count;
                const Real clear = st.clear > 0 ? Furnisher::clearOf(base, st.clear) : 0;
                prevPlaced = false;
                if (!F.place(mods * 0.6, st.d, sidesOf(st), p, st.tall, clear)) return;
                prevPlaced = true;
                prevSide = p.side;
                for (int i = 0; i < mods; ++i) {
                    const Real x = 0.3 + i * 0.6;
                    Piece pc = base;
                    if (kitchen) {
                        // the fridge tower at one end, the sink mid-run, the hob at the far end; a wall cupboard over
                        // every base
                        if (i == 0 && mods >= 4 && tallU != Piece::Count) pc = tallU;
                        else if (i == mods / 2) pc = sink;
                        else if ((i == mods - 1 || (mods == 2 && i == 0)) && hob != Piece::Count) pc = hob;
                    } else {
                        pc = i == 0 && tallU != Piece::Count ? tallU : i == 1 ? sink : base;
                    }
                    F.put(p, pc, x, 0.0);
                    if (wallU != Piece::Count && pc != tallU) F.put(p, wallU, x, 0.0);
                }
                return;
            }
        }
    }
};

}  // namespace

void emitFurniture(std::vector<PlacedPiece>& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, uint32_t seed,
                   Real ceilingY) {
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
        F.learnWalls(rp.walls);
        for (const Vec2& dp : doors) {
            const Vec2 l = F.f.local(dp);
            if (l.x < -0.4 || l.x > F.f.W + 0.4 || l.y < -0.4 || l.y > F.f.D + 0.4) continue;
            F.taken.push_back({l.x - 0.6, l.y - 0.6, l.x + 0.6, l.y + 0.6});
            F.doorways.push_back(F.taken.back());
        }
        // An OPEN PLAN zone has rooms set into it (corner offices, meeting rooms, the kitchenette): they are
        // obstacles here, with a clear margin round them.
        if (room.kind == RoomKind::OpenPlan)
            for (const Room& other : rp.rooms) {
                if (&other == &room || other.kind == RoomKind::OpenPlan) continue;
                Real a0 = 1e9, b0 = 1e9, a1 = -1e9, b1 = -1e9;
                for (const Vec2& v : other.rect) {
                    const Vec2 l = F.f.local(v);
                    a0 = std::min(a0, l.x); a1 = std::max(a1, l.x); b0 = std::min(b0, l.y); b1 = std::max(b1, l.y);
                }
                if (a1 < 0 || a0 > F.f.W || b1 < 0 || b0 > F.f.D) continue;
                F.taken.push_back({a0 - 0.4, b0 - 0.4, a1 + 0.4, b1 + 0.4});
            }
        // Sides by preference: a ring room's window wall is side 0, its front (door) side 2; a house room goes
        // longest wall first.
        const bool ring = rp.topology == PlateTopology::Ring;
        const std::vector<int> longFirst = F.f.W >= F.f.D ? std::vector<int>{0, 2, 1, 3} : std::vector<int>{1, 3, 0, 2};
        Placement p;
        const RoomProgram* prog = FurnitureLibrary::global().program(programFor(room.kind));
        if (prog) {
            ProgramRun run{F, FurnitureLibrary::global(), ring, longFirst, hb, hr};
            for (const FurnStep& st : prog->steps) run.run(st);
        } else if (room.kind == RoomKind::Shop) {
            // SHOPS are furnished by their trade, still in code (the program steps cannot yet say "runs of gondolas
            // with cross aisles" or "checkout lanes either side of the doors"). Every other kind is a room program.
            switch (room.kind) {
            case RoomKind::Shop: {
                // A SHOP by its trade (Glenn: "these small shops"); side 0 is the shopfront.
                auto along = [&](Piece pc, Real w, Real d, const std::vector<int>& sides, int count) {
                    int n = 0;
                    for (int c = 0; c < count; ++c)
                        if (F.place(w, d, sides, p, true)) { F.put(p, pc, w * 0.5, 0.0); ++n; }
                    return n;
                };
                // BISTRO SETS (the furniture library, M2): a table and two chairs facing across it, in a grid of
                // 1.5 x 1.7 m cells clear of the counter and the doors -- each chair a seat of its own.
                auto bistro = [&](Real aisle) {
                    const Real cw = 1.5, cd = 1.7;
                    for (Real z = 0.6; z + cd <= F.f.D - 0.8 + 1e-6; z += cd + aisle)
                        for (Real x = 0.6; x + cw <= F.f.W - 0.6 + 1e-6; x += cw + aisle) {
                            Placement c{&F.f, 0, x, cw, cd, z};
                            const Box2 fp = c.footprint();
                            bool clear = true;
                            for (const Box2& b : F.taken)
                                if (overlaps(fp, b, 0.1)) { clear = false; break; }
                            if (!clear) continue;
                            F.taken.push_back(fp);
                            F.put(c, Piece::BistroChair, cw * 0.5, 0.0);           // facing the table (+z)
                            F.put(c, Piece::BistroTable, cw * 0.5, 0.5);
                            F.put(c, Piece::BistroChair, cw * 0.5, cd, false);     // the far side, facing back
                        }
                };
                auto counterAtBack = [&]() {
                    if (F.place(1.8, 1.6, {2, 1, 3}, p)) F.put(p, Piece::ShopCounter, 0.9, 0.0, true);
                };
                // BIG-BOX floors (Glenn, 2026-10-01): RUNS of a piece along x, `runN` bays to a run, a cross aisle
                // between runs, `aisleZ` between rows -- long aisles you can walk down, not one solid field.
                auto runs = [&](Piece pc, Real w, Real d, Real aisleZ, int runN, Real cross, Real xa, Real xb) {
                    int n = 0;
                    for (Real z = 1.2; z + d <= F.f.D - 1.2 + 1e-6; z += d + aisleZ)
                        for (Real x = xa; x + w <= xb + 1e-6;) {
                            for (int k = 0; k < runN && x + w <= xb + 1e-6; ++k, x += w) {
                                Placement c{&F.f, 0, x, w, d, z};
                                const Box2 fp = c.footprint();
                                bool clear = true;
                                for (const Box2& b : F.taken)
                                    if (overlaps(fp, b, 0.05)) { clear = false; break; }
                                if (!clear) continue;
                                F.taken.push_back(fp);
                                F.put(c, pc, w * 0.5, 0.0);
                                ++n;
                            }
                            x += cross;
                        }
                    return n;
                };
                // THE BAR (storefronts stage 5): a run of `n` 1.8 m bays (fewer where the wall is short) -- the back
                // bar on the wall, the bartender's 0.95 m aisle, the counter, a stool every 0.6 m in front of it.
                auto barRun = [&](const std::vector<int>& sides, int n) {
                    for (int k = n; k >= 1; --k) {
                        const Real w = 1.8 * k, d = 0.5 + 0.95 + 0.75 + 0.5;
                        if (!F.place(w, d, sides, p, true, 0.6)) continue;
                        for (int i = 0; i < k; ++i) {
                            F.put(p, Piece::BackBar, 0.9 + 1.8 * i, 0.0);
                            F.put(p, Piece::BarCounter, 0.9 + 1.8 * i, 1.45);
                        }
                        for (Real x = 0.45; x + 0.3 <= w; x += 0.6) F.put(p, Piece::BarStool, x, 2.26);
                        return true;
                    }
                    return false;
                };
                auto booth = [&](const std::vector<int>& sides) {
                    if (F.place(1.9, 1.3, sides, p, true, 0.5)) F.put(p, Piece::Booth, 0.95, 0.0);
                };
                // POSEUR TABLES: a high table and two stools to a 1.6 x 0.8 cell, the cells 1.2 m apart.
                auto highTables = [&]() {
                    for (Real z = 0.9; z + 0.8 <= F.f.D - 0.8 + 1e-6; z += 0.8 + 1.2)
                        for (Real x = 0.6; x + 1.6 <= F.f.W - 0.6 + 1e-6; x += 1.6 + 1.2) {
                            Placement c{&F.f, 0, x, 1.6, 0.8, z};
                            const Box2 fp = c.footprint();
                            bool clear = true;
                            for (const Box2& b : F.taken)
                                if (overlaps(fp, b, 0.3)) { clear = false; break; }
                            if (!clear) continue;
                            F.taken.push_back(fp);
                            F.put(c, Piece::HighTable, 0.8, 0.05);
                            F.put(c, Piece::BarStool, 0.21, 0.19);
                            F.put(c, Piece::BarStool, 1.39, 0.19);
                        }
                };
                // THE KITCHEN (a restaurant's back): the line on the back wall -- tall unit, hobs, a sink, worktops --
                // a 1.1 m cooks' aisle, the pass facing the dining room.
                auto kitchenAndPass = [&]() {
                    const Real w = std::min(F.f.W - 0.2, Real(6.0));
                    if (w < 2.6 || !F.place(w, 0.6 + 1.1 + 0.8, {2}, p, true, 0.4)) { counterAtBack(); return; }
                    // tall units at the two ends only (a run of hobs, worktops and sinks between)
                    static const Piece kLine[5] = {Piece::KitchenHob, Piece::KitchenHob, Piece::KitchenBase,
                                                   Piece::KitchenSink, Piece::KitchenBase};
                    const int n = static_cast<int>(w / 0.6);
                    for (int i = 0; i < n; ++i)
                        F.put(p, i == 0 || i == n - 1 ? Piece::KitchenTall : kLine[(i - 1) % 5], 0.3 + 0.6 * i, 0.0);
                    F.put(p, Piece::KitchenPass, w * 0.5, 1.7);
                };
                const Real xa = 1.0, xb = F.f.W - 1.0, xm = F.f.W * 0.5;
                if (room.style >= 7 && room.style <= 12) {   // (the big-box store's rooms; 13+ are trades: trades.h)
                    switch (room.style) {
                        case 7:   // WAREHOUSE CLUB: pallet racking, wide aisles for the forklifts
                        case 9:   // HOME IMPROVEMENT: the same racking, wider aisles
                            along(Piece::DrinksFridge, 0.8, 0.75, {1}, 8);
                            runs(Piece::PalletRack, 2.8, 2.2, room.style == 7 ? 3.6 : 4.2, 8, 4.0, xa + 1.0, xb - 1.0);
                            break;
                        case 8:   // ELECTRONICS: the TV wall at the back, shelving down the sides, gondola runs
                            along(Piece::TvUnit, 1.6, 0.42, {2}, 40);
                            along(Piece::WallShelf, 2.0, 0.45, {1, 3}, 30);
                            runs(Piece::Gondola, 2.4, 1.0, 2.4, 6, 3.0, xa + 2.0, xb - 2.0);
                            break;
                        case 10:  // DISCOUNT STORE: clothes on one side of the main aisle, gondolas on the other
                            along(Piece::WallShelf, 2.0, 0.45, {2, 1, 3}, 40);
                            runs(Piece::ClothesRack, 1.6, 0.6, 1.4, 5, 2.4, xa + 1.0, xm - 2.5);
                            runs(Piece::Gondola, 2.4, 1.0, 2.2, 6, 3.0, xm + 2.5, xb - 1.0);
                            break;
                        case 11: {   // THE CHECKOUTS: lanes in a row 3 m in from the doors, the doorway clear
                            for (int sideK = 0; sideK < 2; ++sideK)
                                for (int k = 0; k < 16; ++k) {
                                    const Real x = sideK == 0 ? xm - 6.0 - (k + 1) * 3.0 : xm + 6.0 + k * 3.0;
                                    if (x < xa || x + 0.9 > xb) break;
                                    Placement c{&F.f, 0, x, 0.9, 3.2, 3.0};
                                    F.taken.push_back(c.footprint());
                                    F.put(c, Piece::Checkout, 0.45, 0.0);
                                }
                            break;
                        }
                        default:  // THE STOCKROOM: racking, single-sided rows against nothing in particular
                            runs(Piece::PalletRack, 2.8, 2.2, 3.0, 10, 3.5, xa, xb);
                            break;
                    }
                    break;
                }
                switch (room.style) {   // the unit's TRADE (trades.h)
                    case 0:   // CAFE
                        counterAtBack();
                        along(Piece::DrinksFridge, 0.8, 0.75, {2, 1, 3}, 1);
                        bistro(0.5);
                        break;
                    case 1:   // GROCERY
                        if (F.place(1.8, 1.4, {1, 3}, p)) F.put(p, Piece::ShopCounter, 0.9, 0.0);
                        along(Piece::DrinksFridge, 0.8, 0.75, {2}, 4);
                        F.fill(Piece::Gondola, 2.4, 1.0, 1.0, 1.4);
                        break;
                    case 2:   // BOUTIQUE
                        along(Piece::WallShelf, 2.0, 0.45, {1, 3}, 4);
                        counterAtBack();
                        F.fill(Piece::ClothesRack, 1.6, 0.6, 1.0, 1.2);
                        break;
                    case 3:   // BOOKSHOP
                        along(Piece::Bookcase, 1.2, 0.35, {1, 3, 2}, 10);
                        counterAtBack();
                        if (F.place(1.4, 1.85, {0, 1, 3}, p)) F.put(p, Piece::DiningTable, 0.7, 0.5);
                        break;
                    case 4:   // ELECTRONICS
                        along(Piece::TvUnit, 1.6, 0.42, {2}, 3);
                        along(Piece::WallShelf, 2.0, 0.45, {1, 3}, 2);
                        counterAtBack();
                        F.fill(Piece::DiningTable, 1.4, 0.85, 1.2, 1.4);
                        break;
                    case 5:   // PHARMACY
                        along(Piece::WallShelf, 2.0, 0.45, {1, 3}, 4);
                        counterAtBack();
                        F.fill(Piece::Gondola, 2.4, 1.0, 1.2, 1.6);
                        break;
                    case 6:   // BAKERY
                        along(Piece::DisplayCase, 1.6, 0.7, {2}, 2);
                        counterAtBack();
                        bistro(0.6);
                        break;
                    case 13: {  // RESTAURANT: the kitchen line and the pass across the back, a host's stand by the
                                // door, booths down the side walls, the dining room's tables in the rest
                        kitchenAndPass();
                        if (F.place(0.7, 0.55, {0}, p)) F.put(p, Piece::Lectern, 0.35, 0.0);
                        for (int k = 0; k < 2; ++k) booth({1, 3});
                        bistro(0.8);
                        break;
                    }
                    case 14:    // BAR: the bar down a long wall (back bar, the bartender's aisle, the counter, stools),
                                // booths along the other, poseur tables between
                        if (!barRun({1, 3, 2}, 4)) counterAtBack();
                        for (int k = 0; k < 3; ++k) booth({3, 1, 2});
                        highTables();
                        break;
                    case 15: {  // CLUB: the DJ at the back, the lit floor in front of the booth, the bar down a side,
                                // booths and sofas round the edge
                        if (F.place(3.0, 1.0, {2}, p, true, 0.6)) F.put(p, Piece::DjBooth, 1.5, 0.0);
                        if (F.f.W >= 5.2 && F.f.D >= 6.5) {
                            const Real fw = 4.0, z = std::max(Real(1.2), F.f.D - 1.0 - 0.6 - fw - 0.2);
                            Placement c{&F.f, 0, (F.f.W - fw) * 0.5, fw, fw, z};
                            const Box2 fp = c.footprint();
                            bool clear = true;
                            for (const Box2& b : F.taken) if (overlaps(fp, b, 0.0)) clear = false;
                            if (clear) { F.taken.push_back(fp); F.put(c, Piece::DanceFloor, fw * 0.5, 0.0); }
                        }
                        if (!barRun({1, 3}, 3)) counterAtBack();
                        for (int k = 0; k < 2; ++k) booth({3, 1});
                        along(Piece::Sofa, 2.0, 0.9, {3, 1}, 1);
                        highTables();
                        break;
                    }
                    default:  // a trade with no fit-out yet: a counter
                        counterAtBack();
                        break;
                }
                break;
            }
                default: break;
            }
        }
        // (no program and not a shop: the library is not loaded, and the room stays empty -- the furniture is data)
        // THE LIGHTS (Glenn, 2026-10-02: "actual ceiling light fixtures for where the ambient light comes from"):
        // a working room -- office, open plan, meeting room, kitchenette, shop -- a grid of panels every 2.4 m
        // along its long side; a home's room, a round fitting in its middle (a long hall, one every 4 m).
        if (ceilingY > y0 + 2.0) {
            // kind: 0 an office panel, 1 a round flush fitting, 2 a bar's pendant (on a cord, 0.9 m down)
            auto hangAt = [&](Real a, Real b, int kind) {
                const bool alongV = F.f.D > F.f.W;
                const Vec2 Zd = alongV ? F.f.v : F.f.u;
                const Vec2 Xd(Zd.y, -Zd.x);
                const Real half = kind == 0 ? 0.61 : kind == 1 ? 0.21 : 0.18, h = kind == 0 ? 0.05 : kind == 1 ? 0.08 : 0.90;
                const Vec2 c = F.f.world(a, b) - Zd * half;
                PlacedPiece pp;
                pp.piece = static_cast<uint8_t>(Piece::CeilingLight);
                pp.variant = kind == 0 ? 0u : kind == 1 ? 32u : 32u | 64u | ((hr & 1u) ? 128u : 0u);   // brass or black
                Mat4& M = pp.xform;
                // right-handed: X = Z x up
                Vec3 X(Xd.x, 0, Xd.y), Z(Zd.x, 0, Zd.y);
                if (dot(cross(X, Vec3(0, 1, 0)), Z) < 0) X = X * -1.0;
                M.m[0][0] = X.x; M.m[1][0] = 0; M.m[2][0] = X.z;
                M.m[0][1] = 0;   M.m[1][1] = 1; M.m[2][1] = 0;
                M.m[0][2] = Z.x; M.m[1][2] = 0; M.m[2][2] = Z.z;
                M.m[0][3] = c.x; M.m[1][3] = ceilingY - h; M.m[2][3] = c.y;
                out.push_back(pp);
            };
            const bool working = room.kind == RoomKind::Office || room.kind == RoomKind::OpenPlan ||
                                 room.kind == RoomKind::Meeting || room.kind == RoomKind::Kitchenette ||
                                 room.kind == RoomKind::Shop;
            // a shop is lit as its trade (trades.h): office panels, round fittings in a warm room, a few in a bar
            const int shopLights = room.kind == RoomKind::Shop ? tradeInterior(room.style).lights : 0;
            if (working) {
                const Real pitch = shopLights == 2 ? 3.6 : 2.4;
                const int nx = std::max(1, static_cast<int>(F.f.W / pitch)), nz = std::max(1, static_cast<int>(F.f.D / pitch));
                for (int i = 0; i < nx; ++i)
                    for (int j = 0; j < nz; ++j)
                        hangAt(F.f.W * (i + 0.5) / nx, F.f.D * (j + 0.5) / nz, shopLights);
            } else {
                const Real L = std::max(F.f.W, F.f.D);
                const int n = L > 6.0 ? static_cast<int>(L / 4.0) : 1;
                for (int i = 0; i < n; ++i)
                    hangAt(F.f.W >= F.f.D ? F.f.W * (i + 0.5) / n : F.f.W * 0.5,
                           F.f.W >= F.f.D ? F.f.D * 0.5 : F.f.D * (i + 0.5) / n, 1);
            }
        }
        // The pictures: on the side walls (never the window wall of a room that has one), two in a long living
        // room. The art varies picture to picture.
        const uint32_t art = (hr >> 3) % 24u;   // 24 designs: each is one instanced mesh, so the variety is a draw-call budget
        const bool windowed = ring || rp.topology == PlateTopology::Apartments;
        const std::vector<int> artSides = windowed ? std::vector<int>{1, 3, 2} : std::vector<int>{1, 3, 0, 2};
        switch (room.kind) {
            case RoomKind::Living: {
                const bool two = std::max(F.f.W, F.f.D) > 6.0;
                if (F.hang(artSides, art << 5) && two) F.hang({3, 2, 1}, ((art + 7) % 24u) << 5);
                break;
            }
            case RoomKind::Bed: case RoomKind::Flat: case RoomKind::Office: case RoomKind::Hall:
                F.hang(artSides, art << 5);
                break;
            default: break;
        }
    }
}

}  // namespace engine
