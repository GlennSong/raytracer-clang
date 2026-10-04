#include "furniture_kit.h"
#include "../mesh_builder.h"
#include "lsystem.h"   // houseplants are grown, not modelled
#include <cmath>
#include <map>
#include <memory>
#include <mutex>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;

// The swatch books. Wood: oak, walnut, birch, ebonised. Fabric: grey, navy, sage, rust, oat, charcoal, mustard, teal.
const Vec3 kWood[4] = {{0.66, 0.50, 0.33}, {0.40, 0.27, 0.18}, {0.82, 0.72, 0.56}, {0.24, 0.18, 0.15}};
const Vec3 kFabric[8] = {{0.48, 0.50, 0.53}, {0.20, 0.26, 0.40}, {0.45, 0.53, 0.42}, {0.62, 0.34, 0.24},
                         {0.74, 0.68, 0.58}, {0.22, 0.22, 0.24}, {0.72, 0.56, 0.22}, {0.20, 0.42, 0.44}};
const Vec3 kWhite(0.90, 0.90, 0.88), kBlack(0.07, 0.07, 0.08), kSteel(0.62, 0.64, 0.67), kChrome(0.85, 0.86, 0.88);
const Vec3 kPorcelain(0.95, 0.95, 0.93), kLinen(0.93, 0.92, 0.88);

// A piece under construction: the per-material meshes and the shape-kit words, placed in piece space.
struct Maker {
    FurniturePiece out;
    RenderMesh& m(FurnMat k) { return out.mesh[static_cast<int>(k)]; }
    static void paint(RenderMesh& mm, const Vec3& col) { for (Vertex& v : mm.vertices) v.color = col; }
    // A rounded box centred at c.
    void box(FurnMat k, const Vec3& c, const Vec3& size, double r, const Vec3& col, int segs = 2) {
        // A bevel under 3 mm is invisible and costs ~100 triangles a box: those are sharp (12).
        if (r < 0.003) r = 0;
        RenderMesh b = MeshBuilder::roundedBox(size, r, r > 0.012 ? segs : 1, 2.0);
        paint(b, col);
        MeshBuilder::transform(b, Mat4::translate(c.x, c.y, c.z));
        MeshBuilder::append(m(k), b);
    }
    // The same, turned `ang` radians about +x through its own centre (a reclined back cushion).
    void boxTilted(FurnMat k, const Vec3& c, const Vec3& size, double r, double ang, const Vec3& col) {
        RenderMesh b = MeshBuilder::roundedBox(size, r, 2, 2.0);
        paint(b, col);
        const double ca = std::cos(ang), sa = std::sin(ang);
        for (Vertex& v : b.vertices) {
            const Vec3 p = v.position, n = v.normal;
            v.position = Vec3(p.x, p.y * ca - p.z * sa, p.y * sa + p.z * ca) + c;
            v.normal = Vec3(n.x, n.y * ca - n.z * sa, n.y * sa + n.z * ca);
        }
        MeshBuilder::append(m(k), b);
    }
    // A straight round rod from a to b.
    void rod(FurnMat k, const Vec3& a, const Vec3& b, double r, const Vec3& col, int sides = 10) {
        RenderMesh t = MeshBuilder::tube({a, b}, {r, r}, sides);
        paint(t, col);
        MeshBuilder::append(m(k), t);
        // Caps: a lathe disc at each end would be invisible at these radii; leave the ends open.
    }
    // A turned shape (profile (r, y)) standing at c, squashed in z by `sz` (an oval bowl).
    void turned(FurnMat k, const Vec3& c, const std::vector<std::pair<double, double>>& prof, const Vec3& col,
                double sz = 1.0, int segs = 20) {
        RenderMesh t = MeshBuilder::lathe(prof, segs);
        paint(t, col);
        for (Vertex& v : t.vertices) {
            v.position = Vec3(v.position.x, v.position.y, v.position.z * sz) + c;
            v.normal = normalize(Vec3(v.normal.x * sz, v.normal.y * sz, v.normal.z));
        }
        MeshBuilder::append(m(k), t);
    }
    // A square-section leg, tapering from w0 at the floor... to w1 at the top.
    void leg(FurnMat k, const Vec3& foot, double h, double w, const Vec3& col) {
        box(k, foot + Vec3(0, h * 0.5, 0), Vec3(w, h, w), 0.006, col, 1);
    }
    // A drawer or door FRONT on a face at z = zFace: a slightly rounded panel proud 1 cm, and a bar handle.
    void front(FurnMat k, double x0, double x1, double y0, double y1, double zFace, const Vec3& col, const Vec3& handle,
               bool vertical = false) {
        const double gap = 0.004;
        box(k, Vec3((x0 + x1) * 0.5, (y0 + y1) * 0.5, zFace + 0.009), Vec3(x1 - x0 - gap, y1 - y0 - gap, 0.018), 0.004,
            col, 1);
        const double hz = zFace + 0.03;
        if (vertical) {
            const double hx = x0 + (x1 - x0) * 0.85, hy = (y0 + y1) * 0.5;
            rod(FurnMat::Metal, Vec3(hx, hy - 0.12, hz), Vec3(hx, hy + 0.12, hz), 0.007, handle, 8);
        } else {
            const double hy = y1 - std::min(0.06, (y1 - y0) * 0.3), hx = (x0 + x1) * 0.5;
            rod(FurnMat::Metal, Vec3(hx - 0.09, hy, hz), Vec3(hx + 0.09, hy, hz), 0.007, handle, 8);
        }
    }
};

FurniturePiece build(Piece p, uint32_t variant) {
    Maker k;
    const Vec3 wood = kWood[(variant >> 3) & 3u], fabric = kFabric[variant & 7u];
    const uint32_t style = variant >> 5;
    using F = FurnMat;
    switch (p) {
        case Piece::Desk: {
            // 1.6 x 0.8 top on a pair of steel sled frames, a three-drawer pedestal under the right end.
            k.out.size = {1.6, 0.75, 0.8};
            k.box(F::Wood, {0, 0.73, 0.4}, {1.6, 0.03, 0.8}, 0.006, wood, 1);
            for (double x : {-0.74, 0.74}) {
                k.rod(F::Metal, {x, 0.02, 0.06}, {x, 0.02, 0.74}, 0.016, kBlack);   // foot
                k.rod(F::Metal, {x, 0.70, 0.06}, {x, 0.70, 0.74}, 0.016, kBlack);   // top rail
                k.rod(F::Metal, {x, 0.02, 0.10}, {x, 0.70, 0.10}, 0.016, kBlack);   // uprights
                k.rod(F::Metal, {x, 0.02, 0.70}, {x, 0.70, 0.70}, 0.016, kBlack);
            }
            k.rod(F::Metal, {-0.74, 0.62, 0.08}, {0.74, 0.62, 0.08}, 0.012, kBlack);  // back stretcher
            const double px0 = 0.22, px1 = 0.68;
            k.box(F::Hard, {(px0 + px1) * 0.5, 0.36, 0.38}, {px1 - px0, 0.56, 0.62}, 0.006, kWhite, 1);
            for (int d = 0; d < 3; ++d) {
                const double y0 = 0.10 + d * 0.18;
                k.front(F::Hard, px0, px1, y0, y0 + 0.18, 0.69, kWhite, kChrome);
            }
            break;
        }
        case Piece::OfficeChair: {
            // Five-star base on casters, a gas lift, a moulded seat and a reclined mesh back, armrests.
            k.out.size = {0.66, 1.05, 0.66};
            k.out.solid = false;
            const Vec3 c(0, 0, 0.33);
            for (int i = 0; i < 5; ++i) {
                const double a = 2 * kPi * i / 5 + 0.3;
                const Vec3 tip = c + Vec3(0.30 * std::cos(a), 0.07, 0.30 * std::sin(a));
                k.rod(F::Metal, c + Vec3(0, 0.10, 0), tip, 0.018, kBlack, 8);
                k.turned(F::Hard, tip - Vec3(0, 0.07, 0), {{0.0, 0.0}, {0.028, 0.008}, {0.032, 0.03}, {0.028, 0.055},
                                                           {0.0, 0.065}}, kBlack, 1.0, 10);   // caster
            }
            k.turned(F::Metal, c, {{0.04, 0.08}, {0.04, 0.14}, {0.024, 0.15}, {0.024, 0.42}, {0.03, 0.44}}, kChrome, 1.0, 16);
            k.box(F::Fabric, c + Vec3(0, 0.48, 0.02), {0.50, 0.08, 0.48}, 0.035, fabric * 0.75);
            k.boxTilted(F::Fabric, c + Vec3(0, 0.80, -0.22), {0.46, 0.52, 0.06}, 0.028, -0.18, fabric * 0.75);
            k.rod(F::Metal, c + Vec3(0, 0.46, -0.20), c + Vec3(0, 0.62, -0.25), 0.015, kBlack, 8);
            for (double x : {-0.27, 0.27}) {
                k.rod(F::Metal, c + Vec3(x, 0.46, 0.0), c + Vec3(x, 0.64, 0.0), 0.012, kBlack, 8);
                k.box(F::Hard, c + Vec3(x, 0.655, 0.02), {0.07, 0.03, 0.24}, 0.012, kBlack);
            }
            break;
        }
        case Piece::Monitor: {
            k.out.size = {0.62, 0.48, 0.22};
            k.out.solid = false;
            // The screen faces +z (the chair); the stand stands BEHIND it, toward the wall (Glenn: "the computer
            // monitors are backwards" -- the neck was on the screen side).
            k.box(F::Hard, {0, 0.008, 0.08}, {0.24, 0.016, 0.16}, 0.008, kBlack, 1);
            k.box(F::Hard, {0, 0.17, 0.06}, {0.05, 0.30, 0.025}, 0.008, kBlack, 1);
            k.box(F::Hard, {0, 0.30, 0.085}, {0.60, 0.36, 0.025}, 0.006, kBlack, 1);
            k.box(F::Metal, {0, 0.30, 0.0985}, {0.58, 0.34, 0.004}, 0.0, Vec3(0.02, 0.025, 0.03), 1);   // the glass
            k.box(F::Hard, {0, 0.012, 0.19}, {0.44, 0.02, 0.13}, 0.006, kBlack, 1);   // keyboard
            break;
        }
        case Piece::FilingCabinet: {
            k.out.size = {0.46, 1.05, 0.6};
            k.box(F::Hard, {0, 0.525, 0.30}, {0.46, 1.05, 0.58}, 0.006, kSteel * 0.85, 1);
            for (int d = 0; d < 3; ++d) {
                const double y0 = 0.04 + d * 0.33;
                k.front(F::Hard, -0.22, 0.22, y0, y0 + 0.33, 0.59, kSteel * 0.9, kChrome);
            }
            break;
        }
        case Piece::Bed: {
            // A made double: a timber frame on legs, an upholstered headboard, the mattress, a turned-back
            // duvet, two pillows.
            k.out.size = {1.66, 1.05, 2.12};
            k.box(F::Wood, {0, 0.22, 1.06}, {1.66, 0.14, 2.10}, 0.012, wood, 1);
            for (double x : {-0.76, 0.76})
                for (double z : {0.08, 2.02}) k.leg(F::Wood, {x, 0, z}, 0.16, 0.06, wood * 0.9);
            k.box(F::Fabric, {0, 0.60, 0.05}, {1.66, 0.96, 0.10}, 0.04, fabric * 0.8);
            // MADE (Glenn: "the beds don't have sheets on them"): the fitted sheet over the mattress, the duvet
            // lying ON it and hanging down both sides and the foot, the top sheet turned back over the duvet's head,
            // two pillows against the headboard.
            k.box(F::Fabric, {0, 0.40, 1.08}, {1.56, 0.22, 2.0}, 0.06, kLinen);                    // sheeted mattress
            k.box(F::Fabric, {0, 0.545, 1.37}, {1.70, 0.07, 1.48}, 0.03, fabric);                 // duvet, on top
            for (double x : {-0.85, 0.85})
                k.box(F::Fabric, {x, 0.43, 1.37}, {0.035, 0.30, 1.48}, 0.015, fabric * 0.94);     // hanging sides
            k.box(F::Fabric, {0, 0.43, 2.105}, {1.70, 0.30, 0.035}, 0.015, fabric * 0.94);       // and foot
            k.box(F::Fabric, {0, 0.59, 0.72}, {1.70, 0.05, 0.26}, 0.024, kLinen * 1.03);          // turned-down sheet
            for (double x : {-0.38, 0.38}) k.box(F::Fabric, {x, 0.59, 0.33}, {0.66, 0.14, 0.40}, 0.06, kLinen * 1.02);
            break;
        }
        case Piece::Nightstand: {
            k.out.size = {0.46, 0.95, 0.40};
            k.box(F::Wood, {0, 0.30, 0.20}, {0.46, 0.44, 0.40}, 0.008, wood, 1);
            for (double x : {-0.19, 0.19})
                for (double z : {0.04, 0.36}) k.leg(F::Wood, {x, 0, z}, 0.08, 0.035, wood * 0.9);
            k.front(F::Wood, -0.21, 0.21, 0.30, 0.50, 0.40, wood * 0.95, kChrome);
            // The lamp: a turned base and a drum shade.
            k.turned(F::Ceramic, {0.06, 0.52, 0.18}, {{0.0, 0.0}, {0.07, 0.0}, {0.075, 0.02}, {0.05, 0.12}, {0.06, 0.20},
                                                     {0.015, 0.24}, {0.0, 0.25}}, kPorcelain * 0.92, 1.0, 18);
            k.turned(F::Fabric, {0.06, 0.74, 0.18}, {{0.13, 0.0}, {0.13, 0.002}, {0.10, 0.17}, {0.098, 0.172}},
                     kLinen, 1.0, 20);
            break;
        }
        case Piece::Wardrobe: {
            k.out.size = {1.2, 2.05, 0.6};
            k.box(F::Wood, {0, 1.04, 0.30}, {1.2, 1.98, 0.58}, 0.008, wood, 1);
            k.box(F::Wood, {0, 0.03, 0.29}, {1.16, 0.06, 0.54}, 0.004, wood * 0.7, 1);   // plinth
            k.front(F::Wood, -0.59, 0.0, 0.08, 2.02, 0.59, wood * 1.04, kChrome, true);
            k.front(F::Wood, 0.0, 0.59, 0.08, 2.02, 0.59, wood * 1.04, kChrome, true);
            break;
        }
        case Piece::Sofa: {
            // Three seat cushions, three reclined back cushions, two rolled arms, a base on short legs.
            k.out.size = {2.16, 0.86, 0.94};
            k.box(F::Fabric, {0, 0.24, 0.48}, {2.0, 0.22, 0.86}, 0.03, fabric * 0.9);
            for (double x : {-1.0, 1.0}) k.box(F::Fabric, {x, 0.36, 0.47}, {0.18, 0.46, 0.92}, 0.07, fabric * 0.95);
            for (int i = 0; i < 3; ++i) {
                const double x = -0.62 + i * 0.62;
                k.box(F::Fabric, {x, 0.42, 0.55}, {0.60, 0.15, 0.66}, 0.06, fabric);
                k.boxTilted(F::Fabric, {x, 0.68, 0.18}, {0.60, 0.42, 0.18}, 0.07, -0.16, fabric * 1.03);
            }
            k.box(F::Fabric, {0, 0.56, 0.08}, {2.0, 0.42, 0.14}, 0.03, fabric * 0.9);   // the back frame
            for (double x : {-0.98, 0.98})
                for (double z : {0.08, 0.86}) k.leg(F::Wood, {x, 0, z}, 0.13, 0.05, wood * 0.8);
            break;
        }
        case Piece::CoffeeTable: {
            k.out.size = {1.1, 0.42, 0.6};
            k.box(F::Wood, {0, 0.40, 0.30}, {1.1, 0.035, 0.6}, 0.012, wood, 1);
            k.box(F::Wood, {0, 0.14, 0.30}, {1.0, 0.02, 0.5}, 0.004, wood * 0.9, 1);   // the lower shelf
            for (double x : {-0.50, 0.50})
                for (double z : {0.05, 0.55}) k.leg(F::Wood, {x, 0, z}, 0.385, 0.04, wood * 0.85);
            break;
        }
        case Piece::TvUnit: {
            k.out.size = {1.6, 1.25, 0.42};
            k.box(F::Wood, {0, 0.30, 0.21}, {1.6, 0.42, 0.42}, 0.008, wood, 1);
            for (double x : {-0.74, 0.74}) k.leg(F::Metal, {x, 0, 0.21}, 0.10, 0.03, kBlack);
            k.front(F::Wood, -0.78, -0.26, 0.11, 0.49, 0.42, wood * 1.04, kBlack);
            k.front(F::Wood, 0.26, 0.78, 0.11, 0.49, 0.42, wood * 1.04, kBlack);
            k.box(F::Hard, {0, 0.53, 0.20}, {0.30, 0.02, 0.20}, 0.004, kBlack, 1);     // TV foot
            k.box(F::Hard, {0, 0.58, 0.20}, {0.06, 0.10, 0.03}, 0.004, kBlack, 1);
            k.box(F::Hard, {0, 0.93, 0.20}, {1.25, 0.72, 0.04}, 0.004, kBlack, 1);     // the set
            k.box(F::Metal, {0, 0.93, 0.2205}, {1.22, 0.69, 0.002}, 0.0, Vec3(0.02, 0.02, 0.025), 1);
            break;
        }
        case Piece::KitchenBase:
        case Piece::KitchenSink:
        case Piece::KitchenHob: {
            // One 0.6 m module of a counter run: the carcass on its kick plinth, a door (or drawers), a slice of
            // worktop -- and in it the sink (a stainless bowl and a tap) or the hob (black glass, four rings).
            k.out.size = {0.6, p == Piece::KitchenSink ? 1.18 : 0.92, 0.62};   // the sink's tap stands tall
            k.out.colliderH = 0.92;                                              // but the run collides as a counter
            const Vec3 door = (style & 1u) ? kWhite : fabric * 0.75 + Vec3(0.12, 0.12, 0.12);
            k.box(F::Hard, {0, 0.05, 0.27}, {0.6, 0.10, 0.52}, 0.0, kBlack * 2.0, 1);
            k.box(F::Hard, {0, 0.48, 0.29}, {0.6, 0.76, 0.58}, 0.0, kWhite * 0.95, 1);
            if (p == Piece::KitchenHob) {
                for (int d = 0; d < 3; ++d) k.front(F::Hard, -0.3, 0.3, 0.10 + d * 0.253, 0.10 + (d + 1) * 0.253, 0.58, door, kChrome);
            } else {
                k.front(F::Hard, -0.3, 0.3, 0.10, 0.86, 0.58, door, kChrome, true);
            }
            const Vec3 top = (style & 2u) ? Vec3(0.20, 0.20, 0.21) : wood * 1.05;
            k.box((style & 2u) ? F::Ceramic : F::Wood, {0, 0.88, 0.31}, {0.6, 0.04, 0.62}, 0.003, top, 1);
            if (p == Piece::KitchenSink) {
                k.box(F::Metal, {0, 0.896, 0.33}, {0.46, 0.004, 0.40}, 0.0, kChrome, 1);
                k.box(F::Metal, {0, 0.81, 0.33}, {0.40, 0.16, 0.34}, 0.02, kSteel * 0.6, 2);   // the bowl's floor
                k.turned(F::Metal, {0, 0.90, 0.10}, {{0.025, 0.0}, {0.022, 0.03}, {0.012, 0.04}, {0.012, 0.26}, {0.0, 0.27}},
                         kChrome, 1.0, 12);
                k.rod(F::Metal, {0, 1.15, 0.10}, {0, 1.12, 0.26}, 0.012, kChrome, 10);
            } else if (p == Piece::KitchenHob) {
                k.box(F::Metal, {0, 0.902, 0.32}, {0.56, 0.006, 0.50}, 0.004, Vec3(0.03, 0.03, 0.035), 1);
                for (double x : {-0.13, 0.13})
                    for (double z : {0.20, 0.44})
                        k.turned(F::Hard, {x, 0.905, z}, {{0.0, 0.0}, {0.085, 0.0}, {0.085, 0.002}, {0.0, 0.002}},
                                 Vec3(0.18, 0.18, 0.19), 1.0, 20);
            }
            break;
        }
        case Piece::KitchenTall: {
            // The fridge tower: full height, a long handle.
            k.out.size = {0.6, 2.0, 0.64};
            k.box(F::Hard, {0, 1.0, 0.31}, {0.6, 2.0, 0.62}, 0.004, kWhite * 0.95, 1);
            k.front(F::Hard, -0.3, 0.3, 0.80, 1.98, 0.62, kSteel, kChrome, true);
            k.front(F::Hard, -0.3, 0.3, 0.10, 0.80, 0.62, kSteel, kChrome, true);
            break;
        }
        case Piece::KitchenWall: {
            // A wall cupboard, hung at 1.45 m.
            k.out.size = {0.6, 2.15, 0.35};
            k.out.solid = false;
            k.box(F::Hard, {0, 1.80, 0.17}, {0.6, 0.70, 0.34}, 0.0, kWhite * 0.95, 1);
            const Vec3 door = (style & 1u) ? kWhite : fabric * 0.75 + Vec3(0.12, 0.12, 0.12);
            k.front(F::Hard, -0.3, 0.3, 1.45, 2.15, 0.34, door, kChrome, true);
            break;
        }
        case Piece::DiningTable: {
            k.out.size = {1.4, 0.76, 0.85};
            k.box(F::Wood, {0, 0.74, 0.425}, {1.4, 0.04, 0.85}, 0.012, wood, 1);
            k.box(F::Wood, {0, 0.68, 0.425}, {1.26, 0.08, 0.72}, 0.0, wood * 0.9, 1);   // the apron
            for (double x : {-0.64, 0.64})
                for (double z : {0.07, 0.78}) k.leg(F::Wood, {x, 0, z}, 0.72, 0.055, wood * 0.95);
            break;
        }
        case Piece::DiningChair: {
            k.out.size = {0.46, 0.88, 0.50};
            k.out.solid = false;
            k.box(F::Fabric, {0, 0.46, 0.27}, {0.44, 0.05, 0.42}, 0.02, fabric);
            for (double x : {-0.19, 0.19}) {
                k.leg(F::Wood, {x, 0, 0.45}, 0.44, 0.035, wood);
                k.leg(F::Wood, {x, 0, 0.06}, 0.88, 0.035, wood);
            }
            for (double y : {0.62, 0.80}) k.box(F::Wood, {0, y, 0.06}, {0.36, 0.06, 0.02}, 0.006, wood, 1);
            break;
        }
        case Piece::Bathtub: {
            // A built-in tub: four rounded walls round a sunken floor, a chrome mixer at the wall end.
            k.out.size = {1.70, 0.70, 0.75};   // to the mixer's top
            const double W = 1.70, D = 0.75, H = 0.56, t = 0.07;
            k.box(F::Ceramic, {0, H * 0.5, t * 0.5}, {W, H, t}, 0.02, kPorcelain);
            k.box(F::Ceramic, {0, H * 0.5, D - t * 0.5}, {W, H, t}, 0.02, kPorcelain);
            k.box(F::Ceramic, {-W * 0.5 + t * 0.5, H * 0.5, D * 0.5}, {t, H, D - 2 * t + 0.01}, 0.02, kPorcelain);
            k.box(F::Ceramic, {W * 0.5 - t * 0.5, H * 0.5, D * 0.5}, {t, H, D - 2 * t + 0.01}, 0.02, kPorcelain);
            k.box(F::Ceramic, {0, 0.10, D * 0.5}, {W - 2 * t + 0.01, 0.20, D - 2 * t + 0.01}, 0.04, kPorcelain * 0.97);
            k.turned(F::Metal, {-W * 0.5 + 0.12, H, D * 0.5}, {{0.02, 0.0}, {0.018, 0.12}, {0.0, 0.13}}, kChrome, 1.0, 12);
            k.rod(F::Metal, {-W * 0.5 + 0.12, H + 0.11, D * 0.5}, {-W * 0.5 + 0.26, H + 0.10, D * 0.5}, 0.011, kChrome, 10);
            break;
        }
        case Piece::Toilet: {
            k.out.size = {0.40, 0.80, 0.70};
            k.out.solid = false;
            k.box(F::Ceramic, {0, 0.60, 0.10}, {0.38, 0.36, 0.17}, 0.025, kPorcelain);            // cistern
            k.turned(F::Ceramic, {0, 0, 0.42}, {{0.12, 0.0}, {0.13, 0.05}, {0.15, 0.22}, {0.18, 0.38}, {0.17, 0.40},
                                                {0.0, 0.40}}, kPorcelain, 1.3, 24);              // pedestal + bowl
            k.turned(F::Hard, {0, 0.40, 0.42}, {{0.17, 0.0}, {0.175, 0.012}, {0.16, 0.025}, {0.0, 0.025}}, kWhite, 1.3, 24);
            k.box(F::Metal, {0.12, 0.79, 0.10}, {0.06, 0.01, 0.03}, 0.003, kChrome, 1);           // flush button
            break;
        }
        case Piece::Vanity: {
            k.out.size = {0.80, 1.95, 0.50};
            k.box(F::Wood, {0, 0.47, 0.24}, {0.80, 0.56, 0.48}, 0.006, wood, 1);
            k.front(F::Wood, -0.4, 0.4, 0.20, 0.74, 0.48, wood * 1.04, kChrome);
            k.box(F::Ceramic, {0, 0.765, 0.25}, {0.80, 0.03, 0.50}, 0.006, kPorcelain, 1);
            k.turned(F::Ceramic, {0, 0.78, 0.27}, {{0.0, 0.0}, {0.18, 0.0}, {0.20, 0.09}, {0.19, 0.10}, {0.15, 0.03},
                                                   {0.0, 0.03}}, kPorcelain, 0.8, 24);            // vessel basin
            k.turned(F::Metal, {0, 0.78, 0.05}, {{0.022, 0.0}, {0.018, 0.20}, {0.0, 0.21}}, kChrome, 1.0, 12);
            k.rod(F::Metal, {0, 0.98, 0.05}, {0, 0.96, 0.17}, 0.011, kChrome, 10);
            k.box(F::Metal, {0, 1.45, 0.012}, {0.72, 0.90, 0.012}, 0.006, kChrome, 1);             // the mirror
            break;
        }
        case Piece::LoungeChair: {
            k.out.size = {0.84, 0.80, 0.84};
            k.box(F::Fabric, {0, 0.30, 0.44}, {0.84, 0.24, 0.80}, 0.04, fabric);
            k.box(F::Fabric, {0, 0.46, 0.50}, {0.56, 0.12, 0.62}, 0.05, fabric * 1.04);
            k.boxTilted(F::Fabric, {0, 0.58, 0.12}, {0.84, 0.48, 0.16}, 0.06, -0.12, fabric * 0.96);
            for (double x : {-0.36, 0.36}) k.box(F::Fabric, {x, 0.50, 0.46}, {0.13, 0.24, 0.74}, 0.05, fabric * 0.96);
            for (double x : {-0.36, 0.36})
                for (double z : {0.10, 0.78}) k.leg(F::Wood, {x, 0, z}, 0.18, 0.04, wood);
            break;
        }
        case Piece::Planter: {
            k.out.size = {0.9, 1.3, 0.9};
            k.turned(F::Ceramic, {0, 0, 0.45}, {{0.0, 0.0}, {0.22, 0.0}, {0.28, 0.42}, {0.29, 0.45}, {0.0, 0.45}},
                     Vec3(0.30, 0.30, 0.31), 1.0, 20);
            RenderMesh leaves = MeshBuilder::icosphere(2);
            MeshBuilder::displaceNoise(leaves, Vec3(0, 0, 0), 0.25, 3.0, 7u);
            for (Vertex& v : leaves.vertices) {
                v.position = Vec3(v.position.x * 0.36, v.position.y * 0.45 + 0.85, v.position.z * 0.36 + 0.45);
                v.color = Vec3(0.20, 0.36, 0.16);
            }
            MeshBuilder::append(k.m(F::Fabric), leaves);
            break;
        }
        case Piece::Picture: {
            // WALL ART (Glenn: "the walls are bare"): a framed canvas hung at eye height -- an abstract of two to
            // four colour fields on a ground, in one of three sizes and three frames, the palette by the variant.
            static const Vec3 kSizes[3] = {{0.60, 0.80, 0}, {1.00, 0.70, 0}, {1.40, 0.90, 0}};
            const Vec3 sz = kSizes[style % 3];
            const Real W = sz.x, H = sz.y, cy = 1.55;
            k.out.size = {W + 0.1, cy + H * 0.5 + 0.06, 0.05};
            k.out.solid = false;
            static const Vec3 kFrames[3] = {{0.07, 0.07, 0.08}, {0.92, 0.91, 0.88}, {0.0, 0.0, 0.0}};
            const Vec3 frame = (style / 3) % 3 == 2 ? wood : kFrames[(style / 3) % 3];
            const F fm = (style / 3) % 3 == 2 ? F::Wood : F::Hard;
            const Real fw = 0.04;
            k.box(fm, {0, cy + H * 0.5 + fw * 0.5, 0.02}, {W + 2 * fw, fw, 0.04}, 0.004, frame, 1);
            k.box(fm, {0, cy - H * 0.5 - fw * 0.5, 0.02}, {W + 2 * fw, fw, 0.04}, 0.004, frame, 1);
            k.box(fm, {-W * 0.5 - fw * 0.5, cy, 0.02}, {fw, H, 0.04}, 0.004, frame, 1);
            k.box(fm, {W * 0.5 + fw * 0.5, cy, 0.02}, {fw, H, 0.04}, 0.004, frame, 1);
            static const Vec3 kPal[6][4] = {
                {{0.93, 0.90, 0.84}, {0.85, 0.36, 0.22}, {0.16, 0.24, 0.40}, {0.92, 0.72, 0.30}},
                {{0.20, 0.22, 0.26}, {0.84, 0.80, 0.72}, {0.70, 0.30, 0.30}, {0.40, 0.56, 0.62}},
                {{0.88, 0.86, 0.80}, {0.36, 0.52, 0.40}, {0.70, 0.62, 0.44}, {0.22, 0.30, 0.26}},
                {{0.96, 0.94, 0.90}, {0.10, 0.10, 0.12}, {0.86, 0.20, 0.18}, {0.20, 0.36, 0.70}},
                {{0.82, 0.74, 0.64}, {0.58, 0.38, 0.30}, {0.92, 0.84, 0.70}, {0.40, 0.30, 0.26}},
                {{0.30, 0.40, 0.52}, {0.70, 0.80, 0.86}, {0.94, 0.86, 0.60}, {0.16, 0.20, 0.30}}};
            const Vec3* pal = kPal[(variant ^ (style * 7)) % 6];
            k.box(F::Hard, {0, cy, 0.022}, {W, H, 0.012}, 0.0, pal[0], 1);   // the ground
            // The fields: seeded rectangles, kept inside the canvas.
            uint32_t r = variant * 2654435761u + style * 40503u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            const int fields = 2 + static_cast<int>(next() * 3);
            for (int f = 0; f < fields; ++f) {
                const Real fwid = W * (0.18 + 0.45 * next()), fht = H * (0.15 + 0.5 * next());
                const Real fx = (next() - 0.5) * (W - fwid), fyy = (next() - 0.5) * (H - fht);
                k.box(F::Hard, {fx, cy + fyy, 0.029 + 0.001 * f}, {fwid, fht, 0.002}, 0.0, pal[1 + f % 3], 1);
            }
            break;
        }
        case Piece::Shelving: {
            // A CLOSET's shelves: two white uprights, four shelves of folded clothes and boxes, a hanging rail of
            // garments down one half.
            k.out.size = {1.2, 2.0, 0.5};
            for (double x : {-0.58, 0.58}) k.box(F::Hard, {x, 1.0, 0.25}, {0.03, 2.0, 0.48}, 0.003, kWhite, 1);
            for (double y : {0.30, 0.80, 1.40, 1.90}) k.box(F::Hard, {0.29, y, 0.25}, {0.56, 0.025, 0.46}, 0.002, kWhite, 1);
            k.box(F::Hard, {0, 1.98, 0.25}, {1.16, 0.025, 0.48}, 0.002, kWhite, 1);
            uint32_t r = variant * 747796405u + 1u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            for (double y : {0.31, 0.81, 1.41}) {
                Real x = 0.05;
                while (x < 0.5) {
                    const Real w = 0.14 + 0.12 * next(), hh = 0.10 + 0.18 * next();
                    if (x + w > 0.54) break;
                    k.box(F::Fabric, {x + w * 0.5, y + 0.013 + hh * 0.5, 0.24}, {w, hh, 0.34}, 0.02,
                          kFabric[static_cast<int>(next() * 8) % 8], 1);
                    x += w + 0.02;
                }
            }
            k.rod(F::Metal, {-0.56, 1.75, 0.25}, {0.0, 1.75, 0.25}, 0.012, kChrome, 10);
            for (int g = 0; g < 7; ++g) {
                const Real x = -0.52 + g * 0.075;
                k.box(F::Fabric, {x, 1.30, 0.25}, {0.018, 0.84, 0.42}, 0.008, kFabric[(variant + g * 3) % 8]);
            }
            break;
        }
        case Piece::DeskPod: {
            // BENCHING, running from the windows into the room: three desks (style bit 4: two) either side of a
            // low fabric screen on the x = 0 line, a monitor, keyboard and chair at each. Footprint 3.1 (x) by
            // 4.8 (z) for six, 3.2 for four.
            const bool four = (style & 4u) != 0;
            const Real L = four ? 3.2 : 4.8;
            k.out.size = {3.1, 1.38, L};
            k.out.colliderH = 0.75;   // the desks collide; the low screen above them need not
            k.box(F::Wood, {0, 0.735, L * 0.5}, {1.6, 0.03, L}, 0.004, wood, 1);
            for (double x : {-0.76, 0.76})
                for (double z : {0.05, L * 0.5, L - 0.05}) k.leg(F::Metal, {x, 0, z}, 0.72, 0.04, kWhite * 0.9);
            k.box(F::Fabric, {0, 1.05, L * 0.5}, {0.04, 0.62, L}, 0.012, fabric * 0.7 + Vec3(0.12, 0.12, 0.12));
            const std::vector<double> seatsZ = four ? std::vector<double>{0.8, 2.4} : std::vector<double>{0.8, 2.4, 4.0};
            for (double z : seatsZ)
                for (int side = 0; side < 2; ++side) {
                    const Real s = side == 0 ? -1.0 : 1.0;
                    const Real xm = s * 0.22;
                    k.box(F::Hard, {xm, 0.76, z}, {0.16, 0.012, 0.22}, 0.004, kBlack, 1);              // foot
                    k.box(F::Hard, {xm, 0.92, z}, {0.02, 0.30, 0.04}, 0.004, kBlack, 1);               // neck
                    k.box(F::Hard, {xm + s * 0.02, 1.06, z}, {0.022, 0.34, 0.58}, 0.004, kBlack, 1);   // screen
                    k.box(F::Hard, {s * 0.55, 0.75, z}, {0.14, 0.02, 0.44}, 0.004, kBlack, 1);         // keyboard
                    const Real xch = s * 1.15;
                    k.turned(F::Metal, {xch, 0, z}, {{0.26, 0.0}, {0.26, 0.03}, {0.03, 0.06}, {0.025, 0.44}, {0.0, 0.45}}, kBlack, 1.0, 10);
                    k.box(F::Fabric, {xch, 0.48, z}, {0.46, 0.07, 0.48}, 0.03, fabric * 0.75);
                    k.box(F::Fabric, {xch + s * 0.22, 0.80, z}, {0.05, 0.50, 0.44}, 0.025, fabric * 0.75);
                }
            break;
        }
        case Piece::Cubicle: {
            // A CUBICLE: fabric partitions on three sides (1.4 m), an L-shaped worktop, a pedestal, the chair
            // and the monitor; open at +z.
            k.out.size = {2.4, 1.45, 2.4};
            const Vec3 panel = fabric * 0.6 + Vec3(0.18, 0.18, 0.18);
            k.box(F::Fabric, {0, 0.70, 0.03}, {2.4, 1.4, 0.06}, 0.015, panel);
            for (double x : {-1.17, 1.17}) k.box(F::Fabric, {x, 0.70, 1.2}, {0.06, 1.4, 2.4}, 0.015, panel);
            for (double x : {-1.17, 1.17}) k.box(F::Metal, {x, 1.405, 1.2}, {0.07, 0.012, 2.4}, 0.0, kSteel, 1);
            k.box(F::Metal, {0, 1.405, 0.03}, {2.4, 0.012, 0.07}, 0.0, kSteel, 1);
            k.box(F::Wood, {0, 0.73, 0.40}, {2.2, 0.03, 0.70}, 0.004, wood, 1);                 // the back run
            k.box(F::Wood, {-0.80, 0.73, 1.15}, {0.60, 0.03, 0.85}, 0.004, wood, 1);            // the return
            k.box(F::Hard, {0.75, 0.33, 0.40}, {0.42, 0.62, 0.58}, 0.004, kSteel * 0.85, 1);    // the pedestal
            k.box(F::Hard, {0, 0.76, 0.22}, {0.22, 0.012, 0.16}, 0.004, kBlack, 1);
            k.box(F::Hard, {0, 0.92, 0.20}, {0.04, 0.30, 0.02}, 0.004, kBlack, 1);
            k.box(F::Hard, {0, 1.06, 0.23}, {0.58, 0.34, 0.022}, 0.004, kBlack, 1);
            k.turned(F::Metal, {0, 0, 1.2}, {{0.26, 0.0}, {0.26, 0.03}, {0.03, 0.06}, {0.025, 0.44}, {0.0, 0.45}}, kBlack, 1.0, 10);
            k.box(F::Fabric, {0, 0.48, 1.2}, {0.48, 0.07, 0.46}, 0.03, fabric * 0.75);
            k.box(F::Fabric, {0, 0.80, 1.43}, {0.44, 0.50, 0.05}, 0.025, fabric * 0.75);
            break;
        }
        case Piece::MeetingTable: {
            // A meeting table for six: a long top on two pedestal legs, three chairs a side.
            k.out.size = {3.0, 0.95, 2.4};
            k.box(F::Wood, {0, 0.735, 1.2}, {2.6, 0.04, 1.1}, 0.015, wood, 1);
            for (double x : {-0.8, 0.8}) k.box(F::Metal, {x, 0.36, 1.2}, {0.12, 0.72, 0.7}, 0.01, kSteel * 0.7, 1);
            for (double x : {-0.85, 0.0, 0.85})
                for (int side = 0; side < 2; ++side) {
                    const Real s = side == 0 ? -1.0 : 1.0;
                    const Real zch = 1.2 + s * 0.85;
                    k.turned(F::Metal, {x, 0, zch}, {{0.24, 0.0}, {0.24, 0.03}, {0.03, 0.06}, {0.025, 0.44}, {0.0, 0.45}}, kChrome, 1.0, 10);
                    k.box(F::Fabric, {x, 0.48, zch}, {0.46, 0.07, 0.44}, 0.03, fabric);
                    k.box(F::Fabric, {x, 0.78, zch + s * 0.21}, {0.44, 0.46, 0.05}, 0.025, fabric);
                }
            break;
        }
        case Piece::Whiteboard: {
            // style bit 2: a LECTURE HALL's board, 4 m x 1.3 m (Glenn: "the lecture hall should have a bigger whiteboard")
            const bool wide = (style & 4u) != 0;
            const double bw = wide ? 4.0 : 1.9, bh = wide ? 1.3 : 1.04, cy = wide ? 1.55 : 1.45;
            k.out.size = {bw, wide ? 2.3 : 2.1, 0.06};
            k.out.solid = false;
            k.box(F::Metal, {0, cy, 0.02}, {bw - 0.06, bh, 0.03}, 0.004, kChrome, 1);
            k.box(F::Ceramic, {0, cy, 0.037}, {bw - 0.12, bh - 0.06, 0.004}, 0.0, kPorcelain, 1);
            k.box(F::Metal, {0, cy - bh * 0.5 - 0.01, 0.05}, {bw * 0.74, 0.02, 0.06}, 0.004, kChrome, 1);          // the pen tray
            k.box(F::Hard, {-0.3, 1.6, 0.04}, {0.6, 0.012, 0.002}, 0.0, Vec3(0.15, 0.25, 0.6), 1);   // a scrawl
            k.box(F::Hard, {0.2, 1.35, 0.04}, {0.8, 0.012, 0.002}, 0.0, Vec3(0.6, 0.15, 0.15), 1);
            break;
        }
        case Piece::ShopCounter: {
            // The SHOP COUNTER: a panelled front, a worktop, the till and a card reader. Faces +z (the customer).
            k.out.size = {1.8, 1.25, 0.7};
            k.out.colliderH = 1.0;
            k.box(F::Wood, {0, 0.48, 0.35}, {1.8, 0.96, 0.66}, 0.006, wood, 1);
            k.box(F::Hard, {0, 1.0, 0.36}, {1.84, 0.04, 0.72}, 0.006, (style & 1u) ? kWhite : kBlack * 3.0, 1);
            k.box(F::Hard, {0.4, 1.10, 0.25}, {0.36, 0.16, 0.30}, 0.01, kBlack, 1);        // the till
            k.box(F::Hard, {0.4, 1.22, 0.22}, {0.32, 0.18, 0.03}, 0.004, kBlack, 1);
            k.box(F::Hard, {-0.3, 1.05, 0.5}, {0.08, 0.06, 0.14}, 0.008, kBlack, 1);       // card reader
            break;
        }
        case Piece::Gondola: {
            // A GONDOLA: a double-sided run of shelves (the grocery aisle), four shelves a side of boxed and
            // bottled goods. Runs along x; both faces (+z, -z) stocked.
            k.out.size = {2.4, 1.6, 1.0};
            k.box(F::Hard, {0, 0.06, 0.5}, {2.4, 0.12, 0.96}, 0.004, kBlack * 3.0, 1);       // base
            k.box(F::Hard, {0, 0.85, 0.5}, {2.4, 1.5, 0.04}, 0.004, kWhite * 0.9, 1);        // spine
            uint32_t r = variant * 2246822519u + 3u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            static const Vec3 kGoods[8] = {{0.85, 0.20, 0.15}, {0.95, 0.75, 0.15}, {0.20, 0.45, 0.80}, {0.25, 0.65, 0.30},
                                           {0.90, 0.90, 0.88}, {0.55, 0.30, 0.65}, {0.95, 0.50, 0.15}, {0.30, 0.30, 0.32}};
            for (int side = 0; side < 2; ++side) {
                const Real s = side == 0 ? -1.0 : 1.0;
                for (double y : {0.14, 0.50, 0.86, 1.22}) {
                    k.box(F::Hard, {0, y, 0.5 + s * 0.24}, {2.36, 0.02, 0.44}, 0.002, kWhite * 0.9, 1);
                    Real x = -1.15;
                    while (x < 1.1) {
                        const Real w = 0.10 + 0.22 * next(), hh = 0.14 + 0.16 * next();
                        if (x + w > 1.15) break;
                        k.box(F::Hard, {x + w * 0.5, y + 0.01 + hh * 0.5, 0.5 + s * 0.26}, {w - 0.01, hh, 0.32},
                              0.0, kGoods[static_cast<int>(next() * 8) % 8], 1);
                        x += w;
                    }
                }
            }
            break;
        }
        case Piece::WallShelf: {
            // WALL SHELVING, one-sided against a wall: five shelves of stock.
            k.out.size = {2.0, 2.1, 0.45};
            k.box(F::Wood, {0, 1.05, 0.03}, {2.0, 2.1, 0.04}, 0.003, wood * 0.9, 1);
            for (double x : {-0.98, 0.98}) k.box(F::Wood, {x, 1.05, 0.22}, {0.04, 2.1, 0.44}, 0.003, wood, 1);
            uint32_t r = variant * 3266489917u + 5u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            for (double y : {0.10, 0.50, 0.90, 1.30, 1.70}) {
                k.box(F::Wood, {0, y, 0.22}, {1.92, 0.025, 0.42}, 0.002, wood, 1);
                Real x = -0.92;
                while (x < 0.9) {
                    const Real w = 0.08 + 0.18 * next(), hh = 0.12 + 0.2 * next();
                    if (x + w > 0.94) break;
                    k.box(F::Hard, {x + w * 0.5, y + 0.013 + hh * 0.5, 0.22}, {w - 0.01, hh, 0.30}, 0.0,
                          kFabric[static_cast<int>(next() * 8) % 8] * 1.2, 1);
                    x += w;
                }
            }
            break;
        }
        case Piece::ClothesRack: {
            // A boutique's CLOTHES RAIL: two uprights, a chrome rail, a dozen garments on hangers.
            k.out.size = {1.6, 1.65, 0.6};
            k.out.solid = false;
            for (double x : {-0.75, 0.75}) {
                k.rod(F::Metal, {x, 0.0, 0.3}, {x, 1.55, 0.3}, 0.014, kChrome, 10);
                k.box(F::Metal, {x, 0.01, 0.3}, {0.06, 0.02, 0.55}, 0.004, kChrome, 1);
            }
            k.rod(F::Metal, {-0.75, 1.55, 0.3}, {0.75, 1.55, 0.3}, 0.012, kChrome, 10);
            for (int g = 0; g < 12; ++g) {
                const Real x = -0.66 + g * 0.12;
                const Real len = 0.6 + 0.35 * (((variant + g * 7) % 5) / 4.0);
                k.box(F::Fabric, {x, 1.50 - len * 0.5, 0.3}, {0.03, len, 0.46}, 0.012, kFabric[(variant + g * 3) % 8] * 1.1);
            }
            break;
        }
        case Piece::CafeTable: {
            // A CAFE TABLE for two: a round top on a pedestal, two bentwood-ish chairs.
            k.out.size = {1.5, 0.9, 1.5};
            k.turned(F::Metal, {0, 0, 0.75}, {{0.24, 0.0}, {0.24, 0.02}, {0.03, 0.04}, {0.03, 0.72}, {0.0, 0.73}}, kBlack, 1.0, 14);
            k.turned(F::Wood, {0, 0.72, 0.75}, {{0.0, 0.0}, {0.36, 0.0}, {0.36, 0.03}, {0.0, 0.03}}, wood, 1.0, 24);
            for (int side = 0; side < 2; ++side) {
                const Real z = side == 0 ? 0.18 : 1.32, s = side == 0 ? -1.0 : 1.0;
                for (double x : {-0.18, 0.18})
                    for (double dz : {-0.16, 0.16}) k.leg(F::Wood, {x, 0, z + dz}, 0.44, 0.03, wood * 0.85);
                k.box(F::Wood, {0, 0.46, z}, {0.42, 0.04, 0.40}, 0.01, wood * 0.9, 1);
                k.box(F::Wood, {0, 0.72, z + s * 0.19}, {0.40, 0.42, 0.03}, 0.01, wood * 0.9, 1);
            }
            break;
        }
        case Piece::DisplayCase: {
            // A bakery / jeweller's DISPLAY CASE: a lit glass case on a cabinet, goods on two glass shelves.
            k.out.size = {1.6, 1.25, 0.7};
            k.box(F::Wood, {0, 0.42, 0.35}, {1.6, 0.84, 0.66}, 0.006, wood, 1);
            k.box(F::Hard, {0, 1.24, 0.35}, {1.6, 0.03, 0.66}, 0.004, kWhite, 1);
            for (double x : {-0.79, 0.79}) k.box(F::Metal, {x, 1.04, 0.35}, {0.02, 0.38, 0.64}, 0.0, kChrome, 1);
            uint32_t r = variant * 668265263u + 11u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            static const Vec3 kTreats[4] = {{0.80, 0.55, 0.30}, {0.95, 0.85, 0.65}, {0.45, 0.25, 0.15}, {0.90, 0.40, 0.45}};
            for (double y : {0.86, 1.04})
                for (int i = 0; i < 7; ++i)
                    k.turned(F::Hard, {-0.6 + i * 0.2, y, 0.25 + 0.2 * next()}, {{0.0, 0.0}, {0.06, 0.0}, {0.06, 0.03}, {0.0, 0.05}},
                             kTreats[static_cast<int>(next() * 4) % 4], 1.0, 10);
            break;
        }
        case Piece::DrinksFridge: {
            // A DRINKS FRIDGE: a tall cabinet, a glass door, five shelves of bottles and cans.
            k.out.size = {0.8, 2.0, 0.75};
            k.box(F::Hard, {0, 1.0, 0.37}, {0.8, 2.0, 0.72}, 0.006, kWhite * 0.95, 1);
            k.box(F::Hard, {0, 1.0, 0.72}, {0.70, 1.80, 0.02}, 0.0, Vec3(0.05, 0.06, 0.08), 1);   // the dark interior
            uint32_t r = variant * 2654435761u + 13u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            static const Vec3 kCans[6] = {{0.85, 0.10, 0.10}, {0.10, 0.30, 0.80}, {0.95, 0.85, 0.20}, {0.20, 0.70, 0.30},
                                          {0.90, 0.90, 0.90}, {0.55, 0.20, 0.10}};
            for (int sh = 0; sh < 5; ++sh) {
                const Real y = 0.25 + sh * 0.34;
                for (int i = 0; i < 6; ++i)
                    k.turned(F::Hard, {-0.27 + i * 0.108, y, 0.62}, {{0.0, 0.0}, {0.035, 0.0}, {0.035, 0.18}, {0.015, 0.24}, {0.0, 0.25}},
                             kCans[static_cast<int>(next() * 6) % 6], 1.0, 8);
            }
            k.box(F::Metal, {0, 1.0, 0.745}, {0.74, 1.84, 0.01}, 0.0, Vec3(0.55, 0.62, 0.66), 1);   // the glass
            k.rod(F::Metal, {0.32, 0.7, 0.77}, {0.32, 1.3, 0.77}, 0.01, kChrome, 8);
            break;
        }
        case Piece::Bookcase: {
            // A BOOKCASE: five shelves of books, spines in every colour, the odd one leaning.
            k.out.size = {1.2, 2.0, 0.35};
            k.box(F::Wood, {0, 1.0, 0.02}, {1.2, 2.0, 0.03}, 0.003, wood * 0.85, 1);
            for (double x : {-0.585, 0.585}) k.box(F::Wood, {x, 1.0, 0.18}, {0.03, 2.0, 0.34}, 0.003, wood, 1);
            uint32_t r = variant * 1597334677u + 7u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            // BOOKS (Glenn: "It would be nice to have some more variety in the books"): cloth and leather in sixteen
            // colours, every height and depth, gilt or cream title bands on the spine, now and then a stack lying
            // flat. Each placement is its own bookcase (the library's `variety`: eight designs by position).
            static const Vec3 kSpines[16] = {{0.55, 0.12, 0.10}, {0.12, 0.20, 0.45}, {0.15, 0.40, 0.22}, {0.85, 0.80, 0.70},
                                             {0.10, 0.10, 0.10}, {0.75, 0.55, 0.20}, {0.45, 0.30, 0.50}, {0.30, 0.50, 0.55},
                                             {0.40, 0.22, 0.12}, {0.60, 0.38, 0.22}, {0.35, 0.08, 0.10}, {0.08, 0.12, 0.25},
                                             {0.70, 0.62, 0.40}, {0.55, 0.55, 0.52}, {0.20, 0.30, 0.15}, {0.80, 0.30, 0.20}};
            static const Vec3 kBands[3] = {{0.80, 0.65, 0.25}, {0.88, 0.85, 0.75}, {0.08, 0.08, 0.08}};
            for (double y : {0.04, 0.42, 0.80, 1.18, 1.56}) {
                k.box(F::Wood, {0, y, 0.18}, {1.14, 0.025, 0.32}, 0.002, wood, 1);
                Real x = -0.55;
                const int series = static_cast<int>(next() * 16);   // a shelf often holds a set in one binding
                while (x < 0.53) {
                    if (next() < 0.07 && x < 0.30) {   // a stack lying flat
                        const int n = 3 + static_cast<int>(next() * 3);
                        Real yy = y + 0.013;
                        for (int j = 0; j < n; ++j) {
                            const Real th = 0.025 + 0.02 * next();
                            k.box(F::Hard, {x + 0.12, yy + th * 0.5, 0.17}, {0.22 + 0.04 * next(), th, 0.17 + 0.04 * next()}, 0.002,
                                  kSpines[static_cast<int>(next() * 16) % 16], 1);
                            yy += th;
                        }
                        x += 0.28;
                        continue;
                    }
                    const Real w = 0.02 + 0.045 * next(), hh = 0.18 + 0.13 * next(), dd = 0.16 + 0.08 * next();
                    if (x + w > 0.56) break;
                    const Vec3 col = next() < 0.45 ? kSpines[series] * (0.85 + 0.3 * next())
                                                   : kSpines[static_cast<int>(next() * 16) % 16];
                    k.box(F::Hard, {x + w * 0.5, y + 0.013 + hh * 0.5, 0.29 - dd * 0.5}, {w - 0.003, hh, dd}, 0.002, col, 1);
                    // title bands on the spine's face
                    if (w > 0.026 && next() < 0.75) {
                        const Vec3 band = kBands[static_cast<int>(next() * 3) % 3];
                        const int nb = 1 + static_cast<int>(next() * 2);
                        for (int j = 0; j < nb; ++j) {
                            const Real by = y + 0.013 + hh * (0.62 + 0.22 * j + 0.06 * next());
                            k.box(F::Hard, {x + w * 0.5, std::min(by, y + 0.013 + hh - 0.02), 0.292}, {w - 0.008, 0.012, 0.004},
                                  0.0, band, 1);
                        }
                    }
                    x += w + (next() < 0.04 ? 0.04 : 0.0);   // the odd gap where a book is out
                }
            }
            break;
        }
        case Piece::PalletRack: {
            // A big box's PALLET RACKING (Glenn, 2026-10-01: "big box stores like Costco"): a double-sided bay of
            // steel uprights and orange beams, shelf-height goods on the floor level, shrink-wrapped pallets of
            // cartons on the two levels above. Runs along x, stocked on both faces.
            k.out.size = {2.8, 4.4, 2.2};
            k.out.colliderH = 2.6;   // high enough to wall the aisle; the pallets above are out of reach
            const Vec3 upright(0.16, 0.30, 0.55), beam(0.92, 0.42, 0.10);
            for (double x : {-1.37, 1.37})
                for (double z : {0.06, 1.1, 2.14})
                    k.box(F::Metal, {x, 2.2, z}, {0.07, 4.4, 0.07}, 0.0, upright, 1);
            uint32_t r = variant * 2654435761u + 11u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            static const Vec3 kCarton[5] = {{0.62, 0.48, 0.32}, {0.70, 0.56, 0.38}, {0.55, 0.42, 0.28},
                                            {0.85, 0.85, 0.82}, {0.30, 0.42, 0.62}};
            for (double y : {1.55, 3.0}) {
                for (double z : {0.06, 2.14})
                    k.box(F::Metal, {0, y, z}, {2.74, 0.12, 0.06}, 0.0, beam, 1);
                for (int side = 0; side < 2; ++side) {
                    const Real zc = side == 0 ? 0.58 : 1.62;
                    for (double x : {-0.68, 0.68}) {
                        k.box(F::Wood, {x, y + 0.13, zc}, {1.2, 0.14, 0.95}, 0.0, Vec3(0.72, 0.60, 0.42), 1);   // pallet
                        const Real hh = 0.7 + 0.55 * next();
                        k.box(F::Hard, {x, y + 0.2 + hh * 0.5, zc}, {1.14, hh, 0.9}, 0.01,
                              kCarton[static_cast<int>(next() * 5) % 5], 1);
                    }
                }
            }
            // the floor level: shelf-height goods both sides
            for (int side = 0; side < 2; ++side) {
                const Real zc = side == 0 ? 0.55 : 1.65;
                k.box(F::Metal, {0, 0.08, zc}, {2.7, 0.04, 0.9}, 0.0, kSteel * 0.8, 1);
                Real x = -1.3;
                while (x < 1.25) {
                    const Real w = 0.25 + 0.35 * next(), hh = 0.35 + 0.6 * next();
                    if (x + w > 1.32) break;
                    k.box(F::Hard, {x + w * 0.5, 0.1 + hh * 0.5, zc}, {w - 0.02, hh, 0.8}, 0.0,
                          kCarton[static_cast<int>(next() * 5) % 5] * (0.8 + 0.4 * next()), 1);
                    x += w;
                }
            }
            break;
        }
        case Piece::Checkout: {
            // A CHECKOUT LANE: a long counter running out from the front (z), its belt, the register and card
            // reader at the far end, and the lane's numbered light on a pole.
            k.out.size = {0.9, 2.3, 3.2};
            k.out.colliderH = 0.95;
            k.box(F::Hard, {0, 0.45, 1.6}, {0.85, 0.9, 3.2}, 0.01, Vec3(0.30, 0.31, 0.34), 2);    // the counter
            k.box(F::Hard, {0, 0.915, 1.1}, {0.6, 0.03, 2.0}, 0.005, kBlack, 1);                 // the belt
            k.box(F::Hard, {0, 0.915, 2.6}, {0.75, 0.03, 1.0}, 0.005, kSteel, 1);                // bagging well
            k.box(F::Hard, {0.25, 1.15, 2.3}, {0.30, 0.28, 0.08}, 0.01, kBlack, 1);              // register screen
            k.rod(F::Metal, {0.25, 0.92, 2.3}, {0.25, 1.02, 2.3}, 0.02, kSteel, 8);
            k.box(F::Hard, {-0.3, 1.0, 2.1}, {0.12, 0.16, 0.08}, 0.01, Vec3(0.15, 0.15, 0.17), 1);   // card reader
            k.rod(F::Metal, {0.38, 0.9, 0.1}, {0.38, 2.15, 0.1}, 0.025, kSteel, 8);                // the light pole
            k.box(F::Hard, {0.38, 2.2, 0.1}, {0.3, 0.22, 0.12}, 0.01, Vec3(0.95, 0.85, 0.25), 1);   // its lamp
            break;
        }
        case Piece::Bench: {
            // A PARK BENCH: timber slats on two cast-iron ends with armrests, a slatted back, seats three. Its back at
            // z = 0, the seat facing +z.
            k.out.size = {1.80, 0.86, 0.66};
            k.out.colliderH = 0.46;   // the seat: you walk round the back, not through it
            const Vec3 iron(0.14, 0.15, 0.16);
            for (double x : {-0.84, 0.84}) {
                k.box(F::Metal, {x, 0.22, 0.10}, {0.06, 0.44, 0.06}, 0.01, iron, 1);       // back leg
                k.box(F::Metal, {x, 0.22, 0.56}, {0.06, 0.44, 0.06}, 0.01, iron, 1);       // front leg
                k.box(F::Metal, {x, 0.42, 0.33}, {0.06, 0.05, 0.56}, 0.01, iron, 1);       // seat rail
                k.boxTilted(F::Metal, {x, 0.66, 0.08}, {0.06, 0.48, 0.05}, 0.01, -0.22, iron); // back rail
                k.box(F::Metal, {x, 0.64, 0.42}, {0.07, 0.04, 0.40}, 0.01, iron, 1);       // armrest
                k.box(F::Metal, {x, 0.53, 0.60}, {0.05, 0.22, 0.04}, 0.01, iron, 1);
            }
            for (int i = 0; i < 5; ++i)   // seat slats
                k.box(F::Wood, {0, 0.455, 0.14 + i * 0.095}, {1.74, 0.03, 0.075}, 0.006, wood, 1);
            for (int i = 0; i < 3; ++i)   // back slats, leaning back
                k.boxTilted(F::Wood, {0, 0.56 + i * 0.11, 0.095 - i * 0.025}, {1.74, 0.075, 0.03}, 0.006, -0.22, wood);
            break;
        }
        case Piece::PlazaBench: {
            // A PLAZA BENCH: backless, a thick timber top on two stone plinths; sat on from either side (+z is "front").
            k.out.size = {1.80, 0.46, 0.55};
            const Vec3 stone(0.66, 0.64, 0.60);
            for (double x : {-0.70, 0.70}) k.box(F::Hard, {x, 0.20, 0.275}, {0.30, 0.40, 0.50}, 0.015, stone, 1);
            k.box(F::Wood, {0, 0.43, 0.275}, {1.80, 0.06, 0.55}, 0.01, wood, 1);
            break;
        }
        case Piece::BistroTable: {
            // A BISTRO TABLE: a round top on a cast pedestal (café terraces, café floors; its chairs are their own).
            k.out.size = {0.70, 0.74, 0.70};
            k.turned(F::Metal, {0, 0, 0.35}, {{0.22, 0.0}, {0.22, 0.02}, {0.03, 0.05}, {0.03, 0.71}, {0.0, 0.72}}, kBlack, 1.0, 14);
            k.turned(F::Wood, {0, 0.71, 0.35}, {{0.0, 0.0}, {0.35, 0.0}, {0.35, 0.03}, {0.0, 0.03}}, wood, 1.0, 24);
            break;
        }
        case Piece::BistroChair: {
            // A BISTRO CHAIR: a bentwood-style seat and hooped back on four legs. Back at z = 0, facing +z.
            k.out.size = {0.44, 0.84, 0.48};
            k.out.solid = false;
            for (double x : {-0.17, 0.17})
                for (double z : {0.06, 0.42}) k.leg(F::Wood, {x, 0, z}, 0.45, 0.028, wood * 0.85);
            k.turned(F::Wood, {0, 0.45, 0.25}, {{0.0, 0.0}, {0.21, 0.0}, {0.21, 0.035}, {0.0, 0.035}}, wood * 0.9, 1.0, 20);
            for (double x : {-0.17, 0.17}) k.rod(F::Wood, {x, 0.47, 0.06}, {x * 0.8, 0.84, 0.03}, 0.014, wood * 0.85, 8);
            k.boxTilted(F::Wood, {0, 0.76, 0.035}, {0.34, 0.10, 0.025}, 0.01, -0.08, wood * 0.9);
            break;
        }
        // ---- GOODS (the furniture library, M3): small things the dressing pass stands on anchors. Each sits on
        // y = 0 (the surface), centred on x = 0, z = size.z / 2. The variant's style bits pick the colourway.
        case Piece::DeskLamp: {
            k.out.size = {0.22, 0.46, 0.22};
            k.out.solid = false;
            const Vec3 shade = (variant & 32u) ? kBlack * 2.0 : Vec3(0.75, 0.22, 0.18);
            k.turned(F::Metal, {0, 0, 0.11}, {{0.0, 0.0}, {0.09, 0.0}, {0.09, 0.02}, {0.02, 0.03}, {0.0, 0.03}}, kBlack * 2.0, 1.0, 16);
            k.rod(F::Metal, {0, 0.02, 0.11}, {0.0, 0.30, 0.13}, 0.008, kChrome, 8);
            k.rod(F::Metal, {0, 0.30, 0.13}, {0.0, 0.40, 0.04}, 0.008, kChrome, 8);
            // the shade: a lathe profile runs bottom to top (it was top to bottom -- the shade drew inside out; Glenn:
            // "the desk lamp's shade is inside out"), an inner surface since it is open underneath, a lit bulb inside
            k.turned(F::Hard, {0, 0.34, 0.04}, {{0.07, 0.0}, {0.06, 0.04}, {0.02, 0.12}, {0.0, 0.12}}, shade, 1.0, 16);
            k.turned(F::Hard, {0, 0.34, 0.04}, {{0.0, 0.115}, {0.017, 0.115}, {0.056, 0.038}, {0.066, 0.0}}, shade * 0.6, 1.0, 16);
            k.turned(F::Light, {0, 0.36, 0.04}, {{0.0, 0.0}, {0.02, 0.01}, {0.022, 0.03}, {0.01, 0.05}, {0.0, 0.055}},
                     Vec3(1.0, 0.92, 0.75), 1.0, 10);
            break;
        }
        case Piece::PottedPlant: {
            // A HOUSEPLANT, GROWN (Glenn, 2026-10-03: "LOL is that a cactus on the teacher's desk. We should use the
            // lsystem to build that."): a pot, and a plant from the engine's L-system -- the turtle's stems, a leaf
            // card at every apex turned along its heading. The style bits pick the plant: a leafy bush (a ficus), an
            // arching fern, a rosette of long upright leaves (a snake plant); the variant seeds its growth.
            k.out.size = {0.30, 0.55, 0.30};
            k.out.solid = false;
            const Vec3 pot = (variant & 32u) ? kWhite : Vec3(0.66, 0.36, 0.24);
            k.turned(F::Ceramic, {0, 0, 0.15}, {{0.0, 0.0}, {0.07, 0.0}, {0.095, 0.15}, {0.10, 0.16}, {0.0, 0.16}}, pot, 1.0, 16);
            k.turned(F::Fabric, {0, 0.145, 0.15}, {{0.0, 0.0}, {0.092, 0.0}, {0.0, 0.012}}, Vec3(0.22, 0.15, 0.10), 1.0, 14);
            const int species = static_cast<int>((style >> 1) % 4u);
            LSystem ls;
            TurtleParams tp;
            std::string axiom;
            int iters = 3;
            double leafL = 0.07, leafW = 0.035;
            if (species == 0) {          // bush: branching stems, leaves all over
                ls.rule('X', "F[+XL][-XL]&[XL]/F[^XL]", 1.0);
                ls.rule('X', "F[&XL][/+XL]F[-XL]", 1.0);
                axiom = "FX"; tp.length = 0.045f; tp.radius = 0.006f; tp.angleDeg = 32.0f; iters = 3;
            } else if (species == 1) {   // fern: fronds arching out, leaflets along them
                ls.rule('A', "F[+L][-L]^A", 1.0);
                axiom = "[&&A]/(72)[&&A]/(72)[&&A]/(72)[&&A]/(72)[&&A]";
                tp.length = 0.035f; tp.radius = 0.003f; tp.angleDeg = 22.0f; iters = 6; leafL = 0.04; leafW = 0.016;
            } else if (species == 3) {   // a CACTUS (Glenn: "that's not what a cactus looks like"): a ribbed column,
                                         // arms that turn out and then up, no leaves -- spines and a flower below
                axiom = (variant & 64u) ? "FFF[&F^^FF]F/[&F^^F]F" : "FF[&F^^FFF]FF";
                tp.length = 0.06f; tp.radius = 0.034f; tp.angleDeg = 45.0f; iters = 0;
            } else {                     // rosette: long upright blades from the soil
                axiom = "[&L]/[&L]/[&L]/[&L]/[&L]/[&L]/[L]";
                tp.length = 0.03f; tp.radius = 0.004f; tp.angleDeg = 14.0f; iters = 0; leafL = 0.30; leafW = 0.05;
            }
            tp.radiusTaper = species == 3 ? 0.72f : 0.8f;
            tp.segmentSlices = species == 3 ? 12 : 5;
            std::string sym = iters > 0 ? ls.expand(axiom, iters, variant * 2654435761u + 1u) : axiom;
            for (char& ch : sym) if (ch == 'X' || ch == 'A') ch = 'L';
            for (std::size_t i = 0; i < sym.size(); ++i)   // the plain turtle has no "/(72)": read "/(n)" as one roll
                if (sym[i] == '(') { const std::size_t e = sym.find(')', i); if (e != std::string::npos) sym.erase(i, e - i + 1); }
            const Vec3 soil(0, 0.155, 0.15);
            const Vec3 stemCol(0.25, 0.35, 0.15), leafCol = (species == 2) ? Vec3(0.20, 0.38, 0.16) : Vec3(0.16, 0.40, 0.14);
            RenderMesh stems = buildTurtleMesh(sym, tp);
            const Vec3 cactusCol(0.22, 0.42, 0.24);
            for (Vertex& v : stems.vertices) { v.position = v.position + soil; v.color = species == 3 ? cactusCol : stemCol; }
            MeshBuilder::append(k.m(F::Fabric), stems);
            if (species == 3) {
                // RIBS: a cactus is fluted -- push each vertex out by its angle round its own stem segment (12 slices:
                // every other one out) -- then spines down the ribs and a pink flower on the crown
                RenderMesh& body = k.m(F::Fabric);
                const std::size_t from = body.vertices.size() - stems.vertices.size();
                for (std::size_t i = from; i < body.vertices.size(); ++i)
                    if (((i - from) % 2) == 0) body.vertices[i].position = body.vertices[i].position + body.vertices[i].normal * 0.006;
                uint32_t rr = variant * 2246822519u + 9u;
                auto nx = [&]() { rr ^= rr << 13; rr ^= rr >> 17; rr ^= rr << 5; return (rr & 0xffffu) / 65535.0; };
                for (std::size_t i = from; i < body.vertices.size(); i += 3) {
                    if (nx() > 0.35) continue;
                    const Vertex& v = body.vertices[i];
                    k.rod(F::Hard, v.position, v.position + v.normal * 0.012, 0.0012, Vec3(0.85, 0.82, 0.70), 3);
                }
                Vec3 crown(0, 0, 0);
                for (std::size_t i = from; i < body.vertices.size(); ++i)
                    if (body.vertices[i].position.y > crown.y) crown = body.vertices[i].position;
                if (variant & 128u)
                    k.turned(F::Fabric, crown + Vec3(0, -0.005, 0), {{0.0, 0.0}, {0.018, 0.012}, {0.012, 0.022}, {0.0, 0.016}},
                             Vec3(0.90, 0.35, 0.55), 1.0, 8);
            }
            uint32_t r = variant * 747796405u + 3u;
            auto next = [&]() { r ^= r << 13; r ^= r >> 17; r ^= r << 5; return (r & 0xffffu) / 65535.0; };
            for (const LeafPlacement& lf : species == 3 ? std::vector<LeafPlacement>{} : turtleLeaves(sym, tp)) {
                // a leaf card: long along the heading, flat across it, a little colour each
                Vec3 d = normalize(lf.direction);
                if (species == 2) d = normalize(d + Vec3(0, 1.4, 0));
                const Vec3 ref = std::fabs(d.y) > 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
                const Vec3 xA = normalize(cross(ref, d)), zA = cross(xA, d);
                RenderMesh leaf = MeshBuilder::roundedBox({leafW, leafL, 0.004}, 0.0, 1, 2.0);
                const Vec3 c = leafCol * (0.85 + 0.3 * next());
                for (Vertex& v : leaf.vertices) {
                    const Vec3 q = v.position + Vec3(0, leafL * 0.5, 0);
                    v.position = lf.position + soil + xA * q.x + d * q.y + zA * q.z;
                    const Vec3 n = v.normal;
                    v.normal = normalize(xA * n.x + d * n.y + zA * n.z);
                    v.color = c;
                }
                MeshBuilder::append(k.m(F::Fabric), leaf);
            }
            break;
        }
        case Piece::BookStack: {
            // three or four books lying flat, each a little askew
            k.out.size = {0.30, 0.16, 0.24};
            k.out.solid = false;
            static const Vec3 kCovers[6] = {{0.55, 0.12, 0.10}, {0.12, 0.20, 0.45}, {0.15, 0.40, 0.22}, {0.85, 0.80, 0.70},
                                            {0.10, 0.10, 0.10}, {0.75, 0.55, 0.20}};
            const int n = 3 + static_cast<int>(variant % 2u);
            double y = 0;
            for (int i = 0; i < n; ++i) {
                const double h = 0.028 + 0.012 * ((variant + i) % 3), w = 0.24 - 0.02 * (i % 2), d = 0.17 + 0.015 * (i % 3);
                k.boxTilted(F::Hard, {0.01 * (static_cast<int>((i * 3 + variant) % 5u) - 2), y + h * 0.5, 0.12}, {w, h, d}, 0.003, 0.0,
                            kCovers[(variant / 2 + i) % 6]);
                y += h;
            }
            break;
        }
        case Piece::Kettle: {
            k.out.size = {0.24, 0.26, 0.20};
            k.out.solid = false;
            const Vec3 body = (variant & 32u) ? kWhite : kSteel;
            k.turned(F::Metal, {0, 0, 0.10}, {{0.0, 0.0}, {0.085, 0.0}, {0.09, 0.03}, {0.08, 0.18}, {0.05, 0.21}, {0.0, 0.215}},
                     body, 1.0, 20);
            k.box(F::Hard, {0.10, 0.12, 0.10}, {0.025, 0.16, 0.03}, 0.01, kBlack, 1);   // the handle
            k.rod(F::Metal, {-0.07, 0.10, 0.10}, {-0.12, 0.17, 0.10}, 0.012, body, 8); // the spout
            break;
        }
        case Piece::FruitBowl: {
            k.out.size = {0.30, 0.14, 0.30};
            k.out.solid = false;
            k.turned(F::Ceramic, {0, 0, 0.15}, {{0.0, 0.0}, {0.06, 0.0}, {0.14, 0.07}, {0.145, 0.075}, {0.0, 0.03}}, kPorcelain, 1.0, 20);
            static const Vec3 kFruit[4] = {{0.85, 0.15, 0.10}, {0.95, 0.70, 0.10}, {0.40, 0.65, 0.15}, {0.90, 0.45, 0.10}};
            for (int i = 0; i < 5; ++i) {
                const double a = i * 1.257, r = i == 4 ? 0.0 : 0.06;
                k.turned(F::Hard, {r * std::cos(a), 0.05 + (i == 4 ? 0.04 : 0.0), 0.15 + r * std::sin(a)},
                         {{0.0, 0.0}, {0.03, 0.01}, {0.038, 0.035}, {0.03, 0.06}, {0.0, 0.07}}, kFruit[(variant + i) % 4], 1.0, 10);
            }
            break;
        }
        case Piece::Vase: {
            k.out.size = {0.16, 0.48, 0.16};
            k.out.solid = false;
            static const Vec3 kGlaze[3] = {{0.20, 0.35, 0.55}, {0.85, 0.82, 0.75}, {0.55, 0.25, 0.20}};
            k.turned(F::Ceramic, {0, 0, 0.08}, {{0.0, 0.0}, {0.05, 0.0}, {0.075, 0.10}, {0.05, 0.22}, {0.035, 0.26}, {0.045, 0.28}, {0.0, 0.28}},
                     kGlaze[variant % 3], 1.0, 18);
            for (int i = 0; i < 5; ++i)   // stems
                k.rod(F::Wood, {0.0, 0.26, 0.08}, {0.04 * std::cos(i * 1.3), 0.42 + 0.04 * (i % 2), 0.08 + 0.04 * std::sin(i * 1.3)}, 0.004,
                      Vec3(0.25, 0.40, 0.18), 6);
            break;
        }
        case Piece::Mug: {
            k.out.size = {0.12, 0.10, 0.09};
            k.out.solid = false;
            static const Vec3 kMug[4] = {{0.95, 0.95, 0.93}, {0.20, 0.30, 0.55}, {0.75, 0.20, 0.18}, {0.20, 0.20, 0.22}};
            k.turned(F::Ceramic, {0, 0, 0.045}, {{0.0, 0.0}, {0.04, 0.0}, {0.042, 0.095}, {0.0, 0.095}}, kMug[variant % 4], 1.0, 16);
            k.box(F::Ceramic, {0.05, 0.05, 0.045}, {0.015, 0.06, 0.012}, 0.005, kMug[variant % 4], 1);
            break;
        }
        // THE UNIVERSITY CAMPUS: a dorm's single bed, a classroom's desk and stacking chair, a lecture hall's row of
        // tip-up seats on its own riser, the lectern, a lab's bench, the library's long reading table.
        case Piece::SingleBed: {
            // A made single, 0.95 x 2.05: the frame on legs, a low timber headboard, the duvet, one pillow.
            k.out.size = {0.95, 0.85, 2.05};
            k.box(F::Wood, {0, 0.22, 1.02}, {0.95, 0.12, 2.03}, 0.01, wood, 1);
            for (double x : {-0.42, 0.42})
                for (double z : {0.06, 1.98}) k.leg(F::Wood, {x, 0, z}, 0.16, 0.05, wood * 0.9);
            k.box(F::Wood, {0, 0.52, 0.03}, {0.95, 0.66, 0.05}, 0.01, wood * 0.95, 1);
            k.box(F::Fabric, {0, 0.38, 1.04}, {0.88, 0.20, 1.94}, 0.05, kLinen);
            k.box(F::Fabric, {0, 0.51, 1.30}, {0.98, 0.06, 1.46}, 0.03, fabric);
            for (double x : {-0.49, 0.49}) k.box(F::Fabric, {x, 0.40, 1.30}, {0.03, 0.26, 1.46}, 0.012, fabric * 0.94);
            k.box(F::Fabric, {0, 0.55, 0.30}, {0.66, 0.13, 0.36}, 0.05, kLinen * 1.02);
            break;
        }
        case Piece::SchoolDesk: {
            // A pupil's desk: a laminate top on a tubular steel frame, a book shelf under it.
            k.out.size = {0.70, 0.74, 0.50};
            // (Glenn: "the parts of the students desks don't seem connected together": the legs stopped short of
            // the top and the shelf hung in the air -- a welded frame now: legs into an apron, the shelf on rails)
            const Vec3 steel = kSteel * 0.6;
            k.box(F::Hard, {0, 0.725, 0.25}, {0.70, 0.025, 0.50}, 0.006, wood * 1.1, 1);
            for (double x : {-0.32, 0.32})
                for (double z : {0.04, 0.46}) k.rod(F::Metal, {x, 0.0, z}, {x, 0.715, z}, 0.013, steel, 8);
            for (double z : {0.04, 0.46}) k.rod(F::Metal, {-0.32, 0.70, z}, {0.32, 0.70, z}, 0.012, steel, 8);   // apron
            for (double x : {-0.32, 0.32}) {
                k.rod(F::Metal, {x, 0.70, 0.04}, {x, 0.70, 0.46}, 0.012, steel, 8);
                k.rod(F::Metal, {x, 0.55, 0.04}, {x, 0.55, 0.46}, 0.010, steel, 8);   // the shelf's rails
            }
            k.box(F::Metal, {0, 0.56, 0.25}, {0.64, 0.012, 0.42}, 0.0, steel, 1);
            break;
        }
        case Piece::SchoolChair: {
            // A stacking chair: a moulded shell on four splayed steel legs.
            k.out.size = {0.46, 0.80, 0.48};
            k.out.solid = false;
            const Vec3 shell = kFabric[(variant + 3) & 7u] * 0.9;
            k.box(F::Hard, {0, 0.45, 0.26}, {0.42, 0.03, 0.40}, 0.015, shell);
            k.boxTilted(F::Hard, {0, 0.66, 0.05}, {0.40, 0.34, 0.025}, 0.012, -0.12, shell);
            for (double x : {-0.18, 0.18}) {
                k.rod(F::Metal, {x * 1.1, 0.0, 0.44}, {x, 0.44, 0.40}, 0.011, kSteel * 0.7, 8);
                k.rod(F::Metal, {x * 1.1, 0.0, 0.06}, {x, 0.44, 0.10}, 0.011, kSteel * 0.7, 8);
                k.rod(F::Metal, {x, 0.44, 0.10}, {x, 0.62, 0.06}, 0.011, kSteel * 0.7, 8);
            }
            break;
        }
        case Piece::LectureRow: {
            // A LECTURE HALL's row: six tip-up seats, the writing ledge for the row behind fixed to their backs, on
            // its own carpeted RISER: style bits = the row's TIER, 0.18 m a tier -- a hall's rows step up toward
            // the back, each standing on the flat floor as a solid block (the room program's grid numbers them).
            const double h = 0.18 * static_cast<double>(style & 7u);
            k.out.size = {3.4, 1.1 + h, 1.0};
            // the collider is the RISER, not the seats' box: you step onto your row from the aisle stair beside it
            // (a full-height box made every row a wall)
            k.out.solid = h > 0;
            k.out.colliderH = h;
            const Vec3 carpet = kFabric[(variant + 5) & 7u] * 0.55;
            if (h > 0) k.box(F::Fabric, {0, h * 0.5, 0.50}, {3.4, h, 1.0}, 0.0, carpet, 1);
            // (a seat faces +z like every seat in the kit -- its back toward the riser's rear edge; Glenn: "those
            // seats are facing the wrong way")
            for (int i = 0; i < 6; ++i) {
                const double x = -1.4 + i * 0.56;
                k.box(F::Fabric, {x, h + 0.44, 0.40}, {0.50, 0.08, 0.44}, 0.025, fabric);               // the seat
                k.boxTilted(F::Fabric, {x, h + 0.78, 0.16}, {0.50, 0.56, 0.07}, 0.025, -0.12, fabric);  // its back
                k.box(F::Hard, {x - 0.28, h + 0.35, 0.37}, {0.05, 0.62, 0.55}, 0.01, kBlack * 2.5, 1);   // the standard
            }
            k.box(F::Hard, {1.68, h + 0.35, 0.37}, {0.05, 0.62, 0.55}, 0.01, kBlack * 2.5, 1);
            // the writing ledge for the row behind, on this row's back
            k.box(F::Wood, {0, h + 0.74, 0.06}, {3.36, 0.03, 0.12}, 0.005, wood, 1);
            k.box(F::Hard, {0, h + 0.40, 0.11}, {3.36, 0.70, 0.03}, 0.005, kBlack * 2.5, 1);   // the modesty panel
            break;
        }
        case Piece::AisleStep: {
            // THE AISLE STAIR (Glenn: "there should be stairs that allow you to access the raised auditorium chairs"):
            // beside each row, the aisle climbs its tier in two 9 cm steps -- the lower at the row's front half, the
            // tier's own height at its back half -- so the aisle rises with the rows, a step at a time.
            const double h = 0.18 * static_cast<double>(style & 7u);
            k.out.size = {0.9, std::max(h, 0.02), 1.0};
            k.out.solid = h > 0;
            k.out.colliderH = std::max(h - 0.09, 0.0);   // the walkable truth: the lower tread (the step is 9 cm)
            const Vec3 carpet = kFabric[(variant + 5) & 7u] * 0.55, nose(0.75, 0.70, 0.40);
            if (h > 0) {
                const double lo = std::max(h - 0.09, 0.0);
                if (lo > 0) k.box(F::Fabric, {0, lo * 0.5, 0.75}, {0.9, lo, 0.5}, 0.0, carpet, 1);
                k.box(F::Fabric, {0, h * 0.5, 0.25}, {0.9, h, 0.5}, 0.0, carpet, 1);
                k.box(F::Hard, {0, h + 0.002, 0.49}, {0.9, 0.004, 0.03}, 0.0, nose, 1);    // nosing strips
                if (lo > 0) k.box(F::Hard, {0, lo + 0.002, 0.99}, {0.9, 0.004, 0.03}, 0.0, nose, 1);
            } else {
                k.box(F::Fabric, {0, 0.005, 0.5}, {0.9, 0.01, 1.0}, 0.0, carpet, 1);   // the front row's aisle: a runner
            }
            break;
        }
        case Piece::Bleacher: {
            // A SPORTS FIELD'S STAND: three tiers of aluminium bench planks rising away from the pitch (+z faces it),
            // footboards between, a galvanised frame of uprights and raked stringers, a rail along the back.
            k.out.size = {6.0, 2.1, 2.6};
            k.out.solid = false;
            const Vec3 alu(0.78, 0.80, 0.82), galv(0.55, 0.57, 0.60);
            for (int t = 0; t < 3; ++t) {
                const double seatY = 0.45 + 0.42 * t, z = 2.05 - 0.72 * t;
                k.box(F::Metal, {0, seatY, z}, {6.0, 0.05, 0.28}, 0.008, alu, 1);                        // the seat plank
                k.box(F::Metal, {0, seatY - 0.40, z + 0.30}, {6.0, 0.04, 0.32}, 0.006, alu * 0.92, 1);   // the footboard
            }
            for (double x : {-2.85, -0.95, 0.95, 2.85}) {
                k.rod(F::Metal, {x, 0.0, 2.25}, {x, 0.45, 2.25}, 0.03, galv, 8);    // front leg
                k.rod(F::Metal, {x, 0.0, 0.40}, {x, 1.95, 0.40}, 0.03, galv, 8);    // rear leg
                k.rod(F::Metal, {x, 0.43, 2.25}, {x, 1.30, 0.40}, 0.025, galv, 8);  // raked stringer
            }
            k.rod(F::Metal, {-3.0, 1.95, 0.40}, {3.0, 1.95, 0.40}, 0.025, galv, 8);   // the back rail
            break;
        }
        case Piece::Lectern: {
            k.out.size = {0.70, 1.15, 0.55};
            k.box(F::Wood, {0, 0.50, 0.30}, {0.60, 1.0, 0.45}, 0.01, wood, 1);
            k.boxTilted(F::Wood, {0, 1.09, 0.28}, {0.70, 0.04, 0.52}, 0.008, -0.25, wood * 1.05);
            k.box(F::Wood, {0, 0.03, 0.30}, {0.66, 0.06, 0.50}, 0.004, wood * 0.75, 1);
            break;
        }
        case Piece::LabBench: {
            // A LAB BENCH, 2.4 x 0.75: a black resin top over white cupboards, a sink at one end with its gooseneck
            // tap, a service spine of sockets and gas taps along the back.
            k.out.size = {2.4, 1.15, 0.75};
            k.box(F::Hard, {0, 0.44, 0.38}, {2.36, 0.80, 0.70}, 0.006, kWhite, 1);
            for (int d = 0; d < 4; ++d) {
                const double x0 = -1.16 + d * 0.58;
                k.front(F::Hard, x0, x0 + 0.58, 0.08, 0.80, 0.73, kWhite * 0.97, kChrome, true);
            }
            k.box(F::Hard, {0, 0.865, 0.375}, {2.4, 0.03, 0.75}, 0.004, Vec3(0.08, 0.08, 0.09), 1);
            k.box(F::Ceramic, {0.85, 0.86, 0.40}, {0.40, 0.03, 0.34}, 0.01, Vec3(0.12, 0.12, 0.13), 1);   // the sink
            k.rod(F::Metal, {0.85, 0.88, 0.12}, {0.85, 1.12, 0.12}, 0.012, kChrome, 8);
            k.rod(F::Metal, {0.85, 1.12, 0.12}, {0.85, 1.10, 0.30}, 0.012, kChrome, 8);
            k.box(F::Hard, {-0.2, 1.0, 0.06}, {1.8, 0.24, 0.10}, 0.01, kSteel * 0.85, 1);                  // the spine
            for (int i = 0; i < 5; ++i) k.box(F::Hard, {-1.0 + i * 0.4, 1.0, 0.115}, {0.08, 0.06, 0.01}, 0.0, kWhite, 1);
            break;
        }
        case Piece::ReadingTable: {
            // The LIBRARY's long table for eight: an oak top, a green-shaded lamp at each end, four chairs a side
            // (part of the piece: a reading room is tables in rows).
            k.out.size = {2.6, 1.16, 2.0};
            k.box(F::Wood, {0, 0.74, 1.0}, {2.4, 0.045, 0.95}, 0.01, wood, 1);
            for (double x : {-1.1, 1.1})
                for (double z : {0.6, 1.4}) k.leg(F::Wood, {x, 0, z}, 0.72, 0.07, wood * 0.9);
            // the BANKER'S LAMPS (Glenn: "the green part is a shell and there's a lightbulb or two inside"): an open
            // half-tube of green glass on a brass stem, two lit bulbs under it
            for (double x : {-0.95, 0.95}) {
                const Vec3 brass(0.55, 0.45, 0.25), green(0.10, 0.34, 0.18);
                k.turned(F::Metal, {x, 0.765, 1.0}, {{0.07, 0.0}, {0.07, 0.015}, {0.012, 0.03}, {0.012, 0.30}, {0.0, 0.31}},
                         brass, 1.0, 12);
                k.rod(F::Metal, {x, 1.07, 1.0}, {x, 1.13, 1.0}, 0.008, brass, 6);
                for (int seg = 0; seg < 7; ++seg) {   // the shade: a half tube along x, open underneath
                    const double a0 = kPi * seg / 7.0, a1 = kPi * (seg + 1) / 7.0, am = (a0 + a1) * 0.5;
                    const double r = 0.075, cw = r * (a1 - a0) + 0.004;
                    k.boxTilted(F::Ceramic, {x, 1.10 + r * std::sin(am), 1.0 + r * std::cos(am)}, {0.34, 0.008, cw}, 0.0,
                                kPi * 0.5 - am, green);
                }
                for (double bx : {-0.07, 0.07})
                    k.turned(F::Light, {x + bx, 1.085, 1.0}, {{0.0, 0.0}, {0.022, 0.012}, {0.024, 0.03}, {0.012, 0.05}, {0.0, 0.055}},
                             Vec3(1.0, 0.92, 0.75), 1.0, 10);
            }
            for (double x : {-0.9, -0.3, 0.3, 0.9})
                for (int side = 0; side < 2; ++side) {
                    const double s2 = side == 0 ? -1.0 : 1.0, zc = 1.0 + s2 * 0.72;
                    // (Glenn: "the wooden chairs have a floating head rest": the back stood on nothing -- the rear
                    // legs now run up past the seat and carry it)
                    k.box(F::Wood, {x, 0.46, zc}, {0.44, 0.05, 0.42}, 0.012, wood * 0.95, 1);
                    for (double lx : {-0.18, 0.18}) {
                        k.leg(F::Wood, {x + lx, 0, zc - s2 * 0.17}, 0.44, 0.035, wood * 0.9);   // front legs
                        k.leg(F::Wood, {x + lx, 0, zc + s2 * 0.19}, 0.95, 0.035, wood * 0.9);   // rear legs, up the back
                    }
                    k.box(F::Wood, {x, 0.86, zc + s2 * 0.19}, {0.40, 0.16, 0.03}, 0.008, wood * 0.95, 1);   // top rail
                    k.box(F::Wood, {x, 0.64, zc + s2 * 0.19}, {0.36, 0.05, 0.025}, 0.006, wood * 0.95, 1);  // mid rail
                }
            break;
        }
        case Piece::CeilingLight: {
            // THE ROOM'S LIGHT (Glenn, 2026-10-02: "actual ceiling light fixtures for where the ambient light comes
            // from"): hung by its TOP at y = size.y (placed so that is the ceiling). An office's recessed 600 x 1200
            // panel -- a white frame round a lit diffuser -- or, with style bit 5, a home's round flush fitting.
            k.out.solid = false;
            if (variant & 32u) {
                k.out.size = {0.42, 0.08, 0.42};
                k.turned(F::Hard, {0, 0.06, 0.21}, {{0.0, 0.0}, {0.20, 0.0}, {0.21, 0.02}, {0.0, 0.02}}, kWhite, 1.0, 24);
                k.turned(F::Light, {0, 0.0, 0.21}, {{0.0, 0.0}, {0.18, 0.0}, {0.19, 0.03}, {0.17, 0.06}, {0.0, 0.06}},
                         Vec3(1.0, 0.95, 0.85), 1.0, 24);
            } else {
                k.out.size = {0.62, 0.05, 1.22};
                k.box(F::Hard, {0, 0.035, 0.61}, {0.62, 0.03, 1.22}, 0.0, kWhite, 1);   // the frame
                k.box(F::Light, {0, 0.012, 0.61}, {0.56, 0.02, 1.16}, 0.0, Vec3(1.0, 0.98, 0.94), 1);   // the diffuser
            }
            break;
        }
        case Piece::Rug: {
            k.out.size = {2.0, 0.04, 1.4};
            k.out.solid = false;
            k.box(F::Fabric, {0, 0.026, 0.7}, {2.0, 0.012, 1.4}, 0.005, fabric * 0.85, 1);   // lifted clear of the floor finish
            k.box(F::Fabric, {0, 0.028, 0.7}, {1.76, 0.012, 1.16}, 0.004, kLinen * 0.9, 1);
            k.box(F::Fabric, {0, 0.030, 0.7}, {1.5, 0.012, 0.9}, 0.004, fabric, 1);
            break;
        }
        default: break;
    }
    return k.out;
}

const char* const kPieceNames[kPieceCount] = {
    "desk", "office_chair", "monitor", "filing_cabinet", "bed", "nightstand", "wardrobe", "sofa", "coffee_table",
    "tv_unit", "kitchen_base", "kitchen_sink", "kitchen_hob", "kitchen_tall", "kitchen_wall", "dining_table",
    "dining_chair", "bathtub", "toilet", "vanity", "lounge_chair", "planter", "picture", "shelving", "rug",
    "desk_pod", "cubicle", "meeting_table", "whiteboard",
    "shop_counter", "gondola", "wall_shelf", "clothes_rack", "cafe_table", "display_case", "drinks_fridge", "bookcase",
    "pallet_rack", "checkout", "bench", "plaza_bench", "bistro_table", "bistro_chair", "ceiling_light",
    "desk_lamp", "potted_plant", "book_stack", "kettle", "fruit_bowl", "vase", "mug",
    "single_bed", "school_desk", "school_chair", "lecture_row", "lectern", "lab_bench", "reading_table", "aisle_step", "bleacher"};

}  // namespace

const FurniturePiece& furniturePiece(Piece p, uint32_t variant) {
    static std::mutex mu;
    static std::map<uint64_t, std::unique_ptr<FurniturePiece>> cache;
    const uint64_t key = (static_cast<uint64_t>(p) << 32) | variant;
    std::lock_guard<std::mutex> lock(mu);
    auto it = cache.find(key);
    if (it == cache.end()) it = cache.emplace(key, std::make_unique<FurniturePiece>(build(p, variant))).first;
    return *it->second;
}

const char* furniturePieceName(Piece p) {
    const int i = static_cast<int>(p);
    return i >= 0 && i < kPieceCount ? kPieceNames[i] : "?";
}

bool furniturePieceByName(const std::string& name, Piece& out) {
    for (int i = 0; i < kPieceCount; ++i)
        if (name == kPieceNames[i]) { out = static_cast<Piece>(i); return true; }
    return false;
}

}  // namespace engine
