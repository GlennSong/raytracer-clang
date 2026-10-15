#include "cells.h"
#include <algorithm>
#include <cmath>

namespace engine {

Slice Complex::slice(Real y) const {
    Slice s;
    for (const CellFrame& c : cells) {
        if (!c.mass) continue;
        const Real local = y - c.baseY;
        if (local >= 0 && local >= c.mass->height()) continue;   // this cell stops below y
        const Slice cs = c.mass->slice(local);
        if (cs.empty()) continue;
        s.pieces.push_back(cs.outline());
    }
    return s;
}

Real Complex::height() const {
    Real h = 0;
    for (const CellFrame& c : cells)
        if (c.mass) h = std::max(h, c.baseY + c.mass->height());
    return h;
}

MassBounds Complex::bounds() const {
    MassBounds b;
    bool first = true;
    for (const CellFrame& c : cells) {
        if (!c.mass) continue;
        const MassBounds cb = c.mass->bounds();
        if (first) { b = cb; b.y0 += c.baseY; b.y1 += c.baseY; first = false; continue; }
        b.lo.x = std::min(b.lo.x, cb.lo.x); b.lo.y = std::min(b.lo.y, cb.lo.y);
        b.hi.x = std::max(b.hi.x, cb.hi.x); b.hi.y = std::max(b.hi.y, cb.hi.y);
        b.y0 = std::min(b.y0, cb.y0 + c.baseY);
        b.y1 = std::max(b.y1, cb.y1 + c.baseY);
    }
    return b;
}

RenderMesh Complex::shell(int lod, const Vec3& colour) const {
    RenderMesh m;
    for (const CellFrame& c : cells) {
        if (!c.mass) continue;
        RenderMesh cm = c.mass->shell(lod, colour);
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (Vertex v : cm.vertices) { v.position.y = static_cast<decltype(v.position.y)>(v.position.y + c.baseY); m.vertices.push_back(v); }
        for (uint32_t i : cm.indices) m.indices.push_back(base + i);
    }
    return m;
}

void Complex::resolve(Real tol, Real minRun) {
    adj.clear();
    for (CellFrame& c : cells) {
        c.params.partyWalls = 0;
        c.params.partyDoors = 0;
        for (int k = 0; k < 4; ++k) { c.params.partyN[k] = Vec2(0, 0); c.params.partyAt[k] = 0; c.params.partyTop[k] = 0; }
        c.outerFace = false;
        ensureCCW(c.plan);
    }
    auto party = [&](CellFrame& c, const Vec2& n, Real at, int top) {
        for (int k = 0; k < c.params.partyWalls; ++k)   // the same plane twice (two neighbours along one face): one wall,
            if (dot(c.params.partyN[k], n) > 0.999 && std::fabs(c.params.partyAt[k] - at) < tol) {   // blank to the taller's top
                c.params.partyTop[k] = (top == 0 || c.params.partyTop[k] == 0) ? int16_t(0) : static_cast<int16_t>(std::max<int>(c.params.partyTop[k], top));
                return;
            }
        if (c.params.partyWalls >= 4) return;
        const int k = c.params.partyWalls++;
        c.params.partyN[k] = n;
        c.params.partyAt[k] = at;
        c.params.partyTop[k] = static_cast<int16_t>(top);
    };
    // per cell, per edge: how much of it a neighbour covers (for the outer faces)
    std::vector<std::vector<Real>> covered(cells.size());
    for (std::size_t i = 0; i < cells.size(); ++i) covered[i].assign(cells[i].plan.size(), 0.0);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const Poly2& A = cells[i].plan;
        for (std::size_t j = i + 1; j < cells.size(); ++j) {
            const Poly2& B = cells[j].plan;
            for (std::size_t ea = 0; ea < A.size(); ++ea) {
                const Vec2 a0 = A[ea], a1 = A[(ea + 1) % A.size()];
                const Real la = (a1 - a0).length();
                if (la < 1e-6) continue;
                const Vec2 da = (a1 - a0) * (1.0 / la);
                const Vec2 na(da.y, -da.x);   // outward of a CCW plan
                for (std::size_t eb = 0; eb < B.size(); ++eb) {
                    const Vec2 b0 = B[eb], b1 = B[(eb + 1) % B.size()];
                    const Real lb = (b1 - b0).length();
                    if (lb < 1e-6) continue;
                    const Vec2 db = (b1 - b0) * (1.0 / lb);
                    if (dot(da, db) > -0.98) continue;                                   // facing each other
                    if (std::fabs(dot(b0 - a0, na)) > tol || std::fabs(dot(b1 - a0, na)) > tol) continue;   // on one line
                    const Real t0 = std::max(Real(0), std::min(dot(b0 - a0, da), dot(b1 - a0, da)));
                    const Real t1 = std::min(la, std::max(dot(b0 - a0, da), dot(b1 - a0, da)));
                    if (t1 - t0 < minRun) continue;
                    Adjacency ad;
                    ad.a = static_cast<int>(i);
                    ad.b = static_cast<int>(j);
                    ad.n = na;
                    ad.at = dot(na, a0);
                    ad.p0 = a0 + da * t0;
                    ad.p1 = a0 + da * t1;
                    ad.k0 = 0;
                    ad.k1 = std::min(cells[i].params.floors, cells[j].params.floors);
                    adj.push_back(ad);
                    covered[i][ea] += t1 - t0;
                    // b's edge: the same run measured along db
                    covered[j][eb] += t1 - t0;
                    const int fi = cells[i].params.floors, fj = cells[j].params.floors;
                    party(cells[i], na, ad.at, fj < fi ? fj : 0);
                    party(cells[j], na * -1.0, -ad.at, fi < fj ? fi : 0);
                }
            }
        }
    }
    for (std::size_t i = 0; i < cells.size(); ++i)
        for (std::size_t e = 0; e < cells[i].plan.size(); ++e) {
            const Real L = (cells[i].plan[(e + 1) % cells[i].plan.size()] - cells[i].plan[e]).length();
            if (L - covered[i][e] >= 3.0) cells[i].outerFace = true;
        }
}

void Complex::connectLobbies(const Vec2& faceDir) {
    const std::size_t n = cells.size();
    // the complex's own boundary: the box round every cell; a face on it is a street face
    Poly2 all;
    for (const CellFrame& c : cells) all.insert(all.end(), c.plan.begin(), c.plan.end());
    const OBB2 hull = all.size() >= 3 ? orientedBoundingBox(all) : OBB2{};
    auto onBoundary = [&](const Vec2& m) {
        const Vec2 r = m - hull.center;
        const Real x = std::fabs(dot(r, hull.axis[0])), y = std::fabs(dot(r, hull.axis[1]));
        return std::fabs(x - hull.half[0]) < 0.3 || std::fabs(y - hull.half[1]) < 0.3;
    };
    for (std::size_t i = 0; i < n; ++i) {
        CellFrame& c = cells[i];
        c.streetFace = false;
        // the entrance on the street face nearest the street: faceDir turned to that face's normal
        Real best = -2;
        Vec2 bestN(faceDir.x, faceDir.y);
        for (std::size_t e = 0; e < c.plan.size() && c.outerFace; ++e) {
            const Vec2 a = c.plan[e], b = c.plan[(e + 1) % c.plan.size()];
            const Real L = (b - a).length();
            if (L < 3.0) continue;
            const Vec2 d = (b - a) * (1.0 / L);
            const Vec2 nrm(d.y, -d.x);
            if (!onBoundary((a + b) * 0.5)) continue;   // a court face: outer, but it opens onto nothing
            Real cov = 0;   // covered by a neighbour?
            for (const Adjacency& ad : adj) {
                const bool mine = (ad.a == static_cast<int>(i) && dot(ad.n, nrm) > 0.98) || (ad.b == static_cast<int>(i) && dot(ad.n, nrm) < -0.98);
                if (!mine || std::fabs(dot(nrm, a) - (ad.a == static_cast<int>(i) ? ad.at : -ad.at)) > 0.3) continue;
                cov += (ad.p1 - ad.p0).length();
            }
            if (L - cov < 3.0) continue;
            c.streetFace = true;
            const Real score = dot(nrm, faceDir);
            if (score > best) { best = score; bestN = nrm; }
        }
        c.connected = c.streetFace;
        if (c.streetFace) c.params.faceDir = Vec3(bestN.x, 0, bestN.y);
    }
    // the rest, through a connected neighbour, nearest the street first
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t i = 0; i < n; ++i) {
            CellFrame& c = cells[i];
            if (c.connected) continue;
            int bestAdj = -1;
            Real best = -2;
            for (std::size_t k = 0; k < adj.size(); ++k) {
                const Adjacency& ad = adj[k];
                if (ad.a != static_cast<int>(i) && ad.b != static_cast<int>(i)) continue;
                const int other = ad.a == static_cast<int>(i) ? ad.b : ad.a;
                if (!cells[static_cast<std::size_t>(other)].connected) continue;
                const Vec2 toward = ad.a == static_cast<int>(i) ? ad.n : ad.n * -1.0;   // out of this cell
                const Real score = dot(toward, faceDir) + 0.01 * (ad.p1 - ad.p0).length();
                if (score > best) { best = score; bestAdj = static_cast<int>(k); }
            }
            if (bestAdj < 0) continue;
            const Adjacency& ad = adj[static_cast<std::size_t>(bestAdj)];
            const bool isA = ad.a == static_cast<int>(i);
            CellFrame& nb = cells[static_cast<std::size_t>(isA ? ad.b : ad.a)];
            const Vec2 toward = isA ? ad.n : ad.n * -1.0;
            c.params.faceDir = Vec3(toward.x, 0, toward.y);   // its entrance on the shared wall
            // the neighbour's side: the doorway through its party wall on that plane
            const Vec2 nn = toward * -1.0;
            const Real nat = dot(nn, ad.p0);
            for (int k = 0; k < nb.params.partyWalls; ++k)
                if (dot(nb.params.partyN[k], nn) > 0.999 && std::fabs(nb.params.partyAt[k] - nat) < 0.3)
                    nb.params.partyDoors = static_cast<uint8_t>(nb.params.partyDoors | (1u << k));
            Adjacency& wr = adj[static_cast<std::size_t>(bestAdj)];
            const Real run = (wr.p1 - wr.p0).length();
            wr.openings.push_back({run * 0.5 - 0.6, run * 0.5 + 0.6, 0, 0});
            c.connected = true;
            changed = true;
        }
    }
}

Complex bundledTubes(const Poly2& boxIn, const BuildingParams& params, int nx, int ny, const std::vector<int>& floorsPerCell) {
    Complex cx;
    Poly2 box = boxIn;
    if (box.size() != 4 || nx < 1 || ny < 1) return cx;
    ensureCCW(box);
    const Vec2 o = box[0];
    const Vec2 ex = box[1] - box[0], ey = box[3] - box[0];
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            CellFrame c;
            c.plan = {o + ex * (static_cast<Real>(i) / nx) + ey * (static_cast<Real>(j) / ny),
                      o + ex * (static_cast<Real>(i + 1) / nx) + ey * (static_cast<Real>(j) / ny),
                      o + ex * (static_cast<Real>(i + 1) / nx) + ey * (static_cast<Real>(j + 1) / ny),
                      o + ex * (static_cast<Real>(i) / nx) + ey * (static_cast<Real>(j + 1) / ny)};
            ensureCCW(c.plan);
            c.params = params;
            const std::size_t idx = static_cast<std::size_t>(j * nx + i);
            if (!floorsPerCell.empty()) c.params.floors = std::max(1, floorsPerCell[std::min(idx, floorsPerCell.size() - 1)]);
            c.params.partyWalls = 0;
            c.params.partyDoors = 0;
            c.frame.origin = c.plan[0];
            c.frame.u = normalize(ex);
            c.frame.v = normalize(ey);
            c.role = "tube";
            c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
            cx.cells.push_back(c);
        }
    cx.resolve();
    cx.connectLobbies(Vec2(params.faceDir.x, params.faceDir.z));
    return cx;
}

Complex courtyardWings(const Poly2& boxIn, const BuildingParams& params, Real depth, const std::vector<int>& floorsPerWing, Real minCourt) {
    Complex cx;
    Poly2 box = boxIn;
    if (box.size() != 4 || depth < 4.0) return cx;
    ensureCCW(box);
    const Vec2 o = box[0];
    const Vec2 ex = box[1] - box[0], ey = box[3] - box[0];
    const Real W = ex.length(), H = ey.length();
    if (W - 2 * depth < minCourt || H - 2 * depth < minCourt) return cx;
    const Vec2 ux = ex * (1.0 / W), uy = ey * (1.0 / H);
    auto rect = [&](Real x0, Real y0, Real x1, Real y1) {
        Poly2 r{o + ux * x0 + uy * y0, o + ux * x1 + uy * y0, o + ux * x1 + uy * y1, o + ux * x0 + uy * y1};
        ensureCCW(r);
        return r;
    };
    // the wings (south, east, north, west) then the corners (SW, SE, NE, NW)
    const Poly2 plans[8] = {rect(depth, 0, W - depth, depth),          rect(W - depth, depth, W, H - depth),
                            rect(depth, H - depth, W - depth, H),      rect(0, depth, depth, H - depth),
                            rect(0, 0, depth, depth),                  rect(W - depth, 0, W, depth),
                            rect(W - depth, H - depth, W, H),          rect(0, H - depth, depth, H)};
    for (int i = 0; i < 8; ++i) {
        CellFrame c;
        c.plan = plans[i];
        c.params = params;
        if (!floorsPerWing.empty()) c.params.floors = std::max(1, floorsPerWing[std::min(static_cast<std::size_t>(i), floorsPerWing.size() - 1)]);
        c.params.partyWalls = 0;
        c.params.partyDoors = 0;
        c.frame.origin = c.plan[0];
        c.frame.u = ux;
        c.frame.v = uy;
        c.role = i < 4 ? "wing" : "corner";
        c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
        cx.cells.push_back(c);
    }
    cx.resolve();
    cx.connectLobbies(Vec2(params.faceDir.x, params.faceDir.z));
    return cx;
}

}  // namespace engine
