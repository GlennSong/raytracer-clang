#include "cells.h"
#include <string>
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
    // the complex's own boundary: the convex hull round every cell; a face on it is a street face (a bundle's and
    // a courtyard's outer faces lie on their box; a ring's outer chords on its polygon; a court's faces on neither)
    Poly2 all;
    for (const CellFrame& c : cells) all.insert(all.end(), c.plan.begin(), c.plan.end());
    const Poly2 hull = all.size() >= 3 ? convexHull(all) : Poly2{};
    auto onBoundary = [&](const Vec2& m) {
        for (std::size_t i = 0; i < hull.size(); ++i) {
            const Vec2 a = hull[i], b = hull[(i + 1) % hull.size()];
            const Vec2 ab = b - a;
            const Real L2 = ab.lengthSquared();
            const Real t = L2 > 1e-12 ? std::max(Real(0), std::min(Real(1), dot(m - a, ab) / L2)) : Real(0);
            if ((m - (a + ab * t)).length() < 0.3) return true;
        }
        return false;
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
    // SHARED LOBBIES (ADR-0179): every shared wall that is a whole edge of both cells takes a doorway in its middle
    // bay on BOTH sides -- the two cut the same bay (one layout, one bay width, the edges one segment), so the
    // ground floors of a bundle, a courtyard block or a ring are one lobby. (The inner cell's entrance above
    // already stands on such a wall: its own edge is its entrance, the neighbour's the doorway.)
    auto wholeEdge = [&](const CellFrame& c, const Vec2& nOut, Real at, Real run) {
        for (std::size_t e = 0; e < c.plan.size(); ++e) {
            const Vec2 a = c.plan[e], b = c.plan[(e + 1) % c.plan.size()];
            const Real L = (b - a).length();
            if (L < 1e-6) continue;
            const Vec2 d = (b - a) * (1.0 / L);
            const Vec2 nrm(d.y, -d.x);
            if (dot(nrm, nOut) > 0.999 && std::fabs(dot(nrm, a) - at) < 0.3 && std::fabs(L - run) < 0.3) return true;
        }
        return false;
    };
    for (Adjacency& ad : adj) {
        if (ad.a < 0 || ad.b < 0) continue;
        CellFrame& ca = cells[static_cast<std::size_t>(ad.a)];
        CellFrame& cb = cells[static_cast<std::size_t>(ad.b)];
        const Real run = (ad.p1 - ad.p0).length();
        if (run < 4.0) continue;
        if (!wholeEdge(ca, ad.n, ad.at, run) || !wholeEdge(cb, ad.n * -1.0, -ad.at, run)) continue;
        auto openWall = [&](CellFrame& c, const Vec2& nOut, Real at) {
            for (int k = 0; k < c.params.partyWalls; ++k)
                if (dot(c.params.partyN[k], nOut) > 0.999 && std::fabs(c.params.partyAt[k] - at) < 0.3)
                    c.params.partyDoors = static_cast<uint8_t>(c.params.partyDoors | (1u << k));
        };
        openWall(ca, ad.n, ad.at);
        openWall(cb, ad.n * -1.0, -ad.at);
        bool had = false;
        for (const Opening& o : ad.openings) if (o.k0 == 0 && std::fabs((o.u0 + o.u1) * 0.5 - run * 0.5) < 1.0) had = true;
        if (!had) ad.openings.push_back({run * 0.5 - 0.6, run * 0.5 + 0.6, 0, 0});
    }
}

void Complex::resolveBridges() {
    for (Bridge& br : bridges) {
        br.placed = false;
        if (br.a < 0 || br.b < 0 || br.a >= static_cast<int>(cells.size()) || br.b >= static_cast<int>(cells.size()) || br.a == br.b) continue;
        CellFrame& ca = cells[static_cast<std::size_t>(br.a)];
        CellFrame& cb = cells[static_cast<std::size_t>(br.b)];
        if (ca.plan.size() < 3 || cb.plan.size() < 3) continue;
        // the facing pair: a's edge whose outward normal points at b, b's edge facing back along the same line
        // direction, the two overlapping along their run; the nearest such pair
        Real bestGap = 1e30;
        std::size_t ea = 0, eb = 0;
        Vec2 bestN;
        Real u0 = 0, u1 = 0;   // the overlap along a's edge direction, from a's edge start
        for (std::size_t i = 0; i < ca.plan.size(); ++i) {
            const Vec2 a0 = ca.plan[i], a1 = ca.plan[(i + 1) % ca.plan.size()];
            const Real La = (a1 - a0).length();
            if (La < 2.0) continue;
            const Vec2 da = (a1 - a0) * (1.0 / La);
            const Vec2 na(da.y, -da.x);
            for (std::size_t j = 0; j < cb.plan.size(); ++j) {
                const Vec2 b0 = cb.plan[j], b1 = cb.plan[(j + 1) % cb.plan.size()];
                const Real Lb = (b1 - b0).length();
                if (Lb < 2.0) continue;
                const Vec2 db = (b1 - b0) * (1.0 / Lb);
                const Vec2 nb(db.y, -db.x);
                if (dot(na, nb) > -0.999) continue;                  // not facing back
                const Real gap = dot(b0 - a0, na);
                if (gap < 1.0) continue;                             // b must stand in front of a's face
                // the overlap along a's direction
                const Real tb0 = dot(b0 - a0, da), tb1 = dot(b1 - a0, da);
                const Real o0 = std::max(Real(0), std::min(tb0, tb1)), o1 = std::min(La, std::max(tb0, tb1));
                if (o1 - o0 < std::max(Real(2.0), br.width + 1.0)) continue;
                if (gap < bestGap) { bestGap = gap; ea = i; eb = j; bestN = na; u0 = o0; u1 = o1; }
            }
        }
        if (bestGap > 1e29) continue;
        const int top = std::min(ca.params.floors, cb.params.floors);
        if (top < 3) continue;
        int storey = br.storey > 0 ? br.storey : std::max(2, (top * 2) / 3);
        storey = std::max(1, std::min(storey, top));
        const Vec2 a0 = ca.plan[ea], a1 = ca.plan[(ea + 1) % ca.plan.size()];
        const Vec2 da = normalize(a1 - a0);
        const Vec2 centre = a0 + da * ((u0 + u1) * 0.5);
        const Real w = std::max(Real(1.8), std::min(br.width, (u1 - u0) - 1.0));
        auto link = [&](CellFrame& c, const Vec2& n, const Vec2& at, Real len) {
            if (c.params.links >= 2) return false;
            const int k = c.params.links++;
            c.params.linkN[k] = n;
            c.params.linkC[k] = at;
            c.params.linkStorey[k] = static_cast<int16_t>(storey);
            c.params.linkLen[k] = len;
            c.params.linkW[k] = w;
            return true;
        };
        if (!link(ca, bestN, centre, bestGap)) continue;
        link(cb, bestN * -1.0, centre + bestN * bestGap, 0);
        (void)eb;
        br.storey = storey;
        br.width = w;
        br.span = bestGap;
        br.placed = true;
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
        c.params.top = 9;   // one crown for the block (below), the other roofs bare
        c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
        cx.cells.push_back(c);
    }
    {   // the crown on the tallest cell (the first of them)
        std::size_t tallest = 0;
        for (std::size_t i = 1; i < cx.cells.size(); ++i)
            if (cx.cells[i].params.floors > cx.cells[tallest].params.floors) tallest = i;
        cx.cells[tallest].params.top = params.top;
    }
    cx.resolve();
    cx.connectLobbies(Vec2(params.faceDir.x, params.faceDir.z));
    return cx;
}

Complex ringCells(const Poly2& outlineIn, const BuildingParams& params, Real depth, const std::vector<int>& floorsPerCell, Real minCourt) {
    Complex cx;
    Poly2 outer = outlineIn;
    if (outer.size() < 3 || depth < 4.0) return cx;
    ensureCCW(outer);
    const Poly2 inner = offsetPlanPublic(outer, depth);
    if (inner.size() != outer.size() || area(inner) < 1.0) return cx;
    {
        const OBB2 ib = orientedBoundingBox(inner);
        if (2 * std::min(ib.half[0], ib.half[1]) < minCourt) return cx;
    }
    const std::size_t n = outer.size();
    for (std::size_t i = 0; i < n; ++i) {
        CellFrame c;
        c.plan = {outer[i], outer[(i + 1) % n], inner[(i + 1) % n], inner[i]};
        ensureCCW(c.plan);
        if (area(c.plan) < 20.0) continue;
        c.params = params;
        if (!floorsPerCell.empty()) c.params.floors = std::max(1, floorsPerCell[std::min(i, floorsPerCell.size() - 1)]);
        c.params.partyWalls = 0;
        c.params.partyDoors = 0;
        const Vec2 d = normalize(outer[(i + 1) % n] - outer[i]);
        c.frame.origin = outer[i];
        c.frame.u = d;
        c.frame.v = Vec2(-d.y, d.x);
        c.role = "segment";
        c.params.top = 9;   // a ring's roof is one clean deck: no plant on its segments
        c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
        cx.cells.push_back(c);
    }
    cx.resolve();
    cx.connectLobbies(Vec2(params.faceDir.x, params.faceDir.z));
    return cx;
}

void Complex::openWall(std::size_t ai, Real u0, Real u1, int k0, int k1) {
    if (ai >= adj.size() || k1 < k0) return;
    Adjacency& ad = adj[ai];
    if (ad.a < 0 || ad.b < 0) return;
    const Real run = (ad.p1 - ad.p0).length();
    if (run < 1e-6) return;
    u0 = std::max(Real(0), std::min(u0, u1));
    u1 = std::min(run, std::max(u0, u1));
    if (u1 - u0 < 0.5) return;
    const Vec2 d = (ad.p1 - ad.p0) * (1.0 / run);
    const Vec2 P0 = ad.p0 + d * u0, P1 = ad.p0 + d * u1;
    auto apply = [&](CellFrame& c, const Vec2& nOut, Real at) {
        const Vec2 tangent(-nOut.y, nOut.x);
        for (int k = 0; k < c.params.partyWalls; ++k) {
            if (dot(c.params.partyN[k], nOut) < 0.999 || std::fabs(c.params.partyAt[k] - at) > 0.3) continue;
            const Real t0 = dot(P0, tangent), t1 = dot(P1, tangent);
            c.params.partyOpen0[k] = std::min(t0, t1);
            c.params.partyOpen1[k] = std::max(t0, t1);
            c.params.partyOpenK0[k] = static_cast<int16_t>(k0);
            c.params.partyOpenK1[k] = static_cast<int16_t>(k1);
            // the party door in the middle bay is inside a ground opening across the middle: drop it
            if (k0 == 0 && u0 < run * 0.5 - 0.6 && u1 > run * 0.5 + 0.6) c.params.partyDoors = static_cast<uint8_t>(c.params.partyDoors & ~(1u << k));
        }
    };
    apply(cells[static_cast<std::size_t>(ad.a)], ad.n, ad.at);
    apply(cells[static_cast<std::size_t>(ad.b)], ad.n * -1.0, -ad.at);
    ad.openings.push_back({u0, u1, k0, k1});
}

Complex mallCells(const Poly2& boxIn, const Vec2& faceDirIn, const BuildingParams& mall, const BuildingParams& anchor,
                  const BuildingParams& garage, bool withGarage, Real anchorDepth) {
    Complex cx;
    Poly2 box = boxIn;
    if (box.size() != 4) return cx;
    ensureCCW(box);
    const OBB2 ob = orientedBoundingBox(box);
    const int la = ob.longAxis();
    const Real L = 2 * ob.half[la], D = 2 * ob.half[1 - la];
    const Real spineD = 40.0;                      // the mall's wings are 40 m wide (MallWings::a)
    anchorDepth = std::max(Real(20), anchorDepth);
    if (L < 2 * anchorDepth + 82.0 || D < spineD) return cx;
    const Vec2 u = ob.axis[la];
    Vec2 v = ob.axis[1 - la];                      // v away from the street
    const Vec2 fd = faceDirIn.lengthSquared() > 1e-9 ? normalize(faceDirIn) : Vec2(0, -1);
    if (dot(v, fd) > 0) v = v * -1.0;
    const Vec2 o = ob.center - u * (L * 0.5) - v * (D * 0.5);   // the street-side corner at u = 0
    auto rect = [&](Real u0, Real u1, Real v0, Real v1) {
        Poly2 r = {o + u * u0 + v * v0, o + u * u1 + v * v0, o + u * u1 + v * v1, o + u * u0 + v * v1};
        ensureCCW(r);
        return r;
    };
    auto cell = [&](const Poly2& plan, const BuildingParams& params, const char* role) {
        CellFrame c;
        c.plan = plan;
        c.params = params;
        c.params.partyWalls = 0;
        c.params.partyDoors = 0;
        c.params.links = 0;
        for (int k = 0; k < 4; ++k) { c.params.partyOpenK0[k] = 0; c.params.partyOpenK1[k] = -1; }
        c.frame.origin = plan[0];
        c.frame.u = u;
        c.frame.v = v;
        c.role = role;
        c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
        cx.cells.push_back(c);
    };
    // THE SPINE: the mall between the anchors, its court in its middle, both wings open at their ends
    BuildingParams mp = mall;
    mp.bigBox = 6;
    mp.mallArms = 0x3;
    mp.mallOpenEnds = 0x3;
    mp.faceDir = Vec3(-v.x, 0, -v.y);
    const Poly2 spine = rect(anchorDepth, L - anchorDepth, 0, spineD);
    mp.mallCourt = o + u * (L * 0.5) + v * (spineD * 0.5);
    cell(spine, mp, "spine");
    // THE ANCHORS: the spine's floors and heights, a department store's upper floor, entered from the street too
    for (int side = 0; side < 2; ++side) {
        BuildingParams ap = anchor;
        ap.floors = mall.floors;
        ap.groundHeight = mall.groundHeight;
        ap.floorHeight = mall.floorHeight;
        ap.storeFloors = static_cast<uint8_t>(std::max(0, mall.floors));
        ap.faceDir = Vec3(-v.x, 0, -v.y);
        cell(side == 0 ? rect(0, anchorDepth, 0, spineD) : rect(L - anchorDepth, L, 0, spineD), ap, "anchor");
    }
    // THE GARAGE behind the court, when the box is deep enough for a deck's worth
    const Real gd = D - spineD;
    if (withGarage && gd >= 24.0) {
        BuildingParams gp = garage;
        gp.faceDir = Vec3(v.x, 0, v.y);   // its ramps on the back street
        cell(rect(L * 0.5 - spineD * 0.5, L * 0.5 + spineD * 0.5, spineD, std::min(D, spineD + 44.0)), gp, "garage");
    }
    cx.resolve();
    cx.connectLobbies(fd);
    // THE OPENINGS: the concourse's width (12 m, centred) through each anchor wall on every level; 8 m into the
    // garage at the ground
    for (std::size_t i = 0; i < cx.adj.size(); ++i) {
        const Adjacency& ad = cx.adj[i];
        const std::string& ra = cx.cells[static_cast<std::size_t>(ad.a)].role;
        const std::string& rb = cx.cells[static_cast<std::size_t>(ad.b)].role;
        const bool anchorWall = (ra == "spine" && rb == "anchor") || (ra == "anchor" && rb == "spine");
        const bool garageWall = (ra == "spine" && rb == "garage") || (ra == "garage" && rb == "spine");
        const Real run = (ad.p1 - ad.p0).length();
        if (anchorWall) cx.openWall(i, run * 0.5 - 6.0, run * 0.5 + 6.0, 0, std::max(0, mall.floors));
        else if (garageWall) cx.openWall(i, run * 0.5 - 4.0, run * 0.5 + 4.0, 0, 0);
    }
    return cx;
}

Complex twinTowers(const Poly2& boxIn, const BuildingParams& params, Real gap, const std::vector<int>& floorsPerCell, int bridgeStorey, Real bridgeW) {
    Complex cx;
    Poly2 box = boxIn;
    if (box.size() != 4) return cx;
    ensureCCW(box);
    const Vec2 o = box[0];
    const Vec2 ex = box[1] - box[0], ey = box[3] - box[0];
    const Real L = ex.length();
    gap = std::max(Real(4.0), gap);
    const Real each = (L - gap) * 0.5;
    if (each < kCoreTierMin + 1.0 || ey.length() < kCoreTierMin + 1.0) return cx;
    const Vec2 ux = ex * (1.0 / L);
    for (int i = 0; i < 2; ++i) {
        CellFrame c;
        const Vec2 x0 = o + ux * (i == 0 ? Real(0) : each + gap);
        c.plan = {x0, x0 + ux * each, x0 + ux * each + ey, x0 + ey};
        ensureCCW(c.plan);
        c.params = params;
        const std::size_t idx = static_cast<std::size_t>(i);
        if (!floorsPerCell.empty()) c.params.floors = std::max(1, floorsPerCell[std::min(idx, floorsPerCell.size() - 1)]);
        c.params.partyWalls = 0;
        c.params.partyDoors = 0;
        c.params.links = 0;
        c.frame.origin = c.plan[0];
        c.frame.u = ux;
        c.frame.v = normalize(ey);
        c.role = "twin";
        c.mass = std::make_shared<Stack>(stackOf(c.plan, c.params));
        cx.cells.push_back(c);
    }
    cx.resolve();
    cx.connectLobbies(Vec2(params.faceDir.x, params.faceDir.z));
    Bridge br;
    br.a = 0;
    br.b = 1;
    br.storey = bridgeStorey;
    br.width = bridgeW;
    cx.bridges.push_back(br);
    cx.resolveBridges();
    return cx;
}

}  // namespace engine
