#include "polymesh.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace engine {

namespace {
std::pair<int, int> ekey(int a, int b) { return a < b ? std::make_pair(a, b) : std::make_pair(b, a); }
long long dkey(int a, int b) { return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b); }
}  // namespace

// --- PolyMesh -----------------------------------------------------------------------------------------

int PolyMesh::groupId(const std::string& name) {
    const int g = findGroup(name);
    if (g >= 0) return g;
    groups.push_back(name);
    return static_cast<int>(groups.size()) - 1;
}
int PolyMesh::matId(const std::string& name) {
    const int k = findMat(name);
    if (k >= 0) return k;
    mats.push_back(name);
    return static_cast<int>(mats.size()) - 1;
}
int PolyMesh::findGroup(const std::string& name) const {
    for (std::size_t i = 0; i < groups.size(); ++i) if (groups[i] == name) return static_cast<int>(i);
    return -1;
}
int PolyMesh::findMat(const std::string& name) const {
    for (std::size_t i = 0; i < mats.size(); ++i) if (mats[i] == name) return static_cast<int>(i);
    return -1;
}
float PolyMesh::crease(int a, int b) const {
    const auto it = creases.find(ekey(a, b));
    return it == creases.end() ? 0.0f : it->second;
}
void PolyMesh::setCrease(int a, int b, float s) {
    if (s <= 0.0f) creases.erase(ekey(a, b));
    else creases[ekey(a, b)] = s;
}
float PolyMesh::corner(int v) const {
    const auto it = corners.find(v);
    return it == corners.end() ? 0.0f : it->second;
}
void PolyMesh::setCorner(int v, float s) {
    if (s <= 0.0f) corners.erase(v);
    else corners[v] = s;
}
Vec3 PolyMesh::faceCentroid(int f) const {
    Vec3 c(0, 0, 0);
    for (int i : faces[static_cast<std::size_t>(f)].v) c = c + pts[static_cast<std::size_t>(i)];
    return c / static_cast<double>(faces[static_cast<std::size_t>(f)].v.size());
}
Vec3 PolyMesh::faceNormal(int f) const {
    const auto& v = faces[static_cast<std::size_t>(f)].v;
    Vec3 n(0, 0, 0);   // Newell: robust for non-planar polygons
    for (std::size_t i = 0; i < v.size(); ++i) {
        const Vec3& a = pts[static_cast<std::size_t>(v[i])];
        const Vec3& b = pts[static_cast<std::size_t>(v[(i + 1) % v.size()])];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    const double l = n.length();
    return l > 1e-12 ? n / l : Vec3(0, 1, 0);
}
void PolyMesh::append(const PolyMesh& o) {
    const int base = static_cast<int>(pts.size());
    pts.insert(pts.end(), o.pts.begin(), o.pts.end());
    for (Face f : o.faces) {
        for (int& i : f.v) i += base;
        f.group = groupId(o.groups[static_cast<std::size_t>(f.group)]);
        f.mat = matId(o.mats[static_cast<std::size_t>(f.mat)]);
        faces.push_back(std::move(f));
    }
    for (const auto& [e, s] : o.creases) setCrease(e.first + base, e.second + base, s);
    for (const auto& [v, s] : o.corners) setCorner(v + base, s);
}

// --- sections and loft ----------------------------------------------------------------------------------

std::vector<Vec2> roundedPolygon(const std::vector<Vec2>& c, const std::vector<double>& radii, int segsAll,
                                 const std::vector<int>& segsPer) {
    const std::size_t n = c.size();
    std::vector<Vec2> out;
    if (n < 3) return c;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 p = c[i], a = c[(i + n - 1) % n], b = c[(i + 1) % n];
        const int segs = segsPer.empty() ? std::max(segsAll, 1) : std::max(segsPer[std::min(i, segsPer.size() - 1)], 0);
        if (segs == 0) { out.push_back(p); continue; }   // a sharp corner: exactly one point
        // EVERY corner emits exactly segs + 1 points, so sections built from the same corner list always
        // correspond point for point (a recipe indexes faces by corner). A corner with no radius, or a
        // straight one, gets its points spread over a millimetre along its edges.
        double r = radii.empty() ? 0.0 : radii[std::min(i, radii.size() - 1)];
        const Vec2 da = a - p, db = b - p;
        const double la = da.length(), lb = db.length();
        const Vec2 ua = la > 1e-12 ? da * (1.0 / la) : Vec2(-1, 0), ub = lb > 1e-12 ? db * (1.0 / lb) : Vec2(1, 0);
        const double cosT = std::clamp(ua.x * ub.x + ua.y * ub.y, -1.0, 1.0);
        const double theta = std::acos(cosT);              // the corner's interior angle
        if (r <= 1e-9 || la < 1e-6 || lb < 1e-6 || theta < 1e-3 || theta > 3.1405) {
            // a SHARP corner: its points split evenly either side of it along the two edges -- a chamfer of
            // at most a few centimetres that a crease keeps crisp. Symmetric (a mirrored corner mirrors),
            // and at a real spacing, so the loft grows no sliver strips.
            const double d = std::min({0.05, 0.3 * la, 0.3 * lb});
            const int k = segs + 1, h = k / 2;
            for (int s2 = 0; s2 < k; ++s2) {
                if (k % 2 == 1 && s2 == h) { out.push_back(p); continue; }
                if (s2 < h) out.push_back(p + ua * (d * static_cast<double>(h - s2) / h));
                else out.push_back(p + ub * (d * static_cast<double>(s2 - (k - h) + 1) / h));
            }
            continue;
        }
        double t = r / std::tan(theta * 0.5);              // tangent distance along each edge
        // 45%, not half: two neighbours both clamped to half their shared edge would meet at its midpoint in
        // one duplicated point -- a zero-area strip down the whole loft
        const double tMax = 0.45 * std::min(la, lb);
        if (t > tMax) { t = tMax; r = t * std::tan(theta * 0.5); }
        const Vec2 pa = p + ua * t, pb = p + ub * t;
        // arc centre along the bisector
        Vec2 bis = ua + ub;
        bis = bis * (1.0 / std::max(1e-12, bis.length()));
        const Vec2 ctr = p + bis * (r / std::sin(theta * 0.5));
        const double a0 = std::atan2(pa.y - ctr.y, pa.x - ctr.x), a1raw = std::atan2(pb.y - ctr.y, pb.x - ctr.x);
        double d = a1raw - a0;
        while (d > M_PI) d -= 2 * M_PI;
        while (d < -M_PI) d += 2 * M_PI;
        for (int s = 0; s <= segs; ++s) {
            const double ang = a0 + d * (static_cast<double>(s) / segs);
            out.push_back(Vec2(ctr.x + r * std::cos(ang), ctr.y + r * std::sin(ang)));
        }
    }
    return out;
}

std::vector<Vec2> resampleClosed(const std::vector<Vec2>& poly, int n) {
    const std::size_t m = poly.size();
    if (m < 2 || n < 3) return poly;
    // start: the lowest crossing of x = 0 (a symmetric body's bottom centre), else the lowest point
    double bestY = 1e30, bestS = 0;
    std::vector<double> cum(m + 1, 0.0);
    for (std::size_t i = 0; i < m; ++i) cum[i + 1] = cum[i] + (poly[(i + 1) % m] - poly[i]).length();
    const double total = cum[m];
    bool crossed = false;
    for (std::size_t i = 0; i < m; ++i) {
        const Vec2 a = poly[i], b = poly[(i + 1) % m];
        if ((a.x <= 0 && b.x > 0) || (a.x >= 0 && b.x < 0) || a.x == 0) {
            const double t = std::fabs(b.x - a.x) < 1e-12 ? 0.0 : (0 - a.x) / (b.x - a.x);
            const double y = a.y + (b.y - a.y) * t;
            if (y < bestY) { bestY = y; bestS = cum[i] + t * (cum[i + 1] - cum[i]); crossed = true; }
        }
    }
    if (!crossed)
        for (std::size_t i = 0; i < m; ++i) if (poly[i].y < bestY) { bestY = poly[i].y; bestS = cum[i]; }
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(n));
    std::size_t seg = 0;
    for (int k = 0; k < n; ++k) {
        double s = bestS + total * k / n;
        if (s >= total) s -= total;
        seg = 0;
        while (seg + 1 < m + 1 && cum[seg + 1] < s) ++seg;
        const double L = cum[seg + 1] - cum[seg];
        const double t = L > 1e-12 ? (s - cum[seg]) / L : 0.0;
        out.push_back(poly[seg % m] + (poly[(seg + 1) % m] - poly[seg % m]) * t);
    }
    return out;
}

PolyMesh loft(const std::vector<std::vector<Vec2>>& sections, const std::vector<double>& stations, bool capStart,
              bool capEnd, float capCrease) {
    PolyMesh m;
    const std::size_t R = std::min(sections.size(), stations.size());
    if (R < 2) return m;
    const std::size_t n = sections[0].size();
    for (std::size_t k = 0; k < R; ++k)
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 p = sections[k][std::min(i, sections[k].size() - 1)];
            m.pts.push_back(Vec3(p.x, p.y, stations[k]));
        }
    auto id = [&](std::size_t k, std::size_t i) { return static_cast<int>(k * n + (i % n)); };
    // each section is CCW seen from +z; ring k at lower z than ring k+1. Outward quad order:
    for (std::size_t k = 0; k + 1 < R; ++k)
        for (std::size_t i = 0; i < n; ++i) {
            PolyMesh::Face f;
            f.v = {id(k, i), id(k, i + 1), id(k + 1, i + 1), id(k + 1, i)};
            m.faces.push_back(f);
        }
    // CAPS WITHOUT SLIVERS. A section symmetric about x = 0 closes as a modeller would: a narrow RIM of
    // faces just inside the ring (inner points merged wherever they fall closer than kMinGap, with a
    // triangle at each merge), then a LADDER of quads across the simplified inner outline, each rung
    // joining a point to its mirror. A ring with closely spaced points (a shoulder, a sill ledge) would
    // otherwise make full-width rungs millimetres tall. Asymmetric sections fall back to one n-gon.
    constexpr double kMinGap = 0.05;
    auto symCap = [&](std::size_t k, bool reverse, const std::string& groupName) -> bool {
        const std::vector<Vec2>& ring = sections[k];
        const double tol = 1e-4;
        std::vector<int> mir(n, -1);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j)
                if (std::fabs(ring[j].x + ring[i].x) < tol && std::fabs(ring[j].y - ring[i].y) < tol) { mir[i] = static_cast<int>(j); break; }
            if (mir[i] < 0) return false;
        }
        // the ring's size sets the rim width
        double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
        for (const Vec2& q : ring) { minX = std::min(minX, q.x); maxX = std::max(maxX, q.x); minY = std::min(minY, q.y); maxY = std::max(maxY, q.y); }
        const double rim = std::min(0.04, 0.08 * std::min(maxX - minX, maxY - minY));
        // inner points: each ring point moved inward along its 2-D bisector (CCW ring: inward = left)
        std::vector<Vec2> inner(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 pp = ring[(i + n - 1) % n], p = ring[i], nx = ring[(i + 1) % n];
            Vec2 e0 = p - pp, e1 = nx - p;
            const double l0 = e0.length(), l1 = e1.length();
            e0 = l0 > 1e-12 ? e0 / l0 : e1 / std::max(1e-12, l1);
            e1 = l1 > 1e-12 ? e1 / l1 : e0;
            const Vec2 i0(-e0.y, e0.x), i1(-e1.y, e1.x);   // left normals (inward for a CCW ring)
            Vec2 bi = i0 + i1;
            const double bl = bi.length();
            bi = bl > 1e-9 ? bi / bl : i0;
            inner[i] = p + bi * (rim / std::max(0.4, i0.x * bi.x + i0.y * bi.y));
        }
        // merge runs of inner points closer than kMinGap: group[i] -> a merged inner point. Done on the
        // right half and mirrored so the merged outline stays symmetric.
        std::vector<int> group(n, -1);
        std::vector<Vec2> gpts;
        std::vector<std::size_t> order(n);
        // start the walk at a point whose predecessor is far away, so no group straddles the start
        std::size_t start = 0;
        for (std::size_t i = 0; i < n; ++i)
            if ((inner[i] - inner[(i + n - 1) % n]).length() >= kMinGap) { start = i; break; }
        for (std::size_t s2 = 0; s2 < n; ++s2) order[s2] = (start + s2) % n;
        for (std::size_t s2 = 0; s2 < n; ++s2) {
            const std::size_t i = order[s2];
            if (group[i] >= 0) continue;
            if (ring[i].x < -tol) continue;   // left points follow their mirror
            // a new group from i, taking following right-side points within kMinGap of its first point
            const int gid = static_cast<int>(gpts.size());
            Vec2 acc = inner[i];
            int cnt = 1;
            group[i] = gid;
            for (std::size_t t = s2 + 1; t < n; ++t) {
                const std::size_t j = order[t];
                if (ring[j].x < -tol || group[j] >= 0) break;
                // join while close, or while nearly LEVEL with the group's first point: the ladder's rungs
                // must stand at least ~kMinGap apart, however far a horizontal ledge runs sideways
                const Vec2 dd = inner[j] - inner[i];
                if (dd.length() >= kMinGap && std::fabs(dd.y) >= kMinGap) break;
                group[j] = gid;
                acc = acc + inner[j];
                ++cnt;
            }
            Vec2 c = acc / static_cast<double>(cnt);
            if (std::fabs(ring[i].x) <= tol) c.x = 0;   // an axis point stays on the axis
            gpts.push_back(c);
        }
        // mirror groups for the left side
        std::vector<int> mirGroup(gpts.size(), -1);
        const std::size_t rightGroups = gpts.size();
        for (std::size_t i = 0; i < n; ++i) {
            if (ring[i].x >= -tol) continue;
            const int rg = group[static_cast<std::size_t>(mir[i])];
            if (rg < 0) return false;
            if (std::fabs(gpts[static_cast<std::size_t>(rg)].x) <= tol) { group[i] = rg; continue; }
            if (mirGroup[static_cast<std::size_t>(rg)] < 0) {
                mirGroup[static_cast<std::size_t>(rg)] = static_cast<int>(gpts.size());
                gpts.push_back(Vec2(-gpts[static_cast<std::size_t>(rg)].x, gpts[static_cast<std::size_t>(rg)].y));
            }
            group[i] = mirGroup[static_cast<std::size_t>(rg)];
        }
        (void)rightGroups;
        // the inner outline in ring order (consecutive duplicates removed)
        std::vector<int> outline;
        for (std::size_t s2 = 0; s2 < n; ++s2) {
            const int gg = group[(start + s2) % n];
            if (outline.empty() || outline.back() != gg) outline.push_back(gg);
        }
        while (outline.size() > 1 && outline.front() == outline.back()) outline.pop_back();
        if (outline.size() < 4) return false;
        const int base = static_cast<int>(m.pts.size());
        for (const Vec2& q : gpts) m.pts.push_back(Vec3(q.x, q.y, stations[k]));
        const int g = m.groupId(groupName), gRim = m.groupId(groupName + "_rim");
        auto face = [&](std::vector<int> v, int gid) {
            PolyMesh::Face f;
            f.v = std::move(v);
            if (reverse) std::reverse(f.v.begin(), f.v.end());
            f.group = gid;
            m.faces.push_back(f);
        };
        // the rim (its own group, "<cap>_rim"): ring edge (i, i+1) against inner (group[i], group[i+1])
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            const int gi = base + group[i], gj = base + group[j];
            if (gi == gj) face({id(k, i), id(k, j), gi}, gRim);
            else face({id(k, i), id(k, j), gj, gi}, gRim);
        }
        // the ladder across the inner outline: pair each right-side outline point with its mirror
        std::vector<int> mirOf(gpts.size(), -1);
        for (std::size_t q = 0; q < gpts.size(); ++q)
            for (std::size_t r2 = 0; r2 < gpts.size(); ++r2)
                if (std::fabs(gpts[r2].x + gpts[q].x) < tol && std::fabs(gpts[r2].y - gpts[q].y) < tol) { mirOf[q] = static_cast<int>(r2); break; }
        std::size_t os = 0;
        const std::size_t on = outline.size();
        for (std::size_t q = 0; q < on; ++q)
            if (gpts[static_cast<std::size_t>(outline[q])].x > tol && gpts[static_cast<std::size_t>(outline[(q + on - 1) % on])].x <= tol) { os = q; break; }
        std::vector<int> rightChain;
        for (std::size_t q = 0; q < on; ++q) {
            const int gg = outline[(os + q) % on];
            if (gpts[static_cast<std::size_t>(gg)].x > tol) rightChain.push_back(gg); else break;
        }
        if (rightChain.size() < 2 && on > 4) return false;
        const int before = outline[(os + on - 1) % on], after = outline[(os + rightChain.size()) % on];
        if (std::fabs(gpts[static_cast<std::size_t>(before)].x) <= tol)
            face({base + before, base + rightChain.front(), base + mirOf[static_cast<std::size_t>(rightChain.front())]}, g);
        for (std::size_t q = 0; q + 1 < rightChain.size(); ++q) {
            const int r0 = rightChain[q], r1 = rightChain[q + 1];
            face({base + r0, base + r1, base + mirOf[static_cast<std::size_t>(r1)], base + mirOf[static_cast<std::size_t>(r0)]}, g);
        }
        if (std::fabs(gpts[static_cast<std::size_t>(after)].x) <= tol)
            face({base + rightChain.back(), base + after, base + mirOf[static_cast<std::size_t>(rightChain.back())]}, g);
        return true;
    };
    if (capStart && n > 4 && symCap(0, true, "cap_start")) {   // a quad ring is its own best cap
        for (std::size_t i = 0; i < n; ++i) m.setCrease(id(0, i), id(0, i + 1), capCrease);
        capStart = false;
    }
    if (capEnd && n > 4 && symCap(R - 1, false, "cap_end")) {
        for (std::size_t i = 0; i < n; ++i) m.setCrease(id(R - 1, i), id(R - 1, i + 1), capCrease);
        capEnd = false;
    }
    if (capStart) {   // at the lowest z the outside is -z: reverse the CCW(+z) order
        PolyMesh::Face f;
        for (std::size_t i = n; i-- > 0;) f.v.push_back(id(0, i));
        f.group = m.groupId("cap_start");
        m.faces.push_back(f);
        for (std::size_t i = 0; i < n; ++i) m.setCrease(id(0, i), id(0, i + 1), capCrease);
    }
    if (capEnd) {
        PolyMesh::Face f;
        for (std::size_t i = 0; i < n; ++i) f.v.push_back(id(R - 1, i));
        f.group = m.groupId("cap_end");
        m.faces.push_back(f);
        for (std::size_t i = 0; i < n; ++i) m.setCrease(id(R - 1, i), id(R - 1, i + 1), capCrease);
    }
    return m;
}

// --- editing ----------------------------------------------------------------------------------------------

std::vector<int> extrude(PolyMesh& m, const std::vector<int>& sel, double dist, Vec3 dir) {
    if (sel.empty()) return sel;
    std::unordered_set<int> inSel(sel.begin(), sel.end());
    // directed edges of the selection; a boundary edge is one whose reverse is not in the selection
    std::unordered_map<long long, int> dirEdge;   // (a,b) -> face
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) dirEdge[dkey(v[i], v[(i + 1) % v.size()])] = f;
    }
    // per-vertex offset direction
    std::unordered_map<int, Vec3> nsum;
    for (int f : sel) {
        const Vec3 n = m.faceNormal(f);
        for (int i : m.faces[static_cast<std::size_t>(f)].v) nsum[i] = nsum[i] + n;
    }
    const bool useDir = dir.lengthSquared() > 1e-12;
    const Vec3 d = useDir ? normalize(dir) : Vec3(0, 0, 0);
    std::unordered_map<int, int> dup;
    for (const auto& [i, s] : nsum) {
        Vec3 off;
        if (useDir) off = d * dist;
        else {
            const double l = s.length();
            off = l > 1e-12 ? s * (dist / l) : Vec3(0, 0, 0);
        }
        dup[i] = static_cast<int>(m.pts.size());
        m.pts.push_back(m.pts[static_cast<std::size_t>(i)] + off);
    }
    // side walls along boundary edges (a -> b in a selected face, b -> a not selected)
    std::vector<PolyMesh::Face> walls;
    for (const auto& [k, f] : dirEdge) {
        const int a = static_cast<int>(k >> 32), b = static_cast<int>(k & 0xffffffff);
        if (dirEdge.count(dkey(b, a))) continue;
        PolyMesh::Face w;
        w.v = {a, b, dup[b], dup[a]};
        w.group = m.faces[static_cast<std::size_t>(f)].group;
        w.mat = m.faces[static_cast<std::size_t>(f)].mat;
        w.color = m.faces[static_cast<std::size_t>(f)].color;
        walls.push_back(w);
        // the crease on the old edge moves to the lifted edge; the wall's base is left smooth
        const float s = m.crease(a, b);
        if (s > 0) { m.setCrease(dup[a], dup[b], s); m.setCrease(a, b, 0.0f); }
    }
    // interior creases move with their edges
    std::vector<std::pair<std::pair<int, int>, float>> moved;
    for (const auto& [e, s] : m.creases)
        if (dup.count(e.first) && dup.count(e.second) && dirEdge.count(dkey(e.first, e.second)) &&
            dirEdge.count(dkey(e.second, e.first)))
            moved.push_back({e, s});
    for (const auto& [e, s] : moved) { m.setCrease(e.first, e.second, 0.0f); m.setCrease(dup[e.first], dup[e.second], s); }
    for (int f : sel)
        for (int& i : m.faces[static_cast<std::size_t>(f)].v) i = dup[i];
    for (auto& w : walls) m.faces.push_back(std::move(w));
    return sel;
}

std::vector<int> inset(PolyMesh& m, const std::vector<int>& sel, double amount) {
    std::vector<int> inner;
    for (int f : sel) {
        PolyMesh::Face face = m.faces[static_cast<std::size_t>(f)];
        const std::size_t n = face.v.size();
        const Vec3 c = m.faceCentroid(f), nrm = m.faceNormal(f);
        std::vector<int> in(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec3 p = m.pts[static_cast<std::size_t>(face.v[i])];
            const Vec3 a = m.pts[static_cast<std::size_t>(face.v[(i + n - 1) % n])];
            const Vec3 b = m.pts[static_cast<std::size_t>(face.v[(i + 1) % n])];
            // move along the corner's inward bisector so both edges step in by `amount`
            const Vec3 ea = normalize(p - a), eb = normalize(b - p);
            const Vec3 ia = normalize(cross(nrm, ea)), ib = normalize(cross(nrm, eb));   // inward normals of the edges
            Vec3 bis = ia + ib;
            const double bl = bis.length();
            Vec3 q;
            if (bl < 1e-9) q = p + ia * amount;
            else {
                bis = bis / bl;
                const double cosH = std::max(0.2, dot(bis, ia));
                q = p + bis * (amount / cosH);
            }
            // never past the centroid
            const double toC = (c - p).length(), moved = (q - p).length();
            if (moved > 0.9 * toC) q = p + (q - p) * (0.9 * toC / std::max(1e-12, moved));
            in[i] = static_cast<int>(m.pts.size());
            m.pts.push_back(q);
        }
        for (std::size_t i = 0; i < n; ++i) {
            PolyMesh::Face ring = face;
            ring.v = {face.v[i], face.v[(i + 1) % n], in[(i + 1) % n], in[i]};
            m.faces.push_back(ring);
        }
        m.faces[static_cast<std::size_t>(f)].v = in;
        inner.push_back(f);
    }
    return inner;
}

std::vector<int> insetRegion(PolyMesh& m, const std::vector<int>& sel, double amount) {
    if (sel.empty()) return sel;
    // NEVER FOLD: no outline point moves further than 35% of the shortest region edge it touches, so where
    // the region narrows (a window's end, a tight corner) the inset narrows with it instead of crossing over
    std::unordered_map<int, double> shortest;
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) {
            const int a = v[i], b = v[(i + 1) % v.size()];
            const double l = (m.pts[static_cast<std::size_t>(a)] - m.pts[static_cast<std::size_t>(b)]).length();
            for (int x : {a, b}) {
                auto it = shortest.find(x);
                if (it == shortest.end() || l < it->second) shortest[x] = l;
            }
        }
    }
    extrude(m, sel, 0.0);   // duplicates the region's points in place; its outline grows a (flat) ring
    std::unordered_set<long long> de;
    std::unordered_map<int, Vec3> nsum;
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        const Vec3 fn = m.faceNormal(f);
        for (std::size_t i = 0; i < v.size(); ++i) {
            de.insert(dkey(v[i], v[(i + 1) % v.size()]));
            nsum[v[i]] = nsum[v[i]] + fn;
        }
    }
    std::unordered_map<int, int> next, prev;
    for (long long k : de) {
        const int a = static_cast<int>(k >> 32), b = static_cast<int>(k & 0xffffffff);
        if (de.count(dkey(b, a))) continue;
        next[a] = b;
        prev[b] = a;
    }
    std::vector<std::pair<int, Vec3>> moves;
    for (const auto& [v, w] : next) {
        const auto ip = prev.find(v);
        if (ip == prev.end()) continue;
        const Vec3 p = m.pts[static_cast<std::size_t>(v)];
        const Vec3 n = normalize(nsum[v]);
        const Vec3 e0 = normalize(p - m.pts[static_cast<std::size_t>(ip->second)]);
        const Vec3 e1 = normalize(m.pts[static_cast<std::size_t>(w)] - p);
        const Vec3 i0 = normalize(cross(n, e0)), i1 = normalize(cross(n, e1));   // the region lies to the left
        Vec3 b = i0 + i1;
        const double bl = b.length();
        b = bl > 1e-9 ? b / bl : i0;
        double dist = amount / std::max(0.3, dot(b, i0));
        // `v` is the moved copy; its original's shortest edge is the same length (extrude(0) moved nothing)
        const auto sh = shortest.find(v);
        double lim = sh != shortest.end() ? sh->second : 1e30;
        for (const auto& [orig, len] : shortest)
            if (sh == shortest.end() && (m.pts[static_cast<std::size_t>(orig)] - p).lengthSquared() < 1e-14) { lim = len; break; }
        // an ACUTE corner would send its point far along the bisector (amount / cos) and across the region;
        // cap it -- the band pinches a little at a sharp corner rather than folding
        dist = std::min({dist, 1.5 * amount, 0.35 * lim});
        moves.push_back({v, b * dist});
    }
    for (const auto& [v, d] : moves) m.pts[static_cast<std::size_t>(v)] = m.pts[static_cast<std::size_t>(v)] + d;
    return sel;
}

void creaseBorder(PolyMesh& m, const std::vector<int>& sel, float s) {
    std::unordered_set<long long> de;
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) de.insert(dkey(v[i], v[(i + 1) % v.size()]));
    }
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) {
            const int a = v[i], b = v[(i + 1) % v.size()];
            if (!de.count(dkey(b, a))) m.setCrease(a, b, s);
        }
    }
}

void creaseFaces(PolyMesh& m, const std::vector<int>& sel, float s) {
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) m.setCrease(v[i], v[(i + 1) % v.size()], s);
    }
}

void creaseCorners(PolyMesh& m, const std::vector<int>& sel, double angleDeg, float s) {
    std::unordered_set<long long> de;
    for (int f : sel) {
        const auto& v = m.faces[static_cast<std::size_t>(f)].v;
        for (std::size_t i = 0; i < v.size(); ++i) de.insert(dkey(v[i], v[(i + 1) % v.size()]));
    }
    // the outline as next/prev along its directed boundary edges
    std::unordered_map<int, int> next, prev;
    for (long long k : de) {
        const int a = static_cast<int>(k >> 32), b = static_cast<int>(k & 0xffffffff);
        if (de.count(dkey(b, a))) continue;
        next[a] = b;
        prev[b] = a;
    }
    const double cosLim = std::cos(angleDeg * M_PI / 180.0);
    for (const auto& [v, w] : next) {
        const auto ip = prev.find(v);
        if (ip == prev.end()) continue;
        const Vec3 d0 = m.pts[static_cast<std::size_t>(v)] - m.pts[static_cast<std::size_t>(ip->second)];
        const Vec3 d1 = m.pts[static_cast<std::size_t>(w)] - m.pts[static_cast<std::size_t>(v)];
        if (d0.lengthSquared() < 1e-14 || d1.lengthSquared() < 1e-14) continue;
        if (dot(normalize(d0), normalize(d1)) < cosLim) m.setCorner(v, s);
    }
}

void mirrorX(PolyMesh& m, double eps) {
    const int n = static_cast<int>(m.pts.size());
    std::vector<int> map(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        Vec3 p = m.pts[static_cast<std::size_t>(i)];
        if (std::fabs(p.x) <= eps) { m.pts[static_cast<std::size_t>(i)].x = 0; map[static_cast<std::size_t>(i)] = i; continue; }
        map[static_cast<std::size_t>(i)] = static_cast<int>(m.pts.size());
        m.pts.push_back(Vec3(-p.x, p.y, p.z));
    }
    const std::size_t nf = m.faces.size();
    for (std::size_t f = 0; f < nf; ++f) {
        PolyMesh::Face g = m.faces[f];
        std::reverse(g.v.begin(), g.v.end());   // a mirror flips handedness
        for (int& i : g.v) i = map[static_cast<std::size_t>(i)];
        m.faces.push_back(std::move(g));
    }
    std::vector<std::pair<std::pair<int, int>, float>> add;
    for (const auto& [e, s] : m.creases)
        if (e.first < n && e.second < n) add.push_back({{map[static_cast<std::size_t>(e.first)], map[static_cast<std::size_t>(e.second)]}, s});
    for (const auto& [e, s] : add) m.setCrease(e.first, e.second, s);
    std::vector<std::pair<int, float>> addC;
    for (const auto& [v, s] : m.corners) if (v < n) addC.push_back({map[static_cast<std::size_t>(v)], s});
    for (const auto& [v, s] : addC) m.setCorner(v, s);
    // an edge lying ON the seam was a boundary (sharp) on the half; welded, it is interior -- leave it smooth
}

// --- Catmull-Clark with creases -------------------------------------------------------------------------

namespace {
PolyMesh subdivideOnce(const PolyMesh& m) {
    PolyMesh out;
    out.groups = m.groups;
    out.mats = m.mats;
    const int nv = static_cast<int>(m.pts.size());
    // edges
    std::map<std::pair<int, int>, int> edgeIdx;
    struct E { int a, b; std::vector<int> faces; float s; };
    std::vector<E> edges;
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& v = m.faces[f].v;
        for (std::size_t i = 0; i < v.size(); ++i) {
            const auto k = ekey(v[i], v[(i + 1) % v.size()]);
            auto it = edgeIdx.find(k);
            if (it == edgeIdx.end()) {
                edgeIdx[k] = static_cast<int>(edges.size());
                edges.push_back({k.first, k.second, {static_cast<int>(f)}, m.crease(k.first, k.second)});
            } else {
                edges[static_cast<std::size_t>(it->second)].faces.push_back(static_cast<int>(f));
            }
        }
    }
    // face points
    std::vector<Vec3> fp(m.faces.size());
    for (std::size_t f = 0; f < m.faces.size(); ++f) fp[f] = m.faceCentroid(static_cast<int>(f));
    // edge points
    std::vector<Vec3> ep(edges.size());
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const E& E0 = edges[e];
        const Vec3 mid = (m.pts[static_cast<std::size_t>(E0.a)] + m.pts[static_cast<std::size_t>(E0.b)]) * 0.5;
        const bool boundary = E0.faces.size() != 2;
        if (boundary || E0.s >= 1.0f) { ep[e] = mid; continue; }
        const Vec3 smooth = (m.pts[static_cast<std::size_t>(E0.a)] + m.pts[static_cast<std::size_t>(E0.b)] +
                             fp[static_cast<std::size_t>(E0.faces[0])] + fp[static_cast<std::size_t>(E0.faces[1])]) * 0.25;
        ep[e] = E0.s > 0.0f ? smooth + (mid - smooth) * E0.s : smooth;
    }
    // vertex points
    std::vector<std::vector<int>> vEdges(static_cast<std::size_t>(nv)), vFaces(static_cast<std::size_t>(nv));
    for (std::size_t e = 0; e < edges.size(); ++e) {
        vEdges[static_cast<std::size_t>(edges[e].a)].push_back(static_cast<int>(e));
        vEdges[static_cast<std::size_t>(edges[e].b)].push_back(static_cast<int>(e));
    }
    for (std::size_t f = 0; f < m.faces.size(); ++f)
        for (int i : m.faces[f].v) vFaces[static_cast<std::size_t>(i)].push_back(static_cast<int>(f));
    std::vector<Vec3> vp(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) {
        const Vec3 V = m.pts[static_cast<std::size_t>(i)];
        const auto& es = vEdges[static_cast<std::size_t>(i)];
        const auto& fs = vFaces[static_cast<std::size_t>(i)];
        if (es.empty()) { vp[static_cast<std::size_t>(i)] = V; continue; }
        std::vector<int> sharp;
        float sharpSum = 0;
        for (int e : es) {
            const E& E0 = edges[static_cast<std::size_t>(e)];
            const float s = E0.faces.size() != 2 ? PolyMesh::kInfCrease : E0.s;
            if (s > 0.0f) { sharp.push_back(e); sharpSum += std::min(s, 1.0f); }
        }
        // smooth rule
        const double n = static_cast<double>(es.size());
        Vec3 F(0, 0, 0), Rm(0, 0, 0);
        for (int f : fs) F = F + fp[static_cast<std::size_t>(f)];
        F = F / std::max<double>(1.0, fs.size());
        for (int e : es) {
            const E& E0 = edges[static_cast<std::size_t>(e)];
            Rm = Rm + (m.pts[static_cast<std::size_t>(E0.a)] + m.pts[static_cast<std::size_t>(E0.b)]) * 0.5;
        }
        Rm = Rm / n;
        const Vec3 smoothV = (F + Rm * 2.0 + V * (n - 3.0)) / n;
        Vec3 sharpV;
        if (sharp.size() < 2) { vp[static_cast<std::size_t>(i)] = smoothV; continue; }
        if (sharp.size() == 2) {
            const E& e0 = edges[static_cast<std::size_t>(sharp[0])];
            const E& e1 = edges[static_cast<std::size_t>(sharp[1])];
            const Vec3 a = m.pts[static_cast<std::size_t>(e0.a == i ? e0.b : e0.a)];
            const Vec3 b = m.pts[static_cast<std::size_t>(e1.a == i ? e1.b : e1.a)];
            sharpV = (V * 6.0 + a + b) / 8.0;
        } else {
            sharpV = V;   // a corner
        }
        const double w = std::clamp(sharpSum / static_cast<double>(sharp.size()), 0.0, 1.0);
        vp[static_cast<std::size_t>(i)] = smoothV + (sharpV - smoothV) * w;
    }
    // vertex corners override: a corner stays put (sharpness >= 1), or blends toward it
    for (const auto& [i, s] : m.corners) {
        if (i < 0 || i >= nv) continue;
        const double w = std::clamp(static_cast<double>(s), 0.0, 1.0);
        vp[static_cast<std::size_t>(i)] = vp[static_cast<std::size_t>(i)] + (m.pts[static_cast<std::size_t>(i)] - vp[static_cast<std::size_t>(i)]) * w;
    }
    // assemble: vertex points, then edge points, then face points
    out.pts = vp;
    const int eBase = static_cast<int>(out.pts.size());
    out.pts.insert(out.pts.end(), ep.begin(), ep.end());
    const int fBase = static_cast<int>(out.pts.size());
    out.pts.insert(out.pts.end(), fp.begin(), fp.end());
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& v = m.faces[f].v;
        const std::size_t k = v.size();
        for (std::size_t i = 0; i < k; ++i) {
            const int vi = v[i];
            const int eNext = eBase + edgeIdx[ekey(v[i], v[(i + 1) % k])];
            const int ePrev = eBase + edgeIdx[ekey(v[(i + k - 1) % k], v[i])];
            PolyMesh::Face q;
            q.v = {vi, eNext, fBase + static_cast<int>(f), ePrev};
            q.group = m.faces[f].group;
            q.mat = m.faces[f].mat;
            q.color = m.faces[f].color;
            out.faces.push_back(q);
        }
    }
    for (const auto& [i, s] : m.corners)
        if (s > 1.0f) out.setCorner(i, s >= PolyMesh::kInfCrease ? PolyMesh::kInfCrease : s - 1.0f);
    // child creases: each half of a creased edge keeps sharpness - 1
    for (std::size_t e = 0; e < edges.size(); ++e) {
        const float s = edges[e].s;
        if (s <= 1.0f) continue;
        const float c = s >= PolyMesh::kInfCrease ? PolyMesh::kInfCrease : s - 1.0f;
        out.setCrease(edges[e].a, eBase + static_cast<int>(e), c);
        out.setCrease(edges[e].b, eBase + static_cast<int>(e), c);
    }
    return out;
}
}  // namespace

PolyMesh subdivide(const PolyMesh& m, int levels) {
    PolyMesh cur = m;
    for (int l = 0; l < levels; ++l) cur = subdivideOnce(cur);
    return cur;
}

// --- selection and checks -------------------------------------------------------------------------------

std::vector<int> selectAll(const PolyMesh& m) {
    std::vector<int> s(m.faces.size());
    for (std::size_t i = 0; i < s.size(); ++i) s[i] = static_cast<int>(i);
    return s;
}
std::vector<int> selectGroup(const PolyMesh& m, const std::string& group) {
    std::vector<int> s;
    const int g = m.findGroup(group);
    if (g < 0) return s;
    for (std::size_t i = 0; i < m.faces.size(); ++i) if (m.faces[i].group == g) s.push_back(static_cast<int>(i));
    return s;
}
std::vector<int> selectWhere(const PolyMesh& m, const std::function<bool(const Vec3&, const Vec3&)>& pred) {
    std::vector<int> s;
    for (std::size_t i = 0; i < m.faces.size(); ++i)
        if (pred(m.faceCentroid(static_cast<int>(i)), m.faceNormal(static_cast<int>(i)))) s.push_back(static_cast<int>(i));
    return s;
}

bool raycast(const PolyMesh& m, const Vec3& o, const Vec3& d, double& tOut, Vec3& nOut, int& fOut) {
    double best = 1e30;
    int hit = -1;
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& v = m.faces[f].v;
        for (std::size_t i = 1; i + 1 < v.size(); ++i) {
            const Vec3& a = m.pts[static_cast<std::size_t>(v[0])];
            const Vec3& b = m.pts[static_cast<std::size_t>(v[i])];
            const Vec3& c = m.pts[static_cast<std::size_t>(v[i + 1])];
            const Vec3 e1 = b - a, e2 = c - a, pv = cross(d, e2);
            const double det = dot(e1, pv);
            if (std::fabs(det) < 1e-12) continue;
            const double inv = 1.0 / det;
            const Vec3 tv = o - a;
            const double u = dot(tv, pv) * inv;
            if (u < 0 || u > 1) continue;
            const Vec3 qv = cross(tv, e1);
            const double w = dot(d, qv) * inv;
            if (w < 0 || u + w > 1) continue;
            const double t = dot(e2, qv) * inv;
            if (t > 1e-9 && t < best) { best = t; hit = static_cast<int>(f); }
        }
    }
    if (hit < 0) return false;
    tOut = best;
    nOut = m.faceNormal(hit);
    fOut = hit;
    return true;
}

PolyStats polyStats(const PolyMesh& m, double sliver) {
    PolyStats st;
    st.faces = static_cast<int>(m.faces.size());
    st.points = static_cast<int>(m.pts.size());
    st.corners = static_cast<int>(m.corners.size());
    st.creases = static_cast<int>(m.creases.size());
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& v = m.faces[f].v;
        st.triangles += static_cast<int>(v.size()) - 2;
        double per = 0;
        Vec3 acc(0, 0, 0);
        for (std::size_t i = 0; i < v.size(); ++i) {
            const Vec3& a = m.pts[static_cast<std::size_t>(v[i])];
            const Vec3& b = m.pts[static_cast<std::size_t>(v[(i + 1) % v.size()])];
            per += (b - a).length();
            acc = acc + cross(a, b);
        }
        const double area = 0.5 * std::fabs(dot(acc, m.faceNormal(static_cast<int>(f))));
        const double r = 4.0 * M_PI * area / std::max(1e-12, per * per);
        if (r < st.worst) { st.worst = r; st.worstFace = static_cast<int>(f); }
        st.slivers += r < sliver;
    }
    return st;
}

bool isClosed(const PolyMesh& m) {
    std::unordered_map<long long, int> de;
    for (const auto& f : m.faces)
        for (std::size_t i = 0; i < f.v.size(); ++i) ++de[dkey(f.v[i], f.v[(i + 1) % f.v.size()])];
    for (const auto& [k, c] : de) {
        if (c != 1) return false;
        const int a = static_cast<int>(k >> 32), b = static_cast<int>(k & 0xffffffff);
        const auto it = de.find(dkey(b, a));
        if (it == de.end() || it->second != 1) return false;
    }
    return true;
}

double signedVolume(const PolyMesh& m) {
    double v = 0;
    for (const auto& f : m.faces) {
        const Vec3& a = m.pts[static_cast<std::size_t>(f.v[0])];
        for (std::size_t i = 1; i + 1 < f.v.size(); ++i)
            v += dot(a, cross(m.pts[static_cast<std::size_t>(f.v[i])], m.pts[static_cast<std::size_t>(f.v[i + 1])])) / 6.0;
    }
    return v;
}

// --- to the renderer ------------------------------------------------------------------------------------

std::map<std::string, RenderMesh> toParts(const PolyMesh& m, double autosmoothDeg, double uvScale) {
    std::map<std::string, RenderMesh> out;
    const double cosLim = std::cos(autosmoothDeg * M_PI / 180.0);
    std::vector<Vec3> fn(m.faces.size());
    for (std::size_t f = 0; f < m.faces.size(); ++f) fn[f] = m.faceNormal(static_cast<int>(f));
    // faces around each vertex
    std::vector<std::vector<int>> vFaces(m.pts.size());
    for (std::size_t f = 0; f < m.faces.size(); ++f)
        for (int i : m.faces[f].v) vFaces[static_cast<std::size_t>(i)].push_back(static_cast<int>(f));
    auto hardEdge = [&](int a, int b) { return m.crease(a, b) >= PolyMesh::kInfCrease; };
    auto sharesEdgeHard = [&](int f, int g, int vi) {   // are f and g separated at vi by a hard edge they share?
        const auto& A = m.faces[static_cast<std::size_t>(f)].v;
        const auto& B = m.faces[static_cast<std::size_t>(g)].v;
        for (std::size_t i = 0; i < A.size(); ++i) {
            const int a = A[i], b = A[(i + 1) % A.size()];
            if (a != vi && b != vi) continue;
            const int o = a == vi ? b : a;
            if (std::find(B.begin(), B.end(), o) != B.end() && hardEdge(vi, o)) return true;
        }
        return false;
    };
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& face = m.faces[f];
        if (face.v.size() < 3) continue;
        RenderMesh& rm = out[m.mats[static_cast<std::size_t>(face.mat)]];
        const uint32_t base = static_cast<uint32_t>(rm.vertices.size());
        // box-projected UV frame from the face normal
        const Vec3 n0 = fn[f];
        const double ax = std::fabs(n0.x), ay = std::fabs(n0.y), az = std::fabs(n0.z);
        Vec3 tu, tv;
        if (ax >= ay && ax >= az) { tu = Vec3(0, 0, n0.x > 0 ? -1 : 1); tv = Vec3(0, 1, 0); }
        else if (ay >= az)        { tu = Vec3(1, 0, 0); tv = Vec3(0, 0, n0.y > 0 ? -1 : 1); }
        else                      { tu = Vec3(n0.z > 0 ? 1 : -1, 0, 0); tv = Vec3(0, 1, 0); }
        for (int vi : face.v) {
            Vec3 n(0, 0, 0);
            for (int g : vFaces[static_cast<std::size_t>(vi)]) {
                if (m.faces[static_cast<std::size_t>(g)].mat != face.mat) continue;
                if (dot(fn[static_cast<std::size_t>(g)], n0) < cosLim) continue;
                if (g != static_cast<int>(f) && sharesEdgeHard(static_cast<int>(f), g, vi)) continue;
                n = n + fn[static_cast<std::size_t>(g)];
            }
            if (n.lengthSquared() < 1e-12) n = n0;
            n = normalize(n);
            const Vec3 p = m.pts[static_cast<std::size_t>(vi)];
            Vertex vx(p, n, static_cast<float>(dot(p, tu) / uvScale), static_cast<float>(dot(p, tv) / uvScale));
            Vec3 t = tu - n * dot(tu, n);
            vx.tangent = t.lengthSquared() > 1e-12 ? normalize(t) : Vec3(1, 0, 0);
            vx.color = face.color;
            rm.vertices.push_back(vx);
        }
        // fan, in the engine's winding (outward = cross(c - a, b - a)): emit (0, i+1, i)
        for (std::size_t i = 1; i + 1 < face.v.size(); ++i) {
            rm.indices.push_back(base);
            rm.indices.push_back(base + static_cast<uint32_t>(i + 1));
            rm.indices.push_back(base + static_cast<uint32_t>(i));
        }
    }
    return out;
}

}  // namespace engine
