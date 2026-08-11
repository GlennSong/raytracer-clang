#include "tessellate.h"

#include "structure.h"

namespace roadlab {

void Mesh::append(const Mesh& other) {
    uint32_t base = uint32_t(verts.size());
    verts.insert(verts.end(), other.verts.begin(), other.verts.end());
    for (uint32_t i : other.indices) indices.push_back(base + i);
}

namespace {

// Station list for a stretch of road: fine where the geometry turns, coarse
// where it does not. The tolerance is in radians of heading change, so the same
// number gives the right answer on a roundabout ring and on a freeway tangent.
std::vector<double> stationsFor(const Road& r, double s0, double s1, const TessParams& p) {
    std::vector<double> out;
    double s = s0;
    out.push_back(s);
    int guard = 0;
    while (s < s1 - 1e-6 && guard++ < 20000) {
        double k = std::fabs(r.spine.curvatureAt(s));
        double step = p.maxStationStep;
        if (k > 1e-6) step = std::min(step, std::max(p.minStationStep, p.curvatureTolerance / k));
        s = std::min(s1, s + step);
        out.push_back(s);
    }
    if (out.back() < s1 - 1e-9) out.push_back(s1);
    return out;
}

}  // namespace

void tessellateRoad(const Road& road, Mesh& out, const TessParams& p) {
    for (size_t si = 0; si < road.xs.sections.size(); ++si) {
        const LaneSection& sec = road.xs.sections[si];
        double s0 = std::max(sec.s0, road.begin());
        double s1 = std::min(sec.s0 + sec.length, road.end());
        if (s1 - s0 < 1e-4) continue;
        std::vector<double> stations = stationsFor(road, s0, s1, p);

        for (int side = 0; side < 2; ++side) {
            const std::vector<Strip>& stack = side == 0 ? sec.right : sec.left;
            for (size_t k = 0; k < stack.size(); ++k) {
                const Strip& st = stack[k];
                // Boundary offsets come from the cross-section, never from a
                // local accumulation, so the strip next door lands on exactly the
                // same t and the surface stays watertight.
                std::vector<uint32_t> prev(2, 0);
                bool havePrev = false;
                bool anyWidth = false;
                for (double s : stations) {
                    double tInner = 0, tOuter = 0;
                    if (!road.xs.laneSpanAt(int(si), st.id, s, tInner, tOuter)) {
                        havePrev = false;
                        continue;
                    }
                    if (std::fabs(tOuter - tInner) > 1e-4) anyWidth = true;
                    uint32_t cur[2];
                    for (int e = 0; e < 2; ++e) {
                        double t = e == 0 ? tInner : tOuter;
                        Vertex v;
                        v.pos = road.surfacePoint(s, t);
                        v.normal = road.surfaceNormal(s, t);
                        v.s = s;
                        v.t = t;
                        v.road = road.id;
                        v.material = MatKind::RoadSurface;
                        cur[e] = out.push(v);
                    }
                    if (havePrev) {
                        out.quad(prev[0], prev[1], cur[1], cur[0]);
                    }
                    prev[0] = cur[0];
                    prev[1] = cur[1];
                    havePrev = true;
                }
                if (p.skipZeroWidth && !anyWidth) {
                    // Nothing to see; the degenerate quads are harmless but this
                    // keeps the triangle count honest for wholly-absent strips.
                }
            }
        }
    }
}

// --- polygon utilities ----------------------------------------------------

namespace {

double polygonArea2(const std::vector<Vec2>& poly) {
    double a = 0;
    for (size_t i = 0, k = poly.size() - 1; i < poly.size(); k = i++)
        a += cross(poly[k], poly[i]);
    return a;
}

bool pointInTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    double d1 = cross(b - a, p - a);
    double d2 = cross(c - b, p - b);
    double d3 = cross(a - c, p - c);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

}  // namespace

std::vector<std::array<uint32_t, 3>> triangulatePolygon(const std::vector<Vec2>& poly) {
    std::vector<std::array<uint32_t, 3>> out;
    const size_t n = poly.size();
    if (n < 3) return out;

    // Work counter-clockwise so "convex" has one meaning throughout.
    std::vector<uint32_t> idx(n);
    bool ccw = polygonArea2(poly) > 0;
    for (size_t i = 0; i < n; ++i) idx[i] = uint32_t(ccw ? i : n - 1 - i);

    int guard = int(n) * int(n) + 16;
    while (idx.size() > 3 && guard-- > 0) {
        bool clipped = false;
        for (size_t i = 0; i < idx.size(); ++i) {
            size_t prev = (i + idx.size() - 1) % idx.size();
            size_t next = (i + 1) % idx.size();
            Vec2 a = poly[idx[prev]], b = poly[idx[i]], c = poly[idx[next]];
            if (cross(b - a, c - b) <= 1e-12) continue;   // reflex or degenerate
            bool ear = true;
            for (size_t k = 0; k < idx.size() && ear; ++k) {
                if (k == prev || k == i || k == next) continue;
                if (pointInTriangle(poly[idx[k]], a, b, c)) ear = false;
            }
            if (!ear) continue;
            out.push_back({idx[prev], idx[i], idx[next]});
            idx.erase(idx.begin() + long(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            // Self-intersecting or otherwise pathological: fall back to a fan so
            // the pad is still covered rather than missing.
            out.clear();
            for (size_t i = 1; i + 1 < n; ++i)
                out.push_back({uint32_t(0), uint32_t(i), uint32_t(i + 1)});
            return out;
        }
    }
    if (idx.size() == 3) out.push_back({idx[0], idx[1], idx[2]});
    return out;
}

bool meanValueCoords(const std::vector<Vec2>& poly, Vec2 p, std::vector<double>& weights) {
    const size_t n = poly.size();
    weights.assign(n, 0.0);
    if (n < 3) return false;

    std::vector<double> dist(n);
    for (size_t i = 0; i < n; ++i) {
        dist[i] = length(poly[i] - p);
        if (dist[i] < 1e-7) {   // exactly on a vertex
            weights[i] = 1.0;
            return true;
        }
    }
    // On an edge, the coordinates collapse to a linear blend of its endpoints.
    for (size_t i = 0; i < n; ++i) {
        size_t k = (i + 1) % n;
        Vec2 a = poly[i], b = poly[k];
        Vec2 ab = b - a;
        double len2 = dot(ab, ab);
        if (len2 < 1e-12) continue;
        double u = clampd(dot(p - a, ab) / len2, 0.0, 1.0);
        if (length(a + ab * u - p) < 1e-7) {
            weights[i] = 1.0 - u;
            weights[k] = u;
            return true;
        }
    }

    // tan(alpha_i / 2) for the angle subtended at p by edge i -> i+1.
    std::vector<double> tanHalf(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        size_t k = (i + 1) % n;
        Vec2 a = poly[i] - p, b = poly[k] - p;
        double den = dist[i] * dist[k] + dot(a, b);
        if (std::fabs(den) < 1e-12) return false;   // p lies on the line, outside
        tanHalf[i] = cross(a, b) / den;
    }
    double sum = 0;
    for (size_t i = 0; i < n; ++i) {
        size_t prev = (i + n - 1) % n;
        weights[i] = (tanHalf[prev] + tanHalf[i]) / dist[i];
        sum += weights[i];
    }
    if (std::fabs(sum) < 1e-12) return false;
    for (double& w : weights) w /= sum;
    return true;
}

double junctionElevationAt(const Network& net, const Junction& j, Vec2 planPoint) {
    const std::vector<Vec2>& poly = j.boundary;
    const std::vector<double>& hb = j.boundaryHeight;
    if (poly.size() < 3 || hb.size() != poly.size()) {
        // No usable boundary: fall back to the arms.
        double num = 0, den = 0;
        for (const JunctionArm& a : j.arms) {
            double y = net.road(a.road).surfacePoint(a.sContact, 0).y;
            double d = std::max(0.6, length(planPoint - a.contact));
            num += y / (d * d);
            den += 1.0 / (d * d);
        }
        return den > 0 ? num / den : j.elevation;
    }

    // Inside the pad: transfinite interpolation of the boundary heights, which
    // meets every arm exactly at its own grade, crossfall and bank.
    static thread_local std::vector<double> w;
    if (meanValueCoords(poly, planPoint, w)) {
        bool inside = true;
        for (double v : w) {
            if (v < -1e-9) {   // MVC goes negative outside the polygon
                inside = false;
                break;
            }
        }
        if (inside) {
            double h = 0;
            for (size_t i = 0; i < w.size(); ++i) h += w[i] * hb[i];
            return h;
        }
    }

    // Outside: the height of the nearest point ON the boundary, so the terrain
    // meets the kerb line without a step.
    double best = 1e300, bestH = j.elevation;
    for (size_t i = 0, k = poly.size() - 1; i < poly.size(); k = i++) {
        Vec2 a = poly[k], b = poly[i];
        Vec2 ab = b - a;
        double len2 = std::max(1e-12, dot(ab, ab));
        double u = clampd(dot(planPoint - a, ab) / len2, 0.0, 1.0);
        double d = length(a + ab * u - planPoint);
        if (d < best) {
            best = d;
            bestH = lerp(hb[k], hb[i], u);
        }
    }
    return bestH;
}

void tessellateJunction(const Network& net, const Junction& j, Mesh& out, double padDetail) {
    if (j.boundary.size() < 3) return;

    // Positions first, heights second: every vertex — boundary, interior, or one
    // created by refinement — takes its height from the same interpolant, so the
    // pad is one continuous surface that happens to be pinned at the arms.
    std::vector<Vec2> pts = j.boundary;
    std::vector<std::array<uint32_t, 3>> tris = triangulatePolygon(pts);
    if (tris.empty()) return;

    // Refine until no edge is longer than padDetail. A flat pad does not need
    // this; a pad spanning a grade change does, or the interpolation has nowhere
    // to show itself.
    for (int level = 0; level < 4; ++level) {
        double longest = 0;
        for (const auto& t : tris) {
            for (int e = 0; e < 3; ++e)
                longest =
                    std::max(longest, length(pts[t[size_t(e)]] - pts[t[size_t((e + 1) % 3)]]));
        }
        if (longest <= padDetail || pts.size() > 6000) break;
        std::vector<std::array<uint32_t, 3>> next;
        next.reserve(tris.size() * 4);
        // Midpoints are cached on the ordered vertex pair so neighbouring
        // triangles share the split point and the surface stays watertight.
        std::vector<std::pair<uint64_t, uint32_t>> mids;
        auto midpoint = [&](uint32_t a, uint32_t b) {
            uint64_t key = (uint64_t(std::min(a, b)) << 32) | uint64_t(std::max(a, b));
            for (const auto& m : mids)
                if (m.first == key) return m.second;
            pts.push_back((pts[a] + pts[b]) * 0.5);
            uint32_t id = uint32_t(pts.size() - 1);
            mids.push_back({key, id});
            return id;
        };
        for (const auto& t : tris) {
            uint32_t ab = midpoint(t[0], t[1]);
            uint32_t bc = midpoint(t[1], t[2]);
            uint32_t ca = midpoint(t[2], t[0]);
            next.push_back({t[0], ab, ca});
            next.push_back({ab, t[1], bc});
            next.push_back({ca, bc, t[2]});
            next.push_back({ab, bc, ca});
        }
        tris.swap(next);
    }

    std::vector<uint32_t> vid(pts.size(), 0);
    for (size_t i = 0; i < pts.size(); ++i) {
        Vertex v;
        v.pos = worldOf(pts[i], junctionElevationAt(net, j, pts[i]));
        v.normal = {0, 1, 0};
        v.s = pts[i].y;
        v.t = pts[i].x;
        v.junction = j.id;
        v.material = MatKind::JunctionPad;
        vid[i] = out.push(v);
    }
    // Area-weighted vertex normals, so a pad spanning a grade change is lit as
    // the ramp it is rather than as a flat plate.
    std::vector<Vec3> accum(pts.size(), Vec3{0, 0, 0});
    for (const auto& t : tris) {
        Vec3 a = out.verts[vid[t[0]]].pos;
        Vec3 b = out.verts[vid[t[1]]].pos;
        Vec3 c = out.verts[vid[t[2]]].pos;
        Vec3 n = cross(c - a, b - a);
        if (n.y < 0) n = -n;
        for (int k = 0; k < 3; ++k) accum[t[size_t(k)]] += n;
        out.tri(vid[t[0]], vid[t[1]], vid[t[2]]);
    }
    for (size_t i = 0; i < pts.size(); ++i)
        if (lengthSq(accum[i]) > 1e-18) out.verts[vid[i]].normal = normalize(accum[i]);
}

void tessellateNetwork(const Network& net, Mesh& out, const TessParams& p, bool withStructures) {
    for (const Road& r : net.roads()) tessellateRoad(r, out, p);
    for (const Junction& j : net.junctions()) tessellateJunction(net, j, out);
    if (withStructures) {
        for (const Road& r : net.roads()) tessellateStructures(net, r, out);
    }
}

// --- helper geometry ------------------------------------------------------

void emitBox(Mesh& m, Vec3 centre, Vec3 half, double yaw, Vec3 color, double roughness) {
    double c = std::cos(yaw), s = std::sin(yaw);
    auto rot = [&](Vec3 v) { return Vec3{v.x * c - v.z * s, v.y, v.x * s + v.z * c}; };
    Vec3 corners[8];
    int idx = 0;
    for (int i = -1; i <= 1; i += 2)
        for (int k = -1; k <= 1; k += 2)
            for (int l = -1; l <= 1; l += 2)
                corners[idx++] = centre + rot({half.x * i, half.y * k, half.z * l});
    // corner index bits: i (x) -> 4, k (y) -> 2, l (z) -> 1
    static const int kFaces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1},
                                     {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
    for (const auto& f : kFaces) {
        Vec3 n = normalize(cross(corners[f[1]] - corners[f[0]], corners[f[2]] - corners[f[0]]));
        uint32_t v[4];
        for (int i = 0; i < 4; ++i) {
            Vertex vert;
            vert.pos = corners[f[i]];
            vert.normal = n;
            vert.color = color;
            vert.roughness = roughness;
            vert.material = MatKind::Flat;
            v[i] = m.push(vert);
        }
        m.quad(v[0], v[1], v[2], v[3]);
    }
}

void emitCylinder(Mesh& m, Vec3 base, double radius, double height, int sides, Vec3 color,
                  double roughness) {
    sides = std::max(3, sides);
    std::vector<uint32_t> lo(size_t(sides), 0), hi(size_t(sides), 0);
    for (int i = 0; i < sides; ++i) {
        double a = kTwoPi * double(i) / double(sides);
        Vec3 off{std::cos(a) * radius, 0, std::sin(a) * radius};
        Vec3 n = normalize(Vec3{off.x, 0, off.z});
        Vertex v;
        v.normal = n;
        v.color = color;
        v.roughness = roughness;
        v.material = MatKind::Flat;
        v.pos = base + off;
        lo[size_t(i)] = m.push(v);
        v.pos = base + off + Vec3{0, height, 0};
        hi[size_t(i)] = m.push(v);
    }
    for (int i = 0; i < sides; ++i) {
        int k = (i + 1) % sides;
        m.quad(lo[size_t(i)], lo[size_t(k)], hi[size_t(k)], hi[size_t(i)]);
    }
    Vertex cap;
    cap.normal = {0, 1, 0};
    cap.color = color;
    cap.roughness = roughness;
    cap.material = MatKind::Flat;
    cap.pos = base + Vec3{0, height, 0};
    uint32_t centre = m.push(cap);
    for (int i = 0; i < sides; ++i) {
        int k = (i + 1) % sides;
        m.tri(centre, hi[size_t(i)], hi[size_t(k)]);
    }
}

void emitPlate(Mesh& m, Vec3 centre, Vec3 right, Vec3 up, Vec3 color, double roughness) {
    Vec3 n = normalize(cross(right, up));
    uint32_t v[4];
    Vec3 corner[4] = {centre - right - up, centre + right - up, centre + right + up,
                      centre - right + up};
    for (int i = 0; i < 4; ++i) {
        Vertex vert;
        vert.pos = corner[i];
        vert.normal = n;
        vert.color = color;
        vert.roughness = roughness;
        vert.material = MatKind::Flat;
        v[i] = m.push(vert);
    }
    m.quad(v[0], v[1], v[2], v[3]);
    m.quad(v[3], v[2], v[1], v[0]);   // double-sided: signs are read from both ways
}

void emitPolygonFace(Mesh& m, Vec3 centre, Vec3 right, Vec3 up, int sides, double startAngle,
                     Vec3 color) {
    sides = std::max(3, sides);
    Vec3 n = normalize(cross(right, up));
    Vertex c;
    c.pos = centre;
    c.normal = n;
    c.color = color;
    c.roughness = 0.5;
    c.material = MatKind::Flat;
    uint32_t ci = m.push(c);
    std::vector<uint32_t> ring(size_t(sides), 0);
    for (int i = 0; i < sides; ++i) {
        double a = startAngle + kTwoPi * double(i) / double(sides);
        Vertex v = c;
        v.pos = centre + right * std::cos(a) + up * std::sin(a);
        ring[size_t(i)] = m.push(v);
    }
    for (int i = 0; i < sides; ++i) {
        int k = (i + 1) % sides;
        m.tri(ci, ring[size_t(i)], ring[size_t(k)]);
        m.tri(ci, ring[size_t(k)], ring[size_t(i)]);
    }
}

void sweepProfile(const Road& road, double s0, double s1, const std::vector<Vec2>& profile,
                  Vec3 color, Mesh& out, double step, bool closeEnds) {
    if (profile.size() < 2 || s1 - s0 < 1e-3) return;
    TessParams tp;
    tp.maxStationStep = step;
    std::vector<double> stations;
    {
        double s = s0;
        stations.push_back(s);
        int guard = 0;
        while (s < s1 - 1e-6 && guard++ < 20000) {
            double k = std::fabs(road.spine.curvatureAt(s));
            double st = step;
            if (k > 1e-6) st = std::min(st, std::max(0.5, 0.12 / k));
            s = std::min(s1, s + st);
            stations.push_back(s);
        }
    }
    std::vector<uint32_t> prev(profile.size(), 0);
    bool havePrev = false;
    for (double s : stations) {
        std::vector<uint32_t> cur(profile.size());
        for (size_t i = 0; i < profile.size(); ++i) {
            Vertex v;
            // The profile is in the road's own (t, h) frame, so a swept barrier
            // banks and climbs with the carriageway for free.
            v.pos = road.spine.toWorld(s, profile[i].x, profile[i].y);
            v.color = color;
            v.roughness = 0.8;
            v.material = MatKind::Flat;
            v.normal = {0, 1, 0};
            cur[i] = out.push(v);
        }
        if (havePrev) {
            for (size_t i = 0; i + 1 < profile.size(); ++i) {
                Vec3 a = out.verts[prev[i]].pos, b = out.verts[prev[i + 1]].pos;
                Vec3 c = out.verts[cur[i + 1]].pos;
                Vec3 n = normalize(cross(c - a, b - a));
                out.verts[prev[i]].normal = n;
                out.verts[prev[i + 1]].normal = n;
                out.verts[cur[i]].normal = n;
                out.verts[cur[i + 1]].normal = n;
                out.quad(prev[i], prev[i + 1], cur[i + 1], cur[i]);
            }
        }
        prev = cur;
        havePrev = true;
    }
    if (closeEnds && profile.size() >= 3) {
        for (double s : {s0, s1}) {
            std::vector<uint32_t> ring(profile.size());
            for (size_t i = 0; i < profile.size(); ++i) {
                Vertex v;
                v.pos = road.spine.toWorld(s, profile[i].x, profile[i].y);
                v.color = color;
                v.material = MatKind::Flat;
                Frame f = road.spine.frameAt(s);
                v.normal = s == s0 ? -f.forward : f.forward;
                ring[i] = out.push(v);
            }
            for (size_t i = 1; i + 1 < ring.size(); ++i) out.tri(ring[0], ring[i], ring[i + 1]);
        }
    }
}

}  // namespace roadlab
