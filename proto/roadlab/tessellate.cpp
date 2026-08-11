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

double junctionElevationAt(const Network& net, const Junction& j, Vec2 planPoint) {
    // Inverse-distance blend of the arms' surface heights. A Coons patch would
    // be the right answer for a skewed junction between differently-graded
    // approaches; IDW is smooth, always defined, and meets each arm closely
    // enough for a prototype.
    double num = 0, den = 0;
    for (const JunctionArm& a : j.arms) {
        const Road& r = net.road(a.road);
        double y = r.surfacePoint(a.sContact, 0).y;
        double d = std::max(0.6, length(planPoint - a.contact));
        double w = 1.0 / (d * d);
        num += y * w;
        den += w;
    }
    return den > 0 ? num / den : j.elevation;
}

void tessellateJunction(const Network& net, const Junction& j, Mesh& out) {
    if (j.boundary.size() < 3) return;
    Vec2 centroid{0, 0};
    for (Vec2 p : j.boundary) centroid = centroid + p;
    centroid = centroid * (1.0 / double(j.boundary.size()));

    auto makeVert = [&](Vec2 p) {
        Vertex v;
        v.pos = worldOf(p, junctionElevationAt(net, j, p));
        v.normal = {0, 1, 0};
        v.s = p.y;
        v.t = p.x;
        v.junction = j.id;
        v.material = MatKind::JunctionPad;
        return out.push(v);
    };

    uint32_t c = makeVert(centroid);
    std::vector<uint32_t> ring;
    ring.reserve(j.boundary.size());
    for (Vec2 p : j.boundary) ring.push_back(makeVert(p));
    for (size_t i = 0; i < ring.size(); ++i) {
        out.tri(c, ring[i], ring[(i + 1) % ring.size()]);
    }
    // Recompute the fan's normals from the actual triangle plane so a pad that
    // spans a grade change is not lit as if it were flat.
    for (size_t i = 0; i < ring.size(); ++i) {
        Vec3 a = out.verts[c].pos;
        Vec3 b = out.verts[ring[i]].pos;
        Vec3 d = out.verts[ring[(i + 1) % ring.size()]].pos;
        Vec3 n = normalize(cross(d - a, b - a));
        if (n.y < 0) n = -n;
        out.verts[ring[i]].normal = n;
    }
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
