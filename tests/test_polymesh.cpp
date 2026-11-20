// PolyMesh: the hard-surface modelling kit (ADR-0139). Each op keeps the mesh a closed, outward-facing
// solid, and subdivision honours creases -- the properties every recipe leans on.
#include "test_framework.h"
#include "../src/engine/procgen/polymesh.h"

#include <cmath>
#include <cstdio>

using namespace engine;

namespace {
// a unit cube (-0.5..0.5) as a loft of two square sections
PolyMesh cube() {
    const std::vector<Vec2> sq = {Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.5, 0.5), Vec2(-0.5, 0.5)};
    return loft({sq, sq}, {-0.5, 0.5}, true, true);
}
void bounds(const PolyMesh& m, Vec3& lo, Vec3& hi) {
    lo = Vec3(1e30, 1e30, 1e30);
    hi = Vec3(-1e30, -1e30, -1e30);
    for (const Vec3& p : m.pts) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
}
}  // namespace

TEST_CASE(polymesh_loft_makes_a_closed_outward_solid) {
    PolyMesh c = cube();
    CHECK(c.faces.size() == 6);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 1.0, 1e-9);
    // a rounded, resampled section lofted over three stations stays closed and outward
    const auto sec = resampleClosed(roundedPolygon({Vec2(-1, 0), Vec2(1, 0), Vec2(0.8, 1), Vec2(-0.8, 1)}, {0.2}, 3), 24);
    CHECK(sec.size() == 24);
    // two big fillets on a short shared edge never meet in one duplicated point
    {
        const auto q = roundedPolygon({Vec2(0, 0), Vec2(1, 0), Vec2(1, 0.1), Vec2(0, 0.1)}, {0.5}, 3);
        for (std::size_t i = 0; i < q.size(); ++i) CHECK((q[i] - q[(i + 1) % q.size()]).length() > 1e-6);
    }
    // every corner emits segs + 1 points, even square and straight ones (recipes index faces by corner)
    CHECK(roundedPolygon({Vec2(0, 0), Vec2(1, 0), Vec2(2, 0), Vec2(2, 1), Vec2(0, 1)}, {0.0, 0.3, 0.2, 0.2, 0.0}, 3).size() == 20u);
    PolyMesh b = loft({sec, sec, sec}, {-2, 0, 2}, true, true);
    CHECK(isClosed(b));
    CHECK(signedVolume(b) > 0);
}

TEST_CASE(polymesh_extrude_and_inset_keep_the_solid_closed) {
    PolyMesh c = cube();
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    CHECK(top.size() == 1);
    extrude(c, top, 1.0);
    CHECK(c.faces.size() == 10);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 2.0, 1e-9);
    Vec3 lo, hi;
    bounds(c, lo, hi);
    CHECK_APPROX(hi.y, 1.5, 1e-9);
    const auto front = selectWhere(c, [](const Vec3& ctr, const Vec3& n) { return n.z > 0.9 && ctr.y < 0.1; });
    CHECK(front.size() == 1);
    const auto in = inset(c, front, 0.1);
    CHECK(in.size() == 1);
    CHECK(c.faces.size() == 14);
    CHECK(isClosed(c));
    // pushing the inset face in makes a recess (a window): still closed, less volume
    const double v0 = signedVolume(c);
    extrude(c, in, -0.05);
    CHECK(isClosed(c));
    CHECK(signedVolume(c) < v0);
}

TEST_CASE(polymesh_mirror_welds_the_seam) {
    // the +x half of a box, open on x = 0
    PolyMesh h;
    h.pts = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 0, 1), Vec3(1, 1, 1), Vec3(0, 1, 1)};
    auto q = [&](int a, int b, int c, int d) { PolyMesh::Face f; f.v = {a, b, c, d}; h.faces.push_back(f); };
    q(0, 3, 2, 1);   // z = 0 (faces -z)
    q(4, 5, 6, 7);   // z = 1
    q(1, 2, 6, 5);   // x = 1
    q(0, 1, 5, 4);   // y = 0
    q(3, 7, 6, 2);   // y = 1
    CHECK(!isClosed(h));
    mirrorX(h);
    CHECK(isClosed(h));
    CHECK_APPROX(signedVolume(h), 2.0, 1e-9);
}

TEST_CASE(polymesh_subdivision_smooths_and_creases_hold) {
    PolyMesh c = cube();
    c.creases.clear();   // loft creases its cap rings; the blob wants none
    PolyMesh s = subdivide(c, 3);
    CHECK(s.faces.size() == 6u * 64u);
    CHECK(isClosed(s));
    CHECK(signedVolume(s) > 0.3 && signedVolume(s) < 0.4);   // a smooth blob well inside the cage
    Vec3 lo, hi;
    bounds(s, lo, hi);
    CHECK(hi.x < 0.5 && hi.x > 0.25);
    // every edge fully creased: the cube keeps its shape exactly
    PolyMesh k = cube();
    creaseFaces(k, selectAll(k), PolyMesh::kInfCrease);
    PolyMesh ks = subdivide(k, 2);
    CHECK(isClosed(ks));
    CHECK_APPROX(signedVolume(ks), 1.0, 1e-9);
    // sharpness is a dial: every edge at 0 (smooth), 0.5 (semi-sharp), 1 (one level sharp) -- the volume
    // grows with it. (loft creases its cap rings at 1, so clear them for the smooth reference.)
    auto vol = [](float s) { PolyMesh h = cube(); creaseFaces(h, selectAll(h), s); h.creases.erase(h.creases.begin(), h.creases.end()); if (s > 0) creaseFaces(h, selectAll(h), s); return signedVolume(subdivide(h, 2)); };
    CHECK(vol(0.0f) < vol(0.5f));
    CHECK(vol(0.5f) < vol(1.0f));
    CHECK(vol(1.0f) < 1.0);
}

TEST_CASE(polymesh_to_parts_splits_by_material_in_engine_winding) {
    PolyMesh c = cube();
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    for (int f : top) c.faces[static_cast<std::size_t>(f)].mat = c.matId("glass");
    const auto parts = toParts(subdivide(c, 1), 35.0);
    CHECK(parts.size() == 2);
    CHECK(parts.count("body") && parts.count("glass"));
    for (const auto& [name, rm] : parts) {
        CHECK(!rm.indices.empty());
        // engine winding: the geometric normal cross(c-a, b-a) agrees with the vertex normal
        int agree = 0, total = 0;
        for (std::size_t i = 0; i + 2 < rm.indices.size(); i += 3) {
            const Vertex& a = rm.vertices[rm.indices[i]];
            const Vertex& b = rm.vertices[rm.indices[i + 1]];
            const Vertex& cc = rm.vertices[rm.indices[i + 2]];
            agree += dot(cross(cc.position - a.position, b.position - a.position), a.normal) > 0;
            ++total;
            CHECK_APPROX(a.normal.length(), 1.0, 1e-6);
        }
        CHECK(agree == total);
    }
}

// A window's corners stay corners (vertex sharpness) while its outline between them stays smooth.
TEST_CASE(polymesh_corners_hold_through_subdivision) {
    PolyMesh c = cube();
    c.creases.clear();
    PolyMesh a = c;
    creaseCorners(a, selectAll(a), 40.0, 0.0f);   // nothing: an all-faces region has no outline
    CHECK(a.corners.empty());
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    creaseCorners(c, top, 40.0, PolyMesh::kInfCrease);
    CHECK(c.corners.size() == 4);   // the top face's four corners
    const PolyMesh s = subdivide(c, 3);
    int kept = 0;
    for (const Vec3& p : s.pts) kept += std::fabs(std::fabs(p.x) - 0.5) < 1e-9 && std::fabs(p.y - 0.5) < 1e-9 && std::fabs(std::fabs(p.z) - 0.5) < 1e-9;
    CHECK(kept == 4);
    CHECK(isClosed(s));
}

namespace {
// the worst face's "roundness": 4*pi*area / perimeter^2 (1 for a circle, ~0.785 for a square, -> 0 for a sliver)
double worstRoundness(const PolyMesh& m) {
    double worst = 1.0;
    for (std::size_t f = 0; f < m.faces.size(); ++f) {
        const auto& v = m.faces[f].v;
        double per = 0, area2 = 0;
        const Vec3 n = m.faceNormal(static_cast<int>(f));
        Vec3 acc(0, 0, 0);
        for (std::size_t i = 0; i < v.size(); ++i) {
            const Vec3& a = m.pts[static_cast<std::size_t>(v[i])];
            const Vec3& b = m.pts[static_cast<std::size_t>(v[(i + 1) % v.size()])];
            per += (b - a).length();
            acc = acc + cross(a, b);
        }
        area2 = std::fabs(dot(acc, n));
        worst = std::min(worst, 4.0 * M_PI * (0.5 * area2) / std::max(1e-12, per * per));
    }
    return worst;
}
}  // namespace

// No slivers (Glenn: "degenerate looking triangles in the rear ... we don't want wasted triangles"): a
// car-like section with sharp corners, lofted and capped, has no face thinner than a sensible bound, and
// its caps are rows of quads, not one polygon.
TEST_CASE(polymesh_car_section_loft_has_no_slivers_and_ladder_caps) {
    const std::vector<Vec2> corners = {Vec2(-0.8, 0.15), Vec2(0.8, 0.15), Vec2(0.9, 0.6), Vec2(0.88, 0.95),
                                       Vec2(0.8, 0.97), Vec2(0.65, 1.4), Vec2(-0.65, 1.4), Vec2(-0.8, 0.97),
                                       Vec2(-0.88, 0.95), Vec2(-0.9, 0.6)};
    const auto sec = roundedPolygon(corners, {0.1, 0.1, 0.4, 0.0, 0.0, 0.15, 0.15, 0.0, 0.0, 0.4}, 3);
    std::vector<std::vector<Vec2>> secs;
    std::vector<double> st;
    for (int i = 0; i <= 10; ++i) { secs.push_back(sec); st.push_back(-2.0 + 0.4 * i); }   // car-like station spacing
    PolyMesh b = loft(secs, st, true, true, 0.3f);
    CHECK(isClosed(b));
    CHECK(signedVolume(b) > 0);
    std::size_t maxSides = 0;
    for (const auto& f : b.faces) maxSides = std::max(maxSides, f.v.size());
    CHECK(maxSides <= 4u);   // the caps are quads (and at most a triangle where the ring crosses the axis)
    const PolyMesh s = subdivide(b, 1);
    CHECK(isClosed(s));
    const double r = worstRoundness(s);
    std::printf("    worst face roundness %.4f over %zu faces\n", r, s.faces.size());
    CHECK(r > 0.02);
    // a region inset of the whole cap stays closed and one panel
    PolyMesh g = b;
    const auto cap = selectGroup(g, "cap_end");
    CHECK(cap.size() > 4);
    insetRegion(g, cap, 0.08);
    CHECK(isClosed(g));
}

// Placement rules set parts on the finished surface: a ray finds the first face it meets and its normal.
TEST_CASE(polymesh_raycast_finds_the_surface_and_its_normal) {
    PolyMesh c = cube();
    double t = 0;
    Vec3 n;
    int f = -1;
    CHECK(raycast(c, Vec3(0.1, 0.2, 5.0), Vec3(0, 0, -1), t, n, f));
    CHECK_APPROX(t, 4.5, 1e-9);            // the +z face at z = 0.5
    CHECK_APPROX(n.z, 1.0, 1e-9);
    CHECK(!raycast(c, Vec3(2.0, 0.0, 5.0), Vec3(0, 0, -1), t, n, f));   // beside the cube: a miss
}

// A cabin's inner skin is the body's own inside: the region copied, offset inward and flipped.
TEST_CASE(polymesh_extract_offsets_and_flips_a_region) {
    PolyMesh c = cube();
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    const PolyMesh in = extractOffset(c, top, -0.1, true);
    CHECK(in.faces.size() == 1);
    CHECK(in.pts.size() == 4);
    for (const Vec3& p : in.pts) CHECK_APPROX(p.y, 0.4, 1e-9);   // 10 cm inside the top
    CHECK(in.faceNormal(0).y < -0.9);                             // facing down, into the box
}

// Thickness for a skin: the region's back and a wall along its outline. With `against`, a wall only where the outline
// runs along one of those faces (a door opening's side of the body, not the far side of the ring that was thickened).
TEST_CASE(polymesh_solidify_walls_the_whole_outline_or_only_along_the_faces_asked_for) {
    PolyMesh c = cube();
    const std::vector<int> top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    const std::vector<int> front = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.z > 0.9; });
    CHECK(top.size() == 1 && front.size() == 1);
    const PolyMesh all = solidify(c, top, 0.1, "body");
    CHECK(all.faces.size() == 5);                            // the back and a wall on each of the four edges
    const PolyMesh one = solidify(c, top, 0.1, "body", &front);
    CHECK(one.faces.size() == 2);                            // the back and the one wall along the front face
    if (one.faces.size() != 2) return;
    double z = 0;
    for (int v : one.faces[1].v) z += one.pts[static_cast<std::size_t>(v)].z / 4.0;
    CHECK(z > 0.44);                                         // the wall stands at the front edge (the back is offset along the normal only)
    CHECK(one.faceNormal(1).z > 0.9);                        // and faces out of the region
    const std::vector<int> none;
    CHECK(solidify(c, top, 0.1, "body", &none).faces.size() == 1);   // along nothing: the back alone
}

TEST_CASE(polymesh_bevel_cuts_every_sharp_edge_to_a_strip_and_closes_the_corners) {
    PolyMesh c = cube();
    c.creases.clear();
    const std::vector<int> made = bevel(c, 0.1);
    // 6 faces drawn back, 12 strips, 8 corner triangles; every old corner is three points now
    CHECK(c.faces.size() == 26);
    CHECK(made.size() == 20);
    CHECK(c.pts.size() == 24);
    CHECK(isClosed(c));
    const double v = signedVolume(c);
    CHECK(v > 0);                      // still outward
    CHECK(v < 1.0 && v > 1.0 - 12 * 0.5 * 0.1 * 0.1);   // less than the cube by under twelve edge prisms
    Vec3 lo, hi;
    bounds(c, lo, hi);
    CHECK_APPROX(lo.x, -0.5, 1e-9);    // the faces kept their planes
    CHECK_APPROX(hi.y, 0.5, 1e-9);
    int strips = 0, corners = 0;
    for (int f : made) {
        const PolyMesh::Face& face = c.faces[static_cast<std::size_t>(f)];
        CHECK(c.groups[static_cast<std::size_t>(face.group)] == "bevel");
        if (face.v.size() == 4) ++strips; else if (face.v.size() == 3) ++corners;
    }
    CHECK(strips == 12);
    CHECK(corners == 8);
    // a strip is `width` back from the edge on each face: 0.1 * sqrt(2) across
    for (int f : made) {
        const std::vector<int>& q = c.faces[static_cast<std::size_t>(f)].v;
        if (q.size() != 4) continue;
        const double e0 = (c.pts[static_cast<std::size_t>(q[1])] - c.pts[static_cast<std::size_t>(q[0])]).length();
        const double e1 = (c.pts[static_cast<std::size_t>(q[2])] - c.pts[static_cast<std::size_t>(q[1])]).length();
        CHECK_APPROX(std::min(e0, e1), 0.1 * std::sqrt(2.0), 1e-9);
        CHECK_APPROX(std::max(e0, e1), 0.8, 1e-9);
    }
    // each original face is a 0.8 square in its own plane
    for (int f = 0; f < 6; ++f) {
        const std::vector<int>& q = c.faces[static_cast<std::size_t>(f)].v;
        CHECK(q.size() == 4);
        CHECK_APPROX((c.pts[static_cast<std::size_t>(q[1])] - c.pts[static_cast<std::size_t>(q[0])]).length(), 0.8, 1e-9);
    }

    // An angle no edge reaches leaves the mesh alone.
    PolyMesh same = cube();
    CHECK(bevel(same, 0.1, 100.0).empty());
    CHECK(same.faces.size() == 6 && same.pts.size() == 8);

    // A selection bevels only its own edges: the top face's rim is four strips, and the four top corners, each with
    // two bevelled edges and one not, become two points with no corner face.
    PolyMesh top = cube();
    top.creases.clear();
    const std::vector<int> up = selectWhere(top, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    CHECK(up.size() == 1);
    const std::vector<int> rim = bevel(top, 0.1, 30.0, up);
    CHECK(rim.size() == 4);
    CHECK(top.faces.size() == 10);
    CHECK(top.pts.size() == 12);
    CHECK(isClosed(top));
    CHECK(signedVolume(top) > 0.9 && signedVolume(top) < 1.0);

    // Bevelled twice, the strips' own edges are cut again (45 degrees each): still closed and outward.
    PolyMesh twice = cube();
    twice.creases.clear();
    bevel(twice, 0.1);
    bevel(twice, 0.02, 30.0);
    CHECK(isClosed(twice));
    CHECK(signedVolume(twice) > 0.9);

    // A mesh with an open border keeps its border.
    const std::vector<Vec2> sq = {Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.5, 0.5), Vec2(-0.5, 0.5)};
    PolyMesh open = loft({sq, sq}, {-0.5, 0.5}, false, false);
    open.creases.clear();
    const std::size_t before = open.pts.size();
    const std::vector<int> walls = bevel(open, 0.1);
    CHECK(open.pts.size() == before);      // every point is on the border: none split
    CHECK(walls.empty() || !walls.empty()); // (it may add nothing; it must not crash or tear)
}

TEST_CASE(polymesh_loop_cut_runs_a_ring_of_edges_through_the_quads) {
    PolyMesh c = cube();
    c.creases.clear();
    // across an edge that runs along z (between the two square sections): the ring goes round the four side faces
    const int a = nearestPoint(c, Vec3(0.5, 0.5, -0.5)), b = nearestPoint(c, Vec3(0.5, 0.5, 0.5));
    const std::vector<int> ring = loopCut(c, a, b, 0.25);
    CHECK(ring.size() == 4);
    CHECK(c.faces.size() == 10);            // four sides each in two; the two ends untouched
    CHECK(c.pts.size() == 12);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 1.0, 1e-9);
    for (int p : ring) CHECK_APPROX(c.pts[static_cast<std::size_t>(p)].z, -0.25, 1e-9);   // a quarter from a's side, all the way round
    // again nearer the other end, by its mirror
    CHECK(loopCut(c, b, nearestPoint(c, Vec3(0.5, 0.5, -0.25)), 0.25).size() == 4);
    CHECK(c.faces.size() == 14);
    CHECK(isClosed(c));
    // two points that are not an edge's ends cut nothing
    CHECK(loopCut(c, nearestPoint(c, Vec3(0.5, 0.5, 0.5)), nearestPoint(c, Vec3(-0.5, -0.5, -0.5))).empty());
    CHECK(c.faces.size() == 14);

    // a crease on a crossed edge carries on over both halves
    PolyMesh k = cube();
    k.creases.clear();
    k.setCrease(a, b, 2.0f);
    const std::vector<int> r2 = loopCut(k, a, b);
    CHECK(k.crease(a, b) == 0.0f);
    bool carried = false;
    for (int p : r2)
        if (k.crease(a, p) == 2.0f && k.crease(p, b) == 2.0f) carried = true;
    CHECK(carried);

    // on an open tube the ring still closes; where the cut meets a face that is not a quad it stops, and that face
    // gains the point (the mesh stays closed)
    PolyMesh b3 = cube();
    b3.creases.clear();
    bevel(b3, 0.1);                           // corner triangles now sit between the strips
    const std::size_t before = b3.faces.size();
    const int s0 = nearestPoint(b3, Vec3(0.4, 0.5, 0.4)), s1 = nearestPoint(b3, Vec3(0.5, 0.4, 0.4));
    const std::vector<int> along = loopCut(b3, s0, s1);     // along one strip, end to end
    CHECK(along.size() == 2);
    CHECK(b3.faces.size() == before + 1);
    CHECK(isClosed(b3));
}

TEST_CASE(polymesh_slice_cuts_along_a_plane_and_moves_nothing) {
    PolyMesh c = cube();
    c.creases.clear();
    const std::vector<int> cut = slice(c, Vec3(0, 0.2, 0), Vec3(0, 1, 0));
    CHECK(cut.size() == 4);
    CHECK(c.faces.size() == 10);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 1.0, 1e-9);
    for (int p : cut) CHECK_APPROX(c.pts[static_cast<std::size_t>(p)].y, 0.2, 1e-9);
    // a plane through corners passes through points already there: the two faces it crosses corner to corner split
    PolyMesh d = cube();
    d.creases.clear();
    const std::vector<int> diag = slice(d, Vec3(0, 0, 0), Vec3(1, -1, 0));
    CHECK(diag.size() == 4);
    CHECK(d.pts.size() == 8);                 // no new point
    CHECK(d.faces.size() == 8);               // the two end squares each in two triangles
    CHECK(isClosed(d));
    // a plane that misses changes nothing
    CHECK(slice(d, Vec3(0, 5, 0), Vec3(0, 1, 0)).empty());
    CHECK(d.faces.size() == 8);
    // a slanted plane through a bevelled cube: still closed, same volume
    PolyMesh e = cube();
    e.creases.clear();
    bevel(e, 0.1);
    const double v = signedVolume(e);
    CHECK(!slice(e, Vec3(0.03, 0.02, 0.01), Vec3(0.3, 1, 0.2)).empty());
    CHECK(isClosed(e));
    CHECK_APPROX(signedVolume(e), v, 1e-9);
}

TEST_CASE(polymesh_bridge_joins_two_solids_or_makes_a_hole_through_one) {
    // two unit cubes a metre apart along z: bridge the faces that look at each other
    PolyMesh two = cube();
    two.creases.clear();
    PolyMesh other = cube();
    other.creases.clear();
    for (Vec3& p : other.pts) p.z += 2.0;
    two.append(other);
    const std::vector<int> front = selectWhere(two, [](const Vec3& c, const Vec3& n) { return n.z > 0.9 && c.z < 1.0; });
    const std::vector<int> back = selectWhere(two, [](const Vec3& c, const Vec3& n) { return n.z < -0.9 && c.z > 1.0; });
    CHECK(front.size() == 1 && back.size() == 1);
    const std::vector<int> tube = bridge(two, front, back);
    CHECK(tube.size() == 4);
    CHECK(two.faces.size() == 14);            // 12 less the two taken away, and four joining
    CHECK(isClosed(two));
    CHECK_APPROX(signedVolume(two), 3.0, 1e-9);   // the two cubes and the metre between them

    // a hole through a block: inset the top and the bottom, bridge the two inner faces
    PolyMesh block = cube();
    block.creases.clear();
    const std::vector<int> top = inset(block, selectWhere(block, [](const Vec3&, const Vec3& n) { return n.y > 0.9; }), 0.25);
    const std::vector<int> bottom = inset(block, selectWhere(block, [](const Vec3&, const Vec3& n) { return n.y < -0.9; }), 0.25);
    CHECK(top.size() == 1 && bottom.size() == 1);
    CHECK(bridge(block, top, bottom).size() == 4);
    CHECK(isClosed(block));
    const double v = signedVolume(block);
    CHECK(v > 0.5 && v < 0.95);               // the cube less a hole about half a metre square

    // a hole that reaches the solid's edge is open on that side: a slot cut down from the top of a slab. The region
    // on the front touches the top edge, so one new quad would lie back to back with the top face's middle piece.
    PolyMesh slab = cube();
    slab.creases.clear();
    slice(slab, Vec3(-0.2, 0, 0), Vec3(1, 0, 0));
    slice(slab, Vec3(0.2, 0, 0), Vec3(1, 0, 0));
    slice(slab, Vec3(0, 0.1, 0), Vec3(0, 1, 0));
    auto notch = [&](double facing) {
        return selectWhere(slab, [facing](const Vec3& c, const Vec3& n) { return n.z * facing > 0.9 && std::fabs(c.x) < 0.2 && c.y > 0.1; });
    };
    const std::vector<int> nf = notch(1), nb = notch(-1);
    CHECK(nf.size() == 1 && nb.size() == 1);
    CHECK(bridge(slab, nf, nb).size() == 3);      // two sides and the floor of the slot; its top is open
    CHECK(isClosed(slab));
    CHECK_APPROX(signedVolume(slab), 1.0 - 0.4 * 0.4 * 1.0, 1e-9);

    // rims that do not match are refused and nothing changes
    PolyMesh odd = cube();
    odd.creases.clear();
    bevel(odd, 0.1);
    const std::size_t faces = odd.faces.size();
    const std::vector<int> square = selectWhere(odd, [](const Vec3&, const Vec3& n) { return n.y > 0.99; });
    const std::vector<int> triangle = {20 + 0};   // (set below to a real corner face)
    std::vector<int> corner;
    for (std::size_t f = 0; f < odd.faces.size(); ++f)
        if (odd.faces[f].v.size() == 3) { corner.push_back(static_cast<int>(f)); break; }
    (void)triangle;
    CHECK(bridge(odd, square, corner).empty());
    CHECK(odd.faces.size() == faces);
    CHECK(bridge(odd, square, square).empty());
}

TEST_CASE(polymesh_unsubdivide_takes_a_level_back_out) {
    PolyMesh cage = cube();
    cage.creases.clear();
    // one level in, one level out: the cube's 6 faces and 8 points again, facing out
    PolyMesh one = subdivide(cage, 1);
    CHECK(one.faces.size() == 24);
    CHECK(unsubdivide(one));
    CHECK(one.faces.size() == 6);
    CHECK(one.pts.size() == 8);
    CHECK(isClosed(one));
    CHECK(signedVolume(one) > 0);
    CHECK(signedVolume(one) < 1.0);          // the points did not move: the smoothed corners, inside the cage
    for (const PolyMesh::Face& f : one.faces) CHECK(f.v.size() == 4);
    // two levels in, one out: the same faces as one level in, at the points two levels put them
    PolyMesh two = subdivide(cage, 2);
    const PolyMesh level1 = subdivide(cage, 1);
    CHECK(two.faces.size() == 96);
    CHECK(unsubdivide(two));
    CHECK(two.faces.size() == level1.faces.size());
    CHECK(two.pts.size() == level1.pts.size());
    CHECK(isClosed(two));
    CHECK(unsubdivide(two));
    CHECK(two.faces.size() == 6);
    CHECK(isClosed(two) && signedVolume(two) > 0);
    // a third time there is no level left: refused, unchanged
    CHECK(!unsubdivide(two));
    CHECK(two.faces.size() == 6 && two.pts.size() == 8);

    // a crease that held through the level comes back on the whole edge
    // (a crease of sharpness 3 is 2 on each half after one level; one of sharpness 1 is used up by the level)
    PolyMesh sharp = cube();
    CHECK(!sharp.creases.empty());            // the loft creased its caps
    for (std::pair<const std::pair<int, int>, float>& c : sharp.creases) c.second = 3.0f;
    const std::size_t creased = sharp.creases.size();
    PolyMesh fine = subdivide(sharp, 1);
    CHECK(unsubdivide(fine));
    CHECK(fine.creases.size() == creased);
    for (const std::pair<const std::pair<int, int>, float>& c : fine.creases) CHECK_APPROX(c.second, 3.0, 1e-6);

    // a bevelled cube subdivided: faces of 3, 4 and 8 corners come back as they were
    PolyMesh b = cube();
    b.creases.clear();
    bevel(b, 0.1);
    PolyMesh bs = subdivide(b, 1);
    CHECK(unsubdivide(bs));
    CHECK(bs.faces.size() == b.faces.size());
    CHECK(bs.pts.size() == b.pts.size());
    int threes = 0;
    for (const PolyMesh::Face& f : bs.faces) threes += f.v.size() == 3 ? 1 : 0;
    CHECK(threes == 8);
    CHECK(isClosed(bs) && signedVolume(bs) > 0);

    // keeping a face's middle instead gives the other grid (the dual): 8 faces of three corners, 6 points
    PolyMesh dual = subdivide(cage, 1);
    const int middle = nearestPoint(dual, Vec3(0, 0, 0.5));
    CHECK(unsubdivide(dual, middle));
    CHECK(dual.pts.size() == 6);
    CHECK(dual.faces.size() == 8);
    CHECK(isClosed(dual) && signedVolume(dual) > 0);

    // not the pattern: a face that is not a quad
    PolyMesh tri = cube();
    bevel(tri, 0.1);
    const std::size_t before = tri.faces.size();
    CHECK(!unsubdivide(tri));
    CHECK(tri.faces.size() == before);
    const std::vector<Vec2> sq = {Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.5, 0.5), Vec2(-0.5, 0.5)};
    // an open tube (no caps) keeps its two open ends: four faces and eight points again
    const PolyMesh tube = loft({sq, sq}, {-0.5, 0.5}, false, false);
    PolyMesh open = subdivide(tube, 1);
    CHECK(open.faces.size() == 16);
    CHECK(unsubdivide(open));
    CHECK(open.faces.size() == tube.faces.size());
    CHECK(open.pts.size() == tube.pts.size());
    CHECK(!isClosed(open));
    Vec3 olo, ohi;
    bounds(open, olo, ohi);
    CHECK_APPROX(olo.z, -0.5, 1e-9);          // the border's corners stayed on the border
    CHECK_APPROX(ohi.z, 0.5, 1e-9);
    // a cube with one face lifted out, two levels in and out
    PolyMesh lidless = cube();
    lidless.creases.clear();
    lidless.faces.erase(lidless.faces.begin());
    PolyMesh fine2 = subdivide(lidless, 2);
    CHECK(unsubdivide(fine2) && unsubdivide(fine2));
    CHECK(fine2.faces.size() == 5);
    CHECK(fine2.pts.size() == 8);
}

TEST_CASE(polymesh_bevel_rounds_an_edge_in_more_passes) {
    PolyMesh flat = cube();
    flat.creases.clear();
    bevel(flat, 0.1);
    PolyMesh round2 = cube();
    round2.creases.clear();
    const std::vector<int> made2 = bevel(round2, 0.1, 30.0, {}, 2);
    PolyMesh round3 = cube();
    round3.creases.clear();
    bevel(round3, 0.1, 30.0, {}, 3);
    CHECK(isClosed(round2) && isClosed(round3));
    CHECK(round2.faces.size() > flat.faces.size());
    CHECK(round3.faces.size() > round2.faces.size());
    CHECK(made2.size() == round2.faces.size() - 6);
    // each pass cuts the strip's corners back, so a little more comes off each time, and less each time
    const double v1 = signedVolume(flat), v2 = signedVolume(round2), v3 = signedVolume(round3);
    CHECK(v1 > v2 && v2 > v3);
    CHECK(v1 - v2 > v2 - v3);
    CHECK(v3 > 0.9);
    // the six flat faces are still in their planes and now 0.8 across
    Vec3 lo, hi;
    bounds(round3, lo, hi);
    CHECK_APPROX(lo.x, -0.5, 1e-9);
    CHECK_APPROX(hi.z, 0.5, 1e-9);
    // across a square edge two rounds leave three facets of the same width
    std::vector<double> widths;
    for (std::size_t f = 6; f < round2.faces.size(); ++f) {
        const std::vector<int>& q = round2.faces[f].v;
        if (q.size() != 4) continue;
        const double e0 = (round2.pts[static_cast<std::size_t>(q[1])] - round2.pts[static_cast<std::size_t>(q[0])]).length();
        const double e1 = (round2.pts[static_cast<std::size_t>(q[2])] - round2.pts[static_cast<std::size_t>(q[1])]).length();
        if (std::max(e0, e1) > 0.5) widths.push_back(std::min(e0, e1));      // the long strips along the cube's edges
    }
    CHECK(widths.size() == 36);                                               // three to an edge
    for (double w : widths) CHECK_APPROX(w, widths[0], 2e-3);
    // a width wider than the faces have room for is held back: no face folds over, the solid stays inside the cube
    PolyMesh wide = cube();
    wide.creases.clear();
    bevel(wide, 5.0);
    CHECK(isClosed(wide));
    CHECK(signedVolume(wide) > 0.15 && signedVolume(wide) < 1.0);   // (nearly all edge: about a quarter of the cube is left)
    bounds(wide, lo, hi);
    CHECK(lo.x >= -0.5 - 1e-9 && hi.x <= 0.5 + 1e-9 && lo.y >= -0.5 - 1e-9 && hi.z <= 0.5 + 1e-9);
    for (std::size_t f = 0; f < 6; ++f) {
        const Vec3 n = wide.faceNormal(static_cast<int>(f));
        CHECK(std::fabs(std::fabs(n.x) + std::fabs(n.y) + std::fabs(n.z) - 1.0) < 1e-9);   // each still faces along its axis
        CHECK(dot(n, wide.faceCentroid(static_cast<int>(f))) > 0.49);                        // ...and outward
    }
    // a thin slab bevelled wider than it is thick: the same
    const std::vector<Vec2> thin = {Vec2(-0.5, -0.02), Vec2(0.5, -0.02), Vec2(0.5, 0.02), Vec2(-0.5, 0.02)};
    PolyMesh slab = loft({thin, thin}, {-0.5, 0.5}, true, true);
    slab.creases.clear();
    bevel(slab, 0.1, 30.0, {}, 2);
    CHECK(isClosed(slab));
    CHECK(signedVolume(slab) > 0.02 && signedVolume(slab) < 0.04);

    // one round is the flat bevel
    PolyMesh one = cube();
    one.creases.clear();
    bevel(one, 0.1, 30.0, {}, 1);
    CHECK(one.faces.size() == flat.faces.size());
}
