#include "test_framework.h"

#include "../src/scene.h"
#include "../src/material.h"

using namespace engine;

namespace {

// A unit quad (two triangles) centred at the local origin in the XY plane,
// facing +Z, under material `mat`.
std::vector<Triangle> unitQuadProto(int mat) {
    auto tri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        Triangle t(a, b, c, mat);
        t.n0 = t.n1 = t.n2 = Vec3(0, 0, 1);
        return t;
    };
    Vec3 a(-0.5, -0.5, 0), b(0.5, -0.5, 0), c(0.5, 0.5, 0), d(-0.5, 0.5, 0);
    return {tri(a, b, c), tri(a, c, d)};
}

}  // namespace

TEST_CASE(instance_places_proto_at_translated_position) {
    Scene scene;
    int mat = scene.addMaterial(Material::pbr(Vec3(0.8, 0.2, 0.2), 0, 0.8));
    int proto = scene.addProto(unitQuadProto(mat));
    scene.addInstance(proto, Mat4::translate(5, 0, 0));
    scene.buildAccelerator();

    HitRecord rec;
    Ray ray(Vec3(5, 0, 10), Vec3(0, 0, -1));
    CHECK(scene.intersect(ray, 0.001, 1e9, rec));
    CHECK_APPROX(rec.t, 10.0, 1e-6);
    CHECK_APPROX(rec.point.x, 5.0, 1e-6);
    CHECK_APPROX(rec.point.z, 0.0, 1e-6);
    CHECK_APPROX(rec.normal.z, 1.0, 1e-6);     // front face toward the ray
    CHECK(rec.materialIndex == mat);

    HitRecord miss;
    Ray off(Vec3(5.9, 0, 10), Vec3(0, 0, -1));
    CHECK(!scene.intersect(off, 0.001, 1e9, miss));   // outside the quad footprint
}

TEST_CASE(instance_applies_scale_and_rotation) {
    Scene scene;
    int mat = scene.addMaterial(Material::pbr(Vec3(0.5, 0.5, 0.5), 0, 0.8));
    int proto = scene.addProto(unitQuadProto(mat));
    scene.addInstance(proto, Mat4::scale(3, 3, 3));             // spans [-1.5, 1.5]
    scene.addInstance(proto, Mat4::translate(20, 0, 0) *
                                 Mat4::rotateY(static_cast<Real>(PI * 0.5)));
    scene.buildAccelerator();

    HitRecord rec;                                              // x=1.2 inside x3 quad
    Ray ray(Vec3(1.2, 0, 10), Vec3(0, 0, -1));
    CHECK(scene.intersect(ray, 0.001, 1e9, rec));
    CHECK_APPROX(rec.point.z, 0.0, 1e-6);

    HitRecord rrec;                                            // rotated quad faces +X
    Ray rray(Vec3(30, 0, 0), Vec3(-1, 0, 0));
    CHECK(scene.intersect(rray, 0.001, 1e9, rrec));
    CHECK_APPROX(rrec.point.x, 20.0, 1e-6);
    CHECK_APPROX(std::fabs(rrec.normal.x), 1.0, 1e-6);
}

TEST_CASE(instanced_hit_matches_baked_triangles) {
    // The same quad placed by an instance vs baked as flat world triangles must
    // report the same hit — instancing is transparent to shading.
    Mat4 xform = Mat4::translate(3, 1, -2) * Mat4::rotateY(0.7) * Mat4::scale(2, 2, 2);

    Scene inst;
    int m1 = inst.addMaterial(Material::pbr(Vec3(0.3, 0.7, 0.4), 0, 0.6));
    int p = inst.addProto(unitQuadProto(m1));
    inst.addInstance(p, xform);
    inst.buildAccelerator();

    Scene baked;
    int m2 = baked.addMaterial(Material::pbr(Vec3(0.3, 0.7, 0.4), 0, 0.6));
    for (Triangle t : unitQuadProto(m2)) {
        Triangle w = t;
        w.v0 = xform.transformPoint(t.v0);
        w.v1 = xform.transformPoint(t.v1);
        w.v2 = xform.transformPoint(t.v2);
        w.n0 = normalize(xform.transformDirection(t.n0));
        w.n1 = normalize(xform.transformDirection(t.n1));
        w.n2 = normalize(xform.transformDirection(t.n2));
        baked.addTriangle(w);
    }
    baked.buildAccelerator();

    Ray ray(Vec3(3, 1, 20), Vec3(0, 0, -1));
    HitRecord ri, rb;
    bool hi = inst.intersect(ray, 0.001, 1e9, ri);
    bool hb = baked.intersect(ray, 0.001, 1e9, rb);
    CHECK(hi);
    CHECK(hb);
    if (hi && hb) {
        CHECK_APPROX(ri.t, rb.t, 1e-5);
        CHECK_APPROX(ri.point.x, rb.point.x, 1e-5);
        CHECK_APPROX(ri.point.y, rb.point.y, 1e-5);
        CHECK_APPROX(ri.point.z, rb.point.z, 1e-5);
        CHECK_APPROX(ri.normal.x, rb.normal.x, 1e-5);
        CHECK_APPROX(ri.normal.y, rb.normal.y, 1e-5);
        CHECK_APPROX(ri.normal.z, rb.normal.z, 1e-5);
    }
}

TEST_CASE(tlas_finds_correct_instance_among_many) {
    Scene scene;
    int mat = scene.addMaterial(Material::pbr(Vec3(0.6, 0.6, 0.6), 0, 0.8));
    int proto = scene.addProto(unitQuadProto(mat));
    const int N = 64;
    for (int i = 0; i < N; ++i)
        scene.addInstance(proto, Mat4::translate(i * 4.0, 0, 0));
    scene.buildAccelerator();

    for (int probe : {0, 17, 40, 63}) {
        HitRecord rec;
        Ray ray(Vec3(probe * 4.0, 0, 10), Vec3(0, 0, -1));
        CHECK(scene.intersect(ray, 0.001, 1e9, rec));
        CHECK_APPROX(rec.point.x, probe * 4.0, 1e-6);
    }

    HitRecord miss;                                  // a gap between quads is empty
    Ray gap(Vec3(2, 0, 10), Vec3(0, 0, -1));
    CHECK(!scene.intersect(gap, 0.001, 1e9, miss));
}

TEST_CASE(instances_coexist_with_flat_triangles) {
    Scene scene;
    int mInst = scene.addMaterial(Material::pbr(Vec3(1, 0, 0), 0, 0.8));
    int mFlat = scene.addMaterial(Material::pbr(Vec3(0, 1, 0), 0, 0.8));

    int proto = scene.addProto(unitQuadProto(mInst));
    scene.addInstance(proto, Mat4::translate(0, 0, 0));        // quad at z=0

    for (Triangle t : unitQuadProto(mFlat)) {                  // flat quad at z=5
        Triangle w = t;
        w.v0 = w.v0 + Vec3(0, 0, 5);
        w.v1 = w.v1 + Vec3(0, 0, 5);
        w.v2 = w.v2 + Vec3(0, 0, 5);
        scene.addTriangle(w);
    }
    scene.buildAccelerator();

    HitRecord rec;
    Ray ray(Vec3(0, 0, 10), Vec3(0, 0, -1));
    CHECK(scene.intersect(ray, 0.001, 1e9, rec));
    CHECK(rec.materialIndex == mFlat);                         // z=5 flat quad is nearer
    CHECK_APPROX(rec.point.z, 5.0, 1e-6);

    HitRecord rec2;                                            // behind it, instance wins
    Ray ray2(Vec3(0, 0, 3), Vec3(0, 0, -1));
    CHECK(scene.intersect(ray2, 0.001, 1e9, rec2));
    CHECK(rec2.materialIndex == mInst);
    CHECK_APPROX(rec2.point.z, 0.0, 1e-6);
}
