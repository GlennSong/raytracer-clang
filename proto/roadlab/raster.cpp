#include "raster.h"

#include <atomic>
#include <cstdio>
#include <thread>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace roadlab {

void Framebuffer::resize(int w, int h) {
    width = w;
    height = h;
    color.assign(size_t(w) * size_t(h), Vec3{0, 0, 0});
    depth.assign(size_t(w) * size_t(h), 1e30f);
}

namespace {

struct Mat4 {
    double m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    static Mat4 identity() { return {}; }
};

Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a.m[i * 4 + k] * b.m[k * 4 + j];
            r.m[i * 4 + j] = s;
        }
    return r;
}

struct Vec4 {
    double x, y, z, w;
};

Vec4 transform(const Mat4& m, Vec3 p) {
    return {m.m[0] * p.x + m.m[1] * p.y + m.m[2] * p.z + m.m[3],
            m.m[4] * p.x + m.m[5] * p.y + m.m[6] * p.z + m.m[7],
            m.m[8] * p.x + m.m[9] * p.y + m.m[10] * p.z + m.m[11],
            m.m[12] * p.x + m.m[13] * p.y + m.m[14] * p.z + m.m[15]};
}

Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = normalize(target - eye);
    Vec3 s = normalize(cross(f, up));
    if (length(s) < 1e-9) s = {1, 0, 0};
    Vec3 u = cross(s, f);
    Mat4 r;
    r.m[0] = s.x;  r.m[1] = s.y;  r.m[2] = s.z;  r.m[3] = -dot(s, eye);
    r.m[4] = u.x;  r.m[5] = u.y;  r.m[6] = u.z;  r.m[7] = -dot(u, eye);
    r.m[8] = -f.x; r.m[9] = -f.y; r.m[10] = -f.z; r.m[11] = dot(f, eye);
    r.m[12] = 0;   r.m[13] = 0;   r.m[14] = 0;   r.m[15] = 1;
    return r;
}

Mat4 perspective(double fovY, double aspect, double zn, double zf) {
    double f = 1.0 / std::tan(fovY * 0.5);
    Mat4 r;
    for (double& v : r.m) v = 0;
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zf + zn) / (zn - zf);
    r.m[11] = 2.0 * zf * zn / (zn - zf);
    r.m[14] = -1.0;
    return r;
}

Mat4 orthographic(double halfW, double halfH, double zn, double zf) {
    Mat4 r;
    for (double& v : r.m) v = 0;
    r.m[0] = 1.0 / halfW;
    r.m[5] = 1.0 / halfH;
    r.m[10] = -2.0 / (zf - zn);
    r.m[11] = -(zf + zn) / (zf - zn);
    r.m[15] = 1.0;
    return r;
}

struct Varying {
    Vec3 pos;
    Vec3 normal;
    Vec3 color;
    double s, t, roughness;
};

struct ClipVert {
    Vec4 clip;
    Varying v;
};

Varying lerpVarying(const Varying& a, const Varying& b, double u) {
    Varying r;
    r.pos = lerp(a.pos, b.pos, u);
    r.normal = lerp(a.normal, b.normal, u);
    r.color = lerp(a.color, b.color, u);
    r.s = lerp(a.s, b.s, u);
    r.t = lerp(a.t, b.t, u);
    r.roughness = lerp(a.roughness, b.roughness, u);
    return r;
}

Vec3 toneMap(Vec3 c, double exposure) {
    c = c * exposure;
    // Reinhard with a shoulder, then sRGB-ish gamma. Enough to keep the bright
    // paint from clipping to flat white next to dark asphalt.
    auto f = [](double v) {
        v = std::max(0.0, v);
        v = v / (1.0 + v * 0.72);
        return std::pow(v, 1.0 / 2.2);
    };
    return {f(c.x), f(c.y), f(c.z)};
}

}  // namespace

Camera cameraTopDown(Vec2 lo, Vec2 hi, double aspect, double margin) {
    Camera c;
    lo.x -= margin;
    lo.y -= margin;
    hi.x += margin;
    hi.y += margin;
    Vec2 centre = (lo + hi) * 0.5;
    double spanX = std::max(1.0, hi.x - lo.x);
    double spanZ = std::max(1.0, hi.y - lo.y);
    c.ortho = true;
    c.orthoHeight = std::max(spanZ, spanX / std::max(0.01, aspect));
    c.eye = {centre.x, 2000.0, centre.y};
    c.target = {centre.x, 0.0, centre.y};
    // Straight down needs an up vector that is not parallel to the view: +Z maps
    // to screen-down so the plan reads like a map.
    c.up = {0, 0, -1};
    return c;
}

Camera cameraLook(Vec3 eye, Vec3 target, double fovYDeg) {
    Camera c;
    c.eye = eye;
    c.target = target;
    c.fovYDeg = fovYDeg;
    c.ortho = false;
    return c;
}

Camera cameraDriver(const Road& road, double s, int laneId, double eyeHeight, double lookAhead) {
    double t = road.laneCenterT(laneId, s);
    Vec3 eye = road.surfacePoint(s, t) + Vec3{0, eyeHeight, 0};
    int sec = road.xs.sectionIndexAt(s);
    const Strip* st = sec >= 0 ? road.xs.sections[size_t(sec)].strip(laneId) : nullptr;
    double dir = (st && st->dir < 0) ? -1.0 : 1.0;
    double sAhead = clampd(s + dir * lookAhead, road.begin(), road.end());
    Vec3 target = road.surfacePoint(sAhead, road.laneCenterT(laneId, sAhead)) +
                  Vec3{0, eyeHeight * 0.55, 0};
    return cameraLook(eye, target, 52.0);
}

void renderScene(const Mesh& mesh, const Network& net, const PaintSet& paint, const Camera& cam,
                 const RenderOptions& opt, Framebuffer& fb) {
    fb.resize(opt.width, opt.height);
    const double aspect = double(opt.width) / double(opt.height);
    Mat4 view = lookAt(cam.eye, cam.target, cam.up);
    Mat4 proj = cam.ortho ? orthographic(cam.orthoHeight * 0.5 * aspect, cam.orthoHeight * 0.5,
                                         cam.zNear, cam.zFar)
                          : perspective(cam.fovYDeg * kDeg2Rad, aspect, cam.zNear, cam.zFar);
    Mat4 vp = proj * view;
    Vec3 sun = normalize(opt.sunDir);

    int threads = opt.threads > 0 ? opt.threads : int(std::thread::hardware_concurrency());
    threads = std::max(1, std::min(threads, 32));

    // Sky / background: a simple vertical gradient so a perspective shot has a
    // horizon to sit against.
    for (int y = 0; y < fb.height; ++y) {
        double v = double(y) / double(std::max(1, fb.height - 1));
        Vec3 c = cam.ortho ? Vec3{0.045, 0.05, 0.055}
                           : lerp(opt.skyColor, opt.horizonColor, std::pow(v, 1.6));
        for (int x = 0; x < fb.width; ++x) fb.color[size_t(y) * size_t(fb.width) + size_t(x)] = c;
    }

    auto shadeFragment = [&](const Varying& vy, MatKind mat, int roadId, int junctionId,
                             double fw) {
        SurfaceSample smp;
        Vec3 normal = normalize(vy.normal);
        if (mat == MatKind::RoadSurface && roadId >= 0 && roadId < net.roadCount()) {
            const Road& r = net.road(roadId);
            const RoadPaint* rp =
                size_t(roadId) < paint.roads.size() ? &paint.roads[size_t(roadId)] : nullptr;
            smp = shadeRoadSurface(r, rp, vy.s, vy.t, fw, opt.shade);
            // Micro normal, but only where a fragment is small enough for the
            // detail to mean anything — this is the normal map's LOD.
            if (opt.microNormals && opt.shade.micro && fw < 0.18) {
                const double h = 0.06;
                double hs = surfaceMicroHeight(r, vy.s + h, vy.t, opt.shade) -
                            surfaceMicroHeight(r, vy.s - h, vy.t, opt.shade);
                double ht = surfaceMicroHeight(r, vy.s, vy.t + h, opt.shade) -
                            surfaceMicroHeight(r, vy.s, vy.t - h, opt.shade);
                Frame f = r.spine.frameAt(vy.s);
                double fade = 1.0 - smoothstepd(0.08, 0.18, fw);
                normal = normalize(normal - f.forward * (hs / (2 * h)) * fade -
                                   f.left * (ht / (2 * h)) * fade);
            }
        } else if (mat == MatKind::JunctionPad && junctionId >= 0 &&
                   junctionId < net.junctionCount()) {
            const std::vector<Decal>* pd = size_t(junctionId) < paint.junctions.size()
                                               ? &paint.junctions[size_t(junctionId)]
                                               : nullptr;
            static const std::vector<Decal> kEmpty;
            smp = shadeJunctionPad(net.junction(junctionId), pd ? *pd : kEmpty,
                                   {vy.pos.x, vy.pos.z}, fw, opt.shade);
        } else {
            smp.albedo = vy.color;
            smp.roughness = vy.roughness;
        }

        Vec3 viewDir = normalize(cam.ortho ? (cam.eye - cam.target) : (cam.eye - vy.pos));
        if (dot(normal, viewDir) < 0) normal = -normal;
        double ndl = std::max(0.0, dot(normal, sun));
        Vec3 lit = smp.albedo * (opt.sunColor * ndl);
        // Hemispheric ambient: sky from above, a warm bounce from the ground.
        double up = 0.5 + 0.5 * normal.y;
        lit += smp.albedo * lerp(Vec3{0.10, 0.10, 0.09}, opt.skyColor * 0.65, up);
        if (ndl > 0) {
            Vec3 h = normalize(sun + viewDir);
            double rough = clampd(smp.roughness, 0.04, 1.0);
            double a = rough * rough;
            double expn = 2.0 / (a * a + 1e-4) - 2.0;
            expn = clampd(expn, 1.0, 4096.0);
            double spec = std::pow(std::max(0.0, dot(normal, h)), expn) *
                          (0.04 + 0.9 * smp.metallic) * (expn + 8.0) / 25.0;
            lit += opt.sunColor * spec * ndl;
        }
        return lit;
    };

    std::atomic<int> nextBand{0};
    const int bandHeight = 24;
    const int bandCount = (fb.height + bandHeight - 1) / bandHeight;

    auto worker = [&]() {
        for (;;) {
            int band = nextBand.fetch_add(1);
            if (band >= bandCount) break;
            int y0 = band * bandHeight;
            int y1 = std::min(fb.height, y0 + bandHeight);

            for (size_t tri = 0; tri < mesh.indices.size(); tri += 3) {
                const Vertex& V0 = mesh.verts[mesh.indices[tri + 0]];
                const Vertex& V1 = mesh.verts[mesh.indices[tri + 1]];
                const Vertex& V2 = mesh.verts[mesh.indices[tri + 2]];

                ClipVert cv[3];
                const Vertex* src[3] = {&V0, &V1, &V2};
                for (int i = 0; i < 3; ++i) {
                    cv[i].clip = transform(vp, src[i]->pos);
                    cv[i].v = {src[i]->pos, src[i]->normal, src[i]->color,
                               src[i]->s,   src[i]->t,      src[i]->roughness};
                }

                // Near-plane clip in clip space (w > epsilon). Without it a
                // triangle straddling the camera wraps around the screen.
                ClipVert poly[8];
                int polyCount = 0;
                const double kW = 1e-4;
                for (int i = 0; i < 3; ++i) {
                    const ClipVert& a = cv[i];
                    const ClipVert& b = cv[(i + 1) % 3];
                    bool ain = a.clip.w > kW, bin = b.clip.w > kW;
                    if (ain) poly[polyCount++] = a;
                    if (ain != bin) {
                        double u = (kW - a.clip.w) / (b.clip.w - a.clip.w);
                        ClipVert c;
                        c.clip = {lerp(a.clip.x, b.clip.x, u), lerp(a.clip.y, b.clip.y, u),
                                  lerp(a.clip.z, b.clip.z, u), kW};
                        c.v = lerpVarying(a.v, b.v, u);
                        poly[polyCount++] = c;
                    }
                }
                if (polyCount < 3) continue;

                for (int fan = 1; fan + 1 < polyCount; ++fan) {
                    const ClipVert* p[3] = {&poly[0], &poly[fan], &poly[fan + 1]};
                    double sx[3], sy[3], invW[3], ndcZ[3];
                    bool bad = false;
                    for (int i = 0; i < 3; ++i) {
                        double w = p[i]->clip.w;
                        if (w <= kW) {
                            bad = true;
                            break;
                        }
                        invW[i] = 1.0 / w;
                        sx[i] = (p[i]->clip.x * invW[i] * 0.5 + 0.5) * fb.width;
                        sy[i] = (0.5 - p[i]->clip.y * invW[i] * 0.5) * fb.height;
                        ndcZ[i] = p[i]->clip.z * invW[i];
                    }
                    if (bad) continue;

                    double area = (sx[1] - sx[0]) * (sy[2] - sy[0]) -
                                  (sx[2] - sx[0]) * (sy[1] - sy[0]);
                    if (std::fabs(area) < 1e-9) continue;
                    double invArea = 1.0 / area;

                    int minX = std::max(0, int(std::floor(std::min({sx[0], sx[1], sx[2]}))));
                    int maxX = std::min(fb.width - 1, int(std::ceil(std::max({sx[0], sx[1], sx[2]}))));
                    int minY = std::max(y0, int(std::floor(std::min({sy[0], sy[1], sy[2]}))));
                    int maxY = std::min(y1 - 1, int(std::ceil(std::max({sy[0], sy[1], sy[2]}))));
                    if (minX > maxX || minY > maxY) continue;

                    // Barycentric gradients are constant in screen space, which
                    // is what lets the fragment footprint (and therefore the
                    // marking antialiasing) be exact instead of guessed.
                    double dl[3][2];
                    dl[0][0] = (sy[1] - sy[2]) * invArea;
                    dl[0][1] = (sx[2] - sx[1]) * invArea;
                    dl[1][0] = (sy[2] - sy[0]) * invArea;
                    dl[1][1] = (sx[0] - sx[2]) * invArea;
                    dl[2][0] = (sy[0] - sy[1]) * invArea;
                    dl[2][1] = (sx[1] - sx[0]) * invArea;

                    double sOverW[3], tOverW[3];
                    for (int i = 0; i < 3; ++i) {
                        sOverW[i] = p[i]->v.s * invW[i];
                        tOverW[i] = p[i]->v.t * invW[i];
                    }
                    double dSw[2] = {0, 0}, dTw[2] = {0, 0}, dIw[2] = {0, 0};
                    for (int i = 0; i < 3; ++i) {
                        for (int k = 0; k < 2; ++k) {
                            dSw[k] += dl[i][k] * sOverW[i];
                            dTw[k] += dl[i][k] * tOverW[i];
                            dIw[k] += dl[i][k] * invW[i];
                        }
                    }

                    MatKind mat = V0.material;
                    int roadId = V0.road;
                    int junctionId = V0.junction;

                    for (int y = minY; y <= maxY; ++y) {
                        for (int x = minX; x <= maxX; ++x) {
                            double px = x + 0.5, py = y + 0.5;
                            double l0 = ((sx[1] - px) * (sy[2] - py) -
                                         (sx[2] - px) * (sy[1] - py)) * invArea;
                            double l1 = ((sx[2] - px) * (sy[0] - py) -
                                         (sx[0] - px) * (sy[2] - py)) * invArea;
                            double l2 = 1.0 - l0 - l1;
                            if (l0 < -1e-7 || l1 < -1e-7 || l2 < -1e-7) continue;

                            double z = l0 * ndcZ[0] + l1 * ndcZ[1] + l2 * ndcZ[2];
                            size_t idx = size_t(y) * size_t(fb.width) + size_t(x);
                            if (z >= fb.depth[idx]) continue;

                            double iw = l0 * invW[0] + l1 * invW[1] + l2 * invW[2];
                            if (iw <= 0) continue;
                            double w = 1.0 / iw;

                            Varying vy;
                            vy.pos = (p[0]->v.pos * (l0 * invW[0]) + p[1]->v.pos * (l1 * invW[1]) +
                                      p[2]->v.pos * (l2 * invW[2])) * w;
                            vy.normal = (p[0]->v.normal * (l0 * invW[0]) +
                                         p[1]->v.normal * (l1 * invW[1]) +
                                         p[2]->v.normal * (l2 * invW[2])) * w;
                            vy.color = (p[0]->v.color * (l0 * invW[0]) +
                                        p[1]->v.color * (l1 * invW[1]) +
                                        p[2]->v.color * (l2 * invW[2])) * w;
                            double sw = l0 * sOverW[0] + l1 * sOverW[1] + l2 * sOverW[2];
                            double tw = l0 * tOverW[0] + l1 * tOverW[1] + l2 * tOverW[2];
                            vy.s = sw * w;
                            vy.t = tw * w;
                            vy.roughness = (p[0]->v.roughness * (l0 * invW[0]) +
                                            p[1]->v.roughness * (l1 * invW[1]) +
                                            p[2]->v.roughness * (l2 * invW[2])) * w;

                            // d(attr)/dx from the quotient rule on (attr/w) / (1/w).
                            double fw = 0.02;
                            if (mat == MatKind::RoadSurface || mat == MatKind::JunctionPad) {
                                double gs[2], gt[2];
                                for (int k = 0; k < 2; ++k) {
                                    gs[k] = (dSw[k] - sw * dIw[k] / iw) * w;
                                    gt[k] = (dTw[k] - tw * dIw[k] / iw) * w;
                                }
                                double ls = std::sqrt(gs[0] * gs[0] + gs[1] * gs[1]);
                                double lt = std::sqrt(gt[0] * gt[0] + gt[1] * gt[1]);
                                fw = clampd(std::max(ls, lt) * 0.5, 0.004, 3.0);
                            }

                            Vec3 lit = shadeFragment(vy, mat, roadId, junctionId, fw);
                            fb.depth[idx] = float(z);
                            fb.color[idx] = lit;
                        }
                    }
                }
            }
        }
    };

    std::vector<std::thread> pool;
    for (int i = 1; i < threads; ++i) pool.emplace_back(worker);
    worker();
    for (std::thread& th : pool) th.join();
}

bool writePng(const Framebuffer& fb, const std::string& path, double exposure) {
    std::vector<unsigned char> pixels(size_t(fb.width) * size_t(fb.height) * 3);
    for (size_t i = 0; i < fb.color.size(); ++i) {
        Vec3 c = toneMap(fb.color[i], exposure);
        pixels[i * 3 + 0] = (unsigned char)clampd(c.x * 255.0 + 0.5, 0, 255);
        pixels[i * 3 + 1] = (unsigned char)clampd(c.y * 255.0 + 0.5, 0, 255);
        pixels[i * 3 + 2] = (unsigned char)clampd(c.z * 255.0 + 0.5, 0, 255);
    }
    return stbi_write_png(path.c_str(), fb.width, fb.height, 3, pixels.data(), fb.width * 3) != 0;
}

}  // namespace roadlab
