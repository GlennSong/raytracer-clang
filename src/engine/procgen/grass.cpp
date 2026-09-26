#include "grass.h"
#include "proc_rng.h"
#include "../mesh_builder.h"

#include <cstdio>
#include <string>

#include <cmath>

namespace engine {


RenderMesh grassClump(uint32_t seed, const GrassClumpParams& p) {
    RenderMesh m;
    ProcRng rng(seed);
    const Vec3 up(0, 1, 0);
    auto vertex = [&](const Vec3& pos, double t) {   // t: 0 root .. 1 tip
        Vertex v(pos, up, Vec3(1, 0, 0), 0.0f, static_cast<float>(t));
        v.color = p.rootColor + (p.tipColor - p.rootColor) * (t * t * (3.0 - 2.0 * t));
        m.vertices.push_back(v);
        return static_cast<uint32_t>(m.vertices.size() - 1);
    };
    for (int b = 0; b < p.blades; ++b) {
        // where it stands (area-uniform over the disc), which way it faces and leans
        const double r = p.radius * std::sqrt(rng.next()), a = rng.next() * 6.283185307;
        const Vec3 base(r * std::cos(a), 0.0, r * std::sin(a));
        const double face = rng.next() * 6.283185307;
        const Vec3 across(std::cos(face), 0.0, std::sin(face));        // the blade's width
        const double leanDir = face + 1.5707963 + (rng.next() - 0.5) * 1.2;
        const Vec3 out(std::cos(leanDir), 0.0, std::sin(leanDir));      // the way it bends
        const double h = p.height * (0.65 + 0.7 * rng.next());
        const double w = p.width * (0.8 + 0.4 * rng.next());
        const double lean = p.lean * (0.5 + rng.next()) * h;
        // a curve: little lean at the bend, the rest at the tip
        const double tb = 0.55;
        const Vec3 bend = base + Vec3(0, h * tb, 0) + out * (lean * 0.3);
        const Vec3 tip = base + Vec3(0, h, 0) + out * lean;
        const uint32_t r0 = vertex(base - across * (w * 0.5), 0.0), r1 = vertex(base + across * (w * 0.5), 0.0);
        const uint32_t b0 = vertex(bend - across * (w * 0.32), tb), b1 = vertex(bend + across * (w * 0.32), tb);
        const uint32_t t0 = vertex(tip, 1.0);
        m.indices.insert(m.indices.end(), {r0, r1, b1, r0, b1, b0, b0, b1, t0});
    }
    // FLOWERS: stems (thin blades, taller, upright) each carrying a head at its tip -- petals round a
    // centre, the head tilted a little toward the stem's lean, facing mostly up to the sun
    for (int f = 0; f < p.flowers; ++f) {
        const double r = p.radius * std::sqrt(rng.next()), a = rng.next() * 6.283185307;
        const Vec3 base(r * std::cos(a), 0.0, r * std::sin(a));
        const double face = rng.next() * 6.283185307;
        const Vec3 across(std::cos(face), 0.0, std::sin(face));
        const Vec3 out(std::cos(face + 1.5707963), 0.0, std::sin(face + 1.5707963));
        const double h = p.height * (0.9 + 0.5 * rng.next());
        const Vec3 tip = base + Vec3(0, h, 0) + out * (0.08 * h * rng.next());
        const double w = p.width * 0.35;
        const uint32_t s0 = vertex(base - across * w, 0.0), s1 = vertex(base + across * w, 0.0), s2 = vertex(tip, 1.0);
        m.indices.insert(m.indices.end(), {s0, s1, s2});
        // the head: a centre and `petals` petal tips, each petal a triangle pair (centre, two sides, tip)
        const double size = p.flowerSize * (0.75 + 0.5 * rng.next());
        const Vec3 tilt = normalize(Vec3(0, 1, 0) + out * 0.35);
        const Vec3 u = normalize(cross(tilt, across)), v = normalize(cross(tilt, u));
        const double shade = 0.8 + 0.35 * rng.next();
        auto head = [&](const Vec3& pos, const Vec3& col) {
            Vertex vx(pos, tilt, Vec3(1, 0, 0), 0.0f, 1.0f);
            vx.color = col;
            m.vertices.push_back(vx);
            return static_cast<uint32_t>(m.vertices.size() - 1);
        };
        const Vec3 petal = p.flowerColor * shade;
        const Vec3 centreCol(0.35, 0.22, 0.02);
        const uint32_t c = head(tip + tilt * 0.004, centreCol);
        const double rot = rng.next() * 6.283185307;
        for (int k = 0; k < p.petals; ++k) {
            const double a0 = rot + k * 6.283185307 / p.petals, half = 3.14159265 / p.petals * 0.8;
            const Vec3 d0 = u * std::cos(a0 - half) + v * std::sin(a0 - half), d1 = u * std::cos(a0 + half) + v * std::sin(a0 + half);
            const Vec3 dm = u * std::cos(a0) + v * std::sin(a0);
            const uint32_t q0 = head(tip + d0 * (size * 0.3), petal), q1 = head(tip + d1 * (size * 0.3), petal),
                           q2 = head(tip + dm * size - tilt * (0.15 * size), petal);
            m.indices.insert(m.indices.end(), {c, q0, q2, c, q2, q1});
        }
    }
    return m;
}

RenderMesh grassCardMesh(const GrassClumpParams& p) {
    // a card is a patch of field about a metre across, a little taller than a clump (its tallest blades)
    return MeshBuilder::crossCards(1.3, p.height * 1.25, 3);
}

TextureData grassCardTexture(uint32_t seed, const GrassClumpParams& p) {
    char key[256];
    std::snprintf(key, sizeof key, "grassCard/v1/%u/%.4f,%.4f,%.4f/%.4f,%.4f,%.4f/%.3f", seed, p.rootColor.x, p.rootColor.y,
                  p.rootColor.z, p.tipColor.x, p.tipColor.y, p.tipColor.z, p.lean);
    return bakeCached(key, [&] {
        const int count = 90;
        const double width = 0.03, minHeight = 0.4;
        const Field2 mask = fieldBlades(seed, count, width, p.lean, minHeight);
        const Field2 tone = fieldBladeTone(seed, count, width, p.lean, minHeight);
        // root to tip as the clumps' vertex colours run, each blade a little lighter or darker
        const ColorField2 base = colorMixBy(colorConstant(p.rootColor), colorConstant(p.tipColor),
                                            fieldSmoothstep(fieldGradientY(), 0.0, 1.0));
        return bakeFieldRGBA(colorMul(base, fieldScaleBias(tone, 0.5, 0.75)), mask, 256, false);
    });
}

}  // namespace engine
