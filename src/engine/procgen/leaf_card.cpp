#include "leaf_card.h"

#include "lsystem.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;

uint32_t mixHash(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
double unit(uint32_t h) { return mixHash(h) / 4294967296.0; }

struct V2 {
    double x = 0, y = 0;
};
V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
V2 operator*(V2 a, double s) { return {a.x * s, a.y * s}; }

struct Stem { V2 a, b; double w; };
struct Leaf { V2 base; V2 dir; double len, wid, shade; bool flower = false; };

// The leaf's half-width at t (0 the base, 1 the tip), a fraction of its full half-width, by shape.
double halfWidth(const LeafCardSpec& s, double t) {
    if (t <= 0.0 || t >= 1.0) return 0.0;
    const double sn = std::sin(kPi * t);
    if (s.shape == "needle") return (1.0 - 0.55 * t) * std::min(1.0, t * 10.0) * std::min(1.0, (1.0 - t) * 6.0);
    if (s.shape == "leaflet") return std::pow(sn, 0.55) * (1.0 - 0.3 * t);
    if (s.shape == "lobed") {
        const double env = std::pow(sn, 0.6) * (0.55 + 0.45 * t * (2.0 - t));
        return env * (0.58 + 0.42 * std::fabs(std::sin(s.lobes * kPi * t)));
    }
    if (s.shape == "serrate") {
        const double env = std::pow(std::sin(kPi * std::pow(t, 0.7)), 0.85) * (1.08 - 0.42 * t);
        const double saw = t * s.teeth - std::floor(t * s.teeth);
        return env * (1.0 - 0.10 * saw);
    }
    return std::pow(sn, 0.8);   // oval
}

// THE POLAR SHAPES: an outline r(phi) about a centre, phi measured from the leaf's direction, r a fraction of the
// radius. A maple's palmate leaf; the flowers.
bool polarShape(const std::string& shape) {
    return shape == "palmate" || shape == "blossom" || shape == "daisy" || shape == "floret";
}
double wrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
// Inside the outline at (rho, phi)? (rho a fraction of the radius)
bool insidePolar(const std::string& shape, int lobes, int teeth, int petals, double notch, double eye, double rho,
                 double phi) {
    if (rho > 1.0) return false;
    if (shape == "palmate") {   // lobes fanned over +-140 deg, the middle one longest, toothed, deep sinuses
        const int n = std::max(3, lobes);
        const double spread = 0.78 * kPi, step = 2.0 * spread / (n - 1), w = step * 0.62;
        double r = 0.0;
        for (int k = 0; k < n; ++k) {
            const double pk = -spread + step * k;
            const double Lk = 1.0 - 0.32 * std::pow(std::fabs(pk) / spread, 1.5);
            const double pointed = std::max(0.0, 1.0 - std::fabs(wrapPi(phi - pk)) / w);
            r = std::max(r, 0.26 + (Lk - 0.26) * std::pow(pointed, 0.7));
        }
        const double saw = phi * std::max(1, teeth) * n / kPi;
        r *= 1.0 - 0.06 * (saw - std::floor(saw));
        return rho <= r;
    }
    const int n = std::max(1, petals);
    const double cell = 2.0 * kPi / n;
    const double d = wrapPi(phi) - cell * std::round(wrapPi(phi) / cell);   // from the nearest petal's midline
    const double u = d / (0.5 * cell);                                      // -1 .. 1 across the petal
    if (rho <= eye) return true;
    if (shape == "blossom") {   // round petals, a notch at each tip
        double r = 0.25 + 0.75 * std::pow(std::max(0.0, 1.0 - u * u), 0.35);
        r -= notch * std::max(0.0, 1.0 - std::fabs(u) / 0.22);
        return rho <= r;
    }
    if (shape == "daisy") {     // narrow rays with gaps between, pointed a little at the tip
        const double half = 0.62 * (1.0 - 0.35 * std::max(0.0, rho - 0.75) / 0.25);
        return std::fabs(u) <= half && rho <= 1.0 - 0.25 * std::fabs(u);
    }
    // floret: a hood above, a lip below -- two overlapping ovals along the flower's direction
    const double x = rho * std::cos(phi), y = rho * std::sin(phi);
    const double a = (x - 0.25) / 0.75, b = y / 0.42, c = (x + 0.15) / 0.55, e = (y + 0.18) / 0.5;
    return a * a + b * b <= 1.0 || c * c + e * e <= 1.0;
}

// Draw one card: expand the grammar with this variant's vars, walk it with the 2D turtle.
void drawTwig(const LeafCardSpec& s, uint32_t seed, int row, std::vector<Stem>& stems, std::vector<Leaf>& leaves) {
    std::map<std::string, double> vars;
    vars["row"] = row;   // the atlas row this card is drawn in: row 0 the tip cards (TreeParams::tipCards)
    for (const RecipeRange& g : s.vars) {
        uint32_t h = seed * 0x9E3779B9u;
        for (unsigned char c : g.name) h = (h ^ c) * 16777619u;
        vars[g.name] = g.lo + (g.hi - g.lo) * unit(h);
    }
    ParametricLSystem grammar;
    for (const RecipeRule& r : s.rules)
        grammar.rule(substituteVars(r.predecessor, vars), substituteVars(r.successor, vars), r.weight);
    const ModuleString mods = grammar.expand(parseModuleLiterals(substituteVars(s.axiom, vars)), s.iterations, seed);
    struct St { V2 p; double a; int depth; };
    St st{{0, 0}, kPi * 0.5, 0};   // heading up the card (+y)
    std::vector<St> stack;
    uint32_t h = seed * 2654435761u + 7u;
    for (const Module& m : mods) {
        const double arg = m.params.empty() ? 0.0 : m.params[0];
        switch (m.symbol) {
            case 'F': {
                const V2 d{std::cos(st.a), std::sin(st.a)};
                const V2 q = st.p + d * arg;
                stems.push_back({st.p, q, s.stemWidth * std::pow(0.75, st.depth)});
                st.p = q;
                break;
            }
            case '+': st.a += arg * kPi / 180.0; break;
            case '-': st.a -= arg * kPi / 180.0; break;
            case '[': stack.push_back(st); ++st.depth; break;
            case ']': if (!stack.empty()) { st = stack.back(); stack.pop_back(); } break;
            case 'W': {   // a flower
                if (!s.flower.set) break;
                const double sc = m.params.empty() ? 1.0 : arg;
                h = mixHash(h + 0x77u);
                Leaf lf;
                lf.base = st.p;
                lf.dir = {std::cos(st.a), std::sin(st.a)};
                lf.len = lf.wid = s.flower.size * sc;
                lf.shade = 1.0 - 0.5 * s.shadeJitter * (h / 4294967296.0);
                lf.flower = true;
                leaves.push_back(lf);
                break;
            }
            case 'L': {
                const double sc = m.params.empty() ? 1.0 : arg;
                h = mixHash(h + 0x51u);
                Leaf lf;
                lf.base = st.p;
                lf.dir = {std::cos(st.a), std::sin(st.a)};
                lf.len = s.leafLength * sc;
                lf.wid = s.leafWidth * sc;
                lf.shade = 1.0 - s.shadeJitter * (h / 4294967296.0);
                leaves.push_back(lf);
                break;
            }
            default: break;
        }
    }
}

}  // namespace

LeafCardAtlas leafCardAtlas(const LeafCardSpec& spec, uint32_t seed, int tileW, int tileH) {
    LeafCardAtlas out;
    const int n = std::max(1, spec.variants);
    out.tiles = n;
    const int W = tileW * n, H = tileH * n;
    out.albedo.width = out.normal.width = W;
    out.albedo.height = out.normal.height = H;
    out.albedo.channels = out.normal.channels = 4;
    out.albedo.pixels.assign(static_cast<std::size_t>(W) * H * 4, 0);
    out.normal.pixels.assign(static_cast<std::size_t>(W) * H * 4, 0);
    constexpr int SS = 2;   // supersampling a side
    for (int tile = 0; tile < n * n; ++tile) {
        std::vector<Stem> stems;
        std::vector<Leaf> leaves;
        drawTwig(spec, seed * 7919u + static_cast<uint32_t>(tile) * 104729u + 1u, tile / n, stems, leaves);
        if (stems.empty() && leaves.empty()) continue;
        // the drawing's extent: |x| about the base, y from its lowest to its highest
        double maxX = 1e-6, minY = 0.0, maxY = 1e-6;
        auto grow = [&](V2 p, double pad) {
            maxX = std::max(maxX, std::fabs(p.x) + pad);
            minY = std::min(minY, p.y - pad);
            maxY = std::max(maxY, p.y + pad);
        };
        for (const Stem& s : stems) { grow(s.a, s.w); grow(s.b, s.w); }
        // a polar part's centre and radius (a maple leaf's blade beyond its stalk; a flower round its centre)
        auto polarFrame = [&](const Leaf& l, V2& c, double& R) {
            if (l.flower) { R = 0.5 * l.len; c = l.base + l.dir * (0.8 * R); }
            else { R = 0.58 * l.len; c = l.base + l.dir * (0.42 * l.len); }
        };
        for (const Leaf& l : leaves) {
            if (l.flower || polarShape(spec.shape)) {
                V2 c; double R;
                polarFrame(l, c, R);
                grow(c, R);
                grow(l.base, 0.0);
                continue;
            }
            const V2 side{-l.dir.y, l.dir.x};
            for (double t = 0.0; t <= 1.0; t += 0.1)
                for (double sgn : {-1.0, 1.0}) grow(l.base + l.dir * (l.len * t) + side * (sgn * l.wid * halfWidth(spec, t)), 0.0);
            grow(l.base + l.dir * l.len, 0.0);
        }
        // fitted inside a margin, one scale for both axes (the card's own aspect is the tile's)
        const double margin = 0.06;
        const double sx = (0.5 - margin) * tileW / maxX, sy = (1.0 - 2.0 * margin) * tileH / (maxY - minY);
        const double sc = std::min(sx, sy);
        const double ox = 0.5 * tileW, oy = margin * tileH - minY * sc;
        auto toPx = [&](V2 p) { return V2{ox + p.x * sc, oy + p.y * sc}; };
        // supersampled coverage, brightness and normal for this tile
        const int sw = tileW * SS, sh = tileH * SS;
        std::vector<float> cov(static_cast<std::size_t>(sw) * sh, 0.0f);
        std::vector<Vec3> col(cov.size(), Vec3(0, 0, 0));
        std::vector<float> nx(cov.size(), 0.0f), ny(cov.size(), 0.0f);
        // (each part's brightness times its tint: white tints leave the card the grey the vertex colour tints)
        Vec3 tint = spec.stemTint;
        auto plot = [&](int x, int y, float b, float ax, float ay) {
            if (x < 0 || y < 0 || x >= sw || y >= sh) return;
            const std::size_t k = static_cast<std::size_t>(y) * sw + x;
            cov[k] = 1.0f; col[k] = tint * b; nx[k] = ax; ny[k] = ay;
        };
        for (const Stem& s : stems) {   // a stroke: every sample within half its width of the segment
            const V2 a = toPx(s.a) * SS, b = toPx(s.b) * SS;
            const double r = std::max(0.75, s.w * sc * SS * 0.5);
            const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) - r)), x1 = static_cast<int>(std::ceil(std::max(a.x, b.x) + r));
            const int y0 = static_cast<int>(std::floor(std::min(a.y, b.y) - r)), y1 = static_cast<int>(std::ceil(std::max(a.y, b.y) + r));
            const V2 ab = b - a;
            const double L2 = std::max(1e-9, ab.x * ab.x + ab.y * ab.y);
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const V2 p{x + 0.5, y + 0.5};
                    const double t = std::clamp(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / L2, 0.0, 1.0);
                    const V2 q = a + ab * t;
                    if ((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) <= r * r) plot(x, y, 0.55f, 0.0f, 0.0f);
                }
        }
        for (const Leaf& l : leaves) {   // a polar part: a maple's palmate blade, a flower
            if (!(l.flower || polarShape(spec.shape))) continue;
            V2 cw; double Rw;
            polarFrame(l, cw, Rw);
            const V2 c = toPx(cw) * SS, base = toPx(l.base) * SS;
            const double R = Rw * sc * SS;
            const double ang0 = std::atan2(l.dir.y, l.dir.x);
            const auto& fl = spec.flower;
            if (!l.flower) {   // the maple's stalk, base to blade
                tint = spec.stemTint;
                const V2 ab = c - base;
                const double L2 = std::max(1e-9, ab.x * ab.x + ab.y * ab.y), r = std::max(0.75, spec.stemWidth * 0.4 * sc * SS);
                for (int y = static_cast<int>(std::min(base.y, c.y) - r); y <= static_cast<int>(std::max(base.y, c.y) + r); ++y)
                    for (int x = static_cast<int>(std::min(base.x, c.x) - r); x <= static_cast<int>(std::max(base.x, c.x) + r); ++x) {
                        const V2 p{x + 0.5, y + 0.5};
                        const double t = std::clamp(((p.x - base.x) * ab.x + (p.y - base.y) * ab.y) / L2, 0.0, 1.0);
                        const V2 q = base + ab * t;
                        if ((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) <= r * r) plot(x, y, 0.6f, 0.0f, 0.0f);
                    }
            }
            const double rot = l.flower ? (l.shade * 7.0) : 0.0;   // (each flower turned its own way)
            for (int y = static_cast<int>(c.y - R - 1); y <= static_cast<int>(c.y + R + 1); ++y)
                for (int x = static_cast<int>(c.x - R - 1); x <= static_cast<int>(c.x + R + 1); ++x) {
                    const double px = x + 0.5 - c.x, py = y + 0.5 - c.y;
                    const double rho = std::sqrt(px * px + py * py) / std::max(1e-6, R);
                    const double phi = std::atan2(py, px) - ang0 - (fl.shape == "floret" ? 0.0 : rot);
                    const bool in = l.flower ? insidePolar(fl.shape, 0, 0, fl.petals, fl.notch, fl.eye, rho, phi)
                                             : insidePolar(spec.shape, spec.lobes, spec.teeth, 0, 0, 0, rho, phi);
                    if (!in) continue;
                    float b;
                    if (l.flower) {
                        // (a floret's lower lip wears the eye colour: a 'Hot Lips' salvia's red under white)
                        bool eyeHere = fl.shape != "floret" && rho <= fl.eye;
                        if (fl.shape == "floret") {
                            const double fx = rho * std::cos(phi), fy = rho * std::sin(phi);
                            const double a = (fx - 0.25) / 0.75, bb = fy / 0.42;
                            eyeHere = a * a + bb * bb > 1.0;   // outside the hood: the lip
                        }
                        tint = eyeHere ? fl.eyeTint : fl.tint;
                        b = static_cast<float>(l.shade * (eyeHere ? 0.9 : 0.82 + 0.18 * rho));
                    } else {
                        tint = spec.leafTint;
                        // the veins: darker rays out from the stalk along each lobe
                        const int n = std::max(3, spec.lobes);
                        const double step = 2.0 * 0.78 * kPi / (n - 1);
                        const double dv = std::fabs(wrapPi(phi + 0.78 * kPi) - step * std::round(wrapPi(phi + 0.78 * kPi) / step));
                        b = static_cast<float>(l.shade * (0.86 + 0.14 * rho) * (dv * rho * R < 0.9 * SS ? 0.82 : 1.0));
                    }
                    const double cupk = (l.flower ? 0.6 : spec.cup) * rho;   // cupped: the normal tips out from the centre
                    const double rx = px / std::max(1e-6, rho * R), ry = py / std::max(1e-6, rho * R);
                    plot(x, y, b, static_cast<float>(rx * cupk), static_cast<float>(ry * cupk));
                }
        }
        tint = spec.leafTint;
        for (const Leaf& l : leaves) {   // a leaf: its outline in its own frame, the midrib darker, cupped
            if (l.flower || polarShape(spec.shape)) continue;
            const V2 base = toPx(l.base) * SS;
            const V2 d = l.dir, side{-d.y, d.x};
            const double len = l.len * sc * SS, wid = l.wid * sc * SS;
            const double reach = std::max(len, wid) + 2.0;
            const int x0 = static_cast<int>(base.x - reach), x1 = static_cast<int>(base.x + reach);
            const int y0 = static_cast<int>(base.y - reach), y1 = static_cast<int>(base.y + reach);
            const bool petiole = spec.shape != "needle";
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    const V2 p = V2{x + 0.5, y + 0.5} - base;
                    const double along = p.x * d.x + p.y * d.y, across = p.x * side.x + p.y * side.y;
                    const double t = along / len;
                    if (t < 0.0 || t > 1.0) continue;
                    if (petiole && t < 0.08) {   // the leaf's stalk
                        tint = spec.stemTint;
                        if (std::fabs(across) <= std::max(0.75, spec.stemWidth * 0.4 * sc * SS)) plot(x, y, 0.6f, 0.0f, 0.0f);
                        tint = spec.leafTint;
                        continue;
                    }
                    const double hw = wid * halfWidth(spec, petiole ? (t - 0.08) / 0.92 : t);
                    if (std::fabs(across) > hw || hw <= 0.0) continue;
                    const double f = across / hw;   // -1 .. 1 across the leaf
                    float b = static_cast<float>(l.shade * (0.86 + 0.14 * t));
                    if (spec.tipBright > 0.0) {   // the new growth at the shoots' ends: lighter up the card
                        const double fy = std::clamp((l.base.y - minY) / std::max(1e-6, maxY - minY), 0.0, 1.0);
                        b *= static_cast<float>(1.0 - spec.tipBright * (1.0 - fy * fy));
                    }
                    if (std::fabs(f) < 0.07 && spec.shape != "needle") b *= 0.8f;   // the midrib
                    // cupped about the midrib: the normal tips across the leaf, in the card's (x, y) frame
                    const double tip = spec.cup * f;
                    plot(x, y, b, static_cast<float>(side.x * tip), static_cast<float>(side.y * tip));
                }
        }
        // down to the tile: coverage the alpha, brightness and normal its average over the covered samples
        const int tx = (tile % n) * tileW, ty = (tile / n) * tileH;
        for (int y = 0; y < tileH; ++y)
            for (int x = 0; x < tileW; ++x) {
                float c = 0, ax = 0, ay = 0;
                Vec3 b(0, 0, 0);
                for (int j = 0; j < SS; ++j)
                    for (int i = 0; i < SS; ++i) {
                        const std::size_t k = static_cast<std::size_t>(y * SS + j) * sw + (x * SS + i);
                        if (cov[k] <= 0.0f) continue;
                        c += 1.0f; b = b + col[k]; ax += nx[k]; ay += ny[k];
                    }
                // (the card is drawn with v up: row 0 is the card's foot)
                const std::size_t o = (static_cast<std::size_t>(ty + y) * W + (tx + x)) * 4;
                if (c > 0) { b = b * (1.0 / c); ax /= c; ay /= c; }
                const float a = c / (SS * SS);
                auto byte = [](double v) { return static_cast<uint8_t>(std::clamp(v, 0.0, 1.0) * 255.0 + 0.5); };
                out.albedo.pixels[o + 0] = byte(b.x);
                out.albedo.pixels[o + 1] = byte(b.y);
                out.albedo.pixels[o + 2] = byte(b.z);
                out.albedo.pixels[o + 3] = static_cast<uint8_t>(a * 255.0f + 0.5f);
                const double nz = std::sqrt(std::max(0.0, 1.0 - double(ax) * ax - double(ay) * ay));
                out.normal.pixels[o + 0] = static_cast<uint8_t>((ax * 0.5 + 0.5) * 255.0 + 0.5);
                out.normal.pixels[o + 1] = static_cast<uint8_t>((ay * 0.5 + 0.5) * 255.0 + 0.5);
                out.normal.pixels[o + 2] = static_cast<uint8_t>((nz * 0.5 + 0.5) * 255.0 + 0.5);
                out.normal.pixels[o + 3] = 255;
            }
    }
    // colour under the cut: grown out from the leaves a few texels, so the mips' averages stay leaf-bright
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<uint8_t> next = out.albedo.pixels;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const std::size_t o = (static_cast<std::size_t>(y) * W + x) * 4;
                auto filled = [&](std::size_t q) {
                    const uint8_t* px = &out.albedo.pixels[q];
                    return px[3] > 0 || px[0] > 0 || px[1] > 0 || px[2] > 0;
                };
                if (filled(o)) continue;
                int sum[3] = {0, 0, 0}, cnt = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int xx = x + dx, yy = y + dy;
                        if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
                        const std::size_t q = (static_cast<std::size_t>(yy) * W + xx) * 4;
                        if (filled(q)) { for (int ch = 0; ch < 3; ++ch) sum[ch] += out.albedo.pixels[q + ch]; ++cnt; }
                    }
                if (cnt) for (int ch = 0; ch < 3; ++ch) next[o + ch] = static_cast<uint8_t>(sum[ch] / cnt);
            }
        out.albedo.pixels.swap(next);
    }
    return out;
}

}  // namespace engine
