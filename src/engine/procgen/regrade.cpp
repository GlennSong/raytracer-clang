#include "regrade.h"
#include "texture_field.h"            // boxBlurGrid, chamferSweepGrid
#include "city/polyline.h"            // distToSegment
#include "city/roads/ground_grid.h"   // the levelled patch: absolute heights on a grid
#include "../../job_system.h"         // parallelRows: the shared pool (ADR-0288)
#include <algorithm>
#include <cmath>
#include <memory>

namespace engine {

std::vector<Regrade> regradesFromJson(const nlohmann::json& t) {
    std::vector<Regrade> out;
    if (!t.contains("regrade") || !t["regrade"].is_array()) return out;
    for (const nlohmann::json& r : t["regrade"]) {
        Regrade g;
        if (r.contains("outline") && r["outline"].is_array())
            for (const nlohmann::json& q : r["outline"]) g.outline.push_back(Vec2(q[0].get<double>(), q[1].get<double>()));
        g.blur = r.value("blur", g.blur);
        g.maxSlope = r.value("maxSlope", g.maxSlope);
        g.terrace = r.value("terrace", g.terrace);
        g.bank = r.value("bank", g.bank);
        g.feather = r.value("feather", g.feather);
        g.cell = r.value("cell", g.cell);
        if (g.outline.size() >= 3) out.push_back(std::move(g));
    }
    return out;
}

namespace {

// One regrade's patch: the levelled height, and how much of it holds (1 inside the outline, easing to 0 across the
// feather), on the same grid.
struct RegradePatch {
    roads::GroundGrid level, weight;
};

std::shared_ptr<const RegradePatch> buildPatch(const HeightField& base, const Regrade& g) {
    double lx = 1e30, lz = 1e30, hx = -1e30, hz = -1e30;
    for (const Vec2& q : g.outline) { lx = std::min(lx, q.x); lz = std::min(lz, q.y); hx = std::max(hx, q.x); hz = std::max(hz, q.y); }
    // the grid reaches past the feather by the blur, so the levelled ground near the edge is blurred over real ground
    const double margin = g.feather + g.blur, cell = std::max(1.0, g.cell);
    const double side = std::max(hx - lx, hz - lz) + 2.0 * margin;
    const int n = static_cast<int>(std::ceil(side / cell)) + 1;
    const double x0 = 0.5 * (lx + hx) - 0.5 * (n - 1) * cell, z0 = 0.5 * (lz + hz) - 0.5 * (n - 1) * cell;
    auto patch = std::make_shared<RegradePatch>();
    for (roads::GroundGrid* G : {&patch->level, &patch->weight}) {
        G->x0 = x0; G->y0 = z0; G->res = cell; G->nx = n; G->ny = n;
        G->z.assign(static_cast<std::size_t>(n) * n, 0.0);
    }
    std::vector<float> h(static_cast<std::size_t>(n) * n);
    parallelRows(n, [&](int j) {
        for (int i = 0; i < n; ++i) h[static_cast<std::size_t>(j) * n + i] = static_cast<float>(base(x0 + i * cell, z0 + j * cell));
    });
    // 1. the blur: three box passes are near a Gaussian of about the window's half-width
    const int r = std::max(1, static_cast<int>(std::lround(0.5 * g.blur / cell)));
    const std::vector<float> smooth = boxBlurGrid(h, n, r, 3);
    // 2. the slope cap: the cut (no steeper, never above) and the fill (no steeper, never below), each finished by
    // the other, averaged -- a mean of two capped grounds is capped too
    const float a1 = static_cast<float>(g.maxSlope * cell), a2 = static_cast<float>(g.maxSlope * cell * std::sqrt(2.0));
    auto cut = [&](std::vector<float>& v) { chamferSweepGrid(v, n, a1, a2); };
    auto fill = [&](std::vector<float>& v) {
        for (float& x : v) x = -x;
        chamferSweepGrid(v, n, a1, a2);
        for (float& x : v) x = -x;
    };
    std::vector<float> cutFirst = smooth, fillFirst = smooth;
    cut(cutFirst); fill(cutFirst);
    fill(fillFirst); cut(fillFirst);
    parallelRows(n, [&](int j) {
        for (int i = 0; i < n; ++i) {
            const std::size_t k = static_cast<std::size_t>(j) * n + i;
            double v = 0.5 * (static_cast<double>(cutFirst[k]) + fillFirst[k]);
            // 3. the shelves
            if (g.terrace > 0.0) v = terraceHeight(v, g.terrace, g.bank);
            patch->level.z[k] = v;
            // the weight: whole inside the outline, a smoothstep out across the feather
            const Vec2 p(x0 + i * cell, z0 + j * cell);
            double w = 1.0;
            if (!pointInPolygon(g.outline, p)) {
                double d = 1e30;
                for (std::size_t e = 0; e < g.outline.size(); ++e)
                    d = std::min(d, distToSegment(p, g.outline[e], g.outline[(e + 1) % g.outline.size()]));
                const double t = std::clamp(1.0 - d / std::max(1.0, g.feather), 0.0, 1.0);
                w = t * t * (3.0 - 2.0 * t);
            }
            patch->weight.z[k] = w;
        }
    });
    return patch;
}

}  // namespace

HeightField regradeField(HeightField base, const std::vector<Regrade>& regrades) {
    for (const Regrade& g : regrades) {
        std::shared_ptr<const RegradePatch> patch = buildPatch(base, g);
        base = [base, patch](double x, double z) {
            const double h = base(x, z);
            if (!patch->weight.inside(x, z)) return h;
            const double w = patch->weight.sample(x, z);
            return w <= 0.0 ? h : h + w * (patch->level.sample(x, z) - h);
        };
    }
    return base;
}

}  // namespace engine
