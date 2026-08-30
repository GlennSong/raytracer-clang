#include "block_grade.h"

#include <cmath>

namespace engine {

namespace {

// Evenly sample `count` points around a polygon's perimeter, each nudged `inset`
// metres toward the centroid (so the height is read off the block, not the curb).
std::vector<Vec2> boundarySamples(const Poly2& poly, int count, double inset) {
    std::vector<Vec2> out;
    if (poly.size() < 3 || count < 3) return out;
    Vec2 c = centroid(poly);
    // Total perimeter for even arc-length spacing.
    double perim = 0.0;
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) perim += (poly[(i + 1) % n] - poly[i]).length();
    if (perim < 1e-6) return out;
    double step = perim / count, acc = 0.0, next = 0.0;
    for (int i = 0; i < n && static_cast<int>(out.size()) < count; ++i) {
        Vec2 a = poly[i], b = poly[(i + 1) % n];
        double segLen = (b - a).length();
        while (next <= acc + segLen && static_cast<int>(out.size()) < count) {
            double t = segLen > 1e-9 ? (next - acc) / segLen : 0.0;
            Vec2 p = a + (b - a) * t;
            Vec2 toC = c - p;
            double d = toC.length();
            if (d > 1e-6) p = p + toC * (std::min(inset, d * 0.5) / d);  // nudge inward
            out.push_back(p);
            next += step;
        }
        acc += segLen;
    }
    return out;
}

// Least-squares plane y = c + dx*x + dz*z through (x, z, y) samples. Solves the
// 3x3 normal equations by Cramer's rule; falls back to the mean height (flat) when
// the samples are degenerate (collinear / a point).
bool fitPlane(const std::vector<Vec2>& pts, const std::vector<double>& ys,
              double& c, double& dx, double& dz) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) return false;
    double Sxx = 0, Sxz = 0, Sx = 0, Szz = 0, Sz = 0, Sxy = 0, Szy = 0, Sy = 0;
    for (int i = 0; i < n; ++i) {
        double x = pts[i].x, z = pts[i].y, y = ys[i];
        Sxx += x * x; Sxz += x * z; Sx += x;
        Szz += z * z; Sz += z;
        Sxy += x * y; Szy += z * y; Sy += y;
    }
    double N = n;
    // A = [[Sxx,Sxz,Sx],[Sxz,Szz,Sz],[Sx,Sz,N]], b = [Sxy,Szy,Sy], solve A*[dx,dz,c].
    double det = Sxx * (Szz * N - Sz * Sz) - Sxz * (Sxz * N - Sz * Sx) +
                 Sx * (Sxz * Sz - Szz * Sx);
    if (std::fabs(det) < 1e-6) { c = Sy / N; dx = dz = 0.0; return true; }
    double detDx = Sxy * (Szz * N - Sz * Sz) - Sxz * (Szy * N - Sz * Sy) +
                   Sx * (Szy * Sz - Szz * Sy);
    double detDz = Sxx * (Szy * N - Sz * Sy) - Sxy * (Sxz * N - Sz * Sx) +
                   Sx * (Sxz * Sy - Szy * Sx);
    double detC = Sxx * (Szz * Sy - Szy * Sz) - Sxz * (Sxz * Sy - Szy * Sx) +
                  Sxy * (Sxz * Sz - Szz * Sx);
    dx = detDx / det; dz = detDz / det; c = detC / det;
    return true;
}

TerrainFlatten planePad(const Poly2& block, double c, double dx, double dz, double falloff) {
    std::vector<Vec3> poly;
    poly.reserve(block.size());
    for (const Vec2& v : block) poly.push_back(Vec3(v.x, 0, v.y));
    TerrainFlatten f = makeFlattenPad(std::move(poly), c, falloff);  // sets AABB
    f.dx = dx; f.dz = dz;   // tilt the target plane to grade with the streets
    f.priority = -1;        // roads + building pads (priority 0) override the block grade
    return f;
}

}  // namespace

std::vector<TerrainFlatten> gradeBlock(const Poly2& block, const HeightSampler& ground,
                                       const BlockGradeParams& p) {
    std::vector<TerrainFlatten> out;
    if (block.size() < 3 || !ground) return out;
    std::vector<Vec2> pts = boundarySamples(block, p.boundarySamples, p.inset);
    if (pts.size() < 3) return out;
    std::vector<double> ys;
    ys.reserve(pts.size());
    for (const Vec2& q : pts) ys.push_back(ground(q.x, q.y));
    double c, dx, dz;
    if (!fitPlane(pts, ys, c, dx, dz)) return out;

    // How much does the fitted plane rise/fall across the block?
    double lo = 1e30, hi = -1e30;
    for (const Vec2& v : block) {
        double y = c + dx * v.x + dz * v.y;
        lo = std::min(lo, y); hi = std::max(hi, y);
    }
    double range = hi - lo;

    if (range <= p.maxDrop || area(block) < p.minTerraceArea) {
        out.push_back(planePad(block, c, dx, dz, p.falloff));   // one tilted plane
        return out;
    }

    // Too steep for one plane: TERRACE into flat steps banded along the gradient.
    double glen = std::sqrt(dx * dx + dz * dz);
    if (glen < 1e-9) { out.push_back(planePad(block, c, dx, dz, p.falloff)); return out; }
    Vec2 gdir(dx / glen, dz / glen);   // unit uphill direction
    double t0 = 1e30, t1 = -1e30;
    for (const Vec2& v : block) { double t = dot(gdir, v); t0 = std::min(t0, t); t1 = std::max(t1, t); }
    int nTerraces = std::max(2, static_cast<int>(std::ceil(range / p.maxDrop)));
    double bw = (t1 - t0) / nTerraces;
    // Each band keeps a REDUCED tilt: the fitted gradient, scaled so the band
    // rises at most maxBandRise across its own width, centred on the band's
    // mid-height so the steps still climb the slope. The riser between bands
    // is the remainder. (Keeping the FULL gradient would reproduce the block
    // plane exactly and the steps would vanish; snapping flat — the old code —
    // was the staircase.)
    const double gStep = std::min(glen, p.maxBandRise / std::max(1e-6, bw));
    // CENTRED ON THE MID-HEIGHT: each band's plane passes through the fitted
    // plane on the band's centre line, so no band strays more than
    // (glen - gStep) * bw / 2 from the plane the lots are built against.
    // Anchoring the outer bands to the streets instead (risers carrying all
    // the remainder inside the block) was measured on metro_v2 and was worse
    // on every gate: banks beside roads 1314 -> 2571, pokes 128/1282/2237 ->
    // 156/1620/2758, 15 fewer buildings, relief rejections doubled — the
    // bands strayed up to (glen - gStep) * bw from the plane and the lots on
    // them paid. The cost of centring is a curb-step at the downhill street of
    // (glen - gStep) * bw / 2, which MORE tilt shrinks: that is what
    // maxBandRise trades, and why it is 3 m rather than 1.5.
    for (int i = 0; i < nTerraces; ++i) {
        double a = t0 + i * bw, b = t0 + (i + 1) * bw;
        // Clip the block to the slab a <= gdir·p <= b: two half-plane cuts.
        Poly2 band = clipHalfPlane(block, gdir, b);
        band = clipHalfPlane(band, Vec2(-gdir.x, -gdir.y), -a);
        if (band.size() < 3 || area(band) < 1.0) continue;
        const double mid = (a + b) * 0.5;
        const double level = c + glen * mid;          // the band's mid-height
        // planeY(p) = cBand + gStep * (gdir . p): equals `level` on the band's
        // centre line and rises gStep * bw across the band.
        const double cBand = level - gStep * mid;
        out.push_back(planePad(band, cBand, gStep * gdir.x, gStep * gdir.y, p.falloff));
    }
    if (out.empty()) out.push_back(planePad(block, c, dx, dz, p.falloff));  // clip failed: fall back
    return out;
}

std::vector<TerrainFlatten> gradeBlocks(const std::vector<Poly2>& blocks,
                                        const HeightSampler& ground,
                                        const BlockGradeParams& p) {
    std::vector<TerrainFlatten> out;
    for (const Poly2& b : blocks) {
        std::vector<TerrainFlatten> g = gradeBlock(b, ground, p);
        out.insert(out.end(), g.begin(), g.end());
    }
    return out;
}

}  // namespace engine
