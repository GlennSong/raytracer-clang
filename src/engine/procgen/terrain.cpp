#include "terrain.h"
#include "../mesh_builder.h"
#include "lsystem.h"
#include "skeleton.h"
#include "../../curve.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace engine {

namespace {
double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
double smoothstep(double e0, double e1, double x) {
    double t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}
Vec3 mixv(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }
}  // namespace

namespace {
// 2D point-in-polygon (ray cast) over a footprint's XZ, and the distance from a
// point to the polygon's boundary — together they drive the flatten falloff.
bool pointInFootprint(const std::vector<Vec3>& poly, double x, double z) {
    bool in = false;
    size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i].x, zi = poly[i].z;
        double xj = poly[j].x, zj = poly[j].z;
        if (((zi > z) != (zj > z)) &&
            (x < (xj - xi) * (z - zi) / (zj - zi) + xi))
            in = !in;
    }
    return in;
}
double distanceToFootprint(const std::vector<Vec3>& poly, double x, double z) {
    double best = std::numeric_limits<double>::max();
    size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double ax = poly[j].x, az = poly[j].z;
        double bx = poly[i].x, bz = poly[i].z;
        double ex = bx - ax, ez = bz - az;
        double len2 = ex * ex + ez * ez;
        double t = len2 > 1e-12 ? ((x - ax) * ex + (z - az) * ez) / len2 : 0.0;
        t = clamp01(t);
        double dx = x - (ax + ex * t), dz = z - (az + ez * t);
        best = std::min(best, std::sqrt(dx * dx + dz * dz));
    }
    return best;
}
}  // namespace

// Blend the flatten footprints over a natural height. The strongest (closest)
// footprint wins, so overlapping road/block stamps don't fight; inside a
// footprint the weight is 1 (fully levelled), easing to 0 across `falloff`.
// `dilate` grows every footprint by that many metres for THIS query — a coarse
// CDLOD node samples with dilate ≈ half its cell so a triangle whose corners
// both land outside a narrow road corridor can't interpolate natural ground
// straight across the carriageway (device: "terrain poked through in places").
// Accumulator for one flatten query: fold each candidate region in, tracking the
// lowest overlapping plane (inside wins, lowest of overlaps) and, failing that,
// the strongest falloff blend. Shared by the linear and indexed applyFlatten so
// the two produce bit-for-bit identical heights (only the candidate SET differs).
namespace {
struct FlattenAccum {
    double base;
    double result;
    double bestW = 0.0;
    bool insideAny = false;
    int insidePriority = std::numeric_limits<int>::min();
    double insideMin = std::numeric_limits<double>::max();
    explicit FlattenAccum(double b) : base(b), result(b) {}
    // Fold one inside footprint: the highest priority wins; among equal priority
    // the lowest plane wins (junction rule). Lower-priority footprints are ignored
    // when a higher one already covers the point.
    void foldInside(const TerrainFlatten& r, double x, double z) {
        insideAny = true;
        double y = r.planeY(x, z);
        if (r.priority > insidePriority) { insidePriority = r.priority; insideMin = y; }
        else if (r.priority == insidePriority) insideMin = std::min(insideMin, y);
    }
    void fold(const TerrainFlatten& r, double x, double z, double dilate) {
        if (r.polygon.size() < 3) return;
        const double reach = r.falloff + dilate;
        if (x < r.minX - reach || x > r.maxX + reach ||
            z < r.minZ - reach || z > r.maxZ + reach)
            return;
        if (pointInFootprint(r.polygon, x, z)) { foldInside(r, x, z); return; }
        double d = distanceToFootprint(r.polygon, x, z);
        if (dilate > 0.0 && d <= dilate) { foldInside(r, x, z); return; }  // grown footprint
        if (insideAny) return;   // an inside hit already owns this point
        d -= dilate;
        if (d >= r.falloff) return;
        // Closeness weight: the nearest footprint owns an outside point (its edge
        // treatment dominates the feather/batter of a farther one).
        double w = 1.0 - smoothstep(0.0, r.falloff, d);
        if (w <= bestW) return;
        bestW = w;
        if (r.falloffMode == TerrainFlatten::Falloff::DaylightBatter) {
            // Earthwork cross-section: leave the deck edge (planeY is constant
            // across the corridor, so it reads the deck height at this station) on
            // a cut/fill batter and daylight where it meets natural ground.
            // `base` IS the natural ground here, so min/max clamps at daylight.
            double edgeY = r.planeY(x, z);
            if (base > edgeY)                      // uphill: cut a slope INTO the hill
                result = std::min(edgeY + r.cutBatter * d, base);
            else                                    // downhill: FILL an embankment out
                result = std::max(edgeY - r.fillBatter * d, base);
        } else {
            result = base + (r.planeY(x, z) - base) * w;   // original smoothstep feather
        }
    }
    double value() const { return insideAny ? insideMin : result; }
};
}  // namespace

double applyFlatten(const std::vector<TerrainFlatten>& regions, double x,
                    double z, double base, double dilate) {
    // Inside OVERLAPPING footprints — every junction interior — the ground
    // takes the LOWEST plane, not the first region in list order: two roads
    // crossing on a slope disagree about the junction's height, and only the
    // lower target keeps both decks proud of the ground (device: "the road
    // is being buried by the terrain — it's not conforming").
    FlattenAccum acc(base);
    for (const TerrainFlatten& r : regions) acc.fold(r, x, z, dilate);
    return acc.value();
}

double applyFlatten(const std::vector<TerrainFlatten>& regions, double x,
                    double z, double base) {
    return applyFlatten(regions, x, z, base, 0.0);
}

FlattenGrid buildFlattenGrid(const std::vector<TerrainFlatten>& regions) {
    FlattenGrid g;
    // Union AABB over the *falloff-expanded* footprints — the index only needs
    // to span where edits (plus their blend skirts) actually reach.
    double minX = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();
    int live = 0;
    for (const TerrainFlatten& r : regions) {
        if (r.polygon.size() < 3) continue;
        ++live;
        minX = std::min(minX, r.minX - r.falloff);
        minZ = std::min(minZ, r.minZ - r.falloff);
        maxX = std::max(maxX, r.maxX + r.falloff);
        maxZ = std::max(maxZ, r.maxZ + r.falloff);
    }
    if (live == 0) return g;   // empty grid => callers fall back to linear scan
    double spanX = std::max(1.0, maxX - minX), spanZ = std::max(1.0, maxZ - minZ);
    // Aim for a grid no coarser than the footprints and no finer than ~256 cells
    // per axis, so the bins stay small without exploding memory on big worlds.
    double cell = std::max(std::max(spanX, spanZ) / 256.0, 8.0);
    g.originX = minX;
    g.originZ = minZ;
    g.cell = cell;
    g.nx = std::max(1, static_cast<int>(std::ceil(spanX / cell)));
    g.nz = std::max(1, static_cast<int>(std::ceil(spanZ / cell)));
    g.bins.assign(static_cast<size_t>(g.nx) * g.nz, {});
    auto clampi = [](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
    for (int i = 0; i < static_cast<int>(regions.size()); ++i) {
        const TerrainFlatten& r = regions[i];
        if (r.polygon.size() < 3) continue;
        int x0 = clampi(static_cast<int>((r.minX - r.falloff - g.originX) / cell), 0, g.nx - 1);
        int x1 = clampi(static_cast<int>((r.maxX + r.falloff - g.originX) / cell), 0, g.nx - 1);
        int z0 = clampi(static_cast<int>((r.minZ - r.falloff - g.originZ) / cell), 0, g.nz - 1);
        int z1 = clampi(static_cast<int>((r.maxZ + r.falloff - g.originZ) / cell), 0, g.nz - 1);
        for (int cz = z0; cz <= z1; ++cz)
            for (int cx = x0; cx <= x1; ++cx)
                g.bins[static_cast<size_t>(cz) * g.nx + cx].push_back(i);
    }
    return g;
}

void rebuildFlattenIndex(TerrainParams& params) {
    if (params.flatten.empty()) { params.flattenIndex.reset(); return; }
    params.flattenIndex =
        std::make_shared<const FlattenGrid>(buildFlattenGrid(params.flatten));
}

double applyFlatten(const FlattenGrid& grid, const std::vector<TerrainFlatten>& regions,
                    double x, double z, double base, double dilate) {
    if (grid.empty()) return applyFlatten(regions, x, z, base, dilate);
    FlattenAccum acc(base);
    // A region binned by its falloff-AABB can still be relevant to a query up to
    // `dilate` beyond it (the dilated overload grows every footprint), so widen
    // the cell window by ceil(dilate/cell). fold() re-tests the exact AABB, so
    // over-inclusion only costs a few extra checks — never a wrong result.
    int pad = dilate > 0.0 ? static_cast<int>(std::ceil(dilate / grid.cell)) : 0;
    int cx = static_cast<int>((x - grid.originX) / grid.cell);
    int cz = static_cast<int>((z - grid.originZ) / grid.cell);
    int x0 = std::max(0, cx - pad), x1 = std::min(grid.nx - 1, cx + pad);
    int z0 = std::max(0, cz - pad), z1 = std::min(grid.nz - 1, cz + pad);
    if (cx + pad < 0 || cx - pad > grid.nx - 1 || cz + pad < 0 || cz - pad > grid.nz - 1)
        return base;   // wholly outside the indexed region: no edit reaches here
    // A region can span several bins; fold each at most once per query.
    for (int gz = z0; gz <= z1; ++gz)
        for (int gx = x0; gx <= x1; ++gx)
            for (int idx : grid.bins[static_cast<size_t>(gz) * grid.nx + gx]) {
                const TerrainFlatten& r = regions[idx];
                // Cheap de-dup: only fold a multi-bin region from its lowest-index
                // covering cell in the window (its min bin corner), so we don't
                // fold it twice. Recompute that corner and skip if it's < window.
                int rx0 = static_cast<int>((r.minX - r.falloff - grid.originX) / grid.cell);
                int rz0 = static_cast<int>((r.minZ - r.falloff - grid.originZ) / grid.cell);
                int fx = std::max(rx0, x0), fz = std::max(rz0, z0);
                if (fx != gx || fz != gz) continue;   // folded (or will be) by its corner cell
                acc.fold(r, x, z, dilate);
            }
    return acc.value();
}

static void footprintBounds(TerrainFlatten& f) {
    f.minX = f.minZ = std::numeric_limits<double>::max();
    f.maxX = f.maxZ = std::numeric_limits<double>::lowest();
    for (const Vec3& p : f.polygon) {
        f.minX = std::min(f.minX, static_cast<double>(p.x));
        f.maxX = std::max(f.maxX, static_cast<double>(p.x));
        f.minZ = std::min(f.minZ, static_cast<double>(p.z));
        f.maxZ = std::max(f.maxZ, static_cast<double>(p.z));
    }
}

TerrainFlatten makeFlattenPad(std::vector<Vec3> polygon, double targetY,
                              double falloff) {
    TerrainFlatten f;
    f.polygon = std::move(polygon);
    f.c = targetY;
    f.falloff = falloff;
    footprintBounds(f);
    return f;
}

TerrainFlatten makeFlattenRamp(const Vec3& a, const Vec3& b, double yA, double yB,
                               double halfWidth, double falloff) {
    TerrainFlatten f;
    f.falloff = falloff;
    double ex = b.x - a.x, ez = b.z - a.z;
    double len = std::sqrt(ex * ex + ez * ez);
    if (len < 1e-6) {
        // Degenerate segment: fall back to a constant square pad at yA.
        f.c = yA;
        double hw = std::max(halfWidth, 0.5);
        f.polygon = {Vec3(a.x - hw, 0, a.z - hw), Vec3(a.x + hw, 0, a.z - hw),
                     Vec3(a.x + hw, 0, a.z + hw), Vec3(a.x - hw, 0, a.z + hw)};
        footprintBounds(f);
        return f;
    }
    double ux = ex / len, uz = ez / len;        // along the segment
    double px = -uz, pz = ux;                    // across (left normal)
    // Height varies linearly along u: y = yA + k * (u . (p - a)). Expand into a
    // plane in world XZ so the terrain ramps exactly with the road.
    double k = (yB - yA) / len;
    f.dx = k * ux;
    f.dz = k * uz;
    f.c = yA - (f.dx * a.x + f.dz * a.z);
    f.polygon = {Vec3(a.x + px * halfWidth, 0, a.z + pz * halfWidth),
                 Vec3(b.x + px * halfWidth, 0, b.z + pz * halfWidth),
                 Vec3(b.x - px * halfWidth, 0, b.z - pz * halfWidth),
                 Vec3(a.x - px * halfWidth, 0, a.z - pz * halfWidth)};
    footprintBounds(f);
    return f;
}

Vec3 terrainColor(double height, double normalUp, double noiseValue) {
    // A layered natural palette: lush lowland grass dries to upland meadow and
    // bare earth, warm-grey rock and pale scree take the slopes and high
    // benches, snow caps the peaks — banded by DRYNESS (noise + altitude) and
    // SLOPE, with the noise term breaking every boundary so nothing reads as a
    // hard contour line (device: "a better terrain shader with more realistic
    // colors").
    const Vec3 grass (0.16, 0.32, 0.09);   // lush lowland green
    const Vec3 meadow(0.34, 0.40, 0.16);   // upland dry meadow
    const Vec3 dirt  (0.34, 0.25, 0.15);   // earth brown
    const Vec3 rock  (0.34, 0.31, 0.28);   // warm grey stone
    const Vec3 scree (0.56, 0.53, 0.49);   // pale talus / gravel
    const Vec3 snow  (0.92, 0.94, 0.98);

    const double n = noiseValue;                    // ~[-1, 1]
    const double slope = 1.0 - clamp01(normalUp);   // 0 flat .. 1 vertical
    // Altitude 0..1 across the playable relief, jittered so the treeline and
    // snowline meander instead of ringing the hill at one exact height.
    const double alt = clamp01((height + 4.0) / 60.0 + 0.06 * n);

    // Flat-ground colour by DRYNESS: green -> meadow -> earth. Noise dominates
    // near sea level (grass and dirt patches on the plain), altitude adds the
    // dry upland drift — so high-noise lowland still reads brown as before.
    const double dry = clamp01(0.5 + 0.5 * n + 0.4 * alt);
    Vec3 ground = dry < 0.5 ? mixv(grass, meadow, dry * 2.0)
                            : mixv(meadow, dirt, (dry - 0.5) * 2.0);

    // Slope exposes rock; the higher, steeper benches shed pale scree.
    double rockF = smoothstep(0.34, 0.60, slope + 0.10 * n);
    Vec3 c = mixv(ground, rock, rockF);
    double screeF = smoothstep(0.45, 0.75, alt) * smoothstep(0.18, 0.48, slope);
    c = mixv(c, scree, clamp01(screeF));

    // Snow on high, gentle ground; a jittered snowline, none on cliffs.
    double snowF = smoothstep(0.74, 0.96, alt + 0.05 * n) *
                   (1.0 - smoothstep(0.42, 0.66, slope));
    c = mixv(c, snow, snowF);

    return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z));
}

namespace {
// Ridged multifractal (Musgrave): each octave is ridged (1-|noise|, sharpened)
// and weighted by the previous octave, so detail concentrates on ridges and
// valleys stay smooth — varied, sharp, irregular peaks instead of uniform bumps.
// Returns ~[0,1].
double ridgedMultifractal(const Noise& n, double x, double y, int octaves) {
    double sum = 0.0, freq = 1.0, amp = 0.5, weight = 1.0, total = 0.0;
    for (int o = 0; o < octaves; o++) {
        double s = 1.0 - std::abs(n.noise2(x * freq, y * freq));   // ridge [0,1]
        s *= s;                                                    // sharpen
        s *= weight;                                               // feedback
        weight = clamp01(s * 2.0);                                 // gate next octave
        sum += s * amp;
        total += amp;
        freq *= 2.0;
        amp *= 0.5;
    }
    return total > 0.0 ? sum / total : 0.0;
}

// Nearest distance from (x,z) to the spine polyline + the arc fraction [0,1] of
// the closest point (for along-spine height variation). Arc is approximated by
// segment index (the loader samples the spine ~uniformly).
void spineQuery(const std::vector<Vec3>& spine, double x, double z,
                double& dist, double& arc) {
    dist = 1e30; arc = 0.0;
    const int n = static_cast<int>(spine.size());
    if (n == 1) {
        double dx = x - spine[0].x, dz = z - spine[0].z;
        dist = std::sqrt(dx * dx + dz * dz);
        return;
    }
    for (int i = 1; i < n; i++) {
        double ax = spine[i - 1].x, az = spine[i - 1].z;
        double ex = spine[i].x - ax, ez = spine[i].z - az;
        double seg2 = ex * ex + ez * ez;
        double t = seg2 > 1e-9 ? ((x - ax) * ex + (z - az) * ez) / seg2 : 0.0;
        t = clamp01(t);
        double dx = x - (ax + ex * t), dz = z - (az + ez * t);
        double d = std::sqrt(dx * dx + dz * dz);
        if (d < dist) { dist = d; arc = (i - 1 + t) / (n - 1); }
    }
}
}  // namespace

std::vector<Vec3> sampleRangeSpine(const std::vector<Vec3>& controls, int samples) {
    if (controls.size() < 2) return controls;
    Spline<Vec3> s = Spline<Vec3>::catmullRom(controls);
    std::vector<Vec3> out;
    const int n = std::max(2, samples);
    for (int i = 0; i < n; i++)
        out.push_back(s.eval(static_cast<double>(i) / (n - 1) * s.segments()));
    return out;
}

std::vector<RidgeSegment> buildRangeRidges(float length, float branchAngle,
                                           float falloff, float leaderFalloff,
                                           int iterations, float height,
                                           float depthFalloff, float angleJitter,
                                           uint32_t seed) {
    auto num = [](float v) { return std::to_string(v); };
    // Planar binary-branch grammar (only +/- yaw, so it stays in one plane): a
    // main leader throws off ± spurs that recurse. Reuses the parametric L-system.
    ParametricLSystem g;
    g.rule("A(l)", "F(l)[+(" + num(branchAngle) + ")A(l*" + num(falloff) + ")]" +
                       "[-(" + num(branchAngle) + ")A(l*" + num(falloff) + ")]" +
                       "A(l*" + num(leaderFalloff) + ")");
    ModuleString s = g.expand("A(" + num(length) + ")", iterations, seed);

    // Consumer #2 of the shared Skeleton: lay the turtle's (x,y) growth plane onto
    // the ground (x,z); crest height falls by branch depth.
    Skeleton skel = buildSkeleton(s, angleJitter, seed);
    auto heightAt = [&](int depth) {
        return height * std::pow(depthFalloff, static_cast<float>(depth));
    };
    std::vector<RidgeSegment> ridges;
    for (size_t i = 1; i < skel.nodes.size(); i++) {
        const SkeletonNode& n = skel.nodes[i];
        if (n.parent < 0) continue;
        const SkeletonNode& p = skel.nodes[n.parent];
        RidgeSegment seg;
        seg.a = Vec3(p.pos.x, 0.0, p.pos.y);   // y (turtle up) -> ground z
        seg.b = Vec3(n.pos.x, 0.0, n.pos.y);
        seg.ha = heightAt(p.depth);
        seg.hb = heightAt(n.depth);
        ridges.push_back(seg);
    }
    return ridges;
}

double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ) {
    return terrainHeight(params, noise, worldX, worldZ, 0.0);
}

double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ, double flattenDilate) {
    double nx = worldX * params.noiseScale;
    double nz = worldZ * params.noiseScale;
    double h = params.warp > 0.0
                   ? noise.warpedFbm2(nx, nz, params.warp, params.octaves)
                   : noise.fbm2(nx, nz, params.octaves);
    h *= params.heightScale;

    // Mountain layer: a regional mask decides where it rises (range vs plains),
    // then a domain-warped ridged multifractal gives irregular varied peaks.
    if (params.mountainHeight > 0.0f) {
        double mask = 1.0;
        if (params.mountainMaskScale > 0.0) {
            double m = noise.fbm2(worldX * params.mountainMaskScale,
                                  worldZ * params.mountainMaskScale, 3);
            mask = smoothstep(params.mountainMaskLo, params.mountainMaskHi, m);
        }
        if (mask > 1e-3) {
            double wx = worldX * params.mountainScale;
            double wz = worldZ * params.mountainScale;
            // Domain warp the ridges so they meander instead of looking regular.
            double ox = noise.noise2(wx + 5.2, wz + 1.3) * 0.6;
            double oz = noise.noise2(wx + 9.1, wz + 4.7) * 0.6;
            double mh = ridgedMultifractal(noise, wx + ox, wz + oz, 6);
            h += mh * params.mountainHeight * mask;
        }
    }

    // Spine-driven range: uplift falls off from the range axis (range -> foothills
    // -> plains) and varies along it (tall massifs, low passes), shaping a
    // ridged-multifractal relief.
    if (!params.rangeSpine.empty() && params.rangeHeight > 0.0f) {
        double dist, arc;
        spineQuery(params.rangeSpine, worldX, worldZ, dist, arc);
        double cross = 1.0 - smoothstep(0.0, params.rangeWidth, dist);
        if (cross > 1e-3) {
            double a = noise.noise2(arc * 7.0 + 0.5, 13.7);            // [-1,1]
            double along = 1.0 - params.rangeVariation * (0.5 - 0.5 * a);
            double wx = worldX * params.mountainScale;
            double wz = worldZ * params.mountainScale;
            double ox = noise.noise2(wx + 5.2, wz + 1.3) * 0.6;
            double oz = noise.noise2(wx + 9.1, wz + 4.7) * 0.6;
            double relief = ridgedMultifractal(noise, wx + ox, wz + oz, 6);
            h += relief * params.rangeHeight * cross * along;
        }
    }

    // Branching ridge network: uplift from the nearest ridge segment (its
    // interpolated crest height), falling off with distance, shaped by the
    // ridged multifractal. A main divide + spurs + sub-spurs.
    if (!params.rangeRidges.empty()) {
        double bestD = 1e30, crest = 0.0;
        for (const RidgeSegment& seg : params.rangeRidges) {
            double ex = seg.b.x - seg.a.x, ez = seg.b.z - seg.a.z;
            double seg2 = ex * ex + ez * ez;
            double t = seg2 > 1e-9
                           ? ((worldX - seg.a.x) * ex + (worldZ - seg.a.z) * ez) / seg2
                           : 0.0;
            t = clamp01(t);
            double dx = worldX - (seg.a.x + ex * t), dz = worldZ - (seg.a.z + ez * t);
            double d = std::sqrt(dx * dx + dz * dz);
            if (d < bestD) { bestD = d; crest = seg.ha + (seg.hb - seg.ha) * t; }
        }
        double cross = 1.0 - smoothstep(0.0, params.rangeWidth, bestD);
        if (cross > 1e-3) {
            double wx = worldX * params.mountainScale, wz = worldZ * params.mountainScale;
            double ox = noise.noise2(wx + 5.2, wz + 1.3) * 0.6;
            double oz = noise.noise2(wx + 9.1, wz + 4.7) * 0.6;
            double relief = ridgedMultifractal(noise, wx + ox, wz + oz, 6);
            h += relief * crest * cross;
        }
    }

    // City cut/fill: grade the ground flat under roads and blocks (applied last so
    // it overrides every relief layer there). Use the spatial index when the
    // recipe carries one (ADR-0075 Phase 0) — same height, far fewer footprints
    // tested per sample.
    if (!params.flatten.empty()) {
        if (params.flattenIndex && !params.flattenIndex->empty())
            h = applyFlatten(*params.flattenIndex, params.flatten, worldX, worldZ, h,
                             flattenDilate);
        else
            h = applyFlatten(params.flatten, worldX, worldZ, h, flattenDilate);
    }
    return h;
}

RenderMesh generateTerrain(const TerrainParams& params, const Noise& noise) {
    const int res = std::max(1, params.resolution);
    const int n = res + 1;                       // vertices per side
    const float half = params.size * 0.5f;
    const float step = params.size / static_cast<float>(res);

    RenderMesh mesh;
    mesh.vertices.reserve(static_cast<size_t>(n) * n);
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            double x = -half + i * step;
            double z = -half + j * step;
            double y = terrainHeight(params, noise, x, z);
            // Normal is recomputed below; UV is set by generatePlanarUVs.
            mesh.vertices.push_back(Vertex(Vec3(x, y, z), Vec3(0, 1, 0)));
        }
    }

    // Clockwise-front winding so the top faces up; recomputeNormals then derives
    // the up-facing smooth normals from it.
    MeshBuilder::gridIndices(mesh, n, n);

    MeshBuilder::recomputeNormals(mesh);
    // UVs span [0,1] across the patch (offset by half, scaled by 1/size).
    MeshBuilder::generatePlanarUVs(mesh, /*axis=*/1, /*scale=*/1.0f / params.size);

    // Bake height/slope coloration into per-vertex colors (a low-frequency
    // noise term varies it). The shader multiplies these with the material.
    for (Vertex& v : mesh.vertices) {
        double nv = noise.noise2(v.position.x * 0.15, v.position.z * 0.15);
        v.color = terrainColor(v.position.y, v.normal.y, nv);
    }
    return mesh;
}

RenderMesh generateTerrainRing(const TerrainParams& params, const Noise& noise,
                               float innerHalf, float outerHalf, int cells) {
    RenderMesh mesh;
    cells = std::max(2, cells);
    const int n = cells + 1;
    const float step = (outerHalf * 2.0f) / cells;

    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            float x = -outerHalf + i * step;
            float z = -outerHalf + j * step;
            float y = static_cast<float>(terrainHeight(params, noise, x, z));
            mesh.vertices.push_back(Vertex(Vec3(x, y, z), Vec3(0, 1, 0)));
        }
    }
    for (int j = 0; j < cells; j++) {
        for (int i = 0; i < cells; i++) {
            float x0 = -outerHalf + i * step, x1 = x0 + step;
            float z0 = -outerHalf + j * step, z1 = z0 + step;
            // Skip quads entirely inside the inner hole (left for the finer tile).
            if (std::max(std::abs(x0), std::abs(x1)) <= innerHalf &&
                std::max(std::abs(z0), std::abs(z1)) <= innerHalf)
                continue;
            uint32_t a = static_cast<uint32_t>(j * n + i);
            uint32_t b = a + 1;
            uint32_t c = a + static_cast<uint32_t>(n);
            uint32_t d = c + 1;
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }

    MeshBuilder::recomputeNormals(mesh);
    MeshBuilder::generatePlanarUVs(mesh, /*axis=*/1, /*scale=*/1.0f / (outerHalf * 2.0f));
    for (Vertex& v : mesh.vertices) {
        double nv = noise.noise2(v.position.x * 0.15, v.position.z * 0.15);
        v.color = terrainColor(v.position.y, v.normal.y, nv);
    }
    return mesh;
}

std::vector<RenderMesh> generateTerrainLOD(const TerrainParams& params,
                                           const Noise& noise, int levels, int cells) {
    std::vector<RenderMesh> rings;
    float inner = params.size * 0.5f;          // central tile edge
    for (int l = 0; l < levels; l++) {
        float outer = inner * 2.0f;            // each ring doubles the extent
        rings.push_back(generateTerrainRing(params, noise, inner, outer, cells));
        inner = outer;
    }
    return rings;
}

Vec3 terrainNormal(const TerrainParams& params, const Noise& noise,
                   double worldX, double worldZ, double eps) {
    // Central differences of the height field: n = normalize(-dH/dx, 1, -dH/dz).
    // A fixed eps (independent of which chunk samples it) makes the normal at a
    // shared border position identical for both chunks, so borders are seamless.
    double hL = terrainHeight(params, noise, worldX - eps, worldZ);
    double hR = terrainHeight(params, noise, worldX + eps, worldZ);
    double hD = terrainHeight(params, noise, worldX, worldZ - eps);
    double hU = terrainHeight(params, noise, worldX, worldZ + eps);
    return normalize(Vec3((hL - hR), 2.0 * eps, (hD - hU)));
}

std::vector<TerrainChunk> generateTerrainChunks(const TerrainParams& params,
                                                const Noise& noise,
                                                int chunksPerSide, float chunkSize,
                                                int resolution, float colliderRadius) {
    chunksPerSide = std::max(1, chunksPerSide);
    const int res = std::max(1, resolution);
    const int n = res + 1;                         // vertices per side
    const float step = chunkSize / static_cast<float>(res);
    const double eps = step;                        // shared across chunks -> seamless
    const float worldHalf = chunksPerSide * chunkSize * 0.5f;  // world centred on origin

    std::vector<TerrainChunk> chunks;
    chunks.reserve(static_cast<size_t>(chunksPerSide) * chunksPerSide);

    for (int cz = 0; cz < chunksPerSide; cz++) {
        for (int cx = 0; cx < chunksPerSide; cx++) {
            const double originX = -worldHalf + cx * chunkSize;
            const double originZ = -worldHalf + cz * chunkSize;

            TerrainChunk chunk;
            chunk.cx = cx;
            chunk.cz = cz;
            RenderMesh& mesh = chunk.mesh;
            mesh.vertices.reserve(static_cast<size_t>(n) * n);

            float minY = std::numeric_limits<float>::max();
            float maxY = std::numeric_limits<float>::lowest();
            for (int j = 0; j < n; j++) {
                for (int i = 0; i < n; i++) {
                    double x = originX + i * step;
                    double z = originZ + j * step;
                    double y = terrainHeight(params, noise, x, z);
                    minY = std::min(minY, static_cast<float>(y));
                    maxY = std::max(maxY, static_cast<float>(y));
                    Vertex v(Vec3(x, y, z),
                             terrainNormal(params, noise, x, z, eps));
                    // World-continuous UVs (tile across the whole world).
                    v.u = static_cast<float>(x / chunkSize);
                    v.v = static_cast<float>(z / chunkSize);
                    double nv = noise.noise2(x * 0.15, z * 0.15);
                    v.color = terrainColor(y, v.normal.y, nv);
                    mesh.vertices.push_back(v);
                }
            }

            // Same clockwise-front winding as generateTerrain (top up).
            MeshBuilder::gridIndices(mesh, n, n);

            chunk.boundsMin = Vec3(originX, minY, originZ);
            chunk.boundsMax = Vec3(originX + chunkSize, maxY, originZ + chunkSize);

            double dx = originX + chunkSize * 0.5;   // chunk centre
            double dz = originZ + chunkSize * 0.5;
            chunk.collider = std::sqrt(dx * dx + dz * dz) <= colliderRadius;

            chunks.push_back(std::move(chunk));
        }
    }
    return chunks;
}

}  // namespace engine
