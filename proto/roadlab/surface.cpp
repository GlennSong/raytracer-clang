#include "surface.h"

#include "paint_bake.h"

#include "junction.h"
#include <cstring>

namespace roadlab {

Vec3 paintAlbedo(PaintColor c) {
    switch (c) {
        case PaintColor::White: return {0.80, 0.80, 0.77};
        case PaintColor::Yellow: return {0.76, 0.60, 0.09};
        case PaintColor::Red: return {0.52, 0.10, 0.09};
        case PaintColor::Blue: return {0.09, 0.20, 0.52};
        case PaintColor::Green: return {0.07, 0.40, 0.21};
    }
    return {0.8, 0.8, 0.8};
}

namespace {

// --- 2D signed distance primitives ---------------------------------------
// Everything painted on a road is one of these, evaluated in metres. Because
// they are distances, one `smoothstep(-fw, fw, d)` antialiases any of them, and
// widening `fw` with distance is the only LOD the paint needs.

double sdSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 pa = p - a, ba = b - a;
    double h = clampd(dot(pa, ba) / std::max(1e-9, dot(ba, ba)), 0.0, 1.0);
    return length(pa - ba * h);
}

double sdBox(Vec2 p, Vec2 half) {
    Vec2 d{std::fabs(p.x) - half.x, std::fabs(p.y) - half.y};
    Vec2 m{std::max(d.x, 0.0), std::max(d.y, 0.0)};
    return length(m) + std::min(std::max(d.x, d.y), 0.0);
}

double sdTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    Vec2 e0 = b - a, e1 = c - b, e2 = a - c;
    Vec2 v0 = p - a, v1 = p - b, v2 = p - c;
    Vec2 pq0 = v0 - e0 * clampd(dot(v0, e0) / dot(e0, e0), 0.0, 1.0);
    Vec2 pq1 = v1 - e1 * clampd(dot(v1, e1) / dot(e1, e1), 0.0, 1.0);
    Vec2 pq2 = v2 - e2 * clampd(dot(v2, e2) / dot(e2, e2), 0.0, 1.0);
    double sgn = (e0.x * e2.y - e0.y * e2.x) >= 0 ? 1.0 : -1.0;
    // The distance and the inside/outside test are minimised independently —
    // taking them from the same edge is the classic way to get this wrong.
    double dist = std::min({lengthSq(pq0), lengthSq(pq1), lengthSq(pq2)});
    double side = std::min({sgn * cross(v0, e0), sgn * cross(v1, e1), sgn * cross(v2, e2)});
    return -std::sqrt(dist) * (side >= 0 ? 1.0 : -1.0);
}

// Coverage from a signed distance, with the anti-shimmer rule: once the feature
// is thinner than the filter, keep its total energy instead of letting it flash
// in and out between fragments.
double coverage(double sd, double fw) {
    double a = std::max(fw, 1e-5);
    return 1.0 - smoothstepd(-a, a, sd);
}

// --- stroke font ----------------------------------------------------------
// Road text ("ONLY", "STOP", "BUS", "35") is a handful of line segments in a
// unit box, drawn with the same SDF machinery as the arrows. An MSDF atlas would
// be the production choice; strokes keep the prototype dependency-free and prove
// the point that text does not need a mesh either.

struct Stroke {
    double x0, y0, x1, y1;
};

#define S(a, b, c, d) \
    { a, b, c, d }
const Stroke kSeg0[] = {S(0, 0, 0, 1), S(0, 1, 1, 1), S(1, 1, 1, 0), S(1, 0, 0, 0)};
const Stroke kSeg1[] = {S(0.5, 0, 0.5, 1)};
const Stroke kSeg2[] = {S(0, 1, 1, 1), S(1, 1, 1, 0.5), S(1, 0.5, 0, 0.5), S(0, 0.5, 0, 0),
                        S(0, 0, 1, 0)};
const Stroke kSeg3[] = {S(0, 1, 1, 1), S(1, 1, 1, 0), S(1, 0, 0, 0), S(0, 0.5, 1, 0.5)};
const Stroke kSeg4[] = {S(0, 1, 0, 0.5), S(0, 0.5, 1, 0.5), S(1, 1, 1, 0)};
const Stroke kSeg5[] = {S(1, 1, 0, 1), S(0, 1, 0, 0.5), S(0, 0.5, 1, 0.5), S(1, 0.5, 1, 0),
                        S(1, 0, 0, 0)};
const Stroke kSeg6[] = {S(1, 1, 0, 1), S(0, 1, 0, 0), S(0, 0, 1, 0), S(1, 0, 1, 0.5),
                        S(1, 0.5, 0, 0.5)};
const Stroke kSeg7[] = {S(0, 1, 1, 1), S(1, 1, 1, 0)};
const Stroke kSeg8[] = {S(0, 0, 0, 1), S(0, 1, 1, 1), S(1, 1, 1, 0),
                        S(1, 0, 0, 0), S(0, 0.5, 1, 0.5)};
const Stroke kSeg9[] = {S(1, 0, 1, 1), S(1, 1, 0, 1), S(0, 1, 0, 0.5), S(0, 0.5, 1, 0.5)};
const Stroke kA[] = {S(0, 0, 0.5, 1), S(0.5, 1, 1, 0), S(0.22, 0.42, 0.78, 0.42)};
const Stroke kB[] = {S(0, 0, 0, 1),   S(0, 1, 0.9, 1),     S(0.9, 1, 0.9, 0.55),
                     S(0.9, 0.55, 0, 0.5), S(0, 0.5, 1, 0.45), S(1, 0.45, 1, 0),
                     S(1, 0, 0, 0)};
const Stroke kC[] = {S(1, 1, 0, 1), S(0, 1, 0, 0), S(0, 0, 1, 0)};
const Stroke kD[] = {S(0, 0, 0, 1), S(0, 1, 0.75, 0.9), S(0.75, 0.9, 1, 0.5),
                     S(1, 0.5, 0.75, 0.1), S(0.75, 0.1, 0, 0)};
const Stroke kE[] = {S(1, 1, 0, 1), S(0, 1, 0, 0), S(0, 0, 1, 0), S(0, 0.5, 0.8, 0.5)};
const Stroke kG[] = {S(1, 1, 0, 1), S(0, 1, 0, 0), S(0, 0, 1, 0), S(1, 0, 1, 0.5),
                     S(1, 0.5, 0.5, 0.5)};
const Stroke kH[] = {S(0, 0, 0, 1), S(1, 0, 1, 1), S(0, 0.5, 1, 0.5)};
const Stroke kI[] = {S(0.5, 0, 0.5, 1), S(0.15, 1, 0.85, 1), S(0.15, 0, 0.85, 0)};
const Stroke kK[] = {S(0, 0, 0, 1), S(1, 1, 0, 0.5), S(0, 0.5, 1, 0)};
const Stroke kL[] = {S(0, 1, 0, 0), S(0, 0, 1, 0)};
const Stroke kM[] = {S(0, 0, 0, 1), S(0, 1, 0.5, 0.35), S(0.5, 0.35, 1, 1), S(1, 1, 1, 0)};
const Stroke kN[] = {S(0, 0, 0, 1), S(0, 1, 1, 0), S(1, 0, 1, 1)};
const Stroke kO[] = {S(0, 0, 0, 1), S(0, 1, 1, 1), S(1, 1, 1, 0), S(1, 0, 0, 0)};
const Stroke kP[] = {S(0, 0, 0, 1), S(0, 1, 1, 1), S(1, 1, 1, 0.5), S(1, 0.5, 0, 0.5)};
const Stroke kR[] = {S(0, 0, 0, 1), S(0, 1, 1, 1), S(1, 1, 1, 0.5), S(1, 0.5, 0, 0.5),
                     S(0, 0.5, 1, 0)};
const Stroke kS[] = {S(1, 1, 0, 1), S(0, 1, 0, 0.5), S(0, 0.5, 1, 0.5), S(1, 0.5, 1, 0),
                     S(1, 0, 0, 0)};
const Stroke kT[] = {S(0, 1, 1, 1), S(0.5, 1, 0.5, 0)};
const Stroke kU[] = {S(0, 1, 0, 0), S(0, 0, 1, 0), S(1, 0, 1, 1)};
const Stroke kV[] = {S(0, 1, 0.5, 0), S(0.5, 0, 1, 1)};
const Stroke kW[] = {S(0, 1, 0.25, 0), S(0.25, 0, 0.5, 0.6), S(0.5, 0.6, 0.75, 0),
                     S(0.75, 0, 1, 1)};
const Stroke kX[] = {S(0, 0, 1, 1), S(0, 1, 1, 0)};
const Stroke kY[] = {S(0, 1, 0.5, 0.5), S(1, 1, 0.5, 0.5), S(0.5, 0.5, 0.5, 0)};
#undef S

struct GlyphDef {
    const Stroke* strokes;
    int count;
};

GlyphDef glyphFor(char ch) {
#define G(arr) GlyphDef{arr, int(sizeof(arr) / sizeof(Stroke))}
    switch (ch) {
        case '0': return G(kSeg0);
        case '1': return G(kSeg1);
        case '2': return G(kSeg2);
        case '3': return G(kSeg3);
        case '4': return G(kSeg4);
        case '5': return G(kSeg5);
        case '6': return G(kSeg6);
        case '7': return G(kSeg7);
        case '8': return G(kSeg8);
        case '9': return G(kSeg9);
        case 'A': return G(kA);
        case 'B': return G(kB);
        case 'C': return G(kC);
        case 'D': return G(kD);
        case 'E': return G(kE);
        case 'G': return G(kG);
        case 'H': return G(kH);
        case 'I': return G(kI);
        case 'K': return G(kK);
        case 'L': return G(kL);
        case 'M': return G(kM);
        case 'N': return G(kN);
        case 'O': return G(kO);
        case 'P': return G(kP);
        case 'R': return G(kR);
        case 'S': return G(kS);
        case 'T': return G(kT);
        case 'U': return G(kU);
        case 'V': return G(kV);
        case 'W': return G(kW);
        case 'X': return G(kX);
        case 'Y': return G(kY);
        default: return GlyphDef{nullptr, 0};
    }
#undef G
}

// --- glyph SDFs -----------------------------------------------------------
// Local frame: `u` is across the road (t), `v` is along it (s), with +v being
// the direction the marking faces (the way a driver reading it is going).

double sdArrowHead(Vec2 p, Vec2 tip, Vec2 dir, double len, double halfWidth) {
    Vec2 back = tip - dir * len;
    Vec2 side{-dir.y, dir.x};
    return sdTriangle(p, tip, back + side * halfWidth, back - side * halfWidth);
}

double sdStraightArrow(Vec2 p, double along, double across) {
    double halfL = along * 0.5;
    double headLen = std::min(along * 0.42, across * 1.1);
    double shaftW = std::max(0.12, across * 0.22);
    double d = sdSegment(p, {0, -halfL}, {0, halfL - headLen}) - shaftW * 0.5;
    d = std::min(d, sdArrowHead(p, {0, halfL}, {0, 1}, headLen, across * 0.5));
    return d;
}

double sdTurnArrow(Vec2 p, double along, double across, double sign, bool withThrough) {
    double halfL = along * 0.5;
    double shaftW = std::max(0.12, across * 0.20);
    double bendV = -halfL + along * 0.45;
    double tipU = sign * across * 0.42;
    double headLen = std::min(along * 0.24, across * 0.6);
    // Stem, then a two-segment bend, then a head pointing sideways-forward.
    double d = sdSegment(p, {0, -halfL}, {0, bendV}) - shaftW * 0.5;
    Vec2 mid{sign * across * 0.16, bendV + along * 0.14};
    Vec2 tip{tipU, bendV + along * 0.26};
    d = std::min(d, sdSegment(p, {0, bendV}, mid) - shaftW * 0.5);
    d = std::min(d, sdSegment(p, mid, tip) - shaftW * 0.5);
    Vec2 dir = normalize(tip - mid);
    d = std::min(d, sdArrowHead(p, tip + dir * headLen, dir, headLen, across * 0.24));
    if (withThrough) {
        double t = sdSegment(p, {0, bendV}, {0, halfL - along * 0.22}) - shaftW * 0.5;
        t = std::min(t, sdArrowHead(p, {0, halfL}, {0, 1}, along * 0.22, across * 0.26));
        d = std::min(d, t);
    }
    return d;
}

double sdMergeArrow(Vec2 p, double along, double across, double sign) {
    // The lane-drop arrow: a long shallow sweep, not a turn arrow.
    double halfL = along * 0.5;
    double shaftW = std::max(0.14, across * 0.20);
    Vec2 a{sign * across * 0.34, -halfL};
    Vec2 b{sign * across * 0.30, -halfL + along * 0.30};
    Vec2 c{0, halfL - along * 0.30};
    double d = sdSegment(p, a, b) - shaftW * 0.5;
    d = std::min(d, sdSegment(p, b, c) - shaftW * 0.5);
    d = std::min(d, sdArrowHead(p, {0, halfL}, {0, 1}, along * 0.30, across * 0.30));
    return d;
}

double sdTextRow(Vec2 p, const char* text, double along, double across, double strokeW) {
    size_t n = std::strlen(text);
    if (n == 0) return 1e9;
    // Characters stack along v so the word reads bottom-up as you drive over it,
    // and each is stretched about 2.5:1 the way real road text is, so it reads
    // correctly from a low viewing angle.
    double charV = along / double(n);
    double charU = std::min(across * 0.8, charV * 0.42);
    double d = 1e9;
    for (size_t i = 0; i < n; ++i) {
        GlyphDef g = glyphFor(text[i]);
        if (!g.strokes) continue;
        double v0 = -along * 0.5 + double(i) * charV + charV * 0.12;
        double vh = charV * 0.76;
        for (int k = 0; k < g.count; ++k) {
            const Stroke& st = g.strokes[k];
            Vec2 a{(double(st.x0) - 0.5) * charU, v0 + double(st.y0) * vh};
            Vec2 b{(double(st.x1) - 0.5) * charU, v0 + double(st.y1) * vh};
            d = std::min(d, sdSegment(p, a, b) - strokeW * 0.5);
        }
    }
    return d;
}

double glyphSdf(const Decal& d, Vec2 p) {
    switch (d.kind) {
        case GlyphKind::ArrowThrough:
            return sdStraightArrow(p, d.along, d.across);
        case GlyphKind::ArrowLeft:
            return sdTurnArrow(p, d.along, d.across, +1.0, false);
        case GlyphKind::ArrowRight:
            return sdTurnArrow(p, d.along, d.across, -1.0, false);
        case GlyphKind::ArrowThroughLeft:
            return sdTurnArrow(p, d.along, d.across, +1.0, true);
        case GlyphKind::ArrowThroughRight:
            return sdTurnArrow(p, d.along, d.across, -1.0, true);
        case GlyphKind::ArrowMergeLeft:
            // The tail sits on the side the driver is leaving, so an arrow that
            // moves them LEFT starts on their right (-u).
            return sdMergeArrow(p, d.along, d.across, -1.0);
        case GlyphKind::ArrowMergeRight:
            return sdMergeArrow(p, d.along, d.across, +1.0);
        case GlyphKind::StopBar:
            return sdBox(p, {d.across * 0.5, d.along * 0.5});
        case GlyphKind::YieldTeeth: {
            // A row of triangles pointing at the traffic that has to give way.
            double pitch = 0.9;
            double u = std::fmod(std::fmod(p.x, pitch) + pitch, pitch) - pitch * 0.5;
            if (std::fabs(p.x) > d.across * 0.5) return 1e9;
            return sdTriangle({u, p.y}, {0, d.along * 0.5}, {-0.30, -d.along * 0.5},
                              {0.30, -d.along * 0.5});
        }
        case GlyphKind::CrosswalkZebra: {
            // Bars run ALONG the traffic direction and repeat across it, which is
            // the continental crossing everyone actually paints.
            double pitch = 1.20, bar = 0.60;
            if (std::fabs(p.x) > d.across * 0.5 || std::fabs(p.y) > d.along * 0.5) return 1e9;
            double u = std::fmod(std::fmod(p.x, pitch) + pitch, pitch) - pitch * 0.5;
            return std::fabs(u) - bar * 0.5;
        }
        case GlyphKind::CrosswalkLadder: {
            if (std::fabs(p.x) > d.across * 0.5 || std::fabs(p.y) > d.along * 0.5) return 1e9;
            double rails = std::fabs(std::fabs(p.y) - d.along * 0.5) - 0.10;
            double pitch = 1.20, bar = 0.55;
            double u = std::fmod(std::fmod(p.x, pitch) + pitch, pitch) - pitch * 0.5;
            return std::min(rails, std::fabs(u) - bar * 0.5);
        }
        case GlyphKind::ChevronGore: {
            // Repeating Vs whose apex faces the oncoming traffic.
            double pitch = 3.0;
            double v = std::fmod(std::fmod(p.y, pitch) + pitch, pitch) - pitch * 0.5;
            if (std::fabs(p.x) > d.across * 0.5 || std::fabs(p.y) > d.along * 0.5) return 1e9;
            double half = d.across * 0.5;
            double a = sdSegment({p.x, v}, {-half, 0.9}, {0, -0.7}) - 0.11;
            double b = sdSegment({p.x, v}, {half, 0.9}, {0, -0.7}) - 0.11;
            return std::min(a, b);
        }
        case GlyphKind::ParkingTee: {
            double stem = sdSegment(p, {0, -d.along * 0.5}, {0, 0}) - 0.06;
            double head = sdSegment(p, {-d.across * 0.5, 0}, {d.across * 0.5, 0}) - 0.06;
            return std::min(stem, head);
        }
        case GlyphKind::BikeSymbol: {
            double r = d.across * 0.22;
            double w1 = std::fabs(length(p - Vec2{0, -d.along * 0.26}) - r) - 0.07;
            double w2 = std::fabs(length(p - Vec2{0, d.along * 0.26}) - r) - 0.07;
            double frame = sdSegment(p, {0, -d.along * 0.26}, {0, d.along * 0.20}) - 0.07;
            double bars = sdSegment(p, {-d.across * 0.28, d.along * 0.26},
                                    {d.across * 0.28, d.along * 0.26}) -
                          0.06;
            return std::min({w1, w2, frame, bars});
        }
        case GlyphKind::HovDiamond: {
            double a = std::fabs(p.x) / (d.across * 0.5) + std::fabs(p.y) / (d.along * 0.5);
            double outline = std::fabs(a - 1.0) * std::min(d.across, d.along) * 0.5 - 0.10;
            return outline;
        }
        case GlyphKind::KeepClearGrid: {
            if (std::fabs(p.x) > d.across * 0.5 || std::fabs(p.y) > d.along * 0.5) return 1e9;
            double pitch = 1.6;
            double a = std::fmod(std::fmod(p.x + p.y, pitch) + pitch, pitch) - pitch * 0.5;
            double b = std::fmod(std::fmod(p.x - p.y, pitch) + pitch, pitch) - pitch * 0.5;
            double border = -sdBox(p, {d.across * 0.5 - 0.10, d.along * 0.5 - 0.10});
            return std::min({std::fabs(a) - 0.07, std::fabs(b) - 0.07, border});
        }
        case GlyphKind::Text:
            return sdTextRow(p, d.text, d.along, d.across, std::max(0.12, d.across * 0.13));
    }
    return 1e9;
}

// --- materials ------------------------------------------------------------

struct Material {
    Vec3 albedo;
    double roughness;
    double metallic;
};

Material materialFor(SurfaceKind surf, StripKind kind, double s, double t, double grime,
                     uint32_t seed) {
    (void)kind;
    double n = fbm(s * 0.35 + double(seed), t * 0.35, 4);
    switch (surf) {
        case SurfaceKind::Asphalt: {
            // Aggregate: cellular for the stones, fbm for the binder. Patches are
            // large worley cells with their own age, which is what stops a long
            // road from reading as one flat sheet.
            double agg = worley(s * 3.1, t * 3.1);
            double patch = worley(s * 0.035 + 11.0, t * 0.035 - 7.0);
            double patchAge = 0.5 + 0.5 * std::sin(patch * 37.0 + double(seed));
            double v = 0.052 + 0.030 * agg + 0.020 * n + 0.020 * patchAge * grime;
            // Cracks: thin dark filaments along the fbm ridges.
            double crack = std::fabs(fbm(s * 0.9, t * 0.9, 3) - 0.5);
            double crackMask = 1.0 - smoothstepd(0.0, 0.02 + 0.03 * (1.0 - grime), crack);
            v *= (1.0 - 0.45 * crackMask * grime);
            return {{v, v * 1.01, v * 1.06}, 0.90 - 0.10 * agg, 0.0};
        }
        case SurfaceKind::Concrete: {
            double v = 0.30 + 0.10 * n;
            // Transverse expansion joints every 4.5 m, longitudinal every lane.
            double j1 = std::fabs(std::fmod(std::fmod(s, 4.5) + 4.5, 4.5) - 2.25);
            double v2 = v * (1.0 - 0.35 * (1.0 - smoothstepd(0.0, 0.05, j1 - 0.02)));
            return {{v2, v2, v2 * 0.98}, 0.80, 0.0};
        }
        case SurfaceKind::Gravel: {
            double g = worley(s * 6.0, t * 6.0);
            double v = 0.20 + 0.16 * g + 0.05 * n;
            return {{v, v * 0.97, v * 0.90}, 0.96, 0.0};
        }
        case SurfaceKind::Dirt: {
            double v = 0.16 + 0.09 * n;
            return {{v * 1.25, v * 0.95, v * 0.68}, 0.97, 0.0};
        }
        case SurfaceKind::Grass: {
            double g = fbm(s * 1.4, t * 1.4, 4);
            return {{0.045 + 0.05 * g, 0.10 + 0.09 * g, 0.030 + 0.03 * g}, 0.95, 0.0};
        }
        case SurfaceKind::Brick: {
            double row = std::floor(s / 0.24);
            double off = std::fmod(row, 2.0) * 0.12;
            double bx = std::fabs(std::fmod(std::fmod(t + off, 0.24) + 0.24, 0.24) - 0.12);
            double by = std::fabs(std::fmod(std::fmod(s, 0.12) + 0.12, 0.12) - 0.06);
            double mortar = std::min(smoothstepd(0.0, 0.02, bx - 0.005),
                                     smoothstepd(0.0, 0.02, by - 0.005));
            Vec3 c = lerp(Vec3{0.30, 0.29, 0.27}, Vec3{0.26, 0.11, 0.08}, mortar);
            c = c * (0.85 + 0.3 * n);
            return {c, 0.88, 0.0};
        }
        case SurfaceKind::Steel:
            return {{0.42, 0.43, 0.45}, 0.35, 1.0};
    }
    return {{0.2, 0.2, 0.2}, 0.9, 0.0};
}

// Distance from the nearest wheel path within a lane. Wheel paths sit about
// 0.9 m either side of the lane centre and are where asphalt gets polished,
// darkened, rutted and where paint wears out first.
double wheelPathFactor(double tInLane, double laneWidth) {
    if (laneWidth < 1.0) return 0.0;
    double off = std::min(0.9, laneWidth * 0.28);
    double d = std::min(std::fabs(tInLane - off), std::fabs(tInLane + off));
    return std::exp(-(d * d) / (2.0 * 0.34 * 0.34));
}

}  // namespace

// --- micro displacement ---------------------------------------------------

double surfaceMicroHeight(const Road& road, double s, double t, const ShadeParams& p) {
    if (!p.micro) return 0.0;
    int laneId = 0;
    const Strip* st = road.xs.stripAtT(s, t, &laneId);
    if (!st) return 0.0;
    double h = 0.0;
    switch (st->surface) {
        case SurfaceKind::Asphalt:
            h += 0.0035 * worley(s * 3.1, t * 3.1);
            break;
        case SurfaceKind::Gravel:
            h += 0.012 * worley(s * 6.0, t * 6.0);
            break;
        case SurfaceKind::Concrete: {
            double j = std::fabs(std::fmod(std::fmod(s, 4.5) + 4.5, 4.5) - 2.25);
            h -= 0.010 * (1.0 - smoothstepd(0.0, 0.04, j - 0.015));
            break;
        }
        case SurfaceKind::Grass:
            h += 0.02 * fbm(s * 1.4, t * 1.4, 3);
            break;
        default:
            break;
    }
    // Rumble strips: milled grooves just inside a shoulder's carriageway edge.
    // They are a height field, not a texture, which is why they belong here.
    if (st->kind == StripKind::Shoulder) {
        double tInner = 0, tOuter = 0;
        int sec = road.xs.sectionIndexAt(s);
        if (road.xs.laneSpanAt(sec, laneId, s, tInner, tOuter)) {
            double dEdge = std::fabs(t - tInner);
            if (dEdge < 0.45) {
                double g = std::fmod(std::fmod(s, 0.30) + 0.30, 0.30) - 0.15;
                h -= 0.013 * (1.0 - smoothstepd(0.0, 0.03, std::fabs(g) - 0.055));
            }
        }
    }
    // Ruts: the wheel paths of a well-used lane dish out by a few millimetres.
    if (st->isLane()) {
        int sec = road.xs.sectionIndexAt(s);
        double a = 0, b = 0;
        if (road.xs.laneSpanAt(sec, laneId, s, a, b)) {
            double centre = 0.5 * (a + b);
            h -= 0.006 * p.grime * wheelPathFactor(t - centre, std::fabs(b - a));
        }
    }
    return h;
}

// --- the fragment ---------------------------------------------------------

SurfaceSample shadeRoadSurface(const Road& road, const RoadPaint* paint, double s, double t,
                               double filterWidth, const ShadeParams& params) {
    SurfaceSample out;
    const double fw = std::max(filterWidth, 1e-4);

    int laneId = 0;
    const Strip* strip = road.xs.stripAtT(s, t, &laneId);
    if (!strip) {
        out.albedo = {0.05, 0.05, 0.05};
        return out;
    }

    if (params.debugMode == 1) {
        static const Vec3 kStripColor[] = {
            {0.35, 0.30, 0.20}, {0.15, 0.15, 0.18}, {0.20, 0.28, 0.40}, {0.30, 0.20, 0.35},
            {0.18, 0.38, 0.24}, {0.42, 0.32, 0.12}, {0.14, 0.34, 0.18}, {0.40, 0.36, 0.22},
            {0.45, 0.45, 0.45}, {0.26, 0.26, 0.28}, {0.16, 0.32, 0.14}, {0.55, 0.55, 0.52},
            {0.50, 0.24, 0.22}, {0.22, 0.30, 0.16}};
        out.albedo = kStripColor[int(strip->kind)];
        out.roughness = 0.9;
        return out;
    }

    int sectionIdx = road.xs.sectionIndexAt(s);
    double tInner = 0, tOuter = 0;
    bool haveLane = road.xs.laneSpanAt(sectionIdx, laneId, s, tInner, tOuter);
    double laneCentre = haveLane ? 0.5 * (tInner + tOuter) : 0.0;
    double laneWidth = haveLane ? std::fabs(tOuter - tInner) : 0.0;

    Material m = materialFor(strip->surface, strip->kind, s, t, params.grime, params.seed);

    // Wheel polish: darker, smoother, and where the paint dies first.
    double wheel = 0.0;
    if (strip->isLane() && haveLane) {
        wheel = wheelPathFactor(t - laneCentre, laneWidth);
        m.albedo = m.albedo * (1.0 - 0.22 * wheel * params.grime);
        m.roughness -= 0.16 * wheel;
    }

    // A painted median is hatched, a planted one is not — and the difference is
    // already in the data (surface kind), so nothing new has to be authored.
    double hatch = 0.0;
    if (strip->kind == StripKind::Median && strip->surface == SurfaceKind::Asphalt) {
        double d = std::fmod(std::fmod((s + t) * 0.7071, 1.2) + 1.2, 1.2) - 0.6;
        hatch = coverage(std::fabs(d) - 0.06, fw);
    }

    out.albedo = m.albedo;
    out.roughness = clampd(m.roughness, 0.05, 1.0);
    out.metallic = m.metallic;

    double paintCov = 0.0;
    Vec3 paintCol{0.8, 0.8, 0.8};

    auto accumulate = [&](double cov, Vec3 colour) {
        if (cov <= 1e-4) return;
        // Painted-over-painted keeps the newest colour; coverage saturates.
        double nc = std::min(1.0, paintCov + cov);
        if (cov > paintCov * 0.5) paintCol = colour;
        paintCov = nc;
    };

    if (hatch > 0) accumulate(hatch, paintAlbedo(PaintColor::White));

    // --- lane markings ----------------------------------------------------
    //
    // Delegated to rl_paint.h, which is the same source the GPU backends compile.
    // Keeping a second implementation here is how the CPU reference and the
    // shader drift apart, and drift is invisible: both look like roads.
    if (params.markings) {
        RlBoundary baked[kMaxBakedBoundaries];
        int n = bakeBoundaries(road, s, baked, kMaxBakedBoundaries);
        // Wear's blotch mask is an input rather than something rl_paint computes,
        // so each backend can spend the value noise it already ships.
        float wearMask = float(fbm(s * 0.7 + 31.0, (t + 5.0) * 2.5, 3));
        RlPaint p = rlEvaluateMarkings(baked, n, float(s), float(t), float(fw),
                                       float(params.wear), wearMask, float(wheel));
        if (p.coverage > 1e-4f) accumulate(p.coverage, Vec3{p.r, p.g, p.b});
    }

    // --- glyph decals -----------------------------------------------------
    if (params.decals && paint && !paint->decals.empty()) {
        // Decals are sorted by station, so only the window that can touch this
        // fragment is visited — otherwise every fragment would walk every arrow
        // on the road.
        for (size_t di = paint->lowerBound(s); di < paint->decals.size(); ++di) {
            const Decal& d = paint->decals[di];
            if (d.s - s > paint->maxAlong) break;
            double halfS = d.along * 0.6 + 0.5, halfT = d.across * 0.7 + 0.5;
            if (std::fabs(s - d.s) > halfS || std::fabs(t - d.t) > halfT) continue;
            // Into the decal's own frame. `flip` turns the glyph to face traffic
            // running the other way down the same reference line.
            double du = t - d.t, dv = s - d.s;
            if (d.flip) {
                du = -du;
                dv = -dv;
            }
            if (std::fabs(d.rotate) > 1e-6) {
                double c = std::cos(d.rotate), sn = std::sin(d.rotate);
                double u2 = du * c - dv * sn;
                double v2 = du * sn + dv * c;
                du = u2;
                dv = v2;
            }
            double sd = glyphSdf(d, {du, dv});
            if (sd > fw * 2.0) continue;
            double cov = coverage(sd, fw);
            double wearAmt = clampd(params.wear + d.wear, 0.0, 1.0);
            if (wearAmt > 0.001) {
                double mask = fbm(s * 0.8 + 13.0, t * 2.2 + 4.0, 3);
                cov *= (1.0 - 0.9 * saturate((wearAmt * (0.5 + 0.9 * wheel) - mask + 0.32) * 2.2));
            }
            accumulate(cov, paintAlbedo(d.color));
            // Stop bars sit where every vehicle idles, so the asphalt in front of
            // one is oil-stained. The staining follows the paint, for free.
            if (d.kind == GlyphKind::StopBar) {
                double behind = saturate((d.s - s) / 12.0);
                if (behind > 0 && behind < 1) {
                    out.albedo = out.albedo * (1.0 - 0.25 * (1.0 - behind) * params.grime);
                }
            }
        }
    }

    if (paintCov > 1e-4) {
        out.albedo = lerp(out.albedo, paintCol, paintCov);
        out.roughness = lerp(out.roughness, 0.55, paintCov);
        out.paint = paintCov > 0.5;
    }

    if (params.debugMode == 3) {
        double gs = coverage(std::fabs(std::fmod(std::fmod(s, 10.0) + 10.0, 10.0) - 5.0) - 0.05, fw);
        double gt = coverage(std::fabs(std::fmod(std::fmod(t, 1.0) + 1.0, 1.0) - 0.5) - 0.02, fw);
        out.albedo = lerp(out.albedo, Vec3{0.9, 0.2, 0.2}, gs * 0.7);
        out.albedo = lerp(out.albedo, Vec3{0.2, 0.5, 0.9}, gt * 0.5);
    }

    out.bump = surfaceMicroHeight(road, s, t, params);
    return out;
}

SurfaceSample shadeJunctionPad(const Junction& j, const std::vector<Decal>& decals, Vec2 planPoint,
                               double filterWidth, const ShadeParams& params) {
    // A junction has no arc length, so its pad shades in a local planar frame.
    // Same materials, same SDF glyphs — only the coordinate system differs.
    SurfaceSample out;
    const double fw = std::max(filterWidth, 1e-4);
    Vec2 local = planPoint - j.center;
    Material m = materialFor(SurfaceKind::Asphalt, StripKind::Travel, local.y, local.x,
                             std::min(1.0, params.grime * 1.25), params.seed ^ 0x51u);
    out.albedo = m.albedo;
    out.roughness = m.roughness;

    if (params.debugMode == 1) {
        out.albedo = {0.22, 0.16, 0.16};
        return out;
    }

    double paintCov = 0.0;
    Vec3 paintCol{0.8, 0.8, 0.8};
    if (params.decals) {
        for (const Decal& d : decals) {
            double du = local.x - d.t, dv = local.y - d.s;
            if (std::fabs(du) > d.across * 0.7 + 0.6 || std::fabs(dv) > d.along * 0.7 + 0.6)
                continue;
            if (std::fabs(d.rotate) > 1e-6) {
                double c = std::cos(d.rotate), sn = std::sin(d.rotate);
                double u2 = du * c - dv * sn;
                double v2 = du * sn + dv * c;
                du = u2;
                dv = v2;
            }
            double cov = coverage(glyphSdf(d, {du, dv}), fw);
            if (cov > 1e-4) {
                paintCov = std::min(1.0, paintCov + cov);
                paintCol = paintAlbedo(d.color);
            }
        }
    }
    if (paintCov > 1e-4) {
        out.albedo = lerp(out.albedo, paintCol, paintCov);
        out.roughness = lerp(out.roughness, 0.55, paintCov);
        out.paint = paintCov > 0.5;
    }
    out.bump = 0.0035 * worley(local.y * 3.1, local.x * 3.1);
    return out;
}


// --- rule-driven paint ----------------------------------------------------

void RoadPaint::finalize() {
    std::sort(decals.begin(), decals.end(),
              [](const Decal& a, const Decal& b) { return a.s < b.s; });
    maxAlong = 0;
    for (const Decal& d : decals) maxAlong = std::max(maxAlong, d.along * 0.6 + 0.6);
}

size_t RoadPaint::lowerBound(double s) const {
    double key = s - maxAlong;
    size_t lo = 0, hi = decals.size();
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (decals[mid].s < key)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

namespace {

Decal makeText(const char* txt, double s, double t, double along, double across, bool flip) {
    Decal d;
    d.kind = GlyphKind::Text;
    d.s = s;
    d.t = t;
    d.along = along;
    d.across = across;
    d.flip = flip;
    std::snprintf(d.text, sizeof d.text, "%s", txt);
    return d;
}

// Which glyph a lane wants, given the movements it actually feeds. This is the
// point of deriving paint from topology: the arrow cannot disagree with where
// the lane goes, because it is computed FROM where the lane goes.
GlyphKind arrowForMovements(bool through, bool left, bool right) {
    if (through && left) return GlyphKind::ArrowThroughLeft;
    if (through && right) return GlyphKind::ArrowThroughRight;
    if (left) return GlyphKind::ArrowLeft;
    if (right) return GlyphKind::ArrowRight;
    return GlyphKind::ArrowThrough;
}

}  // namespace

void generateRoadPaint(const Network& net, std::vector<RoadPaint>& paint) {
    paint.assign(size_t(net.roadCount()), RoadPaint{});

    // --- 1. Junction approaches: lane-use arrows, stop bars, zebras --------
    for (const Junction& j : net.junctions()) {
        for (size_t ai = 0; ai < j.arms.size(); ++ai) {
            const JunctionArm& arm = j.arms[ai];
            const Road& r = net.road(arm.road);
            if (r.kind == RoadKind::Connector) continue;
            RoadPaint& rp = paint[size_t(arm.road)];
            // Upstream is whichever way leads away from the pad.
            double up = arm.atEnd ? -1.0 : +1.0;
            int sec = r.xs.sectionIndexAt(arm.atEnd ? std::max(r.begin(), arm.sContact - 1e-3)
                                                    : arm.sContact + 1e-3);
            EndLanes el = endLanes(r, arm.atEnd);

            double barT0 = 1e9, barT1 = -1e9;
            for (int laneId : el.incoming) {
                const Strip* st = r.xs.sections[size_t(std::max(0, sec))].strip(laneId);
                if (!st) continue;
                bool through = false, left = false, right = false;
                for (const Connection& c : j.connections) {
                    if (c.from.road != r.id || c.from.lane != laneId) continue;
                    if (c.turn == TurnKind::Through) through = true;
                    if (c.turn == TurnKind::Left || c.turn == TurnKind::UTurn) left = true;
                    if (c.turn == TurnKind::Right) right = true;
                }
                if (!through && !left && !right) continue;

                double t0 = 0, t1 = 0;
                r.xs.laneSpanAt(sec, laneId, arm.sContact, t0, t1);
                barT0 = std::min(barT0, std::min(t0, t1));
                barT1 = std::max(barT1, std::max(t0, t1));
                double centre = 0.5 * (t0 + t1);
                double width = std::fabs(t1 - t0);

                Decal a;
                a.kind = arrowForMovements(through, left, right);
                a.t = centre;
                a.along = 4.6;
                a.across = std::max(1.6, width * 0.62);
                a.flip = st->dir < 0;
                // Two sets, the way they are actually painted: one at the head of
                // the queue and one back where a driver still has room to move.
                for (double back : {9.0, 26.0}) {
                    double s = arm.sContact + up * back;
                    if (s < r.begin() + 2.0 || s > r.end() - 2.0) continue;
                    a.s = s;
                    rp.decals.push_back(a);
                }
                // A dedicated turn bay gets ONLY under its arrow.
                if (st->kind == StripKind::Turn && !through) {
                    double s = arm.sContact + up * 16.0;
                    if (s > r.begin() + 2.0 && s < r.end() - 2.0)
                        rp.decals.push_back(
                            makeText("ONLY", s, centre, 4.2, std::max(1.4, width * 0.5),
                                     st->dir < 0));
                }
            }

            if (barT1 > barT0) {
                bool stopBar = j.control == JunctionControl::Signalized ||
                               j.control == JunctionControl::AllWayStop ||
                               (j.control == JunctionControl::PriorityStop &&
                                arm.priority < 1e9 && el.incoming.size() > 0);
                bool yieldTeeth = j.control == JunctionControl::RoundaboutEntry ||
                                  j.control == JunctionControl::Yield;
                // The stop line sits behind the crossing, not on it.
                double offset = 2.0;
                if (j.generateCrosswalks && r.allowsPedestrians && r.designSpeed < 90) offset = 6.0;
                double s = arm.sContact + up * offset;
                if (s > r.begin() + 0.5 && s < r.end() - 0.5) {
                    if (stopBar) {
                        Decal bar;
                        bar.kind = GlyphKind::StopBar;
                        bar.s = s;
                        bar.t = 0.5 * (barT0 + barT1);
                        bar.along = 0.5;
                        bar.across = barT1 - barT0;
                        rp.decals.push_back(bar);
                    } else if (yieldTeeth) {
                        Decal teeth;
                        teeth.kind = GlyphKind::YieldTeeth;
                        teeth.s = s;
                        teeth.t = 0.5 * (barT0 + barT1);
                        teeth.along = 0.6;
                        teeth.across = barT1 - barT0;
                        teeth.flip = up > 0;
                        rp.decals.push_back(teeth);
                    }
                }
            }
        }

        for (const Crosswalk& cw : junctionCrosswalks(net, j)) {
            Decal z;
            z.kind = cw.signalised ? GlyphKind::CrosswalkLadder : GlyphKind::CrosswalkZebra;
            z.s = cw.s;
            z.t = 0.5 * (cw.tMin + cw.tMax);
            z.along = cw.depth;
            z.across = cw.tMax - cw.tMin;
            paint[size_t(cw.road)].decals.push_back(z);
        }
    }

    // --- 2. Cross-section rules -------------------------------------------
    for (const Road& r : net.roads()) {
        RoadPaint& rp = paint[size_t(r.id)];
        if (r.kind == RoadKind::Connector) continue;

        // Stations near a junction, so parking and symbols keep clear of them.
        std::vector<double> junctionStations;
        for (const Junction& j : net.junctions())
            for (const JunctionArm& a : j.arms)
                if (a.road == r.id) junctionStations.push_back(a.sContact);
        auto nearJunction = [&](double s, double dist) {
            for (double js : junctionStations)
                if (std::fabs(s - js) < dist) return true;
            return s < r.begin() + 4.0 || s > r.end() - 4.0;
        };

        for (size_t si = 0; si < r.xs.sections.size(); ++si) {
            const LaneSection& sec = r.xs.sections[si];
            double s0 = std::max(sec.s0, r.begin());
            double s1 = std::min(sec.s0 + sec.length, r.end());
            if (s1 - s0 < 1.0) continue;

            for (const std::vector<Strip>* stack : {&sec.left, &sec.right}) {
                for (const Strip& st : *stack) {
                    double wStart = std::max(0.0, st.width.eval(s0 - sec.s0));
                    double wEnd = std::max(0.0, st.width.eval(s1 - sec.s0));
                    double mid = 0.5 * (s0 + s1);
                    double tMid = r.xs.laneCenterT(int(si), st.id, mid);
                    double wMid = std::max(wStart, wEnd);

                    // A lane that tapers to nothing gets merge arrows, and the
                    // arrow's direction comes from which side of the road it is
                    // on — the same fact that made it the lane to drop.
                    if (st.isLane() && wStart > 1.5 && wEnd < 0.6) {
                        for (double f : {0.30, 0.62}) {
                            double s = lerp(s0, s1, f);
                            Decal a;
                            a.kind = GlyphKind::ArrowMergeLeft;
                            a.s = s;
                            a.t = r.xs.laneCenterT(int(si), st.id, s);
                            a.along = 7.0;
                            a.across = std::max(1.8, wStart * 0.66);
                            a.flip = st.dir < 0;
                            rp.decals.push_back(a);
                        }
                        rp.decals.push_back(makeText("MERGE", lerp(s0, s1, 0.06),
                                                     r.xs.laneCenterT(int(si), st.id,
                                                                      lerp(s0, s1, 0.06)),
                                                     5.5, std::max(1.4, wStart * 0.5),
                                                     st.dir < 0));
                    }
                    // A lane growing from nothing is an auxiliary/acceleration
                    // lane; the wedge at its start is the gore, and gores are
                    // chevroned.
                    if (st.isLane() && wStart < 0.6 && wEnd > 1.5) {
                        double goreLen = std::min(22.0, (s1 - s0) * 0.35);
                        Decal g;
                        g.kind = GlyphKind::ChevronGore;
                        g.s = s0 + goreLen * 0.5;
                        g.t = r.xs.laneCenterT(int(si), st.id, s0 + goreLen * 0.5);
                        g.along = goreLen;
                        g.across = std::max(1.2, wEnd * 0.45);
                        g.flip = st.dir < 0;
                        rp.decals.push_back(g);
                    }

                    if (wMid < 1.0) continue;

                    // Parking bays: tees at the bay divisions, kept clear of the
                    // junction the way a hydrant setback would be.
                    if (st.kind == StripKind::Parking) {
                        for (double s = s0 + 3.0; s < s1 - 3.0; s += 6.5) {
                            if (nearJunction(s, 12.0)) continue;
                            Decal d;
                            d.kind = GlyphKind::ParkingTee;
                            d.s = s;
                            d.t = r.xs.laneCenterT(int(si), st.id, s);
                            d.along = 1.0;
                            d.across = std::max(1.4, wMid * 0.8);
                            rp.decals.push_back(d);
                        }
                    }
                    if (st.kind == StripKind::Bike) {
                        for (double s = s0 + 12.0; s < s1 - 8.0; s += 45.0) {
                            Decal d;
                            d.kind = GlyphKind::BikeSymbol;
                            d.s = s;
                            d.t = r.xs.laneCenterT(int(si), st.id, s);
                            d.along = 2.4;
                            d.across = std::max(0.9, wMid * 0.6);
                            d.flip = st.dir < 0;
                            rp.decals.push_back(d);
                        }
                    }
                    if (st.kind == StripKind::Bus) {
                        for (double s = s0 + 20.0; s < s1 - 12.0; s += 70.0) {
                            Decal d;
                            d.kind = GlyphKind::HovDiamond;
                            d.s = s;
                            d.t = r.xs.laneCenterT(int(si), st.id, s);
                            d.along = 2.6;
                            d.across = std::max(1.2, wMid * 0.5);
                            rp.decals.push_back(d);
                            rp.decals.push_back(makeText("BUS", s + (st.dir < 0 ? -7.0 : 7.0),
                                                         d.t, 4.0, std::max(1.3, wMid * 0.5),
                                                         st.dir < 0));
                        }
                    }
                    // A two-way left-turn lane is signed on the road itself.
                    if (st.kind == StripKind::Turn && st.dir == 0) {
                        for (double s = s0 + 30.0; s < s1 - 20.0; s += 90.0) {
                            Decal a;
                            a.kind = GlyphKind::ArrowLeft;
                            a.s = s;
                            a.t = tMid;
                            a.along = 4.0;
                            a.across = 1.5;
                            rp.decals.push_back(a);
                            a.s = s + 12.0;
                            a.flip = true;
                            rp.decals.push_back(a);
                        }
                    }
                }
            }
        }

        // Speed painted on the pavement where the limit changes — derived from
        // the design speed the geometry was built to, not typed twice.
        if (r.pred.type == LinkType::Road && r.pred.id >= 0) {
            double prev = net.road(r.pred.id).designSpeed;
            if (std::fabs(prev - r.designSpeed) > 5.0) {
                int sec = r.xs.sectionIndexAt(r.begin() + 1e-3);
                EndLanes el = endLanes(r, false);
                if (!el.outgoing.empty()) {
                    int laneId = el.outgoing.front();
                    const Strip* st = r.xs.sections[size_t(std::max(0, sec))].strip(laneId);
                    double s = r.begin() + 25.0;
                    if (st && s < r.end() - 10.0) {
                        char num[8];
                        std::snprintf(num, sizeof num, "%d", int(r.designSpeed + 0.5));
                        rp.decals.push_back(makeText(num, s, r.xs.laneCenterT(sec, laneId, s),
                                                     4.0, 1.8, st->dir < 0));
                    }
                }
            }
        }
    }

    for (RoadPaint& rp : paint) rp.finalize();
}

}  // namespace roadlab
