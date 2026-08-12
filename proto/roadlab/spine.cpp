#include "spine.h"

#include <cassert>

namespace roadlab {

// --- Profile1D ------------------------------------------------------------

namespace {

const Profile1D::Piece* pieceAt(const std::vector<Profile1D::Piece>& pieces, double s) {
    if (pieces.empty()) return nullptr;
    // Pieces are appended in increasing s0; a linear walk from the back is
    // faster than a binary search for the handful of pieces a road carries.
    for (size_t i = pieces.size(); i-- > 0;) {
        if (s >= pieces[i].s0 - 1e-9) return &pieces[i];
    }
    return &pieces.front();
}

}  // namespace

double Profile1D::eval(double s) const {
    const Piece* p = pieceAt(pieces, s);
    return p ? p->poly.eval(s - p->s0) : 0.0;
}

double Profile1D::slope(double s) const {
    const Piece* p = pieceAt(pieces, s);
    return p ? p->poly.deriv(s - p->s0) : 0.0;
}

// --- Spine construction ---------------------------------------------------

void Spine::setStart(Vec2 p, double heading) {
    start_ = p;
    startHdg_ = heading;
    finalized_ = false;
}

void Spine::addLine(double length) {
    if (length <= 1e-9) return;
    GeomPrim g;
    g.kind = GeomKind::Line;
    g.length = length;
    prims_.push_back(g);
    finalized_ = false;
}

void Spine::addArc(double length, double curvature) {
    if (length <= 1e-9) return;
    if (std::fabs(curvature) < 1e-9) return addLine(length);
    GeomPrim g;
    g.kind = GeomKind::Arc;
    g.length = length;
    g.curv0 = g.curv1 = curvature;
    prims_.push_back(g);
    finalized_ = false;
}

void Spine::addSpiral(double length, double curvStart, double curvEnd) {
    if (length <= 1e-9) return;
    if (std::fabs(curvStart - curvEnd) < 1e-12) return addArc(length, curvStart);
    GeomPrim g;
    g.kind = GeomKind::Spiral;
    g.length = length;
    g.curv0 = curvStart;
    g.curv1 = curvEnd;
    prims_.push_back(g);
    finalized_ = false;
}

void Spine::addAnchored(GeomKind kind, Vec2 p0, double heading, double length, double curvStart,
                        double curvEnd) {
    if (length <= 1e-9) return;
    GeomPrim g;
    g.kind = kind;
    g.length = length;
    g.curv0 = curvStart;
    g.curv1 = (kind == GeomKind::Spiral) ? curvEnd : curvStart;
    g.p0 = p0;
    g.hdg0 = heading;
    g.anchored = true;
    if (prims_.empty()) {
        start_ = p0;
        startHdg_ = heading;
    }
    prims_.push_back(g);
    finalized_ = false;
}

void Spine::setFlatElevation(double y) { elevation_.set(y); }

void Spine::setElevationKnots(const std::vector<Vec2>& sHeight) {
    elevation_.clear();
    if (sHeight.empty()) return;
    if (sHeight.size() == 1) {
        elevation_.set(sHeight[0].y);
        return;
    }
    // Catmull-Rom style tangents give C1 continuity, so the grade never steps —
    // a stepped grade reads as a kink in the road and shows up immediately in
    // both the render and the vehicle pitch.
    const size_t n = sHeight.size();
    std::vector<double> m(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        size_t a = (i == 0) ? 0 : i - 1;
        size_t b = (i + 1 == n) ? i : i + 1;
        double ds = sHeight[b].x - sHeight[a].x;
        m[i] = ds > 1e-9 ? (sHeight[b].y - sHeight[a].y) / ds : 0.0;
    }
    for (size_t i = 0; i + 1 < n; ++i) {
        double len = sHeight[i + 1].x - sHeight[i].x;
        if (len <= 1e-9) continue;
        elevation_.add(sHeight[i].x,
                       Poly3::hermite(sHeight[i].y, m[i], sHeight[i + 1].y, m[i + 1], len));
    }
}

void Spine::applySuperelevation(double designSpeedKph, double maxE) {
    if (!finalized_) finalize();
    super_.clear();
    if (prims_.empty() || designSpeedKph <= 0) {
        super_.set(0.0);
        return;
    }
    // Target bank for a curve of curvature k: e = v^2 / (127 R), the standard
    // point-mass balance with R in metres and v in km/h, clamped to maxE.
    auto targetRoll = [&](double k) {
        double ak = std::fabs(k);
        if (ak < 1e-6) return 0.0;
        double R = 1.0 / ak;
        double e = clampd(designSpeedKph * designSpeedKph / (127.0 * R), 0.0, maxE);
        // Positive roll raises the left. Banking into a left turn (k > 0) means
        // dropping the left side, hence the negation.
        return -(k > 0 ? 1.0 : -1.0) * std::atan(e);
    };

    for (const GeomPrim& g : prims_) {
        switch (g.kind) {
            case GeomKind::Line:
                super_.add(g.s0, Poly3::constant(0.0));
                break;
            case GeomKind::Arc:
                super_.add(g.s0, Poly3::constant(targetRoll(g.curv0)));
                break;
            case GeomKind::Spiral:
                // The runoff rides the spiral exactly: curvature and bank reach
                // their full value at the same station, which is the whole point
                // of having a transition curve.
                super_.add(g.s0, Poly3::smooth(targetRoll(g.curv0), targetRoll(g.curv1), g.length));
                break;
        }
    }
}

void Spine::evalPrim(const GeomPrim& g, double ds, Vec2& p, double& hdg, double& curv) const {
    ds = clampd(ds, 0.0, g.length);
    switch (g.kind) {
        case GeomKind::Line: {
            p = g.p0 + dirOf(g.hdg0) * ds;
            hdg = g.hdg0;
            curv = 0.0;
            break;
        }
        case GeomKind::Arc: {
            double k = g.curv0;
            hdg = g.hdg0 + k * ds;
            p = {g.p0.x + (std::sin(hdg) - std::sin(g.hdg0)) / k,
                 g.p0.y + (std::cos(g.hdg0) - std::cos(hdg)) / k};
            curv = k;
            break;
        }
        case GeomKind::Spiral: {
            // k(u) = k0 + c*u  =>  hdg(u) = hdg0 + k0*u + c*u^2/2, and position is
            // the integral of (cos hdg, sin hdg) du. No closed form; split into
            // chunks of bounded heading sweep and Gauss-Legendre each one.
            double c = (g.curv1 - g.curv0) / g.length;
            auto headingAt = [&](double u) { return g.hdg0 + g.curv0 * u + 0.5 * c * u * u; };
            double sweep = std::fabs(headingAt(ds) - g.hdg0);
            int nseg = std::max(1, int(std::ceil(sweep / 0.25)));
            nseg = std::min(nseg, 256);
            double h = ds / nseg;
            Vec2 acc = g.p0;
            for (int i = 0; i < nseg; ++i) {
                double u0 = i * h;
                acc.x += gauss8([&](double u) { return std::cos(headingAt(u0 + u)); }, h);
                acc.y += gauss8([&](double u) { return std::sin(headingAt(u0 + u)); }, h);
            }
            p = acc;
            hdg = headingAt(ds);
            curv = g.curv0 + c * ds;
            break;
        }
    }
}

void Spine::primEnd(const GeomPrim& g, Vec2& p, double& hdg) const {
    double curv = 0;
    evalPrim(g, g.length, p, hdg, curv);
}

void Spine::finalize(double maxSampleStep) {
    Vec2 p = start_;
    double hdg = startHdg_;
    double s = 0;
    for (GeomPrim& g : prims_) {
        g.s0 = s;
        if (g.anchored) {
            p = g.p0;
            hdg = g.hdg0;
        } else {
            g.p0 = p;
            g.hdg0 = hdg;
        }
        primEnd(g, p, hdg);
        s += g.length;
    }
    length_ = s;
    if (elevation_.empty()) elevation_.set(0.0);
    if (super_.empty()) super_.set(0.0);
    if (offset_.empty()) offset_.set(0.0);

    // Sample cache. Step is curvature-adaptive so a tight roundabout ring is
    // sampled finely and a freeway tangent is not oversampled.
    samples_.clear();
    double cursor = 0.0;
    while (true) {
        Vec2 sp;
        double sh = 0, sk = 0;
        const GeomPrim* g = primAt(cursor);
        if (g) {
            evalPrim(*g, cursor - g->s0, sp, sh, sk);
        } else {
            sp = start_;
            sh = startHdg_;
        }
        samples_.push_back({cursor, sp, sh, sk});
        if (cursor >= length_ - 1e-9) break;
        double step = maxSampleStep;
        if (std::fabs(sk) > 1e-6) step = std::min(step, std::max(0.25, 0.1 / std::fabs(sk)));
        cursor = std::min(length_, cursor + step);
    }
    finalized_ = true;
}

const GeomPrim* Spine::primAt(double s) const {
    if (prims_.empty()) return nullptr;
    for (size_t i = prims_.size(); i-- > 0;) {
        if (s >= prims_[i].s0 - 1e-9) return &prims_[i];
    }
    return &prims_.front();
}

Frame Spine::frameAt(double s) const {
    Frame f;
    f.s = clampd(s, 0.0, length_);
    const GeomPrim* g = primAt(f.s);
    if (g) {
        evalPrim(*g, f.s - g->s0, f.planPos, f.heading, f.curvature);
    } else {
        f.planPos = start_;
        f.heading = startHdg_;
    }
    double y = elevation_.eval(f.s);
    f.grade = elevation_.slope(f.s);
    f.roll = super_.eval(f.s);
    f.pos = worldOf(f.planPos, y);

    // The cross-section is horizontal in plan, so `left` is the plan normal and
    // never picks up the grade; only `forward` does. `up` follows.
    Vec3 fwd = normalize(Vec3{std::cos(f.heading), f.grade, std::sin(f.heading)});
    Vec3 left{-std::sin(f.heading), 0.0, std::cos(f.heading)};
    Vec3 up = normalize(cross(left, fwd));
    double cr = std::cos(f.roll), sr = std::sin(f.roll);
    f.forward = fwd;
    f.left = normalize(left * cr + up * sr);
    f.up = normalize(up * cr - left * sr);
    return f;
}

double Spine::curvatureAt(double s) const {
    const GeomPrim* g = primAt(clampd(s, 0.0, length_));
    if (!g) return 0.0;
    if (g->kind == GeomKind::Spiral) {
        double c = (g->curv1 - g->curv0) / g->length;
        return g->curv0 + c * clampd(s - g->s0, 0.0, g->length);
    }
    return g->curv0;
}

Vec3 Spine::toWorld(double s, double t, double h) const {
    Frame f = frameAt(s);
    double tt = t + offset_.eval(clampd(s, 0.0, length_));
    // Crossfall sheds water from the crown outward. It is a shape, not a bank:
    // both sides drop, which is why it is |t| and not t.
    double crown = -std::fabs(tt) * crossfall_;
    return f.pos + f.left * tt + f.up * (h + crown);
}

Vec2 Spine::toPlan(double s, double t) const {
    Frame f = frameAt(s);
    double tt = t + offset_.eval(clampd(s, 0.0, length_));
    return f.planPos + perpLeft(dirOf(f.heading)) * tt;
}

bool Spine::toST(Vec2 planPoint, double& s, double& t, double sTolerance) const {
    if (samples_.empty()) return false;
    // Coarse: nearest cached sample. Fine: Newton on d/ds |P - C(s)|^2 = 0, whose
    // derivative includes the curvature term so it converges inside tight arcs
    // too.
    // Coarse-to-fine over the sample cache. A flat scan is O(n) and this runs
    // per terrain vertex per road, which made ground generation the slowest
    // thing in the system; a sqrt-stride pass then a local refine is O(2*sqrt(n))
    // and finds the same seed for any curve that is locally well behaved.
    const size_t n = samples_.size();
    size_t stride = std::max<size_t>(1, size_t(std::sqrt(double(n))));
    size_t best = 0;
    double bestD = 1e300;
    for (size_t i = 0; i < n; i += stride) {
        double d = lengthSq(planPoint - samples_[i].planPos);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    size_t lo = best > stride ? best - stride : 0;
    size_t hi = std::min(n - 1, best + stride);
    for (size_t i = lo; i <= hi; ++i) {
        double d = lengthSq(planPoint - samples_[i].planPos);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    double cur = samples_[best].s;
    for (int iter = 0; iter < 8; ++iter) {
        const GeomPrim* g = primAt(clampd(cur, 0.0, length_));
        Vec2 c;
        double hdg = 0, k = 0;
        if (!g) break;
        evalPrim(*g, clampd(cur, 0.0, length_) - g->s0, c, hdg, k);
        Vec2 tan = dirOf(hdg);
        Vec2 nrm = perpLeft(tan);
        Vec2 d = planPoint - c;
        // Newton on f(s) = (P - C(s))·T(s) = 0, so the step is -f/f'. Getting the
        // sign wrong here does not fail loudly: it walks AWAY from the foot of
        // the perpendicular, doubling the seed's error every iteration, and the
        // residual guard below then rejects the answer. The symptom is an
        // inverse map that quietly never resolves anything.
        double f = dot(d, tan);
        double fp = -1.0 + k * dot(d, nrm);
        if (std::fabs(fp) < 1e-9) break;
        double step = clampd(-f / fp, -20.0, 20.0);
        cur += step;
        if (std::fabs(step) < 1e-7) break;
    }
    double sc = clampd(cur, 0.0, length_);
    Frame f = frameAt(sc);
    Vec2 rel = planPoint - f.planPos;
    Vec2 tan = dirOf(f.heading);
    s = sc;
    t = dot(rel, perpLeft(tan)) - offset_.eval(sc);

    // A genuine foot of perpendicular has NO tangential residual. Reporting `t`
    // without checking that is how a point fifty metres off the end of an arc
    // came back as "four metres from the centreline" — which is what punched
    // wedge-shaped holes in the terrain around every junction.
    double residual = dot(rel, tan);
    if (std::fabs(residual) > 0.25) return false;
    return cur >= -sTolerance && cur <= length_ + sTolerance;
}

void Spine::planBounds(Vec2& lo, Vec2& hi) const {
    lo = {1e300, 1e300};
    hi = {-1e300, -1e300};
    for (const SpineSample& sm : samples_) {
        lo.x = std::min(lo.x, sm.planPos.x);
        lo.y = std::min(lo.y, sm.planPos.y);
        hi.x = std::max(hi.x, sm.planPos.x);
        hi.y = std::max(hi.y, sm.planPos.y);
    }
    if (samples_.empty()) {
        lo = hi = start_;
    }
}

// --- builders -------------------------------------------------------------

Spine spineFromPolyline(const std::vector<Vec2>& points, double cornerRadius, bool useSpirals) {
    Spine sp;
    if (points.size() < 2) {
        sp.setStart(points.empty() ? Vec2{0, 0} : points[0], 0.0);
        sp.finalize();
        return sp;
    }
    const size_t n = points.size();
    std::vector<Vec2> dir(n - 1);
    std::vector<double> segLen(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        Vec2 d = points[i + 1] - points[i];
        segLen[i] = length(d);
        dir[i] = segLen[i] > 1e-9 ? d * (1.0 / segLen[i]) : Vec2{1, 0};
    }

    struct Corner {
        double deflect = 0;   // signed, +ve = left
        double radius = 0;
        double spiralLen = 0;
        double tangent = 0;   // setback from the corner point
        bool active = false;
    };
    std::vector<Corner> corners(n);

    auto solveCorner = [&](size_t i, double radius) {
        Corner c;
        c.deflect = angleDiff(headingOf(dir[i]), headingOf(dir[i - 1]));
        double ad = std::fabs(c.deflect);
        if (ad < 0.5 * kDeg2Rad || radius <= 0.1) return c;   // straight through
        // A deflection this sharp cannot be rounded at any useful radius; leave
        // it as a hard corner and let the junction/hairpin machinery own it.
        ad = std::min(ad, 170.0 * kDeg2Rad);
        c.radius = radius;
        if (useSpirals) {
            // Spiral length from a comfortable curvature rate; capped so the two
            // spirals never eat more deflection than the corner has.
            double ls = std::min(radius * 0.6, radius * ad * 0.45);
            double maxLs = 2.0 * radius * ad * 0.49;
            c.spiralLen = clampd(ls, 0.0, maxLs);
        }
        double p = c.spiralLen * c.spiralLen / (24.0 * radius);   // curve shift
        c.tangent = (radius + p) * std::tan(ad * 0.5) + c.spiralLen * 0.5;
        c.active = true;
        return c;
    };

    std::vector<double> radii(n, cornerRadius);
    for (int pass = 0; pass < 8; ++pass) {
        for (size_t i = 1; i + 1 < n; ++i) corners[i] = solveCorner(i, radii[i]);
        bool shrank = false;
        for (size_t i = 0; i + 1 < n; ++i) {
            double need = (i > 0 ? corners[i].tangent : 0.0) +
                          (i + 2 < n ? corners[i + 1].tangent : 0.0);
            if (need > segLen[i] - 0.05) {
                double scale = std::max(0.15, (segLen[i] - 0.05) / std::max(need, 1e-6));
                if (i > 0) radii[i] *= scale;
                if (i + 2 < n) radii[i + 1] *= scale;
                shrank = true;
            }
        }
        if (!shrank) break;
    }
    for (size_t i = 1; i + 1 < n; ++i) corners[i] = solveCorner(i, radii[i]);

    sp.setStart(points[0], headingOf(dir[0]));
    double consumed = 0.0;   // how much of the current segment the last corner ate
    for (size_t i = 1; i + 1 < n; ++i) {
        const Corner& c = corners[i];
        double straight = segLen[i - 1] - consumed - (c.active ? c.tangent : 0.0);
        if (straight > 1e-6) sp.addLine(straight);
        if (!c.active) {
            consumed = 0.0;
            continue;
        }
        double sign = c.deflect > 0 ? 1.0 : -1.0;
        double k = sign / c.radius;
        double ad = std::min(std::fabs(c.deflect), 170.0 * kDeg2Rad);
        double spiralDeflect = c.spiralLen / (2.0 * c.radius);
        double arcDeflect = std::max(0.0, ad - 2.0 * spiralDeflect);
        if (c.spiralLen > 1e-6) sp.addSpiral(c.spiralLen, 0.0, k);
        if (arcDeflect > 1e-9) sp.addArc(arcDeflect * c.radius, k);
        if (c.spiralLen > 1e-6) sp.addSpiral(c.spiralLen, k, 0.0);
        consumed = c.tangent;
    }
    double tail = segLen[n - 2] - consumed;
    if (tail > 1e-6) sp.addLine(tail);
    sp.finalize();
    return sp;
}

Spine spineArc(Vec2 center, double radius, double a0, double a1, bool ccw) {
    Spine sp;
    double sweep = ccw ? wrapPi(a1 - a0) : wrapPi(a0 - a1);
    if (sweep <= 0) sweep += kTwoPi;
    double k = (ccw ? 1.0 : -1.0) / radius;
    Vec2 p{center.x + radius * std::cos(a0), center.y + radius * std::sin(a0)};
    sp.setStart(p, ccw ? a0 + kPi * 0.5 : a0 - kPi * 0.5);
    // Split so no single primitive sweeps more than a quadrant, which keeps the
    // sample cache evenly spaced around the ring.
    int parts = std::max(1, int(std::ceil(sweep / (kPi * 0.5))));
    for (int i = 0; i < parts; ++i) sp.addArc(sweep * radius / parts, k);
    sp.finalize();
    return sp;
}

Spine spineCircle(Vec2 center, double radius, bool ccw) {
    Spine sp;
    double k = (ccw ? 1.0 : -1.0) / radius;
    // Start at the +X point of the ring, heading tangentially.
    Vec2 p{center.x + radius, center.y};
    sp.setStart(p, ccw ? kPi * 0.5 : -kPi * 0.5);
    // Four quadrant arcs rather than one full turn: keeps every primitive's
    // heading sweep modest, which keeps the sample cache well conditioned.
    for (int i = 0; i < 4; ++i) sp.addArc(0.5 * kPi * radius, k);
    sp.finalize();
    return sp;
}

Spine spineConnector(Vec2 p0, double hdg0, Vec2 p1, double hdg1) {
    // Biarc: two tangent-continuous arcs that hit both poses exactly. Cheaper
    // and more robust than fitting a clothoid pair, and for the short paths
    // inside a junction the curvature step at the joint is not observable.
    Spine sp;
    sp.setStart(p0, hdg0);
    Vec2 t0 = dirOf(hdg0), t1 = dirOf(hdg1);
    Vec2 v = p1 - p0;
    double vv = dot(v, v);
    if (vv < 1e-9) {
        sp.finalize();
        return sp;
    }
    Vec2 tsum = t0 + t1;
    double denom = 2.0 * (1.0 - dot(t0, t1));
    double d;
    if (denom < 1e-9) {
        double vt1 = dot(v, t1);
        // Parallel tangents: either a straight shot or a symmetric S/U turn.
        if (std::fabs(vt1) < 1e-9) {
            sp.addLine(std::sqrt(vv));
            sp.finalize();
            return sp;
        }
        d = vv / (4.0 * vt1);
    } else {
        double vt = dot(v, tsum);
        double disc = vt * vt + denom * vv;
        d = (-vt + std::sqrt(std::max(0.0, disc))) / denom;
    }
    // Keep the joint in a sane place. An ill-conditioned solve can put it behind
    // the start pose, which turns the first arc into a near-full circle.
    double chord = std::sqrt(vv);
    if (!(d > 1e-6) || !std::isfinite(d)) d = chord * 0.5;
    d = clampd(d, chord * 0.15, chord * 1.2);

    Vec2 pm = (p0 + t0 * d + (p1 - t1 * d)) * 0.5;

    // Append the arc (or straight) that leaves `from` along `tan` and lands on
    // `to`. Curvature from the chord: k = 2 * cross(tan, chord) / |chord|^2.
    auto addArcTo = [&](Vec2 from, Vec2 tan, Vec2 to) {
        Vec2 c = to - from;
        double cl = length(c);
        if (cl < 1e-7) return;
        // phi is the SIGNED angle from the tangent to the chord, so the arc turn
        // is 2*phi. Using asin here instead would silently pick the minor arc and
        // send a tight turn the long way round the circle.
        double phi = std::atan2(cross(tan, c), dot(tan, c));
        if (std::fabs(phi) < 1e-6) {
            sp.addLine(cl);
            return;
        }
        double k = 2.0 * std::sin(phi) / cl;
        if (std::fabs(k) < 1e-7) {
            sp.addLine(cl);
            return;
        }
        sp.addArc(std::fabs(phi) * cl / std::fabs(std::sin(phi)), k);
    };

    addArcTo(p0, t0, pm);
    // Take the joint tangent from the arc we actually built rather than from the
    // biarc algebra, so the two halves are G1 by construction even where the
    // solve was ill-conditioned.
    sp.finalize();
    Frame joint = sp.length() > 0 ? sp.frameAt(sp.length()) : Frame{};
    Vec2 jointPos = sp.length() > 0 ? joint.planPos : p0;
    Vec2 jointTan = sp.length() > 0 ? dirOf(joint.heading) : t0;
    addArcTo(jointPos, jointTan, p1);
    sp.finalize();
    // Guard against a degenerate solve. If the poses are badly posed relative to
    // each other (a junction whose arms do not actually meet, say) the biarc can
    // swing most of the way round a circle. A straight connector is the bounded,
    // obviously-wrong-looking answer, which is what you want a bad input to
    // produce — not a silent four-kilometre arc.
    if (sp.length() > 5.0 * chord + 5.0) {
        Spine flat;
        flat.setStart(p0, headingOf(normalize(p1 - p0)));
        flat.addLine(chord);
        flat.finalize();
        return flat;
    }
    return sp;
}

}  // namespace roadlab
