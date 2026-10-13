#include "rail_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace citysim {

using engine::Mat4;
using engine::Vec3;

TrainKind trainKindNamed(const std::string& name) {
    TrainKind k;
    k.name = name;
    if (name == "subway") {            // short cars, quick doors, level boarding, no locomotive to speak of
        k.locoLength = 18.0; k.carLength = 18.0; k.cars = 4; k.vmax = 22.0; k.accel = 1.0; k.decel = 1.0; k.dwell = 22.0;
    } else if (name == "light_rail") { // two articulated sections behind a cab car, street speeds
        k.locoLength = 14.0; k.carLength = 14.0; k.cars = 2; k.vmax = 18.0; k.accel = 1.1; k.decel = 1.1; k.dwell = 20.0;
    }                                  // "intercity" (the default): a locomotive and three long coaches
    return k;
}

namespace {

double dist(const Vec3& a, const Vec3& b) { return (a - b).length(); }

Vec3 flatDir(const Vec3& a, const Vec3& b) {
    Vec3 d = b - a;
    d.y = 0;
    const double l = d.length();
    return l > 1e-9 ? d * (1.0 / l) : Vec3(0, 0, 1);
}

// The curvature of the circle through three points (0 when they are in line).
double curvature(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double ab = dist(a, b), bc = dist(b, c), ca = dist(c, a);
    const double cross = std::fabs((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x));   // twice the plan area
    const double den = ab * bc * ca;
    return den > 1e-9 ? 2.0 * cross / den : 0.0;
}

}  // namespace

double RailLine::wrap(double st) const {
    if (length <= 0) return 0;
    if (loop) {
        double w = std::fmod(st, length);
        return w < 0 ? w + length : w;
    }
    return std::clamp(st, 0.0, length);
}

Vec3 RailLine::at(double st) const {
    if (pts.empty()) return Vec3(0, 0, 0);
    if (pts.size() == 1) return pts.front();
    const double w = wrap(st);
    const auto it = std::upper_bound(s.begin(), s.end(), w);
    std::size_t i = it == s.begin() ? 0 : static_cast<std::size_t>(it - s.begin()) - 1;
    if (i + 1 >= pts.size()) i = pts.size() - 2;
    const double seg = s[i + 1] - s[i];
    const double u = seg > 1e-9 ? std::clamp((w - s[i]) / seg, 0.0, 1.0) : 0.0;
    return pts[i] + (pts[i + 1] - pts[i]) * u;
}

double RailLine::speedLimit(double st) const {
    if (vlim.empty()) return 1e9;
    const double w = wrap(st);
    const auto it = std::upper_bound(s.begin(), s.end(), w);
    std::size_t i = it == s.begin() ? 0 : static_cast<std::size_t>(it - s.begin()) - 1;
    i = std::min(i, vlim.size() - 1);
    return std::min(vlim[i], vlim[std::min(i + 1, vlim.size() - 1)]);
}

void RailSim::build(const std::vector<engine::RailTrack>& tracks, const TrainKind& kind, int trainsPerLine, double joinTol) {
    build(tracks, [&](const std::string&) { return kind; }, [&](double, const TrainKind&) { return trainsPerLine; }, joinTol);
}

void RailSim::build(const std::vector<engine::RailTrack>& tracks, const std::function<TrainKind(const std::string& cls)>& kindOf,
                    const std::function<int(double length, const TrainKind& kind)>& countOf, double joinTol) {
    lines_.clear();
    trains_.clear();
    const int n = static_cast<int>(tracks.size());
    // which track each one runs on into: its end within joinTol of the other's start, heading on the same way
    std::vector<int> succ(static_cast<std::size_t>(n), -1), pred(static_cast<std::size_t>(n), -1);
    for (int i = 0; i < n; ++i) {
        const auto& A = tracks[static_cast<std::size_t>(i)].pts;
        if (A.size() < 2) continue;
        const Vec3 endDir = flatDir(A[A.size() - 2], A.back());
        double best = joinTol;
        for (int j = 0; j < n; ++j) {
            const auto& B = tracks[static_cast<std::size_t>(j)].pts;
            if (j == i || B.size() < 2 || tracks[static_cast<std::size_t>(j)].cls != tracks[static_cast<std::size_t>(i)].cls) continue;   // (a tram never runs on into the main line)
            const double d = dist(A.back(), B.front());
            if (d >= best) continue;
            const Vec3 startDir = flatDir(B[0], B[1]);
            if (endDir.x * startDir.x + endDir.z * startDir.z < 0.7) continue;
            best = d;
            succ[static_cast<std::size_t>(i)] = j;
        }
    }
    for (int i = 0; i < n; ++i)
        if (succ[static_cast<std::size_t>(i)] >= 0) pred[static_cast<std::size_t>(succ[static_cast<std::size_t>(i)])] = i;
    std::vector<char> used(static_cast<std::size_t>(n), 0);
    auto chainFrom = [&](int start) {
        RailLine L;
        int t = start;
        while (t >= 0 && !used[static_cast<std::size_t>(t)]) {
            used[static_cast<std::size_t>(t)] = 1;
            const engine::RailTrack& T = tracks[static_cast<std::size_t>(t)];
            if (L.name.empty()) { L.name = T.edge; L.cls = T.cls; }
            std::size_t first = 0;
            if (!L.pts.empty() && dist(L.pts.back(), T.pts.front()) < 0.5) first = 1;
            const double base = L.pts.empty() ? 0.0 : L.s.back() + (first ? 0.0 : dist(L.pts.back(), T.pts.front()));
            for (std::size_t k = first; k < T.pts.size(); ++k) {
                L.s.push_back(L.pts.empty() ? 0.0 : L.s.back() + dist(L.pts.back(), T.pts[k]));
                L.pts.push_back(T.pts[k]);
            }
            for (engine::RailStop st : T.stops) {
                st.s0 += base;
                st.s1 += base;
                L.stops.push_back(st);
            }
            t = succ[static_cast<std::size_t>(t)];
            if (t == start) { L.loop = true; break; }
        }
        if (L.pts.size() < 2) return;
        if (L.loop) {   // close the ring: back to its first point
            if (dist(L.pts.back(), L.pts.front()) < 0.5) { L.pts.back() = L.pts.front(); }
            else { L.s.push_back(L.s.back() + dist(L.pts.back(), L.pts.front())); L.pts.push_back(L.pts.front()); }
        }
        L.length = L.s.back();
        std::sort(L.stops.begin(), L.stops.end(), [](const engine::RailStop& a, const engine::RailStop& b) { return a.s0 < b.s0; });
        // THE SPEED THE TRACK ALLOWS: the curve's (v^2 r <= lateral, measured over 10 m either side), then braked
        // for ahead of time -- a train meets a curve already slow enough for it
        const TrainKind kind = kindOf(L.cls);
        const std::size_t m = L.pts.size();
        L.vlim.assign(m, kind.vmax);
        for (std::size_t i = 0; i < m; ++i) {
            const std::size_t span = 5;
            std::size_t a = i >= span ? i - span : (L.loop ? i + m - 1 - span : 0);
            std::size_t c = i + span < m ? i + span : (L.loop ? i + span - (m - 1) : m - 1);
            if (a >= m) a = 0;
            if (c >= m) c = m - 1;
            const double k = curvature(L.pts[a], L.pts[i], L.pts[c]);
            if (k > 1e-6) L.vlim[i] = std::min(kind.vmax, std::sqrt(kind.lateral / k));
        }
        for (int pass = 0; pass < (L.loop ? 2 : 1); ++pass)
            for (std::size_t i = m - 1; i-- > 0;) {
                const double ds = L.s[i + 1] - L.s[i];
                const double next = (i + 1 == m - 1 && L.loop) ? L.vlim[0] : L.vlim[i + 1];
                L.vlim[i] = std::min(L.vlim[i], std::sqrt(next * next + 2.0 * kind.decel * ds));
            }
        lines_.push_back(std::move(L));
    };
    for (int i = 0; i < n; ++i)   // open lines first: the ones with a start
        if (!used[static_cast<std::size_t>(i)] && pred[static_cast<std::size_t>(i)] < 0 && tracks[static_cast<std::size_t>(i)].pts.size() >= 2) chainFrom(i);
    for (int i = 0; i < n; ++i)   // then the loops
        if (!used[static_cast<std::size_t>(i)] && tracks[static_cast<std::size_t>(i)].pts.size() >= 2) chainFrom(i);
    // THE TIMETABLE: evenly spread trains, each heading for the first platform ahead of it
    for (std::size_t li = 0; li < lines_.size(); ++li) {
        const RailLine& L = lines_[li];
        const TrainKind kind = kindOf(L.cls);
        const int count = L.length > kind.length() * 3 ? std::max(1, countOf(L.length, kind)) : 1;
        for (int k = 0; k < count; ++k) {
            Train t;
            t.line = static_cast<int>(li);
            t.kind = kind;
            t.s = L.loop ? L.wrap(L.length * k / count + kind.length()) : std::min(L.length, kind.length() + 5.0);
            t.prevS = t.s;
            t.nextStop = 0;
            for (std::size_t si = 0; si < L.stops.size(); ++si)
                if (L.stops[si].s1 - 2.0 > t.s) { t.nextStop = static_cast<int>(si); break; }
            trains_.push_back(t);
        }
    }
}

double RailSim::gapAhead(int ti) const {
    const Train& me = trains_[static_cast<std::size_t>(ti)];
    const RailLine& L = lines_[static_cast<std::size_t>(me.line)];
    double best = 1e9;
    for (std::size_t j = 0; j < trains_.size(); ++j) {
        if (static_cast<int>(j) == ti || trains_[j].line != me.line) continue;
        const double tail = trains_[j].s - trains_[j].kind.length();
        double d = tail - me.s;
        if (L.loop) d = L.wrap(d);
        if (d >= 0) best = std::min(best, d);
    }
    return best;
}

void RailSim::step(double dt) {
    constexpr double kDoorsAfter = 2.0, kDoorsShutBefore = 4.0, kStopShort = 2.0;
    for (std::size_t ti = 0; ti < trains_.size(); ++ti) {
        Train& t = trains_[ti];
        const RailLine& L = lines_[static_cast<std::size_t>(t.line)];
        t.prevS = t.s;
        if (t.dwelling) {
            t.dwellLeft -= dt;
            const bool open = t.dwellLeft > kDoorsShutBefore && t.dwellLeft < t.kind.dwell - kDoorsAfter;
            t.doorSide = open && !L.stops.empty() ? L.stops[static_cast<std::size_t>(t.nextStop)].side : 0;
            if (t.dwellLeft <= 0) {
                t.dwelling = false;
                t.doorSide = 0;
                if (!L.stops.empty()) t.nextStop = (t.nextStop + 1) % static_cast<int>(L.stops.size());
            }
            continue;
        }
        double target = std::min(t.kind.vmax, L.speedLimit(t.s));
        double toStop = 1e9;
        if (!L.stops.empty()) {
            const engine::RailStop& st = L.stops[static_cast<std::size_t>(t.nextStop)];
            toStop = st.s1 - kStopShort - t.s;
            if (L.loop) toStop = L.wrap(toStop);
            target = std::min(target, std::sqrt(2.0 * t.kind.decel * std::max(0.0, toStop)));
        }
        const double gap = gapAhead(static_cast<int>(ti));
        target = std::min(target, std::sqrt(2.0 * t.kind.decel * std::max(0.0, gap - 40.0)));
        if (t.v < target) t.v = std::min(target, t.v + t.kind.accel * dt);
        else t.v = std::max(target, t.v - 1.5 * t.kind.decel * dt);
        double ds = t.v * dt;
        if (!L.stops.empty() && ds >= toStop - 0.05 && t.v < 3.0) {   // at the platform: stop there exactly
            ds = std::max(0.0, toStop);
            t.v = 0;
            t.dwelling = true;
            t.dwellLeft = t.kind.dwell;
        }
        t.s += ds;
        t.odometer += ds;
        if (L.loop) t.s = L.wrap(t.s);
        else if (t.s >= L.length) { t.s = std::min(L.length, t.kind.length() + 5.0); t.prevS = t.s; t.nextStop = 0; }   // (open lines: shuttles owed)
    }
}

double RailSim::headAt(int ti, double alpha) const {
    const Train& t = trains_[static_cast<std::size_t>(ti)];
    const RailLine& L = lines_[static_cast<std::size_t>(t.line)];
    double ds = t.s - t.prevS;
    if (L.loop) {
        if (ds < -L.length / 2) ds += L.length;
        if (ds > L.length / 2) ds -= L.length;
    }
    return L.wrap(t.prevS + ds * std::clamp(alpha, 0.0, 1.0));
}

CarPose RailSim::carPose(int ti, int car, double alpha) const {
    const Train& t = trains_[static_cast<std::size_t>(ti)];
    const RailLine& L = lines_[static_cast<std::size_t>(t.line)];
    double front = headAt(ti, alpha);
    for (int k = 0; k < car; ++k) front -= (k == 0 ? t.kind.locoLength : t.kind.carLength) + t.kind.gap;
    const double len = car == 0 ? t.kind.locoLength : t.kind.carLength;
    // the body hangs between its two bogies, each on the rail
    const Vec3 p1 = L.at(front - 0.18 * len), p2 = L.at(front - 0.82 * len);
    Vec3 f = p1 - p2;
    const double fl = f.length();
    f = fl > 1e-9 ? f * (1.0 / fl) : Vec3(0, 0, 1);
    Vec3 left = engine::cross(Vec3(0, 1, 0), f);
    const double ll = left.length();
    left = ll > 1e-9 ? left * (1.0 / ll) : Vec3(1, 0, 0);
    const Vec3 up = engine::cross(f, left);
    const Vec3 c = (p1 + p2) * 0.5;
    CarPose out;
    out.length = len;
    out.loco = car == 0;
    Mat4& m = out.m;
    m.m[0][0] = left.x; m.m[1][0] = left.y; m.m[2][0] = left.z;
    m.m[0][1] = up.x;   m.m[1][1] = up.y;   m.m[2][1] = up.z;
    m.m[0][2] = f.x;    m.m[1][2] = f.y;    m.m[2][2] = f.z;
    m.m[0][3] = c.x;    m.m[1][3] = c.y;    m.m[2][3] = c.z;
    m.m[3][0] = m.m[3][1] = m.m[3][2] = 0; m.m[3][3] = 1;
    return out;
}

std::vector<CarPose> RailSim::carPoses(int ti, double alpha) const {
    std::vector<CarPose> out;
    for (int c = 0; c < carCount(ti); ++c) out.push_back(carPose(ti, c, alpha));
    return out;
}

engine::Vec3 lineColour(const RailLine& line) {
    if (line.cls == "rail") return engine::Vec3(0.24, 0.24, 0.27);
    std::string key;
    for (const engine::RailStop& st : line.stops) if (key.empty() || st.name < key) key = st.name;
    uint32_t h = 2166136261u;
    for (char c : key) h = (h ^ static_cast<uint8_t>(c)) * 16777619u;
    static const engine::Vec3 kPalette[] = {{0.78, 0.18, 0.55}, {0.0, 0.58, 0.60}, {0.93, 0.55, 0.05},
                                            {0.42, 0.30, 0.75}, {0.15, 0.62, 0.25}, {0.85, 0.15, 0.15}};
    return kPalette[h % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

std::string RailSim::describe() const {
    std::string o;
    char b[256];
    std::snprintf(b, sizeof(b), "%zu lines, %zu trains", lines_.size(), trains_.size());
    o += b;
    for (std::size_t li = 0; li < lines_.size(); ++li) {
        const RailLine& L = lines_[li];
        std::snprintf(b, sizeof(b), "; line %zu (%s) %.0f m%s, %zu stops:", li, L.name.c_str(), L.length, L.loop ? " loop" : "", L.stops.size());
        o += b;
        for (const engine::RailStop& st : L.stops) {
            std::snprintf(b, sizeof(b), " %s@%.0f-%.0f(%s)", st.name.c_str(), st.s0, st.s1, st.side > 0 ? "L" : "R");
            o += b;
        }
    }
    for (std::size_t ti = 0; ti < trains_.size(); ++ti) {
        const Train& t = trains_[ti];
        const Vec3 p = lines_[static_cast<std::size_t>(t.line)].at(t.s);
        std::snprintf(b, sizeof(b), "; train %zu line %d s %.0f (%.0f,%.1f,%.0f) %.1f m/s %s doors %s next %d",
                      ti, t.line, t.s, p.x, p.y, p.z, t.v, t.dwelling ? "dwelling" : "running",
                      t.doorSide > 0 ? "L" : t.doorSide < 0 ? "R" : "shut", t.nextStop);
        o += b;
    }
    return o;
}

}  // namespace citysim
