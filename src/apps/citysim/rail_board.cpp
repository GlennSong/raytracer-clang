#include "rail_board.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Vec3;

namespace {
constexpr double kPlatformDepth = 4.8;   // edge to back: the widest platform class (4.5) and a margin
constexpr double kStopShort = 2.0;       // a train stops this short of the platform's end (rail_sim.cpp)
}  // namespace

bool platformAt(const RailSim& rs, const Vec3& body, int& line, int& stop) {
    double best = 1e9;
    line = stop = -1;
    for (std::size_t li = 0; li < rs.lines().size(); ++li) {
        const RailLine& L = rs.lines()[li];
        for (std::size_t k = 0; k < L.stops.size(); ++k) {
            const engine::RailStop& st = L.stops[k];
            if (std::fabs(body.y - st.top) > 2.5) continue;
            for (double s = st.s0; s <= st.s1; s += 2.0) {
                const Vec3 a = L.at(s), b = L.at(s + 1.0);
                double fx = b.x - a.x, fz = b.z - a.z;
                const double fl = std::hypot(fx, fz);
                if (fl < 1e-9) continue;
                fx /= fl; fz /= fl;
                // the platform's side: left = (fwd.z, -fwd.x) in plan, times `side` (city_rail_riders.cpp's frame)
                const double ox = fz * st.side, oz = -fx * st.side;
                const double dx = body.x - a.x, dz = body.z - a.z;
                const double along = dx * fx + dz * fz, lat = dx * ox + dz * oz;
                if (std::fabs(along) > 1.5 || lat < engine::kPlatformGap - 0.3 || lat > engine::kPlatformGap + kPlatformDepth) continue;
                const double d = std::fabs(body.y - st.top);
                if (d < best) { best = d; line = static_cast<int>(li); stop = static_cast<int>(k); }
            }
        }
    }
    return line >= 0;
}

double trainEta(const RailSim& rs, int ti, int k) {
    const Train& T = rs.trains()[static_cast<std::size_t>(ti)];
    const RailLine& L = rs.lines()[static_cast<std::size_t>(T.line)];
    const int n = static_cast<int>(L.stops.size());
    if (k < 0 || k >= n) return -1;
    if (T.dwelling && T.nextStop == k) return 0;
    double d = L.stops[static_cast<std::size_t>(k)].s1 - kStopShort - T.s;
    if (L.loop) d = L.wrap(d);
    else if (d < 0) return -1;
    // stops it calls at on the way (the one it stands at now is behind it once it leaves)
    const int from = T.dwelling ? (T.nextStop + 1) % n : T.nextStop;
    const int between = L.loop ? ((k - from) % n + n) % n : std::max(0, k - from);
    const double pace = std::min(25.0, 0.75 * T.kind.vmax);   // an average over the run: starting, curves, braking
    return d / pace + between * T.kind.dwell + (T.dwelling ? T.dwellLeft : 0.0);
}

PlatformBoard platformBoard(const RailSim& rs, int line, int stop, int perLine) {
    PlatformBoard b;
    if (line < 0 || line >= static_cast<int>(rs.lines().size())) return b;
    const RailLine& here = rs.lines()[static_cast<std::size_t>(line)];
    if (stop < 0 || stop >= static_cast<int>(here.stops.size())) return b;
    b.line = line; b.stop = stop;
    b.station = here.stops[static_cast<std::size_t>(stop)].name;
    std::vector<int> order{line};
    for (int li = 0; li < static_cast<int>(rs.lines().size()); ++li) if (li != line) order.push_back(li);
    for (int li : order) {
        const RailLine& L = rs.lines()[static_cast<std::size_t>(li)];
        int k = -1;
        for (std::size_t j = 0; j < L.stops.size(); ++j) if (L.stops[j].name == b.station) k = static_cast<int>(j);
        if (k < 0) continue;
        const int n = static_cast<int>(L.stops.size());
        const std::string toward = L.loop || k + 1 < n ? L.stops[static_cast<std::size_t>((k + 1) % n)].name : std::string("the end of the line");
        std::vector<PlatformBoard::Departure> ds;
        for (int ti = 0; ti < static_cast<int>(rs.trains().size()); ++ti) {
            if (rs.trains()[static_cast<std::size_t>(ti)].line != li) continue;
            const double eta = trainEta(rs, ti, k);
            if (eta < 0) continue;
            PlatformBoard::Departure d;
            d.line = li; d.train = ti; d.toward = toward; d.eta = eta; d.here = eta == 0; d.thisPlatform = li == line;
            ds.push_back(d);
        }
        std::sort(ds.begin(), ds.end(), [](const auto& a, const auto& c) { return a.eta < c.eta; });
        if (static_cast<int>(ds.size()) > perLine) ds.resize(static_cast<std::size_t>(perLine));
        b.departures.insert(b.departures.end(), ds.begin(), ds.end());
    }
    return b;
}

}  // namespace citysim
