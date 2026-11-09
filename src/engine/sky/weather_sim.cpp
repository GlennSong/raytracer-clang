#include "weather_sim.h"

#include "../../rt_math.h"   // smoothstep, bilinearGrid
#include <algorithm>
#include <cmath>

namespace engine {

namespace {
// THE SCALES (each the physical one it stands for, tuned against the satellite references: test_weather_sim)
constexpr double kWindRelax = 2400.0;      // s: the wind returns to the prevailing one (the free air above steering it)
constexpr double kBreeze = 1.2e-4;         // 1/s of convergence a unit of heating draws in (a sea breeze: ~2-4 m/s)
constexpr double kConvergenceLift = 1500.0;   // lift per 1/s of convergence (1e-4: 0.15)
constexpr double kSlopeLift = 0.5;         // lift per m/s the wind climbs (0.3 m/s up a slope: 0.15)
constexpr double kHeatLift = 0.6;          // lift per unit of heating (sunlit high ground: ~0.6)
constexpr double kSeaMoisten = 3.0 * 3600.0;   // s: the sea brings the air to its humidity
constexpr double kLandDry = 8.0 * 3600.0;      // s: ...and the land dries it toward its own (drier) share
constexpr double kLandShare = 0.88;        // the land's humidity, a share of the sea's
constexpr double kLiftMoisture = 0.30;     // how much a unit of lift brings the air nearer saturation (rising, it cools)
constexpr double kSaturation = 0.91;       // the humidity (with the lift's help) a cloud forms past
constexpr double kCondense = 600.0;        // s: a cloud forms...
constexpr double kEvaporate = 1500.0;      // ...and goes, slower: carried downwind meanwhile (the plumes)
constexpr int kPressureSweeps = 30;        // pressure sweeps a sub-step (warm-started; red-black SOR)
constexpr double kOverRelax = 1.9;         // the over-relaxation (1: plain Gauss-Seidel)
}  // namespace

void WeatherSim::init(const HeightRaster& terrain, double seaLevel, int n) {
    n_ = std::max(16, n);
    originX_ = terrain.originX;
    originZ_ = terrain.originZ;
    cell_ = terrain.extent / n_;
    const std::size_t count = static_cast<std::size_t>(n_) * n_;
    h_.assign(count, 0.0f);
    land_.assign(count, 0.0f);
    double lowest = 1e30;
    if (seaLevel < -1e29)
        for (float v : terrain.height) lowest = std::min(lowest, static_cast<double>(v));
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const double x = originX_ + (i + 0.5) * cell_, z = originZ_ + (j + 0.5) * cell_;
            const double g = terrain.empty() ? (seaLevel < -1e29 ? 0.0 : seaLevel - 1.0) : terrain.at(x, z);
            const double sea = seaLevel < -1e29 ? lowest : seaLevel;
            h_[at(i, j)] = static_cast<float>(std::max(0.0, g - sea));
            land_[at(i, j)] = seaLevel < -1e29 || g > seaLevel ? 1.0f : 0.0f;
        }
    u_.assign(count, 0.0f);
    v_.assign(count, 0.0f);
    q_.assign(count, 0.8f);
    c_.assign(count, 0.0f);
    lift_.assign(count, 0.0f);
    conv_.assign(count, 0.0f);
    p_.assign(count, 0.0f);
    div_.assign(count, 0.0f);
    tmp_.assign(count, 0.0f);
    tmp2_.assign(count, 0.0f);
}

float WeatherSim::sample(const std::vector<float>& f, double x, double z) const {
    if (n_ <= 0 || f.empty()) return 0.0f;
    // (the values sit at the cells' middles)
    return static_cast<float>(bilinearGrid(n_, n_, (x - originX_) / cell_ - 0.5, (z - originZ_) / cell_ - 0.5,
                                           [&](int i, int j) { return static_cast<Real>(f[at(i, j)]); }));
}

// Semi-Lagrangian: each cell takes what was upwind of it `dt` ago (bilinear; the grid's edge clamps)
void WeatherSim::advect(std::vector<float>& f, const std::vector<float>& u, const std::vector<float>& v, double dt,
                        std::vector<float>& tmp) const {
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            tmp[at(i, j)] = static_cast<float>(bilinearGrid(n_, n_, i - u[at(i, j)] * dt / cell_, j - v[at(i, j)] * dt / cell_,
                                                            [&](int a, int b) { return static_cast<Real>(f[at(a, b)]); }));
        }
    f.swap(tmp);
}

void WeatherSim::step(const WeatherSimInputs& in) {
    if (n_ <= 0 || in.dt <= 0.0) return;
    // sub-steps the wind crosses at most a cell in (and no longer than two minutes: the relaxations' resolution)
    const double fastest = std::max(1.0, static_cast<double>(in.windSpeed) * 1.5 + 4.0);
    const double most = std::min(120.0, 0.9 * cell_ / fastest);
    const int steps = std::max(1, static_cast<int>(std::ceil(in.dt / most)));
    for (int s = 0; s < steps; ++s) subStep(in, in.dt / steps);
}

void WeatherSim::settle(const WeatherSimInputs& in, double seconds) {
    WeatherSimInputs s = in;
    s.dt = seconds;
    step(s);
}

void WeatherSim::subStep(const WeatherSimInputs& in, double dt) {
    const WeatherSimWeather& w = in.weather;
    const double U0 = in.windX * in.windSpeed, V0 = in.windZ * in.windSpeed;
    const double day = std::max(0.0, static_cast<double>(in.sunUp));
    const double night = std::clamp(-static_cast<double>(in.sunUp) * 3.0, 0.0, 1.0);
    const double inv = w.inversion;

    // THE HEATING: sunlit land, more on high ground; by night the land cools under the sea's warmth
    double meanHeat = 0.0;
    for (std::size_t k = 0; k < h_.size(); ++k) {
        const double high = std::clamp(h_[k] / 1200.0, 0.0, 1.0);
        const double heat = w.convection * (land_[k] * (day * (0.35 + 0.65 * high) - 0.3 * night) + (1.0 - land_[k]) * 0.15 * night);
        tmp2_[k] = static_cast<float>(heat);
        meanHeat += heat;
    }
    meanHeat /= static_cast<double>(h_.size());

    // THE WIND: back toward the prevailing one; carried by itself; the far edge is the free air
    const double relax = std::min(1.0, dt / kWindRelax);
    for (std::size_t k = 0; k < u_.size(); ++k) {
        u_[k] += static_cast<float>((U0 - u_[k]) * relax);
        v_[k] += static_cast<float>((V0 - v_[k]) * relax);
    }
    uOld_ = u_;   // (the member's room, kept: no allocation a sub-step)
    advect(u_, uOld_, v_, dt, tmp_);
    advect(v_, uOld_, v_, dt, tmp_);
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i)
            if (i < 2 || j < 2 || i >= n_ - 2 || j >= n_ - 2) {
                u_[at(i, j)] = static_cast<float>(U0);
                v_[at(i, j)] = static_cast<float>(V0);
            }
    // ...made to converge where the heating asks (the projection: grad^2 p = div u - the asked divergence)
    const double inv2c = 1.0 / (2.0 * cell_);
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const int il = std::max(i - 1, 0), ir = std::min(i + 1, n_ - 1), jd = std::max(j - 1, 0), ju = std::min(j + 1, n_ - 1);
            const double d = (u_[at(ir, j)] - u_[at(il, j)]) * inv2c + (v_[at(i, ju)] - v_[at(i, jd)]) * inv2c;
            div_[at(i, j)] = static_cast<float>(d + kBreeze * (tmp2_[at(i, j)] - meanHeat));
        }
    const double c2 = cell_ * cell_;
    // red-black over-relaxed Gauss-Seidel, in place: 30 sweeps of it sit within 0.01 of the converged cover, where 30 of
    // Jacobi's (the first solve) were up to 0.3 off over the island -- a long-range pressure that had not spread yet
    for (int it = 0; it < kPressureSweeps; ++it)
        for (int colour = 0; colour < 2; ++colour)
            for (int j = 0; j < n_; ++j)
                for (int i = (j + colour) & 1; i < n_; i += 2) {
                    const int il = std::max(i - 1, 0), ir = std::min(i + 1, n_ - 1), jd = std::max(j - 1, 0), ju = std::min(j + 1, n_ - 1);
                    const double gs = (p_[at(il, j)] + p_[at(ir, j)] + p_[at(i, jd)] + p_[at(i, ju)] - div_[at(i, j)] * c2) * 0.25;
                    p_[at(i, j)] = static_cast<float>(p_[at(i, j)] + kOverRelax * (gs - p_[at(i, j)]));
                }
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const int il = std::max(i - 1, 0), ir = std::min(i + 1, n_ - 1), jd = std::max(j - 1, 0), ju = std::min(j + 1, n_ - 1);
            u_[at(i, j)] -= static_cast<float>((p_[at(ir, j)] - p_[at(il, j)]) * inv2c);
            v_[at(i, j)] -= static_cast<float>((p_[at(i, ju)] - p_[at(i, jd)]) * inv2c);
            // ground standing above the inversion is a wall: the air goes round it
            const double wall = smoothstep(inv - 250.0, inv + 250.0, h_[at(i, j)]);
            u_[at(i, j)] *= static_cast<float>(1.0 - wall);
            v_[at(i, j)] *= static_cast<float>(1.0 - wall);
        }

    // THE LIFT: the convergence, the wind up the slope (down it: sinking), the heating, the weather's own
    for (int j = 0; j < n_; ++j)
        for (int i = 0; i < n_; ++i) {
            const int il = std::max(i - 1, 0), ir = std::min(i + 1, n_ - 1), jd = std::max(j - 1, 0), ju = std::min(j + 1, n_ - 1);
            const std::size_t k = at(i, j);
            const double d = (u_[at(ir, j)] - u_[at(il, j)]) * inv2c + (v_[at(i, ju)] - v_[at(i, jd)]) * inv2c;
            const double dhx = (h_[at(ir, j)] - h_[at(il, j)]) * inv2c, dhz = (h_[at(i, ju)] - h_[at(i, jd)]) * inv2c;
            const double conv = -d * kConvergenceLift;
            const double slope = (u_[k] * dhx + v_[k] * dhz) * kSlopeLift;
            const double heat = std::max(0.0, static_cast<double>(tmp2_[k])) * kHeatLift;
            // (the outer ring is the free air the grid opens on: pinned to the prevailing wind, its convergence is the
            // pinning's, not the weather's -- it lifted false cloud along the grid's edges)
            const bool ring = i < 4 || j < 4 || i >= n_ - 4 || j >= n_ - 4;
            lift_[k] = static_cast<float>(std::clamp((ring ? 0.0 : conv + slope) + heat + w.lift, -1.0, 2.0));
            conv_[k] = static_cast<float>(std::clamp(heat * 1.4 + std::max(0.0, conv), 0.0, 1.0));
        }

    // THE MOISTURE: carried; the sea moistens the air toward its humidity, the land dries it toward a share of it
    advect(q_, u_, v_, dt, tmp_);
    for (std::size_t k = 0; k < q_.size(); ++k) {
        const double target = land_[k] > 0.5f ? w.humidity * kLandShare : w.humidity;
        const double tau = land_[k] > 0.5f ? kLandDry : kSeaMoisten;
        q_[k] += static_cast<float>((target - q_[k]) * std::min(1.0, dt / tau));
    }

    // THE CLOUD: where the lifted air saturates, and none where the ground stands above the inversion; carried
    for (std::size_t k = 0; k < c_.size(); ++k) {
        const double sat = q_[k] + kLiftMoisture * lift_[k] - kSaturation;
        const double above = smoothstep(inv - 250.0, inv + 250.0, h_[k]);
        const double want = smoothstep(0.0, 0.12, sat) * (1.0 - above);
        const double tau = want > c_[k] ? kCondense : kEvaporate;
        c_[k] += static_cast<float>((want - c_[k]) * std::min(1.0, dt / tau));
    }
    advect(c_, u_, v_, dt, tmp_);
}

}  // namespace engine
