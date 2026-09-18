#include "city_dispatch.h"

#include <algorithm>

namespace citysim {

bool Dispatch::hail(int passenger, int pickup, int dropoff) {
    const Fare f{passenger, pickup, dropoff};
    if (!f.valid()) return false;
    if (driverOfPassenger_.count(passenger)) return false;   // already has a car
    // Sorted insert keeps `queue_` in ascending passenger order, which is what
    // makes assign()'s scan — and so the whole city — reproducible.
    const auto it = std::lower_bound(
        queue_.begin(), queue_.end(), passenger,
        [](const Fare& q, int p) { return q.passenger < p; });
    if (it != queue_.end() && it->passenger == passenger) return false;
    queue_.insert(it, f);
    return true;
}

void Dispatch::cancel(int passenger) {
    const auto it = std::lower_bound(
        queue_.begin(), queue_.end(), passenger,
        [](const Fare& q, int p) { return q.passenger < p; });
    if (it != queue_.end() && it->passenger == passenger) queue_.erase(it);
}

int Dispatch::assign(int driver, const std::function<double(int)>& cost) {
    if (driver < 0 || queue_.empty() || !cost) return -1;
    if (byDriver_.count(driver)) return -1;   // already carrying a fare
    std::size_t best = queue_.size();
    double bestCost = 0;
    for (std::size_t i = 0; i < queue_.size(); ++i) {
        const double c = cost(queue_[i].pickup);
        if (c < 0) continue;                  // unreachable from this driver
        // STRICTLY less: the first (lowest-index) passenger at a given cost
        // wins, because queue_ is in ascending passenger order.
        if (best == queue_.size() || c < bestCost) {
            best = i;
            bestCost = c;
        }
    }
    if (best == queue_.size()) return -1;
    const Fare f = queue_[best];
    queue_.erase(queue_.begin() + static_cast<std::ptrdiff_t>(best));
    byDriver_[driver] = f;
    driverOfPassenger_[f.passenger] = driver;
    return f.passenger;
}

const Fare* Dispatch::fareOf(int driver) const {
    const auto it = byDriver_.find(driver);
    return it == byDriver_.end() ? nullptr : &it->second;
}

int Dispatch::driverFor(int passenger) const {
    const auto it = driverOfPassenger_.find(passenger);
    return it == driverOfPassenger_.end() ? -1 : it->second;
}

void Dispatch::complete(int driver) {
    const auto it = byDriver_.find(driver);
    if (it == byDriver_.end()) return;
    driverOfPassenger_.erase(it->second.passenger);
    byDriver_.erase(it);
}

void Dispatch::clear() {
    queue_.clear();
    byDriver_.clear();
    driverOfPassenger_.clear();
}

std::vector<int> Dispatch::waitingPassengers() const {
    std::vector<int> out;
    out.reserve(queue_.size());
    for (const Fare& f : queue_) out.push_back(f.passenger);
    return out;   // queue_ is maintained in ascending passenger order
}

}  // namespace citysim
