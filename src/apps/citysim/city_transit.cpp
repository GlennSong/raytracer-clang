#include "city_transit.h"

#include <algorithm>

namespace citysim {

bool RideBook::board(int passenger, int driver) {
    if (passenger < 0 || driver < 0 || passenger == driver) return false;
    if (byPassenger_.count(passenger)) return false;   // already aboard something
    if (byPassenger_.count(driver)) return false;      // a rider cannot drive
    byPassenger_[passenger] = driver;
    ++loadByDriver_[driver];
    return true;
}

void RideBook::alight(int passenger) {
    auto it = byPassenger_.find(passenger);
    if (it == byPassenger_.end()) return;
    auto ld = loadByDriver_.find(it->second);
    if (ld != loadByDriver_.end() && --ld->second <= 0) loadByDriver_.erase(ld);
    byPassenger_.erase(it);
}

void RideBook::alightAll(int driver) {
    for (auto it = byPassenger_.begin(); it != byPassenger_.end();)
        it = (it->second == driver) ? byPassenger_.erase(it) : std::next(it);
    loadByDriver_.erase(driver);
}

int RideBook::driverOf(int passenger) const {
    auto it = byPassenger_.find(passenger);
    return it == byPassenger_.end() ? -1 : it->second;
}

int RideBook::load(int driver) const {
    auto it = loadByDriver_.find(driver);
    return it == loadByDriver_.end() ? 0 : it->second;
}

void RideBook::clear() {
    byPassenger_.clear();
    loadByDriver_.clear();
}

std::vector<std::pair<int, int>> RideBook::rides() const {
    std::vector<std::pair<int, int>> out(byPassenger_.begin(), byPassenger_.end());
    std::sort(out.begin(), out.end());   // ascending passenger: deterministic
    return out;
}

}  // namespace citysim
