#include "ground_grid.h"

#include <algorithm>
#include <cmath>

namespace engine::roads {

double GroundGrid::sample(double x, double y) const {
    double fx = (x - x0) / res, fy = (y - y0) / res;
    int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
    i = std::max(0, std::min(nx - 2, i)); j = std::max(0, std::min(ny - 2, j));
    double tx = std::max(0.0, std::min(1.0, fx - i)), ty = std::max(0.0, std::min(1.0, fy - j));
    return (at(i, j) * (1 - tx) + at(i + 1, j) * tx) * (1 - ty) +
           (at(i, j + 1) * (1 - tx) + at(i + 1, j + 1) * tx) * ty;
}

}  // namespace engine::roads
