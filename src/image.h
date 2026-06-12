#ifndef RAYTRACER_IMAGE_H
#define RAYTRACER_IMAGE_H

#include "rt_math.h"
#include <vector>
#include <string>

namespace engine {

class Image {
public:
    int width;
    int height;
    std::vector<Vec3> pixels;

    Image(int width, int height);

    void setPixel(int x, int y, const Vec3& color);
    Vec3 getPixel(int x, int y) const;
    void writePpm(const std::string& filename) const;

    Image bilateralFilter(int radius, double sigmaSpatial, double sigmaColor) const;
};


}  // namespace engine

#endif
