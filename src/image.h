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
    // PNG via the vendored stb_image_write (ADR-0016) — smaller and directly
    // viewable, unlike P3 PPM. The implementation TU lives in
    // model_importer.cpp; here we only call the declaration.
    void writePng(const std::string& filename) const;

    Image bilateralFilter(int radius, double sigmaSpatial, double sigmaColor) const;
};


}  // namespace engine

#endif
