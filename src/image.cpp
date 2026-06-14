#include "image.h"
#include <tinygltf/stb_image_write.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

Image::Image(int width, int height)
    : width(width), height(height), pixels(width * height) {}

void Image::setPixel(int x, int y, const Vec3& color) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        pixels[y * width + x] = color;
    }
}

Vec3 Image::getPixel(int x, int y) const {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        return pixels[y * width + x];
    }
    return {};
}

Image Image::bilateralFilter(int radius, double sigmaSpatial, double sigmaColor) const {
    Image result(width, height);
    double spatialDenom = -2.0 * sigmaSpatial * sigmaSpatial;
    double colorDenom = -2.0 * sigmaColor * sigmaColor;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 centerColor = getPixel(x, y);
            Vec3 weightedSum(0, 0, 0);
            double totalWeight = 0;

            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                    Vec3 neighborColor = getPixel(nx, ny);

                    double spatialDist2 = dx * dx + dy * dy;
                    double spatialWeight = std::exp(spatialDist2 / spatialDenom);

                    Vec3 colorDiff = neighborColor - centerColor;
                    double colorDist2 = colorDiff.lengthSquared();
                    double colorWeight = std::exp(colorDist2 / colorDenom);

                    double weight = spatialWeight * colorWeight;
                    weightedSum += neighborColor * weight;
                    totalWeight += weight;
                }
            }

            result.setPixel(x, y, weightedSum / totalWeight);
        }
    }

    return result;
}

void Image::writePpm(const std::string& filename) const {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 color = clampVec(pixels[y * width + x], 0.0, 1.0);
            int r = static_cast<int>(255.99 * color.x);
            int g = static_cast<int>(255.99 * color.y);
            int b = static_cast<int>(255.99 * color.z);
            file << r << " " << g << " " << b << "\n";
        }
    }
}

void Image::writePng(const std::string& filename) const {
    std::vector<unsigned char> rgb(static_cast<size_t>(width) * height * 3);
    for (int i = 0; i < width * height; i++) {
        Vec3 color = clampVec(pixels[i], 0.0, 1.0);
        rgb[i * 3 + 0] = static_cast<unsigned char>(255.99 * color.x);
        rgb[i * 3 + 1] = static_cast<unsigned char>(255.99 * color.y);
        rgb[i * 3 + 2] = static_cast<unsigned char>(255.99 * color.z);
    }
    stbi_write_png(filename.c_str(), width, height, 3, rgb.data(), width * 3);
}

}  // namespace engine

