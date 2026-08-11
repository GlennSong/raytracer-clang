#ifndef ROADLAB_RASTER_H
#define ROADLAB_RASTER_H

// A small z-buffer rasteriser whose only real job is to prove the shading model.
// It interpolates (s, t) perspective-correctly and hands each fragment to
// surface.h, exactly the way a GPU would — including deriving the fragment's
// footprint in metres from the screen-space gradients, which is what the
// analytic antialiasing needs.
//
// It exists because the interesting claim in this prototype ("markings are a
// shader, not geometry") is only believable if you can look at it, and this
// container has no window.

#include "surface.h"
#include "tessellate.h"
#include <string>
#include <vector>

namespace roadlab {

struct Camera {
    Vec3 eye{0, 200, 0};
    Vec3 target{0, 0, 0};
    Vec3 up{0, 1, 0};
    double fovYDeg = 45.0;
    bool ortho = false;
    double orthoHeight = 200.0;   // world metres covered vertically
    double zNear = 0.25;
    double zFar = 40000.0;
};

// Straight down, framed on a plan-space box. The workhorse view for reading a
// road layout.
Camera cameraTopDown(Vec2 lo, Vec2 hi, double aspect, double margin = 8.0);
Camera cameraLook(Vec3 eye, Vec3 target, double fovYDeg = 45.0);
// An eye-level view down a lane: the only way to judge whether the markings and
// the wear actually read at a driver's angle.
Camera cameraDriver(const Road& road, double s, int laneId, double eyeHeight = 1.5,
                    double lookAhead = 45.0);

struct PaintSet {
    std::vector<RoadPaint> roads;                 // indexed by road id
    std::vector<std::vector<Decal>> junctions;    // indexed by junction id
};

struct RenderOptions {
    int width = 1600;
    int height = 1000;
    ShadeParams shade;
    Vec3 sunDir{-0.42, 0.78, 0.46};
    Vec3 sunColor{1.15, 1.08, 0.98};
    Vec3 skyColor{0.30, 0.40, 0.58};
    Vec3 horizonColor{0.62, 0.68, 0.76};
    double exposure = 1.0;
    bool microNormals = true;
    int threads = 0;   // 0 = hardware concurrency
};

struct Framebuffer {
    int width = 0, height = 0;
    std::vector<Vec3> color;
    std::vector<float> depth;
    void resize(int w, int h);
};

void renderScene(const Mesh& mesh, const Network& net, const PaintSet& paint, const Camera& cam,
                 const RenderOptions& opt, Framebuffer& fb);

bool writePng(const Framebuffer& fb, const std::string& path, double exposure = 1.0);

}  // namespace roadlab

#endif
