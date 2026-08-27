#include "rt_math.h"
#include "image.h"
#include "camera.h"
#include "scene.h"
#include "level_scene.h"
#include "path_tracer.h"
#include "engine/model_importer.h"   // EnvironmentLoader: HDR decode + sun extraction
#include "job_system.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <algorithm>
#ifdef RT_ENABLE_SCRIPTING
#include "engine/scripting/script_vm.h"
#include "engine/scripting/procgen_bindings.h"
#include "engine/procgen/tree.h"   // TextureData
#endif

using namespace engine;  // namespace migration (ADR-0015)

// Write the integer percentage to a sidecar a host (the editor) polls for a
// progress bar. Write-to-temp then rename so a reader never sees a half-written
// value (rename is atomic on POSIX). Best-effort: a failed write is silent —
// progress reporting must never abort a render.
namespace {
void writeProgressFile(const std::string& path, int pct) {
    if (path.empty()) return;
    const std::string tmp = path + ".tmp";
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) return;
    f << pct << "\n";
    f.close();
    std::rename(tmp.c_str(), path.c_str());
}
}  // namespace

int IMAGE_SIZE = 512;          // --size
int SAMPLES_PER_PIXEL = 128;   // --spp
double EXPOSURE = 1.0;         // --exposure (linear scale before gamma)

Scene buildCornellBox() {
    Scene scene;

    int white  = scene.addMaterial(Material::diffuse(Vec3(0.73, 0.73, 0.73)));
    int red    = scene.addMaterial(Material::diffuse(Vec3(0.65, 0.05, 0.05)));
    int green  = scene.addMaterial(Material::diffuse(Vec3(0.12, 0.45, 0.15)));
    int light  = scene.addMaterial(Material::emissive(Vec3(1.0, 0.9, 0.7), 15.0));
    int mirror = scene.addMaterial(Material::metal(Vec3(0.95, 0.95, 0.95), 0.0));
    int glass  = scene.addMaterial(Material::glass(1.5));

    // Floor
    scene.addQuad(Vec3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555), white);
    // Ceiling
    scene.addQuad(Vec3(0, 555, 555), Vec3(555, 0, 0), Vec3(0, 0, -555), white);
    // Back wall
    scene.addQuad(Vec3(0, 0, 555), Vec3(555, 0, 0), Vec3(0, 555, 0), white);
    // Left wall (red)
    scene.addQuad(Vec3(0, 0, 0), Vec3(0, 0, 555), Vec3(0, 555, 0), red);
    // Right wall (green)
    scene.addQuad(Vec3(555, 0, 555), Vec3(0, 0, -555), Vec3(0, 555, 0), green);

    // Ceiling light
    scene.addQuad(Vec3(213, 554, 227), Vec3(130, 0, 0), Vec3(0, 0, 105), light);

    // Glass sphere (left side)
    scene.addSphere(Vec3(185, 100, 190), 100, glass);

    // Mirror sphere mesh (right side, tessellated — exercises the KD-tree)
    scene.addMeshSphere(Vec3(370, 100, 350), 100, mirror, 32, 64);

    scene.buildAccelerator();
    return scene;
}

// CLI (docs/virtual-camera-plan.md, Phases 3 + 5). Two modes:
//   Cornell box (default): lens flags apply directly. The box is ~555 units
//   across, so --scale 100 maps meter-scale optics onto it; try:
//     ./raytracer --fstop 1.0 --focus 800 --scale 100
//   Level mode: renders a viewer level through a camera placed interactively
//   (the sidecar written on viewer quit / by the Cameras panel):
//     ./raytracer --level assets/levels/arena.json --camera "Camera 1"
struct CliOptions {
    std::string level;        // empty = Cornell box
    std::string cameraName;   // empty = first camera in the sidecar
    std::string outFile = "output.png";
    std::string bakeTexture;           // a Lua texture recipe -> bake to outFile, no render
    std::string progressFile;          // empty = no sidecar (CLI default)
    double worldUnitsPerMeter = 0.0;   // 0 = mode default (1 level, 100 Cornell)
    LensParams lens;
    bool showHelp = false;
    // The hour (--hour): light from the day/night cycle instead of the
    // authored sun. -1 = the level as authored.
    double hour = -1.0;
    int    day = -1;             // day of year; -1 = the cycle's default (172)
    double latitude = 1e9;       // degrees N; 1e9 = default (40)
    double moon = -1.0;          // moon age in days; -1 = follow the calendar
    double pollution = 0.0;      // light pollution 0..1 (a city sky glow)
    bool   exposureGiven = false;
    bool   hasEye = false, hasLook = false;
    Vec3   eye, look;
};

CliOptions parseCli(int argc, char** argv) {
    CliOptions opt;
    opt.lens.fStop = 0.0;  // pinhole unless --fstop is given
    opt.lens.focusDistance = 800.0;
    auto next = [&](int& i) { return std::atof(argv[++i]); };
    for (int i = 1; i < argc; i++) {
        std::string flag = argv[i];
        bool hasValue = i < argc - 1;
        if (flag == "--help" || flag == "-h")     opt.showHelp = true;
        else if (!hasValue)                       continue;
        else if (flag == "--level")    opt.level = argv[++i];
        else if (flag == "--camera")   opt.cameraName = argv[++i];
        else if (flag == "--out")      opt.outFile = argv[++i];
        else if (flag == "--bake-texture") opt.bakeTexture = argv[++i];
        else if (flag == "--progress") opt.progressFile = argv[++i];
        else if (flag == "--size")     IMAGE_SIZE = std::max(16, atoi(argv[++i]));
        else if (flag == "--spp")      SAMPLES_PER_PIXEL = std::max(1, atoi(argv[++i]));
        else if (flag == "--exposure") { EXPOSURE = next(i); opt.exposureGiven = true; }
        else if (flag == "--hour")     opt.hour = next(i);
        else if (flag == "--day")      opt.day = atoi(argv[++i]);
        else if (flag == "--latitude") opt.latitude = next(i);
        else if (flag == "--moon")     { std::string m = argv[++i]; opt.moon = (m == "auto") ? -1.0 : std::atof(m.c_str()); }
        else if (flag == "--pollution") opt.pollution = next(i);
        // Three reads, sequenced: argument evaluation order is unspecified,
        // so Vec3(next(i), next(i), next(i)) lands the components permuted.
        else if (flag == "--eye")      { double x = next(i); double y = next(i); double z = next(i); opt.eye = Vec3(x, y, z); opt.hasEye = true; }
        else if (flag == "--look")     { double x = next(i); double y = next(i); double z = next(i); opt.look = Vec3(x, y, z); opt.hasLook = true; }
        else if (flag == "--fstop")    opt.lens.fStop = next(i);
        else if (flag == "--focal")    opt.lens.focalLength = next(i);
        else if (flag == "--focus")    opt.lens.focusDistance = next(i);
        else if (flag == "--k1")       opt.lens.distortionK1 = next(i);
        else if (flag == "--k2")       opt.lens.distortionK2 = next(i);
        else if (flag == "--ca")       opt.lens.chromaticAberration = next(i);
        else if (flag == "--vignette") opt.lens.vignette = next(i);
        else if (flag == "--scale")    opt.worldUnitsPerMeter = next(i);
    }
    return opt;
}

void printUsage() {
    std::cerr <<
        "Usage: raytracer [options]\n"
        "  --level <file>    render a viewer level instead of the Cornell box\n"
        "  --camera <name>   placed camera from <level>.cameras.json (default: first)\n"
        "  --out <file>      output image; .ppm writes P3, anything else PNG\n"
        "                    (default output.png)\n"
        "  --progress <file> write render %% (0-100) to <file> for a host to poll\n"
        "  --size N          image size (default 512)\n"
        "  --spp N           samples per pixel (default 128)\n"
        "  --hour H          light the level from the day/night cycle at hour H (0-24):\n"
        "                    the sun (or the moon at night), the dusk/night sky, stars,\n"
        "                    the moon disc; --day N (1-365), --latitude L, --moon AGE|auto,\n"
        "                    --pollution P (0-1 city sky glow). Night exposure adapts x6\n"
        "                    unless --exposure is given.\n"
        "  --eye X Y Z       camera position (skips the level's camera sidecar);\n"
        "  --look X Y Z      point the camera at (default: -z from the eye)\n"
        "  --exposure E      linear brightness scale (default 1; HDR-lit levels\n"
        "                    read dark offline — try 2-4)\n"
        "Lens (Cornell mode; level cameras carry their own lens):\n"
        "  --fstop N --focal MM --focus D --k1 K --k2 K --ca C --vignette V\n"
        "  --scale U         world units per meter (default: 1 level, 100 Cornell)\n";
}

int main(int argc, char** argv) {
    CliOptions opt = parseCli(argc, argv);
    if (opt.showHelp) {
        printUsage();
        return 0;
    }

    // Texture-bake mode (ADR-0043): run a Lua texture recipe and write the baked
    // image, no scene render — so a procedural texture composed from primitives
    // can be previewed/iterated. Uses the same engine bindings the scene path does.
#ifdef RT_ENABLE_SCRIPTING
    if (!opt.bakeTexture.empty()) {
        std::ifstream f(opt.bakeTexture);
        if (!f) { std::cerr << "Cannot open recipe: " << opt.bakeTexture << "\n"; return 1; }
        std::stringstream ss; ss << f.rdbuf();
        ScriptVM vm;
        openProcgenLibrary(vm);
        TextureData tex;
        std::string err;
        if (!runProcgenTexture(vm, ss.str(), tex, &err)) {
            std::cerr << "texture recipe error: " << err << "\n";
            return 1;
        }
        Image img(tex.width, tex.height);
        for (int y = 0; y < tex.height; ++y)
            for (int x = 0; x < tex.width; ++x) {
                std::size_t i = (static_cast<std::size_t>(y) * tex.width + x) * tex.channels;
                img.setPixel(x, y, Vec3(tex.pixels[i] / 255.0, tex.pixels[i + 1] / 255.0,
                                        tex.pixels[i + 2] / 255.0));
            }
        img.writePng(opt.outFile);
        std::cout << "Baked " << tex.width << "x" << tex.height << " texture -> "
                  << opt.outFile << "\n";
        return 0;
    }
#endif

    Scene scene;
    Vec3 lookFrom(278, 278, -800), lookAt(278, 278, 0);
    double fovDegrees = 40.0;
    LensParams lens = opt.lens;
    double scale = opt.worldUnitsPerMeter;

    if (!opt.level.empty()) {
        std::string hdrPath;
        if (!LevelScene::load(opt.level, scene, &hdrPath)) return 1;

        // Parallel the viewer's environment path (ADR-0016/0017): the HDR is
        // the sky, and its dominant light becomes the shadow-casting sun. The
        // disc is patched out of the map afterwards so the explicit sun isn't
        // counted twice.
        if (!hdrPath.empty()) {
            HdrImage hdrImage = EnvironmentLoader::loadHdr(hdrPath);
            if (hdrImage.valid()) {
                scene.environment.map.width = hdrImage.width;
                scene.environment.map.height = hdrImage.height;
                scene.environment.map.pixels = std::move(hdrImage.pixels);
                if (hdrImage.hasSun) {
                    SceneLight sun;
                    sun.type = SceneLight::Type::Directional;
                    sun.direction = hdrImage.sunDirection;
                    sun.color = hdrImage.sunColor;
                    sun.intensity = hdrImage.sunIntensity;
                    scene.lights.push_back(sun);
                    scene.environment.map.suppressSunDisc();
                }
            }
            // A declared HDR suppresses the importer's default sun (the map
            // is expected to light the scene) — if the file failed to load,
            // fall back so the level isn't left in the dark. A valid map with
            // no sun peak is simply overcast: the sky alone is correct.
            if (!scene.environment.map.valid()) {
                std::cerr << "HDR missing/unreadable (" << hdrPath
                          << "); adding the default noon sun\n";
                SceneLight noon;
                noon.direction = Vec3(0.35, 0.8, 0.25);
                noon.color = Vec3(1.0, 0.95, 0.85);
                noon.intensity = 4.0;
                scene.lights.push_back(noon);
            }
        }

        // THE HOUR: the cycle replaces the authored sun and the sky.
        if (opt.hour >= 0.0) {
            DayNightCycle cycle;
            cycle.timeOfDay = std::fmod(opt.hour, 24.0) / 24.0;
            if (opt.day > 0) cycle.dayOfYear = opt.day;
            if (opt.latitude < 1e8) cycle.latitudeDeg = opt.latitude;
            if (opt.moon >= 0.0) cycle.moonLock = opt.moon;
            const DayNightState st = applyDayNight(scene, cycle, std::max(opt.pollution, 0.0));
            if (!opt.exposureGiven) {
                // The viewer's night adaptation (DayNightSystem::kNightAdapt).
                double f = (st.solarElevation + 0.10) / 0.30;
                f = std::max(0.0, std::min(1.0, f));
                const double ease = f * f * (3.0 - 2.0 * f);
                EXPOSURE *= 1.0 + 5.0 * (1.0 - ease);
            }
        }

        if (opt.hasEye) {
            lookFrom = opt.eye;
            lookAt = opt.hasLook ? opt.look : opt.eye + Vec3(0, 0, -1);
            fovDegrees = lens.verticalFovDegrees();
            if (scale <= 0.0) scale = 1.0;
            std::cerr << "Rendering from --eye in " << opt.level << "\n";
        } else {
            SidecarCamera cam;
            if (!loadSidecarCamera(opt.level, opt.cameraName, cam)) return 1;
            lookFrom = cam.position;
            lookAt = cam.position + cam.forward;
            lens = cam.lens;                       // the placed camera's lens wins
            fovDegrees = lens.verticalFovDegrees();
            if (scale <= 0.0) scale = 1.0;         // levels are meter-scale
            std::cerr << "Rendering \"" << cam.name << "\" in " << opt.level << "\n";
        }
    } else {
        std::cerr << "Building scene...\n";
        scene = buildCornellBox();
        if (scale <= 0.0) scale = 100.0;       // Cornell is ~555 units across
    }

    Camera camera(lookFrom, lookAt, Vec3(0, 1, 0), fovDegrees, 1.0, lens, scale);
    if (lens.fStop > 0.0)
        std::cerr << "Lens: f/" << lens.fStop << " @ " << lens.focalLength
                  << "mm, focus " << lens.focusDistance << "\n";

    std::cerr << "  Triangles: " << scene.triangles.size()
              << ", quads: " << scene.quads.size()
              << ", spheres: " << scene.spheres.size() << "\n";

    JobSystem jobs;  // one shared pool owns the threads; rows are the tasks
    std::cerr << "Rendering with " << jobs.workerCount() << " worker threads, "
              << SAMPLES_PER_PIXEL << " spp...\n";

    RenderConfig config{IMAGE_SIZE, SAMPLES_PER_PIXEL, EXPOSURE};
    writeProgressFile(opt.progressFile, 0);   // reset any stale sidecar up front
    Image image = renderImage(scene, camera, config, jobs,
                              [&opt](float frac) {
        int pct = static_cast<int>(frac * 100);
        std::cerr << "\r  Progress: " << pct << "%   " << std::flush;
        writeProgressFile(opt.progressFile, pct);
    });
    std::cerr << "\r  Progress: 100%                \n";
    std::cerr << "Done rendering.\n";

    std::cerr << "Applying bilateral filter...\n";
    Image denoised = image.bilateralFilter(5, 3.0, 0.1);
    // Format by extension: explicit .ppm keeps the legacy P3 writer; everything
    // else (the default) is PNG via stb_image_write — smaller and viewable.
    auto hasSuffix = [](const std::string& s, const std::string& suf) {
        return s.size() >= suf.size() &&
               s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
    };
    if (hasSuffix(opt.outFile, ".ppm"))
        denoised.writePpm(opt.outFile);
    else
        denoised.writePng(opt.outFile);
    std::cout << "Wrote " << opt.outFile << " (denoised, " << IMAGE_SIZE << "x"
              << IMAGE_SIZE << ", " << SAMPLES_PER_PIXEL << " spp)\n";
    return 0;
}
