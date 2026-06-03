#include "math.h"
#include "renderer/renderer.h"
#include "renderer/window.h"
#include "renderer/orbit_camera.h"
#include "renderer/settings.h"
#include "engine/clock.h"
#include "engine/world.h"
#include <iostream>
#include <vector>

const std::string SETTINGS_FILE = "settings.json";

RenderMesh createQuadMesh(Vec3 corner, Vec3 edge1, Vec3 edge2) {
    RenderMesh mesh;
    Vec3 normal = -normalize(cross(edge1, edge2));

    mesh.vertices = {
        Vertex(corner, normal),
        Vertex(corner + edge1, normal),
        Vertex(corner + edge1 + edge2, normal),
        Vertex(corner + edge2, normal)
    };
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

RenderMesh createSphereMesh(float radius, int stacks, int slices) {
    RenderMesh mesh;
    for (int i = 0; i <= stacks; i++) {
        float theta = PI * i / stacks;
        for (int j = 0; j <= slices; j++) {
            float phi = 2.0 * PI * j / slices;
            float x = radius * std::sin(theta) * std::cos(phi);
            float y = radius * std::cos(theta);
            float z = radius * std::sin(theta) * std::sin(phi);
            Vec3 pos(x, y, z);
            Vec3 norm = normalize(pos);
            float u = static_cast<float>(j) / slices;
            float v = static_cast<float>(i) / stacks;
            mesh.vertices.push_back(Vertex(pos, norm, u, v));
        }
    }

    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < slices; j++) {
            int a = i * (slices + 1) + j;
            int b = a + slices + 1;
            mesh.indices.push_back(a);
            mesh.indices.push_back(b);
            mesh.indices.push_back(a + 1);
            mesh.indices.push_back(b);
            mesh.indices.push_back(b + 1);
            mesh.indices.push_back(a + 1);
        }
    }
    return mesh;
}

RenderMesh createBoxMesh(Vec3 size) {
    RenderMesh mesh;
    float hx = static_cast<float>(size.x * 0.5);
    float hy = static_cast<float>(size.y * 0.5);
    float hz = static_cast<float>(size.z * 0.5);

    struct Face { Vec3 normal; Vec3 verts[4]; };
    Face faces[] = {
        {Vec3(0,0,-1), {Vec3(-hx,-hy,-hz), Vec3(hx,-hy,-hz), Vec3(hx,hy,-hz), Vec3(-hx,hy,-hz)}},
        {Vec3(0,0,1),  {Vec3(hx,-hy,hz), Vec3(-hx,-hy,hz), Vec3(-hx,hy,hz), Vec3(hx,hy,hz)}},
        {Vec3(-1,0,0), {Vec3(-hx,-hy,hz), Vec3(-hx,-hy,-hz), Vec3(-hx,hy,-hz), Vec3(-hx,hy,hz)}},
        {Vec3(1,0,0),  {Vec3(hx,-hy,-hz), Vec3(hx,-hy,hz), Vec3(hx,hy,hz), Vec3(hx,hy,-hz)}},
        {Vec3(0,-1,0), {Vec3(-hx,-hy,hz), Vec3(hx,-hy,hz), Vec3(hx,-hy,-hz), Vec3(-hx,-hy,-hz)}},
        {Vec3(0,1,0),  {Vec3(-hx,hy,-hz), Vec3(hx,hy,-hz), Vec3(hx,hy,hz), Vec3(-hx,hy,hz)}}
    };

    for (auto& f : faces) {
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int i = 0; i < 4; i++)
            mesh.vertices.push_back(Vertex(f.verts[i], f.normal));
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 1);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base);
        mesh.indices.push_back(base + 2);
        mesh.indices.push_back(base + 3);
    }
    return mesh;
}

int main() {
    Settings settings;
    settings.load(SETTINGS_FILE);

    int winWidth = static_cast<int>(settings.getDouble("windowWidth", 1024));
    int winHeight = static_cast<int>(settings.getDouble("windowHeight", 1024));

    Window window;
    if (!window.initialize(winWidth, winHeight, "Raytracer Viewer")) {
        std::cerr << "Failed to create window\n";
        return 1;
    }

    int fbWidth, fbHeight;
    window.getFramebufferSize(fbWidth, fbHeight);

    auto renderer = Renderer::create();
    if (!renderer->initialize(window.getHandle(), fbWidth, fbHeight)) {
        std::cerr << "Failed to initialize renderer\n";
        return 1;
    }

    World world;

    auto addStatic = [&](MeshHandle mesh, const Vec3& position,
                         const RenderMaterial& material) -> Entity& {
        Entity e;
        e.mesh = mesh;
        e.material = material;
        e.transform.position = position;
        e.simulated = false;
        return world.add(e);
    };

    // Room materials
    RenderMaterial whiteMat;
    whiteMat.albedo = Vec3(0.73, 0.73, 0.73);
    whiteMat.roughness = 0.9f;

    RenderMaterial redWall;
    redWall.albedo = Vec3(0.65, 0.05, 0.05);
    redWall.roughness = 0.9f;

    RenderMaterial greenWall;
    greenWall.albedo = Vec3(0.12, 0.45, 0.15);
    greenWall.roughness = 0.9f;

    // Walls bake their position into the mesh geometry, so the entity transform
    // stays at the origin.
    auto floorMesh = createQuadMesh(Vec3(-5, 0, -5), Vec3(10, 0, 0), Vec3(0, 0, 10));
    addStatic(renderer->uploadMesh(floorMesh), Vec3(0, 0, 0), whiteMat);

    auto ceilMesh = createQuadMesh(Vec3(-5, 8, 5), Vec3(10, 0, 0), Vec3(0, 0, -10));
    addStatic(renderer->uploadMesh(ceilMesh), Vec3(0, 0, 0), whiteMat);

    auto backMesh = createQuadMesh(Vec3(-5, 0, 5), Vec3(10, 0, 0), Vec3(0, 8, 0));
    addStatic(renderer->uploadMesh(backMesh), Vec3(0, 0, 0), whiteMat);

    auto leftMesh = createQuadMesh(Vec3(-5, 0, -5), Vec3(0, 0, 10), Vec3(0, 8, 0));
    addStatic(renderer->uploadMesh(leftMesh), Vec3(0, 0, 0), redWall);

    auto rightMesh = createQuadMesh(Vec3(5, 0, 5), Vec3(0, 0, -10), Vec3(0, 8, 0));
    addStatic(renderer->uploadMesh(rightMesh), Vec3(0, 0, 0), greenWall);

    // Red sphere
    RenderMaterial redMat;
    redMat.albedo = Vec3(0.8, 0.1, 0.1);
    redMat.roughness = 0.3f;
    auto sphereMesh = createSphereMesh(1.0f, 32, 64);
    MeshHandle sphereHandle = renderer->uploadMesh(sphereMesh);
    addStatic(sphereHandle, Vec3(-2.0, 1.0, 0.0), redMat);

    // Metal sphere
    RenderMaterial metalMat;
    metalMat.albedo = Vec3(0.9, 0.9, 0.95);
    metalMat.metallic = 1.0f;
    metalMat.roughness = 0.1f;
    addStatic(sphereHandle, Vec3(0.0, 1.0, 0.0), metalMat);

    // Glass-like sphere (semi-transparent)
    RenderMaterial glassMat;
    glassMat.albedo = Vec3(0.5, 0.8, 1.0);
    glassMat.roughness = 0.1f;
    glassMat.opacity = 0.3f;
    addStatic(sphereHandle, Vec3(2.0, 1.0, 0.0), glassMat);

    // Green box — simulated: spins in place so the fixed timestep and time
    // controls are visible. Real motion systems plug into World::step the same way.
    RenderMaterial greenMat;
    greenMat.albedo = Vec3(0.2, 0.7, 0.2);
    greenMat.roughness = 0.5f;
    auto boxMesh = createBoxMesh(Vec3(1.5, 3.0, 1.5));
    Entity box;
    box.mesh = renderer->uploadMesh(boxMesh);
    box.material = greenMat;
    box.transform.position = Vec3(3.0, 1.5, 2.0);
    box.transform.rotation = Vec3(0.0, 0.4, 0.0);
    box.angularVelocity = Vec3(0.0, 0.6, 0.3);
    world.add(box);

    // Lights
    std::vector<PointLight> lights = {
        PointLight(Vec3(0, 7.0, 0), Vec3(1.0, 0.95, 0.9), 25.0f),
        PointLight(Vec3(-3, 5, -3), Vec3(0.4, 0.5, 1.0), 10.0f)
    };

    OrbitCamera camera;
    camera.target = Vec3(
        settings.getDouble("cameraTargetX", 0.0),
        settings.getDouble("cameraTargetY", 1.0),
        settings.getDouble("cameraTargetZ", 0.0));
    camera.distance = static_cast<float>(settings.getDouble("cameraDistance", 8.0));
    camera.yaw = static_cast<float>(settings.getDouble("cameraYaw", 0.0));
    camera.pitch = static_cast<float>(settings.getDouble("cameraPitch", 25.0));

    float exposure = static_cast<float>(settings.getDouble("exposure", 0.5));

    SimClock clock(settings.getDouble("fixedTimestep", 1.0 / 60.0));
    double simSpeed = settings.getDouble("timeScale", 1.0);  // speed when not paused
    bool paused = false;

    std::cerr << "Controls:\n";
    std::cerr << "  Left-drag=orbit, Right-drag=pan, Scroll=zoom\n";
    std::cerr << "  WASD=move, QE=up/down, Shift=fast\n";
    std::cerr << "  Up/Down=exposure, Esc=quit\n";
    std::cerr << "  Space=pause, ','/'.'=slower/faster sim, 0=reset speed\n";

    // Time controls fire on key-press edges, not while held; track last frame's
    // state to detect the rising edge. (A proper event queue would deliver these
    // as discrete press events instead of us reconstructing them here.)
    bool prevSpace = false, prevComma = false, prevPeriod = false, prevNum0 = false;

    while (!window.shouldClose()) {
        window.pollEvents();
        const InputState& input = window.getInput();
        double dt = window.getDeltaTime();

        if (input.keyEscape) break;

        camera.update(input, dt);

        if (input.keyUp) exposure *= 1.0f + 2.0f * static_cast<float>(dt);
        if (input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(dt);
        exposure = std::clamp(exposure, 0.05f, 20.0f);

        if (input.keySpace && !prevSpace) paused = !paused;
        if (input.keyComma && !prevComma)
            simSpeed = std::clamp(simSpeed * 0.5, 0.0625, 16.0);
        if (input.keyPeriod && !prevPeriod)
            simSpeed = std::clamp(simSpeed * 2.0, 0.0625, 16.0);
        if (input.keyNum0 && !prevNum0) { simSpeed = 1.0; paused = false; }
        prevSpace = input.keySpace;
        prevComma = input.keyComma;
        prevPeriod = input.keyPeriod;
        prevNum0 = input.keyNum0;

        clock.setTimeScale(paused ? 0.0 : simSpeed);
        int steps = clock.advance(dt);
        for (int i = 0; i < steps; i++) world.step(clock.fixedStep());
        double alpha = clock.interpolationAlpha();

        int w, h;
        window.getFramebufferSize(w, h);
        if (w != fbWidth || h != fbHeight) {
            fbWidth = w;
            fbHeight = h;
            renderer->resize(w, h);
        }

        float aspect = (h > 0) ? static_cast<float>(w) / h : 1.0f;
        CameraState camState = camera.getCameraState(aspect);

        renderer->beginFrame();
        renderer->setCamera(camState);
        renderer->setLights(lights, exposure);

        for (const Entity& e : world.entities()) {
            Transform t = world.renderTransform(e, alpha);
            renderer->drawMesh(e.mesh, t.matrix(), e.material);
        }

        renderer->endFrame();
    }

    // Save settings on exit
    int ww, wh;
    window.getSize(ww, wh);
    settings.setDouble("windowWidth", ww);
    settings.setDouble("windowHeight", wh);
    settings.setDouble("exposure", exposure);
    settings.setDouble("timeScale", simSpeed);
    settings.setDouble("fixedTimestep", clock.fixedStep());
    settings.setDouble("cameraTargetX", camera.target.x);
    settings.setDouble("cameraTargetY", camera.target.y);
    settings.setDouble("cameraTargetZ", camera.target.z);
    settings.setDouble("cameraDistance", camera.distance);
    settings.setDouble("cameraYaw", camera.yaw);
    settings.setDouble("cameraPitch", camera.pitch);
    settings.save(SETTINGS_FILE);
    std::cerr << "Settings saved to " << SETTINGS_FILE << "\n";

    renderer->shutdown();
    return 0;
}
