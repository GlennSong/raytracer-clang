#include "math.h"
#include "renderer/renderer.h"
#include "renderer/window.h"
#include "renderer/orbit_camera.h"
#include <iostream>
#include <vector>

struct SceneObject {
    MeshHandle mesh;
    Mat4 transform;
    RenderMaterial material;
};

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
    Window window;
    if (!window.initialize(1024, 1024, "Raytracer Viewer")) {
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

    std::vector<SceneObject> objects;

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

    // Floor
    auto floorMesh = createQuadMesh(Vec3(-5, 0, -5), Vec3(10, 0, 0), Vec3(0, 0, 10));
    MeshHandle floorHandle = renderer->uploadMesh(floorMesh);
    objects.push_back({floorHandle, Mat4::identity(), whiteMat});

    // Ceiling
    auto ceilMesh = createQuadMesh(Vec3(-5, 8, 5), Vec3(10, 0, 0), Vec3(0, 0, -10));
    MeshHandle ceilHandle = renderer->uploadMesh(ceilMesh);
    objects.push_back({ceilHandle, Mat4::identity(), whiteMat});

    // Back wall
    auto backMesh = createQuadMesh(Vec3(-5, 0, 5), Vec3(10, 0, 0), Vec3(0, 8, 0));
    MeshHandle backHandle = renderer->uploadMesh(backMesh);
    objects.push_back({backHandle, Mat4::identity(), whiteMat});

    // Left wall (red)
    auto leftMesh = createQuadMesh(Vec3(-5, 0, -5), Vec3(0, 0, 10), Vec3(0, 8, 0));
    MeshHandle leftHandle = renderer->uploadMesh(leftMesh);
    objects.push_back({leftHandle, Mat4::identity(), redWall});

    // Right wall (green)
    auto rightMesh = createQuadMesh(Vec3(5, 0, 5), Vec3(0, 0, -10), Vec3(0, 8, 0));
    MeshHandle rightHandle = renderer->uploadMesh(rightMesh);
    objects.push_back({rightHandle, Mat4::identity(), greenWall});

    // Red sphere
    RenderMaterial redMat;
    redMat.albedo = Vec3(0.8, 0.1, 0.1);
    redMat.roughness = 0.3f;
    auto sphereMesh = createSphereMesh(1.0f, 32, 64);
    MeshHandle sphereHandle = renderer->uploadMesh(sphereMesh);
    objects.push_back({sphereHandle, Mat4::translate(-2.0, 1.0, 0.0), redMat});

    // Metal sphere
    RenderMaterial metalMat;
    metalMat.albedo = Vec3(0.9, 0.9, 0.95);
    metalMat.metallic = 1.0f;
    metalMat.roughness = 0.1f;
    objects.push_back({sphereHandle, Mat4::translate(0.0, 1.0, 0.0), metalMat});

    // Glass-like sphere (semi-transparent)
    RenderMaterial glassMat;
    glassMat.albedo = Vec3(0.5, 0.8, 1.0);
    glassMat.roughness = 0.1f;
    glassMat.opacity = 0.3f;
    objects.push_back({sphereHandle, Mat4::translate(2.0, 1.0, 0.0), glassMat});

    // Green box
    RenderMaterial greenMat;
    greenMat.albedo = Vec3(0.2, 0.7, 0.2);
    greenMat.roughness = 0.5f;
    auto boxMesh = createBoxMesh(Vec3(1.5, 3.0, 1.5));
    MeshHandle boxHandle = renderer->uploadMesh(boxMesh);
    objects.push_back({boxHandle, Mat4::translate(3.0, 1.5, 2.0) * Mat4::rotateY(0.4), greenMat});

    // Lights
    std::vector<PointLight> lights = {
        PointLight(Vec3(0, 7.0, 0), Vec3(1.0, 0.95, 0.9), 25.0f),
        PointLight(Vec3(-3, 5, -3), Vec3(0.4, 0.5, 1.0), 10.0f)
    };

    OrbitCamera camera;
    camera.target = Vec3(0, 1, 0);
    camera.distance = 8.0f;
    camera.pitch = 25.0f;

    float exposure = 1.0f;

    std::cerr << "Controls:\n";
    std::cerr << "  Left-drag=orbit, Right-drag=pan, Scroll=zoom\n";
    std::cerr << "  WASD=move, QE=up/down, Shift=fast\n";
    std::cerr << "  Up/Down=exposure, Esc=quit\n";

    while (!window.shouldClose()) {
        window.pollEvents();
        const InputState& input = window.getInput();
        double dt = window.getDeltaTime();

        if (input.keyEscape) break;

        camera.update(input, dt);

        if (input.keyUp) exposure *= 1.0f + 2.0f * static_cast<float>(dt);
        if (input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(dt);
        exposure = std::clamp(exposure, 0.05f, 20.0f);

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

        for (const auto& obj : objects) {
            renderer->drawMesh(obj.mesh, obj.transform, obj.material);
        }

        renderer->endFrame();
    }

    renderer->shutdown();
    return 0;
}
