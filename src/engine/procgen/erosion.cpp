#include "erosion.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace engine {

float Heightmap::sampleWorld(float worldX, float worldZ) const {
    if (n < 2) return 0.0f;
    // World [-worldSize/2, worldSize/2] -> grid [0, n-1].
    float gx = (worldX / worldSize + 0.5f) * (n - 1);
    float gz = (worldZ / worldSize + 0.5f) * (n - 1);
    gx = std::clamp(gx, 0.0f, static_cast<float>(n - 1));
    gz = std::clamp(gz, 0.0f, static_cast<float>(n - 1));
    int x0 = static_cast<int>(gx), z0 = static_cast<int>(gz);
    int x1 = std::min(x0 + 1, n - 1), z1 = std::min(z0 + 1, n - 1);
    float fx = gx - x0, fz = gz - z0;
    float a = get(x0, z0), b = get(x1, z0), c = get(x0, z1), d = get(x1, z1);
    return (a * (1 - fx) + b * fx) * (1 - fz) + (c * (1 - fx) + d * fx) * fz;
}

Heightmap bakeHeightmap(const TerrainParams& params, const Noise& noise) {
    Heightmap hm;
    hm.n = std::max(2, params.resolution) + 1;
    hm.worldSize = params.size;
    hm.h.resize(static_cast<size_t>(hm.n) * hm.n);
    float half = params.size * 0.5f;
    float step = params.size / (hm.n - 1);
    for (int z = 0; z < hm.n; z++)
        for (int x = 0; x < hm.n; x++)
            hm.set(x, z, static_cast<float>(terrainHeight(
                             params, noise, -half + x * step, -half + z * step)));
    return hm;
}

namespace {

// Height + gradient at a fractional grid position (bilinear).
void heightAndGradient(const Heightmap& hm, float gx, float gz,
                       float& height, float& gx_out, float& gz_out) {
    int x0 = static_cast<int>(gx), z0 = static_cast<int>(gz);
    x0 = std::clamp(x0, 0, hm.n - 2);
    z0 = std::clamp(z0, 0, hm.n - 2);
    float fx = std::clamp(gx - x0, 0.0f, 1.0f), fz = std::clamp(gz - z0, 0.0f, 1.0f);
    float nw = hm.get(x0, z0), ne = hm.get(x0 + 1, z0);
    float sw = hm.get(x0, z0 + 1), se = hm.get(x0 + 1, z0 + 1);
    // Gradient from the bilinear patch derivatives.
    gx_out = (ne - nw) * (1 - fz) + (se - sw) * fz;
    gz_out = (sw - nw) * (1 - fx) + (se - ne) * fx;
    height = (nw * (1 - fx) + ne * fx) * (1 - fz) + (sw * (1 - fx) + se * fx) * fz;
}

}  // namespace

void erode(Heightmap& hm, const ErosionParams& p) {
    if (hm.n < 3) return;
    std::mt19937 gen(p.seed);
    std::uniform_real_distribution<float> pos(0.0f, static_cast<float>(hm.n - 1));

    // Precompute a circular erosion brush (offsets + normalized weights), so a
    // droplet removes material over an area instead of a single cell (no spikes).
    std::vector<std::pair<int, int>> brush;
    std::vector<float> brushW;
    {
        int r = std::max(1, p.erodeRadius);
        float wsum = 0.0f;
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++) {
                float d2 = static_cast<float>(dx * dx + dy * dy);
                if (d2 > r * r) continue;
                float w = 1.0f - std::sqrt(d2) / r;
                brush.push_back({dx, dy});
                brushW.push_back(w);
                wsum += w;
            }
        for (float& w : brushW) w /= wsum;
    }

    // --- Hydraulic: droplet simulation (Lague-style) ---
    for (int i = 0; i < p.droplets; i++) {
        float px = pos(gen), pz = pos(gen);
        float dx = 0.0f, dz = 0.0f;
        float speed = 1.0f, water = 1.0f, sediment = 0.0f;

        for (int life = 0; life < p.maxLifetime; life++) {
            int nx = static_cast<int>(px), nz = static_cast<int>(pz);
            if (nx < 0 || nz < 0 || nx >= hm.n - 1 || nz >= hm.n - 1) break;
            float fx = px - nx, fz = pz - nz;

            float h0, gx, gz;
            heightAndGradient(hm, px, pz, h0, gx, gz);

            // Blend direction: momentum vs downhill (negative gradient).
            dx = dx * p.inertia - gx * (1.0f - p.inertia);
            dz = dz * p.inertia - gz * (1.0f - p.inertia);
            float len = std::sqrt(dx * dx + dz * dz);
            if (len < 1e-6f) break;            // stuck in a pit / flat
            dx /= len; dz /= len;
            float nxp = px + dx, nzp = pz + dz;
            if (nxp < 0 || nzp < 0 || nxp >= hm.n - 1 || nzp >= hm.n - 1) break;

            float h1, ggx, ggz;
            heightAndGradient(hm, nxp, nzp, h1, ggx, ggz);
            float dh = h1 - h0;            // >0 uphill, <0 downhill

            float capacity =
                std::max(-dh, p.minSlope) * speed * water * p.sedimentCapacity;

            if (sediment > capacity || dh > 0.0f) {
                // Deposit: fill the pit (uphill) or shed excess. Bilinear back
                // into the 4 cells of the current position.
                float deposit = (dh > 0.0f) ? std::min(dh, sediment)
                                            : (sediment - capacity) * p.depositSpeed;
                sediment -= deposit;
                hm.set(nx, nz, hm.get(nx, nz) + deposit * (1 - fx) * (1 - fz));
                hm.set(nx + 1, nz, hm.get(nx + 1, nz) + deposit * fx * (1 - fz));
                hm.set(nx, nz + 1, hm.get(nx, nz + 1) + deposit * (1 - fx) * fz);
                hm.set(nx + 1, nz + 1, hm.get(nx + 1, nz + 1) + deposit * fx * fz);
            } else {
                // Erode over the brush, capped so we never dig below the drop.
                float erodeAmt = std::min((capacity - sediment) * p.erodeSpeed, -dh);
                for (size_t b = 0; b < brush.size(); b++) {
                    int bx = nx + brush[b].first, bz = nz + brush[b].second;
                    if (bx < 0 || bz < 0 || bx >= hm.n || bz >= hm.n) continue;
                    float w = erodeAmt * brushW[b];
                    hm.set(bx, bz, hm.get(bx, bz) - w);
                    sediment += w;
                }
            }

            speed = std::sqrt(std::max(0.0f, speed * speed + dh * p.gravity * -1.0f));
            water *= (1.0f - p.evaporation);
            px = nxp; pz = nzp;
        }
    }

    // --- Thermal: slump slopes steeper than the talus drop ---
    for (int it = 0; it < p.thermalIterations; it++) {
        std::vector<float> delta(hm.h.size(), 0.0f);
        for (int z = 1; z < hm.n - 1; z++) {
            for (int x = 1; x < hm.n - 1; x++) {
                float here = hm.get(x, z);
                // Move material to the lowest neighbor if the drop exceeds talus.
                int lx = x, lz = z; float lowest = here;
                const int off[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (auto& o : off) {
                    float nh = hm.get(x + o[0], z + o[1]);
                    if (nh < lowest) { lowest = nh; lx = x + o[0]; lz = z + o[1]; }
                }
                float drop = here - lowest;
                if (drop > p.talus) {
                    float move = (drop - p.talus) * 0.5f * p.thermalRate;
                    delta[static_cast<size_t>(z) * hm.n + x] -= move;
                    delta[static_cast<size_t>(lz) * hm.n + lx] += move;
                }
            }
        }
        for (size_t k = 0; k < hm.h.size(); k++) hm.h[k] += delta[k];
    }
}

RenderMesh generateTerrainMesh(const Heightmap& hm) {
    RenderMesh mesh;
    const int n = hm.n;
    const float half = hm.worldSize * 0.5f;
    const float step = hm.worldSize / (n - 1);
    for (int z = 0; z < n; z++)
        for (int x = 0; x < n; x++)
            mesh.vertices.push_back(
                Vertex(Vec3(-half + x * step, hm.get(x, z), -half + z * step),
                       Vec3(0, 1, 0)));
    for (int j = 0; j < n - 1; j++) {
        for (int i = 0; i < n - 1; i++) {
            uint32_t a = static_cast<uint32_t>(j * n + i);
            uint32_t b = a + 1;
            uint32_t c = a + static_cast<uint32_t>(n);
            uint32_t d = c + 1;
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }
    MeshBuilder::recomputeNormals(mesh);
    MeshBuilder::generatePlanarUVs(mesh, 1, 1.0f / hm.worldSize);
    Noise tint(1234);
    for (Vertex& v : mesh.vertices) {
        double nv = tint.noise2(v.position.x * 0.15, v.position.z * 0.15);
        v.color = terrainColor(v.position.y, v.normal.y, nv);
    }
    return mesh;
}

}  // namespace engine
