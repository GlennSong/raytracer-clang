// GPU erosion bake (metropolis-scale-plan P0.5) — the droplet hydraulic +
// thermal talus simulation from src/engine/procgen/erosion.cpp, ported to
// Metal compute. Compiled at runtime by erosion_gpu.mm (newLibraryWithSource,
// read from disk relative to CWD like the renderer's shaders).
//
// DETERMINISM DESIGN (load-bearing — see erodeGpu):
//  * Droplets run in fixed batches. Within a batch every thread READS the
//    heightmap as it stood after the previous batch (buffer 0 is never written
//    by the droplet/thermal kernels) and WRITES its brush deltas as FIXED-POINT
//    int32 (scale 2^16) via atomic adds into the delta buffer (buffer 1).
//    Integer addition is order-independent, so two runs are bit-identical
//    regardless of GPU thread scheduling.
//  * applyDelta folds the accumulated integers into the float heightmap
//    between batches (the encoder is serial, so dispatch order = data order).
//  * Per-droplet randomness is a counter-based hash of (seed, dropletIndex) —
//    no shared RNG stream, so a droplet's path depends only on its index.
//
// STABILITY (clampDelta + residue carry): the CPU sim self-limits because
// every droplet's erode <= -dh / deposit <= dh caps are re-evaluated against
// the terrain it just modified. Batched droplets all read the same snapshot,
// so hundreds converging on one pit or drainage cell sum their contributions
// unboundedly (and eventually overflow the fixed point). clampDelta restores
// the same invariant at batch granularity: per batch a cell fills at most to
// its highest neighbor and cuts at most to its lowest (relief measured on the
// snapshot); the un-applied remainder stays in the accumulator and drains on
// later batches as the relief updates, so mass is conserved and channel
// incision/pit filling converge across batches instead of being discarded.
// All of it is a deterministic function of (snapshot, integer sum) — the
// determinism contract is untouched.

#include <metal_stdlib>
using namespace metal;

// Layout mirrored EXACTLY (order + 4-byte scalar types) by ErosionUniformsCpu
// in src/engine/procgen/erosion_gpu.mm — keep the two in sync.
struct ErosionUniforms {
    int   n;                  // heightmap vertices per side
    int   batchStart;         // first droplet index in this batch
    int   batchCount;         // droplets in this batch
    int   maxLifetime;
    int   brushCount;         // entries in the erosion brush
    uint  seed;
    float inertia;
    float sedimentCapacity;
    float depositSpeed;
    float erodeSpeed;
    float evaporation;
    float gravity;
    float minSlope;
    float talus;
    float thermalRate;
};

constant float kFixedScale = 65536.0f;          // int32 fixed point, 2^16

// PCG hash (Jarzynski & Olano, "Hash Functions for GPU Rendering").
static inline uint pcgHash(uint v) {
    uint state = v * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Counter-based uniform [0,1): a pure function of (seed, counter), so every
// droplet is independent of scheduling. 24-bit resolution (float-exact).
static inline float rand01(uint seed, uint counter) {
    uint h = pcgHash(pcgHash(seed) ^ counter);
    return (h >> 8) * (1.0f / 16777216.0f);
}

// Fixed-point scatter-add. round() is symmetric, so +v/-v pairs (thermal
// slumping) conserve mass exactly in the integer domain.
static inline void scatterFixed(device atomic_int* delta, int n, int x, int z,
                                float v) {
    int fixedV = (int)round(v * kFixedScale);
    if (fixedV != 0)
        atomic_fetch_add_explicit(&delta[z * n + x], fixedV,
                                  memory_order_relaxed);
}

// Height + gradient at a fractional grid position (bilinear) — the exact
// counterpart of heightAndGradient in erosion.cpp.
static inline void heightAndGradient(device const float* h, int n,
                                     float gx, float gz, thread float& height,
                                     thread float& outGx, thread float& outGz) {
    int x0 = clamp((int)gx, 0, n - 2);
    int z0 = clamp((int)gz, 0, n - 2);
    float fx = clamp(gx - x0, 0.0f, 1.0f), fz = clamp(gz - z0, 0.0f, 1.0f);
    float nw = h[z0 * n + x0], ne = h[z0 * n + x0 + 1];
    float sw = h[(z0 + 1) * n + x0], se = h[(z0 + 1) * n + x0 + 1];
    outGx = (ne - nw) * (1 - fz) + (se - sw) * fz;
    outGz = (sw - nw) * (1 - fx) + (se - ne) * fx;
    height = (nw * (1 - fx) + ne * fx) * (1 - fz) +
             (sw * (1 - fx) + se * fx) * fz;
}

// --- Hydraulic: one thread = one droplet's full lifetime (Lague-style, the
// same terms as the CPU sim). Reads the batch snapshot, scatters fixed-point
// deltas. brushOff is (dx,dz) pairs, flat int array.
kernel void erodeDroplets(device const float*       height   [[buffer(0)]],
                          device atomic_int*        delta    [[buffer(1)]],
                          constant ErosionUniforms& u        [[buffer(2)]],
                          device const int*         brushOff [[buffer(3)]],
                          device const float*       brushW   [[buffer(4)]],
                          uint tid [[thread_position_in_grid]]) {
    if ((int)tid >= u.batchCount) return;
    uint drop = (uint)u.batchStart + tid;
    const int n = u.n;

    // Spawn anywhere on the grid, mirroring the CPU's uniform [0, n-1).
    float px = rand01(u.seed, drop * 2u + 0u) * (n - 1);
    float pz = rand01(u.seed, drop * 2u + 1u) * (n - 1);
    float dx = 0.0f, dz = 0.0f;
    float speed = 1.0f, water = 1.0f, sediment = 0.0f;

    for (int life = 0; life < u.maxLifetime; life++) {
        int nx = (int)px, nz = (int)pz;
        if (nx < 0 || nz < 0 || nx >= n - 1 || nz >= n - 1) break;
        float fx = px - nx, fz = pz - nz;

        float h0, gx, gz;
        heightAndGradient(height, n, px, pz, h0, gx, gz);

        // Blend direction: momentum vs downhill (negative gradient).
        dx = dx * u.inertia - gx * (1.0f - u.inertia);
        dz = dz * u.inertia - gz * (1.0f - u.inertia);
        float len = sqrt(dx * dx + dz * dz);
        if (len < 1e-6f) break;            // stuck in a pit / flat
        dx /= len; dz /= len;
        float nxp = px + dx, nzp = pz + dz;
        if (nxp < 0 || nzp < 0 || nxp >= n - 1 || nzp >= n - 1) break;

        float h1, ggx, ggz;
        heightAndGradient(height, n, nxp, nzp, h1, ggx, ggz);
        float dh = h1 - h0;                // >0 uphill, <0 downhill

        float capacity =
            max(-dh, u.minSlope) * speed * water * u.sedimentCapacity;

        if (sediment > capacity || dh > 0.0f) {
            // Deposit: fill the pit (uphill) or shed excess. Bilinear back
            // into the 4 cells of the current position.
            float deposit = (dh > 0.0f)
                                ? min(dh, sediment)
                                : (sediment - capacity) * u.depositSpeed;
            sediment -= deposit;
            scatterFixed(delta, n, nx, nz, deposit * (1 - fx) * (1 - fz));
            scatterFixed(delta, n, nx + 1, nz, deposit * fx * (1 - fz));
            scatterFixed(delta, n, nx, nz + 1, deposit * (1 - fx) * fz);
            scatterFixed(delta, n, nx + 1, nz + 1, deposit * fx * fz);
        } else {
            // Erode over the brush, capped so we never dig below the drop.
            float erodeAmt = min((capacity - sediment) * u.erodeSpeed, -dh);
            for (int b = 0; b < u.brushCount; b++) {
                int bx = nx + brushOff[2 * b], bz = nz + brushOff[2 * b + 1];
                if (bx < 0 || bz < 0 || bx >= n || bz >= n) continue;
                float w = erodeAmt * brushW[b];
                scatterFixed(delta, n, bx, bz, -w);
                sediment += w;
            }
        }

        speed = sqrt(max(0.0f, speed * speed - dh * u.gravity));
        water *= (1.0f - u.evaporation);
        px = nxp; pz = nzp;
    }
}

// --- Per-batch stabilizer (hydraulic batches only): split each cell's
// accumulated delta into an applied part — clamped to the cell's local relief
// on the pre-batch snapshot — and a residue that stays in the accumulator for
// later batches. One thread per cell; height is read-only here, so neighbor
// reads are race-free (applyApplied writes it in the next dispatch).
kernel void clampDelta(device const float*       height  [[buffer(0)]],
                       device atomic_int*        delta   [[buffer(1)]],
                       constant ErosionUniforms& u       [[buffer(2)]],
                       device int*               applied [[buffer(3)]],
                       uint2 gid [[thread_position_in_grid]]) {
    const int n = u.n;
    int x = (int)gid.x, z = (int)gid.y;
    if (x >= n || z >= n) return;
    int idx = z * n + x;
    int d = atomic_load_explicit(&delta[idx], memory_order_relaxed);
    if (d == 0) { applied[idx] = 0; return; }
    float h = height[idx];
    // Relief bracket including self, so the bounds always contain zero: a
    // strict local minimum admits no further cutting, a peak no piling.
    float mn = h, mx = h;
    if (x > 0)     { float v = height[idx - 1]; mn = min(mn, v); mx = max(mx, v); }
    if (x < n - 1) { float v = height[idx + 1]; mn = min(mn, v); mx = max(mx, v); }
    if (z > 0)     { float v = height[idx - n]; mn = min(mn, v); mx = max(mx, v); }
    if (z < n - 1) { float v = height[idx + n]; mn = min(mn, v); mx = max(mx, v); }
    float df = (float)d * (1.0f / kFixedScale);
    float use = clamp(df, mn - h, mx - h);
    int useFixed = (int)round(use * kFixedScale);
    applied[idx] = useFixed;
    atomic_store_explicit(&delta[idx], d - useFixed, memory_order_relaxed);
}

// --- Fold the clamped (applied) part into the heightmap. Own-cell only, so
// no race with clampDelta's neighbor reads (separate, serially ordered pass).
kernel void applyApplied(device float*             height  [[buffer(0)]],
                         constant ErosionUniforms& u       [[buffer(2)]],
                         device const int*         applied [[buffer(3)]],
                         uint tid [[thread_position_in_grid]]) {
    uint cells = (uint)(u.n * u.n);
    if (tid >= cells) return;
    int d = applied[tid];
    if (d != 0) height[tid] += (float)d * (1.0f / kFixedScale);
}

// --- Thermal: slump slopes steeper than the talus drop. The CPU version
// already double-buffers through a per-iteration delta array, so this is the
// same computation with the delta in fixed point.
kernel void thermalStep(device const float*       height [[buffer(0)]],
                        device atomic_int*        delta  [[buffer(1)]],
                        constant ErosionUniforms& u      [[buffer(2)]],
                        uint2 gid [[thread_position_in_grid]]) {
    const int n = u.n;
    int x = (int)gid.x, z = (int)gid.y;
    if (x < 1 || z < 1 || x >= n - 1 || z >= n - 1) return;
    float here = height[z * n + x];
    // Move material to the lowest neighbor if the drop exceeds talus.
    int lx = x, lz = z; float lowest = here;
    const int off[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int i = 0; i < 4; i++) {
        float nh = height[(z + off[i][1]) * n + (x + off[i][0])];
        if (nh < lowest) { lowest = nh; lx = x + off[i][0]; lz = z + off[i][1]; }
    }
    float drop = here - lowest;
    if (drop > u.talus) {
        float move = (drop - u.talus) * 0.5f * u.thermalRate;
        scatterFixed(delta, n, x, z, -move);
        scatterFixed(delta, n, lx, lz, move);
    }
}

// --- Fold the accumulated fixed-point deltas into the float heightmap and
// clear them for the next batch. The only kernel that writes buffer 0.
kernel void applyDelta(device float*             height [[buffer(0)]],
                       device atomic_int*        delta  [[buffer(1)]],
                       constant ErosionUniforms& u      [[buffer(2)]],
                       uint tid [[thread_position_in_grid]]) {
    uint cells = (uint)(u.n * u.n);
    if (tid >= cells) return;
    int d = atomic_exchange_explicit(&delta[tid], 0, memory_order_relaxed);
    if (d != 0) height[tid] += (float)d * (1.0f / kFixedScale);
}
