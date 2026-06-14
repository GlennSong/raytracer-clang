// Environment providers (ADR-0016): equirect HDR sampling, the procedural
// day/night sky + FBM clouds, the skybox passes, the equirect→cube bake, and
// the IBL precompute kernels (BRDF LUT, GGX prefilter).

// Sample an equirectangular (lat-long) environment map by world-space direction.
// u wraps around the horizon (longitude), v runs zenith→nadir (latitude).
float3 sampleEquirect(texture2d<float> envMap, sampler s, float3 dir) {
    float u = atan2(dir.z, dir.x) * (0.5 / M_PI_F) + 0.5;
    float v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / M_PI_F);
    return envMap.sample(s, float2(u, v)).rgb;
}

// Procedural sky environment. Colors and the sun arc come from `env` (the day/
// night state baked into LightUniforms), so dawn→day→dusk→night grade smoothly
// and the disc fades out as the sun sets. The shader only interpolates; the
// time-of-day curve is computed engine-side (DayNightCycle).
float3 sampleEnvironment(float3 dir, device const LightUniforms& env) {
    float skyBlend = saturate(dir.y);
    float3 sky = mix(env.skyHorizon, env.skyZenith, pow(skyBlend, 0.5));

    // Soft ground plane below horizon
    float horizonBlend = smoothstep(-0.05, 0.05, dir.y);
    float3 col = mix(env.skyGround, sky, horizonBlend);

    // Sun disc and glow — `skySunIntensity` is 0 at night, so they vanish.
    float disc = env.skySunIntensity;
    float3 sc = env.skySunColor;
    float sunDot = max(dot(dir, env.skySunDir), 0.0);
    col += sc * pow(sunDot, 256.0) * 8.0  * disc;   // bright disc
    col += sc * pow(sunDot, 32.0)  * 1.0  * disc;   // inner glow
    col += sc * pow(sunDot, 4.0)   * 0.15 * disc;   // outer halo

    // Subtle atmospheric scattering near horizon, tinted by the sun
    float horizonGlow = pow(1.0 - abs(dir.y), 8.0);
    col += sc * horizonGlow * 0.1 * disc;

    return col;
}

// --- Procedural clouds (ADR-0016 step 3) ---
// An FBM noise layer painted on the sky dome — not volumetric, and never baked
// into reflection probes (a screen/SSR visual only). Overlaid by the skybox and
// composite passes on top of the day/night sky from sampleEnvironment().

float cloudHash(float2 p) {
    p = fract(p * float2(123.34, 345.45));
    p += dot(p, p + 34.345);
    return fract(p.x * p.y);
}

float cloudNoise(float2 p) {
    float2 i = floor(p);
    float2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);                 // smoothstep interpolation
    float a = cloudHash(i + float2(0.0, 0.0));
    float b = cloudHash(i + float2(1.0, 0.0));
    float c = cloudHash(i + float2(0.0, 1.0));
    float d = cloudHash(i + float2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float cloudFbm(float2 p) {
    float sum = 0.0;
    float amp = 0.5;
    for (int i = 0; i < 5; i++) {
        sum += amp * cloudNoise(p);
        p *= 2.02;
        amp *= 0.5;
    }
    return sum;
}

// Overlay clouds onto a base sky color for a viewing direction.
float3 applyClouds(float3 baseSky, float3 dir, device const LightUniforms& env) {
    if (dir.y <= 0.02) return baseSky;           // below horizon: no clouds

    // Planar projection of the dome onto a cloud plane: directions near the
    // horizon stretch (parallax), overhead compresses. Drift the field over time.
    float2 uv = dir.xz / (dir.y + 0.10);
    float2 wind = float2(env.skyCloudTime * 0.02, env.skyCloudTime * 0.012);
    float n = cloudFbm(uv * env.skyCloudScale + wind);

    // Coverage threshold → soft cloud mask, faded near the horizon.
    float cov = env.skyCloudCoverage;
    float mask = smoothstep(cov, cov + 0.20, n) * env.skyCloudDensity;
    mask *= smoothstep(0.02, 0.25, dir.y);
    mask = saturate(mask);

    // Shade: sunlit tops toward the sun, shadowed base away; whole layer tracks
    // day/night brightness (≈0 at night → dark silhouettes) and the sun tint.
    float sunAmt = max(dot(dir, env.skySunDir), 0.0);
    float3 cloudLit  = env.skySunColor;
    float3 cloudDark = float3(0.40, 0.42, 0.50);
    float3 cloudColor = mix(cloudDark, cloudLit, pow(sunAmt, 1.5));
    cloudColor *= (0.25 + 0.75 * env.skySunIntensity);

    return mix(baseSky, cloudColor, mask);
}

// --- Skybox shaders ---
struct SkyboxOut {
    float4 position [[position]];
    float3 viewDir;
};

// Fullscreen triangle: 3 vertices cover the screen without a vertex buffer
vertex SkyboxOut vertexSkybox(
    constant CameraUniforms& camera [[buffer(1)]],
    uint vid [[vertex_id]]
) {
    // Generate fullscreen triangle (oversized, clipped to viewport)
    float2 uv = float2((vid << 1) & 2, vid & 2);
    // z=0.999 — just inside the far plane to avoid far-plane clipping on some GPUs
    float4 clipPos = float4(uv * 2.0 - 1.0, 0.999, 1.0);

    SkyboxOut out;
    out.position = clipPos;

    // Reconstruct the world-space view ray via inverse view-projection.
    // CRITICAL: do NOT normalize here. Far-plane offsets are affine in NDC, so
    // their interpolation across the triangle is exact; normalizing at the
    // vertices warps every interior pixel (up to ~25° with this oversized
    // triangle) — that warp was the historic "mis-oriented cube bake".
    // The fragment shaders normalize per pixel.
    float4 worldPos = camera.invViewProjection * float4(clipPos.xy, 1.0, 1.0);
    out.viewDir = worldPos.xyz / worldPos.w - camera.cameraPosition;
    return out;
}

fragment float4 fragmentSkybox(
    SkyboxOut in [[stage_in]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant EnvUniforms& env [[buffer(5)]],
    texturecube<float> envCube [[texture(0)]],
    sampler envSampler [[sampler(0)]]
) {
    float3 dir = normalize(in.viewDir);
    // HDR provider is pre-baked into a cubemap at load (ADR-0016) — a cheap cube
    // lookup instead of per-sample equirect atan2/acos.
    float3 color = (env.mode == 1) ? envCube.sample(envSampler, dir).rgb
                                   : sampleEnvironment(dir, lightData);
    // Clouds overlay the procedural sky only (a captured HDR has its own), and
    // are skipped during the probe bake (env.cloudsEnabled == 0).
    if (env.mode == 0 && env.cloudsEnabled != 0)
        color = applyClouds(color, dir, lightData);
    // Linear scene-referred radiance — exposure and tone mapping are applied
    // once, in the composite pass (this output also feeds SSR/probe bakes).
    return float4(color, 1.0);
}

// Equirect → cubemap bake (ADR-0016): renders one cube face per draw, sampling
// the equirect HDR by the reconstructed world direction. Reuses vertexSkybox and
// the probe bake's per-face cameras, so the result matches how cubes are sampled.
// Raw radiance only — no exposure or clouds, so a cube sample equals the equirect.
fragment float4 fragmentEquirectBake(
    SkyboxOut in [[stage_in]],
    texture2d<float> equirect [[texture(0)]],
    sampler equirectSampler [[sampler(0)]]
) {
    return float4(sampleEquirect(equirect, equirectSampler, normalize(in.viewDir)), 1.0);
}

// --- BRDF integration LUT compute shader (split-sum, run once at startup) ---

// Van der Corput radical inverse for Hammersley sequence
float radicalInverse_VdC(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

float2 hammersley(uint i, uint N) {
    return float2(float(i) / float(N), radicalInverse_VdC(i));
}

float3 importanceSampleGGX(float2 Xi, float3 N, float roughness) {
    float a = roughness * roughness;
    float phi = 2.0 * M_PI_F * Xi.x;
    float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    float3 H = float3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);

    // Tangent-space to world-space
    float3 up = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    float3 tangent = normalize(cross(up, N));
    float3 bitangent = cross(N, tangent);

    return normalize(tangent * H.x + bitangent * H.y + N * H.z);
}

kernel void integrateBRDF(
    texture2d<float, access::write> lut [[texture(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    float2 texSize = float2(lut.get_width(), lut.get_height());
    if (gid.x >= uint(texSize.x) || gid.y >= uint(texSize.y)) return;

    float NdotV = (float(gid.x) + 0.5) / texSize.x;
    float roughness = (float(gid.y) + 0.5) / texSize.y;
    NdotV = max(NdotV, 0.001);

    float3 V;
    V.x = sqrt(1.0 - NdotV * NdotV);
    V.y = 0.0;
    V.z = NdotV;

    float3 N = float3(0, 0, 1);
    float A = 0.0, B = 0.0;
    const uint SAMPLES = 1024u;

    for (uint i = 0u; i < SAMPLES; i++) {
        float2 Xi = hammersley(i, SAMPLES);
        float3 H = importanceSampleGGX(Xi, N, roughness);
        float3 L = normalize(2.0 * dot(V, H) * H - V);

        float NdotL = max(L.z, 0.0);
        float NdotH = max(H.z, 0.0);
        float VdotH = max(dot(V, H), 0.0);

        if (NdotL > 0.0) {
            float a2 = roughness * roughness * roughness * roughness;
            float G_V = NdotL * (NdotV * (1.0 - sqrt(a2)) + sqrt(a2));
            float G_L = NdotV * (NdotL * (1.0 - sqrt(a2)) + sqrt(a2));
            float G = 0.5 / max(G_V + G_L, 0.001);
            float G_Vis = (G * VdotH * NdotL) / max(NdotH, 0.001);
            float Fc = pow(1.0 - VdotH, 5.0);
            A += (1.0 - Fc) * G_Vis;
            B += Fc * G_Vis;
        }
    }

    A /= float(SAMPLES);
    B /= float(SAMPLES);
    lut.write(float4(A, B, 0, 1), gid);
}

// --- Cubemap pre-filter + irradiance compute shaders ---

// Texel (uv in [-1,1], v down) → sample direction for a cube face. The MSL
// twin of cubeFaceDirection in src/renderer/cube_faces.h — the convention is
// unit-tested there (tests/test_cube_faces.cpp).
float3 cubeFaceDir(int face, float2 uv) {
    switch (face) {
        case 0: return float3( 1, -uv.y, -uv.x);  // +X
        case 1: return float3(-1, -uv.y,  uv.x);  // -X
        case 2: return float3( uv.x,  1,  uv.y);  // +Y
        case 3: return float3( uv.x, -1, -uv.y);  // -Y
        case 4: return float3( uv.x, -uv.y,  1);  // +Z
        default: return float3(-uv.x, -uv.y, -1); // -Z
    }
}

// Clamp a radiance sample's luminance to tame HDR fireflies: a tiny ultra-bright
// sun disc otherwise sparkles across the IBL at finite sample counts (the
// speckles on everything when an HDR is used). The sun's *direct* contribution
// comes from the extracted directional light (ADR-0017), so the IBL only needs
// the sky — clamping the sun out of the convolution is correct, not a loss.
constant float IBL_FIREFLY_CLAMP = 16.0;
inline float3 clampRadiance(float3 c) {
    float lum = max(c.r, max(c.g, c.b));
    return (lum > IBL_FIREFLY_CLAMP) ? c * (IBL_FIREFLY_CLAMP / lum) : c;
}

// GGX importance-sampled radiance prefilter, one (mip, face) per dispatch.
kernel void prefilterEnvMap(
    texturecube<float> inputCube [[texture(0)]],
    texturecube<float, access::write> outputFace [[texture(1)]],
    constant float& roughness [[buffer(0)]],
    constant int& faceIndex [[buffer(1)]],
    sampler envSampler [[sampler(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint size = outputFace.get_width();
    if (gid.x >= size || gid.y >= size) return;

    float2 uv = (float2(gid) + 0.5) / float(size) * 2.0 - 1.0;
    float3 N = normalize(cubeFaceDir(faceIndex, uv));

    if (roughness < 0.01) {
        // No filtering needed for mip 0
        outputFace.write(inputCube.sample(envSampler, N), gid, faceIndex);
        return;
    }

    float3 R = N;
    float3 V = R;
    float3 prefilteredColor = float3(0.0);
    float totalWeight = 0.0;
    const uint SAMPLES = 512u;

    // Sample a blurred source mip as the lobe widens — without this, a tiny
    // very-bright sun disc aliases into fireflies at finite sample counts.
    float srcLod = roughness * 4.0;

    for (uint i = 0u; i < SAMPLES; i++) {
        float2 Xi = hammersley(i, SAMPLES);
        float3 H = importanceSampleGGX(Xi, N, roughness);
        float3 L = normalize(2.0 * dot(V, H) * H - V);

        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0.0) {
            prefilteredColor += clampRadiance(inputCube.sample(envSampler, L, level(srcLod)).rgb) * NdotL;
            totalWeight += NdotL;
        }
    }

    prefilteredColor /= max(totalWeight, 0.001);
    outputFace.write(float4(prefilteredColor, 1.0), gid, faceIndex);
}

// Validation probe (load-time diagnostics): writes the reconstructed bake
// direction as color so the CPU can verify the rasterizer/invVP link of the
// chain numerically.
fragment float4 fragmentDirectionDebug(SkyboxOut in [[stage_in]]) {
    return float4(normalize(in.viewDir) * 0.5 + 0.5, 1.0);
}

// Validation probe (load-time diagnostics): hardware-sample the cube along
// supplied directions so the CPU can verify the full bake→sample chain.
kernel void sampleCubeForValidation(
    texturecube<float> cube [[texture(0)]],
    device const float4* dirs [[buffer(0)]],
    device float4* results [[buffer(1)]],
    sampler s [[sampler(0)]],
    uint i [[thread_position_in_grid]]
) {
    results[i] = cube.sample(s, normalize(dirs[i].xyz), level(4.0));
}

// Cosine-convolved irradiance, one face per dispatch. Stores the cosine-
// weighted average radiance E(n)/π — the value that multiplies albedo
// directly in the ambient term (the Lambert 1/π is already folded in).
kernel void convolveIrradiance(
    texturecube<float> inputCube [[texture(0)]],
    texturecube<float, access::write> outputFace [[texture(1)]],
    constant int& faceIndex [[buffer(0)]],
    sampler envSampler [[sampler(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint size = outputFace.get_width();
    if (gid.x >= size || gid.y >= size) return;

    float2 uv = (float2(gid) + 0.5) / float(size) * 2.0 - 1.0;
    float3 N = normalize(cubeFaceDir(faceIndex, uv));

    float3 up = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    float3 T = normalize(cross(up, N));
    float3 B = cross(N, T);

    // Cosine-weighted hemisphere sampling; a blurred source mip stands in for
    // the many more samples a tiny sun disc would otherwise need.
    const uint SAMPLES = 256u;
    float3 sum = float3(0.0);
    for (uint i = 0u; i < SAMPLES; i++) {
        float2 Xi = hammersley(i, SAMPLES);
        float phi = 2.0 * M_PI_F * Xi.x;
        float cosTheta = sqrt(1.0 - Xi.y);
        float sinTheta = sqrt(Xi.y);
        float3 l = T * (cos(phi) * sinTheta) + B * (sin(phi) * sinTheta) + N * cosTheta;
        sum += clampRadiance(inputCube.sample(envSampler, l, level(4.0)).rgb);
    }
    outputFace.write(float4(sum / float(SAMPLES), 1.0), gid, faceIndex);
}
