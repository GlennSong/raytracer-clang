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

// THE MOON (the month): a sphere disc lit from the TRUE sun, so the terminator
// falls where the phase says — a thin crescent hugging the sun's side, a half
// at the quarters, a full face opposite the sun. Mirrors moonDisc in
// shaders/vulkan/sky.frag (the verified reference); zero below the horizon.
constant float kMoonRadius = 0.026;    // radians (~1.5 deg: ~3x real, so the phase reads)
static float3 moonDisc(float3 dir, device const LightUniforms& env) {
    float I = env.skyMoonIntensity;
    if (I <= 0.0) return float3(0.0);
    float3 m = normalize(env.skyMoonDir);
    float3 sun = normalize(env.skyMoonSun);
    float illum = env.skyMoonIllum;
    float cosM = dot(dir, m);
    const float3 moonTint = float3(0.86, 0.88, 0.95);
    float3 col = moonTint * pow(max(cosM, 0.0), 300.0) * 0.08 * I * illum;
    float cosR = cos(kMoonRadius);
    if (cosM < cosR - 0.0004) return col;
    float3 ref = fabs(m.y) < 0.98 ? float3(0.0, 1.0, 0.0) : float3(1.0, 0.0, 0.0);
    float3 right = normalize(cross(m, ref));
    float3 up = cross(right, m);
    float3 off = dir - m * cosM;
    float2 uv = float2(dot(off, right), dot(off, up)) / sin(kMoonRadius);
    float r2 = dot(uv, uv);
    float edge = 1.0 - smoothstep(0.92, 1.02, sqrt(r2));
    float w = sqrt(max(1.0 - r2, 0.0));
    float3 n = right * uv.x + up * uv.y - m * w;
    float lit = max(dot(n, sun), 0.0);
    col += moonTint * I * (lit + 0.012) * edge;
    return col;
}

// Procedural sky environment. Colors and the sun arc come from `env` (the day/
// night state baked into LightUniforms), so dawn→day→dusk→night grade smoothly
// and the disc fades out as the sun sets. The shader only interpolates; the
// time-of-day curve is computed engine-side (DayNightCycle).
float3 sampleEnvironment(float3 dir, device const LightUniforms& env) {
    float skyBlend = saturate(dir.y);
    float3 sky = mix(env.skyHorizon, env.skyZenith, pow(skyBlend, 0.5));

    // Below the horizon, hold the horizon haze for a band before fading to the
    // ground tint deeper down. On a finite/flat world this lets distant terrain
    // (faded toward the fog/horizon color) dissolve seamlessly into the sky from
    // high vantage points, instead of meeting a hard ground-colored band where
    // the terrain mesh ends.
    float3 lowerHaze = mix(env.skyHorizon, env.skyGround,
                           smoothstep(0.0, -0.4, dir.y));
    float horizonBlend = smoothstep(-0.05, 0.05, dir.y);
    float3 col = mix(lowerHaze, sky, horizonBlend);

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
    col += moonDisc(dir, env);

    return col;
}

// --- Scattering sky (cinematic-sky phase) ---
// Hillaire-style two-LUT sky: a transmittance LUT (256x64, sun/space visibility
// through the shell) and a sky-view LUT (192x108, the full sky dome radiance at
// the camera), both computed by kernels in atmosphere.metal whenever the sun or
// atmosphere parameters change. These helpers define the uv<->parameter
// mappings shared by the bake kernels and every sampler (skybox, composite,
// sky->cube bake), so they must agree exactly.

// Distance from a point at radius r (view cos zenith mu) to the atmosphere top.
static float skyDistToTop(float r, float mu, float Rt) {
    float d = r * r * (mu * mu - 1.0) + Rt * Rt;
    return max(-r * mu + sqrt(max(d, 0.0)), 0.0);
}

// Transmittance LUT mapping (Bruneton): u = normalized distance-to-top, v =
// normalized horizon distance rho. Invertible; covers rays that reach space.
static float2 skyTransRMuToUv(float r, float mu, float Rg, float Rt) {
    float H = sqrt(max(Rt * Rt - Rg * Rg, 0.0));
    float rho = sqrt(max(r * r - Rg * Rg, 0.0));
    float d = skyDistToTop(r, mu, Rt);
    float dMin = Rt - r;
    float dMax = rho + H;
    float u = (d - dMin) / max(dMax - dMin, 1e-4);
    float v = rho / max(H, 1e-4);
    return float2(clamp(u, 0.0, 1.0), clamp(v, 0.0, 1.0));
}

static void skyTransUvToRMu(float2 uv, float Rg, float Rt,
                            thread float& r, thread float& mu) {
    float H = sqrt(max(Rt * Rt - Rg * Rg, 0.0));
    float rho = H * uv.y;
    r = sqrt(rho * rho + Rg * Rg);
    float dMin = Rt - r;
    float dMax = rho + H;
    float d = dMin + uv.x * (dMax - dMin);
    mu = (d <= 0.0) ? 1.0
                    : clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
}

// Transmittance toward direction with cos-zenith mu from height (r above the
// planet center). Returns 0 for rays that hit the ground (earth shadow).
static float3 skySampleTransmittance(texture2d<float> transLut, float r, float mu,
                                     float Rg, float Rt) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    float horizonMu = -sqrt(max(1.0 - (Rg * Rg) / (r * r), 0.0));
    if (mu < horizonMu) return float3(0.0);
    return transLut.sample(s, skyTransRMuToUv(r, mu, Rg, Rt)).rgb;
}

// Sky-view LUT mapping (Hillaire): u = sqrt-warped azimuth from the sun
// (resolution concentrated sun-ward, mirror-symmetric), v = nonlinear view
// zenith with the break exactly on the horizon (v = 0.5), so the horizon line
// stays crisp at 108 rows.
static float2 skyViewDirToUv(float viewZenithCos, float lightViewAngle,
                             float r, float Rg) {
    float vHorizon = sqrt(max(r * r - Rg * Rg, 0.0));
    float cosBeta = vHorizon / r;
    float beta = acos(clamp(cosBeta, -1.0, 1.0));
    float zenithHorizon = M_PI_F - beta;
    float viewZenith = acos(clamp(viewZenithCos, -1.0, 1.0));
    float v;
    if (viewZenith < zenithHorizon) {
        float c = viewZenith / max(zenithHorizon, 1e-4);
        v = (1.0 - sqrt(max(1.0 - c, 0.0))) * 0.5;
    } else {
        float c = (viewZenith - zenithHorizon) / max(beta, 1e-4);
        v = 0.5 + 0.5 * sqrt(saturate(c));
    }
    float u = sqrt(saturate(lightViewAngle / M_PI_F));
    return float2(u, v);
}

static void skyViewUvToDir(float2 uv, float r, float Rg,
                           thread float& viewZenith, thread float& lightViewAngle) {
    float vHorizon = sqrt(max(r * r - Rg * Rg, 0.0));
    float cosBeta = vHorizon / r;
    float beta = acos(clamp(cosBeta, -1.0, 1.0));
    float zenithHorizon = M_PI_F - beta;
    if (uv.y < 0.5) {
        float c = 1.0 - 2.0 * uv.y;      // 1 at zenith row, 0 at horizon
        viewZenith = zenithHorizon * (1.0 - c * c);
    } else {
        float c = 2.0 * uv.y - 1.0;
        viewZenith = zenithHorizon + beta * (c * c);
    }
    lightViewAngle = uv.x * uv.x * M_PI_F;
}

// Sample the sky-view LUT for a world-space direction. The LUT was built at
// camera height env.skyCamHeight with planet-up = world +Y (a flat world), sun
// azimuth at u = 0. Shallow below-horizon dips blend back toward the horizon
// radiance over a few degrees — the LUT's true answer there is the (dark)
// ground bounce, but on a finite flat world the region just under the horizon
// stands in for implied distant terrain, which reads as haze, not soil; this is
// the scattering-sky twin of sampleEnvironment's lowerHaze band, and it lets
// the ocean/terrain edge dissolve instead of meeting a hard dark line.
static float3 sampleSkyViewLut(float3 dir, texture2d<float> skyViewLut,
                               device const LightUniforms& env) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    float Rg = env.skyPlanetRadius;
    float r = Rg + max(env.skyCamHeight, 0.5);
    float3 sunDir = env.skySunDir;
    // Azimuthal angle between view and sun on the horizontal plane.
    float2 vh = dir.xz, sh = sunDir.xz;
    float vl = length(vh), sl = length(sh);
    float azim = (vl > 1e-4 && sl > 1e-4)
        ? acos(clamp(dot(vh / vl, sh / sl), -1.0, 1.0))
        : 0.0;
    float3 col = skyViewLut.sample(s, skyViewDirToUv(dir.y, azim, r, Rg)).rgb;

    // Below-horizon haze band: mix toward the just-above-horizon radiance.
    float vHorizon = sqrt(max(r * r - Rg * Rg, 0.0));
    float zenithHorizon = M_PI_F - acos(clamp(vHorizon / r, -1.0, 1.0));
    float dip = acos(clamp(dir.y, -1.0, 1.0)) - zenithHorizon;   // >0 below horizon
    if (dip > 0.0) {
        const float HAZE_BAND = 0.10;   // ~6 degrees
        float w = 1.0 - saturate(dip / HAZE_BAND);
        if (w > 0.0) {
            float horizonCos = cos(zenithHorizon * 0.995);
            float3 horizonCol =
                skyViewLut.sample(s, skyViewDirToUv(horizonCos, azim, r, Rg)).rgb;
            col = mix(col, horizonCol, w * w);
        }
    }
    return col;
}

// Full scattering-sky radiance for a direction: LUT sky plus an analytic sun
// disc (the LUT is far too coarse for a crisp disc). The disc rides the
// transmittance LUT, so it reddens and dims through the thick horizon air
// exactly like the sky around it, and fades out below the horizon.
static float3 scatteringSkyRadiance(float3 dir, texture2d<float> skyViewLut,
                                    texture2d<float> transLut,
                                    device const LightUniforms& env) {
    float3 col = sampleSkyViewLut(dir, skyViewLut, env);
    float cosSun = dot(dir, env.skySunDir);
    if (cosSun > env.skySunDiscCos - 0.0002 && env.skySunIntensity > 0.0) {
        float Rg = env.skyPlanetRadius;
        float r = Rg + max(env.skyCamHeight, 0.5);
        float3 t = skySampleTransmittance(transLut, r, dir.y, Rg, env.skyAtmosRadius);
        // Soft limb: full radiance inside the disc, quick falloff at the edge.
        float disc = smoothstep(env.skySunDiscCos - 0.0002, env.skySunDiscCos, cosSun);
        // Artistic disc radiance: bright enough to saturate + bloom (the real
        // sun is ~1.5e4x its illuminance — that would blow half-float range).
        col += env.skySunColor * (env.skySunIntensity * 120.0) * t * disc;
    }
    // CIRCUMSOLAR AUREOLE (device: "the sun looks like a white dot"): the warm
    // few-degree glow around the sun is real forward-scattered radiance, not
    // lens bloom — with the post bloom at a whisper the naked disc read as a
    // dot. Two lobes: a tight hot core and a broad soft skirt, both tinted by
    // transmittance so the glow oranges as the sun drops.
    if (env.skySunIntensity > 0.0) {
        float cosSun2 = clamp(dot(dir, env.skySunDir), 0.0, 1.0);
        float Rg = env.skyPlanetRadius;
        float r = Rg + max(env.skyCamHeight, 0.5);
        float3 t = skySampleTransmittance(transLut, r, dir.y, Rg, env.skyAtmosRadius);
        float core = pow(cosSun2, 4000.0);   // ~1.5 deg hot core
        float skirt = pow(cosSun2, 250.0);   // ~7 deg soft skirt
        col += env.skySunColor * env.skySunIntensity * t *
               (core * 2.0 + skirt * 0.35);
    }
    col += moonDisc(dir, env);   // the moon over the scattering sky too
    // Half-float ceiling: the disc + aureole can sum past 65504 and the HDR
    // target folds the overflow into a BLACK sun (measured: a dark disc at
    // the sun's spot). Clamp comfortably under the format's edge.
    return min(col, float3(60000.0));
}

// --- Procedural clouds (ADR-0016 step 3) ---
// An FBM noise layer painted on the sky dome — not volumetric, and never baked
// into reflection probes (a screen/SSR visual only). Overlaid by the skybox and
// composite passes on top of the day/night sky from sampleEnvironment().

// Lattice hash over the float BITS (see hash21 in common.metal): the old
// fract(p*123.34) mangle loses all sub-unit precision once the drift term
// grows the coordinates past ~1e5 — a long session turned the cloud field
// into angular slabs. Bit-identical to the Vulkan sky.frag copy.
float cloudHash(float2 p) {
    uint h = as_type<uint>(p.x) * 0x85EBCA6Bu ^ as_type<uint>(p.y) * 0xC2B2AE35u;
    h = (h ^ (h >> 13)) * 0x27D4EB2Du;
    h ^= h >> 15;
    return float(h >> 8) * (1.0 / 16777216.0);
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
    // Wrapped drift — unbounded time quantizes the noise interpolant; one
    // pattern jump every ~38 days of drift is the whole cost (mirrors sky.frag).
    float2 wind = fmod(float2(env.skyCloudTime * 0.02, env.skyCloudTime * 0.012), 65536.0);
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

// Fullscreen triangle: 3 vertices cover the screen without a vertex buffer.
// It winds COUNTER-CLOCKWISE in screen space (as do the copies of this idiom
// in post_composite/atmosphere/clouds/star): encoders that cull — the main
// pass and probe bake front-face and back-cull for scene meshes — must set
// CullModeNone around this draw or the whole sky silently disappears.
vertex SkyboxOut vertexSkybox(
    constant CameraUniforms& camera [[buffer(1)]],
    uint vid [[vertex_id]]
) {
    // Generate fullscreen triangle (oversized, clipped to viewport)
    float2 uv = float2((vid << 1) & 2, vid & 2);
    // Reverse-Z (ADR-0034 Phase 0): the far plane is z=0. The skybox draws first
    // with no depth write, so this value only needs to stay inside the clip
    // range; a hair above 0 avoids far-plane clipping on some GPUs.
    float4 clipPos = float4(uv * 2.0 - 1.0, 0.0001, 1.0);

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
    texture2d<float> skyViewLut [[texture(10)]],
    texture2d<float> skyTransLut [[texture(11)]],
    sampler envSampler [[sampler(0)]]
) {
    float3 dir = normalize(in.viewDir);
    // HDR provider is pre-baked into a cubemap at load (ADR-0016) — a cheap cube
    // lookup instead of per-sample equirect atan2/acos. The scattering sky
    // (cinematic-sky, env.skyModel == 1) samples the sky-view LUT + sun disc.
    float3 color = (env.mode == 1) ? envCube.sample(envSampler, dir).rgb
                 : (env.skyModel == 1)
                     ? scatteringSkyRadiance(dir, skyViewLut, skyTransLut, lightData)
                     : sampleEnvironment(dir, lightData);
    // Clouds overlay the procedural sky only (a captured HDR has its own), and
    // are skipped during the probe bake (env.cloudsEnabled == 0) and when the
    // volumetric cloud layer replaces them (the renderer clears the flag).
    if (env.mode == 0 && env.cloudsEnabled != 0)
        color = applyClouds(color, dir, lightData);
    // Linear scene-referred radiance — exposure and tone mapping are applied
    // once, in the composite pass (this output also feeds SSR/probe bakes).
    return float4(color, 1.0);
}

// Scattering sky -> cubemap bake (cinematic-sky): renders one cube face per
// draw by sampling the sky LUTs, exactly like fragmentEquirectBake does for a
// captured HDR. The baked cube then feeds the SAME machinery a real HDR uses —
// skybox/composite sky pixels, GGX prefilter + irradiance for IBL, and the
// reflection-probe bake — so lighting matches the backdrop by construction.
fragment float4 fragmentScatteringSkyBake(
    SkyboxOut in [[stage_in]],
    device const LightUniforms& lightData [[buffer(4)]],
    texture2d<float> skyViewLut [[texture(0)]],
    texture2d<float> skyTransLut [[texture(1)]]
) {
    float3 dir = normalize(in.viewDir);
    return float4(scatteringSkyRadiance(dir, skyViewLut, skyTransLut, lightData), 1.0);
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
    constant uint& count [[buffer(2)]],
    sampler s [[sampler(0)]],
    uint i [[thread_position_in_grid]]
) {
    if (i >= count) return;
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
