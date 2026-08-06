// Planetary atmosphere — Metal port of the CPU reference (engine/procgen/
// atmosphere.cpp) and the Vulkan atmosphere.frag, single-scattering Rayleigh + Mie
// (procedural-planet-plan P3). A fullscreen pass: reconstruct the world ray per
// pixel, raymarch the atmosphere shell, composite the in-scattered light over the
// HDR scene by its transmittance. The planet is an analytic sphere so the march
// clips at the surface without a depth fetch.
//
// WIRED (metal_renderer.mm builds atmospherePipeline from vertexAtmosphere +
// fragmentAtmosphereGlow, and endFrame runs the pass after the scene / before
// post). Compiles clean on macOS as of 2026-07-29.
//
// TWO fragment entry points, deliberately:
//   fragmentAtmosphereGlow — BOUND. Additive-only (no scene fetch, no view
//     transmittance); the One+One blend does the compositing, so the limb halo
//     feeds the bloom pass. This is what Metal and WebGPU ship.
//   fragmentAtmosphere     — NOT bound. The full-composite variant
//     (scene * transmittance + inScatter), kept as the 1:1 mirror of
//     shaders/vulkan/atmosphere.frag, which is still shader-only. See the
//     "Planetary atmosphere" row in docs/renderer-parity.md. Do not delete it
//     without resolving that divergence — but note it IS compiled on every
//     launch, so a backend subsetting shaderFiles for compile time should know
//     it is paying for an unbound entry point.

// AtmosphereUniforms is defined in shader_types.h (prepended by the shader loader).

struct AtmosphereOut {
    float4 position [[position]];
    float2 uv;
};

static bool atmRaySphere(float3 origin, float3 dir, float radius,
                         thread float& t0, thread float& t1) {
    float b = dot(origin, dir);
    float c = dot(origin, origin) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0) return false;
    float s = sqrt(disc);
    t0 = -b - s;
    t1 = -b + s;
    return t1 >= 0.0;
}

static float atmDensity(float3 q, float planetRadius, float scaleH) {
    float h = length(q) - planetRadius;
    return exp(-max(h, 0.0) / scaleH);
}

// Returns (odRayleigh, odMie) along pa->pb.
static float2 atmOpticalDepth(float3 pa, float3 pb, int steps,
                              float planetRadius, float rH, float mH) {
    float2 od = float2(0.0);
    float3 seg = pb - pa;
    float len = length(seg);
    if (len < 1e-6) return od;
    float3 d = seg / len;
    float ds = len / float(steps);
    for (int i = 0; i < steps; i++) {
        float3 q = pa + d * (ds * (float(i) + 0.5));
        od.x += atmDensity(q, planetRadius, rH) * ds;
        od.y += atmDensity(q, planetRadius, mH) * ds;
    }
    return od;
}

vertex AtmosphereOut vertexAtmosphere(uint vid [[vertex_id]]) {
    AtmosphereOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    // Flip Y to Metal's texture origin (top-left), exactly like vertexComposite —
    // the fragment's ndc reconstruction does `-(uv.y*2-1)` and the scene sample both
    // assume this y-down uv. Without the flip the ray Y is inverted and the glow
    // mirrors vertically, drifting off the planet as the camera pitches.
    out.uv = float2(uv.x, 1.0 - uv.y);
    return out;
}

fragment float4 fragmentAtmosphere(AtmosphereOut in [[stage_in]],
                                   texture2d<float> sceneColor [[texture(0)]],
                                   constant AtmosphereUniforms& a [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);
    const float PI = 3.14159265359;

    // Metal NDC has a flipped Y vs the UV (see post_common.metal ssrViewPos).
    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = a.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = a.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(a.sunDirection.xyz);

    float3 scene = sceneColor.sample(samp, in.uv).rgb;

    float planetRadius = a.radii.x, atmosRadius = a.radii.y;
    float rH = a.radii.z, mH = a.radii.w;
    float mieCoeff = a.mie.x, mieG = a.mie.y;
    int viewSamples = int(a.mie.z), lightSamples = int(a.mie.w);
    float3 rayleighCoeff = a.rayleighCoeff.rgb;

    float3 origin = camPos - a.planetCenter.xyz;

    float aEnter, aExit;
    if (!atmRaySphere(origin, dir, atmosRadius, aEnter, aExit)) return float4(scene, 1.0);

    float tMax = aExit;
    float g0, g1;
    if (atmRaySphere(origin, dir, planetRadius, g0, g1) && g0 > 0.0) tMax = min(tMax, g0);
    float tEnter = max(0.0, aEnter);
    if (tMax <= tEnter) return float4(scene, 1.0);

    int steps = max(2, viewSamples);
    float ds = (tMax - tEnter) / float(steps);

    float mu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = mieG;
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    float odViewR = 0.0, odViewM = 0.0;
    float3 inscatR = float3(0.0), inscatM = float3(0.0);

    for (int i = 0; i < steps; i++) {
        float t = tEnter + ds * (float(i) + 0.5);
        float3 q = origin + dir * t;

        float dR = atmDensity(q, planetRadius, rH) * ds;
        float dM = atmDensity(q, planetRadius, mH) * ds;
        odViewR += dR; odViewM += dM;

        float s0, s1;
        bool shadowed = atmRaySphere(q, sunDir, planetRadius, s0, s1) && s1 > 0.0 && s0 > 0.0;
        if (shadowed) continue;

        float la0, la1;
        atmRaySphere(q, sunDir, atmosRadius, la0, la1);
        float3 lightExit = q + sunDir * max(0.0, la1);
        float2 odL = atmOpticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH);

        float3 tau = rayleighCoeff * (odViewR + odL.x) + float3(mieCoeff * (odViewM + odL.y));
        float3 tr = exp(-tau);
        inscatR += tr * dR;
        inscatM += tr * dM;
    }

    float3 rayleigh = rayleighCoeff * inscatR * phaseR;
    float3 mieTerm = inscatM * (mieCoeff * phaseM);
    float3 inScatter = a.sunColor.rgb * (rayleigh + mieTerm) * a.sunColor.w;

    float3 tauView = rayleighCoeff * odViewR + float3(mieCoeff * odViewM);
    float3 viewTransmittance = exp(-tauView);

    return float4(scene * viewTransmittance + inScatter, 1.0);
}

// Additive variant used by the realtime pass: outputs ONLY the in-scattered light
// (no scene sample, no transmittance), for a fullscreen triangle blended One+One
// over the HDR scene. The limb halo + haze then bloom in the post pass. This is the
// entry point the Metal renderer's atmospherePipeline binds.
fragment float4 fragmentAtmosphereGlow(AtmosphereOut in [[stage_in]],
                                       constant AtmosphereUniforms& a [[buffer(0)]]) {
    const float PI = 3.14159265359;
    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = a.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = a.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(a.sunDirection.xyz);

    float planetRadius = a.radii.x, atmosRadius = a.radii.y;
    float rH = a.radii.z, mH = a.radii.w;
    float mieCoeff = a.mie.x, mieG = a.mie.y;
    int viewSamples = int(a.mie.z), lightSamples = int(a.mie.w);
    float3 rayleighCoeff = a.rayleighCoeff.rgb;

    float3 origin = camPos - a.planetCenter.xyz;

    float aEnter, aExit;
    if (!atmRaySphere(origin, dir, atmosRadius, aEnter, aExit)) return float4(0.0);
    float tMax = aExit;
    float g0, g1;
    if (atmRaySphere(origin, dir, planetRadius, g0, g1) && g0 > 0.0) tMax = min(tMax, g0);
    float tEnter = max(0.0, aEnter);
    if (tMax <= tEnter) return float4(0.0);

    int steps = max(2, viewSamples);
    float ds = (tMax - tEnter) / float(steps);
    float mu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g = mieG;
    float phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
                   ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

    float odViewR = 0.0, odViewM = 0.0;
    float3 inscatR = float3(0.0), inscatM = float3(0.0);
    for (int i = 0; i < steps; i++) {
        float t = tEnter + ds * (float(i) + 0.5);
        float3 q = origin + dir * t;
        float dR = atmDensity(q, planetRadius, rH) * ds;
        float dM = atmDensity(q, planetRadius, mH) * ds;
        odViewR += dR; odViewM += dM;
        float s0, s1;
        bool shadowed = atmRaySphere(q, sunDir, planetRadius, s0, s1) && s1 > 0.0 && s0 > 0.0;
        if (shadowed) continue;
        float la0, la1;
        atmRaySphere(q, sunDir, atmosRadius, la0, la1);
        float3 lightExit = q + sunDir * max(0.0, la1);
        float2 odL = atmOpticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH);
        float3 tau = rayleighCoeff * (odViewR + odL.x) + float3(mieCoeff * (odViewM + odL.y));
        float3 tr = exp(-tau);
        inscatR += tr * dR;
        inscatM += tr * dM;
    }
    float3 rayleigh = rayleighCoeff * inscatR * phaseR;
    float3 mieTerm = inscatM * (mieCoeff * phaseM);
    float3 inScatter = a.sunColor.rgb * (rayleigh + mieTerm) * a.sunColor.w;
    return float4(inScatter, 1.0);
}

// --- Scattering-sky LUT bake (cinematic-sky phase) ---
// Hillaire-style two-LUT sky for the ground-level scenes (the planet-limb pass
// above stays for space views). The uv<->parameter mappings live in
// environment.metal (skyTrans*/skyView* helpers) so bake and sample agree.
// Dispatched by the renderer only when the sun direction, camera altitude band,
// or atmosphere parameters change — effectively free per frame.

// Ozone absorption (no scattering): a tent profile centered at 25 km. Gives the
// deep zenith blue and keeps sunsets from going green.
constant float3 SKY_OZONE_ABS = float3(0.650e-6, 1.881e-6, 0.085e-6);

static float skyOzoneDensity(float h) {
    return max(0.0, 1.0 - fabs(h - 25000.0) / 15000.0);
}

// Transmittance LUT: for each (r, mu) texel, march to the top of the atmosphere
// accumulating Rayleigh + Mie + ozone optical depth; store exp(-tau).
kernel void skyTransmittanceLut(
    texture2d<float, access::write> lut [[texture(0)]],
    constant SkyLUTUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint W = lut.get_width(), H = lut.get_height();
    if (gid.x >= W || gid.y >= H) return;
    float2 uv = (float2(gid) + 0.5) / float2(W, H);
    float Rg = u.planet.x, Rt = u.planet.y;
    float r, mu;
    skyTransUvToRMu(uv, Rg, Rt, r, mu);

    float dist = skyDistToTop(r, mu, Rt);
    const int STEPS = 48;
    float ds = dist / float(STEPS);
    float3 od = float3(0.0);
    for (int i = 0; i < STEPS; i++) {
        float t = (float(i) + 0.5) * ds;
        float rp = sqrt(max(r * r + t * t + 2.0 * r * mu * t, 1.0));
        float h = max(rp - Rg, 0.0);
        od += (u.rayleigh.xyz * exp(-h / u.rayleigh.w)
             + float3(u.mie.y) * exp(-h / u.mie.z)
             + SKY_OZONE_ABS * skyOzoneDensity(h)) * ds;
    }
    lut.write(float4(exp(-od), 1.0), gid);
}

// Sky-view LUT: single-scattered radiance over the whole dome as seen from the
// camera's altitude, with sun transmittance from the LUT above, an analytic
// per-step scattering integral (Hillaire), a cheap uniform multiple-scattering
// boost (keeps the away-from-sun sky and twilight from collapsing to black),
// and a diffuse ground bounce for below-horizon rays.
kernel void skyViewLut(
    texture2d<float, access::write> lut [[texture(0)]],
    texture2d<float> transLut [[texture(1)]],
    constant SkyLUTUniforms& u [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint W = lut.get_width(), H = lut.get_height();
    if (gid.x >= W || gid.y >= H) return;
    float2 uv = (float2(gid) + 0.5) / float2(W, H);

    float Rg = u.planet.x, Rt = u.planet.y;
    float r = Rg + max(u.sunDirection.w, 0.5);

    float viewZenith, azim;
    skyViewUvToDir(uv, r, Rg, viewZenith, azim);
    float sz = sin(viewZenith);
    float3 dir = float3(sz * cos(azim), cos(viewZenith), sz * sin(azim));

    // Sun in the LUT frame: planet-up = +Y, sun azimuth = 0.
    float cz = clamp(u.sunDirection.y, -1.0, 1.0);
    float3 sunDir = float3(sqrt(max(1.0 - cz * cz, 0.0)), cz, 0.0);

    float3 origin = float3(0.0, r, 0.0);
    float tMax = skyDistToTop(r, dir.y, Rt);
    bool hitGround = false;
    {
        float b = r * dir.y;
        float c = r * r - Rg * Rg;
        float disc = b * b - c;
        if (disc > 0.0) {
            float tg = -b - sqrt(disc);
            if (tg > 0.0) { tMax = tg; hitGround = true; }
        }
    }

    float nu = dot(dir, sunDir);
    float phaseR = 3.0 / (16.0 * M_PI_F) * (1.0 + nu * nu);
    float g = u.mie.w, g2 = g * g;
    float phaseM = 3.0 / (8.0 * M_PI_F) * ((1.0 - g2) * (1.0 + nu * nu)) /
                   ((2.0 + g2) * pow(1.0 + g2 - 2.0 * g * nu, 1.5));

    // Uniform multiple-scattering stand-in, faded through twilight into night.
    float msGain = 0.10 * u.planet.w * saturate(cz * 4.0 + 0.25);

    const int STEPS = 48;
    float ds = tMax / float(STEPS);
    float3 T = float3(1.0);
    float3 L = float3(0.0);
    float3 pLast = origin;
    for (int i = 0; i < STEPS; i++) {
        float t = (float(i) + 0.5) * ds;
        float3 p = origin + dir * t;
        pLast = p;
        float rp = length(p);
        float h = max(rp - Rg, 0.0);
        float rhoR = exp(-h / u.rayleigh.w);
        float rhoM = exp(-h / u.mie.z);
        float3 extinction = max(u.rayleigh.xyz * rhoR + float3(u.mie.y) * rhoM
                              + SKY_OZONE_ABS * skyOzoneDensity(h), float3(1e-9));
        float muS = dot(p / rp, sunDir);
        float3 sunT = skySampleTransmittance(transLut, rp, muS, Rg, Rt);
        float3 S = sunT * (u.rayleigh.xyz * rhoR * phaseR + float3(u.mie.x) * rhoM * phaseM)
                 + (u.rayleigh.xyz * rhoR + float3(u.mie.x) * rhoM) * msGain;
        float3 stepT = exp(-extinction * ds);
        L += T * (S - S * stepT) / extinction;   // analytic in-step integral
        T *= stepT;
    }

    if (hitGround) {
        float rp = max(length(pLast), Rg);
        float muS = dot(pLast / rp, sunDir);
        float3 sunT = skySampleTransmittance(transLut, rp, muS, Rg, Rt);
        L += T * u.ground.rgb * (1.0 / M_PI_F) * sunT * max(muS, 0.0);
    }

    float3 radiance = L * u.sunColor.rgb * u.sunColor.w * u.planet.z;
    lut.write(float4(radiance, 1.0), gid);
}
