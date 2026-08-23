#version 450
// Volumetric clouds — the WIRED port of shaders/metal/clouds.metal (which
// itself began as the P4 draft of this file; the Metal side then grew the
// Perlin-Worley texture set, the weather map, the stratified dithers, the
// two-step march with zero-run stretching and entry refinement, and the
// analytic in-scatter — this rewrite brings all of it back). A shared density
// + lighting + march core over two domains selected by uni.layer.w: 0 = SLAB
// (the cloudscape over a ground scene — depth-occluded) and 1 = SHELL (a
// planet's cloud deck). Renders at HALF resolution into an RGBA16F target as
// a premultiplied overlay — rgb = in-scattered radiance, a = transmittance —
// reading only the scene depth (reverse-Z) for occlusion; clouds_composite.frag
// upsamples it bilaterally onto the full-res HDR scene.
//
// Fullscreen triangle from composite.vert; ndc reconstruction uses the same
// FLIPPED invViewProjection convention as ssao.frag (uv*2-1, no negation).

layout(set = 0, binding = 0) uniform CloudUniforms {
    mat4 invViewProjection;
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunColor;        // rgb, w intensity
    vec4 skyAmbient;      // rgb ambient, w time
    vec4 planetCenter;
    vec4 layer;           // x bottom, y top, z planetRadius, w domainMode
    vec4 params;          // x coverage, y densityScale, z noiseScale, w windSpeed
    vec4 march;           // x viewSteps, y lightSteps, z phaseG, w farDistance
    vec4 detail;          // x detailStrength (edge-erosion depth), yzw unused
} u;

layout(set = 0, binding = 1) uniform sampler2D sceneDepth;
layout(set = 0, binding = 2) uniform sampler3D baseNoise;    // repeat, linear
layout(set = 0, binding = 3) uniform sampler3D detailNoise;  // repeat, linear

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

float clRemap(float x, float a, float b, float c, float d) {
    return c + (x - a) / max(b - a, 1e-5) * (d - c);
}

// The march's start-offset dither, STRATIFIED FOR THE 2x2 UPSAMPLE: the four
// half-res texels feeding one full-res pixel get four DISTINCT offsets
// spanning [0,1) (Bayer 2x2), which the bilateral upsample cancels almost
// exactly. White noise clumps; IGN is tuned for a wider footprint.
float clDither(vec2 pixel) {
    ivec2 c = ivec2(pixel) & 1;
    const float kOrder[4] = float[4](0.0, 0.5, 0.75, 0.25);
    return kOrder[c.y * 2 + c.x] + 0.125;
}

// Interleaved gradient noise (Jimenez) — locally well-distributed, which is
// what the sun-march decorrelation needs.
float clIgn(vec2 pixel) {
    return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

float clLayerHeight(vec3 p) {
    float h = (u.layer.w < 0.5) ? p.y : (length(p - u.planetCenter.xyz) - u.layer.z);
    return clamp((h - u.layer.x) / max(1e-4, u.layer.y - u.layer.x), 0.0, 1.0);
}

// The WEATHER MAP: a kilometres-scale coverage field over the deck, reusing
// the base texture's perlin-worley R at ~140x the billow scale — formations
// and true gaps, with the tile period beyond the march distance. Fixed
// mid-slice: the field must not swim as the ray climbs the slab.
float clWeather(vec2 xz) {
    vec2 q = xz * (u.params.z * 0.035);
    return texture(baseNoise, vec3(q.x, 0.37, q.y)).r;
}

// Coverage after the weather map: fair skies carve banks with real gaps,
// storm coverage saturates the field into a full deck.
float clLocalCoverage(float weather) {
    return clamp(u.params.x * (0.30 + 1.4 * weather), 0.0, 1.0);
}

// `detailFade` scales the high-frequency erosion (1 = full, 0 = shape only) —
// steps longer than the detail texture's cells sample it below Nyquist and
// turn cauliflower into confetti, so far samples keep shape, not noise.
float clDensity(vec3 p, float detailFade) {
    float hf = clLayerHeight(p);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    float profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);
    vec3 wind = vec3(u.params.w * u.skyAmbient.w, 0.0, 0.0);
    vec3 q = (p + wind) * u.params.z;
    // Base shape: R = perlin-worley, GBA = worley fBm at rising frequency.
    vec4 nse = texture(baseNoise, q * 0.25);
    float lowFbm = nse.g * 0.625 + nse.b * 0.25 + nse.a * 0.125;
    float base = clamp(clRemap(nse.r, lowFbm - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    float cov = clLocalCoverage(clWeather(p.xz));
    float d = clamp((base - (1.0 - cov)) / max(1e-3, cov), 0.0, 1.0);
    // Edge erosion: subtract high-frequency worley scaled by (1 - d), so
    // interiors stay solid and edges wisp away.
    vec3 det = texture(detailNoise, q * 2.0).rgb;
    float detailFbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
    d = clamp(d - detailFbm * u.detail.x * detailFade * (1.0 - d), 0.0, 1.0);
    return d * profile * u.params.y;
}

// Henyey-Greenstein, normalised so isotropic (g = 0) returns 1 — the result
// multiplies a directional sun term that carries no compensating 4pi.
float clPhaseHG(float mu, float g) {
    float g2 = g * g;
    return (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

bool clLayerInterval(vec3 origin, vec3 dir, out float t0, out float t1) {
    t0 = 0.0; t1 = 0.0;
    if (u.layer.w < 0.5) {
        if (abs(dir.y) < 1e-5) {
            if (origin.y < u.layer.x || origin.y > u.layer.y) return false;
            t0 = 0.0; t1 = u.march.w; return true;
        }
        float ta = (u.layer.x - origin.y) / dir.y;
        float tb = (u.layer.y - origin.y) / dir.y;
        t0 = max(min(ta, tb), 0.0);
        t1 = max(ta, tb);
        return t1 > 0.0;
    }
    vec3 oc = origin - u.planetCenter.xyz;
    float b = dot(oc, dir);
    float cOut = dot(oc, oc) - u.layer.y * u.layer.y;
    float discOut = b * b - cOut;
    if (discOut < 0.0) return false;
    float sOut = sqrt(discOut);
    float outerFar = -b + sOut;
    if (outerFar < 0.0) return false;
    t0 = max(-b - sOut, 0.0);
    t1 = outerFar;
    float cIn = dot(oc, oc) - u.layer.x * u.layer.x;
    float discIn = b * b - cIn;
    if (discIn > 0.0) {
        float innerNear = -b - sqrt(discIn);
        if (innerNear > 0.0) t1 = min(t1, innerNear);
    }
    return t1 > t0;
}

void main() {
    const vec4 CLEAR = vec4(0.0, 0.0, 0.0, 1.0);

    vec2 ndc = inUV * 2.0 - 1.0;   // flipped invVP: no manual y negation
    vec4 world = u.invViewProjection * vec4(ndc, 1.0, 1.0);
    vec3 camPos = u.cameraPosition.xyz;
    vec3 dir = normalize(world.xyz / world.w - camPos);
    vec3 sunDir = normalize(u.sunDirection.xyz);

    float t0, t1;
    if (!clLayerInterval(camPos, dir, t0, t1)) { outColor = CLEAR; return; }

    // Scene depth occlusion (reverse-Z: background depth == 0 -> far).
    float depth = texture(sceneDepth, inUV).r;
    float tScene = u.march.w;
    if (depth > 0.0) {
        vec4 w = u.invViewProjection * vec4(ndc, depth, 1.0);
        tScene = length(w.xyz / w.w - camPos);
    }
    t1 = min(t1, tScene);
    if (t1 <= t0) { outColor = CLEAR; return; }

    // EMPTY-RAY EARLY-OUT: probe the weather field along the slab span before
    // paying for the march — a fair sky is mostly gaps by design.
    {
        float covA = clLocalCoverage(clWeather((camPos + dir * t0).xz));
        float covB = clLocalCoverage(clWeather((camPos + dir * mix(t0, t1, 0.5)).xz));
        float covC = clLocalCoverage(clWeather((camPos + dir * t1).xz));
        if (max(covA, max(covB, covC)) < 0.02) { outColor = CLEAR; return; }
    }

    int viewSteps = int(u.march.x);
    int lightSteps = int(u.march.y);
    // TWO STEP SIZES: the budget step is what the march affords across the
    // traversal; the FINE step (tied to the layer's own thickness) is what
    // resolves a cloud — grazing rays otherwise sample billows a handful of
    // times and the dither turns every mid-distance deck into weave.
    float budgetStep = (t1 - t0) / float(max(2, viewSteps));
    float slabThick = max(1.0, u.layer.y - u.layer.x);
    float dsNear = min(budgetStep, slabThick / 12.0);
    float ds = dsNear;
    float mu = dot(dir, sunDir);
    float phase = clPhaseHG(mu, u.march.z);

    // Per-pixel start jitter breaks the shells of constant t; a decorrelated
    // offset for the sun march stops shadow slices locking to view slices.
    float jitter = clDither(gl_FragCoord.xy);
    float lightJitter = fract(clIgn(gl_FragCoord.xy) + 0.5);
    // High-frequency erosion budget: full detail while the step resolves the
    // detail cells, faded out past Nyquist.
    float detailCell = 1.0 / max(1e-6, 16.0 * u.params.z);
    float detailFade = 1.0 - smoothstep(0.35 * detailCell, 1.1 * detailCell, ds);

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    vec3 sunLight = u.sunColor.rgb * u.sunColor.w;

    // ZERO-RUN STRETCH: consecutive empty samples grow the stride so gaps
    // between banks are crossed cheaply; any hit resets to the honest step.
    float t = t0 + ds * jitter;
    int dryRun = 0;
    for (int i = 0; i < viewSteps && t < t1; i++) {
        bool stretched = dryRun >= 3;
        float stepLen = stretched
            ? min(ds * (1.0 + float(dryRun)), max(budgetStep, ds * 12.0))
            : ds;
        vec3 p = camPos + dir * t;
        // Detail falls away with distance — erosion finer than the pixel
        // footprint can only alias; far banks keep their SHAPE.
        float farFade = 1.0 - smoothstep(3500.0, 14000.0, t);
        float density = clDensity(p, detailFade * farFade);
        if (density <= 0.001) {
            dryRun += 1;
            t += stepLen;
        } else if (stretched && i < viewSteps - 8) {
            // ENTRY REFINEMENT: a stretched step found cloud, so the bank's
            // face lies somewhere in the stride behind — rewind to the last
            // known-empty sample and re-enter at the honest step. The budget
            // guard stops this eating the step budget in a field of wisps.
            dryRun = 0;
            t = max(t0, t - stepLen);
        } else {
            dryRun = 0;
            // Never integrate a stretched stride's worth — a search step
            // treated as a uniform slab paints a plate through the bank.
            float integrateLen = min(stepLen, ds);
            t += integrateLen;
            stepLen = integrateLen;
            // Sun march over a FIXED SHORT SPAN (240 m), not the whole slab —
            // marching the full slab put lightOD ~64 and exp(-64) == 0, so
            // cloud interiors were lit purely by ambient and never read white.
            float slab = max(1.0, u.layer.y - u.layer.x);
            float lightSpan = min(slab, 240.0);
            float lds = lightSpan / float(max(1, lightSteps));
            float lightOD = 0.0;
            for (int j = 0; j < lightSteps; j++) {
                vec3 lp = p + sunDir * (lds * (float(j) + lightJitter));
                lightOD += clDensity(lp, detailFade * farFade * 0.5) * lds;
            }
            float beer = exp(-lightOD);
            float powder = 1.0 - exp(-2.0 * density * stepLen);
            // Ambient grades with height: bases sit in their own shadow, tops
            // face the open sky — gives the deck its underside.
            float hf = clLayerHeight(p);
            vec3 ambient = u.skyAmbient.rgb * (0.5 + 0.5 * hf);
            // ANALYTIC in-scatter for the step: source * (1 - exp(-sigma*ds)).
            // The rectangle rule (source * density * ds) overshoots by the
            // per-step optical depth and stacked into the horizon white wall.
            vec3 lum = sunLight * beer * phase * powder + ambient;
            float stepT = exp(-density * stepLen);
            scattered += transmittance * lum * (1.0 - stepT);
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    // Distant clouds sit behind kilometres of atmosphere: contrast washes
    // toward the sky (the cloud overlay composites after the scene's own
    // aerial perspective, so the deck needs its own), and the deck ends
    // SOFTLY at the march clamp instead of on a crisp entry ring.
    float aerial = smoothstep(5000.0, 0.85 * u.march.w, t0) * 0.7;
    scattered = mix(scattered, u.skyAmbient.rgb * 1.15, aerial);
    transmittance = mix(transmittance, 1.0, aerial * 0.35);

    float ringFade = 1.0 - smoothstep(0.75 * u.march.w, u.march.w, t0);
    scattered *= ringFade;
    transmittance = mix(1.0, transmittance, ringFade);
    outColor = vec4(scattered, transmittance);
}
