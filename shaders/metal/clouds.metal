// Volumetric clouds — Metal port of the Vulkan clouds.frag (procedural-planet-plan
// P4), WIRED by the cinematic-sky phase. A shared density + lighting + march core
// over two domains selected by uni.layer.w: 0 = SLAB (the cloudscape over a ground
// scene — the procgen city/terrain sky, depth-occluded) and 1 = SHELL (a planet's
// cloud deck from space). Density samples the tileable 3D Perlin-Worley texture
// set baked CPU-side (src/renderer/metal/cloud_noise.h, Schneider/Nubis-style:
// 128^3 base shape + 32^3 detail erosion — this replaced the inline value-noise
// fbm whose thresholded blobs read as small potatoes, not cumulus); lighting is
// a sun light-march (Beer + Powder) with a Henyey-Greenstein phase. Output is
// scene-linear HDR; the composite pass tone maps.
//
// Wiring (metal_renderer.mm): fragmentClouds renders the layer at HALF resolution
// into an offscreen RGBA16F target as a premultiplied overlay — rgb = in-scattered
// radiance, a = transmittance — reading only the scene DEPTH (reverse-Z) for
// occlusion. fragmentCloudComposite then upsamples it bilaterally (depth-guided,
// via cloudOverlaySample) over the full-res HDR scene with One + SourceAlpha
// blending; the composite pass reuses cloudOverlaySample for sky pixels, which it
// re-derives analytically. CloudUniforms lives in shader_types.h (shared C++/MSL).

// The baked Perlin-Worley set tiles, so repeat addressing is the whole wrap
// story; linear filtering does the octave smoothing the ALU fbm used to pay
// for. Bound by the march pass at texture(1)/(2).
constexpr sampler cl_noiseSamp(coord::normalized, filter::linear, address::repeat);

static float cl_remap(float x, float a, float b, float c, float d) {
    return c + (x - a) / max(b - a, 1e-5) * (d - c);
}

// The march's start-offset dither, STRATIFIED FOR THE 2x2 UPSAMPLE. The cloud
// pass runs at half resolution and is resolved by a 4-tap bilateral filter, so
// the four half-res texels feeding one full-res pixel are the only samples
// available to average. An ordered 2x2 pattern gives those four texels four
// DISTINCT offsets spanning [0,1) — a proper stratified estimate that the
// upsample cancels almost exactly. White noise (the original sin-hash) clumps,
// and interleaved gradient noise is tuned for a wider filter footprint; both
// leave residue the eye reads as weave. The tiny IGN term breaks the residual
// grid where the deck is thin, without unstratifying the four taps.
static float cl_dither(float2 pixel) {
    int2 c = int2(pixel) & 1;
    // Bayer 2x2 order {0, 2, 3, 1} / 4, sample-centred by an eighth.
    const float kOrder[4] = {0.0, 0.5, 0.75, 0.25};
    float bayer = kOrder[c.y * 2 + c.x] + 0.125;
    return bayer;
}

// INTERLEAVED GRADIENT NOISE (Jimenez) — the march's start-offset dither.
// A `fract(sin(dot(p, ...)))` hash was here, and white noise is the wrong
// tool: its samples clump, so at half resolution the bilateral upsample
// averages uneven neighbourhoods and the deck reads as SPARKLE rather than
// as a smooth surface. IGN is built to be locally well-distributed — every
// small neighbourhood carries a near-uniform spread of offsets — which is
// exactly what a 2x2 upsample needs to cancel the dither instead of blotching
// it. Same cost, and it also degrades gracefully where the deck is dense.
static float cl_ign(float2 pixel) {
    return fract(52.9829189 *
                 fract(dot(pixel, float2(0.06711056, 0.00583715))));
}

static float cl_layerHeight(float3 p, constant CloudUniforms& u) {
    float h = (u.layer.w < 0.5) ? p.y : (length(p - u.planetCenter.xyz) - u.layer.z);
    return clamp((h - u.layer.x) / max(1e-4, u.layer.y - u.layer.x), 0.0, 1.0);
}

// The WEATHER MAP (WS4, "dense repeating pattern... sparser looking sky"):
// a kilometres-scale coverage field over the deck. Without it, coverage is
// one global threshold and the whole sky is a uniform billow field — the base
// texture tiles every 4/noiseScale metres, so the horizon showed 8+ copies of
// the same pattern. The field reuses the base texture's perlin-worley R at a
// scale ~140x larger than the billows: its features span ~10 km (formations
// and true gaps) and its tile period (~40 km) sits beyond the march distance,
// so no repetition is visible at all. Pinned to the noiseScale knob so
// formations scale WITH the puffs.
static float cl_weather(float2 xz, constant CloudUniforms& u,
                        texture3d<float> baseNoise) {
    float2 q = xz * (u.params.z * 0.035);
    // Fixed mid-slice: the field must not swim as the ray climbs the slab.
    return baseNoise.sample(cl_noiseSamp, float3(q.x, 0.37, q.y)).r;
}

// Coverage after the weather map: fair skies carve banks with real gaps,
// storm coverage saturates the field into a full deck.
static float cl_localCoverage(float weather, constant CloudUniforms& u) {
    return clamp(u.params.x * (0.30 + 1.4 * weather), 0.0, 1.0);
}

// `detailFade` scales the high-frequency erosion (1 = full, 0 = shape only).
// The detail texture's finest worley cells are ~90 m across at the shipped
// noiseScale, so a march step longer than about half that samples them below
// Nyquist and turns cauliflower into per-pixel confetti — the speckle over
// every distant bank. The march passes the fade its CURRENT step earns, so
// near clouds keep their crisp edges and far ones lose the aliasing, not the
// silhouette (the base shape, sampled at 8x lower frequency, is untouched).
static float cl_density(float3 p, constant CloudUniforms& u,
                        texture3d<float> baseNoise, texture3d<float> detailNoise,
                        float detailFade = 1.0) {
    float hf = cl_layerHeight(p, u);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    float profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);
    float3 wind = float3(u.params.w * u.skyAmbient.w, 0.0, 0.0);
    float3 q = (p + wind) * u.params.z;
    // Base shape: R = perlin-worley, GBA = worley fBm at rising frequency.
    // The 0.25 uv scale puts the texture's freq-4 perlin cell at ~1/noiseScale
    // metres — the same feature scale the old value fbm gave params.z, so
    // levels keep their meaning. The worley fBm remap carves the billows.
    float4 nse = baseNoise.sample(cl_noiseSamp, q * 0.25);
    float lowFbm = nse.g * 0.625 + nse.b * 0.25 + nse.a * 0.125;
    float base = clamp(cl_remap(nse.r, lowFbm - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    // Coverage: the authored knob modulated by the weather field at THIS
    // column — threshold + renormalise semantics unchanged per-locale.
    float cov = cl_localCoverage(cl_weather(p.xz, u, baseNoise), u);
    float d = clamp((base - (1.0 - cov)) / max(1e-3, cov), 0.0, 1.0);
    // Edge erosion: subtract high-frequency worley scaled by (1 - d), so
    // interiors stay solid and edges wisp away (the standard erode trick).
    float3 det = detailNoise.sample(cl_noiseSamp, q * 2.0).rgb;
    float detailFbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
    d = clamp(d - detailFbm * u.detail.x * detailFade * (1.0 - d), 0.0, 1.0);
    return d * profile * u.params.y;
}

// Henyey-Greenstein, normalised so ISOTROPIC (g = 0) RETURNS 1 rather than
// 1/4pi. The 1/4pi belongs to a phase function integrated over the sphere; here
// the result multiplies a directional sun term that carries no compensating
// 4pi, so keeping it dimmed every lit sample by ~12.6x and the deck could only
// be recovered by pushing ambient up — which is what made it read flat and grey.
static float cl_phaseHG(float mu, float g) {
    float g2 = g * g;
    return (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

static bool cl_layerInterval(float3 origin, float3 dir, constant CloudUniforms& u,
                             thread float& t0, thread float& t1) {
    if (u.layer.w < 0.5) {
        if (fabs(dir.y) < 1e-5) {
            if (origin.y < u.layer.x || origin.y > u.layer.y) return false;
            t0 = 0.0; t1 = u.march.w; return true;
        }
        float ta = (u.layer.x - origin.y) / dir.y;
        float tb = (u.layer.y - origin.y) / dir.y;
        t0 = max(min(ta, tb), 0.0);
        t1 = max(ta, tb);
        return t1 > 0.0;
    }
    float3 oc = origin - u.planetCenter.xyz;
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

struct CloudsOut {
    float4 position [[position]];
    float2 uv;
};

vertex CloudsOut vertexClouds(uint vid [[vertex_id]]) {
    CloudsOut out;
    float2 uv = float2((vid << 1) & 2, vid & 2);
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    // Flip Y to Metal's texture origin (top-left), exactly like vertexAtmosphere
    // — the fragment's ndc reconstruction does `-(uv.y*2-1)` and the depth
    // sample both assume this y-down uv. Without the flip the ray Y is inverted
    // and the cloud deck mirrors vertically as the camera pitches.
    out.uv = float2(uv.x, 1.0 - uv.y);
    return out;
}

// Half-resolution cloud march. Premultiplied overlay out: rgb = in-scattered
// radiance, a = view transmittance (1 = no cloud). Composite: scene*a + rgb.
fragment float4 fragmentClouds(CloudsOut in [[stage_in]],
                               depth2d<float> sceneDepth [[texture(0)]],
                               texture3d<float> baseNoise [[texture(1)]],
                               texture3d<float> detailNoise [[texture(2)]],
                               constant CloudUniforms& u [[buffer(0)]]) {
    constexpr sampler samp(coord::normalized, filter::linear, address::clamp_to_edge);
    const float4 CLEAR = float4(0.0, 0.0, 0.0, 1.0);

    float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
    float4 world = u.invViewProjection * float4(ndc, 1.0, 1.0);
    float3 camPos = u.cameraPosition.xyz;
    float3 dir = normalize(world.xyz / world.w - camPos);
    float3 sunDir = normalize(u.sunDirection.xyz);

    float t0, t1;
    if (!cl_layerInterval(camPos, dir, u, t0, t1)) return CLEAR;

    // Scene depth occlusion (reverse-Z: background depth == 0 -> far).
    float depth = sceneDepth.sample(samp, in.uv);
    float tScene = u.march.w;
    if (depth > 0.0) {
        float4 w = u.invViewProjection * float4(ndc, depth, 1.0);
        tScene = length(w.xyz / w.w - camPos);
    }
    t1 = min(t1, tScene);
    if (t1 <= t0) return CLEAR;

    // EMPTY-RAY EARLY-OUT (WS4): probe the weather field along the slab span
    // before paying for the march. A clear/fair sky is now MOSTLY gaps by
    // design — without this, every empty pixel still ran the full march
    // (40 steps x 2 fetches), which is why a sparse sky cost as much as a
    // storm. Three probes across the interval; if the field is dry at all of
    // them, there is nothing to march through.
    {
        float covA = cl_localCoverage(
            cl_weather((camPos + dir * t0).xz, u, baseNoise), u);
        float covB = cl_localCoverage(
            cl_weather((camPos + dir * mix(t0, t1, 0.5)).xz, u, baseNoise), u);
        float covC = cl_localCoverage(
            cl_weather((camPos + dir * t1).xz, u, baseNoise), u);
        if (max(covA, max(covB, covC)) < 0.02) return CLEAR;
    }

    int viewSteps = int(u.march.x);
    int lightSteps = int(u.march.y);
    // TWO STEP SIZES, because one cannot serve both jobs. The budget step
    // (interval / viewSteps) is what the march can afford across the whole
    // traversal; the FINE step is what actually resolves a cloud. They only
    // agree overhead: at 8 degrees of elevation the slab traversal is 5 km and
    // the budget step is 126 m, at 3 degrees it is 334 m — against billows
    // ~200-900 m across. A bank then got a handful of samples, and the dither
    // decided WHICH features each pixel happened to hit, which is the noisy
    // weave over every mid-distance deck. The fine step is tied to the layer's
    // own thickness instead of to how far the ray happens to travel, so a
    // cloud is sampled like a cloud from any angle.
    float budgetStep = (t1 - t0) / float(max(2, viewSteps));
    float slabThick = max(1.0, u.layer.y - u.layer.x);
    // slab/12: the near-field step, tuned BY LOOKING. slab/9 was tried and
    // reverted — it measured the same cost at street level and put visible
    // slice layering back into every bank, which is the artifact this whole
    // change exists to remove. Sampling finer than the layer's own thickness
    // is what makes a march read as a volume instead of as a stack of plates.
    float dsNear = min(budgetStep, slabThick / 12.0);
    // ...and the fine step GROWS WITH DISTANCE inside the loop. A cloud two
    // kilometres away covers a handful of half-res pixels, so resolving it at
    // the near-field step buys detail no pixel can show while paying full
    // march cost — a closed storm deck ran 24 ms that way. Scaling the step
    // with the pixel footprint is the same trade the mip pyramid makes.
    float ds = dsNear;
    float mu = dot(dir, sunDir);
    float phase = cl_phaseHG(mu, u.march.z);

    // Per-pixel start jitter breaks up the shells of constant t that a fixed
    // step grid samples — without it the march paints those shells onto the
    // deck as stripes wherever a bank's face lies tangent to one, which is
    // the banding "going up the cloud". IGN, not a sin-hash: see cl_ign.
    float jitter = cl_dither(in.position.xy);
    // A second, decorrelated offset for the sun march. Sharing one jitter
    // would lock the shadow slices to the view slices and re-draw the same
    // stripes in the lighting. Golden-ratio rotation is the cheap decorrelator.
    float lightJitter = fract(cl_ign(in.position.xy) + 0.5);
    // How much high-frequency erosion this ray can afford (see cl_density).
    // The detail texture's finest cells are 1/(16*noiseScale) metres; keep
    // full detail while the step resolves them and fade out past Nyquist.
    float detailCell = 1.0 / max(1e-6, 16.0 * u.params.z);
    float detailFade = 1.0 - smoothstep(0.35 * detailCell, 1.1 * detailCell, ds);
    // Fine steps resolve detail again, so this now mostly stays near 1 — it
    // remains the guard for levels that author a coarse budget or a huge slab.

    float transmittance = 1.0;
    float3 scattered = float3(0.0);
    float3 sunLight = u.sunColor.rgb * u.sunColor.w;

    // ZERO-RUN STRETCH: after a few consecutive empty samples the step grows,
    // so rays crossing gaps between banks skip the emptiness cheaply; any hit
    // resets to the honest step. `t` replaces the i*ds grid.
    float t = t0 + ds * jitter;
    int dryRun = 0;
    for (int i = 0; i < viewSteps && t < t1; i++) {
        // Empty space is crossed with a GEOMETRICALLY growing stride (the
        // fine step is far too small to reach the horizon within the budget),
        // while anything that touches cloud integrates at the fine step. This
        // is the standard empty-space-skipping shape: cheap where there is
        // nothing, honest where there is something.
        const bool stretched = dryRun >= 3;
        float stepLen =
            stretched ? min(ds * (1.0 + float(dryRun)), max(budgetStep, ds * 12.0))
                      : ds;
        float3 p = camPos + dir * t;
        // DETAIL FALLS AWAY WITH DISTANCE — the standard treatment (Nubis /
        // Decima, Frostbite): a bank 15 km out compresses its erosion detail
        // into a couple of pixels, and detail finer than a pixel cannot
        // resolve, it can only alias. That aliasing is what reads as "just
        // noise" on the horizon. Near clouds keep every wisp; far ones keep
        // only their SHAPE, which is what the eye actually reads at that
        // distance anyway. (The step-size fade above is the same idea keyed
        // on the march; this one is keyed on the pixel footprint.)
        float farFade = 1.0 - smoothstep(3500.0, 14000.0, t);
        float density = cl_density(p, u, baseNoise, detailNoise,
                                   detailFade * farFade);
        if (density <= 0.001) {
            dryRun += 1;
            t += stepLen;
        } else if (stretched && i < viewSteps - 8) {
            // ENTRY REFINEMENT. A stretched step just found cloud, so the bank's
            // face lies anywhere in the stride behind this sample —
            // integrating from here would start the cloud up to a step late and
            // stamp that error along the whole face as a stair. Drop back to the
            // last known-empty sample and re-enter at the honest step. Costs one
            // sample per bank entered, and terminates: after the rewind at least
            // four honest steps must pass before the stretch can arm again.
            //
            // The budget guard (i < viewSteps - 8) is what stops this from
            // eating the step budget in a field of wisps. A ray that runs out
            // of steps mid-cloud stops integrating and leaves SKY showing
            // through — thin blue cuts that trace where an earlier bank spent
            // the budget, which is exactly the "cutaway lines" artifact.
            // Refining is worth a lot (denying it entirely puts slice layering
            // back into every wispy bank), so it is spent freely until the
            // budget is nearly gone and only then traded away.
            dryRun = 0;
            t = max(t0, t - stepLen);
        } else {
            dryRun = 0;
            // Out of refinement budget: integrate, but never a stretched
            // stride's worth — a search step treated as a uniform slab paints
            // a plate hundreds of metres thick through the bank.
            float integrateLen = min(stepLen, ds);
            t += integrateLen;
            stepLen = integrateLen;
            // SUN MARCH over a FIXED SHORT SPAN, not the whole slab. This used
            // to step (top - bottom) per sample — the full 700 m — from
            // wherever the sample sat, so lightOD reached ~64 and exp(-64) is
            // exactly zero: every sample inside a cloud was lit purely by
            // ambient. That, not the noise, is why the deck was never white.
            float slab = max(1.0, u.layer.y - u.layer.x);
            float lightSpan = min(slab, 240.0);
            float lds = lightSpan / float(max(1, lightSteps));
            float lightOD = 0.0;
            // The sun march is the dominant cost of a closed deck, but it can
            // only be cut where the cut cannot be SEEN. Reducing the step
            // count past a distance threshold was tried and reverted: it
            // changes the shadow estimate across a shell of constant t, and
            // that shell draws a hard seam across the sky. The saturation bail
            // below is safe for the opposite reason — it fires only where the
            // sample is already in full shadow, so it changes no pixel.
            for (int j = 0; j < lightSteps; j++) {
                float3 lp = p + sunDir * (lds * (float(j) + lightJitter));
                // The shadow march is coarse (a few steps over 240 m), so it
                // gets the same Nyquist treatment as the view ray — undithered
                // detail here only adds noise to the shading, never shape.
                lightOD += cl_density(lp, u, baseNoise, detailNoise,
                                      detailFade * farFade * 0.5) * lds;
            }
            float beer = exp(-lightOD);
            float powder = 1.0 - exp(-2.0 * density * stepLen);
            // Ambient grades with height in the layer: bases sit in their own
            // shadow, tops face the open sky — gives the deck its underside.
            float hf = cl_layerHeight(p, u);
            float3 ambient = u.skyAmbient.rgb * (0.5 + 0.5 * hf);
            // ANALYTIC in-scatter for the step. The source term is radiance and
            // must NOT be multiplied by density: the correct integral of
            // source * sigma * exp(-sigma * s) across the step is
            // source * (1 - exp(-sigma * ds)) = source * (1 - stepT).
            // The old form was source * density * ds — a rectangle rule only
            // valid for thin steps, and at density 0.55 with ds ~= 17.5 m the
            // per-step optical depth is ~9.6, so it overshot by that factor AND
            // the error grew with ds, which is precisely the horizon white wall
            // that `horizonFade` below was invented to hide.
            float3 lum = sunLight * beer * phase * powder + ambient;
            float stepT = exp(-density * stepLen);
            scattered += transmittance * lum * (1.0 - stepT);
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    // The "white glare at the horizon" this used to fade away was not a real
    // horizon effect: the old rectangle-rule in-scatter overshot by roughly the
    // per-step optical depth, and ds grows with the slant path, so grazing rays
    // overshot several times harder than overhead ones and stacked into a wall.
    // The fade hid that — at the cost of dissolving a large share of the deck
    // into blue sky even at zenith, which is the other half of "the clouds
    // don't look right". With the analytic step above, a long slant path now
    // converges to an opaque cloud instead of diverging, so the fade has
    // nothing left to hide and is gone.
    //
    // VISUAL GATE: this needs to be LOOKED at, not measured. Clouds should read
    // white and solid with no per-pixel sparkle, and the deck should continue
    // to the horizon rather than dissolving into blue.
    //
    // ...continue to the horizon and then END SOFTLY (WS4): the march clamps
    // at farDistance, so the deck used to stop on a crisp RING where the slab
    // entry crossed it (~1.7 degrees above the horizon at bottom 900 / far
    // 30000). Fade the contribution over the last quarter of the range so
    // clouds dissolve into the aerial haze the sky already paints there. This
    // is not the retired whole-deck fade: it keys on the ENTRY distance and
    // touches only the final approach to the clamp.
    // Distant clouds sit behind kilometres of atmosphere: they lose contrast
    // toward the sky, which is both physically right and the other half of
    // why a far deck stops reading as noise — aliasing that survives the
    // detail fade is washed out with the contrast. The scene's own aerial
    // perspective does this for terrain; the cloud overlay composites after
    // it, so the deck needs its own.
    float aerial = smoothstep(5000.0, 0.85 * u.march.w, t0) * 0.7;
    scattered = mix(scattered, u.skyAmbient.rgb * 1.15, aerial);
    transmittance = mix(transmittance, 1.0, aerial * 0.35);

    float ringFade = 1.0 - smoothstep(0.75 * u.march.w, u.march.w, t0);
    scattered *= ringFade;
    transmittance = mix(1.0, transmittance, ringFade);
    return float4(scattered, transmittance);
}

// Depth-guided bilateral upsample of the half-res cloud overlay at a full-res
// pixel. Weighted 4-tap: bilinear weights modulated by how well each half-res
// texel's depth matches this pixel (sky texels stay off building silhouettes
// and vice versa). Shared by the scene composite pass below and by the sky
// branch of fragmentComposite (declared there, defined here — one concatenated
// translation unit).
float4 cloudOverlaySample(float2 uv, texture2d<float> cloudTex,
                          depth2d<float> depthTex, constant CameraUniforms& camera) {
    float2 cSize = float2(cloudTex.get_width(), cloudTex.get_height());
    uint2 fullMax = uint2(depthTex.get_width() - 1, depthTex.get_height() - 1);
    float2 fullSize = float2(depthTex.get_width(), depthTex.get_height());

    float d0 = depthTex.read(min(uint2(uv * fullSize), fullMax));

    float2 pos = uv * cSize - 0.5;
    float2 f = fract(pos);
    int2 base = int2(floor(pos));
    float4 acc = float4(0.0);
    float wSum = 0.0;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 2; i++) {
            int2 c = clamp(base + int2(i, j), int2(0), int2(cSize) - 1);
            float2 cuv = (float2(c) + 0.5) / cSize;
            float dd = depthTex.read(min(uint2(cuv * fullSize), fullMax));
            // The weight is a BLEND of the bilinear weight and a flat 0.25.
            //
            // Pure bilinear leaves a weighted remainder of the march's 2x2
            // dither (the stipple). Pure EQUAL weights cancel that dither
            // exactly — and were tried, and were much worse: with flat
            // weights every full-res pixel inside the same half-res 2x2
            // window resolves to the IDENTICAL value, so the upsample paints
            // a hard 2x2 grid locked to the screen. Screen-aligned blocking
            // is far more objectionable than a faint stipple, and it is
            // exactly what it looks like — "the fragment shader gets jacked
            // up because the glitches are aligned with the screen and not the
            // scene."
            //
            // Any blend with a bilinear term varies per pixel, so the blocks
            // cannot form; leaning it toward flat still cancels most of the
            // dither. The depth term stays: it stops the deck bleeding across
            // a silhouette.
            float bw = (i != 0 ? f.x : 1.0 - f.x) * (j != 0 ? f.y : 1.0 - f.y);
            float w = mix(0.25, bw, 0.5) *
                      bilateralDepthWeight(d0, dd, camera.nearPlane,
                                           camera.farPlane, 0.10) + 1e-5;
            acc += cloudTex.read(uint2(c)) * w;
            wSum += w;
        }
    }
    return acc / wSum;
}

// Full-res composite of the half-res cloud overlay onto the HDR scene target.
// Blend state: RGB = One + SourceAlpha (dest * transmittance + scattered),
// alpha untouched. Sky pixels get the same overlay again in fragmentComposite
// (which re-derives the sky analytically); geometry pixels come from here.
fragment float4 fragmentCloudComposite(CloudsOut in [[stage_in]],
                                       texture2d<float> cloudTex [[texture(0)]],
                                       depth2d<float> depthTex [[texture(1)]],
                                       constant CameraUniforms& camera [[buffer(1)]]) {
    return cloudOverlaySample(in.uv, cloudTex, depthTex, camera);
}
