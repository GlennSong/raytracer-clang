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
    mat4 invViewProjection;   // CAMERA-RELATIVE: clip -> the offset from the camera (ADR-0335)
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunColor;        // rgb, w intensity
    vec4 skyAmbient;      // rgb ambient, w time
    vec4 planetCenter;
    vec4 layer;           // x bottom, y top, z planetRadius, w domainMode
    vec4 params;          // x coverage, y densityScale, z noiseScale, w windSpeed
    vec4 march;           // x viewSteps, y lightSteps, z phaseG, w farDistance
    vec4 detail;          // x detailStrength (edge-erosion depth), y 1: the old coherent march offset (the A/B), z 1: the resolve reprojects by cloud depth, w unused
    mat4 prevViewProjection;   // last frame's (clouds_resolve.frag: the history's reprojection)
    vec4 frame;                // x this frame's dither offset in [0,1), y history weight (0: reset), z wisps (0: the old
                               // solid edges, RT_CLOUD_WISPS=0), w the cirrus's cover (0: none) -- ADR-0384
    // THE CLOUD TYPES (ADR-0336): along the axis 0 stratus, 1/3 stratocumulus, 2/3 cumulus, 1 cumulonimbus
    vec4 typeProfile[4];       // the height fractions: rise from x, full by y, full to z, gone by w
    vec4 typeParams[4];        // x density, y erosion, z anvil
    vec4 typeRange;            // x lo, y hi: the live weather's types
    vec4 windOffset;           // xy: the clouds' drift so far along the weather's wind (x, z m: ADR-0337)
    // THE WEATHER MAP (ADR-0386): each type's heights, the map's square in the air, the shape noise's scale
    vec4 typeHeights[4];       // x base, y top (m; top <= base: the deck's span)
    vec4 mapRect;              // x originX, y originZ, z 1 / extent, w 1: the map places the clouds
    vec4 shape;                // x the shape noise's frequency against noiseScale, yz the cirrus's sweep (x, z)
} u;

layout(set = 0, binding = 1) uniform sampler2D sceneDepth;
layout(set = 0, binding = 2) uniform sampler3D baseNoise;    // repeat, linear
layout(set = 0, binding = 3) uniform sampler3D detailNoise;  // repeat, linear
layout(set = 0, binding = 5) uniform sampler2D weatherMap;   // R coverage, G cloud type, B wetness (ADR-0336)

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;
// EACH PIXEL'S CLOUD DEPTH (ADR-0334): the distance to the cloud its ray found, weighted by what each step added to the
// pixel; -1 where it found none. The history reprojects by it (clouds_resolve.frag) -- by the deck's middle height it
// pasted last frame's clouds in the wrong place whenever the camera was near or above the deck, and they slid.
layout(location = 1) out float outCloudDepth;

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
// THE WIND moves the WHOLE field (#59, Glenn: "the clouds still wobble"): only the billow noise used to drift,
// through a coverage map pinned to the ground, so every cloud stood still while its insides churned through it --
// clouds that boil and wobble in place. Now the weather map, the shape and the detail all ride the same wind:
// a cloud drifts across the sky as one body; the detail runs a little ahead, so its edges still evolve, slowly.
vec3 clWind() { return vec3(u.windOffset.x, 0.0, u.windOffset.y); }   // (the weather's wind, integrated: ADR-0337)

// THE WEATHER MAP (ADR-0336): x the coverage field, y the cloud type at this point (the map's G spread over the live
// weather's type range). Its own 2D texture, R the slice of the base volume the march used to read, so the sky keeps
// its layout.
// THE WEATHER MAP THAT PLACES THE CLOUDS (ADR-0386; engine/sky/cloud_map.h): x the footprint (the coverage here: 0
// clear air .. 1 a cloud's heart), y the cell's type slot, z its own top as a fraction of its type's height; zero
// past the map's square. Laid out in the air: the world plus the drift.
vec3 clMap(vec2 xz) {
    vec2 uv = (xz + clWind().xz - u.mapRect.xy) * u.mapRect.z;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) return vec3(0.0);
    return textureLod(weatherMap, uv, 0.0).rgb;
}
bool clMapOn() { return u.mapRect.w > 0.5; }

// The type's shape at this height: x density from its profile, y its density scale, z erosion, w anvil -- the four
// types blended along the axis by where `type` falls between neighbours
vec4 clTypeShape(float type, float hf) {
    const float a = clamp(type, 0.0, 1.0) * 3.0;
    const int i = min(int(a), 2);
    const float f = a - float(i);
    const vec4 P = mix(u.typeProfile[i], u.typeProfile[i + 1], f);
    const vec4 Q = mix(u.typeParams[i], u.typeParams[i + 1], f);
    const float prof = smoothstep(P.x, max(P.y, P.x + 1e-3), hf) * (1.0 - smoothstep(P.z, max(P.w, P.z + 1e-3), hf));
    return vec4(prof, Q.x, Q.y, Q.z);
}

// A map cell's type at height y: x its profile's density there, y the type's density, z its erosion, w its anvil; and
// `hf` the height within the cell (0 its base, 1 its own top). The slot is the cell's own (the map's G), between two
// types only where the bilinear read crosses cells; its top shrinks with the cell (the map's B).
vec4 clCellShape(float slot, float topFrac, float y, out float hf) {
    const float a = clamp(slot, 0.0, 1.0) * 3.0;
    const int i = min(int(a), 2);
    const vec2 H = mix(u.typeHeights[i].xy, u.typeHeights[i + 1].xy, a - float(i));
    const float top = H.x + (H.y - H.x) * clamp(topFrac, 0.05, 1.0);
    hf = (y - H.x) / max(1.0, top - H.x);
    if (hf <= 0.0 || hf >= 1.0) return vec4(0.0);
    return clTypeShape(slot, hf);   // (the same blend of the types' profiles, at the cell's own height)
}

vec2 clWeather(vec2 xz) {
    if (clMapOn()) {
        const vec3 m = clMap(xz);
        return vec2(m.x, m.y);
    }
    vec2 q = (xz + clWind().xz) * (u.params.z * 0.035);
    if (u.detail.w > 0.5) return vec2(textureLod(baseNoise, vec3(q.x, 0.37, q.y), 0.0).r, 2.0 / 3.0);   // (RT_CLOUD_TYPES=0: the old slice, cumulus)
    const vec4 w = textureLod(weatherMap, q, 0.0);
    return vec2(w.r, mix(u.typeRange.x, u.typeRange.y, w.g));
}
// Coverage after the weather map: fair skies carve banks with real gaps,
// storm coverage saturates the field into a full deck.
float clLocalCoverage(float weather) {
    // LOW coverage is gated by the weather map, not just scaled: the threshold
    // rides the top of a noise field, and 3 % still peppered the whole sky
    // with popcorn. Under ~0.25 only the field's peaks may hold cloud, so a
    // clear day is a few isolated puffs; fair and up are unchanged.
    float gate = mix(smoothstep(0.55, 0.85, weather), 1.0, smoothstep(0.05, 0.25, u.params.x));
    return clamp(u.params.x * (0.30 + 1.4 * weather) * gate, 0.0, 1.0);
}

// `detailFade` scales the high-frequency erosion (1 = full, 0 = shape only) —
// steps longer than the detail texture's cells sample it below Nyquist and
// turn cauliflower into confetti, so far samples keep shape, not noise.
// `coarse` (0..1): past where the fine erosion fades (the far banks), how much of the coarse one takes its place
float clDensity(vec3 p, float detailFade, float coarse) {
    float hf = clLayerHeight(p);
    if (hf <= 0.0 || hf >= 1.0) return 0.0;
    // the weather here, and its cloud type's shape at this height (ADR-0336) -- with the map, the cell's own
    // (ADR-0386): its footprint IS the coverage, its type and its height its own
    vec4 shape;
    float cov;
    if (clMapOn()) {
        const vec3 m = clMap(p.xz);
        if (m.x <= 0.004) return 0.0;
        shape = clCellShape(m.y, m.z, p.y, hf);
        cov = m.x;
    } else {
        const vec2 wx = clWeather(p.xz);
        shape = clTypeShape(wx.y, hf);
        cov = clLocalCoverage(wx.x);
    }
    float profile = shape.x;
    if (profile <= 0.0) return 0.0;
    vec3 wind = clWind();
    vec3 q = (p + wind) * (u.params.z * max(u.shape.x, 1e-3));
    // Base shape: R = perlin-worley, GBA = worley fBm at rising frequency.
    vec4 nse = textureLod(baseNoise, q * 0.25, 0.0);   // (explicit: one level, and the march's derivatives are undefined)
    float lowFbm = nse.g * 0.625 + nse.b * 0.25 + nse.a * 0.125;
    float base = clamp(clRemap(nse.r, lowFbm - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    // THE HEIGHT PROFILE BEFORE THE COVERAGE (Nubis, with the map: ADR-0386): the noise thinned toward the cell's base
    // and top, then cut by the footprint -- only the footprint's heart survives up there, so a cell rounds into a
    // dome. Applied after, the cut was the same at every height: each cell stood as a cylinder, flat-sided.
    if (clMapOn()) {
        base *= profile;
        profile = 1.0;
        // ...and the footprint NARROWED as the cell rises (a cumulus is widest at its base and domes over, not a
        // footprint stood up as a wall: Glenn, 2026-11-08, "the clouds still look boxy"): the soft footprint cut back
        // by its height, so the sides lean in and the top is the heart's alone
        const float narrow = 0.8 * smoothstep(0.0, 1.0, hf);
        cov = clamp((cov - narrow) / max(1e-3, 1.0 - narrow), 0.0, 1.0);
    }
    // the ANVIL (Nubis): near the top a cumulonimbus's coverage grows, so the tower spreads into a shelf
    cov = pow(cov, clRemap(clamp(hf, 0.7, 0.8), 0.7, 0.8, 1.0, mix(1.0, 0.5, shape.w)));
    float d = clamp((base - (1.0 - cov)) / max(1e-3, cov), 0.0, 1.0);
    // Edge erosion: subtract high-frequency worley scaled by (1 - d), so
    // interiors stay solid and edges wisp away.
    vec3 dq = q + wind * (0.08 * u.params.z) + vec3(0.0, 0.002 * u.skyAmbient.w * u.params.z, 0.0);
    if (u.frame.z <= 0.0) {
        vec3 det = textureLod(detailNoise, dq * 2.0, 0.0).rgb;
        float detailFbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
        d = clamp(d - detailFbm * u.detail.x * shape.z * detailFade * (1.0 - d), 0.0, 1.0);
        return d * profile * shape.y * u.params.y;
    }
    // WISPS (ADR-0384; Glenn, 2026-11-08: "the clouds are very solid looking and lack any of the wispiness of real
    // clouds"). Worley erosion alone cuts cauliflower, never a torn edge. As Schneider's Nubis (2015/2017):
    // - the detail's lookup SWIRLED by a low-frequency warp (a ~90 m-celled read of the same volume standing in for
    //   his curl texture), most at the base, where cumulus tears into strands, least at the billowing top;
    // - the erosion WISPY at the base and BILLOWY above (the fBm, inverted over the lowest tenth);
    // - and as a remap, so a thin edge is eaten whole, not only thinned (a smoothstep over the result as well, to make
    //   the fringe translucent, only blurred the clouds: tried and dropped);
    // - the far banks, past the fine detail, eroded by the warp's own coarse read, so they keep a ragged outline.
    float erode = 0.0;
    if (detailFade > 0.0 || coarse > 0.0) {
        vec3 wv = textureLod(detailNoise, dq * 0.5 + vec3(0.31, 0.17, 0.53), 0.0).rgb;
        if (detailFade > 0.0) {
            vec3 warp = (wv - 0.5) * 2.0 * (0.15 * u.frame.z) * (1.0 - 0.6 * hf);
            vec3 det = textureLod(detailNoise, (dq + warp) * 2.0, 0.0).rgb;
            float fbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
            fbm = mix(fbm, 1.0 - fbm, clamp(hf * 10.0, 0.0, 1.0));
            erode += fbm * detailFade;
        }
        erode += (wv.r * 0.625 + wv.g * 0.25 + wv.b * 0.125) * coarse * 0.8;
    }
    float e = clamp(erode * u.detail.x * shape.z * 1.8, 0.0, 0.95);
    d = clamp((d - e) / (1.0 - e), 0.0, 1.0);
    return d * profile * shape.y * u.params.y;
}


// Henyey-Greenstein, normalised so isotropic (g = 0) returns 1 — the result
// multiplies a directional sun term that carries no compensating 4pi.
float clPhaseHG(float mu, float g) {
    float g2 = g * g;
    return (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

// Two lobes when the wisps are on (ADR-0384): the cloud's own and a sharp forward one -- the bright rim of a cloud
// edge in front of the sun (the silver lining), which one broad lobe could not give
float clPhase(float mu, float g) {
    float p = clPhaseHG(mu, g);
    return u.frame.z > 0.0 ? mix(p, clPhaseHG(mu, 0.8), 0.1) : p;
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

// THE CIRRUS (ADR-0384): the high ice cloud the deck never had -- mare's tails and veils, most of a fair sky's
// wispiness. A thin sheet at 9 km, not marched: broad veils of the base volume read as a plane, stretched along one
// sweep, faintly streaked by a finer read (fine fibres alone came out as scratches); drifting with the deck's wind, faster (the
// jet stream). Thin (at most half opaque), lit by the sun's forward-scattering ice and the sky. Over the slab only;
// `cover` (u.frame.w) from the level's "cirrus" (0: none). Returns premultiplied rgb and its opacity.
vec4 clCirrus(vec3 camPos, vec3 dir, vec3 sunDir) {
    const float kHeight = 9000.0;
    if (u.frame.w <= 0.0 || u.layer.w > 0.5 || dir.y < 0.02 || camPos.y > kHeight) return vec4(0.0);
    float t = (kHeight - camPos.y) / dir.y;
    vec3 P = camPos + dir * t;
    float c, fade;
    if (clMapOn()) {
        // the VEILS from the weather map's A (ADR-0386: laid out with the clouds, nothing tiling), faded before the
        // map's edge, faintly streaked along its sweep by a finer read
        const vec2 a = u.shape.yz;
        const vec2 xz = P.xz + u.windOffset.xy;
        const vec2 uv = vec2(dot(xz, a), dot(xz, vec2(-a.y, a.x)));
        const vec2 m = (xz - u.mapRect.xy) * u.mapRect.z;
        const float veil = any(lessThan(m, vec2(0.0))) || any(greaterThan(m, vec2(1.0))) ? 0.0 : textureLod(weatherMap, m, 0.0).a;
        const float fib = textureLod(baseNoise, vec3(uv.x / 12000.0, 0.29, uv.y / 900.0), 0.0).r;
        c = veil * (0.55 + 0.45 * fib);
        fade = 1.0 - smoothstep(24000.0, 38000.0, length(P.xz - camPos.xz));
    } else {
        vec2 xz = P.xz + u.windOffset.xy * 2.5;
        const vec2 a = vec2(0.8, 0.6);                               // the sweep the streaks lie along
        vec2 uv = vec2(dot(xz, a), dot(xz, vec2(-a.y, a.x)));
        vec4 pn = textureLod(baseNoise, vec3(uv.x / 24000.0, 0.61, uv.y / 9000.0), 0.0);       // the veils
        float n = pn.r * 0.6 + pn.g * 0.25 + pn.b * 0.15;
        float fib = textureLod(baseNoise, vec3(uv.x / 12000.0, 0.29, uv.y / 900.0), 0.0).r;    // the streaks in them
        float cover = u.frame.w;
        c = smoothstep(1.0 - cover, 1.0 - cover + 0.45, n) * (0.6 + 0.4 * fib);
        fade = 1.0 - smoothstep(60000.0, 140000.0, t);
    }
    float alpha = c * 0.5 * fade;
    vec3 sunLight = u.sunColor.rgb * u.sunColor.w;
    vec3 lit = sunLight * (0.6 * clPhase(dot(dir, sunDir), 0.6)) + u.skyAmbient.rgb;
    return vec4(lit * alpha, alpha);
}

void main() {

    vec2 ndc = inUV * 2.0 - 1.0;   // flipped invVP: no manual y negation
    // THE RAY, from a point ~100 m out (reverse-Z depth 0.002, a few hundred near planes), not the near plane itself: there
    // the point minus the camera is a ~10 cm vector made of two ~10 km world positions in float32, and with the island's
    // 35 km far plane the inverse projection's own rounding on top -- every pixel's direction snapped to a coarse grid, and
    // far clouds came out as cubes (ADR-0332 addendum; Glenn: "the clouds look blocky"). The lab's 8 km far plane hid it.
    vec4 world = u.invViewProjection * vec4(ndc, 0.002, 1.0);
    vec3 camPos = u.cameraPosition.xyz;
    vec3 dir = normalize(world.xyz / world.w);   // (already relative to the camera)
    vec3 sunDir = normalize(u.sunDirection.xyz);

    float t0, t1;
    outCloudDepth = -1.0;
    // Scene depth occlusion (reverse-Z: background depth == 0 -> far).
    float depth = texture(sceneDepth, inUV).r;
    // the cirrus behind everything the deck holds, over the open sky only (ADR-0384)
    const vec4 cirrus = depth > 0.0 ? vec4(0.0) : clCirrus(camPos, dir, sunDir);
    const vec4 CIRRUS_ONLY = vec4(cirrus.rgb, 1.0 - cirrus.a);
    if (!clLayerInterval(camPos, dir, t0, t1)) { outColor = CIRRUS_ONLY; return; }

    float tScene = u.march.w;
    if (depth > 0.0) {
        vec4 w = u.invViewProjection * vec4(ndc, depth, 1.0);
        tScene = length(w.xyz / w.w);
    }
    t1 = min(t1, tScene);
    if (t1 <= t0) { outColor = CIRRUS_ONLY; return; }

    // EMPTY-RAY EARLY-OUT: probe the weather field along the slab span before
    // paying for the march — a fair sky is mostly gaps by design.
    // (three probes -- start, middle, end -- let a cloud smaller than half the span fall between them: the
    // ray was cleared for some pixels and marched for their neighbours, and clouds came out cut along hard
    // lines -- the "break in the rendering" Glenn saw. Eight probes, each jittered along its eighth)
    {
        float cov = 0.0;
        for (int k = 0; k < 8 && cov < 0.02; ++k) {
            float f = (float(k) + fract(clDither(gl_FragCoord.xy) + u.frame.x)) / 8.0;
            const vec2 wq = clWeather((camPos + dir * mix(t0, t1, f)).xz);
            cov = max(cov, clMapOn() ? wq.x : clLocalCoverage(wq.x));
        }
        if (cov < 0.02) { outColor = CIRRUS_ONLY; return; }
    }

    int viewSteps = int(u.march.x);
    int lightSteps = int(u.march.y);
    // TWO STEP SIZES: the budget step is what the march affords across the
    // traversal; the FINE step (tied to the layer's own thickness) is what
    // resolves a cloud — grazing rays otherwise sample billows a handful of
    // times and the dither turns every mid-distance deck into weave.
    float budgetStep = (t1 - t0) / float(max(2, viewSteps));
    float slabThick = max(1.0, u.layer.y - u.layer.x);
    // (with the map the deck spans every type the weather has, up to the towers' 9 km: a twelfth of that is no step
    // to resolve a cloud by, so the fine step stays a cloud's, ~40-120 m)
    float dsNear = min(budgetStep, clMapOn() ? clamp(slabThick / 12.0, 40.0, 120.0) : slabThick / 12.0);
    float ds = dsNear;
    float mu = dot(dir, sunDir);
    float phase = clPhase(mu, u.march.z);

    // Per-pixel start jitter breaks the shells of constant t; a decorrelated
    // offset for the sun march stops shadow slices locking to view slices.
    // ...and that pattern's four phases SHIFTED per 2x2 block by a hash: four offsets alone left every cloud edge
    // stepping between the same four shells -- iso-lines along the clouds' outlines (Glenn: "lines cut contours
    // through them"). The block keeps its own four stratified (what the 2x2 upsample cancels).
    uvec2 hb = uvec2(gl_FragCoord.xy) >> 1u;
    uint hj = hb.x * 7919u + hb.y * 104729u + 1013u;
    hj = (hj ^ (hj >> 16)) * 0x7feb352du; hj = (hj ^ (hj >> 15)) * 0x846ca68bu; hj ^= hj >> 16;
    // ...and the whole pattern rolls by a golden-ratio step every frame (u.frame.x), so the history (clouds_resolve.frag)
    // averages the start offsets away instead of the pattern standing fixed on the screen -- where, as the camera moved,
    // it crawled across the clouds (#187: the sky's frame-to-frame noise p99 3.5 still, 13.5 moving 2 m a frame)
    float jitter = fract(clDither(gl_FragCoord.xy) + 0.25 * (float(hj & 0xFFFFu) / 65536.0) + u.frame.x);
    // ...but now the history averages the march's noise away (ADR-0328), a coherent start offset is the wrong trade: whole
    // rows of pixels sampled the deck at the same distances and the far banks showed the sample shells as horizontal
    // shelves and stair-steps (ADR-0332; Glenn: "the clouds look blocky"). Each pixel takes its own offset, re-rolled
    // every frame. (u.detail.y: 1 the old coherent pattern, the A/B: RT_CLOUD_JITTER=coherent)
    if (u.detail.y < 0.5) {
        uvec2 hq = uvec2(gl_FragCoord.xy);
        uint hk = hq.x * 2654435761u ^ hq.y * 2246822519u ^ 374761393u;
        hk = (hk ^ (hk >> 16)) * 0x7feb352du; hk = (hk ^ (hk >> 15)) * 0x846ca68bu; hk ^= hk >> 16;
        jitter = fract(float(hk & 0xFFFFFFu) / 16777216.0 + u.frame.x);
    }
    // (a hash, not IGN: IGN's structure runs ~15 px along x and barely changes along y, and with no
    // temporal accumulation to average it the sun march printed soft VERTICAL COLUMNS over every
    // distant mountain seen through thin cloud)
    uvec2 hp = uvec2(gl_FragCoord.xy);
    uint hh = hp.x * 1973u + hp.y * 9277u + 26699u;
    hh = (hh ^ (hh >> 16)) * 0x7feb352du; hh = (hh ^ (hh >> 15)) * 0x846ca68bu; hh ^= hh >> 16;
    float lightJitter = fract(float(hh & 0xFFFFFFu) / 16777216.0 + u.frame.x);
    // High-frequency erosion budget: full detail while the step resolves the
    // detail cells, faded out past Nyquist.
    float detailCell = 1.0 / max(1e-6, 16.0 * u.params.z);
    float detailFade = 1.0 - smoothstep(0.35 * detailCell, 1.1 * detailCell, ds);

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    float depthSum = 0.0, depthW = 0.0;
    vec3 sunLight = u.sunColor.rgb * u.sunColor.w;

    // ZERO-RUN STRETCH: consecutive empty samples grow the stride so gaps
    // between banks are crossed cheaply; any hit resets to the honest step.
    float t = t0 + ds * jitter;
    int dryRun = 0;
    for (int i = 0; i < viewSteps && t < t1; i++) {
        // The stretch is capped at 6 steps (was 12): long strides re-entered
        // banks in plates and left horizontal shelves along mid-distance decks.
        bool stretched = dryRun >= 3;
        float stepLen = stretched
            ? min(ds * (1.0 + float(dryRun)), max(budgetStep, ds * 6.0))
            : ds;
        // THE RAY ALWAYS REACHES ITS END: what is left of it over the steps left is the least a step may be -- a ray
        // that ran out of budget was CUT, mid-deck, at a distance that depended on how many rewinds and dry runs came
        // before it, so far clouds ended along hard lines tracing the nearer ones'. Out of budget, steps coarsen.
        float leastStep = (t1 - t) / float(max(1, viewSteps - i));
        stepLen = max(stepLen, leastStep);
        // (each sample at the ray's jitter WITHIN ITS OWN STEP, not only the first: out where the steps are hundreds of
        // metres a start offset of part of the fine step left every pixel sampling the far deck at the same distances --
        // the notched rows (ADR-0332). Re-rolled every frame, the history averages the coarse steps to the right answer.)
        vec3 p = camPos + dir * (t + max(0.0, stepLen - ds) * (u.detail.y < 0.5 ? jitter : 0.0));
        // Detail falls away with distance — erosion finer than the pixel
        // footprint can only alias; far banks keep their SHAPE.
        float farFade = 1.0 - smoothstep(3500.0, 14000.0, t);
        float density = clDensity(p, detailFade * farFade, 1.0 - farFade);
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
            // ...and inside cloud, no more than ~half an optical depth a step: at the fixed slab/12 step (58 m on a
            // 700 m layer) one step through a fair-weather cloud was ~2.8 optical depths, so the first sample inside
            // took 95% and the march painted each cloud's face onto the sample shells. (Never less than the budget
            // still left allows: see leastStep.)
            float integrateLen = max(clamp(0.5 / max(density, 1e-4), ds * 0.25, min(stepLen, ds)), leastStep);
            t += integrateLen;
            stepLen = integrateLen;
            // Sun march over a FIXED SHORT SPAN (240 m), not the whole slab —
            // marching the full slab put lightOD ~64 and exp(-64) == 0, so
            // cloud interiors were lit purely by ambient and never read white.
            float slab = max(1.0, u.layer.y - u.layer.x);
            float lightSpan = min(slab, 240.0);
            float lds = lightSpan / float(max(1, lightSteps));
            float lightOD = 0.0;
            if (u.detail.y < 0.5) {   // (RT_CLOUD_JITTER=coherent: the old straight line too, for the A/B)
                // THE SUN'S CONE (Schneider 2015/2017): samples at growing distances, each pushed off the line inside a cone
                // that widens toward the sun, reading the base shape only (no detail: the light needs the cloud's bulk);
                // then one long sample for the deep shadow. Six samples on one straight line through the full-detail
                // noise streaked every low-sun cloud along the light (the "layers")
                float d0 = 0.0;
                for (int j = 0; j < lightSteps; j++) {
                    float f1 = float(j + 1) / float(lightSteps);
                    float d1 = lightSpan * f1 * f1 * 1.6;             // growing steps: ~10 m first, ~120 m last
                    float dm = mix(d0, d1, lightJitter);
                    uint hc = hh ^ (uint(j) * 0x9E3779B9u) ^ uint(u.frame.x * 65536.0);
                    hc = (hc ^ (hc >> 16)) * 0x7feb352du; hc = (hc ^ (hc >> 15)) * 0x846ca68bu; hc ^= hc >> 16;
                    vec3 rnd = vec3(float(hc & 1023u), float((hc >> 10) & 1023u), float((hc >> 20) & 1023u)) / 511.5 - 1.0;
                    vec3 lp = p + sunDir * dm + rnd * (0.35 * dm);   // the cone: widening with distance
                    lightOD += clDensity(lp, 0.0, 0.0) * (d1 - d0);
                    d0 = d1;
                }
                lightOD += clDensity(p + sunDir * (d0 * 3.0), 0.0, 0.0) * d0;   // the long sample: what lies far up-sun
            } else
            for (int j = 0; j < lightSteps; j++) {
                vec3 lp = p + sunDir * (lds * (float(j) + lightJitter));
                lightOD += clDensity(lp, detailFade * farFade * 0.5, 0.0) * lds;
            }
            // MULTIPLE SCATTERING (Wrenninge's octaves): single scattering with
            // Beer's law says a cloud base under 240 m of cloud gets no sun at
            // all (exp(-64) is 0) and undersides went near-black. Real cloud
            // light bounces many times; the octave sum — each order with the
            // extinction, the sun's contribution and the phase anisotropy
            // halved — is the standard cheap stand-in, and it is what gives a
            // base its grey instead of its shadow.
            float beer = 0.0;
            {
                float a = 1.0, b = 1.0, c = 1.0;
                for (int k = 0; k < 3; k++) {
                    beer += a * exp(-lightOD * b) * clPhaseHG(mu, u.march.z * c);
                    a *= 0.45; b *= 0.5; c *= 0.5;
                }
                beer /= max(1e-4, clPhaseHG(mu, u.march.z));   // `phase` multiplies below
            }
            float powder = 1.0 - exp(-2.0 * density * stepLen);
            // Ambient grades with height: bases sit in their own shadow, tops
            // face the open sky — a gentler ramp than before (0.5 at the base
            // read as a hole in the sky), plus a bounce from the lit ground.
            float hf = clLayerHeight(p);
            vec3 ambient = u.skyAmbient.rgb * (0.70 + 0.30 * hf) +
                           sunLight * 0.06 * (1.0 - hf) * max(0.0, sunDir.y);
            // ANALYTIC in-scatter for the step: source * (1 - exp(-sigma*ds)).
            // The rectangle rule (source * density * ds) overshoots by the
            // per-step optical depth and stacked into the horizon white wall.
            vec3 lum = sunLight * beer * phase * powder + ambient;
            float stepT = exp(-density * stepLen);
            scattered += transmittance * lum * (1.0 - stepT);
            const float wgt = transmittance * (1.0 - stepT);
            depthSum += length(p - camPos) * wgt;
            depthW += wgt;
            transmittance *= stepT;
            if (transmittance < 0.01) break;
        }
    }

    // Distant clouds sit behind kilometres of atmosphere: contrast washes
    // toward the sky (the cloud overlay composites after the scene's own
    // aerial perspective, so the deck needs its own), and the deck ends
    // SOFTLY at the march clamp instead of on a crisp entry ring.
    float aerial = smoothstep(5000.0, 0.85 * u.march.w, t0) * 0.7;
    // (the haze tints the CLOUD a ray found, in proportion to it -- it was mixed into every marched ray, empty ones too, and
    // rays the weather probe skipped got none: the weather map's 320 m texels showed round every far cloud as pale squares,
    // Glenn, 2026-11-05: "squares around the clouds ... they stand out as weird squares")
    scattered = mix(scattered, u.skyAmbient.rgb * 1.15 * (1.0 - transmittance), aerial);
    transmittance = mix(transmittance, 1.0, aerial * 0.35);

    float ringFade = 1.0 - smoothstep(0.75 * u.march.w, u.march.w, t0);
    scattered *= ringFade;
    transmittance = mix(1.0, transmittance, ringFade);
    outColor = vec4(scattered + transmittance * cirrus.rgb, transmittance * (1.0 - cirrus.a));
    outCloudDepth = depthW > 1e-3 ? depthSum / depthW : -1.0;
}
