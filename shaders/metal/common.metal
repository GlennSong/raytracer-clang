// Shared vertex layouts, varyings, and small BRDF helpers used across the
// shader modules. The loader concatenates the modules in dependency order:
// shader_types.h, common, environment, shadows, lighting, post (ADR-0017).

struct Vertex {
    packed_float3 position;
    packed_float3 normal;
    packed_float3 tangent;
    packed_float2 texcoord;
    packed_float3 color;      // per-vertex tint (matches engine Vertex::color)
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
    float3 vertexColor;
};

// VertexOut + per-instance material values carried through interpolation
// (constant within an instance) for the instanced path.
struct FragmentData {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
    float3 vertexColor;
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float flags;
    float3 emission;
    uint textureFlags;
};

// Lit passes write HDR color + view-space normals (for SSR).
struct GBufferOut {
    float4 color [[color(0)]];
    float4 viewNormal [[color(1)]];
};

float fresnelSchlick(float cosTheta, float f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

float3 fresnelSchlickVec(float cosTheta, float3 f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

// Checkerboard pattern based on world position
float3 applyCheckerboard(float3 albedo, float3 worldPos) {
    int cx = int(floor(worldPos.x));
    int cz = int(floor(worldPos.z));
    bool dark = ((cx + cz) & 1) != 0;
    return dark ? albedo * 0.3 : albedo;
}

// --- Procedural surface library (kept byte-for-byte with scene.cpp) ---------
// Analytic, world-space city materials selected by a Surface id (renderer.h /
// material.h) packed into bits 8..15 of the material flags. Each takes a base
// albedo and returns the patterned albedo; no texture maps involved.
constant float SURF_PI = 3.14159265;

float hash21(float a, float b) {
    return fract(sin(a * 12.9898 + b * 78.233) * 43758.5453);
}
float vnoise2(float x, float y) {
    float xi = floor(x), yi = floor(y), xf = x - xi, yf = y - yi;
    float a = hash21(xi, yi), b = hash21(xi + 1.0, yi);
    float c = hash21(xi, yi + 1.0), d = hash21(xi + 1.0, yi + 1.0);
    float ux = xf * xf * (3.0 - 2.0 * xf), uy = yf * yf * (3.0 - 2.0 * yf);
    return a * (1.0 - ux) * (1.0 - uy) + b * ux * (1.0 - uy) +
           c * (1.0 - ux) * uy + d * ux * uy;
}
float fbm2(float x, float y) {
    float v = 0.0, amp = 0.5, f = 1.0;
    for (int i = 0; i < 4; ++i) { v += amp * vnoise2(x * f, y * f); f *= 2.0; amp *= 0.5; }
    return v;
}
float tile1(float x, float m) { return x - m * floor(x / m); }
float2 surfUV(float3 p, float3 n) {
    if (abs(n.y) > 0.5) return float2(p.x, p.z);
    float2 t = float2(n.z, -n.x);
    float tl = length(t);
    t = tl < 1e-6 ? float2(1.0, 0.0) : t / tl;
    return float2(p.x * t.x + p.z * t.y, p.y);
}

float3 surfBrick(float3 base, float u, float v) {
    const float courseH = 0.075, brickL = 0.20, mortar = 0.011;
    float row = floor(v / courseH);
    float off = (fmod(abs(row), 2.0) < 1.0) ? 0.0 : brickL * 0.5;
    float uu = u + off, col = floor(uu / brickL);
    float fy = v - row * courseH, fx = uu - col * brickL;
    float joint = min(min(fy, courseH - fy), min(fx, brickL - fx));
    float h = hash21(col, row), h2 = hash21(col * 1.7 + 3.1, row * 0.9 + 5.7);
    float shade = 0.74 + 0.46 * h;
    if (h2 < 0.12) shade *= 0.6;
    shade *= 0.94 + 0.12 * (fx / brickL);
    float t = saturate((joint - mortar) / 0.004);
    return mix(float3(0.30, 0.29, 0.27), base * shade, t);
}
float3 surfConcrete(float3 base, float u, float v) {
    float n = fbm2(u * 0.6, v * 0.6), fine = vnoise2(u * 9.0, v * 9.0);
    float shade = 0.84 + 0.22 * n + 0.06 * (fine - 0.5);
    float gu = tile1(u, 3.0); gu = min(gu, 3.0 - gu);
    float gv = tile1(v, 3.0); gv = min(gv, 3.0 - gv);
    float jt = smoothstep(0.015, 0.04, min(gu, gv));
    return base * shade * (0.74 + 0.26 * jt);
}
float3 surfStucco(float3 base, float u, float v) {
    float n = fbm2(u * 3.0, v * 3.0), fine = vnoise2(u * 22.0, v * 22.0);
    return base * (0.90 + 0.12 * (n - 0.5) + 0.10 * (fine - 0.5));
}
float3 surfRoofTile(float3 base, float u, float v) {
    const float tileW = 0.18, rowH = 0.32;
    float row = floor(v / rowH);
    float off = (fmod(abs(row), 2.0) < 1.0) ? 0.0 : tileW * 0.5;
    float uu = u + off, col = floor(uu / tileW);
    float fx = uu - col * tileW, fy = v - row * rowH;
    float curve = sin(SURF_PI * (fx / tileW));
    float valley = smoothstep(0.0, 0.02, min(fx, tileW - fx));
    float lap = smoothstep(0.0, 0.05, fy);
    float h = hash21(col, row);
    return base * ((0.55 + 0.5 * curve) * (0.85 + 0.30 * h) * valley * (0.6 + 0.4 * lap));
}
float3 surfShingle(float3 base, float u, float v) {
    const float tabW = 0.30, rowH = 0.14;
    float row = floor(v / rowH);
    float off = (fmod(abs(row), 2.0) < 1.0) ? 0.0 : tabW * 0.5;
    float uu = u + off, col = floor(uu / tabW);
    float fx = uu - col * tabW, fy = v - row * rowH;
    float h = hash21(col, row);
    float key = smoothstep(0.0, 0.012, min(fx, tabW - fx));
    float shadow = smoothstep(0.0, 0.03, fy);
    return base * (0.82 + 0.32 * h) * (0.55 + 0.45 * key) * (0.5 + 0.5 * shadow);
}
float3 surfCorrugated(float3 base, float u, float v) {
    const float ribW = 0.12;
    float rib = cos(2.0 * SURF_PI * u / ribW);
    float shade = 0.72 + 0.28 * rib;
    float rust = fbm2(u * 1.5, v * 0.6);
    float rmask = saturate((rust - 0.62) / 0.18) * 0.45;
    return base * shade * (1.0 - rmask) + float3(0.40, 0.22, 0.12) * rmask;
}
float3 surfAsphalt(float3 base, float u, float v) {
    float spk = vnoise2(u * 30.0, v * 30.0), patch = fbm2(u * 0.4, v * 0.4);
    float shade = clamp(0.86 + 0.20 * (spk - 0.5) + 0.10 * (patch - 0.5), 0.6, 1.02);
    return base * shade;
}
float3 surfPavement(float3 base, float u, float v) {
    const float slab = 1.2;
    float su = tile1(u, slab), sv = tile1(v, slab);
    float joint = min(min(su, slab - su), min(sv, slab - sv));
    float h = hash21(floor(u / slab), floor(v / slab));
    float spk = vnoise2(u * 26.0, v * 26.0);
    float shade = 0.90 + 0.12 * (h - 0.5) + 0.06 * (spk - 0.5);
    float jt = smoothstep(0.02, 0.05, joint);
    return base * shade * (0.6 + 0.4 * jt);
}
float3 surfCobble(float3 base, float u, float v) {
    const float cell = 0.18;
    float cu = u / cell, cv = v / cell, iu = floor(cu), iv = floor(cv);
    float best = 1e9, bh = 0.0;
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            float ci = iu + float(di), cj = iv + float(dj);
            float jx = hash21(ci, cj), jy = hash21(ci + 5.2, cj + 1.7);
            float px = ci + 0.5 + (jx - 0.5) * 0.7, py = cj + 0.5 + (jy - 0.5) * 0.7;
            float dx = cu - px, dy = cv - py, d = dx * dx + dy * dy;
            if (d < best) { best = d; bh = hash21(ci + 9.1, cj + 4.3); }
        }
    float stone = smoothstep(0.0, 0.12, 0.62 - sqrt(best));
    return mix(float3(0.32, 0.30, 0.27), base * (0.7 + 0.6 * bh), stone);
}
float3 surfWood(float3 base, float u, float v) {
    const float boardH = 0.18;
    float row = floor(v / boardH), fy = v - row * boardH;
    float h = hash21(row, 3.0), grain = vnoise2(u * 40.0, row * 9.0 + v * 2.0);
    float shadow = smoothstep(0.0, 0.02, fy);
    return base * (0.85 + 0.20 * h + 0.12 * (grain - 0.5)) * (0.55 + 0.45 * shadow);
}

// Road lane paint from road-local mesh UV (ADR-0044 / Problem 3), mirror of
// scene.cpp surfRoadMarkings. mu = lateral (1 left / 2 centre / 3 right; < 0.5 =
// not carriageway -> plain). Constant AA band (not fwidth) so it is legal in any
// shader stage; MSAA/TAA cleans up the rest.
float3 surfRoadMarkings(float3 base, float mu, float mv, float wu, float wv) {
    // One surface id covers the welded road, split by road-local mu: the
    // sidewalk/curb band (mu in [0,1]) wears concrete pavement, the carriageway
    // (mu in [1,3]) asphalt grain under the lane paint. wu/wv = the world-planar
    // UV the other surfaces tile by. Mirrors scene.cpp / WGSL / Vulkan.
    if (mu < 0.98) {
        // Sidewalk band: concrete grain + scoring joints that FOLLOW the curb
        // (slab tops bake u = -(metres along the kerb loop)). Curb faces (u = 0)
        // stay plain. Mirrors scene.cpp / WGSL / Vulkan.
        float spk = vnoise2(wu * 26.0, wv * 26.0);
        float3 c = base * (0.92 + 0.10 * (spk - 0.5));
        float su = -mu;
        if (su > 0.02) {
            float t = tile1(su, 1.5);
            float jd = min(t, 1.5 - t);
            c = c * (0.68 + 0.32 * smoothstep(0.02, 0.06, jd));
        }
        return c;
    }
    float3 deck = surfAsphalt(base, wu, wv);            // grained asphalt deck
    // Dashed lane DIVIDER strip (u = 4, v = raw arc-length): one thin strip per
    // internal same-direction lane boundary on multilane roads; 3 m of white
    // paint every 7.5 m. Mirrors scene.cpp / WGSL / Vulkan.
    if (mu > 3.5)
        return (fract(mv / 7.5) < 0.4) ? float3(0.86, 0.86, 0.83) : deck;
    float lat = mu - 2.0;                            // [-1, 1], 0 = centreline
    float yL = 1.0 - smoothstep(0.013, 0.019, abs(lat - 0.030));
    float yR = 1.0 - smoothstep(0.013, 0.019, abs(lat + 0.030));
    float y  = max(yL, yR);                          // double-yellow centreline
    float wL = 1.0 - smoothstep(0.016, 0.022, abs(lat - 0.86));
    float wR = 1.0 - smoothstep(0.016, 0.022, abs(lat + 0.86));
    float w  = max(wL, wR);                          // white edge lines
    // The centreline ENDS before a crosswalk (device: the double yellow cut
    // through the zebra bars): the band ends at mv ~3.6, so the yellow fades in
    // just past it. Without crosswalks mv is a large sentinel (full-length line).
    y *= smoothstep(4.0, 4.8, mv);
    float3 c = mix(deck, float3(0.82, 0.68, 0.13), y);
    c = mix(c, float3(0.86, 0.86, 0.83), w);
    // Zebra crosswalk painted into the road texture (ADR-0062): mv = metres PAST
    // the junction mouth (baked by the road mesher), so the band sits set back on
    // the approach, not in the intersection. Bars run across the carriageway (by
    // the lateral coord mu). mv is a large sentinel where no crosswalk belongs.
    // Only the carriageway (mu in [~1,3]) — the raised sidewalk/curb shares this
    // surface but carries a 0..1 UV, so gate on mu > 1.05 to keep paint off the curb.
    float cwEdge = smoothstep(0.5, 0.8, mv) * (1.0 - smoothstep(3.3, 3.6, mv));
    float bars = step(0.5, fract((mu - 1.0) * 4.0));
    c = mix(c, float3(0.90, 0.90, 0.88), cwEdge * bars * step(1.05, mu));
    return c;
}

// Water: depth-graded ocean + animated shoreline foam. meshUV.x = baked water
// depth (m, seaLevel - floor), meshUV.y = baked distance to land (m). Colour
// only — the material's low roughness + <1 opacity give reflection + fresnel
// transparency via shadeSurface, and the animated WAVE NORMALS live there too
// (surfId 12 block). `time` is camera.windTime (seconds).
float3 surfWater(float3 base, float depth, float shore, float3 worldPos, float time) {
    // `base` is the level's water.color (adjustable) — the DEEP ocean tone. The
    // shallows brighten toward teal from it, so darkening water.color darkens the
    // whole body (device feedback: "too light... looks like a sea, not the ocean").
    // Deeper falloff (30 m) keeps most of an open sea in the dark deep tone.
    float3 deep    = base;
    float3 mid     = base * 2.0 + float3(0.010, 0.045, 0.050);   // green-blue
    float3 shallow = base * 3.0 + float3(0.030, 0.130, 0.120);   // teal shallows
    float t = saturate(depth / 30.0);
    float3 c = t < 0.5 ? mix(shallow, mid, t * 2.0) : mix(mid, deep, (t - 0.5) * 2.0);
    // Two-scale mottling, each drifting at its own rate: a fixed single-octave
    // pattern was the "repetitive texture" read. The slow macro patch mimics
    // cloud-shadow / deep-current banding; the finer one is chop shading.
    float m1 = fbm2(worldPos.x * 0.011 + time * 0.006, worldPos.z * 0.011 - time * 0.002);
    float m2 = fbm2(worldPos.x * 0.055 - time * 0.010, worldPos.z * 0.055 + time * 0.004);
    c *= 0.88 + 0.12 * m1 + 0.07 * m2;
    // Sparse whitecap flecks in open water (deep enough that they read as sea
    // state, not shore foam), advecting with the wind and dissolving.
    float cap = fbm2(worldPos.x * 0.09 + time * 0.05, worldPos.z * 0.09 - time * 0.03) *
                (0.55 + 0.45 * fbm2(worldPos.x * 0.021 - time * 0.013, worldPos.z * 0.021));
    float capMask = smoothstep(0.60, 0.78, cap) * saturate(depth / 8.0) * 0.30;
    c = mix(c, float3(0.85, 0.92, 0.95), capMask);
    // LAPPING shoreline foam: a NARROW washing edge plus patchy streaks that
    // fade fast off the beach — the wide saturated halo read as ice from above.
    float band = 6.0 + 4.5 * fbm2(worldPos.x * 0.05, worldPos.z * 0.05) +
                 2.5 * sin(time * 0.8 + 6.2831853 * fbm2(worldPos.x * 0.02, worldPos.z * 0.02));
    if (shore > 0.001 && shore < band) {
        float u = 1.0 - shore / band;
        float pattern = fbm2(worldPos.x * 0.35 + time * 0.22, worldPos.z * 0.35 - time * 0.13);
        pattern *= pattern;                          // patchy streaks, not a wash
        float f = u * u * (0.20 + 0.60 * pattern);
        f += smoothstep(0.85, 1.0, u) * 0.30;   // bright washing edge at the waterline
        c = mix(c, float3(0.92, 0.96, 0.98), saturate(f));
    }
    return c;
}

// Natural ground: the biome colour is baked in the vertex colour (grass/rock/sand/
// snow/sea floor); add fine albedo GRAIN so it isn't a flat wash. The normal
// micro-relief + roughness live in lighting.metal (surfId 13), like the road.
float3 surfTerrain(float3 base, float3 worldPos) {
    float g = fbm2(worldPos.x * 0.5, worldPos.z * 0.5) * 0.6 +
              fbm2(worldPos.x * 2.1, worldPos.z * 2.1) * 0.4;   // ~[0,1]
    return base * (0.90 + 0.18 * g);
}

float3 applySurface(uint id, float3 base, float3 worldPos, float3 n, float2 meshUV,
                    float time) {
    float2 uv = surfUV(worldPos, n);
    float3 c;
    switch (id) {
        case 1u:  c = surfBrick(base, uv.x, uv.y); break;
        case 2u:  c = surfConcrete(base, uv.x, uv.y); break;
        case 3u:  c = surfStucco(base, uv.x, uv.y); break;
        case 4u:  c = surfRoofTile(base, uv.x, uv.y); break;
        case 5u:  c = surfShingle(base, uv.x, uv.y); break;
        case 6u:  c = surfCorrugated(base, uv.x, uv.y); break;
        case 7u:  c = surfAsphalt(base, uv.x, uv.y); break;
        case 8u:  c = surfPavement(base, uv.x, uv.y); break;
        case 9u:  c = surfCobble(base, uv.x, uv.y); break;
        case 10u: c = surfWood(base, uv.x, uv.y); break;
        case 11u: c = surfRoadMarkings(base, meshUV.x, meshUV.y, uv.x, uv.y); break;
        case 12u: c = surfWater(base, meshUV.x, meshUV.y, worldPos, time); break;
        case 13u: c = surfTerrain(base, worldPos); break;
        default:  return base;
    }
    return saturate(c);   // keep albedo energy-conserving (see scene.cpp)
}
