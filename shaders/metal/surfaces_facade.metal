// Analytic facade/ground patterns selected by Surface id, all sharing the
// (base, planar u, v) signature. Kept byte-for-byte with scene.cpp.
// Needs the noise primitives in common.metal (hash21/vnoise2/fbm2/tile1).

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
