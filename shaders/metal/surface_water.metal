// Water surface (Surface::Water, id 12): depth-graded ocean colour + shoreline
// foam, plus the animated multi-scale wave normals and sea-state roughness.

// Water: depth-graded ocean + animated shoreline foam. meshUV.x = baked water
// depth (m, seaLevel - floor), meshUV.y = baked distance to land (m). Colour
// only — the material's low roughness + <1 opacity give reflection + fresnel
// transparency via shadeSurface, and the animated WAVE NORMALS live there too
// (surfaceReliefWater, below). `time` is camera.windTime (seconds).
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

    // OCEAN WAVES (Surface::Water): an animated multi-scale wave normal.
    // Three directional trains (a long swell + two chop trains at offset
    // headings) with ANALYTIC slopes, each amplitude-modulated by a slow noise
    // patch so no train reads as a repeating grid; plus a time-advected micro-
    // ripple that FADES WITH DISTANCE (the old fixed high-frequency noise
    // aliased into uniform far-field sparkle). Crest sharpening + a roughness
    // lift near crests give the sun glint a sea-state structure.
void surfaceReliefWater(float3 worldPos, float2 meshUV, float time, float3 cameraPos,
                        thread float3& normal, thread float& rough) {
        float wx = worldPos.x, wz = worldPos.z;
        float wt = time;
        float2 P = float2(wx, wz);
        // Shallows damp the swell so the waterline doesn't wobble the beach.
        float depthFade = 0.15 + 0.85 * saturate(meshUV.x / 6.0);
        float slopeX = 0.0, slopeZ = 0.0, crest = 0.0;
        // Two swells at offset headings (directional spread: their interference
        // makes CELLS, not parallel stripes) + two chop trains.
        const float3 trainDirLen[4] = {   // dir.x, dir.z, wavelength (m)
            float3( 0.980,  0.199, 58.0),
            float3( 0.845,  0.535, 47.0),
            float3( 0.827, -0.562, 21.0),
            float3( 0.399,  0.917,  8.5),
        };
        const float trainAmp[4]   = {0.115, 0.100, 0.075, 0.080};
        const float trainSpeed[4] = {6.5, 5.8, 3.8, 2.1};
        // Each train fades once its wavelength drops toward the pixel footprint —
        // otherwise the short chop aliases into a corduroy grating at distance.
        float dist = length(cameraPos - worldPos);
        const float trainFade[4] = {0.00035, 0.00045, 0.0012, 0.0035};
        for (int i = 0; i < 4; ++i) {
            float fade = exp(-dist * trainFade[i]);
            if (fade < 0.02) continue;
            float2 d = trainDirLen[i].xy;
            float k = 6.2831853 / trainDirLen[i].z;
            // slow amplitude patches: wave GROUPS, not an infinite even train
            float patch = 0.45 + 0.55 * vnoise2(wx * 0.013 + float(i) * 7.31,
                                                wz * 0.013 - float(i) * 3.17);
            // low-frequency phase warp: bends the crest lines so no train reads
            // as a ruled grating stretching to the horizon (short trains bend more)
            float warpF = 0.008 + 0.014 * float(i);
            float warp = (2.8 + 1.6 * float(i)) * vnoise2(wx * warpF - float(i) * 5.7,
                                                          wz * warpF + float(i) * 2.9);
            float ph = dot(P, d) * k - wt * trainSpeed[i] * k + warp;
            float s = sin(ph), cph = cos(ph);
            float w = trainAmp[i] * patch * fade;
            float slope = w * cph * (0.55 + 0.45 * (s * 0.5 + 0.5));  // sharpened crests
            slopeX += d.x * slope;
            slopeZ += d.y * slope;
            crest += w * (s * 0.5 + 0.5);
        }
        // Micro-ripple: two advected octaves, faded by camera distance so the
        // horizon stays calm instead of shimmering with sub-pixel noise.
        float lodFade = exp(-dist * 0.004);
        if (lodFade > 0.02) {
            float g0 = vnoise2(wx * 1.6 + wt * 0.50, wz * 1.6 + wt * 0.23) +
                       0.5 * vnoise2(wx * 4.7 - wt * 0.40, wz * 4.7 + wt * 0.31);
            float gx = vnoise2(wx * 1.6 + 0.30 + wt * 0.50, wz * 1.6 + wt * 0.23) +
                       0.5 * vnoise2(wx * 4.7 + 0.30 - wt * 0.40, wz * 4.7 + wt * 0.31) - g0;
            float gz = vnoise2(wx * 1.6 + wt * 0.50, wz * 1.6 + 0.30 + wt * 0.23) +
                       0.5 * vnoise2(wx * 4.7 - wt * 0.40, wz * 4.7 + 0.30 + wt * 0.31) - g0;
            slopeX += gx * 0.45 * lodFade;
            slopeZ += gz * 0.45 * lodFade;
        }
        normal = normalize(float3(-slopeX * depthFade, 1.0, -slopeZ * depthFade));
        // Sea-state roughness: glassy troughs, scuffed crests — the sun lane
        // stretches and breaks along the waves instead of one uniform streak.
        rough = clamp(rough + smoothstep(0.10, 0.30, crest) * 0.10, 0.02, 0.30);
}
