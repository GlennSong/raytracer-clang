// Road surface (Surface::Road, id 11): lane paint + sidewalk from road-local
// mesh UV, plus the world-space micro-relief that tilts the normal and varies
// roughness. Albedo and relief live together here.
// Follows surfaces_facade.metal — surfRoadMarkings calls surfAsphalt.

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

    // Road micro-relief (device: roads "don't look like a PBR texture"): the road
    // carries no baked normal/roughness maps (its mesh UV is road-local paint
    // space), so perturb the normal and vary the roughness procedurally from the
    // same world-planar noise the asphalt albedo tiles by. Mirrors WGSL/Vulkan.
void surfaceReliefRoad(float3 worldPos, thread float3& normal, thread float& rough) {
        float rx = worldPos.x, rz = worldPos.z;
        // Three octaves: metre-scale undulation, decimetre patching, and a
        // near-aggregate grain — summed finite differences tilt the normal hard
        // enough to read as bumpy asphalt (device: "no sense of bumpiness").
        float b0 = vnoise2(rx * 2.6, rz * 2.6) + 0.5 * vnoise2(rx * 11.0, rz * 11.0)
                 + 0.3 * vnoise2(rx * 37.0, rz * 37.0);
        float bx = vnoise2(rx * 2.6 + 0.4, rz * 2.6) + 0.5 * vnoise2(rx * 11.0 + 1.7, rz * 11.0)
                 + 0.3 * vnoise2(rx * 37.0 + 2.3, rz * 37.0) - b0;
        float bz = vnoise2(rx * 2.6, rz * 2.6 + 0.4) + 0.5 * vnoise2(rx * 11.0, rz * 11.0 + 1.7)
                 + 0.3 * vnoise2(rx * 37.0, rz * 37.0 + 2.3) - b0;
        // 0.35, not 0.6. This tilt was tuned under the SUN — a distant light
        // whose direction barely changes across a street, where a hard tilt
        // reads as welcome grain. Street lamps are the first CLOSE, LOW lights
        // this world has had: 4 m up and metres away, their direction sweeps
        // rapidly and rakes the surface, so the same tilt turns into blotchy
        // patches under every pool (device: "weird blocky glitches... it's
        // gotta be the lights"). Softer keeps the asphalt reading bumpy by day
        // without the night mottling.
        normal = normalize(normal + float3(-bx, 0.0, -bz) * 0.35);
        float spk = vnoise2(rx * 23.0, rz * 23.0);
        rough = clamp(rough + (spk - 0.5) * 0.25, 0.55, 1.0);
}
