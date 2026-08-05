// Natural ground (Surface::TerrainGround, id 13): albedo grain over the baked
// biome vertex colour, plus slope-scaled normal relief and roughness variation.

// Natural ground: the biome colour is baked in the vertex colour (grass/rock/sand/
// snow/sea floor); add fine albedo GRAIN so it isn't a flat wash. The normal
// micro-relief + roughness live below in surfaceReliefTerrain, like the road.
float3 surfTerrain(float3 base, float3 worldPos) {
    float g = fbm2(worldPos.x * 0.5, worldPos.z * 0.5) * 0.6 +
              fbm2(worldPos.x * 2.1, worldPos.z * 2.1) * 0.4;   // ~[0,1]
    return base * (0.90 + 0.18 * g);
}

    // Natural ground micro-relief (Surface::TerrainGround): the biome COLOUR is baked
    // in the vertex colour; here we give the ground a real SURFACE — a slope-scaled
    // normal perturbation (steep rock tilts hard, flat sand/grass stays gentle) plus
    // roughness variation (rock rough, high+flat snow a touch glossier for sheen) —
    // from the same world-planar value noise the road uses, so the terrain stops
    // reading as a flat-shaded plane. World-space, sub-metre to few-metre octaves.
void surfaceReliefTerrain(float3 worldPos, thread float3& normal, thread float& rough) {
        float wx = worldPos.x, wz = worldPos.z;
        float slope = clamp(1.0 - normal.y, 0.0, 1.0);
        float amp = 0.28 + 0.65 * slope;                 // steeper => more relief
        float g0 = vnoise2(wx * 1.7, wz * 1.7) + 0.5 * vnoise2(wx * 5.3, wz * 5.3)
                 + 0.3 * vnoise2(wx * 15.0, wz * 15.0);
        float gx = vnoise2(wx * 1.7 + 0.4, wz * 1.7) + 0.5 * vnoise2(wx * 5.3 + 1.7, wz * 5.3)
                 + 0.3 * vnoise2(wx * 15.0 + 2.3, wz * 15.0) - g0;
        float gz = vnoise2(wx * 1.7, wz * 1.7 + 0.4) + 0.5 * vnoise2(wx * 5.3, wz * 5.3 + 1.7)
                 + 0.3 * vnoise2(wx * 15.0, wz * 15.0 + 2.3) - g0;
        normal = normalize(normal + float3(-gx, 0.0, -gz) * amp);
        float snowy = clamp((worldPos.y - 80.0) / 30.0, 0.0, 1.0) * (1.0 - slope);
        rough = clamp(mix(0.93, 0.72, snowy) + (vnoise2(wx * 11.0, wz * 11.0) - 0.5) * 0.14,
                      0.55, 1.0);
}
