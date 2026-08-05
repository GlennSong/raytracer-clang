// Surface library dispatchers. Follows every surface_*.metal module.
// applySurface  -> patterned albedo (called before the normal-map sample)
// applySurfaceRelief -> normal/roughness perturbation (called after it)

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

// Normal/roughness relief for the surfaces that carry no baked normal or
// roughness map (road, water, terrain) and must synthesize one. Called from
// shadeSurface AFTER the normal-map sample, so it perturbs whatever normal that
// produced — road and terrain add to it, water replaces it outright.
//
// The three ids are mutually exclusive, so this else-if chain is equivalent to
// the three sequential `if (surfId == …)` blocks it replaces.
void applySurfaceRelief(uint id, float3 worldPos, float2 meshUV, float time,
                        float3 cameraPos,
                        thread float3& normal, thread float& rough) {
    if (id == 11u)      surfaceReliefRoad(worldPos, normal, rough);
    else if (id == 13u) surfaceReliefTerrain(worldPos, normal, rough);
    else if (id == 12u) surfaceReliefWater(worldPos, meshUV, time, cameraPos,
                                           normal, rough);
}
