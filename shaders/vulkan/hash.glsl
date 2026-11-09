// THE ENGINE'S HASHES AND VALUE NOISE, ON THE GPU: bit for bit hash.h (lowbias32, hashCell, unit24) and procgen/noise.h
// (valueNoise2), so a shader reads the same field the CPU does. Keep the two alike.
#ifndef RT_HASH_GLSL
#define RT_HASH_GLSL

uint lowbias32(uint h) {
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

uint hashCell(uint a, uint b, uint seed) {
    return lowbias32(a * 0x9E3779B1u ^ b * 0x85EBCA77u ^ seed * 0xC2B2AE3Du);
}

// a hash's low 24 bits as a number in [0, 1)
float unit24(uint h) { return float(h & 0xFFFFFFu) / 16777216.0; }

// value noise over the grid hash, smoothstep-blended: noise.h valueNoise2 (in float: the CPU's is double)
float valueNoise2(vec2 p, uint seed) {
    const vec2 f = floor(p);
    const ivec2 c = ivec2(f);
    vec2 t = p - f;
    t = t * t * (3.0 - 2.0 * t);
    const float a = unit24(hashCell(uint(c.x), uint(c.y), seed));
    const float b = unit24(hashCell(uint(c.x + 1), uint(c.y), seed));
    const float d = unit24(hashCell(uint(c.x), uint(c.y + 1), seed));
    const float e = unit24(hashCell(uint(c.x + 1), uint(c.y + 1), seed));
    return mix(mix(a, b, t.x), mix(d, e, t.x), t.y);
}

// THE GUST FACTOR (engine/wind.h windGustFactor): 1 - gust/2 .. 1 + gust, patches ~120 m across carried downwind --
// `drift` how far the air has gone (Wind::drift, xz), `gust` how squally
float windGustFactor(vec2 xz, vec2 drift, float gust) {
    const float s = 1.0 / 120.0;
    const vec2 q = xz - drift;
    const float n = 0.65 * valueNoise2(q * s, 0x57u) + 0.35 * valueNoise2(vec2(q.x * s * 2.7 + 5.3, q.y * s * 2.7 - 1.9), 0x57u);
    return 1.0 + gust * (1.5 * n - 0.5);
}

#endif
