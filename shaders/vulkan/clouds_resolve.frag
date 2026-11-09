#version 450
// THE CLOUDS' HISTORY (#187; the shimmer as the camera moves, #59): the half-res march (clouds.frag) blended with last
// frame's resolved clouds, reprojected to where that cloud was on screen then, so the march's per-frame start offsets
// average away instead of a pattern fixed to the screen crawling across the clouds. The history is clamped to this
// frame's 3x3 neighbourhood (no ghost where a cloud edge or a hill's silhouette moved), and the blend is
// 1 - historyWeight of the new frame. Same set layout as the march: the uniforms, then three images.
layout(set = 0, binding = 0) uniform CloudUniforms {
    mat4 invViewProjection;   // CAMERA-RELATIVE: clip -> the offset from the camera (ADR-0335)
    vec4 cameraPosition;      // w the camera's exposure (pre-applied)
    vec4 sunDirection;
    vec4 sunColor;
    vec4 skyAmbient;
    vec4 planetCenter;
    vec4 layer;           // x bottom, y top
    vec4 params;
    vec4 march;           // w farDistance
    vec4 detail;
    mat4 prevViewProjection;  // CAMERA-RELATIVE: the offset from THIS frame's camera -> last frame's clip
    vec4 frame;           // x dither offset, y history weight (0: reset)
} u;
layout(set = 0, binding = 1) uniform sampler2D current;   // this frame's march (rgb in-scatter, a transmittance)
layout(set = 0, binding = 2) uniform sampler2D history;   // last frame's resolve
layout(set = 0, binding = 3) uniform sampler2D sceneDepth;
layout(set = 0, binding = 4) uniform sampler2D cloudDepth;   // the march's: each pixel's distance to its cloud (-1: none)
layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
    vec4 cur = texture(current, inUV);
    if (u.frame.y <= 0.0) { outColor = cur; return; }
    // the neighbourhood's range, this frame
    vec2 texel = 1.0 / vec2(textureSize(current, 0));
    vec4 lo = cur, hi = cur, sum = cur;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) {
            if (x == 0 && y == 0) continue;
            vec4 s = texture(current, inUV + vec2(x, y) * texel);
            lo = min(lo, s);
            hi = max(hi, s);
            sum += s;
        }
    // GRAIN IN LOW LIGHT (ADR-0379; Glenn: "Clouds are grainy in low light"): the march's per-frame noise is the same
    // in the dark, but the night eye amplifies it several stops. What enters the history is the pixel taken most of
    // the way to its 3 x 3 mean -- a cloud has no detail at half resolution that this blurs -- so ten frames of history
    // average a third of the noise they did. (The clamp box above stays the raw neighbourhood's.)
    // ...in low light only (ADR-0384): by day it took the wisps' edge detail with the grain (Glenn: "the clouds are
    // very solid looking"). By the camera's exposure (cameraPosition.w, pre-applied: ~0.7 at noon, hundreds at
    // night): none at 2 and under (EV100 ~13, the afternoon), all of it from 32 (EV100 ~9) -- the sunset's clouds
    // were grainy at 8 (ADR-0386).
    float lowLight = smoothstep(1.0, 5.0, log2(max(u.cameraPosition.w, 1e-6)));
    cur = mix(cur, sum / 9.0, 0.7 * lowLight);
    // WHERE THIS PIXEL'S CLOUD WAS: the point along its ray at ITS cloud's depth (ADR-0334; the march's second target), or
    // with none, at the deck's middle height -- or the scene, nearer -- seen through last frame's camera (the flipped
    // convention of clouds.frag: ndc = uv * 2 - 1). By the middle height alone, clouds slid when the camera was near or
    // above the deck (Glenn: "some of the layers shift as you go above the clouds")
    vec2 ndc = inUV * 2.0 - 1.0;
    vec4 wf = u.invViewProjection * vec4(ndc, 0.002, 1.0);   // (~100 m out, not the near plane: clouds.frag's precision note)
    vec3 cam = u.cameraPosition.xyz;
    vec3 dir = normalize(wf.xyz / wf.w);
    float mid = 0.5 * (u.layer.x + u.layer.y);
    float t = u.march.w;
    if (abs(dir.y) > 1e-4) {
        float tm = (mid - cam.y) / dir.y;
        if (tm > 0.0) t = min(t, tm);
    }
    const float cd = texture(cloudDepth, inUV).r;
    if (cd > 0.0 && u.detail.z > 0.5) t = cd;   // (u.detail.z 0: by the middle height, the A/B: RT_CLOUD_DEPTH=0)
    float depth = texture(sceneDepth, inUV).r;
    if (depth > 0.0) {
        vec4 w = u.invViewProjection * vec4(ndc, depth, 1.0);
        t = min(t, length(w.xyz / w.w));
    }
    vec4 prevClip = u.prevViewProjection * vec4(dir * t, 1.0);
    vec2 prevUV = prevClip.xy / prevClip.w * 0.5 + 0.5;
    if (prevClip.w <= 0.0 || any(lessThan(prevUV, vec2(0.0))) || any(greaterThan(prevUV, vec2(1.0)))) {
        outColor = cur;   // off last frame's screen: nothing to blend
        return;
    }
    vec4 hist = clamp(texture(history, prevUV), lo, hi);
    outColor = mix(cur, hist, u.frame.y);
}
