// --- Composite pass: tone map + gamma from HDR scene texture to LDR drawable ---

// Color grade applied in scene-linear, BEFORE the tone map (the "Look" stage, in
// Blender terms) — so it is display-agnostic and survives an SDR->HDR output swap.
// Both ops are identity at the neutral defaults (contrast=1, saturation=1).
float3 applyGrade(float3 x, float contrast, float saturation) {
    // Saturation around Rec.709 luma.
    float luma = dot(x, float3(0.2126, 0.7152, 0.0722));
    x = max(luma + saturation * (x - luma), 0.0);
    // Contrast in log2 around middle grey (0.18): a filmic S-curve that behaves
    // consistently across the whole HDR range (no crushed/blown ends).
    const float grey = 0.18;
    float3 lx = log2(max(x, 1e-5));
    lx = (lx - log2(grey)) * contrast + log2(grey);
    return exp2(lx);
}

// The display transfer function, applied EXACTLY ONCE per frame.
//
// Which of shader or hardware owns it depends on the render target, and until
// visionOS every backend assumed the same answer. macOS picks a linear-storage
// BGRA8Unorm drawable and Vulkan hunts for a UNORM swapchain, both so the
// shader could own it; CompositorServices refuses anything but an sRGB format
// ("Layer configuration not supported"), so there the hardware owns it and the
// shader must not encode. That implicit global assumption is now an explicit
// per-target flag.
//
// `linearInput` says whether the value still needs the transform at all. Scene
// colour does; most debug views are already display-referred visualisations of
// unit-range data and would be washed out by a second application.
float4 encodeForTarget(float3 c, bool targetEncodesSRGB, bool linearInput) {
    if (linearInput && !targetEncodesSRGB) c = pow(saturate(c), float3(1.0 / 2.2));
    else if (!linearInput && targetEncodesSRGB) {
        // Already display-referred, but the hardware is about to encode: undo it
        // so the net transform is identity. Debug views only — the main path
        // stays linear all the way to the single encode above.
        c = pow(saturate(c), float3(2.2));
    }
    return float4(c, 1.0);
}

// ACES filmic (Stephen Hill fit) — RGB matrices preserve saturation. Returns
// SCENE-REFERRED (linear) colour; the display encode is applied by
// encodeForTarget, not folded in here.
float3 tonemapACES(float3 x) {
    float3 ci = float3(dot(float3(0.59719, 0.35458, 0.04823), x),
                       dot(float3(0.07600, 0.90834, 0.01566), x),
                       dot(float3(0.02840, 0.13383, 0.83777), x));
    float3 cf = (ci * (ci + 0.0245786) - 0.000090537) /
                (ci * (0.983729 * ci + 0.432951) + 0.238081);
    float3 c = float3(dot(float3( 1.60475, -0.53108, -0.07367), cf),
                      dot(float3(-0.10208,  1.10813, -0.00605), cf),
                      dot(float3(-0.00327, -0.07276,  1.07602), cf));
    return saturate(c);   // linear; encodeForTarget applies the display transform
}

// AgX (minimal fit, Benjamin Wrensch / Troy Sobotka) — a modern film view
// transform: gentle highlight rolloff with far less hue skew than ACES. The
// log+sigmoid produces a ~2.2 display-encoded result. Unlike ACES the encode is
// NOT a liftable trailing pow — it is inside the polynomial fit — so linear
// output is recovered by an approximate inverse at the end of tonemapAgX. That
// round-trips cleanly against the shader's own pow(1/2.2), but against a
// hardware sRGB target it differs slightly near black, where true sRGB has a
// linear toe that pure 2.2 does not. A proper linear-output AgX fit would remove
// the approximation; see docs/TECH_DEBT.md.
float3 agxContrastApprox(float3 x) {
    float3 x2 = x * x;
    float3 x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4
         - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

float3 tonemapAgX(float3 val) {
    const float3x3 agxMat = float3x3(
        float3(0.842479062253094, 0.0423282422610123, 0.0423756549057051),
        float3(0.0784335999999992, 0.878468636469772, 0.0784336),
        float3(0.0792237451477643, 0.0791661274605434, 0.879142973793104));
    const float3x3 agxMatInv = float3x3(
        float3(1.19687900512017, -0.0528968517574562, -0.0529716355144438),
        float3(-0.0980208811401368, 1.15190312990417, -0.0980434501171241),
        float3(-0.0990297440797205, -0.0989611768448433, 1.15107367264116));
    const float minEv = -12.47393;
    const float maxEv = 4.026069;
    val = agxMat * val;
    val = clamp(log2(max(val, 1e-10)), minEv, maxEv);
    val = (val - minEv) / (maxEv - minEv);
    val = agxContrastApprox(val);   // -> AgX display-encoded [0,1]
    val = agxMatInv * saturate(val);
    // Back to linear. The encode is inside agxContrastApprox rather than a
    // trailing pow, so this inverse is approximate — see the note above.
    return pow(saturate(val), float3(2.2));
}

struct CompositeOut {
    float4 position [[position]];
    float2 uv;
};

vertex CompositeOut vertexComposite(uint vid [[vertex_id]]) {
    float2 uv = float2((vid << 1) & 2, vid & 2);
    CompositeOut out;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    out.uv = float2(uv.x, 1.0 - uv.y);  // flip Y for Metal texture origin
    return out;
}

fragment float4 fragmentComposite(
    CompositeOut in [[stage_in]],
    texture2d<float> sceneColor [[texture(0)]],
    texture2d<float> ssrTexture [[texture(1)]],
    texture2d<float> aoTexture [[texture(2)]],
    depth2d<float> depthTex [[texture(3)]],
    texture2d<float> normalTexture [[texture(4)]],
    texture2d<float> bloomTexture [[texture(5)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant CompositeUniforms& params [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    sampler smp [[sampler(0)]]
) {
    // Read depth at this fragment's pixel position (no sampler needed)
    float depth = depthTex.read(uint2(in.position.xy));

    // --- Debug visualization modes ---
    if (params.debugView == 1) {
        // AO only — white = no occlusion, black = full occlusion
        float ao = aoTexture.sample(smp, in.uv).r;
        if (depth <= 0.0) ao = 1.0;   // reverse-Z background
        return encodeForTarget(float3(ao), params.targetEncodesSRGB, false);
    }
    if (params.debugView == 2) {
        // SSR only — show reflected color where SSR hit, black elsewhere
        float4 ssr = ssrTexture.sample(smp, in.uv);
        return encodeForTarget(ssr.rgb * ssr.a, params.targetEncodesSRGB, false);
    }
    if (params.debugView == 3) {
        // Depth — linearized on a log scale so it reads usefully at any near/far
        // ratio (a wide far plane otherwise pins a close scene to ~white).
        // white = near, black = far/background.
        float near = camera.nearPlane, far = camera.farPlane;
        float linZ = max(linearizeReverseZ(depth, near, far), near);
        float lin = (depth <= 0.0)
            ? 0.0                                            // background
            : saturate(1.0 - log2(linZ / near) / log2(far / near));
        return encodeForTarget(float3(lin), params.targetEncodesSRGB, false);
    }
    if (params.debugView == 4) {
        // View-space normals visualization
        float3 n = normalTexture.read(uint2(in.position.xy)).xyz;
        if (depth <= 0.0)
            return encodeForTarget(float3(0.5, 0.5, 1.0), params.targetEncodesSRGB, false);
        return encodeForTarget(n, params.targetEncodesSRGB, false);
    }
    if (params.debugView == 5) {
        // Shadow factor (the lit pass wrote grayscale into sceneColor) — raw, no
        // tone map. White = lit, black = shadowed. Sky shows mid-gray.
        if (depth <= 0.0)
            return encodeForTarget(float3(0.3), params.targetEncodesSRGB, false);
        return encodeForTarget(sceneColor.sample(smp, in.uv).rgb, params.targetEncodesSRGB, false);
    }
    if (params.debugView == 6) {
        // Albedo (the lit pass wrote raw material color into sceneColor) — no
        // exposure or tone map, gamma only, so material color shows directly.
        if (depth <= 0.0)
            return encodeForTarget(float3(0.0), params.targetEncodesSRGB, false);
        // Albedo is LINEAR material colour, so it is the one debug view that
        // genuinely needs the display transform.
        return encodeForTarget(sceneColor.sample(smp, in.uv).rgb, params.targetEncodesSRGB, true);
    }
    if (params.debugView == 7) {
        // Facing test: green = front-facing (G-buffer normal points toward the
        // camera), red = facing away. Every visible fragment should be green;
        // red marks a flipped normal or a shown back-face. Brightness = |N·V|.
        if (depth <= 0.0)
            return encodeForTarget(float3(0.0), params.targetEncodesSRGB, false);
        float3 n = normalize(normalTexture.read(uint2(in.position.xy)).xyz * 2.0 - 1.0);
        float3 viewPos = ssrViewPos(depth, in.uv, camera.invProjection);
        float ndv = dot(n, normalize(-viewPos));   // V points to the camera (origin)
        return encodeForTarget(ndv >= 0.0 ? float3(0.0, ndv, 0.0) : float3(-ndv, 0.0, 0.0),
                               params.targetEncodesSRGB, false);
    }
    if (params.debugView == 8) {
        // Shadow cascades: the lit pass wrote a per-cascade tint into sceneColor
        // (red/green/blue/yellow = cascade 0..3); show it raw.
        if (depth <= 0.0)
            return encodeForTarget(float3(0.0), params.targetEncodesSRGB, false);
        return encodeForTarget(sceneColor.sample(smp, in.uv).rgb, params.targetEncodesSRGB, false);
    }

    // --- Normal rendering ---
    float3 hdrColor;
    if (depth <= 0.0) {   // reverse-Z background (cleared far value)
        // Sky pixel — pass the scene image through. The skybox pass already
        // drew the active environment (HDR cube or procedural + clouds) behind
        // the geometry with the real rasterization camera, so it is correct on
        // every projection the engine renders with — including the device
        // compositor's asymmetric infinite-far per-eye matrices. This branch
        // once re-derived the sky from invViewProjection instead: a June 2026
        // workaround (dfc88fc) for a then-misoriented skybox/bake path, kept
        // after the bake was fixed, and the duplicated ray math broke under XR
        // projections (NaN far-plane unproject; device-only black sky). One
        // sky, one code path — skip AO/SSR, which are geometry effects.
        hdrColor = sceneColor.sample(smp, in.uv).rgb;
    } else {
        float4 hdr = sceneColor.sample(smp, in.uv);

        // Apply ambient occlusion if enabled
        if (params.ssaoEnabled != 0) {
            float ao = aoTexture.sample(smp, in.uv).r;
            hdr.rgb *= max(ao, params.aoFloor);
        }

        // Blend SSR reflections if enabled
        if (params.ssrEnabled != 0) {
            float4 ssr = ssrTexture.sample(smp, in.uv);
            hdr.rgb = mix(hdr.rgb, ssr.rgb, ssr.a * params.ssrBlendStrength);
        }

        hdrColor = hdr.rgb;
    }

    // Camera exposure — the single scene-referred multiplier, applied uniformly
    // to sky and objects before tone mapping (nothing upstream pre-exposes).
    hdrColor *= lightData.exposure;

    // Add bloom
    if (params.bloomEnabled != 0) {
        float3 bloom = bloomTexture.sample(smp, in.uv).rgb;
        hdrColor += bloom * params.bloomIntensity;
    }

    // Color grade (the "Look": contrast + saturation) in scene-linear, then the
    // tone map / view transform. Both stages run before the display encode, which
    // each tone mapper folds in — see applyGrade/tonemapACES/tonemapAgX.
    hdrColor = applyGrade(hdrColor, params.gradeContrast, params.gradeSaturation);
    float3 color = (params.tonemapOp == 1) ? tonemapAgX(hdrColor)
                                           : tonemapACES(hdrColor);
    // color is LINEAR here: the tone mappers now return scene-referred values
    // and the display transform is applied once, below.
    return encodeForTarget(color, params.targetEncodesSRGB, true);
}
