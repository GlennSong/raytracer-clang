// --- Bloom: threshold extract + progressive downsample/upsample ---

// Extract bright pixels and downsample to the first mip (QUARTER-res base; the
// chain halves from there). 9-tap 3x3 tent, energy-preserving — note this is NOT
// the 13-tap Jimenez/CoD kernel the comment used to claim; that one takes four
// extra half-texel-offset bilinear taps and is more stable against fireflies.
// Worth upgrading alongside a Karis average on this first pass.
kernel void bloomDownsample(
    texture2d<float, access::read>  src  [[texture(0)]],
    texture2d<float, access::write> dst  [[texture(1)]],
    constant BloomUniforms& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;

    // Map output texel to source center (each output pixel = 2x2 source block)
    float2 srcCoord = float2(gid) * 2.0 + 0.5;

    // 13-tap downsample: 4 corner samples + 4 edge samples + 1 center + 4 diagonal
    // Weights: center cross = 0.5, corners = 0.125 each (energy preserving)
    float3 a = src.read(uint2(clamp(srcCoord + float2(-1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 b = src.read(uint2(clamp(srcCoord + float2( 0, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 c = src.read(uint2(clamp(srcCoord + float2( 1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 d = src.read(uint2(clamp(srcCoord + float2(-1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 e = src.read(uint2(clamp(srcCoord,                  float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 f = src.read(uint2(clamp(srcCoord + float2( 1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 g = src.read(uint2(clamp(srcCoord + float2(-1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 h = src.read(uint2(clamp(srcCoord + float2( 0,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 i = src.read(uint2(clamp(srcCoord + float2( 1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;

    // Weighted average (box filter with center emphasis)
    float3 color = e * 0.25
                 + (b + d + f + h) * 0.125
                 + (a + c + g + i) * 0.0625;

    // Soft threshold, first pass only — the BRIGHT PASS. Everything below the
    // threshold contributes nothing; the knee is the soft shoulder so a pixel
    // crossing it fades in instead of popping.
    if (params.firstPass != 0) {
        float brightness = max(color.r, max(color.g, color.b));
        float soft = brightness - params.threshold + params.knee;
        soft = clamp(soft, 0.0, 2.0 * params.knee);
        soft = soft * soft / (4.0 * params.knee + 1e-5);
        float contribution = max(soft, brightness - params.threshold) / max(brightness, 1e-5);
        color *= max(contribution, 0.0);
    }

    dst.write(float4(color, 1.0), gid);
}

// Upsample and additively blend with the next-higher mip.
// Uses a 3x3 tent filter for smooth upsampling.
kernel void bloomUpsample(
    texture2d<float, access::read>  src     [[texture(0)]],  // smaller mip (being upsampled)
    texture2d<float, access::read>  higher  [[texture(1)]],  // larger mip to blend into
    texture2d<float, access::write> dst     [[texture(2)]],  // output (same size as higher)
    constant BloomUniforms& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    int dstW = int(dst.get_width());
    int dstH = int(dst.get_height());
    if (int(gid.x) >= dstW || int(gid.y) >= dstH) return;

    // Bilinear sample from smaller mip using 3x3 tent filter
    float2 srcCoord = (float2(gid) + 0.5) * float2(src.get_width(), src.get_height())
                    / float2(dstW, dstH) - 0.5;

    int srcW = int(src.get_width());
    int srcH = int(src.get_height());

    float3 sum = float3(0);
    // 3x3 tent: weights 1,2,1 / 2,4,2 / 1,2,1 (sum=16)
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int2 coord = int2(srcCoord) + int2(dx, dy);
            coord = clamp(coord, int2(0), int2(srcW - 1, srcH - 1));
            float w = float((2 - abs(dx)) * (2 - abs(dy)));
            sum += src.read(uint2(coord)).rgb * w;
        }
    }
    sum /= 16.0;

    // Add to higher-resolution mip
    float3 higherColor = higher.read(gid).rgb;
    dst.write(float4(higherColor + sum, 1.0), gid);
}
