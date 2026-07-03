// Bloom: a half-res bright-pass (soft-knee threshold, ported from post.metal's
// bloomDownsample) + a separable Gaussian blur, added back in the composite.
// One bind-group layout { src texture, sampler, uniform } shared by both
// fragment entry points; the blur direction rides the uniform.
struct BloomU {
  params : vec4<f32>,   // x threshold, y knee, z intensity, w mode (0 bright, 1 blur)
  texel  : vec4<f32>,   // xy = blur step in uv (texelSize * direction)
};
@group(0) @binding(0) var srcTex : texture_2d<f32>;
@group(0) @binding(1) var srcSamp : sampler;
@group(0) @binding(2) var<uniform> u : BloomU;

@vertex
fn vs_bloom(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}

// Bright-pass: src is full-res, dst is half-res. Soft-knee threshold.
@fragment
fn fs_bright(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  let srcDim = vec2<f32>(textureDimensions(srcTex));
  let uv = (fc.xy * 2.0) / srcDim;          // half-res frag -> full-res uv
  let c = textureSampleLevel(srcTex, srcSamp, uv, 0.0).rgb;
  let brightness = max(c.r, max(c.g, c.b));
  let knee = max(u.params.y, 1e-4);
  var soft = brightness - u.params.x + knee;
  soft = clamp(soft, 0.0, 2.0 * knee);
  soft = soft * soft / (4.0 * knee + 1e-5);
  let contribution = max(soft, brightness - u.params.x) / max(brightness, 1e-5);
  return vec4<f32>(c * contribution, 1.0);
}

// Separable 9-tap Gaussian along u.texel; src and dst are the same (half) size.
@fragment
fn fs_blur(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  let dim = vec2<f32>(textureDimensions(srcTex));
  let uv = fc.xy / dim;
  let w = array<f32, 5>(0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);
  var sum = textureSampleLevel(srcTex, srcSamp, uv, 0.0).rgb * w[0];
  for (var i = 1; i < 5; i = i + 1) {
    let o = u.texel.xy * f32(i);
    sum += textureSampleLevel(srcTex, srcSamp, uv + o, 0.0).rgb * w[i];
    sum += textureSampleLevel(srcTex, srcSamp, uv - o, 0.0).rgb * w[i];
  }
  return vec4<f32>(sum, 1.0);
}
