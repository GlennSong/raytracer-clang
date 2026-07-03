// Mipmap downsample: a fullscreen triangle that linearly samples the previous
// (larger) mip level; run once per level to fill a texture's mip chain.
@group(0) @binding(0) var srcTex : texture_2d<f32>;
@group(0) @binding(1) var srcSamp : sampler;
struct VOut { @builtin(position) pos : vec4<f32>, @location(0) uv : vec2<f32> };
@vertex
fn vs_blit(@builtin(vertex_index) vid : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vid], 0.0, 1.0);
  o.uv = vec2<f32>(p[vid].x * 0.5 + 0.5, 0.5 - p[vid].y * 0.5);
  return o;
}
@fragment
fn fs_blit(in : VOut) -> @location(0) vec4<f32> {
  return textureSampleLevel(srcTex, srcSamp, in.uv, 0.0);
}
