// A whole roadlab scene, drawn. Vertex stage, fragment stage, camera.
//
// This is the payload of the realtime question. rl_paint.wgsl says the evaluator
// translates; rl_paint_sampler.wgsl says a fragment can reach it; neither draws
// anything. This does — one draw call over the exported mesh, evaluating the
// markings per fragment with no marking texture anywhere in the pipeline.
//
// ONE file, read by both `web/roadlab-paint.html` and
// `tools/roadlab-web-render.py`. The page is what a person looks at and the
// script is what runs headlessly in CI; if they had a shader each, the one
// nobody looks at would be the one that stayed correct.
//
// Concatenated after rl_paint.wgsl and rl_paint_sampler.wgsl:
//
//   cat rl_paint.wgsl rl_paint_sampler.wgsl rl_paint_view.wgsl | naga --stdin-file-path x.wgsl
//
// The vertex layout is webexport.h's, 12 floats:
//   0 pos.xyz   1 normal.xyz   2 (s, t, row)   3 color.rgb

struct RlView {
  viewProj  : mat4x4<f32>,
  sun       : vec4<f32>,   // xyz direction TO the sun, w unused
  params    : vec4<f32>,   // x paint on/off, y road wear 0..1, zw unused
};

@group(0) @binding(0) var<uniform> rlView : RlView;

struct RlVertexOut {
  @builtin(position) clip : vec4<f32>,
  @location(0) st : vec2<f32>,
  // Perspective-correct like `st`, for the reason spelled out in
  // rl_paint_sampler.wgsl: they describe the same point and must agree.
  @location(1) row : f32,
  @location(2) normal : vec3<f32>,
  @location(3) color : vec3<f32>,
};

@vertex
fn rlViewVertex(@location(0) pos : vec3<f32>,
                @location(1) normal : vec3<f32>,
                @location(2) stRow : vec3<f32>,
                @location(3) color : vec3<f32>) -> RlVertexOut {
  var out : RlVertexOut;
  out.clip = rlView.viewProj * vec4<f32>(pos, 1.0);
  out.st = stRow.xy;
  out.row = stRow.z;
  out.normal = normal;
  out.color = color;
  return out;
}

@fragment
fn rlViewFragment(in : RlVertexOut) -> @location(0) vec4<f32> {
  var albedo = in.color;

  // How much road one pixel covers laterally. Screen-space derivatives give it
  // for free, and it is the whole reason a 100 mm stripe stays legible at 200 m
  // with no mip chain and no shimmer.
  //
  // Hoisted ABOVE the branch, not folded into it where it reads better. WGSL
  // requires derivative builtins to sit in uniform control flow: a quad's four
  // fragments must all reach the dpdx or the neighbour differences are taken
  // against a value that was never written. Tint rejects the nested version
  // outright; naga accepts it, so this only shows up in a browser. Which is the
  // argument for having run it in one.
  let fw = max(length(vec2<f32>(dpdx(in.st).y, dpdy(in.st).y)), 1e-4);

  // row < 0 is the export's way of saying "no paint here" — ground, structures,
  // props, and junction pads, whose boundaries the bake does not produce. They
  // share the pipeline and skip the fetch entirely.
  //
  // Toggling params.x changes ONLY this branch, which is what makes the two
  // timings in the viewer a measurement of the evaluator rather than of two
  // different scenes.
  if (in.row >= 0.0 && rlView.params.x > 0.5) {
    var bounds : array<RlBoundary, RL_MAX_BOUNDS>;
    let count = rlFetchBoundaries(in.row, &bounds);

    // wearMask is 1.0 here. In the engine it is surfAsphalt's own value noise;
    // every backend ships noise and none agree bit for bit, so rl_paint.h takes
    // it as an input rather than picking one (see its header).
    let paint = rlEvaluateMarkings(&bounds, count, in.st.x, in.st.y, fw,
                                   rlView.params.y, 1.0, 0.0);
    albedo = mix(albedo, vec3<f32>(paint.r, paint.g, paint.b), paint.coverage);
  }

  // Deliberately plain: a headlamp term and a sky term, no shadows, no
  // tonemapping. The question this page answers is what the marking evaluator
  // costs per fragment, and every watt spent elsewhere is noise in that number.
  let n = normalize(in.normal);
  let lambert = max(dot(n, normalize(rlView.sun.xyz)), 0.0);
  let sky = 0.5 + 0.5 * n.y;
  let lit = albedo * (0.25 * sky + 0.85 * lambert);
  return vec4<f32>(pow(lit, vec3<f32>(1.0 / 2.2)), 1.0);
}
