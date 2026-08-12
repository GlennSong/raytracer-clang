// The WebGPU side of the profile texture: bindings, the fetch, and a fragment
// entry point that shades a road with it.
//
// HAND-WRITTEN, unlike rl_paint.wgsl next to it. That file is generated from
// rl_paint.h because both have to be the same evaluator; this one has no C++
// counterpart to drift from — paint_texture.h describes the layout, and this is
// the only place that layout is expressed as a shader.
//
// It exists mostly so the contract is checked rather than described. Validating
// the generated evaluator alone proves it parses and type-checks; it does not
// prove anything about how a fragment is supposed to reach it. Concatenated
// after rl_paint.wgsl this whole thing goes through naga, so the bindings, the
// texel arithmetic and the pointer-to-array call all have to be real.
//
//   cat rl_paint.wgsl rl_paint_sampler.wgsl | naga --stdin-file-path x.wgsl
//
// Layout, from paint_texture.h:
//
//   profile  4 texels a row, one row per mesh ring
//              0..2  twelve lateral offsets, four to a texel   LINEAR in v
//              3     (styleRow, reserved, reserved, reserved)  NEAREST in v
//   styles   24 texels a row, one row per distinct style set   NEAREST
//              per slot: (style, width, gap, color), (dashOn, dashOff, wear, -)

const RL_OFFSET_TEXELS : i32 = 3;             // ceil(RL_MAX_BOUNDS / 4)
const RL_PROFILE_TEXELS : i32 = 4;            // + the style row index
const RL_STYLE_TEXELS_PER_SLOT : i32 = 2;

@group(1) @binding(0) var rlProfileTex : texture_2d<f32>;
@group(1) @binding(1) var rlStyleTex : texture_2d<f32>;
@group(1) @binding(2) var rlLinear : sampler;

// The offsets, blended between rings by the hardware.
//
// Sampling at an exact texel centre in u is what lets ONE linear sampler serve a
// texture whose two axes want different treatment: the filter still runs
// horizontally, but a centre sample has weight one, so only v actually blends.
fn rlSampleOffsets(texel : i32, row : f32) -> vec4<f32> {
  let dims = vec2<f32>(textureDimensions(rlProfileTex, 0));
  let uv = vec2<f32>((f32(texel) + 0.5) / dims.x, (row + 0.5) / dims.y);
  return textureSampleLevel(rlProfileTex, rlLinear, uv, 0.0);
}

// Fill the fragment's boundary array from the two textures. Returns the count,
// which is what rlEvaluateMarkings wants next to the array.
//
// `row` is the atlas-absolute row coordinate the vertex shader interpolated —
// fractional between rings. Rows are not uniformly spaced (a lane-section seam
// gets a ring pair straddling it), so it cannot be recomputed from s here; the
// mesher has to carry it.
fn rlFetchBoundaries(row : f32, bounds : ptr<function, array<RlBoundary, RL_MAX_BOUNDS>>) -> i32 {
  let profileRows = i32(textureDimensions(rlProfileTex, 0).y);
  let nearRow = clamp(i32(floor(row + 0.5)), 0, profileRows - 1);

  // The style row index is an index: nearest, never blended. A style code
  // halfway between Dashed and Double still selects a branch, so a filtered
  // index fails silently rather than loudly.
  let styleRow = i32(textureLoad(rlProfileTex, vec2<i32>(RL_OFFSET_TEXELS, nearRow), 0).x);
  let styleRows = i32(textureDimensions(rlStyleTex, 0).y);
  let sr = clamp(styleRow, 0, styleRows - 1);

  var offsets : array<vec4<f32>, 3>;
  offsets[0] = rlSampleOffsets(0, row);
  offsets[1] = rlSampleOffsets(1, row);
  offsets[2] = rlSampleOffsets(2, row);

  var count : i32 = 0;
  for (var k : i32 = 0; k < RL_MAX_BOUNDS; k = k + 1) {
    let a = textureLoad(rlStyleTex, vec2<i32>(k * RL_STYLE_TEXELS_PER_SLOT, sr), 0);
    // style < 0 is padding: a slot that holds no boundary, which is NOT the
    // same as a boundary that holds no paint. Conflating them renumbers every
    // lane past the first unmarked one.
    if (a.x < -0.5) { continue; }
    let b = textureLoad(rlStyleTex, vec2<i32>(k * RL_STYLE_TEXELS_PER_SLOT + 1, sr), 0);

    var e : RlBoundary;
    e.t = offsets[k / 4][k % 4];
    e.style = a.x;
    e.width = a.y;
    e.gap = a.z;
    e.color = a.w;
    e.dashOn = b.x;
    e.dashOff = b.y;
    e.wear = b.z;
    (*bounds)[count] = e;
    count = count + 1;
  }
  return count;
}

// --- an entry point that uses it --------------------------------------------

struct RlFragmentIn {
  @builtin(position) clipPos : vec4<f32>,
  @location(0) st : vec2<f32>,        // road station and lateral offset, METRES
  @location(1) @interpolate(linear) row : f32,   // the atlas row, from the ring
  @location(2) wear : vec2<f32>,      // x road wear 0..1, y wheel-path nearness
};

// The filter width the analytic markings need: how much road one pixel covers,
// laterally. Screen-space derivatives of the road-local coordinate give it for
// free, which is the whole reason a 100 mm stripe stays legible at 200 m with no
// mip chain and no shimmer.
fn rlFilterWidth(st : vec2<f32>) -> f32 {
  let dx = dpdx(st);
  let dy = dpdy(st);
  return max(length(vec2<f32>(dx.y, dy.y)), 1e-4);
}

@fragment
fn rlRoadFragment(input : RlFragmentIn) -> @location(0) vec4<f32> {
  var bounds : array<RlBoundary, RL_MAX_BOUNDS>;
  let count = rlFetchBoundaries(input.row, &bounds);

  // The blotch mask is an INPUT to the evaluator (see rl_paint.h): every backend
  // already ships value noise and none of them agree bit for bit, so wear is the
  // one thing each spends its own. Here it stands in as a constant; the engine
  // passes surfAsphalt's own vnoise2.
  let wearMask = 1.0;
  let paint = rlEvaluateMarkings(&bounds, count, input.st.x, input.st.y,
                                 rlFilterWidth(input.st), input.wear.x, wearMask,
                                 input.wear.y);

  let asphalt = vec3<f32>(0.09, 0.09, 0.10);
  let rgb = mix(asphalt, vec3<f32>(paint.r, paint.g, paint.b), paint.coverage);
  return vec4<f32>(rgb, 1.0);
}
