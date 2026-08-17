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
//              0..2  twelve lateral offsets, four to a texel   blended in v
//              3     (styleRow, reserved, reserved, reserved)  nearest in v
//   styles   24 texels a row, one row per distinct style set   nearest
//              per slot: (style, width, gap, color), (dashOn, dashOff, wear, -)
//
// --- why there is no sampler here -------------------------------------------
//
// The offsets want blending between rings, which is what a linear sampler is
// for, and this file used one. It was wrong twice over.
//
// Precision: a sampler is addressed in normalised coordinates, so the row makes
// a round trip through (row + 0.5) / height and back. In f32 that costs about
// row * 6e-8 of row, and the error lands in the offset multiplied by the
// DIFFERENCE between the two rings — which at a taper is metres. Measured on
// the `lanes` demo at row 458 of 645: 0.27 mm. Harmless there, but the term
// grows with the atlas, and an atlas is the thing that grows with the city.
//
// Portability: float32-filterable is an OPTIONAL WebGPU feature. A shader that
// needs it to blend RGBA32F does not run on a browser that lacks it, and the
// fallback — dropping to nearest — silently steps the lane edges instead of
// tapering them.
//
// Two texel loads and an explicit mix cost one extra fetch and fix both: integer
// addressing has no round trip, the arithmetic is the same fp32 lerp
// PaintAtlas::sample does on the CPU, and textureLoad on an unfilterable format
// is core WebGPU.

const RL_OFFSET_TEXELS : i32 = 3;             // ceil(RL_MAX_BOUNDS / 4)
const RL_PROFILE_TEXELS : i32 = 4;            // + the style row index
const RL_STYLE_TEXELS_PER_SLOT : i32 = 2;
// paint_texture.h's kRowsPerTile. The atlas is taller than a texture may be, so
// its rows tile across the width instead of stacking.
const RL_ROWS_PER_TILE : i32 = 64;

@group(1) @binding(0) var rlProfileTex : texture_2d<f32>;
@group(1) @binding(1) var rlStyleTex : texture_2d<f32>;

// One texel of one atlas row, through the tiling.
fn rlProfileTexel(row : i32, texel : i32) -> vec4<f32> {
  let x = (row % RL_ROWS_PER_TILE) * RL_PROFILE_TEXELS + texel;
  let y = row / RL_ROWS_PER_TILE;
  return textureLoad(rlProfileTex, vec2<i32>(x, y), 0);
}

// The offsets, blended between rings by hand.
fn rlBlendOffsets(texel : i32, lo : i32, hi : i32, u : f32) -> vec4<f32> {
  let a = rlProfileTexel(lo, texel);
  let b = rlProfileTexel(hi, texel);
  return a + (b - a) * u;
}

// Fill the fragment's boundary array from the two textures. Returns the count,
// which is what rlEvaluateMarkings wants next to the array.
//
// `row` is the atlas-absolute row coordinate the vertex shader interpolated —
// fractional between rings. Rows are not uniformly spaced (a lane-section seam
// gets a ring pair straddling it), so it cannot be recomputed from s here; the
// mesher has to carry it.
fn rlFetchBoundaries(row : f32, bounds : ptr<function, array<RlBoundary, RL_MAX_BOUNDS>>) -> i32 {
  let profileRows = i32(textureDimensions(rlProfileTex, 0).y) * RL_ROWS_PER_TILE;
  let lo = clamp(i32(floor(row)), 0, profileRows - 1);
  let hi = clamp(lo + 1, 0, profileRows - 1);
  let u = row - floor(row);
  // Nearest, matched to the hardware rule PaintAtlas::sample also matches: a
  // texel's footprint runs from its centre minus half to plus half.
  let nearRow = clamp(i32(floor(row + 0.5)), 0, profileRows - 1);

  // The style row index is an index: nearest, never blended. A style code
  // halfway between Dashed and Double still selects a branch, so a blended
  // index fails silently rather than loudly.
  let styleRow = i32(rlProfileTexel(nearRow, RL_OFFSET_TEXELS).x);
  let styleRows = i32(textureDimensions(rlStyleTex, 0).y);
  let sr = clamp(styleRow, 0, styleRows - 1);

  var offsets : array<vec4<f32>, 3>;
  offsets[0] = rlBlendOffsets(0, lo, hi, u);
  offsets[1] = rlBlendOffsets(1, lo, hi, u);
  offsets[2] = rlBlendOffsets(2, lo, hi, u);

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
  // Perspective-correct, the default, and NOT @interpolate(linear) — which in
  // WGSL means noperspective. The row is a function of the station, and the
  // station is interpolated perspective-correct one location up; interpolating
  // the two differently makes the paint slide along the road with distance,
  // exactly where a road has the most pixels and the least tolerance for it.
  @location(1) row : f32,             // the atlas row, from the ring
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
