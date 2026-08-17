#ifndef ROADLAB_PAINT_TEXTURE_H
#define ROADLAB_PAINT_TEXTURE_H

// The baked boundaries as textures a fragment shader can actually sample.
//
// paint_bake.h answered "what does a station look like as a flat record"; this
// answers "where does the fragment get it from". The measurement in the last
// stage settled the choice: nine painted boundaries is 72 floats a station,
// which is far past any vertex format, and the eighteen RGBA texels it comes to
// instead are nothing. So: a texture.
//
// TWO textures, and the split is the whole design.
//
//   profile  RGBA32F, 4 texels per station ring.
//              texels 0-2  twelve lateral offsets, four to a texel
//              texel  3    (styleRow, reserved, reserved, reserved)
//            Texels 0-2 are BLENDED between adjacent rings, because `t` is the
//            one field that is continuous in s and genuinely wants
//            interpolating. Texel 3 is taken from the nearest ring only — an
//            index cannot be blended.
//
//   styles   RGBA32F, 24 texels wide, one row per DISTINCT style set.
//              per slot: (style, width, gap, color), (dashOn, dashOff, wear, -)
//            Nearest, always, because a blend between two styles is not a third
//            style, it is a number that selects a branch at random.
//
// Both are read with textureLoad and the blend is done in the shader, NOT with
// a hardware linear sampler. Two reasons, both found by measurement rather than
// argued: a sampler addresses in normalised coordinates, and that round trip
// costs about row * 6e-8 of row, which lands in the offset multiplied by the
// difference between the two rings — 0.27 mm on `lanes`, and growing with the
// atlas. And float32-filterable is an optional WebGPU feature, so a filtered
// RGBA32F shader simply does not run on some browsers. Blending by hand is one
// extra fetch, is bit-identical to PaintAtlas::sample, and is what makes the
// tiling below possible at all.
//
// The indirection through `styleRow` is what makes this affordable. Style,
// width, dash pattern and colour are piecewise constant — they change at lane
// section seams and nowhere else — so a row-per-station style texture stores the
// same 96 floats a hundred times over. Deduplicating across the whole network
// collapses it by roughly the ratio of stations to distinct cross-sections,
// which on the demos is 20:1 and on a real city, where a hundred streets share
// one profile, is far more. It is the difference between a texture that scales
// with kilometres of road and one that scales with the number of road types.
//
// --- the contract with the mesher -------------------------------------------
//
// Station rows are NOT uniformly spaced — a lane-section seam gets a row pair
// straddling it (see bakeBoundaryStrip), and their positions are wherever the
// seams fall. So a fragment cannot compute its row from s. Instead:
//
//   * The mesher emits one ring per row, at PaintProfile::stations[i].
//   * Ring i writes `rowBase + i` into a vertex channel.
//   * The rasteriser interpolates it; the fragment has its row coordinate free.
//
// This is the same trick road_mesh.cpp already uses for its road-local U, and it
// is why the seam rings matter twice over: they keep the slot numbering stable
// across a step AND they give the mesh somewhere to put the discontinuity. What
// they do not do is remove it — a lane-section boundary is a real discontinuity,
// confined to the 2 mm between the two rings. tests.cpp measures that width
// rather than assuming it.

#include "paint_bake.h"

namespace roadlab {

// Four offsets to a texel, so twelve slots is three texels, plus one for the
// style row index.
constexpr int kOffsetTexels = (kMaxBakedBoundaries + 3) / 4;
constexpr int kProfileTexelsPerRow = kOffsetTexels + 1;
// Seven of the eight remaining fields, so two texels a slot.
constexpr int kStyleTexelsPerSlot = 2;
constexpr int kStyleTexelsPerRow = kMaxBakedBoundaries * kStyleTexelsPerSlot;

// --- fitting the atlas into an actual texture -------------------------------
//
// One row per station ring means the metro scene's atlas is 19786 rows, and a
// texture is at most 8192 on a side in core WebGPU (16384 on the device this was
// caught on, which it also exceeded). A row-per-station atlas laid out as a
// single column does not scale to a city, full stop.
//
// So the rows tile: kRowsPerTile of them side by side, wrapping down. 19786 rows
// becomes 256 x 310 texels, and the growth goes into a dimension that has room.
//
// This is only possible because the fetch loads texels by integer coordinate and
// blends them itself. A hardware linear filter blends horizontal neighbours too,
// and a horizontal neighbour here is a different station — or a different road.
// Tiling and hardware filtering are mutually exclusive, and the fetch picked the
// one that scales.
constexpr int kRowsPerTile = 64;

struct PaintProfile {
    std::vector<double> stations;   // one per row; ALSO the mesher's ring list
    std::vector<float> profile;     // rows * kProfileTexelsPerRow * 4
    int rowBase = 0;                // this road's first row in the atlas

    int rows() const { return int(stations.size()); }

    // The row coordinate for a station, in the units the vertex channel carries:
    // row index, fractional between rings, atlas-absolute.
    double rowCoord(double s) const;
};

// Every road's profile stacked into one pair of textures.
//
// Roads have wildly different lengths, so a fixed-height page per road would
// waste most of the atlas; stacking rows and handing each road its rowBase
// wastes none.
struct PaintAtlas {
    std::vector<PaintProfile> profiles;   // parallel to Network::roads()
    std::vector<float> profileTex;        // rows * kProfileTexelsPerRow * 4
    std::vector<float> styleTex;          // styleRows * kStyleTexelsPerRow * 4
    int rows = 0;
    int styleRows = 0;

    int profileWidth() const { return kProfileTexelsPerRow; }
    int styleWidth() const { return kStyleTexelsPerRow; }
    size_t bytes() const { return (profileTex.size() + styleTex.size()) * sizeof(float); }

    // `profileTex` above is the logical form: one atlas row after another. These
    // are the shape it takes as a texture, tiled kRowsPerTile wide.
    int imageWidth() const { return kRowsPerTile * kProfileTexelsPerRow; }
    int imageHeight() const { return (rows + kRowsPerTile - 1) / kRowsPerTile; }
    // The tiled image, zero-padded out to the last tile row.
    std::vector<float> profileImage() const;

    // Exactly what the fragment shader does: linear along the offset texels,
    // nearest for the style row index, nearest into the style table, unpack,
    // skip padding. `row` is atlas-absolute, matching PaintProfile::rowCoord.
    int sample(int roadIndex, double row, RlBoundary* out, int maxCount) const;
};

// `step` is the mesher's ring spacing; 2 m matches what tessellate.cpp uses.
PaintAtlas bakePaintAtlas(const Network& net, double step);
// One road, same layout — the atlas of a network of one.
PaintAtlas bakePaintAtlas(const Road& road, double step);

}  // namespace roadlab

#endif
