#ifndef ROADLAB_WEBEXPORT_H
#define ROADLAB_WEBEXPORT_H

// Everything a GPU needs to draw a roadlab scene, as two files.
//
// The prototype renders through its own software rasterizer, which is the right
// tool for a still and the wrong one for the question "is the marking evaluator
// fast enough to run per fragment". Answering that needs the shader running on
// real hardware in a real frame, and the engine cannot host it yet: its material
// has five texture slots and all five are spoken for, and the vertex format has
// no room for the atlas row. So the prototype answers it itself, the same way it
// already answers correctness — by exporting what a renderer would need and
// letting something outside the engine run it.
//
// `<prefix>.bin`   interleaved vertices, then indices, then the two atlas
//                  textures, all little-endian float32/uint32, no padding.
// `<prefix>.json`  the manifest: byte offsets, counts, texture dimensions and a
//                  camera the scene looks good from.
//
// The vertex carries the ATLAS ROW, not just (s, t). The row cannot be recovered
// from s in a shader: rows are not uniformly spaced, because the bake lands a
// pair straddling every lane-section seam so no filter step blends across one.
// Whatever builds the mesh has to carry it, which is exactly the vertex-format
// change the engine would need — so the export makes that requirement concrete
// rather than theoretical.

#include "paint_texture.h"
#include "scene.h"

#include <string>

namespace roadlab {

struct WebExportStats {
    size_t vertices = 0;
    size_t indices = 0;
    int profileRows = 0;
    int styleRows = 0;
    size_t binBytes = 0;
    size_t sampleCases = 0;   // rows in <prefix>.samples.json
    size_t seamCases = 0;     // of those, ones straddling a style-row change
};

// Floats per vertex in the interleaved buffer:
//   pos.xyz  normal.xyz  s  t  row  color.rgb
constexpr int kWebVertexFloats = 12;

// Writes `<prefix>.bin` and `<prefix>.json`. Returns false if either cannot be
// opened; `error` says which.
bool writeWebExport(const Scene& sc, const std::string& prefix, WebExportStats& stats,
                    std::string& error);

}  // namespace roadlab

#endif
