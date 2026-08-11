#ifndef ROADLAB_ODR_H
#define ROADLAB_ODR_H

// OpenDRIVE (.xodr) export.
//
// The model in this prototype was built to be convergent with OpenDRIVE on
// purpose — reference line of line/arc/clothoid primitives, elevation and
// superelevation as cubics in s, lane sections of width polynomials, signed lane
// ordinals, junctions as sets of connecting roads. Writing the exporter is
// therefore mostly a transcription, and that is the point: if it had needed a
// translation layer, the model would have been drifting away from the one
// standard the whole industry already reads.
//
// What it buys: every OpenDRIVE viewer, every traffic simulator that speaks the
// format, and — the reason I want it most — the ability to import real road
// networks and point the junction solver at a real cloverleaf instead of at
// scenes I wrote myself.
//
// Coordinates line up without a transform. This prototype is Y-up with the plan
// in XZ and heading measured from +X toward +Z; OpenDRIVE is Z-up with the plan
// in XY and heading from +X toward +Y. Both rotate the first plan axis toward
// the second, and both put +t to the left, so (x, z, y) maps straight onto
// (x, y, z) with the same signs — including superelevation, where a positive
// value raises the left side in both.

#include "network.h"
#include "junction.h"
#include <string>

namespace roadlab {

struct OdrOptions {
    std::string name = "roadlab";
    std::string date = "";      // empty = omitted, so output is byte-reproducible
    bool includeConnectors = true;
    bool includeObjects = true;   // signals and signs as <objects>/<signals>
    int precision = 10;
};

// The document as a string. Roads are exported over their ACTIVE WINDOW only:
// a junction-trimmed arm or a split piece becomes a road that starts at s = 0,
// with every piecewise function re-based to match.
std::string openDriveString(const Network& net, const OdrOptions& opt = {});

bool writeOpenDrive(const Network& net, const std::string& path, std::string& error,
                    const OdrOptions& opt = {});

// The OpenDRIVE lane type for one of our strips, and the roadMark spelling for
// one of our markings. Exposed because they are the only genuinely lossy part of
// the mapping and worth being able to inspect.
const char* odrLaneType(StripKind kind);
const char* odrRoadMarkType(MarkStyle style);
const char* odrRoadMarkColor(PaintColor color);

}  // namespace roadlab

#endif
