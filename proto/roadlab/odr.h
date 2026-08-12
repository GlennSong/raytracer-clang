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
// `dir` is the strip's travel direction; a Turn strip with dir 0 is a two-way
// left-turn lane, which OpenDRIVE spells "bidirectional".
const char* odrLaneType(StripKind kind, int dir = 1);
const char* odrRoadMarkType(MarkStyle style);
const char* odrRoadMarkColor(PaintColor color);

// --- import ---------------------------------------------------------------

// What the reader could and could not honour. Import is the half that meets data
// nobody here wrote, so it reports rather than guesses silently.
struct OdrImportReport {
    std::string name;               // <header name>
    int roads = 0;
    int junctions = 0;
    int laneSections = 0;
    int lanes = 0;
    int approximatedGeometry = 0;   // poly3 / paramPoly3, sampled into segments
    int extraWidthRecords = 0;      // lanes with more than one <width>; first used
    double maxGeometryDrift = 0;    // metres between a declared pose and the chained one
    std::vector<std::string> notes;
};

// Read a .xodr into a Network. Junctions are marked `imported`, so build()
// adopts them rather than re-resolving them: the file already trimmed the arms
// and produced the connectors, and doing it again would duplicate both.
bool readOpenDrive(const std::string& path, Network& out, std::string& error,
                   OdrImportReport* report = nullptr);
bool openDriveFromString(const std::string& doc, Network& out, std::string& error,
                         OdrImportReport* report = nullptr);

StripKind stripKindFromOdr(const std::string& laneType);
SurfaceKind surfaceKindFromOdr(const std::string& material);
JunctionControl junctionControlFromOdr(const std::string& name);
MarkStyle markStyleFromOdr(const std::string& roadMarkType);
PaintColor paintColorFromOdr(const std::string& roadMarkColor);

}  // namespace roadlab

#endif
