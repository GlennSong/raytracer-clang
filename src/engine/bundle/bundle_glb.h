#ifndef RAYTRACER_ENGINE_BUNDLE_BUNDLE_GLB_H
#define RAYTRACER_ENGINE_BUNDLE_BUNDLE_GLB_H

// A GLB view of any bundle (ADR-0084): the inspection format for Blender, not a load format. Works from
// section NAMES alone — every `<...>/cell/<cx>_<cz>/mesh/<material>` section is a PackedMesh — so it needs
// no knowledge of which producer wrote what. Per material: the cells of each material merged into one glTF
// mesh (one object per material in Blender). Per cell: one GLB per render cell holding that cell's
// materials, plus `index.json`, so a district can be imported alone. Scene extras carry the manifest, the
// small JSON sections, every `/roads/twin` as road JSON and every `/blocks/holes` as rings; node extras
// carry the material's collision flags.

#include "engine/bundle/bundle.h"
#include <functional>
#include <string>
#include <vector>

namespace engine {
namespace bundle {

// `progress`: 0..1 as materials (or cells) are written; return false to stop (the file so far is left).
using GlbProgressFn = std::function<bool(double)>;
bool writeBundleGlb(const Bundle& b, const std::string& path, std::string* err, const GlbProgressFn* progress = nullptr);
bool writeBundleGlbCells(const Bundle& b, const std::string& dir, std::string* err, std::vector<std::string>* files = nullptr, const GlbProgressFn* progress = nullptr);

}  // namespace bundle
}  // namespace engine

#endif
