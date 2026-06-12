#ifndef RAYTRACER_ENGINE_LEVEL_WRITER_H
#define RAYTRACER_ENGINE_LEVEL_WRITER_H

#include "world.h"
#include <string>

namespace engine {

// Serializes the world back to a level JSON — the other half of the
// document-based edit/play loop (docs/edit-mode-plan.md). Only entities
// carrying a SourceSpec are written (runtime spawns never leak into the
// document). Sections the editor doesn't own (version, environment, lighting,
// player) are preserved verbatim from the existing file: the writer patches
// the "entities" array rather than regenerating the document.
struct LevelWriter {
    static bool save(const std::string& path, World& world);
};

}  // namespace engine

#endif
