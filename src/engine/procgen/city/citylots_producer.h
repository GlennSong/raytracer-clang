#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITYLOTS_PRODUCER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITYLOTS_PRODUCER_H

// The lot pre-pass of a LATTICE city level as a bundle producer (ADR-0084, milestone C).
//
// The sibling of lanelab's `lots` producer, for the other generator: a level whose roads come from a
// shape:"road" entity's `generate` recipe. It grows the same lots the loader's terrain pre-pass would,
// out of the world engine::cityPrePassForLevel derives from the level JSON, and writes them through the
// same lot codecs — parts split per render cell, so the loader never chunks at load.
//
// This producer lives in engine_core and registers unconditionally: it needs no lanelab code.

#include "engine/bundle/bake.h"

namespace engine {

extern const char* const kCityLotsProducerName;   // "citylots"
extern const char* const kCityLotsBuildTag;
extern const char* const kCityLotsSectionPrefix;  // "citylots/"

void registerCityLotsProducer();   // idempotent

}  // namespace engine

#endif
