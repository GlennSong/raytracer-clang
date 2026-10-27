#include "event_types.h"

#include "audio/audio_engine.h"

namespace engine {

void describeProperties(PlaySound& e, PropertyVisitor& v) {
    v.field(FieldMeta("clip"), e.clip);
    v.field(FieldMeta("position").units("m"), e.position);
    v.field(FieldMeta("spatial"), e.spatial);
    v.field(FieldMeta("volume").range(0, 4), e.volume);
    v.field(FieldMeta("pitch").range(0.1, 4), e.pitch);
    v.field(FieldMeta("range").units("m"), e.range);
}

void describeEngineEvents(EventBus& bus) {
    describeEvent<PlaySound>(bus, "PlaySound",
                             "Play a sound cue: a clip path or registered name, flat or at a position.");
}

}  // namespace engine
