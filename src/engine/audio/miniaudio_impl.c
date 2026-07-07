/* The single miniaudio implementation TU (ADR-0069). Vendored C compiled as C
 * with warnings off, like Lua; every other file includes only the header. */
#define MINIAUDIO_IMPLEMENTATION

/* Trim what the engine doesn't use: no encoding (we never write audio files)
 * and no generation API (noise/waveform generators — procgen synthesizes its
 * own PCM through AudioEngine::createClip). */
#define MA_NO_ENCODING
#define MA_NO_GENERATION

#include <miniaudio/miniaudio.h>
