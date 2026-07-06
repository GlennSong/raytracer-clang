#ifndef RAYTRACER_PROFILE_H
#define RAYTRACER_PROFILE_H

// Profiling macros (ADR-0068). With RT_ENABLE_PROFILER (CMake option, OFF by
// default) they emit Tracy client zones; otherwise they compile to nothing, so
// instrumentation costs zero in normal builds and stays in place permanently.
// "Measure before optimizing" (design principle #3) — this is the measuring.
//
//   RT_PROFILE_ZONE();            // scope zone named by the function
//   RT_PROFILE_ZONE_NAMED("x");   // scope zone with an explicit literal name
//   RT_PROFILE_FRAME();           // end-of-frame mark (once per frame loop)
//   RT_PROFILE_THREAD("worker");  // name the calling thread in captures
//
// Connect with the Tracy profiler UI (third_party/tracy, v0.13.1) while a
// RT_ENABLE_PROFILER build runs; capture is over the network, works headless.

#ifdef RT_ENABLE_PROFILER

#include <tracy/Tracy.hpp>

#define RT_PROFILE_ZONE() ZoneScoped
#define RT_PROFILE_ZONE_NAMED(name) ZoneScopedN(name)
#define RT_PROFILE_FRAME() FrameMark
#define RT_PROFILE_THREAD(name) tracy::SetThreadName(name)

#else

#define RT_PROFILE_ZONE()
#define RT_PROFILE_ZONE_NAMED(name)
#define RT_PROFILE_FRAME()
#define RT_PROFILE_THREAD(name)

#endif

#endif
