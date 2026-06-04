#ifndef RAYTRACER_CHECK_H
#define RAYTRACER_CHECK_H

#include "log.h"
#include <cstdlib>

// CHECK is always active; ASSERT compiles out under NDEBUG. Both log the failed
// expression with its location, then abort.
#define CHECK(cond)                                            \
    do {                                                       \
        if (!(cond)) {                                         \
            LOG_ERROR << "CHECK failed: " #cond " at "         \
                      << __FILE__ << ":" << __LINE__;          \
            std::abort();                                      \
        }                                                      \
    } while (0)

#ifdef NDEBUG
#define ASSERT(cond) ((void)0)
#else
#define ASSERT(cond)                                           \
    do {                                                       \
        if (!(cond)) {                                         \
            LOG_ERROR << "ASSERT failed: " #cond " at "        \
                      << __FILE__ << ":" << __LINE__;          \
            std::abort();                                      \
        }                                                      \
    } while (0)
#endif

#endif
