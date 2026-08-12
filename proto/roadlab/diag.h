#ifndef ROADLAB_DIAG_H
#define ROADLAB_DIAG_H

// A census of the places this system quietly substitutes a lesser answer.
//
// Every non-trivial geometric routine here has a path where it cannot do the
// thing it was asked and returns something safe instead: a straight line where a
// biarc was wanted, a centroid fan where ear clipping failed, `false` where a
// projection did not converge. Individually each is the right call — a bounded
// wrong-looking answer beats an unbounded one, and a caller that can cope should
// be allowed to.
//
// Collectively they are a blind spot, and an expensive one. `Spine::toST` spent
// the whole life of this prototype returning "no answer" on EVERY query, because
// its Newton step had a sign error and a residual guard turned the diverged
// result into a rejection. Nothing failed. Terrain silently stopped conforming
// to roads, `Network::sample` silently found nothing, and every caller took its
// fallback path exactly as designed. The bug was invisible because a fallback
// firing looks identical to a fallback not being needed.
//
// So: count them. A fallback that fires zero times across every demo and a
// spread of generated cities is dead code or a genuine guard. One that fires on
// every call is a routine that does not work. The interesting ones are in
// between, and you cannot see any of them without a counter.
//
// Cost is one relaxed atomic increment on a path that was already the slow one.

#include <cstdint>
#include <string>
#include <vector>

namespace roadlab {

// Registered once per call site, via the RL_FALLBACK macro below.
class FallbackSite {
public:
    explicit FallbackSite(const char* name);
    void hit();
    const char* name() const { return name_; }
    long count() const;

private:
    const char* name_;
    int slot_;
};

struct FallbackCount {
    std::string site;
    long count = 0;
};

// Every registered site, in registration order, with its current count. Sites
// that have never fired are included — a zero is a result.
std::vector<FallbackCount> fallbackCensus();
void resetFallbackCensus();
long fallbackCount(const char* site);
long totalFallbacks();

// Also counts the CALLS to a routine that has a fallback, so a census can report
// a rate rather than a bare count. "toST rejected 40 times" means nothing until
// you know whether it was called 40 times or 40 million.
class CallSite {
public:
    explicit CallSite(const char* name);
    void hit();

private:
    int slot_;
};

std::vector<FallbackCount> callCensus();
long callCount(const char* site);

}  // namespace roadlab

// Both macros are safe to use on any thread: the function-local static is
// initialised exactly once by the language, and the counter behind it is atomic.
#define RL_FALLBACK(name)                              \
    do {                                               \
        static ::roadlab::FallbackSite rlFbSite(name); \
        rlFbSite.hit();                                \
    } while (0)

#define RL_CALLED(name)                            \
    do {                                           \
        static ::roadlab::CallSite rlCallSite(name); \
        rlCallSite.hit();                          \
    } while (0)

#endif
