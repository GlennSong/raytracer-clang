#include "diag.h"

#include <atomic>
#include <mutex>

namespace roadlab {

namespace {

// Counters live in a fixed array rather than in the FallbackSite objects
// themselves so the census can be read and reset without chasing pointers into
// whatever translation units happened to register. `slot` is handed out once per
// site at static-init time, under the registry lock; after that every hit is a
// single relaxed increment with no locking at all.
constexpr int kMaxSites = 128;

struct Registry {
    std::mutex mu;
    std::vector<std::string> names;
    std::atomic<long> counts[kMaxSites];

    Registry() {
        for (auto& c : counts) c.store(0, std::memory_order_relaxed);
    }

    int reserve(const char* name) {
        std::lock_guard<std::mutex> lock(mu);
        for (size_t i = 0; i < names.size(); ++i)
            if (names[i] == name) return int(i);
        if (names.size() >= kMaxSites) return -1;
        names.push_back(name);
        return int(names.size()) - 1;
    }

    std::vector<FallbackCount> snapshot() {
        std::lock_guard<std::mutex> lock(mu);
        std::vector<FallbackCount> out;
        out.reserve(names.size());
        for (size_t i = 0; i < names.size(); ++i)
            out.push_back({names[i], counts[i].load(std::memory_order_relaxed)});
        return out;
    }

    long lookup(const char* name) {
        std::lock_guard<std::mutex> lock(mu);
        for (size_t i = 0; i < names.size(); ++i)
            if (names[i] == name) return counts[i].load(std::memory_order_relaxed);
        return 0;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mu);
        for (auto& c : counts) c.store(0, std::memory_order_relaxed);
    }
};

// Two registries, because a fallback count is only readable next to the number
// of calls that could have produced it.
Registry& fallbacks() {
    static Registry r;
    return r;
}

Registry& calls() {
    static Registry r;
    return r;
}

}  // namespace

FallbackSite::FallbackSite(const char* name) : name_(name), slot_(fallbacks().reserve(name)) {}

void FallbackSite::hit() {
    if (slot_ >= 0) fallbacks().counts[slot_].fetch_add(1, std::memory_order_relaxed);
}

long FallbackSite::count() const {
    return slot_ >= 0 ? fallbacks().counts[slot_].load(std::memory_order_relaxed) : 0;
}

CallSite::CallSite(const char* name) : slot_(calls().reserve(name)) {}

void CallSite::hit() {
    if (slot_ >= 0) calls().counts[slot_].fetch_add(1, std::memory_order_relaxed);
}

std::vector<FallbackCount> fallbackCensus() { return fallbacks().snapshot(); }
std::vector<FallbackCount> callCensus() { return calls().snapshot(); }
long fallbackCount(const char* site) { return fallbacks().lookup(site); }
long callCount(const char* site) { return calls().lookup(site); }

void resetFallbackCensus() {
    fallbacks().reset();
    calls().reset();
}

long totalFallbacks() {
    long total = 0;
    for (const FallbackCount& fc : fallbackCensus()) total += fc.count;
    return total;
}

}  // namespace roadlab
