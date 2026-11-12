#ifndef RAYTRACER_APPS_DRIVING_TRACE_H
#define RAYTRACER_APPS_DRIVING_TRACE_H

// SEEING WHY (driving-model-design.md 13; issue #256): what each driven car decided recently, and its DRIVER CARD.
//
// THE TRACE: each driven car keeps a short ring of its last kTraceLength decisions. An entry is written only when what
// limits the car CHANGES (another kind of limit, another rule, another car ahead), never on a tick where nothing did.
// An entry keeps the tick and the clock, what held the car back (its Binding), its speed and where it was, and the
// reason behind a rule's answer: which rule, and the values of what it read, so explain() can print it later. The
// ring lives on the trip, so it goes when the trip ends.
//
// While ticking nothing here builds or keeps a string: entries are numbers. Text is made only when someone asks
// (describeCar). Writing the trace never changes what a car does: Driving's stateHash leaves it out and is the same
// with tracing on or off (DrivingSettings::trace); traceHash folds it on its own.
//
// THE DRIVER CARD (describeCar): plain text about one car, built on demand: who it is, where it is going, where it is
// and how fast, what limits it now, what its driver holds (with the rules' reasons), and its recent decisions oldest
// first. The class name comes from the vehicle data and is only printed. Who the driver is and why they are driving
// is the host's to add: the card prints the host's opaque person number only.

#include "apps/driving/decide.h"          // Decision, Reason, ReadValue, CompiledRules, explain
#include "apps/driving/junction_plan.h"   // PlanNote: a junction plan made or dropped
#include "apps/driving/traffic_state.h"   // CarId, Binding

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace driving {

class Driving;

using engine::Real;

// How many decisions a driven car's ring keeps.
constexpr std::size_t kTraceLength = 8;
// How many of a rule's read values an entry keeps (a rule that reads more keeps the first this many, and says so).
constexpr std::size_t kMaxTracedValues = 4;

// THE REASON BEHIND ONE ANSWER, copied out of a Decision (whose values are reused by the next driver).
struct TracedReason {
    RuleIndex rule = kNoRule;
    uint8_t kept = 0;     // how many values are in `values`
    uint32_t read = 0;    // how many the rule read (more than `kept` when some were not kept)
    std::array<ReadValue, kMaxTracedValues> values;
};
// The reason `why` of `decision`, copied.
TracedReason keepReason(const Decision& decision, const Reason& why);
// explain()'s line for a kept reason ("lane ends: lane_ends_ahead is yes; lane_end is 38.2 m"); just the rule's name
// in quotes when no values were kept; empty for no rule.
std::string explainKept(const CompiledRules& rules, const TracedReason& why);

// ONE DECISION IN THE TRACE: the car as it stood once the tick that changed what limits it was done.
struct TraceEntry {
    uint64_t tick = 0;      // Driving::tickCount() after that tick
    double seconds = 0;     // Driving::clockSeconds() after that tick
    Binding binding;        // what limits it from then on
    Real speed = 0;         // m/s
    engine::PathId path;    // where its front was
    Real along = 0;
    TracedReason why;       // Stop or TopSpeed by a rule: that rule's reason, as held
};
bool operator==(const TraceEntry& a, const TraceEntry& b);

// How many notes of junction plans made and dropped a driven car's trace keeps (junction_plan.h PlanNote).
constexpr std::size_t kPlanNoteLength = 8;

// ONE DRIVEN CAR'S TRACE: the ring, and the reasons behind the answers its driver holds now (for the card's "held"
// line, and for the entries); and a second ring of its junction plans made and dropped.
struct CarTrace {
    std::array<TraceEntry, kTraceLength> ring;
    uint32_t written = 0;        // entries ever written on this trip (the ring keeps the last kTraceLength)
    TracedReason heldStop;       // why the place to stop by it holds (rule kNoRule: none held, or not traced)
    TracedReason heldTopSpeed;   // why the top speed it holds
    std::array<PlanNote, kPlanNoteLength> planNotes;
    uint32_t planNotesWritten = 0;   // plan notes ever written on this trip (the ring keeps the last kPlanNoteLength)

    // Adds an entry, dropping the oldest when the ring is full.
    void add(const TraceEntry& e);
    // How many entries the ring holds now, and the i-th of them, oldest first.
    std::size_t size() const;
    const TraceEntry& oldestFirst(std::size_t i) const;
    // The newest entry, or null when none was written.
    const TraceEntry* latest() const;
    // The same for the plan notes.
    void addPlanNote(const PlanNote& n);
    std::size_t planNoteCount() const;
    const PlanNote& planNoteOldestFirst(std::size_t i) const;
};

// ONE NUMBER OVER A CAR'S TRACE (its ring oldest first, the count written, the held reasons), folded in a fixed order.
uint64_t traceHash(const CarTrace& trace);

// THE DRIVER CARD of car `car` (see the top of this file). An unknown car: one plain line.
std::string describeCar(const Driving& driving, CarId car);

}  // namespace driving

#endif
