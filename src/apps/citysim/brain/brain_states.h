#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_STATES_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_STATES_H

// WHAT EACH STATE DOES (ADR-0440): the functions registered under each state's name, for the table to be compiled
// against (brain_table.h). This is the one place in C++ where a state's name is written, once each, at start-up.
//
// BUILT (the first build, design section 9): being at a place (indoors, standing), walking, going on small wheels,
// pulling out, driving, parking, and the states that hold a person (possessed, released, knocked down, stuck, crashed).
// NOT BUILT YET, registered so the table is whole and honest: the transit states (waiting, boarding, riding, alighting,
// hailing), using a spot, a group activity, working a post, asleep. Nobody can enter one; trying is reported.

#include "brain_table.h"

namespace citysim {
namespace brain {

void registerBehaviours(BehaviourSet& set);

}  // namespace brain
}  // namespace citysim

#endif
