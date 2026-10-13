#ifndef RAYTRACER_APPS_CITYSIM_RAIL_BOARD_H
#define RAYTRACER_APPS_CITYSIM_RAIL_BOARD_H

// THE PLATFORM BOARD (Glenn, 2026-10-12: "have a similar thing to the buses when you're on a station platform"). What
// a departure board says to someone standing on a platform: the station, and for each line calling there its next
// trains -- which way they run (the next station), how long until they stand at it, whether one is standing there
// now. The bus HUD counts stops, because a bus's time is a guess about traffic; a train runs on its own track, so this
// gives minutes: the distance along the line at an average running pace, plus a dwell at every stop between.
//
// Pure: RailSim in, a board out (the transit HUD draws it; a test reads it).

#include "rail_sim.h"

#include <string>
#include <vector>

namespace citysim {

struct PlatformBoard {
    std::string station;
    int line = -1, stop = -1;          // the platform you stand on: its line and that line's stop
    struct Departure {
        int line = -1, train = -1;
        std::string toward;            // the line's next station after this one
        double eta = 0;                // seconds until it stands at the platform (0 while it does)
        bool here = false;             // standing at the platform now
        bool thisPlatform = false;     // the line of the platform you are on (the others: across the tracks)
    };
    std::vector<Departure> departures; // this platform's line first, then the others; each line's next two, soonest first
};

// The platform under `body` (the player's capsule centre, world): beside a stop's span, in the band from the
// platform's edge to its back, at about its height. False off every platform.
bool platformAt(const RailSim& rs, const engine::Vec3& body, int& line, int& stop);

// The board for stop `stop` of line `line`: every line calling at a station of that name.
PlatformBoard platformBoard(const RailSim& rs, int line, int stop, int perLine = 2);

// Seconds until train `ti` stands at stop `k` of its line (0 if it does now; < 0 if it never will -- an open line
// it has passed the end of).
double trainEta(const RailSim& rs, int ti, int k);

}  // namespace citysim

#endif
