#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LINE_FRONTAGE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LINE_FRONTAGE_H

// WHERE A POINT FRONTS A ROAD'S CENTRE LINE (ADR-0474), from the search's hit on that line (polyline.h nearestLine: its
// segment, the fraction along it and the nearest point `on`): the station level with the point (metres along the
// road's centre line, read off the line's own stations), and the kerb the point lies toward (`dir` +1: the kerb of the
// traffic running along the centre line; -1: against it; on the line itself: +1), with the unit way across toward it.
// THE ONE STATEMENT of it. RoadModel::frontageAt asks it, and so does the lot pass when it records where a building and
// its doors front (city_lots.cpp LotRoads::frontageOf), so the two cannot answer differently. A header of its own so the
// lot pass can ask it without the whole road model (defined in road_model.cpp, beside towardKerb).

#include "engine/procgen/city/polyline.h"   // NearestLine, Vec2

namespace engine {

struct LineFrontage {
    Real station = 0;
    int dir = 1;
    Vec2 towards = Vec2(1, 0);
};
LineFrontage frontageOnLine(const Vec2* xy, const Real* stations, const NearestLine& hit, const Vec2& p);

}  // namespace engine

#endif
