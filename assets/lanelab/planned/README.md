# Planned cities

Derived, not authored. Each scene here is `city_plan scene` output for a brief in
`assets/city_plans/`, and is regenerated — not hand-edited — when the brief or the planner
changes (ADR-0092):

    ./build-viewer/city_plan scene assets/city_plans/metro_planned.json \
        assets/lanelab/planned/metro_planned_lanelab.json

`metro_planned_lanelab.json` is variant 4 of `metro_3km`, the one Glenn picked on 2026-09-22:
3 x 3 km, a rotated grid core, midtown warping out to a rim boulevard, ring roads and wedge
blocks in the outskirts, and a freeway ring between two frontage roads with two radial spurs.
The plan's own scorecard predicts ~491 blocks and ~4098 buildings; `lanes_tool build` makes
505 enclosed blocks from it in about 15 minutes.
