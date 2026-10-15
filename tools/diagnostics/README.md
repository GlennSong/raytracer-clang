# Road-network diagnostics

Headless probes that read a level's road network straight from its `generate` recipe (the grown street graph,
before any builder meshes it) and measure it -- no renderer. Since the shipped cities moved to the lanes builder
(2026-10-14) their recipes live in `assets/lanelab/sources/`; point the probes there.

Built by CMake (`cmake --build build-viewer --target <name>`):

- **junction_angle_probe** -- the planning-phase view of acute junctions: junctions by degree, the tightest-gap
  distribution, how many break a range of candidate thresholds.
- **road_map_svg** -- the road network, blocks and lots of a recipe as a layered SVG.
- **road_products_probe** -- what the road builder hands the engine for a level (the carve, the surface, the deck,
  the kerb band, the nav graph), by the numbers.

The lattice probes (road_weld_probe, road_poke_probe, curb_weld_probe) measured the lattice mesher's welds, terrain
poke-through and curb band; they retired with it. The lanes builder's own checks are `lanes_tool build`'s invariants,
`tools/lanes_corpus.py` and `rt_bake --audit`.
