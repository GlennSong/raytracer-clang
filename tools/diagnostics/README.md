# Road-network diagnostics

Headless white-box probes that grow the living-city road network and measure
weld/conform quality directly from the geometry — no renderer. Used for the
intersection study in `docs/road-intersection-analysis.md`; keep them around to
re-examine the network after any road-mesh or conform change.

## Build

Compile against the city + procgen sources (from the repo root):

```sh
SRC="src/engine/procgen/city/road_net.cpp src/engine/procgen/city/road_network.cpp \
     src/engine/procgen/city/road_mesh.cpp src/engine/procgen/city/road_offset.cpp \
     src/engine/procgen/city/road_rules.cpp src/engine/procgen/city/road_constraints.cpp \
     src/engine/procgen/city/road_crossings.cpp src/engine/procgen/city/street_kit.cpp \
     src/engine/procgen/city/polygon.cpp src/engine/procgen/city/district.cpp \
     src/engine/procgen/terrain.cpp src/engine/procgen/terrain_lod.cpp \
     src/engine/procgen/noise.cpp src/engine/procgen/skeleton.cpp \
     src/engine/procgen/lsystem.cpp src/engine/procgen/tree.cpp src/engine/procgen/sdf.cpp \
     src/curve.cpp src/engine/mesh_builder.cpp"
c++ -std=c++17 -O1 -isystem third_party tools/diagnostics/road_weld_probe.cpp $SRC -o /tmp/road_weld
c++ -std=c++17 -O1 -isystem third_party tools/diagnostics/road_poke_probe.cpp $SRC -o /tmp/road_poke
```

## road_weld_probe — junction weld quality

Per junction it builds the local arm ribbons + junction pad, boolean-unions them
(the curb boundary the mesher forms), then rasterizes a disc and measures
**GAP** = curb-enclosed ground the actual deck mesh fails to cover (the
terrain-through-the-junction failure), plus the min approach angle and how
fragmented `polygonUnion`'s output was (a robustness proxy).

```sh
/tmp/road_weld arena          # synthetic N-arm junctions across angles
SVG=/tmp/weld_map.svg \
  /tmp/road_weld city         # sweep the living-city net; ranks the worst welds
SVG=/tmp/focus.svg \
  /tmp/road_weld focus 103 63 22   # white-box dump of one region: deck tris +
                                   # centrelines + expected-but-bare cells (red)
```

`focus` is the ground-truth view: filled deck triangles, road centrelines, and
red squares wherever roadway-expected ground has no deck triangle over it.

## road_poke_probe — terrain poke-through

Grows the net on the living-city terrain, computes the road conform, then for
every deck vertex compares deck height to the carved terrain — both **exact**
(offline mesh / collider) and with a **flatten dilation** approximating what a
coarse CDLOD tile sees. Buckets poke-through by distance-to-junction and by
cross-slope.

```sh
/tmp/road_poke 0       # exact carved terrain
/tmp/road_poke 2.0     # coarse-LOD approximation (half-cell flatten dilation)
```
