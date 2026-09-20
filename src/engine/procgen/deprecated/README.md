# Deprecated procgen code

Code that is still **compiled and still reachable**, kept because something
shipped depends on it, and marked here so nobody extends it by accident.

Nothing in this folder is a public interface. A file lands here when a
replacement exists but the levels have not moved yet; it leaves when nothing
selects it any more.

## roads/ — the lattice road builder (moved 2026-09-20)

`road_net_mesh.{h,cpp}` (the mesher, the terrain carve, the retaining walls) and
`road_lattice.{h,cpp}` (the swept-lattice sweeper under it) were the second half
of `procgen/city/road_net.cpp`. The first half — the road entity, its `generate`
recipe and the graphs derived from it (`navRoadGraph`, `roadNetConstrainedGraph`,
the editor ops, the JSON) — is **shared road model**, not old code, and lives in
`procgen/city/roads/road_entity.{h,cpp}`. Both builders use it.

Every shipped level (arena, small_town, piedmont, metropolis, hillcity, metro)
still builds its roads with this, so it still compiles and is still selectable:
the roads module reaches it through the `"lattice"` builder
(`procgen/city/roads/road_builder.h`). New road work belongs in the lanes
builder. When no level selects `"lattice"` any more, delete this folder.

`road_net_internal.h` (in city/roads) is the seam the split left behind: the
sampler, the constraints pass and the weld-chain decomposition that both halves
share. It is not an interface either — when the lattice goes, it collapses back
into `road_entity.cpp`.
