# Tech Debt / Known Issues

Small-but-real problems parked deliberately while bigger systems land. Add
items with enough context that future-us can pick one up cold; delete items
when fixed (git history is the archive).

## Rendering / performance

- **Framerate dips to ~20fps in the arena viewer.** Workable but trending
  down. Suspects, in rough order: the post stack accumulated passes (SSAO,
  SSR, bloom, shadow maps, and now DOF/lens-warp wiring) running at full
  resolution; no instancing for the repeated primitive meshes; per-frame
  mesh-bounds queries per entity in RenderSystem. Needs a frame capture on
  the Mac (Xcode GPU capture) before optimizing blind. Quick levers to try:
  half-res SSAO/SSR, FPS cap comparison, toggling passes in the Debug panel
  to bisect the cost.
- **Realtime depth of field doesn't visibly work** (Metal `dofGather` pass,
  written blind on Linux, default-off). The lens-warp pass (distortion/CA/
  vignette) reportedly works; DOF needs on-device debugging — check the CoC
  scale (sensor-meters -> pixels), that `dofTexture` actually replaces
  `sceneColorTexture` at composite, and the depth fetch. The offline tracer
  is the reference: same LensParams produce correct thin-lens DOF there.
- **Camera gizmos render in reflections/shadows** (they are plain
  Renderables). Fine until a debug-draw layer exists.
- **Repeated edit/play cycles re-upload level meshes** without freeing the
  previous set — slow GPU-memory leak across mode switches. Needs either a
  mesh cache keyed by shape+size or explicit removeMesh on world clear.

## Offline tracer

- **glTF ("mesh") entities are skipped** by the level importer — offline
  renders omit imported models. Tessellating glTF into tracer triangles via
  ModelImporter is the fix (it already produces vertex/index arrays).
- **No HDR importance sampling** — bright skies converge slowly (the
  extracted-sun NEE covers the worst case).
- **Emissive geometry has no NEE** — the Cornell light is found by chance,
  as it always was; fine at 128+ spp with the bilateral filter.

## Viewer / UX

- **`settings.json` cameraStorePath leaks across runs** if a session ends in
  a state that never rewrites it; harmless (stale path just reloads the same
  sidecar) but worth folding into the editor's document handling.
- **Metal lens passes still unverified beyond the warp** — see DOF above;
  also eyeball CA/vignette strength constants against taste.
