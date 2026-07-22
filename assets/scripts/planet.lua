-- Procedural planet recipe (procedural-planet-plan / ADR-0076). Returns a displaced,
-- biome-coloured cube-sphere from planet.rocky; rides the existing PBR path via the
-- baked vertex colours, so it needs no custom shader. Parameters come from the level
-- entity's `opts` (the Lua global `args`); the defaults make it runnable bare.
local a = args or {}
return planet.rocky{
  preset   = a.preset   or 'earth',   -- 'mars' | 'moon' | 'earth'
  seed     = a.seed     or 1,
  radius   = a.radius   or 20,
  face_res = a.face_res or 96,
}
