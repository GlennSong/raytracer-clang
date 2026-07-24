-- Gas giant recipe (procedural-planet-plan): a smooth banded sphere from
-- planet.gas_ball — turbulent belts/zones + vortex storms baked to vertex colours,
-- so it rides the same PBR path as the rocky planet with no texture to wire.
-- Parameters come from the level entity's `opts` (the Lua global `args`).
local a = args or {}
return planet.gas_ball{
  preset   = a.preset   or 'jupiter',   -- 'jupiter' | 'neptune'
  seed     = a.seed     or 1,
  radius   = a.radius   or 20,
  face_res = a.face_res or 96,
}
