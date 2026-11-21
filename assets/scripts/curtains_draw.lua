-- Curtains drawn back for the day and closed again. Every fold is sent toward its side at one speed, further than it
-- can go; each stops where it is gathered (the model's limits, window_coverings.lua), so the curtain bunches up fold by fold (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = {}
for _, side in ipairs{ "left_", "right_" } do
  for i = 1, 6 do parts[#parts + 1] = { node = side .. i, from = 0, to = 5, speed = 0.25, travel = 8, wait = 8, wait_back = 6, delay = 2 } end
end
return require("model_moves").script(parts)
