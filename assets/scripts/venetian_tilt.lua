-- A venetian blind's slats tilted shut one way, then the other (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = {}
for i = 1, 70 do parts[i] = { node = "slat_" .. i, from = -65, to = 65, speed = 40, travel = 4, wait = 5, wait_back = 5, delay = 1 } end
return require("model_moves").script(parts)
