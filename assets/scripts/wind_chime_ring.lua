-- A wind chime in a breeze: each tube swings in its own time, and the striker between them (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = { { node = "striker", sway = 9, period = 2.3 } }
for i = 1, 5 do parts[#parts + 1] = { node = "tube_" .. i, sway = 4 + i, period = 1.1 + i * 0.17, phase = i * 0.4 } end
return require("model_moves").script(parts)
