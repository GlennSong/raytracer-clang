-- A treadmill running: the marks on its belt run back along it, each by one mark's spacing (gym.lua: 0.24 m), and the deck tilts up and down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = { { node = "deck", from = 0, to = 6, speed = 1.5, wait = 10, wait_back = 10, delay = 3 } }
for i = 1, 6 do parts[#parts + 1] = { node = "mark_" .. i, loop = 0.24, speed = 1.6 } end
return require("model_moves").script(parts)
