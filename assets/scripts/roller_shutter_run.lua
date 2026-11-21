-- A shop's roller shutter raised and lowered (#191; a prefab's script, prefabs.lua `roller_shutter_in_use`).
-- Every section of the curtain is sent up at one speed, further than any can go; each stops when it is inside the
-- box (the model's limits, roller_shutter.lua), so the curtain rises as one piece (model_moves.lua).
local parts = {}
for i = 1, 12 do parts[i] = { node = "section_" .. i, from = 0, to = 10, speed = 0.3, wait = 7, wait_back = 5, delay = 2 } end
return require("model_moves").script(parts)
