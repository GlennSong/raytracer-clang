-- A counter window at work: the tray slides out and back; at the end of the day the blind comes down, each strip to its own place (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = { { node = "tray", from = 0, to = 0.2, speed = 0.3, travel = 1, wait = 2, wait_back = 3, delay = 1 },
                { node = "bar", from = 0, to = 5, speed = 0.25, travel = 4, wait = 6, wait_back = 18, delay = 16 } }
for i = 1, 5 do parts[#parts + 1] = { node = "blind_" .. i, from = 0, to = 5, speed = 0.25, travel = 4, wait = 6, wait_back = 18, delay = 16 } end
return require("model_moves").script(parts)
