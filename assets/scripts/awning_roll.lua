-- A shop's awning rolled out for the day and in again (#191; a prefab's script, prefabs.lua `awning_in_use`).
-- The front bar and every strip of cloth are sent out at one speed, much further than they can go; each stops at
-- its own place (the model's limits, awning.lua), so the cloth unrolls strip after strip (model_moves.lua).
local parts = { { node = "bar", from = 0, to = 10, speed = 0.25, wait = 8, wait_back = 5, delay = 2 } }
for i = 1, 10 do parts[#parts + 1] = { node = "strip_" .. i, from = 0, to = 10, speed = 0.25, wait = 8, wait_back = 5, delay = 2 } end
return require("model_moves").script(parts)
