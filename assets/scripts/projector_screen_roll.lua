-- A projector screen rolled down for a lecture and up again (#191; a prefab's script, prefabs.lua).
-- The bar and every strip are sent down at one speed, further than they can go; each stops at its own place
-- (the model's limits, projector_screen.lua), so the screen unrolls (model_moves.lua).
local parts = { { node = "bar", from = 0, to = 10, speed = 0.4, wait = 10, wait_back = 5, delay = 2 } }
for i = 1, 10 do parts[#parts + 1] = { node = "strip_" .. i, from = 0, to = 10, speed = 0.4, wait = 10, wait_back = 5, delay = 2 } end
return require("model_moves").script(parts)
