-- A bag scanner running: the rollers turn, a tray rides through the tunnel, and the curtain strips it passes swing (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = { { node = "tray", loop = 2.0, speed = 0.25 } }
for i = 1, 20 do parts[#parts + 1] = { node = "roller_" .. i, spin = 250 } end
for i = 1, 14 do parts[#parts + 1] = { node = "strip_" .. i, sway = 14 + (i % 3) * 4, period = 1.3 + (i % 4) * 0.2, phase = i * 0.3 } end
return require("model_moves").script(parts)
