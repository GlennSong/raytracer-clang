-- A split-flap board changing: each drum turns on by quarter turns, row after row, the columns a little apart, so the change ripples across (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
local parts = {}
for row = 1, 10 do
  for col = 1, 14 do
    parts[#parts + 1] = { node = string.format("flap_%d_%d", row, col), step = 90 * (1 + (row + col) % 3), speed = 720, wait = 6 + row * 1.5 + col * 0.06 }
  end
end
return require("model_moves").script(parts)
