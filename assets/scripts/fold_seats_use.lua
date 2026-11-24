-- A row of folding seats filling up and emptying (#191; a prefab's script, prefabs.lua): each seat is folded down
-- as someone sits, stays down a good while, and springs up when they leave; a lecture seat's tablet follows.
-- The row's length is the model's: seats it does not have are left out (model_moves.lua).
local parts = {}
for i = 1, 12 do
  local delay, stay = 1 + (i * 5) % 9, 14 + (i * 7) % 11
  parts[#parts + 1] = { node = "seat_" .. i, from = 0, to = 80, speed = 160, wait = stay, wait_back = 4 + (i * 3) % 5, delay = delay }
  parts[#parts + 1] = { node = "tablet_" .. i, from = 0, to = 90, speed = 120, wait = stay - 3, wait_back = 7 + (i * 3) % 5, delay = delay + 1.5 }
end
return require("model_moves").script(parts)
