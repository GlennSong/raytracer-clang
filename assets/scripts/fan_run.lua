-- A fan running: its blades spin, and a desk or stand fan's head sweeps from side to side (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "blades", spin = 540 },
  { node = "head", from = -50, to = 50, speed = 22, wait = 0.4 },
}
