-- A fire hose cabinet opened and its reel swung out, the drum turning as hose is pulled off; then put back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door", from = 0, to = 140, speed = 90, travel = 2, wait = 14, wait_back = 6, delay = 1 },
  { node = "reel", from = 0, to = 95, speed = 60, travel = 2, wait = 8, wait_back = 12, delay = 3.5 },
  { node = "drum", spin = 40 },
}
