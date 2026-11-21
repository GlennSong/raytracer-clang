-- A record playing: the cover is up, the platter turns and the arm rides in across the record (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "cover", from = 0, to = 65, speed = 60, travel = 1.5, wait = 40, wait_back = 3 },
  { node = "platter", spin = 200 },
  { node = "arm", from = 0, to = 26, speed = 12, travel = 3, wait = 30, wait_back = 8.5, delay = 2 },
}
