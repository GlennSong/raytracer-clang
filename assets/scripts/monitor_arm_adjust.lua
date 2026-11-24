-- A monitor being moved about on its arm: swung aside, turned and tilted (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "shoulder", from = -25, to = 30, speed = 20, wait = 4 },
  { node = "pan", from = -20, to = 15, speed = 15, wait = 4, delay = 1 },
  { node = "tilt", from = -10, to = 15, speed = 12, wait = 5, delay = 2 },
}
