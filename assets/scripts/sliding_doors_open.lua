-- Automatic doors with people coming and going: they part, wait and close (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_left", from = 0, to = 3, speed = 0.9, wait = 3, wait_back = 5, delay = 1.5 },
  { node = "door_right", from = 0, to = 3, speed = 0.9, wait = 3, wait_back = 5, delay = 1.5 },
}
