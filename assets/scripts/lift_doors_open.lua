-- A lift's doors opening for a car that has arrived, and closing (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_left", from = 0, to = 2, speed = 0.5, wait = 4, wait_back = 7, delay = 2 },
  { node = "door_right", from = 0, to = 2, speed = 0.5, wait = 4, wait_back = 7, delay = 2 },
}
