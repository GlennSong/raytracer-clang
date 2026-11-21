-- A cupboard in use: each door is opened, left a moment and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 100, speed = 90, wait = 4, wait_back = 7, delay = 1 },
  { node = "door_2", from = 0, to = 100, speed = 90, wait = 3, wait_back = 8, delay = 2 },
}
