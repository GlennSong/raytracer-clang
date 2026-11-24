-- A shower in use: its glass door is opened and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door", from = 0, to = 90, speed = 70, wait = 3, wait_back = 10, delay = 1 },
}
