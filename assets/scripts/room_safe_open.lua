-- A room safe opened and shut: the dial is turned, then the door swings (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "dial", step = 90, speed = 180, wait = 6 },
  { node = "door", from = 0, to = 100, speed = 70, wait = 5, wait_back = 8, delay = 2 },
}
