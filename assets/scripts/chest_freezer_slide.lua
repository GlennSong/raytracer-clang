-- An ice-cream freezer with customers at it: one lid slid open and shut, then the other (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid_left", from = 0, to = 2, speed = 0.7, wait = 3, wait_back = 9, delay = 1.5 },
  { node = "lid_right", from = 0, to = 2, speed = 0.7, wait = 3, wait_back = 9, delay = 8 },
}
