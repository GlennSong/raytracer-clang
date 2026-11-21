-- A till in use: the drawer springs open for each sale and is pushed shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "drawer", from = 0, to = 0.28, speed = 1.2, wait = 3, wait_back = 7, delay = 2 },
  { node = "screen", from = -6, to = 6, speed = 6, wait = 12 },
}
