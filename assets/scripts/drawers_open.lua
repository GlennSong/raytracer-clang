-- A cabinet in use: its drawers are pulled out and pushed back, one after another (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "drawer_1", from = 0, to = 0.42, speed = 0.5, wait = 3, wait_back = 9, delay = 7 },
  { node = "drawer_2", from = 0, to = 0.42, speed = 0.5, wait = 3, wait_back = 9, delay = 4 },
  { node = "drawer_3", from = 0, to = 0.42, speed = 0.5, wait = 3, wait_back = 9, delay = 1 },
  { node = "drawer_4", from = 0, to = 0.42, speed = 0.5, wait = 3, wait_back = 9, delay = 10 },
  { node = "drawer_5", from = 0, to = 0.42, speed = 0.5, wait = 3, wait_back = 9, delay = 13 },
}
