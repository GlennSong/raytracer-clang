-- An espresso machine at work: the lever pulled for each shot, the steam wand swung out and back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lever", from = 0, to = 78, speed = 70, wait = 4, wait_back = 5, delay = 1.5 },
  { node = "wand", from = -30, to = 40, speed = 50, wait = 6, delay = 4 },
}
