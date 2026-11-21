-- A shredder being fed: sheets go down into the slot one after another, and now and then the bin is pulled out (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "sheet", loop = 0.3, speed = 0.12 },
  { node = "bin", from = 0, to = 0.24, speed = 0.3, wait = 3, wait_back = 20, delay = 6 },
}
