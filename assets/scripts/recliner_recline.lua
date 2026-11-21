-- An armchair leant back with its footrest up, then sat up again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "back", from = 0, to = 35, speed = 14, wait = 8, wait_back = 5, delay = 2 },
  { node = "footrest", from = 0, to = 90, speed = 45, wait = 8.5, wait_back = 5, delay = 2 },
}
