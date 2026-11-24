-- A parking barrier letting cars through: its arm lifts, waits and comes down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "arm", from = 0, to = 88, speed = 50, wait = 4, wait_back = 6, delay = 2 },
}
