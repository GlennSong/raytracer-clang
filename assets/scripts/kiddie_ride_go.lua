-- A coin-operated ride with a coin in it: it tips back and forth and bobs up and down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "rock", sway = 8, period = 1.7 },
  { node = "bob", from = 0, to = 0.06, speed = 0.11, wait = 0.05 },
}
