-- A garden gate opened to walk through, and swinging shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "gate", from = 0, to = 85, speed = 70, wait = 3, wait_back = 8, delay = 1 },
}
