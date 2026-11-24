-- A shared table at dinner: the middle is turned to bring the next dish round (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "turntable", step = 72, speed = 35, wait = 5 },
}
