-- A stool with someone on it: it turns a little one way and the other, and a lab stool is raised or lowered now and then (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "swivel", from = -35, to = 30, speed = 40, wait = 3, wait_back = 4 },
  { node = "lift", from = 0, to = 0.12, speed = 0.08, wait = 10, wait_back = 10, delay = 5 },
}
