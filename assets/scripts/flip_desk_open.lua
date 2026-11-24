-- A school desk opened to take out a book, and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid", from = 0, to = 70, speed = 100, wait = 4, wait_back = 8, delay = 1 },
}
