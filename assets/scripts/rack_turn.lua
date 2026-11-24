-- A display that turns: a shopper spinning a rack, or a turntable's motor (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "rack", spin = 18 },
}
