-- THE CLOUDS (ADR-0386; Glenn, 2026-11-08: "I would like to get us more in line with that Decima system"). As Nubis
-- (Schneider, Horizon Zero Dawn): a WEATHER MAP places every cloud -- a footprint each, of its type, its size and its
-- height -- and the 3D noise only shapes the cloud inside it. Before, the noise placed the clouds: every one a piece
-- of the same 5.7 km tile, every one as tall as the deck. engine/weather/cloud_map.cpp bakes the map round the camera
-- from this; clouds.frag marches it. Read once at start.
--
-- types: the four along the weather's type axis (0, 1/3, 2/3, 1; WeatherCycle typeLo..typeHi spreads a day's clouds
-- over them), mild to severe. Each:
--   base, top    metres above the sea: where the type's clouds stand (a cell's own top shrinks with its size: `low`)
--   profile      the density by height, as fractions of base..top: rising from [1] to full at [2], full to [3], gone
--                by [4]
--   density      its extinction scale (x the level's density)
--   erosion      how much the detail noise eats its edges
--   anvil        1: the top spreads into a shelf (cumulonimbus)
--   radius       { min, max } metres: a cell's footprint, half across
--   aspect       how much longer than wide along its own heading (stratiform sheets stretch)
--   low          the smallest cell's top as a fraction of the type's height (the biggest reaches `top`)
-- shadow        how much of the sun a cloud's heart takes on the ground (ADR-0387; the rest comes through and round it)
-- shape_scale   the 3D shape noise's frequency against the level's noiseScale: the noise shapes the inside of a cell
--               now, so its lumps are a cloud's, not a sky's
-- map           extent m across, size texels, rebuild_m: the camera's drift (in the air) that rebakes it

return {
  shape_scale = 3.0,
  shadow = 0.85,
  map = { extent = 80000, size = 1024, rebuild_m = 8000 },
  types = {
    { name = "stratus", base = 500, top = 1100, profile = { 0.00, 0.20, 0.55, 1.00 }, density = 0.70, erosion = 0.40,
      anvil = 0.0, radius = { 4000, 12000 }, aspect = 2.2, low = 0.7 },
    { name = "stratocumulus", base = 900, top = 1700, profile = { 0.00, 0.15, 0.50, 0.95 }, density = 0.85,
      erosion = 0.70, anvil = 0.0, radius = { 800, 2000 }, aspect = 1.7, low = 0.6 },
    -- fair-weather cumulus: flat bases at ~1.2 km, the small ones shallow (humilis), the big ones a kilometre deep;
    -- full only low down, thinning over most of its height, so it domes (with the footprint narrowing as it rises)
    { name = "cumulus", base = 1200, top = 2700, profile = { 0.00, 0.06, 0.30, 1.00 }, density = 1.00, erosion = 1.00,
      anvil = 0.0, radius = { 250, 1300 }, aspect = 1.25, low = 0.3 },
    { name = "cumulonimbus", base = 900, top = 9000, profile = { 0.00, 0.04, 0.85, 1.00 }, density = 1.60,
      erosion = 0.60, anvil = 1.0, radius = { 2500, 6000 }, aspect = 1.1, low = 0.6 },
  },
}
