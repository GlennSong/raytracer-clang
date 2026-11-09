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
-- organisation  0 the cells sprinkled evenly .. 1 bunched into fields and streets with wide clear stretches between
--               (a slow moisture field, swirled round eddies; the weather simulation will write it)
-- size_skew     a cell's radius drawn skewed to the small (1 even): many small clouds, few big ones
-- shadow        how much of the sun a cloud's heart takes on the ground (ADR-0387; the rest comes through and round it)
-- shape_scale   the 3D shape noise's frequency against the level's noiseScale: the noise shapes the inside of a cell
--               now, so its lumps are a cloud's, not a sky's
-- map           extent m across, size texels, rebuild_m: the camera's drift (in the air) that rebakes it
--
-- weathers (ADR-0388): each state the weather walks (engine/weather_cycle.h: ordered mild to severe, the auto walk
-- steps only to a neighbour, easing between them grows or erodes the clouds in place). Every field given:
--   coverage       the share of the sky under cloud (the map's)
--   density        the deck's extinction knob, 0..1 (dark bases toward 1)
--   scale          the noise's domain scale (bigger cells for heavy decks)
--   sun, ambient   multipliers on the sun's light and the sky's fill (an overcast is a diffuser: less sun, more fill)
--   shadow_soften  0 the authored shadows .. 1 shadowless
--   types          { lo, hi } along the type axis the day's clouds spread over
--   wind, gust     m/s at 10 m, and how squally (ADR-0337)

return {
  shape_scale = 3.0,
  shadow = 0.85,
  organisation = 1.0,
  size_skew = 2.2,
  map = { extent = 80000, size = 1024, rebuild_m = 8000 },
  weathers = {
    -- a clear day is a few small puffs at most (Glenn, 2026-09-14: "can we see different weather?"), a light breeze
    clear = { coverage = 0.04, density = 0.25, scale = 0.90, sun = 1.00, ambient = 1.00, shadow_soften = 0.00,
              types = { 0.62, 0.70 }, wind = 3.0, gust = 0.20 },
    -- the everyday sky: a fifth of it under fair-weather cumulus, a little stratocumulus, a moderate breeze
    fair = { coverage = 0.22, density = 0.45, scale = 1.10, sun = 1.00, ambient = 1.05, shadow_soften = 0.08,
             types = { 0.47, 0.75 }, wind = 6.0, gust = 0.35 },
    -- a stratus deck closing over (stratocumulus breaking it), a fresh wind
    overcast = { coverage = 0.90, density = 0.60, scale = 1.90, sun = 0.55, ambient = 1.35, shadow_soften = 0.60,
                 types = { 0.00, 0.40 }, wind = 9.0, gust = 0.40 },
    -- towers among cumulus, a gale gusting ("most of the time the clouds shouldn't be that dramatic unless there's
    -- a storm")
    storm = { coverage = 0.92, density = 1.00, scale = 1.60, sun = 0.28, ambient = 1.45, shadow_soften = 0.85,
              types = { 0.60, 1.00 }, wind = 16.0, gust = 0.70 },
  },
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
