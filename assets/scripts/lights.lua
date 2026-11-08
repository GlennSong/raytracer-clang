-- THE LIGHTS (ADR-0362; src/engine/lights/light_kinds.h). Every real light in the world is one of these kinds: a
-- system that gives off light (a street lamp, a car's headlights, the flashlight, a room's ceiling) publishes where
-- its lights are and of what kind, and ONE light system picks which of them are lit this frame -- by pool, nearest
-- first, the player's own first, fading at the edge of the set so a light never pops. A new kind of light is a row
-- here. Read once at start.
--
-- kinds.<name>
--   type       "point" | "spot"
--   color      { r, g, b } linear      intensity  illuminance at 1 m (ADR-0017 units)
--   range      metres: the falloff window ends here (an emitter may give its own: a room's size, the level's beacons)
--   cone       { inner, outer } radians (spots)
--   offset     { x, y, z } metres from the emitter's point (a lamp's bulb hangs under its head)
--   dusk       true: rides the dusk ramp (dark by day, warming up through twilight)
--   pool       which budget it is picked from (below); none: every one published is lit
--   candela    its real luminous intensity (cd), what the physical sky's world lights it at (ADR-0369; RT_SKY=physical);
--              `intensity` is the old night exposure's value, until the physical sky is the default
--
-- pools.<name>
--   count      at most this many real lights (groups, when `groups`)
--   within     metres from the camera; 0: no limit
--   fade       true: the set fades out toward its edge (the reach, or the farthest picked when `count` binds)
--   groups     true: the emitters' groups are picked whole (a car's two headlights go together)
--   many       { count, within, fade }: the budget where the renderer lights every pixel from its own short list
--              (clustered, ADR-0361: thousands of lights) instead of looping a fixed 32. Unset fields keep the above.
-- On a 32-light renderer the picked set is trimmed to fit by importance (the player's own first, then the brightest
-- where they are), so no renderer drops lights in an order of its own.
-- A level's light block sets the street and beacon pools (CitySimConfig: lampLightCount / lampLightRadius /
-- lampLightRange, lightCount / lightRadius / lightRange).

return {
  -- THE GLOWS' WAY INTO REAL UNITS (ADR-0369): the emission of every glowing surface (lit windows, lamp heads, signs,
  -- lit lenses) was tuned against the old x6 night exposure, about 250x its real luminance; under the physical sky it
  -- is scaled by this until each class declares its own (a lit window, old 1.3, comes to ~100 cd/m^2)
  emission_scale = 0.004,
  -- the city's glow on the night sky (light pollution), low toward downtown, and the full moon's disc above the air
  city_glow_nits = 2.0,
  moon_nits = 2500,

  -- THE REFLECTION PROBES (ADR-0374): one every `spacing` m of street, `height` m over the carriageway, its box (the
  -- reflection's parallax) `side` m past the kerbs either side to the frontages, from `below` under the street to
  -- `above` over it; sites nearer than `merge` are one (junctions); the 64 nearest are held, and one leaves only once it
  -- is `hysteresis` m past the nearest set's edge
  probes = { spacing = 40, height = 5, side = 30, below = 1, above = 45, merge = 20, hysteresis = 15 },

  -- RAY-TRACED REFLECTIONS (ADR-0376; Glenn: "close up raytraced, further away faked"): rays only from surfaces
  -- smoother than `max_roughness`, within `near` m, faded into the probes over the last `fade` m; they hit only these
  -- draw classes (structure, scenery, furniture, groundPaint, simBody: a level's drawDistances names) within `near`; the structures may take at most
  -- `memory_share` of the GPU memory free when they start (and never more than 1.5 GB)
  raytrace = { near = 150, fade = 25, max_roughness = 0.3, memory_share = 0.25, hits = { "structure", "furniture" } },

  -- THE GLOWS (ADR-0369): each class of glowing surface at its real luminance (cd/m^2) under the physical sky, in its
  -- own colour; a class not here keeps its old emission scaled by emission_scale
  glows = {
    window_pane = { nits = 25 },          -- a lit room seen through its window (the room atlas: the fixture at full)
    lit_band = { nits = 300 },            -- a tower's crown band, signage strip, podium uplight
    far_city = { nits = 12 },             -- a far block's windows, averaged over the lit and the dark
    street_lamp_head = { nits = 8000 },   -- a street lamp's lit head
    station_lamp_head = { nits = 5000 },  -- a station canopy's fitting
    sign = { nits = 400 },                -- a shop sign's lettering
    beacon_bulb = { nits = 3000 },        -- an aviation beacon's lamp, its glow and its haze
    beacon_glow = { nits = 2000 },
    beacon_haze = { nits = 800 },
  },

  kinds = {
    -- the street's lamps (StreetFurniture lamp heads): warm sodium-white pools that meet at 26 m spacing
    street_lamp = { candela = 1600, type = "point", color = { 1.0, 0.82, 0.55 }, intensity = 48, range = 34,
                    offset = { 0, -0.35, 0 }, dusk = true, pool = "street" },
    -- a station's canopy fittings (RailStop lamps, 12 m apart): cool white, a shorter pool under a roof
    station_lamp = { candela = 600, type = "point", color = { 0.92, 0.96, 1.0 }, intensity = 26, range = 16,
                     dusk = true, pool = "street" },
    -- a car's headlight: a cone pitched down onto the road ahead (the emitter aims it)
    headlight = { candela = 15000, type = "spot", color = { 1.0, 0.95, 0.82 }, intensity = 22, range = 45,
                  cone = { 0.32, 0.55 }, pool = "headlights" },
    -- the hand-held light (FlashlightSystem)
    flashlight = { candela = 2000, type = "spot", color = { 1.0, 0.93, 0.8 }, intensity = 140, range = 60,
                   cone = { 0.2, 0.42 } },
    -- THE ROOMS (ADR-0365): each ceiling fitting the furniture hangs is a light kept to its own room's box (the walls
    -- stop it), of its fitting's kind -- an office's recessed panel, a home's round flush fitting, a bar's pendant
    room_panel = { candela = 1300, type = "point", color = { 0.95, 0.96, 1.0 }, intensity = 4.5, range = 6, pool = "rooms" },
    room_round = { candela = 700, type = "point", color = { 1.0, 0.86, 0.66 }, intensity = 6, range = 6, pool = "rooms" },
    room_pendant = { candela = 250, type = "point", color = { 1.0, 0.72, 0.45 }, intensity = 2.5, range = 4, pool = "rooms" },
    -- ...and a dim fill a storey (the nearest building, 4 storeys round the player; BuildingInteriorSystem gives the
    -- building's reach): the light the rooms bounce about, and the corridors', which hang no fittings
    room_fill = { candela = 250, type = "point", color = { 1.0, 0.88, 0.70 }, intensity = 5, range = 20 },
    -- an elevator cab's ceiling light
    cab = { candela = 300, type = "point", color = { 1.0, 0.95, 0.86 }, intensity = 9, range = 6 },
    -- city traffic's lamps, from each car's own markers (CityRenderSystem): a short red glow behind the tail lamps,
    -- brighter while braking
    tail_light = { candela = 15, type = "point", color = { 1.0, 0.08, 0.04 }, intensity = 0.8, range = 4, pool = "tails" },
    brake_light = { candela = 80, type = "point", color = { 1.0, 0.08, 0.04 }, intensity = 3.0, range = 6, pool = "tails" },
    -- a bus saloon's ceiling lights (ADR-0378; Glenn: "the bus interior is dark at night"): three down the saloon, a
    -- cool white strip, lit with the street lamps
    bus_cabin = { candela = 350, type = "point", color = { 0.96, 0.98, 1.0 }, intensity = 5, range = 7, dusk = true,
                  pool = "cabins" },
    -- a roof's aviation beacon: a tight red pool on the roof and the plant under it (BeaconLightSystem gives the
    -- level's range, the intensity scaled to keep the pool's edge, and the flash)
    beacon = { candela = 100, type = "point", color = { 1.0, 0.06, 0.03 }, intensity = 64, range = 10,
               dusk = true, pool = "beacons" },
  },

  pools = {
    -- every street lamp in sight lights its pool; a 32-light renderer keeps the nearest 20
    street = { count = 20, within = 160, fade = true, many = { count = 2500, within = 800 } },
    -- the player's car always, then the nearest lit cars, two beams each
    headlights = { count = 3, within = 0, fade = false, groups = true,
                   many = { count = 300, within = 400, fade = true } },
    tails = { count = 0, within = 0, fade = true, groups = true, many = { count = 300, within = 200 } },
    beacons = { count = 6, within = 60, fade = true, many = { count = 64, within = 250 } },
    -- the fittings of the streamed interiors round the player
    rooms = { count = 8, within = 15, fade = true, many = { count = 600, within = 45 } },
    -- the buses' saloons: the nearest bus's three (the one you ride), then every lit saloon near
    cabins = { count = 3, within = 25, fade = true, groups = true, many = { count = 60, within = 150 } },
  },
}
