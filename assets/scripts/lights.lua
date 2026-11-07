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
  kinds = {
    -- the street's lamps (StreetFurniture lamp heads): warm sodium-white pools that meet at 26 m spacing
    street_lamp = { type = "point", color = { 1.0, 0.82, 0.55 }, intensity = 48, range = 34,
                    offset = { 0, -0.35, 0 }, dusk = true, pool = "street" },
    -- a station's canopy fittings (RailStop lamps, 12 m apart): cool white, a shorter pool under a roof
    station_lamp = { type = "point", color = { 0.92, 0.96, 1.0 }, intensity = 26, range = 16,
                     dusk = true, pool = "street" },
    -- a car's headlight: a cone pitched down onto the road ahead (the emitter aims it)
    headlight = { type = "spot", color = { 1.0, 0.95, 0.82 }, intensity = 22, range = 45,
                  cone = { 0.32, 0.55 }, pool = "headlights" },
    -- the hand-held light (FlashlightSystem)
    flashlight = { type = "spot", color = { 1.0, 0.93, 0.8 }, intensity = 140, range = 60,
                   cone = { 0.2, 0.42 } },
    -- a storey's ceiling light in the streamed interior (BuildingInteriorSystem gives the room's reach)
    room = { type = "point", color = { 1.0, 0.88, 0.70 }, intensity = 22, range = 20 },
    -- an elevator cab's ceiling light
    cab = { type = "point", color = { 1.0, 0.95, 0.86 }, intensity = 9, range = 6 },
    -- city traffic's lamps, from each car's own markers (CityRenderSystem): a short red glow behind the tail lamps,
    -- brighter while braking
    tail_light = { type = "point", color = { 1.0, 0.08, 0.04 }, intensity = 0.8, range = 4, pool = "tails" },
    brake_light = { type = "point", color = { 1.0, 0.08, 0.04 }, intensity = 3.0, range = 6, pool = "tails" },
    -- a roof's aviation beacon: a tight red pool on the roof and the plant under it (BeaconLightSystem gives the
    -- level's range, the intensity scaled to keep the pool's edge, and the flash)
    beacon = { type = "point", color = { 1.0, 0.06, 0.03 }, intensity = 64, range = 10,
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
  },
}
