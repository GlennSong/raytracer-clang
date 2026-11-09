-- THE TRADES (ADR-0360; src/engine/procgen/city/trades.h). A ground-floor shop unit IS a business of one trade, and
-- everything about it reads this table: the facade grammar deals a unit its trade (how common each is in the
-- street's shop mix, how many bays it takes), the fascia wears its colour, the level loader makes the unit a citysim
-- place of its type and hours, the lot pass lays a terrace for the trades that sit out, and its room is finished as
-- `room` says. Read once at start; a new trade is a row here.
--
-- BAKED: the trades decide facades and signs. Until this file joins the lots bake key (with the next kLotsBuildTag
-- bump, ADR-0360), an edit here needs that bump -- or RT_NOCACHE=1 -- to show.
-- ORDER MATTERS: units are dealt their trade in this order -- a row put in the middle re-deals every shop after it,
-- and any new weight re-deals every street. A trade dealt some other way (the konbini: two in three grocers) has no
-- weight, and the streets keep their shops.
--
-- trades[i]
--   id        the shop room's style (0-6 as they always were, 13+ since; 7-12 are the big-box store's rooms, 20-23
--             the malls' concourses -- see `rooms`)
--   name      "cafe", "restaurant", ...      place   the citysim place it is: cafe restaurant bar club shop supermarket
--   hours     { open, close }: when customers come (wraps midnight: a bar opens 16, closes 2; 0, 24 round the clock)
--   weight    how common among the trades, per shop mix: everyday, high_street, nightlife, mall
--   bays      { min, max }: the unit's width in bays      fascia  the sign band's lit colour
--   terrace   sets tables out on the pavement
--   room      how it is finished (a trade without one: plain_room):
--     walls   three colours (a unit's own dice picks one)      dado    { height, color }: panelling or tile, 0: none
--     floor   colour      checker  { color, size }: a checker of floor / color in size-m squares
--     ceiling colour      lights   panels | round (warm rooms) | dim (bars, clubs) | none (lit through an atrium)

return {
  plain_room = {
    walls = { { 0.88, 0.87, 0.84 }, { 0.84, 0.86, 0.88 }, { 0.90, 0.88, 0.82 } },
    floor = { 0.55, 0.55, 0.56 }, ceiling = { 0.92, 0.92, 0.90 }, lights = "panels",
  },

  trades = {
    { id = 0, name = "cafe", place = "cafe", hours = { 6.5, 19.0 },
      weight = { everyday = 14, high_street = 14, nightlife = 4, mall = 10 }, bays = { 2, 2 },
      fascia = { 0.95, 0.70, 0.35 }, terrace = true,
      room = { walls = { { 0.80, 0.62, 0.45 }, { 0.55, 0.62, 0.50 }, { 0.86, 0.80, 0.68 } },   -- white tile dado
               dado = { height = 1.1, color = { 0.92, 0.92, 0.88 } },
               floor = { 0.42, 0.30, 0.20 }, ceiling = { 0.90, 0.88, 0.84 }, lights = "round" } },
    { id = 1, name = "grocery", place = "supermarket", hours = { 7.0, 22.0 },
      weight = { everyday = 14, high_street = 6, nightlife = 1, mall = 2 }, bays = { 2, 3 },
      fascia = { 0.35, 0.85, 0.40 },
      room = { walls = { { 0.92, 0.92, 0.90 }, { 0.90, 0.92, 0.92 }, { 0.92, 0.91, 0.88 } },
               floor = { 0.70, 0.70, 0.68 }, ceiling = { 0.94, 0.94, 0.94 }, lights = "panels" } },
    { id = 2, name = "boutique", place = "shop", hours = { 10.0, 19.0 },
      weight = { everyday = 4, high_street = 10, nightlife = 2, mall = 22 }, bays = { 2, 3 },
      fascia = { 0.95, 0.45, 0.65 },
      room = { walls = { { 0.90, 0.84, 0.82 }, { 0.80, 0.82, 0.84 }, { 0.94, 0.92, 0.88 } },
               floor = { 0.78, 0.74, 0.68 }, ceiling = { 0.95, 0.95, 0.94 }, lights = "round" } },
    { id = 3, name = "bookshop", place = "shop", hours = { 9.0, 20.0 },
      weight = { everyday = 4, high_street = 5, nightlife = 1, mall = 8 }, bays = { 2, 3 },
      fascia = { 0.40, 0.65, 1.00 },
      room = { walls = { { 0.20, 0.32, 0.25 }, { 0.18, 0.22, 0.34 }, { 0.42, 0.18, 0.15 } },   -- a library
               dado = { height = 0.9, color = { 0.30, 0.20, 0.13 } },
               floor = { 0.35, 0.24, 0.15 }, ceiling = { 0.86, 0.84, 0.78 }, lights = "round" } },
    { id = 4, name = "electronics", place = "shop", hours = { 10.0, 20.0 },
      weight = { everyday = 3, high_street = 5, nightlife = 0, mall = 10 }, bays = { 2, 3 },
      fascia = { 0.30, 0.85, 1.00 },
      room = { walls = { { 0.90, 0.91, 0.93 }, { 0.25, 0.27, 0.30 }, { 0.88, 0.88, 0.88 } },
               floor = { 0.45, 0.46, 0.48 }, ceiling = { 0.94, 0.94, 0.95 }, lights = "panels" } },
    { id = 5, name = "pharmacy", place = "shop", hours = { 8.0, 21.0 },
      weight = { everyday = 10, high_street = 5, nightlife = 1, mall = 3 }, bays = { 2, 2 },
      fascia = { 0.35, 1.00, 0.55 },
      room = { walls = { { 0.92, 0.94, 0.93 }, { 0.88, 0.93, 0.92 }, { 0.93, 0.93, 0.93 } },
               floor = { 0.78, 0.84, 0.82 }, ceiling = { 0.95, 0.95, 0.95 }, lights = "panels" } },
    { id = 6, name = "bakery", place = "cafe", hours = { 6.0, 17.0 },
      weight = { everyday = 10, high_street = 6, nightlife = 1, mall = 5 }, bays = { 2, 2 },
      fascia = { 1.00, 0.85, 0.50 }, terrace = true,
      room = { walls = { { 0.95, 0.88, 0.70 }, { 0.92, 0.80, 0.78 }, { 0.80, 0.86, 0.80 } },   -- tile + checker
               dado = { height = 1.2, color = { 0.95, 0.95, 0.92 } },
               floor = { 0.08, 0.08, 0.08 }, checker = { color = { 0.92, 0.92, 0.90 }, size = 0.4 },
               ceiling = { 0.95, 0.94, 0.90 }, lights = "round" } },
    { id = 13, name = "restaurant", place = "restaurant", hours = { 11.0, 23.5 },
      weight = { everyday = 8, high_street = 18, nightlife = 22, mall = 12 }, bays = { 2, 4 },
      fascia = { 1.00, 0.40, 0.30 }, terrace = true,
      room = { walls = { { 0.62, 0.30, 0.20 }, { 0.78, 0.62, 0.42 }, { 0.35, 0.40, 0.30 } },
               dado = { height = 1.0, color = { 0.26, 0.16, 0.10 } },
               floor = { 0.25, 0.16, 0.10 }, ceiling = { 0.55, 0.45, 0.36 }, lights = "round" } },
    { id = 14, name = "bar", place = "bar", hours = { 16.0, 2.0 },
      weight = { everyday = 3, high_street = 10, nightlife = 22, mall = 4 }, bays = { 2, 3 },
      fascia = { 0.75, 0.40, 1.00 },
      room = { walls = { { 0.30, 0.07, 0.08 }, { 0.10, 0.20, 0.14 }, { 0.08, 0.10, 0.18 } },   -- oxblood, green, ink
               dado = { height = 1.3, color = { 0.16, 0.09, 0.05 } },
               floor = { 0.13, 0.08, 0.05 }, ceiling = { 0.07, 0.06, 0.06 }, lights = "dim" } },
    { id = 15, name = "club", place = "club", hours = { 21.0, 3.0 },
      weight = { everyday = 0, high_street = 2, nightlife = 10, mall = 0 }, bays = { 3, 5 },
      fascia = { 1.00, 0.25, 0.85 },
      room = { walls = { { 0.04, 0.04, 0.05 }, { 0.08, 0.03, 0.10 }, { 0.03, 0.04, 0.08 } },
               floor = { 0.03, 0.03, 0.035 }, ceiling = { 0.02, 0.02, 0.025 }, lights = "dim" } },
    -- THE KONBINI (Glenn, 2026-10-13: the convenience store): open round the clock, an everyday errand. No weight:
    -- two in three grocers are one (facadeLayout), so the rest of the street keeps its shops.
    { id = 16, name = "konbini", place = "supermarket", hours = { 0.0, 24.0 },
      weight = { everyday = 0, high_street = 0, nightlife = 0, mall = 0 }, bays = { 2, 3 },
      fascia = { 0.30, 0.75, 1.00 },
      room = { walls = { { 0.95, 0.95, 0.95 }, { 0.94, 0.95, 0.96 }, { 0.95, 0.94, 0.92 } },   -- all white, bright
               floor = { 0.82, 0.82, 0.80 }, ceiling = { 0.97, 0.97, 0.97 }, lights = "panels" } },
  },

  -- ROOMS THAT ARE NOT TRADES: the malls' concourses (20 the box mall's, 21 / 22 the American mall's lower / upper,
  -- 23 its court) -- pale walls, a polished two-tone stone floor in 1.2 m squares
  rooms = {
    { id = 20, walls = { { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 } },
      floor = { 0.80, 0.78, 0.74 }, checker = { color = { 0.70, 0.68, 0.64 }, size = 1.2 },
      ceiling = { 0.94, 0.94, 0.93 }, lights = "panels" },
    { id = 21, walls = { { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 } },
      floor = { 0.80, 0.78, 0.74 }, checker = { color = { 0.70, 0.68, 0.64 }, size = 1.2 },
      ceiling = { 0.94, 0.94, 0.93 }, lights = "none" },
    { id = 23, walls = { { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 } },
      floor = { 0.80, 0.78, 0.74 }, checker = { color = { 0.70, 0.68, 0.64 }, size = 1.2 },
      ceiling = { 0.94, 0.94, 0.93 }, lights = "none" },
    { id = 22, walls = { { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 }, { 0.90, 0.89, 0.86 } },
      floor = { 0.80, 0.78, 0.74 }, checker = { color = { 0.70, 0.68, 0.64 }, size = 1.2 },
      ceiling = { 0.96, 0.96, 0.95 }, lights = "round" },
  },
}
