-- FLOOR PROGRAMS (src/engine/procgen/city/floor_zones.h). A storey's plate is cut like a city block into lots
-- (window lots along the glass, corner lots where it turns, interior lots in the court round the core); a program
-- then tags each lot, the way the archetype book picks a lot's building.
--
-- lots       how the plate is cut (metres; m2 for interior_lot)
-- fill       the tag every lot no zone takes gets ...
-- shallow    ... or this one, where the lot's frame is shallower than shallow_depth
-- zones      tried IN ORDER; a lot one takes is not offered to the next. Each:
--   tag        its name (styles refer to it)
--   kind       the room it builds (a RoomKind: open_plan, meeting, office, kitchenette, reception, lounge, desk_row, ...;
--              defaults to the tag); the kind picks the furniture program (furniture_rooms.lua)
--   sites      which lots may take it: "window", "corner", "interior" (all three when absent)
--   count | per_m2 | share   how many: exactly count a floor; one per per_m2 of floor; that share of the lots it may take
--   area       {min, max} m2; min_depth (m) of the furniture's frame
--   enclosed   partitions round it and a door onto circulation (else an open zone, no walls)
--   walls      "glass", "drywall", or "facade" (glass behind a curtain wall, drywall in masonry; the default)
--   near       "lobby" (closest to the lifts first), "core", "far" (farthest from the lifts first); else spread out
--   lifts      only on a floor with lifts (a stair-only office has no reception)
-- styles     a building's habit, picked per building by weight: `set` overrides zones' numbers by tag.
--
-- Keep the office program alike with builtinOfficeProgram() (floor_zones.cpp): it stands where this file is not
-- loaded (procgen without the scripting layer, most tests).

floor_programs = {
  office = {
    lots = { window_depth = 4.5, aisle = 1.6, street = 1.8, lot_width = 4.8, interior_lot = 24, min_side = 2.6 },
    fill = "open_plan",
    shallow = "desk_row", shallow_depth = 3.2,
    zones = {
      { tag = "reception", kind = "reception", sites = {"interior", "window"}, count = 1, near = "lobby", lifts = true,
        area = {12, 60} },
      -- the corner office is the whole corner (a rounded corner's arc too)
      { tag = "corner_office", kind = "office", sites = {"corner"}, share = 1.0, enclosed = true, area = {8, 1e9} },
      { tag = "pantry", kind = "kitchenette", sites = {"interior"}, count = 1, near = "core", area = {10, 50} },
      { tag = "meeting", kind = "meeting", sites = {"interior", "window"}, per_m2 = 320, enclosed = true, walls = "glass",
        area = {10, 45}, min_depth = 3.0 },
      { tag = "office", kind = "office", sites = {"window"}, share = 0.0, enclosed = true, near = "far",
        area = {8, 30}, min_depth = 3.0 },
      { tag = "lounge", kind = "lounge", sites = {"window", "interior"}, count = 1, area = {12, 1e9} },
      { tag = "open_plan", kind = "open_plan" },
      { tag = "desk_row", kind = "desk_row" },
    },
    styles = {
      { name = "corner_offices", weight = 2 },
      { name = "perimeter_offices", weight = 1, set = { office = { share = 0.5 } } },
      { name = "open_floor", weight = 1, set = { meeting = { per_m2 = 500 } } },
    },
  },
}
