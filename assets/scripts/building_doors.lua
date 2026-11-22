-- WHICH DOOR HANGS IN A BUILDING'S DOORWAYS (#191, docs/model-record-design.md "Doors in the doorways").
-- A level names this list with `citysim.prefabDoors`; where it does, every doorway of a building the player is
-- near gets a door prefab (prefabs.lua), run at the size of the opening it fills (hinged_door.lua fill_width and
-- fill_height). Without the list a level has the plain box leaves it always had.
--
--   front        the street door            back          the rear service door
--   inside       a doorway between rooms    inside_glass  a doorway in a glass wall (an office front)
--
-- Each is one prefab's name or a list of them; a building picks one from a list by its own seed and uses it for
-- every door of that sort, so a house's inside doors match. `{}` leaves the doorway open.
-- Looked up by the building's recipe, then by its kind (home, shop, office, civic), then `default`; a row states
-- only what differs.
return {
  default = { front = "apartment_entrance", back = "back_door", inside = "door", inside_glass = "door_glass" },
  types = {
    home   = { front = { "front_door_red", "front_door_green", "front_door_wood" } },
    shop   = { front = "shop_door", inside = "door" },
    office = { front = "office_entrance" },
    civic  = { front = "civic_entrance" },
  },
  recipes = {
    school        = { front = "school_entrance" },
    high_school   = { front = "school_entrance" },
    hospital      = { front = "office_entrance" },
    mall_complex  = { front = "office_entrance", inside = {} },   -- (a mall's shop fronts stand open)
    apartment_block = { front = "apartment_entrance" },
    terrace_condo = { front = "apartment_entrance" },
    pencil_tower  = { front = "apartment_entrance" },
  },
}
