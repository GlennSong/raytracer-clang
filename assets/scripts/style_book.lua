-- style_book.lua — the architect's DATA layer (building-grammar-plan P5).
--
-- SEPARATION OF DUTY: the C++ architect decides WHAT stands WHERE — district
-- tables, landmark quotas, massing, floor counts — deterministic and pinned by
-- tests. This book owns HOW EACH RECIPE LOOKS: any entry here overlays its
-- fields onto the named recipe's BuildingParams just before the building
-- grows, using the same keys the building lab uses. Fields you don't mention
-- keep the recipe's built-in look, so entries stay tiny.
--
-- Available keys (see readBuildingParams): style ("brick"|"stucco"|"concrete"|
-- "metal"|"glass"|"painted"), wall_color = {r,g,b}, trim_color, window_head
-- ("flat"|"segmental"|"round"), window_hood ("none"|"band"|"arch"), lights_x,
-- lights_y, quoins, sill, roof ("flat"|"gable"|"hip"), roof_pitch, parapet,
-- awning, pilasters, portico (column count), entrance_steps, dome,
-- ground_bays, floor_height, ground_height, bay_width ...
--
-- Recipe names: glass_tower, office_slab, commercial_block, civic_hall,
-- civic_midtown, brick_shop, mixed_use, office_midrise, hotel, walkup_homes,
-- oldtown_house, metal_shed, industrial_office, yard_house, duplex,
-- rowhouses (the strip's fallback), rowhouse_unit (each townhome),
-- apartments, corner_shop, school, hospital, courthouse, capitol, police,
-- fire_station, market_hall, university.

style_book = {
  -- The capitol in marble-white — the one active override, proving the pipe.
  capitol = { wall_color = { 0.90, 0.885, 0.85 } },

  -- Examples (uncomment + save; the city restyles on the next level load):
  -- market_hall   = { style = "stucco", roof_pitch = 0.45 },
  -- rowhouse_unit = { roof = "gable" },
  -- hotel         = { style = "painted", trim_color = { 0.30, 0.28, 0.26 } },
  -- oldtown_house = { wall_color = { 0.85, 0.72, 0.55 } },
}
