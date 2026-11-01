-- THE PLACES (citysim; src/apps/citysim/place_kinds.h, plan ~/.claude/plans/kinds-are-data-assign-places.md)
--
-- What KIND each place is, and how many HOMES a home holds. With no file the citysim uses the same catalog built into
-- C++ (defaultPlaceKindCatalog -- a test keeps the two describing identically).
--
-- places.kinds.<name> = {
--   type    = "home" | "shop" | "office" | "park" | "civic" | "cafe" | "restaurant" | "supermarket" | "bar" | "club"
--             the coarse category menus and venues read
--   recipes = { names }   the recipes that grow this kind; a recipe no kind lists takes the kind named after its type,
--                         so every type needs a kind of its own name
--   tags    = { names }   what the sim asks instead of naming a kind: campus (the university's grounds and halls),
--                         classes (students go to class here), library (a student's errand), beds (a hall students
--                         live in, not a home for the town), grounds (a campus lawn), sports (games on it)
--   bed_m2  = m2          a hall's beds: one per this much of its floor
-- }
--
-- places.dwellings.<use> = how a home's floor becomes DWELLINGS, one household each (Glenn, 2026-11-01). Keyed by the
-- building's use (building_uses.lua), the same answer its interior is planned from; a use with none takes apartments.
--   per_unit = true       each grown unit is one dwelling (a house; each house of a row), its bedrooms from house_sizes
--   usable = share, mix = { { beds, m2, share } }   a block of flats: the floor less corridors, stairs and walls, cut
--                         into these sizes in these shares (beds 0: a studio)
-- places.house_sizes = { { beds, m2 } }   a house's bedrooms: the largest whose floor area (every storey) it reaches

places = {
  order = { "home", "shop", "office", "park", "civic", "cafe", "restaurant", "supermarket", "bar", "club",
            "teaching", "library", "residence", "quad", "field" },

  kinds = {
    home        = { type = "home" },
    shop        = { type = "shop" },
    office      = { type = "office" },
    park        = { type = "park" },
    civic       = { type = "civic" },
    cafe        = { type = "cafe" },
    restaurant  = { type = "restaurant" },
    supermarket = { type = "supermarket" },
    bar         = { type = "bar" },
    club        = { type = "club" },

    -- THE UNIVERSITY (campus milestone 4)
    teaching  = { type = "civic", recipes = { "teaching_hall" }, tags = { "campus", "classes" } },
    library   = { type = "civic", recipes = { "campus_library" }, tags = { "campus", "library" } },
    -- a dorm room, its share of the corridor, the bathrooms and the lounge: measured against the beds campusPlan furnishes
    residence = { type = "home", recipes = { "residence_hall" }, tags = { "campus", "beds" }, bed_m2 = 45 },
    quad      = { type = "park", recipes = { "campus_quad", "dorm_courtyard" }, tags = { "campus", "grounds" } },
    field     = { type = "park", recipes = { "sports_field" }, tags = { "campus", "grounds", "sports" } },
  },

  dwellings = {
    house      = { per_unit = true },
    apartments = { usable = 0.75,
                   mix = { { beds = 0, m2 = 40, share = 0.15 }, { beds = 1, m2 = 55, share = 0.35 },
                           { beds = 2, m2 = 75, share = 0.35 }, { beds = 3, m2 = 100, share = 0.15 } } },
  },

  house_sizes = { { beds = 1, m2 = 0 }, { beds = 2, m2 = 70 }, { beds = 3, m2 = 110 }, { beds = 4, m2 = 170 },
                  { beds = 5, m2 = 260 } },
}
