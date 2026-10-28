-- BUILDING USES (src/engine/procgen/city/building_uses.h): what each building's upper floors are FOR, recipe by recipe.
-- The use names the floor's layout: a floor program in floor_programs.lua (office, apartments) or a planner of its
-- own kind (house, hotel, hospital, civic). Read while the game runs: change a line, and the interiors round the
-- player are planned again.
--
-- What shapes the OUTSIDE stays with the recipe (C++, baked in the lot cache): whether a building is homes (its lit
-- windows, flats over shops) and its civic kind (a hotel's entrance, a fire station's open bays). A recipe whose use
-- here is apartments or house must be one the recipe makes residential; a test holds the two together.
--
-- defaults   the use of a recipe not listed, by its place type
-- recipes    recipe name = use

building_uses = {
  defaults = { home = "apartments", office = "office", shop = "office", civic = "office", park = "office", depot = "none" },

  recipes = {
    -- homes: blocks of flats and houses
    apartments      = "apartments",
    bungalow        = "house",
    condo_tower     = "apartments",
    cottage         = "house",
    courtyard_block = "apartments",
    craftsman_house = "house",
    duplex          = "house",
    garden_condo    = "apartments",
    loft_block      = "apartments",
    modern_house    = "house",
    oldtown_house   = "apartments",
    pencil_tower    = "apartments",
    ranch_house     = "house",
    residence_hall  = "apartments",
    rowhouses       = "house",
    terrace_condo   = "apartments",
    villa           = "house",
    walkup_homes    = "apartments",
    yard_house      = "house",

    -- offices
    art_deco_tower      = "office",
    bank                = "office",
    brick_warehouse     = "office",
    bundled_tubes       = "office",
    drum_tower          = "office",
    factory             = "office",
    glass_slab          = "office",
    glass_tower         = "office",
    industrial_office   = "office",
    leaning_tower       = "office",
    lens_tower          = "office",
    metal_shed          = "office",
    office_midrise      = "office",
    office_park         = "office",
    office_slab         = "office",
    parking_garage      = "office",
    podium_tower        = "office",
    ring_block          = "office",
    rounded_tower       = "office",
    sculpted_tower      = "office",
    sky_exposure_tower  = "office",
    stacked_modules     = "office",
    stepped_tower       = "office",
    tapered_glass_tower = "office",
    tower_on_base       = "office",
    twin_towers         = "office",
    twisted_taper       = "office",
    twisted_tower       = "office",
    veil_tower          = "office",
    wavy_tower          = "office",

    -- shops and commercial blocks: the floors over the shops
    big_box          = "office",
    brick_shop       = "apartments",
    cinema           = "office",
    commercial_block = "office",
    corner_shop      = "apartments",
    hotel            = "hotel",
    mall_complex     = "office",
    market_hall      = "office",
    mixed_use        = "apartments",
    oldtown_cafe     = "office",
    oldtown_grand    = "apartments",
    pagoda_tower     = "office",
    strip_mall       = "office",
    supermarket      = "office",

    -- civic buildings: their own rooms on the ground, offices or their own planner above
    campus_library = "civic",
    capitol        = "office",
    church         = "civic",
    civic_hall     = "office",
    civic_midtown  = "office",
    courthouse     = "office",
    fire_station   = "office",
    hospital       = "hospital",
    library        = "civic",
    museum         = "civic",
    plaza          = "office",
    police         = "office",
    school         = "civic",
    teaching_hall  = "civic",
    university     = "civic",

    -- parks
    pocket_park = "office",
  },
}
