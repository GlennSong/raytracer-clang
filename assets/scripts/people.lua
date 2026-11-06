-- THE PEOPLE (citysim; src/apps/citysim/households.h, plan ~/.claude/plans/kinds-are-data-assign-places.md)
--
-- Who lives in the city: their ages, what their days are for, and the households they live in, one to a dwelling
-- (places.lua cuts the homes into dwellings). Time is frozen: nobody ages, nobody is born. With no file the citysim
-- uses the same catalog built into C++ (defaultPeopleCatalog -- a test keeps the two describing identically).
--
-- The numbers are round, not a census of anywhere; a real one can replace them here.
--
-- people.stages.<name> = what a person's days are for
--   job   = true    they look for work; their role is the job's (roles.lua `job`)
--   role  = "<role>"  (no job) the role they hold (roles.lua)
--   walks = true    they never drive: a child, a student in a hall
--   beds  = true    they live in a hall's beds (places tagged beds), not a dwelling
--
-- people.bands.<name> = { ages = { lo, hi }, stages = { <stage> = share, ... }, height = n | { lo, hi } }
--   an age band, the life stages people of those ages lead, and how tall they are against a grown adult (1): at the
--   band's youngest age and its oldest, in a line between (a child grows; default 1)
--
-- people.households.<name> = { share = n, beds = { lo, hi }, members = { group, ... } }
--   a kind of household: its share, the dwellings it fits (bedrooms lo..hi; 0 a studio) and who it is. A group is
--   `count` people of ONE band, drawn once for the group (a couple is two adults, not an adult and a senior):
--   count = n | { lo, hi };  per_bed = true: one per bedroom, within lo..hi (roommates);  bands = { <band> = share }
--   Every dwelling size up to the largest any type takes must fit some type.
--   ties = { { from, "<tie>", to }, ... }   group `from` (counting from 1) is the <tie> of group `to`'s members;
--   from == to ties every pair in one group (a couple: partners)
--
-- people.ties.<name> = { other = "<tie>", older = years }   how two housemates are tied, as each sees it: the tie
--   the other side holds (a parent's child), and how much older this side is at least (the planner keeps it)
--
-- people.halls = { band, stage, share_of_walkers }   who fills the halls' beds, first: at most this share of the
--   walkers (a student has no car)
--
-- people.lodgers = { <band> = share }   anyone past the housing (a level's own count of cars or walkers), on their
--   own: no band here may lead a walk-only stage

people = {
  stages = {
    order = { "child", "student", "working", "at_home", "retired" },
    child   = { walks = true, role = "child" },
    student = { walks = true, beds = true, role = "student" },
    working = { job = true },
    at_home = { role = "at_home" },
    retired = { role = "retired" },
  },

  bands = {
    order = { "child", "young", "adult", "senior" },
    -- (no babies or toddlers: nobody yet carries one, and a two-year-old does not walk to the park alone)
    child  = { ages = { 5, 17 },  stages = { child = 1 }, height = { 0.62, 0.97 } },
    young  = { ages = { 18, 24 }, stages = { working = 0.6, at_home = 0.4 } },
    adult  = { ages = { 25, 64 }, stages = { working = 0.8, at_home = 0.2 }, height = { 1, 0.99 } },
    senior = { ages = { 65, 90 }, stages = { working = 0.15, retired = 0.85 }, height = { 0.98, 0.94 } },
  },

  ties = {
    order = { "partner", "parent", "child", "sibling", "grandparent", "grandchild", "roommate" },
    partner     = { other = "partner" },
    parent      = { other = "child", older = 18 },
    child       = { other = "parent" },
    sibling     = { other = "sibling" },
    grandparent = { other = "grandchild", older = 36 },
    grandchild  = { other = "grandparent" },
    roommate    = { other = "roommate" },
  },

  households = {
    order = { "single", "couple", "roommates", "single_parent", "family", "three_generations", "seniors" },
    single            = { share = 28, beds = { 0, 2 }, members = { { count = 1, bands = { young = 0.25, adult = 0.75 } } } },
    couple            = { share = 22, beds = { 0, 2 }, members = { { count = 2, bands = { young = 0.1, adult = 0.9 } } },
                          ties = { { 1, "partner", 1 } } },
    roommates         = { share = 8,  beds = { 2, 4 },
                          members = { { count = { 2, 4 }, per_bed = true, bands = { young = 0.6, adult = 0.4 } } },
                          ties = { { 1, "roommate", 1 } } },
    single_parent     = { share = 7,  beds = { 1, 3 },
                          members = { { count = 1, bands = { adult = 1 } }, { count = { 1, 2 }, bands = { child = 1 } } },
                          ties = { { 1, "parent", 2 }, { 2, "sibling", 2 } } },
    family            = { share = 20, beds = { 2, 5 },
                          members = { { count = 2, bands = { adult = 1 } }, { count = { 1, 3 }, bands = { child = 1 } } },
                          ties = { { 1, "partner", 1 }, { 1, "parent", 2 }, { 2, "sibling", 2 } } },
    three_generations = { share = 3,  beds = { 3, 5 },
                          members = { { count = 2, bands = { adult = 1 } }, { count = { 1, 3 }, bands = { child = 1 } },
                                      { count = { 1, 2 }, bands = { senior = 1 } } },
                          ties = { { 1, "partner", 1 }, { 1, "parent", 2 }, { 2, "sibling", 2 },
                                   { 3, "partner", 3 }, { 3, "parent", 1 }, { 3, "grandparent", 2 } } },
    seniors           = { share = 12, beds = { 0, 3 }, members = { { count = { 1, 2 }, bands = { senior = 1 } } },
                          ties = { { 1, "partner", 1 } } },
  },

  halls = { band = "young", stage = "student", share_of_walkers = 0.25 },

  lodgers = { young = 0.2, adult = 0.7, senior = 0.1 },
}
