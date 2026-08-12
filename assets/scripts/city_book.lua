-- city_book.lua — the Lot System's TUNABLE NUMBERS, as data.
--
-- AGENTS.md's Procgen Authoring rule (ADR-0042) puts the split here: C++ owns
-- the substrate — the solvers, the mesh ops, the SELECTION algorithms — and Lua
-- owns the recipes and "every tunable number". This file is the second half.
--
-- The point is not that these values are better than the compiled-in ones (they
-- are the same values). It is that "the gherkin is too common" should be one
-- line in a data file you can edit and reload, not a change to a switch
-- statement and a rebuild. That was the finding behind docs/lot-system-plan.md
-- §18: every gap in this pipeline turned into a C++ patch because there was
-- nowhere else for it to go.
--
-- Both sections are OPTIONAL and SPARSE. Anything you do not mention keeps its
-- built-in value, so a book stays short, and a level with no book at all behaves
-- exactly as it did before this file existed.

city_book = {

  -- HOW OFTEN A SHAPE EXISTS IN A CITY, relative to a plain bar at 1.0.
  --
  -- A recipe lists the plan templates that suit it; this says how often each of
  -- them should actually turn up. It is a property of the SHAPE rather than of
  -- any one recipe — a trapezoid is rare everywhere — so one table governs the
  -- whole city instead of thirty-seven recipes each guessing.
  --
  -- Real streets are overwhelmingly bars and Ls. Everything below about 0.1 is
  -- an event on a street, and the bottom group is the "one or two in a city"
  -- band: raise those and you get an architecture competition, which is exactly
  -- what this pipeline produced when the choice was uniform.
  plan_rarity = {
    -- the ordinary city
    bar               = 1.00,
    L                 = 0.70,
    U                 = 0.35,
    T                 = 0.30,
    courtyard         = 0.30,
    H                 = 0.22,
    ["bow-front"]     = 0.20,
    ["courtyard-wings"] = 0.18,

    -- corner and site responses: uncommon, and there because a site asked
    ["chamfered-square"]  = 0.16,
    ["curved-corner-slab"] = 0.12,
    flatiron          = 0.10,
    ["atrium-ring"]   = 0.10,
    ["sawtooth-shed"] = 0.10,   -- industrial only
    cruciform         = 0.08,

    -- landmarks. One or two of these in a city is the whole point.
    pinwheel          = 0.05,
    octagon           = 0.04,
    pentagon          = 0.03,
    hexagon           = 0.03,
    ["cross-apse"]    = 0.03,
    ["wedge-tower"]   = 0.02,   -- the trapezoid
    ["lobed-tower"]   = 0.02,   -- the clover plate
    trefoil           = 0.02,
    ["radial-wings"]  = 0.02,
  },

  -- HOW OFTEN A RECIPE IS CHOSEN within the programs that admit it. Taste about
  -- buildings rather than about shapes: a district may want more walk-ups and
  -- fewer glass towers without touching what a tower looks like.
  --
  -- Left empty on purpose. Every recipe already carries a weight in the book,
  -- and those are now read (they were dead for the whole life of this pipeline).
  -- Add a name here only to overrule one, e.g.
  --
  --   recipe_weight = { glass_tower = 0.3, rowhouse = 4.0 },
  --
  -- A name that matches no recipe is reported at load rather than ignored — a
  -- typo that silently does nothing is the worst kind of tuning bug.
  recipe_weight = {},
}
