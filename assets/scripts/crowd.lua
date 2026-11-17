-- A CROWD OF RESIDENTS MADE FROM THE ROADS (src/apps/citysim/city_crowd.h; ADR-0446). A level names this file
-- (citysim.crowd = "crowd.lua"); its residents, households, cars, days and the places they go between are made from it
-- and the road model when the level loads, seeded by the level's seed. The agent brain then runs them as it runs a
-- hand-written level's residents (city_brain.h).
--
--   residents        how many people
--   intent           what they want at each place: a row of brain.lua's `intents`
--   keep_from_ends   metres a place keeps from either end of its street (out of a junction's mouth)
--   places           the kinds of place, each { kind, count } or { kind, homes = true }: households live at places of
--                    the `homes` kind, one place each (so it has as many as there are households, and no count).
--                    `parking = true`: people drive there, so a place of it is only put where cars can be parked near
--                    it and driven away (a home always is: its household's cars start there). The
--                    places are spread along every street with a sidewalk in proportion to its length, then the kinds
--                    dealt to them in a seeded order
--   household_sizes  { size, share }: how many people live together (shares of their sum)
--   cars             { cars, share }: how many cars a household owns (never more than its members). The 11-11 design
--                    asks for car ownership of 60% or more; two households in three here
--   traits           { name, share }: a share of residents has each (travel.lua's traits: a board, a scooter)
--   days             { share, stops }: a round of stops, each { kind = <a kind of place> } (one of them, picked once per
--                    person; with `beyond`, one at least that many metres from home in a straight line, or the
--                    farthest) or { home = true }, and `minutes = { lo, hi }` there (picked per person). The round comes
--                    back to its last stop, and a person first waits there for its minutes before setting out, so the
--                    last stop's band is when people leave home: the start of the day spread over it
--
-- ESTIMATES, not measured: the shares are a plain small town; the minutes are short so a test sees a whole round in
-- half an hour of the lab's clock (a real working day would be 480 minutes at work).
--
-- TUNED FOR TRAFFIC (sitting 2b, ADR-0446): 350 people, mostly living alone (so a car owner's car is not "in use" by
-- someone else of the household), nearly all going to work at least 900 m away in a straight line (the lab is about a
-- kilometre across: `beyond` past ~1000 m sends everyone to the single farthest place, whose parking then fills) and
-- straight home again. Leaving home over minutes 0-8 is the trade measured: over 0-7 the busiest minute had 82 cars on
-- the road but for only five minutes; over 0-12, about 50 for ten; over 0-8, 59-74 for minutes 6-11.

crowd = {
  residents = 350,
  intent = "be_outside_at",
  keep_from_ends = 15.0,
  places = {
    { kind = "home", homes = true },
    { kind = "work", count = 80, parking = true },
    { kind = "shop", count = 25, parking = true },
    { kind = "lunch", count = 15 },
  },
  household_sizes = {
    { size = 1, share = 0.90 },
    { size = 2, share = 0.10 },
  },
  cars = {
    { cars = 0, share = 0.33 },
    { cars = 1, share = 0.60 },
    { cars = 2, share = 0.07 },
  },
  traits = {
    { name = "has_board", share = 0.08 },
    { name = "has_scooter", share = 0.05 },
  },
  days = {
    -- to work across town and home again: the way out builds over minutes 3-11, the way back from about minute 20
    { share = 0.94, stops = { { kind = "work", beyond = 900, minutes = { 1, 4 } }, { home = true, minutes = { 0, 8 } } } },
    -- out for lunch and home
    { share = 0.03, stops = { { kind = "lunch", minutes = { 3, 6 } }, { home = true, minutes = { 2, 12 } } } },
    -- to a shop and home, a little later
    { share = 0.03, stops = { { kind = "shop", minutes = { 3, 8 } }, { home = true, minutes = { 4, 14 } } } },
  },
}
