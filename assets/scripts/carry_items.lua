-- THE THINGS TO CARRY (ADR-0358; engine/scripting/carry_items.h). `limit`: the heaviest thing a person lifts (kg) --
-- a panel off a car, a bin knocked loose, any of these. `items`: what `carry_spawn <name>` puts down: its box (m),
-- its weight (kg) and its colour.
return {
  limit = 45,
  items = {
    box = { size = { 0.50, 0.36, 0.40 }, mass = 6, color = { 0.62, 0.46, 0.28 } },
    crate = { size = { 0.80, 0.50, 0.60 }, mass = 18, color = { 0.55, 0.42, 0.25 } },
    cooler = { size = { 0.70, 0.42, 0.40 }, mass = 12, color = { 0.15, 0.35, 0.70 } },
    suitcase = { size = { 0.70, 0.48, 0.26 }, mass = 14, color = { 0.10, 0.10, 0.11 } },
    jerrycan = { size = { 0.36, 0.46, 0.18 }, mass = 20, color = { 0.75, 0.12, 0.08 } },
  },
}
