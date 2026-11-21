-- PREFAB SWAPS: kit pieces that a level stands as prefabs where the floor planner placed them (#191, "One
-- catalog"; docs/model-record-design.md). A level opts in: "citysim": { "prefabSwaps": "prefab_swaps.lua" }.
--
--   <a furniture kit piece's name> = { prefab = "<a name in prefabs.lua>" [, back = metres] [, surface = "<marker>"] }
--
-- The prefab stands where the kit piece stood, its back where the kit piece's back was. `back` is how far out
-- from that its own middle stands; without it, half the kit piece's depth. `surface` names a marker of the prefab:
-- whatever the planner stood on the kit piece (a monitor, a mug, a lamp) is tied to that marker and goes where it
-- goes, so a desk that rises carries what is on it. The things stay the kit's, in their batches.
--
-- This is the first, hand-made step of the swap: a list by name. When prefabs carry tags the room programs will
-- ask for a prefab directly and this file goes. Each copy is a full entity today, which is why only a level that
-- asks gets it (a floor of desks is hundreds of them, until many copies are drawn as a group).
return {
  -- offices
  office_chair   = { prefab = "office_chair_adjustable" },
  desk           = { prefab = "standing_desk", surface = "surface" },
  filing_cabinet = { prefab = "filing_cabinet" },
  whiteboard     = { prefab = "whiteboard" },
  desk_lamp      = { prefab = "desk_lamp" },
  copier         = { prefab = "printer" },
  -- homes
  wardrobe       = { prefab = "wardrobe" },
  kitchen_tall   = { prefab = "fridge" },
  kitchen_wall   = { prefab = "kitchen_wall_cupboard" },
  toilet         = { prefab = "toilet_lidded" },
  -- shops and bars
  bar_stool      = { prefab = "bar_stool_swivel" },
}
