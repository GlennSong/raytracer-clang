## ADR-XXXX — The sign kit: US road signs as data, faces drawn into the road-sign atlas, models from the mesh words

*(Draft, to be moved into docs/decisions.md and numbered.)*

**Words used here.**
- **Face**: the printed front of a sign (its colours, border, lettering and symbols).
- **Atlas / page**: one large picture (2048 x 2048 pixels) with many sign faces packed onto it side by side; a sign's
  front surface is told which rectangle of the page is its own.
- **Plaque**: a small plate hung under a sign (ALL WAY under STOP).
- **Sheeting**: the reflective film a sign's face is printed on. It sends headlight back toward the driver
  (**retroreflection**), which is why signs glow at night. How strongly depends on its colour.
- **Near turn / far turn**: a turn toward the kerb, or across the oncoming traffic. The engine drives on the left, so
  a near turn is a left turn here.
- **The MUTCD**: the US federal sign manual ("Manual on Uniform Traffic Control Devices", FHWA, 2009 edition), a
  public-domain US government document. It sets each sign's shape, size, colours and mounting height.

**Context.** The driving model's rules will plant signs from the regulations (#240 writes the regulations, #241 plants
them). Glenn chose US-style signs. The engine already had a road-sign system (ADR-0110): the island's freeway guide
signs are laid out as a `SignFace` (panels, text, shields, arrows), rasterized with the sign font into cached atlas
pages, and stood on posts and gantries. But each kind of sign it knows is a branch in C++ (`layoutSign`:
`if (s.kind == "do-not-enter")`), its posts are boxes written inline, and it has no STOP octagon, YIELD triangle,
symbols, plaques, wear or night shine.

**Decision.**
1. **Signs are data.** `assets/scripts/signs_us.lua` lists every sign: its name, the manual's code, what it stands for
   (`means`, in the control plan's words: `control = "stop"`, `regulation = "speed_limit"`), its plate (shape, size in
   inches, colours, border) and a `legend` function that draws its face. `assets/scripts/sign_kit.lua` holds the
   recipes every sign shares: plate outlines (octagon, point-down triangle, diamond, pentagon, rectangle, circle), a
   face builder, the symbols (arrows of every kind, the red circle and slash, a signal head, a walking figure, a
   bicycle, the keep-left island) and the 3-D model. A new sign is a new entry; no C++ names a sign
   (`sign_kit_a_sign_defined_only_in_lua_builds` adds one in a test).
2. **The face is a texture in the road-sign atlas, not geometry and not vertex colour.** Why:
   - The engine's text is pictures already: the sign font draws into images (`Font::draw`), and the road signs and
     the street-name blades are textured from atlas pages. Lettering as geometry would mean turning font outlines
     into triangles, thousands per sign, for nothing a texture does not do better.
   - Vertex colour can only colour whole triangles: a red ring round a black arrow, letters, a walking figure would
     all need their own cut-up geometry.
   - One system per job: the kit lays faces out as the same `SignFace` the road-sign system uses, extended with two
     element kinds (`poly`: any flat shape, even-odd so a ring makes a border band; `line`: a stroke with round ends)
     and condensed lettering (`xScale`, standing in for the manual's narrower letter series), and packs them with the
     same packer (`packSignImages`, moved out of `bakeRoadSignAtlas`; the island's atlas came out byte-identical to
     the one cached before the change). A face with `background = "none"` draws its own plate shape.
   - Unlike ADR-0110's square quads, the kit's plate mesh IS the outline (the octagon, the triangle), so faces need no
     alpha cut; the plate colour is drawn 0.3 in past the edge so texture filtering never shows the page behind.
3. **Night shine is a second page with the same layout.** Each face is drawn again with every colour replaced by how
   strongly its sheeting returns light, as a share of white's (`sheeting` in the catalogue: ASTM D4956 Type III's
   published minimum coefficients; black legends 0). The renderer does not read it yet. The GLB carries it as material
   extras (`rt_retroreflective: {shine_page, ra_white}`); a renderer flag can read the page later. No renderer change.
4. **Wear is one number 0..1 and a seed, deterministic.** In the pixels (`wearSignFace`): colours fade greyer and paler
   in patches, grime streaks downward and gathers low, and the shine page loses what the sheeting has lost with age
   and what the grime covers. In the mesh (Lua): the plate bows, an upper corner may fold, a post leans. In the
   materials: the metal gets rougher and darker. All numbers are in `wear_look`. Same wear and seed, same bytes
   (`sign_kit_same_seed_same_bytes`).
   - **A worn plate bends away from what holds it.** A sign is bolted to its post: the vertical strip along each post
     or hanger (the post, its clamp's strap and a 2 cm margin) never moves, and the bow and the fold grow with the
     distance from it, TOWARD THE READER only. The first version bent the plate backward. Its bowed edges went 3 cm
     behind the plate, the flat faces between them (the filled face has no inner points) crossed the post, and its
     fold line ran through the post line near the top. Bending only forward means no point goes behind the plate's own
     back and, because the bow is convex, no flat face between them does either. So no inner points are needed and a
     worn sign keeps its triangle count. The fold's line is placed past every held strip, so the folded corner never
     has a post behind it. The lean turns post, clamps and plate together about the post's foot.
     `sign_kit_worn_plates_stay_out_of_their_posts` checks every sign at wear 0, 0.5 and 1 over four seeds, and three
     signs on every mount: no point of a plate (on a grid over each triangle, 1 mm inside the plate) is inside a post,
     clamp, arm or span. The old backward bend fails it on 101 signs.
5. **The model comes from the mesh words**, in Lua: the plate is `mesh.fill` front (textured) and back plus a
   `mesh.skirt` rim, 2 mm deep, corners rounded with `mesh.profile.fillet`; posts, clamps, mast arms and spans are
   boxes and columns. Mounts are a request parameter: `post`, `two_post` (chosen by width), `mast_arm` (reaching from
   the kerb out over the road, the side from `kerb_side`), `overhead` (a span). Heights from MUTCD 2A.18: 7 ft to a
   roadside sign's bottom, a plaque 1 ft lower, 17 ft clear under an overhead sign. A FAR version keeps the outline's
   corners chamfered and drops the clamps: about 40 triangles against about 96 (STOP 160, 244 with its plaque).
6. **Sides come from data.** A request names a turn `near` or `far`; `kerb_side = "left"` in the catalogue turns that
   into the LEFT or RIGHT the legend shows, and which code (R3-2 or R3-1). Changing that one line flips every sign
   (`sign_kit_the_side_a_sign_shows_comes_from_the_data`).
7. **A small C++ reader** (`scripting/sign_kit_lua.h`, `SignKit`) runs the catalogue, deduplicates faces (same layout,
   wear and seed: one slot), rasterizes, wears and packs them, and asks the catalogue for each model with its slots.
   `sign_tool gallery` writes `out/signlab/sign_gallery.glb` and a contact sheet.
8. **Materials are data, in the engine's own material terms.** The engine describes a procgen part's material as a
   `ProcMaterial` (proc_model.h: metallic, roughness, and more) with its base colour in the vertex colours. Lua
   recipes set these numbers either through `material.new{...}` (a userdata the model layer reads) or as plain
   `metallic` / `roughness` / `albedo` fields on a part table (vehicles.lua, read by vehicle_spec.cpp). There is no
   shared library of named materials; furniture_showroom.lua and building_gallery.lua each keep their own local table.
   The kit follows the vehicle kit's pattern. `signs_us.lua` keeps a `materials` table:
   - `face`: the sheeting, not a metal, roughness 0.55;
   - `plate_back`: mill-finish aluminium, metallic 1, roughness 0.62, grey 0.80;
   - `post`: galvanized steel, metallic 1, roughness 0.72, grey 0.62.

   Each has a `source` note, with `[~]` where the number is a guess to tune. Each model part names its material, and
   the C++ part (`SignModelPart`) carries it as a `ProcMaterial` plus the base colour. The tool and the planting task
   read the part; neither holds a number (`sign_kit_materials_come_from_the_catalogue_and_metal_ages` changes one in
   Lua and sees it in the result).
   - A dull metal is metallic 1 with a high roughness, not a half-metallic value. The Vulkan shader gives metals their
     light from the sky and the reflection probes (mesh.frag), so a fully metallic post is not black in shade.
   - The metal is now two parts, plate backs and posts, because their materials differ. Splitting adds no triangles.
   - "Slightly blotchy" is not reached: the analytic surfaces have no plain-metal pattern ("metal" is CorrugatedMetal),
     and adding one is a renderer change. The high roughness gives the soft highlight.

**Consequences.**
- 23 signs, 34 examples; a 2048 page holds the whole gallery (44 distinct faces) at 256 texels a metre.
- `glb_export` can embed a texture per entry (PNG, once per page), an alpha mode and material extras.
- **Not yet / follow-ups:**
  - ADR-0110's island signs still lay out through `layoutSign`'s kind branches and their own gantry code; moving
    them onto this catalogue removes the last C++ sign kinds (a #172 item). The street-name blades
    (`street_signs.h`, 12 in blades from `assets/data/streets.json`) are a second blade system beside the kit's D3-1
    (8 in, MUTCD 2D.43): add to the #117 duplicate sweep.
  - The kit does not cache its pages on disk; planting a city should key them as `bakeRoadSignAtlas` does.
  - Planting should round wear to a few steps and reuse seeds, or every sign gets its own atlas slot.
  - A worn plate's bend moves only the outline's points (the fill has no inner points), so a bow is faceted (safe,
    because it only bends forward; see 4).
  - A shared library of named materials (one `steel`, one `aluminium` for every catalogue) does not exist yet. When it
    does, `signs_us.lua`'s `materials` should name its entries instead of holding numbers.
  - Inner layouts (letter heights, border widths, symbol drawings) are read from the Standard Highway Signs drawings'
    proportions by eye, not checked line by line; R7-108, R3-17 and R10-11a are given as remembered and should be
    checked against the manual.
