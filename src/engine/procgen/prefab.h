#ifndef RAYTRACER_ENGINE_PROCGEN_PREFAB_H
#define RAYTRACER_ENGINE_PROCGEN_PREFAB_H

// A PREFAB (#191, step 7): what a thing in the world is made of, as data. It names the model it shows (a name in
// assets/scripts/models.lua) and says what else a copy of it carries. `assets/scripts/prefabs.lua` is the list:
//
//     return {
//       crane = { model = "crane", physics = { body = "static", friction = 0.7 }, draw = { class = "structure" } },
//     }
//
// STAMPING a prefab (engine/prefab_stamp.h) makes one entity of it at a place: a model instance, the entities that
// draw it (the whole model changing level of detail at once) and its collision bodies, read from the model's shapes.
// A new kind of thing is a new line in the list. No C++ changes.
//
// What a line may say today: `model`, `physics`, `draw`, `script`, `uses`. Sounds and lights at markers, and bodies
// that fall or are thrown, are the later steps of docs/model-record-design.md; a line that names one is refused, not
// ignored.

#include "../../rt_math.h"

#include <string>
#include <utility>
#include <vector>

namespace engine {

// A USE (#191, step 9): one thing a person standing at a copy can do to it, offered through the interaction broker
// (engine/interact_broker.h) at a marker of the model. In prefabs.lua:
//
//     uses = {
//       { at = "fridge_door_handle", say = "open", back = "close", move = { fridge_door = 105 }, speed = 110 },
//       { at = "buttons", say = "switch on", back = "switch off", run = { drum = 300 }, seconds = 40 },
//       { at = "seat", say = "sit", sit = true },
//     }
//
//   at       a marker on the model: where the offer is shown, and how near a person must stand
//   say      the word offered ("open"); back: the word offered once it has been done ("close"). No `back`: the
//            use can only be done again (a bell's button)
//   move     moving nodes and where each goes when the use is done (degrees for a turn, metres for a slide), at
//            `speed` a second (0: at once). Done again (`back`), each returns to 0
//   run      moving nodes that turn or slide on and on while the use is on, each at that many degrees (metres) a
//            second; `seconds`: it turns itself off after that long (0: until used again)
//   stop     with `move`: using it again while the parts are still moving stops them where they are (a desk raised
//            to the height wanted). The word offered meanwhile is "stop"; used after that, it goes the other way
//   coast    with `run`: when it is turned off (or its time is up) the parts do not stop dead but slow to a stop
//            over this many seconds (a roundabout that has been pushed)
//   sway     moving nodes that rock either side of 0 while the use is on, each by that many degrees (metres),
//            once there and back every `period` seconds (a rocking chair while someone sits in it)
//   light    a light of this kind (assets/scripts/lights.lua) is lit while the use is on, at the marker `light_at`
//            (the use's own marker when none is given), aimed the way that marker faces (a lamp switched on; the
//            light inside a fridge whose door is open)
//   sound    what it sounds like, a name in assets/scripts/prefab_sounds.lua ("door", "drawer", "motor", "machine",
//            "click"...): that file says what is heard when the use is done, undone, while its parts move or run,
//            and when they arrive. Heard at the use's marker
//   sit      the person sits at the marker (the furniture's Seated: engine/interaction.h), facing the way the
//            marker faces, and is carried with it if its node moves
//   lie      the same lying down: the eye low over the marker, looking up and along the way it faces
//
//   id       a name for this use, for another to refer to
//   needs    the id of a use that must be ON for this one to be offered (a server slides out only while the rack's
//            door is open). While this one is on, the one it needs cannot be turned off (the door will not shut
//            on a server that is out)
//   unless   the id of a use that must be OFF for this one to be offered (a washing machine is not switched on
//            with its door open, and its door is not opened while it runs: each says `unless` the other)
//
// A use that sits or lies is ON for as long as the person stays, so a seat's own `run`, `sway` or `move` lasts that
// long: sitting on an exercise bike turns its pedals.
//
// MANY LIKE PARTS. A `*` in `at` stands for whatever follows in a marker's name: the use is made once for every
// marker that fits, with the same text put for each `*` in its nodes and ids ({ at = "handle_*", move = {
// ["door_*"] = 110 } } is a use for every locker door, however many the model was built with). With no `*` in
// `at`, a `*` in a node's name takes every moving node that fits (a board's every flap at one touch).
//
// A marker whose role is "seat" or "lie" offers sitting or lying WITHOUT a line saying so, unless a use of the line
// already names that marker: a seat is a seat on every model that has one.
//
// A use whose marker or nodes the model does not have is left out when the copy is stamped, with a warning: one
// line serves a model built with more or fewer of a thing.
struct PrefabUse {
    std::string at, say, back;
    std::string id, needs, unless;
    // stops = "run": doing this use turns that one off (opening a microwave's door stops it).
    std::string stops;
    // near = 2.8: it is done by itself while someone is within that many metres of its marker (level with it), and
    // stays done while they stay: an automatic door, a revolving door. A use with `seconds` lets go that long after
    // they have gone; one without lets go at once.
    Real near = 0;
    std::vector<std::pair<std::string, Real>> move;   // node, value (degrees or metres, as the line says)
    Real speed = 0;
    std::vector<std::pair<std::string, Real>> run;    // node, rate (degrees or metres a second)
    Real seconds = 0;
    bool stop = false;
    Real coast = 0;
    std::vector<std::pair<std::string, Real>> sway;   // node, how far each way (degrees or metres)
    Real period = 2;
    std::string light, lightAt;
    std::string sound;
    bool sit = false;
    bool lie = false;
};

// A LAMP A PREFAB HAS (prefabs.lua `lights = { { at = "lamp", kind = "park_lantern" }, ... }`): a light of a kind in
// lights.lua at one of the model's markers, shining along the marker's +z. It is always there; a kind with
// `dusk = true` comes on at dusk by itself, which is what a lamp post wants. (A light that a person switches is a
// use's `light`.)
struct PrefabLamp {
    std::string at, kind;
};

struct Prefab {
    std::string name;
    std::string model;                 // a name in models.lua
    // ...or, for a model still being written, a recipe run directly (scripting/model_recipes.h obtainRecipeModel):
    // the script's name and its arguments as JSON. `model` is then empty.
    std::string recipe;
    std::string recipeArgs;
    // physics = { body = "static" }: the model's collision shapes become bodies that do not move. No `physics`: a
    // copy is not solid.
    enum class Body { None, Static };
    Body body = Body::None;
    Real friction = -1;                // physics.friction: for every shape; < 0: each shape's own
    // draw = { class = "structure", distance = 400 }: how far it is drawn. `drawClass` is a level's draw class by name
    // (engine/components.h DrawClass: structure, scenery, furniture, ...), whose distance the level sets; `distance`
    // is one in metres that wins over it. Neither: furniture.
    std::string drawClass = "furniture";
    Real drawDistance = 0;
    // script = "crane_swing.lua": a behaviour script (scripting/script_behaviour.h) every copy runs, found as a
    // recipe is (the level's own folder first, then assets/scripts). It moves its copy's nodes with
    // entity.model_set(e, "boom", 40).
    std::string script;
    // uses = { ... }: what a person can do to a copy (PrefabUse, above).
    std::vector<PrefabUse> uses;
    // water = { at = "water", size = { width, length } }: still water to swim in, its surface at that marker, that
    // wide along the marker's x and long along its z (a pool's). Every copy gets a WaterPool (components.h).
    std::string waterAt;
    Real waterW = 0, waterL = 0;
    // lights = { ... }: the lamps every copy has (PrefabLamp, above).
    std::vector<PrefabLamp> lights;
};

}  // namespace engine

#endif
