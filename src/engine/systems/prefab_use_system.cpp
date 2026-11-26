#include "prefab_use_system.h"

#include "physics_system.h"
#include "../prefab_batch.h"

#include "../components.h"
#include "../interact_broker.h"
#include "../interaction.h"   // Seated
#include "../lights/light_board.h"
#include "../lights/light_kinds.h"
#include "../model_motion.h"
#include "../audio/sfx.h"
#include "../../log.h"
#include "../prefab_use.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/model_recipes.h"   // readScriptList
#endif

#include <sstream>
#include <cctype>
#include <nlohmann/json.hpp>

#include <cmath>
#include <vector>

namespace engine {

namespace {

// Where someone sitting at a seat marker is: the eye above it, looking the way the marker faces, and the floor in
// front of it to stand up on.
bool seatAt(World& world, Entity instance, const std::string& marker, bool lying, Seated& out) {
    Vec3 at;
    Quat turn;
    if (!modelMarkerWorld(world, instance, marker, at, turn)) return false;
    Vec3 facing = turn.rotate(Vec3(0, 0, 1));
    facing.y = 0;
    facing = facing.lengthSquared() > 1e-8 ? normalize(facing) : Vec3(0, 0, 1);
    const Transform* place = world.get<Transform>(instance);
    if (lying) {   // on the back: the eye a head's height over the marker, looking up and along
        out.eye = at + Vec3(0, 0.25, 0);
        out.look = normalize(facing + Vec3(0, 1.6, 0));
        out.exit = Vec3(at.x, place != nullptr ? place->position.y : at.y, at.z) + turn.rotate(Vec3(1, 0, 0)) * Real(1.2);
        return true;
    }
    out.eye = at + Vec3(0, 0.72, 0) - facing * Real(0.06);
    out.look = normalize(facing + Vec3(0, -0.08, 0));
    out.exit = Vec3(at.x, place != nullptr ? place->position.y : at.y, at.z) + facing * Real(0.8);
    return true;
}

}  // namespace

void PrefabUseSystem::fixedUpdate(FrameContext& ctx) {
    // the player, as someone who may be walking a roundabout round: where their feet are and how fast they go
    Vec3 feet, velocity;
    bool walker = false;
    const Real dt = ctx.clock.fixedStep();
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (walker || ctx.world.has<Seated>(e) || ctx.world.has<InVehicle>(e)) return;
        Real drop = 0.7;
        if (const CharacterController* cc = ctx.world.get<CharacterController>(e)) drop = cc->halfHeight + cc->radius;
        feet = t.position - Vec3(0, drop, 0);
        if (const PrevTransform* prev = ctx.world.get<PrevTransform>(e)) velocity = (t.position - prev->value.position) * (Real(1) / std::max(dt, Real(1e-6)));
        walker = true;
    });
    stepFreeTurns(ctx.world, dt, walker ? &feet : nullptr, walker ? &velocity : nullptr);
    stepPrefabUses(ctx.world, dt);
}

// THE SOUNDS. prefab_sounds.lua names, for each kind of sound a use may have, a clip for each thing that happens.
void PrefabUseSystem::onStart(FrameContext& ctx) {
    (void)ctx;
    sounds_.clear();
    soundsRead_ = false;
    loops_.clear();
#ifdef RT_ENABLE_SCRIPTING
    nlohmann::json list;
    std::string why;
    if (!readScriptList("prefab_sounds.lua", std::string(), list, &why)) {
        LOG_WARN << "[prefab uses] prefab_sounds.lua: " << why << " (prefabs are silent)";
        return;
    }
    for (nlohmann::json::const_iterator it = list.begin(); it != list.end(); ++it) {
        if (!it.value().is_object()) continue;
        const nlohmann::json& j = it.value();
        Sound s;
        s.done = j.value("done", std::string());
        s.undone = j.value("undone", std::string());
        s.arrived = j.value("arrived", std::string());
        s.shut = j.value("shut", std::string());
        s.moving = j.value("moving", std::string());
        s.running = j.value("running", std::string());
        if (j.contains("playing")) {
            if (j["playing"].is_string()) s.playing.push_back(j["playing"].get<std::string>());
            else if (j["playing"].is_array()) for (const nlohmann::json& n : j["playing"]) if (n.is_string()) s.playing.push_back(n.get<std::string>());
        }
        s.volume = j.value("volume", 1.0f);
        s.pitch = j.value("pitch", 1.0f);
        s.range = j.value("range", 12.0);
        sounds_[it.key()] = s;
    }
    soundsRead_ = true;
    tunes_ = nlohmann::json();
    if (!readScriptList("tunes.lua", std::string(), tunes_, &why)) LOG_WARN << "[prefab uses] tunes.lua: " << why << " (nothing plays music)";
#endif
}

// One of the engine's procedural sounds by name, made the first time it is asked for.
AudioClipHandle PrefabUseSystem::clip(FrameContext& ctx, const std::string& name) {
    const std::map<std::string, AudioClipHandle>::const_iterator had = clips_.find(name);
    if (had != clips_.end()) return had->second;
    const uint32_t rate = ctx.audio.sampleRate();
    std::vector<float> pcm;
    if (name == "latch_open") pcm = sfx::panelOpen(rate, 3);
    else if (name == "door_shut") pcm = sfx::panelShut(sfx::PanelKind::Door, rate, 3);
    else if (name == "lid_shut") pcm = sfx::panelShut(sfx::PanelKind::Trunk, rate, 5);
    else if (name == "heavy_shut") pcm = sfx::panelShut(sfx::PanelKind::Hood, rate, 7);
    else if (name == "knock") pcm = sfx::impact(rate, 13);
    else if (name == "tick") pcm = sfx::elevatorButton(rate, 3);
    else if (name == "ding") pcm = sfx::elevatorDing(true, rate, 7);
    else if (name == "sliding_doors") pcm = sfx::elevatorDoors(rate, 5);
    else if (name == "hum") pcm = sfx::elevatorHum(rate, 9);
    else if (name == "winch") pcm = sfx::reelWinch(rate, 4);
    else if (name == "roll") pcm = sfx::boardRoll(rate, 6);
    else if (name == "freewheel") pcm = sfx::freewheel(rate, 2);
    else if (name.rfind("tune:", 0) == 0) {
        // A TUNE (tunes.lua): each part a line of notes played from the start together. A note is `C4:1` (a pitch and
        // how many beats), `C4+E4+G4:2` (a chord), `-:1` (a rest); `|` is only for the eye.
        const std::string which = name.substr(5);
        if (tunes_.is_object() && tunes_.contains(which) && tunes_[which].is_object()) {
            const nlohmann::json& t = tunes_[which];
            std::vector<sfx::TuneNote> notes;
            int part = 0;
            for (const nlohmann::json& line : t.value("parts", nlohmann::json::array())) {
                if (!line.is_string()) continue;
                std::istringstream words(line.get<std::string>());
                std::string w;
                double at = 0;
                while (words >> w) {
                    const std::size_t colon = w.find(':');
                    if (w == "|" || colon == std::string::npos) continue;
                    const double beats = std::atof(w.substr(colon + 1).c_str());
                    std::istringstream chord(w.substr(0, colon));
                    std::string one;
                    while (std::getline(chord, one, '+')) {
                        if (one.empty() || one == "-") continue;
                        static const int kStep[7] = {9, 11, 0, 2, 4, 5, 7};   // A B C D E F G
                        const char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(one[0])));
                        if (letter < 'A' || letter > 'G') continue;
                        int semi = kStep[letter - 'A'];
                        std::size_t k = 1;
                        if (k < one.size() && one[k] == '#') { ++semi; ++k; }
                        else if (k < one.size() && one[k] == 'b') { --semi; ++k; }
                        const int octave = k < one.size() ? std::atoi(one.c_str() + k) : 4;
                        sfx::TuneNote n;
                        n.pitch = 12 * (octave + 1) + semi;
                        n.start = at;
                        n.length = beats;
                        n.velocity = part == 0 ? 0.85f : 0.5f;   // the tune over its accompaniment
                        notes.push_back(n);
                    }
                    at += beats;
                }
                ++part;
            }
            pcm = sfx::tune(notes, t.value("voice", std::string("piano")) == "record" ? sfx::TuneVoice::Record : sfx::TuneVoice::Piano,
                            t.value("bpm", 100.0), rate, 7);
        }
        if (pcm.empty()) LOG_WARN << "[prefab uses] no tune named '" << which << "' in tunes.lua";
    }
    else LOG_WARN << "[prefab uses] prefab_sounds.lua names the clip '" << name << "', which is not one of the engine's";
    const AudioClipHandle made = pcm.empty() ? AudioClipHandle() : ctx.audio.createClip(pcm.data(), pcm.size(), 1, rate);
    clips_[name] = made;
    return made;
}

void PrefabUseSystem::sounds(FrameContext& ctx) {
    if (!soundsRead_ || !ctx.audio.ready()) {
        ctx.world.each<PrefabUses>([&](Entity, PrefabUses& uses) { for (PrefabUses::Use& u : uses.uses) u.cues = 0; });
        return;
    }
    std::map<std::pair<uint64_t, std::size_t>, bool> wanted;
    ctx.world.each<PrefabUses>([&](Entity e, PrefabUses& uses) {
        for (std::size_t i = 0; i < uses.uses.size(); ++i) {
            PrefabUses::Use& u = uses.uses[i];
            const uint8_t cues = u.cues;
            u.cues = 0;
            if (u.use.sound.empty()) continue;
            const std::map<std::string, Sound>::const_iterator kind = sounds_.find(u.use.sound);
            if (kind == sounds_.end()) continue;
            const Sound& s = kind->second;
            const bool runs = !u.use.run.empty() && (u.on || u.coasting > 0);
            const bool plays = u.on && !s.playing.empty();   // (music: for as long as the use is on)
            const bool loop = (u.moving && !s.moving.empty()) || (runs && !s.running.empty()) || plays;
            if (cues == 0 && !loop) continue;
            Vec3 at;
            Quat turn;
            if (!modelMarkerWorld(ctx.world, e, u.use.at, at, turn)) continue;
            auto once = [&](const std::string& name) {
                if (name.empty()) return;
                const AudioClipHandle c = clip(ctx, name);
                if (!c.valid()) return;
                AudioPlayParams p;
                p.volume = s.volume;
                p.pitch = s.pitch;
                ctx.audio.playAt(c, at, s.range, p);
            };
            if (cues & PrefabUses::Use::Done) once(s.done);
            if (cues & PrefabUses::Use::Undone) once(s.undone);
            if (cues & PrefabUses::Use::Arrived) once(u.on ? s.arrived : (s.shut.empty() ? s.arrived : s.shut));
            if (!loop) continue;
            const std::pair<uint64_t, std::size_t> key{(uint64_t(e.index) << 32) | e.generation, i};
            if (plays && (cues & PrefabUses::Use::Done)) ++played_[key];   // (the next tune each time it is started)
            wanted[key] = true;
            std::map<std::pair<uint64_t, std::size_t>, AudioVoiceHandle>::iterator voice = loops_.find(key);
            if (voice == loops_.end() || !ctx.audio.isPlaying(voice->second)) {
                const AudioClipHandle c = clip(ctx, u.moving && !s.moving.empty() ? s.moving
                                                    : plays ? s.playing[(played_[key] + s.playing.size() - 1) % s.playing.size()] : s.running);
                if (!c.valid()) continue;
                AudioPlayParams p;
                p.volume = s.volume;
                p.pitch = s.pitch;
                p.loop = true;
                loops_[key] = ctx.audio.playAt(c, at, s.range, p);
            } else {
                ctx.audio.setVoicePosition(voice->second, at);
                // a part that is coasting to a stop runs down in pitch and loudness with it
                const float drive = runs && !u.on && u.use.coast > 0 ? static_cast<float>(u.coasting / u.use.coast) : 1.0f;
                ctx.audio.setVoiceVolume(voice->second, s.volume * drive);
                ctx.audio.setVoicePitch(voice->second, s.pitch * (0.5f + 0.5f * drive));
            }
        }
    });
    // a loop whose use no longer moves or runs (or whose copy has gone) stops
    for (std::map<std::pair<uint64_t, std::size_t>, AudioVoiceHandle>::iterator it = loops_.begin(); it != loops_.end();) {
        if (wanted.count(it->first) > 0) { ++it; continue; }
        ctx.audio.stop(it->second);
        it = loops_.erase(it);
    }
}

// THE LAMPS THAT ARE ON: each use with a `light` that is on publishes one candidate on the board's Prefabs channel,
// rebuilt every frame. LightSystem picks and lights them with every other light (ADR-0362); nothing here draws.
void PrefabUseSystem::render(FrameContext& ctx) {
    std::vector<LightCandidate>& lamps = ctx.view.lights.channel(LightChannel::Prefabs);
    lamps.clear();
    ctx.world.each<PrefabLamps>([&](Entity e, PrefabLamps& own) {   // (a lamp post's: always there, its kind rides dusk)
        for (const PrefabLamp& l : own.lamps) {
            Vec3 at;
            Quat turn;
            LightCandidate c;
            c.kind = lightKindId(l.kind.c_str());
            if (c.kind < 0 || !modelMarkerWorld(ctx.world, e, l.at, at, turn)) continue;
            c.position = at;
            c.direction = turn.rotate(Vec3(0, 0, 1));
            lamps.push_back(c);
        }
    });
    // ...and of the copies drawn in batches while they sleep: a lamp post's lamp, a fridge left open
    ctx.world.each<PrefabBatches>([&](Entity, PrefabBatches& b) {
        if (!b.mayLight) return;
        for (const PrefabBatches::Copy& copy : b.copies) {
            if (copy.awake.valid() || copy.gone) continue;
            auto lamp = [&](const std::string& marker, const std::string& kind) {
                Vec3 at;
                Quat turn;
                LightCandidate c;
                c.kind = lightKindId(kind.c_str());
                if (c.kind < 0 || !batchCopyMarker(copy, marker, at, turn)) return;
                c.position = at;
                c.direction = turn.rotate(Vec3(0, 0, 1));
                lamps.push_back(c);
            };
            for (const PrefabLamp& l : copy.line.lights) lamp(l.at, l.kind);
            for (std::size_t i = 0; i < copy.uses.size() && i < copy.on.size(); ++i)
                if (copy.on[i] != 0 && !copy.uses[i].light.empty()) lamp(copy.uses[i].lightAt.empty() ? copy.uses[i].at : copy.uses[i].lightAt, copy.uses[i].light);
        }
    });
    ctx.world.each<PrefabUses>([&](Entity e, PrefabUses& uses) {
        for (const PrefabUses::Use& u : uses.uses) {
            if (!u.on || u.use.light.empty()) continue;
            Vec3 at;
            Quat turn;
            if (!modelMarkerWorld(ctx.world, e, u.use.lightAt.empty() ? u.use.at : u.use.lightAt, at, turn)) continue;
            LightCandidate c;
            c.kind = lightKindId(u.use.light.c_str());
            if (c.kind < 0) continue;
            c.position = at;
            c.direction = turn.rotate(Vec3(0, 0, 1));
            lamps.push_back(c);
        }
    });
}

// THE COPIES DRAWN IN BATCHES (prefab_batch.h): the lively ones near the player wake, the awake ones that have been
// still go back, and the collision of the ones at rest is made (again, when one has woken or gone back).
void PrefabUseSystem::batches(FrameContext& ctx, Entity player, const Vec3& pos) {
    World& world = ctx.world;
    auto removeBody = [&](PhysicsBodyId id) { if (physics_ != nullptr) physics_->physicsWorld().removeBody(id); };
    std::vector<std::pair<Entity, uint32_t>> wake;
    world.each<PrefabBatches>([&](Entity holder, PrefabBatches& b) {
        if (!b.mayStir || !b.near(pos, WAKE_M)) return;
        for (std::size_t i = 0; i < b.copies.size(); ++i)
            if ((b.copies[i].lively || b.copies[i].busy > 0) && !b.copies[i].awake.valid() && (b.copies[i].at.position - pos).length() < WAKE_M)
                wake.emplace_back(holder, static_cast<uint32_t>(i));
    });
    for (const std::pair<Entity, uint32_t>& w : wake) wakeBatchCopy(world, w.first, w.second);

    const Seated* seated = player.valid() ? world.get<Seated>(player) : nullptr;
    std::vector<Entity> sleep;
    world.each<BatchAwake, Transform>([&](Entity e, BatchAwake& a, Transform& t) {
        const PrefabBatches* b = world.alive(a.holder) ? world.get<PrefabBatches>(a.holder) : nullptr;
        if (b == nullptr || a.copy >= b->copies.size()) { sleep.push_back(e); return; }   // (its holder has gone: so does it)
        const PrefabBatches::Copy& c = b->copies[a.copy];
        const bool mine = seated != nullptr && seated->set == e;
        const bool near = (t.position - pos).length() < SLEEP_M;
        // far from the player, a copy that moves by itself or under someone the city put on it goes back at once:
        // nothing is drawn moving that nobody is near enough to see
        if ((c.lively || c.busy > 0) && !near && !mine && !world.has<RidesMarker>(e)) { sleep.push_back(e); return; }
        const bool held = mine || world.has<RidesMarker>(e) || ((c.lively || c.busy > 0) && near) || !batchCopySettled(world, e);
        if (held) { a.still = 0; return; }
        a.still += ctx.frameDelta;
        if (a.still > STILL_S) sleep.push_back(e);
    });
    for (Entity e : sleep)
        if (world.alive(e)) sleepBatchCopy(world, e, removeBody);

}

// The two bodies of each holder (its copies' shapes that never move, and those on parts at rest): made when stale,
// and for a holder that says so (bodiesWithin) only while the player is near it.
void PrefabUseSystem::batchBodies(FrameContext& ctx, const Vec3& pos) {
    if (physics_ == nullptr) return;
    ctx.world.each<PrefabBatches>([&](Entity, PrefabBatches& b) {
        const bool wanted = b.bodiesWithin <= 0 || b.near(pos, b.bodiesWithin);
        const bool kept = b.bodiesWithin <= 0 || b.near(pos, b.bodiesWithin * Real(1.25));
        for (int moving = 0; moving < 2; ++moving) {
            bool& stale = moving ? b.restingStale : b.fixedStale;
            PhysicsBodyId& body = moving ? b.restingBody : b.fixedBody;
            if (body != INVALID_PHYSICS_BODY && !kept) {   // (the player has left: made again when they are back)
                physics_->physicsWorld().removeBody(body);
                body = INVALID_PHYSICS_BODY;
                stale = true;
                continue;
            }
            if (!stale || !wanted) continue;
            stale = false;
            if (body != INVALID_PHYSICS_BODY) physics_->physicsWorld().removeBody(body);
            body = INVALID_PHYSICS_BODY;
            std::vector<Vec3> vertices;
            std::vector<uint32_t> indices;
            batchCollisionTriangles(b, moving != 0, vertices, indices);
            if (!indices.empty()) body = physics_->physicsWorld().addMesh(vertices, indices, Vec3(0, 0, 0), 0.6);
        }
    });
}

void PrefabUseSystem::update(FrameContext& ctx) {
    sounds(ctx);
    Entity player;
    Vec3 pos;
    Real feetDrop = 0.7;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (player.valid()) return;
        player = e;
        pos = t.position;
        if (const CharacterController* cc = ctx.world.get<CharacterController>(e)) feetDrop = cc->halfHeight + cc->radius;
    });
    if (!player.valid()) return;
    batchBodies(ctx, pos);   // (driving too: a car meets a swing)
    if (ctx.world.has<InVehicle>(player) || ctx.world.has<Passenger>(player)) return;
    const Vec3 feet = pos - Vec3(0, feetDrop, 0);

    batches(ctx, player, pos);

    // WHAT DOES ITSELF AS SOMEONE COMES NEAR (a use's `near`): an automatic door opens, a revolving door turns. Done
    // while the player is within reach of the use's marker and on its level; kept done while they stay.
    {
        std::vector<Entity> with;
        ctx.world.each<PrefabUses>([&](Entity e, PrefabUses& u) {
            for (const PrefabUses::Use& one : u.uses) if (one.use.near > 0) { with.push_back(e); break; }
        });
        for (Entity e : with) {
            PrefabUses* uses = ctx.world.get<PrefabUses>(e);
            for (std::size_t i = 0; uses != nullptr && i < uses->uses.size(); ++i) {
                PrefabUses::Use& u = uses->uses[i];
                if (u.use.near <= 0) continue;
                Vec3 at;
                Quat turn;
                if (!modelMarkerWorld(ctx.world, e, u.use.at, at, turn)) continue;
                const bool here = std::hypot(pos.x - at.x, pos.z - at.z) < u.use.near && feet.y < at.y + Real(0.5) && feet.y > at.y - Real(3.5);
                if (here) {
                    if (!u.on) { if (prefabUseOffered(*uses, i)) setPrefabUse(ctx.world, e, i, true); }
                    else if (u.use.seconds > 0) u.left = std::max(u.left, u.use.seconds);
                } else if (u.on && u.use.seconds <= 0 && u.use.back.empty()) {
                    setPrefabUse(ctx.world, e, i, false);
                }
            }
        }
    }

    InteractCommand cmd;
    const bool commanded = takeInteractCommand(ctx.world, player, "prefab", cmd);

    // A COMMAND FOR A COPY DRAWN IN A BATCH: it wakes, and the command is then the stamped copy's (its uses have the
    // same numbers: prefabUsesFor).
    if (commanded && cmd.entity.valid() && ctx.world.alive(cmd.entity) && ctx.world.has<PrefabBatches>(cmd.entity)) {
        const Entity woken = wakeBatchCopy(ctx.world, cmd.entity, static_cast<uint32_t>(cmd.key >> 16));
        cmd.key &= 0xffff;
        cmd.entity = woken;
    }

    // SEATED ON A PREFAB: stand up on the command, on any move key, or when the copy has gone.
    if (Seated* s = ctx.world.get<Seated>(player)) {
        const bool mine = s->set.valid() && ctx.world.alive(s->set) && ctx.world.has<PrefabUses>(s->set);
        if (!mine) return;   // the furniture's, or a seat whose owner has gone (InteractionSystem stands them up)
        const PrefabUses* uses = ctx.world.get<PrefabUses>(s->set);
        const bool move = std::fabs(ctx.actions.axis("cam_forward")) > 0.5 || std::fabs(ctx.actions.axis("cam_right")) > 0.5;
        if (move || commanded || s->piece >= uses->uses.size()) {
            setPrefabUse(ctx.world, s->set, s->piece, false);   // the seat is free again
            ctx.world.remove<Seated>(player);
            return;
        }
        Seated now = *s;
        if (seatAt(ctx.world, s->set, uses->uses[s->piece].use.at, uses->uses[s->piece].use.lie, now)) {   // carried with the seat
            s->eye = now.eye;
            s->exit = now.exit;
        }
        InteractOffer o;
        o.provider = "prefab";
        o.entity = s->set;
        o.key = s->piece;
        o.anchor = s->eye;
        o.tap = uses->uses[s->piece].use.lie ? "get up" : "stand up";
        o.name = uses->uses[s->piece].use.lie ? "lying down" : "seated";
        o.exclusive = true;
        offerInteraction(ctx.world, player, o);
        return;
    }

    // THE COMMAND: do the use the broker focused.
    if (commanded && cmd.entity.valid() && ctx.world.alive(cmd.entity)) {
        if (const PrefabUses* uses = ctx.world.get<PrefabUses>(cmd.entity); uses != nullptr && cmd.key < uses->uses.size()) {
            const PrefabUse use = uses->uses[static_cast<std::size_t>(cmd.key)].use;
            const bool done = doPrefabUse(ctx.world, cmd.entity, static_cast<std::size_t>(cmd.key));
            if (done && (use.sit || use.lie)) {
                Seated seat;
                seat.set = cmd.entity;
                seat.piece = static_cast<uint32_t>(cmd.key);
                if (seatAt(ctx.world, cmd.entity, use.at, use.lie, seat)) ctx.world.add<Seated>(player, seat);
            }
            return;
        }
    }

    // THE OFFERS: every use whose marker is in reach.
    struct Near {
        Entity e;
        std::size_t use;
        Vec3 at;
    };
    std::vector<Near> near;
    ctx.world.each<PrefabUses, Transform>([&](Entity e, PrefabUses& uses, Transform& t) {
        // (the copy's own point first: most copies are nowhere near)
        if (std::fabs(t.position.x - feet.x) > 12 || std::fabs(t.position.z - feet.z) > 12 || std::fabs(t.position.y - feet.y) > 6) return;
        for (std::size_t i = 0; i < uses.uses.size(); ++i) {
            Vec3 at;
            Quat turn;
            if ((uses.uses[i].use.sit || uses.uses[i].use.lie) && uses.uses[i].on) continue;   // (someone is on it)
            if (!prefabUseOffered(uses, i)) continue;
            if (!modelMarkerWorld(ctx.world, e, uses.uses[i].use.at, at, turn)) continue;
            if (std::hypot(at.x - feet.x, at.z - feet.z) > REACH || at.y < feet.y - 0.5 || at.y > feet.y + 2.6) continue;
            near.push_back(Near{e, i, at});
        }
    });
    // ...and the uses of the copies drawn in batches, found from each copy's model and place. The offer names the
    // holder and packs the copy and the use into its key; taking it wakes the copy (above).
    struct BatchNear {
        Entity holder;
        uint32_t copy;
        std::size_t use;
        Vec3 at;
        std::string tap, name;
        bool seat;
    };
    std::vector<BatchNear> batchNear;
    ctx.world.each<PrefabBatches>([&](Entity holder, PrefabBatches& b) {
        if (!b.near(feet, 14)) return;
        for (std::size_t ci = 0; ci < b.copies.size(); ++ci) {
            const PrefabBatches::Copy& c = b.copies[ci];
            if (c.awake.valid() || c.gone || c.uses.empty()) continue;
            if (std::fabs(c.at.position.x - feet.x) > 12 || std::fabs(c.at.position.z - feet.z) > 12 || std::fabs(c.at.position.y - feet.y) > 6) continue;
            PrefabUses as;
            for (std::size_t i = 0; i < c.uses.size(); ++i) {
                PrefabUses::Use u{c.uses[i]};
                u.on = i < c.on.size() && c.on[i] != 0;
                as.uses.push_back(u);
            }
            for (std::size_t i = 0; i < as.uses.size(); ++i) {
                Vec3 at;
                Quat turn;
                if ((as.uses[i].use.sit || as.uses[i].use.lie) && as.uses[i].on) continue;   // (someone is on it)
                if (!prefabUseOffered(as, i) || !batchCopyMarker(c, as.uses[i].use.at, at, turn)) continue;
                if (std::hypot(at.x - feet.x, at.z - feet.z) > REACH || at.y < feet.y - 0.5 || at.y > feet.y + 2.6) continue;
                batchNear.push_back(BatchNear{holder, static_cast<uint32_t>(ci), i, at, prefabUseWord(as.uses[i]), c.prefab,
                                              as.uses[i].use.sit || as.uses[i].use.lie});
            }
        }
    });
    for (const BatchNear& n : batchNear) {
        InteractOffer o;
        o.provider = "prefab";
        o.entity = n.holder;
        o.key = (static_cast<uint64_t>(n.copy) << 16) | static_cast<uint64_t>(n.use);
        o.anchor = n.at;
        o.tap = n.tap;
        o.name = n.name;
        o.reach = REACH;
        o.needsSight = !n.seat;
        offerInteraction(ctx.world, player, o);
    }
    for (const Near& n : near) {
        const PrefabUses* uses = ctx.world.get<PrefabUses>(n.e);
        const ModelInstance* mi = ctx.world.get<ModelInstance>(n.e);
        InteractOffer o;
        o.provider = "prefab";
        o.entity = n.e;
        o.key = n.use;
        o.anchor = n.at;
        o.tap = prefabUseWord(uses->uses[n.use]);
        o.name = mi != nullptr ? mi->prefab : std::string("prefab");
        o.reach = REACH;
        // (a seat is inside the thing's own collision shape as often as not: no sight test, as for a swing)
        o.needsSight = !(uses->uses[n.use].use.sit || uses->uses[n.use].use.lie);
        offerInteraction(ctx.world, player, o);
    }
}

}  // namespace engine
