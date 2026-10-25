#include "train_system.h"
#include "../../engine/riding.h"
#include "city_render.h"
#include "station_boards.h"
#include "station_entrance.h"

#include "../../engine/asset_manager.h"
#include "../../engine/audio/sfx.h"
#include "../../engine/procgen/vehicle/occupant.h"
#include "../../engine/interact_broker.h"
#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/input/input_map.h"
#include "../../engine/systems/audio_system.h"   // the cabin's barrier
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace citysim {

using engine::Entity;
using engine::InstanceGroup;
using engine::Mat4;
using engine::Real;
using engine::Transform;
using engine::Vec3;

namespace {

constexpr Real kWalk = 1.8;               // m/s along a carriage
constexpr Real kStandingEye = 1.62;       // eye above the floor
constexpr Real kEyeAboveTransform = 0.7;  // PlayerSystem's eye above the capsule centre
constexpr Real kWallClear = 0.28;         // how near a wall a standing body comes
constexpr Real kSeatedEye = 0.74;         // eye above the seat's hip point
constexpr Real kSeatReach = 2.2;          // seats offered within this of where you stand

Vec3 col(const Mat4& m, int c) { return Vec3(m.m[0][c], m.m[1][c], m.m[2][c]); }
Vec3 toLocal(const Mat4& m, const Vec3& p) {
    const Vec3 d = p - col(m, 3);
    return Vec3(engine::dot(d, col(m, 0)), engine::dot(d, col(m, 1)), engine::dot(d, col(m, 2)));
}

Entity makeGroup(engine::FrameContext& ctx, const engine::RenderMesh& mesh, const std::string& name, bool glass) {
    if (mesh.vertices.empty()) return Entity{};
    const Entity e = ctx.world.create();
    InstanceGroup g;
    g.mesh = ctx.assets.acquireMesh(mesh, name);
    engine::RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);
    if (glass) { m.roughness = 0.06f; m.opacity = 0.30f; m.metallic = 0.0f; }
    else { m.roughness = 0.45f; m.metallic = 0.15f; }
    g.material = m;
    g.renderLayer = engine::LayerSim;
    g.drawClass = engine::DrawClass::SimBody;
    ctx.world.add<InstanceGroup>(e, g);
    return e;
}

}  // namespace

void TrainSystem::build(engine::FrameContext& ctx) {
    tried_ = true;
    std::vector<engine::RailTrack> tracks;
    ctx.world.each<engine::RailTracks>([&](Entity, engine::RailTracks& rt) {
        tracks.insert(tracks.end(), rt.tracks.begin(), rt.tracks.end());
    });
    if (tracks.empty()) return;
    // THE STOCK, per line by its track's class: a metro's subway, a city line's light rail, else intercity (RT_TRAIN_KIND
    // puts one kind on every line, for a look at the others on the same rails)
    auto kindOf = [](const std::string& cls) {
        std::string name = cls == "metro" ? "subway" : cls == "light_rail" ? "light_rail" : "intercity";
        if (const char* k = std::getenv("RT_TRAIN_KIND")) name = k;
        return trainKindNamed(name);
    };
    // THE TIMETABLE: an intercity train per ~9 km of line, a city train per ~1.5 km, two at least -- the island's 37 km
    // loop ran two each way, a train every ~14 minutes, and its platforms filled with people waiting (46 at 21 min)
    auto countOf = [](double length, const TrainKind& k) {
        if (const char* n = std::getenv("RT_TRAINS_PER_LINE")) return std::max(1, std::atoi(n));
        return std::max(2, static_cast<int>(std::lround(length / (k.name == "intercity" ? 9000.0 : 1500.0))));
    };
    sim_.build(tracks, kindOf, countOf);
    if (city_) city_->simMutable().setRail(&sim_);   // the city's people take these trains
    // one model set per kind running
    stockOf_.clear();
    for (const Train& T : sim_.trains()) {
        int k = -1;
        for (std::size_t i = 0; i < stock_.size(); ++i) if (stock_[i].name == T.kind.name) k = static_cast<int>(i);
        if (k < 0) {
            Stock st;
            st.name = T.kind.name;
            engine::TrainBodySpec spec;
            spec.kind = T.kind.name;
            spec.width = T.kind.width;
            spec.floor = T.kind.floor;
            spec.gap = T.kind.gap;
            spec.length = T.kind.locoLength;
            st.loco = engine::buildTrainLocomotive(spec);
            spec.length = T.kind.carLength;
            st.coach = engine::buildTrainCarriage(spec);
            const std::string tag = "train:" + st.name + ":";
            st.locoBody = makeGroup(ctx, st.loco.body, tag + "loco", false);
            st.locoGlass = makeGroup(ctx, st.loco.glass, tag + "locoglass", true);
            st.coachBody = makeGroup(ctx, st.coach.body, tag + "coach", false);
            st.coachGlass = makeGroup(ctx, st.coach.glass, tag + "coachglass", true);
            st.leaves = makeGroup(ctx, st.coach.doorLeaf, tag + "door", false);
            LOG_INFO << "[trains] " << st.name << " stock: coach " << st.coach.body.indices.size() / 3 << " tris, " << st.coach.seats.size()
                     << " seats; locomotive " << st.loco.body.indices.size() / 3 << " tris";
            stock_.push_back(std::move(st));
            k = static_cast<int>(stock_.size()) - 1;
        }
        stockOf_.push_back(k);
    }
    // THE STATION ENTRANCES (station_entrance.h): a pavilion over each station's way in, on the city's ground, its
    // walls and machines solid; its name board joins the platforms' below
    std::vector<StationBoard> entranceBoards;
    if (city_) {
        auto ground = [&](double x, double z) { return static_cast<double>(city_->groundHeightAt(x, z)); };
        int built = 0;
        for (const StationEntrance& se : stationEntrances(sim_, ground)) {
            const EntranceMeshes em = stationEntranceMeshes(se, ground);
            const Entity body = makeGroup(ctx, em.body, "station:entrance:" + se.name, false);
            const Entity glass = makeGroup(ctx, em.glass, "station:entranceglass:" + se.name, true);
            for (Entity e : {body, glass}) {
                InstanceGroup* g = e.valid() ? ctx.world.get<InstanceGroup>(e) : nullptr;
                if (!g) continue;
                g->transforms.push_back(Mat4::identity());
                g->boundsCenter = se.foot;
                g->boundsRadius = 14.0;
                g->drawDistance = 600.0;
                g->drawClass = engine::DrawClass::Furniture;
                if (e == body) { g->material.metallic = 0.0f; g->material.roughness = 0.6f; }
                entrances_.push_back(e);
            }
            if (!em.collider.indices.empty()) {
                std::vector<Vec3> verts;
                for (const engine::Vertex& v : em.collider.vertices) verts.push_back(v.position);
                std::vector<uint32_t> idx = em.collider.indices;
                const std::size_t one = idx.size();
                for (std::size_t i = 0; i + 2 < one; i += 3) { idx.push_back(idx[i]); idx.push_back(idx[i + 2]); idx.push_back(idx[i + 1]); }
                physics_.physicsWorld().addMesh(verts, idx, Vec3(0, 0, 0), 0.8);
            }
            entranceBoards.push_back({se.name, em.board});
            ++built;
        }
        LOG_INFO << "[trains] station entrances: " << built;
    }
    // THE STATIONS' NAMES on their platforms (station_boards.h): one frame group, one lettered group per name
    if (const engine::Font* font = engine::signFont()) {
        std::vector<StationBoard> boards = stationBoards(sim_);
        boards.insert(boards.end(), entranceBoards.begin(), entranceBoards.end());
        auto place = [&](Entity e, const std::vector<const StationBoard*>& on) {
            InstanceGroup* g = e.valid() ? ctx.world.get<InstanceGroup>(e) : nullptr;
            if (!g || on.empty()) return;
            Vec3 c(0, 0, 0);
            for (const StationBoard* b : on) { g->transforms.push_back(b->m); c = c + col(b->m, 3); }
            c = c * (1.0 / static_cast<double>(on.size()));
            Real r = 0;
            for (const StationBoard* b : on) r = std::max(r, (col(b->m, 3) - c).length());
            g->boundsCenter = c;
            g->boundsRadius = r + 8;
            g->drawDistance = 400.0;
        };
        std::vector<const StationBoard*> all;
        for (const StationBoard& b : boards) all.push_back(&b);
        const Entity frame = makeGroup(ctx, stationBoardFrame(), "train:boardframe", false);
        if (InstanceGroup* g = frame.valid() ? ctx.world.get<InstanceGroup>(frame) : nullptr) { g->material.metallic = 0.0f; g->material.roughness = 0.6f; }
        place(frame, all);
        boards_.push_back(frame);
        std::vector<std::string> names;
        for (const StationBoard& b : boards) if (std::find(names.begin(), names.end(), b.name) == names.end()) names.push_back(b.name);
        const engine::RenderMesh faces = stationBoardFaces();
        for (const std::string& name : names) {
            engine::TextImage img;
            stationBoardImage(*font, name, img);
            const Entity e = makeGroup(ctx, faces, "train:boardfaces", false);
            InstanceGroup* g = e.valid() ? ctx.world.get<InstanceGroup>(e) : nullptr;
            if (!g) continue;
            g->material.albedoMap = ctx.renderer.uploadTexture(img.w, img.h, 4, img.rgba.data());
            g->material.metallic = 0.0f;
            g->material.roughness = 0.5f;
            g->material.flags |= engine::RenderMaterial::FLAG_TWO_SIDED;
            g->drawClass = engine::DrawClass::Furniture;
            std::vector<const StationBoard*> on;
            for (const StationBoard& b : boards) if (b.name == name) on.push_back(&b);
            place(e, on);
            boards_.push_back(e);
        }
        LOG_INFO << "[trains] station name boards: " << boards.size() << " for " << names.size() << " stations";
    }
    // THE CITY'S PEOPLE ABOARD: seated figures, three outfits, in the coach's seats as the riders fill them
    if (city_) {
        const Vec3 shirts[3] = {Vec3(0.30, 0.36, 0.62), Vec3(0.66, 0.30, 0.22), Vec3(0.84, 0.80, 0.70)};
        const Vec3 trousers[3] = {Vec3(0.18, 0.18, 0.20), Vec3(0.30, 0.27, 0.22), Vec3(0.14, 0.16, 0.24)};
        const Vec3 skins[3] = {Vec3(0.78, 0.60, 0.48), Vec3(0.50, 0.35, 0.25), Vec3(0.90, 0.74, 0.62)};
        for (int k = 0; k < 3; ++k) {
            engine::OccupantParams op;
            op.hipToHeel = 0.45;
            op.sittingHeight = 0.86;
            op.backAngle = 12.0;
            op.armReach = 24.0;
            op.shirt = shirts[k];
            op.trousers = trousers[k];
            op.skin = skins[k];
            riders_[k] = makeGroup(ctx, engine::buildSeatedOccupant(op), "train:rider" + std::to_string(k), false);
            if (InstanceGroup* g = riders_[k].valid() ? ctx.world.get<InstanceGroup>(riders_[k]) : nullptr) {
                g->material.metallic = 0.0f;
                g->material.roughness = 0.85f;
                g->drawDistance = 160.0;
            }
        }
    }
    LOG_INFO << "[trains] " << sim_.describe();
}

// Can a body stand at `local` in car `car`? The aisle; full width in a vestibule or a doorway; the gangway's width at
// the ends. `doorSide`: the side whose doors are open (a doorway reaches out to the sill there), 0 none.
bool TrainSystem::walkableAt(int train, int car, const Vec3& local, int doorSide) const {
    const engine::TrainBodyMesh& M = model(train, car);
    if (!M.walkable) return false;
    const Real hl = static_cast<Real>(M.length) / 2, hw = static_cast<Real>(M.width) / 2;
    const Real ax = std::fabs(local.x);
    if (std::fabs(local.z) > hl - 0.15) return ax <= static_cast<Real>(M.gangwayHalf) - 0.2;   // the gangway
    for (double dz : M.doorZ)
        if (std::fabs(local.z - static_cast<Real>(dz)) < static_cast<Real>(M.doorHalf) - 0.15) {
            if (doorSide != 0 && local.x * doorSide > 0) return ax <= hw + 0.6;   // out through the open door
            return ax <= hw - kWallClear;
        }
    // an intercity car's vestibules (between its end and the first seat bay) are full width too
    if (!M.seats.empty()) {
        Real zMin = 1e9, zMax = -1e9;
        for (const Vec3& s : M.seats) { zMin = std::min(zMin, s.z); zMax = std::max(zMax, s.z); }
        if (local.z < zMin - 0.6 || local.z > zMax + 0.6) return ax <= hw - kWallClear;
    }
    return ax <= static_cast<Real>(M.aisleHalf);
}

// SITTING (Glenn, 2026-10-10: "You can't sit anywhere. There's no interaction for sitting"): aboard, every seat within
// reach is an offer of the interaction broker's E, the marker on the seat you look at; seated, the one verb is standing.
// THE TRAINS' SOUNDS, heard from the camera (Glenn: "there's no sound on the train. No train track clacking. No whistle
// or bell when it stops or when the doors open"). Each axle near the ear clacks as it rolls over a rail joint -- every
// kJointSpacing of its own travel, so a car's two bogies give the "ta-dum ... ta-dum" and the beat quickens with speed;
// under it a rolling rumble on the nearest car; the horn coming up to a platform; a chime as the doors open and three
// beeps before they shut. Aboard, the same sounds are simply near: the ear is in the car.
void TrainSystem::sounds(engine::FrameContext& ctx) {
    if (sim_.trains().empty() || !ctx.audio.ready()) return;
    if (!audioTried_) {
        audioTried_ = true;
        const uint32_t rate = ctx.audio.sampleRate();
        auto clip = [&](const std::vector<float>& pcm) { return ctx.audio.createClip(pcm.data(), pcm.size(), 1, rate); };
        for (int k = 0; k < 3; ++k) clack_[k] = clip(engine::sfx::railClack(rate, 11 + 3 * k));
        rumble_ = clip(engine::sfx::railRumble(rate, 5));
        horn_ = clip(engine::sfx::trainHorn(rate, 1));
        chimeOpen_ = clip(engine::sfx::trainDoorChime(true, rate, 1));
        chimeShut_ = clip(engine::sfx::trainDoorChime(false, rate, 1));
    }
    constexpr double kJointSpacing = 18.0;   // metres of rail between joints (jointed track: the clatter people expect)
    constexpr double kClackRange = 70.0, kRumbleRange = 140.0, kHornRange = 600.0, kChimeRange = 35.0;
    const Vec3 ear = ctx.view.camera.position;
    // ABOARD, the line is heard through the car's body (the horn up on the cab roof too); the rumble under
    // the floor is in here with us.
    if (train_ >= 0 && car_ >= 0) {
        const CarPose P = sim_.carPose(train_, car_, 1.0);
        const engine::Vec3 half(1.5, 2.0, P.length * 0.5);
        ctx.audio.setListenerCabin(engine::AudioSystem::cabinEnclosureId((uint64_t(2) << 32) | uint64_t(train_)),
                                   P.m.transformPoint(engine::Vec3(0, half.y, 0)), engine::Quat::fromRotationMatrix(P.m),
                                   half, engine::AudioSystem::kCabin);
    }
    const auto& trains = sim_.trains();
    if (ears_.size() != trains.size()) ears_.assign(trains.size(), TrainEar{});
    engine::AudioPlayParams pp;
    pp.bus = engine::AudioBus::Sfx;
    for (std::size_t ti = 0; ti < trains.size(); ++ti) {
        const Train& T = trains[ti];
        TrainEar& E = ears_[ti];
        const RailLine& L = sim_.lines()[static_cast<std::size_t>(T.line)];
        const std::vector<CarPose> cars = sim_.carPoses(static_cast<int>(ti), 1.0);
        // the nearest car to the ear
        double nearD = 1e30;
        Vec3 nearC(0, 0, 0);
        for (const CarPose& P : cars) {
            const Vec3 c = col(P.m, 3);
            const double d = (c - ear).length();
            if (d < nearD) { nearD = d; nearC = c; }
        }
        // CLATTER: axles (two per bogie, 2.5 m apart; bogies at 0.18 and 0.82 of each body) past a joint since last frame
        if (E.lastOdo >= 0 && T.odometer > E.lastOdo && nearD < kClackRange + 30) {
            const double speed = std::min(1.0, T.v / 25.0);
            double front = 0;   // metres behind the head
            for (int c = 0; c < static_cast<int>(cars.size()); ++c) {
                const double len = cars[static_cast<std::size_t>(c)].length;
                for (double b : {0.18, 0.82})
                    for (double ax : {-1.25, 1.25}) {
                        const double off = front + b * len + ax;
                        if (std::floor((T.odometer - off) / kJointSpacing) <= std::floor((E.lastOdo - off) / kJointSpacing)) continue;
                        const Vec3 at = L.at(T.s - off);
                        if ((at - ear).length() > kClackRange) continue;
                        pp.volume = static_cast<float>(0.25 + 0.6 * speed);
                        pp.pitch = static_cast<float>(0.92 + 0.16 * speed);
                        ctx.audio.playAt(clack_[static_cast<std::size_t>(c + (ax > 0 ? 1 : 0)) % 3], Vec3(at.x, at.y + 0.5, at.z), static_cast<float>(kClackRange), pp);
                    }
                front += len + T.kind.gap;
            }
        }
        E.lastOdo = T.odometer;
        // RUMBLE on the nearest car, by speed
        const float roll = static_cast<float>(std::min(1.0, T.v / 20.0));
        if (nearD < kRumbleRange && roll > 0.02f) {
            if (!E.rumble.valid() || !ctx.audio.isPlaying(E.rumble)) {
                engine::AudioPlayParams lp = pp;
                lp.loop = true;
                lp.volume = 0.0f;
                E.rumble = ctx.audio.playAt(rumble_, nearC, static_cast<float>(kRumbleRange), lp);
            }
            ctx.audio.setVoicePosition(E.rumble, nearC);
            ctx.audio.setVoiceVolume(E.rumble, 0.75f * roll);
            ctx.audio.setVoicePitch(E.rumble, 0.7f + 0.5f * roll);
        } else if (E.rumble.valid()) {
            ctx.audio.stop(E.rumble);
            E.rumble = engine::AudioVoiceHandle{};
        }
        // THE HORN coming into a platform: once, ~300 m out
        if (!T.dwelling && T.nextStop >= 0 && T.nextStop < static_cast<int>(L.stops.size()) && E.hornedFor != T.nextStop) {
            double ahead = L.stops[static_cast<std::size_t>(T.nextStop)].s1 - T.s;
            if (L.loop && ahead < 0) ahead += L.length;
            if (ahead > 0 && ahead < 320.0) {
                E.hornedFor = T.nextStop;
                const Vec3 head = L.at(T.s);
                if ((head - ear).length() < kHornRange) {
                    pp.volume = 0.9f;
                    pp.pitch = 1.0f;
                    ctx.audio.playAt(horn_, Vec3(head.x, head.y + 4.0, head.z), static_cast<float>(kHornRange), pp);
                }
            }
        }
        // THE DOORS: a chime as they open, three beeps a little before they shut -- at each carriage near the ear
        auto chimeAll = [&](engine::AudioClipHandle clip) {
            for (const CarPose& P : cars) {
                if (P.loco) continue;
                const Vec3 c = col(P.m, 3);
                if ((c - ear).length() > kChimeRange + 10) continue;
                pp.volume = 0.7f;
                pp.pitch = 1.0f;
                ctx.audio.playAt(clip, Vec3(c.x, c.y + 2.2, c.z), static_cast<float>(kChimeRange), pp);
            }
        };
        if (T.doorSide != 0 && E.lastDoor == 0) { chimeAll(chimeOpen_); E.beeped = false; }
        if (T.doorSide != 0 && !E.beeped && T.dwellLeft < 5.2) { chimeAll(chimeShut_); E.beeped = true; }
        E.lastDoor = T.doorSide;
    }
}

void TrainSystem::update(engine::FrameContext& ctx) {
    sounds(ctx);
    if (train_ < 0 || car_ < 1) return;
    Entity player;
    ctx.world.each<Transform, engine::ControlledBy>([&](Entity e, Transform&, engine::ControlledBy&) { if (!player.valid()) player = e; });
    if (!player.valid()) return;
    engine::InteractCommand cmd;
    if (engine::takeInteractCommand(ctx.world, player, "train", cmd)) sitRequest_ = seat_ >= 0 ? -1 : static_cast<int>(cmd.key);
    engine::InteractOffer o;
    o.provider = "train";
    o.needsSight = false;
    const CarPose P = sim_.carPose(train_, car_, 1.0);
    if (seat_ >= 0) {
        o.tap = "stand up";
        o.name = "seat";
        o.exclusive = true;
        if (const Transform* pt = ctx.world.get<Transform>(player)) o.anchor = pt->position;
        engine::offerInteraction(ctx.world, player, o);
        return;
    }
    for (std::size_t i = 0; i < coachOf(train_).seats.size(); ++i) {
        const Vec3& s = coachOf(train_).seats[i];
        const Real dx = s.x - local_.x, dz = s.z - local_.z;
        if (dx * dx + dz * dz > kSeatReach * kSeatReach) continue;
        engine::InteractOffer so = o;
        so.key = i;
        so.anchor = P.m.transformPoint(s) + Vec3(0, 0.55, 0);
        so.carried = true;   // (re-placed on the carriage as drawn: InteractCarrier, render())
        so.local = s + Vec3(0, 0.55, 0);
        so.reach = 2.4;
        so.name = "seat";
        so.tap = "sit down";
        engine::offerInteraction(ctx.world, player, so);
    }
}

void TrainSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!tried_) build(ctx);
    if (sim_.trains().empty()) return;
    const Real dt = ctx.clock.fixedStep();
    sim_.step(dt);
    // `train_board <train> <car>`: put the player aboard (a probe's way in; on foot you walk in through a door)
    const std::string req = ctx.settings.getString("train.board", "");
    if (!req.empty()) {
        ctx.settings.setString("train.board", "");
        int t = 0, c = 1;
        if (std::sscanf(req.c_str(), "%d %d", &t, &c) >= 1 && t >= 0 && t < static_cast<int>(sim_.trains().size())) {
            train_ = t;
            car_ = std::clamp(c, 1, sim_.carCount(t) - 1);
            local_ = Vec3(0, static_cast<Real>(coachOf(train_).floor), 0);
            LOG_INFO << "[trains] aboard train " << train_ << " car " << car_ << " (by request)";
        }
    }
    ride(ctx, dt);
    if (++tick_ % 30 == 0) {
        std::string aboard = "on foot";
        if (train_ >= 0) {
            char b[160];
            std::snprintf(b, sizeof(b), "aboard train %d car %d at %.2f %.2f (speed %.1f)", train_, car_, local_.x, local_.z,
                          sim_.trains()[static_cast<std::size_t>(train_)].v);
            aboard = b;
        }
        std::string riders;
        if (city_) {   // the city's people on the line: how many chose it, where they are now
            const RailRiders& rr = city_->sim().railRiders();
            int phase[5] = {0, 0, 0, 0, 0};
            int walking = 0, onBus = 0, still = 0;
            std::vector<double> left;   // metres to the station's way in, of those still on the street
            const CitySim& cs = city_->sim();
            for (int p : rr.passengers()) {
                const RailTrip* t = rr.tripOf(p);
                if (!t) continue;
                ++phase[static_cast<int>(t->phase)];
                if (t->phase != RailTrip::Phase::ToStation || p >= static_cast<int>(cs.agents().size())) continue;
                const Agent& a = cs.agents()[static_cast<std::size_t>(p)];
                if (cs.buses().tripOf(p)) ++onBus; else if (a.moving) ++walking; else ++still;
                const Vec2 acc = rr.stations()[static_cast<std::size_t>(t->from)].access;
                left.push_back(std::hypot(a.pos.x - acc.x, a.pos.y - acc.y));
            }
            std::sort(left.begin(), left.end());
            char lb[160];
            std::snprintf(lb, sizeof(lb), " (walking %d, bus %d, standing %d; to go: nearest %.0f m, median %.0f m)", walking, onBus,
                          still, left.empty() ? 0.0 : left.front(), left.empty() ? 0.0 : left[left.size() / 2]);
            char b[600];
            std::snprintf(b, sizeof(b), "; riders: %zu stations pinned, planned %ld entered %ld boarded %ld set down %ld gave up %ld | "
                          "now to station %d%s, in the way in %d, on platforms %d, aboard %d, on the way out %d",
                          rr.stations().size(), rr.stats.planned, rr.stats.entered, rr.stats.boarded, rr.stats.setDown,
                          rr.stats.gaveUp, phase[0], lb, phase[1], phase[2], phase[3], phase[4]);
            riders = b;
        }
        ctx.settings.setString("trains.telemetry", sim_.describe() + "; " + aboard + riders);
    }
}

void TrainSystem::ride(engine::FrameContext& ctx, Real dt) {
    Entity player;
    ctx.world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&, engine::ControlledBy&) { if (!player.valid()) player = e; });
    if (!player.valid()) { train_ = -1; return; }
    Transform* pt = ctx.world.get<Transform>(player);
    auto* cc = ctx.world.get<engine::CharacterController>(player);
    if (!pt || !cc) return;
    auto pin = [&](const Vec3& pos) {
        pt->position = pos;
        if (auto* prev = ctx.world.get<engine::PrevTransform>(player)) prev->value = *pt;
        if (cc->characterId != engine::INVALID_CHARACTER) physics_.physicsWorld().setCharacterPosition(cc->characterId, pos);
        pinned_ = pos;
    };
    auto leave = [&](const Vec3& where) {
        pin(where);
        if (ctx.world.has<engine::Passenger>(player)) ctx.world.remove<engine::Passenger>(player);
        LOG_INFO << "[trains] you stepped off train " << train_ << " at (" << where.x << ", " << where.z << ")";
        train_ = car_ = -1;
        seat_ = -1;
    };

    if (train_ >= 0) {
        // moved by something else (fast travel, teleport): that is getting off
        if ((pt->position - pinned_).length() > 3.0 && pinned_.lengthSquared() > 0) {
            if (ctx.world.has<engine::Passenger>(player)) ctx.world.remove<engine::Passenger>(player);
            train_ = car_ = -1;
            seat_ = -1;
            return;
        }
    } else {
        // ON FOOT: walking into an open doorway from the platform is boarding
        // (the capsule's own centre-to-feet: it was the 0.7 eye offset, right only while the player was 1.4 m tall)
        const Vec3 feet = pt->position - Vec3(0, cc ? cc->halfHeight + cc->radius : Real(0.9), 0);
        for (int ti = 0; ti < static_cast<int>(sim_.trains().size()) && train_ < 0; ++ti) {
            const Train& T = sim_.trains()[static_cast<std::size_t>(ti)];
            if (T.doorSide == 0) continue;
            for (int c = 1; c < sim_.carCount(ti); ++c) {
                const CarPose P = sim_.carPose(ti, c, 1.0);
                const Vec3 l = toLocal(P.m, feet);
                if (std::fabs(l.z) > P.length / 2 + 1 || l.x * T.doorSide < 0) continue;
                if (std::fabs(l.y - coachOf(ti).floor) > 0.6) continue;   // at platform height, not down on the ballast
                bool door = false;
                for (double dz : coachOf(ti).doorZ) door = door || std::fabs(l.z - static_cast<Real>(dz)) < coachOf(ti).doorHalf - 0.15;
                if (!door || std::fabs(l.x) > coachOf(ti).width / 2 + 0.35) continue;
                train_ = ti;
                car_ = c;
                local_ = Vec3(std::clamp(l.x, Real(-(coachOf(ti).width / 2 - kWallClear)), Real(coachOf(ti).width / 2 - kWallClear)),
                              static_cast<Real>(coachOf(ti).floor), l.z);
                if (!ctx.world.has<engine::Passenger>(player)) ctx.world.add<engine::Passenger>(player);
                LOG_INFO << "[trains] you boarded train " << ti << ", car " << c << " (feet " << feet.x << " " << feet.y << " " << feet.z
                         << ", car-local " << l.x << " " << l.y << " " << l.z << ")";
                break;
            }
        }
        if (train_ < 0) return;
    }
    if (!ctx.world.has<engine::Passenger>(player)) ctx.world.add<engine::Passenger>(player);

    const Train& T = sim_.trains()[static_cast<std::size_t>(train_)];
    CarPose P = sim_.carPose(train_, car_, 1.0);
    if (sitRequest_ != -2) {
        if (sitRequest_ == -1 && seat_ >= 0) {   // stand up in the aisle beside the seat
            const Vec3& s = coachOf(train_).seats[static_cast<std::size_t>(seat_)];
            local_ = Vec3(std::clamp(s.x, Real(-coachOf(train_).aisleHalf), Real(coachOf(train_).aisleHalf)), static_cast<Real>(coachOf(train_).floor), s.z);
            seat_ = -1;
        } else if (sitRequest_ >= 0 && sitRequest_ < static_cast<int>(coachOf(train_).seats.size())) {
            seat_ = sitRequest_;
            LOG_INFO << "[trains] you sat down: seat " << seat_ << " of car " << car_;
        }
        sitRequest_ = -2;
    }
    // HELD IN THE CARRIAGE (engine/riding.h): at `local` in this car's frame, the rest rideStep's -- the pin, the frame
    // blend, the footsteps and the body (seated, or walking only as fast as the feet go in the car: Glenn, "when my
    // guy is on the train he's walking when the train moves"), the view turning with the train
    auto hold = [&](const CarPose& in, const Vec3& local, bool sitting) {
        engine::Passenger* ps = ctx.world.get<engine::Passenger>(player);
        if (!ps) return;
        ps->carrier = in.m;
        ps->local = local;
        ps->sitting = sitting;
        ps->eyeAbove = sitting ? kSeatedEye : kStandingEye;
        ps->hipBelow = kSeatedEye - kEyeAboveTransform;
        engine::rideStep(ctx.world, &physics_.physicsWorld(), player, &fly_);
        pinned_ = pt->position;
    };
    if (seat_ >= 0) {   // seated: pinned to the seat, the walk keys do nothing
        hold(P, coachOf(train_).seats[static_cast<std::size_t>(seat_)], true);
        return;
    }
    // WALK the carriage in its own frame
    const Vec3 fwdAxis = col(P.m, 2), leftAxis = col(P.m, 0);
    Vec3 move = fly_.forward() * static_cast<Real>(ctx.actions.axis("cam_forward")) + fly_.right() * static_cast<Real>(ctx.actions.axis("cam_right"));
    move.y = 0;
    const Real ml = move.length();
    if (ml > 1) move = move * (1 / ml);
    Vec3 next = local_;
    next.x += engine::dot(move, leftAxis) * kWalk * dt;
    next.z += engine::dot(move, fwdAxis) * kWalk * dt;
    // try the step, then each axis alone (slide along a seat row or a wall)
    if (walkableAt(train_, car_, next, T.doorSide)) local_ = next;
    else if (walkableAt(train_, car_, Vec3(next.x, local_.y, local_.z), T.doorSide)) local_.x = next.x;
    else if (walkableAt(train_, car_, Vec3(local_.x, local_.y, next.z), T.doorSide)) local_.z = next.z;
    local_.y = static_cast<Real>(coachOf(train_).floor);
    // THROUGH THE GANGWAY: past the coupling's middle, the point belongs to the next car
    const Real hl = static_cast<Real>(coachOf(train_).length) / 2, half = static_cast<Real>(T.kind.gap) / 2;
    if (local_.z > hl + half - 0.05 || local_.z < -hl - half + 0.05) {
        const int to = local_.z > 0 ? car_ - 1 : car_ + 1;
        if (to >= 1 && to < sim_.carCount(train_)) {
            const Vec3 w = P.m.transformPoint(local_);
            const CarPose Q = sim_.carPose(train_, to, 1.0);
            Vec3 l = toLocal(Q.m, w);
            l.y = static_cast<Real>(coachOf(train_).floor);
            l.x = std::clamp(l.x, Real(-(coachOf(train_).gangwayHalf - 0.2)), Real(coachOf(train_).gangwayHalf - 0.2));
            car_ = to;
            local_ = l;
            P = Q;
        } else {
            local_.z = std::clamp(local_.z, -hl - half + 0.06, hl + half - 0.06);   // the train's last end: a closed door
            local_.z = std::clamp(local_.z, -hl + 0.2, hl - 0.2);
        }
    }
    // OUT OF THE DOOR: past the body's side where it is open
    if (T.doorSide != 0 && local_.x * T.doorSide > coachOf(train_).width / 2 + 0.3) {
        Vec3 out = P.m.transformPoint(Vec3(T.doorSide * static_cast<Real>(coachOf(train_).width / 2 + 0.5), static_cast<Real>(coachOf(train_).floor), local_.z));
        // the capsule's centre over the platform (its own height, not the 0.7 eye offset: that left a 1.8 m body
        // standing 0.15 m into the platform)
        out.y += (cc ? cc->halfHeight + cc->radius : Real(0.9)) + 0.05;
        leave(out);
        return;
    }
    hold(P, local_, false);   // (a gangway's re-expression is a jump in `local`: the body's walk reads no step)
}

void TrainSystem::render(engine::FrameContext& ctx) {
    if (sim_.trains().empty()) return;
    const double alpha = std::clamp(ctx.interpolation, 0.0, 1.0);
    auto group = [&](Entity e) { return e.valid() ? ctx.world.get<InstanceGroup>(e) : nullptr; };
    for (const Stock& k : stock_)
        for (Entity e : {k.locoBody, k.locoGlass, k.coachBody, k.coachGlass, k.leaves})
            if (InstanceGroup* g = group(e)) g->transforms.clear();
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (int ti = 0; ti < static_cast<int>(sim_.trains().size()); ++ti) {
        const Train& T = sim_.trains()[static_cast<std::size_t>(ti)];
        const Stock& k = stock_[static_cast<std::size_t>(stockOf_[static_cast<std::size_t>(ti)])];
        InstanceGroup *lb = group(k.locoBody), *lg = group(k.locoGlass), *cb = group(k.coachBody), *cg = group(k.coachGlass), *dl = group(k.leaves);
        const Real hw = static_cast<Real>(k.coach.width / 2), dh = static_cast<Real>(k.coach.doorHalf);
        for (const CarPose& P : sim_.carPoses(ti, alpha)) {
            const Vec3 c = col(P.m, 3);
            lo = Vec3(std::min(lo.x, c.x), std::min(lo.y, c.y), std::min(lo.z, c.z));
            hi = Vec3(std::max(hi.x, c.x), std::max(hi.y, c.y), std::max(hi.z, c.z));
            if (P.loco) {
                if (lb) lb->transforms.push_back(P.m);
                if (lg) lg->transforms.push_back(P.m);
                continue;
            }
            if (cb) cb->transforms.push_back(P.m);
            if (cg) cg->transforms.push_back(P.m);
            if (!dl) continue;
            for (double dz : k.coach.doorZ)
                for (int s = -1; s <= 1; s += 2) {
                    const Real open = T.doorSide == s ? dh : 0;   // plug doors slide along the skin
                    for (int leaf = -1; leaf <= 1; leaf += 2) {
                        const Real z = static_cast<Real>(dz) + leaf * (dh / 2 + open);
                        dl->transforms.push_back(P.m * Mat4::translate(s * (hw + 0.03), static_cast<Real>(k.coach.floor + k.coach.doorHeight / 2), z));
                    }
                }
        }
    }
    // THE CITY'S RIDERS in their seats: each aboard rider has a carriage (by a hash of who they are) and the next free
    // seat in it, a stride through the seats so a half-empty coach is spread out, not filled from one end
    InstanceGroup* rg[3] = {nullptr, nullptr, nullptr};
    for (int k = 0; k < 3; ++k) {
        rg[k] = riders_[k].valid() ? ctx.world.get<InstanceGroup>(riders_[k]) : nullptr;
        if (rg[k]) rg[k]->transforms.clear();
    }
    if (city_ && rg[0]) {
        const RailRiders& rr = city_->sim().railRiders();
        std::vector<int> filled;   // per (train, car)
        const int maxCars = 16;
        filled.assign(sim_.trains().size() * maxCars, 0);
        for (int p : rr.passengers()) {
            const RailTrip* t = rr.tripOf(p);
            if (!t || t->phase != RailTrip::Phase::Aboard || t->train < 0 || t->train >= static_cast<int>(sim_.trains().size())) continue;
            const int cars = sim_.carCount(t->train) - 1;
            const engine::TrainBodyMesh& coach = coachOf(t->train);
            const int nSeats = static_cast<int>(coach.seats.size());
            if (cars < 1 || nSeats == 0 || coach.seatYaw.size() != coach.seats.size()) continue;
            const uint32_t h = static_cast<uint32_t>(p) * 2654435761u;
            const int car = 1 + static_cast<int>((h >> 8) % static_cast<uint32_t>(cars));
            int& n = filled[static_cast<std::size_t>(t->train) * maxCars + std::min(car, maxCars - 1)];
            if (n >= nSeats) continue;
            int seat = (n * 37 + t->train * 11) % nSeats;   // 37: coprime with the seat count, every seat once
            ++n;
            if (t->train == train_ && car == car_ && seat == seat_) continue;   // the player's
            const CarPose P = sim_.carPose(t->train, car, alpha);
            const Vec3& sp = coach.seats[static_cast<std::size_t>(seat)];
            if (InstanceGroup* g = rg[(h >> 20) % 3])
                g->transforms.push_back(P.m * Mat4::translate(sp.x, sp.y, sp.z) *
                                        Mat4::rotateY(static_cast<Real>(coach.seatYaw[static_cast<std::size_t>(seat)])));
        }
    }
    const Vec3 centre = (lo + hi) * 0.5;
    const Real radius = (hi - lo).length() * 0.5 + 40;
    for (const Stock& k : stock_)
        for (Entity e : {k.locoBody, k.locoGlass, k.coachBody, k.coachGlass, k.leaves})
            if (InstanceGroup* g = group(e)) { g->boundsCenter = centre; g->boundsRadius = radius; }
    for (InstanceGroup* g : rg)
        if (g) { g->boundsCenter = centre; g->boundsRadius = radius; }

    // THE EYE IN THE CARRIAGE AS DRAWN, and the view turning with it
    Entity player;
    ctx.world.each<Transform, engine::ControlledBy>([&](Entity e, Transform&, engine::ControlledBy&) { if (!player.valid()) player = e; });
    if (player.valid() && !(train_ >= 0 && car_ >= 1) && ctx.world.has<engine::InteractCarrier>(player))
        ctx.world.remove<engine::InteractCarrier>(player);
    if (train_ >= 0 && car_ >= 1) {
        const CarPose P = sim_.carPose(train_, car_, alpha);
        // ...and the seat markers on it (the broker draws after this: InteractOffer::carried)
        if (player.valid()) {
            if (!ctx.world.has<engine::InteractCarrier>(player)) ctx.world.add<engine::InteractCarrier>(player);
            ctx.world.get<engine::InteractCarrier>(player)->drawn = P.m;
        }
    }
}

}  // namespace citysim
