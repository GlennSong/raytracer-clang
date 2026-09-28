#include "physics_world.h"

#include "jolt_job_adapter.h"
#include "../../job_system.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace engine {

JPH_SUPPRESS_WARNINGS

namespace {

// Two object layers (and a 1:1 broadphase mapping) is the standard minimal
// setup: static vs moving, so the broadphase never rebuilds the static tree.
namespace Layers {
static constexpr JPH::ObjectLayer NON_MOVING = 0;
static constexpr JPH::ObjectLayer MOVING = 1;
static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers {
static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
static constexpr JPH::BroadPhaseLayer MOVING(1);
static constexpr JPH::uint NUM_LAYERS(2);
}

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        switch (a) {
            case Layers::NON_MOVING: return b == Layers::MOVING;
            case Layers::MOVING: return true;
            default: return false;
        }
    }
};

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mapping[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mapping[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }
    JPH::uint GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::NUM_LAYERS;
    }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return mapping[layer];
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override {
        return "LAYER";
    }
#endif
private:
    JPH::BroadPhaseLayer mapping[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final
    : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
        switch (layer) {
            case Layers::NON_MOVING: return bp == BroadPhaseLayers::MOVING;
            case Layers::MOVING: return true;
            default: return false;
        }
    }
};

void traceImpl(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[1024];
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    std::fprintf(stderr, "[Jolt] %s\n", buffer);
}

// Jolt's allocator/factory/type registration is process-global, so reference-
// count it across PhysicsWorld instances (e.g. across test cases).
int g_globalRefCount = 0;

void globalAcquire() {
    if (g_globalRefCount++ == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = traceImpl;
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

void globalRelease() {
    if (--g_globalRefCount == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

inline JPH::Vec3 toJolt(const Vec3& v) {
    return JPH::Vec3(static_cast<float>(v.x), static_cast<float>(v.y),
                     static_cast<float>(v.z));
}
inline JPH::RVec3 toJoltR(const Vec3& v) {
    return JPH::RVec3(static_cast<JPH::Real>(v.x), static_cast<JPH::Real>(v.y),
                      static_cast<JPH::Real>(v.z));
}
inline JPH::Quat toJolt(const Quat& q) {
    return JPH::Quat(static_cast<float>(q.x), static_cast<float>(q.y),
                     static_cast<float>(q.z), static_cast<float>(q.w));
}
inline Vec3 fromJolt(JPH::RVec3Arg v) {
    return Vec3(static_cast<Real>(v.GetX()), static_cast<Real>(v.GetY()),
               static_cast<Real>(v.GetZ()));
}
inline Quat fromJolt(JPH::QuatArg q) {
    return Quat(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
}
inline Mat4 fromJolt(const JPH::Mat44& m) {
    Mat4 r;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            r.m[row][col] = static_cast<Real>(m(static_cast<JPH::uint>(row),
                                                static_cast<JPH::uint>(col)));
    return r;
}

// 65536: island_8_saltwood's load (5918 buildings, lots, road cells, trees, the fleet) filled 10240, and
// the terrain collider under the spawn failed to add for eight fixed steps -- rebuilt on the main thread
// each time (60 ms a step) -- until streaming freed some. A full pool drops ANY body silently.
constexpr JPH::uint MAX_BODIES = 65536;
constexpr JPH::uint MAX_BODY_PAIRS = 10240;
constexpr JPH::uint MAX_CONTACT_CONSTRAINTS = 10240;

}  // namespace

// Records every NEW contact Jolt reports (ADR-0071). OnContactAdded fires on
// physics worker threads during PhysicsSystem::Update, so the buffer is
// mutex-guarded; drain() hands the batch to the main thread after the step.
// Only added contacts are recorded — persisted (resting) contacts don't
// refire, so the stream is edge-triggered impacts, not per-step spam.
class ContactCollector final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                        const JPH::ContactManifold& manifold,
                        JPH::ContactSettings&) override {
        JPH::RVec3 point = manifold.GetWorldSpaceContactPointOn1(0);
        // Closing speed along the normal at the touch — the impact severity.
        JPH::Vec3 relVel = body2.GetPointVelocity(point) -
                           body1.GetPointVelocity(point);
        ContactEvent event;
        event.bodyA = body1.GetID().GetIndexAndSequenceNumber();
        event.bodyB = body2.GetID().GetIndexAndSequenceNumber();
        event.position = fromJolt(point);
        event.normal = fromJolt(JPH::RVec3(manifold.mWorldSpaceNormal));
        event.approachSpeed = std::fabs(relVel.Dot(manifold.mWorldSpaceNormal));
        std::lock_guard<std::mutex> lock(mutex);
        events.push_back(event);
    }

    std::vector<ContactEvent> drain() {
        std::lock_guard<std::mutex> lock(mutex);
        std::vector<ContactEvent> out;
        out.swap(events);
        return out;
    }

private:
    std::mutex mutex;
    std::vector<ContactEvent> events;
};

struct PhysicsWorld::Impl {
    // Declared before physicsSystem so they outlive it (members destruct in
    // reverse order — physicsSystem references these interfaces).
    JPH::TempAllocatorImpl tempAllocator{10 * 1024 * 1024};
    // Either a JoltJobAdapter over our pool, or a JobSystemSingleThreaded; chosen
    // in initialize() once we know whether a pool was supplied.
    std::unique_ptr<JPH::JobSystem> jobSystem;
    BPLayerInterfaceImpl broadPhaseLayers;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhase;
    ObjectLayerPairFilterImpl objectVsObject;
    ContactCollector contacts;
    JPH::PhysicsSystem physicsSystem;

    // Virtual characters live outside the body simulation, so we own them here.
    // The handle is the index; entries are never compacted (removeCharacter just
    // releases the ref) so existing CharacterIds stay valid.
    struct Character {
        JPH::Ref<JPH::CharacterVirtual> controller;
        float stepHeight = 0.4f;
        // Staged by jumpCharacter, consumed by the next moveCharacter (which
        // owns the vertical axis). 0 = not jumping this step.
        float pendingJump = 0.0f;
        float halfHeight = 0.9f;   // current capsule, for feet-anchored resizes
        float radius = 0.3f;
    };
    std::vector<Character> characters;

    // Wheeled vehicles (ADR-0059). Each owns a VehicleConstraint (also a step
    // listener) over a dynamic chassis body. Slots are never compacted so
    // VehicleIds stay valid; removeVehicle releases the ref and invalidates the id.
    struct Vehicle {
        JPH::Ref<JPH::VehicleConstraint> constraint;
        JPH::BodyID body;
        int wheels = 0;
        // For the yaw assist (VehicleConfig::yawAssist): the last steer
        // input, the steering lock and the wheelbase.
        float steer = 0.0f;
        float maxSteerRad = 0.0f;
        float wheelbase = 2.7f;
        float yawAssist = 0.0f;
        float gripAccel = 9.0f;   // the lateral acceleration the assist lets the car keep (m/s²)
        std::vector<char> diffFront;   // per differential: a front axle's?
        float frontShare = 0.5f;
        float dragArea = 0.0f;         // Cd x A (m^2); > 0 applies quadratic aero drag each step
        float centerLimitedSlip = 1.4f; // the coupling between axles in 4WD (open in 2WD)
        bool tractionSplit = false;     // VehicleConfig::tractionSplit
        float appliedShare = -1.0f;     // the front share the differentials carry this step
    };
    std::vector<Vehicle> vehicles;

    JPH::BodyInterface& bodies() { return physicsSystem.GetBodyInterface(); }
    const JPH::BodyInterface& bodies() const {
        return physicsSystem.GetBodyInterface();
    }
};

PhysicsWorld::PhysicsWorld() = default;
PhysicsWorld::~PhysicsWorld() { shutdown(); }

bool PhysicsWorld::initialize(engine::JobSystem* jobSystem) {
    if (impl) return true;
    globalAcquire();   // registers Jolt's allocator, needed before the job pool
    impl = std::make_unique<Impl>();
    // 4x Jolt's sample constant: cMaxPhysicsJobs (2048) sizes a demo scene,
    // and one metro-scale Update's job graph blew through it — the free list
    // came up empty at slot ~2047 and the adapter shipped a wild Job*
    // (the 0x27ff0 crash trio). Slots are ~128 bytes; 8192 is a megabyte of
    // insurance against a corruption class. Exhaustion within ONE Update
    // cannot be waited out (its barrier only releases refs at WaitForJobs),
    // so headroom is the fix and the adapter's retry loop is the backstop
    // for cross-frame transients only.
    constexpr JPH::uint kMaxJobs = JPH::cMaxPhysicsJobs * 4;
    if (jobSystem) {
        impl->jobSystem = std::make_unique<JoltJobAdapter>(
            *jobSystem, kMaxJobs, JPH::cMaxPhysicsBarriers);
    } else {
        impl->jobSystem =
            std::make_unique<JPH::JobSystemSingleThreaded>(kMaxJobs);
    }
    impl->physicsSystem.Init(MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS,
                             impl->broadPhaseLayers, impl->objectVsBroadPhase,
                             impl->objectVsObject);
    impl->physicsSystem.SetContactListener(&impl->contacts);
    return true;
}

std::vector<ContactEvent> PhysicsWorld::drainContactEvents() {
    if (!impl) return {};
    return impl->contacts.drain();
}

void PhysicsWorld::shutdown() {
    if (!impl) return;
    impl.reset();
    globalRelease();
}

namespace {
JPH::EMotionType toMotionType(BodyMotion m) {
    switch (m) {
        case BodyMotion::Static: return JPH::EMotionType::Static;
        case BodyMotion::Kinematic: return JPH::EMotionType::Kinematic;
        default: return JPH::EMotionType::Dynamic;
    }
}
JPH::ObjectLayer toLayer(BodyMotion m) {
    return m == BodyMotion::Static ? Layers::NON_MOVING : Layers::MOVING;
}

PhysicsBodyId createBody(JPH::BodyInterface& bodies, const JPH::Shape* shape,
                         const Vec3& position, const Quat& orientation,
                         BodyMotion motion, Real restitution, Real friction,
                         bool lockRotation = false, bool continuous = false) {
    JPH::BodyCreationSettings settings(shape, toJoltR(position),
                                       toJolt(orientation), toMotionType(motion),
                                       toLayer(motion));
    settings.mRestitution = static_cast<float>(restitution);
    settings.mFriction = static_cast<float>(friction);
    // Continuous collision (linear sweep) for fast movers — keeps the player from
    // tunnelling through thin terrain colliders on a long fall.
    if (continuous)
        settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
    if (lockRotation) {
        settings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX |
                                JPH::EAllowedDOFs::TranslationY |
                                JPH::EAllowedDOFs::TranslationZ;
    }
    JPH::EActivation activation = motion == BodyMotion::Static
                                      ? JPH::EActivation::DontActivate
                                      : JPH::EActivation::Activate;
    JPH::BodyID id = bodies.CreateAndAddBody(settings, activation);
    return id.GetIndexAndSequenceNumber();
}
}  // namespace

PhysicsBodyId PhysicsWorld::addBox(const Vec3& halfExtent, const Vec3& position,
                                   const Quat& orientation, BodyMotion motion,
                                   Real restitution, Real friction,
                                   bool lockRotation, bool continuous) {
    if (!impl) return INVALID_PHYSICS_BODY;
    JPH::BoxShapeSettings shapeSettings(toJolt(halfExtent));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return INVALID_PHYSICS_BODY;
    return createBody(impl->bodies(), result.Get(), position, orientation, motion,
                      restitution, friction, lockRotation, continuous);
}

PhysicsBodyId PhysicsWorld::addSphere(Real radius, const Vec3& position,
                                      const Quat& orientation, BodyMotion motion,
                                      Real restitution, Real friction,
                                      bool lockRotation) {
    if (!impl) return INVALID_PHYSICS_BODY;
    JPH::SphereShapeSettings shapeSettings(static_cast<float>(radius));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return INVALID_PHYSICS_BODY;
    return createBody(impl->bodies(), result.Get(), position, orientation, motion,
                      restitution, friction, lockRotation);
}

PhysicsBodyId PhysicsWorld::addCapsule(Real halfHeight, Real radius,
                                        const Vec3& position,
                                        const Quat& orientation, BodyMotion motion,
                                        Real restitution, Real friction,
                                        bool lockRotation, bool continuous) {
    if (!impl) return INVALID_PHYSICS_BODY;
    JPH::CapsuleShapeSettings shapeSettings(static_cast<float>(halfHeight),
                                             static_cast<float>(radius));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return INVALID_PHYSICS_BODY;
    return createBody(impl->bodies(), result.Get(), position, orientation, motion,
                      restitution, friction, lockRotation, continuous);
}

struct PreparedMeshShape {
    JPH::RefConst<JPH::Shape> shape;
};

std::shared_ptr<const PreparedMeshShape> PhysicsWorld::prepareMeshShape(const std::vector<Vec3>& vertices,
                                                                        const std::vector<uint32_t>& indices) {
    if (vertices.empty() || indices.size() < 3) return nullptr;
    JPH::VertexList verts;
    verts.reserve(vertices.size());
    for (const Vec3& v : vertices)
        verts.push_back(JPH::Float3(static_cast<float>(v.x), static_cast<float>(v.y),
                                    static_cast<float>(v.z)));
    // The engine winds front faces clockwise, but Jolt's one-sided mesh
    // collision follows counter-clockwise winding (its triangle normal is the
    // solid side). Reverse each triangle so the collision face matches the
    // visual outward normal — otherwise bodies fall through from "above".
    JPH::IndexedTriangleList tris;
    tris.reserve(indices.size() / 3);
    for (size_t i = 0; i + 2 < indices.size(); i += 3)
        tris.push_back(JPH::IndexedTriangle(indices[i], indices[i + 2], indices[i + 1], 0));

    JPH::MeshShapeSettings shapeSettings(verts, tris);
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return nullptr;
    auto out = std::make_shared<PreparedMeshShape>();
    out->shape = result.Get();
    return out;
}

PhysicsBodyId PhysicsWorld::addPreparedMesh(const PreparedMeshShape& shape, const Vec3& position, Real friction) {
    if (!impl || !shape.shape) return INVALID_PHYSICS_BODY;
    return createBody(impl->bodies(), shape.shape.GetPtr(), position, Quat::identity(),
                      BodyMotion::Static, 0.0, friction);
}

PhysicsBodyId PhysicsWorld::addMesh(const std::vector<Vec3>& vertices,
                                    const std::vector<uint32_t>& indices,
                                    const Vec3& position, Real friction) {
    if (!impl) return INVALID_PHYSICS_BODY;
    const auto shape = prepareMeshShape(vertices, indices);
    return shape ? addPreparedMesh(*shape, position, friction) : INVALID_PHYSICS_BODY;
}

void PhysicsWorld::removeBody(PhysicsBodyId id) {
    if (!impl || id == INVALID_PHYSICS_BODY) return;
    JPH::BodyID bid(id);
    impl->bodies().RemoveBody(bid);
    impl->bodies().DestroyBody(bid);
}

void PhysicsWorld::setLinearVelocity(PhysicsBodyId id, const Vec3& velocity) {
    if (!impl || id == INVALID_PHYSICS_BODY) return;
    impl->bodies().SetLinearVelocity(JPH::BodyID(id), toJolt(velocity));
}

Vec3 PhysicsWorld::getLinearVelocity(PhysicsBodyId id) const {
    if (!impl || id == INVALID_PHYSICS_BODY) return Vec3();
    return fromJolt(impl->bodies().GetLinearVelocity(JPH::BodyID(id)));
}

void PhysicsWorld::moveKinematic(PhysicsBodyId id, const Vec3& position,
                                 const Quat& orientation, Real dt) {
    if (!impl || id == INVALID_PHYSICS_BODY || dt <= 0) return;
    impl->bodies().MoveKinematic(JPH::BodyID(id), toJoltR(position),
                                 toJolt(orientation), static_cast<float>(dt));
}

void PhysicsWorld::teleport(PhysicsBodyId id, const Vec3& position,
                            const Quat& orientation) {
    if (!impl || id == INVALID_PHYSICS_BODY) return;
    impl->bodies().SetPositionAndRotation(JPH::BodyID(id), toJoltR(position),
                                          toJolt(orientation),
                                          JPH::EActivation::Activate);
    impl->bodies().SetLinearAndAngularVelocity(JPH::BodyID(id),
                                               JPH::Vec3::sZero(),
                                               JPH::Vec3::sZero());
}

Vec3 PhysicsWorld::bodyPosition(PhysicsBodyId id) const {
    if (!impl || id == INVALID_PHYSICS_BODY) return Vec3();
    return fromJolt(impl->bodies().GetPosition(JPH::BodyID(id)));
}

Quat PhysicsWorld::bodyOrientation(PhysicsBodyId id) const {
    if (!impl || id == INVALID_PHYSICS_BODY) return Quat();
    return fromJolt(impl->bodies().GetRotation(JPH::BodyID(id)));
}

void PhysicsWorld::setGravity(const Vec3& gravity) {
    if (impl) impl->physicsSystem.SetGravity(toJolt(gravity));
}

int PhysicsWorld::bodyCount() const {
    return impl ? static_cast<int>(impl->physicsSystem.GetNumBodies()) : 0;
}

CharacterId PhysicsWorld::addCharacter(Real halfHeight, Real radius,
                                       const Vec3& position, Real stepHeight,
                                       Real maxSlopeDegrees) {
    if (!impl) return INVALID_CHARACTER;
    JPH::CapsuleShapeSettings shapeSettings(static_cast<float>(halfHeight),
                                            static_cast<float>(radius));
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return INVALID_CHARACTER;

    JPH::Ref<JPH::CharacterVirtualSettings> settings =
        new JPH::CharacterVirtualSettings();
    settings->mShape = result.Get();
    settings->mMaxSlopeAngle =
        JPH::DegreesToRadians(static_cast<float>(maxSlopeDegrees));
    // Keep the capsule from catching on the inner edges of the baked walk-surface
    // mesh — the curb/sidewalk colliders are stitched triangle strips.
    settings->mEnhancedInternalEdgeRemoval = true;
    // The "feet" plane sits a hair below the capsule's bottom hemisphere, so a
    // contact has to be roughly underneath to count as ground (not a wall).
    settings->mSupportingVolume =
        JPH::Plane(JPH::Vec3::sAxisY(),
                   -static_cast<float>(halfHeight + radius) + 0.05f);

    Impl::Character ch;
    ch.stepHeight = static_cast<float>(stepHeight);
    ch.halfHeight = static_cast<float>(halfHeight);
    ch.radius = static_cast<float>(radius);
    ch.controller = new JPH::CharacterVirtual(settings, toJoltR(position),
                                              JPH::Quat::sIdentity(),
                                              &impl->physicsSystem);
    impl->characters.push_back(std::move(ch));
    return static_cast<CharacterId>(impl->characters.size() - 1);
}

void PhysicsWorld::removeCharacter(CharacterId id) {
    if (!impl || id >= impl->characters.size()) return;
    impl->characters[id].controller = nullptr;   // release the ref, keep the slot
}

void PhysicsWorld::moveCharacter(CharacterId id, const Vec3& velocity, Real dt) {
    if (!impl || id >= impl->characters.size()) return;
    JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    if (!ch) return;

    const JPH::Vec3 up = JPH::Vec3::sAxisY();
    JPH::Vec3 current = ch->GetLinearVelocity();
    JPH::Vec3 desired = toJolt(velocity);
    desired.SetComponent(1, 0.0f);   // horizontal intent only; we own the vertical

    // On flat ground, hold vertical velocity at zero so it doesn't accumulate;
    // otherwise carry it and integrate gravity so the character falls/settles.
    // A jump staged since the last step wins the vertical axis outright — it
    // must survive the on-ground zeroing below, because the character is by
    // definition still on the ground at the instant it launches.
    const float jump = impl->characters[id].pendingJump;
    impl->characters[id].pendingJump = 0.0f;

    // A MOVING floor carries the character (skyscrapers v2 M6: the elevator
    // cab, any kinematic mover): Jolt reports the ground body's velocity at
    // the contact — zero for everything static — and the character must add
    // it itself (CharacterVirtual owns no body). Not for a DYNAMIC body: the
    // character pushes it, inherits the push, pushes harder — a feedback
    // loop on a light crate.
    JPH::Vec3 groundVel = JPH::Vec3::sZero();
    if (ch->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround) {
        const JPH::BodyID gid = ch->GetGroundBodyID();
        if (!gid.IsInvalid() &&
            impl->bodies().GetMotionType(gid) != JPH::EMotionType::Dynamic)
            groundVel = ch->GetGroundVelocity();
    }
    JPH::Vec3 newVel;
    if (jump > 0.0f) {
        newVel = desired + groundVel + up * jump;   // a jump in a cab keeps the cab's speed
    } else if (ch->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround) {
        newVel = desired + groundVel;
    } else {
        newVel = desired + up * current.Dot(up);
    }
    newVel += impl->physicsSystem.GetGravity() * static_cast<float>(dt);
    ch->SetLinearVelocity(newVel);

    JPH::CharacterVirtual::ExtendedUpdateSettings settings;
    settings.mWalkStairsStepUp = up * impl->characters[id].stepHeight;
    // Step down by at least the step-up height so descending a curb keeps the
    // character grounded instead of briefly going airborne each step — but
    // NOT on the step that launches a jump, or the same stick-to-floor that
    // smooths curbs would pull the leap straight back onto the pavement.
    settings.mStickToFloorStepDown =
        jump > 0.0f ? JPH::Vec3::sZero()
                    : -up * std::max(impl->characters[id].stepHeight, 0.5f);

    ch->ExtendedUpdate(static_cast<float>(dt), impl->physicsSystem.GetGravity(),
                       settings,
                       impl->physicsSystem.GetDefaultBroadPhaseLayerFilter(
                           Layers::MOVING),
                       impl->physicsSystem.GetDefaultLayerFilter(Layers::MOVING),
                       {}, {}, impl->tempAllocator);
}

bool PhysicsWorld::jumpCharacter(CharacterId id, Real speed) {
    if (!impl || id >= impl->characters.size()) return false;
    JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    if (!ch || speed <= 0.0) return false;
    // Grounded only: the ground state is the whole gate, so no caller has to
    // track "am I allowed to jump" (and none can get it wrong).
    if (ch->GetGroundState() != JPH::CharacterBase::EGroundState::OnGround)
        return false;
    impl->characters[id].pendingJump = static_cast<float>(speed);
    return true;
}

bool PhysicsWorld::setCharacterHeight(CharacterId id, Real halfHeight,
                                      Real radius) {
    if (!impl || id >= impl->characters.size()) return false;
    Impl::Character& rec = impl->characters[id];
    JPH::CharacterVirtual* ch = rec.controller.GetPtr();
    if (!ch || halfHeight <= 0.0 || radius <= 0.0) return false;
    const auto newHalf = static_cast<float>(halfHeight);
    const auto newRadius = static_cast<float>(radius);
    if (std::fabs(newHalf - rec.halfHeight) < 1e-4f &&
        std::fabs(newRadius - rec.radius) < 1e-4f)
        return true;   // already that shape

    JPH::CapsuleShapeSettings shapeSettings(newHalf, newRadius);
    JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
    if (result.HasError()) return false;

    // FEET-ANCHORED: the capsule centre moves by the half-height delta so the
    // soles stay put — a crouch settles down instead of sinking, and standing
    // grows out of the top of the head. The position moves BEFORE the fit
    // test, so standing is tested where the taller capsule would actually be.
    const float deltaY = (newHalf + newRadius) - (rec.halfHeight + rec.radius);
    const JPH::RVec3 oldPos = ch->GetPosition();
    ch->SetPosition(oldPos + JPH::RVec3(0, deltaY, 0));

    // The fit test is the whole point, so the tolerance must be a real
    // number, not FLT_MAX (which skips the check): a resting character always
    // penetrates the floor slightly, so allow the solver's own slop and no
    // more — Jolt's own crouch sample uses exactly this bound.
    const float fitSlop =
        1.5f * impl->physicsSystem.GetPhysicsSettings().mPenetrationSlop;
    if (!ch->SetShape(result.Get(), fitSlop,
                      impl->physicsSystem.GetDefaultBroadPhaseLayerFilter(
                          Layers::MOVING),
                      impl->physicsSystem.GetDefaultLayerFilter(Layers::MOVING),
                      {}, {}, impl->tempAllocator)) {
        ch->SetPosition(oldPos);   // no room overhead — stay as we were
        return false;
    }
    rec.halfHeight = newHalf;
    rec.radius = newRadius;
    return true;
}

Vec3 PhysicsWorld::characterPosition(CharacterId id) const {
    if (!impl || id >= impl->characters.size()) return Vec3();
    const JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    return ch ? fromJolt(ch->GetPosition()) : Vec3();
}

bool PhysicsWorld::castRay(const Vec3& origin, const Vec3& dirAndLength,
                           Vec3& hitPoint) const {
    JPH::RRayCast ray{JPH::RVec3(origin.x, origin.y, origin.z),
                      JPH::Vec3(static_cast<float>(dirAndLength.x),
                                static_cast<float>(dirAndLength.y),
                                static_cast<float>(dirAndLength.z))};
    JPH::RayCastResult hit;
    if (!impl->physicsSystem.GetNarrowPhaseQuery().CastRay(ray, hit))
        return false;
    JPH::RVec3 p = ray.GetPointOnRay(hit.mFraction);
    hitPoint = Vec3(p.GetX(), p.GetY(), p.GetZ());
    return true;
}

Vec3 PhysicsWorld::characterVelocity(CharacterId id) const {
    if (!impl || id >= impl->characters.size()) return Vec3();
    const JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    if (!ch) return Vec3();
    JPH::Vec3 v = ch->GetLinearVelocity();
    return Vec3(static_cast<Real>(v.GetX()), static_cast<Real>(v.GetY()),
                static_cast<Real>(v.GetZ()));
}

GroundState PhysicsWorld::characterGroundState(CharacterId id) const {
    if (!impl || id >= impl->characters.size()) return GroundState::InAir;
    const JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    if (!ch) return GroundState::InAir;
    switch (ch->GetGroundState()) {
        case JPH::CharacterBase::EGroundState::OnGround:
            return GroundState::OnGround;
        case JPH::CharacterBase::EGroundState::OnSteepGround:
            return GroundState::OnSteepGround;
        case JPH::CharacterBase::EGroundState::NotSupported:
            return GroundState::NotSupported;
        default:
            return GroundState::InAir;
    }
}

void PhysicsWorld::setCharacterPosition(CharacterId id, const Vec3& position) {
    if (!impl || id >= impl->characters.size()) return;
    if (JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr()) {
        ch->SetPosition(toJoltR(position));
        // A set-position is a teleport: drop any carried velocity, or a respawned
        // character arrives still falling at terminal speed and slams into (or
        // tunnels through) whatever it lands on.
        ch->SetLinearVelocity(JPH::Vec3::sZero());
    }
}

// --- Wheeled vehicle (ADR-0059) --------------------------------------------
// Compiled against the vendored Jolt submodule and exercised headlessly by
// tests/test_driving_lab.cpp (launch envelope, lane hold, kerb climb + its
// no-clearance control, wall stop) — drive the gates there before tuning here.

PhysicsWorld::VehicleId PhysicsWorld::addVehicle(const VehicleConfig& cfg,
                                                 const Vec3& position,
                                                 const Quat& orientation) {
    if (!impl) return INVALID_VEHICLE;

    // Chassis: a dynamic box, mass overridden, sleeping disabled so the controller
    // keeps stepping. The COLLISION box is the config box minus a slice off the
    // underside (floorClearance): the specs pass the full body box, whose floor
    // rides at kerb height and rammed every kerb face nose-first. The slice is
    // carved by shrinking the half-height and shifting the box UP within the
    // body frame, so the roof stays where the spec put it and the wheels (whose
    // attach points are body-frame) are unaffected.
    const float carve = std::min(static_cast<float>(cfg.floorClearance),
                                 static_cast<float>(cfg.chassisHalfExtent.y));
    JPH::ShapeSettings::ShapeResult floorRes;
    if (cfg.approachDegrees > 0.0 || cfg.departureDegrees > 0.0) {
        // The carved box as a hull, its lower nose and tail cut back along the approach and departure
        // lines: each rises from the ground (the body box's floor, where the drawn tyres stand) under
        // its axle, and the floor runs between where the two lines cross it.
        const float hx = static_cast<float>(cfg.chassisHalfExtent.x);
        const float hy = static_cast<float>(cfg.chassisHalfExtent.y);
        const float hz = static_cast<float>(cfg.chassisHalfExtent.z);
        float frontAxle = -hz, rearAxle = hz;
        for (const VehicleWheel& w : cfg.wheels) {
            frontAxle = std::max(frontAxle, static_cast<float>(w.position.z));
            rearAxle = std::min(rearAxle, static_cast<float>(w.position.z));
        }
        if (cfg.wheels.empty()) { frontAxle = hz; rearAxle = -hz; }
        const float ground = -hy, floorY = -hy + carve, roofCut = hy - 0.1f;
        JPH::Array<JPH::Vec3> pts;
        auto end = [&](float sign, float axleZ, Real deg) {   // sign +1 = the nose (+z), -1 = the tail
            const float edgeZ = sign * hz;
            if (deg <= 0.0) {   // square: the floor runs to the end
                for (float x : {-hx, hx}) pts.push_back(JPH::Vec3(x, floorY, edgeZ));
            } else {
                const float t = std::tan(JPH::DegreesToRadians(static_cast<float>(std::min(deg, Real(80)))));
                const float over = sign * (edgeZ - axleZ);   // overhang past the axle (>= 0 normally)
                const float lipY = std::min(roofCut, std::max(floorY, ground + std::max(over, 0.0f) * t));
                const float floorEnd = axleZ + sign * std::max(0.0f, (floorY - ground) / t);
                const float fz = sign > 0 ? std::min(floorEnd, edgeZ) : std::max(floorEnd, edgeZ);
                for (float x : {-hx, hx}) {
                    pts.push_back(JPH::Vec3(x, floorY, fz));
                    pts.push_back(JPH::Vec3(x, lipY, edgeZ));
                }
            }
            for (float x : {-hx, hx}) pts.push_back(JPH::Vec3(x, hy, edgeZ));
        };
        end(1.0f, frontAxle, cfg.approachDegrees);
        end(-1.0f, rearAxle, cfg.departureDegrees);
        JPH::ConvexHullShapeSettings hull(pts, 0.03f);
        floorRes = hull.Create();
        if (floorRes.HasError()) return INVALID_VEHICLE;
    } else {
        JPH::Vec3 half = toJolt(cfg.chassisHalfExtent);
        half.SetY(half.GetY() - carve * 0.5f);
        JPH::BoxShapeSettings shapeSettings(half);
        JPH::ShapeSettings::ShapeResult shapeRes = shapeSettings.Create();
        if (shapeRes.HasError()) return INVALID_VEHICLE;
        JPH::RotatedTranslatedShapeSettings floorSettings(
            JPH::Vec3(0, carve * 0.5f, 0), JPH::Quat::sIdentity(), shapeRes.Get());
        floorRes = floorSettings.Create();
        if (floorRes.HasError()) return INVALID_VEHICLE;
    }
    // Lower the centre of mass below the chassis centre so the car resists rolling
    // in corners (the classic anti-tip tweak).
    JPH::OffsetCenterOfMassShapeSettings comSettings(
        JPH::Vec3(0, static_cast<float>(cfg.comOffsetY), 0), floorRes.Get());
    JPH::ShapeSettings::ShapeResult bodyShapeRes = comSettings.Create();
    if (bodyShapeRes.HasError()) return INVALID_VEHICLE;

    JPH::BodyCreationSettings bodySettings(bodyShapeRes.Get(), toJoltR(position),
                                           toJolt(orientation),
                                           JPH::EMotionType::Dynamic, Layers::MOVING);
    bodySettings.mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateInertia;
    bodySettings.mMassPropertiesOverride.mMass = static_cast<float>(cfg.mass);
    bodySettings.mFriction = static_cast<float>(cfg.friction);
    if (cfg.dragArea > 0.0) bodySettings.mLinearDamping = 0.0f;   // real aero drag instead (update())
    bodySettings.mAllowSleeping = false;
    JPH::Body* chassis = impl->bodies().CreateBody(bodySettings);
    if (!chassis) return INVALID_VEHICLE;
    impl->bodies().AddBody(chassis->GetID(), JPH::EActivation::Activate);

    JPH::VehicleConstraintSettings vs;
    vs.mUp = JPH::Vec3(0, 1, 0);
    vs.mForward = JPH::Vec3(0, 0, 1);
    // The roll cone: Jolt keeps the chassis up axis inside it with a rotation
    // constraint, so a trip lifts two wheels and drops back rather than going
    // over (tests/test_vehicle_handling.cpp: the slanted kerb at 65 km/h).
    if (cfg.maxPitchRollDegrees < 180.0)
        vs.mMaxPitchRollAngle = JPH::DegreesToRadians(static_cast<float>(cfg.maxPitchRollDegrees));
    for (const VehicleWheel& w : cfg.wheels) {
        JPH::WheelSettingsWV* ws = new JPH::WheelSettingsWV();
        ws->mPosition = toJolt(w.position);
        ws->mRadius = static_cast<float>(w.radius);
        ws->mWidth = static_cast<float>(w.width);
        ws->mSuspensionMinLength = static_cast<float>(w.suspensionMin);
        ws->mSuspensionMaxLength = static_cast<float>(w.suspensionMax);
        ws->mSuspensionSpring.mFrequency = static_cast<float>(w.suspensionFrequency);
        ws->mSuspensionSpring.mDamping = static_cast<float>(w.suspensionDamping);
        ws->mMaxSteerAngle = w.steered
            ? JPH::DegreesToRadians(static_cast<float>(cfg.maxSteerDegrees))
            : 0.0f;
        {   // brake balance: brakeTorque is the per-wheel average; the front axle takes brakeFrontBias of it
            int nFront = 0, nRear = 0;
            Real lo = 1e30, hi = -1e30;
            for (const VehicleWheel& o : cfg.wheels) { lo = std::min(lo, o.position.z); hi = std::max(hi, o.position.z); }
            const Real mid = 0.5 * (lo + hi);
            for (const VehicleWheel& o : cfg.wheels) (o.position.z > mid ? nFront : nRear) += 1;
            const Real total = cfg.brakeTorque * static_cast<Real>(cfg.wheels.size());
            const Real bias = std::clamp(cfg.brakeFrontBias, Real(0), Real(1));
            Real t = cfg.brakeTorque;
            if (nFront > 0 && nRear > 0)
                t = w.position.z > mid ? total * bias / nFront : total * (1.0 - bias) / nRear;
            ws->mMaxBrakeTorque = static_cast<float>(t);
        }
        // The lateral slip curve at the config's grip (Jolt's shape: a peak
        // at 3 degrees of slip, 85 % of it once sliding past 20).
        ws->mLateralFriction.Clear();
        ws->mLateralFriction.AddPoint(0.0f, 0.0f);
        ws->mLateralFriction.AddPoint(3.0f, static_cast<float>(cfg.lateralGrip));
        ws->mLateralFriction.AddPoint(20.0f, static_cast<float>(cfg.lateralGrip * 0.85));
        ws->mMaxHandBrakeTorque =
            w.handBrake ? static_cast<float>(cfg.handBrakeTorque) : 0.0f;
        vs.mWheels.push_back(ws);
    }

    // Anti-roll bars: one per axle, pairing the wheels that share a z (an
    // axle line) on opposite sides.
    if (cfg.antiRollStiffness > 0.0) {
        for (int i = 0; i < static_cast<int>(cfg.wheels.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(cfg.wheels.size()); ++j) {
                const Vec3& a = cfg.wheels[i].position;
                const Vec3& b = cfg.wheels[j].position;
                if (std::fabs(a.z - b.z) > 0.10 || a.x * b.x >= 0.0) continue;
                JPH::VehicleAntiRollBar bar;
                bar.mLeftWheel = a.x > b.x ? i : j;
                bar.mRightWheel = a.x > b.x ? j : i;
                bar.mStiffness = static_cast<float>(cfg.antiRollStiffness);
                vs.mAntiRollBars.push_back(bar);
            }
        }
    }

    JPH::WheeledVehicleControllerSettings* controller =
        new JPH::WheeledVehicleControllerSettings();
    controller->mEngine.mMaxTorque = static_cast<float>(cfg.engineTorque);
    controller->mEngine.mMaxRPM = static_cast<float>(cfg.maxRPM);

    // ONE DIFFERENTIAL PER DRIVEN AXLE (VehicleConfig drivetrain): driven wheels grouped by their z, the
    // left one the larger x (as the anti-roll bars take it), front axles those ahead of the axles' middle.
    struct Axle { Real z; std::vector<int> wheels; };
    std::vector<Axle> axles;
    for (int i = 0; i < static_cast<int>(cfg.wheels.size()); ++i) {
        if (!cfg.wheels[i].driven) continue;
        const Real z = cfg.wheels[i].position.z;
        Axle* hit = nullptr;
        for (Axle& a : axles) if (std::fabs(a.z - z) < 0.10) { hit = &a; break; }
        if (hit) hit->wheels.push_back(i);
        else axles.push_back({z, {i}});
    }
    Real zMid = 0.0;
    {
        Real lo = 1e30, hi = -1e30;
        for (const VehicleWheel& w : cfg.wheels) { lo = std::min(lo, w.position.z); hi = std::max(hi, w.position.z); }
        if (!cfg.wheels.empty()) zMid = 0.5 * (lo + hi);
    }
    std::vector<char> diffFront;
    bool anyFront = false, anyRear = false;
    for (const Axle& a : axles) {
        int left = a.wheels.front(), right = a.wheels.front();
        for (int i : a.wheels) {
            if (cfg.wheels[i].position.x > cfg.wheels[left].position.x) left = i;
            if (cfg.wheels[i].position.x < cfg.wheels[right].position.x) right = i;
        }
        JPH::VehicleDifferentialSettings d;
        d.mLeftWheel = left;
        d.mRightWheel = right == left ? -1 : right;
        d.mLimitedSlipRatio = static_cast<float>(cfg.axleLimitedSlip);
        controller->mDifferentials.push_back(d);
        const bool front = a.z > zMid;
        diffFront.push_back(front ? 1 : 0);
        (front ? anyFront : anyRear) = true;
    }
    float share = cfg.frontDriveShare >= 0.0 ? static_cast<float>(std::clamp(cfg.frontDriveShare, Real(0), Real(1)))
                                             : (anyFront && anyRear ? 0.5f : (anyFront ? 1.0f : 0.0f));
    // THE CENTRE COUPLING. Jolt's between-differentials limited slip sends ALL torque to the slower axle once
    // the other spins past the ratio -- at its default 1.4 a "rear-drive" car handed its torque to the fronts
    // the moment the rears slipped, and drove as a 4x4 (measured: RWD, FWD and 4WD climbed identically). A
    // one-axle split therefore runs it OPEN (FLT_MAX), as a part-time 4x4's transfer case in 2WD does.
    const bool oneAxle = share <= 0.001f || share >= 0.999f;
    controller->mDifferentialLimitedSlipRatio =
        oneAxle || cfg.centerLimitedSlip >= 1e29 ? FLT_MAX : static_cast<float>(cfg.centerLimitedSlip);
    // GEARBOX (VehicleConfig): quicker shifts, optional ratios
    controller->mTransmission.mSwitchTime = static_cast<float>(cfg.shiftTime);
    controller->mTransmission.mClutchReleaseTime = static_cast<float>(cfg.clutchReleaseTime);
    controller->mTransmission.mSwitchLatency = static_cast<float>(cfg.shiftLatency);
    if (!cfg.gearRatios.empty()) {
        controller->mTransmission.mGearRatios.clear();
        for (Real g : cfg.gearRatios) controller->mTransmission.mGearRatios.push_back(static_cast<float>(g));
    }
    auto applyShare = [](JPH::Array<JPH::VehicleDifferentialSettings>& diffs, const std::vector<char>& isFront, float frontShare) {
        int nf = 0, nr = 0;
        for (char f : isFront) (f ? nf : nr) += 1;
        float fs = frontShare;
        if (nf == 0) fs = 0.0f;
        if (nr == 0) fs = 1.0f;
        for (std::size_t i = 0; i < diffs.size(); ++i)
            diffs[i].mEngineTorqueRatio = isFront[i] ? (nf ? fs / nf : 0.0f) : (nr ? (1.0f - fs) / nr : 0.0f);
    };
    applyShare(controller->mDifferentials, diffFront, share);
    vs.mController = controller;

    JPH::VehicleConstraint* constraint = new JPH::VehicleConstraint(*chassis, vs);
    // Cast the wheel's actual CYLINDER, not a single ray. A ray only samples
    // straight down from the hub, so a kerb's vertical face was invisible to
    // the suspension and the chassis just rammed it — the "car stops dead at a
    // kerb" bug. The cylinder cast sees the step's edge, rides the contact up,
    // and the car mounts kerb-height steps; walls still read as walls because
    // their contact normal is horizontal (and the chassis box still collides).
    constraint->SetVehicleCollisionTester(
        new JPH::VehicleCollisionTesterCastCylinder(Layers::MOVING));
    impl->physicsSystem.AddConstraint(constraint);
    impl->physicsSystem.AddStepListener(constraint);

    Impl::Vehicle v;
    v.constraint = constraint;
    v.body = chassis->GetID();
    v.wheels = static_cast<int>(cfg.wheels.size());
    v.maxSteerRad = JPH::DegreesToRadians(static_cast<float>(cfg.maxSteerDegrees));
    v.yawAssist = static_cast<float>(std::max(Real(0), cfg.yawAssist));
    v.diffFront = diffFront;
    v.frontShare = share;
    v.centerLimitedSlip = cfg.centerLimitedSlip >= 1e29 ? FLT_MAX : static_cast<float>(cfg.centerLimitedSlip);
    v.dragArea = static_cast<float>(std::max(Real(0), cfg.dragArea));
    v.tractionSplit = cfg.tractionSplit;
    v.appliedShare = share;
    // The assist's yaw-rate cap: the grip on a 0.85 road (the city's), with
    // 10 % in hand so it only ever trims a slide, never a cornering car.
    v.gripAccel = static_cast<float>(1.1 * cfg.lateralGrip * std::sqrt(0.85) * 9.81);
    if (!cfg.wheels.empty()) {
        Real zMin = cfg.wheels.front().position.z, zMax = zMin;
        for (const VehicleWheel& w : cfg.wheels) {
            zMin = std::min(zMin, w.position.z);
            zMax = std::max(zMax, w.position.z);
        }
        v.wheelbase = static_cast<float>(std::max(Real(1.0), zMax - zMin));
    }
    impl->vehicles.push_back(std::move(v));
    return static_cast<VehicleId>(impl->vehicles.size() - 1);
}

void PhysicsWorld::removeVehicle(VehicleId id) {
    if (!impl || id >= impl->vehicles.size()) return;
    Impl::Vehicle& v = impl->vehicles[id];
    if (v.constraint) {
        impl->physicsSystem.RemoveStepListener(v.constraint.GetPtr());
        impl->physicsSystem.RemoveConstraint(v.constraint.GetPtr());
        v.constraint = nullptr;
    }
    if (!v.body.IsInvalid()) {
        impl->bodies().RemoveBody(v.body);
        impl->bodies().DestroyBody(v.body);
        v.body = JPH::BodyID();
    }
}

void PhysicsWorld::setVehicleInput(VehicleId id, Real forward, Real right,
                                   Real brake, Real handBrake) {
    if (!impl || id >= impl->vehicles.size()) return;
    JPH::VehicleConstraint* c = impl->vehicles[id].constraint.GetPtr();
    if (!c) return;
    auto* wc = static_cast<JPH::WheeledVehicleController*>(c->GetController());
    wc->SetDriverInput(static_cast<float>(forward), static_cast<float>(right),
                       static_cast<float>(brake), static_cast<float>(handBrake));
    impl->vehicles[id].steer = static_cast<float>(std::clamp(right, Real(-1), Real(1)));
    // Throttle/steer is meaningless on a sleeping body — wake it.
    if (forward != 0.0 || right != 0.0 || brake != 0.0 || handBrake != 0.0)
        impl->bodies().ActivateBody(impl->vehicles[id].body);
}

void PhysicsWorld::resetVehicleUpright(VehicleId id) {
    if (!impl || id >= impl->vehicles.size()) return;
    JPH::BodyID b = impl->vehicles[id].body;
    if (b.IsInvalid()) return;
    JPH::BodyInterface& bi = impl->bodies();
    // Keep the heading (yaw about world Y), drop the pitch/roll that flipped it.
    JPH::Vec3 fwd = bi.GetRotation(b).RotateAxisZ();      // local +Z (forward) in world
    float yaw = std::atan2(fwd.GetX(), fwd.GetZ());
    JPH::Quat upright = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), yaw);
    JPH::RVec3 pos = bi.GetPosition(b);
    bi.SetPositionAndRotation(b, pos + JPH::RVec3(0, 1.5f, 0), upright,
                              JPH::EActivation::Activate);
    bi.SetLinearVelocity(b, JPH::Vec3::sZero());
    bi.SetAngularVelocity(b, JPH::Vec3::sZero());
}

Vec3 PhysicsWorld::vehiclePosition(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return Vec3();
    return fromJolt(impl->bodies().GetPosition(impl->vehicles[id].body));
}

Quat PhysicsWorld::vehicleOrientation(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return Quat();
    return fromJolt(impl->bodies().GetRotation(impl->vehicles[id].body));
}

Vec3 PhysicsWorld::vehicleVelocity(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return Vec3();
    return fromJolt(impl->bodies().GetLinearVelocity(impl->vehicles[id].body));
}

int PhysicsWorld::vehicleWheelCount(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return 0;
    return impl->vehicles[id].wheels;
}

Mat4 PhysicsWorld::wheelTransform(VehicleId id, int wheel) const {
    if (!impl || id >= impl->vehicles.size()) return Mat4::identity();
    const JPH::VehicleConstraint* c = impl->vehicles[id].constraint.GetPtr();
    if (!c || wheel < 0 || wheel >= impl->vehicles[id].wheels) return Mat4::identity();
    // The wheel mesh is a cylinder spun about its local X (the axle); pass that as
    // the model's right, Y as up. Tune to the actual wheel mesh axis on a build.
    JPH::Mat44 m = c->GetWheelWorldTransform(static_cast<JPH::uint>(wheel),
                                             JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY());
    return fromJolt(m);
}

PhysicsBodyId PhysicsWorld::vehicleBody(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return INVALID_PHYSICS_BODY;
    return impl->vehicles[id].body.GetIndexAndSequenceNumber();
}

void PhysicsWorld::optimizeBroadPhase() {
    if (impl) impl->physicsSystem.OptimizeBroadPhase();
}

// The yaw assist (VehicleConfig::yawAssist): damp the chassis' yaw rate in
// excess of the steered one. Jolt turns a wheel by -right * lock about the
// up axis, so +steer is a NEGATIVE yaw about up (the lab's sign probe: "+steer
// turns toward -x" with forward +z); the Ackermann target is v tan(delta) / L,
// capped at the yaw rate a ~0.95 g road allows at this speed.
void PhysicsWorld::applyYawAssist(std::size_t index) {
    Impl::Vehicle& v = impl->vehicles[index];
    if (v.yawAssist <= 0.0f || !v.constraint) return;
    bool grounded = false;
    for (const JPH::Wheel* w : v.constraint->GetWheels())
        if (w->HasContact()) { grounded = true; break; }
    if (!grounded) return;
    JPH::BodyInterface& bi = impl->bodies();
    const JPH::Quat q = bi.GetRotation(v.body);
    const JPH::Vec3 up = q * JPH::Vec3(0, 1, 0);
    const JPH::Vec3 fwd = q * JPH::Vec3(0, 0, 1);
    const float speed = bi.GetLinearVelocity(v.body).Dot(fwd);
    if (std::fabs(speed) < 3.0f) return;             // parking: leave it alone
    const float yawRate = bi.GetAngularVelocity(v.body).Dot(up);
    const float delta = -v.steer * v.maxSteerRad;
    float target = speed * std::tan(delta) / v.wheelbase;
    const float gripLimit = v.gripAccel / std::fabs(speed);
    target = std::clamp(target, -gripLimit, gripLimit);
    float excess = yawRate - target;
    // Lagging the steered rate on the same side is the driver's business
    // (turn-in, understeer): the assist never adds yaw.
    if (target != 0.0f && yawRate * target >= 0.0f && std::fabs(yawRate) < std::fabs(target))
        excess = 0.0f;
    if (std::fabs(excess) < 0.02f) return;           // a dead band: noise is not a spin
    const JPH::Mat44 invI = bi.GetInverseInertia(v.body);
    const float invIup = (invI * up).Dot(up);
    if (invIup <= 1e-9f) return;
    bi.AddTorque(v.body, up * (-v.yawAssist * excess / invIup));
}

namespace {
// Write a front/rear torque split onto the differentials (each axle's share divided among its differentials).
float writeDriveShare(JPH::WheeledVehicleController* wc, const std::vector<char>& diffFront, float share) {
    JPH::Array<JPH::VehicleDifferentialSettings>& diffs = wc->GetDifferentials();
    int nf = 0, nr = 0;
    for (char f : diffFront) (f ? nf : nr) += 1;
    float fs = std::clamp(share, 0.0f, 1.0f);
    if (nf == 0) fs = 0.0f;
    if (nr == 0) fs = 1.0f;
    for (std::size_t i = 0; i < diffs.size() && i < diffFront.size(); ++i)
        diffs[i].mEngineTorqueRatio = diffFront[i] ? (nf ? fs / nf : 0.0f) : (nr ? (1.0f - fs) / nr : 0.0f);
    return fs;
}
}  // namespace

void PhysicsWorld::setVehicleFrontDriveShare(VehicleId id, Real share) {
    if (!impl || id >= impl->vehicles.size()) return;
    Impl::Vehicle& v = impl->vehicles[id];
    if (!v.constraint) return;
    auto* wc = static_cast<JPH::WheeledVehicleController*>(v.constraint->GetController());
    const float fs = writeDriveShare(wc, v.diffFront, static_cast<float>(share));
    // the centre coupling: open with one axle driven, limited-slip in 4WD (see addVehicle)
    wc->SetDifferentialLimitedSlipRatio(fs <= 0.001f || fs >= 0.999f ? FLT_MAX : v.centerLimitedSlip);
    v.frontShare = fs;
    v.appliedShare = fs;
}

// THE TRACTION SPLIT (VehicleConfig::tractionSplit): in 4WD, an axle with no tyre touching hands its torque to
// the axle that has one; both down (or both up), the driver's split.
void PhysicsWorld::applyTractionSplit(std::size_t index) {
    Impl::Vehicle& v = impl->vehicles[index];
    if (!v.tractionSplit || !v.constraint) return;
    float want = v.frontShare;
    if (v.frontShare > 0.001f && v.frontShare < 0.999f) {
        auto* wc = static_cast<JPH::WheeledVehicleController*>(v.constraint->GetController());
        const JPH::Array<JPH::VehicleDifferentialSettings>& diffs = wc->GetDifferentials();
        bool frontDown = false, rearDown = false;
        for (std::size_t i = 0; i < diffs.size() && i < v.diffFront.size(); ++i)
            for (int wi : {diffs[i].mLeftWheel, diffs[i].mRightWheel})
                if (wi >= 0 && v.constraint->GetWheel(static_cast<JPH::uint>(wi))->HasContact())
                    (v.diffFront[i] ? frontDown : rearDown) = true;
        if (frontDown && !rearDown) want = 1.0f;
        else if (rearDown && !frontDown) want = 0.0f;
    }
    if (want != v.appliedShare) {
        writeDriveShare(static_cast<JPH::WheeledVehicleController*>(v.constraint->GetController()), v.diffFront, want);
        v.appliedShare = want;
    }
}

Real PhysicsWorld::vehicleFrontDriveShare(VehicleId id) const {
    if (!impl || id >= impl->vehicles.size()) return 0.0;
    return impl->vehicles[id].frontShare;
}

PhysicsWorld::VehicleTelemetry PhysicsWorld::vehicleTelemetry(VehicleId id) const {
    VehicleTelemetry t;
    if (!impl || id >= impl->vehicles.size()) return t;
    const Impl::Vehicle& v = impl->vehicles[id];
    if (!v.constraint) return t;
    const JPH::BodyInterface& bi = impl->bodies();
    const JPH::Vec3 fwd = bi.GetRotation(v.body) * JPH::Vec3(0, 0, 1);
    t.speed = bi.GetLinearVelocity(v.body).Dot(fwd);
    const auto* wc = static_cast<const JPH::WheeledVehicleController*>(v.constraint->GetController());
    t.rpm = wc->GetEngine().GetCurrentRPM();
    t.gear = wc->GetTransmission().GetCurrentGear();
    for (int i = 0; i < v.wheels; ++i) {
        const JPH::Wheel* w = v.constraint->GetWheel(static_cast<JPH::uint>(i));
        t.wheelSpin.push_back(w->GetAngularVelocity());
        t.wheelContact.push_back(w->HasContact() ? 1 : 0);
        t.wheelSlip.push_back(static_cast<const JPH::WheelWV*>(w)->mLongitudinalSlip);
    }
    return t;
}

// AERO DRAG (VehicleConfig::dragArea): 0.5 rho CdA |v| v against the chassis' motion, each step.
void PhysicsWorld::applyAeroDrag(std::size_t index) {
    Impl::Vehicle& v = impl->vehicles[index];
    if (v.dragArea <= 0.0f || !v.constraint) return;
    JPH::BodyInterface& bi = impl->bodies();
    const JPH::Vec3 vel = bi.GetLinearVelocity(v.body);
    const float speed = vel.Length();
    if (speed < 0.1f) return;
    bi.AddForce(v.body, vel * (-0.5f * 1.225f * v.dragArea * speed));
}

void PhysicsWorld::update(Real deltaTime, int collisionSteps) {
    if (impl) {
        for (std::size_t i = 0; i < impl->vehicles.size(); ++i) { applyYawAssist(i); applyAeroDrag(i); applyTractionSplit(i); }
        impl->physicsSystem.Update(static_cast<float>(deltaTime), collisionSteps,
                                   &impl->tempAllocator, impl->jobSystem.get());
    }
}

}  // namespace engine

