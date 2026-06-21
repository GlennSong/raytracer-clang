#include "physics_world.h"

#include "jolt_job_adapter.h"
#include "../../job_system.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>

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

constexpr JPH::uint MAX_BODIES = 10240;
constexpr JPH::uint MAX_BODY_PAIRS = 10240;
constexpr JPH::uint MAX_CONTACT_CONSTRAINTS = 10240;

}  // namespace

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
    JPH::PhysicsSystem physicsSystem;

    // Virtual characters live outside the body simulation, so we own them here.
    // The handle is the index; entries are never compacted (removeCharacter just
    // releases the ref) so existing CharacterIds stay valid.
    struct Character {
        JPH::Ref<JPH::CharacterVirtual> controller;
        float stepHeight = 0.4f;
    };
    std::vector<Character> characters;

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
    if (jobSystem) {
        impl->jobSystem = std::make_unique<JoltJobAdapter>(
            *jobSystem, JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers);
    } else {
        impl->jobSystem =
            std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    }
    impl->physicsSystem.Init(MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS,
                             impl->broadPhaseLayers, impl->objectVsBroadPhase,
                             impl->objectVsObject);
    return true;
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

PhysicsBodyId PhysicsWorld::addMesh(const std::vector<Vec3>& vertices,
                                    const std::vector<uint32_t>& indices,
                                    const Vec3& position, Real friction) {
    if (!impl || vertices.empty() || indices.size() < 3) return INVALID_PHYSICS_BODY;
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
    if (result.HasError()) return INVALID_PHYSICS_BODY;
    return createBody(impl->bodies(), result.Get(), position, Quat::identity(),
                      BodyMotion::Static, 0.0, friction);
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
    JPH::Vec3 newVel;
    if (ch->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround) {
        newVel = desired;
    } else {
        newVel = desired + up * current.Dot(up);
    }
    newVel += impl->physicsSystem.GetGravity() * static_cast<float>(dt);
    ch->SetLinearVelocity(newVel);

    JPH::CharacterVirtual::ExtendedUpdateSettings settings;
    settings.mWalkStairsStepUp = up * impl->characters[id].stepHeight;
    // Step down by at least the step-up height so descending a curb keeps the
    // character grounded instead of briefly going airborne each step.
    settings.mStickToFloorStepDown =
        -up * std::max(impl->characters[id].stepHeight, 0.5f);

    ch->ExtendedUpdate(static_cast<float>(dt), impl->physicsSystem.GetGravity(),
                       settings,
                       impl->physicsSystem.GetDefaultBroadPhaseLayerFilter(
                           Layers::MOVING),
                       impl->physicsSystem.GetDefaultLayerFilter(Layers::MOVING),
                       {}, {}, impl->tempAllocator);
}

Vec3 PhysicsWorld::characterPosition(CharacterId id) const {
    if (!impl || id >= impl->characters.size()) return Vec3();
    const JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr();
    return ch ? fromJolt(ch->GetPosition()) : Vec3();
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
    if (JPH::CharacterVirtual* ch = impl->characters[id].controller.GetPtr())
        ch->SetPosition(toJoltR(position));
}

void PhysicsWorld::optimizeBroadPhase() {
    if (impl) impl->physicsSystem.OptimizeBroadPhase();
}

void PhysicsWorld::update(Real deltaTime, int collisionSteps) {
    if (impl) {
        impl->physicsSystem.Update(static_cast<float>(deltaTime), collisionSteps,
                                   &impl->tempAllocator, impl->jobSystem.get());
    }
}

}  // namespace engine

