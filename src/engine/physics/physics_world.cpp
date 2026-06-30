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
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
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
inline Mat4 fromJolt(const JPH::Mat44& m) {
    Mat4 r;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            r.m[row][col] = static_cast<Real>(m(static_cast<JPH::uint>(row),
                                                static_cast<JPH::uint>(col)));
    return r;
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

    // Wheeled vehicles (ADR-0058). Each owns a VehicleConstraint (also a step
    // listener) over a dynamic chassis body. Slots are never compacted so
    // VehicleIds stay valid; removeVehicle releases the ref and invalidates the id.
    struct Vehicle {
        JPH::Ref<JPH::VehicleConstraint> constraint;
        JPH::BodyID body;
        int wheels = 0;
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

void PhysicsWorld::moveKinematic(PhysicsBodyId id, const Vec3& position,
                                 const Quat& orientation, Real dt) {
    if (!impl || id == INVALID_PHYSICS_BODY || dt <= 0) return;
    impl->bodies().MoveKinematic(JPH::BodyID(id), toJoltR(position),
                                 toJolt(orientation), static_cast<float>(dt));
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

// --- Wheeled vehicle (ADR-0058) --------------------------------------------
// UNVERIFIED: written against the documented Jolt v5.5.0 vehicle API; the Jolt
// submodule can't be fetched in this environment, so this has NOT been compiled.
// Likely tuning/rename touch-ups on a real build (member names on WheelSettingsWV
// / VehicleEngineSettings, the collision-tester ctor signature).

PhysicsWorld::VehicleId PhysicsWorld::addVehicle(const VehicleConfig& cfg,
                                                 const Vec3& position,
                                                 const Quat& orientation) {
    if (!impl) return INVALID_VEHICLE;

    // Chassis: a dynamic box, mass overridden, sleeping disabled so the controller
    // keeps stepping. (Lowering the centre of mass via an OffsetCenterOfMassShape
    // is the usual anti-roll tweak — left as a tuning follow-up.)
    JPH::BoxShapeSettings shapeSettings(toJolt(cfg.chassisHalfExtent));
    JPH::ShapeSettings::ShapeResult shapeRes = shapeSettings.Create();
    if (shapeRes.HasError()) return INVALID_VEHICLE;
    // Lower the centre of mass below the chassis centre so the car resists rolling
    // in corners (the classic anti-tip tweak).
    JPH::OffsetCenterOfMassShapeSettings comSettings(
        JPH::Vec3(0, static_cast<float>(cfg.comOffsetY), 0), shapeRes.Get());
    JPH::ShapeSettings::ShapeResult bodyShapeRes = comSettings.Create();
    if (bodyShapeRes.HasError()) return INVALID_VEHICLE;

    JPH::BodyCreationSettings bodySettings(bodyShapeRes.Get(), toJoltR(position),
                                           toJolt(orientation),
                                           JPH::EMotionType::Dynamic, Layers::MOVING);
    bodySettings.mOverrideMassProperties =
        JPH::EOverrideMassProperties::CalculateInertia;
    bodySettings.mMassPropertiesOverride.mMass = static_cast<float>(cfg.mass);
    bodySettings.mFriction = static_cast<float>(cfg.friction);
    bodySettings.mAllowSleeping = false;
    JPH::Body* chassis = impl->bodies().CreateBody(bodySettings);
    if (!chassis) return INVALID_VEHICLE;
    impl->bodies().AddBody(chassis->GetID(), JPH::EActivation::Activate);

    JPH::VehicleConstraintSettings vs;
    vs.mUp = JPH::Vec3(0, 1, 0);
    vs.mForward = JPH::Vec3(0, 0, 1);
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
        ws->mMaxBrakeTorque = static_cast<float>(cfg.brakeTorque);
        ws->mMaxHandBrakeTorque =
            w.handBrake ? static_cast<float>(cfg.handBrakeTorque) : 0.0f;
        vs.mWheels.push_back(ws);
    }

    JPH::WheeledVehicleControllerSettings* controller =
        new JPH::WheeledVehicleControllerSettings();
    controller->mEngine.mMaxTorque = static_cast<float>(cfg.engineTorque);
    controller->mEngine.mMaxRPM = static_cast<float>(cfg.maxRPM);

    // One differential per driven wheel pair (in declaration order), torque split
    // evenly across them. A standard 4-wheeler with two driven fronts/rears gets
    // one differential; an all-wheel-drive layout gets two.
    std::vector<int> driven;
    for (int i = 0; i < static_cast<int>(cfg.wheels.size()); ++i)
        if (cfg.wheels[i].driven) driven.push_back(i);
    for (size_t i = 0; i + 1 < driven.size(); i += 2) {
        JPH::VehicleDifferentialSettings d;
        d.mLeftWheel = driven[i];
        d.mRightWheel = driven[i + 1];
        controller->mDifferentials.push_back(d);
    }
    if (!controller->mDifferentials.empty()) {
        float ratio = 1.0f / static_cast<float>(controller->mDifferentials.size());
        for (JPH::VehicleDifferentialSettings& d : controller->mDifferentials)
            d.mEngineTorqueRatio = ratio;
    }
    vs.mController = controller;

    JPH::VehicleConstraint* constraint = new JPH::VehicleConstraint(*chassis, vs);
    // Raycast wheels against the moving layer's ground. (CastCylinder is the
    // higher-fidelity alternative if wheels clip kerbs.)
    constraint->SetVehicleCollisionTester(
        new JPH::VehicleCollisionTesterRay(Layers::MOVING));
    impl->physicsSystem.AddConstraint(constraint);
    impl->physicsSystem.AddStepListener(constraint);

    Impl::Vehicle v;
    v.constraint = constraint;
    v.body = chassis->GetID();
    v.wheels = static_cast<int>(cfg.wheels.size());
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

void PhysicsWorld::update(Real deltaTime, int collisionSteps) {
    if (impl) {
        impl->physicsSystem.Update(static_cast<float>(deltaTime), collisionSteps,
                                   &impl->tempAllocator, impl->jobSystem.get());
    }
}

}  // namespace engine

