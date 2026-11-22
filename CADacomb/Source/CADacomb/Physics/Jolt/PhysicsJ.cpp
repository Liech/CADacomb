#include "CADacomb/Physics/Jolt/PhysicsJ.h"

#include "CADacomb/Physics/Jolt/BalloonSoftBodyBuilderJ.h"
#include "CADacomb/Physics/Jolt/InternalStrutsSoftBodyBuilderJ.h"
#include "CADacomb/Physics/Jolt/RigidBodyJ.h"
#include "CADacomb/Physics/Jolt/SVOShapeJ.h"
#include "CADacomb/Physics/Jolt/SoftBodyBuilderJ.h"
#include "CADacomb/Physics/Jolt/SoftBodyJ.h"
#include "CADacomb/Physics/Jolt/VolumetricSoftBodyBuilderJ.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/RegisterTypes.h>
#include <algorithm>
#include <cmath>

namespace CADacomb::Physics
{
    std::once_flag PhysicsJ::initOnceFlag;

    PhysicsJ::PhysicsJ()
    {
        ensureInitialized();

        const JPH::uint maxBodies             = 10240;
        const JPH::uint numBodyMutexes        = 0;
        const JPH::uint maxBodyPairs          = 65536;
        const JPH::uint maxContactConstraints = 10240;
        const JPH::uint tempAllocatorBytes    = 10 * 1024 * 1024;
        const JPH::uint maxJobs               = 2048;

        tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(tempAllocatorBytes);
        jobSystem     = std::make_unique<JPH::JobSystemSingleThreaded>(maxJobs);
        physicsSystem = std::make_shared<JPH::PhysicsSystem>();
        physicsSystem->Init(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints, broadPhaseLayerInterface, objectVsBroadPhaseLayerFilter, objectLayerPairFilter);
        physicsSystem->SetGravity(JPH::Vec3(0.0f, -9.81f, 0.0f));
    }

    PhysicsJ::~PhysicsJ() {}

    void PhysicsJ::ensureInitialized()
    {
        std::call_once(initOnceFlag,
                       []()
                       {
                           JPH::RegisterDefaultAllocator();
                           if (!JPH::Factory::sInstance)
                           {
                               JPH::Factory::sInstance = new JPH::Factory();
                           }
                           JPH::RegisterTypes();
                           SVOShapeJ::registerCollisionDispatch();
                       });
    }

    std::shared_ptr<RigidBody> PhysicsJ::createRigidBody(const Triangulation& triangulation, const glm::dmat4& transformation, bool isStatic)
    {
        glm::dvec3 position;
        glm::dquat rotation;
        glm::dmat4 localScale;
        RigidBodyJ::decomposeTransformation(transformation, position, rotation, localScale);

        JPH::RefConst<JPH::Shape> shape = buildShape(triangulation, localScale, isStatic);

        JPH::EMotionType motionType  = isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
        JPH::ObjectLayer objectLayer = isStatic ? ObjectLayerPairFilterJ::nonMovingLayer : ObjectLayerPairFilterJ::movingLayer;

        JPH::BodyCreationSettings settings(shape,
                                           JPH::RVec3(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)),
                                           JPH::Quat(static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z), static_cast<float>(rotation.w)),
                                           motionType,
                                           objectLayer);
        settings.mAllowSleeping = false;

        JPH::BodyInterface& bodyInterface = physicsSystem->GetBodyInterface();
        JPH::BodyID         bodyId        = bodyInterface.CreateAndAddBody(settings, isStatic ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);

        return std::make_shared<RigidBodyJ>(physicsSystem, bodyId, localScale, isStatic);
    }

    std::shared_ptr<RigidBody> PhysicsJ::createRigidBody(const SVO<bool>& svo, const AABB3& localBounds, const glm::dmat4& transformation, bool isStatic)
    {
        glm::dvec3 position;
        glm::dquat rotation;
        glm::dmat4 localScale;
        RigidBodyJ::decomposeTransformation(transformation, position, rotation, localScale);

        AABB3               scaledBounds = localBounds.transformed(localScale);
        JPH::Ref<SVOShapeJ> svoShape     = new SVOShapeJ(svo, scaledBounds);

        JPH::EMotionType motionType  = isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
        JPH::ObjectLayer objectLayer = isStatic ? ObjectLayerPairFilterJ::nonMovingLayer : ObjectLayerPairFilterJ::movingLayer;

        JPH::BodyCreationSettings settings(svoShape,
                                           JPH::RVec3(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)),
                                           JPH::Quat(static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z), static_cast<float>(rotation.w)),
                                           motionType,
                                           objectLayer);
        settings.mAllowSleeping = false;

        JPH::BodyInterface& bodyInterface = physicsSystem->GetBodyInterface();
        JPH::BodyID         bodyId        = bodyInterface.CreateAndAddBody(settings, isStatic ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);

        return std::make_shared<RigidBodyJ>(physicsSystem, bodyId, localScale, isStatic, svoShape);
    }

    std::shared_ptr<SoftBody> PhysicsJ::createSoftBody(const Triangulation& triangulation,
                                                       const glm::dmat4&    transformation,
                                                       SoftBodyMode         mode,
                                                       double               mass,
                                                       double               pressure,
                                                       double               edgeCompliance,
                                                       double               bendCompliance,
                                                       double               volumeCompliance,
                                                       int                  internalResolution,
                                                       double               damping,
                                                       double               friction,
                                                       double               restitution,
                                                       int                  solverIterations)
    {
        glm::dvec3 position;
        glm::dquat rotation;
        glm::dmat4 localScale;
        RigidBodyJ::decomposeTransformation(transformation, position, rotation, localScale);

        std::unique_ptr<SoftBodyBuilderJ> builder;
        if (mode == SoftBodyMode::InternalStruts)
        {
            builder = std::make_unique<InternalStrutsSoftBodyBuilderJ>();
        }
        else if (mode == SoftBodyMode::Volumetric)
        {
            builder = std::make_unique<VolumetricSoftBodyBuilderJ>();
        }
        else
        {
            builder = std::make_unique<BalloonSoftBodyBuilderJ>();
        }

        std::vector<size_t>                   originalToWelded;
        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = builder->build(triangulation, localScale, mass, edgeCompliance, bendCompliance, volumeCompliance, internalResolution, originalToWelded);

        JPH::SoftBodyCreationSettings creationSettings(sharedSettings,
                                                       JPH::RVec3(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)),
                                                       JPH::Quat(static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z), static_cast<float>(rotation.w)),
                                                       ObjectLayerPairFilterJ::movingLayer);
        creationSettings.mNumIterations        = static_cast<JPH::uint32>(std::max(1, solverIterations));
        creationSettings.mLinearDamping        = static_cast<float>(std::max(0.0, damping));
        creationSettings.mFriction             = static_cast<float>(std::max(0.0, friction));
        creationSettings.mRestitution          = static_cast<float>(std::clamp(restitution, 0.0, 1.0));
        creationSettings.mPressure             = builder->resolvePressure(pressure);
        creationSettings.mUpdatePosition       = true;
        creationSettings.mMakeRotationIdentity = false;
        creationSettings.mAllowSleeping        = false;

        JPH::BodyInterface& bodyInterface = physicsSystem->GetBodyInterface();
        JPH::BodyID         bodyId        = bodyInterface.CreateAndAddSoftBody(creationSettings, JPH::EActivation::Activate);

        return std::make_shared<SoftBodyJ>(physicsSystem, bodyId, localScale, originalToWelded);
    }

    void PhysicsJ::setGravity(const glm::dvec3& gravity)
    {
        physicsSystem->SetGravity(JPH::Vec3(static_cast<float>(gravity.x), static_cast<float>(gravity.y), static_cast<float>(gravity.z)));
    }

    glm::dvec3 PhysicsJ::getGravity() const
    {
        JPH::Vec3 g = physicsSystem->GetGravity();
        return glm::dvec3(static_cast<double>(g.GetX()), static_cast<double>(g.GetY()), static_cast<double>(g.GetZ()));
    }

    void PhysicsJ::step(double deltaTime)
    {
        if (deltaTime <= 0.0)
        {
            return;
        }
        const float  fixedStep = 1.0f / 60.0f;
        const double ratio     = deltaTime / static_cast<double>(fixedStep);
        const int    numSteps  = static_cast<int>(std::round(ratio));
        if (numSteps >= 1 && std::abs(ratio - static_cast<double>(numSteps)) < 1e-3)
        {
            for (int i = 0; i < numSteps; i++)
            {
                physicsSystem->Update(fixedStep, 1, tempAllocator.get(), jobSystem.get());
            }
            return;
        }

        int collisionSteps = std::max(1, static_cast<int>(std::ceil(ratio)));
        physicsSystem->Update(static_cast<float>(deltaTime), collisionSteps, tempAllocator.get(), jobSystem.get());
    }

    JPH::RefConst<JPH::Shape> PhysicsJ::buildShape(const Triangulation& triangulation, const glm::dmat4& localScale, bool isStatic) const
    {
        std::vector<glm::vec3> scaledVertices;
        scaledVertices.reserve(triangulation.vertices.size());
        glm::vec3 minCorner(1e30f);
        glm::vec3 maxCorner(-1e30f);

        for (const auto& v : triangulation.vertices)
        {
            glm::dvec3 sv = glm::dvec3(localScale * glm::dvec4(v, 1.0));
            glm::vec3  fv(static_cast<float>(sv.x), static_cast<float>(sv.y), static_cast<float>(sv.z));
            scaledVertices.push_back(fv);
            minCorner = glm::min(minCorner, fv);
            maxCorner = glm::max(maxCorner, fv);
        }

        if (isStatic && triangulation.indices.size() >= 3)
        {
            JPH::TriangleList triangles;
            triangles.reserve(triangulation.indices.size() / 3);
            for (size_t i = 0; i + 2 < triangulation.indices.size(); i += 3)
            {
                size_t i0 = triangulation.indices[i];
                size_t i1 = triangulation.indices[i + 1];
                size_t i2 = triangulation.indices[i + 2];
                if (i0 < scaledVertices.size() && i1 < scaledVertices.size() && i2 < scaledVertices.size())
                {
                    const glm::vec3& v0 = scaledVertices[i0];
                    const glm::vec3& v1 = scaledVertices[i1];
                    const glm::vec3& v2 = scaledVertices[i2];
                    triangles.push_back(JPH::Triangle(JPH::Float3(v0.x, v0.y, v0.z), JPH::Float3(v1.x, v1.y, v1.z), JPH::Float3(v2.x, v2.y, v2.z)));
                }
            }
            if (!triangles.empty())
            {
                JPH::MeshShapeSettings          meshSettings(triangles);
                JPH::ShapeSettings::ShapeResult meshResult = meshSettings.Create();
                if (meshResult.IsValid())
                {
                    return meshResult.Get();
                }
            }
        }

        if (!scaledVertices.empty())
        {
            JPH::Array<JPH::Vec3> hullPoints;
            hullPoints.reserve(scaledVertices.size());
            for (const auto& fv : scaledVertices)
            {
                hullPoints.push_back(JPH::Vec3(fv.x, fv.y, fv.z));
            }
            JPH::ConvexHullShapeSettings    hullSettings(hullPoints);
            JPH::ShapeSettings::ShapeResult hullResult = hullSettings.Create();
            if (hullResult.IsValid())
            {
                return hullResult.Get();
            }

            glm::vec3                 halfExtents = glm::max((maxCorner - minCorner) * 0.5f, glm::vec3(0.05f));
            glm::vec3                 center      = (minCorner + maxCorner) * 0.5f;
            JPH::RefConst<JPH::Shape> box         = new JPH::BoxShape(JPH::Vec3(halfExtents.x, halfExtents.y, halfExtents.z));
            if (glm::length(center) > 1e-5f)
            {
                JPH::RotatedTranslatedShapeSettings rtSettings(JPH::Vec3(center.x, center.y, center.z), JPH::Quat::sIdentity(), box);
                JPH::ShapeSettings::ShapeResult     rtResult = rtSettings.Create();
                if (rtResult.IsValid())
                {
                    return rtResult.Get();
                }
            }
            return box;
        }

        return new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f));
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"

#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/matrix_transform.hpp>

TEST_CASE("PhysicsJ/DynamicBodyFallsAndCollidesWithStaticFloor")
{
    std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();

    CADacomb::Triangulation floorMesh = CADacomb::BaseShapeGenerator::box(20.0f, 1.0f, 20.0f);
    glm::dmat4              floorTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, -1.0, 0.0));
    auto                    floorBody = physics->createRigidBody(floorMesh, floorTf, true);

    CADacomb::Triangulation cubeMesh = CADacomb::BaseShapeGenerator::cube();
    glm::dmat4              cubeTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, 3.0, 0.0));
    auto                    cubeBody = physics->createRigidBody(cubeMesh, cubeTf, false);

    REQUIRE(floorBody->isStatic());
    REQUIRE_FALSE(cubeBody->isStatic());

    for (int i = 0; i < 60; i++)
    {
        physics->step(1.0 / 60.0);
    }

    glm::dmat4 floorAfter = floorBody->getTransformation();
    glm::dmat4 cubeAfter  = cubeBody->getTransformation();

    CHECK(floorAfter == floorTf);
    CHECK(cubeAfter[3].y < 3.0);
    CHECK(cubeAfter[3].y > -0.5);
}

TEST_CASE("PhysicsJ/IncrementalStepsMatchSingleMergedStep")
{
    CADacomb::Triangulation floorMesh = CADacomb::BaseShapeGenerator::box(20.0f, 1.0f, 20.0f);
    glm::dmat4              floorTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, -1.0, 0.0));
    CADacomb::Triangulation cubeMesh  = CADacomb::BaseShapeGenerator::cube();
    glm::dmat4              cubeTf    = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, 3.0, 0.0));

    std::unique_ptr<CADacomb::Physics::Physics> incrementalPhysics = std::make_unique<CADacomb::Physics::PhysicsJ>();
    auto                                        incFloor           = incrementalPhysics->createRigidBody(floorMesh, floorTf, true);
    auto                                        incCube            = incrementalPhysics->createRigidBody(cubeMesh, cubeTf, false);
    for (int i = 0; i < 45; i++)
    {
        incrementalPhysics->step(1.0 / 60.0);
    }

    std::unique_ptr<CADacomb::Physics::Physics> mergedPhysics = std::make_unique<CADacomb::Physics::PhysicsJ>();
    auto                                        mrgFloor      = mergedPhysics->createRigidBody(floorMesh, floorTf, true);
    auto                                        mrgCube       = mergedPhysics->createRigidBody(cubeMesh, cubeTf, false);
    mergedPhysics->step(45.0 / 60.0);

    CHECK(incCube->getTransformation() == mrgCube->getTransformation());
}

TEST_CASE("PhysicsJ/SVOColliderAndDynamicSVOModification")
{
    std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();

    CADacomb::SVO<bool> floorSvo(16, false);
    for (int x = 0; x < 16; x++)
    {
        for (int z = 0; z < 16; z++)
        {
            floorSvo.setVoxel(glm::ivec3(x, 7, z), true);
        }
    }
    CADacomb::AABB3 floorBounds(glm::dvec3(-8.0, -8.0, -8.0), glm::dvec3(8.0, 8.0, 8.0));
    auto            floorBody = physics->createRigidBody(floorSvo, floorBounds, glm::dmat4(1.0), true);

    CADacomb::SVO<bool> fallingSvo(4, true);
    CADacomb::AABB3     fallingBounds(glm::dvec3(-0.5, -0.5, -0.5), glm::dvec3(0.5, 0.5, 0.5));
    glm::dmat4          fallingStart = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, 2.0, 0.0));
    auto                fallingBody  = physics->createRigidBody(fallingSvo, fallingBounds, fallingStart, false);

    physics->step(1.0);
    double restingY = fallingBody->getTransformation()[3].y;
    CHECK(restingY < 1.5);
    CHECK(restingY > -0.2);

    for (int x = 0; x < 16; x++)
    {
        for (int z = 0; z < 16; z++)
        {
            floorSvo.setVoxel(glm::ivec3(x, 7, z), false);
        }
    }
    floorBody->notifyGeometryChanged();

    physics->step(1.0);
    double fallenThroughY = fallingBody->getTransformation()[3].y;
    CHECK(fallenThroughY < -2.0);
}

TEST_CASE("PhysicsJ/SoftBodyDeformsAndSettlesOnStaticFloor")
{
    std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();

    CADacomb::Triangulation floorMesh = CADacomb::BaseShapeGenerator::box(20.0f, 1.0f, 20.0f);
    glm::dmat4              floorTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, -1.0, 0.0));
    auto                    floorBody = physics->createRigidBody(floorMesh, floorTf, true);

    CADacomb::Triangulation softMesh = CADacomb::BaseShapeGenerator::sphere(1.0f, 12, 12);
    glm::dmat4              softTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, 1.8, 0.0));
    auto                    softBody = physics->createSoftBody(softMesh, softTf, CADacomb::Physics::SoftBodyMode::Balloon, 1.0, 1500.0, 0.0001, 0.001, 0.0001, 4, 0.2, 0.3, 0.0, 10);

    std::vector<glm::dvec3> initialVerts = softBody->getDeformedVertices();
    REQUIRE(initialVerts.size() == softMesh.vertices.size());

    physics->step(1.0);

    glm::dmat4              afterTf    = softBody->getTransformation();
    std::vector<glm::dvec3> afterVerts = softBody->getDeformedVertices();
    REQUIRE(afterVerts.size() == softMesh.vertices.size());
    CHECK(afterTf[3].y < 1.8);
    CHECK(afterTf[3].y > -0.5);

    bool anyVertexDeformed = false;
    for (size_t i = 0; i < initialVerts.size(); i++)
    {
        if (glm::distance(initialVerts[i], afterVerts[i]) > 1e-4)
        {
            anyVertexDeformed = true;
            break;
        }
    }
    CHECK(anyVertexDeformed);
}

TEST_CASE("PhysicsJ/SoftBodyInternalStrutsAndVolumetricModes")
{
    std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();

    CADacomb::Triangulation floorMesh = CADacomb::BaseShapeGenerator::box(20.0f, 1.0f, 20.0f);
    glm::dmat4              floorTf   = glm::translate(glm::dmat4(1.0), glm::dvec3(0.0, -1.0, 0.0));
    auto                    floorBody = physics->createRigidBody(floorMesh, floorTf, true);

    CADacomb::Triangulation softMesh = CADacomb::BaseShapeGenerator::sphere(1.0f, 10, 10);
    glm::dmat4              strutsTf = glm::translate(glm::dmat4(1.0), glm::dvec3(-2.5, 1.8, 0.0));
    auto                    strutsSb = physics->createSoftBody(softMesh, strutsTf, CADacomb::Physics::SoftBodyMode::InternalStruts, 1.0, 0.0, 0.0001, 0.001, 0.0002, 4, 0.2, 0.4, 0.0, 12);

    glm::dmat4 volTf = glm::translate(glm::dmat4(1.0), glm::dvec3(2.5, 1.8, 0.0));
    auto       volSb = physics->createSoftBody(softMesh, volTf, CADacomb::Physics::SoftBodyMode::Volumetric, 1.0, 0.0, 0.0001, 0.001, 0.0002, 4, 0.2, 0.4, 0.0, 12);

    physics->step(1.0);

    CHECK(strutsSb->getDeformedVertices().size() == softMesh.vertices.size());
    CHECK(strutsSb->getTransformation()[3].y < 1.8);
    CHECK(strutsSb->getTransformation()[3].y > -0.5);

    CHECK(volSb->getDeformedVertices().size() == softMesh.vertices.size());
    CHECK(volSb->getTransformation()[3].y < 1.8);
    CHECK(volSb->getTransformation()[3].y > -0.5);
}
#endif
