#include "CADacomb/Physics/Jolt/RigidBodyJ.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

namespace CADacomb::Physics
{
    RigidBodyJ::RigidBodyJ(std::shared_ptr<JPH::PhysicsSystem> physicsSystemInput,
                           JPH::BodyID                         bodyIdInput,
                           const glm::dmat4&                   localScaleTransformInput,
                           bool                                isStaticInput,
                           JPH::Ref<SVOShapeJ>                 svoShapeInput)
      : physicsSystem(physicsSystemInput)
      , bodyId(bodyIdInput)
      , svoShape(svoShapeInput)
      , localScaleTransform(localScaleTransformInput)
      , staticBody(isStaticInput)
    {
    }

    RigidBodyJ::~RigidBodyJ()
    {
        auto system = physicsSystem.lock();
        if (system && !bodyId.IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = system->GetBodyInterface();
            if (bodyInterface.IsAdded(bodyId))
            {
                bodyInterface.RemoveBody(bodyId);
            }
            bodyInterface.DestroyBody(bodyId);
        }
    }

    bool RigidBodyJ::isStatic() const
    {
        return staticBody;
    }

    glm::dmat4 RigidBodyJ::getTransformation() const
    {
        auto system = physicsSystem.lock();
        if (!system || bodyId.IsInvalid())
        {
            return localScaleTransform;
        }

        JPH::RVec3 pos;
        JPH::Quat  rot;
        system->GetBodyInterface().GetPositionAndRotation(bodyId, pos, rot);

        glm::dvec3 position(static_cast<double>(pos.GetX()), static_cast<double>(pos.GetY()), static_cast<double>(pos.GetZ()));
        glm::dquat rotation(static_cast<double>(rot.GetW()), static_cast<double>(rot.GetX()), static_cast<double>(rot.GetY()), static_cast<double>(rot.GetZ()));
        glm::dmat4 rigidWorld = glm::translate(glm::dmat4(1.0), position) * glm::mat4_cast(rotation);
        return rigidWorld * localScaleTransform;
    }

    void RigidBodyJ::setTransformation(const glm::dmat4& transformation)
    {
        glm::dvec3 position;
        glm::dquat rotation;
        decomposeTransformation(transformation, position, rotation, localScaleTransform);

        auto system = physicsSystem.lock();
        if (system && !bodyId.IsInvalid())
        {
            system->GetBodyInterface().SetPositionAndRotation(bodyId,
                                                              JPH::RVec3(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)),
                                                              JPH::Quat(static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z), static_cast<float>(rotation.w)),
                                                              staticBody ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);
        }
    }

    void RigidBodyJ::notifyGeometryChanged()
    {
        auto system = physicsSystem.lock();
        if (!system || bodyId.IsInvalid())
        {
            return;
        }

        JPH::BodyInterface& bodyInterface = system->GetBodyInterface();
        if (svoShape != nullptr)
        {
            JPH::Vec3 previousCOM = svoShape->GetCenterOfMass();
            svoShape->refreshMassProperties();
            bodyInterface.NotifyShapeChanged(bodyId, previousCOM, !staticBody, staticBody ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);

            JPH::RMat44 comTransform = bodyInterface.GetCenterOfMassTransform(bodyId);
            JPH::AABox  worldBounds  = svoShape->GetWorldSpaceBounds(comTransform, JPH::Vec3::sReplicate(1.0f));
            worldBounds.ExpandBy(JPH::Vec3::sReplicate(0.5f));
            JPH::DefaultBroadPhaseLayerFilter broadPhaseFilter(system->GetObjectVsBroadPhaseLayerFilter(), 1);
            JPH::DefaultObjectLayerFilter     objectFilter(system->GetObjectLayerPairFilter(), 1);
            bodyInterface.ActivateBodiesInAABox(worldBounds, broadPhaseFilter, objectFilter);
        }
    }

    void RigidBodyJ::decomposeTransformation(const glm::dmat4& matrix, glm::dvec3& position, glm::dquat& rotation, glm::dmat4& localScale)
    {
        position = glm::dvec3(matrix[3]);

        glm::dvec3 col0(matrix[0]);
        glm::dvec3 col1(matrix[1]);
        double     len0 = glm::length(col0);
        double     len1 = glm::length(col1);

        glm::dvec3 r0 = len0 > 1e-12 ? (col0 / len0) : glm::dvec3(1.0, 0.0, 0.0);
        glm::dvec3 r1 = len1 > 1e-12 ? (col1 / len1) : glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 r2 = glm::cross(r0, r1);
        if (glm::length(r2) > 1e-12)
        {
            r2 = glm::normalize(r2);
            r1 = glm::normalize(glm::cross(r2, r0));
        }
        else
        {
            r0 = glm::dvec3(1.0, 0.0, 0.0);
            r1 = glm::dvec3(0.0, 1.0, 0.0);
            r2 = glm::dvec3(0.0, 0.0, 1.0);
        }

        glm::dmat3 rotMat(r0, r1, r2);
        rotation              = glm::normalize(glm::quat_cast(rotMat));
        glm::dmat4 rigidWorld = glm::translate(glm::dmat4(1.0), position) * glm::mat4_cast(rotation);
        localScale            = glm::inverse(rigidWorld) * matrix;
    }
}
