#include "CADacomb/Physics/Jolt/SoftBodyJ.h"

#include "CADacomb/Physics/Jolt/RigidBodyJ.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace CADacomb::Physics
{
    SoftBodyJ::SoftBodyJ(std::shared_ptr<JPH::PhysicsSystem> physicsSystemInput,
                         JPH::BodyID                         bodyIdInput,
                         const glm::dmat4&                   localScaleTransformInput,
                         const std::vector<size_t>&          originalToWeldedIndicesInput)
      : physicsSystem(physicsSystemInput)
      , bodyId(bodyIdInput)
      , localScaleTransform(localScaleTransformInput)
      , inverseLocalScaleTransform(glm::inverse(localScaleTransformInput))
      , originalToWeldedIndices(originalToWeldedIndicesInput)
    {
    }

    SoftBodyJ::~SoftBodyJ()
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

    std::vector<glm::dvec3> SoftBodyJ::getDeformedVertices() const
    {
        std::vector<glm::dvec3> deformedVertices;
        auto                    system = physicsSystem.lock();
        if (!system || bodyId.IsInvalid())
        {
            return deformedVertices;
        }

        JPH::BodyLockRead lock(system->GetBodyLockInterfaceNoLock(), bodyId);
        if (!lock.Succeeded())
        {
            return deformedVertices;
        }

        const JPH::Body& body             = lock.GetBody();
        const auto*      motionProperties = static_cast<const JPH::SoftBodyMotionProperties*>(body.GetMotionProperties());
        if (!motionProperties)
        {
            return deformedVertices;
        }

        const JPH::Array<JPH::SoftBodyVertex>& joltVertices = motionProperties->GetVertices();
        deformedVertices.resize(originalToWeldedIndices.size(), glm::dvec3(0.0));

        for (size_t i = 0; i < originalToWeldedIndices.size(); i++)
        {
            size_t weldedIdx = originalToWeldedIndices[i];
            if (weldedIdx < joltVertices.size())
            {
                JPH::Vec3  localScaledPos = joltVertices[weldedIdx].mPosition;
                glm::dvec3 scaledVec(static_cast<double>(localScaledPos.GetX()), static_cast<double>(localScaledPos.GetY()), static_cast<double>(localScaledPos.GetZ()));
                deformedVertices[i] = glm::dvec3(inverseLocalScaleTransform * glm::dvec4(scaledVec, 1.0));
            }
        }

        return deformedVertices;
    }

    glm::dmat4 SoftBodyJ::getTransformation() const
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

    void SoftBodyJ::setTransformation(const glm::dmat4& transformation)
    {
        glm::dvec3 position;
        glm::dquat rotation;
        RigidBodyJ::decomposeTransformation(transformation, position, rotation, localScaleTransform);
        inverseLocalScaleTransform = glm::inverse(localScaleTransform);

        auto system = physicsSystem.lock();
        if (system && !bodyId.IsInvalid())
        {
            system->GetBodyInterface().SetPositionAndRotation(bodyId,
                                                              JPH::RVec3(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z)),
                                                              JPH::Quat(static_cast<float>(rotation.x), static_cast<float>(rotation.y), static_cast<float>(rotation.z), static_cast<float>(rotation.w)),
                                                              JPH::EActivation::Activate);
        }
    }
}
