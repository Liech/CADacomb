#pragma once

#include "CADacomb/Physics/Interface/SoftBody.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb::Physics
{
    // Jolt Physics implementation of a simulated deformable soft body.
    class SoftBodyJ : public SoftBody
    {
      public:
        SoftBodyJ(std::shared_ptr<JPH::PhysicsSystem> physicsSystem, JPH::BodyID bodyId, const glm::dmat4& localScaleTransform, const std::vector<size_t>& originalToWeldedIndices);
        virtual ~SoftBodyJ();

        std::vector<glm::dvec3> getDeformedVertices() const override;
        glm::dmat4              getTransformation() const override;
        void                    setTransformation(const glm::dmat4& transformation) override;

      private:
        std::weak_ptr<JPH::PhysicsSystem> physicsSystem;
        JPH::BodyID                       bodyId;
        glm::dmat4                        localScaleTransform{ 1.0 };
        glm::dmat4                        inverseLocalScaleTransform{ 1.0 };
        std::vector<size_t>               originalToWeldedIndices;
    };
}
