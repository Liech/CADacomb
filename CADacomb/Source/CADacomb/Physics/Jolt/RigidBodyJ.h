#pragma once

#include "CADacomb/Physics/Interface/RigidBody.h"
#include "CADacomb/Physics/Jolt/SVOShapeJ.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

namespace CADacomb::Physics
{
    // Jolt Physics implementation of a simulated rigid body.
    class RigidBodyJ : public RigidBody
    {
      public:
        RigidBodyJ(std::shared_ptr<JPH::PhysicsSystem> physicsSystem, JPH::BodyID bodyId, const glm::dmat4& localScaleTransform, bool isStatic, JPH::Ref<SVOShapeJ> svoShape = nullptr);
        virtual ~RigidBodyJ();

        bool       isStatic() const override;
        glm::dmat4 getTransformation() const override;
        void       setTransformation(const glm::dmat4& transformation) override;
        void       notifyGeometryChanged() override;

        static void decomposeTransformation(const glm::dmat4& matrix, glm::dvec3& position, glm::dquat& rotation, glm::dmat4& localScale);

      private:
        std::weak_ptr<JPH::PhysicsSystem> physicsSystem;
        JPH::BodyID                       bodyId;
        JPH::Ref<SVOShapeJ>               svoShape;
        glm::dmat4                        localScaleTransform{ 1.0 };
        bool                              staticBody = false;
    };
}
