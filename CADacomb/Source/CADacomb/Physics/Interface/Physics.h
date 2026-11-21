#pragma once

#include "CADacomb/Physics/Interface/RigidBody.h"
#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"

#include <glm/glm.hpp>
#include <memory>

namespace CADacomb::Physics
{
    // Abstract physics engine interface and instancing hub for rigid bodies and simulation stepping.
    class Physics
    {
      public:
        Physics();
        virtual ~Physics();

        virtual std::shared_ptr<RigidBody> createRigidBody(const Triangulation& triangulation, const glm::dmat4& transformation, bool isStatic)             = 0;
        virtual std::shared_ptr<RigidBody> createRigidBody(const SVO<bool>& svo, const AABB3& localBounds, const glm::dmat4& transformation, bool isStatic) = 0;

        virtual void       setGravity(const glm::dvec3& gravity) = 0;
        virtual glm::dvec3 getGravity() const                    = 0;

        virtual void step(double deltaTime) = 0;
    };
}
