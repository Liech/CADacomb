#pragma once

#include "CADacomb/Physics/Interface/RigidBody.h"
#include "CADacomb/Physics/Interface/SoftBody.h"
#include "CADacomb/Physics/SoftBodyMode.h"
#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"

#include <glm/glm.hpp>
#include <memory>

namespace CADacomb::Physics
{
    // Abstract physics engine interface and instancing hub for rigid bodies, soft bodies, and simulation stepping.
    class Physics
    {
      public:
        Physics();
        virtual ~Physics();

        virtual std::shared_ptr<RigidBody> createRigidBody(const Triangulation& triangulation, const glm::dmat4& transformation, bool isStatic)             = 0;
        virtual std::shared_ptr<RigidBody> createRigidBody(const SVO<bool>& svo, const AABB3& localBounds, const glm::dmat4& transformation, bool isStatic) = 0;
        virtual std::shared_ptr<SoftBody>  createSoftBody(const Triangulation& triangulation,
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
                                                          int                  solverIterations)                                                            = 0;

        virtual void       setGravity(const glm::dvec3& gravity) = 0;
        virtual glm::dvec3 getGravity() const                    = 0;

        virtual void step(double deltaTime) = 0;
    };
}
