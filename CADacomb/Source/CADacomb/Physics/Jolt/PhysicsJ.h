#pragma once

#include "CADacomb/Physics/Interface/Physics.h"
#include "CADacomb/Physics/Jolt/BroadPhaseLayerInterfaceJ.h"
#include "CADacomb/Physics/Jolt/ObjectLayerPairFilterJ.h"
#include "CADacomb/Physics/Jolt/ObjectVsBroadPhaseLayerFilterJ.h"

#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <memory>
#include <mutex>

namespace CADacomb::Physics
{
    // Jolt Physics engine implementation and instancing hub.
    class PhysicsJ : public Physics
    {
      public:
        PhysicsJ();
        virtual ~PhysicsJ();

        std::shared_ptr<RigidBody> createRigidBody(const Triangulation& triangulation, const glm::dmat4& transformation, bool isStatic) override;
        std::shared_ptr<RigidBody> createRigidBody(const SVO<bool>& svo, const AABB3& localBounds, const glm::dmat4& transformation, bool isStatic) override;

        void       setGravity(const glm::dvec3& gravity) override;
        glm::dvec3 getGravity() const override;

        void step(double deltaTime) override;

      private:
        static void               ensureInitialized();
        JPH::RefConst<JPH::Shape> buildShape(const Triangulation& triangulation, const glm::dmat4& localScale, bool isStatic) const;

        static std::once_flag                         initOnceFlag;
        BroadPhaseLayerInterfaceJ                     broadPhaseLayerInterface;
        ObjectVsBroadPhaseLayerFilterJ                objectVsBroadPhaseLayerFilter;
        ObjectLayerPairFilterJ                        objectLayerPairFilter;
        std::unique_ptr<JPH::TempAllocatorImpl>       tempAllocator;
        std::unique_ptr<JPH::JobSystemSingleThreaded> jobSystem;
        std::shared_ptr<JPH::PhysicsSystem>           physicsSystem;
    };
}
