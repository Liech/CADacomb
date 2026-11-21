#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace CADacomb::Physics
{
    // Filters collisions between static and dynamic Jolt object layers.
    class ObjectLayerPairFilterJ : public JPH::ObjectLayerPairFilter
    {
      public:
        ObjectLayerPairFilterJ();
        virtual ~ObjectLayerPairFilterJ();

        bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override;

        static constexpr JPH::ObjectLayer nonMovingLayer = 0;
        static constexpr JPH::ObjectLayer movingLayer    = 1;
        static constexpr JPH::uint        numLayers      = 2;
    };
}
