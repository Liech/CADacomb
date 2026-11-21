#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

namespace CADacomb::Physics
{
    // Filters collisions between Jolt object layers and broad-phase layers.
    class ObjectVsBroadPhaseLayerFilterJ : public JPH::ObjectVsBroadPhaseLayerFilter
    {
      public:
        ObjectVsBroadPhaseLayerFilterJ();
        virtual ~ObjectVsBroadPhaseLayerFilterJ();

        bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
    };
}
