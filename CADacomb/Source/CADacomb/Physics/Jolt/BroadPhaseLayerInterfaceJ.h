#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

namespace CADacomb::Physics
{
    // Maps Jolt object layers to broad-phase collision layers.
    class BroadPhaseLayerInterfaceJ : public JPH::BroadPhaseLayerInterface
    {
      public:
        BroadPhaseLayerInterfaceJ();
        virtual ~BroadPhaseLayerInterfaceJ();

        JPH::uint            GetNumBroadPhaseLayers() const override;
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override;
#endif
    };
}
