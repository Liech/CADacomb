#include "CADacomb/Physics/Jolt/BroadPhaseLayerInterfaceJ.h"

#include "CADacomb/Physics/Jolt/ObjectLayerPairFilterJ.h"

namespace CADacomb::Physics
{
    BroadPhaseLayerInterfaceJ::BroadPhaseLayerInterfaceJ() {}

    BroadPhaseLayerInterfaceJ::~BroadPhaseLayerInterfaceJ() {}

    JPH::uint BroadPhaseLayerInterfaceJ::GetNumBroadPhaseLayers() const
    {
        return ObjectLayerPairFilterJ::numLayers;
    }

    JPH::BroadPhaseLayer BroadPhaseLayerInterfaceJ::GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const
    {
        return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(inLayer));
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* BroadPhaseLayerInterfaceJ::GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const
    {
        if (inLayer == JPH::BroadPhaseLayer(0))
        {
            return "NonMoving";
        }
        return "Moving";
    }
#endif
}
