#include "CADacomb/Physics/Jolt/ObjectVsBroadPhaseLayerFilterJ.h"

#include "CADacomb/Physics/Jolt/ObjectLayerPairFilterJ.h"

namespace CADacomb::Physics
{
    ObjectVsBroadPhaseLayerFilterJ::ObjectVsBroadPhaseLayerFilterJ() {}

    ObjectVsBroadPhaseLayerFilterJ::~ObjectVsBroadPhaseLayerFilterJ() {}

    bool ObjectVsBroadPhaseLayerFilterJ::ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const
    {
        if (inLayer1 == ObjectLayerPairFilterJ::nonMovingLayer)
        {
            return inLayer2 == JPH::BroadPhaseLayer(ObjectLayerPairFilterJ::movingLayer);
        }
        if (inLayer1 == ObjectLayerPairFilterJ::movingLayer)
        {
            return true;
        }
        return false;
    }
}
