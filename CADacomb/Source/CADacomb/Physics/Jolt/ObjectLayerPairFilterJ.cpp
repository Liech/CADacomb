#include "CADacomb/Physics/Jolt/ObjectLayerPairFilterJ.h"

namespace CADacomb::Physics
{
    ObjectLayerPairFilterJ::ObjectLayerPairFilterJ() {}

    ObjectLayerPairFilterJ::~ObjectLayerPairFilterJ() {}

    bool ObjectLayerPairFilterJ::ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const
    {
        if (inObject1 == nonMovingLayer)
        {
            return inObject2 == movingLayer;
        }
        if (inObject1 == movingLayer)
        {
            return true;
        }
        return false;
    }
}
