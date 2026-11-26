#include "ClusterLoopReference.h"

namespace CADacomb::CAD
{
    ClusterLoopReference::ClusterLoopReference() {}

    ClusterLoopReference::ClusterLoopReference(size_t chainIndexInput, bool reversedInput)
      : chainIndex(chainIndexInput)
      , reversed(reversedInput)
    {
    }

    ClusterLoopReference::~ClusterLoopReference() {}
}
