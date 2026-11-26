#pragma once

#include <cstddef>

namespace CADacomb::CAD
{
    // Directed reference to a shared ClusterBoundaryChain within a cluster's boundary loop.
    class ClusterLoopReference
    {
      public:
        ClusterLoopReference();
        ClusterLoopReference(size_t chainIndexInput, bool reversedInput);
        virtual ~ClusterLoopReference();

        size_t chainIndex = 0;
        bool   reversed   = false;
    };
}
