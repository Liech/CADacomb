#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::CAD
{
    // Represents a shared boundary polyline chain between two clusters (or a cluster and open boundary).
    class ClusterBoundaryChain
    {
      public:
        ClusterBoundaryChain();
        virtual ~ClusterBoundaryChain();

        size_t startCornerVertex = 0;
        size_t endCornerVertex   = 0;
        size_t clusterA          = 0;
        size_t clusterB          = SIZE_MAX;
        bool   closedLoop        = false;

        std::vector<size_t>     vertexIndices;
        std::vector<glm::dvec3> points;
    };
}
