#pragma once

#include "ClusterBoundaryChain.h"
#include "ClusterLoopReference.h"

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    // Extracts shared corner vertices, boundary chains, and per-cluster loops from a triangulation clustering.
    class ClusterBoundaryGraph
    {
      public:
        ClusterBoundaryGraph();
        virtual ~ClusterBoundaryGraph();

        static ClusterBoundaryGraph build(const Triangulation& mesh, const std::vector<std::vector<size_t>>& clusters);

        void simplifyChains(double chordTolerance);
        void simplifyPlanarChains(double chordTolerance, const std::vector<bool>& clusterIsPlanar);
        void setChainPoints(size_t chainIndex, const std::vector<glm::dvec3>& points);

        const std::vector<ClusterBoundaryChain>&              getChains() const;
        const std::vector<std::vector<ClusterLoopReference>>& getClusterLoops(size_t clusterIndex) const;
        size_t                                                getClusterCount() const;
        std::vector<std::vector<glm::dvec3>>                  buildClusterPointLoops(size_t clusterIndex) const;

      private:
        static void simplifyChain(ClusterBoundaryChain& chain, double toleranceSq);
        static void simplifyPolylineRange(const std::vector<glm::dvec3>& points,
                                          size_t                         startIndex,
                                          size_t                         endIndex,
                                          double                         toleranceSq,
                                          std::vector<bool>&             keepMask);

        std::vector<ClusterBoundaryChain>                           chains;
        std::vector<std::vector<std::vector<ClusterLoopReference>>> clusterLoops;
    };
}
