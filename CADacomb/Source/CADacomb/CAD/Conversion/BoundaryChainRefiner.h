#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::CAD
{
    class ClusterBoundaryGraph;
    class ConeSurfaceFit;
    class CylinderSurfaceFit;
    class PlanarSurfaceFit;
    class SphereSurfaceFit;

    // Projects shared boundary chains and corner vertices onto the intersections of fitted analytical surfaces.
    class BoundaryChainRefiner
    {
      public:
        BoundaryChainRefiner();
        virtual ~BoundaryChainRefiner();

        static void refine(ClusterBoundaryGraph&                  graph,
                           const std::vector<int>&                clusterKind,
                           const std::vector<PlanarSurfaceFit>&   planarFits,
                           const std::vector<CylinderSurfaceFit>& cylinderFits,
                           const std::vector<ConeSurfaceFit>&     coneFits,
                           const std::vector<SphereSurfaceFit>&   sphereFits,
                           double                                 maxSnapDistance);

      private:
        static glm::dvec3 projectOntoClusterSurface(const glm::dvec3&                      pt,
                                                    size_t                                 clusterIdx,
                                                    const std::vector<int>&                clusterKind,
                                                    const std::vector<PlanarSurfaceFit>&   planarFits,
                                                    const std::vector<CylinderSurfaceFit>& cylinderFits,
                                                    const std::vector<ConeSurfaceFit>&     coneFits,
                                                    const std::vector<SphereSurfaceFit>&   sphereFits);

        static glm::dvec3 refinePointOnClusters(const glm::dvec3&                      pt,
                                                const std::vector<size_t>&             incidentClusters,
                                                const std::vector<int>&                clusterKind,
                                                const std::vector<PlanarSurfaceFit>&   planarFits,
                                                const std::vector<CylinderSurfaceFit>& cylinderFits,
                                                const std::vector<ConeSurfaceFit>&     coneFits,
                                                const std::vector<SphereSurfaceFit>&   sphereFits,
                                                double                                 maxSnapDistance);
    };
}
