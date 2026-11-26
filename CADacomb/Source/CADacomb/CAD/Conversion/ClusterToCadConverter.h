#pragma once

#include <memory>
#include <vector>

namespace CADacomb
{
    class Triangulation;
}

namespace CADacomb::CAD
{
    class Kernel;
    class Shape;

    // Converts a triangulation clustering into a stitched CAD Shape using the Kernel interface.
    class ClusterToCadConverter
    {
      public:
        ClusterToCadConverter();
        virtual ~ClusterToCadConverter();

        std::unique_ptr<Shape> convert(const Kernel&                           kernel,
                                       const Triangulation&                    mesh,
                                       const std::vector<std::vector<size_t>>& clusters,
                                       double                                  planarityTolerance,
                                       double                                  boundarySimplifyTolerance,
                                       double                                  stitchTolerance,
                                       bool                                    tryMakeSolid) const;

        std::unique_ptr<Shape> convertPlanar(const Kernel&                           kernel,
                                             const Triangulation&                    mesh,
                                             const std::vector<std::vector<size_t>>& clusters,
                                             double                                  planarityTolerance,
                                             double                                  boundarySimplifyTolerance,
                                             double                                  stitchTolerance,
                                             bool                                    tryMakeSolid) const;

      private:
        static std::vector<std::vector<size_t>> splitWideAngleClusters(const Triangulation&                    mesh,
                                                                       const std::vector<std::vector<size_t>>& clusters,
                                                                       double                                  planarityTolerance);
    };
}
