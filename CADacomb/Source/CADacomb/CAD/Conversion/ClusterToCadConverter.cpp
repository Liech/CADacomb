#include "ClusterToCadConverter.h"

#include "CADacomb/CAD/Interface/Kernel.h"
#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/Triangle/Triangulation.h"
#include "ClusterBoundaryGraph.h"
#include "PlanarSurfaceFit.h"

#include <stdexcept>

namespace CADacomb::CAD
{
    ClusterToCadConverter::ClusterToCadConverter() {}

    ClusterToCadConverter::~ClusterToCadConverter() {}

    std::unique_ptr<Shape> ClusterToCadConverter::convertPlanar(const Kernel&                           kernel,
                                                                const Triangulation&                    mesh,
                                                                const std::vector<std::vector<size_t>>& clusters,
                                                                double                                  planarityTolerance,
                                                                double                                  boundarySimplifyTolerance,
                                                                double                                  stitchTolerance,
                                                                bool                                    tryMakeSolid) const
    {
        if (clusters.empty())
        {
            throw std::runtime_error("ClusterToCadConverter::convertPlanar: no clusters provided");
        }

        ClusterBoundaryGraph graph = ClusterBoundaryGraph::build(mesh, clusters);
        graph.simplifyChains(boundarySimplifyTolerance);

        std::vector<std::unique_ptr<Shape>> faceShapes;
        faceShapes.reserve(clusters.size());

        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            PlanarSurfaceFit fit = PlanarSurfaceFit::fit(mesh, clusters[clusterIndex]);
            if (!fit.isValid() || fit.getMaxDeviation() > planarityTolerance)
            {
                continue;
            }

            std::vector<std::vector<glm::dvec3>> loops = graph.buildClusterPointLoops(clusterIndex);
            if (loops.empty())
            {
                continue;
            }

            faceShapes.push_back(kernel.createPlanarFace(fit.getOrigin(), fit.getNormal(), loops));
        }

        if (faceShapes.empty())
        {
            throw std::runtime_error("ClusterToCadConverter::convertPlanar: no planar faces created");
        }

        std::vector<const Shape*> facePtrs;
        facePtrs.reserve(faceShapes.size());
        for (size_t i = 0; i < faceShapes.size(); i++)
        {
            facePtrs.push_back(faceShapes[i].get());
        }

        std::unique_ptr<Shape> merged   = kernel.mergeShapes(facePtrs);
        std::unique_ptr<Shape> stitched = kernel.stitch(*merged, stitchTolerance, tryMakeSolid);
        kernel.unifyNormals(*stitched);
        return stitched;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/CAD/Interface/TessellationOptions.h"
#include "CADacomb/CAD/Sm/KernelSm.h"
#include "CADacomb/Triangle/BaseShapeGenerator.h"
#include "CADacomb/Triangle/Clustering.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("ClusterToCadConverter/BoxClusteringToPlanarCad")
{
    CADacomb::Triangulation boxTri = CADacomb::BaseShapeGenerator::cube();
    std::vector<std::vector<size_t>> clusters = CADacomb::Clustering::clusterByNormal(boxTri, 0.99);
    REQUIRE(clusters.size() == 6);

    CADacomb::CAD::KernelSm              kernel;
    CADacomb::CAD::ClusterToCadConverter converter;
    std::unique_ptr<CADacomb::CAD::Shape> shape = converter.convertPlanar(kernel, boxTri, clusters, 1e-4, 1e-4, 1e-4, true);

    REQUIRE(shape != nullptr);

    CADacomb::AABB3 triBounds = boxTri.getAABB();
    CADacomb::AABB3 bounds    = shape->getAABB();
    REQUIRE(bounds.min.x == Catch::Approx(triBounds.min.x).margin(1e-3));
    REQUIRE(bounds.min.y == Catch::Approx(triBounds.min.y).margin(1e-3));
    REQUIRE(bounds.min.z == Catch::Approx(triBounds.min.z).margin(1e-3));
    REQUIRE(bounds.max.x == Catch::Approx(triBounds.max.x).margin(1e-3));
    REQUIRE(bounds.max.y == Catch::Approx(triBounds.max.y).margin(1e-3));
    REQUIRE(bounds.max.z == Catch::Approx(triBounds.max.z).margin(1e-3));

    CADacomb::CAD::TessellationOptions     tessOptions;
    std::unique_ptr<CADacomb::Triangulation> tess = kernel.tessellate(*shape, tessOptions);
    REQUIRE(tess != nullptr);
    REQUIRE(tess->indices.size() >= 36);
}
#endif
