#include "ClusterToCadConverter.h"

#include "CADacomb/CAD/Interface/Kernel.h"
#include "CADacomb/CAD/Interface/Shape.h"
#include "CADacomb/Triangle/Triangulation.h"
#include "ClusterBoundaryGraph.h"
#include "CylinderSurfaceFit.h"
#include "NurbsSurfaceFit.h"
#include "PlanarSurfaceFit.h"

#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace CADacomb::CAD
{
    ClusterToCadConverter::ClusterToCadConverter() {}

    ClusterToCadConverter::~ClusterToCadConverter() {}

    std::vector<std::vector<size_t>> ClusterToCadConverter::splitWideAngleClusters(const Triangulation&                    mesh,
                                                                                   const std::vector<std::vector<size_t>>& clusters,
                                                                                   double                                  planarityTolerance)
    {
        const double                     minSeedDot = 0.65;
        std::vector<std::vector<size_t>> refined;
        refined.reserve(clusters.size());

        auto packUndirectedEdge = [](size_t a, size_t b) -> uint64_t
        {
            size_t lo = std::min(a, b);
            size_t hi = std::max(a, b);
            return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
        };

        for (size_t cIdx = 0; cIdx < clusters.size(); cIdx++)
        {
            const std::vector<size_t>& cluster = clusters[cIdx];
            if (cluster.empty())
            {
                continue;
            }

            PlanarSurfaceFit planarFit = PlanarSurfaceFit::fit(mesh, cluster);
            if (planarFit.isValid() && planarFit.getMaxDeviation() <= planarityTolerance)
            {
                refined.push_back(cluster);
                continue;
            }

            glm::dvec3 firstNormal = mesh.getFaceNormal(cluster[0]);
            bool       needsSplit  = false;
            for (size_t i = 1; i < cluster.size(); i++)
            {
                if (glm::dot(mesh.getFaceNormal(cluster[i]), firstNormal) < minSeedDot)
                {
                    needsSplit = true;
                    break;
                }
            }

            if (!needsSplit)
            {
                refined.push_back(cluster);
                continue;
            }

            std::unordered_map<uint64_t, std::vector<size_t>> edgeToLocalTris;
            for (size_t localIdx = 0; localIdx < cluster.size(); localIdx++)
            {
                size_t triIdx = cluster[localIdx];
                size_t i0     = mesh.indices[triIdx * 3 + 0];
                size_t i1     = mesh.indices[triIdx * 3 + 1];
                size_t i2     = mesh.indices[triIdx * 3 + 2];
                edgeToLocalTris[packUndirectedEdge(i0, i1)].push_back(localIdx);
                edgeToLocalTris[packUndirectedEdge(i1, i2)].push_back(localIdx);
                edgeToLocalTris[packUndirectedEdge(i2, i0)].push_back(localIdx);
            }

            std::vector<bool> visited(cluster.size(), false);
            for (size_t startLocal = 0; startLocal < cluster.size(); startLocal++)
            {
                if (visited[startLocal])
                {
                    continue;
                }

                std::vector<size_t> subCluster;
                std::queue<size_t>  q;
                visited[startLocal]   = true;
                glm::dvec3 seedNormal = mesh.getFaceNormal(cluster[startLocal]);
                q.push(startLocal);

                while (!q.empty())
                {
                    size_t currLocal = q.front();
                    q.pop();
                    size_t currTri = cluster[currLocal];
                    subCluster.push_back(currTri);

                    size_t verts[3] = {
                        mesh.indices[currTri * 3 + 0],
                        mesh.indices[currTri * 3 + 1],
                        mesh.indices[currTri * 3 + 2]
                    };

                    for (int e = 0; e < 3; e++)
                    {
                        uint64_t key = packUndirectedEdge(verts[e], verts[(e + 1) % 3]);
                        auto     it  = edgeToLocalTris.find(key);
                        if (it == edgeToLocalTris.end())
                        {
                            continue;
                        }
                        for (size_t nbLocal : it->second)
                        {
                            if (!visited[nbLocal] && glm::dot(mesh.getFaceNormal(cluster[nbLocal]), seedNormal) >= minSeedDot)
                            {
                                visited[nbLocal] = true;
                                q.push(nbLocal);
                            }
                        }
                    }
                }

                refined.push_back( std::move(subCluster) );
            }
        }

        return refined;
    }

    std::unique_ptr<Shape> ClusterToCadConverter::convert(const Kernel&                           kernel,
                                                          const Triangulation&                    mesh,
                                                          const std::vector<std::vector<size_t>>& clusters,
                                                          double                                  planarityTolerance,
                                                          double                                  boundarySimplifyTolerance,
                                                          double                                  stitchTolerance,
                                                          bool                                    tryMakeSolid) const
    {
        if (clusters.empty())
        {
            throw std::runtime_error("ClusterToCadConverter::convert: no clusters provided");
        }

        std::vector<std::vector<size_t>> refinedClusters = splitWideAngleClusters(mesh, clusters, planarityTolerance);

        ClusterBoundaryGraph graph = ClusterBoundaryGraph::build(mesh, refinedClusters);
        graph.simplifyChains(boundarySimplifyTolerance);

        std::vector<std::unique_ptr<Shape>> faceShapes;
        faceShapes.reserve(refinedClusters.size());

        for (size_t clusterIndex = 0; clusterIndex < refinedClusters.size(); clusterIndex++)
        {
            std::vector<std::vector<glm::dvec3>> loops = graph.buildClusterPointLoops(clusterIndex);
            if (loops.empty())
            {
                continue;
            }

            PlanarSurfaceFit planarFit = PlanarSurfaceFit::fit(mesh, refinedClusters[clusterIndex]);
            if (planarFit.isValid() && planarFit.getMaxDeviation() <= planarityTolerance)
            {
                try
                {
                    faceShapes.push_back(kernel.createPlanarFace(planarFit.getOrigin(), planarFit.getNormal(), loops));
                    continue;
                }
                catch (const std::exception&)
                {
                }
            }

            double             cylinderTolerance = std::max(planarityTolerance * 10.0, 1e-2);
            CylinderSurfaceFit cylinderFit       = CylinderSurfaceFit::fit(mesh, refinedClusters[clusterIndex]);
            if (cylinderFit.isValid() && cylinderFit.getMaxDeviation() <= cylinderTolerance)
            {
                try
                {
                    faceShapes.push_back(kernel.createCylindricalFace(cylinderFit.getOrigin(), cylinderFit.getAxis(), cylinderFit.getRadius(), cylinderFit.getHeight(), loops));
                    continue;
                }
                catch (const std::exception&)
                {
                }
            }

            NurbsSurfaceFit nurbsFit = NurbsSurfaceFit::fit(mesh, refinedClusters[clusterIndex]);
            if (nurbsFit.isValid())
            {
                try
                {
                    faceShapes.push_back(kernel.createBSplineFace(nurbsFit.getNumRows(), nurbsFit.getNumCols(), nurbsFit.getControlPoints(), loops));
                    continue;
                }
                catch (const std::exception&)
                {
                }
            }

            for (size_t triIdx : refinedClusters[clusterIndex])
            {
                const glm::dvec3& p0       = mesh.vertices[mesh.indices[triIdx * 3 + 0]];
                const glm::dvec3& p1       = mesh.vertices[mesh.indices[triIdx * 3 + 1]];
                const glm::dvec3& p2       = mesh.vertices[mesh.indices[triIdx * 3 + 2]];
                glm::dvec3        crossVec = glm::cross(p1 - p0, p2 - p0);
                double            len      = glm::length(crossVec);
                if (len < 1e-12)
                {
                    continue;
                }
                faceShapes.push_back(kernel.createPlanarFace(p0, crossVec / len, { { p0, p1, p2 } }));
            }
        }

        if (faceShapes.empty())
        {
            throw std::runtime_error("ClusterToCadConverter::convert: no faces created");
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

TEST_CASE("ClusterToCadConverter/CurvedPatchToNurbsCad")
{
    CADacomb::Triangulation mesh;
    const size_t            gridN = 9;
    for (size_t y = 0; y < gridN; y++)
    {
        for (size_t x = 0; x < gridN; x++)
        {
            double px = static_cast<double>(x) / static_cast<double>(gridN - 1);
            double py = static_cast<double>(y) / static_cast<double>(gridN - 1);
            double pz = 0.2 * std::sin(px * 3.14159);
            mesh.vertices.push_back(glm::dvec3(px, py, pz));
        }
    }

    std::vector<size_t> cluster;
    for (size_t y = 0; y + 1 < gridN; y++)
    {
        for (size_t x = 0; x + 1 < gridN; x++)
        {
            size_t i0 = y * gridN + x;
            size_t i1 = i0 + 1;
            size_t i2 = i0 + gridN;
            size_t i3 = i2 + 1;
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(i0);
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i3);
            cluster.push_back(mesh.indices.size() / 3);
            mesh.indices.push_back(i0);
            mesh.indices.push_back(i3);
            mesh.indices.push_back(i2);
        }
    }

    CADacomb::CAD::KernelSm              kernel;
    CADacomb::CAD::ClusterToCadConverter converter;
    std::unique_ptr<CADacomb::CAD::Shape> shape = converter.convert(kernel, mesh, { cluster }, 1e-4, 1e-4, 1e-3, false);

    REQUIRE(shape != nullptr);

    CADacomb::CAD::TessellationOptions     tessOptions;
    std::unique_ptr<CADacomb::Triangulation> tess = kernel.tessellate(*shape, tessOptions);
    REQUIRE(tess != nullptr);
    REQUIRE(!tess->indices.empty());
}
#endif
