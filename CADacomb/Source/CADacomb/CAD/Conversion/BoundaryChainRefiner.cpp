#include "BoundaryChainRefiner.h"

#include "ClusterBoundaryGraph.h"
#include "ConeSurfaceFit.h"
#include "CylinderSurfaceFit.h"
#include "PlanarSurfaceFit.h"
#include "SphereSurfaceFit.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace CADacomb::CAD
{
    BoundaryChainRefiner::BoundaryChainRefiner() {}

    BoundaryChainRefiner::~BoundaryChainRefiner() {}

    glm::dvec3 BoundaryChainRefiner::projectOntoClusterSurface(const glm::dvec3&                      pt,
                                                                 size_t                                 clusterIdx,
                                                                 const std::vector<int>&                clusterKind,
                                                                 const std::vector<PlanarSurfaceFit>&   planarFits,
                                                                 const std::vector<CylinderSurfaceFit>& cylinderFits,
                                                                 const std::vector<ConeSurfaceFit>&     coneFits,
                                                                 const std::vector<SphereSurfaceFit>&   sphereFits)
    {
        if (clusterIdx >= clusterKind.size())
        {
            return pt;
        }

        int kind = clusterKind[clusterIdx];
        if (kind == 0 && planarFits[clusterIdx].isValid())
        {
            const glm::dvec3& o = planarFits[clusterIdx].getOrigin();
            const glm::dvec3& n = planarFits[clusterIdx].getNormal();
            return pt - glm::dot(pt - o, n) * n;
        }
        if (kind == 1 && cylinderFits[clusterIdx].isValid())
        {
            const glm::dvec3& o      = cylinderFits[clusterIdx].getOrigin();
            const glm::dvec3& a      = cylinderFits[clusterIdx].getAxis();
            double            r      = cylinderFits[clusterIdx].getRadius();
            glm::dvec3        rel    = pt - o;
            glm::dvec3        axial  = glm::dot(rel, a) * a;
            glm::dvec3        radial = rel - axial;
            double            rLen   = glm::length(radial);
            if (rLen > 1e-12)
            {
                return o + axial + (r / rLen) * radial;
            }
            return pt;
        }
        if (kind == 2 && coneFits[clusterIdx].isValid())
        {
            const ConeSurfaceFit& cf     = coneFits[clusterIdx];
            const glm::dvec3&     o      = cf.getOrigin();
            const glm::dvec3&     a      = cf.getAxis();
            double                h      = cf.getHeight();
            double                m      = (cf.getTopRadius() - cf.getBotRadius()) / h;
            glm::dvec3            rel    = pt - o;
            double                hPt    = glm::dot(rel, a);
            glm::dvec3            radial = rel - hPt * a;
            double                rLen   = glm::length(radial);
            if (rLen > 1e-12)
            {
                double t     = (hPt + (rLen - cf.getBotRadius()) * m) / (1.0 + m * m);
                double rProj = std::max(cf.getBotRadius() + m * t, 0.0);
                return o + t * a + (rProj / rLen) * radial;
            }
            return pt;
        }
        if (kind == 3 && sphereFits[clusterIdx].isValid())
        {
            const glm::dvec3& c   = sphereFits[clusterIdx].getCenter();
            double            r   = sphereFits[clusterIdx].getRadius();
            glm::dvec3        dir = pt - c;
            double            len = glm::length(dir);
            if (len > 1e-12)
            {
                return c + (r / len) * dir;
            }
            return pt;
        }

        return pt;
    }

    glm::dvec3 BoundaryChainRefiner::refinePointOnClusters(const glm::dvec3&                      pt,
                                                             const std::vector<size_t>&             incidentClusters,
                                                             const std::vector<int>&                clusterKind,
                                                             const std::vector<PlanarSurfaceFit>&   planarFits,
                                                             const std::vector<CylinderSurfaceFit>& cylinderFits,
                                                             const std::vector<ConeSurfaceFit>&     coneFits,
                                                             const std::vector<SphereSurfaceFit>&   sphereFits,
                                                             double                                 maxSnapDistance)
    {
        std::vector<size_t> analytical;
        analytical.reserve(incidentClusters.size());
        for (size_t cIdx : incidentClusters)
        {
            if (cIdx < clusterKind.size() && clusterKind[cIdx] >= 0 && clusterKind[cIdx] <= 3)
            {
                analytical.push_back(cIdx);
            }
        }

        if (analytical.empty())
        {
            return pt;
        }

        // Exact orthogonal projection onto the intersection line of two non-parallel planes.
        if (analytical.size() == 2 && clusterKind[analytical[0]] == 0 && clusterKind[analytical[1]] == 0)
        {
            const glm::dvec3& oA   = planarFits[analytical[0]].getOrigin();
            const glm::dvec3& nA   = planarFits[analytical[0]].getNormal();
            const glm::dvec3& oB   = planarFits[analytical[1]].getOrigin();
            const glm::dvec3& nB   = planarFits[analytical[1]].getNormal();
            double            nAnB = glm::dot(nA, nB);
            double            det  = 1.0 - nAnB * nAnB;
            if (det > 1e-6)
            {
                double     dA = glm::dot(oA - pt, nA);
                double     dB = glm::dot(oB - pt, nB);
                double     cA = (dA - nAnB * dB) / det;
                double     cB = (dB - nAnB * dA) / det;
                glm::dvec3 q  = pt + cA * nA + cB * nB;
                if (glm::length(q - pt) <= maxSnapDistance)
                {
                    return q;
                }
            }
            return pt;
        }

        glm::dvec3 curr = pt;
        for (int iter = 0; iter < 6; iter++)
        {
            for (size_t cIdx : analytical)
            {
                curr = projectOntoClusterSurface(curr, cIdx, clusterKind, planarFits, cylinderFits, coneFits, sphereFits);
            }
        }

        if (glm::length(curr - pt) <= maxSnapDistance)
        {
            return curr;
        }
        return pt;
    }

    void BoundaryChainRefiner::refine(ClusterBoundaryGraph&                  graph,
                                      const std::vector<int>&                clusterKind,
                                      const std::vector<PlanarSurfaceFit>&   planarFits,
                                      const std::vector<CylinderSurfaceFit>& cylinderFits,
                                      const std::vector<ConeSurfaceFit>&     coneFits,
                                      const std::vector<SphereSurfaceFit>&   sphereFits,
                                      double                                 maxSnapDistance)
    {
        if (maxSnapDistance <= 0.0)
        {
            return;
        }

        const std::vector<ClusterBoundaryChain>& chains = graph.getChains();

        std::unordered_map<size_t, std::unordered_set<size_t>> cornerClusters;
        std::unordered_map<size_t, glm::dvec3>                 cornerOrigPos;
        for (size_t i = 0; i < chains.size(); i++)
        {
            const ClusterBoundaryChain& chain = chains[i];
            if (chain.closedLoop || chain.points.size() < 2)
            {
                continue;
            }
            cornerOrigPos[chain.startCornerVertex] = chain.points.front();
            cornerOrigPos[chain.endCornerVertex]   = chain.points.back();
            cornerClusters[chain.startCornerVertex].insert(chain.clusterA);
            cornerClusters[chain.endCornerVertex].insert(chain.clusterA);
            if (chain.clusterB != SIZE_MAX)
            {
                cornerClusters[chain.startCornerVertex].insert(chain.clusterB);
                cornerClusters[chain.endCornerVertex].insert(chain.clusterB);
            }
        }

        std::unordered_map<size_t, glm::dvec3> refinedCorners;
        for (const auto& kv : cornerOrigPos)
        {
            size_t                     vIdx = kv.first;
            const std::unordered_set<size_t>& cSet = cornerClusters[vIdx];
            std::vector<size_t>        inc(cSet.begin(), cSet.end());
            refinedCorners[vIdx] = refinePointOnClusters(kv.second, inc, clusterKind, planarFits, cylinderFits, coneFits, sphereFits, maxSnapDistance);
        }

        for (size_t chainIdx = 0; chainIdx < chains.size(); chainIdx++)
        {
            const ClusterBoundaryChain& chain = chains[chainIdx];
            if (chain.points.empty())
            {
                continue;
            }

            std::vector<glm::dvec3> pts = chain.points;
            std::vector<size_t>     inc;
            inc.push_back(chain.clusterA);
            if (chain.clusterB != SIZE_MAX)
            {
                inc.push_back(chain.clusterB);
            }

            if (chain.closedLoop)
            {
                for (size_t p = 0; p < pts.size(); p++)
                {
                    pts[p] = refinePointOnClusters(pts[p], inc, clusterKind, planarFits, cylinderFits, coneFits, sphereFits, maxSnapDistance);
                }
            }
            else if (pts.size() >= 2)
            {
                auto itStart = refinedCorners.find(chain.startCornerVertex);
                if (itStart != refinedCorners.end())
                {
                    pts.front() = itStart->second;
                }
                auto itEnd = refinedCorners.find(chain.endCornerVertex);
                if (itEnd != refinedCorners.end())
                {
                    pts.back() = itEnd->second;
                }
                for (size_t p = 1; p + 1 < pts.size(); p++)
                {
                    pts[p] = refinePointOnClusters(pts[p], inc, clusterKind, planarFits, cylinderFits, coneFits, sphereFits, maxSnapDistance);
                }
            }

            graph.setChainPoints(chainIdx, pts);
        }
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"
#include "CADacomb/Triangle/Clustering.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("BoundaryChainRefiner/RefinesPlanePlaneChains")
{
    CADacomb::Triangulation          boxTri   = CADacomb::BaseShapeGenerator::cube();
    std::vector<std::vector<size_t>> clusters = CADacomb::Clustering::clusterByNormal(boxTri, 0.99);

    std::vector<int>                              clusterKind(clusters.size(), 0);
    std::vector<CADacomb::CAD::PlanarSurfaceFit>  planarFits;
    std::vector<CADacomb::CAD::CylinderSurfaceFit> cylinderFits(clusters.size());
    std::vector<CADacomb::CAD::ConeSurfaceFit>     coneFits(clusters.size());
    std::vector<CADacomb::CAD::SphereSurfaceFit>   sphereFits(clusters.size());
    for (size_t i = 0; i < clusters.size(); i++)
    {
        planarFits.push_back(CADacomb::CAD::PlanarSurfaceFit::fit(boxTri, clusters[i]));
    }

    CADacomb::CAD::ClusterBoundaryGraph graph = CADacomb::CAD::ClusterBoundaryGraph::build(boxTri, clusters);
    CADacomb::CAD::BoundaryChainRefiner::refine(graph, clusterKind, planarFits, cylinderFits, coneFits, sphereFits, 1e-2);

    for (const CADacomb::CAD::ClusterBoundaryChain& chain : graph.getChains())
    {
        for (const glm::dvec3& pt : chain.points)
        {
            double distA = std::abs(glm::dot(pt - planarFits[chain.clusterA].getOrigin(), planarFits[chain.clusterA].getNormal()));
            REQUIRE(distA == Catch::Approx(0.0).margin(1e-9));
        }
    }
}
#endif
