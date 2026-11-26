#include "ClusterBoundaryGraph.h"

#include "CADacomb/Triangle/Triangulation.h"

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace CADacomb::CAD
{
    ClusterBoundaryGraph::ClusterBoundaryGraph() {}

    ClusterBoundaryGraph::~ClusterBoundaryGraph() {}

    ClusterBoundaryGraph ClusterBoundaryGraph::build(const Triangulation& mesh, const std::vector<std::vector<size_t>>& clusters)
    {
        ClusterBoundaryGraph graph;
        graph.clusterLoops.resize(clusters.size());

        const size_t triangleCount = mesh.indices.size() / 3;
        if (triangleCount == 0 || clusters.empty())
        {
            return graph;
        }

        std::vector<size_t> triangleCluster(triangleCount, SIZE_MAX);
        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            const std::vector<size_t>& tris = clusters[clusterIndex];
            for (size_t i = 0; i < tris.size(); i++)
            {
                if (tris[i] < triangleCount)
                {
                    triangleCluster[tris[i]] = clusterIndex;
                }
            }
        }

        auto packEdge = [](size_t v0, size_t v1) -> uint64_t {
            return (static_cast<uint64_t>(v0) << 32) | static_cast<uint64_t>(v1);
        };

        std::unordered_map<uint64_t, size_t> edgeToCluster;
        edgeToCluster.reserve(triangleCount * 3);

        std::vector<std::unordered_set<size_t>> clustersAtVertex(mesh.vertices.size());
        for (size_t triIndex = 0; triIndex < triangleCount; triIndex++)
        {
            size_t c = triangleCluster[triIndex];
            if (c == SIZE_MAX)
            {
                continue;
            }
            size_t i0                      = mesh.indices[triIndex * 3 + 0];
            size_t i1                      = mesh.indices[triIndex * 3 + 1];
            size_t i2                      = mesh.indices[triIndex * 3 + 2];
            edgeToCluster[packEdge(i0, i1)] = c;
            edgeToCluster[packEdge(i1, i2)] = c;
            edgeToCluster[packEdge(i2, i0)] = c;
            clustersAtVertex[i0].insert(c);
            clustersAtVertex[i1].insert(c);
            clustersAtVertex[i2].insert(c);
        }

        struct BoundaryHalfEdge
        {
            size_t v0;
            size_t v1;
            size_t neighborCluster;
        };

        std::vector<std::vector<BoundaryHalfEdge>> clusterBoundaryEdges(clusters.size());
        std::vector<bool>                          vertexOnMeshBoundary(mesh.vertices.size(), false);

        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            const std::vector<size_t>& tris = clusters[clusterIndex];
            for (size_t i = 0; i < tris.size(); i++)
            {
                size_t triIndex = tris[i];
                if (triIndex >= triangleCount)
                {
                    continue;
                }
                size_t verts[3] = {
                    mesh.indices[triIndex * 3 + 0],
                    mesh.indices[triIndex * 3 + 1],
                    mesh.indices[triIndex * 3 + 2]
                };

                for (int e = 0; e < 3; e++)
                {
                    size_t                                               v0 = verts[e];
                    size_t                                               v1 = verts[(e + 1) % 3];
                    std::unordered_map<uint64_t, size_t>::const_iterator it = edgeToCluster.find(packEdge(v1, v0));
                    size_t                                               nb = (it != edgeToCluster.end()) ? it->second : SIZE_MAX;

                    if (nb == SIZE_MAX)
                    {
                        vertexOnMeshBoundary[v0] = true;
                        vertexOnMeshBoundary[v1] = true;
                    }

                    if (nb != clusterIndex)
                    {
                        clusterBoundaryEdges[clusterIndex].push_back({ v0, v1, nb });
                    }
                }
            }
        }

        std::vector<bool> isCornerVertex(mesh.vertices.size(), false);
        for (size_t v = 0; v < mesh.vertices.size(); v++)
        {
            if (clustersAtVertex[v].size() >= 3 || (vertexOnMeshBoundary[v] && clustersAtVertex[v].size() >= 2))
            {
                isCornerVertex[v] = true;
            }
        }

        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            const std::vector<BoundaryHalfEdge>& edges = clusterBoundaryEdges[clusterIndex];
            std::unordered_map<size_t, size_t>   inDegree;
            std::unordered_map<size_t, size_t>   outDegree;
            for (size_t i = 0; i < edges.size(); i++)
            {
                outDegree[edges[i].v0]++;
                inDegree[edges[i].v1]++;
            }
            for (size_t i = 0; i < edges.size(); i++)
            {
                if (outDegree[edges[i].v0] != 1 || inDegree[edges[i].v0] != 1)
                {
                    isCornerVertex[edges[i].v0] = true;
                }
            }
        }

        std::unordered_map<uint64_t, ClusterLoopReference> directedEdgeToChainRef;

        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++)
        {
            const std::vector<BoundaryHalfEdge>& edges = clusterBoundaryEdges[clusterIndex];
            if (edges.empty())
            {
                continue;
            }

            std::unordered_map<size_t, std::vector<size_t>> outgoing;
            for (size_t i = 0; i < edges.size(); i++)
            {
                outgoing[edges[i].v0].push_back(i);
            }

            std::vector<bool> visited(edges.size(), false);

            auto traceLoopFromEdge = [&](size_t startEdgeIndex) {
                std::vector<size_t> loopEdgeIndices;
                size_t              currentEdgeIndex = startEdgeIndex;

                while (!visited[currentEdgeIndex])
                {
                    visited[currentEdgeIndex] = true;
                    loopEdgeIndices.push_back(currentEdgeIndex);

                    size_t                                                      nextVertex = edges[currentEdgeIndex].v1;
                    std::unordered_map<size_t, std::vector<size_t>>::iterator outIt      = outgoing.find(nextVertex);
                    if (outIt == outgoing.end())
                    {
                        break;
                    }

                    size_t chosenNext = SIZE_MAX;
                    for (size_t k = 0; k < outIt->second.size(); k++)
                    {
                        size_t candidate = outIt->second[k];
                        if (!visited[candidate])
                        {
                            chosenNext = candidate;
                            break;
                        }
                    }

                    if (chosenNext == SIZE_MAX)
                    {
                        break;
                    }
                    currentEdgeIndex = chosenNext;
                }

                if (loopEdgeIndices.size() < 3)
                {
                    return;
                }

                size_t cornerOffset = SIZE_MAX;
                for (size_t i = 0; i < loopEdgeIndices.size(); i++)
                {
                    if (isCornerVertex[edges[loopEdgeIndices[i]].v0])
                    {
                        cornerOffset = i;
                        break;
                    }
                }

                std::vector<ClusterLoopReference> loopRefs;

                if (cornerOffset == SIZE_MAX)
                {
                    // Closed boundary loop with no corners (e.g., hole or 2-cluster seam).
                    uint64_t firstEdgeKey = packEdge(edges[loopEdgeIndices[0]].v0, edges[loopEdgeIndices[0]].v1);
                    std::unordered_map<uint64_t, ClusterLoopReference>::const_iterator existingIt = directedEdgeToChainRef.find(firstEdgeKey);
                    if (existingIt != directedEdgeToChainRef.end())
                    {
                        loopRefs.push_back(existingIt->second);
                    }
                    else
                    {
                        ClusterBoundaryChain chain;
                        chain.startCornerVertex = edges[loopEdgeIndices[0]].v0;
                        chain.endCornerVertex   = edges[loopEdgeIndices[0]].v0;
                        chain.clusterA          = clusterIndex;
                        chain.clusterB          = edges[loopEdgeIndices[0]].neighborCluster;
                        chain.closedLoop        = true;

                        for (size_t i = 0; i < loopEdgeIndices.size(); i++)
                        {
                            size_t v = edges[loopEdgeIndices[i]].v0;
                            chain.vertexIndices.push_back(v);
                            chain.points.push_back(mesh.vertices[v]);
                        }

                        size_t newChainIndex = graph.chains.size();
                        graph.chains.push_back(chain);

                        for (size_t i = 0; i < loopEdgeIndices.size(); i++)
                        {
                            const BoundaryHalfEdge& he = edges[loopEdgeIndices[i]];
                            directedEdgeToChainRef[packEdge(he.v0, he.v1)] = ClusterLoopReference(newChainIndex, false);
                            directedEdgeToChainRef[packEdge(he.v1, he.v0)] = ClusterLoopReference(newChainIndex, true);
                        }

                        loopRefs.push_back(ClusterLoopReference(newChainIndex, false));
                    }
                }
                else
                {
                    // Rotate loop so it starts at a corner vertex, then split at every corner / neighbor change.
                    std::vector<size_t> rotatedEdges;
                    rotatedEdges.reserve(loopEdgeIndices.size());
                    for (size_t i = 0; i < loopEdgeIndices.size(); i++)
                    {
                        rotatedEdges.push_back(loopEdgeIndices[(cornerOffset + i) % loopEdgeIndices.size()]);
                    }

                    size_t pos = 0;
                    while (pos < rotatedEdges.size())
                    {
                        size_t chainStartPos = pos;
                        size_t neighborC     = edges[rotatedEdges[pos]].neighborCluster;
                        pos++;
                        while (pos < rotatedEdges.size())
                        {
                            size_t vStart = edges[rotatedEdges[pos]].v0;
                            if (isCornerVertex[vStart] || edges[rotatedEdges[pos]].neighborCluster != neighborC)
                            {
                                break;
                            }
                            pos++;
                        }

                        const BoundaryHalfEdge& firstHe    = edges[rotatedEdges[chainStartPos]];
                        uint64_t                firstKey   = packEdge(firstHe.v0, firstHe.v1);
                        std::unordered_map<uint64_t, ClusterLoopReference>::const_iterator existingIt = directedEdgeToChainRef.find(firstKey);
                        if (existingIt != directedEdgeToChainRef.end())
                        {
                            loopRefs.push_back(existingIt->second);
                        }
                        else
                        {
                            ClusterBoundaryChain chain;
                            chain.startCornerVertex = firstHe.v0;
                            chain.endCornerVertex   = edges[rotatedEdges[pos - 1]].v1;
                            chain.clusterA          = clusterIndex;
                            chain.clusterB          = neighborC;
                            chain.closedLoop        = false;

                            for (size_t k = chainStartPos; k < pos; k++)
                            {
                                size_t v = edges[rotatedEdges[k]].v0;
                                chain.vertexIndices.push_back(v);
                                chain.points.push_back(mesh.vertices[v]);
                            }
                            chain.vertexIndices.push_back(chain.endCornerVertex);
                            chain.points.push_back(mesh.vertices[chain.endCornerVertex]);

                            size_t newChainIndex = graph.chains.size();
                            graph.chains.push_back(chain);

                            for (size_t k = chainStartPos; k < pos; k++)
                            {
                                const BoundaryHalfEdge& he = edges[rotatedEdges[k]];
                                directedEdgeToChainRef[packEdge(he.v0, he.v1)] = ClusterLoopReference(newChainIndex, false);
                                directedEdgeToChainRef[packEdge(he.v1, he.v0)] = ClusterLoopReference(newChainIndex, true);
                            }

                            loopRefs.push_back(ClusterLoopReference(newChainIndex, false));
                        }
                    }
                }

                if (!loopRefs.empty())
                {
                    graph.clusterLoops[clusterIndex].push_back(loopRefs);
                }
            };

            for (size_t i = 0; i < edges.size(); i++)
            {
                if (!visited[i] && isCornerVertex[edges[i].v0])
                {
                    traceLoopFromEdge(i);
                }
            }
            for (size_t i = 0; i < edges.size(); i++)
            {
                if (!visited[i])
                {
                    traceLoopFromEdge(i);
                }
            }
        }

        return graph;
    }

    void ClusterBoundaryGraph::simplifyPolylineRange(const std::vector<glm::dvec3>& points,
                                                     size_t                         startIndex,
                                                     size_t                         endIndex,
                                                     double                         toleranceSq,
                                                     std::vector<bool>&             keepMask)
    {
        if (endIndex <= startIndex + 1)
        {
            return;
        }

        const glm::dvec3& a   = points[startIndex];
        const glm::dvec3& b   = points[endIndex];
        glm::dvec3        ab  = b - a;
        double            ab2 = glm::dot(ab, ab);

        double maxDistSq = -1.0;
        size_t maxIndex  = startIndex;

        for (size_t i = startIndex + 1; i < endIndex; i++)
        {
            glm::dvec3 ap     = points[i] - a;
            double     distSq = 0.0;
            if (ab2 > 1e-20)
            {
                double t    = std::clamp(glm::dot(ap, ab) / ab2, 0.0, 1.0);
                glm::dvec3 diff = ap - t * ab;
                distSq          = glm::dot(diff, diff);
            }
            else
            {
                distSq = glm::dot(ap, ap);
            }

            if (distSq > maxDistSq)
            {
                maxDistSq = distSq;
                maxIndex  = i;
            }
        }

        if (maxDistSq > toleranceSq && maxIndex != startIndex)
        {
            keepMask[maxIndex] = true;
            simplifyPolylineRange(points, startIndex, maxIndex, toleranceSq, keepMask);
            simplifyPolylineRange(points, maxIndex, endIndex, toleranceSq, keepMask);
        }
    }

    void ClusterBoundaryGraph::simplifyChain(ClusterBoundaryChain& chain, double toleranceSq)
    {
        if (chain.closedLoop)
        {
            if (chain.points.size() <= 4)
            {
                return;
            }
            size_t            n        = chain.points.size();
            size_t            mid      = n / 2;
            std::vector<bool> keepMask(n, false);
            keepMask[0]   = true;
            keepMask[mid] = true;
            simplifyPolylineRange(chain.points, 0, mid, toleranceSq, keepMask);

            std::vector<glm::dvec3> secondHalf;
            secondHalf.reserve(n - mid + 1);
            for (size_t i = mid; i < n; i++)
            {
                secondHalf.push_back(chain.points[i]);
            }
            secondHalf.push_back(chain.points[0]);
            std::vector<bool> secondMask(secondHalf.size(), false);
            secondMask[0]                     = true;
            secondMask[secondHalf.size() - 1] = true;
            simplifyPolylineRange(secondHalf, 0, secondHalf.size() - 1, toleranceSq, secondMask);

            std::vector<glm::dvec3> simplified;
            for (size_t i = 0; i <= mid; i++)
            {
                if (keepMask[i])
                {
                    simplified.push_back(chain.points[i]);
                }
            }
            for (size_t i = 1; i + 1 < secondHalf.size(); i++)
            {
                if (secondMask[i])
                {
                    simplified.push_back(secondHalf[i]);
                }
            }
            if (simplified.size() >= 3)
            {
                chain.points = simplified;
            }
        }
        else
        {
            if (chain.points.size() <= 2)
            {
                return;
            }
            std::vector<bool> keepMask(chain.points.size(), false);
            keepMask[0]                       = true;
            keepMask[chain.points.size() - 1] = true;
            simplifyPolylineRange(chain.points, 0, chain.points.size() - 1, toleranceSq, keepMask);

            std::vector<glm::dvec3> simplified;
            for (size_t i = 0; i < chain.points.size(); i++)
            {
                if (keepMask[i])
                {
                    simplified.push_back(chain.points[i]);
                }
            }
            chain.points = simplified;
        }
    }

    void ClusterBoundaryGraph::simplifyChains(double chordTolerance)
    {
        if (chordTolerance <= 0.0)
        {
            return;
        }

        double toleranceSq = chordTolerance * chordTolerance;
        for (size_t chainIndex = 0; chainIndex < chains.size(); chainIndex++)
        {
            simplifyChain(chains[chainIndex], toleranceSq);
        }
    }

    void ClusterBoundaryGraph::simplifyPlanarChains(double chordTolerance, const std::vector<bool>& clusterIsPlanar)
    {
        if (chordTolerance <= 0.0)
        {
            return;
        }

        double toleranceSq = chordTolerance * chordTolerance;
        for (size_t chainIndex = 0; chainIndex < chains.size(); chainIndex++)
        {
            ClusterBoundaryChain& chain    = chains[chainIndex];
            bool                  aPlanar  = chain.clusterA < clusterIsPlanar.size() && clusterIsPlanar[chain.clusterA];
            bool                  bPlanar  = chain.clusterB == SIZE_MAX || (chain.clusterB < clusterIsPlanar.size() && clusterIsPlanar[chain.clusterB]);
            if (aPlanar && bPlanar)
            {
                simplifyChain(chain, toleranceSq);
            }
        }
    }

    void ClusterBoundaryGraph::setChainPoints(size_t chainIndex, const std::vector<glm::dvec3>& points)
    {
        chains[chainIndex].points = points;
    }

    const std::vector<ClusterBoundaryChain>& ClusterBoundaryGraph::getChains() const
    {
        return chains;
    }

    const std::vector<std::vector<ClusterLoopReference>>& ClusterBoundaryGraph::getClusterLoops(size_t clusterIndex) const
    {
        return clusterLoops[clusterIndex];
    }

    size_t ClusterBoundaryGraph::getClusterCount() const
    {
        return clusterLoops.size();
    }

    std::vector<std::vector<glm::dvec3>> ClusterBoundaryGraph::buildClusterPointLoops(size_t clusterIndex) const
    {
        std::vector<std::vector<glm::dvec3>> result;
        const std::vector<std::vector<ClusterLoopReference>>& loops = clusterLoops[clusterIndex];

        for (size_t loopIndex = 0; loopIndex < loops.size(); loopIndex++)
        {
            const std::vector<ClusterLoopReference>& loopRefs = loops[loopIndex];
            std::vector<glm::dvec3>                  loopPoints;

            for (size_t refIndex = 0; refIndex < loopRefs.size(); refIndex++)
            {
                const ClusterLoopReference& ref   = loopRefs[refIndex];
                const ClusterBoundaryChain& chain = chains[ref.chainIndex];

                if (chain.closedLoop)
                {
                    if (!ref.reversed)
                    {
                        loopPoints.insert(loopPoints.end(), chain.points.begin(), chain.points.end());
                    }
                    else
                    {
                        for (size_t p = chain.points.size(); p > 0; p--)
                        {
                            loopPoints.push_back(chain.points[p - 1]);
                        }
                    }
                }
                else
                {
                    if (!ref.reversed)
                    {
                        for (size_t p = 0; p + 1 < chain.points.size(); p++)
                        {
                            loopPoints.push_back(chain.points[p]);
                        }
                    }
                    else
                    {
                        for (size_t p = chain.points.size() - 1; p > 0; p--)
                        {
                            loopPoints.push_back(chain.points[p]);
                        }
                    }
                }
            }

            if (loopPoints.size() >= 3)
            {
                result.push_back(loopPoints);
            }
        }

        return result;
    }
}
