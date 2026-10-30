#include "Clustering.h"

#include "CADacomb/Triangle/HalfEdge/HalfEdge.h"
#include "CADacomb/Triangle/HalfEdge/mesh2halfedge.h"
#include "CADacomb/Triangle/Triangulation.h"
#include <algorithm>
#include <cassert>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace CADacomb
{
    Clustering::Clustering() {}

    Clustering::~Clustering() {}

    std::vector<std::vector<size_t>> Clustering::cluster(const Triangulation& tri, std::function<bool(size_t currentIndex, size_t candidateIndex, const Triangulation&)> growFunction)
    {
        size_t                                                   numTriangles = tri.indices.size() / 3;
        std::vector<std::vector<size_t>>                         result;
        std::map<std::pair<size_t, size_t>, std::vector<size_t>> adjacency = tri.getAdjacency();
        std::vector<bool>                                        visited(numTriangles, false);
        for (size_t i = 0; i < numTriangles; i++)
        {
            if (visited[i])
                continue;

            std::vector<size_t> currentPatch;
            std::queue<size_t>  processQueue;

            processQueue.push(i);
            visited[i] = true;

            while (!processQueue.empty())
            {
                size_t currIdx = processQueue.front();
                processQueue.pop();
                currentPatch.push_back(currIdx);

                size_t v[3] = { tri.indices[currIdx * 3 + 0], tri.indices[currIdx * 3 + 1], tri.indices[currIdx * 3 + 2] };

                std::pair<size_t, size_t> edges[3] = {
                    { v[0], v[1] },
                    { v[1], v[2] },
                    { v[2], v[0] }
                };

                for (auto& edge : edges)
                {
                    auto key = (edge.first < edge.second) ? std::make_pair(edge.first, edge.second) : std::make_pair(edge.second, edge.first);

                    if (adjacency.count(key))
                    {
                        for (size_t neighborIdx : adjacency[key])
                        {
                            if (!visited[neighborIdx])
                            {
                                if (growFunction(currIdx, neighborIdx, tri))
                                {
                                    visited[neighborIdx] = true;
                                    processQueue.push(neighborIdx);
                                }
                            }
                        }
                    }
                }
            }
            result.push_back(currentPatch);
        }

        return result;
    }

    std::vector<std::vector<size_t>> Clustering::findBorders(const HalfEdgeMesh& mesh, const std::vector<size_t>& trianglesInCluster)
    {
        std::unordered_set<int64_t> clusterFaces(trianglesInCluster.begin(), trianglesInCluster.end());

        std::unordered_map<size_t, int64_t> vertexToEdge;
        std::vector<int64_t>                boundaryEdges;

        for (size_t fIdx : trianglesInCluster)
        {
            int64_t startEdge = mesh.faces[fIdx].half_edge;
            int64_t curr      = startEdge;
            do
            {
                int64_t twinEdge = mesh.half_edges[curr].twin;
                int64_t twinFace = (twinEdge != SafeNull) ? mesh.half_edges[twinEdge].face : SafeNull;

                if (clusterFaces.find(twinFace) == clusterFaces.end())
                {
                    size_t src        = static_cast<size_t>(mesh.source(curr));
                    vertexToEdge[src] = curr;
                    boundaryEdges.push_back(curr);
                }
                curr = mesh.half_edges[curr].next;
            } while (curr != startEdge);
        }

        std::vector<std::vector<size_t>> allLoops;
        std::unordered_set<int64_t>      visited;

        for (int64_t edge : boundaryEdges)
        {
            if (visited.count(edge))
                continue;

            std::vector<size_t> currentLoop;
            int64_t             curr = edge;

            while (visited.find(curr) == visited.end())
            {
                visited.insert(curr);

                int64_t srcVtx = mesh.source(curr);
                int64_t dstVtx = mesh.source(mesh.half_edges[curr].next);
                currentLoop.push_back(srcVtx);

                auto it = vertexToEdge.find(dstVtx);
                if (it == vertexToEdge.end())
                    break;

                curr = it->second;
                if (curr == edge)
                    break;
            }

            if (!currentLoop.empty())
                allLoops.push_back(currentLoop);
        }

        return allLoops;
    }

    std::vector<std::vector<size_t>> Clustering::splitCluster(const Triangulation& tri, const std::vector<size_t>& triangleCluster, const std::vector<size_t>& loopToBreak)
    {
        if (loopToBreak.empty())
            return { triangleCluster };

        std::unordered_set<size_t> clusterSet(triangleCluster.begin(), triangleCluster.end());
        std::set<size_t>           startSeedCandidates;
        std::unordered_set<size_t> loopVertices(loopToBreak.begin(), loopToBreak.end());

        for (size_t face : triangleCluster)
        {
            for (int i = 0; i < 3; i++)
            {
                size_t id = tri.indices[face * 3 + i];
                if (loopVertices.contains(id))
                    startSeedCandidates.insert(face);
            }
        }

        if (startSeedCandidates.size() < 2)
            return { triangleCluster };

        auto                it     = startSeedCandidates.begin();
        size_t              first  = *it;
        size_t              second = *(++it);
        std::vector<size_t> seeds  = { first, second };

        std::map<std::pair<size_t, size_t>, std::vector<size_t>> adjacency = tri.getAdjacency();

        std::unordered_map<size_t, size_t> ownership;
        std::queue<size_t>                 q;

        for (size_t s : seeds)
        {
            ownership[s] = s;
            q.push(s);
        }

        while (!q.empty())
        {
            size_t currentTri = q.front();
            q.pop();
            size_t currentOwner = ownership[currentTri];

            size_t baseIdx     = currentTri * 3;
            size_t triVerts[3] = { tri.indices[baseIdx], tri.indices[baseIdx + 1], tri.indices[baseIdx + 2] };

            for (int i = 0; i < 3; i++)
            {
                size_t v1 = triVerts[i];
                size_t v2 = triVerts[(i + 1) % 3];
                if (v1 > v2)
                    std::swap(v1, v2);

                const auto& neighbors = adjacency.at({ v1, v2 });
                for (size_t neighborTri : neighbors)
                {
                    if (neighborTri != currentTri && clusterSet.count(neighborTri) && ownership.find(neighborTri) == ownership.end())
                    {
                        ownership[neighborTri] = currentOwner;
                        q.push(neighborTri);
                    }
                }
            }
        }

        std::vector<size_t> clusterA, clusterB;
        for (auto const& [triIdx, owner] : ownership)
        {
            if (owner == seeds[0])
                clusterA.push_back(triIdx);
            else
                clusterB.push_back(triIdx);
        }

        std::set<size_t> clusterASet(clusterA.begin(), clusterA.end());
        for (const auto& x : clusterB)
            assert(!clusterASet.contains(x));

        return { clusterA, clusterB };
    }

    std::vector<std::vector<size_t>> Clustering::clusterWithoutHoles(const Triangulation& tri, std::function<bool(size_t currentIndex, size_t candidateIndex, const Triangulation&)> growFunction)
    {
        std::vector<std::vector<size_t>> clusters = cluster(tri, growFunction);
        std::vector<std::vector<size_t>> result;
        auto                             halfedge = mesh2halfedge::convert(tri);
        std::queue<std::vector<size_t>>  todo;
        for (const auto& x : clusters)
            todo.push(x);

        while (!todo.empty())
        {
            auto currentCluster = todo.front();
            todo.pop();

            std::vector<std::vector<size_t>> borders = findBorders(*halfedge, currentCluster);

            if (borders.size() <= 1)
                result.push_back(currentCluster);
            else
            {
                std::sort(borders.begin(), borders.end(), [](const std::vector<size_t>& a, const std::vector<size_t>& b) { return a.size() < b.size(); });
                auto newClusters = splitCluster(tri, currentCluster, borders[0]);
                for (const auto& x : newClusters)
                    todo.push(x);
            }
        }

        return result;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/HalfEdge/HalfEdgeHealth.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Clustering/FindBordersHoledSurfaceLoopCount")
{
    CADacomb::Triangulation tri;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
            tri.vertices.push_back(glm::dvec3(x, y, 0));

    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 3; x++)
        {
            if (x == 1 && y == 1)
                continue;

            size_t a = y * 4 + x;
            size_t b = y * 4 + x + 1;
            size_t c = (y + 1) * 4 + x + 1;
            size_t d = (y + 1) * 4 + x;
            tri.indices.insert(tri.indices.end(), { a, b, c, a, c, d });
        }
    }

    auto mesh = CADacomb::mesh2halfedge::convert(tri);

    std::vector<size_t> cluster;
    for (size_t i = 0; i < tri.indices.size() / 3; i++)
        cluster.push_back(i);

    std::vector<std::vector<size_t>> borders = CADacomb::Clustering::findBorders(*mesh, cluster);

    REQUIRE(borders.size() == 2);
}

TEST_CASE("Clustering/FindBordersHoledSurfaceVertexCounts")
{
    CADacomb::Triangulation tri;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
            tri.vertices.push_back(glm::dvec3(x, y, 0));

    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 3; x++)
        {
            if (x == 1 && y == 1)
                continue;

            size_t a = y * 4 + x;
            size_t b = y * 4 + x + 1;
            size_t c = (y + 1) * 4 + x + 1;
            size_t d = (y + 1) * 4 + x;
            tri.indices.insert(tri.indices.end(), { a, b, c, a, c, d });
        }
    }

    auto mesh = CADacomb::mesh2halfedge::convert(tri);

    std::vector<size_t> cluster;
    for (size_t i = 0; i < tri.indices.size() / 3; i++)
        cluster.push_back(i);

    std::vector<std::vector<size_t>> borders = CADacomb::Clustering::findBorders(*mesh, cluster);

    REQUIRE(borders.size() == 2);
    std::sort(borders.begin(), borders.end(), [](const auto& a, const auto& b) { return a.size() > b.size(); });

    CHECK(borders[0].size() == 12);
    CHECK(borders[1].size() == 4);
}

TEST_CASE("Clustering/ClusterWithoutHolesCoversCubeVertex")
{
    CADacomb::Triangulation mesh;
    mesh.vertices = {
        { 0, 0, 0 },
        { 1, 0, 0 },
        { 1, 1, 0 },
        { 0, 1, 0 },
        { 0, 0, 1 },
        { 1, 0, 1 },
        { 1, 1, 1 },
        { 0, 1, 1 }
    };
    mesh.indices = { 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7 };

    REQUIRE(CADacomb::HalfEdgeHealth::isHealthy(mesh));

    double threshold    = 0.9;
    auto   growFunction = [threshold](size_t currentIndex, size_t candidateIndex, const CADacomb::Triangulation& tri)
    { return glm::dot(tri.getFaceNormal(currentIndex), tri.getFaceNormal(candidateIndex)) > threshold; };

    std::vector<std::vector<size_t>> clusters = CADacomb::Clustering::clusterWithoutHoles(mesh, growFunction);

    bool vertex4IsAccountedFor = false;
    for (const auto& cluster : clusters)
    {
        for (size_t triIdx : cluster)
        {
            if (mesh.indices[triIdx * 3 + 0] == 4 || mesh.indices[triIdx * 3 + 1] == 4 || mesh.indices[triIdx * 3 + 2] == 4)
                vertex4IsAccountedFor = true;
        }
    }

    REQUIRE(vertex4IsAccountedFor);
}
#endif
