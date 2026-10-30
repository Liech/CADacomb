#pragma once

#include <functional>
#include <vector>

namespace CADacomb
{
    class Triangulation;
    class HalfEdgeMesh;

    class Clustering
    {
      public:
        Clustering();
        virtual ~Clustering();

        static std::vector<std::vector<size_t>> cluster(const Triangulation&, std::function<bool(size_t currentIndex, size_t candidateIndex, const Triangulation&)> growFunction);
        static std::vector<std::vector<size_t>> clusterWithoutHoles(const Triangulation&, std::function<bool(size_t currentIndex, size_t candidateIndex, const Triangulation&)> growFunction);

        static std::vector<std::vector<size_t>> findBorders(const HalfEdgeMesh&, const std::vector<size_t>& trianglesInCluster);
        static std::vector<std::vector<size_t>> splitCluster(const Triangulation&, const std::vector<size_t>& triangleCluster, const std::vector<size_t>& loopToBreak);
    };
}
