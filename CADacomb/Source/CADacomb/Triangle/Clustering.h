#pragma once

#include <functional>
#include <map>
#include <utility>
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
        static std::vector<std::vector<size_t>> clusterByNormal(const Triangulation&, double normalDotThreshold, bool withoutHoles = false);

        static std::vector<std::vector<size_t>> findBorders(const HalfEdgeMesh&, const std::vector<size_t>& trianglesInCluster);
        static std::vector<std::vector<size_t>> splitCluster(const Triangulation&, const std::vector<size_t>& triangleCluster, const std::vector<size_t>& loopToBreak);

      private:
        static std::vector<std::vector<size_t>> cluster(const Triangulation&,
                                                        const std::map<std::pair<size_t, size_t>, std::vector<size_t>>& adjacency,
                                                        std::function<bool(size_t currentIndex, size_t candidateIndex, const Triangulation&)> growFunction);
        static std::vector<std::vector<size_t>> splitCluster(const Triangulation&,
                                                             const std::map<std::pair<size_t, size_t>, std::vector<size_t>>& adjacency,
                                                             const std::vector<size_t>&                                      triangleCluster,
                                                             const std::vector<size_t>&                                      loopToBreak);
    };
}
