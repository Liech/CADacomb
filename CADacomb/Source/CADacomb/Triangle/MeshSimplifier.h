#pragma once

#include "CADacomb/Triangle/Triangulation.h"

#include <cstddef>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb
{
    // Simplifies a triangulation mesh using Quadric Error Metrics (QEM) edge collapse.
    class MeshSimplifier
    {
      public:
        MeshSimplifier();
        virtual ~MeshSimplifier();

        static std::unique_ptr<Triangulation> simplifyByTriangleCount(const Triangulation& input, size_t targetTriangleCount);
        static std::unique_ptr<Triangulation> simplifyByPercent(const Triangulation& input, double targetPercent);

      private:
        static glm::dmat4 computePlaneQuadric(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2);
        static double     evaluateQuadric(const glm::dmat4& q, const glm::dvec3& p);
        static bool       computeOptimalPosition(const glm::dmat4& q, const glm::dvec3& pA, const glm::dvec3& pB, glm::dvec3& outPos, double& outError);
        static void       weldVertices(const Triangulation& input, std::vector<glm::dvec3>& outVertices, std::vector<size_t>& outIndices);
    };
}
