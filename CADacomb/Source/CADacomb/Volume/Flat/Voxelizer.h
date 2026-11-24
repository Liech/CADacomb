#pragma once

#include "CADacomb/Volume/Flat/BinaryVolume.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb
{
    class Voxelizer
    {
      public:
        Voxelizer();
        virtual ~Voxelizer();

        std::unique_ptr<BinaryVolume> voxelize(const std::vector<glm::dvec3>& vertices,
                                               const std::vector<size_t>&     indices,
                                               const glm::dvec3&              start,
                                               const glm::dvec3&              end,
                                               const glm::u64vec3&            resolution);
        std::unique_ptr<BinaryVolume> voxelize(const std::vector<glm::dvec3>& vertices, const glm::dvec3& start, const glm::dvec3& end, const glm::u64vec3& resolution);

      private:
        void spatialHashTriangles(const std::vector<glm::dvec3>&    vertices,
                                  const std::vector<size_t>&        indices,
                                  const glm::dvec3&                 start,
                                  const glm::dvec3&                 end,
                                  const glm::u64vec3&               resolution,
                                  std::vector<std::vector<size_t>>& triangleStack);
        void applyTrianglesToVolume(std::vector<bool>&                      volume,
                                    const std::vector<std::vector<size_t>>& triangleStack,
                                    const std::vector<glm::dvec3>&          vertices,
                                    const std::vector<size_t>&              indices,
                                    const glm::dvec3&                       start,
                                    const glm::dvec3&                       end,
                                    const glm::u64vec3&                     resolution);

        void box(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, glm::dvec2& start, glm::dvec2& end);
        int  edgeSign(const glm::dvec3& a, const glm::dvec3& b, double px, double py, double& outCross) const;
        bool intersectVerticalRayTriangle(double rayX, double rayY, const glm::dvec3& vertex0, const glm::dvec3& vertex1, const glm::dvec3& vertex2, double& outZ) const;
    };
}
