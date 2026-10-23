#pragma once

#include <array>
#include <glm/glm.hpp>
#include <vector>

namespace Library
{
    class MarchingCubes
    {
      public:
        MarchingCubes();
        virtual ~MarchingCubes();

        static std::vector<glm::dvec3> polygonize(const std::vector<bool>&, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::vector<glm::dvec3> polygonize(const bool*, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::vector<glm::dvec3> polygonize(const std::vector<double>&, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue);
        static std::vector<glm::dvec3> polygonize(const std::array<glm::dvec3, 8>& p, const std::array<double, 8>& val, double isolevel);

        static std::vector<double> pack(const std::vector<bool>&, const glm::u64vec3& resolution, const glm::u64vec3& oneVoxelSize);
        static std::vector<double> pack(const bool*, const glm::u64vec3& resolution, const glm::u64vec3& oneVoxelSize);

      private:
        static glm::dvec3 VertexInterp(double isolevel, const glm::dvec3& p1, const glm::dvec3& p2, double valp1, double valp2);

        static const int edgeTable[256];
        static const int triTable[256][16];
    };
}
