#pragma once

#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Volume/Flat/BinaryVolume.h"
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace Library
{
    class SurfaceNets
    {
      public:
        SurfaceNets();
        virtual ~SurfaceNets();

        // Returns unindexed triangle soup (every 3 consecutive vertices is a triangle)
        static std::vector<glm::dvec3> polygonize(const std::vector<bool>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::vector<glm::dvec3> polygonize(const bool* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::vector<glm::dvec3> polygonize(const std::vector<double>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue = 0.5);
        static std::vector<glm::dvec3> polygonize(const double* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue = 0.5);

        // Returns indexed Triangulation (vertices shared across quads/triangles)
        static std::unique_ptr<Triangulation> triangulate(const BinaryVolume& volume);
        static std::unique_ptr<Triangulation> triangulate(const std::vector<bool>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::unique_ptr<Triangulation> triangulate(const bool* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution);
        static std::unique_ptr<Triangulation> triangulate(const std::vector<double>& data,
                                                          const glm::dvec3&          origin,
                                                          const glm::dvec3&          voxelSize,
                                                          const glm::u64vec3&        resolution,
                                                          double                     isovalue = 0.5);
        static std::unique_ptr<Triangulation> triangulate(const double* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue = 0.5);
    };
}
