#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "CADacomb/Triangle/Triangulation.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
#include "CADacomb/Volume/SVO/SVO.h"

namespace Library::Volume
{
    class SVOVoxelizer
    {
      public:
        SVOVoxelizer();
        virtual ~SVOVoxelizer();

        template<typename T = bool>
        std::unique_ptr<SVO<T>> voxelize(const Triangulation&    triangulation,
                                         const glm::dvec3&       start,
                                         const glm::dvec3&       end,
                                         uint32_t                dimension,
                                         T                       solidValue = static_cast<T>(true),
                                         T                       emptyValue = static_cast<T>(false),
                                         const MergeStrategy<T>& strategy   = SVO<T>::defaultMergeStrategy());

      private:
        void voxelizeBlocks(const Triangulation&                                                                                                   triangulation,
                            const glm::dvec3&                                                                                                      start,
                            const glm::dvec3&                                                                                                      end,
                            uint32_t                                                                                                               dimension,
                            uint32_t                                                                                                               blockSize,
                            const std::function<void(const glm::ivec3& blockOffset, bool isSolidHomogeneous, const std::vector<bool>& blockData)>& onBlock) const;

        void spatialHashTrianglesToTiles(const std::vector<glm::dvec3>&    vertices,
                                         const std::vector<size_t>&        indices,
                                         const glm::dvec3&                 start,
                                         const glm::dvec3&                 end,
                                         uint32_t                          numTilesX,
                                         uint32_t                          numTilesY,
                                         std::vector<std::vector<size_t>>& tileTriangleStack) const;

        void box(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, glm::dvec2& start, glm::dvec2& end) const;
        bool intersectRayTriangle(const glm::dvec3& rayOrigin, const glm::dvec3& rayVector, const glm::dvec3& vertex0, const glm::dvec3& vertex1, const glm::dvec3& vertex2, glm::dvec3& out) const;
        bool epsilonEqual(double a, double b) const;
    };

    template<typename T>
    std::unique_ptr<SVO<T>> SVOVoxelizer::voxelize(const Triangulation&    triangulation,
                                                   const glm::dvec3&       start,
                                                   const glm::dvec3&       end,
                                                   uint32_t                dimension,
                                                   T                       solidValue,
                                                   T                       emptyValue,
                                                   const MergeStrategy<T>& strategy)
    {
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);
        auto svo = std::make_unique<SVO<T>>(dimension, emptyValue, strategy);

        if (triangulation.vertices.empty() || triangulation.indices.empty() || dimension == 0)
        {
            return svo;
        }

        uint32_t blockSize = std::min(32u, dimension);

        voxelizeBlocks(triangulation,
                       start,
                       end,
                       dimension,
                       blockSize,
                       [&](const glm::ivec3& blockOffset, bool isSolidHomogeneous, const std::vector<bool>& blockData)
                       {
                           if (isSolidHomogeneous)
                           {
                               SVO<T> solidChunk(blockSize, solidValue, svo->getMergeStrategy());
                               svo->setSubSVO(blockOffset, solidChunk);
                           }
                           else
                           {
                               SVO<T> mixedChunk(blockSize, emptyValue, svo->getMergeStrategy());
                               size_t sliceSize = blockSize * blockSize;

                               for (size_t z = 0; z < blockSize; z++)
                               {
                                   for (size_t y = 0; y < blockSize; y++)
                                   {
                                       for (size_t x = 0; x < blockSize; x++)
                                       {
                                           size_t idx = x + y * blockSize + z * sliceSize;
                                           if (blockData[idx])
                                           {
                                               mixedChunk.setVoxel(glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)), solidValue);
                                           }
                                       }
                                   }
                               }

                               svo->setSubSVO(blockOffset, mixedChunk);
                           }
                       });

        return svo;
    }
}
